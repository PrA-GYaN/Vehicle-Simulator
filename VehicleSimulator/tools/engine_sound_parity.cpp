//===========================================================================
//  engine_sound_parity  --  check audio/EngineSimSynth against EngineSim's
//  own sound worklet.
//
//      python tools/engine_sound_reference.py 454 --out /tmp/s
//      engine_sound_parity /tmp/s.yaml /tmp/s.states.csv /tmp/s.f32 [--tolerance X] [--wav out.wav]
//
//  The reference is EngineSim's dashboard/engine-audio.js, unmodified,
//  rendered under Node over a recorded engine run. This builds the sound
//  spec from the YAML through the same code the simulator uses
//  (makeEngineSimSoundSpec), replays the same per-block states through
//  EngineSimSynth, and compares every output sample.
//
//  The two are the same arithmetic in the same order with the same random
//  streams, so they agree to rounding. They do NOT agree bit for bit, and
//  cannot: sin, exp, pow and tanh come from V8's libm on one side and the C
//  library's on the other, which differ in the last place. Measured, the two
//  outputs differ by one or two units in the last place of a 32-bit float on
//  a few samples (worst 3e-8 of full scale). The default tolerance, 1e-6
//  (-120 dB), is far above that and far below anything a transcription error
//  produces - a missed noise() call alone decorrelates the two completely.
//
//  Offline tool. See README, "Keeping the port in step with EngineSim".
//===========================================================================

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "audio/EngineSimSynth.h"
#include "audio/SoundSynth.h"
#include "config/Config.h"

int main(int argc, char** argv) {
    vsim::setExecutablePath(argc > 0 ? argv[0] : nullptr);
    if (argc < 4) {
        std::fprintf(stderr, "usage: engine_sound_parity <engine.yaml> <states.csv> <reference.f32> "
                             "[--tolerance X] [--wav out.wav]\n");
        return 2;
    }
    double tolerance = 1e-6;
    std::string wavPath;
    for (int i = 4; i + 1 < argc; ++i) {
        if (std::strcmp(argv[i], "--tolerance") == 0) tolerance = std::atof(argv[i + 1]);
        if (std::strcmp(argv[i], "--wav") == 0) wavPath = argv[i + 1];
    }

    vsim::EngineParameters engine;
    vsim::EngineSimSoundConfig sound;
    if (!vsim::loadEngineParameters(argv[1], engine) || !vsim::loadEngineSimSound(argv[1], sound)) return 2;
    if (!engine.useResolved) {
        std::fprintf(stderr, "engine_sound_parity: '%s' has no `resolved:` block\n", argv[1]);
        return 2;
    }

    std::ifstream ref(argv[3], std::ios::binary);
    std::vector<char> raw((std::istreambuf_iterator<char>(ref)), std::istreambuf_iterator<char>());
    std::vector<float> reference(raw.size() / sizeof(float));
    std::memcpy(reference.data(), raw.data(), reference.size() * sizeof(float));

    constexpr int N = vsim::EngineSimSynth::kBlock;
    vsim::EngineSimSynth synth(48000.0);
    synth.configure(vsim::makeEngineSimSoundSpec(sound, engine));
    synth.applyProfile(sound.profile);

    std::ifstream in(argv[2]);
    std::string line;
    std::getline(in, line);     // header: rpm,map,throttle,combusting,fuelCut,starter,ignition,
                                //         indicated,friction,pumping,redline,idle
    std::vector<float> out;
    float L[N], R[N];
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        std::vector<double> v;
        std::stringstream ss(line);
        std::string cell;
        while (std::getline(ss, cell, ',')) v.push_back(std::strtod(cell.c_str(), nullptr));
        if (v.size() != 12) { std::fprintf(stderr, "engine_sound_parity: bad state row\n"); return 2; }
        vsim::EngineSimSynth::State s;
        s.rpm = v[0]; s.map = v[1]; s.throttle = v[2];
        s.combusting = v[3] != 0.0; s.fuelCut = v[4] != 0.0; s.starter = v[5] != 0.0; s.ignition = v[6] != 0.0;
        s.indicated = v[7]; s.friction = v[8]; s.pumping = v[9]; s.redline = v[10]; s.idle = v[11];
        synth.setState(s);
        synth.process(L, R, N);
        for (int i = 0; i < N; ++i) { out.push_back(L[i]); out.push_back(R[i]); }
    }
    if (!wavPath.empty()) vsim::writeWav(wavPath, out, 48000);

    if (out.size() != reference.size()) {
        std::fprintf(stderr, "engine_sound_parity: %zu samples rendered, reference has %zu\n",
                     out.size(), reference.size());
        return 2;
    }
    double worst = 0.0, sumSq = 0.0, errSq = 0.0, peak = 0.0;
    std::size_t worstAt = 0, firstBad = out.size();
    for (std::size_t i = 0; i < out.size(); ++i) {
        const double err = std::abs(double(out[i]) - double(reference[i]));
        if (err > worst) { worst = err; worstAt = i; }
        if (err > tolerance && firstBad == out.size()) firstBad = i;
        sumSq += double(reference[i]) * reference[i];
        errSq += err * err;
        peak = std::max(peak, std::abs(double(reference[i])));
    }
    const double seconds = out.size() / 2 / 48000.0;
    std::printf("engine_sound_parity: %.1f s, %zu samples, reference peak %.3f rms %.4f\n",
                seconds, out.size(), peak, std::sqrt(sumSq / out.size()));
    std::printf("  worst difference %.3g at %.4f s (EngineSim %.7g, port %.7g)\n",
                worst, worstAt / 2 / 48000.0, double(reference[worstAt]), double(out[worstAt]));
    std::printf("  difference rms %.3g  (%.1f dB below the signal)\n", std::sqrt(errSq / out.size()),
                errSq > 0.0 ? 10.0 * std::log10(sumSq / errSq) : 999.0);
    const bool ok = worst <= tolerance;
    if (!ok) std::printf("  first sample out of tolerance at %.4f s\n", firstBad / 2 / 48000.0);
    std::printf(ok ? "engine_sound_parity: PASS\n" : "engine_sound_parity: FAIL\n");
    return ok ? 0 : 1;
}
