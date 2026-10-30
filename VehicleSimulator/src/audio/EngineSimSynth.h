#pragma once
//===========================================================================
//  EngineSimSynth.h  --  EngineSim's engine sound, running natively here.
//
//  SOURCE: ../EngineSim/dashboard/engine-audio.js (the AudioWorklet
//  `EngineSynth`) and ../EngineSim/engine_sim/sound_spec.py. Documented in
//  ../EngineSim/README.md section 6. Like the resolved engine it accompanies,
//  this is NOT part of this project's S1/S2/S3 Knowledge Base, and unlike
//  audio/SoundSynth.h it does not claim to be derived: EngineSim's sound is
//  a model TUNED BY MEASUREMENT (its README 6.3 lists the rules it was tuned
//  to, with the spectra), built on physical structure -
//
//    * a crank-angle clock, with the speed ripple each firing puts into the
//      flywheel, walks a list of valve events placed by the real firing
//      order and valve timing;
//    * each exhaust-valve opening kicks a BLOWDOWN and a DISPLACEMENT pulse
//      (3-pole smoothing filters, so no pulse can click), strong when
//      firing, weak and NEGATIVE when motoring on a fuel cut;
//    * the pulses drive a Kelly-Lochbaum EXHAUST WAVEGUIDE per pipe: header,
//      mid pipe, muffler expansion chamber, tailpipe, with reflections from
//      the area ratios and an open end that loses its highs to radiation.
//      The pipe ringing between firings is what turns a pulse train into an
//      engine note;
//    * intake pulses and roar, throttle hiss, valvetrain tick, starter motor
//      and solenoid, overrun pops; early reflections, compressor, soft clip.
//
//  A TRANSCRIPTION, statement for statement, of the worklet: same event
//  order, same random streams, the same 128-frame blocks (several things are
//  evaluated per block), and `float` wherever the JavaScript stores into a
//  Float32Array - because that rounding is part of what it computes.
//  tools/engine_sound_parity checks it against the original run under Node.
//
//  What is left out: the worklet's own road noise and driveline whine. This
//  simulator has tyres, wind and gear whine of its own (audio/SoundSynth.h),
//  driven by the tyre model; the terms are transcribed but fed zero speed.
//
//  Change the sound in EngineSim first, then here - see README, "Keeping the
//  port in step with EngineSim".
//===========================================================================

#include <array>
#include <cstdint>
#include <vector>

#include "config/EngineParameters.h"
#include "config/VehicleParameters.h"

namespace vsim {

/// What engine_sim/sound_spec.py derives from an engine file - everything
/// engine-specific the synthesiser needs. Built by makeEngineSimSoundSpec().
struct EngineSimSoundSpec {
    struct Section { double lengthM = 1.0, c = 500.0, area = 1.0; };
    struct Pipe { std::array<Section, 4> sections; double bodyHz = 125.0; };

    int cylinders = 0;                    ///< 0 = no spec; the synthesiser stays silent
    std::vector<double> fireAngleDeg;     ///< firing TDC of each cylinder in the cycle
    double evoAfterFireDeg = 130.0;       ///< exhaust valve opens, after that cylinder's firing TDC
    double ivoAfterFireDeg = 350.0;       ///< intake valve opens
    std::vector<int> cylinderPipe;        ///< which exhaust pipe each cylinder feeds
    std::vector<Pipe> pipes;
    double pipeLoss = 0.992;
    double tipDamping = 0.35;
    double runnerLengthM = 0.45;
    double runnerC = 343.0;
    double blowdownTauMaxS = 3.5e-4;
    double inertiaKgM2 = 0.1;
    double rippleGain = 0.66;
    double cylinderStrengthSpread = 0.30;
    double cylinderTimingSpreadDeg = 6.0;
    double seed = 1234567.0;
    double variabilityBase = 0.02;
    double variabilityIdle = 0.05;
    double popProbability = 0.03;
    double starterRatio = 11.0;
    double starterPoles = 4.0;
};

/// sound_spec.sound_spec(): the spec for a resolved engine and its sound
/// hardware. `engine` must have `useResolved` set.
EngineSimSoundSpec makeEngineSimSoundSpec(const EngineSimSoundConfig& sound,
                                          const EngineParameters& engine);

class EngineSimSynth {
public:
    /// The worklet's `state` message: the simulation, ~60 times a second.
    struct State {
        double rpm = 0.0;
        double map = 1.0;            ///< manifold pressure / ambient
        double throttle = 0.0;
        bool   combusting = false;
        bool   fuelCut = false;
        bool   starter = false;
        bool   ignition = false;
        double speed = 0.0;          ///< [mph] - 0 here, see the file banner
        int    gear = 0;
        double load = 0.0;
        bool   pops = true;
        bool   muted = false;
        double redline = 6000.0;
        double idle = 800.0;
        double indicated = 0.0;      ///< torques [N.m]
        double friction = 0.0;
        double pumping = 0.0;
    };
    using Profile = EngineSimSoundConfig::Profile;

    static constexpr int kBlock = 128;   ///< the AudioWorklet render quantum

    explicit EngineSimSynth(double sampleRate = 48000.0);

    void configure(const EngineSimSoundSpec& spec);   ///< the `engine` message
    void setState(const State& s) { m_s = s; }        ///< the `state` message
    void applyProfile(const Profile& p);              ///< the `profile` message
    const Profile& profile() const { return m_pf; }
    bool configured() const { return m_configured; }

    /// AudioWorkletProcessor.process(): one block of `n` frames (the worklet
    /// is always called with 128; several quantities are per block).
    void process(float* left, float* right, int n);

    double sampleRate() const { return m_sr; }

private:
    // --- the worklet's helper classes ---------------------------------------
    struct Biquad {
        double b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0, z1 = 0, z2 = 0;
        enum Type { LP, HP, BP, LS, HS, Peak };
        void set(Type type, double fc, double q, double sr, double gainDb = 0.0);
        double process(double x) {      // transposed direct form II
            const double y = b0 * x + z1;
            z1 = b1 * x - a1 * y + z2;
            z2 = b2 * x - a2 * y;
            return y;
        }
    };
    /// Tube with a lossy reflecting end: y[n] = x[n] + g LP(y[n-D]).
    struct Pipe {
        std::vector<float> buf;
        std::size_t i = 0;
        double g = 0.0, a = 0.0, lp = 0.0;
        void init(double sr, double lengthM, double soundSpeed, double reflection, double damping);
        double process(double x);
    };
    /// Three cascaded one-poles: an impulse in gives t^2 e^(-t/tau) out.
    struct Shaper {
        double a = 0.1, y1 = 0, y2 = 0, y3 = 0, norm = 1, in = 0;
        void setTau(double tauS, double sr);
        void kick(double amplitude) { in += amplitude * norm; }
        double process() {
            y1 += a * (in - y1); in = 0;
            y2 += a * (y1 - y2);
            y3 += a * (y2 - y3);
            return y3;
        }
    };
    struct Delay {
        std::vector<float> b;
        std::size_t i = 0;
        void init(int n);
        double out() const { return b[i]; }
        void push(double x) { b[i] = static_cast<float>(x); if (++i == b.size()) i = 0; }
    };
    /// One exhaust pipe as a chain of uniform tube sections.
    struct ExhaustPipe {
        std::vector<Delay> F, B;          ///< toward the tip / back toward the valves
        std::vector<float> K, fo, bo;     ///< junction reflections; section outputs
        double loss = 0.992, rValve = 0.7, rOpen = -0.9, aOpen = 0.35, lp = 0.0;
        void init(double sr, const EngineSimSoundSpec::Pipe& pipe, double loss, double tipDamping);
        double process(double src);
    };
    struct Reverb {
        struct Line { std::vector<float> b; std::size_t i = 0; double f = 0.0; };
        std::array<Line, 4> cL, cR;
        std::array<Line, 2> aL, aR;
        double fb = 0.80, damp = 0.35, l = 0.0, r = 0.0;
        void init(double sr);
        double side(double x, std::array<Line, 4>& combs, std::array<Line, 2>& aps);
        void process(double x) { l = side(x, cL, aL); r = side(x, cR, aR); }
    };
    struct Exhaust {
        Shaper bd, disp;
        double noise = 0.0;
        ExhaustPipe pipe;
        Biquad body;
        double prev = 0.0, radLP = 0.0;
    };

    double noise();
    double gauss() { const double a = noise(), b = noise(), c = noise(); return (a + b + c) * 0.577; }
    void exhaustEvent(int cylinder);
    void deriveProfile();

    double m_sr;
    bool m_configured = false;
    EngineSimSoundSpec m_sp;
    State m_s;
    Profile m_pf;

    double m_rpm = 0.0, m_map = 1.0, m_throttle = 0.0;
    std::uint32_t m_seed = 22222u, m_seed2 = 987654321u;

    double m_noiseLP = 0, m_noiseLP2 = 0, m_jetLP = 0, m_jetLP2 = 0, m_wander = 0, m_rumLP = 0, m_rumLP2 = 0;
    Shaper m_intakeShaper;
    double m_fuelCutT = 99.0;
    bool   m_starterPrev = false;
    double m_solenoid = 0, m_solenoidPhase = 0, m_starterPhase = 0, m_whinePhase = 0, m_tickEnv = 0;
    double m_compEnv = 0.1, m_compGain = 0.2;
    double m_tqInd = 0, m_tqFric = 0, m_tqPump = 0;

    Biquad m_dcL, m_dcR, m_intakeBP, m_intakeLP, m_hissHP, m_hissLP, m_roadLP, m_starterBP,
           m_tickBP, m_outLPL, m_outLPR, m_lsL, m_lsR, m_hsL, m_hsR;
    Reverb m_reverb;

    static constexpr int kErLen = 2048;
    std::array<float, kErLen> m_erBuf{};
    int m_erI = 0;
    std::array<int, 4> m_erD{};
    std::array<float, 4> m_erGL{{0.20f, 0.11f, 0.13f, 0.06f}};
    std::array<float, 4> m_erGR{{0.11f, 0.18f, 0.08f, 0.10f}};

    // built by configure()
    int m_n = 0, m_pipes = 0;
    std::vector<int> m_cylPipe;
    std::vector<Exhaust> m_ex;
    std::array<std::array<double, 2>, 2> m_pan{};
    Pipe m_intakeRunner;
    std::vector<float> m_cylGain;
    std::vector<double> m_evA, m_evO;     ///< event angle; per-event timing offset [deg]
    std::vector<int> m_evT, m_evC;        ///< event type (0 exhaust, 1 intake); cylinder
    int m_E = 0, m_next = 0;
    double m_theta = 0.0, m_nextBase = 0.0, m_nextA = 0.0;

    // derived from the profile
    double m_erScale = 1, m_revWet = 0, m_radW = 2.2, m_lopeS = 1, m_nlf = 0, m_nhf = 0;
    double m_tickA = 0, m_intkS = 1, m_popsS = 1, m_compExp = 0.2, m_driveK = 1;
    bool m_shelfLow = false, m_shelfHigh = false;
};

} // namespace vsim
