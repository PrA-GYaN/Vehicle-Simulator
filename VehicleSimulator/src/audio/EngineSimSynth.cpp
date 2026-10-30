#include "audio/EngineSimSynth.h"

#include <algorithm>
#include <cmath>

#include "core/Units.h"

//===========================================================================
//  A TRANSCRIPTION of ../EngineSim/dashboard/engine-audio.js. Statement
//  order, operator association and the float/double split all follow the
//  JavaScript, including where it reads oddly: tools/engine_sound_parity
//  compares this against the original rendered under Node, and a "harmless"
//  rearrangement shows up there. Variable names are the worklet's.
//===========================================================================

namespace vsim {
namespace {

constexpr double TAU = kPi * 2.0;
constexpr double kCompKnee = 1.5, kCompKneeE = 0.5;
constexpr double kCompG = 0.29, kCompE0 = 4.0;                    // leveller: gain at the reference level, reference level
constexpr double kJetA = 0.3, kJetG = 1.3, kRadA = 0.33;          // exhaust brightness: jet-noise bandwidth / level, radiation band limit

/// JavaScript Math.round for the non-negative values used here.
int jsRound(double x) { return static_cast<int>(std::floor(x + 0.5)); }

/// Python's float % (engine_sim/sound_spec.py).
double pymod(double a, double b) {
    double r = std::fmod(a, b);
    if (r != 0.0 && ((b < 0.0) != (r < 0.0))) r += b;
    return r;
}

/// sound_spec.speed_of_sound: c = sqrt(gamma R T) ~ 20.05 sqrt(T).
double speedOfSound(double tempK) { return 20.05 * std::sqrt(tempK); }

double pick(const std::vector<double>& values, std::size_t k, double fallback) {
    if (values.empty()) return fallback;
    return k < values.size() ? values[k] : values.back();
}

} // namespace

//---------------------------------------------------------------------------
//  engine_sim/sound_spec.py
//---------------------------------------------------------------------------

EngineSimSoundSpec makeEngineSimSoundSpec(const EngineSimSoundConfig& sound,
                                          const EngineParameters& engineIn) {
    EngineParameters engine = engineIn;
    engine.validate();
    const ResolvedEngineParameters& r = engine.resolved;
    EngineSimSoundSpec sp;
    const int n = engine.numCylinders;
    const std::size_t un = static_cast<std::size_t>(n);
    sp.cylinders = n;

    // EngineConfig.fire_angles_deg
    sp.fireAngleDeg.assign(un, 0.0);
    double a = 0.0;
    for (std::size_t k = 0; k < un; ++k) {
        sp.fireAngleDeg[static_cast<std::size_t>(r.firingOrder[k] - 1)] = pymod(a, 720.0);
        a += r.firingIntervalsDeg.empty() ? 720.0 / n : r.firingIntervalsDeg[k];
    }

    // Valve events, in degrees after each cylinder's firing TDC.
    const double io = pymod(720.0 - r.intakeOpenBtdcDeg, 720.0);
    const double eo = 540.0 - r.exhaustOpenBbdcDeg;
    sp.evoAfterFireDeg = eo - 360.0;
    sp.ivoAfterFireDeg = pymod(io - 360.0, 720.0);

    // One pipe per bank on a dual exhaust; everything into one otherwise.
    std::vector<int> bank(un, 0);
    if (sound.cylinderBank.size() == un) {
        for (std::size_t k = 0; k < un; ++k) bank[k] = std::max(0, sound.cylinderBank[k]);
    }
    const int banks = *std::max_element(bank.begin(), bank.end()) + 1;
    const int pipes = sound.dualExhaust ? banks : 1;
    sp.cylinderPipe.assign(un, 0);
    if (sound.dualExhaust) sp.cylinderPipe = bank;

    const double cHead = speedOfSound(sound.headerTempK);
    const double cMid = speedOfSound(sound.midTempK);
    const double cMuf = speedOfSound(sound.mufflerTempK);
    const double cTail = speedOfSound(sound.tailTempK);
    for (int k = 0; k < pipes; ++k) {
        const std::size_t uk = static_cast<std::size_t>(k);
        EngineSimSoundSpec::Pipe pipe;
        pipe.sections[0] = {pick(sound.headerLengthM, uk, 0.55), cHead, 1.0};
        pipe.sections[1] = {pick(sound.midLengthM, uk, 1.10), cMid, 1.0};
        pipe.sections[2] = {pick(sound.mufflerLengthM, uk, 0.40), cMuf, sound.mufflerAreaRatio};
        pipe.sections[3] = {pick(sound.tailLengthM, uk, 0.90), cTail, 1.0};
        pipe.bodyHz = pick(sound.mufflerBodyHz, uk, 125.0);
        sp.pipes.push_back(pipe);
    }
    sp.pipeLoss = sound.pipeLoss;
    sp.tipDamping = sound.tipDamping;
    sp.runnerLengthM = sound.intakeRunnerLengthM;
    sp.runnerC = speedOfSound(sound.intakeRunnerTempK);
    sp.blowdownTauMaxS = sound.blowdownK * engine.bore / cHead;
    sp.inertiaKgM2 = engine.engineInertia;
    sp.rippleGain = sound.crankRippleGain;
    sp.cylinderStrengthSpread = sound.cylinderStrengthSpread;
    sp.cylinderTimingSpreadDeg = sound.cylinderTimingSpreadDeg;
    sp.seed = sound.noiseSeed;
    sp.variabilityBase = sound.combustionVariability;
    sp.variabilityIdle = sound.idleVariability;
    sp.popProbability = sound.popProbability;
    sp.starterRatio = sound.starterRatio;
    sp.starterPoles = sound.starterPoles;
    return sp;
}

//---------------------------------------------------------------------------
//  The worklet's helper classes
//---------------------------------------------------------------------------

void EngineSimSynth::Biquad::set(Type type, double fc, double q, double sr, double gainDb) {
    const double w = TAU * std::min(fc, sr * 0.45) / sr, c = std::cos(w), s = std::sin(w), al = s / (2 * q);
    double B0, B1, B2, a0, A1, A2;
    if (type == LP) { B0 = (1 - c) / 2; B1 = 1 - c; B2 = B0; a0 = 1 + al; A1 = -2 * c; A2 = 1 - al; }
    else if (type == HP) { B0 = (1 + c) / 2; B1 = -(1 + c); B2 = B0; a0 = 1 + al; A1 = -2 * c; A2 = 1 - al; }
    else if (type == BP) { B0 = al; B1 = 0; B2 = -al; a0 = 1 + al; A1 = -2 * c; A2 = 1 - al; }
    else if (type == LS || type == HS) {       // RBJ shelving, slope S = 1
        const double A = std::pow(10.0, gainDb / 40), sq = 2 * std::sqrt(A) * (s / 2) * std::sqrt(2.0);
        if (type == LS) {
            B0 = A * ((A + 1) - (A - 1) * c + sq); B1 = 2 * A * ((A - 1) - (A + 1) * c); B2 = A * ((A + 1) - (A - 1) * c - sq);
            a0 = (A + 1) + (A - 1) * c + sq; A1 = -2 * ((A - 1) + (A + 1) * c); A2 = (A + 1) + (A - 1) * c - sq;
        } else {
            B0 = A * ((A + 1) + (A - 1) * c + sq); B1 = -2 * A * ((A - 1) + (A + 1) * c); B2 = A * ((A + 1) + (A - 1) * c - sq);
            a0 = (A + 1) - (A - 1) * c + sq; A1 = 2 * ((A - 1) - (A + 1) * c); A2 = (A + 1) - (A - 1) * c - sq;
        }
    } else {                                   // peaking EQ
        const double A = std::pow(10.0, gainDb / 40);
        B0 = 1 + al * A; B1 = -2 * c; B2 = 1 - al * A; a0 = 1 + al / A; A1 = -2 * c; A2 = 1 - al / A;
    }
    b0 = B0 / a0; b1 = B1 / a0; b2 = B2 / a0; a1 = A1 / a0; a2 = A2 / a0;
}

void EngineSimSynth::Pipe::init(double sr, double lengthM, double soundSpeed, double reflection,
                                double damping) {
    const int D = std::max(2, jsRound(2 * lengthM / soundSpeed * sr));
    buf.assign(static_cast<std::size_t>(D), 0.0f);
    i = 0; g = reflection; a = damping; lp = 0.0;
}

double EngineSimSynth::Pipe::process(double x) {
    lp += a * (buf[i] - lp);
    const double y = x + g * lp;
    buf[i] = static_cast<float>(y);
    if (++i == buf.size()) i = 0;
    return y;
}

void EngineSimSynth::Shaper::setTau(double tauS, double sr) {
    a = 1 - std::exp(-1 / std::max(1.0, tauS * sr));
    norm = 1 / (a * 0.2707);                 // peak of the 3-pole impulse response ~ 0.2707 a
}

void EngineSimSynth::Delay::init(int n) {
    b.assign(static_cast<std::size_t>(std::max(1, n)), 0.0f);
    i = 0;
}

void EngineSimSynth::ExhaustPipe::init(double sr, const EngineSimSoundSpec::Pipe& pipe,
                                       double lossPerPass, double tipDamping) {
    const std::size_t S = pipe.sections.size();
    F.assign(S, Delay{});
    B.assign(S, Delay{});
    for (std::size_t j = 0; j < S; ++j) {
        const int n = jsRound(pipe.sections[j].lengthM / pipe.sections[j].c * sr);
        F[j].init(n);
        B[j].init(n);
    }
    K.assign(S - 1, 0.0f);
    // Junction reflection from the area ratio. A plain expansion chamber of ratio 6 has deep stop-bands
    // that notch out the V8's rumble and leave a fixed-pitch honk; real mufflers are perforated /
    // absorptive, so the effective ratio is compressed (A^0.65).
    std::vector<double> ae(S);
    for (std::size_t j = 0; j < S; ++j) ae[j] = std::pow(pipe.sections[j].area, 0.65);
    for (std::size_t j = 0; j + 1 < S; ++j) {
        K[j] = static_cast<float>((ae[j] - ae[j + 1]) / (ae[j] + ae[j + 1]));
    }
    fo.assign(S, 0.0f);
    bo.assign(S, 0.0f);
    loss = lossPerPass;        // wall friction per pass
    rValve = 0.7;              // valve/port end: mostly closed -> positive reflection
    rOpen = -0.9;              // open tip: inverted reflection ...
    aOpen = tipDamping;        // ... losing highs to radiation
    lp = 0.0;
}

double EngineSimSynth::ExhaustPipe::process(double src) {
    const std::size_t S = F.size();
    const double g = loss;
    for (std::size_t j = 0; j < S; ++j) {
        fo[j] = static_cast<float>(F[j].out() * g);
        bo[j] = static_cast<float>(B[j].out() * g);
    }
    F[0].push(src + rValve * bo[0]);
    for (std::size_t j = 0; j + 1 < S; ++j) {
        const double k = K[j];
        F[j + 1].push((1 + k) * fo[j] - k * bo[j + 1]);
        B[j].push(k * fo[j] + (1 - k) * bo[j + 1]);
    }
    const double tip = fo[S - 1];
    lp += aOpen * (tip - lp);
    B[S - 1].push(rOpen * lp);
    return tip + rOpen * lp;                 // the transmitted (radiated) part
}

void EngineSimSynth::Reverb::init(double sr) {
    const double k = sr / 44100;
    auto mk = [k](Line& line, int n) {
        line.b.assign(static_cast<std::size_t>(std::max(2, jsRound(n * k))), 0.0f);
        line.i = 0; line.f = 0.0;
    };
    const int cl[4] = {1116, 1188, 1277, 1356}, cr[4] = {1139, 1211, 1300, 1379};
    const int al[2] = {556, 441}, ar[2] = {579, 464};
    for (std::size_t j = 0; j < 4; ++j) { mk(cL[j], cl[j]); mk(cR[j], cr[j]); }
    for (std::size_t j = 0; j < 2; ++j) { mk(aL[j], al[j]); mk(aR[j], ar[j]); }
    l = r = 0.0;
}

double EngineSimSynth::Reverb::side(double x, std::array<Line, 4>& combs, std::array<Line, 2>& aps) {
    double sum = 0;
    for (Line& c : combs) {
        const double y = c.b[c.i];
        c.f += damp * (y - c.f);             // damping: low-pass in the feedback path
        c.b[c.i] = static_cast<float>(x + fb * c.f);
        if (++c.i == c.b.size()) c.i = 0;
        sum += y;
    }
    sum *= 0.25;
    for (Line& a : aps) {
        const double y = a.b[a.i];
        const double out = y - sum;
        a.b[a.i] = static_cast<float>(sum + 0.5 * y);
        if (++a.i == a.b.size()) a.i = 0;
        sum = out;
    }
    return sum;
}

//---------------------------------------------------------------------------
//  EngineSynth
//---------------------------------------------------------------------------

EngineSimSynth::EngineSimSynth(double sampleRate) : m_sr(sampleRate) {
    const double sr = m_sr;
    m_intakeShaper.setTau(0.0015, sr);
    m_dcL.set(Biquad::HP, 25, 0.7, sr);
    m_dcR.set(Biquad::HP, 25, 0.7, sr);
    m_intakeLP.set(Biquad::LP, 1200, 0.7, sr);
    m_hissHP.set(Biquad::HP, 2200, 0.7, sr);
    m_hissLP.set(Biquad::LP, 6500, 0.7, sr);
    m_roadLP.set(Biquad::LP, 380, 0.7, sr);
    m_tickBP.set(Biquad::BP, 2300, 3, sr);
    m_outLPL.set(Biquad::LP, 7000, 0.7, sr);
    m_outLPR.set(Biquad::LP, 7000, 0.7, sr);
    m_reverb.init(sr);
    const double taps[4] = {0.0071, 0.0133, 0.0197, 0.0293};     // early reflections [s]
    for (std::size_t j = 0; j < 4; ++j) m_erD[j] = jsRound(taps[j] * sr);
    deriveProfile();
}

/// Build everything engine-specific from the spec. Nothing about a particular
/// engine lives in this file.
void EngineSimSynth::configure(const EngineSimSoundSpec& spec) {
    m_sp = spec;
    m_configured = false;
    const EngineSimSoundSpec& sp = m_sp;
    const double sr = m_sr;
    const int n = m_n = sp.cylinders;
    if (n <= 0 || sp.pipes.empty()) return;
    const std::size_t un = static_cast<std::size_t>(n);

    m_cylPipe = sp.cylinderPipe;
    m_cylPipe.resize(un, 0);
    m_pipes = static_cast<int>(sp.pipes.size());
    m_ex.assign(sp.pipes.size(), Exhaust{});
    for (std::size_t b = 0; b < sp.pipes.size(); ++b) {
        m_ex[b].pipe.init(sr, sp.pipes[b], sp.pipeLoss, sp.tipDamping);
        m_ex[b].body.set(Biquad::Peak, sp.pipes[b].bodyHz, 0.9, sr, 4);     // muffler shell / body
    }
    for (int& pipe : m_cylPipe) pipe = std::clamp(pipe, 0, m_pipes - 1);
    if (m_pipes > 1) m_pan = {{{0.95, 0.3}, {0.3, 0.95}}};
    else             m_pan = {{{0.8, 0.8}, {0.8, 0.8}}};
    m_intakeRunner.init(sr, sp.runnerLengthM, sp.runnerC, -0.55, 0.35);

    // Fixed per-cylinder differences (runner length, trim, ring seal). They
    // repeat every cycle, so they add sub-harmonics: the lumpy idle "lope".
    double seed = sp.seed;
    auto rnd = [&seed]() {
        seed = std::fmod(seed * 1103515245.0 + 12345.0, 2147483648.0);
        return seed / 2147483648.0 - 0.5;
    };
    m_cylGain.assign(un, 1.0f);
    for (std::size_t c = 0; c < un; ++c) m_cylGain[c] = static_cast<float>(1 + sp.cylinderStrengthSpread * rnd());

    // Sorted event list: exhaust open (type 0) and intake open (type 1), at
    // the engine's valve timing, relative to each cylinder's firing TDC.
    struct Event { double angle; int type; int cylinder; double offset; };
    std::vector<Event> ev;
    for (int c = 0; c < n; ++c) {
        const double fire = sp.fireAngleDeg[static_cast<std::size_t>(c)];
        ev.push_back({std::fmod(fire + sp.evoAfterFireDeg + 720, 720.0), 0, c, sp.cylinderTimingSpreadDeg * rnd()});
        ev.push_back({std::fmod(fire + sp.ivoAfterFireDeg + 720, 720.0), 1, c, 0.0});
    }
    std::stable_sort(ev.begin(), ev.end(), [](const Event& x, const Event& y) { return x.angle < y.angle; });
    m_E = static_cast<int>(ev.size());
    m_evA.clear(); m_evT.clear(); m_evC.clear(); m_evO.clear();
    for (const Event& e : ev) {
        m_evA.push_back(e.angle);
        m_evT.push_back(e.type);
        m_evC.push_back(e.cylinder);
        m_evO.push_back(e.offset);            // per-event timing offset [deg]
    }
    m_theta = 0; m_next = 0; m_nextBase = m_evA[0];
    m_configured = true;
    deriveProfile();
    m_nextA = m_nextBase + m_evO[0] * m_lopeS;
}

void EngineSimSynth::applyProfile(const Profile& p) {
    auto take = [](double& dst, double v) { if (std::isfinite(v)) dst = v; };
    take(m_pf.vol, p.vol); take(m_pf.conv, p.conv); take(m_pf.comp, p.comp); take(m_pf.drive, p.drive);
    take(m_pf.lf, p.lf); take(m_pf.hf, p.hf); take(m_pf.ring, p.ring); take(m_pf.rasp, p.rasp);
    take(m_pf.lope, p.lope); take(m_pf.nlf, p.nlf); take(m_pf.nhf, p.nhf); take(m_pf.tick, p.tick);
    take(m_pf.intk, p.intk); take(m_pf.pops, p.pops);
    deriveProfile();
}

/// applyProfile(): turn the dials into the scalars / filters the audio loop uses.
void EngineSimSynth::deriveProfile() {
    const Profile& f = m_pf;
    const double sr = m_sr;
    m_erScale = std::min(3.0, f.conv / 0.35);
    const double over = std::max(0.0, f.conv - 0.35);
    m_revWet = 0.8 * (over * over);                              // tail only above the default room
    m_shelfLow = std::abs(f.lf) > 0.05; m_shelfHigh = std::abs(f.hf) > 0.05;
    m_lsL.set(Biquad::LS, 160, 1, sr, f.lf); m_lsR.set(Biquad::LS, 160, 1, sr, f.lf);
    m_hsL.set(Biquad::HS, 2500, 1, sr, f.hf); m_hsR.set(Biquad::HS, 2500, 1, sr, f.hf);
    const double lpHz = 7000 * (1 + std::max(0.0, f.hf) / 12 * 0.7);   // let boosted treble through
    m_outLPL.set(Biquad::LP, lpHz, 0.7, sr); m_outLPR.set(Biquad::LP, lpHz, 0.7, sr);
    for (Exhaust& e : m_ex) {                                    // exhaust ring: pipe reflections
        e.pipe.rValve = 0.15 + 0.5 * f.ring;                     // loop gain 0.34 at the default: pipe modes
        e.pipe.rOpen = -(0.45 + 0.5 * f.ring);                    // are broad (Q ~ 4), not a fixed-pitch hum
    }
    m_radW = 8.8 * f.rasp * f.rasp;                              // 2.2 at the default 0.5
    m_lopeS = std::pow(f.lope / 0.5, 1.5);                       // 1 at default, ~2.8 at max
    m_nlf = f.nlf; m_nhf = f.nhf;                                // additive noise sources
    m_tickA = f.tick * 1.5;                                      // extra valve-tick per event
    m_intkS = 1 + 11 * f.intk;                                   // intake level multiplier
    m_popsS = f.pops * 2;
    m_compExp = 0.2 * f.comp / 0.6;                              // leveller strength: 0 = flat gain, 0.2 at the default
    m_driveK = 1 + 4 * f.drive;
}

double EngineSimSynth::noise() {             // xorshift32 -> [-1, 1)
    std::uint32_t x = m_seed;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    m_seed = x;
    return m_seed / 2147483648.0 - 1;
}

/// Exhaust valve opens on cylinder c: kick that pipe's pulse shapers.
void EngineSimSynth::exhaustEvent(int c) {
    const State& s = m_s;
    const double rpm = std::max(m_rpm, 30.0), map = m_map;
    Exhaust& e = m_ex[static_cast<std::size_t>(m_cylPipe[static_cast<std::size_t>(c)])];
    // A real exhaust gets much louder with speed (more, faster flow) and with load (higher
    // blowdown pressure): ~20 dB from idle to full throttle at the limiter, not 5.
    const double rpmAmp = std::pow(std::min(std::max(rpm, 300.0), 9000.0) / 800, 0.6);
    double bd, disp, noiseAmt;
    if (s.combusting) {
        // p_EVO - p_exh: grows with charge; light load is a soft "chuff", full load a hard crack
        const double cov = (m_sp.variabilityBase + m_sp.variabilityIdle * (1 - map)) * std::min(2.0, m_lopeS);
        const double k = (1 + (m_cylGain[static_cast<std::size_t>(c)] - 1) * m_lopeS)
                       * std::max(0.25, 1 + cov * gauss()) * rpmAmp;
        // boost adds some noise but a turbo engine is not 3x louder: count it at a quarter
        const double load = std::pow(std::min(map, 1.0) + 0.25 * std::max(0.0, std::min(map, 2.5) - 1), 1.2);
        bd = (0.2 + 0.8 * load) * k;
        disp = (0.08 + 0.2 * load) * k;
        noiseAmt = 0.15 + 0.85 * load * load;     // jet noise through the valve: grows steeply with load
    } else {
        // Motoring (fuel cut): the cylinder holds only the expanded,
        // sub-atmospheric charge, so when the exhaust valve opens gas rushes
        // back IN (reversion) - a strong negative pulse at high vacuum; this
        // is the hollow note of engine braking.
        bd = 0.5 * (map - 0.45) * rpmAmp; disp = 0.08 * rpmAmp; noiseAmt = 0.05;
        if (s.pops && s.ignition && s.fuelCut && m_fuelCutT < 2.0 && rpm > 1800 &&
            (noise() + 1) * 0.5 < m_sp.popProbability * m_popsS) {          // overrun pop
            bd = (1.4 + 0.4 * (noise() + 1)) * rpmAmp; noiseAmt = 1.5;
        }
    }
    e.bd.kick(bd);
    e.disp.kick(disp);
    e.noise = noiseAmt;
    m_tickEnv += 0.0007 * std::min(1.0, rpm / 3000) + m_tickA * std::min(2.5, rpm / 1200);   // valve seating tick
}

void EngineSimSynth::process(float* L, float* R, int N) {
    if (!m_configured) {                     // no engine yet
        std::fill(L, L + N, 0.0f);
        std::fill(R, R + N, 0.0f);
        return;
    }
    const State& s = m_s;
    const double sr = m_sr;
    const EngineSimSoundSpec& sp = m_sp;
    const double kRpm = 1 - std::exp(-1 / (0.03 * sr));
    const double kMap = 1 - std::exp(-1 / (0.02 * sr));
    const double kTick = std::exp(-1 / (0.0008 * sr));
    const double kSol = std::exp(-1 / (0.025 * sr));
    const double kCompA = 1 - std::exp(-1 / (0.005 * sr)), kCompR = 1 - std::exp(-1 / (0.25 * sr));
    m_intakeBP.set(Biquad::BP, 250 + m_rpm * 0.12, 1.1, sr);
    {
        // Blowdown: the initial choked burst through the opening valve is
        // sub-millisecond; the spectrum of this 3-pole pulse falls 18 dB/oct
        // above 1/(2 pi tau) ~ 450 Hz, keeping the 125-500 Hz "blat" the ear
        // hears at idle. Displacement: a quarter of the exhaust stroke.
        const double rpmC = std::max(m_rpm, 60.0);
        const double tauBd = std::min(sp.blowdownTauMaxS, 60 / (rpmC * 6) / 6);
        const double tauDisp = std::max(0.002, 180 / (rpmC * 6) / 4.5);
        for (Exhaust& e : m_ex) { e.bd.setTau(tauBd, sr); e.disp.setTau(tauDisp, sr); }
    }
    m_starterBP.set(Biquad::BP, 180 + m_rpm * 1.2, 2.0, sr);
    if (s.fuelCut) m_fuelCutT += N / sr; else m_fuelCutT = 0;
    if (s.starter && !m_starterPrev) m_solenoid = 1;
    if (!s.starter && m_starterPrev) m_solenoid = 0.5;
    m_starterPrev = s.starter;
    const double vol = s.muted ? 0 : m_pf.vol;
    const double lopeS = m_lopeS, nlf = m_nlf, nhf = m_nhf, intkS = m_intkS, radW = m_radW;
    const bool shelfLow = m_shelfLow, shelfHigh = m_shelfHigh;
    const double erScale = m_erScale;
    const double revWet = m_revWet, driveK = m_driveK, driveN = 1 / std::sqrt(m_driveK);
    const double redline = (s.redline != 0.0) ? s.redline : 6000;
    const int banks = m_pipes;
    const bool moving = s.speed > 0.5;
    const double v = s.speed / 2.237;
    const double roadAmt = moving ? std::min(1.0, (v / 35) * (v / 35)) * 0.3 : 0;
    const double whineAmt = moving && s.gear > 0 ? 0.006 * (0.3 + std::abs(s.load)) : 0;
    const bool starterOn = s.starter;
    // Steady turbulent flow noise ~ exhaust mass flow: off by default, comes
    // in with the ~HF dial; charge clamped at ambient so boost adds none.
    const double flowRoar = 0.5 * nhf * std::min(1.0, m_rpm / redline) * (s.combusting ? std::min(1.0, m_map) : 0.2);
    // Crank-speed ripple from the physics: each firing delivers
    // W_cyl = T_ind 4 pi / N into the flywheel, a speed swing ~ g W_cyl / (I w^2).
    const double kT = 1 - std::exp(-N / (0.03 * sr));
    m_tqInd += (s.indicated - m_tqInd) * kT; m_tqFric += (s.friction - m_tqFric) * kT; m_tqPump += (s.pumping - m_tqPump) * kT;
    const double om = m_rpm * (kPi / 30);
    double rippleBlk = 0;
    if (m_rpm > 20) {
        const double tq = s.combusting ? m_tqInd : 0.35 * (m_tqFric + m_tqPump);   // motoring: compression asymmetry only
        const double Wcyl = std::max(0.0, tq) * 4 * kPi / m_n;
        rippleBlk = std::min(0.6, std::min(0.25, sp.rippleGain * Wcyl / (sp.inertiaKgM2 * om * om)) * lopeS);
    }
    // slow random rpm wander (real engines never hold a perfectly steady speed)
    m_wander += (gauss() - m_wander) * (1 - std::exp(-N / (0.25 * sr)));
    const double wanderK = 1 + m_wander * 0.0015;
    // compressor gain: evaluated per block from the level envelope
    // Gentle leveller, not a flattener: out ~ env^(1-e). Idle and full throttle must stay ~20 dB
    // apart, as on a real car; the old env^0.5 law squeezed that to 5 dB. (at e = 0 the gain is flat)
    const double xe = (m_compEnv + 0.02) / kCompE0;
    double g = kCompG * std::pow(xe, -m_compExp);
    if (xe > kCompKnee) g *= std::pow(xe / kCompKnee, -kCompKneeE);          // limiter above the knee: holds the peaks under the soft clip
    g = g < 0.02 ? 0.02 : g > 5 ? 5 : g;
    const double gS = m_compGain, gE = gS + (g - gS) * (1 - std::exp(-N / (0.08 * sr)));
    m_compGain = gE;
    const double gStart = gS, gEnd = gE;

    const int mask = kErLen - 1;
    for (int i = 0; i < N; i++) {
        m_rpm += kRpm * (s.rpm - m_rpm);
        m_map += kMap * (s.map - m_map);
        m_throttle += kMap * (s.throttle - m_throttle);
        const double rpm = m_rpm, map = m_map;

        // --- crank angle with per-stroke speed ripple (idle lope) ---
        if (rpm > 1) {
            const double ripple = rippleBlk;
            // speed ~ 1 + a sin(): a cycle then takes T0/sqrt(1-a^2), so divide
            // by sqrt(1-a^2) to keep the mean speed (and the pitch) at the rpm
            m_theta += rpm * wanderK * 6 / sr / std::sqrt(1 - ripple * ripple)
                     * (1 + ripple * std::sin(m_theta * TAU * m_n / 720));
            while (m_theta >= m_nextA) {             // walk the sorted event list
                const int e = m_next;
                const std::size_t ue = static_cast<std::size_t>(e);
                if (m_evT[ue] == 0) exhaustEvent(m_evC[ue]);
                else m_intakeShaper.kick(-0.25 * (0.25 + m_throttle));
                const int nx = e + 1 == m_E ? 0 : e + 1;
                const std::size_t unx = static_cast<std::size_t>(nx);
                double gap = m_evA[unx] - m_evA[ue]; if (gap < 0) gap += 720;
                // No random per-event timing: irregularity must repeat every
                // cycle (fixed per-cylinder offsets) to stay tonal - random
                // jitter smears the harmonics into broadband "wind".
                m_next = nx; m_nextBase += gap;
                m_nextA = m_nextBase + m_evO[unx] * lopeS;
            }
            if (m_theta >= 720) { m_theta -= 720; m_nextA -= 720; m_nextBase -= 720; }
        }

        // --- exhaust: shaped pulses -> waveguide -> radiation ---
        const double nz = noise();
        m_noiseLP += 0.21 * (nz - m_noiseLP);             // ~1.8 kHz turbulence (intake, valvetrain)
        m_noiseLP2 += 0.21 * (m_noiseLP - m_noiseLP2);
        m_jetLP += kJetA * (nz - m_jetLP); m_jetLP2 += kJetA * (m_jetLP - m_jetLP2);   // jet noise through the valve: wider band, 12 dB/oct roll-off
        const double turb = m_noiseLP2 * 2.2, jet = m_jetLP2 * kJetG;
        double oL = 0, oR = 0;
        for (int b = 0; b < banks; b++) {
            Exhaust& e = m_ex[static_cast<std::size_t>(b)];
            const double bdv = e.bd.process();
            const double env = bdv < 0 ? -bdv : bdv;
            const double dispv = e.disp.process();
            const double src = bdv + dispv + turb * (env * e.noise * 0.12 + flowRoar * map * map) + jet * env * e.noise;
            const double p = e.pipe.process(src);
            const double rad = p - e.prev; e.prev = p;    // far-field radiation ~ d/dt
            e.radLP += kRadA * (rad - e.radLP);           // ... band-limited: no sharp edges
            const double y = e.body.process(p * 0.8 + e.radLP * radW + (bdv + dispv) * 0.45) * 6.0;      // make-up gain; + radiation that bypasses the muffler
            const auto& pan = m_pan[static_cast<std::size_t>(b & 1)];
            oL += y * pan[0]; oR += y * pan[1];
        }

        // --- intake: runner pulses + roar with airflow ---
        const double flow = std::min(1.2, rpm / redline) * (0.15 + m_throttle);
        const double roar = m_intakeBP.process(m_noiseLP) * flow * 0.2;
        const double intake = (m_intakeLP.process(m_intakeRunner.process(m_intakeShaper.process() * (0.3 + map) + roar * 0.3)) * 0.6 + roar * 0.3) * intkS;
        // throttle hiss: needs real airflow through a nearly shut plate
        // (decel at speed), not idle - idle airflow is tiny
        const double vac = map < 0.6 ? (0.6 - map) / 0.6 : 0;
        const double rn = rpm / redline;
        const double hissAmt = vac * rn * std::sqrt(rn) * 0.02
                             + nhf * (0.1 + 3 * rn * rn) * (0.4 + 0.6 * m_throttle) * 0.5;
        double hiss = 0;
        if (hissAmt > 1e-4) hiss = m_hissLP.process(m_hissHP.process(noise())) * hissAmt;
        // ~LF: low-frequency rumble, noise low-passed to ~110 Hz
        double rumble = 0;
        if (nlf > 0) {                       // own random stream: dials never disturb the engine's noise sequence
            std::uint32_t x = m_seed2; x ^= x << 13; x ^= x >> 17; x ^= x << 5; m_seed2 = x;
            m_rumLP += 0.0143 * (m_seed2 / 2147483648.0 - 1 - m_rumLP); m_rumLP2 += 0.0143 * (m_rumLP - m_rumLP2);
            rumble = m_rumLP2 * nlf * 9 * (0.3 + 0.7 * rn) * (0.5 + 0.5 * map);
        }
        oL += intake + hiss + rumble; oR += intake + hiss + rumble;

        // --- valvetrain tick ---
        if (m_tickEnv > 1e-5) {
            const double tk = m_tickBP.process(m_noiseLP * m_tickEnv) * 2.0;
            if (m_tickEnv > 3.5) m_tickEnv = 3.5;
            m_tickEnv *= kTick;
            oL += tk; oR += tk;
        }

        // --- starter motor ---
        if (starterOn || m_solenoid > 0.001) {
            double st = 0;
            if (starterOn) {
                const double fArm = (rpm > 40 ? rpm : 40) * sp.starterRatio / 60;      // armature speed from the starter gearing
                m_starterPhase += fArm * sp.starterPoles / sr; if (m_starterPhase >= 1) m_starterPhase -= 1;
                const double band = m_starterBP.process(m_starterPhase * 2 - 1 + 0.4 * noise()) * 0.35;
                st = band + std::sin(TAU * m_starterPhase * 2) * 0.05;
            }
            if (m_solenoid > 0.001) {
                m_solenoidPhase += 70 / sr;
                const double tone = std::sin(TAU * m_solenoidPhase) * m_solenoid * 0.6;
                st += tone + noise() * m_solenoid * 0.2;
                m_solenoid *= kSol;
            }
            oL += st; oR += st;
        }

        // --- driveline whine + road/wind noise (never, here: speed is 0) ---
        if (roadAmt > 0) {
            const double road = m_roadLP.process(noise()) * roadAmt;
            double wh = 0;
            if (whineAmt > 0) {
                m_whinePhase += rpm / 60 * 17 / sr; if (m_whinePhase >= 1) m_whinePhase -= 1;
                wh = std::sin(TAU * m_whinePhase) * whineAmt;
            }
            oL += road + wh; oR += road * 0.9 + wh;
        }

        // --- early reflections ---
        const int w = m_erI;
        m_erBuf[static_cast<std::size_t>(w)] = static_cast<float>((oL + oR) * 0.5);
        const double r0 = m_erBuf[static_cast<std::size_t>((w - m_erD[0]) & mask)],
                     r1 = m_erBuf[static_cast<std::size_t>((w - m_erD[1]) & mask)],
                     r2 = m_erBuf[static_cast<std::size_t>((w - m_erD[2]) & mask)],
                     r3 = m_erBuf[static_cast<std::size_t>((w - m_erD[3]) & mask)];
        const double dry = (oL + oR) * 0.5;
        oL += (r0 * m_erGL[0] + r1 * m_erGL[1] + r2 * m_erGL[2] + r3 * m_erGL[3]) * erScale;
        oR += (r0 * m_erGR[0] + r1 * m_erGR[1] + r2 * m_erGR[2] + r3 * m_erGR[3]) * erScale;
        m_erI = (w + 1) & mask;
        if (revWet > 1e-3) {
            m_reverb.process(dry);
            oL += m_reverb.l * revWet; oR += m_reverb.r * revWet;
        }

        // --- compressor (evens out idle vs WOT loudness), DC block, soft clip ---
        oL = m_outLPL.process(m_dcL.process(oL)); oR = m_outLPR.process(m_dcR.process(oR));
        if (shelfLow) { oL = m_lsL.process(oL); oR = m_lsR.process(oR); }
        if (shelfHigh) { oL = m_hsL.process(oL); oR = m_hsR.process(oR); }
        const double lvl = (oL < 0 ? -oL : oL) + (oR < 0 ? -oR : oR);
        m_compEnv += (lvl > m_compEnv ? kCompA : kCompR) * (lvl - m_compEnv);
        const double cg = (gStart + (gEnd - gStart) * i / N) * 0.8;
        oL = std::tanh(oL * cg * driveK) * driveN * vol;
        oR = std::tanh(oR * cg * driveK) * driveN * vol;
        L[i] = static_cast<float>(oL); R[i] = static_cast<float>(oR);
    }
}

} // namespace vsim
