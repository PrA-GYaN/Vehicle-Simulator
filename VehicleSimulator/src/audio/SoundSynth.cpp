#include "audio/SoundSynth.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>

#include "core/Units.h"

namespace vsim {
namespace {

constexpr double kFourPi = 4.0 * kPi;

// Air / exhaust gas - the same constants the engine model uses (Heywood
// Table D.1), so the gas the sound travels through is the gas the engine
// burned.
constexpr double kGamma = 1.4;
constexpr double kRAir = 287.058;           // [J/(kg.K)]
constexpr double kCp = kGamma * kRAir / (kGamma - 1.0);   // ~1005 J/(kg.K)
constexpr double kSpeedOfSoundAir = 343.0;  // [m/s] at 20 C - propagation to the listener
constexpr double kRhoAir = 1.2;             // [kg/m^3]

/// Radial compressor work coefficient psi = specific work / U_tip^2. Euler's
/// equation gives the slip factor times (1 - backsweep term) - 0.7-0.9 for
/// automotive compressor wheels. AUTHORED at the middle of that range.
constexpr double kCompressorWorkCoefficient = 0.8;

/// Radius of the tailpipe's open end [m]: sets where ka = 1, below which the
/// open end reflects rather than radiates.
constexpr double kTailpipeRadius = 0.03;

/// Reference pressure difference for a "full-strength" blowdown pulse [Pa].
/// Scales the pulses into a sensible range; loudness relationships do not
/// depend on it.
constexpr double kBlowdownReference = 5.0e5;

/// Engine speed the pulse flow RATES are normalised at [rad/s] (4000 rpm).
/// A pulse's volume velocity is the charge it moves divided by the time it
/// takes, and that time is a fixed crank angle - so flow rate goes as
/// (charge) x omega. Scale only; the omega dependence is the physics.
constexpr double kFlowReferenceOmega = 4000.0 * 2.0 * 3.14159265358979 / 60.0;

/// Strouhal number of vortex shedding from bluff bodies - the frequency
/// wind noise centres on is St U / D. 0.2 is the classic cylinder value.
constexpr double kStrouhal = 0.2;

/// Level calibration per source. The synthesis gives each source's
/// DEPENDENCE on the physics (rpm, load, boost, slip, speed); it cannot give
/// absolute loudness, because nothing here is in calibrated pascals. These
/// set the balance between sources at a reference condition - the F40 at
/// 6000 rpm WOT, full boost, 40 m/s, heard from the chase camera - to
/// roughly: exhaust -12 dBFS, turbo -24, intake -22, gear whine -30, tyre
/// squeal -18, wind -28. AUTHORED; the UI mix sliders sit on top.
/// Block and body structural modes [Hz] with their Q and relative gain -
/// what a combustion impact rings. AUTHORED: no structural model exists
/// for any of these cars; the values sit where engine-block bending, mount
/// and floor/bulkhead panel modes typically are (tens to hundreds of Hz).
constexpr double kModeHz[6]   = {52.0, 97.0, 158.0, 246.0, 380.0, 590.0};
constexpr double kModeQ[6]    = {5.0, 7.0, 8.0, 9.0, 10.0, 12.0};
constexpr float  kModeGain[6] = {1.0f, 0.9f, 0.75f, 0.6f, 0.45f, 0.3f};

/// Indicated work per cylinder per cycle that counts as a "full" combustion
/// impact [J]; F40 at full boost is ~900 J. Scale only.
constexpr double kCombustionReferenceJ = 1000.0;
/// Second-order shaking force treated as "full" [N]. Scale only.
constexpr double kShakingReferenceN = 4000.0;

/// dU/dt normalised to unity gain at this frequency - keeps the radiated
/// level in a sensible numeric range. Scale only.
constexpr double kRadiationRefHz = 500.0;

constexpr float kExhaustGain = 1.55f;
constexpr float kIntakeGain  = 5.1f;
constexpr float kTurboGain   = 4.0f;
constexpr float kGearGain    = 12.0f;
constexpr float kTyreGain    = 3.0f;
constexpr float kWindGain    = 6.0f;
constexpr float kStructureGain = 1.7f;
// EngineSim's synthesiser comes out already levelled (its own compressor and
// soft clip, peaking around 0.3-0.7). This puts it on the exhaust bus at the
// loudness the model above has at full throttle from the chase camera, so
// the mix sliders and the other sources keep their balance. AUTHORED.
constexpr float kEngineSimGain = 3.5f;

double smoothstep(double lo, double hi, double x) {
    const double t = std::clamp((x - lo) / (hi - lo), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

double speedOfSound(double temperatureK) {
    return std::sqrt(kGamma * kRAir * std::max(temperatureK, 150.0));
}

/// Did the crank pass `angle` going from `prev` to `now` (both in [0,4pi),
/// `wrapped` if it crossed 4pi -> 0 on the way)?
bool crossed(double prev, double now, bool wrapped, double angle) {
    return wrapped ? (angle > prev || angle <= now) : (angle > prev && angle <= now);
}

} // namespace

//===========================================================================
//  Building blocks
//===========================================================================

void SoundSynth::OnePole::setCutoff(double hz, double fs) {
    hz = std::clamp(hz, 1.0, fs * 0.45);
    a = static_cast<float>(1.0 - std::exp(-2.0 * kPi * hz / fs));
}

void SoundSynth::Biquad::lowpass(double hz, double q, double fs) {
    hz = std::clamp(hz, 10.0, fs * 0.45);
    const double w = 2.0 * kPi * hz / fs, c = std::cos(w), al = std::sin(w) / (2.0 * q);
    const double a0 = 1.0 + al;
    b0 = static_cast<float>((1.0 - c) / 2.0 / a0);
    b1 = static_cast<float>((1.0 - c) / a0);
    b2 = b0;
    a1 = static_cast<float>(-2.0 * c / a0);
    a2 = static_cast<float>((1.0 - al) / a0);
}

void SoundSynth::Biquad::bandpass(double hz, double q, double fs) {
    hz = std::clamp(hz, 10.0, fs * 0.45);
    const double w = 2.0 * kPi * hz / fs, c = std::cos(w), al = std::sin(w) / (2.0 * q);
    const double a0 = 1.0 + al;
    b0 = static_cast<float>(al / a0);   // constant 0 dB peak gain
    b1 = 0.0f;
    b2 = static_cast<float>(-al / a0);
    a1 = static_cast<float>(-2.0 * c / a0);
    a2 = static_cast<float>((1.0 - al) / a0);
}

float SoundSynth::DelayLine::read(double d) const {
    const std::size_t n = buf.size();
    d = std::clamp(d, 1.0, static_cast<double>(n - 2));
    double pos = static_cast<double>(w) - d;
    while (pos < 0.0) pos += static_cast<double>(n);
    const std::size_t i0 = static_cast<std::size_t>(pos) % n;
    const std::size_t i1 = (i0 + 1) % n;
    const float f = static_cast<float>(pos - std::floor(pos));
    return buf[i0] + (buf[i1] - buf[i0]) * f;
}

float SoundSynth::Waveguide::process(float in) {
    // Waves arriving at each end after one trip down the pipe.
    const float atOpenEnd = fwd.read(delay);
    const float atValve   = bwd.read(delay);

    // Open end: below ka ~ 1 nearly everything reflects, inverted (R ~ -1);
    // above it the pipe radiates. The low-pass splits off the reflected part.
    // What leaves is VOLUME VELOCITY, U = (1 - R) p+ : it doubles at low
    // frequency where the pressure cancels - the pipe end is a velocity
    // antinode. The caller takes dU/dt for the far field.
    const float low = openEnd.lp(atOpenEnd);
    const float radiated = atOpenEnd + low;

    fwd.push(wallLoss.lp(in + valveReflection * atValve));
    bwd.push(-openReflection * low);
    return radiated;
}

//===========================================================================
//  Setup
//===========================================================================

SoundSynth::SoundSynth(double sampleRate) : m_fs(sampleRate), m_es(sampleRate) {
    configure(VehicleSoundConfig{}, EngineGeometry{});
}

void SoundSynth::configure(const VehicleSoundConfig& cfg, const EngineGeometry& engine) {
    m_cfg = cfg;
    m_geometry = engine;
    m_numCylinders = std::clamp(engine.cylinders, 1, 16);
    const int n = m_numCylinders;

    m_fireAngles.clear();
    m_fireBank.clear();
    const bool authoredAngles = static_cast<int>(cfg.firingAnglesDeg.size()) == n;
    const bool authoredBanks  = static_cast<int>(cfg.firingBanks.size()) == n;
    int banks = 1;
    for (int k = 0; k < n; ++k) {
        double deg = authoredAngles ? cfg.firingAnglesDeg[static_cast<std::size_t>(k)]
                                    : 720.0 * k / n;
        deg = std::fmod(std::fmod(deg, 720.0) + 720.0, 720.0);
        m_fireAngles.push_back(deg * kPi / 180.0);
        const int b = authoredBanks ? std::clamp(cfg.firingBanks[static_cast<std::size_t>(k)], 0, 3) : 0;
        m_fireBank.push_back(b);
        banks = std::max(banks, b + 1);
    }

    // Per-cylinder signature: fixed strength and timing offsets, from a
    // fixed seed so the same car always sounds the same.
    m_cylGain.assign(static_cast<std::size_t>(n), 1.0);
    m_cylTiming.assign(static_cast<std::size_t>(n), 0.0);
    {
        std::uint32_t r = 0x2545F491u;
        auto uni = [&r]() {
            r ^= r << 13; r ^= r >> 17; r ^= r << 5;
            return static_cast<double>(r) / 4294967295.0 * 2.0 - 1.0;
        };
        for (int k = 0; k < n; ++k) {
            const auto kk = static_cast<std::size_t>(k);
            const double g = (uni() + uni() + uni()) / 1.7320508;
            m_cylGain[kk] = std::max(0.5, 1.0 + cfg.cylinderVariation * g);
            m_cylTiming[kk] = uni() * 0.8 * kPi / 180.0;   // +-0.8 deg
        }
    }

    // Second-order shaking. Piston k is at TDC when the crank is at phi_k, so
    // its second-order inertia force along its bore is
    //   m r w^2 (r/l) cos(2(theta - phi_k)).
    // Summing the bore-direction vectors of all cylinders gives the engine's
    // free force - zero for a balanced layout, by construction.
    {
        const double half = (banks >= 2 ? cfg.veeAngleDeg : 0.0) * 0.5 * kPi / 180.0;
        double sxRe = 0, sxIm = 0, syRe = 0, syIm = 0;
        for (int k = 0; k < n; ++k) {
            const auto kk = static_cast<std::size_t>(k);
            const double ang = (m_fireBank[kk] == 0) ? half : -half;
            const double ex = std::sin(ang), ey = std::cos(ang);
            const double ph = -2.0 * m_fireAngles[kk];
            sxRe += ex * std::cos(ph); sxIm += ex * std::sin(ph);
            syRe += ey * std::cos(ph); syIm += ey * std::sin(ph);
        }
        const double mx = std::hypot(sxRe, sxIm), my = std::hypot(syRe, syIm);
        m_secondOrderFactor = std::hypot(mx, my);
        m_secondOrderPhase = (my >= mx) ? std::atan2(syIm, syRe) : std::atan2(sxIm, sxRe);
        if (m_secondOrderFactor < 1e-6) m_secondOrderFactor = 0.0;
    }
    // Apply the timing scatter after the balance sum (which is geometry).
    for (int k = 0; k < n; ++k) {
        const auto kk = static_cast<std::size_t>(k);
        m_fireAngles[kk] = std::fmod(m_fireAngles[kk] + m_cylTiming[kk] + kFourPi, kFourPi);
    }

    m_banks.assign(static_cast<std::size_t>(banks), Bank{});
    for (Bank& b : m_banks) {
        b.pipe.fwd.resize(8192);
        b.pipe.bwd.resize(8192);
        b.pipe.wallLoss.setCutoff(7000.0, m_fs);
        b.turbine.setCutoff(18000.0, m_fs);
    }
    m_intakeRunner = Waveguide{};
    m_intakeRunner.fwd.resize(1024);
    m_intakeRunner.bwd.resize(1024);
    m_intakeRunner.wallLoss.setCutoff(5000.0, m_fs);
    m_intakeRunner.valveReflection = 0.7f;
    m_intakePulses.clear();
    m_intakePulses.reserve(32);
    m_combustion.clear();
    m_combustion.reserve(32);
    for (Bank& b : m_banks) b.pulses.reserve(32);
    m_exhaustRadPrev = m_intakeRadPrev = 0.0f;

    // Air box: Helmholtz resonator at ambient temperature.
    {
        const double area = kPi * cfg.snorkelRadius * cfg.snorkelRadius;
        const double fh = speedOfSound(293.15) / (2.0 * kPi)
                        * std::sqrt(area / (std::max(cfg.airboxVolume, 1e-4) * std::max(cfg.snorkelLength, 0.02)));
        m_diag.airboxHz = fh;
        m_airbox.bandpass(fh, 4.0, m_fs);
    }
    for (std::size_t i = 0; i < m_modes.size(); ++i) {
        m_modes[i].bandpass(kModeHz[i], kModeQ[i], m_fs);
        m_modeGain[i] = kModeGain[i];
    }
    m_diag.shakingForceN = 0.0;

    // Silencer: 4th order (24 dB/oct). A reactive/absorptive silencer's
    // transmission loss climbs steeply above its corner; with the +6 dB/oct
    // of the dU/dt radiation on top, a gentler filter leaves a bright peak
    // AT the corner - a whistle, not an exhaust note.
    m_muffler1.lowpass(cfg.mufflerCutoffHz, 0.54, m_fs);
    m_muffler2.lowpass(cfg.mufflerCutoffHz, 1.31, m_fs);   // Butterworth pair
    m_throttleHissBp.bandpass(2800.0, 1.2, m_fs);
    m_mechBp.bandpass(5200.0, 2.0, m_fs);
    m_intakeLp.setCutoff(2500.0, m_fs);
    m_turboWhooshBp.bandpass(1800.0, 0.8, m_fs);
    m_bovBp.bandpass(3200.0, 0.9, m_fs);
    m_bovLowBp.bandpass(900.0, 1.0, m_fs);
    m_rollLp.lowpass(700.0, 0.7, m_fs);

    // Tread-block stick-slip squeal sits in the high hundreds of Hz; the
    // four tyres are not identical, so neither are their squeals.
    const double squeal[4] = {820.0, 870.0, 760.0, 800.0};
    for (int i = 0; i < 4; ++i) {
        m_squealHz[static_cast<std::size_t>(i)] = squeal[i];
        m_squealBp[static_cast<std::size_t>(i)].bandpass(squeal[i], 18.0, m_fs);
    }

    // EngineSim's engine sound, if this vehicle asks for it.
    m_es = EngineSimSynth(m_fs);
    m_useEs = false;
    if (engine.engineSim.cylinders > 0) {
        m_es.configure(engine.engineSim);
        m_es.applyProfile(cfg.engineSim.profile);
        m_useEs = m_es.configured();
    }
    m_esPos = EngineSimSynth::kBlock;
    m_propagationSide.resize(1u << 16);
    m_cabinExhaustSide.setCutoff(320.0, m_fs);

    m_propagation.resize(1u << 16);   // 1.36 s at 48 kHz = 470 m of air
    m_cabinExhaust.setCutoff(320.0, m_fs);
    m_cabinEngine.setCutoff(2400.0, m_fs);
    m_cabinTyres.setCutoff(500.0, m_fs);
    m_cabinWind.setCutoff(1200.0, m_fs);
    m_cabinStructure.setCutoff(900.0, m_fs);

    m_crank = 0.0;
    m_turboPhase = m_turboPhase2 = m_gearPhase = 0.0;
    m_haveState = false;
    m_dcX = m_dcY = 0.0f;
}

void SoundSynth::setState(const SoundState& s) {
    m_target = s;
    if (!m_haveState) {
        m_cur = s;
        m_propDelay = m_lastPropDelay = s.distance / kSpeedOfSoundAir * m_fs;
        m_pan = s.pan;
        m_haveState = true;
    }
}

float SoundSynth::noise() {
    // xorshift32 - fast, deterministic, good enough for audio noise
    m_rng ^= m_rng << 13;
    m_rng ^= m_rng >> 17;
    m_rng ^= m_rng << 5;
    return static_cast<float>(m_rng) * (2.0f / 4294967295.0f) - 1.0f;
}

void SoundSynth::firePulse(int bank, double amplitude, double riseSamples, double decaySamples) {
    if (std::abs(amplitude) < 1e-6) return;
    Bank& b = m_banks[static_cast<std::size_t>(bank)];
    if (b.pulses.size() > 24) return;   // cannot happen at sane rpm; bound it anyway
    b.pulses.push_back(Pulse{0.0, riseSamples, decaySamples, static_cast<float>(amplitude)});
}

float SoundSynth::pulseValue(const Pulse& p) {
    if (p.t < p.rise) {
        return p.amp * static_cast<float>(0.5 * (1.0 - std::cos(kPi * p.t / p.rise)));
    }
    return p.amp * static_cast<float>(std::exp(-(p.t - p.rise) / p.decay));
}

void SoundSynth::smoothTowardTarget() {
    // ~12 ms glide: fast enough to follow a blip, slow enough that 60 Hz
    // state updates do not zipper.
    const double a = 1.0 - std::exp(-1.0 / (0.012 * m_fs));
    SoundState& c = m_cur;
    const SoundState& t = m_target;
    auto glide = [a](double& v, double target) { v += (target - v) * a; };

    glide(c.crankOmega, t.crankOmega);
    glide(c.throttle, t.throttle);
    glide(c.blowdownPressurePa, t.blowdownPressurePa);
    glide(c.exhaustManifoldPa, t.exhaustManifoldPa);
    glide(c.airMassPerCylinderKg, t.airMassPerCylinderKg);
    glide(c.intakePressurePa, t.intakePressurePa);
    glide(c.chargeTemperatureK, t.chargeTemperatureK);
    glide(c.engineTorqueNm, t.engineTorqueNm);
    glide(c.indicatedTorqueNm, t.indicatedTorqueNm);
    glide(c.boostPa, t.boostPa);
    glide(c.speed, t.speed);
    for (std::size_t i = 0; i < 4; ++i) {
        glide(c.wheels[i].slipPowerW, t.wheels[i].slipPowerW);
        glide(c.wheels[i].frictionUsage, t.wheels[i].frictionUsage);
    }
    glide(c.distance, t.distance);
    glide(c.pan, t.pan);

    // The exhaust gas does not change temperature in 12 ms - the gas in
    // the pipe and the pipe wall have thermal inertia. A slower glide keeps
    // a fuel cut from snapping the pipe resonance down in one frame.
    const double aThermal = 1.0 - std::exp(-1.0 / (0.25 * m_fs));
    c.blowdownTemperatureK += (t.blowdownTemperatureK - c.blowdownTemperatureK) * aThermal;

    // Discrete / slow-changing values copy straight across.
    c.engineActive = t.engineActive;
    c.referenceAirMassKg = t.referenceAirMassKg;
    c.ambientPressurePa = t.ambientPressurePa;
    c.ambientTemperatureK = t.ambientTemperatureK;
    c.peakTorqueNm = t.peakTorqueNm;
    c.redlineOmega = t.redlineOmega;
    c.numCylinders = t.numCylinders;
    c.turbo = t.turbo;
    c.maxBoostPa = t.maxBoostPa;
    c.compressorEfficiency = t.compressorEfficiency;
    c.clutchLocked = t.clutchLocked;
    c.inGear = t.inGear;
    c.interior = t.interior;
    c.engineBehindListener = t.engineBehindListener;
}

//===========================================================================
//  Render
//===========================================================================

void SoundSynth::render(float* out, int frames) {
    if (!m_haveState || m_mix.muted) {
        std::fill(out, out + 2 * frames, 0.0f);
        if (m_haveState) {
            // keep the clocks moving so un-muting does not jump
            m_crank = std::fmod(m_crank + m_target.crankOmega * frames / m_fs, kFourPi);
        }
        return;
    }

    const SoundState& T = m_target;

    //-----------------------------------------------------------------------
    //  Per-block coefficients - things that change over tens of ms, set once
    //  per buffer from the target state rather than every sample.
    //-----------------------------------------------------------------------
    const double gasTemp = std::clamp(m_cur.blowdownTemperatureK * m_cfg.exhaustGasTemperatureFraction,
                                      320.0, 1400.0);
    const double cExhaust = speedOfSound(gasTemp);
    const double pipeLen = std::clamp(m_cfg.exhaustPipeLength, 0.2, 6.0);
    const double pipeDelay = std::clamp(pipeLen / cExhaust * m_fs, 2.0, 8000.0);
    const double boostRatio = T.turbo ? std::clamp(T.boostPa / std::max(T.maxBoostPa, 1.0), 0.0, 1.0) : 0.0;
    for (Bank& b : m_banks) {
        b.pipe.delay = pipeDelay;
        b.pipe.openEnd.setCutoff(cExhaust / (2.0 * kPi * kTailpipeRadius), m_fs);
        // The turbine wheel sits in the exhaust stream and takes energy out
        // of each pulse - and smooths it - more the harder it is working.
        // AUTHORED shape (no turbine map), scaled by boost.
        b.turbine.setCutoff(18000.0 - 14000.0 * boostRatio, m_fs);
    }
    const double cCharge = speedOfSound(T.chargeTemperatureK);
    m_intakeRunner.delay = std::clamp(0.35 / cCharge * m_fs, 2.0, 1000.0);
    m_intakeRunner.openEnd.setCutoff(cCharge / (2.0 * kPi * 0.035), m_fs);

    // Turbo shaft speed from the compressor work balance - see header.
    double turboShaftOmega = 0.0;
    if (T.turbo && T.boostPa > 0.0) {
        const double pr = (T.ambientPressurePa + T.boostPa) / T.ambientPressurePa;
        const double specificWork = kCp * T.ambientTemperatureK *
            (std::pow(pr, (kGamma - 1.0) / kGamma) - 1.0) / std::max(T.compressorEfficiency, 0.3);
        const double tipSpeed = std::sqrt(specificWork / kCompressorWorkCoefficient);
        turboShaftOmega = tipSpeed / std::max(m_cfg.turboTipRadius, 0.005);
    }
    const double bladePassHz = turboShaftOmega / (2.0 * kPi) * std::max(m_cfg.turboBlades, 1);

    const double gearHz = T.crankOmega / (2.0 * kPi) * std::max(m_cfg.gearTeeth, 1);

    // Wind: Strouhal frequencies for a large (body, D ~ 0.25 m) and a small
    // (mirror/trim, D ~ 0.015 m) length scale.
    const double v = std::max(T.speed, 0.0);
    m_windLowBp.bandpass(std::clamp(kStrouhal * v / 0.25, 30.0, 900.0), 0.7, m_fs);
    m_windHighBp.bandpass(std::clamp(kStrouhal * v / 0.015, 300.0, 7000.0), 0.6, m_fs);

    // Air absorption: high frequencies die with distance (ISO 9613-like in
    // shape; the corner frequency is AUTHORED).
    const double absorptionHz = 20000.0 / (1.0 + T.distance / 80.0);
    m_airAbsorptionL.setCutoff(absorptionHz, m_fs);
    m_airAbsorptionR.setCutoff(absorptionHz, m_fs);

    const double blowdownLenRad = std::clamp(m_cfg.blowdownCrankDeg, 10.0, 180.0) * kPi / 180.0;
    const double targetPropDelay = T.distance / kSpeedOfSoundAir * m_fs;
    const double propDelayAlpha = 1.0 - std::exp(-1.0 / (0.03 * m_fs));

    // dU/dt as a first difference, scaled to unity gain at kRadiationRefHz.
    const float radiationScale = static_cast<float>(m_fs / (2.0 * kPi * kRadiationRefHz));

    const float mE = m_mix.exhaust, mI = m_mix.intake, mT = m_mix.turbo,
                mTr = m_mix.transmission, mTy = m_mix.tyres, mW = m_mix.wind,
                mS = m_mix.structure;
    float peak = 0.0f;
    const double delayAtStart = m_propDelay;

    for (int f = 0; f < frames; ++f) {
        smoothTowardTarget();
        const SoundState& s = m_cur;

        float exhaust = 0.0f, intake = 0.0f, mech = 0.0f, turbo = 0.0f, trans = 0.0f, structure = 0.0f;

        //-------------------------------------------------------------------
        //  ENGINE, as EngineSim hears it (audio/EngineSimSynth.h). It runs in
        //  its own 128-frame blocks, each started from the newest state - the
        //  AudioWorklet's quantum, and what its per-block arithmetic assumes.
        //  It does its own smoothing of speed, manifold pressure and throttle,
        //  and it runs from a standstill: the starter is one of its sounds.
        //-------------------------------------------------------------------
        float esSide = 0.0f;
        const bool esActive = m_useEs && s.engineActive;
        if (esActive) {
            if (m_esPos >= EngineSimSynth::kBlock) {
                EngineSimSynth::State st;
                st.rpm = T.crankOmega * 60.0 / (2.0 * kPi);
                st.map = T.manifoldRatio;
                st.throttle = T.pedal;
                st.combusting = T.combusting;
                st.fuelCut = T.fuelCut;
                st.starter = T.starter;
                st.ignition = true;
                st.redline = T.redlineOmega * 60.0 / (2.0 * kPi);
                st.idle = T.idleOmega * 60.0 / (2.0 * kPi);
                st.indicated = T.grossTorqueNm;
                st.friction = T.frictionTorqueNm;
                st.pumping = T.pumpingTorqueNm;
                // speed / gear / load stay 0: its road noise and driveline
                // whine are replaced by the tyre, wind and gear sources below.
                m_es.setState(st);
                m_es.process(m_esL.data(), m_esR.data(), EngineSimSynth::kBlock);
                m_esPos = 0;
            }
            const float l = m_esL[static_cast<std::size_t>(m_esPos)];
            const float r = m_esR[static_cast<std::size_t>(m_esPos)];
            ++m_esPos;
            exhaust = 0.5f * (l + r) * kEngineSimGain;
            esSide  = 0.5f * (l - r) * kEngineSimGain;   // the two pipes, left against right
        }

        //-------------------------------------------------------------------
        //  ENGINE: the crank turns; cylinders fire when it passes them.
        //-------------------------------------------------------------------
        if (s.engineActive && s.crankOmega > 1.0) {
            const double omega = s.crankOmega;
            const double prev = m_crank;
            m_crank += omega / m_fs;
            const bool wrapped = m_crank >= kFourPi;
            if (wrapped) m_crank -= kFourPi;
            // Mass flow through the engine, relative to a full charge at
            // redline - the level of the flow-noise sources.
            const double flow = s.airMassPerCylinderKg / std::max(s.referenceAirMassKg, 1e-9)
                              * omega / std::max(s.redlineOmega, 1.0);

            if (!esActive) {
            for (std::size_t k = 0; k < m_fireAngles.size(); ++k) {
                // Power stroke: blowdown when the exhaust valve opens.
                if (crossed(prev, m_crank, wrapped, m_fireAngles[k])) {
                    // SIGNED. Under fuel cut the cylinder opens BELOW
                    // the manifold and exhaust gas flows back in - a weak
                    // rarefaction pulse, which is the overrun sound. Clamping
                    // at zero made a lifted engine silent.
                    double dp = s.blowdownPressurePa - s.exhaustManifoldPa;
                    // Forward blowdown is choked flow (cylinder at several
                    // times manifold pressure); backflow into a cylinder
                    // below the manifold is subsonic and weaker. AUTHORED
                    // factor for the difference.
                    if (dp < 0.0) dp *= 0.5;
                    // Cyclic variability: sum of three uniforms ~ Gaussian.
                    const double g = (noise() + noise() + noise()) / 1.7320508;
                    const double var = m_cylGain[k] * (1.0 + m_cfg.cyclicVariation * g);
                    // Charge released ~ dp; released over a fixed crank
                    // angle, so the flow RATE also scales with omega.
                    const double amp = dp / kBlowdownReference * var * omega / kFlowReferenceOmega;
                    // Blowdown: the valve snaps open (fast rise over ~12 % of
                    // the blowdown angle), then the cylinder empties (decay).
                    const double len = std::max(blowdownLenRad / omega * m_fs, 8.0);
                    firePulse(m_fireBank[k], amp, std::max(0.12 * len, 2.0), 0.35 * len);

                    // Combustion impact on the structure: the indicated work
                    // this cylinder just delivered, over the ~40 deg the
                    // pressure takes to rise and fall.
                    if (s.indicatedTorqueNm > 0.0 && m_combustion.size() < 24) {
                        const double work = s.indicatedTorqueNm * kFourPi / std::max(m_numCylinders, 1);
                        const double ca = 20.0 * kPi / 180.0 / omega * m_fs;
                        m_combustion.push_back(Pulse{0.0, std::max(ca, 2.0), std::max(ca * 1.2, 2.0),
                                                     static_cast<float>(work / kCombustionReferenceJ * var)});
                    }
                }
                // Intake stroke of the same cylinder, 360 deg later.
                const double intakeAngle = std::fmod(m_fireAngles[k] + 2.0 * kPi, kFourPi);
                if (crossed(prev, m_crank, wrapped, intakeAngle) && m_intakePulses.size() < 24) {
                    const double amp = s.airMassPerCylinderKg / std::max(s.referenceAirMassKg, 1e-9)
                                     * m_cylGain[k] * omega / kFlowReferenceOmega;
                    const double len = std::max(kPi / omega * m_fs, 8.0);   // the 180 deg intake stroke
                    m_intakePulses.push_back(Pulse{0.0, 0.3 * len, 0.35 * len, static_cast<float>(amp)});
                }
            }

            // Pulses -> (turbine) -> pipe, per bank.
            for (Bank& b : m_banks) {
                float q = 0.0f;
                for (Pulse& p : b.pulses) {
                    q += pulseValue(p);
                    p.t += 1.0;
                }
                b.pulses.erase(std::remove_if(b.pulses.begin(), b.pulses.end(), pulseDone),
                               b.pulses.end());
                if (s.turbo) q = b.turbine.lp(q * static_cast<float>(1.0 - 0.45 * boostRatio));
                exhaust += b.pipe.process(q);
            }
            {
                // Far field of the tailpipe: p ~ dU/dt.
                const float u = m_muffler2.process(m_muffler1.process(exhaust));
                exhaust = (u - m_exhaustRadPrev) * radiationScale;
                m_exhaustRadPrev = u;
            }

            float qi = 0.0f;
            for (Pulse& p : m_intakePulses) {
                qi += pulseValue(p);
                p.t += 1.0;
            }
            m_intakePulses.erase(std::remove_if(m_intakePulses.begin(), m_intakePulses.end(), pulseDone),
                                 m_intakePulses.end());
            {
                // Runner -> air box (Helmholtz) -> snorkel mouth, radiating
                // dU/dt. The resonator is what makes it a roar rather than a
                // rustle; a turbo compressor upstream damps it with boost.
                const float runnerU = m_intakeRunner.process(qi);
                const float u = m_intakeLp.lp(4.0f * m_airbox.process(runnerU) + 0.35f * runnerU)
                              * static_cast<float>(1.0 - 0.6 * boostRatio);
                intake = (u - m_intakeRadPrev) * radiationScale * static_cast<float>(0.4 + 0.6 * s.throttle);
                m_intakeRadPrev = u;
            }

            //---------------------------------------------------------------
            //  STRUCTURE-BORNE: combustion impacts ringing the block/body
            //  modes, plus the second-order reciprocating shake.
            //---------------------------------------------------------------
            float impact = 0.0f;
            for (Pulse& p : m_combustion) {
                impact += pulseValue(p);
                p.t += 1.0;
            }
            m_combustion.erase(std::remove_if(m_combustion.begin(), m_combustion.end(), pulseDone),
                               m_combustion.end());
            for (std::size_t i = 0; i < m_modes.size(); ++i) {
                structure += m_modeGain[i] * m_modes[i].process(impact);
            }
            structure *= 6.0f;
            if (m_secondOrderFactor > 0.0) {
                const double r = 0.5 * m_geometry.stroke;
                const double f2 = m_cfg.reciprocatingMass * r * omega * omega
                                * (r / std::max(m_geometry.rodLength, r * 1.5)) * m_secondOrderFactor;
                const double a2 = std::min(f2 / kShakingReferenceN, 3.0);
                const double th = 2.0 * m_crank - m_secondOrderPhase;
                structure += static_cast<float>(0.05 * a2 * (std::sin(th) + 0.25 * std::sin(2.0 * th)));
                m_diag.shakingForceN = f2;
            }

            // Throttle hiss: jet across a part-open plate. Lighthill: power
            // ~ U^8, so amplitude ~ U^4.
            const double upstream = s.ambientPressurePa + (s.turbo ? s.boostPa : 0.0);
            const double dpThrottle = std::max(0.0, upstream - s.intakePressurePa);
            const double uJet = std::sqrt(2.0 * dpThrottle / kRhoAir);
            const double hiss = std::min(1.0, std::pow(uJet / 300.0, 4.0)) * std::min(1.0, flow * 3.0);
            intake += m_throttleHissBp.process(noise()) * static_cast<float>(0.06 * hiss);

            // Valvetrain / mechanical: broadband, rising with speed squared.
            const double rn = omega / std::max(s.redlineOmega, 1.0);
            mech = m_mechBp.process(noise()) * static_cast<float>(0.012 * rn * rn);
            }   // !esActive

            //---------------------------------------------------------------
            //  TURBO
            //---------------------------------------------------------------
            if (s.turbo) {
                if (bladePassHz > 20.0 && bladePassHz < 0.45 * m_fs) {
                    m_turboPhase += 2.0 * kPi * bladePassHz / m_fs;
                    m_turboPhase2 += 2.0 * kPi * bladePassHz * 1.006 / m_fs;   // twin, not identical
                    if (m_turboPhase > 2.0 * kPi) m_turboPhase -= 2.0 * kPi;
                    if (m_turboPhase2 > 2.0 * kPi) m_turboPhase2 -= 2.0 * kPi;
                    const double pr = (s.ambientPressurePa + s.boostPa) / s.ambientPressurePa;
                    const double level = std::min(1.5, flow * 2.0) * (pr - 1.0);
                    turbo += static_cast<float>(0.035 * level *
                             (std::sin(m_turboPhase) + 0.7 * std::sin(m_turboPhase2)));
                    turbo += m_turboWhooshBp.process(noise()) * static_cast<float>(0.04 * level);
                }
                // Blow-off valve: opens on a lift with boost still in the
                // plenum; the vented jet is loud (U^8) and short-lived, since
                // the engine model already dumps boost fast on a lift.
                if (s.throttle < 0.15 && s.boostPa > 3000.0) {
                    const double u = std::sqrt(2.0 * s.boostPa / kRhoAir);
                    const double uMax = std::sqrt(2.0 * std::max(s.maxBoostPa, 1.0) / kRhoAir);
                    const double bov = std::pow(u / uMax, 4.0);
                    const float n = noise();
                    turbo += static_cast<float>(0.5 * bov) * (m_bovBp.process(n) + 0.6f * m_bovLowBp.process(n));
                }
            }

            //---------------------------------------------------------------
            //  GEAR WHINE - only while the gearbox is carrying torque.
            //---------------------------------------------------------------
            if (s.clutchLocked && s.inGear && gearHz < 0.45 * m_fs) {
                m_gearPhase += 2.0 * kPi * gearHz / m_fs;
                if (m_gearPhase > 2.0 * kPi) m_gearPhase -= 2.0 * kPi;
                const double load = std::abs(s.engineTorqueNm) / std::max(s.peakTorqueNm, 1.0);
                trans = static_cast<float>(0.03 * (0.2 + 0.8 * std::min(load, 1.2)) * std::sin(m_gearPhase));
            }
        }

        //-------------------------------------------------------------------
        //  TYRES
        //-------------------------------------------------------------------
        const double sv = s.speed;
        float tyres = m_rollLp.process(noise()) * static_cast<float>(0.06 * std::min(std::pow(sv / 30.0, 1.75), 4.0));
        for (std::size_t i = 0; i < 4; ++i) {
            // Squeal is stick-slip at the edge of grip: gate on how close to
            // the friction limit the tyre is, scale with sliding power.
            const double near = smoothstep(0.80, 1.0, s.wheels[i].frictionUsage);
            const double a = near * std::min(std::sqrt(s.wheels[i].slipPowerW / 20000.0), 1.5);
            if (a > 1e-4) tyres += m_squealBp[i].process(noise()) * static_cast<float>(1.2 * a);
        }

        //-------------------------------------------------------------------
        //  WIND - Curle: amplitude ~ U^3.
        //-------------------------------------------------------------------
        const double windAmp = std::min(std::pow(sv / 50.0, 3.0), 3.0);
        const float wn = noise();
        const float wind = static_cast<float>(windAmp) *
                           (0.35f * m_windLowBp.process(wn) + 0.2f * m_windHighBp.process(wn));

        //-------------------------------------------------------------------
        //  LISTENER
        //-------------------------------------------------------------------
        float left = 0.0f, right = 0.0f;
        if (!esActive) exhaust *= kExhaustGain;
        intake *= kIntakeGain;
        mech *= kIntakeGain;
        turbo *= kTurboGain;
        trans *= kGearGain;
        tyres *= kTyreGain;
        structure *= kStructureGain;
        const float windOut = wind * kWindGain;
        // Outside, the structure radiates through the body panels - weaker
        // than the tailpipe; inside it is most of what you hear (below).
        const float carSignal = exhaust * mE + (intake + mech) * mI + structure * mS * 0.5f
                              + turbo * mT + trans * mTr + tyres * mTy + windOut * mW * 0.4f;
        m_propagation.push(carSignal);
        m_propagationSide.push(esSide * mE);
        m_propDelay += (targetPropDelay - m_propDelay) * propDelayAlpha;

        if (s.interior) {
            // Through the body shell: mass-law transmission loss rises with
            // frequency, so what reaches the cabin is the low end.
            const float engineGain = s.engineBehindListener ? 1.0f : 0.6f;
            const float mono =
                  m_cabinExhaust.lp(exhaust) * mE * 0.9f
                // EngineSim's signal is the whole engine - intake, valve
                // gear, starter - not only the tailpipe, so half of it also
                // takes the engine-bay path into the cabin. AUTHORED split.
                + m_cabinEngine.lp((intake + mech) * mI + turbo * mT
                                   + (esActive ? exhaust * mE * 0.5f : 0.0f)) * engineGain
                + m_cabinStructure.lp(structure) * mS * 1.3f
                + trans * mTr * 0.8f
                + m_cabinTyres.lp(tyres) * mTy * 0.7f
                + m_cabinWind.lp(windOut) * mW;
            left = right = 0.7f * mono;   // cabin level: ~3 dB of headroom below the soft clip
            const float side = 0.7f * m_cabinExhaustSide.lp(esSide) * mE * 0.9f;
            left += side;
            right -= side;
        } else {
            // Exterior: what was emitted distance/c ago, spread over
            // distance. The delay's own rate of change is the Doppler shift.
            const float heard = m_propagation.read(m_propDelay);
            const float gain = static_cast<float>(4.0 / std::max(s.distance, 4.0));
            const double pan = std::clamp(s.pan, -1.0, 1.0);
            const double theta = (pan + 1.0) * kPi / 4.0;
            left  = m_airAbsorptionL.lp(heard) * gain * static_cast<float>(std::cos(theta)) * 1.41f;
            right = m_airAbsorptionR.lp(heard) * gain * static_cast<float>(std::sin(theta)) * 1.41f;
            // The left/right difference of a dual exhaust, heard from where
            // the two pipes can be told apart; it fades as the car moves off
            // to one side of the view.
            const float side = m_propagationSide.read(m_propDelay) * gain
                             * static_cast<float>(1.0 - std::abs(pan));
            left += side;
            right -= side;
        }

        // Master, DC blocker, soft clip.
        left *= m_mix.master;
        right *= m_mix.master;
        const float mid = 0.5f * (left + right);
        const float dc = mid - m_dcX + 0.995f * m_dcY;
        m_dcX = mid; m_dcY = dc;
        const float corr = dc - mid;
        left = std::tanh(left + corr);
        right = std::tanh(right + corr);

        out[2 * f] = left;
        out[2 * f + 1] = right;
        peak = std::max(peak, std::max(std::abs(left), std::abs(right)));
    }

    //-----------------------------------------------------------------------
    //  Diagnostics
    //-----------------------------------------------------------------------
    m_diag.firingFrequencyHz = m_target.engineActive
        ? m_target.crankOmega / (4.0 * kPi) * static_cast<double>(m_numCylinders) : 0.0;
    m_diag.pipeFundamentalHz = cExhaust / (4.0 * pipeLen);
    m_diag.exhaustGasTempK = gasTemp;
    m_diag.blowdownDeltaBar = std::max(0.0, T.blowdownPressurePa - T.exhaustManifoldPa) / 1e5;
    m_diag.turboShaftRpm = turboShaftOmega * 60.0 / (2.0 * kPi);
    m_diag.turboBladePassHz = bladePassHz;
    m_diag.gearWhineHz = gearHz;
    if (frames > 0 && !T.interior) {
        m_diag.dopplerRatio = 1.0 - (m_propDelay - delayAtStart) / frames;
    } else {
        m_diag.dopplerRatio = 1.0;
    }
    m_diag.peakLevel = peak;
}

//===========================================================================
//  WAV output
//===========================================================================

bool writeWav(const std::string& path, const std::vector<float>& samples, int sampleRate) {
    std::ofstream f(path, std::ios::binary);
    if (!f) return false;
    const std::uint32_t dataBytes = static_cast<std::uint32_t>(samples.size() * 2);
    auto u32 = [&f](std::uint32_t v) { f.write(reinterpret_cast<const char*>(&v), 4); };
    auto u16 = [&f](std::uint16_t v) { f.write(reinterpret_cast<const char*>(&v), 2); };
    f.write("RIFF", 4); u32(36 + dataBytes); f.write("WAVE", 4);
    f.write("fmt ", 4); u32(16); u16(1); u16(2);
    u32(static_cast<std::uint32_t>(sampleRate));
    u32(static_cast<std::uint32_t>(sampleRate) * 4); u16(4); u16(16);
    f.write("data", 4); u32(dataBytes);
    for (float x : samples) {
        const auto v = static_cast<std::int16_t>(std::lround(std::clamp(x, -1.0f, 1.0f) * 32767.0f));
        f.write(reinterpret_cast<const char*>(&v), 2);
    }
    return static_cast<bool>(f);
}

} // namespace vsim
