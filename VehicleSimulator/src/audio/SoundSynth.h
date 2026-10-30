#pragma once
//===========================================================================
//  SoundSynth.h  --  vehicle sound, synthesised from the physics.
//
//  No recordings. Every sound here is generated, sample by sample, from the
//  state the simulation already computes (audio/SoundState.h):
//
//  ENGINE / EXHAUST
//    * A crank-angle clock integrates the engine's own omega. Each cylinder
//      fires when the crank passes its firing angle (VehicleSoundConfig) -
//      so the firing frequency is N/60 x n/2 because the crank turns, not
//      because a pitch was set, and an uneven firing order is uneven.
//    * Each firing releases a BLOWDOWN pulse whose strength is the pressure
//      difference across the opening exhaust valve: p_4 from the engine's
//      own Otto cycle (Engine::TorqueBreakdown) minus the manifold. Its
//      duration is a fixed crank angle, so it is shorter - brighter - at
//      high rpm. Fuel cut (lift-off, rev limiter) leaves p_4 near ambient,
//      and the engine goes quiet on its own.
//    * The pulse is ASYMMETRIC: the valve snaps open on a cylinder at
//      several bar, the flow jumps (choked), then decays as the cylinder
//      empties. The sharp front is what carries the harmonics - the bark.
//    * Each cylinder has its own fixed strength (charge maldistribution)
//      as well as random cycle-to-cycle variation, so the note has energy
//      at HALF orders too - the lumpy undertone every real engine has.
//    * The pulse travels down an exhaust pipe modelled as a digital
//      waveguide: a delay line each way, of length L / c with
//      c = sqrt(gamma R T) at the exhaust gas temperature, a valve-end and
//      an open-end reflection. So the pipe resonances f = (2k-1) c / 4L
//      shift with load and temperature on their own.
//    * RADIATION: what leaves the tailpipe is a VOLUME VELOCITY, and a
//      small source radiates far-field pressure p ~ rho/(4 pi r) dU/dt.
//      At the open end the pressure wave reflects inverted, so pressure
//      nearly cancels there while velocity DOUBLES - U_out = (1 - R) p+.
//      The radiated sound is the derivative of that. (An earlier version
//      took the transmitted PRESSURE, (1 + R) p+, and so high-passed the
//      exhaust twice - which is why it sounded like air.)
//
//  STRUCTURE-BORNE (the "vibration")
//    * Each combustion event hits the crank, block and mounts with a force
//      proportional to the indicated work of that cylinder; the block and
//      body panels ring at their own modes (a modal resonator bank). This
//      is the hard, resonant thump under the exhaust note, and most of what
//      reaches the cabin.
//    * The second-order shaking force of the reciprocating masses,
//          F2 = m_rec r w^2 (r/l) |sum_k e_bank(k) exp(2 i phi_k)|,
//      computed from the actual stroke, rod length, firing angles and bank
//      geometry - so an inline-4 and a flat-plane V8 shake at twice crank
//      frequency with a force growing as rpm^2, and a cross-plane V8 comes
//      out balanced, as it is.
//    * Muffler: a low-pass. Turbine: absorbs and smooths pulse energy in
//      proportion to boost (AUTHORED - no turbine map).
//
//  INTAKE
//    * Induction pulses on the intake stroke of each cylinder, strength =
//      trapped charge / a full charge, through a short intake runner
//      waveguide at the charge temperature, into the AIR BOX - a Helmholtz
//      resonator, f = c/(2 pi) sqrt(A / (V L)) - which is the induction
//      roar. A turbo's compressor sits in the way and damps it with boost.
//    * Throttle hiss: flow noise across the plate. Jet noise power goes as
//      U^8 (Lighthill); U from the pressure drop across the throttle.
//
//  TURBO
//    * Compressor whine at blade-pass frequency. Shaft speed from Euler's
//      turbomachinery work equation: the specific work the compressor does,
//      c_p T_1 (PR^((g-1)/g) - 1) / eta_C (Heywood Eq 6.42 per unit mass),
//      equals psi U_tip^2, so omega = U_tip / r_tip. For the F40's 1.1 bar
//      this lands around 130 000 rpm - the right order for a small turbo.
//    * Blow-off valve: on a lift with boost in the plenum, the vented jet's
//      noise, again U^8.
//
//  DRIVELINE, TYRES, AIR
//    * Gear whine at input-shaft frequency x tooth count, level with torque.
//    * Tyre squeal from frictional power at the contact patch (|F| x sliding
//      speed) once the tyre is near the friction limit; rolling noise.
//    * Wind: Curle's dipole law - intensity goes as U^6, so amplitude as
//      U^3 - with the spectrum centred on a Strouhal frequency St U / D.
//
//  LISTENER
//    * Exterior cameras hear the car through a propagation delay line of
//      distance / speed of sound. The delay changes as the distance does,
//      so Doppler shift comes out of the geometry, not a formula - a TV
//      camera hears the pass-by drop in pitch. 1/r spreading, and a
//      distance-dependent air-absorption low-pass.
//    * The cockpit hears it through the body: exhaust and tyres low-passed
//      (mass-law transmission loss), the engine loud if it sits behind you.
//
//  Loudness is NOT absolute (there is no calibrated dB SPL here): the
//  RELATIONSHIPS are what is derived - how each source scales with rpm,
//  load, boost, slip and speed. The per-source mix levels are exposed.
//
//  ENGINESIM'S ENGINE SOUND (optional, per vehicle)
//    * A vehicle with a `sound: engine_sim:` block is heard through
//      EngineSim's own synthesiser instead (audio/EngineSimSynth.h): the
//      four ENGINE / EXHAUST, STRUCTURE-BORNE and INTAKE groups above are
//      then switched off and its output takes their place on the exhaust
//      bus. TURBO, DRIVELINE, TYRES, AIR and the LISTENER below stay as they
//      are, so the car still passes a trackside camera with a Doppler shift
//      and still sounds different from the cockpit. Its two exhaust pipes
//      stay left and right: the stereo difference travels the same path as
//      the rest.
//
//  Deterministic and device-free: render() just fills a buffer. The same
//  class drives the live audio device (audio/AudioOutput.h) and the
//  offline WAV render (--audio-out) used to test it.
//===========================================================================

#include <cstdint>
#include <string>
#include <vector>

#include "audio/EngineSimSynth.h"
#include "audio/SoundState.h"
#include "config/VehicleParameters.h"

namespace vsim {

/// The engine hardware the synthesiser needs beyond VehicleSoundConfig:
/// cylinder count for the firing events, stroke and rod length for the
/// reciprocating shaking force. cylinders == 0 means no detailed engine.
struct EngineGeometry {
    int    cylinders = 4;
    double stroke = 0.086;     ///< [m]
    double rodLength = 0.144;  ///< [m]
    /// Set (cylinders > 0) when the vehicle asks for EngineSim's engine
    /// sound: the engine, exhaust and intake are then EngineSimSynth's.
    EngineSimSoundSpec engineSim;
};

class SoundSynth {
public:
    explicit SoundSynth(double sampleRate = 48000.0);

    /// Per-vehicle hardware: firing order, pipe length, muffler, turbo
    /// wheel... Resets all synthesis state. Call on vehicle load.
    void configure(const VehicleSoundConfig& cfg, const EngineGeometry& engine);

    /// New target state. Continuous quantities glide toward it over a few
    /// milliseconds, so a 60 Hz update never steps audibly.
    void setState(const SoundState& s);

    /// Fill `frames` interleaved stereo float samples.
    void render(float* interleavedStereo, int frames);

    struct Mix {
        float master = 0.8f;
        float exhaust = 1.0f;
        float intake = 0.7f;
        float structure = 0.8f;   ///< block/body vibration
        float turbo = 0.6f;
        float transmission = 0.35f;
        float tyres = 0.8f;
        float wind = 0.6f;
        bool  muted = false;
    };
    Mix& mix() { return m_mix; }

    /// What the synthesiser derived, for the UI (read on the main thread
    /// under the same lock as setState()).
    struct Diagnostics {
        double firingFrequencyHz = 0.0;
        double pipeFundamentalHz = 0.0;   ///< c / 4L at the current gas temperature
        double exhaustGasTempK = 0.0;
        double blowdownDeltaBar = 0.0;    ///< across the opening valve
        double airboxHz = 0.0;            ///< Helmholtz resonance of the intake air box
        double shakingForceN = 0.0;       ///< second-order reciprocating force amplitude
        double turboShaftRpm = 0.0;
        double turboBladePassHz = 0.0;
        double gearWhineHz = 0.0;
        double dopplerRatio = 1.0;        ///< emitted / received time rate
        double peakLevel = 0.0;           ///< output peak over the last buffer
    };
    const Diagnostics& diagnostics() const { return m_diag; }

    /// True when the engine is being synthesised by EngineSimSynth.
    bool usesEngineSim() const { return m_useEs; }
    /// Its dials (EngineSim README 6.6). Takes effect on the next block.
    const EngineSimSynth::Profile& engineSimProfile() const { return m_es.profile(); }
    void setEngineSimProfile(const EngineSimSynth::Profile& p) { m_es.applyProfile(p); }

    double sampleRate() const { return m_fs; }

    //-----------------------------------------------------------------------
    //  Building blocks - public so they can be unit-tested; nothing outside
    //  the synthesiser needs them.
    //-----------------------------------------------------------------------
    struct OnePole {
        float a = 0.0f, z = 0.0f;
        void setCutoff(double hz, double fs);
        float lp(float x) { z += a * (x - z); return z; }
    };
    struct Biquad {
        float b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0, z1 = 0, z2 = 0;
        void lowpass(double hz, double q, double fs);
        void bandpass(double hz, double q, double fs);
        float process(float x) {
            const float y = b0 * x + z1;
            z1 = b1 * x - a1 * y + z2;
            z2 = b2 * x - a2 * y;
            return y;
        }
    };
    /// Fractional-delay ring buffer (linear interpolation).
    struct DelayLine {
        std::vector<float> buf;
        std::size_t w = 0;
        void resize(std::size_t n) { buf.assign(n, 0.0f); w = 0; }
        void push(float x) { buf[w] = x; w = (w + 1) % buf.size(); }
        float read(double delaySamples) const;
    };
    /// A pipe as a digital waveguide: one delay line each way.
    struct Waveguide {
        DelayLine fwd, bwd;
        OnePole openEnd;   ///< low frequencies reflect at the open end, highs radiate
        OnePole wallLoss;  ///< viscothermal loss along the pipe
        double delay = 64.0;          ///< one-way, samples
        /// Reflection at the engine end, and the magnitude of the (inverted)
        /// low-frequency reflection at the open end. The round trip,
        /// 0.5 x 0.8 = 0.4, sets how peaky the pipe resonances are: a real
        /// system loses energy at the turbine/collector and in the
        /// silencer's perforations, so its rpm-to-rpm level swing is a few
        /// dB, not tens. AUTHORED.
        float valveReflection = 0.5f;
        float openReflection = 0.8f;
        float process(float in);
    };

private:
    /// A flow pulse: raised-cosine rise over `rise` samples, then an
    /// exponential decay with time constant `decay` samples.
    struct Pulse { double t = 0.0, rise = 1.0, decay = 1.0; float amp = 0.0f; };
    static float pulseValue(const Pulse& p);
    static bool  pulseDone(const Pulse& p) { return p.t > p.rise + 6.0 * p.decay; }
    struct Bank {
        std::vector<Pulse> pulses;
        Waveguide pipe;
        OnePole turbine;
    };

    float noise();                    ///< white, [-1,1]
    void  firePulse(int bank, double amplitude, double riseSamples, double decaySamples);
    void  smoothTowardTarget();

    double m_fs;
    VehicleSoundConfig m_cfg;
    int m_numCylinders = 4;
    std::vector<double> m_fireAngles;     ///< rad in [0, 4pi)
    std::vector<int>    m_fireBank;
    std::vector<double> m_cylGain;        ///< fixed per-cylinder strength
    std::vector<double> m_cylTiming;      ///< fixed per-cylinder timing offset [rad]
    EngineGeometry m_geometry;
    double m_secondOrderFactor = 0.0;     ///< |sum e_bank exp(2 i phi)| [-]
    double m_secondOrderPhase = 0.0;      ///< its phase [rad]

    SoundState m_target;
    SoundState m_cur;                     ///< per-sample smoothed copy
    bool m_haveState = false;

    // engine
    double m_crank = 0.0;                 ///< [0, 4pi)
    std::vector<Bank> m_banks;
    Waveguide m_intakeRunner;
    std::vector<Pulse> m_intakePulses;
    Biquad m_muffler1, m_muffler2;
    float m_exhaustRadPrev = 0.0f, m_intakeRadPrev = 0.0f;   ///< for dU/dt
    Biquad m_airbox;                      ///< Helmholtz resonator
    // structure-borne: combustion impulses into block/body modes
    std::vector<Pulse> m_combustion;
    std::array<Biquad, 6> m_modes;
    std::array<float, 6> m_modeGain{};
    Biquad m_throttleHissBp, m_mechBp;
    OnePole m_intakeLp;

    // turbo
    double m_turboPhase = 0.0, m_turboPhase2 = 0.0;
    Biquad m_turboWhooshBp, m_bovBp, m_bovLowBp;

    // driveline
    double m_gearPhase = 0.0;

    // tyres / wind
    std::array<Biquad, 4> m_squealBp;
    std::array<double, 4> m_squealHz{};
    Biquad m_rollLp, m_windLowBp, m_windHighBp;

    // EngineSim's engine sound, rendered in its own 128-frame blocks
    bool m_useEs = false;
    EngineSimSynth m_es;
    std::array<float, EngineSimSynth::kBlock> m_esL{}, m_esR{};
    int m_esPos = EngineSimSynth::kBlock;
    DelayLine m_propagationSide;          ///< its stereo difference, on the same path
    OnePole m_cabinExhaustSide;

    // listener
    DelayLine m_propagation;
    double m_propDelay = 0.0;             ///< samples, smoothed
    double m_lastPropDelay = 0.0;
    OnePole m_airAbsorptionL, m_airAbsorptionR;
    OnePole m_cabinExhaust, m_cabinEngine, m_cabinTyres, m_cabinWind, m_cabinStructure;
    float m_dcX = 0.0f, m_dcY = 0.0f;     ///< DC blocker state
    double m_pan = 0.0;

    std::uint32_t m_rng = 0x9E3779B9u;
    Mix m_mix;
    Diagnostics m_diag;
};

/// Write 16-bit stereo PCM. Returns false on I/O failure.
bool writeWav(const std::string& path, const std::vector<float>& interleavedStereo,
              int sampleRate);

} // namespace vsim
