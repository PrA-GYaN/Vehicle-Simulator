#include "config/Config.h"

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <set>
#include <string>

#include <yaml-cpp/yaml.h>

namespace fs = std::filesystem;

namespace vsim {
namespace {

fs::path g_executableDir;

/// Read a scalar if present, otherwise leave the destination untouched.
/// Templated so an absent key never silently zeroes a good default.
template <typename T>
void read(const YAML::Node& node, const char* key, T& out) {
    if (node && node[key] && !node[key].IsNull()) {
        try {
            out = node[key].as<T>();
        } catch (const YAML::Exception& e) {
            std::cerr << "[config] key '" << key << "' has the wrong type ("
                      << e.what() << "); keeping default\n";
        }
    }
}

/// Read an angle stored in DEGREES in YAML into a RADIANS destination.
/// Configuration is human-facing, the physics is not - see Units.h.
void readAngleDeg(const YAML::Node& node, const char* key, double& outRadians) {
    if (node && node[key] && !node[key].IsNull()) {
        try {
            outRadians = degToRad(node[key].as<double>());
        } catch (const YAML::Exception&) {
            std::cerr << "[config] angle '" << key << "' is not a number; keeping default\n";
        }
    }
}

void readVec3(const YAML::Node& node, const char* key, glm::dvec3& out) {
    if (node && node[key] && node[key].IsSequence() && node[key].size() == 3) {
        try {
            out.x = node[key][0].as<double>();
            out.y = node[key][1].as<double>();
            out.z = node[key][2].as<double>();
        } catch (const YAML::Exception&) {
            std::cerr << "[config] '" << key << "' is not a 3-number sequence; keeping default\n";
        }
    }
}

/// The `resolved:` block of a detailed engine - the crank-angle-resolved core.
/// Keys are EngineSim's own (engine_sim/config.py), so they can be carried
/// over from an engine .toml as they are; see ResolvedEngineParameters.h.
///
/// Unlike the rest of this file, an unknown key here is reported: the block
/// has some ninety keys, most of them long, and a silently ignored typo in
/// `iac_feedforward` is an engine that will not idle with nothing to say why.
void readResolvedEngine(const YAML::Node& node, ResolvedEngineParameters& r) {
    std::set<std::string> known;
    auto rd = [&](const YAML::Node& from, const char* key, auto& out) {
        known.insert(key);
        read(from, key, out);
    };
    auto warnUnknown = [&](const YAML::Node& from, const char* where) {
        if (!from.IsMap()) return;
        for (const auto& kv : from) {
            const std::string key = kv.first.as<std::string>("");
            if (!known.count(key)) {
                std::cerr << "[config] unknown key '" << key << "' in " << where
                          << " - ignored (see config/ResolvedEngineParameters.h)\n";
            }
        }
    };

    // layout
    rd(node, "num_valves_per_cyl",   r.numValvesPerCyl);
    rd(node, "firing_order",         r.firingOrder);
    rd(node, "firing_intervals_deg", r.firingIntervalsDeg);

    // valve events and geometry
    rd(node, "intake_open_btdc_deg",   r.intakeOpenBtdcDeg);
    rd(node, "intake_close_abdc_deg",  r.intakeCloseAbdcDeg);
    rd(node, "exhaust_open_bbdc_deg",  r.exhaustOpenBbdcDeg);
    rd(node, "exhaust_close_atdc_deg", r.exhaustCloseAtdcDeg);
    rd(node, "intake_max_lift_m",       r.intake.maxLiftM);
    rd(node, "intake_ramp_open_frac",   r.intake.rampOpenFrac);
    rd(node, "intake_head_dia_m",       r.intake.headDiaM);
    rd(node, "intake_port_dia_m",       r.intake.portDiaM);
    rd(node, "intake_seat_angle_deg",   r.intake.seatAngleDeg);
    rd(node, "intake_discharge_coeff",  r.intake.dischargeCoeff);
    rd(node, "exhaust_max_lift_m",      r.exhaust.maxLiftM);
    rd(node, "exhaust_ramp_open_frac",  r.exhaust.rampOpenFrac);
    rd(node, "exhaust_head_dia_m",      r.exhaust.headDiaM);
    rd(node, "exhaust_port_dia_m",      r.exhaust.portDiaM);
    rd(node, "exhaust_seat_angle_deg",  r.exhaust.seatAngleDeg);
    rd(node, "exhaust_discharge_coeff", r.exhaust.dischargeCoeff);
    rd(node, "vvt_intake_min_deg",  r.vvtIntakeMinDeg);
    rd(node, "vvt_intake_max_deg",  r.vvtIntakeMaxDeg);
    rd(node, "vvt_exhaust_min_deg", r.vvtExhaustMinDeg);
    rd(node, "vvt_exhaust_max_deg", r.vvtExhaustMaxDeg);
    rd(node, "vvt_rate_deg_per_s",  r.vvtRateDegPerS);
    rd(node, "vvt_auto",            r.vvtAuto);

    // air path and exhaust
    rd(node, "manifold_volume_m3",           r.manifoldVolumeM3);
    rd(node, "throttle_max_area_m2",         r.throttleMaxAreaM2);
    rd(node, "throttle_discharge_coeff",     r.throttleDischargeCoeff);
    rd(node, "exhaust_plenum_volume_m3",     r.exhaustPlenumVolumeM3);
    rd(node, "exhaust_tail_dia_m",           r.exhaustTailDiaM);
    rd(node, "exhaust_tail_discharge_coeff", r.exhaustTailDischargeCoeff);
    rd(node, "exhaust_post_volume_m3",       r.exhaustPostVolumeM3);

    // run state and ECU
    rd(node, "min_firing_rpm",           r.minFiringRpm);
    rd(node, "crank_revs_to_fire",       r.crankRevsToFire);
    rd(node, "stall_rpm",                r.stallRpm);
    rd(node, "dfco_enter_ratio",         r.dfcoEnterRatio);
    rd(node, "dfco_resume_ratio",        r.dfcoResumeRatio);
    rd(node, "dfco_after_start_delay_s", r.dfcoAfterStartDelayS);
    rd(node, "starter_stall_torque_nm",  r.starterStallTorqueNm);
    rd(node, "starter_free_speed_rpm",   r.starterFreeSpeedRpm);
    rd(node, "iac_feedforward",          r.iacFeedforward);
    rd(node, "iac_min",                  r.iacMin);
    rd(node, "iac_max",                  r.iacMax);
    rd(node, "iac_kp",                   r.iacKp);
    rd(node, "iac_ki",                   r.iacKi);
    rd(node, "iac_resolved_gain_scale",  r.iacResolvedGainScale);
    rd(node, "iac_resolved_kp_fraction", r.iacResolvedKpFraction);
    rd(node, "fast_idle_extra_ratio",    r.fastIdleExtraRatio);
    rd(node, "fast_idle_decay_s",        r.fastIdleDecayS);
    rd(node, "idle_spark_gain_deg_per_rpm", r.idleSparkGainDegPerRpm);
    rd(node, "idle_spark_reserve_deg",      r.idleSparkReserveDeg);
    rd(node, "idle_spark_max_advance_deg",  r.idleSparkMaxAdvanceDeg);
    rd(node, "idle_spark_max_retard_deg",   r.idleSparkMaxRetardDeg);

    // combustion and knock
    rd(node, "spark_timing_deg_btdc",   r.sparkTimingDegBtdc);
    rd(node, "use_mbt_timing",          r.useMbtTiming);
    rd(node, "wiebe_a",                 r.wiebeA);
    rd(node, "wiebe_m",                 r.wiebeM);
    rd(node, "combustion_duration_deg", r.combustionDurationDeg);
    rd(node, "use_two_zone",            r.useTwoZone);
    rd(node, "burn_residual_lo",        r.burnResidualLo);
    rd(node, "burn_residual_hi",        r.burnResidualHi);
    rd(node, "burn_lean_lo",            r.burnLeanLo);
    rd(node, "burn_lean_hi",            r.burnLeanHi);
    rd(node, "burn_rich_lo",            r.burnRichLo);
    rd(node, "burn_rich_hi",            r.burnRichHi);
    rd(node, "burn_misfire_quality",    r.burnMisfireQuality);
    rd(node, "wall_temperature_k",      r.wallTemperatureK);
    rd(node, "use_knock_model",         r.useKnockModel);
    rd(node, "fuel_octane_ron",         r.fuelOctaneRon);
    rd(node, "knock_retard_step_deg",   r.knockRetardStepDeg);
    rd(node, "knock_retard_max_deg",    r.knockRetardMaxDeg);
    rd(node, "knock_retard_recovery_deg_per_s", r.knockRetardRecoveryDegPerS);

    // numerics
    rd(node, "crank_angle_resolution_deg", r.crankAngleResolutionDeg);
    rd(node, "steps_per_ms",               r.stepsPerMs);
    rd(node, "max_substeps_per_update",    r.maxSubstepsPerUpdate);
    rd(node, "engine_timestep_s",          r.engineTimestepS);

    known.insert("turbo");
    warnUnknown(node, "engine.detailed.resolved");

    // Optional turbocharger - presence opts in. EngineSim's turbo_* keys,
    // without the prefix.
    if (const YAML::Node t = node["turbo"]) {
        r.useTurbo = true;
        known.clear();
        ResolvedTurboParameters& tp = r.turbo;
        rd(t, "count",                     tp.count);
        rd(t, "target_boost_pa",           tp.targetBoostPa);
        rd(t, "wastegate_gain",            tp.wastegateGain);
        rd(t, "inertia_kgm2",              tp.inertiaKgM2);
        rd(t, "design_speed_rps",          tp.designSpeedRps);
        rd(t, "design_flow_kgs",           tp.designFlowKgS);
        rd(t, "design_pressure_ratio",     tp.designPressureRatio);
        rd(t, "compressor_efficiency",     tp.compressorEfficiency);
        rd(t, "turbine_efficiency",        tp.turbineEfficiency);
        rd(t, "mechanical_efficiency",     tp.mechanicalEfficiency);
        rd(t, "turbine_area_m2",           tp.turbineAreaM2);
        rd(t, "wastegate_area_m2",         tp.wastegateAreaM2);
        rd(t, "boost_volume_m3",           tp.boostVolumeM3);
        rd(t, "intercooler_effectiveness", tp.intercoolerEffectiveness);
        warnUnknown(t, "engine.detailed.resolved.turbo");
    }
}

/// The `sound: engine_sim:` block - the engine's sound hardware as EngineSim
/// describes it. Keys are EngineSim's EngineConfig field names (the [exhaust],
/// [intake], [sound] and [layout] sections of an engine file), so they carry
/// across unchanged; see EngineSimSoundConfig in VehicleParameters.h. Unknown
/// keys are reported, as for `resolved:`.
void readEngineSimSound(const YAML::Node& node, EngineSimSoundConfig& c) {
    std::set<std::string> known;
    auto rd = [&](const YAML::Node& from, const char* key, auto& out) {
        known.insert(key);
        read(from, key, out);
    };
    // Per-pipe lists: one number means "every pipe", as in an engine file.
    auto rdList = [&](const char* key, std::vector<double>& out) {
        known.insert(key);
        const YAML::Node v = node[key];
        if (v && v.IsScalar()) {
            try { out = {v.as<double>()}; }
            catch (const YAML::Exception&) {
                std::cerr << "[config] key '" << key << "' is not a number or a list; keeping default\n";
            }
        } else {
            read(node, key, out);
        }
    };

    std::string layout;
    rd(node, "exhaust_layout", layout);
    if (layout == "dual")        c.dualExhaust = true;
    else if (layout == "single") c.dualExhaust = false;
    else if (!layout.empty()) {
        std::cerr << "[config] sound.engine_sim exhaust_layout '" << layout
                  << "' is not single|dual; keeping single\n";
    }
    rdList("exhaust_header_length_m",  c.headerLengthM);
    rdList("exhaust_mid_length_m",     c.midLengthM);
    rdList("exhaust_muffler_length_m", c.mufflerLengthM);
    rdList("exhaust_tail_length_m",    c.tailLengthM);
    rdList("exhaust_muffler_body_hz",  c.mufflerBodyHz);
    rd(node, "exhaust_muffler_area_ratio", c.mufflerAreaRatio);
    rd(node, "exhaust_header_temp_k",      c.headerTempK);
    rd(node, "exhaust_mid_temp_k",         c.midTempK);
    rd(node, "exhaust_muffler_temp_k",     c.mufflerTempK);
    rd(node, "exhaust_tail_temp_k",        c.tailTempK);
    rd(node, "exhaust_pipe_loss",          c.pipeLoss);
    rd(node, "exhaust_tip_damping",        c.tipDamping);
    rd(node, "cylinder_bank",              c.cylinderBank);
    rd(node, "intake_runner_length_m",     c.intakeRunnerLengthM);
    rd(node, "intake_runner_temp_k",       c.intakeRunnerTempK);
    rd(node, "sound_cylinder_strength_spread",   c.cylinderStrengthSpread);
    rd(node, "sound_cylinder_timing_spread_deg", c.cylinderTimingSpreadDeg);
    rd(node, "sound_noise_seed",                 c.noiseSeed);
    rd(node, "sound_blowdown_k",                 c.blowdownK);
    rd(node, "sound_crank_ripple_gain",          c.crankRippleGain);
    rd(node, "sound_combustion_variability",     c.combustionVariability);
    rd(node, "sound_idle_variability",           c.idleVariability);
    rd(node, "sound_pop_probability",            c.popProbability);
    rd(node, "sound_starter_ratio",              c.starterRatio);
    rd(node, "sound_starter_poles",              c.starterPoles);

    auto warnUnknown = [&](const YAML::Node& from, const char* where) {
        if (!from.IsMap()) return;
        for (const auto& kv : from) {
            const std::string key = kv.first.as<std::string>("");
            if (!known.count(key)) {
                std::cerr << "[config] unknown key '" << key << "' in " << where
                          << " - ignored (see EngineSimSoundConfig in config/VehicleParameters.h)\n";
            }
        }
    };
    known.insert("profile");
    warnUnknown(node, "sound.engine_sim");

    // The listener's dials (EngineSim README 6.6).
    if (const YAML::Node pf = node["profile"]) {
        known.clear();
        EngineSimSoundConfig::Profile& f = c.profile;
        rd(pf, "vol", f.vol);   rd(pf, "conv", f.conv); rd(pf, "comp", f.comp); rd(pf, "drive", f.drive);
        rd(pf, "lf", f.lf);     rd(pf, "hf", f.hf);     rd(pf, "ring", f.ring); rd(pf, "rasp", f.rasp);
        rd(pf, "lope", f.lope); rd(pf, "nlf", f.nlf);   rd(pf, "nhf", f.nhf);   rd(pf, "tick", f.tick);
        rd(pf, "intk", f.intk); rd(pf, "pops", f.pops);
        warnUnknown(pf, "sound.engine_sim.profile");
    }

    // Ranges the synthesiser is safe in. The dials are EngineSim's ranges.
    auto lengths = [](std::vector<double>& v, double fallback) {
        if (v.empty()) v = {fallback};
        for (double& x : v) x = clampd(sanitize(x, fallback), 0.05, 10.0);
    };
    lengths(c.headerLengthM, 0.55); lengths(c.midLengthM, 1.10);
    lengths(c.mufflerLengthM, 0.40); lengths(c.tailLengthM, 0.90);
    if (c.mufflerBodyHz.empty()) c.mufflerBodyHz = {125.0};
    for (double& x : c.mufflerBodyHz) x = clampd(sanitize(x, 125.0), 20.0, 2000.0);
    c.mufflerAreaRatio = clampd(sanitize(c.mufflerAreaRatio, 5.0), 0.2, 50.0);
    for (double* t : {&c.headerTempK, &c.midTempK, &c.mufflerTempK, &c.tailTempK, &c.intakeRunnerTempK}) {
        *t = clampd(sanitize(*t, 500.0), 200.0, 1500.0);
    }
    c.pipeLoss   = clampd(sanitize(c.pipeLoss, 0.992), 0.5, 1.0);
    c.tipDamping = clampd(sanitize(c.tipDamping, 0.35), 0.01, 1.0);
    c.intakeRunnerLengthM = clampd(sanitize(c.intakeRunnerLengthM, 0.45), 0.05, 3.0);
    c.cylinderStrengthSpread  = clampd(sanitize(c.cylinderStrengthSpread, 0.30), 0.0, 1.5);
    c.cylinderTimingSpreadDeg = clampd(sanitize(c.cylinderTimingSpreadDeg, 6.0), 0.0, 30.0);
    c.blowdownK             = clampd(sanitize(c.blowdownK, 1.75), 0.1, 20.0);
    c.crankRippleGain       = clampd(sanitize(c.crankRippleGain, 0.66), 0.0, 5.0);
    c.combustionVariability = clampd(sanitize(c.combustionVariability, 0.02), 0.0, 1.0);
    c.idleVariability       = clampd(sanitize(c.idleVariability, 0.05), 0.0, 1.0);
    c.popProbability        = clampd(sanitize(c.popProbability, 0.03), 0.0, 1.0);
    c.starterRatio          = clampd(sanitize(c.starterRatio, 11.0), 1.0, 50.0);
    c.starterPoles          = (c.starterPoles >= 1 && c.starterPoles <= 16) ? c.starterPoles : 4;
    EngineSimSoundConfig::Profile& f = c.profile;
    for (double* d : {&f.vol, &f.conv, &f.comp, &f.drive, &f.ring, &f.rasp, &f.lope, &f.nlf,
                      &f.nhf, &f.tick, &f.intk, &f.pops}) {
        *d = clampd(sanitize(*d, 0.5), 0.0, 1.0);
    }
    f.lf = clampd(sanitize(f.lf), -12.0, 12.0);
    f.hf = clampd(sanitize(f.hf), -12.0, 12.0);
}

/// The `detailed:` engine block. See config/EngineParameters.h for the
/// source notice: this is Heywood via ../EngineSim, NOT part of this
/// project's own Knowledge Base.
void readDetailedEngine(const YAML::Node& d, EngineParameters& e) {
    const YAML::Node resolved = d["resolved"];
    if (resolved) {
        // A resolved engine starts from Heywood Eq 13.6 converted correctly
        // (N in rev/min -> per rev/s: 900 and 18), not from the 15 / 0.005
        // the cycle-averaged model was calibrated with. See EngineParameters.h.
        e.frictionC2PaPerRevS = 900.0;
        e.frictionC3PaPerRevS2 = 18.0;
    }
    read(d, "bore",                  e.bore);
    read(d, "stroke",                e.stroke);
    read(d, "rod_length",            e.rodLength);
    read(d, "compression_ratio",     e.compressionRatio);
    read(d, "num_cylinders",         e.numCylinders);
    read(d, "inertia",               e.engineInertia);
    read(d, "idle_rpm",              e.idleRpm);
    read(d, "redline_rpm",           e.redlineRpm);
    read(d, "limiter_hysteresis_rpm", e.limiterHysteresisRpm);
    read(d, "ambient_pressure_pa",   e.ambientPressurePa);
    read(d, "intake_temperature_k",  e.intakeTemperatureK);
    read(d, "max_volumetric_efficiency", e.maxVolumetricEfficiency);
    read(d, "ve_reference_rpm",      e.veReferenceRpm);
    read(d, "stoich_afr",            e.stoichAfr);
    read(d, "lower_heating_value",   e.lowerHeatingValue);
    read(d, "combustion_efficiency", e.combustionEfficiency);
    if (const YAML::Node f = d["friction"]) {
        read(f, "c1_pa",             e.frictionC1Pa);
        read(f, "c2_pa_per_rev_s",   e.frictionC2PaPerRevS);
        read(f, "c3_pa_per_rev_s2",  e.frictionC3PaPerRevS2);
        read(f, "exhaust_back_pressure_pa", e.exhaustBackPressurePa);
    }
    // Optional turbocharger - presence of the block opts in, same
    // pattern as `detailed:` itself. See EngineParameters.h. This is the
    // CYCLE-AVERAGED model's turbo; a resolved engine's is `resolved.turbo`,
    // and this block is then inert (a vehicle file may keep both, so that
    // deleting `resolved:` falls back to the cycle-averaged engine).
    if (const YAML::Node t = d["turbo"]) {
        e.hasTurbo = true;
        read(t, "max_boost_pa",               e.maxBoostPa);
        read(t, "boost_threshold_rpm",        e.boostThresholdRpm);
        read(t, "full_boost_rpm",             e.fullBoostRpm);
        read(t, "spool_time_constant_s",      e.spoolTimeConstantS);
        read(t, "blow_off_time_constant_s",   e.blowOffTimeConstantS);
        read(t, "compressor_efficiency",      e.compressorEfficiency);
        read(t, "intercooler_effectiveness",  e.intercoolerEffectiveness);
    }
    // Optional crank-angle-resolved core - presence opts in.
    if (resolved) {
        e.useResolved = true;
        readResolvedEngine(resolved, e.resolved);
    }
}

} // namespace

//---------------------------------------------------------------------------
//  Validation
//---------------------------------------------------------------------------

void TireParameters::validate() {
    // Lateral friction. The KB's own range across every surface and tyre it
    // names runs from 0.10 (ice) to 1.8 (a Grand Prix tyre at light load); the
    // bounds here are wider still on purpose, so that an experiment can be run
    // outside the quoted range deliberately rather than being silently clipped
    // back into it.
    lateralFriction = clampd(sanitize(lateralFriction, 0.90), 0.01, 3.0);

    // mu_x/mu_y. 1.0 is [S1] §2.9's "pure-circle case"; anything else makes the
    // friction boundary an ellipse.
    longitudinalFrictionRatio =
        clampd(sanitize(longitudinalFrictionRatio, 1.0), 0.1, 3.0);

    // A NEGATIVE load sensitivity would mean friction rising with load, which
    // inverts the effect KB §2.4.3 describes. Zero is legal and means "disable
    // this term".
    frictionLoadSensitivity =
        clampd(sanitize(frictionLoadSensitivity, 0.10), 0.0, 2.0);

    // Cornering coefficient. Zero would give a tyre with no cornering
    // stiffness at all, which makes the normalised slip zero and the vehicle
    // undriveable, so the floor is above zero.
    corneringCoefficient =
        clampd(sanitize(corneringCoefficient, 0.15 * 180.0 / kPi), 0.1, 100.0);

    // Camber stiffness as a fraction of cornering stiffness. KB §12.2 gives
    // radial ~0.10 and bias ~0.17-0.20; zero disables camber thrust entirely.
    camberStiffnessRatio =
        clampd(sanitize(camberStiffnessRatio, 0.10), 0.0, 1.0);

    // `b` of Eq. 6-36. Negative would make cornering stiffness RISE without
    // bound with load, reversing the load-sensitivity effect the coefficient
    // exists to model.
    loadSensitivityB = std::max(0.0, sanitize(loadSensitivityB, 0.0));

    peakSlipRatio = clampd(sanitize(peakSlipRatio, 0.15), 0.005, 1.0);

    // The universal curve. B' and C' are floored above zero because the peak
    // location is found by inverting a function of both; C' < 1 would remove
    // the peak altogether, which is not a tyre.
    curveB = clampd(sanitize(curveB, 0.714), 0.01, 100.0);
    curveC = clampd(sanitize(curveC, 1.40), 1.0, 3.0);
    curveD = clampd(sanitize(curveD, 1.00), 0.05, 2.0);
    // E' > 1 makes phi(x) non-monotonic and the peak inversion meaningless.
    curveE = clampd(sanitize(curveE, -0.20), -5.0, 1.0);

    aligningMomentCoefficient =
        clampd(sanitize(aligningMomentCoefficient, 0.043 * 0.3048 * 180.0 / kPi),
               0.0, 10.0);

    // Tyre vertical rate sets the deflection and hence all three radii. Zero
    // would make an infinitely stiff tyre (deflection zero, R_e = R_u), which
    // is a legal degenerate case but not a useful one.
    verticalRate = clampd(sanitize(verticalRate, 175127.0), 1000.0, 1.0e8);

    relaxationLengthRevolutions =
        clampd(sanitize(relaxationLengthRevolutions, 0.75), 0.0, 10.0);

    staticCamber = clampAbs(sanitize(staticCamber, 0.0), degToRad(10.0));
}

void SuspensionParameters::validate() {
    // Ride frequency. The floor is well below [S3]'s softest quoted class
    // (30 cpm = 0.5 Hz) and the ceiling well above its stiffest road figure,
    // so a deliberate experiment is not clipped.
    rideFrequencyFront = clampd(sanitize(rideFrequencyFront, 1.25), 0.1, 10.0);
    rideFrequencyRear  = clampd(sanitize(rideFrequencyRear, 1.20),  0.1, 10.0);

    // Roll centre heights. A roll centre BELOW ground is physically meaningful
    // and common, so negatives are allowed; the bounds only keep the geometry
    // sane relative to the vehicle.
    rollCentreHeightFront = clampd(sanitize(rollCentreHeightFront, 0.0787), -1.0, 1.0);
    rollCentreHeightRear  = clampd(sanitize(rollCentreHeightRear, 0.1730),  -1.0, 1.0);

    // A negative bar rate would subtract roll stiffness, which no bar does.
    antiRollRateFront = std::max(0.0, sanitize(antiRollRateFront, 0.0));
    antiRollRateRear  = std::max(0.0, sanitize(antiRollRateRear, 0.0));

    unsprungFractionDriven =
        clampd(sanitize(unsprungFractionDriven, 0.15), 0.0, 0.6);
    unsprungFractionNonDriven =
        clampd(sanitize(unsprungFractionNonDriven, 0.10), 0.0, 0.6);

    // -1 is perfect camber compensation (wheel upright in roll); 0 is none.
    // Values outside that band are unusual but not impossible.
    camberChangePerRoll =
        clampd(sanitize(camberChangePerRoll, 0.0), -2.0, 1.0);

    // zeta <= 0 would give an undamped or negatively-damped oscillator - not
    // a suspension. The ceiling is well above [S3]'s "considerably higher
    // than passenger cars" race figure, so a deliberate experiment is not
    // clipped.
    dampingRatioFront = clampd(sanitize(dampingRatioFront, 0.25), 0.05, 3.0);
    dampingRatioRear  = clampd(sanitize(dampingRatioRear, 0.25),  0.05, 3.0);
}

void VehicleParameters::validate() {
    // Wheelbase appears in the denominator of the yaw rate equation. A value
    // of zero or negative is not a "bad tuning choice", it is a division by
    // zero on the first physics step.
    if (!(wheelbase > 0.05)) {
        std::cerr << "[config] vehicle '" << name << "': wheelbase " << wheelbase
                  << " m is invalid; clamped to 0.05 m\n";
        wheelbase = 0.05;
    }
    mass       = clampd(sanitize(mass, 1500.0), 1.0, 1.0e6);
    length     = clampd(sanitize(length, 4.5), 0.1, 40.0);
    width      = clampd(sanitize(width, 1.8),  0.1, 10.0);
    height     = clampd(sanitize(height, 1.4), 0.1, 10.0);
    frontTrack = clampd(sanitize(frontTrack, 1.55), 0.1, 10.0);
    rearTrack  = clampd(sanitize(rearTrack, 1.55),  0.1, 10.0);
    cgHeight   = clampd(sanitize(cgHeight, 0.55),   0.01, height);

    // A weight distribution of exactly 0 or 1 puts the CG on an axle, which
    // is legal geometry but makes several derived quantities degenerate.
    frontWeightDistribution =
        clampd(sanitize(frontWeightDistribution, 0.55), 0.05, 0.95);

    // Steering beyond ~80 degrees makes tan(delta) explode, and a real road
    // wheel cannot reach it anyway.
    maxSteeringAngle = clampd(sanitize(maxSteeringAngle, degToRad(35.0)),
                              degToRad(1.0), degToRad(80.0));
    steeringRate     = clampd(sanitize(steeringRate, degToRad(180.0)),
                              degToRad(1.0), degToRad(2000.0));

    maxAcceleration = clampd(sanitize(maxAcceleration, 4.0), 0.01, 100.0);
    maxBraking      = clampd(sanitize(maxBraking, 8.0),      0.01, 100.0);
    maxSpeed        = clampd(sanitize(maxSpeed, 60.0),       0.1,  200.0);

    wheelRadius = clampd(sanitize(wheelRadius, 0.34), 0.05, 2.0);
    wheelWidth  = clampd(sanitize(wheelWidth, 0.22),  0.02, 1.0);

    // Road load. A negative coefficient would ACCELERATE a coasting vehicle,
    // so the lower bound is zero rather than something merely small. Zero is
    // legal and means "disable this term".
    rollingResistanceCoeff = clampd(sanitize(rollingResistanceCoeff, 0.015), 0.0, 1.0);
    dragCoefficient        = clampd(sanitize(dragCoefficient, 0.32),         0.0, 3.0);
    frontalArea            = clampd(sanitize(frontalArea, 2.2),              0.0, 50.0);
    engineBrakingCoeff     = clampd(sanitize(engineBrakingCoeff, 0.012),     0.0, 2.0);

    // Wheel inertia divides the wheel's rotational equation of motion
    // (Gillespie Eq. 3-18), so zero is a division by zero on the first step.
    wheelInertia = clampd(sanitize(wheelInertia, 1.2), 0.001, 1000.0);

    // Zero is meaningful here - it selects the documented fallback in
    // effectiveYawInertia(). Negative is not.
    yawInertia = std::max(0.0, sanitize(yawInertia, 0.0));

    // Zero selects the ideal proportioning derived from Gillespie Eq. 3-29/30.
    brakeProportionFront = clampd(sanitize(brakeProportionFront, 0.0), 0.0, 1.0);

    tire.validate();
    suspension.validate();

    if (useDetailedEngine) {
        detailedEngine.validate();
        gearbox.validate();
    }
}

void SimulationConfig::validate() {
    // Below ~0.1 ms the accumulator does more work than the physics.
    // Above ~0.1 s explicit Euler visibly degrades on a curved path.
    timestep  = clampd(sanitize(timestep, 0.01), 0.0001, 0.1);
    timeScale = clampd(sanitize(timeScale, 1.0), 0.0, 100.0);
    maxStepsPerFrame = std::max(1, std::min(maxStepsPerFrame, 1000));
}

//---------------------------------------------------------------------------
//  Path resolution
//---------------------------------------------------------------------------

void setExecutablePath(const char* argv0) {
    if (!argv0) return;
    std::error_code ec;
    fs::path p = fs::absolute(fs::path(argv0), ec);
    if (!ec) g_executableDir = p.parent_path();
}

std::string resolveResourcePath(const std::string& relativePath) {
    std::error_code ec;
    if (fs::exists(relativePath, ec)) return relativePath;

    if (!g_executableDir.empty()) {
        fs::path candidate = g_executableDir / relativePath;
        if (fs::exists(candidate, ec)) return candidate.string();

        // Also try one level up, which covers running straight out of a
        // build tree layout like build/bin/exe with assets at build/.
        fs::path up = g_executableDir.parent_path() / relativePath;
        if (fs::exists(up, ec)) return up.string();
    }
    return relativePath;  // let the caller report the failure
}

//---------------------------------------------------------------------------
//  Loaders
//---------------------------------------------------------------------------

AppConfig loadAppConfig(const std::string& path) {
    AppConfig cfg;
    const std::string resolved = resolveResourcePath(path);

    YAML::Node root;
    try {
        root = YAML::LoadFile(resolved);
    } catch (const std::exception& e) {
        std::cerr << "[config] could not read '" << resolved << "' (" << e.what()
                  << "); using built-in defaults\n";
        cfg.simulation.validate();
        return cfg;
    }

    if (const YAML::Node n = root["simulation"]) {
        read(n, "timestep",            cfg.simulation.timestep);
        read(n, "time_scale",          cfg.simulation.timeScale);
        read(n, "realtime",            cfg.simulation.realtime);
        read(n, "max_steps_per_frame", cfg.simulation.maxStepsPerFrame);
        read(n, "dynamics_model",      cfg.simulation.dynamicsModel);
    }

    if (const YAML::Node n = root["initial_state"]) {
        readVec3(n, "position", cfg.initialState.position);
        readAngleDeg(n, "yaw",  cfg.initialState.yaw);
        read(n, "velocity",     cfg.initialState.velocity);
    }

    if (const YAML::Node n = root["camera"]) {
        read(n, "distance",    cfg.camera.distance);
        read(n, "height",      cfg.camera.height);
        read(n, "follow_speed",cfg.camera.followSpeed);
        read(n, "look_ahead",  cfg.camera.lookAhead);
        read(n, "fov",         cfg.camera.fovDegrees);
        read(n, "near_plane",  cfg.camera.nearPlane);
        read(n, "far_plane",   cfg.camera.farPlane);
    }

    if (const YAML::Node n = root["controls"]) {
        readAngleDeg(n, "steering_rate",        cfg.controls.steeringRate);
        readAngleDeg(n, "steering_return_rate", cfg.controls.steeringReturnRate);
        read(n, "throttle_rate", cfg.controls.throttleRate);
        read(n, "clutch_rate",   cfg.controls.clutchRate);
        read(n, "brake_rate",    cfg.controls.brakeRate);
    }

    if (const YAML::Node n = root["telemetry"]) {
        read(n, "enabled",          cfg.telemetry.enabledAtStart);
        read(n, "output_directory", cfg.telemetry.outputDirectory);
        read(n, "max_samples",      cfg.telemetry.maxSamples);
    }

    if (const YAML::Node n = root["experiment"]) {
        read(n, "type",           cfg.experiment.type);
        read(n, "initial_speed",  cfg.experiment.initialSpeed);
        readAngleDeg(n, "steering_angle", cfg.experiment.steeringAngle);
        read(n, "duration",       cfg.experiment.duration);
        read(n, "step_time",      cfg.experiment.stepTime);
        read(n, "period",         cfg.experiment.period);
        read(n, "auto_start",     cfg.experiment.autoStart);
    }

    if (const YAML::Node n = root["android_input"]) {
        read(n, "enabled",          cfg.androidInput.enabled);
        read(n, "port",             cfg.androidInput.port);
        read(n, "timeout_seconds",  cfg.androidInput.timeoutSeconds);
        read(n, "auto_adb_reverse", cfg.androidInput.autoAdbReverse);
    }

    if (const YAML::Node n = root["audio"]) {
        read(n, "enabled",       cfg.audio.enabled);
        read(n, "master_volume", cfg.audio.masterVolume);
        read(n, "sample_rate",   cfg.audio.sampleRate);
    }

    if (const YAML::Node n = root["window"]) {
        read(n, "width",  cfg.windowWidth);
        read(n, "height", cfg.windowHeight);
        read(n, "vsync",  cfg.vsync);
    }

    if (const YAML::Node n = root["environment"]) {
        read(n, "air_density", cfg.environment.airDensity);
        readAngleDeg(n, "grade", cfg.environment.gradeAngle);
        read(n, "headwind", cfg.environment.headwind);
    }

    if (const YAML::Node n = root["environment_model"]) {
        read(n, "path",         cfg.environmentModel.path);
        read(n, "forward_axis", cfg.environmentModel.forwardAxis);
        read(n, "up_axis",      cfg.environmentModel.upAxis);
        if (const YAML::Node sp = n["spawn"]) {
            cfg.environmentModel.hasSpawn = true;
            readVec3(sp, "position", cfg.environmentModel.spawnPosition);
            readAngleDeg(sp, "yaw", cfg.environmentModel.spawnYaw);
        }
    }

    read(root, "default_vehicle", cfg.defaultVehicle);
    read(root, "default_track",   cfg.defaultTrack);

    cfg.simulation.validate();
    return cfg;
}

VehicleParameters loadVehicleParameters(const std::string& path) {
    VehicleParameters p;
    const std::string resolved = resolveResourcePath(path);

    YAML::Node root;
    try {
        root = YAML::LoadFile(resolved);
    } catch (const std::exception& e) {
        std::cerr << "[config] could not read vehicle '" << resolved << "' ("
                  << e.what() << "); using built-in defaults\n";
        p.validate();
        return p;
    }

    const YAML::Node v = root["vehicle"] ? root["vehicle"] : root;

    read(v, "name",      p.name);
    read(v, "mass",      p.mass);
    read(v, "wheelbase", p.wheelbase);

    if (const YAML::Node n = v["dimensions"]) {
        read(n, "length", p.length);
        read(n, "width",  p.width);
        read(n, "height", p.height);
    }
    if (const YAML::Node n = v["track"]) {
        read(n, "front", p.frontTrack);
        read(n, "rear",  p.rearTrack);
    }
    if (const YAML::Node n = v["cg"]) {
        read(n, "height", p.cgHeight);
        read(n, "front_weight_distribution", p.frontWeightDistribution);
    }
    if (const YAML::Node n = v["steering"]) {
        readAngleDeg(n, "max_angle",     p.maxSteeringAngle);
        readAngleDeg(n, "steering_rate", p.steeringRate);
    }
    if (const YAML::Node n = v["engine"]) {
        read(n, "max_acceleration", p.maxAcceleration);

        //-- detailed engine (optional) -------------------------------------
        //
        // Presence of this block, not a separate boolean, is what opts a
        // vehicle in - same pattern as VehicleModelConfig::hasModel(). See
        // config/EngineParameters.h for the source notice: this is Heywood
        // via ../EngineSim, NOT part of this project's own Knowledge Base.
        if (const YAML::Node d = n["detailed"]) {
            p.useDetailedEngine = true;
            readDetailedEngine(d, p.detailedEngine);
        }
    }
    if (const YAML::Node n = v["gearbox"]) {
        // AUTHORED, not from the vehicle-dynamics KB — see GearboxParameters.h.
        // Only meaningful together with `engine.detailed` above.
        read(n, "ratios",                     p.gearbox.gearRatios);
        read(n, "final_drive",                p.gearbox.finalDrive);
        read(n, "efficiency",                 p.gearbox.efficiency);
        read(n, "upshift_redline_fraction",   p.gearbox.upshiftRedlineFraction);
        read(n, "downshift_redline_fraction", p.gearbox.downshiftRedlineFraction);
        read(n, "min_shift_interval_s",       p.gearbox.minShiftIntervalS);
        read(n, "clutch_capacity_nm",         p.gearbox.clutchCapacityNm);
        read(n, "clutch_engage_time_s",       p.gearbox.clutchEngageTimeS);
        read(n, "launch_rpm",                 p.gearbox.launchRpm);

        // Modes. Unknown values keep the default and say so, the same policy
        // as the drivetrain layout above - a typo must never silently change
        // how the car drives.
        std::string shiftMode;
        read(n, "shift_mode", shiftMode);
        if (shiftMode == "manual")         p.gearbox.shiftMode = ShiftMode::Manual;
        else if (shiftMode == "automatic") p.gearbox.shiftMode = ShiftMode::Automatic;
        else if (!shiftMode.empty()) {
            std::cerr << "[config] vehicle '" << p.name << "': gearbox shift_mode '"
                      << shiftMode << "' is not automatic|manual; keeping automatic\n";
        }

        std::string clutchMode;
        read(n, "clutch_mode", clutchMode);
        if (clutchMode == "manual")         p.gearbox.clutchMode = ClutchMode::Manual;
        else if (clutchMode == "automatic") p.gearbox.clutchMode = ClutchMode::Automatic;
        else if (!clutchMode.empty()) {
            std::cerr << "[config] vehicle '" << p.name << "': gearbox clutch_mode '"
                      << clutchMode << "' is not automatic|manual; keeping automatic\n";
        }
    }
    if (const YAML::Node n = v["brakes"]) {
        read(n, "max_deceleration",     p.maxBraking);
        read(n, "front_proportion",     p.brakeProportionFront);
    }
    if (const YAML::Node n = v["limits"]) {
        read(n, "max_speed", p.maxSpeed);
    }
    if (const YAML::Node n = v["wheels"]) {
        read(n, "radius",  p.wheelRadius);
        read(n, "width",   p.wheelWidth);
        read(n, "inertia", p.wheelInertia);
    }
    if (const YAML::Node n = v["inertia"]) {
        read(n, "yaw", p.yawInertia);
    }
    //-- suspension --------------------------------------------------------
    //
    // The roll chain of KB §3.5.1: ride frequency -> ride rate -> roll rate ->
    // roll gradient -> lateral load transfer in three terms. See
    // config/SuspensionParameters.h for what each value is and where it comes
    // from.
    if (const YAML::Node n = v["suspension"]) {
        read(n, "ride_frequency_front",   p.suspension.rideFrequencyFront);
        read(n, "ride_frequency_rear",    p.suspension.rideFrequencyRear);
        read(n, "roll_centre_height_front", p.suspension.rollCentreHeightFront);
        read(n, "roll_centre_height_rear",  p.suspension.rollCentreHeightRear);
        read(n, "anti_roll_rate_front",   p.suspension.antiRollRateFront);
        read(n, "anti_roll_rate_rear",    p.suspension.antiRollRateRear);
        read(n, "unsprung_fraction_driven",     p.suspension.unsprungFractionDriven);
        read(n, "unsprung_fraction_non_driven", p.suspension.unsprungFractionNonDriven);
        read(n, "camber_change_per_roll", p.suspension.camberChangePerRoll);
        read(n, "damping_ratio_front", p.suspension.dampingRatioFront);
        read(n, "damping_ratio_rear",  p.suspension.dampingRatioRear);
    }
    if (const YAML::Node n = v["drivetrain"]) {
        std::string layout;
        read(n, "layout", layout);
        if (layout == "front") {
            p.driveLayout = DriveLayout::FrontWheelDrive;
        } else if (layout == "rear") {
            p.driveLayout = DriveLayout::RearWheelDrive;
        } else if (!layout.empty()) {
            // No silent fallback: KB Section 5.7.3 treats front and rear drive
            // as materially different cases, and neither source covers AWD.
            std::cerr << "[config] vehicle '" << p.name << "': drivetrain layout '"
                      << layout << "' is not one of front|rear; keeping "
                      << "rear-wheel drive\n";
        }
    }
    if (const YAML::Node n = v["road_load"]) {
        read(n, "rolling_resistance", p.rollingResistanceCoeff);
        read(n, "drag_coefficient",   p.dragCoefficient);
        read(n, "frontal_area",       p.frontalArea);
        read(n, "engine_braking",     p.engineBrakingCoeff);
    }

    //-- tyre -------------------------------------------------------------
    //
    // Angles are authored in DEGREES and coefficients per DEGREE, because
    // that is how the Knowledge Base quotes every one of them (KB Section 0.3:
    // cornering stiffness in lb/deg, camber coefficient in lb/lb/deg, aligning
    // moment coefficient in ft-lb/lb/deg). Converting on load keeps the config
    // readable against the KB and the physics in radians, per Units.h.
    read(v, "traction_control", p.tractionControl);
    p.tractionControl = std::min(std::max(p.tractionControl, 0), 3);

    if (const YAML::Node n = v["tire"]) {
        std::string compound;
        read(n, "compound", compound);
        p.tireCompound = tireCompoundFromString(compound);
        read(n, "lateral_friction",            p.tire.lateralFriction);
        read(n, "longitudinal_friction_ratio", p.tire.longitudinalFrictionRatio);
        read(n, "friction_load_sensitivity",   p.tire.frictionLoadSensitivity);

        // lb/lb/deg -> per radian
        if (n["cornering_coefficient"] && !n["cornering_coefficient"].IsNull()) {
            try {
                p.tire.corneringCoefficient =
                    n["cornering_coefficient"].as<double>() * 180.0 / kPi;
            } catch (const YAML::Exception&) {
                std::cerr << "[config] tire.cornering_coefficient is not a "
                             "number; keeping default\n";
            }
        }

        read(n, "camber_stiffness_ratio", p.tire.camberStiffnessRatio);
        read(n, "load_sensitivity_b",     p.tire.loadSensitivityB);
        read(n, "peak_slip_ratio",        p.tire.peakSlipRatio);

        // The universal curve fit - [S3] Eqs. (14.5)-(14.7).
        if (const YAML::Node c = n["universal_curve"]) {
            read(c, "b", p.tire.curveB);
            read(c, "c", p.tire.curveC);
            read(c, "d", p.tire.curveD);
            read(c, "e", p.tire.curveE);
        }

        // ft-lb/lb/deg -> m per radian
        if (n["aligning_moment_coefficient"] &&
            !n["aligning_moment_coefficient"].IsNull()) {
            try {
                p.tire.aligningMomentCoefficient =
                    n["aligning_moment_coefficient"].as<double>() * 0.3048 * 180.0 / kPi;
            } catch (const YAML::Exception&) {
                std::cerr << "[config] tire.aligning_moment_coefficient is not "
                             "a number; keeping default\n";
            }
        }

        read(n, "vertical_rate",              p.tire.verticalRate);
        read(n, "relaxation_length_revolutions",
             p.tire.relaxationLengthRevolutions);
        readAngleDeg(n, "static_camber",      p.tire.staticCamber);
    }

    //-- how the vehicle is drawn -------------------------------------------
    //
    // Entirely a rendering concern - see config/VehicleParameters.h. Absent
    // or empty `path` leaves the vehicle drawn with the generated car body,
    // whose silhouette `body_style` selects.
    if (const YAML::Node n = v["model"]) {
        read(n, "body_style",     p.model.bodyStyle);
        read(n, "path",           p.model.path);
        read(n, "forward_axis",   p.model.forwardAxis);
        read(n, "up_axis",        p.model.upAxis);
        if (const YAML::Node w = n["wheel_nodes"]) {
            read(w, "front_left",  p.model.frontLeftNode);
            read(w, "front_right", p.model.frontRightNode);
            read(w, "rear_left",   p.model.rearLeftNode);
            read(w, "rear_right",  p.model.rearRightNode);
        }
    }

    // Presentation-only blocks - cameras and sound. See VehicleParameters.h.
    auto readPoint = [](const YAML::Node& n, const char* key, LocalPoint& out) {
        const YAML::Node v = n[key];
        if (v && v.IsSequence() && v.size() == 3) {
            for (std::size_t i = 0; i < 3; ++i) out.xyz[i] = v[i].as<double>();
            out.set = true;
        }
    };
    if (const YAML::Node n = v["cameras"]) {
        readPoint(n, "driver_eye", p.cameras.driverEye);
        readPoint(n, "bonnet",     p.cameras.bonnet);
        readPoint(n, "bumper",     p.cameras.bumper);
        readPoint(n, "wheel",      p.cameras.wheel);
    }
    if (const YAML::Node n = v["sound"]) {
        VehicleSoundConfig& sc = p.sound;
        read(n, "firing_angles_deg",            sc.firingAnglesDeg);
        read(n, "firing_banks",                 sc.firingBanks);
        read(n, "exhaust_pipe_length",          sc.exhaustPipeLength);
        read(n, "exhaust_gas_temperature_fraction", sc.exhaustGasTemperatureFraction);
        read(n, "blowdown_crank_deg",           sc.blowdownCrankDeg);
        read(n, "muffler_cutoff_hz",            sc.mufflerCutoffHz);
        read(n, "cyclic_variation",             sc.cyclicVariation);
        read(n, "turbo_tip_radius",             sc.turboTipRadius);
        read(n, "turbo_blades",                 sc.turboBlades);
        read(n, "gear_teeth",                   sc.gearTeeth);
        read(n, "cylinder_variation",           sc.cylinderVariation);
        read(n, "reciprocating_mass",           sc.reciprocatingMass);
        read(n, "vee_angle_deg",                sc.veeAngleDeg);
        read(n, "airbox_volume",                sc.airboxVolume);
        read(n, "snorkel_length",               sc.snorkelLength);
        read(n, "snorkel_radius",               sc.snorkelRadius);
        readPoint(n, "engine_position",  sc.engine);
        readPoint(n, "exhaust_position", sc.exhaust);

        // EngineSim's engine sound - presence opts in, and it needs the
        // resolved engine to take firing order and valve events from.
        if (const YAML::Node es = n["engine_sim"]) {
            if (p.useDetailedEngine && p.detailedEngine.useResolved) {
                sc.useEngineSim = true;
                readEngineSimSound(es, sc.engineSim);
            } else {
                std::cerr << "[config] vehicle '" << p.name << "': sound.engine_sim needs an "
                             "`engine: detailed: resolved:` engine - ignored\n";
            }
        }
    }

    p.validate();
    return p;
}

bool loadEngineParameters(const std::string& path, EngineParameters& out) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(resolveResourcePath(path));
    } catch (const std::exception& e) {
        std::cerr << "[config] could not read engine '" << path << "' (" << e.what() << ")\n";
        return false;
    }
    // Accept the block on its own, or a whole vehicle file. (Looked up
    // through a const node: yaml-cpp's non-const operator[] creates the key.)
    const YAML::Node& croot = root;
    auto findBlock = [&croot]() -> YAML::Node {
        if (croot["detailed"]) return croot["detailed"];
        if (croot["engine"] && croot["engine"]["detailed"]) return croot["engine"]["detailed"];
        if (croot["vehicle"] && croot["vehicle"]["engine"]
            && croot["vehicle"]["engine"]["detailed"]) {
            return croot["vehicle"]["engine"]["detailed"];
        }
        return YAML::Node(YAML::NodeType::Undefined);
    };
    const YAML::Node d = findBlock();
    if (!d.IsDefined() || !d.IsMap()) {
        std::cerr << "[config] '" << path << "' has no `detailed:` engine block\n";
        return false;
    }
    out = EngineParameters{};
    readDetailedEngine(d, out);
    out.validate();
    return true;
}

bool loadEngineSimSound(const std::string& path, EngineSimSoundConfig& out) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(resolveResourcePath(path));
    } catch (const std::exception& e) {
        std::cerr << "[config] could not read '" << path << "' (" << e.what() << ")\n";
        return false;
    }
    const YAML::Node& croot = root;
    const YAML::Node holder = croot["vehicle"] ? croot["vehicle"] : croot;
    if (!holder["sound"] || !holder["sound"]["engine_sim"]) {
        std::cerr << "[config] '" << path << "' has no `sound: engine_sim:` block\n";
        return false;
    }
    out = EngineSimSoundConfig{};
    readEngineSimSound(holder["sound"]["engine_sim"], out);
    return true;
}

std::vector<std::string> listAvailableVehicles(const std::string& vehicleDir) {
    std::vector<std::string> names;
    const std::string resolved = resolveResourcePath(vehicleDir);

    std::error_code ec;
    if (!fs::is_directory(resolved, ec)) return names;

    for (const auto& entry : fs::directory_iterator(resolved, ec)) {
        if (ec) break;
        if (!entry.is_regular_file()) continue;
        const auto ext = entry.path().extension().string();
        if (ext == ".yaml" || ext == ".yml") {
            names.push_back(entry.path().stem().string());
        }
    }
    std::sort(names.begin(), names.end());
    return names;
}

} // namespace vsim
