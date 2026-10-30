#pragma once
//===========================================================================
//  ResolvedEngineParameters.h  --  what the crank-angle-resolved engine core
//  needs on top of EngineParameters.
//
//  SOURCE NOTICE - same status as EngineParameters.h: Heywood via the
//  EngineSim project (../EngineSim), NOT this project's S1/S2/S3
//  vehicle-dynamics Knowledge Base. Every field here is the C++ twin of a
//  field of EngineSim's `EngineConfig` (engine_sim/config.py), with the same
//  meaning, the same unit and - unless a comment says otherwise - the same
//  default. The YAML key is EngineSim's own key name, so a value can be
//  carried across from an engine .toml unchanged; tools/engine_toml_to_yaml.py
//  does it for a whole file.
//
//  Which numbers are physics and which are AUTHORED is recorded where
//  EngineSim records it: ../EngineSim/KB_GAPS.md (G-E03..G-E06 for the ECU and
//  run-state logic) and the comments in engine_sim/config.py. In short:
//  geometry, valve events and the gas-exchange/combustion/heat-transfer laws
//  are Heywood; everything an ECU does (idle air, spark trim, fuel cut, VVT
//  schedule, wastegate law) and every turbo "map" figure is authored.
//
//  See physics/ResolvedEngine.h for what these drive.
//===========================================================================

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <vector>

#include "core/Units.h"

namespace vsim {

/// One valve train (intake or exhaust). Diameters are PER VALVE; the count
/// comes from ResolvedEngineParameters::numValvesPerCyl.
struct ValveTrainParameters {
    double maxLiftM       = 9.5e-3;   ///< [m] peak lift
    /// Fraction of the event spent opening (0..1); the rest closes. 0.5 is
    /// the symmetric raised-cosine hump. EngineSim's ramp_open_frac - its
    /// ramp_close_frac must be 1 minus this, so only one is stored.
    double rampOpenFrac   = 0.5;
    double headDiaM       = 30.0e-3;  ///< [m] valve head diameter
    double portDiaM       = 30.0e-3;  ///< [m] port throat diameter
    double seatAngleDeg   = 45.0;     ///< [deg] from the head plane (Heywood Eq 6.7)
    double dischargeCoeff = 0.67;     ///< [-]
};

/// EngineSim's turbo_system.py components. Twin turbos are one turbo with
/// the flow, area and inertia of the pair (`count`).
struct ResolvedTurboParameters {
    int    count                   = 1;
    double targetBoostPa           = 150000.0;  ///< [Pa] ABSOLUTE charge pressure the wastegate holds
    double wastegateGain           = 0.5;       ///< [-] scales the wastegate PI law
    double inertiaKgM2             = 1.2e-4;    ///< [kg.m^2] shaft + wheels, per turbo
    double designSpeedRps          = 2200.0;    ///< [rev/s] compressor map design speed
    double designFlowKgS           = 0.14;      ///< [kg/s] per turbo at the design point
    double designPressureRatio     = 2.4;       ///< [-]
    double compressorEfficiency    = 0.72;      ///< [-] Heywood Eq 6.42
    double turbineEfficiency       = 0.78;      ///< [-] Heywood Eq 6.44-6.48
    double mechanicalEfficiency    = 0.95;      ///< [-]
    double turbineAreaM2           = 5.0e-4;    ///< [m^2] effective flow area, all turbines
    double wastegateAreaM2         = 3.5e-4;    ///< [m^2] fully open, all wastegates
    double boostVolumeM3           = 8.0e-3;    ///< [m^3] intercooler + charge piping
    double intercoolerEffectiveness = 0.7;      ///< [-]
};

struct ResolvedEngineParameters {
    // --- layout -----------------------------------------------------------
    int numValvesPerCyl = 4;
    /// 1-based cylinder numbers in firing order. Empty = 1, 2, .. N.
    std::vector<int> firingOrder;
    /// Crank degrees between successive firings, in firing order (sum 720).
    /// Empty = even fire (720/N).
    std::vector<double> firingIntervalsDeg;

    // --- valve events [crank degrees] -------------------------------------
    double intakeOpenBtdcDeg   = 10.0;
    double intakeCloseAbdcDeg  = 50.0;
    double exhaustOpenBbdcDeg  = 50.0;
    double exhaustCloseAtdcDeg = 10.0;
    ValveTrainParameters intake;
    ValveTrainParameters exhaust{9.5e-3, 0.5, 26.0e-3, 26.0e-3, 45.0, 0.67};

    // --- variable valve timing (cam advance, + = events earlier) -----------
    double vvtIntakeMinDeg  = 0.0;
    double vvtIntakeMaxDeg  = 0.0;
    double vvtExhaustMinDeg = 0.0;
    double vvtExhaustMaxDeg = 0.0;
    double vvtRateDegPerS   = 80.0;
    bool   vvtAuto          = true;

    // --- air path ---------------------------------------------------------
    double manifoldVolumeM3       = 0.003;   ///< [m^3] intake plenum
    double throttleMaxAreaM2      = 0.003;   ///< [m^2] effective area at WOT
    double throttleDischargeCoeff = 0.65;

    // --- exhaust (backpressure) -------------------------------------------
    double exhaustPlenumVolumeM3     = 15.0e-3;
    double exhaustTailDiaM           = 45.0e-3;
    double exhaustTailDischargeCoeff = 0.70;
    double exhaustPostVolumeM3       = 8.0e-3;   ///< behind the turbine (turbo only)

    // --- run state / ECU (EngineSim KB_GAPS G-E03..G-E06, AUTHORED) --------
    double minFiringRpm        = 120.0;
    double crankRevsToFire     = 1.5;
    double stallRpm            = 250.0;
    double dfcoEnterRatio      = 1.5;    ///< EngineSim default 1.2; every shipped engine file sets 1.5
    double dfcoResumeRatio     = 1.2;
    double dfcoAfterStartDelayS = 2.5;
    /// Starter torque at the crank. EngineSim's dataclass default is 60, which
    /// its own notes say stalls against the resolved compression stroke; the
    /// shipped engine files use 200-300, so the default here is in that range.
    double starterStallTorqueNm = 250.0;
    double starterFreeSpeedRpm  = 400.0;

    // --- idle control -------------------------------------------------------
    double iacFeedforward        = 0.008;  ///< fraction of throttleMaxAreaM2
    double iacMin                = 0.001;
    double iacMax                = 0.012;
    double iacKp                 = 0.03;
    double iacKi                 = 0.02;
    double iacResolvedGainScale  = 0.15;
    double iacResolvedKpFraction = 1.0;
    double fastIdleExtraRatio    = 0.5;
    double fastIdleDecayS        = 4.0;
    double idleSparkGainDegPerRpm = 0.12;
    double idleSparkReserveDeg    = 8.0;
    double idleSparkMaxAdvanceDeg = 16.0;
    double idleSparkMaxRetardDeg  = 25.0;

    // --- combustion ---------------------------------------------------------
    double sparkTimingDegBtdc    = 25.0;   ///< used when useMbtTiming is false
    bool   useMbtTiming          = true;
    double wiebeA                = 5.0;    ///< Heywood Eq 9.28
    double wiebeM                = 2.0;
    double combustionDurationDeg = 60.0;
    bool   useTwoZone            = true;
    double burnResidualLo  = 0.14;
    double burnResidualHi  = 0.44;
    double burnLeanLo      = 1.30;
    double burnLeanHi      = 1.75;
    double burnRichLo      = 0.80;
    double burnRichHi      = 0.50;
    double burnMisfireQuality = 0.15;
    double wallTemperatureK   = 400.0;     ///< Woschni, Heywood Eq 12.16-12.19

    // --- knock (Heywood Eq 9.60-9.64) ---------------------------------------
    bool   useKnockModel = false;
    double fuelOctaneRon = 91.0;
    double knockRetardStepDeg         = 3.0;
    double knockRetardMaxDeg          = 15.0;
    double knockRetardRecoveryDegPerS = 1.5;

    // --- turbocharger (optional `turbo:` block) -----------------------------
    bool useTurbo = false;
    ResolvedTurboParameters turbo;

    // --- numerics -----------------------------------------------------------
    /// The cylinder state is integrated on substeps of at most this many crank
    /// degrees (EngineSim's native_crank_angle_resolution_deg).
    double crankAngleResolutionDeg = 1.0;
    /// Cap on substeps x cylinders per millisecond of simulated time
    /// (EngineSim's native_steps_per_ms).
    int stepsPerMs = 600;
    int maxSubstepsPerUpdate = 64;
    /// The engine's own time step [s]. The ECU loops (idle air, spark trim,
    /// wastegate) were tuned in EngineSim at 1-2 ms, far finer than this
    /// simulator's physics step, so the engine sub-steps inside each physics
    /// step at no more than this. NOT an EngineSim field.
    double engineTimestepS = 0.002;

    int intakeValveCount() const  { return std::max(1, (numValvesPerCyl + 1) / 2); }
    int exhaustValveCount() const { return std::max(1, numValvesPerCyl / 2); }

    /// Clamp into ranges the solver is safe in. `numCylinders` is the owning
    /// EngineParameters' cylinder count.
    void validate(int numCylinders, double idleRpm) {
        numValvesPerCyl = (numValvesPerCyl >= 2 && numValvesPerCyl <= 5) ? numValvesPerCyl : 4;

        // Firing order must name each cylinder exactly once; anything else
        // falls back to 1..N rather than leaving a cylinder without a phase.
        bool orderOk = static_cast<int>(firingOrder.size()) == numCylinders;
        if (orderOk) {
            std::vector<char> seen(static_cast<std::size_t>(numCylinders), 0);
            for (int c : firingOrder) {
                if (c < 1 || c > numCylinders || seen[static_cast<std::size_t>(c - 1)]) { orderOk = false; break; }
                seen[static_cast<std::size_t>(c - 1)] = 1;
            }
        }
        if (!orderOk) {
            firingOrder.resize(static_cast<std::size_t>(numCylinders));
            for (int i = 0; i < numCylinders; ++i) firingOrder[static_cast<std::size_t>(i)] = i + 1;
        }
        double sum = 0.0;
        bool intervalsOk = static_cast<int>(firingIntervalsDeg.size()) == numCylinders;
        for (double v : firingIntervalsDeg) { if (!(v > 0.0)) intervalsOk = false; sum += v; }
        if (!intervalsOk || std::abs(sum - 720.0) > 1e-6) firingIntervalsDeg.clear();

        intakeOpenBtdcDeg   = clampd(sanitize(intakeOpenBtdcDeg, 10.0), 0.0, 90.0);
        intakeCloseAbdcDeg  = clampd(sanitize(intakeCloseAbdcDeg, 50.0), 0.0, 90.0);
        exhaustOpenBbdcDeg  = clampd(sanitize(exhaustOpenBbdcDeg, 50.0), 0.0, 90.0);
        exhaustCloseAtdcDeg = clampd(sanitize(exhaustCloseAtdcDeg, 10.0), 0.0, 90.0);
        for (ValveTrainParameters* v : {&intake, &exhaust}) {
            v->maxLiftM       = clampd(sanitize(v->maxLiftM, 9.5e-3), 1.0e-3, 0.03);
            v->rampOpenFrac   = clampd(sanitize(v->rampOpenFrac, 0.5), 0.05, 0.95);
            v->headDiaM       = clampd(sanitize(v->headDiaM, 0.03), 0.01, 0.10);
            v->portDiaM       = clampd(sanitize(v->portDiaM, 0.03), 0.01, 0.10);
            v->seatAngleDeg   = clampd(sanitize(v->seatAngleDeg, 45.0), 0.0, 89.0);
            v->dischargeCoeff = clampd(sanitize(v->dischargeCoeff, 0.67), 0.05, 1.0);
        }
        vvtIntakeMinDeg  = clampd(sanitize(vvtIntakeMinDeg), -60.0, 60.0);
        vvtIntakeMaxDeg  = clampd(sanitize(vvtIntakeMaxDeg), vvtIntakeMinDeg, 60.0);
        vvtExhaustMinDeg = clampd(sanitize(vvtExhaustMinDeg), -60.0, 60.0);
        vvtExhaustMaxDeg = clampd(sanitize(vvtExhaustMaxDeg), vvtExhaustMinDeg, 60.0);
        vvtRateDegPerS   = clampd(sanitize(vvtRateDegPerS, 80.0), 1.0, 1000.0);

        manifoldVolumeM3       = clampd(sanitize(manifoldVolumeM3, 0.003), 2.0e-4, 0.05);
        throttleMaxAreaM2      = clampd(sanitize(throttleMaxAreaM2, 0.003), 1.0e-4, 0.05);
        throttleDischargeCoeff = clampd(sanitize(throttleDischargeCoeff, 0.65), 0.1, 1.0);
        exhaustPlenumVolumeM3  = clampd(sanitize(exhaustPlenumVolumeM3, 0.015), 1.0e-3, 0.2);
        exhaustTailDiaM        = clampd(sanitize(exhaustTailDiaM, 0.045), 0.01, 0.3);
        exhaustTailDischargeCoeff = clampd(sanitize(exhaustTailDischargeCoeff, 0.70), 0.1, 1.0);
        exhaustPostVolumeM3    = clampd(sanitize(exhaustPostVolumeM3, 0.008), 1.0e-3, 0.2);

        stallRpm        = clampd(sanitize(stallRpm, 250.0), 50.0, idleRpm - 50.0);
        minFiringRpm    = clampd(sanitize(minFiringRpm, 120.0), 20.0, stallRpm);
        crankRevsToFire = clampd(sanitize(crankRevsToFire, 1.5), 0.0, 20.0);
        dfcoEnterRatio  = clampd(sanitize(dfcoEnterRatio, 1.5), 1.05, 5.0);
        dfcoResumeRatio = clampd(sanitize(dfcoResumeRatio, 1.2), 1.02, dfcoEnterRatio);
        dfcoAfterStartDelayS = clampd(sanitize(dfcoAfterStartDelayS, 2.5), 0.0, 60.0);
        starterStallTorqueNm = clampd(sanitize(starterStallTorqueNm, 250.0), 0.0, 5000.0);
        starterFreeSpeedRpm  = clampd(sanitize(starterFreeSpeedRpm, 400.0), 50.0, 2000.0);

        iacMin         = clampd(sanitize(iacMin, 0.001), 0.0, 0.2);
        iacMax         = clampd(sanitize(iacMax, 0.012), iacMin, 0.5);
        iacFeedforward = clampd(sanitize(iacFeedforward, 0.008), iacMin, iacMax);
        iacKp = clampd(sanitize(iacKp, 0.03), 0.0, 10.0);
        iacKi = clampd(sanitize(iacKi, 0.02), 0.0, 10.0);
        iacResolvedGainScale  = clampd(sanitize(iacResolvedGainScale, 0.15), 0.0, 10.0);
        iacResolvedKpFraction = clampd(sanitize(iacResolvedKpFraction, 1.0), 0.0, 10.0);
        fastIdleExtraRatio = clampd(sanitize(fastIdleExtraRatio, 0.5), 0.0, 3.0);
        fastIdleDecayS     = clampd(sanitize(fastIdleDecayS, 4.0), 0.1, 120.0);
        idleSparkGainDegPerRpm = clampd(sanitize(idleSparkGainDegPerRpm, 0.12), 0.0, 5.0);
        idleSparkReserveDeg    = clampd(sanitize(idleSparkReserveDeg, 8.0), 0.0, 40.0);
        idleSparkMaxAdvanceDeg = clampd(sanitize(idleSparkMaxAdvanceDeg, 16.0), 0.0, 60.0);
        idleSparkMaxRetardDeg  = clampd(sanitize(idleSparkMaxRetardDeg, 25.0), 0.0, 60.0);

        sparkTimingDegBtdc    = clampd(sanitize(sparkTimingDegBtdc, 25.0), 0.0, 60.0);
        wiebeA                = clampd(sanitize(wiebeA, 5.0), 1.0, 10.0);
        wiebeM                = clampd(sanitize(wiebeM, 2.0), 0.5, 6.0);
        combustionDurationDeg = clampd(sanitize(combustionDurationDeg, 60.0), 10.0, 120.0);
        burnMisfireQuality    = clampd(sanitize(burnMisfireQuality, 0.15), 0.0, 1.0);
        wallTemperatureK      = clampd(sanitize(wallTemperatureK, 400.0), 300.0, 700.0);
        fuelOctaneRon         = clampd(sanitize(fuelOctaneRon, 91.0), 60.0, 130.0);
        knockRetardStepDeg    = clampd(sanitize(knockRetardStepDeg, 3.0), 0.0, 20.0);
        knockRetardMaxDeg     = clampd(sanitize(knockRetardMaxDeg, 15.0), 0.0, 40.0);
        knockRetardRecoveryDegPerS = clampd(sanitize(knockRetardRecoveryDegPerS, 1.5), 0.0, 100.0);

        turbo.count                = (turbo.count >= 1 && turbo.count <= 4) ? turbo.count : 1;
        turbo.targetBoostPa        = clampd(sanitize(turbo.targetBoostPa, 150000.0), 1.05e5, 4.0e5);
        turbo.wastegateGain        = clampd(sanitize(turbo.wastegateGain, 0.5), 0.01, 20.0);
        turbo.inertiaKgM2          = clampd(sanitize(turbo.inertiaKgM2, 1.2e-4), 1.0e-6, 1.0e-2);
        turbo.designSpeedRps       = clampd(sanitize(turbo.designSpeedRps, 2200.0), 100.0, 6000.0);
        turbo.designFlowKgS        = clampd(sanitize(turbo.designFlowKgS, 0.14), 1.0e-3, 5.0);
        turbo.designPressureRatio  = clampd(sanitize(turbo.designPressureRatio, 2.4), 1.1, 6.0);
        turbo.compressorEfficiency = clampd(sanitize(turbo.compressorEfficiency, 0.72), 0.3, 0.9);
        turbo.turbineEfficiency    = clampd(sanitize(turbo.turbineEfficiency, 0.78), 0.3, 0.9);
        turbo.mechanicalEfficiency = clampd(sanitize(turbo.mechanicalEfficiency, 0.95), 0.5, 1.0);
        turbo.turbineAreaM2        = clampd(sanitize(turbo.turbineAreaM2, 5.0e-4), 1.0e-5, 1.0e-2);
        turbo.wastegateAreaM2      = clampd(sanitize(turbo.wastegateAreaM2, 3.5e-4), 0.0, 1.0e-2);
        turbo.boostVolumeM3        = clampd(sanitize(turbo.boostVolumeM3, 8.0e-3), 5.0e-4, 0.1);
        turbo.intercoolerEffectiveness = clampd(sanitize(turbo.intercoolerEffectiveness, 0.7), 0.0, 0.95);

        crankAngleResolutionDeg = clampd(sanitize(crankAngleResolutionDeg, 1.0), 0.25, 12.0);
        stepsPerMs              = (stepsPerMs >= 8 && stepsPerMs <= 100000) ? stepsPerMs : 600;
        maxSubstepsPerUpdate    = (maxSubstepsPerUpdate >= 1 && maxSubstepsPerUpdate <= 4096)
                                ? maxSubstepsPerUpdate : 64;
        engineTimestepS         = clampd(sanitize(engineTimestepS, 0.002), 1.0e-4, 0.02);
    }
};

} // namespace vsim
