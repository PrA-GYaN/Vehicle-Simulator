#pragma once
//===========================================================================
//  ResolvedEngine.h  --  the crank-angle-resolved engine of EngineSim v0.7,
//  running natively in this simulator.
//
//  SOURCE: ../EngineSim (Heywood, "Internal Combustion Engine Fundamentals",
//  via ../EngineSim/ENGINE_KB.md). NOT part of this project's S1/S2/S3
//  vehicle-dynamics Knowledge Base - see config/EngineParameters.h.
//
//  WHAT IT IS
//  ------------------------------------------------------------------------
//  Every cylinder carries its own gas (mass, temperature, burned fraction,
//  fuel) round the 720 deg cycle at its own phase. Valves, an intake plenum,
//  an exhaust plenum and - if fitted - a turbine, compressor and charge-air
//  volume connect them. Torque, manifold pressure, volumetric efficiency,
//  pumping loss, wall heat loss, exhaust back-pressure, boost and turbo lag
//  are all OUTPUTS of that integration; none is a curve or a time constant.
//  That is the difference from physics/Engine.cpp's cycle-averaged model,
//  where each of those is an authored function.
//
//  HOW IT IS BUILT - two layers, kept deliberately separate:
//
//    * The cylinder physics is NOT re-implemented. EngineSim already has it
//      in C (engine_sim/native/enginecore.c, its "compiled core", proven
//      bit-identical to its Python reference by EngineSim's own parity
//      tests). That file is vendored unchanged into physics/enginecore/ and
//      compiled into this project. One call, es_advance(), runs the substep
//      loop for one engine step.
//
//    * Everything EngineSim does in Python AROUND that loop is transcribed
//      here, function for function:
//          engine_sim/cylinder.py       Cylinder.__init__, geometry tables,
//                                       initialize_charge, load_fuel, cam phase
//          engine_sim/cylinder_bank.py  CylinderBank (reset, advance, the
//                                       learned speed-density air model, prime)
//          engine_sim/turbo_system.py   Turbocharger (shaft, wastegate law)
//          engine_sim/engine.py         Engine.update - the resolved branch:
//                                       run state, DFCO, fuel-cut limiter,
//                                       idle air + spark trim, MBT spark,
//                                       knock retard, VVT, friction, starter,
//                                       crank dynamics
//      Same statement order and the same arithmetic, so the two agree to
//      rounding; tools/engine_parity checks that against a trace recorded
//      from EngineSim itself.
//
//  Unlike EngineSim, the cylinders' state lives directly in the flat arrays
//  the C core works on - there are no per-cylinder objects to copy in and out
//  on every step, which is where EngineSim's own profile says a third of its
//  compiled step goes.
//
//  NOT CARRIED OVER (all optional and off in every shipped EngineSim engine
//  file): fuel film, closed-loop lambda trim, random cyclic variability,
//  the component friction breakdown, CO/HC emissions, diesel, the
//  torque-clamp limiter and the pure-Python cylinder path. The limiter here
//  is always EngineSim's "fuel_cut" type.
//
//  TWO WAYS TO DRIVE IT, matching Engine.h:
//    * update()         the engine integrates its own speed against a load
//                       (clutch slipping or released).
//    * updateAtSpeed()  the crank is held at a speed imposed from outside
//                       (clutch locked: wheel speed x ratio). Everything else
//                       - manifold, turbo, ECU - still evolves.
//===========================================================================

#include <array>
#include <vector>

#include "config/EngineParameters.h"

namespace vsim {

class ResolvedEngine {
public:
    struct Input {
        double throttle     = 0.0;    ///< [0,1]
        double loadTorqueNm = 0.0;    ///< signed; negative = the driveline motoring the engine
        bool   ignition     = true;   ///< false = no spark/fuel, the engine only motors
        bool   starter      = false;  ///< starter motor engaged
    };

    /// Everything known after one engine step. Torques are for the whole
    /// engine, averaged over the crank angle the step swept.
    struct Output {
        double rpm = 0.0;
        double omega = 0.0;                 ///< [rad/s] after the step
        double crankAngleDeg = 0.0;         ///< [0,720)

        // torque balance [N.m]
        double grossTorqueNm     = 0.0;     ///< gross indicated (excl. the pumping loop)
        double pumpingTorqueNm   = 0.0;     ///< positive = loss; negative on a boosted intake
        double frictionTorqueNm  = 0.0;     ///< rubbing friction, Heywood Eq 13.5/13.6
        double accessoryTorqueNm = 0.0;
        double starterTorqueNm   = 0.0;
        double loadTorqueNm      = 0.0;
        double brakeTorqueNm     = 0.0;     ///< gross - pumping - friction - accessory: at the flywheel
        double netTorqueNm       = 0.0;     ///< brake + starter - load: what accelerates the crank

        // air and fuel
        double manifoldPressurePa   = 0.0;
        double mapFilteredPa        = 0.0;  ///< what the ECU meters fuel against
        double intakeTemperatureK   = 0.0;  ///< charge temperature (after the intercooler)
        double airEstimateKg        = 0.0;  ///< ECU's belief, per cylinder per cycle
        double trappedAirKg         = 0.0;  ///< what the cylinders actually trapped
        double fuelPerCycleKg       = 0.0;  ///< per cylinder; 0 under fuel cut
        double lambda               = 1.0;
        double volumetricEfficiency = 0.0;  ///< measured against manifold density
        double airMassFlowKgS       = 0.0;
        double iacPosition          = 0.0;  ///< idle-air valve, fraction of WOT throttle area

        // combustion
        double sparkTimingDegBtdc    = 0.0;
        double combustionDurationDeg = 0.0;
        double imepNetBar            = 0.0;
        double pmepBar               = 0.0;
        double peakPressurePa        = 0.0;
        double thermalEfficiency     = 0.0;  ///< net indicated work / fuel energy, last cycle
        double residualFraction      = 0.0;
        double burnQuality           = 1.0;
        double misfireRate           = 0.0;
        double knockRetardDeg        = 0.0;
        double heatLossW             = 0.0;
        double vvtIntakeDeg          = 0.0;
        double vvtExhaustDeg         = 0.0;

        // exhaust and turbo
        double exhaustPressurePa     = 0.0;  ///< plenum: what the exhaust valves see
        double exhaustTemperatureK   = 0.0;
        double boostPressurePa       = 0.0;  ///< ABSOLUTE charge pressure upstream of the throttle
        double turboSpeedRps         = 0.0;
        double wastegatePosition     = 0.0;
        double turbinePowerW         = 0.0;

        // run state
        bool firing     = false;   ///< ECU synced and above firing speed
        bool combusting = false;   ///< firing and not in fuel cut
        bool fuelCut    = false;   ///< DFCO or the rev limiter
        bool limiterActive = false;
        bool starter    = false;   ///< the starter motor was engaged this step
        int  substeps   = 0;       ///< crank-angle substeps the last step took
    };

    explicit ResolvedEngine(const EngineParameters& params);

    /// Warm start: running at idle speed, a few fired cycles already behind it.
    void start();
    /// Stopped and cold - needs the starter.
    void reset();

    const Output& update(const Input& input, double dt);
    const Output& updateAtSpeed(double omega, const Input& input, double dt);

    double omega() const { return m_omega; }
    const Output& lastOutput() const { return m_out; }
    const EngineParameters& parameters() const { return m_p; }
    int cylinderCount() const { return m_n; }

    /// Cylinder 1's state where its exhaust valve opens - the blowdown pulse
    /// the sound synthesis is made from. Pressure from the recorded pressure
    /// trace (one bin per 4 deg), temperature from pV = mRT with the mass
    /// trapped at intake-valve closing.
    void blowdownState(double& pressurePa, double& temperatureK) const;

    /// Cylinder 1's pressure against its local crank angle, 180 bins of 4 deg
    /// (0 = TDC at the start of the intake stroke, 360 = firing TDC) [Pa].
    const std::vector<double>& pressureTrace() const { return m_trace; }

    /// One cylinder's instantaneous pressure [Pa]; `index` is 0-based.
    double cylinderPressurePa(int index) const;

private:
    const Output& step(const Input& input, double dt, bool imposed, double imposedOmega);

    // --- engine_sim/cylinder.py ---------------------------------------------
    void buildGeometry();
    double exactVolume(double angleDeg) const;
    bool intakeIsOpen(double angleDeg) const;
    void initCylinder(int index, double localAngleDeg);
    void setCamPhase(double intakeAdvanceDeg, double exhaustAdvanceDeg);

    // --- engine_sim/cylinder_bank.py ----------------------------------------
    void resetBank(double thetaDeg);
    void advanceBank(double thetaDeg, double dthetaDeg, double dt, double rpm,
                     double manifoldPressure, double intakeTemperature,
                     double throttleArea, double fuelPerCycle, double airRef,
                     double sparkBtdc, double burnDeg, bool holdManifold);
    double airEstimate(double pManifold, double tManifold, double rpm) const;
    void learnAirModel(double pManifold, double tManifold, double rpm, double trappedAirKg);
    void refreshCycleSummary();
    void primeBank(double thetaDeg, double rpm, double manifoldPressure,
                   double intakeTemperature, double sparkBtdc, double burnDeg, int cycles);

    // --- engine_sim/turbo_system.py -----------------------------------------
    void resetTurbo();
    double chargeTemperature(double pressureRatio) const;
    void advanceShaft(double turbineEnergyJ, double compressorEnergyJ);
    void controlWastegate(double dt);

    // --- engine_sim/engine.py -----------------------------------------------
    void resetEcu(bool running);
    double idleTargetOmega() const;
    double idleAirControl(double throttle, double omega, bool fireOk, double dt);
    void updateVvt(double rpm, double load, double dt);

    double* cyl(int index) { return &m_cs[static_cast<std::size_t>(index) * m_ncs]; }
    const double* cyl(int index) const { return &m_cs[static_cast<std::size_t>(index) * m_ncs]; }

    EngineParameters m_p;
    int m_n = 0;                 ///< cylinders
    std::size_t m_ncs = 0;       ///< doubles per cylinder in m_cs

    // geometry (shared by every cylinder)
    double m_clearanceVolume = 0.0;
    double m_displacedVolume = 0.0;
    double m_pistonArea = 0.0;
    double m_crankR = 0.0;
    double m_rodRatioSq = 0.0;
    double m_intakeOpenDeg = 0.0, m_intakeDurationDeg = 0.0;
    double m_exhaustOpenDeg = 0.0, m_exhaustDurationDeg = 0.0;
    double m_ivcBaseDeg = 0.0;
    std::vector<double> m_vol, m_areaIn, m_areaEx;
    std::vector<double> m_fire;  ///< firing TDC of each cylinder in the cycle [deg]

    // the arrays the C core works on
    std::vector<double> m_cs, m_trace, m_loc, m_beginK, m_K, m_B;

    // bank state
    double m_cycleStartDeg = 0.0;
    double m_camIn = 0.0, m_camEx = 0.0;
    double m_resDeg = 1.0;
    int    m_stepBudget = 600;
    double m_mapFiltered = 0.0, m_mapSlow = 0.0;
    double m_veMeasured = 0.0;
    std::array<double, 3> m_theta{};                  ///< learned air model: a0, b, a1
    std::array<std::array<double, 3>, 3> m_cov{};
    double m_refX = -1.0, m_refN = -1.0;
    std::vector<double> m_ivcSeen, m_w0, m_p0, m_h0;
    int m_summaryDue = 0;
    // exhaust plenum (+ post-turbine volume)
    double m_exP = 0.0, m_exT = 0.0, m_exM = 0.0;
    double m_postP = 0.0, m_postT = 0.0, m_postM = 0.0;
    double m_exVolume = 0.0, m_postVolume = 0.0;
    double m_tailFlow = 0.0, m_turbFlow = 0.0, m_wgFlow = 0.0, m_turbPower = 0.0;
    // bank outputs of the last advance
    double m_grossTorque = 0.0, m_pumpingTorque = 0.0, m_indicatedTorque = 0.0;
    double m_airMassFlow = 0.0, m_throttleMassFlow = 0.0, m_heatLossW = 0.0;
    double m_bankManifoldPressure = 0.0;
    int    m_substepsLast = 0;
    // per-cycle summary
    double m_trappedAirKg = 0.0, m_cycleWorkJ = 0.0, m_cyclePumpingWorkJ = 0.0;
    double m_peakPressurePa = 0.0;
    double m_knockIntegral = 0.0;
    bool   m_knockDetected = false;
    long   m_knockEvents = 0;
    double m_residualFraction = 0.05, m_burnQuality = 1.0, m_misfireRate = 0.0;
    long   m_misfireTotal = 0, m_cyclesTotal = 0;

    // turbocharger
    bool   m_turbo = false;
    double m_tInertia = 0.0, m_tDesignOmega = 1.0, m_tDesignFlow = 0.0, m_tDesignRatio = 1.0;
    double m_tEtaC = 1.0, m_tEtaM = 1.0, m_tVolume = 1.0, m_tIntercooler = 0.0;
    double m_tTargetBoost = 0.0, m_tGain = 1.0, m_tBypassArea = 0.0, m_tBovArea = 0.0;
    double m_tOmega = 0.0, m_boostPressure = 0.0, m_boostTemperature = 0.0;
    double m_wastegatePosition = 0.0, m_wgIntegral = 0.0;

    // engine / ECU state
    double m_omega = 0.0;        ///< [rad/s]
    double m_thetaRad = 0.0;     ///< crank angle [rad], 0..4 pi
    double m_manifoldPressure = 0.0;
    bool   m_dfcoActive = false, m_limiterCut = false;
    bool   m_synced = false, m_wasFiring = false;
    double m_syncRevs = 0.0, m_fireTime = 0.0, m_dfcoClock = 0.0, m_starterRecent = 0.0;
    double m_iacIntegral = 0.0;
    double m_vvtIn = 0.0, m_vvtEx = 0.0;
    double m_knockRetardDeg = 0.0;
    long   m_knockEventsSeen = 0;

    Output m_out;
};

} // namespace vsim
