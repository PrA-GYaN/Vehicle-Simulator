#pragma once
//===========================================================================
//  VehicleParameters.h  --  static description of a vehicle.
//
//  These values come from config/vehicles/*.yaml. Nothing here is hardcoded
//  in C++ except the fallback defaults, which exist only so a missing or
//  partial config file degrades to something drivable instead of dividing
//  by zero.
//
//  All values are in SI units (metres, kilograms, seconds, RADIANS) by the
//  time they reach this struct. The YAML files express angles in degrees for
//  human readability; Config.cpp converts on load.
//===========================================================================

#include <array>
#include <string>
#include <vector>

#include "config/EngineParameters.h"
#include "config/GearboxParameters.h"
#include "config/SuspensionParameters.h"
#include "config/TireCompound.h"
#include "config/TireParameters.h"
#include "core/Units.h"
#include "core/WheelState.h"

namespace vsim {

/// How the vehicle is drawn.
///
/// Entirely a rendering concern - nothing here is read by the physics. The
/// default is the GENERATED car (src/rendering/CarMesh.h), whose every
/// dimension comes from the parameters below, so it always matches the
/// vehicle being simulated. An imported .glb can be substituted for it; see
/// src/rendering/GltfModel.h for what that path does and does not support.
struct VehicleModelConfig {
    /// Body shape family for the generated model: "auto", "race", "sports",
    /// "saloon" or "suv". "auto" picks one from the vehicle's own
    /// proportions - see resolveCarBodyStyle() in rendering/CarMesh.h. Only
    /// the silhouette changes; the wheelbase, track, overall dimensions and
    /// wheel size are taken from this struct's owner either way.
    std::string bodyStyle = "auto";

    /// Path to a .glb file, relative to the working directory or the
    /// executable (resolved the same way as everything else - see
    /// Config::resolveResourcePath). Empty (the default) = draw the
    /// generated car instead.
    std::string path;

    /// Which glTF axis is "forward" / "up" in the file's own coordinate
    /// system. The third axis is derived - see rendering/GltfModel.h.
    std::string forwardAxis = "+z";
    std::string upAxis      = "+y";

    /// Scene-graph node names for the four wheel hub assemblies. Everything
    /// at or below each name is treated as that wheel's geometry and
    /// re-positioned every frame by the physics-derived wheel pose instead
    /// of being drawn as static chassis geometry. Leave a name empty if the
    /// model has no separately-animatable wheel for that position - it will
    /// fall back to the procedural wheel mesh.
    std::string frontLeftNode;
    std::string frontRightNode;
    std::string rearLeftNode;
    std::string rearRightNode;

    bool hasModel() const { return !path.empty(); }
};

/// A point in the vehicle-local frame (+X forward, +Y up, +Z right, origin
/// on the ground under the REAR AXLE - core/Coordinates.h), with a flag for
/// whether the YAML actually gave one. Unset points are derived from the
/// vehicle's dimensions by whoever reads them.
struct LocalPoint {
    std::array<double, 3> xyz{0.0, 0.0, 0.0};
    bool set = false;
};

/// Where the vehicle-mounted cameras sit (rendering/Camera.h). Purely a
/// presentation concern, like VehicleModelConfig. Worth authoring for an
/// imported model - a default derived from length/height can land inside
/// real bodywork the procedural car does not have.
struct VehicleCameraConfig {
    LocalPoint driverEye;   ///< cockpit view: the driver's eyes
    LocalPoint bonnet;      ///< bonnet/hood view: just above the scuttle
    LocalPoint bumper;      ///< bumper view: nose, near the ground
    LocalPoint wheel;       ///< wheel view: on the flank, looking forward past the front wheel
};

/// How the vehicle SOUNDS - read only by src/audio/, never by the physics.
///
/// What is and is not derived: the pulses themselves come from the engine
/// model (firing events on the crank angle the engine turns, blowdown
/// pressure and temperature from its own Otto cycle - see Engine.h), the
/// pipe resonance from the speed of sound in the exhaust gas, the turbo
/// whine from the compressor work balance. What is AUTHORED is hardware the
/// engine model does not describe: firing order, pipe lengths, muffler,
/// turbo wheel size, gear tooth count. Every default below produces a
/// plausible generic engine, so a vehicle needs none of this block to be
/// heard.
/// The engine's sound hardware as EngineSim describes it - the `[exhaust]`,
/// `[intake]` and `[sound]` sections of an EngineSim engine file, under
/// EngineSim's own field names (engine_sim/config.py). Opted into by a
/// `sound: engine_sim:` block; with it, the engine, exhaust and intake are
/// synthesised by audio/EngineSimSynth - a transcription of EngineSim's own
/// synthesiser - instead of by the model described below. Needs a resolved
/// engine (it takes firing order and valve events from there).
///
/// Lists are per exhaust pipe: one value applies to every pipe.
struct EngineSimSoundConfig {
    // --- [exhaust] ---------------------------------------------------------
    /// "dual": one pipe per cylinder bank (stereo, and on a cross-plane V8
    /// uneven per bank - the burble). "single": all banks merge.
    bool dualExhaust = false;
    std::vector<double> headerLengthM  = {0.55};   ///< primaries + collector [m]
    std::vector<double> midLengthM     = {1.10};
    std::vector<double> mufflerLengthM = {0.40};
    std::vector<double> tailLengthM    = {0.90};
    double mufflerAreaRatio = 5.0;                 ///< chamber / pipe cross-section
    std::vector<double> mufflerBodyHz  = {125.0};  ///< boom of the muffler shell
    /// Gas temperature in each section [K]; speed of sound c = 20.05 sqrt(T).
    double headerTempK  = 725.0;
    double midTempK     = 622.0;
    double mufflerTempK = 526.0;
    double tailTempK    = 439.0;
    double pipeLoss   = 0.992;                     ///< wall friction: gain per pass
    double tipDamping = 0.35;                      ///< open-end high-frequency loss (0..1)

    // --- [layout] ----------------------------------------------------------
    /// Bank of each cylinder, cylinders 1..N. Empty = all on bank 0.
    std::vector<int> cylinderBank;

    // --- [intake] ----------------------------------------------------------
    double intakeRunnerLengthM = 0.45;
    double intakeRunnerTempK   = 293.0;

    // --- [sound]: the only free "taste" parameters ---------------------------
    double cylinderStrengthSpread   = 0.30;   ///< fixed strength differences between cylinders (lope)
    double cylinderTimingSpreadDeg  = 6.0;    ///< fixed exhaust-timing differences
    int    noiseSeed                = 1234567;
    double blowdownK                = 1.75;   ///< blowdown time constant = k bore / c(header)
    double crankRippleGain          = 0.66;   ///< share of cylinder work seen as crank-speed ripple
    double combustionVariability    = 0.02;   ///< random cycle-to-cycle strength variation
    double idleVariability          = 0.05;   ///< extra of it at closed throttle
    double popProbability           = 0.03;   ///< chance of an overrun pop per exhaust event
    double starterRatio             = 11.0;   ///< starter armature revs per crank rev
    int    starterPoles             = 4;

    // --- the listener's dials (EngineSim README 6.6) -------------------------
    // lf / hf in dB, the rest 0..1. Defaults are EngineSim's "Stock".
    // (double, not float: the synthesiser is checked sample-for-sample
    // against EngineSim's, and 0.35f is not 0.35.)
    struct Profile {
        double vol = 0.7, conv = 0.35, comp = 0.6, drive = 0.0;
        double lf = 0.0, hf = 0.0, ring = 0.6, rasp = 0.5;
        double lope = 0.5, nlf = 0.0, nhf = 0.0, tick = 0.0, intk = 0.0, pops = 0.5;
    } profile;
};

struct VehicleSoundConfig {
    /// Crank angle of each cylinder's firing within the 720 deg cycle, in
    /// firing order [deg]. Empty = evenly spaced (720/n), which is right for
    /// an inline-4, a flat-plane V8 and most sixes; a cross-plane V8 is not
    /// even PER BANK, which is what gives it its burble - see
    /// `firingBanks`.
    std::vector<double> firingAnglesDeg;

    /// Which exhaust bank each firing event (same order as above) exits
    /// into. Empty = everything into one collector.
    std::vector<int> firingBanks;

    /// Primary-to-tailpipe length per bank [m]. Sets the pipe's quarter-wave
    /// resonances f = (2k-1) c / 4L, with c from the exhaust gas temperature.
    double exhaustPipeLength = 2.4;

    /// Fraction of the blowdown temperature T_4 the gas still has as it
    /// fills the pipe (it cools against the manifold walls). AUTHORED.
    double exhaustGasTemperatureFraction = 0.65;

    /// Crank angle over which the blowdown pulse leaves the cylinder [deg].
    /// Heywood Sec 6.5: the exhaust valve opens 40-60 deg before BDC and the
    /// blowdown phase is over by shortly after - so a pulse is SHORTER in
    /// time at higher rpm, which is what makes a revving engine brighter.
    double blowdownCrankDeg = 60.0;

    /// Muffler/silencer low-pass corner [Hz]. AUTHORED.
    double mufflerCutoffHz = 900.0;

    /// Cycle-to-cycle variation of each pulse's strength [-]. Heywood
    /// Ch.9: coefficient of variation of imep is a few percent in a healthy
    /// SI engine. AUTHORED at the typical figure.
    double cyclicVariation = 0.03;

    /// Turbo compressor wheel: tip radius [m] and main blade count. Shaft
    /// speed comes from Euler's work equation (see audio/SoundSynth.cpp);
    /// the whine is at shaft speed x blades.
    double turboTipRadius = 0.025;
    int    turboBlades = 6;

    /// Teeth on the gearbox input gear - the whine sits at shaft
    /// frequency x teeth. AUTHORED.
    int    gearTeeth = 23;

    /// Fixed cylinder-to-cylinder spread in pulse strength [-] - unequal
    /// charge distribution between cylinders (Heywood Sec 7.6). Unlike the
    /// cycle-to-cycle variation above it repeats every cycle, so it puts
    /// energy at HALF engine orders - the lumpy undertone of a real engine.
    /// AUTHORED.
    double cylinderVariation = 0.06;

    /// Reciprocating mass per cylinder (piston, pin, rings, small end of
    /// the rod) [kg]. With the stroke, rod length and firing angles this
    /// gives the second-order shaking force - see audio/SoundSynth.cpp.
    /// AUTHORED.
    double reciprocatingMass = 0.45;

    /// Angle between the two banks of a V engine [deg]. Only used when
    /// `firingBanks` names two banks.
    double veeAngleDeg = 90.0;

    /// Intake air box as a Helmholtz resonator: f = c/(2 pi) sqrt(A / (V L))
    /// with A, L the snorkel's area and length and V the box volume - the
    /// low "induction roar". AUTHORED hardware.
    double airboxVolume = 0.010;     ///< [m^3]
    double snorkelLength = 0.25;     ///< [m]
    double snorkelRadius = 0.035;    ///< [m]

    /// Engine position along the car (vehicle-local x of the block) [m].
    /// Unset = midway along the wheelbase. Used to decide whether the
    /// engine is in front of or behind the driver in the cockpit mix.
    LocalPoint engine;
    /// Tailpipe position (vehicle-local) [m]. Unset = the car's tail.
    LocalPoint exhaust;

    /// EngineSim's engine sound instead of the model above - see
    /// EngineSimSoundConfig. Turbo, gearbox, tyres, wind and the listener
    /// stay this project's either way.
    bool useEngineSim = false;
    EngineSimSoundConfig engineSim;
};

/// Which quantities the runtime UI is allowed to touch.
///
/// Exposed so the UI can label each control honestly instead of silently
/// letting the user change something that will not take effect.
enum class ParamMutability {
    Live,            ///< safe to change while the simulation is running
    RestartRequired, ///< only read at startup / on reset
    Derived          ///< computed from other parameters; read-only
};

struct VehicleParameters {
    std::string name = "Default";

    // --- mass ---------------------------------------------------------
    double mass = 1500.0;                 ///< [kg]

    // --- body dimensions (visual + reference geometry) ----------------
    double length = 4.5;                  ///< [m] bumper to bumper
    double width  = 1.8;                  ///< [m] excluding mirrors
    double height = 1.4;                  ///< [m] ground to roof

    // --- chassis geometry ---------------------------------------------
    double wheelbase  = 2.7;              ///< [m] front axle to rear axle
    double frontTrack = 1.55;             ///< [m] lateral wheel spacing, front
    double rearTrack  = 1.55;             ///< [m] lateral wheel spacing, rear

    // --- centre of gravity --------------------------------------------
    double cgHeight = 0.55;               ///< [m] above ground

    /// Fraction of static weight carried by the FRONT axle, 0..1.
    ///
    /// Note the lever-arm relationship this implies: the front axle carries
    /// more weight when the CG is closer to the front, i.e. when the distance
    /// from the front axle to the CG is small. Hence the derivation below.
    double frontWeightDistribution = 0.55;

    // --- steering ------------------------------------------------------
    double maxSteeringAngle = degToRad(35.0);  ///< [rad] at the road wheel
    double steeringRate     = degToRad(180.0); ///< [rad/s] actuator slew rate

    // --- longitudinal limits -------------------------------------------
    double maxAcceleration = 4.0;         ///< [m/s^2]
    double maxBraking      = 8.0;         ///< [m/s^2] magnitude, positive
    double maxSpeed        = 60.0;        ///< [m/s]

    // --- detailed engine + gearbox (optional) ---------------------------
    //
    // `maxAcceleration` above is the KB-honest default: a force demand, not
    // an outcome, because neither Gillespie/Blundell&Harty/Milliken gives an
    // engine torque map (KB gap G17, OPEN in VEHICLE_DYNAMICS_KNOWLEDGE_BASE.md).
    //
    // `detailedEngine`/`gearbox` are the documented exception: an engine
    // torque map sourced from Heywood via ../EngineSim, NOT from this
    // project's own Knowledge Base — see config/EngineParameters.h and
    // config/GearboxParameters.h for the full source notice, and
    // physics/Powertrain.h for exactly how it drives the vehicle and what it
    // cannot do (no clutch/torque-converter model, so no launch from rest).
    //
    // A vehicle opts in by setting `useDetailedEngine = true` (Config.cpp
    // only sets this when the YAML actually has an `engine: detailed:`
    // block); every vehicle that does not is completely unaffected, still
    // reading only `maxAcceleration` exactly as before this existed.
    bool useDetailedEngine = false;
    EngineParameters detailedEngine;
    GearboxParameters gearbox;

    bool hasDetailedEngine() const { return useDetailedEngine; }

    // --- road load (resistance to motion) -------------------------------
    //
    // These are what make a coasting vehicle slow down. See physics/RoadLoad.h.

    /// Base rolling resistance coefficient [-], at low speed.
    /// Gillespie Ch.4 table: 0.015 passenger car on concrete, 0.08 on
    /// medium-hard soil, 0.30 on sand; 0.012 for a heavy truck on concrete.
    double rollingResistanceCoeff = 0.015;

    /// Aerodynamic drag coefficient [-]. Only meaningful together with
    /// `frontalArea` - a C_D quoted against a different reference area is a
    /// different number. Gillespie Ch.4 quotes 0.85 / 0.50 / 0.45 / 0.32 for
    /// cars across eras.
    double dragCoefficient = 0.32;

    /// Frontal (projected) area [m^2]. The reference area for `dragCoefficient`.
    double frontalArea = 2.2;

    /// Engine braking, as a deceleration per unit speed [1/s].
    ///
    /// NOT from Gillespie - see the note in physics/RoadLoad.cpp. A stand-in
    /// for overrun losses in gear, which rise with engine RPM and therefore
    /// (in a fixed gear) with road speed.
    double engineBrakingCoeff = 0.012;

    // --- wheels ---------------------------------------------------------

    /// UNLOADED tire radius R_u [m] (KB Section 2.15).
    ///
    /// No longer visual-only. KB Section 2.15 defines three radii that are
    /// never equal - unloaded R_u, loaded R_l, and effective rolling R_e - and
    /// the tire model derives the other two from this one and the load.
    double wheelRadius = 0.34;
    double wheelWidth  = 0.22;            ///< [m] visual only

    /// Rotational inertia of one wheel-and-tire assembly I_w [kg.m^2].
    ///
    /// NOT QUANTIFIED BY THE KNOWLEDGE BASE. KB Eq. 3-18
    ///     F_b = (T_b - I_w * alpha_w) / r
    /// names I_w and KB Section 5.4 lists it among the driveline inertias, but
    /// no source gives a value or a radius of gyration for a road wheel. KB
    /// Section 1.6 gives unsprung WEIGHT per axle (10 % of GAWR non-driven,
    /// 15 % driven) which is a mass, not an inertia.
    ///
    /// The parameter cannot be avoided: wheel lock and wheel spin ARE the
    /// rotational dynamics of the wheel, and KB Section 6.7 says explicitly
    /// that during lockup "I_w * alpha_w is large and Eq. 3-18 is required".
    /// The value below is authored data with no KB basis.
    /// See README -> Open Questions.
    double wheelInertia = 1.2;

    // --- inertia --------------------------------------------------------

    /// Yaw moment of inertia I_zz about the CG [kg.m^2].
    ///
    /// NOT QUANTIFIED BY THE KNOWLEDGE BASE. KB Section 1.11 defines the
    /// inertia tensor and the radius of gyration (I = m*k^2), and KB Eq. 1-2
    /// and Eq. 2.236 give the rotational equation of motion that needs it, but
    /// neither source gives a yaw radius of gyration for any vehicle. KB
    /// Section 12.1 lists only the PITCH radius of gyration, and even that
    /// with no typical value.
    ///
    /// Zero means "derive a fallback": m * b * c, i.e. k_z^2 = b*c. That form
    /// is borrowed from the Dynamic Index DI = k^2/(b*c) of KB Eq. 5-40, where
    /// DI = 1.0 is called ideal - but Eq. 5-40 is about the PITCH plane and
    /// the KB says nothing of the sort about yaw. Treat the fallback as
    /// authored data. See README -> Open Questions.
    double yawInertia = 0.0;

    // --- suspension -------------------------------------------------------

    /// Ride and roll rates, roll centres, anti-roll bars and unsprung mass.
    ///
    /// This replaces what used to be a single authored `roll stiffness front
    /// fraction`. [S3] supplies the whole chain from ride frequency to the
    /// three-term lateral load transfer, so the front/rear split is now
    /// COMPUTED - and, per KB §3.5.2, it is not the same number as the roll
    /// rate distribution. See physics/SuspensionModel.h.
    SuspensionParameters suspension;

    // --- brakes ----------------------------------------------------------

    /// Fraction of total brake force sent to the FRONT axle, 0..1.
    ///
    /// KB Section 6.8.2: a fixed hydraulic system produces a STRAIGHT LINE
    /// through the origin on the front-force / rear-force diagram, so front
    /// force is a fixed proportion of total. A straight line cannot follow the
    /// curved ideal proportioning locus, so a fixed proportioning "can be
    /// ideal at exactly one value of mu and is suboptimal everywhere else".
    ///
    /// Zero means "derive from the KB's ideal proportioning". KB Eq. 3-29/3-30
    /// give the forces that lock both axles simultaneously at a deceleration
    /// of mu g:
    ///     F_xfi = mu*(W_fs + mu*W*h/L)
    ///     F_xri = mu*(W_rs - mu*W*h/L)
    /// whose front share is  W_fs/W + mu*h/L.  That is the value used, taken
    /// at the tire's peak friction - i.e. the brakes are proportioned ideally
    /// for the dry surface they were designed on, and behave exactly as KB
    /// Section 6.8.2 predicts on any other.
    double brakeProportionFront = 0.0;

    // --- drivetrain -------------------------------------------------------

    /// Which axle is driven. KB Section 5.7.3 derives the traction limit
    /// separately for front and rear drive; all-wheel drive is not covered by
    /// either source (gap G19, no differential or torque-split model).
    DriveLayout driveLayout = DriveLayout::RearWheelDrive;

    // --- tires ------------------------------------------------------------

    /// The tire. See config/TireParameters.h - every field there is traced to
    /// the Knowledge Base, and the three that the KB does not quantify are
    /// marked as such.
    TireParameters tire;

    /// Which compound that tyre is run as. Standard (the default) is the
    /// tyre exactly as described above, with no wear; Soft / Medium / Hard
    /// scale its grip and give it a life - see config/TireCompound.h, and
    /// note that those numbers are authored, not from the Knowledge Base.
    /// Changed at runtime through Vehicle::setTireCompound().
    TireCompound tireCompound = TireCompound::Standard;

    /// Traction control level the vehicle starts with: 0 off, 1 (least
    /// intervention) to 3 (most). A driver aid, not a property of the car's
    /// mechanics - see Vehicle::setTractionControl().
    int tractionControl = 0;

    // --- visual model -------------------------------------------------

    /// Imported 3D model, or empty for the procedural body/wheel boxes.
    /// Purely a rendering concern - see VehicleModelConfig above.
    VehicleModelConfig model;

    /// Vehicle-mounted camera positions and how it sounds - both
    /// presentation-only; see the structs above.
    VehicleCameraConfig cameras;
    VehicleSoundConfig  sound;

    //-------------------------------------------------------------------
    //  DERIVED QUANTITIES
    //
    //  Never stored, always computed, so they cannot drift out of sync with
    //  the values they depend on.
    //-------------------------------------------------------------------

    /// Longitudinal distance from the FRONT axle to the CG [m].
    ///
    /// A front weight distribution of 0.55 means the front axle carries 55%
    /// of the load, which requires the CG to sit 45% of the wheelbase behind
    /// the front axle. The complement is intentional, not a typo.
    double frontAxleToCg() const {
        return wheelbase * (1.0 - frontWeightDistribution);
    }

    /// Longitudinal distance from the CG to the REAR axle [m].
    double rearAxleToCg() const {
        return wheelbase * frontWeightDistribution;
    }

    /// Static load on each axle [N], level ground, at rest.
    double staticFrontAxleLoad() const {
        return mass * kGravity * frontWeightDistribution;
    }
    double staticRearAxleLoad() const {
        return mass * kGravity * (1.0 - frontWeightDistribution);
    }

    /// Static vertical load on ONE wheel [N], level ground, at rest.
    ///
    /// KB Eq. 1-6/1-7 give the axle loads; the KB's own lumped treatment puts
    /// half of an axle's load on each of its wheels (as in Eq. 2-21, which
    /// writes the rear wheel static term as W*b/(2L)).
    double staticWheelLoad(int wheel) const {
        const bool front = (wheel == FrontLeft || wheel == FrontRight);
        return 0.5 * (front ? staticFrontAxleLoad() : staticRearAxleLoad());
    }

    /// Yaw moment of inertia actually used [kg.m^2] - authored, or the
    /// documented fallback. See `yawInertia` above for why this is not a
    /// Knowledge Base value.
    double effectiveYawInertia() const {
        if (yawInertia > 1e-6) return yawInertia;
        const double i = mass * frontAxleToCg() * rearAxleToCg();
        return (i > 1e-6) ? i : mass;
    }

    /// Front brake force share actually used [-] - authored, or the ideal
    /// proportioning of KB Eq. 3-29/3-30 evaluated at the tire's peak
    /// friction. See `brakeProportionFront` above for the derivation.
    double effectiveBrakeProportionFront() const {
        if (brakeProportionFront > 1e-6) {
            return clampd(brakeProportionFront, 0.0, 1.0);
        }
        const double ideal =
            frontWeightDistribution + tire.lateralFriction * cgHeight / wheelbase;
        return clampd(ideal, 0.05, 0.95);
    }

    /// True when this wheel receives drive torque.
    bool isDrivenWheel(int wheel) const {
        const bool front = (wheel == FrontLeft || wheel == FrontRight);
        return (driveLayout == DriveLayout::FrontWheelDrive) ? front : !front;
    }

    /// Smallest achievable turn radius at full lock [m], measured about the
    /// rear axle: R = L / tan(delta_max). This is the Ackermann relation.
    double minTurnRadius() const {
        const double t = std::tan(maxSteeringAngle);
        return (std::abs(t) < 1e-9) ? 0.0 : wheelbase / std::abs(t);
    }

    /// Steady-state coast-down deceleration at a given speed [m/s^2],
    /// excluding engine braking and grade. Useful as a sanity figure: a
    /// passenger car sits around 0.04 g (~0.4 m/s^2) at motorway speed.
    /// Defined in VehicleParameters rather than RoadLoad so the UI can show it
    /// without needing environment state.
    double coastDownDeceleration(double speed, double airDensity = 1.225) const {
        const double fr = rollingResistanceCoeff * (1.0 + speed * 2.2369362920544 / 100.0);
        const double drag = 0.5 * airDensity * speed * speed
                          * dragCoefficient * frontalArea;
        return fr * kGravity + (mass > 0.0 ? drag / mass : 0.0);
    }

    /// Speed at which aerodynamic drag equals rolling resistance [m/s].
    ///
    /// Gillespie notes this crossover happens at roughly 50-60 mph
    /// (22-27 m/s) for a passenger car. Solving
    ///     f_r * m * g = 0.5 * rho * V^2 * C_D * A
    /// ignoring the mild speed term in f_r gives a closed form.
    double dragCrossoverSpeed(double airDensity = 1.225) const {
        const double denom = 0.5 * airDensity * dragCoefficient * frontalArea;
        if (!(denom > 1e-9)) return 0.0;
        return std::sqrt(rollingResistanceCoeff * mass * kGravity / denom);
    }

    /// Front overhang used purely to place the body mesh: the body box is
    /// centred between the axles plus this offset.
    double bodyCentreOffset() const {
        // Body centre sits midway along the wheelbase measured from the rear
        // axle reference point.
        return wheelbase * 0.5;
    }

    /// Clamp every field into a physically sensible range.
    ///
    /// A wheelbase of zero would make yaw_dot = v/L * tan(delta) infinite on
    /// the very first step, so this is a hard requirement, not a nicety.
    void validate();
};

} // namespace vsim
