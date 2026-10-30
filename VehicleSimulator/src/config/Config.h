#pragma once
//===========================================================================
//  Config.h  --  YAML-backed configuration.
//
//  Loading is total: every loader returns a fully-populated struct. Missing
//  keys fall back to the struct's default, missing files produce a warning
//  and the defaults. The simulator always starts.
//===========================================================================

#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "config/VehicleParameters.h"
#include "core/VehicleState.h"
#include "physics/RoadLoad.h"

namespace vsim {

/// Fixed-timestep integrator settings.
struct SimulationConfig {
    /// Physics timestep [s]. 0.01 = 100 Hz.
    /// Smaller is more accurate and more expensive; the integrator is
    /// explicit Euler, so accuracy is first-order in this value.
    double timestep = 0.01;

    /// Simulated seconds per wall-clock second. 0.5 = half speed.
    double timeScale = 1.0;

    /// When true the simulation is paced to wall-clock time. When false it
    /// steps once per frame as fast as it can - used by headless experiments.
    bool realtime = true;

    /// Upper bound on physics steps executed per rendered frame.
    ///
    /// Without this, a long stall (window drag, breakpoint, disk hitch) makes
    /// the accumulator demand hundreds of steps, which takes even longer, which
    /// grows the accumulator further. That runaway is the classic "spiral of
    /// death". We clamp and drop the surplus time instead.
    int maxStepsPerFrame = 8;

    /// Which vehicle dynamics model to run. See VehicleDynamicsFactory.
    ///
    ///   "four_wheel"         four individual tyres, tyre forces, slip,
    ///                        load transfer, friction circle  (default)
    ///   "kinematic_bicycle"  the geometric model - no forces at all
    ///
    /// The kinematic model is retained deliberately. It is not a lesser
    /// version of the same thing: it is a valid description of a vehicle
    /// below roughly 15 mph, where KB Section 9.4.13 shows steering geometry
    /// rather than grip limits path curvature.
    std::string dynamicsModel = "four_wheel";

    void validate();
};

/// Initial / reset state.
struct InitialStateConfig {
    glm::dvec3 position{0.0, 0.0, 0.0};  ///< rear axle reference point [m]
    double yaw      = 0.0;               ///< [rad] (degrees in YAML)
    double velocity = 0.0;               ///< [m/s]
};

/// Chase camera tuning.
struct CameraConfig {
    double distance    = 8.0;   ///< [m] behind the vehicle
    double height      = 4.0;   ///< [m] above the vehicle
    double followSpeed = 5.0;   ///< [1/s] exponential smoothing rate
    double lookAhead   = 2.0;   ///< [m] ahead of the vehicle to aim at
    double fovDegrees  = 60.0;
    /// Near plane [m]. NOT a free choice: depth buffer precision is dominated
    /// by this value, and the circuit is a set of near-coplanar layers
    /// (ground, grass, asphalt, paint) spread over kilometres. At 0.1 m the
    /// far end of a main straight has less depth resolution than the few
    /// millimetres that separate the road from the grass, and the road
    /// visibly disappears under the verge. 0.5 m is fifty times the precision
    /// and is still closer than the camera ever gets to the car.
    /// See Renderer::drawEnvironment(), which also applies polygon offsets.
    double nearPlane   = 0.5;
    double farPlane    = 2500.0;
};

/// Driver-input shaping. These are actuator rates, not vehicle limits.
struct ControlsConfig {
    /// Road wheel angular rate [rad/s]. YAML expresses this in deg/s.
    /// A real steering system cannot step instantly, and neither should this.
    double steeringRate = degToRad(180.0);

    /// How fast throttle/brake ramp toward the requested value [1/s].
    double throttleRate = 2.0;
    double brakeRate    = 5.0;

    /// How fast the clutch pedal travels [1/s]. A keyboard gives a binary
    /// key; this is what turns it into pedal travel. ~8/s is about an eighth
    /// of a second floor-to-top, which is a brisk but human clutch foot.
    double clutchRate   = 8.0;

    /// How fast steering returns to centre when no key is held [rad/s].
    double steeringReturnRate = degToRad(240.0);
};

/// Sound output - see src/audio/SoundSynth.h. Presentation only.
struct AudioConfig {
    bool   enabled = true;
    double masterVolume = 0.8;   ///< [0,1]
    double sampleRate = 48000.0;
};

/// Android USB (ADB) controller input. See src/input/AndroidUsbInput.h and
/// AndroidController/PROTOCOL.md for the wire format.
struct AndroidInputConfig {
    /// Master switch. When false, AndroidUsbInput is never constructed and
    /// nothing about this feature touches the rest of the simulator.
    bool enabled = false;

    /// TCP port both sides agree on. The simulator listens on
    /// 127.0.0.1:<port>; the phone connects to the same port number on its
    /// own loopback, which `adb reverse` tunnels through the USB cable.
    int port = 7788;

    /// How long without a valid packet before the input is treated as
    /// disconnected and the vehicle falls back to a safe/centred state.
    double timeoutSeconds = 0.5;

    /// Run `adb reverse tcp:<port> tcp:<port>` automatically on startup.
    /// Best-effort: failure (adb missing, no device attached) is logged and
    /// otherwise ignored, since the user can always run it by hand.
    bool autoAdbReverse = true;
};

struct TelemetryConfig {
    bool        enabledAtStart = false;
    std::string outputDirectory = "logs";
    /// Ring buffer capacity, in samples. At 100 Hz, 60000 = 10 minutes.
    std::size_t maxSamples = 60000;
};

/// An imported 3D model used AS the driving environment itself - a real
/// terrain scan, rather than the procedural Track ribbon. See
/// src/environment/TerrainHeightField.h for how its geometry becomes ground
/// height for the physics, and src/rendering/GltfModel.h for the loader
/// (reused unchanged: an environment model has no wheel nodes, so every
/// primitive bakes into the static group).
struct EnvironmentModelConfig {
    /// Path to a .glb file. Empty (the default) = no environment model; the
    /// procedural Track/ground is used exactly as before this existed.
    std::string path;

    std::string forwardAxis = "+z";
    std::string upAxis      = "+y";

    /// Authored spawn override. A raw terrain scan carries no drivable
    /// centreline for any source to derive a spawn point from - unlike
    /// every Track preset, this is honestly authored, not computed. When
    /// `hasSpawn` is false, Application picks a default near the model's
    /// bounding-box centre via a flattest-patch heuristic over the built
    /// TerrainHeightField - a reasonable starting guess, logged so it can be
    /// corrected here after a look at the result.
    bool       hasSpawn = false;
    glm::dvec3 spawnPosition{0.0};
    double     spawnYaw = 0.0;   ///< [rad] (degrees in YAML)

    bool hasModel() const { return !path.empty(); }
};

/// Declarative experiment description. See experiments/Experiment.h.
struct ExperimentConfig {
    std::string type = "none";
    double initialSpeed  = 0.0;   ///< [m/s]
    double steeringAngle = 0.0;   ///< [rad]  (degrees in YAML)
    double duration      = 10.0;  ///< [s]
    double stepTime      = 1.0;   ///< [s] when the step is applied
    double period        = 3.0;   ///< [s] slalom period
    bool   autoStart     = false;
};

/// Everything read from config/simulation.yaml.
struct AppConfig {
    SimulationConfig      simulation;
    InitialStateConfig    initialState;
    CameraConfig          camera;
    ControlsConfig        controls;
    TelemetryConfig       telemetry;
    ExperimentConfig      experiment;
    EnvironmentConditions environment;
    AndroidInputConfig    androidInput;
    AudioConfig           audio;
    EnvironmentModelConfig environmentModel;

    /// Vehicle config to load at startup, relative to config/vehicles/.
    std::string defaultVehicle = "ferrari_f40";

    /// Track to build at startup. See Track::createByName() for the accepted
    /// names. Overridden by --track on the command line.
    std::string defaultTrack = "apex_gp";

    int  windowWidth  = 1600;
    int  windowHeight = 900;
    bool vsync        = true;
};

//---------------------------------------------------------------------------
//  Loading
//---------------------------------------------------------------------------

/// Load application config. Returns defaults (with a warning) if unreadable.
AppConfig loadAppConfig(const std::string& path);

/// Load a vehicle. Returns defaults (with a warning) if unreadable.
VehicleParameters loadVehicleParameters(const std::string& path);

/// Load only an engine: a file holding a `detailed:` block at the top level
/// (what tools/engine_toml_to_yaml.py writes), or any vehicle file that has
/// one. For the offline engine tools - the simulator itself loads engines as
/// part of a vehicle. Returns false, with a message, if there is none.
bool loadEngineParameters(const std::string& path, EngineParameters& out);

/// Load only a `sound: engine_sim:` block, from a vehicle file or from what
/// tools/engine_toml_to_yaml.py writes. For the offline engine tools.
bool loadEngineSimSound(const std::string& path, EngineSimSoundConfig& out);

/// Enumerate `config/vehicles/*.yaml`, returning bare names without extension.
std::vector<std::string> listAvailableVehicles(const std::string& vehicleDir);

/// Resolve a runtime asset path.
///
/// Tries, in order: the path as given, then relative to the executable's
/// directory. This makes both `./build/bin/VehicleSimulator` and
/// `cd build/bin && ./VehicleSimulator` work without a wrapper script.
std::string resolveResourcePath(const std::string& relativePath);

/// Record the executable path so resolveResourcePath() can use it.
void setExecutablePath(const char* argv0);

} // namespace vsim
