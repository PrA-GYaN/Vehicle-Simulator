#include "simulation/Simulation.h"

#include <algorithm>

#include "physics/VehicleDynamicsFactory.h"

namespace vsim {

Simulation::Simulation(const AppConfig& config, const VehicleParameters& vehicleParams)
    : m_config(config),
      m_vehicle(vehicleParams, config.controls,
                createVehicleDynamics(config.simulation.dynamicsModel)),
      m_telemetry(config.telemetry),
      m_dynamicsModel(config.simulation.dynamicsModel) {
    // Road load depends on the environment (air density, grade, wind), which
    // is configuration rather than vehicle data, so it is injected here.
    m_vehicle.setEnvironment(m_config.environment);
    m_vehicle.reset(m_config.initialState);

    if (m_config.experiment.autoStart) {
        startExperiment(experimentTypeFromString(m_config.experiment.type));
    }
}

//---------------------------------------------------------------------------
//  The fixed-timestep loop
//---------------------------------------------------------------------------

int Simulation::advance(double frameTime, const ControlInput& driverInput) {
    m_lastStepCount    = 0;
    m_clampedLastFrame = false;

    if (m_paused) return 0;

    const double dt = m_config.simulation.timestep;
    if (!(dt > 0.0)) return 0;

    // Guard the incoming frame time. A debugger pause, a window drag or the
    // very first frame after loading can hand us a huge or negative value.
    if (!isFinite(frameTime) || frameTime < 0.0) frameTime = 0.0;

    if (m_config.simulation.realtime) {
        //-------------------------------------------------------------------
        //  Real-time pacing.
        //
        //  Cap a single frame's contribution before it reaches the
        //  accumulator. Half a second is generous for a genuine hitch and
        //  small enough that we never try to simulate a minute of dropped
        //  time in one frame.
        //-------------------------------------------------------------------
        constexpr double kMaxFrameTime = 0.5;
        if (frameTime > kMaxFrameTime) {
            frameTime = kMaxFrameTime;
            m_clampedLastFrame = true;
        }

        m_accumulator += frameTime * m_config.simulation.timeScale;
    } else {
        // Non-realtime: one step per frame, as fast as the loop will go.
        m_accumulator += dt;
    }

    //-----------------------------------------------------------------------
    //  Drain the accumulator in whole steps.
    //
    //  The step budget is the second half of the spiral-of-death guard: even
    //  after clamping frameTime, a very small dt combined with a slow machine
    //  could demand more steps than we can afford. We execute what we can and
    //  discard the rest, which makes the simulation run slow rather than
    //  freeze - and we report it via didClampLastFrame().
    //-----------------------------------------------------------------------
    const int maxSteps = m_config.simulation.maxStepsPerFrame;

    while (m_accumulator >= dt && m_lastStepCount < maxSteps) {
        ControlInput input = driverInput;

        // A running experiment overrides the driver entirely.
        if (m_experiment) {
            input = m_experiment->controlAt(m_experimentTime, m_vehicle);
        }

        stepOnce(input, dt);

        m_accumulator -= dt;
        ++m_lastStepCount;
    }

    if (m_accumulator >= dt) {
        // We hit the step budget with time still owed. Drop it, otherwise the
        // debt compounds every frame and the simulation never recovers.
        m_accumulator      = 0.0;
        m_clampedLastFrame = true;
    }

    return m_lastStepCount;
}

void Simulation::singleStep(const ControlInput& driverInput) {
    const double dt = m_config.simulation.timestep;
    if (!(dt > 0.0)) return;

    ControlInput input = driverInput;
    if (m_experiment) input = m_experiment->controlAt(m_experimentTime, m_vehicle);
    stepOnce(input, dt);
}

void Simulation::stepOnce(const ControlInput& input, double dt) {
    m_vehicle.step(input, dt);

    ++m_stepCount;

    // Telemetry is captured HERE - inside the fixed-step loop - so the sample
    // interval is exactly dt. Sampling in the render loop instead would give
    // a jittery, frame-rate-dependent trace.
    m_telemetry.record(m_vehicle);
    m_telemetry.samplePlots(m_vehicle);

    recordTrajectory();

    if (m_experiment) {
        m_experimentTime += dt;
        if (m_experiment->isComplete(m_experimentTime)) {
            stopExperiment();
        }
    }
}

//---------------------------------------------------------------------------
//  Reset / reload
//---------------------------------------------------------------------------

void Simulation::reset() {
    stopExperiment();
    m_vehicle.reset(m_config.initialState);
    m_telemetry.clear();
    m_trajectory.clear();
    m_accumulator = 0.0;
    m_stepCount = 0;
    m_lastStepCount = 0;
    m_clampedLastFrame = false;
}

void Simulation::reloadVehicle(const VehicleParameters& params) {
    // Traction control level and tyre compound are the DRIVER's choices, made
    // at runtime, not properties of the car being swapped in: they carry
    // over, the same way the dynamics model does.
    const int tcLevel = m_vehicle.tractionControl();
    const TireCompound compound = m_vehicle.tireCompound();

    // Keep whichever model is currently running. Swapping the vehicle should
    // not silently swap the physics underneath it.
    m_vehicle = Vehicle(params, m_config.controls,
                        createVehicleDynamics(m_dynamicsModel));
    m_vehicle.setTractionControl(tcLevel);
    m_vehicle.setTireCompound(compound);
    m_vehicle.setEnvironment(m_config.environment);
    // The new Vehicle starts with no terrain - without this it would sit at
    // ground-plane zero while the real surface is elsewhere, which is exactly
    // what made a car switched at runtime appear to float. See setTerrain().
    m_vehicle.setTerrain(m_terrain);
    reset();
}

void Simulation::setDynamicsModel(const std::string& name) {
    if (name == m_dynamicsModel) return;
    m_dynamicsModel = name;
    m_vehicle.setDynamics(createVehicleDynamics(name));

    // Per-wheel state belongs to the model that produced it. Switching models
    // invalidates it, and the new model seeds its own on the next step.
    m_vehicle.mutableState().wheelDataValid = false;
}

//---------------------------------------------------------------------------
//  Experiments
//---------------------------------------------------------------------------

void Simulation::startExperiment(ExperimentType type) {
    if (type == ExperimentType::None) { stopExperiment(); return; }

    m_experiment = std::make_unique<Experiment>(type, m_config.experiment);
    m_experimentTime = 0.0;

    m_experiment->begin(m_vehicle, m_config.initialState);

    // A scripted run is only useful if it is recorded.
    m_telemetry.clear();
    m_telemetry.setRecording(true);
    m_trajectory.clear();
    m_accumulator = 0.0;
    m_paused = false;
}

void Simulation::stopExperiment() {
    m_experiment.reset();
    m_experimentTime = 0.0;
}

//---------------------------------------------------------------------------
//  Trajectory trail
//---------------------------------------------------------------------------

void Simulation::recordTrajectory() {
    const glm::dvec3 p = m_vehicle.state().position;

    // Sample by DISTANCE, not by time. A time-sampled trail bunches up at low
    // speed and stretches at high speed; a distance-sampled one is uniform,
    // which is what you want when reading a path off the screen.
    if (m_trajectory.empty() ||
        glm::distance(m_trajectory.back(), p) >= kTrajectorySpacing) {
        m_trajectory.push_back(p);
        if (m_trajectory.size() > kMaxTrajectoryPoints) {
            m_trajectory.erase(m_trajectory.begin());
        }
    }
}

} // namespace vsim
