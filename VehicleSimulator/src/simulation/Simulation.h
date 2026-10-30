#pragma once
//===========================================================================
//  Simulation.h  --  fixed-timestep driver.
//
//  THE CENTRAL GUARANTEE
//  ---------------------
//  Physics advances in fixed increments of `timestep` seconds, regardless of
//  how fast or how erratically the renderer is running. A 30 fps machine and
//  a 240 fps machine executing the same input sequence produce bit-identical
//  trajectories.
//
//  This matters more than it might seem. With a variable timestep, the yaw
//  integration error depends on frame rate, so the same steering input
//  produces a different path on different hardware - and telemetry becomes
//  incomparable between runs.
//
//  HOW IT WORKS
//  ------------
//  An accumulator collects real elapsed time and is drained in whole steps:
//
//      accumulator += frameTime * timeScale
//      while (accumulator >= dt) { stepPhysics(dt); accumulator -= dt; }
//
//  The leftover in the accumulator (always < dt) is carried to the next
//  frame, so no time is lost.
//===========================================================================

#include <memory>
#include <string>

#include "config/Config.h"
#include "experiments/Experiment.h"
#include "telemetry/Telemetry.h"
#include "vehicle/Vehicle.h"

namespace vsim {

class Simulation {
public:
    Simulation(const AppConfig& config, const VehicleParameters& vehicleParams);

    //-- driving the loop -----------------------------------------------

    /// Feed one rendered frame's worth of wall-clock time into the simulation.
    /// `frameTime` is real seconds since the previous call.
    /// Returns the number of physics steps actually executed.
    int advance(double frameTime, const ControlInput& driverInput);

    /// Restore the configured initial state. Clears telemetry and stops any
    /// running experiment.
    void reset();

    //-- control ----------------------------------------------------------

    void setPaused(bool p) { m_paused = p; }
    void togglePaused()    { m_paused = !m_paused; }
    bool isPaused() const  { return m_paused; }

    /// Execute exactly one physics step while paused. Useful for inspecting
    /// a manoeuvre frame by frame.
    void singleStep(const ControlInput& driverInput);

    //-- accessors --------------------------------------------------------

    Vehicle&       vehicle()       { return m_vehicle; }
    const Vehicle& vehicle() const { return m_vehicle; }

    Telemetry&       telemetry()       { return m_telemetry; }
    const Telemetry& telemetry() const { return m_telemetry; }

    SimulationConfig&       simConfig()       { return m_config.simulation; }
    const SimulationConfig& simConfig() const { return m_config.simulation; }

    AppConfig&       appConfig()       { return m_config; }
    const AppConfig& appConfig() const { return m_config; }

    /// Simulation time since the last reset [s].
    double time() const { return m_vehicle.state().time; }

    /// Physics steps executed since the last reset.
    long long stepCount() const { return m_stepCount; }

    /// Physics steps executed during the most recent advance() call.
    int lastStepCount() const { return m_lastStepCount; }

    /// True when the accumulator had to be clamped, i.e. the machine could not
    /// keep up and simulated time was dropped. Surfaced in the UI because a
    /// silently slowed-down simulation is a trap.
    bool didClampLastFrame() const { return m_clampedLastFrame; }

    /// Fraction of a step left in the accumulator, 0..1. A renderer can use
    /// this to interpolate between physics states for smoother motion.
    double interpolationAlpha() const {
        return (m_config.simulation.timestep > 0.0)
                   ? m_accumulator / m_config.simulation.timestep
                   : 0.0;
    }

    //-- experiments ------------------------------------------------------

    /// Start a scripted run. Resets the vehicle to the experiment's condition.
    void startExperiment(ExperimentType type);
    void stopExperiment();
    bool experimentRunning() const { return m_experiment != nullptr; }
    const Experiment* experiment() const { return m_experiment.get(); }
    double experimentTime() const { return m_experimentTime; }

    /// Rebuild the vehicle with new parameters (used when switching config).
    /// Keeps the active dynamics model, the environment AND the terrain.
    void reloadVehicle(const VehicleParameters& params);

    /// Ground height field for the active environment, non-owning; null means
    /// flat ground. Held HERE, not only on the Vehicle, because reloadVehicle()
    /// constructs a brand-new Vehicle: a terrain pointer that lived only on the
    /// old one is silently lost, and the new vehicle then sits at ground-plane
    /// zero while the real surface is tens of metres above or below it. Setting
    /// it through the Simulation keeps that impossible to get wrong.
    void setTerrain(const TerrainHeightField* t) {
        m_terrain = t;
        m_vehicle.setTerrain(t);
    }
    const TerrainHeightField* terrain() const { return m_terrain; }

    /// Swap the dynamics model in place, preserving the vehicle and its state.
    /// Names come from availableDynamicsModels().
    void setDynamicsModel(const std::string& name);
    const std::string& dynamicsModelName() const { return m_dynamicsModel; }

    /// Trailing path of the vehicle, for the trajectory debug overlay.
    /// Sampled by distance rather than by time so the trail stays uniform at
    /// any speed.
    const std::vector<glm::dvec3>& trajectory() const { return m_trajectory; }
    void clearTrajectory() { m_trajectory.clear(); }

private:
    void stepOnce(const ControlInput& input, double dt);
    void recordTrajectory();

    AppConfig m_config;
    Vehicle   m_vehicle;
    Telemetry m_telemetry;

    /// Factory key of the model currently installed in m_vehicle.
    std::string m_dynamicsModel;

    /// Non-owning; see setTerrain(). Re-applied by reloadVehicle().
    const TerrainHeightField* m_terrain = nullptr;

    std::unique_ptr<Experiment> m_experiment;
    double m_experimentTime = 0.0;

    double m_accumulator = 0.0;
    bool   m_paused = false;

    long long m_stepCount = 0;
    int  m_lastStepCount = 0;
    bool m_clampedLastFrame = false;

    std::vector<glm::dvec3> m_trajectory;
    static constexpr std::size_t kMaxTrajectoryPoints = 4000;
    static constexpr double      kTrajectorySpacing   = 0.5;  // [m]
};

} // namespace vsim
