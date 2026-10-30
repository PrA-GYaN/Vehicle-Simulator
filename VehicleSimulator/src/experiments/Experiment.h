#pragma once
//===========================================================================
//  Experiment.h  --  scripted, repeatable manoeuvres.
//
//  An experiment REPLACES the driver. It is handed the simulation time and
//  the current vehicle state, and returns the control input for this step.
//
//  Deliberately independent of the renderer and of the input system: the same
//  experiment can be run interactively (watch it happen) or headless (collect
//  telemetry). That separation is what makes this the foundation for
//  automated vehicle-dynamics regression tests later.
//===========================================================================

#include <memory>
#include <string>
#include <vector>

#include "config/Config.h"
#include "core/VehicleState.h"

namespace vsim {

class Vehicle;

enum class ExperimentType {
    None,
    StraightLine,
    ConstantSteering,
    StepSteering,
    AccelerationTest,
    BrakingTest,
    Slalom,
    CoastDown
};

ExperimentType experimentTypeFromString(const std::string& s);
const char*    experimentTypeToString(ExperimentType t);
std::vector<std::string> availableExperiments();

/// A running scripted manoeuvre.
class Experiment {
public:
    Experiment(ExperimentType type, const ExperimentConfig& cfg);

    /// Prepare the vehicle for the run.
    ///
    /// Most experiments specify an initial speed. Rather than waiting for the
    /// vehicle to accelerate up to it - which would contaminate the trace with
    /// an arbitrary run-up - the state is set directly. This is a deliberate
    /// choice: the experiment starts at the condition being tested.
    void begin(Vehicle& vehicle, const InitialStateConfig& initial);

    /// Produce this step's control input.
    /// `t` is seconds since begin().
    ControlInput controlAt(double t, const Vehicle& vehicle) const;

    /// True once the configured duration has elapsed.
    bool isComplete(double t) const { return t >= m_config.duration; }

    ExperimentType type() const { return m_type; }
    const ExperimentConfig& config() const { return m_config; }
    ExperimentConfig& mutableConfig() { return m_config; }

    /// One-line description of what this run does, for the UI.
    std::string description() const;

private:
    ExperimentType   m_type;
    ExperimentConfig m_config;

    /// Normalised steering that produces the configured road wheel angle on
    /// this particular vehicle. Resolved in begin(), because the mapping
    /// depends on the vehicle's maximum lock.
    double m_normalisedSteering = 0.0;
};

} // namespace vsim
