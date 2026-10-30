#include "experiments/Experiment.h"

#include <cmath>
#include <sstream>

#include "vehicle/Vehicle.h"

namespace vsim {

ExperimentType experimentTypeFromString(const std::string& s) {
    if (s == "straight_line")     return ExperimentType::StraightLine;
    if (s == "constant_steering") return ExperimentType::ConstantSteering;
    if (s == "step_steering")     return ExperimentType::StepSteering;
    if (s == "acceleration_test") return ExperimentType::AccelerationTest;
    if (s == "braking_test")      return ExperimentType::BrakingTest;
    if (s == "slalom")            return ExperimentType::Slalom;
    if (s == "coast_down")        return ExperimentType::CoastDown;
    return ExperimentType::None;
}

const char* experimentTypeToString(ExperimentType t) {
    switch (t) {
        case ExperimentType::StraightLine:     return "straight_line";
        case ExperimentType::ConstantSteering: return "constant_steering";
        case ExperimentType::StepSteering:     return "step_steering";
        case ExperimentType::AccelerationTest: return "acceleration_test";
        case ExperimentType::BrakingTest:      return "braking_test";
        case ExperimentType::Slalom:           return "slalom";
        case ExperimentType::CoastDown:        return "coast_down";
        case ExperimentType::None:             break;
    }
    return "none";
}

std::vector<std::string> availableExperiments() {
    return {"straight_line", "constant_steering", "step_steering",
            "acceleration_test", "braking_test", "slalom", "coast_down"};
}

Experiment::Experiment(ExperimentType type, const ExperimentConfig& cfg)
    : m_type(type), m_config(cfg) {
    if (!(m_config.duration > 0.0)) m_config.duration = 10.0;
    if (!(m_config.period   > 0.0)) m_config.period   = 3.0;
}

void Experiment::begin(Vehicle& vehicle, const InitialStateConfig& initial) {
    vehicle.reset(initial);

    // Jump straight to the test condition. See the header for why.
    //
    // AccelerationTest is the exception: it is defined as a standing-start
    // run, so it ignores the shared `initial_speed` field. Without this it
    // would silently inherit whatever speed the last constant-radius test
    // used and report a bogus 0-to-x time.
    const double v0 = (m_type == ExperimentType::AccelerationTest)
                          ? 0.0
                          : clampd(m_config.initialSpeed, 0.0,
                                   vehicle.parameters().maxSpeed);
    vehicle.mutableState().velocity = v0;

    // Convert the configured ROAD WHEEL angle into the normalised steering
    // input, since that is what controlAt() must return. Clamped to the
    // vehicle's lock - asking for 40 degrees on a car that can only reach 35
    // should give 35, not a silently ignored command.
    const double maxLock = vehicle.parameters().maxSteeringAngle;
    m_normalisedSteering = (maxLock > 1e-9)
                               ? clampAbs(m_config.steeringAngle / maxLock, 1.0)
                               : 0.0;
}

ControlInput Experiment::controlAt(double t, const Vehicle& vehicle) const {
    ControlInput in;

    switch (m_type) {
        case ExperimentType::None:
            break;

        //-------------------------------------------------------------------
        //  StraightLine - hold the initial speed, zero steer.
        //
        //  A simple proportional speed hold. Enough to keep the vehicle at
        //  the test condition; not a control-systems exercise.
        //-------------------------------------------------------------------
        case ExperimentType::StraightLine: {
            const double err = m_config.initialSpeed - vehicle.state().velocity;
            in.throttle = clampd(err * 0.5, 0.0, 1.0);
            in.brake    = clampd(-err * 0.5, 0.0, 1.0);
            in.steering = 0.0;
            break;
        }

        //-------------------------------------------------------------------
        //  ConstantSteering - fixed steer angle, speed held.
        //
        //  This is the classic constant-radius/constant-speed cornering test.
        //  With the kinematic model the resulting path is an exact circle of
        //  radius L / tan(delta), which makes it a good sanity check on the
        //  integrator: plot x vs z and it should close.
        //-------------------------------------------------------------------
        case ExperimentType::ConstantSteering: {
            const double err = m_config.initialSpeed - vehicle.state().velocity;
            in.throttle = clampd(err * 0.5, 0.0, 1.0);
            in.brake    = clampd(-err * 0.5, 0.0, 1.0);
            in.steering = m_normalisedSteering;
            break;
        }

        //-------------------------------------------------------------------
        //  StepSteering - straight, then an abrupt steer input.
        //
        //  The standard transient handling input. The kinematic model has no
        //  transient response at all (yaw rate follows steer instantly, only
        //  filtered by the steering slew rate), so this run mostly documents
        //  what the model CANNOT do - which is exactly what makes it valuable
        //  as a baseline once a dynamic model is added.
        //-------------------------------------------------------------------
        case ExperimentType::StepSteering: {
            const double err = m_config.initialSpeed - vehicle.state().velocity;
            in.throttle = clampd(err * 0.5, 0.0, 1.0);
            in.brake    = clampd(-err * 0.5, 0.0, 1.0);
            in.steering = (t >= m_config.stepTime) ? m_normalisedSteering : 0.0;
            break;
        }

        //-------------------------------------------------------------------
        //  AccelerationTest - full throttle from rest, straight.
        //-------------------------------------------------------------------
        case ExperimentType::AccelerationTest:
            in.throttle = 1.0;
            in.brake    = 0.0;
            in.steering = 0.0;
            break;

        //-------------------------------------------------------------------
        //  BrakingTest - hold the initial speed until stepTime, then full
        //  brake. Stopping distance is then read straight off the telemetry.
        //-------------------------------------------------------------------
        case ExperimentType::BrakingTest: {
            if (t < m_config.stepTime) {
                const double err = m_config.initialSpeed - vehicle.state().velocity;
                in.throttle = clampd(err * 0.5, 0.0, 1.0);
                in.brake    = clampd(-err * 0.5, 0.0, 1.0);
            } else {
                in.throttle = 0.0;
                in.brake    = 1.0;
            }
            in.steering = 0.0;
            break;
        }

        //-------------------------------------------------------------------
        //  CoastDown - release everything and let road load slow the vehicle.
        //
        //  This is the standard way real-world road load is measured (SAE
        //  J1263 / J2263): bring the vehicle to speed, shift to neutral,
        //  release the controls, and fit the deceleration curve to extract
        //  the rolling resistance and aerodynamic drag coefficients.
        //
        //  Here it runs in reverse - the coefficients are configured and the
        //  curve is the output - which makes it the direct validation of
        //  physics/RoadLoad.cpp. With all road load disabled the vehicle
        //  coasts at constant speed forever, which is Newton's first law
        //  working correctly on an empty force balance.
        //-------------------------------------------------------------------
        case ExperimentType::CoastDown:
            in.throttle = 0.0;
            in.brake    = 0.0;
            in.steering = 0.0;
            break;

        //-------------------------------------------------------------------
        //  Slalom - sinusoidal steering at the configured period.
        //-------------------------------------------------------------------
        case ExperimentType::Slalom: {
            const double err = m_config.initialSpeed - vehicle.state().velocity;
            in.throttle = clampd(err * 0.5, 0.0, 1.0);
            in.brake    = clampd(-err * 0.5, 0.0, 1.0);
            in.steering = m_normalisedSteering
                        * std::sin(kTwoPi * t / m_config.period);
            break;
        }
    }

    in.clampToValidRange();
    return in;
}

std::string Experiment::description() const {
    std::ostringstream os;
    switch (m_type) {
        case ExperimentType::None:
            return "No experiment - manual driving";
        case ExperimentType::StraightLine:
            os << "Hold " << m_config.initialSpeed << " m/s straight for "
               << m_config.duration << " s";
            break;
        case ExperimentType::ConstantSteering:
            os << "Hold " << m_config.initialSpeed << " m/s at "
               << radToDeg(m_config.steeringAngle) << " deg steer for "
               << m_config.duration << " s";
            break;
        case ExperimentType::StepSteering:
            os << "Step to " << radToDeg(m_config.steeringAngle) << " deg at t="
               << m_config.stepTime << " s, " << m_config.initialSpeed << " m/s";
            break;
        case ExperimentType::AccelerationTest:
            os << "Full throttle from rest for " << m_config.duration << " s";
            break;
        case ExperimentType::BrakingTest:
            os << "Full brake from " << m_config.initialSpeed << " m/s at t="
               << m_config.stepTime << " s";
            break;
        case ExperimentType::CoastDown:
            os << "Coast down from " << m_config.initialSpeed
               << " m/s with no throttle or brake";
            break;
        case ExperimentType::Slalom:
            os << "Slalom +/-" << radToDeg(m_config.steeringAngle) << " deg, period "
               << m_config.period << " s at " << m_config.initialSpeed << " m/s";
            break;
    }
    return os.str();
}

} // namespace vsim
