#pragma once
//===========================================================================
//  VehicleState.h  --  the simulation's mutable state.
//
//  CONFIGURATION describes the vehicle (mass, wheelbase, limits).
//  STATE describes what the vehicle is doing right now.
//  These are kept in separate structs and never merged.
//===========================================================================

#include <array>

#include <glm/glm.hpp>

#include "core/Coordinates.h"
#include "core/Units.h"
#include "core/WheelState.h"

namespace vsim {

/// Instantaneous state of the vehicle.
///
/// REFERENCE POINT
/// ---------------
/// `position` is the REAR AXLE reference point, projected onto the ground
/// plane - not the centre of gravity and not the centre of the body.
///
/// This is not an arbitrary choice. The kinematic bicycle model
///
///     yaw_dot = v / L * tan(delta)
///
/// is exact only when the tracked point is the rear axle: that is the point
/// which, under the no-slip assumption, moves exactly along the vehicle's
/// heading. Referencing the CG instead introduces a sideslip term the model
/// does not have. Keeping the reference at the rear axle keeps the model
/// self-consistent, and the CG position is derived from it on demand.
struct VehicleState {
    glm::dvec3 position{0.0, 0.0, 0.0};  ///< rear axle reference point [m]
    double yaw           = 0.0;          ///< heading from +X            [rad]
    double velocity      = 0.0;          ///< forward speed  (body u)    [m/s]
    double acceleration  = 0.0;          ///< longitudinal accel (a_x)   [m/s^2]
    double steeringAngle = 0.0;          ///< actual road wheel angle    [rad]
    double yawRate       = 0.0;          ///< d(yaw)/dt                  [rad/s]

    //-----------------------------------------------------------------------
    //  LATERAL / SIDESLIP STATE
    //
    //  Zero under the kinematic bicycle model, which cannot represent them.
    //  Populated by any model that solves a force balance.
    //
    //  KB Section 1.3 defines three DIFFERENT angles, and they must not be
    //  conflated:
    //      heading angle  psi  - where the vehicle is POINTING
    //      course angle   nu   - where the vehicle is GOING
    //      sideslip angle beta - the difference,  beta = nu - psi
    //-----------------------------------------------------------------------

    /// Lateral velocity in the vehicle frame [m/s], positive to the vehicle's
    /// RIGHT. This is `v` of the SAE vehicle-fixed system (KB Section 1.2).
    double lateralVelocity = 0.0;

    /// Lateral acceleration a_y in the vehicle frame [m/s^2], positive right.
    double lateralAcceleration = 0.0;

    /// Sideslip angle beta [rad] at the CG (KB Section 1.3, Eq. 6-23).
    /// The angle between the vehicle's x axis and its velocity vector - the
    /// physical statement that a vehicle in a corner is crabbing.
    double sideslipAngle = 0.0;

    /// Rate of change of sideslip angle [rad/s].
    ///     beta_dot = A_y / V - omega                       (KB Eq. 7.29, S2)
    /// KB Section 9.4.15 calls this the transient quantity drivers actually
    /// feel; it is identically zero in steady state.
    double sideslipRate = 0.0;

    /// Body roll angle phi [rad], POSITIVE = leaning to the vehicle's RIGHT.
    ///
    ///     phi = (roll gradient) * a_y / g            (KB §3.5.1, [S3] p.682)
    ///
    /// Quasi-static: the roll gradient is a steady-state result and no source
    /// gives a roll equation of motion (KB gap G9). A right-hand turn produces
    /// a NEGATIVE phi - [S3] p.681: "the car rolls to the outside of the turn."
    double rollAngle = 0.0;

    //-----------------------------------------------------------------------
    //  PER-WHEEL STATE
    //-----------------------------------------------------------------------

    /// Per-wheel tire state. Only meaningful when `wheelDataValid` is true.
    std::array<WheelDynamics, WheelCount> wheels{};

    /// True when the active dynamics model populates `wheels`. The kinematic
    /// bicycle model does not - it has no tires in any meaningful sense - so
    /// the UI and telemetry must not present its zeros as measurements.
    bool wheelDataValid = false;

    /// Simulation time since the last reset [s]. Advanced by the fixed-step
    /// integrator, so it is exact and independent of wall-clock jitter.
    double time = 0.0;

    /// Unit heading vector in world space.
    glm::dvec3 forward() const { return coords::heading(yaw); }

    /// Unit "vehicle right" vector in world space.
    glm::dvec3 right() const { return coords::right(yaw); }

    /// World-space velocity vector.
    ///
    /// Built from BOTH body-frame components, so a model that produces
    /// sideslip shows it: the arrow points where the vehicle is going (course
    /// angle nu), not where it is pointing (heading psi). Under the kinematic
    /// model `lateralVelocity` is always zero and this reduces to the old
    /// heading-aligned form exactly.
    glm::dvec3 velocityVector() const {
        return forward() * velocity + right() * lateralVelocity;
    }

    /// Speed along the path [m/s] - the magnitude of the velocity vector.
    double speed() const {
        return std::sqrt(velocity * velocity + lateralVelocity * lateralVelocity);
    }

    /// Replace any non-finite component with a safe value.
    /// Called after every integration step - see Units.h for the rationale.
    void sanitizeAll() {
        position.x          = vsim::sanitize(position.x);
        position.y          = vsim::sanitize(position.y);
        position.z          = vsim::sanitize(position.z);
        yaw                 = vsim::sanitize(yaw);
        velocity            = vsim::sanitize(velocity);
        acceleration        = vsim::sanitize(acceleration);
        steeringAngle       = vsim::sanitize(steeringAngle);
        yawRate             = vsim::sanitize(yawRate);
        lateralVelocity     = vsim::sanitize(lateralVelocity);
        lateralAcceleration = vsim::sanitize(lateralAcceleration);
        sideslipAngle       = vsim::sanitize(sideslipAngle);
        sideslipRate        = vsim::sanitize(sideslipRate);
        rollAngle           = vsim::sanitize(rollAngle);
        time                = vsim::sanitize(time);
        for (auto& w : wheels) w.sanitizeAll();
    }
};

/// Normalised control input. Dimensionless on purpose: the mapping from these
/// numbers to physical units (m/s^2, radians) lives in the vehicle layer, so
/// input sources (keyboard, experiment script, future autopilot) never need
/// to know anything about the vehicle they are driving.
struct ControlInput {
    double throttle = 0.0;  ///< [0, 1]
    double brake    = 0.0;  ///< [0, 1]
    double steering = 0.0;  ///< [-1, +1]   +1 = full RIGHT (see Coordinates.h)

    /// Clutch PEDAL travel [0, 1]: 0 = fully engaged (pedal up, drive
    /// connected), 1 = fully disengaged (pedal to the floor).
    ///
    /// Only read when the vehicle has a detailed engine AND its gearbox is in
    /// manual-clutch mode (see config/GearboxParameters.h). Every other
    /// vehicle, and every experiment script that never sets it, leaves it at
    /// 0 and is completely unaffected - a released clutch is the neutral
    /// default, not a special case.
    double clutch = 0.0;

    /// Gear-change REQUEST for this step: -1 down, 0 none, +1 up.
    ///
    /// Edge-triggered rather than continuous, unlike everything else in this
    /// struct: a gear change is an event. It is consumed by the gearbox on
    /// the step it arrives and never latches, so a held key shifts once
    /// rather than sweeping through the whole box.
    int shiftRequest = 0;

    void clampToValidRange() {
        throttle = clampd(sanitize(throttle), 0.0, 1.0);
        brake    = clampd(sanitize(brake),    0.0, 1.0);
        steering = clampAbs(sanitize(steering), 1.0);
        clutch   = clampd(sanitize(clutch),   0.0, 1.0);
        shiftRequest = (shiftRequest > 0) ? 1 : ((shiftRequest < 0) ? -1 : 0);
    }
};

} // namespace vsim
