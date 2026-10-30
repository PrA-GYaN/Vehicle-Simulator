#pragma once
//===========================================================================
//  Coordinates.h  --  THE single source of truth for coordinate conventions.
//
//  Every conversion between "yaw angle" and "direction vector", and every
//  construction of a vehicle model matrix, goes through this header.  Nothing
//  anywhere else in the project is allowed to write `cos(yaw), 0, sin(yaw)`
//  by hand.  If the convention ever changes, it changes here and only here.
//===========================================================================
//
//  WORLD FRAME  (right-handed, OpenGL-style)
//  -----------------------------------------
//      +X  ->  world "east"      (ground plane axis)
//      +Y  ->  UP                (vertical axis)
//      +Z  ->  world "south"     (ground plane axis)
//
//  The ground plane is X-Z.  Y is always vertical.  This matches the
//  X = left/right, Y = up, Z = forward/back layout requested for the project.
//
//
//  YAW
//  ---
//  Yaw (psi) is the vehicle heading, measured in the X-Z ground plane and
//  referenced from the +X axis:
//
//      heading(psi) = ( cos(psi), 0, sin(psi) )
//
//  This follows directly from the kinematic bicycle model this project is
//  built on:
//
//      x_dot = v * cos(yaw)
//      z_dot = v * sin(yaw)
//
//  CONSEQUENCE, stated explicitly because it surprises people:
//  at yaw = 0 the vehicle points along +X, not along +Z.  +Z is a ground
//  plane axis, but it is not the zero-yaw heading.  Increasing yaw sweeps
//  the heading from +X toward +Z.
//
//  Viewed from above (looking down -Y) with +X drawn to the right and +Z
//  drawn downward, increasing yaw is a CLOCKWISE rotation.
//
//
//  VEHICLE LOCAL FRAME
//  -------------------
//      local +X  ->  FORWARD
//      local +Y  ->  UP
//      local +Z  ->  RIGHT      (the vehicle's own right-hand side)
//
//  This basis is right-handed: X cross Y = Z.
//  All vehicle geometry (body box, wheels, axles) is authored in this frame,
//  so "put a wheel at the front-left" is literally
//  ( +frontAxleDistance, wheelRadius, -halfTrack ).
//
//
//  SIGN OF STEERING
//  ----------------
//      yaw_dot = v / L * tan(delta)
//
//  With the heading convention above, a POSITIVE steer angle delta produces
//  a positive yaw rate, which rotates the heading from +X toward +Z, which is
//  a turn toward the vehicle's own RIGHT.
//
//      delta > 0  ->  steer RIGHT
//      delta < 0  ->  steer LEFT
//
//  The normalised steering input follows the same sign: +1 = full right.
//
//===========================================================================

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace vsim::coords {

/// World up axis. Constant everywhere in the project.
inline constexpr glm::dvec3 kWorldUp{0.0, 1.0, 0.0};

/// World-frame heading (unit forward vector) for a given yaw angle.
/// This is the definition the kinematic model integrates along.
inline glm::dvec3 heading(double yaw) {
    return {std::cos(yaw), 0.0, std::sin(yaw)};
}

/// World-frame "vehicle right" unit vector for a given yaw angle.
/// Derived as forward x up so the local basis stays right-handed.
inline glm::dvec3 right(double yaw) {
    return {-std::sin(yaw), 0.0, std::cos(yaw)};
}

/// Yaw angle of an arbitrary ground-plane direction. Inverse of heading().
inline double yawFromDirection(const glm::dvec3& d) {
    return std::atan2(d.z, d.x);
}

/// Rotation that maps the vehicle local frame onto the world frame.
///
/// A rotation of `-yaw` about +Y is what reproduces heading() exactly:
/// a right-handed rotation about +Y by theta maps (1,0,0) -> (cos t, 0, -sin t),
/// so theta = -yaw gives (cos yaw, 0, sin yaw). Verified, not guessed.
inline glm::dmat4 yawRotation(double yaw) {
    return glm::rotate(glm::dmat4(1.0), -yaw, kWorldUp);
}

/// Full model matrix for a body at `position` with heading `yaw`.
inline glm::dmat4 modelMatrix(const glm::dvec3& position, double yaw) {
    return glm::translate(glm::dmat4(1.0), position) * yawRotation(yaw);
}

/// Full model matrix for a body at `position` with heading `yaw`, pitch and
/// roll - the Euler sequence VEHICLE_DYNAMICS_KNOWLEDGE_BASE.md Section 1.4
/// states (yaw, then pitch, then roll - Body Z-Y-X), applied about the
/// vehicle's OWN axes so pitch and roll are always relative to the current
/// heading, not the world.
///
/// SIGN CONVENTIONS, fixed here once - neither the KB nor `yaw`'s own
/// convention decides these, so this project decides them exactly once:
///   * positive pitch = nose UP (rotation about the vehicle's local RIGHT
///     axis, i.e. +Z local)
///   * positive roll  = leaning to the vehicle's RIGHT (rotation about the
///     vehicle's local FORWARD axis, i.e. +X local) - matching the sign
///     already fixed for VehicleState::rollAngle.
/// Both are small-angle, quasi-static quantities in this project (see
/// SuspensionModel.h and FourWheelModel.cpp) - this function only decides
/// how an angle becomes a matrix, not where the angle comes from.
inline glm::dmat4 modelMatrixYPR(const glm::dvec3& position, double yaw,
                                 double pitch, double roll) {
    glm::dmat4 m = glm::translate(glm::dmat4(1.0), position) * yawRotation(yaw);
    // Right-hand rule about the local axis: rotating +X (forward) toward +Y
    // (up) about local +Z is nose-up; rotating +Y (up) toward +Z (right)
    // about local +X is a lean to the right. Both signs verified against the
    // standard rotation matrix, not guessed.
    m = glm::rotate(m, pitch, glm::dvec3(0.0, 0.0, 1.0));  // local +Z = right
    m = glm::rotate(m, roll,  glm::dvec3(1.0, 0.0, 0.0));  // local +X = forward
    return m;
}

/// Transform a vector expressed in the vehicle local frame into world space.
inline glm::dvec3 localToWorld(const glm::dvec3& local, double yaw) {
    return heading(yaw) * local.x + kWorldUp * local.y + right(yaw) * local.z;
}

/// Wrap an angle to (-pi, pi]. Used to keep yaw from growing without bound
/// over long runs, which would slowly bleed precision out of sin/cos.
inline double wrapAngle(double a) {
    constexpr double kTwoPi = 6.283185307179586476925286766559;
    a = std::fmod(a + 3.14159265358979323846, kTwoPi);
    if (a <= 0.0) a += kTwoPi;
    return a - 3.14159265358979323846;
}

} // namespace vsim::coords
