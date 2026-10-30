#pragma once
//===========================================================================
//  SteeringGeometry.h  --  Ackerman steering geometry, in one place.
//
//  KB Section 8.2, Eq. 8-1 / 8-2 (equivalently Eq. 6-1 / 6-2):
//
//      delta_o = atan[ L / (R + t/2) ]        (OUTSIDE wheel)
//      delta_i = atan[ L / (R - t/2) ]        (INSIDE  wheel)
//
//  with the average, or Ackerman, steer angle (KB Eq. 6-3, RADIANS):
//
//      delta = L / R
//
//  KB Section 8.2: all four wheel axes must intersect at a single turn centre
//  for the wheels to roll without lateral slip, and because the inside and
//  outside front wheels run at different radii they must be steered to
//  different angles. The inside wheel is always steered MORE, and the
//  difference grows as R falls - so Ackerman matters at parking speeds and is
//  nearly irrelevant at highway speeds.
//
//  KB Section 8.2 [ASSUME] is worth repeating: these are LOW-SPEED (kinematic)
//  relations that assume zero slip angle at every tire. They are used here to
//  distribute one commanded centreline steer angle onto two road wheels, not
//  to predict the path - the path comes from the tire forces.
//
//  KB Section 8.2 also notes that a real trapezoidal linkage can only
//  approximate exact Ackerman, and that linkages are often designed
//  deliberately away from it because at speed the loaded outside tire wants
//  relatively MORE steer, not less. Section 8.12 quantifies that as an
//  "Ackermann fraction". The fraction is a design parameter neither source
//  gives for any particular vehicle, so exact Ackerman is used and the
//  deviation is recorded in README -> Open Questions.
//===========================================================================

#include <cmath>

#include "core/Units.h"

namespace vsim {

/// Left and right road wheel steer angles for a commanded centreline angle.
struct AckermanAngles {
    double left  = 0.0;   ///< [rad], positive = right
    double right = 0.0;   ///< [rad], positive = right
};

/// Distribute a centreline steer angle onto the two front wheels.
///
/// @param wheelbase   L [m]
/// @param frontTrack  t [m]
/// @param delta       commanded centreline steer angle [rad], + = right
inline AckermanAngles ackermanSteerAngles(double wheelbase,
                                          double frontTrack,
                                          double delta) {
    AckermanAngles out;
    if (!(std::abs(delta) > 1e-5)) return out;

    // Turn radius about the rear axle centre. Signed: positive = turning right.
    const double R = wheelbase / std::tan(delta);

    // The turn centre lies on the rear axle line, offset by R along the
    // vehicle's right. The left wheel sits at -t/2, the right at +t/2, so
    // their radii differ by half a track either way.
    //
    // Guard the denominators: at full lock on a wide track the turn centre can
    // fall between the wheels, which would flip a sign and snap a wheel
    // through 180 degrees. This is a numerical guard on a geometric
    // singularity, not a physical statement.
    const double denomLeft  = R + frontTrack * 0.5;
    const double denomRight = R - frontTrack * 0.5;

    constexpr double kMinRadius = 0.05;
    const double dl = (std::abs(denomLeft) < kMinRadius)
                          ? std::copysign(kMinRadius, denomLeft) : denomLeft;
    const double dr = (std::abs(denomRight) < kMinRadius)
                          ? std::copysign(kMinRadius, denomRight) : denomRight;

    out.left  = std::atan(wheelbase / dl);
    out.right = std::atan(wheelbase / dr);
    return out;
}

} // namespace vsim
