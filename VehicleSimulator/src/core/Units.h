#pragma once
//===========================================================================
//  Units.h  --  Unit policy and numeric hygiene.
//
//  UNIT POLICY (applies to every quantity that crosses a module boundary):
//
//      length          metres              m
//      mass            kilograms           kg
//      time            seconds             s
//      speed           metres per second   m/s
//      acceleration    m/s^2
//      angle           RADIANS             rad
//      angular rate    radians per second  rad/s
//
//  Degrees appear in exactly two places, and nowhere else:
//    * configuration files (human-facing), converted on load
//    * the debug UI (human-facing), converted on display
//  Everything in between is radians. This is deliberate: mixing degrees into
//  the physics is the single most common source of silent 57x errors.
//===========================================================================

#include <cmath>
#include <limits>

namespace vsim {

inline constexpr double kPi     = 3.14159265358979323846;
inline constexpr double kTwoPi  = 2.0 * kPi;
inline constexpr double kGravity = 9.80665;          // m/s^2

inline constexpr double degToRad(double d) { return d * (kPi / 180.0); }
inline constexpr double radToDeg(double r) { return r * (180.0 / kPi); }
inline constexpr double mpsToKph(double v) { return v * 3.6; }
inline constexpr double kphToMps(double v) { return v / 3.6; }

/// True only for values that are safe to feed back into the integrator.
inline bool isFinite(double v) {
    return std::isfinite(v);
}

/// Replace non-finite values with a fallback.
///
/// Every state variable passes through this after every physics step. A NaN
/// that reaches the state is permanent - it contaminates position, then the
/// camera, then the render matrix, and the whole application goes black with
/// no diagnostic. Catching it at the source is much cheaper.
inline double sanitize(double v, double fallback = 0.0) {
    return std::isfinite(v) ? v : fallback;
}

/// Clamp helper (std::clamp requires <algorithm> and is constexpr-noisy here).
inline constexpr double clampd(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

/// Signed clamp to a symmetric range.
inline constexpr double clampAbs(double v, double limit) {
    return clampd(v, -limit, limit);
}

/// Move `current` toward `target` at no more than `rate` units per second.
/// This is the primitive behind steering rate limiting and throttle/brake
/// ramping - actuators cannot step instantaneously and neither should we.
inline double moveToward(double current, double target, double rate, double dt) {
    const double maxStep = rate * dt;
    const double delta   = target - current;
    if (delta > maxStep)  return current + maxStep;
    if (delta < -maxStep) return current - maxStep;
    return target;
}

} // namespace vsim
