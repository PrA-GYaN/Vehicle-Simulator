#include "physics/RoadLoad.h"

#include <cmath>

namespace vsim {
namespace {

/// m/s -> mph. The rolling resistance speed relation below is an empirical
/// fit published in mph, so the conversion has to happen somewhere. Doing it
/// here, once, is safer than rewriting the constants and hoping.
constexpr double kMpsToMph = 2.2369362920544;

} // namespace

RoadLoad computeRoadLoad(const VehicleParameters& params,
                         const EnvironmentConditions& env,
                         double speed,
                         double throttle) {
    RoadLoad load;

    const double m = params.mass;
    if (!(m > 0.0) || !isFinite(m)) return load;

    // Road load depends on how fast the vehicle is moving, not which way.
    const double v = std::abs(sanitize(speed));

    //-----------------------------------------------------------------------
    //  1. ROLLING RESISTANCE
    //
    //  Gillespie Eq. 4-12:   R_x = f_r * W
    //
    //  This is NOT friction in the tyre-grip sense. It is the energy lost to
    //  hysteresis as the tyre carcass and tread are repeatedly deformed
    //  through the contact patch, plus scrubbing, plus deflection of the road
    //  surface. It is present whenever the wheels are turning and is almost
    //  independent of speed.
    //
    //  The mild speed dependence follows the FORM of Gillespie Eq. 4-14:
    //
    //      f_r = 0.01 * (1 + V/100)        [V in mph]
    //
    //  DEVIATION FROM THE SOURCE, stated explicitly: the book's literal
    //  constant 0.01 is replaced by the configurable base coefficient
    //  `rollingResistanceCoeff`, so different surfaces and vehicle classes can
    //  be represented (the book's own table gives 0.015 for a passenger car on
    //  concrete, 0.08 on medium-hard soil, 0.30 on sand). The shape of the
    //  speed term is unchanged.
    //-----------------------------------------------------------------------
    const double vMph = v * kMpsToMph;
    load.effectiveFr = params.rollingResistanceCoeff * (1.0 + vMph / 100.0);

    load.rollingForce = load.effectiveFr * m * kGravity;   // [N]
    load.rollingResistance = load.effectiveFr * kGravity;  // [m/s^2] = f_r * g

    //-----------------------------------------------------------------------
    //  2. AERODYNAMIC DRAG
    //
    //  Gillespie Eq. 4-2:    D_A = 0.5 * rho * V^2 * C_D * A
    //
    //  Note the V^2: drag is negligible at parking speed and dominant at
    //  motorway speed. The two curves cross at roughly 50-60 mph for a
    //  passenger car, which is exactly why fuel economy falls off a cliff
    //  above that.
    //
    //  A is the FRONTAL area, and C_D is only meaningful together with it -
    //  a drag coefficient quoted against a different reference area is a
    //  different number.
    //
    //  Headwind adds to the airspeed the body sees (Eq. 4-5).
    //-----------------------------------------------------------------------
    const double airspeed = v + std::max(0.0, env.headwind);
    const double dragForce =
        0.5 * env.airDensity * airspeed * airspeed
        * params.dragCoefficient * params.frontalArea;   // [N]

    load.dragForce = dragForce;
    load.aerodynamicDrag = dragForce / m;                 // [m/s^2]

    //-----------------------------------------------------------------------
    //  3. ENGINE BRAKING
    //
    //  *** NOT FROM GILLESPIE. ***
    //
    //  The source models the powertrain as a torque source (Ch.2) and does not
    //  give a closed-form overrun/engine-braking relation. This term is a
    //  pragmatic approximation added so that lifting off the throttle produces
    //  the deceleration a driver expects from a car in gear.
    //
    //  Modelled as a deceleration proportional to speed - a stand-in for the
    //  fact that pumping and friction losses rise with engine RPM, and RPM is
    //  proportional to road speed in a given gear - and faded out as the
    //  throttle opens, because a driven engine is not retarding the vehicle.
    //
    //  Replace this the moment a real engine map and gearbox exist.
    //-----------------------------------------------------------------------
    const double offThrottle = 1.0 - clampd(throttle, 0.0, 1.0);
    load.engineBraking = params.engineBrakingCoeff * v * offThrottle;

    //-----------------------------------------------------------------------
    //  4. GRADE
    //
    //  Gillespie Eq. 3-14:   R_g = W * sin(theta)
    //
    //  SIGNED, unlike the others: a downhill grade assists motion. The current
    //  environment is flat so this is zero, but the term belongs here so that
    //  adding terrain later is a data change rather than a physics change.
    //-----------------------------------------------------------------------
    load.grade = kGravity * std::sin(sanitize(env.gradeAngle));

    //-----------------------------------------------------------------------
    //  5. Fade out near standstill.
    //
    //  Rolling resistance and engine braking must not act on a stationary
    //  vehicle. Without this, a car at rest gets a residual deceleration that
    //  the integrator flips into a tiny reverse velocity, then forward again,
    //  and the vehicle jitters around zero forever.
    //
    //  Aerodynamic drag needs no such guard - it already vanishes as V^2.
    //-----------------------------------------------------------------------
    constexpr double kFadeSpeed = 0.2;   // [m/s]
    if (v < kFadeSpeed) {
        const double fade = v / kFadeSpeed;
        load.rollingResistance *= fade;
        load.engineBraking     *= fade;
        load.rollingForce      *= fade;
    }

    return load;
}

} // namespace vsim
