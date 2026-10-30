#pragma once
//===========================================================================
//  RoadLoad.h  --  forces that resist forward motion.
//
//  These are the forces a coasting vehicle feels. They are what make a real
//  car slow down when you lift off the throttle, and their absence is why a
//  pure kinematic model coasts forever.
//
//  This is a SEPARATE concern from the powertrain and the brakes:
//    * the powertrain and brakes are commanded by the driver
//    * road load is imposed by the environment and the vehicle's own speed
//
//  Keeping it in its own module means a future dynamic model can apply each
//  component at its correct point of action (aerodynamic drag at the
//  aerodynamic centre, rolling resistance at the contact patches) instead of
//  receiving one lumped number.
//
//  SOURCES
//  -------
//  Rolling resistance and aerodynamic drag follow Gillespie, "Fundamentals of
//  Vehicle Dynamics" (SAE R-114, 1992), Chapter 4. Equation numbers below are
//  the book's. Engine braking is NOT from that source and is flagged as such.
//===========================================================================

#include "config/VehicleParameters.h"
#include "core/Units.h"

namespace vsim {

/// Environmental conditions the road load depends on.
struct EnvironmentConditions {
    /// Air density [kg/m^3]. Standard sea-level value is 1.225 at 15 C.
    /// Falls with altitude and temperature (Gillespie Eq. 4-3b).
    double airDensity = 1.225;

    /// Road grade angle [rad]. Positive is uphill.
    /// The current environment is flat, so this is 0, but the term is here
    /// because grade is dynamically identical to a steady longitudinal
    /// acceleration and terrain is an obvious future addition.
    double gradeAngle = 0.0;

    /// Headwind component along the vehicle's direction of travel [m/s].
    /// Positive opposes motion (Gillespie Eq. 4-5: a headwind adds to the
    /// airspeed the body sees).
    double headwind = 0.0;
};

/// Road load broken into its components, all as DECELERATIONS in m/s^2.
///
/// Every component is a magnitude (>= 0). The caller applies the sign, because
/// only the caller knows which way the vehicle is currently travelling.
///
/// The breakdown is kept rather than pre-summed so the debug UI can show which
/// term dominates at a given speed - which is the single most instructive
/// thing about road load. Rolling resistance rules at low speed, aerodynamic
/// drag takes over around 50-60 mph (Gillespie Ch.4).
struct RoadLoad {
    double rollingResistance = 0.0;  ///< [m/s^2]
    double aerodynamicDrag   = 0.0;  ///< [m/s^2]
    double engineBraking     = 0.0;  ///< [m/s^2]
    double grade             = 0.0;  ///< [m/s^2] SIGNED: +ve resists, -ve assists

    /// Total resistive deceleration excluding grade [m/s^2], always >= 0.
    double resistiveTotal() const {
        return rollingResistance + aerodynamicDrag + engineBraking;
    }

    /// Effective rolling resistance coefficient actually used [-].
    /// Retained for display; it varies with speed.
    double effectiveFr = 0.0;

    /// Aerodynamic drag force [N] and rolling resistance force [N], for the UI.
    double dragForce    = 0.0;
    double rollingForce = 0.0;
};

/// Compute the road load acting on a vehicle at a given speed.
///
/// @param params    vehicle parameters (mass, f_r, C_D, frontal area, ...)
/// @param env       environmental conditions
/// @param speed     forward speed [m/s]. Sign is ignored; magnitude is used.
/// @param throttle  applied throttle [0,1]. Engine braking fades out as the
///                  throttle is opened, because a driven engine is not
///                  retarding the vehicle.
RoadLoad computeRoadLoad(const VehicleParameters& params,
                         const EnvironmentConditions& env,
                         double speed,
                         double throttle);

} // namespace vsim
