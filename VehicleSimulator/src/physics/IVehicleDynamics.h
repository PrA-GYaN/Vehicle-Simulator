#pragma once
//===========================================================================
//  IVehicleDynamics.h  --  the swap point for vehicle physics models.
//
//  Everything above this interface (simulation, telemetry, UI, rendering)
//  is written against VehicleState and never against a specific model. That
//  is what makes it possible to drop in a dynamic bicycle model, a four-wheel
//  model with individual tyre forces, or a full multibody model later without
//  touching anything else.
//
//      IVehicleDynamics
//             |
//             +-- KinematicBicycleModel      (implemented)
//             |
//             +-- FourWheelModel             (implemented: four individual
//                                             tyres, per-wheel loads, slip,
//                                             friction circle)
//
//  CONTRACT for implementors
//  -------------------------
//   * update() advances `state` by exactly `dt` seconds. One step, no loops.
//   * `dt` is always > 0 and finite. The caller guarantees this.
//   * `params` and `input` are inputs only and must not be modified.
//   * The implementation must leave `state` finite. If an internal value
//     goes non-finite the model should recover rather than propagate it.
//   * update() must be pure with respect to global state: two calls with the
//     same arguments must produce the same result. This is what lets the
//     experiment framework and telemetry be reproducible.
//===========================================================================

#include <string>

#include "config/VehicleParameters.h"
#include "core/VehicleState.h"
#include "environment/TerrainHeightField.h"

namespace vsim {

/// Physical control demand, in engineering units rather than normalised
/// units. The vehicle layer converts ControlInput -> DriveCommand so that
/// physics models never have to know what "throttle = 0.6" means for a
/// particular car.
struct DriveCommand {
    /// NET longitudinal acceleration [m/s^2], ready to integrate.
    ///
    /// This is drive minus brake minus road load, already limited by the
    /// vehicle's capability and already guarded so it cannot reverse the
    /// vehicle through zero in a single step. A model that just wants to
    /// integrate should use this and ignore the breakdown below.
    double longitudinalAcceleration = 0.0;

    /// Commanded ROAD WHEEL steer angle [rad], already rate-limited and
    /// clamped to the vehicle's mechanical lock.
    /// Positive = steer right (see core/Coordinates.h).
    double steerAngle = 0.0;

    //-- breakdown ------------------------------------------------------
    //
    // The same demand, separated by origin. The kinematic model does not use
    // these, but a future model must: aerodynamic drag acts at the
    // aerodynamic centre (so it produces a pitching moment), rolling
    // resistance acts at the contact patches, and drive/brake forces act at
    // whichever wheels are driven or braked. Lumping them into one number
    // discards exactly the information a load-transfer model needs.
    //
    // All are magnitudes in m/s^2 unless noted.

    double driveAcceleration      = 0.0;  ///< from the powertrain, >= 0
    double brakeAcceleration      = 0.0;  ///< from the brakes, >= 0
    double rollingResistance      = 0.0;  ///< road load, >= 0
    double aerodynamicDrag        = 0.0;  ///< road load, >= 0
    double engineBraking          = 0.0;  ///< road load, >= 0
    double gradeAcceleration      = 0.0;  ///< SIGNED: +ve resists motion

    /// Effective rolling resistance coefficient f_r [-] at the current speed.
    ///
    /// Carried alongside `rollingResistance` (which is the same thing already
    /// reduced to a whole-vehicle deceleration) because a per-wheel model
    /// cannot use the lumped form: Gillespie Eq. 4-12 writes R_x = f_r*W with
    /// W the TOTAL weight, and a four-wheel model needs f_r*F_z at each
    /// contact patch, with the load that patch is actually carrying.
    double rollingResistanceCoeff = 0.0;
};

class IVehicleDynamics {
public:
    virtual ~IVehicleDynamics() = default;

    /// Advance `state` by `dt` seconds.
    ///
    /// `terrain` is non-owning and may be null - null means flat ground
    /// (z_road = 0 everywhere), which is what every track built by
    /// environment/Track.h except the imported Nurburgring scene produces.
    /// A model with no notion of per-wheel vertical dynamics (e.g.
    /// KinematicBicycleModel) is free to ignore it entirely.
    virtual void update(VehicleState& state,
                        const VehicleParameters& params,
                        const DriveCommand& command,
                        double dt,
                        const TerrainHeightField* terrain = nullptr) = 0;

    /// Human-readable model name, shown in the UI and written into telemetry
    /// headers so a log file records which model produced it.
    virtual const char* name() const = 0;

    /// One-line description of the modelling assumptions, shown in the UI.
    /// Keeping this next to the implementation stops the documentation and
    /// the code from drifting apart.
    virtual const char* assumptions() const = 0;

    /// Called on reset. Models with internal state (integrator history, tyre
    /// relaxation, filters) clear it here. Stateless models need do nothing.
    virtual void reset() {}
};

} // namespace vsim
