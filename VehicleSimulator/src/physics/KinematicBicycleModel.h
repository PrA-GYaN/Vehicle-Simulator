#pragma once
//===========================================================================
//  KinematicBicycleModel.h
//===========================================================================

#include "physics/IVehicleDynamics.h"

namespace vsim {

/// Kinematic (geometric) bicycle model.
///
/// The two wheels of each axle are collapsed onto the vehicle centreline,
/// giving a two-wheel "bicycle". Forces are not modelled at all: the vehicle
/// is assumed to go exactly where its wheels point.
///
/// GOVERNING EQUATIONS (rear-axle reference point)
/// -----------------------------------------------
///     x_dot   = v * cos(psi)
///     z_dot   = v * sin(psi)
///     psi_dot = (v / L) * tan(delta)
///     v_dot   = a
///
/// where
///     x, z    ground-plane position of the REAR AXLE   [m]
///     psi     heading, measured from +X                [rad]
///     v       forward speed                            [m/s]
///     delta   front road wheel steer angle             [rad]
///     L       wheelbase                                [m]
///     a       commanded longitudinal acceleration      [m/s^2]
///
/// WHERE psi_dot COMES FROM
/// ------------------------
/// Under the no-slip assumption the vehicle turns about a point on the
/// extension of the rear axle. From the steering geometry, the turn radius
/// about the rear axle is
///
///     R = L / tan(delta)
///
/// which is the Ackermann relation. Yaw rate is then just the speed divided
/// by the radius, psi_dot = v / R = (v / L) * tan(delta). Nothing about
/// mass, tyres or grip enters, which is exactly why this model is cheap and
/// exactly why it stops being valid at speed.
///
/// ASSUMPTIONS - all of them, stated plainly
/// -----------------------------------------
///  1. NO TYRE SLIP. Every wheel rolls exactly along its own plane. There is
///     no slip angle, so no lateral tyre force is ever computed.
///  2. NO SIDESLIP. The velocity vector is always parallel to the heading.
///     A real car crabs in a corner; this one cannot.
///  3. NO LOAD TRANSFER. Mass and CG height are carried as parameters but are
///     never used by this model. They exist for the models that come later.
///  4. NO ROLL, PITCH OR SUSPENSION. The body is rigid and always level.
///  5. PLANAR MOTION ONLY. y stays at its initial value; no terrain, no grade.
///  6. INSTANT FORCE RESPONSE. Commanded acceleration is achieved exactly.
///     There is NO TRACTION LIMIT: the model never checks whether the tyres
///     could actually transmit the demanded force.
///
///  What this model DOES include, as of the road load addition:
///  resistance to motion - rolling resistance, aerodynamic drag, engine
///  braking and grade (physics/RoadLoad.h). These are longitudinal forces
///  only. They slow a coasting vehicle correctly but, being lumped into a
///  single acceleration, they produce no pitching moment and no load
///  transfer, because this model has no mechanism to represent either.
///
/// VALIDITY
/// --------
/// Good at low lateral acceleration - roughly below 0.3 g, which for a 2.7 m
/// wheelbase car is comfortable parking-to-suburban driving. Above that, a
/// real vehicle needs slip angles to generate the cornering force, and its
/// path departs from what this model predicts. The model will happily let you
/// take a hairpin at 200 km/h; the absence of a complaint is not validation.
///
/// INTEGRATION
/// -----------
/// Explicit (forward) Euler, evaluated in a deliberate order - see the .cpp.
/// First-order accurate: halving the timestep halves the error.
class KinematicBicycleModel final : public IVehicleDynamics {
public:
    void update(VehicleState& state,
                const VehicleParameters& params,
                const DriveCommand& command,
                double dt,
                const TerrainHeightField* terrain = nullptr) override;

    const char* name() const override { return "Kinematic Bicycle"; }

    const char* assumptions() const override {
        return "No tyre slip, no sideslip, no load transfer, no suspension. "
               "Valid below ~0.3 g lateral.";
    }
};

} // namespace vsim
