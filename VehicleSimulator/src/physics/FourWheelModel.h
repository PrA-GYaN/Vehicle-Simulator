#pragma once
//===========================================================================
//  FourWheelModel.h  --  a dynamic four-wheel vehicle model built only from
//                        the Vehicle Dynamics Knowledge Base.
//
//  WHAT REPLACES WHAT
//  ------------------
//  The kinematic bicycle model assumes the vehicle goes exactly where its
//  wheels point: no tire forces, no slip, no sideslip, no load transfer, no
//  traction limit. This model computes forces at four contact patches and lets
//  the vehicle's motion follow from them. The kinematic model is kept and
//  remains selectable - it is a valid low-lateral-acceleration model and the
//  KB says so (Section 9.4.1, Section 9.4.13: geometry governs below about
//  15 mph).
//
//  THE CAUSAL CHAIN, WHICH IS THE KB'S OWN
//  ---------------------------------------
//  KB Section 0.5 states the organising idea of the whole subject:
//  "almost everything in vehicle dynamics is a chain that begins with vertical
//  load and ends with acceleration, with the tire in the middle."  The step
//  function below is that chain, in that order:
//
//      operating condition (V, delta, throttle, brake)
//        -> vertical load distribution        Section 3   (Eq. 1-8/1-9, 6-27)
//        -> tire operating state              Section 2   (Eq. 10-1, Sec 1.5)
//        -> tire forces and moments           Section 2   (TireModel)
//        -> resultant forces and moments      Section 1.7 (Eq. 1-1, 1-2)
//        -> accelerations
//        -> vehicle states
//        -> back to the top
//
//  KB Section 0.5 also names the two feedback closures, and both are present:
//  acceleration feeds back into load transfer, and the resulting loads change
//  what the tires can do.
//
//  WHERE THE EQUATIONS OF MOTION COME FROM
//  ---------------------------------------
//  This matters, because it is the one place where it would be easiest to
//  quietly import something the KB does not contain.
//
//  KB gap G8 is OPEN and is called "the most significant remaining gap":
//  neither source derives the transient yaw/sideslip equations of motion, and
//  the KB says in terms that "a reader wanting the classical linear
//  bicycle-model state-space form [v_dot, r_dot] = A[v, r] + B*delta will not
//  find it in either source". So that form is NOT used here.
//
//  What the KB does give is the general rigid-body statement:
//      KB Eq. 1-1   / Eq. 2.225 [S2]:  sum F = m * a_G
//      KB Eq. 1-2   / Eq. 2.236 [S2]:  M_z = I_zz * omega_dot_z
//                                       (+ (I_yy - I_xx)*omega_x*omega_y,
//                                        which vanishes in planar motion)
//  and, critically, KB Section 1.11 [S2] says where to write each one:
//  "translational equations are best written in the fixed global frame;
//  rotational equations are best written in the body-fixed frame."
//
//  That is exactly what this model does, and it is the reason it needs no
//  rotating-frame transport terms of its own: the translational state is a
//  world-frame velocity vector, integrated directly from the world-frame
//  resultant force, and the body-frame components u and v are recovered from
//  it afterwards. The apparent "centrifugal" behaviour appears on its own,
//  because the heading rotates under the velocity vector.
//
//  WHAT THIS MODEL DOES NOT DO
//  ---------------------------
//  There is no roll, pitch or suspension model. That is not an oversight: the
//  KB's roll model (Eq. 6-31) needs the sprung CG height above the roll axis
//  h_1 and the front and rear roll stiffnesses K_phi, and KB Section 12.1/12.3
//  give no typical value for any of them. Everything downstream of body roll -
//  the geometric/elastic split of lateral load transfer (Eq. 6-27), camber
//  understeer K_camber (Eq. 6-56), roll steer K_roll_steer (Eq. 6-57), and
//  rollover (Section 9.5) - is therefore out of reach and is not faked.
//  See README -> Open Questions.
//
//  INTEGRATION
//  -----------
//  Explicit (forward) Euler for the vehicle states, matching the rest of the
//  project. The wheel rotational equation is the one exception and is treated
//  semi-implicitly; the reason is in the .cpp, and KB gap G35 records that
//  neither source offers any guidance on solver choice or stiffness.
//===========================================================================

#include "physics/IVehicleDynamics.h"
#include "physics/TireModel.h"

namespace vsim {

class FourWheelModel final : public IVehicleDynamics {
public:
    void update(VehicleState& state,
                const VehicleParameters& params,
                const DriveCommand& command,
                double dt,
                const TerrainHeightField* terrain = nullptr) override;

    const char* name() const override { return "Four-Wheel Tire Force"; }

    const char* assumptions() const override {
        return "Four individual tires, Magic Formula pure slip bounded by the "
               "friction circle, load transfer from longitudinal and lateral "
               "acceleration, rigid body in the ground plane. No roll, pitch "
               "or suspension - the KB does not quantify them.";
    }

    void reset() override {}
};

} // namespace vsim
