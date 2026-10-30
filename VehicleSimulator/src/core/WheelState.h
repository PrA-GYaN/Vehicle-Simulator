#pragma once
//===========================================================================
//  WheelState.h  --  per-wheel state, as the Knowledge Base defines it.
//
//  KNOWLEDGE BASE BASIS
//  --------------------
//  KB Section 2.2 ("The tire as a model block") lists exactly what a tire
//  consumes and produces:
//
//      INPUTS   Fz (vertical load), alpha (slip angle), gamma (inclination),
//               slip ratio, mu available, inflation pressure, V
//      OUTPUTS  Fx, Fy, Mz, My, Mx
//
//  This struct carries those quantities, plus the wheel rotational speed that
//  the slip ratio is defined against (KB Eq. 10-1) and the steer angle that
//  orients the tire axis system (KB Section 1.5).
//
//  Quantities NOT carried, and why:
//    * gamma (inclination/camber). Camber relative to the ground is
//      gamma_g = gamma_b + phi (KB Eq. 6-50), which needs the body roll angle
//      phi and the suspension's camber-vs-roll characteristic d(gamma)/d(phi).
//      KB Eq. 6-51 states that characteristic only as an unspecified function
//      f_gamma(track, geometry, roll angle) - it is gap G13, PARTIAL. There is
//      no roll model here either (see README, Open Questions), so camber is
//      structurally zero rather than assumed.
//    * Mx (overturning moment) and My (rolling resistance moment as an output).
//      My is applied as an input torque to the wheel (KB Section 2.5.6); it is
//      not reported as a tire output because nothing consumes it.
//
//  SIGN CONVENTIONS - fixed here once, never re-decided elsewhere
//  --------------------------------------------------------------
//  KB Section 13.3 trap T3: SAE axes have z DOWN, but SAE J670e defines
//  Vertical Force = -Normal Force so that tire load is POSITIVE UP. This file
//  follows J670e: `verticalLoad` is positive up, always >= 0.
//
//  KB Section 13.3 trap T4: Gillespie (Eq. 6-5) writes Fy = C_alpha * alpha,
//  J670e writes Fy = -C_alpha * alpha. Both use a positive C_alpha; the KB
//  says explicitly they must not be mixed. THIS PROJECT USES GILLESPIE'S
//  CONVENTION, because every downstream relation in the KB that this code
//  implements (Eq. 6-12/6-13, 6-15, 6-49) is written in it:
//
//      positive slip angle  ->  positive lateral force
//      positive lateral force  ->  toward the vehicle's RIGHT (+y, SAE)
//
//  Longitudinal force follows the KB Section 1.5 table: tractive positive,
//  braking negative.
//===========================================================================

#include "core/Units.h"

namespace vsim {

/// Indices into every four-wheel array in the project. Front/rear, left/right.
enum WheelIndex : int {
    FrontLeft  = 0,
    FrontRight = 1,
    RearLeft   = 2,
    RearRight  = 3,
    WheelCount = 4
};

/// Instantaneous state of one tire. Every field is either an input the KB's
/// tire block consumes or an output it produces.
struct WheelDynamics {
    //-- tire block inputs (KB Section 2.2) ------------------------------

    /// Vertical load Fz [N], POSITIVE UP (KB trap T3, SAE J670e).
    /// Static load plus longitudinal transfer (KB Eq. 1-8/1-9) plus lateral
    /// transfer (KB Section 3.5). Clamped at zero: a tire cannot pull down.
    double verticalLoad = 0.0;

    /// Slip angle alpha [rad] - the angle between the direction of wheel
    /// travel and the wheel plane (KB Section 1.5). Gillespie sign convention.
    double slipAngle = 0.0;

    /// Inclination (camber) angle gamma [rad] - the angle between the wheel
    /// plane and the vertical (KB Section 1.5). POSITIVE means the top of the
    /// wheel leans toward the vehicle's RIGHT, which is the direction the
    /// resulting camber thrust acts in.
    ///
    /// Comes from the body roll angle plus the static alignment: KB Eq. 6-50,
    /// gamma_g = gamma_b + phi. See physics/FourWheelModel.cpp step 3e.
    double camberAngle = 0.0;

    /// Slip ratio in the Knowledge Base's own braked-wheel form,
    ///     S = (V - R_e * omega) / V          (KB Eq. 10-1 / Eq. 3-20)
    /// so 0 = free rolling, +1 = locked, negative = driving.
    /// Reported in this form so it can be read against the KB directly.
    double slipRatio = 0.0;

    /// Friction coefficient available at THIS wheel's current load [-].
    /// Load-sensitive: KB Section 2.4.3 gives mu falling ~0.01 per 10 % load
    /// increase near the tire's rated load.
    double frictionAvailable = 0.0;

    //-- wheel rotational state ------------------------------------------

    /// Wheel angular velocity omega [rad/s]. Positive = rolling forward.
    /// A genuine [STATE]: integrated from KB Eq. 3-18 rearranged.
    double angularVelocity = 0.0;

    /// Road wheel steer angle delta [rad], positive = right.
    /// Per wheel, from Ackerman geometry (KB Eq. 8-1/8-2).
    double steerAngle = 0.0;

    /// Effective rolling radius R_e [m] (KB Section 2.15, Eq. 5.11).
    /// This is the radius the slip ratio is defined against - NOT the
    /// unloaded and NOT the loaded radius.
    double effectiveRollingRadius = 0.0;

    /// Loaded radius R_l [m] (KB Section 2.15). The torque-to-force lever arm.
    double loadedRadius = 0.0;

    //-- tire block outputs ----------------------------------------------

    /// Longitudinal force Fx [N] in the TIRE axis system: tractive positive,
    /// braking negative (KB Section 1.5).
    double longitudinalForce = 0.0;

    /// Lateral force Fy [N] in the TIRE axis system, positive to the
    /// vehicle's right for a positive slip angle (Gillespie convention).
    double lateralForce = 0.0;

    /// Aligning moment Mz [N.m] = -Fy * p (KB Section 2.8).
    double aligningMoment = 0.0;

    //-- derived indicators (display and telemetry only) -----------------

    /// Fraction of the friction circle in use:
    ///     sqrt(Fx^2 + Fy^2) / (mu * Fz)          (KB Section 2.9)
    /// 1.0 means the tire is exactly on the friction circle.
    double frictionUsage = 0.0;

    /// True when the tire is past the peak of the mu-slip curve, i.e. in the
    /// NEGATIVE-SLOPE region KB Section 2.4.2 identifies as the physical
    /// origin of lockup instability.
    bool beyondFrictionPeak = false;

    /// True when the wheel has stopped rotating while the vehicle is still
    /// moving - the 100 % slip endpoint of KB Eq. 10-1.
    bool locked = false;

    /// True when a DRIVEN wheel is turning faster than the road speed by more
    /// than the peak-slip location, i.e. driving past the peak of the mu-slip
    /// curve (KB Section 2.4.2).
    bool spinning = false;

    /// Tyre wear, 0 (new) to 1 (finished). A [STATE]: accumulated from the
    /// energy the contact patch dissipates in sliding, at a rate set by the
    /// compound. Stays 0 on the Standard compound, which does not wear.
    /// See config/TireCompound.h - an authored model, not a sourced one.
    double wear = 0.0;

    //-- terrain-following quarter-car state ------------------------------
    //
    // A decoupled one-corner ride model per wheel (`Tireand suspension.md`
    // §18, [S3] RCVD Ch.22, Fig 22.5): body-mass-on-spring-and-damper sitting
    // above an unsprung mass on a tyre spring to the ground. World-space Y
    // and its rate, absolute (not spawn-relative). See
    // physics/FourWheelModel.cpp step 1/3 for the integration and
    // environment/TerrainHeightField.h for where the ground height comes
    // from. On a flat track (terrain == nullptr, z_road == 0 everywhere)
    // these four states are seeded to exactly (0,0,0,0), which is an exact
    // fixed point of the system regardless of mass/rate/damping - so every
    // existing vehicle config is numerically unaffected by their presence.
    double chassisHeight   = 0.0;  ///< [STATE] z_c, this corner's sprung mass height [m]
    double chassisVelocity = 0.0;  ///< [STATE] d(z_c)/dt [m/s]
    double wheelHeight     = 0.0;  ///< [STATE] z_w, unsprung mass (wheel) height [m]
    double wheelVelocity   = 0.0;  ///< [STATE] d(z_w)/dt [m/s]

    void sanitizeAll() {
        verticalLoad           = vsim::sanitize(verticalLoad);
        slipAngle              = vsim::sanitize(slipAngle);
        camberAngle            = vsim::sanitize(camberAngle);
        slipRatio              = vsim::sanitize(slipRatio);
        frictionAvailable      = vsim::sanitize(frictionAvailable);
        angularVelocity        = vsim::sanitize(angularVelocity);
        steerAngle             = vsim::sanitize(steerAngle);
        effectiveRollingRadius = vsim::sanitize(effectiveRollingRadius);
        loadedRadius           = vsim::sanitize(loadedRadius);
        longitudinalForce      = vsim::sanitize(longitudinalForce);
        lateralForce           = vsim::sanitize(lateralForce);
        aligningMoment         = vsim::sanitize(aligningMoment);
        frictionUsage          = vsim::sanitize(frictionUsage);
        chassisHeight          = vsim::sanitize(chassisHeight);
        chassisVelocity        = vsim::sanitize(chassisVelocity);
        wheelHeight            = vsim::sanitize(wheelHeight);
        wheelVelocity          = vsim::sanitize(wheelVelocity);
        wear                   = vsim::sanitize(wear);
    }
};

/// Which axle receives drive torque.
///
/// KB Section 5.7.3 derives the traction limit separately for a front drive
/// axle and a rear drive axle, and the two differ in sign of the load-transfer
/// term - so the distinction is a real one the KB makes. The KB does NOT cover
/// all-wheel drive (no torque-split model exists in either source; gap G19), so
/// it is not offered here. See README, Open Questions.
enum class DriveLayout {
    RearWheelDrive,
    FrontWheelDrive
};

} // namespace vsim
