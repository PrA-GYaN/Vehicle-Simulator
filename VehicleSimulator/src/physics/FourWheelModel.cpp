#include "physics/FourWheelModel.h"

#include <cmath>

#include "core/Coordinates.h"
#include "environment/TerrainHeightField.h"
#include "physics/SteeringGeometry.h"
#include "physics/SuspensionModel.h"

namespace vsim {
namespace {

//---------------------------------------------------------------------------
//  NUMERICAL CONSTANTS. Neither is a physical parameter, and neither comes
//  from the Knowledge Base. They exist because two quantities the KB defines
//  are undefined at zero speed and the KB says so without saying what to do
//  about it.
//---------------------------------------------------------------------------

/// Floor on the speed used as the DENOMINATOR of slip ratio and slip angle.
///
/// KB Section 2.4.1 [LIMIT]: "V = 0 makes slip undefined." That limitation is
/// stated and never resolved - neither source gives a low-speed formulation
/// (see README -> Open Questions). Below this speed the reported slip is
/// therefore smaller than the definition would give, so the tires produce less
/// force and the vehicle behaves lazily at walking pace. That is a known,
/// bounded consequence of a documented singularity, not a tuning knob.
constexpr double kSlipReferenceSpeed = 1.0;   // [m/s]

/// Speed below which the vehicle is snapped to a full stop.
///
/// Retained from the kinematic model, which used the same guard for the same
/// reason: without it a decelerating vehicle approaches zero asymptotically
/// and creeps forever at 1e-9 m/s, which looks like a bug and pollutes
/// telemetry with meaningless motion.
constexpr double kStopSpeed = 1.0e-3;         // [m/s]

} // namespace

void FourWheelModel::update(VehicleState& state,
                            const VehicleParameters& params,
                            const DriveCommand& command,
                            double dt,
                            const TerrainHeightField* terrain) {
    if (!(dt > 0.0) || !isFinite(dt)) return;

    const TireModel tire(params.tire);

    const double m  = params.mass;
    const double L  = params.wheelbase;
    const double h  = params.cgHeight;
    const double tf = params.frontTrack;
    const double tr = params.rearTrack;
    const double Iz = params.effectiveYawInertia();
    const double Iw = (params.wheelInertia > 1e-6) ? params.wheelInertia : 1e-6;
    const double Ru = params.wheelRadius;

    // KB Section 1.8: b is the front axle to CG distance, c is CG to rear
    // axle, L = b + c. Note the lever-arm relationship the KB flags as a
    // frequent source of confusion - the FRONT axle load depends on c.
    const double b = params.frontAxleToCg();
    const double c = params.rearAxleToCg();

    //-----------------------------------------------------------------------
    //  0. Read the state, and the operating condition.
    //-----------------------------------------------------------------------
    const double psi0 = state.yaw;
    const double u0   = state.velocity;         // body x, forward   [m/s]
    const double v0   = state.lateralVelocity;  // body y, rightward [m/s]
    const double r0   = state.yawRate;          // yaw velocity      [rad/s]

    // World-frame heading/right, needed both by the terrain query below and
    // by the translational integration in step 6.
    const glm::dvec3 fwd0   = coords::heading(psi0);
    const glm::dvec3 right0 = coords::right(psi0);

    const double delta =
        clampAbs(sanitize(command.steerAngle), params.maxSteeringAngle);

    // Ackerman geometry puts the commanded centreline angle onto the two front
    // road wheels (KB Eq. 8-1/8-2). The rear wheels do not steer - four-wheel
    // steering (KB Section 8.10) is not modelled.
    const AckermanAngles front = ackermanSteerAngles(L, tf, delta);

    double steer[WheelCount] = {front.left, front.right, 0.0, 0.0};

    // Wheel positions relative to the CG, in the SAE vehicle frame
    // (x forward, y out the RIGHT side - KB Section 1.2).
    const double wx[WheelCount] = { b,        b,       -c,       -c      };
    const double wy[WheelCount] = {-tf * 0.5, tf * 0.5, -tr * 0.5, tr * 0.5};

    //-----------------------------------------------------------------------
    //  1. VERTICAL LOAD DISTRIBUTION - KB Section 3, "Layer 2".
    //
    //  KB Section 3 opens by saying why this comes before the tire: "the
    //  tire's entire capability scales with vertical load, and vertical load is
    //  NOT a static property."
    //
    //  Static, KB Eq. 1-6/1-7 (halved per wheel as in Eq. 2-21).
    //
    //  Longitudinal transfer, KB Eq. 1-8/1-9 in their working form:
    //      W_f = W_fs - W*(a_x/g)*(h/L)
    //      W_r = W_rs + W*(a_x/g)*(h/L)
    //  written with a SIGNED a_x, so braking (a_x < 0) moves load forward on
    //  its own. KB trap T2 warns that Chapter 3 switches to a positive
    //  deceleration D_x and reverses the signs; that form is deliberately not
    //  used here, so there is one convention and no chance of mixing them.
    //  In SI, W*(a_x/g) is just m*a_x.
    //
    //  Lateral transfer. This is the [S3] chain of KB Section 3.5.1, evaluated
    //  in physics/SuspensionModel: ride frequency -> ride rate -> roll rate ->
    //  roll gradient -> load transfer in THREE terms (elastic through the
    //  springs and bars, geometric through the links at the roll centre, and
    //  the unsprung mass's own transfer about its own CG height).
    //
    //  What used to be here was the total W*a_y*h/t split by an AUTHORED roll
    //  stiffness fraction. KB Section 3.5.2 is blunt about why that is not good
    //  enough: "Total lateral load transfer distribution is sometimes
    //  erroneously called 'roll couple distribution'. The problem is that roll
    //  couple distribution is only part of the mechanism." In [S3]'s own
    //  worked example the two differ by ten percentage points.
    //
    //  THE IMPLICIT COUPLING. KB Section 3.1 [LIMIT] says a_x appears in the
    //  equations that set the axle loads, and the axle loads set the traction
    //  limit that bounds a_x - "this coupling is implicit". The KB resolves it
    //  algebraically for the special cases it solves in closed form (Eq. 2-23,
    //  Eq. 3-27). Here it is resolved by using the PREVIOUS step's
    //  accelerations, which is what an explicit integrator does and is exact in
    //  the limit dt -> 0. The same applies to the roll angle.
    //-----------------------------------------------------------------------
    const double ax_prev = sanitize(state.acceleration);
    const double ay_prev = sanitize(state.lateralAcceleration);

    const double dWx = (L > 1e-6) ? m * ax_prev * h / L : 0.0;   // [N] to rear

    const SuspensionState susp = computeSuspension(params);

    //  Load transfer sensitivities are per g, so scale by a_y in g.
    const double ayG  = ay_prev / kGravity;
    const double dFzf = susp.loadTransferFrontPerG * ayG;
    const double dFzr = susp.loadTransferRearPerG * ayG;

    //  Body roll angle, quasi-static. Positive means leaning to the vehicle's
    //  RIGHT; the gradient is negative, so a right-hand turn (positive a_y)
    //  rolls the body LEFT - [S3] p.681: "the car rolls to the outside of the
    //  turn."
    const double phi = rollAngle(susp, ay_prev);

    double Fz[WheelCount];
    for (int i = 0; i < WheelCount; ++i) {
        const bool isFront = (i == FrontLeft || i == FrontRight);
        const bool isLeft  = (i == FrontLeft || i == RearLeft);

        double load = params.staticWheelLoad(i);

        // Longitudinal: accelerating (a_x > 0) unloads the front, loads the
        // rear. Half of the axle transfer lands on each wheel of that axle.
        load += (isFront ? -0.5 : +0.5) * dWx;

        // Lateral: a positive a_y is directed to the vehicle's RIGHT, i.e. the
        // vehicle is turning right, so the OUTSIDE wheels are the LEFT ones
        // and load transfers onto them. [S3] p.682 states the sign the same
        // way - "increased load on the outside or decreased load on the
        // inside" - and KB Eq. 6-27 is written as (outside load - inside load).
        const double dFz = isFront ? dFzf : dFzr;
        load += (isLeft ? +dFz : -dFz);

        // A tire cannot pull down. KB Section 2.6.4 notes the load-sensitivity
        // treatment "fails entirely" in the limit where the inside wheel lifts;
        // a lifted wheel carrying zero load and making zero force is the part
        // that is certainly right.
        Fz[i] = (load > 0.0) ? load : 0.0;
    }

    //-----------------------------------------------------------------------
    //  1b. TERRAIN-FOLLOWING SUSPENSION - per-wheel quarter car.
    //
    //  `Tireand suspension.md` §18 / [S3] RCVD Ch.22 p.787-791 (Fig 22.5):
    //  the "one corner" ride model is two masses in series - the sprung
    //  corner mass m_c on the ride spring k_s and damper C, sitting above the
    //  unsprung mass m_w on the tyre spring k_t to the road:
    //
    //      m_c * z_c'' = -k_s(z_c - z_w) - C(z_c' - z_w')
    //      m_w * z_w'' =  k_s(z_c - z_w) + C(z_c' - z_w') - k_t(z_w - z_road)
    //
    //  with C_crit = 2*sqrt(k_s*m_c) and zeta = C/C_crit ([S3] p.787-788,
    //  p.810).
    //
    //  NO GRAVITY TERM. This is deliberate, not an omission: gravity's STATIC
    //  effect is already the whole content of step 1 above (the quasi-static
    //  Fz chain the KB's own load-transfer sections define). This quarter
    //  car represents only the DYNAMIC response to a moving road input
    //  z_road(t) about that static equilibrium - exactly what [S3]'s own
    //  ride/vibration treatment of this model analyses. Superposing the two
    //  (static Fz + dynamic terrain delta) is licensed by `Tireand
    //  suspension.md` §3.5.4 / [S3] p.666: wheel-load changes from different
    //  causes may be "numerically added... to produce loads that are valid
    //  for combined operational conditions."
    //
    //  This also gives the model an exact, checkable no-op on every existing
    //  flat/procedural track: seeded to (z_c, z_w) = (z_road, z_road) with
    //  zero velocity, EVERY term on the right-hand side is proportional to a
    //  displacement or velocity that starts at exactly zero, so with
    //  z_road == 0 the state stays at (0,0,0,0) for the rest of the run,
    //  regardless of mass/rate/damping - see WheelState.h.
    //
    //  Decoupled per wheel, matching the source's own one-corner
    //  simplification - this does not attempt the coupled rigid-body
    //  pitch/roll equations of motion the KB does not supply (gaps G8/G9).
    //-----------------------------------------------------------------------
    double chassisH[WheelCount], chassisV[WheelCount];
    double wheelH[WheelCount],   wheelV[WheelCount];

    for (int i = 0; i < WheelCount; ++i) {
        const bool isFront = (i == FrontLeft || i == FrontRight);

        // Wheel world XZ, rear-axle-relative (the state's own reference
        // point - see VehicleState.h), NOT CG-relative like wx[]/wy[] above.
        const double axleX = isFront ? L : 0.0;
        const double worldX = state.position.x + fwd0.x * axleX + right0.x * wy[i];
        const double worldZ = state.position.z + fwd0.z * axleX + right0.z * wy[i];
        const double zRoad = terrain ? terrain->heightAt(worldX, worldZ) : 0.0;

        // Corner masses and rate straight from the already-computed
        // SuspensionState - see SuspensionModel.cpp: rideRateFront/Rear are
        // already PER-CORNER (KRF = wF^2 * (WsF/g) * 0.5), and
        // sprungWeight * sprungWeightFractionFront is WsF, the front sprung
        // axle weight, so halving it gives the front corner's own share.
        const double mc = isFront
            ? susp.sprungWeight * susp.sprungWeightFractionFront / kGravity * 0.5
            : susp.sprungWeight * (1.0 - susp.sprungWeightFractionFront) / kGravity * 0.5;
        const double mw = isFront
            ? susp.unsprungWeightFront / kGravity * 0.5
            : susp.unsprungWeightRear  / kGravity * 0.5;
        const double ks = isFront ? susp.rideRateFront : susp.rideRateRear;
        const double kt = params.tire.verticalRate;
        const double zeta = isFront ? params.suspension.dampingRatioFront
                                    : params.suspension.dampingRatioRear;
        const double cCrit = 2.0 * std::sqrt(std::max(0.0, ks * mc));
        const double cDamp = zeta * cCrit;

        // Seed on the first step after a reset - see the seeding note on
        // angular velocity above, same reasoning: start AT the local road
        // height with zero velocity rather than presenting a startup
        // transient as physics.
        double zc = state.wheelDataValid ? state.wheels[i].chassisHeight   : zRoad;
        double zcDot = state.wheelDataValid ? state.wheels[i].chassisVelocity : 0.0;
        double zw = state.wheelDataValid ? state.wheels[i].wheelHeight    : zRoad;
        double zwDot = state.wheelDataValid ? state.wheels[i].wheelVelocity  : 0.0;

        // Semi-implicit (symplectic) Euler - the same choice, for the same
        // reason, as the wheel rotational equation below: a lightly-damped
        // spring-mass system at wheel-hop frequencies (several Hz) is stiff
        // relative to a 10 ms step, and explicit Euler on it is unstable.
        const double mcSafe = std::max(mc, 1e-6);
        const double mwSafe = std::max(mw, 1e-6);

        const double springDamperForce = ks * (zc - zw) + cDamp * (zcDot - zwDot);
        const double zcAccel = -springDamperForce / mcSafe;
        const double zwAccel = (springDamperForce - kt * (zw - zRoad)) / mwSafe;

        const double zcDotNext = zcDot + dt * zcAccel;
        const double zwDotNext = zwDot + dt * zwAccel;
        const double zcNext = zc + dt * zcDotNext;
        const double zwNext = zw + dt * zwDotNext;

        chassisH[i] = sanitize(zcNext);
        chassisV[i] = sanitize(zcDotNext);
        wheelH[i]   = sanitize(zwNext);
        wheelV[i]   = sanitize(zwDotNext);

        // Dynamic delta on top of the static Fz already computed above.
        // Clamped to +-3x that wheel's static load as a numerical safety
        // guard against a degenerate/gapped mesh cell producing an
        // unreasonable road height - not physics, see TerrainHeightField.h.
        const double deltaFz = kt * (zRoad - wheelH[i]);
        const double clampMag = 3.0 * Fz[i];
        const double deltaFzClamped = clampAbs(deltaFz, std::max(clampMag, 1.0));

        Fz[i] = std::max(0.0, Fz[i] + deltaFzClamped);
    }

    //-----------------------------------------------------------------------
    //  2. DRIVE, BRAKE AND ROLLING RESISTANCE TORQUES.
    //
    //  There is no engine model, and there cannot be one: KB gap G17 records
    //  that the engine torque map "is an input throughout Chapter 2; no model
    //  or representative map is given". The project's existing
    //  `max_acceleration` and `max_deceleration` are kept as the demand, and
    //  converted into wheel torque through the loaded radius, which is the
    //  KB's own conversion (Eq. 3-19: F_b = T_b / r; Eq. 3-31: F_b = G*P_a/r).
    //-----------------------------------------------------------------------
    const double driveForceDemand  = m * std::max(0.0, sanitize(command.driveAcceleration));
    const double brakeForceDemand  = m * std::max(0.0, sanitize(command.brakeAcceleration));
    const double engineBrakeDemand = m * std::max(0.0, sanitize(command.engineBraking));
    const double frEffective       = std::max(0.0, sanitize(command.rollingResistanceCoeff));

    // Speed governor. The project has always capped top speed; with no engine
    // map there is no physical power ceiling to produce one (KB Section 5.3
    // needs a torque curve, gap G17). Implemented as a fuel cut rather than by
    // clamping the velocity state, because clamping a state in a force-based
    // model destroys momentum.
    //
    // The cap is applied to WHEEL surface speed, not road speed. That is the
    // same physical statement - KB Section 5.4/5.8 put the engine and the
    // wheels in a fixed ratio through the driveline, so a maximum engine speed
    // in a given gear IS a maximum wheel speed - but it is applied where the
    // torque is actually made. Without it a spinning wheel has nothing to stop
    // it: KB Section 2.4.2's falling branch means the ground reaction beyond
    // the peak is smaller than the torque driving it, so a constant-torque
    // source spins the wheel up without bound.
    const bool governed = (u0 >= params.maxSpeed);

    int drivenCount = 0;
    for (int i = 0; i < WheelCount; ++i) {
        if (params.isDrivenWheel(i)) ++drivenCount;
    }

    //-----------------------------------------------------------------------
    //  3. PER-WHEEL: slip state -> tire forces -> wheel rotational dynamics.
    //-----------------------------------------------------------------------
    double Fx_body_total = 0.0;   // [N] along vehicle x
    double Fy_body_total = 0.0;   // [N] along vehicle y (right)
    double Mz_total      = 0.0;   // [N.m] about vehicle z (down) - yaw

    WheelDynamics out[WheelCount];

    for (int i = 0; i < WheelCount; ++i) {
        WheelDynamics& w = out[i];
        w.verticalLoad = Fz[i];
        w.steerAngle   = steer[i];
        w.chassisHeight   = chassisH[i];
        w.chassisVelocity = chassisV[i];
        w.wheelHeight     = wheelH[i];
        w.wheelVelocity   = wheelV[i];

        const bool isFront = (i == FrontLeft || i == FrontRight);
        const bool isLeft  = (i == FrontLeft || i == RearLeft);

        const double Re = tire.effectiveRollingRadius(Fz[i], Ru);
        const double Rl = tire.loadedRadius(Fz[i], Ru);
        w.effectiveRollingRadius = Re;
        w.loadedRadius           = Rl;

        //-- 3a. Velocity of this contact patch, in the vehicle frame.
        //
        //  Rigid-body velocity composition: v_P = v_G + omega x r_P. With
        //  omega = (0, 0, r) in the SAE frame (z down, KB Section 1.2) and
        //  r_P = (x_w, y_w, 0):
        //      u_w = u - r*y_w
        //      v_w = v + r*x_w
        //  The KB assumes a rigid vehicle throughout (Section 1.6 lumped mass,
        //  Section 3.1 [ASSUME]) and [S2] Chapter 2 covers rigid-body velocity
        //  analysis, but neither source writes this equation out. It is noted
        //  in README -> Open Questions as a derivation rather than a citation.
        const double uw = u0 - r0 * wy[i];
        const double vw = v0 + r0 * wx[i];

        //-- 3b. Rotate into the TIRE axis system (KB Section 1.5).
        //
        //  X' is the intersection of the wheel plane with the road, positive
        //  forward; Y' completes the right-hand system, positive right.
        const double cd = std::cos(steer[i]);
        const double sd = std::sin(steer[i]);
        const double Vx =  uw * cd + vw * sd;   // along the wheel plane
        const double Vy = -uw * sd + vw * cd;   // across it

        // Denominator guard - see kSlipReferenceSpeed.
        const double Vref = std::max(std::abs(Vx), kSlipReferenceSpeed);

        //-- 3c. Slip angle (KB Section 1.5): the angle between the direction
        //  of wheel travel and the wheel plane. Gillespie's sign convention
        //  (KB trap T4): positive slip angle -> positive lateral force.
        //
        //  The magnitude of Vx is used rather than its signed value so that a
        //  vehicle rolling backwards measures its slip angle against the wheel
        //  plane rather than reporting ~180 degrees. Neither source treats a
        //  reversing wheel - KB Section 2.17 notes it took the Delft Tyre 97
        //  version of the Magic Formula to lift that restriction.
        const double alpha = -std::atan2(Vy, Vref);
        w.slipAngle = alpha;

        //-- 3d. Slip ratio.
        //
        //  KB Eq. 10-1 / Eq. 3-20, braked-wheel form:
        //      S = (V - R_e*omega) / V
        //  0 = free rolling, +1 = locked, negative under driving (KB Section
        //  2.4.1: "under driving, r*omega > V and the expression goes
        //  negative; the book uses the braking form as the reference
        //  definition"). KB Section 2.16 warns that the DRIVING-side
        //  normalisation differs between authors and "must be confirmed, not
        //  assumed" - so the single KB form is used for both, unchanged.
        //
        //  R_e, not R_u and not R_l: KB Section 2.15 is explicit that using
        //  either of the others produces a spurious force on a freely rolling
        //  tire.
        //
        //  SEEDING. On the first step after a reset (or after switching from a
        //  model that has no wheels) there is no wheel speed to carry forward.
        //  It is seeded to the free-rolling value V/R_e, which by the
        //  definition of the effective rolling radius is exactly the speed
        //  that makes the slip ratio zero. Starting from zero instead would
        //  present every wheel as locked and brake the vehicle to a halt in
        //  the first few steps - an artefact of initialisation, not physics.
        const double omega = state.wheelDataValid
                                 ? state.wheels[i].angularVelocity
                                 : ((Re > 1e-6) ? Vx / Re : 0.0);

        const double slipS = (Vx - Re * omega) / Vref;
        w.slipRatio      = slipS;
        w.angularVelocity = omega;

        //-- 3e. Inclination (camber) angle.
        //
        //  KB Eq. 6-50:  gamma_g = gamma_b + phi  - camber relative to the
        //  ground is camber relative to the body plus the body roll angle.
        //  gamma_b is the static alignment plus whatever the suspension's
        //  camber-vs-roll characteristic adds, KB Eq. 6-51's d(gamma)/d(phi).
        //
        //  That characteristic is gap G13, PARTIAL in all three sources: [S1]
        //  leaves it as an unspecified function, [S3] gives the RATE as
        //  arctan(1/fvsa length) but no source gives an fvsa length for any
        //  vehicle. The parameter therefore defaults to zero, which is the
        //  honest "no camber compensation" case - the wheel simply leans with
        //  the body - rather than an invented compensation figure.
        //
        //  Sign: positive gamma means the top of the wheel leans toward the
        //  vehicle's RIGHT, producing camber thrust to the right - "a lateral
        //  force in the direction of the tilt" [S3] p.46. Static NEGATIVE
        //  camber (tops inboard) is therefore +|c| on the left wheel and -|c|
        //  on the right, which cancels in a straight line and helps the loaded
        //  outside tyre in a corner.
        const double camberFromRoll =
            phi * (1.0 + params.suspension.camberChangePerRoll);
        const double staticTerm =
            isLeft ? -params.tire.staticCamber : +params.tire.staticCamber;
        const double gamma = camberFromRoll + staticTerm;
        w.camberAngle = gamma;

        //-- 3f. The tire block.
        TireInputs in;
        in.verticalLoad         = Fz[i];
        in.ratedLoad            = params.staticWheelLoad(i);
        in.slipAngle            = alpha;
        in.slipRatio            = -slipS;              // driving positive [S3]
        in.camberAngle          = gamma;
        in.rollingSpeed         = std::abs(Vx);
        in.unloadedRadius       = Ru;
        in.previousLateralForce = state.wheels[i].lateralForce;
        in.dt                   = dt;

        // This wheel's tyre: the vehicle's tyre as the selected compound,
        // at this wheel's own state of wear (config/TireCompound.h). On the
        // Standard compound this is `tire` itself, unchanged.
        const double wearNow = state.wheelDataValid ? state.wheels[i].wear : 0.0;
        const bool compounded = params.tireCompound != TireCompound::Standard;
        const TireModel wornTire(compounded
            ? applyTireCompound(params.tire, params.tireCompound, wearNow) : params.tire);

        const TireForces f = wornTire.evaluate(in);

        // Wear from sliding work. The patch slides longitudinally at
        // (V_x - R_e*omega) and laterally at V_y (both in the wheel's own
        // frame); force times
        // sliding speed is the power turned into heat and abraded rubber.
        // A freely rolling tyre has neither and wears nothing.
        w.wear = wearNow;
        if (compounded) {
            const double slideX = std::abs(Vx - Re * omega);
            const double slideY = std::abs(Vy);
            const double power = std::abs(f.longitudinal) * slideX + std::abs(f.lateral) * slideY;
            const double life = tireCompoundSpec(params.tireCompound).lifeEnergy;
            if (life > 0.0) w.wear = std::min(1.0, wearNow + power * dt / life);
        }

        w.longitudinalForce = f.longitudinal;
        w.lateralForce      = f.lateral;
        w.aligningMoment    = f.aligningMoment;
        w.frictionAvailable = f.frictionAvailable;
        w.frictionUsage     = f.frictionUsage;

        //-- 3g. Wheel rotational dynamics.
        //
        //  KB Eq. 3-18, rearranged:
        //      F_b = (T_b - I_w*alpha_w)/r    =>    I_w*alpha_w = T_b - F_b*r
        //  and generally, with drive torque, brake torque, the rolling
        //  resistance moment and the ground reaction:
        //
        //      I_w * omega_dot = T_drive - T_brake - M_roll - F_x*R_l
        //
        //  KB Section 6.7 [LIMIT] is explicit that the simplified F_b = T_b/r
        //  "is valid only while the wheel is NOT undergoing significant
        //  angular acceleration. During lockup or ABS cycling, I_w*alpha_w is
        //  large and Eq. 3-18 is required." Wheel lock and wheel spin ARE that
        //  regime, so the full form is what is integrated.
        //
        //  Rolling resistance enters as a MOMENT, which KB Section 2.5.6 says
        //  is what it fundamentally is: "rolling resistance is NOT a
        //  horizontal force applied at the contact patch - it is the
        //  consequence of an asymmetric pressure distribution. The force form
        //  (f_r*W) is the convenient equivalent." Magnitude f_r*F_z*R_l, from
        //  Eq. 4-12 with the J670e moment relation.
        const bool overspeed = (Re * omega >= params.maxSpeed);

        double Tdrive = 0.0;
        if (!governed && !overspeed && drivenCount > 0 && params.isDrivenWheel(i)) {
            // Equal torque to each driven wheel. KB Section 3.4 states that an
            // open differential is limited by its lightly loaded wheel -
            // "both drive wheels can only transmit twice what the lightly
            // loaded wheel can transmit" - which is the equal-torque split.
            // No limited-slip or locking behaviour is modelled; KB gap G19
            // records that no source gives a differential model.
            Tdrive = driveForceDemand * Rl / drivenCount;
        }

        // Brake torque. KB Section 6.8.2: a fixed hydraulic system splits
        // total braking effort in a fixed front/rear proportion.
        const double share = params.effectiveBrakeProportionFront();
        const double axleShare = isFront ? share : (1.0 - share);
        double Tbrake = brakeForceDemand * axleShare * 0.5 * Rl;

        // Engine braking is a driveline torque, so it acts on the driven
        // wheels. It is flagged in RoadLoad.cpp as NOT from the source, and
        // that flag stands: KB gap G17 has no engine model of any kind.
        if (drivenCount > 0 && params.isDrivenWheel(i)) {
            Tbrake += engineBrakeDemand * Rl / drivenCount;
        }

        const double Mroll = frEffective * Fz[i] * Rl;

        //  Brake and rolling-resistance torques are RESISTIVE: they oppose
        //  rotation, and at rest they oppose whatever is trying to start it,
        //  up to their own magnitude - a brake holds a stationary wheel. They
        //  must never drive the wheel backwards through zero, which would be
        //  the discretisation reversing the wheel rather than physics.
        //  The ground force's moment arm about the wheel centre is the LOADED
        //  radius, not the effective rolling radius. KB Section 2.5.6 states
        //  the torque balance in exactly that form:  T = (F_x + F_r) * R_l.
        //  R_e is a kinematic quantity - the radius that makes slip zero - and
        //  using it as a lever arm would put a few percent of error into every
        //  torque-to-force conversion in the model.
        const double Tres = Tbrake + Mroll;                  // magnitude >= 0
        const double Tapplied = Tdrive - f.longitudinal * Rl;

        double Tnet;
        if (std::abs(omega) > 1e-9) {
            Tnet = Tapplied - std::copysign(Tres, omega);
        } else if (std::abs(Tapplied) <= Tres) {
            Tnet = 0.0;                     // held: the brake wins at standstill
        } else {
            Tnet = Tapplied - std::copysign(Tres, Tapplied);
        }

        //  SEMI-IMPLICIT UPDATE, and why.
        //
        //  Linearised about the current slip, the wheel equation has time
        //  constant  tau = I_w * V / (C_s * R_e^2). For a road car at 10 m/s
        //  that is a couple of milliseconds - far below a 10 ms physics step -
        //  so explicit Euler on it oscillates and diverges. KB gap G35 records
        //  that neither source discusses solver choice or stiffness, and notes
        //  that a stiff fast mode alongside a soft slow one "is inherently a
        //  stiff numerical problem".
        //
        //  The whole net torque is therefore advanced with one Newton step
        //  against the ground-reaction term's Jacobian,
        //      dF_x/d(omega) = (dF_x/d slip) * R_e / V
        //  taken from the tire's own LOCAL slope. The divisor cannot move the
        //  fixed point - at equilibrium Tnet is zero and the divisor multiplies
        //  zero - so this changes the path to the answer and not the answer.
        //  As dt -> 0 the divisor tends to 1 and the update becomes explicit.
        //
        //  Using the local slope rather than the origin slope matters: past
        //  the peak of the mu-slip curve the slope is zero or negative, the
        //  divisor drops to 1, and the wheel is free to run away to lockup or
        //  spin exactly as KB Section 2.4.2 says it must.
        const double slope = (f.longitudinalSlipSlope * Re) / Vref;  // N/(rad/s)
        const double damp  = 1.0 + dt * slope * Rl / Iw;

        double omegaNext = omega + dt * Tnet / (Iw * damp);

        // A resistive torque may bring the wheel to rest but never past it.
        if (std::abs(omega) > 1e-9 && omegaNext * omega < 0.0) {
            const double omegaFree = omega + dt * Tapplied / (Iw * damp);
            if (omegaFree * omega >= 0.0) omegaNext = 0.0;
        }

        w.angularVelocity = sanitize(omegaNext);

        //-- 3h. Lock and spin indicators (KB Section 2.4.2).
        //
        //  Nothing here is a threshold invented to make an effect appear. The
        //  peak of the mu-slip curve is a parameter of the tire model, and the
        //  KB identifies the region beyond it as the one with NEGATIVE slope,
        //  "the physical origin of wheel-lockup instability": past the peak, an
        //  increase in slip REDUCES the force resisting further deceleration
        //  of the wheel, so the wheel spins down to lockup essentially
        //  instantaneously.
        const bool moving = std::abs(Vx) > kSlipReferenceSpeed * 0.1;
        w.beyondFrictionPeak = moving && std::abs(slipS) > tire.peakSlipRatio();
        w.locked   = moving && std::abs(w.angularVelocity) < 1e-3;
        w.spinning = params.isDrivenWheel(i) && (slipS < -tire.peakSlipRatio());

        //-- 3i. Accumulate into the vehicle frame.
        //
        //  Rotation of the tire-frame force by the steer angle (KB Section 1.9
        //  rotation matrices, reduced to a single rotation about z):
        //      F_x,body = F_x*cos(delta) - F_y*sin(delta)
        //      F_y,body = F_x*sin(delta) + F_y*cos(delta)
        const double fxb = f.longitudinal * cd - f.lateral * sd;
        const double fyb = f.longitudinal * sd + f.lateral * cd;

        Fx_body_total += fxb;
        Fy_body_total += fyb;

        //  Yaw moment about the CG of a force applied at (x_w, y_w), plus the
        //  tire's own aligning moment (KB Section 2.8). Including M_z is
        //  equivalent to moving the lateral force back by the pneumatic trail,
        //  which is exactly what the trail physically is.
        Mz_total += wx[i] * fyb - wy[i] * fxb + f.aligningMoment;
    }

    //-----------------------------------------------------------------------
    //  4. NON-TIRE FORCES.
    //
    //  KB Section 1.7: "a road vehicle has essentially only one way to
    //  generate a force: the tire contact patch, PLUS aerodynamics and
    //  gravity." Those two are the whole of this section.
    //
    //  Aerodynamic drag opposes motion (KB Eq. 4-2, computed in RoadLoad.cpp).
    //  It is applied at the CG. KB Eq. 1-3 carries a D_A*h_a term for the
    //  pitching effect of drag acting at its own height h_a, but no source
    //  gives h_a, and the KB's own working form (Eq. 1-8/1-9) drops that term
    //  as well - which is the form used in section 1 above.
    //
    //  Grade is signed. The existing convention is preserved: RoadLoad's
    //  `grade` is positive when it resists motion.
    //-----------------------------------------------------------------------
    const double dragForce = m * std::max(0.0, sanitize(command.aerodynamicDrag));
    if (std::abs(u0) > 1e-6) {
        Fx_body_total -= std::copysign(dragForce, u0);
    }
    Fx_body_total -= m * sanitize(command.gradeAcceleration);

    //-----------------------------------------------------------------------
    //  5. NEWTON'S SECOND LAW - KB Section 1.7, Eq. 1-1 and Eq. 1-2.
    //-----------------------------------------------------------------------
    const double ax = Fx_body_total / m;    // body x [m/s^2]
    const double ay = Fy_body_total / m;    // body y [m/s^2]

    //  Rotational form. KB Eq. 1-2 is ΣT = I*alpha; KB Eq. 2.236 [S2] gives
    //  the general Euler equation
    //      M_z = I_zz*omega_dot_z + (I_yy - I_xx)*omega_x*omega_y
    //  whose coupling term needs TWO simultaneous non-zero angular rates. In
    //  planar motion roll and pitch rates are identically zero, so the term
    //  vanishes and the two forms agree exactly. KB Section 1.12 makes this
    //  point directly: "in a steady turn on a flat road only yaw rate is
    //  significant, so the coupling is negligible."
    const double rdot = Mz_total / Iz;

    //-----------------------------------------------------------------------
    //  6. INTEGRATE.
    //
    //  Translation in the GLOBAL frame, rotation in the body frame - the split
    //  KB Section 1.11 [S2] recommends, and the reason no transport/Coriolis
    //  term has to be written by hand. Forward Euler throughout, evaluated at
    //  the start of the interval, matching the kinematic model's convention so
    //  that the two models integrate identically and differ only in physics.
    //-----------------------------------------------------------------------
    // The state's reference point is the REAR AXLE; the body is rigid, so the
    // CG is a fixed offset along the heading and converting back and forth is
    // exact.
    const glm::dvec3 cg0 = state.position + fwd0 * c;

    glm::dvec3 vel0 = fwd0 * u0 + right0 * v0;
    const glm::dvec3 accWorld = fwd0 * ax + right0 * ay;

    glm::dvec3 vel1 = vel0 + accWorld * dt;
    const glm::dvec3 cg1 = cg0 + vel0 * dt;

    const double psi1 = coords::wrapAngle(psi0 + r0 * dt);
    const double r1   = r0 + rdot * dt;

    // Recover the body-frame components against the NEW heading. This is where
    // the rotating frame enters, and it enters for free.
    const glm::dvec3 fwd1   = coords::heading(psi1);
    const glm::dvec3 right1 = coords::right(psi1);

    double u1 = glm::dot(vel1, fwd1);
    double v1 = glm::dot(vel1, right1);

    // Stop snap - see kStopSpeed.
    if (std::sqrt(u1 * u1 + v1 * v1) < kStopSpeed) {
        u1 = 0.0;
        v1 = 0.0;
        vel1 = glm::dvec3(0.0);
    }

    //-----------------------------------------------------------------------
    //  7. COMMIT.
    //-----------------------------------------------------------------------
    // position.y is deliberately left untouched by terrain: it stays on the
    // flat ground-plane the rigid body is integrated on (fwd0/right0 are
    // horizontal by construction, so nothing above ever gives it a vertical
    // component - see Vehicle::cgPosition()'s comment). Real elevation is
    // carried entirely by Vehicle::terrainHeave() (this function's own
    // chassisH[], averaged) and applied once, at the point of use
    // (Renderer's heavePos) - adding it here too would double it.
    state.position = cg1 - fwd1 * c;
    state.yaw      = psi1;
    state.yawRate  = r1;

    state.velocity            = u1;
    state.lateralVelocity     = v1;
    state.acceleration        = ax;
    state.lateralAcceleration = ay;
    state.steeringAngle       = delta;
    state.rollAngle           = phi;

    //  Sideslip angle beta - KB Section 1.3.
    //
    //  "The angle between the vehicle's x axis and the velocity vector. The
    //  difference between pointing and going", with beta = nu - psi. Signed in
    //  the SAE sense used throughout this project: positive means the velocity
    //  vector lies to the vehicle's RIGHT of its heading.
    //
    //  Note that Gillespie's Eq. 6-23 works in magnitudes with beta positive
    //  "into the turn" (KB Section 9.4.7), which is the opposite sign for a
    //  right-hand turn. KB Section 13.3 documents several such magnitude-
    //  versus-signed conflicts in the source; this file uses the signed
    //  definition of Section 1.3 consistently and never mixes the two.
    const double speed = std::sqrt(u1 * u1 + v1 * v1);
    state.sideslipAngle = (speed > kStopSpeed) ? std::atan2(v1, u1) : 0.0;

    //  Body slip angle rate - KB Eq. 7.29 [S2], Section 9.4.15:
    //      beta_dot = A_y / V - omega
    //  the part of the yaw motion that is NOT path-following. Identically zero
    //  in steady state, which is precisely why the KB's steady-state framework
    //  cannot address it.
    state.sideslipRate =
        (speed > kSlipReferenceSpeed) ? (ay / speed - r1) : 0.0;

    for (int i = 0; i < WheelCount; ++i) state.wheels[i] = out[i];
    state.wheelDataValid = true;

    state.time += dt;
    state.sanitizeAll();
}

} // namespace vsim
