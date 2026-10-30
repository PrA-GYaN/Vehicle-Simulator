#pragma once
//===========================================================================
//  TireModel.h  --  the behaviour of ONE tire, and nothing else.
//
//  WHY THIS IS ITS OWN MODULE
//  --------------------------
//  KB §2 is titled "Layer 1 - The Tire" and opens by saying the tire is "not a
//  component of the vehicle; it is the entire interface between the vehicle's
//  internal machinery and the outside world". KB §2.2 draws the tire as a block
//  with named inputs and named outputs. This file is that block. Replacing the
//  tire model means replacing this file; nothing in the vehicle model reaches
//  inside it.
//
//  WHICH MODEL THIS IS
//  -------------------
//  The NONDIMENSIONAL tyre model of KB §2.20 - [S3] Milliken & Milliken, RCVD
//  Chapter 14, Eqs. (14.1)-(14.21). Full treatment in
//  `Tireand suspension.md` §10-§11.
//
//  It replaces the earlier arrangement (a per-load Magic Formula fit for each
//  axis, with the friction circle applied afterwards as a clamp). The reason is
//  not that the earlier one was wrong - KB §2.20.2 says the two "do not
//  conflict", [S2] fitting raw curves per load and [S3] fitting one curve for
//  all loads - but that this one answers a question the earlier one could not:
//
//      KB gap G1, the largest recorded, asked for a model returning (F_x, F_y)
//      for an arbitrary (slip ratio, slip angle, F_z, camber) combination.
//      [S1] gave a capability BOUNDARY with no equation; [S2] gave Fiala,
//      which its own authors say "cannot represent combined cornering and
//      braking". [S3] supplies a complete model, and G1 is now RESOLVED.
//
//  The consequence for this file is worth stating plainly: **the friction
//  ellipse is no longer enforced, it is a property.** Eqs. (14.20)/(14.21)
//  satisfy F_bar^2 + F_bar_x^2 = R(k)^2 identically, and R never exceeds
//  D' = 1, so
//
//      (F_y/(mu_y F_z))^2 + (F_x/(mu_x F_z))^2  <=  1
//
//  falls out of the algebra. Nothing is clamped and no allocation rule has to
//  be invented. When mu_x = mu_y this is exactly [S1] §2.9's "pure-circle
//  case".
//
//  THE PHYSICAL IDEA BEHIND IT  -  KB §2.20.1, [S3] p.44
//  ----------------------------------------------------
//      "Although it is convenient to think in terms of longitudinal and
//       lateral forces, and slip ratio and slip angle, the tire only knows a
//       RESULTANT FORCE and a RESULTANT SLIP VELOCITY."
//
//  Data taken at five slip angles and five slip ratios collapses onto a single
//  curve when plotted that way. Everything below is the bookkeeping needed to
//  get in and out of that one scalar relationship.
//
//  WHAT THIS BLOCK COMPUTES, AND FROM WHICH KB EQUATION
//  ----------------------------------------------------
//   1. Loaded and effective rolling radii     KB §2.15, Eq. 5.11 [S2]
//   2. Available friction at this load        KB §2.4.3 [S1]
//   3. Cornering stiffness at this load       KB Eq. 6-6, 6-35/6-36 [S1]
//   4. Camber stiffness at this load          KB §12.2 [S3]
//   5. Longitudinal slip stiffness k_x        derived - see slipStiffnessAt()
//   6. Normalised slip, camber and slip ratio KB §2.20.2, Eqs. (14.4/8/10)
//   7. The universal curve                    Eqs. (14.5)-(14.7)
//   8. Combined slip -> (F_x, F_y)            Eqs. (14.13)-(14.21)
//   9. Relaxation-length lag on F_y           KB §2.11 [S1], §2.19 [S2]
//  10. Aligning moment M_z = -F_y * t         KB §2.8
//
//  WHAT THIS BLOCK DELIBERATELY DOES NOT COMPUTE
//  ---------------------------------------------
//   * Combined-slip M_z and M_x. KB §2.20.4: "No combined-slip M_z or M_x is
//     given." M_z is computed from the lateral force and a constant trail.
//   * The peak-and-decay of the pneumatic trail with slip angle (KB §2.8).
//     Gap G4, PARTIAL: [S3] adds the peak M_z values at three loads and the
//     fact that M_z peaks at about HALF the slip angle of the F_y peak, but no
//     t(alpha, F_z) function exists in any source.
//   * Conicity and ply steer (KB §2.10). [S3] §11.2 notes they are removed
//     from the data by the +/- slip-angle averaging the normalisation needs.
//   * Inflation pressure, temperature and wear (gaps G5, G7 - still OPEN in
//     all three sources).
//
//  All of the above are listed in README -> Open Questions.
//===========================================================================

#include "config/TireParameters.h"

namespace vsim {

/// One evaluation's worth of tire inputs. Mirrors the input list of the tire
/// block diagram in KB §2.2, minus the quantities no source quantifies
/// (inflation pressure, temperature).
struct TireInputs {
    /// Vertical load F_z [N], POSITIVE UP (KB trap T3). Clamped at >= 0 by
    /// the caller: a tire cannot pull the road up.
    double verticalLoad = 0.0;

    /// Reference load for the load-sensitivity relations [N].
    ///
    /// KB §2.4.3 phrases friction load sensitivity "near the tire's RATED
    /// load", and KB Eq. 6-6 quotes the cornering coefficient "at 100 % rated
    /// load". No source says what a given vehicle's tires are rated at, so the
    /// caller supplies the wheel's STATIC load as the reference. That is an
    /// assumption, recorded in README -> Open Questions.
    double ratedLoad = 0.0;

    /// Slip angle alpha [rad], Gillespie sign convention (KB trap T4):
    /// positive alpha produces positive (rightward) lateral force.
    double slipAngle = 0.0;

    /// Longitudinal slip ratio, [S3]'s SAE sign - DRIVING POSITIVE:
    ///     S = (Omega * R_e) / (V cos alpha) - 1          [Eq. (14.11)]
    ///
    /// KB §3.3 of `Tireand suspension.md` records the conversion between the
    /// definitions in circulation as `SR_[S3] = - Slip_[S1]`, so this is the
    /// negative of the Knowledge Base's braked-wheel form
    /// S = (V - R_e*omega)/V (KB Eq. 10-1 / Eq. 3-20). The magnitude is
    /// identical; [S1]'s form is what gets reported to the UI and telemetry.
    double slipRatio = 0.0;

    /// Inclination (camber) angle gamma [rad], signed so that POSITIVE means
    /// the top of the wheel leans toward the vehicle's RIGHT, which produces
    /// camber thrust toward the right - "a lateral force in the direction of
    /// the tilt" [S3] p.46.
    double camberAngle = 0.0;

    /// Rolling speed along the wheel plane, |V cos alpha| [m/s]. Sets the time
    /// constant of the relaxation-length lag (KB §2.19: tau = length / V).
    double rollingSpeed = 0.0;

    /// Unloaded radius R_u [m] (KB §2.15). Carried on the input rather than in
    /// TireParameters because the project already authors it as the wheel
    /// radius, and one source of truth beats two.
    double unloadedRadius = 0.0;

    /// Lateral force carried over from the previous step [N]. The relaxation
    /// length makes F_y a first-order lagged response, so it is a [STATE].
    double previousLateralForce = 0.0;

    /// Integration step [s].
    double dt = 0.0;
};

/// Everything the tire block produces, plus the intermediate quantities the
/// vehicle model and the UI need in order to explain the result.
struct TireForces {
    double longitudinal   = 0.0;  ///< F_x [N], tractive +, braking -
    double lateral        = 0.0;  ///< F_y [N], positive to the vehicle's right
    double aligningMoment = 0.0;  ///< M_z [N.m] = -F_y * t

    double frictionAvailable = 0.0;  ///< mu_y at this load [-]
    double frictionLimit     = 0.0;  ///< mu_y * F_z [N]
    double frictionUsage     = 0.0;  ///< R(k), the normalised resultant [-]

    double corneringStiffness = 0.0; ///< C_alpha at this load [N/rad]
    double slipStiffness      = 0.0; ///< k_x at this load [N per unit slip]
    double camberStiffness    = 0.0; ///< C_gamma at this load [N/rad]
    double pneumaticTrail     = 0.0; ///< t [m]

    /// Normalised resultant slip k = sqrt(S_bar^2 + beta_bar^2), Eq. (14.15).
    /// The single scalar the tyre actually responds to. Its peak sits at
    /// TireModel::normalisedPeakSlip(); beyond that the tyre is sliding.
    double normalisedSlip = 0.0;

    /// LOCAL slope dF_x/d(slip) at the current operating point
    /// [N per unit slip], floored at zero.
    ///
    /// Not physics the KB asks for - it is the Jacobian the caller needs to
    /// integrate the wheel's rotational equation of motion stably. It is
    /// produced here because only the tire block knows its own curve. Note
    /// that it goes to zero at the peak and would be NEGATIVE beyond it, which
    /// is KB §2.4.2's unstable branch; it is floored at zero so that the
    /// integrator damps but never anti-damps.
    double longitudinalSlipSlope = 0.0;
};

/// The individual tire.
///
/// Stateless: the one piece of tire state any source models (the
/// relaxation-length lag) is passed in and returned rather than stored, so a
/// single instance serves all four wheels and the model stays reproducible step
/// to step, which is what IVehicleDynamics promises its callers.
class TireModel {
public:
    explicit TireModel(const TireParameters& params);

    /// Evaluate the tire. This is the whole block.
    TireForces evaluate(const TireInputs& in) const;

    //-- the universal curve (KB §2.20.2) --------------------------------

    /// The normalised force produced by a normalised slip, Eqs. (14.5)-(14.7).
    /// Odd in `x`. This one function serves lateral slip, camber, longitudinal
    /// slip and the combined resultant - that is [S3]'s "one curve fits all".
    double universalCurve(double x) const;

    /// The normalised slip at which the universal curve peaks.
    ///
    /// Solved once in the constructor from B', C', D', E'. Everything the KB
    /// used to make this project author - the slip angle at peak lateral
    /// force, the longitudinal slip stiffness - is recovered from this number.
    double normalisedPeakSlip() const { return m_peakNormalisedSlip; }

    //-- geometry (KB §2.15) ---------------------------------------------

    /// Vertical deflection under load [m], from the tire vertical rate.
    double deflection(double verticalLoad) const;

    /// Loaded radius R_l [m] - wheel centre to the centre of contact.
    double loadedRadius(double verticalLoad, double unloadedRadius) const;

    /// Effective rolling radius R_e [m] (KB Eq. 5.11 [S2]).
    ///
    /// The radius that makes V = R_e * omega true for a free-rolling tire, and
    /// therefore the ONLY radius that gives zero slip ratio when the tire is
    /// not slipping. KB §2.15 is emphatic that using R_u or R_l here produces
    /// a spurious force on a freely rolling tire.
    ///
    /// [S3] Eq. T6 gives `R_e = R_0/(1 + SR_0)` but no R_e(load) relation, so
    /// [S2] remains the source for this (KB gap G3, CLOSED by [S2]).
    double effectiveRollingRadius(double verticalLoad, double unloadedRadius) const;

    //-- load-dependent properties ---------------------------------------

    /// Lateral friction coefficient available at this load (KB §2.4.3).
    double frictionAt(double verticalLoad, double ratedLoad) const;

    /// Cornering stiffness C_alpha of this tire at this load [N/rad]
    /// (KB Eq. 6-6 with the Eq. 6-35/6-36 load polynomial).
    double corneringStiffnessAt(double verticalLoad, double ratedLoad) const;

    /// Camber stiffness C_gamma at this load [N/rad] (KB §12.2 [S3]).
    double camberStiffnessAt(double verticalLoad, double ratedLoad) const;

    /// Longitudinal slip stiffness k_x at this load [N per unit slip ratio].
    ///
    /// DERIVED, not authored. [S3] names k_x as a required parameter of the
    /// model (Eq. 14.10) and gives no value for any tyre. But it also states
    /// two things that between them determine it:
    ///
    ///   1. the universal curve fits F_bar_x against S_bar with the SAME
    ///      B', C', D', E' as the lateral curve  [S3] p.479;
    ///   2. the slip ratio at peak longitudinal force is 0.10-0.15
    ///      (KB §12.2, [S3] pp.37-38; [S1] gives 15-20 % on dry).
    ///
    /// The peak of the universal curve is at a known normalised slip
    /// S_bar_peak, and S_bar = k_x*S/(mu_x*F_z), so
    ///
    ///     k_x = S_bar_peak * mu_x * F_z / S_peak
    ///
    /// This is a derivation from two Knowledge Base statements, not a value
    /// from outside it. It is flagged in README -> Open Questions all the same,
    /// because the KB does not perform the derivation itself.
    double slipStiffnessAt(double verticalLoad, double ratedLoad) const;

    /// Slip angle at which lateral force peaks [rad], at this load.
    ///
    /// Also DERIVED rather than authored, by the same argument applied to
    /// Eq. (14.4):  a_bar = C_alpha*tan(alpha)/(mu_y*F_z), so
    ///
    ///     alpha_peak = arctan( a_bar_peak * mu_y * F_z / C_alpha )
    ///
    /// Reported so it can be checked against what the sources say it should
    /// be: [S1]'s charts show saturation "at roughly ten to fifteen degrees"
    /// for a road tyre (KB §2.6.2), and [S3] gives 3-7 deg for race tyres,
    /// which have three to five times the cornering stiffness (KB §12.2).
    double peakSlipAngle(double verticalLoad, double ratedLoad) const;

    //-- accessors --------------------------------------------------------

    const TireParameters& parameters() const { return m_p; }

    /// Slip ratio at which longitudinal force peaks [-].
    /// Past this the mu-slip curve has NEGATIVE slope, which KB §2.4.2
    /// identifies as the physical origin of wheel-lockup instability.
    double peakSlipRatio() const { return m_p.peakSlipRatio; }

private:
    TireParameters m_p;
    double m_peakNormalisedSlip = 1.0;
};

} // namespace vsim
