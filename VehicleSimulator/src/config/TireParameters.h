#pragma once
//===========================================================================
//  TireParameters.h  --  static description of ONE tire.
//
//  The tyre model is the NONDIMENSIONAL model of Knowledge Base §2.20
//  (`Tireand suspension.md` §11), from [S3] Milliken & Milliken, RCVD Ch.14.
//  These are exactly the parameters that model requires:
//
//      mu_y, mu_x   lateral and longitudinal friction coefficients
//      C_alpha      cornering stiffness
//      k_x          longitudinal slip stiffness
//      C_gamma      camber stiffness
//      t            pneumatic trail
//      B', C', D', E'   the universal curve fit
//
//  Every field is traced to the Knowledge Base. Where the KB gives a value or
//  a range, the default is taken from it and the citation is on the field.
//  Where the KB names a parameter but does NOT quantify it, that is said in
//  the comment and the item appears in README -> Open Questions.
//
//  WHAT THE THIRD SOURCE CHANGED HERE
//  ----------------------------------
//  Two of the three "not a Knowledge Base value" parameters this file used to
//  carry are gone, because [S3] supplies what was missing:
//
//    * the slip angle at peak lateral force is no longer authored. The
//      universal curve fixes the peak at a known NORMALISED slip, so the peak
//      slip ANGLE follows from the cornering stiffness and the friction
//      coefficient. See TireModel::peakSlipAngle().
//    * the longitudinal slip stiffness k_x is no longer authored. It is
//      derived from the KB's quoted slip ratio at peak F_x and the same
//      normalised peak. See TireModel::slipStiffnessAt().
//
//  UNITS. The Knowledge Base is in US customary units and warns (§0.3, §13.3
//  trap T12) that embedded constants are unit-specific. No KB equation with an
//  embedded constant is used here: Eqs. (14.1)-(14.21) are dimensionless or
//  dimensionally homogeneous, so they carry over to SI unchanged. The KB's
//  *values* are converted once, here, with the conversion shown.
//===========================================================================

#include "core/Units.h"

namespace vsim {

struct TireParameters {
    //-----------------------------------------------------------------------
    //  FRICTION
    //-----------------------------------------------------------------------

    /// Lateral friction coefficient mu_y [-] at the tire's rated load.
    ///
    /// KB §2.4.2 / §12.2:  dry 0.70-0.90, wet 0.25-0.50, ice 0.10-0.15.
    /// KB §9.4.13 [S2]: max lateral acceleration in g equals mu, "typically
    /// mu ~ 0.9". [S3] adds measured race-tyre values - Eagle GT-S 1.10 / 1.06
    /// / 0.96 at 900 / 1350 / 1800 lb, and a Grand Prix tyre "up to 1.8 at
    /// light load" (KB §12.2). The default is the road-car value.
    ///
    /// In the nondimensional model this is the peak of the lateral curve:
    /// D' = 1.00 exactly, so peak F_y = mu_y * F_z.
    double lateralFriction = 0.90;

    /// Longitudinal friction coefficient, as a ratio mu_x / mu_y [-].
    ///
    /// [S3] Eq. (14.9) names mu_x and states (KB §2.20.2) that "friction
    /// coefficients may differ in the lateral and longitudinal directions",
    /// but gives NO VALUE for any tyre. The default of 1.0 is not a guess: it
    /// is [S1] §2.9's explicitly named "pure-circle case", where the friction
    /// boundary is a circle of radius mu*F_z rather than an ellipse.
    /// Any other value makes it an ellipse. See README -> Open Questions.
    double longitudinalFrictionRatio = 1.0;

    /// Load sensitivity of the friction coefficient [-] per unit relative
    /// load change.
    ///
    /// KB §2.4.3 [S1]: "mu decreases by approximately 0.01 for each 10 %
    /// increase in load near the tire's rated load."  0.01 per 0.10 = 0.10.
    ///
    /// [S3] §6 corroborates the effect and names it "tire load sensitivity",
    /// stating it "has a major effect on the balance (plow/spin) of a race car
    /// operating near the limit" - but supplies no algebraic law, only plotted
    /// curves. The [S1] rule is therefore the one implemented; [S3]'s Eagle
    /// GT-S numbers (1.10 -> 0.96 as load doubles from 900 to 1800 lb) are a
    /// useful independent check and are close to it.
    double frictionLoadSensitivity = 0.10;

    //-----------------------------------------------------------------------
    //  STIFFNESSES
    //-----------------------------------------------------------------------

    /// Cornering coefficient CC_alpha = C_alpha / F_z, in 1/RADIAN.
    ///
    /// KB Eq. 6-6 and §2.6.3 give, in 1/deg:
    ///     0.20  general, at 100 % rated load
    ///     0.12  bias-ply
    ///     0.15  radial
    ///     0.25 - 0.30+  low-aspect-ratio performance radial
    /// [S3] Table 2.2 adds measured race-tyre stiffnesses (KB §12.2).
    /// Default = 0.15/deg (radial), converted: 0.15 * 180/pi = 8.594 /rad.
    ///
    /// This is `C` in Eq. (14.4). Note the consequence, which is now a
    /// DERIVED result rather than an authored one: with mu_y = 0.90 this
    /// places the peak of the lateral force curve at about 15.7 deg of slip -
    /// see TireModel::peakSlipAngle().
    double corneringCoefficient = 0.15 * 180.0 / kPi;

    /// Camber stiffness as a fraction of cornering stiffness, C_gamma/C_alpha.
    ///
    /// KB §12.2 [S3]: radial approximately 1/10 (10 %), bias-ply 1/5 to 1/6
    /// (17-20 %). [S3] p.727 states the same as a force equivalence: "for many
    /// radial tires, 1.0 deg of camber produces about the same lateral force
    /// as 0.1 deg of steer (10:1)."
    ///
    /// [S1] gave only a single undifferentiated band of "10-20 % of C_alpha"
    /// (KB §2.7); [S3] separates it by construction. Default = radial.
    /// This is `G` in Eq. (14.8).
    double camberStiffnessRatio = 0.10;

    /// Quadratic coefficient `b` of the cornering-stiffness/load polynomial
    /// (KB Eq. 6-35/6-36), in 1/(N * rad).
    ///
    ///     C_alpha(per tire) = a * F_z - b * F_z^2
    ///
    /// NOT QUANTIFIED BY ANY SOURCE. KB §2.14 lists `a`, `b` as "vehicle/tire
    /// specific"; [S3] supplies measured stiffnesses at single loads but no
    /// polynomial. The default is 0, which collapses Eq. 6-36 exactly onto the
    /// KB's own simpler statement C_alpha = CC_alpha * F_z (Eq. 6-6).
    ///
    /// Note that with the nondimensional model, load sensitivity no longer
    /// depends on this term alone: mu_y already falls with load (above), and
    /// C_alpha enters the normalisation, so weight transfer changes tyre
    /// behaviour whether or not `b` is authored.
    /// See README -> Open Questions.
    double loadSensitivityB = 0.0;

    /// Slip ratio at which longitudinal force peaks [-].
    ///
    /// KB §12.2: [S3] gives 0.10-0.15 in the text and about 0.18 read from
    /// Figs. 2.16/2.17; [S1] gives 15-20 % on dry (Ch.10), 10-20 % (Ch.3), and
    /// "a few percent" on ice - which is why lockup on ice is nearly
    /// immediate. Default 0.15, inside every one of those ranges.
    ///
    /// This is NOT a shape parameter. The shape comes from the universal
    /// curve. It is the quantity from which the longitudinal slip stiffness
    /// k_x is derived - see TireModel::slipStiffnessAt().
    double peakSlipRatio = 0.15;

    //-----------------------------------------------------------------------
    //  THE UNIVERSAL CURVE  -  KB §2.20.2, [S3] Eqs. (14.5)-(14.7)
    //
    //      F_bar = D' sin(theta),  theta = C' arctan(B' phi),
    //      phi   = (1-E') a_bar + (E'/B') arctan(B' a_bar)
    //
    //  [S3] p.479: "once the data are made dimensionless, 'one curve fits
    //  all.'"  The SAME four numbers fit normalised lateral force against
    //  normalised slip angle, normalised camber thrust, normalised
    //  longitudinal force against normalised slip ratio, AND the normalised
    //  resultant force against the normalised resultant slip. That is the
    //  whole reason this model can do combined slip at all.
    //
    //  These are the source's own fitted values for the P195/70R-14 tyre,
    //  fitted to data at five loads simultaneously [S3] p.476.
    //-----------------------------------------------------------------------

    double curveB = 0.714;   ///< B'
    double curveC = 1.40;    ///< C'
    double curveD = 1.00;    ///< D'  (peak of the normalised curve = 1)
    double curveE = -0.20;   ///< E'

    //-----------------------------------------------------------------------
    //  ALIGNING MOMENT
    //-----------------------------------------------------------------------

    /// Aligning moment coefficient: Mz per unit load per unit slip angle,
    /// in m/rad (dimensions of a length, per radian).
    ///
    /// KB §2.8 / §12.2, in ft-lb/lb/deg: bias-ply 0.033, radial 0.043.
    /// Default = radial: 0.043 * 0.3048 m/ft * 180/pi deg/rad = 0.7510.
    ///
    /// The pneumatic trail - `T_z` in [S3] Eq. (14.2), "the ratio of
    /// self-aligning torque to lateral force for small slip angles" - is
    /// derived from this and the cornering stiffness rather than authored:
    ///     t = coefficient * F_z / C_alpha                    (KB §2.8)
    double aligningMomentCoefficient = 0.043 * 0.3048 * 180.0 / kPi;

    //-----------------------------------------------------------------------
    //  STRUCTURE
    //-----------------------------------------------------------------------

    /// Tire vertical rate K_t [N/m].
    ///
    /// KB §12.2 gives two worked-example values: 1000 lb/in [S1] and 2000
    /// lb/in [S3] (the latter used in all three of [S3]'s examples). Neither
    /// is a measured range. Default = 1000 lb/in converted:
    /// 1000 * 4.4482216 N/lb / 0.0254 m/in = 175 127 N/m.
    ///
    /// Used for the vertical deflection that sets the loaded and effective
    /// rolling radii (KB Eq. 5.11), and as the tyre spring in series with the
    /// suspension in the ride rate (KB §7.2). It is NOT a suspension rate -
    /// KB trap T7 warns that spring rate, suspension rate and ride rate are
    /// three different things, and this is none of them.
    double verticalRate = 175127.0;

    /// Relaxation length, expressed in tire revolutions [-].
    ///
    /// KB §2.11 [S1]: the rolling distance needed to develop ~63 % of the
    /// steady-state lateral force is "approximately one-half to one tire
    /// revolution". Default 0.75 - the midpoint of the KB's stated range.
    ///
    /// KB §2.19 [S2] adds that this is a first-order lag whose TIME constant
    /// is tau = (relaxation length)/V, so it matters at low speed and vanishes
    /// at high speed.
    ///
    /// [S3] has no tyre transient content at all - KB §12.2 records relaxation
    /// length as "absent from [S3]" - so this remains [S1]/[S2] material.
    double relaxationLengthRevolutions = 0.75;

    //-----------------------------------------------------------------------
    //  ALIGNMENT
    //-----------------------------------------------------------------------

    /// Static camber [rad], signed so that NEGATIVE means the top of the wheel
    /// leans inboard - the usual convention and the usual sign.
    ///
    /// [S3] pp.49-50 gives the camber that maximises peak lateral force as
    /// about -5 deg for modern wide radials, worth about +6 % of peak F_y at
    /// -6 deg and 1200 lb. Those are RACE figures for a cambered test tyre,
    /// not a recommended road-car alignment, so the default here is zero
    /// rather than a number lifted out of that context.
    double staticCamber = 0.0;

    void validate();
};

} // namespace vsim
