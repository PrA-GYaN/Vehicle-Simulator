#include "physics/SuspensionModel.h"

#include <cmath>

#include "physics/TireModel.h"

namespace vsim {

SuspensionState computeSuspension(const VehicleParameters& p) {
    SuspensionState s;

    const SuspensionParameters& sp = p.suspension;

    const double W  = p.mass * kGravity;                 // total weight [N]
    const double L  = p.wheelbase;
    const double h  = p.cgHeight;
    const double tF = p.frontTrack;
    const double tR = p.rearTrack;

    const double WF = W * p.frontWeightDistribution;     // KB Eq. 1-6
    const double WR = W - WF;                            // KB Eq. 1-7

    //-----------------------------------------------------------------------
    //  1. SPRUNG / UNSPRUNG SPLIT  -  KB §1.6.
    //
    //  [S1] p.164: unsprung weight per axle is approximately 10 % of GAWR for
    //  a non-driven axle and 15 % for a driven one. The axle load stands in for
    //  the Gross Axle Weight Rating - the KB gives no other way to get it.
    //
    //  [S3] needs this split for all three of its load transfer terms, and
    //  gives no unsprung fraction of its own, so this stays [S1] material.
    //-----------------------------------------------------------------------
    const bool frontDriven = (p.driveLayout == DriveLayout::FrontWheelDrive);

    const double fracF = frontDriven ? sp.unsprungFractionDriven
                                     : sp.unsprungFractionNonDriven;
    const double fracR = frontDriven ? sp.unsprungFractionNonDriven
                                     : sp.unsprungFractionDriven;

    const double WuF = WF * fracF;
    const double WuR = WR * fracR;
    const double Ws  = W - WuF - WuR;

    if (!(Ws > 1e-6)) return s;                          // degenerate config

    const double WsF = WF - WuF;
    const double WsR = WR - WuR;
    const double aS  = WsF / Ws;                         // sprung weight dist.

    s.sprungWeight              = Ws;
    s.unsprungWeightFront       = WuF;
    s.unsprungWeightRear        = WuR;
    s.sprungWeightFractionFront = aS;

    //-----------------------------------------------------------------------
    //  2. SPRUNG CG HEIGHT AND THE ROLL MOMENT ARM  -  [S3] p.603, KB §3.5.1.
    //
    //      h_s  = ( W h - W_uF RL_F - W_uR RL_R ) / W_s
    //      h_2  = h_s - [ z_F + ( z_R - z_F )( 1 - a_S ) ]
    //
    //  The second line is the perpendicular distance from the sprung CG down to
    //  the neutral roll axis, interpolated between the two roll centre heights
    //  at the sprung CG's own longitudinal position.
    //
    //  [S3] uses the tyre's STATIC LOADED RADIUS as the unsprung CG height in
    //  its worked example. KB §3.5.1 flags that as "RCVD's own choice in this
    //  example ... an assumption of the worked example, not a general
    //  definition" - and it is used here for want of anything better, and
    //  recorded in README -> Open Questions.
    //-----------------------------------------------------------------------
    const TireModel tire(p.tire);
    const double RL = tire.loadedRadius(0.25 * W, p.wheelRadius);   // per wheel

    const double hs = (W * h - WuF * RL - WuR * RL) / Ws;

    const double zF = sp.rollCentreHeightFront;
    const double zR = sp.rollCentreHeightRear;
    const double h2 = hs - (zF + (zR - zF) * (1.0 - aS));

    s.sprungCgHeight = hs;
    s.rollMomentArm  = h2;

    //-----------------------------------------------------------------------
    //  3. RIDE RATE FROM RIDE FREQUENCY  -  [S3] pp.601-602.
    //
    //      K_R = 4 pi^2 omega^2 ( W_S/2 ) / g          [lb/in, with g in in/s^2]
    //
    //  In SI the same relation with the sprung CORNER MASS is simply
    //
    //      K_R = ( 2 pi f )^2 * m_corner               [N/m]
    //
    //  which is just k = omega^2 m - the source's constants are the in/ft and
    //  in/s^2 conversions, and KB trap T12 warns that converting an equation
    //  with an embedded constant means re-deriving it, which is what this is.
    //
    //  KB §12.3 records that [S3] gives TWO forms of this relation - one using
    //  the total corner weight, one using the sprung corner weight - and does
    //  not comment on the difference. The sprung form is used here because it
    //  is the one in [S3]'s more complete second example, and because the ride
    //  frequency of interest is the body's.
    //-----------------------------------------------------------------------
    const double wF = kTwoPi * sp.rideFrequencyFront;
    const double wR = kTwoPi * sp.rideFrequencyRear;

    const double KRF = wF * wF * (WsF / kGravity) * 0.5;   // [N/m] per corner
    const double KRR = wR * wR * (WsR / kGravity) * 0.5;

    s.rideRateFront = KRF;
    s.rideRateRear  = KRR;

    //-----------------------------------------------------------------------
    //  4. ROLL RATE FROM RIDE RATE  -  [S3] p.589.
    //
    //      K_phi = 12 * K_R * t^2 / 2          [lb-ft/rad, K_R in lb/in, t in ft]
    //
    //  The 12 converts lb/in to lb/ft; in SI, K_phi = K_R t^2 / 2 [N.m/rad].
    //
    //  KB §3.5.1 notes that [S1]'s equivalent (Eq. 6-26) is K_phi = 0.5 K_s s^2
    //  with s the SPRING spacing, whereas [S3] uses the ride rate and the
    //  TRACK, because the installation ratio is already folded into the ride
    //  rate. The two are the same relation applied at different points in the
    //  chain and "must not be interchanged" - this file uses [S3]'s throughout.
    //
    //  The anti-roll bar adds directly to the axle roll rate. That gap - G14,
    //  "anti-roll bar rate as a separate contributor" - was OPEN until [S3].
    //-----------------------------------------------------------------------
    const double KF = KRF * tF * tF * 0.5 + std::max(0.0, sp.antiRollRateFront);
    const double KR = KRR * tR * tR * 0.5 + std::max(0.0, sp.antiRollRateRear);

    s.rollRateFront = KF;
    s.rollRateRear  = KR;

    const double Ktot = KF + KR;
    s.rollRateDistributionFront = (Ktot > 1e-9) ? KF / Ktot : 0.5;

    //-----------------------------------------------------------------------
    //  5. ROLL GRADIENT  -  [S3] p.682, and [S1] Eq. 6-32 for the same thing.
    //
    //      phi / A_Y = - W_s h_2 / ( K_F + K_R - W_s h_2 )        [rad per g]
    //
    //  The `- W_s h_2` in the DENOMINATOR is the destabilising gravity moment:
    //  once the body has rolled, the sprung weight acting at the displaced CG
    //  adds further roll moment. KB §9.4.8 spells out the consequences - roll
    //  stiffness must EXCEED W_s h_2 for the roll angle to be finite at all,
    //  and as it approaches it the gradient goes to infinity. "This is a
    //  genuine physical instability, not a modelling artefact, and it is why
    //  very soft-sprung, high-CG vehicles roll dramatically."
    //
    //  Both sources carry this term and agree on its structure ([S1] writes
    //  W h_1, [S3] writes W_s h_2). [S3]'s simpler Chapter 16 form omits it;
    //  the Chapter 18 form used here retains it.
    //-----------------------------------------------------------------------
    const double gravityMoment = Ws * h2;               // [N.m/rad]
    double denom = Ktot - gravityMoment;

    if (!(denom > 0.05 * Ktot)) {
        // Statically unstable in roll, or close enough that the gradient would
        // be meaningless. Clamped so the simulation stays finite; the flag is
        // surfaced in the UI rather than being hidden.
        s.rollUnstable = true;
        denom = 0.05 * Ktot;
        if (!(denom > 1e-9)) return s;
    }

    s.rollGradient = -gravityMoment / denom;            // [rad per g], negative

    //-----------------------------------------------------------------------
    //  6. LATERAL LOAD TRANSFER, IN THREE TERMS  -  [S3] p.682, KB §3.5.1.
    //
    //      dW_F     W_s ⎡     h_2 K_F'          (l - a_s)      ⎤   W_uF
    //      ──── =  ──── ⎢ ───────────────── + ─────────── z_RF ⎥ + ──── z_WF
    //       A_Y     t_F ⎣ K_F + K_R - W_s h_2      l           ⎦    t_F
    //
    //      K_F' = K_F - (l - a_s) W_s h_2 / l
    //
    //  and correspondingly at the rear with a_s/l in place of (l - a_s)/l.
    //
    //  Note the symbol trap: [S3] writes `a_s` for a DISTANCE (front axle to
    //  sprung CG) in these equations and for a FRACTION (sprung weight
    //  distribution) in the h_2 equation of step 2. By the lever-arm rule of
    //  KB §1.8, (l - a_s)/l IS the front sprung weight fraction, so the two
    //  meanings coincide numerically here and `aS` serves for both.
    //
    //  "The units of these expressions are pounds of load change per 'g' of
    //  lateral acceleration - increased load on the outside or decreased load
    //  on the inside."
    //-----------------------------------------------------------------------
    const double KFp = KF - aS * gravityMoment;
    const double KRp = KR - (1.0 - aS) * gravityMoment;

    const double zWF = RL;     // unsprung CG height, see step 2
    const double zWR = RL;

    s.elasticFrontPerG   = (Ws / tF) * (h2 * KFp / denom);
    s.geometricFrontPerG = (Ws / tF) * (aS * zF);
    s.unsprungFrontPerG  = (WuF / tF) * zWF;

    s.elasticRearPerG    = (Ws / tR) * (h2 * KRp / denom);
    s.geometricRearPerG  = (Ws / tR) * ((1.0 - aS) * zR);
    s.unsprungRearPerG   = (WuR / tR) * zWR;

    s.loadTransferFrontPerG =
        s.elasticFrontPerG + s.geometricFrontPerG + s.unsprungFrontPerG;
    s.loadTransferRearPerG =
        s.elasticRearPerG + s.geometricRearPerG + s.unsprungRearPerG;

    const double total = s.loadTransferFrontPerG + s.loadTransferRearPerG;
    s.loadTransferDistributionFront = (std::abs(total) > 1e-9)
                                          ? s.loadTransferFrontPerG / total
                                          : 0.5;

    //  A note on what NOT to expect here. The two distributions computed above
    //  - roll rate and load transfer - are different numbers, and KB §3.5.2
    //  says that confusing them is a named and common modelling error worth
    //  ten percentage points in [S3]'s own example. Both are reported.

    return s;
}

double rollAngle(const SuspensionState& s, double lateralAcceleration) {
    //  phi = (roll gradient) * A_Y, with A_Y in g.
    //
    //  Quasi-static, because that is all any source supports: KB §3.5.4 records
    //  [S3]'s own statement that its whole wheel-load treatment is steady state
    //  and that "transient behavior of the vehicle and sprung and unsprung mass
    //  dynamics (ride/roadholding) are ignored". There is no roll equation of
    //  motion in any of the three sources - gap G9.
    return s.rollGradient * (lateralAcceleration / kGravity);
}

} // namespace vsim
