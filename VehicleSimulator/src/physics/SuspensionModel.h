#pragma once
//===========================================================================
//  SuspensionModel.h  --  roll rate, roll gradient and lateral load transfer.
//
//  WHAT THIS IS
//  ------------
//  The chain of Knowledge Base §3.5.1 and `Tireand suspension.md` §20-§21,
//  from [S3] Milliken & Milliken, RCVD Chapters 16 and 18:
//
//      ride frequency  ->  ride rate  ->  roll rate  ->  roll gradient
//                                                     ->  load transfer
//
//  Before the third source there was no roll model here at all, because the
//  parameters [S1] Eq. 6-31 needs - the sprung CG height above the roll axis
//  and the two axle roll stiffnesses - had no typical value in any source. The
//  front/rear split of lateral load transfer had to be an authored fraction.
//  It no longer does.
//
//  THREE MECHANISMS, NOT TWO  -  KB §3.5.1
//  ---------------------------------------
//  [S1] Eq. 6-27 splits lateral load transfer into an ELASTIC term (through the
//  springs, requiring the body to actually roll) and a GEOMETRIC term (through
//  the links at the roll centre, instantaneous). [S3] adds a third:
//
//      elastic    (W_s/t) h_2 K'/(K_F + K_R - W_s h_2)
//      geometric  (W_s/t) (sprung weight fraction) z_R
//      unsprung   (W_u/t) z_W
//
//  and makes the sprung/total mass distinction explicit throughout, which [S1]
//  does not.
//
//  THE WARNING THIS MODEL EXISTS TO RESPECT  -  KB §3.5.2
//  -----------------------------------------------------
//  [S3] p.679: "Total lateral load transfer distribution (TLLTD) is sometimes
//  erroneously called 'roll couple distribution.' The problem is that roll
//  couple distribution is only part of the mechanism that controls the
//  distribution of loads in a steady turn."  In [S3]'s own worked example the
//  two differ by TEN PERCENTAGE POINTS - roll rate distribution 65/35 against
//  load transfer distribution 55/45.
//
//  A simulator that sets the front/rear split from the roll-stiffness ratio
//  alone is wrong by exactly that much. This project used to do precisely that.
//  Both numbers are now computed and both are reported.
//
//  WHAT IS STILL NOT MODELLED
//  --------------------------
//   * Roll DYNAMICS. Roll angle here is quasi-static: phi = (roll gradient) x
//     lateral acceleration, evaluated every step. [S3] Ch.18 is explicitly
//     steady state - "transient behavior of the vehicle and sprung and unsprung
//     mass dynamics are ignored" - and KB gaps G8/G9 (transient yaw/sideslip
//     and yaw-roll coupled equations) remain OPEN in all three sources.
//   * The damper's contribution to transient wheel load, which [S3] §25 says
//     can be as large as the anti-roll bar's.
//   * Roll centre migration with travel and roll (acknowledged by [S3], not
//     modelled by it).
//   * Pitch. There is no pitch equivalent of this chain in any source; the
//     longitudinal load transfer is the rigid-body result of KB Eq. 1-8/1-9.
//
//  All of these are listed in README -> Open Questions.
//===========================================================================

#include "config/VehicleParameters.h"

namespace vsim {

/// Everything the roll chain produces. All SI, all derived - nothing here is
/// authored, and nothing is stored between steps.
struct SuspensionState {
    //-- mass split (KB §1.6) --------------------------------------------
    double sprungWeight        = 0.0;  ///< W_s [N]
    double unsprungWeightFront = 0.0;  ///< W_uF [N]
    double unsprungWeightRear  = 0.0;  ///< W_uR [N]
    double sprungWeightFractionFront = 0.5;  ///< a_S, sprung weight distribution

    //-- geometry (KB §3.5.1, [S3] p.603) ---------------------------------
    double sprungCgHeight = 0.0;   ///< h_s, sprung CG above ground [m]
    double rollMomentArm  = 0.0;   ///< h_2, sprung CG to the neutral roll axis [m]

    //-- rates ([S3] pp.588-589) ------------------------------------------
    double rideRateFront = 0.0;    ///< K_RF [N/m]
    double rideRateRear  = 0.0;    ///< K_RR [N/m]
    double rollRateFront = 0.0;    ///< K_F  [N.m/rad], including any bar
    double rollRateRear  = 0.0;    ///< K_R  [N.m/rad], including any bar

    /// Front share of total roll rate, K_F/(K_F+K_R) [-]. NOT the same as the
    /// load transfer distribution below - see the header.
    double rollRateDistributionFront = 0.5;

    //-- response ([S3] p.682) --------------------------------------------

    /// Roll gradient phi/A_Y [rad per g]. NEGATIVE: [S3] p.681 - "if A_Y is
    /// positive (a RH turn) ... the car rolls to the OUTSIDE of the turn."
    double rollGradient = 0.0;

    /// True when the suspension is statically unstable in roll, i.e. total roll
    /// rate does not exceed the gravity term W_s*h_2. KB §9.4.8 calls this "a
    /// genuine physical instability, not a modelling artefact". The gradient is
    /// clamped when it happens so the simulation stays finite.
    bool rollUnstable = false;

    //-- load transfer ([S3] p.682, three terms) ---------------------------
    double loadTransferFrontPerG = 0.0;  ///< dW_F/A_Y [N per g]
    double loadTransferRearPerG  = 0.0;  ///< dW_R/A_Y [N per g]

    /// Front total lateral load transfer distribution, TLLTD [-].
    double loadTransferDistributionFront = 0.5;

    //-- breakdown, for the UI --------------------------------------------
    double elasticFrontPerG   = 0.0;  ///< [N per g]
    double geometricFrontPerG = 0.0;
    double unsprungFrontPerG  = 0.0;
    double elasticRearPerG    = 0.0;
    double geometricRearPerG  = 0.0;
    double unsprungRearPerG   = 0.0;
};

/// Evaluate the whole chain from the vehicle's authored parameters.
///
/// Pure and cheap - about thirty floating-point operations - so it is called
/// every physics step rather than cached, which keeps the dynamics model
/// reproducible when parameters are edited live in the UI.
SuspensionState computeSuspension(const VehicleParameters& params);

/// Body roll angle [rad] at a given lateral acceleration.
///
///     phi = (roll gradient) * (a_y / g)
///
/// Quasi-static: KB §3.5.4 records that [S3]'s whole wheel-load treatment is
/// steady state. Positive phi means the body leans to the vehicle's RIGHT.
double rollAngle(const SuspensionState& s, double lateralAcceleration);

} // namespace vsim
