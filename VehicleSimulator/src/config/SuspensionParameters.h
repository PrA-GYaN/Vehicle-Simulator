#pragma once
//===========================================================================
//  SuspensionParameters.h  --  what the roll and load-transfer model needs.
//
//  This file exists because of what the third source added. Before it, the
//  Knowledge Base named the roll model's parameters - the sprung CG height
//  above the roll axis, the front and rear roll stiffnesses - and gave a
//  typical value for none of them, so there was no roll model at all and the
//  front/rear split of lateral load transfer had to be authored as a bare
//  fraction.
//
//  [S3] Milliken & Milliken supplies the missing chain, end to end:
//
//      ride frequency  ->  ride rate        [S3] p.588/601, KB §12.3
//      ride rate       ->  roll rate        [S3] p.589
//      roll rate       ->  roll gradient    [S3] p.586, p.682
//      roll gradient   ->  lateral load transfer, in three terms
//                                           [S3] p.682, KB §3.5.1
//
//  and gives typical values by vehicle class for the two ends of it - ride
//  frequency (KB §12.3) and roll gradient (KB §12.3, [S3] Table 16.1) - so the
//  chain can be both driven and checked against the sources.
//
//  UNITS are SI throughout. [S3]'s equations carry factors of 12 (lb/in to
//  lb/ft), 57.3 (rad to deg) and 1375 (both at once); those are unit
//  conversions, not physics, and they vanish in SI. KB §13.3 trap T12 warns
//  that converting an equation with an embedded constant means re-deriving the
//  constant, which is what has been done here - see SuspensionModel.cpp.
//===========================================================================

#include "core/Units.h"

namespace vsim {

struct SuspensionParameters {
    //-----------------------------------------------------------------------
    //  RIDE FREQUENCIES  -  the top of the chain
    //-----------------------------------------------------------------------

    /// Undamped ride natural frequency at the front / rear axle [Hz].
    ///
    /// KB §12.3 [S3] p.582, converted from cycles per minute:
    ///     passenger cars           30-50 cpm   = 0.50-0.83 Hz
    ///     sports cars              70-90 cpm   = 1.17-1.50 Hz
    ///     Indy, no ground effects  95-120 cpm  = 1.58-2.00 Hz
    ///
    /// KB §12.3 [S1] gives a different bound for the same quantity: the Olley
    /// criterion puts the body natural frequency at 1.3 Hz or below. The two
    /// bands overlap only at their edges, and this is a genuine divergence
    /// between the sources rather than something to average - [S3]'s numbers
    /// are for the soft American sedans of its examples, [S1]'s is a design
    /// criterion. The shipped configs sit inside [S1]'s bound and above [S3]'s
    /// passenger band, and the resulting roll gradient is reported so it can be
    /// checked against [S3]'s own table. See README -> Open Questions.
    ///
    /// [S3] p.589: "on most rear-drive race cars the front natural frequency is
    /// higher than the rear, based on front roll stiffness requirements."
    double rideFrequencyFront = 1.25;
    double rideFrequencyRear  = 1.20;

    //-----------------------------------------------------------------------
    //  ROLL CENTRES
    //-----------------------------------------------------------------------

    /// Roll centre height at the front / rear axle [m], above ground.
    ///
    /// KB §12.3 / `Tireand suspension.md` §27.2: [S3] gives only WORKED-EXAMPLE
    /// values - 0.25 and 0.625 ft, and 3.1 and 6.8 in - never a typical range,
    /// and states that roll centre migration with travel and roll is
    /// "acknowledged but not modelled". The defaults below are [S3]'s
    /// second-example values converted (3.1 in = 0.0787 m, 6.8 in = 0.173 m).
    ///
    /// This is the GEOMETRIC path of lateral load transfer - KB §3.5's
    /// "instantaneous, no roll motion needed" term. Raising a roll centre moves
    /// load transfer at that axle from the elastic (lagged) path to the
    /// geometric (instant) one. See README -> Open Questions.
    double rollCentreHeightFront = 0.0787;
    double rollCentreHeightRear  = 0.1730;

    //-----------------------------------------------------------------------
    //  ANTI-ROLL BARS
    //-----------------------------------------------------------------------

    /// Additional roll rate contributed by the anti-roll bar at each axle
    /// [N.m/rad of body roll].
    ///
    /// KB gap G14 - "anti-roll bar rate as a separate contributor" - was OPEN
    /// until [S3]. `Tireand suspension.md` §19 now supplies the required
    /// additional roll rate, the bar installation ratio and the bar torsional
    /// rate, with worked numbers.
    ///
    /// Zero means no bar, which is honest rather than typical: neither source
    /// gives a typical bar rate for a road car. The shipped configs author a
    /// value chosen so that the resulting ROLL GRADIENT lands on [S3]
    /// Table 16.1's figure for that class of vehicle - both ends of that
    /// calculation are Knowledge Base numbers, and the relation between them is
    /// [S3]'s. See README -> Open Questions.
    double antiRollRateFront = 0.0;
    double antiRollRateRear  = 0.0;

    //-----------------------------------------------------------------------
    //  UNSPRUNG MASS
    //-----------------------------------------------------------------------

    /// Unsprung weight at each axle, as a fraction of that axle's load.
    ///
    /// KB §1.6 [S1] p.164: approximately 10 % of GAWR for a NON-DRIVEN axle and
    /// 15 % for a DRIVEN one. The model picks between them from the configured
    /// drive layout, so these two fields are the [S1] values themselves rather
    /// than per-axle authored numbers.
    ///
    /// [S3] §21.2 needs the sprung/unsprung split for all three of its load
    /// transfer terms, and `Tireand suspension.md` §27.2 records that [S3]
    /// gives no typical unsprung mass fraction of its own - so this remains
    /// [S1] material.
    double unsprungFractionDriven    = 0.15;
    double unsprungFractionNonDriven = 0.10;

    //-----------------------------------------------------------------------
    //  CAMBER
    //-----------------------------------------------------------------------

    /// Camber change of the wheel relative to the BODY, per unit body roll
    /// [rad/rad]. KB Eq. 6-51's d(gamma)/d(phi).
    ///
    /// Ground camber is gamma_g = gamma_b + phi (KB Eq. 6-50), so this value
    /// controls how much of the body's roll the tyre actually sees:
    ///     0   no camber compensation - the wheel leans with the body, and the
    ///         outside tyre gains positive camber, losing lateral force
    ///    -1   perfect compensation - the wheel stays upright in roll
    ///
    /// NOT QUANTIFIED BY ANY SOURCE. KB Eq. 6-51 leaves the characteristic as
    /// an unspecified function f_gamma(track, geometry, roll angle) - gap G13.
    /// [S3] §16.4 advances it to a closed-form RATE, camber change per inch of
    /// travel = arctan(1/fvsa length), but `Tireand suspension.md` §27.2 records
    /// that no source gives an fvsa length for any vehicle, so the rate cannot
    /// be evaluated. Gap G13 is PARTIAL.
    ///
    /// The default of 0 is the explicit "not modelled" case - the wheel simply
    /// leans with the body - rather than an invented compensation figure.
    /// See README -> Open Questions.
    double camberChangePerRoll = 0.0;

    //-----------------------------------------------------------------------
    //  DAMPING RATIO  -  the quarter-car / terrain-following model
    //-----------------------------------------------------------------------

    /// Damper damping ratio zeta = C / C_crit at the front / rear axle [-],
    /// used only by the per-wheel terrain-following quarter-car model
    /// (`Tireand suspension.md` §18, [S3] RCVD Ch.22 p.787-788,
    /// C_crit = 2*sqrt(k*m)). [S3] p.810 gives typical values: passenger
    /// cars ~0.25, "road-holding optimum" ~0.5, race cars "considerably
    /// higher" than passenger. Default is the passenger figure.
    ///
    /// On a flat track this value has NO effect - see WheelState.h's note on
    /// why the terrain states are an exact fixed point at zero regardless of
    /// zeta - so it is inert for every vehicle that never encounters terrain.
    double dampingRatioFront = 0.25;
    double dampingRatioRear  = 0.25;

    void validate();
};

} // namespace vsim
