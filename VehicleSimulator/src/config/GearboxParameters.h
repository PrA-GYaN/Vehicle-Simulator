#pragma once
//===========================================================================
//  GearboxParameters.h  --  what the driveline needs to turn engine torque
//  into wheel torque.
//
//  SOURCE NOTICE
//  ------------------------------------------------------------------------
//  Gear ratios, final drive, and shift points are per-vehicle DESIGN DATA,
//  not physics — Gillespie/Blundell & Harty/Milliken give the RELATIONSHIP
//  (KB Section 5.4/5.8: engine and wheel are in a fixed ratio) but never a
//  ratio SET for any vehicle, so every number below is AUTHORED, exactly
//  like VehicleParameters::wheelInertia.
//
//  What is deliberately NOT here, because the vehicle-dynamics Knowledge
//  Base explicitly does not cover it (OPEN gaps, not guessed):
//    * torque converter behaviour (KB gap G18)
//    * a differential / torque-split model (KB gap G19) — this driveline
//      sends equal torque to both driven wheels, the open-differential case
//    * clutch engagement dynamics (KB gap G20) — there is no clutch pedal
//      input in ControlInput, so gear changes here are treated as an
//      instantaneous, lossless re-ratio, and take-off from a dead stop is
//      handled by the engine's own idle governor rather than by modelling
//      slip (see physics/Powertrain.h for exactly where that shows up).
//===========================================================================

#include <vector>

#include "core/Units.h"

namespace vsim {

/// Who changes gear.
enum class ShiftMode {
    Automatic,  ///< the box shifts itself against the thresholds below
    Manual      ///< the driver shifts; the thresholds are ignored
};

/// Who works the clutch.
enum class ClutchMode {
    Automatic,  ///< progressively engaged for you on a launch
    Manual      ///< driven by ControlInput::clutch (a real pedal)
};

const char* shiftModeName(ShiftMode m);
const char* clutchModeName(ClutchMode m);

struct GearboxParameters {
    //-----------------------------------------------------------------------
    //  MODES
    //
    //  Independent on purpose: "automatic shifter, manual clutch" is a real
    //  and useful combination (it is how you drive a sequential box), and so
    //  is "manual shifter, automatic clutch" (a paddle-shift car, or anyone
    //  who does not want to manage a third pedal on a keyboard).
    //-----------------------------------------------------------------------
    ShiftMode  shiftMode  = ShiftMode::Automatic;
    ClutchMode clutchMode = ClutchMode::Automatic;
    /// Forward gear ratios, 1st to top gear, in order. Engine turns are
    /// `ratio[gear] * finalDrive` for one wheel turn. AUTHORED per vehicle.
    std::vector<double> gearRatios = {3.36, 2.09, 1.47, 1.14, 0.92, 0.78};

    /// Final drive (differential) ratio. AUTHORED per vehicle.
    double finalDrive = 3.90;

    /// Driveline mechanical efficiency [-] — gearbox + differential losses.
    /// NOT A KNOWLEDGE BASE VALUE (neither source quantifies driveline
    /// losses). 0.90 is a commonly cited rule-of-thumb order of magnitude
    /// for a manual transmission, held here as authored data with no
    /// citation, same status as VehicleParameters::wheelInertia.
    double efficiency = 0.90;

    /// Upshift when locked engine speed exceeds this fraction of redline.
    /// AUTHORED control logic — there is no clutch/gear-select input to
    /// drive a driver-commanded shift, so the box shifts itself.
    double upshiftRedlineFraction = 0.92;

    /// Downshift when locked engine speed falls below this fraction of
    /// redline (checked against the ratio one gear down). AUTHORED.
    double downshiftRedlineFraction = 0.35;

    /// Minimum time between automatic shifts [s], so the box does not
    /// hunt between two gears at a borderline speed. AUTHORED.
    double minShiftIntervalS = 0.4;

    //-----------------------------------------------------------------------
    //  LAUNCH CLUTCH
    //
    //  Needed because a rigid driveline cannot start a stationary car: below
    //  idle-implied wheel speed there is no way to connect an idling engine
    //  to stopped wheels without either stalling the engine or teleporting
    //  torque. Real cars solve it with a slipping friction clutch or a torque
    //  converter; the KB covers NEITHER (gaps G20 and G18, both OPEN).
    //
    //  What is modelled here is the friction-clutch case, and only its
    //  defining property: a friction clutch transmits torque up to its
    //  capacity, and while the two sides are slipping it transmits exactly
    //  that capacity. That is Coulomb friction at a known contact, not an
    //  invented powertrain characteristic - but the CAPACITY VALUE and the
    //  engagement schedule below are AUTHORED, the same status as
    //  VehicleParameters::wheelInertia. A real clutch's capacity comes from
    //  plate area, friction coefficient and clamp load, none of which any
    //  source here gives.
    //-----------------------------------------------------------------------

    /// Torque the clutch can transmit while slipping [N.m], engine side.
    /// AUTHORED.
    ///
    /// Must sit ABOVE the engine's peak net torque, or the clutch can never
    /// pull a revving engine back down to the wheels and the two sides never
    /// converge - the car then slips the clutch indefinitely instead of
    /// locking. The progressive engagement below is what stops a capacity
    /// this high from simply stalling the engine the instant it bites.
    double clutchCapacityNm = 260.0;

    /// Time for the clutch to be fully "let out" during a launch [s].
    /// AUTHORED - this is a driver's left foot, which no source models.
    double clutchEngageTimeS = 0.8;

    /// Engine speed below which the clutch releases completely, as a
    /// fraction of idle speed. Stops a heavily-loaded launch from dragging
    /// the engine down to a stall - which is what a driver (or an automatic)
    /// does by lifting or slipping further. AUTHORED.
    double clutchReleaseIdleFraction = 0.55;

    /// Engine speed at which the AUTOMATIC clutch is fully home on a
    /// full-throttle launch [rpm]. 0 (the default) keeps the plain timed
    /// engagement above, which starts biting the moment the throttle opens.
    ///
    /// That was right for the cycle-averaged engine, which (unphysically)
    /// made its best torque at idle. A resolved engine makes what a real one
    /// does there - and a turbocharged one makes next to nothing until the
    /// turbines have gas to work with - so a clutch that grabs at idle bogs
    /// the launch for seconds. With this set, the clutch instead bites in
    /// proportion to engine speed, from just above idle to this speed (scaled
    /// down with part throttle, so a gentle start is still a gentle start),
    /// which is what a driver does: bring the revs up, feed the clutch in
    /// against them. The engine settles where its torque equals what the
    /// clutch is passing, and the clutch goes home when the car catches up.
    /// AUTHORED, like the rest of the engagement schedule - a left foot.
    double launchRpm = 0.0;

    void validate() {
        // (modes are enums - no clamping needed)
        if (gearRatios.empty()) gearRatios = {3.36, 2.09, 1.47, 1.14, 0.92, 0.78};
        for (double& r : gearRatios) r = clampd(sanitize(r, 1.0), 0.1, 10.0);
        finalDrive              = clampd(sanitize(finalDrive, 3.90), 1.0, 10.0);
        efficiency              = clampd(sanitize(efficiency, 0.90), 0.5, 1.0);
        upshiftRedlineFraction  = clampd(sanitize(upshiftRedlineFraction, 0.92), 0.3, 1.0);
        downshiftRedlineFraction = clampd(sanitize(downshiftRedlineFraction, 0.35), 0.05,
                                           upshiftRedlineFraction - 0.05);
        minShiftIntervalS       = clampd(sanitize(minShiftIntervalS, 0.4), 0.05, 5.0);
        clutchCapacityNm        = clampd(sanitize(clutchCapacityNm, 260.0), 0.0, 5000.0);
        clutchEngageTimeS       = clampd(sanitize(clutchEngageTimeS, 0.8), 0.05, 5.0);
        clutchReleaseIdleFraction =
            clampd(sanitize(clutchReleaseIdleFraction, 0.55), 0.0, 0.95);
        launchRpm               = clampd(sanitize(launchRpm, 0.0), 0.0, 20000.0);
    }
};

} // namespace vsim
