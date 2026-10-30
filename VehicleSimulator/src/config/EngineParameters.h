#pragma once
//===========================================================================
//  EngineParameters.h  --  what the detailed engine model needs.
//
//  SOURCE NOTICE — READ BEFORE EDITING
//  ------------------------------------------------------------------------
//  Every other file in src/config/ is traced to this project's own Knowledge
//  Base ([S1] Gillespie, [S2] Blundell & Harty, [S3] Milliken & Milliken —
//  see README, "Every equation here comes from one place"). NONE of those
//  three sources gives an engine torque map: it is recorded as KB gap G17,
//  "T_e is an input throughout Chapter 2/5; no model or representative map
//  is given" (VEHICLE_DYNAMICS_KNOWLEDGE_BASE.md), and the project's own
//  rule is explicit — "where the Knowledge Base does not contain enough to
//  implement something, it is not implemented and not guessed."
//
//  This struct is the documented exception, invited by RoadLoad.cpp's own
//  comment on engineBrakingCoeff: "Replace this the moment a real engine map
//  and gearbox exist." Every field below is sourced from a FOURTH, separate
//  and domain-appropriate reference — Heywood, "Internal Combustion Engine
//  Fundamentals" (McGraw-Hill, 1988) — via the EngineSim project in this same
//  repository (../EngineSim/ENGINE_KB.md, ../EngineSim/engine_sim/*.py). It
//  is NOT part of this project's S1/S2/S3 vehicle-dynamics Knowledge Base,
//  and equation numbers cited below are Heywood's, not Gillespie's or
//  Milliken's. Anything below with no Heywood citation is AUTHORED, exactly
//  like wheelInertia/yawInertia in VehicleParameters.h, and is flagged the
//  same way.
//
//  See src/physics/Engine.h for what these parameters drive.
//===========================================================================

#include "config/ResolvedEngineParameters.h"
#include "core/Units.h"

namespace vsim {

struct EngineParameters {
    // --- identification --------------------------------------------------
    // No default name here; EngineParameters only exists on a vehicle that
    // opted into the detailed model (see VehicleParameters::hasDetailedEngine).

    // --- geometry (Heywood Eq 2.1-2.8, ENGINE_KB.md Part XVII SA) ---------
    double bore           = 0.0825;  ///< [m]
    double stroke         = 0.086;   ///< [m]
    double rodLength      = 0.144;   ///< [m] connecting rod, centre to centre
    double compressionRatio = 10.0;  ///< [-] Heywood Eq 2.1
    int    numCylinders   = 4;

    double displacementPerCylinder() const {
        return (kPi / 4.0) * bore * bore * stroke;   ///< V_d = (pi/4) B^2 S
    }
    double totalDisplacement() const {
        return displacementPerCylinder() * static_cast<double>(numCylinders);
    }
    double clearanceVolume() const {
        return (compressionRatio > 1.0)
            ? displacementPerCylinder() / (compressionRatio - 1.0)
            : displacementPerCylinder();
    }

    // --- rotational inertia (Heywood Eq 14.52) ----------------------------
    //
    /// I_E [kg.m^2] - crankshaft, reciprocating assembly AND FLYWHEEL.
    ///
    /// NOT the value EngineSim uses (0.015). That figure is a bare engine on
    /// a dyno, which is what EngineSim simulates; a car's engine carries a
    /// flywheel whose whole purpose is to add rotational inertia, and it
    /// dominates the total. Using the dyno figure here makes the engine
    /// absurdly twitchy - it snaps from idle to redline in a few hundredths
    /// of a second under no load - and makes a clutch launch impossible to
    /// control, because the engine outruns the clutch faster than the car
    /// can pick up any speed at all.
    ///
    /// AUTHORED: neither Heywood nor this project's vehicle-dynamics KB
    /// gives a flywheel inertia. Same status as VehicleParameters::wheelInertia.
    double engineInertia  = 0.20;    ///< [kg.m^2]

    // --- speed limits --------------------------------------------------
    // idleRpm/redlineRpm are AUTHORED, per-engine design choices — Heywood
    // gives no "correct" value for an unspecified engine. The rev-limiter
    // MECHANISM (torque-cut with hysteresis) is also AUTHORED: EngineSim's
    // own KB_GAPS.md was corrected during this project to record that "rev
    // limiter", "redline", and the section it was previously (wrongly)
    // attributed to do not appear anywhere in ENGINE_KB.md at all. Do not
    // re-introduce that citation here.
    double idleRpm         = 800.0;
    double redlineRpm      = 7000.0;
    double limiterHysteresisRpm = 200.0;  ///< AUTHORED — prevents limiter chatter

    double idleOmega() const    { return idleRpm    * kTwoPi / 60.0; }  ///< [rad/s]
    double redlineOmega() const { return redlineRpm * kTwoPi / 60.0; }  ///< [rad/s]
    double limiterHysteresisOmega() const { return limiterHysteresisRpm * kTwoPi / 60.0; }

    // --- friction (Heywood Eq 13.5/13.6) -----------------------------------
    // tfmep = C1 + C2*N + C3*N^2, N in rev/s. Same coefficients and same
    // physical meaning as EngineSim's engine_sim/config.py.
    double frictionC1Pa       = 97000.0;   ///< [Pa]           0.97 bar
    double frictionC2PaPerRevS = 15.0;     ///< [Pa/(rev/s)]   0.15 bar/1000rev/s
    double frictionC3PaPerRevS2 = 0.005;   ///< [Pa/(rev/s)^2] 0.05 bar/(1000rev/s)^2
    double exhaustBackPressurePa = 103000.0;  ///< [Pa]

    // --- air system --------------------------------------------------------
    double ambientPressurePa      = 101325.0;  ///< [Pa]
    double intakeTemperatureK     = 293.15;    ///< [K]
    double maxVolumetricEfficiency = 0.92;     ///< [-] Heywood Eq 2.26/2.27

    /// Engine speed that sets how fast eta_v falls away with rpm, in the
    /// authored Eq 2.26 shape eta_v = max * (1 - 0.5 * (rpm/ref)^1.5) - see
    /// volumetricEfficiency() in physics/Engine.cpp. AUTHORED: 6000 is the
    /// value EngineSim ported with for its 2.0 L engine, and stays the
    /// default so every existing vehicle is unchanged. A higher-revving
    /// engine whose breathing holds up further (bigger valves, tuned
    /// runners) needs a higher figure, or its power peak lands far below
    /// where the real one does.
    double veReferenceRpm = 6000.0;            ///< [rpm]

    // --- turbocharger (optional) -------------------------------------------
    //
    // Opted into by a `turbo:` block, exactly as `detailed:` opts a vehicle
    // into this struct at all. With no block, hasTurbo stays false and the
    // engine is naturally aspirated, identical to before.
    //
    // WHAT IS HEYWOOD: the compressor raises charge TEMPERATURE as well as
    // pressure, by Eq 6.40 rearranged,
    //     T_02 = T_01 * (1 + ((p_02/p_01)^((gamma-1)/gamma) - 1) / eta_C),
    // and an aftercooler (Fig 14-24) takes some of that back out. The hotter
    // the charge, the less of it fits in the cylinder (Eq 2.27's rho_i), so
    // boost does not buy torque in proportion to pressure.
    //
    // WHAT IS AUTHORED: the boost LEVEL (a wastegate setting, not physics),
    // the spool curve, and the lag. Heywood's own turbo dynamics, Eq 14.51
    // (I_TC w dw/dt + B w^2 = W_C + W_T), needs compressor and turbine MAPS
    // for a specific turbocharger, which the KB does not give for any unit.
    // It is stood in for by a first-order lag toward a target boost that
    // rises with engine speed (more exhaust energy) - the turbo-lag
    // behaviour Eq 14.51 produces (Fig 14-25), not its mechanism. Same
    // status as the idle governor (EngineSim gap G-E03).
    bool   hasTurbo = false;
    double maxBoostPa = 0.0;              ///< [Pa] gauge, wastegate setting. AUTHORED
    double boostThresholdRpm = 2500.0;    ///< [rpm] boost starts building. AUTHORED
    double fullBoostRpm = 4000.0;         ///< [rpm] wastegate opens. AUTHORED
    double spoolTimeConstantS = 0.6;      ///< [s] lag building boost. AUTHORED
    double blowOffTimeConstantS = 0.08;   ///< [s] boost dumped on a lift. AUTHORED
    /// Compressor isentropic efficiency eta_C [-], Heywood Eq 6.40. 0.70 is
    /// the figure Heywood's own Prob 6.9 uses; his maps (Fig 6-46) peak
    /// around 0.7-0.8 for automotive radial compressors.
    double compressorEfficiency = 0.70;
    /// Aftercooler (intercooler) effectiveness [-]: the fraction of the
    /// compressor's temperature rise it removes. AUTHORED - Heywood shows
    /// the component (Fig 14-24) but gives no effectiveness figure.
    double intercoolerEffectiveness = 0.0;

    // --- fuel system (Heywood Table D.4) ------------------------------------
    double stoichAfr           = 14.7;      ///< [-] gasoline
    double lowerHeatingValue   = 44.0e6;    ///< [J/kg]
    double combustionEfficiency = 0.95;     ///< [-]

    // --- crank-angle-resolved core (optional) --------------------------------
    //
    // Opted into by a `resolved:` block inside `detailed:`, the same
    // presence-is-the-switch pattern as `turbo:` above. With it, the engine
    // is physics/ResolvedEngine - EngineSim v0.7's resolved cylinders - and
    // everything above this line splits three ways:
    //
    //   STILL USED   geometry, engineInertia, idle/redline/limiter hysteresis,
    //                the three friction coefficients, ambient pressure, intake
    //                temperature, the fuel constants.
    //   SEEDS ONLY   maxVolumetricEfficiency - the ECU's first guess at its
    //                air model; the cylinders' real breathing replaces it
    //                within a few cycles.
    //   NOT USED     veReferenceRpm, exhaustBackPressurePa and the whole
    //                authored turbo model (spool curve, lag time constants):
    //                volumetric efficiency, back-pressure, boost and lag are
    //                outputs of the resolved core, not inputs. Its turbo is
    //                `resolved.turbo`. validate() mirrors that turbo's target
    //                boost and compressor efficiency into hasTurbo / maxBoostPa /
    //                compressorEfficiency above, so read-only consumers (UI,
    //                sound) need no second code path.
    //
    // FRICTION COEFFICIENTS: the defaults above (15 and 0.005) are the ones
    // the cycle-averaged port was calibrated with. EngineSim has since found
    // them to be a unit slip - Heywood Eq 13.6 takes N in rev/MIN, so per
    // rev/s the coefficients are 900 and 18 (3.7 bar at 6000 rpm instead of
    // 1.0). They are left alone here so every existing cycle-averaged vehicle
    // is unchanged; the config loader starts a `resolved:` engine from the
    // corrected 900 / 18 instead (config/Config.cpp).
    bool useResolved = false;
    ResolvedEngineParameters resolved;

    /// Clamp every field into a physically sensible range, matching
    /// VehicleParameters::validate()'s role for the rest of the vehicle.
    void validate() {
        bore                    = clampd(sanitize(bore, 0.0825), 0.03, 0.20);
        stroke                  = clampd(sanitize(stroke, 0.086), 0.03, 0.20);
        rodLength               = clampd(sanitize(rodLength, 0.144), bore, 0.40);
        compressionRatio        = clampd(sanitize(compressionRatio, 10.0), 6.0, 23.0);
        numCylinders            = (numCylinders >= 1 && numCylinders <= 16) ? numCylinders : 4;
        engineInertia           = clampd(sanitize(engineInertia, 0.20), 0.001, 5.0);
        idleRpm                 = clampd(sanitize(idleRpm, 800.0), 300.0, 2000.0);
        redlineRpm              = clampd(sanitize(redlineRpm, 7000.0), idleRpm + 500.0, 20000.0);
        limiterHysteresisRpm    = clampd(sanitize(limiterHysteresisRpm, 200.0), 10.0, 1000.0);
        frictionC1Pa            = clampd(sanitize(frictionC1Pa, 97000.0), 0.0, 5.0e5);
        // Upper bounds leave room for Heywood Eq 13.6 converted per rev/s
        // (900 and 18) - see the friction note on `useResolved` above.
        frictionC2PaPerRevS     = clampd(sanitize(frictionC2PaPerRevS, 15.0), 0.0, 5000.0);
        frictionC3PaPerRevS2    = clampd(sanitize(frictionC3PaPerRevS2, 0.005), 0.0, 100.0);
        exhaustBackPressurePa   = clampd(sanitize(exhaustBackPressurePa, 103000.0), 5.0e4, 3.0e5);
        ambientPressurePa       = clampd(sanitize(ambientPressurePa, 101325.0), 5.0e4, 1.2e5);
        intakeTemperatureK      = clampd(sanitize(intakeTemperatureK, 293.15), 250.0, 350.0);
        // Above 1 only for a resolved engine, where it is no more than the
        // ECU's opening guess (a boosted EngineSim engine file may say 1.15).
        maxVolumetricEfficiency = clampd(sanitize(maxVolumetricEfficiency, 0.92), 0.3,
                                         useResolved ? 1.5 : 1.0);
        veReferenceRpm          = clampd(sanitize(veReferenceRpm, 6000.0), 2000.0, 20000.0);
        maxBoostPa              = clampd(sanitize(maxBoostPa, 0.0), 0.0, 3.0e5);
        boostThresholdRpm       = clampd(sanitize(boostThresholdRpm, 2500.0), 0.0, redlineRpm);
        fullBoostRpm            = clampd(sanitize(fullBoostRpm, 4000.0), boostThresholdRpm + 100.0, 25000.0);
        spoolTimeConstantS      = clampd(sanitize(spoolTimeConstantS, 0.6), 0.01, 5.0);
        blowOffTimeConstantS    = clampd(sanitize(blowOffTimeConstantS, 0.08), 0.01, 5.0);
        compressorEfficiency    = clampd(sanitize(compressorEfficiency, 0.70), 0.3, 0.9);
        intercoolerEffectiveness = clampd(sanitize(intercoolerEffectiveness, 0.0), 0.0, 0.95);
        if (maxBoostPa <= 0.0) hasTurbo = false;
        stoichAfr               = clampd(sanitize(stoichAfr, 14.7), 5.0, 40.0);
        lowerHeatingValue       = clampd(sanitize(lowerHeatingValue, 44.0e6), 1.0e6, 1.0e8);
        combustionEfficiency    = clampd(sanitize(combustionEfficiency, 0.95), 0.3, 1.0);

        if (useResolved) {
            resolved.validate(numCylinders, idleRpm);
            // Derived read-outs for consumers that only know the fields above.
            hasTurbo = resolved.useTurbo;
            maxBoostPa = hasTurbo
                ? std::max(1.0, resolved.turbo.targetBoostPa - ambientPressurePa) : 0.0;
            if (hasTurbo) {
                compressorEfficiency = resolved.turbo.compressorEfficiency;
                intercoolerEffectiveness = resolved.turbo.intercoolerEffectiveness;
            }
        }
    }
};

} // namespace vsim
