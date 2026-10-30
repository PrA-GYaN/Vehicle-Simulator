#pragma once
//===========================================================================
//  TireCompound.h  --  dry-weather tyre compounds: Soft, Medium, Hard.
//
//  THIS IS AUTHORED, NOT FROM THE KNOWLEDGE BASE.
//  -----------------------------------------------
//  Everything in TireParameters.h is traced to a source. Nothing here is:
//  none of the three references gives compound data, a wear law or a
//  grip-versus-wear curve, so the numbers below were chosen to reproduce the
//  ONE thing that is public knowledge about the three-compound system in
//  Grand Prix racing - the trade it sets up:
//
//      Soft     most grip, shortest life
//      Medium   in between
//      Hard     least grip, longest life
//
//  with a step in pace of a few percent of grip between neighbours, and a
//  tyre that fades slowly as it wears and then falls away sharply near the
//  end of its life ("the cliff"). Treat the magnitudes as tuning values.
//
//  WHAT A COMPOUND CHANGES
//  -----------------------
//    * Peak friction, both directions, by a factor on the vehicle's own
//      configured tyre (so a compound is relative to that car's tyre, not an
//      absolute number that would make an SUV grip like a race car).
//    * How fast the tyre wears. Wear is driven by the energy the contact
//      patch dissipates in SLIDING - force times sliding speed - which is the
//      one wear quantity the model already computes honestly: a tyre rolling
//      freely wears nothing, a tyre cornering hard wears steadily, a spinning
//      or locked one wears very fast.
//
//  WHAT IT DOES NOT MODEL
//  ----------------------
//  Temperature. Real compounds differ as much in their working temperature
//  window as in outright grip, and a cold or overheated tyre is slow
//  whatever it is made of. There is no tyre thermal state anywhere in this
//  simulator, so warm-up, overheating, graining and blistering do not exist
//  here. Nor do flat spots, punctures, or wet-weather tyres.
//
//  Standard is "the tyre exactly as the vehicle file describes it, and it
//  never wears" - the behaviour every vehicle had before compounds existed,
//  and still the default.
//===========================================================================

#include <algorithm>
#include <string>

#include "config/TireParameters.h"

namespace vsim {

enum class TireCompound { Standard, Soft, Medium, Hard };

struct TireCompoundSpec {
    const char* name;
    double gripFactor;   ///< on the configured friction coefficient
    double lifeEnergy;   ///< sliding energy that wears one tyre out [J]; 0 = never
};

inline const TireCompoundSpec& tireCompoundSpec(TireCompound c) {
    // Life: a tyre cornering at its limit dissipates on the order of 10-15 kW
    // in the contact patch; over a lap of the Apex circuit that comes to a
    // few hundred kJ per tyre. 6 MJ is therefore some 15 hard laps.
    static const TireCompoundSpec kSpecs[] = {
        {"Standard", 1.00,  0.0},
        {"Soft",     1.10,  6.0e6},
        {"Medium",   1.05, 12.0e6},
        {"Hard",     1.00, 21.0e6},
    };
    return kSpecs[static_cast<int>(c)];
}

inline const char* tireCompoundName(TireCompound c) { return tireCompoundSpec(c).name; }

/// "soft" / "medium" / "hard" / "standard", case as written in YAML.
/// Anything else is Standard.
inline TireCompound tireCompoundFromString(const std::string& s) {
    if (s == "soft"   || s == "Soft")   return TireCompound::Soft;
    if (s == "medium" || s == "Medium") return TireCompound::Medium;
    if (s == "hard"   || s == "Hard")   return TireCompound::Hard;
    return TireCompound::Standard;
}

inline TireCompound nextTireCompound(TireCompound c) {
    return static_cast<TireCompound>((static_cast<int>(c) + 1) % 4);
}

/// Grip left in a tyre worn by the fraction `wear` (0 new, 1 finished), as a
/// factor on its new grip: a slow, steady fade for most of its life, then
/// the cliff over the last fifth.
inline double tireWearGripFactor(double wear) {
    const double w = std::min(std::max(wear, 0.0), 1.0);
    const double u = std::min(std::max((w - 0.80) / 0.20, 0.0), 1.0);
    const double cliff = u * u * (3.0 - 2.0 * u);
    return 1.0 - 0.05 * w - 0.22 * cliff;
}

/// The vehicle's tyre, as this compound at this state of wear.
inline TireParameters applyTireCompound(const TireParameters& base, TireCompound c,
                                        double wear) {
    if (c == TireCompound::Standard) return base;
    TireParameters p = base;
    p.lateralFriction *= tireCompoundSpec(c).gripFactor * tireWearGripFactor(wear);
    return p;
}

} // namespace vsim
