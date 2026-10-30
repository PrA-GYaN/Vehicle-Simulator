#pragma once
//===========================================================================
//  Gearbox.h  --  gear ratio state + a simple automatic shift.
//
//  AUTHORED, not from any Knowledge Base — see config/GearboxParameters.h
//  for exactly what that means here and why (KB gaps G18/G19/G20).
//
//  SCOPE: forward gears + automatic shifting only. There is no clutch/gear-
//  select input in ControlInput (see core/VehicleState.h), so there is
//  nothing for a driver to hold in neutral or select reverse with. Adding
//  either is future work that needs an input first, not a physics change —
//  documented here rather than faked.
//===========================================================================

#include "config/GearboxParameters.h"

namespace vsim {

/// Gear index meaning NEUTRAL - no ratio, nothing transmitted.
///
/// Forward gears are 0-based (0 = 1st), so neutral has to sit below them.
/// currentRatio() already returns 0 for any out-of-range index, and a zero
/// ratio is exactly what neutral is, so the torque path in Powertrain needs
/// no special case: zero ratio means the clutch cannot lock and no torque
/// reaches the wheels, which is the definition of being in neutral.
inline constexpr int kNeutralGear = -1;

class Gearbox {
public:
    explicit Gearbox(const GearboxParameters& params);

    /// 0-based forward gear index (0 = 1st), or kNeutralGear.
    int currentGear() const { return m_currentGear; }

    bool inNeutral() const { return m_currentGear <= kNeutralGear; }

    /// Engine-turns-per-wheel-turn for the CURRENT gear: ratio[gear] * finalDrive.
    double currentRatio() const;

    /// Advance the gearbox one step.
    ///
    /// In ShiftMode::Automatic this considers an automatic shift against the
    /// configured thresholds. In ShiftMode::Manual it only ages the shift
    /// debounce timer - gear changes then come from shiftUp()/shiftDown().
    ///
    /// `wheelOmega` is the driven wheel's own angular speed [rad/s] (i.e.
    /// vehicleSpeed / wheelRadius, magnitude); `redlineOmega` is the engine's
    /// own redline [rad/s], used to turn the AUTHORED shift-point fractions
    /// in GearboxParameters into actual thresholds.
    void update(double wheelOmega, double redlineOmega, double dt);

    /// Driver-commanded shift. Returns true if a gear change actually
    /// happened - false at the end of the box, or while the shift debounce
    /// from the previous change is still running.
    ///
    /// Shifting down out of 1st selects NEUTRAL, and shifting up out of
    /// neutral selects 1st, so the whole range is reachable with two keys.
    ///
    /// Deliberately usable in BOTH shift modes: blipping a manual shift on an
    /// otherwise automatic box is a real thing, and refusing it here would
    /// mean the UI had to know the mode to know whether to offer the button.
    bool shiftUp();
    bool shiftDown();

    /// Highest gear index (0-based), i.e. gearRatios.size() - 1.
    int topGear() const;

    void reset();  ///< back to 1st gear, shift timer cleared

    const GearboxParameters& parameters() const { return m_params; }

    /// Live access, so the UI can switch shift/clutch mode without rebuilding
    /// the vehicle (which would reset the car's state along with it).
    GearboxParameters& mutableParameters() { return m_params; }

private:
    GearboxParameters m_params;
    int m_currentGear = 0;
    double m_timeSinceShiftS = 0.0;
};

} // namespace vsim
