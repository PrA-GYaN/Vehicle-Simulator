#pragma once
//===========================================================================
//  Powertrain.h  --  ties Engine + Gearbox into what Vehicle.cpp needs:
//  a drive acceleration and an engine-braking deceleration, both >= 0,
//  matching the existing DriveCommand / RoadLoad contracts exactly. This is
//  the swap point analogous to IVehicleDynamics — Vehicle owns one of these
//  only for a vehicle that opted into the detailed engine (see
//  VehicleParameters::hasDetailedEngine), and everything downstream of
//  DriveCommand/RoadLoad is unaware it exists.
//
//  COUPLING MODEL — read this before changing it
//  ------------------------------------------------------------------------
//  A real driveline needs a clutch or a torque converter to connect an
//  idling engine to stationary wheels without stalling. Neither is in either
//  Knowledge Base this repository is built from: torque-converter behaviour
//  is KB gap G18 and clutch engagement is G20, both OPEN. What is modelled
//  here is the friction-clutch case and only its defining property — a
//  clutch transmits torque up to its capacity, and transmits exactly that
//  capacity while slipping. See config/GearboxParameters.h for which parts
//  of that are Coulomb friction and which are authored.
//
//  Three states, not two:
//    * LOCKED    — driveline rigid. Engine speed is READ from wheel speed,
//                  not integrated; torque flows both ways through the gear
//                  ratio, which is where engine braking comes from.
//    * SLIPPING  — the clutch passes its capacity torque. The engine
//                  integrates its own speed against that as a load. This is
//                  what makes a standing start possible.
//    * RELEASED  — clutch out of the picture; the engine free-idles and
//                  nothing reaches the wheels.
//
//  THE ENGINE behind this is one of two models (physics/Engine.h): the
//  cycle-averaged torque chain, or EngineSim's crank-angle-resolved core.
//  Nothing in the coupling model above changes with it - LOCKED imposes the
//  crank speed on either, SLIPPING and RELEASED let either integrate its
//  own. The three places they differ are marked where they occur in
//  Powertrain.cpp: the locked rev limiter (the resolved engine cuts its own
//  fuel), what `engineTorqueNm` reports, and stalling.
//
//  SHIFTING and CLUTCHING are independently configurable (ShiftMode /
//  ClutchMode in config/GearboxParameters.h), because "automatic shifter,
//  manual clutch" and "manual shifter, automatic clutch" are both real ways
//  to drive. In manual-clutch mode the pedal comes straight from
//  ControlInput::clutch; in automatic mode the clutch is let out
//  progressively for you on a launch.
//
//===========================================================================

#include "config/EngineParameters.h"
#include "config/GearboxParameters.h"
#include "physics/Engine.h"
#include "physics/Gearbox.h"

namespace vsim {

struct PowertrainOutput {
    double driveAccelerationMps2 = 0.0;   ///< >= 0, feeds DriveCommand::driveAcceleration
    double engineBrakingMps2     = 0.0;   ///< >= 0, REPLACES RoadLoad::engineBraking when active
    double engineRpm             = 0.0;
    double engineTorqueNm        = 0.0;   ///< net torque the engine is producing right now
    int    currentGear           = 0;     ///< 0-based, 0 = 1st
    bool   locked                = false; ///< true = rigidly coupled to the wheels this step
    bool   limiterActive         = false;
    double clutchEngagement      = 0.0;   ///< [0,1] how far the clutch is engaged
};

class Powertrain {
public:
    Powertrain(const EngineParameters& engineParams,
               const GearboxParameters& gearboxParams,
               double wheelRadius,
               double vehicleMass);

    /// One physics step.
    ///
    /// `vehicleSpeed` is signed [m/s]; only its magnitude is used (matching
    /// RoadLoad::computeRoadLoad's convention) since this model does not
    /// offer a reverse gear (see GearboxParameters.h).
    ///
    /// `clutchPedal` [0,1] is only read in ClutchMode::Manual; `shiftRequest`
    /// (-1/0/+1) is only read in ShiftMode::Manual. Both default to the
    /// neutral "driver is not touching it" values so an automatic vehicle,
    /// and every experiment script, is unaffected.
    PowertrainOutput update(double throttle, double vehicleSpeed, double dt,
                            double clutchPedal = 0.0, int shiftRequest = 0);

    void reset();

    /// Switch shift/clutch mode on a RUNNING powertrain. Vehicle owns the
    /// authoritative VehicleParameters copy; this pushes a change into the
    /// live gearbox, which holds its own copy.
    void setModes(ShiftMode shift, ClutchMode clutch) {
        m_gearbox.mutableParameters().shiftMode  = shift;
        m_gearbox.mutableParameters().clutchMode = clutch;
    }

    const Engine&   engine()   const { return m_engine; }
    const Gearbox&  gearbox()  const { return m_gearbox; }

    /// Result of the most recent update() call. Default-constructed (all
    /// zero) before the first update() - Telemetry reads this after
    /// Vehicle::step() completes, rather than recomputing it.
    const PowertrainOutput& lastOutput() const { return m_lastOutput; }

private:
    Engine   m_engine;
    Gearbox  m_gearbox;
    double   m_wheelRadius;
    double   m_vehicleMass;
    PowertrainOutput m_lastOutput;

    /// Clutch state, with hysteresis - a clutch that locks and releases on
    /// the same threshold chatters. Locks when the wheels have caught the
    /// engine up (slip closed); releases only when the wheels fall below
    /// idle-implied speed, i.e. the car is nearly stopped.
    bool m_clutchLocked = false;

    /// How far the clutch is "let out" [0,1] during a launch. Ramps in over
    /// clutchEngageTimeS rather than snapping to full capacity, because that
    /// is what a driver's left foot does - and because a clutch that grabs
    /// at full capacity from an idling engine simply stalls it.
    double m_clutchEngagement = 0.0;

    /// Rev limiter for the LOCKED driveline, with its own hysteresis.
    ///
    /// Engine::update()'s limiter only guards the free-revving path, where
    /// the engine integrates its own speed. Once locked, engine speed is READ
    /// from the wheels, so nothing was capping it - a car held in gear could
    /// be driven arbitrarily far past redline by its own wheels with no
    /// consequence at all, which makes shifting pointless.
    bool m_lockedLimiterActive = false;
};

} // namespace vsim
