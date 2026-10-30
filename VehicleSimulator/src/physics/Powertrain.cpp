#include "physics/Powertrain.h"

#include <algorithm>
#include <cmath>

#include "core/Units.h"

namespace vsim {

Powertrain::Powertrain(const EngineParameters& engineParams,
                       const GearboxParameters& gearboxParams,
                       double wheelRadius,
                       double vehicleMass)
    : m_engine(engineParams)
    , m_gearbox(gearboxParams)
    , m_wheelRadius(std::max(1e-3, sanitize(wheelRadius, 0.34)))
    , m_vehicleMass(std::max(1.0, sanitize(vehicleMass, 1500.0))) {
    reset();
}

void Powertrain::reset() {
    m_engine.reset(/*idle=*/true);
    m_gearbox.reset();
    m_clutchLocked = false;
    m_clutchEngagement = 0.0;
    m_lockedLimiterActive = false;
    m_lastOutput = PowertrainOutput{};
}

PowertrainOutput Powertrain::update(double throttle, double vehicleSpeed, double dt,
                                    double clutchPedal, int shiftRequest) {
    PowertrainOutput out;

    const GearboxParameters& gp = m_gearbox.parameters();
    const double wheelOmega = std::abs(sanitize(vehicleSpeed)) / m_wheelRadius;  ///< [rad/s]
    const double idleOmega = m_engine.parameters().idleOmega();

    //-----------------------------------------------------------------------
    //  1. GEAR SELECTION.
    //
    //  The gearbox auto-shifts only in ShiftMode::Automatic (it checks the
    //  mode itself). A driver request is honoured in either mode - on an
    //  automatic that is a manual override, which the box may undo on its
    //  next evaluation, exactly like pulling a paddle on a real auto.
    //-----------------------------------------------------------------------
    m_gearbox.update(wheelOmega, m_engine.parameters().redlineOmega(), dt);
    if (shiftRequest > 0)      m_gearbox.shiftUp();
    else if (shiftRequest < 0) m_gearbox.shiftDown();

    out.currentGear = m_gearbox.currentGear();

    const double ratio = m_gearbox.currentRatio();
    const double impliedOmega = wheelOmega * ratio;

    //-----------------------------------------------------------------------
    //  2. CLUTCH ENGAGEMENT [0,1].  0 = fully released, 1 = fully engaged.
    //
    //  MANUAL   - straight from the pedal. This is the whole point of the
    //             mode: the driver decides, including badly.
    //  AUTOMATIC- progressively let out while there is throttle, because a
    //             clutch that grabs at full capacity from an idling engine
    //             stalls it, and one that engages proportionally to throttle
    //             lets the engine flare against no load. Both of those were
    //             tried and neither works; see README.
    //-----------------------------------------------------------------------
    if (gp.clutchMode == ClutchMode::Manual) {
        m_clutchEngagement = 1.0 - clampd(sanitize(clutchPedal), 0.0, 1.0);
    } else if (m_clutchLocked) {
        m_clutchEngagement = 1.0;          // already driving: pedal is up
    } else if (throttle > 0.02) {
        const double launchOmega = gp.launchRpm * kTwoPi / 60.0;
        if (launchOmega > idleOmega) {
            // Rev-matched launch (GearboxParameters::launchRpm): the clutch
            // bites in proportion to engine speed, so the engine is allowed
            // up to where it makes torque before it is asked for any. The
            // target is approached at the same rate the timed engagement
            // uses, in either direction - a foot, not a switch.
            const double biteOmega = 1.1 * idleOmega;
            const double fullOmega = std::max(
                biteOmega + 1.0,
                idleOmega + clampd(throttle, 0.0, 1.0) * (launchOmega - idleOmega));
            const double target = clampd((m_engine.omega() - biteOmega) / (fullOmega - biteOmega), 0.0, 1.0);
            if (ratio > 1e-9 && impliedOmega >= idleOmega && impliedOmega >= m_engine.omega()) {
                // The car has caught the engine up: there is no slip left to
                // meter, so the pedal comes all the way up and the lock test
                // below takes it from here. Feeding it in gradually instead
                // leaves the two sides chattering against each other at
                // matched speed - the clutch alternately dragging the engine
                // down and being kicked back up by it, once every few steps.
                m_clutchEngagement = 1.0;
            } else {
                m_clutchEngagement = moveToward(m_clutchEngagement, target,
                                                1.0 / std::max(1e-3, gp.clutchEngageTimeS), dt);
            }
        } else {
            m_clutchEngagement += dt / std::max(1e-3, gp.clutchEngageTimeS);
        }
    } else {
        m_clutchEngagement = 0.0;          // no throttle: free idle, no creep
    }
    m_clutchEngagement = clampd(m_clutchEngagement, 0.0, 1.0);

    //  Anti-stall. Applied in BOTH modes: the cycle-averaged engine has no
    //  stall or restart model, so dragged to zero it would leave the vehicle
    //  permanently dead with no way back. Slipping further is what a driver
    //  does anyway when the revs fall away. In manual mode this means the
    //  clutch will save you from a badly dumped pedal - a deliberate
    //  playability choice, not a claim about clutch physics.
    //
    //  The resolved engine CAN stall (below its stall speed the ECU loses
    //  sync and the fire goes out) and restarts itself on the starter - see
    //  Engine.cpp, stepResolved. The anti-stall is kept for it all the same:
    //  the release speed sits above every shipped engine's stall speed, so
    //  what you get from a dumped clutch is a bog, not three seconds of
    //  cranking.
    if (m_engine.omega() < idleOmega * gp.clutchReleaseIdleFraction) {
        m_clutchEngagement = 0.0;
    }

    const bool fullyEngaged = (m_clutchEngagement > 0.99);
    out.clutchEngagement = m_clutchEngagement;

    //-----------------------------------------------------------------------
    //  3. LOCK STATE, with hysteresis on two DIFFERENT conditions.
    //
    //  Lock only once the wheels have caught the engine up - that is what
    //  closing the slip means. Testing "implied >= idle" alone (as an earlier
    //  version did) locks while the engine is still spun up, teleporting its
    //  speed straight down to the wheel-implied value and destroying all its
    //  rotational energy in one step.
    //
    //  Release when the wheels drop below idle-implied speed (car nearly
    //  stopped, engine would be dragged to a stall) or when the clutch is no
    //  longer fully engaged - which is how a manual shift works.
    //-----------------------------------------------------------------------
    if (ratio <= 1e-9) {
        m_clutchLocked = false;
    } else if (m_clutchLocked) {
        if (impliedOmega < idleOmega || !fullyEngaged) m_clutchLocked = false;
    } else if (fullyEngaged && impliedOmega >= idleOmega && impliedOmega >= m_engine.omega()) {
        m_clutchLocked = true;
    }
    const bool locked = m_clutchLocked;
    out.locked = locked;

    //-----------------------------------------------------------------------
    //  4. TORQUE PATH.
    //-----------------------------------------------------------------------
    const double efficiency = gp.efficiency;

    // Reflected inertia: the crankshaft's own inertia, seen at the wheel
    // through the gear ratio, adds to the effective mass the drive force has
    // to accelerate (higher gears reflect less of it). Heywood does not give
    // this relation for a vehicle driveline, but it follows directly from
    // I_E and the ratio, both already in scope here.
    const double reflectedInertiaMass =
        m_engine.parameters().engineInertia * ratio * ratio / (m_wheelRadius * m_wheelRadius);
    const double effectiveMass = m_vehicleMass + reflectedInertiaMass;

    Engine::TorqueBreakdown breakdown;
    double wheelTorque = 0.0;

    if (locked) {
        // Rigid coupling: engine speed is READ from the wheel, not integrated.
        //
        // Which means the engine's OWN limiter cannot act here - it lives in
        // Engine::update(), on the path that integrates engine speed. Apply
        // it as what a rev limiter physically is: a fuel cut. With the
        // throttle forced shut the DFCO path in Engine takes over, so the
        // car decelerates against friction and pumping until it drops back
        // under the threshold. Hysteresis stops it chattering on the line.
        const double redlineOmega = m_engine.parameters().redlineOmega();
        const double hyst = m_engine.parameters().limiterHysteresisOmega();
        m_lockedLimiterActive = m_lockedLimiterActive
            ? (impliedOmega > redlineOmega - hyst)
            : (impliedOmega >= redlineOmega);

        // The resolved engine needs none of that: its limiter IS a fuel cut,
        // decided inside the engine from crank speed whichever way the crank
        // is being driven, so it gets the driver's throttle untouched (closing
        // the throttle as well would also pump its manifold down, which a
        // limiter does not do).
        const double engineThrottle =
            (m_lockedLimiterActive && !m_engine.isResolved()) ? 0.0 : throttle;
        breakdown = m_engine.evaluateTorqueAt(impliedOmega, engineThrottle, dt);
        if (m_engine.isResolved()) m_lockedLimiterActive = m_engine.limiterActive();
        out.engineRpm = impliedOmega * 60.0 / kTwoPi;
        wheelTorque = breakdown.netTorqueNm * ratio * efficiency;

    } else if (ratio > 1e-9 && m_clutchEngagement > 0.0) {
        // Slipping: a friction clutch transmits its capacity while the two
        // sides turn at different speeds. The engine integrates its own speed
        // against that as a load, so a launch genuinely pulls the engine down
        // and the governor genuinely fights back.
        m_lockedLimiterActive = false;
        //
        // Coulomb friction acts AGAINST the slip, so the direction depends
        // on which side is faster. Engine faster (a launch): the clutch
        // loads the engine and drives the wheels. Wheels faster (a turbo
        // engine bogged off-boost while the car keeps rolling, or a
        // downshift): the clutch spins the engine UP and brakes the car -
        // capped at what closes the slip this step, so it does not
        // overshoot and chatter across zero. Without this the clutch only
        // ever pulled the engine down, and an engine that could not out-rev
        // a slipping clutch never got the chance to lock.
        const double capacity = m_clutchEngagement * gp.clutchCapacityNm;
        const double slip = m_engine.omega() - impliedOmega;
        const double clutchTorque = (slip >= 0.0)
            ? capacity
            : -std::min(capacity, m_engine.parameters().engineInertia * (-slip) / std::max(dt, 1e-6));
        breakdown = m_engine.update(throttle, clutchTorque, dt);
        out.engineRpm = m_engine.rpm();

        // Only what the clutch passes reaches the wheels - NOT the engine's
        // whole net torque. That is the difference between slipping and locked.
        wheelTorque = clutchTorque * ratio * efficiency;

    } else {
        m_lockedLimiterActive = false;
        // Clutch fully released (or no gear): free idle, nothing transmitted.
        breakdown = m_engine.update(throttle, 0.0, dt);
        out.engineRpm = m_engine.rpm();
    }

    out.limiterActive = locked ? m_lockedLimiterActive : m_engine.limiterActive();
    // Resolved engine: the flywheel torque, whatever the clutch is doing with
    // it. Its net torque while slipping or free is what is left over to
    // accelerate the crank, which swings with every firing and is not what
    // anyone means by "engine torque".
    out.engineTorqueNm = m_engine.isResolved() ? breakdown.brakeTorqueNm
                                               : breakdown.netTorqueNm;

    const double driveForce = std::max(0.0, wheelTorque) / m_wheelRadius;
    const double brakeForce = std::max(0.0, -wheelTorque) / m_wheelRadius;

    out.driveAccelerationMps2 = (effectiveMass > 0.0) ? driveForce / effectiveMass : 0.0;
    out.engineBrakingMps2 = (m_vehicleMass > 0.0) ? brakeForce / m_vehicleMass : 0.0;

    m_lastOutput = out;
    return out;
}

} // namespace vsim
