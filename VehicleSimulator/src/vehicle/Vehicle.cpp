#include "vehicle/Vehicle.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "core/Coordinates.h"
#include "physics/RoadLoad.h"
#include "physics/SteeringGeometry.h"
#include "physics/TireModel.h"

namespace vsim {

Vehicle::Vehicle(const VehicleParameters& params,
                 const ControlsConfig& controls,
                 std::unique_ptr<IVehicleDynamics> dynamics)
    : m_params(params),
      m_controls(controls),
      m_dynamics(std::move(dynamics)) {
    m_params.validate();
    m_tcLevel = std::min(std::max(m_params.tractionControl, 0), kTractionControlLevels);
    if (m_params.hasDetailedEngine()) {
        m_powertrain = std::make_unique<Powertrain>(
            m_params.detailedEngine, m_params.gearbox, m_params.wheelRadius, m_params.mass);
    }
    updateWheelPoses(0.0);
}

void Vehicle::syncPowertrainModes() {
    if (m_powertrain) {
        m_powertrain->setModes(m_params.gearbox.shiftMode, m_params.gearbox.clutchMode);
    }
}

void Vehicle::setDynamics(std::unique_ptr<IVehicleDynamics> d) {
    if (d) m_dynamics = std::move(d);
}

//---------------------------------------------------------------------------
//  Actuator model
//---------------------------------------------------------------------------

void Vehicle::updateActuators(const ControlInput& request, double dt) {
    m_requested = request;
    m_requested.clampToValidRange();

    //-- steering ---------------------------------------------------------
    //
    // The driver's normalised request maps onto the mechanical lock:
    //      steering = +1  ->  +maxSteeringAngle  (full right)
    //
    // The actual road wheel angle then chases that target at a finite rate.
    // Two different rates are used, which matches how a real car behaves:
    //   * turning INTO a corner is limited by how fast the driver can turn
    //     the wheel (steeringRate)
    //   * returning to centre is faster, because caster trail and the
    //     driver's grip both help it back (steeringReturnRate)
    const double targetSteer = m_requested.steering * m_params.maxSteeringAngle;

    const bool returningToCentre = std::abs(targetSteer) < std::abs(m_applied.steering)
                                   && std::abs(targetSteer) < 1e-6;

    const double slewRate = returningToCentre ? m_controls.steeringReturnRate
                                              : m_controls.steeringRate;

    m_applied.steering = moveToward(m_applied.steering, targetSteer, slewRate, dt);
    m_applied.steering = clampAbs(m_applied.steering, m_params.maxSteeringAngle);

    //-- throttle and brake -----------------------------------------------
    //
    // Ramped rather than instantaneous, so a keypress produces a smooth pedal
    // application. Emergency brake bypasses both ramps: that is the point.
    if (m_emergencyBrake) {
        m_applied.throttle = 0.0;
        m_applied.brake    = 1.0;
    } else {
        m_applied.throttle =
            moveToward(m_applied.throttle, m_requested.throttle, m_controls.throttleRate, dt);
        m_applied.brake =
            moveToward(m_applied.brake, m_requested.brake, m_controls.brakeRate, dt);

        m_applied.throttle = clampd(m_applied.throttle, 0.0, 1.0);
        m_applied.brake    = clampd(m_applied.brake, 0.0, 1.0);
    }

    //-- clutch -----------------------------------------------------------
    //
    // Ramped like the other pedals, for the same reason: the keyboard gives
    // a binary key and a clutch that snaps between fully in and fully out
    // makes a manual launch impossible to modulate. Not touched by the
    // emergency brake - dipping the clutch is not part of that.
    m_applied.clutch =
        moveToward(m_applied.clutch, m_requested.clutch, m_controls.clutchRate, dt);
    m_applied.clutch = clampd(m_applied.clutch, 0.0, 1.0);

    // Edge-triggered, so it is passed straight through rather than ramped.
    m_applied.shiftRequest = m_requested.shiftRequest;
}

//---------------------------------------------------------------------------
//  Step
//---------------------------------------------------------------------------

void Vehicle::step(const ControlInput& rawInput, double dt) {
    if (!(dt > 0.0) || !isFinite(dt)) return;

    updateActuators(rawInput, dt);

    //-----------------------------------------------------------------------
    //  Normalised input  ->  physical acceleration demand.
    //
    //  Three groups of longitudinal force act on the vehicle:
    //
    //    DRIVE       throttle x the engine's acceleration capability. Always
    //                acts forward (there is no reverse gear in this model).
    //
    //    BRAKE       brake x the braking capability. Always OPPOSES the
    //                current direction of travel - without the sign flip,
    //                holding the brake in reverse would accelerate the
    //                vehicle backwards indefinitely.
    //
    //    ROAD LOAD   rolling resistance, aerodynamic drag, engine braking and
    //                grade. Imposed by the environment and the vehicle's own
    //                speed rather than by the driver. See physics/RoadLoad.h.
    //                This is what makes a coasting vehicle slow down.
    //
    //  Brake and road load are both resistive, so they are summed into one
    //  opposing magnitude and clamped ONCE. Clamping them separately would let
    //  their sum still overshoot through zero and reverse the vehicle.
    //-----------------------------------------------------------------------
    const double v0 = m_state.velocity;

    //-----------------------------------------------------------------------
    //  Traction control: the throttle the ENGINE sees.
    //
    //  The error is how far the worst driven wheel's slip ratio is past the
    //  level's target, and the cut is a PI controller on it: the
    //  proportional term catches a wheel as it lets go, the integral term
    //  holds the cut that keeps it at the target, so the slip settles ON the
    //  target rather than a band above it. The gains are tuned by hand
    //  against tools/lap_test --launch, not taken from a source: firm enough
    //  to catch a 500 hp car's wheels within a few tenths of a second,
    //  gentle enough not to bog the engine. Targets are multiples of the
    //  tyre's own peak-slip parameter, so "at the peak" stays at the peak
    //  whatever tyre the vehicle file describes.
    //
    //  Slip comes from the previous step's wheel state. That is one physics
    //  step (10 ms by default) of delay, well inside what a real system
    //  manages from wheel-speed sensors.
    //-----------------------------------------------------------------------
    double throttle = m_applied.throttle;
    if (m_tcLevel > 0 && m_state.wheelDataValid) {
        static constexpr double kTargetOfPeak[kTractionControlLevels] = {1.35, 0.90, 0.55};
        const double target = kTargetOfPeak[m_tcLevel - 1] * m_params.tire.peakSlipRatio;

        // Driving slip of the worst driven wheel: how much faster its tread
        // is moving than the road, as a fraction of road speed. Below
        // kLaunchSpeed the road speed in the denominator is held at that
        // value: slip RATIO is singular at a standstill (any wheel motion at
        // all is infinite slip), and a controller fed that would close the
        // throttle completely and never let the car leave the line.
        constexpr double kLaunchSpeed = 5.0;   // [m/s]
        const double reference = std::max(std::abs(v0), kLaunchSpeed);
        double slip = 0.0;
        for (int i = 0; i < WheelCount; ++i) {
            if (!m_params.isDrivenWheel(i)) continue;
            const WheelDynamics& w = m_state.wheels[i];
            slip = std::max(slip, (w.effectiveRollingRadius * w.angularVelocity - v0) / reference);
        }
        const double error = slip - target;

        constexpr double kProportional = 3.5;    // full cut at ~0.3 of slip over target
        constexpr double kIntegral     = 14.0;   // [1/s]
        // The integral only builds while the driver is asking for drive, and
        // bleeds away on its own once the wheels have hooked up.
        m_tcIntegral += kIntegral * error * dt;
        if (throttle < 0.02) m_tcIntegral = 0.0;
        m_tcIntegral = clampd(m_tcIntegral, 0.0, 0.90);

        m_tcCut = clampd(kProportional * error + m_tcIntegral, 0.0, 0.95);
        throttle *= (1.0 - m_tcCut);
    } else {
        m_tcCut = 0.0;
        m_tcIntegral = 0.0;
    }

    double driveAccel      = throttle * m_params.maxAcceleration;
    const double brakeMag  = m_applied.brake * m_params.maxBraking;

    m_roadLoad = computeRoadLoad(m_params, m_environment, v0, throttle);

    if (m_powertrain) {
        // Real engine+gearbox model (physics/Powertrain.h) replaces both the
        // simple throttle*maxAcceleration demand above AND RoadLoad's
        // coefficient-based engine-braking stand-in - see RoadLoad.cpp's own
        // comment on engineBrakingCoeff: "Replace this the moment a real
        // engine map and gearbox exist." Rolling resistance and aerodynamic
        // drag from computeRoadLoad() above are unaffected and still used.
        const PowertrainOutput pt = m_powertrain->update(
            throttle, v0, dt, m_applied.clutch, m_requested.shiftRequest);
        driveAccel = pt.driveAccelerationMps2;
        m_roadLoad.engineBraking = pt.engineBrakingMps2;
    }

    const double resistMag = m_roadLoad.resistiveTotal();   // >= 0
    const double opposeMag = brakeMag + resistMag;          // >= 0

    // Grade is signed and acts along the world, not against motion: gravity
    // pulls a vehicle downhill whichever way it is pointing.
    const double gradeAccel = -m_roadLoad.grade;

    double netAccel = driveAccel + gradeAccel;

    constexpr double kMovingThreshold = 1e-4;   // [m/s]

    if (v0 > kMovingThreshold) {
        netAccel -= opposeMag;
        // Never let one step carry the speed past zero into reverse.
        const double aToStop = -v0 / dt;
        if (netAccel < aToStop) netAccel = aToStop;

    } else if (v0 < -kMovingThreshold) {
        netAccel += opposeMag;
        const double aToStop = -v0 / dt;          // positive
        if (netAccel > aToStop) netAccel = aToStop;

    } else {
        //  Essentially stationary. Resistance must not push a parked vehicle,
        //  so it only acts to cancel the drive force, never to reverse it.
        //  This is a crude stand-in for static rolling resistance - the drive
        //  force has to exceed it before the vehicle creeps.
        const double forward = driveAccel + gradeAccel;
        if (forward > resistMag)       netAccel = forward - resistMag;
        else if (forward < -resistMag) netAccel = forward + resistMag;
        else                           netAccel = 0.0;
    }

    DriveCommand cmd;
    cmd.longitudinalAcceleration = netAccel;
    cmd.steerAngle               = m_applied.steering;

    cmd.driveAcceleration = driveAccel;
    cmd.brakeAcceleration = brakeMag;
    cmd.rollingResistance = m_roadLoad.rollingResistance;
    cmd.aerodynamicDrag   = m_roadLoad.aerodynamicDrag;
    cmd.engineBraking     = m_roadLoad.engineBraking;
    cmd.gradeAcceleration = m_roadLoad.grade;

    // The coefficient itself, not just its lumped effect: a per-wheel model
    // applies f_r against the load each contact patch is carrying.
    cmd.rollingResistanceCoeff = m_roadLoad.effectiveFr;

    m_dynamics->update(m_state, m_params, cmd, dt, m_terrain);

    updateWheelPoses(dt);
}

void Vehicle::setTractionControl(int level) {
    m_tcLevel = std::min(std::max(level, 0), kTractionControlLevels);
    m_tcCut = 0.0;
    m_tcIntegral = 0.0;
}

void Vehicle::setTireCompound(TireCompound compound) {
    m_params.tireCompound = compound;
    for (WheelDynamics& w : m_state.wheels) w.wear = 0.0;
}

void Vehicle::reset(const InitialStateConfig& initial) {
    m_state = VehicleState{};
    m_state.position = initial.position;
    m_state.yaw      = coords::wrapAngle(initial.yaw);
    m_state.velocity = clampd(initial.velocity,
                              -0.35 * m_params.maxSpeed, m_params.maxSpeed);
    m_state.time = 0.0;

    if (m_powertrain) m_powertrain->reset();

    m_requested = ControlInput{};
    m_applied   = ControlInput{};
    m_emergencyBrake = false;
    m_tcCut = 0.0;
    m_tcIntegral = 0.0;
    m_wheelSpinPerWheel.fill(0.0);

    if (m_dynamics) m_dynamics->reset();
    updateWheelPoses(0.0);
}

//---------------------------------------------------------------------------
//  Derived quantities
//---------------------------------------------------------------------------

glm::dvec3 Vehicle::cgPosition() const {
    // The state position is the REAR AXLE, on the flat ground-plane the
    // physics actually simulates (KB gap: no roll/pitch/suspension model -
    // see the dynamics model's own description). terrainHeave() carries the
    // REAL absolute elevation whenever a terrain field is active (it is 0 on
    // every flat/procedural track, so this is a no-op there) - without it,
    // every visual anchor derived from this function stays pinned near the
    // vehicle's SPAWN elevation while the car itself (drawn via Renderer's
    // own heavePos) correctly climbs and descends with the terrain, which is
    // exactly what made the camera "float" above/away from the visible car
    // and track on a real elevation-varying scan (see README, Open Questions).
    //
    // The CG sits rearAxleToCg metres forward of the state position, at
    // cgHeight above the (locally levelled) ground.
    return m_state.position
         + m_state.forward() * m_params.rearAxleToCg()
         + coords::kWorldUp * (m_params.cgHeight + terrainHeave());
}

glm::dvec3 Vehicle::frontAxlePosition() const {
    return m_state.position + m_state.forward() * m_params.wheelbase
         + coords::kWorldUp * terrainHeave();
}

glm::dvec3 Vehicle::bodyCentrePosition() const {
    return m_state.position + m_state.forward() * m_params.bodyCentreOffset()
         + coords::kWorldUp * terrainHeave();
}

double Vehicle::turningRadius() const {
    const double t = std::tan(m_state.steeringAngle);
    if (std::abs(t) < 1e-6) return std::numeric_limits<double>::infinity();
    return m_params.wheelbase / t;   // signed: positive = turning right
}

double Vehicle::pathRadius() const {
    // Gillespie Eq. 6-71:  R = V / r.
    const double r = m_state.yawRate;
    if (std::abs(r) < 1e-6) return std::numeric_limits<double>::infinity();
    return m_state.speed() / r;      // signed: positive = turning right
}

double Vehicle::lateralAcceleration() const {
    if (m_state.wheelDataValid) {
        // The force balance actually solved this step.
        return m_state.lateralAcceleration;
    }
    // Kinematic fallback: a_y = v^2 / R, carrying the sign of R so left/right
    // is distinguishable. No tyre was asked whether it could produce this.
    const double R = turningRadius();
    if (!std::isfinite(R) || std::abs(R) < 1e-6) return 0.0;
    return (m_state.velocity * m_state.velocity) / R;
}

double Vehicle::understeerRatio() const {
    //  KB Eq. 7.5:   omega_geom = V * delta_avg / L     (Ackermann kinematics)
    //  KB Eq. 7.16:  US = omega_geom / omega
    const double r = m_state.yawRate;
    if (std::abs(r) < 1e-4) return 1.0;

    const double omegaGeom =
        m_state.velocity * m_state.steeringAngle / m_params.wheelbase;
    if (std::abs(omegaGeom) < 1e-9) return 1.0;

    return omegaGeom / r;
}

double Vehicle::understeerGradientTires() const {
    //  Gillespie Eq. 6-15:  K_tires = W_f/C_af - W_r/C_ar,  with the AXLE
    //  load and the AXLE cornering stiffness (both tyres) at each end. Both
    //  are read from the current tyre state, so the number moves with load
    //  transfer rather than being a static property of the configuration.
    if (!m_state.wheelDataValid) return 0.0;

    const TireModel tire(m_params.tire);

    double Wf = 0.0, Wr = 0.0, Caf = 0.0, Car = 0.0;
    for (int i = 0; i < WheelCount; ++i) {
        const double Fz = m_state.wheels[i].verticalLoad;
        const double Ca = tire.corneringStiffnessAt(Fz, m_params.staticWheelLoad(i));
        if (i == FrontLeft || i == FrontRight) { Wf += Fz; Caf += Ca; }
        else                                   { Wr += Fz; Car += Ca; }
    }

    if (!(Caf > 1e-6) || !(Car > 1e-6)) return 0.0;
    return Wf / Caf - Wr / Car;      // [rad per g]
}

//---------------------------------------------------------------------------
//  Terrain-derived body attitude
//---------------------------------------------------------------------------

double Vehicle::terrainHeave() const {
    if (!m_state.wheelDataValid) return 0.0;
    double sum = 0.0;
    for (int i = 0; i < WheelCount; ++i) sum += m_state.wheels[i].chassisHeight;
    return sum / WheelCount;
}

double Vehicle::terrainPitch() const {
    if (!m_state.wheelDataValid) return 0.0;
    const double frontAvg = 0.5 * (m_state.wheels[FrontLeft].chassisHeight +
                                   m_state.wheels[FrontRight].chassisHeight);
    const double rearAvg  = 0.5 * (m_state.wheels[RearLeft].chassisHeight +
                                   m_state.wheels[RearRight].chassisHeight);
    if (!(m_params.wheelbase > 1e-6)) return 0.0;
    // Front higher than rear -> nose tilts up -> positive, matching
    // modelMatrixYPR's convention.
    return std::atan2(frontAvg - rearAvg, m_params.wheelbase);
}

double Vehicle::terrainRoll() const {
    if (!m_state.wheelDataValid) return 0.0;
    const double leftAvg  = 0.5 * (m_state.wheels[FrontLeft].chassisHeight +
                                   m_state.wheels[RearLeft].chassisHeight);
    const double rightAvg = 0.5 * (m_state.wheels[FrontRight].chassisHeight +
                                   m_state.wheels[RearRight].chassisHeight);
    const double track = 0.5 * (m_params.frontTrack + m_params.rearTrack);
    if (!(track > 1e-6)) return 0.0;
    // Left higher than right -> right side lower -> leaning right ->
    // positive, matching VehicleState::rollAngle's convention.
    return std::atan2(leftAvg - rightAvg, track);
}

//---------------------------------------------------------------------------
//  Wheel geometry
//---------------------------------------------------------------------------

void Vehicle::updateWheelPoses(double dt) {
    const double L  = m_params.wheelbase;
    const double tf = m_params.frontTrack;
    const double tr = m_params.rearTrack;
    const double r  = m_params.wheelRadius;

    //-----------------------------------------------------------------------
    //  ROLLING ANIMATION
    //
    //  Two cases, and the difference is worth seeing on screen.
    //
    //  When the active model integrates a wheel speed of its own, that speed
    //  is what the wheel is drawn spinning at - so a LOCKED wheel visibly
    //  stops while the car is still moving, and a SPINNING wheel visibly races
    //  ahead of the road speed. Those are the phenomena of KB Section 2.4.
    //
    //  When it does not (the kinematic model has no wheel dynamics), the wheel
    //  is animated at the free-rolling rate omega = v / r, as before.
    //-----------------------------------------------------------------------
    std::array<double, WheelCount> omega{};
    if (m_state.wheelDataValid) {
        for (int i = 0; i < WheelCount; ++i) {
            omega[i] = m_state.wheels[i].angularVelocity;
        }
    } else {
        const double w0 = (r > 1e-6) ? m_state.velocity / r : 0.0;
        omega.fill(w0);
    }

    if (dt > 0.0) {
        for (int i = 0; i < WheelCount; ++i) {
            m_wheelSpinPerWheel[i] =
                coords::wrapAngle(m_wheelSpinPerWheel[i] + omega[i] * dt);
        }
    }

    //-----------------------------------------------------------------------
    //  ACKERMAN STEERING GEOMETRY
    //
    //  The commanded steer angle is a single angle for a virtual wheel on the
    //  centreline. The two real front wheels sit half a track either side of
    //  it and therefore follow different radii, so they need different angles
    //  (Gillespie Eq. 8-1/8-2). The geometry lives in physics/SteeringGeometry
    //  so that the renderer and the physics cannot drift apart - the four-wheel
    //  model steers its tyres with exactly this function.
    //-----------------------------------------------------------------------
    AckermanAngles steer;
    if (m_state.wheelDataValid) {
        // Prefer what the physics actually used.
        steer.left  = m_state.wheels[FrontLeft].steerAngle;
        steer.right = m_state.wheels[FrontRight].steerAngle;
    } else {
        steer = ackermanSteerAngles(L, tf, m_state.steeringAngle);
    }

    //-----------------------------------------------------------------------
    //  TERRAIN BOB
    //
    //  Each wheel's own vertical position relative to the body, from the
    //  terrain quarter car's UNSPRUNG (wheelHeight) state - independent per
    //  wheel, unlike the body's own heave/pitch/roll (see terrainHeave() /
    //  terrainPitch() / terrainRoll(), and the header comment on why a rigid
    //  body cannot show this independence itself). Relative to the body's
    //  own heave, so this is suspension travel, not an absolute height -
    //  the body transform (Renderer::drawVehicle) already carries the heave.
    //  Zero on flat ground / without wheel data, by the same reasoning as
    //  everywhere else this state is used.
    //-----------------------------------------------------------------------
    const double heave = terrainHeave();
    std::array<double, WheelCount> bob{};
    if (m_state.wheelDataValid) {
        for (int i = 0; i < WheelCount; ++i) {
            bob[i] = m_state.wheels[i].wheelHeight - heave;
        }
    }

    //  Vehicle local frame: +X forward, +Y up, +Z right.
    //  Rear axle is the origin, so the front axle is at x = L.
    auto& w = m_wheels;

    w[FrontLeft]  = {{L,   r + bob[FrontLeft],  -tf * 0.5}, steer.left,  m_wheelSpinPerWheel[FrontLeft],  true};
    w[FrontRight] = {{L,   r + bob[FrontRight],  tf * 0.5}, steer.right, m_wheelSpinPerWheel[FrontRight], true};
    w[RearLeft]   = {{0.0, r + bob[RearLeft],   -tr * 0.5}, 0.0,         m_wheelSpinPerWheel[RearLeft],   false};
    w[RearRight]  = {{0.0, r + bob[RearRight],   tr * 0.5}, 0.0,         m_wheelSpinPerWheel[RearRight],  false};
}

} // namespace vsim
