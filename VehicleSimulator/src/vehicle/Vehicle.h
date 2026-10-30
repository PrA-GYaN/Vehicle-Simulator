#pragma once
//===========================================================================
//  Vehicle.h  --  owns state + parameters + a dynamics model.
//
//  RESPONSIBILITIES
//    * hold the VehicleState and VehicleParameters
//    * translate normalised ControlInput into a physical DriveCommand
//    * apply actuator rate limits (steering slew, throttle/brake ramp)
//    * expose derived quantities (CG position, wheel poses, turn radius)
//
//  It does NOT integrate anything itself - that is the dynamics model's job.
//  It does NOT know about time stepping - that is the Simulation's job.
//  It does NOT know about rendering at all.
//===========================================================================

#include <array>
#include <memory>

#include "config/Config.h"
#include "config/VehicleParameters.h"
#include "core/VehicleState.h"
#include "environment/TerrainHeightField.h"
#include "physics/IVehicleDynamics.h"
#include "physics/Powertrain.h"
#include "physics/RoadLoad.h"

namespace vsim {

/// Where a wheel is and how it is turned, expressed in the VEHICLE LOCAL
/// frame (+X forward, +Y up, +Z right). The renderer consumes this; the
/// physics does not. Producing it here keeps all geometry in one place.
struct WheelPose {
    glm::dvec3 localPosition{0.0};  ///< hub centre, vehicle local frame [m]
    double     steerAngle = 0.0;    ///< about +Y, positive = right [rad]
    double     spinAngle  = 0.0;    ///< rolling rotation about the hub [rad]
    bool       steered    = false;  ///< front wheels steer, rear do not
};

// WheelIndex (FrontLeft, FrontRight, RearLeft, RearRight, WheelCount) now
// lives in core/WheelState.h, next to the per-wheel physics state it indexes,
// so the physics and the renderer cannot disagree about which wheel is which.

class Vehicle {
public:
    Vehicle(const VehicleParameters& params,
            const ControlsConfig& controls,
            std::unique_ptr<IVehicleDynamics> dynamics);

    //-- stepping -------------------------------------------------------

    /// Advance by exactly one physics timestep.
    ///
    /// `rawInput` is the driver's *request*. It is rate-limited into the
    /// actuator state before being converted to a physical command, which is
    /// why steering does not snap to full lock the instant a key is pressed.
    void step(const ControlInput& rawInput, double dt);

    /// Restore the configured initial state and clear actuator state.
    void reset(const InitialStateConfig& initial);

    //-- accessors ------------------------------------------------------

    const VehicleState&      state()      const { return m_state; }
    VehicleState&            mutableState()     { return m_state; }
    const VehicleParameters& parameters() const { return m_params; }
    VehicleParameters&       mutableParameters() { return m_params; }
    const ControlsConfig&    controls()   const { return m_controls; }
    ControlsConfig&          mutableControls()  { return m_controls; }

    /// Actuator state after rate limiting - what the vehicle is *actually*
    /// doing, as opposed to what the driver asked for.
    const ControlInput& appliedInput() const { return m_applied; }

    //-- traction control ---------------------------------------------------
    //
    // A driver aid: it watches the driven wheels' slip ratio and closes the
    // throttle when they spin past a target. Level 0 is off; 1 lets the
    // tyres run a little past the peak of their mu-slip curve before it
    // steps in, 2 holds them at about the peak, 3 keeps them well short of
    // it. It acts on the THROTTLE - the same place a real system acts, by
    // cutting engine torque - so it works identically with the simple drive
    // model and with the engine + gearbox. It needs wheel speeds, so it does
    // nothing under the kinematic model, which has none.
    //
    // It does not brake individual wheels, and it does not look at yaw or
    // slip angle: it is traction control, not stability control.
    static constexpr int kTractionControlLevels = 3;
    void setTractionControl(int level);
    int  tractionControl() const { return m_tcLevel; }
    /// Fraction of the driver's throttle currently being withheld, 0..1.
    double tractionControlCut() const { return m_tcCut; }

    //-- tyres ----------------------------------------------------------------

    /// Fit a set of tyres of the given compound. They go on NEW: wear is
    /// reset on all four wheels, including when the compound is unchanged.
    void setTireCompound(TireCompound compound);
    TireCompound tireCompound() const { return m_params.tireCompound; }
    const ControlInput& requestedInput() const { return m_requested; }

    const IVehicleDynamics& dynamics() const { return *m_dynamics; }
    void setDynamics(std::unique_ptr<IVehicleDynamics> d);

    /// Road load acting right now, broken into components. Populated every
    /// step; read by the debug UI to show which resistance term dominates.
    const RoadLoad& roadLoad() const { return m_roadLoad; }

    /// Detailed engine+gearbox model, or null when this vehicle uses the
    /// simple `max_acceleration` demand instead (see
    /// VehicleParameters::hasDetailedEngine, physics/Powertrain.h). Null is
    /// the common case - check before dereferencing, same convention as
    /// terrain() above.
    const Powertrain* powertrain() const { return m_powertrain.get(); }

    /// Push this vehicle's configured shift/clutch modes into the live
    /// powertrain. Call after changing parameters().gearbox at runtime -
    /// the powertrain keeps its own copy, so it does not see the change.
    void syncPowertrainModes();

    /// Environmental conditions used for the road load calculation.
    const EnvironmentConditions& environment() const { return m_environment; }
    EnvironmentConditions& mutableEnvironment() { return m_environment; }
    void setEnvironment(const EnvironmentConditions& e) { m_environment = e; }

    /// Ground height field for the active environment, non-owning. Null (the
    /// default) means flat ground - see IVehicleDynamics::update(). Mirrors
    /// setEnvironment() above.
    const TerrainHeightField* terrain() const { return m_terrain; }
    void setTerrain(const TerrainHeightField* t) { m_terrain = t; }

    /// Emergency brake overrides throttle and applies full braking.
    void setEmergencyBrake(bool on) { m_emergencyBrake = on; }
    bool emergencyBrake() const { return m_emergencyBrake; }

    //-- derived quantities ---------------------------------------------

    /// World position of the centre of gravity, projected to the ground plane
    /// plus the CG height. Derived from the rear-axle reference point.
    glm::dvec3 cgPosition() const;

    /// World position of the front axle centre.
    glm::dvec3 frontAxlePosition() const;

    /// Body centre used for placing the visual mesh.
    glm::dvec3 bodyCentrePosition() const;

    /// GEOMETRIC (Ackerman) turn radius about the rear axle [m].
    ///
    ///     R = L / tan(delta)                    (Gillespie Eq. 6-3, 8-1/8-2)
    ///
    /// This is where the steering geometry alone says the vehicle would go.
    /// Under a force-based model the actual path differs - that difference IS
    /// understeer or oversteer. Compare with pathRadius().
    /// Returns infinity when travelling straight. Callers must check.
    double turningRadius() const;

    /// ACTUAL path radius [m], from the vehicle's own motion: R = V / r.
    ///
    /// Gillespie Eq. 6-71 (`R = V^2/a_y = V/r`). Under the kinematic model
    /// this is identical to turningRadius() by construction; under a dynamic
    /// model it is not, and the ratio of the two is a direct reading of
    /// understeer - see the US ratio, KB Eq. 7.16.
    /// Returns infinity when travelling straight.
    double pathRadius() const;

    /// Lateral acceleration a_y [m/s^2], positive to the vehicle's right.
    ///
    /// When the active model solves a force balance, this is the real thing -
    /// the y component of sum(F)/m. When it does not (the kinematic model),
    /// it falls back to the KINEMATIC consequence of the steering geometry,
    /// v^2/R, which no tyre was ever asked to produce. The UI shows which.
    double lateralAcceleration() const;

    /// Understeer ratio US = omega_geometric / omega_actual (KB Eq. 7.16).
    ///
    ///     > 1  understeer      = 1  neutral steer      < 1  oversteer
    ///
    /// KB Section 9.4.14 is emphatic that this is NOT Gillespie's understeer
    /// gradient K (deg/g): both are called "understeer" and both separate the
    /// same three behaviours, but US is a dimensionless ratio at one operating
    /// point and K cannot be read off it.
    /// Returns 1.0 (neutral) when there is no meaningful yaw rate.
    double understeerRatio() const;

    /// Tyre-and-weight-distribution term of the understeer gradient [rad/g],
    ///
    ///     K_tires = W_f/C_af - W_r/C_ar               (Gillespie Eq. 6-15)
    ///
    /// evaluated at the CURRENT wheel loads and cornering stiffnesses, so it
    /// moves with load transfer. This is term 1 of the seven in the KB's
    /// understeer budget (Section 11) - the only one this project can compute,
    /// because the other six need suspension and steering-system parameters no
    /// source quantifies. Returns 0 when the active model has no tyres.
    double understeerGradientTires() const;

    /// Poses of all four wheels in the vehicle local frame.
    const std::array<WheelPose, WheelCount>& wheelPoses() const { return m_wheels; }

    //-- terrain-derived body attitude (rendering only) ------------------
    //
    // Rigid-body heave/pitch/roll fitted from the 4 wheels' own
    // `chassisHeight` (the SPRUNG mass corner of the terrain quarter car -
    // see WheelState.h / FourWheelModel.cpp step 1b). A rigid body has only
    // these 3 vertical/rotational degrees of freedom, so any 4th
    // ("diagonal warp") component of the 4 corner heights is discarded here
    // by construction - the wheels themselves, independently suspended,
    // still show it via their own wheelHeight (see updateWheelPoses()).
    // Zero whenever there is no per-wheel state to derive it from
    // (wheelDataValid false, e.g. the kinematic model) or on flat ground,
    // where every chassisHeight is exactly zero - see WheelState.h.

    /// Average corner sprung-mass height [m] - the body's overall ride
    /// height offset due to terrain.
    double terrainHeave() const;

    /// Nose-up pitch [rad] from the front/rear chassisHeight halves -
    /// see core/Coordinates.h::modelMatrixYPR for the sign convention.
    double terrainPitch() const;

    /// Roll [rad] from the left/right chassisHeight halves, in the SAME
    /// sign convention as VehicleState::rollAngle (positive = leaning
    /// right), so the two are directly additive (superposition, licensed
    /// the same way as the Fz terrain delta - see FourWheelModel.cpp).
    double terrainRoll() const;

private:
    void updateActuators(const ControlInput& request, double dt);
    void updateWheelPoses(double dt);

    VehicleParameters m_params;
    ControlsConfig    m_controls;
    VehicleState      m_state;

    std::unique_ptr<IVehicleDynamics> m_dynamics;

    /// Constructed only when m_params.hasDetailedEngine() - see
    /// physics/Powertrain.h. Null otherwise, which is the common case.
    std::unique_ptr<Powertrain> m_powertrain;

    EnvironmentConditions m_environment;
    RoadLoad              m_roadLoad;
    const TerrainHeightField* m_terrain = nullptr;

    ControlInput m_requested;   ///< raw driver request this step
    ControlInput m_applied;     ///< after rate limiting

    int    m_tcLevel = 0;
    double m_tcCut = 0.0;       ///< throttle fraction withheld right now
    double m_tcIntegral = 0.0;  ///< integral term of the slip controller
    bool         m_emergencyBrake = false;

    std::array<WheelPose, WheelCount> m_wheels{};

    /// Accumulated rolling angle per wheel [rad]. Per wheel rather than one
    /// shared value, because under a model with wheel dynamics the four wheels
    /// genuinely rotate at different rates - that is what lock and spin are.
    std::array<double, WheelCount> m_wheelSpinPerWheel{};
};

} // namespace vsim
