#pragma once
//===========================================================================
//  Engine.h  --  the engine torque map that closes KB gap G17.
//
//  TWO MODELS BEHIND ONE INTERFACE. Which one a vehicle gets is decided by
//  its config (EngineParameters::useResolved, i.e. a `resolved:` block):
//
//    * CYCLE-AVERAGED (the default, everything below this paragraph): one
//      air-standard Otto cycle evaluated per step. Stateless apart from
//      engine speed and an authored boost lag; cheap enough to probe 64
//      times a frame for a torque curve.
//    * RESOLVED (physics/ResolvedEngine.h): EngineSim v0.7's crank-angle-
//      resolved cylinders. This class then only adapts it to the interface
//      Powertrain already drives - it sub-steps the engine at its own finer
//      time step, engages the starter if the engine has stalled, and fills
//      the same TorqueBreakdown. Read ResolvedEngine.h for that model; the
//      scope notes below describe the cycle-averaged one only.
//
//  SOURCE: ported from ../EngineSim (Python), which implements Heywood,
//  "Internal Combustion Engine Fundamentals" (McGraw-Hill, 1988) — see
//  ../EngineSim/ENGINE_KB.md and ../EngineSim/KB_GAPS.md for the equation-
//  by-equation traceability. This is NOT part of this project's S1/S2/S3
//  vehicle-dynamics Knowledge Base; see config/EngineParameters.h for the
//  full source notice.
//
//  SCOPE OF THE PORT — deliberately narrower than EngineSim's current
//  Python version:
//    * air-standard Otto cycle, cycle-averaged indicated torque (Heywood
//      Eq 2.21), lumped rubbing-friction/pumping/accessory losses (Heywood
//      Eq 13.5/13.6/§13.7), idle PI governor, hysteresis rev limiter.
//    * uses a CONSTANT specific heat (CV_AIR), not EngineSim's temperature-
//      dependent gas_properties.py — that is the "v0.1/v0.2 core", which is
//      the validated part; EngineSim's v0.3 additions (turbo, knock, fuel
//      film, heat transfer, manifold dynamics, diesel, emissions, cyclic
//      variability, Bishop friction breakdown) are NOT ported here. If a
//      vehicle ever needs those, port them the same way this file was
//      ported, not by re-deriving them from scratch.
//
//  (The paragraph above is the cycle-averaged model's scope. EngineSim's
//  manifold dynamics, heat transfer, knock, valve flow and turbocharger DID
//  later arrive here - not as additions to this model, but whole, as the
//  resolved core.)
//
//  TWO WAYS TO DRIVE THIS ENGINE, both intentional (see Powertrain.h):
//    * evaluateTorque() looks up the torque chain at a GIVEN speed without
//      touching the engine's own state — for when the driveline has locked
//      engine speed to (wheel speed * gear ratio), i.e. the engine has no
//      speed of its own to integrate.
//    * update() integrates the engine's OWN speed under its OWN inertia,
//      with the idle governor and rev limiter active — for when the
//      driveline is decoupled (neutral, or wheel speed implies less than
//      idle) and the engine is free-running.
//===========================================================================

#include <optional>
#include <vector>

#include "config/EngineParameters.h"
#include "physics/ResolvedEngine.h"

namespace vsim {

class Engine {
public:
    explicit Engine(const EngineParameters& params);

    /// One timestep's torque balance, before crankshaft dynamics are applied.
    /// All torques in [N.m], already summed over every cylinder.
    struct TorqueBreakdown {
        double indicatedTorqueNm  = 0.0;  ///< T_ind, Heywood Eq 2.21
        double frictionTorqueNm   = 0.0;  ///< T_fric, Heywood Eq 13.5
        double pumpingTorqueNm    = 0.0;  ///< T_pump
        double accessoryTorqueNm  = 0.0;  ///< T_acc
        double loadTorqueNm       = 0.0;  ///< T_load, as supplied by the caller
        double netTorqueNm        = 0.0;  ///< T_ind - T_fric - T_pump - T_acc - T_load
        /// Torque at the flywheel, before any load: T_ind - T_fric - T_pump -
        /// T_acc. On the resolved engine it is the mean over one firing
        /// period (see Engine.cpp, windowedShaftTorque), and at an imposed
        /// speed netTorqueNm carries the same value.
        double brakeTorqueNm      = 0.0;
        double intakePressurePa   = 0.0;
        double volumetricEfficiency = 0.0;
        double thermalEfficiency  = 0.0;
        double boostPa            = 0.0;  ///< turbo boost used, gauge [Pa]; 0 if NA
        double chargeTemperatureK = 0.0;  ///< intake charge T after compressor + aftercooler
        /// State 4 of the same Otto cycle - end of expansion, which is where
        /// the exhaust valve opens and the blowdown pulse leaves the
        /// cylinder. p_4 = m R T_4 / V_BDC, ideal gas. Read by the sound
        /// synthesis (src/audio/), which makes each exhaust pulse from it;
        /// nothing in the physics uses them.
        double exhaustBlowdownPressurePa = 0.0;  ///< p_4 [Pa], absolute
        double exhaustBlowdownTemperatureK = 0.0; ///< T_4 [K]
        double airMassPerCylinderKg = 0.0;       ///< trapped charge per cycle [kg]
        double fuelMassPerCylinderKg = 0.0;      ///< 0 under fuel cut (DFCO / limiter)
    };

    /// Evaluate the torque chain at `omega` [rad/s] and `throttle` [0,1]
    /// against `loadTorqueNm`, without changing the engine's own state.
    /// See the file banner for when to use this instead of update().
    ///
    /// CYCLE-AVERAGED MODEL ONLY. A resolved engine has no stateless answer
    /// to "what torque at this speed" - that takes running it there until
    /// the manifold, the turbo and the ECU's air model settle, which is what
    /// physics/EngineDyno.h does. Called on a resolved engine this returns
    /// the cycle-averaged formula evaluated on the shared parameters: a
    /// rough estimate, not that engine's curve.
    TorqueBreakdown evaluateTorque(double omega, double throttle, double loadTorqueNm) const;

    /// Advance the engine's own speed by `dt`, under its own inertia, with
    /// the idle governor and rev limiter both active. `loadTorqueNm` is
    /// whatever external load is applied (0 when fully decoupled, e.g.
    /// neutral). Returns the breakdown at the START of the step, i.e. the
    /// torques that produced the acceleration just applied.
    TorqueBreakdown update(double throttle, double loadTorqueNm, double dt);

    /// Torque at an externally imposed speed (the locked driveline, where
    /// omega is wheel speed x ratio, not integrated here), advancing the
    /// turbocharger's boost lag by dt on the way. evaluateTorque() cannot do
    /// this - it is const, and assumes steady-state boost - so without this
    /// the locked path would never spool the turbo at all. Identical to
    /// evaluateTorque() on a naturally aspirated engine.
    TorqueBreakdown evaluateTorqueAt(double omega, double throttle, double dt);

    /// idle=true starts the engine at idle speed; false leaves it stopped.
    void reset(bool idle);

    /// True when the crank-angle-resolved core is driving this engine.
    bool isResolved() const { return m_resolved.has_value(); }
    /// The resolved core, for read-outs the TorqueBreakdown does not carry
    /// (manifold pressure, lambda, spark, turbo speed...). Null on the
    /// cycle-averaged model.
    const ResolvedEngine* resolved() const { return m_resolved ? &*m_resolved : nullptr; }

    /// A figure for the engine's peak wide-open-throttle torque [N.m], for
    /// consumers that only need a scale (the gear-whine level). Cycle-
    /// averaged: probed off the model. Resolved: the highest flywheel torque
    /// seen so far, starting from a bmep-based guess (11 bar naturally
    /// aspirated, scaled by the boost target) - deterministic, and it never
    /// has to run the engine anywhere it is not already running.
    double peakTorqueEstimateNm() const;

    double omega() const { return m_omega; }              ///< [rad/s]
    double rpm() const;                                    ///< derived, [RPM]
    bool   limiterActive() const { return m_limiterActive; }
    double boostPa() const { return m_boostPa; }          ///< current boost, gauge [Pa]
    const  EngineParameters& parameters() const { return m_params; }
    const  TorqueBreakdown& lastBreakdown() const { return m_lastBreakdown; }

private:
    TorqueBreakdown computeBreakdown(double omega, double throttle, double loadTorqueNm,
                                     double boostPa) const;
    /// Steady-state boost the turbo would reach at this speed and throttle.
    double targetBoostPa(double omega, double throttle) const;
    /// First-order lag of m_boostPa toward targetBoostPa() - see
    /// EngineParameters.h for why this stands in for Heywood Eq 14.51.
    void advanceBoost(double omega, double throttle, double dt);
    double applyLimiter(double omegaNew);
    /// One physics step of the resolved core: `imposed` holds the crank at
    /// `omega` (locked driveline), otherwise the engine integrates its own
    /// speed against `loadTorqueNm`.
    TorqueBreakdown stepResolved(double throttle, double loadTorqueNm, double dt,
                                 bool imposed, double omega);

    EngineParameters m_params;
    std::optional<ResolvedEngine> m_resolved;
    /// Flywheel torque of the last sub-steps, newest last - see
    /// windowedShaftTorque() in Engine.cpp.
    struct TorqueSample { double angle; double dt; double torqueNm; };
    std::vector<TorqueSample> m_torqueWindow;
    double windowedShaftTorque(double stepAngle);
    double m_shaftTorqueFiltered = 0.0;   ///< mean flywheel torque over one firing period [N.m]
    double m_peakTorqueSeen = 0.0;        ///< resolved: see peakTorqueEstimateNm()
    double m_omega = 0.0;           ///< [rad/s] — the sole integrated state
    double m_idleIntegral = 0.0;    ///< idle-governor PI integral term
    double m_boostPa = 0.0;         ///< [Pa] gauge - turbo lag state, 0 when NA
    bool   m_limiterWasActive = false;
    bool   m_limiterActive = false;
    TorqueBreakdown m_lastBreakdown;
};

} // namespace vsim
