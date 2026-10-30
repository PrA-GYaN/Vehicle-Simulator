#include "physics/KinematicBicycleModel.h"

#include <cmath>

#include "core/Coordinates.h"

namespace vsim {

void KinematicBicycleModel::update(VehicleState& state,
                                   const VehicleParameters& params,
                                   const DriveCommand& command,
                                   double dt,
                                   const TerrainHeightField* /*terrain*/) {
    // No per-wheel vertical dynamics in this model (see the header's
    // ASSUMPTION 4: "no roll, pitch or suspension") - terrain is ignored.
    //-----------------------------------------------------------------------
    // 0. Guard the inputs.
    //
    // The caller promises dt > 0 and finite, but a bad config or a runtime
    // parameter edit can still produce a nonsense steer angle. Guarding here
    // rather than trusting the caller keeps the model safe to reuse.
    //-----------------------------------------------------------------------
    if (!(dt > 0.0) || !isFinite(dt)) return;

    const double L = params.wheelbase;   // already validated > 0.05 m

    // tan(delta) diverges at +-90 deg. The mechanical lock is well below that
    // (validate() caps it at 80 deg) but a runtime edit could still reach the
    // asymptote, so clamp defensively before calling tan().
    const double delta =
        clampAbs(sanitize(command.steerAngle), params.maxSteeringAngle);

    const double accelCmd = sanitize(command.longitudinalAcceleration);

    //-----------------------------------------------------------------------
    // 1. Integrate speed.        v_dot = a
    //
    // Done FIRST, but the updated speed is deliberately NOT used to propagate
    // position and yaw below. Using the pre-step speed for the kinematics is
    // plain forward Euler and keeps every derivative consistently evaluated at
    // the start of the interval. Mixing the two (semi-implicit) would be a
    // different integrator and would quietly change the trajectory.
    //-----------------------------------------------------------------------
    const double v0 = state.velocity;

    double v = v0 + accelCmd * dt;

    // Speed limits. Two separate concerns:
    //
    //  a) Reverse. The model is symmetric and works fine for v < 0, so
    //     reverse is allowed, but capped well below the forward limit -
    //     no production car reverses at its top speed.
    //  b) Forward. Capped at the configured maximum.
    const double maxReverse = -0.35 * params.maxSpeed;
    v = clampd(v, maxReverse, params.maxSpeed);

    // Snap tiny residual speeds to zero. Without this, a braking vehicle
    // asymptotically approaches zero and creeps forever at 1e-9 m/s, which
    // looks like a bug and pollutes telemetry with meaningless motion.
    if (std::abs(v) < 1e-4) v = 0.0;

    //-----------------------------------------------------------------------
    // 2. Yaw rate.             psi_dot = (v / L) * tan(delta)
    //
    // Evaluated with the START-of-step speed v0, matching forward Euler.
    //
    // Note the v0 factor: yaw rate is proportional to speed. A stationary
    // vehicle with the wheels turned does not rotate - which is correct for
    // this model and correct for a real car that is not being pushed.
    //-----------------------------------------------------------------------
    const double yawRate = (v0 / L) * std::tan(delta);

    //-----------------------------------------------------------------------
    // 3. Integrate pose.
    //
    //      x   += v * cos(psi) * dt
    //      z   += v * sin(psi) * dt
    //      psi += psi_dot * dt
    //
    // The heading vector comes from coords::heading() rather than an inline
    // cos/sin pair, so the convention lives in exactly one place.
    //-----------------------------------------------------------------------
    const glm::dvec3 forward = coords::heading(state.yaw);

    state.position += forward * (v0 * dt);
    state.yaw       = coords::wrapAngle(state.yaw + yawRate * dt);

    //-----------------------------------------------------------------------
    // 4. Commit derived state.
    //-----------------------------------------------------------------------
    state.velocity      = v;
    state.yawRate       = yawRate;
    state.steeringAngle = delta;

    // Report the acceleration actually realised over the step, not the value
    // that was requested. When the speed limiter clips, these differ, and the
    // realised value is what telemetry and the UI should show.
    state.acceleration = (v - v0) / dt;

    state.time += dt;

    //-----------------------------------------------------------------------
    // 5. Final safety net.
    //-----------------------------------------------------------------------
    state.sanitizeAll();
}

} // namespace vsim
