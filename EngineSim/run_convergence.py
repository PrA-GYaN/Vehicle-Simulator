"""Timestep convergence verification.

Run the same engine experiment at multiple timesteps and compare results.
This verifies numerical stability and convergence.

Source: ENGINE_KB.md §28 (Timestep verification).
"""

from __future__ import annotations

import sys
import os
import math

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from engine_sim.config import EngineConfig, TorquePoint
from engine_sim.engine import Engine, EngineInput
from engine_sim.telemetry import TelemetryRecorder


def run_convergence_test():
    """Run free-rev experiment at multiple timesteps and compare."""
    timesteps = [0.010, 0.005, 0.001, 0.0005]
    duration = 3.0
    results = {}

    for dt in timesteps:
        config = EngineConfig(
            torque_curve=[
                TorquePoint(1000, 100.0),
                TorquePoint(2000, 155.0),
                TorquePoint(3000, 180.0),
                TorquePoint(4000, 182.0),
                TorquePoint(5000, 165.0),
                TorquePoint(6000, 138.0),
                TorquePoint(7000, 100.0),
            ],
        )
        engine = Engine(config)
        engine.start()

        recorder = TelemetryRecorder()
        t = 0.0
        n_steps = int(duration / dt)
        for _ in range(n_steps):
            out = engine.update(EngineInput(throttle=1.0, load_torque=0.0), dt)
            recorder.record(t, out)
            t += dt

        final = engine.last_output
        results[dt] = {
            "final_rpm": final.rpm,
            "final_omega": final.angular_velocity,
            "final_power": final.power,
            "max_torque": max(e.engine_torque for e in recorder.entries),
            "max_power": max(e.engine_power for e in recorder.entries),
            "steps": n_steps,
        }

    # Print comparison table
    print("\n=== Timestep Convergence Test ===")
    print(f"{'dt (s)':>10} {'Steps':>8} {'Final RPM':>12} {'Max Power (kW)':>16} {'Final ω (rad/s)':>18}")
    print("-" * 70)

    base = results[timesteps[0]]
    for dt in timesteps:
        r = results[dt]
        rpm_diff = abs(r["final_rpm"] - base["final_rpm"])
        power_diff = abs(r["max_power"] - base["max_power"])
        print(f"{dt:10.4f} {r['steps']:8d} {r['final_rpm']:12.1f} "
              f"{r['max_power']/1000:16.2f} {r['final_omega']:18.4f}")

    print("\nConvergence analysis (relative to coarsest timestep):")
    for dt in timesteps[1:]:
        r = results[dt]
        rpm_err = abs(r["final_rpm"] - base["final_rpm"]) / base["final_rpm"] * 100
        power_err = abs(r["max_power"] - base["max_power"]) / base["max_power"] * 100
        print(f"  dt={dt:.4f}: RPM error={rpm_err:.3f}%, Power error={power_err:.3f}%")


if __name__ == "__main__":
    run_convergence_test()
