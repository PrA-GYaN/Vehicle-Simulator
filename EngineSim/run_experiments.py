"""Run all engine experiments and print results.

This is the main experiment runner — an engine-only test/dyno environment
that does not require a complete vehicle.

Source: ENGINE_KB.md §22, §23.
"""

from __future__ import annotations

import sys
import os
import math

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from engine_sim.config import EngineConfig, load_config
from engine_sim.engine import Engine, EngineInput
from engine_sim.experiments import EngineExperiments
from engine_sim.telemetry import TelemetryRecorder
from engine_sim.debug_ui import format_debug_panel, format_torque_balance


def print_experiment_header(name: str, description: str):
    print(f"\n{'='*60}")
    print(f"  EXPERIMENT: {name}")
    print(f"  {description}")
    print(f"{'='*60}")


def run_all_experiments():
    """Run all six required engine experiments."""
    config = load_config(os.path.join("config", "default_engine.yaml"))
    exp = EngineExperiments(config)

    dt = 0.001  # 1 ms timestep

    # === Experiment 1: Free Rev ===
    print_experiment_header("Free Rev", "Full throttle from idle")
    result = exp.free_rev(duration=5.0, dt=dt)
    print(result.result_summary)
    if result.final_output:
        print("\nFinal state:")
        print(format_debug_panel(result.final_output))

    # === Experiment 2: Throttle Step ===
    print_experiment_header("Throttle Step", "0→100% at 3000 RPM")
    result = exp.throttle_step(initial_rpm=3000.0, duration=3.0, dt=dt)
    print(result.result_summary)
    if result.final_output:
        print("\nFinal state:")
        print(format_debug_panel(result.final_output))

    # === Experiment 3: Throttle Release ===
    print_experiment_header("Throttle Release", "100→0% at 5000 RPM")
    result = exp.throttle_release(initial_rpm=5000.0, duration=5.0, dt=dt)
    print(result.result_summary)
    if result.final_output:
        print("\nFinal state:")
        print(format_debug_panel(result.final_output))

    # === Experiment 4: Constant Throttle ===
    print_experiment_header("Constant Throttle", "Steady-state at various throttle positions")
    results = exp.constant_throttle(
        throttle_values=[0.25, 0.5, 0.75, 1.0],
        duration=5.0, dt=dt,
    )
    for r in results:
        print(f"\n  {r.name}: Final RPM={r.final_output.rpm:.0f}, "
              f"Power={r.final_output.power/1000:.1f} kW")

    # === Experiment 5: External Load ===
    print_experiment_header("External Load", "50 N·m load at 50% throttle")
    result = exp.external_load(
        load_torque=50.0, throttle=0.5, duration=5.0, dt=dt,
    )
    print(result.result_summary)
    if result.final_output:
        print("\nFinal state:")
        print(format_debug_panel(result.final_output))
        print()
        print(format_torque_balance(result.final_output))

    # === Experiment 6: RPM Sweep ===
    print_experiment_header("RPM Sweep", "WOT sweep through RPM range")
    result = exp.rpm_sweep(duration=8.0, dt=dt)
    print(result.result_summary)
    if result.final_output:
        print("\nFinal state:")
        print(format_debug_panel(result.final_output))

    # Save telemetry for each experiment
    print("\n" + "=" * 60)
    print("  Saving telemetry files...")
    print("=" * 60)

    os.makedirs("output", exist_ok=True)

    # Re-run and save key experiments
    for name, fn, kwargs in [
        ("free_rev", exp.free_rev, {"duration": 5.0, "dt": dt}),
        ("throttle_step", exp.throttle_step, {"initial_rpm": 3000.0, "duration": 3.0, "dt": dt}),
        ("rpm_sweep", exp.rpm_sweep, {"duration": 8.0, "dt": dt}),
    ]:
        result = fn(**kwargs)
        if result.telemetry:
            path = f"output/{name}_telemetry.csv"
            result.telemetry.save(path)
            print(f"  Saved: {path}")

    print("\nDone.")


if __name__ == "__main__":
    run_all_experiments()
