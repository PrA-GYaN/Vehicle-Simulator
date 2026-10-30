"""Real-time benchmark of the engine physics.

    python run_benchmark.py [--engine 454 1nz vr38] [--seconds 1.0] [--python]

For each engine, holds a set of speeds with the dyno (full throttle and closed
throttle) and reports the *real-time factor*: simulated seconds per wall-clock
second at the dashboard's 1 ms physics step. The dashboard needs >= 1.0 at every
point to stay in step with the clock; the table flags anything below 1.5 (not
enough headroom for the browser stream and the OS).
"""

from __future__ import annotations

import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from engine_sim.engine_file import load_engine_file, resolve_engine   # noqa: E402
from engine_sim.native_core import native_available                    # noqa: E402
from engine_sim.powertrain import Powertrain                           # noqa: E402

DT = 0.001
_dt = DT


def measure(le, rpm: float, throttle: float, seconds: float, dt: float = DT) -> float:
    pt = Powertrain(le.engine, le.vehicle)
    pt.engine.start()
    c = pt.controls
    c.ignition, c.throttle, c.dyno_enabled, c.dyno_rpm = True, throttle, True, rpm
    for _ in range(300):                      # settle
        pt.step(dt)
    n = int(seconds / dt)
    t0 = time.process_time()
    for _ in range(n):
        pt.step(dt)
    # CPU time of this process, so other programs on the machine do not move the number.
    return n * dt / (time.process_time() - t0)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--engine", nargs="*", default=["1nz", "454", "vr38"])
    ap.add_argument("--seconds", type=float, default=1.0)
    ap.add_argument("--budget", type=int, default=None,
                    help="override max_cylinder_steps_per_ms (the quality/speed knob; default: the engine file's)")
    ap.add_argument("--dt", type=float, default=DT, help="physics step [s] (the dashboard uses 0.001)")
    ap.add_argument("--legacy", action="store_true", help="also time the cycle-averaged path")
    ap.add_argument("--python", action="store_true",
                    help="time the pure-Python core (use_native_core = false) instead of the compiled one")
    args = ap.parse_args()

    worst = 1e9
    for ref in args.engine:
        le = load_engine_file(resolve_engine(ref))
        cfg = le.engine
        if args.budget:
            cfg.max_cylinder_steps_per_ms = args.budget
            cfg.native_steps_per_ms = args.budget
        if args.python:
            cfg.use_native_core = False
        top = cfg.redline_rpm
        points = [cfg.idle_rpm, 0.25 * top, 0.5 * top, 0.75 * top, 0.95 * top]
        core = "compiled" if (cfg.use_native_core and native_available()) else "python"
        print(f"\n{cfg.name}  ({cfg.num_cylinders} cyl, resolved={cfg.use_resolved_cylinders}, core={core})")
        print(f"  {'rpm':>6}  {'WOT':>8}  {'closed':>8}")
        for rpm in points:
            row = []
            for thr in (1.0, 0.0):
                f = measure(le, rpm, thr, args.seconds, args.dt)
                worst = min(worst, f)
                row.append(f)
            flag = "  <-- below 1.5x" if min(row) < 1.5 else ""
            print(f"  {rpm:6.0f}  {row[0]:7.2f}x  {row[1]:7.2f}x{flag}")
    print(f"\nworst real-time factor: {worst:.2f}x")


if __name__ == "__main__":
    main()
