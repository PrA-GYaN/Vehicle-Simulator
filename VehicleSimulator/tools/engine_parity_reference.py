#!/usr/bin/env python3
"""Record a reference trace from EngineSim for the C++ port to be checked against.

    python tools/engine_parity_reference.py 1nz --out /tmp/parity_1nz
    build/bin/engine_parity /tmp/parity_1nz.yaml /tmp/parity_1nz.csv

Drives EngineSim's own `Engine` (compiled core) through a scripted run and
writes two files:

    <out>.yaml   the engine, in this simulator's format (engine_toml_to_yaml)
    <out>.csv    one row per engine step: the inputs, then what EngineSim
                 computed

`engine_parity` loads the YAML into physics/ResolvedEngine, replays the inputs
and compares every output column. The script covers each branch of the ECU and
both ways the vehicle drives the engine:

    warm start and idle            idle-air PI, spark trim, fast-idle ramp
    free rev at full throttle      the fuel-cut rev limiter
    lift                           decel fuel cut, dashpot, fuel resume
    full throttle against a load   MBT spark, VVT, turbo spool
    part throttle
    imposed speed sweep            the locked-driveline path (speed is set
                                   from outside, as wheel speed x ratio)
    cold start on the starter      sync, first fire, the start-up flare

Agreement is to rounding (see README, "Keeping the port in step with
EngineSim"), so this is a regression test for the transcription, not a
tolerance on the physics. Standard library only.
"""

from __future__ import annotations

import argparse
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import engine_toml_to_yaml as conv                                   # noqa: E402

DT = 0.002

COLUMNS = ["cmd", "imposed", "dt", "throttle", "load", "starter", "omega_set",
           "rpm", "net_torque", "gross_torque", "pumping_torque", "friction_torque",
           "manifold_pressure", "lambda", "spark", "burn", "iac",
           "exhaust_pressure", "exhaust_temperature", "boost_pressure", "turbo_rps",
           "wastegate", "ve", "air_per_cycle", "fuel_per_cycle", "crank_angle",
           "imep_net_bar", "peak_pressure"]


def script(cfg):
    """Yield (cmd, imposed, throttle, load, starter, omega_set) per step.

    cmd: 0 = step, 1 = Engine.start() then step, 2 = Engine.reset() then step.
    """
    def run(seconds, throttle, load=0.0, starter=False, cmd=0):
        for k in range(int(round(seconds / DT))):
            yield (cmd if k == 0 else 0, 0, throttle, load, starter, 0.0)

    litres = cfg.total_displacement * 1000.0
    load = 45.0 * litres                       # about half of a naturally aspirated WOT torque
    yield from run(1.5, 0.0, cmd=1)            # warm start, idle
    yield from run(1.0, 1.0)                   # free rev at full throttle: into the limiter
    yield from run(2.0, 0.0)                   # lift: fuel cut, then catch the idle
    yield from run(1.5, 1.0, load)             # full throttle against a load
    yield from run(1.0, 0.3, 0.2 * load)       # part throttle
    yield from run(1.0, 0.0)                   # lift again, from under load

    def rads(rpm):
        return rpm * math.pi / 30.0

    lo, hi = 1.5 * cfg.idle_rpm, 0.9 * cfg.redline_rpm
    n = int(round(2.0 / DT))
    for k in range(n):                         # imposed speed, WOT, rising
        yield (0, 1, 1.0, 0.0, False, rads(lo + (hi - lo) * k / (n - 1)))
    for k in range(int(round(1.0 / DT))):      # imposed speed, closed throttle: motoring
        yield (0, 1, 0.0, 0.0, False, rads(0.6 * cfg.redline_rpm))

    yield from run(2.0, 0.0, starter=True, cmd=2)   # cold: crank on the starter
    yield from run(1.5, 0.0)                         # release it, settle


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("engine", help='EngineSim engine id ("1nz", "454", "vr38") or a .toml path')
    ap.add_argument("--out", required=True, help="output path without extension")
    ap.add_argument("--enginesim", default=conv.DEFAULT_ENGINESIM)
    args = ap.parse_args()

    le = conv.load(args.engine, args.enginesim)
    cfg = le.engine
    from engine_sim.engine import Engine, EngineInput                 # noqa: E402
    from engine_sim.native_core import native_available, native_status  # noqa: E402
    if not (cfg.use_native_core and native_available()):
        print(f"engine_parity_reference: needs EngineSim's compiled core ({native_status()})",
              file=sys.stderr)
        return 1
    for note in conv.unsupported(cfg):
        print(f"engine_parity_reference: note: {note}", file=sys.stderr)

    with open(args.out + ".yaml", "w") as f:
        f.write(conv.engine_yaml(cfg, None, os.path.basename(le.path)))

    eng = Engine(cfg)
    rows = 0
    with open(args.out + ".csv", "w") as f:
        f.write(",".join(COLUMNS) + "\n")
        for cmd, imposed, throttle, load, starter, omega_set in script(cfg):
            if cmd == 1:
                eng.start()
            elif cmd == 2:
                eng.reset()
            if imposed:
                eng.state.omega = omega_set
            o = eng.update(EngineInput(throttle=throttle, load_torque=load, starter=starter), DT)
            values = [cmd, imposed, DT, throttle, load, int(starter), omega_set,
                      o.rpm, o.net_torque, o.indicated_torque, o.pumping_torque, o.friction_torque,
                      o.manifold_pressure, o.lambda_val, o.spark_timing_deg,
                      o.combustion_duration_deg, o.iac_position,
                      o.exhaust_pressure, o.exhaust_temperature, o.boost_pressure,
                      o.turbo_speed_rps, o.wastegate_position, o.volumetric_efficiency,
                      o.air_mass_per_cycle, o.fuel_mass_per_cycle, o.crank_angle_deg,
                      o.imep_net_bar, o.peak_pressure_pa]
            f.write(",".join(repr(float(v)) if isinstance(v, float) else str(v) for v in values) + "\n")
            rows += 1
    print(f"engine_parity_reference: {cfg.name}: {rows} steps -> {args.out}.csv, {args.out}.yaml")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
