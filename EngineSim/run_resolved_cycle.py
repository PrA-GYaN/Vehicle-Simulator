"""Dump one crank-resolved cycle of one cylinder and print its energy audit.

    python run_resolved_cycle.py [--engine 1nz] [--rpm 3000] [--throttle 1.0]

Runs the engine on the resolved core (use_resolved_cylinders) pinned at a speed
by a dyno, waits for periodic steady state, then records cylinder 1 every
couple of crank degrees through one 720° cycle into output/resolved_cycle.csv:
pressure, temperature, mass, valve lifts, valve mass flows, torque, wall heat
rate. The audit shows where the fuel's energy went.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from engine_sim.engine_file import load_engine_file, resolve_engine   # noqa: E402
from engine_sim.powertrain import Powertrain                           # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--engine", default="1nz")
    ap.add_argument("--rpm", type=float, default=3000.0)
    ap.add_argument("--throttle", type=float, default=1.0)
    ap.add_argument("--out", default=os.path.join(HERE, "output", "resolved_cycle.csv"))
    args = ap.parse_args()

    le = load_engine_file(resolve_engine(args.engine))
    cfg = le.engine
    cfg.use_resolved_cylinders = True
    cfg.crank_angle_resolution_deg = 2.0
    pt = Powertrain(cfg, le.vehicle)
    pt.engine.start()
    c = pt.controls
    c.ignition, c.throttle, c.dyno_enabled, c.dyno_rpm = True, args.throttle, True, args.rpm
    dt = 0.0005
    for _ in range(int(1.5 / dt)):
        pt.step(dt)

    cyl = pt.engine.bank.cylinders[0]
    rows = []
    cycle_s = 120.0 / args.rpm
    for _ in range(int(cycle_s / dt) + 2):
        o = pt.step(dt)
        rows.append({
            "crank_deg": round(o.engine.crank_angle_deg, 2),
            "p_bar": cyl.pressure / 1e5, "T_K": cyl.temperature,
            "mass_mg": cyl.mass * 1e6,
            "intake_lift_mm": cyl.lift_intake * 1e3, "exhaust_lift_mm": cyl.lift_exhaust * 1e3,
            "intake_flow_gs": cyl.mass_flow_intake * 1e3, "exhaust_flow_gs": cyl.mass_flow_exhaust * 1e3,
            "gas_torque_Nm": cyl.torque, "wall_heat_W": cyl.wall_heat_transfer_rate,
            "map_kPa": o.engine.manifold_pressure / 1e3, "exhaust_kPa": o.engine.exhaust_pressure / 1e3,
        })
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)

    e = pt.engine.last_output
    q = cyl.last_cycle_heat_release_j
    work, wall = cyl.last_cycle_work_j, cyl.last_cycle_heat_loss_j
    pump = cyl.last_cycle_pumping_work_j
    print(f"{cfg.name}: {args.rpm:.0f} rpm, throttle {args.throttle:.2f}")
    print(f"  manifold {e.manifold_pressure / 1e3:.1f} kPa, exhaust {e.exhaust_pressure / 1e3:.1f} kPa, "
          f"VE {e.volumetric_efficiency:.3f}, lambda {e.lambda_val:.3f}")
    print(f"  peak pressure {cyl.last_peak_pressure_pa / 1e5:.1f} bar, peak temperature {cyl.last_peak_temperature_k:.0f} K")
    print(f"  IMEP net {e.imep_net_bar:.2f} bar, gross {e.imep_gross_bar:.2f} bar, PMEP {e.pmep_bar:.2f} bar")
    print("  energy audit, cylinder 1, one cycle:")
    print(f"    heat released   {q:8.1f} J   100.0 %")
    print(f"    net work        {work:8.1f} J   {100 * work / q:5.1f} %   (incl. pumping {pump:+.1f} J)")
    print(f"    wall heat       {wall:8.1f} J   {100 * wall / q:5.1f} %")
    print(f"    exhaust+blowdn  {q - work - wall:8.1f} J   {100 * (q - work - wall) / q:5.1f} %   (remainder)")
    print(f"  wrote {len(rows)} rows to {os.path.relpath(args.out, HERE)}")


if __name__ == "__main__":
    main()
