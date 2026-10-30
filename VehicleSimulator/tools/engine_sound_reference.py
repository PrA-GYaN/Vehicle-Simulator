#!/usr/bin/env python3
"""Record EngineSim's own engine sound for the C++ synthesiser to be checked against.

    python tools/engine_sound_reference.py 454 --out /tmp/s454 [--profile wild]
    build/bin/engine_sound_parity /tmp/s454.yaml /tmp/s454.states.csv /tmp/s454.f32

audio/EngineSimSynth.cpp is a transcription of EngineSim's sound worklet
(dashboard/engine-audio.js). This drives EngineSim's engine through a run that
exercises every source in it, and renders the ORIGINAL worklet over that run
under Node (tools/engine_sound_render.mjs). Four files come out:

    <out>.yaml         the engine and its `sound: engine_sim:` block, in this
                       simulator's format (engine_toml_to_yaml)
    <out>.spec.json    what EngineSim hands its worklet: sound_spec(), the dials
    <out>.states.csv   one row per 128-frame audio block: the `state` message
    <out>.f32          the worklet's output, interleaved stereo float32, 48 kHz

`engine_sound_parity` builds the same spec from the YAML through the simulator's
own code path, replays the states through EngineSimSynth and compares every
sample. The run: a cold start on the starter (starter whine, solenoid), idle
(lope, crank ripple), a free rev into the limiter, a lift at speed (the hollow
overrun note, pops, throttle hiss), full throttle under load.

--profile wild moves all fourteen dials off their defaults, which switches on
the sources that are silent at "Stock" (reverb tail, shelves, rumble, air,
valve tick). Needs Node on the PATH. Standard library only.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import engine_toml_to_yaml as conv                                   # noqa: E402

SAMPLE_RATE = 48000
BLOCK = 128
DT = BLOCK / SAMPLE_RATE

PROFILES = {
    "stock": {"vol": 0.7, "conv": 0.35, "comp": 0.6, "drive": 0.0, "lf": 0.0, "hf": 0.0, "ring": 0.6,
              "rasp": 0.5, "lope": 0.5, "nlf": 0.0, "nhf": 0.0, "tick": 0.0, "intk": 0.0, "pops": 0.5},
    "wild": {"vol": 0.55, "conv": 0.8, "comp": 0.9, "drive": 0.4, "lf": 6.0, "hf": 5.0, "ring": 0.85,
             "rasp": 0.8, "lope": 0.9, "nlf": 0.6, "nhf": 0.5, "tick": 0.4, "intk": 0.5, "pops": 1.0},
}

COLUMNS = ["rpm", "map", "throttle", "combusting", "fuelCut", "starter", "ignition",
           "indicated", "friction", "pumping", "redline", "idle"]


def script(cfg):
    """(throttle, load, starter) per audio block."""
    def run(seconds, throttle, load=0.0, starter=False):
        for _ in range(int(round(seconds / DT))):
            yield throttle, load, starter

    load = 45.0 * cfg.total_displacement * 1000.0
    yield from run(1.6, 0.0, starter=True)      # cold: crank on the starter
    yield from run(1.4, 0.0)                    # release it, settle to idle
    yield from run(0.9, 1.0)                    # free rev into the limiter
    yield from run(2.0, 0.0)                    # lift: overrun, pops, hiss, catch the idle
    yield from run(0.4, 0.5)                    # bring the revs up...
    yield from run(1.2, 1.0, load)              # ...and full throttle under load
    yield from run(0.8, 0.0)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("engine", help='EngineSim engine id ("1nz", "454", "vr38") or a .toml path')
    ap.add_argument("--out", required=True, help="output path without extension")
    ap.add_argument("--profile", choices=sorted(PROFILES), default="stock")
    ap.add_argument("--enginesim", default=conv.DEFAULT_ENGINESIM)
    args = ap.parse_args()

    node = shutil.which("node")
    if node is None:
        print("engine_sound_reference: needs Node (to run EngineSim's worklet offline)", file=sys.stderr)
        return 1
    le = conv.load(args.engine, args.enginesim)
    cfg = le.engine
    from engine_sim.engine import Engine, EngineInput                 # noqa: E402
    from engine_sim.sound_spec import sound_spec                      # noqa: E402
    profile = PROFILES[args.profile]

    with open(args.out + ".yaml", "w") as f:
        f.write(conv.engine_yaml(cfg, None, os.path.basename(le.path), profile=profile))
    with open(args.out + ".spec.json", "w") as f:
        json.dump({"sampleRate": SAMPLE_RATE, "sound": sound_spec(cfg), "profile": profile}, f)

    eng = Engine(cfg)
    eng.reset()
    rows = 0
    with open(args.out + ".states.csv", "w") as f:
        f.write(",".join(COLUMNS) + "\n")
        for throttle, load, starter in script(cfg):
            o = eng.update(EngineInput(throttle=throttle, load_torque=load, starter=starter), DT)
            # What dashboard/app.js feedAudio() sends (speed, gear and load stay 0:
            # road noise and driveline whine are the vehicle simulator's own).
            values = [o.rpm, o.intake_pressure / cfg.ambient_pressure_pa, throttle,
                      int(o.combusting), int(o.fuel_cut), int(starter), 1,
                      o.indicated_torque, o.friction_torque, o.pumping_torque,
                      cfg.redline_rpm, cfg.idle_rpm]
            f.write(",".join(repr(float(v)) if isinstance(v, float) else str(v) for v in values) + "\n")
            rows += 1

    worklet = os.path.join(args.enginesim, "dashboard", "engine-audio.js")
    res = subprocess.run([node, os.path.join(HERE, "engine_sound_render.mjs"), worklet,
                          args.out + ".spec.json", args.out + ".states.csv", args.out + ".f32"])
    if res.returncode != 0:
        return res.returncode
    print(f"engine_sound_reference: {cfg.name} ({args.profile}): {rows} blocks, "
          f"{rows * DT:.1f} s -> {args.out}.f32")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
