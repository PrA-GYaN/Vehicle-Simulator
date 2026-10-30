#!/usr/bin/env python3
"""Vendor EngineSim's compiled cylinder core into the vehicle simulator.

    python tools/sync_enginecore.py            # copy / regenerate
    python tools/sync_enginecore.py --check    # exit 1 if the vendored copy is stale

The resolved engine (src/physics/ResolvedEngine.cpp) does not re-implement the
cylinder physics: it compiles ../EngineSim/engine_sim/native/enginecore.c as it
is. Three files in src/physics/enginecore/ come from EngineSim and are written
ONLY by this script - never edit them by hand:

    enginecore.c     byte-for-byte copy of EngineSim's C inner loop
    fields.h         layout of the flat state arrays, generated from the
                     attribute lists in engine_sim/native_core.py (the same
                     header EngineSim generates for itself at build time, plus
                     the three array sizes)
    gas_tables.inc   the eight c_v / internal-energy tables of
                     engine_sim/gas_tables.py, as C initialisers

Run it whenever EngineSim's cylinder physics changes, then rebuild and re-run
tools/engine_parity_reference.py + engine_parity (README, "Keeping the port in
step with EngineSim"). Standard library only.
"""

from __future__ import annotations

import argparse
import hashlib
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DEST = os.path.normpath(os.path.join(HERE, "..", "src", "physics", "enginecore"))
DEFAULT_ENGINESIM = os.path.normpath(os.path.join(HERE, "..", "..", "EngineSim"))


def generate(enginesim: str) -> dict:
    """{file name: contents} of the three vendored files."""
    sys.path.insert(0, enginesim)
    from engine_sim import native_core as nc                      # noqa: E402
    from engine_sim.gas_tables import (                           # noqa: E402
        CV_AIR, CV_BURNED, CV_MIX, DUB_AIR, DUB_MIX, GRID_N, U_AIR, U_BURNED, U_MIX,
    )

    with open(os.path.join(enginesim, "engine_sim", "native", "enginecore.c"), "rb") as f:
        source = f.read()
    digest = hashlib.sha256(source).hexdigest()

    fields = nc._fields_header()
    fields += "/* array sizes, added by VehicleSimulator/tools/sync_enginecore.py */\n"
    fields += f"#define ES_NCS {len(nc._FIELDS)}\n"
    fields += f"#define ES_NK {len(nc._K_NAMES)}\n"
    fields += f"#define ES_NB {len(nc._B_NAMES)}\n"
    fields += f'#define ES_SOURCE_SHA256 "{digest}"\n'

    # Same order es_init_tables() reads them in (native_core._load).
    tables = (CV_AIR, CV_MIX, CV_BURNED, U_AIR, U_MIX, U_BURNED, DUB_AIR, DUB_MIX)
    lines = ["/* generated from engine_sim/gas_tables.py by tools/sync_enginecore.py - do not edit.",
             f" * 8 tables x {GRID_N} nodes (20 K grid, 0..5000 K), in es_init_tables() order:",
             " * CV_AIR, CV_MIX, CV_BURNED, U_AIR, U_MIX, U_BURNED, DUB_AIR, DUB_MIX. */"]
    for table in tables:
        assert len(table) == GRID_N
        for k in range(0, GRID_N, 4):
            lines.append(" ".join(f"{float(v)!r}," for v in table[k:k + 4]))
    gas = "\n".join(lines) + "\n"

    return {"enginecore.c": source, "fields.h": fields.encode(), "gas_tables.inc": gas.encode()}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--enginesim", default=DEFAULT_ENGINESIM, help="path to the EngineSim project")
    ap.add_argument("--check", action="store_true", help="only report whether the vendored copy is current")
    args = ap.parse_args()

    if not os.path.isfile(os.path.join(args.enginesim, "engine_sim", "native", "enginecore.c")):
        print(f"sync_enginecore: no EngineSim at {args.enginesim}", file=sys.stderr)
        return 2
    files = generate(args.enginesim)

    stale = []
    for name, data in files.items():
        path = os.path.join(DEST, name)
        try:
            with open(path, "rb") as f:
                current = f.read()
        except OSError:
            current = None
        if current != data:
            stale.append(name)
            if not args.check:
                os.makedirs(DEST, exist_ok=True)
                with open(path, "wb") as f:
                    f.write(data)

    if args.check:
        if stale:
            print("sync_enginecore: STALE - " + ", ".join(stale)
                  + " differ from EngineSim. Run tools/sync_enginecore.py.")
            return 1
        print("sync_enginecore: vendored core matches EngineSim")
        return 0
    print("sync_enginecore: " + ("updated " + ", ".join(stale) if stale else "already current"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
