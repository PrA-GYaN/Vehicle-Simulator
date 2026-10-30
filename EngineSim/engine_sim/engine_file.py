"""Engine definition files: one TOML file per engine holds everything.

    config/engines/chevrolet_454.toml      geometry, inertia, ECU, valves,
    config/engines/toyota_1nz_fe.toml      exhaust, intake, vehicle, sound

The same file drives the physics (`EngineConfig`, `VehicleConfig`), the
animation and the synthesized sound (`sound_spec.py`), so an engine is tuned in
exactly one place. Section → field mapping (see README.md §3):

    [engine] [layout] [valves]  → EngineConfig field of the same name
    [exhaust] key               → EngineConfig.exhaust_<key>
    [intake]  key               → EngineConfig.intake_<key>
    [sound]   key               → EngineConfig.sound_<key>
    [vehicle] key               → VehicleConfig field

Mistakes are reported with the file name, section, key and (for typos) a
suggestion; TOML syntax errors include line and column. Standard library only
(Python 3.11+ for `tomllib`).

Check a file from the command line:   python -m engine_sim.engine_file [file.toml ...]
"""

from __future__ import annotations

import dataclasses
import difflib
import math
import os
from dataclasses import dataclass
from typing import Any, List, Optional, Tuple

from .config import EngineConfig

try:
    import tomllib
except ImportError:      # Python < 3.11
    tomllib = None

ENGINES_DIR = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                            "..", "config", "engines"))

SECTION_PREFIX = {"engine": "", "layout": "", "valves": "", "exhaust": "exhaust_",
                  "intake": "intake_", "sound": "sound_"}
TOP_LEVEL_KEYS = {"id", "name", "description"}
# List fields whose elements cannot be inferred from a default value
LIST_KIND = {"firing_intervals_deg": float}
# Fields that exist on the dataclass but are not settable from a file
NOT_SETTABLE = {"torque_curve", "RPM_TO_RADS"}


class EngineFileError(ValueError):
    """A problem in an engine file, phrased for the person editing it."""


@dataclass
class LoadedEngine:
    id: str
    name: str
    description: str
    path: str
    engine: EngineConfig
    vehicle: Any            # powertrain.VehicleConfig

    def as_tuple(self):
        return self.engine, self.vehicle


@dataclass
class EngineListing:
    id: str
    name: str
    path: str
    error: Optional[str] = None


# ---------------------------------------------------------------- coercion
def _kind_of(default: Any, key: str):
    if isinstance(default, bool):
        return bool
    if isinstance(default, int):
        return int
    if isinstance(default, float):
        return float
    if isinstance(default, str):
        return str
    if isinstance(default, list):
        if default:
            return (list, _kind_of(default[0], key))
        return (list, LIST_KIND.get(key, float))
    if default is None and key in LIST_KIND:
        return (list, LIST_KIND[key])
    return None


def _coerce(where: str, key: str, value: Any, default: Any):
    kind = _kind_of(default, key)
    if kind is None:
        raise EngineFileError(f"{where} '{key}' cannot be set from a file")

    def one(v, k, label):
        if k is bool:
            if not isinstance(v, bool):
                raise EngineFileError(f"{where} '{key}'{label} must be true or false, got {v!r}")
            return v
        if k is str:
            if not isinstance(v, str):
                raise EngineFileError(f"{where} '{key}'{label} must be text in quotes, got {v!r}")
            return v
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            raise EngineFileError(f"{where} '{key}'{label} must be a number, got {v!r}")
        if k is int:
            if isinstance(v, float) and not v.is_integer():
                raise EngineFileError(f"{where} '{key}'{label} must be a whole number, got {v!r}")
            return int(v)
        if not math.isfinite(v):
            raise EngineFileError(f"{where} '{key}'{label} must be a finite number, got {v!r}")
        return float(v)

    if isinstance(kind, tuple):
        items = value if isinstance(value, list) else [value]      # a lone value = one-item list
        if not items:
            raise EngineFileError(f"{where} '{key}' must not be empty")
        return [one(v, kind[1], f"[{i}]") for i, v in enumerate(items)]
    if isinstance(value, list):
        raise EngineFileError(f"{where} '{key}' must be a single value, got a list")
    return one(value, kind, "")


def _apply(section: str, table: dict, target: Any, prefix: str, path: str, valid: set):
    for key, value in table.items():
        name = prefix + key
        if name not in valid or name in NOT_SETTABLE:
            local = sorted(v[len(prefix):] for v in valid if v.startswith(prefix) and v not in NOT_SETTABLE)
            close = difflib.get_close_matches(key, local, n=1, cutoff=0.6)
            hint = f" Did you mean '{close[0]}'?" if close else f" Valid keys: {', '.join(local)}."
            raise EngineFileError(f"{os.path.basename(path)}: unknown key '{key}' in [{section}].{hint}")
        default = getattr(target, name)
        try:
            setattr(target, name, _coerce(f"{os.path.basename(path)}: [{section}]", key, value, default))
        except EngineFileError:
            raise


# ---------------------------------------------------------------- validation
def _positive(where, **vals):
    for k, v in vals.items():
        if not (v > 0):
            raise EngineFileError(f"{where} {k} must be greater than 0, got {v}")


def exhaust_pipe_count(cfg: EngineConfig) -> int:
    return (max(cfg.cylinder_bank) + 1) if cfg.exhaust_layout == "dual" else 1


def validate_engine(cfg: EngineConfig, veh: Any, path: str) -> None:
    f = os.path.basename(path)
    n = cfg.num_cylinders
    if n < 1:
        raise EngineFileError(f"{f}: [engine] num_cylinders must be at least 1")
    if len(cfg.cylinder_bank) != n:
        raise EngineFileError(f"{f}: [layout] cylinder_bank needs one entry per cylinder ({n}), got {len(cfg.cylinder_bank)}")
    if any(b < 0 for b in cfg.cylinder_bank):
        raise EngineFileError(f"{f}: [layout] cylinder_bank entries are bank numbers starting at 0")
    if sorted(cfg.firing_order) != list(range(1, n + 1)):
        raise EngineFileError(f"{f}: [layout] firing_order must use each cylinder 1..{n} exactly once, got {cfg.firing_order}")
    if cfg.firing_intervals_deg is not None:
        iv = cfg.firing_intervals_deg
        if len(iv) != n or abs(sum(iv) - 720.0) > 0.5 or min(iv) <= 0:
            raise EngineFileError(f"{f}: [layout] firing_intervals_deg needs {n} positive values adding up to 720 (got {len(iv)} values summing to {sum(iv):g})")
    _positive(f + ": [engine]", bore_m=cfg.bore_m, stroke_m=cfg.stroke_m, rod_length_m=cfg.rod_length_m,
              engine_inertia_kgm2=cfg.engine_inertia_kgm2, idle_rpm=cfg.idle_rpm, redline_rpm=cfg.redline_rpm)
    if cfg.compression_ratio <= 1:
        raise EngineFileError(f"{f}: [engine] compression_ratio must be greater than 1")
    if cfg.rod_length_m < cfg.stroke_m / 2:
        raise EngineFileError(f"{f}: [engine] rod_length_m must be longer than half the stroke")
    if not (0 < cfg.stall_rpm < cfg.idle_rpm < cfg.redline_rpm):
        raise EngineFileError(f"{f}: [engine] need 0 < stall_rpm < idle_rpm < redline_rpm "
                              f"(got {cfg.stall_rpm:g}, {cfg.idle_rpm:g}, {cfg.redline_rpm:g})")
    if cfg.limiter_type not in ("torque_cut", "fuel_cut"):
        raise EngineFileError(f"{f}: [engine] limiter_type must be \"torque_cut\" or \"fuel_cut\"")
    for k in ("intake_open_btdc_deg", "intake_close_abdc_deg", "exhaust_open_bbdc_deg", "exhaust_close_atdc_deg"):
        v = getattr(cfg, k)
        if not (0 <= v <= 90):
            raise EngineFileError(f"{f}: [valves] {k} should be between 0 and 90 degrees, got {v:g}")
    _positive(f + ": [valves]", intake_head_dia_m=cfg.intake_head_dia_m, intake_port_dia_m=cfg.intake_port_dia_m,
              exhaust_head_dia_m=cfg.exhaust_head_dia_m, exhaust_port_dia_m=cfg.exhaust_port_dia_m,
              intake_max_lift_m=cfg.intake_max_lift_m, exhaust_max_lift_m=cfg.exhaust_max_lift_m)
    for k in ("intake_discharge_coeff", "exhaust_discharge_coeff", "exhaust_tail_discharge_coeff"):
        if not (0 < getattr(cfg, k) <= 1):
            raise EngineFileError(f"{f}: {k} must be in (0, 1], got {getattr(cfg, k):g}")
    for k in ("intake_seat_angle_deg", "exhaust_seat_angle_deg"):
        if not (0 <= getattr(cfg, k) < 90):
            raise EngineFileError(f"{f}: [valves] {k} must be in [0, 90) degrees (0 = flat seat), got {getattr(cfg, k):g}")
    for v in ("intake", "exhaust"):
        o, c = getattr(cfg, v + "_ramp_open_frac"), getattr(cfg, v + "_ramp_close_frac")
        if not (0 < o < 1) or abs(o + c - 1.0) > 1e-6:
            raise EngineFileError(f"{f}: [valves] {v}_ramp_open_frac and {v}_ramp_close_frac must add up to 1 "
                                  f"(0 < open < 1), got {o:g} + {c:g}")
    _positive(f + ": [exhaust]", plenum_volume_m3=cfg.exhaust_plenum_volume_m3, tail_dia_m=cfg.exhaust_tail_dia_m)
    _positive(f + ": [engine]", crank_angle_resolution_deg=cfg.crank_angle_resolution_deg,
              max_substeps_per_update=cfg.max_substeps_per_update, num_valves_per_cyl=cfg.num_valves_per_cyl)
    if cfg.use_resolved_cylinders and cfg.engine_type != "SI":
        raise EngineFileError(f"{f}: [engine] use_resolved_cylinders needs engine_type = \"SI\" "
                              "(the resolved core integrates a spark-ignition burn)")
    if cfg.exhaust_layout not in ("single", "dual"):
        raise EngineFileError(f"{f}: [exhaust] layout must be \"single\" or \"dual\", got {cfg.exhaust_layout!r}")
    banks = max(cfg.cylinder_bank) + 1
    if cfg.exhaust_layout == "dual" and banks < 2:
        raise EngineFileError(f"{f}: [exhaust] layout = \"dual\" needs two cylinder banks (a V or flat engine); "
                              "use \"single\" for an inline engine")
    pipes = exhaust_pipe_count(cfg)
    for k in ("header_length_m", "mid_length_m", "muffler_length_m", "tail_length_m", "muffler_body_hz"):
        vals = getattr(cfg, "exhaust_" + k)
        if len(vals) not in (1, pipes):
            raise EngineFileError(f"{f}: [exhaust] {k} needs 1 value (used for every pipe) or {pipes} values "
                                  f"(one per pipe), got {len(vals)}")
        if min(vals) <= 0:
            raise EngineFileError(f"{f}: [exhaust] {k} must be greater than 0")
    _positive(f + ": [exhaust]", muffler_area_ratio=cfg.exhaust_muffler_area_ratio,
              header_temp_k=cfg.exhaust_header_temp_k, mid_temp_k=cfg.exhaust_mid_temp_k,
              muffler_temp_k=cfg.exhaust_muffler_temp_k, tail_temp_k=cfg.exhaust_tail_temp_k)
    if not (0 < cfg.exhaust_pipe_loss <= 1):
        raise EngineFileError(f"{f}: [exhaust] pipe_loss must be in (0, 1]; 1 = lossless, typical 0.99")
    if not (0 < cfg.exhaust_tip_damping <= 1):
        raise EngineFileError(f"{f}: [exhaust] tip_damping must be in (0, 1]")
    _positive(f + ": [intake]", runner_length_m=cfg.intake_runner_length_m, runner_temp_k=cfg.intake_runner_temp_k)
    _positive(f + ": [sound]", blowdown_k=cfg.sound_blowdown_k, starter_ratio=cfg.sound_starter_ratio,
              starter_poles=cfg.sound_starter_poles)
    for k in ("cylinder_strength_spread", "combustion_variability", "idle_variability", "pop_probability", "crank_ripple_gain"):
        if getattr(cfg, "sound_" + k) < 0:
            raise EngineFileError(f"{f}: [sound] {k} must not be negative")
    if not veh.gear_ratios or min(veh.gear_ratios) <= 0:
        raise EngineFileError(f"{f}: [vehicle] gear_ratios needs at least one positive ratio")
    _positive(f + ": [vehicle]", mass_kg=veh.mass_kg, wheel_radius_m=veh.wheel_radius_m, final_drive=veh.final_drive)


def _expand_per_pipe(cfg: EngineConfig) -> None:
    """Per-pipe lists may hold one value; expand so every list has one entry per pipe."""
    pipes = exhaust_pipe_count(cfg)
    for k in ("header_length_m", "mid_length_m", "muffler_length_m", "tail_length_m", "muffler_body_hz"):
        vals = getattr(cfg, "exhaust_" + k)
        if len(vals) == 1:
            setattr(cfg, "exhaust_" + k, vals * pipes)


# ---------------------------------------------------------------- loading
def load_engine_file(path: str) -> LoadedEngine:
    """Load and validate one engine file. Raises EngineFileError with a readable message."""
    from .powertrain import VehicleConfig            # local import: powertrain imports this module
    path = os.path.abspath(path)
    fname = os.path.basename(path)
    if tomllib is None:
        raise EngineFileError("Engine files are TOML; this needs Python 3.11 or newer (tomllib)")
    try:
        with open(path, "rb") as fh:
            data = tomllib.load(fh)
    except FileNotFoundError:
        raise EngineFileError(f"Engine file not found: {path}") from None
    except tomllib.TOMLDecodeError as e:
        raise EngineFileError(f"{fname}: not valid TOML: {e}") from None
    except OSError as e:
        raise EngineFileError(f"{fname}: cannot read: {e}") from None

    cfg, veh = EngineConfig(), VehicleConfig()
    valid_engine = {f.name for f in dataclasses.fields(EngineConfig)}
    valid_vehicle = {f.name for f in dataclasses.fields(VehicleConfig)}
    for key in data:
        if key in TOP_LEVEL_KEYS or key in SECTION_PREFIX or key == "vehicle":
            continue
        opts = sorted(TOP_LEVEL_KEYS | set(SECTION_PREFIX) | {"vehicle"})
        close = difflib.get_close_matches(key, opts, n=1, cutoff=0.6)
        raise EngineFileError(f"{fname}: unknown section or key '{key}'."
                              + (f" Did you mean '{close[0]}'?" if close else f" Known: {', '.join(opts)}."))
    for key in ("id", "name", "description"):
        if key in data and not isinstance(data[key], str):
            raise EngineFileError(f"{fname}: '{key}' must be text in quotes")
    for section, prefix in SECTION_PREFIX.items():
        table = data.get(section, {})
        if not isinstance(table, dict):
            raise EngineFileError(f"{fname}: [{section}] must be a table")
        _apply(section, table, cfg, prefix, path, valid_engine)
    vtable = data.get("vehicle", {})
    if not isinstance(vtable, dict):
        raise EngineFileError(f"{fname}: [vehicle] must be a table")
    _apply("vehicle", vtable, veh, "", path, valid_vehicle)

    ident = data.get("id") or os.path.splitext(fname)[0]
    cfg.name = data.get("name", cfg.name if "name" in data else ident)
    validate_engine(cfg, veh, path)
    _expand_per_pipe(cfg)
    return LoadedEngine(ident, cfg.name, data.get("description", ""), path, cfg, veh)


def list_engines(directory: str = ENGINES_DIR) -> List[EngineListing]:
    """Every *.toml in the engines directory (broken files are listed with their error)."""
    out: List[EngineListing] = []
    if not os.path.isdir(directory):
        return out
    for fn in sorted(os.listdir(directory)):
        if not fn.endswith(".toml"):
            continue
        p = os.path.join(directory, fn)
        try:
            e = load_engine_file(p)
            out.append(EngineListing(e.id, e.name, p))
        except EngineFileError as err:
            out.append(EngineListing(os.path.splitext(fn)[0], os.path.splitext(fn)[0], p, str(err)))
    return out


def resolve_engine(ref: str, directory: str = ENGINES_DIR) -> str:
    """Engine id (e.g. "454") or a path to a .toml file → path."""
    if ref.endswith(".toml") or os.sep in ref or (os.altsep and os.altsep in ref):
        return os.path.abspath(ref)
    for e in list_engines(directory):
        if e.id == ref:
            return e.path
    ids = [e.id for e in list_engines(directory)]
    raise EngineFileError(f"No engine '{ref}' in {directory}. Available: {', '.join(ids) or '(none)'}")


# ---------------------------------------------------------------- command line
def describe(le: LoadedEngine) -> str:
    """Human summary of an engine file, including what the sound will do."""
    from .sound_spec import sound_spec
    c, v = le.engine, le.vehicle
    sp = sound_spec(c)
    n = c.num_cylinders
    lines = [f"{le.name}   ({le.id})   {le.path}",
             f"  {n} cylinders, {c.total_displacement * 1000:.2f} L, bore x stroke {c.bore_m * 1000:.1f} x {c.stroke_m * 1000:.1f} mm, CR {c.compression_ratio:g}:1",
             f"  idle {c.idle_rpm:g} rpm, redline {c.redline_rpm:g} rpm, crank+flywheel inertia {c.engine_inertia_kgm2:g} kg m2",
             f"  firing order {'-'.join(map(str, c.firing_order))}; firing frequency {n * c.idle_rpm / 120:.0f} Hz at idle, {n * c.redline_rpm / 120:.0f} Hz at redline",
             f"  vehicle {v.mass_kg:g} kg, {len(v.gear_ratios)} gears, final drive {v.final_drive:g}"]
    for k, pipe in enumerate(sp["exhaust"]["pipes"]):
        secs = pipe["sections"]
        total = sum(s["length_m"] for s in secs)
        f_qw = secs[0]["c"] / (4.0 * total)
        lines.append(f"  exhaust pipe {k + 1} ({sp['exhaust']['layout']}): {total:.2f} m, quarter-wave ring ~{f_qw:.0f} Hz "
                     f"(header {secs[0]['length_m']:.2f} + mid {secs[1]['length_m']:.2f} + muffler {secs[2]['length_m']:.2f} + tail {secs[3]['length_m']:.2f} m)")
    return "\n".join(lines)


def main(argv: Optional[List[str]] = None) -> int:
    import sys
    args = sys.argv[1:] if argv is None else argv
    paths = args or [e.path for e in list_engines()]
    bad = 0
    for p in paths:
        try:
            print(describe(load_engine_file(resolve_engine(p) if not os.path.exists(p) else p)))
            print("  OK\n")
        except EngineFileError as e:
            bad += 1
            print(f"ERROR {e}\n")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
