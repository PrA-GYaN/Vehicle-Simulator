"""Optional compiled inner loop for the resolved engine.

The resolved cylinder model costs about 16 µs per cylinder-substep in CPython, which
is the interpreter's floor. ``native/enginecore.c`` is a line-for-line transcription of
``Cylinder.step``, ``begin_cycle`` and the substep loop of ``CylinderBank.advance``
(cylinders, exhaust plenum and turbine stage, boost plenum, throttle, manifold), about
30× faster. :class:`NativeCylinderBank` runs that loop; everything around it — fuel
metering, the ECU's air-model learning, the shaft and wastegate, the per-cycle
summaries — stays the unchanged Python of :class:`CylinderBank`.

How the two sides share state. The ``Cylinder`` objects stay the source of truth. Before
the call each cylinder's attributes are packed into one flat array of doubles, the C
loop works on that array, and afterwards the attributes are written back, so every
Python reader (the engine, the dashboard, the tests) sees ordinary attributes and a
cylinder can still be stepped in Python between calls. The layout (``fields.h``) is
generated from the attribute lists below, so the two sides cannot drift apart.

Building. The library is compiled on first use with the system C compiler (gcc, cc or
clang) into a per-user cache keyed on the source hash, with ``-ffp-contract=off`` and no
fast-math so it rounds like the Python path. If there is no compiler, the build fails, or
``ENGINESIM_NATIVE=0`` is set, :func:`native_available` is False and the engine silently
uses the Python bank.
"""

from __future__ import annotations

import ctypes
import hashlib
import os
import shutil
import struct
import subprocess
import sys
import tempfile
from array import array
from operator import attrgetter
from pathlib import Path
from typing import List, Optional

from .config import EngineConfig
from .cylinder import CYCLE_DEG, TRACE_BINS
from .cylinder_bank import CylinderBank
from .gas_tables import (
    CV_AIR, CV_BURNED, CV_MIX, DUB_AIR, DUB_MIX, U_AIR, U_BURNED, U_MIX,
)
from .thermodynamics import R_AIR

_SOURCE = Path(__file__).with_name("native") / "enginecore.c"

# ── Layout ────────────────────────────────────────────────────────────────

# Cylinder attributes that are plain numbers (or flags) and travel as they are.
_PLAIN = [
    "mass", "burned_mass", "temperature", "pressure", "fuel_mass",
    "_edge_a", "_edge_v", "_h_c", "_h_age", "_runner_burned", "_zone_active",
    "_mu", "_mb", "_tu", "_tb", "_yu", "_m_at_spark", "_phi_cycle",
    "no_mole_fraction", "last_no_ppm", "peak_burned_temperature_k",
    "last_peak_burned_temperature_k", "unburned_temperature_k",
    "burn_quality", "residual_at_spark", "misfired", "misfire_count",
    "spark_angle_deg", "combustion_duration_deg", "_burn_window",
    "fuel_energy_remaining", "_pool_total", "_burned_fraction_prev", "_unburned_at_spark",
    "_motoring_reference_pa", "_motoring_reference_k", "_motoring_reference_m3",
    "combustion_occurred", "fired_this_cycle", "_pool_pending", "energy_scale",
    "work_net", "work_pumping", "angle_swept",
    "_cycle_work", "_cycle_pumping", "_cycle_heat_loss", "_cycle_heat_release",
    "last_cycle_work_j", "last_cycle_pumping_work_j", "last_cycle_heat_loss_j",
    "last_cycle_heat_release_j", "peak_pressure_pa", "peak_temperature_k",
    "last_peak_pressure_pa", "last_peak_temperature_k", "residual_mass_kg",
    "trapped_air_kg", "trapped_mass_kg", "_fuel_in_cycle", "cycles_completed",
    "ivc_events", "last_reference_pressure_pa", "last_reference_temperature_k",
    "knock_integral", "knock_detected", "knock_events",
    "torque", "dm_in", "dm_ex", "_dt", "wall_heat_transfer_rate", "burned_fraction",
    "heat_released_total", "heat_loss_total",
    "_ivc_deg", "_exhaust_open", "_cam_in", "_cam_ex", "_cam_on", "record_trace",
]
# Attributes that need translating: None → -1, table references, the inlet-enthalpy tuple.
_SPECIAL = ["_fuel_per_air", "_last_angle", "_fuel_tab", "_inlet_t", "_inlet_h"]
_FIELDS = _PLAIN + _SPECIAL
_BOOLS = ["misfired", "combustion_occurred", "fired_this_cycle", "_pool_pending",
          "knock_detected", "_zone_active", "_cam_on", "record_trace"]
_INTS = ["misfire_count", "cycles_completed", "ivc_events", "knock_events", "_h_age"]

_K_NAMES = [
    "n_cyl", "stroke", "k_woschni", "area_const", "area_per_volume", "displaced_volume",
    "wall_t", "stoich", "lhv_eff", "wiebe_a", "wiebe_m1", "knock_on", "misfire_q", "two_zone",
    "clearance_volume", "res_lo", "res_hi", "lean_lo", "lean_hi", "rich_lo", "rich_hi", "octane",
    "p_amb", "ex_t_amb", "ex_volume", "tail_area", "tail_cd", "has_turbine", "turb_area",
    "wg_area", "post_volume", "eta_turb",
    "design_omega", "design_ratio", "design_flow", "eta_c", "t_ambient",
]
_B_NAMES = [
    "n_sub", "sub_deg", "sub_dt", "rpm", "p_man", "t_man", "k_man", "thr_cd", "p_upstream",
    "map_f", "alpha", "fuel_per_cycle", "air_ref", "spark_local", "burn_deg", "hold_manifold",
    "use_turbo", "p_b", "t_b", "k_boost", "by_cd", "bov_cd", "t_amb", "omega", "wg_pos",
    "ex_p", "ex_t", "ex_m", "post_p", "post_t", "post_m",
    "p_min", "p_max", "flow_in_sum", "thr_sum", "e_turbine", "e_comp",
    "tail_flow", "turb_flow", "wg_flow", "turb_power",
]
_KI = {n: i for i, n in enumerate(_K_NAMES)}
_BI = {n: i for i, n in enumerate(_B_NAMES)}
_NCS = len(_FIELDS)


def _fields_header() -> str:
    lines = ["/* generated from native_core.py — do not edit */"]
    lines += [f"#define F_{n} {i}" for i, n in enumerate(_FIELDS)]
    lines += [f"#define K_{n} {i}" for i, n in enumerate(_K_NAMES)]
    lines += [f"#define B_{n} {i}" for i, n in enumerate(_B_NAMES)]
    return "\n".join(lines) + "\n"


# ── Building and loading ──────────────────────────────────────────────────

_LIB = None
_LIB_TRIED = False
_LIB_ERROR = ""


def _cache_dir() -> Path:
    base = os.environ.get("ENGINESIM_CACHE")
    if base:
        return Path(base)
    try:
        uid = os.getuid()
    except AttributeError:          # pragma: no cover  (Windows)
        uid = 0
    return Path(tempfile.gettempdir()) / f"enginesim-native-{uid}"


def _compiler() -> Optional[str]:
    env = os.environ.get("CC")
    for cand in ([env] if env else []) + ["gcc", "cc", "clang"]:
        if cand and shutil.which(cand):
            return cand
    return None


def _build() -> Path:
    source = _SOURCE.read_text()
    header = _fields_header()
    cc = _compiler()
    if cc is None:
        raise RuntimeError("no C compiler found")
    tag = hashlib.sha256((source + header + cc + sys.platform).encode()).hexdigest()[:16]
    ext = ".dll" if sys.platform.startswith("win") else (".dylib" if sys.platform == "darwin" else ".so")
    out_dir = _cache_dir()
    out_dir.mkdir(parents=True, exist_ok=True)
    lib = out_dir / f"enginecore-{tag}{ext}"
    if lib.exists():
        return lib
    work = Path(tempfile.mkdtemp(prefix="build-", dir=out_dir))
    try:
        (work / "fields.h").write_text(header)
        shutil.copy(_SOURCE, work / "enginecore.c")
        tmp_out = work / f"out{ext}"
        cmd = [cc, "-O2", "-shared", "-fPIC", "-ffp-contract=off", "-fno-fast-math",
               "-fno-math-errno", "-o", str(tmp_out), str(work / "enginecore.c"), "-lm"]
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
        if res.returncode != 0:
            raise RuntimeError("C build failed:\n" + res.stderr[-2000:])
        os.replace(tmp_out, lib)
    finally:
        shutil.rmtree(work, ignore_errors=True)
    return lib


def _load():
    global _LIB, _LIB_TRIED, _LIB_ERROR
    if _LIB_TRIED:
        return _LIB
    _LIB_TRIED = True
    if os.environ.get("ENGINESIM_NATIVE", "1").strip().lower() in ("0", "off", "false", "no"):
        _LIB_ERROR = "disabled by ENGINESIM_NATIVE"
        return None
    try:
        lib = ctypes.CDLL(str(_build()))
        vp = ctypes.c_void_p
        lib.es_init_tables.argtypes = [vp]
        lib.es_init_tables.restype = None
        lib.es_advance.argtypes = [vp, vp, vp, vp, vp, ctypes.c_int, vp, vp, vp, vp, vp, ctypes.c_int]
        lib.es_advance.restype = None
        tables = array("d")
        for t in (CV_AIR, CV_MIX, CV_BURNED, U_AIR, U_MIX, U_BURNED, DUB_AIR, DUB_MIX):
            tables.extend(t)
        lib.es_init_tables(tables.buffer_info()[0])
        _LIB = lib
    except Exception as exc:        # no compiler, build error, wrong platform … → Python path
        _LIB = None
        _LIB_ERROR = str(exc)
    return _LIB


def native_available() -> bool:
    """True when the compiled core can be used on this machine."""
    return _load() is not None


def native_status() -> str:
    """One line saying whether the compiled core is in use, and why not if it is not."""
    return "compiled core available" if native_available() else f"Python core ({_LIB_ERROR})"


# ── The bank ──────────────────────────────────────────────────────────────

_PLAIN_GET = attrgetter(*_PLAIN)
_PACK = struct.Struct(f"{_NCS}d")
_NPLAIN = len(_PLAIN)


class NativeCylinderBank(CylinderBank):
    """``CylinderBank`` whose substep loop runs in compiled code (see the module docstring)."""

    native = True

    def __init__(self, cfg: EngineConfig):
        lib = _load()
        if lib is None:
            raise RuntimeError(f"compiled core unavailable: {_LIB_ERROR}")
        self._lib = lib
        super().__init__(cfg)
        n = len(self.cylinders)
        # Geometry tables are shared by every cylinder of an engine.
        cyl = self.cylinders[0]
        self._vol = array("d", cyl._vol_tab)
        self._ain = array("d", cyl._area_in)
        self._aex = array("d", cyl._area_ex)
        self._cs = array("d", bytes(8 * _NCS * n))
        self._trace = array("d", bytes(8 * TRACE_BINS))
        self._loc = array("d", bytes(8 * n))
        self._bk = array("d", bytes(8 * n))
        self._B = array("d", bytes(8 * len(_B_NAMES)))
        self._es = array("d", bytes(8 * max(1, n)))
        self._addr = {k: v.buffer_info()[0] for k, v in (
            ("vol", self._vol), ("ain", self._ain), ("aex", self._aex), ("cs", self._cs),
            ("trace", self._trace), ("loc", self._loc), ("bk", self._bk), ("B", self._B),
            ("es", self._es))}
        self._K = array("d", self._constants())
        self._addr["K"] = self._K.buffer_info()[0]
        # With the loop compiled, the cylinder-step budget and the crank-angle resolution can
        # follow the (much larger) native throughput instead of the interpreter's.
        self._nominal_budget = max(2 * n, int(cfg.native_steps_per_ms))
        self.step_budget = self._nominal_budget
        if cfg.native_crank_angle_resolution_deg > 0.0:
            self._res_deg = cfg.native_crank_angle_resolution_deg

    def _constants(self) -> List[float]:
        cfg = self.cfg
        cyl = self.cylinders[0]
        ex = self.exhaust
        t = self.turbo
        k = dict(
            n_cyl=len(self.cylinders), stroke=cyl.stroke, k_woschni=cyl._k_woschni,
            area_const=cyl._area_const, area_per_volume=cyl._area_per_volume,
            displaced_volume=cyl.displaced_volume, wall_t=cyl._wall_t, stoich=cyl._stoich,
            lhv_eff=cyl._lhv_eff, wiebe_a=cyl._wiebe_a, wiebe_m1=cyl._wiebe_m1,
            knock_on=1.0 if cyl._knock_on else 0.0, misfire_q=cyl._misfire_q,
            two_zone=1.0 if cyl._two_zone else 0.0, clearance_volume=cyl.clearance_volume,
            res_lo=cfg.burn_residual_lo, res_hi=cfg.burn_residual_hi,
            lean_lo=cfg.burn_lean_lo, lean_hi=cfg.burn_lean_hi,
            rich_lo=cfg.burn_rich_lo, rich_hi=cfg.burn_rich_hi, octane=cfg.fuel_octane_ron,
            p_amb=cfg.ambient_pressure_pa, ex_t_amb=ex.ambient_temperature_k,
            ex_volume=ex.volume_m3, tail_area=ex.tail_area_m2, tail_cd=ex.discharge_coeff,
            has_turbine=1.0 if ex.has_turbine else 0.0, turb_area=ex.turbine_area_m2,
            wg_area=ex.wastegate_area_m2, post_volume=ex.post_volume_m3,
            eta_turb=ex.turbine_efficiency,
            design_omega=t.design_omega if t else 1.0, design_ratio=t.design_ratio if t else 1.0,
            design_flow=t.design_flow if t else 0.0, eta_c=t.eta_c if t else 1.0,
            t_ambient=t.t_ambient if t else 293.0)
        return [float(k[name]) for name in _K_NAMES]

    # ── Python ⇄ flat arrays ──────────────────────────────────────────────

    def _pack_cylinders(self) -> None:
        pack_into = _PACK.pack_into
        cs = self._cs
        off = 0
        stride = _NCS * 8
        for cyl in self.cylinders:
            fpa = cyl._fuel_per_air
            last = cyl._last_angle
            inlet = cyl._inlet_cache
            pack_into(cs, off, *_PLAIN_GET(cyl),
                      -1.0 if fpa is None else fpa,
                      -1.0 if last is None else last,
                      1.0 if cyl._cvu is CV_MIX else 0.0,
                      inlet[0], inlet[1])
            off += stride
        self._trace[:] = array("d", self.cylinders[0].p_trace)

    def _unpack_cylinders(self) -> None:
        cs = self._cs
        unpack_from = _PACK.unpack_from
        off = 0
        stride = _NCS * 8
        for cyl in self.cylinders:
            vals = unpack_from(cs, off)
            off += stride
            d = cyl.__dict__
            d.update(zip(_PLAIN, vals))
            for name in _BOOLS:
                d[name] = d[name] != 0.0
            for name in _INTS:
                d[name] = int(d[name])
            fpa, last, fuel_tab, inlet_t, inlet_h = vals[_NPLAIN:]
            d["_fuel_per_air"] = None if fpa < 0.0 else fpa
            d["_last_angle"] = None if last < 0.0 else last
            d["_inlet_cache"] = (inlet_t, inlet_h)
            if fuel_tab != 0.0:
                d["_cvu"], d["_uu"], d["_dub"] = CV_MIX, U_MIX, DUB_MIX
            else:
                d["_cvu"], d["_uu"], d["_dub"] = CV_AIR, U_AIR, DUB_AIR
        self.cylinders[0].p_trace[:] = self._trace.tolist()

    # ── The substep loop ──────────────────────────────────────────────────

    def _substeps(self, n_sub, sub_deg, sub_dt, rpm, p_man, t_man, throttle_area, p_upstream,
                  hold_manifold, loc, begin_k, fuel_per_cycle, air_ref, spark_local, burn_deg,
                  energy_scale, alpha):
        cfg = self.cfg
        ex = self.exhaust
        turbo = self.turbo
        B = self._B
        bi = _BI
        B[bi["n_sub"]] = n_sub
        B[bi["sub_deg"]] = sub_deg
        B[bi["sub_dt"]] = sub_dt
        B[bi["rpm"]] = rpm
        B[bi["p_man"]] = p_man
        B[bi["t_man"]] = t_man
        B[bi["k_man"]] = R_AIR * t_man / cfg.manifold_volume_m3
        B[bi["thr_cd"]] = throttle_area * cfg.throttle_discharge_coeff
        B[bi["p_upstream"]] = p_upstream
        B[bi["map_f"]] = self.map_filtered
        B[bi["alpha"]] = alpha
        B[bi["fuel_per_cycle"]] = fuel_per_cycle
        B[bi["air_ref"]] = air_ref
        B[bi["spark_local"]] = spark_local
        B[bi["burn_deg"]] = burn_deg
        B[bi["hold_manifold"]] = 1.0 if hold_manifold else 0.0
        if turbo is not None:
            ex.wastegate_position = turbo.wastegate_position
            B[bi["use_turbo"]] = 1.0
            B[bi["p_b"]] = turbo.boost_pressure
            B[bi["t_b"]] = turbo.boost_temperature
            B[bi["k_boost"]] = R_AIR * turbo.boost_temperature / turbo.volume
            B[bi["by_cd"]] = turbo.bypass_area * 0.7
            B[bi["bov_cd"]] = (turbo.bov_area * 0.7
                               if throttle_area < 0.05 * cfg.throttle_max_area_m2 else 0.0)
            B[bi["t_amb"]] = cfg.intake_temperature_k
            B[bi["omega"]] = turbo.omega
            B[bi["wg_pos"]] = turbo.wastegate_position
        else:
            B[bi["use_turbo"]] = 0.0
        B[bi["ex_p"]] = ex.pressure
        B[bi["ex_t"]] = ex.temperature
        B[bi["ex_m"]] = ex.mass
        B[bi["post_p"]] = ex.post_pressure
        B[bi["post_t"]] = ex.post_temperature
        B[bi["post_m"]] = ex.post_mass
        B[bi["tail_flow"]] = ex.tail_mass_flow
        B[bi["turb_flow"]] = ex.turbine_mass_flow
        B[bi["wg_flow"]] = ex.wastegate_mass_flow
        B[bi["turb_power"]] = ex.turbine_power_w
        self._loc[:] = array("d", loc)
        self._bk[:] = array("d", begin_k)
        n_es = 0
        if energy_scale is not None and len(energy_scale) > 0:
            n_es = len(energy_scale)
            if n_es > len(self._es):
                self._es = array("d", bytes(8 * n_es))
                self._addr["es"] = self._es.buffer_info()[0]
            self._es[:n_es] = array("d", energy_scale)

        self._pack_cylinders()
        a = self._addr
        self._lib.es_advance(a["K"], a["vol"], a["ain"], a["aex"], a["cs"], _NCS, a["trace"],
                             a["B"], a["loc"], a["bk"], a["es"], n_es)
        self._unpack_cylinders()

        ex.pressure = B[bi["ex_p"]]
        ex.temperature = B[bi["ex_t"]]
        ex.mass = B[bi["ex_m"]]
        ex.post_pressure = B[bi["post_p"]]
        ex.post_temperature = B[bi["post_t"]]
        ex.post_mass = B[bi["post_m"]]
        ex.tail_mass_flow = B[bi["tail_flow"]]
        ex.turbine_mass_flow = B[bi["turb_flow"]]
        ex.wastegate_mass_flow = B[bi["wg_flow"]]
        ex.turbine_power_w = B[bi["turb_power"]]
        return (B[bi["p_man"]], B[bi["p_min"]], B[bi["p_max"]], B[bi["map_f"]],
                B[bi["flow_in_sum"]], B[bi["thr_sum"]], B[bi["p_b"]],
                B[bi["e_turbine"]], B[bi["e_comp"]])
