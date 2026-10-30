"""One cylinder's gas state, resolved over crank angle.

This is the heart of the resolved engine model: for every cylinder the gas is
carried from substep to substep as mass, temperature, burned mass and chemical
energy, and the pressure that follows from the equation of state is the *same*
pressure that produces crank torque and the *same* pressure the dashboard traces.
Nothing downstream re-derives it.

Per substep the cylinder does, in order:

  1. Geometry — V(θ) at both edges of the substep (KB Eq 2.5-2.6, tabulated).
  2. Gas exchange — signed compressible flow through each valve, area set by the
     cam lift profile (tabulated, see valve_flow for the law). Positive mass
     flow is into the cylinder, so overlap reversion and scavenging need no
     special case. Each transfer is limited to the mass that would equalise the
     two sides, so an explicit step can never overshoot the pressure it is
     flowing toward.
  3. Open-system first law at constant volume — fresh charge brings its
     enthalpy in, leaving gas takes the cylinder's enthalpy out, the Wiebe burn
     (KB Eq 9.28) releases chemical energy, the Woschni correlation
     (KB Eq 12.16-12.19) takes heat out through the walls.
  4. Reversible compression/expansion across the volume change, applied with the
     c_v of the gas at the mean temperature: T·V^(R/c_v) = const, and the work
     the gas does, W = m·c_v·(T_before − T_after), is the energy actually taken
     out of the gas, so the torque integral can never disagree with the first
     law, however coarse the substep.
  5. Torque — the work over the substep divided by the crank angle swept,
     T = (W − p_crankcase·ΔV)/Δθ (KB Eq 2.11), split into the gross part and
     the pumping loop (the intake and exhaust strokes).

The gas is a two-component mixture: unburned charge and equilibrium burned
products, tracked by the burned mass y. c_v comes from gas_tables, where the
burned gas includes dissociation (KB Fig 4-15), so γ falls from ~1.35 in
compression to ~1.2 at peak temperature and the burn tops out near 2700 K
rather than the ~3400 K an air-like gas would reach.

Fuel accounting. ``begin_cycle(fuel)`` (legacy, used by the unit tests) books the
fuel in one go as chemical energy only, without adding its mass to the gas.
``begin_cycle(fuel, ..., air_mass_ref_kg=...)`` is what the engine uses: the fuel
is delivered *with* the incoming charge in proportion to the air passing the
intake valve, so the gas mass, the equivalence ratio and the pressure all include
it, and reversion carries it back out.

Speed. This function is the hot path of the whole simulator (every cylinder,
every few crank degrees), so it works from precomputed tables — volume, valve
flow area and lift against crank angle, c_v and internal energy against
temperature — and keeps its arithmetic inline. The Woschni, valve-flow and
Wiebe formulas it inlines are cross-checked against their reference
implementations in tests/test_resolved_engine.py.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

from .config import EngineConfig
from .gas_tables import (
    CV_AIR, CV_BURNED, CV_MIX, DUB_AIR, DUB_MIX, GRID_INV, GRID_N, GRID_STEP,
    U_AIR, U_BURNED, U_MIX,
)
from .emissions_model import nox_equilibrium_concentration, nox_formation_rate
from .spark_model import livengood_wu_integral
from .thermodynamics import GAMMA, N_COMPRESSION, R_AIR
from .valve_flow import LIFT_DEADBAND_M
from .valve_profile import valve_profiles_from_config

# Local cycle frame: 0° = TDC at the start of the intake stroke,
# 180° = intake BDC, 360° = firing TDC, 540° = exhaust BDC.
STROKE_HALF_DEG = 180.0
CYCLE_DEG = 720.0

# Bins in the recorded pressure trace (one per 4° of crank angle).
TRACE_BINS = 180
_TRACE_BIN_INV = TRACE_BINS / CYCLE_DEG

_MIN_TEMPERATURE_K = 50.0
_LAST_NODE = GRID_N - 1

# Geometry and valve tables are shared by every cylinder of an engine.
_VOLUME_STEP_INV = 4.0                 # volume table: 0.25° grid
_VOLUME_N = int(CYCLE_DEG * _VOLUME_STEP_INV) + 2
_VALVE_STEP_INV = 2.0                  # valve tables: 0.5° grid
_VALVE_N = int(CYCLE_DEG * _VALVE_STEP_INV)

# Orifice-flow constants (KB Eq 7.18-7.20), γ = GAMMA.
_PR_CRIT = (2.0 / (GAMMA + 1.0)) ** (GAMMA / (GAMMA - 1.0))
_C_CHOKED = math.sqrt(2.0 / R_AIR * GAMMA / (GAMMA + 1.0)
                      * (2.0 / (GAMMA + 1.0)) ** (1.0 / (GAMMA - 1.0)))
_C_SUBSONIC = math.sqrt(2.0 * GAMMA / (R_AIR * (GAMMA - 1.0)))
_E_LOW = 2.0 / GAMMA
_E_HIGH = (GAMMA + 1.0) / GAMMA

# Woschni constants (KB Eq 12.18-12.19)
_C1_GAS_EXCHANGE = 6.18
_C1_COMPRESSION = 2.28
_C2_COMBUSTION = 3.24e-3

# The flow predictor-corrector only pays when a substep moves a real fraction of the
# charge; below this it would change the answer by less than the table error.
_SECOND_ORDER_FRACTION = 0.02
_THIRD_PASS_FRACTION = 0.10

_TABLE_CACHE: Dict[tuple, tuple] = {}


def _fall(x: float, lo: float, hi: float) -> float:
    """Smoothstep from 0 at ``lo`` to 1 at ``hi`` (lo may be above hi: then it falls the other way)."""
    if lo == hi:
        return 0.0 if x < lo else 1.0
    t = (x - lo) / (hi - lo)
    if t <= 0.0:
        return 0.0
    if t >= 1.0:
        return 1.0
    return t * t * (3.0 - 2.0 * t)


def _u_burned(t: float) -> float:
    """Sensible internal energy of equilibrium burned gas at T, relative to 300 K [J/kg]."""
    n = min(_LAST_NODE, int(t * GRID_INV))
    return U_BURNED[n] + (t - n * GRID_STEP) * CV_BURNED[n]


def _u_mix(t: float, y: float, u_unburned, cv_unburned) -> float:
    """Sensible internal energy of unburned charge (with burned fraction y) at T [J/kg]."""
    n = min(_LAST_NODE, int(t * GRID_INV))
    d = t - n * GRID_STEP
    uu = u_unburned[n] + d * cv_unburned[n]
    return uu + (U_BURNED[n] + d * CV_BURNED[n] - uu) * y


def _cv_burned(t: float) -> float:
    n = min(_LAST_NODE, int(t * GRID_INV))
    return CV_BURNED[n]


def _cv_mix(t: float, y: float, cv_unburned) -> float:
    n = min(_LAST_NODE, int(t * GRID_INV))
    return cv_unburned[n] + (CV_BURNED[n] - cv_unburned[n]) * y


def _invert_h_burned(h: float, guess: float) -> float:
    """Temperature at which burned gas has enthalpy h = u + R·T [J/kg] (Newton on the table)."""
    t = guess if guess > 200.0 else 200.0
    for _ in range(4):
        n = min(_LAST_NODE, int(t * GRID_INV))
        cp = CV_BURNED[n] + R_AIR
        t += (h - (U_BURNED[n] + (t - n * GRID_STEP) * CV_BURNED[n] + R_AIR * t)) / cp
        if t < 200.0:
            t = 200.0
        elif t > 5000.0:
            t = 5000.0
    return t


def _orifice(area_cd: float, p_cyl: float, p_port: float, t_cyl: float, t_port: float) -> float:
    """Signed mass flow rate through a valve, positive into the cylinder [kg/s].

    The same relation as valve_flow.valve_mass_flow_signed (KB Eq 7.18-7.20), with
    the discharge coefficient folded into ``area_cd`` and the constants hoisted.
    """
    if p_cyl >= p_port:
        up, dn, tu, sign = p_cyl, p_port, t_cyl, -1.0
    else:
        up, dn, tu, sign = p_port, p_cyl, t_port, 1.0
    pr = dn / up
    if pr <= _PR_CRIT:
        return sign * area_cd * up * _C_CHOKED / math.sqrt(tu)
    b = pr ** _E_LOW - pr ** _E_HIGH
    if b <= 0.0:
        return 0.0
    return sign * area_cd * up * _C_SUBSONIC * math.sqrt(b / tu)


def _geometry_tables(cfg: EngineConfig, intake_valve, exhaust_valve):
    """Volume, valve-area and valve-lift tables for this geometry (cached)."""
    key = (cfg.bore_m, cfg.stroke_m, cfg.rod_length_m, cfg.compression_ratio,
           intake_valve, exhaust_valve,
           cfg.intake_head_dia_m, cfg.intake_port_dia_m, cfg.intake_seat_angle_deg,
           cfg.exhaust_head_dia_m, cfg.exhaust_port_dia_m, cfg.exhaust_seat_angle_deg,
           cfg.intake_valve_count, cfg.exhaust_valve_count,
           cfg.intake_discharge_coeff, cfg.exhaust_discharge_coeff)
    hit = _TABLE_CACHE.get(key)
    if hit is not None:
        return hit
    r = cfg.stroke_m / 2.0
    l = cfg.rod_length_m
    ratio_sq = (r / l) ** 2
    a_p = math.pi / 4.0 * cfg.bore_m ** 2
    vc = cfg.clearance_volume
    volume = []
    for k in range(_VOLUME_N):
        th = math.radians(k / _VOLUME_STEP_INV)
        s = math.sin(th)
        x = (l + r) - r * math.cos(th) - l * math.sqrt(1.0 - ratio_sq * s * s)
        volume.append(vc + a_p * x)

    def valve(profile, head, port, seat, count, cd):
        area_cd, lift = [], []
        a_port = math.pi / 4.0 * port ** 2
        cos_b = math.cos(math.radians(seat))
        for k in range(_VALVE_N):
            lf = profile.lift(k / _VALVE_STEP_INV)
            lift.append(lf)
            if lf <= LIFT_DEADBAND_M:
                area_cd.append(0.0)
            else:
                area_cd.append(cd * count * min(math.pi * head * lf * cos_b, a_port))
        return area_cd, lift

    ai, li = valve(intake_valve, cfg.intake_head_dia_m, cfg.intake_port_dia_m,
                   cfg.intake_seat_angle_deg, cfg.intake_valve_count, cfg.intake_discharge_coeff)
    ae, le = valve(exhaust_valve, cfg.exhaust_head_dia_m, cfg.exhaust_port_dia_m,
                   cfg.exhaust_seat_angle_deg, cfg.exhaust_valve_count, cfg.exhaust_discharge_coeff)
    tables = (volume, ai, li, ae, le)
    _TABLE_CACHE[key] = tables
    return tables


class Cylinder:
    """Gas state of one cylinder, carried continuously across cycles.

    Attributes:
        number: Cylinder number, 1-based.
        mass: Gas mass in the cylinder including fuel [kg].
        temperature: Mean gas temperature [K].
        pressure: Cylinder pressure at the end of the last substep [Pa] — the
            pressure that makes torque.
        fuel_mass: Fuel present in the cylinder (burned + unburned) [kg].
        burned_mass: Mass of burned products in the gas [kg].
        torque: Mean gas torque over the last substep [N·m].
        indicated_torque: Net indicated torque averaged over the last update [N·m].
        pumping_torque: Pumping loss averaged over the last update [N·m], positive.
        gross_torque: Indicated torque excluding pumping [N·m].
        fired_this_cycle: True once combustion has started in the current cycle —
            read by the engine to fire a sound event exactly once per cycle.
        p_trace: Pressure recorded against local crank angle, one value per 4°
            [Pa]; only filled when ``record_trace`` is on (the bank turns it on
            for cylinder 1, which is the one the dashboard plots).
        last_cycle_work_j: Net indicated work of the last complete cycle [J].
        last_cycle_pumping_work_j: Pumping work of the last complete cycle [J]
            (negative on a throttled engine).
        peak_pressure_pa, peak_temperature_k: Peaks of the cycle in progress.
        trapped_air_kg: Fresh air inducted in the current cycle, as of the
            intake valve closing [kg].
    """

    def __init__(self, number: int, cfg: EngineConfig, record_trace: bool = True):
        self.number = number
        self.cfg = cfg
        self.record_trace = record_trace

        # --- Geometry ---
        self.bore = cfg.bore_m
        self.stroke = cfg.stroke_m
        self.rod_length = cfg.rod_length_m
        self.clearance_volume = cfg.clearance_volume
        self.displaced_volume = cfg.displacement_per_cyl
        self.piston_area = math.pi / 4.0 * self.bore ** 2
        self.intake_valve, self.exhaust_valve = valve_profiles_from_config(cfg)
        self._crank_r = self.stroke / 2.0
        self._rod_ratio_sq = (self._crank_r / self.rod_length) ** 2
        self._ivc_base = (self.intake_valve.open_deg + self.intake_valve.duration_deg) % CYCLE_DEG
        self._ivc_deg = self._ivc_base
        # Variable valve timing: cam advance [crank degrees, + = events earlier]. The lift
        # and area tables are indexed at (angle + advance), so a shifted cam costs two adds.
        self._cam_in = 0.0
        self._cam_ex = 0.0
        self._cam_on = False
        (self._vol_tab, self._area_in, self._lift_in,
         self._area_ex, self._lift_ex) = _geometry_tables(cfg, self.intake_valve, self.exhaust_valve)

        # --- Constants hoisted out of the substep ---
        self._wall_t = cfg.wall_temperature_k
        self._k_woschni = 3.26 * self.bore ** -0.2
        self._area_const = math.pi * self.bore ** 2 / 2.0       # chamber area A = π·B²/2 + 4·V/B
        self._area_per_volume = 4.0 / self.bore
        self._stoich = cfg.stoich_afr
        self._lhv_eff = cfg.lower_heating_value * cfg.combustion_efficiency
        self._wiebe_a = cfg.wiebe_a
        self._wiebe_m1 = cfg.wiebe_m + 1.0
        self._knock_on = bool(cfg.use_knock_model)
        self._misfire_q = cfg.burn_misfire_quality
        self.burn_quality = 1.0            # quality of the charge at the last spark (see _burn_quality)
        self.residual_at_spark = 0.0       # burned-gas mass fraction of the charge at the last spark
        self.misfired = False              # the cycle in progress is a misfire
        self.misfire_count = 0
        self._runner_burned = 0.0          # burned gas pushed up the intake runner, waiting to return

        # --- Two-zone burn (see _two_zone_step) ---
        self._two_zone = bool(cfg.use_two_zone)
        self._zone_active = False
        self._mu = self._mb = 0.0          # unburned / burned zone mass [kg]
        self._tu = self._tb = 0.0          # zone temperatures [K]
        self._yu = 0.0                     # burned (residual) fraction inside the unburned zone
        self._m_at_spark = 0.0
        self._phi_cycle = 1.0              # fuel/air equivalence ratio of this cycle's charge
        self.no_mole_fraction = 0.0        # NO in the burned zone (mole fraction), Zeldovich
        self.last_no_ppm = 0.0             # NO of the last burn at the exhaust valve opening
        self.peak_burned_temperature_k = 0.0
        self.last_peak_burned_temperature_k = 0.0
        self.unburned_temperature_k = 0.0  # end-gas temperature while a flame is burning (0 otherwise)
        self._exhaust_base = self.exhaust_valve.open_deg
        self._exhaust_open = self._exhaust_base
        self._edge_a = -1.0e9              # angle and volume at the end of the last substep
        self._edge_v = 0.0
        self._h_c = 0.0                    # Woschni coefficient, refreshed every other quiet substep
        self._h_age = 99
        self._inlet_cache: Tuple[float, float] = (-1.0, 0.0)   # (T_in, u_u(T_in) + R·T_in)

        # --- Thermodynamic state ---
        self.mass = 0.0
        self.burned_mass = 0.0
        self.temperature = cfg.intake_temperature_k
        self.pressure = 0.0
        self.fuel_mass = 0.0
        self._cvu, self._uu, self._dub = CV_AIR, U_AIR, DUB_AIR

        # --- Combustion state ---
        self.spark_angle_deg = 360.0 - cfg.spark_timing_deg_btdc
        self.combustion_duration_deg = cfg.combustion_duration_deg
        self._burn_window = 2.0 * self.combustion_duration_deg + 20.0
        self.fuel_energy_remaining = 0.0
        self._pool_total = 0.0
        self._burned_fraction_prev = 0.0
        self._unburned_at_spark = 0.0
        self._motoring_reference_pa = 0.0
        self._motoring_reference_k = self.temperature
        self._motoring_reference_m3 = self.clearance_volume
        self.combustion_occurred = False
        self.fired_this_cycle = False
        # Fuel delivered with the intake charge (fuel mass per kg of air). None
        # means the legacy whole-cycle booking.
        self._fuel_per_air: Optional[float] = None
        # When fuel rides in with the charge, the energy pool is sized at the
        # spark from the fuel that actually arrived, not from the request.
        self._pool_pending = False
        self.energy_scale = 1.0
        self._last_angle: Optional[float] = None

        # --- Work accumulators (reset each engine update) ---
        self.work_net = 0.0
        self.work_pumping = 0.0
        self.angle_swept = 0.0

        # --- Per-cycle accumulators (reset at each begin_cycle) ---
        self._cycle_work = 0.0
        self._cycle_pumping = 0.0
        self._cycle_heat_loss = 0.0
        self._cycle_heat_release = 0.0
        self.last_cycle_work_j = 0.0
        self.last_cycle_pumping_work_j = 0.0
        self.last_cycle_heat_loss_j = 0.0
        self.last_cycle_heat_release_j = 0.0
        self.peak_pressure_pa = 0.0
        self.peak_temperature_k = 0.0
        self.last_peak_pressure_pa = 0.0
        self.last_peak_temperature_k = 0.0
        self.residual_mass_kg = 0.0
        self.trapped_air_kg = 0.0
        self.trapped_mass_kg = 0.0
        self._fuel_in_cycle = 0.0
        self.cycles_completed = 0
        self.ivc_events = 0
        self.last_reference_pressure_pa = 0.0
        self.last_reference_temperature_k = 0.0

        # --- Knock (Livengood-Wu integral on the unburned end gas) ---
        self.knock_integral = 0.0
        self.knock_detected = False
        self.knock_events = 0

        # --- Outputs averaged over the last update ---
        self.torque = 0.0
        self.indicated_torque = 0.0
        self.pumping_torque = 0.0
        self.gross_torque = 0.0

        # --- Diagnostics ---
        self.heat_released_total = 0.0
        self.heat_loss_total = 0.0
        self.dm_in = 0.0                  # mass through the intake valve in the last substep [kg]
        self.dm_ex = 0.0                  # ... and through the exhaust valve (+ = into the cylinder)
        self._dt = 1.0
        self.wall_heat_transfer_rate = 0.0
        self.burned_fraction = 0.0
        self.p_trace: List[float] = [0.0] * TRACE_BINS

    # ── Variable valve timing ─────────────────────────────────────────────

    def set_cam_phase(self, intake_advance_deg: float, exhaust_advance_deg: float) -> None:
        """Advance the intake and exhaust cams [crank degrees; + = events earlier].

        The whole lift curve moves, so opening, closing and the overlap between the two
        valves all follow. Events the cylinder reports (intake closing, the knock
        window) move with it.
        """
        self._cam_in = float(intake_advance_deg)
        self._cam_ex = float(exhaust_advance_deg)
        self._cam_on = self._cam_in != 0.0 or self._cam_ex != 0.0
        self._ivc_deg = (self._ivc_base - self._cam_in) % CYCLE_DEG
        self._exhaust_open = (self._exhaust_base - self._cam_ex) % CYCLE_DEG

    @property
    def intake_open_deg(self) -> float:
        """Where the intake valve opens now [cycle degrees], cam advance included."""
        return (self.intake_valve.open_deg - self._cam_in) % CYCLE_DEG

    # ── Flows and lifts of the last substep ───────────────────────────────

    @property
    def mass_flow_intake(self) -> float:
        """Mass flow through the intake valve, + = into the cylinder [kg/s]."""
        return self.dm_in / self._dt

    @property
    def mass_flow_exhaust(self) -> float:
        """Mass flow through the exhaust valve, + = into the cylinder [kg/s]."""
        return self.dm_ex / self._dt

    @property
    def lift_intake(self) -> float:
        """Intake valve lift at the last substep's angle [m]."""
        return self._lift_in[int((((self._last_angle or 0.0) + self._cam_in) % 720.0) * 2.0)]

    @property
    def lift_exhaust(self) -> float:
        """Exhaust valve lift at the last substep's angle [m]."""
        return self._lift_ex[int((((self._last_angle or 0.0) + self._cam_ex) % 720.0) * 2.0)]

    # ── Geometry ──────────────────────────────────────────────────────────

    def volume(self, angle_deg: float) -> float:
        """Instantaneous cylinder volume [m^3] at a local cycle angle (exact, untabulated)."""
        th = math.radians(angle_deg)
        s = math.sin(th)
        x = ((self.rod_length + self._crank_r) - self._crank_r * math.cos(th)
             - self.rod_length * math.sqrt(1.0 - self._rod_ratio_sq * s * s))
        return self.clearance_volume + self.piston_area * x

    def volume_rate(self, angle_deg: float) -> float:
        """dV/dθ [m^3/rad] at a local cycle angle."""
        theta = math.radians(angle_deg)
        s, c = math.sin(theta), math.cos(theta)
        disc = math.sqrt(1.0 - self._rod_ratio_sq * s * s)
        r = self._crank_r
        return self.piston_area * (r * s + r * math.sqrt(self._rod_ratio_sq) * s * c / disc)

    # ── Cycle bookkeeping ─────────────────────────────────────────────────

    def begin_cycle(self, fuel_mass_kg: float, spark_angle_deg: float,
                    combustion_duration_deg: float,
                    air_mass_ref_kg: Optional[float] = None) -> None:
        """Start a new cycle: load the fuel and reset combustion.

        Args:
            fuel_mass_kg: Fuel to be delivered to this cylinder this cycle [kg].
            spark_angle_deg: Local cycle angle of the spark [degrees].
            combustion_duration_deg: Wiebe burn duration [degrees].
            air_mass_ref_kg: The air mass the fuel was metered against [kg].
                If given, the fuel is delivered with the intake charge at
                ``fuel_mass_kg / air_mass_ref_kg`` per kg of air, so its mass
                enters the gas. If omitted the whole fuel is booked at once as
                energy only (the legacy path).
        """
        self._close_cycle_books()
        self.knock_integral = 0.0
        self.knock_detected = False
        self.residual_mass_kg = self.mass
        self.trapped_air_kg = 0.0
        self.trapped_mass_kg = 0.0
        self._fuel_in_cycle = 0.0
        fuel_mass_kg = max(0.0, fuel_mass_kg)
        self._pool_pending = False
        self.energy_scale = 1.0
        if air_mass_ref_kg is not None and air_mass_ref_kg > 0.0:
            self._fuel_per_air = fuel_mass_kg / air_mass_ref_kg
            self._pool_pending = True
            self.fuel_energy_remaining = 0.0
        else:
            self._fuel_per_air = None
            self.fuel_mass = fuel_mass_kg
            # Combustion efficiency is applied once, here, by shrinking the pool
            # of releasable energy. Applying it per substep instead would
            # compound it into the energy actually released and hand back more
            # than the fuel had.
            self.fuel_energy_remaining = fuel_mass_kg * self._lhv_eff
        self._pool_total = self.fuel_energy_remaining
        self._set_unburned_table(fuel_mass_kg > 0.0)
        self.spark_angle_deg = spark_angle_deg
        self.combustion_duration_deg = max(1.0, combustion_duration_deg)
        self._burn_window = 2.0 * self.combustion_duration_deg + 20.0
        self._burned_fraction_prev = 0.0
        self._motoring_reference_pa = 0.0
        self._motoring_reference_k = self.temperature
        self._motoring_reference_m3 = self.clearance_volume
        self.combustion_occurred = False
        self.fired_this_cycle = False
        self.misfired = False
        self.burned_fraction = 0.0
        self.peak_pressure_pa = 0.0
        self.peak_temperature_k = 0.0
        if self._zone_active:
            self._merge_zones()
        self.no_mole_fraction = 0.0
        self.peak_burned_temperature_k = 0.0
        self.unburned_temperature_k = 0.0

    def _set_unburned_table(self, with_fuel: bool) -> None:
        """Unburned charge is petrol-air when fuelled, plain air when motoring."""
        if with_fuel:
            self._cvu, self._uu, self._dub = CV_MIX, U_MIX, DUB_MIX
        else:
            self._cvu, self._uu, self._dub = CV_AIR, U_AIR, DUB_AIR

    def _close_cycle_books(self) -> None:
        """Latch the finished cycle's totals before the next one starts."""
        if self._cycle_work != 0.0 or self._cycle_pumping != 0.0 or self.peak_pressure_pa > 0.0:
            self.last_cycle_work_j = self._cycle_work
            self.last_cycle_pumping_work_j = self._cycle_pumping
            self.last_cycle_heat_loss_j = self._cycle_heat_loss
            self.last_cycle_heat_release_j = self._cycle_heat_release
            self.last_peak_pressure_pa = self.peak_pressure_pa
            self.last_peak_temperature_k = self.peak_temperature_k
            self.last_peak_burned_temperature_k = self.peak_burned_temperature_k
            if self._motoring_reference_pa > 0.0:
                self.last_reference_pressure_pa = self._motoring_reference_pa
                self.last_reference_temperature_k = self._motoring_reference_k
            self.cycles_completed += 1
        self._cycle_work = 0.0
        self._cycle_pumping = 0.0
        self._cycle_heat_loss = 0.0
        self._cycle_heat_release = 0.0

    def _burn_quality(self, mass: float, burned_mass: float) -> float:
        """How well the charge at the spark can burn, 0..1 (see EngineConfig burn_*)."""
        cfg = self.cfg
        y = burned_mass / mass if mass > 0.0 else 0.0
        self.residual_at_spark = y
        q = 1.0 - _fall(y, cfg.burn_residual_lo, cfg.burn_residual_hi)
        fuel = self._fuel_in_cycle
        if fuel > 0.0:
            lam = (mass - burned_mass - fuel) / (fuel * self._stoich)
            self._phi_cycle = 1.0 / lam if lam > 0.0 else 1.0
            q = min(q, 1.0 - _fall(lam, cfg.burn_lean_lo, cfg.burn_lean_hi),
                    1.0 - _fall(lam, cfg.burn_rich_lo, cfg.burn_rich_hi))
        return q

    def load_fuel(self, fuel_mass_kg: float, air_mass_ref_kg: float) -> None:
        """Start delivering fuel with the charge, part-way through the intake.

        A port injector can inject any time up to the intake valve closing, so a
        cylinder whose cycle began with no fuel commanded (a fuel cut that has
        just ended, the first cycles of a start) picks fuel up as soon as it is
        wanted instead of waiting out a whole 720° cycle. The fuel that rides in
        is proportional to the air that follows, so a late start is a leaner
        charge, not a wrong one.
        """
        if (self._fuel_per_air is None and fuel_mass_kg > 0.0 and air_mass_ref_kg > 0.0):
            self._fuel_per_air = fuel_mass_kg / air_mass_ref_kg
            self._pool_pending = True
            self._set_unburned_table(True)

    @property
    def fuel_loaded(self) -> bool:
        """True once fuel is being delivered for the current cycle."""
        return self._fuel_per_air is not None

    def reset_accumulators(self) -> None:
        """Zero the work accumulators ahead of one engine update."""
        self.work_net = 0.0
        self.work_pumping = 0.0
        self.angle_swept = 0.0
        self.heat_released_total = 0.0
        self.heat_loss_total = 0.0

    def finalize_update(self) -> None:
        """Turn this update's accumulated work into reported torques."""
        if self.angle_swept <= 0.0:
            self.indicated_torque = 0.0
            self.pumping_torque = 0.0
            self.gross_torque = 0.0
            return
        self.indicated_torque = self.work_net / self.angle_swept
        # The pumping loop is negative on a throttled engine (gas pushed in at
        # manifold pressure, pushed out at backpressure), so negate to report
        # the loss as a positive number and never subtract it a second time.
        self.pumping_torque = -self.work_pumping / self.angle_swept
        self.gross_torque = self.indicated_torque + self.pumping_torque

    # ── Thermodynamic helpers ─────────────────────────────────────────────

    @property
    def equivalence_ratio(self) -> float:
        """Local fuel/air equivalence ratio."""
        if self.mass <= 0.0:
            return 0.0
        return min(3.0, self.fuel_mass * self.cfg.stoich_afr / self.mass)

    @property
    def burned_mass_fraction(self) -> float:
        """Mass fraction of the gas that is burned product, y."""
        return self.burned_mass / self.mass if self.mass > 0.0 else 0.0

    @property
    def mean_effective_pressure_pa(self) -> float:
        """Net IMEP of the last complete cycle [Pa]."""
        return self.last_cycle_work_j / self.displaced_volume

    def initialize_charge(self, mass_kg: float, temperature_k: float,
                          angle_deg: float = 180.0,
                          burned_fraction: float = 0.0) -> None:
        """Seed the cylinder with a charge (used at start-up / first cycle).

        Args:
            mass_kg: Charge mass to place in the cylinder [kg].
            temperature_k: Charge temperature [K].
            angle_deg: Local cycle angle the charge is trapped at [degrees].
                BDC (180) is the meaningful default: a charge is trapped at its
                largest volume, so seeding at TDC would imply a pressure several
                bar higher than the manifold and would blow the charge back out
                through the still-open intake valve.
            burned_fraction: Mass fraction of the charge that is burned product.
        """
        self.mass = max(0.0, mass_kg)
        self.burned_mass = self.mass * min(1.0, max(0.0, burned_fraction))
        self.temperature = max(_MIN_TEMPERATURE_K, temperature_k)
        self.pressure = self.mass * R_AIR * self.temperature / max(
            self.volume(angle_deg), 1.0e-9)

    # ── Integration ───────────────────────────────────────────────────────

    def step(self, angle_deg: float, dtheta_deg: float, dt: float,
             rpm: float, p_intake: float, t_intake: float,
             p_exhaust: float, t_exhaust: float,
             p_crankcase: float = 0.0,
             wall_temperature_k: Optional[float] = None) -> None:
        """Advance this cylinder by one crank-angle substep.

        Args:
            angle_deg: Local cycle angle at the substep midpoint [degrees]
                (0 = intake TDC, 360 = firing TDC).
            dtheta_deg: Crank angle swept in this substep [degrees].
            dt: Time step [s].
            rpm: Engine speed [rev/min].
            p_intake: Intake runner pressure at the intake valve [Pa].
            t_intake: Intake runner temperature [K].
            p_exhaust: Exhaust plenum pressure at the exhaust valve [Pa].
            t_exhaust: Exhaust plenum temperature [K].
            p_crankcase: Crankcase pressure [Pa].
            wall_temperature_k: Wall temperature for heat loss [K].
        """
        if dt <= 0.0 or dtheta_deg <= 0.0:
            return

        R = R_AIR
        angle = angle_deg % 720.0
        half = 0.5 * dtheta_deg

        # 1. Volumes at the two edges of the substep (tabulated, linear): the
        # swept volume dV, and their mean is the volume the gas lives in.
        vt = self._vol_tab
        a0 = angle - half
        d_edge = self._edge_a - a0
        if -1e-9 < d_edge < 1e-9:
            v0 = self._edge_v                  # the previous substep ended where this one starts
        else:
            x = (a0 % 720.0) * 4.0
            i = int(x)
            v0 = vt[i] + (x - i) * (vt[i + 1] - vt[i])
        a1 = angle + half
        x = (a1 % 720.0) * 4.0
        i = int(x)
        v1 = vt[i] + (x - i) * (vt[i + 1] - vt[i])
        self._edge_a = a1
        self._edge_v = v1
        volume = 0.5 * (v0 + v1)
        dv = v1 - v0

        mass = self.mass
        temperature = self.temperature
        # The state (m, T) belongs to the *start* edge of this substep.
        pressure = mass * R * temperature / v0 if mass > 0.0 else 0.0

        # Intake valve closing: latch what the charge was.
        last = self._last_angle
        if last is not None:
            ivc = self._ivc_deg
            if (last < ivc <= angle) if angle >= last else (ivc > last or ivc <= angle):
                self.trapped_mass_kg = mass
                self._runner_burned = 0.0       # whatever is still in the runner stays out
                # Fresh air = the unburned part of the charge, less the fuel in it.
                # (Counting the burned residual by its mass at the start of the cycle
                # would call exhaust gas pulled back in during overlap "air".)
                unburned = mass - self.burned_mass
                self.trapped_air_kg = max(0.0, unburned - self._fuel_in_cycle)
                self.ivc_events += 1
        self._last_angle = angle

        # 2. Valve areas (Cd·A, tabulated) and flows. Signed: + = into the cylinder.
        if self._cam_on:
            a_in = self._area_in[int(((angle + self._cam_in) % 720.0) * 2.0)]
            a_ex = self._area_ex[int(((angle + self._cam_ex) % 720.0) * 2.0)]
        else:
            ia = int(angle * 2.0)
            a_in = self._area_in[ia]
            a_ex = self._area_ex[ia]
        gas_exchange = a_in > 0.0 or a_ex > 0.0
        if self._zone_active and gas_exchange:
            # A valve has opened with the flame still burning (a very late spark):
            # merge the zones into one gas, conserving energy, and carry on single-zone.
            self._merge_zones()
            temperature = self.temperature
            pressure = mass * R * temperature / v0 if mass > 0.0 else 0.0

        # Gas c_v at the start state: unburned/burned mix by burned mass fraction.
        burned0 = self.burned_mass
        y0 = burned0 / mass if mass > 0.0 else 0.0
        node = int(temperature * GRID_INV)
        if node > _LAST_NODE:
            node = _LAST_NODE
        cvu = self._cvu
        cv0 = cvu[node] + (CV_BURNED[node] - cvu[node]) * y0

        # 3b. Wall heat loss (Woschni, KB Eq 12.16-12.19), inlined. h_c moves slowly
        # outside the burn and the gas exchange, so there it is refreshed every
        # other substep and reused in between.
        q_wall = 0.0
        if mass > 0.0:
            comb = self.combustion_occurred
            if comb or gas_exchange or self._h_age >= 1:
                p_ref = self._motoring_reference_pa
                w = (_C1_GAS_EXCHANGE if gas_exchange else _C1_COMPRESSION) * self.stroke * rpm * (1.0 / 30.0)
                if comb and not gas_exchange and p_ref > 0.0 and pressure > 0.0:
                    # p_motored: the polytrope through the state at the spark,
                    # p_m = p_r·(V_r/V)^n — rises as the piston keeps compressing
                    # after the spark and falls with the volume through the expansion.
                    v_ref = self._motoring_reference_m3
                    dp = pressure - p_ref * (v_ref / volume) ** N_COMPRESSION
                    if dp > 0.0:
                        w += (_C2_COMBUSTION * self.displaced_volume * self._motoring_reference_k
                              / (p_ref * v_ref)) * dp
                self._h_c = (self._k_woschni * (pressure * 1.0e-3) ** 0.8
                             * (temperature if temperature > 300.0 else 300.0) ** -0.55 * w ** 0.8)
                self._h_age = 0
            else:
                self._h_age += 1
            wall_t = self._wall_t if wall_temperature_k is None else wall_temperature_k
            q_wall = self._h_c * (self._area_const + self._area_per_volume * volume) * (temperature - wall_t)
        self.wall_heat_transfer_rate = q_wall

        # Quiet substep: both valves shut and no burn due. Nothing crosses the
        # boundary and the composition cannot change, so only the walls and the
        # volume change act on the gas — a much shorter path through the physics.
        if (a_in == 0.0 and a_ex == 0.0 and mass > 0.0 and not self._zone_active
                and not ((self._pool_pending or self.fuel_energy_remaining > 0.0)
                         and 0.0 < angle - self.spark_angle_deg < self._burn_window)):
            self.dm_in = 0.0
            self.dm_ex = 0.0
            self._dt = dt
            t1 = temperature - q_wall * dt / (mass * cv0) if q_wall != 0.0 else temperature
            if t1 < _MIN_TEMPERATURE_K:
                t1 = _MIN_TEMPERATURE_K
            work_gas = 0.0
            if dv != 0.0:
                ln_r = math.log(v0 / v1)
                n_1 = int(t1 * GRID_INV)
                if n_1 > _LAST_NODE:
                    n_1 = _LAST_NODE
                cv_s = cvu[n_1] + (CV_BURNED[n_1] - cvu[n_1]) * y0
                if t1 > 1800.0:
                    # Hot burned gas: c_v changes quickly with T, so use the mean over the step.
                    n_g = int(0.5 * (t1 + t1 * math.exp(R * ln_r / cv_s)) * GRID_INV)
                    if n_g > _LAST_NODE:
                        n_g = _LAST_NODE
                    cv_s = cvu[n_g] + (CV_BURNED[n_g] - cvu[n_g]) * y0
                t2 = t1 * math.exp(R * ln_r / cv_s)
                if t2 < _MIN_TEMPERATURE_K:
                    t2 = _MIN_TEMPERATURE_K
                work_gas = mass * cv_s * (t1 - t2)
                t1 = t2
            self.temperature = t1
            pressure_end = mass * R * t1 / v1
        else:
            dm_in = dm_ex = 0.0
            m_run = mass
            if a_in > 0.0 or a_ex > 0.0:
                v_eq = v0 / (R * temperature)
                # Predictor for the second-order flow: the state the cylinder would
                # reach at the end of the substep from the volume change alone.
                ln_r = math.log(v0 / v1) if dv != 0.0 else 0.0
                t_pred = temperature * math.exp(R * ln_r / cv0)
                if a_in > 0.0 and p_intake > 0.0:
                    md1 = _orifice(a_in, pressure, p_intake, temperature, t_intake)
                    dm = md1 * dt
                    if dm > _SECOND_ORDER_FRACTION * m_run or -dm > _SECOND_ORDER_FRACTION * m_run:
                        p_end = (m_run + dm) * R * t_pred / v1
                        dm = 0.5 * (md1 + _orifice(a_in, p_end, p_intake, t_pred, t_intake)) * dt
                        if dm > _THIRD_PASS_FRACTION * m_run or -dm > _THIRD_PASS_FRACTION * m_run:
                            # A big transfer (blowdown): iterate the corrector once more.
                            p_end = (m_run + dm) * R * t_pred / v1
                            dm = 0.5 * (md1 + _orifice(a_in, p_end, p_intake, t_pred, t_intake)) * dt
                    # Equalisation limit: never carry the cylinder past the port pressure.
                    m_eq = p_intake * v_eq
                    if dm > 0.0:
                        lim = m_eq - m_run
                        dm = dm if dm < lim else (lim if lim > 0.0 else 0.0)
                    else:
                        lim = m_run - m_eq
                        dm = dm if -dm < lim else (-lim if lim > 0.0 else 0.0)
                    dm_in = dm
                    m_run += dm
                if a_ex > 0.0 and p_exhaust > 0.0:
                    md1 = _orifice(a_ex, pressure, p_exhaust, temperature, t_exhaust)
                    dm = md1 * dt
                    if dm > _SECOND_ORDER_FRACTION * m_run or -dm > _SECOND_ORDER_FRACTION * m_run:
                        p_end = (m_run + dm) * R * t_pred / v1
                        dm = 0.5 * (md1 + _orifice(a_ex, p_end, p_exhaust, t_pred, t_exhaust)) * dt
                        if dm > _THIRD_PASS_FRACTION * m_run or -dm > _THIRD_PASS_FRACTION * m_run:
                            # A big transfer (blowdown): iterate the corrector once more.
                            p_end = (m_run + dm) * R * t_pred / v1
                            dm = 0.5 * (md1 + _orifice(a_ex, p_end, p_exhaust, t_pred, t_exhaust)) * dt
                    m_eq = p_exhaust * v_eq
                    if dm > 0.0:
                        lim = m_eq - m_run
                        dm = dm if dm < lim else (lim if lim > 0.0 else 0.0)
                    else:
                        lim = m_run - m_eq
                        dm = dm if -dm < lim else (-lim if lim > 0.0 else 0.0)
                    dm_ex = dm
            self.dm_in = dm_in
            self.dm_ex = dm_ex
            self._dt = dt

            # 3a. Heat release from the Wiebe curve (KB Eq 9.28).
            q_chem = 0.0
            d_burned_mass = 0.0
            dx_step = 0.0
            if self._pool_pending or self.fuel_energy_remaining > 0.0:
                since = angle - self.spark_angle_deg
                # The window is bounded so a cycle that begins late in the cycle
                # frame (the intake valve opens at ~710°, after this cycle's spark
                # angle has gone by on the circle) cannot read that as a spark that
                # happened a moment ago. A burn is over well inside twice its
                # duration; the load simply waits for the next pass of the spark.
                if 0.0 < since < self._burn_window:
                    if self._pool_pending:
                        # The spark has come: size the pool from the fuel that arrived,
                        # scaled by how well this charge can burn.
                        q = self._burn_quality(mass, burned0)
                        self.burn_quality = q
                        if q < self._misfire_q:
                            q = 0.0
                            self.misfire_count += 1
                            self.misfired = True
                        self._pool_total = self._fuel_in_cycle * self._lhv_eff * self.energy_scale * q
                        self.fuel_energy_remaining = self._pool_total
                        self._pool_pending = False
                    dur = self.combustion_duration_deg
                    if since >= dur:
                        xb = 1.0
                    else:
                        xb = 1.0 - math.exp(-self._wiebe_a * (since / dur) ** self._wiebe_m1)
                    dx = xb - self._burned_fraction_prev
                    if dx > 0.0:
                        if not self.combustion_occurred:
                            self.combustion_occurred = True
                            self.fired_this_cycle = True
                            # Snapshot the cylinder state at the spark, before any
                            # release: this is Woschni's p_r / T_r / V_r reference
                            # state (KB Eq 12.18) and the start of the motored curve.
                            self._motoring_reference_pa = pressure
                            self._motoring_reference_k = temperature
                            self._motoring_reference_m3 = v0
                            self._unburned_at_spark = mass - burned0
                        # Each substep releases its share of the *whole* charge's
                        # energy, dx·Q, so the heat release follows the Wiebe curve
                        # and the energy sums to exactly Q over the burn. (An earlier
                        # revision multiplied by the energy *remaining*, which
                        # released only 1 − e^(−x) of it smoothly and dumped the
                        # last third in a single step at the end of the burn.)
                        q_chem = dx * self._pool_total
                        if q_chem > self.fuel_energy_remaining:
                            q_chem = self.fuel_energy_remaining
                        self.fuel_energy_remaining -= q_chem
                        d_burned_mass = dx * self._unburned_at_spark
                        dx_step = dx
                        self._burned_fraction_prev = xb
                        self.burned_fraction = xb
                        self.heat_released_total += q_chem
                        self._cycle_heat_release += q_chem

            # A flame with both valves shut is solved as two zones (burned / unburned) at
            # a common pressure; everything else is the single-zone first law below.
            if (self._two_zone and dm_in == 0.0 and dm_ex == 0.0
                    and (self._zone_active or q_chem > 0.0)):
                work_gas, pressure_end = self._two_zone_step(
                    dt, v0, v1, q_chem, dx_step, q_wall, temperature, pressure, mass,
                    self._wall_t if wall_temperature_k is None else wall_temperature_k)
            else:
                # 4. First law at constant volume (flow + chemistry + wall), in the form
                #    m'·c_v·ΔT = Σ dm·(h − u) + Q − Q_wall − m'·Δu_composition,
                #    which needs u only where mass arrives (its enthalpy vs the gas it joins).
                dm_fuel = 0.0
                dm_ret = 0.0
                if dm_in < 0.0:
                    # Gas pushed back up the intake runner is burned gas (overlap reversion);
                    # it waits there and is the first thing drawn back in, so it comes back
                    # as residual instead of vanishing into the manifold.
                    self._runner_burned += -dm_in * y0
                elif dm_in > 0.0 and self._runner_burned > 0.0:
                    dm_ret = dm_in if dm_in < self._runner_burned else self._runner_burned
                    self._runner_burned -= dm_ret
                fpa = self._fuel_per_air
                if fpa is not None and dm_in > 0.0:
                    # Fuel is metered against the fresh air that comes in through the intake
                    # valve; nothing is delivered by backflow through the exhaust, and the
                    # burned gas returning from the runner carries none.
                    dm_fuel = fpa * (dm_in - dm_ret)
                    self._fuel_in_cycle += dm_fuel
                dm_gas = dm_in + dm_ex
                mass_new = mass + dm_gas + dm_fuel
                if mass_new < 0.0:
                    mass_new = 0.0
                work_gas = 0.0
                if mass_new > 0.0:
                    # Burned mass: gas that leaves takes its share; combustion makes
                    # more; back-flow from the exhaust is burned gas, fresh charge is not.
                    bm = burned0
                    dm_out = (dm_in if dm_in < 0.0 else 0.0) + (dm_ex if dm_ex < 0.0 else 0.0)
                    if dm_out < 0.0 and mass > 0.0:
                        keep = 1.0 + dm_out / mass
                        bm = bm * keep if keep > 0.0 else 0.0
                    if dm_ex > 0.0:
                        bm += dm_ex
                    bm += dm_ret + d_burned_mass
                    if bm > mass_new:
                        bm = mass_new
                    y1 = bm / mass_new

                    # Σ dm·(h_in − u_old) for arriving mass; leaving gas costs R·T each kg.
                    uu = self._uu
                    num = q_chem - q_wall * dt
                    u_old = 0.0
                    if dm_in > 0.0 or dm_ex > 0.0 or dm_fuel > 0.0:
                        u_u = uu[node] + (temperature - node * GRID_STEP) * cvu[node]
                        u_b = U_BURNED[node] + (temperature - node * GRID_STEP) * CV_BURNED[node]
                        u_old = u_u + (u_b - u_u) * y0
                    if dm_in > 0.0 or dm_fuel > 0.0:
                        cache = self._inlet_cache
                        if cache[0] != t_intake:
                            n_in = int(t_intake * GRID_INV)
                            if n_in > _LAST_NODE:
                                n_in = _LAST_NODE
                            cache = (t_intake, uu[n_in] + (t_intake - n_in * GRID_STEP) * cvu[n_in] + R * t_intake)
                            self._inlet_cache = cache
                        num += (dm_in - dm_ret + dm_fuel) * (cache[1] - u_old)
                        if dm_ret > 0.0:
                            # Returning runner gas is burned gas that has cooled on the way.
                            t_r = t_intake + 0.35 * (temperature - t_intake)
                            n_r = int(t_r * GRID_INV)
                            if n_r > _LAST_NODE:
                                n_r = _LAST_NODE
                            num += dm_ret * (U_BURNED[n_r] + (t_r - n_r * GRID_STEP) * CV_BURNED[n_r]
                                             + R * t_r - u_old)
                    elif dm_in < 0.0:
                        num += dm_in * R * temperature
                    if dm_ex > 0.0:
                        n_e = int(t_exhaust * GRID_INV)
                        if n_e > _LAST_NODE:
                            n_e = _LAST_NODE
                        num += dm_ex * (U_BURNED[n_e] + (t_exhaust - n_e * GRID_STEP) * CV_BURNED[n_e]
                                        + R * t_exhaust - u_old)
                    elif dm_ex < 0.0:
                        num += dm_ex * R * temperature
                    if y1 != y0:
                        num -= mass_new * (y1 - y0) * self._dub[node]

                    # c_v at the temperature the step passes through (mid-point).
                    denom = mass_new * cv0
                    t_mid = temperature + 0.5 * num / denom
                    n_m = int((t_mid if t_mid > 0.0 else 0.0) * GRID_INV)
                    if n_m > _LAST_NODE:
                        n_m = _LAST_NODE
                    y_m = 0.5 * (y0 + y1)
                    cv_m = cvu[n_m] + (CV_BURNED[n_m] - cvu[n_m]) * y_m
                    t1 = temperature + num / (mass_new * cv_m)
                    if t1 < _MIN_TEMPERATURE_K:
                        t1 = _MIN_TEMPERATURE_K

                    # 5. Reversible compression / expansion across the volume change,
                    #    with c_v at the mean temperature of the process.
                    if dv != 0.0:
                        ln_r = math.log(v0 / v1)
                        n_1 = int(t1 * GRID_INV)
                        if n_1 > _LAST_NODE:
                            n_1 = _LAST_NODE
                        cv_g = cvu[n_1] + (CV_BURNED[n_1] - cvu[n_1]) * y1
                        t_guess = t1 * math.exp(R * ln_r / cv_g)
                        n_g = int(0.5 * (t1 + t_guess) * GRID_INV)
                        if n_g > _LAST_NODE:
                            n_g = _LAST_NODE
                        cv_s = cvu[n_g] + (CV_BURNED[n_g] - cvu[n_g]) * y1
                        t2 = t1 * math.exp(R * ln_r / cv_s)
                        if t2 < _MIN_TEMPERATURE_K:
                            t2 = _MIN_TEMPERATURE_K
                        work_gas = mass_new * cv_s * (t1 - t2)
                        t1 = t2
                    self.temperature = t1
                    self.burned_mass = bm
                    if fpa is not None:
                        # Fuel leaves with the gas it is mixed into; fuel that rode in and
                        # is pushed back out (reversion in compression) is not trapped, so
                        # it must not fuel the burn.
                        if dm_out < 0.0 and mass > 0.0:
                            keep = 1.0 + dm_out / mass
                            if keep < 0.0:
                                keep = 0.0
                            self.fuel_mass *= keep
                            self._fuel_in_cycle *= keep
                        self.fuel_mass += dm_fuel
                    self.mass = mass_new
                    pressure_end = mass_new * R * t1 / v1
                else:
                    self.mass = 0.0
                    self.burned_mass = 0.0
                    self.temperature = temperature if temperature > _MIN_TEMPERATURE_K else _MIN_TEMPERATURE_K
                    pressure_end = 0.0

        # 6. Torque and work. Gas pressure on the crown drives the crank; a
        #    crankcase above atmospheric would subtract, so the difference is used.
        work = work_gas - p_crankcase * dv
        dtheta_rad = dtheta_deg * 0.017453292519943295
        self.torque = work / dtheta_rad
        self.work_net += work
        self._cycle_work += work
        # The pumping loop is the intake and exhaust strokes, where the piston
        # moves against a pressure it did not create. Compression and expansion
        # have dV/dθ of each sign but zero net volume change, so they contribute
        # nothing to this integral.
        if angle < 180.0 or angle > 540.0:
            self.work_pumping += work
            self._cycle_pumping += work
        self.angle_swept += dtheta_rad
        if q_wall > 0.0:
            lost = q_wall * dt
            self.heat_loss_total += lost
            self._cycle_heat_loss += lost

        # Pressure at the end of the step, for telemetry and the trace.
        self.pressure = pressure_end
        if pressure_end > self.peak_pressure_pa:
            self.peak_pressure_pa = pressure_end
        if self.temperature > self.peak_temperature_k:
            self.peak_temperature_k = self.temperature
        if self._knock_on and self.fuel_mass > 0.0:
            self._accumulate_knock(angle, dt)
        if self.record_trace:
            self.p_trace[int(angle * _TRACE_BIN_INV)] = pressure_end

    # ── Two-zone burn ─────────────────────────────────────────────────────

    def _merge_zones(self) -> None:
        """Collapse the two zones into one gas, conserving energy, and leave zone mode."""
        mu, mb = self._mu, self._mb
        m = mu + mb
        self._zone_active = False
        self.unburned_temperature_k = 0.0
        if m <= 0.0:
            return
        yu = self._yu
        u_total = mu * _u_mix(self._tu, yu, self._uu, self._cvu) + mb * _u_burned(self._tb)
        y = (mb + yu * mu) / m
        t = (mu * self._tu + mb * self._tb) / m
        for _ in range(3):
            n = min(_LAST_NODE, int(t * GRID_INV))
            cv = self._cvu[n] + (CV_BURNED[n] - self._cvu[n]) * y
            t += (u_total / m - _u_mix(t, y, self._uu, self._cvu)) / cv
        self.temperature = max(_MIN_TEMPERATURE_K, t)
        self.burned_mass = mb + yu * mu
        self.mass = m

    def _two_zone_step(self, dt: float, v0: float, v1: float, q_chem: float, dx: float,
                       q_wall: float, t_mean: float, pressure: float, mass: float,
                       wall_t: float) -> tuple:
        """One substep of the flame as two zones at a common pressure.

        The *unburned* zone is the charge not yet reached by the flame and the *burned*
        zone is everything behind it. Each substep:

          1. the wall takes heat from each zone in proportion to its excess temperature;
          2. the flame converts dm = dx·m₀ of unburned gas into products, releasing
             q_chem: the adiabatic flame temperature follows from
             u_b(T_f) = u_u(T_u) + q/dm, and the new products mix into the burned zone
             at constant energy;
          3. both zones are compressed or expanded isentropically from the old pressure
             to the one at which they just fill the new volume, V_u(p) + V_b(p) = V.
             c_v is taken at each zone's mean temperature over the step.

        The work done on the piston is whatever the first law leaves: the energy of
        both zones before (plus the heat released, minus the heat lost) less their
        energy after. Because the unburned zone is only compressed — never burned and
        re-expanded — its temperature is the true end-gas temperature, which is what
        the Livengood-Wu knock integral needs; the burned zone's temperature, not
        the mean, is what NO forms at.

        Returns (work done by the gas on the piston [J], pressure at the end [Pa]).
        """
        R = R_AIR
        if not self._zone_active:
            self._zone_active = True
            self._mu, self._mb = mass, 0.0
            self._tu = self._tb = t_mean
            self._yu = self.burned_mass / mass if mass > 0.0 else 0.0
            self._m_at_spark = mass
        mu, mb, tu, tb, yu = self._mu, self._mb, self._tu, self._tb, self._yu
        cvu = self._cvu
        uu_tab = self._uu
        u_before = ((mu * _u_mix(tu, yu, uu_tab, cvu) if mu > 0.0 else 0.0)
                    + (mb * _u_burned(tb) if mb > 0.0 else 0.0))
        q_lost = 0.0

        # 1. Heat lost to the walls, shared by excess temperature (taken at the
        #    reference pressure, so through c_p).
        if q_wall != 0.0:
            wu = mu * (tu - wall_t) if tu > wall_t else 0.0
            wb = mb * (tb - wall_t) if tb > wall_t else 0.0
            tot = wu + wb
            if tot > 0.0:
                qw = q_wall * dt / tot
                q_lost = q_wall * dt
                if mu > 0.0 and wu > 0.0:
                    tu -= qw * wu / (mu * (_cv_mix(tu, yu, cvu) + R))
                if mb > 0.0 and wb > 0.0:
                    tb -= qw * wb / (mb * (_cv_burned(tb) + R))
        # 2. The flame front: dm of unburned gas becomes products at the flame temperature.
        dm_b = dx * self._m_at_spark
        if dm_b > mu:
            dm_b = mu
        if dm_b > 0.0:
            # Burning at the reference pressure is an enthalpy balance on each kg:
            # h_b(T_f) = h_u(T_u) + q/dm, with h = u + R·T.
            h_f = _u_mix(tu, yu, uu_tab, cvu) + R * tu + q_chem / dm_b
            t_f = _invert_h_burned(h_f, tu + q_chem / (dm_b * (_cv_burned(tu) + R)))
            if mb > 0.0:
                h_mix = (mb * (_u_burned(tb) + R * tb) + dm_b * h_f) / (mb + dm_b)
                tb = _invert_h_burned(h_mix, (mb * tb + dm_b * t_f) / (mb + dm_b))
                self.no_mole_fraction *= mb / (mb + dm_b)      # fresh products carry no NO yet
            else:
                tb = t_f
            mb += dm_b
            mu -= dm_b

        # 3. Common pressure: isentropic compression/expansion of both zones to volume v1.
        #    Solved twice: the second sweep uses c_v at each zone's mean temperature.
        p0 = pressure
        tu1, tb1, p1 = tu, tb, p0
        cv_u = _cv_mix(tu, yu, cvu) if mu > 0.0 else 1.0
        cv_b = _cv_burned(tb) if mb > 0.0 else 1.0
        for sweep in range(2):
            ku = R / (cv_u + R)                     # (γ − 1)/γ of each zone
            kb = R / (cv_b + R)
            ln_p = math.log(p0)
            p1 = p0
            for _ in range(4):
                p_ratio = p1 / p0
                vu = mu * R * tu * p_ratio ** ku / p1 if mu > 0.0 else 0.0
                vb = mb * R * tb * p_ratio ** kb / p1 if mb > 0.0 else 0.0
                vol = vu + vb
                kappa = (vu * ku + vb * kb) / vol if vol > 0.0 else 0.0
                ln_p += math.log(vol / v1) / (1.0 - kappa)       # d ln V/d ln p = κ − 1
                p1 = math.exp(ln_p)
            tu1 = tu * (p1 / p0) ** ku if mu > 0.0 else tu
            tb1 = tb * (p1 / p0) ** kb if mb > 0.0 else tb
            cv_u_used, cv_b_used = cv_u, cv_b
            if sweep == 0:
                if mu > 0.0:
                    cv_u = _cv_mix(0.5 * (tu + tu1), yu, cvu)
                if mb > 0.0:
                    cv_b = _cv_burned(0.5 * (tb + tb1))
        # Work on the piston: what the first law leaves of the energy the zones had,
        # plus the heat released, minus the heat lost, less the energy they end with.
        u_after = ((mu * _u_mix(tu1, yu, uu_tab, cvu) if mu > 0.0 else 0.0)
                   + (mb * _u_burned(tb1) if mb > 0.0 else 0.0))
        work = u_before + q_chem - q_lost - u_after
        self._mu, self._mb, self._tu, self._tb = mu, mb, tu1, tb1

        # 4. NO forms in the burned zone at its own temperature (extended Zeldovich).
        if mb > 0.0 and tb1 > 1700.0:
            phi = self._phi_cycle
            x_o2 = max(0.003, 0.20 * (1.0 - phi)) if phi < 1.0 else 0.003
            d_no = nox_formation_rate(tb1, p1, x_o2, 0.72, dt)       # mol/cm³
            conc = (p1 / 101325.0) / (82.057 * tb1)
            x_eq = nox_equilibrium_concentration(tb1, p1, x_o2, 0.72)
            self.no_mole_fraction = min(x_eq, self.no_mole_fraction + d_no / conc)
        if tb1 > self.peak_burned_temperature_k:
            self.peak_burned_temperature_k = tb1

        # State for the rest of the machine: mass-mean temperature, total burned mass.
        m = mu + mb
        if mu <= 1.0e-9 * m:
            self._zone_active = False
            self.temperature = tb1
            self.burned_mass = m
            self.unburned_temperature_k = 0.0
            self.last_no_ppm = self.no_mole_fraction * 1.0e6
        else:
            self.temperature = (mu * tu1 + mb * tb1) / m
            self.burned_mass = mb + yu * mu
            self.unburned_temperature_k = tu1
        return work, p1

    def _accumulate_knock(self, angle_deg: float, dt: float) -> None:
        """Livengood-Wu integral on the unburned end gas (KB Eq 9.60-9.64).

        Between intake closing and exhaust opening the end gas is compressed
        isentropically by the rising pressure: before the spark that is just the
        bulk charge, afterwards T_u = T_r·(p/p_r)^((n−1)/n) from the state at the
        spark. The integral stops accumulating once the flame has consumed
        most of the charge, and a knock is flagged when it reaches 1.
        """
        if not (self._ivc_deg <= angle_deg < self._exhaust_open):
            return
        if self.burned_fraction > 0.9 or self.mass <= 0.0:
            return
        if self._zone_active:
            t_u = self.unburned_temperature_k          # the end gas itself (two-zone burn)
        elif self.combustion_occurred and self._motoring_reference_pa > 0.0:
            # Single-zone fallback: compress the state at the spark isentropically.
            t_u = self._motoring_reference_k * (
                self.pressure / self._motoring_reference_pa) ** ((N_COMPRESSION - 1.0) / N_COMPRESSION)
        else:
            t_u = self.temperature
        self.knock_integral += livengood_wu_integral(
            t_u, self.pressure, dt, self.cfg.fuel_octane_ron)
        if self.knock_integral >= 1.0 and not self.knock_detected:
            self.knock_detected = True
            self.knock_events += 1
