"""Engine — the core rotational dynamics simulation with thermodynamic cycle.

This v0.2 replaces the torque-curve lookup with a full thermodynamic model:
    throttle → intake pressure → trapped mass → fuel mass →
    Otto cycle work → indicated torque → friction → net torque →
    crankshaft dynamics → RPM

Governing equation:
    (I_E + I_L) dω/dt = T_indicated - T_friction - T_pumping - T_accessory - T_load

Source: ENGINE_KB.md Eq 14.52, Eq 13.5, Eq 15.2, Part XXIV.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional

from .config import EngineConfig
from .state import EngineState, AirState, FuelState, CombustionState, FrictionState
from .thermodynamics import R_AIR, N_COMPRESSION, N_EXPANSION
from .piston_kinematics import cylinder_volume
from .air_model import (
    volumetric_efficiency, intake_pressure, trapped_mass
)
from .fuel_model import (
    fuel_mass_per_cycle, lambda_from_afr, equivalence_ratio
)
from .combustion_model import cycle_averaged_torque, otto_cycle_work, wiebe_mass_fraction_burned

# New v0.3 models
from .gas_properties import cp_air, cv_air, gamma_air, cp_burned, cv_burned
from .manifold_model import (
    throttle_flow_rate, throttle_effective_area, manifold_pressure_dynamics,
)
from .spark_model import mbt_timing, combustion_duration, livengood_wu_integral
from .heat_transfer import woschni_heat_transfer_coeff, heat_transfer_rate, chamber_surface_area
from .fuel_film import FuelFilmState, fuel_film_dynamics, fuel_film_tau, fuel_film_x
from .lambda_controller import LambdaController
from .cyclic_variability import cyclic_variability_cov
from .turbo_model import TurboState, compressor_power, turbine_power, turbo_shaft_dynamics, wastegate_control
from .emissions_model import nox_emission_index, hc_emissions_fraction
from .friction_breakdown import total_friction_breakdown
from .diesel_combustion import ignition_delay, watson_heat_release, diesel_combustion_duration, premixed_fraction
from .cylinder_bank import CylinderBank
from .native_core import NativeCylinderBank, native_available


@dataclass
class EngineInput:
    """Engine inputs for one timestep.

    load_torque may be negative: that is the driveline motoring the engine
    (e.g. a coasting vehicle in gear), which is what engine braking is.
    """
    throttle: float = 0.0
    load_torque: float = 0.0
    ignition: bool = True     # False = no spark/fuel, engine only motors
    starter: bool = False     # True = starter motor engaged
    vvt_intake_deg: Optional[float] = None    # override the ECU's intake cam advance [crank deg]
    vvt_exhaust_deg: Optional[float] = None   # override the exhaust cam advance


# Degrees per sample when integrating wall heat transfer over the cycle. The
# legacy cycle-averaged model has no crank-resolved state, so the gas state is
# reconstructed from the air-standard Otto polytropes at this resolution.
_HEAT_LOSS_SAMPLES = 90


def _integrated_wall_heat_loss(cfg: EngineConfig, cycle: dict,
                               p_intake: float, T_intake: float,
                               omega: float, time_per_cycle: float,
                               model: str = "woschni") -> float:
    """Wall heat loss per 4-stroke cycle for one cylinder [J].

    Wall heat transfer is an instantaneous rate that varies by orders of
    magnitude over the cycle: near zero during gas exchange, highest during
    combustion and early expansion. Charging the peak rate against the whole
    cycle duration over-counts the loss by the peak-to-mean ratio, so it is
    integrated over crank angle instead.

    The gas state is reconstructed from the same air-standard Otto polytropes
    ``otto_cycle_work`` uses (KB §4.2), evaluated at the current volume, with
    the Wiebe curve spreading the heat release so the fired part of the cycle
    is not treated as an instantaneous jump.
    """
    if omega <= 0.0 or time_per_cycle <= 0.0:
        return 0.0
    Vc = cfg.clearance_volume
    V_bdc = Vc * cfg.compression_ratio
    T1 = cycle.get("T1", T_intake)
    T3 = cycle["T3"]
    p3 = cycle["p3"]
    # Spark at TDC minus the configured timing, referenced from firing TDC.
    spark_deg = 360.0 - cfg.spark_timing_deg_btdc
    burn_deg = max(1.0, combustion_duration(
        omega * 60.0 / (2.0 * math.pi),
        min(1.0, p_intake / 101325.0) if 101325.0 > 0 else 1.0,
        1.0))

    mean_piston_speed = cfg.stroke_m * omega / math.pi
    total = 0.0
    for i in range(_HEAT_LOSS_SAMPLES):
        theta = (i + 0.5) * 720.0 / _HEAT_LOSS_SAMPLES
        # cylinder_volume takes radians; the rest of this loop works in degrees.
        vol = cylinder_volume(math.radians(theta), Vc,
                              cfg.compression_ratio, cfg.rod_ratio)
        if vol <= 0.0:
            continue
        # Phase within the 720° cycle: 180-360 compression, 360-540 power,
        # remainder gas exchange. Reconstruct p and T from the polytropes.
        if theta < 180.0 or theta >= 540.0:
            T_gas = T_intake
            p_gas = p_intake
        elif theta < 360.0:
            # Compression BDC → TDC.
            frac = (theta - 180.0) / 180.0
            T_gas = T1 * (V_bdc / vol) ** (N_COMPRESSION - 1.0)
            p_gas = p_intake * (V_bdc / vol) ** N_COMPRESSION
        else:
            # Power stroke TDC → BDC, with the Wiebe burn spread in.
            frac = (theta - 360.0) / 180.0
            x_b = wiebe_mass_fraction_burned(
                theta - spark_deg, 0.0, burn_deg, cfg.wiebe_a, cfg.wiebe_m)
            T2 = T1 * (V_bdc / Vc) ** (N_COMPRESSION - 1.0)
            p2 = p_intake * (V_bdc / Vc) ** N_COMPRESSION
            # Woschni's reference state (KB Eq 12.18) is the cylinder at the
            # start of combustion, not intake conditions: the term carries
            # (p − p_m)/p_r, so a low p_r inflates w by ~10x.
            # Blend from the end of compression to the fired peak state.
            T_gas = T2 + x_b * (T3 - T2)
            p_gas = p2 * (T_gas / T2) if T2 > 0 else p2
        area = chamber_surface_area(cfg.bore_m, cfg.stroke_m, vol, clearance_volume=Vc)
        p_motored = p_intake * (V_bdc / max(vol, Vc)) ** N_COMPRESSION
        T2 = T1 * (V_bdc / Vc) ** (N_COMPRESSION - 1.0)
        p2 = p_intake * (V_bdc / Vc) ** N_COMPRESSION
        if model == "woschni":
            h_c = woschni_heat_transfer_coeff(
                cfg.bore_m, p_gas, T_gas, mean_piston_speed, vol, Vc,
                p2, T2, p_motored,
                combustion_occurred=(360.0 <= theta < 540.0),
                reference_volume=V_bdc,
                displaced_volume=cfg.displacement_per_cyl,
                gas_exchange=(theta < 180.0 or theta >= 540.0),
            )
        else:
            from .heat_transfer import annand_heat_transfer_coeff
            h_c = annand_heat_transfer_coeff(
                cfg.bore_m, p_gas, T_gas, mean_piston_speed)
        q_w = heat_transfer_rate(h_c, area, T_gas, cfg.wall_temperature_k)
        total += max(0.0, q_w) * (time_per_cycle / _HEAT_LOSS_SAMPLES)

    return total


@dataclass
class EngineOutput:
    """Engine outputs for one timestep — all quantities after an update.

    Includes v0.1 fields (backward compatible) plus new thermodynamic fields.
    """
    # --- v0.1 fields (kept for backward compat) ---
    rpm: float = 0.0
    angular_velocity: float = 0.0
    throttle: float = 0.0
    engine_torque: float = 0.0        # WOT torque (legacy: from curve)
    friction_torque: float = 0.0
    load_torque: float = 0.0
    net_torque: float = 0.0
    angular_acceleration: float = 0.0
    power: float = 0.0
    limiter_active: bool = False

    # --- New v0.2 thermodynamic fields ---
    crank_angle_deg: float = 0.0
    intake_pressure: float = 0.0      # Pa
    intake_temperature: float = 0.0   # K
    air_mass_per_cycle: float = 0.0   # kg
    fuel_mass_per_cycle: float = 0.0  # kg
    afr: float = 14.7
    lambda_val: float = 1.0
    volumetric_efficiency: float = 0.0
    mass_flow_rate: float = 0.0       # kg/s

    # Torque decomposition
    indicated_torque: float = 0.0     # N·m (gross indicated)
    pumping_torque: float = 0.0       # N·m
    accessory_torque: float = 0.0     # N·m

    # Combustion
    cyl_pressure: float = 0.0         # Pa
    cyl_temperature: float = 0.0      # K
    stroke_phase: int = 0             # 0=intake, 1=comp, 2=power, 3=exhaust
    cycle_work: float = 0.0           # J per cycle per cylinder
    thermal_efficiency: float = 0.0

    # --- New v0.3 fields (KB gap fixes) ---
    mass_fraction_burned: float = 0.0    # Wiebe x_b
    manifold_pressure: float = 0.0       # Pa (dynamic)
    spark_timing_deg: float = 0.0        # actual spark [deg BTDC]
    combustion_duration_deg: float = 0.0 # total Δθ [deg]
    heat_transfer_loss_w: float = 0.0    # heat loss to walls [W]
    boost_pressure: float = 0.0          # Pa (turbo boost)
    nox_ppm: float = 0.0
    co_ppm: float = 0.0
    hc_ppm: float = 0.0
    knock_integral: float = 0.0
    knock_detected: bool = False

    # --- Controls / run state ---
    ignition: bool = True
    combusting: bool = False          # spark + fuel actually delivered this step
    fuel_cut: bool = False            # DFCO or fuel-cut limiter active
    starter_torque: float = 0.0       # N·m from starter motor
    governor_torque: float = 0.0      # N·m from torque-mode idle governor
    iac_position: float = 0.0         # idle-air bypass, fraction of WOT area

    # --- Resolved-core fields (zero / empty when use_resolved_cylinders is off) ---
    resolved: bool = False            # True when these came from the crank-resolved cylinders
    exhaust_pressure: float = 0.0     # Pa, exhaust plenum (what the exhaust valves see)
    exhaust_temperature: float = 0.0  # K
    imep_net_bar: float = 0.0         # net indicated mean effective pressure, last cycle
    imep_gross_bar: float = 0.0       # gross (excl. the pumping loop)
    pmep_bar: float = 0.0             # pumping mean effective pressure (positive = loss)
    peak_pressure_pa: float = 0.0     # mean per-cylinder peak firing pressure, last cycle
    knock_retard_deg: float = 0.0     # spark retard applied by knock control
    cylinder_pressures_pa: List[float] = field(default_factory=list)      # instantaneous, per cylinder
    cylinder_temperatures_k: List[float] = field(default_factory=list)
    cylinder_masses_kg: List[float] = field(default_factory=list)
    intake_lift_m: List[float] = field(default_factory=list)              # valve lift, per cylinder
    exhaust_lift_m: List[float] = field(default_factory=list)
    vvt_intake_deg: float = 0.0       # intake cam advance now [crank deg, + = earlier]
    misfire_rate: float = 0.0         # recent fraction of cycles that misfired
    burn_quality: float = 1.0         # mean burn quality (1 = complete burn)
    residual_fraction: float = 0.0    # burned-gas fraction of the charge at the spark
    turbo_speed_rps: float = 0.0      # turbo shaft speed [rev/s] (resolved core with use_turbo)
    wastegate_position: float = 0.0   # 0 closed .. 1 open
    turbine_power_w: float = 0.0      # shaft power the turbine wheel is producing [W]
    exhaust_post_pressure_pa: float = 0.0   # behind the turbine (downpipe)
    vvt_exhaust_deg: float = 0.0


class Engine:
    """Internal combustion engine rotational dynamics simulator.

    This class implements the core engine physics based on ENGINE_KB.md.
    v0.2 replaces the torque-curve lookup with a thermodynamic Otto cycle model.

    Architecture:
        EngineInput (throttle, load_torque)
            → Air model (trapped mass, ηv)
            → Fuel model (fuel mass, AFR)
            → Combustion model (Otto cycle work → indicated torque)
            → Friction model (rubbing + pumping + accessories)
            → Crankshaft dynamics (I·α = ΣT)
            → EngineOutput (RPM, torque breakdown, ...)

    Source: ENGINE_KB.md Eq 14.52, Part XXIV.
    """

    def __init__(self, config: Optional[EngineConfig] = None):
        self.config = config or EngineConfig()
        self.state = EngineState()
        self._last_output = EngineOutput()
        self._limiter_was_active = False
        self._idle_integral = 0.0

        # v0.3 new model states
        self._fuel_film = FuelFilmState(
            x=self.config.fuel_film_x,
            tau=self.config.fuel_film_tau,
        )
        self._lambda_ctrl = LambdaController(
            target_lambda=self.config.lambda_target,
            integrator_gain=self.config.lambda_integrator_gain,
            transport_delay_s=self.config.lambda_transport_delay_s,
            enabled=self.config.use_lambda_controller,
        )
        self._turbo_state = TurboState()
        self._knock_integral = 0.0
        self._last_stroke_phase = 0
        self._combustion_occurred_this_cycle = False
        self._last_combustion_theta = 0.0
        self._heat_loss_w = 0.0
        self._wall_loss_per_cycle_j = 0.0
        self._dfco_active = False
        self._limiter_cut = False
        self._iac_integral = 0.0
        self._synced = True        # ECU has crank/cam sync (see Step 0a)
        self._sync_revs = 0.0
        self._fire_time = 0.0      # seconds the engine has been firing (resolved core: fast-idle ramp)
        self._dfco_clock = 0.0     # same clock for the post-start DFCO delay; start() skips the delay
        self._starter_recent = 0.0
        self._was_firing = False
        self._rt_budget_s: Optional[float] = None
        self._vvt_in = 0.0
        self._vvt_ex = 0.0
        self._bank: Optional[CylinderBank] = self._make_bank()
        self._knock_retard_deg = 0.0
        self._knock_events_seen = 0

    def _make_bank(self) -> Optional[CylinderBank]:
        """The crank-resolved cylinders, if the config asks for them.

        The resolved core integrates a spark-ignition Wiebe burn, so a
        compression-ignition config keeps the cycle-averaged path.
        """
        cfg = self.config
        if cfg.use_resolved_cylinders and cfg.engine_type == "SI":
            if cfg.use_native_core and native_available():
                bank = NativeCylinderBank(cfg)
            else:
                bank = CylinderBank(cfg)
            bank.set_time_budget(self._rt_budget_s)
            return bank
        return None

    def set_realtime_budget(self, seconds_per_update: Optional[float]) -> None:
        """Hold the resolved core to a wall-clock budget per physics step.

        The bank then trims its cylinder-step budget (coarser substeps at speed)
        whenever it runs slower than this, and restores it when there is headroom,
        so a real-time front end keeps up with the clock on any machine. None (the
        default) leaves the solver deterministic. No effect on the cycle-averaged path.
        """
        self._rt_budget_s = seconds_per_update
        if self._bank is not None:
            self._bank.set_time_budget(seconds_per_update)

    @property
    def resolved(self) -> bool:
        """True when the crank-resolved cylinder model is driving the torque."""
        return self._bank is not None

    @property
    def bank(self) -> Optional[CylinderBank]:
        """The resolved cylinders (None on the cycle-averaged path)."""
        return self._bank

    def set_config(self, config: EngineConfig) -> None:
        """Swap in a new configuration while running (live tuning): the engine keeps its
        speed and crank angle; sub-models built from config values are re-created."""
        self.config = config
        self._fuel_film = FuelFilmState(x=config.fuel_film_x, tau=config.fuel_film_tau)
        self._lambda_ctrl = LambdaController(
            target_lambda=config.lambda_target,
            integrator_gain=config.lambda_integrator_gain,
            transport_delay_s=config.lambda_transport_delay_s,
            enabled=config.use_lambda_controller,
        )
        bank = self._make_bank()
        if bank is not None:
            # Keep the running gas state when only a tunable changed; rebuild
            # when the geometry, valves or cylinder count did.
            old = self._bank
            if old is not None and self._same_layout(old.cfg, config):
                old.cfg = config
                for cyl in old.cylinders:
                    cyl.cfg = config
                bank = old
            else:
                bank.reset(self.state.crank_angle_deg % 720.0)
        self._bank = bank

    @staticmethod
    def _same_layout(a: EngineConfig, b: EngineConfig) -> bool:
        keys = ("bore_m", "stroke_m", "rod_length_m", "compression_ratio", "num_cylinders",
                "firing_order", "firing_intervals_deg",
                "intake_open_btdc_deg", "intake_close_abdc_deg",
                "exhaust_open_bbdc_deg", "exhaust_close_atdc_deg",
                "intake_max_lift_m", "exhaust_max_lift_m",
                "intake_ramp_open_frac", "intake_ramp_close_frac",
                "exhaust_ramp_open_frac", "exhaust_ramp_close_frac",
                "exhaust_plenum_volume_m3", "exhaust_tail_dia_m")
        return all(getattr(a, k) == getattr(b, k) for k in keys)

    @property
    def state(self) -> EngineState:
        return self._state

    @state.setter
    def state(self, value: EngineState):
        self._state = value

    def start(self) -> None:
        self.state.reset(idle_rads=self.config.idle_rads)
        self._limiter_was_active = False
        self._idle_integral = 0.0
        self._fuel_film.reset()
        self._lambda_ctrl.reset()
        self._turbo_state.reset()
        self._knock_integral = 0.0
        self._last_stroke_phase = 0
        self._combustion_occurred_this_cycle = False
        self._last_combustion_theta = 0.0
        self._heat_loss_w = 0.0
        self._wall_loss_per_cycle_j = 0.0
        self._dfco_active = False
        self._limiter_cut = False
        self._iac_integral = 0.0
        self._synced = True        # start() drops the engine straight to idle
        self._sync_revs = 0.0
        self._fire_time = 0.0      # a fresh start still gets the fast-idle ramp
        self._dfco_clock = 1.0e9   # but it is already running: no start-up flare to ride out
        # Initialize manifold pressure to ambient
        self.state.manifold_pressure = self.config.ambient_pressure_pa
        self.state.manifold_temperature = self.config.intake_temperature_k
        self._reset_resolved()
        self._prime_resolved()

    def _reset_resolved(self) -> None:
        self._knock_retard_deg = 0.0
        self._knock_events_seen = 0
        self._bank = self._make_bank()

    def _prime_resolved(self) -> None:
        """Warm start for start(): a few fired cycles at a typical idle vacuum."""
        bank, cfg = self._bank, self.config
        if bank is None:
            return
        p_idle = 0.25 * cfg.ambient_pressure_pa
        n_rpm = self.state.omega * 30.0 / math.pi
        dtb = combustion_duration(n_rpm, 0.35, 1.0)
        bank.prime(self.state.crank_angle_deg % 720.0, n_rpm, p_idle,
                   cfg.intake_temperature_k, mbt_timing(dtb, n_rpm), dtb)
        self.state.manifold_pressure = p_idle

    def reset(self) -> None:
        self.state.reset(idle_rads=0.0)
        self._limiter_was_active = False
        self._idle_integral = 0.0
        self._fuel_film.reset()
        self._lambda_ctrl.reset()
        self._turbo_state.reset()
        self._knock_integral = 0.0
        self._last_stroke_phase = 0
        self._combustion_occurred_this_cycle = False
        self._last_combustion_theta = 0.0
        self._heat_loss_w = 0.0
        self._wall_loss_per_cycle_j = 0.0
        self._dfco_active = False
        self._limiter_cut = False
        self._iac_integral = 0.0
        self._synced = False
        self._sync_revs = 0.0
        self._fire_time = 0.0
        self._dfco_clock = 0.0
        self.state.manifold_pressure = self.config.ambient_pressure_pa
        self.state.manifold_temperature = self.config.intake_temperature_k
        self._reset_resolved()

    def update(self, inp: EngineInput, dt: float) -> EngineOutput:
        """Advance the engine simulation by one timestep.

        The full chain: throttle → manifold → air → fuel → combustion →
        heat transfer → friction → dynamics → RPM.
        """
        cfg = self.config
        st = self.state
        omega = st.omega
        N_rps = omega / (2.0 * math.pi)
        N_rpm = N_rps * 60.0
        cycles_per_s = N_rps / 2.0   # 4-stroke: one cycle per two revolutions

        throttle = max(0.0, min(1.0, inp.throttle))
        st.throttle = throttle

        # --- Step 0a: Run-state flags (KB_GAPS.md G-E04/G-E05/G-E06) ---
        # Decel fuel cutoff (DFCO): real ECUs cut fuel entirely at closed
        # throttle once meaningfully above idle, producing genuine engine
        # braking (friction + pumping only, no combustion assist). Needed
        # because G-E01's lumped throttle model floors intake pressure at
        # 0.2*p_amb rather than true vacuum, so closed-throttle combustion
        # would otherwise stay strong up to a very high RPM. Hysteresis
        # (enter/resume ratios) stops it chattering at the threshold.
        # Decel fuel cut waits out the start-up flare on the resolved core: the
        # manifold is still near atmospheric when the engine first fires, so it
        # revs past DFCO speed, and cutting fuel there drops it below stall speed.
        # The spark retard and idle-air loops bring it down instead.
        dfco_armed = self._bank is None or self._dfco_clock >= cfg.dfco_after_start_delay_s
        if throttle < 0.01 and omega > cfg.idle_rads * cfg.dfco_enter_ratio and dfco_armed:
            self._dfco_active = True
        elif throttle >= 0.01 or omega < cfg.idle_rads * cfg.dfco_resume_ratio:
            self._dfco_active = False

        # Fuel-cut rev limiter: cut at redline, resume below redline - hyst.
        # The speed then bounces off the limiter instead of being clamped.
        if cfg.limiter_type == "fuel_cut":
            if omega >= cfg.redline_rads:
                self._limiter_cut = True
            elif omega < cfg.redline_rads - cfg.limiter_hysteresis_rads:
                self._limiter_cut = False
        else:
            self._limiter_cut = False

        # Spark needs ignition on, enough crank speed for the charge to be
        # inducted and fired, and crank/cam sync: after a stop the ECU needs
        # crank_revs_to_fire revolutions of cranking before the first
        # injection, which is what makes a start take a moment of cranking.
        # Stall (KB_GAPS.md G-E05): below stall_rpm, without the starter,
        # combustion can't be sustained under load — sync is lost and must be
        # regained by cranking, or by the driveline pushing the engine back
        # above stall_rpm (a bump start).
        cranking = inp.starter or omega >= cfg.stall_rads
        if (not inp.ignition or omega < 0.5 * cfg.min_firing_rads
                or not cranking):
            self._synced = False
            self._sync_revs = 0.0
        elif not self._synced:
            self._sync_revs += omega * dt / (2.0 * math.pi)
            self._synced = self._sync_revs >= cfg.crank_revs_to_fire
        fire_ok = inp.ignition and self._synced and omega >= cfg.min_firing_rads
        self._fire_time = self._fire_time + dt if fire_ok else 0.0
        # The DFCO delay is for the flare after a *starter* start; a bump start (the
        # driveline spinning the engine) has no such flare and is cut as normal.
        self._starter_recent = 3.0 if inp.starter else max(0.0, self._starter_recent - dt)
        if fire_ok and not self._was_firing:
            self._dfco_clock = 0.0 if self._starter_recent > 0.0 else 1.0e9
        self._was_firing = fire_ok
        self._dfco_clock = self._dfco_clock + dt if fire_ok else 0.0
        fuel_cut = self._dfco_active or self._limiter_cut
        combusting = fire_ok and not fuel_cut

        # --- Step 0: Determine upstream intake pressure ---
        # If turbocharged, upstream is boosted; otherwise ambient.
        if cfg.use_turbo:
            p_upstream = self._turbo_state.boost_pressure_pa
        else:
            p_upstream = cfg.ambient_pressure_pa

        p_atm = cfg.ambient_pressure_pa

        # --- Step 1: Intake / manifold dynamics ---
        bank = self._bank
        T_intake = cfg.intake_temperature_k
        if bank is not None and bank.turbo is not None:
            T_intake = bank.turbo.boost_temperature      # charge air after the intercooler
        st.manifold_temperature = T_intake

        iac = 0.0
        A_throttle = 0.0
        if bank is not None:
            # Resolved core: the intake plenum is integrated inside the cylinder
            # bank, substep by substep, because it is the cylinders' own pulsed
            # intake flow that empties it. Here we only set the throttle area.
            if cfg.use_idle_air_control:
                iac = self._idle_air_control(throttle, omega, fire_ok, dt)
            A_throttle = (throttle_effective_area(throttle, cfg.throttle_max_area_m2)
                          + iac * cfg.throttle_max_area_m2)
            p_intake = st.manifold_pressure
        elif cfg.use_manifold_dynamics:
            # Throttle-plate orifice flow (KB Eq 7.18-7.20), plus the idle-air
            # bypass valve in parallel with the plate.
            if cfg.use_idle_air_control:
                iac = self._idle_air_control(throttle, omega, fire_ok, dt)
            A_throttle = (throttle_effective_area(throttle, cfg.throttle_max_area_m2)
                          + iac * cfg.throttle_max_area_m2)
            m_dot_throttle = throttle_flow_rate(
                p_upstream, st.manifold_pressure, T_intake,
                A_throttle, cfg.throttle_discharge_coeff,
            )
            # Cylinder air flow: Σṁ_{a,cyl} = η_v · ρ_m · V_d · N / 2  (KB Eq 7.22 basis)
            eta_v_for_flow = volumetric_efficiency(omega, cfg.max_volumetric_efficiency)
            rho_m = st.manifold_pressure / (R_AIR * T_intake) if T_intake > 0 else 0.0
            m_dot_cyl = eta_v_for_flow * rho_m * cfg.total_displacement * N_rps / 2.0
            # Manifold dynamics (KB Eq 7.22)
            p_intake = manifold_pressure_dynamics(
                st.manifold_pressure, m_dot_throttle, m_dot_cyl,
                cfg.manifold_volume_m3, T_intake, dt,
            )
            st.manifold_pressure = p_intake
        else:
            # Instantaneous intake pressure from throttle position
            p_intake = intake_pressure(throttle, p_upstream)

        # --- Step 2: Volumetric efficiency & trapped mass ---
        rho_i = p_intake / (R_AIR * T_intake) if T_intake > 0 else 0.0
        if bank is not None:
            # What the ECU *believes* each cylinder inducts (speed-density, with
            # η_v learned from the cylinders' own trapped mass). The air that
            # actually arrives is whatever the valves, ports and plenums deliver.
            eta_v = bank.ve_measured
            m_air = bank.air_estimate(bank.map_filtered, T_intake, N_rpm)
        else:
            eta_v = volumetric_efficiency(omega, cfg.max_volumetric_efficiency)
            m_air = trapped_mass(eta_v, p_intake, T_intake,
                                cfg.displacement_per_cyl)

        # m_air is the true per-cylinder per-cycle mass; flow is
        # m_air · N_cyl · N / n_R (KB Eq 2.27, n_R = 2 for a 4-stroke).
        st.air = AirState(
            intake_pressure=p_intake,
            intake_temperature=T_intake,
            air_density=rho_i,
            trapped_mass=m_air,
            volumetric_efficiency=eta_v,
            mass_flow_rate=(bank.air_mass_flow if bank is not None
                            else m_air * cfg.num_cylinders * cycles_per_s),
        )

        # --- Step 3: Fuel system ---
        # Lambda controller (KB Eq 7.15-7.17) — reads actual AFR from last cycle
        lambda_correction = 1.0
        sensor_voltage = 0.45
        # Actual AFR deviation comes from fuel-film transients and cycle-to-cycle effects.
        # Use the previous cycle's fuel/air ratio as the "measured" lambda.
        if cfg.use_lambda_controller:
            # The sensor sees the exhaust AFR from the previous cycle
            prev_afr = st.fuel.afr if st.fuel.afr > 0 else cfg.stoich_afr
            measured_lambda = lambda_from_afr(prev_afr, cfg.stoich_afr)
            lambda_correction, sensor_voltage = self._lambda_ctrl.update(
                measured_lambda, 800.0, dt,
            )
            st.lambda_correction = lambda_correction
            st.sensor_voltage = sensor_voltage

        # Base fuel mass from air (stoichiometric)
        m_fuel_base = fuel_mass_per_cycle(m_air, cfg.stoich_afr, cfg.stoich_afr)

        # Apply lambda correction (fuel trim from closed-loop controller)
        m_fuel_cmd = m_fuel_base * lambda_correction

        # Fuel-film dynamics (KB Eq 7.25-7.27)
        # The film model creates a transient disturbance: on throttle opening,
        # only (1-x) of metered fuel goes directly to cylinders; the rest
        # accumulates in the film and evaporates with time constant τ.
        m_fuel_actual = m_fuel_cmd
        if cfg.use_fuel_film:
            # Per-cycle fuel mass → flow rate (one injection per cylinder per cycle).
            m_dot_fuel_injected = m_fuel_cmd * cfg.num_cylinders * cycles_per_s
            m_dot_fuel_cyl, new_film_state = fuel_film_dynamics(
                m_dot_fuel_injected, self._fuel_film, dt,
            )
            self._fuel_film = new_film_state
            # Convert back to per-cycle mass: ṁ_fuel_cyl / (N_cyl · cycles/s)
            denom = cfg.num_cylinders * cycles_per_s
            m_fuel_actual = m_dot_fuel_cyl / denom if denom > 0 else m_fuel_cmd
            m_fuel_actual = max(0.0, m_fuel_actual)

        # No fuel delivered when not combusting (ignition off, cranking below
        # firing speed, DFCO or fuel-cut limiter) — see Step 0a.
        if not combusting:
            m_fuel_actual = 0.0

        # Actual AFR after fuel-film dynamics (for lambda controller feedback).
        # On the resolved core the air is what the cylinders really trapped, so
        # a wrong ECU estimate shows up here as a genuine mixture error.
        m_air_actual = m_air
        if bank is not None and bank.trapped_air_kg > 0.0:
            m_air_actual = bank.trapped_air_kg
        actual_afr = m_air_actual / m_fuel_actual if m_fuel_actual > 0 else cfg.stoich_afr * 10.0
        actual_lambda = lambda_from_afr(actual_afr, cfg.stoich_afr)
        phi = equivalence_ratio(actual_afr, cfg.stoich_afr)

        st.fuel = FuelState(
            fuel_mass_per_cycle=m_fuel_actual,
            fuel_mass_flow_rate=m_fuel_actual * cfg.num_cylinders * cycles_per_s,
            afr=actual_afr,
            lambda_val=actual_lambda,
            equivalence_ratio=phi,
        )

        # --- Step 4: Combustion, torque and gas exchange ---
        if bank is not None:
            # Crank-angle-resolved core. Spark, burn duration and fuel set the
            # inputs; torque, pressure traces, wall loss, pumping, trapped air
            # and exhaust backpressure all come out of the cylinder integration.
            load_frac = min(1.0, bank.map_filtered / p_atm) if p_atm > 0 else 1.0
            # A dilute charge burns slowly: residual beyond what the low-load term of the
            # burn-duration law already assumes (~12 %) stretches the burn further.
            delta_theta_b = combustion_duration(
                N_rpm, load_frac, phi, 0.05 + max(0.0, bank.residual_fraction - 0.12))
            # No real flame needs more than ~75° from spark to burnt, and no road engine
            # runs more than ~35° of spark advance: the empirical laws above are not
            # meant for the corners (deep vacuum, very dilute charge) the resolved core can reach.
            delta_theta_b = min(delta_theta_b, 75.0)
            spark_base = (min(35.0, mbt_timing(delta_theta_b, N_rpm)) if cfg.use_mbt_timing
                          else cfg.spark_timing_deg_btdc)
            # Idle spark trim: the fast torque loop at closed throttle (the
            # idle-air valve behind it is slow, it waits on the manifold).
            idle_trim = 0.0
            if (throttle < 0.01 and combusting and cfg.idle_spark_gain_deg_per_rpm > 0.0):
                # A torque reserve: idle runs with the spark held back from MBT, so the
                # trim can add torque (advance) as well as take it away (retard).
                idle_trim = max(-cfg.idle_spark_max_retard_deg,
                                min(cfg.idle_spark_max_advance_deg,
                                    cfg.idle_spark_gain_deg_per_rpm
                                    * (self._idle_target_rads() * 30.0 / math.pi - N_rpm)
                                    - cfg.idle_spark_reserve_deg))
            spark_timing = max(0.0, spark_base + idle_trim - self._knock_retard_deg)

            energy_scale = None
            if cfg.use_cyclic_variability and omega > cfg.idle_rads * 0.5:
                import random
                sigma = cyclic_variability_cov(N_rpm, load_frac, phi) / 100.0
                energy_scale = [max(0.5, min(1.5, random.gauss(1.0, sigma)))
                                for _ in range(cfg.num_cylinders)]

            self._update_vvt(inp, N_rpm, load_frac, dt)
            bank.advance(
                st.crank_angle_deg % 720.0, math.degrees(omega * dt), dt, N_rpm,
                manifold_pressure=st.manifold_pressure, intake_temperature=T_intake,
                throttle_area=A_throttle, p_upstream=p_upstream,
                fuel_per_cycle=m_fuel_actual, air_ref=m_air,
                spark_btdc=spark_timing, burn_deg=delta_theta_b,
                energy_scale=energy_scale)
            st.manifold_pressure = p_intake = bank.manifold_pressure
            st.air.mass_flow_rate = bank.air_mass_flow
            T_ind = bank.gross_torque       # pumping is reported separately below

            # Knock control: each detected knock retards the spark, which then
            # recovers with time. (The cycle-averaged path scales torque instead.)
            knock_detected = cfg.use_knock_model and bank.knock_detected
            self._knock_integral = bank.knock_integral
            if cfg.use_knock_model:
                new_events = bank.knock_events - self._knock_events_seen
                self._knock_events_seen = bank.knock_events
                if new_events > 0:
                    self._knock_retard_deg = min(
                        cfg.knock_retard_max_deg,
                        self._knock_retard_deg + cfg.knock_retard_step_deg * new_events)
                else:
                    self._knock_retard_deg = max(
                        0.0, self._knock_retard_deg - cfg.knock_retard_recovery_deg_per_s * dt)

            self._heat_loss_w = bank.heat_loss_w
            self._wall_loss_per_cycle_j = bank.cycle_heat_loss_j

            cyls = bank.cylinders
            n_c = float(len(cyls))
            q_fuel_cycle = m_fuel_actual * cfg.lower_heating_value
            cycle = {
                # State at the spark (end of compression), peaks, and exhaust.
                "T2": sum(c.last_reference_temperature_k for c in cyls) / n_c or T_intake,
                "p2": sum(c.last_reference_pressure_pa for c in cyls) / n_c or p_intake,
                "T3": bank.peak_burned_temperature_k or bank.peak_temperature_k,
                "p3": bank.peak_pressure_pa,
                "T4": bank.exhaust.temperature,
                "W_net": bank.cycle_work_j,
                "eta_th": (max(0.0, min(1.0, bank.cycle_work_j / q_fuel_cycle))
                           if q_fuel_cycle > 0.0 else 0.0),
            }
            x_b = cyls[0].burned_fraction

            st.combustion = CombustionState(
                indicated_torque=T_ind,
                work_net=cycle["W_net"],
                T2=cycle["T2"], T3=cycle["T3"],
                p2=cycle["p2"], p3=cycle["p3"],
                eta_th=cycle["eta_th"],
                mass_fraction_burned=x_b,
                spark_timing_deg=spark_timing,
                combustion_duration_deg=delta_theta_b,
                knock_integral=self._knock_integral,
                knock_detected=knock_detected,
            )
        else:
            # --- Step 4: Combustion ---
            Vc = cfg.clearance_volume
            rc = cfg.compression_ratio
            R_rod = cfg.rod_ratio
            W_fuel = m_fuel_actual * cfg.lower_heating_value

            # Combustion duration and spark timing (KB §9.2.3)
            load_frac = min(1.0, p_intake / p_atm) if p_atm > 0 else 1.0
            delta_theta_b = combustion_duration(N_rpm, load_frac, phi)
            if cfg.use_mbt_timing:
                spark_timing = mbt_timing(delta_theta_b, N_rpm)
            else:
                spark_timing = cfg.spark_timing_deg_btdc

            # Otto cycle work (air-standard) — uses temperature-dependent cv
            T_ind = cycle_averaged_torque(
                p_intake, T_intake, Vc, rc, R_rod, m_air,
                cfg.lower_heating_value, m_fuel_actual, cfg.combustion_efficiency, phi,
            )
            # cycle_averaged_torque returns ONE cylinder's average indicated torque;
            # scale to the whole engine. Friction/pumping/accessory losses already
            # use total_displacement (all cylinders), so this must match or the
            # torque balance is wrong by a factor of num_cylinders.
            T_ind *= cfg.num_cylinders
            cycle = otto_cycle_work(
                p_intake, T_intake, Vc, rc, R_rod, m_air,
                W_fuel, cfg.combustion_efficiency, phi,
            )

            # --- Step 4a: Cyclic variability (KB Eq 9.50) ---
            cov_imep = 0.0
            if cfg.use_cyclic_variability and omega > cfg.idle_rads * 0.5:
                cov_imep = cyclic_variability_cov(
                    N_rpm, load_frac, phi,
                )
                # Apply variation to indicated torque
                import random
                sigma_frac = cov_imep / 100.0
                variation = random.gauss(1.0, sigma_frac)
                variation = max(0.5, min(1.5, variation))
                T_ind *= variation
                cycle["W_net"] *= variation
                cycle["T3"] *= variation ** 0.3  # peak T varies less than work

            # --- Step 4b: Spark timing phasing effect ---
            # MBT timing means peak pressure ~8° ATC.偏离 MBT reduces torque.
            # Simplified: torque scales with cos-deviation from optimal phasing
            mbt = mbt_timing(delta_theta_b, N_rpm) if cfg.use_mbt_timing else spark_timing
            phasing_error = abs(spark_timing - mbt)
            phasing_factor = 1.0 - 0.01 * phasing_error ** 1.5  # ~1% loss per degree偏离
            phasing_factor = max(0.7, min(1.0, phasing_factor))
            T_ind *= phasing_factor

            # --- Step 4c: Wiebe x_b for current crank angle (diagnostics) ---
            # Spark fires at (spark_timing) degrees BTDC, so θ₀ = -spark_timing relative to TDC
            theta_ca_deg = st.crank_angle_deg
            # Convert to 0-720 cycle: TDC compression = 360°, TDC exhaust = 540°
            ca_in_cycle = theta_ca_deg % 720.0
            # Combustion starts when spark fires: at (360 - spark_timing) degrees
            theta_combustion_start = 360.0 - spark_timing
            x_b = wiebe_mass_fraction_burned(
                ca_in_cycle, theta_combustion_start, delta_theta_b,
                cfg.wiebe_a, cfg.wiebe_m,
            )

            # --- Step 4d: Knock (Livengood-Wu integral, KB Eq 9.60-9.64) ---
            # Evaluated on the unburned-zone (end-of-compression) state T2/p2,
            # accumulated through the compression + power strokes of the current
            # combustion event, and reset when a new event begins.
            knock_detected = False
            if cfg.use_knock_model:
                if st.stroke_phase == 1 and self._last_stroke_phase != 1:
                    self._knock_integral = 0.0
                if st.stroke_phase in (1, 2):
                    self._knock_integral += livengood_wu_integral(
                        cycle["T2"], cycle["p2"], dt, cfg.fuel_octane_ron,
                    )
                knock_detected = self._knock_integral >= 1.0
            self._last_stroke_phase = st.stroke_phase

            # --- Step 5: Heat transfer (KB Eq 12.14-12.19) ---
            Q_loss_w = 0.0
            if cfg.use_heat_transfer and cycle["T3"] > 300.0 and omega > 10.0:
                mean_piston_speed = cfg.stroke_m * omega / math.pi  # m/s
                cyl_vol_now = cylinder_volume(
                    st.theta, cfg.clearance_volume, cfg.compression_ratio, cfg.rod_ratio,
                )
                A_chamber = chamber_surface_area(cfg.bore_m, cfg.stroke_m, cyl_vol_now,
                                                 clearance_volume=cfg.clearance_volume)

                # Motoring pressure: the pressure trace the cylinder would follow
                # with no combustion, i.e. a single polytropic curve referenced
                # from intake (IVC) conditions through the whole 720°. This is
                # the standard Woschni "motored" reference used to isolate the
                # combustion-driven term of the gas velocity.
                V_ivc = Vc * rc
                p_motored = p_intake * (V_ivc / max(cyl_vol_now, Vc)) ** N_COMPRESSION

                if cfg.heat_transfer_model == "woschni":
                    h_c = woschni_heat_transfer_coeff(
                        cfg.bore_m, cycle["p3"], cycle["T3"],
                        mean_piston_speed, cyl_vol_now, cfg.clearance_volume,
                        p_intake, T_intake, p_motored,
                        combustion_occurred=(st.stroke_phase == 2),
                        displaced_volume=cfg.displacement_per_cyl,
                        gas_exchange=(st.stroke_phase in (0, 3)),
                    )
                else:
                    from .heat_transfer import annand_heat_transfer_coeff
                    h_c = annand_heat_transfer_coeff(
                        cfg.bore_m, cycle["p3"], cycle["T3"], mean_piston_speed,
                    )

                Q_loss_w = heat_transfer_rate(h_c, A_chamber, cycle["T3"], cfg.wall_temperature_k)
                self._heat_loss_w = Q_loss_w

                # Diagnostic only — see below. Integrating the correlation over
                # crank angle is _integrated_wall_heat_loss(), which reports the
                # magnitude that *would* be rejected.
                time_per_cycle = 4.0 * math.pi / max(omega, 1.0)
                self._wall_loss_per_cycle_j = _integrated_wall_heat_loss(
                    cfg, cycle, p_intake, T_intake, omega, time_per_cycle,
                    model=cfg.heat_transfer_model,
                )
                # Do NOT charge this against W_net. Woschni is a crank-resolved
                # correlation: it needs the gas temperature and pressure at each
                # angle. This cycle-averaged path only has the air-standard Otto
                # peak state, and that peak is itself unphysical — constant-volume
                # heat addition of all the fuel drives T3 to ~4400 K and 152 bar at
                # stoichiometric load, where a real SI cylinder sees ~2500 K and
                # 50 bar. Integrating a correlation against that state yields a flux
                # orders of magnitude too high, which zeroes the net work and stalls
                # the engine. Reporting it as a diagnostic only.
                #
                # Wall heat loss that actually participates in the energy balance is
                # computed by the resolved model in engine_sim/cylinder.py, which
                # integrates h_c·A·(T−T_wall) over the real crank-angle temperature
                # history inside the first law it already solves.

            # Knock-limited torque retard — applied last so it survives regardless
            # of whether the heat-transfer step above recomputed T_ind.
            if knock_detected:
                T_ind *= cfg.knock_torque_retard_factor

            st.combustion = CombustionState(
                indicated_torque=T_ind,
                work_net=cycle["W_net"],
                T2=cycle["T2"], T3=cycle["T3"],
                p2=cycle["p2"], p3=cycle["p3"],
                eta_th=cycle["eta_th"],
                mass_fraction_burned=x_b,
                spark_timing_deg=spark_timing,
                combustion_duration_deg=delta_theta_b,
                knock_integral=self._knock_integral,
                knock_detected=knock_detected,
            )

        # --- Step 6: Friction and losses ---
        if cfg.use_friction_breakdown:
            # Component-level friction breakdown (KB Part XI)
            mean_piston_speed = cfg.stroke_m * omega / math.pi
            # imep_c from the actual Otto cycle (not peak firing pressure p3)
            # W_net can be transiently negative (e.g. heavy heat loss or
            # cyclic-variability derating) — clamp before deriving imep.
            imep_c_pa = max(0.0, cycle["W_net"]) / cfg.displacement_per_cyl
            imep_c_bar = imep_c_pa / 1e5
            peak_p_bar = cycle["p3"] / 1e5  # for piston gas-loading only
            friction_comp = total_friction_breakdown(
                cfg.bore_m, cfg.stroke_m, mean_piston_speed, N_rps,
                peak_p_bar, imep_c_bar, N_rpm,
                cfg.num_cylinders, cfg.num_valves_per_cyl,
            )
            T_fric = (friction_comp["piston_mep"] + friction_comp["bearing_mep"]
                      + friction_comp["valvetrain_mep"]) * cfg.total_displacement / (4.0 * math.pi)
            T_pump = friction_comp["pumping_mep"] * cfg.total_displacement / (4.0 * math.pi)
            T_acc = friction_comp["accessory_mep"] * cfg.total_displacement / (4.0 * math.pi)
        else:
            # Lumped friction model (KB Eq 13.5-13.6)
            tfmep = (cfg.friction_C1_Pa
                     + cfg.friction_C2_Pa_per_rev_s * N_rps
                     + cfg.friction_C3_Pa_per_rev_s2 * N_rps ** 2)
            T_fric = max(tfmep * cfg.total_displacement / (4.0 * math.pi), 0.0)

            # Pumping losses (KB Eq 13.8-13.12)
            p_i_gauge = p_intake - p_atm
            p_e_gauge = cfg.exhaust_back_pressure_pa - p_atm
            pmep = max(0.0, -p_i_gauge + p_e_gauge)
            speed_scale = 1.0 + 0.5 * (N_rps / 50.0)
            pmep *= speed_scale
            T_pump = pmep * cfg.total_displacement / (4.0 * math.pi)

            # Accessory losses (KB §13.7)
            acc_mep = 8000.0 * (1.0 + N_rps / 50.0)
            T_acc = acc_mep * cfg.total_displacement / (4.0 * math.pi)

        if bank is not None:
            # The pumping loop is the gas work on the intake and exhaust strokes
            # the cylinders just integrated: intake vacuum and exhaust
            # backpressure are in it, so the empirical pumping-MEP term above is
            # replaced, not added to. Positive = loss; negative = a boosted
            # engine being pushed round by its own intake stroke.
            T_pump = bank.pumping_torque

        st.friction = FrictionState(
            rubbing_friction_torque=T_fric,
            pumping_torque=T_pump,
            accessory_torque=T_acc,
            total_loss_torque=T_fric + T_pump + T_acc,
        )

        # --- Step 7: Net crankshaft torque (KB Eq 14.52) ---
        # Signed: a negative load is the driveline driving the engine.
        T_load = inp.load_torque
        T_net = T_ind - T_fric - T_pump - T_acc - T_load

        # --- Step 8: Idle speed governor, torque mode (KB_GAPS.md G-E03) ---
        # Used only when the idle-air valve (use_idle_air_control) is not.
        # Not from Heywood — an authored stand-in for idle spark/air control.
        # It acts only while the engine is actually idling (closed throttle,
        # firing, not in DFCO) and may at most cancel the combustion torque
        # (the most spark retard/cut can do). Everything above idle is left
        # to friction + pumping, so engine braking is physical: previously
        # its proportional term alone applied ~-390 N·m at 7000 rpm on a
        # throttle lift, swamping the real losses.
        idle_torque = 0.0
        idle_active = (throttle < 0.01 and combusting
                       and not (cfg.use_manifold_dynamics and cfg.use_idle_air_control))
        if idle_active:
            error = cfg.idle_rads - omega
            integral = max(-50.0, min(50.0, self._idle_integral + error * dt))
            Kp, Ki = 0.6, 2.0
            idle_torque = Kp * error + Ki * integral
            if idle_torque < -T_ind:
                idle_torque = -T_ind           # saturated: don't wind up
            else:
                self._idle_integral = integral
        elif throttle >= 0.01 or not fire_ok:
            self._idle_integral = 0.0
        # (during DFCO the integral is frozen, so idle is re-caught cleanly)

        T_net += idle_torque

        # --- Step 8b: Starter motor (KB_GAPS.md G-E06) ---
        # Series-wound DC motor through a pinion: torque falls linearly from
        # stall to zero at free speed; the overrunning clutch means it can
        # never drag the engine back.
        T_starter = 0.0
        if inp.starter and cfg.starter_free_speed_rads > 0:
            T_starter = cfg.starter_stall_torque_nm * max(
                0.0, 1.0 - omega / cfg.starter_free_speed_rads)
        T_net += T_starter

        # --- Step 9: Crankshaft dynamics (KB Eq 14.52) ---
        I_total = cfg.total_inertia
        alpha = T_net / I_total if I_total > 0 else 0.0
        omega_new = omega + alpha * dt
        omega_new = max(omega_new, 0.0)

        # --- Step 10: Rev limiter ---
        if cfg.limiter_type == "fuel_cut":
            limiter_active = self._limiter_cut
        else:
            omega_new, limiter_active = self._apply_limiter(
                omega_new, omega, throttle, dt
            )

        # --- Step 11: Integrate crank angle ---
        # (The resolved cylinders were advanced through omega·dt, so the crank
        # angle follows the same distance or the two would drift apart.)
        st.theta += (omega if bank is not None else omega_new) * dt
        st.theta = math.fmod(st.theta, 4.0 * math.pi)
        if st.theta < 0:
            st.theta += 4.0 * math.pi

        st.omega = omega_new
        st.limiter_active = limiter_active

        # --- Step 12: Turbo dynamics (if equipped) ---
        boost_pressure = cfg.ambient_pressure_pa
        if self._bank is not None and self._bank.turbo is not None:
            boost_pressure = self._bank.turbo.boost_pressure       # the real components, in the bank
        elif cfg.use_turbo and omega > 10.0:
            # Exhaust gas temperature (from last cycle's expansion + blowdown)
            # Real exhaust temp ≈ 800-1200K, NOT peak combustion T
            T_exhaust = cycle.get("T4", 800.0)  # T4 from Otto expansion ≈ exhaust temp
            T_exhaust = max(600.0, min(1200.0, T_exhaust))

            # Exhaust mass flow = air + fuel
            m_dot_exhaust = (st.air.mass_flow_rate
                             + (m_fuel_actual * cfg.num_cylinders * cycles_per_s))

            # Turbine pressure ratio (boost + backpressure)
            p_exhaust = (bank.exhaust.pressure if bank is not None
                         else cfg.exhaust_back_pressure_pa)
            pr_turbine = max(1.0, self._turbo_state.boost_pressure_pa / max(p_exhaust, 1.0))

            W_turb = turbine_power(
                m_dot_exhaust, T_exhaust, pr_turbine,
                cfg.turbo_turbine_efficiency,
            )

            # Compressor uses corrected speed/flow for efficiency
            corrected_speed = N_rpm * math.sqrt(293.0 / max(T_intake, 200.0))
            corrected_flow = st.air.mass_flow_rate * math.sqrt(T_intake / 293.0) / (p_upstream / 101325.0) if p_upstream > 0 else 0.0
            from .turbo_model import compressor_map
            eta_c = compressor_map(corrected_speed, corrected_flow,
                                   self._turbo_state.compressor_pressure_ratio)
            self._turbo_state.compressor_efficiency = eta_c

            W_comp = compressor_power(
                st.air.mass_flow_rate, T_intake,
                self._turbo_state.compressor_pressure_ratio,
                eta_c,
            )

            self._turbo_state.shaft_speed_rps = turbo_shaft_dynamics(
                W_turb, W_comp, self._turbo_state.shaft_speed_rps,
                dt=dt,
            )

            # Boost pressure from compressor map (not ad hoc line)
            if st.air.mass_flow_rate > 0 and T_intake > 0:
                # π_C from compressor energy: W_comp = ṁ·cp·T1·(π^((γ-1)/γ)-1)/η
                gamma_c = gamma_air(T_intake)
                cp_c = cp_air(T_intake)
                term = (W_comp * eta_c / (st.air.mass_flow_rate * cp_c * T_intake)) + 1.0
                pr_new = term ** (gamma_c / (gamma_c - 1.0))
                self._turbo_state.compressor_pressure_ratio = max(1.0, min(3.0, pr_new))
            boost_pressure = cfg.ambient_pressure_pa * self._turbo_state.compressor_pressure_ratio
            self._turbo_state.boost_pressure_pa = boost_pressure

            # Wastegate control
            self._turbo_state.wastegate_position, self._turbo_state.is_wastegate_active = wastegate_control(
                boost_pressure, cfg.turbo_target_boost_pa,
                self._turbo_state.wastegate_position, dt,
                cfg.turbo_wastegate_gain,
            )
            # Reduce effective exhaust energy when wastegate opens
            if self._turbo_state.wastegate_position > 0:
                W_turb *= (1.0 - 0.8 * self._turbo_state.wastegate_position)

        # --- Step 13: Emissions (KB Part IX) ---
        nox = 0.0
        co = 0.0
        hc = 0.0
        if cfg.compute_emissions:
            T3 = cycle.get("T3", 0)
            if bank is not None:
                nox = bank.nox_ppm            # integrated Zeldovich kinetics in the burned zone
            elif T3 > 1800.0:
                nox = nox_emission_index(T3, 2.0, 0.05, cycle["p3"] / 1e5)

            # CO: use actual kinetics model (KB Eq 11.21-11.22) if available
            from .emissions_model import co_oxidation_rate
            if T3 > 1000.0:
                # CO formed during combustion, then partially oxidized during expansion
                co_formed = 500.0 if phi > 1.0 else 50.0  # base CO formation
                co_remaining = co_oxidation_rate(T3, co_formed / 1e6, 0.05, dt=0.001)
                co = max(0.0, co_formed + co_remaining * 1e6)
            else:
                co = 0.0

            # Unburned fuel from partial burns and misfires raises HC on the resolved core.
            hc = hc_emissions_fraction(
                phi, cfg.combustion_efficiency * (bank.burn_quality if bank is not None else 1.0)) * 1000.0

        # --- Step 14: Diesel/CI mode (if configured) ---
        if cfg.engine_type == "CI":
            # Override Otto-cycle torque with diesel combustion model
            if cfg.diesel_injection_rate_kg_s > 0:
                tau_id = ignition_delay(cycle.get("T2", 800.0), cycle.get("p2", 30e5),
                                        cfg.fuel_cetane_number)
                pf = premixed_fraction(tau_id, cfg.diesel_injection_rate_kg_s,
                                       m_air, cfg.diesel_stoich_afr)
                # Diesel torque ≈ fuel_energy × η_c × (1 + premixed bonus)
                diesel_eff = cfg.combustion_efficiency * (1.0 + 0.1 * pf)
                W_diesel = m_fuel_actual * cfg.lower_heating_value * diesel_eff
                # W_diesel is per-cylinder (m_fuel_actual is per-cylinder-per-cycle);
                # scale to the whole engine like the SI path above.
                T_ind_diesel = W_diesel / (4.0 * math.pi) * cfg.num_cylinders
                T_ind = T_ind_diesel
                cycle["W_net"] = W_diesel
                cycle["eta_th"] = diesel_eff

        # --- Step 15: Compute outputs ---
        rpm_new = omega_new * 60.0 / (2.0 * math.pi)
        power = T_net * omega_new

        res_fields = {}
        if bank is not None:
            vd = cfg.displacement_per_cyl
            imep_net = bank.cycle_work_j / vd
            pmep = -bank.cycle_pumping_work_j / vd
            res_fields = dict(
                resolved=True,
                exhaust_pressure=bank.exhaust.pressure,
                exhaust_temperature=bank.exhaust.temperature,
                imep_net_bar=imep_net / 1e5,
                imep_gross_bar=(imep_net + pmep) / 1e5,
                pmep_bar=pmep / 1e5,
                peak_pressure_pa=bank.peak_pressure_pa,
                knock_retard_deg=self._knock_retard_deg,
                cylinder_pressures_pa=[c.pressure for c in bank.cylinders],
                cylinder_temperatures_k=[c.temperature for c in bank.cylinders],
                cylinder_masses_kg=[c.mass for c in bank.cylinders],
                intake_lift_m=[c.lift_intake for c in bank.cylinders],
                exhaust_lift_m=[c.lift_exhaust for c in bank.cylinders],
                misfire_rate=bank.misfire_rate,
                burn_quality=bank.burn_quality,
                residual_fraction=bank.residual_fraction,
                vvt_intake_deg=self._vvt_in,
                vvt_exhaust_deg=self._vvt_ex,
                turbo_speed_rps=bank.turbo.speed_rps if bank.turbo is not None else 0.0,
                wastegate_position=bank.turbo.wastegate_position if bank.turbo is not None else 0.0,
                turbine_power_w=bank.exhaust.turbine_power_w,
                exhaust_post_pressure_pa=bank.exhaust.post_pressure if bank.exhaust.has_turbine else 0.0,
            )

        out = EngineOutput(
            rpm=rpm_new,
            angular_velocity=omega_new,
            throttle=throttle,
            engine_torque=T_ind,
            friction_torque=T_fric,
            load_torque=T_load,
            net_torque=T_net,
            angular_acceleration=alpha,
            power=power,
            limiter_active=limiter_active,
            crank_angle_deg=st.crank_angle_deg,
            intake_pressure=p_intake,
            intake_temperature=T_intake,
            air_mass_per_cycle=m_air,
            fuel_mass_per_cycle=m_fuel_actual,
            afr=actual_afr,
            lambda_val=actual_lambda,
            volumetric_efficiency=eta_v,
            mass_flow_rate=st.air.mass_flow_rate,
            indicated_torque=T_ind,
            pumping_torque=T_pump,
            accessory_torque=T_acc,
            cyl_pressure=cycle["p3"],
            cyl_temperature=cycle["T3"],
            stroke_phase=st.stroke_phase,
            cycle_work=cycle["W_net"],
            thermal_efficiency=cycle["eta_th"],
            mass_fraction_burned=x_b,
            manifold_pressure=st.manifold_pressure,
            spark_timing_deg=spark_timing,
            combustion_duration_deg=delta_theta_b,
            heat_transfer_loss_w=self._heat_loss_w,
            boost_pressure=boost_pressure,
            nox_ppm=nox,
            co_ppm=co,
            hc_ppm=hc,
            knock_integral=self._knock_integral,
            knock_detected=knock_detected,
            ignition=inp.ignition,
            combusting=combusting,
            fuel_cut=fuel_cut,
            starter_torque=T_starter,
            governor_torque=idle_torque,
            iac_position=iac,
            **res_fields,
        )
        self._last_output = out
        return out

    @staticmethod
    def _smoothstep(lo: float, hi: float, x: float) -> float:
        if x <= lo:
            return 0.0
        if x >= hi:
            return 1.0
        t = (x - lo) / (hi - lo)
        return t * t * (3.0 - 2.0 * t)

    def vvt_schedule(self, rpm: float, load: float) -> tuple:
        """ECU target (intake advance, exhaust advance) [crank deg] for a speed and load.

        Intake: at its minimum (least overlap) at idle and light load; brought in from
        ~15 % of redline and held through mid speeds at higher load; backed out toward
        high rpm so the later intake closing helps top-end breathing. Exhaust: moves
        with load and speed over its own range. ``load`` is MAP/ambient.
        """
        cfg = self.config
        r = rpm / cfg.redline_rpm if cfg.redline_rpm > 0 else 0.0
        load_f = self._smoothstep(0.30, 0.90, load)
        f_in = load_f * self._smoothstep(0.12, 0.32, r) * (1.0 - 0.7 * self._smoothstep(0.55, 0.95, r))
        f_ex = load_f * self._smoothstep(0.25, 0.80, r)
        return (cfg.vvt_intake_min_deg + (cfg.vvt_intake_max_deg - cfg.vvt_intake_min_deg) * f_in,
                cfg.vvt_exhaust_min_deg + (cfg.vvt_exhaust_max_deg - cfg.vvt_exhaust_min_deg) * f_ex)

    def _update_vvt(self, inp: "EngineInput", rpm: float, load: float, dt: float) -> None:
        """Move the cams toward their targets (override or schedule) at the actuator rate."""
        cfg = self.config
        can_move = (cfg.vvt_intake_max_deg != cfg.vvt_intake_min_deg
                    or cfg.vvt_exhaust_max_deg != cfg.vvt_exhaust_min_deg
                    or inp.vvt_intake_deg is not None or inp.vvt_exhaust_deg is not None)
        if not can_move:
            return
        if cfg.vvt_auto:
            t_in, t_ex = self.vvt_schedule(rpm, load)
        else:
            t_in, t_ex = cfg.vvt_intake_min_deg, cfg.vvt_exhaust_min_deg
        if inp.vvt_intake_deg is not None:
            t_in = inp.vvt_intake_deg
        if inp.vvt_exhaust_deg is not None:
            t_ex = inp.vvt_exhaust_deg
        step = cfg.vvt_rate_deg_per_s * dt
        self._vvt_in += max(-step, min(step, t_in - self._vvt_in))
        self._vvt_ex += max(-step, min(step, t_ex - self._vvt_ex))
        self._bank.set_cam_phase(self._vvt_in, self._vvt_ex)

    def _idle_target_rads(self) -> float:
        """Idle speed target [rad/s].

        On the resolved core the target starts high after the engine first fires
        and decays to the idle speed (a fast-idle ramp, as a real ECU does when
        starting): the controller then sees a small error all the way down from
        the start-up flare, instead of a large one that winds it up and leaves the
        engine below idle when it returns.
        """
        cfg = self.config
        if self._bank is None or cfg.fast_idle_extra_ratio <= 0.0:
            return cfg.idle_rads
        return cfg.idle_rads * (1.0 + cfg.fast_idle_extra_ratio
                                * math.exp(-self._fire_time / cfg.fast_idle_decay_s))

    def _idle_air_control(self, throttle: float, omega: float,
                          fire_ok: bool, dt: float) -> float:
        """Idle-air bypass valve position [fraction of WOT throttle area].

        KB_GAPS.md G-E03: authored ECU logic, not from Heywood. The valve adds
        air in parallel with the throttle plate, so idle torque comes out of
        the manifold/combustion physics instead of an injected torque.
        PI on normalised speed error while idling; outside idle (throttle
        open, DFCO, not firing) the integral is frozen and the P term dropped,
        so the valve holds its last idle position — the "dashpot" that keeps
        the engine from dipping or stalling when fuel resumes after a decel.
        """
        cfg = self.config
        scale = cfg.iac_resolved_gain_scale if self._bank is not None else 1.0
        # On the resolved core the valve never closes right down: after a start-up
        # flare a nearly shut valve lets the manifold pump down far below the idle
        # pressure, and the engine then sags under idle while the manifold slowly
        # refills (a dashpot's job).
        iac_floor = max(cfg.iac_min, 0.55 * cfg.iac_feedforward) if self._bank is not None else cfg.iac_min
        kp, ki = cfg.iac_kp * scale * cfg.iac_resolved_kp_fraction if self._bank is not None else cfg.iac_kp, cfg.iac_ki * scale
        target = self._idle_target_rads()
        err = (target - omega) / cfg.idle_rads if cfg.idle_rads > 0 else 0.0
        base = cfg.iac_feedforward + self._iac_integral
        if not fire_ok:
            # Cranking / key off: open for starting air, don't integrate. On the
            # resolved core the valve stays at its idle position instead: the
            # engine pumps the manifold down while it cranks, and a wide-open
            # valve leaves it near atmospheric at first fire, which flares the
            # engine past DFCO speed and then drops it below stall speed.
            if self._bank is not None:
                return max(iac_floor, min(cfg.iac_max, base))
            return max(iac_floor, min(cfg.iac_max, base + kp * max(err, 0.0)))
        # Trim authority (resolved core): the integral may move the valve only so far
        # from its feed-forward position. Without a limit the long overspeed after a
        # start winds it far shut, and the engine sags below idle when it comes back.
        i_lo = -0.4 * cfg.iac_feedforward if self._bank is not None else -1e9
        i_hi = 0.8 * cfg.iac_feedforward if self._bank is not None else 1e9
        if throttle < 0.01 and not self._dfco_active:
            u = base + kp * err
            integral = self._iac_integral + ki * err * dt
            if iac_floor <= u <= cfg.iac_max or (u > cfg.iac_max) == (err < 0):
                self._iac_integral = max(i_lo, min(i_hi, integral))   # conditional anti-windup
            u = cfg.iac_feedforward + self._iac_integral + kp * err
            return max(iac_floor, min(cfg.iac_max, u))
        if self._bank is not None and self._dfco_active:
            # Dashpot: during a fuel cut the valve is held at least at its idle
            # position, so the manifold is not pumped down to a vacuum the
            # engine cannot recover from when fuel resumes.
            base = max(base, cfg.iac_feedforward)
        return max(iac_floor, min(cfg.iac_max, base))

    def _apply_limiter(
        self, omega_new: float, omega_old: float, throttle: float, dt: float
    ) -> tuple[float, bool]:
        redline = self.config.redline_rads
        hyst = self.config.limiter_hysteresis_rads
        if self._limiter_was_active:
            active = omega_new > (redline - hyst)
        else:
            active = omega_new >= redline
        if active:
            omega_limited = min(omega_new, redline)
            if throttle > 0.01:
                omega_limited = min(omega_limited, redline - hyst * 0.1)
            return omega_limited, True
        return omega_new, False

    @property
    def last_output(self) -> EngineOutput:
        return self._last_output
