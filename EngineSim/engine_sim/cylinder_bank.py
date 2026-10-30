"""The resolved engine core: every cylinder, the intake plenum and the exhaust.

``CylinderBank`` is what ``Engine.update()`` runs when
``EngineConfig.use_resolved_cylinders`` is on. It owns

  * one :class:`~engine_sim.cylinder.Cylinder` per cylinder, each carrying its
    own gas state around the 720° cycle at its own phase (set by the firing
    order and ``firing_intervals_deg``),
  * the exhaust plenum (:class:`~engine_sim.exhaust_system.ExhaustSystem`) that
    every exhaust valve discharges into, and
  * the intake manifold pressure, which it integrates itself because it is the
    cylinders' own pulsed intake flow — not a volumetric-efficiency table —
    that empties it.

One call to :meth:`advance` covers the crank angle swept in one engine timestep,
split into substeps of at most ``crank_angle_resolution_deg``. On every substep:

    throttle flow in → cylinders breathe from / exhale into the two plenums →
    manifold and exhaust pressures move with the mass that actually crossed

so cylinder-to-cylinder interaction (a neighbour's intake stroke pulling the
plenum down, two exhaust valves open together raising backpressure) is physics
rather than an overlay.

Fuel is metered the way a speed-density ECU does it: a filtered manifold
pressure and an air model *learned from what each cylinder actually trapped*
(slope and intercept in MAP, with the slope varying with speed, fitted by recursive
least squares) give an air estimate, and the fuel for that estimate is delivered
with the charge. Whatever the estimate gets wrong (transients, scavenging, a tuned
exhaust) shows up honestly as an air/fuel ratio error.
"""

from __future__ import annotations

import math
from time import perf_counter
from typing import List, Optional

from .air_model import volumetric_efficiency
from .config import EngineConfig
from .cylinder import CYCLE_DEG, Cylinder, _orifice
from .exhaust_system import ExhaustSystem
from .turbo_system import Turbocharger
from .thermodynamics import R_AIR

# The ECU's air model, learned from what the cylinders trap. A real speed-density
# ECU carries a calibrated table of η_v over (MAP, rpm) because η_v measured against
# manifold density collapses at low MAP, where residual gas and back-flow dominate.
# Here the table is the three-parameter model
#
#     m_air/(V_d·ρ_ref) = (a0 + a1·n)·x − b,    x = MAP/1e5,  n = rpm/6000
#
# (a slope that falls with speed and an intercept for the residual), fitted online
# by recursive least squares with forgetting.
_RLS_FORGET = 0.985
_RLS_P0 = 50.0
_RLS_P_MAX = 400.0
_RLS_MOVE = 0.03                    # |Δ(MAP/1e5)| + |Δ(rpm/6000)| that counts as a new operating point
_RLS_INIT = (0.9, 0.0, -0.1)       # a0, b, a1
_VE_BOUNDS = (0.2, 1.4)
# The per-cylinder summaries (last-cycle work, peaks, knock) change once a cycle, so
# they are refreshed every this many updates rather than every one.
_SUMMARY_EVERY = 8
_SLOW_MAP_TAU = 0.15                # s, slow average of MAP used to tell steady from transient
_STEADY_MAP_BAND = 0.02             # |MAP − slow MAP| / MAP below which a boosted engine learns


class CylinderBank:
    """All cylinders plus the plenums they breathe through.

    Attributes after :meth:`advance` (all averaged over the update):
        indicated_torque: Net indicated torque of the whole engine [N·m]
            (gross minus pumping).
        pumping_torque: Pumping loss [N·m], positive.
        gross_torque: Indicated torque excluding pumping [N·m].
        air_mass_flow: Net air flow into the cylinders [kg/s].
        heat_loss_w: Wall heat loss of the whole engine [W].
        manifold_pressure: Intake plenum pressure at the end of the update [Pa].
        map_filtered: Cycle-filtered manifold pressure the ECU sees [Pa].
        air_model parameters (``_theta``): the ECU's learned speed-density model, see
            ``air_estimate``.
        ve_measured: Volumetric efficiency of the most recent intake events.
    """

    def __init__(self, cfg: EngineConfig):
        self.cfg = cfg
        n = cfg.num_cylinders
        self.cylinders: List[Cylinder] = [Cylinder(i + 1, cfg, record_trace=(i == 0)) for i in range(n)]
        self._nominal_budget = max(2 * n, int(cfg.max_cylinder_steps_per_ms))
        self._res_deg = cfg.crank_angle_resolution_deg
        self.step_budget = self._nominal_budget
        self._time_budget: Optional[float] = None
        self._governed = False
        self._t_ema = 0.0
        self._gov_count = 0
        self._summary_due = 0
        turbo = bool(cfg.use_turbo)
        self.turbo: Optional[Turbocharger] = Turbocharger(cfg) if turbo else None
        self.exhaust = ExhaustSystem(
            cfg.exhaust_plenum_volume_m3, cfg.exhaust_tail_dia_m,
            cfg.exhaust_tail_discharge_coeff, cfg.ambient_pressure_pa,
            initial_temperature_k=cfg.intake_temperature_k,
            turbine_area_m2=cfg.turbo_turbine_area_m2 if turbo else 0.0,
            wastegate_area_m2=cfg.turbo_wastegate_area_m2 if turbo else 0.0,
            post_volume_m3=cfg.exhaust_post_volume_m3,
            turbine_efficiency=cfg.turbo_turbine_efficiency)
        self._fire = cfg.fire_angles_deg
        # A cycle's bookkeeping starts when the intake valve opens, so the fuel
        # that rides in with the first degrees of overlap is metered too.
        self._cycle_start_deg = self.cylinders[0].intake_valve.open_deg
        self.cam_intake_deg = 0.0
        self.cam_exhaust_deg = 0.0
        self.reset()

    # ── State ─────────────────────────────────────────────────────────────

    def reset(self, theta_deg: float = 0.0) -> None:
        """Fill every cylinder with ambient air at the given crank angle."""
        cfg = self.cfg
        self.exhaust.pressure = cfg.ambient_pressure_pa
        self.exhaust.temperature = cfg.intake_temperature_k
        self.exhaust.mass = (self.exhaust.pressure * self.exhaust.volume_m3
                             / (R_AIR * self.exhaust.temperature))
        self.exhaust.post_pressure = cfg.ambient_pressure_pa
        self.exhaust.post_temperature = cfg.intake_temperature_k
        self.exhaust.post_mass = (self.exhaust.post_pressure * self.exhaust.post_volume_m3
                                  / (R_AIR * self.exhaust.post_temperature))
        self.exhaust.wastegate_position = 0.0
        if self.turbo is not None:
            self.turbo.reset()
        for cyl, fire in zip(self.cylinders, self._fire):
            fresh = Cylinder(cyl.number, cfg, record_trace=(cyl.number == 1))
            fresh.set_cam_phase(self.cam_intake_deg, self.cam_exhaust_deg)
            local = self._local_angle(theta_deg, fire)
            fresh.initialize_charge(
                cfg.ambient_pressure_pa * fresh.volume(local)
                / (R_AIR * cfg.intake_temperature_k),
                cfg.intake_temperature_k, local)
            fresh.begin_cycle(0.0, 360.0 - cfg.spark_timing_deg_btdc,
                              cfg.combustion_duration_deg)
            self.cylinders[cyl.number - 1] = fresh
        self.map_filtered = cfg.ambient_pressure_pa
        self.ve_learned = volumetric_efficiency(0.0, cfg.max_volumetric_efficiency)
        self.ve_measured = self.ve_learned
        a0 = cfg.max_volumetric_efficiency if cfg.max_volumetric_efficiency < 1.0 else 0.95
        self._theta = [a0 * _RLS_INIT[0] / 0.9, _RLS_INIT[1], _RLS_INIT[2]]
        self._cov = [[_RLS_P0 if i == j else 0.0 for j in range(3)] for i in range(3)]
        self._ref_point = (-1.0, -1.0)
        self._map_slow = self.map_filtered
        self.trapped_air_kg = 0.0           # mean fresh air per cylinder per cycle
        self.indicated_torque = 0.0
        self.pumping_torque = 0.0
        self.gross_torque = 0.0
        self.air_mass_flow = 0.0
        self.heat_loss_w = 0.0
        self.manifold_pressure = cfg.ambient_pressure_pa
        self.manifold_pressure_min = self.manifold_pressure_max = self.manifold_pressure
        self.throttle_mass_flow = 0.0
        self.knock_detected = False
        self.knock_integral = 0.0
        self.knock_events = 0
        self.residual_fraction = 0.05       # mean burned-gas fraction of the charge at the spark
        self.nox_ppm = 0.0                  # NO in the burned gas of the last cycles (Zeldovich, two-zone)
        self.peak_burned_temperature_k = 0.0   # mean peak temperature of the burned zone
        self.burn_quality = 1.0             # mean burn quality of the last cycles (1 = complete burn)
        self.misfire_rate = 0.0             # recent fraction of cycles that misfired
        self.misfire_total = 0
        self._cycles_total = 0
        self.cycle_work_j = 0.0             # mean net indicated work / cylinder / cycle
        self.cycle_pumping_work_j = 0.0
        self.cycle_heat_loss_j = 0.0
        self.peak_pressure_pa = 0.0
        self.peak_temperature_k = 0.0
        self.substeps_last = 0
        self._ivc_seen = [c.ivc_events for c in self.cylinders]
        self._w0 = [0.0] * len(self.cylinders)
        self._p0 = [0.0] * len(self.cylinders)
        self._h0 = [0.0] * len(self.cylinders)
        self._summary_due = 0

    @staticmethod
    def _local_angle(theta_deg: float, fire_deg: float) -> float:
        """Cylinder-local cycle angle: 360° is that cylinder's firing TDC."""
        return (theta_deg - fire_deg + 360.0) % CYCLE_DEG

    # ── Variable valve timing ─────────────────────────────────────────────

    def set_cam_phase(self, intake_advance_deg: float, exhaust_advance_deg: float) -> None:
        """Move every cylinder's intake / exhaust cam (crank degrees advanced)."""
        if (intake_advance_deg == self.cam_intake_deg
                and exhaust_advance_deg == self.cam_exhaust_deg):
            return
        self.cam_intake_deg = intake_advance_deg
        self.cam_exhaust_deg = exhaust_advance_deg
        for cyl in self.cylinders:
            cyl.set_cam_phase(intake_advance_deg, exhaust_advance_deg)
        self._cycle_start_deg = self.cylinders[0].intake_open_deg

    # ── ECU air estimate ──────────────────────────────────────────────────

    def air_estimate(self, p_manifold: float, t_manifold: float, rpm: float = 0.0) -> float:
        """Fresh air per cylinder per cycle the ECU believes is inducted [kg].

        Speed-density from the learned model (see the module constants): the
        normalised air mass is ``(a0 + a1·n)·MAP − b``, scaled by V_d·1e5/(R·T).
        """
        if t_manifold <= 0.0 or p_manifold <= 0.0:
            return 0.0
        a0, b, a1 = self._theta
        y = (a0 + a1 * (rpm / 6000.0)) * (p_manifold / 1.0e5) - b
        if y < 0.02:
            y = 0.02
        return y * 1.0e5 / (R_AIR * t_manifold) * self.cfg.displacement_per_cyl

    def _learn_air_model(self, p_manifold: float, t_manifold: float, rpm: float,
                         trapped_air_kg: float) -> None:
        """One recursive-least-squares update from a closed intake event."""
        scale = 1.0e5 / (R_AIR * t_manifold) * self.cfg.displacement_per_cyl
        y = trapped_air_kg / scale
        x = p_manifold / 1.0e5
        n = rpm / 6000.0
        phi = (x, -1.0, x * n)
        # Forget old data only when the operating point has moved: at a steady point
        # the three regressors are collinear, so forgetting would inflate the
        # covariance along directions no data constrains and the parameters would
        # drift there until a clamp bit and broke the fit.
        ref_x, ref_n = self._ref_point
        if abs(x - ref_x) + abs(n - ref_n) > _RLS_MOVE:
            forget = _RLS_FORGET
            self._ref_point = (x, n)
        else:
            forget = 1.0
        P = self._cov
        theta = self._theta
        Pphi = [sum(P[i][j] * phi[j] for j in range(3)) for i in range(3)]
        denom = forget + sum(phi[i] * Pphi[i] for i in range(3))
        gain = [Pphi[i] / denom for i in range(3)]
        err = y - sum(theta[i] * phi[i] for i in range(3))
        for i in range(3):
            theta[i] += gain[i] * err
        for i in range(3):
            for j in range(3):
                P[i][j] = (P[i][j] - gain[i] * Pphi[j]) / forget
        # Keep the covariance bounded (a flat operating point carries no
        # information about some directions, and forgetting would inflate them).
        trace = P[0][0] + P[1][1] + P[2][2]
        if trace > _RLS_P_MAX:
            k = _RLS_P_MAX / trace
            for i in range(3):
                for j in range(3):
                    P[i][j] *= k
        theta[0] = min(1.5, max(0.3, theta[0]))
        theta[1] = min(0.5, max(-0.3, theta[1]))
        theta[2] = min(0.3, max(-0.8, theta[2]))

    # ── Integration ───────────────────────────────────────────────────────

    def advance(self, theta_deg: float, dtheta_deg: float, dt: float, rpm: float,
                *, manifold_pressure: float, intake_temperature: float,
                throttle_area: float, p_upstream: float,
                fuel_per_cycle: float, air_ref: float,
                spark_btdc: float, burn_deg: float,
                energy_scale: Optional[List[float]] = None,
                hold_manifold: bool = False) -> None:
        """Run every cylinder, both plenums and the throttle over one engine step.

        Args:
            theta_deg: Engine crank angle at the start of the step [degrees,
                0..720; cylinder 1's firing TDC is ``fire_angles_deg[0]``].
            dtheta_deg: Crank angle swept this step [degrees].
            dt: Time step [s].
            rpm: Engine speed [rev/min].
            manifold_pressure: Intake plenum pressure at the start [Pa].
            intake_temperature: Intake air temperature [K].
            throttle_area: Throttle + idle-air bypass flow area [m^2].
            p_upstream: Pressure upstream of the throttle (ambient / boost) [Pa].
            fuel_per_cycle: Fuel to load into each cylinder that begins a cycle
                during this step [kg] (0 = motoring).
            air_ref: Air mass that fuel was metered against [kg].
            spark_btdc: Spark timing, crank degrees before firing TDC.
            burn_deg: Wiebe burn duration [degrees].
            energy_scale: Optional per-cylinder multiplier on the energy the
                next cycle can release (cycle-to-cycle variability).
            hold_manifold: Keep the manifold pressure fixed (used to pre-roll a
                warm start).
        """
        cfg = self.cfg
        cyls = self.cylinders
        n_cyl = len(cyls)
        if dt <= 0.0 or dtheta_deg <= 0.0:
            # A stopped crank produces no torque; do not leave the last update's
            # values standing.
            self.gross_torque = self.pumping_torque = self.indicated_torque = 0.0
            self.air_mass_flow = self.heat_loss_w = self.throttle_mass_flow = 0.0
            self.manifold_pressure = manifold_pressure
            return
        t_wall0 = perf_counter() if self._governed else 0.0

        # Substeps: as fine as `crank_angle_resolution_deg`, but never more than
        # the substep cap or the cylinder-step budget allow.
        res = self._res_deg if self._res_deg > 0.1 else 0.1
        n_sub = int(math.ceil(dtheta_deg / res))
        # The budget is cylinder-steps per millisecond of simulated time, so a longer
        # physics step is allowed proportionally more substeps.
        cap = int(self.step_budget * dt * 1000.0 + 0.5) // n_cyl
        if cap < 1:
            cap = 1
        if n_sub > cap:
            n_sub = cap
        if n_sub > cfg.max_substeps_per_update:
            n_sub = int(cfg.max_substeps_per_update)
        if n_sub < 1:
            n_sub = 1
        sub_deg = dtheta_deg / n_sub
        sub_dt = dt / n_sub
        spark_local = 360.0 - spark_btdc
        t_man = intake_temperature
        # MAP filter: one firing interval, so the ECU sees the mean not the pulses.
        alpha = sub_deg / (CYCLE_DEG / n_cyl)
        if alpha > 1.0:
            alpha = 1.0
        fire = self._fire

        # Per cylinder: local angle at the middle of the first substep, and the
        # substep (if any) in which its cycle begins — the intake valve opening.
        loc = [(theta_deg - off + 360.0 + 0.5 * sub_deg) % CYCLE_DEG for off in fire]
        thr_x = CYCLE_DEG - sub_deg - 1e-9
        begin_k = []
        for off in fire:
            x0 = ((theta_deg - off + 360.0) - self._cycle_start_deg) % CYCLE_DEG
            if x0 >= thr_x:
                begin_k.append(0)
            else:
                kk = int(math.ceil((thr_x - x0) / sub_deg))
                begin_k.append(kk if kk < n_sub else -1)
        if fuel_per_cycle > 0.0:
            # A cylinder that began its cycle without fuel (a fuel cut that has just
            # ended) picks it up as soon as its intake valve is open.
            for i, cyl in enumerate(cyls):
                if cyl._fuel_per_air is None and begin_k[i] != 0 and cyl.intake_valve.is_open(loc[i]):
                    cyl.load_fuel(fuel_per_cycle, air_ref)
                    cyl.spark_angle_deg = spark_local
                    cyl.combustion_duration_deg = max(1.0, burn_deg)
                    cyl._burn_window = 2.0 * cyl.combustion_duration_deg + 20.0

        p_man, p_min, p_max, map_f, flow_in_sum, thr_sum, p_b, e_turbine, e_comp = self._substeps(
            n_sub, sub_deg, sub_dt, rpm, manifold_pressure, t_man, throttle_area, p_upstream,
            hold_manifold, loc, begin_k, fuel_per_cycle, air_ref, spark_local, burn_deg,
            energy_scale, alpha)
        turbo = self.turbo
        p_amb = cfg.ambient_pressure_pa
        self.map_filtered = map_f
        if turbo is not None:
            turbo.boost_pressure = p_b
            turbo.boost_temperature = turbo.charge_temperature(p_b / p_amb)
            turbo.advance_shaft(e_turbine, e_comp)
            turbo.control_wastegate(dt)

        # Learn volumetric efficiency from the intake events that closed.
        rho = map_f / (R_AIR * t_man)
        # On a turbocharged engine the trapped air also depends on the exhaust backpressure
        # (turbine and wastegate), which the speed-density model cannot see, so while boost is
        # moving the data would pull the fit away from where it holds at a steady point.
        # Learn only when the manifold pressure has settled against its own slow average.
        self._map_slow += min(1.0, dt / _SLOW_MAP_TAU) * (map_f - self._map_slow)
        steady = turbo is None or abs(map_f - self._map_slow) < _STEADY_MAP_BAND * map_f
        seen = self._ivc_seen
        for i in range(n_cyl):
            cyl = cyls[i]
            if cyl.ivc_events != seen[i]:
                seen[i] = cyl.ivc_events
                if rho > 0.0 and cyl.trapped_air_kg > 0.0:
                    self.ve_measured = cyl.trapped_air_kg / (rho * cfg.displacement_per_cyl)
                    if steady:
                        self._learn_air_model(map_f, t_man, rpm, cyl.trapped_air_kg)

        # Work and heat of this update, as differences of the cylinders' running
        # totals (no per-update resets).
        swept = dtheta_deg * 0.017453292519943295
        w_net = w_pump = heat = 0.0
        w0, p0, h0 = self._w0, self._p0, self._h0
        for i in range(n_cyl):
            cyl = cyls[i]
            wn, wp, ht = cyl.work_net, cyl.work_pumping, cyl.heat_loss_total
            w_net += wn - w0[i]
            w_pump += wp - p0[i]
            heat += ht - h0[i]
            w0[i], p0[i], h0[i] = wn, wp, ht
        self.indicated_torque = w_net / swept
        self.pumping_torque = -w_pump / swept
        self.gross_torque = self.indicated_torque + self.pumping_torque
        self.air_mass_flow = flow_in_sum / dt
        self.throttle_mass_flow = thr_sum / n_sub
        self.heat_loss_w = heat / dt
        self.manifold_pressure = p_man
        self.manifold_pressure_min = p_min
        self.manifold_pressure_max = p_max
        self.substeps_last = n_sub
        self._summary_due -= 1
        if self._summary_due <= 0:
            self._summary_due = _SUMMARY_EVERY
            self._refresh_cycle_summary()
        if self._governed:
            self._govern(perf_counter() - t_wall0)

    def _substeps(self, n_sub, sub_deg, sub_dt, rpm, p_man, t_man, throttle_area, p_upstream,
                  hold_manifold, loc, begin_k, fuel_per_cycle, air_ref, spark_local, burn_deg,
                  energy_scale, alpha):
        """The substep loop of :meth:`advance` in Python (``NativeCylinderBank`` replaces it).

        Returns (manifold pressure, its min, its max, filtered MAP, intake mass, throttle
        mass, boost pressure, turbine energy, compressor energy) at the end of the step.
        """
        cfg = self.cfg
        cyls = self.cylinders
        n_cyl = len(cyls)
        inv_sub_dt = 1.0 / sub_dt
        p_amb = cfg.ambient_pressure_pa
        p_min = p_max = p_man
        k_man = R_AIR * t_man / cfg.manifold_volume_m3
        thr_cd = throttle_area * cfg.throttle_discharge_coeff
        map_f = self.map_filtered
        exhaust = self.exhaust
        flow_in_sum = 0.0
        thr_sum = 0.0
        turbo = self.turbo
        p_b = e_turbine = e_comp = 0.0
        if turbo is not None:
            p_b = turbo.boost_pressure
            t_b = turbo.boost_temperature
            exhaust.wastegate_position = turbo.wastegate_position
            k_boost = R_AIR * t_b / turbo.volume
            by_cd = turbo.bypass_area * 0.7
            bov_cd = turbo.bov_area * 0.7 if throttle_area < 0.05 * cfg.throttle_max_area_m2 else 0.0
            t_amb = cfg.intake_temperature_k
            e_turbine = e_comp = 0.0
        for k in range(n_sub):
            p_exh = exhaust.pressure
            t_exh = exhaust.temperature
            d_in = 0.0
            d_ex = 0.0
            hot_dm = 0.0
            hot_dm_t = 0.0
            for i in range(n_cyl):
                cyl = cyls[i]
                if begin_k[i] == k:
                    cyl.begin_cycle(fuel_per_cycle, spark_local, burn_deg,
                                    air_mass_ref_kg=air_ref if fuel_per_cycle > 0.0 else None)
                    if energy_scale is not None:
                        cyl.energy_scale = energy_scale[i % len(energy_scale)]
                a = loc[i]
                cyl.step(a, sub_deg, sub_dt, rpm, p_man, t_man, p_exh, t_exh, p_amb)
                a += sub_deg
                loc[i] = a - CYCLE_DEG if a >= CYCLE_DEG else a
                d_in += cyl.dm_in
                e = cyl.dm_ex
                d_ex += e
                if e < 0.0:
                    hot_dm -= e
                    hot_dm_t -= e * cyl.temperature

            exhaust.update(-d_ex * inv_sub_dt,
                           hot_dm_t / hot_dm if hot_dm > 0.0 else exhaust.temperature,
                           sub_dt)

            m_th = 0.0
            if turbo is not None:
                # Throttle draws on the charge-air volume; the compressor, the unspun-compressor
                # check valve and the blow-off valve feed or drain it.
                if thr_cd > 0.0 and p_b > p_man:
                    m_th = _orifice(thr_cd, p_man, p_b, t_man, t_b)
                pr = p_b / p_amb
                m_c = turbo.compressor_flow(pr)
                m_in = m_c + (_orifice(by_cd, p_b, p_amb, t_b, t_amb) if p_b < p_amb else 0.0)
                if bov_cd > 0.0 and p_b > p_amb:
                    m_in += _orifice(bov_cd, p_b, p_amb, t_b, t_amb)       # negative: venting
                p_b += k_boost * (m_in - m_th) * sub_dt
                if p_b < 0.3 * p_amb:
                    p_b = 0.3 * p_amb
                e_comp += turbo.compressor_power(m_c, pr) * sub_dt
                e_turbine += exhaust.turbine_power_w * sub_dt
            elif thr_cd > 0.0 and p_upstream > p_man:
                m_th = _orifice(thr_cd, p_man, p_upstream, t_man, t_man)
            if not hold_manifold:
                p_man += k_man * (m_th * sub_dt - d_in)
                if p_man < 1000.0:
                    p_man = 1000.0
                elif p_man > 300000.0:
                    p_man = 300000.0
            if p_man < p_min:
                p_min = p_man
            elif p_man > p_max:
                p_max = p_man
            map_f += alpha * (p_man - map_f)
            flow_in_sum += d_in
            thr_sum += m_th
        return p_man, p_min, p_max, map_f, flow_in_sum, thr_sum, p_b, e_turbine, e_comp

    # ── Real-time governor ────────────────────────────────────────────────

    def set_time_budget(self, seconds_per_update: Optional[float]) -> None:
        """Hold the bank to a wall-clock budget per physics step (None = off).

        With a budget set, the cylinder-step budget is trimmed whenever the bank
        takes longer than that and restored when there is headroom, so the
        simulation keeps up with the clock on a slower machine at the price of
        coarser substeps at speed. Off (the default) the bank is deterministic.
        """
        self._time_budget = seconds_per_update
        self._governed = seconds_per_update is not None and seconds_per_update > 0.0
        if not self._governed:
            self.step_budget = self._nominal_budget
        self._t_ema = 0.0
        self._gov_count = 0

    def _govern(self, elapsed: float) -> None:
        self._t_ema += 0.1 * (elapsed - self._t_ema)
        self._gov_count += 1
        if self._gov_count < 12:
            return
        self._gov_count = 0
        floor = 2 * len(self.cylinders)
        if self._t_ema > self._time_budget:
            self.step_budget = max(floor, int(self.step_budget * 0.85))
        elif self._t_ema < 0.7 * self._time_budget and self.step_budget < self._nominal_budget:
            self.step_budget = min(self._nominal_budget,
                                       self.step_budget + max(2, self.step_budget // 8))

    def prime(self, theta_deg: float, rpm: float, manifold_pressure: float,
              intake_temperature: float, spark_btdc: float, burn_deg: float,
              cycles: int = 3) -> None:
        """Pre-roll a few fired cycles at fixed manifold pressure.

        Gives a warm start: the cylinders hold a realistic trapped charge and
        residual, the exhaust plenum is warm, the learned volumetric efficiency
        is set and a cycle's worth of heat-transfer / pressure history exists,
        instead of the first fired cycles running from an ambient-air fill.
        The crank angle is left where it started (whole cycles are run).
        """
        cfg = self.cfg
        # A warm engine's exhaust is hot; start the plenum there rather than at
        # ambient (it would take seconds of running to warm up from cold).
        self.exhaust.temperature = max(self.exhaust.temperature, 650.0)
        self.exhaust.mass = (self.exhaust.pressure * self.exhaust.volume_m3
                             / (R_AIR * self.exhaust.temperature))
        chunk = 10.0
        dt = chunk / (6.0 * max(rpm, 1.0))
        for k in range(int(cycles * CYCLE_DEG / chunk)):
            air = self.air_estimate(manifold_pressure, intake_temperature, rpm)
            self.advance(theta_deg + k * chunk, chunk, dt, rpm,
                         manifold_pressure=manifold_pressure,
                         intake_temperature=intake_temperature,
                         throttle_area=0.0, p_upstream=cfg.ambient_pressure_pa,
                         fuel_per_cycle=air / cfg.stoich_afr, air_ref=air,
                         spark_btdc=spark_btdc, burn_deg=burn_deg,
                         hold_manifold=True)
        self.map_filtered = manifold_pressure
        self.manifold_pressure = manifold_pressure
        self._refresh_cycle_summary()

    def _refresh_cycle_summary(self) -> None:
        cyls = self.cylinders
        n = float(len(cyls))
        self.cycle_work_j = sum(c.last_cycle_work_j for c in cyls) / n
        self.cycle_pumping_work_j = sum(c.last_cycle_pumping_work_j for c in cyls) / n
        self.cycle_heat_loss_j = sum(c.last_cycle_heat_loss_j for c in cyls) / n
        self.peak_pressure_pa = sum(c.last_peak_pressure_pa for c in cyls) / n
        self.peak_temperature_k = sum(c.last_peak_temperature_k for c in cyls) / n
        trapped = [c.trapped_air_kg for c in cyls if c.trapped_air_kg > 0.0]
        self.trapped_air_kg = sum(trapped) / len(trapped) if trapped else 0.0
        self.knock_integral = max(c.knock_integral for c in cyls)
        self.knock_detected = any(c.knock_detected for c in cyls)
        self.knock_events = sum(c.knock_events for c in cyls)
        self.residual_fraction = sum(c.residual_at_spark for c in cyls) / n
        self.nox_ppm = sum(c.last_no_ppm for c in cyls) / n
        self.peak_burned_temperature_k = sum(c.last_peak_burned_temperature_k for c in cyls) / n
        self.burn_quality = sum(c.burn_quality for c in cyls) / n
        misfires = sum(c.misfire_count for c in cyls)
        cycles = sum(c.cycles_completed for c in cyls)
        if cycles > self._cycles_total:
            self.misfire_rate += 0.25 * ((misfires - self.misfire_total) / (cycles - self._cycles_total)
                                         - self.misfire_rate)
            self.misfire_total, self._cycles_total = misfires, cycles
