"""Tests for the v0.7 additions to the resolved core: turbine in the exhaust path,
two-zone burn, variable valve timing, and the misfire / partial-burn limit."""

from __future__ import annotations

import math
import statistics

import pytest

from engine_sim.config import EngineConfig
from engine_sim.cylinder import Cylinder, _fall
from engine_sim.cylinder_bank import CylinderBank
from engine_sim.engine import Engine, EngineInput
from engine_sim.engine_file import load_engine_file, resolve_engine
from engine_sim.exhaust_system import ExhaustSystem
from engine_sim.gas_tables import internal_energy
from engine_sim.powertrain import Powertrain
from engine_sim.turbo_system import Turbocharger

from tests.test_resolved_engine import _cfg, _hold_speed


def _turbo_cfg(**over) -> EngineConfig:
    cfg = _cfg(use_idle_air_control=False, use_turbo=True, turbo_count=1,
               turbo_target_boost_pa=170_000.0, turbo_turbine_area_m2=3.0e-4,
               turbo_wastegate_area_m2=6.0e-4, turbo_design_flow_kgs=0.10,
               turbo_wastegate_gain=1.0, compression_ratio=9.0)
    for k, v in over.items():
        setattr(cfg, k, v)
    return cfg


def _run(pt, seconds, dt=0.001):
    for _ in range(int(seconds / dt)):
        pt.step(dt)


# ── Turbine in the exhaust path ───────────────────────────────────────────

def test_turbine_stage_balances_flow_and_extracts_power():
    """Fed a steady flow, the plenum settles where turbine + wastegate + tail pass it."""
    ex = ExhaustSystem(0.015, 0.05, turbine_area_m2=3.0e-4, wastegate_area_m2=5.0e-4,
                       post_volume_m3=8e-3)
    for _ in range(60000):
        ex.update(0.08, 900.0, 1.0e-4)
    flow_out = ex.turbine_mass_flow + ex.wastegate_mass_flow
    assert flow_out == pytest.approx(0.08, rel=0.02)
    assert ex.tail_mass_flow == pytest.approx(0.08, rel=0.02)
    assert ex.pressure > ex.post_pressure > 101_325.0          # a real pressure drop across the stage
    assert ex.turbine_power_w > 5_000.0
    # The wheel's power left the gas: the stream behind it is cooler than the plenum.
    assert ex.post_temperature < ex.temperature - 20.0


def test_wastegate_relieves_backpressure_and_power():
    def settle(pos):
        ex = ExhaustSystem(0.015, 0.05, turbine_area_m2=3.0e-4, wastegate_area_m2=5.0e-4)
        ex.wastegate_position = pos
        for _ in range(60000):
            ex.update(0.08, 900.0, 1.0e-4)
        return ex
    closed, open_ = settle(0.0), settle(1.0)
    assert open_.pressure < 0.8 * closed.pressure
    assert open_.turbine_power_w < 0.6 * closed.turbine_power_w


def test_compressor_map_and_shaft_energy_balance():
    cfg = _turbo_cfg()
    t = Turbocharger(cfg)
    assert t.compressor_flow(1.5) == 0.0                           # unspun: no flow
    t.omega = t.design_omega
    f_low, f_high = t.compressor_flow(1.2), t.compressor_flow(2.0)
    assert f_low > f_high > 0.0                                    # flow falls toward surge
    assert t.compressor_flow(10.0) == 0.0                          # beyond the speed line
    assert t.compressor_power(0.1, 2.0) > t.compressor_power(0.1, 1.5) > 0.0
    # ½·I·ω² changes by η_m·E_T − E_C
    e0 = 0.5 * t.inertia * t.omega ** 2
    t.advance_shaft(100.0, 20.0)
    assert 0.5 * t.inertia * t.omega ** 2 == pytest.approx(e0 + t.eta_m * 100.0 - 20.0, rel=1e-9)
    t.omega = 0.0
    t.advance_shaft(0.0, 5.0)                                      # cannot go negative
    assert t.omega == 0.0


def test_boost_builds_with_lag_and_the_wastegate_holds_it():
    pt = Powertrain(_turbo_cfg())
    pt.engine.start()
    c = pt.controls
    c.ignition, c.throttle, c.dyno_enabled, c.dyno_rpm = True, 1.0, True, 3500.0
    _run(pt, 0.35)
    early = pt.last.engine.boost_pressure
    _run(pt, 4.0)
    late = pt.last.engine.boost_pressure
    assert pt.engine.bank.turbo.speed_rps > 500.0
    assert early < 0.9 * late                                       # spool-up takes time
    assert 130_000.0 < late < 210_000.0                             # held near the 170 kPa target
    assert pt.last.engine.wastegate_position > 0.0
    assert pt.last.engine.manifold_pressure > 1.2 * 101_325.0


def test_wastegate_pi_law_settles_without_hunting():
    t = Turbocharger(_turbo_cfg())
    # A first-order plant: boost chases a set-point that falls as the wastegate opens.
    boost, dt, trace = 101_325.0, 0.01, []
    for k in range(3000):
        free_boost = 260_000.0 * (1.0 - 0.55 * t.wastegate_position)
        boost += (free_boost - boost) * dt / 0.5
        t.boost_pressure = boost
        t.control_wastegate(dt)
        if k > 2000:
            trace.append(boost)
    assert max(trace) - min(trace) < 0.01 * t.target_boost          # no limit cycle
    assert trace[-1] == pytest.approx(t.target_boost, rel=0.02)     # integral action: no offset
    assert 0.0 < t.wastegate_position < 1.0
    t.boost_pressure = 101_325.0
    for _ in range(500):
        t.control_wastegate(dt)
    assert t.wastegate_position == 0.0                              # shut while spooling, no wind-up


def test_turbo_boost_is_steady_at_a_steady_point():
    pt = Powertrain(_turbo_cfg())
    pt.engine.start()
    c = pt.controls
    c.ignition, c.throttle, c.dyno_enabled, c.dyno_rpm = True, 1.0, True, 3500.0
    _run(pt, 5.0)
    boosts = []
    for _ in range(1000):
        boosts.append(pt.step(0.002).engine.boost_pressure)
    assert max(boosts) - min(boosts) < 0.04 * 170_000.0


def test_turbocharging_adds_torque_and_backpressure():
    cfg_na = _cfg(use_idle_air_control=False, compression_ratio=9.0)
    na, _ = _hold_speed(cfg_na, 3500.0, 1.0, settle_s=1.0)
    pt = Powertrain(_turbo_cfg())
    pt.engine.start()
    c = pt.controls
    c.ignition, c.throttle, c.dyno_enabled, c.dyno_rpm = True, 1.0, True, 3500.0
    _run(pt, 5.0)
    e = pt.last.engine
    assert pt.last.dyno_torque > 1.3 * na["tq"]
    assert e.exhaust_pressure > 1.4 * na["exh"]                     # the turbine is the restriction
    assert e.exhaust_post_pressure_pa < e.exhaust_pressure


def test_smaller_turbine_spools_faster_and_raises_backpressure():
    def run(area):
        pt = Powertrain(_turbo_cfg(turbo_turbine_area_m2=area, turbo_target_boost_pa=400_000.0))
        pt.engine.start()
        c = pt.controls
        c.ignition, c.throttle, c.dyno_enabled, c.dyno_rpm = True, 1.0, True, 3000.0
        _run(pt, 1.0)
        return pt.last.engine
    small, large = run(1.6e-4), run(5.0e-4)
    assert small.exhaust_pressure > large.exhaust_pressure
    assert small.boost_pressure > large.boost_pressure              # quicker spool at the same time


def test_closed_throttle_vents_the_charge_air():
    pt = Powertrain(_turbo_cfg())
    pt.engine.start()
    c = pt.controls
    c.ignition, c.throttle, c.dyno_enabled, c.dyno_rpm = True, 1.0, True, 3500.0
    _run(pt, 3.0)
    peak = pt.last.engine.boost_pressure
    c.throttle = 0.0
    _run(pt, 0.6)
    assert pt.last.engine.boost_pressure < 0.75 * peak              # blow-off valve, not trapped boost


def test_vr38_file_is_turbocharged():
    le = load_engine_file(resolve_engine("vr38"))
    assert le.engine.use_turbo and le.engine.turbo_count == 2
    pt = Powertrain(le.engine, le.vehicle)
    pt.engine.start()
    c = pt.controls
    c.ignition, c.throttle, c.dyno_enabled, c.dyno_rpm = True, 1.0, True, 4000.0
    _run(pt, 4.0)
    assert 150_000.0 < pt.last.engine.boost_pressure < 230_000.0


# ── Two-zone burn ─────────────────────────────────────────────────────────

def _fired_cycle(two_zone, spark=25.0, rpm=3000.0):
    from tests.test_resolved_physics import _run_cycle
    cfg = EngineConfig()
    cfg.use_two_zone = two_zone
    cyl, peak_p, _ = _run_cycle(cfg, rpm=rpm, dtheta_deg=1.0, spark_btdc=spark)
    return cfg, cyl, peak_p


def test_two_zone_agrees_with_single_zone_on_work_and_pressure():
    _, one, p1 = _fired_cycle(False)
    _, two, p2 = _fired_cycle(True)
    assert two.work_net == pytest.approx(one.work_net, rel=0.03)
    assert p2 == pytest.approx(p1, rel=0.12)


def test_burned_zone_is_hotter_than_the_mean_and_makes_the_no():
    _, one, _ = _fired_cycle(False)
    _, two, _ = _fired_cycle(True)
    assert two.last_peak_burned_temperature_k > two.last_peak_temperature_k + 100.0
    assert 2500.0 < two.last_peak_burned_temperature_k < 3200.0
    assert 500.0 < two.last_no_ppm < 20_000.0
    assert one.last_no_ppm == 0.0                                    # single zone has no zones to form it in


def test_nox_rises_with_spark_advance():
    no = [_fired_cycle(True, spark=s)[1].last_no_ppm for s in (5.0, 20.0, 40.0)]
    assert no[0] < no[1] < no[2]


def test_end_gas_is_colder_than_the_mean_and_feeds_knock():
    cfg = EngineConfig()
    cfg.use_knock_model = True
    cyl = Cylinder(1, cfg)
    cyl.initialize_charge(4.2e-4, 330.0, 180.0)
    cyl.begin_cycle(2.9e-5, 335.0, 55.0)
    rpm, dth = 3000.0, 1.0
    dt = dth / (6.0 * rpm)
    seen = []
    for k in range(300, 380):
        cyl.step(k * dth + 0.5, dth, dt, rpm, 0.0, 293.0, 0.0, 700.0)
        if cyl._zone_active:
            seen.append((cyl.unburned_temperature_k, cyl.temperature, cyl._tb))
    assert seen
    t_u, t_mean, t_b = max(seen, key=lambda x: x[2])
    assert t_u < t_mean < t_b
    assert cyl.knock_integral > 0.0


def test_zones_merge_conserving_energy():
    cfg = EngineConfig()
    cyl = Cylinder(1, cfg)
    cyl.initialize_charge(4.2e-4, 330.0, 180.0)
    cyl.begin_cycle(2.9e-5, 335.0, 55.0)
    rpm, dth = 3000.0, 1.0
    dt = dth / (6.0 * rpm)
    for k in range(300, 345):
        cyl.step(k * dth + 0.5, dth, dt, rpm, 0.0, 293.0, 0.0, 700.0)
    assert cyl._zone_active
    mu, mb, yu = cyl._mu, cyl._mb, cyl._yu
    u_zones = (mu * internal_energy(cyl._tu, yu, cyl._uu)
               + mb * internal_energy(cyl._tb, 1.0))
    cyl._merge_zones()
    assert not cyl._zone_active
    y = cyl.burned_mass / cyl.mass
    u_merged = cyl.mass * internal_energy(cyl.temperature, y, cyl._uu)
    assert u_merged == pytest.approx(u_zones, rel=2e-3, abs=2.0)


# ── Variable valve timing ─────────────────────────────────────────────────

def test_cam_advance_moves_the_events_and_the_lift_curve():
    cfg = _cfg()
    cyl = Cylinder(1, cfg)
    base_ivc, base_exo = cyl._ivc_deg, cyl._exhaust_open
    cyl.set_cam_phase(20.0, 10.0)
    assert cyl._ivc_deg == pytest.approx((base_ivc - 20.0) % 720.0)
    assert cyl._exhaust_open == pytest.approx((base_exo - 10.0) % 720.0)
    assert cyl.intake_open_deg == pytest.approx((cyl.intake_valve.open_deg - 20.0) % 720.0)
    # The shifted lift at angle a equals the original lift at a + advance.
    dt = 2.0 / (6.0 * 3000.0)
    cyl.initialize_charge(3.0e-5, 400.0, 180.0)
    cyl.step(100.0, 2.0, dt, 3000.0, 90_000.0, 293.0, 105_000.0, 700.0)
    assert cyl.lift_intake == pytest.approx(cyl.intake_valve.lift(120.0), abs=1e-4)
    cyl.set_cam_phase(0.0, 0.0)
    assert not cyl._cam_on


def test_zero_phase_changes_nothing():
    a = Powertrain(_cfg(use_idle_air_control=False))
    b = Powertrain(_cfg(use_idle_air_control=False, vvt_intake_min_deg=0.0, vvt_intake_max_deg=30.0))
    for pt in (a, b):
        pt.engine.start()
        c = pt.controls
        c.ignition, c.throttle, c.dyno_enabled, c.dyno_rpm = True, 1.0, True, 3000.0
        c.vvt_override, c.vvt_intake_deg = (pt is b), 0.0
        _run(pt, 0.5)
    assert a.last.dyno_torque == pytest.approx(b.last.dyno_torque, rel=1e-6)


def _vvt_point(rpm, adv, **over):
    cfg = _cfg(use_idle_air_control=False, vvt_intake_min_deg=-30.0, vvt_intake_max_deg=45.0, **over)
    pt = Powertrain(cfg)
    pt.engine.start()
    c = pt.controls
    c.ignition, c.throttle, c.dyno_enabled, c.dyno_rpm = True, 1.0, True, rpm
    c.vvt_override, c.vvt_intake_deg = True, adv
    _run(pt, 1.6)
    tq = ve = 0.0
    for _ in range(400):
        o = pt.step(0.001)
        tq += o.dyno_torque
        ve += o.engine.volumetric_efficiency
    return tq / 400, ve / 400


def test_advancing_a_late_closing_cam_restores_low_speed_breathing():
    """A cam that closes late (85° ABDC) loses charge back up the runner at low speed;
    advancing it puts the closing back where the engine can use it."""
    late = _vvt_point(1500.0, 0.0, intake_close_abdc_deg=85.0)
    adv = _vvt_point(1500.0, 35.0, intake_close_abdc_deg=85.0)
    assert adv[1] > 1.05 * late[1]
    assert adv[0] > 1.05 * late[0]


def test_over_advancing_the_cam_costs_the_top_end():
    best = _vvt_point(6000.0, 15.0)
    over = _vvt_point(6000.0, 45.0)
    assert best[0] > 1.1 * over[0]


def test_vvt_actuator_is_rate_limited_and_scheduled():
    cfg = _cfg(vvt_intake_min_deg=0.0, vvt_intake_max_deg=30.0, vvt_rate_deg_per_s=100.0)
    eng = Engine(cfg)
    eng.start()
    # idle, light load: the schedule holds the minimum
    for _ in range(300):
        eng.update(EngineInput(throttle=0.0, ignition=True), 0.001)
    assert eng._vvt_in < 0.3
    # override: moves at the actuator rate, not instantly
    for _ in range(100):
        eng.update(EngineInput(throttle=0.0, ignition=True, vvt_intake_deg=25.0), 0.001)
    assert eng._vvt_in == pytest.approx(10.0, abs=0.5)
    # the ECU schedule: mid-speed full load brings the cam in, redline backs it out
    mid = eng.vvt_schedule(0.40 * cfg.redline_rpm, 1.0)[0]
    top = eng.vvt_schedule(0.98 * cfg.redline_rpm, 1.0)[0]
    assert mid > 0.8 * 30.0 > top > 0.0
    assert eng.vvt_schedule(0.40 * cfg.redline_rpm, 0.25)[0] == 0.0


def test_shipped_vvt_engines_schedule_inside_their_range():
    for ref in ("1nz", "vr38"):
        cfg = load_engine_file(resolve_engine(ref)).engine
        assert cfg.vvt_intake_max_deg > cfg.vvt_intake_min_deg
        eng = Engine(cfg)
        for r in (0.1, 0.4, 0.8, 1.0):
            for load in (0.2, 1.0):
                a, _ = eng.vvt_schedule(r * cfg.redline_rpm, load)
                assert cfg.vvt_intake_min_deg <= a <= cfg.vvt_intake_max_deg


# ── Misfire / partial burn ────────────────────────────────────────────────

def test_burn_quality_falls_with_dilution_and_mixture():
    cfg = _cfg()
    cyl = Cylinder(1, cfg)
    cyl._fuel_in_cycle = 3.0e-5
    m = 5.0e-4
    fresh = lambda y, lam: (m, y * m)
    assert cyl._burn_quality(m, 0.05 * m) == pytest.approx(1.0)
    q_mid = cyl._burn_quality(m, 0.30 * m)
    assert 0.1 < q_mid < 0.9
    assert cyl._burn_quality(m, 0.45 * m) == pytest.approx(0.0, abs=0.01)
    # lean: more air for the same fuel
    cyl._fuel_in_cycle = 3.0e-5 / 1.5
    assert cyl._burn_quality(m, 0.0) < 0.5
    assert _fall(0.2, 0.2, 0.4) == 0.0 and _fall(0.4, 0.2, 0.4) == 1.0
    assert 0.0 < _fall(0.3, 0.2, 0.4) < 1.0
    assert _fall(0.5, 0.8, 0.5) == 1.0 and _fall(0.8, 0.8, 0.5) == 0.0   # a rich-side limit falls the other way


def test_a_cycle_below_the_stability_limit_misfires_and_releases_nothing():
    cfg = _cfg(burn_misfire_quality=0.9)       # any real dilution is a misfire
    cyl = Cylinder(1, cfg)
    cyl.initialize_charge(4.5e-4, 330.0, 250.0, burned_fraction=0.35)
    cyl.begin_cycle(3.0e-5, 335.0, 55.0, air_mass_ref_kg=4.4e-4)
    cyl._fuel_in_cycle = 3.0e-5                # (no intake stroke in this harness: book the fuel)
    cyl.fuel_mass = 3.0e-5
    dth, rpm = 2.0, 1500.0
    dt = dth / (6.0 * rpm)
    for k in range(125, 215):                  # 250° .. 430°, valves shut throughout
        cyl.step(k * dth + 1.0, dth, dt, rpm, 0.0, 293.0, 0.0, 700.0)
    assert cyl.misfired and cyl.misfire_count == 1
    assert cyl.heat_released_total == 0.0
    # the same charge, undiluted, burns
    ok = Cylinder(1, cfg)
    ok.initialize_charge(4.5e-4, 330.0, 250.0)
    ok.begin_cycle(3.0e-5, 335.0, 55.0, air_mass_ref_kg=4.4e-4)
    ok._fuel_in_cycle = 3.0e-5
    ok.fuel_mass = 3.0e-5
    for k in range(125, 215):
        ok.step(k * dth + 1.0, dth, dt, rpm, 0.0, 293.0, 0.0, 700.0)
    assert not ok.misfired and ok.heat_released_total > 400.0


def test_the_intake_runner_returns_reverted_gas_as_residual():
    """At idle the manifold is at deep vacuum and burned gas is pushed up the runner; it comes back."""
    pt = Powertrain(_cfg())
    pt.engine.start()
    pt.controls.ignition = True
    _run(pt, 4.0)
    res = pt.engine.bank.residual_fraction
    assert 0.07 < res < 0.30                                  # real idle: 10-25 %
    # a wide-open engine at speed carries much less
    wot, p2 = _hold_speed(_cfg(use_idle_air_control=False), 3000.0, 1.0)
    assert p2.engine.bank.residual_fraction < 0.6 * res


def test_dilute_idle_misfires_but_a_normal_one_does_not():
    ok = Powertrain(_cfg())
    ok.engine.start()
    ok.controls.ignition = True
    _run(ok, 5.0)
    assert ok.engine.bank.misfire_rate < 0.02
    assert ok.engine.bank.burn_quality > 0.9
    # an engine whose burn limits sit below its idle dilution cannot hold combustion
    bad = Powertrain(_cfg(burn_residual_lo=0.02, burn_residual_hi=0.08))
    bad.engine.start()
    bad.controls.ignition = True
    _run(bad, 3.0)
    assert bad.engine.bank.misfire_rate > 0.3


def test_partial_burn_raises_hc_and_is_reported():
    cfg = _cfg(compute_emissions=True, burn_residual_lo=0.08, burn_residual_hi=0.45)
    pt = Powertrain(cfg)
    pt.engine.start()
    pt.controls.ignition = True
    _run(pt, 5.0)
    hc_part = pt.last.engine.hc_ppm
    cfg2 = _cfg(compute_emissions=True)
    pt2 = Powertrain(cfg2)
    pt2.engine.start()
    pt2.controls.ignition = True
    _run(pt2, 5.0)
    assert pt.engine.bank.burn_quality < pt2.engine.bank.burn_quality
    assert hc_part > pt2.last.engine.hc_ppm


def test_dilution_slows_the_burn():
    eng = Engine(_cfg())
    eng.start()
    for _ in range(300):
        eng.update(EngineInput(throttle=0.15, ignition=True), 0.001)
    base = eng.last_output.combustion_duration_deg
    eng.bank.residual_fraction = 0.35
    eng.update(EngineInput(throttle=0.15, ignition=True), 0.001)
    assert eng.last_output.combustion_duration_deg > base
