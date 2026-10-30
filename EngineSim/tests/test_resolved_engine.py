"""Tests for the crank-resolved engine core wired into Engine.update().

These cover what the unit tests in test_resolved_physics.py cannot: that the
cylinders, both plenums, the ECU fuel metering and the crank dynamics work as one
engine — plus regression tests for defects found while integrating them
(see UPDATES.md §6).
"""

from __future__ import annotations

import copy
import math
import statistics

import pytest

from engine_sim.config import EngineConfig
from engine_sim.cylinder import Cylinder
from engine_sim.cylinder_bank import CylinderBank
from engine_sim.engine import Engine, EngineInput
from engine_sim.engine_file import EngineFileError, load_engine_file, resolve_engine
from engine_sim.exhaust_system import ExhaustSystem
from engine_sim.heat_transfer import chamber_surface_area, woschni_heat_transfer_coeff
from engine_sim.powertrain import Powertrain
from engine_sim.telemetry import TelemetryRecorder
from engine_sim.valve_profile import ValveProfile


def _cfg(**over) -> EngineConfig:
    """A 1NZ-like inline four that idles with the idle-air valve."""
    cfg = EngineConfig(
        bore_m=0.075, stroke_m=0.0847, rod_length_m=0.1325, compression_ratio=10.5,
        engine_inertia_kgm2=0.10, idle_rpm=800.0, stall_rpm=350.0, redline_rpm=6400.0,
        limiter_type="fuel_cut", throttle_max_area_m2=0.0022,
        use_manifold_dynamics=True, use_idle_air_control=True,
        starter_stall_torque_nm=200.0, use_resolved_cylinders=True,
        dfco_enter_ratio=1.5, dfco_resume_ratio=1.2,
        crank_angle_resolution_deg=4.0,
        use_native_core=False,     # these tests pin the Python core's budget/resolution; test_native_* cover the compiled one
    )
    for k, v in over.items():
        setattr(cfg, k, v)
    return cfg


def _hold_speed(cfg: EngineConfig, rpm: float, throttle: float, settle_s: float = 0.9,
                measure_s: float = 0.4, dt: float = 0.001):
    """Run the engine pinned at a speed by a dyno and average what it reports."""
    pt = Powertrain(cfg)
    pt.engine.start()
    c = pt.controls
    c.ignition, c.throttle, c.dyno_enabled, c.dyno_rpm = True, throttle, True, rpm
    for _ in range(int(settle_s / dt)):
        pt.step(dt)
    acc = {"tq": 0.0, "ve": 0.0, "lam": 0.0, "map": 0.0, "exh": 0.0, "imep": 0.0,
           "pmep": 0.0, "peak": 0.0}
    n = int(measure_s / dt)
    for _ in range(n):
        o = pt.step(dt)
        e = o.engine
        acc["tq"] += o.dyno_torque
        acc["ve"] += e.volumetric_efficiency
        acc["lam"] += e.lambda_val
        acc["map"] += e.manifold_pressure
        acc["exh"] += e.exhaust_pressure
        acc["imep"] += e.imep_net_bar
        acc["pmep"] += e.pmep_bar
        acc["peak"] += e.peak_pressure_pa
    return {k: v / n for k, v in acc.items()}, pt


# ── Wiring ────────────────────────────────────────────────────────────────

def test_resolved_core_is_opt_in():
    """The cycle-averaged path stays the default; the flag switches the core."""
    assert Engine().resolved is False
    assert Engine(_cfg()).resolved is True
    assert Engine(_cfg(engine_type="CI")).resolved is False      # SI-only core


def test_engine_drives_torque_from_the_cylinders():
    eng = Engine(_cfg())
    eng.start()
    tq = []
    for i in range(500):
        out = eng.update(EngineInput(throttle=0.3, ignition=True), 0.001)
        if i >= 300:
            tq.append(out.indicated_torque)
    assert out.resolved
    assert len(out.cylinder_pressures_pa) == 4
    assert out.exhaust_pressure > 0 and out.exhaust_temperature > 400.0
    assert out.imep_net_bar > 1.0
    # Instantaneous torque pulses with the firing; its mean is what drives the crank.
    assert statistics.mean(tq) > 10.0


def test_torque_contract_no_double_pumping():
    """gross = net + pumping on the cylinders, and the engine subtracts pumping once."""
    eng = Engine(_cfg())
    eng.start()
    for _ in range(600):
        out = eng.update(EngineInput(throttle=0.15, ignition=True), 0.001)
    bank = eng.bank
    assert bank.gross_torque == pytest.approx(bank.indicated_torque + bank.pumping_torque, rel=1e-9)
    assert out.pumping_torque == pytest.approx(bank.pumping_torque, rel=1e-9)
    expected_net = (out.indicated_torque - out.friction_torque - out.pumping_torque
                    - out.accessory_torque - out.load_torque + out.governor_torque
                    + out.starter_torque)
    assert out.net_torque == pytest.approx(expected_net, rel=1e-9)


def test_pumping_loss_follows_the_throttle():
    part, _ = _hold_speed(_cfg(use_idle_air_control=False), 3000.0, 0.15)
    wot, _ = _hold_speed(_cfg(use_idle_air_control=False), 3000.0, 1.0)
    assert part["pmep"] > 3.0 * wot["pmep"]
    assert part["map"] < 0.7 * wot["map"]


# ── Gas exchange emerges instead of being tabulated ───────────────────────

def test_volumetric_efficiency_is_measured_not_looked_up():
    res, _ = _hold_speed(_cfg(use_idle_air_control=False), 3000.0, 1.0)
    assert 0.6 < res["ve"] < 1.05
    assert abs(res["ve"] - 0.92 * (1.0 - 0.5 * (3000.0 / 6000.0) ** 1.5)) > 0.02


def test_valve_size_changes_high_rpm_breathing():
    """Small valves choke at speed — a lever the old VE table never had."""
    big, _ = _hold_speed(_cfg(use_idle_air_control=False), 6000.0, 1.0, settle_s=0.8)
    small, _ = _hold_speed(
        _cfg(use_idle_air_control=False, intake_head_dia_m=0.018, intake_port_dia_m=0.018),
        6000.0, 1.0, settle_s=0.8)
    assert small["ve"] < 0.85 * big["ve"]
    assert small["tq"] < 0.85 * big["tq"]


def test_exhaust_restriction_raises_backpressure_and_costs_torque():
    free, _ = _hold_speed(_cfg(use_idle_air_control=False, exhaust_tail_dia_m=0.060), 4500.0, 1.0)
    pinched, _ = _hold_speed(_cfg(use_idle_air_control=False, exhaust_tail_dia_m=0.026), 4500.0, 1.0)
    assert pinched["exh"] > free["exh"] + 20_000.0
    assert pinched["pmep"] > free["pmep"]
    assert pinched["tq"] < free["tq"]


def test_manifold_pressure_pulses_with_the_cylinders():
    eng = Engine(_cfg())
    eng.start()
    lo, hi = [], []
    for _ in range(600):
        eng.update(EngineInput(throttle=0.3, ignition=True), 0.001)
        lo.append(eng.bank.manifold_pressure_min)
        hi.append(eng.bank.manifold_pressure_max)
    assert max(hi[-200:]) - min(lo[-200:]) > 300.0
    assert eng.bank.map_filtered == pytest.approx(eng.state.manifold_pressure, rel=0.15)


def test_air_fuel_ratio_stays_near_stoich_without_a_lambda_loop():
    """Fuel is metered against a learned VE; the cylinders' real air must agree."""
    for rpm, thr in ((1500.0, 0.2), (3000.0, 0.5), (4500.0, 1.0)):
        res, _ = _hold_speed(_cfg(use_idle_air_control=False), rpm, thr, settle_s=1.2)
        assert 0.9 < res["lam"] < 1.1, (rpm, res["lam"])


def test_torque_curve_has_the_right_shape():
    tq = [_hold_speed(_cfg(use_idle_air_control=False), rpm, 1.0)[0]["tq"]
          for rpm in (1500.0, 3500.0, 6000.0)]
    assert tq[0] > 80.0 and tq[1] > 80.0
    assert tq[2] < tq[1]


# ── Firing order and phase ────────────────────────────────────────────────

def test_cylinders_peak_in_firing_order_and_evenly():
    eng = Engine(_cfg())
    eng.start()
    eng.state.omega = 3000.0 * math.pi / 30.0
    dt = 0.0005
    for _ in range(int(0.12 / dt)):
        eng.update(EngineInput(throttle=0.6, ignition=True), dt)
    best = [0.0] * 4
    seen = [0.0] * 4
    cycle_s = 2.0 * 60.0 / 3000.0
    for _ in range(int(cycle_s / dt)):
        out = eng.update(EngineInput(throttle=0.6, ignition=True), dt)
        for i, p in enumerate(out.cylinder_pressures_pa):
            if p > best[i]:
                best[i], seen[i] = p, out.crank_angle_deg
    order = sorted(range(4), key=lambda i: seen[i])
    assert [i + 1 for i in order] in ([1, 3, 4, 2], [3, 4, 2, 1], [4, 2, 1, 3], [2, 1, 3, 4])
    gaps = [((seen[order[(k + 1) % 4]] - seen[order[k]]) % 720.0) for k in range(4)]
    assert all(abs(g - 180.0) < 30.0 for g in gaps)


def test_odd_fire_intervals_change_the_phasing():
    cfg = _cfg(num_cylinders=2, firing_order=[1, 2], cylinder_bank=[0, 0],
               firing_intervals_deg=[270.0, 450.0])
    assert CylinderBank(cfg)._fire == pytest.approx([0.0, 270.0])


# ── Energy and mass ───────────────────────────────────────────────────────

def test_cylinder_energy_balance_closes_over_a_fired_cycle():
    cfg = _cfg()
    bank = CylinderBank(cfg)
    bank.prime(0.0, 3000.0, 90_000.0, cfg.intake_temperature_k, 25.0, 60.0, cycles=4)
    cyl = bank.cylinders[0]
    q_fuel = cyl.last_cycle_heat_release_j
    work, wall = cyl.last_cycle_work_j, cyl.last_cycle_heat_loss_j
    assert q_fuel > 800.0
    # Slightly optimistic on purpose-of-record: no dissociation, crevice or blow-by
    # loss, so net indicated efficiency lands ~4 points above a real engine (README §11).
    assert 0.25 < work / q_fuel < 0.50
    assert 0.08 < wall / q_fuel < 0.30
    assert work + wall < q_fuel


def test_mass_is_conserved_across_the_valves():
    cfg = _cfg()
    cyl = Cylinder(1, cfg)
    cyl.initialize_charge(3.0e-5, 700.0, 180.0)
    cyl.begin_cycle(0.0, 335.0, 60.0)
    dtheta, rpm = 2.0, 3000.0
    dt = dtheta / (6.0 * rpm)
    m0 = cyl.mass
    through_in = through_ex = 0.0
    for i in range(360):
        cyl.step((i * dtheta + 1.0) % 720.0, dtheta, dt, rpm, 90_000.0, 293.0, 105_000.0, 700.0,
                 101_325.0)
        through_in += cyl.mass_flow_intake * dt
        through_ex += cyl.mass_flow_exhaust * dt
    assert cyl.mass == pytest.approx(m0 + through_in + through_ex, rel=1e-6)
    assert through_in > 0.0 and through_ex < 0.0


def test_fuel_rides_in_with_the_air():
    cfg = _cfg()
    cyl = Cylinder(1, cfg)
    cyl.initialize_charge(2.0e-5, 400.0, 180.0, burned_fraction=1.0)     # the residual is burned gas
    air_ref = 4.0e-4
    cyl.begin_cycle(air_ref / cfg.stoich_afr, 335.0, 60.0, air_mass_ref_kg=air_ref)
    dtheta, rpm = 2.0, 2500.0
    dt = dtheta / (6.0 * rpm)
    for i in range(int(250 / dtheta)):          # 710° → IVC at 230° (next cycle frame)
        a = (710.0 + i * dtheta) % 720.0
        cyl.step((a + 1.0) % 720.0, dtheta, dt, rpm, 95_000.0, 293.0, 100_000.0, 600.0, 101_325.0)
    assert cyl.trapped_air_kg > 1.0e-4
    assert cyl._fuel_in_cycle * cfg.stoich_afr == pytest.approx(cyl.trapped_air_kg, rel=0.05)


def test_late_fuel_load_does_not_wait_a_whole_cycle():
    cfg = _cfg()
    cyl = Cylinder(1, cfg)
    cyl.initialize_charge(2.0e-5, 400.0, 180.0)
    cyl.begin_cycle(0.0, 335.0, 60.0)
    assert not cyl.fuel_loaded
    cyl.load_fuel(3.0e-5, 4.0e-4)
    assert cyl.fuel_loaded


def test_a_cycle_that_starts_after_the_spark_angle_does_not_fire_at_once():
    """Cycles begin at intake-valve opening (~710°), past an idle spark at ~355°."""
    cfg = _cfg()
    cyl = Cylinder(1, cfg)
    cyl.initialize_charge(5.0e-5, 500.0, 180.0)
    air_ref = 1.0e-4
    cyl.begin_cycle(air_ref / 14.7, 355.0, 30.0, air_mass_ref_kg=air_ref)
    dt = 2.0 / (6.0 * 800.0)
    for k in range(5):
        cyl.step(711.0 + 2.0 * k, 2.0, dt, 800.0, 40_000.0, 293.0, 101_325.0, 500.0, 101_325.0)
    assert not cyl.fired_this_cycle
    assert cyl.heat_released_total == 0.0


# ── Defects fixed while integrating (see UPDATES.md §6) ───────────────────

def test_woschni_pins_kpa_basis_and_temperature_exponent():
    bore, p, T, sp = 0.08, 2.0e6, 1500.0, 10.0
    h = woschni_heat_transfer_coeff(
        bore, p, T, sp, 2.0e-4, 5.0e-5, 1.0e5, 300.0, p, combustion_occurred=False,
        displaced_volume=4.0e-4)
    w = 2.28 * sp
    expect = 3.26 * bore ** -0.2 * (p / 1e3) ** 0.8 * T ** -0.55 * w ** 0.8
    assert h == pytest.approx(expect, rel=1e-9)
    assert h > 5.0 * 3.26 * bore ** -0.2 * (p / 1e5) ** 0.8 * T ** -0.55 * w ** 0.8


def test_woschni_gas_exchange_uses_the_larger_c1():
    common = dict(bore=0.08, pressure=1.2e5, temperature=400.0, mean_piston_speed=10.0,
                  cylinder_volume=3.0e-4, clearance_volume=5.0e-5, reference_pressure=1e5,
                  reference_temp=300.0, motoring_pressure=1.2e5, displaced_volume=4.0e-4)
    hx = woschni_heat_transfer_coeff(**common, gas_exchange=True)
    hc = woschni_heat_transfer_coeff(**common, gas_exchange=False)
    assert hx / hc == pytest.approx((6.18 / 2.28) ** 0.8, rel=1e-9)


def test_woschni_combustion_term_survives_at_tdc():
    kw = dict(bore=0.08, pressure=4.0e6, temperature=2500.0, mean_piston_speed=10.0,
              cylinder_volume=5.0e-5, clearance_volume=5.0e-5, reference_pressure=8.0e5,
              reference_temp=700.0, motoring_pressure=1.5e6, reference_volume=6.0e-5,
              combustion_occurred=True)
    fired = woschni_heat_transfer_coeff(**kw, displaced_volume=4.5e-4)
    no_vd = woschni_heat_transfer_coeff(**kw)          # legacy fallback: swept volume = 0 at TDC
    assert fired > 1.3 * no_vd


def test_chamber_area_follows_the_piston():
    bore, stroke = 0.08, 0.08
    vd = math.pi / 4 * bore ** 2 * stroke
    vc = vd / 9.0
    a_tdc = chamber_surface_area(bore, stroke, vc, clearance_volume=vc)
    a_bdc = chamber_surface_area(bore, stroke, vc + vd, clearance_volume=vc)
    assert a_bdc > 2.0 * a_tdc
    assert a_tdc == pytest.approx(math.pi * bore ** 2 / 2 + 4 * vc / bore, rel=1e-12)


def test_normalized_volume_has_the_right_rod_term():
    from engine_sim.piston_kinematics import cylinder_volume, cylinder_volume_geometry
    cfg = _cfg()
    a = (cfg.bore_m, cfg.stroke_m, cfg.rod_length_m, cfg.clearance_volume)
    for deg in range(0, 361, 10):
        assert cylinder_volume(math.radians(deg), cfg.clearance_volume, cfg.compression_ratio,
                               cfg.rod_ratio) == pytest.approx(
            cylinder_volume_geometry(math.radians(deg), *a), rel=1e-9)


def test_reversion_leaves_the_plenum_at_its_own_enthalpy():
    ex = ExhaustSystem(0.015, 0.045, initial_temperature_k=500.0)
    ex.update(-0.02, 1500.0, 1.0e-3)       # the "cylinder" temperature must be ignored
    assert ex.temperature == pytest.approx(500.0, abs=15.0)


def test_valve_ramps_must_tile_the_event():
    with pytest.raises(ValueError):
        ValveProfile(open_deg=710.0, duration_deg=240.0, ramp_open_frac=0.5, ramp_close_frac=0.4)


# ── Behaviour of the whole engine ─────────────────────────────────────────

def test_fuel_cut_gives_engine_braking_without_combustion():
    eng = Engine(_cfg())
    eng.start()
    eng.state.omega = 3500.0 * math.pi / 30.0
    for _ in range(500):
        out = eng.update(EngineInput(throttle=0.0, ignition=True), 0.001)
    assert out.fuel_cut and not out.combusting
    assert out.net_torque < 0.0
    assert out.rpm < 3500.0


def test_engine_starts_from_rest_and_idles():
    pt = Powertrain(_cfg())
    c = pt.controls
    c.ignition, c.starter = True, True
    rpms, burning = [], 0
    for i in range(7000):
        o = pt.step(0.001)
        if i == 1500:
            c.starter = False
        if i > 5000:
            rpms.append(o.engine_rpm)
            burning += o.engine.combusting
    assert min(rpms) > 600.0
    assert 650.0 < statistics.mean(rpms) < 1000.0
    assert burning > 0.8 * len(rpms)            # firing, bar the odd decel fuel cut


def test_knock_control_retards_the_spark_and_lowers_peak_pressure():
    base, _ = _hold_speed(_cfg(use_idle_air_control=False, compression_ratio=12.5), 2500.0, 1.0)
    cfg = _cfg(use_idle_air_control=False, compression_ratio=12.5, use_knock_model=True,
               fuel_octane_ron=86.0)
    knocky, pt = _hold_speed(cfg, 2500.0, 1.0, settle_s=1.5)
    assert pt.engine.last_output.knock_retard_deg > 2.0
    assert knocky["peak"] < base["peak"]


def test_resolution_barely_changes_the_answer():
    coarse, _ = _hold_speed(_cfg(use_idle_air_control=False, crank_angle_resolution_deg=8.0),
                            3000.0, 1.0)
    fine, _ = _hold_speed(_cfg(use_idle_air_control=False, crank_angle_resolution_deg=2.0),
                          3000.0, 1.0)
    assert coarse["tq"] == pytest.approx(fine["tq"], rel=0.05)


def test_live_config_swap_keeps_the_gas_state():
    eng = Engine(_cfg())
    eng.start()
    for _ in range(300):
        eng.update(EngineInput(throttle=0.3, ignition=True), 0.001)
    bank = eng.bank
    mass_before = [c.mass for c in bank.cylinders]
    tweaked = copy.deepcopy(eng.config)
    tweaked.spark_timing_deg_btdc = 20.0
    eng.set_config(tweaked)
    assert eng.bank is bank
    assert [c.mass for c in eng.bank.cylinders] == mass_before
    changed = copy.deepcopy(tweaked)
    changed.compression_ratio = 9.0
    eng.set_config(changed)
    assert eng.bank is not bank


def test_telemetry_records_the_resolved_fields():
    eng = Engine(_cfg())
    eng.start()
    rec = TelemetryRecorder()
    for i in range(200):
        rec.record(i * 0.001, eng.update(EngineInput(throttle=0.3, ignition=True), 0.001))
    e = rec.entries[-1]
    assert e.resolved and e.exhaust_pressure > 0 and e.peak_pressure_pa > 0


def test_real_engine_files_load_and_start_resolved():
    for ref in ("454", "1nz", "vr38"):
        le = load_engine_file(resolve_engine(ref))
        cfg = le.engine
        cfg.use_resolved_cylinders = True
        pt = Powertrain(cfg, le.vehicle)
        c = pt.controls
        c.ignition, c.starter = True, True
        for i in range(1800):
            pt.step(0.001)
            if i == 1500:
                c.starter = False
        assert pt.engine.last_output.resolved
        assert pt.last.engine_rpm > 300.0, ref


def test_engine_file_rejects_resolved_compression_ignition(tmp_path):
    src = open(resolve_engine("1nz")).read()
    bad = tmp_path / "bad.toml"
    # (the shipped file already sets use_resolved_cylinders = true)
    bad.write_text(src.replace("[engine]", '[engine]\nengine_type = "CI"', 1))
    with pytest.raises(EngineFileError, match="use_resolved_cylinders"):
        load_engine_file(str(bad))


def test_engine_file_rejects_ramps_that_do_not_add_up(tmp_path):
    src = open(resolve_engine("1nz")).read()
    bad = tmp_path / "bad.toml"
    bad.write_text(src.replace("[valves]", "[valves]\nintake_ramp_open_frac = 0.3\nintake_ramp_close_frac = 0.3", 1))
    with pytest.raises(EngineFileError, match="add up to 1"):
        load_engine_file(str(bad))


# ── v0.6: speed, heat release, variable-c_v gas, ECU air model ────────────

def _fired_cylinder(rpm=3000.0, dth=2.0, p_man=90_000.0, cycles=4, spark=25.0, **cfg_over):
    """One cylinder run to periodic steady state with delivered fuel; returns it after the last cycle starts."""
    from engine_sim.cylinder import Cylinder
    cfg = _cfg(**cfg_over)
    cyl = Cylinder(1, cfg)
    cyl.initialize_charge(3.0e-5, 700.0, 180.0, burned_fraction=1.0)
    air = 4.5e-4 * p_man / 101_325.0
    dt = dth / (6.0 * rpm)
    return cfg, cyl, air, dt


def test_heat_release_follows_the_wiebe_curve_with_no_end_of_burn_dump():
    """Each substep releases dx·(whole charge energy); the sum is exactly the pool.

    (It used to release dx·(energy *remaining*), which freed only 1 − e^(−x) of the fuel
    along the curve and dumped the last third in the single step where the burn ended.)
    """
    from engine_sim.cylinder import Cylinder
    cfg = _cfg()
    cyl = Cylinder(1, cfg)
    cyl.initialize_charge(4.5e-4, 330.0, 180.0)
    cyl.begin_cycle(3.0e-5, 335.0, 60.0)
    pool = 3.0e-5 * cfg.lower_heating_value * cfg.combustion_efficiency
    dth, rpm = 2.0, 3000.0
    dt = dth / (6.0 * rpm)
    steps = []
    for k in range(150, 200):
        before = cyl.heat_released_total
        cyl.step(k * dth + 1.0, dth, dt, rpm, 0.0, 293.0, 0.0, 700.0)
        steps.append((k * dth, cyl.heat_released_total - before))
    released = sum(q for _, q in steps)
    assert released == pytest.approx(pool, rel=1e-6)
    assert max(q for _, q in steps) < 0.10 * pool          # no lump anywhere (the dump was 36 %)
    # Cumulative release tracks the Wiebe fraction at the end of each step.
    cum = 0.0
    for angle, q in steps:
        cum += q
        since = angle + 0.5 * dth - 335.0            # the burn is evaluated at the substep's mid-angle
        if 0.0 < since < 60.0:
            expect = 1.0 - math.exp(-cfg.wiebe_a * (since / 60.0) ** (cfg.wiebe_m + 1.0))
            assert cum / pool == pytest.approx(expect, abs=0.02)


def test_orifice_matches_the_reference_valve_flow():
    from engine_sim.cylinder import _orifice
    from engine_sim.valve_flow import valve_mass_flow_signed
    for pc, pp, tc, tp in ((9.0e4, 1.0e5, 400.0, 300.0), (3.0e5, 1.1e5, 900.0, 650.0),
                           (6.0e4, 1.0e5, 350.0, 300.0), (1.0e5, 1.0e5, 400.0, 300.0),
                           (1.2e6, 1.0e5, 1500.0, 700.0)):
        area, cd = 6.0e-4, 0.67
        assert _orifice(area * cd, pc, pp, tc, tp) == pytest.approx(
            valve_mass_flow_signed(pc, pp, tc, tp, area, cd), rel=1e-9, abs=1e-12)


def test_inline_woschni_matches_the_reference_correlation():
    """The cylinder inlines Woschni for speed; it must stay the same function."""
    from engine_sim.cylinder import Cylinder
    cfg = _cfg()
    cyl = Cylinder(1, cfg)
    cyl.initialize_charge(5.0e-4, 520.0, 180.0)
    cyl.begin_cycle(0.0, 335.0, 60.0)
    angle, dth, rpm = 300.0, 1.0, 3000.0
    dt = dth / (6.0 * rpm)
    v0 = cyl.volume(angle - 0.5)
    p0 = cyl.mass * 287.058 * cyl.temperature / v0
    t0 = cyl.temperature
    cyl.step(angle, dth, dt, rpm, 0.0, 293.0, 0.0, 700.0)
    volume = 0.5 * (v0 + cyl.volume(angle + 0.5))
    h_ref = woschni_heat_transfer_coeff(
        cfg.bore_m, p0, t0, cfg.stroke_m * rpm / 30.0, volume, cfg.clearance_volume,
        1.0, 293.0, p0, combustion_occurred=False, displaced_volume=cfg.displacement_per_cyl,
        gas_exchange=False)
    q_ref = h_ref * (math.pi * cfg.bore_m ** 2 / 2 + 4 * volume / cfg.bore_m) * (t0 - cfg.wall_temperature_k)
    assert cyl.wall_heat_transfer_rate == pytest.approx(q_ref, rel=2e-3)


def test_burned_gas_has_the_dissociation_signature():
    """γ falls with temperature and burned fraction; KB Fig 4-15 anchors the burned c_p."""
    from engine_sim.gas_tables import cv_mixture, gamma_mixture, internal_energy
    assert 1.33 < gamma_mixture(700.0, 0.0) < 1.37                  # KB: γ_u ≈ 1.35 in compression
    assert gamma_mixture(2500.0, 1.0) < 1.20                        # dissociation holds γ_b down
    assert gamma_mixture(1500.0, 1.0) < gamma_mixture(1500.0, 0.0)
    # c_p,b ≈ 2.45 kJ/kg·K at 2750 K, φ = 1 (the KB's sharp equilibrium peak)
    assert (cv_mixture(2750.0, 1.0) + 287.058) / 1000.0 == pytest.approx(2.45, abs=0.1)
    assert internal_energy(2000.0, 1.0) > internal_energy(1000.0, 1.0) > internal_energy(300.0, 1.0) == pytest.approx(0.0, abs=1.0)


def test_peak_temperature_and_efficiency_are_realistic():
    """With equilibrium burned gas the burn tops out near 2700 K, not the ~3400 K of an air-like gas."""
    from engine_sim.cylinder_bank import CylinderBank
    cfg = _cfg()
    bank = CylinderBank(cfg)
    bank.prime(0.0, 3000.0, 95_000.0, cfg.intake_temperature_k, 25.0, 60.0, cycles=4)
    cyl = bank.cylinders[0]
    assert 2300.0 < cyl.last_peak_temperature_k < 3000.0
    eta = cyl.last_cycle_work_j / cyl.last_cycle_heat_release_j
    assert 0.30 < eta < 0.42


def test_first_law_closes_with_variable_cv_between_ivc_and_evo():
    """ΔU of the trapped gas = Q_chem − Q_wall − W, with U from the same tables (no flow in between)."""
    from engine_sim.cylinder import Cylinder
    from engine_sim.gas_tables import internal_energy
    cfg = _cfg()
    cyl = Cylinder(1, cfg)
    cyl.initialize_charge(3.0e-5, 700.0, 180.0, burned_fraction=1.0)
    rpm, dth = 3000.0, 1.0
    dt = dth / (6.0 * rpm)
    air = 4.0e-4
    result = None
    for cycle in range(5):
        cyl.begin_cycle(air / cfg.stoich_afr, 335.0, 60.0, air_mass_ref_kg=air)
        snap = None
        for k in range(int(720.0 / dth)):
            a = (710.0 + k * dth) % 720.0
            mid = a + dth / 2.0
            if snap is None and 250.0 <= mid < 250.0 + dth:          # intake closed (IVC = 230°)
                snap = (cyl.mass, cyl.temperature, cyl.burned_mass_fraction, cyl.heat_released_total,
                        cyl.heat_loss_total, cyl.work_net)
            if snap is not None and result is None and mid >= 480.0 and mid < 480.0 + dth and cycle == 4:
                u1 = cyl.mass * internal_energy(cyl.temperature, cyl.burned_mass_fraction)
                u0 = snap[0] * internal_energy(snap[1], snap[2])
                dq = (cyl.heat_released_total - snap[3]) - (cyl.heat_loss_total - snap[4])
                dw = cyl.work_net - snap[5]
                result = (u1 - u0, dq - dw)
            cyl.step(mid % 720.0, dth, dt, rpm, 90_000.0, 293.15, 105_000.0, 700.0, 101_325.0)
            if mid >= 480.0 and mid < 481.0 and cycle < 4:
                pass
    # (heat_released/heat_loss/work_net accumulate over the whole run, so differences are the interval's)
    assert result is not None
    d_u, expected = result
    # The crankcase term (−p_cc·ΔV) is zero over a closed expansion+compression that returns
    # to a different volume only by the piston travel; allow for it and the table resolution.
    assert d_u == pytest.approx(expected, rel=0.04, abs=15.0)


def test_substeps_follow_the_cylinder_step_budget():
    cfg = _cfg(max_cylinder_steps_per_ms=12, crank_angle_resolution_deg=1.0)
    eng = Engine(cfg)
    eng.start()
    for _ in range(30):
        eng.update(EngineInput(throttle=1.0, ignition=True), 0.001)
    # 4 cylinders, budget 12 → 3 substeps per update, however fast the crank turns.
    eng.state.omega = 6000.0 * math.pi / 30.0
    eng.update(EngineInput(throttle=1.0, ignition=True), 0.001)
    assert eng.bank.substeps_last == 3
    eng.state.omega = 300.0 * math.pi / 30.0
    eng.update(EngineInput(throttle=1.0, ignition=True), 0.001)
    assert eng.bank.substeps_last == 2           # 1.8° per ms at 1° resolution


def test_realtime_governor_trims_and_restores_the_budget():
    cfg = _cfg()
    eng = Engine(cfg)
    eng.start()
    nominal = eng.bank.step_budget
    eng.set_realtime_budget(1.0e-7)                  # impossible: it must give up fidelity
    for _ in range(400):
        eng.update(EngineInput(throttle=1.0, ignition=True), 0.001)
    assert eng.bank.step_budget < nominal
    assert eng.bank.step_budget >= 2 * cfg.num_cylinders
    eng.set_realtime_budget(10.0)                    # generous: it must recover
    for _ in range(600):
        eng.update(EngineInput(throttle=1.0, ignition=True), 0.001)
    assert eng.bank.step_budget == nominal
    eng.set_realtime_budget(None)
    assert eng.bank.step_budget == nominal


def test_ecu_air_model_learns_the_cylinders_air():
    """The slope-intercept model must predict the trapped air at part and full load."""
    for rpm, thr in ((1500.0, 0.12), (3000.0, 0.3), (3000.0, 1.0), (5000.0, 1.0)):
        cfg = _cfg(use_idle_air_control=False)
        pt = Powertrain(cfg)
        pt.engine.start()
        c = pt.controls
        c.ignition, c.throttle, c.dyno_enabled, c.dyno_rpm = True, thr, True, rpm
        for _ in range(1500):
            pt.step(0.001)
        bank = pt.engine.bank
        est = bank.air_estimate(bank.map_filtered, cfg.intake_temperature_k, rpm)
        assert est == pytest.approx(bank.trapped_air_kg, rel=0.07), (rpm, thr)


def test_fast_idle_ramp_decays_to_the_idle_speed():
    eng = Engine(_cfg())
    eng.start()
    idle = eng.config.idle_rads
    assert eng._idle_target_rads() == pytest.approx(idle * (1.0 + eng.config.fast_idle_extra_ratio))
    eng._fire_time = 60.0
    assert eng._idle_target_rads() == pytest.approx(idle, rel=1e-4)
