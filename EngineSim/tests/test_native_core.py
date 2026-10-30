"""The compiled inner loop must be the Python model, only faster.

Every scenario runs the same drive through ``CylinderBank`` and ``NativeCylinderBank`` with
the same step budget and compares what the engine reports. The C code is a line-for-line
transcription built without fast-math, so the two agree to rounding (the checks use 1e-9
relative); a mismatch means a transcription slip.
"""

import pytest

from engine_sim.config import EngineConfig
from engine_sim.cylinder_bank import CylinderBank
from engine_sim.native_core import NativeCylinderBank, native_available, native_status
from engine_sim.powertrain import Powertrain
from tests.test_resolved_engine import _cfg

pytestmark = pytest.mark.skipif(not native_available(), reason=f"no compiled core: {native_status()}")


def _drive(bank, cfg, rpm, steps, p_man=40000.0, throttle=5e-4, fuel_scale=1.0,
           spark=20.0, burn=45.0, cam=None, fuel_on=True, dt=0.002, switch_at=None):
    bank.step_budget = bank._nominal_budget = cfg.max_cylinder_steps_per_ms
    theta, dth = 0.0, rpm * 6.0 * dt
    log = []
    for k in range(steps):
        if switch_at is not None and k == switch_at:
            bank.__class__ = CylinderBank       # carry on in Python from the native state
        if cam is not None:
            bank.set_cam_phase(*cam(k))
        air = bank.air_estimate(p_man, 300.0, rpm)
        bank.advance(theta, dth, dt, rpm,
                     manifold_pressure=bank.manifold_pressure if k else p_man,
                     intake_temperature=300.0, throttle_area=throttle, p_upstream=101325.0,
                     fuel_per_cycle=(fuel_scale * air / cfg.stoich_afr if fuel_on else 0.0),
                     air_ref=air, spark_btdc=spark, burn_deg=burn)
        theta = (theta + dth) % 720.0
        log.append((bank.indicated_torque, bank.pumping_torque, bank.manifold_pressure,
                    bank.exhaust.pressure, bank.exhaust.temperature, bank.map_filtered,
                    bank.air_mass_flow, bank.heat_loss_w))
    return log


def _same(a, b, tol=1e-9):
    assert len(a) == len(b)
    for x, y in zip(a, b):
        for u, v in zip(x, y):
            assert v == pytest.approx(u, rel=tol, abs=1e-9)


def _banks(cfg):
    cfg.native_crank_angle_resolution_deg = cfg.crank_angle_resolution_deg   # same substeps on both sides
    return CylinderBank(cfg), NativeCylinderBank(cfg)


def _final_state_matches(py, nat, tol=1e-9):
    for cp, cn in zip(py.cylinders, nat.cylinders):
        for name in ("mass", "temperature", "pressure", "burned_mass", "fuel_mass", "work_net",
                     "work_pumping", "heat_loss_total", "last_cycle_work_j", "last_peak_pressure_pa",
                     "knock_integral", "residual_at_spark", "burn_quality", "last_no_ppm",
                     "trapped_air_kg", "_runner_burned"):
            assert getattr(cn, name) == pytest.approx(getattr(cp, name), rel=tol, abs=1e-12), name
        for name in ("ivc_events", "cycles_completed", "misfire_count", "knock_events"):
            assert getattr(cn, name) == getattr(cp, name), name
    assert nat.cylinders[0].p_trace == pytest.approx(py.cylinders[0].p_trace, rel=tol)


@pytest.mark.parametrize("rpm", [800.0, 3000.0, 6000.0])
def test_fired_cycles_match_python(rpm):
    cfg = _cfg()
    py, nat = _banks(cfg)
    _same(_drive(py, cfg, rpm, 120), _drive(nat, cfg, rpm, 120))
    _final_state_matches(py, nat)


def test_motoring_and_fuel_cut_match_python():
    cfg = _cfg()
    py, nat = _banks(cfg)
    a = _drive(py, cfg, 2500.0, 60, fuel_on=False)
    b = _drive(nat, cfg, 2500.0, 60, fuel_on=False)
    _same(a, b)
    # resume fuelling part-way: the cylinders pick fuel up mid-intake (load_fuel)
    _same(_drive(py, cfg, 2500.0, 60), _drive(nat, cfg, 2500.0, 60))
    _final_state_matches(py, nat)


def test_wide_open_throttle_with_two_zone_and_knock_match_python():
    cfg = _cfg(use_knock_model=True, compression_ratio=12.5, use_two_zone=True)
    py, nat = _banks(cfg)
    kw = dict(p_man=95000.0, throttle=2.2e-3, spark=38.0)
    _same(_drive(py, cfg, 3500.0, 140, **kw), _drive(nat, cfg, 3500.0, 140, **kw))
    _final_state_matches(py, nat)
    assert max(c.knock_integral for c in nat.cylinders) > 0.0


def test_single_zone_matches_python():
    cfg = _cfg(use_two_zone=False)
    py, nat = _banks(cfg)
    _same(_drive(py, cfg, 3000.0, 100, p_man=70000.0), _drive(nat, cfg, 3000.0, 100, p_man=70000.0))
    _final_state_matches(py, nat)


def test_misfire_and_runner_buffer_match_python():
    cfg = _cfg()
    py, nat = _banks(cfg)
    kw = dict(p_man=28000.0, fuel_scale=0.45)           # lean and diluted: partial burns, misfires
    _same(_drive(py, cfg, 800.0, 200, **kw), _drive(nat, cfg, 800.0, 200, **kw))
    _final_state_matches(py, nat)
    assert sum(c.misfire_count for c in nat.cylinders) > 0


def test_variable_cam_phase_matches_python():
    cfg = _cfg()
    py, nat = _banks(cfg)
    cam = lambda k: (25.0 if k > 40 else 0.0, -10.0 if k > 80 else 0.0)
    _same(_drive(py, cfg, 3000.0, 140, cam=cam), _drive(nat, cfg, 3000.0, 140, cam=cam))
    _final_state_matches(py, nat)


@pytest.mark.parametrize("engine_file", ["Nissan_Gt_R35.toml", "chevrolet_454.toml", "toyota_1nz_fe.toml"])
def test_shipped_engines_match_python(engine_file):
    from engine_sim.engine_file import load_engine_file
    from pathlib import Path
    path = Path(__file__).resolve().parent.parent / "config" / "engines" / engine_file
    cfg = load_engine_file(str(path)).engine
    py, nat = _banks(cfg)
    kw = dict(p_man=60000.0, throttle=1.2e-3)
    _same(_drive(py, cfg, 4500.0, 100, **kw), _drive(nat, cfg, 4500.0, 100, **kw))
    _final_state_matches(py, nat)


def test_turbo_boost_and_backpressure_match_python():
    from engine_sim.engine_file import load_engine_file
    from pathlib import Path
    path = Path(__file__).resolve().parent.parent / "config" / "engines" / "Nissan_Gt_R35.toml"
    cfg = load_engine_file(str(path)).engine
    assert cfg.use_turbo
    py, nat = _banks(cfg)
    kw = dict(p_man=100000.0, throttle=cfg.throttle_max_area_m2)
    _same(_drive(py, cfg, 4000.0, 300, **kw), _drive(nat, cfg, 4000.0, 300, **kw), tol=1e-8)
    assert nat.turbo.omega == pytest.approx(py.turbo.omega, rel=1e-8)
    assert nat.turbo.boost_pressure == pytest.approx(py.turbo.boost_pressure, rel=1e-8)
    assert nat.turbo.wastegate_position == pytest.approx(py.turbo.wastegate_position, rel=1e-8, abs=1e-12)
    assert nat.exhaust.turbine_power_w == pytest.approx(py.exhaust.turbine_power_w, rel=1e-8)
    _final_state_matches(py, nat, tol=1e-8)


def test_state_round_trips_so_python_can_take_over_mid_run():
    cfg = _cfg()
    cfg.native_crank_angle_resolution_deg = cfg.crank_angle_resolution_deg
    all_py = CylinderBank(cfg)
    mixed = NativeCylinderBank(cfg)
    _same(_drive(all_py, cfg, 3000.0, 120), _drive(mixed, cfg, 3000.0, 120, switch_at=60))
    _final_state_matches(all_py, mixed)


def test_energy_scale_is_applied_per_cylinder():
    cfg = _cfg()
    py, nat = _banks(cfg)
    for bank in (py, nat):
        bank.step_budget = bank._nominal_budget = cfg.max_cylinder_steps_per_ms
    outs = []
    for bank in (py, nat):
        theta, dt, rpm = 0.0, 0.002, 2500.0
        dth = rpm * 6.0 * dt
        for k in range(100):
            air = bank.air_estimate(50000.0, 300.0, rpm)
            bank.advance(theta, dth, dt, rpm, manifold_pressure=bank.manifold_pressure if k else 50000.0,
                         intake_temperature=300.0, throttle_area=8e-4, p_upstream=101325.0,
                         fuel_per_cycle=air / cfg.stoich_afr, air_ref=air, spark_btdc=20.0,
                         burn_deg=45.0, energy_scale=[1.0, 0.7, 1.1, 0.9])
            theta = (theta + dth) % 720.0
        outs.append([c.last_cycle_work_j for c in bank.cylinders])
    assert outs[1] == pytest.approx(outs[0], rel=1e-9)
    assert max(outs[0]) - min(outs[0]) > 0.0


def test_engine_uses_the_compiled_core_by_default_and_can_opt_out():
    cfg = _cfg(use_native_core=True)
    assert isinstance(Powertrain(cfg).engine.bank, NativeCylinderBank)
    cfg = _cfg(use_native_core=False)
    bank = Powertrain(cfg).engine.bank
    assert type(bank) is CylinderBank


def test_native_core_is_much_faster_than_python():
    import time
    cfg = _cfg()
    times = {}
    for cls in (CylinderBank, NativeCylinderBank):
        bank = cls(cfg)
        bank.step_budget = bank._nominal_budget = cfg.max_cylinder_steps_per_ms
        t0 = time.thread_time()
        _drive(bank, cfg, 5000.0, 60)
        times[cls] = time.thread_time() - t0
    assert times[NativeCylinderBank] * 4.0 < times[CylinderBank]


def test_substeps_follow_the_native_budget_and_resolution():
    import math
    from engine_sim.engine import Engine, EngineInput
    cfg = _cfg(use_native_core=True, native_steps_per_ms=12, native_crank_angle_resolution_deg=1.0)
    eng = Engine(cfg)
    eng.start()
    for _ in range(30):
        eng.update(EngineInput(throttle=1.0, ignition=True), 0.001)
    eng.state.omega = 6000.0 * math.pi / 30.0
    eng.update(EngineInput(throttle=1.0, ignition=True), 0.001)
    assert eng.bank.substeps_last == 3               # 4 cylinders, budget 12 → 3 per millisecond
    eng.state.omega = 300.0 * math.pi / 30.0
    eng.update(EngineInput(throttle=1.0, ignition=True), 0.001)
    assert eng.bank.substeps_last == 2               # 1.8° per ms at 1° resolution


def test_realtime_governor_trims_and_restores_the_native_budget():
    from engine_sim.engine import Engine, EngineInput
    cfg = _cfg(use_native_core=True)
    eng = Engine(cfg)
    eng.start()
    nominal = eng.bank.step_budget
    assert nominal >= cfg.native_steps_per_ms
    eng.set_realtime_budget(1.0e-9)
    for _ in range(400):
        eng.update(EngineInput(throttle=1.0, ignition=True), 0.001)
    assert eng.bank.step_budget < nominal
    eng.set_realtime_budget(10.0)
    for _ in range(600):
        eng.update(EngineInput(throttle=1.0, ignition=True), 0.001)
    assert eng.bank.step_budget == nominal


def test_finer_native_resolution_changes_the_answer_only_a_little():
    """The compiled core's 1° substeps against the Python core's 4°: same engine, near the same torque."""
    from tests.test_resolved_engine import _hold_speed
    coarse, _ = _hold_speed(_cfg(use_idle_air_control=False), 3000.0, 1.0)
    fine, _ = _hold_speed(_cfg(use_idle_air_control=False, use_native_core=True), 3000.0, 1.0)
    assert fine["tq"] == pytest.approx(coarse["tq"], rel=0.06)
    assert fine["lam"] == pytest.approx(coarse["lam"], abs=0.03)


@pytest.mark.parametrize("engine_file", ["toyota_1nz_fe.toml", "chevrolet_454.toml", "Nissan_Gt_R35.toml"])
def test_compiled_engines_idle_and_run_in_real_time(engine_file):
    import time
    from pathlib import Path
    from engine_sim.engine_file import load_engine_file
    path = Path(__file__).resolve().parent.parent / "config" / "engines" / engine_file
    le = load_engine_file(str(path))
    pt = Powertrain(le.engine, le.vehicle)
    assert isinstance(pt.engine.bank, NativeCylinderBank)
    pt.engine.start()
    c = pt.controls
    c.ignition, c.throttle, c.dyno_enabled, c.dyno_rpm = True, 1.0, True, 0.65 * le.engine.redline_rpm
    for _ in range(300):
        pt.step(0.001)
    t0 = time.thread_time()
    for _ in range(1000):
        pt.step(0.001)
    cpu = time.thread_time() - t0
    assert cpu < 1.0, f"{engine_file} needs {cpu:.2f} CPU-s per simulated second"


def test_boosted_v6_holds_lambda_and_boost_at_full_load():
    """The ECU's air model must not be dragged off by the boost transient (it once ran λ 1.9)."""
    from pathlib import Path
    from engine_sim.engine_file import load_engine_file
    path = Path(__file__).resolve().parent.parent / "config" / "engines" / "Nissan_Gt_R35.toml"
    le = load_engine_file(str(path))
    for rpm in (3500.0, 6600.0):
        pt = Powertrain(le.engine, le.vehicle)
        pt.engine.start()
        c = pt.controls
        c.ignition, c.throttle, c.dyno_enabled, c.dyno_rpm = True, 1.0, True, rpm
        for _ in range(int(5.0 / 0.002)):
            pt.step(0.002)
        lam, boost = [], []
        for _ in range(300):
            e = pt.step(0.002).engine
            lam.append(e.lambda_val)
            boost.append(e.boost_pressure)
        assert sum(lam) / len(lam) == pytest.approx(1.0, abs=0.04), rpm
        assert max(boost) - min(boost) < 0.05 * le.engine.turbo_target_boost_pa
        assert sum(boost) / len(boost) == pytest.approx(le.engine.turbo_target_boost_pa, rel=0.06)
