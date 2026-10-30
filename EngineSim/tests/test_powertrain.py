"""Tests for run-state logic (ignition, starter, DFCO, fuel-cut limiter),
engine braking through the drivetrain, and the dyno.

Run: python -m pytest tests/  (or python tests/test_powertrain.py)
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from engine_sim.config import EngineConfig
from engine_sim.engine import Engine, EngineInput
from engine_sim.powertrain import Powertrain, preset_chevy_454, preset_1nz_fe

DT = 0.001


def run(pt: Powertrain, seconds: float):
    out = None
    for _ in range(int(round(seconds / DT))):
        out = pt.step(DT)
    return out


def started(preset=preset_1nz_fe) -> Powertrain:
    pt = Powertrain(*preset())
    pt.controls.ignition = True
    pt.controls.starter = True
    run(pt, 1.0)
    pt.controls.starter = False
    run(pt, 3.0)
    return pt


def test_wot_torque_is_physical():
    """bmep at WOT must be in the NA SI range (~8-14 bar), not 2x it."""
    cfg = EngineConfig()
    eng = Engine(cfg)
    eng.start()
    eng.state.omega = 2000 * math.pi / 30
    o = eng.update(EngineInput(throttle=1.0), 0.0)
    brake = o.indicated_torque - o.friction_torque - o.pumping_torque - o.accessory_torque
    bmep_bar = brake * 4 * math.pi / cfg.total_displacement / 1e5
    assert 8.0 < bmep_bar < 14.0, bmep_bar


def test_starter_cranks_then_engine_catches_and_idles():
    for preset in (preset_1nz_fe, preset_chevy_454):
        pt = Powertrain(*preset())
        pt.controls.ignition = True
        pt.controls.starter = True
        out = run(pt, 0.15)
        assert not out.engine.combusting, "must crank before first fire (sync)"
        # Cranking speed is the starter's free speed (~330-400 rpm) plus the speed
        # ripple of the resolved cylinders' compression strokes.
        assert 100 < out.engine_rpm < 550, out.engine_rpm
        run(pt, 0.85)
        pt.controls.starter = False
        out = run(pt, 3.0)
        idle = pt.cfg.idle_rpm
        assert out.engine.combusting
        # (a few seconds after the first fire the resolved core is still on its fast-idle ramp)
        assert abs(out.engine_rpm - idle) < 0.3 * idle, out.engine_rpm


def test_no_start_without_ignition():
    pt = Powertrain(*preset_1nz_fe())
    pt.controls.starter = True
    out = run(pt, 1.0)
    assert not out.engine.combusting
    assert out.engine_rpm < 450
    pt.controls.starter = False
    out = run(pt, 2.0)
    assert out.engine_rpm < 1.0


def test_key_off_spins_down():
    pt = started()
    pt.controls.ignition = False
    out = run(pt, 3.0)
    assert out.engine_rpm < 1.0
    assert out.engine.fuel_mass_per_cycle == 0.0


def test_engine_braking_in_gear():
    """Closed throttle in gear: fuel is cut and the vehicle is slowed only by
    the engine's own losses reflected through the driveline."""
    pt = started()
    c = pt.controls
    c.gear = 1
    c.throttle = 0.7
    run(pt, 3.0)
    c.gear = 2
    run(pt, 2.0)
    c.throttle = 0.0
    out = run(pt, 0.5)
    v0 = out.vehicle_speed_ms
    e = out.engine
    assert e.fuel_cut and not e.combusting
    assert out.clutch_locked
    assert out.clutch_torque < 0.0, "vehicle must be driving the engine"
    losses = e.friction_torque + e.pumping_torque + e.accessory_torque
    # No combustion assist: whatever the cylinders contribute is a loss, never a gain.
    # (A motored cylinder is not exactly neutral — wall heat loss during compression
    # makes the gas work slightly negative — so allow for that, within reason.)
    assert -0.35 * losses < out.flywheel_torque + losses < 1e-6, "no combustion assist"
    assert e.governor_torque == 0.0, "no artificial braking torque"
    out = run(pt, 2.0)
    assert out.vehicle_speed_ms < v0 - 0.5, (v0, out.vehicle_speed_ms)


def test_engine_braking_stronger_than_neutral_coast():
    def coast(gear):
        pt = started()
        c = pt.controls
        c.gear = 2
        c.throttle = 0.8
        run(pt, 4.0)
        c.throttle = 0.0
        c.gear = gear
        v0 = run(pt, 0.6).vehicle_speed_ms
        return v0 - run(pt, 2.0).vehicle_speed_ms
    assert coast(2) > 2.0 * coast(0)


def test_throttle_lift_decel_is_losses_only():
    """Free-revving lift from high rpm: the idle governor must not add braking
    (it used to apply ~-390 N·m at 7000 rpm)."""
    cfg = EngineConfig()
    eng = Engine(cfg)
    eng.start()
    for _ in range(3000):
        eng.update(EngineInput(throttle=1.0), DT)
    for _ in range(50):
        o = eng.update(EngineInput(throttle=0.0), DT)
    assert o.fuel_cut
    assert o.governor_torque == 0.0
    losses = o.friction_torque + o.pumping_torque + o.accessory_torque
    assert abs(o.net_torque + losses) < 1e-6


def test_idle_governor_never_exceeds_combustion_torque():
    cfg = EngineConfig()
    eng = Engine(cfg)
    eng.start()
    for _ in range(3000):
        o = eng.update(EngineInput(throttle=0.0), DT)
        assert o.governor_torque >= -o.indicated_torque - 1e-9


def test_fuel_cut_limiter_bounces():
    pt = started()
    pt.controls.throttle = 1.0
    run(pt, 3.0)
    rpms, cuts = [], 0
    for _ in range(1000):
        o = pt.step(DT)
        rpms.append(o.engine_rpm)
        cuts += o.engine.limiter_active
    red = pt.cfg.redline_rpm
    assert max(rpms) < red + 100
    assert min(rpms) > red - pt.cfg.limiter_hysteresis_rpm - 150
    assert 0 < cuts < 1000, "limiter should cycle, not hold"


def test_stall_when_clutch_dumped_manually():
    pt = started()
    pt.vehicle.clutch_engage_rate = 50.0     # a real dump, not a slow release
    c = pt.controls
    c.auto_clutch = False
    c.clutch_pedal = 1.0
    c.gear = 1
    run(pt, 0.5)
    c.clutch_pedal = 0.0
    out = run(pt, 2.0)
    assert out.engine_rpm < 1.0, out.engine_rpm


def test_bump_start():
    """Rolling in gear with ignition on, dropping the clutch restarts the
    engine without the starter."""
    pt = started()
    c = pt.controls
    c.gear, c.throttle = 2, 0.6
    run(pt, 3.0)
    c.ignition, c.throttle = False, 0.0
    c.auto_clutch, c.clutch_pedal = False, 1.0
    out = run(pt, 3.0)
    assert out.engine_rpm < 1.0 and out.vehicle_speed_ms > 3.0
    c.ignition, c.clutch_pedal = True, 0.0
    run(pt, 1.0)
    c.clutch_pedal = 1.0
    out = run(pt, 2.0)
    assert out.engine.combusting
    # (the resolved core rides a fast-idle ramp for a few seconds after it first fires)
    assert abs(out.engine_rpm - pt.cfg.idle_rpm) < 400


def test_dyno_holds_speed_and_reads_motoring_torque():
    pt = started(preset_chevy_454)
    c = pt.controls
    c.dyno_enabled = True
    c.dyno_rpm = 3000
    c.throttle = 1.0
    out = run(pt, 2.0)
    assert abs(out.engine_rpm - 3000) < 30
    assert out.dyno_torque > 300
    c.throttle = 0.0
    out = run(pt, 1.5)
    assert abs(out.engine_rpm - 3000) < 30
    assert out.dyno_torque < 0, "closed throttle on the dyno = motoring"


if __name__ == "__main__":
    tests = [v for k, v in dict(globals()).items() if k.startswith("test_")]
    failed = 0
    for t in tests:
        try:
            t()
            print(f"  PASS: {t.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"  FAIL: {t.__name__}: {e}")
    print(f"\n{len(tests) - failed}/{len(tests)} passed")
    sys.exit(1 if failed else 0)
