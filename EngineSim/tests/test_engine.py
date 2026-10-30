"""Automated tests for Engine Simulator v0.1.

Tests every independently verifiable equation from the KB.
All tests use assertions — no test framework dependency.

Source: ENGINE_KB.md §26 (Required tests).
"""

from __future__ import annotations

import math
import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from engine_sim.config import EngineConfig, TorquePoint
from engine_sim.state import EngineState
from engine_sim.engine import Engine, EngineInput, EngineOutput
from engine_sim.telemetry import TelemetryRecorder


# ============================================================
# Test utilities
# ============================================================

def assert_close(a, b, tol=1e-6, msg=""):
    """Assert two floats are within tolerance."""
    if abs(a - b) > tol:
        raise AssertionError(
            f"{msg}: expected {b}, got {a}, diff={abs(a-b):.2e}"
        )


def assert_positive(a, msg=""):
    if a <= 0:
        raise AssertionError(f"{msg}: expected positive, got {a}")


def assert_negative(a, msg=""):
    if a >= 0:
        raise AssertionError(f"{msg}: expected negative, got {a}")


def assert_in_range(a, lo, hi, msg=""):
    if a < lo or a > hi:
        raise AssertionError(f"{msg}: {a} not in [{lo}, {hi}]")


# ============================================================
# Test 1: RPM ↔ angular velocity conversion
# Source: ENGINE_KB.md — ω = 2πRPM/60, RPM = ω·60/(2π)
# ============================================================

def test_rpm_conversion():
    """Verify RPM ↔ angular velocity conversion."""
    state = EngineState()

    # 0 RPM → 0 rad/s
    state.omega = 0.0
    assert_close(state.rpm, 0.0, msg="0 RPM → 0 rad/s")

    # 60 RPM → 2π rad/s ≈ 6.283
    state.omega = 2.0 * math.pi
    assert_close(state.rpm, 60.0, msg="2π rad/s → 60 RPM")

    # 1000 RPM → 1000 * 2π/60 ≈ 104.72 rad/s
    expected_omega = 1000.0 * 2.0 * math.pi / 60.0
    state.omega = expected_omega
    assert_close(state.rpm, 1000.0, msg="1000 RPM → 104.72 rad/s")

    # Round-trip: RPM → omega → RPM
    for rpm_val in [500, 1500, 3000, 5500, 7000]:
        omega = rpm_val * 2.0 * math.pi / 60.0
        state.omega = omega
        assert_close(state.rpm, rpm_val, msg=f"Round-trip {rpm_val} RPM")

    print("  PASS: test_rpm_conversion")


# ============================================================
# Test 2: Zero net torque → constant omega
# Source: ENGINE_KB.md Eq 14.52
#
# v0.2 note: the thermodynamic model always fires (indicated torque
# depends on throttle/omega but is never gated to exactly zero), so
# "zero torque" can no longer be achieved by zeroing the torque curve.
# Instead we probe the indicated+loss torque at the operating point
# (dt=0 leaves state unchanged: alpha*0 == 0) and apply exactly that
# as load_torque so the net torque is zero by construction, then
# verify the resulting equilibrium is stable.
# ============================================================

def test_zero_net_torque():
    """If commanded load exactly balances indicated+loss torque, omega is stable."""
    config = EngineConfig(
        engine_inertia_kgm2=0.1,
        load_inertia_kgm2=0.0,
        idle_rpm=0.0,
        redline_rpm=10000.0,
    )
    engine = Engine(config)
    engine.start()
    engine.state.omega = 300.0  # clear of the idle controller's range

    probe = engine.update(EngineInput(throttle=0.5, load_torque=0.0), dt=0.0)
    balancing_load = (probe.indicated_torque - probe.friction_torque
                       - probe.pumping_torque - probe.accessory_torque)

    dt = 0.001
    for _ in range(1000):
        out = engine.update(EngineInput(throttle=0.5, load_torque=balancing_load), dt)

    assert_close(out.net_torque, 0.0, tol=1.0,
                 msg="Balancing load should give ~zero net torque")
    assert_close(engine.state.omega, 300.0, tol=1.0,
                 msg="Zero net torque → constant omega")

    print("  PASS: test_zero_net_torque")


# ============================================================
# Test 3: Positive net torque → positive angular acceleration
# Source: ENGINE_KB.md Eq 14.52
# ============================================================

def test_positive_net_torque():
    """Positive net torque must produce positive angular acceleration."""
    config = EngineConfig(
        engine_inertia_kgm2=0.1,
        idle_rpm=0.0,
        redline_rpm=10000.0,
        torque_curve=[TorquePoint(3000, 200.0)],
        friction_C1_Pa=0.0,
        friction_C2_Pa_per_rev_s=0.0,
        friction_C3_Pa_per_rev_s2=0.0,
    )
    engine = Engine(config)
    engine.start()
    engine.state.omega = 50.0  # Some initial speed

    out = engine.update(EngineInput(throttle=1.0, load_torque=0.0), dt=0.001)
    assert_positive(out.angular_acceleration,
                    msg="Positive net torque → positive alpha")
    assert_positive(out.net_torque, msg="Net torque should be positive")

    print("  PASS: test_positive_net_torque")


# ============================================================
# Test 4: Negative net torque → negative angular acceleration
# Source: ENGINE_KB.md Eq 14.52
# ============================================================

def test_negative_net_torque():
    """Negative net torque must produce negative angular acceleration."""
    config = EngineConfig(
        engine_inertia_kgm2=0.1,
        idle_rpm=0.0,
        redline_rpm=10000.0,
        torque_curve=[TorquePoint(3000, 100.0)],
        friction_C1_Pa=0.0,
        friction_C2_Pa_per_rev_s=0.0,
        friction_C3_Pa_per_rev_s2=0.0,
    )
    engine = Engine(config)
    engine.start()
    engine.state.omega = 50.0  # Some initial speed

    # Apply load that exceeds engine torque. Indicated torque is now correctly
    # scaled by num_cylinders (whole-engine, not single-cylinder), so this
    # needs a larger load than before to guarantee it still exceeds it.
    out = engine.update(EngineInput(throttle=0.5, load_torque=1000.0), dt=0.001)
    assert_negative(out.angular_acceleration,
                    msg="Negative net torque → negative alpha")

    print("  PASS: test_negative_net_torque")


# ============================================================
# Test 5: Inertia effect
# Source: ENGINE_KB.md Eq 14.52: α = T_net / I
# ============================================================

def test_inertia_effect():
    """Larger inertia → smaller angular acceleration for same torque."""
    torque_curve = [TorquePoint(3000, 200.0)]

    config_small = EngineConfig(
        engine_inertia_kgm2=0.05,
        torque_curve=torque_curve,
        friction_C1_Pa=0.0,
        friction_C2_Pa_per_rev_s=0.0,
        friction_C3_Pa_per_rev_s2=0.0,
        redline_rpm=10000.0,
    )
    config_large = EngineConfig(
        engine_inertia_kgm2=0.20,
        torque_curve=torque_curve,
        friction_C1_Pa=0.0,
        friction_C2_Pa_per_rev_s=0.0,
        friction_C3_Pa_per_rev_s2=0.0,
        redline_rpm=10000.0,
    )

    engine_small = Engine(config_small)
    engine_small.start()
    engine_small.state.omega = 50.0

    engine_large = Engine(config_large)
    engine_large.start()
    engine_large.state.omega = 50.0

    inp = EngineInput(throttle=1.0, load_torque=0.0)
    out_small = engine_small.update(inp, dt=0.001)
    out_large = engine_large.update(inp, dt=0.001)

    assert out_small.angular_acceleration > out_large.angular_acceleration, (
        f"Smaller inertia ({config_small.engine_inertia_kgm2}) should give "
        f"larger alpha ({out_small.angular_acceleration}) than larger inertia "
        f"({config_large.engine_inertia_kgm2}, alpha={out_large.angular_acceleration})"
    )

    print("  PASS: test_inertia_effect")


# ============================================================
# Test 6: Power equation
# Source: ENGINE_KB.md Eq 2.13: P = T * omega
#
# v0.2 note: engine.py computes brake power directly as
# `power = T_net * omega_new`, i.e. against net (not gross) torque —
# the throttle-scaled "gross torque" concept from v0.1 no longer
# exists since throttle now acts upstream (on intake pressure), not
# as a multiplier on indicated torque.
# ============================================================

def test_power_equation():
    """Verify power = net torque × angular velocity."""
    config = EngineConfig(redline_rpm=10000.0)
    engine = Engine(config)
    engine.start()
    engine.state.omega = 300.0  # Some speed

    out = engine.update(EngineInput(throttle=0.6, load_torque=20.0), dt=0.001)

    expected_power = out.net_torque * out.angular_velocity
    assert_close(out.power, expected_power, tol=1e-6,
                 msg="P = T_net × omega")

    print("  PASS: test_power_equation")


# ============================================================
# Test 7: Torque interpolation
# Source: ENGINE_KB.md Part XVII §B (torque curve data)
# ============================================================

def test_torque_interpolation():
    """Verify known points and interpolation of torque curve."""
    config = EngineConfig(
        torque_curve=[
            TorquePoint(1000, 100.0),
            TorquePoint(2000, 200.0),
            TorquePoint(3000, 150.0),
        ]
    )

    # Known points
    assert_close(config.torque_at_rpm(1000), 100.0, msg="Point at 1000 RPM")
    assert_close(config.torque_at_rpm(2000), 200.0, msg="Point at 2000 RPM")
    assert_close(config.torque_at_rpm(3000), 150.0, msg="Point at 3000 RPM")

    # Interpolated point (midway between 1000 and 2000)
    t_mid = config.torque_at_rpm(1500)
    assert_close(t_mid, 150.0, msg="Interpolated at 1500 RPM (midpoint)")

    # Interpolated point (1/4 between 2000 and 3000)
    t_q = config.torque_at_rpm(2250)
    assert_close(t_q, 187.5, msg="Interpolated at 2250 RPM (1/4)")

    # Extrapolation below
    t_low = config.torque_at_rpm(500)
    # Should extrapolate from 1000-2000 slope: slope = 100/1000 = 0.1 per RPM
    # At 500: 100 + 0.1*(500-1000) = 50.0
    assert_close(t_low, 50.0, msg="Extrapolated below 1000 RPM")

    # Extrapolation above
    t_high = config.torque_at_rpm(4000)
    # Should extrapolate from 2000-3000 slope: slope = -50/1000 = -0.05
    # At 4000: 150 + (-0.05)*(4000-3000) = 100.0
    assert_close(t_high, 100.0, msg="Extrapolated above 3000 RPM")

    print("  PASS: test_torque_interpolation")


# ============================================================
# Test 8: RPM limiting
# Source: ENGINE_KB.md Part XXI §37, Part XXIII
# ============================================================

def test_rpm_limiting():
    """Verify limiter prevents RPM from exceeding redline."""
    config = EngineConfig(
        redline_rpm=6000.0,
        limiter_hysteresis_rpm=200.0,
        torque_curve=[TorquePoint(5000, 200.0), TorquePoint(7000, 150.0)],
        friction_C1_Pa=0.0,
        friction_C2_Pa_per_rev_s=0.0,
        friction_C3_Pa_per_rev_s2=0.0,
    )
    engine = Engine(config)
    engine.start()
    engine.state.omega = 5000.0 * config.RPM_TO_RADS

    # Run at full throttle until limiter activates
    for _ in range(10000):
        out = engine.update(EngineInput(throttle=1.0, load_torque=0.0), dt=0.001)
        if out.limiter_active:
            break

    assert out.limiter_active, "Limiter should activate"
    # RPM should not significantly exceed redline
    assert out.rpm <= config.redline_rpm + 100.0, \
        f"RPM {out.rpm:.0f} exceeds redline {config.redline_rpm} by too much"

    print("  PASS: test_rpm_limiting")


# ============================================================
# Test 9: Friction torque is positive and increases with speed
# Source: ENGINE_KB.md Eq 13.5
# ============================================================

def test_friction_model():
    """Verify friction torque is positive and increases with speed."""
    config = EngineConfig()

    T_fric_low = config.friction_torque(50.0)   # ~800 RPM
    T_fric_high = config.friction_torque(300.0)  # ~2860 RPM

    assert_positive(T_fric_low, msg="Friction at low speed should be positive")
    assert_positive(T_fric_high, msg="Friction at high speed should be positive")
    assert T_fric_high > T_fric_low, \
        f"Friction at high speed ({T_fric_high}) should exceed low speed ({T_fric_low})"

    print("  PASS: test_friction_model")


# ============================================================
# Test 10: Torque balance consistency
# Source: ENGINE_KB.md §29
#
# v0.2 note: gross torque is no longer throttle × T_WOT; the full
# balance is T_indicated - T_friction - T_pumping - T_accessory - T_load.
# ============================================================

def test_torque_balance_consistency():
    """Verify torque balance: indicated - friction - pumping - accessory - load = net."""
    config = EngineConfig(redline_rpm=10000.0)
    engine = Engine(config)
    engine.start()
    engine.state.omega = 300.0  # clear of the idle controller's range

    out = engine.update(EngineInput(throttle=0.8, load_torque=30.0), dt=0.001)

    expected_net = (out.indicated_torque - out.friction_torque
                    - out.pumping_torque - out.accessory_torque
                    - out.load_torque)

    assert_close(out.net_torque, expected_net, tol=0.01,
                 msg="Torque balance: T_ind - T_fric - T_pump - T_acc - T_load = T_net")

    # Verify alpha = T_net / I
    expected_alpha = expected_net / config.total_inertia
    assert_close(out.angular_acceleration, expected_alpha, tol=0.01,
                 msg="alpha = T_net / I")

    print("  PASS: test_torque_balance_consistency")


# ============================================================
# Test 11: Engine start/reset
# ============================================================

def test_engine_start_reset():
    """Verify engine start and reset behavior."""
    config = EngineConfig(idle_rpm=800.0)
    engine = Engine(config)

    # Initially stopped
    assert_close(engine.state.omega, 0.0, msg="Initially stopped")

    # Start
    engine.start()
    expected_idle_omega = 800.0 * config.RPM_TO_RADS
    assert_close(engine.state.omega, expected_idle_omega,
                 msg="After start, omega = idle")

    # Reset
    engine.reset()
    assert_close(engine.state.omega, 0.0, msg="After reset, omega = 0")

    print("  PASS: test_engine_start_reset")


# ============================================================
# Test 12: Deterministic output (same inputs → same outputs)
# ============================================================

def test_deterministic():
    """Same inputs and state must produce identical outputs."""
    config = EngineConfig(
        torque_curve=[TorquePoint(3000, 200.0)],
        friction_C1_Pa=50000.0,
    )

    engine1 = Engine(EngineConfig(
        torque_curve=list(config.torque_curve),
        friction_C1_Pa=config.friction_C1_Pa,
    ))
    engine1.start()
    engine1.state.omega = 100.0

    engine2 = Engine(EngineConfig(
        torque_curve=list(config.torque_curve),
        friction_C1_Pa=config.friction_C1_Pa,
    ))
    engine2.start()
    engine2.state.omega = 100.0

    inp = EngineInput(throttle=0.7, load_torque=20.0)
    out1 = engine1.update(inp, dt=0.001)
    out2 = engine2.update(inp, dt=0.001)

    for field in ['rpm', 'angular_velocity', 'engine_torque', 'friction_torque',
                  'net_torque', 'angular_acceleration', 'power']:
        v1 = getattr(out1, field)
        v2 = getattr(out2, field)
        assert_close(v1, v2, tol=1e-10, msg=f"Deterministic check on {field}")

    print("  PASS: test_deterministic")


# ============================================================
# Test 13: Telemetry recording
# ============================================================

def test_telemetry():
    """Verify telemetry records correctly."""
    config = EngineConfig(torque_curve=[TorquePoint(3000, 100.0)])
    engine = Engine(config)
    engine.start()

    recorder = TelemetryRecorder()
    recorder.clear()

    for i in range(100):
        out = engine.update(EngineInput(throttle=0.5), dt=0.001)
        recorder.record(i * 0.001, out)

    assert recorder.count == 100, f"Expected 100 entries, got {recorder.count}"

    rpm_trace = recorder.get_rpm_trace()
    assert len(rpm_trace) == 100, f"RPM trace length mismatch: {len(rpm_trace)}"

    print("  PASS: test_telemetry")


# ============================================================
# Test 14: Sanity checks — no NaN or Infinity
# ============================================================

def test_no_nan_or_inf():
    """Engine should not produce NaN or Infinity."""
    config = EngineConfig(
        torque_curve=[TorquePoint(3000, 200.0)],
        redline_rpm=8000.0,
    )
    engine = Engine(config)
    engine.start()

    for _ in range(5000):
        out = engine.update(EngineInput(throttle=1.0, load_torque=0.0), dt=0.001)
        for field in ['rpm', 'angular_velocity', 'engine_torque',
                      'friction_torque', 'net_torque', 'angular_acceleration', 'power']:
            val = getattr(out, field)
            assert not math.isnan(val), f"{field} is NaN at omega={engine.state.omega}"
            assert not math.isinf(val), f"{field} is Inf at omega={engine.state.omega}"

    print("  PASS: test_no_nan_or_inf")


# ============================================================
# Test 15: Engine doesn't reverse
# ============================================================

def test_no_reverse():
    """Engine omega should never go negative."""
    config = EngineConfig(
        torque_curve=[TorquePoint(3000, 100.0)],
        friction_C1_Pa=200000.0,  # High friction
        redline_rpm=10000.0,
    )
    engine = Engine(config)
    engine.start()
    engine.state.omega = 50.0  # Low speed with high friction

    # Apply heavy load to decelerate
    for _ in range(10000):
        out = engine.update(EngineInput(throttle=0.0, load_torque=500.0), dt=0.001)
        assert engine.state.omega >= 0.0, \
            f"Omega went negative: {engine.state.omega}"

    print("  PASS: test_no_reverse")


# ============================================================
# Test 16: Gas properties (temperature-dependent cp/cv)
# ============================================================

def test_gas_properties():
    """Verify temperature-dependent gas properties."""
    from engine_sim.gas_properties import cp_air, cv_air, gamma_air, R_AIR

    # At 300K: cp ≈ 1005, cv ≈ 718, γ ≈ 1.4
    cp_300 = cp_air(300.0)
    cv_300 = cv_air(300.0)
    gamma_300 = gamma_air(300.0)

    assert_close(cp_300, 1005.0, tol=50.0, msg="cp at 300K")
    assert_close(cv_300, 718.0, tol=50.0, msg="cv at 300K")
    assert_close(gamma_300, 1.4, tol=0.05, msg="gamma at 300K")

    # cp should increase with temperature (real gas behavior)
    cp_2000 = cp_air(2000.0)
    assert cp_2000 > cp_300, f"cp should increase with T: {cp_300} at 300K, {cp_2000} at 2000K"

    # R = cp - cv
    assert_close(cp_300 - cv_300, R_AIR, tol=1.0, msg="R = cp - cv")

    print("  PASS: test_gas_properties")


# ============================================================
# Test 17: Wiebe function
# ============================================================

def test_wiebe_function():
    """Verify Wiebe mass-fraction-burned curve."""
    from engine_sim.combustion_model import wiebe_mass_fraction_burned

    # At start: x_b = 0
    x_b_0 = wiebe_mass_fraction_burned(0.0, 0.0, 60.0, a=5.0, m=2.0)
    assert_close(x_b_0, 0.0, tol=0.01, msg="x_b at start")

    # At end: x_b ≈ 1
    x_b_end = wiebe_mass_fraction_burned(60.0, 0.0, 60.0, a=5.0, m=2.0)
    assert_close(x_b_end, 1.0, tol=0.01, msg="x_b at end")

    # Midpoint: x_b should be ~0.5 (S-curve shape)
    x_b_mid = wiebe_mass_fraction_burned(30.0, 0.0, 60.0, a=5.0, m=2.0)
    assert_in_range(x_b_mid, 0.3, 0.7, msg="x_b at midpoint")

    # Monotonically increasing
    prev = 0.0
    for theta in range(0, 61, 5):
        xb = wiebe_mass_fraction_burned(float(theta), 0.0, 60.0)
        assert xb >= prev, f"Wiebe not monotonic at {theta}°"
        prev = xb

    print("  PASS: test_wiebe_function")


# ============================================================
# Test 18: Throttle flow
# ============================================================

def test_throttle_flow():
    """Verify throttle-plate orifice flow equations."""
    from engine_sim.manifold_model import throttle_flow_rate, throttle_effective_area

    # Zero throttle → zero flow
    flow_zero = throttle_flow_rate(101325.0, 101325.0, 293.0, 0.0)
    assert_close(flow_zero, 0.0, msg="Zero area → zero flow")

    # WOT, no pressure drop → zero flow
    flow_no_dp = throttle_flow_rate(101325.0, 101325.0, 293.0, 0.003)
    assert_close(flow_no_dp, 0.0, tol=0.01, msg="No pressure drop → zero flow")

    # Open throttle with pressure drop → positive flow
    flow_open = throttle_flow_rate(101325.0, 50000.0, 293.0, 0.003)
    assert_positive(flow_open, msg="Open throttle → positive flow")

    # Choked flow at large pressure ratio
    flow_choked = throttle_flow_rate(101325.0, 10000.0, 293.0, 0.003)
    assert_positive(flow_choked, msg="Choked flow exists")

    # Effective area
    area = throttle_effective_area(1.0, 0.003)
    assert_close(area, 0.003, tol=0.001, msg="WOT area")

    area_idle = throttle_effective_area(0.05, 0.003)
    assert area_idle < area, "Idle area < WOT area"

    print("  PASS: test_throttle_flow")


# ============================================================
# Test 19: Manifold dynamics
# ============================================================

def test_manifold_dynamics():
    """Verify manifold filling/emptying dynamics."""
    from engine_sim.manifold_model import manifold_pressure_dynamics, manifold_time_constant

    # Steady state: inflow = outflow → pressure unchanged
    p_new = manifold_pressure_dynamics(101325.0, 0.1, 0.1, 0.003, 293.0, 0.001)
    assert_close(p_new, 101325.0, tol=100.0, msg="Steady state pressure")

    # More inflow than outflow → pressure rises
    p_rise = manifold_pressure_dynamics(101325.0, 0.2, 0.1, 0.003, 293.0, 0.001)
    assert p_rise > 101325.0, f"Pressure should rise: {p_rise}"

    # More outflow than inflow → pressure drops
    p_drop = manifold_pressure_dynamics(101325.0, 0.05, 0.1, 0.003, 293.0, 0.001)
    assert p_drop < 101325.0, f"Pressure should drop: {p_drop}"

    # Time constant
    tau = manifold_time_constant(0.003, 0.85, 0.0015, 50.0)
    assert tau > 0, f"Time constant should be positive: {tau}"
    assert tau < 1.0, f"Time constant should be < 1s: {tau}"

    print("  PASS: test_manifold_dynamics")


# ============================================================
# Test 20: Heat transfer
# ============================================================

def test_heat_transfer():
    """Verify Woschni and Annand heat transfer correlations."""
    from engine_sim.heat_transfer import (
        woschni_heat_transfer_coeff, annand_heat_transfer_coeff,
        heat_transfer_rate, chamber_surface_area,
    )

    # Woschni: positive h_c
    h_c_w = woschni_heat_transfer_coeff(
bore=0.0825, pressure=50e5, temperature=2500.0,
            mean_piston_speed=10.0, cylinder_volume=3e-4,
            clearance_volume=3e-5, reference_pressure=1e5,
            reference_temp=300.0, motoring_pressure=0.6e5,
            combustion_occurred=True,
        )
    assert_positive(h_c_w, msg="Woschni h_c positive")

    # Annand: positive h_c
    h_c_a = annand_heat_transfer_coeff(
        bore=0.0825, pressure=50e5, temperature=2500.0,
        mean_piston_speed=10.0,
    )
    assert_positive(h_c_a, msg="Annand h_c positive")

    # Heat transfer rate: positive when T_gas > T_wall
    Q = heat_transfer_rate(h_c_w, 0.01, 2500.0, 400.0)
    assert_positive(Q, msg="Heat loss positive")

    # Negative when T_gas < T_wall (heat into gas)
    Q_neg = heat_transfer_rate(h_c_w, 0.01, 300.0, 400.0)
    assert Q_neg < 0, "Heat gain should be negative"

    # Surface area
    A = chamber_surface_area(0.0825, 0.086, 3e-4)
    assert_positive(A, msg="Surface area positive")

    print("  PASS: test_heat_transfer")


# ============================================================
# Test 21: Spark timing
# ============================================================

def test_spark_timing():
    """Verify MBT spark timing calculation."""
    from engine_sim.spark_model import mbt_timing, combustion_duration

    # Combustion duration
    dur = combustion_duration(3000.0, 1.0, 1.0)
    assert_in_range(dur, 20.0, 120.0, msg="Combustion duration in range")

    # MBT timing
    mbt = mbt_timing(60.0, 3000.0)
    assert_in_range(mbt, 5.0, 50.0, msg="MBT timing in range")

    # Higher speed → more advance
    mbt_low = mbt_timing(60.0, 1500.0)
    mbt_high = mbt_timing(60.0, 5000.0)
    assert mbt_high >= mbt_low, "Higher speed → more advance"

    print("  PASS: test_spark_timing")


# ============================================================
# Test 22: Fuel film model
# ============================================================

def test_fuel_film():
    """Verify fuel-film wall-wetting dynamics."""
    from engine_sim.fuel_film import FuelFilmState, fuel_film_dynamics, fuel_film_tau, fuel_film_x

    state = FuelFilmState(x=0.3, tau=0.5, film_mass=0.0)

    # Steady state: fuel in = fuel out
    m_dot_in = 0.001  # kg/s
    m_cyl, new_state = fuel_film_dynamics(m_dot_in, state, 0.001)
    assert m_cyl > 0, "Fuel flow to cylinders positive"

    # Build up film over time
    state2 = FuelFilmState(x=0.3, tau=0.5, film_mass=0.0)
    for _ in range(1000):
        m_cyl, state2 = fuel_film_dynamics(m_dot_in, state2, 0.001)
    # Film should reach quasi-steady state
    assert state2.film_mass > 0, "Film mass should build up"

    # Time constant
    tau = fuel_film_tau(3000.0, 101325.0, 293.0)
    assert tau > 0, "Tau positive"

    # Film fraction
    x_wot = fuel_film_x(1.0, "throttle_body")
    x_idle = fuel_film_x(0.05, "throttle_body")
    assert x_idle > x_wot, "More film at idle than WOT"

    print("  PASS: test_fuel_film")


# ============================================================
# Test 23: Lambda controller
# ============================================================

def test_lambda_controller():
    """Verify closed-loop lambda controller."""
    from engine_sim.lambda_controller import LambdaController

    ctrl = LambdaController(target_lambda=1.0, integrator_gain=5.0,
                            transport_delay_s=0.05, enabled=True)

    # At stoich: correction should be ~1.0
    corr, V = ctrl.update(1.0, 800.0, 0.001)
    assert_close(corr, 1.0, tol=0.15, msg="Stoich correction")

    # Lean (λ > 1): should add fuel (correction > 1)
    for _ in range(500):
        corr, V = ctrl.update(1.2, 800.0, 0.001)
    assert corr > 1.0, f"Lean → add fuel: {corr}"

    # Rich (λ < 1): should remove fuel (correction < 1)
    ctrl2 = LambdaController(enabled=True)
    for _ in range(100):
        corr, V = ctrl2.update(0.8, 800.0, 0.001)
    assert corr < 1.0, f"Rich → remove fuel: {corr}"

    # Limit cycle frequency
    f = ctrl.limit_cycle_frequency
    assert f > 0, "Limit cycle frequency positive"

    print("  PASS: test_lambda_controller")


# ============================================================
# Test 24: Turbo model
# ============================================================

def test_turbo_model():
    """Verify turbocharger compressor/turbine models."""
    from engine_sim.turbo_model import (
        compressor_power, turbine_power, compressor_outlet_temp,
        turbo_shaft_dynamics, TurboState,
    )

    # Compressor power (positive = consumed)
    W_c = compressor_power(0.05, 300.0, 2.0, 0.72)
    assert W_c > 0, "Compressor power positive"

    # Outlet temp higher than inlet
    T_out = compressor_outlet_temp(300.0, 2.0, 0.72)
    assert T_out > 300.0, "Outlet temp > inlet"

    # Turbine power (positive = produced)
    W_t = turbine_power(0.05, 900.0, 1.5, 0.78)
    assert W_t > 0, "Turbine power positive"

    # Shaft dynamics
    state = TurboState()
    new_speed = turbo_shaft_dynamics(W_t, W_c, 50000.0, dt=0.001)
    assert new_speed >= 0, "Shaft speed non-negative"

    print("  PASS: test_turbo_model")


# ============================================================
# Test 25: Emissions model
# ============================================================

def test_emissions_model():
    """Verify NOx/HC emissions models."""
    from engine_sim.emissions_model import nox_emission_index, hc_emissions_fraction

    # NOx at low T should be negligible
    nox_low = nox_emission_index(1500.0, 2.0, 0.05, 50.0)
    assert nox_low < 1.0, f"Low T → low NOx: {nox_low}"

    # NOx at high T should be significant
    nox_high = nox_emission_index(2500.0, 2.0, 0.05, 50.0)
    assert nox_high > nox_low, "Higher T → more NOx"

    # HC fraction
    hc_stoich = hc_emissions_fraction(1.0, 0.95)
    hc_rich = hc_emissions_fraction(1.3, 0.95)
    assert hc_rich > hc_stoich, "Rich → more HC"

    print("  PASS: test_emissions_model")


# ============================================================
# Test 26: Friction breakdown
# ============================================================

def test_friction_breakdown():
    """Verify component-level friction breakdown."""
    from engine_sim.friction_breakdown import total_friction_breakdown

    result = total_friction_breakdown(
        bore=0.0825, stroke=0.086, mean_piston_speed=10.0,
        shaft_speed_rps=50.0, peak_pressure_bar=50.0,
        imep_c_bar=10.0, speed_rpm=3000.0,
    )

    assert result["piston_mep"] > 0, "Piston friction positive"
    assert result["bearing_mep"] > 0, "Bearing friction positive"
    assert result["valvetrain_mep"] > 0, "Valvetrain friction positive"
    assert result["total_mep"] > 0, "Total friction positive"

    # Piston should be largest component
    assert result["piston_mep"] > result["bearing_mep"], "Piston > bearing"
    assert result["piston_mep"] > result["valvetrain_mep"], "Piston > valvetrain"

    print("  PASS: test_friction_breakdown")


# ============================================================
# Test 27: Diesel combustion model
# ============================================================

def test_diesel_combustion():
    """Verify diesel CI combustion model."""
    from engine_sim.diesel_combustion import (
        ignition_delay, watson_heat_release, diesel_combustion_duration,
    )

    # Ignition delay
    delay = ignition_delay(800.0, 40e5, 50.0)
    assert delay > 0, "Delay positive"
    assert delay < 0.1, f"Delay should be < 100ms: {delay}"

    # Higher temp → shorter delay
    delay_hot = ignition_delay(1000.0, 40e5, 50.0)
    assert delay_hot < delay, "Hotter → shorter delay"

    # Watson heat release: x_b = 0 before start
    x_b = watson_heat_release(0.0, 10.0, 10.0, 50.0)
    assert_close(x_b, 0.0, msg="x_b before start")

    # x_b = 1 after complete
    x_b = watson_heat_release(80.0, 10.0, 10.0, 50.0)
    assert_close(x_b, 1.0, tol=0.01, msg="x_b after complete")

    # Duration
    dur = diesel_combustion_duration(2000.0, 1.0, 2.0)
    assert_in_range(dur, 20.0, 100.0, msg="Diesel combustion duration")

    print("  PASS: test_diesel_combustion")


# ============================================================
# Test 28: Engine with new features enabled
# ============================================================

def test_engine_with_manifold_dynamics():
    """Verify engine works with manifold dynamics enabled."""
    config = EngineConfig(
        use_manifold_dynamics=True,
        manifold_volume_m3=0.003,
        redline_rpm=10000.0,
    )
    engine = Engine(config)
    engine.start()
    engine.state.omega = 200.0

    # Run for a bit
    for _ in range(100):
        out = engine.update(EngineInput(throttle=0.5, load_torque=10.0), dt=0.001)

    assert out.rpm > 0, "Engine should be running"
    assert not math.isnan(out.rpm), "RPM should not be NaN"
    assert not math.isinf(out.rpm), "RPM should not be Inf"

    print("  PASS: test_engine_with_manifold_dynamics")


def test_engine_with_heat_transfer():
    """Verify engine works with heat transfer enabled."""
    config = EngineConfig(
        use_heat_transfer=True,
        heat_transfer_model="woschni",
        redline_rpm=10000.0,
    )
    engine = Engine(config)
    engine.start()
    engine.state.omega = 200.0

    for _ in range(100):
        out = engine.update(EngineInput(throttle=0.5, load_torque=10.0), dt=0.001)

    assert out.rpm > 0, "Engine should be running"
    assert out.heat_transfer_loss_w >= 0, "Heat loss should be non-negative"

    print("  PASS: test_engine_with_heat_transfer")


def test_engine_with_turbo():
    """Verify engine works with turbo enabled."""
    config = EngineConfig(
        use_turbo=True,
        turbo_compressor_efficiency=0.72,
        turbo_turbine_efficiency=0.78,
        redline_rpm=10000.0,
    )
    engine = Engine(config)
    engine.start()
    engine.state.omega = 200.0

    for _ in range(200):
        out = engine.update(EngineInput(throttle=0.8, load_torque=10.0), dt=0.001)

    assert out.rpm > 0, "Engine should be running"
    assert out.boost_pressure >= config.ambient_pressure_pa, "Boost >= ambient"

    print("  PASS: test_engine_with_turbo")


# ============================================================
# Run all tests
# ============================================================

def run_all_tests():
    """Run all engine simulator tests."""
    tests = [
        # Original tests
        test_rpm_conversion,
        test_zero_net_torque,
        test_positive_net_torque,
        test_negative_net_torque,
        test_inertia_effect,
        test_power_equation,
        test_torque_interpolation,
        test_rpm_limiting,
        test_friction_model,
        test_torque_balance_consistency,
        test_engine_start_reset,
        test_deterministic,
        test_telemetry,
        test_no_nan_or_inf,
        test_no_reverse,
        # New v0.3 tests
        test_gas_properties,
        test_wiebe_function,
        test_throttle_flow,
        test_manifold_dynamics,
        test_heat_transfer,
        test_spark_timing,
        test_fuel_film,
        test_lambda_controller,
        test_turbo_model,
        test_emissions_model,
        test_friction_breakdown,
        test_diesel_combustion,
        test_engine_with_manifold_dynamics,
        test_engine_with_heat_transfer,
        test_engine_with_turbo,
    ]

    print(f"Running {len(tests)} tests...")
    passed = 0
    failed = 0
    for test_fn in tests:
        try:
            test_fn()
            passed += 1
        except Exception as e:
            print(f"  FAIL: {test_fn.__name__}: {e}")
            failed += 1

    print(f"\nResults: {passed} passed, {failed} failed out of {len(tests)}")
    return failed == 0


if __name__ == "__main__":
    success = run_all_tests()
    sys.exit(0 if success else 1)
