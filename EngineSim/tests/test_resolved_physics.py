"""Tests for the crank-resolved cylinder model and its supporting physics.

Covers the four new modules plus the sign convention fix in
piston_kinematics: valve lift profiles, valve flow areas and orifice mass flow,
exhaust backpressure, and the per-cylinder first-law integration that produces
torque.
"""

from __future__ import annotations

import math

import pytest

from engine_sim.combustion_model import wiebe_mass_fraction_burned
from engine_sim.config import EngineConfig
from engine_sim.cylinder import Cylinder
from engine_sim.exhaust_system import ExhaustSystem
from engine_sim.piston_kinematics import (
    cylinder_volume,
    cylinder_volume_geometry,
    cylinder_volume_rate,
    piston_acceleration,
    piston_displacement,
    piston_velocity,
)
from engine_sim.valve_flow import (
    LIFT_DEADBAND_M,
    valve_flow_area,
    valve_mass_flow,
    valve_mass_flow_signed,
)
from engine_sim.valve_profile import ValveProfile, lift_table, valve_profiles_from_config


# ── Piston kinematics ────────────────────────────────────────────────────

def test_displacement_is_zero_at_tdc_and_stroke_at_bdc():
    """x(0) = 0 and x(pi) = 2r = S, per the documented convention."""
    r, l = 0.043, 0.144
    assert piston_displacement(0.0, r, l) == pytest.approx(0.0, abs=1e-12)
    assert piston_displacement(math.pi, r, l) == pytest.approx(2 * r, rel=1e-12)


def test_displacement_increases_monotonically_from_tdc_to_bdc():
    """Displacement from TDC must grow as the piston descends, 0 → 180°."""
    r, l = 0.043, 0.144
    values = [piston_displacement(math.radians(d), r, l) for d in range(0, 181, 5)]
    assert all(b > a for a, b in zip(values, values[1:]))


def test_velocity_is_positive_while_descending():
    """Piston moves away from TDC on the intake stroke: velocity positive."""
    r, l, omega = 0.043, 0.144, 100.0
    assert piston_velocity(math.radians(45), r, l, omega) > 0.0
    assert piston_velocity(math.radians(315), r, l, omega) < 0.0


def test_acceleration_sign_flips_at_bdc():
    """Piston speeds up leaving TDC and slows down arriving at BDC."""
    r, l, omega = 0.043, 0.144, 100.0
    assert piston_acceleration(math.radians(5), r, l, omega) > 0.0
    assert piston_acceleration(math.radians(175), r, l, omega) < 0.0


def test_geometry_volume_matches_normalized_form():
    """V(θ) from raw geometry must agree with the normalized rc/Rod-ratio form."""
    cfg = EngineConfig()
    args = (cfg.bore_m, cfg.stroke_m, cfg.rod_length_m, cfg.clearance_volume)
    rc, rod = cfg.compression_ratio, cfg.rod_ratio
    for d in range(0, 361, 15):
        geo = cylinder_volume_geometry(math.radians(d), *args)
        norm = cylinder_volume(math.radians(d), cfg.clearance_volume, rc, rod)
        # Both are the exact crank-slider relation (KB Eq 2.5-2.6), so they
        # must agree to rounding. (The normalized form once had 1/R where R
        # belongs and was ~12% off mid-stroke; a loose tolerance hid that.)
        assert geo == pytest.approx(norm, rel=1e-9)


def test_geometry_volume_endpoints_give_compression_ratio():
    cfg = EngineConfig()
    args = (cfg.bore_m, cfg.stroke_m, cfg.rod_length_m, cfg.clearance_volume)
    assert cylinder_volume_geometry(0.0, *args) == pytest.approx(cfg.clearance_volume)
    assert (cylinder_volume_geometry(math.pi, *args) / cfg.clearance_volume
            == pytest.approx(cfg.compression_ratio, rel=1e-9))


def test_volume_rate_sign_matches_stroke_direction():
    """dV/dθ > 0 descending (intake), < 0 on compression."""
    cfg = EngineConfig()
    b, s, l = cfg.bore_m, cfg.stroke_m, cfg.rod_length_m
    assert cylinder_volume_rate(math.radians(90), b, s, l) > 0.0
    assert cylinder_volume_rate(math.radians(270), b, s, l) < 0.0
    assert cylinder_volume_rate(0.0, b, s, l) == pytest.approx(0.0, abs=1e-12)
    assert cylinder_volume_rate(math.pi, b, s, l) == pytest.approx(0.0, abs=1e-12)


def test_volume_rate_matches_numeric_derivative():
    """The analytic dV/dθ agrees with differentiating V(θ) numerically."""
    cfg = EngineConfig()
    b, s, l, vc = cfg.bore_m, cfg.stroke_m, cfg.rod_length_m, cfg.clearance_volume
    h = 1e-4
    for d in range(10, 360, 17):
        th = math.radians(d)
        numeric = (cylinder_volume_geometry(th + h, b, s, l, vc)
                   - cylinder_volume_geometry(th - h, b, s, l, vc)) / (2 * h)
        assert cylinder_volume_rate(th, b, s, l) == pytest.approx(numeric, rel=1e-4)


def test_volume_is_symmetric_about_bdc():
    cfg = EngineConfig()
    args = (cfg.bore_m, cfg.stroke_m, cfg.rod_length_m, cfg.clearance_volume)
    for d in range(1, 360, 7):
        assert (cylinder_volume_geometry(math.radians(d), *args)
                == pytest.approx(cylinder_volume_geometry(math.radians(360 - d), *args),
                                 abs=1e-15))


# ── Valve lift profiles ───────────────────────────────────────────────────

def test_lift_is_zero_when_seated_and_maximal_at_peak():
    prof = ValveProfile(open_deg=710.0, duration_deg=240.0, max_lift_m=0.0095)
    assert prof.lift(prof.peak_deg) == pytest.approx(0.0095, rel=1e-9)
    assert prof.lift(400.0) == 0.0
    assert prof.lift(710.0) == pytest.approx(0.0, abs=1e-12)
    assert prof.lift(prof.close_deg) == pytest.approx(0.0, abs=1e-12)


def test_default_ramps_reproduce_sine_squared_hump():
    """ramp 0.5/0.5 must be the sin² hump the dashboard has always drawn."""
    prof = ValveProfile(open_deg=710.0, duration_deg=240.0, max_lift_m=0.01)
    for frac in (0.1, 0.25, 0.5, 0.75, 0.9):
        angle = prof.open_deg + frac * prof.duration_deg
        assert prof.lift(angle) == pytest.approx(
            0.01 * math.sin(math.pi * frac) ** 2, rel=1e-9)


def test_asymmetric_ramps_shift_the_peak_and_break_symmetry():
    prof = ValveProfile(open_deg=710.0, duration_deg=240.0, max_lift_m=0.01,
                        ramp_open_frac=0.25, ramp_close_frac=0.75)
    # Peak lands at the end of the opening ramp, a quarter into the event
    # (reported wrapped into the 0..720 frame, so 770° reads as 50°).
    assert prof.peak_deg == pytest.approx(50.0)
    assert prof.lift(prof.peak_deg) == pytest.approx(0.01, rel=1e-9)
    # Opening is fast, so by 25% of the event lift is already at maximum...
    assert prof.lift(prof.open_deg + 0.25 * 240.0) == pytest.approx(0.01, rel=1e-9)
    # ...and the long closing ramp means lift is still well above zero at 62.5%,
    # where the symmetric profile has already fallen back past it.
    symmetric = ValveProfile(710.0, 240.0, 0.01)
    assert prof.lift(prof.open_deg + 0.625 * 240.0) < symmetric.lift(
        prof.open_deg + 0.625 * 240.0)
    assert prof.lift(prof.open_deg + 0.5 * 240.0) == pytest.approx(0.0075, rel=1e-9)


def test_profile_window_wraps_across_zero_degrees():
    """An intake event opening at 710° must still be lifting at 0° and 20°."""
    prof = ValveProfile(open_deg=710.0, duration_deg=240.0, max_lift_m=0.01)
    assert prof.is_open(0.0)
    assert prof.is_open(20.0)
    assert prof.lift(0.0) > 0.0
    assert not prof.is_open(400.0)


def test_degenerate_profile_is_shut():
    assert ValveProfile(0.0, 0.0).lift(0.0) == 0.0
    assert ValveProfile(0.0, 200.0, max_lift_m=0.0).lift(50.0) == 0.0


def test_lift_table_spans_full_cycle():
    table = lift_table(ValveProfile(710.0, 240.0, 0.01), step_deg=5.0)
    assert len(table) == 145
    assert max(table) == pytest.approx(0.01, rel=1e-9)


def test_profiles_from_config_use_file_timing_and_lifts():
    cfg = EngineConfig()
    intake, exhaust = valve_profiles_from_config(cfg)
    io, idur = cfg.intake_lift_window
    assert (intake.open_deg, intake.duration_deg) == pytest.approx((io, idur))
    assert intake.max_lift_m == cfg.intake_max_lift_m
    assert exhaust.max_lift_m == cfg.exhaust_max_lift_m
    # Road-engine timing: intake starts before TDC, exhaust ends after it.
    assert intake.is_open(0.0)
    assert exhaust.is_open(0.0)


# ── Valve flow area and orifice mass flow ─────────────────────────────────

def test_flow_area_is_zero_below_deadband():
    assert valve_flow_area(LIFT_DEADBAND_M, 0.032, 0.032) == 0.0
    assert valve_flow_area(0.0, 0.032, 0.032) == 0.0


def test_flow_area_grows_with_lift_then_saturates_at_port():
    """Area is the annular gap until the port throat takes over."""
    head, port = 0.032, 0.032
    small = valve_flow_area(0.001, head, port)
    medium = valve_flow_area(0.004, head, port)
    port_area = math.pi / 4.0 * port ** 2
    assert 0.0 < small < medium < port_area
    assert valve_flow_area(0.02, head, port) == pytest.approx(port_area)


def test_flow_area_gap_uses_seat_angle():
    """Gap area scales with cos(β) (KB Eq 6.7): a shallower seat opens wider.

    β is measured from the valve-head plane, so β = 0 is a flat seat that
    opens the full lift and β = 90° would be a cylinder that opens nothing.
    """
    shallow = valve_flow_area(0.002, 0.032, 0.032, seat_angle_deg=20.0)
    steep = valve_flow_area(0.002, 0.032, 0.032, seat_angle_deg=60.0)
    assert shallow > steep
    assert shallow / steep == pytest.approx(
        math.cos(math.radians(20.0)) / math.cos(math.radians(60.0)), rel=1e-9)
    # β = 45° (the default) is where sin and cos agree, so the usual valve is unchanged.
    assert valve_flow_area(0.002, 0.032, 0.032) == pytest.approx(
        math.pi * 0.032 * 0.002 * math.sqrt(0.5), rel=1e-9)


def test_flow_area_handles_zero_port():
    assert valve_flow_area(0.005, 0.032, 0.0) == 0.0


def test_choked_flow_ignores_downstream_pressure():
    """Well below critical ratio the flow must not respond to p_down."""
    area = 8.0e-4
    deep = valve_mass_flow(4.0e5, 1.0e4, 700.0, area)
    shallow = valve_mass_flow(4.0e5, 1.9e5, 700.0, area)
    assert deep == pytest.approx(shallow, rel=1e-9)


def test_subsonic_flow_falls_as_downstream_pressure_rises():
    area = 8.0e-4
    strong = valve_mass_flow(4.0e5, 3.6e5, 700.0, area)
    weak = valve_mass_flow(4.0e5, 3.9e5, 700.0, area)
    assert 0.0 < weak < strong


def test_flow_is_zero_without_a_pressure_drop():
    assert valve_mass_flow(3.0e5, 3.0e5, 700.0, 8.0e-4) == 0.0
    assert valve_mass_flow(3.0e5, 4.0e5, 700.0, 8.0e-4) == 0.0
    assert valve_mass_flow(3.0e5, 1.0e5, 700.0, 0.0) == 0.0


def test_signed_flow_reverses_with_the_pressure_difference():
    """The same open valve fills or empties depending on which side is higher."""
    area = 8.0e-4
    filling = valve_mass_flow_signed(9.0e4, 1.0e5, 300.0, 300.0, area)
    emptying = valve_mass_flow_signed(3.0e5, 1.0e5, 800.0, 700.0, area)
    assert filling > 0.0
    assert emptying < 0.0
    # Reversion through the intake valve: cylinder still above manifold.
    reversion = valve_mass_flow_signed(1.5e5, 9.0e4, 400.0, 300.0, area)
    assert reversion < 0.0


def test_signed_flow_is_zero_when_seated():
    assert valve_mass_flow_signed(9.0e4, 1.0e5, 300.0, 300.0, 0.0) == 0.0


# ── Exhaust backpressure ─────────────────────────────────────────────────

def test_exhaust_plenum_starts_at_ambient_and_rises_when_fed():
    ex = ExhaustSystem(15e-3, 45e-3, ambient_pressure_pa=101325.0)
    assert ex.pressure == pytest.approx(101325.0, rel=1e-9)
    for _ in range(20):
        ex.update(0.05, 900.0, 1e-4)
    assert ex.pressure > 101325.0
    assert ex.temperature > 293.0


def test_exhaust_plenum_drains_towards_ambient():
    ex = ExhaustSystem(15e-3, 45e-3, ambient_pressure_pa=101325.0)
    for _ in range(20):
        ex.update(0.05, 900.0, 1e-4)
    raised = ex.pressure
    for _ in range(200):
        ex.update(0.0, 700.0, 1e-3)
    assert ex.pressure < raised
    assert ex.pressure == pytest.approx(101325.0, rel=0.05)


def test_narrower_tail_pipe_raises_backpressure():
    """Restriction must show up as higher plenum pressure for the same flow."""
    pressures = []
    for dia in (30e-3, 60e-3):
        ex = ExhaustSystem(15e-3, dia, ambient_pressure_pa=101325.0)
        for _ in range(50):
            ex.update(0.05, 900.0, 1e-4)
        pressures.append(ex.pressure)
    assert pressures[0] > pressures[1]


def test_exhaust_mass_stays_positive_and_pressure_finite():
    ex = ExhaustSystem(15e-3, 45e-3)
    for _ in range(500):
        ex.update(0.4, 1200.0, 1e-3)
    assert ex.mass > 0.0
    assert math.isfinite(ex.pressure) and ex.pressure > 0.0


# ── Cylinder integration ──────────────────────────────────────────────────

def _run_cycle(cfg: EngineConfig, fuel_frac: float = 1.0, load: float = 1.0,
               rpm: float = 3000.0, dtheta_deg: float = 1.0,
               p_intake: float = 91192.0, p_exhaust: float = 110000.0,
               spark_btdc: float | None = None, settle_cycles: int = 3):
    """Drive one cylinder to periodic steady state, then measure one full cycle.

    Seeding a single charge and stepping from TDC is not a physical starting
    point: the clearance volume is ~1/10th of the swept volume, so the seed
    lands at ~70 bar and the measured cycle is dominated by that artificial
    blowdown rather than by the charge the valves actually establish. Instead,
    seed a small residual at BDC, spin enough cycles for the trapped mass and
    temperature to settle, and measure only the last one.
    """
    cyl = Cylinder(1, cfg)
    # Seed a burned-gas residual at BDC: a fraction of the free-air charge
    # would give ~70 bar at this clearance volume.
    cyl.initialize_charge(2.0e-5, 600.0, angle_deg=180.0)
    spark = cfg.spark_timing_deg_btdc if spark_btdc is None else spark_btdc
    spark_angle = 360.0 - spark
    dt = (dtheta_deg / 360.0) * (60.0 / rpm)
    steps = int(720.0 / dtheta_deg)
    peak_p = peak_t = 0.0
    # Air actually trapped at BDC is only known after the intake stroke of the
    # previous cycle, so fuel is metered from that; scaling it from the
    # exhaust-stroke residual leaves the charge far too lean to make torque.
    trapped_air = cfg.displacement_per_cyl * cfg.max_volumetric_efficiency
    for cycle in range(settle_cycles + 1):
        cyl.begin_cycle(trapped_air / cfg.stoich_afr * fuel_frac * load,
                        spark_angle, cfg.combustion_duration_deg)
        cyl.reset_accumulators()
        peak_p = peak_t = 0.0
        for i in range(steps):
            angle = (i * dtheta_deg) % 720.0
            cyl.step(angle + dtheta_deg / 2.0, dtheta_deg, dt, rpm,
                     p_intake, cfg.intake_temperature_k, p_exhaust, 700.0,
                     wall_temperature_k=cfg.wall_temperature_k)
            peak_p = max(peak_p, cyl.pressure)
            peak_t = max(peak_t, cyl.temperature)
            if abs(angle - 180.0) < dtheta_deg:
                trapped_air = cyl.mass
        cyl.finalize_update()
    return cyl, peak_p, peak_t


def test_cylinder_charges_during_the_intake_stroke():
    """Mass must rise while the intake valve is open."""
    cfg = EngineConfig()
    cyl = Cylinder(1, cfg)
    cyl.initialize_charge(3e-4, 293.0)
    dt = (1.0 / 360.0) * (60.0 / 3000.0)
    m_start = cyl.mass
    for _ in range(180):
        cyl.step(90.5, 1.0, dt, 3000.0, 101325.0, 293.0, 105000.0, 700.0)
    assert cyl.mass > m_start * 1.5


def test_cylinder_scavenges_during_the_exhaust_stroke():
    cfg = EngineConfig()
    cyl = Cylinder(1, cfg)
    cyl.initialize_charge(5e-4, 900.0)
    dt = (1.0 / 360.0) * (60.0 / 3000.0)
    m_start = cyl.mass
    for _ in range(180):
        cyl.step(630.5, 1.0, dt, 3000.0, 101325.0, 293.0, 105000.0, 700.0)
    assert cyl.mass < m_start * 0.5


def test_combustion_raises_pressure_and_temperature():
    cfg = EngineConfig()
    _, peak_p, peak_t = _run_cycle(cfg)
    assert peak_p > 30.0e5          # > 30 bar
    assert peak_p < 150.0e5         # sane, not runaway
    assert peak_t > 1500.0          # > 1500 K


def test_torque_is_positive_and_physically_sized_at_wot():
    """One cylinder at WOT should make a few tens of N·m."""
    cfg = EngineConfig()
    cyl, _, _ = _run_cycle(cfg, fuel_frac=1.0, load=1.0)
    assert 15.0 < cyl.indicated_torque < 120.0


def test_thermal_efficiency_is_engineered_range():
    """First-law efficiency over the cycle must land in a plausible band."""
    cfg = EngineConfig()
    cyl, _, _ = _run_cycle(cfg)
    q_fuel = cyl.fuel_mass * cfg.lower_heating_value
    eta = cyl.work_net / q_fuel
    assert 0.20 < eta < 0.45


def test_less_fuel_gives_less_torque():
    """The model must be fuel-limited, not producing torque for free."""
    cfg = EngineConfig()
    rich, _, _ = _run_cycle(cfg, fuel_frac=1.0, load=1.0)
    lean, _, _ = _run_cycle(cfg, fuel_frac=0.5, load=1.0)
    assert lean.indicated_torque < rich.indicated_torque


def test_throttle_reduces_torque_and_adds_pumping_loss():
    """Closing the throttle must cost net torque and raise pumping loss."""
    cfg = EngineConfig()
    wide, _, _ = _run_cycle(cfg, p_intake=91192.0)
    closed, _, _ = _run_cycle(cfg, p_intake=25000.0)
    assert closed.indicated_torque < wide.indicated_torque
    assert closed.pumping_torque >= wide.pumping_torque


def test_pumping_torque_is_reported_as_a_positive_loss():
    cfg = EngineConfig()
    cyl, _, _ = _run_cycle(cfg)
    assert cyl.pumping_torque >= 0.0
    assert cyl.gross_torque == pytest.approx(
        cyl.indicated_torque + cyl.pumping_torque, rel=1e-9)


def test_high_backpressure_costs_torque():
    """Exhaust backpressure is a pumping loss, so it must show up as less torque."""
    cfg = EngineConfig()
    free_flow, _, _ = _run_cycle(cfg, p_exhaust=102000.0)
    restricted, _, _ = _run_cycle(cfg, p_exhaust=180000.0)
    assert restricted.indicated_torque < free_flow.indicated_torque


def test_wall_heat_loss_is_bounded_and_ordered():
    """Wall losses must be real, and must not consume the cycle's work.

    The tight check on how efficient the cycle is belongs in
    test_thermal_efficiency_is_engineered_range. Here what matters is that
    wall losses exist, stay a minority of the fuel energy, and that fuel
    energy going to walls plus work still leaves a positive remainder for the
    exhaust enthalpy and blowdown work that the first law must also account
    for.
    """
    cfg = EngineConfig()
    cyl, _, _ = _run_cycle(cfg)
    q_fuel = cyl.fuel_mass * cfg.lower_heating_value
    assert cyl.heat_loss_total > 0.0, "no wall heat loss at all is not physical"
    assert cyl.heat_loss_total < 0.5 * q_fuel, "walls are eating the cycle"
    remainder = cyl.heat_released_total - cyl.heat_loss_total - cyl.work_net
    assert remainder > 0.0, (
        "fuel energy must exceed wall losses plus work, leaving the exhaust "
        "enthalpy and blowdown share")


def test_firing_flag_fires_exactly_once_per_cycle():
    cfg = EngineConfig()
    cyl = Cylinder(1, cfg)
    cyl.initialize_charge(4e-4, 293.0)
    cyl.begin_cycle(3e-5, 360.0 - cfg.spark_timing_deg_btdc, 60.0)
    dt = (1.0 / 360.0) * (60.0 / 3000.0)
    fire_angle = None
    angle = 0.0
    for _ in range(720):
        cyl.fired_this_cycle = False
        cyl.step(angle + 0.5, 1.0, dt, 3000.0, 91192.0, 293.0, 110000.0, 700.0)
        if cyl.fired_this_cycle and fire_angle is None:
            fire_angle = angle
        angle = (angle + 1.0) % 720.0
    assert fire_angle is not None
    # Combustion starts after the spark and before the burn window closes.
    spark = 360.0 - cfg.spark_timing_deg_btdc
    assert spark <= fire_angle <= spark + cfg.combustion_duration_deg


def test_torque_peaks_at_mbt_and_falls_off_both_ways():
    """Torque must trace an MBT curve, not rise monotonically with spark advance.

    Comparing two fixed spark angles cannot show this: any pair that straddles
    maximum combustion pressure trades a knock-limited gain against a
    heat-loss loss. Sweeping the range does.
    """
    cfg = EngineConfig()
    timings = (5.0, 15.0, 25.0, 40.0)
    torques = [_run_cycle(cfg, spark_btdc=s)[0].indicated_torque for s in timings]
    best = max(range(len(torques)), key=lambda i: torques[i])
    assert best not in (0, len(torques) - 1), (
        f"MBT must be interior to the sweep, got peak at {timings[best]}: {torques}")
    # Torque has to fall off on both sides of the peak.
    assert torques[best] > torques[0]
    assert torques[best] > torques[-1]


def test_full_fuel_energy_is_released_over_the_cycle():
    """Wiebe must integrate to the whole charge, not a fraction of it."""
    cfg = EngineConfig()
    cyl, _, _ = _run_cycle(cfg)
    expected = cyl.fuel_mass * cfg.lower_heating_value * cfg.combustion_efficiency
    assert cyl.heat_released_total == pytest.approx(expected, rel=1e-6)


def test_results_are_insensitive_to_substep_resolution():
    """The explicit first law must be converged at the default substep size."""
    cfg = EngineConfig()
    fine, _, _ = _run_cycle(cfg, dtheta_deg=0.5)
    coarse, _, _ = _run_cycle(cfg, dtheta_deg=2.0)
    assert coarse.indicated_torque == pytest.approx(fine.indicated_torque, rel=0.05)


def test_state_stays_finite_and_positive_across_cycles():
    """No drift, no NaN, no negative mass when the cycle repeats."""
    cfg = EngineConfig()
    cyl = Cylinder(1, cfg)
    cyl.initialize_charge(4e-4, 293.0)
    dt = (2.0 / 360.0) * (60.0 / 4000.0)
    for _ in range(5):
        cyl.begin_cycle(3e-5, 360.0 - cfg.spark_timing_deg_btdc, 60.0)
        cyl.reset_accumulators()
        angle = 0.0
        for _ in range(360):
            cyl.step(angle + 1.0, 2.0, dt, 4000.0, 91192.0, 293.0, 110000.0, 700.0)
            angle = (angle + 2.0) % 720.0
        assert cyl.mass > 0.0
        assert math.isfinite(cyl.pressure) and cyl.pressure > 0.0
        assert math.isfinite(cyl.temperature) and cyl.temperature > 50.0


def test_torque_sign_follows_the_stroke():
    """Torque is positive where the piston descends and negative where it rises."""
    cfg = EngineConfig()
    cyl, _, _ = _run_cycle(cfg)
    # dV/dθ is positive descending (intake/expansion) and negative rising
    # (compression/exhaust), so with pressure positive the torque follows it.
    assert cyl.volume_rate(90.0) > 0.0
    assert cyl.volume_rate(270.0) < 0.0
    # The cycle is integrated clockwise from 0°, ending just before intake TDC,
    # where the piston is driving back up and must resist the crank.
    assert cyl.angle_swept > 0.0
    assert cyl.torque < 0.0


def test_zero_time_step_is_a_no_op():
    cfg = EngineConfig()
    cyl = Cylinder(1, cfg)
    cyl.initialize_charge(4e-4, 293.0)
    m, t = cyl.mass, cyl.temperature
    cyl.step(90.5, 1.0, 0.0, 3000.0, 101325.0, 293.0, 105000.0, 700.0)
    assert cyl.mass == m and cyl.temperature == t


def test_wiebe_curve_is_monotonic_and_bounded():
    prev = -1.0
    for d in range(0, 121, 5):
        x = wiebe_mass_fraction_burned(d, 0.0, 60.0)
        assert 0.0 <= x <= 1.0
        assert x >= prev
        prev = x
    assert wiebe_mass_fraction_burned(60.0, 0.0, 60.0) == pytest.approx(1.0)
    assert wiebe_mass_fraction_burned(-1.0, 0.0, 60.0) == 0.0