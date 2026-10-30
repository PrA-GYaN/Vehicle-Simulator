#!/usr/bin/env python3
"""Turn an EngineSim engine file into this simulator's `detailed:` engine block.

    python tools/engine_toml_to_yaml.py 1nz                      # an EngineSim engine id
    python tools/engine_toml_to_yaml.py path/to/engine.toml -o engine.yaml

EngineSim defines an engine in one TOML file (../EngineSim/config/engines/);
this simulator defines it inside a vehicle's YAML. The physics is the same
code on both sides, so this is a change of spelling, not a conversion: every
resolved-core setting is written under `resolved:` with EngineSim's own key
name and value.

The output has three top-level blocks:

    detailed:   paste under `engine:` in a vehicle file. It also loads as it
                is - `engine_dyno engine.yaml` runs it on the dyno.
    gearbox:    the engine file's [vehicle] gearing, as a starting point.
    sound:      firing angles and banks, and `engine_sim:` - the engine file's
                [exhaust], [intake] and [sound] sections - for the vehicle's
                `sound:` block.

What this simulator has no use for (EngineSim's vehicle mass and drag) is left
out, and settings the port does not implement are reported on stderr. Standard library
only (Python 3.11+).
"""

from __future__ import annotations

import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_ENGINESIM = os.path.normpath(os.path.join(HERE, "..", "..", "EngineSim"))

# resolved: key  ->  EngineConfig attribute (same name unless noted)
RESOLVED_KEYS = [
    "num_valves_per_cyl", "firing_order", "firing_intervals_deg",
    "intake_open_btdc_deg", "intake_close_abdc_deg", "exhaust_open_bbdc_deg", "exhaust_close_atdc_deg",
    "intake_max_lift_m", "intake_ramp_open_frac", "intake_head_dia_m", "intake_port_dia_m",
    "intake_seat_angle_deg", "intake_discharge_coeff",
    "exhaust_max_lift_m", "exhaust_ramp_open_frac", "exhaust_head_dia_m", "exhaust_port_dia_m",
    "exhaust_seat_angle_deg", "exhaust_discharge_coeff",
    "vvt_intake_min_deg", "vvt_intake_max_deg", "vvt_exhaust_min_deg", "vvt_exhaust_max_deg",
    "vvt_rate_deg_per_s", "vvt_auto",
    "manifold_volume_m3", "throttle_max_area_m2", "throttle_discharge_coeff",
    "exhaust_plenum_volume_m3", "exhaust_tail_dia_m", "exhaust_tail_discharge_coeff",
    "exhaust_post_volume_m3",
    "min_firing_rpm", "crank_revs_to_fire", "stall_rpm",
    "dfco_enter_ratio", "dfco_resume_ratio", "dfco_after_start_delay_s",
    "starter_stall_torque_nm", "starter_free_speed_rpm",
    "iac_feedforward", "iac_min", "iac_max", "iac_kp", "iac_ki",
    "iac_resolved_gain_scale", "iac_resolved_kp_fraction",
    "fast_idle_extra_ratio", "fast_idle_decay_s",
    "idle_spark_gain_deg_per_rpm", "idle_spark_reserve_deg",
    "idle_spark_max_advance_deg", "idle_spark_max_retard_deg",
    "spark_timing_deg_btdc", "use_mbt_timing", "wiebe_a", "wiebe_m", "combustion_duration_deg",
    "use_two_zone", "burn_residual_lo", "burn_residual_hi", "burn_lean_lo", "burn_lean_hi",
    "burn_rich_lo", "burn_rich_hi", "burn_misfire_quality", "wall_temperature_k",
    "use_knock_model", "fuel_octane_ron", "knock_retard_step_deg", "knock_retard_max_deg",
    "knock_retard_recovery_deg_per_s",
    "max_substeps_per_update",
]
# resolved: turbo: key  ->  EngineConfig.turbo_<key>
TURBO_KEYS = [
    "count", "target_boost_pa", "wastegate_gain", "inertia_kgm2", "design_speed_rps",
    "design_flow_kgs", "design_pressure_ratio", "compressor_efficiency", "turbine_efficiency",
    "mechanical_efficiency", "turbine_area_m2", "wastegate_area_m2", "boost_volume_m3",
    "intercooler_effectiveness",
]


def _fmt(value) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (list, tuple)):
        return "[" + ", ".join(_fmt(v) for v in value) + "]"
    if isinstance(value, float):
        return repr(value)          # round-trips exactly
    return str(value)


def unsupported(cfg) -> list:
    """Settings of this engine that the vehicle simulator's port does not implement."""
    notes = []
    if not cfg.use_resolved_cylinders:
        notes.append("use_resolved_cylinders is false: the port runs the resolved core regardless")
    if cfg.engine_type != "SI":
        notes.append(f'engine_type = "{cfg.engine_type}": the resolved core is spark-ignition only')
    if cfg.limiter_type != "fuel_cut":
        notes.append(f'limiter_type = "{cfg.limiter_type}": the port always uses a fuel-cut limiter')
    if not cfg.use_idle_air_control:
        notes.append("use_idle_air_control is false: the port always idles on the idle-air valve")
    for flag in ("use_fuel_film", "use_lambda_controller", "use_cyclic_variability",
                 "use_friction_breakdown", "compute_emissions"):
        if getattr(cfg, flag):
            notes.append(f"{flag} is on: not ported, ignored")
    if cfg.load_inertia_kgm2:
        notes.append("load_inertia_kgm2 is folded into `inertia`")
    return notes


# sound: engine_sim: key  ->  EngineConfig attribute of the same name
SOUND_KEYS = [
    "exhaust_layout",
    "exhaust_header_length_m", "exhaust_mid_length_m", "exhaust_muffler_length_m",
    "exhaust_tail_length_m", "exhaust_muffler_area_ratio", "exhaust_muffler_body_hz",
    "exhaust_header_temp_k", "exhaust_mid_temp_k", "exhaust_muffler_temp_k", "exhaust_tail_temp_k",
    "exhaust_pipe_loss", "exhaust_tip_damping",
    "cylinder_bank",
    "intake_runner_length_m", "intake_runner_temp_k",
    "sound_cylinder_strength_spread", "sound_cylinder_timing_spread_deg", "sound_noise_seed",
    "sound_blowdown_k", "sound_crank_ripple_gain", "sound_combustion_variability",
    "sound_idle_variability", "sound_pop_probability", "sound_starter_ratio", "sound_starter_poles",
]


def engine_yaml(cfg, vehicle=None, source: str = "", profile=None) -> str:
    """The YAML text for one EngineConfig (and, optionally, its VehicleConfig)."""
    out = []
    w = out.append
    w("# Generated by tools/engine_toml_to_yaml.py" + (f" from {source}" if source else ""))
    w(f"# {cfg.name}")
    w("detailed:")
    w(f"    bore: {_fmt(cfg.bore_m)}")
    w(f"    stroke: {_fmt(cfg.stroke_m)}")
    w(f"    rod_length: {_fmt(cfg.rod_length_m)}")
    w(f"    compression_ratio: {_fmt(cfg.compression_ratio)}")
    w(f"    num_cylinders: {cfg.num_cylinders}")
    w(f"    inertia: {_fmt(cfg.total_inertia)}")
    w(f"    idle_rpm: {_fmt(cfg.idle_rpm)}")
    w(f"    redline_rpm: {_fmt(cfg.redline_rpm)}")
    w(f"    limiter_hysteresis_rpm: {_fmt(cfg.limiter_hysteresis_rpm)}")
    w(f"    ambient_pressure_pa: {_fmt(cfg.ambient_pressure_pa)}")
    w(f"    intake_temperature_k: {_fmt(cfg.intake_temperature_k)}")
    w(f"    max_volumetric_efficiency: {_fmt(cfg.max_volumetric_efficiency)}")
    w(f"    stoich_afr: {_fmt(cfg.stoich_afr)}")
    w(f"    lower_heating_value: {_fmt(cfg.lower_heating_value)}")
    w(f"    combustion_efficiency: {_fmt(cfg.combustion_efficiency)}")
    w("    friction:")
    w(f"        c1_pa: {_fmt(cfg.friction_C1_Pa)}")
    w(f"        c2_pa_per_rev_s: {_fmt(cfg.friction_C2_Pa_per_rev_s)}")
    w(f"        c3_pa_per_rev_s2: {_fmt(cfg.friction_C3_Pa_per_rev_s2)}")
    w("    resolved:")
    for key in RESOLVED_KEYS:
        value = getattr(cfg, key)
        if value is None:                      # firing_intervals_deg: even fire
            continue
        w(f"        {key}: {_fmt(value)}")
    # The compiled core's resolution and budget are EngineSim's native_* settings.
    w(f"        crank_angle_resolution_deg: {_fmt(cfg.native_crank_angle_resolution_deg)}")
    w(f"        steps_per_ms: {cfg.native_steps_per_ms}")
    if cfg.use_turbo:
        w("        turbo:")
        for key in TURBO_KEYS:
            w(f"            {key}: {_fmt(getattr(cfg, 'turbo_' + key))}")

    if vehicle is not None:
        w("")
        w("# From the engine file's [vehicle] section - a starting point, not part of the engine.")
        w("gearbox:")
        w(f"    ratios: {_fmt([float(g) for g in vehicle.gear_ratios])}")
        w(f"    final_drive: {_fmt(float(vehicle.final_drive))}")
        w(f"    clutch_capacity_nm: {_fmt(float(vehicle.clutch_capacity_nm))}")

    w("")
    w("# For the vehicle's `sound:` block. `engine_sim:` is the engine file's [exhaust],")
    w("# [intake] and [sound] sections: with it the engine is heard through EngineSim's own")
    w("# synthesiser. The two firing lists are for this simulator's older engine sound,")
    w("# which is used if `engine_sim:` is removed.")
    fire = cfg.fire_angles_deg
    order = cfg.firing_order
    w("sound:")
    w(f"    firing_angles_deg: {_fmt([fire[c - 1] for c in order])}")
    w(f"    firing_banks: {_fmt([cfg.cylinder_bank[c - 1] for c in order])}")
    w("    engine_sim:")
    for key in SOUND_KEYS:
        value = getattr(cfg, key)
        w(f"        {key}: {value if isinstance(value, str) else _fmt(value)}")
    if profile:
        w("        profile:")
        for key, value in profile.items():
            w(f"            {key}: {_fmt(float(value))}")
    return "\n".join(out) + "\n"


def load(ref: str, enginesim: str = DEFAULT_ENGINESIM):
    sys.path.insert(0, enginesim)
    from engine_sim.engine_file import load_engine_file, resolve_engine     # noqa: E402
    return load_engine_file(resolve_engine(ref) if not ref.endswith(".toml") else ref)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("engine", help='EngineSim engine id ("1nz", "454", "vr38") or a .toml path')
    ap.add_argument("-o", "--output", help="write here instead of stdout")
    ap.add_argument("--enginesim", default=DEFAULT_ENGINESIM, help="path to the EngineSim project")
    args = ap.parse_args()

    le = load(args.engine, args.enginesim)
    for note in unsupported(le.engine):
        print(f"engine_toml_to_yaml: note: {note}", file=sys.stderr)
    text = engine_yaml(le.engine, le.vehicle, os.path.basename(le.path))
    if args.output:
        with open(args.output, "w") as f:
            f.write(text)
    else:
        sys.stdout.write(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
