"""Everything the browser's sound synth and animation need to know about an engine.

Derived entirely from EngineConfig (which comes from the engine's TOML file),
so the JavaScript contains no engine-specific constants: change the file and
the exhaust resonances, valve events, firing pattern, starter whine and idle
lope all change with it. Sent to the page in /api/config as "sound".

Physical links (each documented in README.md §3):
    firing order / intervals / banks → when and on which pipe each pulse fires
    valve timing                     → exhaust-valve-open event (blowdown), intake pulse,
                                       valve-lift animation, cylinder-pressure plot
    exhaust lengths / temperatures   → pipe delays: c = 20.05·√T, delay = 2L/c
    muffler area ratio               → reflection at the chamber (A1−A2)/(A1+A2)
    bore and header gas speed        → blowdown pulse time constant  τ = k·bore/c
    engine inertia, cylinders, work  → crank-speed ripple  δ = g·W_cyl/(I·ω²)
    starter gearing                  → starter whine pitch
"""

from __future__ import annotations

import math
from typing import Any, Dict

from .config import EngineConfig
from .engine_file import exhaust_pipe_count


def speed_of_sound(temp_k: float) -> float:
    """Speed of sound in hot exhaust gas [m/s]: c = √(γ·R·T) ≈ 20.05·√T."""
    return 20.05 * math.sqrt(temp_k)


def sound_spec(cfg: EngineConfig) -> Dict[str, Any]:
    n = cfg.num_cylinders
    banks = max(cfg.cylinder_bank) + 1
    pipes = exhaust_pipe_count(cfg)
    dual = cfg.exhaust_layout == "dual"

    def pick(values, k):
        return values[k] if k < len(values) else values[-1]

    c_head = speed_of_sound(cfg.exhaust_header_temp_k)
    c_mid = speed_of_sound(cfg.exhaust_mid_temp_k)
    c_muf = speed_of_sound(cfg.exhaust_muffler_temp_k)
    c_tail = speed_of_sound(cfg.exhaust_tail_temp_k)
    pipe_list = []
    for k in range(pipes):
        pipe_list.append({
            "sections": [
                {"name": "header", "length_m": pick(cfg.exhaust_header_length_m, k), "c": c_head, "area": 1.0},
                {"name": "mid", "length_m": pick(cfg.exhaust_mid_length_m, k), "c": c_mid, "area": 1.0},
                {"name": "muffler", "length_m": pick(cfg.exhaust_muffler_length_m, k), "c": c_muf,
                 "area": cfg.exhaust_muffler_area_ratio},
                {"name": "tail", "length_m": pick(cfg.exhaust_tail_length_m, k), "c": c_tail, "area": 1.0},
            ],
            "body_hz": pick(cfg.exhaust_muffler_body_hz, k),
        })
    io, idur = cfg.intake_lift_window
    eo, edur = cfg.exhaust_lift_window
    return {
        "cylinders": n,
        "fire_angle_deg": cfg.fire_angles_deg,           # cycle angle of each cylinder's firing TDC
        "bank": list(cfg.cylinder_bank),
        "banks": banks,
        "bank_angle_deg": cfg.bank_angle_deg,
        # exhaust-valve-open and intake-valve-open, in degrees after that cylinder's firing TDC
        "evo_after_fire_deg": eo - 360.0,
        "ivo_after_fire_deg": (io - 360.0) % 720.0,
        "valves": {"intake_open": io, "intake_dur": idur, "exhaust_open": eo, "exhaust_dur": edur},
        "exhaust": {
            "layout": cfg.exhaust_layout,
            "cylinder_pipe": list(cfg.cylinder_bank) if dual else [0] * n,
            "pipes": pipe_list,
            "pipe_loss": cfg.exhaust_pipe_loss,
            "tip_damping": cfg.exhaust_tip_damping,
        },
        "intake": {
            "runner_length_m": cfg.intake_runner_length_m,
            "runner_c": speed_of_sound(cfg.intake_runner_temp_k),
        },
        "blowdown_tau_max_s": cfg.sound_blowdown_k * cfg.bore_m / c_head,
        "inertia_kgm2": cfg.engine_inertia_kgm2,
        "ripple_gain": cfg.sound_crank_ripple_gain,
        "cylinder_strength_spread": cfg.sound_cylinder_strength_spread,
        "cylinder_timing_spread_deg": cfg.sound_cylinder_timing_spread_deg,
        "seed": cfg.sound_noise_seed,
        "variability": {"base": cfg.sound_combustion_variability, "idle": cfg.sound_idle_variability},
        "pop_probability": cfg.sound_pop_probability,
        "starter": {"ratio": cfg.sound_starter_ratio, "poles": cfg.sound_starter_poles},
    }
