"""Crank-resolved cylinder pressure trace for display (P–θ and P–V plots).

Single-zone first-law integration over one 720° cycle (KB Eq 14.14 form):

    dp/dθ = −γ·(p/V)·dV/dθ + (γ − 1)/V · dQ/dθ,     Q(θ) = Q_tot · x_b(θ)

with x_b the Wiebe burn fraction (KB Eq 9.28), V(θ) from the crank-slider
(KB Eq 2.6). Intake and exhaust strokes are held at manifold / exhaust
pressure and blowdown after EVO is an exponential decay — display-grade
simplifications (KB_GAPS.md G-E08), not used by the torque model.

Angles: 0° = TDC at start of intake, 360° = firing TDC.
"""

from __future__ import annotations

import math
from typing import Dict, List

from .config import EngineConfig
from .combustion_model import wiebe_mass_fraction_burned
from .piston_kinematics import cylinder_volume

GAMMA_TRACE = 1.32          # mean of unburned (~1.35) and burned (~1.28) mixture
BLOWDOWN_DEG = 18.0         # e-folding angle of the blowdown decay


def cylinder_pressure_trace(cfg: EngineConfig, map_pa: float, fuel_mass_kg: float,
                            spark_deg_btdc: float, burn_deg: float,
                            exhaust_pa: float = 105_000.0,
                            step_deg: float = 4.0) -> Dict[str, List[float]]:
    """Return {"theta": [...deg], "p": [...kPa], "v": [...cm³]} for one cycle."""
    Vc, rc, R = cfg.clearance_volume, cfg.compression_ratio, cfg.rod_ratio
    EVO_DEG = cfg.exhaust_lift_window[0]        # exhaust valve opening [cycle degrees], from the engine file
    q_tot = max(0.0, fuel_mass_kg) * cfg.lower_heating_value * cfg.combustion_efficiency
    theta0 = 360.0 - spark_deg_btdc
    burn = max(burn_deg, 10.0)
    g = GAMMA_TRACE

    th_out, p_out, v_out = [], [], []
    n = int(720.0 / step_deg)
    p = map_pa
    p_evo = None
    prev_V = cylinder_volume(math.radians(180.0), Vc, rc, R)
    prev_xb = 0.0
    for i in range(n + 1):
        th = i * step_deg
        V = cylinder_volume(math.radians(th), Vc, rc, R)
        if th < 180.0:
            p = map_pa
        elif th <= EVO_DEG:
            # Integrate with sub-steps for accuracy near TDC
            sub = 4
            for k in range(1, sub + 1):
                t = th - step_deg + k * step_deg / sub if th > 180.0 else 180.0
                Vk = cylinder_volume(math.radians(t), Vc, rc, R)
                xb = wiebe_mass_fraction_burned(t, theta0, burn, cfg.wiebe_a, cfg.wiebe_m)
                dQ = q_tot * (xb - prev_xb)
                p += -g * p / Vk * (Vk - prev_V) + (g - 1.0) / Vk * dQ
                prev_V, prev_xb = Vk, xb
            p_evo = p
        elif th < 540.0:
            p = exhaust_pa + (p_evo - exhaust_pa) * math.exp(-(th - EVO_DEG) / BLOWDOWN_DEG)
        else:
            p = exhaust_pa
        th_out.append(th)
        p_out.append(max(p, 500.0) / 1000.0)
        v_out.append(V * 1e6)
    return {"theta": th_out, "p": p_out, "v": v_out}


def resolved_pressure_trace(cylinder) -> Dict[str, List[float]]:
    """The pressure a resolved cylinder actually recorded, in the same shape.

    Unlike :func:`cylinder_pressure_trace` nothing here is reconstructed: the
    values are the pressure the first-law integration produced on its last pass
    through each crank angle (one per 4°), so the plot shows intake vacuum,
    valve events, blowdown and exhaust backpressure as the gas exchange made them.
    """
    from .cylinder import TRACE_BINS
    step = 720.0 / TRACE_BINS
    th_out = [i * step for i in range(TRACE_BINS + 1)]
    p_pa = list(cylinder.p_trace) + [cylinder.p_trace[0]]
    p_out = [max(p, 500.0) / 1000.0 for p in p_pa]
    v_out = [cylinder.volume(th) * 1e6 for th in th_out]
    return {"theta": th_out, "p": p_out, "v": v_out}
