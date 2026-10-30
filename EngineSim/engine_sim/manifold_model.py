"""Manifold filling/emptying dynamics and throttle-plate flow.

All equations from ENGINE_KB.md (Heywood 1988) Part VI.

Key equations:
    dm_{a,m}/dt = ṁ_{a,th} − Σ ṁ_{a,cyl}          (KB Eq 7.21)
    dp_m/dt = (R·T_m/V_m)(ṁ_{a,th} − η_v·ρ_m·V_d·N/2)   (KB Eq 7.22)

Throttle-plate orifice flow (KB Eq 7.18-7.20):
    Choked when p_downstream/p_upstream ≤ (2/(γ+1))^(γ/(γ-1))
    Subsonic: m_dot = C_D·A_T·p_up·sqrt(2γ/(R·T_up·(γ-1))·((p_down/p_up)^(2/γ) - (p_down/p_up)^((γ+1)/γ)))
"""

from __future__ import annotations
import math

from .thermodynamics import R_AIR, GAMMA


def throttle_flow_rate(p_upstream: float, p_downstream: float,
                       T_upstream: float, throttle_area: float,
                       discharge_coeff: float = 0.65) -> float:
    """Mass flow rate through throttle plate.

    KB Eq 7.18-7.20: 1-D compressible flow through an orifice.
    Choked flow when pressure ratio < critical.

    Args:
        p_upstream: Pressure upstream [Pa] (intake side, ~ambient).
        p_downstream: Pressure downstream [Pa] (manifold pressure).
        T_upstream: Temperature upstream [K].
        throttle_area: Effective throttle area [m^2].
        discharge_coeff: Throttle discharge coefficient (0.6-0.7 typical).

    Returns:
        Mass flow rate [kg/s].
    """
    if p_upstream <= 0 or T_upstream <= 0 or throttle_area <= 0:
        return 0.0

    gamma = GAMMA
    R = R_AIR
    pr = p_downstream / p_upstream

    # No forward flow possible if downstream isn't lower than upstream
    # (e.g. a transient where manifold pressure briefly exceeds boost).
    if pr >= 1.0:
        return 0.0

    # Critical pressure ratio for choking (KB Eq 7.18)
    pr_crit = (2.0 / (gamma + 1.0)) ** (gamma / (gamma - 1.0))

    if pr <= pr_crit:
        # Choked flow (KB Eq 7.19)
        m_dot = (discharge_coeff * throttle_area * p_upstream
                 * math.sqrt(2.0 / (R * T_upstream)
                             * gamma / (gamma + 1.0)
                             * (2.0 / (gamma + 1.0)) ** (1.0 / (gamma - 1.0))))
    else:
        # Subsonic flow (KB Eq 7.20). The bracket term is analytically >= 0
        # for pr in (pr_crit, 1), but can go slightly negative right at the
        # pr -> 1 boundary from floating-point rounding — clamp it.
        bracket = max(0.0, pr ** (2.0 / gamma) - pr ** ((gamma + 1.0) / gamma))
        m_dot = (discharge_coeff * throttle_area * p_upstream
                 * math.sqrt(2.0 * gamma / (R * T_upstream * (gamma - 1.0)) * bracket))

    return max(m_dot, 0.0)


def throttle_effective_area(throttle_pos: float, max_area: float = 0.003) -> float:
    """Effective throttle area from pedal position [0-1].

    Simplified model: area scales roughly with cos(θ) for butterfly valve.
    At idle (closed): small residual area. At WOT: full area.

    Args:
        throttle_pos: Throttle position [0, 1].
        max_area: Maximum effective area at WOT [m^2].

    Returns:
        Effective area [m^2].
    """
    # Butterfly valve: the plate rotates θ = pos·90° from closed, and the open
    # annulus grows as A_eff ≈ A_max·(1 − cos θ) — very progressive: 10 % pedal
    # (9°) opens ~1 % of the area, 50 % opens ~29 %. (An earlier power law,
    # pos^0.8, opened 16 % at 10 % pedal, enough air to free-rev a big V8 into
    # the limiter at the lightest touch.)
    pos = max(0.0, min(1.0, throttle_pos))
    return max_area * (1.0 - math.cos(pos * math.pi / 2.0))


def manifold_pressure_dynamics(p_manifold: float, m_dot_throttle: float,
                               m_dot_cylinders: float, V_manifold: float,
                               T_manifold: float, dt: float) -> float:
    """Manifold pressure dynamics (filling/emptying).

    KB Eq 7.22:
        dp_m/dt = (R·T_m/V_m)(ṁ_{a,th} − Σ ṁ_{a,cyl})

    where Σ ṁ_{a,cyl} = η_v · ρ_m · V_d · N / 2

    Args:
        p_manifold: Current manifold pressure [Pa].
        m_dot_throttle: Air flow past throttle [kg/s].
        m_dot_cylinders: Air flow into cylinders [kg/s].
        V_manifold: Manifold plenum volume [m^3].
        T_manifold: Manifold gas temperature [K] (assumed ~constant).
        dt: Timestep [s].

    Returns:
        New manifold pressure [Pa].
    """
    if V_manifold <= 0 or T_manifold <= 0:
        return p_manifold

    R = R_AIR
    dp_dt = (R * T_manifold / V_manifold) * (m_dot_throttle - m_dot_cylinders)
    p_new = p_manifold + dp_dt * dt

    # Clamp: can't go below absolute zero or above ambient * reasonable factor
    p_new = max(p_new, 1000.0)  # minimum ~1 kPa (deep vacuum)
    p_new = min(p_new, 300000.0)  # max ~3 atm (boosted)

    return p_new


def manifold_time_constant(V_manifold: float, eta_v: float,
                           V_displaced: float, N_rps: float) -> float:
    """Manifold pressure response time constant.

    KB Eq 7.22 basis: τ = 2·V_m/(η_v·V_d·N)

    Args:
        V_manifold: Manifold plenum volume [m^3].
        eta_v: Volumetric efficiency.
        V_displaced: Total engine displacement [m^3].
        N_rps: Engine speed [rev/s].

    Returns:
        Time constant [s].
    """
    denominator = eta_v * V_displaced * N_rps
    if denominator <= 0:
        return 1.0  # default fallback
    return 2.0 * V_manifold / denominator
