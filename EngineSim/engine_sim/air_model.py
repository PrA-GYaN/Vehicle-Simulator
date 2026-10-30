"""Air intake model — volumetric efficiency and trapped mass.

All equations from ENGINE_KB.md (Heywood 1988).

Key equations:
    η_v ≈ 0.85 - 0.5·(N/6000)^1.5              (KB §2.10, Eq 2.26)
    m_trap = η_v · ρ_i · V_d  (per cycle; flow = m_trap·N/n_R, KB Eq 2.27)
    ρ_i = p_i / (R·T_i)                          (ideal gas at intake)
    p_i ≈ p_atm · throttle                        (GAP G-E01: assumption)
"""

from __future__ import annotations
import math

from .thermodynamics import R_AIR
from .piston_kinematics import cylinder_volume


def volumetric_efficiency(omega: float, max_eta_v: float = 0.92) -> float:
    """Volumetric efficiency vs engine speed.

    KB §2.10 Eq 2.26: η_v peaks around 0.5-0.7·N_max for NA engines.
    Simplified model: η_v decreases at high speed due to flow restrictions.

    Args:
        omega: Engine angular velocity [rad/s].
        max_eta_v: Maximum volumetric efficiency (typical 0.85-0.95).

    Returns:
        Volumetric efficiency [0-1].
    """
    N = omega * 60.0 / (2.0 * math.pi)  # RPM
    # Normalized speed: peaks at ~3000 RPM for typical NA SI
    N_norm = N / 6000.0
    # KB Eq 2.26: η_v decreases at high speed
    eta_v = max_eta_v * (1.0 - 0.5 * N_norm ** 1.5)
    return max(eta_v, 0.3)  # floor to prevent unrealistic values


def intake_pressure(throttle: float, ambient_pa: float = 101325.0) -> float:
    """Intake manifold pressure from throttle position.

    KB GAP G-E01: Throttle-to-torque relationship not explicitly given.
    Approximation: at WOT (throttle=1), p_i ≈ p_atm.
    At idle (throttle≈0), p_i ≈ 0.2·p_atm (throttled NA SI).

    Args:
        throttle: Throttle position [0.0, 1.0].
        ambient_pa: Ambient pressure [Pa].

    Returns:
        Intake pressure [Pa] (absolute).
    """
    # Nonlinear throttle model: p_i drops sharply at low throttle
    p_min = 0.2 * ambient_pa   # idle vacuum
    p_max = ambient_pa          # WOT
    return p_min + (p_max - p_min) * throttle ** 0.5


def trapped_mass(eta_v: float, p_i: float, T_i: float,
                 displacement: float, n_R: int = 2) -> float:
    """Trapped mass per cylinder per cycle.

    From the definition of η_v (KB Eq 2.27): ṁ_a = η_v · ρ_i · V_d · N / n_R,
    and there are N / n_R cycles per second, so the mass inducted per cycle is

        m_trap = η_v · ρ_i · V_d

    where ρ_i = p_i / (R · T_i). (An earlier revision also divided this by
    n_R, which halved the charge and was compensated by a 4x-too-large
    work→torque factor — together a net 2x torque error.)

    Args:
        eta_v: Volumetric efficiency.
        p_i: Intake pressure [Pa].
        T_i: Intake temperature [K].
        displacement: Displaced volume per cylinder [m^3].
        n_R: Revolutions per power stroke (2 for 4-stroke). Unused here —
             kept for call compatibility; it belongs in the flow rate.

    Returns:
        Trapped mass per cylinder [kg].
    """
    if T_i <= 0.0 or p_i <= 0.0:
        return 0.0
    rho_i = p_i / (R_AIR * T_i)
    return eta_v * rho_i * displacement
