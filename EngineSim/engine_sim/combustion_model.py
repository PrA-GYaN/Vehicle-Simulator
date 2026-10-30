"""Combustion model — cycle-averaged air-standard Otto cycle.

All equations from ENGINE_KB.md (Heywood 1988).

The air-standard Otto cycle:
    1→2: Polytropic compression (n ≈ 1.32)
    2→3: Constant-volume heat addition (combustion)
    3→4: Polytropic expansion (n ≈ 1.28)
    4→1: Constant-volume heat rejection

Energy balance (KB Eq 2.21):
    W_net = Q_in - Q_out
    Q_in = m · c_v · (T3 - T2)
    Q_out = m · c_v · (T4 - T1)
"""

from __future__ import annotations
import math

from .thermodynamics import (
    R_AIR, GAMMA, CV_AIR, N_COMPRESSION, N_EXPANSION,
    polytropic_pressure, polytropic_temperature,
)
from .gas_properties import cv_air, cv_burned
from .piston_kinematics import cylinder_volume


def otto_cycle_work(p1: float, T1: float, Vc: float, rc: float,
                    R: float, mass: float,
                    fuel_energy: float, combustion_eff: float,
                    phi: float = 1.0) -> dict:
    """Compute air-standard Otto cycle quantities.

    Uses energy balance: W_net = Q_in - Q_out, not direct work integration.
    This avoids sign issues with polytropic exponents.

    Args:
        p1: Intake pressure (at IVC) [Pa].
        T1: Intake temperature (at IVC) [K].
        Vc: Clearance volume [m^3].
        rc: Compression ratio.
        R: Rod ratio (l/r).
        mass: Trapped mass [kg].
        fuel_energy: Total fuel chemical energy per cycle [J].
        combustion_eff: Combustion efficiency [0-1].
        phi: Equivalence ratio of the burned mixture (KB Eq 4.9-4.11) —
             used for composition-dependent burned-gas cv during heat
             rejection (state 4→1). Defaults to stoichiometric.

    Returns:
        Dict with cycle quantities.
    """
    if mass <= 0.0 or Vc <= 0.0:
        return {
            "T2": T1, "T3": T1, "T4": T1,
            "p2": p1, "p3": p1, "p4": p1,
            "W_comp": 0.0, "W_exp": 0.0,
            "W_net": 0.0, "T_ind": 0.0,
            "Q_in": 0.0, "Q_out": 0.0, "eta_th": 0.0,
        }

    # Volumes
    V_tdc = Vc
    V_bdc = Vc * rc

    # ── State 1→2: Polytropic compression ──
    p2 = polytropic_pressure(p1, V_bdc, V_tdc, N_COMPRESSION)
    T2 = polytropic_temperature(T1, V_bdc, V_tdc, N_COMPRESSION)

    # ── State 2→3: Constant-volume heat addition ──
    Q_in = fuel_energy * combustion_eff
    # Q_in = m * cv(T2) * (T3 - T2)  →  T3 = T2 + Q_in / (m * cv_T2)
    # Use cv at T2 (unburned gas) for compression end → combustion start
    cv_T2 = cv_air(T2)
    T3 = T2 + Q_in / (mass * cv_T2)
    p3 = p2 * (T3 / T2) if T2 > 0 else p2

    # ── State 3→4: Polytropic expansion ──
    p4 = polytropic_pressure(p3, V_tdc, V_bdc, N_EXPANSION)
    T4 = polytropic_temperature(T3, V_tdc, V_bdc, N_EXPANSION)

    # ── Energy balance ──
    # Q_out = m * cv_avg * (T4 - T1)  (heat rejected at constant volume)
    # State 4→1 is burned-gas exhaust, not air — use the equivalence-ratio-
    # dependent burned-gas cv (KB Eq 4.9-4.11), averaged between T4 and T1.
    cv_avg = cv_burned((T4 + T1) / 2.0, phi)
    Q_out = mass * cv_avg * (T4 - T1) if T4 > T1 else 0.0

    # Net work from first law: W_net = Q_in - Q_out
    W_net = Q_in - Q_out

    # Thermal efficiency
    eta_th = W_net / Q_in if Q_in > 0 else 0.0

    # Average indicated torque over one 4-stroke cycle (720° = 4π rad):
    # T_i = W_net / (n_R · 2π)  — KB Eq 15.2 with imep = W/V_d
    T_ind = W_net / (4.0 * math.pi)

    return {
        "T2": T2, "T3": T3, "T4": T4,
        "p2": p2, "p3": p3, "p4": p4,
        "W_comp": 0.0, "W_exp": 0.0,  # not used in energy-balance approach
        "W_net": W_net, "T_ind": T_ind,
        "Q_in": Q_in, "Q_out": Q_out, "eta_th": eta_th,
    }


def cycle_averaged_torque(p_intake: float, T_intake: float,
                          Vc: float, rc: float, R: float,
                          mass: float, lower_heating_value: float,
                          fuel_mass: float, combustion_eff: float,
                          phi: float = 1.0) -> float:
    """Compute cycle-averaged indicated torque.

    Args:
        p_intake: Intake pressure [Pa].
        T_intake: Intake temperature [K].
        Vc: Clearance volume [m^3].
        rc: Compression ratio.
        R: Rod ratio.
        mass: Trapped air mass [kg].
        lower_heating_value: Fuel LHV [J/kg].
        fuel_mass: Fuel mass per cycle [kg].
        combustion_eff: Combustion efficiency [0-1].
        phi: Equivalence ratio of the burned mixture. Defaults to
             stoichiometric.

    Returns:
        Average indicated torque over cycle [N·m].
    """
    if mass <= 0.0 or fuel_mass <= 0.0:
        return 0.0

    fuel_energy = fuel_mass * lower_heating_value
    result = otto_cycle_work(
        p_intake, T_intake, Vc, rc, R, mass,
        fuel_energy, combustion_eff, phi,
    )
    return result["T_ind"]


def wiebe_mass_fraction_burned(
    theta: float,
    theta_0: float,
    delta_theta: float,
    a: float = 5.0,
    m: float = 2.0,
) -> float:
    """Wiebe function for mass-fraction-burned vs crank angle.

    KB Eq 9.28 / 14.32:
        x_b = 1 − exp[ −a · ((θ − θ₀)/Δθ)^{m+1} ]

    This is the standard empirical S-curve fit to mass-fraction-burned data.
    Typical values: a=5, m=2.

    Args:
        theta: Current crank angle [degrees].
        theta_0: Start of combustion [degrees].
        delta_theta: Total combustion duration [degrees] (x_b = 0 to ~1).
        a: Wiebe efficiency parameter (default 5.0).
        m: Wiebe shape factor (default 2.0).

    Returns:
        Mass fraction burned x_b [0-1].
    """
    if delta_theta <= 0:
        return 0.0

    d_theta = theta - theta_0
    if d_theta <= 0:
        return 0.0
    if d_theta >= delta_theta:
        return 1.0

    x_b = 1.0 - math.exp(-a * (d_theta / delta_theta) ** (m + 1.0))
    return max(0.0, min(1.0, x_b))


def wiebe_heat_release_rate(
    theta: float,
    theta_0: float,
    delta_theta: float,
    fuel_energy: float,
    a: float = 5.0,
    m: float = 2.0,
) -> float:
    """Instantaneous heat release rate from Wiebe function.

    dQ/dθ = fuel_energy · dx_b/dθ

    where dx_b/dθ = a(m+1)/Δθ · ((θ−θ₀)/Δθ)^m · exp[−a((θ−θ₀)/Δθ)^{m+1}]

    Args:
        theta: Current crank angle [degrees].
        theta_0: Start of combustion [degrees].
        delta_theta: Total combustion duration [degrees].
        fuel_energy: Total fuel chemical energy [J].
        a: Wiebe parameter.
        m: Wiebe parameter.

    Returns:
        Heat release rate dQ/dθ [J/degree].
    """
    if delta_theta <= 0 or fuel_energy <= 0:
        return 0.0

    d_theta = theta - theta_0
    if d_theta <= 0 or d_theta >= delta_theta:
        return 0.0

    ratio = d_theta / delta_theta
    dx_dtheta = (a * (m + 1.0) / delta_theta
                 * ratio ** m
                 * math.exp(-a * ratio ** (m + 1.0)))

    return fuel_energy * dx_dtheta
