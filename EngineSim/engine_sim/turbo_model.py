"""Turbocharger model — compressor/turbine matching.

All equations from ENGINE_KB.md (Heywood 1988) Part V.

Key equations:
    Compressor isentropic efficiency (KB Eq 6.38-6.42):
        η_CTT = (T_{02s} - T_{01}) / (T_{02} - T_{01})

    Turbine power (KB Eq 6.44-6.49):
        Ẇ_T = η_T · ṁ_T · c_p · T_{04} · [1 - (p_{05}/p_{04})^((γ-1)/γ)]

    Turbo shaft balance (KB Eq 6.49):
        −Ẇ_C = η_m · Ẇ_T

    Boost pressure:
        p_boost = p_atm · π_C  (compressor pressure ratio)
"""

from __future__ import annotations
import math

from .thermodynamics import R_AIR, GAMMA


class TurboState:
    """Dynamic state of the turbocharger."""

    def __init__(self):
        self.shaft_speed_rps: float = 0.0       # turbo shaft speed [rev/s]
        self.compressor_pressure_ratio: float = 1.0
        self.compressor_efficiency: float = 0.72
        self.turbine_efficiency: float = 0.78
        self.boost_pressure_pa: float = 101325.0
        self.compressor_outlet_temp_k: float = 293.0
        self.is_wastegate_active: bool = False
        self.wastegate_position: float = 0.0  # 0=closed, 1=fully open

    def reset(self) -> None:
        self.shaft_speed_rps = 0.0
        self.compressor_pressure_ratio = 1.0
        self.compressor_efficiency = 0.72
        self.turbine_efficiency = 0.78
        self.boost_pressure_pa = 101325.0
        self.compressor_outlet_temp_k = 293.0
        self.is_wastegate_active = False
        self.wastegate_position = 0.0


def compressor_power(
    mass_flow_rate: float,
    inlet_temp_k: float,
    pressure_ratio: float,
    compressor_efficiency: float,
    gamma: float = GAMMA,
    cp: float = 1005.0,
) -> float:
    """Compressor power requirement.

    KB Eq 6.42:
        −Ẇ_C = ṁ_i · c_{p,i} · (T_{02} - T_{01})
              = [ṁ_i · c_{p,i} · T_{01} / η_CTT] · [(p_{02}/p_{01})^((γ-1)/γ) - 1]

    Args:
        mass_flow_rate: Air mass flow through compressor [kg/s].
        inlet_temp_k: Compressor inlet temperature [K].
        pressure_ratio: π_C = p_02/p_01.
        compressor_efficiency: Isentropic efficiency η_CTT.
        gamma: Specific heat ratio.
        cp: Specific heat at constant pressure [J/(kg·K)].

    Returns:
        Compressor power [W] (negative = power required).
    """
    if compressor_efficiency <= 0 or mass_flow_rate <= 0:
        return 0.0

    T01 = inlet_temp_k
    pi_c = max(1.0, pressure_ratio)

    # Isentropic outlet temperature (KB Eq 6.39)
    T02s = T01 * pi_c ** ((gamma - 1.0) / gamma)

    # Actual outlet temperature
    T02 = T01 + (T02s - T01) / compressor_efficiency

    # Power (KB Eq 6.42)
    W_dot_C = mass_flow_rate * cp * (T02 - T01)

    return W_dot_C  # positive = power consumed


def compressor_outlet_temp(
    inlet_temp_k: float,
    pressure_ratio: float,
    compressor_efficiency: float,
    gamma: float = GAMMA,
) -> float:
    """Compressor outlet temperature.

    Args:
        inlet_temp_k: Inlet temperature [K].
        pressure_ratio: π_C.
        compressor_efficiency: η_CTT.

    Returns:
        Outlet temperature [K].
    """
    T01 = inlet_temp_k
    pi_c = max(1.0, pressure_ratio)
    T02s = T01 * pi_c ** ((gamma - 1.0) / gamma)
    return T01 + (T02s - T01) / compressor_efficiency


def turbine_power(
    mass_flow_rate: float,
    inlet_temp_k: float,
    pressure_ratio: float,
    turbine_efficiency: float,
    gamma: float = GAMMA,
    cp: float = 1100.0,
) -> float:
    """Turbine power output.

    KB Eq 6.44-6.48:
        Ẇ_T = η_T · ṁ_T · c_p · T_{04} · [1 - (p_{05}/p_{04})^((γ-1)/γ)]

    Args:
        mass_flow_rate: Exhaust mass flow through turbine [kg/s].
        inlet_temp_k: Turbine inlet temperature [K] (exhaust temp).
        pressure_ratio: p_04/p_05 (inlet/outlet, >1).
        turbine_efficiency: η_T.
        gamma: Specific heat ratio for exhaust.
        cp: Specific heat for exhaust [J/(kg·K)].

    Returns:
        Turbine power [W] (positive = power produced).
    """
    if turbine_efficiency <= 0 or mass_flow_rate <= 0:
        return 0.0

    T04 = inlet_temp_k
    pr = max(1.0, pressure_ratio)

    # Isentropic expansion
    T05s = T04 * (1.0 / pr) ** ((gamma - 1.0) / gamma)

    # Actual expansion
    T05 = T04 - turbine_efficiency * (T04 - T05s)

    # Power (KB Eq 6.46)
    W_dot_T = mass_flow_rate * cp * (T04 - T05)

    return W_dot_T


def turbo_shaft_dynamics(
    turbine_power: float,
    compressor_power: float,
    shaft_speed_rps: float,
    shaft_inertia: float = 1e-5,
    mechanical_efficiency: float = 0.95,
    dt: float = 0.001,
) -> float:
    """Turbocharger shaft speed dynamics.

    KB Eq 6.49: −Ẇ_C = η_m · Ẇ_T (steady state)
    Dynamic: I_T · dω_T/dt = η_m · Ẇ_T − Ẇ_C

    Args:
        turbine_power: Power from turbine [W].
        compressor_power: Power to compressor [W].
        shaft_speed_rps: Current turbo shaft speed [rev/s].
        shaft_inertia: Turbo shaft inertia [kg·m^2].
        mechanical_efficiency: Bearing efficiency η_m.
        dt: Timestep [s].

    Returns:
        New shaft speed [rev/s].
    """
    omega = shaft_speed_rps * 2.0 * math.pi  # convert to rad/s

    # Net torque on turbo shaft
    T_turbine = turbine_power / max(omega, 1.0)
    T_compressor = compressor_power / max(omega, 1.0)
    T_net = mechanical_efficiency * T_turbine - T_compressor

    # Angular acceleration
    if shaft_inertia > 0:
        alpha = T_net / shaft_inertia
    else:
        alpha = 0.0

    omega_new = omega + alpha * dt
    omega_new = max(0.0, omega_new)

    return omega_new / (2.0 * math.pi)  # convert back to rev/s


def wastegate_control(
    boost_pressure_pa: float,
    target_boost_pa: float,
    current_position: float,
    dt: float = 0.001,
    gain: float = 0.5,
) -> tuple[float, bool]:
    """Wastegate position control.

    Opens wastegate when boost exceeds target.

    Args:
        boost_pressure_pa: Current boost pressure [Pa].
        target_boost_pa: Target boost pressure [Pa].
        current_position: Current wastegate position [0-1].
        dt: Timestep [s].
        gain: Controller gain.

    Returns:
        Tuple of (new_position, is_active).
    """
    error = boost_pressure_pa - target_boost_pa
    if error > 0:
        new_pos = current_position + gain * error / target_boost_pa * dt
    else:
        new_pos = current_position - gain * 0.5 * dt  # close slowly

    new_pos = max(0.0, min(1.0, new_pos))
    active = new_pos > 0.01

    return new_pos, active


def compressor_map(
    corrected_speed: float,
    corrected_flow: float,
    pressure_ratio: float,
) -> float:
    """Simplified compressor map — efficiency lookup.

    Returns compressor isentropic efficiency from corrected speed/flow/PR.
    Uses a parabolic efficiency surface centered on the design point.

    Args:
        corrected_speed: Corrected speed N/√T [rpm/√K].
        corrected_flow: Corrected mass flow ṁ√T/p [kg/s·√K/kPa].
        pressure_ratio: π_C.

    Returns:
        Compressor efficiency η_CTT.
    """
    # Design point (normalized)
    N_design = 80000.0  # rpm/√K
    m_design = 0.05     # kg/s·√K/kPa
    pi_design = 2.0

    # Normalized distances
    dN = (corrected_speed - N_design) / N_design
    dm = (corrected_flow - m_design) / m_design
    dpi = (pressure_ratio - pi_design) / pi_design

    # Parabolic efficiency surface: η = η_max * (1 - c1*dN² - c2*dm² - c3*dpi²)
    eta_max = 0.78
    c1, c2, c3 = 0.5, 0.3, 0.2

    eta = eta_max * (1.0 - c1 * dN ** 2 - c2 * dm ** 2 - c3 * dpi ** 2)

    return max(0.5, min(0.85, eta))
