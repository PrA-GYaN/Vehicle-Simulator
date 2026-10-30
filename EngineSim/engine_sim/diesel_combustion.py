"""Diesel/CI combustion model — premixed + mixing-controlled burning.

All equations from ENGINE_KB.md (Heywood 1988) Part VIII.

Key equations:
    Ignition delay (KB Eq 10.35-10.38):
        τ_id = A · p⁻ⁿ · exp(E_a/(R·T))

    Watson heat-release model (KB Eq 14.46-14.50):
        x_b(t) = x_b,premixed(t) + x_b,mixing(t)
        Premixed: rapid initial spike after ignition delay
        Mixing-controlled: gradual burn ∝ fuel-air mixing rate
"""

from __future__ import annotations
import math


class DieselCombustionState:
    """State of diesel combustion process."""

    def __init__(self):
        self.ignition_delay_s: float = 0.0
        self.fuelInjected_per_cycle_kg: float = 0.0
        self.premixed_fraction: float = 0.2
        self.combustion_duration_deg: float = 60.0
        self.mass_fraction_burned: float = 0.0

    def reset(self) -> None:
        self.ignition_delay_s = 0.0
        self.fuelInjected_per_cycle_kg = 0.0
        self.premixed_fraction = 0.2
        self.combustion_duration_deg = 60.0
        self.mass_fraction_burned = 0.0


def ignition_delay(
    temperature: float,
    pressure: float,
    fuel_cetane: float = 50.0,
) -> float:
    """Diesel ignition delay.

    KB Eq 10.35-10.38:
        τ_id = A · p⁻ⁿ · exp(E_a/(R·T))

    Calibrated for typical diesel fuel.

    Args:
        temperature: Air temperature at TDC [K].
        pressure: Cylinder pressure at injection [Pa].
        fuel_cetane: Cetane number of fuel.

    Returns:
        Ignition delay [seconds].
    """
    if temperature <= 0 or pressure <= 0:
        return 0.01

    # Base: isooctane-like reference
    A = 1.0e-3
    n = 1.0
    E_R = 3000.0  # K = E_a/R for diesel autoignition (~25 kJ/mol)

    p_bar = pressure / 1e5
    tau = A * p_bar ** (-n) * math.exp(E_R / temperature)

    # Cetane correction: higher CN → shorter delay
    cn_factor = 1.0 + 0.02 * (50.0 - fuel_cetane)
    tau *= max(0.3, min(3.0, cn_factor))

    return max(0.0005, min(0.05, tau))


def watson_heat_release(
    theta: float,
    theta_start: float,
    delta_theta_premixed: float,
    delta_theta_mixing: float,
    premixed_fraction: float = 0.2,
    total_combustion_deg: float = 60.0,
) -> float:
    """Watson two-phase heat release model.

    KB Eq 14.46-14.50:
        Premixed phase: rapid spike (Wiebe with short duration)
        Mixing-controlled phase: slower burn (Wiebe with longer duration)

    Args:
        theta: Current crank angle [degrees].
        theta_start: Start of combustion [degrees].
        delta_theta_premixed: Duration of premixed phase [degrees].
        delta_theta_mixing: Duration of mixing phase [degrees].
        premixed_fraction: Fraction of fuel burned in premixed phase.
        total_combustion_deg: Total combustion duration [degrees].

    Returns:
        Cumulative mass fraction burned [0-1].
    """
    if delta_theta_premixed <= 0 or delta_theta_mixing <= 0:
        return 0.0

    d_theta = theta - theta_start
    if d_theta <= 0:
        return 0.0

    # Premixed phase (Wiebe, a=5, m=0.8 — rapid)
    if d_theta <= delta_theta_premixed:
        a_premixed = 5.0
        m_premixed = 0.8
        x_premixed = (1.0 - math.exp(-a_premixed * (d_theta / delta_theta_premixed) ** (m_premixed + 1.0)))
        return premixed_fraction * x_premixed

    # Mixing-controlled phase
    d_theta_mix = d_theta - delta_theta_premixed
    if d_theta_mix <= 0:
        return premixed_fraction

    a_mixing = 5.0
    m_mixing = 2.0
    x_mixing = (1.0 - math.exp(-a_mixing * (d_theta_mix / delta_theta_mixing) ** (m_mixing + 1.0)))

    return premixed_fraction + (1.0 - premixed_fraction) * x_mixing


def diesel_combustion_duration(
    speed_rpm: float,
    load_fraction: float,
    swirl_ratio: float = 2.0,
) -> float:
    """Total combustion duration for diesel engine.

    KB Fig 10-7, §10.3:
        Mixing-controlled combustion dominates.
        Duration depends on mixing rate (swirl, injection timing).

    Args:
        speed_rpm: Engine speed [RPM].
        load_fraction: Relative load [0-1].
        swirl_ratio: Swirl ratio (higher → faster mixing → shorter).

    Returns:
        Total combustion duration [crank-angle degrees].
    """
    # Base duration at 2000 RPM, WOT
    base = 60.0  # degrees

    # Speed effect: more degrees at higher speed
    speed_factor = speed_rpm / 2000.0

    # Load effect: more fuel → longer duration
    load_factor = 0.5 + 0.5 * load_fraction

    # Swirl effect: more swirl → faster mixing → shorter
    swirl_factor = 1.0 / max(0.5, swirl_ratio / 2.0)

    duration = base * speed_factor * load_factor * swirl_factor

    return max(20.0, min(100.0, duration))


def premixed_fraction(
    ignition_delay_s: float,
    injection_rate_kg_s: float,
    air_mass_kg: float,
    stoich_afr: float = 32.0,
) -> float:
    """Fraction of fuel burned in premixed phase.

    KB §10.3.1, Fig 10-7:
        During ignition delay, fuel continues to be injected and
        mixes with air. This premixed fuel burns rapidly once
        ignition occurs.

    More fuel injected during delay → larger premixed spike.

    Args:
        ignition_delay_s: Ignition delay [s].
        injection_rate_kg_s: Fuel injection rate [kg/s].
        air_mass_kg: Air mass in cylinder [kg].
        stoich_afr: Stoichiometric AFR for diesel (~32).

    Returns:
        Premixed fraction [0-1].
    """
    fuel_during_delay = ignition_delay_s * injection_rate_kg_s
    air_for_combustion = air_mass_kg / stoich_afr

    if air_for_combustion <= 0:
        return 0.2

    # Fraction of stoichiometric fuel that was premixed
    premixed = fuel_during_delay / air_for_combustion

    return max(0.05, min(0.6, premixed))
