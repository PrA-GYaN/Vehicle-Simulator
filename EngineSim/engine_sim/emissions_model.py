"""Emissions models — NOx, CO, HC formation.

All equations from ENGINE_KB.md (Heywood 1988) Part IX.

Key equations:
    Extended Zeldovich mechanism for NO (KB Eq 11.1-11.13):
        O + N₂ = NO + N
        N + O₂ = NO + O
        N + OH = NO + H

    d[NO]/dt = (6×10¹⁶/T^{1/2})·exp(-69090/T)·[O₂]_e^{1/2}·[N₂]_e   (KB Eq 11.11)

    CO oxidation (KB Eq 11.21-11.22):
        d[CO]/dt = -4.3×10¹¹·[CO]·[O₂]^{0.25}·exp(-E/(RT))

    HC emissions (KB §11.4):
        Crevice, wall quench, oil absorption mechanisms
"""

from __future__ import annotations
import math


def nox_formation_rate(
    temperature: float,
    pressure: float,
    o2_molar_fraction: float,
    n2_molar_fraction: float = 0.71,
    dt: float = 0.001,
) -> float:
    """NO formation rate via extended Zeldovich mechanism.

    KB Eq 11.11:
        d[NO]/dt = (6×10¹⁶ / T^{1/2}) · exp(-69090/T) · [O₂]_e^{1/2} · [N₂]_e

    The strongly temperature-dependent activation energy means NO forms
    primarily at high temperatures (> 2000 K).

    Args:
        temperature: Local gas temperature [K].
        pressure: Cylinder pressure [Pa].
        o2_molar_fraction: O₂ mole fraction in burned gas.
        n2_molar_fraction: N₂ mole fraction (default 0.71 for air).
        dt: Timestep [s].

    Returns:
        NO formation [moles per unit volume per second] × dt = concentration increment.
    """
    if temperature < 1500.0:
        return 0.0  # negligible NO formation below ~1500 K

    # Temperature-dependent pre-exponential and activation energy
    # KB Eq 11.11
    A = 6.0e16
    E_R = 69090.0  # E_a/R for forward reaction [K]

    # Concentrations [mol/cm³] — convert from mole fraction
    # pV = nRT → n/V = p/(RT)
    R_cal = 82.057  # cm³·atm/(mol·K)
    p_atm = pressure / 101325.0  # convert Pa to atm
    conc_total = p_atm / (R_cal * temperature)  # mol/cm³

    o2_conc = o2_molar_fraction * conc_total
    n2_conc = n2_molar_fraction * conc_total

    # d[NO]/dt in mol/(cm³·s)
    dNO_dt = (A / math.sqrt(temperature)
              * math.exp(-E_R / temperature)
              * math.sqrt(max(0.0, o2_conc))
              * n2_conc)

    return dNO_dt * dt  # concentration increment


def nox_equilibrium_concentration(
    temperature: float,
    pressure: float,
    o2_molar_fraction: float = 0.05,
    n2_molar_fraction: float = 0.71,
) -> float:
    """Equilibrium NO concentration.

    KB Eq 11.8-11.10:
        [NO]_e = (K_NO · [O₂]_e · [N₂]_e)^{1/2}

    Args:
        temperature: Temperature [K].
        pressure: Pressure [Pa].
        o2_molar_fraction: O₂ mole fraction.
        n2_molar_fraction: N₂ mole fraction.

    Returns:
        Equilibrium NO mole fraction.
    """
    if temperature < 1500.0:
        return 0.0

    # Equilibrium constant (KB Eq 11.10)
    K_NO = 3.6e3 * math.exp(-31090.0 / temperature)  # atm^{-1/2}

    p_atm = pressure / 101325.0
    R_cal = 82.057
    conc_total = p_atm / (R_cal * temperature)

    o2_conc = o2_molar_fraction * conc_total
    n2_conc = n2_molar_fraction * conc_total

    no_eq = math.sqrt(K_NO * o2_conc * n2_conc)

    return no_eq / conc_total  # convert back to mole fraction


def co_oxidation_rate(
    temperature: float,
    co_molar_fraction: float,
    o2_molar_fraction: float,
    dt: float = 0.001,
) -> float:
    """CO oxidation rate.

    KB Eq 11.21-11.22 (from Problem 3.8):
        d[CO]/dt = -4.3×10¹¹ · [CO] · [O₂]^{0.25} · exp(-E/(RT))
        E/R = 20,000 K

    Args:
        temperature: Gas temperature [K].
        co_molar_fraction: CO mole fraction.
        o2_molar_fraction: O₂ mole fraction.
        dt: Timestep [s].

    Returns:
        CO concentration change (negative = consumption).
    """
    if temperature < 1000.0 or co_molar_fraction <= 0:
        return 0.0

    A = 4.3e11
    E_R = 20000.0  # K

    # Rate of CO disappearance
    dCO_dt = -A * co_molar_fraction * (o2_molar_fraction ** 0.25) * math.exp(-E_R / temperature)

    return dCO_dt * dt


def hc_emissions_fraction(
    equivalence_ratio: float,
    combustion_efficiency: float = 0.95,
    crevice_fraction: float = 0.02,
    wall_quench_fraction: float = 0.005,
) -> float:
    """HC emissions as fraction of fuel carbon.

    KB §11.4:
        Sources: crevices (~2%), wall quenching (~0.5%),
        oil absorption/desorption, bulk quenching.

    For stoichiometric SI engines, typical HC = 0.5-2% of fuel carbon.

    Args:
        equivalence_ratio: Equivalence ratio φ.
        combustion_efficiency: Overall combustion efficiency.
        crevice_fraction: Unburned HC from crevices.
        wall_quench_fraction: Unburned HC from wall quenching.

    Returns:
        HC emission index [kg HC / kg fuel].
    """
    # Base HC from incomplete combustion
    incomplete = 1.0 - combustion_efficiency

    # Rich mixtures → more HC (insufficient O₂)
    if equivalence_ratio > 1.0:
        rich_factor = 1.0 + 2.0 * (equivalence_ratio - 1.0)
    else:
        rich_factor = 1.0

    # Total HC fraction
    hc_fraction = (incomplete + crevice_fraction + wall_quench_fraction) * rich_factor

    return min(0.1, hc_fraction)  # cap at 10%


def nox_emission_index(
    peak_temperature: float,
    residence_time_ms: float,
    o2_fraction: float = 0.05,
    pressure_bar: float = 50.0,
) -> float:
    """Simplified NOx emission index.

    Uses integral of Zeldovich rate over characteristic time.

    Args:
        peak_temperature: Peak combustion temperature [K].
        residence_time_ms: Time at peak temperature [ms].
        o2_fraction: O₂ mole fraction in burned gas.
        pressure_bar: Cylinder pressure [bar].

    Returns:
        NOx emission index [g NOx / kg fuel].
    """
    if peak_temperature < 1800.0:
        return 0.1  # very low NOx at low T

    # Simplified: use rate at peak T × residence time
    # KB Eq 11.11
    A = 6.0e16
    E_R = 69090.0

    rate = A / math.sqrt(peak_temperature) * math.exp(-E_R / peak_temperature)
    rate *= math.sqrt(max(0.0, o2_fraction))

    # Integrate over residence time
    residence_s = residence_time_ms / 1000.0
    no_conc = rate * residence_s  # approximate [mol/cm³]

    # Convert to emission index (g NOx / kg fuel)
    # Typical range: 1-20 g/kg for SI engines
    ei_nox = no_conc * 1e6  # rough scaling

    return max(0.1, min(50.0, ei_nox))
