"""Fuel delivery model — fuel mass and AFR.

All equations from ENGINE_KB.md (Heywood 1988).

Key equations:
    φ = 1/λ = (A/F)_s / (A/F)        (KB Eq 3.8)
    m_fuel = m_air / (λ · (A/F)_s)    (derived from KB Eq 3.9)
"""

from __future__ import annotations
import math


def fuel_mass_per_cycle(air_mass: float, afr: float,
                        stoich_afr: float = 14.7) -> float:
    """Fuel mass per cylinder per cycle.

    For stoichiometric operation:
        m_fuel = m_air / (A/F)_s

    For lean/rich:
        m_fuel = m_air / (λ · (A/F)_s) = m_air / afr

    Args:
        air_mass: Trapped air mass per cylinder [kg].
        afr: Target air-fuel ratio.
        stoich_afr: Stoichiometric AFR (14.7 for gasoline).

    Returns:
        Fuel mass per cylinder per cycle [kg].
    """
    if afr <= 0.0:
        return 0.0
    return air_mass / afr


def lambda_from_afr(afr: float, stoich_afr: float = 14.7) -> float:
    """Lambda (excess air ratio).

    λ = (A/F) / (A/F)_s     (KB Eq 3.8 inverse)

    λ > 1: lean (excess air)
    λ = 1: stoichiometric
    < 1: rich (excess fuel)

    Args:
        afr: Actual air-fuel ratio.
        stoich_afr: Stoichiometric AFR.

    Returns:
        Lambda (dimensionless).
    """
    if stoich_afr <= 0.0:
        return 1.0
    return afr / stoich_afr


def equivalence_ratio(afr: float, stoich_afr: float = 14.7) -> float:
    """Equivalence ratio φ.

    φ = (A/F)_s / (A/F) = 1/λ    (KB Eq 3.8)

    φ > 1: rich
    φ = 1: stoichiometric
    φ < 1: lean

    Args:
        afr: Actual air-fuel ratio.
        stoich_afr: Stoichiometric AFR.

    Returns:
        Equivalence ratio (dimensionless).
    """
    lam = lambda_from_afr(afr, stoich_afr)
    return 1.0 / lam if lam > 0.0 else 1.0
