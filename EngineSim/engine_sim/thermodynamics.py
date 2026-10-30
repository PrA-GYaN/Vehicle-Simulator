"""Thermodynamic constants and polytropic process equations.

All equations from ENGINE_KB.md (Heywood 1988).

Key equations:
    Polytropic process: pV^n = const
    p2 = p1 * (V1/V2)^n                    (implicit from KB §VII)
    W = (p2*V2 - p1*V1) / (1 - n)          (KB Eq 2.21)
    Ideal gas: pV = mRT
"""

from __future__ import annotations
import math

# ── Physical constants ──
R_AIR = 287.058          # J/(kg·K) — specific gas constant for air (KB Table D.1)
GAMMA = 1.4              # specific heat ratio for air (KB Assumption A13)
CV_AIR = R_AIR / (GAMMA - 1.0)   # ~718 J/(kg·K)
CP_AIR = GAMMA * CV_AIR           # ~1005 J/(kg·K)

# Polytropic exponents (KB Assumption A13)
N_COMPRESSION = 1.32     # compression stroke polytropic index
N_EXPANSION = 1.28       # expansion stroke polytropic index

# ── Polytropic process equations ──


def polytropic_pressure(p1: float, V1: float, V2: float, n: float) -> float:
    """Pressure from polytropic process pV^n = const.

    p2 = p1 * (V1 / V2)^n

    Args:
        p1: Initial pressure [Pa].
        V1: Initial volume [m^3].
        V2: Final volume [m^3].
        n: Polytropic exponent.

    Returns:
        Final pressure [Pa].
    """
    if V2 <= 0.0:
        return p1
    return p1 * (V1 / V2) ** n


def polytropic_temperature(T1: float, V1: float, V2: float, n: float) -> float:
    """Temperature from polytropic process.

    T2 = T1 * (V1 / V2)^(n-1)

    Args:
        T1: Initial temperature [K].
        V1: Initial volume [m^3].
        V2: Final volume [m^3].
        n: Polytropic exponent.

    Returns:
        Final temperature [K].
    """
    if V2 <= 0.0:
        return T1
    return T1 * (V1 / V2) ** (n - 1.0)


def polytropic_work(p1: float, V1: float, p2: float, V2: float, n: float) -> float:
    """Work done during polytropic process.

    W = (p2*V2 - p1*V1) / (1 - n)    (KB Eq 2.21)

    Positive W means work done BY the gas (expansion).
    Negative W means work done ON the gas (compression).

    Args:
        p1: Initial pressure [Pa].
        V1: Initial volume [m^3].
        p2: Final pressure [Pa].
        V2: Final volume [m^3].
        n: Polytropic exponent.

    Returns:
        Work done [J].
    """
    if abs(n - 1.0) < 1e-12:
        # Isothermal limit
        return p1 * V1 * math.log(V2 / V1) if V1 > 0 else 0.0
    return (p2 * V2 - p1 * V1) / (1.0 - n)


def ideal_gas_pressure(m: float, T: float, V: float) -> float:
    """Pressure from ideal gas law: pV = mRT.

    Args:
        m: Mass [kg].
        T: Temperature [K].
        V: Volume [m^3].

    Returns:
        Pressure [Pa].
    """
    if V <= 0.0:
        return 0.0
    return m * R_AIR * T / V
