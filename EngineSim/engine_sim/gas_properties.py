"""Working-fluid thermodynamic properties — temperature-dependent cp, cv, gamma.

All equations from ENGINE_KB.md (Heywood 1988) Part III.

Key equations:
    M_b = m_RP / n_b                          (KB Eq 4.10)
    M_u = m_RP / n_u                          (KB Eq 4.11)
    cv = (∂u/∂T)_v                            (KB Eq 4.12a)
    cp = (∂h/∂T)_p                            (KB Eq 4.12b)
    γ = cp / cv

Uses polynomial curve fits (NASA-style, KB Eq 4.39-4.41) for unburned
air-fuel mixture and burned products as functions of temperature.
"""

from __future__ import annotations
import math

# ── Gas constant ──
R_UNIVERSAL = 8314.46  # J/(kmol·K) — universal gas constant
R_AIR = 287.058        # J/(kg·K) — specific gas constant for air

# ── Polynomial coefficients for cp/R (NASA 7-coefficient form, 300-1000K range)
# Fits to JANAF data for major species (Heywood Table 4.11)
# Format: [a1, a2, a3, a4, a5, a6, a7] where
# cp/R = a1*T^-2 + a2*T^-1 + a3 + a4*T + a5*T^2 + a6*T^3 + a7*T^4

# N2 (dominant species in air and burned gas)
_N2_COEFFS = [0.0, 0.0, 3.53180, -0.12371e-6, 0.05152e-9, -0.47092e-13, 0.0]

# O2
_O2_COEFFS = [0.0, 0.0, 3.69757, 0.61340e-3, -0.12588e-6, 0.06790e-9, -0.08492e-13]

# CO2
_CO2_COEFFS = [0.0, 0.0, 2.40087, 0.52499e-2, -0.24100e-5, 0.06893e-9, -0.06398e-13]

# H2O
_H2O_COEFFS = [0.0, 0.0, 2.41000, 0.28905e-2, -0.10017e-5, 0.03697e-9, -0.04078e-13]

# Air (simplified: N2 + O2 weighted)
_AIR_COEFFS = [0.0, 0.0, 3.56847, 0.07770e-3, -0.00558e-6, 0.01830e-9, -0.01620e-13]


def _cp_over_R(T: float, coeffs: list) -> float:
    """cp/R from NASA polynomial. T in K."""
    T = max(T, 200.0)  # clamp for stability
    a1, a2, a3, a4, a5, a6, a7 = coeffs
    return (a1 / (T * T) + a2 / T + a3 + a4 * T
            + a5 * T * T + a6 * T * T * T + a7 * T * T * T * T)


_AIR_A3, _AIR_A4, _AIR_A5, _AIR_A6, _AIR_A7 = _AIR_COEFFS[2:]


def cp_air(T: float) -> float:
    """Specific heat at constant pressure for air [J/(kg·K)].
    Source: KB Eq 4.12b, Table 4.11.

    Args:
        T: Temperature [K].

    Returns:
        cp [J/(kg·K)].
    """
    if T < 200.0:
        T = 200.0
    # Horner form of the a1 = a2 = 0 polynomial (this is on the cylinder's hot path).
    return R_AIR * (_AIR_A3 + T * (_AIR_A4 + T * (_AIR_A5 + T * (_AIR_A6 + T * _AIR_A7))))


def cv_air(T: float) -> float:
    """Specific heat at constant volume for air [J/(kg·K)].
    Source: KB Eq 4.12a.

    Args:
        T: Temperature [K].

    Returns:
        cv [J/(kg·K)].
    """
    gamma = gamma_air(T)
    return cp_air(T) / gamma


def gamma_air(T: float) -> float:
    """Specific heat ratio γ = cp/cv for air.
    Source: KB Assumption A13.

    Args:
        T: Temperature [K].

    Returns:
        γ (dimensionless).
    """
    cp = cp_air(T)
    return cp / (cp - R_AIR)


def gas_constant_air() -> float:
    """Specific gas constant for air R = R_u / M_air [J/(kg·K)]."""
    return R_AIR


# ── Properties for burned gas mixture ──
# Simplified: use air-like properties at high T (dominated by N2, CO2, H2O)
# For more accurate model, would use equilibrium composition

def cp_burned(T: float, phi: float = 1.0) -> float:
    """Specific heat of burned gas mixture [J/(kg·K)].
    Accounts for composition effects via φ correction.

    Source: KB Eq 4.9-4.11, Fig 4-14.
    At stoichiometric (φ=1): M_b ≈ 28.8 g/mol (similar to air).
    Rich mixtures: lower M_b → higher cp per kg.

    Args:
        T: Temperature [K].
        phi: Equivalence ratio.

    Returns:
        cp [J/(kg·K)].
    """
    cp_base = cp_air(T)
    # φ correction: richer mixture has more CO2/H2O → higher cp
    phi_factor = 1.0 + 0.15 * max(0.0, phi - 1.0)
    return cp_base * phi_factor


def cv_burned(T: float, phi: float = 1.0) -> float:
    """Specific heat at constant volume for burned gas [J/(kg·K)]."""
    cp = cp_burned(T, phi)
    R = R_AIR
    return cp / (cp / (cp - R))


def gamma_burned(T: float, phi: float = 1.0) -> float:
    """Specific heat ratio for burned gas."""
    cp = cp_burned(T, phi)
    return cp / (cp - R_AIR)
