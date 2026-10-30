"""Tabulated working-fluid properties for the resolved cylinder.

The resolved cylinder carries a *burned mass fraction* y, so the gas it holds is
a mixture of unburned charge and burned products and its heat capacity depends
on temperature **and** on y. These are the lookup tables behind that, built once
at import on a 20 K grid (0..5000 K) so a substep needs only a list index:

    CV_AIR, CV_MIX, CV_BURNED   c_v of dry air / the unburned stoichiometric
                                petrol-air mixture / equilibrium burned gas
                                [J/(kg·K)]
    U_AIR, U_MIX, U_BURNED      sensible internal energy relative to 300 K [J/kg]
    DUB_AIR, DUB_MIX            U_BURNED − U_(unburned) at the same T [J/kg], the
                                energy offset when mass changes composition

Sources
  * Air: the NASA polynomial in gas_properties.py (KB Table 4.11).
  * Unburned mixture: air plus 1/(1+AFR) of petrol vapour, c_p,fuel ≈ 1.4 +
    0.0027·(T − 300) kJ/(kg·K). Gives γ ≈ 1.34 at 700 K (KB: γ_u ≈ 1.35).
  * Burned gas: stoichiometric equilibrium products. Frozen-composition c_p from
    JANAF for the N2/CO2/H2O mixture below ~1700 K, and the equilibrium values
    read from KB Fig 4-15 (30 atm, φ = 1: ≈ 1.45 / 1.7 / 2.45 kJ/(kg·K) at
    1750 / 2250 / 2750 K) above it. The steep rise above ~2200 K is
    dissociation absorbing energy — it is what holds real peak temperatures
    near 2700 K instead of the ~3400 K an air-like gas would give, and it is why
    γ_b is 1.15-1.25 rather than 1.3 during the expansion.
"""

from __future__ import annotations

from .gas_properties import cp_air
from .thermodynamics import R_AIR

GRID_STEP = 20.0                   # K
GRID_INV = 1.0 / GRID_STEP
GRID_N = 251                       # 0 .. 5000 K
T_REF = 300.0

_FUEL_MASS_FRACTION = 1.0 / (1.0 + 14.7)

# T [K] → equilibrium burned-gas cp [J/(kg·K)] (stoichiometric, ~30 atm)
_CP_BURNED_POINTS = [
    (200.0, 1050.0), (300.0, 1075.0), (500.0, 1128.0), (750.0, 1210.0),
    (1000.0, 1280.0), (1250.0, 1340.0), (1500.0, 1390.0), (1750.0, 1450.0),
    (2000.0, 1550.0), (2250.0, 1700.0), (2500.0, 2000.0), (2750.0, 2450.0),
    (3000.0, 2900.0), (3250.0, 3000.0), (3500.0, 2800.0), (4000.0, 2300.0),
    (4500.0, 2000.0), (5000.0, 1900.0),
]


def _interp(points, t: float) -> float:
    if t <= points[0][0]:
        return points[0][1]
    for (t0, v0), (t1, v1) in zip(points, points[1:]):
        if t <= t1:
            return v0 + (v1 - v0) * (t - t0) / (t1 - t0)
    return points[-1][1]


def _cp_burned(t: float) -> float:
    return _interp(_CP_BURNED_POINTS, t)


def _cp_mix(t: float) -> float:
    t = max(t, 200.0)
    cp_fuel = min(3600.0, 1400.0 + 2.7 * (t - 300.0))
    ca = cp_air(t)
    return ca + _FUEL_MASS_FRACTION * (cp_fuel - ca)


def _cv_table(cp_fn):
    return [cp_fn(max(k * GRID_STEP, 200.0)) - R_AIR for k in range(GRID_N)]


def _u_table(cv):
    """∫ c_v dT from 300 K on the grid (trapezoid), zero at the 300 K node."""
    k_ref = int(T_REF / GRID_STEP)
    u = [0.0] * GRID_N
    for k in range(k_ref + 1, GRID_N):
        u[k] = u[k - 1] + 0.5 * (cv[k - 1] + cv[k]) * GRID_STEP
    for k in range(k_ref - 1, -1, -1):
        u[k] = u[k + 1] - 0.5 * (cv[k] + cv[k + 1]) * GRID_STEP
    return u


CV_AIR = _cv_table(cp_air)
CV_MIX = _cv_table(_cp_mix)
CV_BURNED = _cv_table(_cp_burned)
U_AIR = _u_table(CV_AIR)
U_MIX = _u_table(CV_MIX)
U_BURNED = _u_table(CV_BURNED)
DUB_AIR = [b - a for a, b in zip(U_AIR, U_BURNED)]
DUB_MIX = [b - a for a, b in zip(U_MIX, U_BURNED)]


def cv_mixture(temperature_k: float, burned_fraction: float, unburned=CV_MIX) -> float:
    """c_v of the gas at T with burned mass fraction y [J/(kg·K)] (for tests and diagnostics)."""
    i = min(GRID_N - 1, max(0, int(temperature_k * GRID_INV)))
    cu = unburned[i]
    return cu + (CV_BURNED[i] - cu) * burned_fraction


def gamma_mixture(temperature_k: float, burned_fraction: float, unburned=CV_MIX) -> float:
    cv = cv_mixture(temperature_k, burned_fraction, unburned)
    return (cv + R_AIR) / cv


def internal_energy(temperature_k: float, burned_fraction: float = 0.0,
                    unburned=U_MIX) -> float:
    """Sensible internal energy relative to 300 K [J/kg], linear within a grid cell."""
    t = max(0.0, temperature_k)
    i = min(GRID_N - 2, int(t * GRID_INV))
    frac = (t - i * GRID_STEP) * GRID_INV
    uu = unburned[i] + (unburned[i + 1] - unburned[i]) * frac
    ub = U_BURNED[i] + (U_BURNED[i + 1] - U_BURNED[i]) * frac
    return uu + (ub - uu) * burned_fraction
