"""Cyclic combustion variability model.

All equations from ENGINE_KB.md (Heywood 1988) Part VII.

Key equations:
    COV_imep = (σ_imep / imep) × 100         (KB Eq 9.50)
    Δθ_d = C·(l_I/u')^{1/3}·(l_M/S_L)^{2/3}  (KB Eq 9.51)

Cyclic variability increases near:
    - Lean limit (φ < 0.9)
    - High EGR rates
    - Idle / light load
    - Low turbulence levels
"""

from __future__ import annotations
import math
import random


def cyclic_variability_cov(
    speed_rpm: float,
    load_fraction: float,
    equivalence_ratio: float = 1.0,
    egr_fraction: float = 0.0,
    residual_fraction: float = 0.05,
) -> float:
    """Estimate COV of IMEP.

    KB Eq 9.50, Fig 9-36:
        COV_imep increases with:
        - Lean mixtures (φ < 1)
        - High EGR
        - Low load
        - Low speed (at idle)

    Typical: 2-5% at normal operation, up to 10-15% near limits.

    Args:
        speed_rpm: Engine speed [RPM].
        load_fraction: Relative load [0-1].
        equivalence_ratio: Equivalence ratio φ.
        egr_fraction: EGR fraction [0-1].
        residual_fraction: Residual gas fraction [0-1].

    Returns:
        COV of IMEP [%].
    """
    # Base COV at stoichiometric, WOT, 3000 RPM
    cov_base = 2.0  # percent

    # Equivalence ratio effect: increases sharply for lean mixtures
    # KB Fig 9-36: COV rises steeply below φ ≈ 0.9
    if equivalence_ratio < 1.0:
        phi_factor = 1.0 + 20.0 * (1.0 - equivalence_ratio) ** 2
    else:
        phi_factor = 1.0 + 5.0 * (equivalence_ratio - 1.0) ** 2

    # EGR effect: increases COV
    # KB Fig 9-36: ~10% COV at 20% EGR, rises steeply above
    egr_factor = 1.0 + 15.0 * egr_fraction ** 2

    # Dilution from residuals
    dilution = egr_fraction + residual_fraction
    dilution_factor = 1.0 + 10.0 * dilution ** 2

    # Load effect: low load → higher COV
    load_factor = 1.0 / max(load_fraction, 0.05)

    # Speed effect: idle is worst
    if speed_rpm < 1200:
        speed_factor = 1.0 + 2.0 * (1200.0 - speed_rpm) / 1200.0
    else:
        speed_factor = 1.0

    cov = cov_base * phi_factor * egr_factor * dilution_factor * load_factor * speed_factor

    return max(0.5, min(25.0, cov))


def cycle_to_cycle_variation(
    cov_imep: float,
    mean_imep: float,
) -> tuple[float, float]:
    """Generate a randomized IMEP for one cycle based on COV.

    Uses normal distribution with std dev = COV/100 * mean.

    Args:
        cov_imep: COV of IMEP [%].
        mean_imep: Mean IMEP [Pa].

    Returns:
        Tuple of (actual_imep, sigma_imep).
    """
    sigma = cov_imep / 100.0 * mean_imep
    actual = random.gauss(mean_imep, sigma)
    # Prevent negative IMEP
    actual = max(0.1 * mean_imep, actual)
    return actual, sigma


def flame_development_angle(
    turbulence_intensity: float = 1.0,
    integral_length_scale: float = 0.005,
    laminar_flame_speed: float = 0.4,
    mean_flame_radius: float = 0.02,
) -> float:
    """Flame development angle (0-10% mass burned).

    KB Eq 9.51:
        Δθ_d = C · (l_I/u')^{1/3} · (l_M/S_L)^{2/3}

    Args:
        turbulence_intensity: Turbulence intensity u' [m/s].
        integral_length_scale: Integral length scale l_I [m].
        laminar_flame_speed: Laminar flame speed S_L [m/s].
        mean_flame_radius: Mean flame radius l_M [m].

    Returns:
        Flame development angle Δθ_d [crank-angle degrees].
    """
    if turbulence_intensity <= 0 or laminar_flame_speed <= 0:
        return 30.0  # default

    C = 1.0  # calibration constant

    ratio_turb = integral_length_scale / turbulence_intensity
    ratio_flame = mean_flame_radius / laminar_flame_speed

    delta_theta_d = C * (ratio_turb ** (1.0 / 3.0)) * (ratio_flame ** (2.0 / 3.0))

    # Convert from normalized time to crank angle degrees
    # Approximate: at 3000 RPM, 1 ms ≈ 18°
    delta_theta_d *= 1000.0  # rough scaling

    return max(5.0, min(60.0, delta_theta_d))
