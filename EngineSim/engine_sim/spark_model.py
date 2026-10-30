"""Spark timing model — MBT timing and knock assessment.

All equations from ENGINE_KB.md (Heywood 1988) Part VII.

Key equations:
    MBT timing (§9.2.3, §15.3.1):
        θ_MBT ≈ 0.5·Δθ_b before TDC
        where Δθ_b = combustion duration

    Livengood-Wu integral for knock (KB Eq 9.60-9.64):
        ∫(1/τ) dt = 1  →  knock occurs
        where τ = ignition delay ( Arrhenius form )
"""

from __future__ import annotations
import math


def mbt_timing(combustion_duration_deg: float, speed_rpm: float) -> float:
    """Minimum spark advance for best torque (MBT) timing.

    KB §9.2.3: MBT timing typically occurs when 50% mass fraction
    burned is at ~8° ATC, so spark at approximately:
        θ_spark = -(0.5 * Δθ_b - 8°)  before TDC

    For typical SI engines: 10-40° BTC.

    Args:
        combustion_duration_deg: Total combustion duration Δθ [crank-angle degrees].
        speed_rpm: Engine speed [RPM].

    Returns:
        MBT spark timing [degrees before TDC, positive = advanced].
    """
    # Spark fires about halfway through combustion duration before TDC
    # so that 50% burn occurs near 8° ATC
    theta_spark = 0.5 * combustion_duration_deg - 8.0

    # Clamp to practical range
    theta_spark = max(5.0, min(50.0, theta_spark))

    # Speed correction: at higher speed, need more advance
    # because time for combustion decreases
    speed_factor = 1.0 + 0.3 * max(0.0, (speed_rpm - 3000.0) / 4000.0)
    theta_spark *= speed_factor

    return max(5.0, min(50.0, theta_spark))


def combustion_duration(speed_rpm: float, load_fraction: float,
                        equivalence_ratio: float = 1.0,
                        residual_fraction: float = 0.05) -> float:
    """Total combustion duration (10% to 90% mass burned).

    KB §9.2.3, Fig 9-16: Δθ_b depends on turbulence intensity,
    flame speed, and chamber geometry.

    Empirical correlation (Heywood Fig 9-16 fit):
        Δθ_b ≈ 180° / (speed_factor * load_factor)

    Args:
        speed_rpm: Engine speed [RPM].
        load_fraction: Relative load (0-1, 1 = WOT).
        equivalence_ratio: Equivalence ratio φ.
        residual_fraction: Residual gas fraction.

    Returns:
        Combustion duration [crank-angle degrees].
    """
    # Base duration at 3000 RPM, WOT, stoich
    delta_theta_base = 60.0  # degrees (typical for fast-burn SI)

    # Speed effect. Turbulence intensity scales with engine speed, so the flame
    # speed rises almost in proportion and the burn takes *nearly the same crank
    # angle* at any speed (KB Fig 9-16), rising only gently with rpm. A linear
    # speed_rpm/3000 factor — what this had — gives an unphysically fast 27°
    # burn at idle, which puts the MBT spark at ~5° and starves the idle of
    # spark authority.
    speed_factor = (max(speed_rpm, 300.0) / 3000.0) ** 0.5

    # Load effect: lower load → slower burn → more degrees
    load_factor = 0.5 + 0.5 * load_fraction

    # Dilution effect: more residuals → slower burn
    dilution_factor = 1.0 + 3.0 * residual_fraction

    # Equivalence ratio effect:偏离stoich → slower burn
    phi_factor = 1.0 + 2.0 * (equivalence_ratio - 1.0) ** 2

    delta_theta = (delta_theta_base * speed_factor * dilution_factor
                   * phi_factor / max(load_factor, 0.1))

    return max(20.0, min(120.0, delta_theta))


def autoignition_delay(temperature: float, pressure: float,
                       fuel_octane: float = 91.0) -> float:
    """Autoignition (ignition-delay) time via the Douaud-Eyzat correlation.

    KB Eq 9.62:
        τ [ms] = 17.68 · (ON/100)^3.402 · p[atm]⁻¹·⁷ · exp(3800/T)

    This is the standard correlation cited in the KB for knock-onset
    prediction (used inside the Livengood-Wu integral, Eq 9.60/9.64).
    Higher octane number (ON) and higher pressure/temperature all act
    to change delay in the physically correct direction: τ decreases
    as T or p increases, and increases with higher octane number.

    Args:
        temperature: Unburned-gas temperature [K].
        pressure: Unburned-gas pressure [Pa].
        fuel_octane: Octane number (RON/ON), typical 87-98.

    Returns:
        Ignition delay [seconds].
    """
    if temperature <= 0 or pressure <= 0:
        return 100.0  # effectively no knock

    p_atm = max(pressure / 101325.0, 0.01)  # Pa -> atm
    on_term = (max(fuel_octane, 1.0) / 100.0) ** 3.402

    tau_ms = 17.68 * on_term * p_atm ** (-1.7) * math.exp(3800.0 / temperature)

    return max(1.0e-6, tau_ms / 1000.0)  # ms -> s


def livengood_wu_integral(
    temperature: float,
    pressure: float,
    dt: float,
    fuel_octane: float = 91.0,
) -> float:
    """Livengood-Wu integral for autoignition/knock prediction.

    KB Eq 9.60/9.64:
        ∫₀ᵗ (1/τ) dt = 1  →  knock occurs

    where τ is the Douaud-Eyzat ignition delay (KB Eq 9.62), evaluated
    at the current unburned-gas temperature and pressure.

    Args:
        temperature: Unburned-gas temperature [K].
        pressure: Unburned-gas pressure [Pa].
        dt: Timestep [s].
        fuel_octane: Octane number (RON/ON).

    Returns:
        Integral increment for this timestep (sum to 1.0 = knock onset).
    """
    tau = autoignition_delay(temperature, pressure, fuel_octane)
    if tau <= 0:
        return 0.0

    return dt / tau
