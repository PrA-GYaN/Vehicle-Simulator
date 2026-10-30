"""In-cylinder heat transfer models — Woschni and Annand correlations.

All equations from ENGINE_KB.md (Heywood 1988) Part X.

Key equations:
    Woschni (KB Eq 12.16-12.19):
        h_c = 3.26 · B^(-0.2) · p[kPa]^(0.8) · T^(-0.55) · w^(0.8)
        w = C_1·S_p + C_2·(V_d·T_r)/(p_r·V_r)·(p - p_motored)

    Annand (KB Eq 12.14):
        h_c = a · (k/B) · Re^b · (p - p_motored)^c

    Q̇_w = h_c · A_s · (T_gas - T_wall)
"""

from __future__ import annotations
import math

from .gas_properties import R_AIR

# Woschni gas-velocity constants (KB Eq 12.18)
C1_GAS_EXCHANGE = 6.18
C1_COMPRESSION = 2.28
C2_COMBUSTION = 3.24e-3


def woschni_heat_transfer_coeff(
    bore: float,
    pressure: float,
    temperature: float,
    mean_piston_speed: float,
    cylinder_volume: float,
    clearance_volume: float,
    reference_pressure: float,
    reference_temp: float,
    motoring_pressure: float,
    combustion_occurred: bool = False,
    reference_volume: float | None = None,
    displaced_volume: float | None = None,
    gas_exchange: bool = False,
) -> float:
    """Woschni correlation for in-cylinder heat transfer coefficient.

    KB Eq 12.16-12.19:
        h_c = 3.26 · B^(-0.2) · p[kPa]^(0.8) · T^(-0.55) · w^(0.8)

    Gas velocity w (KB Eq 12.18):
        w = C_1·S_p + C_2·(V_d·T_r)/(p_r·V_r)·(p − p_m)

    with the constants chosen per phase of the cycle:

        gas exchange             C_1 = 6.18   C_2 = 0
        compression              C_1 = 2.28   C_2 = 0
        combustion + expansion   C_1 = 2.28   C_2 = 3.24e-3

    p_r, T_r, V_r are the working-fluid state at the *inlet valve closing or
    start of combustion* reference. That choice matters: the combustion term
    is dimensionally K·(1/K) → m/s, so using low-pressure intake conditions
    with clearance volume instead of the start-of-combustion state inflates
    (p − p_m)/p_r by ~an order of magnitude and hands w values of several
    hundred m/s where the KB expects ~10 m/s, comparable to mean piston speed.

    Args:
        bore: Cylinder bore [m].
        pressure: Cylinder pressure [Pa].
        temperature: Mean gas temperature [K].
        mean_piston_speed: Mean piston speed [m/s].
        cylinder_volume: Current cylinder volume [m^3].
        clearance_volume: Clearance volume [m^3].
        reference_pressure: p_r, reference-state pressure [Pa] (IVC or start
            of combustion).
        reference_temp: T_r, reference-state temperature [K].
        motoring_pressure: Motoring pressure p_m at the same crank angle [Pa].
        combustion_occurred: True if combustion has started.
        reference_volume: V_r, reference-state volume [m^3]. Defaults to
            ``cylinder_volume`` (start-of-combustion reference).
        displaced_volume: V_d, the engine's *displaced* volume per cylinder
            [m^3] — a constant, not the instantaneous swept volume (which is
            zero at TDC and would switch the combustion term off exactly where
            it matters). If omitted it falls back to ``cylinder_volume −
            clearance_volume``, the old instantaneous value; every caller in
            this package passes the real figure.
        gas_exchange: True while a valve is open (selects C_1 = 6.18).

    Returns:
        Heat transfer coefficient h_c [W/(m^2·K)].
    """
    if bore <= 0 or pressure <= 0 or temperature <= 0:
        return 0.0
    if reference_volume is None:
        reference_volume = cylinder_volume
    if displaced_volume is None:
        displaced_volume = cylinder_volume - clearance_volume

    # Woschni constants (KB Eq 12.18): C₁ is per stroke phase, and C₂ exists
    # only during combustion and expansion.
    if gas_exchange and not combustion_occurred:
        C1 = C1_GAS_EXCHANGE
    else:
        C1 = C1_COMPRESSION
    C2_active = C2_COMBUSTION if combustion_occurred else 0.0

    combustion_term = 0.0
    if (C2_active > 0.0 and reference_pressure > 0
            and reference_volume > 0 and displaced_volume > 0):
        combustion_term = (C2_active * (displaced_volume * reference_temp)
                           / (reference_pressure * reference_volume)
                           * max(0.0, pressure - motoring_pressure))

    w = C1 * mean_piston_speed + combustion_term

    # Woschni correlation (KB Eq 12.19). The constant is calibrated with
    # pressure in kPa, so p must be converted to kPa and not to bar: bar
    # under-predicts h_c by (1000/100)^0.8 ≈ 40x, which suppresses wall heat
    # loss enough to push indicated thermal efficiency above the air-standard
    # limit. The temperature exponent is -0.55 (Eq 12.19; Eq 12.17 gives
    # 0.75 − 1.62·0.8 = −0.546).
    p_kpa = pressure / 1e3
    T_K = max(temperature, 300.0)

    return (3.26
            * bore ** (-0.2)
            * p_kpa ** 0.8
            * T_K ** (-0.55)
            * w ** 0.8)


def annand_heat_transfer_coeff(
    bore: float,
    pressure: float,
    temperature: float,
    mean_piston_speed: float,
    gas_viscosity: float = 2.5e-5,
    gas_conductivity: float = 0.06,
    reynolds_exponent: float = 0.8,
) -> float:
    """Annand correlation for in-cylinder heat transfer coefficient.

    KB Eq 12.14:
        h_c = a · (k/B) · Re^b

    where Re = ρ·S_p·B / μ

    Args:
        bore: Cylinder bore [m].
        pressure: Cylinder pressure [Pa].
        temperature: Mean gas temperature [K].
        mean_piston_speed: Mean piston speed [m/s].
        gas_viscosity: Dynamic viscosity of gas [Pa·s].
        gas_conductivity: Thermal conductivity of gas [W/(m·K)].
        reynolds_exponent: Reynolds exponent b (typically 0.8).

    Returns:
        Heat transfer coefficient h_c [W/(m^2·K)].
    """
    if bore <= 0 or temperature <= 0:
        return 0.0

    # Gas density from ideal gas
    rho = pressure / (R_AIR * temperature) if temperature > 0 else 0.0

    # Reynolds number based on mean piston speed
    Re = rho * mean_piston_speed * bore / gas_viscosity if gas_viscosity > 0 else 0.0

    # Annand constant a (typically 0.35-0.8 for SI engines)
    a = 0.52

    h_c = a * (gas_conductivity / bore) * Re ** reynolds_exponent

    return h_c


def heat_transfer_rate(
    h_c: float,
    surface_area: float,
    gas_temp: float,
    wall_temp: float,
) -> float:
    """Convective heat transfer rate to walls.

    Q̇_w = h_c · A_s · (T_gas - T_wall)

    Args:
        h_c: Heat transfer coefficient [W/(m^2·K)].
        surface_area: Chamber surface area [m^2].
        gas_temp: Mean gas temperature [K].
        wall_temp: Mean wall temperature [K].

    Returns:
        Heat transfer rate INTO walls [W] (positive = heat loss from gas).
    """
    return h_c * surface_area * (gas_temp - wall_temp)


def chamber_surface_area(bore: float, stroke: float,
                         cylinder_volume: float,
                         num_valves: int = 4,
                         clearance_volume: float | None = None) -> float:
    """Combustion chamber surface area at the current volume [m^2].

    Disc chamber (KB Fig 12-1): flat head, flat piston crown and the liner
    exposed above the piston. With the real clearance volume given, the gas
    height is exactly V/A_p (clearance included), so

        A = 2·A_p + π·B·V/A_p = π·B²/2 + 4·V/B

    which is ~0.6·π·B² at TDC and ~1.5·π·B² at BDC for a square engine. A
    wall area fixed at its BDC value overstates the loss at TDC, where the
    heat flux peaks, by a factor of about two and a half.

    Args:
        bore: Cylinder bore [m].
        stroke: Stroke [m] (only used by the legacy estimate below).
        cylinder_volume: Current cylinder volume [m^3].
        num_valves: Unused; kept for call compatibility.
        clearance_volume: Clearance volume [m^3]. If omitted, the older
            estimate (clearance = V_d/10, no liner above the crown at TDC)
            is used.

    Returns:
        Surface area [m^2].
    """
    B = bore
    A_head = math.pi / 4.0 * B * B
    if clearance_volume is not None:
        return 2.0 * A_head + math.pi * B * max(cylinder_volume, clearance_volume) / A_head

    S = stroke
    V_d = A_head * S
    Vc = V_d / 10.0  # rough clearance volume estimate
    if cylinder_volume > Vc:
        piston_pos = (cylinder_volume - Vc) / A_head
    else:
        piston_pos = 0.0
    piston_pos = max(0.0, min(S, piston_pos))
    return math.pi * B * piston_pos + 2.0 * A_head
