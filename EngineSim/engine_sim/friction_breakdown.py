"""Component-level friction breakdown — Bishop-style correlations.

All equations from ENGINE_KB.md (Heywood 1988) Part XI.

Key equations:
    Piston assembly friction (KB Eq 13.18-13.19):
        fpist_mep ∝ piston_speed, ring_tension, gas_pressure

    Bearing friction (KB Eq 13.20-13.24):
        fbear_mep ∝ (μ·N·D²·L) / (clearance·load)

    Valve train friction (KB Eq 13.25):
        fvalve_mep ∝ number_of_valves, cam_profile

    Valve-pumping mep (KB Eq 13.13):
        mep(valve pump) = 8.96·(imep_c/1124)^0.5·(N/1000)^1.7·(2.98/F)^1.28
"""

from __future__ import annotations
import math


def piston_assembly_friction(
    bore: float,
    stroke: float,
    mean_piston_speed: float,
    num_rings: int = 3,
    ring_tension_kpa: float = 14.0,
    peak_cylinder_pressure_bar: float = 50.0,
) -> float:
    """Piston assembly friction MEP (piston + rings + pin).

    KB Eq 13.18-13.19, §13.5.2:
        Piston assembly is the dominant friction source (~50% of total).
        Friction depends on:
        - Ring tension (radial force against wall)
        - Gas pressure behind rings (increases radial force)
        - Piston speed (hydrodynamic vs boundary regimes)
        - Lubricant viscosity

    Approximate correlation:
        fmep_piston = K_tension + K_gas·p_peak + K_speed·S_p

    Args:
        bore: Cylinder bore [m].
        stroke: Stroke [m].
        mean_piston_speed: Mean piston speed [m/s].
        num_rings: Number of rings (typically 3).
        ring_tension_kpa: Ring tension contribution to MEP [kPa].
        peak_cylinder_pressure_bar: Peak cylinder pressure [bar].

    Returns:
        Piston friction MEP [Pa].
    """
    # Ring tension contribution: each compression ring ~7kPa, oil ring ~14kPa
    # KB §13.5.2: "each compression ring contributes about 7kPa mep"
    # "oil rings contribute about TWICE the friction of each compression ring"
    f_tension = ring_tension_kpa * 1000.0  # convert to Pa

    # Gas loading contribution: increases with peak pressure
    # KB §13.5.2: "gas pressure behind compression rings increases radial force"
    f_gas = 0.02 * peak_cylinder_pressure_bar * 1e5  # ~2% of peak p

    # Speed contribution: hydrodynamic component ∝ speed
    f_speed = 5000.0 * mean_piston_speed  # empirical coefficient

    return f_tension + f_gas + f_speed


def bearing_friction(
    shaft_speed_rps: float,
    bearing_diameter: float,
    bearing_length: float,
    oil_viscosity: float = 0.01,
    bearing_load_n: float = 5000.0,
    clearance_ratio: float = 0.001,
) -> float:
    """Crankshaft bearing friction MEP.

    KB Eq 13.20-13.24, §13.5.3:
        Hydrodynamic journal bearing friction.
        Sommerfeld number S = (μ·N·D·L)/(W·c²/D²)

    For full journal bearing in hydrodynamic regime:
        f = 2π²·(μ·N·D·L)/(W·c) · (D/c)

    Args:
        shaft_speed_rps: Crankshaft speed [rev/s].
        bearing_diameter: Bearing diameter [m].
        bearing_length: Bearing width [m].
        oil_viscosity: Dynamic viscosity of oil [Pa·s].
        bearing_load_n: Bearing load [N].
        clearance_ratio: Relative clearance c/D.

    Returns:
        Bearing friction MEP [Pa].
    """
    if shaft_speed_rps <= 0 or bearing_diameter <= 0:
        return 0.0

    c = clearance_ratio * bearing_diameter  # absolute clearance

    # Petroff's equation for unloaded bearing (KB Eq 13.20)
    # T_friction = 2π²·μ·N·D³·L / c
    T_petroff = (2.0 * math.pi * math.pi * oil_viscosity * shaft_speed_rps
                 * bearing_diameter ** 3 * bearing_length / c)

    # Load correction factor (increases with load)
    # Sommerfeld number
    S = (oil_viscosity * shaft_speed_rps * bearing_diameter * bearing_length
         / (bearing_load_n * clearance_ratio ** 2)) if bearing_load_n > 0 else 10.0
    load_factor = 1.0 + 0.5 / max(S, 0.1)

    # Convert to MEP: T_friction / (π/4 · B² · S)
    # Use bearing diameter as scale
    A_ref = math.pi / 4.0 * bearing_diameter ** 2
    fmep_bearing = T_petroff * load_factor / A_ref

    return fmep_bearing


def valvetrain_friction(
    shaft_speed_rps: float,
    num_valves: int = 4,
    cam_type: str = "overhead",
) -> float:
    """Valvetrain friction MEP.

    KB Eq 13.25, §13.5.4:
        ~25% of total rubbing friction in typical SI engine.
        Increases with speed and number of valves.

    Args:
        shaft_speed_rps: Crankshaft speed [rev/s].
        num_valves: Number of valves per cylinder.
        cam_type: "overhead" or "pushrod".

    Returns:
        Valvetrain friction MEP [Pa].
    """
    # Base: ~25 kPa at 3000 RPM for 4-valve OHV
    base_mep = 25000.0  # Pa

    # Speed scaling
    speed_factor = shaft_speed_rps / 50.0  # normalize to ~3000 RPM

    # Valve count scaling
    valve_factor = num_valves / 4.0

    # Cam type: pushrod has higher friction
    type_factor = 1.3 if cam_type == "pushrod" else 1.0

    return base_mep * speed_factor * valve_factor * type_factor


def valve_pumping_mep(
    imep_c_bar: float,
    speed_rpm: float,
    F: float = 0.02,
    is_diesel: bool = False,
) -> float:
    """Valve-pumping friction MEP.

    KB Eq 13.13:
        mep(valve pump) = 8.96·(imep_c/1124)^0.5·(N/1000)^1.7·(2.98/F)^1.28

    where F = n_iv·n_c·D_iv²/V_d [m⁻¹]

    Args:
        imep_c_bar: Indicated MEP from combustion [kPa].
        speed_rpm: Engine speed [RPM].
        F: Valve flow parameter [m⁻¹].
        is_diesel: True for diesel (uses different imep_c reference).

    Returns:
        Valve-pumping MEP [Pa].
    """
    if F <= 0:
        F = 0.02  # default

    imep_ref = 1124.0 if is_diesel else 1124.0

    # Clamp to non-negative before the fractional powers below — a negative
    # base with a fractional exponent silently returns a complex number in
    # Python (no exception), which then corrupts everything downstream.
    imep_ratio = max(0.0, imep_c_bar / imep_ref)
    speed_ratio = max(0.0, speed_rpm / 1000.0)

    mep = (8.96 * imep_ratio ** 0.5
           * speed_ratio ** 1.7
           * (2.98 / F) ** 1.28)

    return mep * 1000.0  # convert from kPa to Pa


def total_friction_breakdown(
    bore: float,
    stroke: float,
    mean_piston_speed: float,
    shaft_speed_rps: float,
    peak_pressure_bar: float,
    imep_c_bar: float,
    speed_rpm: float,
    num_cylinders: int = 4,
    num_valves_per_cyl: int = 4,
) -> dict:
    """Complete Bishop-style friction breakdown.

    KB §13.5, Fig 13-14:
        Piston assembly: ~50%
        Valve train: ~25%
        Crankshaft bearings: ~10%
        Accessories: ~15%

    Returns dict with component MEPs and torques.
    """
    bearing_d = bore * 0.6  # approximate bearing diameter
    bearing_l = bore * 0.3

    f_piston = piston_assembly_friction(
        bore, stroke, mean_piston_speed,
        peak_cylinder_pressure_bar=peak_pressure_bar,
    )

    f_bearing = bearing_friction(
        shaft_speed_rps, bearing_d, bearing_l,
    )

    f_valve = valvetrain_friction(
        shaft_speed_rps, num_valves_per_cyl,
    )

    f_pump = valve_pumping_mep(imep_c_bar, speed_rpm)

    f_accessory = 8000.0 * (1.0 + shaft_speed_rps / 50.0)

    total_mep = f_piston + f_bearing + f_valve + f_pump + f_accessory

    return {
        "piston_mep": f_piston,
        "bearing_mep": f_bearing,
        "valvetrain_mep": f_valve,
        "pumping_mep": f_pump,
        "accessory_mep": f_accessory,
        "total_mep": total_mep,
    }
