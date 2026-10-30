"""Piston kinematics — crank-slider mechanism equations.

All equations from ENGINE_KB.md (Heywood 1988).

Key equations:
    x(θ) = r·cos(θ) + l·√(1 - r²·sin²(θ)) - (l + r)    (KB Eq 2.5)
    V(θ) = V_c + (π/4)·B²·x(θ)                           (KB Eq 2.6)
    dx/dθ = -r·sin(θ) - r²·sin(θ)·cos(θ) / √(l²-r²·sin²(θ))  (KB Eq 2.11)
"""

from __future__ import annotations
import math


def piston_displacement(theta: float, r: float, l: float) -> float:
    """Piston displacement from TDC.

    KB Eq 2.5:
        x(θ) = (l + r) − r·cos(θ) − l·√(1 − (r/l)²·sin²(θ))

    At TDC (θ=0): x = 0
    At BDC (θ=π): x = 2r = S (stroke)

    Args:
        theta: Crank angle from TDC [rad].
        r: Crank radius = S/2 [m].
        l: Connecting rod length [m].

    Returns:
        Piston displacement from TDC [m], positive as the piston descends
        (so V = V_c + (π/4)·B²·x grows through the intake stroke).
    """
    ratio = r / l
    sin_t = math.sin(theta)
    cos_t = math.cos(theta)
    disc = math.sqrt(1.0 - (ratio * sin_t) ** 2)
    return (l + r) - r * cos_t - l * disc


def piston_velocity(theta: float, r: float, l: float, omega: float) -> float:
    """Piston velocity.

    v = dx/dt = (dx/dθ)·ω

KB Eq 2.11:
        dx/dθ = r·sin(θ) + r²·sin(θ)·cos(θ) / √(l²-r²·sin²(θ))

    Args:
        theta: Crank angle from TDC [rad].
        r: Crank radius = S/2 [m].
        l: Connecting rod length [m].
        omega: Crankshaft angular velocity [rad/s].

    Returns:
        Piston velocity [m/s] (positive = descending, away from TDC).
    """
    ratio = r / l
    sin_t = math.sin(theta)
    cos_t = math.cos(theta)
    disc = math.sqrt(1.0 - (ratio * sin_t) ** 2)

    dx_dtheta = r * sin_t + (r * ratio * sin_t * cos_t) / disc
    return dx_dtheta * omega


def piston_acceleration(theta: float, r: float, l: float, omega: float) -> float:
    """Piston acceleration.

    a = d²x/dt² = (d²x/dθ²)·ω² + (dx/dθ)·(dω/dt)

    Simplified for constant ω (dω/dt = 0):
        a ≈ (d²x/dθ²)·ω²

    Args:
        theta: Crank angle from TDC [rad].
        r: Crank radius [m].
        l: Connecting rod length [m].
        omega: Crankshaft angular velocity [rad/s].

    Returns:
        Piston acceleration [m/s^2].
    """
    ratio = r / l
    sin_t = math.sin(theta)
    cos_t = math.cos(theta)
    disc = math.sqrt(1.0 - (ratio * sin_t) ** 2)

    # d²x/dθ² ≈ r·cos(θ) + r²·cos(2θ)/l  (approximate)
    d2x_dtheta2 = r * cos_t + (r * ratio * cos_t * cos_t) / disc
    # More accurate second derivative
    d2x_dtheta2 = r * cos_t * (1.0 + ratio / disc)

    return d2x_dtheta2 * omega * omega


def cylinder_volume(theta: float, Vc: float, rc: float, R: float) -> float:
    """Cylinder volume at given crank angle.

    KB Eq 2.6:
        V(θ) = V_c + (π/4)·B²·x(θ)

    Using normalized form:
        V(θ) = V_c · [1 + (rc-1) · S_p(θ)]

    where S_p(θ) is the normalized piston position [0,1].

    Args:
        theta: Crank angle from TDC [rad].
        Vc: Clearance volume [m^3].
        rc: Compression ratio (V_bdc / V_c).
        R: Rod ratio (l/r, typically 3-4).

    Returns:
        Cylinder volume [m^3].
    """
    sin_t = math.sin(theta)
    cos_t = math.cos(theta)
    disc = math.sqrt(1.0 - sin_t * sin_t / (R * R))

    # Normalized piston position: 0 at TDC, 1 at BDC. From KB Eq 2.5 with
    # l = R·r:  x/(2r) = ½·(1 − cos θ + R·(1 − √(1 − sin²θ/R²))).
    # (An earlier revision had 1/R in place of R, which understated the
    # rod-obliquity term by R² ≈ 11x and put the mid-stroke volume ~12% off.)
    S_p = 0.5 * (1.0 - cos_t + R * (1.0 - disc))

    return Vc * (1.0 + (rc - 1.0) * S_p)


def cylinder_volume_geometry(theta: float, bore: float, stroke: float,
                             rod_length: float, clearance_volume: float) -> float:
    """Cylinder volume from the raw geometry, in the same form as KB Eq 2.6.

        V(θ) = V_c + (π/4)·B²·x(θ)

    This is the geometry-explicit twin of :func:`cylinder_volume`: both are
    exact crank-slider results, but this one takes bore, stroke and rod length
    directly, so changing any of them changes the pressure-volume path the
    torque model integrates over (KB Eq 2.5, Eq 2.6).

    Args:
        theta: Crank angle from TDC [rad].
        bore: Cylinder bore [m].
        stroke: Stroke S [m] (crank radius r = S/2).
        rod_length: Connecting-rod length l [m].
        clearance_volume: Clearance volume V_c [m^3].

    Returns:
        Cylinder volume [m^3].
    """
    area = math.pi / 4.0 * bore * bore
    return clearance_volume + area * piston_displacement(theta, stroke / 2.0, rod_length)


def cylinder_volume_rate(theta: float, bore: float, stroke: float,
                         rod_length: float) -> float:
    """dV/dθ from the crank-slider geometry [m^3/rad].

        dV/dθ = (π/4)·B² · dx/dθ

    This is the quantity that converts cylinder pressure into crank torque
    (T = p·dV/dθ, KB Eq 2.11). Positive while the piston descends, so gas
    pressure acting on the crown produces positive torque on the expansion
    stroke and negative torque on compression.

    Args:
        theta: Crank angle from TDC [rad].
        bore: Cylinder bore [m].
        stroke: Stroke S [m].
        rod_length: Connecting-rod length l [m].

    Returns:
        dV/dθ [m^3/rad].
    """
    r = stroke / 2.0
    ratio = r / rod_length
    sin_t = math.sin(theta)
    cos_t = math.cos(theta)
    disc = math.sqrt(1.0 - (ratio * sin_t) ** 2)

    dx_dtheta = r * sin_t + (r * ratio * sin_t * cos_t) / disc
    return (math.pi / 4.0 * bore * bore) * dx_dtheta


def cylinder_volume_derivative(theta: float, displacement: float,
                               Vc: float, R: float) -> float:
    """Derivative of cylinder volume with respect to crank angle.

    dV/dθ = V_d · dS_p/dθ

    This is needed for computing torque from cylinder pressure:
        T = p · A_p · ds/dθ

    Args:
        theta: Crank angle from TDC [rad].
        displacement: Total displaced volume [m^3].
        Vc: Clearance volume [m^3].
        R: Rod ratio (l/r).

    Returns:
        dV/dθ [m^3/rad].
    """
    sin_t = math.sin(theta)
    cos_t = math.cos(theta)
    disc = math.sqrt(1.0 - sin_t * sin_t / (R * R))

    # dS_p/dθ = 0.5 * [sin(θ) + sin(θ)·cos(θ) / (R·disc)]
    ds_dtheta = 0.5 * sin_t * (1.0 + cos_t / (R * disc))

    Vd = displacement
    return Vd * ds_dtheta
