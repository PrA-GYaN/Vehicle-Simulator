"""Mass flow through a poppet valve — the gas-exchange primitive.

One valve is a compressible orifice (the same 1-D relation the throttle plate
uses, KB Eq 7.18-7.20), with one difference that matters a lot: the effective
area is not fixed but is set by valve lift. A valve only passes mass while it
is off its seat, and the area it offers is the smaller of two limits:

    A_lift  = π·D_head·L·cos(β_seat)          the annular gap around the head
    A_port  = π·D_port²/4                     the port throat

At small lift the gap limits the flow; once the head is far enough off its seat
the port throat does, which is why flow through a valve stops rising with lift
partway through the event even though the lift keeps climbing.

Direction is signed: flow follows the pressure difference, so the same
function serves the intake valve (manifold → cylinder) and the exhaust valve
(cylinder → exhaust). During overlap both are open and the two pressures decide
which way each one runs — that is what produces reversion and scavenging.

For choked flow (cylinder pressure well below the intake runner, or well above
the exhaust) the mass flow saturates at the throat's critical value and stops
responding to the far-side pressure, exactly as real flow does.
"""

from __future__ import annotations

import math

from .thermodynamics import GAMMA, R_AIR

# Effective flow area below this many metres of lift is treated as shut: below
# ~0.1 mm a real valve's leakage is indistinguishable from zero and letting a
# tiny area into the mass balance only adds numerical noise.
LIFT_DEADBAND_M = 1.0e-4


def valve_flow_area(lift_m: float, head_dia_m: float, port_dia_m: float,
                    seat_angle_deg: float = 45.0) -> float:
    """Effective flow area of an open valve [m^2].

    Args:
        lift_m: Valve lift above the seat [m].
        head_dia_m: Valve head diameter [m].
        port_dia_m: Intake/exhaust port (throat) diameter [m].
        seat_angle_deg: Seat angle β [deg] as in Heywood Fig 6-12: the angle
            of the seat face from the valve-head plane (0° = flat seat, 45°
            and 30° are the usual road-engine values — *not* the included
            cone angle). The gap normal to the seat is L·cos β (KB Eq 6.7), so
            a shallower seat opens wider for the same lift.

    Returns:
        Minimum of the annular lift gap and the port throat area [m^2].
    """
    if lift_m <= LIFT_DEADBAND_M or head_dia_m <= 0.0 or port_dia_m <= 0.0:
        return 0.0
    a_lift = (math.pi * head_dia_m * lift_m
              * math.cos(math.radians(seat_angle_deg)))
    a_port = math.pi / 4.0 * port_dia_m ** 2
    return min(a_lift, a_port)


def valve_mass_flow(p_up: float, p_down: float, T_up: float,
                    area: float, discharge_coeff: float = 0.67,
                    gamma: float = GAMMA) -> float:
    """Signed mass flow through a valve orifice [kg/s].

    Positive when `p_up` > `p_down` (gas flows up → down). Choked when the
    pressure ratio falls below the critical value; subsonic otherwise.

    Args:
        p_up: Upstream (higher-pressure side) pressure [Pa].
        p_down: Downstream pressure [Pa].
        T_up: Upstream gas temperature [K].
        area: Effective flow area [m^2].
        discharge_coeff: Valve discharge coefficient (0.6-0.7 typical).
        gamma: Specific heat ratio of the flowing gas.

    Returns:
        Mass flow rate [kg/s], signed; 0 when the valve is shut or p_up <= p_down.
    """
    if area <= 0.0 or p_up <= p_down or T_up <= 0.0:
        return 0.0

    pr = p_down / p_up
    pr_crit = (2.0 / (gamma + 1.0)) ** (gamma / (gamma - 1.0))

    if pr <= pr_crit:
        # Choked: independent of downstream pressure (KB Eq 7.19).
        return (discharge_coeff * area * p_up
                * math.sqrt(2.0 / (R_AIR * T_up)
                            * gamma / (gamma + 1.0)
                            * (2.0 / (gamma + 1.0)) ** (1.0 / (gamma - 1.0))))

    # Subsonic (KB Eq 7.20). The bracket is analytically >= 0 for pr in
    # (pr_crit, 1) but can go marginally negative at the pr -> 1 boundary from
    # floating-point rounding, hence the clamp.
    bracket = max(0.0, pr ** (2.0 / gamma) - pr ** ((gamma + 1.0) / gamma))
    return (discharge_coeff * area * p_up
            * math.sqrt(2.0 * gamma / (R_AIR * T_up * (gamma - 1.0)) * bracket))


def valve_mass_flow_signed(p_cylinder: float, p_port: float, T_cylinder: float,
                           T_port: float, area: float,
                           discharge_coeff: float = 0.67,
                           gamma: float = GAMMA) -> float:
    """Mass flow through a valve, signed *into the cylinder* [kg/s].

    Positive means gas entering the cylinder (intake valve running), negative
    means gas leaving it (exhaust valve running, or reversion back down the
    intake runner). Which side is "up" is decided per call from the pressure
    difference, so an open intake valve over a cylinder that is still above
    manifold pressure correctly flows backwards.

    Args:
        p_cylinder: Cylinder pressure [Pa].
        p_port: Port/manifold pressure on the other side of the valve [Pa].
        T_cylinder: Cylinder gas temperature [K].
        T_port: Port gas temperature [K].
        area: Effective flow area [m^2].
        discharge_coeff: Valve discharge coefficient.
        gamma: Specific heat ratio of the flowing gas.

    Returns:
        Mass flow rate into the cylinder [kg/s], signed.
    """
    if area <= 0.0:
        return 0.0
    if p_cylinder >= p_port:
        # Cylinder → port (exhaust running, or reversion).
        return -valve_mass_flow(p_cylinder, p_port, T_cylinder, area,
                                discharge_coeff, gamma)
    # Port → cylinder (intake filling).
    return valve_mass_flow(p_port, p_cylinder, T_port, area,
                           discharge_coeff, gamma)