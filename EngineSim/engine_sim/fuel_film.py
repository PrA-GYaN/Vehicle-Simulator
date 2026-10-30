"""Fuel-film wall-wetting model — transient fuel dynamics.

All equations from ENGINE_KB.md (Heywood 1988) Part VI.

Key equations:
    ṁ_{f,p} = x·ṁ_{f,m} − m_{f,p}/τ          (KB Eq 7.25)

where:
    x = fraction of metered fuel entering as liquid film
    τ = film evaporation time constant
    ṁ_{f,m} = metered fuel flow rate
    ṁ_{f,p} = fuel flow entering cylinder (fuel actually burned)
    m_{f,p} = film mass on manifold walls

This model captures the lean spike on rapid throttle opening.
"""

from __future__ import annotations
import math


class FuelFilmState:
    """Dynamic state of the fuel film in the intake manifold."""

    def __init__(self, x: float = 0.3, tau: float = 0.5,
                 film_mass: float = 0.0):
        """
        Args:
            x: Fraction of metered fuel that enters as liquid film [0-1].
               KB §7.6.3: typically 0.2-0.5 for throttle-body injection.
            tau: Film evaporation time constant [s].
                 KB §7.6.3: typically 0.3-1.0 s.
            film_mass: Initial film mass [kg].
        """
        self.x = x
        self.tau = tau
        self.film_mass = film_mass

    def reset(self, film_mass: float = 0.0) -> None:
        self.film_mass = film_mass


def fuel_film_dynamics(
    metered_fuel_rate: float,
    state: FuelFilmState,
    dt: float,
) -> tuple[float, FuelFilmState]:
    """Update fuel-film model and return fuel actually delivered to cylinders.

    KB Eq 7.25:
        dm_{f,p}/dt = x · ṁ_{f,m} − m_{f,p}/τ

    where:
        ṁ_{f,p} = fuel flow into cylinders (what the engine actually gets)
        ṁ_{f,m} = metered fuel flow (what the injector delivers)
        x = fraction going to film
        τ = evaporation time constant

    Args:
        metered_fuel_rate: Fuel flow rate from injector [kg/s].
        state: Current FuelFilmState.
        dt: Timestep [s].

    Returns:
        Tuple of (fuel_to_cylinders [kg/s], updated state).
    """
    x = state.x
    tau = state.tau
    m_film = state.film_mass

    # Fuel entering cylinders = direct flow + film evaporation
    # Direct flow: (1-x) of metered fuel goes directly as vapor
    # Film contribution: m_film / τ evaporates into the air stream
    m_dot_direct = (1.0 - x) * metered_fuel_rate
    m_dot_film = m_film / tau if tau > 0 else 0.0

    m_dot_cylinders = m_dot_direct + m_dot_film

    # Update film mass
    # Film gains: x * metered_fuel_rate
    # Film loses: m_film / τ (evaporation)
    dm_film = (x * metered_fuel_rate - m_film / tau) * dt
    new_film_mass = max(0.0, m_film + dm_film)

    new_state = FuelFilmState(x=x, tau=tau, film_mass=new_film_mass)

    return m_dot_cylinders, new_state


def fuel_film_tau(speed_rpm: float, manifold_pressure_pa: float,
                  intake_temp_k: float = 293.0,
                  fuel_type: str = "gasoline") -> float:
    """Estimate fuel-film evaporation time constant.

    KB §7.6.3: τ depends on manifold conditions.
    Higher temperature → shorter τ (faster evaporation).
    Lower pressure → longer τ (slower evaporation).
    Higher speed → shorter τ (more air flow, better atomization).

    Args:
        speed_rpm: Engine speed [RPM].
        manifold_pressure_pa: Manifold pressure [Pa].
        intake_temp_k: Intake temperature [K].
        fuel_type: "gasoline" or "methanol".

    Returns:
        Time constant τ [s].
    """
    # Base τ at 2000 RPM, 1 atm, 20°C
    tau_base = 0.5  # seconds (gasoline)

    if fuel_type == "methanol":
        tau_base = 0.8  # methanol evaporates slower

    # Temperature effect: higher T → faster evaporation
    temp_factor = 293.0 / max(intake_temp_k, 250.0)

    # Pressure effect: lower pressure → faster evaporation (in vacuum)
    press_factor = (101325.0 / max(manifold_pressure_pa, 10000.0)) ** 0.5

    # Speed effect: higher speed → more turbulence → faster evaporation
    speed_factor = 2000.0 / max(speed_rpm, 500.0)

    return tau_base * temp_factor * press_factor * speed_factor


def fuel_film_x(throttle_pos: float, injection_type: str = "throttle_body") -> float:
    """Estimate fraction of fuel entering as liquid film.

    KB §7.6.3, Fig 7-27:
        At part throttle: most fuel impacts throttle plate → high x
        At WOT: better atomization → lower x
        Port injection: lower x than throttle body injection.

    Args:
        throttle_pos: Throttle position [0-1].
        injection_type: "throttle_body" or "port_injection".

    Returns:
        Film fraction x [0-1].
    """
    if injection_type == "port_injection":
        # Port injection: lower wall-wetting
        x_base = 0.15
        # Less film at high throttle (more air velocity)
        x_throttle = 0.3 * (1.0 - 0.5 * throttle_pos)
    else:
        # Throttle-body injection: more wall-wetting
        x_base = 0.3
        # Significant at part throttle, less at WOT
        x_throttle = 0.5 * (1.0 - 0.7 * throttle_pos)

    return max(0.05, min(0.8, x_base + x_throttle))
