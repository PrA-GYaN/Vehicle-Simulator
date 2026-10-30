"""Exhaust backpressure — a lumped plenum fed by the cylinders, drained by the tail pipe.

Why this exists: the exhaust valve does not see ambient pressure. It sees a
plenum that fills with the pulse of gas arriving from the cylinder and drains
through the tail pipe at a rate set by the pipe's own area. Model that and three
effects appear for free instead of needing special cases:

  * Exhaust backpressure costs pumping work, so it shows up in torque rather
    than as a correction factor.
  * Scavenging: late exhaust closing can trap a charge of burned gas instead of
    drawing in fresh air, so volumetric efficiency stops being a constant.
  * Pulse interaction: cylinders feeding the same plenum at once see a higher
    pressure than one feeding it alone, which is what makes cross-plane
    over-run and scavenging uneven.

State is mass and internal energy in a rigid, adiabatic volume:

    dm/dt   = ṁ_cyl − ṁ_tail
    dU/dt   = ṁ_cyl·h(T_cyl) − ṁ_tail·h(T_tail)
    p       = R·T·m/V

with ṁ_tail the restricted orifice flow to ambient (KB Eq 7.18-7.20, area =
tail bore, so a small-diameter or long pipe raises backpressure by itself).
"""

from __future__ import annotations

import math

from .thermodynamics import CV_AIR, GAMMA, R_AIR
from .valve_flow import valve_mass_flow

# The plenum is not allowed to run away to zero mass (the tail orifice would then
# admit an unbounded slug of ambient air). Floor it at a fraction of a gram.
MIN_PLENUM_MASS_KG = 1.0e-3


class ExhaustSystem:
    """Lumped exhaust plenum with a restricted tail-pipe orifice to ambient.

    Attributes:
        pressure: Plenum pressure [Pa] — what the exhaust valves see.
        temperature: Plenum gas temperature [K].
        mass: Gas mass currently in the plenum [kg].
        tail_mass_flow: Signed flow leaving the plenum through the tail [kg/s]
            (negative when ambient air is being drawn backwards into it).
    """

    def __init__(self, plenum_volume_m3: float, tail_dia_m: float,
                 discharge_coeff: float = 0.70,
                 ambient_pressure_pa: float = 101325.0,
                 initial_temperature_k: float = 700.0,
                 turbine_area_m2: float = 0.0,
                 wastegate_area_m2: float = 0.0,
                 post_volume_m3: float = 8.0e-3,
                 turbine_efficiency: float = 0.70):
        self.volume_m3 = max(float(plenum_volume_m3), 1.0e-4)
        self.tail_area_m2 = math.pi / 4.0 * tail_dia_m ** 2
        self.discharge_coeff = discharge_coeff
        self.ambient_pressure_pa = ambient_pressure_pa
        self.ambient_temperature_k = 293.0
        self.temperature = float(initial_temperature_k)
        self.pressure = float(ambient_pressure_pa)
        self.mass = self.pressure * self.volume_m3 / (R_AIR * self.temperature)
        self.tail_mass_flow = 0.0

        # Optional turbine stage: plenum → (turbine ‖ wastegate) → post-turbine volume
        # → tail pipe → ambient. The plenum pressure the exhaust valves see is then set
        # by how much flow the turbine (a restriction that also extracts power) and the
        # wastegate can pass, so boost and backpressure are one coupled system.
        self.has_turbine = turbine_area_m2 > 0.0
        self.turbine_area_m2 = turbine_area_m2
        self.wastegate_area_m2 = wastegate_area_m2
        self.wastegate_position = 0.0          # 0 = closed, 1 = fully open
        self.turbine_efficiency = turbine_efficiency
        self.post_volume_m3 = max(float(post_volume_m3), 1.0e-4)
        self.post_temperature = float(initial_temperature_k)
        self.post_pressure = float(ambient_pressure_pa)
        self.post_mass = self.post_pressure * self.post_volume_m3 / (R_AIR * self.post_temperature)
        self.turbine_mass_flow = 0.0           # through the turbine wheel [kg/s]
        self.wastegate_mass_flow = 0.0         # around it [kg/s]
        self.turbine_power_w = 0.0             # shaft power the wheel produced [W]

    @property
    def density(self) -> float:
        """Plenum gas density [kg/m^3]."""
        return self.pressure / (R_AIR * self.temperature)

    def update(self, mdot_from_cylinders: float, t_cylinders_k: float,
               dt: float) -> None:
        """Advance the plenum by one time step.

        Args:
            mdot_from_cylinders: Net mass arriving from the exhaust valves [kg/s]
                (positive into the plenum).
            t_cylinders_k: Temperature of that arriving gas [K].
            dt: Time step [s].
        """
        if dt <= 0.0:
            return
        if self.has_turbine:
            self._update_with_turbine(mdot_from_cylinders, t_cylinders_k, dt)
            return

        # Tail orifice: gas always flows toward the lower-pressure side.
        if self.pressure >= self.ambient_pressure_pa:
            mdot_tail = valve_mass_flow(
                self.pressure, self.ambient_pressure_pa, self.temperature,
                self.tail_area_m2, self.discharge_coeff)
            tail_h = GAMMA * CV_AIR * self.temperature
        else:
            mdot_tail = -valve_mass_flow(
                self.ambient_pressure_pa, self.pressure, self.ambient_temperature_k,
                self.tail_area_m2, self.discharge_coeff)
            tail_h = GAMMA * CV_AIR * self.ambient_temperature_k
        self.tail_mass_flow = mdot_tail

        # Never let the tail drain more than is there (explicit steps overshoot
        # near equilibrium and would otherwise drive mass negative).
        available = self.mass - mdot_tail * dt
        if available < MIN_PLENUM_MASS_KG:
            mdot_tail = max(0.0, (self.mass - MIN_PLENUM_MASS_KG) / dt)
            available = self.mass - mdot_tail * dt

        # Gas arriving from a cylinder brings h(T_cylinder); gas drawn back into
        # a cylinder (reversion, negative flow) leaves at the plenum's own
        # enthalpy, not the cylinder's.
        t_in = t_cylinders_k if mdot_from_cylinders >= 0.0 else self.temperature
        energy = self.mass * CV_AIR * self.temperature
        energy += (mdot_from_cylinders * GAMMA * CV_AIR * t_in
                   - mdot_tail * tail_h) * dt

        self.mass = max(available + mdot_from_cylinders * dt, MIN_PLENUM_MASS_KG)
        self.temperature = max(energy / (self.mass * CV_AIR), 50.0)
        self.pressure = R_AIR * self.temperature * self.mass / self.volume_m3

    def _update_with_turbine(self, mdot_from_cylinders: float, t_cylinders_k: float,
                             dt: float) -> None:
        """Plenum → (turbine ‖ wastegate) → post-turbine volume → tail → ambient.

        The turbine is an orifice with the stage's effective area (KB Eq 7.18-7.20)
        that also produces shaft power, Ẇ = η·ṁ·c_p·T₃·[1 − (p₄/p₃)^((γ−1)/γ)]
        (KB Eq 6.44-6.48); the wastegate is a second orifice in parallel that
        produces none. The power is taken out of the gas, so the post-turbine
        stream leaves cooler by Ẇ/(ṁ·c_p). Neither passage runs backwards.
        """
        cp = GAMMA * CV_AIR
        p3, t3, p4, t4 = self.pressure, self.temperature, self.post_pressure, self.post_temperature
        m_t = m_w = w_t = 0.0
        if p3 > p4:
            m_t = valve_mass_flow(p3, p4, t3, self.turbine_area_m2, 0.8)
            wg_area = self.wastegate_area_m2 * self.wastegate_position
            if wg_area > 0.0:
                m_w = valve_mass_flow(p3, p4, t3, wg_area, 0.7)
            w_t = (m_t * cp * t3 * self.turbine_efficiency
                   * (1.0 - (p4 / p3) ** ((GAMMA - 1.0) / GAMMA)))
        # The plenum cannot give away more than it holds.
        out = (m_t + m_w) * dt
        room = self.mass - MIN_PLENUM_MASS_KG
        if out > room > 0.0:
            scale = room / out
            m_t, m_w, w_t = m_t * scale, m_w * scale, w_t * scale
        elif room <= 0.0:
            m_t = m_w = w_t = 0.0
        self.turbine_mass_flow, self.wastegate_mass_flow, self.turbine_power_w = m_t, m_w, w_t

        # Tail pipe from the post-turbine volume.
        if p4 >= self.ambient_pressure_pa:
            m_tail = valve_mass_flow(p4, self.ambient_pressure_pa, t4, self.tail_area_m2,
                                     self.discharge_coeff)
            h_tail = cp * t4
        else:
            m_tail = -valve_mass_flow(self.ambient_pressure_pa, p4, self.ambient_temperature_k,
                                      self.tail_area_m2, self.discharge_coeff)
            h_tail = cp * self.ambient_temperature_k
        room4 = self.post_mass + (m_t + m_w) * dt - MIN_PLENUM_MASS_KG
        if m_tail * dt > room4 > 0.0:
            m_tail = room4 / dt
        elif room4 <= 0.0:
            m_tail = max(0.0, m_tail)
        self.tail_mass_flow = m_tail

        # Plenum energy and mass.
        t_in = t_cylinders_k if mdot_from_cylinders >= 0.0 else t3
        e3 = self.mass * CV_AIR * t3 + (mdot_from_cylinders * cp * t_in - (m_t + m_w) * cp * t3) * dt
        self.mass = max(self.mass + (mdot_from_cylinders - m_t - m_w) * dt, MIN_PLENUM_MASS_KG)
        self.temperature = max(e3 / (self.mass * CV_AIR), 50.0)
        self.pressure = R_AIR * self.temperature * self.mass / self.volume_m3

        # Post-turbine volume: the stream arrives having given up the turbine's power.
        e4 = self.post_mass * CV_AIR * t4 + (((m_t + m_w) * cp * t3 - w_t) - m_tail * h_tail) * dt
        self.post_mass = max(self.post_mass + (m_t + m_w - m_tail) * dt, MIN_PLENUM_MASS_KG)
        self.post_temperature = max(e4 / (self.post_mass * CV_AIR), 50.0)
        self.post_pressure = R_AIR * self.post_temperature * self.post_mass / self.post_volume_m3
