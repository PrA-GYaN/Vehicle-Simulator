"""A dynamic turbocharger for the resolved engine core.

The cycle-averaged path (turbo_model.py) estimates boost from an energy balance on
cycle-mean quantities. Here the three parts are real components with their own
state, coupled through the flows the cylinders actually produce:

    exhaust plenum ──► turbine ‖ wastegate ──► post-turbine volume ──► tail pipe
          ▲                 │ shaft power                                    
          │                 ▼
    cylinders          compressor ──► charge-air volume ──► throttle ──► manifold

* **Turbine and wastegate** are orifices out of the exhaust plenum
  (ExhaustSystem._update_with_turbine): the wheel passes only the flow its effective
  area allows at the pressure ratio across it and produces
  Ẇ_T = η·ṁ·c_p·T₃·[1 − (p₄/p₃)^((γ−1)/γ)] (KB Eq 6.44-6.48), so *exhaust
  backpressure rises as boost is demanded* and falls when the wastegate opens.
* **Compressor.** Speed lines of a standard map: the pressure-ratio capability
  grows as the square of shaft speed, flow at a given ratio falls off as
  √(1 − (Δπ/Δπ_max)²) toward surge, power is Ẇ_C = ṁ·c_p·T₁·(π^((γ−1)/γ) − 1)/η_C
  (KB Eq 6.42). It is a map you can scale (design speed, flow and ratio), not a
  measured one.
* **Shaft.** ½·I·dω²/dt = η_m·Ẇ_T − Ẇ_C, so spool-up takes as long as the inertia and
  the surplus turbine power say — turbo lag emerges instead of being a time constant.
* **Charge-air volume** (intercooler and piping) is integrated like the manifold,
  fed by the compressor and drained by the throttle; a check-valve path lets air in
  around an unspun compressor and a blow-off valve vents it when the throttle shuts.
* **Wastegate** is held by a PI law on the boost error: the integral finds the opening the
  steady point needs and the proportional term damps the overshoot while the shaft is still
  accelerating.

Twin-turbo engines are modelled as one turbo with the flow, area and inertia of the
pair (``turbo_count``): the two wheels turn together at the same speed, so the sum
is exact for symmetric banks.
"""

from __future__ import annotations

import math

from .thermodynamics import CP_AIR, GAMMA, R_AIR

_KAPPA = (GAMMA - 1.0) / GAMMA
_WG_KP = 6.0                       # wastegate opening per unit relative boost error
_WG_KI = 3.0                       # ... and per unit relative error · second


class Turbocharger:
    """Shaft, compressor map, charge-air volume and wastegate control.

    Attributes:
        omega: Shaft speed [rad/s].
        boost_pressure: Charge-air pressure upstream of the throttle [Pa].
        boost_temperature: Charge-air temperature after the intercooler [K].
        wastegate_position: 0 (closed) .. 1 (open), commanded by the boost controller.
    """

    def __init__(self, cfg):
        n = max(1, int(cfg.turbo_count))
        self.count = n
        self.inertia = cfg.turbo_inertia_kgm2 * n
        self.design_omega = cfg.turbo_design_speed_rps * 2.0 * math.pi
        self.design_flow = cfg.turbo_design_flow_kgs * n
        self.design_ratio = cfg.turbo_design_pressure_ratio
        self.eta_c = cfg.turbo_compressor_efficiency
        self.eta_m = cfg.turbo_mechanical_efficiency
        self.volume = cfg.turbo_boost_volume_m3
        self.intercooler = cfg.turbo_intercooler_effectiveness
        self.target_boost = cfg.turbo_target_boost_pa
        self.gain = cfg.turbo_wastegate_gain
        self.bypass_area = 3.0e-3 * n          # check-valve path around an unspun compressor [m²]
        self.bov_area = 6.0e-4 * n             # blow-off valve [m²]
        self.p_ambient = cfg.ambient_pressure_pa
        self.t_ambient = cfg.intake_temperature_k
        self.reset()

    def reset(self) -> None:
        self.omega = 0.0
        self.boost_pressure = self.p_ambient
        self.boost_temperature = self.t_ambient
        self.wastegate_position = 0.0
        self._wg_integral = 0.0
        self.compressor_flow_last = 0.0
        self.shaft_energy_in = 0.0
        self.shaft_energy_out = 0.0

    @property
    def speed_rps(self) -> float:
        return self.omega / (2.0 * math.pi)

    def compressor_flow(self, pressure_ratio: float) -> float:
        """Mass flow the compressor delivers at this ratio and the current shaft speed [kg/s]."""
        x = self.omega / self.design_omega
        if x <= 0.0:
            return 0.0
        head = (self.design_ratio - 1.0) * x * x * 1.15
        r = (pressure_ratio - 1.0) / head
        if r >= 1.0:
            return 0.0
        if r < 0.0:
            r = 0.0
        return self.design_flow * 1.25 * x * math.sqrt(1.0 - r * r)

    def compressor_power(self, mass_flow: float, pressure_ratio: float) -> float:
        """Shaft power the compressor absorbs [W]."""
        if mass_flow <= 0.0 or pressure_ratio <= 1.0:
            return 0.0
        return mass_flow * CP_AIR * self.t_ambient * (pressure_ratio ** _KAPPA - 1.0) / self.eta_c

    def charge_temperature(self, pressure_ratio: float) -> float:
        """Charge-air temperature leaving the intercooler [K]."""
        if pressure_ratio <= 1.0:
            return self.t_ambient
        t_out = self.t_ambient * (1.0 + (pressure_ratio ** _KAPPA - 1.0) / self.eta_c)
        return self.t_ambient + (1.0 - self.intercooler) * (t_out - self.t_ambient)

    def advance_shaft(self, turbine_energy_j: float, compressor_energy_j: float) -> None:
        """Energy balance on the shaft over an interval: ½·I·Δ(ω²) = η_m·E_T − E_C."""
        e = 0.5 * self.inertia * self.omega ** 2 + self.eta_m * turbine_energy_j - compressor_energy_j
        self.omega = math.sqrt(2.0 * e / self.inertia) if e > 0.0 else 0.0

    def control_wastegate(self, dt: float) -> None:
        """Hold the boost at its target with a PI law on the wastegate opening.

        The wastegate opens by ``kp·e`` plus the integral of ``e`` (e = boost above target,
        as a fraction of the target), both scaled by ``turbo_wastegate_gain``. The integral
        is what holds the wastegate where the steady-state boost needs it; it is clamped to
        the valve's own travel (no wind-up while spooling with the gate shut), and the
        proportional part is what stops the overshoot while the shaft is still gaining speed.
        An integrate-only law with a slow close hunts with the spool-up lag.
        """
        e = (self.boost_pressure - self.target_boost) / self.target_boost
        k = self.gain
        i_max = 1.0 / (_WG_KI * k)
        self._wg_integral = min(i_max, max(0.0, self._wg_integral + e * dt))
        pos = k * (_WG_KP * e + _WG_KI * self._wg_integral)
        self.wastegate_position = min(1.0, max(0.0, pos))
