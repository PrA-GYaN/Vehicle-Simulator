"""Closed-loop lambda (λ) control model.

All equations from ENGINE_KB.md (Heywood 1988) Part VI.

Key equations:
    Nernst equation for O₂ sensor (KB Eq 7.15):
        V_o = (RT/4F) · ln(p″_{O₂}/p′_{O₂})

    Limit-cycle frequency (KB Eq 7.16):
        f_LC = 1/(4·τ_L)

    Peak-to-peak oscillation (KB Eq 7.17):
        Δφ = 2·K·τ_L

where:
    τ_L = transport delay (fuel admission to sensor)
    K = integrator gain
"""

from __future__ import annotations
import math


class LambdaController:
    """Closed-loop λ (lambda) controller with limit-cycle behavior."""

    def __init__(
        self,
        target_lambda: float = 1.0,
        integrator_gain: float = 2.0,
        transport_delay_s: float = 0.05,
        sensor_gain: float = 100.0,
        enabled: bool = True,
    ):
        """
        Args:
            target_lambda: Target lambda value (1.0 = stoichiometric).
            integrator_gain: Controller gain K [lambda units / s].
            transport_delay_s: Transport delay τ_L [s] (fuel to sensor).
            sensor_gain: O₂ sensor gain (V per lambda unit).
            enabled: Whether closed-loop control is active.
        """
        self.target_lambda = target_lambda
        self.K = integrator_gain
        self.tau_L = transport_delay_s
        self.sensor_gain = sensor_gain
        self.enabled = enabled

        self._integral_error = 0.0
        self._prev_error = 0.0
        self._sensor_voltage = 0.45  # ~0.45V at stoich (Nernst)
        self._fuel_correction = 1.0

    def reset(self) -> None:
        self._integral_error = 0.0
        self._prev_error = 0.0
        self._sensor_voltage = 0.45
        self._fuel_correction = 1.0

    def update(
        self,
        actual_lambda: float,
        exhaust_temp_k: float,
        dt: float,
    ) -> tuple[float, float]:
        """Update lambda controller.

        Args:
            actual_lambda: Measured lambda from O₂ sensor.
            exhaust_temp_k: Exhaust gas temperature [K] (affects sensor response).
            dt: Timestep [s].

        Returns:
            Tuple of (fuel_correction_factor, sensor_voltage).
        """
        if not self.enabled:
            return 1.0, 0.45

        # Nernst O₂ sensor voltage (KB Eq 7.15)
        # High voltage (>0.8V) = rich (λ < 1)
        # Low voltage (<0.2V) = lean (λ > 1)
        # Steep transition around λ = 1.0
        self._sensor_voltage = self._nernst_voltage(actual_lambda, exhaust_temp_k)

        # Reference voltage at stoichiometric
        V_ref = 0.45

        # Comparator output (lean = negative error → need more fuel)
        # Lean (λ > 1): V_sensor < V_ref → comparator < 0 → need to add fuel
        comparator = V_ref - self._sensor_voltage

        # Integral controller (KB Fig 7-19)
        self._integral_error += comparator * dt
        self._integral_error = max(-1.0, min(1.0, self._integral_error))

        # Fuel correction: lean → add fuel (correction > 1), rich → remove fuel
        correction = self.K * self._integral_error

        # Fuel factor: >1 = add fuel (lean), <1 = remove fuel (rich)
        self._fuel_correction = 1.0 + correction * 0.1

        # Clamp to reasonable range
        self._fuel_correction = max(0.8, min(1.2, self._fuel_correction))

        return self._fuel_correction, self._sensor_voltage

    def _nernst_voltage(self, lambda_val: float, exhaust_temp_k: float) -> float:
        """Nernst equation for O₂ sensor voltage.

        KB Eq 7.15:
            V_o = (RT/4F) · ln(p″_{O₂}/p′_{O₂})

        Simplified: steep step function around λ = 1.0.

        Args:
            lambda_val: Actual lambda.
            exhaust_temp_k: Exhaust temperature [K].

        Returns:
            Sensor voltage [V].
        """
        # Temperature effect on sensor response
        T_ref = 800.0  # reference temp for sensor [K]
        temp_factor = min(1.0, exhaust_temp_k / T_ref) if exhaust_temp_k > 0 else 0.5

        # Nernst-like step function
        # At λ = 1.0: V = 0.45V (midpoint)
        # Rich (λ < 1): V → 0.9V
        # Lean (λ > 1): V → 0.1V
        V_rich = 0.9
        V_lean = 0.1
        V_stoich = 0.45

        # Sharp transition using tanh
        # Width of transition: narrower = more ideal sensor
        width = 0.02 * temp_factor  # narrower at high temp (faster response)
        x = (lambda_val - 1.0) / max(width, 0.005)
        V = V_stoich + (V_rich - V_lean) / 2.0 * math.tanh(-x * 5.0)

        return V

    @property
    def limit_cycle_frequency(self) -> float:
        """Limit-cycle frequency (KB Eq 7.16).
        Returns:
            Frequency [Hz].
        """
        return 1.0 / (4.0 * self.tau_L) if self.tau_L > 0 else 10.0

    @property
    def peak_to_peak_oscillation(self) -> float:
        """Peak-to-peak equivalence ratio oscillation (KB Eq 7.17).
        Returns:
            Δφ (dimensionless).
        """
        return 2.0 * self.K * self.tau_L

    @property
    def fuel_correction(self) -> float:
        return self._fuel_correction

    @property
    def sensor_voltage(self) -> float:
        return self._sensor_voltage
