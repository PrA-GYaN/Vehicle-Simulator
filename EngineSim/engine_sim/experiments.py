"""Engine experiments — engine-only test/dyno environment.

Each experiment runs the engine independently without a complete vehicle.
Results are recorded via telemetry for analysis.

Source: ENGINE_KB.md requirements §23.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional

from .config import EngineConfig
from .engine import Engine, EngineInput, EngineOutput
from .telemetry import TelemetryRecorder


@dataclass
class ExperimentResult:
    """Result of a single experiment run."""
    name: str = ""
    description: str = ""
    telemetry: Optional[TelemetryRecorder] = None
    final_output: Optional[EngineOutput] = None
    result_summary: str = ""


class EngineExperiments:
    """Engine-only experiments and dyno tests."""

    def __init__(self, config: Optional[EngineConfig] = None):
        self.config = config or EngineConfig()

    def _create_engine(self) -> Engine:
        return Engine(EngineConfig(
            name=self.config.name,
            bore_m=self.config.bore_m,
            stroke_m=self.config.stroke_m,
            rod_length_m=self.config.rod_length_m,
            compression_ratio=self.config.compression_ratio,
            num_cylinders=self.config.num_cylinders,
            engine_inertia_kgm2=self.config.engine_inertia_kgm2,
            load_inertia_kgm2=0.0,
            idle_rpm=self.config.idle_rpm,
            redline_rpm=self.config.redline_rpm,
            limiter_type=self.config.limiter_type,
            limiter_hysteresis_rpm=self.config.limiter_hysteresis_rpm,
            friction_C1_Pa=self.config.friction_C1_Pa,
            friction_C2_Pa_per_rev_s=self.config.friction_C2_Pa_per_rev_s,
            friction_C3_Pa_per_rev_s2=self.config.friction_C3_Pa_per_rev_s2,
            ambient_pressure_pa=self.config.ambient_pressure_pa,
            intake_temperature_k=self.config.intake_temperature_k,
            max_volumetric_efficiency=self.config.max_volumetric_efficiency,
            stoich_afr=self.config.stoich_afr,
            lower_heating_value=self.config.lower_heating_value,
            fuel_density=self.config.fuel_density,
            combustion_efficiency=self.config.combustion_efficiency,
            torque_curve=list(self.config.torque_curve),
        ))

    def run_experiment(
        self, name: str, duration: float, dt: float,
        throttle_fn=None, load_fn=None,
    ) -> ExperimentResult:
        if throttle_fn is None:
            throttle_fn = lambda t: 0.0
        if load_fn is None:
            load_fn = lambda t: 0.0

        engine = self._create_engine()
        engine.start()
        recorder = TelemetryRecorder()
        recorder.clear()

        t = 0.0
        n_steps = int(duration / dt)
        for _ in range(n_steps):
            inp = EngineInput(throttle=throttle_fn(t), load_torque=load_fn(t))
            out = engine.update(inp, dt)
            recorder.record(t, out)
            t += dt

        return ExperimentResult(
            name=name, telemetry=recorder,
            final_output=engine.last_output,
            result_summary=recorder.summary(),
        )

    def free_rev(self, duration=5.0, dt=0.001, throttle_start=1.0) -> ExperimentResult:
        result = self.run_experiment(
            name="Free Rev", duration=duration, dt=dt,
            throttle_fn=lambda t: throttle_start,
        )
        result.description = f"Free rev at {throttle_start*100:.0f}% throttle"
        return result

    def throttle_step(self, initial_rpm=3000.0, duration=3.0, dt=0.001) -> ExperimentResult:
        engine = self._create_engine()
        engine.start()
        engine.state.omega = initial_rpm * self.config.RPM_TO_RADS

        recorder = TelemetryRecorder()
        recorder.clear()
        t = 0.0
        n_steps = int(duration / dt)
        step_time = 0.5
        for _ in range(n_steps):
            throttle = 1.0 if t >= step_time else 0.0
            out = engine.update(EngineInput(throttle=throttle), dt)
            recorder.record(t, out)
            t += dt

        return ExperimentResult(
            name="Throttle Step",
            description=f"0→100% at t={step_time}s, initial RPM={initial_rpm:.0f}",
            telemetry=recorder, final_output=engine.last_output,
            result_summary=recorder.summary(),
        )

    def throttle_release(self, initial_rpm=5000.0, duration=5.0, dt=0.001) -> ExperimentResult:
        engine = self._create_engine()
        engine.start()
        engine.state.omega = initial_rpm * self.config.RPM_TO_RADS

        recorder = TelemetryRecorder()
        recorder.clear()
        t = 0.0
        release_time = 0.5
        n_steps = int(duration / dt)
        for _ in range(n_steps):
            throttle = 0.0 if t >= release_time else 1.0
            out = engine.update(EngineInput(throttle=throttle), dt)
            recorder.record(t, out)
            t += dt

        return ExperimentResult(
            name="Throttle Release",
            description=f"100→0% at t={release_time}s, initial RPM={initial_rpm:.0f}",
            telemetry=recorder, final_output=engine.last_output,
            result_summary=recorder.summary(),
        )

    def constant_throttle(self, throttle_values=None, duration=5.0, dt=0.001) -> List[ExperimentResult]:
        if throttle_values is None:
            throttle_values = [0.1, 0.25, 0.5, 0.75, 1.0]
        results = []
        for tv in throttle_values:
            result = self.run_experiment(
                name=f"Constant Throttle {tv*100:.0f}%",
                duration=duration, dt=dt,
                throttle_fn=lambda t, tv=tv: tv,
            )
            result.description = f"Constant throttle at {tv*100:.0f}%"
            results.append(result)
        return results

    def external_load(self, load_torque=50.0, throttle=0.5,
                      duration=5.0, dt=0.001) -> ExperimentResult:
        result = self.run_experiment(
            name="External Load", duration=duration, dt=dt,
            throttle_fn=lambda t: throttle,
            load_fn=lambda t: load_torque,
        )
        result.description = f"Load={load_torque:.0f} N·m at {throttle*100:.0f}% throttle"
        return result

    def rpm_sweep(self, duration=10.0, dt=0.001) -> ExperimentResult:
        engine = self._create_engine()
        engine.start()
        recorder = TelemetryRecorder()
        recorder.clear()
        t = 0.0
        n_steps = int(duration / dt)
        for _ in range(n_steps):
            throttle = min(1.0, t / (duration * 0.3))
            out = engine.update(EngineInput(throttle=throttle), dt)
            recorder.record(t, out)
            t += dt

        return ExperimentResult(
            name="RPM Sweep", description="WOT sweep through RPM range",
            telemetry=recorder, final_output=engine.last_output,
            result_summary=recorder.summary(),
        )
