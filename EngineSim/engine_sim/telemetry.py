"""Telemetry recording system for engine simulation.

Records engine state at each physics timestep for analysis and debugging.

Source: ENGINE_KB.md Part XVI §27.
"""

from __future__ import annotations

import csv
import os
from dataclasses import dataclass, fields, asdict
from typing import List, Optional

from .engine import EngineOutput


@dataclass
class TelemetryEntry:
    """One timestep of telemetry data."""
    time: float = 0.0
    engine_rpm: float = 0.0
    engine_angular_velocity: float = 0.0
    engine_throttle: float = 0.0
    engine_torque: float = 0.0
    engine_friction_torque: float = 0.0
    engine_load_torque: float = 0.0
    engine_net_torque: float = 0.0
    engine_angular_acceleration: float = 0.0
    engine_power: float = 0.0
    engine_limiter: bool = False
    # v0.2 thermodynamic fields
    crank_angle_deg: float = 0.0
    intake_pressure: float = 0.0
    intake_temperature: float = 0.0
    air_mass_per_cycle: float = 0.0
    fuel_mass_per_cycle: float = 0.0
    afr: float = 14.7
    lambda_val: float = 1.0
    volumetric_efficiency: float = 0.0
    mass_flow_rate: float = 0.0
    indicated_torque: float = 0.0
    pumping_torque: float = 0.0
    accessory_torque: float = 0.0
    cyl_pressure: float = 0.0
    cyl_temperature: float = 0.0
    stroke_phase: int = 0
    cycle_work: float = 0.0
    thermal_efficiency: float = 0.0
    # New v0.3 fields
    mass_fraction_burned: float = 0.0
    manifold_pressure: float = 0.0
    spark_timing_deg: float = 0.0
    combustion_duration_deg: float = 0.0
    heat_transfer_loss_w: float = 0.0
    boost_pressure: float = 0.0
    nox_ppm: float = 0.0
    co_ppm: float = 0.0
    hc_ppm: float = 0.0
    knock_integral: float = 0.0
    knock_detected: bool = False
    # Resolved-core fields (0 on the cycle-averaged path)
    resolved: bool = False
    exhaust_pressure: float = 0.0
    exhaust_temperature: float = 0.0
    imep_net_bar: float = 0.0
    imep_gross_bar: float = 0.0
    pmep_bar: float = 0.0
    peak_pressure_pa: float = 0.0
    knock_retard_deg: float = 0.0


class TelemetryRecorder:
    """Records engine telemetry at each physics timestep."""

    def __init__(self):
        self._entries: List[TelemetryEntry] = []

    def clear(self) -> None:
        self._entries.clear()

    @property
    def entries(self) -> List[TelemetryEntry]:
        return list(self._entries)

    @property
    def count(self) -> int:
        return len(self._entries)

    def record(self, time: float, output: EngineOutput) -> None:
        entry = TelemetryEntry(
            time=time,
            engine_rpm=output.rpm,
            engine_angular_velocity=output.angular_velocity,
            engine_throttle=output.throttle,
            engine_torque=output.engine_torque,
            engine_friction_torque=output.friction_torque,
            engine_load_torque=output.load_torque,
            engine_net_torque=output.net_torque,
            engine_angular_acceleration=output.angular_acceleration,
            engine_power=output.power,
            engine_limiter=output.limiter_active,
            crank_angle_deg=output.crank_angle_deg,
            intake_pressure=output.intake_pressure,
            intake_temperature=output.intake_temperature,
            air_mass_per_cycle=output.air_mass_per_cycle,
            fuel_mass_per_cycle=output.fuel_mass_per_cycle,
            afr=output.afr,
            lambda_val=output.lambda_val,
            volumetric_efficiency=output.volumetric_efficiency,
            mass_flow_rate=output.mass_flow_rate,
            indicated_torque=output.indicated_torque,
            pumping_torque=output.pumping_torque,
            accessory_torque=output.accessory_torque,
            cyl_pressure=output.cyl_pressure,
            cyl_temperature=output.cyl_temperature,
            stroke_phase=output.stroke_phase,
            cycle_work=output.cycle_work,
            thermal_efficiency=output.thermal_efficiency,
            mass_fraction_burned=getattr(output, 'mass_fraction_burned', 0.0),
            manifold_pressure=getattr(output, 'manifold_pressure', 0.0),
            spark_timing_deg=getattr(output, 'spark_timing_deg', 0.0),
            combustion_duration_deg=getattr(output, 'combustion_duration_deg', 0.0),
            heat_transfer_loss_w=getattr(output, 'heat_transfer_loss_w', 0.0),
            boost_pressure=getattr(output, 'boost_pressure', 0.0),
            nox_ppm=getattr(output, 'nox_ppm', 0.0),
            co_ppm=getattr(output, 'co_ppm', 0.0),
            hc_ppm=getattr(output, 'hc_ppm', 0.0),
            knock_integral=getattr(output, 'knock_integral', 0.0),
            knock_detected=getattr(output, 'knock_detected', False),
            resolved=getattr(output, 'resolved', False),
            exhaust_pressure=getattr(output, 'exhaust_pressure', 0.0),
            exhaust_temperature=getattr(output, 'exhaust_temperature', 0.0),
            imep_net_bar=getattr(output, 'imep_net_bar', 0.0),
            imep_gross_bar=getattr(output, 'imep_gross_bar', 0.0),
            pmep_bar=getattr(output, 'pmep_bar', 0.0),
            peak_pressure_pa=getattr(output, 'peak_pressure_pa', 0.0),
            knock_retard_deg=getattr(output, 'knock_retard_deg', 0.0),
        )
        self._entries.append(entry)

    def save(self, path: str) -> None:
        if not self._entries:
            return
        field_names = [f.name for f in fields(TelemetryEntry)]
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        with open(path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=field_names)
            writer.writeheader()
            for entry in self._entries:
                writer.writerow(asdict(entry))

    def get_rpm_trace(self) -> List[float]:
        return [e.engine_rpm for e in self._entries]

    def get_torque_trace(self) -> List[float]:
        return [e.engine_net_torque for e in self._entries]

    def get_power_trace(self) -> List[float]:
        return [e.engine_power for e in self._entries]

    def get_time_trace(self) -> List[float]:
        return [e.time for e in self._entries]

    def get_throttle_trace(self) -> List[float]:
        return [e.engine_throttle for e in self._entries]

    def get_indicated_torque_trace(self) -> List[float]:
        return [e.indicated_torque for e in self._entries]

    def get_friction_torque_trace(self) -> List[float]:
        return [e.engine_friction_torque for e in self._entries]

    def get_intake_pressure_trace(self) -> List[float]:
        return [e.intake_pressure for e in self._entries]

    def summary(self) -> str:
        if not self._entries:
            return "No telemetry recorded."
        lines = [
            "=== Telemetry Summary ===",
            f"  Duration: {self._entries[-1].time:.3f} s",
            f"  Steps: {len(self._entries)}",
            f"  RPM range: {min(e.engine_rpm for e in self._entries):.0f} - "
            f"{max(e.engine_rpm for e in self._entries):.0f}",
            f"  Max indicated torque: {max(e.indicated_torque for e in self._entries):.1f} N·m",
            f"  Max brake power: {max(e.engine_power for e in self._entries)/1000:.1f} kW",
            f"  Limiter activations: {sum(1 for e in self._entries if e.engine_limiter)}",
            f"  Avg thermal efficiency: "
            f"{sum(e.thermal_efficiency for e in self._entries)/len(self._entries)*100:.1f}%",
        ]
        return "\n".join(lines)
