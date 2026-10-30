"""Powertrain — engine + clutch + gearbox + vehicle + dyno + fuel accounting.

Engine braking lives here: with the clutch locked and the throttle closed the
engine's own losses (friction + pumping + accessories, fuel cut by DFCO) are
reflected through the gearbox and decelerate the vehicle, and the vehicle in
turn drives the engine (a negative load torque on the crank).

Model (all engine-referred, G = gear ratio · final drive):

    I_v = (m·r_w² + I_wheels) / G² + I_gearbox        reflected vehicle inertia
    T_res = (F_aero + F_roll + F_brake) · r_w / G      reflected road load

    Slipping clutch:   T_c = cap · sign(ω_e − ω_in)
    Locked clutch:     α   = (T_e − T_res) / (I_e + I_v)
                       T_c = T_e − I_e·α          (stays locked while |T_c| ≤ cap)

    I_e·dω_e/dt  = T_e − T_c − T_dyno
    I_v·dω_in/dt = T_c − T_res

The clutch/vehicle/dyno equations are standard rigid-body dynamics; the KB
does not cover drivetrains (KB_GAPS.md G-E07), so parameters are AUTHORED.
The engine's own torque T_e is taken from the previous step (1 ms lag) so the
engine update itself stays unchanged.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional

from .config import EngineConfig
from .engine import Engine, EngineInput, EngineOutput

G_ACCEL = 9.81
AIR_DENSITY = 1.204           # kg/m³ at 20 °C
LB_FT_PER_NM = 0.737562
W_PER_HP = 745.7


@dataclass
class VehicleConfig:
    """Vehicle and driveline parameters (AUTHORED, KB_GAPS.md G-E07)."""
    mass_kg: float = 1150.0
    wheel_radius_m: float = 0.29
    gear_ratios: List[float] = field(
        default_factory=lambda: [3.545, 1.904, 1.310, 1.031, 0.815])
    final_drive: float = 4.312
    wheel_inertia_kgm2: float = 3.2       # all four wheels + axles
    gearbox_inertia_kgm2: float = 0.01    # clutch disc + input shaft
    cd_a_m2: float = 0.66                 # drag coefficient × frontal area
    rolling_resistance: float = 0.012
    clutch_capacity_nm: float = 260.0
    brake_max_decel_g: float = 0.9
    clutch_engage_rate: float = 2.5       # engagement fraction per second
    clutch_release_rate: float = 12.0


@dataclass
class Controls:
    """Driver inputs, set by the UI each frame."""
    throttle: float = 0.0
    ignition: bool = False
    starter: bool = False
    clutch_pedal: float = 0.0     # 0 = released (engaged) .. 1 = floored
    gear: int = 0                 # 0 = neutral, 1..n
    brake: float = 0.0            # 0..1
    auto_clutch: bool = True
    dyno_enabled: bool = False
    dyno_rpm: float = 1000.0
    vvt_override: bool = False     # True: hold the cams at the two angles below instead of the ECU schedule
    vvt_intake_deg: float = 0.0
    vvt_exhaust_deg: float = 0.0


@dataclass
class PowertrainOutput:
    engine: EngineOutput
    engine_rpm: float = 0.0
    vehicle_speed_ms: float = 0.0
    gear: int = 0
    clutch_engagement: float = 0.0
    clutch_locked: bool = False
    clutch_torque: float = 0.0
    dyno_torque: float = 0.0
    flywheel_torque: float = 0.0      # brake torque available at the flywheel
    fuel_used_kg: float = 0.0
    distance_m: float = 0.0


class Powertrain:
    """Engine coupled to a vehicle through a friction clutch and gearbox."""

    def __init__(self, engine_cfg: Optional[EngineConfig] = None,
                 vehicle_cfg: Optional[VehicleConfig] = None):
        self.engine = Engine(engine_cfg or EngineConfig())
        self.vehicle = vehicle_cfg or VehicleConfig()
        self.controls = Controls()
        self.reset()

    # ------------------------------------------------------------------
    def reset(self) -> None:
        self.engine.reset()
        self.omega_in = 0.0            # gearbox input shaft speed [rad/s]
        self.v = 0.0                   # vehicle speed [m/s]
        self.gear = 0
        self.engagement = 1.0
        self.locked = False
        self._T_eng_prev = 0.0
        self._dyno_integral = 0.0
        self.fuel_used_kg = 0.0
        self.distance_m = 0.0
        self.time_s = 0.0
        self.last: PowertrainOutput = PowertrainOutput(engine=self.engine.last_output)

    @property
    def cfg(self) -> EngineConfig:
        return self.engine.config

    def apply_configs(self, engine_cfg: EngineConfig, vehicle_cfg: Optional[VehicleConfig] = None) -> None:
        """Live re-tune: use new engine / vehicle settings without stopping the engine."""
        self.engine.set_config(engine_cfg)
        if vehicle_cfg is not None:
            self.vehicle = vehicle_cfg
            self.gear = min(self.gear, len(vehicle_cfg.gear_ratios))

    def total_ratio(self, gear: int) -> float:
        if gear <= 0 or gear > len(self.vehicle.gear_ratios):
            return 0.0
        return self.vehicle.gear_ratios[gear - 1] * self.vehicle.final_drive

    def shift(self, gear: int) -> None:
        gear = max(0, min(len(self.vehicle.gear_ratios), gear))
        if gear == self.gear:
            return
        self.gear = gear
        # Shifting breaks the lock; the clutch dips and re-engages at the
        # normal rate (a quick driver clutch-dip / shift assist).
        self.locked = False
        self.engagement = 0.0
        G = self.total_ratio(gear)
        if G > 0:
            self.omega_in = self.v / self.vehicle.wheel_radius_m * G

    # ------------------------------------------------------------------
    def _reflected_inertia(self, G: float) -> float:
        vc = self.vehicle
        return ((vc.mass_kg * vc.wheel_radius_m ** 2 + vc.wheel_inertia_kgm2) / (G * G)
                + vc.gearbox_inertia_kgm2)

    def _road_force(self, v: float, brake: float) -> float:
        """Resisting force [N], positive opposes forward motion.

        Rolling resistance and brakes are Coulomb forces; tanh smoothing
        around zero speed keeps the explicit integration stable at rest.
        """
        vc = self.vehicle
        s = math.tanh(v / 0.3)
        f_aero = 0.5 * AIR_DENSITY * vc.cd_a_m2 * v * abs(v)
        f_roll = vc.rolling_resistance * vc.mass_kg * G_ACCEL * s
        f_brake = brake * vc.brake_max_decel_g * vc.mass_kg * G_ACCEL * s
        return f_aero + f_roll + f_brake

    def _clutch_target(self, c: Controls, rpm: float) -> float:
        if self.gear == 0:
            return 1.0
        manual = 1.0 - max(0.0, min(1.0, c.clutch_pedal))
        if not c.auto_clutch:
            return manual
        # Auto clutch: released at idle, bites progressively as the engine
        # revs up (like a launch), and opens before the engine is lugged
        # down into a stall.
        idle = self.cfg.idle_rpm
        if self.locked:
            auto = 1.0 if rpm > 0.8 * idle else 0.0
        else:
            x = (rpm - 1.05 * idle) / (0.9 * idle)
            auto = max(0.0, min(1.0, x)) ** 1.5
        return min(manual, auto)

    def _dyno_torque(self, c: Controls, omega: float, dt: float) -> float:
        """Speed-holding dynamometer (absorbs or motors), PI on speed."""
        if not c.dyno_enabled:
            self._dyno_integral = 0.0
            return 0.0
        I = self.cfg.total_inertia
        Kp, Ki, T_max = 40.0 * I, 400.0 * I, 2000.0
        err = omega - c.dyno_rpm * math.pi / 30.0
        integral = self._dyno_integral + err * dt
        T = Kp * err + Ki * integral
        if abs(T) < T_max:
            self._dyno_integral = integral
        return max(-T_max, min(T_max, T))

    # ------------------------------------------------------------------
    def step(self, dt: float, controls: Optional[Controls] = None) -> PowertrainOutput:
        c = controls or self.controls
        vc = self.vehicle
        eng = self.engine
        I_e = self.cfg.total_inertia
        omega_e = eng.state.omega
        rpm = omega_e * 30.0 / math.pi

        c.gear = max(0, min(len(vc.gear_ratios), c.gear))
        if c.gear != self.gear:
            self.shift(c.gear)
        G = self.total_ratio(self.gear)

        # Clutch engagement follows the pedal / auto-clutch with finite rates
        target = self._clutch_target(c, rpm)
        rate = vc.clutch_engage_rate if target > self.engagement else vc.clutch_release_rate
        d_eng = rate * dt
        self.engagement += max(-d_eng, min(d_eng, target - self.engagement))
        cap = vc.clutch_capacity_nm * self.engagement if G > 0 else 0.0

        T_dyno = self._dyno_torque(c, omega_e, dt)

        # --- Clutch torque ---
        T_c = 0.0
        if G > 0 and cap > 0:
            I_v = self._reflected_inertia(G)
            T_res = self._road_force(self.v, c.brake) * vc.wheel_radius_m / G
            T_e_side = self._T_eng_prev - T_dyno
            if self.locked:
                alpha = (T_e_side - T_res) / (I_e + I_v)
                T_req = T_e_side - I_e * alpha
                if abs(T_req) <= cap:
                    T_c = T_req
                else:
                    self.locked = False
                    T_c = math.copysign(cap, T_req)
            else:
                slip = omega_e - self.omega_in
                if slip != 0.0:
                    T_c = math.copysign(cap, slip)
        else:
            self.locked = False

        # --- Engine ---
        out = eng.update(EngineInput(
            throttle=c.throttle, load_torque=T_c + T_dyno,
            ignition=c.ignition, starter=c.starter,
            vvt_intake_deg=c.vvt_intake_deg if c.vvt_override else None,
            vvt_exhaust_deg=c.vvt_exhaust_deg if c.vvt_override else None,
        ), dt)
        self._T_eng_prev = out.net_torque + out.load_torque

        # --- Vehicle ---
        if G > 0:
            I_v = self._reflected_inertia(G)
            T_res = self._road_force(self.v, c.brake) * vc.wheel_radius_m / G
            slip_before = omega_e - self.omega_in
            self.omega_in += (T_c - T_res) / I_v * dt
            if self.omega_in < 0.0:
                self.omega_in = 0.0          # no reverse gear, no hills
            if self.locked:
                w = (I_e * eng.state.omega + I_v * self.omega_in) / (I_e + I_v)
                eng.state.omega = self.omega_in = max(0.0, w)
            elif cap > 0:
                slip_after = eng.state.omega - self.omega_in
                if slip_before * slip_after <= 0.0 or abs(slip_after) < 0.3:
                    # Slip crossed zero this step: lock, conserving momentum
                    w = (I_e * eng.state.omega + I_v * self.omega_in) / (I_e + I_v)
                    eng.state.omega = self.omega_in = max(0.0, w)
                    self.locked = True
            self.v = self.omega_in * vc.wheel_radius_m / G
        else:
            F = self._road_force(self.v, c.brake)
            self.v = max(0.0, self.v - F / (vc.mass_kg + vc.wheel_inertia_kgm2
                                            / vc.wheel_radius_m ** 2) * dt)

        # --- Accounting ---
        self.fuel_used_kg += eng.state.fuel.fuel_mass_flow_rate * dt
        self.distance_m += self.v * dt
        self.time_s += dt

        omega_now = eng.state.omega
        flywheel = (out.indicated_torque - out.friction_torque
                    - out.pumping_torque - out.accessory_torque)
        self.last = PowertrainOutput(
            engine=out,
            engine_rpm=omega_now * 30.0 / math.pi,
            vehicle_speed_ms=self.v,
            gear=self.gear,
            clutch_engagement=self.engagement if G > 0 else 0.0,
            clutch_locked=self.locked,
            clutch_torque=T_c,
            dyno_torque=T_dyno,
            flywheel_torque=flywheel,
            fuel_used_kg=self.fuel_used_kg,
            distance_m=self.distance_m,
        )
        return self.last


# ----------------------------------------------------------------------
# Engines: defined by the TOML files in config/engines/ (one file per engine).
# ----------------------------------------------------------------------

def load_engine(ref: str):
    """(EngineConfig, VehicleConfig) for an engine id ("454") or a path to a .toml file."""
    from .engine_file import load_engine_file, resolve_engine
    return load_engine_file(resolve_engine(ref)).as_tuple()


def preset_chevy_454():
    """Chevrolet 454 big-block V8: config/engines/chevrolet_454.toml."""
    return load_engine("454")


def preset_1nz_fe():
    """Toyota 1NZ-FE 1.5 L I4: config/engines/toyota_1nz_fe.toml."""
    return load_engine("1nz")


def _discover_presets() -> dict:
    from .engine_file import list_engines
    return {e.id: (lambda path=e.path: load_engine(path)) for e in list_engines() if not e.error}


PRESETS = _discover_presets()      # id -> loader, for the engines present at import time
