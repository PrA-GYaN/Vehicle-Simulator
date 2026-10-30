"""Engine state — the minimal persistent dynamic state for the engine.

Dynamic state variables:
    omega: crankshaft angular velocity [rad/s] (true ODE state)
    theta: crank angle [rad] (integrates omega)

All other quantities are either inputs or derived outputs.

Source: ENGINE_KB.md Part XVI §25, §29, Eq 14.52.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class AirState:
    """Air intake state for current cycle."""
    intake_pressure: float = 101325.0    # Pa (absolute)
    intake_temperature: float = 293.15   # K
    air_density: float = 1.2             # kg/m^3
    trapped_mass: float = 0.0            # kg per cylinder per cycle
    volumetric_efficiency: float = 0.85  # dimensionless
    mass_flow_rate: float = 0.0          # kg/s


@dataclass
class FuelState:
    """Fuel delivery state for current cycle."""
    fuel_mass_per_cycle: float = 0.0     # kg per cylinder per cycle
    fuel_mass_flow_rate: float = 0.0     # kg/s
    afr: float = 14.7                    # actual air-fuel ratio
    lambda_val: float = 1.0              # λ = actual/stoich AFR
    equivalence_ratio: float = 1.0       # φ = 1/λ


@dataclass
class CombustionState:
    """Combustion results for current cycle."""
    indicated_torque: float = 0.0        # N·m (cycle-averaged)
    work_net: float = 0.0                # J per cycle
    T2: float = 0.0                      # K, end of compression
    T3: float = 0.0                      # K, peak combustion
    p2: float = 0.0                      # Pa, end of compression
    p3: float = 0.0                      # Pa, peak combustion
    eta_th: float = 0.0                  # thermal efficiency
    mass_fraction_burned: float = 0.0    # current x_b (Wiebe)
    spark_timing_deg: float = 0.0        # actual spark timing [deg BTDC]
    combustion_duration_deg: float = 0.0 # Δθ [crank-angle degrees]
    knock_integral: float = 0.0          # Livengood-Wu integral (1.0 = onset)
    knock_detected: bool = False


@dataclass
class FrictionState:
    """Friction and loss torques."""
    rubbing_friction_torque: float = 0.0  # N·m
    pumping_torque: float = 0.0           # N·m
    accessory_torque: float = 0.0         # N·m
    total_loss_torque: float = 0.0        # N·m
    # Component breakdown (KB Part XI)
    piston_friction_torque: float = 0.0
    bearing_friction_torque: float = 0.0
    valvetrain_friction_torque: float = 0.0


@dataclass
class EngineState:
    """Dynamic engine state.

    The sole dynamic state variable is angular velocity omega.
    Crank angle theta integrates omega for cycle timing.

    Source: ENGINE_KB.md Eq 14.52, Part XVI §25.
    """
    # --- Dynamic state ---
    omega: float = 0.0         # Crankshaft angular velocity [rad/s]
    theta: float = 0.0         # Crank angle from TDC intake [rad]

    # --- Derived quantities (not state — computed each step) ---
    throttle: float = 0.0
    limiter_active: bool = False

    # --- Sub-model states ---
    air: AirState = None
    fuel: FuelState = None
    combustion: CombustionState = None
    friction: FrictionState = None

    # --- Manifold dynamics ---
    manifold_pressure: float = 101325.0  # Pa (dynamic, from ODE)
    manifold_temperature: float = 293.15  # K

    # --- Turbo state ---
    turbo_boost_pressure: float = 101325.0
    turbo_shaft_speed_rps: float = 0.0
    wastegate_position: float = 0.0

    # --- Emissions ---
    nox_ppm: float = 0.0
    co_ppm: float = 0.0
    hc_ppm: float = 0.0

    # --- Lambda control ---
    lambda_correction: float = 1.0
    sensor_voltage: float = 0.45

    def __post_init__(self):
        if self.air is None:
            self.air = AirState()
        if self.fuel is None:
            self.fuel = FuelState()
        if self.combustion is None:
            self.combustion = CombustionState()
        if self.friction is None:
            self.friction = FrictionState()

    @property
    def rpm(self) -> float:
        return self.omega * 60.0 / (2.0 * math.pi)

    @property
    def angular_velocity(self) -> float:
        return self.omega

    @property
    def crank_angle_deg(self) -> float:
        return self.theta * 180.0 / math.pi

    @property
    def stroke_phase(self) -> int:
        """0=intake, 1=compression, 2=power, 3=exhaust."""
        ca = math.fmod(self.theta * 180.0 / math.pi, 720.0)
        if ca < 0:
            ca += 720.0
        if ca < 180:
            return 0
        if ca < 360:
            return 1
        if ca < 540:
            return 2
        return 3

    def reset(self, idle_rads: float = 0.0) -> None:
        self.omega = idle_rads
        self.theta = 0.0
        self.throttle = 0.0
        self.limiter_active = False
        self.air = AirState()
        self.fuel = FuelState()
        self.combustion = CombustionState()
        self.friction = FrictionState()
