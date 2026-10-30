"""Engine configuration — data-driven parameters for the engine simulator.

All quantities use SI units internally. RPM is used for configuration interface
and converted to rad/s at load time. Every parameter has its unit documented.

Sources: ENGINE_KB.md Parts XVII, XXIV.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional


@dataclass
class TorquePoint:
    """A single point on the WOT torque curve.
    (Retained for compatibility; the thermodynamic model supersedes this.)
    """
    rpm: float
    torque: float


@dataclass
class EngineConfig:
    """Complete engine configuration.

    Units:
        All angular quantities in rad/s internally.
        RPM values stored as rad/s after conversion.
        Torque in N·m, power in W, inertia in kg·m^2,
        displacement in m^3, friction coefficients in Pa.

    Source: ENGINE_KB.md Part XVII (Parameter Catalog), Part XXIV.
    """

    # --- Engine identification ---
    name: str = "1NZ-FE 1.5L SI 4-cyl"

    # --- Geometry (Source: KB Part XVII §A) ---
    bore_m: float = 0.0825          # 82.5 mm (KB §I Table 1.3)
    stroke_m: float = 0.086         # 86 mm
    rod_length_m: float = 0.144     # 144 mm
    compression_ratio: float = 10.0 # rc (KB Eq 2.1)
    num_cylinders: int = 4

    @property
    def crank_radius(self) -> float:
        """Crank radius r = S/2 [m]."""
        return self.stroke_m / 2.0

    @property
    def rod_ratio(self) -> float:
        """Rod ratio R = l/r."""
        return self.rod_length_m / self.crank_radius if self.crank_radius > 0 else 4.0

    @property
    def displacement_per_cyl(self) -> float:
        """Displaced volume per cylinder [m^3].  V_d = (π/4)·B²·S"""
        return (math.pi / 4.0) * self.bore_m ** 2 * self.stroke_m

    @property
    def clearance_volume(self) -> float:
        """Clearance volume per cylinder [m^3].  V_c = V_d / (rc - 1)"""
        rd = self.displacement_per_cyl
        return rd / (self.compression_ratio - 1.0) if self.compression_ratio > 1.0 else rd

    @property
    def total_displacement(self) -> float:
        """Total engine displacement [m^3]."""
        return self.displacement_per_cyl * self.num_cylinders

    @property
    def displacement_m3(self) -> float:
        """Alias for total displacement [m^3]."""
        return self.total_displacement

    # --- Rotational inertia (Source: KB Part XVII §E, Eq 14.52) ---
    engine_inertia_kgm2: float = 0.015  # I_E [kg·m^2]
    load_inertia_kgm2: float = 0.0      # I_L [kg·m^2]

    @property
    def total_inertia(self) -> float:
        """Total rotational inertia I_E + I_L [kg·m^2].
        Source: ENGINE_KB.md Eq 14.52.
        """
        return self.engine_inertia_kgm2 + self.load_inertia_kgm2

    # --- Speed limits (Source: KB Part XVII §A, Part XXI §37) ---
    idle_rpm: float = 800.0
    redline_rpm: float = 7000.0
    limiter_type: str = "torque_cut"
    limiter_hysteresis_rpm: float = 200.0

    @property
    def idle_rads(self) -> float:
        return self.idle_rpm * 2.0 * math.pi / 60.0

    @property
    def redline_rads(self) -> float:
        return self.redline_rpm * 2.0 * math.pi / 60.0

    @property
    def limiter_hysteresis_rads(self) -> float:
        return self.limiter_hysteresis_rpm * 2.0 * math.pi / 60.0

    # --- Run state / ECU (KB_GAPS.md G-E03..G-E06; authored, not from KB) ---
    # limiter_type: "torque_cut" clamps speed at redline; "fuel_cut" cuts
    # fuel at redline and resumes at redline - hysteresis (speed bounces).
    min_firing_rpm: float = 120.0      # below this the engine only motors
    crank_revs_to_fire: float = 1.5    # cranking revs before sync + first fire
    stall_rpm: float = 250.0           # below this, unassisted, the engine dies
    dfco_enter_ratio: float = 1.2      # DFCO when closed throttle & ω > ratio·idle
    dfco_resume_ratio: float = 1.2     # fuel resumes when ω < ratio·idle
    dfco_after_start_delay_s: float = 2.5   # resolved core: no decel fuel cut this long after the engine first fires

    # Starter motor, referred to the crankshaft (pinion ratio included)
    starter_stall_torque_nm: float = 60.0
    starter_free_speed_rpm: float = 400.0

    # Idle-air bypass valve (only with use_manifold_dynamics). Positions are
    # fractions of throttle_max_area_m2. When off, the torque-mode governor
    # in engine.py Step 8 holds idle instead.
    use_idle_air_control: bool = False
    iac_feedforward: float = 0.008
    iac_min: float = 0.001
    iac_max: float = 0.012             # ~3x the idle position: real IAC authority
    iac_kp: float = 0.03               # per unit normalised speed error
    iac_ki: float = 0.02               # per unit normalised speed error · s
    # The resolved core breathes through a pulsed intake with a real cycle delay on
    # top of the manifold lag, so the same PI gains that hold the cycle-averaged
    # model steady make it hunt: a gain this much smaller has the same phase
    # margin. It scales kp and ki only while use_resolved_cylinders is on.
    iac_resolved_gain_scale: float = 0.15
    fast_idle_extra_ratio: float = 0.5           # resolved core: idle target is this much higher just after the first fire...
    fast_idle_decay_s: float = 4.0               # ...and decays back to the idle speed with this time constant
    iac_resolved_kp_fraction: float = 1.0      # extra factor on kp only (the spark loop carries the fast correction)

    @property
    def min_firing_rads(self) -> float:
        return self.min_firing_rpm * 2.0 * math.pi / 60.0

    @property
    def stall_rads(self) -> float:
        return self.stall_rpm * 2.0 * math.pi / 60.0

    @property
    def starter_free_speed_rads(self) -> float:
        return self.starter_free_speed_rpm * 2.0 * math.pi / 60.0

    # --- Layout: used by the animation and by the sound (see sound_spec.py) ---
    # Firing order (1-based cylinder numbers) and which bank each cylinder
    # (1..N) sits on.
    firing_order: List[int] = field(default_factory=lambda: [1, 3, 4, 2])
    cylinder_bank: List[int] = field(default_factory=lambda: [0, 0, 0, 0])
    bank_angle_deg: float = 0.0        # angle between banks (0 = inline)
    # Crank degrees between successive firings, in firing order (sum = 720).
    # None = evenly spaced (720/N). Odd-fire engines (e.g. 90° V6) set it.
    firing_intervals_deg: Optional[List[float]] = None

    # --- Valve timing [crank degrees] (typical road engine) ---
    intake_open_btdc_deg: float = 10.0     # intake opens before exhaust-stroke TDC
    intake_close_abdc_deg: float = 50.0    # intake closes after intake BDC
    exhaust_open_bbdc_deg: float = 50.0    # exhaust opens before power-stroke BDC
    exhaust_close_atdc_deg: float = 10.0   # exhaust closes after exhaust-stroke TDC

    # --- Valve geometry and cam shape ---
    # Diameters are PER VALVE; the number of valves comes from num_valves_per_cyl
    # (see intake_valve_count / exhaust_valve_count), so a 4-valve head passes
    # twice the single-valve area. Lift follows a raised-cosine hump over the
    # event window with the opening and closing ramps split by the ramp fractions;
    # 0.5/0.5 is the symmetric sin² hump. Asymmetric ramps model a race cam's
    # fast-open/slow-close.
    intake_max_lift_m: float = 9.5e-3
    intake_ramp_open_frac: float = 0.5
    intake_ramp_close_frac: float = 0.5
    intake_head_dia_m: float = 30.0e-3     # valve head diameter, per valve
    intake_port_dia_m: float = 30.0e-3     # port throat diameter, per valve
    intake_seat_angle_deg: float = 45.0    # seat angle β from the head plane (KB Eq 6.7), not the included angle
    intake_discharge_coeff: float = 0.67   # valve discharge coefficient
    exhaust_max_lift_m: float = 9.5e-3
    exhaust_ramp_open_frac: float = 0.5
    exhaust_ramp_close_frac: float = 0.5
    exhaust_head_dia_m: float = 26.0e-3
    exhaust_port_dia_m: float = 26.0e-3
    exhaust_seat_angle_deg: float = 45.0
    exhaust_discharge_coeff: float = 0.67

    @property
    def intake_valve_count(self) -> int:
        """Intake valves per cylinder (a 4-valve head has 2, a 3-valve head 2, a 2-valve head 1)."""
        return max(1, (self.num_valves_per_cyl + 1) // 2)

    @property
    def exhaust_valve_count(self) -> int:
        """Exhaust valves per cylinder (a 4-valve head has 2, a 3-valve head 1, a 2-valve head 1)."""
        return max(1, self.num_valves_per_cyl // 2)

    # --- Variable valve timing (resolved core) ---
    # Cam advance in crank degrees (+ = valve events earlier). The ECU moves each cam
    # within [min, max] at up to vvt_rate_deg_per_s; with min = max = 0 (the default) the
    # cam is fixed. vvt_auto schedules the target from speed and load: intake advance is
    # held at its minimum at idle (least overlap, steadiest burn), is brought in at
    # mid speeds and light-to-full load (earlier intake closing = more low-end torque), and
    # is backed out toward high rpm (later closing = top-end breathing). A driver or test
    # can override either cam with EngineInput.vvt_intake_deg / vvt_exhaust_deg.
    vvt_intake_min_deg: float = 0.0
    vvt_intake_max_deg: float = 0.0
    vvt_exhaust_min_deg: float = 0.0
    vvt_exhaust_max_deg: float = 0.0
    vvt_rate_deg_per_s: float = 80.0
    vvt_auto: bool = True

    # --- Exhaust backpressure (physical, not sound-only) ---
    # The exhaust valve sees a plenum pressure, not ambient: the plenum fills
    # from the cylinders and drains through the tail pipe orifice, so backpressure
    # rises with flow and with restriction and falls as the pipe empties.
    exhaust_plenum_volume_m3: float = 15.0e-3
    exhaust_tail_dia_m: float = 45.0e-3
    exhaust_tail_discharge_coeff: float = 0.70

    # --- Resolved (crank-angle) engine core ---
    # False: the cycle-averaged model (fast; torque from the air-standard Otto
    # cycle). True: every cylinder's gas is integrated over crank angle and the
    # torque, manifold pressure, exhaust backpressure, volumetric efficiency and
    # wall heat loss all emerge from that (see README §2.3). SI engines only.
    use_resolved_cylinders: bool = False
    # Idle spark trim (resolved core): at closed throttle the spark is moved by this
    # many degrees per rpm of speed error — the fast inner loop of idle control,
    # because spark changes torque instantly while the idle-air valve has to wait
    # for the manifold to fill. Positive error (below idle) advances the spark.
    idle_spark_gain_deg_per_rpm: float = 0.12
    idle_spark_reserve_deg: float = 8.0         # idle spark is held this far back from MBT so the trim can add torque
    idle_spark_max_advance_deg: float = 16.0
    idle_spark_max_retard_deg: float = 25.0
    # Knock control in the resolved core (needs use_knock_model): each detected
    # knock retards the spark by this much, which then recovers with time.
    knock_retard_step_deg: float = 3.0
    knock_retard_max_deg: float = 15.0
    knock_retard_recovery_deg_per_s: float = 1.5

    # --- Crank-angle resolution of the resolved solver ---
    # The cylinder state is integrated on substeps of at most this many crank
    # degrees. Cost is (substeps × cylinders) per physics step, so it is capped
    # two ways: max_substeps_per_update bounds the substeps outright, and
    # max_cylinder_steps_per_ms bounds substeps × cylinders per millisecond of
    # simulated time (so it means the same at any physics step) — an 8-cylinder
    # engine therefore takes coarser substeps at speed than a 4-cylinder, and
    # every engine costs about the same per physics step. Excess angular travel
    # per update is still covered, just in coarser substeps than requested (the
    # reversible-work formulation keeps torque within ~5 % down to ~9° substeps).
    crank_angle_resolution_deg: float = 4.0
    max_substeps_per_update: int = 64
    max_cylinder_steps_per_ms: int = 32
    # Compiled inner loop (engine_sim/native_core.py). When a C compiler is available the
    # substep loop runs ~30× faster, so the budget and the resolution follow it: finer
    # substeps (a 1° default) and a much larger cylinder-step budget. With no compiler, or
    # use_native_core = false, the Python values above apply.
    use_native_core: bool = True
    native_crank_angle_resolution_deg: float = 1.0
    native_steps_per_ms: int = 600

    # --- Exhaust system (sound). Lists are per exhaust pipe: a single value
    # applies to all pipes; "dual" = one pipe per bank, "single" = all banks merge.
    exhaust_layout: str = "single"
    exhaust_header_length_m: List[float] = field(default_factory=lambda: [0.55])
    exhaust_mid_length_m: List[float] = field(default_factory=lambda: [1.10])
    exhaust_muffler_length_m: List[float] = field(default_factory=lambda: [0.40])
    exhaust_tail_length_m: List[float] = field(default_factory=lambda: [0.90])
    exhaust_muffler_area_ratio: float = 5.0      # muffler chamber / pipe cross-section
    exhaust_muffler_body_hz: List[float] = field(default_factory=lambda: [125.0])
    # Gas temperature in each section [K]; speed of sound c = 20.05·√T
    exhaust_header_temp_k: float = 725.0
    exhaust_mid_temp_k: float = 622.0
    exhaust_muffler_temp_k: float = 526.0
    exhaust_tail_temp_k: float = 439.0
    exhaust_pipe_loss: float = 0.992             # wall friction: gain per pass
    exhaust_tip_damping: float = 0.35            # open-end high-frequency loss (0..1): higher = duller

    # --- Intake (sound) ---
    intake_runner_length_m: float = 0.45
    intake_runner_temp_k: float = 293.0

    # --- Sound character (sound). These are the only free "taste" parameters;
    # everything else the sound does follows from the sections above. ---
    sound_cylinder_strength_spread: float = 0.30   # fixed pulse-strength differences between cylinders (idle lope)
    sound_cylinder_timing_spread_deg: float = 6.0  # fixed exhaust-timing differences between cylinders
    sound_noise_seed: int = 1234567                # seed for those fixed differences
    sound_blowdown_k: float = 1.75                 # blowdown time constant = k·bore / c(header)
    sound_crank_ripple_gain: float = 0.66          # share of cylinder work seen as crank-speed ripple
    sound_combustion_variability: float = 0.02     # random cycle-to-cycle strength variation (fraction)
    sound_idle_variability: float = 0.05           # extra variation at closed throttle (fraction)
    sound_pop_probability: float = 0.03            # chance of an overrun pop per exhaust event
    sound_starter_ratio: float = 11.0              # starter armature revs per crank rev
    sound_starter_poles: int = 4                   # starter motor poles (whine harmonic)

    # --- Derived layout helpers ---
    @property
    def fire_angles_deg(self) -> List[float]:
        """Cycle angle [0, 720) of each cylinder's firing TDC, indexed by cylinder
        number - 1, from firing_order and firing_intervals_deg."""
        n = self.num_cylinders
        ivs = self.firing_intervals_deg or [720.0 / n] * n
        out = [0.0] * n
        a = 0.0
        for k, cyl in enumerate(self.firing_order):
            out[cyl - 1] = a % 720.0
            a += ivs[k]
        return out

    @property
    def intake_lift_window(self) -> tuple:
        """(open, duration) of the intake valve in cycle degrees (0 = TDC end of exhaust)."""
        return ((720.0 - self.intake_open_btdc_deg) % 720.0,
                180.0 + self.intake_open_btdc_deg + self.intake_close_abdc_deg)

    @property
    def exhaust_lift_window(self) -> tuple:
        """(open, duration) of the exhaust valve in cycle degrees; 360 = firing TDC."""
        return (540.0 - self.exhaust_open_bbdc_deg,
                180.0 + self.exhaust_open_bbdc_deg + self.exhaust_close_atdc_deg)

    # --- Torque curve (retained for backward compat; superseded by thermo model) ---
    torque_curve: List[TorquePoint] = field(default_factory=lambda: [
        TorquePoint(500, 60.0), TorquePoint(1000, 100.0),
        TorquePoint(1500, 130.0), TorquePoint(2000, 155.0),
        TorquePoint(2500, 170.0), TorquePoint(3000, 180.0),
        TorquePoint(3500, 185.0), TorquePoint(4000, 182.0),
        TorquePoint(4500, 175.0), TorquePoint(5000, 165.0),
        TorquePoint(5500, 152.0), TorquePoint(6000, 138.0),
        TorquePoint(6500, 120.0), TorquePoint(7000, 100.0),
    ])

    def torque_at_rpm(self, rpm: float) -> float:
        """Interpolate WOT torque from configured curve (backward compat)."""
        if not self.torque_curve:
            return 0.0
        sorted_curve = sorted(self.torque_curve, key=lambda p: p.rpm)
        if rpm <= sorted_curve[0].rpm:
            if len(sorted_curve) >= 2:
                p0, p1 = sorted_curve[0], sorted_curve[1]
                slope = (p1.torque - p0.torque) / (p1.rpm - p0.rpm) if p1.rpm != p0.rpm else 0.0
                return p0.torque + slope * (rpm - p0.rpm)
            return sorted_curve[0].torque
        if rpm >= sorted_curve[-1].rpm:
            if len(sorted_curve) >= 2:
                p0, p1 = sorted_curve[-2], sorted_curve[-1]
                slope = (p1.torque - p0.torque) / (p1.rpm - p0.rpm) if p1.rpm != p0.rpm else 0.0
                return p1.torque + slope * (rpm - sorted_curve[-1].rpm)
            return sorted_curve[-1].torque
        for i in range(len(sorted_curve) - 1):
            p0, p1 = sorted_curve[i], sorted_curve[i + 1]
            if p0.rpm <= rpm <= p1.rpm:
                t = (rpm - p0.rpm) / (p1.rpm - p0.rpm)
                return p0.torque + t * (p1.torque - p0.torque)
        return sorted_curve[-1].torque

    # --- Friction model (Source: KB Part XI, Eq 13.5-13.7) ---
    # KB Eq 13.6 (SI): tfmep(bar) = 0.97 + 0.15*(N/1000) + 0.05*(N/1000)^2
    # where N = engine speed in rev/MIN (ENGINE_KB.md §13.5.1). The model
    # evaluates it with N in rev/s, so with N_rpm = 60·N_rps:
    #   C1 = 0.97 bar                         = 97000 Pa
    #   C2 = 0.15 bar·60/1000 per rev/s       = 900 Pa per rev/s
    #   C3 = 0.05 bar·(60/1000)^2 per (rev/s)² = 18 Pa per (rev/s)^2
    # → 3.67 bar at 6000 rpm, as Heywood. (Before v0.4 these were converted
    # as if N were rev/s — 15 and 0.005 — making friction ~flat with speed:
    # 1.0 bar at 6000 rpm, so engines free-revved far too easily.)
    friction_C1_Pa: float = 97_000.0
    friction_C2_Pa_per_rev_s: float = 900.0
    friction_C3_Pa_per_rev_s2: float = 18.0
    exhaust_back_pressure_pa: float = 103_000.0

    def friction_torque(self, omega: float) -> float:
        """Friction torque from rubbing friction mep (KB Eq 13.5)."""
        N_rev_s = omega / (2.0 * math.pi)
        tfmep = (self.friction_C1_Pa
                 + self.friction_C2_Pa_per_rev_s * N_rev_s
                 + self.friction_C3_Pa_per_rev_s2 * N_rev_s ** 2)
        return max(tfmep * self.total_displacement / (4.0 * math.pi), 0.0)

    # --- Air system ---
    ambient_pressure_pa: float = 101_325.0
    intake_temperature_k: float = 293.15   # 20°C
    max_volumetric_efficiency: float = 0.92

    # --- Fuel system ---
    stoich_afr: float = 14.7       # gasoline (KB Table D.4)
    lower_heating_value: float = 44.0e6  # J/kg (KB Table D.4)
    fuel_density: float = 750.0    # kg/m^3

    # --- Combustion ---
    combustion_efficiency: float = 0.95

    # --- Spark timing (KB §9.2.3, §15.3.1) ---
    spark_timing_deg_btdc: float = 25.0  # degrees before TDC
    use_mbt_timing: bool = True  # auto-compute MBT if True
    fuel_octane_ron: float = 91.0

    # --- Knock (KB Eq 9.60-9.64, Douaud-Eyzat correlation) ---
    use_knock_model: bool = False
    knock_torque_retard_factor: float = 0.85  # fractional torque retained under knock

    # --- Combustion stability (resolved core) ---
    # A real flame fails to propagate reliably in a charge that is too dilute (exhaust
    # residual), too lean or too rich: the burn slows, goes partial, and at the stability
    # limit the cylinder misfires. Burn quality q falls smoothly from 1 to 0 between
    # each pair of limits (residual mass fraction at the spark; lambda lean; lambda
    # rich), the lowest of the three applies, the released energy scales with q, and
    # below burn_misfire_quality the cycle does not burn at all.
    burn_residual_lo: float = 0.14
    burn_residual_hi: float = 0.44
    burn_lean_lo: float = 1.30
    burn_lean_hi: float = 1.75
    burn_rich_lo: float = 0.80
    burn_rich_hi: float = 0.50
    burn_misfire_quality: float = 0.15
    # Solve the burn as two zones (unburned end gas / burned products) at a common
    # pressure: gives a physical end-gas temperature for knock and the temperature NO
    # forms at. False keeps the single mean-temperature gas throughout.
    use_two_zone: bool = True

    # --- Combustion model ---
    wiebe_a: float = 5.0       # Wiebe efficiency parameter
    wiebe_m: float = 2.0       # Wiebe shape factor (KB Eq 9.28)
    combustion_duration_deg: float = 60.0  # total Δθ at reference conditions

    # --- Manifold dynamics (KB Eq 7.21-7.22) ---
    manifold_volume_m3: float = 0.003   # plenum volume [m^3]
    use_manifold_dynamics: bool = False  # True = use filling/emptying ODE

    # --- Throttle (KB Eq 7.18-7.20) ---
    throttle_max_area_m2: float = 0.003  # effective area at WOT [m^2]
    throttle_discharge_coeff: float = 0.65

    # --- Heat transfer (KB Eq 12.14-12.19) ---
    use_heat_transfer: bool = False
    heat_transfer_model: str = "woschni"  # "woschni" or "annand"
    wall_temperature_k: float = 400.0     # mean wall temperature [K]

    # --- Fuel film (KB Eq 7.25-7.27) ---
    use_fuel_film: bool = False
    fuel_film_x: float = 0.3       # fraction going to wall film
    fuel_film_tau: float = 0.5     # evaporation time constant [s]

    # --- Lambda control (KB Eq 7.15-7.17) ---
    use_lambda_controller: bool = False
    lambda_target: float = 1.0
    lambda_integrator_gain: float = 2.0
    lambda_transport_delay_s: float = 0.05

    # --- Cyclic variability (KB Eq 9.50-9.51) ---
    use_cyclic_variability: bool = False
    base_cov_imep: float = 2.0  # percent

    # --- Turbocharging (KB Eq 6.32-6.49) ---
    use_turbo: bool = False
    turbo_compressor_efficiency: float = 0.72
    turbo_turbine_efficiency: float = 0.78
    turbo_target_boost_pa: float = 150000.0  # target boost [Pa]
    turbo_wastegate_gain: float = 0.5
    # Resolved core (turbo_system.py): the turbo is a set of real components. Twin-turbo
    # engines are one turbo with the flow, area and inertia of the pair (turbo_count).
    turbo_count: int = 1
    turbo_inertia_kgm2: float = 1.2e-4             # shaft + wheels, per turbo
    turbo_design_speed_rps: float = 2200.0         # compressor map design speed [rev/s] (~130 krpm)
    turbo_design_flow_kgs: float = 0.14            # per turbo, at the design point
    turbo_design_pressure_ratio: float = 2.4
    turbo_mechanical_efficiency: float = 0.95
    turbo_turbine_area_m2: float = 5.0e-4          # effective flow area of the turbine stage (all turbos)
    turbo_wastegate_area_m2: float = 3.5e-4        # fully open
    turbo_boost_volume_m3: float = 8.0e-3          # intercooler + charge piping
    turbo_intercooler_effectiveness: float = 0.7
    exhaust_post_volume_m3: float = 8.0e-3         # downpipe, catalyst and muffler volume behind the turbine

    # --- Emissions (KB Part IX) ---
    compute_emissions: bool = False
    fuel_cetane_number: float = 50.0  # for diesel mode

    # --- Diesel/CI mode (KB Part VIII) ---
    engine_type: str = "SI"  # "SI" or "CI"
    diesel_injection_rate_kg_s: float = 0.001
    diesel_stoich_afr: float = 32.0

    # --- Friction breakdown (KB Part XI) ---
    use_friction_breakdown: bool = False  # True = use component model
    num_rings: int = 3
    ring_tension_kpa: float = 14.0
    bearing_clearance_ratio: float = 0.001
    num_valves_per_cyl: int = 4

    # --- Conversion constant ---
    RPM_TO_RADS: float = 2.0 * math.pi / 60.0

    # --- Legacy alias ---
    @property
    def displacement_m3_legacy(self) -> float:
        return self.total_displacement


def load_config(path: str = None) -> EngineConfig:
    """Load an engine configuration.

    A ``.toml`` path is an engine definition file (config/engines/*.toml, see
    README.md §3; no extra packages needed). Anything else is read as the legacy
    YAML file, which needs PyYAML; if it cannot be read a warning says so and the
    defaults are returned (it used to fail silently).
    """
    config = EngineConfig()
    if path is None:
        return config
    if str(path).endswith('.toml'):
        from .engine_file import load_engine_file
        return load_engine_file(path).engine
    import warnings
    try:
        import yaml
    except ImportError:
        warnings.warn(f"PyYAML is not installed, so {path} was NOT read and the default engine is used. "
                      "Engines are defined in config/engines/*.toml, which needs no extra packages.",
                      stacklevel=2)
        return config
    try:
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
    except (FileNotFoundError, OSError) as e:
        warnings.warn(f"Could not read {path} ({e}); using the default engine.", stacklevel=2)
        return config
    if not data or 'engine' not in data:
        return config
    eng = data['engine']
    # Scalar parameters
    for key in ('name', 'bore_m', 'stroke_m', 'rod_length_m',
                'compression_ratio', 'num_cylinders',
                'engine_inertia_kgm2', 'load_inertia_kgm2',
                'idle_rpm', 'redline_rpm', 'limiter_type',
                'limiter_hysteresis_rpm',
                'ambient_pressure_pa', 'intake_temperature_k',
                'max_volumetric_efficiency',
                'stoich_afr', 'lower_heating_value', 'fuel_density',
                'combustion_efficiency'):
        if key in eng:
            setattr(config, key, eng[key])
    # Legacy
    if 'displacement_liters' in eng:
        pass  # ignore, we use geometry now
    fric = eng.get('friction', {})
    for key in ('C1_Pa', 'C2_Pa_per_rev_s', 'C3_Pa_per_rev_s2'):
        if key in fric:
            setattr(config, 'friction_' + key, fric[key])
    tc = eng.get('torque_curve', [])
    if tc:
        config.torque_curve = [TorquePoint(rp['rpm'], rp['torque']) for rp in tc]
    return config
