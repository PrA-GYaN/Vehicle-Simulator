"""Cam profiles — valve lift versus crank angle.

A cam profile turns the engine file's open/close angles plus a maximum lift and
a pair of ramp fractions into a lift curve, and that curve is the single source
of truth for both the physics (it sets the flow area, hence the mass flow) and
the animation (the dashboard reads the same numbers from /api/config).

Conventions. Angles are *cycle* degrees in the 0..720 frame the rest of the
codebase uses: 0 = TDC at the start of the intake stroke, 360 = firing TDC.
Intake opens a few degrees before 0 and closes in the compression stroke;
exhaust opens in the expansion stroke and closes just after 0. A profile is
therefore a wrapped window on the 720° circle, not an interval.

Profile shape. Within the event window the lift rises from 0 to `max_lift_m`
over the first `ramp_open_frac` of the window and falls back over the rest,
using the same raised-cosine on each side:

    lift/L = ½·(1 − cos(π·s/f))          while s < f
    lift/L = ½·(1 + cos(π·(s−f)/(1−f)))  while s ≥ f

f = 0.5 reproduces the sin²(π·s) hump used before, so an engine file that sets
no ramp parameters gets the profile the dashboard has always drawn. f ≠ 0.5 is
asymmetric: a small f means a fast-opening, slow-closing (late, high-lift) ramp,
which is how a race cam differs from a road cam. Both ramps are configurable
independently (`ramp_open_frac`, `ramp_close_frac`); the peak sits at the
fraction where opening ends.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Tuple


@dataclass(frozen=True)
class ValveProfile:
    """One valve's lift profile over the 720° cycle.

    Attributes:
        open_deg: Crank angle at which lift starts rising [cycle degrees].
        duration_deg: Total event duration (opening + closing) [cycle degrees].
        max_lift_m: Peak lift [m].
        ramp_open_frac: Fraction of the event spent opening (0..1).
        ramp_close_frac: Fraction of the event spent closing (0..1).
    """
    open_deg: float
    duration_deg: float
    max_lift_m: float = 0.0095
    ramp_open_frac: float = 0.5
    ramp_close_frac: float = 0.5

    def __post_init__(self):
        # The two ramps must tile the event: if they do not add up to 1 the
        # closing cosine never reaches zero (or overshoots it) at the end of the
        # window and the valve snaps shut.
        if not (0.0 < self.ramp_open_frac < 1.0
                and abs(self.ramp_open_frac + self.ramp_close_frac - 1.0) < 1e-6):
            raise ValueError(
                "ValveProfile ramp fractions must satisfy 0 < ramp_open_frac < 1 and "
                f"ramp_open_frac + ramp_close_frac = 1, got {self.ramp_open_frac} and "
                f"{self.ramp_close_frac}")

    @property
    def close_deg(self) -> float:
        """Crank angle at which the valve is fully seated [cycle degrees]."""
        return (self.open_deg + self.duration_deg) % 720.0

    @property
    def peak_deg(self) -> float:
        """Crank angle of peak lift [cycle degrees]."""
        return (self.open_deg + self.ramp_open_frac * self.duration_deg) % 720.0

    def lift(self, angle_deg: float) -> float:
        """Valve lift at a crank angle [m].

        Args:
            angle_deg: Crank angle in the 0..720 cycle frame [degrees].

        Returns:
            Lift above the seat [m]; 0 when the valve is seated.
        """
        if self.duration_deg <= 0.0 or self.max_lift_m <= 0.0:
            return 0.0
        x = (angle_deg - self.open_deg) % 720.0
        if x >= self.duration_deg:
            return 0.0
        s = x / self.duration_deg
        if s < self.ramp_open_frac:
            return self.max_lift_m * 0.5 * (1.0 - math.cos(math.pi * s / self.ramp_open_frac))
        return self.max_lift_m * 0.5 * (
            1.0 + math.cos(math.pi * (s - self.ramp_open_frac) / self.ramp_close_frac))

    def is_open(self, angle_deg: float) -> bool:
        """True when the valve is off its seat at this angle."""
        if self.duration_deg <= 0.0 or self.max_lift_m <= 0.0:
            return False
        return ((angle_deg - self.open_deg) % 720.0) < self.duration_deg


def lift_table(profile: ValveProfile, step_deg: float = 5.0) -> List[float]:
    """Lift sampled across the whole 720° cycle — for the dashboard plot.

    Sampling the same profile object the physics evaluates means the valve
    trace on screen is the flow area the model actually used, not a second
    curve that can drift from it.
    """
    n = int(720.0 / step_deg) + 1
    return [profile.lift(i * step_deg) for i in range(n)]


def valve_profiles_from_config(cfg) -> Tuple[ValveProfile, ValveProfile]:
    """Build (intake, exhaust) profiles from an EngineConfig.

    The open/close angles and durations come from the engine file's [valves]
    section (via the config's existing `intake_lift_window` /
    `exhaust_lift_window`); the lifts and ramp fractions come from the new
    per-valve cam fields.
    """
    io, idur = cfg.intake_lift_window
    eo, edur = cfg.exhaust_lift_window
    return (
        ValveProfile(
            open_deg=io,
            duration_deg=idur,
            max_lift_m=cfg.intake_max_lift_m,
            ramp_open_frac=cfg.intake_ramp_open_frac,
            ramp_close_frac=cfg.intake_ramp_close_frac,
        ),
        ValveProfile(
            open_deg=eo,
            duration_deg=edur,
            max_lift_m=cfg.exhaust_max_lift_m,
            ramp_open_frac=cfg.exhaust_ramp_open_frac,
            ramp_close_frac=cfg.exhaust_ramp_close_frac,
        ),
    )