"""Debug UI — text-based engine diagnostics panel.

Source: ENGINE_KB.md §25 (Debug UI requirements).
"""

from __future__ import annotations

from .engine import EngineOutput


def format_debug_panel(output: EngineOutput) -> str:
    lines = [
        "ENGINE",
        "─" * 40,
        f"  RPM                  {output.rpm:8.1f}",
        f"  Throttle             {output.throttle*100:7.1f} %",
        f"  Crank angle          {output.crank_angle_deg:8.1f} deg",
        f"  Stroke phase         {['Intake','Compress','Power','Exhaust'][output.stroke_phase]}",
        "─" * 40,
        f"  Indicated torque     {output.indicated_torque:8.1f} Nm",
        f"  Friction torque      {output.friction_torque:8.1f} Nm",
        f"  Pumping torque       {output.pumping_torque:8.1f} Nm",
        f"  Accessory torque     {output.accessory_torque:8.1f} Nm",
        f"  Load torque          {output.load_torque:8.1f} Nm",
        f"  Net torque           {output.net_torque:8.1f} Nm",
        "─" * 40,
        f"  Intake pressure      {output.intake_pressure/1000:8.1f} kPa",
        f"  Intake temperature   {output.intake_temperature:8.1f} K",
        f"  Air mass/cycle       {output.air_mass_per_cycle*1000:8.3f} g",
        f"  Fuel mass/cycle      {output.fuel_mass_per_cycle*1000:8.4f} g",
        f"  Volumetric eff.      {output.volumetric_efficiency*100:8.1f} %",
        f"  AFR                  {output.afr:8.2f}",
        f"  Lambda               {output.lambda_val:8.3f}",
        "─" * 40,
        f"  Cyl pressure (peak)  {output.cyl_pressure/1000:8.1f} kPa",
        f"  Cyl temp (peak)      {output.cyl_temperature:8.1f} K",
        f"  Thermal efficiency   {output.thermal_efficiency*100:8.1f} %",
        f"  Cycle work           {output.cycle_work:8.2f} J",
        "─" * 40,
        f"  Angular velocity     {output.angular_velocity:8.2f} rad/s",
        f"  Angular accel        {output.angular_acceleration:8.2f} rad/s2",
        f"  Power (brake)        {output.power/1000:8.2f} kW",
        f"  Limiter              {'ON' if output.limiter_active else 'OFF':>8s}",
        "─" * 40,
    ]
    return "\n".join(lines)


def format_torque_balance(output: EngineOutput) -> str:
    return (
        f"Torque balance:\n"
        f"  Indicated torque:    {output.indicated_torque:8.2f} Nm\n"
        f"  - Friction:          {output.friction_torque:8.2f} Nm\n"
        f"  - Pumping:           {output.pumping_torque:8.2f} Nm\n"
        f"  - Accessories:       {output.accessory_torque:8.2f} Nm\n"
        f"  - Load:              {output.load_torque:8.2f} Nm\n"
        f"  = Net:               {output.net_torque:8.2f} Nm\n"
        f"  Net / Inertia = α:   {output.angular_acceleration:8.2f} rad/s2"
    )
