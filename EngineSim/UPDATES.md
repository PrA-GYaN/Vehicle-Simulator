# Resolved Engine Model — Review, Integration and Status

**Document:** `UPDATES.md`
**Subject:** crank-angle-resolved four-stroke engine model (`use_resolved_cylinders`)
**Repository:** `/data/Repos/REAL_CAR/EngineSim`
**Test status:** `207 passed` (v0.5: 147; v0.6 added 10; v0.7 added 50: the turbo components, two-zone burn, VVT and burn stability (27) and the compiled-core parity and engine tests (23))
**Compared against:** `EngineSim_backup_before_Improv/` (the state before the resolved modules were added)

This revises the earlier version of this file (which recorded the resolved model as built but *not*
wired into the engine). That version was reviewed, found to contain real defects and one wrong
diagnosis, and the work was then completed: the resolved core now drives the engine when switched on.

---

## 0a. v0.7 — turbo in the exhaust path, two-zone burn, VVT, misfire, and a compiled core

**Goal:** close the five open items of the v0.6 list. All five are implemented, tested and documented;
the last one (speed) is a compiled C inner loop that reproduces the Python one to rounding.

### 0a.1 What was done
| Item | Implementation | Where |
|---|---|---|
| **Turbine in the resolved exhaust path** | Exhaust plenum → (turbine ‖ wastegate) → post-turbine volume → tail pipe. The turbine is an orifice (Cd 0.8) that also takes `Ẇ = η·ṁ·c_p·T₃·[1 − (p₄/p₃)^((γ−1)/γ)]` out of the gas; the wastegate (Cd 0.7 × opening) passes flow and no power. The compressor is a speed-line map, the shaft obeys `½·I·dω²/dt = η_m·Ẇ_T − Ẇ_C`, the charge-air volume is fed by the compressor and drained by the throttle, with a check-valve path round an unspun compressor and a blow-off valve. The wastegate is held by a PI law (see defect 2) | `exhaust_system.py` (`_update_with_turbine`), new `turbo_system.py`, `cylinder_bank.py` |
| **Two-zone thermodynamics** | Unburned and burned zones at a common pressure while a flame burns with the valves shut: enthalpy-balance flame temperature, mixing of new products into the burned zone at constant energy, isentropic compression/expansion of both zones to the common pressure (two sweeps, c_v at the mean temperature), work from the first law (`U_before + Q − Q_wall − U_after`). The end-gas temperature drives the knock integral; the burned-zone temperature drives NO (extended Zeldovich, per cylinder). Zones merge (energy-conserving) if a valve opens with a flame still burning | `cylinder.py` (`_two_zone_step`, `_merge_zones`) |
| **Variable valve timing** | `set_cam_phase(intake_adv, exhaust_adv)` shifts the table index of the lift/area tables, so events, overlap, intake closing and the knock window follow. ECU schedule from speed/load with a slew rate (`vvt_*`), `Controls.vvt_override` | `cylinder.py`, `cylinder_bank.py`, `engine.py`, `powertrain.py` |
| **Misfire / partial-burn limit** | Burn quality `q` = min of three smoothsteps (residual fraction, λ lean, λ rich) scales the released energy; `q <` `burn_misfire_quality` ⇒ no burn. Overlap reversion now sends *burned* gas up the intake runner into a buffer that is drawn back first, so the residual is physical. Idle MAP rises from ~19 to ~23 kPa with 16 % residual | `cylinder.py` (`_burn_quality`, `_runner_burned`), `config.py` |
| **Speed: compiled inner loop** | `native/enginecore.c`: `Cylinder.step`, `begin_cycle`, the exhaust/turbine stage, boost plenum, throttle and manifold ODE, transcribed line for line; `native_core.NativeCylinderBank` packs the cylinders into flat arrays, calls it once per engine step and writes the state back. Built on first use with `-O2 -ffp-contract=off` into a per-user cache keyed on the source hash, with a silent fall-back to Python | `native_core.py`, `native/enginecore.c`, `engine.py` (`_make_bank`) |

### 0a.2 Defects found along the way
| # | Defect | Effect / fix |
|---|---|---|
| 1 | Two-zone first version: flame temperature from a u-based balance referenced to p₀ | 15 % too much work. Now an enthalpy balance for T_f and a ΔU-based work term; closure test against the single zone |
| 2 | **Wastegate hunted**: the old law integrated the error upward and closed at a fixed slow rate | Boost swung ±15 kPa on a 1.5 s cycle, so every dyno number depended on the phase it was sampled at. Now PI (`kp 6`, `ki 3` × `turbo_wastegate_gain`, clamped integral): boost within ~3 % of target at every speed |
| 3 | **ECU air model dragged to its clamps by the boost transient** | Trapped air on a boosted engine also depends on exhaust backpressure, which the speed-density model cannot see; fitting through the wastegate transient pinned `a₀ = 1.5, a₁ = −0.8` and leaned the V6 to λ 1.4–1.9 above 5000 rpm. RLS learning is now gated to quasi-steady MAP (|MAP − its 0.15 s average| < 2 %) on boosted engines; λ = 1.00 at every point |
| 4 | Misfire limits stalled the 454 and the VR38 on the starter | Limits relaxed (residual 0.14/0.44), burn duration clamped (≤ 75°) and spark (≤ 35°), idle spark reserve 8° (gain 0.12), per-engine idle feed-forward (1NZ 0.006, 454 0.009, VR38 0.009) |
| 5 | Decel fuel cut fired after a bump start | The post-start delay only applies once the starter has started the engine |
| 6 | Governor recovery too slow for the compiled core's larger budget (+2 per 12 steps from a trimmed 600) | Now +⅛ of the current budget (min 2) per 12 steps |
| 7 | VVT tests expected the wrong direction at 6000 rpm | Optimum advance with the default cam is ≈ 15°; a late-closing cam shows the low-speed gain; 45° over-advance hurts high speed (documented, tests adapted) |

### 0a.3 Compiled-core design and verification
- **State sharing.** The `Cylinder` objects stay the source of truth. Before the call each one is packed into one row of a flat `double` array (86 fields: 81 plain attributes plus the fuel-table choice, the inlet-enthalpy cache and the two `None` sentinels), the C loop works in place, and afterwards the attributes are written back with `__dict__.update`, so every Python reader and the ECU code are unchanged and a bank can be handed back to Python mid-run (tested). The layout header (`F_*`, `K_*`, `B_*`) is generated from the Python lists, so the two sides cannot drift.
- **Parity.** `tests/test_native_core.py` runs the same drive through both banks (same substeps and budget) and compares 8 outputs per step and every cylinder's state. Scenarios: three speeds, motoring then fuel resume, WOT with two-zone and knock, single zone, misfire and runner buffer, cam phasing, the three shipped engines, the turbo, per-cylinder energy scale. Agreement is bit-exact on most (tolerance 1e-9, 1e-8 on the turbo).
- **Cost.** The C loop is ≈ 0.6 µs per cylinder substep (16 µs in Python); the pack/unpack costs ≈ 30 µs per cylinder per engine step. 8 cylinders at 6000 rpm: 1.5 ms → 0.8 ms per 2 ms step at the *same* 4°-class substeps, and 0.6 ms at 64 substeps (≈ 1.1°).
- **Resolution.** With the compiled core the defaults become 1° and 600 cylinder-steps per ms (`native_crank_angle_resolution_deg`, `native_steps_per_ms`), against 4° / 32 for Python.

`run_benchmark.py --dt 0.002` (CPU time, other programs running, ±15 %), full-throttle points idle → 95 % redline:

| Engine | compiled (1° substeps) | pure Python (governed) |
|---|---|---|
| 1NZ I4 | 4.9 / 5.0 / 4.2 / 3.5 / 3.3× | 4.6 / 2.7 / 1.6 / 1.1 / 1.0× |
| 454 V8 | 3.9 / 3.8 / 3.1 / 2.6 / 2.2× | 3.5 / 1.7 / 1.1 / 1.1 / 1.0× |
| VR38 V6 turbo | 4.1 / 3.8 / 3.3 / 2.3 / 2.6× | 3.3 / 1.7 / 1.1 / 1.0 / 1.1× |

Remaining cost at 6000 rpm on the V8 (per 2 ms step, ≈ 1.1 ms): the C loop ≈ 0.33 ms, copying state in and out ≈ 0.25 ms, Python glue (`Engine.update`, the ECU, the bank prologue) the rest.

### 0a.4 Verification
Single cylinder, 3000 rpm, 0.91 bar, 1°, two-zone: MBT 25° (41.9 N·m indicated, 52.6 bar, burned-zone 2806 K, η_ind 35.9 %); single-zone gives the same torque within 1–3 % and ≈ 10 % higher peak pressure.
WOT (compiled core, 1°): 1NZ 125 / 112 / 97 / 77 N·m at 1600 / 3200 / 4800 / 6100 rpm; 454 582 / 509 / 402 / 283 N·m at 1400 / 2800 / 4100 / 5200; VR38 (turbo, 200 kPa target) 663 / 570 / 485 / 391 N·m at 1800 / 3500 / 5200 / 6600 rpm with exhaust 158 → 224 kPa against 102 → 112 kPa post-turbine. λ 1.00 everywhere. Cold start then idle (35 s): 1NZ 801 ± 17, 454 661 ± 6, VR38 751 ± 12 rpm, all ≈ 23 kPa, no misfires.

### 0a.5 New / changed files
New: `engine_sim/turbo_system.py`, `engine_sim/native_core.py`, `engine_sim/native/enginecore.c`, `tests/test_resolved_v07.py`, `tests/test_native_core.py`.
Substantially changed: `cylinder.py` (two-zone, VVT, burn quality, runner buffer, NO), `cylinder_bank.py` (turbo integration, `_substeps` split out, learning gate, governor recovery), `exhaust_system.py` (turbine stage), `engine.py` (VVT schedule, bank selection, idle/DFCO changes, new outputs), `config.py`, `powertrain.py`, the three engine files (`use_native_core`, VR38 turbo block, VVT keys, per-engine idle), `interactive_dashboard.py` / `dashboard/app.js` (core banner, boost gauge, VVT-shifted lift), `run_benchmark.py` (`--python`), and `tests/test_resolved_engine.py` (`_cfg` pins the Python core).
Config: `use_native_core`, `native_crank_angle_resolution_deg`, `native_steps_per_ms`, `use_two_zone`, `burn_*`, `vvt_*`, `turbo_*` components, `exhaust_post_volume_m3`, `idle_spark_*` / `fast_idle_*` / `dfco_after_start_delay_s` (v0.6), knock retard keys.

---

## 0. v0.6 — real-time speed, and the physics it exposed

**Goal:** make the resolved core (now on in all three shipped engines) run in real time, and fix whatever
else was needed. Result: it runs at ≈ 1× near redline on this (loaded) machine at the nominal cap and is
held to the clock by a governor; and reading the hot path line by line turned up a defect in the heat release
that had been distorting every number since v0.5.

### 0.1 Speed
| Change | Effect |
|---|---|
| `Cylinder.step` rewritten table-driven: volume (0.25°), valve area/lift (0.5°), c_v and u (20 K) from lists; Woschni, orifice flow and Wiebe inlined; edge-volume reuse; hoisted constants | ≈ 25 → ≈ 17 µs per substep on average |
| Quiet-substep short path (valves shut, no burn due) | ≈ 13 µs vs ≈ 25 µs through the gas exchange |
| Woschni `h_c` refreshed every other quiet substep; second-order flow corrector only when a substep moves > 2 % of the charge | small, no measurable accuracy change |
| Bank loop: local-angle bookkeeping, begin-cycle substep precomputed, running-total work/heat (no per-update resets or finalize), summaries every 8 updates, manifold ODE and throttle flow inlined | per-update overhead roughly halved |
| `max_cylinder_steps_per_ms` (32): cap on substeps × cylinders per simulated millisecond | every engine costs about the same per unit time; coarser substeps at speed on bigger engines |
| Wall-clock governor (`CylinderBank.set_time_budget`, dashboard) | trims the cap down to 2 substeps/cylinder when over budget, restores it with headroom |
| Dashboard physics step 2 ms for the resolved engine (cap is per ms, so same cylinder cost) | ≈ 20–25 % lower total cost, idle and WOT unchanged |

Accuracy of coarse substeps (single cylinder, 6000 rpm): net torque +5 %, peak pressure −11 % from 1° to 12°;
whole engine at the 8-cylinder cap (≈ 8° substeps at 4500 rpm) torque +5 %, PMEP −15 %. The second-order flow
corrector and the exact reversible work are what keep it that small.

`run_benchmark.py` (CPU-time based, so other programs matter less, but still ±20 % here because a browser and
the dashboard were running): nominal cap ≈ 2.5–3× at idle falling to ≈ 1.0–1.2× at redline on the 1NZ and
≈ 0.8–1.2× on the V6/V8; at cap 16 nearly every point is ≥ 1.5× (worst ≈ 1.2×, the 1NZ at redline).
The remaining cost is the interpreter; see §7.

### 0.2 Defects found
| # | Defect | Effect |
|---|---|---|
| 1 | **Wiebe release was `Δx × energy remaining`** (v0.5 code), then "fixed" by dumping the remainder when the curve saturated | Only 63 % of the fuel was released along the curve; **37 % (450 J at 3000 rpm) came out in one substep at the end of the burn**. Peak T ≈ 3400 K, indicated efficiency 42–46 %, distorted pressure traces. Now `Δx × total` |
| 2 | Burned gas used the air polynomial | γ stayed ≈ 1.32–1.35 through the expansion; no dissociation. Replaced by equilibrium tables (KB Fig 4-15) with a tracked burned fraction: peak T 2500–2800 K, η_ind peak 36 % |
| 3 | Trapped "air" = mass − residual − fuel | counted exhaust gas drawn back in during overlap as air → ECU estimates too rich at low MAP. Now unburned mass less fuel |
| 4 | Single learned VE | collapsed at low MAP (VE vs manifold density falls from 0.9 to 0.4), λ 0.77–1.45 in transients. Replaced by a slope-intercept-in-MAP model, RLS-learned |
| 5 | First RLS version drifted along its null direction at a steady operating point (collinear regressors + forgetting) until a clamp bit | ECU air 30–45 % off. Forgetting now only when the operating point moves |
| 6 | Idle/start-up hunted or stalled once the physics was right (flare to 1400–1800 rpm, manifold pumped to 9 kPa, sag below stall) | fast-idle ramp, IAC trim authority and floor, dashpot during fuel cut, DFCO delay after start, IAC held at idle while cranking, torque-neutral spark trim, per-engine gain (454: 0.06), VR38 feed-forward 0.0055 |
| 7 | Reversion-fuel accounting, spark-window, etc. (v0.5 list) | see §6 |

### 0.3 New / changed files
New: `engine_sim/gas_tables.py`, `run_benchmark.py`. Substantially changed: `cylinder.py` (rewritten), `cylinder_bank.py` (loop, RLS air model, governor), `engine.py` (idle control, DFCO clock, realtime budget), `interactive_dashboard.py` (2 ms step, governor, substep size), `config.py`, `dashboard/app.js`, the three engine files, tests.

Config: new `max_cylinder_steps_per_ms`, `iac_resolved_kp_fraction`, `fast_idle_extra_ratio`, `fast_idle_decay_s`, `dfco_after_start_delay_s`, `idle_spark_reserve_deg`; changed defaults `idle_spark_gain_deg_per_rpm` 0.03 → 0.04, `idle_spark_max_retard_deg` 12 → 25.

### 0.4 Verification
Single cylinder, 3000 rpm, 0.91 bar, 1°: MBT near 20–25°, peak 58 bar / 2624 K at 25°, η_ind 36.3 %, wall loss 13–24 %.
1NZ WOT: 123 / 120 / 103 / 77 N·m at 1500 / 3000 / 4500 / 6000 rpm (the cycle-averaged path: 144 / 117 / 82 / 39); 454: 600 / 525 / 389 N·m at 1500 / 3000 / 4500; VR38: 312 / 280 / 157 N·m at 1500 / 3500 / 6500. λ ≈ 1.00 everywhere. Energy audit, 1NZ 3000 rpm: 39.8 % work, 17.0 % wall, 43.3 % exhaust. Cold start then idle (35 s): 1NZ 801 ± 19, 454 643 ± 6, VR38 751 ± 13 rpm. Full suite: 157 passed.

---

## 1. Summary

| | Before this pass | Now |
|---|---|---|
| Resolved cylinder modules | built, tested in isolation, **not called by `Engine`** | drive `Engine.update()` when `use_resolved_cylinders = true` (`EngineConfig` default off; **set to true in all three shipped engine files**) |
| Exhaust plenum | never instantiated | live, fed by every exhaust valve |
| Manifold pressure | VE-table flow | integrated per crank-angle substep from the cylinders' own flow |
| Volumetric efficiency | table | emerges from valves/ports/plenums; ECU learns it for fuel metering |
| Pumping, wall loss | empirical / diagnostic | computed in the cylinders |
| Dashboard P-θ/P-V, valve plot | reconstructed / sin² hump | recorded cylinder pressure / the physics' cam table |
| Known defects | 12 listed, 6 more found in review | all fixed (§6), plus integration defects |
| Tests | 114 | 147 |

---

## 2. Review of the previous update — was it implemented correctly?

**The structure and most of the physics were sound.** The first law with extensive `U`, the
combustion-efficiency single application, the Wiebe tail fix, the Woschni kPa basis, the wrapped valve
window, the signed valve flow and the pumping-by-stroke bookkeeping were all correct and are kept.

**What was wrong or incomplete** (each verified by reading the code and by measurement):

| # | Finding | Verdict |
|---|---|---|
| 1 | "Seat angle: `sin(45°)` overstates area ~1.85×" | **The diagnosis was wrong.** Heywood Eq 6.7 is `A = π·L·cos β·(…)` with β measured from the valve-head plane; for the default 45° `sin β = cos β`, so the default was *right*. What was wrong was the formula for every other angle (`sin β` makes a steeper seat open wider, backwards) and the doc comment ("included angle"). Now `cos β`, documented, test inverted |
| 2 | Woschni exponent "`−0.55` in docstring vs `−0.53` in code" | KB Eq 12.19 says `−0.55`; the code was wrong. Fixed |
| 3 | `swept_volume` is the instantaneous volume (zero at TDC) | Worse than stated: it switched the combustion velocity term **off at TDC**, exactly where heat flux peaks. Fixed with an explicit displaced-volume parameter |
| 4 | `C1_GAS_EXCHANGE` dead code | Now selected during gas exchange (valve open), C₁ = 6.18 |
| 5 | Motoring-pressure clamp "pins `p_motored` through expansion" | Worse: `max(1, V(360°)/V)` is **always 1** (V ≥ V(360°)), so `p_motored` was constant for the whole burn. Replaced by `p_r·(V_r/V)^n` through the state at the spark |
| 6 | `cylinder_volume()` vs geometry "disagree near peak piston velocity" | A real bug: `(1/R)(1−√…)` where Eq 2.5 gives `R(1−√…)` (≈12 % volume error). The test tolerance (`rel=0.15`) was loose enough to hide it. Fixed; test now `rel=1e-9`. This also corrected the legacy path's displayed volume |
| 7 | "`engine_file.py` does not parse the new fields" | **Not true.** Every `EngineConfig` field is settable from the file already (`[valves] intake_max_lift_m = …` worked). What was missing was *validation* (sizes > 0, ramps that tile the event) and documentation. Added both |
| 8 | Wall area fixed at the BDC value (`Cylinder.surface_area`, "fixed for the engine's life") | Not in the old list: overstated the area at TDC by ~2.5×, the loss peak. Now `A = π·B²/2 + 4·V/B`, following the piston |
| 9 | `ExhaustSystem` reversion used the cylinder temperature as the enthalpy of gas leaving the plenum | Fixed |
| 10 | Test harness passed `wall_temperature_k` as the 9th positional argument, which is `p_crankcase` | Fixed (keyword) |
| 11 | Fuel mass was booked as energy but never added to the gas | ~6 % of the charge mass missing from pressure and mixture. Fixed in the engine path (fuel rides in with the air) |
| 12 | Torque `= p·dV/dθ(mid)·Δθ` with the pressure at the *start* of the step and `p = mRT/V_mid` | Slightly inconsistent explicit scheme (state belongs to the start edge, not the midpoint). Replaced by an exact reversible-work formulation (§3.3) |
| 13 | `1NZ-FE` engine file had `compression_ratio = 20.5` | Typo (real: 10.5; the README states real geometry). Corrected |

---

## 2. Files

### 2.1 New
| File | Purpose |
|---|---|
| `engine_sim/cylinder_bank.py` | The resolved core: all cylinders at their own phase, the exhaust plenum, the intake manifold ODE per substep, speed-density fuel metering with learned η_v, warm-start pre-roll |
| `tests/test_resolved_engine.py` | 33 integration + regression tests |
| `run_resolved_cycle.py` | One recorded cycle of cylinder 1 → `output/resolved_cycle.csv` + energy audit |

### 2.2 Substantially changed
| File | Change |
|---|---|
| `engine_sim/cylinder.py` | Reworked (§3.3): open-system first law + exact reversible compression, equalisation-limited valve transfers, fuel with the charge, spark-time energy pool, late fuel load, per-cycle books, recorded pressure trace, knock integral, trapped-air measurement |
| `engine_sim/engine.py` | Resolved branch in `update()`, `_make_bank()`, warm start, knock retard, idle spark trim, 15 new `EngineOutput` fields, live `set_config` that keeps the gas state |
| `engine_sim/heat_transfer.py` | Woschni: `T^-0.55`, per-phase C₁, displaced volume, `gas_exchange` flag; `chamber_surface_area(..., clearance_volume=)` |
| `engine_sim/config.py` | See §2.4 |
| `engine_sim/valve_flow.py` | `cos β` seat law |
| `engine_sim/piston_kinematics.py` | normalized `cylinder_volume()` rod term |
| `engine_sim/spark_model.py` | `combustion_duration()` speed law (§6) |
| `engine_sim/exhaust_system.py` | reversion enthalpy |
| `engine_sim/valve_profile.py` | ramp-fraction validation |
| `engine_sim/engine_file.py` | validation of the new keys |
| `engine_sim/gas_properties.py` | Horner form of `cp_air` (hot path) |
| `engine_sim/telemetry.py`, `cycle_trace.py` | resolved fields; `resolved_pressure_trace()` |
| `interactive_dashboard.py`, `dashboard/app.js` | resolved trace and readouts, cam table for valve plot and animation |
| `config/engines/*.toml` | per-valve sizes/lifts, valve counts, exhaust sizing, starter torque (§5), 1NZ compression ratio |

### 2.3 Documentation
`README.md` (§1, 2.1, 2.3, 2.4, 3.3, 7, 8, 10, 11, 12), `ENGINE_EXPLAINED.md` (§2.4/2.5, 12), `KB_GAPS.md` (G-E08, Woschni/valve rows, new correction entry), this file.

### 2.4 Config changes
New: `use_resolved_cylinders`, `iac_resolved_gain_scale` (0.15), `idle_spark_gain_deg_per_rpm` (0.03), `idle_spark_max_advance_deg` (8), `idle_spark_max_retard_deg` (12), `knock_retard_step_deg` (3), `knock_retard_max_deg` (15), `knock_retard_recovery_deg_per_s` (1.5); properties `intake_valve_count` / `exhaust_valve_count` (from `num_valves_per_cyl`).
Changed: valve diameters are now **per valve** (defaults 30/26 mm, were 32/30 for a single valve); `crank_angle_resolution_deg` 2 → 4.
Removed: `exhaust_ambient_pressure_pa` (it duplicated `ambient_pressure_pa` and would have disagreed at altitude).

---

## 3. How it works

*(Sections 3 and 4 describe the v0.5 scheme and its numbers. v0.6 changed the cylinder's internals — tables, burned-gas c_v, `Δx × Q` heat release, second-order flows — and the verification numbers; see §0. The structure below (bank, substeps, torque contract) still holds.)*

### 3.1 The bank (`CylinderBank.advance`)
One engine step sweeps `Δθ = ω·dt`; it is cut into substeps of ≤ `crank_angle_resolution_deg`
(capped at `max_substeps_per_update`). Per substep: every cylinder steps (local angle
`(θ − fire_angle + 360) mod 720`, so firing order, odd-fire intervals and phase come from the file);
the exhaust plenum takes the net flow the exhaust valves delivered; the manifold ODE takes throttle flow
in and the cylinders' *actual* intake flow out. Cylinder cycle bookkeeping starts at intake-valve opening
(710° for the shipped timing), so the first degrees of overlap are metered.

### 3.2 Engine integration
- **Step 1–2** (air): the throttle area (incl. the idle-air valve) goes to the bank; the ECU's air estimate is `η_v,learned·ρ(filtered MAP)·V_d`.
- **Step 3** (fuel): unchanged (fuel film, lambda trim); the measured trapped air feeds the lambda sensor.
- **Step 4–5** (combustion, heat loss): replaced by `bank.advance(...)`. Spark = MBT (+ idle trim − knock retard). `cycle` (`W_net`, `T3`, `p3`, `T4`, …) is synthesised from the bank so emissions, friction breakdown and the turbo see resolved values.
- **Step 6** (losses): the pumping term is the cylinders' pumping loop, not the empirical MEP. Torque contract: `T_net = gross − friction − pumping − accessory − load`; `gross = indicated + pumping`, pumping subtracted once.
- **Step 11**: the crank advances by `ω·dt` (the angle the cylinders were advanced through).
- Turbo: exhaust temperature and pressure come from the plenum.

### 3.3 The cylinder substep
1. Volumes at both substep edges (the state `(m, T)` belongs to the start edge).
2. Valve flow: `valve_mass_flow_signed`, then `_limit_transfer` caps the mass at what equalises cylinder and port (the orifice law has infinite slope at zero ΔP, so an explicit step otherwise overshoots and rings).
3. First law at constant volume: `U' = U + Σ ṁ·h + Q_chem − Q_wall·dt`, `T' = U'/(m'·cv)`.
4. Reversible volume change: `T'' = T'·(V₀/V₁)^(R/cv)`, work `W = m'·cv·(T' − T'')`. The integral of `W` is therefore the energy actually removed from the gas; **torque cannot disagree with the first law at any substep size** (measured: ≤ 1.5 % change from 1° to 6° substeps; the old scheme needed 1–2°).
5. `torque = (W − p_crankcase·ΔV)/Δθ`; pumping = intake and exhaust strokes.
6. Fuel: delivered in proportion to the air through the intake valve, removed in proportion with outflow; the burn's energy pool is sized at the spark from the fuel trapped.

---

## 4. Verification

### 4.1 Single cylinder, 3000 rpm, 0.91 bar, stoich, 1° substeps (corrected physics)
```
 spark   Tind   pump  gross  peakP  peakT  wallQ   eta_ind
  45    46.68   2.46  49.14   91.0   3864  428.3    38.1%
  35    50.27   2.48  52.74   73.8   3611  350.1    41.1%
  25    51.00   2.50  53.50   60.9   3441  294.1    41.7%
  20    50.35   2.52  52.87   53.8   3389  274.1    41.2%
  15    49.08   2.54  51.62   46.8   3356  258.7    40.2%
  10    47.30   2.56  49.86   40.2   3338  247.4    38.7%
   5    45.10   2.58  47.69   34.3   3333  239.3    36.9%
   0    42.60   2.61  45.21   29.2   3338  233.7    34.9%
```
(N·m, bar, K, J.) Interior MBT near 25°, peak pressure monotone in spark. Wall loss is 15–28 % of fuel energy (was 20–27 %; the corrected area and Woschni terms moved it, in both directions).

### 4.2 Whole engine, dyno-held WOT (4° substeps)
```
1NZ-FE (CR 10.5)         rpm   torque  VE     exh kPa  PMEP   peak p
                         1500  145     0.87   102      0.04   63
                         3000  136     0.88   104      0.14   58
                         4500  106     0.86   107      0.38   63
                         6000   74     0.82   112      0.68   54
454 V8 (dual-pipe sized) 1500  716     ~0.88  103      0.09   —      (legacy path: 622)
                         3000  617     ~0.82  106      0.41   —      (legacy: 498)
                         4500  464     ~0.73  111      0.81   —      (legacy: 334)
```
λ within 1 % of stoich everywhere. Legacy and resolved torque agree in order of magnitude; the resolved curve falls off above ~4000 rpm more than a real tuned 1NZ-FE (109 hp @ 6000) because there are no wave effects.

### 4.3 Energy audit (`run_resolved_cycle.py --engine 1nz --rpm 3000`)
Heat released 1127.6 J = 100 %; net work 501.3 J (44.5 %, incl. −6.2 J pumping); wall 212.4 J (18.8 %); exhaust + blowdown 413.9 J (36.7 %). Efficiency is a few points high (§7).

### 4.4 Idle and start
Cold start with the shipped engines from rest (starter held 1.5 s), last 10 s of 25 s: 1NZ → 800 ± 16 rpm, 454 → 651 ± 4, VR38 → 750 ± 12 (converging in ~10 s) at ~19 kPa MAP.

---

## 5. Things that had to change for the resolved core to be usable

- **Idle hunting.** With the cycle-averaged IAC gains the resolved engine limit-cycled between ~500 and ~1400 rpm. Cause: the pulsed intake adds ~one cycle of delay on top of the manifold lag (τ ≈ 0.4 s at idle), so the same PI gains have no phase margin. Fix: kp/ki scaled by `iac_resolved_gain_scale` (0.15) **and** a fast spark-trim loop (spark moves torque instantly; the valve waits on the manifold).
- **Slow low-speed burn.** `combustion_duration` scaled the burn angle ∝ rpm: 27° at idle, MBT spark ≈ 5° (no authority). Now ∝ √rpm (unchanged at 3000 rpm): ~45° burn, ~12–20° idle spark.
- **Starter torque.** The resolved engine has a real compression stroke (~100 N·m peak for 1.5 L), so the old 60 N·m starter stalled against it. Shipped files now set 200 / 300 / 250 N·m (1NZ / 454 / VR38). Harmless for the cycle-averaged path.
- **Late fuel.** A cylinder whose cycle began during a fuel cut waited a whole 720° for fuel; after a DFCO resume the idle then undershot and stalled. Fuel is now picked up while the intake valve is open.
- **Warm start.** `Engine.start()` pre-rolls three fired cycles at idle vacuum with a warm exhaust; `reset()` leaves the engine cold.
- **Valve count.** A single 32 mm intake valve is too little area for a 4-valve head. Diameters are per valve and `num_valves_per_cyl` sets the count.

---

## 6. Defects fixed (cumulative)

From the first version of the model: Woschni pressure basis (bar → kPa) and reference state; `motoring_pressure = 0`; mean piston speed 4× low; first law dividing ΔU by the new mass (intake as a heat source); combustion efficiency applied twice; stranded Wiebe tail; legacy heat loss charged at peak rate × cycle time (engine stall).

Found and fixed in this pass: the table in §1 / §2 above, plus
- a cycle that began at 710° (after the spark angle on the circle) firing at once;
- gas-exchange flow overshooting pressure equality at coarse substeps (equalisation limiter);
- fuel pushed back out by reversion still fuelling the burn;
- `combustion_duration()` linear speed law;
- 1NZ-FE `compression_ratio`.

Each has a quantitative regression test; notably `test_woschni_pins_kpa_basis_and_temperature_exponent`
(closes the "no quantitative Woschni regression test" item), the combustion-term-at-TDC, C₁ phase,
motored-pressure and chamber-area tests, and a mass-conservation test across the valves.

---

## 7. Open items and limitations

1. **Speed.** Done in v0.7 (§0a): the compiled core is ≥ 2× real time on every shipped engine at 1° substeps; the pure-Python core is real-time through the governor. What remains is Python glue (≈ 65 % of a compiled step) and the state copy in and out of the C arrays — see README §11.
2. **No wave dynamics.** Lumped manifold and exhaust: no intake/exhaust tuning; high-rpm torque under-predicted.
3. **Two-zone, not flame propagation.** Dissociation, an end-gas temperature and burned-zone NO are modelled (§0a); there is still no flame geometry (the burn rate is the Wiebe curve), no kinetics beyond Zeldovich, and no crevice / blow-by, so motored-cycle loss and part-load efficiency are rough and the idle MAP (~23 kPa) is a little below a real engine's (~30).
4. **Knock on boosted charge.** Douaud-Eyzat is calibrated for naturally aspirated conditions; on the VR38 it saturates the retard (−8 to −10 % torque), so that file leaves `use_knock_model` off, and its low-speed torque is high (nothing limits it). Compressor map and turbine are scaled models, not measured ones.
5. **Per-engine valve sizes and turbo maps are approximate** (comments in the TOMLs say so).
6. **Seat-angle model** covers only the low-lift regime of KB Eq 6.7 (the port area caps the rest).
7. **Dashboard** changes were syntax-checked and exercised through the server (`/api/config`, snapshot, trace) and the startup banner; the Playwright check `tests/ui_check.py` was **not re-run**.
8. `run_convergence.py` is still stale (v0.1 model).
9. `Cylinder.step` is long (≈ 430 lines with the two-zone path) and now exists twice (Python and C); the parity tests are what keep them identical, and any change to the physics must be made in both.
10. **A third consumer.** `../VehicleSimulator` compiles a byte-for-byte copy of `native/enginecore.c` and carries a hand transcription of the Python around it (`CylinderBank`, `Turbocharger`, the resolved branch of `Engine.update`) in `src/physics/ResolvedEngine.cpp`. A change to `enginecore.c` reaches it through `VehicleSimulator/tools/sync_enginecore.py`; a change to that Python must be repeated there, and `tools/engine_parity_reference.py` + `engine_parity` check it against a trace recorded from this project (bit-identical on the three shipped engines as of v0.7). See README §13.
    The sound is consumed the same way: `dashboard/engine-audio.js` and `sound_spec.py` are transcribed into `VehicleSimulator/src/audio/EngineSimSynth.cpp`, checked against the worklet itself rendered under Node (`tools/engine_sound_reference.py` + `engine_sound_parity`; worst difference 3e-8 of full scale).
11. **Bit-level reproducibility depends on the Python version.** The per-cylinder summaries and the air model's dot products use the built-in `sum()`, compensated since CPython 3.12. Results differ in the last bit on older Pythons, which a gas-table node lookup amplifies to ~1e-5 in torque within a few cycles, after which sensitive transients (idle, fuel resume) run visibly differently. Harmless to the physics; relevant to anything compared against a recorded trace (README §13).

---

## 8. Using it

```toml
# config/engines/<file>.toml
[engine]
use_resolved_cylinders = true
crank_angle_resolution_deg = 4.0     # 6–8 = faster, a few % less accurate
```
```bash
python run_resolved_cycle.py --engine 1nz --rpm 3000 --throttle 1.0    # one cycle + energy audit
python interactive_dashboard.py --engine path/to/that.toml             # P-θ / P-V shows the real cylinder
```
Telemetry (`TelemetryRecorder`) gains `exhaust_pressure`, `exhaust_temperature`, `imep_net_bar`,
`imep_gross_bar`, `pmep_bar`, `peak_pressure_pa`, `knock_retard_deg`; `EngineOutput` also carries
per-cylinder pressure, temperature, mass and valve lift.

---

## 9. Reference

- `ENGINE_KB.md` — Heywood 1988: Eq 2.5/2.6/2.11 (geometry, torque), Eq 6.7 (valve flow area), Eq 7.18–7.22 (orifice flow, manifold), Eq 9.28 (Wiebe), Eq 9.60–9.64 (knock), Eq 12.16–12.19 (Woschni).
- `KB_GAPS.md` — G-E08 (pressure trace; now closed for the resolved mode), Woschni and valve rows, the v0.5 correction entries.
