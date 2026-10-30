# How the EngineSim Engine Works, From Start to End

A plain-language walkthrough of everything that happens between "you press a key" and "you hear and see an engine respond". There are no equations here on purpose. The goal is that after reading this you can picture what the simulation is doing at every moment, know why it behaves the way it does, and know where in the code to look.

For parameter tables, formulas and references, see [README.md](README.md). For where each physical idea comes from in the textbook, see [KB_GAPS.md](KB_GAPS.md).

## Contents

1. [The one big idea](#1-the-one-big-idea)
2. [The cast: what the pieces are](#2-the-cast-what-the-pieces-are)
3. [One tick of the engine (1 millisecond)](#3-one-tick-of-the-engine-1-millisecond)
4. [Optional models that plug into the tick](#4-optional-models-that-plug-into-the-tick)
5. [Stories: what the engine does in real situations](#5-stories-what-the-engine-does-in-real-situations)
6. [The car around the engine: clutch, gearbox, vehicle, dyno](#6-the-car-around-the-engine-clutch-gearbox-vehicle-dyno)
7. [The engine file: one description, three uses](#7-the-engine-file-one-description-three-uses)
8. [The server and the dashboard loop](#8-the-server-and-the-dashboard-loop)
9. [Reading the instruments](#9-reading-the-instruments)
10. [The dyno test and the torque/power curve](#10-the-dyno-test-and-the-torquepower-curve)
11. [How the sound is made](#11-how-the-sound-is-made)
12. [What is simplified, and what that means for the numbers](#12-what-is-simplified-and-what-that-means-for-the-numbers)
13. [Where to look in the code](#13-where-to-look-in-the-code)

---

## 1. The one big idea

Think of the engine as a **heavy spinning wheel** (the crankshaft plus flywheel). Three things act on it:

- **Pushing it forward:** the combustion in the cylinders. This is the only thing that speeds the engine up by itself.
- **Dragging it back:** friction (rubbing metal and oil), pumping (the piston sucking against a nearly closed throttle), and accessories (water pump, oil pump).
- **Whatever the outside world does to it:** the clutch and the car it is connected to, or the dyno holding it at a speed, or the starter motor spinning it up.

**The engine speed is never typed in.** Nobody tells the simulation "the engine is at 3000 rpm". Instead, every millisecond the simulation adds up all the pushes and drags, and the leftover (the *net* push) is what accelerates or slows the wheel. Heavy wheel: it changes speed slowly. Light wheel: it changes speed quickly. If the push exactly equals the drag, the speed stays where it is. That is all "revving" and "idling" and "stalling" are.

Everything else in the project exists to answer one question well: **how big is the push from combustion right now, and how big is the drag right now?** Combustion depends on how much air got in, how much fuel went with it, and when the spark fired. Drag depends mostly on speed and on how closed the throttle is.

---

## 2. The cast: what the pieces are

| Piece | One-line role | Code |
|---|---|---|
| **Engine** | The spinning wheel plus the chain "throttle → air → fuel → combustion → torque" and all the drags | `engine_sim/engine.py` |
| **Config** | Every number describing the engine: size, friction, limits, which optional models are on | `engine_sim/config.py` |
| **Air model / manifold model** | How much air reaches the cylinders | `air_model.py`, `manifold_model.py` |
| **Fuel model** | How much fuel goes with that air | `fuel_model.py`, `fuel_film.py`, `lambda_controller.py` |
| **Combustion model** | Turns air + fuel into a push (torque) | `combustion_model.py`, `spark_model.py`, `heat_transfer.py` |
| **Resolved cylinder** ★ | The same job done degree by degree: gas state carried across the 720° cycle | `cylinder.py` |
| **Cylinder bank** ★ | All the cylinders at their own phases, plus the intake manifold and the exhaust, stepped together; also the fuel metering | `cylinder_bank.py` |
| **Valve cam** ★ | Turns the engine file's timing numbers into lift versus crank angle | `valve_profile.py` |
| **Valve flow** ★ | How much gas actually moves through an open valve, and which way | `valve_flow.py` |
| **Exhaust system** ★ | The pressure the exhaust valve pushes against | `exhaust_system.py` |
| **Friction models** | The drags | `config.py`, `friction_breakdown.py` |
| **Powertrain** | Clutch, gearbox, car, brakes, dyno, fuel used, distance driven | `powertrain.py` |
| **Engine file loader** | Reads one text file per engine and builds everything from it | `engine_file.py` |
| **Sound spec** | Derives from that same file everything the sound synthesizer needs | `sound_spec.py` |
| **Server** | Runs the simulation 1000 times a second and streams the results | `interactive_dashboard.py` |
| **Dashboard** | The page you look at: dials, animation, plots, sound | `dashboard/` |

★ = the **crank-angle-resolved** parts. They run the engine when `use_resolved_cylinders = true` in the engine file (on in all three shipped engine files; it is several times slower than the cycle-averaged chain, so set it to `false` for the fast path); otherwise the engine uses the cycle-averaged chain above. See §2.5.

The engine itself knows nothing about the car, the page or the sound. The powertrain wraps the engine and adds a car. The server wraps the powertrain and adds a clock and a network connection. The dashboard only ever *receives numbers* and *sends your key presses back*.

### 2.5 The degree-by-degree engine (optional: `use_resolved_cylinders`)

The tick described in §3 answers "how much air" and "how much heat" once per millisecond and averages
the result over the cycle. The ★ modules instead walk the crank in small steps — 4 degrees by
default — and let the physics accumulate on its own. Switch it on in the engine file (`[engine]
use_resolved_cylinders = true`); the rest of this section describes what the engine then does instead of
Steps 2, 4 and 5 of §3.

The idea is that a cylinder is a **closed system with a door**, and once you keep track of what is inside
it, almost everything else stops being a special case:

- The **same pressure** makes the torque, drives the heat transfer, and moves the gas through the valves.
- **Spark advance** works because the heat release happens at a chosen angle, not all at once.
- **Reversion and scavenging** work because gas flows whichever way the pressure difference points.
- **Pumping loss** falls out of the intake and exhaust strokes rather than being a correction factor.
- **Volumetric efficiency** is no longer a table: it is whatever the valves, ports and manifold let in, so
  smaller valves really do choke at high rpm, and a restrictive exhaust really does cost power.
- **The cylinders talk to each other** through the two shared volumes: one cylinder's intake stroke
  pulls the manifold down for the next, and exhaust pulses stack up in the exhaust.

For each step of a cylinder, in order: work out the volume from the crank geometry, get the pressure from
the gas it contains, push gas in and out through whichever valves are open (carrying its enthalpy with
it, and never more than would level the two pressures), release some of the fuel's heat on the Wiebe
curve, lose some heat to the walls, then let the gas expand or be squeezed across the volume change —
and the work that takes *is* the torque. The energy balance:

```
dU = (heat from fuel) − (heat to walls) + (heat carried in and out by the gas)   — with the valves' flow
dU = −p·dV                                                                       — across the volume change
```

`U` is the energy of everything in the cylinder, so it has to be carried from one step to the next rather
than recomputed — getting that wrong is what previously made intake air heat the charge up to 718 K.

**The fuel computer is realistic about what it knows.** It estimates the air from the manifold pressure and
an air model it *learns* from what each cylinder actually trapped (how much air a given manifold pressure
and speed really gives) — so in a hurry (a sudden throttle change) it is briefly wrong and the mixture
drifts, as in a real car. Closed-throttle idle is held by a slow idle-air valve, a fast spark trim, and a
"fast idle" at start-up that begins high and settles to idle speed, like a real ECU's; knock (when enabled)
retards the spark instead of scaling the torque.

**The burned gas is not air.** After combustion the gas in the cylinder is a hot mixture of CO₂, H₂O and
nitrogen, and above about 2200 K it starts to *dissociate* — splitting into fragments, which soaks up energy.
The model carries how much of the gas is burned and uses burned-gas properties from the Heywood
equilibrium data, which is why the peak temperature is about 2500–2800 K (an air-like gas would reach ~3400 K)
and why a quarter or more of the fuel's energy cannot be turned into work.

Measured, it behaves the way a real engine does: peak pressure falls steadily as you retard the spark,
torque is best a little before top dead centre and worse on both sides of that, about a sixth of the
fuel's energy leaves through the walls and about 36 % ends up as work at best, and a 1.5 L four-cylinder
holds its 800 rpm idle and makes about 120 N·m from 1500 to 3000 rpm. It is still a lumped-volume model: there
are no pressure *waves* in the pipes, so high-rpm torque is lower than a tuned real engine's.
`README.md` §2.3 has the numbers, and `python run_resolved_cycle.py` prints one cycle's energy audit.

**The flame has two sides.** While the flame is burning, the cylinder is treated as two bodies of gas at the
same pressure: the *unburned* charge ahead of the flame, which is only ever squeezed (so its temperature is the
real temperature knock cares about), and the *burned* gas behind it, which is hot enough to make nitric oxide.
Each step the flame turns a little unburned gas into burned gas at the temperature the fuel's energy allows,
and both bodies are then squeezed or relaxed together until they fill the new volume. This is where the
exhaust's NO comes from, and why the knock check no longer has to guess the end-gas temperature.

**A flame can fail.** If the charge is too diluted with exhaust gas, or too lean or too rich, the flame burns
slowly and partly, and past a limit not at all — a **misfire**. That is what stops the engine idling at an
unrealistically low manifold pressure: a throttle that is closed too far leaves too much exhaust gas in the
cylinder for the flame to survive, so the engine settles at about 23 kPa with a sixth of the charge being old
exhaust gas. Gas that is pushed back up the intake runner during valve overlap is exhaust gas, waits there,
and is the first thing drawn back in.

**The cam can move.** Variable valve timing slides the whole valve-lift curve earlier or later in the cycle,
so the opening, closing and the overlap between intake and exhaust all move together. The ECU advances the
intake cam with speed and load, as real VVT systems do; the dashboard can override it.

**The turbocharger is made of real parts.** With a turbo, the exhaust no longer leaks out of a fixed hole. It
has to push through a **turbine** (a restriction that also steals energy to spin the shaft) or round it
through the **wastegate**, and the compressor on the other end of the shaft pumps air into a volume that the
throttle draws from. So boost and exhaust backpressure are linked: asking for more boost raises the pressure
the pistons must push the exhaust against, opening the wastegate relieves both, and turbo lag is simply the
shaft's inertia against the energy it is being given. The wastegate is held by a small feedback controller so
boost settles at its target instead of swinging.

**It is built to keep up with the clock.** Stepping every cylinder every few degrees is expensive. The inner
loop — the arithmetic repeated for every cylinder every degree — is written twice: once in Python, and once in
C, which the simulator compiles itself the first time it runs if the computer has a C compiler (it says so in
the dashboard's start-up text). The C version does exactly the same arithmetic about thirty times faster, which
lets it take 1° steps and still run two to five times faster than the clock on every shipped engine. Without a
compiler the Python version runs instead: table-driven, with a cap on the cylinder-steps per millisecond and a
governor that times itself and takes slightly coarser steps at high rpm whenever it would otherwise fall
behind (near redline on a V8 the steps are then about 13° and the torque reads a few percent high). The
tests run both versions side by side and require the same answers. `python run_benchmark.py` shows the
speed of each engine.

---

## 3. One tick of the engine (1 millisecond)

The simulation moves forward in tiny steps of **one millisecond**. At each step, the engine runs through the same sequence of questions, top to bottom. This is the heart of the whole project, so it is described in order.

### Step 0: What state is the engine in?

Before any physics, the engine decides what it is *allowed* to do right now.

- **Is the ignition on?** If not, there is no spark and no fuel, so no combustion. The engine can only coast.
- **Is it synchronised?** A real computer needs to know where the crankshaft is before it fires the first spark. In the simulation the starter must turn the engine for about one and a half revolutions before the first fire. This is why starting takes a moment of cranking rather than catching instantly. If the engine slows below its stall speed with no starter helping, it *loses* sync and dies. It can only come back by cranking again, or by the car pushing it back above stall speed (a bump start).
- **Is fuel being cut?** Two reasons. (1) **Rev limiter:** at redline the fuel is cut, and only restored once the speed has dropped a little below redline. That gap is why the engine *bounces* off the limiter instead of sitting at redline. (2) **Decel fuel cut-off:** if your foot is off the throttle and the engine is well above idle, the computer shuts the fuel completely. This is what produces real engine braking.

The result of this step is a single yes/no: **is combustion happening this tick?** If the answer is no, every later step that depends on combustion produces zero push, and only the drags remain.

### Step 1: How much air is waiting at the cylinders? (intake and manifold)

Between the throttle plate and the cylinders sits the **intake manifold**, a plenum full of air. Picture a balloon with a tap (the throttle) feeding it and the cylinders sucking from it.

- With the throttle nearly closed, the cylinders suck air out faster than the narrow gap lets it in, so the manifold pressure drops well below the outside air. That is **vacuum**, and it is why a closed-throttle engine has weak push and strong drag.
- With the throttle wide open the gap is large, the manifold refills, and pressure rises close to outside air pressure. Full push.

When the manifold model is on (all the shipped engines use it) the manifold pressure is a real *remembered* quantity: it takes a moment to fill or empty. That delay is the realistic lag you feel when you press the pedal and the engine does not respond instantly. Without the manifold model, pressure would jump straight to a value set by the pedal position.

The throttle's opening does not scale linearly. A throttle plate is a disc in a tube, so the first few degrees open very little flow and the middle of the travel opens a lot. The simulation uses that same shape.

An **idle air bypass valve** can sit in parallel with the throttle. When the throttle is closed, the computer opens this valve slightly to keep the engine breathing enough to idle. This means idle torque comes from real airflow, not from a trick.

### Step 2: How much air is actually trapped in each cylinder?

Not every bit of air in the manifold ends up in the cylinder. How completely a cylinder fills is called **volumetric efficiency**. It is highest at mid speeds. At low speed some air is pushed back out before the valve closes. At high speed the air simply cannot flow in fast enough through the valves and runners. The simulation uses a curve that rises, peaks, then falls with speed.

The air mass per cylinder per cycle is then simply: how dense the manifold air is (pressure and temperature) multiplied by how much volume the cylinder sweeps and by that filling efficiency. **This single number, air per cylinder per cycle, is the main lever for torque.** More air means room for more fuel, which means more energy released.

### Step 3: How much fuel goes with it?

The base rule is simple: add the amount of fuel that mixes perfectly with that air (for petrol, about one part fuel to 14.7 parts air). Two optional models then refine this:

- **Fuel film (wall wetting):** some of the injected fuel sticks to the manifold walls as a wet film and evaporates slowly. When you stab the throttle, extra fuel is needed to wet the walls before the cylinders get their share, so the mixture briefly goes **lean**. The lean spike on rapid throttle opening is a real effect and the model reproduces it. When you lift off, the film releases fuel that was waiting, causing a brief rich moment.
- **Lambda controller (closed loop):** a simulated oxygen sensor in the exhaust reports whether the previous cycle was rich or lean, and the controller nudges the fuel up or down to pull it back toward the ideal. Because it looks at the *actual* result (including the fuel-film disturbance), it genuinely reacts to something.

If combustion is not allowed this tick (Step 0), the fuel is simply zero.

The output of this step is the **air-fuel ratio** you see on the dashboard dial, and the oxygen reading derived from it. When fuel is cut, the AFR dial shows CUT and the oxygen reading rises towards fresh-air levels.

### Step 4: Combustion becomes torque

Here air plus fuel becomes a push. The model is a simplified, idealised version of what happens in one cylinder over one full four-stroke cycle (two crank revolutions, 720 degrees):

1. **Intake:** the piston goes down, the cylinder fills.
2. **Compression:** the piston squeezes the mixture. It heats up and its pressure rises. Squeezing harder (higher compression ratio) makes the later push more efficient.
3. **Power:** the spark fires and the fuel burns. Its chemical energy becomes heat, which raises the pressure sharply and drives the piston down. The expanding gas gives back most of the work spent compressing, plus a lot more. The *difference* is the useful work.
4. **Exhaust:** the piston pushes the burnt gas out.

The simulation computes the useful work from one cycle, then converts that into an **average torque**: the steady twisting force that would deliver the same work over those two revolutions. Real engines produce pulses of torque (one per firing); the simulation uses the smooth average. That average is computed for one cylinder, then multiplied by the number of cylinders to get the whole engine's push.

How hot the gases get, and so how much work is extracted, depends on the gases' properties, which change with temperature. The simulation uses temperature-dependent properties for fresh air and for burnt gas rather than one fixed number.

**The result is the "indicated torque"**: the push that the combustion itself creates, before any drag is taken off.

### Step 4 (continued): spark timing, knock, and cycle-to-cycle variation

Combustion takes time, so the spark has to fire *before* the piston reaches the top. Fire too early and the pressure fights the piston coming up. Fire too late and the burn finishes after the piston has already started falling, wasting energy. There is a "just right" timing (called MBT, minimum advance for best torque) that depends on speed and load. The simulation works out that ideal and **reduces torque slightly for every degree you are away from it**. The dashboard's spark dial shows the advance in use.

**Knock** is the engine's enemy: the unburned mixture ahead of the flame heats and squeezes until it ignites by itself, which is damaging. The simulation tracks how close the end-gas is getting to self-ignition, using the pressure, temperature (on the degree-by-degree engine, the real unburned-gas temperature of the two-zone flame) and the fuel's octane rating. When it gets there, a knock event is flagged and torque is pulled back (as if the computer had retarded the spark). High compression plus low-octane fuel triggers it; the opposite does not.

**Cyclic variability** (optional): no two real cycles burn identically. When enabled, each cycle's work is randomly scaled by a small amount, with more scatter at idle, with light load and with very lean or rich mixtures.

There is also a **Wiebe curve**, the standard S-shaped description of how fast fuel burns over the crank angle. In this project it is computed and reported for diagnostics and spark phasing, but it does *not* yet drive the heat release itself. The cylinder pressure is still the idealised cycle (see section 12).

### Step 5: Heat lost to the walls (optional)

A real cylinder loses a lot of heat to the cooled walls, especially at low speed where the gas has more time to touch them.

**On the currently active path this loss is measured but no longer subtracted from the work.** That is deliberate. The Woschni correlation is a *crank-resolved* formula — it needs the gas pressure and temperature at every angle — but this path only has one averaged cycle result, whose peak state is itself a fiction: adding all the fuel at constant volume predicts about 4400 K and 152 bar, where a real petrol cylinder sees nearer 2500 K and 50 bar. Hand that to the correlation and the predicted loss becomes absurdly large, net work goes negative, and **the engine stalls** (the dashboard tests for this).

So the resolved cylinder in §2.5 does the real wall heat loss, inside the energy balance it is already solving, where the correct temperature history actually exists. Here it is reported as a diagnostic number only.

### Step 6: The drags

Three drags are computed, each turned into a torque opposing rotation.

- **Friction:** pistons and rings rubbing the bores, bearings spinning in their oil, valves being pushed open against springs. It grows with speed, roughly gently at first and then more steeply. There is a simple lumped version and an optional detailed version that adds up piston, bearing and valvetrain contributions separately.
- **Pumping:** every intake stroke, the piston sucks against low manifold pressure, and every exhaust stroke pushes against exhaust backpressure. With the throttle nearly closed, this is a big drag (it is half of why a closed-throttle engine slows you down). At full throttle it nearly vanishes.
- **Accessories:** the water pump, oil pump and similar, which scale up with speed.

### Step 7: The net push

Now the main balance: **combustion push, minus friction, minus pumping, minus accessories, minus whatever the outside world is loading it with.** The load might be the clutch carrying the car's weight, or the dyno, or nothing at all.

If the load is *negative*, the outside world is pushing the engine instead. This is how the engine is dragged around by the car's momentum during engine braking, or spun by the dyno or the starter.

### Step 8: Idle control and the starter

- **Idle governor:** when the throttle is closed and the engine is near idle speed, a controller adds or removes a little torque (as the computer would by adjusting spark or air) to hold the idle speed steady. It has limits: it can at most *cancel* the combustion push, it cannot invent drag. It is switched off during decel fuel cut. The air-bypass valve version does the same job through real airflow instead of added torque.
- **Starter motor:** when you hold the start button, the starter adds a strong torque that fades as the engine speeds up, like a real electric motor running out of speed. It cranks the engine until it is fast enough to fire and has the required sync revolutions.

### Step 9: Speed changes (the crankshaft)

This is the step that implements the big idea. The net torque from step 7 is divided by the wheel's inertia to get the **acceleration**, and the acceleration changes the speed a little for this millisecond. The new speed becomes the starting point for the next tick. Heavy inertia (flywheel and clutch together) means small steps in speed; light inertia means big steps.

The engine can never run backwards in this simulation, so the speed is held at zero or above.

### Step 10: Rev limiter

If the speed has reached redline, the limiter acts. In the default **fuel-cut** mode, fuel is cut on the *next* tick (Step 0) and restored once the speed has fallen a bit below redline. In **torque-cut** mode the speed is clamped instead.

### Step 11: Crank angle

The simulation also tracks **where the crankshaft is** in its 720-degree cycle, by adding to it according to the current speed. This angle decides which stroke the cylinders are in, drives the animation, and tells the exhaust valves when to open for the sound.

---

## 4. Optional models that plug into the tick

All of these are **off by default** so a basic engine behaves simply; enabling a model adds the matching step in the chain.

| Model | What it adds in plain terms |
|---|---|
| **Manifold dynamics** | Intake pressure has memory: throttle response lags realistically. Needed for idle air control. |
| **Idle air control** | A bypass valve keeps the engine breathing at idle; holds its position during decel so the engine catches idle when fuel returns. |
| **Fuel film** | Lean spike on throttle stab, rich blip on lift off. |
| **Lambda controller** | The mixture is corrected by oxygen-sensor feedback. |
| **Heat transfer** | Heat loss to the cylinder walls reduces work. |
| **Knock model** | Detects self-ignition risk and retards torque. |
| **Cyclic variability** | Cycle-to-cycle scatter, worse at idle. |
| **Friction breakdown** | Friction is the sum of piston, bearing and valvetrain contributions rather than one lump. |
| **Turbocharger** | Exhaust energy spins a turbine, which spins a compressor that raises intake pressure. It has a shaft that takes time to spin up (turbo lag) and a wastegate that limits boost. On the cycle-averaged path the boosted pressure simply replaces outside air at the intake; on the degree-by-degree engine (§2.5) the turbine, wastegate, compressor and charge-air volume are real parts and the exhaust backpressure they create is felt by the cylinders. |
| **Emissions** | Estimates of nitrogen oxides (formed in hot burnt gas), carbon monoxide and unburnt hydrocarbons. On the cycle-averaged path they are single-point estimates; on the degree-by-degree engine the nitric oxide is tracked through the burn in the burned gas, and a misfiring cycle sends its fuel out unburned. |
| **Diesel mode** | Replaces the spark-ignition torque with a diesel-style one built on ignition delay and a premixed burn fraction. A fuller diesel model exists in the code but is not wired in. |

---

## 5. Stories: what the engine does in real situations

### Cold start

You hold **S**. The ignition switches on and the starter motor torque spins the crank up. For the first fraction of a second there is **no fire**: the engine is just being motored and the ECU is waiting for the sync revolutions and enough crank speed. Then the first fuel and spark arrive. Combustion adds a push on top of the starter. Once the engine speed is above the starter's usable speed the starter's push fades away, and the engine runs on its own. The idle controller takes over and settles the engine at idle speed. If you let go of the button too early, the engine falls below stall speed, loses sync and dies.

### Steady idle

The throttle is shut, so the manifold is in deep vacuum and each cylinder breathes only a little air. Push is small but so is drag. They balance at idle speed. The idle controller makes tiny corrections. Combustion is at its least consistent here, which is why idle is slightly lumpy.

### Pressing the throttle

The throttle plate opens, air starts flowing into the manifold, the manifold pressure climbs (with a short delay), the cylinders get more air and so more fuel, the push grows, and the net torque turns positive. The speed climbs. If the fuel film is on, there is a brief lean moment as the walls wet. As speed rises, friction rises too, and so does the push from more cycles per second until volumetric efficiency starts falling. The speed settles where push equals drag, or hits the limiter first.

### Wide-open throttle pull

Maximum air, so maximum push. Torque typically peaks in the middle of the rev range where filling is best and drops at the top. Power (torque multiplied by speed) peaks higher up than torque does. The engine hits redline and the limiter starts cutting fuel.

### Lifting off at speed (engine braking)

Throttle closes. Manifold pressure plunges into vacuum (the dashboard shows a deep negative inches-of-mercury reading). Pumping drag jumps. Because the engine is well above idle and the throttle is closed, **decel fuel cut-off** shuts the fuel. Now only drag remains, so the torque goes negative and the engine slows. If it is connected to the car through the clutch, the car's momentum drives the engine and *the engine's drags slow the car*, which is genuine engine braking. When the speed falls to near idle, fuel returns and the idle controller catches the engine so it does not stall.

### Hitting the limiter

Speed reaches redline, the fuel cuts, and with no push the drag slows the engine just below redline, so fuel returns, the engine pushes past redline again, and the cycle repeats. On the dashboard this shows as the state "REV LIMITER" and the rapid chatter, and in the sound as the characteristic stutter.

### Stalling

With the auto-clutch off, you can dump the clutch in gear at low speed. The load of the car suddenly exceeds the engine's push at idle, the engine slows below stall speed, sync is lost, and the engine dies ("STALLED · HOLD START"). The same thing happens if you switch the ignition off and let it run down.

---

## 6. The car around the engine: clutch, gearbox, vehicle, dyno

The **powertrain** (`powertrain.py`) wraps the engine in a car and also runs every tick, just before the engine.

- **Gearbox:** a set of gear ratios and a final drive. Gear 0 is neutral, so the engine is disconnected and spins freely.
- **Clutch:** a friction plate that can carry a limited torque. It has two modes. When **slipping**, the engine and the car are at different speeds and the clutch transmits its capacity (scaled by how engaged it is) from the faster side to the slower side. When **locked**, they turn together in a fixed ratio and the clutch carries whatever torque is needed. It locks as soon as the speeds meet, and momentum is conserved when it does. If the demanded torque exceeds capacity, it slips again. Engagement follows the pedal or, with **auto-clutch** on, an automatic rule that disengages at low engine speed (so you cannot stall) and when shifting.
- **Vehicle:** mass, wheel size, air drag, rolling resistance and brakes. The road's resistance, reflected back through the gearing, is the load the engine feels. In neutral, the car simply slows down by itself.
- **Fuel accounting:** adds up the fuel burned and the distance driven, which gives the fuel, cost and mpg readouts.
- **Dyno:** see below.

Gear changes briefly disengage the clutch so the engine can match its new speed. The vehicle never reverses, and there are no hills or tyre slip.

**The dyno** is a machine that holds the engine at a chosen speed. It is a controller that watches the engine speed and applies exactly enough opposing torque to keep it there. If the engine is pushing harder than needed, the dyno absorbs the extra. If the engine is not pushing enough (for example with the throttle closed), the dyno *drives* the engine up to the target speed. Because the speed is held fixed, the torque the dyno applies is exactly the engine's output at that speed, which is what the dashboard then reports as torque and power.

---

## 7. The engine file: one description, three uses

Every engine is described by **one text file** (TOML) in `config/engines/`. The same file feeds:

1. **The physics:** the size, inertia, speeds, friction, valve timing, vehicle weight and gears all go into the engine and the powertrain configuration.
2. **The animation:** bore, stroke and rod length shape the pistons; firing order and bank angle set the V; valve timing opens the valves on screen.
3. **The sound:** firing order and intervals say when each pulse happens; valve timing says at which crank angle; the exhaust pipe lengths and temperatures decide pitch and ringing; the flywheel inertia decides how lumpy the idle is.

Because one file drives all three, they can never disagree. Changing the exhaust length changes the sound and nothing else; changing the flywheel changes both how the engine revs *and* how lumpy it sounds.

**Live reload.** The server watches the file. When you save it, the server re-reads it, re-applies the new numbers to the running engine without stopping it, and tells the page, which rebuilds its animation and the sound synth. A typo gives a red message naming the file, section, key and a suggestion, and the previous settings keep running until the file is valid again. A reset (Backspace) restarts from the file's current contents.

Three engines ship: a **Chevrolet 454 V8** (big, lumpy, uneven per bank), a **Toyota 1NZ-FE inline four** (light, even, revvy), and a **Nissan GT-R VR38 V6** (smooth idle, sharper high-rpm note).

---

## 8. The server and the dashboard loop

```
 your keys / clicks ──► browser page ──POST /api/control──► server ──► powertrain ──► engine
                              ▲                                                         │
                              └────────── 60 times a second: telemetry (SSE) ◄──────────┘
```

**The server** (`interactive_dashboard.py`) runs a loop with a stopwatch. It keeps track of how much real time has passed and runs that many 1-millisecond physics ticks to catch up, so the engine runs in real time. If the computer falls far behind it drops time rather than spiralling. In **slow motion** the simulated time passes more slowly than the wall clock (down to 1/32), so you can watch valves and pistons; the audio and animation follow.

Roughly 60 times a second it takes a **snapshot** of the engine (speed, torque, manifold pressure, AFR, spark, state flags, fuel used, and a cylinder-pressure trace) and pushes it to every connected page as one message on a one-way stream (Server-Sent Events). This is why the page needs no polling.

**Controls travel the other way.** Whenever your inputs change, the page sends the full set (throttle, ignition, starter, clutch, gear, brake, auto-clutch, dyno on/off and speed) to the server, which copies them into the powertrain's control record, clamping them to legal ranges (for example the dyno speed cannot exceed redline).

**The page** (`dashboard/app.js`) does three jobs at once:

1. **Keyboard and mouse to controls.** Keys like Q, W, E, R hold a throttle opening; Space ramps a pedal up while held and back down on release; sliders set a held throttle.
2. **Drawing.** An animation loop redraws on every screen refresh. The engine animation uses its own crank angle that it advances from the latest engine speed (so it stays smooth between 60 Hz messages). Dials and plots refresh at half rate to save time. It avoids re-drawing anything that has not visibly changed.
3. **Feeding the sound.** Every telemetry message is forwarded to the audio worklet.

---

## 9. Reading the instruments

| Instrument | What it tells you |
|---|---|
| **Engine speed** | The crank speed. The red zone is where the limiter cuts fuel. |
| **Vehicle speed** | Road speed. Locked clutch: directly tied to engine speed through the current gear. |
| **Throttle / clutch / brake bars** | The pedals as the simulation sees them. Clutch shows 100 when fully disengaged. |
| **Gear and lamps** | IGN (ignition), START (starter), FUEL CUT (no injection), CLUTCH LOCK or SLIP (in gear), DYNO (held by dyno). |
| **State pill (top bar)** | Plain-language status: OFF, CRANKING, RUNNING, STALLED, REV LIMITER, FUEL CUT, ENGINE BRAKING, DYNO HOLD. |
| **Manifold (inHg)** | Vacuum. About −22 at idle, near 0 at full throttle. Deep vacuum on lift-off is engine braking. |
| **Air flow (SCFM)** | How much air the engine is drawing. |
| **Volumetric efficiency** | How completely the cylinders are filling. Peaks mid-range. |
| **Air / fuel** | Ideal for petrol is about 14.7. Lower is rich, higher is lean. CUT means no fuel. |
| **Exhaust O₂** | Leftover oxygen: near zero at ideal mixture, high when lean or during fuel cut. |
| **Spark** | Degrees before top dead centre the spark fires. The computer advances it as speed rises. |
| **Torque and power** | At the flywheel (or measured at the dyno when it is on). Negative means engine braking. |
| **Fuel used** | Fuel burned, its cost, miles per gallon and distance. |
| **Plots** | History of speed and manifold pressure; the valve-lift diagram; cylinder pressure through one cycle; the pressure-volume loop (whose enclosed area is the work done); torque and power history; and the dyno curve. |

---

## 10. The dyno test and the torque/power curve

**Why a curve needs a dyno.** Torque and power depend on engine speed. To draw the whole curve you need to measure at many speeds, each held steady. The dyno does the holding.

**What happens when you press D.**

1. The dyno switches on and holds the engine near its current speed.
2. A setup window opens. You choose the throttle opening to test at (normally 100%), the lowest and highest speed, the step between points, how long to wait at each point for things to settle, and how long to average the measurement.
3. **Run sweep** starts an automatic test; **Hold only** just holds the first speed so you can move the slider yourself; **Cancel** turns the dyno off.

**During a sweep.** The page takes control of the throttle and the dyno speed. It puts the car in neutral, turns the ignition on, and sets the first speed. Because the dyno motors the engine up to speed, you do not need to start the engine first. At each speed it waits until the speed has settled (the engine reaches the target and the readings stop moving), then averages torque and power over the measuring window and records one point. Then it moves to the next speed. It uses *simulated* time to wait, so slow motion does not break it. When the last point is done the dyno switches off, your own throttle setting is restored, and a message announces the peak torque and peak power with their speeds.

**Reading the plot.** Torque is the white line, power the teal line. Power is torque times speed, so power's line always rises faster than torque's with speed, and the two lines cross at one fixed speed (about 5252 rpm in these units). The dashed line marks it. Click the plot to open the detailed view, where hovering gives exact numbers and a table lists every measured point.

**Why the last point can dip.** Testing at redline runs into the fuel-cut limiter, which reads as a sharp drop or negative torque. The default sweep stops just short of redline for this reason.

---

## 11. How the sound is made

There are no recordings. The sound is **built from scratch in the browser, in real time, from what the simulation is doing.**

**What the simulation provides.** Engine speed, manifold pressure, throttle, whether combustion is happening, whether fuel is cut, whether the starter is on, plus the engine description (firing order, valve timing, exhaust layout) from the engine file.

**Step A: When does each cylinder "fire" the sound?** The synth keeps its own crank angle and walks through the 720-degree cycle. Each cylinder has a moment where its exhaust valve opens; at that moment the burnt gas rushes out. The firing order and any uneven spacing decide *when*. A cross-plane V8 fires its cylinders in an uneven pattern per bank, and since each bank has its own exhaust pipe, the unevenness is audible as the V8 burble. An inline four fires every 180 degrees evenly, giving a smooth buzz.

**Step B: How hard does each pulse hit?** It depends on what the cylinder holds. With combustion, the strength follows the manifold pressure (more air burnt means a bigger pulse), and each cylinder gets a small *permanent* strength difference that repeats every cycle (that is what gives an idle its rhythmic lope). With fuel cut, there is no combustion, and at high vacuum the exhaust actually draws gas backwards, so the pulse is negative and hollow: that is the sound of engine braking. After lift-off, occasional random bigger pulses are overrun pops.

**Step C: The shape of each pulse.** A pulse is a short, smooth burst (never a hard edge, or it would click) plus a longer, lower push for the piston sweeping out the rest of the gas.

**Step D: The exhaust pipe.** This is where an engine's note comes from. Each pipe is modelled as a chain of tubes: header, mid-pipe, a wider muffler chamber, and the tailpipe. Sound waves travel down, bounce at the junctions and at the open end, and travel back. Each bounce reflects a little energy back, so the pipe **keeps ringing between firings**. That ringing is what converts a train of separate pulses into a continuous engine note. Pipe lengths set the pitch; the muffler's width sets how quiet or muffled it is; how hot the gas is changes the pitch slightly; how much the open end dulls the highs sets how raspy it is. Dual exhausts are two separate pipes sent to the left and right speakers.

**Step E: Extra sources.** Intake roar and suction pulses, a throttle hiss only when air really flows past a nearly closed throttle, valve tick, the starter whine and solenoid clunk, road and driveline noise.

**Step F: The final mix.** Early reflections give a sense of room. Tone shelves, a compressor and a soft limiter shape the loudness. Your personal **sound dials** (volume, bass, treble, ring, rasp, lope, pops and so on) are applied here; they are listening preferences and are saved in the browser, not part of the engine.

**Why it feels connected to the engine.** The engine speed is smoothed so the pitch glides rather than steps. A small speed ripple is added: each cylinder's firing nudges the crank speed a little, so a light flywheel or few cylinders produces lumpier sound than a heavy flywheel or many cylinders, just like real life.

---

## 12. What is simplified, and what that means for the numbers

Knowing these tells you which behaviours to trust.

- **By default combustion is one idealised calculation per tick, not a degree-by-degree pressure trace.** The torque you see is a smooth average, not the pulses of a real engine, and the cylinder-pressure plot is drawn from a separate, simplified trace for display. With `use_resolved_cylinders` on (§2.5) both change: the torque pulses with the firing, and the pressure plot shows what cylinder 1 actually recorded, gas exchange included.
- **The degree-by-degree engine is the expensive one.** It shows real gas exchange, backpressure and measured volumetric efficiency, and the dashboard keeps it in step with the clock by coarsening its steps at high rpm (the P–θ plot says so when it does). It has no pressure waves in the pipes, so high-rpm torque is lower than a tuned real engine's. It does model misfires, variable valve timing, the burned and unburned halves of the flame and a turbocharger made of real parts. Without a C compiler it keeps up with the clock only by coarsening its steps at high rpm; with one it does not need to.
- **The numbers are plausible, not calibrated.** Torque and power are in the right range for each engine class and respond in the right direction, but they have not been checked against a measured dyno curve of any real engine. The dyno curve shows what *this model* produces.
- **Idle manifold pressure reads somewhat low** compared with a real engine, on the default path because the heat-transfer model that would correct it does not participate in the active energy balance (§3, Step 5); the resolved core includes real wall loss and a misfire limit, so it idles at about 23 kPa — closer, though still a bit below the ~30 kPa of a real engine, because crevice and blow-by losses are not modelled.
- **The car is simple.** One clutch, one gearbox, drag and rolling resistance and brakes. No tyre slip, no flexing driveline, no hills, no reverse.
- **Engine temperature does not change.** No warm-up, no coolant or oil temperature.
- **Diesel and emissions are rough estimates.**
- **The sound is physically *motivated*, not physically *solved*.** Timing, strength and pipe resonances follow the simulation; the waveforms themselves are authored.
- **Several control behaviours are authored, not from the textbook:** idle control, decel fuel cut, the rev limiter, the starter, and the vehicle numbers. They are standard real-world behaviour, not invented physics, but the textbook does not specify them.

---

## 13. Where to look in the code

| If you want to understand… | Start here |
|---|---|
| The millisecond tick, in order | `engine_sim/engine.py`, method `update` (the numbered steps match section 3) |
| What counts as "an engine" (all numbers and switches) | `engine_sim/config.py` |
| Air in, fuel in | `air_model.py`, `manifold_model.py`, `fuel_model.py`, `fuel_film.py`, `lambda_controller.py` |
| Torque from combustion, spark, knock | `combustion_model.py`, `spark_model.py` |
| Heat loss, friction | `heat_transfer.py`, `friction_breakdown.py` |
| Clutch, gearbox, vehicle, dyno | `engine_sim/powertrain.py` |
| Turbo | `turbo_model.py` (cycle-averaged), `turbo_system.py` + `exhaust_system.py` (degree-by-degree) |
| The compiled inner loop | `engine_sim/native_core.py`, `engine_sim/native/enginecore.c` |
| Two-zone flame, misfire, valve timing | `engine_sim/cylinder.py` |
| How an engine file becomes config | `engine_sim/engine_file.py` |
| How an engine file becomes sound inputs | `engine_sim/sound_spec.py` |
| The real-time loop and the API | `interactive_dashboard.py` |
| Keys, controls, dials, plots, dyno test | `dashboard/app.js` |
| Tooltips, knobs, sound presets | `dashboard/ui.js` |
| The sound synthesizer | `dashboard/engine-audio.js` |
| The tests that pin the behaviour above | `tests/` |
