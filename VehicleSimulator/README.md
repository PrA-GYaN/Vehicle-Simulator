# Vehicle Simulator

A small, configurable 3D vehicle dynamics sandbox in C++17.

Drive a vehicle around a test track with a chase camera, a full ImGui debug
interface, real-time graphs, telemetry logging to CSV, and a scripted
experiment framework.

Two physics models ship, switchable at runtime:

- **`four_wheel`** *(default)* — four individual tyres. A true combined-slip
  tyre model, per-wheel vertical loads with longitudinal and lateral weight
  transfer, body roll, camber, slip angles, slip ratios, vehicle sideslip,
  wheel lock and wheel spin.
  See **[The four-wheel tyre-force model](#the-four-wheel-tyre-force-model)**.
- **`kinematic_bicycle`** — the geometric model. The vehicle goes exactly where
  its wheels point. Kept because it is a *valid* description at low speed, not
  because it is a lesser version of the other one.

### Every equation here comes from one place

The physics is derived **exclusively** from the Knowledge Base in this
repository — `../VEHICLE_DYNAMICS_KNOWLEDGE_BASE.md` and its companion
`../Tireand suspension.md` — themselves built from three sources:

- **[S1]** Gillespie, *Fundamentals of Vehicle Dynamics*, SAE R-114, 1992
- **[S2]** Blundell & Harty, *The Multibody Systems Approach to Vehicle
  Dynamics*, 2004
- **[S3]** Milliken & Milliken, *Race Car Vehicle Dynamics*, SAE R-146, 1995

Nothing in the physics is a formula from general knowledge, a tuned constant,
or a game-physics shortcut. Where the Knowledge Base does not contain enough to
implement something, it is **not implemented and not guessed** — it is written
down in **[Open Questions](#open-questions)** instead.

### What the third source changed

[S3] arrived after the four-wheel model was already working, and it closed
three of the things this README previously had to list as unresolved. The
changes were not cosmetic:

| Was | Now |
|---|---|
| Two pure-slip curves with the **friction circle applied afterwards as a clamp**, and an admitted arbitrary rule for splitting the excess (KB gap **G1**, the largest recorded) | A genuine **combined-slip model**. G1 is **RESOLVED**. The friction ellipse is now an *identity* of the algebra — nothing is clamped and no allocation rule exists to be arbitrary about |
| Slip angle at peak lateral force **authored**, read off a chart | **Derived** from the cornering stiffness and the universal curve |
| Longitudinal slip stiffness `k_x` **authored**, with no source value at all | **Derived** from the KB's quoted peak slip ratio and the same curve |
| **No roll model.** Roll angle absent, camber structurally zero, and the front/rear split of lateral load transfer an authored bare fraction | The full chain: ride frequency → ride rate → roll rate → roll gradient → load transfer in **three** terms. The split is now computed, and body roll and camber exist |
| Anti-roll bars absent (KB gap **G14**) | **RESOLVED** — a separate contributor to axle roll rate |

Three parameters that used to carry a "not a Knowledge Base value" flag are
gone. The ones that remain are listed below and flagged in the YAML.

---

## What's new since the previous version of this README

The last revision of this file (21 Sep 2026) predates a substantial amount of
work. Everything below is in the code now; the detailed sections further down
have been updated to match, and stale statements (e.g. "no shadows", "cannot
pull away from a dead stop", "the track is decorative") have been corrected.

### Apex Grand Prix Circuit (12 Oct 2026)

| Change | Where |
|---|---|
| **New circuit `apex_gp`** — an original 4.93 km, 13-corner clockwise Grand Prix layout with a pit lane. Authored from straights and clothoid-eased corners; two straights are solved for so the lap closes exactly | `environment/circuits/ApexGrandPrix.cpp`, `Track::appendCorner` |
| **Photographed surface materials** — ten CC0 texture sets with normal and roughness maps, and trees built from leaf cards | `assets/textures/venue/`, `tools/bake_venue_textures.py`, `rendering/Texture.{h,cpp}` |
| **A complete venue** for it — pit building, race control, paddock, grandstands, barriers, catch fencing, tyre walls, signage, terrain, trees, hills | `rendering/ApexVenue*.{h,cpp}` |
| **Venue rendering path** — chunked and frustum-culled static geometry, instancing, distance LOD, per-vertex materials, linear-light shading with tone mapping, an analytic sky, a second wide shadow map. Used by `apex_gp` only; every other track draws exactly as before | `rendering/Venue.{h,cpp}`, `assets/shaders/basic.frag`, `sky.{vert,frag}` |
| **`--tour <dir>`** (screenshots and frame times all the way round a track), **`--fullscreen`**, and **`lap_test`** (an autopilot lap through the real physics) | `application/Application.cpp`, `tools/lap_test.cpp` |

See [The Apex Grand Prix Circuit](#the-apex-grand-prix-circuit).

### Resolved engine (12 Oct 2026)

EngineSim's crank-angle-resolved engine (its v0.7) now runs natively in this
simulator, alongside the cycle-averaged model that was ported earlier. See
**[The detailed engine + gearbox model](#the-detailed-engine--gearbox-model--how-the-engine-is-connected-to-the-car)**.

| Change | Where |
|---|---|
| **Resolved engine core** — every cylinder's gas integrated over crank angle; manifold pressure, volumetric efficiency, pumping, back-pressure, boost and turbo lag are outputs. EngineSim's C cylinder core is vendored unchanged; its Python engine is transcribed around it. Opt in with a `resolved:` block inside `detailed:` | `physics/ResolvedEngine.{h,cpp}`, `physics/enginecore/`, `config/ResolvedEngineParameters.h` |
| **Checked against EngineSim itself** — bit-identical on all 7250 steps of a scripted run, for three engines | `tools/engine_parity_reference.py`, `tools/engine_parity.cpp` |
| **`sports_car` and `ferrari_f40` moved to the resolved engine**, with valve gear, plenums and (F40) twin turbochargers. Their torque curves changed — see *Calibration, stated honestly* | `config/vehicles/` |
| **Engine dyno** — a private copy of the engine held at each speed until it settles. Command-line tool, and the source of the Powertrain panel's torque curve for resolved engines | `physics/EngineDyno.{h,cpp}`, `tools/engine_dyno.cpp` |
| **EngineSim engine file → vehicle YAML** converter; the `resolved:` keys are EngineSim's own | `tools/engine_toml_to_yaml.py` |
| **Rev-matched launch** for the automatic clutch (`launch_rpm`), needed once idle torque became realistic | `config/GearboxParameters.h`, `physics/Powertrain.cpp` |
| **Powertrain panel**: engine internals (manifold, exhaust, lambda, spark, IMEP, idle valve, cam advance, turbo speed, wastegate) and cylinder 1's live pressure trace | `ui/DebugUI.cpp` |
| **Telemetry**: `engine_map_pa`, `engine_boost_pa`, `engine_lambda` appended | `telemetry/Telemetry.cpp` |
| **Friction coefficients**: this project was still carrying EngineSim's pre-v0.4 unit slip (15 / 0.005 instead of 900 / 18), and clamping anything larger. Resolved engines use the corrected values; the cycle-averaged defaults are unchanged | `config/EngineParameters.h`, `config/Config.cpp` |
| **EngineSim's engine sound** — its synthesiser (firing events → blowdown and displacement pulses → a Kelly–Lochbaum exhaust waveguide, plus intake, starter, overrun pops) transcribed to C++ and checked sample-for-sample against the JavaScript original. Opt in with `sound: engine_sim:`; both resolved-engine cars use it. Turbo, gearbox, tyres, wind and the listener stay this project's. Fourteen dials and six presets in the Sound panel. See **[Sound](#sound)** | `audio/EngineSimSynth.{h,cpp}`, `tools/engine_sound_*` |
| **Build now needs a C compiler** as well as C++ (for the vendored core) | `CMakeLists.txt` |

### Vehicles

| Change | Where |
|---|---|
| **Ferrari F40** added — the project's own Blender model (`../F40_Render`), exported for the simulator by `../F40_Render/export_sim.py`, with the detailed engine, a **twin-turbo** model and a 5-speed gearbox | `config/vehicles/ferrari_f40.yaml`, `assets/models/ferrari_f40.glb` |
| **Mercedes-AMG GT3** (Red Bull livery) added as an imported model; wheelbase, tracks and wheel radius **measured** from the model's wheel-hub nodes | `config/vehicles/gt3_redbull.yaml` |
| **Imported glTF/GLB vehicle models**, with per-wheel spin/steer from the physics and a measured (not authored) rear-axle origin | `src/rendering/GltfModel.{h,cpp}`, `model:` YAML block |
| **Staggered tyres drawn correctly** — a model's front hubs may sit lower than its rear (F40: 0.314 m vs 0.333 m); the loader measures the difference and the renderer applies it to the model's own wheels. Render-only; the physics still has one radius | `GltfModel::hubHeightOffset` |
| **Procedural car body** — the generated car is now a lofted body with wheel arches cut at the real axle positions, in five styles (`auto`, `race`, `sports`, `saloon`, `suv`) | `src/rendering/CarMesh.{h,cpp}`, `model: body_style:` |

### Powertrain

| Change | Where |
|---|---|
| **Launch from a dead stop** — the driveline is now three-state (locked / slipping / released) with a friction clutch that transmits its capacity while slipping | `physics/Powertrain.{h,cpp}` |
| **Manual shifting and a manual clutch**, independently selectable (`shift_mode`, `clutch_mode`); `Z`/`X` shift, `Left Shift` is the clutch pedal. Anti-stall releases the clutch below a fraction of idle | `config/GearboxParameters.h`, `input/Input.cpp` |
| **Turbocharging** (opt-in `turbo:` block): Heywood Eq 6.40 compressor heating, intercooler effectiveness, boost-dependent exhaust back-pressure, spool curve and turbo lag — see **[Turbocharging](#turbocharging)** | `config/EngineParameters.h`, `physics/Engine.cpp` |
| **`ve_reference_rpm`** — the speed at which volumetric efficiency falls away is now per-engine instead of hard-coded at 6000 rpm (default unchanged) | `config/EngineParameters.h` |
| **Clutch bug fixed**: slipping-clutch torque now follows the slip direction. Previously it always loaded the engine, so an engine that could not out-rev its clutch (a turbo engine off-boost) never locked. The sports car's acceleration-test telemetry is byte-identical before and after | `physics/Powertrain.cpp` |
| **Powertrain UI panel** — gear, rpm bar, torque/power, clutch state, a live WOT torque/power curve with bmep, and boost/charge temperature for turbo engines | `ui/DebugUI.cpp` |

### Tracks, terrain and suspension

| Change | Where |
|---|---|
| **Track presets**: `test_track`, `race_circuit`, `nurburgring` (GP-Strecke, 3.95 km, centreline measured off the reference scan), `nordschleife` (20.8 km from OpenStreetMap, **with real elevation that is driven on**), `straight`, `nurburgring_scan` (the raw scan). `--track <name>` or `default_track:` | `environment/Track.{h,cpp}`, `environment/data/` |
| **Generated circuit scenery** — asphalt, edge lines, kerbs, runoff, armco, start line and grid, gantry, grandstands, all derived from the physics centreline | `rendering/TrackMesh.{h,cpp}` |
| **Terrain height for the physics** — a height field built from an imported scene (or the Nordschleife centreline) feeds the tyres real ground height | `environment/TerrainHeightField.{h,cpp}` |
| **Per-wheel quarter-car suspension** ([S3] Ch.22) — sprung and unsprung masses per corner on the ride spring, damper and tyre spring; new `damping_ratio_front/rear` parameters. An exact no-op on flat ground | `physics/FourWheelModel.cpp` |
| **Body attitude rendered** — terrain heave, pitch and roll plus the quasi-static cornering roll are now drawn | `rendering/Renderer.cpp` |
| **Offline centreline tools** | `tools/extract_track_centreline.cpp`, `tools/fetch_nordschleife_centreline.py` |

### Rendering and input

| Change | Where |
|---|---|
| **Shadows** — one 2048² directional shadow map fitted around the car and snapped to texels | `rendering/ShadowMap.{h,cpp}` |
| **Textures** (base colour from imported models) | `rendering/Texture.{h,cpp}` |
| **Ten camera views** — chase, chase far, cockpit, bonnet, bumper, wheel, TV trackside, helicopter, free, top-down (`V`, `1`–`0`). See **[Cameras](#cameras)** | `rendering/Camera.{h,cpp}` |
| **Sound synthesised from the physics** — engine, exhaust, intake, turbo, blow-off, gearbox, tyres, wind, with Doppler. See **[Sound](#sound)** | `src/audio/` |
| **See-through glass** on imported models (glTF `BLEND`, factor or texture alpha) | `rendering/GltfModel.cpp`, `Renderer.cpp` |
| **Android USB controller** — see below | `input/AndroidUsbInput.{h,cpp}` |

---

## Quick start

```bash
cd VehicleSimulator
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j

./build/bin/VehicleSimulator
```

Then drive with **W / A / S / D**.

If the panels cover too much of the view, press **F1** to hide them, or start
with `--hide-ui`.

---

## Requirements

| Component | Version | How it is obtained |
|---|---|---|
| C++ compiler | C++17 | system |
| C compiler | C99 | system — for EngineSim's cylinder core (`src/physics/enginecore/enginecore.c`) |
| CMake | ≥ 3.20 | system |
| OpenGL | 3.3 core | system driver |
| GLM | any recent | system package, else fetched |
| yaml-cpp | any recent | system package, else fetched |
| GLFW | 3.4 | system package, else fetched |
| Dear ImGui | v1.92.9b | always fetched |

GLM, yaml-cpp and GLFW are used from your system if CMake can find them, and
downloaded automatically if it cannot. ImGui is always fetched — there is no
sensible system package for it. **The first configure needs network access**
unless everything is already installed.

Python (3.11+, standard library only) is **optional**: the simulator neither
embeds nor runs it. It is needed only for the engine tools that read EngineSim
(`tools/sync_enginecore.py`, `engine_toml_to_yaml.py`,
`engine_parity_reference.py`, `engine_sound_reference.py`) — and if CMake
finds both Python and `../EngineSim`, configure checks that the vendored engine
core is current. The sound check additionally needs **Node**, to run
EngineSim's audio worklet offline.

There is no GLEW or GLAD dependency: `src/rendering/GLLoader.h` declares the
~45 OpenGL entry points the renderer actually uses and resolves them through
`glfwGetProcAddress`.

Installing the optional system packages makes the first build much faster:

```bash
# Arch
sudo pacman -S glfw glm yaml-cpp cmake ninja

# Debian / Ubuntu
sudo apt install libglfw3-dev libglm-dev libyaml-cpp-dev cmake ninja-build
```

### Offline / air-gapped builds

Point CMake at pre-downloaded sources:

```bash
cmake -S . -B build \
  -DFETCHCONTENT_SOURCE_DIR_GLFW=/path/to/glfw \
  -DFETCHCONTENT_SOURCE_DIR_IMGUI=/path/to/imgui
```

### Build options

| Option | Default | Effect |
|---|---|---|
| `VS_USE_SYSTEM_GLM` | `ON` | prefer system GLM |
| `VS_USE_SYSTEM_YAMLCPP` | `ON` | prefer system yaml-cpp |
| `VS_USE_SYSTEM_GLFW` | `ON` | prefer system GLFW |

---

## Running

```bash
./build/bin/VehicleSimulator                          # drive the default car
./build/bin/VehicleSimulator --vehicle sports_car     # pick a vehicle
./build/bin/VehicleSimulator --vehicle ferrari_f40 --track nordschleife
./build/bin/VehicleSimulator --track apex_gp          # the Apex Grand Prix Circuit
./build/bin/VehicleSimulator --vehicle ferrari_f40 --camera cockpit
./build/bin/VehicleSimulator --hide-ui                # clean 3D view
./build/bin/VehicleSimulator --experiment slalom      # watch a scripted run
./build/bin/VehicleSimulator --help
```

`config/` and `assets/` are copied next to the executable at build time, so it
runs from any working directory.

### Headless experiments

Runs with no window, records telemetry at the physics rate, writes a CSV, exits.
This is the hook for automated vehicle-dynamics regression tests.

```bash
./build/bin/VehicleSimulator --headless --experiment braking_test --duration 6
# -> logs/run_001.csv
```

| Flag | Meaning |
|---|---|
| `--config <path>` | simulation config (default `config/simulation.yaml`) |
| `--vehicle <name>` | resolved as `config/vehicles/<name>.yaml` |
| `--track <name>` | `test_track`, `race_circuit`, `nurburgring`, `nordschleife`, `straight`, `nurburgring_scan` |
| `--camera <view>` | `chase`, `chase_far`, `cockpit`, `bonnet`, `bumper`, `wheel`, `tv`, `helicopter`, `free`, `top_down` |
| `--audio-out <file.wav>` | with `--headless`: also render the run's synthesised sound to a WAV |
| `--experiment <type>` | see the experiment list below |
| `--duration <s>` | override the experiment duration |
| `--headless` | no window; requires `--experiment` |
| `--hide-ui` | start with panels hidden |

---

## Controls

| Key | Action |
|---|---|
| `W` / `Up` | throttle |
| `S` / `Down` | brake |
| `A` / `Left` | steer left |
| `D` / `Right` | steer right |
| `Space` | emergency brake |
| `X` / `Z` | shift up / down (detailed-engine vehicles; overrides an automatic box) |
| `Left Shift` | clutch pedal (when `clutch_mode: manual`) |
| `R` | reset vehicle |
| `P` | pause / resume |
| `N` | single physics step (while paused) |
| `T` | toggle telemetry recording |
| `F2` | export telemetry to CSV |
| `F1` | toggle debug panels |
| `C` | reset camera |
| `V` | cycle camera view |
| `1` … `0` | chase, chase far, cockpit, bonnet, bumper, wheel, TV, helicopter, free, top-down |
| `M` | mute / unmute sound |
| `G` | traction control level: 1 → 2 → 3 → 1 |
| `H` | traction control on / off (remembers the level) |
| `Y` | fit a new set of tyres: Standard → Soft → Medium → Hard |
| `Esc` | quit |
| Mouse drag | orbit (chase views) / turn your head (car-mounted views) |
| Scroll | zoom (chase) / field of view (car-mounted views) |
| `I J K L` + `Q E` | move the free camera |

Steering does **not** snap to the requested angle. The road wheels slew toward
it at a configurable rate (`controls.steering_rate`), as a real steering system
would.

---

## Traction control and tyre compounds

Both are also in the **Vehicle** panel, and both show in the HUD.

### Traction control — off, or level 1 to 3

`G` cycles the level, `H` switches it on and off. A vehicle file can set the
level it starts with (`traction_control: 0..3`, default 0).

It watches the driven wheels and closes the throttle when they spin past a
slip target — the same place a real system acts, by cutting engine torque, so
it behaves identically with the simple drive model and the engine + gearbox.
The targets are multiples of the tyre's own peak-slip parameter:

| Level | Slip target | Character |
|---|---|---|
| 1 | 1.35 × peak | lets the tyres run a little past the peak; quickest |
| 2 | 0.90 × peak | holds them at the peak |
| 3 | 0.55 × peak | keeps them well short of it; safest, slowest off the line |

Standing start, flat out, `gt3_redbull` (`lap_test --launch --tc N`):

| | 0–100 km/h | speed after 12 s | time spent past the peak |
|---|---|---|---|
| off | 5.23 s | 217 km/h | 11.8 s |
| 1 | 4.65 s | 242 km/h | 11.1 s |
| 2 | 5.22 s | 236 km/h | 0.8 s |
| 3 | 6.74 s | 201 km/h | 1.4 s |

It is a PI controller on slip, with gains tuned by hand against that test.
It limits wheelspin only: it does not brake individual wheels and does not
look at yaw or slip angle, so it is **not stability control** and will not
catch a slide that is not caused by wheelspin. It needs wheel speeds, so it
does nothing under the kinematic model.

### Tyres — Soft, Medium, Hard

`Y` fits a new set of the next compound (`tire: compound: soft` in a vehicle
file sets the one it starts on). The default, **Standard**, is the tyre
exactly as the vehicle file describes it, with no wear — what every car had
before, and numerically unchanged.

| Compound | Grip | Life (sliding work per tyre) |
|---|---|---|
| Soft | +10 % | 6 MJ |
| Medium | +5 % | 12 MJ |
| Hard | +0 % | 21 MJ |

Grip is a factor on that car's own friction coefficient. Each tyre wears
separately, by the energy its contact patch dissipates in sliding (force ×
sliding speed): rolling freely wears nothing, cornering wears steadily,
wheelspin and lock-ups wear fast. Grip fades about 5 % over the tyre's life
and then drops a further 22 % over the last fifth. Reset (`R`) fits new tyres.

**These numbers are authored.** Unlike the rest of the tyre model, nothing
here is traced to the Knowledge Base, which has no compound or wear data;
the values were chosen to give the trade the three-compound system is known
for. And there is **no tyre temperature**: no warm-up, overheating, graining
or blistering, which in reality separate the compounds as much as outright
grip does. See `config/TireCompound.h`.

## Android controller (optional)

An Android phone can act as a physical controller over the USB cable: tilt to
steer, two large touch pedals for throttle/brake, and a handbrake button. See
[`../AndroidController/README.md`](../AndroidController/README.md) for the
app and [`../AndroidController/PROTOCOL.md`](../AndroidController/PROTOCOL.md)
for the wire format both sides speak.

It is disabled by default and never affects a keyboard-only setup. Enable it
in `config/simulation.yaml`:

```yaml
android_input:
    enabled: true
    port: 7788               # must match the app if you change it
    timeout_seconds: 0.5
    auto_adb_reverse: true    # runs `adb reverse tcp:7788 tcp:7788` for you
```

`src/input/AndroidUsbInput.{h,cpp}` is a TCP server that runs on its own
thread; `Application` treats it as just another input source (see
`Application::activeControlInput()`/`activeHandbrake()`), used automatically
whenever it is connected and falling back to keyboard input the instant it
isn't. The **Android Controller** panel in the debug UI (and a HUD line at
the top right) show live connection state, packet counts and the exact
values the vehicle is receiving.

If communication drops - cable unplugged, app killed, phone put to sleep -
the vehicle is forced to a safe state (throttle 0, brake 0, steering
centred, handbrake off) rather than continuing to drive on the last value it
heard. There is no path back to keyboard control that skips this: the safety
check lives in `AndroidUsbInput` itself, not in `Application`.

---

## Architecture

```
Application
    │
    ├── Renderer ────── Camera, DebugDraw, Shader, Mesh, MeshFactory,
    │                   CarMesh, GltfModel, TrackMesh, ShadowMap, Texture
    ├── Input ───────── keyboard, optional Android USB controller
    ├── UI (ImGui)
    ├── Audio ───────── SoundSynth (from engine/tyre/camera state), miniaudio device
    ├── Simulation ──── fixed-timestep accumulator
    │      │
    │      └── Vehicle ──── control mapping, actuator rate limits, geometry
    │             │
    │             └── IVehicleDynamics
    │                    ├── FourWheelModel ─┬── TireModel        (one tyre)
    │                    │                   ├── SuspensionModel  (roll chain)
    │                    │                   └── quarter car per wheel (terrain)
    │                    │
    │                    └── (optional) Powertrain ── Gearbox, clutch, and an Engine:
    │                                                   cycle-averaged, or ResolvedEngine
    │                                                   (EngineSim's cylinder core + ECU)
    │                    └── KinematicBicycleModel
    │
    ├── Environment (Track, TerrainHeightField)
    ├── Configuration (YAML)
    └── Telemetry
```

### The physics does not depend on rendering

This is enforced by the build, not by convention. `CMakeLists.txt` defines two
targets:

- **`vsim_core`** — config, physics, vehicle, simulation, environment,
  telemetry, experiments. Links against **GLM and yaml-cpp only**. (The
  resolved engine's cylinder core is a C source file compiled into it, not a
  library.)
- **`VehicleSimulator`** — rendering, input, UI, application. Links against
  `vsim_core` plus GLFW, ImGui and OpenGL.

If anyone ever includes a rendering header from the simulation side, the link
fails. The renderer only ever receives `const Simulation&` / `const Vehicle&`,
so deleting it entirely would leave the physics untouched.

### Source layout

```
src/
  core/          Coordinates.h (conventions), Units.h, VehicleState.h,
                 WheelState.h (per-wheel tyre state + sign conventions)
  config/        Config.{h,cpp}, VehicleParameters.h, TireParameters.h,
                 SuspensionParameters.h, EngineParameters.h,
                 ResolvedEngineParameters.h, GearboxParameters.h
  physics/       IVehicleDynamics.h, factory,
                 TireModel.{h,cpp}          <- one tyre, KB §2 / §2.20
                 SuspensionModel.{h,cpp}    <- the roll chain, KB §3.5.1
                 FourWheelModel.{h,cpp}     <- the vehicle, KB §1 / §3
                 KinematicBicycleModel.{h,cpp}
                 SteeringGeometry.h, RoadLoad.{h,cpp}
                 Engine.{h,cpp}, Gearbox.{h,cpp}, Powertrain.{h,cpp}
                 ResolvedEngine.{h,cpp}     <- EngineSim's engine, transcribed
                 enginecore/                <- EngineSim's C cylinder core, VENDORED
                                               (written only by tools/sync_enginecore.py)
                 EngineDyno.{h,cpp}         <- speed-holding dynamometer
  vehicle/       Vehicle.{h,cpp}
  simulation/    Simulation.{h,cpp}      <- fixed timestep lives here
  environment/   Track.{h,cpp}           <- pure geometry, no GL
                 TerrainHeightField.{h,cpp}  <- ground height for the tyres
                 data/                   <- measured centreline tables
  telemetry/     Telemetry.{h,cpp}
  experiments/   Experiment.{h,cpp}
  input/         Input.{h,cpp}, AndroidUsbInput.{h,cpp}
  rendering/     GLLoader, Shader, Mesh, MeshFactory, Camera, DebugDraw, Renderer,
                 CarMesh (generated body), GltfModel + GltfLoaderImpl (imported
                 models), TrackMesh (circuit scenery), ShadowMap, Texture,
                 Venue (chunked/instanced venue drawing), ApexVenue* (the
                 Apex Grand Prix venue generator)
  ui/            DebugUI.{h,cpp}
  audio/         SoundState.h, SoundStateBuilder (sim -> numbers), SoundSynth
                 (the synthesis), AudioOutput (miniaudio device),
                 EngineSimSynth (EngineSim's engine sound, transcribed)
  application/   Application.{h,cpp}
tools/           extract_track_centreline.cpp, fetch_nordschleife_centreline.py
                 (offline; their OUTPUT is committed under environment/data/)
                 engine_dyno.cpp, engine_parity.cpp, engine_parity_reference.py,
                 engine_sound_parity.cpp, engine_sound_reference.py,
                 engine_sound_render.mjs, engine_toml_to_yaml.py, sync_enginecore.py
                 (the engine tools - see the engine section)
```

---

## Coordinate system

Defined once, in **`src/core/Coordinates.h`**. Nothing anywhere else in the
project writes `cos(yaw), 0, sin(yaw)` by hand.

**World frame** (right-handed):

| Axis | Direction |
|---|---|
| `+X` | ground plane |
| `+Y` | **up** |
| `+Z` | ground plane |

**Yaw** is measured in the X–Z plane **from the +X axis**:

```
heading(psi) = ( cos(psi), 0, sin(psi) )
```

which follows directly from the model's `x_dot = v cos(yaw)`,
`z_dot = v sin(yaw)`.

> **Read this once:** at `yaw = 0` the vehicle points along **+X**, not +Z.
> +Z is a ground-plane axis but it is not the zero-yaw heading. Increasing yaw
> sweeps the heading from +X toward +Z.

**Vehicle local frame** — `+X` forward, `+Y` up, `+Z` right, origin at the
**rear axle**. All vehicle geometry is authored in this frame.

**Steering sign**: `delta > 0` steers **right** (heading rotates +X → +Z), and
the normalised steering input follows the same sign, so `+1` = full right.

**Units**: metres, kilograms, seconds, **radians** everywhere internally.
Degrees appear only in YAML files and in the UI, and are converted at the
boundary. See `src/core/Units.h`.

---

## The four-wheel tyre-force model

`src/physics/FourWheelModel.{h,cpp}`, `src/physics/TireModel.{h,cpp}` and
`src/physics/SuspensionModel.{h,cpp}`

### The chain, which is the Knowledge Base's own

KB Section 0.5 states the organising idea of the entire subject:

> almost everything in vehicle dynamics is a chain that begins with vertical
> load and ends with acceleration, with the tire in the middle.

`FourWheelModel::update()` is that chain, in that order:

```
operating condition (V, delta, throttle, brake)
  -> vertical load at each of four contact patches      KB Section 3
  -> tyre operating state (Fz, alpha, slip ratio)       KB Sections 1.5, 2.4, 2.6
  -> tyre forces and moments (Fx, Fy, Mz)               KB Sections 2.9, 2.17
  -> resultant force and moment about the CG            KB Eq. 1-1, Eq. 1-2
  -> accelerations
  -> vehicle states (position, heading, u, v, yaw rate, four wheel speeds)
  -> back to the top
```

The loop closes twice, exactly as KB Section 0.5 says it must: acceleration
causes weight transfer, weight transfer changes what the tyres can do, and what
the tyres do sets the acceleration.

### The tyre: one curve, and one scalar slip

The tyre model is the **nondimensional model** of KB §2.20 — [S3] RCVD
Chapter 14. Its idea is a single sentence from [S3] p.44:

> Although it is convenient to think in terms of longitudinal and lateral
> forces, and slip ratio and slip angle, **the tire only knows a resultant
> force and a resultant slip velocity.**

Divide every quantity by the friction-limited force and the initial stiffness
and the load dependence divides out, leaving **one universal curve** that fits
normalised lateral force, normalised camber thrust, normalised longitudinal
force *and* the normalised combined resultant. [S3] p.479: *"once the data are
made dimensionless, 'one curve fits all.'"*

```
normalise      ᾱ = C_α tan α /(μ_y F_z)      S̄ = k_x S /(μ_x F_z)
               γ̄ = C_γ sin γ /(μ_y F_z)                  (14.4, 14.8, 14.10)

camber         β̄ = ᾱ /(1 − γ̄ sgn α)                              (14.13)

resultant      k = √(S̄² + β̄²)          R = R(k)              (14.15, 14.16)
               R is the universal curve:
               R = D′ sin[C′ arctan(B′φ)],
               φ = (1−E′)k + (E′/B′) arctan(B′k)          (14.5)–(14.7)
               with B′=0.714, C′=1.40, D′=1.00, E′=−0.20   ([S3] p.476)

split          η₀ = C_α μ_x /(k_x μ_y)                             (14.19)
               η(k) = ½(1+η₀) − ½(1−η₀)cos(k/2),  |k| ≤ 2π         (14.18)
               F̄   = η R tan α /√(S² + η² tan²α)                   (14.20)
               F̄_x =   R   S   /√(S² + η² tan²α)                   (14.21)

restore        F̄ = FN(1 − γ̄ sgn α) + γ̄        F_y = F̄ μ_y F_z, F_x = F̄_x μ_x F_z
```

**The friction ellipse is now an identity, not a clamp.** Squaring and adding
(14.20) and (14.21) gives `F̄² + F̄_x² = R²` exactly, and `R` never exceeds
`D′ = 1`, so

```
(F_y /(μ_y·F_z))²  +  (F_x /(μ_x·F_z))²   ≤  1
```

holds by construction. The previous version of this model computed two
pure-slip forces and then scaled them back onto the circle, and had to admit
that the Knowledge Base did not say how to allocate the excess. That admission
is gone: there is nothing left to allocate. When `μ_x = μ_y` the ellipse is
exactly [S1] §2.9's "pure-circle case".

Two parameters that used to be authored now fall out of the same curve. It
peaks at a known normalised slip (2.678 for [S3]'s coefficients), so:

- **the slip angle at peak lateral force** = `arctan(2.678·μ_y·F_z / C_α)`,
  which for the shipped road tyre is 15.7° — against the "roughly ten to
  fifteen degrees" [S1]'s charts show (KB §2.6.2), and for the sports car's
  stiffer tyre 9.6°, heading toward the 3–7° [S3] quotes for race tyres;
- **the longitudinal slip stiffness** `k_x = 2.678·μ_x·F_z / S_peak`, from the
  peak slip ratio the KB *does* quote (0.10–0.15).

### The roll chain

`SuspensionModel` is [S3] Chapters 16 and 18, via KB §3.5.1:

```
ride frequency  →  ride rate    K_R = (2πf)² m_corner
                →  roll rate    K_φ = K_R t²/2   (+ anti-roll bar)
                →  roll gradient  φ/A_Y = −W_s h₂ /(K_F + K_R − W_s h₂)
                →  lateral load transfer, in three terms
```

The `−W_s h₂` in the denominator is the destabilising gravity moment both [S1]
(Eq. 6-32) and [S3] (p.682) carry: roll stiffness must *exceed* it for the roll
angle to be finite at all. The UI says so in red when it does not.

Load transfer has **three** mechanisms, not two:

| Term | Path | Set by |
|---|---|---|
| elastic | through the springs and bars — needs the body to actually roll | roll rates, sprung CG height above the roll axis |
| geometric | through the links at the roll centre — instantaneous | roll centre heights |
| unsprung | the unsprung mass about its own CG height | unsprung weights |

[S1] Eq. 6-27 has the first two; [S3] adds the third. And the number this
produces is deliberately **not** the roll-stiffness ratio — [S3] p.679 calls
that confusion a named error, worth ten percentage points in its own example.
Both are shown in the UI, and for the shipped saloon they differ by six.

### What each piece is, and where it comes from

| Behaviour | Model used | Knowledge Base source |
|---|---|---|
| Longitudinal slip | `S = (V − R_e·ω)/V`; `S_[S3] = −S_[S1]` | Eq. 10-1, Eq. 3-20, `Tire…md` §3.3 |
| Effective rolling radius | `R_e = R_l + 2·δ_z/3` | Eq. 5.11 [S2], §2.15 |
| Slip angle | angle between wheel travel and the wheel plane, in the tyre axis system | §1.5 |
| **Tyre forces, pure and combined** | **nondimensional model, Eqs. (14.1)–(14.21)** | **§2.20 [S3]** |
| — the universal curve | `B′,C′,D′,E′ = 0.714, 1.40, 1.00, −0.20` | §2.20.2 [S3] p.476 |
| — friction ellipse | an identity of (14.20)/(14.21), not a clamp | §2.20.3 |
| Cornering stiffness vs load | `C_α = a·F_z − b·F_z²` | Eq. 6-35/6-36 |
| Camber stiffness | `C_γ/C_α` = 1/10 radial, 1/5–1/6 bias | §12.2 [S3] |
| Friction vs load | `μ` falls 0.01 per 10 % load increase | §2.4.3 |
| Aligning moment | `M_z = −F_y·t`, `t` derived from the aligning-moment coefficient | §2.8 |
| Tyre transient | first-order lag, `τ = (relaxation length)/V` | §2.11, §2.19 |
| Longitudinal load transfer | `ΔW = W·(a_x/g)·(h/L)` | Eq. 1-8/1-9; [S3] p.684 agrees |
| **Sprung/unsprung split** | 10 % of GAWR non-driven, 15 % driven | §1.6 [S1] |
| **Sprung CG and roll moment arm** | `h_s`, `h_RM` | §3.5.1, [S3] p.603 |
| **Ride rate, roll rate, roll gradient** | `K_R`→`K_φ`→`φ/A_Y` | §3.5.1, [S3] pp.588–589, 682 |
| **Anti-roll bar** | separate contributor to axle roll rate | `Tire…md` §19 (gap G14) |
| **Lateral load transfer** | three terms, elastic + geometric + unsprung | §3.5.1 [S3] p.682 |
| **Camber** | `γ_g = γ_b + φ` | Eq. 6-50 |
| Wheel rotation | `I_w·ω̇ = T_drive − T_brake − M_roll − F_x·R_l` | Eq. 3-18, §2.5.6 |
| Rolling resistance | applied as the moment it actually is | §2.5.6 |
| Brake proportioning | fixed front/rear split, ideal at one `μ` | Eq. 3-29/3-30, §6.8.2 |
| Steering | exact Ackerman onto two front wheels | Eq. 8-1/8-2 |
| Body equations of motion | `ΣF = m·a` in the global frame, `M_z = I_zz·ω̇` in the body frame | Eq. 1-1/1-2, Eq. 2.225/2.236 [S2], §1.11 |
| Sideslip | `β` = angle between heading and velocity vector | §1.3 |
| Sideslip rate | `β̇ = A_y/V − ω` | Eq. 7.29 [S2], §9.4.15 |
| Understeer ratio | `US = ω_geometric / ω_actual` | Eq. 7.16 [S2], §9.4.14 |

Rows in **bold** are new since the third source.

### Where the equations of motion did *not* come from

This is worth stating plainly, because it is the easiest place to smuggle in
something the sources do not contain.

KB gap **G8 is OPEN** and is called *"the most significant remaining gap"*:
neither source derives the transient yaw/sideslip equations of motion, and the
Knowledge Base says outright that a reader wanting the classical linear
bicycle-model state-space form `[v̇, ṙ] = A·[v, r] + B·δ` *"will not find it in
either source"*.

So that form is **not used**. What is used is the general rigid-body statement
both sources do give — `ΣF = m·a_G` and `M_z = I_zz·ω̇_z` — together with KB
Section 1.11's instruction on *where to write each one*: translation in the
fixed global frame, rotation in the body frame. That split is why the model
needs no hand-written rotating-frame transport terms: the world-frame velocity
vector is integrated directly, the body-frame components `u` and `v` are
recovered from it afterwards, and the apparent centrifugal behaviour appears on
its own because the heading rotates underneath the velocity vector.

### What emerges rather than being coded

None of the following is a special case, a hack or a tuned effect. Each is a
consequence of the chain above.

- **Understeer and oversteer.** No understeer gradient is applied to anything.
  The vehicle turns less than its steering geometry asks for because the front
  tyres need slip angle to make force, and because load transfer costs the
  loaded axle grip. Watch `US` in the Tyres panel rise with speed.
- **Wheel lock.** Brake torque exceeds what the tyre can react, the wheel
  decelerates, slip passes the peak of the μ-slip curve into the region KB
  Section 2.4.2 identifies as having **negative slope** — where more slip means
  *less* force resisting further wheel deceleration — and the wheel spins down
  to a stop. Set `peak_friction: 0.35` (KB's wet range) and brake hard.
- **The inside wheel locks first** under trail braking, because cornering has
  already taken load off it.
- **Wheelspin**, and the fact that sustained wheelspin is *slower* than
  modulated throttle, because past the peak the tyre delivers the sliding
  friction rather than the peak. Try full throttle in `sports_car`.
- **Sideslip changing sign with speed.** At low speed the model's `β` has one
  sign, above roughly 11 m/s in `basic_car` it has the other. KB Section 9.4.7
  calls this *"a real and observable phenomenon"* and gives its speed as
  Eq. 6-24; the model reproduces that speed to within a couple of percent
  without being told about it.
- **Braking eats cornering.** Braking and cornering forces come out of one
  resultant, so the harder you brake the less the tyre will turn you — and now
  by an amount the model computes rather than clamps.
- **Body roll, and the camber it causes.** With no camber compensation
  authored (no source quantifies it), the wheel leans with the body, so the
  loaded outside tyre gains positive camber and loses some of its lateral
  force. That is the real penalty a suspension exists to avoid, and it is
  visible in the Tyres panel's `gamma` column.
- **Balance responds to the anti-roll bars.** Stiffening the front bar moves
  load transfer forward and the vehicle toward understeer, which is the design
  rule of KB §9.4.9 arriving through the roll chain rather than being applied.

### Individual tyre model

`TireModel` is a separate module with the exact input and output list of the
tyre block diagram in KB §2.2. It is stateless — the one piece of tyre state
any source models, the relaxation-length lag, is passed in and returned — so a
single instance serves all four wheels and `update()` stays reproducible, which
is what `IVehicleDynamics` promises its callers.

Replacing the tyre model means replacing one file. Nothing in the vehicle model
reaches inside it, which is why swapping the [S2] per-load Magic Formula for
the [S3] nondimensional model touched no other physics file. The two are not in
conflict — KB §2.20.2 records that [S2] fits raw curves per load while [S3]
fits one curve for all loads — but only the second can do combined slip.

### Numerical notes

Explicit forward Euler throughout, matching the rest of the project, with one
exception: the **wheel rotational equation is integrated semi-implicitly**.
Linearised about the current slip its time constant is a couple of
milliseconds — far below a 10 ms physics step — so explicit Euler on it
diverges. One Newton step against the tyre's own local slope fixes it without
moving the answer: at equilibrium the net torque is zero, and the divisor
multiplies zero. KB gap **G35** records that neither source offers any guidance
on solver choice or stiffness.

Verified against timestep: `dt = 0.001` and `dt = 0.010` agree to three
significant figures on both a sub-limit and an at-the-limit step steer.

---

## The kinematic bicycle model

`src/physics/KinematicBicycleModel.{h,cpp}`

Retained and still selectable. It is not a lesser version of the four-wheel
model — KB Section 9.4.13 shows that below roughly 15 mph it is the steering
*geometry* rather than grip that limits path curvature, which is exactly what
this model describes.

```
x_dot   = v * cos(psi)
z_dot   = v * sin(psi)
psi_dot = (v / L) * tan(delta)
v_dot   = a
```

The two wheels of each axle are collapsed onto the centreline. The vehicle is
assumed to go exactly where its wheels point.

`psi_dot` comes from the Ackermann relation: under no slip the vehicle turns
about a point on the extension of the rear axle at radius `R = L / tan(delta)`,
so `psi_dot = v / R`. No mass, tyre or grip term enters — which is why the model
is cheap, and why it stops being valid at speed.

**The reference point is the rear axle, not the CG.** That is what makes
`psi_dot = v/L·tan(delta)` exact: the rear axle is the point that moves exactly
along the heading. The CG position is derived from it on demand.

### Assumptions — all of them

1. **No tyre slip.** No slip angle, therefore no lateral tyre force is computed.
2. **No sideslip.** Velocity is always parallel to the heading. A real car crabs
   in a corner; this one cannot.
3. **No load transfer.** `mass` is used for the road load force balance, but
   `cg_height` is not read at all by this model.
4. **No roll, pitch or suspension.** The body is rigid and always level.
5. **Planar motion only.** No terrain. Grade is applied as a gravity component,
   but the ground is still drawn flat.
6. **Instant force response.** Commanded acceleration is achieved exactly.
   There is **no traction limit** — the model never checks whether the tyres
   could actually transmit the demanded force.

Longitudinal **resistance** *is* modelled — see below.

### Validity

Good below roughly **0.3 g lateral**. Above that a real vehicle needs slip
angles to generate cornering force and its path departs from this prediction.

That 0.3 g boundary is not a rule of thumb: both sources arrive at it
independently (KB Sections 9.4.5 and 9.4.13), and [S2] states the tyre stays
*"substantially linear"* only below about 3 m/s².

While this model is selected, the HUD shows live lateral acceleration and turns
**orange with an explicit warning** once you exceed it. The model will happily
let you take a hairpin at 200 km/h — the absence of a complaint is not
validation. Switch to `four_wheel` and it will complain by not making the
corner.

### Road load — why the car slows down when you lift off

`src/physics/RoadLoad.{h,cpp}`

Without these terms the acceleration command is `drive + brake` and nothing
else, so releasing the throttle leaves `a = 0` and the vehicle coasts at
constant speed forever. That is Newton's first law applied correctly to an
**incomplete force balance**, not a bug — but it is not a car.

Four resisting terms are modelled:

| Term | Relation | Source |
|---|---|---|
| Rolling resistance | `R_x = f_r·W`, with `f_r = f_r0·(1 + V_mph/100)` | Gillespie Eq. 4-12, 4-14 |
| Aerodynamic drag | `D_A = ½·ρ·V²·C_D·A` | Gillespie Eq. 4-2 |
| Grade | `R_g = W·sin θ` (signed — downhill assists) | Gillespie Eq. 3-14 |
| Engine braking | `a = k·V·(1 − throttle)` | **not from Gillespie** — see below |

Rolling resistance is **not** tyre grip. It is hysteresis loss as the carcass
and tread are deformed through the contact patch, plus scrubbing and road
deflection. It is nearly speed-independent. Aerodynamic drag goes as `V²`, so
it is negligible at parking speed and dominant on the motorway — the two cross
at **~50 mph** for these vehicles, matching the 50–60 mph Gillespie quotes.

Two deliberate deviations from the source, both flagged in the code:

- The book's literal `0.01` in Eq. 4-14 is replaced by the configurable
  `rolling_resistance` so different surfaces and vehicle classes work. The
  *form* of the speed term is unchanged.
- **Engine braking is not in Gillespie at all.** The book models the powertrain
  as a torque source and gives no overrun relation. It is a pragmatic stand-in
  so lifting off in gear decelerates the way a driver expects, modelled as
  proportional to speed (RPM tracks road speed in a fixed gear). Set
  `engine_braking: 0` to coast as if in neutral.

The components are kept separate all the way through `DriveCommand`, not
pre-summed. A future load-transfer model needs them apart: aerodynamic drag
acts at the aerodynamic centre and produces a pitching moment, rolling
resistance acts at the contact patches. Lumping them discards exactly that.

The **Road load** UI panel shows the live breakdown, which term dominates at
the current speed, the drag/rolling crossover speed, and a button to zero
everything and restore the coast-forever behaviour.

### Integration

Explicit (forward) Euler, first-order: halving the timestep halves the error.
All derivatives are evaluated at the start of the interval.

Brake and road load are both resistive, so they are summed and clamped **once**
against `|v|/dt`. Clamping them separately would let their sum still carry the
speed through zero and reverse the vehicle. Rolling resistance and engine
braking also fade out below 0.2 m/s, so a stationary car does not jitter around
zero.

---

## The detailed engine + gearbox model — how the engine is connected to the car

By default the powertrain is `max_acceleration: <N> m/s²` — a **force demand**,
not an engine torque at a speed. That is deliberate: neither Gillespie,
Blundell & Harty, nor Milliken & Milliken gives an engine torque map (KB gap
**G17**, OPEN — `T_e` is an input throughout their own longitudinal chain, never
modelled). Making one up would be exactly the "formula from general knowledge"
this project's own rule forbids.

What follows is the documented exception: a real engine + gearbox, sourced from
a **fourth reference outside this project's Knowledge Base** — Heywood,
*Internal Combustion Engine Fundamentals* — via **`EngineSim`**, a separate,
independently-developed Python simulator elsewhere in this repository
(`../EngineSim/ENGINE_KB.md`, `../EngineSim/engine_sim/*.py`). It is opt-in per
vehicle and every vehicle that doesn't opt in is byte-for-byte unaffected.

### Two engine models behind one interface

`physics/Engine` is what `Powertrain` drives. Behind it sits one of two models,
chosen per vehicle by its config:

| | **Cycle-averaged** (`detailed:` alone) | **Resolved** (`detailed:` + `resolved:`) |
|---|---|---|
| What it is | One air-standard Otto cycle evaluated per step — the core of EngineSim v0.2 | EngineSim v0.7's crank-angle-resolved cylinders |
| State | Engine speed, and a boost lag | Every cylinder's gas (mass, temperature, burned fraction, fuel); intake, exhaust and charge-air plenums; turbo shaft; ECU |
| Volumetric efficiency | An authored curve (`ve_reference_rpm`) | What the valves, ports and plenums deliver |
| Manifold pressure | A function of throttle | Integrated: throttle flow in, the cylinders' own pulsed flow out |
| Pumping loss, exhaust back-pressure | Authored terms | The gas work on the intake and exhaust strokes; a plenum that fills and drains |
| Boost and turbo lag | A target curve and two time constants | Turbine power, compressor map, shaft inertia, wastegate |
| Idle | A torque injected by a PI governor | An idle-air valve and spark trim acting on the real manifold |
| Rev limiter | Speed clamp (free) / closed throttle (locked) | Fuel cut, in both |
| Can it stall? | No | Yes — and it cranks itself back to life |
| Torque curve | A pure function, probed 64× per frame | Measured on a dyno copy of the engine (`EngineDyno`) |
| Cost | Nothing | 8–14× faster than real time on one core — see below |
| Source | `physics/Engine.cpp` | `physics/ResolvedEngine.{h,cpp}` + `physics/enginecore/` |

`sports_car` and `ferrari_f40` ship with the resolved engine. Deleting their
`resolved:` block drops either back to the cycle-averaged one.

### Why a port, not a bridge

`EngineSim` is Python; this project is C++17 with no Python embedding. Rather
than bridge two runtimes for a per-physics-tick call, the engine is
re-implemented natively here. That was first done for the *validated core* of
EngineSim as it then stood (the cycle-averaged model, below), and its later
additions were deliberately left out until they had settled.

They have since settled into the resolved core, and EngineSim itself now runs
its inner loop in C. That changed what "port" has to mean:

```
src/config/EngineParameters.h            geometry, inertia, friction, fuel/air constants (both models)
src/config/ResolvedEngineParameters.h    valves, plenums, ECU, turbo components (resolved)
src/config/GearboxParameters.h           gear ratios, final drive, shift points, clutch (all AUTHORED)
src/physics/Engine.{h,cpp}               the interface; the cycle-averaged model; the adapter to the resolved one
src/physics/ResolvedEngine.{h,cpp}       EngineSim's Python engine, transcribed
src/physics/enginecore/enginecore.c      EngineSim's C cylinder core, VENDORED UNCHANGED
src/physics/enginecore/fields.h          } generated from EngineSim by
src/physics/enginecore/gas_tables.inc    } tools/sync_enginecore.py
src/physics/EngineDyno.{h,cpp}           speed-holding dynamometer (UI torque curve, engine_dyno)
src/physics/Gearbox.{h,cpp}              gear state + a simple automatic shift
src/physics/Powertrain.{h,cpp}           ties Engine + Gearbox into what Vehicle.cpp needs
```

### The resolved engine

Every cylinder carries its own gas round the 720° cycle at its own phase (set
by `firing_order`). On each crank-angle substep it breathes through its valves
(compressible orifice flow, area from the cam lift profile), burns (Wiebe,
solved as unburned and burned zones at a common pressure), loses heat to the
walls (Woschni) and does work on the piston; the intake manifold, the exhaust
plenum and — if fitted — the turbine, compressor and charge-air volume move
with the mass that actually crossed. The equations and their Heywood numbers
are documented where they live: `../EngineSim/README.md` §2.3,
`../EngineSim/UPDATES.md`, and the header of `physics/ResolvedEngine.h`.

**How it is built — two layers, kept deliberately separate.**

1. **The cylinder physics is not re-implemented.** EngineSim already has it in
   C (`engine_sim/native/enginecore.c`, proven against its Python reference by
   EngineSim's own parity tests). That file is copied here byte for byte and
   compiled into `vsim_core`, with the same floating-point flags EngineSim
   builds it with (`-ffp-contract=off`, no fast-math). One call,
   `es_advance()`, runs the substep loop of one engine step.
2. **Everything EngineSim does in Python around that loop is transcribed**
   into `ResolvedEngine.cpp`, function for function and in the same statement
   order:

   | EngineSim | Here | What |
   |---|---|---|
   | `cylinder.py` (`__init__`, `_geometry_tables`, `initialize_charge`, `load_fuel`, `set_cam_phase`) | `buildGeometry`, `initCylinder`, `setCamPhase` | volume and valve-area tables, the first charge, cam phasing |
   | `cylinder_bank.py` (`CylinderBank`) | `resetBank`, `advanceBank`, `airEstimate`, `learnAirModel`, `refreshCycleSummary`, `primeBank` | substep count, cycle starts, late fuel, the learned speed-density air model, warm start |
   | `native_core.py` (`NativeCylinderBank`) | the `K` / `B` array filling in `advanceBank` | the call into the C core |
   | `turbo_system.py` (`Turbocharger`) | `chargeTemperature`, `advanceShaft`, `controlWastegate` | intercooler, shaft energy balance, wastegate PI |
   | `engine.py` (`Engine.update`, resolved branch) | `step`, `idleAirControl`, `updateVvt` | run state, decel fuel cut, limiter, idle air + spark trim, MBT spark, knock retard, VVT, friction, starter, crank dynamics |

Unlike EngineSim, the cylinders' state *lives* in the flat arrays the C core
works on: there are no per-cylinder objects to copy in and out each step,
which is where EngineSim's own profile puts a third of its compiled step.

**Not carried over** — all optional, and off in every engine file EngineSim
ships: fuel-film dynamics, closed-loop lambda trim, random cyclic variability,
the component friction breakdown, CO/HC emissions, diesel, the speed-clamp
limiter, and the pure-Python cylinder path. `tools/engine_toml_to_yaml.py`
says so if an engine file turns one on.

**What the adapter adds** (`Engine::stepResolved`, none of it EngineSim's):

- *Sub-stepping.* EngineSim's ECU loops were tuned at a 1–2 ms step; this
  simulator's physics step is 10 ms. The engine therefore runs
  `engine_timestep_s` (2 ms) sub-steps inside each physics step, with the
  clutch load — or, when locked, the crank speed — held constant across them.
- *Torque averaged over a firing period.* A sub-step at low speed sweeps part
  of a firing interval, so its torque swings from one cylinder's compression
  to the next one's power stroke. The engine's own speed integrates that
  honestly (it is what makes an idle lumpy). A **locked** driveline, though,
  hands flywheel torque straight to the tyres, and this model has neither the
  driveline compliance that absorbs the ripple nor a step that could resolve
  it — at 100 Hz it would alias into a slow, fictitious surge. What leaves the
  engine is therefore a boxcar average over whole firing periods (one firing
  interval on an even-fire engine, 720° otherwise): the true mean torque at
  any speed and phase, with at most one firing interval of lag. AUTHORED, and
  the same quantity the cycle-averaged model reports by construction.
- *The starter.* There is no key in this simulator. An engine that is not
  firing — stalled, or reset cold — is cranked until it is. A reset starts it
  warm (EngineSim's `Engine.start()`: three fired cycles at idle vacuum
  already behind it), and for ~10 s afterwards it idles above its idle speed:
  that is EngineSim's fast-idle ramp, not a fault.

**Verification.** `tools/engine_parity_reference.py` drives EngineSim's own
`Engine` through a scripted run — warm start and idle, free rev into the
limiter, lift (decel fuel cut, fuel resume), full throttle against a load,
part throttle, an imposed-speed sweep (the locked-driveline path), a cold
start on the starter — and records every step. `build/bin/engine_parity`
replays the inputs through `ResolvedEngine` and compares 21 outputs per step
(speed, the torque balance, manifold and exhaust state, lambda, spark, idle
valve, boost, turbo speed, wastegate, IMEP, peak pressure). For EngineSim's
three engines — the 1NZ-FE four (VVT), the 454 V8 and the twin-turbo VR38 V6 —
the two agree **exactly**, on all 7250 steps (tolerance 1e-12 relative; every
column prints `exact`). Not "close": the same C core fed the same numbers in
the same order gives the same bits.

**Speed.** `engine_dyno` reports engine-seconds per CPU-second for the whole
engine (C core plus the C++ around it) at 1° substeps, wide-open throttle
across the rev range, on one core of a 2 GHz laptop (i3-6006U):

| Engine | × real time |
|---|---|
| 1.5 L four | 14 |
| 2.0 L four (`sports_car`) | 13 |
| 7.4 L V8 | 11 |
| 3.8 L twin-turbo V6 | 11–14 |
| 2.9 L twin-turbo V8 (`ferrari_f40`) | 8–9 |

(±15 %: other programs were running.) EngineSim reports 2–5× for the same
core under its Python glue. A headless 20 s acceleration run — engine, tyres,
everything — takes 3.6 s for the F40 and 1.9 s for the sports car.

### Turning it on

A vehicle opts in the same way an imported 3D model does
(`VehicleModelConfig::hasModel()`) — by the **presence** of a config block, not
a separate flag to remember to set:

```yaml
engine:
    max_acceleration: 7.5   # still here — this is what's used if `detailed` is absent
    detailed:                # <- presence of this block opts the vehicle in
        bore: 0.086
        stroke: 0.086
        compression_ratio: 11.5
        redline_rpm: 8000
        # ... see config/EngineParameters.h for every field and its source
        resolved:            # <- presence of THIS block makes it the resolved engine
            num_valves_per_cyl: 4
            firing_order: [1, 3, 4, 2]
            intake_head_dia_m: 0.0350
            throttle_max_area_m2: 0.0030
            # ... see config/ResolvedEngineParameters.h
            turbo:           # <- and this one turbocharges it
                count: 2
                target_boost_pa: 211000     # ABSOLUTE
                turbine_area_m2: 1.2e-3
gearbox:
    ratios: [3.36, 2.09, 1.47, 1.14, 0.92, 0.78]
    final_drive: 3.90
```

**Every key under `resolved:` is EngineSim's own** (`engine_sim/config.py`),
with the same unit, meaning and default, so a value moves between an engine
`.toml` and a vehicle `.yaml` unchanged. Three differences in spelling only:
`[exhaust] plenum_volume_m3` is `exhaust_plenum_volume_m3` (EngineSim's own
field name); `turbo_*` keys lose the prefix inside `turbo:`; and
`native_crank_angle_resolution_deg` / `native_steps_per_ms` are
`crank_angle_resolution_deg` / `steps_per_ms`, because there is no other core
here. One key is new: `engine_timestep_s`. An unknown key under `resolved:` is
reported on stderr — the block has some ninety, and a silently ignored typo in
`iac_feedforward` is an engine that will not idle with nothing to say why.

How `detailed:`'s own fields are used by the resolved engine:

| Field | Resolved engine |
|---|---|
| `bore`, `stroke`, `rod_length`, `compression_ratio`, `num_cylinders`, `inertia`, `idle_rpm`, `redline_rpm`, `limiter_hysteresis_rpm`, `ambient_pressure_pa`, `intake_temperature_k`, `stoich_afr`, `lower_heating_value`, `combustion_efficiency`, `friction: c1/c2/c3` | used as they are |
| `max_volumetric_efficiency` | only the ECU's first guess at its air model; the cylinders' real breathing replaces it within a few cycles |
| `ve_reference_rpm`, `friction: exhaust_back_pressure_pa`, `turbo:` (the block directly under `detailed:`) | **not used** — volumetric efficiency, back-pressure, boost and lag are outputs. They may stay in the file as the cycle-averaged fallback |

**To bring an EngineSim engine across**, `tools/engine_toml_to_yaml.py` writes
the whole block from its `.toml`, plus a `gearbox:` from its `[vehicle]`
section and the firing angles for `sound:`:

```bash
python tools/engine_toml_to_yaml.py vr38 -o /tmp/vr38.yaml   # an EngineSim id, or a .toml path
./build/bin/engine_dyno /tmp/vr38.yaml                       # it loads as it is
```

### Engine tools

Built with the default target, linked against `vsim_core` only (no window, no
audio). `<vehicle>` is a name under `config/vehicles/` or a path to any YAML
with a `detailed:` block.

| Tool | What it does |
|---|---|
| `build/bin/engine_dyno <vehicle> [--points N] [--throttle X] [--csv]` | Holds the engine at N speeds from idle to redline until its torque settles, and prints torque, power, bmep, manifold pressure, boost, exhaust back-pressure, lambda, volumetric efficiency and spark at each. Works for both models. This is where every engine figure in this README comes from |
| `build/bin/engine_dyno <vehicle> --transient RPM` | Holds RPM with the throttle shut, snaps it open, prints the response every 0.1 s. The only way to know a resolved engine's turbo lag — it is an output |
| `tools/engine_toml_to_yaml.py <engine>` | EngineSim engine file → `detailed:` block (above) |
| `tools/engine_parity_reference.py <engine> --out P` + `build/bin/engine_parity P.yaml P.csv` | Records a run from EngineSim and checks the C++ engine against it (above) |
| `tools/engine_sound_reference.py <engine> --out P [--profile wild]` + `build/bin/engine_sound_parity P.yaml P.states.csv P.f32 [--wav out.wav]` | The same check for the sound: renders EngineSim's own worklet under Node (`tools/engine_sound_render.mjs`) and compares `EngineSimSynth` with it, sample by sample. See **[Sound](#sound)** |
| `tools/sync_enginecore.py [--check]` | Re-vendors EngineSim's C core, its array layout and its gas tables into `src/physics/enginecore/` |

The Powertrain panel runs the same sweep as `engine_dyno` on a private copy of
the engine, 4 ms per frame, and draws the curve as the points arrive.

### Keeping the port in step with EngineSim

The cylinder physics now exists in three places — EngineSim's Python, its C,
and the copy of that C here — and the engine around it, and its sound, in two
each. What keeps them
identical is mechanical, not vigilance:

1. **Change the physics in EngineSim first**, and get EngineSim's own tests
   green (`tests/test_native_core.py` is what keeps its Python and its C
   together).
2. `python tools/sync_enginecore.py` — copies `enginecore.c` and regenerates
   `fields.h` and `gas_tables.inc`. Never edit those three by hand. CMake runs
   the same script with `--check` at configure time and **warns** if the
   vendored copy has fallen behind (a warning, not an error: this project
   still builds without EngineSim or Python present).
3. If the change touched EngineSim's *Python* engine — `engine.py`,
   `cylinder_bank.py`, `turbo_system.py`, the set-up code of `cylinder.py`, or
   a new `EngineConfig` field — make the same change in `ResolvedEngine.cpp`
   (and `ResolvedEngineParameters.h`, `Config.cpp`, `engine_toml_to_yaml.py`
   for a new field). A changed array layout (`_K_NAMES`, `_B_NAMES`, the
   cylinder field lists in `native_core.py`) shows up as a compile error or a
   parity failure, never as a silent misread: the C++ uses the generated
   `F_*` / `K_*` / `B_*` names, not numbers.
4. Rebuild, then for each of `1nz`, `454`, `vr38`:
   ```bash
   python tools/engine_parity_reference.py 1nz --out /tmp/p_1nz
   ./build/bin/engine_parity /tmp/p_1nz.yaml /tmp/p_1nz.csv
   ```
   Anything but `exact` in every column is a transcription error — the tool
   prints the first step and column that went wrong.
5. **The sound is a second transcription with its own check.** A change to
   `dashboard/engine-audio.js` or `engine_sim/sound_spec.py` has to be made
   again in `src/audio/EngineSimSynth.cpp` (a new `[exhaust]` / `[sound]` field
   also in `EngineSimSoundConfig`, `Config.cpp` and `engine_toml_to_yaml.py`).
   Then, for each engine and for both `--profile stock` and `--profile wild`
   (which switches on the sources that are silent at the default dials):
   ```bash
   python tools/engine_sound_reference.py 454 --out /tmp/s454
   ./build/bin/engine_sound_parity /tmp/s454.yaml /tmp/s454.states.csv /tmp/s454.f32
   ```
   A pass is a worst difference around 1e-8. The JavaScript stores into
   `Float32Array`s in places and that rounding is part of what it computes:
   `EngineSimSynth` uses `float` in exactly those places and `double`
   everywhere else, and the random streams must be drawn in the same order —
   one extra or missing `noise()` call decorrelates the two completely.

Two things to know when a parity run fails for no visible reason:

- **EngineSim's last bits depend on the Python version.** It sums per-cylinder
  figures and the air model's dot products with the built-in `sum()`, which
  CPython has compensated (Neumaier) since 3.12. `ResolvedEngine.cpp` mirrors
  that (`PySum`). Against EngineSim on an older Python the two part company:
  a last-bit difference in the trapped-air average tips a gas-table node
  lookup a few cycles later (1e-5 in torque within 0.3 s), and from there the
  idle and fuel-cut transients, which are sensitive, run visibly differently
  within seconds. The same engine, statistically; not the same trace.
- **`EngineParameters::validate()` clamps.** A value outside its range is
  silently pulled in before the engine sees it, and the engine then differs
  from EngineSim's from the first step. (This is how the friction clamps below
  were found.)

### The cycle-averaged model

The original port, and still the engine of any vehicle with `detailed:` but no
`resolved:`.

| Behaviour | Relation | Source |
|---|---|---|
| Air-standard Otto cycle | Polytropic compression/expansion, constant-volume heat addition, cycle-averaged indicated torque | Heywood Eq. 2.21, 2.26/2.27b |
| Rubbing friction | `tfmep = C1 + C2·N + C3·N²` | Heywood Eq. 13.5/13.6 |
| Pumping + accessory losses | Intake-vacuum/exhaust-backpressure and speed-scaled baseline terms | `EngineSim`'s own derivation, not a direct Heywood equation |
| Idle / closed-throttle governor | PI, corrects **both directions** (see "Bugs" below) | AUTHORED — Heywood gives no idle-control model |
| Rev limiter | Torque-cut at redline with hysteresis | AUTHORED — not in Heywood at all |
| Crankshaft dynamics | `I·dω/dt = T_net` | Heywood Eq. 14.52 |

`cycle_averaged_torque()` returns **one cylinder's** average torque; `Engine`
scales it by `numCylinders` before it ever reaches the torque balance, so it's
on the same footing as friction/pumping/accessory, which already use total
displacement. Missing that scaling was one of the bugs this port itself
surfaced in `EngineSim` — see below.

### Turbocharging

**Resolved engine** (`resolved: turbo:`). The turbo is components with their
own state, coupled through the flows the cylinders produce:

```
exhaust plenum ──► turbine ‖ wastegate ──► post-turbine volume ──► tail pipe
      ▲                 │ shaft power
      │                 ▼
  cylinders        compressor ──► charge-air volume ──► throttle ──► manifold
```

| Behaviour | Relation | Source |
|---|---|---|
| Turbine | An orifice of the stage's effective area that also takes `Ẇ = η·ṁ·c_p·T₃·[1 − (p₄/p₃)^((γ−1)/γ)]` out of the gas — so back-pressure rises as boost is demanded | Heywood Eq 6.44–6.48, 7.18–7.20 |
| Compressor | Speed lines of a scalable map; `Ẇ = ṁ·c_p·T₁·(π^((γ−1)/γ) − 1)/η_C` | Heywood Eq 6.42; the map itself AUTHORED |
| Shaft | `½·I·dω²/dt = η_m·Ẇ_T − Ẇ_C` — **turbo lag is this**, not a time constant | Heywood Eq 14.51 |
| Charge-air volume | Fed by the compressor, drained by the throttle; a check valve round an unspun compressor; a blow-off valve when the throttle shuts | filling/emptying, Eq 7.21–7.22 |
| Intercooler | `T_i = T_amb + (1 − ε)·(T_02 − T_amb)`, `T_02` from Eq 6.40 | Heywood Eq 6.40; ε AUTHORED |
| Wastegate | PI on the boost error | AUTHORED |

Twin turbos are one turbo with the flow, area and inertia of the pair
(`count`), which is exact for symmetric banks. What each parameter *does* is
best seen on the dyno: `turbine_area_m2` sets the speed at which boost arrives
(and the back-pressure paid for it at the top), `inertia_kgm2` sets how long it
takes to arrive.

**Cycle-averaged engine** (`turbo:` directly under `detailed:`). Without the
block the engine is naturally aspirated and byte-for-byte what it was.

```yaml
detailed:
    ve_reference_rpm: 8000      # where eta_v falls away (default 6000)
    turbo:
        max_boost_pa: 110000.0          # gauge - the wastegate setting
        boost_threshold_rpm: 2500
        full_boost_rpm: 4000
        spool_time_constant_s: 0.6
        blow_off_time_constant_s: 0.08
        compressor_efficiency: 0.70     # Heywood Prob 6.9
        intercooler_effectiveness: 0.70
```

| Behaviour | Relation | Source |
|---|---|---|
| Charge heating by the compressor | `T_02 = T_01·(1 + (PR^((γ−1)/γ) − 1)/η_C)` | Heywood Eq 6.40 |
| Intercooler | `T_i = T_02 − ε·(T_02 − T_01)` | component from Heywood Fig 14-24; ε AUTHORED |
| Denser, hotter charge | enters `m_a = η_v·ρ_i·V_d` through `ρ_i = p_i/(R·T_i)` | Heywood Eq 2.27 |
| Exhaust back-pressure rises one-for-one with boost | keeps pumping work from becoming a free gain | AUTHORED (no turbine map) |
| Boost vs engine speed | smoothstep from threshold to full boost, × throttle | AUTHORED |
| Turbo lag | first-order lag toward the target; a faster one on lift (blow-off valve) | AUTHORED stand-in for Heywood Eq 14.51 |

Here the lag reproduces Eq 14.51's behaviour (Fig 14-25), not its mechanism.
Boost advances in every driveline state, including locked, where the engine
speed is imposed (`Engine::evaluateTorqueAt`).

### Calibration, stated honestly

All figures from `build/bin/engine_dyno <vehicle>` — steady state, wide-open
throttle.

**Ferrari F40** (F120A: Ferrari's published 82.0 × 69.5 mm, 2936 cc, 7.8:1,
1.1 bar; valve sizes, cam events and every turbo figure AUTHORED — none is
published).

| rpm | 2200 | 2800 | 3400 | 4000 | 5300 | 6500 | 7100 | 7700 |
|---|---|---|---|---|---|---|---|---|
| N·m | 274 | 482 | 478 | 468 | 438 | 397 | 376 | 354 |
| kW | 63 | 142 | 171 | 198 | 242 | 271 | 281 | 286 |
| boost, bar gauge | 0.24 | 1.11 | 1.14 | 1.15 | 1.16 | 1.15 | 1.15 | 1.14 |

Against the published 577 N·m at 4000 rpm and 352 kW (478 PS) at 7000: the
**shape** is the car's — next to nothing below 2500 rpm, then both turbos
arrive together and the curve is flat — and the **level** is ~19 % short on
torque and ~20 % on power. A throttle snapped open at 3500 rpm reaches full
boost in about 1.5 s; at 5000 rpm, in 0.8 s. The gap is the model, and it is
left visible rather than closed by pushing boost or combustion efficiency past
real values: the mixture is held stoichiometric (the real engine runs rich
under boost), there is no intake or exhaust wave tuning, and the intercooler
effectiveness is a guess. The cycle-averaged model made 480 N·m and 247 kW,
but with an authored boost curve and an authored lag.

**Sports car** (an invented 2.0 L four). 170 N·m at 2200 rpm — 10.7 bar bmep,
against Heywood's 8.5–10.5 for naturally aspirated SI engines — and 81 kW at
6000 rpm. The torque is right for the displacement. The **power is about half**
of what a real engine of this specification makes, because the curve falls
from 2200 rpm where a real one keeps climbing to 7000. Two reasons, both the
model's: no wave tuning (which is what holds a real engine's breathing up at
speed), and Heywood's friction correlation, which was fitted to production
engines below ~6000 rpm and charges 5.4 bar at 8000.

**On the road** (`--headless --experiment acceleration_test --track straight`):

| | cycle-averaged | resolved |
|---|---|---|
| `sports_car` 0–100 km/h | 13.6 s | 12.5 s |
| `ferrari_f40` 0–100 km/h | 6.9 s | 7.2 s |
| `ferrari_f40` 0–200 km/h | 16.8 s | 16.7 s |

The F40's figures are tyre-limited, not engine-limited: from ~3800 rpm in
first gear the rear tyres are spinning and delivering their sliding friction
(a constant 4.3 m/s²). See "What's still open".

### The coupling model — a friction clutch

A clutch or torque converter is what lets an idling engine reach stationary
wheels without stalling. Neither is modelled in either Knowledge Base this
project draws from (torque converter **G18**, clutch engagement **G20**, both
OPEN). What `Powertrain` models is the friction-clutch case and only its
defining property: **a clutch transmits torque up to its capacity, and exactly
that capacity while slipping** — Coulomb friction at a known contact. The
capacity and the engagement schedule are AUTHORED (`GearboxParameters.h`).

Three states:

- **Locked** — driveline rigid. Engine speed is *read from* road speed × gear
  ratio, not integrated; its torque is reflected to the wheels in both
  directions — drive **and** engine braking.
- **Slipping** — the clutch passes its capacity torque. The engine integrates
  its own speed against that load, so a launch genuinely pulls the revs down.
  The torque acts **against the slip**: engine faster than the wheels loads the
  engine and drives the car; wheels faster than the engine spins the engine up
  and brakes the car (capped at what closes the slip in one step). The clutch
  locks once the slip closes at full engagement.
- **Released** — the engine free-idles; nothing reaches the wheels.

So a detailed-engine vehicle **does** pull away from rest. In `clutch_mode:
automatic` the clutch is let out over `clutch_engage_time_s` whenever there is
throttle; in `manual` it follows the pedal (`Left Shift`). In both modes an
anti-stall releases the clutch if the engine is dragged below
`clutch_release_idle_fraction` of idle. (The cycle-averaged engine has no
stall model, so this is what keeps it alive; the resolved engine can stall and
restarts itself, and the anti-stall is kept so a dumped clutch gives a bog
rather than three seconds of cranking.)

**`launch_rpm`** (gearbox, default 0 = off). The timed engagement starts
biting the moment the throttle opens. That suited the cycle-averaged engine,
which — unphysically — made its best torque at idle. A resolved engine makes
what a real one does there, and a turbocharged one makes next to nothing until
its turbines have gas to work with, so a clutch that grabs at idle bogs the
launch. With `launch_rpm` set, the automatic clutch instead bites **in
proportion to engine speed**, from just above idle to `launch_rpm` (scaled
down with part throttle), and goes fully home the moment the car has caught
the engine up. The engine settles where its torque equals what the clutch is
passing. AUTHORED — a left foot. `ferrari_f40` sets 4500.

`shift_mode: automatic` shifts on `upshift/downshift_redline_fraction`;
`manual` only on `X`/`Z`. The two modes are independent.

Reflected engine inertia is included once locked: `I_E·ratio²/r_wheel²` adds to
the effective mass the drive force must accelerate, so higher gears correctly
feel lighter. This follows directly from `I_E` and the ratio (both already in
scope); the KB gives no equivalent relation for a vehicle driveline, so this is
ordinary mechanics, not a KB-sourced formula.

### Wiring into the vehicle

`Vehicle` owns an optional `std::unique_ptr<Powertrain>`, constructed only when
`VehicleParameters::hasDetailedEngine()` is true. Inside `Vehicle::step()`, if
it exists, its output **replaces two things** that would otherwise come from
elsewhere:

```cpp
driveAccel = pt.driveAccelerationMps2;       // was: throttle * maxAcceleration
m_roadLoad.engineBraking = pt.engineBrakingMps2;  // was: RoadLoad's speed-proportional stand-in
```

`RoadLoad.cpp`'s own comment on `engineBrakingCoeff` calls this out by name:
*"Replace this the moment a real engine map and gearbox exist."* Rolling
resistance and aerodynamic drag from `computeRoadLoad()` are untouched — only
the powertrain-shaped terms are swapped.

Nothing in that contract changed for the resolved engine. `Powertrain` calls
the same two entry points on `Engine` — `update()` when the clutch is slipping
or released (the engine integrates its own speed), `evaluateTorqueAt()` when it
is locked (the crank is held at wheel speed × ratio) — and the adapter in
`Engine.cpp` routes each to the resolved core. It differs from the
cycle-averaged path in three marked places in `Powertrain.cpp`: the locked rev
limiter (the resolved engine cuts its own fuel, so its throttle is left
alone), what `engineTorqueNm` reports (flywheel torque, whatever the clutch is
doing with it), and stalling.

### Telemetry

Engine RPM, torque, current gear, lock state, and rev-limiter state are
recorded every sample (`engine_active`, `engine_rpm`, `engine_torque_nm`,
`engine_gear`, `engine_locked`, `engine_limiter_active` — zero/false on a
vehicle without the detailed engine), followed by the air path:
`engine_map_pa` (intake manifold, absolute), `engine_boost_pa` (charge
pressure above ambient) and `engine_lambda`. On the resolved engine those
three are states of the model — lambda wanders off 1.0 in transients, and
reads 10 (the model's "no fuel" marker) under a fuel cut. On the
cycle-averaged engine they are what its formulas assume, and lambda is always
1.

`engine_torque_nm` is the engine's net torque (after any clutch load) on the
cycle-averaged model, and the flywheel torque averaged over a firing period on
the resolved one.

### Bugs found along the way — fixed in both places

Building the first port and smoke-testing "does the engine actually idle at
idle" surfaced two real bugs in `EngineSim` itself, not just the port:

1. **The idle governor didn't govern.** It only ever *added* torque below
   1.5× idle speed and did nothing above it. Since closed-throttle combustion
   is never quite zero in the cycle-averaged model (intake pressure floors at
   `0.2×p_amb`, not true vacuum), a "closed throttle" engine ran away past its
   own governor's ceiling and settled at its own unopposed equilibrium instead
   — **~5960 RPM, not 800**, for the default config.
2. **No engine braking on lift-off at speed**, for the same underlying reason
   — closed-throttle combustion kept producing net-positive torque across most
   of the RPM range. Fixed with decel fuel cutoff (DFCO): closed throttle and
   meaningfully above idle now cuts fuel to zero, leaving only
   friction/pumping/accessory torque. Standard ECU behaviour, not invented
   physics.

Both were fixed at the source in `../EngineSim/engine_sim/engine.py`, then
mirrored into `src/physics/Engine.cpp` here the same way.

Bringing the resolved engine across found two more, this time on this side:

3. **This project was still carrying EngineSim's old friction coefficients.**
   Heywood Eq 13.6 takes N in rev/**min**; converted per rev/s its speed terms
   are 900 and 18. EngineSim once had 15 and 0.005 — the same numbers
   converted as if N were rev/s, which makes friction nearly flat with speed
   (1.0 bar at 6000 rpm instead of 3.7) — fixed that in its v0.4, and the fix
   never made it here. Worse, `EngineParameters::validate()` *clamped* the two
   coefficients to 500 and 5, so the corrected values could not even be set:
   the first parity run failed on friction torque at step 0. The clamps are
   now wide enough, a `resolved:` engine defaults to 900 / 18, and both shipped
   vehicles say which they use. **The cycle-averaged defaults are deliberately
   still 15 / 0.005** — that model's calibration (and every figure this README
   quoted for it) was done with them.
4. **The automatic clutch only worked because the old engine was wrong.** See
   `launch_rpm` above: it bit at idle, where the cycle-averaged model made
   peak torque. The first launch of the resolved F40 dragged it to 530 rpm.

### What's still open

- **Engine speed follows road speed, not wheel speed.** When locked, the
  engine is geared to the *vehicle's* speed, and the drive torque goes to the
  tyres. A tyre that breaks traction therefore spins up freely (slip ratios of
  −3 to −5 through first gear in the F40) without dragging the revs up with
  it. Coupling the engine to the driven wheels' own speed is the fix, and with
  a resolved engine it would also give the limiter something real to do.
- **No traction control and no boost-by-gear** — which is why the F40's
  0–100 km/h is tyre-limited at twice the real car's time.
- **The resolved engine is always stoichiometric and has no wave dynamics** —
  EngineSim's own open items (`../EngineSim/UPDATES.md` §7), and the reason
  both shipped engines are short at the top end. Fixing them belongs in
  EngineSim, after which steps 1–4 above bring them here.
- **Knock on boosted charge.** EngineSim's knock model saturates on a boosted
  engine (its own note), so `use_knock_model` is off for the F40 and nothing
  limits its low-speed, high-boost torque.
- **EngineSim's engine sound has no turbocharger in it.** Turbo whine and the
  blow-off valve are still this project's, layered on top, and a turbine does
  not soften the exhaust pulses as it would. See **[Sound](#sound)**.
- **No reverse gear.** There's no gear-select input to request it with.
- **Open differential, equal torque to both driven wheels** (G19, OPEN).
- **Gear changes are instantaneous and lossless** in automatic mode — no
  torque interruption or shift time. The resolved engine's speed steps with
  the ratio.
- **No torque converter** (G18) — every detailed-engine car is a manual.

---

## The Apex Grand Prix Circuit

`--track apex_gp`, `default_track: apex_gp` in `config/simulation.yaml`, or
**Track → apex_gp** in the Simulation panel.

An original circuit — not a real one, and not derived from one. 4.93 km,
13 corners, clockwise, 14 m wide:

| | | |
|---|---|---|
| Main straight, 900 m | start line, grid, pits on the right | |
| T1 | right, R 42 m, 100° | the heavy stop; paved runoff, grandstand |
| T2 | left, R 62 m | |
| T3 | right, R 210 m | fast sweeper |
| T4 | right, R 80 m | |
| T5 | left, R 110 m, 150° | long carousel |
| T6 | right, R 28 m, 178° | hairpin; paved runoff, grandstand |
| Back straight | the longest after the main straight | footbridge |
| T7–T8 | left-right, R 30 m | chicane |
| T9 | right, R 125 m | grandstand |
| T10 | left, R 58 m | |
| T11 | right, R 46 m | grandstand |
| T12 | left, R 90 m | kink |
| T13 | right, R 80 m, 121° | onto the main straight; pit entry on the inside |

**What is there.** Asphalt with a rubbered racing line, edge lines, start line,
22 grid slots, skid marks into the braking zones; raised red-and-white kerbs at
apex, entry and exit; gravel or paved runoff on the outside of every corner;
armco on the straights and corner insides, concrete wall with catch fencing and
tyre walls where cars arrive or people stand; a pit lane with entry and exit
roads, pit wall, 36 garages, a two-storey pit building, race control tower,
paddock with transporters and hospitality units; a four-block roofed main
grandstand, two open stands beside it and four at corners, with crowds;
marshal posts, distance boards (150/100/50), corner numbers, a start gantry,
three footbridges, a service road and perimeter fence; car park, background
buildings, an infield lake, woodland, and a ring of hills inside the far plane.
All team and sponsor names are invented.

**How it is built.** `Track::createApexGrandPrix()` produces the centreline and
the pit-lane centreline (`Track::pitLane()`), like every other preset. The
renderer recognises the track by `Track::id()` and builds a `Venue` from it
(`rendering/ApexVenue.cpp` and the four `ApexVenue*.cpp` files beside it)
instead of calling the generic `TrackMesh` generator. Nothing is loaded from
disk: the venue is generated when the track is loaded.

**How it is drawn.** See the banner of `rendering/Venue.h`. In short: static
geometry is batched by layer on a 180 m grid and frustum-culled; seats' crowds,
fence posts, tyre stacks, trees, parked cars and transporters are instanced;
surfaces are photographed materials (see Surface textures, below);
trees and spectators have distance ranges; the frame is drawn front to back.
Lighting is one sun with two shadow maps (a sharp one round the car, a wide one
for buildings and trees that is redrawn only as the car moves), hemispheric sky
light, an analytic sky shared by the fog and the reflections, and an ACES tone
map. The original lighting path is untouched and still draws every other track.

**What it does not do — the same limits as every track here.**

- The surface is flat. No elevation, no banking, no kerb you can feel: the
  kerbs are drawn 8 cm high and the tyres do not know.
- There is no collision with anything. Barriers, walls and buildings can be
  driven through; grass and gravel grip like asphalt.
- The pit lane is geometry. No speed limit, no pit stops, no lap timing.
- Surfaces are photographed materials (below), but the GEOMETRY is still
  generated low-polygon work: buildings are boxes, spectators are blocks, and
  signage is a 5×7 dot font. There are no imported scenery models.
- The TV camera's trackside positions are placed by the generic rule and a few
  of them stand inside the pit wall or a grandstand.

### Surface textures

The venue's surfaces are photographed materials with normal and roughness
maps: asphalt, grass, gravel, concrete, painted plaster, galvanised steel,
tyre rubber, profiled steel roofing, bark and leaves. All ten are **CC0
(public domain) from [ambientCG](https://ambientcg.com)**; the list, with
links, is `assets/textures/venue/SOURCES.md`. They are 12 MB in total and
ship in the repository — the simulator never touches the network.

- **Baking.** `tools/bake_venue_textures.py` downloads each material and
  writes two 1024² JPEGs: colour (ambient occlusion multiplied in, normalised
  to mid-grey so the venue's palette still decides what colour a surface is)
  and detail (normal x, y and roughness). Run it to rebuild or swap a material.
- **Drawing.** Two texture arrays, one layer per material, block-compressed
  on upload and anisotropically filtered. Meshes carry no material UVs: a
  surface is textured by projecting its world position along the axis its
  normal is nearest, so nothing stretches and texel size is the same on
  everything. The normal map bends the shading normal; roughness feeds the
  specular term.
- **Trees.** Up close a tree is a bark-textured trunk and a crown of leaf
  cards — cut-out sprays of leaves, anti-aliased with alpha-to-coverage — with
  a solid textured crown at middle distance and a single shape beyond. A
  hidden solid crown casts the shadow.
- **If the textures are missing** the venue still draws, in plain vertex
  colour, and says so on the console.

The leaf cards rely on multisampling for their cut-out; the window asks for
4× MSAA. Asphalt, grass and trees are the visible gain. Buildings and stands
gain surface grain but are the same shapes as before.

### Checking a circuit: `--tour` and `lap_test`

```bash
./build/bin/VehicleSimulator --track apex_gp --hide-ui --fullscreen --tour shots/
./build/bin/lap_test --track apex_gp --vehicle basic_car
```

`--tour` puts the car at 20 stations round the lap (plus the pit lane and five
detached views), renders each, writes a PPM screenshot and prints the frame
time, draw calls and triangles. Measured on the development machine — Intel HD
Graphics 520 (Skylake GT2), Mesa, 1366×768 full screen, 4× MSAA, vsync off,
28 views:

| | mean | worst |
|---|---|---|
| `apex_gp` | 12.4 ms (81 fps) | 18.9 ms (53 fps) |

That is with the photographed textures; before them, on vertex colour and
shader noise alone, the same run measured 10.7 ms mean and 16.4 ms worst. The
venue is 2.6 M triangles in total, of which a frame submits a few hundred
thousand, shadow passes included.

`lap_test` drives a flying lap with a pure-pursuit autopilot through the real
`Simulation` and reports the furthest the car strayed from the centreline. It
tests the track, not the driver: at its default 0.32 g it gets `basic_car`,
`sports_car` and `suv` round `apex_gp` within 1.5 m of the centreline (half
width 7 m), and round `race_circuit` and `nurburgring` likewise. It does not
get `gt3_redbull` or `ferrari_f40` round **any** circuit — its throttle law
spins them — so those two say nothing about a track either way.

## Cameras

`src/rendering/Camera.{h,cpp}`. `V` cycles, `1`–`0` select, `--camera` starts in one.

| View | What it is |
|---|---|
| Chase / Chase far | Smoothed, behind and above; mouse orbits, scroll zooms |
| Cockpit | The driver's eyes, **rigidly bolted to the body** — it pitches and rolls with it (terrain heave/pitch/roll and cornering roll, the same transform the body is drawn with). The head moves against sustained acceleration (≈4 mm per m/s², capped at 6 cm — AUTHORED). Mouse turns the head, scroll sets the field of view |
| Bonnet | Above the scuttle, over the bonnet |
| Bumper | On the nose near the road; the car is not drawn |
| Wheel | On the flank, looking forward past the front wheel |
| TV trackside | Fixed cameras every 140 m along the centreline, outside each corner, 9 m back and 3 m up; cuts to the nearest (with hysteresis), pans after the car and zooms to hold it at constant size. On a track with no usable centreline it rigs a camera ahead of the car instead |
| Helicopter | 40 m back, 28 m up, slow to follow |
| Free / Top down | Debug views |

**Mount points** come from an optional per-vehicle block, in vehicle-local
metres (x forward from the rear axle, y up from the ground, z right). Unset
points are derived from `length`/`height`/`width`, which suits the generated
car but can land inside real bodywork on an imported one — both imported cars
author theirs:

```yaml
cameras:
    driver_eye: [1.33, 0.95, -0.335]   # F40: measured off the seat/headrest nodes
    bonnet:     [2.35, 0.90, 0.0]
    bumper:     [3.60, 0.30, 0.0]
    wheel:      [1.30, 0.45, -1.06]
```

**Near plane.** The world needs a 0.5 m near plane (see `CameraConfig::nearPlane`),
far too large for a lens a few centimetres from the dashboard. Car-mounted views
therefore render in two passes: the world with the normal projection, then the
depth buffer is cleared and the car drawn with a 2 cm near plane. From a camera
bolted to the car nothing outside it can be between the lens and its bodywork,
so drawing the car last is correct.

**Glass.** Cockpit views need to see out, so imported models' glass is now
drawn see-through: glTF `alphaMode: BLEND` materials whose factor alpha is
below 1, *or* whose texture actually contains translucent texels (the GT3
keeps its glass transparency in the texture and marks every material `BLEND`),
are drawn last, blended, without depth writes. The F40 export writes its glass
at alpha 0.30.

---

## Sound

`src/audio/`. **No recordings** — every sound is synthesised sample by sample
from state the simulation already computes. What is *derived* is each source's
dependence on the physics; absolute loudness is not calibrated (nothing here is
in pascals at an ear), so per-source levels are balanced at a reference
condition and exposed as mix sliders in the **Sound** panel.

| Source | How it is made | Physics it comes from |
|---|---|---|
| Firing | A crank-angle clock integrates the engine's own ω; each cylinder fires when the crank passes its angle | Engine speed; firing order from `sound:` (even 720/n by default) — so the firing frequency is `N/60 · n/2` because the crank turns, and an uneven order is uneven |
| Exhaust pulse | Blowdown pulse per firing: the valve snaps open (fast rise over ~12 % of a 60° blowdown, Heywood §6.5) and the cylinder empties (exponential decay) — the sharp front carries the harmonics. Flow **rate** = released charge ∝ `p₄ − p_manifold`, over a fixed crank angle, so it also scales with ω | `p₄ = m R T₄ / V_BDC` from the engine's own Otto cycle (`TorqueBreakdown`). Fuel cut leaves p₄ below the manifold: a weaker **reverse** pulse — the overrun sound |
| Per-cylinder signature | Each cylinder has a fixed strength (±6 %) and timing (±0.8°) offset, plus random cycle-to-cycle variation (3 %) | Charge maldistribution (Heywood §7.6) and cyclic variability (Ch. 9) — puts energy at **half orders**, the lumpy undertone of a real engine |
| Exhaust pipe | Digital waveguide per bank: delay `L/c` each way, lossy reflections at the collector/turbine end (0.5) and open end (0.8, inverted below `ka≈1`) | `c = √(γRT)` at the exhaust gas temperature (a fraction of T₄, with thermal lag), so the pipe resonances `(2k−1)c/4L` move with load |
| Radiation | Far-field pressure `p ∝ dU/dt` of the tailpipe's **volume velocity** `U = (1 − R)·p⁺`, which doubles at low frequency where the pressure cancels | Small-source (monopole) radiation |
| Silencer, turbine | 4th-order low-pass (24 dB/oct) at `muffler_cutoff_hz`; the turbine absorbs and smooths pulses in proportion to boost | Boost (AUTHORED shape — no turbine map) |
| Engine vibration (structure-borne) | Each combustion impact rings a bank of block/body modes (52–590 Hz); plus the **second-order shaking force** `m r ω² (r/l)·|Σ e_bank e^{2iφ}|` at twice crank frequency | Indicated work per cylinder; stroke, rod length, firing angles and bank geometry — an inline-4 and a flat-plane V8 shake, a cross-plane V8 comes out balanced by construction. Mode frequencies are AUTHORED. Loudest in the cockpit |
| Intake | Induction pulses on each intake stroke → runner waveguide → **air box Helmholtz resonator** `f = c/2π·√(A/VL)` → radiated dU/dt; throttle-plate jet hiss | Trapped charge × ω; air-box volume and snorkel from `sound:`; Lighthill U⁸ jet noise; a turbo compressor damps it with boost |
| Turbo whine | Tone at blade-pass frequency, twin turbos slightly detuned | Shaft speed from Euler's work equation: `c_p T₁ (PR^((γ−1)/γ) − 1)/η_C = ψ U²` (Heywood Eq 6.42 per unit mass). The F40's 1.1 bar gives ≈130 000 rpm, ≈13 kHz |
| Blow-off valve | Vented jet noise on a lift with boost in the plenum | Lighthill U⁸ from boost; its duration is the engine model's own blow-off decay |
| Gear whine | Tone at input-shaft frequency × tooth count, while the clutch is locked | Engine speed, torque |
| Tyres | Rolling noise ∝ v^1.75; per-tyre stick-slip squeal gated near the friction limit | Sliding power `|F| · V·√(S² + tan²α)` from each tyre's own force, slip ratio and slip angle |
| Wind | Band noise at Strouhal frequencies `St·U/D`, amplitude ∝ U³ | Curle's dipole law (intensity ∝ U⁶) |
| Listener | Exterior: a propagation delay line of `distance/343 m/s`, 1/r spreading, air-absorption low-pass, stereo pan from the camera. **Doppler comes out of the delay changing** — no Doppler formula. Cockpit: mass-law cabin low-pass on exhaust, tyres, wind; the engine loud if it sits behind you | Camera ↔ car geometry |

**Verified** by rendering runs to WAV and analysing them (`--headless
--audio-out`): 98–100 % of the energy sits at engine orders (firing
harmonics and half orders) from idle to 7000 rpm, with the spectral centroid
at ~200 Hz at idle and ~1 kHz at 6500 rpm; level rises ~15 dB from 1000 to
7000 rpm at full throttle with a few dB of pipe resonance; idle is ~26 dB and
overrun ~17 dB below full power. The dominant spectral peak sits on the
firing frequency or its 2nd harmonic through every gear change; a TV-camera drive-by at ≈60 m/s gives
a received/emitted frequency ratio of 0.82–0.84 receding (theory
`c/(c+v)` = 0.85) and 1.06–1.13 approaching, with the level peaking at each
camera and the stereo image crossing as the car passes. Synthesis costs about
5 % of one core.

**Per-vehicle hardware** (all optional, `VehicleSoundConfig`): firing angles
and banks, vee angle, reciprocating mass, pipe length, gas-temperature
fraction, blowdown angle, silencer corner, cyclic and cylinder-to-cylinder
variation, air-box volume and snorkel, turbo wheel radius and blades, gear
teeth, engine and tailpipe positions.

An earlier version of this synthesiser sounded like air rather than an
engine. Two physics errors caused it, both fixed: the tailpipe radiated the
transmitted *pressure* `(1 + R)p⁺` — which cancels at low frequency — instead
of the volume velocity, high-passing the exhaust twice; and pulse height
ignored rpm, so a shorter high-rpm pulse moved less gas. The intake was also
filtered noise until the air-box resonator replaced it, and there was no
structure-borne sound at all. The F40 is a flat-plane V8 (banks
alternate every firing); the GT3's block describes its cross-plane V8 (banks
do not alternate — the burble).

### EngineSim's engine sound

A vehicle with a `sound: engine_sim:` block is heard through **EngineSim's own
engine synthesiser** instead of the engine rows of the table above.
`src/audio/EngineSimSynth.{h,cpp}` is a statement-for-statement transcription
of EngineSim's AudioWorklet (`../EngineSim/dashboard/engine-audio.js`) and of
`engine_sim/sound_spec.py`, which derives everything engine-specific from the
engine's own data; `../EngineSim/README.md` §6 documents the model and how it
was tuned. `sports_car` and `ferrari_f40` both use it.

What it is, briefly: a crank-angle clock — with the speed ripple each firing
puts into the flywheel — walks a list of valve events placed by the real firing
order and valve timing. Each exhaust-valve opening kicks a **blowdown** and a
**displacement** pulse (3-pole smoothing filters, so nothing can click), strong
when firing, weak and *negative* when motoring on a fuel cut — the hollow note
of engine braking. The pulses drive a Kelly–Lochbaum **exhaust waveguide** per
pipe — header, mid pipe, muffler expansion chamber, tailpipe — whose ringing
between firings is what turns a pulse train into an engine note. Around that:
intake pulses and roar, throttle hiss, valvetrain tick, the starter motor and
its solenoid, overrun pops, early reflections, a compressor and a soft clip.

It is a different kind of model from the rest of this section, and says so:
not derived quantity by quantity, but physically *structured* and then **tuned
by measurement** (EngineSim's README §6.3 gives the rules and the spectra).

**How it sits in this simulator.**

| | With `engine_sim:` | Without |
|---|---|---|
| Firing, exhaust pulse, exhaust pipe, radiation, silencer | EngineSim | this project (table above) |
| Intake, throttle hiss, valve gear | EngineSim | this project |
| Starter motor, overrun pops | EngineSim | — |
| Engine vibration (structure-borne) | — (not in EngineSim's model) | this project |
| Turbo whine, blow-off valve, gear whine, tyres, wind | this project | this project |
| Listener: distance, Doppler, pan, cabin | this project | this project |

- **What drives it** is what EngineSim's dashboard sends its worklet, taken
  from the resolved engine each frame: speed, manifold pressure ÷ ambient,
  throttle, `combusting`, `fuel_cut`, `starter`, and the gross-indicated,
  friction and pumping torques (the last three set the crank ripple). Firing
  order, valve events, bore and inertia come from the engine block; the
  exhaust and intake hardware from `sound: engine_sim:`.
- **Its own road noise and driveline whine are fed zero speed** — tyres, wind
  and gear whine here come from the tyre model and the gearbox.
- **Its output goes on the exhaust bus** (the *Engine (EngineSim)* mix slider),
  through the same propagation delay as everything else, so a trackside camera
  still hears the pass-by drop in pitch. A dual exhaust stays left and right:
  the stereo difference travels the same path and fades as the car moves to one
  side of the view. In the cockpit half of it also takes the engine-bay path
  (it is the whole engine, not only the tailpipe). AUTHORED routing.
- **It runs in 128-frame blocks**, the AudioWorklet's quantum, each started
  from the newest state: several of its quantities are evaluated per block.
- **Loudness** is matched to the old model at full throttle from the chase
  camera (`kEngineSimGain`). EngineSim's compressor then holds idle only ~6 dB
  below that, where the old model's idle was ~26 dB down — turn `COMP` down
  for natural dynamics.
- It costs 2–3 % of one core.

**The hardware** — keys are EngineSim's own field names, the `[exhaust]`,
`[intake]`, `[sound]` and `[layout]` sections of an engine file
(`EngineSimSoundConfig` in `config/VehicleParameters.h` lists them all):

```yaml
sound:
    engine_sim:
        exhaust_layout: dual                    # one pipe per bank (stereo) | single
        cylinder_bank: [0, 0, 0, 0, 1, 1, 1, 1]
        exhaust_header_length_m: [0.45, 0.48]   # per pipe, or one number for all
        exhaust_mid_length_m: [0.70, 0.76]
        exhaust_muffler_length_m: 0.40
        exhaust_tail_length_m: [0.30, 0.33]
        exhaust_muffler_area_ratio: 4.0         # bigger = quieter, more muffled
        exhaust_header_temp_k: 850              # hotter gas = higher pipe pitch
        exhaust_tip_damping: 0.22               # lower = brighter, raspier
        sound_cylinder_strength_spread: 0.10    # fixed cylinder differences: the idle lope
        sound_pop_probability: 0.05
        profile: { ring: 0.7, rasp: 0.6 }       # the dials, below
```

`tools/engine_toml_to_yaml.py` writes the block from an EngineSim engine file.
It needs the resolved engine (firing order and valve events come from there);
without one it is ignored, with a message.

**The dials.** EngineSim's fourteen, with its ranges, defaults and presets
(Stock, Muscle, Track, Smooth, Vintage, Rumble), in the **Sound** panel under
*Engine sound profile*: `VOL`, `CONV` (room), `COMP`, `DRIVE`; `+LF`, `+HF`,
`RING` (how long a pulse rings in the pipes), `RASP`; `LOPE`, `~LF`, `~HF`,
`TICK`, `INTK`, `POPS`. A vehicle sets its starting point with `profile:`.

**Verified against the original.** `tools/engine_sound_reference.py` drives
EngineSim's engine through a run (cold start on the starter, idle, a free rev
into the limiter, a lift with overrun, full throttle under load) and renders
EngineSim's **unmodified** `engine-audio.js` over it under Node;
`build/bin/engine_sound_parity` replays the same states through
`EngineSimSynth` and compares every sample. For all three EngineSim engines,
at the default dials and with all fourteen moved, the two differ by at most
3 × 10⁻⁸ of full scale — one or two units in the last place of a 32-bit float
— and the difference is 187–205 dB below the signal. (Not bit-identical, and it cannot be: `sin`, `exp`, `pow`
and `tanh` come from V8's maths library on one side and the C library's on the
other.)

**Without `engine_sim:`**, a resolved engine still feeds this project's own
synthesiser from real state rather than an Otto-cycle estimate: the blowdown
pressure is cylinder 1's recorded pressure just before its exhaust valve opens
(temperature from `pV = mRT` with the mass trapped at intake-valve closing),
the manifold the pulse discharges into is the exhaust plenum, the induction
pulse is the air the cylinders actually trapped, and boost is the charge-air
pressure.

**Engine sound needs the detailed engine.** Vehicles on the simple
`max_acceleration` demand have no crank, cycle or pressures to synthesise from,
so they get tyres and wind only — currently every car except `sports_car` and
`ferrari_f40` (which both use EngineSim's engine sound, above).

`audio:` in `config/simulation.yaml` enables it and sets the master volume; a
missing sound device is logged and the simulator runs silent. The device is
[miniaudio](https://miniaud.io) (single header, fetched at configure time like
cgltf).

---

## Fixed timestep

`src/simulation/Simulation.cpp`

Physics advances in exact increments of `simulation.timestep` regardless of
frame rate:

```
accumulator += frameTime * timeScale
while (accumulator >= dt) { stepPhysics(dt); accumulator -= dt; }
```

The resolved engine sub-steps inside each physics step — 2 ms engine steps
(`engine_timestep_s`), each cut into crank-angle substeps of about 1° — and is
equally deterministic: nothing in it reads a clock.

A 30 fps machine and a 240 fps machine executing the same inputs produce the
same trajectory. With a variable timestep the yaw integration error would depend
on frame rate, making telemetry incomparable between runs.

Two guards against the "spiral of death" after a stall:

- a single frame contributes at most 0.5 s to the accumulator;
- at most `max_steps_per_frame` steps run per frame, and surplus time is
  **dropped** rather than compounding.

When either triggers, the UI says so — a silently slowed simulation is a trap.

Telemetry is sampled **inside** the fixed-step loop, so the sample interval is
exactly `dt`.

---

## Configuration

Nothing about a vehicle is hardcoded in C++ (the struct defaults exist only so
a missing file degrades to something drivable rather than dividing by zero).

```
config/
  simulation.yaml
  vehicles/
    basic_car.yaml      family saloon
    sports_car.yaml     short wheelbase, low CG, rear weight bias; resolved NA engine
    suv.yaml            long wheelbase, high CG, heavy
    gt3_redbull.yaml    Mercedes-AMG GT3, imported model, geometry measured from it
    ferrari_f40.yaml    Ferrari F40, own model, resolved twin-turbo engine + 5-speed
```

Add a vehicle by dropping a new YAML file into `config/vehicles/` — it appears
in the UI dropdown automatically.

```yaml
vehicle:
    name: "BasicCar"
    mass: 1500.0
    dimensions:  { length: 4.5, width: 1.8, height: 1.4 }
    wheelbase: 2.7
    track:       { front: 1.55, rear: 1.55 }
    cg:          { height: 0.55, front_weight_distribution: 0.55 }
    steering:    { max_angle: 35.0, steering_rate: 180.0 }   # degrees
    inertia:     { yaw: 0.0 }            # 0 = documented fallback, OQ-6
    engine:      { max_acceleration: 4.0 }        # tractive force DEMAND
    drivetrain:  { layout: rear }                 # front | rear
    brakes:      { max_deceleration: 8.0, front_proportion: 0.0 }
    limits:      { max_speed: 60.0 }
    wheels:      { radius: 0.34, width: 0.22, inertia: 1.2 }
    road_load:
        rolling_resistance: 0.015   # f_r0 [-]
        drag_coefficient: 0.32      # C_D  [-]
        frontal_area: 2.2           # A    [m^2]
        engine_braking: 0.012       # [1/s], 0 = coast as if in neutral

    # The roll chain. Everything downstream of these - roll rate, roll
    # gradient, the three-term load transfer and its front/rear split - is
    # COMPUTED, not authored.
    suspension:
        ride_frequency_front: 1.25        # Hz - see OQ-4 on the two bands
        ride_frequency_rear: 1.20
        roll_centre_height_front: 0.0787  # m - Milliken example values, OQ-4
        roll_centre_height_rear: 0.1730
        anti_roll_rate_front: 9100.0      # N.m/rad, sized to Table 16.1
        anti_roll_rate_rear: 0.0
        unsprung_fraction_driven: 0.15    # Gillespie: 15 % driven, 10 % not
        unsprung_fraction_non_driven: 0.10
        camber_change_per_roll: 0.0       # no compensation - see OQ-5
        damping_ratio_front: 0.30         # zeta = C/C_crit, terrain quarter car
        damping_ratio_rear: 0.30          # ([S3] p.787-810)

    # The nondimensional combined-slip tyre model.
    tire:
        lateral_friction: 0.90           # dry 0.70-0.90, wet 0.25-0.50, ice 0.10-0.15
        longitudinal_friction_ratio: 1.0 # mu_x/mu_y; 1 = pure circle - OQ-1
        friction_load_sensitivity: 0.10  # mu falls 0.01 per 10 % load
        cornering_coefficient: 0.15      # 1/deg: bias 0.12, radial 0.15, perf 0.25+
        camber_stiffness_ratio: 0.10     # radial ~1/10, bias ~1/5 (Milliken)
        load_sensitivity_b: 0.0          # Eq. 6-36 `b` - see OQ-2
        peak_slip_ratio: 0.15            # dry 10-20 %; k_x is derived from it
        universal_curve: { b: 0.714, c: 1.40, d: 1.00, e: -0.20 }   # Milliken p.476
        aligning_moment_coefficient: 0.043 # ft-lb/lb/deg, radial
        vertical_rate: 175127.0          # N/m (= 1000 lb/in)
        relaxation_length_revolutions: 0.75
        static_camber: 0.0               # deg, negative = tops inboard
```

**Tyre coefficients are authored per DEGREE and angles in DEGREES**, because
that is how the Knowledge Base quotes every one of them. They are converted on
load, like every other angle in the project.

Values marked in the YAML as **not Knowledge Base values**, or as
worked-example values rather than typical ones:
`longitudinal_friction_ratio` (OQ-1), `load_sensitivity_b` (OQ-2), the roll
centre heights and ride frequencies (OQ-4), `camber_change_per_roll` (OQ-5),
and the two inertias (OQ-6).

The `engine:` block above (`max_acceleration`) is what every vehicle uses
unless it also has an `engine: detailed:` + top-level `gearbox:` block, which
switches that one vehicle to a real Heywood-sourced engine + gearbox instead —
see **[The detailed engine + gearbox model](#the-detailed-engine--gearbox-model--how-the-engine-is-connected-to-the-car)**
and `config/vehicles/sports_car.yaml` for the worked example. A `resolved:`
block inside `detailed:` makes that engine the crank-angle-resolved one; its
keys are listed, with units and defaults, in
`src/config/ResolvedEngineParameters.h`.

Note what is **no longer** in that list. The slip angle at peak lateral force
and the longitudinal slip stiffness used to be authored here with a flag on
each; both are now derived from the universal curve.

`config/simulation.yaml` additionally carries an `environment:` block with air
density, road grade and headwind, which the road load model reads; a
`default_track:` (see `--track`); and an `environment_model:` block pointing at
the raw Nürburgring scan used by the `nurburgring_scan` track.

#### Drawing the vehicle — `model:`

Purely a rendering concern; nothing here is read by the physics.

```yaml
model:
    body_style: auto        # generated car: auto | race | sports | saloon | suv
    # -- or an imported model --
    path: "assets/models/ferrari_f40.glb"
    forward_axis: "-z"      # which glTF axis is the car's forward / up
    up_axis: "+y"
    wheel_nodes:            # node whose subtree is that wheel (spins + steers)
        front_left:  "Wheel_FL"
        front_right: "Wheel_FR"
        rear_left:   "Wheel_RL"
        rear_right:  "Wheel_RR"
```

The importer (`src/rendering/GltfModel.h`) is deliberately narrow: base-colour
texture/factor only, every material opaque, one static group plus four wheel
groups. The rear-axle origin and each hub's height are **measured** from the
wheel nodes, never authored. Wheel positions in the physics still come from the
YAML's wheelbase/track — a model's wheels contribute their shape only, so the
YAML's geometry should be measured from the model (as both imported cars' are).

The F40's GLB is produced from `../F40_Render/F40.blend` by
`blender -b F40.blend --python export_sim.py`. That step groups each corner's
rotating parts under a `Wheel_*` empty (calipers stay on the chassis), drops the
studio floor, and flattens the Blender paint/glass mask shaders into per-face
flat materials the importer can read. Rerun it after changing the model.

`front_axle_to_cg` and `rear_axle_to_cg` are **derived** from `wheelbase` and
`front_weight_distribution`, never stored, so they cannot drift out of sync.

Angles are **degrees in YAML** and converted to radians on load.

Every value is validated on load and clamped to a physically sensible range.

### Runtime UI

Each editable field is tagged so you know what you are changing:

- **`[LIVE]`** — takes effect immediately, safe mid-run
- **`[RESTART]`** — stored now, applied on the next reset
- **`[DERIVED]`** — computed, read-only, greyed out

---

## Debug visualisation

Toggle individually in the **Debug visualisation** panel:

vehicle coordinate frame · world coordinate frame · centre of mass ·
velocity vector · heading vector · steering direction · wheel contact points ·
trajectory trail · turning circle · ground grid

Axis colours are consistent everywhere: **red = +X forward**,
**green = +Y up**, **blue = +Z right**.

The HUD reports velocity, speed, acceleration, steering angle, yaw rate,
turning radius, heading, position and lateral acceleration.

---

## Telemetry

Recorded at every physics step. `T` toggles recording, `F2` exports.

```
timestamp, x, y, z, yaw, velocity, acceleration,
steering_angle, yaw_rate, throttle, brake,
lateral_velocity, lateral_acceleration, sideslip, sideslip_rate,
roll_angle, wheel_data_valid,
fz_{fl,fr,rl,rr}, slip_angle_{...}, camber_{...}, slip_ratio_{...},
fx_{...}, fy_{...}, friction_usage_{...}, wheel_speed_{...}
```

Units are `s, m, m, m, rad, m/s, m/s², rad, rad/s, -, -` then
`m/s, m/s², rad, rad/s, rad, bool` then, per wheel,
`N, rad, rad, -, N, N, -, rad/s`.
Values are written with 17 significant digits so a run round-trips exactly.

Vehicles with the detailed engine also fill `engine_active, engine_rpm,
engine_torque_nm, engine_gear, engine_locked, engine_limiter_active,
engine_map_pa, engine_boost_pa, engine_lambda` (zero on every other vehicle,
lambda 1) — see the engine section. The last three were appended on
12 Oct 2026; anything reading the CSV by column *name* is unaffected.

`friction_usage` is `R(k)`, the normalised resultant force of Milliken
Eq. (14.16) — the fraction of the friction ellipse in use. It reaches 1 at the
limit and cannot exceed it.

`slip_ratio` is the Knowledge Base's own braked-wheel form,
`S = (V − R_e·ω)/V`: **0** free rolling, **+1** locked, negative under drive.

`wheel_data_valid` is 0 for rows produced by a model that has no tyres, so a
trace can never be misread as measurements that were never made.

Files go to `logs/run_001.csv`, `run_002.csv`, … The lowest unused number is
always chosen, so **existing logs are never overwritten**. A `#` comment header
records the timestamp and which physics model produced the trace.

Real-time graphs (speed, acceleration, steering, yaw rate) are in the **Graphs**
window and run independently of whether recording is armed.

---

## Experiments

Scripted, repeatable manoeuvres that **replace the driver**. Independent of the
renderer, so the same run works interactively or headless.

| Type | What it does |
|---|---|
| `straight_line` | hold the initial speed, zero steer |
| `constant_steering` | fixed steer angle at held speed — a constant-radius circle |
| `step_steering` | straight, then an abrupt steer input at `step_time` |
| `acceleration_test` | full throttle from rest |
| `braking_test` | hold speed, then full brake at `step_time` |
| `slalom` | sinusoidal steering at `period` |
| `coast_down` | release everything and let road load slow the vehicle |

```yaml
experiment:
    type: constant_steering
    initial_speed: 10.0
    steering_angle: 10.0    # degrees, positive = right
    duration: 10.0
    step_time: 1.0
    period: 3.0
```

Experiments jump the vehicle straight to the test condition rather than driving
a run-up, so the trace is not contaminated by an arbitrary approach.

### Verification

These runs were checked against closed-form results:

| Check | Theory | Measured |
|---|---|---|
| Constant-steering path radius (`L`=2.7 m, δ=10°) | `L/tan(δ)` = 15.3125 m | 15.3125 m (circle fit, 0.000 % error) |
| Steady yaw rate | `v/L·tan(δ)` = 0.65306 rad/s | 0.65306 rad/s |
| Braking stop time from 10 m/s at 8 m/s² | `v/a` = 1.2495 s | 1.340 s (+0.09 s from the 0.2 s brake ramp) |
| Braking distance | `v²/2a` = 6.245 m | 7.137 m (+0.89 m from the ramp; predicted ≈ `v₀·ramp/2` = 1.0 m) |
| Timestep convergence, dt 0.01 → 0.005 → 0.0025 | first-order | final position stable to 5 s.f. |
| Coast-down deceleration vs. closed-form road load | `f_r·g + ½ρV²C_D·A/m + kV` | agrees to **7×10⁻⁵ m/s²** across the run |
| Drag = rolling crossover | Gillespie: ~50–60 mph | 48–52 mph across the three vehicles |
| Cornering radius after adding road load | unchanged (road load is longitudinal) | 15.3125 m, 0.0001 % error |

#### Four-wheel tyre-force model

Checked against the Knowledge Base's own equations and values, with
`basic_car` as shipped:

**The tyre**

| Check | Knowledge Base | Measured |
|---|---|---|
| Universal curve peak value | `D′ = 1.00` | **1.00000**, at normalised slip **2.6778** |
| Curve is odd in its argument | required by the ± slip-angle averaging of [S3] §11.2 | `R(−1.5)` = −0.92442, `−R(1.5)` = −0.92442 |
| Lateral force peak | `μ_y·F_z`, at the slip angle the curve implies | `F_y/F_z` = **0.9000** at **15.67°**; predicted **15.66°** |
| — and against the sources | [S1] charts show saturation at 10–15° for a road tyre; [S3] gives 3–7° for race tyres | 15.7° road tyre, **9.6°** for the sports car's stiffer tyre |
| Longitudinal force peak | `μ_x·F_z` at the quoted peak slip ratio | `F_x/F_z` = **0.9000** at **0.1500** |
| Small-slip limit, lateral (Eq. 14.4) | `F_y → C_α·tan α` | 151.56 N vs 151.70 N at 0.25°, **0.09 % low** |
| Small-slip limit, longitudinal ([S3] p.483) | `F_x → k_x·S` | 129.88 N vs 129.99 N at S = 0.002, **0.08 % low** |
| Camber thrust at zero slip (Eq. 14.14 reduction) | `F_y = C_γ sin γ` | 121.33 N vs 121.33 N at 2°; 303.01 vs 303.01 at 5° |
| Camber:steer force equivalence | radial ≈ 10:1 ([S3] p.727) | **10.0 : 1** |
| **Friction ellipse** | `(F_x/μ_xF_z)² + (F_y/μ_yF_z)² ≤ 1`, as an identity | ≤ **1.000000** at every combination tested; **0.998** at the limit, never above |
| Tyre radii ordering (Eq. 5.1) | `R_u ≥ R_e ≥ R_l` | 0.3400 ≥ 0.3323 ≥ 0.3169 m |
| Spurious slip if the wrong radius is used (§2.15) | non-zero on a freely rolling tyre | **−2.3 %** with `R_u`, **+4.6 %** with `R_l`, **0.0 %** with `R_e` |
| Friction load sensitivity (§2.4.3) | −0.01 per +10 % load | μ = 0.940 at 0.6 `F_z0`, 0.860 at 1.4 `F_z0` |

**The roll chain**

| Check | Knowledge Base | Measured |
|---|---|---|
| Unsprung split (§1.6) | 10 % of GAWR non-driven, 15 % driven | `W_uF` 809 N, `W_uR` 993 N of 14 710 N |
| Roll gradient | [S3] Table 16.1 "semi-soft: contemporary middle-market sedans" = **7.0 deg/g**; [S1] range 3–7 | **6.81 deg/g** |
| — sports car | between "very firm" 4.2 and "extremely firm" 3.0 | **3.18 deg/g** |
| — SUV | "soft: basic family transportation" 7.5 | **7.50 deg/g** |
| **Three-term transfer sums to the rigid-body result** | `W·h/t_ave` ([S3] p.679, [S1] Eq. 9-1) | 5219.7 vs 5219.7 N/g — **100.0000 %** |
| Roll-rate distribution ≠ load-transfer distribution ([S3] p.679) | they differ; [S3]'s example by 10 points | **65.1 %** front roll rate against **59.1 %** front TLLTD — 6 points |
| Front TLLTD vs the recommended start ([S3] p.604) | weight distribution + 5 % | 59.1 % against 55 % + 5 % = 60 % |
| Longitudinal load transfer (Eq. 1-8/1-9) | `W·(a_x/g)·(h/L)` | +1276 N per front wheel at −8.35 m/s²; predicted **+1276 N** |

**The vehicle**

| Check | Knowledge Base | Measured |
|---|---|---|
| Speed at which sideslip changes sign (Eq. 6-24) | 11.2 m/s | **11.2 m/s** |
| Understeer progression, 5° steer, 5 → 24 m/s | `US > 1` and rising (§9.4.14) | 1.016 → 1.078 → 1.287 → 1.838 → **2.709** |
| Timestep convergence, dt 0.010 → 0.001, at-the-limit step steer | — | `a_y` 6.552 → 6.557 m/s², `β` −3.961 → −3.974°, `φ` −4.549 → −4.553° |
| Locked-wheel deceleration, μ = 0.35 | below `μ_peak·g` (§2.4.2 sliding branch) | **3.09 m/s² = 0.31 g**, all four wheels at slip = 1.000, recovering on release |
| Wheelspin and recovery | past the peak the wheel runs away (§2.4.2) | sports car spins from rest at full throttle, recovers within 0.5 s of lifting to 25 % |
| Kinematic model unchanged | `ψ̇ = v/L·tan δ` | 1.0928 vs 1.0926, `β` = 0, `φ` = 0, wheel data flagged invalid |

Two rows are worth dwelling on. The friction-ellipse row used to read
"1.000000 — clamped"; it now reads "never above 1" because nothing clamps it.
And nothing in the code decides that a wheel should lock or how much
deceleration a locked wheel gives — both fall out of the curve and the wheel's
rotational equation.

---

## Extending it

The interface to implement is `src/physics/IVehicleDynamics.h`:

```cpp
class IVehicleDynamics {
public:
    virtual void update(VehicleState& state,
                        const VehicleParameters& params,
                        const DriveCommand& command,
                        double dt) = 0;
    virtual const char* name() const = 0;
    virtual const char* assumptions() const = 0;
    virtual void reset() {}
};
```

To add a model: write the class, add one line to
`src/physics/VehicleDynamicsFactory.cpp`. Nothing else changes — the UI reports
whichever model is active and its stated assumptions automatically, and
telemetry records the model name in the CSV header.

Contract for implementors:

- `update()` advances the state by exactly `dt`. One step, no internal loops.
- `dt` is always finite and `> 0`; the caller guarantees it.
- `params` and `command` are inputs only.
- The state must be left finite — recover internally rather than propagating a
  NaN. (`VehicleState::sanitizeAll()` is the backstop; a NaN that reaches the
  state contaminates position, then the camera, then the render matrix, and the
  window goes black with no diagnostic.)
- `update()` must be pure with respect to global state, so experiments and
  telemetry stay reproducible.

`VehicleParameters` carries `mass`, `cg_height`, track widths, weight
distribution, inertias and full `TireParameters` and `SuspensionParameters`
blocks. The kinematic model reads almost none of it; the four-wheel model reads
all of it.

To replace only the **tyre**, implement something with the interface of
`src/physics/TireModel.h` — its inputs and outputs are the tyre block diagram of
KB §2.2 — and leave the vehicle model alone. That boundary has now been
exercised: swapping the [S2] per-load Magic Formula for the [S3] nondimensional
model changed that one file and its parameters, and nothing else in the physics.

The same is true of `src/physics/SuspensionModel.h`, which turns authored ride
frequencies and roll centres into a roll gradient and a three-term load
transfer. The four-wheel model asks it two questions per step and does not know
how it answers them.

**Not implemented, and why:** roll and pitch *dynamics*, rollover, ABS, traction
control, differentials beyond an open one, all-wheel drive, damper force curves,
road friction variation and banking. Most of these are missing because the
Knowledge Base does not contain enough to build them, not as a scoping
decision — see **[Open Questions](#open-questions)** for what specifically is
missing in each case, and **[What's missing for realism](#whats-missing-for-realism)**
for the full list, including the engineering gaps that are *not* source-limited.

---

## Open Questions

Everything below is a place where the Knowledge Base does not contain enough to
implement a requirement correctly. **Nothing here has been guessed, and nothing
has been filled in from general vehicle-dynamics or game-physics knowledge.**
Each entry states what is missing, what it affects, and why the affected part
cannot be completed without it.

`Gnn` references are the Knowledge Base's own gap numbers (KB §18, with the
post-[S3] status in `Tireand suspension.md` §30.5).

### What the third source closed

These were open questions in the previous version of this README. They are
listed here rather than deleted, because the record of *which source closed
what* is worth keeping.

| Was | Closed by | How |
|---|---|---|
| **A combined-slip force model** (gap **G1**) | [S3] Ch.14, KB §2.20 | The nondimensional model returns `(F_x, F_y)` for arbitrary simultaneous slip ratio, slip angle and camber. The friction ellipse became an identity of the algebra, so the arbitrary allocation rule the previous version had to admit to no longer exists |
| **The slip angle at peak lateral force** | [S3] Eqs. (14.4)–(14.7) | Derived from the cornering stiffness, the friction coefficient and the universal curve, instead of read off an [S1] chart |
| **The longitudinal slip stiffness `k_x`** | [S3] Eq. (14.10) + KB §12.2 | Derived from the KB's quoted slip ratio at peak `F_x` against the same curve. `k_x` still has no *directly quoted* value anywhere — see **OQ-3** for what that costs |
| **Anti-roll bar rate** (gap **G14**) | [S3] Ch.16, `Tire…md` §19 | A separate, authored contributor to axle roll rate |
| **Roll stiffness and roll gradient** | [S3] Ch.16/18 | Ride frequency and roll gradient are both tabulated by vehicle class, so the whole chain between them can be driven *and* checked |
| **The front/rear split of lateral load transfer** | [S3] p.682 | Computed from the three-term formula rather than authored as a fraction — and, per [S3] p.679, it is *not* the roll-stiffness ratio |
| **Camber stiffness by construction** | [S3] p.46, p.727 | radial ≈ 1/10 of cornering stiffness, bias ≈ 1/5–1/6, where [S1] gave only an undifferentiated 10–20 % band |

### A. Parameters the Knowledge Base names but never quantifies

These appear in Knowledge Base equations by name. The equations are implemented;
the values had to come from somewhere, and each is flagged in the YAML.

---

**OQ-1 — the longitudinal friction coefficient `μ_x`.**

*Missing:* a value. [S3] Eq. (14.9) names `μ_x` and KB §2.20.2 quotes the point
being made — *"included in the above definitions is the fact that friction
coefficients may differ in the lateral and longitudinal directions"* — but
`Tireand suspension.md` §27.1 records the availability of a value as **NO**,
for any tyre.

*Affects:* the shape of the friction boundary. `μ_x = μ_y` makes it a circle;
anything else makes it an ellipse, and moves how much braking force is
available relative to cornering force.

*Why it cannot be completed:* the shipped ratio of 1.0 is not a guess — it is
[S1] §2.9's explicitly named **"pure-circle case"**, the base case that source
describes. But the sources agree the real tyre is an ellipse, and none of them
says by how much.

---

**OQ-2 — `b`, the quadratic coefficient of the cornering-stiffness/load
polynomial.**

*Missing:* a numerical value. KB Eq. 6-35/6-36 write `C_α = a·F_z − b·F_z²`,
and KB §2.14 lists `a` and `b` as *"vehicle/tire specific"* with no number
anywhere. [S3] supplies measured cornering stiffnesses (KB §12.2) but each at a
single load, so no polynomial can be fitted from them either.

*Affects:* the curvature of cornering stiffness with load, and through it the
lateral-load-transfer understeer term `K_llt` (KB Eq. 6-49). KB §2.6.4 calls
Eq. 6-41 *"one of the most important results in the book"*: an axle loses
cornering stiffness in proportion to the **square** of its lateral load
transfer, governed entirely by `b`.

*Why it cannot be completed:* with `b = 0` the polynomial collapses onto the
KB's own simpler statement `C_α = CC_α·F_z` (Eq. 6-6), so the default is a KB
model rather than a guess. The consequence is narrower than it used to be —
weight transfer still changes tyre behaviour, through `μ(F_z)` and through the
normalisation itself — but the specific sub-limit mechanism of Eq. 6-49 stays
switched off. Set `tire.load_sensitivity_b` if you have measured data.

Worth noting alongside it: with a constant cornering coefficient,
`K_tires = W_f/C_αf − W_r/C_αr` is **identically zero**, so static weight
distribution has no first-order effect through that particular term. The
understeer the model does show comes from friction load sensitivity and tyre
saturation — the mechanism KB §9.4.16 describes for *saturated* slip angles.

---

**OQ-3 — the longitudinal slip stiffness `k_x`, and how it is obtained here.**

*Missing:* a directly quoted value. [S3] Eq. (14.10) names `k_x` as *"the
initial slope of longitudinal force with slip ratio"* and requires it;
`Tireand suspension.md` §27.1 records the availability as **NO (value)**.

*Affects:* how steeply longitudinal force rises with slip, and therefore how
fast a wheel locks or spins up.

*What is implemented, and what it rests on:* `k_x` is **derived**, not
authored, from two statements the KB does make —

1. the universal curve fits `F̄_x` against `S̄` with the same `B′, C′, D′, E′`
   as the lateral curve ([S3] p.479);
2. the slip ratio at peak `F_x` is 0.10–0.15 (KB §12.2; [S1] gives 15–20 % dry).

The curve's peak sits at a known normalised slip, and `S̄ = k_x·S/(μ_x·F_z)`,
so `k_x` follows. **The Knowledge Base does not perform this derivation
itself**, which is why it is recorded here rather than treated as a plain
citation. It is an inference from two KB facts, not a value from outside.

---

**OQ-4 — roll centre heights, and the ride-frequency band.**

*Missing:* two things, both about the top of the roll chain.

*Roll centre heights.* `Tireand suspension.md` §27.2 records that [S3] gives
only **worked-example** values — 0.25 and 0.625 ft, and 3.1 and 6.8 in — never
a typical range, and that roll centre migration with travel and roll is
*"acknowledged but not modelled"*. The shipped values are [S3]'s own
second-example figures.

*Ride frequency.* Two sources give incompatible bands for the same quantity.
[S3] p.582 puts passenger cars at 30–50 cpm (0.50–0.83 Hz); [S1]'s Olley
criterion puts the body natural frequency at 1.3 Hz **or below**. They barely
overlap. The shipped configs sit inside [S1]'s bound and above [S3]'s passenger
band, and the model reports the resulting **roll gradient** so it can be checked
against [S3] Table 16.1 — which is how the anti-roll bar rates were sized. Both
ends of that calibration are Knowledge Base numbers and the relation between
them is [S3]'s, but the choice of where to sit between the two bands is not.

*Affects:* the geometric term of lateral load transfer, and the overall
stiffness of the roll chain.

---

**OQ-5 — camber change with body roll, `∂γ/∂φ`.**

*Missing:* the characteristic, or an input from which to evaluate it. KB
Eq. 6-51 leaves it as an unspecified function `f_γ(track, geometry, roll
angle)` — gap **G13**. [S3] §16.4 advances it to a closed-form **rate**, camber
change per inch of travel = `arctan(1/fvsa length)`, but
`Tireand suspension.md` §27.2 records that no source gives an fvsa length for
any vehicle, so the rate cannot be evaluated. G13 remains **PARTIAL**.

*Affects:* how much of the body's roll the tyre actually sees. The default of
zero is the explicit *no camber compensation* case — the wheel leans with the
body, the loaded outside tyre gains positive camber, and it loses some lateral
force. That is a real suspension behaviour (it is what camber compensation
exists to avoid) and it is the honest limiting case, but a real independent
suspension would recover part of it.

*Related and unresolved:* **camber roll-off**. [S3] p.48 names the fact that
camber thrust and slip-angle force stop being simply additive near the limit,
and gives no magnitude or function. The nondimensional model's Eq. (14.13)
combination is what the source supplies in its place.

---

**OQ-6 — wheel inertia `I_w` and yaw inertia `I_zz`.**

*Missing:* both values. KB Eq. 3-18 names `I_w` and KB §6.7 says explicitly
that during lockup *"I_w·α_w is large and Eq. 3-18 is required"*. KB §1.11
defines the inertia tensor and the radius of gyration, and KB §12.1 lists only
the **pitch** radius of gyration, itself with no typical value. Nothing in any
of the three sources gives a **yaw** radius of gyration.

*Affects:* `I_w` sets how quickly a wheel locks or spins up, so it cannot be
omitted; `I_zz` sets the yaw response, so nor can that.

*Why it cannot be completed:* `I_w` is authored per vehicle and flagged in the
YAML. `I_zz` defaults to `m·b·c`; that form is borrowed from the Dynamic Index
`DI = k²/(b·c)` of KB Eq. 5-40, where `DI = 1.0` is called ideal — but Eq. 5-40
is about the **pitch** plane and the KB says nothing of the sort about yaw.
Treat the fallback as authored data.

---

**OQ-7 — the tyre's rated load, and the unsprung CG height.**

*Missing:* two reference quantities the load relations need.

*Rated load.* KB §2.4.3 phrases friction load sensitivity *"near the tire's
rated load"* and KB Eq. 6-6 quotes the cornering coefficient *"at 100 % rated
load"*, but no source says how to obtain it for a vehicle. The model uses each
wheel's **static** load. That is an assumption, and it is not the KB's.

*Unsprung CG height.* [S3] uses the tyre's static loaded radius in its worked
example, and KB §3.5.1 flags that as *"RCVD's own choice in this example … an
assumption of the worked example, not a general definition"*. The model does the
same, for want of anything better.

*Affects:* the reference point of both load-sensitivity relations, and the
third (unsprung) term of lateral load transfer — about 3–5 % of the total in the
shipped vehicles.

### B. Models the Knowledge Base describes but does not supply

---

**OQ-8 — roll and pitch dynamics. (KB gaps G8, G9 — OPEN in all three
sources.)**

*Missing:* any equation of motion for roll. Roll here is **quasi-static**:
`φ = (roll gradient) × a_y`, evaluated every step. KB §3.5.4 records [S3]'s own
statement that its whole wheel-load treatment is steady state and that
*"transient behavior of the vehicle and sprung and unsprung mass dynamics
(ride/roadholding) are ignored."* KB §18.2 keeps G8 (transient yaw/sideslip
equations) and G9 (yaw-roll coupled equations) OPEN after [S3].

*Affects:* the body reaches its steady roll angle instantly. There is no roll
overshoot, no roll rate, and no phase lag between steering and roll — all of
which a real vehicle has, and which KB §9.5 says reduce the rollover threshold
by 30–50 %.

*Also missing, and related:* [S3] §25 says the **damper's** contribution to
transient wheel load can be as large as the anti-roll bar's, and supplies no
equation for it. There is no pitch equivalent of the roll chain in any source;
longitudinal load transfer is the rigid-body result of KB Eq. 1-8/1-9.

---

**OQ-9 — rollover.**

*Missing:* the parameters for anything past the first model in the sequence. KB
§9.5 presents rollover as a progression of models, each lowering the threshold:
rigid-body quasi-static (computable — `t/2h`), then suspended quasi-static
(−5 %), tyre lateral deflection (−5 %), and transient overshoot (−30 % for cars,
−50 % for heavy trucks). The later ones need roll-axis geometry and tyre lateral
compliance that no source quantifies, and the transient one needs OQ-8.

*Affects:* the SUV in particular. Implementing only the first model would give a
number KB Ch.9 itself says is 30–50 % optimistic, so none is implemented.

---

**OQ-10 — the six other terms of the understeer budget.**

KB §11 is the sources' own master synthesis:
`K_total = K_tires + K_camber + K_roll steer + K_lfcs + K_at + K_llt + K_strg`.

Only `K_tires` (Eq. 6-15) is computed and reported. Of the rest:

- `K_camber` and `K_roll steer` need `∂γ/∂φ` and the roll steer coefficients
  `ε_f`, `ε_r` — **OQ-5** and, for roll steer, `Tireand suspension.md` §22.1,
  which gives the measurement rig and sign convention but no closed form;
- `K_lfcs` needs the lateral force compliances `A_f`, `A_r`. [S3] Ch.23
  classifies suspension compliance completely and gives the sign rules, and
  `Tireand suspension.md` §27.2 records **no values, no matrix** for any
  vehicle;
- `K_llt` needs `b` — **OQ-2**;
- `K_strg` needs the steering system stiffness `K_ss`.

`K_strg` is worth calling out again. KB §11.2 says *"the steering system term is
not small"* — 4–6 deg/g, *"comparable to, and often larger than, the tyre
term"* — and that *"any directional response model that includes cornering
stiffness but omits steering compliance has a first-order omission."* This model
has that omission, because no source gives `K_ss` for any vehicle.

---

**OQ-11 — the pneumatic trail as a function of slip angle. (Gap G4, PARTIAL.)**

*Missing:* `t(α, F_z)`. KB §2.8 describes `t` decreasing, reaching zero and
**reversing** as the sliding region grows, so that `M_z` peaks and decays
*before* `F_y` saturates. [S3] adds numbers to that description — KB §12.2
records that `M_z` peaks at about **half** the slip angle of the `F_y` peak, and
[S3] p.32 that the steering rim force falls **about 30 % in the last degree**
before breakaway — but still no function.

*Affects:* the steering-feel cue that warns a driver of the limit. KB §2.8 is
explicit: *"a model that omits the M_z peak-and-decay removes that cue
entirely."* `M_z = −F_y·t` is implemented with a **constant** trail derived from
the KB's aligning-moment coefficient, which is the same form KB Eq. 6-61 and
Eq. 8-8 use. It saturates with `F_y`, but it does not peak and decay.

*Also missing:* combined-slip `M_z` and `M_x`. KB §2.20.4: *"No combined-slip
M_z or M_x is given."*

---

**OQ-12 — the powertrain. (KB gaps G17 PARTIALLY CLOSED, G18/G19/G20 — OPEN.)**

*G17 update:* an engine now exists — `src/physics/Engine.h` /
`ResolvedEngine.h` / `Gearbox.h` / `Powertrain.h`, opt-in per vehicle via an
`engine: detailed:` + `gearbox:` YAML block (see
`config/vehicles/sports_car.yaml` for a worked example), in two forms: a
cycle-averaged torque map, and EngineSim's crank-angle-resolved engine. It is **not** sourced from this project's own Knowledge Base:
neither Gillespie/Blundell&Harty/Milliken gives one, which is exactly why G17
was open. It is ported from a fourth, domain-appropriate source — Heywood,
*Internal Combustion Engine Fundamentals* — via the `EngineSim` project
elsewhere in this repository. See `config/EngineParameters.h` for the full
source notice before treating anything in that block as KB-traced.

*Still missing, and NOT guessed at either:* torque converter equations
(**G18**), a differential model (**G19** — the detailed model sends equal
torque to both driven wheels, same open-differential behaviour the simple
model already had), and clutch engagement (**G20**). Their absence has a
concrete, honestly-documented consequence: what stands in for G20 is an
AUTHORED friction clutch — Coulomb friction with an authored capacity and an
authored engagement schedule (`GearboxParameters.h`) — which is what lets a
detailed-engine vehicle pull away from rest. It is a stand-in, not a closed
gap; see the engine section's "The coupling model".

*Affects, for a vehicle NOT using the detailed engine (the default, and still
every vehicle except `sports_car` and `ferrari_f40` as shipped):* everything
below is unchanged.

- Drive is a constant **force demand** (`mass × max_acceleration`) rather than
  an engine torque at a speed. Top speed is therefore a **governor**, not a
  power balance — applied to wheel surface speed, since KB §5.4/5.8 put engine
  and wheel in a fixed driveline ratio. Without it a spinning wheel has nothing
  to stop it.
- Driven wheels get **equal torque**, which is the open differential KB §3.4
  describes. No limited-slip or locking behaviour.
- **All-wheel drive is not offered.** KB §5.7.3 derives the traction limit for a
  front drive axle and for a rear drive axle; no source models a torque split
  between axles.
- **Engine braking** remains flagged as *not from Gillespie*, as it always was
  — unless the detailed engine is active, in which case `Powertrain` supplies
  the real value and this stand-in is overridden (see `RoadLoad.cpp`).

---

**OQ-13 — active systems. (KB gaps G22, G33.)**

*Missing:* an ABS control law (**G22**, PARTIAL — KB §16.4 describes the
strategy but gives no implementable law), and traction/stability control
(**G33**).

*Affects:* nothing is implemented, so flooring the throttle in a powerful car
produces sustained wheelspin and therefore **slower** acceleration than
modulated throttle — the negative-slope branch of KB §2.4.2, and exactly why
traction control exists. Likewise a locked wheel stays locked until the driver
lifts.

---

**OQ-14 — the road. (KB gaps G29, G30.)**

*Missing:* split-μ and variable-μ surfaces (**G29**, not advanced by [S3] — it
notes only that nondimensionalisation *"allows potential for including
variations of surface friction"*), and general three-dimensional road geometry
(**G30**, now PARTIAL).

*What [S3] added and this model does not use:* banked turns. KB §3.5.3 gives
`A_Y = A_α cos α − sin α` and an effective weight `W′ = W(A_α sin α + cos α)`
that replaces `W` throughout the load-transfer chain, plus the "neutral speed"
below which the *inside* wheel is the more heavily loaded. **This is
implementable and is not implemented**, because the track and terrain are flat
and decorative — adding banking to the physics without adding it to the world
would put the vehicle on a slope the scenery does not have. Road crown and a
general 3-D surface remain missing from every source.

Friction is one number for the whole world.

---

**OQ-15 — tyre thermal, pressure and wear effects. (KB gaps G5, G7 — OPEN in
all three sources.)**

*Missing:* any thermal model, the pressure and temperature dependence of `C_α`,
`C_γ` and `μ`, and any wear model. [S3] §12 names six pressure mechanisms with
their directions of effect and supplies not one coefficient, states that an
optimum tyre temperature exists without giving it, and declares speed effects
*"not generalisable"*. Tyre vertical damping is not mentioned anywhere in any
source.

*Affects:* the tyre never goes off, never warms up, and never cares what it is
inflated to.

---

**OQ-16 — dampers and suspension friction. (KB gaps G15, G16.)**

*Missing:* a damper force-velocity **function** (**G15**, now PARTIAL: [S3]
Fig. 22.26 gives one measured curve with a digressive bump / near-linear rebound
shape and a ≈3:1 rebound-to-bump ratio, plus usable damping ratios — passenger
cars ≈0.25, ride optimum ≈0.1–0.2, road-holding optimum ≈0.5 — but no
equation), and any suspension friction model (**G16**, still OPEN: [S3] names
friction as a wheel-load contributor and shows it as *"almost unnoticeable"* in
one simulation).

*Affects:* there is no damping in the roll chain, which is part of why roll is
quasi-static (**OQ-8**), and no bump-stop or progressive-spring behaviour.

---

**OQ-17 — aerodynamic line of action and lift.**

*Missing:* the height `h_a` of the aerodynamic drag's line of action, which KB
Eq. 1-3 carries as a `D_A·h_a` term in the axle-load balance; and any front/rear
split of the lift coefficient (KB gap **G27**).

*Affects:* drag is applied at the CG and produces no pitching moment, and
aerodynamic lift does not alter wheel loads. KB Eq. 1-8/1-9 — the *working* form
the model uses — drops both terms as well, so this is consistent with the KB's
own simplification, but it is a simplification. [S3] §21.8 explicitly includes
an aerodynamic term in its superposition of wheel loads.

### C. Definitions and conventions the Knowledge Base leaves open

---

**OQ-18 — behaviour at and near zero speed.**

*Missing:* any low-speed formulation. KB §2.4.1 states the problem — *"V = 0
makes slip undefined"* — and never resolves it. Slip angle has the same
singularity.

*What is implemented, and what is a choice:* the denominator is floored at
1 m/s. Below that the reported slip is smaller than the definition would give,
so the tyres make less force and the vehicle behaves lazily at parking speed.
The floor is a numerical regularisation of a documented singularity, not a
physical model.

A related consequence: because a tyre at zero slip makes zero force, there is no
static friction and nothing holds a stopped vehicle on a grade. No source gives
a standstill tyre model.

---

**OQ-19 — slip ratio conventions, and reverse.**

*Slip ratio.* KB §2.16 warns that *"slip ratio has been subject to various
definitions"* and that the driving-side normalisation *"must be confirmed, not
assumed"*. `Tireand suspension.md` §3.3 now gives the full inventory and its
conversions, including `SR_[S3] = −Slip_[S1]`, and the model uses [S3]'s SAE
form internally while reporting [S1]'s to the UI and telemetry. This one is
resolved by documentation rather than by choice — but under heavy wheelspin at
low road speed the reported ratio is large in magnitude, which is the definition
behaving as defined and not a fault.

*Reverse.* No source treats a wheel running backwards. KB §2.17 notes it took
the **Delft Tyre 97** version of the Magic Formula to *"allow the wheel to run
backwards"*. The magnitude of the wheel-plane velocity is used in the slip-angle
denominator so that a reversing vehicle measures slip against the wheel plane
rather than reporting ~180°. That is a convention, not a source.

---

**OQ-20 — the sign of the sideslip angle, and of camber.**

*Sideslip.* KB §1.3 defines `β = ν − ψ`, the signed angle between the vehicle's
`x` axis and its velocity vector. KB §9.4.7 / Eq. 6-23 works in magnitudes with
`β` positive *"into the turn"*, the **opposite sign** for a right-hand turn. KB
§13.3 documents several such magnitude-versus-signed conflicts and says they
must not be mixed. The signed definition of §1.3 is used consistently; the
*magnitude* and the *speed at which `β` changes sign* both match Eq. 6-24
exactly.

*Camber.* [S3] §7.1 flags camber angle versus inclination angle as a
sign-convention trap in its own right, and then §11.1 records that Chapter 14
*"uses camber angle interchangeably with inclination angle"* — the source
suspends its own distinction for the very chapter this tyre model comes from.
One signed convention is used here (positive = top of the wheel leaning to the
vehicle's right, thrust in the same direction) and never mixed.

---

**OQ-21 — rigid-body velocity composition, and the Ackermann fraction.**

*Velocity composition.* The slip angle at a wheel needs the velocity of that
contact patch, `v_P = v_G + ω × r_P`. All three sources assume a rigid vehicle
and [S2] Ch.2 covers rigid-body velocity analysis, but the Knowledge Base does
not reproduce this particular equation. Flagged as a derivation rather than a
citation.

*Ackermann fraction.* KB §8.2 notes that a real trapezoidal linkage can only
approximate exact Ackerman, and that linkages are *frequently designed
deliberately away from it* because at speed the loaded outside tyre wants
relatively **more** steer, not less. KB §8.12 names the "Ackermann fraction" and
gives no figure for any vehicle. Exact Ackerman (Eq. 8-1/8-2) is used.

---

**OQ-22 — the camber combination order, and numerical integration.**

*Combination order.* KB §2.20.4 flags that [S3] *"does not state whether step 2
(camber) and step 3 (slip ratio) are to be composed in this order"*. The
implementation follows the KB's own suggested sequence and inherits the flag:
what the source does state is that the same universal curve fits every one of
`F̄(ᾱ)`, `F̄(γ̄)`, `FN(β̄)`, `F̄_x(S̄)` and `R(k)`. [S3] also states Eqs.
(14.13)/(14.14) *"for positive camber angles"*; applying them to negative camber
is an interpretation.

*Integration.* KB gap **G35**, PARTIAL: no source discusses solver choice,
stiffness or timestep beyond noting that a stiff fast mode alongside a soft slow
one *"is inherently a stiff numerical problem"*. The wheel rotational equation
is therefore integrated semi-implicitly — one Newton step against the tyre's own
local slope — which is an engineering decision with no KB backing. It changes
the path to the answer and not the answer (at equilibrium the net torque is
zero, and the divisor multiplies zero), and the timestep convergence row in
[Verification](#verification) is the evidence.

---
## Known limitations

- The kinematic model has no traction limit — see the assumptions above. The
  four-wheel model does; that is most of the point of it.
- The four-wheel model has **roll but not roll dynamics**: the body reaches its
  steady roll angle instantly, with no overshoot, no roll rate and no lag behind
  the steering. No source gives a roll equation of motion. Longitudinal load
  transfer is computed at the rigid-body level from Eq. 1-8/1-9, without a pitch
  angle of its own. The per-wheel quarter car adds heave/pitch/roll **from
  terrain only**, each corner decoupled. See **OQ-8**.
- Roll, and terrain heave/pitch/roll, are **drawn** as a rigid body attitude;
  there is no articulated suspension geometry to show.
- No rollover check. See **OQ-9** for why only the crudest of the Knowledge
  Base's four rollover models is computable, and why that one is not enough.
- Only the **Nordschleife** and the raw **`nurburgring_scan`** have elevation.
  Every other track is flat, and no track has banking or camber (**OQ-14**).
  `environment.grade` applies a gravity component without tilting the scenery.
- A stopped vehicle on a grade will creep, because a tyre at zero slip makes
  zero force and no source gives a standstill tyre model. See **OQ-18**.
- **Off-track is not different from on-track**: grass, gravel and kerbs have the
  same friction as asphalt, and barriers, grandstands and other scenery have no
  collision.
- The detailed engine's speed follows **road** speed, not driven-wheel speed, so
  wheelspin does not drag the revs up — see the engine section's open items.
- The resolved engine takes real CPU: 8–14× faster than real time for the
  engine alone on one 2 GHz core. A slow machine running a V8 near redline at
  a raised time scale can run out; `resolved: steps_per_ms` and
  `crank_angle_resolution_deg` are the quality/speed knobs (coarser substeps
  cost a few percent of torque accuracy — `../EngineSim/UPDATES.md` §0.1).
- The physics has **one wheel radius**. Staggered tyres (F40) are drawn at their
  own sizes but simulated at the rear's.
- Imported models: base colour only (no PBR), glass transparent but unsorted,
  calipers and the cockpit steering wheel do not turn.
- Sound loudness is balanced, not calibrated; engine sound only on
  detailed-engine vehicles; no reverb/ground reflection; the F40's cockpit
  glass edge follows the mesh faces, so it is jagged.
- Config-only edits (YAML) are copied next to the binary only when the target
  relinks — run `cmake -E copy_directory config build/bin/config` or touch a
  source file after editing one.
- Ackermann inside/outside wheel angles are shared between the renderer and the
  physics (`src/physics/SteeringGeometry.h`). Under the four-wheel model the
  tyres really are steered to those angles; under the kinematic model they
  remain a visual refinement, because a bicycle model has one steer angle.

---

## What's missing for realism

What would most improve how real the simulator feels and measures, grouped by
area. **Source** says whether the gap is blocked by the Knowledge Base (an OQ /
KB gap — needs a new reference before it can be done under this project's
rules) or is an engineering gap that could be built from what is already here.
Roughly in order of impact within each group.

### Driveline and engine

| Missing | Why it matters | Source |
|---|---|---|
| Engine coupled to **driven-wheel speed**, not road speed | Wheelspin currently runs away without dragging the revs; launches and traction-limited acceleration are wrong for every detailed-engine car | Engineering |
| **Limited-slip / locking differential** | Open diff (equal torque) is the least realistic case for any performance car; affects corner exit, power oversteer | KB G19 |
| **Shift time and torque interruption** | Automatic shifts are instant and lossless | Engineering (G20-adjacent) |
| **Reverse gear**, **AWD**, **torque converter** | Cannot back out of a spin; no AWD or automatic cars | Input missing / G18 |
| **Mixture enrichment; knock on boosted charge** | The resolved engine times its spark (MBT, idle trim, knock retard) but is always stoichiometric, so it cannot show why turbo SI engines run rich; its knock model saturates under boost and is off for the F40 | EngineSim open items (`UPDATES.md` §7) |
| **Intake and exhaust wave dynamics** | Lumped plenums: no ram or pulse tuning, so both shipped engines fall away at the top end — the sports car makes about half a real 2.0 L's power | EngineSim open item |
| **Measured turbo maps; surge** | The resolved turbo is real components (turbine, compressor, shaft, wastegate) but its compressor map is a scalable shape and its turbine an orifice — every figure for the F40's IHI units is authored | Needs maps |
| **Engine torque calibration** | The F40 is ~19 % short on torque and ~20 % on power against published figures; friction is Heywood's Eq 13.6, extrapolated past the speeds it was fitted to | Engineering — see *Calibration, stated honestly* |
| **Traction control, launch control, boost-by-gear** | Full throttle in a low gear simply spins the tyres; the F40's 0–100 km/h is tyre-limited | Engineering |
| **Fuel consumption and mass change** | Mass and CG are constant over a stint | Engineering |

### Tyres

| Missing | Why it matters | Source |
|---|---|---|
| **Thermal, pressure and wear** effects | Grip is identical on lap 1 and lap 20, cold or hot | KB G5, G7 — **OQ-15** |
| **Load sensitivity of cornering stiffness** (`load_sensitivity_b`) | Set to 0 on every vehicle — no measured value | **OQ-2** |
| **Independent `μ_x`** | Friction ellipse is a circle | **OQ-1** |
| **Standstill / low-speed tyre model** | Creep on a grade at rest; slip ratios ill-defined near 0 m/s | **OQ-18** |
| **Per-axle tyre sizes** (staggered radius, width, stiffness) | F40 and most mid-engined cars run bigger rear tyres | Engineering |

### Chassis and suspension

| Missing | Why it matters | Source |
|---|---|---|
| **Roll and pitch dynamics** (coupled body equations of motion) | No roll overshoot or phase lag; transient handling and weight transfer timing are wrong | KB G8, G9 — **OQ-8** |
| **Suspension kinematics**: camber gain, bump steer, roll steer, anti-dive/anti-squat | Camber change is zero; pitch under braking/acceleration is not resisted by geometry | **OQ-5**, OQ-10 |
| **Damper force curves** (non-linear, bump/rebound) and suspension friction | Only a linear damping ratio on the terrain quarter car | KB G15, G16 — **OQ-16** |
| **Bump stops, travel limits, ride height → aero** | Suspension can travel without limit | Engineering |
| **Rollover** | Not detected | **OQ-9** |
| **Compliance** (bushings, chassis torsion) | Everything is rigid | Not in KB |

### Aerodynamics

| Missing | Why it matters | Source |
|---|---|---|
| **Downforce / lift, front-rear split, centre of pressure** | The GT3 has its drag but none of its downforce; high-speed grip and balance are badly wrong for any winged car | **OQ-17** |
| **Ride-height and yaw sensitivity**, slipstream | No aero platform, no drafting | Not in KB |

### Brakes and driver aids

| Missing | Why it matters | Source |
|---|---|---|
| **Brake torque model** (pressure → caliper → disc torque per wheel) | Brakes are a deceleration demand split by a fixed bias | Engineering |
| **Brake temperature and fade** | Brakes never fade | Not in KB |
| **ABS, traction control, stability control** | No modern car behaves without them | KB G22, G33 — **OQ-13** |
| **Adjustable brake bias in the cockpit** | Bias is derived or fixed per vehicle | Engineering |

### Track and environment

| Missing | Why it matters | Source |
|---|---|---|
| **Surface friction by material** (asphalt, kerb, grass, gravel, wet) | Running wide costs nothing | KB G29/G30 — **OQ-14** |
| **Banking and camber**, elevation on every track | Only the Nordschleife has real elevation | **OQ-14** / data |
| **Collision** with barriers and scenery | Cars drive through armco | Engineering |
| **Weather**: rain, standing water, temperature | One dry condition | Not in KB |
| **Road roughness / surface profile** | The quarter car only ever sees smooth terrain | Engineering (the model is ready for it) |

### Numerics

| Missing | Why it matters | Source |
|---|---|---|
| **Semi-implicit or higher-order integration**, sub-stepping stiff parts | Explicit Euler at 100 Hz is marginal for tyre relaxation, tyre vertical rate and the unsprung mass | **OQ-22** / engineering |

### Driver interface and presentation

| Missing | Why it matters | Source |
|---|---|---|
| **Steering force feedback** from aligning moment | The aligning moment is computed but never reaches the driver — the primary grip cue in a real car | Engineering |
| **Analogue wheel/pedal input** (SDL / HID) | Keyboard is on/off; the Android controller is the only analogue input | Engineering |
| **Calibrated sound** — absolute levels, cabin transfer functions, ground reflection, reverb | Sound is physics-*shaped* but levels are balanced by ear-free targets, not measured | Engineering |
| **Engine sound for simple-model cars** | Only detailed-engine vehicles have an engine to listen to | Needs their `engine: detailed:` data |
| **Animated steering wheel, working gauges, mirrors** | Cockpit instruments are static geometry | Engineering |
| **PBR materials, sorted transparency** for imported models | No metallic/roughness; glass panes are unsorted | Engineering |
