# Vehicle Simulator

A physics-based driving simulator in two parts that run together:

- **[VehicleSimulator](VehicleSimulator/)** — a real-time 3D vehicle-dynamics simulator in C++17. A four-wheel combined-slip tyre model with load transfer, body roll and camber, traction control, tyre compounds, a clutch and gearbox, live telemetry graphs, CSV logging and a headless experiment runner.
- **[EngineSim](EngineSim/)** — a crank-resolved internal-combustion engine simulator in Python, with a browser dashboard and synthesised engine sound. Its cylinder core is also vendored into the vehicle simulator, so the detailed-engine cars (such as the F40 and sports car) run the same engine model.

It starts with a **Ferrari F40** (twin-turbo V8) on the **Apex Grand Prix Circuit**, an original 4.9 km, 13-corner track. Handling is not scripted: understeer, oversteer, wheel lock and wheel spin emerge from the tyre and load-transfer model.

## How it is made

- **Equations come from textbooks, not from game physics.** The vehicle model follows Gillespie (*Fundamentals of Vehicle Dynamics*), Blundell & Harty (*The Multibody Systems Approach to Vehicle Dynamics*) and Milliken & Milliken (*Race Car Vehicle Dynamics*); the engine model follows Heywood (*Internal Combustion Engine Fundamentals*). Where the sources do not give enough to implement something, it is left out and listed as an open question in the simulator's README instead of being guessed.
- **Physics is separate from rendering.** The physics core (`vsim_core`) has no knowledge of OpenGL, GLFW or ImGui, so it can run headless for experiments and regression tests.
- **One engine, two languages.** EngineSim is the reference implementation. Its inner loop is a small C file, compiled on demand there (about 30x faster than pure Python) and vendored unchanged into the C++ simulator. Parity tools (`engine_parity`, `engine_sound_parity`) check the C++ port against traces recorded from EngineSim.
- **Tested.** 207 Python tests cover the engine model, engine files, the compiled core and the resolved cycle. The C++ side has headless experiments that write telemetry to CSV, and a lap test that reports whether the car stayed on the road.

## Quick start

### 1. Vehicle simulator (C++)

Needs a C++17 and a C compiler, CMake 3.20+, and OpenGL 3.3. GLM, yaml-cpp and GLFW are used from your system if present and downloaded otherwise; Dear ImGui is always downloaded, so the first configure needs network access.

```bash
# optional, makes the first build much faster
sudo pacman -S glfw glm yaml-cpp cmake ninja                       # Arch
sudo apt install libglfw3-dev libglm-dev libyaml-cpp-dev cmake ninja-build   # Debian / Ubuntu

cd VehicleSimulator
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/bin/VehicleSimulator
```

Drive with **W / A / S / D**, brake with **S**, shift with **X / Z**, reset with **R**, hide the panels with **F1**, change camera with **V**, quit with **Esc**. The full key list is in the [simulator README](VehicleSimulator/README.md#controls).

```bash
./build/bin/VehicleSimulator --vehicle sports_car              # pick a vehicle
./build/bin/VehicleSimulator --track nordschleife              # pick a track
./build/bin/VehicleSimulator --camera cockpit                  # pick a camera
./build/bin/VehicleSimulator --headless --experiment braking_test --duration 6   # no window, writes logs/run_001.csv
./build/bin/VehicleSimulator --help
```

Vehicles: `ferrari_f40` (default), `sports_car`, `basic_car`, `suv`, `gt3_redbull`.
Tracks: `apex_gp` (default), `race_circuit`, `test_track`, `straight`, `nordschleife`, `nurburgring`, `nurburgring_scan`.
Defaults are set by `default_vehicle` and `default_track` in `VehicleSimulator/config/simulation.yaml`.

### 2. Engine simulator (Python)

Needs Python 3.11+. The engine core and dashboard use only the standard library; `pytest` is needed for the tests. A C compiler (`gcc`, `cc` or `clang`) is optional and makes it much faster.

```bash
cd EngineSim
python interactive_dashboard.py                  # opens http://127.0.0.1:8080
python interactive_dashboard.py --engine vr38    # engines: 454, 1nz, vr38
python run_experiments.py                        # six engine-only experiments -> output/*.csv
python -m pytest tests/                          # 207 tests
```

In the dashboard, click or press a key to enable sound, hold **S** to crank, use **Q W E R** or **Space** for throttle, and press **H** for every shortcut.

## Repository layout

```
VehicleSimulator/   C++17 simulator: src/, config/ (simulation + vehicles), assets/, tools/, CMakeLists.txt
EngineSim/          Python engine simulator: engine_sim/, config/engines/, dashboard/, tests/
```

Each folder has its own README with the full details. The simulator's README is long and covers the tyre model, the roll chain, traction control, the engine coupling and the open questions.

## Not included in this repository

- **Two third-party 3D models:** the Mercedes-AMG GT3 and the Nürburgring GP. `gt3_redbull` and `nurburgring` still run, using a procedural car body and ground.
- **The knowledge-base documents** the READMEs and source comments refer to (`VEHICLE_DYNAMICS_KNOWLEDGE_BASE.md`, `ENGINE_KB.md`, `Tireand suspension.md`, `KB_GAPS.md`). They are derived from copyrighted textbooks, so links to them in the READMEs do not resolve.
- Build output and generated telemetry.

Textures in `VehicleSimulator/assets/textures/venue` are CC0 material from [ambientCG](https://ambientcg.com). The Ferrari F40 model is the project's own.
