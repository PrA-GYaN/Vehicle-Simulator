"""Interactive Engine Simulator — real-time dashboard with synthesized sound.

Runs the powertrain simulation (engine + clutch + gearbox + vehicle + dyno)
in real time on a background thread and serves a browser dashboard that
streams telemetry over Server-Sent Events. Engine sound is synthesized in the
browser (Web Audio worklet) from the simulated firing events.

No external Python packages required — stdlib only.

Usage:
    python interactive_dashboard.py [--port 8080] [--engine 454|1nz] [--no-browser]

Keyboard controls are listed in the dashboard (press H).
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import threading
import time
from dataclasses import asdict, fields
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from engine_sim.powertrain import (
    Powertrain, Controls, LB_FT_PER_NM, W_PER_HP,
)
from engine_sim.engine_file import (
    EngineFileError, list_engines, load_engine_file, resolve_engine,
)
from engine_sim.sound_spec import sound_spec
from engine_sim.native_core import native_status
from engine_sim.cycle_trace import cylinder_pressure_trace, resolved_pressure_trace
from engine_sim.valve_profile import lift_table, valve_profiles_from_config

STATIC_DIR = os.path.join(HERE, "dashboard")
DT = 0.001                    # physics step [s] for the cycle-averaged engine
DT_RESOLVED = 0.002           # ... and for the crank-resolved one: half the per-step overhead, same cylinder cost
FRAME_S = 1.0 / 60.0          # telemetry publish period [s]
TRACE_EVERY = 6               # frames between pressure-trace updates
FUEL_DENSITY_KG_L = 0.745
USD_PER_GAL = 4.75
M3S_TO_CFM = 2118.88
PA_PER_INHG = 3386.39
WATCH_S = 0.4                 # how often the engine file is checked for edits
CONTENT_TYPES = {".html": "text/html", ".js": "text/javascript",
                 ".css": "text/css", ".svg": "image/svg+xml"}


class Simulation:
    """Owns the powertrain and advances it in real time on a thread."""

    def __init__(self, engine_ref: str):
        self.lock = threading.Lock()
        self.frame_cond = threading.Condition()
        self.frame_id = 0
        self.frame_json = "{}"
        self.time_scale = 1.0
        self.steps_per_s = 0.0
        self.rt_ratio = 1.0
        self._trace = None
        self.cfg_version = 0           # bumped whenever a (re)loaded engine file is applied
        self.cfg_error = None          # message while the engine file on disk is invalid
        self._load(engine_ref)
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        self._watcher = threading.Thread(target=self._watch, daemon=True)
        self._watcher.start()

    # -- engine file -----------------------------------------------------
    @staticmethod
    def _stamp(path: str):
        try:
            st = os.stat(path)
            return (st.st_mtime_ns, st.st_size)
        except OSError:
            return None

    def _load(self, ref: str) -> None:
        """Load an engine (id or .toml path) and start it fresh. Raises EngineFileError."""
        path = resolve_engine(ref)
        le = load_engine_file(path)
        self.path = path
        self.preset = le.id
        self.file_stamp = self._stamp(path)
        self.pt = Powertrain(le.engine, le.vehicle)
        self.dt = self._step_for(self.pt)
        self._apply_budget()
        self.cfg_version += 1
        self.cfg_error = None
        self._trace = None

    @staticmethod
    def _step_for(pt) -> float:
        return DT_RESOLVED if pt.engine.resolved else DT

    def _apply_budget(self) -> None:
        """Give the resolved core a share of each physics step's wall-clock time.

        A physics step may take dt / time_scale seconds of wall time to stay in step
        with the clock; the cylinders get 70 % of that, the rest being the
        powertrain, the snapshots and the browser stream. The core coarsens its
        substeps at speed if it ever runs over (the quality governor).
        """
        self.pt.engine.set_realtime_budget(0.7 * self.dt / self.time_scale)

    def reload(self) -> bool:
        """Re-read the current engine file and apply it WITHOUT stopping the engine.
        On a mistake in the file the old settings keep running and the error is reported."""
        stamp = self._stamp(self.path)
        try:
            le = load_engine_file(self.path)
        except EngineFileError as e:
            self.cfg_error = str(e)
            self.file_stamp = stamp
            print(f"  ! {e}")
            return False
        with self.lock:
            self.pt.apply_configs(le.engine, le.vehicle)
            self.dt = self._step_for(self.pt)
            self._apply_budget()
            self.preset = le.id
            self.file_stamp = stamp
            self.cfg_version += 1
            self.cfg_error = None
            self._trace = None
        print(f"  Reloaded {os.path.basename(self.path)}")
        return True

    def _watch(self) -> None:
        """Apply edits to the engine file as soon as it is saved (waits for the write to settle)."""
        while True:
            time.sleep(WATCH_S)
            stamp = self._stamp(self.path)
            if stamp is None or stamp == self.file_stamp:
                continue
            time.sleep(0.15)
            if self._stamp(self.path) != stamp:
                continue                    # still being written
            self.reload()

    def _rel_path(self) -> str:
        try:
            return os.path.relpath(self.path, HERE)
        except ValueError:
            return self.path

    def engine_info(self) -> dict:
        cfg = self.pt.cfg
        n = cfg.num_cylinders
        spec = sound_spec(cfg)
        # The lift curves the physics uses, so the plot and the animation show the
        # flow area the model actually saw (asymmetric cams included).
        intake_valve, exhaust_valve = valve_profiles_from_config(cfg)
        step = 5.0
        valve_lift = {
            "step_deg": step,
            "intake": [round(x / intake_valve.max_lift_m, 4) for x in lift_table(intake_valve, step)],
            "exhaust": [round(x / exhaust_valve.max_lift_m, 4) for x in lift_table(exhaust_valve, step)],
            "intake_peak_deg": intake_valve.peak_deg,
            "exhaust_peak_deg": exhaust_valve.peak_deg,
            "intake_max_mm": intake_valve.max_lift_m * 1000.0,
            "exhaust_max_mm": exhaust_valve.max_lift_m * 1000.0,
        }
        return {
            "resolved": self.pt.engine.resolved,
            "turbo": bool(self.pt.engine.resolved and cfg.use_turbo),
            "vvt": bool(cfg.vvt_intake_max_deg != cfg.vvt_intake_min_deg
                        or cfg.vvt_exhaust_max_deg != cfg.vvt_exhaust_min_deg),
            "valve_lift": valve_lift,
            "preset": self.preset,
            "presets": [{"id": e.id, "name": e.name, "error": e.error} for e in list_engines()],
            "config_file": self._rel_path(),
            "config_version": self.cfg_version,
            "config_error": self.cfg_error,
            "sound": spec,                     # everything the synth needs, derived from the engine file
            "valves": spec["valves"],
            "name": cfg.name,
            "displacement_l": cfg.total_displacement * 1000.0,
            "displacement_ci": cfg.total_displacement * 61023.7,
            "cylinders": n,
            "firing_order": cfg.firing_order,
            "fire_angle_deg": spec["fire_angle_deg"],
            "bank": cfg.cylinder_bank,
            "bank_angle_deg": cfg.bank_angle_deg,
            "bore_mm": cfg.bore_m * 1000.0,
            "stroke_mm": cfg.stroke_m * 1000.0,
            "rod_ratio": cfg.rod_ratio,
            "idle_rpm": cfg.idle_rpm,
            "redline_rpm": cfg.redline_rpm,
            "gears": len(self.pt.vehicle.gear_ratios),
            "wheel_radius_m": self.pt.vehicle.wheel_radius_m,
            "final_drive": self.pt.vehicle.final_drive,
        }

    # -- control API -----------------------------------------------------
    def apply(self, data: dict) -> dict:
        with self.lock:
            action = data.get("action")
            err = None
            try:
                if action == "reset":
                    self._load(self.path)                 # re-reads the file: picks up your latest edits
                elif action == "preset" and data.get("name"):
                    self._load(data["name"])
            except EngineFileError as e:
                err = self.cfg_error = str(e)
                print(f"  ! {e}")
            if action == "reload":
                threading.Thread(target=self.reload, daemon=True).start()   # takes the lock itself
            if "time_scale" in data:
                self.time_scale = max(1.0 / 64.0, min(1.0, float(data["time_scale"])))
                self._apply_budget()
            ctl = data.get("controls", {})
            c = self.pt.controls
            for f in fields(Controls):
                if f.name in ctl:
                    v = ctl[f.name]
                    cur = getattr(c, f.name)
                    setattr(c, f.name, type(cur)(v) if not isinstance(cur, bool) else bool(v))
            c.throttle = max(0.0, min(1.0, c.throttle))
            c.brake = max(0.0, min(1.0, c.brake))
            c.clutch_pedal = max(0.0, min(1.0, c.clutch_pedal))
            c.dyno_rpm = max(0.0, min(self.pt.cfg.redline_rpm, c.dyno_rpm))
            return {"ok": err is None, "error": err, "controls": asdict(c), "time_scale": self.time_scale}

    # -- loop ------------------------------------------------------------
    def _run(self) -> None:
        wall_prev = time.perf_counter()
        sim_debt = 0.0
        next_frame = wall_prev
        frames = 0
        step_count, rate_t0 = 0, wall_prev
        sim_adv, wall_adv = 0.0, 0.0
        while True:
            now = time.perf_counter()
            wall_dt = now - wall_prev
            wall_prev = now
            sim_debt += wall_dt * self.time_scale
            sim_debt = min(sim_debt, 0.05)       # drop time if we fall far behind
            steps = 0
            with self.lock:
                dt = self.dt
                while sim_debt >= dt:
                    self.pt.step(dt)
                    sim_debt -= dt
                    steps += 1
            step_count += steps
            sim_adv += steps * dt
            wall_adv += wall_dt
            if now - rate_t0 >= 0.5:
                self.steps_per_s = step_count / (now - rate_t0)
                self.rt_ratio = sim_adv / wall_adv if wall_adv > 0 else 0.0
                step_count, rate_t0, sim_adv, wall_adv = 0, now, 0.0, 0.0
            if now >= next_frame:
                next_frame = max(next_frame + FRAME_S, now)
                frames += 1
                with self.lock:
                    snap = self._snapshot(frames % TRACE_EVERY == 0)
                payload = json.dumps(snap, separators=(",", ":"))
                with self.frame_cond:
                    self.frame_json = payload
                    self.frame_id += 1
                    self.frame_cond.notify_all()
            time.sleep(0.001)

    def _snapshot(self, with_trace: bool) -> dict:
        pt, cfg, c = self.pt, self.pt.cfg, self.pt.controls
        o = pt.last
        e = o.engine
        rpm = o.engine_rpm
        omega = rpm * math.pi / 30.0
        torque_nm = o.dyno_torque if c.dyno_enabled else o.flywheel_torque
        fuel_l = o.fuel_used_kg / FUEL_DENSITY_KG_L
        fuel_gal = fuel_l / 3.78541
        miles = o.distance_m / 1609.34
        lam = e.lambda_val if e.combusting else float("inf")
        o2 = 20.9 if not e.combusting else max(0.0, 20.9 * (lam - 1.0) / lam)
        if with_trace or self._trace is None:
            bank = pt.engine.bank
            if bank is not None:
                # The cylinder's own recorded pressure, not a reconstruction.
                tr = resolved_pressure_trace(bank.cylinders[0])
            else:
                tr = cylinder_pressure_trace(
                    cfg, e.intake_pressure, e.fuel_mass_per_cycle,
                    e.spark_timing_deg, e.combustion_duration_deg or 50.0,
                    exhaust_pa=cfg.exhaust_back_pressure_pa)
            self._trace = {"p": [round(x, 1) for x in tr["p"]],
                           "v": [round(x, 1) for x in tr["v"]]}
        snap = {
            "t": round(pt.time_s, 4),
            "rpm": rpm,
            "throttle": c.throttle,
            "ignition": c.ignition,
            "starter": c.starter,
            "combusting": e.combusting,
            "fuel_cut": e.fuel_cut,
            "limiter": e.limiter_active,
            "stalled": c.ignition and rpm < 1.0 and not c.starter,
            "gear": o.gear,
            "clutch": o.clutch_engagement,
            "clutch_locked": o.clutch_locked,
            "auto_clutch": c.auto_clutch,
            "brake": c.brake,
            "speed_mph": o.vehicle_speed_ms * 2.23694,
            "dyno": c.dyno_enabled,
            "dyno_rpm": c.dyno_rpm,
            "torque_lbft": torque_nm * LB_FT_PER_NM,
            "hp": torque_nm * omega / W_PER_HP,
            "torque_nm": torque_nm,
            "indicated_nm": e.indicated_torque,
            "friction_nm": e.friction_torque,
            "pumping_nm": e.pumping_torque,
            "accessory_nm": e.accessory_torque,
            "clutch_nm": o.clutch_torque,
            "map_kpa": e.intake_pressure / 1000.0,
            "map_inhg": (e.intake_pressure - cfg.ambient_pressure_pa) / PA_PER_INHG,
            "map_ratio": e.intake_pressure / cfg.ambient_pressure_pa,
            "air_scfm": e.mass_flow_rate / 1.204 * M3S_TO_CFM,
            "ve": e.volumetric_efficiency,
            "afr": e.afr if e.combusting else None,
            "o2": o2,
            "spark_deg": e.spark_timing_deg,
            "iac": e.iac_position,
            "resolved": e.resolved,
            "exh_kpa": e.exhaust_pressure / 1000.0,
            "imep_bar": e.imep_net_bar,
            "pmep_bar": e.pmep_bar,
            "peak_bar": e.peak_pressure_pa / 1e5,
            "knock_deg": e.knock_retard_deg,
            "boost_kpa": e.boost_pressure / 1000.0,
            "turbo_krpm": e.turbo_speed_rps * 0.06,
            "wastegate": e.wastegate_position,
            "vvt_in": e.vvt_intake_deg,
            "vvt_ex": e.vvt_exhaust_deg,
            "misfire": e.misfire_rate,
            "burn_q": e.burn_quality,
            "residual": e.residual_fraction,
            "nox_ppm": e.nox_ppm,
            "fuel_l": fuel_l,
            "fuel_gal": fuel_gal,
            "fuel_usd": fuel_gal * USD_PER_GAL,
            "mpg": miles / fuel_gal if fuel_gal > 1e-6 else 0.0,
            "miles": miles,
            "time_scale": self.time_scale,
            "cfg_ver": self.cfg_version,
            "cfg_err": self.cfg_error,
            "steps_per_s": self.steps_per_s,
            "substep_deg": (rpm * 6.0 * self.dt / max(1, pt.engine.bank.substeps_last)
                            if pt.engine.bank is not None else 0.0),
            "rt_ratio": self.rt_ratio,
        }
        if with_trace:
            snap["trace"] = self._trace
        return snap


SIM: Simulation = None


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *args):
        pass

    def _send(self, code: int, body: bytes, ctype: str) -> None:
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, obj) -> None:
        self._send(200, json.dumps(obj).encode(), "application/json")

    def do_GET(self):
        path = urlparse(self.path).path
        if path == "/api/stream":
            return self._stream()
        if path == "/api/config":
            with SIM.lock:
                return self._json(SIM.engine_info())
        if path == "/":
            path = "/index.html"
        name = os.path.normpath(path.lstrip("/"))
        full = os.path.join(STATIC_DIR, name)
        if name.startswith("..") or not os.path.isfile(full):
            return self._send(404, b"not found", "text/plain")
        with open(full, "rb") as f:
            body = f.read()
        self._send(200, body, CONTENT_TYPES.get(os.path.splitext(full)[1], "application/octet-stream"))

    def do_POST(self):
        length = int(self.headers.get("Content-Length", 0))
        try:
            data = json.loads(self.rfile.read(length) or b"{}")
        except json.JSONDecodeError:
            data = {}
        if urlparse(self.path).path == "/api/control":
            return self._json(SIM.apply(data))
        self._send(404, b"not found", "text/plain")

    def _stream(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        last = -1
        try:
            while True:
                with SIM.frame_cond:
                    SIM.frame_cond.wait_for(lambda: SIM.frame_id != last, timeout=1.0)
                    last, payload = SIM.frame_id, SIM.frame_json
                self.wfile.write(f"data: {payload}\n\n".encode())
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        self.close_connection = True


def main():
    global SIM
    parser = argparse.ArgumentParser(description="Engine Simulator dashboard")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--engine", default="454", metavar="ID_OR_FILE",
                        help="engine id from config/engines/ (e.g. 454, 1nz) or a path to an engine .toml file")
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args()

    try:
        SIM = Simulation(args.engine)
    except EngineFileError as e:
        sys.exit(f"\n  Cannot start: {e}\n")
    server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    server.daemon_threads = True
    url = f"http://127.0.0.1:{args.port}"
    print("\n  Engine Simulator Dashboard")
    print("  ──────────────────────────")
    print(f"  Engine: {SIM.pt.cfg.name}   ({SIM._rel_path()})")
    bank = SIM.pt.engine.bank
    print(f"  Core:   {'compiled (C) cylinder loop' if getattr(bank, 'native', False) else native_status()}")
    print("  Edit that file and save: changes apply live, no restart needed.")
    print(f"  Open:   {url}")
    print("  Keys:   A ignition · S (hold) starter · Q/W/E/R or Space throttle · H help")
    print("  Ctrl+C to stop\n")
    if not args.no_browser:
        import webbrowser
        threading.Timer(0.5, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n  Stopped.")
        server.server_close()


if __name__ == "__main__":
    main()
