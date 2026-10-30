"""Engine definition files (config/engines/*.toml): loading, validation, the sound spec derived
from them, live re-tuning of a running engine, and the dashboard's hot reload."""

import math
import os
import sys
import time

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from engine_sim.engine_file import (EngineFileError, ENGINES_DIR, list_engines, load_engine_file,
                                    resolve_engine)
from engine_sim.powertrain import Powertrain, load_engine, preset_chevy_454, preset_1nz_fe
from engine_sim.sound_spec import sound_spec, speed_of_sound

F454 = os.path.join(ENGINES_DIR, "chevrolet_454.toml")
F1NZ = os.path.join(ENGINES_DIR, "toyota_1nz_fe.toml")


def edited(tmp_path, base, **changes):
    """Copy an engine file, replacing whole lines starting with `key =`."""
    lines = open(base).read().splitlines()
    for key, new in changes.items():
        idx = [i for i, ln in enumerate(lines) if ln.startswith(key + " =")]
        assert idx, f"{key} not in {base}"
        lines[idx[0]] = f"{key} = {new}" if new is not None else ""
    p = tmp_path / "e.toml"
    p.write_text("\n".join(lines) + "\n")
    return str(p)


def test_shipped_engines_load_and_are_listed():
    ids = {e.id: e for e in list_engines()}
    assert {"454", "1nz"} <= set(ids)
    assert all(e.error is None for e in ids.values())
    assert resolve_engine("454") == F454 and resolve_engine(F1NZ) == F1NZ
    ecfg, vcfg = preset_chevy_454()
    assert ecfg.num_cylinders == 8 and ecfg.exhaust_layout == "dual" and len(vcfg.gear_ratios) == 4
    ecfg, _ = preset_1nz_fe()
    assert ecfg.num_cylinders == 4 and ecfg.exhaust_layout == "single"


def test_unknown_engine_lists_the_available_ones():
    with pytest.raises(EngineFileError, match="Available: .*454"):
        resolve_engine("no_such_engine")


def test_sound_spec_follows_the_file():
    sp = sound_spec(load_engine_file(F454).engine)
    assert sp["cylinders"] == 8 and sp["exhaust"]["layout"] == "dual" and len(sp["exhaust"]["pipes"]) == 2
    assert sp["exhaust"]["cylinder_pipe"] == [0, 1, 0, 1, 0, 1, 0, 1]
    assert sp["fire_angle_deg"] == [0, 630, 270, 180, 450, 360, 540, 90]
    assert sp["valves"] == {"intake_open": 710, "intake_dur": 240, "exhaust_open": 490, "exhaust_dur": 240}
    assert sp["evo_after_fire_deg"] == 130 and sp["ivo_after_fire_deg"] == 350
    h = sp["exhaust"]["pipes"][0]["sections"][0]
    assert h["length_m"] == 0.85 and math.isclose(h["c"], speed_of_sound(725), rel_tol=1e-9)
    assert math.isclose(speed_of_sound(725), 540, rel_tol=0.005)
    assert sp["exhaust"]["pipes"][1]["sections"][0]["length_m"] == 1.00        # per-pipe list
    assert math.isclose(sp["blowdown_tau_max_s"], 1.75 * 0.10795 / h["c"])
    one = sound_spec(load_engine_file(F1NZ).engine)
    assert one["exhaust"]["layout"] == "single" and one["exhaust"]["cylinder_pipe"] == [0, 0, 0, 0]
    assert len(one["exhaust"]["pipes"]) == 1


def test_editing_the_file_changes_the_spec(tmp_path):
    p = edited(tmp_path, F454, header_length_m="[0.5, 0.5]", muffler_area_ratio="9.0", engine_inertia_kgm2="0.9",
               header_temp_k="900")
    sp = sound_spec(load_engine_file(p).engine)
    secs = sp["exhaust"]["pipes"][0]["sections"]
    assert secs[0]["length_m"] == 0.5 and secs[2]["area"] == 9.0
    assert math.isclose(secs[0]["c"], speed_of_sound(900)) and sp["inertia_kgm2"] == 0.9


def test_single_value_applies_to_every_pipe(tmp_path):
    p = edited(tmp_path, F454, mid_length_m="1.3")
    ecfg = load_engine_file(p).engine
    assert ecfg.exhaust_mid_length_m == [1.3, 1.3]


def test_odd_fire_intervals(tmp_path):
    p = edited(tmp_path, F1NZ, firing_order="[1, 3, 4, 2]")
    text = open(p).read().replace("[layout]", "[layout]\nfiring_intervals_deg = [270, 90, 270, 90]")
    open(p, "w").write(text)
    # order 1-3-4-2 fires cylinder 1 at 0°, 3 at 270°, 4 at 360°, 2 at 630°
    assert load_engine_file(p).engine.fire_angles_deg == [0.0, 630.0, 270.0, 360.0]


def _write(tmp_path, text):
    p = tmp_path / "bad.toml"
    p.write_text(text)
    return str(p)


def test_typo_gets_a_suggestion(tmp_path):
    text = open(F454).read().replace("header_length_m", "heder_length_m")
    with pytest.raises(EngineFileError, match=r"unknown key 'heder_length_m' in \[exhaust\]\. Did you mean 'header_length_m'"):
        load_engine_file(_write(tmp_path, text))


def test_unknown_section(tmp_path):
    with pytest.raises(EngineFileError, match="unknown section or key 'exhuast'.*Did you mean 'exhaust'"):
        load_engine_file(_write(tmp_path, open(F454).read() + "\n[exhuast]\nlayout = 'x'\n"))


def test_syntax_error_has_a_line_number(tmp_path):
    with pytest.raises(EngineFileError, match=r"not valid TOML.*line \d+"):
        load_engine_file(_write(tmp_path, open(F454).read() + "\nbore_m = = 3\n"))


@pytest.mark.parametrize("old,new,needle", [
    ("firing_order = [1, 8, 4, 3, 6, 5, 7, 2]", "firing_order = [1, 8, 4, 3, 6, 5, 7, 7]", "firing_order must use each cylinder"),
    ("layout = \"dual\"", "layout = \"triple\"", r"layout must be .single. or .dual."),
    ("num_cylinders = 8", "num_cylinders = 6", "cylinder_bank needs one entry per cylinder"),
    ("header_length_m = [0.85, 1.00]", "header_length_m = [0.85, 1.0, 1.2]", "header_length_m needs 1 value"),
    ("stall_rpm = 300", "stall_rpm = 900", "stall_rpm < idle_rpm < redline_rpm"),
    ("muffler_area_ratio = 6.0", "muffler_area_ratio = 0", "muffler_area_ratio must be greater than 0"),
    ("bore_m = 0.10795 ", "bore_m = \"big\" ", "'bore_m' must be a number"),
    ("use_idle_air_control = true", "use_idle_air_control = 1", "must be true or false"),
    ("tip_damping = 0.35 ", "tip_damping = 2.5 ", "tip_damping must be in"),
    ("exhaust_open_bbdc_deg = 50 ", "exhaust_open_bbdc_deg = 250 ", "between 0 and 90"),
])
def test_invalid_values_are_explained(tmp_path, old, new, needle):
    text = open(F454).read()
    assert old in text, old
    with pytest.raises(EngineFileError, match=needle):
        load_engine_file(_write(tmp_path, text.replace(old, new, 1)))


def test_dual_exhaust_needs_two_banks(tmp_path):
    text = open(F1NZ).read().replace('layout = "single"', 'layout = "dual"')
    with pytest.raises(EngineFileError, match="needs two cylinder banks"):
        load_engine_file(_write(tmp_path, text))


def test_broken_file_is_listed_with_its_error(tmp_path):
    (tmp_path / "ok.toml").write_text(open(F1NZ).read())
    (tmp_path / "broken.toml").write_text("bore_m = = 1")
    got = {e.id: e for e in list_engines(str(tmp_path))}
    assert got["1nz"].error is None and "not valid TOML" in got["broken"].error


def test_live_retune_keeps_the_engine_running():
    pt = Powertrain(*load_engine("454"))
    pt.controls.ignition = True
    pt.controls.starter = True
    for _ in range(1000):
        pt.step(0.001)
    pt.controls.starter = False
    for _ in range(3000):
        pt.step(0.001)
    rpm0 = pt.last.engine_rpm
    assert 550 < rpm0 < 900            # still on the fast-idle ramp a few seconds after the first fire
    new_e, new_v = load_engine("454")
    new_e.engine_inertia_kgm2 = 0.9
    new_e.exhaust_muffler_area_ratio = 9.0
    pt.apply_configs(new_e, new_v)
    for _ in range(1000):
        pt.step(0.001)
    assert pt.cfg.engine_inertia_kgm2 == 0.9
    assert abs(pt.last.engine_rpm - rpm0) < 100, "engine must keep idling through a live re-tune"


def _sim(tmp_path, base=F454):
    import interactive_dashboard as d
    p = tmp_path / "live.toml"
    p.write_text(open(base).read())
    return d.Simulation(str(p)), p


def test_dashboard_hot_reload(tmp_path):
    sim, p = _sim(tmp_path)
    v0 = sim.cfg_version
    assert sim.engine_info()["sound"]["exhaust"]["pipes"][0]["sections"][0]["length_m"] == 0.85
    # a good edit is applied and bumps the version
    p.write_text(p.read_text().replace("header_length_m = [0.85, 1.00]", "header_length_m = [0.60, 1.00]"))
    assert sim.reload() and sim.cfg_version == v0 + 1 and sim.cfg_error is None
    assert sim.engine_info()["sound"]["exhaust"]["pipes"][0]["sections"][0]["length_m"] == 0.60
    # a broken edit keeps the previous settings and reports the problem
    p.write_text(p.read_text().replace("header_length_m", "heder_length_m"))
    assert not sim.reload()
    assert sim.cfg_version == v0 + 1 and "heder_length_m" in sim.cfg_error
    assert sim.engine_info()["config_error"] == sim.cfg_error
    assert sim.engine_info()["sound"]["exhaust"]["pipes"][0]["sections"][0]["length_m"] == 0.60
    # fixing it recovers
    p.write_text(p.read_text().replace("heder_length_m", "header_length_m"))
    assert sim.reload() and sim.cfg_error is None


def test_file_watcher_applies_a_saved_edit(tmp_path):
    sim, p = _sim(tmp_path)
    v0 = sim.cfg_version
    time.sleep(0.6)
    p.write_text(p.read_text().replace("redline_rpm = 5500", "redline_rpm = 5800"))
    for _ in range(40):
        if sim.cfg_version > v0:
            break
        time.sleep(0.1)
    assert sim.cfg_version > v0, "watcher did not pick up the saved file"
    assert sim.pt.cfg.redline_rpm == 5800


def test_switching_engine_with_a_broken_target_reports_error(tmp_path):
    sim, p = _sim(tmp_path)
    (tmp_path / "bad.toml").write_text("oops = = 1")
    r = sim.apply({"action": "preset", "name": str(tmp_path / "bad.toml")})
    assert r["ok"] is False and "not valid TOML" in r["error"]
    assert sim.pt.cfg.num_cylinders == 8           # still the old engine
