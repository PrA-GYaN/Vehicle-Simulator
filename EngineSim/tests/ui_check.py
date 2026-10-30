"""Optional end-to-end check of the browser dashboard (not part of the pytest suite).

Starts the dashboard server, drives the real page in headless Chromium and checks:
  * every sound dial has a full hover tooltip; every static tooltip is well formed
  * knob drag / wheel / keyboard / double-click, presets, reset, persistence across reload
  * on-screen Start, gear buttons and throttle slider drive the simulation
  * turning the sound dials really changes the audio leaving the browser
    (band energy measured from the live AudioWorklet output; needs numpy)
  * editing the engine .toml file on disk while the dashboard runs: the page reloads it, the
    sound changes, a broken file shows the red banner without stopping the engine, fixing it recovers

Setup (once):   pip install playwright numpy && playwright install chromium
Run:            python tests/ui_check.py [--port 8791]
"""

import argparse
import asyncio
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time
import urllib.request

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
fails = []


def check(name, cond, extra=""):
    print(("PASS " if cond else "FAIL ") + name + (f"  [{extra}]" if extra else ""))
    if not cond:
        fails.append(name)


TAP = """
window.__rec = []; window.__on = false;
const a = S.audio, sp = a.ctx.createScriptProcessor(4096, 2, 2); a.node.connect(sp); sp.connect(a.ctx.destination);
sp.onaudioprocess = (e) => { if (window.__on) window.__rec.push(Array.from(e.inputBuffer.getChannelData(0))); };
"""


async def run(url, engine_file):
    from playwright.async_api import async_playwright

    async def val(pg, code):
        return await pg.evaluate("(c) => knobs[PROFILE.find(p => p.code === c).id].value", code)

    async def drag(pg, code, dy):
        box = await pg.locator(f".knob:has(.knob-code:text-is('{code}'))").bounding_box()
        x, y = box["x"] + box["width"] / 2, box["y"] + box["height"] / 2
        await pg.mouse.move(x, y); await pg.mouse.down()
        await pg.mouse.move(x, y + dy, steps=8); await pg.mouse.up()

    async def record(pg, secs):
        await pg.evaluate("window.__rec = []; window.__on = true")
        await pg.wait_for_timeout(int(secs * 1000))
        await pg.evaluate("window.__on = false")
        return await pg.evaluate("window.__rec.flat()")

    async with async_playwright() as p:
        b = await p.chromium.launch(args=["--autoplay-policy=no-user-gesture-required"])
        ctx = await b.new_context(viewport={"width": 1920, "height": 1080})
        pg = await ctx.new_page()
        errs = []
        pg.on("console", lambda m: errs.append(m.text) if m.type == "error" else None)
        pg.on("pageerror", lambda e: errs.append(str(e)))
        await pg.goto(url); await pg.wait_for_timeout(700)
        await pg.mouse.click(960, 540); await pg.wait_for_timeout(500)

        n = await pg.locator(".knob").count()
        check("14 sound dials built", n == 14, str(n))
        bad = []
        for i in range(n):
            k = pg.locator(".knob").nth(i)
            await k.hover(); await pg.wait_for_timeout(420)
            txt = await pg.evaluate("document.getElementById('tip').hidden ? '' : document.getElementById('tip').innerText")
            if not all(s in txt for s in ("Turn down", "Turn up", "Now:")) or len(txt) < 90:
                bad.append(await k.locator(".knob-code").inner_text())
            await pg.mouse.move(5, 5)
        check("every dial shows a full tooltip on hover", not bad, ",".join(bad))
        weak = await pg.evaluate("""[...document.querySelectorAll('[data-tip]')].map(e => e.getAttribute('data-tip'))
                                     .filter(t => !t.includes('::') || t.split('::')[1].length < 25)""")
        check("every static tooltip has a title and a real explanation", not weak, str(weak[:3]))

        k = pg.locator(".knob:has(.knob-code:text-is('+LF'))")
        v0 = await val(pg, "+LF"); await drag(pg, "+LF", -60); v1 = await val(pg, "+LF")
        check("drag up raises +LF", v1 > v0 + 1, f"{v0:.2f} -> {v1:.2f} dB")
        await drag(pg, "+LF", 400)
        check("drag clamps at -12 dB", abs(await val(pg, "+LF") + 12) < 1e-9)
        await k.dblclick(); check("double-click resets", await val(pg, "+LF") == 0)
        await k.hover(); await pg.mouse.wheel(0, -100)
        check("wheel over a dial changes it, not throttle/gear",
              await val(pg, "+LF") > 0 and await pg.evaluate("ctl.gear === 0 && cruise === 0"))
        await k.dblclick(); await k.focus()
        await pg.keyboard.press("ArrowUp"); await pg.keyboard.press("ArrowUp")
        check("arrows on a focused dial adjust it and do not shift gear",
              await val(pg, "+LF") > 0 and await pg.evaluate("ctl.gear") == 0)
        await k.dblclick(); await pg.evaluate("document.activeElement.blur()")
        await pg.keyboard.press("ArrowUp")
        check("arrow up shifts gear when no dial is focused", await pg.evaluate("ctl.gear") == 1)
        await pg.keyboard.press("n")

        await drag(pg, "RING", -80); ring = await val(pg, "RING"); await pg.wait_for_timeout(400)
        await pg.reload(); await pg.wait_for_timeout(800)
        check("settings persist across reload", abs(await val(pg, "RING") - ring) < 1e-6 and ring != 0.6)
        await pg.mouse.click(960, 540); await pg.wait_for_timeout(300)

        await pg.locator(".presets button", has_text="Muscle").click()
        on = await pg.evaluate("document.querySelector('.presets button.on')?.textContent")
        check("preset applies and is highlighted", abs(await val(pg, "+LF") - 5) < 1e-6 and on == "Muscle")
        await drag(pg, "+HF", -30)
        check("editing a dial clears the preset highlight",
              await pg.evaluate("document.querySelector('.presets button.on') === null"))
        await pg.locator("#mix-reset").click()
        check("Reset returns every dial to default",
              await pg.evaluate("PROFILE.every(p => Math.abs(knobs[p.id].value - p.def) < 1e-9)"))

        box = await pg.locator("#b-start").bounding_box()
        await pg.mouse.move(box["x"] + 20, box["y"] + 10); await pg.mouse.down()
        await pg.wait_for_timeout(1500); await pg.mouse.up(); await pg.wait_for_timeout(2500)
        rpm = await pg.evaluate("S.st.rpm")
        check("on-screen Start cranks and the engine idles",
              500 < rpm < 900 and await pg.inner_text("#state-pill") == "RUNNING", f"rpm={rpm:.0f}")
        await pg.locator("#gear-seg button", has_text="2").click()
        check("gear button selects gear 2", await pg.evaluate("ctl.gear") == 2)
        await pg.locator("#gear-seg button", has_text="N").click()
        await pg.evaluate("const t = document.getElementById('thr-slider'); t.value = 30; t.dispatchEvent(new Event('input'))")
        await pg.wait_for_timeout(1500)
        check("throttle slider revs the engine", await pg.evaluate("S.st.rpm") > 1000)
        await pg.evaluate("const t = document.getElementById('thr-slider'); t.value = 0; t.dispatchEvent(new Event('input'))")
        await pg.wait_for_timeout(2500)

        audio = {}
        try:
            import numpy as np
        except ImportError:
            np = None
            print("SKIP audio measurements (numpy not installed)")
        bands = [(20, 125), (125, 500), (500, 2000), (2000, 8000)]
        sr = 48000

        def spec(x):
            x = np.array(x[sr // 2:], dtype=float); m = 1 << int(np.log2(len(x)))
            X = np.abs(np.fft.rfft(x[-m:] * np.hanning(m))) ** 2; f = np.fft.rfftfreq(m, 1 / sr)
            return [10 * np.log10(X[(f >= a) & (f < c)].sum() + 1e-20) for a, c in bands], float(np.sqrt(np.mean(x ** 2)))
        if np is not None:
            await pg.evaluate(TAP); await pg.wait_for_timeout(300)
            for name, js in [("base", "0"), ("lf_up", "knobs.lf.set(12,true)"), ("lf_dn", "knobs.lf.set(-12,true)"),
                             ("hf_up", "knobs.hf.set(12,true)"), ("hf_dn", "knobs.hf.set(-12,true)"),
                             ("vol_0", "knobs.vol.set(0,true)"), ("vol_1", "knobs.vol.set(1,true)"),
                             ("ring_0", "knobs.ring.set(0,true)"), ("ring_1", "knobs.ring.set(1,true)"),
                             ("conv_1", "knobs.conv.set(1,true)"), ("drive_1", "knobs.drive.set(1,true)"),
                             ("tick_1", "knobs.tick.set(1,true)"), ("nlf_1", "knobs.nlf.set(1,true)"),
                             ("nhf_1", "knobs.nhf.set(1,true)")]:
                await pg.locator("#mix-reset").click(); await pg.wait_for_timeout(100)
                await pg.evaluate(js); await pg.wait_for_timeout(600)
                audio[name] = await record(pg, 1.5)
            sr = await pg.evaluate("S.audio.ctx.sampleRate")
            await pg.locator("#mix-reset").click()
        # ---- one engine file drives everything: edit it on disk while the dashboard runs
        async def wait_for(js, secs=6.0):
            for _ in range(int(secs / 0.1)):
                if await pg.evaluate(js):
                    return True
                await pg.wait_for_timeout(100)
            return False

        v0 = await pg.evaluate("S.eng.config_version")
        base_low = None
        if np is not None:
            await pg.locator("#mix-reset").click(); await pg.wait_for_timeout(300)
            before = await record(pg, 1.5)
        text = open(engine_file).read()
        open(engine_file, "w").write(text.replace("muffler_area_ratio = 6.0", "muffler_area_ratio = 14.0"))
        ok = await wait_for(f"S.eng.config_version > {v0} && S.eng.sound.exhaust.pipes[0].sections[2].area === 14")
        check("saving the engine file reloads it in the running dashboard", ok)
        check("the engine kept running through the reload", await pg.evaluate("S.st.rpm") > 500)
        if np is not None and ok:
            await pg.wait_for_timeout(600)
            after = await record(pg, 1.5)
            (b0, _), (b1, _) = spec(before), spec(after)
            check("a bigger muffler in the file makes the low end quieter", b1[0] < b0[0] - 2.0, f"{b0[0]:.1f} -> {b1[0]:.1f} dB")
        open(engine_file, "w").write(text + "\nbore_m = = 1\n")
        ok = await wait_for("!document.getElementById('cfg-error').hidden")
        banner = await pg.inner_text("#cfg-error")
        check("a broken engine file shows the red banner with the reason", ok and "not valid TOML" in banner, banner[:70])
        check("the engine keeps running the previous settings", await pg.evaluate("S.st.rpm") > 500)
        open(engine_file, "w").write(text.replace("header_length_m = [0.85, 1.00]", "header_length_m = [0.85, 1.00]  # fixed"))
        ok = await wait_for("document.getElementById('cfg-error').hidden")
        check("fixing the file clears the banner", ok)
        check("no console errors during the whole session", not errs, "; ".join(errs[:3]))
        await b.close()

    if audio:
        base, base_rms = spec(audio["base"])
        r = {k: spec(v) for k, v in audio.items() if k != "base"}
        d = lambda k, i: r[k][0][i] - base[i]
        check("+LF +12 dB raises the low band", d("lf_up", 0) > 6 and d("lf_up", 0) > d("lf_up", 3) + 6)
        check("+LF -12 dB cuts the low band", d("lf_dn", 0) < -6)
        check("+HF +12 dB raises the top band", d("hf_up", 3) > 4 and d("hf_up", 3) > d("hf_up", 0) + 4)
        check("+HF -12 dB cuts the top band", d("hf_dn", 3) < -4)
        check("VOL 0 is silent", r["vol_0"][1] < 1e-6)
        check("VOL 100% is louder than 70%", r["vol_1"][1] > base_rms * 1.2)
        check("RING changes the sound", max(abs(d("ring_0", i)) for i in range(4)) > 3)
        check("CONV adds room energy", max(abs(d("conv_1", i)) for i in range(4)) > 1.5)
        check("DRIVE adds level and harmonics", r["drive_1"][1] > base_rms * 1.3)
        check("TICK adds top-band energy", d("tick_1", 3) > 4)
        check("~LF adds low rumble", d("nlf_1", 0) > 3)
        check("~HF adds air", d("nhf_1", 3) > 3)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8791)
    args = ap.parse_args()
    try:
        import playwright  # noqa: F401
    except ImportError:
        sys.exit("playwright is not installed: pip install playwright numpy && playwright install chromium")
    tmp = tempfile.mkdtemp()                          # a private copy, so the test can edit it freely
    engine_file = os.path.join(tmp, "test_engine.toml")
    shutil.copy(os.path.join(ROOT, "config", "engines", "chevrolet_454.toml"), engine_file)
    srv = subprocess.Popen([sys.executable, os.path.join(ROOT, "interactive_dashboard.py"),
                            "--port", str(args.port), "--no-browser", "--engine", engine_file],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    url = f"http://127.0.0.1:{args.port}/"
    try:
        for _ in range(50):
            try:
                urllib.request.urlopen(url + "api/config", timeout=0.5); break
            except Exception:
                time.sleep(0.2)
        asyncio.run(run(url, engine_file))
    finally:
        srv.terminate()
        shutil.rmtree(tmp, ignore_errors=True)
    print(f"\n{len(fails)} failure(s)")
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
