'use strict';
// EngineSim dashboard. Telemetry (SSE) → instruments, engine animation, plots
// and the audio worklet; keyboard / on-screen controls → driver inputs (POST).
// Depends on ui.js (Tip, Knob, PROFILE, SOUND_PRESETS).

const $ = (id) => document.getElementById(id);
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
const r0 = (v) => { const n = Math.round(v); return String(n === 0 ? 0 : n); };   // never "-0"
const C = {
  text: '#e8ebf2', dim: '#8d96a8', faint: '#5d6576', line: '#262b35', track: '#262b35',
  red: '#ff5c5c', accent: '#ffb454', teal: '#4fd1c5', green: '#5fd38d', blue: '#6ea8ff',
  pink: '#f39ab8', yellow: '#f2c14e', panel: '#14171d',
};
C.orange = C.accent;   // plots / engine drawing use this name
const FONT = 'system-ui, -apple-system, "Segoe UI", Roboto, sans-serif';
const MONO = 'ui-monospace, "SF Mono", "JetBrains Mono", Menlo, Consolas, monospace';

// ------------------------------------------------------------------ state
const S = {
  eng: null,          // /api/config
  st: null,           // latest telemetry frame
  trace: null,
  theta: 0,           // client crank angle for animation [deg]
  lastFrameT: performance.now(),
  pktTimes: [],
  lastPkt: 0,
  fps: 60,
  hist: { rpm: [], map: [], tq: [], hp: [] },
  audio: { ctx: null, node: null, analyser: null, rms: 0, peak: 0, intake: 0, hiss: 0,
           muted: false, pops: true, buf: null, peakHold: 0, peakHoldT: 0 },
  profile: null,      // sound dials, id → value
  smooth: {},
  fireGlow: [],
  prevRpm: 0,
  dyno: { curve: [], run: null, wasOff: false, hoverX: null, tableKey: null },   // curve: measured {rpm, tq, hp} points; run: sweep in progress
};
const HIST_N = 600;

const ctl = {
  throttle: 0, ignition: false, starter: false, clutch_pedal: 0, gear: 0, brake: 0,
  auto_clutch: true, dyno_enabled: false, dyno_rpm: 3000,
};
let cruise = 0, pedal = 0, timeScale = 1;
const keys = new Set();

// ------------------------------------------------------------------ server I/O
let sending = false, dirty = false, lastSent = '';
function pushControls(extra) {
  const body = JSON.stringify(Object.assign({ controls: ctl }, extra || {}));
  if (!extra && body === lastSent) return;
  if (sending && !extra) { dirty = true; return; }
  sending = true; lastSent = body;
  fetch('/api/control', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body })
    .catch(() => {})
    .finally(() => { sending = false; if (dirty) { dirty = false; pushControls(); } });
}

async function loadConfig() {
  S.eng = await (await fetch('/api/config')).json();
  const e = S.eng, nb = Math.max(...e.bank) + 1;
  const sel = $('engine-select');
  sel.innerHTML = e.presets.map((p) => `<option value="${p.id}"${p.error ? ' disabled' : ''}>${p.name}${p.error ? ' (error in file)' : ''}</option>`).join('');
  sel.value = e.preset;
  setText($('cfg-file'), e.config_file.split(/[\\/]/).pop());
  S.cfgVer = e.config_version; showConfigError(e.config_error);
  $('engine-meta').textContent = `${e.displacement_l.toFixed(1)} L · ${Math.round(e.displacement_ci)} ci · ${nb > 1 ? 'V' : 'I'}${e.cylinders} · ${e.firing_order.join('-')}`;
  $('engine-sub').textContent = `${e.bore_mm.toFixed(0)} × ${e.stroke_mm.toFixed(0)} mm · idle ${Math.round(e.idle_rpm)} · redline ${Math.round(e.redline_rpm)} rpm`;
  buildGearButtons();
  const ds = $('dyno-slider'); ds.max = Math.round(e.redline_rpm / 250) * 250;
  if (S.loadedEngine !== e.preset) { ctl.dyno_rpm = Math.max(1000, Math.round(e.redline_rpm * 0.4 / 250) * 250); dynoDefaults(); }
  S.loadedEngine = e.preset;
  buildIgnition();
  S.fireGlow = new Array(e.cylinders).fill(0);
  if (S.audio.node) { S.audio.node.port.postMessage({ type: 'engine', engine: e }); sendProfile(); }
}

function connectStream() {
  const es = new EventSource('/api/stream');
  es.onmessage = (ev) => {
    const st = JSON.parse(ev.data);
    if (st.trace) S.trace = st.trace;
    S.st = st;
    const now = performance.now();
    S.lastPkt = now;
    S.pktTimes.push(now);
    while (S.pktTimes.length && now - S.pktTimes[0] > 1000) S.pktTimes.shift();
    pushHist(S.hist.rpm, st.rpm); pushHist(S.hist.map, st.map_kpa);
    pushHist(S.hist.tq, st.torque_lbft); pushHist(S.hist.hp, st.hp);
    feedAudio(st);
    events(st);
    dynoTick(st);
    if (st.cfg_ver !== S.cfgVer && !S.reloading) hotReload(st.cfg_ver);
    if (st.cfg_err !== S.cfgErr) showConfigError(st.cfg_err);
  };
  es.onerror = () => toast('Connection lost. Retrying…');
}
// The engine file was saved and re-applied by the server: fetch the new definition and rebuild
async function hotReload(ver) {
  S.reloading = true;
  try {
    await loadConfig();
    toast('Engine file reloaded');
  } finally { S.reloading = false; S.cfgVer = ver; }
}
function showConfigError(msg) {
  S.cfgErr = msg || null;
  const b = $('cfg-error');
  b.hidden = !msg;
  if (msg) b.textContent = `Engine file error. Still running the previous settings.  ${msg}`;
}
function pushHist(a, v) { a.push(v); if (a.length > HIST_N) a.shift(); }

// ------------------------------------------------------------------ audio
async function initAudio() {
  if (S.audio.ctx) { S.audio.ctx.resume(); return; }
  // 'playback' = larger output buffer: no dropouts/crackle, ~40-80 ms extra latency
  const actx = new AudioContext({ latencyHint: 'playback' });
  S.audio.ctx = actx;
  await actx.audioWorklet.addModule('engine-audio.js');
  const node = new AudioWorkletNode(actx, 'engine-synth', { outputChannelCount: [2] });
  const an = actx.createAnalyser();
  an.fftSize = 16384;
  node.connect(an); an.connect(actx.destination);
  node.port.onmessage = (e) => {
    const m = e.data;
    if (m.type === 'level') Object.assign(S.audio, { rms: m.rms, peak: m.peak, intake: m.intake, hiss: m.hiss });
  };
  S.audio.node = node; S.audio.analyser = an;
  S.audio.buf = new Float32Array(an.fftSize);
  if (S.eng) node.port.postMessage({ type: 'engine', engine: S.eng });
  sendProfile();
}

function feedAudio(st) {
  const n = S.audio.node; if (!n) return;
  n.port.postMessage({ type: 'state', state: {
    rpm: st.rpm, map: st.map_ratio, throttle: st.throttle, combusting: st.combusting,
    fuelCut: st.fuel_cut, starter: st.starter, ignition: st.ignition, speed: st.speed_mph,
    gear: st.gear, load: clamp(st.clutch_nm / 300, -1, 1),
    indicated: st.indicated_nm, friction: st.friction_nm, pumping: st.pumping_nm,
    redline: S.eng ? S.eng.redline_rpm : 6000, idle: S.eng ? S.eng.idle_rpm : 800,
    muted: S.audio.muted, pops: S.audio.pops,
  }});
}

// ------------------------------------------------------------------ sound profile (mixer)
const PROFILE_KEY = 'enginesim.profile.v1';
const knobs = {};
let saveTimer = 0;

function loadProfile() {
  const p = profileDefaults();
  try {
    const o = JSON.parse(localStorage.getItem(PROFILE_KEY) || 'null');
    if (o) for (const s of PROFILE) if (typeof o[s.id] === 'number' && isFinite(o[s.id])) p[s.id] = clamp(o[s.id], s.min, s.max);
  } catch (e) { /* storage unavailable: defaults */ }
  return p;
}
function saveProfile() {
  clearTimeout(saveTimer);
  saveTimer = setTimeout(() => { try { localStorage.setItem(PROFILE_KEY, JSON.stringify(S.profile)); } catch (e) { /* ignore */ } }, 200);
}
function sendProfile() {
  if (S.audio.node) S.audio.node.port.postMessage({ type: 'profile', profile: S.profile });
}
function onKnob(id, v) {
  S.profile[id] = v; sendProfile(); saveProfile(); markPreset();
}
function applyPreset(preset) {
  for (const s of PROFILE) {
    const v = s.id in preset.v ? preset.v[s.id] : s.def;
    S.profile[s.id] = v; knobs[s.id].setSilent(v);
  }
  sendProfile(); saveProfile(); markPreset();
  toast(`Sound: ${preset.name}`);
}
function markPreset() {
  // volume is a personal preference, so it does not count when matching a preset
  let match = null;
  for (const pr of SOUND_PRESETS) {
    let same = true;
    for (const s of PROFILE) {
      if (s.id === 'vol') continue;
      const v = s.id in pr.v ? pr.v[s.id] : s.def;
      if (Math.abs(v - S.profile[s.id]) > (s.max - s.min) * 0.006) { same = false; break; }
    }
    if (same) { match = pr.id; break; }
  }
  for (const b of $('presets').children) b.classList.toggle('on', b.dataset.id === match);
}
function nudge(id, d) { const k = knobs[id]; k.set(k.value + d, true); }

function buildMixer() {
  S.profile = loadProfile();
  for (const s of PROFILE) {
    const k = new Knob(s, S.profile[s.id], onKnob);
    knobs[s.id] = k;
    $(`knobs-${s.group}`).appendChild(k.root);
  }
  const box = $('presets');
  for (const pr of SOUND_PRESETS) {
    const b = document.createElement('button');
    b.textContent = pr.name; b.dataset.id = pr.id;
    Tip.bind(b, () => ({ title: `Preset: ${pr.name}`, body: pr.desc, foot: 'Sets every dial. Your own tweaks are remembered.' }));
    b.addEventListener('click', () => { applyPreset(pr); b.blur(); });
    box.appendChild(b);
  }
  $('mix-reset').addEventListener('click', (e) => { applyPreset(SOUND_PRESETS[0]); e.currentTarget.blur(); });
  markPreset();
}

// ------------------------------------------------------------------ input
const HOLD_THROTTLE = { KeyQ: 0.10, KeyW: 0.25, KeyE: 0.50, KeyR: 1.00 };
const TOGGLE_KEYS = new Set(['KeyA', 'KeyD', 'KeyC', 'KeyP', 'KeyM', 'KeyV', 'KeyH', 'KeyN',
  'ArrowUp', 'ArrowDown', 'Backspace']);

function onKeyDown(e) {
  if (e.ctrlKey || e.metaKey || e.altKey) return;
  if (!$('dyno-plot-modal').hidden) {   // detailed plot view: only Esc does anything
    if (e.code === 'Escape') { e.preventDefault(); closeDynoPlot(); }
    return;
  }
  if (!$('dyno-modal').hidden) {       // the dyno setup window owns the keyboard (typing numbers must not drive the car)
    if (e.code === 'Escape') { e.preventDefault(); dynoModalCancel(); }
    return;
  }
  const t = e.target;
  // native controls keep their own arrow / home / end behaviour
  if (t && t.matches && t.matches('input[type=range],select') &&
      ['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', 'Home', 'End', 'PageUp', 'PageDown'].includes(e.code)) return;
  // KeyD opens a form that takes focus: stop the same keypress from being typed into it
  if (['Space', 'ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', 'Backspace', 'KeyD'].includes(e.code)) e.preventDefault();
  gateOff();
  if (e.repeat && TOGGLE_KEYS.has(e.code)) return;
  keys.add(e.code);
  const gears = S.eng ? S.eng.gears : 4;
  switch (e.code) {
    case 'KeyA': ctl.ignition = !ctl.ignition; toast(ctl.ignition ? 'Ignition on' : 'Ignition off'); break;
    case 'KeyS': ctl.ignition = true; ctl.starter = true; break;
    case 'KeyD': toggleDyno(); break;
    case 'ArrowLeft': if (!S.dyno.run) ctl.dyno_rpm = Math.max(500, ctl.dyno_rpm - 250); break;
    case 'ArrowRight': if (!S.dyno.run) ctl.dyno_rpm = Math.min(+$('dyno-slider').max, ctl.dyno_rpm + 250); break;
    case 'ArrowUp': ctl.gear = Math.min(gears, ctl.gear + 1); toast(`Gear ${gearName(ctl.gear)}`); break;
    case 'ArrowDown': ctl.gear = Math.max(0, ctl.gear - 1); toast(`Gear ${gearName(ctl.gear)}`); break;
    case 'KeyN': case 'Digit0': ctl.gear = 0; toast('Neutral'); break;
    case 'ShiftLeft': case 'ShiftRight': ctl.clutch_pedal = 1; break;
    case 'KeyC': toggleAuto(); break;
    case 'KeyB': ctl.brake = 1; break;
    case 'KeyZ': cruise = clamp(+(cruise - 0.02).toFixed(2), 0, 1); break;
    case 'KeyX': cruise = clamp(+(cruise + 0.02).toFixed(2), 0, 1); break;
    case 'KeyP': S.audio.pops = !S.audio.pops; toast(S.audio.pops ? 'Overrun pops on' : 'Overrun pops off'); break;
    case 'KeyM': toggleMute(); break;
    case 'Minus': case 'NumpadSubtract': nudge('vol', -0.05); break;
    case 'Equal': case 'NumpadAdd': nudge('vol', 0.05); break;
    case 'Comma': setTimeScale(timeScale / 2); break;
    case 'Period': setTimeScale(timeScale * 2); break;
    case 'KeyV': cycleEngine(); break;
    case 'Backspace': resetSim(); break;
    case 'KeyH': case 'Slash': $('help').hidden = !$('help').hidden; break;
    case 'Escape': $('help').hidden = true; break;
    default:
      if (/^Digit[1-9]$/.test(e.code)) {
        const g = +e.code.slice(5);
        if (g <= gears) { ctl.gear = g; toast(`Gear ${g}`); }
      }
  }
  pushControls();
}

function onKeyUp(e) {
  keys.delete(e.code);
  switch (e.code) {
    case 'KeyS': ctl.starter = false; break;
    case 'ShiftLeft': case 'ShiftRight':
      if (!keys.has('ShiftLeft') && !keys.has('ShiftRight')) ctl.clutch_pedal = 0; break;
    case 'KeyB': ctl.brake = 0; break;
  }
  pushControls();
}

function updateThrottle(dt) {
  let held = 0;
  for (const k in HOLD_THROTTLE) if (keys.has(k)) held = Math.max(held, HOLD_THROTTLE[k]);
  pedal = keys.has('Space') ? Math.min(1, pedal + dt * 1.2) : Math.max(0, pedal - dt * 5);
  ctl.throttle = S.dyno.run ? S.dyno.run.thr : Math.max(held, pedal, cruise);
  pushControls();
}

function toggleDyno() {
  if (ctl.dyno_enabled) { abortDyno(); ctl.dyno_enabled = false; closeDynoModal(); toast('Dyno off'); return; }
  ctl.dyno_enabled = true;
  if (S.st) ctl.dyno_rpm = Math.max(1000, Math.round(S.st.rpm / 250) * 250);
  openDynoModal(true);
}

// ------------------------------------------------------------------ dyno test (modal + sweep)
const dm = (id) => $(`dm-${id}`);
function dynoDefaults() {
  const e = S.eng, top = Math.floor(e.redline_rpm / 250) * 250;
  for (const id of ['start', 'end']) dm(id).max = String(top);
  dm('start').value = Math.max(1000, Math.round(e.idle_rpm * 1.3 / 250) * 250);
  dm('end').value = Math.floor(top * 0.95 / 250) * 250;   // stay off the rev limiter, which would cut fuel and dip the curve
  dm('step').value = top > 4000 ? 500 : 250;
}
function dynoSetup() {       // read + validate the form; returns {rpms, thr, settle, meas} or {error}
  const num = (id) => parseFloat(dm(id).value);
  const thr = num('thr'), a = num('start'), b = num('end'), step = num('step'), settle = num('settle'), meas = num('meas');
  const top = S.eng ? S.eng.redline_rpm : 7000;
  if (![thr, a, b, step, settle, meas].every(isFinite)) return { error: 'Fill in every field with a number.' };
  if (thr < 1 || thr > 100) return { error: 'Throttle must be between 1 and 100 %.' };
  if (a < 500 || b > top) return { error: `Speeds must be between 500 and ${Math.round(top)} rpm (this engine's redline).` };
  if (b < a) return { error: 'End speed must not be below the start speed.' };
  if (step < 50) return { error: 'Step must be at least 50 rpm.' };
  if (settle < 0.2 || meas < 0.1) return { error: 'Settle must be at least 0.2 s and measure at least 0.1 s.' };
  const rpms = [];
  for (let r = a; r <= b + 1e-6 && rpms.length < 80; r += step) rpms.push(Math.round(r));
  if (rpms.length < 2 && b > a) rpms.push(Math.round(b));
  if (rpms[rpms.length - 1] !== Math.round(b) && b - rpms[rpms.length - 1] > step * 0.25 && rpms.length < 80) rpms.push(Math.round(b));
  return { rpms, thr: thr / 100, settle, meas };
}
function dynoInfo() {
  const o = dynoSetup(), err = $('dm-err'), info = $('dm-info');
  err.hidden = !o.error; err.textContent = o.error || '';
  info.textContent = o.error ? '' : `${o.rpms.length} test point${o.rpms.length === 1 ? '' : 's'} · about ${Math.round(o.rpms.length * (o.settle + o.meas))} s of simulated time`;
  $('dm-run').disabled = !!o.error;
}
function openDynoModal(fromOff) {
  S.dyno.wasOff = !!fromOff;
  gateOff();
  $('dyno-modal').hidden = false;
  dynoInfo();
  dm('thr').focus(); dm('thr').select();
}
function closeDynoModal() { $('dyno-modal').hidden = true; }
function dynoModalCancel() {
  closeDynoModal();
  if (S.dyno.wasOff && !S.dyno.run) { ctl.dyno_enabled = false; toast('Dyno off'); }
  pushControls();
}
function dynoHold() {
  const o = dynoSetup(); if (o.error) return dynoInfo();
  closeDynoModal();
  cruise = o.thr; ctl.ignition = true; ctl.dyno_enabled = true; ctl.dyno_rpm = o.rpms[0];
  toast(`Dyno holding ${ctl.dyno_rpm} rpm at ${Math.round(o.thr * 100)} % throttle`);
  pushControls();
}
function dynoStart() {
  const o = dynoSetup(); if (o.error) return dynoInfo();
  closeDynoModal();
  S.dyno.curve = [];
  S.dyno.run = { rpms: o.rpms, i: 0, phase: 'settle', t0: null, thr: o.thr, settle: o.settle, meas: o.meas,
                 acc: null, prevCruise: cruise };
  cruise = o.thr; pedal = 0;
  Object.assign(ctl, { ignition: true, starter: false, gear: 0, clutch_pedal: 0, brake: 0, dyno_enabled: true, dyno_rpm: o.rpms[0], throttle: o.thr });
  toast(`Dyno sweep: ${o.rpms.length} points at ${Math.round(o.thr * 100)} % throttle`);
  pushControls();
}
function abortDyno(msg) {
  const r = S.dyno.run; if (!r) return;
  S.dyno.run = null; cruise = r.prevCruise; ctl.dyno_enabled = false;
  if (msg) toast(msg);
  pushControls();
}
function dynoTick(st) {     // advance the sweep from telemetry; sim time (st.t) keeps it correct in slow motion
  const r = S.dyno.run; if (!r) return;
  if (r.t0 === null) r.t0 = st.t;
  const target = r.rpms[r.i], el = st.t - r.t0;
  if (r.phase === 'settle') {
    const near = Math.abs(st.rpm - target) <= Math.max(25, target * 0.01);
    if ((el >= r.settle && near) || el >= Math.max(r.settle * 4, r.settle + 3)) { r.phase = 'measure'; r.t0 = st.t; r.acc = { n: 0, rpm: 0, tq: 0, hp: 0 }; }
    return;
  }
  const a = r.acc; a.n++; a.rpm += st.rpm; a.tq += st.torque_lbft; a.hp += st.hp;
  if (el < r.meas) return;
  S.dyno.curve.push({ rpm: a.rpm / a.n, tq: a.tq / a.n, hp: a.hp / a.n });
  if (++r.i < r.rpms.length) { r.phase = 'settle'; r.t0 = st.t; ctl.dyno_rpm = r.rpms[r.i]; pushControls(); return; }
  const c = S.dyno.curve, pk = (k) => c.reduce((m, p) => (p[k] > m[k] ? p : m), c[0]);
  const t = pk('tq'), h = pk('hp');
  abortDyno(`Sweep done: peak ${r0(t.tq)} lb-ft @ ${r0(t.rpm)} rpm · ${r0(h.hp)} hp @ ${r0(h.rpm)} rpm`);
}
function toggleAuto() {
  ctl.auto_clutch = !ctl.auto_clutch;
  toast(ctl.auto_clutch ? 'Auto clutch on' : 'Auto clutch off: hold Shift for the clutch');
}
function toggleMute() { S.audio.muted = !S.audio.muted; toast(S.audio.muted ? 'Sound muted' : 'Sound on'); }

function setTimeScale(v) {
  timeScale = clamp(v, 1 / 32, 1);
  pushControls({ time_scale: timeScale });
  toast(timeScale === 1 ? 'Real time' : `Slow motion 1/${Math.round(1 / timeScale)}`);
}

async function setEngine(id) {
  resetControls();
  await fetch('/api/control', { method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ action: 'preset', name: id, controls: ctl }) });
  await loadConfig();
  S.hist = { rpm: [], map: [], tq: [], hp: [] };
  S.dyno.curve = [];
  toast(S.eng.name);
}
function cycleEngine() {
  const list = S.eng.presets, i = list.findIndex((p) => p.id === S.eng.preset);
  setEngine(list[(i + 1) % list.length].id);
}

function resetControls() {
  S.dyno.run = null; closeDynoModal();
  Object.assign(ctl, { throttle: 0, ignition: false, starter: false, clutch_pedal: 0, gear: 0, brake: 0, dyno_enabled: false });
  cruise = 0; pedal = 0;
}
function resetSim() {
  resetControls();
  pushControls({ action: 'reset' });
  S.hist = { rpm: [], map: [], tq: [], hp: [] };
  toast('Simulation reset');
}
function gearName(g) { return g === 0 ? 'N' : String(g); }

let gated = true;
function gateOff() {
  if (!gated) return;
  gated = false;
  $('audio-gate').hidden = true;
  initAudio().catch((err) => { console.error(err); toast('Audio unavailable'); });
}

// ------------------------------------------------------------------ toasts / events
let toastTimer = 0;
function toast(msg) {
  const t = $('toast'); t.textContent = msg; t.classList.add('show');
  clearTimeout(toastTimer); toastTimer = setTimeout(() => t.classList.remove('show'), 1700);
}
function events(st) {
  if (st.ignition && !st.starter && S.prevRpm > 150 && st.rpm < 60) toast('Engine stalled');
  S.prevRpm = st.rpm;
}

// ------------------------------------------------------------------ on-screen controls
function holdButton(el, down, up) {
  el.addEventListener('pointerdown', (e) => {
    if (e.button !== 0) return;
    e.preventDefault(); gateOff(); el.setPointerCapture(e.pointerId);
    down(); pushControls();
  });
  const end = () => { up(); pushControls(); };
  el.addEventListener('pointerup', end);
  el.addEventListener('pointercancel', end);
}
function setFill(input) {
  const p = (input.value - input.min) / (input.max - input.min) * 100;
  input.style.setProperty('--fill', `${p}%`);
}
function buildGearButtons() {
  const seg = $('gear-seg'), n = S.eng.gears;
  seg.innerHTML = ['N'].concat(Array.from({ length: n }, (_, i) => String(i + 1)))
    .map((g, i) => `<button data-g="${i}">${g}</button>`).join('');
  seg.querySelectorAll('button').forEach((b) => b.addEventListener('click', () => {
    gateOff(); ctl.gear = +b.dataset.g; pushControls(); b.blur();
  }));
  S.gearEls = [...seg.children];
}
function wireControls() {
  $('b-ign').addEventListener('click', (e) => { gateOff(); ctl.ignition = !ctl.ignition; pushControls(); e.currentTarget.blur(); });
  $('b-auto').addEventListener('click', (e) => { toggleAuto(); pushControls(); e.currentTarget.blur(); });
  $('b-dyno').addEventListener('click', (e) => { toggleDyno(); pushControls(); e.currentTarget.blur(); });
  $('b-dyno-test').addEventListener('click', (e) => { openDynoModal(!ctl.dyno_enabled); if (!ctl.dyno_enabled) { ctl.dyno_enabled = true; pushControls(); } e.currentTarget.blur(); });
  $('b-dyno-abort').addEventListener('click', (e) => { abortDyno('Sweep stopped'); e.currentTarget.blur(); });
  P('dyno').parentNode.addEventListener('click', openDynoPlot);
  $('dp-close').addEventListener('click', closeDynoPlot);
  $('dyno-plot-modal').addEventListener('pointerdown', (e) => { if (e.target === $('dyno-plot-modal')) closeDynoPlot(); });
  const big = $('dyno-big');
  big.addEventListener('pointermove', (e) => { S.dyno.hoverX = e.clientX - big.getBoundingClientRect().left; });
  big.addEventListener('pointerleave', () => { S.dyno.hoverX = null; });
  $('dm-form').addEventListener('input', dynoInfo);
  $('dm-form').addEventListener('submit', (e) => { e.preventDefault(); dynoStart(); });
  $('dm-cancel').addEventListener('click', dynoModalCancel);
  $('dm-hold').addEventListener('click', dynoHold);
  holdButton($('b-start'), () => { ctl.ignition = true; ctl.starter = true; }, () => { ctl.starter = false; });
  holdButton($('b-clutch'), () => { ctl.clutch_pedal = 1; }, () => { ctl.clutch_pedal = 0; });
  holdButton($('b-brake'), () => { ctl.brake = 1; }, () => { ctl.brake = 0; });

  const thr = $('thr-slider'), dyn = $('dyno-slider');
  thr.addEventListener('input', () => { gateOff(); cruise = thr.value / 100; setFill(thr); });
  thr.addEventListener('wheel', (e) => { cruise = clamp(+(cruise - Math.sign(e.deltaY) * 0.02).toFixed(2), 0, 1); e.preventDefault(); }, { passive: false });
  dyn.addEventListener('input', () => { ctl.dyno_rpm = +dyn.value; setFill(dyn); pushControls(); });
  for (const el of [thr, dyn]) el.addEventListener('pointerup', () => el.blur());

  $('engine-select').addEventListener('change', (e) => { gateOff(); setEngine(e.target.value); e.target.blur(); });
  $('btn-reload').addEventListener('click', (e) => {
    gateOff(); pushControls({ action: 'reload' }); toast('Reloading engine file…'); e.currentTarget.blur();
  });
  $('btn-mute').addEventListener('click', (e) => { gateOff(); toggleMute(); e.currentTarget.blur(); });
  $('btn-help').addEventListener('click', (e) => { $('help').hidden = false; e.currentTarget.blur(); });

  const seg = $('speed-seg');
  for (const [label, v] of [['1×', 1], ['½', 0.5], ['¼', 0.25], ['⅛', 0.125]]) {
    const b = document.createElement('button'); b.textContent = label; b.dataset.v = v;
    b.addEventListener('click', () => { setTimeScale(v); b.blur(); });
    seg.appendChild(b);
  }
}

// Keep the on-screen controls in step with the real state (keys, pointer, resets)
const cls = (el, name, on) => { if (el.__c !== on) { el.__c = on; el.classList.toggle(name, on); } };
function syncControls() {
  cls($('b-ign'), 'on', ctl.ignition); cls($('b-auto'), 'on', ctl.auto_clutch); cls($('b-dyno'), 'on', ctl.dyno_enabled);
  cls($('b-start'), 'pressed', ctl.starter); cls($('b-clutch'), 'pressed', ctl.clutch_pedal > 0.5); cls($('b-brake'), 'pressed', ctl.brake > 0.5);
  if (S.gearEls) S.gearEls.forEach((b, i) => cls(b, 'on', i === ctl.gear));
  const thr = $('thr-slider'), dyn = $('dyno-slider');
  const tv = Math.round(cruise * 100);
  if (+thr.value !== tv) { thr.value = tv; setFill(thr); }
  if (+dyn.value !== ctl.dyno_rpm) { dyn.value = ctl.dyno_rpm; setFill(dyn); }
  const run = S.dyno.run, dynoOff = !ctl.dyno_enabled || !!run;
  if (dyn.disabled !== dynoOff) dyn.disabled = dynoOff;
  $('b-dyno-abort').hidden = !run; $('b-dyno-test').hidden = !!run;
  setText($('thr-readout'), `set ${tv} %  ·  actual ${Math.round(ctl.throttle * 100)} %`);
  setText($('dyno-readout'), run ? `sweep ${Math.min(run.i + 1, run.rpms.length)}/${run.rpms.length} · ${ctl.dyno_rpm} rpm` : `${ctl.dyno_rpm} rpm`);
  cls($('btn-mute'), 'muted', S.audio.muted);
  for (const b of $('speed-seg').children) cls(b, 'on', Math.abs(+b.dataset.v - timeScale) < 1e-6);
}

// ------------------------------------------------------------------ canvas helpers
// Canvas sizes are cached and updated by a ResizeObserver, so the render loop
// never reads layout (no forced reflow per frame).
const sizes = new Map();
const resizeObs = new ResizeObserver((entries) => {
  for (const e of entries) setSize(e.target, e.contentRect.width, e.contentRect.height);
});
function setSize(cv, w, h) {
  const dpr = window.devicePixelRatio || 1, prev = sizes.get(cv);
  cv.width = Math.max(1, Math.round(w * dpr)); cv.height = Math.max(1, Math.round(h * dpr));
  sizes.set(cv, { ctx: prev ? prev.ctx : cv.getContext('2d'), w, h, dpr, ver: prev ? prev.ver + 1 : 1 });
}
function fit(cv, clear = true) {
  let sz = sizes.get(cv);
  if (!sz) {
    const r = cv.getBoundingClientRect();
    setSize(cv, r.width, r.height); sz = sizes.get(cv); resizeObs.observe(cv);
  }
  sz.ctx.setTransform(sz.dpr, 0, 0, sz.dpr, 0, 0);
  if (clear) sz.ctx.clearRect(0, 0, sz.w, sz.h);
  return sz;
}

// DOM writes only when the value actually changes
function setText(el, v) { if (el.__v !== v) { el.__v = v; el.textContent = v; } }

function smooth(key, v, k = 0.25) {
  const p = S.smooth[key];
  const n = (p === undefined || !isFinite(p)) ? v : p + (v - p) * k;
  S.smooth[key] = n; return n;
}

// Radial dial: 270° sweep, coloured zones on a track, tick ring with numbers,
// needle, value + unit under the hub. The static face is rendered once per
// size into an offscreen canvas; a frame only blits it and draws needle +
// text, and is skipped when neither changed visibly.
const gaugeCache = new WeakMap();
function gaugeGeom(w, h) {
  const pad = 4;
  const R = Math.max(14, Math.min(w / 2 - pad, (h - 2 * pad) / 1.9));
  return { R, cx: w / 2, cy: pad + (h - 2 * pad - 1.9 * R) / 2 + R, a0: Math.PI * 0.75, sweep: Math.PI * 1.5 };
}
const fmtNum = (v) => (Number.isInteger(v) ? String(v) : v.toFixed(1));
function gaugeFace(sz, o, key) {
  const { w, h, dpr } = sz;
  const bg = document.createElement('canvas');
  bg.width = Math.max(1, Math.round(w * dpr)); bg.height = Math.max(1, Math.round(h * dpr));
  const ctx = bg.getContext('2d'); ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  const { R, cx, cy, a0, sweep } = gaugeGeom(w, h);
  const t = (v) => a0 + sweep * clamp((v - o.min) / (o.max - o.min), 0, 1);
  const lw = R * 0.085;
  ctx.lineCap = 'butt';
  ctx.strokeStyle = C.track; ctx.lineWidth = lw;
  ctx.beginPath(); ctx.arc(cx, cy, R * 0.9, a0, a0 + sweep); ctx.stroke();
  for (const [from, to, col] of (o.bands || [])) {
    ctx.strokeStyle = col; ctx.globalAlpha = 0.9;
    ctx.beginPath(); ctx.arc(cx, cy, R * 0.9, t(from), t(to)); ctx.stroke();
  }
  ctx.globalAlpha = 1;
  const major = o.major || 10, minor = o.minor || 2, n = major * minor;
  for (const isMaj of [false, true]) {          // two batched paths instead of n strokes
    ctx.strokeStyle = isMaj ? C.text : C.faint; ctx.lineWidth = isMaj ? Math.max(1.3, R * 0.025) : 1;
    ctx.beginPath();
    for (let i = 0; i <= n; i++) {
      if ((i % minor === 0) !== isMaj) continue;
      const a = a0 + sweep * i / n;
      const r1 = R * (isMaj ? 0.7 : 0.76), r2 = R * 0.82;
      ctx.moveTo(cx + Math.cos(a) * r1, cy + Math.sin(a) * r1);
      ctx.lineTo(cx + Math.cos(a) * r2, cy + Math.sin(a) * r2);
    }
    ctx.stroke();
  }
  // numbers: thin them out (to a divisor of the major count) so neighbours never overlap
  const fs = Math.max(8, R * 0.15), fmt = o.label || fmtNum;
  const texts = Array.from({ length: major + 1 }, (_, i) => fmt(o.min + (o.max - o.min) * i / major));
  const textW = Math.max(...texts.map((t) => t.length)) * fs * 0.62 + 6;
  let every = Math.max(1, Math.ceil(textW / (R * 0.58 * sweep / major)));
  if (R < 42) every = major;                       // tiny dial: just the two end values
  while (major % every !== 0 && every < major) every++;
  ctx.fillStyle = C.dim; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
  ctx.font = `600 ${fs}px ${MONO}`;
  for (let i = 0; i <= major; i += every) {
    const a = a0 + sweep * i / major;
    ctx.fillText(texts[i], cx + Math.cos(a) * R * 0.58, cy + Math.sin(a) * R * 0.58);
  }
  return { bg, key, ver: sz.ver, sig: null };
}
function gauge(cv, o) {
  const sz = fit(cv, false);
  const key = `${o.min}|${o.max}|${o.major}|${o.minor}|${o.bands ? o.bands.join(';') : ''}`;
  let gc = gaugeCache.get(cv);
  if (!gc || gc.ver !== sz.ver || gc.key !== key) { gc = gaugeFace(sz, o, key); gaugeCache.set(cv, gc); }
  const frac = clamp((o.value - o.min) / (o.max - o.min), -0.02, 1.02);
  const sig = `${Math.round(frac * 1500)}|${o.text}|${o.unit || ''}`;
  if (gc.sig === sig) return;                   // nothing visible changed
  gc.sig = sig;
  const { ctx, w, h, dpr } = sz;
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  ctx.clearRect(0, 0, cv.width, cv.height);
  ctx.drawImage(gc.bg, 0, 0);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  const { R, cx, cy, a0, sweep } = gaugeGeom(w, h);
  const a = a0 + sweep * frac;
  ctx.lineCap = 'round';
  ctx.strokeStyle = C.accent; ctx.lineWidth = Math.max(2, R * 0.045);
  ctx.beginPath(); ctx.moveTo(cx - Math.cos(a) * R * 0.1, cy - Math.sin(a) * R * 0.1);
  ctx.lineTo(cx + Math.cos(a) * R * 0.84, cy + Math.sin(a) * R * 0.84); ctx.stroke();
  ctx.fillStyle = C.panel; ctx.strokeStyle = C.accent; ctx.lineWidth = Math.max(1.5, R * 0.03);
  ctx.beginPath(); ctx.arc(cx, cy, R * 0.09, 0, 7); ctx.fill(); ctx.stroke();
  ctx.fillStyle = C.text; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
  ctx.font = `700 ${Math.max(11, R * 0.25)}px ${MONO}`;
  ctx.fillText(o.text, cx, cy + R * 0.66);
  if (o.unit) {
    ctx.fillStyle = C.dim; ctx.font = `700 ${Math.max(8, R * 0.13)}px ${FONT}`;
    ctx.fillText(o.unit, cx, cy + R * 0.66 + Math.max(11, R * 0.25) * 0.85);
  }
}

const elCache = new Map();
function G(name) { let e = elCache.get('g' + name); if (!e) { e = document.querySelector(`canvas[data-g="${name}"]`); elCache.set('g' + name, e); } return e; }
function P(name) { let e = elCache.get('p' + name); if (!e) { e = document.querySelector(`canvas[data-plot="${name}"]`); elCache.set('p' + name, e); } return e; }

// ------------------------------------------------------------------ engine geometry
function cycleOf(c) {            // cylinder-local cycle angle: 360 = firing TDC
  return (((S.theta - S.eng.fire_angle_deg[c]) % 720) + 720 + 360) % 720;
}
function lift(alpha, open, dur) {
  const x = ((alpha - open) % 720 + 720) % 720;
  return x < dur ? Math.sin(Math.PI * x / dur) ** 2 : 0;
}
// valve windows come from the engine file: [valves] intake_/exhaust_ open/close angles.
// When the server sends the cam tables (the exact lift curves the physics integrates, ramp
// shape included) they are used; the sin² hump is the fallback.
function tableLift(tab, step, a) {
  const n = tab.length - 1, x = (((a % 720) + 720) % 720) / step, i = Math.floor(x), f = x - i;
  return tab[i % n] * (1 - f) + tab[(i + 1) % n] * f;
}
// Variable valve timing: the cam is advanced by st.vvt_in / st.vvt_ex crank degrees, which
// moves the whole lift curve (the server reports the current advance every frame).
const camIn = () => (S.st && S.st.vvt_in) || 0, camEx = () => (S.st && S.st.vvt_ex) || 0;
const intakeLift = (a) => S.eng.valve_lift
  ? tableLift(S.eng.valve_lift.intake, S.eng.valve_lift.step_deg, a + camIn())
  : lift(a + camIn(), S.eng.valves.intake_open, S.eng.valves.intake_dur);
const exhaustLift = (a) => S.eng.valve_lift
  ? tableLift(S.eng.valve_lift.exhaust, S.eng.valve_lift.step_deg, a + camEx())
  : lift(a + camEx(), S.eng.valves.exhaust_open, S.eng.valves.exhaust_dur);

function bankAngles() {
  const e = S.eng, nb = Math.max(...e.bank) + 1;
  return nb === 1 ? [0] : [-e.bank_angle_deg / 2, e.bank_angle_deg / 2];
}

// Representative cylinder per bank that shares the drawn crank pin
function representatives(phis) {
  const e = S.eng, reps = [];
  const c0 = e.bank.indexOf(0); reps.push(c0);
  for (let b = 1; b < phis.length; b++) {
    const target = e.fire_angle_deg[c0] + phis[0] - phis[b];
    let best = -1, bd = 1e9;
    e.bank.forEach((bk, c) => {
      if (bk !== b) return;
      const d = Math.abs((((e.fire_angle_deg[c] - target) % 360) + 540) % 360 - 180);
      if (d < bd) { bd = d; best = c; }
    });
    reps.push(best);
  }
  return reps;
}

function drawEngine() {
  const { ctx, w, h } = fit($('engine-canvas'));
  const e = S.eng, st = S.st; if (!e) return;
  const phis = bankAngles(), reps = representatives(phis);
  const inline = phis.length === 1;
  // size the drawing from its real extents: `above` crank centre (in crank radii), `below` = crank disc
  const above = inline ? 3.6 + e.rod_ratio : 2.6 + e.rod_ratio * 0.72, below = 1.7, wide = inline ? 4.2 : 10.5;
  const r = Math.min(h * 0.88 / (above + below), w * 0.9 / wide);
  const L = r * e.rod_ratio;
  const bw = r * 2 * (e.bore_mm / e.stroke_mm);
  const ph = r * 1.1;                              // piston height
  const cx = w / 2, cy = (h - (above + below) * r) / 2 + above * r;
  const deck = L + r + ph * 0.4 + r * 0.25;         // crank centre → deck
  const psi = phis[0] - (S.theta - e.fire_angle_deg[reps[0]]);
  const rad = (d) => d * Math.PI / 180;
  const pinW = [cx + r * Math.sin(rad(psi)), cy - r * Math.cos(rad(psi))];
  const ign = st && st.ignition, burning = st && st.combusting;
  const spark = st ? st.spark_deg : 20;
  const V = e.valves;

  const pistonPos = [];
  phis.forEach((phi, b) => {
    const c = reps[b], alpha = cycleOf(c);
    const beta = rad(psi - phi);
    const s = r * Math.cos(beta) + Math.sqrt(L * L - (r * Math.sin(beta)) ** 2);
    pistonPos.push([cx + s * Math.sin(rad(phi)), cy - s * Math.cos(rad(phi))]);
    ctx.save(); ctx.translate(cx, cy); ctx.rotate(rad(phi));
    // cylinder block
    const wall = bw * 0.16, bottom = -(L - r) + ph * 0.2;
    ctx.fillStyle = '#5b6070';
    ctx.fillRect(-bw / 2 - wall, -deck, wall, deck + bottom);
    ctx.fillRect(bw / 2, -deck, wall, deck + bottom);
    // chamber gas
    const crown = -(s + ph * 0.4);
    let gas = 'rgba(79,209,197,0.10)';
    if (alpha >= 360 - spark && alpha < 500 && burning) {
      const k = Math.max(0, 1 - (alpha - 360) / 140);
      gas = `rgba(255,${120 + 60 * k},60,${0.25 + 0.6 * k})`;
    } else if (alpha >= V.exhaust_open || alpha < (V.exhaust_open + V.exhaust_dur) % 720) gas = 'rgba(150,120,100,0.22)';
    else if (alpha >= (V.intake_open + V.intake_dur) % 720 && alpha < 360) gas = 'rgba(79,209,197,0.18)';
    ctx.fillStyle = gas; ctx.fillRect(-bw / 2, -deck, bw, deck + crown);
    // piston
    ctx.fillStyle = '#eceef3'; ctx.fillRect(-bw / 2 + 1, crown, bw - 2, ph);
    ctx.fillStyle = '#b4b9c6'; ctx.fillRect(-bw / 2 + 1, crown + ph * 0.12, bw - 2, ph * 0.06);
    ctx.fillRect(-bw / 2 + 1, crown + ph * 0.24, bw - 2, ph * 0.06);
    ctx.fillStyle = '#4a4f5c'; ctx.beginPath(); ctx.arc(0, -s, r * 0.12, 0, 7); ctx.fill();
    // head
    const headH = r * 0.9;
    ctx.fillStyle = '#464b59'; ctx.fillRect(-bw / 2 - wall, -deck - headH, bw + 2 * wall, headH);
    // valves (intake left, exhaust right) and cams
    const vx = bw * 0.26, vr = bw * 0.17, travel = r * 0.35;
    [[-vx, intakeLift(alpha), C.teal, S.eng.valve_lift?.intake_peak_deg ?? (V.intake_open + V.intake_dur / 2)], [vx, exhaustLift(alpha), C.yellow, S.eng.valve_lift?.exhaust_peak_deg ?? (V.exhaust_open + V.exhaust_dur / 2)]].forEach(([x, l, col, peak]) => {
      const y = -deck + l * travel;
      ctx.fillStyle = C.pink;
      ctx.fillRect(x - r * 0.05, y - headH - r * 0.35, r * 0.1, headH + r * 0.35);
      ctx.fillRect(x - vr, y - r * 0.08, vr * 2, r * 0.1);
      // cam: rotates at half crank speed, lobe points at the valve at peak lift
      const camA = rad((alpha - peak) / 2);
      const ccx = x, ccy = -deck - headH - r * 0.75, cr = r * 0.3;
      ctx.save(); ctx.translate(ccx, ccy); ctx.rotate(camA);
      ctx.fillStyle = col; ctx.beginPath();
      ctx.arc(0, 0, cr, 0, 7); ctx.moveTo(-cr * 0.85, cr * 0.5);
      ctx.quadraticCurveTo(0, cr * 2.1, cr * 0.85, cr * 0.5); ctx.fill();
      ctx.restore();
    });
    // spark plug + flash
    ctx.fillStyle = '#d3d7e0'; ctx.fillRect(-r * 0.06, -deck - headH - r * 0.3, r * 0.12, headH + r * 0.3);
    const since = alpha - (360 - spark);
    if (ign && since >= 0 && since < 14) {
      ctx.fillStyle = burning ? 'rgba(255,220,120,0.95)' : 'rgba(210,230,255,0.9)';
      ctx.beginPath(); ctx.arc(0, -deck + r * 0.05, r * (0.12 + 0.2 * (1 - since / 14)), 0, 7); ctx.fill();
    }
    ctx.restore();
  });

  // crank: disc + counterweight, rods, pin
  ctx.fillStyle = '#8b90a0'; ctx.beginPath(); ctx.arc(cx, cy, r * 1.55, 0, 7); ctx.fill();
  // counterweight: sector opposite the pin (pin sits at canvas angle ψ − 90°)
  const cw = rad(psi) + Math.PI / 2;
  ctx.fillStyle = '#c3c7d2'; ctx.beginPath(); ctx.moveTo(cx, cy);
  ctx.arc(cx, cy, r * 1.45, cw - 1.1, cw + 1.1); ctx.closePath(); ctx.fill();
  ctx.strokeStyle = C.pink; ctx.lineCap = 'round'; ctx.lineWidth = r * 0.34;
  pistonPos.forEach(([x, y]) => { ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(pinW[0], pinW[1]); ctx.stroke(); });
  ctx.fillStyle = '#e4e6ec'; ctx.beginPath(); ctx.arc(pinW[0], pinW[1], r * 0.22, 0, 7); ctx.fill();
  ctx.fillStyle = '#5b6070'; ctx.beginPath(); ctx.arc(cx, cy, r * 0.28, 0, 7); ctx.fill();
}

// ------------------------------------------------------------------ cylinder firing lamps (one canvas)
function buildIgnition() {
  const e = S.eng, nb = Math.max(...e.bank) + 1;
  S.ignRows = Array.from({ length: nb }, (_, b) => e.bank.map((k, c) => (k === b ? c : -1)).filter((c) => c >= 0));
  S.ignSig = null;
}

function drawIgnition(dt) {
  const e = S.eng, st = S.st; if (!e || !st || !S.ignRows) return;
  let sig = st.combusting ? 'c' : 'n';
  for (let c = 0; c < e.cylinders; c++) {
    const since = cycleOf(c) - (360 - st.spark_deg);
    if (st.ignition && st.rpm > 30 && since >= 0 && since < Math.max(30, 720 / e.cylinders * 0.6)) S.fireGlow[c] = 1;
    else S.fireGlow[c] *= Math.exp(-dt / 0.06);
    sig += Math.round(S.fireGlow[c] * 20);
  }
  const sz = fit($('ign-canvas'), false);
  sig += '|' + sz.ver;
  if (sig === S.ignSig) return;
  S.ignSig = sig;
  const { ctx, w, h } = sz;
  ctx.clearRect(0, 0, w, h);
  const rows = S.ignRows.length, cols = Math.max(...S.ignRows.map((r) => r.length));
  const cellW = w / cols, cellH = h / rows, rad = Math.max(3, Math.min(cellW, cellH) * 0.36);
  const col = st.combusting ? [255, 190, 110] : [200, 215, 240];
  S.ignRows.forEach((row, ri) => row.forEach((c, ci) => {
    const x = cellW * (ci + 0.5), y = cellH * (ri + 0.5), g = S.fireGlow[c];
    if (g > 0.05) {
      ctx.fillStyle = `rgba(${col[0]},${col[1]},${col[2]},${0.25 * g})`;
      ctx.beginPath(); ctx.arc(x, y, rad * (1 + 0.35 * g), 0, 7); ctx.fill();
    }
    const m = (i) => Math.round(52 + (col[i] - 52) * g);
    ctx.fillStyle = `rgb(${m(0)},${m(1) + 4},${m(2) + 10})`;
    ctx.beginPath(); ctx.arc(x, y, rad, 0, 7); ctx.fill();
  }));
}

// ------------------------------------------------------------------ plots
function linePlot(cv, series, opts = {}) {
  const { ctx, w, h } = fit(cv);
  const pad = 4, top = 16;
  let lo = opts.min, hi = opts.max;
  if (lo === undefined || hi === undefined) {
    let a = Infinity, b = -Infinity;
    for (const s of series) { const d = s.data; for (let i = 0; i < d.length; i++) { const v = d[i]; if (v < a) a = v; if (v > b) b = v; } }
    if (lo === undefined) lo = Math.min(0, a); if (hi === undefined) hi = Math.max(b * 1.1, lo + (opts.minSpan || 1));
  }
  for (const s of series) {
    const d = s.data; if (d.length < 2) continue;
    ctx.strokeStyle = s.color; ctx.lineWidth = 1.4; ctx.lineJoin = 'round'; ctx.beginPath();
    const n = s.n || d.length;
    d.forEach((v, i) => {
      const x = pad + (w - 2 * pad) * (s.xs ? s.xs[i] : i / (n - 1));
      const y = h - pad - (h - pad - top) * (clamp(v, lo, hi) - lo) / (hi - lo);
      i ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
    });
    ctx.stroke();
  }
  if (opts.marker !== undefined) {
    ctx.strokeStyle = 'rgba(255,255,255,0.45)'; ctx.lineWidth = 1;
    const x = pad + (w - 2 * pad) * opts.marker;
    ctx.beginPath(); ctx.moveTo(x, top); ctx.lineTo(x, h - pad); ctx.stroke();
  }
  if (opts.label) {
    ctx.fillStyle = C.dim; ctx.font = `600 10px ${MONO}`; ctx.textAlign = 'right'; ctx.textBaseline = 'top';
    ctx.fillText(opts.label, w - 6, 5);
  }
}

function drawPlots() {
  const e = S.eng, st = S.st; if (!e || !st) return;
  linePlot(P('rpm'), [{ data: S.hist.rpm, color: C.teal, n: HIST_N }],
           { min: 0, max: e.redline_rpm + 500, label: `${Math.round(st.rpm)}` });
  linePlot(P('map'), [{ data: S.hist.map, color: C.accent, n: HIST_N }],
           { min: 0, max: 110, label: `${st.map_kpa.toFixed(0)} kPa` });
  const alpha = cycleOf(e.bank.indexOf(0));
  const xs = [], il = [], el = [];
  for (let a = 0; a <= 720; a += 6) { xs.push(a / 720); il.push(intakeLift(a)); el.push(exhaustLift(a)); }
  linePlot(P('valves'), [{ data: il, xs, color: C.teal }, { data: el, xs, color: C.accent }],
           { min: 0, max: 1.15, marker: alpha / 720 });
  if (S.trace) {
    const p = S.trace.p, v = S.trace.v, n = p.length;
    const pmax = Math.max(1000, ...p);
    linePlot(P('pcyl'), [{ data: p, xs: p.map((_, i) => i / (n - 1)), color: C.accent }],
             { min: 0, max: pmax * 1.05, marker: alpha / 720,
               label: `${(pmax / 100).toFixed(0)} bar` + (st.resolved ? ` · IMEP ${st.imep_bar.toFixed(1)} · EXH ${st.exh_kpa.toFixed(0)} kPa${S.eng.turbo ? ` · BOOST ${st.boost_kpa.toFixed(0)} kPa · ${st.turbo_krpm.toFixed(0)}k rpm` : ''}${st.misfire > 0.05 ? ` · MISFIRE ${(100 * st.misfire).toFixed(0)}%` : ''}${st.substep_deg > 6 ? ` · ${st.substep_deg.toFixed(0)}°/step` : ''}` : '') });
    // log P – log V loop
    const { ctx, w, h } = fit(P('pv'));
    const lv0 = Math.log(Math.min(...v)), lv1 = Math.log(Math.max(...v));
    const lp0 = Math.log(10), lp1 = Math.log(Math.max(2000, pmax * 1.2));
    ctx.strokeStyle = C.accent; ctx.lineWidth = 1.4; ctx.beginPath();
    for (let i = 0; i < n; i++) {
      const x = 6 + (w - 12) * (Math.log(v[i]) - lv0) / (lv1 - lv0);
      const y = h - 5 - (h - 22) * (Math.log(Math.max(10, p[i])) - lp0) / (lp1 - lp0);
      i ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
    }
    ctx.stroke();
    const k = Math.round(alpha / 720 * (n - 1));
    const x = 6 + (w - 12) * (Math.log(v[k]) - lv0) / (lv1 - lv0);
    const y = h - 5 - (h - 22) * (Math.log(Math.max(10, p[k])) - lp0) / (lp1 - lp0);
    ctx.fillStyle = C.text; ctx.beginPath(); ctx.arc(x, y, 2.8, 0, 7); ctx.fill();
  }
  const tq = S.hist.tq, hp = S.hist.hp;
  let hi = 50, lo = 0;
  for (let i = 0; i < tq.length; i++) { hi = Math.max(hi, Math.abs(tq[i]), hp[i]); lo = Math.min(lo, tq[i], hp[i]); }
  linePlot(P('tq'), [{ data: tq, color: C.text, n: HIST_N }, { data: hp, color: C.teal, n: HIST_N }],
           { min: lo, max: hi * 1.1, label: `${r0(st.torque_lbft)} / ${r0(st.hp)}` });
  drawDynoCurve(st);
  if (!$('dyno-plot-modal').hidden) { drawDynoCurve(st, $('dyno-big'), true); dynoTable(); }
}

// Torque and power against engine speed: the measured dyno curve, with the live reading as a ring
function niceStep(range, n) {
  const raw = Math.max(range, 1e-6) / n, mag = 10 ** Math.floor(Math.log10(raw)), f = raw / mag;
  return (f < 1.5 ? 1 : f < 3.5 ? 2 : f < 7.5 ? 5 : 10) * mag;
}
function drawDynoCurve(st, cv = P('dyno'), big = false) {
  const { ctx, w, h } = fit(cv);
  const e = S.eng, pts = S.dyno.curve, live = st.dyno && st.rpm > 50;
  const fs = big ? 12 : 9, padL = big ? 58 : 34, padR = big ? 20 : 10, padT = big ? 44 : 22, padB = big ? 38 : 16;
  const xMax = Math.ceil(e.redline_rpm / 500) * 500;
  let hi = 50, lo = 0;
  for (const p of pts) { hi = Math.max(hi, p.tq, p.hp); lo = Math.min(lo, p.tq, p.hp); }
  if (live) { hi = Math.max(hi, st.torque_lbft, st.hp); lo = Math.min(lo, st.torque_lbft, st.hp); }
  hi *= 1.12; if (lo < 0) lo *= 1.12;
  const X = (r) => padL + (w - padL - padR) * clamp(r / xMax, 0, 1);
  const Y = (v) => padT + (h - padT - padB) * (1 - (v - lo) / (hi - lo));
  ctx.font = `600 ${fs}px ${MONO}`; ctx.lineWidth = 1;
  const ys = niceStep(hi - lo, big ? 7 : 4);
  ctx.textAlign = 'right'; ctx.textBaseline = 'middle';
  for (let v = Math.ceil(lo / ys) * ys; v <= hi; v += ys) {
    const y = Math.round(Y(v)) + 0.5;
    ctx.strokeStyle = v === 0 ? C.faint : C.line; ctx.beginPath(); ctx.moveTo(padL, y); ctx.lineTo(w - padR, y); ctx.stroke();
    ctx.fillStyle = C.faint; ctx.fillText(String(Math.round(v)), padL - 5, y);
  }
  const xs = niceStep(xMax, Math.max(2, Math.floor((w - padL - padR) / (big ? 70 : 55))));
  ctx.textAlign = 'center'; ctx.textBaseline = 'alphabetic';
  for (let r = 0; r <= xMax; r += xs) {
    const x = Math.round(X(r)) + 0.5;
    ctx.strokeStyle = C.line; ctx.beginPath(); ctx.moveTo(x, padT); ctx.lineTo(x, h - padB); ctx.stroke();
    ctx.fillStyle = C.faint; ctx.fillText(r >= 1000 ? `${r / 1000}k` : String(r), x, h - padB + fs + 4);
  }
  if (big) {
    ctx.fillStyle = C.dim; ctx.font = `700 ${fs}px ${FONT}`; ctx.textAlign = 'center';
    ctx.fillText('ENGINE SPEED (rpm)', (padL + w - padR) / 2, h - 5);
    ctx.save(); ctx.translate(13, (padT + h - padB) / 2); ctx.rotate(-Math.PI / 2);
    ctx.fillText('TORQUE (lb-ft) · POWER (hp)', 0, 0); ctx.restore();
    ctx.font = `600 ${fs}px ${MONO}`;
  }
  if (5252 < xMax) {                 // hp = torque × rpm / 5252: the curves must cross here
    const x = Math.round(X(5252)) + 0.5;
    ctx.save(); ctx.setLineDash([3, 3]); ctx.strokeStyle = C.faint; ctx.beginPath(); ctx.moveTo(x, padT); ctx.lineTo(x, h - padB); ctx.stroke(); ctx.restore();
    ctx.fillStyle = C.faint; ctx.textAlign = 'right'; ctx.fillText(big ? '5252 rpm: tq = hp' : '5252', x - 3, padT + fs);
  }
  const line = (key, col) => {
    ctx.strokeStyle = col; ctx.fillStyle = col; ctx.lineWidth = big ? 2.6 : 1.8; ctx.lineJoin = 'round'; ctx.beginPath();
    pts.forEach((p, i) => (i ? ctx.lineTo(X(p.rpm), Y(p[key])) : ctx.moveTo(X(p.rpm), Y(p[key]))));
    ctx.stroke();
    for (const p of pts) { ctx.beginPath(); ctx.arc(X(p.rpm), Y(p[key]), big ? 3.4 : 2, 0, 7); ctx.fill(); }
  };
  line('hp', C.teal); line('tq', C.text);
  let leg = [];
  if (pts.length) {
    const pk = (k) => pts.reduce((m, p) => (p[k] > m[k] ? p : m), pts[0]);
    const t = pk('tq'), p = pk('hp');
    for (const [pt, k, col] of [[t, 'tq', C.text], [p, 'hp', C.teal]]) {
      ctx.strokeStyle = col; ctx.lineWidth = 1.5; ctx.beginPath(); ctx.arc(X(pt.rpm), Y(pt[k]), big ? 7 : 4.5, 0, 7); ctx.stroke();
    }
    leg = big ? [[`peak torque ${r0(t.tq)} lb-ft @ ${r0(t.rpm)} rpm`, C.text], [`peak power ${r0(p.hp)} hp @ ${r0(p.rpm)} rpm`, C.teal]]
              : [[`${r0(t.tq)} @ ${r0(t.rpm)}`, C.text], [`${r0(p.hp)} @ ${r0(p.rpm)}`, C.teal]];
  }
  ctx.textBaseline = 'top'; ctx.font = `600 ${big ? 12 : 10}px ${MONO}`; ctx.textAlign = 'right';
  let lx = w - 10, ly = big ? 12 : 5;
  if (big) for (const [txt, col] of leg) { ctx.fillStyle = col; ctx.fillText(txt, lx, ly); ly += 16; }
  else for (const [txt, col] of leg) { ctx.fillStyle = col; ctx.fillText(txt, lx, ly); lx -= ctx.measureText(txt).width + 14; }
  if (live) {
    const x = X(st.rpm);
    ctx.strokeStyle = 'rgba(255,255,255,0.3)'; ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(x, padT); ctx.lineTo(x, h - padB); ctx.stroke();
    for (const [v, col] of [[st.torque_lbft, C.text], [st.hp, C.teal]]) {
      ctx.strokeStyle = col; ctx.fillStyle = C.panel; ctx.lineWidth = 1.6; ctx.beginPath(); ctx.arc(x, Y(v), big ? 5 : 3.6, 0, 7); ctx.fill(); ctx.stroke();
    }
  }
  if (!pts.length) {
    ctx.fillStyle = C.dim; ctx.font = `600 ${big ? 14 : 11}px ${FONT}`; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    ctx.fillText(S.dyno.run ? 'Measuring…' : 'Press D, then run a sweep to draw the curve', (padL + w - padR) / 2, (padT + h - padB) / 2);
  }
  if (big && pts.length && S.dyno.hoverX !== null) {          // crosshair snaps to the nearest measured point
    let best = pts[0];
    for (const p of pts) if (Math.abs(X(p.rpm) - S.dyno.hoverX) < Math.abs(X(best.rpm) - S.dyno.hoverX)) best = p;
    const x = X(best.rpm);
    ctx.strokeStyle = C.accent; ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(x, padT); ctx.lineTo(x, h - padB); ctx.stroke();
    for (const [v, col] of [[best.tq, C.text], [best.hp, C.teal]]) {
      ctx.fillStyle = col; ctx.strokeStyle = C.panel; ctx.lineWidth = 2; ctx.beginPath(); ctx.arc(x, Y(v), 5.5, 0, 7); ctx.fill(); ctx.stroke();
    }
    const rows = [[`${r0(best.rpm)} rpm`, C.accent], [`${best.tq.toFixed(1)} lb-ft  (${(best.tq / LB_FT_PER_NM).toFixed(0)} N·m)`, C.text], [`${best.hp.toFixed(1)} hp  (${(best.hp * 0.7457).toFixed(1)} kW)`, C.teal]];
    ctx.font = `600 12px ${MONO}`;
    const bw = Math.max(...rows.map((r) => ctx.measureText(r[0]).width)) + 20, bh = rows.length * 18 + 10;
    const bx = x + 14 + bw > w - padR ? x - 14 - bw : x + 14, by = padT + 8;
    ctx.fillStyle = 'rgba(14,16,21,0.94)'; ctx.strokeStyle = C.line2 || '#343b49'; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.roundRect(bx, by, bw, bh, 8); ctx.fill(); ctx.stroke();
    ctx.textAlign = 'left'; ctx.textBaseline = 'middle';
    rows.forEach(([t, col], i) => { ctx.fillStyle = col; ctx.fillText(t, bx + 10, by + 14 + i * 18); });
  }
}

// ---- detailed view: click the dyno plot to open it large, with a hover read-out and the data table
const LB_FT_PER_NM = 0.737562;
function openDynoPlot() {
  gateOff(); S.dyno.hoverX = null; S.dyno.tableKey = null;
  $('dyno-plot-modal').hidden = false;
  if (S.st) { fit($('dyno-big')); drawDynoCurve(S.st, $('dyno-big'), true); }
}
function closeDynoPlot() { $('dyno-plot-modal').hidden = true; }
function dynoTable() {
  const c = S.dyno.curve, key = c.length + ':' + (S.dyno.run ? 1 : 0);
  if (S.dyno.tableKey === key) return;
  S.dyno.tableKey = key;
  $('dp-table').innerHTML = c.length
    ? '<tr><th>rpm</th><th>lb-ft</th><th>N·m</th><th>hp</th><th>kW</th></tr>' + c.map((p) =>
      `<tr><td>${r0(p.rpm)}</td><td>${p.tq.toFixed(1)}</td><td>${(p.tq / LB_FT_PER_NM).toFixed(0)}</td><td>${p.hp.toFixed(1)}</td><td>${(p.hp * 0.7457).toFixed(1)}</td></tr>`).join('')
    : '<tr><td class="dim">No measurements yet. Press D and run a sweep.</td></tr>';
}

function drawWave() {
  const { ctx, w, h } = fit($('wave'));
  const an = S.audio.analyser;
  ctx.strokeStyle = C.teal; ctx.lineWidth = 1;
  if (!an) {
    ctx.fillStyle = C.dim; ctx.font = `600 11px ${FONT}`; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    ctx.fillText('Audio is off. Click or press any key.', w / 2, h / 2 + 6); return;
  }
  const buf = S.audio.buf; an.getFloatTimeDomainData(buf);
  const n = buf.length, cols = Math.max(1, Math.floor(w - 16)), per = n / cols;
  const mid = h / 2 + 8, amp = (h - 30) / 2;
  ctx.beginPath();
  for (let x = 0; x < cols; x++) {
    let lo = 1, hi = -1;
    const s0 = Math.floor(x * per), s1 = Math.floor((x + 1) * per);
    for (let i = s0; i < s1; i++) { const v = buf[i]; if (v < lo) lo = v; if (v > hi) hi = v; }
    const g = 1 / Math.max(0.15, S.smooth.wavePk || 0.3);
    ctx.moveTo(8 + x, mid - hi * amp * g); ctx.lineTo(8 + x, mid - lo * amp * g + 0.5);
  }
  ctx.stroke();
  smooth('wavePk', Math.max(S.audio.peak, 0.05), 0.02);
}

// Output level bar with peak hold and a clip warning
function drawLevel(now) {
  const { ctx, w, h } = fit($('lvl-canvas'));
  ctx.fillStyle = C.track; ctx.beginPath(); ctx.roundRect(0, 0, w, h, h / 2); ctx.fill();
  const a = S.audio, lvl = smooth('lvl', a.peak, 0.4);
  if (a.peak >= a.peakHold || now - a.peakHoldT > 1200) { a.peakHold = a.peak; a.peakHoldT = now; }
  const x = clamp(lvl, 0, 1) * w;
  const g = ctx.createLinearGradient(0, 0, w, 0);
  g.addColorStop(0, C.green); g.addColorStop(0.65, C.green); g.addColorStop(0.85, C.accent); g.addColorStop(1, C.red);
  ctx.fillStyle = g; ctx.beginPath(); ctx.roundRect(0, 0, Math.max(h, x), h, h / 2); ctx.fill();
  ctx.fillStyle = a.peakHold > 0.98 ? C.red : C.text;
  ctx.fillRect(clamp(a.peakHold, 0, 1) * w - 1, 0, 2, h);
}

// Live spectrum of what you hear: log frequency axis, so BASS / TREBLE / RING changes are visible
const SPEC_BANDS = 96, SPEC_F0 = 20, SPEC_F1 = 12000;
function drawSpectrum() {
  const { ctx, w, h } = fit($('spec-canvas'));
  const an = S.audio.analyser;
  const padL = 4, padB = 13, plotW = w - padL - 4, plotH = h - padB - 3;
  ctx.fillStyle = C.faint; ctx.font = `600 9px ${MONO}`; ctx.textBaseline = 'alphabetic';
  const xf = (f) => padL + plotW * Math.log(f / SPEC_F0) / Math.log(SPEC_F1 / SPEC_F0);
  ctx.strokeStyle = C.line; ctx.lineWidth = 1;
  for (const [f, t] of [[50, '50'], [100, '100'], [200, '200'], [500, '500'], [1000, '1k'], [2000, '2k'], [5000, '5k'], [10000, '10k']]) {
    const x = Math.round(xf(f)) + 0.5;
    ctx.beginPath(); ctx.moveTo(x, 2); ctx.lineTo(x, padB ? h - padB : h); ctx.stroke();
    ctx.textAlign = 'center'; ctx.fillText(t, x, h - 3);
  }
  if (!an) return;
  if (!S.specBuf) S.specBuf = new Float32Array(an.frequencyBinCount);
  an.minDecibels = -105; an.maxDecibels = -25; an.smoothingTimeConstant = 0.8;
  an.getFloatFrequencyData(S.specBuf);
  const binHz = S.audio.ctx.sampleRate / an.fftSize, buf = S.specBuf;
  const top = (db) => 3 + plotH * (1 - clamp((db + 105) / 80, 0, 1));
  ctx.beginPath(); ctx.moveTo(padL, 3 + plotH);
  let lastY = 3 + plotH;
  for (let b = 0; b < SPEC_BANDS; b++) {
    const f0 = SPEC_F0 * Math.pow(SPEC_F1 / SPEC_F0, b / SPEC_BANDS), f1 = SPEC_F0 * Math.pow(SPEC_F1 / SPEC_F0, (b + 1) / SPEC_BANDS);
    let k0 = Math.floor(f0 / binHz), k1 = Math.max(k0, Math.ceil(f1 / binHz) - 1), m = -200;
    for (let k = k0; k <= k1 && k < buf.length; k++) if (buf[k] > m) m = buf[k];
    lastY = top(m);
    ctx.lineTo(padL + plotW * (b + 0.5) / SPEC_BANDS, lastY);
  }
  ctx.lineTo(padL + plotW, lastY); ctx.lineTo(padL + plotW, 3 + plotH); ctx.closePath();
  const g = ctx.createLinearGradient(0, 0, 0, h);
  g.addColorStop(0, 'rgba(255,180,84,0.55)'); g.addColorStop(1, 'rgba(255,180,84,0.05)');
  ctx.fillStyle = g; ctx.fill();
  ctx.strokeStyle = C.accent; ctx.lineWidth = 1.3; ctx.stroke();
}

function pedals(st) {
  const clutch = st.gear === 0 ? ctl.clutch_pedal : 1 - st.clutch;
  for (const [id, v] of [['thr', st.throttle], ['clu', clutch], ['brk', st.brake]]) {
    const pct = Math.round(clamp(v, 0, 1) * 100);
    const bar = $(`p-${id}`);
    if (bar.__p !== pct) { bar.__p = pct; bar.style.width = `${pct}%`; setText($(`p-${id}-v`), String(pct)); }
  }
}

// ------------------------------------------------------------------ main render
let frameNo = 0, renderErrors = 0;
function render(now) {
  try { renderFrame(now); } catch (err) {
    if (renderErrors++ < 3) console.error('render error:', err);
  }
  requestAnimationFrame(render);
}
function renderFrame(now) {
  frameNo++;
  const slow = (frameNo & 1) === 0;       // 30 Hz tier: diagnostics, plots, text
  const dt = Math.max(0, Math.min(0.1, (now - S.lastFrameT) / 1000));
  S.lastFrameT = now;
  S.fps += ((dt > 0 ? 1 / dt : 60) - S.fps) * 0.05;
  updateThrottle(dt);
  const st = S.st, e = S.eng;
  if (st && e) {
    // advance animation crank angle from rpm (slow-motion aware)
    S.theta = (S.theta + st.rpm * 6 * dt * (st.time_scale || 1)) % 720;
    const redMax = Math.ceil((e.redline_rpm + 500) / 1000) * 1000;
    const big = e.cylinders >= 8;

    if (slow) {
      syncControls();
      setState(st);
      setText($('c-fps'), S.fps.toFixed(0));
      setText($('c-rt'), `${st.rt_ratio.toFixed(2)}×`);
      setText($('c-hz'), String(Math.round(st.steps_per_s)));
      const lat = S.audio.ctx ? (S.lastPkt ? performance.now() - S.lastPkt : 0) + ((S.audio.ctx.baseLatency || 0) + (S.audio.ctx.outputLatency || 0)) * 1000 : null;
      setText($('c-lat'), lat == null ? '--' : String(Math.round(smooth('lat', lat, 0.1))));
      lamps(st);
      pedals(st);
      setText($('gear'), gearName(st.gear));
      setText($('f-l'), `${st.fuel_l.toFixed(3)} L`);
      setText($('f-gal'), `${st.fuel_gal.toFixed(3)} gal`);
      setText($('f-usd'), `$${st.fuel_usd.toFixed(2)}`);
      setText($('f-mpg'), `${st.mpg.toFixed(1)} mpg`);
      setText($('f-mi'), `${st.miles.toFixed(2)} mi`);
      drawPlots();
      drawLevel(now);
      drawSpectrum();
    }

    // instruments
    gauge(G('rpm'), { min: 0, max: redMax, value: st.rpm, text: r0(st.rpm), unit: 'RPM', major: redMax / 1000, minor: 2,
                      label: (v) => String(v / 1000), bands: [[e.redline_rpm, redMax, C.red]] });
    const mphMax = big ? 160 : 120;
    gauge(G('mph'), { min: 0, max: mphMax, value: smooth('mph', st.speed_mph, 0.4), text: r0(st.speed_mph), unit: 'MPH', major: mphMax / 20, minor: 2 });

    // vitals
    const scfmMax = big ? 800 : 250;
    // A turbocharged engine reads boost above zero: widen the scale (psi-like range in inHg).
    const boosted = S.eng.turbo;
    gauge(G('mapg'), { min: -30, max: boosted ? 35 : 5, value: smooth('mapg', st.map_inhg, 0.4), text: r0(st.map_inhg), unit: 'inHg',
                       major: boosted ? 13 : 7, minor: 1,
                       bands: boosted ? [[-30, -20, C.teal], [0, 35, C.accent]] : [[-30, -20, C.teal], [0, 5, C.accent]] });
    gauge(G('scfm'), { min: 0, max: scfmMax, value: smooth('scfm', st.air_scfm, 0.3), text: st.air_scfm.toFixed(0), unit: 'SCFM', major: 4, minor: 4,
                       bands: [[scfmMax * 0.8, scfmMax, C.accent]] });
    const ve = st.ve * 100 * (st.rpm > 20 ? 1 : 0);
    gauge(G('ve'), { min: 0, max: 100, value: smooth('ve', ve, 0.3), text: ve.toFixed(0), unit: '%', major: 5, minor: 2,
                     bands: [[0, 30, C.red], [80, 100, C.teal]] });
    gauge(G('afr'), { min: 8, max: 20, value: st.afr == null ? 20 : st.afr, text: st.afr == null ? (st.ignition ? 'CUT' : '--') : st.afr.toFixed(1), unit: 'AFR', major: 6, minor: 2,
                      bands: [[8, 12, C.accent], [14.2, 15.2, C.green], [16, 20, C.blue]] });
    gauge(G('o2'), { min: 0, max: 21, value: st.o2, text: st.o2.toFixed(1), unit: '% O₂', major: 7, minor: 3 });
    gauge(G('spark'), { min: 0, max: 50, value: st.combusting ? st.spark_deg : 0, text: st.combusting ? st.spark_deg.toFixed(0) : '--', unit: '° BTDC', major: 5, minor: 2 });
    const tqR = big ? [-200, 600] : [-50, 150], hpR = big ? [-100, 500] : [-30, 120];
    gauge(G('torque'), { min: tqR[0], max: tqR[1], value: smooth('tq', st.torque_lbft, 0.3), text: r0(st.torque_lbft), unit: 'LB-FT', major: (tqR[1] - tqR[0]) / (big ? 100 : 50), minor: 2,
                         bands: [[tqR[0], 0, C.teal]] });
    gauge(G('hp'), { min: hpR[0], max: hpR[1], value: smooth('hp', st.hp, 0.3), text: r0(st.hp), unit: 'HP', major: (hpR[1] - hpR[0]) / (big ? 100 : 30), minor: 2,
                     bands: [[hpR[0], 0, C.teal]] });

    drawEngine();
    drawIgnition(dt);
  }
  drawWave();
}

// ------------------------------------------------------------------ status widgets
function setState(st) {
  let text, c = '';
  if (st.starter) { text = st.rpm < 400 ? 'CRANKING' : 'STARTING'; c = 'crank'; }
  else if (!st.ignition) { text = st.rpm > 50 ? 'IGNITION OFF · COASTING' : 'ENGINE OFF'; c = st.rpm > 50 ? 'warn' : ''; }
  else if (st.rpm < 30) { text = 'STALLED · HOLD START'; c = 'bad'; }
  else if (st.limiter && st.throttle > 0.01) { text = 'REV LIMITER'; c = 'bad'; }
  else if (st.fuel_cut && st.gear > 0 && st.clutch_locked) { text = `ENGINE BRAKING · ${r0(Math.abs(st.torque_lbft))} LB-FT`; c = 'cut'; }
  else if (st.fuel_cut) { text = 'FUEL CUT'; c = 'cut'; }
  else if (st.dyno) { text = 'DYNO HOLD'; c = 'dyno'; }
  else { text = 'RUNNING'; c = 'run'; }
  const el = $('state-pill');
  setText(el, text);
  if (el.__cls !== c) { el.__cls = c; el.className = 'pill' + (c ? ' ' + c : ''); }
}
function lamps(st) {
  cls($('lamp-ign'), 'on', st.ignition);
  cls($('lamp-start'), 'on', st.starter);
  cls($('lamp-cut'), 'on', st.fuel_cut && st.ignition);
  cls($('lamp-dyno'), 'on', st.dyno);
  const cl = $('lamp-clutch'), active = st.gear > 0;
  cls(cl, 'on', active);
  setText(cl, active ? (st.clutch_locked ? 'CLUTCH LOCK' : 'CLUTCH SLIP') : 'CLUTCH');
}

// ------------------------------------------------------------------ boot
window.addEventListener('keydown', onKeyDown);
window.addEventListener('keyup', onKeyUp);
window.addEventListener('blur', () => { keys.clear(); ctl.starter = false; ctl.brake = 0; ctl.clutch_pedal = 0; pushControls(); });
$('audio-gate').addEventListener('pointerdown', gateOff);
document.addEventListener('pointerdown', gateOff, true);      // any click enables audio
$('help').addEventListener('pointerdown', () => { $('help').hidden = true; });
buildMixer();
wireControls();
$('mute-waves').parentNode.setAttribute('data-muted', '0');
loadConfig().then(() => { pushControls({ action: 'reset' }); connectStream(); requestAnimationFrame(render); });
