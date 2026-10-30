'use strict';
// Reusable UI pieces: hover tooltips, rotary knobs, and the sound-profile spec.
// (Plain script; everything is exposed as globals for app.js.)

// ------------------------------------------------------------------ tooltips
// Any element with data-tip="Title::Body text" (or bound with Tip.bind) gets a
// hover / keyboard-focus tooltip. Dynamic tips pass a function returning
// {title, body, lines[]}.
const Tip = (() => {
  const el = document.createElement('div');
  el.id = 'tip'; el.setAttribute('role', 'tooltip'); el.hidden = true;
  document.body.appendChild(el);
  const fns = new WeakMap();
  let cur = null, timer = 0, visible = false;

  function content(t) {
    const esc = (s) => String(s).replace(/[&<>]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;' }[c]));
    let h = `<div class="tip-title">${esc(t.title)}</div>`;
    if (t.body) h += `<div class="tip-body">${esc(t.body)}</div>`;
    for (const l of (t.lines || [])) h += `<div class="tip-line"><b>${esc(l[0])}</b> ${esc(l[1])}</div>`;
    if (t.foot) h += `<div class="tip-foot">${esc(t.foot)}</div>`;
    return h;
  }
  function resolve(target) {
    const f = fns.get(target);
    if (f) return f();
    const raw = target.getAttribute('data-tip');
    if (!raw) return null;
    const i = raw.indexOf('::');
    return i < 0 ? { title: raw } : { title: raw.slice(0, i), body: raw.slice(i + 2) };
  }
  function place() {
    if (!cur) return;
    const r = cur.getBoundingClientRect(), w = el.offsetWidth, h = el.offsetHeight, m = 8;
    let x = r.left + r.width / 2 - w / 2;
    x = Math.max(m, Math.min(innerWidth - w - m, x));
    let y = r.bottom + 8;
    if (y + h > innerHeight - m) y = Math.max(m, r.top - h - 8);
    el.style.left = `${Math.round(x)}px`; el.style.top = `${Math.round(y)}px`;
  }
  function show(target) {
    const t = resolve(target);
    if (!t) return;
    cur = target; el.innerHTML = content(t); el.hidden = false; visible = true; place();
  }
  function hide() { clearTimeout(timer); cur = null; el.hidden = true; visible = false; }
  function find(node) { return node && node.closest ? node.closest('[data-tip],.has-tip') : null; }

  document.addEventListener('pointerover', (e) => {
    const t = find(e.target);
    if (!t || t === cur) return;
    clearTimeout(timer);
    if (visible) show(t); else timer = setTimeout(() => show(t), 280);
  });
  document.addEventListener('pointerout', (e) => {
    const t = find(e.target);
    if (!t) return;
    if (e.relatedTarget && t.contains(e.relatedTarget)) return;
    hide();
  });
  document.addEventListener('focusin', (e) => { const t = find(e.target); if (t && e.target.matches(':focus-visible')) show(t); });
  document.addEventListener('focusout', hide);
  document.addEventListener('scroll', hide, true);
  document.addEventListener('keydown', (e) => { if (e.key === 'Escape') hide(); }, true);

  return {
    bind(target, fn) { fns.set(target, fn); target.classList.add('has-tip'); },
    // re-render the visible tip (e.g. while a knob is being dragged)
    refresh(target) { if (visible && cur === target) show(target); },
    hide,
  };
})();

// ------------------------------------------------------------------ sound profile spec
// Each entry is one knob. `fmt` turns the raw value into display text.
const pct = (v) => `${Math.round(v * 100)} %`;
const dB = (v) => `${v > 0.05 ? '+' : ''}${v.toFixed(1)} dB`;

const PROFILE = [
  // ---- MIX
  { id: 'vol', group: 'MIX', code: 'VOL', name: 'Master volume', min: 0, max: 1, def: 0.7, fmt: pct,
    body: 'Overall loudness of everything you hear.',
    lo: 'quieter', hi: 'louder', foot: 'Keys: − / +   ·   M mutes' },
  { id: 'conv', group: 'MIX', code: 'CONV', name: 'Room / convolution', min: 0, max: 1, def: 0.35, fmt: pct,
    body: 'How much of the room around the engine you hear: early reflections off nearby walls, and at higher settings a reverb tail.',
    lo: 'dry, close-miked, "in your ear"', hi: 'bigger, echoey space (garage, tunnel)' },
  { id: 'comp', group: 'MIX', code: 'COMP', name: 'Compressor', min: 0, max: 1, def: 0.6, fmt: pct,
    body: 'Evens out loudness so an idling engine is not buried under a screaming one.',
    lo: 'natural dynamics: idle is quiet, revs are loud', hi: 'everything close to the same loudness, more "in-your-face" idle' },
  { id: 'drive', group: 'MIX', code: 'DRIVE', name: 'Saturation / drive', min: 0, max: 1, def: 0, fmt: pct,
    body: 'Soft-clips the signal, adding harmonics.',
    lo: 'clean', hi: 'warmer, then gritty, then distorted and aggressive' },
  // ---- TONE
  { id: 'lf', group: 'TONE', code: '+LF', name: 'Bass (low-frequency gain)', min: -12, max: 12, def: 0, bipolar: true, fmt: dB,
    body: 'Boosts or cuts the deep thump below about 160 Hz: the part you feel in your chest.',
    lo: 'thinner, less boom (good for laptop speakers)', hi: 'fatter, more boom (good on headphones / subwoofer)' },
  { id: 'hf', group: 'TONE', code: '+HF', name: 'Treble (high-frequency gain)', min: -12, max: 12, def: 0, bipolar: true, fmt: dB,
    body: 'Boosts or cuts everything above about 2.5 kHz: bite, edge and air.',
    lo: 'darker, softer, muffled', hi: 'brighter, crisper, more edge' },
  { id: 'ring', group: 'TONE', code: 'RING', name: 'Exhaust resonance', min: 0, max: 1, def: 0.6, fmt: pct,
    body: 'How long each exhaust pulse keeps ringing inside the pipes before it dies away.',
    lo: 'dead, short "thump" like a stuffed muffler', hi: 'long, tonal, droning like an open straight-pipe' },
  { id: 'rasp', group: 'TONE', code: 'RASP', name: 'Exhaust edge / rasp', min: 0, max: 1, def: 0.5, fmt: pct,
    body: 'How much of the sharp pressure edge of each firing is radiated from the tailpipe tip.',
    lo: 'mellow and round', hi: 'raspy, crackly, race-car' },
  // ---- CHARACTER
  { id: 'lope', group: 'CHARACTER', code: 'LOPE', name: 'Idle lope / unevenness', min: 0, max: 1, def: 0.5, fmt: pct,
    body: 'How uneven the cylinders and crank speed are from one firing to the next.',
    lo: 'perfectly smooth, even', hi: 'lumpy "cammed" idle, more uneven note' },
  { id: 'nlf', group: 'CHARACTER', code: '~LF', name: 'Low-frequency noise (rumble)', min: 0, max: 1, def: 0, fmt: pct,
    body: 'Adds deep rumble: turbulent gas flow and chassis vibration. Grows with rpm and load.',
    lo: 'none (default)', hi: 'heavy, subsonic rumble' },
  { id: 'nhf', group: 'CHARACTER', code: '~HF', name: 'High-frequency noise (air)', min: 0, max: 1, def: 0, fmt: pct,
    body: 'Adds a steady rush of air and hiss on top of the engine. Grows with rpm and throttle.',
    lo: 'none (default)', hi: 'airy, hissy, "wind" texture' },
  { id: 'tick', group: 'CHARACTER', code: 'TICK', name: 'Valvetrain tick', min: 0, max: 1, def: 0, fmt: pct,
    body: 'Mechanical clatter of valves and lifters, once per valve event.',
    lo: 'none (default)', hi: 'loud tappety, mechanical, old-engine sound' },
  { id: 'intk', group: 'CHARACTER', code: 'INTK', name: 'Intake sound', min: 0, max: 1, def: 0, fmt: pct,
    body: 'Level of the induction roar and the suction pulses of the intake valves: the whoosh when you open the throttle.',
    lo: 'subtle (default)', hi: 'loud induction roar, "big carb" sound' },
  { id: 'pops', group: 'CHARACTER', code: 'POPS', name: 'Overrun pops & crackle', min: 0, max: 1, def: 0.5, fmt: pct,
    body: 'How often the exhaust pops when you lift off the throttle at speed (unburnt fuel igniting in the hot pipes).',
    lo: 'off', hi: 'frequent pops and crackles', foot: 'Key P switches pops on / off' },
];
const PROFILE_BY_ID = Object.fromEntries(PROFILE.map((p) => [p.id, p]));
const profileDefaults = () => Object.fromEntries(PROFILE.map((p) => [p.id, p.def]));

// Presets list only what differs from the defaults.
const SOUND_PRESETS = [
  { id: 'stock', name: 'Stock', desc: 'The default sound.', v: {} },
  { id: 'muscle', name: 'Muscle', desc: 'Fat bass, long ring and a lumpy idle.',
    v: { lf: 5, hf: -1, ring: 0.7, rasp: 0.55, lope: 0.8, comp: 0.7, drive: 0.15, nlf: 0.25, conv: 0.45 } },
  { id: 'track', name: 'Track', desc: 'Bright, raspy and even; a race engine.',
    v: { lf: -2, hf: 4, ring: 0.5, rasp: 0.85, lope: 0.1, comp: 0.8, tick: 0.15, drive: 0.1, intk: 0.3, conv: 0.2 } },
  { id: 'smooth', name: 'Smooth', desc: 'Soft, quiet and refined.',
    v: { vol: 0.55, lf: 2, hf: -4, ring: 0.45, rasp: 0.25, lope: 0.05, comp: 0.5, conv: 0.4 } },
  { id: 'vintage', name: 'Vintage', desc: 'Warm and gritty, with tappets and room.',
    v: { lf: 3, hf: -6, ring: 0.7, rasp: 0.4, lope: 0.7, drive: 0.45, comp: 0.8, conv: 0.6, tick: 0.3 } },
  { id: 'rumble', name: 'Rumble', desc: 'Subsonic thunder for headphones and subs.',
    v: { lf: 9, hf: -2, ring: 0.8, rasp: 0.35, lope: 0.9, nlf: 0.8, comp: 0.7, conv: 0.5 } },
];

// ------------------------------------------------------------------ rotary knob
// Drag up/down (Shift = fine), wheel, arrow keys when focused, double-click to reset.
const KNOB_A0 = 135, KNOB_SWEEP = 270;   // degrees; 0° = 3 o'clock, clockwise (SVG)

class Knob {
  constructor(spec, value, onChange) {
    this.spec = spec; this.value = value; this.onChange = onChange;
    const root = this.root = document.createElement('div');
    root.className = 'knob'; root.tabIndex = 0;
    root.setAttribute('role', 'slider'); root.setAttribute('aria-label', spec.name);
    root.innerHTML = `
      <div class="knob-code">${spec.code}</div>
      <svg viewBox="0 0 64 64" aria-hidden="true">
        <path class="k-track"/><path class="k-fill"/><circle class="k-def" r="1.8"/>
        <circle class="k-body" cx="32" cy="32" r="18.5"/><line class="k-ptr"/>
      </svg>
      <div class="knob-val"></div>`;
    this.elTrack = root.querySelector('.k-track'); this.elFill = root.querySelector('.k-fill');
    this.elDef = root.querySelector('.k-def'); this.elPtr = root.querySelector('.k-ptr');
    this.elVal = root.querySelector('.knob-val');
    this.elTrack.setAttribute('d', this.arc(KNOB_A0, KNOB_A0 + KNOB_SWEEP));
    const da = this.angle(spec.def), dp = this.pt(da, 26);
    this.elDef.setAttribute('cx', dp[0]); this.elDef.setAttribute('cy', dp[1]);
    this.render();

    Tip.bind(root, () => this.tip());
    root.addEventListener('pointerdown', (e) => this.down(e));
    root.addEventListener('dblclick', () => this.set(spec.def, true));
    root.addEventListener('wheel', (e) => {
      e.preventDefault(); e.stopPropagation();
      this.set(this.value - Math.sign(e.deltaY) * this.range() * (e.shiftKey ? 0.005 : 0.02), true);
    }, { passive: false });
    root.addEventListener('keydown', (e) => this.key(e));
  }
  range() { return this.spec.max - this.spec.min; }
  angle(v) { return KNOB_A0 + KNOB_SWEEP * (v - this.spec.min) / this.range(); }
  pt(a, r) { const t = a * Math.PI / 180; return [32 + r * Math.cos(t), 32 + r * Math.sin(t)]; }
  arc(a0, a1, r = 26) {
    if (Math.abs(a1 - a0) < 0.5) return '';
    const p0 = this.pt(a0, r), p1 = this.pt(a1, r), lo = a1 - a0;
    return `M${p0[0].toFixed(2)} ${p0[1].toFixed(2)}A${r} ${r} 0 ${Math.abs(lo) > 180 ? 1 : 0} ${lo > 0 ? 1 : 0} ${p1[0].toFixed(2)} ${p1[1].toFixed(2)}`;
  }
  render() {
    const s = this.spec, a = this.angle(this.value);
    const from = s.bipolar ? this.angle(0) : KNOB_A0;
    this.elFill.setAttribute('d', this.arc(from, a));
    const p0 = this.pt(a, 7), p1 = this.pt(a, 16);
    this.elPtr.setAttribute('x1', p0[0]); this.elPtr.setAttribute('y1', p0[1]);
    this.elPtr.setAttribute('x2', p1[0]); this.elPtr.setAttribute('y2', p1[1]);
    this.elVal.textContent = s.fmt(this.value);
    this.root.classList.toggle('is-default', Math.abs(this.value - s.def) < this.range() * 0.004);
    this.root.setAttribute('aria-valuemin', s.min); this.root.setAttribute('aria-valuemax', s.max);
    this.root.setAttribute('aria-valuenow', this.value.toFixed(3)); this.root.setAttribute('aria-valuetext', s.fmt(this.value));
  }
  set(v, notify) {
    v = Math.max(this.spec.min, Math.min(this.spec.max, v));
    if (Math.abs(this.spec.def - v) < this.range() * 0.006) v = this.spec.def;    // sticky default
    if (v === this.value) return;
    this.value = v; this.render(); Tip.refresh(this.root);
    if (notify && this.onChange) this.onChange(this.spec.id, v);
  }
  setSilent(v) { this.value = Math.max(this.spec.min, Math.min(this.spec.max, v)); this.render(); Tip.refresh(this.root); }
  down(e) {
    if (e.button !== 0) return;
    e.preventDefault();
    const y0 = e.clientY, v0 = this.value, el = this.root;
    el.setPointerCapture(e.pointerId); el.classList.add('dragging');
    const move = (m) => this.set(v0 + (y0 - m.clientY) / (m.shiftKey ? 900 : 170) * this.range(), true);
    const up = () => {
      el.classList.remove('dragging');
      el.removeEventListener('pointermove', move); el.removeEventListener('pointerup', up);
      el.removeEventListener('pointercancel', up);
      el.blur();                                   // hand the keyboard back to driving
    };
    el.addEventListener('pointermove', move); el.addEventListener('pointerup', up); el.addEventListener('pointercancel', up);
  }
  key(e) {
    const step = this.range() / 50;
    const keys = {
      ArrowUp: step, ArrowRight: step, ArrowDown: -step, ArrowLeft: -step,
      PageUp: step * 5, PageDown: -step * 5,
    };
    if (e.key in keys) { this.set(this.value + keys[e.key], true); }
    else if (e.key === 'Home') this.set(this.spec.min, true);
    else if (e.key === 'End') this.set(this.spec.max, true);
    else if (e.key === 'Enter' || e.key === ' ') this.set(this.spec.def, true);
    else return;
    e.preventDefault(); e.stopPropagation();
  }
  tip() {
    const s = this.spec;
    return {
      title: `${s.code}: ${s.name}`, body: s.body,
      lines: [['Turn down:', s.lo], ['Turn up:', s.hi], ['Now:', `${s.fmt(this.value)}  (default ${s.fmt(s.def)})`]],
      foot: s.foot || 'Drag up/down (Shift = fine) · wheel · double-click to reset',
    };
  }
}
