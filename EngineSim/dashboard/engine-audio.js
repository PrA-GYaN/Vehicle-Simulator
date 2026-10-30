// Engine sound synthesizer (AudioWorklet).
//
// Built from the events a real engine makes, driven by the simulated state
// streamed from Python (rpm, manifold pressure, combustion, fuel cut...):
//
//   * per cylinder, at exhaust-valve opening, placed by the real firing order
//     (a cross-plane V8's uneven per-bank spacing gives the burble):
//       - a BLOWDOWN pulse: short (the cylinder empties through a choked valve
//         in ~1-3 ms whatever the rpm) — the midrange "bark" of idle;
//       - a DISPLACEMENT pulse: the piston pushing the rest out over the
//         exhaust stroke (crank-angle-limited, low-frequency body);
//     both are made by kicking a 3-pole smoothing filter (gamma-shaped
//     response: zero value AND zero slope at onset), so no pulse can click;
//     turbulence noise is low-passed and rides on the pulse envelope;
//     strength ∝ cylinder pressure at EVO: high when firing, weak/negative
//     when motoring on fuel cut (why engine braking sounds hollow);
//   * per bank (true duals) the pulses drive a physical EXHAUST WAVEGUIDE:
//     header → mid pipe → muffler expansion chamber → tailpipe → open end.
//     Each section is a pair of travelling-wave delay lines (Kelly–Lochbaum);
//     junction reflections come from the cross-section area ratios, the open
//     end reflects inverted with high-frequency radiation loss. The pipe
//     ringing between firings is what turns a pulse train into an engine note;
//     (the muffler chamber's reflection uses an effective area A^0.65 — real mufflers are
//     perforated/absorptive — and part of each pulse bypasses the pipe: shell/port radiation;
//     together they keep the pipe's notches from erasing the V8 rumble or leaving a fixed hum)
//   * level follows physics: pulse strength ∝ load and (rpm/800)^0.6 (≈ 15-17 dB idle→WOT at the
//     limiter), jet noise through the valve grows with load^2 (the crack), and the output stage is a
//     gentle leveller + limiter, not a compressor that flattens idle and full throttle together;
//   * intake roar, throttle hiss (only with real airflow through a nearly
//     closed plate), valvetrain tick, starter whine + solenoid, road noise,
//     overrun pops, early reflections, gentle compressor, soft clip.
//
// Performance: no allocation in process(); no pulse pool (pulses are filter
// kicks); events found by walking a sorted event list.

const TAU = Math.PI * 2;
const COMP_KNEE = 1.5, COMP_KNEE_E = 0.5;
const COMP_G = 0.29, COMP_E0 = 4.0;                // leveller: gain at the reference level, reference level
const JET_A = 0.3, JET_G = 1.3, RAD_A = 0.33;   // exhaust brightness: jet-noise bandwidth / level, radiation band limit

class Biquad {
  constructor() { this.b0 = 1; this.b1 = 0; this.b2 = 0; this.a1 = 0; this.a2 = 0; this.z1 = 0; this.z2 = 0; }
  set(type, fc, q, sr, gainDb = 0) {
    const w = TAU * Math.min(fc, sr * 0.45) / sr, c = Math.cos(w), s = Math.sin(w), al = s / (2 * q);
    let b0, b1, b2, a0, a1, a2;
    if (type === 'lp') { b0 = (1 - c) / 2; b1 = 1 - c; b2 = b0; a0 = 1 + al; a1 = -2 * c; a2 = 1 - al; }
    else if (type === 'hp') { b0 = (1 + c) / 2; b1 = -(1 + c); b2 = b0; a0 = 1 + al; a1 = -2 * c; a2 = 1 - al; }
    else if (type === 'bp') { b0 = al; b1 = 0; b2 = -al; a0 = 1 + al; a1 = -2 * c; a2 = 1 - al; }
    else if (type === 'ls' || type === 'hs') { // RBJ shelving, slope S = 1
      const A = Math.pow(10, gainDb / 40), sq = 2 * Math.sqrt(A) * (s / 2) * Math.SQRT2;
      if (type === 'ls') {
        b0 = A * ((A + 1) - (A - 1) * c + sq); b1 = 2 * A * ((A - 1) - (A + 1) * c); b2 = A * ((A + 1) - (A - 1) * c - sq);
        a0 = (A + 1) + (A - 1) * c + sq; a1 = -2 * ((A - 1) + (A + 1) * c); a2 = (A + 1) + (A - 1) * c - sq;
      } else {
        b0 = A * ((A + 1) + (A - 1) * c + sq); b1 = -2 * A * ((A - 1) + (A + 1) * c); b2 = A * ((A + 1) + (A - 1) * c - sq);
        a0 = (A + 1) - (A - 1) * c + sq; a1 = 2 * ((A - 1) - (A + 1) * c); a2 = (A + 1) - (A - 1) * c - sq;
      }
    }
    else { // peaking EQ
      const A = Math.pow(10, gainDb / 40);
      b0 = 1 + al * A; b1 = -2 * c; b2 = 1 - al * A; a0 = 1 + al / A; a1 = -2 * c; a2 = 1 - al / A;
    }
    this.b0 = b0 / a0; this.b1 = b1 / a0; this.b2 = b2 / a0; this.a1 = a1 / a0; this.a2 = a2 / a0;
    return this;
  }
  process(x) { // transposed direct form II
    const y = this.b0 * x + this.z1;
    this.z1 = this.b1 * x - this.a1 * y + this.z2;
    this.z2 = this.b2 * x - this.a2 * y;
    return y;
  }
}

// Tube with a lossy reflecting end: y[n] = x[n] + g·LP(y[n-D])
class Pipe {
  constructor(sr, lengthM, soundSpeed, reflection, damping) {
    this.D = Math.max(2, Math.round(2 * lengthM / soundSpeed * sr));
    this.buf = new Float32Array(this.D);
    this.i = 0; this.g = reflection; this.a = damping; this.lp = 0;
  }
  process(x) {
    this.lp += this.a * (this.buf[this.i] - this.lp);
    const y = x + this.g * this.lp;
    this.buf[this.i] = y;
    if (++this.i === this.D) this.i = 0;
    return y;
  }
}

// Three cascaded one-poles: an impulse in gives t²·e^(−t/τ) out — a smooth,
// band-limited pulse. kick(A) makes a pulse of peak ≈ A.
class Shaper {
  constructor() { this.a = 0.1; this.y1 = 0; this.y2 = 0; this.y3 = 0; this.norm = 1; this.in = 0; }
  setTau(tauS, sr) {
    this.a = 1 - Math.exp(-1 / Math.max(1, tauS * sr));
    this.norm = 1 / (this.a * 0.2707);            // peak of the 3-pole impulse response ≈ 0.2707·a
  }
  kick(A) { this.in += A * this.norm; }
  process() {
    const a = this.a;
    this.y1 += a * (this.in - this.y1); this.in = 0;
    this.y2 += a * (this.y1 - this.y2);
    this.y3 += a * (this.y2 - this.y3);
    return this.y3;
  }
}

class Delay {
  constructor(n) { this.n = Math.max(1, n | 0); this.b = new Float32Array(this.n); this.i = 0; }
  out() { return this.b[this.i]; }
  push(x) { this.b[this.i] = x; if (++this.i === this.n) this.i = 0; }
}

// One exhaust pipe as a chain of uniform tube sections (pressure waves). The
// sections come from the engine file: {length_m, c (speed of sound), area}.
class ExhaustPipe {
  constructor(sr, pipe, loss, tipDamping) {
    const sec = pipe.sections;
    this.S = sec.length;
    this.F = sec.map((x) => new Delay(Math.round(x.length_m / x.c * sr)));   // toward the tip
    this.B = sec.map((x) => new Delay(Math.round(x.length_m / x.c * sr)));   // back toward the valves
    this.K = new Float32Array(this.S - 1);                                    // junction reflection
    // Junction reflection from the area ratio. A plain expansion chamber of ratio 6 has deep stop-bands
    // (−15 dB ripple) that notch out the V8's 90–200 Hz rumble and leave a fixed-pitch honk; real
    // mufflers are perforated / absorptive, so the effective ratio is compressed (A^0.65).
    const ae = sec.map((x) => Math.pow(x.area, 0.65));
    for (let j = 0; j < this.S - 1; j++) this.K[j] = (ae[j] - ae[j + 1]) / (ae[j] + ae[j + 1]);
    this.fo = new Float32Array(this.S); this.bo = new Float32Array(this.S);
    this.loss = loss;             // wall friction per pass
    this.rValve = 0.7;            // valve/port end: mostly closed → positive reflection  (RING dial)
    this.rOpen = -0.9;            // open tip: inverted reflection …                      (RING dial)
    this.aOpen = tipDamping;      // … losing highs to radiation
    this.lp = 0; this.prevOut = 0;
  }
  process(src) {
    const S = this.S, F = this.F, B = this.B, K = this.K, fo = this.fo, bo = this.bo, g = this.loss;
    for (let j = 0; j < S; j++) { fo[j] = F[j].out() * g; bo[j] = B[j].out() * g; }
    F[0].push(src + this.rValve * bo[0]);
    for (let j = 0; j < S - 1; j++) {
      const k = K[j];
      F[j + 1].push((1 + k) * fo[j] - k * bo[j + 1]);
      B[j].push(k * fo[j] + (1 - k) * bo[j + 1]);
    }
    const tip = fo[S - 1];
    this.lp += this.aOpen * (tip - this.lp);
    B[S - 1].push(this.rOpen * this.lp);
    const out = tip + this.rOpen * this.lp;       // transmitted (radiated) part
    return out;
  }
}

// Small Freeverb-style room tail: 4 damped combs + 2 allpasses per side.
class Reverb {
  constructor(sr) {
    const k = sr / 44100;
    const mk = (n) => ({ b: new Float32Array(Math.max(2, Math.round(n * k))), i: 0, f: 0 });
    this.cL = [1116, 1188, 1277, 1356].map(mk);
    this.cR = [1139, 1211, 1300, 1379].map(mk);
    this.aL = [556, 441].map(mk); this.aR = [579, 464].map(mk);
    this.fb = 0.80; this.damp = 0.35;
    this.l = 0; this.r = 0;
  }
  side(x, combs, aps) {
    let sum = 0;
    for (let j = 0; j < 4; j++) {
      const c = combs[j], y = c.b[c.i];
      c.f += this.damp * (y - c.f);                // damping: low-pass in the feedback path
      c.b[c.i] = x + this.fb * c.f;
      if (++c.i === c.b.length) c.i = 0;
      sum += y;
    }
    sum *= 0.25;
    for (let j = 0; j < 2; j++) {
      const a = aps[j], y = a.b[a.i];
      const out = y - sum;
      a.b[a.i] = sum + 0.5 * y;
      if (++a.i === a.b.length) a.i = 0;
      sum = out;
    }
    return sum;
  }
  process(x) { this.l = this.side(x, this.cL, this.aL); this.r = this.side(x, this.cR, this.aR); }
}

// User-adjustable sound profile (see README.md §6.6). lf / hf are dB, the rest 0..1.
const PROFILE_DEFAULT = { vol: 0.7, conv: 0.35, comp: 0.6, drive: 0, lf: 0, hf: 0, ring: 0.6,
                          rasp: 0.5, lope: 0.5, nlf: 0, nhf: 0, tick: 0, intk: 0, pops: 0.5 };

class EngineSynth extends AudioWorkletProcessor {
  constructor() {
    super();
    const sr = this.sr = sampleRate;
    this.s = { rpm: 0, map: 1, throttle: 0, combusting: false, fuelCut: false, starter: false,
               ignition: false, speed: 0, gear: 0, load: 0, pops: true, muted: false,
               redline: 6000, idle: 800, indicated: 0, friction: 0, pumping: 0 };
    this.rpm = 0; this.map = 1; this.throttle = 0;
    this.seed = 22222; this.seed2 = 987654321;
    this.pf = Object.assign({}, PROFILE_DEFAULT);

    this.noiseLP = 0; this.noiseLP2 = 0; this.jetLP = 0; this.jetLP2 = 0; this.wander = 0; this.rumLP = 0; this.rumLP2 = 0;
    this.intakeShaper = new Shaper();
    this.intakeShaper.setTau(0.0015, sr);

    this.fuelCutT = 99; this.starterPrev = false; this.solenoid = 0; this.solenoidPhase = 0;
    this.starterPhase = 0; this.whinePhase = 0; this.tickEnv = 0;
    this.compEnv = 0.1; this.compGain = 0.2;
    this.levelAcc = 0; this.levelN = 0; this.peak = 0; this.intakeAcc = 0; this.hissAcc = 0;

    this.dcL = new Biquad().set('hp', 25, 0.7, sr);
    this.dcR = new Biquad().set('hp', 25, 0.7, sr);
    this.intakeBP = new Biquad();
    this.intakeLP = new Biquad().set('lp', 1200, 0.7, sr);
    this.hissHP = new Biquad().set('hp', 2200, 0.7, sr);
    this.hissLP = new Biquad().set('lp', 6500, 0.7, sr);
    this.roadLP = new Biquad().set('lp', 380, 0.7, sr);
    this.starterBP = new Biquad();
    this.tickBP = new Biquad().set('bp', 2300, 3, sr);
    this.outLPL = new Biquad().set('lp', 7000, 0.7, sr);
    this.outLPR = new Biquad().set('lp', 7000, 0.7, sr);
    this.lsL = new Biquad(); this.lsR = new Biquad();       // bass shelf
    this.hsL = new Biquad(); this.hsR = new Biquad();       // treble shelf
    this.reverb = new Reverb(sr);

    // early reflections: flat tap tables
    this.erLen = 2048;
    this.erBuf = new Float32Array(this.erLen);
    this.erI = 0;
    this.erD = Int32Array.from([0.0071, 0.0133, 0.0197, 0.0293], (t) => Math.round(t * sr));
    this.erGL = Float32Array.from([0.20, 0.11, 0.13, 0.06]);
    this.erGR = Float32Array.from([0.11, 0.18, 0.08, 0.10]);

    this.sp = null;              // engine definition (eng.sound); silent until one arrives
    this.tqInd = 0; this.tqFric = 0; this.tqPump = 0;

    this.port.onmessage = (e) => {
      const m = e.data;
      if (m.type === 'state') Object.assign(this.s, m.state);
      else if (m.type === 'engine') this.configure(m.engine);
      else if (m.type === 'profile') this.applyProfile(m.profile);
    };
  }

  // Build everything engine-specific from the engine file's `sound` block
  // (python: engine_sim/sound_spec.py). Nothing about a particular engine lives here.
  configure(eng) {
    const sp = this.sp = eng.sound, sr = this.sr, n = sp.cylinders;
    this.n = n;
    this.cylPipe = Uint8Array.from(sp.exhaust.cylinder_pipe);      // which exhaust pipe each cylinder feeds
    this.pipes = sp.exhaust.pipes.length;
    this.ex = sp.exhaust.pipes.map((pipe) => ({
      bd: new Shaper(), disp: new Shaper(),                        // pulse generators
      noise: 0,
      pipe: new ExhaustPipe(sr, pipe, sp.exhaust.pipe_loss, sp.exhaust.tip_damping),
      body: new Biquad().set('peak', pipe.body_hz, 0.9, sr, 4),    // muffler shell / body
      prev: 0, radLP: 0,
    }));
    this.pan = this.pipes > 1 ? [[0.95, 0.3], [0.3, 0.95]] : [[0.8, 0.8]];
    this.intakeRunner = new Pipe(sr, sp.intake.runner_length_m, sp.intake.runner_c, -0.55, 0.35);
    // Fixed per-cylinder differences (runner length, trim, ring seal). They repeat every
    // cycle, so they add sub-harmonics: the lumpy idle "lope".
    let seed = sp.seed;
    const rnd = () => { seed = (seed * 1103515245 + 12345) % 2147483648; return seed / 2147483648 - 0.5; };
    this.cylGain = Float32Array.from({ length: n }, () => 1 + sp.cylinder_strength_spread * rnd());
    // Sorted event list: exhaust open (type 0) and intake open (type 1), at the valve
    // timing given in the engine file, relative to each cylinder's firing TDC.
    const ev = [];
    for (let c = 0; c < n; c++) {
      ev.push([(sp.fire_angle_deg[c] + sp.evo_after_fire_deg + 720) % 720, 0, c, sp.cylinder_timing_spread_deg * rnd()]);
      ev.push([(sp.fire_angle_deg[c] + sp.ivo_after_fire_deg + 720) % 720, 1, c, 0]);
    }
    ev.sort((a, b) => a[0] - b[0]);
    this.evA = Float64Array.from(ev, (e) => e[0]);
    this.evT = Uint8Array.from(ev, (e) => e[1]);
    this.evC = Uint8Array.from(ev, (e) => e[2]);
    this.evO = Float64Array.from(ev, (e) => e[3]);           // per-event timing offset [deg]
    this.E = ev.length;
    this.theta = 0; this.next = 0; this.nextBase = this.evA[0];
    this.applyProfile();
    this.nextA = this.nextBase + this.evO[0] * this.lopeS;
  }

  // Turn the user profile into the scalars / filters the audio loop uses.
  applyProfile(p) {
    if (p) for (const k in p) if (k in PROFILE_DEFAULT && Number.isFinite(p[k])) this.pf[k] = p[k];
    const f = this.pf, sr = this.sr;
    this.erScale = Math.min(3, f.conv / 0.35);
    this.revWet = 0.8 * Math.max(0, f.conv - 0.35) ** 2;         // tail only above the default room
    this.shelfLow = Math.abs(f.lf) > 0.05; this.shelfHigh = Math.abs(f.hf) > 0.05;
    this.lsL.set('ls', 160, 1, sr, f.lf); this.lsR.set('ls', 160, 1, sr, f.lf);
    this.hsL.set('hs', 2500, 1, sr, f.hf); this.hsR.set('hs', 2500, 1, sr, f.hf);
    const lpHz = 7000 * (1 + Math.max(0, f.hf) / 12 * 0.7);      // let boosted treble through
    this.outLPL.set('lp', lpHz, 0.7, sr); this.outLPR.set('lp', lpHz, 0.7, sr);
    if (this.ex) for (const e of this.ex) {                        // exhaust ring: pipe reflections
      e.pipe.rValve = 0.15 + 0.5 * f.ring;                         // loop gain 0.34 at the default: pipe modes
      e.pipe.rOpen = -(0.45 + 0.5 * f.ring);                        // are broad (Q ≈ 4), not a fixed-pitch hum
    }
    this.radW = 8.8 * f.rasp * f.rasp;                              // 2.2 at the default 0.5
    this.lopeS = Math.pow(f.lope / 0.5, 1.5);                        // 1 at default, ~2.8 at max
    this.nlf = f.nlf; this.nhf = f.nhf;                              // additive noise sources
    this.tickA = f.tick * 1.5;                                       // extra valve-tick per event
    this.intkS = 1 + 11 * f.intk;                                    // intake level multiplier
    this.popsS = f.pops * 2;
    this.compExp = 0.2 * f.comp / 0.6;                               // leveller strength: 0 = flat gain, 0.2 at the default
    this.driveK = 1 + 4 * f.drive;
  }

  noise() { // xorshift32 → [-1, 1)
    let x = this.seed;
    x ^= x << 13; x ^= x >>> 17; x ^= x << 5;
    this.seed = x >>> 0;
    return this.seed / 2147483648 - 1;
  }
  gauss() { return (this.noise() + this.noise() + this.noise()) * 0.577; }

  // Exhaust valve opens on cylinder c: kick that bank's pulse shapers
  exhaustEvent(c) {
    const s = this.s, rpm = Math.max(this.rpm, 30), map = this.map;
    const e = this.ex[this.cylPipe[c]];
    // A real exhaust gets much louder with speed (more, faster flow) and with load (higher
    // blowdown pressure): ~20 dB from idle to full throttle at the limiter, not 5.
    const rpmAmp = Math.pow(Math.min(Math.max(rpm, 300), 9000) / 800, 0.6);
    let bd, disp, noiseAmt;
    if (s.combusting) {
      // p_EVO − p_exh: grows with charge; light load is a soft "chuff", full load a hard crack
      const cov = (this.sp.variability.base + this.sp.variability.idle * (1 - map)) * Math.min(2, this.lopeS);   // cycle-to-cycle strength
      const k = (1 + (this.cylGain[c] - 1) * this.lopeS) * Math.max(0.25, 1 + cov * this.gauss()) * rpmAmp;
      // boost adds some noise but a turbo engine is not 3x louder: count it at a quarter
      const load = Math.pow(Math.min(map, 1) + 0.25 * Math.max(0, Math.min(map, 2.5) - 1), 1.2);
      bd = (0.2 + 0.8 * load) * k;
      disp = (0.08 + 0.2 * load) * k;
      noiseAmt = 0.15 + 0.85 * load * load;     // jet noise through the valve: grows steeply with load
    } else {
      // Motoring (fuel cut / key off): the cylinder holds only the expanded,
      // sub-atmospheric charge, so when the exhaust valve opens gas rushes
      // back IN (reversion) — a strong negative pulse at high vacuum; this is
      // the hollow note of engine braking
      bd = 0.5 * (map - 0.45) * rpmAmp; disp = 0.08 * rpmAmp; noiseAmt = 0.05;
      if (s.pops && s.ignition && s.fuelCut && this.fuelCutT < 2.0 && rpm > 1800 &&
          (this.noise() + 1) * 0.5 < this.sp.pop_probability * this.popsS) {       // overrun pop
        bd = (1.4 + 0.4 * (this.noise() + 1)) * rpmAmp; noiseAmt = 1.5;
      }
    }
    e.bd.kick(bd);
    e.disp.kick(disp);
    e.noise = noiseAmt;
    this.tickEnv += 0.0007 * Math.min(1, rpm / 3000) + this.tickA * Math.min(2.5, rpm / 1200);   // valve seating tick
  }

  process(inputs, outputs) {
    const out = outputs[0];
    const L = out[0], R = out[1] || out[0], N = L.length;
    if (!this.sp) { L.fill(0); if (R !== L) R.fill(0); return true; }     // no engine yet
    const s = this.s, sr = this.sr, sp = this.sp;
    const kRpm = 1 - Math.exp(-1 / (0.03 * sr));
    const kMap = 1 - Math.exp(-1 / (0.02 * sr));
    const kTick = Math.exp(-1 / (0.0008 * sr));
    const kSol = Math.exp(-1 / (0.025 * sr));
    const kCompA = 1 - Math.exp(-1 / (0.005 * sr)), kCompR = 1 - Math.exp(-1 / (0.25 * sr));
    this.intakeBP.set('bp', 250 + this.rpm * 0.12, 1.1, sr);
    {
      // blowdown: ~0.35-0.6 ms time constant (peak at 2τ); displacement: a
      // quarter of the exhaust stroke duration, so it spans the stroke
      // Blowdown pulse: the initial choked burst through the opening valve is
      // sub-millisecond; the spectrum of this 3-pole pulse falls 18 dB/oct
      // above 1/(2πτ) ≈ 450 Hz, keeping the 125-500 Hz "blat" that the ear
      // actually hears at idle (τ = 1.4 ms put the corner at 110 Hz and left
      // only an inaudible 30-60 Hz throb).
      const rpmC = Math.max(this.rpm, 60);
      const tauBd = Math.min(sp.blowdown_tau_max_s, 60 / (rpmC * 6) / 6);
      const tauDisp = Math.max(0.002, 180 / (rpmC * 6) / 4.5);
      for (let b = 0; b < this.pipes; b++) { this.ex[b].bd.setTau(tauBd, sr); this.ex[b].disp.setTau(tauDisp, sr); }
    }
    this.starterBP.set('bp', 180 + this.rpm * 1.2, 2.0, sr);
    if (s.fuelCut) this.fuelCutT += N / sr; else this.fuelCutT = 0;
    if (s.starter && !this.starterPrev) this.solenoid = 1;
    if (!s.starter && this.starterPrev) this.solenoid = 0.5;
    this.starterPrev = s.starter;
    const vol = s.muted ? 0 : this.pf.vol;
    const lopeS = this.lopeS, nlf = this.nlf, nhf = this.nhf, intkS = this.intkS, radW = this.radW;
    const shelfLow = this.shelfLow, shelfHigh = this.shelfHigh, erScale = this.erScale;
    const revWet = this.revWet, driveK = this.driveK, driveN = 1 / Math.sqrt(this.driveK);
    const redline = s.redline || 6000;
    const banks = this.pipes, ex = this.ex, pan = this.pan;
    const moving = s.speed > 0.5, v = s.speed / 2.237;
    const roadAmt = moving ? Math.min(1, (v / 35) * (v / 35)) * 0.3 : 0;
    const whineAmt = moving && s.gear > 0 ? 0.006 * (0.3 + Math.abs(s.load)) : 0;
    const starterOn = s.starter;
    let sumSq = 0, pk = 0, intakeLvl = 0, hissLvl = 0;
    let bank0 = 0, bank1 = 0;
    // steady turbulent flow noise ∝ exhaust mass flow (rpm × charge): the roar under load.
    // It is broadband noise that sounds like a constant "swish" of air while the throttle is open,
    // so it is now off by default and comes in with the ~HF (air) dial; the charge is clamped at
    // ambient so a boosted engine does not make it louder than a naturally aspirated one.
    const flowRoar = 0.5 * nhf * Math.min(1, this.rpm / redline) * (s.combusting ? Math.min(1, this.map) : 0.2);
    // Crank-speed ripple from the physics: each firing delivers W_cyl = T_ind·4π/N of work into the
    // flywheel, giving a speed swing δ ≈ g·W_cyl/(I·ω²). A heavy flywheel (large I) or many
    // cylinders idles smoothly; a light one or few cylinders lopes.
    const kT = 1 - Math.exp(-N / (0.03 * sr));
    this.tqInd += (s.indicated - this.tqInd) * kT; this.tqFric += (s.friction - this.tqFric) * kT; this.tqPump += (s.pumping - this.tqPump) * kT;
    const om = this.rpm * (Math.PI / 30);
    let rippleBlk = 0;
    if (this.rpm > 20) {
      const tq = s.combusting ? this.tqInd : 0.35 * (this.tqFric + this.tqPump);   // motoring: compression asymmetry only
      const Wcyl = Math.max(0, tq) * 4 * Math.PI / this.n;
      rippleBlk = Math.min(0.6, Math.min(0.25, sp.ripple_gain * Wcyl / (sp.inertia_kgm2 * om * om)) * lopeS);
    }
    // slow random rpm wander (real engines never hold a perfectly steady speed)
    this.wander += (this.gauss() - this.wander) * (1 - Math.exp(-N / (0.25 * sr)));
    const wanderK = 1 + this.wander * 0.0015;
    // compressor gain: evaluated per block from the level envelope
    // Gentle leveller, not a flattener: out ∝ env^(1−e). Idle and full throttle must stay ~20 dB
    // apart, as on a real car; the old env^0.5 law squeezed that to 5 dB. (at e = 0 the gain is a flat 2)
    const xe = (this.compEnv + 0.02) / COMP_E0;
    let g = COMP_G * Math.pow(xe, -this.compExp);
    if (xe > COMP_KNEE) g *= Math.pow(xe / COMP_KNEE, -COMP_KNEE_E);        // limiter above the knee: holds the peaks under the soft clip
    g = g < 0.02 ? 0.02 : g > 5 ? 5 : g;
    const gS = this.compGain, gE = gS + (g - gS) * (1 - Math.exp(-N / (0.08 * sr)));
    this.compGain = gE;
    const gStart = gS, gEnd = gE;

    for (let i = 0; i < N; i++) {
      this.rpm += kRpm * (s.rpm - this.rpm);
      this.map += kMap * (s.map - this.map);
      this.throttle += kMap * (s.throttle - this.throttle);
      const rpm = this.rpm, map = this.map;

      // --- crank angle with per-stroke speed ripple (idle lope) ---
      if (rpm > 1) {
        const ripple = rippleBlk;
        // speed ∝ 1 + a·sin(): a cycle then takes T0/√(1−a²), so divide by √(1−a²) to keep
        // the mean speed (and the pitch) exactly at the simulated rpm
        this.theta += rpm * wanderK * 6 / sr / Math.sqrt(1 - ripple * ripple)
                    * (1 + ripple * Math.sin(this.theta * TAU * this.n / 720));
        while (this.theta >= this.nextA) {                 // walk the sorted event list
          const e = this.next;
          if (this.evT[e] === 0) this.exhaustEvent(this.evC[e]);
          else this.intakeShaper.kick(-0.25 * (0.25 + this.throttle));
          const nx = e + 1 === this.E ? 0 : e + 1;
          let gap = this.evA[nx] - this.evA[e]; if (gap < 0) gap += 720;
          // No random per-event timing: irregularity must repeat every cycle
          // (fixed per-cylinder offsets, set in configure) to stay tonal —
          // random jitter smears the harmonics into broadband "wind".
          this.next = nx; this.nextBase += gap;
          this.nextA = this.nextBase + this.evO[nx] * lopeS;
        }
        if (this.theta >= 720) { this.theta -= 720; this.nextA -= 720; this.nextBase -= 720; }
      }

      // --- exhaust: shaped pulses → waveguide → radiation ---
      const nz = this.noise();
      this.noiseLP += 0.21 * (nz - this.noiseLP);                  // ~1.8 kHz turbulence (intake, valvetrain)
      this.noiseLP2 += 0.21 * (this.noiseLP - this.noiseLP2);
      this.jetLP += JET_A * (nz - this.jetLP); this.jetLP2 += JET_A * (this.jetLP - this.jetLP2);   // jet noise through the valve: wider band, 12 dB/oct roll-off
      const turb = this.noiseLP2 * 2.2, jet = this.jetLP2 * JET_G;
      let oL = 0, oR = 0;
      for (let b = 0; b < banks; b++) {
        const e = ex[b];
        const bdv = e.bd.process();
        const env = bdv < 0 ? -bdv : bdv;
        const disp = e.disp.process();
        const src = bdv + disp + turb * (env * e.noise * 0.12 + flowRoar * map * map) + jet * env * e.noise;
        const p = e.pipe.process(src);
        const rad = p - e.prev; e.prev = p;           // far-field radiation ∝ d/dt
        e.radLP += RAD_A * (rad - e.radLP);           // …band-limited: no sharp edges
        const y = e.body.process(p * 0.8 + e.radLP * radW + (bdv + disp) * 0.45) * 6.0;      // make-up gain; + radiation that bypasses the muffler
        oL += y * pan[b][0]; oR += y * pan[b][1];
      }

      // --- intake: runner pulses + roar with airflow ---
      const flow = Math.min(1.2, rpm / redline) * (0.15 + this.throttle);
      const roar = this.intakeBP.process(this.noiseLP) * flow * 0.2;
      const intake = (this.intakeLP.process(this.intakeRunner.process(this.intakeShaper.process() * (0.3 + map) + roar * 0.3)) * 0.6 + roar * 0.3) * intkS;
      intakeLvl += intake < 0 ? -intake : intake;
      // throttle hiss: needs real airflow through a nearly shut plate (decel at speed),
      // not idle — idle airflow is tiny
      const vac = map < 0.6 ? (0.6 - map) / 0.6 : 0;
      const rn = rpm / redline;
      // + ~HF: steady air rush (rpm, throttle) added on top; 0 by default
      const hissAmt = vac * rn * Math.sqrt(rn) * 0.02
                    + nhf * (0.1 + 3 * rn * rn) * (0.4 + 0.6 * this.throttle) * 0.5;
      let hiss = 0;
      if (hissAmt > 1e-4) { hiss = this.hissLP.process(this.hissHP.process(this.noise())) * hissAmt; hissLvl += hiss < 0 ? -hiss : hiss; }
      // ~LF: low-frequency rumble (gas flow / chassis), noise low-passed to ~110 Hz
      let rumble = 0;
      if (nlf > 0) {                      // own random stream: knobs never disturb the engine's noise sequence
        let x = this.seed2; x ^= x << 13; x ^= x >>> 17; x ^= x << 5; this.seed2 = x >>> 0;
        this.rumLP += 0.0143 * (this.seed2 / 2147483648 - 1 - this.rumLP); this.rumLP2 += 0.0143 * (this.rumLP - this.rumLP2);
        rumble = this.rumLP2 * nlf * 9 * (0.3 + 0.7 * rn) * (0.5 + 0.5 * map);
      }
      oL += intake + hiss + rumble; oR += intake + hiss + rumble;

      // --- valvetrain tick ---
      if (this.tickEnv > 1e-5) {
        const tk = this.tickBP.process(this.noiseLP * this.tickEnv) * 2.0;
        if (this.tickEnv > 3.5) this.tickEnv = 3.5;
        this.tickEnv *= kTick;
        oL += tk; oR += tk;
      }

      // --- starter motor ---
      if (starterOn || this.solenoid > 0.001) {
        let st = 0;
        if (starterOn) {
          const fArm = (rpm > 40 ? rpm : 40) * sp.starter.ratio / 60;      // armature speed from the starter gearing
          this.starterPhase += fArm * sp.starter.poles / sr; if (this.starterPhase >= 1) this.starterPhase -= 1;
          st = this.starterBP.process(this.starterPhase * 2 - 1 + 0.4 * this.noise()) * 0.35
             + Math.sin(TAU * this.starterPhase * 2) * 0.05;
        }
        if (this.solenoid > 0.001) {
          this.solenoidPhase += 70 / sr;
          st += Math.sin(TAU * this.solenoidPhase) * this.solenoid * 0.6 + this.noise() * this.solenoid * 0.2;
          this.solenoid *= kSol;
        }
        oL += st; oR += st;
      }

      // --- driveline whine + road/wind noise ---
      if (roadAmt > 0) {
        const road = this.roadLP.process(this.noise()) * roadAmt;
        let wh = 0;
        if (whineAmt > 0) {
          this.whinePhase += rpm / 60 * 17 / sr; if (this.whinePhase >= 1) this.whinePhase -= 1;
          wh = Math.sin(TAU * this.whinePhase) * whineAmt;
        }
        oL += road + wh; oR += road * 0.9 + wh;
      }

      // --- early reflections ---
      const er = this.erBuf, mask = this.erLen - 1, w = this.erI;
      er[w] = (oL + oR) * 0.5;
      const d = this.erD, gl = this.erGL, gr = this.erGR;
      let r0 = er[(w - d[0]) & mask], r1 = er[(w - d[1]) & mask], r2 = er[(w - d[2]) & mask], r3 = er[(w - d[3]) & mask];
      const dry = (oL + oR) * 0.5;
      oL += (r0 * gl[0] + r1 * gl[1] + r2 * gl[2] + r3 * gl[3]) * erScale;
      oR += (r0 * gr[0] + r1 * gr[1] + r2 * gr[2] + r3 * gr[3]) * erScale;
      this.erI = (w + 1) & mask;
      if (revWet > 1e-3) {
        this.reverb.process(dry);
        oL += this.reverb.l * revWet; oR += this.reverb.r * revWet;
      }

      // --- compressor (evens out idle vs WOT loudness), DC block, soft clip ---
      oL = this.outLPL.process(this.dcL.process(oL)); oR = this.outLPR.process(this.dcR.process(oR));
      if (shelfLow) { oL = this.lsL.process(oL); oR = this.lsR.process(oR); }
      if (shelfHigh) { oL = this.hsL.process(oL); oR = this.hsR.process(oR); }
      const lvl = (oL < 0 ? -oL : oL) + (oR < 0 ? -oR : oR);
      this.compEnv += (lvl > this.compEnv ? kCompA : kCompR) * (lvl - this.compEnv);
      const cg = (gStart + (gEnd - gStart) * i / N) * 0.8;
      oL = Math.tanh(oL * cg * driveK) * driveN * vol;
      oR = Math.tanh(oR * cg * driveK) * driveN * vol;
      L[i] = oL; if (R !== L) R[i] = oR;
      sumSq += oL * oL; const a = oL < 0 ? -oL : oL; if (a > pk) pk = a;
    }

    this.levelAcc += sumSq; this.levelN += N; if (pk > this.peak) this.peak = pk;
    this.intakeAcc += intakeLvl / N; this.hissAcc += hissLvl / N;
    if (this.levelN >= sr / 30) {
      const blocks = this.levelN / N;
      this.port.postMessage({ type: 'level', rms: Math.sqrt(this.levelAcc / this.levelN), peak: this.peak,
                              intake: this.intakeAcc / blocks, hiss: this.hissAcc / blocks });
      this.levelAcc = 0; this.levelN = 0; this.peak = 0; this.intakeAcc = 0; this.hissAcc = 0;
    }
    return true;
  }
}

registerProcessor('engine-synth', EngineSynth);
