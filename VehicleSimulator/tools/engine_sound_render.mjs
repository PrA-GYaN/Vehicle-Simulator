// Render EngineSim's sound worklet offline, under Node.
//
//   node tools/engine_sound_render.mjs <engine-audio.js> <spec.json> <states.csv> <out.f32>
//
// Not run by hand: tools/engine_sound_reference.py calls it. It loads
// ../EngineSim/dashboard/engine-audio.js UNCHANGED, stubs the three things an
// AudioWorklet global scope provides (sampleRate, AudioWorkletProcessor,
// registerProcessor), then feeds the processor exactly what the dashboard
// would: one `engine` message, one `profile` message, and before every
// 128-frame block one `state` message from the CSV. The output is raw
// interleaved stereo float32 - what the browser would have sent to the
// speakers - for build/bin/engine_sound_parity to compare against.
import fs from 'node:fs';
import vm from 'node:vm';

const [workletPath, specPath, statesPath, outPath] = process.argv.slice(2);
if (!outPath) {
  console.error('usage: node engine_sound_render.mjs <engine-audio.js> <spec.json> <states.csv> <out.f32>');
  process.exit(2);
}
const job = JSON.parse(fs.readFileSync(specPath, 'utf8'));     // { sampleRate, sound, profile }

let Processor = null;
globalThis.sampleRate = job.sampleRate;
globalThis.AudioWorkletProcessor = class { constructor() { this.port = { onmessage: null, postMessage() {} }; } };
globalThis.registerProcessor = (name, cls) => { Processor = cls; };
vm.runInThisContext(fs.readFileSync(workletPath, 'utf8'), { filename: workletPath });

const proc = new Processor();
const send = (data) => proc.port.onmessage({ data });
send({ type: 'engine', engine: { sound: job.sound } });
send({ type: 'profile', profile: job.profile });

const lines = fs.readFileSync(statesPath, 'utf8').trim().split('\n');
const cols = lines[0].split(',');
const N = 128;
const out = new Float32Array((lines.length - 1) * N * 2);
const L = new Float32Array(N), R = new Float32Array(N);
const flags = new Set(['combusting', 'fuelCut', 'starter', 'ignition']);
for (let b = 1; b < lines.length; b++) {
  const v = lines[b].split(',');
  const state = { speed: 0, gear: 0, load: 0, pops: true, muted: false };
  cols.forEach((c, k) => { state[c] = flags.has(c) ? v[k] === '1' : Number(v[k]); });
  send({ type: 'state', state });
  proc.process([], [[L, R]]);
  const o = (b - 1) * N * 2;
  for (let i = 0; i < N; i++) { out[o + 2 * i] = L[i]; out[o + 2 * i + 1] = R[i]; }
}
fs.writeFileSync(outPath, Buffer.from(out.buffer));
