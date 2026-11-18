// Purpose: measure how much the hopper's stack flexes and rings: the distance from its highest node
// to its lowest node (the pod apex to the engine tail) over a stand and a full-throttle lift-off.
// Value: the numbers milestone 5 needs to judge "a tall rocket wobbles less with rigid segments".
// Usage: node --experimental-strip-types scripts/hopper-wobble.mjs
import { readFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';

const root = join(dirname(fileURLToPath(import.meta.url)), '..');
const src = (name) => import(pathToFileURL(join(root, 'src', name)).href);
const { Core } = await src('core.ts');
const { World } = await src('world.ts');
const { CommandBatch } = await src('commands.ts');
const { Recorder } = await src('recording.ts');
const { scenarioFor } = await src('craft-page.ts');
const { flightCommands } = await src('flight.ts');

const core = await Core.load(new Uint8Array(readFileSync(join(root, 'target', 'wasm32-unknown-unknown', 'release', 'beam_core.wasm'))));
const craftText = readFileSync(join(root, 'test', 'fixtures', 'hopper.craft.json'), 'utf8');

for (const [name, throttle] of [['stand', 0], ['full throttle', 1]]) {
  const scenario = scenarioFor('fly', craftText);
  const world = new World(core);
  const rec = new Recorder(world);
  rec.setup(scenario.flight.setup);
  const p0 = world.snapshot().positions;
  let hi = 0, lo = 0;
  for (let i = 0; i < world.nodeCount; i++) {
    if (p0[3 * i + 1] > p0[3 * hi + 1]) hi = i;
    if (p0[3 * i + 1] < p0[3 * lo + 1]) lo = i;
  }
  const len = (p) => Math.hypot(p[3 * hi] - p[3 * lo], p[3 * hi + 1] - p[3 * lo + 1], p[3 * hi + 2] - p[3 * lo + 2]);
  const l0 = len(p0);
  let minL = l0, maxL = l0, maxLate = 0, maxLateSeen = 0;
  const steps = 600;
  for (let s = 0; s < steps; s++) {
    const b = new CommandBatch();
    if (throttle > 0) flightCommands(b, scenario.flight, [{ step: 0, throttle }], s);
    rec.step(b);
    const l = len(world.snapshot().positions);
    minL = Math.min(minL, l); maxL = Math.max(maxL, l);
    if (s >= steps / 2) maxLate = Math.max(maxLate, Math.abs(l - l0));
    maxLateSeen = Math.max(maxLateSeen, Math.abs(l - l0));
  }
  console.log(`${name}: rest length ${l0.toFixed(4)} m, range ${minL.toFixed(4)} to ${maxL.toFixed(4)} m (${(100 * (minL - l0) / l0).toFixed(3)}% to ${(100 * (maxL - l0) / l0).toFixed(3)}%), worst change in the second half ${(maxLate * 1000).toFixed(2)} mm`);
  world.free();
}
