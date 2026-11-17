// Purpose: find how heavy a truck has to be to break the 3D bridge, and where on the span it is when
// the first beam yields and the first beam breaks. Headless, in Node (WebAssembly core).
// Value: sizes the "overload" truck of src/scenes3d.ts with a measurement instead of a guess.
// Usage: node scripts/truck-capacity.mjs [mass_kg ...]   (default: a sweep, then a bisection)
import { readFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';

const root = join(dirname(fileURLToPath(import.meta.url)), '..');
const src = (name) => import(pathToFileURL(join(root, 'src', name)).href);
const { Core } = await src('core.ts');
const { World } = await src('world.ts');
const { CommandBatch } = await src('commands.ts');
const { EVENT_YIELD, EVENT_BREAK } = await src('core.ts');
const { buildBridge3, rollTruck3, newRides } = await src('scenes3d.ts');

const core = await Core.load(new Uint8Array(readFileSync(join(root, 'target', 'wasm32-unknown-unknown', 'release', 'beam_core.wasm'))));

/** Rolls a truck of `mass` from step 0 for `seconds`; reports where it was at the first yield and break. */
function run(mass, seconds = 14) {
  const world = new World(core);
  const build = new CommandBatch();
  const layout = buildBridge3(build);
  world.apply(build);
  const truck = { mass, speed: 4 };
  const rides = newRides();
  const out = { mass, yieldX: null, breakX: null, fellAt: null, breaks: 0 };
  for (let s = 0; s < Math.round(seconds * 60); s++) {
    const b = new CommandBatch();
    rollTruck3(b, layout, truck, s, rides.heavy, world.positions());
    world.apply(b);
    world.step();
    const x = truck.speed * s / 60;
    for (const e of world.events()) {
      if (e.kind === EVENT_YIELD && out.yieldX === null) out.yieldX = x;
      if (e.kind === EVENT_BREAK) {
        out.breaks++;
        if (out.breakX === null) out.breakX = x;
      }
    }
    if (rides.heavy.fell && out.fellAt === null) out.fellAt = x;
  }
  world.free();
  return out;
}

const f = (v) => (v === null ? '   -  ' : v.toFixed(1).padStart(6));
const show = (r) => console.log(`${String(r.mass).padStart(8)} kg  yield at x=${f(r.yieldX)}  break at x=${f(r.breakX)}  fell at x=${f(r.fellAt)}  breaks=${r.breaks}`);

const given = process.argv.slice(2).map(Number);
if (given.length) {
  for (const m of given) show(run(m));
} else {
  console.log('sweep (deck is 24 m, middle is x = 12):');
  for (const m of [5000, 10000, 20000, 30000, 40000, 50000, 60000, 80000, 100000, 150000, 200000]) show(run(m));
  // Lightest truck that breaks anything, to 100 kg.
  let lo = 5000;
  let hi = 200000;
  while (hi - lo > 100) {
    const mid = Math.round((lo + hi) / 2 / 100) * 100;
    if (run(mid).breaks > 0) hi = mid;
    else lo = mid;
  }
  console.log(`lightest truck that breaks a beam: about ${hi} kg`);
  show(run(hi));
}
