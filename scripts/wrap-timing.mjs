// Purpose: the milestone 5 timing table: the hopper's hop and the tall rocket, sleeping on against
// off: nodes, beams, bodies, substeps and milliseconds per step for each flight phase, in Node.
// Value: the one place the frame-time saving is measured (a live page shares one CPU and uses
// software rendering). Each run is repeated and the lowest time per phase kept, to shed noise.
// Usage: node --experimental-strip-types scripts/wrap-timing.mjs [repeats]   (default 5)
import { readFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';

const root = join(dirname(fileURLToPath(import.meta.url)), '..');
const src = (name) => import(pathToFileURL(join(root, 'src', name)).href);
const { Core } = await src('core.ts');
const { World } = await src('world.ts');
const { CommandBatch } = await src('commands.ts');
const { prepareHop, hopCommands, HOP_STEPS } = await src('landing.ts');
const { runTallRocket } = await src('tall-rocket.ts');

const repeats = Number(process.argv[2] ?? 5);
const core = await Core.load(new Uint8Array(readFileSync(join(root, 'target', 'wasm32-unknown-unknown', 'release', 'beam_core.wasm'))));
const hopper = readFileSync(join(root, 'test', 'fixtures', 'hopper.craft.json'), 'utf8');
const rocket = readFileSync(join(root, 'test', 'fixtures', 'tall-rocket.craft.json'), 'utf8');

function hop(sleeping) {
  const h = prepareHop(hopper, sleeping ? { sleeping } : { segments: false, rigid: false });
  const world = new World(core);
  world.apply(h.flight.setup);
  const nodes = world.nodeCount, beams = world.beamCount;
  const phases = new Map();
  let touch = -1;
  const craft = h.flight.loaded.craft;
  for (let s = 0; s < HOP_STEPS; s++) {
    const b = new CommandBatch();
    hopCommands(b, h, s);
    world.apply(b);
    const t0 = performance.now();
    const sub = world.step();
    const ms = performance.now() - t0;
    world.events();
    if (touch < 0 && s > 20 && s >= 220) {
      const p = world.positions();
      for (const [i, n] of craft.nodes.entries()) if (p[3 * i + 1] - n.radius < 0.002 + 0) { touch = s; break; }
    }
    const phase = s < 60 ? 'lift-off' : s < 220 ? 'thrust' : touch < 0 ? 'coast' : s < touch + 60 ? 'touchdown' : 'rest';
    let alive = 0;
    for (const f of world.bodies().flags) if ((f & 1) !== 0) alive++;
    const q = phases.get(phase) ?? { steps: 0, sub: 0, ms: 0, bodies: 0 };
    q.steps++; q.sub += sub; q.ms += ms; q.bodies += alive;
    phases.set(phase, q);
  }
  const hash = world.stateHash; const phys = world.physicsHash;
  world.free();
  return { nodes, beams, phases, hash, phys };
}

const best = (runs, key) => runs.reduce((a, r) => Math.min(a, r), Infinity);
const rows = [];
for (const sleeping of [false, true]) {
  const runs = Array.from({ length: repeats }, () => hop(sleeping));
  const r0 = runs[0];
  for (const [name] of r0.phases) {
    const q = r0.phases.get(name);
    const ms = best(runs.map((r) => r.phases.get(name).ms / r.phases.get(name).steps));
    rows.push(`| hopper | ${name} | ${sleeping ? 'on' : 'off'} | ${r0.nodes} | ${r0.beams} | ${(q.bodies / q.steps).toFixed(1)} | ${(q.sub / q.steps).toFixed(1)} | ${ms.toFixed(3)} |`);
  }
  console.error(`hop sleeping ${sleeping}: state 0x${r0.hash.toString(16)} physics 0x${r0.phys.toString(16)}`);
}
for (const sleeping of [false, true]) {
  const runs = Array.from({ length: repeats }, () => runTallRocket(core, rocket, sleeping));
  const r0 = runs[0];
  const w = new World(core);
  w.free();
  for (const [name, q] of r0.phases) {
    const ms = best(runs.map((r) => r.phases.get(name).ms / r.phases.get(name).steps));
    rows.push(`| rocket | ${name} | ${sleeping ? 'on' : 'off'} | 147 | 808 | ${q.bodiesMin}-${q.bodiesMax} | ${(q.substeps / q.steps).toFixed(1)} | ${ms.toFixed(3)} |`);
  }
}
console.log('| craft | phase | sleeping | nodes | beams | bodies | substeps/step | ms/step |\n|---|---|---|---|---|---|---|---|');
console.log(rows.join('\n'));
