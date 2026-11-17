// Purpose: record the tower and the bridge standing and under each action as .rcrd files, and time
// the WebAssembly core on them in Node.
// Value: the input for `cargo run --release -p beam-core --example scene_stats` (native contacts
// and timing) and a quick Node number to set beside the browser's. Usage:
//   node scripts/scene-recordings.mjs [outDir] [steps]     (defaults: artifacts/timing, 900)
import { mkdirSync, readFileSync, writeFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';

const root = join(dirname(fileURLToPath(import.meta.url)), '..');
const outDir = process.argv[2] ?? join(root, 'artifacts', 'timing');
const steps = Number(process.argv[3] ?? 900);
const src = (name) => import(pathToFileURL(join(root, 'src', name)).href);
const { Core } = await src('core.ts');
const { World } = await src('world.ts');
const { CommandBatch } = await src('commands.ts');
const { Recorder, encodeRecording } = await src('recording.ts');
const { STANDARD_TRUCK } = await src('bridge.ts');
const { buildTower, buildBridge3, towerAction, bridgeAction } = await src('scenes3d.ts');

const wasm = join(root, 'target', 'wasm32-unknown-unknown', 'release', 'beam_core.wasm');
const core = await Core.load(new Uint8Array(readFileSync(wasm)));
mkdirSync(outDir, { recursive: true });

const runs = [
  ['tower', null],
  ['tower', 'knock'],
  ['tower', 'removeColumn'],
  ['tower', 'overload'],
  ['bridge', null],
  ['bridge', 'knock'],
  ['bridge', 'removeColumn'],
  ['bridge', 'overload'],
];

console.log('run                    nodes beams steps  mean ms (Node, WebAssembly)  worst ms  max substeps');
for (const [scene, action] of runs) {
  const world = new World(core);
  const rec = new Recorder(world);
  const build = new CommandBatch();
  const layout = scene === 'tower' ? buildTower(build) : buildBridge3(build);
  rec.setup(build);
  let sum = 0;
  let worst = 0;
  let sub = 0;
  for (let s = 0; s < steps; s++) {
    const b = new CommandBatch();
    if (scene === 'tower') {
      if (action) towerAction(b, layout, action, s);
    } else bridgeAction(b, layout, action, s, STANDARD_TRUCK);
    const t = performance.now();
    sub = Math.max(sub, rec.step(b));
    const ms = performance.now() - t;
    sum += ms;
    worst = Math.max(worst, ms);
  }
  const name = `${scene}-${action ?? 'stand'}`;
  writeFileSync(join(outDir, `${name}.rcrd`), encodeRecording(rec.finish()));
  console.log(
    `${name.padEnd(22)} ${String(world.nodeCount).padStart(5)} ${String(world.beamCount).padStart(5)} ${String(steps).padStart(5)} ${(sum / steps).toFixed(3).padStart(10)} ${worst.toFixed(3).padStart(24)} ${String(sub).padStart(12)}`,
  );
  world.free();
}
