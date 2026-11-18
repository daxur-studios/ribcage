// Purpose: record the hopper standing, lifting off, hopping and dropping as .rcrd files, and time
// the WebAssembly core on them in Node.
// Value: the input for `cargo run --release -p beam-core --example scene_stats` (native contacts and
// timing, as ER 003's table) and a quick Node number to set beside it. Usage:
//   node scripts/hopper-recordings.mjs [outDir]     (default: artifacts/timing-hopper)
import { mkdirSync, readFileSync, writeFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';

const root = join(dirname(fileURLToPath(import.meta.url)), '..');
const outDir = process.argv[2] ?? join(root, 'artifacts', 'timing-hopper');
const src = (name) => import(pathToFileURL(join(root, 'src', name)).href);
const { Core } = await src('core.ts');
const { World } = await src('world.ts');
const { CommandBatch } = await src('commands.ts');
const { Recorder, encodeRecording } = await src('recording.ts');
const { scenarioFor } = await src('craft-page.ts');
const { flightCommands } = await src('flight.ts');
const { HOP_STEPS, ROUGH_SPOTS } = await src('landing.ts');

const wasm = join(root, 'target', 'wasm32-unknown-unknown', 'release', 'beam_core.wasm');
const core = await Core.load(new Uint8Array(readFileSync(wasm)));
const craftText = readFileSync(join(root, 'test', 'fixtures', 'hopper.craft.json'), 'utf8');
mkdirSync(outDir, { recursive: true });

const FULL = [{ step: 0, throttle: 1 }];
const runs = [
  // name, steps, scenario, script
  ['hopper-stand', 600, scenarioFor('fly', craftText), null],
  ['hopper-liftoff', 600, scenarioFor('fly', craftText), (b, f, s) => flightCommands(b, f, FULL, s)],
  ['hopper-hop', HOP_STEPS, scenarioFor('hop', craftText), null],
  ['hopper-drop3-pad', 600, scenarioFor('drop', craftText, { speed: 3 }), null],
  ['hopper-drop7-pad', 600, scenarioFor('drop', craftText, { speed: 7 }), null],
  ['hopper-drop7-rough', 600, scenarioFor('drop', craftText, { speed: 7, spot: ROUGH_SPOTS[0] }), null],
];

console.log('run                    nodes beams steps  mean ms (Node, WebAssembly)  worst ms  max substeps');
for (const [name, steps, scenario, extra] of runs) {
  const world = new World(core);
  const rec = new Recorder(world);
  rec.setup(scenario.flight.setup);
  let sum = 0;
  let worst = 0;
  let sub = 0;
  for (let s = 0; s < steps; s++) {
    const b = new CommandBatch();
    if (scenario.script) scenario.script(b, s);
    else if (extra) extra(b, scenario.flight, s);
    const t = performance.now();
    sub = Math.max(sub, rec.step(b));
    const ms = performance.now() - t;
    sum += ms;
    worst = Math.max(worst, ms);
  }
  writeFileSync(join(outDir, `${name}.rcrd`), encodeRecording(rec.finish()));
  console.log(
    `${name.padEnd(22)} ${String(world.nodeCount).padStart(5)} ${String(world.beamCount).padStart(5)} ${String(steps).padStart(5)} ${(sum / steps).toFixed(3).padStart(10)} ${worst.toFixed(3).padStart(24)} ${String(sub).padStart(12)}`,
  );
  world.free();
}
