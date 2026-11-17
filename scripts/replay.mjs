// Purpose: replay a saved recording in Node and say whether it comes out identical.
// Value: the check that a run saved from the browser (or an older build) still replays exactly,
// without opening a page. Usage: node scripts/replay.mjs <file.rcrd>   (exit code 1 if it differs)
import { readFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';

const root = join(dirname(fileURLToPath(import.meta.url)), '..');
const file = process.argv[2];
if (!file) {
  console.error('usage: node scripts/replay.mjs <file.rcrd>');
  process.exit(2);
}
const src = (name) => import(pathToFileURL(join(root, 'src', name)).href);
const { Core } = await src('core.ts');
const { World } = await src('world.ts');
const { decodeRecording, replayRecording } = await src('recording.ts');

const wasm = join(root, 'target', 'wasm32-unknown-unknown', 'release', 'beam_core.wasm');
const core = await Core.load(new Uint8Array(readFileSync(wasm)));
const rec = decodeRecording(new Uint8Array(readFileSync(file)));
const result = replayRecording(() => new World(core), rec);
const hex = (n) => '0x' + n.toString(16).padStart(16, '0');
console.log(`${result.steps} steps, hash ${hex(result.hash)}`);
if (result.ok) console.log('identical to the recording: every checkpoint and the final hash match');
else {
  if (result.mismatch) console.log(`DIFFERS at step ${result.mismatch.step}: expected ${hex(result.mismatch.expected)}, got ${hex(result.mismatch.actual)}`);
  if (result.rejected > 0) console.log(`the core refused ${result.rejected} recorded commands`);
  process.exit(1);
}
