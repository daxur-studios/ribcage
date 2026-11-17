// Purpose: build the browser page into dist/ with no bundler: compile src/ and web/ with plain
// tsc (relative `.ts` imports are rewritten to `.js`), then copy the page and the wasm next to it.
// Value: the editor runs from static files, so any file server works and nothing is generated
// that a person must keep in step.
import { execFileSync } from 'node:child_process';
import { copyFileSync, existsSync, mkdirSync, rmSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';

const root = join(dirname(fileURLToPath(import.meta.url)), '..');
const dist = join(root, 'dist');
const wasm = join(root, 'target', 'wasm32-unknown-unknown', 'release', 'beam_core.wasm');

if (!existsSync(wasm)) {
  console.error('beam_core.wasm is missing: run `pnpm build:wasm` first.');
  process.exit(1);
}

rmSync(dist, { recursive: true, force: true });
const tsc = join(root, 'node_modules', 'typescript', 'bin', 'tsc');
execFileSync(process.execPath, [tsc, '-p', join(root, 'tsconfig.web.json')], { stdio: 'inherit' });

mkdirSync(join(dist, 'web'), { recursive: true });
copyFileSync(join(root, 'web', 'index.html'), join(dist, 'web', 'index.html'));
copyFileSync(wasm, join(dist, 'web', 'beam_core.wasm'));
// The sample the "Replay sample" button plays: the same file the tests replay.
copyFileSync(join(root, 'test', 'fixtures', 'weak-bridge.rcrd'), join(dist, 'web', 'weak-bridge.rcrd'));
console.log('built dist/web (open /web/index.html from a server: pnpm serve)');
