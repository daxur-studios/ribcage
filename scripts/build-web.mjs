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
copyFileSync(join(root, 'web', 'terrain.html'), join(dist, 'web', 'terrain.html'));
copyFileSync(wasm, join(dist, 'web', 'beam_core.wasm'));

// Three.js, copied from node_modules (pinned to one exact version in package.json) so the page
// loads it through an import map and needs no bundler. Only the files the page uses.
const three = join(root, 'node_modules', 'three');
const vendor = join(dist, 'web', 'vendor');
const vendored = [
  ['build/three.core.js', 'three.core.js'],
  ['build/three.webgpu.js', 'three.webgpu.js'],
  ['build/three.tsl.js', 'three.tsl.js'],
  ['examples/jsm/controls/OrbitControls.js', 'addons/controls/OrbitControls.js'],
  ['LICENSE', 'LICENSE-three.txt'],
];
for (const [from, to] of vendored) {
  const target = join(vendor, to);
  mkdirSync(dirname(target), { recursive: true });
  copyFileSync(join(three, from), target);
}
// The sample the "Replay sample" button plays: the same file the tests replay.
// The craft page's hopper and its sample flight (the committed hard landing).
copyFileSync(join(root, 'web', 'craft.html'), join(dist, 'web', 'craft.html'));
copyFileSync(join(root, 'test', 'fixtures', 'hopper.craft.json'), join(dist, 'web', 'hopper.craft.json'));
copyFileSync(join(root, 'test', 'fixtures', 'hard-landing.rcrd'), join(dist, 'web', 'hard-landing.rcrd'));
copyFileSync(join(root, 'test', 'fixtures', 'weak-bridge.rcrd'), join(dist, 'web', 'weak-bridge.rcrd'));
copyFileSync(join(root, 'test', 'fixtures', 'tower-collapse.rcrd'), join(dist, 'web', 'tower-collapse.rcrd'));
console.log('built dist/web (open /web/index.html or /web/terrain.html from a server: pnpm serve)');
