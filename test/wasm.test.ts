import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";
import { CommandBatch, REJECTED } from "../src/commands.ts";
import { Core, EVENT_BREAK } from "../src/core.ts";

// Same numbers as core/tests/ffi.rs, recorded from the native build. If the solver changes on
// purpose, re-record both together. Equal hashes here are what show native and WebAssembly agree.
const SMALL_BENCH_120 = 0x935259e4d2c6ff87n;
const HAND_BUILT_30 = 0xb62c7e7dd415f3a1n;
/** A 5 by 4 terrain with heights `(k % 7) * 0.5`, one step (`TERRAIN_ONLY` in core/tests/ffi.rs). */
const TERRAIN_ONLY = 0x424288d9b2e8abb4n;
/** That terrain with a triangle and two lone nodes on it, after 120 steps (`TERRAIN_SCENE_120` in core/tests/ffi.rs). */
const TERRAIN_SCENE_120 = 0x05e2e896a698c2f5n;
/** End hash of the reported scene after 720 steps at the default rule (`cargo run --release -p beam-core --example bench`). */
const STANDARD_BENCH_720 = 0x871092299f3cea56n;

const WASM_PATH = new URL("../target/wasm32-unknown-unknown/release/beam_core.wasm", import.meta.url);

function loadBytes(): Uint8Array<ArrayBuffer> {
  try {
    return new Uint8Array(readFileSync(WASM_PATH));
  } catch {
    throw new Error("beam_core.wasm is missing: run `pnpm build:wasm` first");
  }
}

test("the module imports nothing and loads", async () => {
  const bytes = loadBytes();
  assert.deepEqual(WebAssembly.Module.imports(new WebAssembly.Module(bytes)), []);
  const core = await Core.load(bytes);
  assert.ok(core.memoryBytes > 0);
});

test("the small benchmark scene gives the native hash", async () => {
  const core = await Core.load(loadBytes());
  const h = core.benchNew(false);
  for (let i = 0; i < 120; i++) core.benchStep(h);
  assert.equal(core.stepCount(h), 120n);
  assert.equal(core.stateHash(h), SMALL_BENCH_120);
  assert.equal(core.events(h).length, 0);
  core.hostFree(h);
});

test("a hand-built scene gives the native hash", async () => {
  const core = await Core.load(loadBytes());
  const h = core.hostNew();
  const a = core.addNode(h, 0, 0, 0, 0, 0.1);
  const b = core.addNode(h, 1, 0, 0, 1, 0.1);
  core.addBeam(h, a, b, { rest: 1, compliance: 1e-6, damping: 0.5, yieldStrain: 0.01, breakStrain: 0.05 });
  for (let i = 0; i < 30; i++) core.step(h);
  assert.equal(core.stateHash(h), HAND_BUILT_30);
  core.hostFree(h);
});

test("a terrain-only world gives the native hash, and bad terrain records are refused", async () => {
  const core = await Core.load(loadBytes());
  const h = core.hostNew();
  const batch = new CommandBatch();
  batch.setTerrain(5, 4, -4, 1, 2, 0.8);
  batch.allTerrainHeights(Array.from({ length: 20 }, (_, k) => (k % 7) * 0.5));
  assert.equal(batch.count, 5); // the grid, then 6 + 6 + 6 + 2 heights
  const ok = core.applyCommands(h, batch);
  assert.equal(ok.rejected, 0);
  assert.ok(ok.results.every((r) => r === 1));
  core.step(h);
  assert.equal(core.stateHash(h), TERRAIN_ONLY);

  // A bad record is refused, the one after it still applies.
  const bad = new CommandBatch();
  bad.setTerrain(1, 4, 0, 0, 1, 0.5);
  bad.terrainHeights(19, [1.5]);
  const r = core.applyCommands(h, bad);
  assert.deepEqual([...r.results], [REJECTED, 1]);
  assert.equal(r.rejected, 1);
  assert.notEqual(core.stateHash(h), TERRAIN_ONLY);
  core.hostFree(h);
});

test("ground contact on terrain gives the native hash", async () => {
  const core = await Core.load(loadBytes());
  const h = core.hostNew();
  const batch = new CommandBatch();
  batch.setTerrain(5, 4, -4, 1, 2, 0.8);
  batch.allTerrainHeights(Array.from({ length: 20 }, (_, k) => (k % 7) * 0.5));
  const nodes: [number, number, number, number, number][] = [
    [-1, 4, 3, 0.1, 0.2],
    [0, 4, 3.5, 0.1, 0.2],
    [-0.5, 4.8, 4, 0.1, 0.2],
    [3.5, 2, 6.5, 1, 0.1],
    [6, 1, 0, 1, 0.1],
  ];
  for (const [x, y, z, invMass, radius] of nodes) batch.addNode(x, y, z, invMass, radius);
  for (const [a, b] of [[0, 1], [1, 2], [2, 0]]) {
    batch.addBeam(a, b, { rest: 1.1, compliance: 1e-6, damping: 50, yieldStrain: 0.5, breakStrain: 1 });
  }
  assert.equal(core.applyCommands(h, batch).rejected, 0);
  for (let i = 0; i < 120; i++) core.step(h);
  assert.equal(core.stateHash(h), TERRAIN_SCENE_120);
  assert.equal(core.events(h).length, 0);
  core.hostFree(h);
});

test("a terrain record carries one to six heights", () => {
  const b = new CommandBatch();
  assert.throws(() => b.terrainHeights(0, []), /1 to 6 heights/);
  assert.throws(() => b.terrainHeights(0, [1, 2, 3, 4, 5, 6, 7]), /1 to 6 heights/);
  assert.equal(b.count, 0);
  b.allTerrainHeights([]);
  assert.equal(b.count, 0);
  b.allTerrainHeights(new Float64Array(13));
  assert.equal(b.count, 3);
});

test("positions and render buffer are read in place", async () => {
  const core = await Core.load(loadBytes());
  const h = core.hostNew();
  core.addNode(h, 0.1, 2.5, 0, 0, 0.1);
  core.addNode(h, -3, 4.25, 1, 1, 0.1);
  assert.deepEqual([...core.positions(h)], [0.1, 2.5, 0, -3, 4.25, 1]);
  assert.deepEqual([...core.render(h)], [Math.fround(0.1), 2.5, 0, -3, 4.25, 1]);
  core.hostFree(h);
});

test("an event is decoded from the core's memory", async () => {
  const core = await Core.load(loadBytes());
  const h = core.hostNew();
  const a = core.addNode(h, 0, 0, 0, 0, 0.1);
  const b = core.addNode(h, 1.5, 0, 0, 1, 0.1);
  core.addBeam(h, a, b, { rest: 1, compliance: 1e-6, damping: 0.5, yieldStrain: 0.01, breakStrain: 0.05 });
  core.step(h);
  const events = core.events(h);
  assert.equal(events.length, 1);
  assert.equal(events[0]!.kind, EVENT_BREAK);
  assert.equal(events[0]!.index, 0);
  assert.equal(events[0]!.step, 1n);
  assert.ok(events[0]!.value > 0.05);
  core.hostFree(h);
});

test("memory growth does not invalidate reads", async () => {
  const core = await Core.load(loadBytes());
  const h = core.hostNew();
  const before = core.memoryBytes;
  // Enough nodes and beams to make the core's arrays grow memory.
  let prev = core.addNode(h, 0, 0, 0, 0, 0.1);
  for (let i = 1; i < 60_000; i++) {
    const n = core.addNode(h, i, 0, 0, 1, 0.1);
    core.addBeam(h, prev, n, { rest: 1, compliance: 1e-6, damping: 0.5, yieldStrain: 0.01, breakStrain: 0.05 });
    prev = n;
  }
  assert.ok(core.memoryBytes > before, "memory grew");
  const p = core.positions(h);
  assert.equal(p.length, 60_000 * 3);
  assert.equal(p[59_999 * 3], 59_999);
  core.hostFree(h);
});

// The reported scene, 720 steps: several seconds, so opt in with RIBCAGE_SLOW=1. Also prints the
// first WebAssembly timing (ER 002, phase 2.1).
test("the reported benchmark scene gives the native hash", { skip: !process.env.RIBCAGE_SLOW }, async () => {
  const core = await Core.load(loadBytes());
  const h = core.benchNew(true);
  assert.equal(core.beamCount(h), 11_136);
  for (let i = 0; i < 120; i++) core.benchStep(h);
  const times: number[] = [];
  let substeps = 0;
  for (let i = 0; i < 600; i++) {
    const t = performance.now();
    substeps = core.benchStep(h);
    times.push(performance.now() - t);
  }
  assert.equal(core.stateHash(h), STANDARD_BENCH_720);
  times.sort((a, b) => a - b);
  const mean = times.reduce((s, v) => s + v, 0) / times.length;
  console.log(
    `wasm in Node ${process.version}: ${substeps} substeps, mean ${mean.toFixed(2)} ms, median ${times[300]!.toFixed(2)} ms, p95 ${times[570]!.toFixed(2)} ms per step`,
  );
  core.hostFree(h);
});
