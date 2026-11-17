import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";
import { Core, EVENT_BREAK } from "../src/core.ts";

// Same numbers as core/tests/ffi.rs, recorded from the native build. If the solver changes on
// purpose, re-record both together. Equal hashes here are what show native and WebAssembly agree.
const SMALL_BENCH_120 = 0x935259e4d2c6ff87n;
const HAND_BUILT_30 = 0xb62c7e7dd415f3a1n;
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
