import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";
import { CommandBatch, NO_SEGMENT, REJECTED } from "../src/commands.ts";
import { Core, EVENT_BREAK } from "../src/core.ts";

// Same numbers as core/tests/ffi.rs, recorded from the native build. If the solver changes on
// purpose, re-record both together. Equal hashes here are what show native and WebAssembly agree.
const SMALL_BENCH_120 = 0x935259e4d2c6ff87n;
const HAND_BUILT_30 = 0xb62c7e7dd415f3a1n;
/** A 5 by 4 terrain with heights `(k % 7) * 0.5`, one step (`TERRAIN_ONLY` in core/tests/ffi.rs). */
const TERRAIN_ONLY = 0x424288d9b2e8abb4n;
/** That terrain with a triangle and two lone nodes on it, after 120 steps (`TERRAIN_SCENE_120` in core/tests/ffi.rs). */
const TERRAIN_SCENE_120 = 0x2bdef035f12d0c86n;
/** A three-node craft with two thrusters and mid-run throttle changes, after 120 steps (`THRUST_SCENE_120` in core/tests/ffi.rs). */
const THRUST_SCENE_120 = 0x765d0f4475514944n;
/** A box and a tetrahedron as two segments, made rigid, merged and released (`BODY_SCENE_61` and `BODY_SCENE_120` in core/tests/ffi.rs). */
const BODY_SCENE_61 = 0x06024ffac786af7bn;
const BODY_SCENE_120 = 0x722722cfee9354f3n;
/** The body scene with sleeping on and the thrust cut before step 150, after 200 steps (core/tests/sleep.rs). */
const SLEEP_SCENE_200 = 0x8fb3e3c32250148bn;
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

test("a thrusting craft gives the native hash", async () => {
  const core = await Core.load(loadBytes());
  const h = core.hostNew();
  const setup = new CommandBatch();
  for (const [x, y, z, invMass] of [[0, 50, 0, 0.1], [1.5, 50, 0, 0.1], [0.75, 50, 1.3, 0.05]] as const) {
    setup.addNode(x, y, z, invMass, 0.1);
  }
  for (const [a, b] of [[0, 1], [1, 2], [2, 0]]) {
    setup.addBeam(a, b, { rest: 1.5, compliance: 1e-6, damping: 50, yieldStrain: 0.5, breakStrain: 1 });
  }
  setup.addThruster(0, 1, 400);
  setup.addThruster(2, 1, 250);
  setup.setThrottle(0, 1);
  const first = core.applyCommands(h, setup);
  assert.equal(first.rejected, 0);
  assert.deepEqual([...first.results.slice(6, 8)], [0, 1], "thrusters are numbered in the order added");
  for (let s = 0; s < 120; s++) {
    if (s === 20 || s === 70) {
      const more = new CommandBatch();
      if (s === 20) more.setThrottle(1, 0.6);
      else more.setThrottle(0, 0.3);
      assert.equal(core.applyCommands(h, more).rejected, 0);
    }
    core.step(h);
  }
  assert.equal(core.stateHash(h), THRUST_SCENE_120);
  assert.ok(core.positions(h)[0]! > 0.5, "the craft flew along +x");
  core.hostFree(h);
});

test("segments and a rigid body give the native hashes", async () => {
  const core = await Core.load(loadBytes());
  const h = core.hostNew();
  const setup = new CommandBatch();
  const spec = (rest: number) => ({ rest, compliance: 1e-5, damping: 20, yieldStrain: 5, breakStrain: 10 });
  for (let k = 0; k < 8; k++) {
    setup.addNode(k & 1 ? 0.5 : -0.5, k & 2 ? 50.5 : 49.5, k & 4 ? 0.5 : -0.5, 0.1, 0.1);
  }
  for (const [x, y, z] of [[2, 50, 0], [3, 50, 0], [2.5, 50, 0.8660254037844386], [2.5, 50.816496580927726, 0.28867513459481287]] as const) {
    setup.addNode(x, y, z, 0.2, 0.1);
  }
  for (let i = 0; i < 8; i++) {
    for (const bit of [1, 2, 4]) if ((i & bit) === 0) setup.addBeam(i, i | bit, spec(1));
  }
  for (const [a, b] of [[8, 9], [8, 10], [8, 11], [9, 10], [9, 11], [10, 11]] as const) setup.addBeam(a, b, spec(1));
  setup.addBeam(1, 8, spec(1.5));
  setup.addSegment(NO_SEGMENT, 0, 0, 0);
  setup.addSegment(0, 0, 0.01, 1e-5);
  for (let n = 0; n < 12; n++) setup.setNodeSegment(n, n < 8 ? 0 : 1);
  setup.addThruster(0, 1, 300);
  setup.setThrottle(0, 1);
  assert.equal(core.applyCommands(h, setup).rejected, 0);
  for (let s = 0; s < 120; s++) {
    const more = new CommandBatch();
    if (s === 10) more.makeRigid(0);
    if (s === 40) more.makeRigid(1, 0);
    if (s === 80) more.releaseRigid(1);
    if (s === 100) more.releaseRigid(0);
    if (more.count > 0) {
      const r = core.applyCommands(h, more);
      assert.equal(r.rejected, 0, `step ${s}`);
    }
    core.step(h);
    if (s === 60) {
      const bodies = core.bodies(h);
      assert.equal(bodies.flags[0], 1, "one body, in slot 0");
      assert.ok([...bodies.nodeBody].every((b) => b === 0), "every node rides it");
      assert.equal(core.stateHash(h), BODY_SCENE_61);
    }
  }
  assert.equal(core.stateHash(h), BODY_SCENE_120);
  assert.equal(core.bodies(h).flags[0], 0, "released");
  core.hostFree(h);
});

test("sleeping two segments into one body gives the native hash", async () => {
  const core = await Core.load(loadBytes());
  const h = core.hostNew();
  const setup = new CommandBatch();
  const spec = (rest: number) => ({ rest, compliance: 1e-5, damping: 20, yieldStrain: 5, breakStrain: 10 });
  for (let k = 0; k < 8; k++) {
    setup.addNode(k & 1 ? 0.5 : -0.5, k & 2 ? 50.5 : 49.5, k & 4 ? 0.5 : -0.5, 0.1, 0.1);
  }
  for (const [x, y, z] of [[2, 50, 0], [3, 50, 0], [2.5, 50, 0.8660254037844386], [2.5, 50.816496580927726, 0.28867513459481287]] as const) {
    setup.addNode(x, y, z, 0.2, 0.1);
  }
  for (let i = 0; i < 8; i++) {
    for (const bit of [1, 2, 4]) if ((i & bit) === 0) setup.addBeam(i, i | bit, spec(1));
  }
  for (const [a, b] of [[8, 9], [8, 10], [8, 11], [9, 10], [9, 11], [10, 11]] as const) setup.addBeam(a, b, spec(1));
  setup.addBeam(1, 8, spec(1.5));
  setup.addSegment(NO_SEGMENT, 0, 0, 0);
  setup.addSegment(0, 0, 0.01, 1e-5);
  for (let n = 0; n < 12; n++) setup.setNodeSegment(n, n < 8 ? 0 : 1);
  setup.addThruster(0, 1, 300);
  setup.setThrottle(0, 1);
  setup.setSleeping(true);
  assert.equal(core.applyCommands(h, setup).rejected, 0);
  for (let s = 0; s < 200; s++) {
    if (s === 150) {
      const cut = new CommandBatch();
      cut.setThrottle(0, 0);
      assert.equal(core.applyCommands(h, cut).rejected, 0);
    }
    core.step(h);
    if (s === 149) {
      const sl = core.sleep(h);
      assert.deepEqual([...sl.asleep], [1, 1], "both asleep before the cut");
      const b = core.bodies(h);
      assert.equal(b.segmentBody[0], b.segmentBody[1], "one body");
      assert.ok(Math.abs(b.mass[b.segmentBody[0]!]! - 100) < 1e-9, "summed mass");
    }
  }
  assert.deepEqual([...core.sleep(h).asleep], [0, 0], "awake after the cut");
  assert.equal(core.stateHash(h), SLEEP_SCENE_200);
  core.hostFree(h);
});

test("a bad thruster record is rejected through the boundary and the rest applies", async () => {
  const core = await Core.load(loadBytes());
  const h = core.hostNew();
  const b = new CommandBatch();
  b.addNode(0, 10, 0, 0.1, 0.1);
  b.addNode(1, 10, 0, 0.1, 0.1);
  b.addThruster(0, 0, 100); // the same node twice
  b.addThruster(0, 1, -1); // negative thrust
  b.addThruster(0, 1, 100); // fine: thruster 0
  b.setThrottle(0, 2); // above 1
  b.setThrottle(5, 0.5); // no such thruster
  b.setThrottle(0, 0.5); // fine
  const r = core.applyCommands(h, b);
  assert.deepEqual([...r.results], [0, 1, REJECTED, REJECTED, 0, REJECTED, REJECTED, 1]);
  assert.equal(r.rejected, 4);
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
