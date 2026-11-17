import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";
import { CommandBatch, REJECTED } from "../src/commands.ts";
import { Core, EVENT_BREAK, EVENT_YIELD, type CoreEvent } from "../src/core.ts";
import {
  buildBridge,
  rollTruck,
  STANDARD_TRUCK,
  STRONG_BRIDGE,
  WEAK_BRIDGE,
  type BridgeLayout,
  type BridgeSpec,
} from "../src/bridge.ts";
import { World } from "../src/world.ts";

const WASM_PATH = new URL("../target/wasm32-unknown-unknown/release/beam_core.wasm", import.meta.url);

async function loadCore(): Promise<Core> {
  try {
    return await Core.load(new Uint8Array(readFileSync(WASM_PATH)));
  } catch (e) {
    throw new Error(`beam_core.wasm is missing or stale: run \`pnpm build:wasm\` first (${String(e)})`);
  }
}

interface Run {
  world: World;
  layout: BridgeLayout;
  /** The bytes of every step's command list, in order (what a recording holds). */
  lists: Uint8Array[];
  events: { step: number; event: CoreEvent }[];
  /** Largest |strain / yield strain| seen on any live beam, sampled every step. */
  peakRatio: number;
  /** Lowest node height seen at the end of any step. */
  lowest: number;
  maxSubsteps: number;
}

/** Truck crossing time plus a few seconds to settle. */
function stepsFor(layout: BridgeLayout): number {
  return Math.ceil((layout.span / STANDARD_TRUCK.speed) * 60) + 240;
}

function runBridge(core: Core, spec: BridgeSpec): Run {
  const world = new World(core);
  const build = new CommandBatch();
  const layout = buildBridge(build, spec);
  const built = world.apply(build);
  assert.equal(built.rejected, 0);
  // The layout's indices are the ones a fresh world hands out.
  assert.deepEqual([...built.results].slice(0, layout.deck.length), layout.deck);

  const run: Run = { world, layout, lists: [build.bytes.slice()], events: [], peakRatio: 0, lowest: 0, maxSubsteps: 0 };
  const steps = stepsFor(layout);
  for (let s = 0; s < steps; s++) {
    const cmds = new CommandBatch();
    rollTruck(cmds, layout, STANDARD_TRUCK, s);
    assert.equal(world.apply(cmds).rejected, 0);
    run.lists.push(cmds.bytes.slice());
    run.maxSubsteps = Math.max(run.maxSubsteps, world.step());
    for (const event of world.events()) run.events.push({ step: s, event });
    const snap = world.snapshot();
    for (const r of snap.beams.ratio) run.peakRatio = Math.max(run.peakRatio, Math.abs(r));
    for (let i = 1; i < snap.positions.length; i += 3) run.lowest = Math.min(run.lowest, snap.positions[i]!);
  }
  return run;
}

test("a strong bridge holds the truck: no yield, no break, nearly no sag", async () => {
  const core = await loadCore();
  const run = runBridge(core, STRONG_BRIDGE);
  assert.deepEqual(run.events, []);
  assert.ok(run.peakRatio < 0.6, `peak strain was ${run.peakRatio.toFixed(2)} of yield`);
  assert.ok(run.lowest > -0.05, `lowest node ${run.lowest.toFixed(3)} m`);
  assert.ok(run.maxSubsteps >= 4 && run.maxSubsteps <= 32, `${run.maxSubsteps} substeps`);

  // After the truck has gone and the bridge has settled, every deck node is back at rest height.
  const p = run.world.positions();
  for (const n of run.layout.deck) assert.ok(Math.abs(p[n * 3 + 1]!) < 5e-3, `deck node ${n} at ${p[n * 3 + 1]}`);
  run.world.free();
});

test("an under-built bridge yields, then breaks", async () => {
  const core = await loadCore();
  const run = runBridge(core, WEAK_BRIDGE);
  const firstYield = run.events.find((e) => e.event.kind === EVENT_YIELD);
  const firstBreak = run.events.find((e) => e.event.kind === EVENT_BREAK);
  assert.ok(firstYield, "a beam should yield");
  assert.ok(firstBreak, "a beam should break");
  assert.ok(firstYield.step <= firstBreak.step, "the first yield comes before the first break");
  assert.ok(run.peakRatio > 1, "strain went past yield");
  // A broken beam is flagged in the render data.
  const flags = run.world.snapshot().beams.flags;
  assert.ok(flags.some((f) => (f & 2) !== 0), "some beam is flagged broken");
  run.world.free();
});

test("a planar scene keeps z exactly zero", async () => {
  const core = await loadCore();
  for (const spec of [STRONG_BRIDGE, WEAK_BRIDGE]) {
    const run = runBridge(core, spec);
    const p = run.world.positions();
    for (let i = 2; i < p.length; i += 3) assert.ok(p[i] === 0, `node ${(i - 2) / 3} has z = ${p[i]}`);
    run.world.free();
  }
});

test("the same commands give the same hash, and a replay of the saved lists matches", async () => {
  const core = await loadCore();
  const a = runBridge(core, WEAK_BRIDGE);
  const b = runBridge(core, WEAK_BRIDGE);
  assert.equal(a.world.stateHash, b.world.stateHash);

  // Replay: a fresh world gets the saved byte lists and nothing else.
  const replay = new World(core);
  for (const [i, bytes] of a.lists.entries()) {
    assert.equal(replay.apply(CommandBatch.fromBytes(bytes)).rejected, 0);
    if (i > 0) replay.step(); // list 0 is the build, the rest are one per step
  }
  assert.equal(replay.stateHash, a.world.stateHash);
  assert.equal(replay.stepCount, a.world.stepCount);
  for (const w of [a.world, b.world, replay]) w.free();
});

test("bad records are rejected and the rest still apply", async () => {
  const core = await loadCore();
  const w = new World(core);
  const spec = { rest: 1, compliance: 1e-6, damping: 0.5, yieldStrain: 0.01, breakStrain: 0.05 };
  const c = new CommandBatch();
  c.addNode(0, 0, 0, 0, 0.1); // 0
  c.addNode(1, 0, 0, 1, 0.1); // 1
  c.addNode(Number.NaN, 0, 0, 1, 0.1); // rejected
  c.addBeam(0, 0, spec); // rejected: same node twice
  c.addBeam(0, 7, spec); // rejected: no such node
  c.addBeam(0, 1, spec); // beam 0
  c.addForce(1, 0, Number.POSITIVE_INFINITY, 0); // rejected
  c.deleteBeam(3); // rejected
  c.deleteNode(1); // ok, takes the beam with it
  c.addForce(1, 0, -1, 0); // rejected: dead node
  const r = w.apply(c);
  assert.deepEqual([...r.results], [0, 1, REJECTED, REJECTED, REJECTED, 0, REJECTED, REJECTED, 1, REJECTED]);
  assert.equal(r.rejected, 6);
  w.free();
});

test("a batch of unaligned or partial bytes is refused, not misread", () => {
  assert.throws(() => CommandBatch.fromBytes(new Uint8Array(65)), /whole number/);
});

test("memory growth does not break reads", async () => {
  const core = await loadCore();
  const w = new World(core);
  const before = core.memoryBytes;
  const c = new CommandBatch();
  const N = 60_000;
  for (let i = 0; i < N; i++) c.addNode(i, -i, 0, 1, 0.1);
  const r = w.apply(c);
  assert.equal(r.rejected, 0);
  assert.equal(r.results[N - 1], N - 1);
  assert.ok(core.memoryBytes > before, "memory should have grown");
  const snap = w.snapshot();
  assert.equal(snap.positions.length, N * 3);
  assert.equal(snap.positions[(N - 1) * 3], N - 1);
  assert.equal(snap.positions[(N - 1) * 3 + 1], -(N - 1));
  w.free();
});
