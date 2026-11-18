import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";
import { CommandBatch, NO_SEGMENT, REJECTED, SEG_NEVER_LIVE } from "../src/commands.ts";
import { Core } from "../src/core.ts";
import { prepareFlight } from "../src/flight.ts";
import { HOP_STEPS, hopCommands, prepareHop } from "../src/landing.ts";
import { compileCraft } from "../src/parts/assemble.ts";
import { compilePart } from "../src/parts/part.ts";
import { decodeRecording, replayRecording } from "../src/recording.ts";
import { World } from "../src/world.ts";

const WASM_PATH = new URL("../target/wasm32-unknown-unknown/release/beam_core.wasm", import.meta.url);
const HOPPER = readFileSync(new URL("./fixtures/hopper.craft.json", import.meta.url), "utf8");
const HOP_HASH = 0xd92f2aad01c0e048n;

async function loadCore(): Promise<Core> {
  return Core.load(new Uint8Array(readFileSync(WASM_PATH)));
}

/** The hopper's setup with segments, applied to a fresh world. */
async function hopperWorld() {
  const core = await loadCore();
  const flight = prepareFlight(HOPPER, undefined, { segments: true });
  const world = new World(core);
  const applied = world.apply(flight.setup);
  assert.equal(applied.rejected, 0);
  return { world, flight, craft: flight.loaded.craft, segmentOf: flight.segmentOf! };
}

test("the hopper's segments in the core match the assembler's", async () => {
  const { world, craft, segmentOf } = await hopperWorld();
  const seg = world.segments();
  assert.equal(seg.flags.length, craft.segments.length);
  assert.equal(seg.seamA.length, craft.seams.length, "one seam per craft seam (the craft's seams form a tree)");
  assert.deepEqual(segmentOf, craft.segments.map((_, i) => i), "for the hopper the core's numbers are the craft's");

  // Membership: every node is in the segment the craft says, and nothing else is.
  assert.equal(seg.nodeSegment.length, craft.nodes.length);
  for (const [i, n] of craft.nodes.entries()) assert.equal(seg.nodeSegment[i], segmentOf[n.segment], `node ${i}`);

  // Flags: engine and pod segments are never live, the rest are.
  const neverLive = craft.segments.map((s) => (s.neverLive ? SEG_NEVER_LIVE : 0));
  assert.deepEqual([...seg.flags], neverLive);
  assert.ok(neverLive.some((f) => f !== 0) && neverLive.some((f) => f === 0));

  // Tree: exactly one root, and every other segment's parent and seam properties are the craft seam
  // between the two, area and bending equal to the bit.
  const roots = [...seg.parent].filter((p) => p === NO_SEGMENT).length;
  assert.equal(roots, 1);
  for (let s = 0; s < craft.segments.length; s++) {
    const p = seg.parent[s]!;
    if (p === NO_SEGMENT) {
      assert.equal(seg.seam[s], NO_SEGMENT);
      continue;
    }
    assert.ok(p < s || craft.segments.length > 0, "a parent is added before its child");
    const k = seg.seam[s]!;
    assert.equal(seg.seamA[k], p);
    assert.equal(seg.seamB[k], s);
    const seam = craft.seams.find((q) => (q.segmentA === p && q.segmentB === s) || (q.segmentA === s && q.segmentB === p));
    assert.ok(seam, `a craft seam between segments ${p} and ${s}`);
    assert.equal(seg.seamArea[k], seam.area);
    assert.equal(seg.seamBending[k], seam.bending);
    assert.ok(seam.area > 0 && seam.bending >= 0);
  }

  // Each segment's stored mass is the sum of its nodes' masses as the core sees them (1 / (1 / m)).
  const expected = new Array<number>(craft.segments.length).fill(0);
  for (const n of craft.nodes) expected[n.segment]! += 1 / (1 / n.mass);
  for (let s = 0; s < expected.length; s++) {
    assert.ok(Math.abs(seg.mass[s]! - expected[s]!) <= 1e-12 * expected[s]!, `segment ${s}: ${seg.mass[s]} against ${expected[s]}`);
  }
  world.free();
});

test("the hopper's stored segment masses sum to 4600 kg", async () => {
  const { world, craft } = await hopperWorld();
  const seg = world.segments();
  let total = 0;
  for (const m of seg.mass) total += m;
  let direct = 0;
  for (const n of craft.nodes) direct += n.mass;
  assert.equal(direct, 4600, "the craft's own node masses sum to 4600 exactly");
  // The core derives each mass as 1 / (1 / m), which can differ from m by a rounding, so the
  // exact-sum claim is measured, not assumed: it must hold to 1e-12 relative, and the difference is logged.
  console.log(`# segment masses sum to ${total} (4600 - sum = ${4600 - total})`);
  assert.ok(Math.abs(total - 4600) <= 4600 * 1e-12, `${total}`);
  world.free();
});

test("a segment's centre and inertia match a direct sum in TypeScript within 1e-12", async () => {
  const { world, craft } = await hopperWorld();
  const core = world;
  const check = (label: string) => {
    const pos = core.positions();
    for (let s = 0; s < craft.segments.length; s++) {
      let m = 0;
      let cx = 0;
      let cy = 0;
      let cz = 0;
      for (const [i, n] of craft.nodes.entries()) {
        if (n.segment !== s) continue;
        const mi = 1 / (1 / n.mass);
        m += mi;
        cx += pos[3 * i]! * mi;
        cy += pos[3 * i + 1]! * mi;
        cz += pos[3 * i + 2]! * mi;
      }
      cx /= m;
      cy /= m;
      cz /= m;
      const t = [0, 0, 0, 0, 0, 0];
      for (const [i, n] of craft.nodes.entries()) {
        if (n.segment !== s) continue;
        const mi = 1 / (1 / n.mass);
        const x = pos[3 * i]! - cx;
        const y = pos[3 * i + 1]! - cy;
        const z = pos[3 * i + 2]! - cz;
        t[0]! += mi * (y * y + z * z);
        t[1]! += mi * (x * x + z * z);
        t[2]! += mi * (x * x + y * y);
        t[3]! -= mi * (x * y);
        t[4]! -= mi * (x * z);
        t[5]! -= mi * (y * z);
      }
      const p = core.segmentProps(s)!;
      assert.ok(p, `${label}: segment ${s} has props`);
      // Relative to the size of the thing: |centre| (plus 1 m, since a centre can sit near zero) and the trace.
      const cScale = Math.max(Math.abs(cx), Math.abs(cy), Math.abs(cz), 1);
      const iScale = t[0]! + t[1]! + t[2]!;
      assert.ok(Math.abs(p.centre[0] - cx) <= 1e-12 * cScale && Math.abs(p.centre[1] - cy) <= 1e-12 * cScale && Math.abs(p.centre[2] - cz) <= 1e-12 * cScale, `${label}: segment ${s} centre`);
      for (let k = 0; k < 6; k++) assert.ok(Math.abs(p.inertia[k]! - t[k]!) <= 1e-12 * iScale, `${label}: segment ${s} inertia ${k}: ${p.inertia[k]} against ${t[k]}`);
      assert.equal(p.mass, core.segments().mass[s]);
    }
  };
  check("at rest");
  for (let i = 0; i < 90; i++) core.step();
  check("after 90 steps");
  assert.equal(core.segmentProps(99), null);
  world.free();
});

test("bad segment records are rejected and the rest of the batch applies", async () => {
  const core = await loadCore();
  const world = new World(core);
  const setup = new CommandBatch();
  for (let i = 0; i < 4; i++) setup.addNode(i, 10, 0, 1 / 5, 0.1);
  assert.equal(world.apply(setup).rejected, 0);

  const b = new CommandBatch();
  const rec = {
    root: b.addSegment(NO_SEGMENT, 0, 0, 0), // 0 good: segment 0
    unknownParent: b.addSegment(7, 0, 1, 1), // bad: parent does not exist
    selfParent: b.addSegment(1, 0, 1, 1), // bad: its own number, a cycle of one
    child: b.addSegment(0, 0, 0.5, 0.25), // good: segment 1
    badFlag: b.addSegment(0, 4, 1, 1), // bad: an unknown flag bit
    nanArea: b.addSegment(0, 0, NaN, 1), // bad: area is not a number
    infBending: b.addSegment(0, 0, 1, Infinity), // bad
    negArea: b.addSegment(0, 0, -1, 1), // bad
    first: b.setNodeSegment(0, 0), // good
    again: b.setNodeSegment(0, 1), // bad: node already assigned
    noSegment: b.setNodeSegment(1, 9), // bad: no such segment
    noNode: b.setNodeSegment(99, 0), // bad: no such node
    second: b.setNodeSegment(1, 1), // good
    last: b.addSegment(1, SEG_NEVER_LIVE, 0.1, 0.01), // good: segment 2, the batch carries on to the end
  };
  const bad = [rec.unknownParent, rec.selfParent, rec.badFlag, rec.nanArea, rec.infBending, rec.negArea, rec.again, rec.noSegment, rec.noNode];
  const r = world.apply(b);
  assert.equal(r.rejected, bad.length);
  for (let i = 0; i < b.count; i++) assert.equal(r.results[i] === REJECTED, bad.includes(i), `record ${i}`);
  assert.equal(r.results[rec.child], 1, "rejected segments take no number");
  assert.equal(r.results[rec.last], 2);

  const seg = world.segments();
  assert.deepEqual([...seg.parent], [NO_SEGMENT, 0, 1]);
  assert.equal(seg.seamA.length, 2);
  assert.deepEqual([...seg.nodeSegment], [0, 1, NO_SEGMENT, NO_SEGMENT]);
  assert.deepEqual([...seg.mass], [5, 5, 0]);
  world.free();
});

test("compileCraft without segments is compilePart, byte for byte", () => {
  const flight = prepareFlight(HOPPER);
  const craft = flight.loaded.craft;
  const plain = new CommandBatch();
  compilePart(plain, craft);
  const off = new CommandBatch();
  assert.equal(compileCraft(off, craft, { segments: false }), null);
  assert.deepEqual(off.bytes.slice(), plain.bytes.slice());
  const on = new CommandBatch();
  compileCraft(on, craft);
  assert.equal(on.count, plain.count + craft.segments.length + craft.nodes.length);
  assert.deepEqual(on.bytes.slice(0, plain.bytes.byteLength), plain.bytes.slice(), "segments come after everything else");
});

/** The scripted hop for 1320 steps, with or without segment commands in its setup. */
async function runHop(segments: boolean) {
  const core = await loadCore();
  const hop = prepareHop(HOPPER, { segments });
  const world = new World(core);
  assert.equal(world.apply(hop.flight.setup).rejected, 0);
  for (let s = 0; s < HOP_STEPS; s++) {
    const cmds = new CommandBatch();
    hopCommands(cmds, hop, s);
    world.apply(cmds);
    world.step();
  }
  const out = { state: world.stateHash, physics: world.physicsHash, positions: world.positions(), steps: world.stepCount, segments: world.segments().flags.length };
  world.free();
  return out;
}

test("the live hop with segment commands emitted still ends at the same hash: segments change no physics", async () => {
  const plain = await runHop(false);
  const withSegments = await runHop(true);
  assert.equal(plain.steps, BigInt(HOP_STEPS));
  assert.equal(plain.segments, 0);
  assert.ok(withSegments.segments > 0);
  assert.equal(plain.state, HOP_HASH, "the hop without segments, as before");
  assert.equal(plain.physics, HOP_HASH, "with no segments the two hashes are equal");
  assert.equal(withSegments.physics, HOP_HASH, "with segments, the physics hash is the same");
  assert.deepEqual(withSegments.positions, plain.positions);
  assert.notEqual(withSegments.state, HOP_HASH, "the full hash covers the segments");
  console.log(`# hop with segments: full hash 0x${withSegments.state.toString(16)}, physics hash 0x${withSegments.physics.toString(16)}`);
});

test("the three goldens replay to their hashes", async () => {
  const core = await loadCore();
  for (const name of ["weak-bridge", "tower-collapse", "hard-landing"]) {
    const stored = new Uint8Array(readFileSync(new URL(`./fixtures/${name}.rcrd`, import.meta.url)));
    const rec = decodeRecording(stored);
    const result = replayRecording(() => new World(core), rec);
    assert.equal(result.ok, true, `${name}: ${JSON.stringify(result.mismatch)}`);
    assert.equal(result.hash, rec.finalHash, name);
    if (name === "hard-landing") assert.equal(result.hash, HOP_HASH);
  }
});
