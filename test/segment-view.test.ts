import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";
import { CommandBatch } from "../src/commands.ts";
import { Core } from "../src/core.ts";
import { scenarioFor } from "../src/craft-page.ts";
import {
  ASLEEP_RGB,
  beamModeColours,
  bodyBoxes,
  LIVE_RGB,
  modeCounts,
  RIGID_RGB,
  rootVesselNodes,
  segmentModes,
  vesselBoxes,
  vesselCount,
} from "../src/segment-view.ts";
import { HOP_STEPS } from "../src/landing.ts";
import { World } from "../src/world.ts";

const WASM_PATH = new URL("../target/wasm32-unknown-unknown/release/beam_core.wasm", import.meta.url);
const HOPPER = readFileSync(new URL("./fixtures/hopper.craft.json", import.meta.url), "utf8");
const ROCKET = readFileSync(new URL("./fixtures/tall-rocket.craft.json", import.meta.url), "utf8");

async function loadCore(): Promise<Core> {
  return Core.load(new Uint8Array(readFileSync(WASM_PATH)));
}

function run(world: World, sc: ReturnType<typeof scenarioFor>, steps: number): void {
  assert.equal(world.apply(sc.flight.setup).rejected, 0);
  for (let s = 0; s < steps; s++) {
    const b = new CommandBatch();
    sc.script?.(b, s);
    if (b.count > 0) world.apply(b);
    world.step();
  }
}

test("the hopper's pod and engine are never-live rigid and the rest is live at the start", async () => {
  const core = await loadCore();
  const world = new World(core);
  run(world, scenarioFor("hop", HOPPER, { sleeping: true }), 2);
  const modes = segmentModes(world.segments(), world.sleep());
  const c = modeCounts(modes);
  assert.equal(c.rigid, 2);
  assert.equal(c.asleep, 0);
  assert.equal(c.live, modes.length - 2);
  world.free();
});

test("after the hop's rest the quiet segments show as asleep, and with sleeping off none do", async () => {
  const core = await loadCore();
  const on = new World(core);
  run(on, scenarioFor("hop", HOPPER, { sleeping: true }), HOP_STEPS);
  const asleep = modeCounts(segmentModes(on.segments(), on.sleep())).asleep;
  assert.ok(asleep > 0, `asleep ${asleep}`);
  const off = new World(core);
  run(off, scenarioFor("hop", HOPPER, { sleeping: false }), HOP_STEPS);
  assert.equal(modeCounts(segmentModes(off.segments(), off.sleep())).asleep, 0);
  on.free();
  off.free();
});

test("a beam takes the colour of its livest end, and a node with no segment counts as live", async () => {
  const modes = ["live", "asleep", "rigid"] as const;
  // nodes 0..3 in segments 0, 1, 2 and none.
  const nodeSegment = [0, 1, 2, 0xffffffff];
  const ends = [0, 1, 1, 2, 2, 3, 2, 2];
  const out = new Uint8Array(12);
  beamModeColours(ends, nodeSegment, modes, out);
  assert.deepEqual([...out.subarray(0, 3)], LIVE_RGB);
  assert.deepEqual([...out.subarray(3, 6)], ASLEEP_RGB);
  assert.deepEqual([...out.subarray(6, 9)], LIVE_RGB);
  assert.deepEqual([...out.subarray(9, 12)], RIGID_RGB);
});

test("each live body's box holds its nodes, and one vessel has no vessel box and no root subset", async () => {
  const core = await loadCore();
  const world = new World(core);
  run(world, scenarioFor("hop", HOPPER, { sleeping: true }), 20);
  const bodies = world.bodies();
  const pos = world.positions();
  const { ids, boxes } = bodyBoxes(bodies, pos, 0);
  assert.ok(ids.length >= 2);
  for (const [k, id] of ids.entries()) {
    for (const [n, b] of bodies.nodeBody.entries()) {
      if (b !== id) continue;
      for (let j = 0; j < 3; j++) {
        assert.ok(pos[3 * n + j]! >= boxes[6 * k + j]! - 1e-6 && pos[3 * n + j]! <= boxes[6 * k + 3 + j]! + 1e-6);
      }
    }
  }
  const vessels = world.vessels();
  assert.equal(vesselCount(vessels), 1);
  assert.equal(vesselBoxes(vessels, pos).ids.length, 0);
  assert.equal(rootVesselNodes(vessels), null);
  world.free();
});

test("the tall rocket sits on its clamp with sleeping segments, as one vessel", async () => {
  const core = await loadCore();
  const world = new World(core);
  const sc = scenarioFor("rocket", HOPPER, { sleeping: true, rocketText: ROCKET });
  run(world, sc, 300);
  const c = modeCounts(segmentModes(world.segments(), world.sleep()));
  assert.ok(c.asleep > 0, "the rocket on the clamp should have segments asleep");
  assert.ok(c.rigid >= 2);
  assert.equal(vesselCount(world.vessels()), 1);
  world.free();
});

test("the hard landing's snapped leg becomes its own vessel and the root keeps the rest", async () => {
  const core = await loadCore();
  const world = new World(core);
  run(world, scenarioFor("hop", HOPPER, { sleeping: true }), HOP_STEPS);
  const vessels = world.vessels();
  const n = vesselCount(vessels);
  assert.ok(n >= 2, `vessels ${n}`);
  const root = rootVesselNodes(vessels)!;
  assert.ok(root.length > world.nodeCount / 2, "the root vessel is most of the craft");
  assert.ok(root.includes([...vessels.nodeVessel].findIndex((v) => v !== 0xffffffff)), "it holds the lowest live node");
  assert.equal(vesselBoxes(vessels, world.positions()).ids.length, n);
  world.free();
});
