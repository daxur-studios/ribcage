import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";
import { CommandBatch } from "../src/commands.ts";
import { Core, EVENT_BREAK, EVENT_FREEZE, EVENT_YIELD } from "../src/core.ts";
import { STANDARD_TRUCK } from "../src/bridge.ts";
import {
  ACTION_START_STEP,
  HEAVY_TRUCK,
  MEMBER,
  bridgeAction,
  buildBridge3,
  buildTower,
  newRides,
  towerAction,
  type ActionKind,
  type BridgeRides,
} from "../src/scenes3d.ts";
import {
  VALLEY_LEDGE_LEFT,
  VALLEY_LEDGE_RIGHT,
  PLAIN_FLAT_HALF,
  heightAt,
  plainTerrain,
  valleyTerrain,
  type Terrain,
} from "../src/terrain.ts";
import { World } from "../src/world.ts";

const WASM_PATH = new URL("../target/wasm32-unknown-unknown/release/beam_core.wasm", import.meta.url);

async function loadCore(): Promise<Core> {
  try {
    return await Core.load(new Uint8Array(readFileSync(WASM_PATH)));
  } catch (e) {
    throw new Error(`beam_core.wasm is missing or stale: run \`pnpm build:wasm\` first (${String(e)})`);
  }
}

type Scene = "tower" | "bridge";

/** Below this speed (m/s, from the change in position over one step) a node counts as at rest. */
const REST_SPEED = 1e-3;
/**
 * The core clamps a node at 1000 m/s (`DEFAULT_MAX_SPEED`) but does not report it, so the test
 * watches the speed it can see and asks for a wide margin below the clamp.
 */
const SPEED_LIMIT = 500;

interface Run {
  world: World;
  terrain: Terrain;
  /** Bytes of every command list in order: the build, then one per step. */
  lists: Uint8Array[];
  yields: number;
  breaks: number;
  freezes: number;
  firstBreak: number;
  /** Last step at which any node moved faster than `REST_SPEED`, and its speed at the last step. */
  lastMoving: number;
  endSpeed: number;
  maxSpeed: number;
  maxSubsteps: number;
  /** Lowest `node y - ground height` seen at any node at the end of any step. */
  lowestGap: number;
  /** The bridge's trucks: which lost their footing, and where. */
  rides: BridgeRides;
}

function runScene(core: Core, scene: Scene, kind: ActionKind | null, seconds: number): Run {
  const world = new World(core);
  const build = new CommandBatch();
  const layout = scene === "tower" ? buildTower(build) : buildBridge3(build);
  assert.equal(world.apply(build).rejected, 0);
  const run: Run = {
    world,
    terrain: layout.terrain,
    lists: [build.bytes.slice()],
    yields: 0,
    breaks: 0,
    freezes: 0,
    firstBreak: -1,
    lastMoving: -1,
    endSpeed: 0,
    maxSpeed: 0,
    maxSubsteps: 0,
    lowestGap: Infinity,
    rides: newRides(),
  };
  let prev = world.positions();
  const steps = Math.round(seconds * 60);
  const rides = run.rides;
  for (let s = 0; s < steps; s++) {
    const cmds = new CommandBatch();
    if (scene === "tower") {
      if (kind !== null) towerAction(cmds, layout as ReturnType<typeof buildTower>, kind, s);
    } else {
      bridgeAction(cmds, layout as ReturnType<typeof buildBridge3>, kind, s, STANDARD_TRUCK, rides, prev);
    }
    assert.equal(world.apply(cmds).rejected, 0, `list rejected at step ${s}`);
    run.lists.push(cmds.bytes.slice());
    run.maxSubsteps = Math.max(run.maxSubsteps, world.step());
    for (const e of world.events()) {
      if (e.kind === EVENT_YIELD) run.yields++;
      else if (e.kind === EVENT_BREAK) {
        run.breaks++;
        if (run.firstBreak < 0) run.firstBreak = s;
      } else if (e.kind === EVENT_FREEZE) run.freezes++;
    }
    const p = world.positions();
    let fastest = 0;
    for (let i = 0; i < p.length; i += 3) {
      const dx = p[i]! - prev[i]!;
      const dy = p[i + 1]! - prev[i + 1]!;
      const dz = p[i + 2]! - prev[i + 2]!;
      fastest = Math.max(fastest, Math.sqrt(dx * dx + dy * dy + dz * dz) * 60);
      run.lowestGap = Math.min(run.lowestGap, p[i + 1]! - heightAt(layout.terrain, p[i]!, p[i + 2]!));
    }
    if (fastest >= REST_SPEED) run.lastMoving = s;
    run.maxSpeed = Math.max(run.maxSpeed, fastest);
    run.endSpeed = fastest;
    prev = p;
  }
  return run;
}

const secondsOf = (step: number): number => step / 60;

for (const scene of ["tower", "bridge"] as const) {
  test(`the ${scene} stands for 20 s: no yield, no break, then at rest`, async () => {
    const core = await loadCore();
    // The bridge carries the standard truck across (6 s) in this run.
    const run = runScene(core, scene, null, 20);
    assert.equal(run.yields, 0, "no beam yields");
    assert.equal(run.breaks, 0, "no beam breaks");
    assert.equal(run.freezes, 0, "no island freezes");
    assert.ok(run.endSpeed < REST_SPEED, `still moving at ${run.endSpeed} m/s`);
    assert.ok(run.lowestGap > -MEMBER.radius - 0.01, `a node went ${-run.lowestGap} m below the ground`);
    run.world.free();
  });
}

/** How long after the action starts, in seconds, a collapse has to be at rest by. */
const REST_BY: Record<Scene, number> = { tower: 35, bridge: 50 };
const RUN_SECONDS: Record<Scene, number> = { tower: 45, bridge: 60 };
const ACTIONS: ActionKind[] = ["knock", "removeColumn", "overload"];

for (const scene of ["tower", "bridge"] as const) {
  for (const kind of ACTIONS) {
    test(`the ${scene} collapses under ${kind}, stays above ground, and comes to rest`, async () => {
      const core = await loadCore();
      const run = runScene(core, scene, kind, RUN_SECONDS[scene]);
      assert.ok(run.breaks >= 1, `no beam broke (${run.yields} yields)`);
      assert.ok(run.firstBreak >= ACTION_START_STEP, `a beam broke at step ${run.firstBreak}, before the action`);
      // No node below the ground by more than its radius plus a centimetre.
      assert.ok(run.lowestGap > -MEMBER.radius - 0.01, `lowest node ${run.lowestGap} m above ground`);
      assert.equal(run.freezes, 0, "no island freezes");
      assert.ok(run.maxSpeed < SPEED_LIMIT, `fastest node ${run.maxSpeed} m/s is near the clamp`);
      assert.ok(run.maxSubsteps <= 32, `${run.maxSubsteps} substeps`);
      const restedBy = secondsOf(run.lastMoving) - secondsOf(ACTION_START_STEP);
      assert.ok(restedBy < REST_BY[scene], `still moving ${restedBy.toFixed(1)} s after the action started`);
      assert.ok(run.endSpeed < REST_SPEED, `still moving at ${run.endSpeed} m/s at the end`);
      run.world.free();
    });
  }
}

test("the overload truck breaks the bridge with the truck at the middle of the span, and then falls with it", async () => {
  const core = await loadCore();
  const run = runScene(core, "bridge", "overload", 20);
  assert.ok(run.firstBreak >= ACTION_START_STEP, "a beam broke");
  // The truck starts at the left end at ACTION_START_STEP; the deck is 24 m long, the middle at 12 m.
  const x = (HEAVY_TRUCK.speed * (run.firstBreak - ACTION_START_STEP)) / 60;
  assert.ok(x > 9 && x < 15, `first break with the truck at ${x.toFixed(1)} m of 24`);
  const fell = run.rides.heavy.fell;
  assert.ok(fell !== null, "the truck lost its footing");
  assert.ok(fell.x > 9 && fell.x < 15, `the truck fell at x = ${fell.x.toFixed(1)}`);
  // From then its weight is gone: every later list is empty (nothing else acts in this run).
  const after = run.lists.slice(1 + ACTION_START_STEP + fell.k + 1);
  assert.ok(after.length > 600, "a long stretch after the fall");
  assert.ok(after.every((l) => CommandBatch.fromBytes(l).count === 0), "no force commands after the truck fell");
  run.world.free();
});

test("a truck that the deck holds never loses its footing", async () => {
  const core = await loadCore();
  const run = runScene(core, "bridge", null, 12);
  assert.equal(run.rides.light.fell, null);
  run.world.free();
});

for (const [scene, kind] of [["tower", "knock"], ["bridge", "overload"]] as const) {
  test(`the ${scene} ${kind} gives the same hash twice, and a replay of the saved lists matches`, async () => {
    const core = await loadCore();
    const a = runScene(core, scene, kind, 12);
    const b = runScene(core, scene, kind, 12);
    assert.equal(a.world.stateHash, b.world.stateHash);
    assert.deepEqual(a.lists, b.lists);

    const replay = new World(core);
    for (const [i, bytes] of a.lists.entries()) {
      assert.equal(replay.apply(CommandBatch.fromBytes(bytes)).rejected, 0);
      if (i > 0) replay.step(); // list 0 is the build, the rest are one per step
    }
    assert.equal(replay.stateHash, a.world.stateHash);
    for (const w of [a.world, b.world, replay]) w.free();
  });
}

test("the terrains are exact where the scenes rely on them", () => {
  const plain = plainTerrain();
  for (let x = -PLAIN_FLAT_HALF; x <= PLAIN_FLAT_HALF; x += 0.75) {
    for (let z = -PLAIN_FLAT_HALF; z <= PLAIN_FLAT_HALF; z += 0.75) assert.equal(heightAt(plain, x, z), 0, `plain at ${x}, ${z}`);
  }
  assert.ok(plain.heights.some((h) => h > 0.3), "the plain has hills");

  const valley = valleyTerrain();
  for (let z = -15; z <= 15; z += 1.5) {
    for (let x = -12; x <= VALLEY_LEDGE_LEFT; x += 1) assert.equal(heightAt(valley, x, z), 0, `left ledge at ${x}, ${z}`);
    for (let x = VALLEY_LEDGE_RIGHT; x <= 48; x += 1) assert.equal(heightAt(valley, x, z), 0, `right ledge at ${x}, ${z}`);
  }
  assert.ok(heightAt(valley, 12, 0) < -3, "the valley floor is well below the ledges");
  // The same arithmetic gives the same grid every time.
  assert.deepEqual(valleyTerrain().heights, valley.heights);
  assert.deepEqual(plainTerrain().heights, plain.heights);
});
