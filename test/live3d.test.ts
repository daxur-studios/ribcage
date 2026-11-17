import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";
import { CommandBatch } from "../src/commands.ts";
import { Core, EVENT_BREAK } from "../src/core.ts";
import { STANDARD_TRUCK } from "../src/bridge.ts";
import { lerpPositions, pickNode } from "../src/render3d.ts";
import {
  ACTION_START_STEP,
  bridgeAction,
  buildBridge3,
  buildTower,
  liveBridgeCommands,
  liveTowerCommands,
  towerAction,
  type ActionKind,
  type ArmedAction,
} from "../src/scenes3d.ts";
import { World } from "../src/world.ts";

const WASM_PATH = new URL("../target/wasm32-unknown-unknown/release/beam_core.wasm", import.meta.url);

async function loadCore(): Promise<Core> {
  return Core.load(new Uint8Array(readFileSync(WASM_PATH)));
}

test("interpolating between two render buffers: the ends, the middle, held to 0..1", () => {
  const prev = Float32Array.from([0, 0, 0, 10, 20, 30]);
  const curr = Float32Array.from([2, 4, 6, 10, 20, 30]);
  const out = new Float32Array(6);
  lerpPositions(prev, curr, 0, out);
  assert.deepEqual([...out], [...prev]);
  lerpPositions(prev, curr, 1, out);
  assert.deepEqual([...out], [...curr]);
  lerpPositions(prev, curr, 0.25, out);
  assert.deepEqual([...out], [0.5, 1, 1.5, 10, 20, 30]); // the still node stays exactly where it is
  lerpPositions(prev, curr, 7, out);
  assert.deepEqual([...out], [...curr]);
  lerpPositions(prev, curr, -3, out);
  assert.deepEqual([...out], [...prev]);
  // A node with no earlier place (a shorter `prev`) is drawn where it is.
  lerpPositions(Float32Array.from([0, 0, 0]), curr, 0.5, out);
  assert.deepEqual([...out].slice(3), [10, 20, 30]);
});

test("picking a node: the nearest sphere in front, none when missed, behind, or gone", () => {
  // Three nodes along +x at 5, 8 and 8 (gone), and one off to the side.
  const p = Float32Array.from([5, 0, 0, 8, 0, 0, 8, 0, 0, 5, 3, 0]);
  const alive = Uint32Array.from([1, 1, 0, 1]);
  assert.equal(pickNode(p, alive, 0.4, 0, 0, 0, 1, 0, 0), 0, "the nearer of two in line");
  assert.equal(pickNode(p, alive, 0.4, 6, 0, 0, 1, 0, 0), 1, "the one ahead once the first is behind");
  assert.equal(pickNode(p, alive, 0.4, 9, 0, 0, 1, 0, 0), -1, "nothing ahead");
  assert.equal(pickNode(p, alive, 0.4, 0, 0.5, 0, 1, 0, 0), -1, "a ray 0.5 off misses a 0.4 sphere");
  assert.equal(pickNode(p, alive, 0.4, 0, 0.39, 0, 1, 0, 0), 0, "a ray 0.39 off grazes it");
  assert.equal(pickNode(p, alive, 0.4, 5, 10, 0, 0, -1, 0), 3, "straight down onto the one above");
  assert.equal(pickNode(p, alive, 0.4, 5.1, 0, 0, 0, 1, 0), 0, "origin inside a sphere picks it");
  // A sloped ray: from (0, 3, 0) aimed at (5, 0, 0).
  const len = Math.hypot(5, -3);
  assert.equal(pickNode(p, alive, 0.2, 0, 3, 0, 5 / len, -3 / len, 0), 0);
});

/** The bytes of a command list as a string, for comparing lists. */
const bytesOf = (fill: (b: CommandBatch) => void): string => {
  const b = new CommandBatch();
  fill(b);
  return Buffer.from(b.bytes).toString("hex");
};

for (const kind of ["knock", "removeColumn", "overload"] as ActionKind[]) {
  test(`live tower ${kind}: armed at the standard start it is the headless action; armed later it is shifted`, () => {
    const layout = buildTower(new CommandBatch());
    const at = (start: number): ArmedAction => ({ kind, start });
    for (let step = 0; step < 420; step++) {
      assert.equal(
        bytesOf((b) => liveTowerCommands(b, layout, at(ACTION_START_STEP), step)),
        bytesOf((b) => towerAction(b, layout, kind, step)),
        `step ${step}`,
      );
      assert.equal(
        bytesOf((b) => liveTowerCommands(b, layout, at(ACTION_START_STEP + 50), step + 50)),
        bytesOf((b) => towerAction(b, layout, kind, step)),
        `shifted, step ${step}`,
      );
    }
    assert.equal(bytesOf((b) => liveTowerCommands(b, layout, null, 300)), "", "nothing armed, nothing sent");
  });
}

for (const kind of ["knock", "removeColumn"] as ActionKind[]) {
  test(`live bridge ${kind}: the truck crosses as usual, the action is the headless one at its start`, () => {
    const layout = buildBridge3(new CommandBatch());
    for (let step = 0; step < 420; step++) {
      assert.equal(
        bytesOf((b) => liveBridgeCommands(b, layout, { kind, start: ACTION_START_STEP }, step, STANDARD_TRUCK)),
        bytesOf((b) => bridgeAction(b, layout, kind, step, STANDARD_TRUCK)),
        `step ${step}`,
      );
    }
    for (let step = 0; step < 420; step++) {
      assert.equal(
        bytesOf((b) => liveBridgeCommands(b, layout, null, step, STANDARD_TRUCK)),
        bytesOf((b) => bridgeAction(b, layout, null, step, STANDARD_TRUCK)),
        `plain, step ${step}`,
      );
    }
  });
}

test("live bridge overload: the light truck until the press, then the heavy one as the headless action", () => {
  const layout = buildBridge3(new CommandBatch());
  const armed: ArmedAction = { kind: "overload", start: 200 };
  for (let step = 0; step < 200; step++) {
    assert.equal(
      bytesOf((b) => liveBridgeCommands(b, layout, armed, step, STANDARD_TRUCK)),
      bytesOf((b) => bridgeAction(b, layout, null, step, STANDARD_TRUCK)),
      `before, step ${step}`,
    );
  }
  for (let step = 200; step < 500; step++) {
    assert.equal(
      bytesOf((b) => liveBridgeCommands(b, layout, armed, step, STANDARD_TRUCK)),
      bytesOf((b) => bridgeAction(b, layout, "overload", step - 200 + ACTION_START_STEP, STANDARD_TRUCK)),
      `after, step ${step}`,
    );
  }
});

for (const [scene, node] of [["tower", 24], ["bridge", 8]] as const) {
  test(`an aimed knock on the ${scene}'s node ${node} is deterministic and brings beams down`, async () => {
    const core = await loadCore();
    const hashes: bigint[] = [];
    let breaks = 0;
    for (let rep = 0; rep < 2; rep++) {
      const world = new World(core);
      const build = new CommandBatch();
      const layout = scene === "tower" ? buildTower(build) : buildBridge3(build);
      assert.equal(world.apply(build).rejected, 0);
      const armed: ArmedAction = { kind: "knock", start: 90, aim: { node, dx: 0.6, dz: 0.8 } };
      for (let s = 0; s < 1200; s++) {
        const cmds = new CommandBatch();
        if (scene === "tower") liveTowerCommands(cmds, layout as ReturnType<typeof buildTower>, armed, s);
        else liveBridgeCommands(cmds, layout as ReturnType<typeof buildBridge3>, armed, s, STANDARD_TRUCK);
        assert.equal(world.apply(cmds).rejected, 0);
        world.step();
        for (const e of world.events()) if (e.kind === EVENT_BREAK && rep === 0) breaks++;
      }
      hashes.push(world.stateHash);
      world.free();
    }
    assert.equal(hashes[0], hashes[1]);
    assert.ok(breaks >= 1, `no beam broke under the aimed knock`);
  });
}
