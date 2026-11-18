import assert from "node:assert/strict";
import { existsSync, readFileSync, writeFileSync } from "node:fs";
import test from "node:test";
import { CommandBatch } from "../src/commands.ts";
import { Core } from "../src/core.ts";
import { STANDARD_TRUCK } from "../src/bridge.ts";
import { decodeRecording, encodeRecording, Recorder, replayRecording, type Recording } from "../src/recording.ts";
import {
  ACTION_START_STEP,
  buildBridge3,
  buildTower,
  identifyScene,
  liveBridgeCommands,
  liveTowerCommands,
  towerAction,
  type ArmedAction,
  type BridgeLayout3,
  type TowerLayout,
} from "../src/scenes3d.ts";
import { World } from "../src/world.ts";

const WASM_PATH = new URL("../target/wasm32-unknown-unknown/release/beam_core.wasm", import.meta.url);
const FIXTURE = new URL("./fixtures/tower-collapse.rcrd", import.meta.url);

/** The tower knocked at step 120 and run for 600 steps (10 s): the hash the page shows too. */
const TOWER_COLLAPSE_600 = 0xa33aaf9bc3c996dan;

async function loadCore(): Promise<Core> {
  return Core.load(new Uint8Array(readFileSync(WASM_PATH)));
}

/** Records the tower with the headless knock (as the page's `?action=knock` does) for `steps` steps. */
function recordTowerKnock(core: Core, steps: number): { rec: Recording; hash: bigint } {
  const world = new World(core);
  const recorder = new Recorder(world);
  const build = new CommandBatch();
  const layout = buildTower(build);
  assert.equal(recorder.setup(build).rejected, 0);
  for (let s = 0; s < steps; s++) {
    const cmds = new CommandBatch();
    towerAction(cmds, layout, "knock", s);
    recorder.step(cmds);
  }
  const rec = recorder.finish();
  const hash = world.stateHash;
  world.free();
  return { rec, hash };
}

/** Records a live run the way the page does: its commands come from `liveTowerCommands` or `liveBridgeCommands`. */
function recordLive(core: Core, scene: "tower" | "bridge", armed: ArmedAction | null, steps: number): { rec: Recording; hash: bigint } {
  const world = new World(core);
  const recorder = new Recorder(world);
  const build = new CommandBatch();
  const layout = scene === "tower" ? buildTower(build) : buildBridge3(build);
  assert.equal(recorder.setup(build).rejected, 0);
  for (let s = 0; s < steps; s++) {
    const cmds = new CommandBatch();
    if (scene === "tower") liveTowerCommands(cmds, layout as TowerLayout, armed, s);
    else liveBridgeCommands(cmds, layout as BridgeLayout3, armed, s, STANDARD_TRUCK);
    recorder.step(cmds);
  }
  const rec = recorder.finish();
  const hash = world.stateHash;
  world.free();
  return { rec, hash };
}

test("the committed tower collapse replays to its hash, and recording it again gives the same file", async () => {
  const core = await loadCore();
  const { rec, hash } = recordTowerKnock(core, 600);
  const fresh = encodeRecording(rec);
  if (process.env.RIBCAGE_UPDATE_FIXTURE) writeFileSync(FIXTURE, fresh);
  assert.ok(existsSync(FIXTURE), "fixture missing: run once with RIBCAGE_UPDATE_FIXTURE=1");
  const stored = new Uint8Array(readFileSync(FIXTURE));
  assert.ok(stored.byteLength < 150 * 1024, `the fixture is ${stored.byteLength} bytes, over the 150 KB budget`);

  const result = replayRecording(() => new World(core), decodeRecording(stored));
  assert.equal(result.ok, true);
  assert.equal(result.steps, 600);
  assert.equal(result.hash, hash);
  assert.equal(result.hash, TOWER_COLLAPSE_600);
  // The bytes too: the format, the scene and the run are all stable. After a deliberate solver or
  // scene change, regenerate with RIBCAGE_UPDATE_FIXTURE=1 and log why in the ER.
  // Same bytes except the ABI field: the stored file was made at ABI 3, before the thrusters
  // (ABI 4 only added command kinds), and replaying it above is the real check.
  const atStoredAbi = fresh.slice();
  atStoredAbi.set(stored.subarray(8, 12), 8);
  assert.deepEqual(atStoredAbi, stored);
});

test("the golden's setup is the standard tower, so the page can draw it", async () => {
  const core = await loadCore();
  const rec = decodeRecording(new Uint8Array(readFileSync(FIXTURE)));
  const found = identifyScene(rec.lists[0]!);
  assert.equal(found?.scene, "tower");
  assert.equal((found?.layout as TowerLayout).nodeCount, 28);
  const bridge = new CommandBatch();
  buildBridge3(bridge);
  assert.equal(identifyScene(bridge.bytes)?.scene, "bridge");
  assert.equal(identifyScene(new Uint8Array(0)), null);
  const other = rec.lists[0]!.slice();
  other[other.length - 1]! ^= 1;
  assert.equal(identifyScene(other), null, "a setup that differs by one bit is not the standard scene");
  void core;
});

test("a changed command in the golden is caught at the first checkpoint after it", async () => {
  const core = await loadCore();
  const rec = decodeRecording(new Uint8Array(readFileSync(FIXTURE)));
  // The knock pushes from step 120 on; make list 150 (applied before step 149) push 1 kN harder.
  const lists = rec.lists.map((l) => l.slice());
  const target = lists[150]!;
  assert.ok(target.byteLength >= 64, "the knock is still pushing at step 149");
  new DataView(target.buffer, target.byteOffset).setFloat64(16, new DataView(target.buffer, target.byteOffset).getFloat64(16, true) + 1000, true);
  const result = replayRecording(() => new World(core), { ...rec, lists });
  assert.equal(result.ok, false);
  assert.equal(result.mismatch?.step, 180, "the first checkpoint at or after step 150");
});

for (const [scene, armed] of [
  ["tower", { kind: "knock", start: 90, aim: { node: 24, dx: 0.6, dz: 0.8 } }],
  ["tower", { kind: "removeColumn", start: 200 }],
  ["bridge", { kind: "overload", start: 150 }],
  ["bridge", null],
] as [("tower" | "bridge"), ArmedAction | null][]) {
  test(`a live ${scene} run (${armed?.kind ?? "plain"}) as the page records it replays to its hash, through the file format`, async () => {
    const core = await loadCore();
    const { rec, hash } = recordLive(core, scene, armed, 480);
    assert.equal(identifyScene(rec.lists[0]!)?.scene, scene);
    const back = decodeRecording(encodeRecording(rec));
    const result = replayRecording(() => new World(core), back);
    assert.equal(result.ok, true);
    assert.equal(result.hash, hash);
    assert.equal(result.hash, rec.finalHash);
  });
}

test("recording the page's live run with the standard action start equals the headless recording", async () => {
  const core = await loadCore();
  const live = recordLive(core, "tower", { kind: "knock", start: ACTION_START_STEP }, 300);
  const headless = recordTowerKnock(core, 300);
  assert.deepEqual(encodeRecording(live.rec), encodeRecording(headless.rec));
  assert.equal(live.hash, 0x3e54ed9ad74469f3n);
});
