import assert from "node:assert/strict";
import { existsSync, readFileSync, writeFileSync } from "node:fs";
import test from "node:test";
import { CommandBatch } from "../src/commands.ts";
import { Core } from "../src/core.ts";
import { bridgeDesign, STRONG_BRIDGE, WEAK_BRIDGE, type BridgeSpec } from "../src/bridge.ts";
import { compileDesign, deckOf, rollLoad, STANDARD_TRUCK } from "../src/design.ts";
import {
  decodeRecording,
  encodeRecording,
  Player,
  Recorder,
  replayRecording,
  type Recording,
} from "../src/recording.ts";
import { World } from "../src/world.ts";

const WASM_PATH = new URL("../target/wasm32-unknown-unknown/release/beam_core.wasm", import.meta.url);
const FIXTURE = new URL("./fixtures/weak-bridge.rcrd", import.meta.url);

/** The weak bridge after 600 steps with the truck on: the same hash the editor shows in a browser (ER 002, phase 2.3). */
const WEAK_600 = 0xb580a6d8faa6610dn;

async function loadCore(): Promise<Core> {
  return Core.load(new Uint8Array(readFileSync(WASM_PATH)));
}

function recordBridge(core: Core, spec: BridgeSpec, steps: number): { rec: Recording; hash: bigint } {
  const world = new World(core);
  const design = bridgeDesign(spec);
  const deck = deckOf(design);
  const recorder = new Recorder(world);
  const build = new CommandBatch();
  compileDesign(build, design, spec);
  assert.equal(recorder.setup(build).rejected, 0);
  for (let s = 0; s < steps; s++) {
    const cmds = new CommandBatch();
    rollLoad(cmds, deck, STANDARD_TRUCK, s);
    recorder.step(cmds);
  }
  const rec = recorder.finish();
  const hash = world.stateHash;
  world.free();
  return { rec, hash };
}

test("a recorded run replays to the same hash, and the file round-trips byte for byte", async () => {
  const core = await loadCore();
  const { rec, hash } = recordBridge(core, WEAK_BRIDGE, 600);
  assert.equal(hash, WEAK_600);
  assert.equal(rec.steps, 600);
  assert.equal(rec.lists.length, 601);
  assert.equal(rec.checkpoints.length, 10);
  assert.equal(rec.finalHash, hash);

  const result = replayRecording(() => new World(core), rec);
  assert.equal(result.ok, true);
  assert.equal(result.hash, hash);
  assert.equal(result.steps, 600);

  const bytes = encodeRecording(rec);
  const back = decodeRecording(bytes);
  assert.deepEqual(encodeRecording(back), bytes);
  assert.equal(replayRecording(() => new World(core), back).hash, hash);
});

test("the committed recording still replays to its hash, and recording it again gives the same file", async () => {
  const core = await loadCore();
  const fresh = encodeRecording(recordBridge(core, WEAK_BRIDGE, 600).rec);
  if (process.env.RIBCAGE_UPDATE_FIXTURE) writeFileSync(FIXTURE, fresh);
  assert.ok(existsSync(FIXTURE), "fixture missing: run once with RIBCAGE_UPDATE_FIXTURE=1");
  const stored = new Uint8Array(readFileSync(FIXTURE));

  const result = replayRecording(() => new World(core), decodeRecording(stored));
  assert.equal(result.ok, true);
  assert.equal(result.hash, WEAK_600);
  // The file's own bytes: the format and the recorded run are both stable. If this fails after a
  // deliberate solver change, regenerate with RIBCAGE_UPDATE_FIXTURE=1 and log why in the ER.
  // Same bytes except the ABI field: the stored file was made at ABI 2, before the terrain
  // commands (ABI 3 only added command kinds), and replaying it above is the real check.
  const abiBytes = stored.subarray(8, 12);
  const freshAtStoredAbi = fresh.slice();
  freshAtStoredAbi.set(abiBytes, 8);
  assert.deepEqual(freshAtStoredAbi, stored);
});

test("a changed command is caught at the first checkpoint after it", async () => {
  const core = await loadCore();
  const { rec } = recordBridge(core, STRONG_BRIDGE, 300);
  // Make the truck 1 kN heavier for one step: list 200 is applied before step 199 (0-based).
  const lists = rec.lists.map((l) => l.slice());
  const target = lists[200]!;
  assert.ok(target.byteLength >= 64, "the truck is on the bridge at step 199");
  new DataView(target.buffer).setFloat64(16 + 8 * 1, -1000, true); // first record's fy
  const tampered: Recording = { ...rec, lists };
  const result = replayRecording(() => new World(core), tampered);
  assert.equal(result.ok, false);
  assert.equal(result.mismatch?.step, 240); // the first checkpoint at or after step 200
  assert.notEqual(result.mismatch?.actual, result.mismatch?.expected);
});

test("a player can be stepped by hand, as the editor does", async () => {
  const core = await loadCore();
  const { rec } = recordBridge(core, STRONG_BRIDGE, 130);
  const world = new World(core);
  const p = new Player(world, rec);
  assert.throws(() => p.advance(), /start first/);
  assert.equal(p.start().rejected, 0);
  assert.equal(p.step, 0);
  let n = 0;
  while (!p.done) {
    assert.ok(p.advance() >= 4);
    n++;
  }
  assert.equal(n, 130);
  assert.equal(p.advance(), -1);
  assert.equal(p.mismatch, null);
  assert.equal(world.stateHash, rec.finalHash);
  world.free();
});

test("a recording with no steps and no setup is still a valid file", async () => {
  const core = await loadCore();
  const world = new World(core);
  const rec = new Recorder(world).finish();
  world.free();
  assert.equal(rec.steps, 0);
  const back = decodeRecording(encodeRecording(rec));
  const result = replayRecording(() => new World(core), back);
  assert.equal(result.ok, true);
  assert.equal(result.steps, 0);
});

test("anything that is not exactly a recording is refused, with a reason", async () => {
  const core = await loadCore();
  const good = encodeRecording(recordBridge(core, STRONG_BRIDGE, 10).rec);
  const withByte = (i: number, value: number): Uint8Array => {
    const c = good.slice();
    c[i] = value;
    return c;
  };
  assert.throws(() => decodeRecording(good.slice(0, 10)), /too short/);
  assert.throws(() => decodeRecording(withByte(0, 0x58)), /first bytes/);
  assert.throws(() => decodeRecording(withByte(4, 9)), /format 9/);
  assert.throws(() => decodeRecording(withByte(8, 99)), /core ABI 99/);
  assert.throws(() => decodeRecording(withByte(8, 1)), /core ABI 1, this build reads ABI 2 to 3/);
  assert.throws(() => decodeRecording(withByte(8, 4)), /core ABI 4/);
  // ABI 2 (before the terrain commands) is still read; ABI 3 is the current one.
  assert.equal(decodeRecording(withByte(8, 2)).steps, 10);
  assert.equal(decodeRecording(withByte(8, 3)).steps, 10);
  assert.throws(() => decodeRecording(withByte(12, 32)), /32-byte records/);
  assert.throws(() => decodeRecording(good.slice(0, good.byteLength - 1)), /truncated/);
  const extra = new Uint8Array(good.byteLength + 1);
  extra.set(good);
  assert.throws(() => decodeRecording(extra), /extra bytes/);
  assert.throws(() => decodeRecording(withByte(20, 0)), /inconsistent/);
});

test("the recorder refuses a second setup and a bad interval", async () => {
  const core = await loadCore();
  const world = new World(core);
  assert.throws(() => new Recorder(world, 0), /whole number/);
  const r = new Recorder(world);
  r.setup(new CommandBatch());
  assert.throws(() => r.setup(new CommandBatch()), /first, once/);
  world.free();
});
