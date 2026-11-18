import assert from "node:assert/strict";
import { existsSync, readFileSync, writeFileSync } from "node:fs";
import test from "node:test";
import { CommandBatch } from "../src/commands.ts";
import { Core, EVENT_BREAK, EVENT_FREEZE, EVENT_YIELD } from "../src/core.ts";
import type { Flight } from "../src/flight.ts";
import {
  HOP_STEPS,
  PAD,
  ROUGH_SPOTS,
  centreOf,
  hopCommands,
  joinedLegs,
  partNodes,
  prepareDrop,
  prepareHop,
  tiltDegrees,
  type Spot,
} from "../src/landing.ts";
import { decodeRecording, encodeRecording, Recorder, replayRecording } from "../src/recording.ts";
import { heightAt } from "../src/terrain.ts";
import { World } from "../src/world.ts";

const WASM_PATH = new URL("../target/wasm32-unknown-unknown/release/beam_core.wasm", import.meta.url);
const HOPPER = readFileSync(new URL("./fixtures/hopper.craft.json", import.meta.url), "utf8");
const FIXTURE = new URL("./fixtures/hard-landing.rcrd", import.meta.url);

async function loadCore(): Promise<Core> {
  return Core.load(new Uint8Array(readFileSync(WASM_PATH)));
}

/** Steps a drop is run for: 20 s. Touchdown is within the first second, so the last 2 s are 17 to 19 s after it. */
const STEPS = 1200;
/** "At rest": no body node faster than this, in metres per second, in the rest window. */
const REST_SPEED = 0.25;

interface Landing {
  touchStep: number;
  steps: number;
  yields: number;
  broken: Set<number>;
  freezes: number;
  /** Deepest a node's surface went below the ground, in metres (negative when never below). */
  deepest: number;
  /** Fastest any node moved in the whole run, m/s. */
  fastest: number;
  /** Fastest body node (tank, pod, engine) and fastest leg node in the rest window, m/s. */
  bodyRest: number;
  legRest: number;
  tilt: number;
  legs: boolean[];
  final: Float64Array;
}

/** Runs a world from `flight.setup` for `steps` steps with `commands` per step, and reads the landing off it. */
function watch(core: Core, flight: Flight, steps: number, commands?: (batch: CommandBatch, step: number) => void): Landing {
  const craft = flight.loaded.craft;
  const world = new World(core);
  world.apply(flight.setup);
  const body = new Set([...partNodes(craft, "tank"), ...partNodes(craft, "pod"), ...partNodes(craft, "engine")]);
  const out: Landing = { touchStep: -1, steps, yields: 0, broken: new Set(), freezes: 0, deepest: -Infinity, fastest: 0, bodyRest: 0, legRest: 0, tilt: 0, legs: [], final: new Float64Array(0) };
  let prev = world.positions().slice();
  const restFrom = steps - 120;
  for (let s = 0; s < steps; s++) {
    const batch = new CommandBatch();
    commands?.(batch, s);
    world.apply(batch);
    world.step();
    for (const e of world.events()) {
      if (e.kind === EVENT_YIELD) out.yields++;
      else if (e.kind === EVENT_BREAK) out.broken.add(e.index);
      else if (e.kind === EVENT_FREEZE) out.freezes++;
    }
    const p = world.positions();
    let low = Infinity;
    for (const [i, n] of craft.nodes.entries()) {
      const d = heightAt(flight.field, p[3 * i]!, p[3 * i + 2]!) - (p[3 * i + 1]! - n.radius);
      if (d > out.deepest) out.deepest = d;
      const c = -d;
      if (c < low) low = c;
      const v = Math.hypot(p[3 * i]! - prev[3 * i]!, p[3 * i + 1]! - prev[3 * i + 1]!, p[3 * i + 2]! - prev[3 * i + 2]!) * 60;
      if (v > out.fastest) out.fastest = v;
      if (s >= restFrom) {
        if (body.has(i)) out.bodyRest = Math.max(out.bodyRest, v);
        else out.legRest = Math.max(out.legRest, v);
      }
    }
    if (out.touchStep < 0 && s > 20 && low < 0.002) out.touchStep = s;
    prev = p.slice();
  }
  out.final = world.positions().slice();
  out.tilt = tiltDegrees(craft, out.final);
  out.legs = joinedLegs(craft, out.broken);
  world.free();
  return out;
}

/** The checks every run must pass, whatever the speed. */
function commonBands(label: string, r: Landing, unbrokenAtRest: boolean): void {
  const radius = 0.05; // the largest node radius is below this
  assert.ok(r.deepest < radius + 0.01, `${label}: a node went ${r.deepest.toFixed(3)} m below the ground`);
  assert.ok(r.fastest < 100, `${label}: a node reached ${r.fastest.toFixed(1)} m/s (clamped nodes would sit at 1000)`);
  assert.equal(r.freezes, 0, `${label}: an island froze`);
  assert.ok(r.touchStep > 0 && r.touchStep + 900 <= r.steps - 120, `${label}: touchdown at step ${r.touchStep}, so the rest window (the last 2 s) starts under 15 s after it`);
  assert.ok(r.bodyRest < REST_SPEED, `${label}: the body still moves at ${r.bodyRest.toFixed(2)} m/s at the end`);
  if (unbrokenAtRest) assert.ok(r.legRest < REST_SPEED, `${label}: a leg still moves at ${r.legRest.toFixed(2)} m/s at the end`);
}

test("the hopper stands 20 s on the pad without a single yield", async () => {
  const core = await loadCore();
  const r = watch(core, prepareDrop(HOPPER, PAD, 0), STEPS);
  assert.equal(r.yields, 0);
  assert.equal(r.broken.size, 0);
  assert.ok(r.tilt < 15, `tilt ${r.tilt}`);
  assert.ok(r.bodyRest < 0.01 && r.legRest < 0.01, `still moving: ${r.bodyRest}, ${r.legRest}`);
  assert.ok(r.deepest < 0.06);
});

test("a 2 m/s touchdown on the pad yields nothing", async () => {
  const core = await loadCore();
  const r = watch(core, prepareDrop(HOPPER, PAD, 2), STEPS);
  assert.equal(r.yields, 0);
  assert.equal(r.broken.size, 0);
  assert.ok(r.tilt < 15, `tilt ${r.tilt}`);
  commonBands("2 m/s pad", r, true);
});

for (const spot of [PAD, ...ROUGH_SPOTS] as Spot[]) {
  test(`a 3 m/s touchdown at ${spot.name} breaks nothing and the craft stands`, async () => {
    const core = await loadCore();
    const r = watch(core, prepareDrop(HOPPER, spot, 3), STEPS);
    assert.equal(r.broken.size, 0, `${r.broken.size} beams broke`);
    assert.deepEqual(r.legs, [true, true, true, true]);
    assert.ok(r.tilt < 15, `tilt ${r.tilt.toFixed(1)}°`);
    commonBands(`3 m/s ${spot.name}`, r, true);
  });
}

test("a 7 m/s touchdown on the rough spots snaps a leg and tips the craft over, on at least four of five", async (t) => {
  const core = await loadCore();
  let snapped = 0;
  for (const spot of ROUGH_SPOTS) {
    const r = watch(core, prepareDrop(HOPPER, spot, 7), STEPS);
    const lost = r.legs.filter((j) => !j).length;
    const tipped = r.tilt > 60;
    t.diagnostic(`${spot.name}: legs ${r.legs.map((j) => (j ? 1 : 0)).join("")} tilt ${r.tilt.toFixed(0)}° broken ${r.broken.size} yields ${r.yields}`);
    if (lost > 0 && tipped) snapped++;
    commonBands(`7 m/s ${spot.name}`, r, false);
  }
  assert.ok(snapped >= 4, `a leg snapped and the craft tipped on ${snapped} of 5 spots`);
});

test("a 7 m/s touchdown on the flat pad: recorded, not required", async (t) => {
  const core = await loadCore();
  const r = watch(core, prepareDrop(HOPPER, PAD, 7), STEPS);
  t.diagnostic(`pad at 7 m/s: legs ${r.legs.map((j) => (j ? 1 : 0)).join("")} tilt ${r.tilt.toFixed(0)}° broken ${r.broken.size} yields ${r.yields}`);
  commonBands("7 m/s pad", r, false);
});

/** Records the scripted hop for `HOP_STEPS` steps, and what happened to the craft. */
function recordHop(core: Core): { bytes: Uint8Array; hash: bigint; landing: Landing } {
  const hop = prepareHop(HOPPER);
  const landing = watch(core, hop.flight, HOP_STEPS, (b, s) => hopCommands(b, hop, s));
  const world = new World(core);
  const recorder = new Recorder(world);
  assert.equal(recorder.setup(hop.flight.setup).rejected, 0);
  for (let s = 0; s < HOP_STEPS; s++) {
    const cmds = new CommandBatch();
    hopCommands(cmds, hop, s);
    recorder.step(cmds);
  }
  const rec = recorder.finish();
  const hash = world.stateHash;
  world.free();
  return { bytes: encodeRecording(rec), hash, landing };
}

test("the scripted hop lands hard in the rough, snaps a leg, and tips the craft over", async () => {
  const core = await loadCore();
  const { landing: r } = recordHop(core);
  assert.ok(r.legs.some((j) => !j), `no leg snapped (legs ${r.legs.join(",")})`);
  assert.ok(r.tilt > 60, `tilt ${r.tilt.toFixed(0)}°`);
  commonBands("hop", r, false);
  const tank = centreOf(r.final, partNodes(prepareHop(HOPPER).flight.loaded.craft, "tank"));
  assert.ok(tank[0] > 7.5, `it came down at x ${tank[0].toFixed(1)}, short of the rough area`);
});

test("the committed hard landing replays to its hash, and recording it again gives the same file", async () => {
  const core = await loadCore();
  const { bytes: fresh, hash } = recordHop(core);
  if (process.env.RIBCAGE_UPDATE_FIXTURE) writeFileSync(FIXTURE, fresh);
  assert.ok(existsSync(FIXTURE), "fixture missing: run once with RIBCAGE_UPDATE_FIXTURE=1");
  const stored = new Uint8Array(readFileSync(FIXTURE));
  assert.ok(stored.byteLength < 150 * 1024, `the fixture is ${stored.byteLength} bytes, over the 150 KB budget`);
  const result = replayRecording(() => new World(core), decodeRecording(stored));
  assert.equal(result.ok, true);
  assert.equal(result.steps, HOP_STEPS);
  assert.equal(result.hash, hash);
  assert.deepEqual(fresh, stored);
});
