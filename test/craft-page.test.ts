import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";
import { CommandBatch } from "../src/commands.ts";
import { Core } from "../src/core.ts";
import {
  craftCentre,
  followStep,
  GALLERY_PARTS,
  galleryCraft,
  readout,
  scenarioFor,
  SLOW_FACTOR,
  SLOW_HOLD_STEPS,
  slowFactor,
  throttleCommands,
  throttleKey,
} from "../src/craft-page.ts";
import { craftPanels } from "../src/craft-panels.ts";
import { flightCommands, prepareFlight, type ThrottleScript } from "../src/flight.ts";
import { clearance, HOP_STEPS } from "../src/landing.ts";
import { loadCraft } from "../src/parts/craft-file.ts";
import { decodeRecording, Recorder, replayRecording } from "../src/recording.ts";
import { World } from "../src/world.ts";

const WASM_PATH = new URL("../target/wasm32-unknown-unknown/release/beam_core.wasm", import.meta.url);
const HOPPER_TEXT = readFileSync(new URL("./fixtures/hopper.craft.json", import.meta.url), "utf8");
/** The page plays the hop rigid (the default from plan 005, phase 5.4). */
const FIXTURE = new URL("./fixtures/hard-landing-rigid.rcrd", import.meta.url);

async function loadCore(): Promise<Core> {
  return Core.load(new Uint8Array(readFileSync(WASM_PATH)));
}

test("the throttle keys step by tenths, and full and cut jump", () => {
  assert.equal(throttleKey(0, "ArrowUp"), 0.1);
  assert.equal(throttleKey(0.3, "w"), 0.4);
  assert.equal(throttleKey(0.7, "ArrowDown"), 0.6);
  assert.equal(throttleKey(0.3, "s"), 0.2);
  assert.equal(throttleKey(0.2, "z"), 1);
  assert.equal(throttleKey(0.8, "x"), 0);
  // Clamped, and no drift from adding tenths in floating point.
  assert.equal(throttleKey(1, "ArrowUp"), 1);
  assert.equal(throttleKey(0, "ArrowDown"), 0);
  let t = 0;
  for (let i = 0; i < 10; i++) t = throttleKey(t, "ArrowUp")!;
  assert.equal(t, 1);
  for (let i = 0; i < 3; i++) t = throttleKey(t, "ArrowDown")!;
  assert.equal(t, 0.7);
  for (const other of ["a", " ", "Enter", "ArrowLeft"]) assert.equal(throttleKey(0.5, other), null);
});

test("a throttle command is written only when the throttle changes", () => {
  const b = new CommandBatch();
  assert.equal(throttleCommands(b, 2, 0, 0), 0);
  assert.equal(b.count, 0);
  assert.equal(throttleCommands(b, 2, 0.5, 0), 0.5);
  assert.equal(b.count, 2, "one command per thruster");
  assert.equal(throttleCommands(b, 2, 0.5, 0.5), 0.5);
  assert.equal(b.count, 2, "nothing more for a held throttle");
});

test("the follow camera closes half the gap each half-life, at any frame rate", () => {
  const start: [number, number, number] = [0, 0, 0];
  const goal: [number, number, number] = [8, -4, 2];
  const one = followStep(start, goal, 1, 1);
  assert.deepEqual(one.map((v, i) => v / goal[i]!), [0.5, 0.5, 0.5]);
  // Two half-second frames close the same gap as one second.
  const half = followStep(followStep(start, goal, 0.5, 1), goal, 0.5, 1);
  for (let i = 0; i < 3; i++) assert.ok(Math.abs(half[i]! - one[i]!) < 1e-12);
  // Sixty frames of 1/60 s too.
  let p = start;
  for (let i = 0; i < 60; i++) p = followStep(p, goal, 1 / 60, 1);
  for (let i = 0; i < 3; i++) assert.ok(Math.abs(p[i]! - one[i]!) < 1e-9);
  // A craft that stands still is followed to rest, and a zero half-life snaps.
  assert.deepEqual(followStep(goal, goal, 0.016, 1), goal);
  assert.deepEqual(followStep(start, goal, 0.016, 0), goal);
});

test("slow motion: on the way down near the ground, and a short while after the first touch", () => {
  assert.equal(slowFactor(20, 9, -1), 1, "high up");
  assert.equal(slowFactor(3, 1, -1), 1, "low but not falling fast");
  assert.equal(slowFactor(3, 6, -1), SLOW_FACTOR, "low and falling");
  assert.equal(slowFactor(0, 6, 0), SLOW_FACTOR, "at the touch");
  assert.equal(slowFactor(0, 0, SLOW_HOLD_STEPS - 1), SLOW_FACTOR, "still inside the hold");
  assert.equal(slowFactor(0, 0, SLOW_HOLD_STEPS), 1, "hold over");
  assert.equal(slowFactor(3, 6, SLOW_HOLD_STEPS + 5), 1, "after the hold it stays at full pace, even bouncing");
});

test("every gallery part loads on its own and rests just touching the pad", () => {
  for (const spec of GALLERY_PARTS) {
    const text = galleryCraft(spec.type);
    const sc = scenarioFor("part", text, { part: spec.type });
    const craft = sc.flight.loaded.craft;
    const pos = new Float64Array(3 * craft.nodes.length);
    for (const [i, n] of craft.nodes.entries()) pos.set([n.x, n.y, n.z], 3 * i);
    const c = clearance(craft, pos, sc.flight.field);
    assert.ok(Math.abs(c) < 1e-9, `${spec.type}: lowest point ${c} m from the pad`);
  }
  assert.throws(() => galleryCraft("wing"), /no gallery part/);
});

test("the skin of a craft has one slab per cell, bound to a tetrahedron that is not flat", () => {
  const craft = loadCraft(HOPPER_TEXT).craft;
  const set = craftPanels(craft);
  assert.ok(craft.panels.length > 0);
  assert.equal(set.cells.length, craft.panels.length);
  assert.equal(set.panelStart.length, craft.panels.length + 1);
  assert.equal(set.cellOf.length * 3, set.positions.length);
  assert.equal(set.cellOf.length * 3, set.normals.length);
  for (let i = 0; i < set.panelStart.length - 1; i++) assert.ok(set.panelStart[i + 1]! > set.panelStart[i]!, `cell ${i} has no panel`);
  for (const v of set.positions) assert.ok(Number.isFinite(v));
  for (const v of set.normals) assert.ok(Number.isFinite(v));
  // Every vertex's binding weights add to one and name a node of the craft.
  const w = set.binding.weights;
  for (let v = 0; v < set.cellOf.length; v++) {
    assert.ok(Math.abs(w[4 * v]! + w[4 * v + 1]! + w[4 * v + 2]! + w[4 * v + 3]! - 1) < 1e-9, `vertex ${v}: weights do not add to one`);
    for (let k = 0; k < 4; k++) assert.ok(set.binding.nodes[4 * v + k]! < craft.nodes.length);
  }
  // A slab stands outside the craft: no vertex nearer the axis than the wall it stands on.
  const tank = craft.tree.find((t) => t.id === "tank")!;
  const ring = [...new Set(tank.nodeOf)].map((i) => Math.hypot(craft.nodes[i]!.x, craft.nodes[i]!.z));
  const wall = Math.max(...ring);
  let outside = 0;
  for (let v = 0; v < set.cellOf.length; v++) if (Math.hypot(set.positions[3 * v]!, set.positions[3 * v + 2]!) > wall + 0.04) outside++;
  assert.ok(outside > 0, "no slab stands out past the tank wall");
});

test("every gallery part has a skin, except the ones that have no panels", () => {
  for (const spec of GALLERY_PARTS) {
    const craft = loadCraft(galleryCraft(spec.type)).craft;
    const set = craftPanels(craft);
    assert.equal(set.cells.length, craft.panels.length, spec.type);
  }
});

test("keys pressed on the page record to the same bytes and hash as the same throttle as a script", async () => {
  const core = await loadCore();
  const STEPS = 150;
  // The player presses full at step 0, and down five times at step 60 (to a half).
  const pressed = new Map<number, string[]>([
    [0, ["z"]],
    [60, ["ArrowDown", "ArrowDown", "ArrowDown", "ArrowDown", "ArrowDown"]],
  ]);
  const flight = prepareFlight(HOPPER_TEXT);

  const world = new World(core);
  const rec = new Recorder(world);
  assert.equal(rec.setup(flight.setup).rejected, 0);
  let wanted = 0;
  let sent = 0;
  for (let s = 0; s < STEPS; s++) {
    for (const key of pressed.get(s) ?? []) wanted = throttleKey(wanted, key) ?? wanted;
    const batch = new CommandBatch();
    sent = throttleCommands(batch, flight.thrusterCount, wanted, sent);
    rec.step(batch);
  }
  const keyed = rec.finish();
  const keyedHash = world.stateHash;
  world.free();

  const script: ThrottleScript = [
    { step: 0, throttle: 1 },
    { step: 60, throttle: 0.5 },
  ];
  const world2 = new World(core);
  const rec2 = new Recorder(world2);
  rec2.setup(prepareFlight(HOPPER_TEXT).setup);
  for (let s = 0; s < STEPS; s++) {
    const batch = new CommandBatch();
    flightCommands(batch, flight, script, s);
    rec2.step(batch);
  }
  const scripted = rec2.finish();
  assert.equal(keyedHash, world2.stateHash);
  world2.free();
  assert.deepEqual(keyed.lists, scripted.lists, "the command lists differ");

  // The keyed flight replays to its own hash.
  const replay = replayRecording(() => new World(core), keyed);
  assert.equal(replay.ok, true);
  assert.equal(replay.hash, keyedHash);
});

test("the page's hop scenario ends on the hash of the committed rigid hard landing", async () => {
  const core = await loadCore();
  const sc = scenarioFor("hop", HOPPER_TEXT);
  assert.ok(sc.script);
  const world = new World(core);
  assert.equal(world.apply(sc.flight.setup).rejected, 0);
  for (let s = 0; s < HOP_STEPS; s++) {
    const batch = new CommandBatch();
    sc.script!(batch, s);
    if (batch.count > 0) world.apply(batch);
    world.step();
  }
  const stored = decodeRecording(new Uint8Array(readFileSync(FIXTURE)));
  assert.equal(world.stateHash, stored.finalHash);
  world.free();
});

test("the readout of a craft at rest above the pad", async () => {
  const core = await loadCore();
  const sc = scenarioFor("drop", HOPPER_TEXT, { speed: 5 });
  const world = new World(core);
  world.apply(sc.flight.setup);
  const p0 = world.positions();
  world.step();
  const p1 = world.positions();
  const r = readout(sc.flight.loaded.craft, sc.flight.field, p1, p0, 1 / 60, new Set());
  assert.ok(Math.abs(r.altitude - 5 * 5 / (2 * 9.81)) < 0.05, `altitude ${r.altitude}`);
  assert.ok(r.tilt !== null && r.tilt < 1, `tilt ${r.tilt}`);
  assert.equal(r.lostLegs, 0);
  assert.equal(r.legCount, 4);
  assert.ok(r.climb < 0 && r.speed > 0, "it has started to fall");
  const c = craftCentre(sc.flight.loaded.craft, p1);
  assert.ok(c[1] > 0);
  world.free();
  // A craft that is not the hopper has altitude and speed but no leg or tilt readout.
  const lone = scenarioFor("part", galleryCraft("tank"), { part: "tank" }).flight.loaded.craft;
  const pos = new Float64Array(3 * lone.nodes.length);
  const out = readout(lone, sc.flight.field, pos, pos, 1 / 60, new Set());
  assert.equal(out.tilt, null);
  assert.equal(out.lostLegs, null);
});
