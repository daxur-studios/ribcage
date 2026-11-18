import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";
import { CommandBatch } from "../src/commands.ts";
import { Core, EVENT_BREAK, EVENT_FREEZE, EVENT_YIELD } from "../src/core.ts";
import { checkScript, flightCommands, prepareFlight, type Flight, type ThrottleScript } from "../src/flight.ts";
import { partMass, SUBSTEP_BUDGET } from "../src/parts/part.ts";
import { heightAt, landingField, LANDING_PAD_HALF } from "../src/terrain.ts";
import { World } from "../src/world.ts";

const WASM_PATH = new URL("../target/wasm32-unknown-unknown/release/beam_core.wasm", import.meta.url);
const HOPPER_TEXT = readFileSync(new URL("./fixtures/hopper.craft.json", import.meta.url), "utf8");

const G = 9.81;
const STEP_SECONDS = 1 / 60;

async function loadCore(): Promise<Core> {
  return Core.load(new Uint8Array(readFileSync(WASM_PATH)));
}

type V = [number, number, number];

/** The mean position of the craft nodes of one part of the hopper. */
function centroid(flight: Flight, id: string, pos: Float64Array): V {
  const entry = flight.loaded.craft.tree.find((t) => t.id === id);
  assert.ok(entry, `no part ${id}`);
  const nodes = [...new Set(entry.nodeOf)];
  const c: V = [0, 0, 0];
  for (const n of nodes) for (let k = 0; k < 3; k++) c[k]! += pos[3 * n + k]! / nodes.length;
  return c;
}

/** The angle, in degrees, between two directions. */
function angleBetween(a: V, b: V): number {
  const dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  const la = Math.hypot(...a);
  const lb = Math.hypot(...b);
  return (Math.acos(Math.max(-1, Math.min(1, dot / (la * lb)))) * 180) / Math.PI;
}

function axisOf(flight: Flight, pos: Float64Array): V {
  const top = centroid(flight, "pod", pos);
  const bottom = centroid(flight, "engine", pos);
  return [top[0] - bottom[0], top[1] - bottom[1], top[2] - bottom[2]];
}

interface Run {
  hash: bigint;
  /** Pod height (mean node y) before the first step, then after each step: `steps + 1` numbers. */
  podY: number[];
  /** The largest angle between the craft's axis and where it started, in degrees. */
  maxTilt: number;
  /** Largest mean node speed among the last 30 steps, in m/s. */
  lateSpeed: number;
  bad: number;
  maxSubsteps: number;
  nodes: number;
  beams: number;
  msPerStep: number;
  lowestNodeGap: number;
  /** After each step, the height of the lowest node above the ground under it (`steps` numbers). */
  gap: number[];
}

async function fly(script: ThrottleScript, steps: number): Promise<Run> {
  checkScript(script);
  const flight = prepareFlight(HOPPER_TEXT);
  const world = new World(await loadCore());
  assert.equal(world.apply(flight.setup).rejected, 0);
  const start = world.positions();
  const axis0 = axisOf(flight, start);
  const podY = [centroid(flight, "pod", start)[1]];
  let maxTilt = 0;
  let bad = 0;
  let maxSubsteps = 0;
  let lateSpeed = 0;
  const gap: number[] = [];
  let before = start;
  const t0 = performance.now();
  for (let s = 0; s < steps; s++) {
    const cmds = new CommandBatch();
    flightCommands(cmds, flight, script, s);
    if (cmds.count > 0) assert.equal(world.apply(cmds).rejected, 0);
    maxSubsteps = Math.max(maxSubsteps, world.step());
    for (const e of world.events()) if (e.kind === EVENT_YIELD || e.kind === EVENT_BREAK || e.kind === EVENT_FREEZE) bad++;
    const pos = world.positions();
    podY.push(centroid(flight, "pod", pos)[1]);
    maxTilt = Math.max(maxTilt, angleBetween(axis0, axisOf(flight, pos)));
    if (s >= steps - 30) {
      let sum = 0;
      for (let i = 0; i < pos.length; i += 3) sum += Math.hypot(pos[i]! - before[i]!, pos[i + 1]! - before[i + 1]!, pos[i + 2]! - before[i + 2]!);
      lateSpeed = Math.max(lateSpeed, sum / (pos.length / 3) / STEP_SECONDS);
    }
    let low = Infinity;
    for (let i = 0; i < pos.length; i += 3) low = Math.min(low, pos[i + 1]! - heightAt(flight.field, pos[i]!, pos[i + 2]!));
    gap.push(low);
    before = pos;
  }
  const msPerStep = (performance.now() - t0) / steps;
  const end = world.positions();
  let lowestNodeGap = Infinity;
  for (let i = 0; i < end.length; i += 3) lowestNodeGap = Math.min(lowestNodeGap, end[i + 1]! - heightAt(flight.field, end[i]!, end[i + 2]!));
  const run = { hash: world.stateHash, podY, maxTilt, lateSpeed, bad, maxSubsteps, nodes: world.nodeCount, beams: world.beamCount, msPerStep, lowestNodeGap, gap };
  world.free();
  return run;
}

const FULL_FROM_60: ThrottleScript = [{ step: 60, throttle: 1 }];

test("the landing field has a dead-flat pad, a rough area beside it, and is the same every time", () => {
  const f = landingField();
  assert.equal(f.nx, 65);
  assert.equal(f.cell, 0.5);
  for (let x = -LANDING_PAD_HALF; x <= LANDING_PAD_HALF; x += 0.5) {
    for (let z = -LANDING_PAD_HALF; z <= LANDING_PAD_HALF; z += 0.5) assert.equal(heightAt(f, x, z), 0, `pad at ${x}, ${z}`);
  }
  let max = 0;
  for (const h of f.heights) max = Math.max(max, h);
  assert.ok(max > 0.1 && max <= 0.5, `the bumps reach ${max} m`);
  assert.deepEqual([...landingField().heights], [...f.heights]);
});

test("a throttle script is checked, and a step with no key adds no commands", () => {
  assert.throws(() => checkScript([{ step: 1.5, throttle: 0 }]), /whole number/);
  assert.throws(() => checkScript([{ step: -1, throttle: 0 }]), /whole number/);
  assert.throws(() => checkScript([{ step: 5, throttle: 1 }, { step: 5, throttle: 0 }]), /not after step 5/);
  assert.throws(() => checkScript([{ step: 5, throttle: 1.01 }]), /from 0 to 1/);
  assert.throws(() => checkScript([{ step: 5, throttle: Number.NaN }]), /from 0 to 1/);
  checkScript([]);
  checkScript(FULL_FROM_60);

  const flight = prepareFlight(HOPPER_TEXT);
  assert.equal(flight.thrusterCount, 1);
  const b = new CommandBatch();
  flightCommands(b, flight, FULL_FROM_60, 59);
  flightCommands(b, flight, FULL_FROM_60, 61);
  assert.equal(b.count, 0);
  flightCommands(b, flight, FULL_FROM_60, 60);
  assert.equal(b.count, 1);
  const again = new CommandBatch();
  flightCommands(again, prepareFlight(HOPPER_TEXT), FULL_FROM_60, 60);
  assert.deepEqual(again.bytes, b.bytes);
});

test("the setup sends the thruster to the core, and the core accepts every record", async () => {
  const flight = prepareFlight(HOPPER_TEXT);
  const world = new World(await loadCore());
  const r = world.apply(flight.setup);
  assert.equal(r.rejected, 0);
  // The last record is the thruster: result 0, the first thruster.
  assert.equal(r.results[r.results.length - 1], 0);
  assert.equal(world.nodeCount, flight.loaded.craft.nodes.length);
  world.free();
});

test("the hopper rests on the pad with the engine off: no yield, no break, standing still", async () => {
  const run = await fly([], 600);
  assert.equal(run.bad, 0);
  assert.ok(run.lateSpeed < 1e-3, `still moving at ${run.lateSpeed} m/s`);
  assert.ok(run.lowestNodeGap > -0.01 && run.lowestNodeGap < 0.1, `feet are ${run.lowestNodeGap} m from the ground`);
  assert.ok(run.maxTilt < 1, `tilted ${run.maxTilt} degrees`);
});

test("at full throttle the pod accelerates at thrust over mass minus gravity, level, with nothing yielding", async (t) => {
  const flight = prepareFlight(HOPPER_TEXT);
  const mass = partMass(flight.loaded.craft);
  const thrust = flight.loaded.craft.thrusters[0]!.maxThrust;
  const accel = thrust / mass - G;
  assert.ok(accel > 0, "the engine can lift the craft");
  const run = await fly(FULL_FROM_60, 60 + 180);
  // The plan's literal check: the height gained in 3 s. The legs are sprung, so the first moments
  // also release the spring's compression (the craft is not rigid), and the gain runs a little over.
  const gain = run.podY[240]! - run.podY[60]!;
  const expectedGain = 0.5 * accel * 3 * 3;
  // The same law, with the spring transient gone: the second difference of the pod height over
  // three samples half a second apart, from 1.5 s after the throttle opens.
  const dt = 30 * STEP_SECONDS;
  const measured = (run.podY[210]! - 2 * run.podY[180]! + run.podY[150]!) / (dt * dt);
  t.diagnostic(
    `gain ${gain.toFixed(3)} m vs ${expectedGain.toFixed(3)} m (${((gain / expectedGain - 1) * 100).toFixed(2)}%); acceleration ${measured.toFixed(3)} vs ${accel.toFixed(3)} m/s2 (${((measured / accel - 1) * 100).toFixed(2)}%); tilt ${run.maxTilt.toFixed(3)} deg`,
  );
  assert.ok(Math.abs(measured - accel) <= 0.05 * accel, `acceleration ${measured} m/s2, expected ${accel}`);
  assert.ok(Math.abs(gain - expectedGain) <= 0.1 * expectedGain, `gain ${gain} m, expected ${expectedGain} m`);
  assert.ok(run.maxTilt < 5, `tilted ${run.maxTilt} degrees`);
  assert.equal(run.bad, 0, "no yield, break or freeze");
  assert.ok(run.lowestNodeGap > 20, `lowest node ${run.lowestNodeGap} m up`);
});

/** Throttle up to `top` in twelve equal keys, ten steps apart, from step 60: a pilot easing on the engine. */
function ramp(top: number): ThrottleScript {
  return Array.from({ length: 12 }, (_, k) => ({ step: 60 + 10 * k, throttle: (top * (k + 1)) / 12 }));
}

test("just under hover thrust, eased on, the hopper stays on the pad; switched on at once it hops but does not break", async (t) => {
  const flight = prepareFlight(HOPPER_TEXT);
  const mass = partMass(flight.loaded.craft);
  const thrust = flight.loaded.craft.thrusters[0]!.maxThrust;
  const throttle = 0.55;
  assert.ok(thrust * throttle < mass * G, "this throttle is under hover");
  assert.ok(thrust * throttle > 0.9 * mass * G, "and only just under it");

  // Eased on over 1.2 s: the legs unload slowly, the feet never leave the ground.
  const eased = await fly(ramp(throttle), 60 + 540);
  const easedUp = Math.max(...eased.gap.slice(60));
  const easedRise = Math.max(...eased.podY.slice(60)) - eased.podY[60]!;
  // Switched on in one step: the legs spring open under the sudden unloading and the whole craft
  // hops, the thrust itself never exceeding the weight (a finding, ER 004).
  const sudden = await fly([{ step: 60, throttle }], 60 + 540);
  const suddenUp = Math.max(...sudden.gap.slice(60));
  t.diagnostic(
    `eased: feet at most ${easedUp.toFixed(4)} m up, pod rises ${easedRise.toFixed(3)} m; at once: feet ${suddenUp.toFixed(3)} m up, back at ${sudden.gap[sudden.gap.length - 1]!.toFixed(3)} m; tilt ${eased.maxTilt.toFixed(3)} / ${sudden.maxTilt.toFixed(3)} deg`,
  );
  assert.equal(eased.bad, 0);
  assert.ok(easedUp < 0.1, `feet were ${easedUp} m up`);
  assert.ok(eased.maxTilt < 5, `tilted ${eased.maxTilt} degrees`);
  assert.equal(sudden.bad, 0);
  assert.ok(suddenUp < 3, `the hop reached ${suddenUp} m`);
  assert.ok(sudden.maxTilt < 5, `tilted ${sudden.maxTilt} degrees`);
});

test("the same script twice gives the same hash, and a different script gives another", async () => {
  const a = await fly(FULL_FROM_60, 200);
  const b = await fly(FULL_FROM_60, 200);
  assert.equal(a.hash, b.hash);
  assert.deepEqual(a.podY, b.podY);
  const c = await fly([{ step: 60, throttle: 0.9 }], 200);
  assert.notEqual(a.hash, c.hash);
});

test("the hopper's cost, standing and lifting off, is within the substep budget", async (t) => {
  const standing = await fly([], 120);
  const lifting = await fly(FULL_FROM_60, 240);
  t.diagnostic(
    `hopper: ${standing.nodes} nodes, ${standing.beams} beams; standing ${standing.maxSubsteps} substeps, ${standing.msPerStep.toFixed(2)} ms/step; lifting ${lifting.maxSubsteps} substeps, ${lifting.msPerStep.toFixed(2)} ms/step`,
  );
  assert.ok(standing.maxSubsteps <= SUBSTEP_BUDGET);
  assert.ok(lifting.maxSubsteps <= SUBSTEP_BUDGET);
});
