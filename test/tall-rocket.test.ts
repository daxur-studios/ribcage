/**
 * The tall rocket (plan 005, phase 5.11): a pod, five tanks and an engine on a launch clamp, made
 * only from the existing part generators, run with sleeping on and with everything live.
 *
 * Tolerances are stated against the measured numbers (ER 005, 5.11): standing and under thrust
 * sleeping is better than all-live; under the knock and the settle it is slightly worse, within
 * `KNOCK_WOBBLE_RATIO` of all-live. The wake spreads to the whole craft on a knock and the rocket
 * does not go back to sleep within the 35 s run: logged as findings, not tuned away.
 */

import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";
import { Core } from "../src/core.ts";
import { prepareTallRocket, runTallRocket, STAND_STEPS, type TallRocketRun } from "../src/tall-rocket.ts";
import { NODE_MASS_RATIO_BUDGET, partBudget, partMass, SUBSTEP_BUDGET } from "../src/parts/part.ts";

const WASM_PATH = new URL("../target/wasm32-unknown-unknown/release/beam_core.wasm", import.meta.url);
const ROCKET = readFileSync(new URL("./fixtures/tall-rocket.craft.json", import.meta.url), "utf8");

/** Sleeping on, under the knock and the settle: tip excursion at most this times all-live's. */
const KNOCK_WOBBLE_RATIO = 1.5;
/** The standing rocket's largest node speed over its last 2 s, metres per second, at most. */
const STAND_REST_SPEED = 0.5;

let cache: Promise<{ off: TallRocketRun; on: TallRocketRun }> | null = null;
function runs() {
  cache ??= (async () => {
    const core = await Core.load(new Uint8Array(readFileSync(WASM_PATH)));
    return { off: runTallRocket(core, ROCKET, false), on: runTallRocket(core, ROCKET, true) };
  })();
  return cache;
}

test("the tall rocket loads and assembles; its mass is the sum of its parts; it is within the budgets", (t) => {
  const { flight } = prepareTallRocket(ROCKET);
  const craft = flight.loaded.craft;
  const b = partBudget(craft);
  t.diagnostic(`${craft.nodes.length} nodes, ${craft.beams.length} beams, ${craft.segments.length} segments, mass ${partMass(craft)} kg, ${b.substeps} substeps, node-mass ratio ${b.massRatio.toFixed(2)}`);
  assert.equal(partMass(craft), 800 + 5 * 1000 + 1200, "engine + five tanks + pod");
  assert.ok(b.substeps <= SUBSTEP_BUDGET, `${b.substeps} substeps`);
  assert.ok(b.massRatio <= NODE_MASS_RATIO_BUDGET, `mass ratio ${b.massRatio}`);
  assert.ok(flight.thrusterCount >= 1);
});

test("it stands on the clamp for 10 s with no yield and comes to rest", async () => {
  const core = await Core.load(new Uint8Array(readFileSync(WASM_PATH)));
  for (const sleeping of [false, true]) {
    const run = runTallRocket(core, ROCKET, sleeping, 0, STAND_STEPS);
    const s = run.phases.get("standing")!;
    assert.equal(s.yields, 0, `sleeping ${sleeping}: yielded`);
    assert.equal(s.breaks, 0, `sleeping ${sleeping}: broke`);
    assert.ok(run.restSpeed < STAND_REST_SPEED, `sleeping ${sleeping}: still moving at ${run.restSpeed} m/s`);
  }
});

test("the whole run breaks and yields nothing, in either mode", async () => {
  const { off, on } = await runs();
  for (const [name, r] of [["all-live", off], ["sleeping", on]] as const) {
    assert.equal(r.yielded.size, 0, `${name} yielded`);
    assert.equal(r.broken.size, 0, `${name} broke`);
    assert.equal(r.vessels, 1, `${name} split`);
  }
});

test("with sleeping on the segments merge into fewer bodies and the wobble is no worse than all-live", async (t) => {
  const { off, on } = await runs();
  for (const [phase, a] of off.phases) {
    const b = on.phases.get(phase)!;
    t.diagnostic(
      `${phase}: all-live/sleeping: ${(a.substeps / a.steps).toFixed(1)}/${(b.substeps / b.steps).toFixed(1)} sub, ` +
        `${(a.ms / a.steps).toFixed(2)}/${(b.ms / b.steps).toFixed(2)} ms, length range ${(a.lengthRange * 1e3).toFixed(2)}/${(b.lengthRange * 1e3).toFixed(2)} mm, ` +
        `tip max ${(a.tipMax * 1e3).toFixed(2)}/${(b.tipMax * 1e3).toFixed(2)} mm, tip range ${(a.tipRange * 1e3).toFixed(2)}/${(b.tipRange * 1e3).toFixed(2)} mm, ` +
        `bodies ${a.bodiesMin}-${a.bodiesMax}/${b.bodiesMin}-${b.bodiesMax}`,
    );
  }
  t.diagnostic(`bodies at most 2 from step ${on.firstStepAtMostBodies(2)}, at most 1 from ${on.firstStepAtMostBodies(1)}; ${on.wakes} wakes, ${on.sleeps} sleeps`);
  const ao = off.phases.get("standing")!;
  const bo = on.phases.get("standing")!;
  assert.ok(Math.min(...off.bodies) >= 2, "all-live has the pod and the engine as bodies");
  assert.ok(Math.min(...on.bodies) < Math.min(...off.bodies), "sleeping merged bodies");
  assert.ok(bo.bodiesMin < ao.bodiesMin);
  assert.ok(on.firstStepAtMostBodies(1) >= 0, "the whole stack became one body at some point");
  for (const phase of ["standing", "thrust"] as const) {
    const a = off.phases.get(phase)!;
    const b = on.phases.get(phase)!;
    assert.ok(b.substeps < a.substeps, `${phase}: fewer substeps`);
    assert.ok(b.tipMax <= a.tipMax, `${phase}: tip off axis ${b.tipMax} against ${a.tipMax}`);
    assert.ok(b.tipRange <= a.tipRange, `${phase}: tip range`);
    assert.ok(b.lengthRange <= a.lengthRange + 1e-9, `${phase}: stack length range`);
  }
  for (const phase of ["knock", "settle"] as const) {
    const a = off.phases.get(phase)!;
    const b = on.phases.get(phase)!;
    assert.ok(b.tipMax <= a.tipMax * KNOCK_WOBBLE_RATIO, `${phase}: tip ${b.tipMax} against ${a.tipMax}`);
    assert.ok(b.tipRange <= a.tipRange * KNOCK_WOBBLE_RATIO, `${phase}: tip range ${b.tipRange} against ${a.tipRange}`);
  }
});

test("every sleep and wake switch keeps momentum and spin to 1e-12 and adds no energy", async () => {
  const { off, on } = await runs();
  assert.equal(off.switches.passes, 0);
  assert.ok(on.switches.passes > 0, "sleeping on switched");
  assert.ok(on.switches.momentum <= 1e-12, `momentum ${on.switches.momentum}`);
  assert.ok(on.switches.spin <= 1e-12, `spin ${on.switches.spin}`);
  assert.ok(on.switches.sleepEnergy <= 1e-12, `sleep energy ${on.switches.sleepEnergy}`);
  assert.ok(on.switches.wakeEnergy <= 1e-12, `wake energy ${on.switches.wakeEnergy}`);
});

test("the same run twice gives the same hash", async () => {
  const core = await Core.load(new Uint8Array(readFileSync(WASM_PATH)));
  const { on, off } = await runs();
  assert.equal(runTallRocket(core, ROCKET, true).hash, on.hash);
  assert.equal(runTallRocket(core, ROCKET, false).hash, off.hash);
  assert.notEqual(on.hash, off.hash);
});
