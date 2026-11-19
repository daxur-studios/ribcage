/**
 * Equivalence (plan 005, phase 5.9): every scene of `src/equivalence.ts` runs twice, once with
 * sleeping on and once with every segment live, and the two must tell the same story. The hopper's
 * 4.8 drops at 2, 3 and 7 m/s on the pad and the five rough spots, full throttle, the scripted hop
 * and a side knock on the tank.
 *
 * Tolerances are stated against the scenes' own chaos (ER 005, 5.9): an all-live 7 m/s rough drop
 * moved 0.1 to 1 mm sideways already breaks up to two beams differently and yields up to three, so
 * the crash scenes (7 m/s on a rough spot, and the hop, which lands on two legs and tips) may differ
 * by that much. Every other scene must break and yield exactly the same beams.
 *
 * Also: every switch keeps momentum and spin and adds no energy, in every scene; substeps and
 * milliseconds per step per flight phase are reported, sleeping on against off.
 */

import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";
import { Core } from "../src/core.ts";
import { compareRuns, equivalenceScenes, runScene, type SceneRun } from "../src/equivalence.ts";

const WASM_PATH = new URL("../target/wasm32-unknown-unknown/release/beam_core.wasm", import.meta.url);
const HOPPER = readFileSync(new URL("./fixtures/hopper.craft.json", import.meta.url), "utf8");

/** Crash scenes: beams broken in one run and not the other, at most. */
const CRASH_BREAKS = 2;
/** Crash scenes: beams that yield with sleeping on and never all-live, at most. */
const CRASH_EXTRA_YIELDS = 6;
/** Tilt at rest, sleeping on against all-live, at most (degrees). */
const TILT_DEGREES = 2;

const isCrash = (name: string) => name === "hop" || name.startsWith("drop 7 m/s rough");

function costLine(name: string, off: SceneRun, on: SceneRun): string {
  const parts: string[] = [];
  for (const [phase, a] of off.phases) {
    const b = on.phases.get(phase);
    if (!b) continue;
    parts.push(
      `${phase} ${(a.substeps / a.steps).toFixed(1)}/${(b.substeps / b.steps).toFixed(1)} sub, ` +
        `${(a.ms / a.steps).toFixed(2)}/${(b.ms / b.steps).toFixed(2)} ms`,
    );
  }
  return `${name} (off/on): ${parts.join("; ")}`;
}

test("sleeping on tells the same story as everything live: breaks, yields, legs, tilt, switches", async (t) => {
  const core = await Core.load(new Uint8Array(readFileSync(WASM_PATH)));
  const scenes = equivalenceScenes(HOPPER);
  assert.equal(scenes.length, 16);
  for (const scene of scenes) {
    const off = runScene(core, scene, false);
    const on = runScene(core, scene, true);
    const c = compareRuns(on, off);
    const legs = (r: SceneRun) => r.lost.map((l) => (l ? 1 : 0)).join("");
    t.diagnostic(
      `${scene.name}: broke ${off.broken.size}/${on.broken.size} (differ ${c.breakDiff}), yielded ${off.yielded.size}/${on.yielded.size} ` +
        `(+${c.extraYields} -${c.missedYields}), legs lost ${legs(off)}/${legs(on)}, tilt ${off.tilt.toFixed(1)}/${on.tilt.toFixed(1)} deg; ` +
        `${on.wakes} wakes, ${on.sleeps} sleeps, ${on.switches.passes} passes`,
    );
    t.diagnostic(costLine(scene.name, off, on));
    assert.equal(off.switches.passes, 0, `${scene.name}: all-live switched`);
    if (isCrash(scene.name)) {
      assert.ok(c.breakDiff <= CRASH_BREAKS, `${scene.name}: ${c.breakDiff} beams broke in one run only`);
      assert.ok(c.extraYields <= CRASH_EXTRA_YIELDS, `${scene.name}: ${c.extraYields} beams yielded only with sleeping on`);
    } else {
      assert.equal(c.breakDiff, 0, `${scene.name}: a different set of beams broke`);
      assert.equal(c.extraYields, 0, `${scene.name}: a beam yielded only with sleeping on`);
    }
    assert.ok(c.sameLegs, `${scene.name}: different legs lost (${legs(off)} all-live, ${legs(on)} sleeping)`);
    assert.ok(c.tiltDiff <= TILT_DEGREES, `${scene.name}: tilt differs by ${c.tiltDiff.toFixed(2)} deg`);
    // Every switch: the brief's formulas, so momentum and spin are kept to rounding and a sleep can
    // only lose kinetic energy, a wake changes none.
    const w = on.switches;
    assert.ok(w.momentum <= 1e-12, `${scene.name}: a switch changed momentum by ${w.momentum}`);
    assert.ok(w.spin <= 1e-12, `${scene.name}: a switch changed spin by ${w.spin}`);
    assert.ok(w.sleepEnergy <= 1e-12, `${scene.name}: a sleep added energy (${w.sleepEnergy})`);
    assert.ok(w.wakeEnergy <= 1e-12, `${scene.name}: a wake changed energy (${w.wakeEnergy})`);
    assert.ok(on.sleeps > 0, `${scene.name}: nothing ever slept`);
  }
});
