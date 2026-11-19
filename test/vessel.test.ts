/**
 * Plan 005, phase 5.10: splitting into vessels on the hopper. A leg cut off in free fall (live,
 * then asleep) becomes vessel 1 with the masses adding to 4600 kg (to rounding) and the momentum
 * changing only by gravity; asleep, the one body is divided without adding energy. The scripted
 * hard landing reports the leg it tears off as its own vessel, sleeping on and off, matching the
 * beam walk of `src/landing.ts`. The headless cases are `core/tests/vessel.rs`; the native and
 * WebAssembly golden is the cut scene in `test/wasm.test.ts`. Measured numbers are in ER 005, phase 5.10.
 */
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";
import { CommandBatch } from "../src/commands.ts";
import { Core, EVENT_BREAK, EVENT_SPLIT, EVENT_VESSEL_SEGMENT } from "../src/core.ts";
import { HOP_STEPS, hopCommands, joinedLegs, PAD, prepareDrop, prepareHop } from "../src/landing.ts";
import { World } from "../src/world.ts";

const WASM_PATH = new URL("../target/wasm32-unknown-unknown/release/beam_core.wasm", import.meta.url);
const HOPPER = readFileSync(new URL("./fixtures/hopper.craft.json", import.meta.url), "utf8");
const loadCore = async (): Promise<Core> => Core.load(new Uint8Array(readFileSync(WASM_PATH)));
/** The hopper's legs are segments 6 to 9 (tank 0 to 3, pod 4, engine 5). */
const LEG0 = 6;
const MASS = 4600;
const G = 9.81;
const DT = 1 / 60;
/** A drop from high enough that the craft falls for about two seconds: long enough to sleep in the air. */
const DROP_SPEED = 20;
/**
 * The vessels' masses are sums of fractional node masses, so their total can be a few ulp off the
 * craft's 4600 kg (measured 1 to 3 ulp, ER 005, phase 5.10); exact would need stored masses.
 */
const MASS_ROUNDING = 1e-12 * MASS;
const massClose = (m: number) => Math.abs(m - MASS) <= MASS_ROUNDING;

const sub = (a: readonly number[], b: readonly number[]) => a.map((x, i) => x - b[i]!);
const add = (a: readonly number[], b: readonly number[]) => a.map((x, i) => x + b[i]!);
const norm = (v: readonly number[]) => Math.hypot(...v);

/** The beams with exactly one end in segment `s`: its seam to the rest of the craft. */
function seamBeams(world: World, craft: { beams: { a: number; b: number }[] }, s: number): number[] {
  const seg = world.segments().nodeSegment;
  return craft.beams.flatMap((b, i) => ((seg[b.a] === s) !== (seg[b.b] === s) ? [i] : []));
}

interface Cut {
  ids: number;
  segmentVessel: number[];
  masses: [number, number];
  /** Momentum change over the cut step, minus gravity's impulse. */
  dp: number[];
  splits: [number, number][];
  listed: [number, number][];
  checks: ReturnType<World["vessels"]>["checks"];
  bodies: { leg: number; rest: number };
  asleepBefore: boolean;
}

/** Drops the hopper, cuts leg 0's seam before step `at`, and reads the step that splits it. */
async function cutLegInFlight(sleeping: boolean, at: number): Promise<Cut> {
  const flight = prepareDrop(HOPPER, PAD, DROP_SPEED, undefined, { sleeping });
  const world = new World(await loadCore());
  assert.equal(world.apply(flight.setup).rejected, 0);
  for (let s = 0; s < at; s++) world.step();
  const before = world.vesselProps(0);
  assert.equal(world.vessels().ids, 1, "one vessel before the cut");
  // The pod and engine are rigid from spawn and never report asleep (segments 4 and 5).
  const asleepBefore = [...world.sleep().asleep].every((a, s) => a === 1 || s === 4 || s === 5);
  const cut = new CommandBatch();
  for (const b of seamBeams(world, flight.loaded.craft, LEG0)) cut.breakBeam(b);
  assert.ok(cut.count > 0);
  assert.equal(world.apply(cut).rejected, 0);
  world.step();
  const v = world.vessels();
  const p0 = world.vesselProps(0);
  const p1 = world.vesselProps(1);
  const splits: [number, number][] = [];
  const listed: [number, number][] = [];
  for (const e of world.events()) {
    if (e.kind === EVENT_SPLIT) splits.push([e.index, e.value]);
    if (e.kind === EVENT_VESSEL_SEGMENT) listed.push([e.index, e.value]);
  }
  const b = world.bodies();
  const out: Cut = {
    ids: v.ids,
    segmentVessel: [...v.segmentVessel],
    masses: [p0.mass, p1.mass],
    dp: sub(add(p0.momentum, p1.momentum), add(before.momentum, [0, -MASS * G * DT, 0])),
    splits,
    listed,
    checks: v.checks,
    bodies: { leg: b.segmentBody[LEG0]!, rest: b.segmentBody[0]! },
    asleepBefore,
  };
  world.free();
  return out;
}

function checkSplit(label: string, c: Cut): void {
  assert.equal(c.ids, 2, `${label}: two vessels`);
  assert.deepEqual(c.segmentVessel, [0, 0, 0, 0, 0, 0, 1, 0, 0, 0], `${label}: leg 0 alone in vessel 1`);
  assert.ok(massClose(c.masses[0] + c.masses[1]), `${label}: masses ${c.masses.join(" + ")}`);
  assert.deepEqual(c.splits, [[1, 0]], `${label}: one split record`);
  assert.deepEqual(c.listed, [[LEG0, 1]], `${label}: the leg's segment listed`);
  // Momentum: the cut adds nothing; the step's change is gravity alone (no contact in the air).
  assert.ok(norm(c.dp) < 1e-9 * MASS * 30, `${label}: momentum off by ${norm(c.dp).toExponential(2)} N·s`);
}

test("a hopper leg cut off in the air, live: two vessels, 4600 kg, momentum kept", async (t) => {
  const c = await cutLegInFlight(false, 60);
  t.diagnostic(`live: masses ${c.masses.map((m) => m.toFixed(3)).join(" + ")} = 4600 ${(c.masses[0] + c.masses[1] - MASS).toExponential(1)}; momentum off by ${norm(c.dp).toExponential(2)} N·s`);
  checkSplit("live", c);
  assert.equal(c.checks.length, 0, "no body to divide");
});

test("the same cut asleep: the body is divided, momentum and spin kept, no energy added", async (t) => {
  const c = await cutLegInFlight(true, 100);
  assert.ok(c.asleepBefore, "the whole craft asleep in the air before the cut");
  checkSplit("asleep", c);
  assert.equal(c.checks.length, 1, "one division record");
  const k = c.checks[0]!;
  const dp = norm(sub(k.momentumAfter, k.momentumBefore));
  const dl = norm(sub(k.spinAfter, k.spinBefore));
  const de = k.energyAfter - k.energyBefore;
  t.diagnostic(`asleep: division |dp| ${dp.toExponential(2)} N·s, |dL| ${dl.toExponential(2)} N·m·s, dE ${de.toExponential(2)} J; leg body ${c.bodies.leg}, rest ${c.bodies.rest}`);
  const p = MASS * 30;
  assert.ok(dp <= 1e-12 * p, `momentum ${dp}`);
  assert.ok(dl <= 1e-12 * p * 100, `spin ${dl}`);
  assert.ok(de <= 1e-12 * 0.5 * MASS * 900, `energy up by ${de} J`);
  assert.notEqual(c.bodies.leg, c.bodies.rest, "the leg no longer rides the craft's body");
});

for (const sleeping of [true, false]) {
  test(`the hard landing reports the leg it tears off as its own vessel (sleeping ${sleeping ? "on" : "off"})`, async (t) => {
    const hop = prepareHop(HOPPER, { sleeping });
    const world = new World(await loadCore());
    assert.equal(world.apply(hop.flight.setup).rejected, 0);
    const broken = new Set<number>();
    const splits: string[] = [];
    for (let s = 0; s < HOP_STEPS; s++) {
      const b = new CommandBatch();
      hopCommands(b, hop, s);
      world.apply(b);
      world.step();
      for (const e of world.events()) {
        if (e.kind === EVENT_BREAK) broken.add(e.index);
        if (e.kind === EVENT_SPLIT) splits.push(`step ${s}: vessel ${e.index} from ${e.value}`);
      }
    }
    const v = world.vessels();
    const joined = joinedLegs(hop.flight.loaded.craft, broken);
    const masses = Array.from({ length: v.ids }, (_, k) => world.vesselProps(k).mass);
    t.diagnostic(`${splits.join("; ")}; legs joined ${joined.map((j) => (j ? 1 : 0)).join("")}; segment vessels ${[...v.segmentVessel].join(",")}; masses ${masses.map((m) => m.toFixed(1)).join(", ")} kg`);
    assert.ok(v.ids >= 2, "a split happened");
    for (let leg = 0; leg < 4; leg++) {
      const away = v.segmentVessel[LEG0 + leg] !== 0;
      assert.equal(away, !joined[leg], `leg ${leg}: in its own vessel exactly when the beam walk finds it cut off`);
    }
    for (let s = 0; s < LEG0; s++) assert.equal(v.segmentVessel[s], 0, `segment ${s} stays with the craft`);
    const total = masses.reduce((a, b) => a + b, 0);
    assert.ok(massClose(total), `the vessels' masses add to the craft's: ${total}`);
    world.free();
  });
}
