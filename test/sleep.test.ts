/**
 * Plan 005, phase 5.8: sleep and wake on the hopper. Every switch in the standing, hop (lift-off,
 * powered flight, coast, touchdown) and drop runs keeps momentum and spin and never adds kinetic
 * energy; the hopper standing on the pad sleeps whole; a craft held near the thresholds for ten
 * simulated minutes gains no energy and switches no faster than the minimum live time allows; the
 * same run twice hashes the same. The native/WebAssembly check is the sleep scene in
 * `test/wasm.test.ts` against `core/tests/sleep.rs`. Measured numbers are in ER 005, phase 5.8.
 */
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";
import { CommandBatch } from "../src/commands.ts";
import { Core, EVENT_SLEEP, EVENT_WAKE, EVENT_YIELD, WAKE_CONTACT, type SwitchCheck } from "../src/core.ts";
import { HOP_STEPS, hopCommands, PAD, partNodes, prepareDrop, prepareHop } from "../src/landing.ts";
import { World } from "../src/world.ts";

const WASM_PATH = new URL("../target/wasm32-unknown-unknown/release/beam_core.wasm", import.meta.url);
const HOPPER = readFileSync(new URL("./fixtures/hopper.craft.json", import.meta.url), "utf8");
const loadCore = async (): Promise<Core> => Core.load(new Uint8Array(readFileSync(WASM_PATH)));
/** `sleep::MIN_LIVE_STEPS` and `sleep::QUIET_STEPS`: a wake is followed by at least this many live steps. */
const MIN_CYCLE = 60;
/** The hopper's legs are segments 6 to 9 (tank 0 to 3, pod 4, engine 5). */
const LEGS = [6, 7, 8, 9];
const TEN_MINUTES = 36000;

interface Run {
  world: World;
  checks: { step: number; c: SwitchCheck }[];
  wakes: { step: number; segment: number; reason: number }[];
  sleeps: { step: number; segment: number }[];
  firstYield: number;
  substeps: number[];
  mass: number;
}

/** Runs `steps` steps with sleeping on (unless `on` is false), collecting switches and events. */
async function run(setup: CommandBatch, steps: number, perStep: (b: CommandBatch, s: number, mass: number) => void, on = true, keep = false): Promise<Run> {
  const world = new World(await loadCore());
  assert.equal(world.apply(setup).rejected, 0);
  if (on) {
    const b = new CommandBatch();
    b.setSleeping(true);
    assert.equal(world.apply(b).rejected, 0);
  }
  const mass = [...world.segments().mass].reduce((a, b) => a + b, 0);
  const out: Run = { world, checks: [], wakes: [], sleeps: [], firstYield: -1, substeps: [], mass };
  for (let s = 0; s < steps; s++) {
    const b = new CommandBatch();
    perStep(b, s, mass);
    world.apply(b);
    out.substeps.push(world.step());
    for (const e of world.events()) {
      if (e.kind === EVENT_WAKE) out.wakes.push({ step: s, segment: e.index, reason: e.value });
      if (e.kind === EVENT_SLEEP) out.sleeps.push({ step: s, segment: e.index });
      if (e.kind === EVENT_YIELD && out.firstYield < 0) out.firstYield = s;
    }
    if (on) for (const c of world.sleep().checks) out.checks.push({ step: s, c });
  }
  if (!keep) world.free();
  return out;
}

const norm = (v: readonly number[]) => Math.hypot(...v);
const diff = (a: readonly number[], b: readonly number[]) => norm(a.map((x, i) => x - b[i]!));

/** The worst switch errors of a run, relative to its mass at 10 m/s (and 20 m from the origin). */
function worstSwitch(r: Run) {
  const p = r.mass * 10;
  const l = p * 20;
  const e = 0.5 * r.mass * 100;
  let dp = 0;
  let dl = 0;
  let sleepUp = -Infinity;
  let wakeChange = 0;
  for (const { c } of r.checks) {
    dp = Math.max(dp, diff(c.momentumAfter, c.momentumBefore) / p);
    dl = Math.max(dl, diff(c.spinAfter, c.spinBefore) / l);
    const de = (c.energyAfter - c.energyBefore) / e;
    if (c.kind === 2) sleepUp = Math.max(sleepUp, de);
    else wakeChange = Math.max(wakeChange, Math.abs(de));
  }
  return { dp, dl, sleepUp, wakeChange };
}

/** Steps from each segment's wake to its next sleep: never under the minimum cycle. */
function shortestCycle(r: Run): number {
  const woke = new Map<number, number>();
  let shortest = Infinity;
  const all = [...r.wakes.map((w) => ({ ...w, wake: true })), ...r.sleeps.map((s) => ({ ...s, wake: false, reason: 0 }))].sort((a, b) => a.step - b.step);
  for (const e of all) {
    if (e.wake) woke.set(e.segment, e.step);
    else if (woke.has(e.segment)) shortest = Math.min(shortest, e.step - woke.get(e.segment)!);
  }
  return shortest;
}

test("every switch keeps momentum and spin and adds no energy: standing, the hop and the drops", async (t) => {
  const hop = prepareHop(HOPPER);
  const runs: [string, Run][] = [
    ["standing", await run(prepareDrop(HOPPER, PAD, 0).setup, 600, () => {})],
    ["hop", await run(hop.flight.setup, HOP_STEPS, (b, s) => hopCommands(b, hop, s))],
    ["drop 3 m/s", await run(prepareDrop(HOPPER, PAD, 3).setup, 600, () => {})],
    ["drop 7 m/s", await run(prepareDrop(HOPPER, PAD, 7).setup, 600, () => {})],
  ];
  let total = 0;
  for (const [name, r] of runs) {
    const w = worstSwitch(r);
    t.diagnostic(`${name}: ${r.wakes.length} wakes, ${r.sleeps.length} sleeps, ${r.checks.length} passes; worst momentum ${w.dp.toExponential(1)}, spin ${w.dl.toExponential(1)}, energy on sleep ${w.sleepUp.toExponential(1)}, on wake ${w.wakeChange.toExponential(1)}`);
    total += r.checks.length;
    // Rounding: a few hundred additions of numbers of the scale's size.
    assert.ok(w.dp <= 1e-12, `${name}: momentum off by ${w.dp}`);
    assert.ok(w.dl <= 1e-12, `${name}: spin off by ${w.dl}`);
    assert.ok(w.sleepUp <= 1e-12, `${name}: a sleep added energy (${w.sleepUp})`);
    assert.ok(w.wakeChange <= 1e-12, `${name}: a wake changed energy (${w.wakeChange})`);
    assert.ok(shortestCycle(r) >= MIN_CYCLE, `${name}: a segment slept ${shortestCycle(r)} steps after waking`);
  }
  // The hop goes through every kind of switch: sleeps in powered flight, wakes on thrust and contact.
  const h = runs[1]![1];
  assert.ok(h.sleeps.some((s) => s.step > 24 && s.step < 220), "something sleeps in powered flight");
  assert.ok(h.wakes.some((w) => w.step === 220), "the engine cut wakes the craft");
  assert.ok(total > 20, `only ${total} switch passes`);
});

test("the hopper standing on the pad sleeps whole: one body, fewer substeps", async (t) => {
  const setup = () => prepareDrop(HOPPER, PAD, 0).setup;
  const on = await run(setup(), 600, () => {}, true, true);
  const off = await run(setup(), 600, () => {}, false);
  const sl = on.world.sleep();
  const b = on.world.bodies();
  const bodies = [...b.flags].filter((f) => f & 1).length;
  const subOn = Math.max(...on.substeps.slice(-60));
  const subOff = Math.max(...off.substeps.slice(-60));
  const asleepAt = Math.max(...on.sleeps.map((s) => s.step));
  t.diagnostic(`standing: asleep whole from step ${asleepAt}; readings ${[...sl.ratio].map((r) => r.toFixed(3)).join(" ")}; substeps ${subOn} asleep against ${subOff} live`);
  assert.equal(bodies, 1, "one body");
  assert.ok([...b.segmentBody].every((s) => s === b.segmentBody[0]), "every segment rides it");
  assert.equal(on.wakes.length, 0, "nothing wakes it");
  assert.ok(subOn < subOff, `substeps ${subOn} asleep, ${subOff} live`);
  on.world.free();
});

test("the legs asleep in flight wake on contact before the touchdown's first yield", async (t) => {
  const hop = prepareHop(HOPPER);
  const on = await run(hop.flight.setup, HOP_STEPS, (b, s) => hopCommands(b, hop, s));
  const off = await run(hop.flight.setup, HOP_STEPS, (b, s) => hopCommands(b, hop, s), false);
  const asleepBefore = LEGS.filter((leg) => {
    const sl = on.sleeps.filter((s) => s.segment === leg && s.step < 260).at(-1);
    const wk = on.wakes.filter((w) => w.segment === leg && w.step < 260).at(-1);
    return sl !== undefined && (wk === undefined || wk.step < sl.step);
  });
  const contact = on.wakes.filter((w) => LEGS.includes(w.segment) && w.step >= 260);
  const firstWake = Math.min(...contact.map((w) => w.step));
  t.diagnostic(`legs asleep before touchdown: ${asleepBefore.join(" ")}; wakes ${contact.map((w) => `${w.segment}@${w.step} (reason ${w.reason})`).join(", ")}; first yield ${on.firstYield} (sleeping off: ${off.firstYield})`);
  assert.equal(asleepBefore.length, 4, "all four legs slept in flight");
  assert.ok(contact.some((w) => w.reason === WAKE_CONTACT), "a foot's contact wakes them");
  assert.ok(firstWake <= on.firstYield, `woke at ${firstWake}, first yield at ${on.firstYield}`);
  assert.equal(on.firstYield, off.firstYield, "the touchdown starts on the same step");
});

test("ten simulated minutes near the thresholds: no energy gained, no switch faster than allowed", async (t) => {
  // Standing: the legs read 98% of the sleep level for ten minutes.
  const stand = prepareDrop(HOPPER, PAD, 0);
  const speeds = (w: World, prev: Float64Array) => {
    const p = w.positions();
    let sum = 0;
    for (let i = 0; i < p.length; i++) sum += ((p[i]! - prev[i]!) * 60) ** 2;
    prev.set(p);
    return sum;
  };
  const world = new World(await loadCore());
  world.apply(stand.setup);
  const b0 = new CommandBatch();
  b0.setSleeping(true);
  world.apply(b0);
  const prev = world.positions().slice();
  let second = 0;
  let last = 0;
  let switches = 0;
  for (let s = 0; s < TEN_MINUTES; s++) {
    world.step();
    switches += world.events().filter((e) => e.kind === EVENT_WAKE || e.kind === EVENT_SLEEP).length;
    const v2 = speeds(world, prev);
    if (s >= 3600 && s < 7200) second = Math.max(second, v2);
    if (s >= TEN_MINUTES - 3600) last = Math.max(last, v2);
  }
  t.diagnostic(`standing ten minutes: ${switches} switches; sum of squared node speeds, worst in minute 2 ${second.toExponential(2)}, in minute 10 ${last.toExponential(2)} m²/s²`);
  assert.equal(switches, 8, "the eight live segments sleep once and stay asleep");
  assert.ok(last <= second + 1e-12, "no energy gained");
  world.free();

  // A load cycled across the wake level: a downward push on the pod of 30% of the craft's weight,
  // on for 2.5 s and off for 2.5 s. Ten minutes with RIBCAGE_SLOW, two otherwise.
  const steps = process.env.RIBCAGE_SLOW ? TEN_MINUTES : 7200;
  const nodes = partNodes(stand.loaded.craft, "pod");
  const cycled = await run(prepareDrop(HOPPER, PAD, 0).setup, steps, (b, s, mass) => {
    if (s >= 600 && Math.floor(s / 150) % 2 === 0) for (const n of nodes) b.addForce(n, 0, (-0.3 * 9.81 * mass) / nodes.length, 0);
  });
  const w = worstSwitch(cycled);
  const cycle = shortestCycle(cycled);
  t.diagnostic(`cycled load, ${steps / 3600} min: ${cycled.wakes.length} wakes, ${cycled.sleeps.length} sleeps, shortest wake-to-sleep ${cycle} steps; worst energy on sleep ${w.sleepUp.toExponential(1)}, on wake ${w.wakeChange.toExponential(1)}`);
  assert.ok(cycled.wakes.length > 0 && cycled.sleeps.length > 0, "the load switches it");
  assert.ok(cycle >= MIN_CYCLE, `slept ${cycle} steps after waking`);
  assert.ok(w.sleepUp <= 1e-12 && w.wakeChange <= 1e-12, "no energy from a switch");
  // At most one sleep per segment per minimum cycle.
  assert.ok(cycled.sleeps.length <= (10 * steps) / MIN_CYCLE);
});

test("the same hop with sleeping twice gives the same hash", async () => {
  const hop = prepareHop(HOPPER);
  const hash = async () => {
    const r = await run(hop.flight.setup, HOP_STEPS, (b, s) => hopCommands(b, hop, s), true, true);
    const h = r.world.stateHash;
    r.world.free();
    return h;
  };
  assert.equal(await hash(), await hash());
});
