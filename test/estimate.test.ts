/**
 * Plan 005, phase 5.7: the stress estimate, read-only. Checks it against the solver's own beam
 * force (`last_lambda`, the true spring stress) on the hopper (all-live and rigid; the hop and
 * drops) and on the rover with its chassis cut into a front and a rear segment, and that turning
 * it on changes no hash. Bounds are stated per case below and logged in ER 005, phase 5.7.
 *
 * What is compared, per seam and per step: the estimate's worst beam load ratio (|T| / F_yield)
 * against the measured worst ratio of the same beams. The estimate is a step average and includes
 * the dampers and any joint; the measurement is the last substep's spring force alone, so single
 * steps differ under impact. The bounds are on steps where the seam carries at least a tenth of
 * its run's peak (below that, both are noise around zero).
 */
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";
import { CommandBatch, NO_SEGMENT, SEG_NEVER_LIVE } from "../src/commands.ts";
import { Core } from "../src/core.ts";
import { HOP_STEPS, hopCommands, PAD, prepareDrop, prepareHop } from "../src/landing.ts";
import { driveRover, rover, type RoverParams } from "../src/parts/rover.ts";
import { compilePart } from "../src/parts/part.ts";
import { compileTerrain, landingField } from "../src/terrain.ts";
import { World } from "../src/world.ts";

const WASM_PATH = new URL("../target/wasm32-unknown-unknown/release/beam_core.wasm", import.meta.url);
const HOPPER = readFileSync(new URL("./fixtures/hopper.craft.json", import.meta.url), "utf8");
const HOP_HASH = 0xd92f2aad01c0e048n;
const RIGID_HOP_HASH = 0x5a1c383b76d89ea6n;
const loadCore = async (): Promise<Core> => Core.load(new Uint8Array(readFileSync(WASM_PATH)));

/** Per-seam traces of a run: estimated and measured worst ratio per step. */
interface Trace {
  est: number[][];
  meas: number[][];
  hash: bigint;
  physics: bigint;
  positions: Float64Array;
}

async function trace(setup: CommandBatch, steps: number, perStep: (b: CommandBatch, s: number) => void, on = true): Promise<Trace> {
  const core = await loadCore();
  const world = new World(core);
  assert.equal(world.apply(setup).rejected, 0);
  world.setEstimate(on);
  const est: number[][] = [];
  const meas: number[][] = [];
  for (let s = 0; s < steps; s++) {
    const b = new CommandBatch();
    perStep(b, s);
    world.apply(b);
    world.step();
    if (!on) continue;
    const e = world.estimate();
    for (let k = 0; k < e.ratio.length; k++) {
      (est[k] ??= []).push(e.ratio[k]!);
      (meas[k] ??= []).push(e.measured[k]!);
    }
  }
  const out = { est, meas, hash: world.stateHash, physics: world.physicsHash, positions: world.positions().slice() };
  world.free();
  return out;
}

interface Stats {
  /** Seams compared (with a finite, nonzero measured peak). */
  seams: number;
  /** Steps compared over all seams. */
  samples: number;
  /** Percentiles of estimate / measured over the compared steps. */
  p5: number;
  p50: number;
  p95: number;
  /** Worst over seams of (peak estimate / peak measured) - 1, low side and high side. */
  peakLow: number;
  peakHigh: number;
}

function stats(t: Trace, from = 0, floor = 0.1): Stats {
  const q: number[] = [];
  let seams = 0;
  let peakLow = 0;
  let peakHigh = 0;
  for (let k = 0; k < t.est.length; k++) {
    const e = t.est[k]!.slice(from);
    const m = t.meas[k]!.slice(from);
    const mPeak = Math.max(0, ...m.filter(Number.isFinite));
    if (!(mPeak > 0)) continue;
    seams++;
    const ePeak = Math.max(...e);
    peakLow = Math.min(peakLow, ePeak / mPeak - 1);
    peakHigh = Math.max(peakHigh, ePeak / mPeak - 1);
    for (let s = 0; s < m.length; s++) if (m[s]! >= floor * mPeak) q.push(e[s]! / m[s]!);
  }
  q.sort((a, b) => a - b);
  const at = (p: number) => q[Math.min(q.length - 1, Math.floor(p * q.length))]!;
  return { seams, samples: q.length, p5: at(0.05), p50: at(0.5), p95: at(0.95), peakLow, peakHigh };
}

/** The hop's phases in steps: powered, coasting, the touchdown and the rest after it. */
const HOP_PHASES: [string, number, number][] = [["powered", 10, 220], ["coast", 220, 268], ["touchdown", 268, 420], ["rest", 420, HOP_STEPS]];

const show = (name: string, s: Stats) =>
  `${name}: ${s.seams} seams, ${s.samples} steps; estimate/measured p5 ${s.p5.toFixed(3)}, median ${s.p50.toFixed(3)}, p95 ${s.p95.toFixed(3)}; peak ${(100 * s.peakLow).toFixed(1)}% to +${(100 * s.peakHigh).toFixed(1)}%`;

test("turning the estimate on changes no hash: the hop, all-live and rigid", async () => {
  const live = prepareHop(HOPPER, { segments: true, rigid: false });
  const a = await trace(live.flight.setup, HOP_STEPS, (b, s) => hopCommands(b, live, s));
  assert.equal(a.physics, HOP_HASH, "all-live hop with segments and the estimate on");
  const rigid = prepareHop(HOPPER);
  const r = await trace(rigid.flight.setup, HOP_STEPS, (b, s) => hopCommands(b, rigid, s));
  assert.equal(r.hash, RIGID_HOP_HASH, "rigid hop with the estimate on");
});

test("hopper: the estimate against the measured beam stress", async (t) => {
  const live = prepareHop(HOPPER, { segments: true, rigid: false });
  const hop = await trace(live.flight.setup, HOP_STEPS, (b, s) => hopCommands(b, live, s));
  const sHop = stats(hop);
  t.diagnostic(show("hop, all-live", sHop));
  const window = (tr: Trace, a: number, b: number): Trace => ({ ...tr, est: tr.est.map((e) => e.slice(a, b)), meas: tr.meas.map((m) => m.slice(a, b)) });
  for (const [name, a, b] of HOP_PHASES) t.diagnostic(show(`hop all-live, ${name}`, stats(window(hop, a, b))));
  // Standing on the pad before the engine: the first second, after a settle.
  const stand = prepareDrop(HOPPER, PAD, 0, undefined, { segments: true, rigid: false });
  const st = await trace(stand.setup, 120, () => {});
  const sStand = stats(st, 60);
  t.diagnostic(show("standing, all-live", sStand));
  const rows: [string, Stats][] = [["hop", sHop], ["standing", sStand]];
  for (const v of [3, 7]) {
    const d = prepareDrop(HOPPER, PAD, v, undefined, { segments: true, rigid: false });
    const tr = await trace(d.setup, 240, () => {});
    const s = stats(tr);
    t.diagnostic(show(`drop ${v} m/s, all-live`, s));
    rows.push([`drop ${v}`, s]);
  }
  const rigid = prepareHop(HOPPER);
  const rh = await trace(rigid.flight.setup, HOP_STEPS, (b, s) => hopCommands(b, rigid, s));
  const sRigid = stats(rh);
  t.diagnostic(show("hop, rigid", sRigid));
  for (const [name, a, b] of HOP_PHASES) t.diagnostic(show(`hop rigid, ${name}`, stats(window(rh, a, b))));
  rows.push(["hop rigid", sRigid]);
  for (const [name, s] of rows) {
    assert.ok(s.seams > 0 && s.samples > 0, `${name}: nothing compared`);
  }
  const phase = (tr: Trace, i: number) => stats(window(tr, HOP_PHASES[i]![1], HOP_PHASES[i]![2]));
  const within = (name: string, v: number, lo: number, hi: number) => assert.ok(v >= lo && v <= hi, `${name}: ${v} outside [${lo}, ${hi}]`);
  // Static and quasi-static loads (standing, the powered climb): the median within 10%, one-sided peaks.
  within("standing median", sStand.p50, 0.9, 1.05);
  within("standing peak low", sStand.peakLow, -0.2, 0);
  within("standing peak high", sStand.peakHigh, 0, 0.05);
  for (const [name, tr, low] of [["all-live", hop, -0.6], ["rigid", rh, -0.25]] as const) {
    const s = phase(tr, 0);
    within(`${name} powered median`, s.p50, 0.9, 1.05);
    within(`${name} powered peak low`, s.peakLow, low, 0);
    within(`${name} powered peak high`, s.peakHigh, 0, 0.1);
    // The touchdown: steps differ (step average with dampers against the last substep), peaks hold.
    within(`${name} touchdown peak high`, phase(tr, 2).peakHigh, 0, name === "rigid" ? 0.1 : 0.5);
    // The known limit: after yield the beams hold locked-in stress that crosses no cut.
    within(`${name} rest median (locked-in stress unseen)`, phase(tr, 3).p50, 0, 0.5);
  }
  for (const [name, s] of rows.filter(([n]) => n.startsWith("drop"))) {
    within(`${name} median`, s.p50, 0.8, 1.05);
    within(`${name} peak low`, s.peakLow, -0.45, 0);
    within(`${name} peak high`, s.peakHigh, 0, 0.05);
  }
});

test("the estimate's cost on the all-live hop", async (t) => {
  const live = prepareHop(HOPPER, { segments: true, rigid: false });
  const time = async (on: boolean) => {
    const t0 = performance.now();
    await trace(live.flight.setup, HOP_STEPS, (b, s) => hopCommands(b, live, s), on);
    return (performance.now() - t0) / HOP_STEPS;
  };
  // Best of three, interleaved: other test files run beside this one.
  await time(false);
  let off = Infinity;
  let on = Infinity;
  for (let i = 0; i < 3; i++) {
    off = Math.min(off, await time(false));
    on = Math.min(on, await time(true));
  }
  t.diagnostic(`all-live hop: ${off.toFixed(3)} ms/step off, ${on.toFixed(3)} ms/step on (reads included)`);
  // Stated bound: under a quarter of the step (it measured about 1.5%; the slack is for timer noise).
  assert.ok(on <= 1.25 * off, `the estimate costs ${on} ms/step against ${off}`);
});

const PARAMS: RoverParams = {
  length: 2,
  width: 1.2,
  height: 0.5,
  x: -6,
  z: 0,
  axleHeight: 0.452,
  chassisMass: 340,
  stub: 0.3,
  wheel: { rimNodes: 24, radius: 0.4, material: "steel", memberArea: 2e-4, mass: 30, nodeRadius: 0.05 },
};
const CAP = 150;

/**
 * The rover as `compileRover` builds it, but with the chassis cut in two segments for the test:
 * the front half (+x: corners 0 and 3, their tops 4 and 7, stubs 8 and 11) is the root, the rear
 * half its child, and each wheel a child of its own half. Sixteen chassis beams cross the cut.
 * The motion is the stock rover's: segments do not enter the solve. (The physics hash is not
 * equal: a body's slot is its lowest segment, so the cut moves the wheels from slots 1-4 to 2-5.)
 */
function cutRover(batch: CommandBatch, split: boolean) {
  const r = rover(PARAMS);
  compileTerrain(batch, landingField());
  compilePart(batch, r.craft);
  const front = new Set([0, 3, 4, 7, 8, 11]);
  // Segments: 0 front (or the whole chassis), 1 rear, then the wheels.
  batch.addSegment(NO_SEGMENT, 0, 0, 0);
  if (split) batch.addSegment(0, 0, 0, 0);
  const half = (corner: number) => (split && !front.has(corner) ? 1 : 0);
  const base = split ? 2 : 1;
  for (const [k] of r.wheels.entries()) batch.addSegment(half(k), SEG_NEVER_LIVE, 0, 0);
  for (const [i, n] of r.craft.nodes.entries()) {
    const seg = n.segment === 0 ? half(i) : base + (n.segment - 1);
    batch.setNodeSegment(i, seg);
  }
  for (let k = 0; k < r.wheels.length; k++) batch.makeRigid(base + k);
  for (const w of r.wheels) {
    const [a0, a1, ra, b0, b1, rb] = w.hinge;
    batch.addJoint(a0, a1, ra, b0, b1, rb);
  }
  return r;
}

/**
 * The rover's script: settle, a 4 kN push up on the front +z wheel's hub (meant as a rock; it launches the
 * rover through ground contact with the rigid wheels, a finding for 5.3/5.6 in ER 005 phase 5.7), drive, stop.
 */
const ROCK_FROM = 120;
const ROCK_TO = 300;
const DRIVE_FROM = 300;
const DRIVE_TO = 540;
const ROVER_STEPS = 720;
function roverScript(r: ReturnType<typeof rover>) {
  const hub = r.wheels[0]!.hinge.slice(3, 5);
  return (b: CommandBatch, s: number) => {
    if (s >= ROCK_FROM && s < ROCK_TO) for (const n of hub) b.addForce(n, 0, 2000, 0);
    if (s === DRIVE_FROM) driveRover(b, r, 2, CAP);
    if (s === DRIVE_TO) driveRover(b, r, 0, CAP);
  };
}

test("rover: the cut changes no physics, and the estimate against the measured chassis stress", async (t) => {
  const plainBatch = new CommandBatch();
  const plain = cutRover(plainBatch, false);
  const p = await trace(plainBatch, ROVER_STEPS, roverScript(plain), false);
  const cutBatch = new CommandBatch();
  const cut = cutRover(cutBatch, true);
  const c = await trace(cutBatch, ROVER_STEPS, roverScript(cut));
  assert.deepEqual(c.positions, p.positions, "the front/rear cut and the estimate change no motion");
  // Seam 0 is the chassis cut; seams 1..4 are the wheel hinges (no beams).
  assert.ok(c.est.length === 5);
  for (let k = 1; k < 5; k++) assert.ok(c.est[k]!.every((v) => v === 0), `wheel seam ${k} has no beams`);
  const phases: [string, number, number][] = [["standing", 60, ROCK_FROM], ["rock", ROCK_FROM + 30, ROCK_TO], ["driven", DRIVE_FROM + 30, DRIVE_TO], ["stopping", DRIVE_TO, ROVER_STEPS]];
  const within = (name: string, v: number, lo: number, hi: number) => assert.ok(v >= lo && v <= hi, `rover ${name}: ${v} outside [${lo}, ${hi}]`);
  const bounds: Record<string, (s: Stats, e: number, m: number) => void> = {
    // The known limit: standing, the frame's self-balanced stress under its own weight crosses no cut.
    standing: (_s, e, m) => within("standing estimate / measured", e / m, 0, 0.1),
    rock: (s) => {
      within("rock peak", s.peakLow, -0.2, 0);
      within("rock peak high", s.peakHigh, 0, 0.05);
    },
    driven: (s) => {
      within("driven peak", s.peakLow, -0.2, 0);
      within("driven peak high", s.peakHigh, 0, 0.05);
    },
    stopping: (s) => {
      within("stopping median", s.p50, 0.9, 1.1);
      within("stopping peak high", s.peakHigh, 0, 0.1);
    },
  };
  for (const [name, a, b] of phases) {
    const sub: typeof c = { ...c, est: [c.est[0]!.slice(a, b)], meas: [c.meas[0]!.slice(a, b)] };
    const s = stats(sub);
    const ePeak = Math.max(...sub.est[0]!);
    const mPeak = Math.max(...sub.meas[0]!);
    t.diagnostic(`${show(`rover ${name}`, s)}; peak estimate ${ePeak.toExponential(3)}, measured ${mPeak.toExponential(3)}`);
    bounds[name]!(s, ePeak, mPeak);
  }
});
