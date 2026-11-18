import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";
import { CommandBatch } from "../src/commands.ts";
import { BEAM_BROKEN, Core, EVENT_BREAK, EVENT_FREEZE, EVENT_YIELD } from "../src/core.ts";
import { GRAVITY } from "../src/design.ts";
import { MATERIALS, STIFFNESS_SCALE, dampingFor, member, reducedMass, type MaterialId } from "../src/parts/material.ts";
import {
  NODE_MASS_RATIO_BUDGET,
  SUBSTEP_BUDGET,
  SUBSTEP_MAX,
  SUBSTEP_MIN,
  ZETA_STRUCTURE,
  compilePart,
  emptyPart,
  partBudget,
  partMass,
  resolveBeam,
  type Part,
} from "../src/parts/part.ts";
import { engine, type EngineParams } from "../src/parts/engine.ts";
import { leg, type LegParams } from "../src/parts/leg.ts";
import { pod, type PodParams } from "../src/parts/pod.ts";
import { RING_COUNTS, ringPoint, unitCosSin, distributeMass } from "../src/parts/shared.ts";
import { ringBracing, tank, type TankParams } from "../src/parts/tank.ts";
import { truss, type TrussParams } from "../src/parts/truss.ts";
import { compileTerrain, flatTerrain } from "../src/terrain.ts";
import { World } from "../src/world.ts";

const WASM_PATH = new URL("../target/wasm32-unknown-unknown/release/beam_core.wasm", import.meta.url);

async function loadCore(): Promise<Core> {
  try {
    return await Core.load(new Uint8Array(readFileSync(WASM_PATH)));
  } catch (e) {
    throw new Error(`beam_core.wasm is missing or stale: run \`pnpm build:wasm\` first (${String(e)})`);
  }
}

const STEP = 1 / 60;

/** Two nodes `length` metres apart on the x axis, joined by one member. */
function strut(material: MaterialId, area: number, length: number, massA: number, massB: number, zeta = ZETA_STRUCTURE): Part {
  const p = emptyPart("strut");
  p.segments.push({ neverLive: false });
  p.nodes.push({ x: 0, y: 0, z: 0, mass: massA, radius: 0.05, segment: 0 });
  p.nodes.push({ x: length, y: 0, z: 0, mass: massB, radius: 0.05, segment: 0 });
  p.beams.push({ a: 0, b: 1, material, area, zeta, ring: false });
  return p;
}

// ---------------------------------------------------------------------------------------------
// Materials

test("a member's stiffness is s·E·A and its yield force is σ_y·A, whatever the scale", () => {
  assert.equal(STIFFNESS_SCALE, 0.1);
  for (const id of ["steel", "aluminium"] as const) {
    const m = MATERIALS[id];
    const a = 4e-4;
    const mem = member(id, a);
    assert.equal(mem.ea, STIFFNESS_SCALE * m.youngPa * a);
    assert.ok(Math.abs(mem.yieldForce! / (mem.ea * mem.yieldStrain) - 1) < 1e-12, `${id}: EA · yield strain is σ_y · A`);
    assert.equal(mem.yieldForce, m.yieldPa! * a);
    assert.equal(mem.massPerLength, m.density * a);
  }
  // ER 004's table.
  assert.ok(Math.abs(member("steel", 1).yieldStrain - 0.0175) < 1e-12);
  assert.ok(Math.abs(member("aluminium", 1).yieldStrain - 0.0285714285714) < 1e-9);
  assert.equal(member("steel", 1).breakStrain, 0.1);
  assert.equal(member("aluminium", 1).breakStrain, 0.08);
  const carbon = member("carbon", 1);
  assert.equal(carbon.breakStrain, 0.05);
  assert.equal(carbon.yieldForce, null);
  assert.ok(carbon.yieldStrain > carbon.breakStrain, "a brittle member breaks before it can yield");
  // 600 MPa strength over the scaled modulus is the break strain.
  assert.ok(Math.abs(carbon.breakStrain * carbon.ea - 600e6 * 1) < 1);
});

test("bad members are refused", () => {
  assert.throws(() => member("steel", 0), /area/);
  assert.throws(() => member("steel", -1), /area/);
  assert.throws(() => member("steel", Number.NaN), /area/);
  assert.throws(() => member("unobtainium" as MaterialId, 1), /unknown material/);
});

test("damping from a ratio uses the reduced mass, and a pinned end counts as infinitely heavy", () => {
  assert.equal(reducedMass(2, 2), 1);
  assert.equal(reducedMass(300, 100), 75);
  assert.equal(reducedMass(Infinity, 5), 5);
  assert.equal(reducedMass(5, Infinity), 5);
  assert.equal(reducedMass(Infinity, Infinity), 0);
  // c = ζ · 2 · sqrt(k · μ), k = EA / L
  assert.equal(dampingFor(30000, 1, 300, 100, 0.1), 0.1 * 2 * Math.sqrt(30000 * 75));
  assert.equal(dampingFor(30000, 2, 300, 100, 0.1), 0.1 * 2 * Math.sqrt(15000 * 75));
});

// ---------------------------------------------------------------------------------------------
// Part: mass, compile, budget

test("a part's mass is the sum of its nodes, in index order, and a lattice compiles to nodes then beams", async () => {
  const p = strut("steel", 4e-4, 1.5, 12.5, 37.5);
  assert.equal(partMass(p), 50);
  assert.equal(partMass(emptyPart("nothing")), 0);

  const batch = new CommandBatch();
  compilePart(batch, p);
  assert.equal(batch.count, 3);
  const core = await loadCore();
  const world = new World(core);
  assert.equal(world.apply(batch).rejected, 0);
  assert.equal(world.nodeCount, 2);
  assert.equal(world.beamCount, 1);
  world.free();

  // The same part always gives the same bytes.
  const again = new CommandBatch();
  compilePart(again, strut("steel", 4e-4, 1.5, 12.5, 37.5));
  assert.deepEqual(again.bytes, batch.bytes);
});

test("a part with a bad node or beam is refused with a reason", () => {
  const noMass = strut("steel", 4e-4, 1, 0, 10);
  assert.throws(() => compilePart(new CommandBatch(), noMass), /positive mass/);
  const stacked = strut("steel", 4e-4, 0, 10, 10);
  assert.throws(() => compilePart(new CommandBatch(), stacked), /no length/);
  const lost = strut("steel", 4e-4, 1, 10, 10);
  lost.beams[0]!.b = 7;
  assert.throws(() => compilePart(new CommandBatch(), lost), /do not both exist/);
  assert.throws(() => partBudget(lost), /do not both exist/);
  const self = strut("steel", 4e-4, 1, 10, 10);
  self.beams[0]!.b = 0;
  assert.throws(() => compilePart(new CommandBatch(), self), /to itself/);
});

test("a beam override replaces the material's stiffness and strains", () => {
  const p = strut("aluminium", 4e-4, 2, 10, 10);
  p.beams[0]!.override = { ea: 1e5, yieldStrain: 0.5, breakStrain: 0.9 };
  const r = resolveBeam(p, p.beams[0]!);
  assert.equal(r.ea, 1e5);
  assert.equal(r.compliance, 2 / 1e5);
  assert.equal(r.yieldStrain, 0.5);
  assert.equal(r.breakStrain, 0.9);
});

/** A unit cube: 8 nodes, 12 edges, 12 face diagonals and 4 body diagonals, with a mix of masses. */
function cube(area: number, materialFor: (k: number) => MaterialId = () => "steel"): Part {
  const p = emptyPart("cube");
  p.segments.push({ neverLive: false });
  for (let i = 0; i < 8; i++) {
    p.nodes.push({ x: i & 1, y: (i >> 1) & 1, z: (i >> 2) & 1, mass: 10 + 3 * i, radius: 0.05, segment: 0 });
  }
  const pairs: [number, number][] = [];
  for (let a = 0; a < 8; a++) for (let b = a + 1; b < 8; b++) pairs.push([a, b]); // edges, face and body diagonals: all 28
  for (const [k, [a, b]] of pairs.entries()) p.beams.push({ a, b, material: materialFor(k), area, zeta: ZETA_STRUCTURE, ring: false });
  return p;
}

test("the reported substep count equals what the core uses, on lattices from the minimum to the cap", async () => {
  const core = await loadCore();
  const seen = new Set<number>();
  const lattices: [string, Part][] = [
    ["soft steel cube", cube(1e-5)],
    ["steel cube 1e-4", cube(1e-4)],
    ["steel cube 3e-4", cube(3e-4)],
    ["steel cube 1e-3 (cap)", cube(1e-3)],
    ["mixed materials", cube(2e-4, (k) => (["steel", "aluminium", "carbon"] as const)[k % 3]!)],
    ["one stiff strut among soft ones", (() => { const q = cube(1e-5); q.beams[5]!.area = 5e-3; return q; })()],
    ["a sprung bay override", (() => { const q = cube(2e-4); q.beams[3]!.override = { ea: 1e3 }; return q; })()],
    ["a single strut", strut("aluminium", 1e-4, 1, 50, 50)],
  ];
  for (const [name, part] of lattices) {
    const batch = new CommandBatch();
    compilePart(batch, part);
    const world = new World(core);
    assert.equal(world.apply(batch).rejected, 0, name);
    const used = world.step();
    const reported = partBudget(part).substeps;
    assert.equal(reported, used, `${name}: reported ${reported}, the core used ${used}`);
    seen.add(used);
    world.free();
  }
  assert.ok(seen.has(SUBSTEP_MIN), "a lattice at the minimum");
  assert.ok(seen.has(SUBSTEP_MAX), "a lattice at the cap");
  assert.ok([...seen].some((n) => n > SUBSTEP_MIN && n < SUBSTEP_MAX), "a lattice in between");
});

test("the budget check reports the node-mass ratio and each budget a part misses", () => {
  const even = strut("aluminium", 1e-4, 1, 20, 20);
  const b = partBudget(even);
  assert.equal(b.massRatio, 1);
  assert.equal(b.minNodeMass, 20);
  assert.equal(b.maxNodeMass, 20);
  assert.equal(b.withinBudget, true);
  assert.deepEqual(b.problems, []);
  assert.equal(b.omegaMax, Math.sqrt((1 / 20 + 1 / 20) / resolveBeam(even, even.beams[0]!).compliance));

  const lopsided = strut("aluminium", 1e-4, 1, 10, 150);
  const l = partBudget(lopsided);
  assert.equal(l.massRatio, 15);
  assert.equal(l.withinBudget, false);
  assert.ok(l.problems.some((s) => s.includes("times the lightest")));
  assert.ok(15 > NODE_MASS_RATIO_BUDGET);

  const stiff = strut("steel", 5e-2, 0.5, 5, 5);
  const s = partBudget(stiff);
  assert.equal(s.substeps, SUBSTEP_MAX);
  assert.ok(s.substeps > SUBSTEP_BUDGET);
  assert.equal(s.withinBudget, false);
  assert.ok(s.problems.some((t) => t.includes("substeps")));

  const none = partBudget(emptyPart("nothing"));
  assert.equal(none.substeps, SUBSTEP_MIN);
  assert.equal(none.massRatio, 0);
  assert.equal(none.withinBudget, true);
});

// ---------------------------------------------------------------------------------------------
// A strut pulled and pushed in the core

const STRUT_AREA = 4e-4;
const STRUT_LENGTH = 1;
const STRUT_MASS = 1000;
/** The pull reaches this share of the yield force over `RAMP_STEPS`, slow enough to be static. */
const RAMP_PEAK = 1.02;
const RAMP_STEPS = 3000;
/**
 * Past yield the pull is held this far over the yield force, so the strut flows slowly to its break
 * (the beam's damper limits the flow to `excess / c`; this gives a few centimetres a second, about
 * 0.03% of strain a step, so the step that reports the break is not far past it).
 */
const HOLD_EXCESS = 1e-3;
const HOLD_LIMIT = 1500;

interface PullResult {
  /** Force applied (newtons, on each end) on the step the first yield event arrived. */
  forceAtYield: number;
  /** Strain, from the positions, at the end of that step. */
  strainAtYield: number;
  /** |strain| at the end of the step before the break event, and at the end of the step that reported it. */
  strainBeforeBreak: number;
  strainAtBreak: number;
  /** The strain the break event itself reports. */
  eventStrain: number;
}

/** Pulls (`sign` +1) or pushes (`sign` −1) a strut along its axis, with gravity cancelled on both ends. */
function pull(core: Core, material: MaterialId, sign: 1 | -1): { result: PullResult; part: Part } {
  const part = strut(material, STRUT_AREA, STRUT_LENGTH, STRUT_MASS, STRUT_MASS);
  const mem = member(material, STRUT_AREA);
  const yieldForce = mem.yieldForce!;
  const world = new World(core);
  const build = new CommandBatch();
  compilePart(build, part);
  assert.equal(world.apply(build).rejected, 0);

  const strain = (): number => {
    const p = world.positions();
    return (Math.sqrt((p[3]! - p[0]!) ** 2 + (p[4]! - p[1]!) ** 2 + (p[5]! - p[2]!) ** 2) - STRUT_LENGTH) / STRUT_LENGTH;
  };
  const push = (force: number): void => {
    const cmds = new CommandBatch();
    // Gravity is cancelled on both nodes, so the strut feels only the pull.
    cmds.addForce(0, -sign * force, STRUT_MASS * GRAVITY, 0);
    cmds.addForce(1, sign * force, STRUT_MASS * GRAVITY, 0);
    assert.equal(world.apply(cmds).rejected, 0);
  };

  let forceAtYield = Number.NaN;
  let strainAtYield = Number.NaN;
  let yielded = false;
  let previousStrain = 0;
  for (let s = 0; s < RAMP_STEPS + HOLD_LIMIT; s++) {
    const force = yielded ? yieldForce * (1 + HOLD_EXCESS) : (yieldForce * RAMP_PEAK * (s + 1)) / RAMP_STEPS;
    push(force);
    world.step();
    const now = Math.abs(strain());
    for (const e of world.events()) {
      if (e.kind === EVENT_YIELD && !yielded) {
        yielded = true;
        forceAtYield = force;
        strainAtYield = now;
      } else if (e.kind === EVENT_BREAK) {
        assert.ok(yielded, "a break with no yield first");
        world.free();
        return { result: { forceAtYield, strainAtYield, strainBeforeBreak: previousStrain, strainAtBreak: now, eventStrain: Math.abs(e.value) }, part };
      }
    }
    assert.ok(s < RAMP_STEPS || yielded, "the ramp ended and the strut had not yielded");
    previousStrain = now;
  }
  world.free();
  throw new Error(`${material} strut did not break within ${HOLD_LIMIT} held steps`);
}

for (const material of ["steel", "aluminium"] as const) {
  for (const [name, sign] of [["tension", 1], ["compression", -1]] as const) {
    test(`a ${material} strut in ${name} yields at σ_y·A within 1% and breaks at its stated strain`, async () => {
      const core = await loadCore();
      const { result: r } = pull(core, material, sign);
      const mem = member(material, STRUT_AREA);

      assert.ok(
        Math.abs(r.forceAtYield / mem.yieldForce! - 1) < 0.01,
        `yielded at ${r.forceAtYield} N, σ_y·A is ${mem.yieldForce} N (${(100 * (r.forceAtYield / mem.yieldForce! - 1)).toFixed(3)}%)`,
      );
      // The strain it had then is the yield strain, since the pull is static.
      assert.ok(Math.abs(r.strainAtYield / mem.yieldStrain - 1) < 0.02, `strain at yield ${r.strainAtYield}, stated ${mem.yieldStrain}`);

      // Break is on total strain, checked once a step: the step before it is under, the step that
      // reports it is at or over by no more than one step of flow.
      assert.ok(r.strainBeforeBreak < mem.breakStrain, `the step before the break was already at ${r.strainBeforeBreak}`);
      assert.ok(r.strainAtBreak >= mem.breakStrain * (1 - 1e-9), `broke at ${r.strainAtBreak}, stated ${mem.breakStrain}`);
      assert.ok(r.strainAtBreak - mem.breakStrain < 0.005, `broke ${r.strainAtBreak - mem.breakStrain} past the stated strain`);
      assert.ok(Math.abs(r.eventStrain - r.strainAtBreak) < 1e-9, "the event reports the strain the positions give");
    });
  }
}

// ---------------------------------------------------------------------------------------------
// Damping from a ratio

test("damping from ζ on a two-node spring decays at ζ·ω_n within 5%", async () => {
  const core = await loadCore();
  const massA = 300;
  const massB = 100;
  const zeta = 0.3;
  // A slow spring: the solver's own implicit stepping adds a decay of about ω²·h/2 (ER 004, phase
  // 4.1), which is 2% of the expected rate here and would be 40% at ω = 20 rad/s.
  const omegaN = 3;
  const mu = reducedMass(massA, massB);
  const k = omegaN * omegaN * mu;
  // Aluminium at the area that gives exactly this stiffness over 1 m.
  const area = k / (STIFFNESS_SCALE * MATERIALS.aluminium.youngPa);
  const part = strut("aluminium", area, 1, massA, massB, zeta);
  assert.ok(Math.abs(resolveBeam(part, part.beams[0]!).ea / k - 1) < 1e-12);

  const world = new World(core);
  const build = new CommandBatch();
  compilePart(build, part);
  assert.equal(world.apply(build).rejected, 0);

  // One step of equal and opposite push gives the two ends a relative speed, then the spring rings.
  const kick = new CommandBatch();
  // (small enough that the ringing stays well under this soft member's yield force)
  kick.addForce(0, -100, 0, 0);
  kick.addForce(1, 100, 0, 0);
  assert.equal(world.apply(kick).rejected, 0);
  const sep: number[] = [];
  const steps = 600;
  let failures = 0;
  for (let s = 0; s < steps; s++) {
    world.step();
    failures += world.events().filter((e) => e.kind === EVENT_YIELD || e.kind === EVENT_BREAK).length;
    const p = world.positions();
    sep.push(Math.sqrt((p[3]! - p[0]!) ** 2 + (p[4]! - p[1]!) ** 2 + (p[5]! - p[2]!) ** 2) - 1);
  }
  assert.equal(failures, 0, "the ringing stays elastic");
  world.free();

  // Peaks of |separation| with a parabola through the three samples round each, as (time, ln amplitude).
  const points: [number, number][] = [];
  const largest = Math.max(...sep.map(Math.abs));
  for (let i = 2; i < sep.length - 1; i++) {
    const y0 = Math.abs(sep[i - 1]!);
    const y1 = Math.abs(sep[i]!);
    const y2 = Math.abs(sep[i + 1]!);
    if (y1 < y0 || y1 < y2) continue;
    const denom = y0 - 2 * y1 + y2;
    const shift = denom === 0 ? 0 : (0.5 * (y0 - y2)) / denom;
    const peak = y1 - 0.25 * (y0 - y2) * shift;
    if (peak < 0.003 * largest) continue;
    points.push([(i + 1 + shift) * STEP, Math.log(peak)]);
  }
  assert.ok(points.length >= 6, `${points.length} peaks to fit`);
  const n = points.length;
  const mt = points.reduce((s, q) => s + q[0], 0) / n;
  const my = points.reduce((s, q) => s + q[1], 0) / n;
  const slope = points.reduce((s, q) => s + (q[0] - mt) * (q[1] - my), 0) / points.reduce((s, q) => s + (q[0] - mt) ** 2, 0);
  const rate = -slope;
  const expected = zeta * omegaN;
  assert.ok(Math.abs(rate / expected - 1) < 0.05, `decay rate ${rate.toFixed(4)} /s, expected ${expected} /s`);
});

// ---------------------------------------------------------------------------------------------
// Truss and tank generators (phase 4.2)

const TANKS: TankParams[] = [
  { radius: 1, length: 2.5, ringNodes: 8, rings: 5, material: "aluminium", wallArea: 5e-4, mass: 2000 },
  { radius: 0.5, length: 1, ringNodes: 4, rings: 2, material: "steel", wallArea: 2e-4, mass: 200 },
  { radius: 1.5, length: 3, ringNodes: 12, rings: 4, material: "aluminium", wallArea: 4e-4, mass: 1500 },
  { radius: 0.8, length: 2, ringNodes: 16, rings: 3, material: "carbon", wallArea: 3e-4, mass: 2000 },
  { radius: 2, length: 4, ringNodes: 24, rings: 6, material: "aluminium", wallArea: 6e-4, mass: 4000 },
  { radius: 1, length: 1.5, ringNodes: 6, rings: 3, material: "steel", wallArea: 3e-4, mass: 1000 },
];

const TRUSSES: TrussParams[] = [
  { section: "square", length: 4, width: 1, bays: 4, material: "aluminium", memberArea: 3e-4, mass: 300 },
  { section: "triangle", length: 3, width: 0.8, bays: 3, material: "steel", memberArea: 1.5e-4, mass: 200 },
  { section: "square", length: 1, width: 1, bays: 1, material: "carbon", memberArea: 2e-4, mass: 80 },
  { section: "triangle", length: 6, width: 1, bays: 6, material: "aluminium", memberArea: 3e-4, mass: 400 },
  { section: "square", length: 2, width: 0.5, bays: 2, material: "steel", memberArea: 1e-4, mass: 150 },
];

type Pose = "standing" | "on its side";

interface SweepCase {
  name: string;
  build: () => Part;
  stated: number;
  /** How the rest test sets it down. A slender truss rocks on its end nodes (ER 004, finding), so it lies on a face. */
  poses: Pose[];
}

const SWEEP: SweepCase[] = [
  ...TANKS.map((p, i): SweepCase => ({ name: `tank ${i} (${p.ringNodes} nodes, ${p.rings} rings, ${p.material})`, build: () => tank(p), stated: p.mass, poses: ["standing"] })),
  ...TRUSSES.map((p, i): SweepCase => ({ name: `truss ${i} (${p.section}, ${p.bays} bays, ${p.material})`, build: () => truss(p), stated: p.mass!, poses: p.length <= 2 * p.width ? ["standing", "on its side"] : ["on its side"] })),
];

// The hopper's own parts (ER 004), and a few other shapes, for the checks every part must pass.
const ENGINES: EngineParams[] = [
  { ringNodes: 8, topRadius: 1, nozzleRadius: 0.5, height: 1, material: "steel", memberArea: 4e-4, mass: 800, maxThrust: 80e3 },
  { ringNodes: 4, topRadius: 0.4, nozzleRadius: 0.8, height: 0.8, material: "aluminium", memberArea: 2e-4, mass: 300, maxThrust: 20e3 },
  { ringNodes: 12, topRadius: 1.2, nozzleRadius: 0.6, height: 1.5, material: "steel", memberArea: 3e-4, mass: 1000, maxThrust: 100e3 },
];
const PODS: PodParams[] = [
  { ringNodes: 8, bottomRadius: 1, topRadius: 0.6, height: 1.2, material: "aluminium", memberArea: 4e-4, mass: 1200 },
  { ringNodes: 6, bottomRadius: 0.7, topRadius: 0.4, height: 1, apexRise: 0.5, material: "carbon", memberArea: 2e-4, mass: 600 },
  { ringNodes: 16, bottomRadius: 1.4, topRadius: 0.8, height: 1.5, material: "aluminium", memberArea: 3e-4, mass: 1800 },
];
/** The hopper's leg: 75 kN crush force over 0.15 m, a spring that reaches it after 0.25 m (ER 004). */
const LEG: LegParams = {
  length: 3,
  footOffset: 1.5,
  footSize: 0.35,
  mountWidth: 0.35,
  mountHeight: 0.35,
  bodyWidth: 0.9,
  material: "aluminium",
  memberArea: 3e-4,
  mass: 150,
  sprungStiffness: 3e5,
  sprungStroke: 0.3,
  crushForce: 75e3,
  crushStroke: 0.15,
};
const LEGS: LegParams[] = [LEG, { ...LEG, footOffset: 0.9, crushForce: 40e3, mass: 150 }];

const PARTS: SweepCase[] = [
  ...ENGINES.map((p, i): SweepCase => ({ name: `engine ${i} (${p.ringNodes} nodes, ${p.material})`, build: () => engine(p), stated: p.mass, poses: ["standing"] })),
  ...PODS.map((p, i): SweepCase => ({ name: `pod ${i} (${p.ringNodes} nodes, ${p.material})`, build: () => pod(p), stated: p.mass, poses: ["standing"] })),
  ...LEGS.map((p, i): SweepCase => ({ name: `leg ${i} (offset ${p.footOffset} m, ${p.crushForce} N)`, build: () => leg(p), stated: p.mass, poses: ["on its side"] })),
];

/** The part turned so its axis is along x and lying on a wall face, just above the ground. */
function onItsSide(part: Part): Part {
  const low = Math.min(...part.nodes.map((n) => n.z));
  return { ...part, nodes: part.nodes.map((n) => ({ ...n, x: n.y, y: n.z - low + n.radius + 0.001, z: n.x })) };
}

function commandsOf(part: Part): Uint8Array {
  const batch = new CommandBatch();
  compilePart(batch, part);
  return batch.bytes.slice();
}

/** The same part with every node moved up by `dy`. */
function lifted(part: Part, dy: number): Part {
  return { ...part, nodes: part.nodes.map((n) => ({ ...n, y: n.y + dy })) };
}

test("the ring table gives unit vectors, exactly opposite points, and refuses other angles", () => {
  for (let u = 0; u < 24; u++) {
    if (u % 12 === 1 || u % 12 === 5 || u % 12 === 7 || u % 12 === 11) continue;
    const [c, s] = unitCosSin(u);
    assert.ok(Math.abs(c * c + s * s - 1) < 1e-15, `unit ${u}`);
    const [oc, os] = unitCosSin(u + 24);
    assert.ok(oc === -c && os === -s, `unit ${u} and its opposite`);
  }
  assert.deepEqual([...unitCosSin(0)], [1, 0]);
  assert.deepEqual([...unitCosSin(12)], [0, 1]);
  assert.deepEqual([...unitCosSin(24)], [-1, 0]);
  assert.ok(unitCosSin(4)[0] === Math.sqrt(3) / 2 && unitCosSin(4)[1] === 0.5, "30° from sqrt(3)/2 and 1/2");
  assert.ok(Math.abs(unitCosSin(2)[0] - 0.9659258262890683) < 1e-15, "15° from the half-angle square root");
  assert.throws(() => unitCosSin(1), /not a multiple/);
  assert.throws(() => unitCosSin(0.5), /whole number/);
  assert.deepEqual([...ringPoint(4, 1, 2)], [0, 2]);
});

for (const c of [...SWEEP, ...PARTS]) {
  test(`${c.name}: compiles, mass is exactly the stated mass, within the budget, same bytes twice`, async () => {
    const part = c.build();
    const batch = new CommandBatch();
    compilePart(batch, part);
    const world = new World(await loadCore());
    assert.equal(world.apply(batch).rejected, 0);
    assert.equal(world.nodeCount, part.nodes.length);
    assert.equal(world.beamCount, part.beams.length);
    world.free();

    assert.ok(partMass(part) === c.stated, `mass ${partMass(part)} is not the stated ${c.stated}`);
    for (const n of part.nodes) assert.ok(n.mass > 0);
    const b = partBudget(part);
    assert.ok(b.withinBudget, b.problems.join("; "));
    assert.ok(b.substeps <= SUBSTEP_BUDGET && b.massRatio <= NODE_MASS_RATIO_BUDGET);

    assert.deepEqual(commandsOf(part), commandsOf(c.build()));
  });

  for (const pose of c.poses) test(`${c.name}, ${pose}: rests on flat ground for 10 s with no yield and every node under 1 mm/s`, async () => {
    const part = c.build();
    const world = new World(await loadCore());
    const build = new CommandBatch();
    compileTerrain(build, flatTerrain(41, 41, -20, -20, 1));
    compilePart(build, pose === "standing" ? lifted(part, part.nodes[0]!.radius + 0.001) : onItsSide(part));
    assert.equal(world.apply(build).rejected, 0);
    let bad = 0;
    let prev = world.positions();
    let endSpeed = 0;
    for (let s = 0; s < 600; s++) {
      world.step();
      for (const e of world.events()) if (e.kind === EVENT_YIELD || e.kind === EVENT_BREAK || e.kind === EVENT_FREEZE) bad++;
      const p = world.positions();
      let fastest = 0;
      for (let i = 0; i < p.length; i += 3) {
        const d = Math.sqrt((p[i]! - prev[i]!) ** 2 + (p[i + 1]! - prev[i + 1]!) ** 2 + (p[i + 2]! - prev[i + 2]!) ** 2);
        fastest = Math.max(fastest, d / STEP);
      }
      endSpeed = fastest;
      prev = p;
    }
    assert.equal(bad, 0, "no yield, break or freeze");
    assert.ok(endSpeed < 1e-3, `still moving at ${endSpeed} m/s after 10 s`);
    for (let i = 1; i < prev.length; i += 3) assert.ok(prev[i]! > -0.01, "above the ground");
    world.free();
  });
}

test("tank ring points are exact: at the stated radius within 1e-12 m, opposite nodes exact negatives", () => {
  assert.deepEqual([...new Set(TANKS.map((t) => t.ringNodes))].sort((a, b) => a - b), [...RING_COUNTS], "the sweep covers every ring count");
  for (const p of TANKS) {
    const part = tank(p);
    const n = p.ringNodes;
    for (let k = 0; k < p.rings; k++) {
      for (let j = 0; j < n; j++) {
        const a = part.nodes[k * n + j]!;
        assert.ok(Math.abs(Math.sqrt(a.x * a.x + a.z * a.z) - p.radius) < 1e-12, `ring ${k} node ${j} off its radius`);
        const o = part.nodes[k * n + ((j + n / 2) % n)]!;
        assert.ok(o.x === -a.x && o.z === -a.z && o.y === a.y, `ring ${k} node ${j} and its opposite`);
        assert.ok(!Object.is(a.x, -0) && !Object.is(a.z, -0), "no negative zero");
      }
    }
  }
});

test("a truss's corners are exact: the square at ±w/2, the triangle on its circle with sides w", () => {
  const sq = truss(TRUSSES[0]!);
  assert.deepEqual(sq.nodes.slice(0, 4).map((n) => [n.x, n.z]), [[0.5, 0.5], [-0.5, 0.5], [-0.5, -0.5], [0.5, -0.5]]);
  const tri = truss(TRUSSES[1]!);
  const r = 0.8 / Math.sqrt(3);
  for (const n of tri.nodes.slice(0, 3)) assert.ok(Math.abs(Math.sqrt(n.x * n.x + n.z * n.z) - r) < 1e-12);
  const [a, b, c] = tri.nodes as [PartNodeT, PartNodeT, PartNodeT];
  const side = (p: PartNodeT, q: PartNodeT): number => Math.sqrt((p.x - q.x) ** 2 + (p.z - q.z) ** 2);
  assert.ok(Math.abs(side(a, b) - 0.8) < 1e-12 && Math.abs(side(b, c) - 0.8) < 1e-12 && Math.abs(side(c, a) - 0.8) < 1e-12, "sides are the width");
  assert.equal(a.x, 0); // the top corner is on the z axis, at 90°
});
type PartNodeT = Part["nodes"][number];

test("every wall quad has the longitudinal and both diagonals, and ring beams are flagged", () => {
  for (const c of SWEEP) {
    const part = c.build();
    const has = (a: number, b: number): boolean => part.beams.some((m) => (m.a === a && m.b === b) || (m.a === b && m.b === a));
    const n = part.rings[0]!.nodes.length;
    const levels = part.nodes.length / n;
    for (let k = 0; k < levels - 1; k++) {
      for (let j = 0; j < n; j++) {
        const j1 = (j + 1) % n;
        assert.ok(has(k * n + j, (k + 1) * n + j), `${c.name}: longitudinal ${k},${j}`);
        assert.ok(has(k * n + j, (k + 1) * n + j1), `${c.name}: diagonal one ${k},${j}`);
        assert.ok(has(k * n + j1, (k + 1) * n + j), `${c.name}: diagonal two ${k},${j}`);
      }
    }
    const keys = part.beams.map((m) => `${Math.min(m.a, m.b)}-${Math.max(m.a, m.b)}`);
    assert.equal(new Set(keys).size, keys.length, `${c.name}: duplicate beams`);
    const ring = part.beams.filter((m) => m.ring);
    assert.ok(ring.every((m) => Math.floor(m.a / n) === Math.floor(m.b / n)), `${c.name}: a ring beam leaves its ring`);
    assert.equal(ring.length + 3 * n * (levels - 1), part.beams.length, `${c.name}: every other beam is a wall beam`);
    assert.ok(part.beams.every((m) => m.zeta === ZETA_STRUCTURE));
  }
});

test("each ring keeps its shape: the perimeter, the chords and the diameters, each once", () => {
  assert.equal(ringBracing(4).length, 6, "a square: four sides and two diagonals");
  assert.equal(ringBracing(6).length, 6 + 6 + 3);
  assert.equal(ringBracing(8).length, 8 + 8 + 4);
  assert.equal(ringBracing(12).length, 12 + 12 + 12 + 6, "from 12 nodes on, the quarter chords too");
  assert.equal(ringBracing(24).length, 24 + 24 + 24 + 12);
  for (const n of RING_COUNTS) {
    const pairs = ringBracing(n);
    const keys = new Set(pairs.map(([a, b]) => `${Math.min(a, b)}-${Math.max(a, b)}`));
    assert.equal(keys.size, pairs.length, `${n}: no pair twice`);
    for (let j = 0; j < n; j++) assert.ok(keys.has(`${Math.min(j, (j + 1) % n)}-${Math.max(j, (j + 1) % n)}`), `${n}: side ${j}`);
    for (let j = 0; j < n / 2; j++) assert.ok(keys.has(`${j}-${j + n / 2}`), `${n}: diameter ${j}`);
  }
});

test("segments, seams, rings and patches of every tank, and of a truss", () => {
  for (const p of TANKS) {
    const part = tank(p);
    const n = p.ringNodes;
    const bays = p.rings - 1;
    assert.equal(part.segments.length, bays);
    assert.ok(part.segments.every((s) => !s.neverLive));
    assert.equal(part.seams.length, bays - 1);
    for (const [i, s] of part.seams.entries()) {
      assert.equal(s.segmentA, i);
      assert.equal(s.segmentB, i + 1);
      assert.ok(Math.abs(s.area / (n * p.wallArea) - 1) < 1e-12, "axial area is the sum of the longitudinals");
      assert.ok(Math.abs(s.bending / (n * p.wallArea * p.radius * p.radius) - 1) < 1e-12, "bending is Σ A · d² about the axis");
    }
    assert.deepEqual(part.rings.map((r) => r.id), ["bottom", "top"]);
    assert.deepEqual([...part.rings[0]!.axis], [0, -1, 0]);
    assert.deepEqual([...part.rings[1]!.axis], [0, 1, 0]);
    assert.ok(Math.abs(part.rings[1]!.centre[1] - p.length) < 1e-12);
    assert.ok(Math.abs(part.nodes[(p.rings - 1) * n]!.y - p.length) < 1e-12);
    for (const r of part.rings) {
      assert.equal(r.nodes.length, n);
      assert.equal(r.radius, p.radius);
    }
    assert.equal(part.patches.length, n * bays);
    assert.ok(part.nodes.every((v) => v.segment >= 0 && v.segment < bays));
    assert.ok(part.capsule !== null && part.capsule.radius === p.radius);
  }
  const t = truss(TRUSSES[0]!);
  assert.equal(t.segments.length, 4);
  assert.equal(t.seams.length, 3);
  assert.ok(Math.abs(t.seams[0]!.area - 4 * 3e-4) < 1e-15);
  assert.ok(Math.abs(t.seams[0]!.bending - 4 * 3e-4 * 0.5) < 1e-15, "four rails at d² = 0.5 about the axis");
  assert.equal(t.patches.length, 16);
  assert.equal(t.rings[0]!.nodes.length, 4);
  assert.equal(truss(TRUSSES[1]!).rings[0]!.nodes.length, 3);
});

test("panels: each wall quad has four tetrahedra, none flat, made of nodes of its bay", () => {
  for (const c of SWEEP) {
    const part = c.build();
    const n = part.rings[0]!.nodes.length;
    const bays = part.nodes.length / n - 1;
    assert.equal(part.panels.length, n * bays, `${c.name}: one panel on every wall quad`);
    for (const [q, cell] of part.panels.entries()) {
      const b = Math.floor(q / n);
      assert.equal(cell.tets.length, 4, `${c.name}: four tetrahedra in a panel`);
      for (const tet of cell.tets) {
        for (const i of tet) assert.ok(Math.floor(i / n) === b || Math.floor(i / n) === b + 1, `${c.name}: a tetrahedron node outside its bay`);
        const [p0, p1, p2, p3] = tet.map((i) => part.nodes[i]!) as [PartNodeT, PartNodeT, PartNodeT, PartNodeT];
        const e1 = [p1.x - p0.x, p1.y - p0.y, p1.z - p0.z] as const;
        const e2 = [p2.x - p0.x, p2.y - p0.y, p2.z - p0.z] as const;
        const e3 = [p3.x - p0.x, p3.y - p0.y, p3.z - p0.z] as const;
        const det = e1[0] * (e2[1] * e3[2] - e2[2] * e3[1]) + e1[1] * (e2[2] * e3[0] - e2[0] * e3[2]) + e1[2] * (e2[0] * e3[1] - e2[1] * e3[0]);
        const len = (v: readonly [number, number, number]): number => Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        assert.ok(Math.abs(det) > 1e-6 * len(e1) * len(e2) * len(e3), `${c.name}: a flat tetrahedron`);
      }
    }
  }
});

test("a stated mass below the members' own mass is refused, and bad parameters are refused with a reason", () => {
  const base = TANKS[0]!;
  assert.throws(() => tank({ ...base, mass: 10 }), /below the members' own mass/);
  assert.throws(() => tank({ ...base, ringNodes: 5 }), /ring has 5 nodes/);
  assert.throws(() => tank({ ...base, ringNodes: 32 }), /ring has 32 nodes/);
  assert.throws(() => tank({ ...base, rings: 1 }), /rings/);
  assert.throws(() => tank({ ...base, radius: 0 }), /radius/);
  assert.throws(() => tank({ ...base, length: -1 }), /length/);
  assert.throws(() => tank({ ...base, wallArea: 0 }), /area/);
  assert.throws(() => tank({ ...base, mass: Number.NaN }), /mass/);
  const t = TRUSSES[0]!;
  assert.throws(() => truss({ ...t, mass: 1 }), /below the members' own mass/);
  assert.throws(() => truss({ ...t, bays: 0 }), /bays/);
  assert.throws(() => truss({ ...t, section: "hex" as "square" }), /section/);
  assert.throws(() => truss({ ...t, width: 0 }), /width/);
  // No stated mass: exactly the members' own mass.
  const { mass: _unused, ...rest } = t;
  const bare = truss(rest);
  let own = 0;
  for (const b of bare.beams) {
    const p = bare.nodes[b.a]!;
    const q = bare.nodes[b.b]!;
    own += MATERIALS[b.material].density * b.area * Math.sqrt((q.x - p.x) ** 2 + (q.y - p.y) ** 2 + (q.z - p.z) ** 2);
  }
  assert.ok(Math.abs(partMass(bare) / own - 1) < 1e-12);
});

test("the stated mass is each member's own mass at its ends plus an even share of the rest", () => {
  const p = TANKS[0]!;
  const part = tank(p);
  const bare = tank(p);
  distributeMass(bare, null);
  const extra = part.nodes.map((n, i) => n.mass - bare.nodes[i]!.mass);
  for (let i = 0; i < extra.length; i++) assert.ok(Math.abs(extra[i]! / extra[0]! - 1) < 1e-9, `node ${i} gets the same share`);
});

// ---------------------------------------------------------------------------------------------
// A tank squeezed across a diameter springs back

interface Squeeze {
  deflection: number;
  residual: number;
  failures: number;
}

/** Floats the part (gravity cancelled), squeezes one ring across a diameter, lets go, and measures. */
function squeezeRun(core: Core, part: Part, n: number, ring: number, force: number): Squeeze {
  const world = new World(core);
  const build = new CommandBatch();
  compilePart(build, part);
  assert.equal(world.apply(build).rejected, 0);
  const a = ring * n;
  const b = ring * n + n / 2;
  const gap = (): number => {
    const p = world.positions();
    return Math.sqrt((p[b * 3]! - p[a * 3]!) ** 2 + (p[b * 3 + 1]! - p[a * 3 + 1]!) ** 2 + (p[b * 3 + 2]! - p[a * 3 + 2]!) ** 2);
  };
  const d0 = gap();
  let failures = 0;
  const run = (steps: number, f: (s: number) => number): number[] => {
    const seen: number[] = [];
    for (let s = 0; s < steps; s++) {
      const cmds = new CommandBatch();
      for (const [i, node] of part.nodes.entries()) cmds.addForce(i, 0, node.mass * GRAVITY, 0);
      const squeeze = f(s);
      // Inward along the line between the two nodes, re-aimed each step. A fixed direction would act
      // like a compressed rod on a floating body and slowly turn it (ER 004).
      const q = world.positions();
      const dx = q[b * 3]! - q[a * 3]!;
      const dy = q[b * 3 + 1]! - q[a * 3 + 1]!;
      const dz = q[b * 3 + 2]! - q[a * 3 + 2]!;
      const len = Math.sqrt(dx * dx + dy * dy + dz * dz);
      const k = squeeze / len;
      cmds.addForce(a, dx * k, dy * k, dz * k);
      cmds.addForce(b, -dx * k, -dy * k, -dz * k);
      assert.equal(world.apply(cmds).rejected, 0);
      world.step();
      failures += world.events().filter((e) => e.kind === EVENT_YIELD || e.kind === EVENT_BREAK).length;
      seen.push(gap());
    }
    return seen;
  };
  const mean = (xs: number[]): number => xs.reduce((x, y) => x + y, 0) / xs.length;
  // Smooth in and out, so the load is gentle and the held and released readings are steady.
  const smooth = (s: number, steps: number): number => {
    const u = (s + 1) / steps;
    return u * u * (3 - 2 * u);
  };
  run(600, (s) => force * smooth(s, 600)); // ramp up
  const held = mean(run(600, () => force).slice(-200));
  run(600, (s) => force * (1 - smooth(s, 600))); // ramp down
  const end = mean(run(600, () => 0).slice(-200));
  world.free();
  return { deflection: d0 - held, residual: Math.abs(end - d0), failures };
}

for (const idx of [0, 2, 4, 5]) {
  const p = TANKS[idx]!;
  test(`a ${p.ringNodes}-node ${p.material} tank squeezed across a diameter springs back`, async () => {
    const core = await loadCore();
    const part = tank(p);
    const force = 0.05 * member(p.material, p.wallArea).yieldForce!;
    const mid = Math.floor(p.rings / 2);
    const r = squeezeRun(core, part, p.ringNodes, mid, force);
    assert.equal(r.failures, 0, "elastic: nothing yields");
    assert.ok(r.deflection > 0, "the ring is pushed in");
    assert.ok(r.deflection < 0.02 * p.radius, `squeezed ${r.deflection} m of a ${p.radius} m ring: a mechanism`);
    assert.ok(r.residual < 0.05 * r.deflection + 1e-9, `${r.residual} m left of ${r.deflection} m`);

    // The test can see a mechanism: with the ring members taken out the same push squeezes much further.
    const loose: Part = { ...part, beams: part.beams.filter((m) => !m.ring) };
    const l = squeezeRun(core, loose, p.ringNodes, mid, force);
    assert.ok(Math.abs(l.deflection) > 5 * r.deflection, `without ring bracing it squeezed ${l.deflection} m against ${r.deflection} m`);
  });
}

// ---------------------------------------------------------------------------------------------
// Leg, engine and pod generators (phase 4.3)

/** The crush bay's beams in a leg: after 30 ring beams and the upper leg's 12, twelve of them. */
const CRUSH_BEAMS: readonly [number, number] = [42, 54];
const inCrush = (beam: number): boolean => beam >= CRUSH_BEAMS[0] && beam < CRUSH_BEAMS[1];

interface Press {
  /** Yield and break events in order: `[kind, beam, step, force over the crush force]`. */
  events: [number, number, number, number][];
  /** How far the crush bay has shortened after each step, in metres. */
  crush: number[];
  /** How many of the crush bay's beams are broken after each step. */
  broken: number[];
}

/**
 * A headless press: the mount pinned, gravity cancelled, a force along the leg's axis on the four foot
 * nodes, rising linearly to `peak` times the crush force over `ramp` steps and then held for `hold`
 * more. The foot is held square to the axis by stiff guides that give no push along it, the way a
 * press's platen would; without them a leg pushed on its end at 45 kN folds sideways at its soft bays
 * (ER 004).
 */
function pressLeg(core: Core, p: LegParams, peak: number, ramp: number, hold: number): Press {
  const part = leg(p);
  const world = new World(core);
  const build = new CommandBatch();
  compilePart(build, part);
  for (let i = 0; i < 4; i++) build.setInvMass(i, 0);
  assert.equal(world.apply(build).rejected, 0);
  const drop = Math.sqrt(p.length * p.length - p.footOffset * p.footOffset);
  const u = [p.footOffset / p.length, -drop / p.length];
  const across = [drop / p.length, p.footOffset / p.length, 0];
  const guides = new CommandBatch();
  const at = world.positions();
  const reach = 50;
  const stiffness = 3e6;
  let next = part.nodes.length;
  for (let i = 16; i < 20; i++) {
    for (const dir of [across, [0, 0, 1]]) {
      guides.addNode(at[i * 3]! + reach * dir[0]!, at[i * 3 + 1]! + reach * dir[1]!, at[i * 3 + 2]! + reach * dir[2]!, 0, 0.01);
      guides.addBeam(i, next++, { rest: reach, compliance: 1 / stiffness, damping: 0, yieldStrain: 1e9, breakStrain: 1e9 });
    }
  }
  assert.equal(world.apply(guides).rejected, 0);
  const centre = (pos: Float64Array, k: number): number[] => [0, 1, 2].map((a) => (pos[4 * k * 3 + a]! + pos[(4 * k + 1) * 3 + a]! + pos[(4 * k + 2) * 3 + a]! + pos[(4 * k + 3) * 3 + a]!) / 4);
  const gap = (pos: Float64Array): number => {
    const a = centre(pos, 1);
    const b = centre(pos, 2);
    return Math.sqrt((a[0]! - b[0]!) ** 2 + (a[1]! - b[1]!) ** 2 + (a[2]! - b[2]!) ** 2);
  };
  const gap0 = gap(world.positions());
  const out: Press = { events: [], crush: [], broken: [] };
  for (let s = 0; s < ramp + hold; s++) {
    const f = p.crushForce * (s < ramp ? (peak * (s + 1)) / ramp : peak);
    const cmds = new CommandBatch();
    for (const [i, n] of part.nodes.entries()) cmds.addForce(i, 0, n.mass * GRAVITY, 0);
    for (let i = 16; i < 20; i++) cmds.addForce(i, (-u[0]! * f) / 4, (-u[1]! * f) / 4, 0);
    assert.equal(world.apply(cmds).rejected, 0);
    world.step();
    for (const e of world.events()) if (e.kind === EVENT_YIELD || e.kind === EVENT_BREAK) out.events.push([e.kind, e.index, s, f / p.crushForce]);
    out.crush.push(gap0 - gap(world.positions()));
    out.broken.push(Array.from(world.snapshot().beams.flags.slice(CRUSH_BEAMS[0], CRUSH_BEAMS[1])).filter((fl) => (fl & BEAM_BROKEN) !== 0).length);
  }
  world.free();
  return out;
}

test("a leg pressed along its axis stays elastic below the crush force", async () => {
  const core = await loadCore();
  for (const p of LEGS) {
    const r = pressLeg(core, p, 0.9, 1600, 200);
    assert.deepEqual(r.events, [], "no yield and no break below the crush force");
    const last = r.crush[r.crush.length - 1]!;
    // The crush bay is five strokes long, so an elastic squeeze is a few per cent of that.
    assert.ok(last > 0 && last < 0.03 * 5 * p.crushStroke, `crush bay shortened ${last} m, an elastic amount`);
  }
});

test("a leg pressed past the crush force gives way in the crush bay first and nowhere else, then the crush bay breaks", async () => {
  const core = await loadCore();
  for (const p of LEGS) {
    const r = pressLeg(core, p, 1.1, 1800, 1200);
    const yields = r.events.filter((e) => e[0] === EVENT_YIELD);
    const breaks = r.events.filter((e) => e[0] === EVENT_BREAK);
    assert.ok(yields.length > 0 && breaks.length > 0, "the crush bay yields and then breaks");
    const firstYield = yields[0]!;
    const firstBreak = breaks[0]!;
    assert.ok(inCrush(firstYield[1]), `first yield is beam ${firstYield[1]}, outside the crush bay`);
    assert.ok(firstYield[3] >= 0.9 && firstYield[3] <= 1.05, `first yield at ${firstYield[3]} of the crush force`);
    assert.ok(inCrush(firstBreak[1]), `first break is beam ${firstBreak[1]}, outside the crush bay`);
    // The upper leg, the spring, the foot and every ring outlast the crush bay: nothing outside it
    // yields or breaks until it has broken.
    for (const e of r.events) if (e[2] <= firstBreak[2]) assert.ok(inCrush(e[1]), `beam ${e[1]} outside the crush bay had an event (${e[0]}) at step ${e[2]}`);
    // It crushes a good part of its stroke before it breaks, and the bay is gone after.
    assert.ok(r.crush[firstBreak[2]]! >= 0.25 * p.crushStroke, `broke after ${r.crush[firstBreak[2]]} m of a ${p.crushStroke} m stroke`);
    assert.ok(r.broken[r.broken.length - 1]! >= 4, "the bay is broken through");
  }
});

/** Applies `apply`'s forces for 600 steps, ramped in smoothly, and reports yields and breaks and the final speed. */
function holdThrust(core: Core, part: Part, apply: (cmds: CommandBatch, f: number) => void, peak: number, pinned: number[] = []): { failures: number; fastest: number } {
  const world = new World(core);
  const build = new CommandBatch();
  compilePart(build, part);
  for (const i of pinned) build.setInvMass(i, 0);
  assert.equal(world.apply(build).rejected, 0);
  let failures = 0;
  let prev = world.positions();
  let fastest = 0;
  const ramp = 90;
  for (let s = 0; s < 600; s++) {
    const u = Math.min(1, (s + 1) / ramp);
    const cmds = new CommandBatch();
    apply(cmds, peak * u * u * (3 - 2 * u));
    assert.equal(world.apply(cmds).rejected, 0);
    world.step();
    failures += world.events().filter((e) => e.kind === EVENT_YIELD || e.kind === EVENT_BREAK).length;
    const now = world.positions();
    if (s === 599) {
      for (let i = 0; i < now.length; i += 3) fastest = Math.max(fastest, Math.sqrt((now[i]! - prev[i]!) ** 2 + (now[i + 1]! - prev[i + 1]!) ** 2 + (now[i + 2]! - prev[i + 2]!) ** 2) / STEP);
    }
    prev = now;
  }
  world.free();
  return { failures, fastest };
}

test("the engine carries the hopper's full thrust through its lattice with no yield", async () => {
  const core = await loadCore();
  const p = ENGINES[0]!;
  const part = engine(p);
  const n = p.ringNodes;
  assert.deepEqual(part.thrusters, [{ tail: 2 * n, head: 2 * n + 1, maxThrust: p.maxThrust }]);
  // The thruster pushes half of the thrust on the head and half on the tail; the tank above holds the
  // top ring, which is pinned. (Balancing the thrust with forces on the top ring instead leaves a free
  // body that tumbles under fixed-direction loads: ER 004.) The engine's own weight is cancelled.
  const r = holdThrust(
    core,
    part,
    (cmds, f) => {
      for (const [i, node] of part.nodes.entries()) cmds.addForce(i, 0, node.mass * GRAVITY, 0);
      cmds.addForce(2 * n, 0, f / 2, 0);
      cmds.addForce(2 * n + 1, 0, f / 2, 0);
    },
    p.maxThrust,
    Array.from({ length: n }, (_, j) => n + j),
  );
  assert.equal(r.failures, 0, "no yield or break at full thrust");
  assert.ok(r.fastest < 1e-3, `still moving at ${r.fastest} m/s`);
});

test("the pod carries the hopper's full thrust through its lattice with no yield", async () => {
  const core = await loadCore();
  const p = PODS[0]!;
  const part = pod(p);
  // The whole 80 kN pushes up on the bottom ring and the pod accelerates freely at T / m: a harder
  // load than it will ever carry, since the pod rides on the rest of the craft.
  const r = holdThrust(
    core,
    part,
    (cmds, f) => {
      for (const [i, node] of part.nodes.entries()) cmds.addForce(i, 0, node.mass * GRAVITY, 0);
      for (let j = 0; j < p.ringNodes; j++) cmds.addForce(j, 0, f / p.ringNodes, 0);
    },
    ENGINES[0]!.maxThrust,
  );
  assert.equal(r.failures, 0, "no yield or break at full thrust");
});

test("an engine's and a pod's panels: one per wall quad, the pod's nose one per facet, none flat", () => {
  for (const [part, n, nose] of [[engine(ENGINES[0]!), ENGINES[0]!.ringNodes, 0], [pod(PODS[0]!), PODS[0]!.ringNodes, PODS[0]!.ringNodes]] as const) {
    assert.equal(part.panels.length, n + nose);
    for (const [q, cell] of part.panels.entries()) {
      const tets = q < n ? 4 : 3;
      assert.equal(cell.tets.length, tets, `panel ${q} has ${tets} tetrahedra`);
      for (const tet of cell.tets) {
        const [p0, p1, p2, p3] = [part.nodes[tet[0]]!, part.nodes[tet[1]]!, part.nodes[tet[2]]!, part.nodes[tet[3]]!];
        const e1 = [p1.x - p0.x, p1.y - p0.y, p1.z - p0.z] as const;
        const e2 = [p2.x - p0.x, p2.y - p0.y, p2.z - p0.z] as const;
        const e3 = [p3.x - p0.x, p3.y - p0.y, p3.z - p0.z] as const;
        const det = e1[0] * (e2[1] * e3[2] - e2[2] * e3[1]) + e1[1] * (e2[2] * e3[0] - e2[0] * e3[2]) + e1[2] * (e2[0] * e3[1] - e2[1] * e3[0]);
        const len = (v: readonly [number, number, number]): number => Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        assert.ok(Math.abs(det) > 1e-6 * len(e1) * len(e2) * len(e3), `panel ${q}: a flat tetrahedron`);
      }
    }
  }
});

test("engine, pod and leg: shape, attachments and never-live flags", () => {
  const e = engine(ENGINES[0]!);
  assert.equal(e.nodes.length, 18, "two rings of 8, a tail and a head");
  assert.deepEqual(e.rings.map((r) => r.id), ["top"]);
  assert.deepEqual([...e.rings[0]!.axis], [0, 1, 0]);
  assert.ok(e.segments.every((s) => s.neverLive));
  assert.ok(e.nodes[16]!.x === 0 && e.nodes[16]!.y === 0 && e.nodes[17]!.x === 0 && e.nodes[17]!.y === 1, "tail on the axis at the nozzle, head on the axis at the top");
  const q = pod(PODS[0]!);
  assert.equal(q.nodes.length, 17, "two rings of 8 and the apex");
  assert.deepEqual(q.rings.map((r) => r.id), ["bottom"]);
  assert.deepEqual([...q.rings[0]!.axis], [0, -1, 0]);
  assert.ok(q.segments.every((s) => s.neverLive));
  assert.ok(q.nodes[16]!.x === 0 && q.nodes[16]!.y > PODS[0]!.height, "the apex is above the top ring");
  const l = leg(LEG);
  assert.equal(l.nodes.length, 20);
  assert.equal(l.segments.length, 1);
  assert.ok(!l.segments[0]!.neverLive);
  assert.deepEqual(l.patches.map((m) => m.id), ["mount"]);
  assert.equal(l.rings.length, 0);
  const foot = [16, 17, 18, 19].map((i) => l.nodes[i]!);
  const mid = (k: "x" | "y" | "z"): number => foot.reduce((s, v) => s + v[k], 0) / 4;
  assert.ok(Math.abs(mid("x") - LEG.footOffset) < 1e-12 && Math.abs(Math.sqrt(mid("x") ** 2 + mid("y") ** 2) - LEG.length) < 1e-12, "the foot is `length` from the mount's centre");
  assert.ok(Math.abs(Math.max(...foot.map((v) => v.x)) - Math.min(...foot.map((v) => v.x)) - LEG.footSize) < 1e-12, "the foot's square");
});

test("the new generators refuse what cannot work, with the reason", () => {
  const e = ENGINES[0]!;
  assert.throws(() => engine({ ...e, ringNodes: 5 }), /ring has 5 nodes/);
  assert.throws(() => engine({ ...e, maxThrust: 0 }), /thrust/);
  assert.throws(() => engine({ ...e, height: 0 }), /height/);
  assert.throws(() => engine({ ...e, mass: 10 }), /below the members' own mass/);
  const q = PODS[0]!;
  assert.throws(() => pod({ ...q, ringNodes: 10 }), /ring has 10 nodes/);
  assert.throws(() => pod({ ...q, apexRise: 0 }), /apex rise/);
  assert.throws(() => pod({ ...q, topRadius: -1 }), /top radius/);
  assert.throws(() => pod({ ...q, mass: 10 }), /below the members' own mass/);
  assert.throws(() => leg({ ...LEG, length: 0 }), /length/);
  assert.throws(() => leg({ ...LEG, footOffset: 3.2 }), /foot offset/);
  assert.throws(() => leg({ ...LEG, footOffset: -1 }), /foot offset/);
  assert.throws(() => leg({ ...LEG, crushStroke: 1 }), /upper leg would be/);
  assert.throws(() => leg({ ...LEG, sprungStiffness: 1e4 }), /would yield/);
  assert.throws(() => leg({ ...LEG, memberArea: 2e-5 }), /give the members an area of at least/);
  assert.throws(() => leg({ ...LEG, mass: 1 }), /below the members' own mass/);
});
