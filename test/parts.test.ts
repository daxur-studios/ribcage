import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";
import { CommandBatch } from "../src/commands.ts";
import { Core, EVENT_BREAK, EVENT_YIELD } from "../src/core.ts";
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
