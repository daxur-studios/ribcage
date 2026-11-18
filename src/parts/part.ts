/**
 * Purpose: what a part is (nodes, beams, panels, rings, seams, segments, thrusters), how it becomes
 * commands for the core, its mass, and a budget check that says how many substeps the core will
 * pick for it and how uneven its node masses are.
 * Value: every generator (truss, tank, leg, engine, pod) fills this one shape, the assembler welds
 * these shapes, and the craft file saves them; the budget check lets a generator be tested for
 * "fits the substep budget" without a core, and the same sum is checked against the core in tests.
 *
 * Only `+ − × ÷` and `sqrt` appear in the maths here (generator rule: the same inputs give the
 * same bits on every engine). Beam numbers come from one resolver, `resolveBeam`, used by both
 * `compilePart` and `partBudget`, so the substep count reported here is the one the core computes
 * from the commands.
 */

import { STEP_SECONDS } from '../design.ts';
import type { CommandBatch } from '../commands.ts';
import type { SkinCell } from '../skin.ts';
import { dampingFor, member, type MaterialId } from './material.ts';

/** Damping ratio for ordinary structure (ER 004: the spec's minimum damping on every beam). */
export const ZETA_STRUCTURE = 0.05;
/** Damping ratio for the sprung bay of a leg (ER 004: no bounce). */
export const ZETA_SPRUNG = 0.6;

/** The core's substep rule, `core/src/solver.rs`: the smallest `n` in `MIN..=MAX` with `ω_max · h ≤ C`. */
export const SUBSTEP_MIN = 4;
export const SUBSTEP_MAX = 32;
export const SUBSTEP_C = 1.0;

/** ER 004's budget: a default part, and the hopper, step at no more than this many substeps. */
export const SUBSTEP_BUDGET = 24;
/** ER 004's budget: heaviest to lightest node across a craft. */
export const NODE_MASS_RATIO_BUDGET = 10;

export type V3 = readonly [number, number, number];

export interface PartNode {
  x: number;
  y: number;
  z: number;
  /** Mass in kilograms, members' share plus contents. Always positive: a part has no pinned nodes. */
  mass: number;
  /** Collision radius in metres. */
  radius: number;
  /** Index into `Part.segments`. */
  segment: number;
}

/** A member's numbers, replaced where a beam is not simply "this material at this area" (leg bays). */
export interface BeamOverride {
  /** Axial stiffness in newtons, replacing `s · E · A` (a soft sprung bay). */
  ea?: number;
  yieldStrain?: number;
  breakStrain?: number;
}

export interface PartBeam {
  a: number;
  b: number;
  material: MaterialId;
  /** Cross-section area in square metres. */
  area: number;
  /** Damping ratio ζ; the core's damping comes from it, the beam's stiffness and its end masses. */
  zeta: number;
  /** True for a ring's own perimeter and chords: the assembler drops the child's, keeps the parent's. */
  ring: boolean;
  override?: BeamOverride;
}

/** A ring a part can be joined at: its nodes in order round the axis, its centre and its outward axis. */
export interface AttachmentRing {
  id: string;
  nodes: readonly number[];
  centre: V3;
  /** Unit vector pointing out of the part through the ring. */
  axis: V3;
  radius: number;
}

/** A mount patch: two adjacent nodes on each of two adjacent rings, where a leg is welded on. */
export interface MountPatch {
  id: string;
  /** Four nodes: two on one ring, then the two beside them on the next ring. */
  nodes: readonly [number, number, number, number];
}

/**
 * The cut between two segments of one part, with the section properties milestone 5 will need:
 * axial area `Σ A` of the members crossing the cut, and bending stiffness `Σ A · d²` with `d` each
 * member's distance from the cut's centroid.
 */
export interface Seam {
  segmentA: number;
  segmentB: number;
  area: number;
  bending: number;
}

export interface Segment {
  /** True for segments that must never be simulated as live lattices once rigid bodies exist (engine, pod). */
  neverLive: boolean;
}

/** One coarse collision capsule (emitted, unused until milestone 8). */
export interface Capsule {
  a: V3;
  b: V3;
  radius: number;
}

/** An engine's thrust: `tail` and `head` are node indices, the push acts along tail to head. */
export interface PartThruster {
  tail: number;
  head: number;
  /** Maximum thrust in newtons. */
  maxThrust: number;
}

export interface Part {
  /** Part type name, for example `tank`. */
  type: string;
  nodes: PartNode[];
  beams: PartBeam[];
  /** Skin panels, in the 3.8 shape: the tetrahedra of the lattice cell each panel binds to. */
  panels: SkinCell[];
  rings: AttachmentRing[];
  patches: MountPatch[];
  seams: Seam[];
  segments: Segment[];
  capsule: Capsule | null;
  thrusters: PartThruster[];
}

/** A part with nothing in it, for a generator to fill. */
export function emptyPart(type: string): Part {
  return { type, nodes: [], beams: [], panels: [], rings: [], patches: [], seams: [], segments: [], capsule: null, thrusters: [] };
}

/** Everything the core is told about one beam. */
export interface ResolvedBeam {
  rest: number;
  compliance: number;
  damping: number;
  yieldStrain: number;
  breakStrain: number;
  /** Axial stiffness used, in newtons. */
  ea: number;
}

/**
 * The beam's numbers, from its two end nodes and its member. Used by `compilePart` and by
 * `partBudget` alike. `sqrt` is exactly rounded everywhere; `Math.hypot` is not, so it is not used.
 */
export function resolveBeam(part: Part, beam: PartBeam): ResolvedBeam {
  const p = part.nodes[beam.a];
  const q = part.nodes[beam.b];
  if (p === undefined || q === undefined) throw new Error(`a ${part.type} beam joins nodes ${beam.a} and ${beam.b}, which do not both exist`);
  if (beam.a === beam.b) throw new Error(`a ${part.type} beam joins node ${beam.a} to itself`);
  const rest = Math.sqrt((q.x - p.x) * (q.x - p.x) + (q.y - p.y) * (q.y - p.y) + (q.z - p.z) * (q.z - p.z));
  if (!(rest > 0)) throw new Error(`a ${part.type} beam between nodes ${beam.a} and ${beam.b} has no length`);
  const m = member(beam.material, beam.area);
  const o = beam.override;
  const ea = o?.ea ?? m.ea;
  if (!(ea > 0) || !Number.isFinite(ea)) throw new Error(`a ${part.type} beam's stiffness must be a positive number, not ${ea}`);
  return {
    rest,
    compliance: rest / ea,
    damping: dampingFor(ea, rest, p.mass, q.mass, beam.zeta),
    yieldStrain: o?.yieldStrain ?? m.yieldStrain,
    breakStrain: o?.breakStrain ?? m.breakStrain,
    ea,
  };
}

/**
 * Appends the part to `batch`: its nodes in order (so node `i` of the part is node `i` of an empty
 * world), then its beams in order, then its thrusters in order.
 */
export function compilePart(batch: CommandBatch, part: Part): void {
  for (const [i, n] of part.nodes.entries()) {
    if (!(n.mass > 0) || !Number.isFinite(n.mass)) throw new Error(`a ${part.type} node ${i} has mass ${n.mass}; a part's nodes all have positive mass`);
    batch.addNode(n.x, n.y, n.z, 1 / n.mass, n.radius);
  }
  for (const b of part.beams) {
    const r = resolveBeam(part, b);
    batch.addBeam(b.a, b.b, { rest: r.rest, compliance: r.compliance, damping: r.damping, yieldStrain: r.yieldStrain, breakStrain: r.breakStrain });
  }
  // Thrusters last, so the nodes and beams keep their indices; thruster `i` of the part is
  // thruster `i` of an empty world, throttle zero until a command sets it.
  for (const t of part.thrusters) batch.addThruster(t.tail, t.head, t.maxThrust);
}

/** Total mass in kilograms: the node masses added in index order, so the same part always gives the same sum. */
export function partMass(part: Part): number {
  let total = 0;
  for (const n of part.nodes) total += n.mass;
  return total;
}

/**
 * The core's substep rule in TypeScript, from the largest `ω² = (w_a + w_b) / compliance` over the
 * beams. A beam between two pinned nodes (`w = 0`) asks for nothing; a compliance of zero asks for
 * the maximum. The same additions, divisions and comparisons, in the same order, as the core.
 */
export function substepCount(omega2Max: number): number {
  const omegaMax = Math.sqrt(omega2Max);
  let n = SUBSTEP_MIN;
  while (n < SUBSTEP_MAX && omegaMax * (STEP_SECONDS / n) > SUBSTEP_C) n++;
  return n;
}

export interface PartBudget {
  /** Substeps the core will pick for this part on its own. */
  substeps: number;
  /** The largest `ω` of any beam, in radians per second. */
  omegaMax: number;
  minNodeMass: number;
  maxNodeMass: number;
  /** Heaviest to lightest node; 1 for a part with one node mass, 0 for an empty part. */
  massRatio: number;
  withinBudget: boolean;
  /** One sentence for each budget the part misses; empty when `withinBudget`. */
  problems: string[];
}

/**
 * What the core will do with this part: the substep count its rule picks (the part as one island,
 * which a connected part is) and the ratio of its heaviest to its lightest node. Compare with
 * `SUBSTEP_BUDGET` and `NODE_MASS_RATIO_BUDGET`.
 */
export function partBudget(part: Part): PartBudget {
  let omega2Max = 0;
  let forceMax = false;
  for (const b of part.beams) {
    const p = part.nodes[b.a];
    const q = part.nodes[b.b];
    if (p === undefined || q === undefined) throw new Error(`a ${part.type} beam joins nodes ${b.a} and ${b.b}, which do not both exist`);
    const w = 1 / p.mass + 1 / q.mass;
    if (w === 0) continue;
    const c = resolveBeam(part, b).compliance;
    if (c === 0) {
      forceMax = true;
      continue;
    }
    const o2 = w / c;
    if (o2 > omega2Max) omega2Max = o2;
  }
  const substeps = forceMax ? SUBSTEP_MAX : substepCount(omega2Max);
  let minNodeMass = Infinity;
  let maxNodeMass = 0;
  for (const n of part.nodes) {
    if (n.mass < minNodeMass) minNodeMass = n.mass;
    if (n.mass > maxNodeMass) maxNodeMass = n.mass;
  }
  if (part.nodes.length === 0) minNodeMass = 0;
  const massRatio = part.nodes.length === 0 ? 0 : maxNodeMass / minNodeMass;
  const problems: string[] = [];
  if (substeps > SUBSTEP_BUDGET) problems.push(`the core would use ${substeps} substeps, over the budget of ${SUBSTEP_BUDGET}`);
  if (massRatio > NODE_MASS_RATIO_BUDGET) problems.push(`the heaviest node is ${massRatio} times the lightest, over the budget of ${NODE_MASS_RATIO_BUDGET}`);
  return {
    substeps,
    omegaMax: Math.sqrt(omega2Max),
    minNodeMass,
    maxNodeMass,
    massRatio,
    withinBudget: problems.length === 0,
    problems,
  };
}
