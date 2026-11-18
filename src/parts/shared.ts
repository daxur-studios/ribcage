/**
 * Purpose: what the truss and tank generators have in common: exact ring points without sine or
 * cosine, the mass rule (members' own mass to their end nodes, the rest of the stated mass spread
 * evenly), and `buildTube`, which turns a cross-section and a number of rings into the nodes, beams,
 * panels, attachment rings, mount patches, seams and segments of a braced tube.
 * Value: the two generators differ only in their cross-section, their ring bracing and their
 * parameters, so the bracing rules (both diagonals in every wall quad, ER 001) and the mass rule are
 * stated and tested once. Later generators (leg, engine, pod) reuse the ring table and the mass rule.
 *
 * Only `+ − × ÷` and `sqrt` appear in the maths here (generator rule: the same inputs give the same
 * bits on every engine).
 */

import type { SkinCell } from '../skin.ts';
import { MATERIALS, type MaterialId } from './material.ts';
import { ZETA_STRUCTURE, emptyPart, partMass, type Part, type V3 } from './part.ts';

/** Ring node counts the generators accept (ER 004): their angles are multiples of 15° or 22.5°. */
export const RING_COUNTS = [4, 6, 8, 12, 16, 24] as const;
export type RingCount = (typeof RING_COUNTS)[number];

/** Collision radius of a part's node in metres, when the caller gives none. */
export const DEFAULT_NODE_RADIUS = 0.05;

/** Angle unit of the table: 7.5°, so 15° is 2 units, 22.5° is 3, and a circle is 48. */
const UNITS_PER_TURN = 48;
const UNITS_PER_QUADRANT = 12;

const SQRT2_2 = Math.sqrt(2) / 2;
const SQRT3_2 = Math.sqrt(3) / 2;
// Half-angle formulas: cos(θ/2) = sqrt((1 + cos θ) / 2), sin(θ/2) = sqrt((1 − cos θ) / 2).
const COS15 = Math.sqrt((1 + SQRT3_2) / 2);
const SIN15 = Math.sqrt((1 - SQRT3_2) / 2);
const COS22 = Math.sqrt((1 + SQRT2_2) / 2);
const SIN22 = Math.sqrt((1 - SQRT2_2) / 2);

/** (cos, sin) of the first quadrant's angles that rings use, by unit: 0°, 15°, 22.5°, 30°, 45°, 60°, 67.5°, 75°. */
const FIRST_QUADRANT: Readonly<Record<number, readonly [number, number]>> = {
  0: [1, 0],
  2: [COS15, SIN15],
  3: [COS22, SIN22],
  4: [SQRT3_2, 0.5],
  6: [SQRT2_2, SQRT2_2],
  8: [0.5, SQRT3_2],
  9: [SIN22, COS22],
  10: [SIN15, COS15],
};

/**
 * Cosine and sine of `units` steps of 7.5°, from exact values and half-angle square roots. Only
 * multiples of 15° and 22.5° are in the table; any other angle is refused. Turning by a quadrant is
 * a swap and a sign, so opposite points are exact negatives of each other.
 */
export function unitCosSin(units: number): readonly [number, number] {
  if (!Number.isInteger(units)) throw new Error(`an angle is a whole number of 7.5° units, not ${units}`);
  const u = ((units % UNITS_PER_TURN) + UNITS_PER_TURN) % UNITS_PER_TURN;
  const q = Math.floor(u / UNITS_PER_QUADRANT);
  const r = u % UNITS_PER_QUADRANT;
  const base = FIRST_QUADRANT[r];
  if (base === undefined) throw new Error(`${r * 7.5}° into a quadrant is not a multiple of 15° or 22.5°`);
  const [c, s] = base;
  // `+ 0` turns a negative zero into a zero, so equal points always have equal bits.
  if (q === 0) return [c + 0, s + 0];
  if (q === 1) return [-s + 0, c + 0];
  if (q === 2) return [-c + 0, -s + 0];
  return [s + 0, -c + 0];
}

/** Whether `n` is a ring node count the generators accept. */
export function isRingCount(n: number): n is RingCount {
  return (RING_COUNTS as readonly number[]).includes(n);
}

/**
 * Point `k` of an `n`-point ring of the given radius, in the xz plane at x = r·cos, z = r·sin, point
 * 0 on the +x axis and the points counter-clockwise seen from +y. `turn` adds a whole number of
 * 7.5° units (the root yaw and the weld clock use the same table).
 */
export function ringPoint(count: RingCount, k: number, radius: number, turn = 0): readonly [number, number] {
  const [c, s] = unitCosSin(k * (UNITS_PER_TURN / count) + turn);
  return [radius * c + 0, radius * s + 0];
}

function positive(name: string, v: number): void {
  if (!(v > 0) || !Number.isFinite(v)) throw new Error(`${name} must be a positive number, not ${v}`);
}

/** Refuses a count of bays or rings that is not a whole number of at least `min`. */
export function wholeAtLeast(name: string, v: number, min: number): void {
  if (!Number.isInteger(v) || v < min) throw new Error(`${name} must be a whole number of at least ${min}, not ${v}`);
}

export function requirePositive(name: string, v: number): void {
  positive(name, v);
}

/**
 * Sets every node's mass: each member's own mass (density × area × length) goes half to each end
 * node, and what is left of the `stated` mass is spread evenly over all nodes. `stated` of `null`
 * means the members' own mass and nothing else. A stated mass below the members' own is refused.
 * The last node takes the rounding, so the node masses added in index order give `stated` exactly.
 */
export function distributeMass(part: Part, stated: number | null): void {
  const count = part.nodes.length;
  const share: number[] = new Array<number>(count).fill(0);
  let own = 0;
  for (const b of part.beams) {
    const p = part.nodes[b.a]!;
    const q = part.nodes[b.b]!;
    const length = Math.sqrt((q.x - p.x) * (q.x - p.x) + (q.y - p.y) * (q.y - p.y) + (q.z - p.z) * (q.z - p.z));
    const m = MATERIALS[b.material].density * b.area * length;
    own += m;
    share[b.a]! += m * 0.5;
    share[b.b]! += m * 0.5;
  }
  if (stated !== null) {
    positive('the stated mass', stated);
    if (stated < own) throw new Error(`the stated mass ${stated} kg is below the members' own mass ${own} kg: the members alone weigh more`);
  }
  const contents = stated === null ? 0 : (stated - own) / count;
  for (let i = 0; i < count; i++) part.nodes[i]!.mass = share[i]! + contents;
  if (stated === null) return;
  let prefix = 0;
  for (let i = 0; i < count - 1; i++) prefix += part.nodes[i]!.mass;
  const last = stated - prefix;
  if (!(last > 0)) throw new Error(`the stated mass ${stated} kg leaves no mass for the last node`);
  part.nodes[count - 1]!.mass = last;
  if (partMass(part) !== stated) throw new Error(`the node masses cannot be made to add up to ${stated} kg exactly`);
}

export interface TubeSpec {
  /** Part type name. */
  type: string;
  /** One ring's points (x, z) in order round the axis, at least three. The tube's axis is y. */
  section: ReadonlyArray<readonly [number, number]>;
  /** Distance from the axis to the section's points, for the attachment rings and the capsule. */
  sectionRadius: number;
  /** Rings along the axis, at least two; the tube has `rings − 1` bays. */
  rings: number;
  /** Distance between two neighbouring rings. */
  spacing: number;
  /** In-ring members as pairs of section point indices: perimeter, chords, diameters (flagged as ring beams). */
  ringBeams: ReadonlyArray<readonly [number, number]>;
  material: MaterialId;
  /** Cross-section area of every member. */
  area: number;
  nodeRadius: number;
  /** Stated mass in kilograms, or `null` for the members' own mass. */
  mass: number | null;
}

/**
 * A braced tube from `section` repeated at `rings` heights from y = 0 to y = (rings − 1) · spacing.
 *
 * - Nodes: ring by ring, so node `k · n + j` is point `j` of ring `k`.
 * - Beams: each ring's own members first (ring beams), then bay by bay, for every wall quad, the
 *   longitudinal member and **both** diagonals (ER 001: one diagonal makes a twist-and-crush mechanism).
 * - Segments: one per bay; ring 0 and ring 1 are in segment 0, ring `k` is in segment `k − 1`, so a
 *   bay's top ring and the bay's own members are in the bay, and a mount patch on the lowest bay is
 *   inside one segment. The members of bay `b` (b ≥ 1) cross the seam between segments `b − 1` and `b`.
 * - Seams: for each seam the axial area `Σ A` and bending `Σ A · d²` of the longitudinals that cross
 *   it, `d` from the centroid of their section points. Diagonals carry shear, not axial or bending
 *   load, so they are left out.
 * - Panels: one per wall quad, four tetrahedra (one per corner), each made of three corners of the
 *   quad and one more node of the same ring across the section, so none is flat.
 * - Attachment rings: `bottom` (outward axis −y) and `top` (+y).
 * - Mount patches: one on every wall quad, `side-<bay>-<point>`.
 */
export function buildTube(s: TubeSpec): Part {
  const n = s.section.length;
  const part = emptyPart(s.type);
  for (let b = 0; b < s.rings - 1; b++) part.segments.push({ neverLive: false });
  const node = (k: number, j: number): number => k * n + (((j % n) + n) % n);
  for (let k = 0; k < s.rings; k++) {
    const y = k * s.spacing;
    const segment = k === 0 ? 0 : k - 1;
    for (const [x, z] of s.section) part.nodes.push({ x, y, z, mass: 0, radius: s.nodeRadius, segment });
  }
  const beam = (a: number, b: number, ring: boolean): void => {
    part.beams.push({ a, b, material: s.material, area: s.area, zeta: ZETA_STRUCTURE, ring });
  };
  for (let k = 0; k < s.rings; k++) for (const [a, b] of s.ringBeams) beam(node(k, a), node(k, b), true);
  const off = Math.max(2, n >> 1);
  for (let k = 0; k < s.rings - 1; k++) {
    const tets: SkinCell[] = [];
    for (let j = 0; j < n; j++) {
      const a = node(k, j);
      const b = node(k, j + 1);
      const c = node(k + 1, j);
      const d = node(k + 1, j + 1);
      beam(a, c, false);
      beam(a, d, false);
      beam(b, c, false);
      const xk = node(k, j + off);
      const xk1 = node(k + 1, j + off);
      tets.push({ tets: [[a, b, c, xk], [b, a, d, xk], [c, d, a, xk1], [d, c, b, xk1]] });
      part.patches.push({ id: `side-${k}-${j}`, nodes: [a, b, c, d] });
    }
    part.panels.push(...tets);
  }
  // Seams: the longitudinals of bay b cross between segments b − 1 and b.
  for (let k = 1; k < s.rings - 1; k++) {
    let cx = 0;
    let cz = 0;
    for (const [x, z] of s.section) {
      cx += x;
      cz += z;
    }
    cx /= n;
    cz /= n;
    let area = 0;
    let bending = 0;
    for (const [x, z] of s.section) {
      area += s.area;
      bending += s.area * ((x - cx) * (x - cx) + (z - cz) * (z - cz));
    }
    part.seams.push({ segmentA: k - 1, segmentB: k, area, bending });
  }
  const top = (s.rings - 1) * s.spacing;
  const ringNodes = (k: number): number[] => Array.from({ length: n }, (_, j) => node(k, j));
  const up: V3 = [0, 1, 0];
  const down: V3 = [0, -1, 0];
  part.rings.push({ id: 'bottom', nodes: ringNodes(0), centre: [0, 0, 0], axis: down, radius: s.sectionRadius });
  part.rings.push({ id: 'top', nodes: ringNodes(s.rings - 1), centre: [0, top, 0], axis: up, radius: s.sectionRadius });
  part.capsule = { a: [0, 0, 0], b: [0, top, 0], radius: s.sectionRadius };
  distributeMass(part, s.mass);
  return part;
}
