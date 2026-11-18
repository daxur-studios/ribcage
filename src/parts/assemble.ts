/**
 * Purpose: the assembler. It welds parts into one craft: a child's attachment ring (or mount patch)
 * is placed against a parent's, turned by a whole number of ring steps (the "clock", a permutation of
 * the ring's nodes, no angles), and its nodes are merged into the parent's. The output is one craft
 * in the part shape (so it compiles to commands with `compilePart` and is checked with `partBudget`
 * like any part) plus the assembly tree: each part's parent, its join and the seam between them.
 * Value: the hopper, and every later craft, is a list of parts and joins; the craft file (4.5)
 * saves exactly that list, and the same list always gives the same bits.
 *
 * The weld, from Plan 004's "Settled" section:
 * - The child is moved rigidly (a rotation and a translation) so its ring faces the parent's: axes
 *   opposed, centres equal, child node `j` on parent node `j + clock` (or `clock − j` when the two
 *   rings are numbered in opposite senses about the shared axis). A mount patch is placed the other
 *   way round: the child's outward side is the patch's outward side, and its four nodes sit on the
 *   patch's four, in order (clock 0 only).
 * - Every merged pair must be within 1 mm of each other after the move; the merged node is put
 *   exactly where the parent has it. Rings must have the same node count and radius.
 * - Masses add; the merged node keeps the parent's segment; the child's ring beams between merged
 *   nodes are dropped (the parent's carry the ring); every other child beam that touches a merged
 *   node is a seam beam.
 * - Segments of every part are kept, in craft order. A join adds a seam between the parent's segment
 *   and the child's, with the section properties of its seam beams: `Σ A cos²θ` and `Σ A cos²θ d²`,
 *   where `θ` is the beam's angle to the join's axis and `d` the merged end's distance from the
 *   centroid of the merged nodes, so a longeron counts in full and a cross brace hardly at all.
 * - The total mass is exactly the parts' total: the last node takes the rounding, as in a part.
 *
 * Only `+ − × ÷` and `sqrt` appear in the maths (see `shared.ts`); angles come from the same table as
 * the generators' rings. Nothing is changed in the parts given: the assembler works on copies, and a
 * refused assembly returns nothing at all (an `AssemblyError` with a code and a reason).
 */

import type { SkinCell, Tet } from '../skin.ts';
import {
  SUBSTEP_BUDGET,
  partBudget,
  partMass,
  type AttachmentRing,
  type Capsule,
  type Part,
  type PartBeam,
  type PartBudget,
  type PartNode,
  type PartThruster,
  type Seam,
  type Segment,
  type V3,
} from './part.ts';
import { NO_SEGMENT, SEG_NEVER_LIVE, type CommandBatch } from '../commands.ts';
import { compilePart } from './part.ts';
import { unitCosSin } from './shared.ts';

/** How far apart two nodes to be merged may be after the child is placed, in metres (Plan 004: 1 mm). */
export const WELD_TOLERANCE = 1e-3;

/**
 * The heaviest to lightest node a whole craft may have. A part keeps to 10, but a weld puts a
 * neighbour's share onto a merged node (the hopper's tank ring takes the engine's and a leg's, 18.2),
 * and the hopper stands for 20 s with no yield at that ratio. Decided in ER 004, phase 4.4.
 */
export const CRAFT_NODE_MASS_RATIO_BUDGET = 20;

/** Why an assembly is refused. */
export type AssemblyRefusal =
  | 'no-root'
  | 'duplicate-part'
  | 'unknown-part'
  | 'unknown-port'
  | 'bad-clock'
  | 'bad-part'
  | 'self-join'
  | 'attached-twice'
  | 'cycle'
  | 'disconnected'
  | 'port-used'
  | 'port-kind'
  | 'node-count'
  | 'radius'
  | 'ring-shape'
  | 'patch-shape'
  | 'patch-inward'
  | 'patch-clock'
  | 'substeps'
  | 'mass-ratio';

export class AssemblyError extends Error {
  readonly code: AssemblyRefusal;
  constructor(code: AssemblyRefusal, message: string) {
    super(message);
    this.name = 'AssemblyError';
    this.code = code;
  }
}

/** A part in an assembly, by the name the joins use. */
export interface AssemblyPart {
  id: string;
  part: Part;
}

/** One join: the child's ring or patch against the parent's. */
export interface AssemblyJoin {
  child: string;
  /** A ring id or a patch id of the child. */
  childPort: string;
  parent: string;
  /** A ring id or a patch id of the parent. */
  parentPort: string;
  /** Whole ring steps the child is turned by (rings only; a patch takes 0). */
  clock: number;
}

/** Where the root part goes: a position and a yaw in whole 15° steps (turning +x toward +z). */
export interface RootPlacement {
  x: number;
  y: number;
  z: number;
  yaw: number;
}

export interface Assembly {
  parts: readonly AssemblyPart[];
  root: string;
  joins: readonly AssemblyJoin[];
  place?: RootPlacement;
}

/** A part's place in the craft and in the assembly tree. */
export interface TreeEntry {
  id: string;
  type: string;
  /** Index of the parent's entry in `Craft.tree`, or `null` for the root. */
  parent: number | null;
  /** The join that attached it (`null` for the root). */
  join: AssemblyJoin | null;
  /** The craft node each of the part's nodes became (merged nodes point at the parent's node). */
  nodeOf: number[];
  /** Where the part's own nodes were once it was placed, before merging (the weld's evidence). */
  placed: V3[];
  /** First craft segment of this part; part segment `s` is craft segment `segmentBase + s`. */
  segmentBase: number;
  /** Index into `Craft.seams` of the seam this join made, or `null` for the root. */
  seam: number | null;
  /** Craft beam indices of the part's seam beams (the beams touching a merged node). */
  seamBeams: number[];
  /** Number of the child's ring beams dropped at the join. */
  droppedBeams: number;
  /** Number of the child's nodes merged into the parent's. */
  merged: number;
  /** The part's coarse capsule, placed in the craft (milestone 8 uses it). */
  capsule: Capsule | null;
}

export interface CraftBeam extends PartBeam {
  /** True for a beam that touches a node merged at a join. */
  seam: boolean;
  /** Index into `Craft.tree` of the part that brought the beam. */
  part: number;
}

/**
 * The welded craft. It is a part (`type` `craft`), so `compilePart`, `partBudget` and `partMass`
 * take it as it is. Its `rings` and `patches` are the ones still free, named `<part id>.<port id>`.
 */
export interface Craft extends Part {
  beams: CraftBeam[];
  tree: TreeEntry[];
  /** The craft's own budget numbers (substeps and node-mass ratio). */
  budget: PartBudget;
}

type Mat = readonly [V3, V3, V3];

function dot(a: V3, b: V3): number {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
function cross(a: V3, b: V3): V3 {
  return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
}
function sub(a: V3, b: V3): V3 {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}
function add(a: V3, b: V3): V3 {
  return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}
function scale(a: V3, s: number): V3 {
  return [a[0] * s, a[1] * s, a[2] * s];
}
function length(a: V3): number {
  return Math.sqrt(dot(a, a));
}
function unit(a: V3, what: string, code: AssemblyRefusal): V3 {
  const l = length(a);
  if (!(l > 0) || !Number.isFinite(l)) throw new AssemblyError(code, `${what} has no direction`);
  return [a[0] / l, a[1] / l, a[2] / l];
}
function removeAlong(v: V3, axis: V3): V3 {
  return sub(v, scale(axis, dot(v, axis)));
}
function pos(n: PartNode): V3 {
  return [n.x, n.y, n.z];
}

/** A rigid move: points and free vectors (a vector is turned, not shifted). */
interface Move {
  point: (p: V3) => V3;
  vector: (v: V3) => V3;
}

/**
 * The rigid move that takes frame `from` onto frame `to` (each three orthonormal, right-handed
 * vectors) and `fromOrigin` onto `toOrigin`: `p ↦ toOrigin + Σ ((p − fromOrigin) · from_i) to_i`.
 */
function frameMove(from: Mat, fromOrigin: V3, to: Mat, toOrigin: V3): Move {
  const vector = (v: V3): V3 => {
    let out: V3 = [0, 0, 0];
    for (let i = 0; i < 3; i++) out = add(out, scale(to[i]!, dot(v, from[i]!)));
    return out;
  };
  return { vector, point: (p) => add(toOrigin, vector(sub(p, fromOrigin))) };
}

/** Turns +x toward +z by `yaw` steps of 15° (the table is in 7.5° units). */
function yawMove(yaw: number, shift: V3): Move {
  const [c, s] = unitCosSin(2 * yaw);
  const vector = (v: V3): V3 => [c * v[0] - s * v[2], v[1], s * v[0] + c * v[2]];
  return { vector, point: (p) => add(vector(p), shift) };
}

/** Which way a ring's nodes go round its axis: +1 or −1 (0 when it has no area). */
function ringSense(part: Part, ring: AttachmentRing, axis: V3): number {
  let s = 0;
  const n = ring.nodes.length;
  for (let k = 0; k < n; k++) {
    const p = sub(pos(part.nodes[ring.nodes[k]!]!), ring.centre);
    const q = sub(pos(part.nodes[ring.nodes[(k + 1) % n]!]!), ring.centre);
    s += dot(cross(p, q), axis);
  }
  return s > 0 ? 1 : s < 0 ? -1 : 0;
}

/** The frame of a mount patch: outward, up and tangent unit vectors, and its centre. */
function patchFrame(at: readonly V3[], nodes: readonly number[], what: string): { frame: Mat; centre: V3 } {
  const [a, b, c, d] = nodes.map((i) => at[i]!) as [V3, V3, V3, V3];
  const t = unit(add(sub(b, a), sub(d, c)), `${what}'s tangent`, 'patch-shape');
  const u0 = add(sub(c, a), sub(d, b));
  const u = unit(removeAlong(u0, t), `${what}'s up`, 'patch-shape');
  const centre = scale(add(add(a, b), add(c, d)), 0.25);
  return { frame: [cross(u, t), u, t], centre };
}

/** A part once placed: where its nodes are, how it was moved, and what it has joined. */
interface Placed {
  entry: number;
  part: Part;
  at: V3[];
  move: Move;
  nodeOf: number[];
  used: Set<string>;
}

/**
 * Welds the parts. Refuses, with an `AssemblyError` that says why: no root, a repeated or unknown
 * part, a port that does not exist, a clock that is not a whole number, a part joined to itself or
 * to two parents, a cycle, a part not connected to the root, a ring or patch used twice (or two
 * patches that share a node), a ring joined to a patch, rings of different node counts or radii,
 * rings or patches that do not meet within 1 mm once placed.
 */
export function assemble(assembly: Assembly): Craft {
  const byId = new Map<string, Part>();
  for (const { id, part } of assembly.parts) {
    if (byId.has(id)) throw new AssemblyError('duplicate-part', `two parts are called ${id}`);
    byId.set(id, part);
    for (const [i, n] of part.nodes.entries()) {
      if (!(n.mass > 0) || !Number.isFinite(n.mass)) throw new AssemblyError('bad-part', `${id} node ${i} has mass ${n.mass}`);
    }
  }
  if (!byId.has(assembly.root)) throw new AssemblyError('no-root', `the root part ${assembly.root} is not among the parts`);

  // Structure first: every refusal that needs no geometry.
  const parentOf = new Map<string, string>();
  for (const j of assembly.joins) {
    if (!byId.has(j.child)) throw new AssemblyError('unknown-part', `a join names the part ${j.child}, which is not in the assembly`);
    if (!byId.has(j.parent)) throw new AssemblyError('unknown-part', `a join names the part ${j.parent}, which is not in the assembly`);
    if (j.child === j.parent) throw new AssemblyError('self-join', `${j.child} is joined to itself`);
    if (!Number.isInteger(j.clock)) throw new AssemblyError('bad-clock', `the clock of ${j.child} on ${j.parent} is ${j.clock}; it must be a whole number of ring steps`);
    if (parentOf.has(j.child)) {
      throw new AssemblyError('attached-twice', `${j.child} is attached twice (to ${parentOf.get(j.child)} and to ${j.parent})`);
    }
    parentOf.set(j.child, j.parent);
  }
  for (const id of byId.keys()) {
    const seen = new Set<string>([id]);
    for (let at = parentOf.get(id); at !== undefined; at = parentOf.get(at)) {
      if (seen.has(at)) throw new AssemblyError('cycle', `${id} is its own ancestor: the joins go round in a cycle through ${at}`);
      seen.add(at);
    }
  }
  if (parentOf.has(assembly.root)) {
    throw new AssemblyError('attached-twice', `the root ${assembly.root} is also attached to ${parentOf.get(assembly.root)}; the root has no parent`);
  }
  for (const id of byId.keys()) {
    let top = id;
    for (let p = parentOf.get(top); p !== undefined; p = parentOf.get(top)) top = p;
    if (top !== assembly.root) throw new AssemblyError('disconnected', `${id} is not connected to the root ${assembly.root}`);
  }
  const place = assembly.place ?? { x: 0, y: 0, z: 0, yaw: 0 };
  if (!Number.isInteger(place.yaw)) throw new AssemblyError('bad-clock', `the root's yaw must be a whole number of 15° steps, not ${place.yaw}`);
  for (const v of [place.x, place.y, place.z]) {
    if (!Number.isFinite(v)) throw new AssemblyError('bad-part', `the root's position must be finite numbers`);
  }

  // The craft is built into fresh arrays: nothing of the input is touched.
  const nodes: PartNode[] = [];
  const beams: CraftBeam[] = [];
  const panels: SkinCell[] = [];
  const thrusters: PartThruster[] = [];
  const segments: Segment[] = [];
  const seams: Seam[] = [];
  const tree: TreeEntry[] = [];
  const placedBy = new Map<string, Placed>();
  const used = new Map<string, Set<string>>();
  for (const id of byId.keys()) used.set(id, new Set<string>());

  /** Marks a port used, refusing one used before and a patch that shares a node with a used patch. */
  const claim = (owner: string, part: Part, port: string, text: string): void => {
    const taken = used.get(owner)!;
    if (taken.has(port)) throw new AssemblyError('port-used', `${owner} ${port} is already used; ${text}`);
    const patch = part.patches.find((p) => p.id === port);
    if (patch !== undefined) {
      for (const other of part.patches) {
        if (other.id === port || !taken.has(other.id)) continue;
        if (patch.nodes.some((i) => other.nodes.includes(i))) {
          throw new AssemblyError('port-used', `${owner} ${port} shares a node with ${other.id}, which is already used; ${text}`);
        }
      }
    }
    taken.add(port);
  };

  /** Adds a part: its nodes (all new except the merged ones), beams, panels, thrusters, segments, seams. */
  const addPart = (
    id: string,
    part: Part,
    at: V3[],
    move: Move,
    parent: Placed | null,
    merge: ReadonlyMap<number, number>,
    join: AssemblyJoin | null,
    joinAxis: V3 | null,
  ): void => {
    const entry = tree.length;
    const segmentBase = segments.length;
    for (const s of part.segments) segments.push({ neverLive: s.neverLive });
    const nodeOf: number[] = [];
    for (const [i, n] of part.nodes.entries()) {
      const into = merge.get(i);
      if (into !== undefined) {
        // The merged node stays where the parent has it, keeps the parent's segment, and the masses add.
        const target = nodes[into]!;
        target.mass += n.mass;
        if (n.radius > target.radius) target.radius = n.radius;
        nodeOf.push(into);
      } else {
        nodeOf.push(nodes.length);
        nodes.push({ x: at[i]![0], y: at[i]![1], z: at[i]![2], mass: n.mass, radius: n.radius, segment: segmentBase + n.segment });
      }
    }
    const mergedNodes = new Set<number>(merge.values());
    let dropped = 0;
    const seamBeams: number[] = [];
    for (const b of part.beams) {
      const ma = merge.has(b.a);
      const mb = merge.has(b.b);
      if (ma && mb && b.ring) {
        dropped++;
        continue;
      }
      const touches = ma || mb;
      if (touches) seamBeams.push(beams.length);
      const copy: CraftBeam = { a: nodeOf[b.a]!, b: nodeOf[b.b]!, material: b.material, area: b.area, zeta: b.zeta, ring: b.ring, seam: touches, part: entry };
      if (b.override !== undefined) copy.override = { ...b.override };
      beams.push(copy);
    }
    for (const cell of part.panels) panels.push({ tets: cell.tets.map((t): Tet => [nodeOf[t[0]]!, nodeOf[t[1]]!, nodeOf[t[2]]!, nodeOf[t[3]]!]) });
    for (const t of part.thrusters) thrusters.push({ tail: nodeOf[t.tail]!, head: nodeOf[t.head]!, maxThrust: t.maxThrust });
    for (const s of part.seams) seams.push({ segmentA: segmentBase + s.segmentA, segmentB: segmentBase + s.segmentB, area: s.area, bending: s.bending });

    let seam: number | null = null;
    if (parent !== null && joinAxis !== null && merge.size > 0) {
      const first = merge.keys().next().value as number;
      const segmentA = nodes[merge.get(first)!]!.segment;
      const segmentB = segmentBase + part.nodes[first]!.segment;
      let centroid: V3 = [0, 0, 0];
      for (const into of mergedNodes) centroid = add(centroid, pos(nodes[into]!));
      centroid = scale(centroid, 1 / mergedNodes.size);
      let area = 0;
      let bending = 0;
      for (const bi of seamBeams) {
        const b = beams[bi]!;
        const p = pos(nodes[b.a]!);
        const q = pos(nodes[b.b]!);
        const span = sub(q, p);
        const cos = dot(span, joinAxis) / length(span);
        const eff = b.area * cos * cos;
        const end = mergedNodes.has(b.a) ? p : q;
        const lateral = removeAlong(sub(end, centroid), joinAxis);
        area += eff;
        bending += eff * dot(lateral, lateral);
      }
      seam = seams.length;
      seams.push({ segmentA, segmentB, area, bending });
    }

    tree.push({
      id,
      type: part.type,
      parent: parent === null ? null : parent.entry,
      join,
      nodeOf,
      placed: at.map((p) => [p[0], p[1], p[2]] as V3),
      segmentBase,
      seam,
      seamBeams,
      droppedBeams: dropped,
      merged: merge.size,
      capsule: part.capsule === null ? null : { a: move.point(part.capsule.a), b: move.point(part.capsule.b), radius: part.capsule.radius },
    });
    placedBy.set(id, { entry, part, at, move, nodeOf, used: used.get(id)! });
  };

  // The root.
  {
    const part = byId.get(assembly.root)!;
    const move = yawMove(place.yaw, [place.x, place.y, place.z]);
    addPart(assembly.root, part, part.nodes.map((n) => move.point(pos(n))), move, null, new Map(), null, null);
  }

  // The joins, each once its parent is in, in the order given.
  const remaining = [...assembly.joins];
  while (remaining.length > 0) {
    const index = remaining.findIndex((j) => placedBy.has(j.parent));
    if (index < 0) throw new AssemblyError('disconnected', `${remaining[0]!.child} is not connected to the root ${assembly.root}`);
    const join = remaining.splice(index, 1)[0]!;
    const child = byId.get(join.child)!;
    const parent = placedBy.get(join.parent)!;
    const text = `${join.child} ${join.childPort} on ${join.parent} ${join.parentPort}`;

    const cRing = child.rings.find((r) => r.id === join.childPort);
    const cPatch = child.patches.find((p) => p.id === join.childPort);
    const pRing = parent.part.rings.find((r) => r.id === join.parentPort);
    const pPatch = parent.part.patches.find((p) => p.id === join.parentPort);
    if (cRing === undefined && cPatch === undefined) throw new AssemblyError('unknown-port', `${join.child} has no ring or patch ${join.childPort}`);
    if (pRing === undefined && pPatch === undefined) throw new AssemblyError('unknown-port', `${join.parent} has no ring or patch ${join.parentPort}`);

    const merge = new Map<number, number>();
    let move: Move;
    let axis: V3;

    if (cRing !== undefined && pRing !== undefined) {
      if (cRing.nodes.length !== pRing.nodes.length) {
        throw new AssemblyError('node-count', `the rings differ in node count (${join.child} ${join.childPort} has ${cRing.nodes.length}, ${join.parent} ${join.parentPort} has ${pRing.nodes.length}); ${text}`);
      }
      if (!(Math.abs(cRing.radius - pRing.radius) <= WELD_TOLERANCE)) {
        throw new AssemblyError('radius', `the rings differ in radius (${join.child} ${join.childPort} is ${cRing.radius} m, ${join.parent} ${join.parentPort} is ${pRing.radius} m; the limit is ${WELD_TOLERANCE} m); ${text}`);
      }
      const n = cRing.nodes.length;
      const cAxis = unit(cRing.axis, `${join.child} ${join.childPort}'s axis`, 'ring-shape');
      const pAxis = unit(pRing.axis, `${join.parent} ${join.parentPort}'s axis`, 'ring-shape');
      const sc = ringSense(child, cRing, cAxis);
      const sp = ringSense(parent.part, pRing, pAxis);
      if (sc === 0 || sp === 0) throw new AssemblyError('ring-shape', `a ring has no sense of rotation; ${text}`);
      // Seen along the parent's axis the child's ring runs the other way round: the numberings agree when the senses are opposite.
      const agree = -sc === sp;
      const clock = ((join.clock % n) + n) % n;
      const partner = (j: number): number => (agree ? (j + clock) % n : (((clock - j) % n) + n) % n);
      claim(join.parent, parent.part, join.parentPort, text);
      claim(join.child, child, join.childPort, text);

      const pCentre = parent.move.point(pRing.centre);
      const pAxisPlaced = parent.move.vector(pAxis);
      const parentFirst = parent.at[pRing.nodes[partner(0)]!]!;
      const e1 = cAxis;
      const e2 = unit(removeAlong(sub(pos(child.nodes[cRing.nodes[0]!]!), cRing.centre), e1), `${join.child} ${join.childPort}'s first node`, 'ring-shape');
      const f1 = scale(pAxisPlaced, -1);
      const f2 = unit(removeAlong(sub(parentFirst, pCentre), f1), `${join.parent} ${join.parentPort}'s node`, 'ring-shape');
      move = frameMove([e1, e2, cross(e1, e2)], cRing.centre, [f1, f2, cross(f1, f2)], pCentre);
      axis = pAxisPlaced;
      for (let j = 0; j < n; j++) merge.set(cRing.nodes[j]!, parent.nodeOf[pRing.nodes[partner(j)]!]!);
    } else if (cPatch !== undefined && pPatch !== undefined) {
      if (join.clock !== 0) throw new AssemblyError('patch-clock', `a mount patch takes no clock, not ${join.clock}; ${text}`);
      claim(join.parent, parent.part, join.parentPort, text);
      claim(join.child, child, join.childPort, text);
      const cf = patchFrame(child.nodes.map(pos), cPatch.nodes, `${join.child} ${join.childPort}`);
      const pf = patchFrame(parent.at, pPatch.nodes, `${join.parent} ${join.parentPort}`);
      let body: V3 = [0, 0, 0];
      for (const p of parent.at) body = add(body, p);
      body = scale(body, 1 / parent.at.length);
      if (!(dot(sub(pf.centre, body), pf.frame[0]) > 0)) {
        throw new AssemblyError('patch-inward', `${join.parent} ${join.parentPort} faces into its part, so nothing can stand on it; ${text}`);
      }
      move = frameMove(cf.frame, cf.centre, pf.frame, pf.centre);
      axis = pf.frame[0];
      for (let k = 0; k < 4; k++) merge.set(cPatch.nodes[k]!, parent.nodeOf[pPatch.nodes[k]!]!);
    } else {
      throw new AssemblyError('port-kind', `a ring cannot be joined to a mount patch (${join.child} ${join.childPort} is a ${cRing !== undefined ? 'ring' : 'patch'}, ${join.parent} ${join.parentPort} is a ${pRing !== undefined ? 'ring' : 'patch'}); ${text}`);
    }

    const at = child.nodes.map((n) => move.point(pos(n)));
    // Every merged pair must meet.
    for (const [ci, craftNode] of merge) {
      const gap = length(sub(at[ci]!, pos(nodes[craftNode]!)));
      if (!(gap <= WELD_TOLERANCE)) {
        const code: AssemblyRefusal = cRing !== undefined ? 'ring-shape' : 'patch-shape';
        throw new AssemblyError(code, `${join.child} node ${ci} lands ${gap} m from the ${join.parent} node it joins, over the limit of ${WELD_TOLERANCE} m; ${text}`);
      }
    }
    addPart(join.child, child, at, move, parent, merge, join, axis);
  }

  // Total mass: exactly the parts' total, the last node taking the rounding.
  let target = 0;
  for (const { part } of assembly.parts) target += partMass(part);
  fixTotalMass(nodes, target);

  // The rings and patches still free, in craft node numbers.
  const rings: AttachmentRing[] = [];
  const patches: Part['patches'] = [];
  for (const entry of tree) {
    const placed = placedBy.get(entry.id)!;
    for (const r of placed.part.rings) {
      if (placed.used.has(r.id)) continue;
      rings.push({ id: `${entry.id}.${r.id}`, nodes: r.nodes.map((i) => entry.nodeOf[i]!), centre: placed.move.point(r.centre), axis: placed.move.vector(r.axis), radius: r.radius });
    }
    for (const p of placed.part.patches) {
      if (placed.used.has(p.id)) continue;
      const [a, b, c, d] = p.nodes;
      patches.push({ id: `${entry.id}.${p.id}`, nodes: [entry.nodeOf[a]!, entry.nodeOf[b]!, entry.nodeOf[c]!, entry.nodeOf[d]!] });
    }
  }

  const craft = { type: 'craft', nodes, beams, panels, rings, patches, seams, segments, capsule: null, thrusters, tree } as Craft;
  const budget = partBudget(craft);
  if (budget.substeps > SUBSTEP_BUDGET) {
    throw new AssemblyError('substeps', `the craft would need ${budget.substeps} substeps, over the budget of ${SUBSTEP_BUDGET}`);
  }
  if (budget.massRatio > CRAFT_NODE_MASS_RATIO_BUDGET) {
    throw new AssemblyError('mass-ratio', `the heaviest node is ${budget.massRatio} times the lightest, over the craft budget of ${CRAFT_NODE_MASS_RATIO_BUDGET}`);
  }
  // A craft is held to its own ratio budget, which is looser than a single part's (ER 004, phase 4.4).
  budget.problems = budget.problems.filter((p) => !p.includes('the heaviest node'));
  budget.withinBudget = budget.problems.length === 0;
  craft.budget = budget;
  return craft;
}

/** Makes the node masses, added in index order, give `target` exactly (the last node takes the rounding). */
function fixTotalMass(nodes: PartNode[], target: number): void {
  const last = nodes[nodes.length - 1]!;
  let prefix = 0;
  for (let i = 0; i < nodes.length - 1; i++) prefix += nodes[i]!.mass;
  last.mass = target - prefix;
  for (let tries = 0; tries < 8; tries++) {
    let sum = 0;
    for (const n of nodes) sum += n.mass;
    if (sum === target) return;
    last.mass += target - sum;
  }
}

/** Options of `compileCraft`. */
export interface CompileCraftOptions {
  /** Emit the segment commands after the craft's thrusters (default true; plan 005, phase 5.1). */
  segments?: boolean;
}

/**
 * Appends a craft to `batch`: `compilePart` (nodes, beams, thrusters), then, unless `segments` is
 * false, its segments, seams and tree as `CMD_ADD_SEGMENT` and `CMD_SET_NODE_SEGMENT` records, in
 * that order. Returns the core's segment number for each craft segment (for an empty world these
 * are the numbers the records get), or `null` when no segments were emitted.
 *
 * The core's tree is made from the craft's seams: from craft segment 0 (the root part's first), a
 * breadth-first walk over the seams gives every segment its parent and the seam's area and bending;
 * a segment the seams do not reach becomes a root. A core segment needs its parent to exist first,
 * so records go out in "lowest-numbered segment whose parent is already out" order; for the hopper
 * that is craft order, so the core's numbers equal the craft's. They change no physics.
 */
export function compileCraft(batch: CommandBatch, craft: Craft, options: CompileCraftOptions = {}): number[] | null {
  compilePart(batch, craft);
  if (options.segments === false) return null;
  const count = craft.segments.length;
  // Tree from the seams, discovered breadth-first from segment 0 and then from any segment left over.
  const parent: (number | null)[] = new Array(count).fill(null);
  const seamOf: (number | null)[] = new Array(count).fill(null);
  const seen: boolean[] = new Array(count).fill(false);
  for (let start = 0; start < count; start++) {
    if (seen[start]) continue;
    seen[start] = true;
    const queue = [start];
    for (let head = 0; head < queue.length; head++) {
      const s = queue[head]!;
      for (const [k, seam] of craft.seams.entries()) {
        const other = seam.segmentA === s ? seam.segmentB : seam.segmentB === s ? seam.segmentA : -1;
        if (other < 0 || other >= count || seen[other]) continue;
        seen[other] = true;
        parent[other] = s;
        seamOf[other] = k;
        queue.push(other);
      }
    }
  }
  // Emit in "lowest ready segment first" order.
  const coreOf: number[] = new Array(count).fill(-1);
  let next = 0;
  for (let emitted = 0; emitted < count; emitted++) {
    let pick = -1;
    for (let s = 0; s < count; s++) {
      if (coreOf[s]! < 0 && (parent[s] === null || coreOf[parent[s]!]! >= 0)) {
        pick = s;
        break;
      }
    }
    const seam = seamOf[pick] === null ? null : craft.seams[seamOf[pick]!]!;
    batch.addSegment(
      parent[pick] === null ? NO_SEGMENT : coreOf[parent[pick]!]!,
      craft.segments[pick]!.neverLive ? SEG_NEVER_LIVE : 0,
      seam?.area ?? 0,
      seam?.bending ?? 0,
    );
    coreOf[pick] = next++;
  }
  for (const [i, n] of craft.nodes.entries()) batch.setNodeSegment(i, coreOf[n.segment]!);
  return coreOf;
}
