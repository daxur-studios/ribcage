/**
 * Purpose: the landing leg generator: a four-cornered lattice from a mount patch on a tank wall out
 * to a foot, in four stages from the mount to the foot: the upper leg (an ordinary stiff lattice), a
 * crush bay (every member yields together at one force and breaks together at one stroke, so the
 * leg gives way at a steady force), a sprung bay (every member soft, so the foot moves in a spring
 * with no bounce) and a short stiff foot adapter. The crush and sprung bays are straight square
 * prisms along the axis, so a push along the axis loads them evenly and does not shear them. The
 * mount patch and the foot are not square to the axis; the upper leg and the foot adapter are the
 * stiff wedges that turn the square section into them.
 * Value: the leg is where a hard landing is taken. The spring takes a gentle landing, the crush bay
 * takes a hard one at a force the tank can bear, and the upper leg is checked to stay whole while it
 * does, so the leg fails where the design says and nowhere else.
 *
 * Only `+ − × ÷` and `sqrt` appear in the maths (see `shared.ts`).
 *
 * Frame: the mount patch is in the plane x = 0 centred on the origin, outward is +x, the tangent
 * from the patch's first node to its second is +z, up is +y. The foot's centre is
 * `(footOffset, −drop, 0)` with `drop = sqrt(length² − footOffset²)`, so the straight line from the
 * mount's centre to the foot's centre is `length` long.
 */

import { MATERIALS, member, type MaterialId } from './material.ts';
import { ZETA_SPRUNG, ZETA_STRUCTURE, emptyPart, type BeamOverride, type Part, type PartBeam } from './part.ts';
import { DEFAULT_NODE_RADIUS, distributeMass, requirePositive, ringBracing } from './shared.ts';

export interface LegParams {
  /** Straight distance from the mount's centre to the foot's centre in metres. */
  length: number;
  /** How far out from the mount the foot's centre is, in metres (0 up to just under `length`). */
  footOffset: number;
  /** Side of the foot's square in metres. */
  footSize: number;
  /** Width of the mount patch (along the wall's tangent) in metres. */
  mountWidth: number;
  /** Height of the mount patch (along the tank's axis) in metres. */
  mountHeight: number;
  material: MaterialId;
  /** Cross-section area of every member in square metres (the upper leg's stiffness and strength). */
  memberArea: number;
  /** Stated total mass in kilograms: at least the members' own mass. */
  mass: number;
  /** Spring constant of the sprung bay along the leg's axis in newtons per metre. */
  sprungStiffness: number;
  /** How far the spring is pressed at the crush force, in metres: the bay's working stroke. */
  sprungStroke: number;
  /** Force along the leg's axis at which the crush bay gives way, in newtons. */
  crushForce: number;
  /** How far the crush bay shortens before its members break, in metres. */
  crushStroke: number;
  /** Side of the square section of the crush and sprung bays in metres (default: the largest of the mount's sides and the foot's). A wider section holds the soft bays against folding. */
  bodyWidth?: number;
  /** Collision radius of each node in metres (default 0.05). */
  nodeRadius?: number;
}

/** Bay length as a multiple of its stroke: long enough that the bay's strain stays moderate. */
const BAY_PER_STROKE = 2.5;
/** The foot adapter is this share of the leg's length. */
const FOOT_SHARE = 0.15;
/** The upper leg is at least this share of the leg's length. */
const UPPER_SHARE_MIN = 0.25;
/** The crush bay's elastic strain along its axis when it yields. */
const CRUSH_YIELD_STRAIN = 0.02;
/** Share of the crush force carried by the four longerons; the eight diagonals carry the rest. */
const CRUSH_LONGERON_SHARE = 0.75;
/** The sprung bay yields at this multiple of its stroke, and breaks at twice that. */
const SPRUNG_YIELD_STROKES = 1.5;
/** The sprung bay must still be elastic at the crush force, with this margin. */
const SPRUNG_MARGIN = 1.1;
/** The upper leg's longerons carry at least this multiple of the crush force. */
const UPPER_MARGIN = 2;

type V = [number, number, number];

/**
 * A leg. Node order: five stations of four nodes, so node `4k + j` is corner `j` of station `k`:
 * station 0 is the mount (nodes 0 to 3), 1 the top of the crush bay, 2 the bottom of the crush bay
 * (the top of the sprung bay), 3 the bottom of the sprung bay, 4 the foot (nodes 16 to 19). Corners
 * go round the section: `j` = 0 is (low, low), 1 (low, high), 2 (high, high), 3 (high, low), where
 * the first coordinate is up on the mount, outward and up across the axis in the square stations and
 * outward on the foot, and the second is along the tangent. Beams: the stations' in-ring members
 * (flagged ring), then the four bays of twelve (four longerons and eight diagonals each, in
 * wall-quad order): bay 0 the upper leg, 1 the crush bay, 2 the sprung bay, 3 the foot adapter. One segment, one mount patch
 * (`mount`: lower pair, then upper pair), no attachment rings, no skin panels. Refuses bad
 * parameters, an upper leg too weak to outlast the crush bay (with the area it would need), a spring
 * that yields before the crush force, and a stated mass below the members' own.
 */
export function leg(p: LegParams): Part {
  requirePositive('a leg length', p.length);
  requirePositive('a leg foot size', p.footSize);
  requirePositive('a leg mount width', p.mountWidth);
  requirePositive('a leg mount height', p.mountHeight);
  requirePositive('a leg member area', p.memberArea);
  requirePositive('a leg sprung stiffness', p.sprungStiffness);
  requirePositive('a leg sprung stroke', p.sprungStroke);
  requirePositive('a leg crush force', p.crushForce);
  requirePositive('a leg crush stroke', p.crushStroke);
  if (!(p.footOffset >= 0) || !Number.isFinite(p.footOffset)) throw new Error(`a leg foot offset must be zero or more, not ${p.footOffset}`);
  if (!(p.footOffset < p.length)) throw new Error(`a leg foot offset ${p.footOffset} m must be less than its length ${p.length} m`);
  const nodeRadius = p.nodeRadius ?? DEFAULT_NODE_RADIUS;
  requirePositive('a leg node radius', nodeRadius);

  const drop = Math.sqrt(p.length * p.length - p.footOffset * p.footOffset);
  const sprungLength = BAY_PER_STROKE * p.sprungStroke;
  const crushLength = BAY_PER_STROKE * p.crushStroke;
  const footLength = FOOT_SHARE * p.length;
  const upperLength = p.length - sprungLength - crushLength - footLength;
  if (upperLength < UPPER_SHARE_MIN * p.length) {
    throw new Error(
      `the upper leg would be ${upperLength} m of a ${p.length} m leg (the spring takes ${sprungLength} m, the crush bay ${crushLength} m and the foot ${footLength} m); it must be at least ${UPPER_SHARE_MIN * p.length} m`,
    );
  }
  if (SPRUNG_YIELD_STROKES * p.sprungStiffness * p.sprungStroke < SPRUNG_MARGIN * p.crushForce) {
    throw new Error(
      `the spring (${p.sprungStiffness} N/m over ${p.sprungStroke} m) would yield at ${SPRUNG_YIELD_STROKES * p.sprungStiffness * p.sprungStroke} N, before the crush force ${p.crushForce} N`,
    );
  }

  const bodyWidth = p.bodyWidth ?? Math.max(p.mountWidth, p.mountHeight, p.footSize);
  requirePositive('a leg body width', bodyWidth);

  // Station k sits `along[k]` down the axis. The end stations are the mount's and the foot's corners;
  // the three between are squares across the axis, so the two soft bays are straight prisms.
  const along = [0, upperLength, upperLength + crushLength, upperLength + crushLength + sprungLength, p.length];
  // The unit axis from the mount's centre to the foot's centre, and the unit across it in the leg's plane.
  const u: V = [p.footOffset / p.length, -drop / p.length, 0];
  const across: V = [drop / p.length, p.footOffset / p.length, 0];
  const mount = (j: number): V => {
    const [first, second] = CORNER[j]!;
    return [0, first * (p.mountHeight / 2), second * (p.mountWidth / 2)];
  };
  const foot = (j: number): V => {
    const [first, second] = CORNER[j]!;
    return [p.footOffset + first * (p.footSize / 2), -drop, second * (p.footSize / 2)];
  };
  const body = (k: number, j: number): V => {
    const [first, second] = CORNER[j]!;
    const s = along[k]!;
    const h = bodyWidth / 2;
    return [u[0] * s + first * h * across[0], u[1] * s + first * h * across[1], second * h];
  };

  const part = emptyPart('leg');
  part.segments.push({ neverLive: false });
  for (let k = 0; k < 5; k++) {
    for (let j = 0; j < 4; j++) {
      const at = k === 0 ? mount(j) : k === 4 ? foot(j) : body(k, j);
      part.nodes.push({ x: at[0], y: at[1], z: at[2], mass: 0, radius: nodeRadius, segment: 0 });
    }
  }

  const push = (a: number, b: number, zeta: number, ring: boolean, override?: BeamOverride): void => {
    const beam: PartBeam = { a, b, material: p.material, area: p.memberArea, zeta, ring };
    if (override !== undefined) beam.override = override;
    part.beams.push(beam);
  };
  for (let k = 0; k < 5; k++) for (const [a, b] of ringBracing(4)) push(4 * k + a, 4 * k + b, ZETA_STRUCTURE, true);

  /** The bay's twelve beams as node pairs, in wall-quad order (longeron, diagonal, diagonal). */
  const bayPairs = (bay: number): [number, number][] => {
    const out: [number, number][] = [];
    for (let j = 0; j < 4; j++) {
      const a = 4 * bay + j;
      const b = 4 * bay + ((j + 1) % 4);
      const c = 4 * (bay + 1) + j;
      const d = 4 * (bay + 1) + ((j + 1) % 4);
      out.push([a, c], [a, d], [b, c]);
    }
    return out;
  };
  interface Axial {
    /** Length of the beam. */
    ell: number;
    /** Cosine of the angle between the beam and the axis. */
    cos: number;
    /** Strain of the beam per unit strain of the bay along the axis. */
    r: number;
  }
  const axial = (bay: number): { beams: [number, number][]; geometry: Axial[]; span: number } => {
    const beams = bayPairs(bay);
    const span = along[bay + 1]! - along[bay]!;
    const geometry = beams.map(([a, b]): Axial => {
      const lo = part.nodes[a]!;
      const hi = part.nodes[b]!;
      const dx = hi.x - lo.x;
      const dy = hi.y - lo.y;
      const dz = hi.z - lo.z;
      const ell = Math.sqrt(dx * dx + dy * dy + dz * dz);
      const lengthways = dx * u[0] + dy * u[1] + dz * u[2];
      return { ell, cos: lengthways / ell, r: (lengthways * span) / (ell * ell) };
    });
    return { beams, geometry, span };
  };

  // Bay 0, the upper leg: ordinary members. Its four longerons must outlast the crush force.
  const upper = axial(0);
  const mem = member(p.material, p.memberArea);
  const capacity = mem.yieldForce ?? MATERIALS[p.material].breakStrain * mem.ea;
  let carried = 0;
  for (let i = 0; i < upper.beams.length; i += 3) carried += capacity * upper.geometry[i]!.cos;
  if (carried < UPPER_MARGIN * p.crushForce) {
    const need = (p.memberArea * UPPER_MARGIN * p.crushForce) / carried;
    throw new Error(
      `the upper leg's four longerons carry ${carried} N along the leg, under ${UPPER_MARGIN} times the crush force ${p.crushForce} N; give the members an area of at least ${need} m² (now ${p.memberArea} m²)`,
    );
  }
  for (const [a, b] of upper.beams) push(a, b, ZETA_STRUCTURE, false);

  // Bay 1, the crush bay. Every member yields at the same strain of the bay and breaks at the same
  // stroke; the longerons carry most of the force. A member's share of the force along the axis is
  // fixed, so its own yield force is that share over its cosine.
  const crush = axial(1);
  for (const [i, [a, b]] of crush.beams.entries()) {
    const g = crush.geometry[i]!;
    const share = i % 3 === 0 ? CRUSH_LONGERON_SHARE / 4 : (1 - CRUSH_LONGERON_SHARE) / 8;
    const yieldForce = (p.crushForce * share) / g.cos;
    const yieldStrain = g.r * CRUSH_YIELD_STRAIN;
    push(a, b, ZETA_STRUCTURE, false, { ea: yieldForce / yieldStrain, yieldStrain, breakStrain: (g.r * p.crushStroke) / crush.span });
  }

  // Bay 2, the sprung bay. Every member has the same spring constant along its own length, so the
  // bay as a whole is a spring of `sprungStiffness` along the axis. It yields at 1.5 strokes and
  // breaks at 3, and is damped to ζ = 0.6.
  const sprung = axial(2);
  let cos2 = 0;
  for (const g of sprung.geometry) cos2 += g.cos * g.cos;
  const kappa = p.sprungStiffness / cos2;
  for (const [i, [a, b]] of sprung.beams.entries()) {
    const g = sprung.geometry[i]!;
    const strain = (g.r * p.sprungStroke) / sprung.span;
    push(a, b, ZETA_SPRUNG, false, { ea: kappa * g.ell, yieldStrain: SPRUNG_YIELD_STROKES * strain, breakStrain: 2 * SPRUNG_YIELD_STROKES * strain });
  }

  // Bay 3, the foot adapter: ordinary members, turning the square section into the foot's.
  for (const [a, b] of bayPairs(3)) push(a, b, ZETA_STRUCTURE, false);

  part.patches.push({ id: 'mount', nodes: [0, 1, 3, 2] });
  part.capsule = { a: [0, 0, 0], b: [p.footOffset, -drop, 0], radius: bodyWidth / 2 };
  distributeMass(part, p.mass);
  return part;
}

/** The section's corners as (first, second) signs, going round: low-low, low-high, high-high, high-low. */
const CORNER: readonly (readonly [number, number])[] = [
  [-1, -1],
  [-1, 1],
  [1, 1],
  [1, -1],
];
