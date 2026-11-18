/**
 * Purpose: a small test rover: a box chassis (a lattice with a stub for each axle) and four wheels
 * (`wheel.ts`), each wheel hinged to its stub with a rotor on the hinge. `compileRover` writes it as
 * commands: the nodes and beams of every part, one segment per part, each wheel made rigid, and the
 * four joints; `driveRover` sets the rotors.
 * Value: the first craft with a moving part. The wheels are joined by hinge, not by weld, so the
 * ring assembler is not used; parts are placed and their nodes appended in a fixed order, so the
 * same numbers always give the same commands.
 *
 * Only `+ - * /` and `sqrt` appear in the maths (see `shared.ts`).
 */

import { NO_SEGMENT, SEG_NEVER_LIVE, JOINT_ROTOR, type CommandBatch } from '../commands.ts';
import { distributeMass } from './shared.ts';
import { ZETA_STRUCTURE, compilePart, emptyPart, type Part } from './part.ts';
import { wheel, type WheelParams } from './wheel.ts';

export interface RoverParams {
  /** Chassis length (x), width between the wheel hinges (z), and height (y), in metres. */
  length: number;
  width: number;
  height: number;
  /** Where the chassis's centre starts in x and z, and the height of the axles above the ground. */
  x: number;
  z: number;
  axleHeight: number;
  /** Stated chassis mass in kilograms. */
  chassisMass: number;
  wheel: Omit<WheelParams, 'axleLength'>;
  /** Length of each wheel's axle stub outside the chassis in metres. */
  stub: number;
}

export interface RoverWheel {
  /** Craft segment of the wheel. */
  segment: number;
  /** The hinge's six nodes: chassis side `a0, a1, ra`, wheel side `b0, b1, rb`. */
  hinge: readonly [number, number, number, number, number, number];
  /** Every node of the wheel (its rim, then its axle nodes). */
  nodes: number[];
  /** +1 for a wheel on the +z side, -1 on the -z side. */
  side: 1 | -1;
}

export interface Rover {
  /** The whole rover in the part shape: chassis nodes first, then each wheel's. */
  craft: Part;
  chassisNodes: number[];
  wheels: RoverWheel[];
  /** Wheel rolling radius in metres: the rim radius. A rim of node spheres travels its perimeter per turn, so the node radius adds nothing (ER 005, phase 5.6). */
  rollingRadius: number;
}

/** The chassis: 8 box corners (bottom 0 to 3, top 4 to 7) and 4 axle stubs (8 to 11). */
function chassis(p: RoverParams): Part {
  const part = emptyPart('chassis');
  part.segments.push({ neverLive: false });
  const hx = p.length / 2;
  const hz = p.width / 2;
  const y0 = p.axleHeight;
  const radius = 0.05;
  // Bottom: (+x,+z) (-x,+z) (-x,-z) (+x,-z); the top above each.
  const corners: [number, number][] = [[hx, hz], [-hx, hz], [-hx, -hz], [hx, -hz]];
  for (const [x, z] of corners) part.nodes.push({ x: p.x + x, y: y0, z: p.z + z, mass: 1, radius, segment: 0 });
  for (const [x, z] of corners) part.nodes.push({ x: p.x + x, y: y0 + p.height, z: p.z + z, mass: 1, radius, segment: 0 });
  // A stub outside each bottom corner, along the axle (z).
  for (const [x, z] of corners) part.nodes.push({ x: p.x + x, y: y0, z: p.z + z + (z > 0 ? p.stub : -p.stub), mass: 1, radius, segment: 0 });
  const area = 5e-4;
  const beam = (a: number, b: number): void => {
    part.beams.push({ a, b, material: 'aluminium', area, zeta: ZETA_STRUCTURE, ring: false });
  };
  for (let i = 0; i < 4; i++) {
    const j = (i + 1) % 4;
    beam(i, j);
    beam(4 + i, 4 + j);
    beam(i, 4 + i);
    // Side faces: both diagonals.
    beam(i, 4 + j);
    beam(j, 4 + i);
  }
  // Floor and roof diagonals.
  beam(0, 2);
  beam(1, 3);
  beam(4, 6);
  beam(5, 7);
  // Each stub is braced to its corner, the top node above it, and the other bottom corner on its side
  // (corners 0 and 1 are on +z, 2 and 3 on -z), so it is a rigid tetrahedron on a face.
  const sameSide = [1, 0, 3, 2];
  for (let i = 0; i < 4; i++) {
    beam(8 + i, i);
    beam(8 + i, 4 + i);
    beam(8 + i, sameSide[i]!);
  }
  distributeMass(part, p.chassisMass);
  return part;
}

/** A rover: chassis plus four wheels at the bottom corners, each wheel's hinge at its corner and stub. */
export function rover(p: RoverParams): Rover {
  const chas = chassis(p);
  const craft = emptyPart('rover');
  craft.segments.push({ neverLive: false });
  for (const n of chas.nodes) craft.nodes.push({ ...n, segment: 0 });
  for (const b of chas.beams) craft.beams.push({ ...b });
  const chassisNodes = chas.nodes.map((_, i) => i);
  const wheels: RoverWheel[] = [];
  const hz = p.width / 2;
  const hx = p.length / 2;
  const corners: [number, number][] = [[hx, hz], [-hx, hz], [-hx, -hz], [hx, -hz]];
  const proto = wheel({ ...p.wheel, axleLength: p.stub });
  const rim = p.wheel.rimNodes;
  for (let i = 0; i < 4; i++) {
    const [cx, cz] = corners[i]!;
    const outward = cz > 0 ? 1 : -1;
    // The wheel's centre is half a stub outside its corner. Its local axle runs -z to +z; the -z side
    // is mirrored so the inboard hub node always sits at the chassis corner.
    const mid = cz + outward * (p.stub / 2);
    const segment = craft.segments.length;
    craft.segments.push({ neverLive: true });
    const base = craft.nodes.length;
    const nodes: number[] = [];
    for (const [k, q] of proto.nodes.entries()) {
      craft.nodes.push({ x: p.x + cx + q.x, y: p.axleHeight + q.y, z: p.z + mid + outward * q.z, mass: q.mass, radius: q.radius, segment });
      nodes.push(base + k);
    }
    for (const b of proto.beams) craft.beams.push({ ...b, a: b.a + base, b: b.b + base });
    // Chassis side: the corner (inboard), the stub (outboard) and the top node above the corner as the
    // reference. Wheel side: its inboard and outboard hub nodes and rim node 0 as the reference.
    wheels.push({ segment, hinge: [i, 8 + i, 4 + i, base + rim, base + rim + 1, base], nodes, side: outward > 0 ? 1 : -1 });
  }
  return { craft, chassisNodes, wheels, rollingRadius: p.wheel.radius };
}

export interface RoverCompiled {
  /** The core's joint index for each wheel, in `Rover.wheels` order (an empty world numbers them 0 to 3). */
  joints: number[];
}

/**
 * Appends the rover to `batch` (an empty world, or one with only a terrain): nodes and beams, the
 * chassis segment and a never-live segment for each wheel (a child of the chassis, with no seam
 * area), the wheels made rigid when `rigid` is true, then one hinge per wheel.
 */
export function compileRover(batch: CommandBatch, r: Rover, options: { rigid?: boolean } = {}): RoverCompiled {
  const rigid = options.rigid ?? true;
  compilePart(batch, r.craft);
  for (const [i, s] of r.craft.segments.entries()) {
    batch.addSegment(i === 0 ? NO_SEGMENT : 0, s.neverLive ? SEG_NEVER_LIVE : 0, 0, 0);
  }
  for (const [i, n] of r.craft.nodes.entries()) batch.setNodeSegment(i, n.segment);
  if (rigid) for (const w of r.wheels) batch.makeRigid(w.segment);
  const joints: number[] = [];
  for (const [k, w] of r.wheels.entries()) {
    const [a0, a1, ra, b0, b1, rb] = w.hinge;
    batch.addJoint(a0, a1, ra, b0, b1, rb);
    joints.push(k);
  }
  return { joints };
}

/**
 * Sets every wheel's rotor to roll the rover toward +x at `speed` radians per second of wheel turn
 * (the -z side's hinge axis points the other way, so its rotor takes the opposite sign), with the
 * drive torque capped at `cap` newton-metres per wheel. A speed of 0 holds the wheels still.
 */
export function driveRover(batch: CommandBatch, r: Rover, speed: number, cap: number): void {
  for (const [k, w] of r.wheels.entries()) batch.setJointTarget(k, JOINT_ROTOR, w.side > 0 ? -speed : speed, 0, cap);
}
