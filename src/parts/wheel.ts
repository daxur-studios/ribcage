/**
 * Purpose: the wheel generator: a rim ring of nodes in one plane and a hub of two axle nodes on the
 * ring's axis, braced rim to hub. Never-live: it is one rigid body from spawn, so the rim carries no
 * strain and no rim beam is solved. A wheel is not welded to anything; it is hinged to an axle on a
 * mount (`rover.ts`), and a rotor on that hinge drives it.
 * Value: the wheel is the first part that moves against its parent, one pure function from a few
 * numbers to a `Part`.
 *
 * Only `+ − × ÷` and `sqrt` appear in the maths (see `shared.ts`).
 */

import type { MaterialId } from './material.ts';
import { ZETA_STRUCTURE, emptyPart, type Part } from './part.ts';
import { DEFAULT_NODE_RADIUS, isRingCount, requirePositive, ringPoint, RING_COUNTS } from './shared.ts';

export interface WheelParams {
  /** Nodes in the rim: one of 4, 6, 8, 12, 16, 24. */
  rimNodes: number;
  /** Radius of the rim ring in metres (the rolling radius is this plus the node radius). */
  radius: number;
  /** Distance between the two axle nodes in metres. */
  axleLength: number;
  material: MaterialId;
  /** Cross-section area of every member in square metres. */
  memberArea: number;
  /** Stated mass of the whole wheel in kilograms, mostly on the rim. */
  mass: number;
  /** Share of the mass on the rim ring, 0 to 1 (default 0.8). */
  rimShare?: number;
  nodeRadius?: number;
  /** Turn the rim half a node step (default true), so the wheel rests on two nodes and not balanced on one. */
  halfTurn?: boolean;
}

/**
 * A wheel with its axle along z and its centre at the origin. Node order: the rim (0 to n−1, in the
 * xy plane, node 0 on +x, counter-clockwise seen from +z), then the axle's two nodes: `n` at
 * z = −axleLength/2 and `n + 1` at z = +axleLength/2. The one segment is never-live. Members: the
 * rim's perimeter and every rim node braced to both axle nodes.
 */
export function wheel(p: WheelParams): Part {
  if (!isRingCount(p.rimNodes)) throw new Error(`a wheel rim has ${p.rimNodes} nodes; the counts allowed are ${RING_COUNTS.join(', ')}`);
  requirePositive('a wheel radius', p.radius);
  requirePositive('a wheel axle length', p.axleLength);
  requirePositive('a wheel member area', p.memberArea);
  requirePositive('a wheel mass', p.mass);
  const share = p.rimShare ?? 0.8;
  if (!(share > 0 && share < 1)) throw new Error(`a wheel rim share is between 0 and 1, not ${share}`);
  const nodeRadius = p.nodeRadius ?? DEFAULT_NODE_RADIUS;
  requirePositive('a wheel node radius', nodeRadius);
  const n = p.rimNodes;
  // Half a node step, by the half-angle formulas from the step's own cosine: cos(t/2) = sqrt((1 + cos t) / 2).
  const [cStep] = ringPoint(n, 1, 1);
  const ch = (p.halfTurn ?? true) ? Math.sqrt((1 + cStep) / 2) : 1;
  const sh = (p.halfTurn ?? true) ? Math.sqrt((1 - cStep) / 2) : 0;
  const part = emptyPart('wheel');
  part.segments.push({ neverLive: true });
  const rimMass = (p.mass * share) / n;
  const hubMass = (p.mass * (1 - share)) / 2;
  for (let k = 0; k < n; k++) {
    const [px, py] = ringPoint(n, k, p.radius);
    const x = px * ch - py * sh;
    const y = px * sh + py * ch;
    part.nodes.push({ x, y, z: 0, mass: rimMass, radius: nodeRadius, segment: 0 });
  }
  const half = p.axleLength / 2;
  part.nodes.push({ x: 0, y: 0, z: -half, mass: hubMass, radius: nodeRadius, segment: 0 });
  part.nodes.push({ x: 0, y: 0, z: half, mass: hubMass, radius: nodeRadius, segment: 0 });
  const beam = (a: number, b: number): void => {
    part.beams.push({ a, b, material: p.material, area: p.memberArea, zeta: ZETA_STRUCTURE, ring: false });
  };
  for (let k = 0; k < n; k++) beam(k, (k + 1) % n);
  for (let k = 0; k < n; k++) {
    beam(k, n);
    beam(k, n + 1);
  }
  beam(n, n + 1);
  return part;
}
