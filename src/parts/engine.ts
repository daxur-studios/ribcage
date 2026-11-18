/**
 * Purpose: the engine generator: a bell (a cone frustum from the nozzle ring up to the top ring) with
 * a tail node on the axis at the nozzle and a head node on the axis at the top, both braced to both
 * rings, and a thruster from tail to head. Never-live: once rigid bodies exist the engine is one.
 * Value: the thrust of the hopper goes into the lattice through the head and tail, so the engine is
 * the part where "thrust" meets structure; one pure function from a few numbers to a `Part`.
 *
 * Only `+ − × ÷` and `sqrt` appear in the maths (see `shared.ts`).
 */

import type { MaterialId } from './material.ts';
import type { Part } from './part.ts';
import { DEFAULT_NODE_RADIUS, buildFrustum, isRingCount, requirePositive, RING_COUNTS } from './shared.ts';

export interface EngineParams {
  /** Nodes in each ring: one of 4, 6, 8, 12, 16, 24. */
  ringNodes: number;
  /** Radius of the top ring, where the engine is joined to what is above it, in metres. */
  topRadius: number;
  /** Radius of the nozzle ring in metres. */
  nozzleRadius: number;
  /** Height from the nozzle ring to the top ring in metres. */
  height: number;
  material: MaterialId;
  /** Cross-section area of every member in square metres. */
  memberArea: number;
  /** Stated total mass in kilograms: at least the members' own mass. */
  mass: number;
  /** Maximum thrust in newtons. */
  maxThrust: number;
  /** Collision radius of each node in metres (default 0.05). */
  nodeRadius?: number;
}

/**
 * An engine. Node order: the nozzle ring (0 to n−1), the top ring (n to 2n−1), the tail (2n, on the
 * axis in the nozzle plane) and the head (2n+1, on the axis in the top plane). The thrust acts along
 * tail to head, which is +y. The one attachment ring is `top` (outward axis +y). Refuses bad
 * parameters with the reason, and a stated mass below the members' own.
 */
export function engine(p: EngineParams): Part {
  if (!isRingCount(p.ringNodes)) throw new Error(`an engine ring has ${p.ringNodes} nodes; the counts allowed are ${RING_COUNTS.join(', ')}`);
  requirePositive('an engine top radius', p.topRadius);
  requirePositive('an engine nozzle radius', p.nozzleRadius);
  requirePositive('an engine height', p.height);
  requirePositive('an engine member area', p.memberArea);
  requirePositive('an engine maximum thrust', p.maxThrust);
  const nodeRadius = p.nodeRadius ?? DEFAULT_NODE_RADIUS;
  requirePositive('an engine node radius', nodeRadius);
  const part = buildFrustum({
    type: 'engine',
    count: p.ringNodes,
    lowerRadius: p.nozzleRadius,
    upperRadius: p.topRadius,
    height: p.height,
    attach: 'upper',
    attachId: 'top',
    axisNodes: [
      { y: 0, joins: ['lower', 'upper'] },
      { y: p.height, joins: ['lower', 'upper'] },
    ],
    material: p.material,
    area: p.memberArea,
    nodeRadius,
    mass: p.mass,
    neverLive: true,
  });
  part.thrusters.push({ tail: 2 * p.ringNodes, head: 2 * p.ringNodes + 1, maxThrust: p.maxThrust });
  return part;
}
