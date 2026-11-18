/**
 * Purpose: the pod generator: a shell (a cone frustum from the bottom ring up to a smaller top ring)
 * closed by an apex node on the axis above the top ring, braced to every node of the top ring.
 * Never-live: once rigid bodies exist the pod is one.
 * Value: the pod is the top of the hopper and the part the player is "in"; one pure function from a
 * few numbers to a `Part`, with a shell panel on every wall quad and on every facet of the nose.
 *
 * Only `+ − × ÷` and `sqrt` appear in the maths (see `shared.ts`).
 */

import type { MaterialId } from './material.ts';
import type { Part } from './part.ts';
import { DEFAULT_NODE_RADIUS, buildFrustum, isRingCount, requirePositive, RING_COUNTS } from './shared.ts';

export interface PodParams {
  /** Nodes in each ring: one of 4, 6, 8, 12, 16, 24. */
  ringNodes: number;
  /** Radius of the bottom ring, where the pod is joined to what is below it, in metres. */
  bottomRadius: number;
  /** Radius of the top ring in metres. */
  topRadius: number;
  /** Height from the bottom ring to the top ring in metres. */
  height: number;
  /** Height of the apex above the top ring in metres (default: half the top radius). */
  apexRise?: number;
  material: MaterialId;
  /** Cross-section area of every member in square metres. */
  memberArea: number;
  /** Stated total mass in kilograms: at least the members' own mass. */
  mass: number;
  /** Collision radius of each node in metres (default 0.05). */
  nodeRadius?: number;
}

/**
 * A pod. Node order: the bottom ring (0 to n−1), the top ring (n to 2n−1), the apex (2n). The one
 * attachment ring is `bottom` (outward axis −y). Panels: one cell on every wall quad, and one on every
 * facet of the nose (apex and two neighbours on the top ring), each with three tetrahedra. Refuses
 * bad parameters with the reason, and a stated mass below the members' own.
 */
export function pod(p: PodParams): Part {
  if (!isRingCount(p.ringNodes)) throw new Error(`a pod ring has ${p.ringNodes} nodes; the counts allowed are ${RING_COUNTS.join(', ')}`);
  requirePositive('a pod bottom radius', p.bottomRadius);
  requirePositive('a pod top radius', p.topRadius);
  requirePositive('a pod height', p.height);
  requirePositive('a pod member area', p.memberArea);
  const rise = p.apexRise ?? p.topRadius / 2;
  requirePositive('a pod apex rise (a flat nose would be a mechanism)', rise);
  const nodeRadius = p.nodeRadius ?? DEFAULT_NODE_RADIUS;
  requirePositive('a pod node radius', nodeRadius);
  const n = p.ringNodes;
  const part = buildFrustum({
    type: 'pod',
    count: p.ringNodes,
    lowerRadius: p.bottomRadius,
    upperRadius: p.topRadius,
    height: p.height,
    attach: 'lower',
    attachId: 'bottom',
    axisNodes: [{ y: p.height + rise, joins: ['upper'] }],
    material: p.material,
    area: p.memberArea,
    nodeRadius,
    mass: p.mass,
    neverLive: true,
  });
  // The nose: a facet is the apex and two neighbours on the top ring; the fourth corner of each
  // tetrahedron is the bottom ring node across the axis, far from the facet's plane.
  const apex = 2 * n;
  const off = Math.max(2, n >> 1);
  for (let j = 0; j < n; j++) {
    const a = n + j;
    const b = n + ((j + 1) % n);
    const x = (j + off) % n;
    part.panels.push({ tets: [[apex, a, b, x], [a, apex, b, x], [b, a, apex, x]] });
  }
  return part;
}
