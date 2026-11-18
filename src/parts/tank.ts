/**
 * Purpose: the tank generator: a round tube on the y axis from radius, length, ring node count,
 * number of rings, material, wall area and stated mass, with its ring, longitudinal and crossed
 * diagonal members, panels, end attachment rings, side mount patches and one segment per bay.
 * Value: a tank is the body of the hopper and of every later rocket; one pure function from a few
 * numbers to a `Part`, with exact ring points, so a saved craft rebuilds to the same bits.
 *
 * Only `+ − × ÷` and `sqrt` appear in the maths (see `shared.ts`).
 */

import type { MaterialId } from './material.ts';
import type { Part } from './part.ts';
import {
  DEFAULT_NODE_RADIUS,
  buildTube,
  isRingCount,
  requirePositive,
  ringPoint,
  wholeAtLeast,
  RING_COUNTS,
} from './shared.ts';

export interface TankParams {
  /** Ring radius in metres. */
  radius: number;
  /** Length from the bottom ring to the top ring in metres. */
  length: number;
  /** Nodes in each ring: one of 4, 6, 8, 12, 16, 24. */
  ringNodes: number;
  /** Number of rings, at least 2 (one bay between each two). */
  rings: number;
  material: MaterialId;
  /** Cross-section area of each wall member in square metres. */
  wallArea: number;
  /** Stated total mass in kilograms: at least the members' own mass. */
  mass: number;
  /** Collision radius of each node in metres (default 0.05). */
  nodeRadius?: number;
}

/**
 * The in-ring members of an `n`-gon, so a ring keeps its shape in its plane: the perimeter, the
 * chords that skip one node (a closed strip of triangles, which is rigid), for 12 nodes and more
 * the quarter chords (skip `n / 4`), and the diameters. Each pair once, in a fixed order.
 */
export function ringBracing(n: number): [number, number][] {
  const pairs: [number, number][] = [];
  const seen = new Set<number>();
  const add = (a: number, b: number): void => {
    const lo = Math.min(a, b);
    const hi = Math.max(a, b);
    const key = lo * n + hi;
    if (seen.has(key)) return;
    seen.add(key);
    pairs.push([a, b]);
  };
  for (let j = 0; j < n; j++) add(j, (j + 1) % n);
  for (let j = 0; j < n; j++) add(j, (j + 2) % n);
  // From 12 nodes on, the skip-one chords make near-flat triangles (150° and more) and the ring
  // can buckle under the tank's own weight; quarter chords (skip n / 4) make right triangles with
  // the diameters and keep it stiff (ER 004, phase 4.2).
  if (n >= 12) for (let j = 0; j < n; j++) add(j, (j + n / 4) % n);
  for (let j = 0; j < n / 2; j++) add(j, j + n / 2);
  return pairs;
}

/** A tank. Refuses bad parameters with the reason, and a stated mass below the members' own. */
export function tank(p: TankParams): Part {
  requirePositive('a tank radius', p.radius);
  requirePositive('a tank length', p.length);
  requirePositive('a tank wall area', p.wallArea);
  wholeAtLeast('the number of tank rings', p.rings, 2);
  if (!isRingCount(p.ringNodes)) throw new Error(`a tank ring has ${p.ringNodes} nodes; the counts allowed are ${RING_COUNTS.join(', ')}`);
  const nodeRadius = p.nodeRadius ?? DEFAULT_NODE_RADIUS;
  requirePositive('a tank node radius', nodeRadius);
  const section: [number, number][] = [];
  for (let j = 0; j < p.ringNodes; j++) {
    const [x, z] = ringPoint(p.ringNodes, j, p.radius);
    section.push([x, z]);
  }
  return buildTube({
    type: 'tank',
    section,
    sectionRadius: p.radius,
    rings: p.rings,
    spacing: p.length / (p.rings - 1),
    ringBeams: ringBracing(p.ringNodes),
    material: p.material,
    area: p.wallArea,
    nodeRadius,
    mass: p.mass,
  });
}
