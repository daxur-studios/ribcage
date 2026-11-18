/**
 * Purpose: the truss generator: a square or triangular tube on the y axis from length, width, bays,
 * material and member area, with longitudinal members, crossed diagonals in every wall quad, ring
 * members, panels, end attachment rings, side mount patches and one segment per bay.
 * Value: trusses are the light structure between tanks and the ground; same pure-function shape and
 * the same exact points as the tank, so the assembler treats them alike.
 *
 * Only `+ − × ÷` and `sqrt` appear in the maths (see `shared.ts`).
 */

import type { MaterialId } from './material.ts';
import type { Part } from './part.ts';
import { DEFAULT_NODE_RADIUS, buildTube, requirePositive, unitCosSin, wholeAtLeast } from './shared.ts';

export interface TrussParams {
  section: 'square' | 'triangle';
  /** Length from the bottom ring to the top ring in metres. */
  length: number;
  /** Side of the square, or side of the equilateral triangle, in metres. */
  width: number;
  /** Number of bays, at least 1. */
  bays: number;
  material: MaterialId;
  /** Cross-section area of every member in square metres. */
  memberArea: number;
  /** Stated total mass in kilograms, at least the members' own; leave out for the members' own mass. */
  mass?: number;
  /** Collision radius of each node in metres (default 0.05). */
  nodeRadius?: number;
}

/**
 * A truss. The square's corners are at `(±w/2, ±w/2)`; the triangle's are at 90°, 210° and 330° on
 * a circle of radius `w / sqrt(3)`, so its sides are `w`. A triangle ring has three nodes, so it can
 * only be joined to another truss's triangle ring (rings join at equal node counts). Refuses bad
 * parameters with the reason, and a stated mass below the members' own.
 */
export function truss(p: TrussParams): Part {
  if (p.section !== 'square' && p.section !== 'triangle') throw new Error(`a truss section is "square" or "triangle", not "${String(p.section)}"`);
  requirePositive('a truss length', p.length);
  requirePositive('a truss width', p.width);
  requirePositive('a truss member area', p.memberArea);
  wholeAtLeast('the number of truss bays', p.bays, 1);
  const nodeRadius = p.nodeRadius ?? DEFAULT_NODE_RADIUS;
  requirePositive('a truss node radius', nodeRadius);
  const section: [number, number][] = [];
  let sectionRadius: number;
  let ringBeams: [number, number][];
  if (p.section === 'square') {
    const h = p.width / 2;
    section.push([h, h], [-h, h], [-h, -h], [h, -h]);
    sectionRadius = Math.sqrt(h * h + h * h);
    ringBeams = [[0, 1], [1, 2], [2, 3], [3, 0], [0, 2], [1, 3]];
  } else {
    sectionRadius = p.width / Math.sqrt(3);
    for (let j = 0; j < 3; j++) {
      const [c, s] = unitCosSin(12 + 16 * j);
      section.push([sectionRadius * c + 0, sectionRadius * s + 0]);
    }
    ringBeams = [[0, 1], [1, 2], [2, 0]];
  }
  return buildTube({
    type: 'truss',
    section,
    sectionRadius,
    rings: p.bays + 1,
    spacing: p.length / p.bays,
    ringBeams,
    material: p.material,
    area: p.memberArea,
    nodeRadius,
    mass: p.mass ?? null,
  });
}
