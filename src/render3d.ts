/**
 * Purpose: the plain-number geometry the 3D page needs, with no Three.js in it: where a beam's
 * cylinder goes between its two end points, and the triangles of a terrain.
 * Value: the page only copies these numbers into Three.js objects, so the maths is tested in
 * Node, and the terrain is drawn from the very triangles the core collides with.
 */

import type { Terrain } from './terrain.ts';

/** A cylinder along +Y of length 1 and radius 1, moved to span two points: where it sits, how it turns, how big it is. */
export interface BeamPlacement {
  /** Middle of the beam. */
  x: number;
  y: number;
  z: number;
  /** Unit quaternion turning +Y onto the direction from end a to end b. */
  qx: number;
  qy: number;
  qz: number;
  qw: number;
  /** Distance between the ends. */
  length: number;
}

/** Placement of a cylinder between `a` and `b` (each three numbers, read from `p` at `a * 3` and `b * 3`). */
export function placeBeam(p: ArrayLike<number>, a: number, b: number, out: BeamPlacement): BeamPlacement {
  const ax = p[a * 3]!;
  const ay = p[a * 3 + 1]!;
  const az = p[a * 3 + 2]!;
  const dx = p[b * 3]! - ax;
  const dy = p[b * 3 + 1]! - ay;
  const dz = p[b * 3 + 2]! - az;
  const length = Math.sqrt(dx * dx + dy * dy + dz * dz);
  out.x = ax + dx / 2;
  out.y = ay + dy / 2;
  out.z = az + dz / 2;
  out.length = length;
  if (length < 1e-9) {
    out.qx = 0;
    out.qy = 0;
    out.qz = 0;
    out.qw = 1;
    return out;
  }
  // Rotation taking (0, 1, 0) to the unit direction (ux, uy, uz): axis (uz, 0, -ux) by half-angle
  // formulas, q = (cross(y, u), 1 + dot(y, u)), then normalised.
  const ux = dx / length;
  const uy = dy / length;
  const uz = dz / length;
  const qx = uz;
  const qz = -ux;
  // `1 + uy` loses its digits when the beam points nearly straight down; this is the same number.
  const qw = uy >= 0 ? 1 + uy : (ux * ux + uz * uz) / (1 - uy);
  const n = Math.sqrt(qx * qx + qz * qz + qw * qw);
  if (n < 1e-12) {
    // Pointing exactly down: a half turn about x.
    out.qx = 1;
    out.qy = 0;
    out.qz = 0;
    out.qw = 0;
    return out;
  }
  out.qx = qx / n;
  out.qy = 0;
  out.qz = qz / n;
  out.qw = qw / n;
  return out;
}

export function newPlacement(): BeamPlacement {
  return { x: 0, y: 0, z: 0, qx: 0, qy: 0, qz: 0, qw: 1, length: 0 };
}

/** Rotates `(vx, vy, vz)` by the placement's quaternion (for tests and picking). */
export function rotateBy(q: BeamPlacement, vx: number, vy: number, vz: number): [number, number, number] {
  // v' = v + 2w (q × v) + 2 q × (q × v)
  const tx = 2 * (q.qy * vz - q.qz * vy);
  const ty = 2 * (q.qz * vx - q.qx * vz);
  const tz = 2 * (q.qx * vy - q.qy * vx);
  return [
    vx + q.qw * tx + (q.qy * tz - q.qz * ty),
    vy + q.qw * ty + (q.qz * tx - q.qx * tz),
    vz + q.qw * tz + (q.qx * ty - q.qy * tx),
  ];
}

/**
 * Node positions part of the way between two render buffers: `out = prev + alpha * (curr - prev)`.
 * Drawing only: the core is stepped by whole fixed steps, never by frame time. A node that has the
 * same place in both stays exactly there; `alpha` outside 0 to 1 is held to it.
 */
export function lerpPositions(prev: ArrayLike<number>, curr: ArrayLike<number>, alpha: number, out: Float32Array | Float64Array): void {
  const a = alpha < 0 ? 0 : alpha > 1 ? 1 : alpha;
  const n = Math.min(curr.length, out.length);
  for (let i = 0; i < n; i++) {
    const c = curr[i]!;
    const p = i < prev.length ? prev[i]! : c;
    out[i] = p + a * (c - p);
  }
}

/**
 * The nearest live node whose sphere of `radius` the ray from `(ox, oy, oz)` along the unit
 * direction `(dx, dy, dz)` hits in front of its origin, or -1. `alive` says which slots hold a node
 * (bit 0 of the core's node flags).
 */
export function pickNode(
  positions: ArrayLike<number>,
  alive: ArrayLike<number>,
  radius: number,
  ox: number,
  oy: number,
  oz: number,
  dx: number,
  dy: number,
  dz: number,
): number {
  let best = -1;
  let bestT = Infinity;
  for (let i = 0; i < alive.length; i++) {
    if (!(alive[i]! & 1)) continue;
    const cx = positions[i * 3]! - ox;
    const cy = positions[i * 3 + 1]! - oy;
    const cz = positions[i * 3 + 2]! - oz;
    const along = cx * dx + cy * dy + cz * dz; // distance to the closest approach
    const off2 = cx * cx + cy * cy + cz * cz - along * along; // squared distance from the ray's line
    const inside = radius * radius - off2;
    if (inside < 0) continue;
    const t = along - Math.sqrt(inside); // where the ray enters the sphere
    const hit = t >= 0 ? t : along + Math.sqrt(inside) >= 0 ? 0 : -1; // origin inside: hit at once
    if (hit >= 0 && hit < bestT) {
      bestT = hit;
      best = i;
    }
  }
  return best;
}

/**
 * The terrain as triangles: one vertex per grid point (`x`, height, `z`) and two triangles per
 * cell, split along the diagonal from `(i, j)` to `(i + 1, j + 1)` as the core splits them, wound
 * so the normal points up (+y).
 */
export function terrainTriangles(t: Terrain): { positions: Float32Array; indices: Uint32Array } {
  const positions = new Float32Array(t.nx * t.nz * 3);
  for (let j = 0; j < t.nz; j++) {
    for (let i = 0; i < t.nx; i++) {
      const k = (i + j * t.nx) * 3;
      positions[k] = t.x0 + i * t.cell;
      positions[k + 1] = t.heights[i + j * t.nx]!;
      positions[k + 2] = t.z0 + j * t.cell;
    }
  }
  const indices = new Uint32Array((t.nx - 1) * (t.nz - 1) * 6);
  let n = 0;
  for (let j = 0; j + 1 < t.nz; j++) {
    for (let i = 0; i + 1 < t.nx; i++) {
      const v00 = i + j * t.nx;
      const v10 = v00 + 1;
      const v01 = v00 + t.nx;
      const v11 = v01 + 1;
      // The triangle where x runs at least as far as z (the core's `fx >= fz`), then the other.
      indices[n++] = v00;
      indices[n++] = v11;
      indices[n++] = v10;
      indices[n++] = v00;
      indices[n++] = v01;
      indices[n++] = v11;
    }
  }
  return { positions, indices };
}
