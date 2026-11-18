/**
 * Purpose: height-map terrains for the milestone 3 scenes, built with arithmetic only, and the
 * commands that load one into a world.
 * Value: `+ - * /` and `sqrt` give the same bits in every engine, so the same terrain (and so the
 * same recording) comes out in Node, a browser and the native core; there is no `sin`, `cos` or
 * `Math.pow` here on purpose.
 *
 * A terrain is the grid the core takes (`terrain::Terrain` in core/src/terrain.rs): `nx` by `nz`
 * points `cell` metres apart from `(x0, z0)`, heights `heights[i + j * nx]`, two triangles per
 * cell split from `(i, j)` to `(i + 1, j + 1)`. `heightAt` reads it the same way the core does,
 * so a test can ask where the ground is.
 */

import { MAX_TERRAIN_GRID, type CommandBatch } from './commands.ts';

export interface Terrain {
  nx: number;
  nz: number;
  x0: number;
  z0: number;
  cell: number;
  friction: number;
  heights: Float64Array;
}

/** Dry rock or concrete (ER 003: the default coefficient). */
export const DEFAULT_FRICTION = 0.8;

/** A flat terrain at height zero. */
export function flatTerrain(nx: number, nz: number, x0: number, z0: number, cell: number, friction = DEFAULT_FRICTION): Terrain {
  if (nx < 2 || nz < 2 || nx > MAX_TERRAIN_GRID || nz > MAX_TERRAIN_GRID) throw new Error(`terrain grid ${nx} by ${nz} is not 2..${MAX_TERRAIN_GRID}`);
  return { nx, nz, x0, z0, cell, friction, heights: new Float64Array(nx * nz) };
}

/** Appends the commands that set this terrain: the grid, then every height. */
export function compileTerrain(batch: CommandBatch, t: Terrain): void {
  batch.setTerrain(t.nx, t.nz, t.x0, t.z0, t.cell, t.friction);
  batch.allTerrainHeights(t.heights);
}

/**
 * Ground height at `(x, z)`, by the core's rule: the triangle under the point, exact, and the
 * edge heights continuing flat past the grid.
 */
export function heightAt(t: Terrain, x: number, z: number): number {
  const last = [t.nx - 1, t.nz - 1];
  const axis = (coord: number, origin: number, points: number): { cell: number; frac: number } => {
    let u = (coord - origin) / t.cell;
    if (!(u >= 0)) u = 0;
    else if (u > points) u = points;
    let i = Math.floor(u);
    if (i > points - 1) i = points - 1;
    return { cell: i, frac: u - i };
  };
  const ax = axis(x, t.x0, last[0]!);
  const az = axis(z, t.z0, last[1]!);
  const at = (i: number, j: number): number => t.heights[i + j * t.nx]!;
  const { cell: i, frac: fx } = ax;
  const { cell: j, frac: fz } = az;
  const h00 = at(i, j);
  const h10 = at(i + 1, j);
  const h01 = at(i, j + 1);
  const h11 = at(i + 1, j + 1);
  return fx >= fz ? h00 + fx * (h10 - h00) + fz * (h11 - h10) : h00 + fx * (h11 - h01) + fz * (h01 - h00);
}

/** A round hill: height `h` at `(cx, cz)`, falling smoothly to zero at distance `r` and beyond. */
export function bump(x: number, z: number, cx: number, cz: number, r: number, h: number): number {
  const d2 = ((x - cx) * (x - cx) + (z - cz) * (z - cz)) / (r * r);
  if (d2 >= 1) return 0;
  const s = 1 - d2;
  return h * s * s;
}

/** 0 at `t <= 0`, 1 at `t >= 1`, the cubic smoothstep between (flat at both ends). */
export function smoothstep(t: number): number {
  if (t <= 0) return 0;
  if (t >= 1) return 1;
  return t * t * (3 - 2 * t);
}

function fill(t: Terrain, f: (x: number, z: number) => number): Terrain {
  for (let j = 0; j < t.nz; j++) {
    for (let i = 0; i < t.nx; i++) t.heights[i + j * t.nx] = f(t.x0 + i * t.cell, t.z0 + j * t.cell);
  }
  return t;
}

/** Half the side, in metres, of the dead-flat square at the middle of the plain. */
export const PLAIN_FLAT_HALF = 4;

/**
 * A plain for the tower: 41 by 41 points, 1 m apart, from -20 to 20 on both axes. Flat at height
 * zero within `PLAIN_FLAT_HALF` of the origin, gently bumpy beyond (a few hills up to 0.8 m).
 */
export function plainTerrain(): Terrain {
  const hills: [number, number, number, number][] = [
    [11, 4, 5, 0.6],
    [-10, -9, 6, 0.8],
    [4, -13, 5, 0.5],
    [-12, 10, 5, 0.7],
    [14, -12, 4, 0.4],
  ];
  return fill(flatTerrain(41, 41, -20, -20, 1), (x, z) => {
    let h = 0;
    for (const [cx, cz, r, hh] of hills) h += bump(x, z, cx, cz, r, hh);
    return h;
  });
}

/** Where the valley's two ledges end and its sides start, along x. Both ledges are at height zero. */
export const VALLEY_LEDGE_LEFT = 2;
export const VALLEY_LEDGE_RIGHT = 22;
/** Width of each side slope and depth of the valley floor, in metres. */
export const VALLEY_SLOPE = 6;
export const VALLEY_DEPTH = 5;

/**
 * A valley for the bridge: 61 by 31 points, 1 m apart, from `x = -12` to 48 and `z = -15` to 15.
 * Two flat ledges at height zero (`x <= 2` and `x >= 22`, over the whole width), smooth sides
 * `VALLEY_SLOPE` wide down to a floor `VALLEY_DEPTH` below, with a few bumps on the floor.
 */
export function valleyTerrain(): Terrain {
  const floorBumps: [number, number, number, number][] = [
    [11, -3, 3, 0.5],
    [13.5, 4, 3.5, 0.7],
    [9.5, 7, 3, 0.4],
    [14.5, -8, 3, 0.5],
  ];
  return fill(flatTerrain(61, 31, -12, -15, 1), (x, z) => {
    const left = smoothstep((x - VALLEY_LEDGE_LEFT) / VALLEY_SLOPE);
    const right = smoothstep((VALLEY_LEDGE_RIGHT - x) / VALLEY_SLOPE);
    const floor = left < right ? left : right;
    let h = -VALLEY_DEPTH * floor;
    for (const [cx, cz, r, hh] of floorBumps) h += bump(x, z, cx, cz, r, hh);
    return h;
  });
}

/** Half the side, in metres, of the dead-flat landing pad at the middle of the landing field. */
export const LANDING_PAD_HALF = 6;
/** The rough area of the landing field: bumps are placed inside this box (x from, x to, z from, z to). */
export const ROUGH_AREA = { x0: 7.5, x1: 14.5, z0: -8, z1: 8 } as const;
/** How many bumps the rough area has. */
const ROUGH_BUMPS = 48;
/** Bump heights run from `ROUGH_HEIGHT_MIN` up by up to `ROUGH_HEIGHT_SPAN`, in metres. */
const ROUGH_HEIGHT_MIN = 0.05;
const ROUGH_HEIGHT_SPAN = 0.1;

/**
 * The landing field for the craft tests: 65 by 65 points, 0.5 m apart, from -16 to 16 on both
 * axes. Dead flat at height zero within `LANDING_PAD_HALF` of the origin (a square pad). Beside it,
 * inside `ROUGH_AREA`, `ROUGH_BUMPS` round bumps 0.05 to 0.15 m high and 1.6 to 3.2 m across (3 to 6 cells), put
 * where a small integer generator says (they overlap, so the ground reaches 0.3 m), so the same field comes out every time and a foot
 * touching down there meets a slope of any aspect.
 */
export function landingField(): Terrain {
  const bumps: [number, number, number, number][] = [];
  let seed = 20260;
  const next = (): number => {
    seed = (seed * 1664525 + 1013904223) % 4294967296;
    return seed / 4294967296;
  };
  for (let k = 0; k < ROUGH_BUMPS; k++) {
    const x = ROUGH_AREA.x0 + (ROUGH_AREA.x1 - ROUGH_AREA.x0) * next();
    const z = ROUGH_AREA.z0 + (ROUGH_AREA.z1 - ROUGH_AREA.z0) * next();
    const r = 0.8 + 0.8 * next();
    const h = ROUGH_HEIGHT_MIN + ROUGH_HEIGHT_SPAN * next();
    bumps.push([x, z, r, h]);
  }
  return fill(flatTerrain(65, 65, -16, -16, 0.5), (x, z) => {
    let h = 0;
    for (const [cx, cz, r, hh] of bumps) h += bump(x, z, cx, cz, r, hh);
    return h;
  });
}
