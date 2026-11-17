/**
 * Purpose: the two milestone 3 scenes, a tower on a plain and a bridge across a valley, and the
 * actions that bring each down, all as command lists (like `design.ts` and `bridge.ts`).
 * Value: the page (3.5, 3.6) and the headless tests (test/scenes3d.test.ts) start the very same
 * worlds and push them the very same ways, and a recording of either is just the lists.
 *
 * Nothing is pinned. Feet and bridge ends rest on the terrain and are held by weight and
 * friction (ER 003), so a knocked tower can tip and slide.
 *
 * Numbers are **[assumption]**s of a test bed, as in milestone 2: the strong member of
 * `bridge.ts` (`EA` 2e8 N, 100 kg a node, node radius 0.15 m) everywhere.
 */

import { STRONG_BRIDGE } from './bridge.ts';
import type { CommandBatch } from './commands.ts';
import { GRAVITY, locateTruck, type Deck, type Material, type Truck } from './design.ts';
import { compileTerrain, plainTerrain, valleyTerrain, type Terrain } from './terrain.ts';

/** The member every beam of both scenes is made of. */
export const MEMBER: Material = STRONG_BRIDGE;

/** A node of a 3D structure before it is built. */
export interface Point3 {
  x: number;
  y: number;
  z: number;
}

export interface Struct3 {
  nodes: Point3[];
  beams: { a: number; b: number }[];
}

/**
 * Appends the commands that build `s`: every node in order, then every beam in order. Applied
 * after the terrain to an otherwise empty world, node `i` is node `i` and beam `j` is beam `j`.
 */
export function compileStruct(batch: CommandBatch, s: Struct3, m: Material = MEMBER): void {
  const invMass = 1 / m.nodeMass;
  for (const n of s.nodes) batch.addNode(n.x, n.y, n.z, invMass, m.radius);
  for (const b of s.beams) {
    const p = s.nodes[b.a]!;
    const q = s.nodes[b.b]!;
    // sqrt is exactly rounded everywhere; Math.hypot is not, and engines differ.
    const rest = Math.sqrt((q.x - p.x) * (q.x - p.x) + (q.y - p.y) * (q.y - p.y) + (q.z - p.z) * (q.z - p.z));
    batch.addBeam(b.a, b.b, {
      rest,
      compliance: rest / m.ea,
      damping: m.damping,
      yieldStrain: m.yieldStrain,
      breakStrain: m.breakStrain,
    });
  }
}

/** The ways to bring a scene down. */
export type ActionKind = 'knock' | 'removeColumn' | 'overload';

/**
 * When an action starts, in steps. The scenes start at rest on the ground; a short settle first
 * lets them take their own weight before anything happens.
 */
export const ACTION_START_STEP = 120;

// ------------------------------------------------------------------------------------------
// Tower

export interface TowerSpec {
  /** Storeys, each a cube-like box of `storey` metres. */
  storeys: number;
  /** Height of one storey in metres. */
  storey: number;
  /** Side of the square plan in metres. */
  width: number;
}

/** Six storeys of 3 m on a 4 m square: 18 m tall, a slender tower that can tip. */
export const STANDARD_TOWER: TowerSpec = { storeys: 6, storey: 3, width: 4 };

export interface TowerLayout {
  spec: TowerSpec;
  terrain: Terrain;
  /** Node index of corner `c` (0 to 3 around the plan) at level `l` (0 is the feet): `4 * l + c`. */
  node: (level: number, corner: number) => number;
  /** Beam indices of storey `s`'s column at corner `c`, and the four face diagonals that end on it. */
  column: (storey: number, corner: number) => number[];
  beamCount: number;
  nodeCount: number;
}

/** The tower as a structure: feet on the ground at `y = radius`, corners going round the plan. */
export function towerStruct(spec: TowerSpec): { struct: Struct3; column: TowerLayout['column'] } {
  const { storeys, storey, width } = spec;
  const h = width / 2;
  const sign: [number, number][] = [
    [-1, -1],
    [1, -1],
    [1, 1],
    [-1, 1],
  ];
  const nodes: Point3[] = [];
  for (let l = 0; l <= storeys; l++) {
    for (const [sx, sz] of sign) nodes.push({ x: sx * h, y: MEMBER.radius + l * storey, z: sz * h });
  }
  const id = (l: number, c: number): number => 4 * l + (c & 3);
  const beams: Struct3['beams'] = [];
  const cols: number[][][] = [];
  for (let s = 0; s < storeys; s++) {
    cols.push([[], [], [], []]);
    const own = (c: number, beam: number): void => {
      cols[s]![c & 3]!.push(beam);
    };
    const add = (a: number, b: number): number => beams.push({ a, b }) - 1;
    for (let c = 0; c < 4; c++) own(c, add(id(s, c), id(s + 1, c)));
    // Each face is crossed by two diagonals. Each ends on two corners' columns: low end and high end.
    for (let c = 0; c < 4; c++) {
      const up = add(id(s, c), id(s + 1, c + 1));
      own(c, up);
      own(c + 1, up);
      const down = add(id(s, c + 1), id(s + 1, c));
      own(c, down);
      own(c + 1, down);
    }
  }
  // Floor and ceiling rings, crossed by two diagonals each, at every level including the feet.
  for (let l = 0; l <= storeys; l++) {
    for (let c = 0; c < 4; c++) beams.push({ a: id(l, c), b: id(l, c + 1) });
    beams.push({ a: id(l, 0), b: id(l, 2) });
    beams.push({ a: id(l, 1), b: id(l, 3) });
  }
  const column = (s: number, c: number): number[] => [...cols[s]![c & 3]!];
  return { struct: { nodes, beams }, column };
}

/** Appends the plain's terrain and the tower to `batch` (an empty world). */
export function buildTower(batch: CommandBatch, spec: TowerSpec = STANDARD_TOWER): TowerLayout {
  const terrain = plainTerrain();
  compileTerrain(batch, terrain);
  const { struct, column } = towerStruct(spec);
  compileStruct(batch, struct);
  return { spec, terrain, node: (l, c) => 4 * l + (c & 3), column, beamCount: struct.beams.length, nodeCount: struct.nodes.length };
}

/** Newtons the knock pushes the roof with, along `+x`, spread over its four corners. */
export const TOWER_KNOCK_FORCE = 3e4;
/** Seconds the knock lasts. */
export const TOWER_KNOCK_SECONDS = 0.5;
/** Newtons of roof load the overload reaches, spread over the four roof corners. */
export const TOWER_OVERLOAD_FORCE = 2e6;
/** The overload grows to full in this many steps, is held for `OVERLOAD_HOLD_STEPS`, then goes. */
export const OVERLOAD_RAMP_STEPS = 120;
export const OVERLOAD_HOLD_STEPS = 180;

/**
 * The load's share of its full value at step `k` after the action starts: a ramp up, a hold, then
 * nothing (the load is removed so a collapsed scene can come to rest).
 */
export function overloadShare(k: number): number {
  if (k < 0 || k >= OVERLOAD_RAMP_STEPS + OVERLOAD_HOLD_STEPS) return 0;
  return k >= OVERLOAD_RAMP_STEPS ? 1 : (k + 1) / OVERLOAD_RAMP_STEPS;
}

/**
 * Appends the tower's action commands for step `step`, counted from the start of the run (after
 * the build); nothing happens before `ACTION_START_STEP`.
 *
 * - knock: a sideways push on the roof corners for a short time;
 * - removeColumn: the bottom storey's columns (and the diagonals on them) at two neighbouring
 *   corners are cut, so one side of the tower is left with no feet;
 * - overload: a roof load that ramps up past what the columns carry.
 */
export function towerAction(batch: CommandBatch, layout: TowerLayout, kind: ActionKind, step: number): void {
  if (step < ACTION_START_STEP) return;
  const k = step - ACTION_START_STEP;
  const roof = [0, 1, 2, 3].map((c) => layout.node(layout.spec.storeys, c));
  if (kind === 'knock') {
    if (k < Math.round(TOWER_KNOCK_SECONDS * 60)) for (const n of roof) batch.addForce(n, TOWER_KNOCK_FORCE / 4, 0, 0);
  } else if (kind === 'removeColumn') {
    // Neighbouring corners share their face's diagonals: each beam is deleted once.
    if (k === 0) for (const b of new Set([...layout.column(0, 0), ...layout.column(0, 1)])) batch.deleteBeam(b);
  } else {
    const share = overloadShare(k);
    if (share > 0) for (const n of roof) batch.addForce(n, 0, (-TOWER_OVERLOAD_FORCE * share) / 4, 0);
  }
}

// ------------------------------------------------------------------------------------------
// Bridge

export interface BridgeSpec3 {
  bays: number;
  bayLength: number;
  depth: number;
  /** Distance between the two trusses in metres. */
  width: number;
}

/** Eight bays of 3 m (24 m), 3 m deep, trusses 3 m apart. */
export const STANDARD_BRIDGE3: BridgeSpec3 = { bays: 8, bayLength: 3, depth: 3, width: 3 };

export interface BridgeLayout3 {
  spec: BridgeSpec3;
  terrain: Terrain;
  /** Deck node of truss `side` (0 or 1) at position `i` along the span (0 to `bays`). */
  deck: (side: number, i: number) => number;
  /** Top node of truss `side` over bay `i` (0 to `bays - 1`). */
  top: (side: number, i: number) => number;
  /** Bottom chord beam of truss `side` in bay `i`. */
  chord: (side: number, i: number) => number;
  /** The two diagonals across the deck in bay `i` (they carry the bottom chord's load if it is cut). */
  deckDiagonals: (bay: number) => number[];
  /** The deck line the truck rolls along (truss 0's deck nodes and their x). */
  rail: Deck;
  nodeCount: number;
  beamCount: number;
}

export function bridgeStruct(spec: BridgeSpec3): { struct: Struct3; layout: Omit<BridgeLayout3, 'spec' | 'terrain' | 'rail'> } {
  const { bays, bayLength: L, depth: H, width: W } = spec;
  const r = MEMBER.radius;
  const per = 2 * bays + 1;
  const deck = (s: number, i: number): number => s * per + i;
  const top = (s: number, i: number): number => s * per + bays + 1 + i;
  const nodes: Point3[] = [];
  for (let s = 0; s < 2; s++) {
    const z = (s === 0 ? -1 : 1) * (W / 2);
    for (let i = 0; i <= bays; i++) nodes.push({ x: i * L, y: r, z });
    for (let i = 0; i < bays; i++) nodes.push({ x: (i + 0.5) * L, y: r + H, z });
  }
  const beams: Struct3['beams'] = [];
  const add = (a: number, b: number): number => beams.push({ a, b }) - 1;
  const chords: number[][] = [[], []];
  const diagonals: number[][] = [];
  for (let s = 0; s < 2; s++) {
    for (let i = 0; i < bays; i++) chords[s]!.push(add(deck(s, i), deck(s, i + 1)));
    for (let i = 0; i + 1 < bays; i++) add(top(s, i), top(s, i + 1));
    for (let i = 0; i < bays; i++) {
      add(deck(s, i), top(s, i));
      add(top(s, i), deck(s, i + 1));
    }
  }
  // Across the span: cross beams under the deck and in the roof, crossed diagonals in both
  // planes, and sway braces from each top node to the far truss's deck nodes.
  for (let i = 0; i <= bays; i++) add(deck(0, i), deck(1, i));
  for (let i = 0; i < bays; i++) add(top(0, i), top(1, i));
  for (let i = 0; i < bays; i++) {
    diagonals.push([add(deck(0, i), deck(1, i + 1)), add(deck(1, i), deck(0, i + 1))]);
  }
  for (let i = 0; i + 1 < bays; i++) {
    add(top(0, i), top(1, i + 1));
    add(top(1, i), top(0, i + 1));
  }
  for (let i = 0; i < bays; i++) {
    for (let s = 0; s < 2; s++) {
      add(top(s, i), deck(1 - s, i));
      add(top(s, i), deck(1 - s, i + 1));
    }
  }
  return {
    struct: { nodes, beams },
    layout: { deck, top, chord: (s, i) => chords[s]![i]!, deckDiagonals: (i) => diagonals[i]!, nodeCount: nodes.length, beamCount: beams.length },
  };
}

/** Appends the valley's terrain and the bridge to `batch` (an empty world). */
export function buildBridge3(batch: CommandBatch, spec: BridgeSpec3 = STANDARD_BRIDGE3): BridgeLayout3 {
  const terrain = valleyTerrain();
  compileTerrain(batch, terrain);
  const { struct, layout } = bridgeStruct(spec);
  compileStruct(batch, struct);
  const ids = Array.from({ length: spec.bays + 1 }, (_, i) => layout.deck(0, i));
  const rail: Deck = { ids, xs: ids.map((_, i) => i * spec.bayLength) };
  return { spec, terrain, rail, ...layout };
}

/** A truck weighing 200 t at the standard speed: far past what the bridge carries. */
export const HEAVY_TRUCK: Truck = { mass: 200000, speed: 4 };

/** Newtons of sideways knock on the middle of the deck, along `+z`. */
export const BRIDGE_KNOCK_FORCE = 2e5;
export const BRIDGE_KNOCK_SECONDS = 0.5;

/**
 * Appends the truck's weight for step `step`: split over the four deck nodes around it (two
 * trusses, two nodes along each), as `design.ts`'s `rollLoad` does for one truss. Returns
 * whether the truck is still on the deck.
 */
export function rollTruck3(batch: CommandBatch, layout: BridgeLayout3, truck: Truck, step: number): boolean {
  const at = locateTruck(layout.rail, truck, step);
  if (at === null) return false;
  const w = (truck.mass * GRAVITY) / 2; // per truss
  for (let s = 0; s < 2; s++) {
    batch.addForce(layout.deck(s, at.bay), 0, -w * (1 - at.t), 0);
    batch.addForce(layout.deck(s, at.bay + 1), 0, -w * at.t, 0);
  }
  return true;
}

/**
 * The bridge's commands for step `step`. The `light` truck (the standard one of `bridge.ts`)
 * crosses from step 0 in every run except the overload, where `HEAVY_TRUCK` rolls from
 * `ACTION_START_STEP` instead. `kind` null is the plain crossing.
 *
 * - knock: a sideways push on the middle of the deck;
 * - removeColumn: the middle bay is cut across the bottom (both chords and both deck diagonals);
 * - overload: the heavy truck.
 */
export function bridgeAction(batch: CommandBatch, layout: BridgeLayout3, kind: ActionKind | null, step: number, light: Truck): void {
  const mid = layout.spec.bays >> 1;
  if (kind === 'overload') {
    if (step >= ACTION_START_STEP) rollTruck3(batch, layout, HEAVY_TRUCK, step - ACTION_START_STEP);
    return;
  }
  rollTruck3(batch, layout, light, step);
  if (kind === null || step < ACTION_START_STEP) return;
  const k = step - ACTION_START_STEP;
  if (kind === 'knock') {
    if (k < Math.round(BRIDGE_KNOCK_SECONDS * 60)) {
      for (let s = 0; s < 2; s++) batch.addForce(layout.deck(s, mid), 0, 0, BRIDGE_KNOCK_FORCE / 2);
    }
  } else if (k === 0) {
    for (const b of layout.deckDiagonals(mid)) batch.deleteBeam(b);
    for (let s = 0; s < 2; s++) batch.deleteBeam(layout.chord(s, mid));
  }
}
