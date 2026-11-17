/**
 * Purpose: a structure as the editor holds it before it runs: nodes at points, beams between
 * them, and a few nodes pinned. `compileDesign` turns it into command records (layout rule 8),
 * so the page, the headless tests and a recording all start a world the same way.
 * Value: the editor edits plain data, never the running world, so "stop" is just dropping the
 * world and nothing needs undoing; the load that rolls across is defined here too, for any deck.
 *
 * Planar: every node is at z = 0 and every force is in the plane.
 */

import type { CommandBatch } from './commands.ts';

/** Standard gravity, for the weight of the rolling load. */
export const GRAVITY = 9.81;

/** Fixed step in seconds (60 Hz). */
export const STEP_SECONDS = 1 / 60;

export interface DesignNode {
  x: number;
  y: number;
  /** A pinned node has zero inverse mass and never moves. */
  pinned: boolean;
}

export interface DesignBeam {
  a: number;
  b: number;
}

export interface Design {
  nodes: DesignNode[];
  beams: DesignBeam[];
}

/** What every node and beam of a design is made of (one material for the whole test bed). */
export interface Material {
  /** Mass of every free node in kilograms. */
  nodeMass: number;
  /** Axial stiffness `EA` of every beam in newtons: compliance is `rest / EA`. */
  ea: number;
  /** Beam damping in newton-seconds per metre. */
  damping: number;
  yieldStrain: number;
  breakStrain: number;
  /** Node radius in metres (drawing and, later, contact). */
  radius: number;
}

/**
 * Appends the commands that build `design` to `batch`: every node in order, then every beam in
 * order. Applied to an **empty** world, node `i` of the design is node `i` of the world and beam
 * `j` is beam `j`; the editor and the truck rely on that.
 */
export function compileDesign(batch: CommandBatch, design: Design, material: Material): void {
  const invMass = 1 / material.nodeMass;
  for (const n of design.nodes) batch.addNode(n.x, n.y, 0, n.pinned ? 0 : invMass, material.radius);
  for (const b of design.beams) {
    const p = design.nodes[b.a]!;
    const q = design.nodes[b.b]!;
    // sqrt is exactly rounded everywhere; Math.hypot is not, and engines differ.
    const rest = Math.sqrt((q.x - p.x) * (q.x - p.x) + (q.y - p.y) * (q.y - p.y));
    batch.addBeam(b.a, b.b, {
      rest,
      compliance: rest / material.ea,
      damping: material.damping,
      yieldStrain: material.yieldStrain,
      breakStrain: material.breakStrain,
    });
  }
}

export interface Truck {
  /** Mass in kilograms (only its weight acts on the bridge: a point load, no inertia). */
  mass: number;
  /** Speed in metres per second along the deck. */
  speed: number;
}

export const STANDARD_TRUCK: Truck = { mass: 3000, speed: 4 };

/** The nodes a load rolls along, left to right. */
export interface Deck {
  ids: number[];
  xs: number[];
}

/**
 * The deck of a design: every node at the height of the leftmost pinned node (the supports),
 * ordered by x. Empty when nothing is pinned.
 */
export function deckOf(design: Design): Deck {
  let level: DesignNode | undefined;
  for (const n of design.nodes) {
    if (n.pinned && (level === undefined || n.x < level.x)) level = n;
  }
  if (level === undefined) return { ids: [], xs: [] };
  const ids: number[] = [];
  for (let i = 0; i < design.nodes.length; i++) if (design.nodes[i]!.y === level.y) ids.push(i);
  ids.sort((a, b) => design.nodes[a]!.x - design.nodes[b]!.x || a - b);
  return { ids, xs: ids.map((i) => design.nodes[i]!.x) };
}

/** Where the truck is, in metres from the start of the deck, at the start of step `step` (0-based). */
export function truckPosition(truck: Truck, step: number): number {
  return truck.speed * step * STEP_SECONDS;
}

/**
 * Appends the force commands for the truck's weight during step `step`: split between the two
 * deck nodes either side of it (a simply supported deck between nodes). Nothing once it has left
 * the deck. Returns whether the truck is still on it.
 */
export function rollLoad(batch: CommandBatch, deck: Deck, truck: Truck, step: number): boolean {
  const at = locateTruck(deck, truck, step);
  if (at === null) return false;
  const w = truck.mass * GRAVITY;
  batch.addForce(deck.ids[at.bay]!, 0, -w * (1 - at.t), 0);
  batch.addForce(deck.ids[at.bay + 1]!, 0, -w * at.t, 0);
  return true;
}

/**
 * Which stretch of the deck the truck is over at the start of step `step`: the index of the deck
 * node on its left and how far along to the next (0 to 1). `null` when it is not on the deck.
 */
export function locateTruck(deck: Deck, truck: Truck, step: number): { bay: number; t: number } | null {
  const last = deck.ids.length - 1;
  if (last < 1) return null;
  const x = deck.xs[0]! + truckPosition(truck, step);
  if (x < deck.xs[0]! || x > deck.xs[last]!) return null;
  let bay = 0;
  while (bay + 1 < last && deck.xs[bay + 1]! <= x) bay++;
  const x0 = deck.xs[bay]!;
  return { bay, t: (x - x0) / (deck.xs[bay + 1]! - x0) }; // 0 at the left node, 1 at the right
}

/** Seconds the truck needs to cross the whole deck. */
export function crossingSeconds(deck: Deck, truck: Truck): number {
  const last = deck.ids.length - 1;
  return last < 1 ? 0 : (deck.xs[last]! - deck.xs[0]!) / truck.speed;
}
