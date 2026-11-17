/**
 * Purpose: the bridge scene for milestone 2: a Warren truss between two pinned supports, and a
 * load that rolls across it. Both are command lists, so the same scene runs headlessly in tests
 * and in the editor, and a recording of it is just the lists.
 * Value: the first structure that has to carry a moving load with the real solver; "a strong
 * bridge holds and an under-built bridge breaks" is the milestone's check.
 *
 * The truss is planar: every node has z = 0 and every force is in the plane, which keeps z
 * exactly 0 (checked in test/bridge.test.ts).
 *
 * Numbers are **[assumption]** until the editor exists: a 24 m span of eight 3 m bays, 3 m deep,
 * 100 kg per node, chosen so a strong bridge uses about 20 substeps (the rule's cap is 32) and
 * stays cheap enough to run many times in a test.
 */

import type { CommandBatch } from './commands.ts';

/** Standard gravity, for the weight of the rolling load. */
export const GRAVITY = 9.81;

/** Fixed step in seconds (60 Hz). */
export const STEP_SECONDS = 1 / 60;

export interface BridgeSpec {
  /** Bays along the span. */
  bays: number;
  /** Length of one bay in metres. */
  bayLength: number;
  /** Truss depth in metres. */
  depth: number;
  /** Mass of every node in kilograms. */
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

/** A bridge that carries the standard truck with margin. */
export const STRONG_BRIDGE: BridgeSpec = {
  bays: 8,
  bayLength: 3,
  depth: 3,
  nodeMass: 100,
  ea: 2e8,
  damping: 5e3,
  yieldStrain: 0.00125,
  breakStrain: 0.01,
  radius: 0.15,
};

/** The same bridge with beams a tenth as stiff and strong (2e7 against 2e8): yields under the truck and breaks. */
export const WEAK_BRIDGE: BridgeSpec = { ...STRONG_BRIDGE, ea: 2e7 };

export interface Truck {
  /** Mass in kilograms (only its weight acts on the bridge: a point load, no inertia). */
  mass: number;
  /** Speed in metres per second along the deck. */
  speed: number;
}

export const STANDARD_TRUCK: Truck = { mass: 3000, speed: 4 };

export interface BridgeLayout {
  spec: BridgeSpec;
  /** Deck nodes (the bottom chord), left to right. The first and last are the pinned supports. */
  deck: number[];
  /** Top chord nodes, left to right (one per bay, over the middle of the bay). */
  top: number[];
  /** Every beam, in the order they were added. */
  beams: number[];
  /** Span in metres. */
  span: number;
}

/**
 * Appends the commands that build the bridge to `batch`. The node and beam indices are those the
 * world will hand out when the batch is applied to an **empty** world, in order; `applyBridge`
 * in the tests checks that against the results.
 */
export function buildBridge(batch: CommandBatch, spec: BridgeSpec): BridgeLayout {
  const { bays, bayLength: L, depth: H } = spec;
  const invMass = 1 / spec.nodeMass;
  const deck: number[] = [];
  const top: number[] = [];
  let next = 0; // the index the world will give the next node, for a fresh world
  for (let i = 0; i <= bays; i++) {
    const pinned = i === 0 || i === bays;
    batch.addNode(i * L, 0, 0, pinned ? 0 : invMass, spec.radius);
    deck.push(next++);
  }
  for (let i = 0; i < bays; i++) {
    batch.addNode((i + 0.5) * L, H, 0, invMass, spec.radius);
    top.push(next++);
  }

  const beams: number[] = [];
  let nextBeam = 0;
  const beam = (a: number, b: number, ax: number, ay: number, bx: number, by: number): void => {
    // sqrt is exactly rounded everywhere; Math.hypot is not, and engines differ.
    const rest = Math.sqrt((bx - ax) * (bx - ax) + (by - ay) * (by - ay));
    batch.addBeam(a, b, {
      rest,
      compliance: rest / spec.ea,
      damping: spec.damping,
      yieldStrain: spec.yieldStrain,
      breakStrain: spec.breakStrain,
    });
    beams.push(nextBeam++);
  };
  // Bottom chord, top chord, then the diagonals up and down in every bay.
  for (let i = 0; i < bays; i++) beam(deck[i]!, deck[i + 1]!, i * L, 0, (i + 1) * L, 0);
  for (let i = 0; i + 1 < bays; i++) beam(top[i]!, top[i + 1]!, (i + 0.5) * L, H, (i + 1.5) * L, H);
  for (let i = 0; i < bays; i++) {
    beam(deck[i]!, top[i]!, i * L, 0, (i + 0.5) * L, H);
    beam(top[i]!, deck[i + 1]!, (i + 0.5) * L, H, (i + 1) * L, 0);
  }
  return { spec, deck, top, beams, span: bays * L };
}

/** Where the truck is, in metres from the left support, at the start of step `step` (0-based). */
export function truckPosition(truck: Truck, step: number): number {
  return truck.speed * step * STEP_SECONDS;
}

/**
 * Appends the force commands for the truck's weight during step `step`: split between the two
 * deck nodes either side of it (a simply supported deck between nodes). Nothing once it has left
 * the bridge. Returns whether the truck is still on the bridge.
 */
export function rollTruck(batch: CommandBatch, layout: BridgeLayout, truck: Truck, step: number): boolean {
  const x = truckPosition(truck, step);
  const L = layout.spec.bayLength;
  if (x < 0 || x > layout.span) return false;
  const bay = Math.min(Math.floor(x / L), layout.spec.bays - 1);
  const t = (x - bay * L) / L; // 0 at the left node of the bay, 1 at the right
  const w = truck.mass * GRAVITY;
  batch.addForce(layout.deck[bay]!, 0, -w * (1 - t), 0);
  batch.addForce(layout.deck[bay + 1]!, 0, -w * t, 0);
  return true;
}
