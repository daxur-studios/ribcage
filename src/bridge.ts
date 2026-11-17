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
import {
  compileDesign,
  rollLoad,
  type Deck,
  type Design,
  type Truck,
} from './design.ts';

export { GRAVITY, STANDARD_TRUCK, STEP_SECONDS, truckPosition, type Truck } from './design.ts';

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

/** The truss as a design: deck nodes, then top nodes, then bottom chord, top chord and diagonals. */
export function bridgeDesign(spec: BridgeSpec): Design {
  const { bays, bayLength: L, depth: H } = spec;
  const nodes: Design['nodes'] = [];
  for (let i = 0; i <= bays; i++) nodes.push({ x: i * L, y: 0, pinned: i === 0 || i === bays });
  for (let i = 0; i < bays; i++) nodes.push({ x: (i + 0.5) * L, y: H, pinned: false });
  const deck = (i: number): number => i;
  const top = (i: number): number => bays + 1 + i;
  const beams: Design['beams'] = [];
  for (let i = 0; i < bays; i++) beams.push({ a: deck(i), b: deck(i + 1) });
  for (let i = 0; i + 1 < bays; i++) beams.push({ a: top(i), b: top(i + 1) });
  for (let i = 0; i < bays; i++) {
    beams.push({ a: deck(i), b: top(i) });
    beams.push({ a: top(i), b: deck(i + 1) });
  }
  return { nodes, beams };
}

/**
 * Appends the commands that build the bridge to `batch`. The node and beam indices are those the
 * world will hand out when the batch is applied to an **empty** world, in order; the tests check
 * that against the results.
 */
export function buildBridge(batch: CommandBatch, spec: BridgeSpec): BridgeLayout {
  const design = bridgeDesign(spec);
  compileDesign(batch, design, spec);
  const { bays, bayLength: L } = spec;
  const deck = Array.from({ length: bays + 1 }, (_, i) => i);
  const top = Array.from({ length: bays }, (_, i) => bays + 1 + i);
  const beams = design.beams.map((_, i) => i);
  return { spec, deck, top, beams, span: bays * L };
}

/**
 * Appends the force commands for the truck's weight during step `step` (see `rollLoad`).
 * Returns whether the truck is still on the bridge.
 */
export function rollTruck(batch: CommandBatch, layout: BridgeLayout, truck: Truck, step: number): boolean {
  const L = layout.spec.bayLength;
  const deck: Deck = { ids: layout.deck, xs: layout.deck.map((_, i) => i * L) };
  return rollLoad(batch, deck, truck, step);
}
