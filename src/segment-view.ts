/**
 * Purpose: what the craft page's "segment state" view reads off the core: for each segment whether
 * it is live, asleep or never-live rigid, the colour of each beam that follows from it, the box
 * around each rigid body and each vessel, and which vessel is the root the follow camera keeps.
 * Value: plan 005, phase 5.12. These are plain functions of the core's copies (`SegmentView`,
 * `SleepView`, `BodyView`, `VesselView`), so Node tests pin them and the page only draws them.
 *
 * Nothing here touches the core's step or state.
 */

import { SEG_NEVER_LIVE } from './commands.ts';
import { NO_VESSEL, type BodyView, type SegmentView, type SleepView, type VesselView } from './core.ts';
import type { Rgb } from './stress-colour.ts';

export type SegmentMode = 'live' | 'asleep' | 'rigid';

/** Simulated as a lattice this step (it carries and shows stress). */
export const LIVE_RGB: Rgb = [255, 140, 50];
/** A barely loaded segment that went rigid: one body with its neighbours. */
export const ASLEEP_RGB: Rgb = [60, 200, 150];
/** Never live: an engine, a pod or a wheel, rigid from the start. */
export const RIGID_RGB: Rgb = [120, 110, 255];

const MODE_RGB: Record<SegmentMode, Rgb> = { live: LIVE_RGB, asleep: ASLEEP_RGB, rigid: RIGID_RGB };

/** The colour of a mode. */
export function modeRgb(mode: SegmentMode): Rgb {
  return MODE_RGB[mode];
}

/** Each segment's mode: never-live first, then asleep (when sleeping has run), otherwise live. */
export function segmentModes(seg: SegmentView, sleep: SleepView | null): SegmentMode[] {
  const out: SegmentMode[] = [];
  for (let s = 0; s < seg.flags.length; s++) {
    if ((seg.flags[s]! & SEG_NEVER_LIVE) !== 0) out.push('rigid');
    else if (sleep !== null && s < sleep.asleep.length && sleep.asleep[s] !== 0) out.push('asleep');
    else out.push('live');
  }
  return out;
}

/** How many segments are in each mode. */
export function modeCounts(modes: readonly SegmentMode[]): Record<SegmentMode, number> {
  const c: Record<SegmentMode, number> = { live: 0, asleep: 0, rigid: 0 };
  for (const m of modes) c[m]++;
  return c;
}

/** Which mode wins on a beam that joins two segments: a live end makes the beam live. */
const RANK: Record<SegmentMode, number> = { live: 2, asleep: 1, rigid: 0 };

/**
 * One colour per beam (three bytes each) from the modes of its two end nodes' segments. A node with
 * no segment counts as live. `out` must hold three bytes per beam.
 */
export function beamModeColours(beamEnds: ArrayLike<number>, nodeSegment: ArrayLike<number>, modes: readonly SegmentMode[], out: Uint8Array): void {
  const modeOf = (node: number): SegmentMode => {
    const s = nodeSegment[node];
    return s === undefined || s >= modes.length ? 'live' : modes[s]!;
  };
  const beams = out.length / 3;
  for (let i = 0; i < beams; i++) {
    const a = modeOf(beamEnds[2 * i]!);
    const b = modeOf(beamEnds[2 * i + 1]!);
    const [r, g, bl] = MODE_RGB[RANK[a] >= RANK[b] ? a : b];
    out[3 * i] = r;
    out[3 * i + 1] = g;
    out[3 * i + 2] = bl;
  }
}

/** Boxes around groups of nodes: `ids[k]` is the group, `boxes` holds `minx miny minz maxx maxy maxz` for each. */
export interface GroupBoxes {
  ids: number[];
  boxes: Float32Array;
}

/**
 * The axis-aligned box around each group of nodes, groups in order of id. `groupOf[node]` is the
 * node's group; `valid` says which ids are groups (others, such as "none", are skipped).
 */
export function groupBoxes(groupOf: ArrayLike<number>, valid: (id: number) => boolean, positions: ArrayLike<number>, pad = 0): GroupBoxes {
  const found = new Map<number, number[]>();
  const nodes = Math.min(groupOf.length, Math.floor(positions.length / 3));
  for (let n = 0; n < nodes; n++) {
    const id = groupOf[n]!;
    if (!valid(id)) continue;
    const x = positions[3 * n]!;
    const y = positions[3 * n + 1]!;
    const z = positions[3 * n + 2]!;
    const b = found.get(id);
    if (!b) found.set(id, [x, y, z, x, y, z]);
    else {
      b[0] = Math.min(b[0]!, x);
      b[1] = Math.min(b[1]!, y);
      b[2] = Math.min(b[2]!, z);
      b[3] = Math.max(b[3]!, x);
      b[4] = Math.max(b[4]!, y);
      b[5] = Math.max(b[5]!, z);
    }
  }
  const ids = [...found.keys()].sort((a, b) => a - b);
  const boxes = new Float32Array(ids.length * 6);
  for (const [k, id] of ids.entries()) {
    const b = found.get(id)!;
    for (let j = 0; j < 3; j++) {
      boxes[6 * k + j] = b[j]! - pad;
      boxes[6 * k + 3 + j] = b[3 + j]! + pad;
    }
  }
  return { ids, boxes };
}

/** The box around each live rigid body (a body slot is alive when its flag says so). */
export function bodyBoxes(bodies: BodyView, positions: ArrayLike<number>, pad = 0.15): GroupBoxes {
  return groupBoxes(bodies.nodeBody, (id) => id < bodies.flags.length && (bodies.flags[id]! & 1) !== 0, positions, pad);
}

/** The box around each vessel, or none while the craft is one vessel (nothing to outline). */
export function vesselBoxes(vessels: VesselView, positions: ArrayLike<number>, pad = 0.3): GroupBoxes {
  if (vesselCount(vessels) < 2) return { ids: [], boxes: new Float32Array(0) };
  return groupBoxes(vessels.nodeVessel, (id) => id !== NO_VESSEL, positions, pad);
}

/** How many vessels have nodes now. */
export function vesselCount(vessels: VesselView): number {
  const seen = new Set<number>();
  for (const v of vessels.nodeVessel) if (v !== NO_VESSEL) seen.add(v);
  return seen.size;
}

/**
 * The nodes of the root vessel, which the follow camera keeps, or `null` while the craft is one
 * vessel (every node counts). The root is the vessel of the lowest-numbered live node, which is the
 * one that kept the identity at every split (plan 005, phase 5.10).
 */
export function rootVesselNodes(vessels: VesselView): number[] | null {
  if (vesselCount(vessels) < 2) return null;
  let root = NO_VESSEL;
  for (const v of vessels.nodeVessel) {
    if (v !== NO_VESSEL) {
      root = v;
      break;
    }
  }
  const nodes: number[] = [];
  for (const [n, v] of vessels.nodeVessel.entries()) if (v === root) nodes.push(n);
  return nodes;
}
