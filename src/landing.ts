/**
 * Purpose: the landing scenarios for a craft: a drop that touches down at a chosen speed on a
 * chosen spot, the scripted hop, and the readings a test or the page takes from a landing (which
 * legs are still joined to the pod, how far the craft has tipped, whether it is at rest).
 * Value: the hard landing (milestone 4.8) is judged by these functions in tests, recorded as a
 * golden, and shown by the craft page (4.9), all from the same code.
 *
 * A drop places the craft at rest above the ground so that its lowest point is `v² / 2g` above
 * what it will meet; free fall carries no internal load, so the touchdown speed is exact to the
 * step's resolution. The hop is a throttle and nudge timeline played as commands.
 */

import type { CommandBatch } from './commands.ts';
import { loadCraft, type LoadedCraft } from './parts/craft-file.ts';
import { prepareFlight, type Flight, type FlightOptions } from './flight.ts';
import { heightAt, landingField, type Terrain } from './terrain.ts';

/** Standard gravity of the core's default world (`core/src/world.rs`), metres per second squared. */
export const GRAVITY = 9.81;

/** A place on the landing field to put a craft above: the craft's axis goes through `(x, z)`. */
export interface Spot {
  name: string;
  x: number;
  z: number;
  /** Whole 15° steps the craft is turned about its axis before it falls. */
  yaw: number;
}

/** The middle of the pad. */
export const PAD: Spot = { name: 'pad', x: 0, z: 0, yaw: 0 };

/**
 * Five fixed spots in the rough area (`ROUGH_AREA`), each with its own turn so the four feet meet
 * different slopes. They were picked by a search (ER 004, phase 4.8 and its review): on these a
 * touchdown at 3 m/s breaks nothing and, at 7 m/s, a leg is lost (`lostLegs`) and the craft tips
 * over, and each still does so when moved 0.25 m either way or turned one step (at least five of
 * those seven variations in six). They are chosen, not typical: `scatterSpots` is the unchosen set.
 */
export const ROUGH_SPOTS: readonly Spot[] = [
  { name: 'rough-a', x: 10.75, z: -4.5, yaw: 2 },
  { name: 'rough-b', x: 10.5, z: -2.75, yaw: 1 },
  { name: 'rough-c', x: 12.5, z: -0.75, yaw: 4 },
  { name: 'rough-d', x: 11.25, z: 3, yaw: 3 },
  { name: 'rough-e', x: 10, z: -5.5, yaw: 0 },
];

/**
 * `count` spots spread over the rough area by a small integer generator (the same on every
 * machine), with yaw 0 to 5: the unchosen set the hard-landing claims are also read against.
 */
export function scatterSpots(count: number, seed = 777): Spot[] {
  let state = seed;
  const next = (): number => {
    state = (state * 1664525 + 1013904223) % 4294967296;
    return state / 4294967296;
  };
  const out: Spot[] = [];
  for (let i = 0; i < count; i++) {
    const x = 8 + 6 * next();
    const z = -6 + 12 * next();
    out.push({ name: `scatter-${i}`, x, z, yaw: Math.floor(next() * 6) });
  }
  return out;
}

/** The drop height in metres for a touchdown at `speed` m/s. */
export function dropHeight(speed: number): number {
  return (speed * speed) / (2 * GRAVITY);
}

/** `craftText` with its root's place replaced (whole 15° yaw steps); the text stays a valid craft file. */
export function placeCraft(craftText: string, place: { x: number; y: number; z: number; yaw: number }): string {
  const file = JSON.parse(craftText) as { place: { x: number; y: number; z: number; yaw: number } };
  file.place = { x: place.x, y: place.y, z: place.z, yaw: place.yaw };
  return JSON.stringify(file, null, 2);
}

/** Height of the lowest point of the craft above the terrain under it: node height minus radius minus ground, smallest over nodes. */
export function clearance(craft: LoadedCraft['craft'], positions: ArrayLike<number>, field: Terrain): number {
  let low = Infinity;
  for (const [i, n] of craft.nodes.entries()) {
    const c = positions[3 * i + 1]! - n.radius - heightAt(field, positions[3 * i]!, positions[3 * i + 2]!);
    if (c < low) low = c;
  }
  return low;
}

/** The craft's own node positions, as the loaded craft places them. */
function craftPositions(craft: LoadedCraft['craft']): Float64Array {
  const out = new Float64Array(3 * craft.nodes.length);
  for (const [i, n] of craft.nodes.entries()) {
    out[3 * i] = n.x;
    out[3 * i + 1] = n.y;
    out[3 * i + 2] = n.z;
  }
  return out;
}

/**
 * A flight whose craft stands above `spot` at rest, so that it touches down at about `speed` m/s
 * (`speed` 0 puts it just touching). Its lowest point, over the terrain under the nodes as they
 * start, is `dropHeight(speed)` above the ground.
 */
export function prepareDrop(craftText: string, spot: Spot, speed: number, field: Terrain = landingField(), options: FlightOptions = {}): Flight {
  const base = loadCraft(craftText).file.place;
  const first = loadCraft(placeCraft(craftText, { x: spot.x, y: base.y, z: spot.z, yaw: spot.yaw }));
  const c0 = clearance(first.craft, craftPositions(first.craft), field);
  const y = base.y + dropHeight(speed) - c0;
  return prepareFlight(placeCraft(craftText, { x: spot.x, y, z: spot.z, yaw: spot.yaw }), field, options);
}

/** Indices (into the craft's nodes) of the four feet of leg `leg` (0 to 3) and of the pod's nodes. */
export function footNodes(craft: LoadedCraft['craft'], leg: number): number[] {
  const entry = craft.tree.find((t) => t.id === `leg${leg}`);
  if (!entry) throw new Error(`the craft has no part leg${leg}`);
  return [16, 17, 18, 19].map((k) => entry.nodeOf[k]!);
}

/** Craft nodes of the part with this id. */
export function partNodes(craft: LoadedCraft['craft'], id: string): number[] {
  const entry = craft.tree.find((t) => t.id === id);
  if (!entry) throw new Error(`the craft has no part ${id}`);
  return [...new Set(entry.nodeOf)];
}

/**
 * Which legs are still joined to the pod: a walk from the pod's nodes along every beam that has
 * not broken, and a leg is joined when any node of its foot is reached.
 */
export function joinedLegs(craft: LoadedCraft['craft'], broken: ReadonlySet<number>): boolean[] {
  const n = craft.nodes.length;
  const next: number[][] = Array.from({ length: n }, () => []);
  for (const [i, b] of craft.beams.entries()) {
    if (broken.has(i)) continue;
    next[b.a]!.push(b.b);
    next[b.b]!.push(b.a);
  }
  const seen = new Uint8Array(n);
  const stack = partNodes(craft, 'pod');
  for (const s of stack) seen[s] = 1;
  while (stack.length > 0) {
    const a = stack.pop()!;
    for (const b of next[a]!) {
      if (!seen[b]) {
        seen[b] = 1;
        stack.push(b);
      }
    }
  }
  const out: boolean[] = [];
  for (let leg = 0; leg < 4; leg++) out.push(footNodes(craft, leg).some((f) => seen[f] === 1));
  return out;
}

/**
 * How many of the crush bay's twelve members of leg `leg` have not broken. The bay is bay 1 of the
 * leg: the members between its stations 1 and 2 (nodes 4 to 7 and 8 to 11 of the part).
 */
export function crushLive(craft: LoadedCraft['craft'], broken: ReadonlySet<number>, leg: number): number {
  const entry = craft.tree.find((t) => t.id === `leg${leg}`);
  if (!entry) throw new Error(`the craft has no part leg${leg}`);
  const top = new Set([4, 5, 6, 7].map((k) => entry.nodeOf[k]!));
  const bottom = new Set([8, 9, 10, 11].map((k) => entry.nodeOf[k]!));
  let live = 0;
  for (const [i, b] of craft.beams.entries()) {
    if (broken.has(i)) continue;
    if ((top.has(b.a) && bottom.has(b.b)) || (top.has(b.b) && bottom.has(b.a))) live++;
  }
  return live;
}

/** A leg whose crush bay has fewer live members than this is lost. An even 7 m/s drop leaves six of twelve; a folded leg one to four. */
export const LEG_LOST_BELOW = 3;

/**
 * Which legs are lost: cut off from the pod (`joinedLegs`), or hanging by a crush bay of fewer than
 * `LEG_LOST_BELOW` members. A crush bay can fold about one hinge member that survives, which keeps
 * the foot connected but leaves a leg that carries nothing, so a fold counts as a snap (ER 004,
 * phase 4.8 review).
 */
export function lostLegs(craft: LoadedCraft['craft'], broken: ReadonlySet<number>): boolean[] {
  const joined = joinedLegs(craft, broken);
  return joined.map((j, leg) => !j || crushLive(craft, broken, leg) < LEG_LOST_BELOW);
}

/** The centre of a set of nodes. */
export function centreOf(positions: ArrayLike<number>, nodes: readonly number[]): [number, number, number] {
  const c: [number, number, number] = [0, 0, 0];
  for (const i of nodes) for (let k = 0; k < 3; k++) c[k]! += positions[3 * i + k]! / nodes.length;
  return c;
}

/**
 * The craft's axis from the engine's nozzle end to the pod's apex (the last pod node), as a vector,
 * and its angle in degrees from straight up.
 */
export function tiltDegrees(craft: LoadedCraft['craft'], positions: ArrayLike<number>): number {
  const tail = craft.thrusters[0]!.tail;
  const pod = partNodes(craft, 'pod');
  const apex = pod[pod.length - 1]!;
  const dx = positions[3 * apex]! - positions[3 * tail]!;
  const dy = positions[3 * apex + 1]! - positions[3 * tail + 1]!;
  const dz = positions[3 * apex + 2]! - positions[3 * tail + 2]!;
  return (Math.acos(Math.max(-1, Math.min(1, dy / Math.sqrt(dx * dx + dy * dy + dz * dz)))) * 180) / Math.PI;
}

/** Thrust of the sideways nudge thruster of the hop, newtons (ER 004: assumed, a cold-gas jet). */
export const NUDGE_THRUST = 3000;

/** A flight with a second thruster that pushes across the tank, for the scripted hop. */
export interface Hop {
  flight: Flight;
  /** Index of the nudge thruster in the world (the craft's own thrusters come first). */
  nudge: number;
}

/**
 * The hop's flight: the craft on the pad as it is filed, plus the nudge thruster, joining the two
 * nodes of the tank's middle ring that are furthest apart along x, so it pushes toward +x at about
 * the height of the centre of mass. The thruster is appended to the setup, after the craft's own.
 */
export function prepareHop(craftText: string, options: FlightOptions = {}): Hop {
  const flight = prepareFlight(craftText, undefined, options);
  const craft = flight.loaded.craft;
  const tank = partNodes(craft, 'tank');
  const ys = [...new Set(tank.map((i) => craft.nodes[i]!.y))].sort((a, b) => a - b);
  const midY = ys[Math.floor(ys.length / 2)]!;
  const ring = tank.filter((i) => Math.abs(craft.nodes[i]!.y - midY) < 1e-3).sort((a, b) => craft.nodes[a]!.x - craft.nodes[b]!.x);
  flight.setup.addThruster(ring[0]!, ring[ring.length - 1]!, NUDGE_THRUST);
  return { flight, nudge: craft.thrusters.length };
}

/** From this step, set this throttle (engine or nudge). */
export type HopKey = readonly [step: number, throttle: number];

/**
 * The hop, in steps of 1/60 s: the engine lifts the craft off the pad (0.85, then 0.52 from 0.4 s,
 * then a hold just under its weight from 2.5 s), the nudge pushes across from 0.33 s, and both are
 * cut at 3.67 s (step 220, in the middle of the range of cuts, 180 to 300, that all lose a leg) so the craft falls and touches down at about 7.4 m/s, 7.4 m from where it left.
 */
export const HOP_ENGINE: readonly HopKey[] = [[0, 0.85], [24, 0.52], [150, 0.564], [220, 0]];
export const HOP_NUDGE: readonly HopKey[] = [[20, 1], [220, 0]];
/** Steps the hop is run for: 22 s, touchdown at about 4.6 s and then a rest window that starts 15 s after it. */
export const HOP_STEPS = 1320;

/** Appends the commands of one step of the hop: the throttle of any key that starts on that step. */
export function hopCommands(batch: CommandBatch, hop: Hop, step: number): void {
  for (const [s, th] of HOP_ENGINE) if (s === step) batch.setThrottle(0, th);
  for (const [s, th] of HOP_NUDGE) if (s === step) batch.setThrottle(hop.nudge, th);
}
