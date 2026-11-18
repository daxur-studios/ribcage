/**
 * Purpose: the parts of the craft page that are plain numbers and need no browser: the throttle
 * keys, the follow camera, the slow-motion rule, the scenarios (fly, hop, drop, one part on the pad)
 * and the readouts.
 * Value: they are tested in Node, so the page itself only wires them to a canvas and a keyboard,
 * and a flight flown with keys is recorded as the same command lists a script makes.
 *
 * Nothing here touches the core's step or its state: the page asks for a throttle, and the only
 * thing that reaches the world is a `setThrottle` command in a recorded list.
 */

import type { CommandBatch } from './commands.ts';
import { prepareFlight, type Flight } from './flight.ts';
import { centreOf, clearance, hopCommands, lostLegs, PAD, prepareDrop, prepareHop, tiltDegrees, type Spot } from './landing.ts';
import type { LoadedCraft } from './parts/craft-file.ts';
import { saveCraft, type CraftPartSpec } from './parts/craft-file.ts';
import type { Terrain } from './terrain.ts';

type V = [number, number, number];

// ---- throttle keys -------------------------------------------------------------------------

/** How far one press of the throttle keys moves the throttle. */
export const THROTTLE_STEP = 0.1;

/** The throttle, to the nearest tenth, kept in 0 to 1. */
function tenths(x: number): number {
  return Math.max(0, Math.min(1, Math.round(x * 10) / 10));
}

/**
 * The throttle a key asks for, given the one now set, or `null` for a key that is not a throttle key.
 * Up (or w) and down (or s) move it by a tenth; z is full and x is cut.
 */
export function throttleKey(current: number, key: string): number | null {
  switch (key) {
    case 'ArrowUp':
    case 'w':
      return tenths(current + THROTTLE_STEP);
    case 'ArrowDown':
    case 's':
      return tenths(current - THROTTLE_STEP);
    case 'z':
      return 1;
    case 'x':
      return 0;
    default:
      return null;
  }
}

/**
 * Appends the command that makes the throttle `wanted`, if it is not what was last sent; returns
 * what was sent last. A step with no change adds no record, so a held throttle costs no bytes.
 */
export function throttleCommands(batch: CommandBatch, thrusterCount: number, wanted: number, lastSent: number): number {
  if (wanted === lastSent) return lastSent;
  for (let t = 0; t < thrusterCount; t++) batch.setThrottle(t, wanted);
  return wanted;
}

// ---- follow camera -------------------------------------------------------------------------

/**
 * One frame of a camera that follows the craft: the look-at point moves toward `centre` so that
 * half the gap is closed every `halfLife` seconds, whatever the frame time. The page moves the
 * camera by the same amount, so what the player has orbited to stays as it was.
 */
export function followStep(target: V, centre: V, dt: number, halfLife: number): V {
  const k = halfLife <= 0 ? 1 : 1 - Math.pow(2, -dt / halfLife);
  return [target[0] + (centre[0] - target[0]) * k, target[1] + (centre[1] - target[1]) * k, target[2] + (centre[2] - target[2]) * k];
}

// ---- slow motion at touchdown --------------------------------------------------------------

/** Slow motion starts this high above the ground when falling faster than `SLOW_FALL_SPEED`. */
export const SLOW_BELOW = 4;
export const SLOW_FALL_SPEED = 2;
/** It stays on this many steps after the first touch, long enough to watch a leg fold and go. */
export const SLOW_HOLD_STEPS = 40;
/** The wall-clock pace while it is on. */
export const SLOW_FACTOR = 0.1;
/** The craft counts as touching when its lowest point is closer than this to the ground, metres. */
export const TOUCH_CLEARANCE = 0.05;

/**
 * The pace multiplier for the next frame: `SLOW_FACTOR` on the way down near the ground and for a
 * short while after the first touch, 1 otherwise. It changes how fast the page asks for steps, never
 * what a step does, so a run is the same at any pace. `sinceTouch` is the steps since the first
 * touch, or -1 for none yet; `descent` is the downward speed, metres per second, positive falling.
 */
export function slowFactor(altitude: number, descent: number, sinceTouch: number): number {
  if (sinceTouch >= 0) return sinceTouch < SLOW_HOLD_STEPS ? SLOW_FACTOR : 1;
  return altitude < SLOW_BELOW && descent > SLOW_FALL_SPEED ? SLOW_FACTOR : 1;
}

// ---- readouts ------------------------------------------------------------------------------

/** What the page prints about the craft. `null` where the craft has no such thing. */
export interface Readout {
  /** Height of the lowest point above the ground under it, metres. */
  altitude: number;
  /** Mean node speed, metres per second. */
  speed: number;
  /** Mean node vertical speed, metres per second, positive up. */
  climb: number;
  /** Degrees between the craft's axis and straight up, or `null` without an engine and a pod. */
  tilt: number | null;
  /** Legs lost, of the legs there are, or `null` for a craft without the hopper's four legs. */
  lostLegs: number | null;
  legCount: number;
}

/** True when the craft has the parts the leg and tilt readouts read: a pod, four legs and an engine. */
export function isHopperShaped(craft: LoadedCraft['craft']): boolean {
  const ids = new Set(craft.tree.map((t) => t.id));
  return craft.thrusters.length > 0 && ids.has('pod') && [0, 1, 2, 3].every((i) => ids.has(`leg${i}`));
}

/** The readout from positions now and one step ago (`dt` seconds apart). */
export function readout(craft: LoadedCraft['craft'], field: Terrain, now: ArrayLike<number>, before: ArrayLike<number>, dt: number, broken: ReadonlySet<number>): Readout {
  const n = craft.nodes.length;
  let mx = 0;
  let my = 0;
  let mz = 0;
  for (let i = 0; i < n; i++) {
    mx += (now[3 * i]! - before[3 * i]!) / n;
    my += (now[3 * i + 1]! - before[3 * i + 1]!) / n;
    mz += (now[3 * i + 2]! - before[3 * i + 2]!) / n;
  }
  const shaped = isHopperShaped(craft);
  const lost = shaped ? lostLegs(craft, broken).filter(Boolean).length : null;
  return {
    altitude: clearance(craft, now, field),
    speed: Math.hypot(mx, my, mz) / dt,
    climb: my / dt,
    tilt: shaped ? tiltDegrees(craft, now) : null,
    lostLegs: lost,
    legCount: shaped ? 4 : 0,
  };
}

/** The centre of the craft's nodes, which the follow camera tracks. */
export function craftCentre(craft: LoadedCraft['craft'], positions: ArrayLike<number>): V {
  return centreOf(
    positions,
    craft.nodes.map((_, i) => i),
  );
}

// ---- scenarios -----------------------------------------------------------------------------

export type ScenarioKind = 'fly' | 'hop' | 'drop' | 'part';

/** What the page starts: a flight, and the commands a script adds on each step (none for hand flying). */
export interface Scenario {
  kind: ScenarioKind;
  flight: Flight;
  /** Appends the scripted commands of a step, or `null` when the player or nothing drives it. */
  script: ((batch: CommandBatch, step: number) => void) | null;
  /** Steps the scenario runs for before holding, or `null` for no end. */
  steps: number | null;
}

export interface ScenarioOptions {
  spot?: Spot;
  /** Touchdown speed for a drop, metres per second. */
  speed?: number;
  /** The part type for the gallery. */
  part?: string;
}

/** The default part of each type, for the gallery: the parameters the hopper uses. */
export const GALLERY_PARTS: readonly CraftPartSpec[] = [
  { id: 'tank', type: 'tank', params: { radius: 1, length: 2.5, ringNodes: 8, rings: 5, material: 'aluminium', wallArea: 5e-4, mass: 2000 } },
  { id: 'truss', type: 'truss', params: { section: 'square', length: 4, width: 1, bays: 4, material: 'aluminium', memberArea: 3e-4, mass: 300 } },
  { id: 'engine', type: 'engine', params: { ringNodes: 8, topRadius: 1, nozzleRadius: 0.5, height: 1, material: 'steel', memberArea: 4e-4, mass: 800, maxThrust: 80e3 } },
  { id: 'pod', type: 'pod', params: { ringNodes: 8, bottomRadius: 1, topRadius: 0.6, height: 1.2, material: 'aluminium', memberArea: 4e-4, mass: 1200 } },
  {
    id: 'leg',
    type: 'leg',
    params: {
      length: 3,
      footOffset: 1.5,
      footSize: 0.35,
      mountWidth: 0.7653668647301797,
      mountHeight: 0.625,
      bodyWidth: 0.9,
      material: 'aluminium',
      memberArea: 2e-4,
      mass: 150,
      sprungStiffness: 3e5,
      sprungStroke: 0.3,
      crushForce: 75e3,
      crushStroke: 0.15,
    },
  },
];

/** The craft file of one default part on its own, as the loader reads it. Throws for a type that is not in the gallery. */
export function galleryCraft(type: string): string {
  const spec = GALLERY_PARTS.find((p) => p.type === type);
  if (!spec) throw new Error(`there is no gallery part "${type}" (one of ${GALLERY_PARTS.map((p) => p.type).join(', ')})`);
  return saveCraft({ root: spec.id, place: { x: 0, y: 0, z: 0, yaw: 0 }, parts: [spec], joins: [] });
}

/** Builds a scenario from a craft file's text. Throws what the craft loader throws. */
export function scenarioFor(kind: ScenarioKind, craftText: string, opts: ScenarioOptions = {}): Scenario {
  switch (kind) {
    case 'fly':
      return { kind, flight: prepareFlight(craftText), script: null, steps: null };
    case 'hop': {
      const hop = prepareHop(craftText);
      return { kind, flight: hop.flight, script: (batch, step) => hopCommands(batch, hop, step), steps: null };
    }
    case 'drop':
      return { kind, flight: prepareDrop(craftText, opts.spot ?? PAD, opts.speed ?? 7), script: null, steps: null };
    case 'part':
      return { kind, flight: prepareDrop(galleryCraft(opts.part ?? 'tank'), PAD, 0), script: null, steps: null };
  }
}
