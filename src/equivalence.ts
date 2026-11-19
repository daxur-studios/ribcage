/**
 * Purpose: the equivalence scenes of plan 005, phase 5.9: the hopper's drops, full throttle, the
 * scripted hop and a side knock on the tank, each run once with sleeping on and once with every
 * segment live, and the readings that compare the two (which beams broke and yielded, which legs
 * were lost, the tilt at rest, substeps and milliseconds per step per flight phase).
 * Value: sleeping is a cost saving, not a change to the game. This is the test that says so, and
 * the measurement the sleep thresholds in `core/src/sleep.rs` are tuned against.
 *
 * "Everything live" is the flight default with sleeping off: the engine and pod are rigid bodies
 * either way (they are never-live, phase 5.4), so the only difference between the two runs of a
 * scene is the one command `CMD_SET_SLEEPING`.
 */

import { CommandBatch } from './commands.ts';
import { EVENT_BREAK, EVENT_FREEZE, EVENT_SLEEP, EVENT_WAKE, EVENT_YIELD, type Core } from './core.ts';
import { flightCommands, prepareFlight, type Flight } from './flight.ts';
import { HOP_STEPS, PAD, ROUGH_SPOTS, hopCommands, lostLegs, partNodes, prepareDrop, prepareHop, tiltDegrees, type Spot } from './landing.ts';
import { heightAt } from './terrain.ts';
import { World } from './world.ts';

/** One scene: its setup and per-step commands, how long it runs, and its flight phases by step. */
export interface Scene {
  name: string;
  steps: number;
  /** Builds the flight with sleeping on or off. */
  flight(sleeping: boolean): { flight: Flight; commands: (batch: CommandBatch, step: number) => void };
}

/** What a run of a scene left: per beam yields and breaks, the legs lost, tilt, cost per phase. */
export interface SceneRun {
  broken: Set<number>;
  yielded: Set<number>;
  yields: number;
  freezes: number;
  lost: boolean[];
  tilt: number;
  /** First step a node came within 2 mm of the ground after step 20 (-1 when none). */
  touchStep: number;
  wakes: number;
  sleeps: number;
  /** Steps on which at least one segment was asleep at the end of the step. */
  asleepSteps: number;
  phases: Map<string, { steps: number; substeps: number; maxSubsteps: number; ms: number }>;
  /**
   * The worst switch pass, relative to the craft's mass at 10 m/s (spin: and 20 m from the origin;
   * energy: its kinetic energy at 10 m/s): momentum and spin change, energy gained on a sleep, and
   * energy changed on a wake. Zero with sleeping off.
   */
  switches: { passes: number; momentum: number; spin: number; sleepEnergy: number; wakeEnergy: number };
  hash: bigint;
}

/**
 * Side knock: newtons along +x on the tank's middle ring, spread over its nodes. 70 kN for 0.2 s
 * yields about ten beams all-live and leaves the craft standing; 85 kN breaks some and tips it
 * part way, 100 kN knocks it over.
 */
export const KNOCK_FORCE = 70000;
/** The knock starts on this step (the standing craft is asleep whole by about step 310) and lasts this many. */
export const KNOCK_STEP = 420;
export const KNOCK_STEPS = 12;

/** The nodes of the tank's middle ring (the ring nearest the middle in height). */
export function tankMiddleRing(flight: Flight): number[] {
  const craft = flight.loaded.craft;
  const tank = partNodes(craft, 'tank');
  const ys = [...new Set(tank.map((i) => craft.nodes[i]!.y))].sort((a, b) => a - b);
  const midY = ys[Math.floor(ys.length / 2)]!;
  return tank.filter((i) => Math.abs(craft.nodes[i]!.y - midY) < 1e-3);
}

const DROP_STEPS = 1200;

function dropScene(craftText: string, spot: Spot, speed: number): Scene {
  return {
    name: `drop ${speed} m/s ${spot.name}`,
    steps: DROP_STEPS,
    flight: (sleeping) => ({ flight: prepareDrop(craftText, spot, speed, undefined, { sleeping }), commands: () => {} }),
  };
}

/** Every scene of the equivalence test, in a fixed order. */
export function equivalenceScenes(craftText: string): Scene[] {
  const spots: Spot[] = [PAD, ...ROUGH_SPOTS];
  const out: Scene[] = [dropScene(craftText, PAD, 2)];
  for (const s of spots) out.push(dropScene(craftText, s, 3));
  for (const s of spots) out.push(dropScene(craftText, s, 7));
  out.push({
    name: 'full throttle',
    steps: 600,
    flight: (sleeping) => {
      const flight = prepareFlight(craftText, undefined, { sleeping });
      const script = [{ step: 0, throttle: 1 }];
      return { flight, commands: (b, s) => flightCommands(b, flight, script, s) };
    },
  });
  out.push({
    name: 'hop',
    steps: HOP_STEPS,
    flight: (sleeping) => {
      const hop = prepareHop(craftText, { sleeping });
      return { flight: hop.flight, commands: (b, s) => hopCommands(b, hop, s) };
    },
  });
  out.push({
    name: 'side knock',
    steps: DROP_STEPS,
    flight: (sleeping) => {
      const flight = prepareDrop(craftText, PAD, 0, undefined, { sleeping });
      const ring = tankMiddleRing(flight);
      return {
        flight,
        commands: (b, s) => {
          if (s >= KNOCK_STEP && s < KNOCK_STEP + KNOCK_STEPS) for (const n of ring) b.addForce(n, KNOCK_FORCE / ring.length, 0, 0);
        },
      };
    },
  });
  return out;
}

/** The flight phase of a step: standing or thrust before any engine cut, coast until touchdown, then touchdown for a second, then rest. */
function phaseOf(scene: Scene, step: number, touch: number): string {
  if (scene.name === 'full throttle') return step < 60 ? 'lift-off' : 'thrust';
  if (scene.name === 'hop') {
    if (step < 60) return 'lift-off';
    if (step < 220) return 'thrust';
    if (touch < 0) return 'coast';
    return step < touch + 60 ? 'touchdown' : 'rest';
  }
  if (scene.name === 'side knock') return step < KNOCK_STEP ? 'standing' : step < KNOCK_STEP + 60 ? 'knock' : 'rest';
  if (touch < 0) return 'fall';
  return step < touch + 60 ? 'touchdown' : 'rest';
}

/** Runs a scene once. */
export function runScene(core: Core, scene: Scene, sleeping: boolean): SceneRun {
  const { flight, commands } = scene.flight(sleeping);
  const craft = flight.loaded.craft;
  const world = new World(core);
  world.apply(flight.setup);
  const out: SceneRun = {
    broken: new Set(),
    yielded: new Set(),
    yields: 0,
    freezes: 0,
    lost: [],
    tilt: 0,
    touchStep: -1,
    wakes: 0,
    sleeps: 0,
    asleepSteps: 0,
    phases: new Map(),
    switches: { passes: 0, momentum: 0, spin: 0, sleepEnergy: 0, wakeEnergy: 0 },
    hash: 0n,
  };
  const mass = [...world.segments().mass].reduce((a, b) => a + b, 0);
  const p0 = mass * 10;
  const e0 = 0.5 * mass * 100;
  const gap = (a: readonly number[], b: readonly number[]) => Math.hypot(a[0]! - b[0]!, a[1]! - b[1]!, a[2]! - b[2]!);
  for (let s = 0; s < scene.steps; s++) {
    const b = new CommandBatch();
    commands(b, s);
    world.apply(b);
    const t0 = performance.now();
    const sub = world.step();
    const ms = performance.now() - t0;
    for (const e of world.events()) {
      if (e.kind === EVENT_YIELD) {
        out.yields++;
        out.yielded.add(e.index);
      } else if (e.kind === EVENT_BREAK) out.broken.add(e.index);
      else if (e.kind === EVENT_FREEZE) out.freezes++;
      else if (e.kind === EVENT_WAKE) out.wakes++;
      else if (e.kind === EVENT_SLEEP) out.sleeps++;
    }
    if (sleeping) {
      const view = world.sleep();
      if (view.asleep.some((a) => a !== 0)) out.asleepSteps++;
      const w = out.switches;
      for (const c of view.checks) {
        w.passes++;
        w.momentum = Math.max(w.momentum, gap(c.momentumAfter, c.momentumBefore) / p0);
        w.spin = Math.max(w.spin, gap(c.spinAfter, c.spinBefore) / (p0 * 20));
        const de = (c.energyAfter - c.energyBefore) / e0;
        if (c.kind === 2) w.sleepEnergy = Math.max(w.sleepEnergy, de);
        else w.wakeEnergy = Math.max(w.wakeEnergy, Math.abs(de));
      }
    }
    const phase = phaseOf(scene, s, out.touchStep);
    const p = out.phases.get(phase) ?? { steps: 0, substeps: 0, maxSubsteps: 0, ms: 0 };
    p.steps++;
    p.substeps += sub;
    p.maxSubsteps = Math.max(p.maxSubsteps, sub);
    p.ms += ms;
    out.phases.set(phase, p);
    if (out.touchStep < 0 && s > 20 && scene.name !== 'side knock' && scene.name !== 'full throttle') {
      const pos = world.positions();
      for (const [i, n] of craft.nodes.entries()) {
        if (pos[3 * i + 1]! - n.radius - heightAt(flight.field, pos[3 * i]!, pos[3 * i + 2]!) < 0.002) {
          out.touchStep = s + 1;
          break;
        }
      }
    }
  }
  const final = world.positions();
  out.tilt = tiltDegrees(craft, final);
  out.lost = lostLegs(craft, out.broken);
  out.hash = world.stateHash;
  world.free();
  return out;
}

/** The difference between a run with sleeping on and one all-live. */
export interface Comparison {
  /** Beams broken in one run and not the other. */
  breakDiff: number;
  /** Beams that yielded with sleeping on and never all-live. */
  extraYields: number;
  /** Beams that yielded all-live and never with sleeping on. */
  missedYields: number;
  sameLegs: boolean;
  tiltDiff: number;
}

export function compareRuns(on: SceneRun, off: SceneRun): Comparison {
  let breakDiff = 0;
  for (const k of on.broken) if (!off.broken.has(k)) breakDiff++;
  for (const k of off.broken) if (!on.broken.has(k)) breakDiff++;
  let extraYields = 0;
  for (const k of on.yielded) if (!off.yielded.has(k)) extraYields++;
  let missedYields = 0;
  for (const k of off.yielded) if (!on.yielded.has(k)) missedYields++;
  return {
    breakDiff,
    extraYields,
    missedYields,
    sameLegs: on.lost.every((l, i) => l === off.lost[i]),
    tiltDiff: Math.abs(on.tilt - off.tilt),
  };
}
