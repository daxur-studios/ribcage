/**
 * Purpose: the tall-rocket scenario of plan 005, phase 5.11: a pod, five tanks and an engine
 * stacked on a launch clamp, stood for ten seconds, put under full thrust, given a side knock on
 * a tank, and left to settle; run once with sleeping on and once with every segment live, with the
 * readings that say how much the stack wobbles, how many bodies it forms and what it costs.
 * Value: the roadmap check "touching rigid segments merge into one body, and a tall rocket
 * wobbles less than with everything live" is judged by these numbers in `test/tall-rocket.test.ts`,
 * and the page (5.12) runs the same scenario.
 *
 * The clamp is the core's existing pin (`CMD_SET_INV_MASS` with zero): the engine's nozzle ring
 * is pinned in place, so the host needs no new command.
 *
 * Wobble is measured two ways, as in ER 004 (4.11) and sideways:
 *  - stack length: highest to lowest node, and how far it moves in the second half of a phase;
 *  - tip: the pod's apex node, its horizontal distance off the clamp's axis.
 */

import { CommandBatch } from './commands.ts';
import { EVENT_BREAK, EVENT_SLEEP, EVENT_WAKE, EVENT_YIELD, type Core } from './core.ts';
import { prepareFlight, type Flight, type FlightOptions } from './flight.ts';
import { partNodes } from './landing.ts';
import { World } from './world.ts';

/** The phases, by step (60 steps a second): stand 10 s, full thrust 10 s, knock under thrust, settle with the engine cut. */
export const STAND_STEPS = 600;
export const THRUST_STEP = 600;
export const KNOCK_STEP = 1200;
/** The knock lasts this many steps (0.2 s). */
export const KNOCK_STEPS = 12;
export const CUT_STEP = 1500;
export const END_STEP = 2100;

/**
 * Side knock: newtons along +x on the middle tank's middle ring, spread over its nodes, for
 * `KNOCK_STEPS`. Chosen from a sweep (ER 005, 5.11): 30 kN is the largest round figure that
 * yields no beam in either run over the 2100 steps; 40 kN yields one or two beams. So the rocket
 * does not break and the two runs can be compared.
 */
export const KNOCK_FORCE = 30000;

export type Phase = 'standing' | 'thrust' | 'knock' | 'settle';

export function phaseOf(step: number): Phase {
  if (step < THRUST_STEP) return 'standing';
  if (step < KNOCK_STEP) return 'thrust';
  if (step < CUT_STEP) return 'knock';
  return 'settle';
}

/** A tall rocket placed on its clamp, and the node numbers its readings need. */
export interface TallRocket {
  flight: Flight;
  /** The clamp: the engine's nozzle ring, pinned. */
  clamp: number[];
  /** The pod's apex node: the tip of the stack. */
  tip: number;
  /** The nodes of the knocked ring. */
  ring: number[];
  /** Axis of the clamp: the mean x and z of its nodes at spawn. */
  axis: [number, number];
}

/** Builds the setup: the craft file on the landing field with sleeping on or off, and the clamp. */
export function prepareTallRocket(craftText: string, options: FlightOptions = {}): TallRocket {
  const flight = prepareFlight(craftText, undefined, options);
  const craft = flight.loaded.craft;
  const engine = flight.loaded.craft.tree.find((t) => t.id === 'engine');
  if (!engine) throw new Error('the craft has no engine');
  const ringNodes = engine.nodeOf.length > 0 ? new Set(partNodes(craft, 'engine')) : new Set<number>();
  // The nozzle ring is the lowest ring of the engine.
  let low = Infinity;
  for (const i of ringNodes) low = Math.min(low, craft.nodes[i]!.y);
  const clamp = [...ringNodes].filter((i) => Math.abs(craft.nodes[i]!.y - low) < 1e-3 && !craft.thrusters.some((t) => t.tail === i || t.head === i));
  for (const n of clamp) flight.setup.setInvMass(n, 0);
  const pod = partNodes(craft, 'pod');
  let tip = pod[0]!;
  for (const i of pod) if (craft.nodes[i]!.y > craft.nodes[tip]!.y) tip = i;
  const tanks = craft.tree.filter((t) => t.id.startsWith('tank'));
  const mid = tanks[Math.floor(tanks.length / 2)]!;
  const tank = partNodes(craft, mid.id);
  const ys = [...new Set(tank.map((i) => craft.nodes[i]!.y))].sort((a, b) => a - b);
  const midY = ys[Math.floor(ys.length / 2)]!;
  const ring = tank.filter((i) => Math.abs(craft.nodes[i]!.y - midY) < 1e-3);
  let ax = 0;
  let az = 0;
  for (const n of clamp) {
    ax += craft.nodes[n]!.x;
    az += craft.nodes[n]!.z;
  }
  return { flight, clamp, tip, ring, axis: [ax / clamp.length, az / clamp.length] };
}

/** The commands of one step: full thrust from `THRUST_STEP` to `CUT_STEP`, and the knock. */
export function tallRocketCommands(batch: CommandBatch, rocket: TallRocket, step: number, knock = KNOCK_FORCE): void {
  if (step === THRUST_STEP) batch.setThrottle(0, 1);
  if (step === CUT_STEP) batch.setThrottle(0, 0);
  if (step >= KNOCK_STEP && step < KNOCK_STEP + KNOCK_STEPS) {
    for (const n of rocket.ring) batch.addForce(n, knock / rocket.ring.length, 0, 0);
  }
}

/** What one phase measured. */
export interface PhaseStats {
  steps: number;
  substeps: number;
  maxSubsteps: number;
  ms: number;
  /** Stack length range (max − min) over the second half of the phase, metres. */
  lengthRange: number;
  /** Largest change of the stack length from its value at step 0, over the phase, metres. */
  lengthDrift: number;
  /** Largest distance of the tip off the clamp's axis over the phase, and its range over the second half, metres. */
  tipMax: number;
  tipRange: number;
  /** Fewest and most bodies over the phase. */
  bodiesMin: number;
  bodiesMax: number;
  yields: number;
  breaks: number;
}

export interface TallRocketRun {
  phases: Map<Phase, PhaseStats>;
  /** The number of bodies every 30 steps, from step 0. */
  bodies: number[];
  /** The number of segments asleep every 30 steps. */
  asleep: number[];
  /** First step with this many bodies or fewer (-1 when never). */
  firstStepAtMostBodies(n: number): number;
  stackLength0: number;
  wakes: number;
  sleeps: number;
  yielded: Set<number>;
  broken: Set<number>;
  /** The worst switch pass, relative to the rocket's mass at 10 m/s and its kinetic energy at 10 m/s. */
  switches: { passes: number; momentum: number; spin: number; sleepEnergy: number; wakeEnergy: number };
  /** Largest speed of any node over the last 120 steps, metres per second. */
  restSpeed: number;
  /** Vessels at the end. */
  vessels: number;
  hash: bigint;
}

/** Runs the scenario to `END_STEP` and reads it. `knock` overrides the knock force (for the sweep). */
export function runTallRocket(core: Core, craftText: string, sleeping: boolean, knock = KNOCK_FORCE, steps = END_STEP): TallRocketRun {
  const rocket = prepareTallRocket(craftText, { sleeping });
  const world = new World(core);
  world.apply(rocket.flight.setup);
  const run: TallRocketRun = {
    phases: new Map(),
    bodies: [],
    asleep: [],
    firstStepAtMostBodies(n) {
      const k = this.bodies.findIndex((b) => b <= n);
      return k < 0 ? -1 : k * 30;
    },
    stackLength0: 0,
    wakes: 0,
    sleeps: 0,
    yielded: new Set(),
    broken: new Set(),
    switches: { passes: 0, momentum: 0, spin: 0, sleepEnergy: 0, wakeEnergy: 0 },
    restSpeed: 0,
    vessels: 0,
    hash: 0n,
  };
  const nodes = rocket.flight.loaded.craft.nodes;
  const mass = [...world.segments().mass].reduce((a, b) => a + b, 0);
  const p0 = mass * 10;
  const e0 = 0.5 * mass * 100;
  const gap = (a: readonly number[], b: readonly number[]) => Math.hypot(a[0]! - b[0]!, a[1]! - b[1]!, a[2]! - b[2]!);
  const stack = (pos: Float64Array) => {
    let lo = Infinity;
    let hi = -Infinity;
    for (let i = 0; i < nodes.length; i++) {
      lo = Math.min(lo, pos[3 * i + 1]!);
      hi = Math.max(hi, pos[3 * i + 1]!);
    }
    return hi - lo;
  };
  const tipOff = (pos: Float64Array) => Math.hypot(pos[3 * rocket.tip]! - rocket.axis[0], pos[3 * rocket.tip + 2]! - rocket.axis[1]);
  run.stackLength0 = stack(world.positions());
  const lengths = new Map<Phase, number[]>();
  const tips = new Map<Phase, number[]>();
  const prev = new Float64Array(nodes.length * 3);
  for (let s = 0; s < steps; s++) {
    const b = new CommandBatch();
    tallRocketCommands(b, rocket, s, knock);
    world.apply(b);
    const t0 = performance.now();
    const sub = world.step();
    const ms = performance.now() - t0;
    const phase = phaseOf(s);
    let yields = 0;
    let breaks = 0;
    for (const e of world.events()) {
      if (e.kind === EVENT_YIELD) {
        yields++;
        run.yielded.add(e.index);
      } else if (e.kind === EVENT_BREAK) {
        breaks++;
        run.broken.add(e.index);
      } else if (e.kind === EVENT_WAKE) run.wakes++;
      else if (e.kind === EVENT_SLEEP) run.sleeps++;
    }
    const pos = world.positions();
    const body = world.bodies();
    let alive = 0;
    for (const f of body.flags) if ((f & 1) !== 0) alive++;
    const sleepView = sleeping ? world.sleep() : null;
    if (sleepView) {
      const w = run.switches;
      for (const c of sleepView.checks) {
        w.passes++;
        w.momentum = Math.max(w.momentum, gap(c.momentumAfter, c.momentumBefore) / p0);
        w.spin = Math.max(w.spin, gap(c.spinAfter, c.spinBefore) / (p0 * 20));
        const de = (c.energyAfter - c.energyBefore) / e0;
        if (c.kind === 2) w.sleepEnergy = Math.max(w.sleepEnergy, de);
        else w.wakeEnergy = Math.max(w.wakeEnergy, Math.abs(de));
      }
    }
    if (s % 30 === 0) {
      run.bodies.push(alive);
      run.asleep.push(sleepView ? sleepView.asleep.reduce((a, v) => a + (v !== 0 ? 1 : 0), 0) : 0);
    }
    const len = stack(pos);
    const tipD = tipOff(pos);
    let st = run.phases.get(phase);
    if (!st) {
      st = { steps: 0, substeps: 0, maxSubsteps: 0, ms: 0, lengthRange: 0, lengthDrift: 0, tipMax: 0, tipRange: 0, bodiesMin: Infinity, bodiesMax: 0, yields: 0, breaks: 0 };
      run.phases.set(phase, st);
      lengths.set(phase, []);
      tips.set(phase, []);
    }
    st.steps++;
    st.substeps += sub;
    st.maxSubsteps = Math.max(st.maxSubsteps, sub);
    st.ms += ms;
    st.lengthDrift = Math.max(st.lengthDrift, Math.abs(len - run.stackLength0));
    st.tipMax = Math.max(st.tipMax, tipD);
    st.bodiesMin = Math.min(st.bodiesMin, alive);
    st.bodiesMax = Math.max(st.bodiesMax, alive);
    st.yields += yields;
    st.breaks += breaks;
    lengths.get(phase)!.push(len);
    tips.get(phase)!.push(tipD);
    if (s >= steps - 120) {
      let v = 0;
      for (let i = 0; i < nodes.length; i++) {
        v = Math.max(v, Math.hypot(pos[3 * i]! - prev[3 * i]!, pos[3 * i + 1]! - prev[3 * i + 1]!, pos[3 * i + 2]! - prev[3 * i + 2]!) * 60);
      }
      run.restSpeed = Math.max(run.restSpeed, v);
    }
    prev.set(pos);
  }
  for (const [phase, st] of run.phases) {
    const range = (a: number[]) => {
      const half = a.slice(Math.floor(a.length / 2));
      return Math.max(...half) - Math.min(...half);
    };
    st.lengthRange = range(lengths.get(phase)!);
    st.tipRange = range(tips.get(phase)!);
  }
  const vv = world.vessels();
  const seen = new Set<number>();
  for (const v of vv.nodeVessel) if (v !== 0xffffffff) seen.add(v);
  run.vessels = seen.size;
  run.hash = world.stateHash;
  world.free();
  return run;
}
