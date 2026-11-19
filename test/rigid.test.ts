/**
 * Plan 005, phase 5.4: the hopper with its engine and pod rigid. What this file measures and
 * reports: the engine and pod lattices carry no strain (their beams are skipped), the effective
 * mass ratio at every seam node (the live node's mass against the body's `1 / w` along the seam
 * beam), and substeps and milliseconds per step per flight phase, rigid against all-live.
 * The lift-off checks and 4.8's bands themselves run rigid in `flight.test.ts` and `landing.test.ts`.
 */
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";
import { CommandBatch } from "../src/commands.ts";
import { Core, EVENT_BREAK, EVENT_FREEZE, EVENT_YIELD } from "../src/core.ts";
import { HOP_STEPS, hopCommands, PAD, prepareDrop, prepareHop } from "../src/landing.ts";
import type { LoadedCraft } from "../src/parts/craft-file.ts";
import { SUBSTEP_BUDGET } from "../src/parts/part.ts";
import { heightAt } from "../src/terrain.ts";
import { World } from "../src/world.ts";

const WASM_PATH = new URL("../target/wasm32-unknown-unknown/release/beam_core.wasm", import.meta.url);
const HOPPER = readFileSync(new URL("./fixtures/hopper.craft.json", import.meta.url), "utf8");

async function loadCore(): Promise<Core> {
  return Core.load(new Uint8Array(readFileSync(WASM_PATH)));
}

/** The 5.3 case a free heavy body was shown to converge like a pin at: 800 kg against 6.6 kg links. */
const MEASURED_RATIO = 800 / 6.6;

/**
 * The craft nodes a part's rigid body is made of: the nodes of its segment. A part's nodes that
 * were merged into its parent at the join keep the parent's (live) segment, so they are not in it.
 */
function bodyNodes(craft: LoadedCraft["craft"], id: string): number[] {
  const entry = craft.tree.find((t) => t.id === id)!;
  return craft.nodes.flatMap((n, i) => (n.segment === entry.segmentBase ? [i] : []));
}

type V = [number, number, number];
const sub = (a: V, b: V): V => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const cross = (a: V, b: V): V => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const dot = (a: V, b: V): number => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

/** Solves `I x = b` for a symmetric 3×3 `I` (rows), by Cramer's rule. */
function solve3(m: number[][], b: V): V {
  const det = (a: number[][]) =>
    a[0]![0]! * (a[1]![1]! * a[2]![2]! - a[1]![2]! * a[2]![1]!) - a[0]![1]! * (a[1]![0]! * a[2]![2]! - a[1]![2]! * a[2]![0]!) + a[0]![2]! * (a[1]![0]! * a[2]![1]! - a[1]![1]! * a[2]![0]!);
  const d = det(m);
  const out: V = [0, 0, 0];
  for (let k = 0; k < 3; k++) {
    const c = m.map((row, i) => row.map((v, j) => (j === k ? b[i]! : v)));
    out[k] = det(c) / d;
  }
  return out;
}

test("the engine and pod are rigid bodies and their lattices carry no strain through the whole hop", async (t) => {
  const core = await loadCore();
  const hop = prepareHop(HOPPER, { sleeping: false });
  const craft = hop.flight.loaded.craft;
  const world = new World(core);
  assert.equal(world.apply(hop.flight.setup).rejected, 0);
  const rigidParts = ["engine", "pod"];
  const inside = new Map<string, number[]>();
  for (const id of rigidParts) {
    const nodes = new Set(bodyNodes(craft, id));
    inside.set(id, craft.beams.flatMap((b, k) => (nodes.has(b.a) && nodes.has(b.b) ? [k] : [])));
  }
  const p0 = world.positions().slice();
  const len = (p: Float64Array, k: number) => {
    const b = craft.beams[k]!;
    return Math.hypot(p[3 * b.b]! - p[3 * b.a]!, p[3 * b.b + 1]! - p[3 * b.a + 1]!, p[3 * b.b + 2]! - p[3 * b.a + 2]!);
  };
  const worst = new Map<string, { stretch: number; ratio: number }>(rigidParts.map((id) => [id, { stretch: 0, ratio: 0 }]));
  const watched = new Set([...inside.values()].flat());
  let insideEvents = 0;
  for (let s = 0; s < HOP_STEPS; s++) {
    const cmds = new CommandBatch();
    hopCommands(cmds, hop, s);
    world.apply(cmds);
    world.step();
    for (const e of world.events()) if ((e.kind === EVENT_YIELD || e.kind === EVENT_BREAK) && watched.has(e.index)) insideEvents++;
    const p = world.positions();
    const ratio = world.snapshot().beams.ratio;
    for (const [id, beams] of inside) {
      const w = worst.get(id)!;
      for (const k of beams) {
        w.stretch = Math.max(w.stretch, Math.abs(len(p, k) - len(p0, k)) / len(p0, k));
        w.ratio = Math.max(w.ratio, Math.abs(ratio[k]!));
      }
    }
  }
  const bodies = world.bodies();
  world.free();
  assert.equal(bodies.flags.filter((f) => f !== 0).length, 2, "two bodies: the engine and the pod");
  for (const id of rigidParts) {
    const w = worst.get(id)!;
    t.diagnostic(`${id}: ${inside.get(id)!.length} inner beams, worst relative length change ${w.stretch.toExponential(2)}, worst strain ratio ${w.ratio.toExponential(2)}`);
    assert.ok(w.stretch < 1e-12, `${id}: an inner beam changed length by ${w.stretch} relative`);
    assert.ok(w.ratio < 1e-9, `${id}: an inner beam reads strain ${w.ratio} of yield`);
  }
  assert.equal(insideEvents, 0, "no inner beam of the engine or pod yields or breaks");
});

test("the effective mass ratio at every seam node is at or below the 800 : 6.6 case of 5.3", async (t) => {
  const core = await loadCore();
  const flight = prepareDrop(HOPPER, PAD, 0);
  const craft = flight.loaded.craft;
  const world = new World(core);
  assert.equal(world.apply(flight.setup).rejected, 0);
  const p = world.positions();
  const at = (i: number): V => [p[3 * i]!, p[3 * i + 1]!, p[3 * i + 2]!];
  const bodies = world.bodies();
  world.free();
  let worstAll = 0;
  for (const id of ["engine", "pod"]) {
    const nodes = bodyNodes(craft, id);
    const set = new Set(nodes);
    // The body as the core makes it at spawn: mass, centre, and inertia about the centre.
    let mass = 0;
    const c: V = [0, 0, 0];
    for (const i of nodes) {
      const m = craft.nodes[i]!.mass;
      mass += m;
      for (let k = 0; k < 3; k++) c[k]! += m * at(i)[k]!;
    }
    for (let k = 0; k < 3; k++) c[k]! /= mass;
    const I = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
    for (const i of nodes) {
      const m = craft.nodes[i]!.mass;
      const r = sub(at(i), c);
      const rr = dot(r, r);
      for (let a = 0; a < 3; a++) for (let b = 0; b < 3; b++) I[a]![b]! += m * ((a === b ? rr : 0) - r[a]! * r[b]!);
    }
    const slot = bodies.nodeBody[nodes[0]!]!;
    assert.ok(Math.abs(bodies.mass[slot]! - mass) <= 1e-9 * mass, `${id}: body mass ${bodies.mass[slot]} against ${mass}`);
    // Every seam beam: one end on the body, the other live.
    let worst = 0;
    let lowest = Infinity;
    let count = 0;
    const lines: string[] = [];
    for (const b of craft.beams) {
      const onA = set.has(b.a);
      if (onA === set.has(b.b)) continue;
      const ride = onA ? b.a : b.b;
      const live = onA ? b.b : b.a;
      const d = sub(at(live), at(ride));
      const l = Math.sqrt(dot(d, d));
      const n: V = [d[0] / l, d[1] / l, d[2] / l];
      const rn = cross(sub(at(ride), c), n);
      const w = 1 / mass + dot(rn, solve3(I, rn));
      const effective = 1 / w;
      const ratio = effective / craft.nodes[live]!.mass;
      worst = Math.max(worst, ratio);
      lowest = Math.min(lowest, ratio);
      count++;
      if (lines.length < 3 || ratio === worst) lines.push(`${ride}->${live} ${effective.toFixed(1)} kg : ${craft.nodes[live]!.mass.toFixed(1)} kg`);
    }
    t.diagnostic(`${id}: body ${mass.toFixed(1)} kg, ${count} seam beams, effective : live from ${lowest.toFixed(2)} to ${worst.toFixed(2)} (5.3 measured ${MEASURED_RATIO.toFixed(1)}); e.g. ${lines.slice(0, 3).join(", ")}`);
    assert.ok(count > 0, `${id} has seam beams`);
    worstAll = Math.max(worstAll, worst);
  }
  assert.ok(worstAll <= MEASURED_RATIO, `a seam node sees ${worstAll.toFixed(1)} : 1, above the measured ${MEASURED_RATIO.toFixed(1)} : 1`);
});

interface PhaseCost {
  steps: number;
  maxSubsteps: number;
  meanSubsteps: number;
  ms: number;
}

/** Runs the hop (and a standing craft) once, and returns per phase the substeps and ms per step. */
function phaseCosts(core: Core, rigid: boolean): Record<string, PhaseCost> {
  const acc = new Map<string, { steps: number; max: number; sum: number; ms: number }>();
  const add = (phase: string, sub: number, ms: number) => {
    const a = acc.get(phase) ?? { steps: 0, max: 0, sum: 0, ms: 0 };
    a.steps++;
    a.max = Math.max(a.max, sub);
    a.sum += sub;
    a.ms += ms;
    acc.set(phase, a);
  };
  // Standing: the craft just touching the pad, engine off, after the first second.
  {
    const flight = prepareDrop(HOPPER, PAD, 0, undefined, { rigid });
    const world = new World(core);
    world.apply(flight.setup);
    for (let s = 0; s < 360; s++) {
      const t0 = performance.now();
      const sub = world.step();
      if (s >= 60) add("standing", sub, performance.now() - t0);
    }
    world.free();
  }
  // The hop: lift-off is the first second of thrust, coast from the cut to touchdown, touchdown the second after it.
  const hop = prepareHop(HOPPER, { rigid });
  const craft = hop.flight.loaded.craft;
  const world = new World(core);
  world.apply(hop.flight.setup);
  let touch = -1;
  for (let s = 0; s < HOP_STEPS; s++) {
    const cmds = new CommandBatch();
    hopCommands(cmds, hop, s);
    world.apply(cmds);
    const t0 = performance.now();
    const sub = world.step();
    const ms = performance.now() - t0;
    for (const e of world.events()) assert.notEqual(e.kind, EVENT_FREEZE);
    if (s < 60) add("lift-off", sub, ms);
    else if (s >= 220 && touch < 0) add("coast", sub, ms);
    else if (touch >= 0 && s < touch + 60) add("touchdown", sub, ms);
    if (s >= 220 && touch < 0) {
      const p = world.positions();
      for (const [i, n] of craft.nodes.entries()) {
        if (p[3 * i + 1]! - n.radius - heightAt(hop.flight.field, p[3 * i]!, p[3 * i + 2]!) < 0.002) touch = s + 1;
      }
    }
  }
  world.free();
  const out: Record<string, PhaseCost> = {};
  for (const phase of ["standing", "lift-off", "coast", "touchdown"]) {
    const a = acc.get(phase)!;
    out[phase] = { steps: a.steps, maxSubsteps: a.max, meanSubsteps: a.sum / a.steps, ms: a.ms / a.steps };
  }
  return out;
}

test("substeps and ms per step per flight phase, rigid against all-live", async (t) => {
  const core = await loadCore();
  phaseCosts(core, true); // warm up the WebAssembly code before timing
  const live = phaseCosts(core, false);
  const rigid = phaseCosts(core, true);
  for (const phase of Object.keys(live)) {
    const a = live[phase]!;
    const b = rigid[phase]!;
    t.diagnostic(
      `${phase.padEnd(9)} (${String(a.steps).padStart(3)} steps): all-live ${a.maxSubsteps} max / ${a.meanSubsteps.toFixed(2)} mean substeps, ${a.ms.toFixed(3)} ms; rigid ${b.maxSubsteps} max / ${b.meanSubsteps.toFixed(2)} mean, ${b.ms.toFixed(3)} ms`,
    );
    assert.ok(b.maxSubsteps <= SUBSTEP_BUDGET, `${phase}: rigid takes ${b.maxSubsteps} substeps`);
    assert.ok(b.maxSubsteps <= a.maxSubsteps, `${phase}: rigid takes more substeps (${b.maxSubsteps}) than all-live (${a.maxSubsteps})`);
  }
});
