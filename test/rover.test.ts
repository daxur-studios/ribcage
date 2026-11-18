/**
 * Plan 005, phase 5.6: a rover on four wheels, each a rigid body hinged to the chassis with a
 * rotor. Headless. What this file measures: the creep of the rover at rest on the pad, its speed
 * with the rotors driven against rotor speed × rolling radius, its stop, that no wheel beam is
 * solved, that the same script twice gives the same hash, and that a push at one corner tips it at
 * the static tipping force with no energy from the contacts.
 */
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";
import { CommandBatch } from "../src/commands.ts";
import { Core } from "../src/core.ts";
import { compileRover, driveRover, rover, type RoverParams } from "../src/parts/rover.ts";
import { compileTerrain, landingField } from "../src/terrain.ts";
import { World } from "../src/world.ts";

const WASM_PATH = new URL("../target/wasm32-unknown-unknown/release/beam_core.wasm", import.meta.url);
const loadCore = async (): Promise<Core> => Core.load(new Uint8Array(readFileSync(WASM_PATH)));

const PARAMS: RoverParams = {
  length: 2,
  width: 1.2,
  height: 0.5,
  x: -6,
  z: 0,
  axleHeight: 0.452,
  chassisMass: 340,
  stub: 0.3,
  wheel: { rimNodes: 24, radius: 0.4, material: "steel", memberArea: 2e-4, mass: 30, nodeRadius: 0.05 },
};
const CAP = 150; // N·m per wheel
const DRIVE_FROM = 180; // step: 3 s settle
const DRIVE_TO = 180 + 300; // 5 s driven
const STEPS = DRIVE_TO + 240; // 4 s to stop

interface Run {
  hash: bigint;
  x: Float64Array; // chassis centre x per step
  y: Float64Array;
  creep: number;
  peak: number;
  ms: number;
  substeps: number[];
  bodies: number;
  rimBeamRatio: number;
  mass: number;
}

async function run(speed = 2, rigid = true, steps = STEPS): Promise<Run> {
  const core = await loadCore();
  const r = rover(PARAMS);
  const world = new World(core);
  const setup = new CommandBatch();
  compileTerrain(setup, landingField());
  const { joints } = compileRover(setup, r, { rigid });
  assert.equal(joints.length, 4);
  assert.equal(world.apply(setup).rejected, 0);
  const rim = new Set(r.wheels.flatMap((w) => w.nodes));
  const rimBeams = r.craft.beams.flatMap((b, k) => (rim.has(b.a) && rim.has(b.b) ? [k] : []));
  const xs = new Float64Array(steps);
  const ys = new Float64Array(steps);
  const substeps: number[] = [];
  let peak = 0;
  let rimBeamRatio = 0;
  const t0 = performance.now();
  for (let s = 0; s < steps; s++) {
    const cmds = new CommandBatch();
    if (s === DRIVE_FROM) driveRover(cmds, r, speed, CAP);
    if (s === DRIVE_TO) driveRover(cmds, r, 0, CAP);
    world.apply(cmds);
    substeps.push(world.step());
    const p = world.positions();
    let cx = 0;
    let cy = 0;
    for (const i of r.chassisNodes) {
      cx += p[3 * i]!;
      cy += p[3 * i + 1]!;
    }
    xs[s] = cx / r.chassisNodes.length;
    ys[s] = cy / r.chassisNodes.length;
    for (const v of world.joints().peak) peak = Math.max(peak, v / CAP);
    if (rigid) {
      const ratio = world.snapshot().beams.ratio;
      for (const k of rimBeams) rimBeamRatio = Math.max(rimBeamRatio, Math.abs(ratio[k]!));
    }
  }
  const ms = (performance.now() - t0) / steps;
  const bodies = world.bodies().flags.filter((f) => f !== 0).length;
  const hash = world.stateHash;
  world.free();
  // Creep: how far the chassis moved horizontally between 2 s and the start of the drive.
  const creep = Math.abs(xs[DRIVE_FROM - 1]! - xs[120]!);
  return { hash, x: xs, y: ys, creep, peak, ms, substeps, bodies, rimBeamRatio, mass: 0 };
}

test("the rover rests on the pad without creeping", async (t) => {
  const r = await run();
  t.diagnostic(`creep from 2 s to 3 s ${r.creep.toExponential(2)} m; chassis height ${r.y[DRIVE_FROM - 1]!.toFixed(4)} m; ${r.bodies} bodies`);
  assert.equal(r.bodies, 4, "four wheel bodies");
  assert.ok(r.creep < 1e-3, `creep ${r.creep} m in 1 s`);
});

test("driven, it rolls at about rotor speed times radius and stops when the rotors stop", async (t) => {
  const radius = rover(PARAMS).rollingRadius;
  // Mean speed over the last two seconds of the driven window, and over the last second of the run.
  const a = DRIVE_TO - 120;
  const b = DRIVE_TO - 1;
  for (const speed of [1, 2, 4, 8]) {
    const r = await run(speed);
    const expect = speed * radius;
    const v = (r.x[b]! - r.x[a]!) / ((b - a) / 60);
    const end = STEPS - 1;
    const vStop = Math.abs(r.x[end]! - r.x[end - 60]!);
    const slip = 1 - v / expect;
    t.diagnostic(`rotor ${speed} rad/s: ${v.toFixed(4)} m/s against ${expect.toFixed(4)} (slip ${(slip * 100).toFixed(1)}%); last second after the stop ${vStop.toExponential(2)} m; peak/cap ${r.peak.toFixed(6)}; ${r.ms.toFixed(3)} ms/step, substeps ${Math.min(...r.substeps)} to ${Math.max(...r.substeps)}`);
    assert.ok(v > 0, `rolls toward +x, not ${v}`);
    assert.ok(Math.abs(slip) < (speed === 2 ? 0.05 : 0.2), `slip ${slip} at ${speed} rad/s`);
    assert.ok(vStop < 0.05, `still rolling ${vStop} m in the last second at ${speed} rad/s`);
    assert.ok(r.peak <= 1 + 1e-9, `drive peak ${r.peak} of the cap`);
  }
});

test("the wheels' rim nodes ride their bodies: no rim beam is solved", async (t) => {
  const r = await run(2, true, DRIVE_FROM + 120);
  t.diagnostic(`worst rim beam strain ratio ${r.rimBeamRatio.toExponential(2)}`);
  assert.equal(r.bodies, 4);
  assert.ok(r.rimBeamRatio < 1e-9, `a rim beam reads ${r.rimBeamRatio}`);
});

test("the same script twice gives the same hash", async () => {
  const a = await run(2, true, DRIVE_FROM + 240);
  const b = await run(2, true, DRIVE_FROM + 240);
  assert.equal(a.hash, b.hash);
});

/**
 * A push up on one wheel's hub, from 2 s, held while it rises (the 5.7 stress test's "rock"). Returns
 * that wheel's lowest rim gap to the pad at the last step, and over the push the mechanical energy
 * gained (from finite-difference node velocities, node masses and g = 9.81) against the push's work.
 */
async function cornerPush(perNode: number, steps: number): Promise<{ gap: number; gain: number; work: number }> {
  const core = await loadCore();
  const r = rover(PARAMS);
  const world = new World(core);
  const setup = new CommandBatch();
  compileTerrain(setup, landingField());
  compileRover(setup, r);
  assert.equal(world.apply(setup).rejected, 0);
  const mass = r.craft.nodes.map((n) => n.mass);
  const hub = r.wheels[0]!.hinge.slice(3, 5);
  const dt = 1 / 60;
  const energy = (p: Float64Array, q: Float64Array) => {
    let e = 0;
    for (let k = 0; k < mass.length; k++) {
      const [vx, vy, vz] = [0, 1, 2].map((c) => (p[3 * k + c]! - q[3 * k + c]!) / dt);
      e += mass[k]! * (0.5 * (vx! * vx! + vy! * vy! + vz! * vz!) + 9.81 * p[3 * k + 1]!);
    }
    return e;
  };
  let prev = world.positions().slice();
  let work = 0;
  let e0 = 0;
  let e = 0;
  for (let s = 0; s < steps; s++) {
    const cmds = new CommandBatch();
    const on = s >= 120;
    if (on) for (const n of hub) cmds.addForce(n, 0, perNode, 0);
    world.apply(cmds);
    world.step();
    const p = world.positions().slice();
    if (on) for (const n of hub) work += perNode * (p[3 * n + 1]! - prev[3 * n + 1]!);
    e = energy(p, prev);
    if (s === 119) e0 = e;
    prev = p;
  }
  const gap = Math.min(...r.wheels[0]!.nodes.map((n) => prev[3 * n + 1]! - 0.05));
  world.free();
  return { gap, gain: e - e0, work };
}

// The 5.7 "rock" (4 kN on one hub) flips the rover. That is the physics, not the contacts: lifting a
// corner tips the rover about the opposite wheel once the push passes about half its weight (2.255 kN
// from the settled contact geometry and centre of mass, against 4.51 kN of weight), and the pivot's
// normal force adds momentum but no work. ER 005, phase 5.7 follow-up.
test("a corner push tips the rover at about half its weight, and the contacts add no energy", async (t) => {
  const hold = await cornerPush(1050, 195); // 2.1 kN, 7% under the static tipping push
  const tip = await cornerPush(1150, 195); // 2.3 kN, 2% over it
  const flip = await cornerPush(2000, 260); // 4 kN, flips it
  t.diagnostic(`wheel 0 gap after 1.25 s: ${hold.gap.toFixed(3)} m at 2.1 kN, ${tip.gap.toFixed(3)} m at 2.3 kN`);
  t.diagnostic(`4 kN over 2.3 s: energy gained ${flip.gain.toFixed(0)} J against ${flip.work.toFixed(0)} J of work (${(100 * (flip.gain / flip.work - 1)).toFixed(2)}%)`);
  assert.ok(hold.gap < 0.03, `2.1 kN lifted wheel 0 by ${hold.gap} m`);
  assert.ok(tip.gap > 0.3, `2.3 kN lifted wheel 0 only ${tip.gap} m`);
  assert.ok(flip.work > 1e4, `the push did ${flip.work} J`);
  assert.ok(flip.gain <= 1.005 * flip.work && flip.gain >= 0.97 * flip.work, `gained ${flip.gain} J for ${flip.work} J of work`);
});
