import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";
import { CommandBatch } from "../src/commands.ts";
import { Core, EVENT_BREAK, EVENT_FREEZE, EVENT_YIELD } from "../src/core.ts";
import { AssemblyError, CRAFT_NODE_MASS_RATIO_BUDGET, assemble, type Assembly, type AssemblyJoin, type AssemblyRefusal, type Craft } from "../src/parts/assemble.ts";
import { engine } from "../src/parts/engine.ts";
import { leg } from "../src/parts/leg.ts";
import { NODE_MASS_RATIO_BUDGET, SUBSTEP_BUDGET, compilePart, partBudget, partMass, type Part } from "../src/parts/part.ts";
import { pod } from "../src/parts/pod.ts";
import { ringPoint, unitCosSin } from "../src/parts/shared.ts";
import { tank } from "../src/parts/tank.ts";
import { compileTerrain, flatTerrain } from "../src/terrain.ts";
import { World } from "../src/world.ts";

const WASM_PATH = new URL("../target/wasm32-unknown-unknown/release/beam_core.wasm", import.meta.url);
const STEP = 1 / 60;

async function loadCore(): Promise<Core> {
  return Core.load(readFileSync(WASM_PATH));
}

const TANK = tank({ radius: 1, length: 2.5, ringNodes: 8, rings: 5, material: "aluminium", wallArea: 5e-4, mass: 2000 });
const ENGINE = engine({ ringNodes: 8, topRadius: 1, nozzleRadius: 0.5, height: 1, material: "steel", memberArea: 4e-4, mass: 800, maxThrust: 80e3 });
const POD = pod({ ringNodes: 8, bottomRadius: 1, topRadius: 0.6, height: 1.2, material: "aluminium", memberArea: 4e-4, mass: 1200 });

function dist(a: readonly number[], b: readonly number[]): number {
  return Math.sqrt((a[0]! - b[0]!) ** 2 + (a[1]! - b[1]!) ** 2 + (a[2]! - b[2]!) ** 2);
}

/** A hopper leg whose mount is exactly the size of one wall quad of the tank's lowest bay. */
function hopperLeg(): Part {
  const [a, b, c] = TANK.patches.find((p) => p.id === "side-0-0")!.nodes.map((i) => TANK.nodes[i]!) as [Part["nodes"][number], Part["nodes"][number], Part["nodes"][number]];
  return leg({
    length: 3,
    footOffset: 1.5,
    footSize: 0.35,
    mountWidth: dist([a.x, a.y, a.z], [b.x, b.y, b.z]),
    mountHeight: dist([a.x, a.y, a.z], [c.x, c.y, c.z]),
    bodyWidth: 0.9,
    material: "aluminium",
    memberArea: 2e-4,
    mass: 150,
    sprungStiffness: 3e5,
    sprungStroke: 0.3,
    crushForce: 75e3,
    crushStroke: 0.15,
  });
}

const LEG_POINTS = [0, 2, 4, 6];

/** The hopper from its seven parts: pod on the tank's top ring, engine on its bottom ring, a leg on every other wall quad of the lowest bay. */
function hopper(place?: Assembly["place"]): Assembly {
  const l = hopperLeg();
  const a: Assembly = {
    parts: [
      { id: "tank", part: TANK },
      { id: "pod", part: POD },
      { id: "engine", part: ENGINE },
      ...LEG_POINTS.map((j) => ({ id: `leg${j / 2}`, part: l })),
    ],
    root: "tank",
    joins: [
      { child: "pod", childPort: "bottom", parent: "tank", parentPort: "top", clock: 0 },
      { child: "engine", childPort: "top", parent: "tank", parentPort: "bottom", clock: 0 },
      ...LEG_POINTS.map((j): AssemblyJoin => ({ child: `leg${j / 2}`, childPort: "mount", parent: "tank", parentPort: `side-0-${j}`, clock: 0 })),
    ],
  };
  if (place !== undefined) a.place = place;
  return a;
}

function refusal(a: Assembly): AssemblyError {
  try {
    assemble(a);
  } catch (e) {
    if (e instanceof AssemblyError) return e;
    throw e;
  }
  assert.fail("the assembly was not refused");
}

function expectRefused(a: Assembly, code: AssemblyRefusal, reason: RegExp): void {
  const before = JSON.stringify(a);
  const e = refusal(a);
  assert.equal(e.code, code, e.message);
  assert.match(e.message, reason);
  assert.equal(JSON.stringify(a), before, "a refused assembly changes nothing");
}

test("the hopper assembles from its seven parts: nodes, mass, tree and segments", () => {
  const a = hopper();
  const before = JSON.stringify(a);
  const craft = assemble(a);
  assert.equal(JSON.stringify(a), before, "the parts given are not touched");

  const sumNodes = a.parts.reduce((n, p) => n + p.part.nodes.length, 0);
  const merged = craft.tree.reduce((n, e) => n + e.merged, 0);
  assert.equal(merged, 8 + 8 + 4 * 4);
  assert.equal(craft.nodes.length, sumNodes - merged);
  assert.equal(craft.nodes.length, 123);

  let total = 0;
  for (const p of a.parts) total += partMass(p.part);
  assert.equal(total, 4600);
  assert.ok(partMass(craft) === total, `the craft weighs ${partMass(craft)} kg, the parts ${total} kg`);
  for (const n of craft.nodes) assert.ok(n.mass > 0);

  assert.deepEqual(craft.tree.map((e) => [e.id, e.parent]), [["tank", null], ["pod", 0], ["engine", 0], ["leg0", 0], ["leg1", 0], ["leg2", 0], ["leg3", 0]]);
  assert.equal(craft.segments.length, a.parts.reduce((n, p) => n + p.part.segments.length, 0));
  // Segments and never-live flags carried through, merged nodes keeping the parent's segment.
  const podEntry = craft.tree[1]!;
  const engineEntry = craft.tree[2]!;
  assert.equal(craft.segments[podEntry.segmentBase]!.neverLive, true);
  assert.equal(craft.segments[engineEntry.segmentBase]!.neverLive, true);
  assert.equal(craft.segments.filter((s) => s.neverLive).length, 2);
  for (const [i, n] of POD.nodes.entries()) {
    const c = craft.nodes[podEntry.nodeOf[i]!]!;
    if (i < 8) assert.ok(c.segment === 3, "the pod's ring nodes keep the tank's segment");
    else assert.equal(c.segment, podEntry.segmentBase + n.segment);
  }
  // Seams: the parts' own, then one per join, between the parent's segment and the child's.
  assert.equal(craft.seams.length, TANK.seams.length + 6);
  for (const e of craft.tree.slice(1)) {
    const s = craft.seams[e.seam!]!;
    assert.equal(s.segmentB, e.segmentBase);
    assert.ok(s.area > 0 && s.bending >= 0);
    assert.ok(s.segmentA < e.segmentBase);
  }
  assert.equal(craft.tree[0]!.seam, null);
  // The engine's thruster follows its nodes.
  assert.equal(craft.thrusters.length, 1);
  assert.equal(craft.thrusters[0]!.maxThrust, 80e3);
  assert.equal(craft.thrusters[0]!.tail, engineEntry.nodeOf[ENGINE.thrusters[0]!.tail]);
  assert.equal(craft.thrusters[0]!.head, engineEntry.nodeOf[ENGINE.thrusters[0]!.head]);
  // Everything is used: no free ring is left on the tank, pod or engine that was joined.
  assert.deepEqual(craft.rings.map((r) => r.id), []);
  assert.equal(craft.patches.length, TANK.patches.length - LEG_POINTS.length);
});

test("merged nodes sit within 1e-9 m of where both parts put them", () => {
  const craft = assemble(hopper({ x: 3, y: 5, z: -2, yaw: 5 }));
  for (const e of craft.tree.slice(1)) {
    const parent = craft.tree[e.parent!]!;
    let seen = 0;
    for (const [i, node] of e.nodeOf.entries()) {
      const owner = craft.nodes[node]!;
      if (node >= craft.nodes.length) assert.fail("a node is missing");
      const isMerged = parent.nodeOf.includes(node);
      if (!isMerged) continue;
      seen++;
      assert.ok(dist([owner.x, owner.y, owner.z], e.placed[i]!) < 1e-9, `${e.id} node ${i} is ${dist([owner.x, owner.y, owner.z], e.placed[i]!)} m from the parent's`);
    }
    assert.equal(seen, e.merged, `${e.id}: every merged node is checked`);
  }
});

test("the child's ring beams are gone, and every beam touching a merged node is marked", () => {
  const a = hopper();
  const craft = assemble(a);
  const total = a.parts.reduce((n, p) => n + p.part.beams.length, 0);
  const dropped = craft.tree.reduce((n, e) => n + e.droppedBeams, 0);
  assert.equal(craft.beams.length, total - dropped);
  // A ring of 8 has 8 perimeter, 8 chord and 4 diameter beams; a leg's mount has 4 + 2.
  assert.deepEqual(craft.tree.map((e) => e.droppedBeams), [0, 20, 20, 6, 6, 6, 6]);
  for (const [bi, b] of craft.beams.entries()) {
    const e = craft.tree[b.part]!;
    const mergedHere = new Set(e.nodeOf.filter((n) => craft.tree[e.parent ?? 0]!.nodeOf.includes(n) && e.parent !== null));
    const touches = e.parent !== null && (mergedHere.has(b.a) || mergedHere.has(b.b));
    assert.equal(b.seam, touches, `beam ${bi} of ${e.id}`);
    assert.equal(e.seamBeams.includes(bi), touches);
  }
  assert.deepEqual(craft.tree.map((e) => e.seamBeams.length), [0, 24, 40, 12, 12, 12, 12]);
  // A child's own ring beams are not beams of the craft between two merged nodes.
  const podEntry = craft.tree[1]!;
  const ringNodes = new Set(POD.rings[0]!.nodes.map((i) => podEntry.nodeOf[i]!));
  for (const b of craft.beams) if (b.part === 1) assert.ok(!(ringNodes.has(b.a) && ringNodes.has(b.b)), "no beam of the pod joins two nodes of the weld ring");
  // The parent's ring beams stay: the tank's top ring is still braced.
  const topNodes = new Set(TANK.rings.find((r) => r.id === "top")!.nodes);
  assert.equal(craft.beams.filter((b) => b.part === 0 && topNodes.has(b.a) && topNodes.has(b.b)).length, 20);
});

test("the hopper is within the budget, and its numbers are the ones logged", () => {
  const craft = assemble(hopper());
  const b = partBudget(craft);
  assert.equal(craft.budget.substeps, b.substeps);
  assert.equal(craft.budget.massRatio, b.massRatio);
  assert.ok(craft.budget.withinBudget, craft.budget.problems.join("; "));
  assert.ok(b.massRatio > NODE_MASS_RATIO_BUDGET, "the hopper is over a single part's ratio, which is why a craft has its own");
  assert.ok(b.substeps <= SUBSTEP_BUDGET, `${b.substeps} substeps`);
  assert.ok(b.massRatio <= CRAFT_NODE_MASS_RATIO, `ratio ${b.massRatio}`);
  assert.equal(craft.nodes.length, 123);
  assert.equal(craft.beams.length, 612);
});

const CRAFT_NODE_MASS_RATIO = CRAFT_NODE_MASS_RATIO_BUDGET;

test("a craft over the budget is refused: a pod far heavier than the tank", () => {
  const heavy = pod({ ringNodes: 8, bottomRadius: 1, topRadius: 0.6, height: 1.2, material: "aluminium", memberArea: 4e-4, mass: 400000 });
  const a: Assembly = {
    parts: [{ id: "tank", part: TANK }, { id: "pod", part: heavy }],
    root: "tank",
    joins: [{ child: "pod", childPort: "bottom", parent: "tank", parentPort: "top", clock: 0 }],
  };
  expectRefused(a, "mass-ratio", /times the lightest, over the craft budget of 20/);
});

test("each clock turns the child by whole ring steps: the engine's nozzle node 0 goes round the ring", () => {
  for (let k = -9; k < 17; k++) {
    const a = hopper();
    a.joins = a.joins.map((j) => (j.child === "engine" ? { ...j, clock: k } : j));
    const craft = assemble(a);
    const e = craft.tree[2]!;
    const [x, z] = ringPoint(8, ((k % 8) + 8) % 8, 0.5);
    const p = craft.nodes[e.nodeOf[0]!]!;
    assert.ok(dist([p.x, p.y, p.z], [x, -1, z]) < 1e-9, `clock ${k}: nozzle node 0 at ${p.x}, ${p.y}, ${p.z}`);
    // The ring itself lands on the ring: engine ring node j on the tank's bottom ring node j + k.
    for (let j = 0; j < 8; j++) assert.equal(e.nodeOf[8 + j], TANK_BOTTOM[(((j + k) % 8) + 8) % 8]);
  }
  // The pod, turned two steps, has its marked top node 0 two steps round.
  const a = hopper();
  a.joins = a.joins.map((j) => (j.child === "pod" ? { ...j, clock: 2 } : j));
  const craft = assemble(a);
  const p = craft.nodes[craft.tree[1]!.nodeOf[8]!]!;
  const [x, z] = ringPoint(8, 2, 0.6);
  assert.ok(dist([p.x, p.y, p.z], [x, 2.5 + 1.2, z]) < 1e-9);
});
const TANK_BOTTOM = TANK.rings.find((r) => r.id === "bottom")!.nodes;

test("two rings that run the other way round are paired the other way round, and still meet", () => {
  // A second tank joined top to top is upside down: its ring runs the other way seen along the shared axis.
  const upper = tank({ radius: 1, length: 1, ringNodes: 8, rings: 2, material: "aluminium", wallArea: 5e-4, mass: 300 });
  for (let k = 0; k < 8; k++) {
    const a: Assembly = {
      parts: [{ id: "low", part: TANK }, { id: "high", part: upper }],
      root: "low",
      joins: [{ child: "high", childPort: "top", parent: "low", parentPort: "top", clock: k }],
    };
    const craft = assemble(a);
    const e = craft.tree[1]!;
    const topRing = TANK.rings.find((r) => r.id === "top")!.nodes;
    const childTop = upper.rings.find((r) => r.id === "top")!.nodes;
    for (let j = 0; j < 8; j++) assert.equal(e.nodeOf[childTop[j]!], craft.tree[0]!.nodeOf[topRing[(((k - j) % 8) + 8) % 8]!], `clock ${k}, node ${j}`);
    // The child's far ring is above the join, at twice the height.
    const far = craft.nodes[e.nodeOf[upper.rings.find((r) => r.id === "bottom")!.nodes[0]!]!]!;
    assert.ok(Math.abs(far.y - 3.5) < 1e-9, `far ring at ${far.y}`);
    assert.equal(craft.nodes.length, TANK.nodes.length + upper.nodes.length - 8);
  }
});

test("the root's placement moves and turns the whole craft, by whole 15° steps", () => {
  const plain = assemble(hopper());
  const placed = assemble(hopper({ x: 3, y: 5, z: -2, yaw: 3 }));
  const [c, s] = unitCosSin(6);
  assert.equal(placed.nodes.length, plain.nodes.length);
  for (const [i, n] of plain.nodes.entries()) {
    const m = placed.nodes[i]!;
    assert.ok(dist([m.x, m.y, m.z], [3 + c * n.x - s * n.z, 5 + n.y, -2 + s * n.x + c * n.z]) < 1e-9, `node ${i}`);
    assert.equal(m.mass, n.mass);
  }
  assert.ok(Math.abs(placed.tree[3]!.capsule!.a[1] - (5 + plain.tree[3]!.capsule!.a[1])) < 1e-9);
  expectRefused({ ...hopper(), place: { x: 0, y: 0, z: 0, yaw: 0.5 } }, "bad-clock", /whole number of 15°/);
});

test("the same parts and joins give the same craft, bit for bit, however many times", () => {
  const one = assemble(hopper());
  const two = assemble(hopper());
  assert.equal(JSON.stringify(one), JSON.stringify(two));
  const bytes = (c: Craft): Uint8Array => {
    const batch = new CommandBatch();
    compilePart(batch, c);
    return batch.bytes.slice();
  };
  assert.deepEqual(bytes(one), bytes(two));
  assert.ok(bytes(one).length > 0);
});

test("a join list in any order gives the same craft once parents come first", () => {
  const forward = hopper();
  const back = hopper();
  back.joins = [...back.joins].reverse();
  // The legs are listed before the engine and pod now, so the node order differs, but the craft is
  // the same size and weighs the same.
  const f = assemble(forward);
  const b = assemble(back);
  assert.equal(b.nodes.length, f.nodes.length);
  assert.equal(b.beams.length, f.beams.length);
  assert.ok(partMass(b) === partMass(f));
});

test("a refused assembly says why and changes nothing", () => {
  const l = hopperLeg();
  const wide = tank({ radius: 1.5, length: 1, ringNodes: 8, rings: 2, material: "aluminium", wallArea: 5e-4, mass: 300 });
  const twelve = tank({ radius: 1, length: 1, ringNodes: 12, rings: 2, material: "aluminium", wallArea: 5e-4, mass: 300 });
  const close = tank({ radius: 1.0005, length: 1, ringNodes: 8, rings: 2, material: "aluminium", wallArea: 5e-4, mass: 300 });
  const pair = (child: Part, port = "bottom", clock = 0): Assembly => ({
    parts: [{ id: "a", part: TANK }, { id: "b", part: child }],
    root: "a",
    joins: [{ child: "b", childPort: port, parent: "a", parentPort: "top", clock }],
  });

  expectRefused(pair(wide), "radius", /radius.*1\.5 m.*1 m/);
  expectRefused(pair(twelve), "node-count", /node count.*12.*8/);
  assert.doesNotThrow(() => assemble(pair(close)), "rings within 1 mm are welded");

  const hop = hopper();
  expectRefused({ ...hop, joins: [...hop.joins, { child: "pod", childPort: "bottom", parent: "tank", parentPort: "top", clock: 0 }] }, "attached-twice", /pod is attached twice/);
  expectRefused({ ...hop, joins: [...hop.joins, { child: "leg0", childPort: "mount", parent: "leg1", parentPort: "mount", clock: 0 }] }, "attached-twice", /leg0 is attached twice/);

  // Ring already used: a second pod on the tank's top ring.
  const two: Assembly = {
    parts: [{ id: "tank", part: TANK }, { id: "p1", part: POD }, { id: "p2", part: POD }],
    root: "tank",
    joins: [
      { child: "p1", childPort: "bottom", parent: "tank", parentPort: "top", clock: 0 },
      { child: "p2", childPort: "bottom", parent: "tank", parentPort: "top", clock: 1 },
    ],
  };
  expectRefused(two, "port-used", /tank top is already used/);
  // A patch that shares a node with a patch already used.
  const crowded: Assembly = {
    parts: [{ id: "tank", part: TANK }, { id: "l1", part: l }, { id: "l2", part: l }],
    root: "tank",
    joins: [
      { child: "l1", childPort: "mount", parent: "tank", parentPort: "side-0-0", clock: 0 },
      { child: "l2", childPort: "mount", parent: "tank", parentPort: "side-0-1", clock: 0 },
    ],
  };
  expectRefused(crowded, "port-used", /shares a node with side-0-0/);
  // The same patch twice.
  expectRefused({ ...crowded, joins: [crowded.joins[0]!, { ...crowded.joins[1]!, parentPort: "side-0-0" }] }, "port-used", /side-0-0 is already used/);

  // A cycle: a is b's child and b is a's child, apart from the root.
  const cyc: Assembly = {
    parts: [{ id: "tank", part: TANK }, { id: "x", part: POD }, { id: "y", part: POD }],
    root: "tank",
    joins: [
      { child: "x", childPort: "bottom", parent: "y", parentPort: "bottom", clock: 0 },
      { child: "y", childPort: "bottom", parent: "x", parentPort: "bottom", clock: 0 },
    ],
  };
  expectRefused(cyc, "cycle", /cycle/);
  expectRefused({ ...two, joins: [{ child: "tank", childPort: "bottom", parent: "p1", parentPort: "bottom", clock: 0 }, two.joins[0]!] }, "cycle", /tank is its own ancestor/);
  expectRefused({ ...two, joins: [{ child: "p1", childPort: "bottom", parent: "p1", parentPort: "bottom", clock: 0 }] }, "self-join", /joined to itself/);

  // A part that no join reaches.
  expectRefused({ ...two, joins: [two.joins[0]!] }, "disconnected", /p2 is not connected to the root tank/);

  // Names and ports, on an assembly of one pod on one tank.
  const one: Assembly = { parts: [{ id: "tank", part: TANK }, { id: "p1", part: POD }], root: "tank", joins: [two.joins[0]!] };
  expectRefused({ ...one, joins: [{ ...one.joins[0]!, child: "ghost" }] }, "unknown-part", /ghost/);
  expectRefused({ ...one, root: "ghost" }, "no-root", /ghost/);
  expectRefused({ ...one, parts: [...one.parts, { id: "p1", part: POD }] }, "duplicate-part", /two parts are called p1/);
  expectRefused({ ...one, joins: [{ ...one.joins[0]!, childPort: "nowhere" }] }, "unknown-port", /p1 has no ring or patch nowhere/);
  expectRefused({ ...one, joins: [{ ...one.joins[0]!, parentPort: "nowhere" }] }, "unknown-port", /tank has no ring or patch nowhere/);
  expectRefused({ ...one, joins: [{ ...one.joins[0]!, clock: 0.5 }] }, "bad-clock", /whole number of ring steps/);
  expectRefused({ ...one, joins: [{ ...one.joins[0]!, clock: Number.NaN }] }, "bad-clock", /whole number/);

  // A ring against a patch, a clock on a patch, a mount of the wrong size, a patch on the wrong side.
  const solo: Assembly = { parts: [{ id: "tank", part: TANK }, { id: "l1", part: l }], root: "tank", joins: [crowded.joins[0]!] };
  expectRefused({ ...solo, joins: [{ child: "l1", childPort: "mount", parent: "tank", parentPort: "top", clock: 0 }] }, "port-kind", /ring cannot be joined to a mount patch/);
  expectRefused({ ...solo, joins: [{ ...solo.joins[0]!, clock: 1 }] }, "patch-clock", /no clock/);
  const small = leg({ length: 3, footOffset: 1.5, footSize: 0.35, mountWidth: 0.35, mountHeight: 0.35, bodyWidth: 0.9, material: "aluminium", memberArea: 3e-4, mass: 150, sprungStiffness: 3e5, sprungStroke: 0.3, crushForce: 75e3, crushStroke: 0.15 });
  expectRefused({ ...solo, parts: [{ id: "tank", part: TANK }, { id: "l1", part: small }] }, "patch-shape", /l1 node \d lands .* m from the tank node it joins/);
  // A ring that is not where its axis says: a pod whose ring is moved away.
  const bent: Part = { ...POD, nodes: POD.nodes.map((n, i) => (i === 3 ? { ...n, x: n.x + 0.01 } : n)) };
  expectRefused(pair(bent), "ring-shape", /node 3 lands .* m from the a node it joins/);

  // An assembly refused part-way leaves the next one unharmed.
  const ok = assemble(hopper());
  assert.equal(ok.nodes.length, 123);
});

test("a refused assembly leaves the parts exactly as they were, node by node", () => {
  const snapshot = JSON.stringify([TANK, POD, ENGINE]);
  const a = hopper();
  a.joins = [...a.joins, { child: "pod", childPort: "bottom", parent: "tank", parentPort: "top", clock: 3 }];
  assert.throws(() => assemble(a), AssemblyError);
  assert.equal(JSON.stringify([TANK, POD, ENGINE]), snapshot);
  assemble(hopper());
  assert.equal(JSON.stringify([TANK, POD, ENGINE]), snapshot);
});

test("the hopper stands on flat ground for 20 s with no yield and comes to rest, within the substep budget", async () => {
  const flat = assemble(hopper());
  let lowest = Infinity;
  for (const n of flat.nodes) lowest = Math.min(lowest, n.y - n.radius);
  const craft = assemble(hopper({ x: 0, y: -lowest + 0.001, z: 0, yaw: 0 }));
  const world = new World(await loadCore());
  const build = new CommandBatch();
  compileTerrain(build, flatTerrain(41, 41, -20, -20, 1));
  compilePart(build, craft);
  assert.equal(world.apply(build).rejected, 0);
  assert.equal(world.nodeCount, craft.nodes.length);
  assert.equal(world.beamCount, craft.beams.length);
  let bad = 0;
  let prev = world.positions();
  let endSpeed = 0;
  let worst = 0;
  for (let s = 0; s < 1200; s++) {
    worst = Math.max(worst, world.step());
    for (const e of world.events()) if (e.kind === EVENT_YIELD || e.kind === EVENT_BREAK || e.kind === EVENT_FREEZE) bad++;
    const p = world.positions();
    let fastest = 0;
    for (let i = 0; i < p.length; i += 3) {
      const d = Math.sqrt((p[i]! - prev[i]!) ** 2 + (p[i + 1]! - prev[i + 1]!) ** 2 + (p[i + 2]! - prev[i + 2]!) ** 2);
      fastest = Math.max(fastest, d / STEP);
    }
    endSpeed = fastest;
    prev = p;
  }
  assert.equal(bad, 0, "no yield, break or freeze");
  assert.ok(endSpeed < 1e-3, `still moving at ${endSpeed} m/s after 20 s`);
  assert.ok(worst <= SUBSTEP_BUDGET, `the core used ${worst} substeps`);
  assert.equal(worst, craft.budget.substeps, "the core picks the substep count the budget reports");
  for (let i = 1; i < prev.length; i += 3) assert.ok(prev[i]! > -0.01, "above the ground");
  world.free();
});

void NODE_MASS_RATIO_BUDGET;
