import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";
import { CommandBatch } from "../src/commands.ts";
import { Core, EVENT_BREAK, EVENT_FREEZE, EVENT_YIELD } from "../src/core.ts";
import { assemble, type Assembly } from "../src/parts/assemble.ts";
import { CRAFT_FORMAT, CRAFT_VERSION, CraftFileError, loadCraft, parseCraft, saveCraft, type CraftFile, type CraftFileRefusal } from "../src/parts/craft-file.ts";
import { engine } from "../src/parts/engine.ts";
import { leg } from "../src/parts/leg.ts";
import { compilePart, partMass, type Part } from "../src/parts/part.ts";
import { pod } from "../src/parts/pod.ts";
import { tank } from "../src/parts/tank.ts";
import { compileTerrain, flatTerrain } from "../src/terrain.ts";
import { World } from "../src/world.ts";

const WASM_PATH = new URL("../target/wasm32-unknown-unknown/release/beam_core.wasm", import.meta.url);
const FIXTURE = new URL("./fixtures/hopper.craft.json", import.meta.url);

async function loadCore(): Promise<Core> {
  return Core.load(readFileSync(WASM_PATH));
}

const HOPPER_TEXT = readFileSync(FIXTURE, "utf8");
const PLACE = { x: 0, y: 1.82, z: 0, yaw: 0 };

/** The hopper built straight from the generators, the way the assembler tests build it. */
function hopperInCode(): Assembly {
  const tk = tank({ radius: 1, length: 2.5, ringNodes: 8, rings: 5, material: "aluminium", wallArea: 5e-4, mass: 2000 });
  const [a, b, c] = tk.patches.find((p) => p.id === "side-0-0")!.nodes.map((i) => tk.nodes[i]!) as [Part["nodes"][number], Part["nodes"][number], Part["nodes"][number]];
  const dist = (p: typeof a, q: typeof a): number => Math.sqrt((p.x - q.x) ** 2 + (p.y - q.y) ** 2 + (p.z - q.z) ** 2);
  const l = leg({
    length: 2.4,
    footOffset: 1.2,
    footSize: 0.35,
    mountWidth: dist(a, b),
    mountHeight: dist(a, c),
    bodyWidth: 0.7,
    material: "aluminium",
    memberArea: 2e-4,
    mass: 150,
    sprungStiffness: 5e5,
    sprungStroke: 0.12,
    crushForce: 45e3,
    crushStroke: 0.3,
  });
  const en = engine({ ringNodes: 8, topRadius: 1, nozzleRadius: 0.5, height: 1, material: "steel", memberArea: 4e-4, mass: 800, maxThrust: 80e3 });
  const po = pod({ ringNodes: 8, bottomRadius: 1, topRadius: 0.6, height: 1.2, material: "aluminium", memberArea: 4e-4, mass: 1200 });
  return {
    parts: [{ id: "tank", part: tk }, { id: "pod", part: po }, { id: "engine", part: en }, ...[0, 1, 2, 3].map((i) => ({ id: `leg${i}`, part: l }))],
    root: "tank",
    joins: [
      { child: "pod", childPort: "bottom", parent: "tank", parentPort: "top", clock: 0 },
      { child: "engine", childPort: "top", parent: "tank", parentPort: "bottom", clock: 0 },
      ...[0, 1, 2, 3].map((i) => ({ child: `leg${i}`, childPort: "mount", parent: "tank", parentPort: `side-0-${i * 2}`, clock: 0 })),
    ],
    place: PLACE,
  };
}

function commandBytes(craft: Part): Uint8Array {
  const batch = new CommandBatch();
  compilePart(batch, craft);
  return batch.bytes.slice();
}

/** The fixture as a plain object to be broken in some way, then loaded as text. */
function broken(change: (o: any) => void): string {
  const o = JSON.parse(HOPPER_TEXT);
  change(o);
  return JSON.stringify(o);
}

function refused(source: string, code: CraftFileRefusal, reason: RegExp): void {
  let e: unknown;
  try {
    loadCraft(source);
  } catch (err) {
    e = err;
  }
  assert.ok(e instanceof CraftFileError, `the file was not refused with a craft file error: ${String(e)}`);
  assert.equal(e.code, code, e.message);
  assert.match(e.message, reason);
}

test("the committed hopper loads, and saving it again gives the same text, byte for byte", () => {
  const { file, craft } = loadCraft(HOPPER_TEXT);
  assert.equal(saveCraft(file), HOPPER_TEXT);
  assert.ok(HOPPER_TEXT.endsWith("}\n") && !HOPPER_TEXT.includes("\r"), "LF line endings, one newline at the end");
  assert.equal(file.parts.length, 7);
  assert.equal(file.joins.length, 6);
  assert.equal(craft.nodes.length, 123);
  assert.equal(craft.beams.length, 612);
  assert.ok(partMass(craft) === 4600);
  // Parse then save with no assembling gives the same text too, and twice in a row too.
  assert.equal(saveCraft(parseCraft(HOPPER_TEXT)), HOPPER_TEXT);
  assert.equal(saveCraft(parseCraft(saveCraft(parseCraft(HOPPER_TEXT)))), HOPPER_TEXT);
});

test("the keys of a file may come in any order, and saving puts them in the canonical one", () => {
  const o = JSON.parse(HOPPER_TEXT);
  const reverse = (v: any): any => {
    if (Array.isArray(v)) return v.map(reverse);
    if (v !== null && typeof v === "object") return Object.fromEntries(Object.entries(v).reverse().map(([k, x]) => [k, reverse(x)]));
    return v;
  };
  const shuffled = JSON.stringify(reverse(o), null, 4);
  assert.notEqual(shuffled, HOPPER_TEXT);
  assert.equal(saveCraft(parseCraft(shuffled)), HOPPER_TEXT);
});

test("the hopper in the file is the hopper in code: the same commands and the same state after 600 steps", async () => {
  const fromFile = loadCraft(HOPPER_TEXT).craft;
  const fromCode = assemble(hopperInCode());
  assert.deepEqual(commandBytes(fromFile), commandBytes(fromCode));
  assert.equal(JSON.stringify(fromFile), JSON.stringify(fromCode));

  // The landing field arrives in phase 4.7; until then the flat test ground stands in for it.
  const run = async (craft: Part): Promise<{ hash: bigint; bad: number }> => {
    const world = new World(await loadCore());
    const build = new CommandBatch();
    compileTerrain(build, flatTerrain(41, 41, -20, -20, 1));
    compilePart(build, craft);
    assert.equal(world.apply(build).rejected, 0);
    let bad = 0;
    for (let s = 0; s < 600; s++) {
      world.step();
      for (const e of world.events()) if (e.kind === EVENT_YIELD || e.kind === EVENT_BREAK || e.kind === EVENT_FREEZE) bad++;
    }
    const hash = world.stateHash;
    world.free();
    return { hash, bad };
  };
  const a = await run(fromFile);
  const b = await run(fromCode);
  assert.equal(a.bad, 0, "the hopper settles on the ground without a yield");
  assert.equal(a.hash, b.hash);
  assert.notEqual(a.hash, 0n);
});

test("every part type and every optional field survives save and load", () => {
  const file: CraftFile = {
    root: "a",
    place: { x: -3.5, y: 0.25, z: 7, yaw: -4 },
    parts: [
      { id: "a", type: "truss", params: { section: "square", length: 4, width: 1, bays: 2, material: "steel", memberArea: 2e-4, nodeRadius: 0.04 } },
      { id: "b", type: "truss", params: { section: "square", length: 2, width: 1, bays: 1, material: "carbon", memberArea: 2e-4, mass: 200 } },
      { id: "p", type: "pod", params: { ringNodes: 4, bottomRadius: 0.7071, topRadius: 0.3, height: 0.6, apexRise: 0.2, material: "aluminium", memberArea: 2e-4, mass: 300, nodeRadius: 0.03 } },
    ],
    joins: [
      { child: "b", childPort: "bottom", parent: "a", parentPort: "top", clock: 1 },
      { child: "p", childPort: "bottom", parent: "b", parentPort: "top", clock: -1 },
    ],
  };
  const out = saveCraft(file);
  const back = parseCraft(out);
  assert.deepEqual(back, file);
  assert.equal(saveCraft(back), out);
  assert.match(out, /"apexRise": 0\.2/);
  assert.doesNotMatch(out, /undefined/);
  const loaded = loadCraft(out);
  assert.ok(loaded.craft.nodes.length > 0);
  assert.deepEqual(commandBytes(loaded.craft), commandBytes(loadCraft(out).craft));
  assert.equal(CRAFT_FORMAT, "ribcage-craft");
  assert.equal(CRAFT_VERSION, 1);
});

test("a file that is not a craft file is refused with a reason", () => {
  refused("{not json", "syntax", /not valid JSON/);
  refused("", "syntax", /not valid JSON/);
  refused("[]", "shape", /an object, not an array/);
  refused("null", "shape", /an object, not null/);
  refused(broken((o) => (o.format = "other")), "format", /"other", not "ribcage-craft"/);
  refused(broken((o) => delete o.format), "format", /format/);
});

test("a later or a nonsense version is refused", () => {
  refused(broken((o) => (o.version = 2)), "version", /version 2.*up to 1/);
  refused(broken((o) => (o.version = 99)), "version", /version 99/);
  refused(broken((o) => (o.version = 0)), "version", /not a whole number from 1/);
  refused(broken((o) => (o.version = 1.5)), "version", /1\.5/);
  refused(broken((o) => (o.version = "1")), "version", /"1"/);
  // A later version may have fields this one has never heard of: the version is what is reported.
  refused(broken((o) => { o.version = 2; o.damage = []; }), "version", /version 2/);
});

test("an unknown field, a missing field or a field of the wrong shape is refused where it is", () => {
  refused(broken((o) => (o.damage = [])), "unknown-field", /the craft file has a field "damage"/);
  refused(broken((o) => (o.parts[1].colour = "red")), "unknown-field", /parts\[1\] has a field "colour"/);
  refused(broken((o) => (o.parts[0].params.wallThickness = 0.1)), "unknown-field", /parts\[0\]\.params has a field "wallThickness"/);
  refused(broken((o) => (o.joins[2].spring = 1)), "unknown-field", /joins\[2\] has a field "spring"/);
  refused(broken((o) => (o.place.roll = 1)), "unknown-field", /place has a field "roll"/);
  refused(broken((o) => delete o.joins), "missing-field", /missing "joins"/);
  refused(broken((o) => delete o.root), "missing-field", /missing "root"/);
  refused(broken((o) => delete o.parts[0].params.radius), "missing-field", /parts\[0\]\.params is missing "radius"/);
  refused(broken((o) => delete o.joins[0].clock), "missing-field", /joins\[0\] is missing "clock"/);
  refused(broken((o) => delete o.place.yaw), "missing-field", /place is missing "yaw"/);
  refused(broken((o) => (o.parts = {})), "shape", /parts must be a list/);
  refused(broken((o) => (o.joins = "none")), "shape", /joins must be a list/);
  refused(broken((o) => (o.parts[0] = 5)), "shape", /parts\[0\] must be an object/);
  refused(broken((o) => (o.place = [0, 0, 0, 0])), "shape", /place must be an object/);
});

test("an unknown part type is refused with the types there are", () => {
  refused(broken((o) => (o.parts[1].type = "wing")), "unknown-part-type", /parts\[1\] \(pod\) has the part type "wing".*tank, truss, leg, engine, pod/);
  refused(broken((o) => (o.parts[1].type = 3)), "unknown-part-type", /3/);
  refused(broken((o) => (o.parts[1].type = "constructor")), "unknown-part-type", /"constructor"/);
});

test("a bad number, material or name is refused, and so is a number that is not finite", () => {
  refused(broken((o) => (o.parts[0].params.radius = "1")), "bad-value", /parts\[0\]\.params\.radius must be a finite number, not "1"/);
  refused(broken((o) => (o.parts[0].params.radius = null)), "bad-value", /radius.*null/);
  refused(broken((o) => (o.place.y = true)), "bad-value", /place\.y/);
  refused(broken((o) => (o.parts[0].params.material = "wood")), "bad-value", /material must be one of steel, aluminium, carbon, not "wood"/);
  refused(broken((o) => (o.parts[0].params.material = "constructor")), "bad-value", /"constructor"/);
  refused(broken((o) => (o.parts[0].id = "")), "bad-value", /parts\[0\]\.id/);
  refused(broken((o) => (o.joins[0].parent = 4)), "bad-value", /joins\[0\]\.parent/);
  // JSON has no NaN or Infinity, but a number too big for a double reads as Infinity.
  refused(HOPPER_TEXT.replace('"mass": 2000', '"mass": 1e999'), "bad-value", /parts\[0\]\.params\.mass must be a finite number, not Infinity/);
  refused(HOPPER_TEXT.replace('"y": 1.82', '"y": -1e999'), "bad-value", /place\.y.*-Infinity/);
  refused(broken((o) => (o.parts = o.parts.map((p: any) => (p.type === "tank" ? { ...p, params: { ...p.params, section: "square" } } : p)))), "unknown-field", /section/);
});

test("a part the generator refuses refuses the file, with the generator's reason", () => {
  refused(broken((o) => (o.parts[0].params.radius = -1)), "bad-part", /part tank \(tank\): .*radius/);
  refused(broken((o) => (o.parts[0].params.ringNodes = 7)), "bad-part", /part tank \(tank\)/);
  refused(broken((o) => (o.parts[0].params.mass = 1)), "bad-part", /part tank \(tank\): .*mass/);
  refused(broken((o) => (o.parts[2].params.maxThrust = -5)), "bad-part", /part engine \(engine\)/);
});

test("joins the assembler refuses refuse the file, with the assembler's reason", () => {
  refused(broken((o) => (o.parts[2].params.ringNodes = 12)), "bad-join", /node count/);
  refused(broken((o) => (o.parts[2].params.topRadius = 1.5)), "bad-join", /radius/);
  refused(broken((o) => (o.joins[0].clock = 0.5)), "bad-join", /whole number/);
  refused(broken((o) => (o.joins[0].childPort = "nowhere")), "bad-join", /pod has no ring or patch nowhere/);
  refused(broken((o) => (o.joins[0].child = "ghost")), "bad-join", /ghost/);
  refused(broken((o) => (o.root = "ghost")), "bad-join", /ghost/);
  refused(broken((o) => o.joins.push({ ...o.joins[0] })), "bad-join", /pod is attached twice/);
  refused(broken((o) => (o.joins[1].parentPort = "top")), "bad-join", /already used/);
  refused(broken((o) => o.parts.push({ ...o.parts[1] })), "bad-join", /two parts are called pod/);
  refused(broken((o) => o.parts.pop()), "bad-join", /leg3/);
  refused(broken((o) => (o.place.yaw = 0.5)), "bad-join", /whole number of 15°/);
});

test("saving refuses what loading would refuse, and writes nothing", () => {
  const file = parseCraft(HOPPER_TEXT);
  const nan = structuredClone(file);
  (nan.parts[0]!.params as { radius: number }).radius = Number.NaN;
  assert.throws(() => saveCraft(nan), (e: unknown) => e instanceof CraftFileError && e.code === "bad-value" && /radius/.test(e.message));
  const inf = structuredClone(file);
  inf.place.x = Number.POSITIVE_INFINITY;
  assert.throws(() => saveCraft(inf), (e: unknown) => e instanceof CraftFileError && e.code === "bad-value");
  const wing = structuredClone(file) as { parts: { type: string }[] };
  wing.parts[0]!.type = "wing";
  assert.throws(() => saveCraft(wing as unknown as CraftFile), (e: unknown) => e instanceof CraftFileError && e.code === "unknown-part-type");
  const extra = structuredClone(file) as unknown as { joins: Record<string, unknown>[] };
  extra.joins[0]!.spring = 1;
  assert.throws(() => saveCraft(extra as unknown as CraftFile), (e: unknown) => e instanceof CraftFileError && e.code === "unknown-field");
  // The file given is left as it was, and saves as before.
  assert.equal(saveCraft(file), HOPPER_TEXT);
});

test("loading builds a new craft each time and never changes the file's text or its parsed form", () => {
  const first = loadCraft(HOPPER_TEXT);
  const snapshot = JSON.stringify(first.file);
  const second = loadCraft(HOPPER_TEXT);
  assert.notEqual(first.craft, second.craft);
  assert.equal(JSON.stringify(first.craft), JSON.stringify(second.craft));
  first.craft.nodes[0]!.mass = 1;
  assert.equal(JSON.stringify(first.file), snapshot);
  assert.ok(partMass(loadCraft(HOPPER_TEXT).craft) === 4600);
});
