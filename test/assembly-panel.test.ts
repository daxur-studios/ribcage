import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";
import { AssemblyPanel, defaultId, defaultParams, PART_FIELDS, PART_TYPES, type PanelResult } from "../src/assembly-panel.ts";
import { GALLERY_PARTS } from "../src/craft-page.ts";
import { loadCraft } from "../src/parts/craft-file.ts";

const HOPPER_TEXT = readFileSync(new URL("./fixtures/hopper.craft.json", import.meta.url), "utf8");

function ok(r: PanelResult): void {
  assert.ok(r.ok, r.ok ? "" : r.reason);
}

function refusedWith(r: PanelResult, reason: RegExp): void {
  assert.ok(!r.ok, "the action was applied, but it should have been refused");
  assert.match(r.reason, reason);
}

/** The hopper the way a person builds it on the panel: tank, pod on top, engine below, a leg on each of four side patches. */
function buildHopper(panel = new AssemblyPanel()): AssemblyPanel {
  ok(panel.addPart("tank"));
  ok(panel.addPart("pod", undefined, { parent: "tank", parentPort: "top" }));
  ok(panel.addPart("engine", undefined, { parent: "tank", parentPort: "bottom" }));
  for (let i = 0; i < 4; i++) ok(panel.addPart("leg", undefined, { parent: "tank", parentPort: `side-0-${i * 2}` }, `leg${i}`));
  return panel;
}

test("the panel's actions give the hopper's craft file, byte for byte", () => {
  // The fixture is itself shown (craft-file.test.ts) to compile to the same commands as the hopper built in code.
  const panel = buildHopper();
  assert.equal(panel.text(), HOPPER_TEXT);
  assert.equal(panel.parts.length, 7);
  assert.equal(panel.joins.length, 6);
  assert.deepEqual(panel.place, { x: 0, y: 2.339576, z: 0, yaw: 0 });
});

test("a craft file saved by the panel loads and is the same craft", () => {
  const panel = buildHopper();
  const loaded = loadCraft(panel.text()!);
  assert.equal(loaded.craft.nodes.length, panel.craft!.nodes.length);
  assert.equal(loaded.craft.beams.length, panel.craft!.beams.length);
  const again = new AssemblyPanel();
  ok(again.load(panel.text()!));
  assert.equal(again.text(), panel.text());
});

test("loading the committed hopper and saving it gives the same text; a file that does not load changes nothing", () => {
  const panel = new AssemblyPanel();
  ok(panel.load(HOPPER_TEXT));
  assert.equal(panel.text(), HOPPER_TEXT);
  const before = panel.text();
  refusedWith(panel.load("{ not json"), /not valid JSON/);
  refusedWith(panel.load(HOPPER_TEXT.replace('"clock": 0', '"clock": 0.5')), /clock/);
  assert.equal(panel.text(), before);
});

test("default ids: the type's name, then type1, type2", () => {
  assert.equal(defaultId("tank", []), "tank");
  assert.equal(defaultId("tank", ["tank"]), "tank1");
  assert.equal(defaultId("tank", ["tank", "tank1", "leg"]), "tank2");
  const panel = new AssemblyPanel();
  ok(panel.addPart("tank"));
  ok(panel.addPart("tank", undefined, { parent: "tank", parentPort: "top" }));
  assert.deepEqual(panel.parts.map((p) => p.id), ["tank", "tank1"]);
  refusedWith(panel.addPart("pod", undefined, { parent: "tank1", parentPort: "top" }, "tank"), /already a part called "tank"/);
});

test("every part type has a form: the fields the file reads, and defaults that build", () => {
  assert.deepEqual([...PART_TYPES].sort(), GALLERY_PARTS.map((p) => p.type).sort());
  for (const type of PART_TYPES) {
    const params = defaultParams(type);
    const keys = PART_FIELDS[type].map((f) => f.key);
    for (const k of Object.keys(params)) assert.ok(keys.includes(k), `${type}: default has a field "${k}" the file does not read`);
    for (const f of PART_FIELDS[type]) if (!f.optional) assert.ok(k(params, f.key), `${type}: no default for ${f.key}`);
    ok(new AssemblyPanel().addPart(type));
  }
  function k(p: Record<string, unknown>, key: string): boolean {
    return p[key] !== undefined;
  }
});

test("free ports follow the craft: a tank has two rings and its side patches; joining uses one up", () => {
  const panel = new AssemblyPanel();
  assert.deepEqual(panel.freePorts(), []);
  ok(panel.addPart("tank"));
  const ports = panel.freePorts();
  assert.deepEqual(ports.filter((p) => p.kind === "ring").map((p) => p.name), ["tank.bottom", "tank.top"]);
  assert.ok(ports.some((p) => p.name === "tank.side-0-0" && p.kind === "patch" && p.nodes === 4));
  const top = ports.find((p) => p.name === "tank.top")!;
  const bottom = ports.find((p) => p.name === "tank.bottom")!;
  assert.ok(top.centre[1] > bottom.centre[1], "the top ring is above the bottom ring");
  ok(panel.addPart("pod", undefined, { parent: "tank", parentPort: "top" }));
  assert.ok(!panel.freePorts().some((p) => p.name === "tank.top"));
  assert.ok(panel.freePorts().some((p) => p.name === "pod.bottom") === false, "the pod's own ring is used by the join");
  refusedWith(panel.addPart("engine", undefined, { parent: "tank", parentPort: "top" }), /not a free ring or patch/);
});

test("a part that does not fit is refused with the reason, and the craft is as it was", () => {
  const panel = new AssemblyPanel();
  ok(panel.addPart("tank"));
  const before = panel.text();
  const wide = { ...defaultParams("engine"), topRadius: 0.8 };
  refusedWith(panel.addPart("engine", wide, { parent: "tank", parentPort: "bottom" }), /differ in radius/);
  refusedWith(panel.addPart("engine", { ...defaultParams("engine"), ringNodes: 6 }, { parent: "tank", parentPort: "bottom" }), /node count/);
  refusedWith(panel.addPart("leg", undefined, { parent: "tank", parentPort: "top" }), /no ring to put on tank\.top/);
  refusedWith(panel.addPart("pod", undefined, { parent: "tank", parentPort: "side-0-0" }), /no patch to put on tank\.side-0-0/);
  refusedWith(panel.addPart("pod", undefined, { parent: "tank", parentPort: "top", childPort: "nowhere" }), /nowhere/);
  refusedWith(panel.addPart("pod", undefined, { parent: "wing", parentPort: "top" }), /not a free ring or patch/);
  refusedWith(panel.addPart("pod"), /pick a free ring or patch/);
  refusedWith(panel.addPart("pod", { ...defaultParams("pod"), height: -1 }, { parent: "tank", parentPort: "top" }), /pod/);
  assert.equal(panel.text(), before);
  assert.equal(panel.parts.length, 1);
});

test("a patch takes a leg of the right size, and a leg's mount is fitted to the patch it is put on", () => {
  const panel = new AssemblyPanel();
  ok(panel.addPart("tank"));
  const wrong = { ...defaultParams("leg"), mountWidth: 0.2, mountHeight: 0.2 };
  ok(panel.addPart("leg", wrong, { parent: "tank", parentPort: "side-0-0" }));
  const params = panel.parts[1]!.params as unknown as Record<string, number>;
  assert.equal(params.mountWidth, 0.7653668647301797);
  assert.equal(params.mountHeight, 0.625);
  // two legs may not share a patch, and a neighbouring patch that shares a node is refused with its reason
  refusedWith(panel.addPart("leg", undefined, { parent: "tank", parentPort: "side-0-0" }), /not a free ring or patch/);
  refusedWith(panel.addPart("leg", undefined, { parent: "tank", parentPort: "side-0-1" }), /share|node|used|patch/);
});

test("turn: a ring part's clock goes round and wraps, a root turns by yaw, a patch part does not turn", () => {
  const panel = new AssemblyPanel();
  ok(panel.addPart("tank"));
  ok(panel.turn(1));
  assert.equal(panel.place.yaw, 1);
  ok(panel.turn(-1));
  assert.equal(panel.place.yaw, 0);
  ok(panel.addPart("pod", undefined, { parent: "tank", parentPort: "top" }));
  const flat = panel.text();
  ok(panel.turn(1));
  assert.equal(panel.joins[0]!.clock, 1);
  assert.notEqual(panel.text(), flat);
  ok(panel.turn(-2));
  assert.equal(panel.joins[0]!.clock, 7, "one step back from zero is the last of the ring's 8");
  ok(panel.turn(1));
  assert.equal(panel.text(), flat);
  ok(panel.turn(8));
  assert.equal(panel.text(), flat, "a whole turn is no turn");
  refusedWith(panel.turn(0.5), /whole number/);
  ok(panel.addPart("leg", undefined, { parent: "tank", parentPort: "side-0-0" }));
  refusedWith(panel.turn(1), /does not turn/);
  assert.equal(panel.joins.at(-1)!.clock, 0);
});

test("remove last takes the newest part and its join, down to nothing", () => {
  const panel = buildHopper();
  ok(panel.removeLast());
  assert.deepEqual(panel.parts.map((p) => p.id), ["tank", "pod", "engine", "leg0", "leg1", "leg2"]);
  assert.equal(panel.joins.length, 5);
  assert.ok(panel.freePorts().some((p) => p.name === "tank.side-0-6"), "the patch is free again");
  while (panel.parts.length > 0) ok(panel.removeLast());
  assert.equal(panel.text(), null);
  assert.equal(panel.craft, null);
  assert.deepEqual(panel.freePorts(), []);
  refusedWith(panel.removeLast(), /nothing to remove/);
  // and it can be built again from nothing
  buildHopper(panel);
  assert.equal(panel.text(), HOPPER_TEXT);
});

test("the root's place: a typed height is kept, and the height that stands on the pad is found again", () => {
  const panel = new AssemblyPanel();
  ok(panel.addPart("tank"));
  ok(panel.setPlace({ x: 3, y: 7, z: -2, yaw: 2 }));
  assert.deepEqual(panel.place, { x: 3, y: 7, z: -2, yaw: 2 });
  ok(panel.addPart("pod", undefined, { parent: "tank", parentPort: "top" }));
  assert.equal(panel.place.y, 7, "a typed height stays when parts are added");
  refusedWith(panel.setPlace({ x: 0, y: Number.NaN, z: 0, yaw: 0 }), /finite/);
  assert.equal(panel.place.y, 7);
  const standing = new AssemblyPanel();
  buildHopper(standing);
  standing.setPlace({ x: 0, y: 5, z: 0, yaw: 0 }, true);
  assert.equal(standing.place.y, 2.339576);
});
