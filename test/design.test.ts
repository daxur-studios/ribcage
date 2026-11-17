import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";
import { CommandBatch } from "../src/commands.ts";
import { Core } from "../src/core.ts";
import { bridgeDesign, STRONG_BRIDGE } from "../src/bridge.ts";
import {
  compileDesign,
  deckOf,
  locateTruck,
  rollLoad,
  STANDARD_TRUCK,
  type Design,
} from "../src/design.ts";
import { World } from "../src/world.ts";

const WASM_PATH = new URL("../target/wasm32-unknown-unknown/release/beam_core.wasm", import.meta.url);

test("the deck is the nodes level with the leftmost pin, left to right", () => {
  const d = bridgeDesign(STRONG_BRIDGE);
  const deck = deckOf(d);
  assert.deepEqual(deck.ids, [0, 1, 2, 3, 4, 5, 6, 7, 8]);
  assert.deepEqual(deck.xs, [0, 3, 6, 9, 12, 15, 18, 21, 24]);

  // Nothing pinned: no deck, so no truck.
  const free: Design = { nodes: [{ x: 0, y: 0, pinned: false }, { x: 1, y: 0, pinned: false }], beams: [] };
  assert.deepEqual(deckOf(free), { ids: [], xs: [] });
  assert.equal(rollLoad(new CommandBatch(), deckOf(free), STANDARD_TRUCK, 0), false);
});

test("the truck's weight is split between the two deck nodes beside it, and leaves the deck", () => {
  const deck = deckOf(bridgeDesign(STRONG_BRIDGE));
  // Step 0: at the left support, all of it on node 0.
  assert.deepEqual(locateTruck(deck, STANDARD_TRUCK, 0), { bay: 0, t: 0 });
  // 1.5 m along (4 m/s, 22.5 steps): halfway through the first bay.
  const mid = locateTruck(deck, STANDARD_TRUCK, 22.5)!;
  assert.equal(mid.bay, 0);
  assert.ok(Math.abs(mid.t - 0.5) < 1e-12);
  // Past the far support: not on the deck.
  assert.equal(locateTruck(deck, STANDARD_TRUCK, 60 * 7), null);

  const b = new CommandBatch();
  assert.equal(rollLoad(b, deck, STANDARD_TRUCK, 0), true);
  assert.equal(b.count, 2);
});

test("a design compiles to the indices the editor assumes", async () => {
  const core = await Core.load(new Uint8Array(readFileSync(WASM_PATH)));
  const design: Design = {
    nodes: [
      { x: 0, y: 0, pinned: true },
      { x: 4, y: 0, pinned: false },
      { x: 2, y: 3, pinned: false },
    ],
    beams: [
      { a: 0, b: 1 },
      { a: 1, b: 2 },
      { a: 2, b: 0 },
    ],
  };
  const w = new World(core);
  const batch = new CommandBatch();
  compileDesign(batch, design, STRONG_BRIDGE);
  const r = w.apply(batch);
  assert.equal(r.rejected, 0);
  // Nodes come first, then beams, each numbered from 0 as in the design.
  assert.deepEqual([...r.results], [0, 1, 2, 0, 1, 2]);
  assert.equal(w.nodeCount, 3);
  assert.equal(w.beamCount, 3);
  // The pinned node did not move; the free ones are held up by the pinned one only through beams.
  const snap = w.snapshot();
  assert.deepEqual([...snap.nodeFlags].map((f) => (f & 2) !== 0), [true, false, false]);
  assert.deepEqual([...snap.beams.ends], [0, 1, 1, 2, 2, 0]);
  w.free();
});
