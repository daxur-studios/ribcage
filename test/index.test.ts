import assert from "node:assert/strict";
import test from "node:test";
import { hello } from "../src/index.ts";

test("hello returns greeting", () => {
  assert.equal(hello(), "Hello from node-beam-physics");
});
