import assert from "node:assert/strict";
import test from "node:test";
import { newPlacement, placeBeam, rotateBy, terrainTriangles } from "../src/render3d.ts";
import { STRESS_BLUE, STRESS_GREY, STRESS_RED, stressCss, stressRgb } from "../src/stress-colour.ts";
import { heightAt, plainTerrain, valleyTerrain, type Terrain } from "../src/terrain.ts";

const close = (a: number, b: number, eps = 1e-9): boolean => Math.abs(a - b) <= eps;

test("a beam's cylinder sits at the middle, is as long as the beam, and its +Y axis points along it", () => {
  const dirs: [number, number, number][] = [
    [0, 1, 0],
    [0, -1, 0],
    [1, 0, 0],
    [0, 0, -1],
    [1, 2, 3],
    [-3, 0.5, 2],
    [1e-3, -5, 1e-3],
  ];
  for (const [dx, dy, dz] of dirs) {
    const p = [1, 2, 3, 1 + dx, 2 + dy, 3 + dz];
    const out = placeBeam(p, 0, 1, newPlacement());
    const len = Math.sqrt(dx * dx + dy * dy + dz * dz);
    assert.ok(close(out.length, len), `length for ${dx}, ${dy}, ${dz}`);
    assert.ok(close(out.x, 1 + dx / 2) && close(out.y, 2 + dy / 2) && close(out.z, 3 + dz / 2));
    assert.ok(close(out.qx ** 2 + out.qy ** 2 + out.qz ** 2 + out.qw ** 2, 1), "unit quaternion");
    const [vx, vy, vz] = rotateBy(out, 0, 1, 0);
    assert.ok(close(vx, dx / len, 1e-6) && close(vy, dy / len, 1e-6) && close(vz, dz / len, 1e-6), `axis for ${dx}, ${dy}, ${dz}`);
  }
});

test("a zero-length beam gets the identity turn and length 0", () => {
  const out = placeBeam([4, 5, 6, 4, 5, 6], 0, 1, newPlacement());
  assert.equal(out.length, 0);
  assert.deepEqual([out.qx, out.qy, out.qz, out.qw], [0, 0, 0, 1]);
});

test("the stress scale: grey at zero, red in tension, blue in compression, saturated past one", () => {
  assert.deepEqual(stressRgb(0), [...STRESS_GREY]);
  assert.deepEqual(stressRgb(1), [...STRESS_RED]);
  assert.deepEqual(stressRgb(7), [...STRESS_RED]);
  assert.deepEqual(stressRgb(-1), [...STRESS_BLUE]);
  assert.deepEqual(stressRgb(-3), [...STRESS_BLUE]);
  // Halfway is the midpoint of grey and the end colour, rounded.
  assert.deepEqual(stressRgb(0.5), [Math.round((138 + 255) / 2), Math.round((147 + 93) / 2), Math.round((163 + 93) / 2)]);
  assert.equal(stressCss(0), "rgb(138,147,163)");
});

/** Height of the drawn triangle surface at `(x, z)`, found by locating the triangle in the mesh. */
function drawnHeight(t: Terrain, m: ReturnType<typeof terrainTriangles>, x: number, z: number): number {
  const i = Math.min(t.nx - 2, Math.floor((x - t.x0) / t.cell));
  const j = Math.min(t.nz - 2, Math.floor((z - t.z0) / t.cell));
  const cell = i + j * (t.nx - 1);
  for (const tri of [cell * 2, cell * 2 + 1]) {
    const v = [0, 1, 2].map((k) => m.indices[tri * 3 + k]! * 3);
    const [a, b, c] = v as [number, number, number];
    const px = (k: number): number => m.positions[k]!;
    // Barycentric weights in the x-z plane.
    const d = (px(b + 2) - px(c + 2)) * (px(a) - px(c)) + (px(c) - px(b)) * (px(a + 2) - px(c + 2));
    const w1 = ((px(b + 2) - px(c + 2)) * (x - px(c)) + (px(c) - px(b)) * (z - px(c + 2))) / d;
    const w2 = ((px(c + 2) - px(a + 2)) * (x - px(c)) + (px(a) - px(c)) * (z - px(c + 2))) / d;
    const w3 = 1 - w1 - w2;
    if (w1 >= -1e-9 && w2 >= -1e-9 && w3 >= -1e-9) return w1 * px(a + 1) + w2 * px(b + 1) + w3 * px(c + 1);
  }
  throw new Error(`no triangle at ${x}, ${z}`);
}

test("the drawn terrain triangles are the ones the core collides with, and face up", () => {
  for (const t of [plainTerrain(), valleyTerrain()]) {
    const m = terrainTriangles(t);
    assert.equal(m.indices.length, (t.nx - 1) * (t.nz - 1) * 6);
    // Same height as `heightAt` (the core's rule) at points inside the grid, off the grid lines.
    for (let n = 0; n < 400; n++) {
      const x = t.x0 + ((n * 0.6180339887) % 1) * (t.nx - 1) * t.cell;
      const z = t.z0 + ((n * 0.7548776662) % 1) * (t.nz - 1) * t.cell;
      assert.ok(close(drawnHeight(t, m, x, z), heightAt(t, x, z), 1e-5), `at ${x}, ${z}`);
    }
    // Every triangle's normal has a positive y.
    for (let k = 0; k < m.indices.length; k += 3) {
      const [a, b, c] = [m.indices[k]! * 3, m.indices[k + 1]! * 3, m.indices[k + 2]! * 3];
      const e1 = [0, 1, 2].map((q) => m.positions[b + q]! - m.positions[a + q]!);
      const e2 = [0, 1, 2].map((q) => m.positions[c + q]! - m.positions[a + q]!);
      assert.ok(e1[2]! * e2[0]! - e1[0]! * e2[2]! > 0, `triangle ${k / 3} faces down`);
    }
  }
});
