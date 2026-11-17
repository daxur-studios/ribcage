import assert from "node:assert/strict";
import test from "node:test";
import { CommandBatch } from "../src/commands.ts";
import { Core } from "../src/core.ts";
import { bridgePanels, towerPanels, type PanelSet } from "../src/panels3d.ts";
import { buildTower, STANDARD_BRIDGE3, STANDARD_TOWER, towerAction } from "../src/scenes3d.ts";
import { NODE_TEXTURE_WIDTH, nodeTextureRows, packNodeTexture, shadeVertexF32, skinAttributes, skinMesh } from "../src/skin.ts";
import { World } from "../src/world.ts";
import { readFileSync } from "node:fs";

const sets: [string, PanelSet][] = [
  ["tower", towerPanels(STANDARD_TOWER)],
  ["bridge", bridgePanels(STANDARD_BRIDGE3)],
];

type M3 = [number, number, number, number, number, number, number, number, number];

/** Applies `x ↦ A x + t` to every point (three numbers each). */
function affine(p: ArrayLike<number>, A: M3, t: [number, number, number]): Float64Array {
  const out = new Float64Array(p.length);
  for (let i = 0; i < p.length; i += 3) {
    const x = p[i]!;
    const y = p[i + 1]!;
    const z = p[i + 2]!;
    out[i] = A[0] * x + A[1] * y + A[2] * z + t[0];
    out[i + 1] = A[3] * x + A[4] * y + A[5] * z + t[1];
    out[i + 2] = A[6] * x + A[7] * y + A[8] * z + t[2];
  }
  return out;
}

/** Rotation by `angle` about the unit axis `(ax, ay, az)` (Rodrigues). */
function rotation(ax: number, ay: number, az: number, angle: number): M3 {
  const l = Math.sqrt(ax * ax + ay * ay + az * az);
  const [x, y, z] = [ax / l, ay / l, az / l];
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  const k = 1 - c;
  return [c + x * x * k, x * y * k - z * s, x * z * k + y * s, y * x * k + z * s, c + y * y * k, y * z * k - x * s, z * x * k - y * s, z * y * k + x * s, c + z * z * k];
}

/** The inverse transpose of a 3 × 3 matrix, up to a positive factor: its cofactor matrix. */
function cofactor(A: M3): M3 {
  const [a, b, c, d, e, f, g, h, i] = A;
  return [e * i - f * h, f * g - d * i, d * h - e * g, c * h - b * i, a * i - c * g, b * g - a * h, b * f - c * e, c * d - a * f, a * e - b * d];
}

function skin(s: PanelSet, nodes: ArrayLike<number>): { pos: Float64Array; nrm: Float64Array } {
  const pos = new Float64Array(s.binding.vertexCount * 3);
  const nrm = new Float64Array(s.binding.vertexCount * 3);
  skinMesh(s.binding, nodes, pos, nrm);
  return { pos, nrm };
}

/** The largest difference between two lists of numbers. */
function maxDiff(a: ArrayLike<number>, b: ArrayLike<number>): number {
  assert.equal(a.length, b.length);
  let m = 0;
  for (let i = 0; i < a.length; i++) m = Math.max(m, Math.abs(a[i]! - b[i]!));
  return m;
}

/** Normals moved by `A`: `cof(A) n`, normalised. */
function turnNormals(n: ArrayLike<number>, A: M3): Float64Array {
  const out = affine(n, cofactor(A), [0, 0, 0]);
  for (let i = 0; i < out.length; i += 3) {
    const l = Math.hypot(out[i]!, out[i + 1]!, out[i + 2]!);
    out[i]! /= l;
    out[i + 1]! /= l;
    out[i + 2]! /= l;
  }
  return out;
}

for (const [name, s] of sets) {
  test(`${name}: every vertex binds to four nodes of its own cell with weights summing to one`, () => {
    const b = s.binding;
    assert.ok(b.vertexCount > 0);
    for (let v = 0; v < b.vertexCount; v++) {
      const cellNodes = new Set(s.cells[s.cellOf[v]!]!.tets.flat());
      const four = [...b.nodes.subarray(v * 4, v * 4 + 4)];
      assert.equal(new Set(four).size, 4, `vertex ${v} has four different nodes`);
      for (const n of four) assert.ok(cellNodes.has(n), `vertex ${v}: node ${n} is in its cell`);
      const sum = b.weights[v * 4]! + b.weights[v * 4 + 1]! + b.weights[v * 4 + 2]! + b.weights[v * 4 + 3]!;
      assert.ok(Math.abs(sum - 1) < 1e-12, `vertex ${v} weights sum to ${sum}`);
    }
    // A panel's vertices all belong to one cell.
    for (let k = 0; k + 1 < s.panelStart.length; k++) {
      const cells = new Set(s.cellOf.subarray(s.panelStart[k]!, s.panelStart[k + 1]!));
      assert.equal(cells.size, 1, `panel ${k} is in one cell`);
    }
  });

  test(`${name}: the rest pose reproduces the mesh`, () => {
    const { pos, nrm } = skin(s, s.rest);
    // Exact up to rounding in the last bits of a float64: far below the 32-bit render precision.
    assert.ok(maxDiff(pos, s.positions) < 1e-12, `positions off by ${maxDiff(pos, s.positions)}`);
    assert.ok(maxDiff(nrm, s.normals) < 1e-12, `normals off by ${maxDiff(nrm, s.normals)}`);
  });

  test(`${name}: a rotated and moved lattice gives the rotated and moved mesh`, () => {
    for (const [ax, ay, az, angle, t] of [
      [0, 1, 0, 0.7, [3, -2, 5]],
      [1, 2, -0.5, 2.4, [-40, 10, 120]],
      [0.3, -1, 0.2, Math.PI, [0, 0, 0]],
    ] as [number, number, number, number, [number, number, number]][]) {
      const R = rotation(ax, ay, az, angle);
      const { pos, nrm } = skin(s, affine(s.rest, R, t));
      assert.ok(maxDiff(pos, affine(s.positions, R, t)) < 1e-6, `positions off by ${maxDiff(pos, affine(s.positions, R, t))}`);
      assert.ok(maxDiff(nrm, affine(s.normals, R, [0, 0, 0])) < 1e-9, "normals turn with the lattice");
    }
  });

  test(`${name}: a stretched lattice gives the stretched mesh`, () => {
    for (const A of [
      [1.25, 0, 0, 0, 1.25, 0, 0, 0, 1.25],
      [1.3, 0, 0, 0, 0.8, 0, 0, 0, 1.1],
      [1, 0.2, 0, 0, 1, 0, 0.1, 0, 0.9],
    ] as M3[]) {
      const t: [number, number, number] = [1, 2, 3];
      const { pos, nrm } = skin(s, affine(s.rest, A, t));
      assert.ok(maxDiff(pos, affine(s.positions, A, t)) < 1e-6, `positions off by ${maxDiff(pos, affine(s.positions, A, t))}`);
      assert.ok(maxDiff(nrm, turnNormals(s.normals, A)) < 1e-9, "normals follow the cofactor of the stretch");
    }
  });

  test(`${name}: the shader's maths in 32-bit floats agrees with the reference`, () => {
    const attrs = skinAttributes(s.binding);
    const nodeCount = s.rest.length / 3;
    // A non-affine lattice: every node pushed by its own amount, as in a collapse.
    const moved = Float64Array.from(s.rest, (x, i) => x + 0.3 * Math.sin(i * 1.7) + (i % 3 === 1 ? 2 : 0));
    for (const nodes of [s.rest, moved, affine(s.rest, rotation(1, 1, 0, 1.1), [50, 3, -20])]) {
      const tex = new Float32Array(NODE_TEXTURE_WIDTH * nodeTextureRows(nodeCount) * 4);
      packNodeTexture(nodes, tex);
      const { pos, nrm } = skin(s, nodes);
      const out = new Float64Array(6);
      let dp = 0;
      let dn = 0;
      for (let v = 0; v < s.binding.vertexCount; v++) {
        shadeVertexF32(tex, attrs, v, out);
        for (let k = 0; k < 3; k++) {
          dp = Math.max(dp, Math.abs(out[k]! - pos[v * 3 + k]!));
          dn = Math.max(dn, Math.abs(out[3 + k]! - nrm[v * 3 + k]!));
        }
      }
      // 32-bit floats near 60 m are 4e-6 m apart; a handful of roundings stays far below a millimetre.
      assert.ok(dp < 1e-4, `positions off by ${dp} m`);
      assert.ok(dn < 1e-4, `normals off by ${dn}`);
    }
  });
}

test("the node texture holds node i at column i mod W, row i / W", () => {
  const n = NODE_TEXTURE_WIDTH + 3;
  const p = Float64Array.from({ length: n * 3 }, (_, i) => i);
  const tex = new Float32Array(NODE_TEXTURE_WIDTH * nodeTextureRows(n) * 4);
  assert.equal(nodeTextureRows(n), 2);
  packNodeTexture(p, tex);
  const i = NODE_TEXTURE_WIDTH + 1;
  assert.deepEqual([...tex.subarray(i * 4, i * 4 + 4)], [i * 3, i * 3 + 1, i * 3 + 2, 1]);
});

test("in the knocked tower's collapse, every panel stays on its cell", async () => {
  const core = await Core.load(new Uint8Array(readFileSync(new URL("../target/wasm32-unknown-unknown/release/beam_core.wasm", import.meta.url))));
  const world = new World(core);
  const setup = new CommandBatch();
  const layout = buildTower(setup);
  world.apply(setup);
  const s = towerPanels(STANDARD_TOWER);
  const restDist = anchorDistances(s, s.rest);
  let worst = 0;
  for (let step = 0; step < 600; step++) {
    const batch = new CommandBatch();
    towerAction(batch, layout, "knock", step);
    world.apply(batch);
    world.step();
    if (step % 30 !== 29) continue;
    const nodes = world.positions();
    const { pos } = skin(s, nodes);
    assert.ok(pos.every(Number.isFinite));
    // A vertex never ends up further from its anchor node than at rest plus a few centimetres:
    // the panel goes where its cell goes.
    const d = anchorDistances(s, nodes, pos);
    for (let v = 0; v < d.length; v++) worst = Math.max(worst, d[v]! - restDist[v]!);
  }
  world.free();
  assert.ok(worst < 0.05, `a vertex drifted ${worst} m from its anchor node`);
});

/** Each vertex's distance to its tetrahedron's anchor node. */
function anchorDistances(s: PanelSet, nodes: ArrayLike<number>, pos: ArrayLike<number> = s.positions): Float64Array {
  const out = new Float64Array(s.binding.vertexCount);
  for (let v = 0; v < out.length; v++) {
    const a = s.binding.nodes[v * 4]! * 3;
    out[v] = Math.hypot(pos[v * 3]! - nodes[a]!, pos[v * 3 + 1]! - nodes[a + 1]!, pos[v * 3 + 2]! - nodes[a + 2]!);
  }
  return out;
}
