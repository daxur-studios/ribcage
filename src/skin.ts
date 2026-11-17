/**
 * Purpose: binds mesh vertices to the nodes of a lattice cell and places them again from where
 * those nodes have moved: four nodes per vertex (a tetrahedron of the cell), barycentric weights
 * for the position, and the cofactor of the tetrahedron's deformation for the normal.
 * Value: the page's TSL vertex function does exactly this per vertex on the GPU; here it is plain
 * numbers, so Node can test it (rest pose, rigid motion, stretch) and check the shader's maths
 * (`shadeVertexF32`) against it.
 *
 * Why it is exact for any rigid motion and any stretch: a point is `Σ wᵢ pᵢ` with `Σ wᵢ = 1`, and
 * any affine map `x ↦ A x + t` of the four nodes moves that point to `A (Σ wᵢ pᵢ) + t`. Normals
 * follow `cof(A) = det(A) A⁻ᵀ`, which is what a surface's normal does under `A`; the tetrahedron's
 * edges give `A` on the GPU without a matrix inverse (see `bindVertex`).
 */

/** Four node indices that are not in one plane. The first is the tetrahedron's anchor. */
export type Tet = readonly [number, number, number, number];

/**
 * A lattice cell, for skinning: the tetrahedra its vertices may bind to. A vertex binds to the
 * tetrahedron whose anchor node is nearest to it at rest (ties go to the first), so a panel's
 * corner follows the cell corner it sits at.
 */
export interface SkinCell {
  tets: readonly Tet[];
}

/** Per vertex: four nodes, four weights summing to one, and the normal in the tetrahedron's rest frame. */
export interface SkinBinding {
  vertexCount: number;
  /** Four node indices per vertex. */
  nodes: Uint32Array;
  /** Four barycentric weights per vertex. */
  weights: Float64Array;
  /**
   * Three numbers per vertex: the rest normal `n` as `m = Dᵀ n / det D`, where `D` holds the
   * tetrahedron's rest edges `eᵢ = pᵢ − p₀` as columns. The skinned normal is then
   * `m₁ (e₂×e₃) + m₂ (e₃×e₁) + m₃ (e₁×e₂)` with the current edges, which is `cof(F) n` for the
   * cell's deformation `F` (up to a positive factor, so it only needs normalising).
   */
  restNormal: Float64Array;
}

/** A tetrahedron is refused when its volume is below this share of its three edge lengths' product. */
const FLAT_TET = 1e-6;

/** The tetrahedron of `cell` whose anchor is nearest to `(x, y, z)` at rest; ties go to the first. */
export function chooseTet(rest: ArrayLike<number>, cell: SkinCell, x: number, y: number, z: number): Tet {
  let best = cell.tets[0]!;
  let bestD = Infinity;
  for (const t of cell.tets) {
    const a = t[0] * 3;
    const dx = rest[a]! - x;
    const dy = rest[a + 1]! - y;
    const dz = rest[a + 2]! - z;
    const d = dx * dx + dy * dy + dz * dz;
    if (d < bestD) {
      bestD = d;
      best = t;
    }
  }
  return best;
}

/**
 * Binds one vertex (position `x, y, z`, unit normal `nx, ny, nz`) to the tetrahedron `tet` of the
 * rest node positions `rest`, writing slot `v` of `out`. Throws for a flat tetrahedron.
 *
 * With rest edges `eᵢ` and `d = x − p₀`: the weights are `wᵢ = (cᵢ · d) / det D` for the
 * cofactor columns `c₁ = e₂×e₃`, `c₂ = e₃×e₁`, `c₃ = e₁×e₂`, and `w₀ = 1 − w₁ − w₂ − w₃`;
 * the normal's rest frame numbers are `mᵢ = (eᵢ · n) / det D`.
 */
export function bindVertex(rest: ArrayLike<number>, tet: Tet, x: number, y: number, z: number, nx: number, ny: number, nz: number, out: SkinBinding, v: number): void {
  const [i0, i1, i2, i3] = tet;
  const p0x = rest[i0 * 3]!;
  const p0y = rest[i0 * 3 + 1]!;
  const p0z = rest[i0 * 3 + 2]!;
  const e = [i1, i2, i3].map((i) => [rest[i * 3]! - p0x, rest[i * 3 + 1]! - p0y, rest[i * 3 + 2]! - p0z] as const);
  const [e1, e2, e3] = e as [V3, V3, V3];
  const c1 = cross(e2, e3);
  const c2 = cross(e3, e1);
  const c3 = cross(e1, e2);
  const det = dot(e1, c1);
  const scale = len(e1) * len(e2) * len(e3);
  if (!(Math.abs(det) > FLAT_TET * scale)) throw new Error(`tetrahedron ${tet.join(', ')} is flat`);
  const d: V3 = [x - p0x, y - p0y, z - p0z];
  const w1 = dot(c1, d) / det;
  const w2 = dot(c2, d) / det;
  const w3 = dot(c3, d) / det;
  const n: V3 = [nx, ny, nz];
  out.nodes.set(tet, v * 4);
  out.weights[v * 4] = 1 - w1 - w2 - w3;
  out.weights[v * 4 + 1] = w1;
  out.weights[v * 4 + 2] = w2;
  out.weights[v * 4 + 3] = w3;
  out.restNormal[v * 3] = dot(e1, n) / det;
  out.restNormal[v * 3 + 1] = dot(e2, n) / det;
  out.restNormal[v * 3 + 2] = dot(e3, n) / det;
}

/** An empty binding for `vertexCount` vertices. */
export function newBinding(vertexCount: number): SkinBinding {
  return {
    vertexCount,
    nodes: new Uint32Array(vertexCount * 4),
    weights: new Float64Array(vertexCount * 4),
    restNormal: new Float64Array(vertexCount * 3),
  };
}

/**
 * Binds a whole mesh: vertex `v` (at `positions[3v..]`, normal `normals[3v..]`) belongs to cell
 * `cellOf[v]` and binds to that cell's nearest-anchored tetrahedron.
 */
export function bindMesh(rest: ArrayLike<number>, cells: readonly SkinCell[], positions: ArrayLike<number>, normals: ArrayLike<number>, cellOf: ArrayLike<number>): SkinBinding {
  const count = cellOf.length;
  const b = newBinding(count);
  for (let v = 0; v < count; v++) {
    const x = positions[v * 3]!;
    const y = positions[v * 3 + 1]!;
    const z = positions[v * 3 + 2]!;
    const tet = chooseTet(rest, cells[cellOf[v]!]!, x, y, z);
    bindVertex(rest, tet, x, y, z, normals[v * 3]!, normals[v * 3 + 1]!, normals[v * 3 + 2]!, b, v);
  }
  return b;
}

/**
 * The reference: places every vertex from node positions `p` (three per node) in full precision,
 * writing positions and unit normals (three per vertex). The shader repeats this per vertex.
 */
export function skinMesh(b: SkinBinding, p: ArrayLike<number>, outPos: Float64Array, outNormal: Float64Array): void {
  for (let v = 0; v < b.vertexCount; v++) {
    const i0 = b.nodes[v * 4]! * 3;
    const i1 = b.nodes[v * 4 + 1]! * 3;
    const i2 = b.nodes[v * 4 + 2]! * 3;
    const i3 = b.nodes[v * 4 + 3]! * 3;
    const w0 = b.weights[v * 4]!;
    const w1 = b.weights[v * 4 + 1]!;
    const w2 = b.weights[v * 4 + 2]!;
    const w3 = b.weights[v * 4 + 3]!;
    for (let k = 0; k < 3; k++) outPos[v * 3 + k] = w0 * p[i0 + k]! + w1 * p[i1 + k]! + w2 * p[i2 + k]! + w3 * p[i3 + k]!;
    const e1: V3 = [p[i1]! - p[i0]!, p[i1 + 1]! - p[i0 + 1]!, p[i1 + 2]! - p[i0 + 2]!];
    const e2: V3 = [p[i2]! - p[i0]!, p[i2 + 1]! - p[i0 + 1]!, p[i2 + 2]! - p[i0 + 2]!];
    const e3: V3 = [p[i3]! - p[i0]!, p[i3 + 1]! - p[i0 + 1]!, p[i3 + 2]! - p[i0 + 2]!];
    const m1 = b.restNormal[v * 3]!;
    const m2 = b.restNormal[v * 3 + 1]!;
    const m3 = b.restNormal[v * 3 + 2]!;
    const c1 = cross(e2, e3);
    const c2 = cross(e3, e1);
    const c3 = cross(e1, e2);
    const nx = m1 * c1[0] + m2 * c2[0] + m3 * c3[0];
    const ny = m1 * c1[1] + m2 * c2[1] + m3 * c3[1];
    const nz = m1 * c1[2] + m2 * c2[2] + m3 * c3[2];
    const l = Math.sqrt(nx * nx + ny * ny + nz * nz);
    const k = l > 0 ? 1 / l : 0;
    outNormal[v * 3] = nx * k;
    outNormal[v * 3 + 1] = ny * k;
    outNormal[v * 3 + 2] = nz * k;
  }
}

// ---- what the GPU sees --------------------------------------------------------------------

/** Texels per row of the node texture: node `i` is at `(i mod W, ⌊i / W⌋)`. */
export const NODE_TEXTURE_WIDTH = 256;

/** Rows the node texture needs for `nodeCount` nodes (at least one). */
export function nodeTextureRows(nodeCount: number): number {
  return Math.max(1, Math.ceil(nodeCount / NODE_TEXTURE_WIDTH));
}

/**
 * Copies node positions (three per node) into an RGBA float texture's data, one texel per node:
 * `x, y, z` and `1` in alpha. Called once per drawn frame with the drawn positions.
 */
export function packNodeTexture(positions: ArrayLike<number>, out: Float32Array): void {
  const n = Math.min(positions.length / 3, out.length / 4);
  for (let i = 0; i < n; i++) {
    out[i * 4] = positions[i * 3]!;
    out[i * 4 + 1] = positions[i * 3 + 1]!;
    out[i * 4 + 2] = positions[i * 3 + 2]!;
    out[i * 4 + 3] = 1;
  }
}

/** The vertex attributes the shader reads, as 32-bit floats (node indices are exact below 2²⁴). */
export interface SkinAttributes {
  /** `cellNodes`: four node indices per vertex. */
  nodes: Float32Array;
  /** `cellWeights`: four weights per vertex. */
  weights: Float32Array;
  /** `restNormal`: three numbers per vertex. */
  restNormal: Float32Array;
}

export function skinAttributes(b: SkinBinding): SkinAttributes {
  return { nodes: Float32Array.from(b.nodes), weights: Float32Array.from(b.weights), restNormal: Float32Array.from(b.restNormal) };
}

const f = Math.fround;

/**
 * A CPU copy of the TSL vertex function, step for step in 32-bit floats (each operation rounded as
 * the GPU rounds it): reads the four nodes from the texture data, places the vertex
 * `Σ wᵢ pᵢ` and turns the normal by the cofactor of the current edges. Writes position and the
 * unnormalised normal for vertex `v` into `out` (six numbers); the shader normalises after
 * interpolation, so this does too, last.
 */
export function shadeVertexF32(tex: Float32Array, a: SkinAttributes, v: number, out: Float64Array): void {
  const node = (k: number): V3 => {
    const i = a.nodes[v * 4 + k]! | 0;
    const col = i % NODE_TEXTURE_WIDTH;
    const row = (i / NODE_TEXTURE_WIDTH) | 0;
    const t = (row * NODE_TEXTURE_WIDTH + col) * 4;
    return [tex[t]!, tex[t + 1]!, tex[t + 2]!];
  };
  const p0 = node(0);
  const p1 = node(1);
  const p2 = node(2);
  const p3 = node(3);
  const w = [a.weights[v * 4]!, a.weights[v * 4 + 1]!, a.weights[v * 4 + 2]!, a.weights[v * 4 + 3]!];
  for (let k = 0; k < 3; k++) {
    // p0 * w.x + p1 * w.y + p2 * w.z + p3 * w.w, left to right.
    out[k] = f(f(f(f(p0[k]! * w[0]!) + f(p1[k]! * w[1]!)) + f(p2[k]! * w[2]!)) + f(p3[k]! * w[3]!));
  }
  const sub = (p: V3, q: V3): V3 => [f(p[0] - q[0]), f(p[1] - q[1]), f(p[2] - q[2])];
  const crs = (p: V3, q: V3): V3 => [f(f(p[1] * q[2]) - f(p[2] * q[1])), f(f(p[2] * q[0]) - f(p[0] * q[2])), f(f(p[0] * q[1]) - f(p[1] * q[0]))];
  const e1 = sub(p1, p0);
  const e2 = sub(p2, p0);
  const e3 = sub(p3, p0);
  const c1 = crs(e2, e3);
  const c2 = crs(e3, e1);
  const c3 = crs(e1, e2);
  const m = [a.restNormal[v * 3]!, a.restNormal[v * 3 + 1]!, a.restNormal[v * 3 + 2]!];
  let nx = 0;
  let ny = 0;
  let nz = 0;
  for (let k = 0; k < 3; k++) {
    // c1 * m.x + c2 * m.y + c3 * m.z, left to right.
    const n = f(f(f(c1[k]! * m[0]!) + f(c2[k]! * m[1]!)) + f(c3[k]! * m[2]!));
    if (k === 0) nx = n;
    else if (k === 1) ny = n;
    else nz = n;
  }
  const l = Math.sqrt(nx * nx + ny * ny + nz * nz);
  out[3] = l > 0 ? nx / l : 0;
  out[4] = l > 0 ? ny / l : 0;
  out[5] = l > 0 ? nz / l : 0;
}

// ---- small vector helpers -----------------------------------------------------------------

type V3 = readonly [number, number, number];

function cross(a: V3, b: V3): V3 {
  return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
}

function dot(a: V3, b: V3): number {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

function len(a: V3): number {
  return Math.sqrt(dot(a, a));
}
