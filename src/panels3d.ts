/**
 * Purpose: the panels drawn over the milestone 3 scenes: floor slabs and wall panels on the tower,
 * deck slabs on the bridge. Each panel is a thin box that belongs to one lattice cell and is bound
 * to that cell's nodes (`skin.ts`), so it moves, tilts and bends with the cell.
 * Value: panels part at cell edges when a structure comes apart, which reads right without seam
 * tearing (milestone 9); the page only uploads these arrays and the node texture.
 *
 * Sizes are drawing **[assumption]**s: they only have to clear the drawn beams (radius 0.07 m)
 * and nodes (0.15 m) and read as floors, walls and a road.
 */

import { bridgeStruct, towerStruct, type BridgeSpec3, type Point3, type TowerSpec } from './scenes3d.ts';
import { bindMesh, type SkinBinding, type SkinCell, type Tet } from './skin.ts';

/** A panel mesh in its rest pose, with each vertex's cell, and its binding to the nodes. */
export interface PanelSet {
  /** Three per vertex, rest pose. */
  positions: Float64Array;
  /** Three per vertex, unit, rest pose (flat per box face). */
  normals: Float64Array;
  /** Three per vertex, 0 to 1. */
  colours: Float32Array;
  indices: Uint32Array;
  /** The cell each vertex belongs to. */
  cellOf: Uint32Array;
  cells: SkinCell[];
  /** Node rest positions the binding was made against (three per node). */
  rest: Float64Array;
  binding: SkinBinding;
  /** First vertex of each panel, and one past the last (`panelStart[k]` to `panelStart[k + 1]`). */
  panelStart: number[];
}

export const FLOOR_RGB: readonly [number, number, number] = [0.64, 0.63, 0.6];
export const WALL_RGB: readonly [number, number, number] = [0.78, 0.6, 0.44];
export const DECK_RGB: readonly [number, number, number] = [0.42, 0.43, 0.46];

/** The deck slab's top, above the deck nodes' centres, in metres (the truck stands on it). */
export const DECK_SLAB_TOP = 0.2;

type V = [number, number, number];

const sub = (a: V, b: V): V => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const add = (a: V, b: V): V => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const mul = (a: V, k: number): V => [a[0] * k, a[1] * k, a[2] * k];
const length = (a: V): number => Math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
const unit = (a: V): V => mul(a, 1 / length(a));
const cross = (a: V, b: V): V => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const dot = (a: V, b: V): number => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const pt = (p: Point3): V => [p.x, p.y, p.z];

/** Collects boxes as flat-shaded triangles: four vertices and one normal per face. */
class PanelBuilder {
  positions: number[] = [];
  normals: number[] = [];
  colours: number[] = [];
  indices: number[] = [];
  cellOf: number[] = [];
  panelStart: number[] = [];

  /** A box from corner `o` along the edges `u`, `v` and `w` (any handedness; faces are wound outward). */
  box(cell: number, o: V, u: V, v: V, w: V, rgb: readonly [number, number, number]): void {
    this.panelStart.push(this.cellOf.length);
    const c = (i: number, j: number, k: number): V => add(add(add(o, mul(u, i)), mul(v, j)), mul(w, k));
    const centre = add(o, mul(add(add(u, v), w), 0.5));
    const faces: V[][] = [
      [c(0, 0, 0), c(1, 0, 0), c(1, 1, 0), c(0, 1, 0)],
      [c(0, 0, 1), c(1, 0, 1), c(1, 1, 1), c(0, 1, 1)],
      [c(0, 0, 0), c(1, 0, 0), c(1, 0, 1), c(0, 0, 1)],
      [c(0, 1, 0), c(1, 1, 0), c(1, 1, 1), c(0, 1, 1)],
      [c(0, 0, 0), c(0, 1, 0), c(0, 1, 1), c(0, 0, 1)],
      [c(1, 0, 0), c(1, 1, 0), c(1, 1, 1), c(1, 0, 1)],
    ];
    for (let f of faces) {
      let n = unit(cross(sub(f[2]!, f[0]!), sub(f[3]!, f[1]!)));
      const mid = mul(add(add(f[0]!, f[1]!), add(f[2]!, f[3]!)), 0.25);
      if (dot(n, sub(mid, centre)) < 0) {
        f = [f[0]!, f[3]!, f[2]!, f[1]!];
        n = mul(n, -1);
      }
      const base = this.cellOf.length;
      for (const p of f) {
        this.positions.push(...p);
        this.normals.push(...n);
        this.colours.push(...rgb);
        this.cellOf.push(cell);
      }
      this.indices.push(base, base + 1, base + 2, base, base + 2, base + 3);
    }
  }

  finish(rest: Float64Array, cells: SkinCell[]): PanelSet {
    const positions = Float64Array.from(this.positions);
    const normals = Float64Array.from(this.normals);
    const cellOf = Uint32Array.from(this.cellOf);
    return {
      positions,
      normals,
      colours: Float32Array.from(this.colours),
      indices: Uint32Array.from(this.indices),
      cellOf,
      cells,
      rest,
      binding: bindMesh(rest, cells, positions, normals, cellOf),
      panelStart: [...this.panelStart, cellOf.length],
    };
  }
}

const flatten = (nodes: Point3[]): Float64Array => Float64Array.from(nodes.flatMap((p) => [p.x, p.y, p.z]));

// ---- tower ---------------------------------------------------------------------------------

/** Floor slab: inset from the ring beams, from just below the ring plane up. */
const FLOOR_INSET = 0.12;
const FLOOR_LOW = -0.02;
const FLOOR_HIGH = 0.14;
/** Wall panels: a band from above the floor slab, standing out past the nodes, with the corners open. */
const WALL_INSET = 0.25;
const WALL_FROM = 0.2;
const WALL_TO = 1.15;
const WALL_OUT = 0.2;
const WALL_THICK = 0.07;

/**
 * The tower's panels. Storey `s` is one cell: its four floor corners and four ceiling corners, with
 * one tetrahedron anchored at each corner (the corner, its two ring neighbours and the corner
 * straight above or below). It carries its floor slab and a wall band on each of its four faces;
 * the top storey also carries the roof slab.
 */
export function towerPanels(spec: TowerSpec): PanelSet {
  const { struct } = towerStruct(spec);
  const rest = flatten(struct.nodes);
  const id = (l: number, c: number): number => 4 * l + (c & 3);
  const cells: SkinCell[] = [];
  for (let s = 0; s < spec.storeys; s++) {
    const tets: Tet[] = [];
    for (let c = 0; c < 4; c++) tets.push([id(s, c), id(s, c + 1), id(s, c + 3), id(s + 1, c)]);
    for (let c = 0; c < 4; c++) tets.push([id(s + 1, c), id(s + 1, c + 1), id(s + 1, c + 3), id(s, c)]);
    cells.push({ tets });
  }
  const P = (l: number, c: number): V => pt(struct.nodes[id(l, c)]!);
  const up: V = [0, 1, 0];
  const b = new PanelBuilder();
  for (let l = 0; l <= spec.storeys; l++) {
    const cell = Math.min(l, spec.storeys - 1);
    const a = P(l, 0);
    const u = sub(P(l, 1), a);
    const v = sub(P(l, 3), a);
    const lu = length(u);
    const lv = length(v);
    const o = add(add(add(a, mul(u, FLOOR_INSET / lu)), mul(v, FLOOR_INSET / lv)), mul(up, FLOOR_LOW));
    b.box(cell, o, mul(u, (lu - 2 * FLOOR_INSET) / lu), mul(v, (lv - 2 * FLOOR_INSET) / lv), mul(up, FLOOR_HIGH - FLOOR_LOW), FLOOR_RGB);
  }
  for (let s = 0; s < spec.storeys; s++) {
    const centre = mul(add(P(s, 0), P(s, 2)), 0.5);
    for (let c = 0; c < 4; c++) {
      const a = P(s, c);
      const edge = sub(P(s, c + 1), a);
      const le = length(edge);
      const along = mul(edge, 1 / le);
      let out = unit(cross(along, up));
      if (dot(out, sub(mul(add(a, P(s, c + 1)), 0.5), centre)) < 0) out = mul(out, -1);
      const o = add(add(add(a, mul(along, WALL_INSET)), mul(out, WALL_OUT)), mul(up, WALL_FROM));
      b.box(s, o, mul(along, le - 2 * WALL_INSET), mul(out, WALL_THICK), mul(up, WALL_TO - WALL_FROM), WALL_RGB);
    }
  }
  return b.finish(rest, cells);
}

// ---- bridge --------------------------------------------------------------------------------

/** Deck slabs: a short gap between bays so the seams show, inset from the chords across. */
const DECK_GAP = 0.04;
const DECK_INSET = 0.12;

/**
 * The bridge's deck slabs. Bay `i` is one cell: its four deck nodes and the two top nodes over it,
 * with one tetrahedron anchored at each deck corner (the corner, its neighbours along and across
 * the deck, and the top node of its own truss). The deck nodes alone are in one plane, so every
 * tetrahedron needs a top node.
 */
export function bridgePanels(spec: BridgeSpec3): PanelSet {
  const { struct, layout } = bridgeStruct(spec);
  const rest = flatten(struct.nodes);
  const { deck, top } = layout;
  const cells: SkinCell[] = [];
  for (let i = 0; i < spec.bays; i++) {
    cells.push({
      tets: [
        [deck(0, i), deck(0, i + 1), deck(1, i), top(0, i)],
        [deck(0, i + 1), deck(0, i), deck(1, i + 1), top(0, i)],
        [deck(1, i), deck(1, i + 1), deck(0, i), top(1, i)],
        [deck(1, i + 1), deck(1, i), deck(0, i + 1), top(1, i)],
      ],
    });
  }
  const b = new PanelBuilder();
  const up: V = [0, 1, 0];
  for (let i = 0; i < spec.bays; i++) {
    const a = pt(struct.nodes[deck(0, i)]!);
    const u = sub(pt(struct.nodes[deck(0, i + 1)]!), a);
    const v = sub(pt(struct.nodes[deck(1, i)]!), a);
    const lu = length(u);
    const lv = length(v);
    const o = add(add(a, mul(u, DECK_GAP / 2 / lu)), mul(v, DECK_INSET / lv));
    b.box(i, o, mul(u, (lu - DECK_GAP) / lu), mul(v, (lv - 2 * DECK_INSET) / lv), mul(up, DECK_SLAB_TOP), DECK_RGB);
  }
  return b.finish(rest, cells);
}
