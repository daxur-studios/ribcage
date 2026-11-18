/**
 * Purpose: the skin of an assembled craft: one thin slab on every skin cell its parts supply, bound
 * to the cell's nodes so it moves, tilts and bends with them (`skin.ts`).
 * Value: parts give only cells, no mesh (ER 004), so this is the one place a craft's outside is made;
 * the craft page draws it with the same node texture and vertex function as the tower and the bridge.
 *
 * A tube or frustum wall cell is four tetrahedra on one quad of the wall, with a node across the
 * section as each one's fourth point; the slab stands on that quad, on the side away from those
 * inner nodes. A pod nose cell is three tetrahedra on one triangle. Sizes are drawing
 * **[assumption]**s: the slab clears the drawn beams (radius 0.07 m) so they stay visible as ribs.
 */

import { PanelBuilder, type PanelSet } from './panels3d.ts';
import type { Craft } from './parts/assemble.ts';
import type { SkinCell } from './skin.ts';

type V = [number, number, number];
type Rgb = readonly [number, number, number];

/** The colour of the skin by part type. */
export const SKIN_RGB: Readonly<Record<string, Rgb>> = {
  tank: [0.82, 0.83, 0.86],
  pod: [0.92, 0.9, 0.84],
  engine: [0.5, 0.44, 0.4],
  truss: [0.62, 0.64, 0.68],
  leg: [0.7, 0.7, 0.72],
};
const FALLBACK_RGB: Rgb = [0.7, 0.7, 0.7];

/** The slab stands this far off the nodes' plane, and is this thick, metres. */
export const SKIN_STANDOFF = 0.06;
export const SKIN_THICKNESS = 0.05;

const sub = (a: V, b: V): V => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const add = (a: V, b: V): V => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const mul = (a: V, k: number): V => [a[0] * k, a[1] * k, a[2] * k];
const dot = (a: V, b: V): number => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a: V, b: V): V => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const unit = (a: V): V => mul(a, 1 / Math.sqrt(dot(a, a)));

/** The surface of one cell: its corner nodes (3 or 4) and an interior node the slab stands away from. */
function cellSurface(cell: SkinCell): { corners: number[]; inner: number } {
  const t0 = cell.tets[0]!;
  const inner = t0[3];
  if (cell.tets.length === 4) {
    // [a,b,c,x], [b,a,d,x], ...: the corners are a, b, c and the third node of the second tetrahedron.
    return { corners: [t0[0], t0[1], t0[2], cell.tets[1]![2]], inner };
  }
  if (cell.tets.length === 3) return { corners: [t0[0], t0[1], t0[2]], inner };
  throw new Error(`a skin cell of ${cell.tets.length} tetrahedra has no slab shape (3 or 4 expected)`);
}

/** The corners in order round the polygon, by angle about their centre in their plane. */
function roundOrder(points: V[], normal: V): number[] {
  let c: V = [0, 0, 0];
  for (const p of points) c = add(c, p);
  c = mul(c, 1 / points.length);
  const e1 = unit(sub(points[0]!, c));
  const e2 = cross(normal, e1);
  const angle = points.map((p) => Math.atan2(dot(sub(p, c), e2), dot(sub(p, c), e1)));
  return points.map((_, i) => i).sort((i, j) => angle[i]! - angle[j]!);
}

/** The type of the part that owns every one of `nodes`, or `''`. */
function ownerType(craft: Craft, nodes: readonly number[]): string {
  for (const t of craft.tree) {
    const own = new Set(t.nodeOf);
    if (nodes.every((n) => own.has(n))) return t.type;
  }
  return '';
}

/** The craft's skin: one slab per cell of `craft.panels`, in cell order. */
export function craftPanels(craft: Craft): PanelSet {
  const rest = new Float64Array(3 * craft.nodes.length);
  for (const [i, n] of craft.nodes.entries()) {
    rest[3 * i] = n.x;
    rest[3 * i + 1] = n.y;
    rest[3 * i + 2] = n.z;
  }
  const at = (i: number): V => [rest[3 * i]!, rest[3 * i + 1]!, rest[3 * i + 2]!];
  const b = new PanelBuilder();
  for (const [k, cell] of craft.panels.entries()) {
    const { corners, inner } = cellSurface(cell);
    const pts = corners.map(at);
    let normal = unit(cross(sub(pts[1]!, pts[0]!), sub(pts[2]!, pts[0]!)));
    const order = roundOrder(pts, normal);
    const ring = order.map((i) => pts[i]!);
    let centre: V = [0, 0, 0];
    for (const p of ring) centre = add(centre, p);
    centre = mul(centre, 1 / ring.length);
    if (dot(normal, sub(centre, at(inner))) < 0) normal = mul(normal, -1);
    const rgb = SKIN_RGB[ownerType(craft, [...corners, inner])] ?? FALLBACK_RGB;
    b.prism(k, ring, normal, SKIN_STANDOFF, SKIN_STANDOFF + SKIN_THICKNESS, rgb);
  }
  return b.finish(rest, craft.panels);
}
