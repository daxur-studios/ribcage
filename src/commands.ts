/**
 * Purpose: the command buffer (layout rule 8) on the TypeScript side: a list of fixed-layout
 * records that the host fills and the core applies, in order, before a step.
 * Value: every change to a running world is one of these records, so a recording is just "the
 * list per step" and a replay applies the same lists again (milestone 2.4). Nothing here talks
 * to the core; a batch is plain bytes you can copy, save and compare.
 *
 * Layout (`command::Command` in core/src/command.rs, 64 bytes, little endian):
 *   0 kind u32 | 4 a u32 | 8 b u32 | 12 reserved u32 | 16..64 six f64 numbers
 */

export const CMD_ADD_NODE = 1;
export const CMD_ADD_BEAM = 2;
export const CMD_DELETE_NODE = 3;
export const CMD_DELETE_BEAM = 4;
export const CMD_ADD_FORCE = 5;
export const CMD_SET_INV_MASS = 6;
export const CMD_SET_TERRAIN = 7;
export const CMD_TERRAIN_HEIGHTS = 8;
export const CMD_ADD_THRUSTER = 9;
export const CMD_SET_THROTTLE = 10;
export const CMD_ADD_SEGMENT = 11;
export const CMD_SET_NODE_SEGMENT = 12;

/** No segment: a root's parent, a node with no segment (`segment::NO_SEGMENT`). */
export const NO_SEGMENT = 0xffff_ffff;
/** Segment flag: never simulated as a live lattice once rigid bodies exist (`segment::SEG_NEVER_LIVE`). */
export const SEG_NEVER_LIVE = 1;

/** Most grid points along one side of a terrain (`terrain::MAX_GRID`). */
export const MAX_TERRAIN_GRID = 257;
/** Most heights one `CMD_TERRAIN_HEIGHTS` record carries. */
export const HEIGHTS_PER_RECORD = 6;

/** The result slot of a rejected record (`command::REJECTED`). */
export const REJECTED = 0xffff_ffff;

/** Bytes per record; the loader checks this against the core's own number. */
export const COMMAND_BYTES = 64;

/** Parameters of a beam, as in `world::BeamSpec`. */
export interface BeamSpec {
  rest: number;
  compliance: number;
  damping: number;
  yieldStrain: number;
  breakStrain: number;
}

export class CommandBatch {
  #buf = new ArrayBuffer(COMMAND_BYTES * 64);
  #view = new DataView(this.#buf);
  #count = 0;

  /** Records in the batch. */
  get count(): number {
    return this.#count;
  }

  /** The used bytes (a view, valid until the next add). Copy it to keep it. */
  get bytes(): Uint8Array {
    return new Uint8Array(this.#buf, 0, this.#count * COMMAND_BYTES);
  }

  clear(): void {
    this.#count = 0;
  }

  /** A batch holding exactly these bytes (a recording read back). */
  static fromBytes(bytes: Uint8Array): CommandBatch {
    if (bytes.byteLength % COMMAND_BYTES !== 0) {
      throw new Error(`command bytes (${bytes.byteLength}) are not a whole number of ${COMMAND_BYTES}-byte records`);
    }
    const b = new CommandBatch();
    b.#reserve(bytes.byteLength / COMMAND_BYTES);
    new Uint8Array(b.#buf).set(bytes);
    b.#count = bytes.byteLength / COMMAND_BYTES;
    return b;
  }

  #reserve(records: number): void {
    const need = records * COMMAND_BYTES;
    if (need <= this.#buf.byteLength) return;
    let size = this.#buf.byteLength;
    while (size < need) size *= 2;
    const next = new ArrayBuffer(size);
    new Uint8Array(next).set(new Uint8Array(this.#buf, 0, this.#count * COMMAND_BYTES));
    this.#buf = next;
    this.#view = new DataView(next);
  }

  #push(kind: number, a: number, b: number, f: readonly number[]): number {
    this.#reserve(this.#count + 1);
    const o = this.#count * COMMAND_BYTES;
    const v = this.#view;
    v.setUint32(o, kind, true);
    v.setUint32(o + 4, a, true);
    v.setUint32(o + 8, b, true);
    v.setUint32(o + 12, 0, true);
    for (let i = 0; i < 6; i++) v.setFloat64(o + 16 + i * 8, f[i] ?? 0, true);
    return this.#count++;
  }

  /** Returns the record's position in the batch; the core's result for it is at that slot. */
  addNode(x: number, y: number, z: number, invMass: number, radius: number): number {
    return this.#push(CMD_ADD_NODE, 0, 0, [x, y, z, invMass, radius]);
  }

  addBeam(a: number, b: number, s: BeamSpec): number {
    return this.#push(CMD_ADD_BEAM, a, b, [s.rest, s.compliance, s.damping, s.yieldStrain, s.breakStrain]);
  }

  deleteNode(node: number): number {
    return this.#push(CMD_DELETE_NODE, node, 0, []);
  }

  deleteBeam(beam: number): number {
    return this.#push(CMD_DELETE_BEAM, beam, 0, []);
  }

  /** Newtons on a node for the coming step only. */
  addForce(node: number, fx: number, fy: number, fz: number): number {
    return this.#push(CMD_ADD_FORCE, node, 0, [fx, fy, fz]);
  }

  /** Zero pins the node; anything else frees it with that inverse mass. */
  setInvMass(node: number, invMass: number): number {
    return this.#push(CMD_SET_INV_MASS, node, 0, [invMass]);
  }

  /**
   * Sets a flat terrain (all heights zero) of `nx` by `nz` points, `cell` metres apart, its first
   * point at `(x0, z0)`, with one friction coefficient. Replaces any terrain the world had.
   */
  setTerrain(nx: number, nz: number, x0: number, z0: number, cell: number, friction: number): number {
    return this.#push(CMD_SET_TERRAIN, nx, nz, [x0, z0, cell, friction]);
  }

  /** Writes up to six heights starting at grid index `start` (`i + j * nx`). */
  terrainHeights(start: number, heights: readonly number[]): number {
    if (heights.length < 1 || heights.length > HEIGHTS_PER_RECORD) {
      throw new Error(`a terrain record carries 1 to ${HEIGHTS_PER_RECORD} heights, not ${heights.length}`);
    }
    return this.#push(CMD_TERRAIN_HEIGHTS, start, heights.length, heights);
  }

  /**
   * Adds a thruster from node `tail` to node `head` with a maximum thrust in newtons; its throttle
   * starts at zero. The core's result for it is the thruster's index (numbered in the order added).
   */
  addThruster(tail: number, head: number, maxThrust: number): number {
    return this.#push(CMD_ADD_THRUSTER, tail, head, [maxThrust]);
  }

  /** Sets a thruster's throttle, 0 to 1. It pushes every step from now until it is set again. */
  setThrottle(thruster: number, throttle: number): number {
    return this.#push(CMD_SET_THROTTLE, thruster, 0, [throttle]);
  }

  /**
   * Adds a segment (plan 005). `parent` is a segment already added, or `NO_SEGMENT` for a root;
   * `area` and `bending` describe the seam to the parent (finite, zero or more; a root's are checked
   * and not stored). The core's result is the segment's index, numbered in the order added.
   */
  addSegment(parent: number, flags: number, area: number, bending: number): number {
    return this.#push(CMD_ADD_SEGMENT, parent, flags, [area, bending]);
  }

  /** Puts a node in a segment; its mass joins the segment's stored mass. A node has one segment. */
  setNodeSegment(node: number, segment: number): number {
    return this.#push(CMD_SET_NODE_SEGMENT, node, segment, []);
  }

  /** The whole height array as records of six (the last may be shorter), in index order. */
  allTerrainHeights(heights: ArrayLike<number>): void {
    for (let i = 0; i < heights.length; i += HEIGHTS_PER_RECORD) {
      this.terrainHeights(i, Array.from({ length: Math.min(HEIGHTS_PER_RECORD, heights.length - i) }, (_, k) => heights[i + k]!));
    }
  }
}
