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
export const CMD_MAKE_RIGID = 13;
export const CMD_RELEASE_RIGID = 14;
/** Adds a hinge joint (plan 005, phase 5.5). See `CommandBatch.addJoint`. */
export const CMD_ADD_JOINT = 15;
/** Sets a joint's drive. See `CommandBatch.setJointTarget`. */
export const CMD_SET_JOINT_TARGET = 16;
/** Switches sleeping on or off (plan 005, phase 5.8). See `CommandBatch.setSleeping`. */
export const CMD_SET_SLEEPING = 17;
/** Breaks a beam as if it had snapped (plan 005, phase 5.10). See `CommandBatch.breakBeam`. */
export const CMD_BREAK_BEAM = 18;
/** Sets the air: density, scale height, wind, speed of sound (plan 006, phase 6.1). See `CommandBatch.setAir`. */
export const CMD_SET_AIR = 19;
/** Adds an aero element (plan 006, phase 6.2). See `CommandBatch.addAeroElement`. */
export const CMD_ADD_AERO_ELEMENT = 20;
/** Adds a weighted node to the newest aero element. See `CommandBatch.addAeroNode`. */
export const CMD_ADD_AERO_NODE = 21;
/** Sets an aero element's frame nodes. See `CommandBatch.setAeroFrame`. */
export const CMD_SET_AERO_FRAME = 22;
/** Joint drive modes (`joint::JOINT_FREE`, `JOINT_SERVO`, `JOINT_ROTOR`). */
export const JOINT_FREE = 0;
export const JOINT_SERVO = 1;
export const JOINT_ROTOR = 2;

/** No segment: a root's parent, a node with no segment (`segment::NO_SEGMENT`). */
export const NO_SEGMENT = 0xffff_ffff;
/** No body: a loose node, a segment that is not rigid (`body::NO_BODY`). */
export const NO_BODY = 0xffff_ffff;
/** Body flag: the slot holds a live body (`body::BODY_ALIVE`). */
export const BODY_ALIVE = 1;
/** Segment flag: never simulated as a live lattice once rigid bodies exist (`segment::SEG_NEVER_LIVE`). */
export const SEG_NEVER_LIVE = 1;
/** Segment flag: the sleep module never puts it to sleep (`segment::SEG_NEVER_SLEEP`; a wing in flight). */
export const SEG_NEVER_SLEEP = 2;

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

  /**
   * Makes a segment rigid (plan 005, phase 5.2), merged with `withSegment`'s group when given (and
   * with any body either already rides). Keeps momentum and spin; no node moves. The core's result
   * is the body's slot (its lowest segment), or rejected for a rod, a single node or no mass.
   */
  makeRigid(segment: number, withSegment: number = NO_SEGMENT): number {
    return this.#push(CMD_MAKE_RIGID, segment, withSegment, []);
  }

  /** Releases a rigid segment: its nodes go loose with the body's velocity, the rest stays rigid. */
  releaseRigid(segment: number): number {
    return this.#push(CMD_RELEASE_RIGID, segment, 0, []);
  }

  /**
   * Adds a hinge joint (plan 005, phase 5.5). Each side is two nodes on the axis and one reference
   * node off it; each node may be loose, pinned or riding a rigid body. `a0` is pinned to `b0` and
   * `a1` to `b1`, so only the turn about the axis is free. The angle is zero in the current pose;
   * `min` and `max` are limits in radians, `-pi <= min <= 0 <= max <= pi` (`-pi` and `pi` for none).
   * The core's result is the joint's index, or rejected for a dead or repeated node, all six on one
   * body, a limit out of range, or a zero-length axis or arm.
   */
  addJoint(a0: number, a1: number, refA: number, b0: number, b1: number, refB: number, min: number = -Math.PI, max: number = Math.PI): number {
    return this.#push(CMD_ADD_JOINT, a0, b0, [a1, b1, refA, refB, min, max]);
  }

  /**
   * Sets a joint's drive: `JOINT_FREE`, `JOINT_SERVO` (target angle in [-pi, pi], moved toward at
   * `rate` rad/s) or `JOINT_ROTOR` (target speed in rad/s), with the drive torque capped at `cap`
   * newton-metres in every substep.
   */
  setJointTarget(joint: number, mode: number, target: number, rate: number, cap: number): number {
    return this.#push(CMD_SET_JOINT_TARGET, joint, mode, [target, rate, cap]);
  }

  /**
   * Switches sleeping on or off (plan 005, phase 5.8): barely loaded segments go rigid and wake
   * under load or on a trigger. On also switches the stress estimate on. Off wakes every sleeping
   * segment at the start of the next step. Off by default.
   */
  setSleeping(on: boolean): number {
    return this.#push(CMD_SET_SLEEPING, on ? 1 : 0, 0, []);
  }

  /**
   * Breaks a beam as if it had snapped (plan 005, phase 5.10): it stays in its slot, flagged
   * broken, and carries nothing. The coming step checks whether the structure fell into separate
   * vessels. Rejected for a beam that is not live or is already broken. No break event.
   */
  breakBeam(beam: number): number {
    return this.#push(CMD_BREAK_BEAM, beam, 0, []);
  }

  /**
   * Sets the air (plan 006, phase 6.1): sea-level density in kg/m^3 (default 1.225), scale height
   * in metres (8500; 0 means constant density), wind velocity in m/s (zero) and speed of sound in
   * m/s (340, above zero). Density at height y is `rho0 * exp(-y / H)`. Replaces the settings whole;
   * a world that never sends it has no air entry in its hash.
   */
  setAir(rho0: number, scaleHeight: number, wind: readonly [number, number, number], soundSpeed: number): number {
    return this.#push(CMD_SET_AIR, 0, 0, [rho0, scaleHeight, wind[0], wind[1], wind[2], soundSpeed]);
  }

  /**
   * Adds an aero element (plan 006, phase 6.2). Kind 1 is a body station; `coef` is
   * `[A_front, C_A, A_base, A_plan, C_base]` (finite, zero or more; missing ones are zero). The
   * result is the element index. Follow with `addAeroNode` for each node and `setAeroFrame`.
   */
  addAeroElement(kind: number, coef: readonly number[]): number {
    const f = [0, 0, 0, 0, 0, 0];
    for (let i = 0; i < Math.min(coef.length, 6); i++) f[i] = coef[i]!;
    return this.#push(CMD_ADD_AERO_ELEMENT, kind, 0, f);
  }

  /** Adds a weighted node to the newest aero element (weight above zero; weights are normalised). */
  addAeroNode(element: number, node: number, weight: number): number {
    return this.#push(CMD_ADD_AERO_NODE, element, node, [weight]);
  }

  /** Sets an aero element's frame nodes; a station needs `[tail, nose]`. Unused slots are omitted. */
  setAeroFrame(element: number, frame: readonly number[]): number {
    const f = [frame[1] ?? 0, frame[2] ?? 0xffffffff, frame[3] ?? 0xffffffff];
    return this.#push(CMD_SET_AERO_FRAME, element, frame[0] ?? 0xffffffff, f);
  }

  /** The whole height array as records of six (the last may be shorter), in index order. */
  allTerrainHeights(heights: ArrayLike<number>): void {
    for (let i = 0; i < heights.length; i += HEIGHTS_PER_RECORD) {
      this.terrainHeights(i, Array.from({ length: Math.min(HEIGHTS_PER_RECORD, heights.length - i) }, (_, k) => heights[i + k]!));
    }
  }
}
