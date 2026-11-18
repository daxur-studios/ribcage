/**
 * Purpose: loads the Rust core (WebAssembly) and exposes its C-style boundary as typed methods.
 * Value: one place that knows the ABI (core/src/ffi.rs), so the rest of the host never touches raw
 * exports. Runs in Node and in the browser: it takes the module's bytes, not a file path.
 *
 * Memory rule: WebAssembly memory can move when it grows, so a view is made fresh from
 * `memory.buffer` for every read and never kept across a call into the core.
 */

import { COMMAND_BYTES, type CommandBatch } from './commands.ts';

/** The ABI version this loader was written for (`ABI_VERSION` in core/src/ffi.rs). */
export const ABI_VERSION = 6;

/** A host pointer inside the core (an integer to JavaScript). */
export type HostPtr = number;

interface Exports {
  memory: WebAssembly.Memory;
  rc_abi_version(): number;
  rc_host_new(): number;
  rc_bench_new(standard: number): number;
  rc_host_free(h: number): void;
  rc_add_node(h: number, x: number, y: number, z: number, invMass: number, radius: number): number;
  rc_add_beam(
    h: number,
    a: number,
    b: number,
    rest: number,
    compliance: number,
    damping: number,
    yieldStrain: number,
    breakStrain: number,
  ): number;
  rc_add_force(h: number, node: number, fx: number, fy: number, fz: number): void;
  rc_step(h: number): number;
  rc_bench_step(h: number): number;
  rc_state_hash(h: number): bigint;
  rc_step_count(h: number): bigint;
  rc_node_count(h: number): number;
  rc_beam_count(h: number): number;
  rc_pos_ptr(h: number): number;
  rc_event_count(h: number): number;
  rc_events_ptr(h: number): number;
  rc_write_render(h: number): number;
  rc_command_bytes(): number;
  rc_commands_reserve(h: number, count: number): number;
  rc_apply_commands(h: number): number;
  rc_results_ptr(h: number): number;
  rc_node_flags_ptr(h: number): number;
  rc_beam_flags_ptr(h: number): number;
  rc_write_beams(h: number): number;
  rc_beam_ratio_ptr(h: number): number;
  rc_physics_hash(h: number): bigint;
  rc_segment_count(h: number): number;
  rc_seam_count(h: number): number;
  rc_node_segment_ptr(h: number): number;
  rc_segment_flags_ptr(h: number): number;
  rc_segment_parent_ptr(h: number): number;
  rc_segment_seam_ptr(h: number): number;
  rc_segment_mass_ptr(h: number): number;
  rc_seam_a_ptr(h: number): number;
  rc_seam_b_ptr(h: number): number;
  rc_seam_area_ptr(h: number): number;
  rc_seam_bending_ptr(h: number): number;
  rc_segment_props(h: number, segment: number): number;
  rc_body_slot_count(h: number): number;
  rc_body_flags_ptr(h: number): number;
  rc_body_centre_ptr(h: number): number;
  rc_body_rot_ptr(h: number): number;
  rc_body_vel_ptr(h: number): number;
  rc_body_omega_ptr(h: number): number;
  rc_body_mass_ptr(h: number): number;
  rc_node_body_ptr(h: number): number;
  rc_segment_body_ptr(h: number): number;
}

/** Segments, seams and each node's segment, as copies (plan 005, phase 5.1). */
export interface SegmentView {
  /** Segment per node slot (`NO_SEGMENT` for none). */
  nodeSegment: Uint32Array;
  flags: Uint32Array;
  /** Parent segment per segment (`NO_SEGMENT` for a root). */
  parent: Uint32Array;
  /** Seam to the parent per segment (`NO_SEGMENT` for a root). */
  seam: Uint32Array;
  /** Stored mass per segment. */
  mass: Float64Array;
  /** Per seam: the parent segment, the child segment, axial area and bending section. */
  seamA: Uint32Array;
  seamB: Uint32Array;
  seamArea: Float64Array;
  seamBending: Float64Array;
}

/** Rigid bodies, as copies (plan 005, phase 5.2). A slot is numbered as its body's lowest segment. */
export interface BodyView {
  /** Flags per slot (`BODY_ALIVE`); empty until the first body is made. */
  flags: Uint32Array;
  /** Per slot, three numbers each: centre of mass, linear velocity, angular velocity (world frame). */
  centre: Float64Array;
  vel: Float64Array;
  omega: Float64Array;
  /** Per slot, four numbers `w x y z`: the rotation from the body frame to the world. */
  rot: Float64Array;
  mass: Float64Array;
  /** Body per node slot and per segment (`NO_BODY` for none). */
  nodeBody: Uint32Array;
  segmentBody: Uint32Array;
}

/** A segment's mass, centre and inertia from the current positions (`segment::SegmentProps`). */
export interface SegmentProps {
  mass: number;
  centre: [number, number, number];
  /** `[xx, yy, zz, xy, xz, yz]`, the products negated. */
  inertia: [number, number, number, number, number, number];
}

/** What one batch of commands did: a result per record and the number rejected. */
export interface ApplyResult {
  /** New index for an add, 1 for another accepted command, `REJECTED` for a rejected one. */
  results: Uint32Array;
  rejected: number;
}

/** Per-beam output for drawing (one entry per beam slot, live or not). */
export interface BeamView {
  /** Two node indices per beam. */
  ends: Uint32Array;
  /** Signed strain in units of the yield strain: 1 is at yield, negative is compression. */
  ratio: Float32Array;
  /** `BEAM_*` flags. */
  flags: Uint32Array;
}

export const BEAM_ALIVE = 1;
export const BEAM_BROKEN = 2;
export const BEAM_YIELDED = 4;
export const NODE_ALIVE = 1;
export const NODE_PINNED = 2;
export const NODE_FROZEN = 4;

/** One event record (`event::Event`: 24 bytes, little endian). */
export interface CoreEvent {
  kind: number;
  index: number;
  step: bigint;
  value: number;
}

export const EVENT_YIELD = 1;
export const EVENT_BREAK = 2;
export const EVENT_FREEZE = 3;

const EVENT_BYTES = 24;

export class Core {
  readonly #x: Exports;

  private constructor(x: Exports) {
    this.#x = x;
  }

  /** Compiles and starts the core from the bytes of `beam_core.wasm`. */
  static async load(bytes: BufferSource): Promise<Core> {
    // The module imports nothing (checked by test/wasm.test.ts), so the import object is empty.
    const { instance } = await WebAssembly.instantiate(bytes, {});
    const x = instance.exports as unknown as Exports;
    const version = x.rc_abi_version();
    if (version !== ABI_VERSION) {
      throw new Error(`core ABI version ${version}, loader expects ${ABI_VERSION}: rebuild the wasm`);
    }
    if (x.rc_command_bytes() !== COMMAND_BYTES) {
      throw new Error(`core command record is ${x.rc_command_bytes()} bytes, loader expects ${COMMAND_BYTES}`);
    }
    return new Core(x);
  }

  /** Size of the module's linear memory in bytes (grows as scenes grow). */
  get memoryBytes(): number {
    return this.#x.memory.buffer.byteLength;
  }

  hostNew(): HostPtr {
    return this.#x.rc_host_new();
  }

  /** The benchmark scene: `standard` is the reported 11,136-beam scene, otherwise the small one. */
  benchNew(standard: boolean): HostPtr {
    return this.#x.rc_bench_new(standard ? 1 : 0);
  }

  hostFree(h: HostPtr): void {
    this.#x.rc_host_free(h);
  }

  addNode(h: HostPtr, x: number, y: number, z: number, invMass: number, radius: number): number {
    return this.#x.rc_add_node(h, x, y, z, invMass, radius);
  }

  addBeam(
    h: HostPtr,
    a: number,
    b: number,
    spec: { rest: number; compliance: number; damping: number; yieldStrain: number; breakStrain: number },
  ): number {
    return this.#x.rc_add_beam(h, a, b, spec.rest, spec.compliance, spec.damping, spec.yieldStrain, spec.breakStrain);
  }

  addForce(h: HostPtr, node: number, fx: number, fy: number, fz: number): void {
    this.#x.rc_add_force(h, node, fx, fy, fz);
  }

  /** One fixed 60 Hz step; returns the largest substep count any island used. */
  step(h: HostPtr): number {
    return this.#x.rc_step(h);
  }

  /** The benchmark scene's gust, then one step. */
  benchStep(h: HostPtr): number {
    return this.#x.rc_bench_step(h);
  }

  stateHash(h: HostPtr): bigint {
    // i64 comes back signed; the hash is an unsigned 64-bit value.
    return BigInt.asUintN(64, this.#x.rc_state_hash(h));
  }

  /** The state hash without the segment section (equal to `stateHash` until a segment exists). */
  physicsHash(h: HostPtr): bigint {
    return BigInt.asUintN(64, this.#x.rc_physics_hash(h));
  }

  stepCount(h: HostPtr): bigint {
    return BigInt.asUintN(64, this.#x.rc_step_count(h));
  }

  nodeCount(h: HostPtr): number {
    return this.#x.rc_node_count(h);
  }

  beamCount(h: HostPtr): number {
    return this.#x.rc_beam_count(h);
  }

  /** A copy of the node positions at full precision, three numbers per node. */
  positions(h: HostPtr): Float64Array {
    const n = this.#x.rc_node_count(h) * 3;
    const ptr = this.#x.rc_pos_ptr(h);
    return new Float64Array(this.#x.memory.buffer, ptr, n).slice();
  }

  /** A copy of the render buffer: positions as 32-bit floats, three per node. Call once per frame. */
  render(h: HostPtr): Float32Array {
    const n = this.#x.rc_node_count(h) * 3;
    const ptr = this.#x.rc_write_render(h);
    return new Float32Array(this.#x.memory.buffer, ptr, n).slice();
  }

  /** The events of the last step, in the core's order. */
  events(h: HostPtr): CoreEvent[] {
    const n = this.#x.rc_event_count(h);
    const ptr = this.#x.rc_events_ptr(h);
    const view = new DataView(this.#x.memory.buffer, ptr, n * EVENT_BYTES);
    const out: CoreEvent[] = [];
    for (let i = 0; i < n; i++) {
      const o = i * EVENT_BYTES;
      out.push({
        kind: view.getUint32(o, true),
        index: view.getUint32(o + 4, true),
        step: view.getBigUint64(o + 8, true),
        value: view.getFloat64(o + 16, true),
      });
    }
    return out;
  }

  /**
   * Applies a batch of commands in order (rule 8). Call before the step, never during one.
   * An empty batch is a no-op.
   */
  applyCommands(h: HostPtr, batch: CommandBatch): ApplyResult {
    const n = batch.count;
    if (n === 0) return { results: new Uint32Array(0), rejected: 0 };
    // Reserve first (it can grow memory), then take a fresh view to copy the bytes in.
    const ptr = this.#x.rc_commands_reserve(h, n);
    new Uint8Array(this.#x.memory.buffer, ptr, n * COMMAND_BYTES).set(batch.bytes);
    const rejected = this.#x.rc_apply_commands(h);
    const results = new Uint32Array(this.#x.memory.buffer, this.#x.rc_results_ptr(h), n).slice();
    return { results, rejected };
  }

  /** Beam ends, strain ratios and flags, as copies. Call once per rendered frame. */
  beams(h: HostPtr): BeamView {
    const n = this.#x.rc_beam_count(h);
    const endsPtr = this.#x.rc_write_beams(h);
    const ratioPtr = this.#x.rc_beam_ratio_ptr(h);
    const flagsPtr = this.#x.rc_beam_flags_ptr(h);
    const mem = this.#x.memory.buffer;
    return {
      ends: new Uint32Array(mem, endsPtr, n * 2).slice(),
      ratio: new Float32Array(mem, ratioPtr, n).slice(),
      flags: new Uint32Array(mem, flagsPtr, n).slice(),
    };
  }

  /** A copy of the node flags, one per node slot. */
  nodeFlags(h: HostPtr): Uint32Array {
    return new Uint32Array(this.#x.memory.buffer, this.#x.rc_node_flags_ptr(h), this.#x.rc_node_count(h)).slice();
  }

  /** Segments, seams and the node-to-segment table, as copies. */
  segments(h: HostPtr): SegmentView {
    const x = this.#x;
    const nodes = x.rc_node_count(h);
    const segs = x.rc_segment_count(h);
    const seams = x.rc_seam_count(h);
    const mem = x.memory.buffer;
    return {
      nodeSegment: new Uint32Array(mem, x.rc_node_segment_ptr(h), nodes).slice(),
      flags: new Uint32Array(mem, x.rc_segment_flags_ptr(h), segs).slice(),
      parent: new Uint32Array(mem, x.rc_segment_parent_ptr(h), segs).slice(),
      seam: new Uint32Array(mem, x.rc_segment_seam_ptr(h), segs).slice(),
      mass: new Float64Array(mem, x.rc_segment_mass_ptr(h), segs).slice(),
      seamA: new Uint32Array(mem, x.rc_seam_a_ptr(h), seams).slice(),
      seamB: new Uint32Array(mem, x.rc_seam_b_ptr(h), seams).slice(),
      seamArea: new Float64Array(mem, x.rc_seam_area_ptr(h), seams).slice(),
      seamBending: new Float64Array(mem, x.rc_seam_bending_ptr(h), seams).slice(),
    };
  }

  /** Rigid bodies and the node and segment tables, as copies. */
  bodies(h: HostPtr): BodyView {
    const x = this.#x;
    const slots = x.rc_body_slot_count(h);
    const mem = x.memory.buffer;
    return {
      flags: new Uint32Array(mem, x.rc_body_flags_ptr(h), slots).slice(),
      centre: new Float64Array(mem, x.rc_body_centre_ptr(h), slots * 3).slice(),
      vel: new Float64Array(mem, x.rc_body_vel_ptr(h), slots * 3).slice(),
      omega: new Float64Array(mem, x.rc_body_omega_ptr(h), slots * 3).slice(),
      rot: new Float64Array(mem, x.rc_body_rot_ptr(h), slots * 4).slice(),
      mass: new Float64Array(mem, x.rc_body_mass_ptr(h), slots).slice(),
      nodeBody: new Uint32Array(mem, x.rc_node_body_ptr(h), x.rc_node_count(h)).slice(),
      segmentBody: new Uint32Array(mem, x.rc_segment_body_ptr(h), x.rc_segment_count(h)).slice(),
    };
  }

  /** One segment's mass, centre and inertia now; `null` when it does not exist or has no mass. */
  segmentProps(h: HostPtr, segment: number): SegmentProps | null {
    const ptr = this.#x.rc_segment_props(h, segment);
    if (ptr === 0) return null;
    const v = new Float64Array(this.#x.memory.buffer, ptr, 10);
    return {
      mass: v[0]!,
      centre: [v[1]!, v[2]!, v[3]!],
      inertia: [v[4]!, v[5]!, v[6]!, v[7]!, v[8]!, v[9]!],
    };
  }
}
