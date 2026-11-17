/**
 * Purpose: loads the Rust core (WebAssembly) and exposes its C-style boundary as typed methods.
 * Value: one place that knows the ABI (core/src/ffi.rs), so the rest of the host never touches raw
 * exports. Runs in Node and in the browser: it takes the module's bytes, not a file path.
 *
 * Memory rule: WebAssembly memory can move when it grows, so a view is made fresh from
 * `memory.buffer` for every read and never kept across a call into the core.
 */

/** The ABI version this loader was written for (`ABI_VERSION` in core/src/ffi.rs). */
export const ABI_VERSION = 1;

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
}

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
}
