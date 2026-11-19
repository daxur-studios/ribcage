/**
 * Purpose: record a run and replay it. A recording is the command list applied before each step
 * (layout rule 8) plus hash checkpoints; replaying feeds the same lists to a fresh world and
 * checks that every checkpoint and the final hash come out the same.
 * Value: the proof that the same inputs always give the same result, in a file you can keep: a
 * run saved today still replays on a later build, in Node or in a browser, or the first
 * checkpoint that differs says where determinism broke.
 *
 * File layout (little endian, `FORMAT_VERSION` 1):
 *   0   "RCRD"
 *   4   u32 format version | 8 u32 ABI version | 12 u32 bytes per command record
 *   16  u32 steps | 20 u32 checkpoint interval | 24 u32 checkpoint count | 28 u32 reserved (0)
 *   32  u64 final state hash (after the last step)
 *   40  u64 checkpoint hashes: the hash after step `interval`, `2 * interval`, ...
 *   then `steps + 1` lists: list 0 is applied to the empty world before step 0, list `k` before
 *   step `k` (0-based). Each list is u32 record count, then that many command records.
 */

import { CommandBatch, COMMAND_BYTES } from './commands.ts';
import { ABI_VERSION, type ApplyResult } from './core.ts';
import { World } from './world.ts';

export const FORMAT_VERSION = 1;
/** The oldest core ABI whose recordings still replay (2: before the terrain commands; 3 to 10 also read). */
export const MIN_READABLE_ABI = 2;
const MAGIC = [0x52, 0x43, 0x52, 0x44]; // "RCRD"
const HEADER_BYTES = 40;

/** Hash checkpoints are taken every this many steps (one simulated second). */
export const DEFAULT_CHECKPOINT_EVERY = 60;

export interface Recording {
  /** Steps taken. */
  steps: number;
  /** Steps between checkpoints. */
  interval: number;
  /** `steps + 1` lists of command bytes; list 0 is the setup before the first step. */
  lists: Uint8Array[];
  /** `floor(steps / interval)` hashes. */
  checkpoints: bigint[];
  /** The state hash after the last step. */
  finalHash: bigint;
}

/** Where a replay first disagreed with the recording. */
export interface Mismatch {
  /** The step count at which the hashes differed. */
  step: number;
  expected: bigint;
  actual: bigint;
}

/** Drives a world and keeps what it was given: the same four calls the editor and tests use. */
export class Recorder {
  readonly #world: World;
  readonly #interval: number;
  readonly #lists: Uint8Array[] = [];
  readonly #checkpoints: bigint[] = [];
  #steps = 0;

  constructor(world: World, interval = DEFAULT_CHECKPOINT_EVERY) {
    if (!Number.isInteger(interval) || interval < 1) throw new Error('checkpoint interval must be a whole number of steps');
    this.#world = world;
    this.#interval = interval;
  }

  /** Applies the build commands and records them as list 0. Call once, before the first step. */
  setup(batch: CommandBatch): ApplyResult {
    if (this.#lists.length > 0) throw new Error('setup must come first, once');
    const result = batch.count > 0 ? this.#world.apply(batch) : { results: new Uint32Array(0), rejected: 0 };
    this.#lists.push(batch.bytes.slice());
    return result;
  }

  /** Applies `batch` (if any), takes one step, records the list. Returns the substeps used. */
  step(batch?: CommandBatch): number {
    if (this.#lists.length === 0) this.#lists.push(new Uint8Array(0));
    if (batch !== undefined && batch.count > 0) this.#world.apply(batch);
    const substeps = this.#world.step();
    this.#lists.push(batch === undefined ? new Uint8Array(0) : batch.bytes.slice());
    this.#steps++;
    if (this.#steps % this.#interval === 0) this.#checkpoints.push(this.#world.stateHash);
    return substeps;
  }

  get steps(): number {
    return this.#steps;
  }

  /** The recording so far (a copy; the run can go on). */
  finish(): Recording {
    const lists = this.#lists.length === 0 ? [new Uint8Array(0)] : this.#lists.map((l) => l.slice());
    return {
      steps: this.#steps,
      interval: this.#interval,
      lists,
      checkpoints: [...this.#checkpoints],
      finalHash: this.#world.stateHash,
    };
  }
}

/** Feeds a recording to a world one step at a time, checking each hash it has a record of. */
export class Player {
  readonly #world: World;
  readonly #rec: Recording;
  #at = 0;
  #started = false;
  #mismatch: Mismatch | null = null;
  #rejected = 0;

  constructor(world: World, rec: Recording) {
    this.#world = world;
    this.#rec = rec;
  }

  /** Applies list 0, the setup. */
  start(): ApplyResult {
    if (this.#started) throw new Error('already started');
    this.#started = true;
    const first = this.#rec.lists[0]!;
    if (first.byteLength === 0) return { results: new Uint32Array(0), rejected: 0 };
    const r = this.#world.apply(CommandBatch.fromBytes(first));
    this.#rejected += r.rejected;
    return r;
  }

  /** Steps taken so far. */
  get step(): number {
    return this.#at;
  }

  get done(): boolean {
    return this.#at >= this.#rec.steps;
  }

  /** The first hash that differed, or `null` (so far). */
  get mismatch(): Mismatch | null {
    return this.#mismatch;
  }

  /** Records the core refused while replaying (a recording of a healthy run has none). */
  get rejected(): number {
    return this.#rejected;
  }

  /** Applies the next list and steps once; returns the substeps, or -1 when there is nothing left. */
  advance(): number {
    if (!this.#started) throw new Error('start first');
    if (this.done) return -1;
    const bytes = this.#rec.lists[this.#at + 1]!;
    if (bytes.byteLength > 0) this.#rejected += this.#world.apply(CommandBatch.fromBytes(bytes)).rejected;
    const substeps = this.#world.step();
    this.#at++;
    const k = this.#at;
    if (k % this.#rec.interval === 0) this.#compare(k, this.#rec.checkpoints[k / this.#rec.interval - 1]!);
    if (k === this.#rec.steps) this.#compare(k, this.#rec.finalHash);
    return substeps;
  }

  #compare(step: number, expected: bigint): void {
    const actual = this.#world.stateHash;
    if (actual !== expected && this.#mismatch === null) this.#mismatch = { step, expected, actual };
  }
}

export interface ReplayResult {
  steps: number;
  hash: bigint;
  /** True when every checkpoint and the final hash matched and no record was refused. */
  ok: boolean;
  mismatch: Mismatch | null;
  rejected: number;
}

/** Replays a whole recording on a fresh world (made by `makeWorld`) and frees it. */
export function replayRecording(makeWorld: () => World, rec: Recording): ReplayResult {
  const world = makeWorld();
  try {
    const p = new Player(world, rec);
    p.start();
    while (!p.done) p.advance();
    return { steps: p.step, hash: world.stateHash, ok: p.mismatch === null && p.rejected === 0, mismatch: p.mismatch, rejected: p.rejected };
  } finally {
    world.free();
  }
}

export function encodeRecording(rec: Recording): Uint8Array {
  if (rec.lists.length !== rec.steps + 1) throw new Error('a recording needs steps + 1 command lists');
  if (rec.checkpoints.length !== Math.floor(rec.steps / rec.interval)) throw new Error('wrong number of checkpoints');
  let size = HEADER_BYTES + rec.checkpoints.length * 8;
  for (const l of rec.lists) {
    if (l.byteLength % COMMAND_BYTES !== 0) throw new Error('a command list is not whole records');
    size += 4 + l.byteLength;
  }
  const out = new Uint8Array(size);
  const v = new DataView(out.buffer);
  out.set(MAGIC, 0);
  v.setUint32(4, FORMAT_VERSION, true);
  v.setUint32(8, ABI_VERSION, true);
  v.setUint32(12, COMMAND_BYTES, true);
  v.setUint32(16, rec.steps, true);
  v.setUint32(20, rec.interval, true);
  v.setUint32(24, rec.checkpoints.length, true);
  v.setUint32(28, 0, true);
  v.setBigUint64(32, rec.finalHash, true);
  let o = HEADER_BYTES;
  for (const c of rec.checkpoints) {
    v.setBigUint64(o, c, true);
    o += 8;
  }
  for (const l of rec.lists) {
    v.setUint32(o, l.byteLength / COMMAND_BYTES, true);
    out.set(l, o + 4);
    o += 4 + l.byteLength;
  }
  return out;
}

/** Reads a recording back. Throws, saying why, on anything that is not exactly a recording this build understands. */
export function decodeRecording(bytes: Uint8Array): Recording {
  if (bytes.byteLength < HEADER_BYTES) throw new Error('not a recording: too short');
  if (!MAGIC.every((m, i) => bytes[i] === m)) throw new Error('not a recording: wrong first bytes');
  const v = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const format = v.getUint32(4, true);
  if (format !== FORMAT_VERSION) throw new Error(`recording format ${format}, this build reads ${FORMAT_VERSION}`);
  const abi = v.getUint32(8, true);
  // ABI 3 to 7 only added command kinds (the terrain, the thrusters, the segments, the rigid
  // bodies, the joints) and, for the last four, hash sections that exist only once a thruster,
  // segment, body or joint does, so every older list means the same thing in them. ABI 8 added
  // only the stress estimate's readers, which no command or hash touches. ABI 9 added the
  // sleeping command (kind 17) and a hash section that exists only once it has run. ABI 10 added
  // the break command (kind 18), two event kinds and a hash section that exists only once a split
  // has happened. An older list still means the same commands; only its end hash can move, and only
  // where a beam broke in a world with segments (the rigid hop, re-recorded on purpose in 5.10).
  if (abi < MIN_READABLE_ABI || abi > ABI_VERSION) {
    throw new Error(`recorded with core ABI ${abi}, this build reads ABI ${MIN_READABLE_ABI} to ${ABI_VERSION}`);
  }
  const recBytes = v.getUint32(12, true);
  if (recBytes !== COMMAND_BYTES) throw new Error(`recorded with ${recBytes}-byte records, this build uses ${COMMAND_BYTES}`);
  const steps = v.getUint32(16, true);
  const interval = v.getUint32(20, true);
  const cps = v.getUint32(24, true);
  if (interval < 1 || cps !== Math.floor(steps / interval)) throw new Error('recording header is inconsistent');
  if (v.getUint32(28, true) !== 0) throw new Error('recording header is inconsistent');
  const finalHash = v.getBigUint64(32, true);
  let o = HEADER_BYTES;
  if (o + cps * 8 > bytes.byteLength) throw new Error('recording is truncated');
  const checkpoints: bigint[] = [];
  for (let i = 0; i < cps; i++, o += 8) checkpoints.push(v.getBigUint64(o, true));
  const lists: Uint8Array[] = [];
  for (let i = 0; i <= steps; i++) {
    if (o + 4 > bytes.byteLength) throw new Error('recording is truncated');
    const n = v.getUint32(o, true);
    o += 4;
    if (n > (bytes.byteLength - o) / COMMAND_BYTES) throw new Error('recording is truncated');
    lists.push(bytes.slice(o, o + n * COMMAND_BYTES));
    o += n * COMMAND_BYTES;
  }
  if (o !== bytes.byteLength) throw new Error('recording has extra bytes at the end');
  return { steps, interval, lists, checkpoints, finalHash };
}
