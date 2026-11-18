/**
 * Purpose: the craft file. A craft is saved as the parts that make it (type and generator
 * parameters), the joins between them and where the root part stands, never as nodes and beams, so
 * a file is small, readable, and builds the same craft on every machine.
 * Value: one text format that the editor, the tests and later milestones share. Saving writes one
 * canonical text (fixed key order, JavaScript's shortest round-trip numbers, two-space indent, one
 * trailing newline), so load-then-save gives the same text and a loaded craft compiles to the same
 * command bytes as the one assembled in code. Anything the file does not say exactly is refused
 * with a reason: an unknown field or part type, a later version, a bad number, a bad join.
 *
 * Layout (version 1), in this key order: `format` ("ribcage-craft"), `version` (1), `root` (a part
 * id), `place` ({x, y, z, yaw}: metres and whole 15° steps), `parts` (each {id, type, params}) and
 * `joins` (each {child, childPort, parent, parentPort, clock}). The order of parts and joins is
 * the order of nodes in the craft, so it is kept. No damage layer yet: wrecks are not saved.
 */

import { AssemblyError, assemble, type Assembly, type AssemblyJoin, type Craft, type RootPlacement } from './assemble.ts';
import { engine, type EngineParams } from './engine.ts';
import { leg, type LegParams } from './leg.ts';
import { MATERIALS } from './material.ts';
import type { Part } from './part.ts';
import { pod, type PodParams } from './pod.ts';
import { tank, type TankParams } from './tank.ts';
import { truss, type TrussParams } from './truss.ts';

export const CRAFT_FORMAT = 'ribcage-craft';
export const CRAFT_VERSION = 1;

/** Why a craft file is refused. */
export type CraftFileRefusal =
  | 'syntax'
  | 'format'
  | 'version'
  | 'shape'
  | 'unknown-field'
  | 'missing-field'
  | 'bad-value'
  | 'unknown-part-type'
  | 'bad-part'
  | 'bad-join';

export class CraftFileError extends Error {
  readonly code: CraftFileRefusal;
  constructor(code: CraftFileRefusal, message: string, options?: ErrorOptions) {
    super(message, options);
    this.name = 'CraftFileError';
    this.code = code;
  }
}

export type CraftPartSpec =
  | { id: string; type: 'tank'; params: TankParams }
  | { id: string; type: 'truss'; params: TrussParams }
  | { id: string; type: 'leg'; params: LegParams }
  | { id: string; type: 'engine'; params: EngineParams }
  | { id: string; type: 'pod'; params: PodParams };

export type CraftPartType = CraftPartSpec['type'];

/** What a craft file says: parts, joins and the root's place. */
export interface CraftFile {
  root: string;
  place: RootPlacement;
  parts: CraftPartSpec[];
  joins: AssemblyJoin[];
}

type Kind = 'number' | 'material' | 'section';
/** One parameter of a part type: its key, what kind of value it takes, and whether it may be left out. */
export interface Field {
  key: string;
  kind: Kind;
  optional?: true;
}

const num = (key: string, optional?: true): Field => (optional === undefined ? { key, kind: 'number' } : { key, kind: 'number', optional });
const material: Field = { key: 'material', kind: 'material' };

/** The parameters of each part type, in the order they are written. */
const SCHEMAS: Readonly<Record<CraftPartType, readonly Field[]>> = {
  tank: [num('radius'), num('length'), num('ringNodes'), num('rings'), material, num('wallArea'), num('mass'), num('nodeRadius', true)],
  truss: [{ key: 'section', kind: 'section' }, num('length'), num('width'), num('bays'), material, num('memberArea'), num('mass', true), num('nodeRadius', true)],
  leg: [
    num('length'),
    num('footOffset'),
    num('footSize'),
    num('mountWidth'),
    num('mountHeight'),
    material,
    num('memberArea'),
    num('mass'),
    num('sprungStiffness'),
    num('sprungStroke'),
    num('crushForce'),
    num('crushStroke'),
    num('bodyWidth', true),
    num('nodeRadius', true),
  ],
  engine: [num('ringNodes'), num('topRadius'), num('nozzleRadius'), num('height'), material, num('memberArea'), num('mass'), num('maxThrust'), num('nodeRadius', true)],
  pod: [num('ringNodes'), num('bottomRadius'), num('topRadius'), num('height'), num('apexRise', true), material, num('memberArea'), num('mass'), num('nodeRadius', true)],
};

/** The parameter fields of each part type, in the order a file writes them (the assembly panel builds its form from these). */
export const PART_FIELDS = SCHEMAS;
export const PART_TYPES = Object.keys(SCHEMAS) as CraftPartType[];

type Raw = Record<string, unknown>;

function isRaw(v: unknown): v is Raw {
  return typeof v === 'object' && v !== null && !Array.isArray(v);
}

function describe(v: unknown): string {
  if (typeof v === 'number') return String(v);
  if (typeof v === 'string') return JSON.stringify(v);
  if (Array.isArray(v)) return 'an array';
  if (v === null) return 'null';
  return typeof v === 'object' ? 'an object' : String(v);
}

/** Checks that `obj` has exactly the keys in `keys` (the ones in `optional` may be left out). */
function exactKeys(obj: unknown, keys: readonly string[], optional: readonly string[], where: string): Raw {
  if (!isRaw(obj)) throw new CraftFileError('shape', `${where} must be an object, not ${describe(obj)}`);
  for (const k of Object.keys(obj)) {
    if (!keys.includes(k)) throw new CraftFileError('unknown-field', `${where} has a field "${k}" that this version does not know`);
  }
  for (const k of keys) {
    if (obj[k] === undefined && !optional.includes(k)) throw new CraftFileError('missing-field', `${where} is missing "${k}"`);
  }
  return obj;
}

function finite(v: unknown, where: string): number {
  if (typeof v !== 'number' || !Number.isFinite(v)) throw new CraftFileError('bad-value', `${where} must be a finite number, not ${describe(v)}`);
  return v;
}

function text(v: unknown, where: string): string {
  if (typeof v !== 'string' || v.length === 0) throw new CraftFileError('bad-value', `${where} must be a non-empty text, not ${describe(v)}`);
  return v;
}

function readParams(type: CraftPartType, raw: unknown, where: string): Raw {
  const schema = SCHEMAS[type];
  const obj = exactKeys(raw, schema.map((f) => f.key), schema.filter((f) => f.optional).map((f) => f.key), where);
  const out: Raw = {};
  for (const f of schema) {
    const v = obj[f.key];
    if (v === undefined) continue;
    const at = `${where}.${f.key}`;
    if (f.kind === 'number') out[f.key] = finite(v, at);
    else if (f.kind === 'material') {
      if (typeof v !== 'string' || !Object.hasOwn(MATERIALS, v)) throw new CraftFileError('bad-value', `${at} must be one of ${Object.keys(MATERIALS).join(', ')}, not ${describe(v)}`);
      out[f.key] = v;
    } else {
      if (v !== 'square' && v !== 'triangle') throw new CraftFileError('bad-value', `${at} must be "square" or "triangle", not ${describe(v)}`);
      out[f.key] = v;
    }
  }
  return out;
}

/** Checks a whole file's shape and values (not whether the parts build or join) and returns it in canonical order. */
function readFile(raw: unknown): CraftFile {
  if (!isRaw(raw)) throw new CraftFileError('shape', `a craft file is an object, not ${describe(raw)}`);
  if (raw.format !== CRAFT_FORMAT) throw new CraftFileError('format', `the file's format is ${describe(raw.format)}, not "${CRAFT_FORMAT}"`);
  const v = raw.version;
  if (typeof v !== 'number' || !Number.isInteger(v) || v < 1) throw new CraftFileError('version', `the file's version is ${describe(v)}, not a whole number from 1`);
  if (v > CRAFT_VERSION) throw new CraftFileError('version', `the file is version ${v}, and this build reads versions up to ${CRAFT_VERSION}`);
  const top = exactKeys(raw, ['format', 'version', 'root', 'place', 'parts', 'joins'], [], 'the craft file');

  const root = text(top.root, 'root');
  const place = exactKeys(top.place, ['x', 'y', 'z', 'yaw'], [], 'place');
  const placed: RootPlacement = { x: finite(place.x, 'place.x'), y: finite(place.y, 'place.y'), z: finite(place.z, 'place.z'), yaw: finite(place.yaw, 'place.yaw') };

  if (!Array.isArray(top.parts)) throw new CraftFileError('shape', `parts must be a list, not ${describe(top.parts)}`);
  const parts = (top.parts as unknown[]).map((p, i): CraftPartSpec => {
    const where = `parts[${i}]`;
    const o = exactKeys(p, ['id', 'type', 'params'], [], where);
    const id = text(o.id, `${where}.id`);
    const type = o.type;
    if (typeof type !== 'string' || !PART_TYPES.includes(type as CraftPartType)) {
      throw new CraftFileError('unknown-part-type', `${where} (${id}) has the part type ${describe(type)}; the types are ${PART_TYPES.join(', ')}`);
    }
    const params = readParams(type as CraftPartType, o.params, `${where}.params`);
    return { id, type, params } as unknown as CraftPartSpec;
  });

  if (!Array.isArray(top.joins)) throw new CraftFileError('shape', `joins must be a list, not ${describe(top.joins)}`);
  const joins = (top.joins as unknown[]).map((j, i): AssemblyJoin => {
    const where = `joins[${i}]`;
    const o = exactKeys(j, ['child', 'childPort', 'parent', 'parentPort', 'clock'], [], where);
    return {
      child: text(o.child, `${where}.child`),
      childPort: text(o.childPort, `${where}.childPort`),
      parent: text(o.parent, `${where}.parent`),
      parentPort: text(o.parentPort, `${where}.parentPort`),
      clock: finite(o.clock, `${where}.clock`),
    };
  });

  return { root, place: placed, parts, joins };
}

/** Reads the text of a craft file and checks it. Refuses with the reason; builds nothing. */
export function parseCraft(source: string): CraftFile {
  let raw: unknown;
  try {
    raw = JSON.parse(source);
  } catch (e) {
    throw new CraftFileError('syntax', `the file is not valid JSON: ${e instanceof Error ? e.message : String(e)}`, { cause: e });
  }
  return readFile(raw);
}

/** The canonical text of a craft file. Checks the file first, so it never writes what it would refuse to read. */
export function saveCraft(file: CraftFile): string {
  const c = readFile({ format: CRAFT_FORMAT, version: CRAFT_VERSION, root: file.root, place: file.place, parts: file.parts, joins: file.joins });
  const out = { format: CRAFT_FORMAT, version: CRAFT_VERSION, root: c.root, place: c.place, parts: c.parts, joins: c.joins };
  return `${JSON.stringify(out, null, 2)}\n`;
}

function buildPart(spec: CraftPartSpec): Part {
  switch (spec.type) {
    case 'tank':
      return tank(spec.params);
    case 'truss':
      return truss(spec.params);
    case 'leg':
      return leg(spec.params);
    case 'engine':
      return engine(spec.params);
    case 'pod':
      return pod(spec.params);
  }
}

/** Builds the parts of a file with their generators. A generator that refuses its parameters refuses the file. */
export function buildAssembly(file: CraftFile): Assembly {
  const parts = file.parts.map((spec) => {
    try {
      return { id: spec.id, part: buildPart(spec) };
    } catch (e) {
      if (e instanceof Error) throw new CraftFileError('bad-part', `part ${spec.id} (${spec.type}): ${e.message}`, { cause: e });
      throw e;
    }
  });
  return { parts, root: file.root, joins: file.joins.map((j) => ({ ...j })), place: { ...file.place } };
}

export interface LoadedCraft {
  file: CraftFile;
  assembly: Assembly;
  craft: Craft;
}

/** Reads, checks, builds and assembles a craft file. Every refusal is a `CraftFileError` with its reason. */
export function loadCraft(source: string): LoadedCraft {
  const file = parseCraft(source);
  const assembly = buildAssembly(file);
  try {
    return { file, assembly, craft: assemble(assembly) };
  } catch (e) {
    if (e instanceof AssemblyError) throw new CraftFileError('bad-join', e.message, { cause: e });
    throw e;
  }
}
