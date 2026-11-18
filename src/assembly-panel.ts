/**
 * Purpose: the logic of the craft page's assembly panel, with no page in it: pick a part type and
 * its parameters, attach it to a free ring or mount patch of the craft so far, turn the last part by
 * clock steps, remove the last part, and save or load the craft as a craft file.
 * Value: the panel's actions are plain calls that a Node test makes, so "the panel gives the same craft
 * file as building the hopper in code" is a test, and the page only has to show what this says. Every
 * action is checked by building the whole craft again; one that fails is not applied, and the reason
 * (the assembler's or the generator's own words) is given back to be shown.
 *
 * The craft is always held as a craft file (`CraftFile`), the same list of parts and joins the
 * loader reads, so what the panel saves is what `loadCraft` loads. The first part added is the root;
 * each later part is joined to a free port by one of its own. A leg's mount size is read off the
 * patch it is put on (the same distances a test measures), since a leg that does not fit is refused.
 */

import { GALLERY_PARTS } from './craft-page.ts';
import { PAD, prepareDrop } from './landing.ts';
import { AssemblyError, type AssemblyJoin, type RootPlacement } from './parts/assemble.ts';
import {
  CraftFileError,
  loadCraft,
  parseCraft,
  PART_FIELDS,
  PART_TYPES,
  saveCraft,
  type CraftFile,
  type CraftPartSpec,
  type CraftPartType,
  type LoadedCraft,
} from './parts/craft-file.ts';
import type { Part, V3 } from './parts/part.ts';

export { PART_FIELDS, PART_TYPES };
export type { CraftPartType };

/** Whether an action was applied, and if not, why not (the words to show). */
export type PanelResult = { ok: true } | { ok: false; reason: string };

/** A ring or patch of the craft that nothing is attached to yet. */
export interface FreePort {
  /** `<part id>.<port id>`, as the craft names it. */
  name: string;
  part: string;
  port: string;
  kind: 'ring' | 'patch';
  /** Where it is in the craft, metres (the ring's centre, or the middle of the patch's four nodes). */
  centre: V3;
  /** Nodes in the ring (4 for a patch). */
  nodes: number;
}

/** Where a new part goes: a free port of the craft, and which port of the new part meets it. */
export interface AttachChoice {
  parent: string;
  parentPort: string;
  /** The new part's port; left out, the first one of the right kind. */
  childPort?: string;
  /** Whole ring steps the new part is turned by; left out, 0. */
  clock?: number;
}

const EMPTY_PLACE: RootPlacement = { x: 0, y: 0, z: 0, yaw: 0 };

/** The default parameters of a part type (the hopper's), as a fresh copy. */
export function defaultParams(type: CraftPartType): Record<string, number | string> {
  const spec = GALLERY_PARTS.find((p) => p.type === type);
  if (!spec) throw new Error(`there is no part type "${type}" (one of ${PART_TYPES.join(', ')})`);
  return { ...(spec.params as unknown as Record<string, number | string>) };
}

/** The id a new part of this type gets when none is given: the type's name, then `type1`, `type2`, … */
export function defaultId(type: CraftPartType, taken: readonly string[]): string {
  if (!taken.includes(type)) return type;
  for (let n = 1; ; n++) if (!taken.includes(`${type}${n}`)) return `${type}${n}`;
}

function reasonOf(e: unknown): string {
  if (e instanceof CraftFileError || e instanceof AssemblyError || e instanceof Error) return e.message;
  return String(e);
}

const round6 = (v: number): number => Math.round(v * 1e6) / 1e6;

function patchCentre(part: LoadedCraft['craft'], nodes: readonly number[]): V3 {
  let x = 0;
  let y = 0;
  let z = 0;
  for (const i of nodes) {
    const n = part.nodes[i]!;
    x += n.x;
    y += n.y;
    z += n.z;
  }
  return [x / nodes.length, y / nodes.length, z / nodes.length];
}

export class AssemblyPanel {
  /** When true (the default), the root is set to stand on the pad after every change, as the hopper's file is. */
  autoStand = true;
  #file: CraftFile = { root: '', place: { ...EMPTY_PLACE }, parts: [], joins: [] };
  #loaded: LoadedCraft | null = null;

  /** The parts so far, in the order they were added. */
  get parts(): readonly CraftPartSpec[] {
    return this.#file.parts;
  }

  get joins(): readonly AssemblyJoin[] {
    return this.#file.joins;
  }

  get place(): RootPlacement {
    return { ...this.#file.place };
  }

  /** The craft as built, or `null` while there are no parts. */
  get craft(): LoadedCraft['craft'] | null {
    return this.#loaded?.craft ?? null;
  }

  /** The craft file as it stands, or `null` while there are no parts. */
  file(): CraftFile | null {
    return this.#file.parts.length === 0 ? null : structuredClone(this.#file);
  }

  /** The craft file's text, exactly as Save writes it, or `null` while there are no parts. */
  text(): string | null {
    return this.#file.parts.length === 0 ? null : saveCraft(this.#file);
  }

  /** The id the next part of this type would get. */
  nextId(type: CraftPartType): string {
    return defaultId(type, this.#file.parts.map((p) => p.id));
  }

  /** Rings and patches nothing is attached to, in craft order. */
  freePorts(): FreePort[] {
    const craft = this.craft;
    if (!craft) return [];
    const out: FreePort[] = [];
    for (const r of craft.rings) {
      const dot = r.id.indexOf('.');
      out.push({ name: r.id, part: r.id.slice(0, dot), port: r.id.slice(dot + 1), kind: 'ring', centre: [...r.centre] as V3, nodes: r.nodes.length });
    }
    for (const p of craft.patches) {
      const dot = p.id.indexOf('.');
      out.push({ name: p.id, part: p.id.slice(0, dot), port: p.id.slice(dot + 1), kind: 'patch', centre: patchCentre(craft, p.nodes), nodes: p.nodes.length });
    }
    return out;
  }

  /** The ports of a part type's own that can meet a free port of this kind (its ring ids, or its patch ids). */
  portsFor(type: CraftPartType, kind: 'ring' | 'patch', params: Record<string, number | string> = defaultParams(type)): string[] {
    // A part's port ids do not depend on its parameters, so when these do not build, the defaults name them.
    const part = this.#generate({ id: 'probe', type, params } as unknown as CraftPartSpec) ?? this.#generate({ id: 'probe', type, params: defaultParams(type) } as unknown as CraftPartSpec);
    if (!part) return [];
    return (kind === 'ring' ? part.rings : part.patches).map((p) => p.id);
  }

  #generate(spec: CraftPartSpec): Part | null {
    try {
      const probe: CraftFile = { root: spec.id, place: { ...EMPTY_PLACE }, parts: [spec], joins: [] };
      return loadCraft(saveCraft(probe)).assembly.parts[0]!.part;
    } catch {
      return null;
    }
  }

  /** Checks and builds a whole file; on success it is the panel's state (with the root standing on the pad when `autoStand`). */
  #commit(file: CraftFile): PanelResult {
    if (file.parts.length === 0) {
      this.#file = { root: '', place: { ...file.place }, parts: [], joins: [] };
      this.#loaded = null;
      return { ok: true };
    }
    try {
      let next = file;
      let loaded = loadCraft(saveCraft(next));
      if (this.autoStand) {
        const y = standHeight(loaded, next.place);
        if (y !== next.place.y) {
          next = { ...next, place: { ...next.place, y } };
          loaded = loadCraft(saveCraft(next));
        }
      }
      this.#file = next;
      this.#loaded = loaded;
      return { ok: true };
    } catch (e) {
      return { ok: false, reason: reasonOf(e) };
    }
  }

  /**
   * Adds a part. The first part is the root and takes no `attach`; every later one needs it. `params`
   * left out are the type's defaults; `id` left out is `nextId(type)`. A part that does not fit is
   * refused with the reason and the craft is as it was.
   */
  addPart(type: CraftPartType, params?: Record<string, number | string>, attach?: AttachChoice, id?: string): PanelResult {
    if (!PART_TYPES.includes(type)) return { ok: false, reason: `there is no part type "${type}" (one of ${PART_TYPES.join(', ')})` };
    const first = this.#file.parts.length === 0;
    if (!first && !attach) return { ok: false, reason: 'pick a free ring or patch to attach it to' };
    const partId = id !== undefined && id !== '' ? id : this.nextId(type);
    if (this.#file.parts.some((p) => p.id === partId)) return { ok: false, reason: `there is already a part called "${partId}"` };
    const spec = { id: partId, type, params: { ...(params ?? defaultParams(type)) } } as unknown as CraftPartSpec;
    if (first) return this.#commit({ root: partId, place: this.#file.place, parts: [spec], joins: [] });

    const parent = this.freePorts().find((p) => p.part === attach!.parent && p.port === attach!.parentPort);
    if (!parent) return { ok: false, reason: `${attach!.parent}.${attach!.parentPort} is not a free ring or patch of this craft` };
    if (parent.kind === 'patch' && type === 'leg') this.#fitMount(spec, attach!);
    const childPort = attach!.childPort ?? this.portsFor(type, parent.kind, spec.params as unknown as Record<string, number | string>)[0];
    if (childPort === undefined) return { ok: false, reason: `a ${type} has no ${parent.kind} to put on ${parent.name}` };
    const join: AssemblyJoin = { child: partId, childPort, parent: attach!.parent, parentPort: attach!.parentPort, clock: parent.kind === 'ring' ? (attach!.clock ?? 0) : 0 };
    return this.#commit({ ...this.#file, parts: [...this.#file.parts, spec], joins: [...this.#file.joins, join] });
  }

  /** Sets a leg's mount to the size of the patch it goes on: the distances along the patch's two edges, in the parent part's own frame. */
  #fitMount(spec: CraftPartSpec, attach: AttachChoice): void {
    const parent = this.#loaded?.assembly.parts.find((p) => p.id === attach.parent)?.part;
    const patch = parent?.patches.find((p) => p.id === attach.parentPort);
    if (!parent || !patch) return;
    const [a, b, c] = patch.nodes.map((i) => parent.nodes[i]!) as [Part['nodes'][number], Part['nodes'][number], Part['nodes'][number]];
    const dist = (p: typeof a, q: typeof a): number => Math.sqrt((p.x - q.x) ** 2 + (p.y - q.y) ** 2 + (p.z - q.z) ** 2);
    const params = spec.params as unknown as Record<string, number>;
    params.mountWidth = dist(a, b);
    params.mountHeight = dist(a, c);
  }

  /**
   * Turns the last part by `steps` whole ring steps (negative turns the other way): its clock, or, for
   * the root, its yaw in 15° steps. A patch does not turn. A turn that does not fit is refused.
   */
  turn(steps: number): PanelResult {
    if (!Number.isInteger(steps)) return { ok: false, reason: `a turn is a whole number of steps, not ${steps}` };
    const f = this.#file;
    if (f.parts.length === 0) return { ok: false, reason: 'there is no part to turn yet' };
    const joins = f.joins.slice();
    const last = joins[joins.length - 1];
    if (!last) return this.#commit({ ...f, place: { ...f.place, yaw: f.place.yaw + steps } });
    const ring = this.#loaded?.assembly.parts.find((p) => p.id === last.child)?.part.rings.find((r) => r.id === last.childPort);
    if (!ring) return { ok: false, reason: `${last.child} is on a mount patch, which has one way round only, so it does not turn` };
    const n = ring.nodes.length;
    joins[joins.length - 1] = { ...last, clock: (((last.clock + steps) % n) + n) % n };
    return this.#commit({ ...f, joins });
  }

  /** Removes the last part and its join (the root, when it is the only part). */
  removeLast(): PanelResult {
    const f = this.#file;
    if (f.parts.length === 0) return { ok: false, reason: 'there is nothing to remove' };
    const parts = f.parts.slice(0, -1);
    return this.#commit({ ...f, root: parts.length === 0 ? '' : f.root, parts, joins: parts.length === 0 ? [] : f.joins.slice(0, -1) });
  }

  /** Sets where the root stands (x, y, z in metres, yaw in whole 15° steps) and turns height-following off, as typing a height means it. */
  setPlace(place: RootPlacement, keepStanding = false): PanelResult {
    this.autoStand = keepStanding;
    return this.#commit({ ...this.#file, place: { ...place } });
  }

  /** Empties the panel. */
  clear(): void {
    this.#commit({ root: '', place: { ...EMPTY_PLACE }, parts: [], joins: [] });
    this.autoStand = true;
  }

  /** Replaces the craft with a craft file's text (the file's own place is kept as written). A file that does not load changes nothing. */
  load(source: string): PanelResult {
    let file: CraftFile;
    try {
      file = parseCraft(source);
      loadCraft(source);
    } catch (e) {
      return { ok: false, reason: reasonOf(e) };
    }
    const before = this.autoStand;
    this.autoStand = false;
    const r = this.#commit(file);
    this.autoStand = before;
    return r;
  }
}

/** How far above the pad the lowest point starts, metres: the 4 mm the hopper's file has, so nothing begins inside the ground. */
export const STAND_GAP = 0.004;

/** Height of the root that stands the craft on the pad (lowest point `STAND_GAP` above it), to six decimal places: the hopper's is 2.339576. */
export function standHeight(loaded: LoadedCraft, place: RootPlacement): number {
  const text = saveCraft(loaded.file);
  const dropped = prepareDrop(text, { ...PAD, x: place.x, z: place.z, yaw: place.yaw }, 0);
  return round6(dropped.loaded.file.place.y + STAND_GAP);
}
