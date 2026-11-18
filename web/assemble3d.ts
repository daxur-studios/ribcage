/**
 * Purpose: the assembly panel on the craft page: the form for a part's parameters, the list of free
 * rings and mount patches (also drawn as markers in the view, to click), the turn and remove buttons,
 * the refusal text, the root's place, and a demo that assembles the hopper one part at a time.
 * Value: the page shows and clicks; every decision (what fits, what a part is called, what the file
 * says) is made by `src/assembly-panel.ts`, which the Node tests drive with the same calls. Drawing
 * the craft is the host's job (it makes a paused world from the craft file's text), so a part
 * appears in the view the moment it is accepted, one after another. Drawing only reads.
 */

import { Mesh, MeshBasicMaterial, Raycaster, SphereGeometry, Vector2, Vector3 } from 'three/webgpu';
import { AssemblyPanel, defaultParams, PART_FIELDS, PART_TYPES, type AttachChoice, type CraftPartType, type PanelResult } from '../src/assembly-panel.ts';
import { MATERIALS } from '../src/parts/material.ts';
import type { Stage } from './stage3d.ts';

/** What the panel needs from the page. */
export interface BuildHost {
  stage: Stage;
  /** Draws the craft with this file text on the pad, paused, or nothing for `null`. Returns the reason it could not, or `null`. */
  show(text: string | null, keepView: boolean): string | null;
  /** Leaves building and flies this craft file text. */
  fly(text: string): void;
}

const $ = <T extends HTMLElement>(id: string): T => {
  const el = document.getElementById(id);
  if (!el) throw new Error(`missing #${id}`);
  return el as T;
};

const RING_RGB = 0x4aa3ff;
const PATCH_RGB = 0xffa23c;
const PICKED_RGB = 0xffe14a;
/** A click is a press and release this close together, pixels (further is an orbit drag). */
const CLICK_SLOP = 5;
/** Milliseconds between parts in the demo. */
const DEMO_GAP = 700;

/** The hopper's parts and joins, as the demo and the fixture have them. */
const HOPPER_STEPS: { type: CraftPartType; id: string; to?: AttachChoice }[] = [
  { type: 'tank', id: 'tank' },
  { type: 'pod', id: 'pod', to: { parent: 'tank', parentPort: 'top' } },
  { type: 'engine', id: 'engine', to: { parent: 'tank', parentPort: 'bottom' } },
  ...[0, 1, 2, 3].map((i) => ({ type: 'leg' as const, id: `leg${i}`, to: { parent: 'tank', parentPort: `side-0-${i * 2}` } })),
];

export interface Builder {
  readonly panel: AssemblyPanel;
  readonly isOpen: boolean;
  /** True while the view shows the craft being built (not a flight). */
  readonly building: boolean;
  open(): void;
  close(): void;
  /** Stops showing the build (markers off) but leaves the panel open: a flight has started. */
  leave(): void;
  /** Puts a craft file's text into the panel; the refusal reason when it does not load. */
  loadText(text: string): PanelResult;
  /** Where on the page a marker is, pixels from the canvas's top-left, or `null` when it is behind the camera or not there. */
  markerAt(name: string): [number, number] | null;
  /** Runs the hopper demo; resolves when its last part is on. */
  demo(): Promise<void>;
}

export function createBuilder(host: BuildHost): Builder {
  const { stage } = host;
  const panel = new AssemblyPanel();
  const stageEl = $('stage');
  const root = $('build-panel');
  const typeSel = $<HTMLSelectElement>('b-type');
  const idIn = $<HTMLInputElement>('b-id');
  const paramsEl = $('b-params');
  const targetSel = $<HTMLSelectElement>('b-target');
  const childSel = $<HTMLSelectElement>('b-child');
  const addBtn = $<HTMLButtonElement>('b-add');
  const turnMinus = $<HTMLButtonElement>('b-turn-minus');
  const turnPlus = $<HTMLButtonElement>('b-turn-plus');
  const removeBtn = $<HTMLButtonElement>('b-remove');
  const xIn = $<HTMLInputElement>('b-x');
  const yIn = $<HTMLInputElement>('b-y');
  const zIn = $<HTMLInputElement>('b-z');
  const yawIn = $<HTMLInputElement>('b-yaw');
  const standBox = $<HTMLInputElement>('b-stand');
  const msgEl = $('b-msg');
  const listEl = $('b-list');
  const flyBtn = $<HTMLButtonElement>('b-fly');
  const demoBtn = $<HTMLButtonElement>('b-demo');
  const clearBtn = $<HTMLButtonElement>('b-clear');
  const buildBtn = $<HTMLButtonElement>('build');
  const canvas = stage.canvas;

  const markers = new Map<string, Mesh>();
  let picked = '';
  let building = false;
  let demoRunning = false;
  let first = true;

  const type = (): CraftPartType => typeSel.value as CraftPartType;

  function say(text: string, refused = false): void {
    msgEl.textContent = text;
    msgEl.classList.toggle('refused', refused);
  }

  // ---- the parameter form ------------------------------------------------------------------

  function renderParams(): void {
    const values = defaultParams(type());
    paramsEl.replaceChildren();
    for (const f of PART_FIELDS[type()]) {
      const label = document.createElement('label');
      label.append(f.key + (f.optional ? ' (optional)' : ''));
      let input: HTMLInputElement | HTMLSelectElement;
      if (f.kind === 'number') {
        input = document.createElement('input');
        input.type = 'number';
        input.step = 'any';
        input.value = values[f.key] === undefined ? '' : String(values[f.key]);
      } else {
        input = document.createElement('select');
        for (const o of f.kind === 'material' ? Object.keys(MATERIALS) : ['square', 'triangle']) input.add(new Option(o, o));
        input.value = String(values[f.key] ?? input.options[0]!.value);
      }
      input.dataset.key = f.key;
      input.dataset.kind = f.kind;
      input.addEventListener('change', refreshChild);
      label.append(input);
      paramsEl.append(label);
    }
  }

  function readParams(): Record<string, number | string> {
    const out: Record<string, number | string> = {};
    for (const el of paramsEl.querySelectorAll<HTMLInputElement | HTMLSelectElement>('[data-key]')) {
      const key = el.dataset.key!;
      if (el.dataset.kind === 'number') {
        if (el.value.trim() !== '') out[key] = Number(el.value);
      } else out[key] = el.value;
    }
    return out;
  }

  // ---- targets, markers and the list --------------------------------------------------------

  function refreshTargets(prefer = ''): void {
    const ports = panel.freePorts();
    const keep = prefer || targetSel.value;
    targetSel.replaceChildren();
    if (panel.parts.length === 0) {
      targetSel.add(new Option('(this is the first part, the root)', ''));
      targetSel.disabled = true;
    } else {
      targetSel.disabled = false;
      for (const p of ports) targetSel.add(new Option(`${p.name}  (${p.kind}${p.kind === 'ring' ? `, ${p.nodes} nodes` : ''})`, p.name));
      if (ports.some((p) => p.name === keep)) targetSel.value = keep;
    }
    picked = targetSel.value;
    refreshChild();
    drawMarkers();
  }

  function refreshChild(): void {
    const ports = panel.freePorts();
    const target = ports.find((p) => p.name === targetSel.value);
    const keep = childSel.value;
    childSel.replaceChildren();
    if (!target) {
      childSel.add(new Option('—', ''));
      childSel.disabled = true;
      return;
    }
    childSel.disabled = false;
    for (const id of panel.portsFor(type(), target.kind, { ...defaultParams(type()), ...readParams() })) childSel.add(new Option(id, id));
    if (childSel.options.length === 0) childSel.add(new Option(`(a ${type()} has no ${target.kind})`, ''));
    else if ([...childSel.options].some((o) => o.value === keep)) childSel.value = keep;
  }

  function refreshList(): void {
    const lines = panel.parts.map((p) => {
      const j = panel.joins.find((x) => x.child === p.id);
      return j ? `${p.id} (${p.type}) → ${j.parent}.${j.parentPort}${j.clock ? `, turned ${j.clock}` : ''}` : `${p.id} (${p.type}) · root`;
    });
    const craft = panel.craft;
    listEl.textContent = lines.length === 0 ? 'no parts yet' : `${lines.join('\n')}\n${craft!.nodes.length} nodes, ${craft!.beams.length} beams`;
  }

  function syncPlace(): void {
    const p = panel.place;
    xIn.value = String(p.x);
    yIn.value = String(p.y);
    zIn.value = String(p.z);
    yawIn.value = String(p.yaw);
    standBox.checked = panel.autoStand;
    yIn.disabled = panel.autoStand;
  }

  function syncButtons(): void {
    const some = panel.parts.length > 0;
    const busy = demoRunning;
    addBtn.disabled = busy;
    turnMinus.disabled = busy || !some;
    turnPlus.disabled = busy || !some;
    removeBtn.disabled = busy || !some;
    flyBtn.disabled = busy || !some;
    demoBtn.disabled = busy;
    clearBtn.disabled = busy || !some;
  }

  const raycaster = new Raycaster();

  function clearMarkers(): void {
    for (const m of markers.values()) {
      stage.scene.remove(m);
      m.geometry.dispose();
      (m.material as MeshBasicMaterial).dispose();
    }
    markers.clear();
  }

  function drawMarkers(): void {
    clearMarkers();
    if (!building) return;
    for (const p of panel.freePorts()) {
      const on = p.name === picked;
      const m = new Mesh(new SphereGeometry(on ? 0.2 : p.kind === 'ring' ? 0.15 : 0.1, 14, 10), new MeshBasicMaterial({ color: on ? PICKED_RGB : p.kind === 'ring' ? RING_RGB : PATCH_RGB, depthTest: false, transparent: true, opacity: 0.9 }));
      m.position.set(p.centre[0], p.centre[1], p.centre[2]);
      m.renderOrder = 10;
      m.userData.port = p.name;
      stage.scene.add(m);
      markers.set(p.name, m);
    }
  }

  /** Redraws the craft in the view and everything that follows from it. */
  function redraw(): void {
    const err = host.show(panel.text(), !first);
    first = panel.parts.length === 0;
    building = true;
    stageEl.classList.add('building');
    if (err) say(err, true);
    refreshTargets();
    refreshList();
    syncPlace();
    syncButtons();
  }

  // ---- actions ------------------------------------------------------------------------------

  function attachChoice(): AttachChoice | undefined {
    if (panel.parts.length === 0) return undefined;
    const port = targetSel.value;
    const dot = port.indexOf('.');
    return { parent: port.slice(0, dot), parentPort: port.slice(dot + 1), ...(childSel.value ? { childPort: childSel.value } : {}), clock: 0 };
  }

  function add(): void {
    const r = panel.addPart(type(), readParams(), attachChoice(), idIn.value.trim());
    const id = panel.parts.at(-1)?.id ?? '';
    if (r.ok) idIn.value = '';
    redrawAfter(r, `Added ${id}.`);
  }

  /** After an action: the craft is redrawn only if it changed, and the message says what happened. */
  function redrawAfter(r: PanelResult, done: string): void {
    if (!r.ok) {
      say(`Refused: ${r.reason}`, true);
      refreshTargets();
      return;
    }
    say('');
    redraw();
    if (msgEl.textContent === '') say(done);
  }

  function placeFromInputs(): void {
    const r = panel.setPlace({ x: Number(xIn.value), y: Number(yIn.value), z: Number(zIn.value), yaw: Number(yawIn.value) }, standBox.checked);
    redrawAfter(r, 'Moved the root.');
    syncPlace();
  }

  async function demo(): Promise<void> {
    if (demoRunning) return;
    demoRunning = true;
    panel.clear();
    redraw();
    for (const [i, s] of HOPPER_STEPS.entries()) {
      const r = panel.addPart(s.type, undefined, s.to, s.id);
      redrawAfter(r, `Added ${s.id} (${i + 1} of ${HOPPER_STEPS.length}).`);
      if (!r.ok) break;
      await new Promise((resolve) => setTimeout(resolve, DEMO_GAP));
    }
    demoRunning = false;
    syncButtons();
  }

  // ---- clicking a marker --------------------------------------------------------------------

  const ndc = new Vector2();
  const projected = new Vector3();
  let down: { x: number; y: number } | null = null;

  canvas.addEventListener('pointerdown', (e) => {
    down = { x: e.clientX, y: e.clientY };
  });
  canvas.addEventListener('pointerup', (e) => {
    if (!building || !down) return;
    const moved = Math.hypot(e.clientX - down.x, e.clientY - down.y);
    down = null;
    if (moved > CLICK_SLOP) return;
    const r = canvas.getBoundingClientRect();
    ndc.set(((e.clientX - r.left) / r.width) * 2 - 1, -(((e.clientY - r.top) / r.height) * 2 - 1));
    raycaster.setFromCamera(ndc, stage.camera);
    const hit = raycaster.intersectObjects([...markers.values()], false)[0];
    if (!hit) return;
    const name = String(hit.object.userData.port);
    targetSel.value = name;
    picked = name;
    refreshChild();
    drawMarkers();
    say(`Attach to ${name}: pick a part and press Add part.`);
  });

  // ---- wiring -------------------------------------------------------------------------------

  for (const t of PART_TYPES) typeSel.add(new Option(t, t));
  typeSel.value = 'tank';
  renderParams();
  refreshTargets();
  refreshList();
  syncPlace();
  syncButtons();
  say('Pick a part and press Add part. The first one is the root; after that, pick where it goes.');

  typeSel.addEventListener('change', () => {
    renderParams();
    refreshChild();
  });
  targetSel.addEventListener('change', () => {
    picked = targetSel.value;
    refreshChild();
    drawMarkers();
  });
  addBtn.addEventListener('click', add);
  turnMinus.addEventListener('click', () => redrawAfter(panel.turn(-1), 'Turned one step back.'));
  turnPlus.addEventListener('click', () => redrawAfter(panel.turn(1), 'Turned one step on.'));
  removeBtn.addEventListener('click', () => {
    const id = panel.parts.at(-1)?.id ?? '';
    redrawAfter(panel.removeLast(), `Removed ${id}.`);
  });
  clearBtn.addEventListener('click', () => {
    panel.clear();
    redraw();
    say('Cleared.');
  });
  for (const el of [xIn, yIn, zIn, yawIn]) el.addEventListener('change', placeFromInputs);
  standBox.addEventListener('change', placeFromInputs);
  flyBtn.addEventListener('click', () => {
    const text = panel.text();
    if (text !== null) host.fly(text);
  });
  demoBtn.addEventListener('click', () => void demo());

  function leave(): void {
    building = false;
    stageEl.classList.remove('building');
    clearMarkers();
  }

  return {
    panel,
    get isOpen() {
      return !root.hidden;
    },
    get building() {
      return building;
    },
    open() {
      root.hidden = false;
      buildBtn.classList.add('on');
      first = true;
      redraw();
    },
    close() {
      root.hidden = true;
      buildBtn.classList.remove('on');
      leave();
    },
    leave,
    loadText(text: string): PanelResult {
      const r = panel.load(text);
      if (r.ok) {
        refreshTargets();
        refreshList();
        syncPlace();
        syncButtons();
        if (building) redraw();
      }
      return r;
    },
    markerAt(name: string): [number, number] | null {
      const m = markers.get(name);
      if (!m) return null;
      projected.copy(m.position).project(stage.camera);
      if (projected.z > 1) return null;
      const r = canvas.getBoundingClientRect();
      return [r.left + ((projected.x + 1) / 2) * r.width, r.top + ((1 - projected.y) / 2) * r.height];
    },
    demo,
  };
}
