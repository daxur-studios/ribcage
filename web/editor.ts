/**
 * Purpose: the bridge editor page: place nodes and beams with the mouse, press run, then pause,
 * single-step and slow time down, with beams coloured by stress.
 * Value: the first place a person can see the engine run; and it drives the world only through
 * the same four calls the headless tests use (commands in, step, events and render data out), so
 * what the page shows is what the tests check.
 *
 * The editor edits a plain `Design`. Run builds a fresh world from it; Stop drops the world and
 * the design is still there, so nothing needs undoing. The scene is planar (z = 0). Drawing is
 * Canvas 2D: Three.js arrives with the 3D milestone.
 */

import { CommandBatch } from '../src/commands.ts';
import { BEAM_BROKEN, BEAM_YIELDED, Core, EVENT_BREAK, EVENT_YIELD } from '../src/core.ts';
import {
  compileDesign,
  deckOf,
  locateTruck,
  rollLoad,
  STANDARD_TRUCK,
  STEP_SECONDS,
  type Deck,
  type Design,
  type Material,
} from '../src/design.ts';
import { bridgeDesign, STRONG_BRIDGE, WEAK_BRIDGE } from '../src/bridge.ts';
import { decodeRecording, encodeRecording, Player, Recorder, type Recording } from '../src/recording.ts';
import { World, type Snapshot } from '../src/world.ts';

type Tool = 'node' | 'beam' | 'pin' | 'delete';

/** Snap spacing for placed nodes, metres. */
const GRID = 0.5;
/** Pick radius for nodes and beams, pixels. */
const PICK_PX = 11;
/** Most steps taken for one drawn frame; a slow machine slows the simulation down instead of spiralling. */
const MAX_STEPS_PER_FRAME = 8;

const MATERIALS: Record<string, Material> = { strong: STRONG_BRIDGE, weak: WEAK_BRIDGE };

const $ = <T extends HTMLElement>(id: string): T => {
  const el = document.getElementById(id);
  if (!el) throw new Error(`missing #${id}`);
  return el as T;
};

const canvas = $<HTMLCanvasElement>('view');
const ctx = canvas.getContext('2d')!;
const statusEl = $('status');
const hintEl = $('hint');
const runBtn = $<HTMLButtonElement>('run');
const pauseBtn = $<HTMLButtonElement>('pause');
const stepBtn = $<HTMLButtonElement>('step');
const speedSel = $<HTMLSelectElement>('speed');
const truckBox = $<HTMLInputElement>('truck');
const materialSel = $<HTMLSelectElement>('material');
const toolButtons = [...document.querySelectorAll<HTMLButtonElement>('[data-tool]')];
const saveBtn = $<HTMLButtonElement>('save');
const loadInput = $<HTMLInputElement>('load');
const sampleBtn = $<HTMLButtonElement>('sample');

interface Run {
  world: World;
  deck: Deck;
  /** A live run is recorded as it goes; a replay has a player instead. */
  recorder: Recorder | null;
  player: Player | null;
  /** What a replay is of (shown in the status line). */
  label: string;
  /** Steps in the recording being replayed (0 for a live run). */
  total: number;
  /** Steps taken so far (also the index of the next step). */
  step: number;
  playing: boolean;
  /** Fractional steps owed by the clock. */
  owed: number;
  yielded: number;
  broken: number;
  substeps: number;
  snap: Snapshot;
}

let core: Core;
let design: Design = bridgeDesign(STRONG_BRIDGE);
let material: Material = STRONG_BRIDGE;
let tool: Tool = 'node';
/** First node of a beam being drawn, or -1. */
let pending = -1;
let hover: { x: number; y: number } | null = null;
let run: Run | null = null;
/** The last live run that was stopped, kept so it can still be saved. */
let lastRecording: Recording | null = null;
let message = '';

// ---- view -----------------------------------------------------------------------------------

let cssW = 1;
let cssH = 1;

function resize(): void {
  const dpr = window.devicePixelRatio || 1;
  const r = canvas.getBoundingClientRect();
  cssW = Math.max(1, r.width);
  cssH = Math.max(1, r.height);
  canvas.width = Math.round(cssW * dpr);
  canvas.height = Math.round(cssH * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}

/** Pixels per metre, and the screen position of world (0, 0). The view is fixed: x 12 m is the centre. */
function viewParams(): { scale: number; ox: number; oy: number } {
  const scale = Math.min(cssW / 32, cssH / 18);
  return { scale, ox: cssW / 2 - 12 * scale, oy: cssH * 0.6 };
}

function toScreen(x: number, y: number): [number, number] {
  const v = viewParams();
  return [v.ox + x * v.scale, v.oy - y * v.scale];
}

function toWorld(px: number, py: number): { x: number; y: number } {
  const v = viewParams();
  return { x: (px - v.ox) / v.scale, y: (v.oy - py) / v.scale };
}

const snap = (v: number): number => Math.round(v / GRID) * GRID;

// ---- editing --------------------------------------------------------------------------------

function nodeAt(px: number, py: number): number {
  let best = -1;
  let bestD = PICK_PX * PICK_PX;
  design.nodes.forEach((n, i) => {
    const [sx, sy] = toScreen(n.x, n.y);
    const d = (sx - px) ** 2 + (sy - py) ** 2;
    if (d <= bestD) {
      bestD = d;
      best = i;
    }
  });
  return best;
}

function beamAt(px: number, py: number): number {
  let best = -1;
  let bestD = (PICK_PX * 0.6) ** 2;
  design.beams.forEach((b, i) => {
    const p = design.nodes[b.a]!;
    const q = design.nodes[b.b]!;
    const [ax, ay] = toScreen(p.x, p.y);
    const [bx, by] = toScreen(q.x, q.y);
    const dx = bx - ax;
    const dy = by - ay;
    const len2 = dx * dx + dy * dy;
    const t = len2 === 0 ? 0 : Math.max(0, Math.min(1, ((px - ax) * dx + (py - ay) * dy) / len2));
    const d = (ax + t * dx - px) ** 2 + (ay + t * dy - py) ** 2;
    if (d <= bestD) {
      bestD = d;
      best = i;
    }
  });
  return best;
}

function addNode(x: number, y: number): number {
  const found = design.nodes.findIndex((n) => n.x === x && n.y === y);
  if (found >= 0) return found;
  design.nodes.push({ x, y, pinned: false });
  return design.nodes.length - 1;
}

function addBeam(a: number, b: number): void {
  if (a === b) return;
  if (design.beams.some((s) => (s.a === a && s.b === b) || (s.a === b && s.b === a))) return;
  design.beams.push({ a, b });
}

function removeNode(k: number): void {
  design.nodes.splice(k, 1);
  design.beams = design.beams
    .filter((b) => b.a !== k && b.b !== k)
    .map((b) => ({ a: b.a > k ? b.a - 1 : b.a, b: b.b > k ? b.b - 1 : b.b }));
  pending = -1;
}

function pointerDown(ev: PointerEvent): void {
  if (run) {
    message = 'Stop the run to edit.';
    return;
  }
  const rect = canvas.getBoundingClientRect();
  const px = ev.clientX - rect.left;
  const py = ev.clientY - rect.top;
  const w = toWorld(px, py);
  const hit = nodeAt(px, py);
  message = '';
  switch (tool) {
    case 'node':
      if (hit < 0) addNode(snap(w.x), snap(w.y));
      break;
    case 'beam': {
      const k = hit >= 0 ? hit : addNode(snap(w.x), snap(w.y));
      if (pending < 0) pending = k;
      else {
        addBeam(pending, k);
        pending = -1;
      }
      break;
    }
    case 'pin':
      if (hit >= 0) design.nodes[hit]!.pinned = !design.nodes[hit]!.pinned;
      break;
    case 'delete': {
      if (hit >= 0) removeNode(hit);
      else {
        const b = beamAt(px, py);
        if (b >= 0) design.beams.splice(b, 1);
      }
      break;
    }
  }
}

function pointerMove(ev: PointerEvent): void {
  const rect = canvas.getBoundingClientRect();
  const w = toWorld(ev.clientX - rect.left, ev.clientY - rect.top);
  hover = { x: snap(w.x), y: snap(w.y) };
}

function setTool(t: Tool): void {
  tool = t;
  pending = -1;
  for (const b of toolButtons) b.classList.toggle('on', b.dataset.tool === t);
}

// ---- running --------------------------------------------------------------------------------

function startRun(): void {
  if (design.nodes.length === 0 || design.beams.length === 0) {
    message = 'Add nodes and beams first (or load the Bridge).';
    return;
  }
  stopRun();
  const world = new World(core);
  const batch = new CommandBatch();
  compileDesign(batch, design, material);
  const recorder = new Recorder(world);
  const result = recorder.setup(batch);
  if (result.rejected > 0) {
    message = `The core refused ${result.rejected} of the commands; the design was not started.`;
    world.free();
    return;
  }
  run = {
    world,
    deck: deckOf(design),
    recorder,
    player: null,
    label: '',
    total: 0,
    step: 0,
    playing: true,
    owed: 0,
    yielded: 0,
    broken: 0,
    substeps: 0,
    snap: world.snapshot(),
  };
  pending = -1;
  message = '';
  syncButtons();
}

/** Replays a recording on a fresh world, starting to play at once. */
function startReplay(rec: Recording, label: string): void {
  stopRun();
  const world = new World(core);
  const player = new Player(world, rec);
  const result = player.start();
  if (result.rejected > 0) {
    message = `The core refused ${result.rejected} of the recorded setup commands.`;
    world.free();
    return;
  }
  run = {
    world,
    deck: { ids: [], xs: [] },
    recorder: null,
    player,
    label,
    total: rec.steps,
    step: 0,
    playing: true,
    owed: 0,
    yielded: 0,
    broken: 0,
    substeps: 0,
    snap: world.snapshot(),
  };
  pending = -1;
  message = '';
  syncButtons();
}

function stopRun(): void {
  if (run?.recorder && run.recorder.steps > 0) lastRecording = run.recorder.finish();
  run?.world.free();
  run = null;
  syncButtons();
}

/** One fixed step: the truck's weight as commands, then the core's step, then its events. A replay takes its commands from the recording instead. */
function stepOnce(r: Run): void {
  if (r.player) {
    const s = r.player.advance();
    if (s < 0) return;
    r.substeps = s;
  } else {
    const batch = new CommandBatch();
    if (truckBox.checked) rollLoad(batch, r.deck, STANDARD_TRUCK, r.step);
    r.substeps = r.recorder!.step(batch);
  }
  for (const e of r.world.events()) {
    if (e.kind === EVENT_YIELD) r.yielded++;
    else if (e.kind === EVENT_BREAK) r.broken++;
  }
  r.step++;
  if (r.player?.done) {
    r.playing = false;
    syncButtons();
  }
}

function currentRecording(): Recording | null {
  if (run?.recorder) return run.recorder.steps > 0 ? run.recorder.finish() : null;
  return run ? null : lastRecording;
}

function saveRecording(): void {
  const rec = currentRecording();
  if (!rec) return;
  const blob = new Blob([encodeRecording(rec) as Uint8Array<ArrayBuffer>], { type: 'application/octet-stream' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = `ribcage-${rec.steps}-steps.rcrd`;
  a.click();
  URL.revokeObjectURL(a.href);
  message = `Saved ${rec.steps} steps (${blob.size} bytes).`;
}

function openRecording(bytes: Uint8Array, label: string): void {
  try {
    startReplay(decodeRecording(bytes), label);
  } catch (e) {
    message = `Could not open ${label}: ${e instanceof Error ? e.message : String(e)}`;
  }
}

async function replaySample(): Promise<void> {
  const res = await fetch('./weak-bridge.rcrd');
  if (!res.ok) {
    message = `could not load the sample recording (${res.status})`;
    return;
  }
  openRecording(new Uint8Array(await res.arrayBuffer()), 'sample (weak bridge)');
}

function syncSave(): void {
  saveBtn.disabled = currentRecording() === null;
}

function syncButtons(): void {
  const running = run !== null;
  runBtn.textContent = running ? 'Stop' : 'Run';
  pauseBtn.disabled = !running;
  pauseBtn.textContent = run && !run.playing ? 'Resume' : 'Pause';
  stepBtn.disabled = run === null || run.playing || (run.player?.done ?? false);
  truckBox.disabled = run?.player != null;
  for (const b of toolButtons) b.disabled = running;
  $<HTMLButtonElement>('preset-bridge').disabled = running;
  $<HTMLButtonElement>('clear').disabled = running;
  materialSel.disabled = running;
}

function togglePause(): void {
  if (!run) return;
  run.playing = !run.playing;
  run.owed = 0;
  syncButtons();
}

function singleStep(): void {
  if (!run || run.playing) return;
  stepOnce(run);
  run.snap = run.world.snapshot();
}

// ---- drawing --------------------------------------------------------------------------------

const GREY: [number, number, number] = [138, 147, 163];
const RED: [number, number, number] = [255, 93, 93];
const BLUE: [number, number, number] = [74, 163, 255];

function stressColour(ratio: number): string {
  const t = Math.min(1, Math.abs(ratio));
  const to = ratio >= 0 ? RED : BLUE;
  const c = GREY.map((g, i) => Math.round(g + (to[i]! - g) * t));
  return `rgb(${c[0]},${c[1]},${c[2]})`;
}

function drawGrid(): void {
  const v = viewParams();
  ctx.lineWidth = 1;
  ctx.strokeStyle = '#161b23';
  ctx.beginPath();
  for (let x = -4; x <= 28; x++) {
    ctx.moveTo(v.ox + x * v.scale, 0);
    ctx.lineTo(v.ox + x * v.scale, cssH);
  }
  for (let y = -10; y <= 10; y++) {
    ctx.moveTo(0, v.oy - y * v.scale);
    ctx.lineTo(cssW, v.oy - y * v.scale);
  }
  ctx.stroke();
  ctx.strokeStyle = '#273040';
  ctx.beginPath();
  ctx.moveTo(0, v.oy);
  ctx.lineTo(cssW, v.oy);
  ctx.stroke();
}

function drawNode(sx: number, sy: number, pinned: boolean, radiusPx: number, colour: string): void {
  if (pinned) {
    ctx.fillStyle = '#e6c15a';
    ctx.beginPath();
    ctx.moveTo(sx, sy - radiusPx);
    ctx.lineTo(sx + radiusPx * 1.3, sy + radiusPx);
    ctx.lineTo(sx - radiusPx * 1.3, sy + radiusPx);
    ctx.closePath();
    ctx.fill();
  } else {
    ctx.fillStyle = colour;
    ctx.beginPath();
    ctx.arc(sx, sy, radiusPx, 0, Math.PI * 2);
    ctx.fill();
  }
}

function draw(): void {
  ctx.clearRect(0, 0, cssW, cssH);
  ctx.fillStyle = '#0b0e13';
  ctx.fillRect(0, 0, cssW, cssH);
  drawGrid();
  const v = viewParams();
  const nodeR = Math.max(3.5, v.scale * 0.14);

  if (run) {
    const { positions, beams, nodeFlags } = run.snap;
    const at = (i: number): [number, number] => toScreen(positions[i * 3]!, positions[i * 3 + 1]!);
    for (let i = 0; i < beams.flags.length; i++) {
      const [ax, ay] = at(beams.ends[i * 2]!);
      const [bx, by] = at(beams.ends[i * 2 + 1]!);
      const f = beams.flags[i]!;
      ctx.beginPath();
      ctx.moveTo(ax, ay);
      ctx.lineTo(bx, by);
      if (f & BEAM_BROKEN) {
        ctx.setLineDash([3, 5]);
        ctx.strokeStyle = '#555c69';
        ctx.lineWidth = 1.5;
      } else {
        ctx.setLineDash([]);
        ctx.strokeStyle = f & BEAM_YIELDED ? '#d96bff' : stressColour(beams.ratio[i]!);
        ctx.lineWidth = 4;
      }
      ctx.stroke();
    }
    ctx.setLineDash([]);
    for (let i = 0; i < nodeFlags.length; i++) {
      if (!(nodeFlags[i]! & 1)) continue; // NODE_ALIVE
      const [sx, sy] = at(i);
      drawNode(sx, sy, (nodeFlags[i]! & 2) !== 0, nodeR, '#cfd6e2'); // NODE_PINNED
    }
    drawTruck(run, at);
  } else {
    for (const b of design.beams) {
      const p = design.nodes[b.a]!;
      const q = design.nodes[b.b]!;
      const [ax, ay] = toScreen(p.x, p.y);
      const [bx, by] = toScreen(q.x, q.y);
      ctx.beginPath();
      ctx.moveTo(ax, ay);
      ctx.lineTo(bx, by);
      ctx.strokeStyle = stressColour(0);
      ctx.lineWidth = 4;
      ctx.stroke();
    }
    design.nodes.forEach((n, i) => {
      const [sx, sy] = toScreen(n.x, n.y);
      drawNode(sx, sy, n.pinned, nodeR, i === pending ? '#4aa3ff' : '#cfd6e2');
    });
    if (pending >= 0 && hover) {
      const p = design.nodes[pending]!;
      const [ax, ay] = toScreen(p.x, p.y);
      const [bx, by] = toScreen(hover.x, hover.y);
      ctx.setLineDash([4, 4]);
      ctx.strokeStyle = '#4aa3ff';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(ax, ay);
      ctx.lineTo(bx, by);
      ctx.stroke();
      ctx.setLineDash([]);
    }
    if (hover && (tool === 'node' || tool === 'beam')) {
      const [hx, hy] = toScreen(hover.x, hover.y);
      ctx.strokeStyle = '#4aa3ff';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.arc(hx, hy, nodeR + 3, 0, Math.PI * 2);
      ctx.stroke();
    }
  }
}

/** The truck: a box riding on the deck between the two deck nodes its weight is split over. */
function drawTruck(r: Run, at: (i: number) => [number, number]): void {
  if (!truckBox.checked || r.player) return; // a replay carries the loads as forces, not as a design to draw them from
  const loc = locateTruck(r.deck, STANDARD_TRUCK, r.step);
  if (!loc) return;
  const [ax, ay] = at(r.deck.ids[loc.bay]!);
  const [bx, by] = at(r.deck.ids[loc.bay + 1]!);
  const x = ax + (bx - ax) * loc.t;
  const y = ay + (by - ay) * loc.t;
  const scale = viewParams().scale;
  const w = 3 * scale;
  const h = 1.2 * scale;
  ctx.fillStyle = '#e6a23c';
  ctx.fillRect(x - w / 2, y - h - 4, w, h);
  ctx.fillStyle = '#0b0e13';
  for (const dx of [-0.9, 0.9]) {
    ctx.beginPath();
    ctx.arc(x + dx * scale, y - 4, 0.3 * scale, 0, Math.PI * 2);
    ctx.fill();
  }
}

function hex(n: bigint): string {
  return '0x' + n.toString(16).padStart(16, '0');
}

function updateText(): void {
  if (run) {
    const secs = (run.step * STEP_SECONDS).toFixed(2);
    const p = run.player;
    const state = p ? (p.done ? 'replay finished' : run.playing ? 'replaying' : 'replay paused') : run.playing ? 'running' : 'paused';
    const of = p ? ` of ${run.total}` : '';
    const verdict = p
      ? p.mismatch
        ? `\nDIFFERS from the recording at step ${p.mismatch.step}: expected ${hex(p.mismatch.expected)}, got ${hex(p.mismatch.actual)}`
        : p.rejected > 0
          ? `\nthe core refused ${p.rejected} recorded commands`
          : p.done
            ? '\nidentical to the recording: every checkpoint and the final hash match'
            : '\nidentical to the recording so far'
      : '';
    statusEl.innerHTML =
      `<b>${state}</b>${p ? ` ${run.label}` : ''} · step ${run.step}${of} (${secs} s) · ${run.substeps} substeps · ` +
      `yielded ${run.yielded} · broken ${run.broken} · hash ${hex(run.world.stateHash)}` +
      verdict +
      (message ? `\n${message}` : '');
    hintEl.textContent = '';
  } else {
    const pinned = design.nodes.filter((n) => n.pinned).length;
    statusEl.innerHTML =
      `<b>editing</b> · ${design.nodes.length} nodes (${pinned} pinned) · ${design.beams.length} beams` +
      (message ? `\n${message}` : '');
    hintEl.textContent =
      tool === 'node'
        ? 'Node: click to place (snaps to 0.5 m). Pin the supports, join with beams, then Run.'
        : tool === 'beam'
          ? pending >= 0
            ? 'Beam: click the second node.'
            : 'Beam: click a node (or empty space) to start.'
          : tool === 'pin'
            ? 'Pin: click a node to pin or free it. The truck rolls along the nodes level with the leftmost pin.'
            : 'Delete: click a node or a beam.';
  }
}

// ---- frame loop -----------------------------------------------------------------------------

let last = performance.now();

function frame(now: number): void {
  const dt = Math.min(0.1, (now - last) / 1000);
  last = now;
  if (run?.playing) {
    run.owed += dt * Number(speedSel.value) * 60;
    const n = Math.min(MAX_STEPS_PER_FRAME, Math.floor(run.owed));
    run.owed = n === MAX_STEPS_PER_FRAME ? 0 : run.owed - n;
    for (let i = 0; i < n; i++) stepOnce(run);
    if (n > 0) run.snap = run.world.snapshot();
  }
  draw();
  updateText();
  syncSave();
  requestAnimationFrame(frame);
}

// ---- start ----------------------------------------------------------------------------------

function loadBridge(): void {
  design = bridgeDesign(STRONG_BRIDGE);
  pending = -1;
  message = '';
}

async function main(): Promise<void> {
  const res = await fetch('./beam_core.wasm');
  if (!res.ok) throw new Error(`could not load beam_core.wasm (${res.status})`);
  core = await Core.load(await res.arrayBuffer());

  for (const b of toolButtons) b.addEventListener('click', () => setTool(b.dataset.tool as Tool));
  $('preset-bridge').addEventListener('click', loadBridge);
  $('clear').addEventListener('click', () => {
    design = { nodes: [], beams: [] };
    pending = -1;
  });
  materialSel.addEventListener('change', () => {
    material = MATERIALS[materialSel.value] ?? STRONG_BRIDGE;
  });
  runBtn.addEventListener('click', () => (run ? stopRun() : startRun()));
  pauseBtn.addEventListener('click', togglePause);
  stepBtn.addEventListener('click', singleStep);
  saveBtn.addEventListener('click', saveRecording);
  sampleBtn.addEventListener('click', () => void replaySample());
  loadInput.addEventListener('change', () => {
    const file = loadInput.files?.[0];
    loadInput.value = '';
    if (file) void file.arrayBuffer().then((b) => openRecording(new Uint8Array(b), file.name));
  });
  canvas.addEventListener('pointerdown', pointerDown);
  canvas.addEventListener('pointermove', pointerMove);
  canvas.addEventListener('pointerleave', () => (hover = null));
  window.addEventListener('resize', resize);
  window.addEventListener('keydown', (e) => {
    if (e.target instanceof HTMLSelectElement) return;
    const tools: Tool[] = ['node', 'beam', 'pin', 'delete'];
    if (e.key >= '1' && e.key <= '4' && !run) setTool(tools[Number(e.key) - 1]!);
    else if (e.key === ' ') {
      e.preventDefault();
      if (run) togglePause();
      else startRun();
    } else if (e.key === '.') singleStep();
    else if (e.key === 'Escape') {
      if (run) stopRun();
      else pending = -1;
    }
  });

  resize();
  setTool('node');
  syncButtons();
  requestAnimationFrame(frame);
}

main().catch((e: unknown) => {
  statusEl.textContent = `could not start: ${String(e)}`;
  console.error(e);
});
