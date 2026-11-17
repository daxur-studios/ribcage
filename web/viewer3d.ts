/**
 * Purpose: the 3D page: the tower on its plain and the bridge over its valley, drawn with Three.js
 * from the core's render buffers, with an orbit camera, a scene picker, Run and Stop.
 * Value: the first place the milestone 3 scenes can be seen; it drives the world only through the
 * same calls as the headless tests (commands in, step, render data out), so what it shows is what
 * the tests check.
 *
 * Drawing only reads. The core steps at a fixed 60 Hz from the page's clock and never sees a frame
 * time. `?scene=bridge&steps=300` starts that scene paused after exactly that many steps, so the
 * hash on screen can be compared with the one Node computes for the same steps.
 */

import {
  AmbientLight,
  BufferAttribute,
  BoxGeometry,
  BufferGeometry,
  Color,
  CylinderGeometry,
  DirectionalLight,
  DynamicDrawUsage,
  HemisphereLight,
  InstancedMesh,
  Matrix4,
  Mesh,
  MeshStandardMaterial,
  PerspectiveCamera,
  Quaternion,
  Scene,
  SphereGeometry,
  Vector3,
  WebGPURenderer,
} from 'three/webgpu';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { CommandBatch } from '../src/commands.ts';
import { BEAM_ALIVE, BEAM_BROKEN, BEAM_YIELDED, Core } from '../src/core.ts';
import { locateTruck, STANDARD_TRUCK, STEP_SECONDS, type Truck } from '../src/design.ts';
import { lerpPositions, newPlacement, pickNode, placeBeam, terrainTriangles } from '../src/render3d.ts';
import { decodeRecording, encodeRecording, Player, Recorder, type Recording } from '../src/recording.ts';
import {
  ACTION_START_STEP,
  buildBridge3,
  buildTower,
  HEAVY_TRUCK,
  identifyScene,
  liveBridgeCommands,
  liveTowerCommands,
  MEMBER,
  STANDARD_BRIDGE3,
  STANDARD_TOWER,
  type ActionKind,
  type ArmedAction,
  type BridgeLayout3,
  type TowerLayout,
} from '../src/scenes3d.ts';
import { stressRgb, YIELDED_RGB } from '../src/stress-colour.ts';
import type { Terrain } from '../src/terrain.ts';
import { World, type Snapshot } from '../src/world.ts';

type SceneName = 'tower' | 'bridge';

/** Radius of a drawn beam, metres. */
const BEAM_DRAW_RADIUS = 0.07;
/** Most steps taken for one drawn frame; a slow machine slows the simulation down instead of spiralling. */
const MAX_STEPS_PER_FRAME = 8;

const $ = <T extends HTMLElement>(id: string): T => {
  const el = document.getElementById(id);
  if (!el) throw new Error(`missing #${id}`);
  return el as T;
};

const canvas = $<HTMLCanvasElement>('view');
const statusEl = $('status');
const sceneSel = $<HTMLSelectElement>('scene');
const runBtn = $<HTMLButtonElement>('run');
const stopBtn = $<HTMLButtonElement>('stop');
const nodesBox = $<HTMLInputElement>('nodes');
const resetViewBtn = $<HTMLButtonElement>('reset-view');
const pauseBtn = $<HTMLButtonElement>('pause');
const stepBtn = $<HTMLButtonElement>('step');
const speedSel = $<HTMLSelectElement>('speed');
const knockBtn = $<HTMLButtonElement>('knock');
const removeBtn = $<HTMLButtonElement>('remove');
const overloadBtn = $<HTMLButtonElement>('overload');
const saveBtn = $<HTMLButtonElement>('save');
const loadInput = $<HTMLInputElement>('load');
const sampleBtn = $<HTMLButtonElement>('sample');

/** Radius of the sphere a click picks a node with, metres (the drawn node is smaller and hard to hit). */
const PICK_RADIUS = 0.5;

interface Run {
  world: World;
  scene: SceneName;
  layout: TowerLayout | BridgeLayout3;
  step: number;
  playing: boolean;
  /** Fractional steps owed by the clock. */
  owed: number;
  /** Stop playing after this many steps (the `?steps=` hook), or -1. */
  until: number;
  substeps: number;
  /** The render buffer after the last step, and the one before it (drawing blends the two). */
  snap: Snapshot;
  prev: Float32Array;
  /** How far between `prev` and `snap` the drawing is, 0 to 1. */
  alpha: number;
  /** What is drawn now: `prev` and `snap` blended. */
  drawn: Float32Array;
  /** The action that was started on this run, if any. */
  armed: ArmedAction | null;
  /** A live run is recorded as it goes; a replay has a player instead. */
  recorder: Recorder | null;
  player: Player | null;
  /** What a replay is of (shown in the status line), and the steps it has. */
  label: string;
  total: number;
  /** Steps taken in the current one-second window, and the rate measured over the last one. */
  windowSteps: number;
  windowStart: number;
  rate: number;
}

// ---- Three.js scene ------------------------------------------------------------------------

// WebGPU where the browser has it, WebGL 2 otherwise; `?webgl=1` forces the fallback for checking it.
const renderer = new WebGPURenderer({ canvas, antialias: true, forceWebGL: new URLSearchParams(location.search).get('webgl') === '1' });
const scene = new Scene();
scene.background = new Color(0x0b0e13);
const camera = new PerspectiveCamera(45, 1, 0.1, 400);
const controls = new OrbitControls(camera, canvas);
controls.enableDamping = true;

scene.add(new AmbientLight(0xffffff, 0.25));
scene.add(new HemisphereLight(0xcfe0ff, 0x2a2f3a, 1.4));
const sun = new DirectionalLight(0xffffff, 2.4);
sun.position.set(18, 30, 12);
scene.add(sun);

/** Where the camera starts for each scene: position and look-at target. */
const VIEWS: Record<SceneName, { from: [number, number, number]; to: [number, number, number] }> = {
  tower: { from: [22, 14, 26], to: [0, 8, 0] },
  bridge: { from: [34, 14, 30], to: [12, -1, 0] },
};

let terrainMesh: Mesh | null = null;
let beamMesh: InstancedMesh | null = null;
let nodeMesh: InstancedMesh | null = null;

/** The bridge's truck: a box on the deck (the heavy one is bigger and red). Drawing only. */
const truckMesh = new Mesh(new BoxGeometry(1, 1, 1), new MeshStandardMaterial({ color: 0xe8b24a, roughness: 0.6 }));
truckMesh.visible = false;
scene.add(truckMesh);
const TRUCK_SIZE = { light: [2.4, 1.1, 1.5], heavy: [4.2, 1.9, 2.2] } as const;
/** The node a knock would hit, shown while aiming. */
const aimMesh = new Mesh(new SphereGeometry(PICK_RADIUS, 16, 12), new MeshStandardMaterial({ color: 0xff9a3c, transparent: true, opacity: 0.55, roughness: 0.4 }));
aimMesh.visible = false;
scene.add(aimMesh);
/** True while the next click on a node knocks it. */
let aiming = false;
let core: Core;
let run: Run | null = null;
let message = '';
/** The last live run's recording, kept after Stop so it can still be saved. */
let lastRecording: Recording | null = null;
let rendererName = '';

function disposeMesh(m: Mesh | null): void {
  if (!m) return;
  scene.remove(m);
  m.geometry.dispose();
}

/** The ground, from the same triangles the core uses. */
function showTerrain(t: Terrain): void {
  disposeMesh(terrainMesh);
  const { positions, indices } = terrainTriangles(t);
  const g = new BufferGeometry();
  g.setAttribute('position', new BufferAttribute(positions, 3));
  g.setIndex(new BufferAttribute(indices, 1));
  // Vertex colours: lower ground darker, so a valley reads as a valley.
  const colours = new Float32Array(t.nx * t.nz * 3);
  for (let i = 0; i < t.nx * t.nz; i++) {
    const h = positions[i * 3 + 1]!;
    const k = Math.max(0, Math.min(1, (h + 5) / 6));
    colours[i * 3] = 0.16 + 0.2 * k;
    colours[i * 3 + 1] = 0.2 + 0.22 * k;
    colours[i * 3 + 2] = 0.17 + 0.14 * k;
  }
  g.setAttribute('color', new BufferAttribute(colours, 3));
  g.computeVertexNormals();
  terrainMesh = new Mesh(g, new MeshStandardMaterial({ vertexColors: true, flatShading: true, roughness: 1, metalness: 0 }));
  scene.add(terrainMesh);
}

/** One instanced cylinder per beam and one sphere per node, sized for this world. */
function makeInstances(nodes: number, beams: number): void {
  disposeMesh(beamMesh);
  disposeMesh(nodeMesh);
  const cyl = new CylinderGeometry(BEAM_DRAW_RADIUS, BEAM_DRAW_RADIUS, 1, 8, 1);
  beamMesh = new InstancedMesh(cyl, new MeshStandardMaterial({ roughness: 0.55, metalness: 0.1 }), beams);
  beamMesh.instanceMatrix.setUsage(DynamicDrawUsage);
  beamMesh.frustumCulled = false;
  beamMesh.setColorAt(0, new Color(1, 1, 1)); // makes the colour buffer
  scene.add(beamMesh);
  nodeMesh = new InstancedMesh(new SphereGeometry(MEMBER.radius, 12, 8), new MeshStandardMaterial({ color: 0xcfd6e2, roughness: 0.6 }), nodes);
  nodeMesh.instanceMatrix.setUsage(DynamicDrawUsage);
  nodeMesh.frustumCulled = false;
  nodeMesh.visible = nodesBox.checked;
  scene.add(nodeMesh);
}

const place = newPlacement();
const tmpM = new Matrix4();
const tmpP = new Vector3();
const tmpQ = new Quaternion();
const tmpS = new Vector3();
const tmpC = new Color();
const HIDDEN = new Matrix4().makeScale(0, 0, 0);

/** Copies a snapshot's beams and flags, with node positions `positions` (blended between steps), into the instance buffers. */
function drawSnapshot(s: Snapshot, positions: Float32Array): void {
  if (!beamMesh || !nodeMesh) return;
  const { beams, nodeFlags } = s;
  const n = beams.flags.length;
  for (let i = 0; i < n; i++) {
    const f = beams.flags[i]!;
    if (!(f & BEAM_ALIVE) || f & BEAM_BROKEN) {
      beamMesh.setMatrixAt(i, HIDDEN);
      continue;
    }
    placeBeam(positions, beams.ends[i * 2]!, beams.ends[i * 2 + 1]!, place);
    tmpP.set(place.x, place.y, place.z);
    tmpQ.set(place.qx, place.qy, place.qz, place.qw);
    tmpS.set(1, place.length, 1);
    beamMesh.setMatrixAt(i, tmpM.compose(tmpP, tmpQ, tmpS));
    const [r, g, b] = f & BEAM_YIELDED ? YIELDED_RGB : stressRgb(beams.ratio[i]!);
    beamMesh.setColorAt(i, tmpC.setRGB(r / 255, g / 255, b / 255));
  }
  beamMesh.instanceMatrix.needsUpdate = true;
  if (beamMesh.instanceColor) beamMesh.instanceColor.needsUpdate = true;
  if (nodeMesh.visible) {
    for (let i = 0; i < nodeFlags.length; i++) {
      if (!(nodeFlags[i]! & 1)) {
        nodeMesh.setMatrixAt(i, HIDDEN);
        continue;
      }
      tmpM.makeTranslation(positions[i * 3]!, positions[i * 3 + 1]!, positions[i * 3 + 2]!);
      nodeMesh.setMatrixAt(i, tmpM);
    }
    nodeMesh.instanceMatrix.needsUpdate = true;
  }
}

/** Draws the run's current picture: the two last render buffers blended, the truck, the aim marker. */
function drawRun(r: Run): void {
  let positions = r.snap.positions;
  if (r.playing && r.alpha < 1) {
    lerpPositions(r.prev, r.snap.positions, r.alpha, r.drawn);
    positions = r.drawn;
  }
  drawSnapshot(r.snap, positions);
  drawTruck(r, positions);
}

/** The truck sits on the deck at the time the drawing is at (the last applied step, plus how far into the next). */
function drawTruck(r: Run, positions: Float32Array): void {
  truckMesh.visible = false;
  // A replay carries the truck as forces in its commands; which truck it was is not in the file.
  if (r.scene !== 'bridge' || r.player) return;
  const layout = r.layout as BridgeLayout3;
  const heavy = r.armed?.kind === 'overload' && r.step > r.armed.start;
  const truck: Truck = heavy ? HEAVY_TRUCK : STANDARD_TRUCK;
  const t0 = r.step - 1 + (r.playing ? r.alpha : 1) - (heavy ? r.armed!.start : 0);
  if (t0 < 0) return;
  const at = locateTruck(layout.rail, truck, t0);
  if (at === null) return;
  let x = 0;
  let y = 0;
  let z = 0;
  for (let s = 0; s < 2; s++) {
    const a = layout.deck(s, at.bay) * 3;
    const b = layout.deck(s, at.bay + 1) * 3;
    x += (positions[a]! * (1 - at.t) + positions[b]! * at.t) / 2;
    y += (positions[a + 1]! * (1 - at.t) + positions[b + 1]! * at.t) / 2;
    z += (positions[a + 2]! * (1 - at.t) + positions[b + 2]! * at.t) / 2;
  }
  const [sx, sy, sz] = heavy ? TRUCK_SIZE.heavy : TRUCK_SIZE.light;
  truckMesh.scale.set(sx, sy, sz);
  truckMesh.position.set(x, y + MEMBER.radius + sy / 2, z);
  (truckMesh.material as MeshStandardMaterial).color.setHex(heavy ? 0xd9523e : 0xe8b24a);
  truckMesh.visible = true;
}

function resetView(): void {
  const v = VIEWS[(run?.scene ?? (sceneSel.value as SceneName))];
  camera.position.set(...v.from);
  controls.target.set(...v.to);
  controls.update();
}

function resize(): void {
  const r = canvas.getBoundingClientRect();
  const w = Math.max(1, Math.round(r.width));
  const h = Math.max(1, Math.round(r.height));
  renderer.setPixelRatio(window.devicePixelRatio || 1);
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}

// ---- running -------------------------------------------------------------------------------

function startRun(name: SceneName, until = -1, action: ActionKind | null = null): void {
  stopRun();
  setAiming(false);
  const world = new World(core);
  const batch = new CommandBatch();
  const layout = name === 'tower' ? buildTower(batch, STANDARD_TOWER) : buildBridge3(batch, STANDARD_BRIDGE3);
  const recorder = new Recorder(world);
  const result = recorder.setup(batch);
  if (result.rejected > 0) {
    message = `The core refused ${result.rejected} of the commands; the scene was not started.`;
    world.free();
    return;
  }
  showTerrain(layout.terrain);
  makeInstances(layout.nodeCount, layout.beamCount);
  const snap = world.snapshot();
  run = {
    world,
    scene: name,
    layout,
    step: 0,
    playing: until !== 0,
    owed: 0,
    until,
    substeps: 0,
    snap,
    prev: snap.positions,
    alpha: 1,
    drawn: new Float32Array(snap.positions.length),
    // `?action=` starts an action at the standard step, which is exactly the headless action.
    armed: action === null ? null : { kind: action, start: ACTION_START_STEP },
    recorder,
    player: null,
    label: '',
    total: 0,
    windowSteps: 0,
    windowStart: performance.now(),
    rate: 0,
  };
  message = '';
  drawRun(run);
  resetView();
  syncButtons();
}

/** Replays a recording on a fresh world of the scene it was made on, starting to play at once (or paused after `until` steps). */
function startReplay(rec: Recording, label: string, until = -1): void {
  const found = identifyScene(rec.lists[0]!);
  if (found === null) {
    message = `Could not open ${label}: its setup is not the standard tower or bridge, so there is nothing to draw it on.`;
    return;
  }
  stopRun();
  setAiming(false);
  const world = new World(core);
  const player = new Player(world, rec);
  const result = player.start();
  if (result.rejected > 0) {
    message = `The core refused ${result.rejected} of the recorded setup commands.`;
    world.free();
    return;
  }
  const { scene: name, layout } = found;
  sceneSel.value = name;
  showTerrain(layout.terrain);
  makeInstances(layout.nodeCount, layout.beamCount);
  const snap = world.snapshot();
  run = {
    world,
    scene: name,
    layout,
    step: 0,
    playing: until !== 0,
    owed: 0,
    until,
    substeps: 0,
    snap,
    prev: snap.positions,
    alpha: 1,
    drawn: new Float32Array(snap.positions.length),
    armed: null,
    recorder: null,
    player,
    label,
    total: rec.steps,
    windowSteps: 0,
    windowStart: performance.now(),
    rate: 0,
  };
  message = '';
  drawRun(run);
  resetView();
  syncButtons();
}

function stopRun(): void {
  if (run?.recorder && run.recorder.steps > 0) lastRecording = run.recorder.finish();
  run?.world.free();
  run = null;
  syncButtons();
}

/** One fixed step: this scene's commands (the bridge's truck, an armed action), then the core's step. */
function stepOnce(r: Run): void {
  if (r.player) {
    const s = r.player.advance();
    if (s < 0) return;
    r.substeps = s;
  } else {
    const batch = new CommandBatch();
    if (r.scene === 'bridge') liveBridgeCommands(batch, r.layout as BridgeLayout3, r.armed, r.step, STANDARD_TRUCK);
    else liveTowerCommands(batch, r.layout as TowerLayout, r.armed, r.step);
    r.substeps = r.recorder!.step(batch);
  }
  r.step++;
  r.windowSteps++;
}

/** True once a replay has fed the last list; it then holds still. */
function replayDone(r: Run): boolean {
  return r.player?.done ?? false;
}

/** Takes `n` steps and refreshes the two buffers drawing blends; `prev` is the picture before the last step. */
function advance(r: Run, n: number): void {
  if (n <= 0) return;
  for (let i = 0; i < n; i++) {
    if (i === n - 1) r.prev = i === 0 ? r.snap.positions : r.world.snapshot().positions;
    stepOnce(r);
  }
  r.snap = r.world.snapshot();
}

function setPaused(paused: boolean): void {
  if (!run) return;
  if (!paused && replayDone(run)) return; // nothing left to play
  run.playing = !paused;
  run.owed = 0;
  run.until = -1;
  run.alpha = 1;
  drawRun(run);
  syncButtons();
}

function singleStep(): void {
  if (!run || run.playing || replayDone(run)) return;
  run.until = -1;
  advance(run, 1);
  run.alpha = 1;
  drawRun(run);
}

/** Starts an action on the run, at the step it is on. */
function arm(kind: ActionKind, aim?: ArmedAction['aim']): void {
  if (!run || run.armed) return;
  run.armed = aim ? { kind, start: run.step, aim } : { kind, start: run.step };
  message = `${kind === 'knock' ? 'knock' : kind === 'removeColumn' ? 'column removed' : 'overload'} at step ${run.step}`;
  setAiming(false);
  syncButtons();
}

function setAiming(on: boolean): void {
  aiming = on && run !== null && run.armed === null;
  aimMesh.visible = false;
  knockBtn.textContent = aiming ? 'Knock: click a node' : 'Knock…';
  canvas.style.cursor = aiming ? 'crosshair' : '';
}

/** What Save would write: the live run so far, or the last one after Stop; never a replay. */
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
  a.download = `ribcage-3d-${rec.steps}-steps.rcrd`;
  a.click();
  URL.revokeObjectURL(a.href);
  message = `Saved ${rec.steps} steps (${blob.size} bytes).`;
}

function openRecording(bytes: Uint8Array, label: string, until = -1): void {
  try {
    startReplay(decodeRecording(bytes), label, until);
  } catch (e) {
    message = `Could not open ${label}: ${e instanceof Error ? e.message : String(e)}`;
  }
}

async function replaySample(until = -1): Promise<void> {
  const res = await fetch('./tower-collapse.rcrd');
  if (!res.ok) {
    message = `could not load the sample recording (${res.status})`;
    return;
  }
  openRecording(new Uint8Array(await res.arrayBuffer()), 'sample: tower collapse', until);
}

/** Save needs steps to save; checked without copying the recording. */
function canSave(): boolean {
  return run ? run.recorder !== null && run.recorder.steps > 0 : lastRecording !== null;
}

function syncButtons(): void {
  runBtn.textContent = run ? 'Restart' : 'Run';
  stopBtn.disabled = run === null;
  pauseBtn.disabled = run === null;
  pauseBtn.textContent = run && !run.playing ? 'Resume' : 'Pause';
  stepBtn.disabled = run === null || run.playing || replayDone(run);
  const free = run !== null && run.armed === null && run.player === null;
  knockBtn.disabled = !free;
  removeBtn.disabled = !free;
  overloadBtn.disabled = !free;
}

// ---- picking -------------------------------------------------------------------------------

const ndc = new Vector3();
const rayOrigin = new Vector3();
const rayDir = new Vector3();

/** The node under the pointer (client coordinates), or -1. */
function nodeAt(clientX: number, clientY: number): number {
  if (!run) return -1;
  const r = canvas.getBoundingClientRect();
  ndc.set(((clientX - r.left) / r.width) * 2 - 1, -((clientY - r.top) / r.height) * 2 + 1, 0.5);
  camera.updateMatrixWorld();
  ndc.unproject(camera);
  rayOrigin.copy(camera.position);
  rayDir.copy(ndc).sub(rayOrigin).normalize();
  const positions = run.playing && run.alpha < 1 ? run.drawn : run.snap.positions;
  return pickNode(positions, run.snap.nodeFlags, PICK_RADIUS, rayOrigin.x, rayOrigin.y, rayOrigin.z, rayDir.x, rayDir.y, rayDir.z);
}

/** Knocks the node that was clicked, away from the camera along the ground. */
function knockNode(node: number): void {
  const look = camera.getWorldDirection(rayDir);
  const len = Math.hypot(look.x, look.z);
  // Straight down or up has no ground direction: fall back to +x.
  const dx = len > 1e-6 ? look.x / len : 1;
  const dz = len > 1e-6 ? look.z / len : 0;
  arm('knock', { node, dx, dz });
}

let downAt: { x: number; y: number } | null = null;

function onPointerDown(e: PointerEvent): void {
  downAt = { x: e.clientX, y: e.clientY };
}

/** A press and release within a few pixels is a click; a longer drag was the camera's. */
function onPointerUp(e: PointerEvent): void {
  const d = downAt;
  downAt = null;
  if (!aiming || !d || e.button !== 0) return;
  if (Math.hypot(e.clientX - d.x, e.clientY - d.y) > 5) return;
  const node = nodeAt(e.clientX, e.clientY);
  if (node >= 0) knockNode(node);
  else message = 'No node there: click on a joint.';
}

function onPointerMove(e: PointerEvent): void {
  if (!aiming || !run) return;
  const node = nodeAt(e.clientX, e.clientY);
  if (node < 0) {
    aimMesh.visible = false;
    return;
  }
  const p = run.playing && run.alpha < 1 ? run.drawn : run.snap.positions;
  aimMesh.position.set(p[node * 3]!, p[node * 3 + 1]!, p[node * 3 + 2]!);
  aimMesh.visible = true;
}

// ---- frame loop ----------------------------------------------------------------------------

let last = performance.now();

function updateText(): void {
  saveBtn.disabled = !canSave();
  if (!run) {
    statusEl.textContent = message || 'Pick a scene and press Run.';
    return;
  }
  const secs = (run.step * STEP_SECONDS).toFixed(2);
  const hash = '0x' + run.world.stateHash.toString(16).padStart(16, '0');
  const p = run.player;
  const speedText = speedSel.options[speedSel.selectedIndex]!.text;
  const state = p
    ? p.done
      ? 'replay finished'
      : run.playing
        ? `replaying ${speedText} (${run.rate.toFixed(0)} steps/s)`
        : 'replay paused'
    : run.playing
      ? `running ${speedText} (${run.rate.toFixed(0)} steps/s)`
      : 'paused';
  const check = p
    ? p.mismatch
      ? ` · DIFFERS at step ${p.mismatch.step}: expected 0x${p.mismatch.expected.toString(16).padStart(16, '0')}`
      : p.rejected > 0
        ? ` · the core refused ${p.rejected} recorded commands`
        : p.done
          ? ' · identical to the recording: every checkpoint and the final hash match'
          : ' · matching the recording so far'
    : '';
  const of = p ? ` of ${run.total} (${run.label})` : '';
  const action = run.armed ? ` · ${run.armed.kind}${run.armed.aim ? ' (aimed)' : ''} from step ${run.armed.start}` : '';
  statusEl.innerHTML =
    `<b>${state}</b> · ${run.scene}${action} · step ${run.step}${of} (${secs} s) · ${run.substeps} substeps · ` +
    `${run.world.nodeCount} nodes, ${run.world.beamCount} beams · hash ${hash} · ${rendererName}${check}` +
    (aiming ? '\nKnock: click a node to push it away from the camera. Esc cancels.' : message ? `\n${message}` : '');
}

function frame(now: number): void {
  const dt = Math.min(0.1, (now - last) / 1000);
  last = now;
  if (run?.playing) {
    run.owed += dt * Number(speedSel.value) * 60;
    let n = Math.min(MAX_STEPS_PER_FRAME, Math.floor(run.owed));
    run.owed = n === MAX_STEPS_PER_FRAME ? 0 : run.owed - n;
    const capped = n === MAX_STEPS_PER_FRAME;
    if (run.until >= 0) n = Math.min(n, run.until - run.step);
    if (run.player) n = Math.min(n, run.total - run.step);
    advance(run, n);
    run.alpha = capped ? 1 : run.owed;
    if ((run.until >= 0 && run.step >= run.until) || replayDone(run)) {
      run.playing = false;
      run.alpha = 1;
      syncButtons();
    }
    if (now - run.windowStart >= 1000) {
      run.rate = (run.windowSteps * 1000) / (now - run.windowStart);
      run.windowSteps = 0;
      run.windowStart = now;
    }
  }
  if (run) drawRun(run);
  controls.update();
  void renderer.render(scene, camera);
  updateText();
  requestAnimationFrame(frame);
}

// ---- start ---------------------------------------------------------------------------------

async function main(): Promise<void> {
  const res = await fetch('./beam_core.wasm');
  if (!res.ok) throw new Error(`could not load beam_core.wasm (${res.status})`);
  core = await Core.load(await res.arrayBuffer());

  await renderer.init();
  rendererName = (renderer.backend as { isWebGPUBackend?: boolean }).isWebGPUBackend ? 'WebGPU' : 'WebGL 2';

  const params = new URLSearchParams(location.search);
  const wanted = params.get('scene');
  if (wanted === 'tower' || wanted === 'bridge') sceneSel.value = wanted;
  const steps = params.get('steps');
  const until = steps === null ? -1 : Math.max(0, Number(steps) | 0);

  const actionParam = params.get('action');
  const action: ActionKind | null = actionParam === 'knock' || actionParam === 'removeColumn' || actionParam === 'overload' ? actionParam : null;
  const speed = params.get('speed');
  if (speed !== null && [...speedSel.options].some((o) => o.value === speed)) speedSel.value = speed;

  sceneSel.addEventListener('change', () => startRun(sceneSel.value as SceneName));
  runBtn.addEventListener('click', () => startRun(sceneSel.value as SceneName));
  stopBtn.addEventListener('click', () => {
    stopRun();
    setAiming(false);
    message = 'Stopped.';
  });
  pauseBtn.addEventListener('click', () => run && setPaused(run.playing));
  stepBtn.addEventListener('click', singleStep);
  knockBtn.addEventListener('click', () => setAiming(!aiming));
  removeBtn.addEventListener('click', () => arm('removeColumn'));
  overloadBtn.addEventListener('click', () => arm('overload'));
  saveBtn.addEventListener('click', saveRecording);
  sampleBtn.addEventListener('click', () => void replaySample());
  loadInput.addEventListener('change', () => {
    const file = loadInput.files?.[0];
    loadInput.value = '';
    if (file) void file.arrayBuffer().then((b) => openRecording(new Uint8Array(b), file.name));
  });
  nodesBox.addEventListener('change', () => {
    if (nodeMesh) nodeMesh.visible = nodesBox.checked;
    if (run) drawRun(run);
  });
  resetViewBtn.addEventListener('click', resetView);
  window.addEventListener('resize', resize);
  canvas.addEventListener('pointerdown', onPointerDown);
  canvas.addEventListener('pointerup', onPointerUp);
  canvas.addEventListener('pointermove', onPointerMove);
  window.addEventListener('keydown', (e) => {
    if (e.target instanceof HTMLSelectElement) return;
    if (e.key === ' ') {
      if (e.target instanceof HTMLButtonElement) return; // the button's own click does it
      e.preventDefault();
      if (run) setPaused(run.playing);
    } else if (e.key === '.') singleStep();
    else if (e.key === 'Escape') setAiming(false);
  });

  resize();
  if (params.get('replay') === 'sample') await replaySample(until);
  else startRun(sceneSel.value as SceneName, until, action);
  requestAnimationFrame(frame);
}

main().catch((e: unknown) => {
  statusEl.textContent = `could not start: ${String(e)}`;
  console.error(e);
});
