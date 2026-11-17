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
import { STANDARD_TRUCK, STEP_SECONDS } from '../src/design.ts';
import { newPlacement, placeBeam, terrainTriangles } from '../src/render3d.ts';
import {
  bridgeAction,
  buildBridge3,
  buildTower,
  MEMBER,
  STANDARD_BRIDGE3,
  STANDARD_TOWER,
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
  snap: Snapshot;
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
let core: Core;
let run: Run | null = null;
let message = '';
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

/** Copies a snapshot into the instance buffers. */
function drawSnapshot(s: Snapshot): void {
  if (!beamMesh || !nodeMesh) return;
  const { positions, beams, nodeFlags } = s;
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

function startRun(name: SceneName, until = -1): void {
  stopRun();
  const world = new World(core);
  const batch = new CommandBatch();
  const layout = name === 'tower' ? buildTower(batch, STANDARD_TOWER) : buildBridge3(batch, STANDARD_BRIDGE3);
  const result = world.apply(batch);
  if (result.rejected > 0) {
    message = `The core refused ${result.rejected} of the commands; the scene was not started.`;
    world.free();
    return;
  }
  showTerrain(layout.terrain);
  makeInstances(layout.nodeCount, layout.beamCount);
  const snap = world.snapshot();
  run = { world, scene: name, layout, step: 0, playing: until !== 0, owed: 0, until, substeps: 0, snap };
  message = '';
  drawSnapshot(snap);
  resetView();
  syncButtons();
}

function stopRun(): void {
  run?.world.free();
  run = null;
  syncButtons();
}

/** One fixed step: the bridge's truck as commands, then the core's step. */
function stepOnce(r: Run): void {
  const batch = new CommandBatch();
  if (r.scene === 'bridge') bridgeAction(batch, r.layout as BridgeLayout3, null, r.step, STANDARD_TRUCK);
  r.world.apply(batch);
  r.substeps = r.world.step();
  r.step++;
}

function syncButtons(): void {
  runBtn.textContent = run ? 'Restart' : 'Run';
  stopBtn.disabled = run === null;
}

// ---- frame loop ----------------------------------------------------------------------------

let last = performance.now();

function updateText(): void {
  if (!run) {
    statusEl.textContent = message || 'Pick a scene and press Run.';
    return;
  }
  const secs = (run.step * STEP_SECONDS).toFixed(2);
  const hash = '0x' + run.world.stateHash.toString(16).padStart(16, '0');
  statusEl.innerHTML =
    `<b>${run.playing ? 'running' : 'paused'}</b> · ${run.scene} · step ${run.step} (${secs} s) · ${run.substeps} substeps · ` +
    `${run.world.nodeCount} nodes, ${run.world.beamCount} beams · hash ${hash} · ${rendererName}` +
    (message ? `\n${message}` : '');
}

function frame(now: number): void {
  const dt = Math.min(0.1, (now - last) / 1000);
  last = now;
  if (run?.playing) {
    run.owed += dt * 60;
    let n = Math.min(MAX_STEPS_PER_FRAME, Math.floor(run.owed));
    run.owed = n === MAX_STEPS_PER_FRAME ? 0 : run.owed - n;
    if (run.until >= 0) n = Math.min(n, run.until - run.step);
    for (let i = 0; i < n; i++) stepOnce(run);
    if (n > 0) {
      run.snap = run.world.snapshot();
      drawSnapshot(run.snap);
    }
    if (run.until >= 0 && run.step >= run.until) run.playing = false;
  }
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

  sceneSel.addEventListener('change', () => startRun(sceneSel.value as SceneName));
  runBtn.addEventListener('click', () => startRun(sceneSel.value as SceneName));
  stopBtn.addEventListener('click', () => {
    stopRun();
    message = 'Stopped.';
  });
  nodesBox.addEventListener('change', () => {
    if (nodeMesh) nodeMesh.visible = nodesBox.checked;
    if (run) drawSnapshot(run.snap);
  });
  resetViewBtn.addEventListener('click', resetView);
  window.addEventListener('resize', resize);

  resize();
  startRun(sceneSel.value as SceneName, steps === null ? -1 : Math.max(0, Number(steps) | 0));
  requestAnimationFrame(frame);
}

main().catch((e: unknown) => {
  statusEl.textContent = `could not start: ${String(e)}`;
  console.error(e);
});
