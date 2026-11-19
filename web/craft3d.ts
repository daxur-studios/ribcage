/**
 * Purpose: the craft page: the hopper on the landing field, flown with the throttle keys or the
 * scripted hop, dropped from a set height, or any one default part on its own; with a follow
 * camera, slow motion at touchdown, and Save and Load for recordings and craft files.
 * Value: the first place a built craft is seen flying and landing. It drives the world only through
 * the same calls as the headless tests (commands in, step, render data out), and a flight made
 * with keys is recorded as the same command lists a script makes, so it can be saved and replayed.
 *
 * Drawing only reads. The core steps at a fixed 60 Hz from the page's clock and never sees a frame
 * time; slow motion changes how fast steps are asked for, never what a step does.
 * `?scene=hop&steps=1320` starts the scripted hop paused after exactly that many steps, so the hash
 * on screen can be compared with the one Node computes for the same steps.
 */

import { CommandBatch } from '../src/commands.ts';
import { Core, EVENT_BREAK, EVENT_YIELD, type BodyView, type SegmentView, type VesselView } from '../src/core.ts';
import {
  craftCentre,
  followStep,
  GALLERY_PARTS,
  galleryCraft,
  readout,
  scenarioFor,
  slowFactor,
  TOUCH_CLEARANCE,
  throttleCommands,
  throttleKey,
  type Readout,
  type Scenario,
  type ScenarioKind,
} from '../src/craft-page.ts';
import { craftPanels } from '../src/craft-panels.ts';
import { STEP_SECONDS } from '../src/design.ts';
import { prepareFlight } from '../src/flight.ts';
import { clearance, HOP_STEPS, PAD, ROUGH_SPOTS, type Spot } from '../src/landing.ts';
import type { LoadedCraft } from '../src/parts/craft-file.ts';
import { lerpPositions } from '../src/render3d.ts';
import {
  beamModeColours,
  bodyBoxes,
  modeCounts,
  rootVesselNodes,
  segmentModes,
  vesselBoxes,
  vesselCount,
  type SegmentMode,
} from '../src/segment-view.ts';
import { decodeRecording, encodeRecording, Player, Recorder, type Recording } from '../src/recording.ts';
import type { Terrain } from '../src/terrain.ts';
import { World, type Snapshot } from '../src/world.ts';
import { createBuilder } from './assemble3d.ts';
import { Stage } from './stage3d.ts';

let MAX_STEPS_PER_FRAME = 8;
/** Seconds in which the follow camera closes half the gap to the craft. */
const FOLLOW_HALF_LIFE = 0.3;
/** Radius of a drawn node, metres. */
const NODE_DRAW_RADIUS = 0.1;
/** The craft is airborne once its lowest point is higher than this, metres (so a craft resting on the pad is not "touching down"). */
const AIRBORNE_CLEARANCE = 0.3;

const $ = <T extends HTMLElement>(id: string): T => {
  const el = document.getElementById(id);
  if (!el) throw new Error(`missing #${id}`);
  return el as T;
};

const canvas = $<HTMLCanvasElement>('view');
const statusEl = $('status');
const readoutEl = $('readout');
const flyBtn = $<HTMLButtonElement>('fly');
const hopBtn = $<HTMLButtonElement>('hop');
const rocketBtn = $<HTMLButtonElement>('rocket');
const roverBtn = $<HTMLButtonElement>('rover');
const sleepBox = $<HTMLInputElement>('sleep');
const stateBox = $<HTMLInputElement>('state');
const stateLegend = $('state-legend');
const stressLegend = $('stress-legend');
const dropBtns = [...document.querySelectorAll<HTMLButtonElement>('button.drop')];
const spotSel = $<HTMLSelectElement>('spot');
const partSel = $<HTMLSelectElement>('part');
const tDownBtn = $<HTMLButtonElement>('t-down');
const tUpBtn = $<HTMLButtonElement>('t-up');
const tFullBtn = $<HTMLButtonElement>('t-full');
const tCutBtn = $<HTMLButtonElement>('t-cut');
const pauseBtn = $<HTMLButtonElement>('pause');
const stepBtn = $<HTMLButtonElement>('step');
const rateSel = $<HTMLSelectElement>('rate');
const slowBox = $<HTMLInputElement>('slow');
const saveBtn = $<HTMLButtonElement>('save');
const loadInput = $<HTMLInputElement>('load');
const sampleBtn = $<HTMLButtonElement>('sample');
const saveCraftBtn = $<HTMLButtonElement>('save-craft');
const loadCraftInput = $<HTMLInputElement>('load-craft');
const beamsBox = $<HTMLInputElement>('beams');
const panelsBox = $<HTMLInputElement>('panels');
const nodesBox = $<HTMLInputElement>('nodes');
const stressBox = $<HTMLInputElement>('stress');
const followBox = $<HTMLInputElement>('follow');
const resetViewBtn = $<HTMLButtonElement>('reset-view');

type Craft = LoadedCraft['craft'];

interface Run {
  world: World;
  kind: ScenarioKind;
  scenario: Scenario;
  craft: Craft;
  field: Terrain;
  step: number;
  playing: boolean;
  /** Fractional steps owed by the clock. */
  owed: number;
  /** Stop playing after this many steps, or -1. */
  until: number;
  substeps: number;
  snap: Snapshot;
  prev: Float32Array;
  alpha: number;
  drawn: Float32Array;
  /** A live run is recorded as it goes; a replay has a player instead. */
  recorder: Recorder | null;
  player: Player | null;
  label: string;
  total: number;
  /** The throttle the keys ask for, and the one last sent to the core. */
  wanted: number;
  sent: number;
  /** Node positions now and one step ago (full precision, for the readouts). */
  pos: Float64Array;
  before: Float64Array;
  broken: Set<number>;
  yields: number;
  /** True once the craft has been clear of the ground; the first touch after that starts slow motion. */
  airborne: boolean;
  touchStep: number;
  read: Readout;
  /** The pace multiplier slow motion gives this frame. */
  slow: number;
  windowSteps: number;
  windowStart: number;
  rate: number;
  coreMs: number;
  coreMsMax: number;
  coreSteps: number;
  /** Sleeping on or off now, or `null` for a replay (the recording decides). */
  sleeping: boolean | null;
  /** A change of the sleeping box, waiting to go out as a recorded `CMD_SET_SLEEPING` on the next step. */
  sleepAsk: boolean | null;
  /** What the core says about segments, bodies and vessels, as of the last step (see `refreshState`). */
  seg: SegmentView | null;
  modes: SegmentMode[];
  bodies: BodyView | null;
  vessels: VesselView | null;
  /** The root vessel's nodes after a split (the follow camera keeps them), or `null` while it is one vessel. */
  rootNodes: number[] | null;
  vesselCount: number;
  beamRgb: Uint8Array;
}

const stage = new Stage(canvas, new URLSearchParams(location.search).get('webgl') === '1');
const { camera, controls } = stage;

let core: Core;
let run: Run | null = null;
let message = '';
let rendererName = '';
/** The craft that Fly, Scripted hop and Drop use, as a craft file's text, and its file name. */
let craftText = '';
let craftName = 'hopper.craft.json';
/** The tall rocket's craft file text. */
let rocketText = '';
let dropSpeed = 7;
/** What the last started scenario was, for Restart. */
let lastStart: { kind: ScenarioKind; until: number } = { kind: 'fly', until: -1 };
/** The last live run's recording, kept after it is replaced so it can still be saved. */
let lastRecording: Recording | null = null;
/** The `?stopat=N` hook: pause once, exactly at step N. -1 for none. */
let stopAt = -1;

const spots: readonly Spot[] = [PAD, ...ROUGH_SPOTS];

// ---- the assembly panel --------------------------------------------------------------------

/** Draws the craft being assembled, paused on the pad (or clears the view when nothing is built); `keepView` leaves the camera where it is. */
function showBuild(text: string | null, keepView: boolean): string | null {
  if (text === null) {
    stopRun();
    stage.clearCraft();
    message = 'Nothing built yet: add a part.';
    return null;
  }
  let flight;
  try {
    flight = prepareFlight(text, undefined, { sleeping: sleepBox.checked });
  } catch (e) {
    return `Could not draw the craft: ${e instanceof Error ? e.message : String(e)}`;
  }
  const cam = camera.position.clone();
  const target = controls.target.clone();
  stopRun();
  const world = new World(core);
  const recorder = new Recorder(world);
  const result = recorder.setup(flight.setup);
  if (result.rejected > 0) {
    world.free();
    return `The core refused ${result.rejected} of the commands for this craft.`;
  }
  if (!openRun(world, 'fly', { kind: 'fly', flight, script: null, steps: null }, recorder, null, '', 0, 0)) return message;
  if (keepView) {
    camera.position.copy(cam);
    controls.target.copy(target);
    controls.update();
  }
  return null;
}

const builder = createBuilder({
  stage,
  show: showBuild,
  fly: (text) => openCraft(text, 'built.craft.json'),
});

// ---- drawing -------------------------------------------------------------------------------

function drawRun(r: Run): void {
  let positions = r.snap.positions;
  if (r.playing && r.alpha < 1) {
    lerpPositions(r.prev, r.snap.positions, r.alpha, r.drawn);
    positions = r.drawn;
  }
  const state = stateBox.checked && r.seg !== null;
  stage.drawSnapshot(r.snap, positions, state ? r.beamRgb : null);
  stage.drawPanels(positions);
  if (state && r.bodies && r.vessels) {
    stage.drawBoxes('body', bodyBoxes(r.bodies, positions).boxes);
    stage.drawBoxes('vessel', vesselBoxes(r.vessels, positions).boxes);
  } else stage.hideBoxes();
}

/**
 * Reads what the core knows about segments, bodies and vessels once per frame (copies; the core's
 * step does not see this): the modes and beam colours for the segment-state view, the root vessel
 * for the follow camera, and the counts the status line prints. Costs a few array copies.
 */
function refreshState(r: Run): void {
  if (r.scenario.flight.segmentOf === null) return;
  r.seg ??= r.world.segments();
  r.modes = segmentModes(r.seg, r.world.sleep());
  r.bodies = r.world.bodies();
  r.vessels = r.world.vessels();
  r.vesselCount = vesselCount(r.vessels);
  r.rootNodes = rootVesselNodes(r.vessels);
  if (stateBox.checked) beamModeColours(r.snap.beams.ends, r.seg.nodeSegment, r.modes, r.beamRgb);
}

/** Where the camera starts: back and up from the craft, looking at it. */
function resetView(): void {
  if (!run) return;
  const c = craftCentre(run.craft, run.snap.positions);
  const o: [number, number, number] = run.kind === 'part' ? [5, 3, 7] : run.kind === 'rocket' ? [20, 8, 26] : run.kind === 'rover' ? [5, 2, 7] : [10, 5, 13];
  stage.setView([c[0] + o[0], c[1] + o[1], c[2] + o[2]], c);
}

/** Moves the look-at point toward the craft and the camera with it, so the orbit the player chose is kept. */
function followCraft(r: Run, dt: number): void {
  if (!followBox.checked) return;
  const positions = r.playing && r.alpha < 1 ? r.drawn : r.snap.positions;
  const c = craftCentre(r.craft, positions, r.rootNodes);
  const t = controls.target;
  const next = followStep([t.x, t.y, t.z], c, dt, FOLLOW_HALF_LIFE);
  camera.position.x += next[0] - t.x;
  camera.position.y += next[1] - t.y;
  camera.position.z += next[2] - t.z;
  t.set(next[0], next[1], next[2]);
}

// ---- running -------------------------------------------------------------------------------

/** Opens a run on `world`, which has had its setup applied. Returns false (and frees the world) when it does not fit the craft. */
function openRun(world: World, kind: ScenarioKind, scenario: Scenario, recorder: Recorder | null, player: Player | null, label: string, total: number, until: number): boolean {
  const craft = scenario.flight.loaded.craft;
  if (world.nodeCount !== craft.nodes.length) {
    message = `Could not open ${label || kind}: its world has ${world.nodeCount} nodes and this craft has ${craft.nodes.length}, so it is not a recording of this craft.`;
    world.free();
    return false;
  }
  stage.showTerrain(scenario.flight.field);
  stage.makeInstances(world.nodeCount, world.beamCount, NODE_DRAW_RADIUS);
  stage.makePanels(craftPanels(craft), world.nodeCount);
  stage.applyLayers();
  const snap = world.snapshot();
  const pos = world.positions().slice();
  run = {
    world,
    kind,
    scenario,
    craft,
    field: scenario.flight.field,
    step: 0,
    playing: until !== 0,
    owed: 0,
    until,
    substeps: 0,
    snap,
    prev: snap.positions,
    alpha: 1,
    drawn: new Float32Array(snap.positions.length),
    recorder,
    player,
    label,
    total,
    wanted: 0,
    sent: 0,
    pos,
    before: pos.slice(),
    broken: new Set(),
    yields: 0,
    airborne: false,
    touchStep: -1,
    read: readout(craft, scenario.flight.field, pos, pos, STEP_SECONDS, new Set()),
    slow: 1,
    windowSteps: 0,
    windowStart: performance.now(),
    rate: 0,
    coreMs: 0,
    coreMsMax: 0,
    coreSteps: 0,
    sleeping: player ? null : sleepBox.checked,
    sleepAsk: null,
    seg: null,
    modes: [],
    bodies: null,
    vessels: null,
    rootNodes: null,
    vesselCount: 1,
    beamRgb: new Uint8Array(snap.beams.flags.length * 3),
  };
  message = '';
  refreshState(run);
  drawRun(run);
  resetView();
  syncButtons();
  return true;
}

function currentSpot(): Spot {
  return spots[Number(spotSel.value)] ?? PAD;
}

/** Starts a scenario on a fresh world. `until` stops it after that many steps (0 holds at the start), or -1 for no stop. */
function startScenario(kind: ScenarioKind, until = -1): void {
  let scenario: Scenario;
  try {
    scenario = scenarioFor(kind, craftText, { spot: currentSpot(), speed: dropSpeed, part: partSel.value, sleeping: sleepBox.checked, rocketText });
  } catch (e) {
    message = `Could not start ${kind}: ${e instanceof Error ? e.message : String(e)}`;
    return;
  }
  stopRun();
  const world = new World(core);
  const recorder = new Recorder(world);
  const result = recorder.setup(scenario.flight.setup);
  if (result.rejected > 0) {
    message = `The core refused ${result.rejected} of the commands; the scenario was not started.`;
    world.free();
    return;
  }
  lastStart = { kind, until };
  builder.leave();
  openRun(world, kind, scenario, recorder, null, '', 0, until);
}

/** Replays a recording on a fresh world drawn as the loaded craft on the landing field; the hashes say whether it is that craft's. */
function startReplay(rec: Recording, label: string, until = -1): void {
  let flight;
  try {
    flight = prepareFlight(craftText);
  } catch (e) {
    message = `Could not open ${label}: ${e instanceof Error ? e.message : String(e)}`;
    return;
  }
  stopRun();
  const world = new World(core);
  const player = new Player(world, rec);
  const result = player.start();
  if (result.rejected > 0) {
    message = `The core refused ${result.rejected} of the recorded setup commands.`;
    world.free();
    return;
  }
  openRun(world, 'fly', { kind: 'fly', flight, script: null, steps: null }, null, player, label, rec.steps, until);
}

function stopRun(): void {
  if (run?.recorder && run.recorder.steps > 0) lastRecording = run.recorder.finish();
  run?.world.free();
  run = null;
  syncButtons();
}

/** One fixed step: the scripted or keyed commands, then the core's step; then what the page reads off it. */
function stepOnce(r: Run): void {
  const t0 = performance.now();
  if (r.player) {
    const s = r.player.advance();
    if (s < 0) return;
    r.substeps = s;
  } else {
    const batch = new CommandBatch();
    if (r.sleepAsk !== null) {
      batch.setSleeping(r.sleepAsk);
      r.sleeping = r.sleepAsk;
      r.sleepAsk = null;
    }
    if (r.scenario.script) r.scenario.script(batch, r.step);
    else if (r.kind === 'fly') r.sent = throttleCommands(batch, r.scenario.flight.thrusterCount, r.wanted, r.sent);
    r.substeps = r.recorder!.step(batch);
  }
  const ms = performance.now() - t0;
  r.coreMs += ms;
  r.coreMsMax = Math.max(r.coreMsMax, ms);
  r.coreSteps++;
  r.step++;
  r.windowSteps++;
  for (const e of r.world.events()) {
    if (e.kind === EVENT_BREAK) r.broken.add(e.index);
    else if (e.kind === EVENT_YIELD) r.yields++;
  }
  r.before.set(r.pos);
  r.pos.set(r.world.positions());
  r.read = readout(r.craft, r.field, r.pos, r.before, STEP_SECONDS, r.broken);
  if (r.read.altitude > AIRBORNE_CLEARANCE) r.airborne = true;
  if (r.airborne && r.touchStep < 0 && clearance(r.craft, r.pos, r.field) < TOUCH_CLEARANCE) r.touchStep = r.step;
}

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
  refreshState(r);
}

function setPaused(paused: boolean): void {
  if (!run) return;
  if (!paused && replayDone(run)) return;
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

/** A key or button asks for a throttle; it reaches the core as a recorded command on the next step. */
function pressThrottle(key: string): boolean {
  const next = run ? throttleKey(run.wanted, key) : null;
  if (!run || next === null) return false;
  if (run.kind !== 'fly' || run.player) {
    message = run.player ? 'A replay flies itself.' : 'This scenario is not flown by hand: press Hopper first.';
    return true;
  }
  run.wanted = next;
  return true;
}

// ---- recordings and craft files ------------------------------------------------------------

function currentRecording(): Recording | null {
  if (run?.recorder) return run.recorder.steps > 0 ? run.recorder.finish() : null;
  return run ? null : lastRecording;
}

function canSave(): boolean {
  return run ? run.recorder !== null && run.recorder.steps > 0 : lastRecording !== null;
}

function download(name: string, data: Uint8Array<ArrayBuffer> | string, type: string): number {
  const blob = new Blob([data], { type });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = name;
  a.click();
  URL.revokeObjectURL(a.href);
  return blob.size;
}

function saveRecording(): void {
  const rec = currentRecording();
  if (!rec) return;
  const size = download(`ribcage-craft-${rec.steps}-steps.rcrd`, encodeRecording(rec) as Uint8Array<ArrayBuffer>, 'application/octet-stream');
  message = `Saved ${rec.steps} steps (${size} bytes).`;
}

function openRecording(bytes: Uint8Array, label: string, until = -1): void {
  try {
    startReplay(decodeRecording(bytes), label, until);
  } catch (e) {
    message = `Could not open ${label}: ${e instanceof Error ? e.message : String(e)}`;
  }
}

async function replaySample(until = -1): Promise<void> {
  const res = await fetch('./hard-landing.rcrd');
  if (!res.ok) {
    message = `could not load the sample recording (${res.status})`;
    return;
  }
  openRecording(new Uint8Array(await res.arrayBuffer()), 'sample: hard landing', until);
}

/** Loads a craft file and flies it; a file that does not load leaves the page as it was. */
function openCraft(text: string, name: string): void {
  try {
    scenarioFor('fly', text);
  } catch (e) {
    message = `Could not load ${name}: ${e instanceof Error ? e.message : String(e)}`;
    return;
  }
  craftText = text;
  craftName = name;
  builder.leave();
  builder.loadText(text);
  startScenario('fly');
  if (run) message = `Loaded ${name}.`;
}

// ---- buttons and text ----------------------------------------------------------------------

function syncButtons(): void {
  pauseBtn.disabled = run === null;
  pauseBtn.textContent = run && !run.playing ? 'Resume' : 'Pause';
  stepBtn.disabled = run === null || run.playing || replayDone(run);
  const hand = run !== null && run.kind === 'fly' && run.player === null;
  for (const b of [tDownBtn, tUpBtn, tFullBtn, tCutBtn]) b.disabled = !hand;
  flyBtn.classList.toggle('on', run?.kind === 'fly' && run.player === null);
  rocketBtn.classList.toggle('on', run?.kind === 'rocket');
  roverBtn.classList.toggle('on', run?.kind === 'rover');
  sleepBox.disabled = run === null || run.player !== null || run.scenario.flight.segmentOf === null;
  hopBtn.classList.toggle('on', run?.kind === 'hop');
  for (const b of dropBtns) b.classList.toggle('on', run?.kind === 'drop' && Number(b.dataset.speed) === dropSpeed);
}

const hex = (h: bigint): string => '0x' + h.toString(16).padStart(16, '0');

/** Sleeping, segments asleep, bodies and vessels, for the status line (empty for a craft with no segments). */
function stateText(r: Run): string {
  if (r.seg === null) return '';
  const c = modeCounts(r.modes);
  const bodies = r.bodies ? r.bodies.flags.filter((f) => (f & 1) !== 0).length : 0;
  const sleeping = r.sleeping === null ? 'sleeping per recording' : r.sleeping ? 'sleeping on' : 'sleeping off';
  const vessels = r.vesselCount > 1 ? ` · <span class="warn">${r.vesselCount} vessels</span>` : '';
  return `${sleeping} · ${c.asleep} asleep, ${c.live} live, ${c.rigid} rigid · ${bodies} bodies${vessels} · `;
}

function updateText(): void {
  saveBtn.disabled = !canSave();
  if (!run) {
    readoutEl.textContent = '';
    statusEl.textContent = message || 'Pick a scenario.';
    return;
  }
  const r = run.read;
  const lines: string[] = [];
  if (run.kind === 'fly' && !run.player) {
    const pct = Math.round(run.wanted * 100);
    lines.push(`throttle ${'█'.repeat(Math.round(run.wanted * 10))}${'░'.repeat(10 - Math.round(run.wanted * 10))} ${pct}%`);
  } else if (run.scenario.script) {
    lines.push('<span class="dim">scripted throttle</span>');
  }
  lines.push(`altitude ${r.altitude.toFixed(1)} m · speed ${r.speed.toFixed(1)} m/s · climb ${r.climb.toFixed(1)} m/s`);
  if (r.tilt !== null) lines.push(`tilt ${r.tilt.toFixed(0)}°`);
  if (r.lostLegs !== null) lines.push(r.lostLegs > 0 ? `<span class="warn">legs lost ${r.lostLegs} of ${r.legCount}</span>` : `legs lost 0 of ${r.legCount}`);
  if (run.broken.size > 0) lines.push(`<span class="warn">${run.broken.size} beams broken</span>`);
  if (run.slow < 1) lines.push('<span class="warn">slow motion</span>');
  readoutEl.innerHTML = lines.join('\n');

  const p = run.player;
  const rateText = rateSel.options[rateSel.selectedIndex]!.text;
  const state = p
    ? p.done
      ? 'replay finished'
      : run.playing
        ? `replaying ${rateText} (${run.rate.toFixed(0)} steps/s)`
        : 'replay paused'
    : run.playing
      ? `running ${rateText}${run.slow < 1 ? ' slow-mo' : ''} (${run.rate.toFixed(0)} steps/s)`
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
  const what = builder.building ? 'assembling' : run.kind === 'part' ? `part ${partSel.value}` : run.kind === 'drop' ? `drop ${dropSpeed} m/s at ${currentSpot().name}` : run.kind;
  statusEl.innerHTML =
    `<b>${state}</b> · ${p ? 'replay' : what} · step ${run.step}${of} (${(run.step * STEP_SECONDS).toFixed(2)} s) · ${run.substeps} substeps · ` +
    (run.coreSteps > 0 ? `core ${(run.coreMs / run.coreSteps).toFixed(2)} ms/step (worst ${run.coreMsMax.toFixed(2)}) · ` : '') +
    stateText(run) +
    `${run.world.nodeCount} nodes, ${run.world.beamCount} beams · hash ${hex(run.world.stateHash)} · ${rendererName}${check}` +
    (message ? `\n${message}` : '');
}

// ---- frame loop ----------------------------------------------------------------------------

let last = performance.now();

function frame(now: number): void {
  const dt = Math.min(0.1, (now - last) / 1000);
  last = now;
  if (run?.playing) {
    run.slow = slowBox.checked ? slowFactor(run.read.altitude, -run.read.climb, run.touchStep < 0 ? -1 : run.step - run.touchStep) : 1;
    run.owed += dt * Number(rateSel.value) * run.slow * 60;
    let n = Math.min(MAX_STEPS_PER_FRAME, Math.floor(run.owed));
    run.owed = n === MAX_STEPS_PER_FRAME ? 0 : run.owed - n;
    const capped = n === MAX_STEPS_PER_FRAME;
    if (run.until >= 0) n = Math.min(n, run.until - run.step);
    if (stopAt >= 0 && !run.player) n = Math.min(n, Math.max(0, stopAt - run.step));
    if (run.player) n = Math.min(n, run.total - run.step);
    advance(run, n);
    run.alpha = capped ? 1 : run.owed;
    const reachedStop = stopAt >= 0 && !run.player && run.step >= stopAt;
    if (reachedStop) stopAt = -1;
    if ((run.until >= 0 && run.step >= run.until) || replayDone(run) || reachedStop) {
      run.playing = false;
      run.alpha = 1;
      syncButtons();
    }
    if (now - run.windowStart >= 1000) {
      run.rate = (run.windowSteps * 1000) / (now - run.windowStart);
      run.windowSteps = 0;
      run.windowStart = now;
    }
  } else if (run) {
    run.slow = 1;
  }
  if (run) {
    drawRun(run);
    followCraft(run, dt);
  }
  controls.update();
  stage.render();
  updateText();
  requestAnimationFrame(frame);
}

// ---- start ---------------------------------------------------------------------------------

function syncLegend(): void {
  stateLegend.style.display = stateBox.checked ? 'block' : 'none';
  stressLegend.style.display = stateBox.checked ? 'none' : 'block';
}

/** Test hook: counts errors on the page so a headless run can say none happened. */
function countErrors(): void {
  const mark = (): void => document.documentElement.setAttribute('data-errors', String(Number(document.documentElement.getAttribute('data-errors') ?? '0') + 1));
  document.documentElement.setAttribute('data-errors', '0');
  window.addEventListener('error', mark);
  window.addEventListener('unhandledrejection', mark);
  const original = console.error.bind(console);
  console.error = (...a: unknown[]) => {
    mark();
    original(...a);
  };
}

async function main(): Promise<void> {
  countErrors();
  const res = await fetch('./beam_core.wasm');
  if (!res.ok) throw new Error(`could not load beam_core.wasm (${res.status})`);
  core = await Core.load(await res.arrayBuffer());
  const craftRes = await fetch('./hopper.craft.json');
  if (!craftRes.ok) throw new Error(`could not load hopper.craft.json (${craftRes.status})`);
  craftText = await craftRes.text();
  builder.loadText(craftText);

  rendererName = await stage.init();

  for (const [i, s] of spots.entries()) spotSel.add(new Option(s.name === 'pad' ? 'pad (flat)' : `rough: ${s.name}`, String(i)));
  for (const p of GALLERY_PARTS) partSel.add(new Option(p.type, p.type));
  partSel.add(new Option('— pick a part —', ''), 0);
  partSel.value = '';

  const params = new URLSearchParams(location.search);
  const sceneParam = params.get('scene');
  const kind: ScenarioKind =
    sceneParam === 'hop' || sceneParam === 'drop' || sceneParam === 'part' || sceneParam === 'rocket' || sceneParam === 'rover' ? sceneParam : 'fly';
  const rocketRes = await fetch('./tall-rocket.craft.json');
  if (rocketRes.ok) rocketText = await rocketRes.text();
  rocketBtn.disabled = rocketText === '';
  if (params.get('sleep') === '0') sleepBox.checked = false;
  if (params.get('view') === 'state') stateBox.checked = true;
  syncLegend();
  const speedParam = Number(params.get('drop'));
  if (speedParam > 0) dropSpeed = speedParam;
  const spotParam = params.get('spot');
  if (spotParam !== null && spots[Number(spotParam)]) spotSel.value = spotParam;
  const partParam = params.get('part');
  if (partParam !== null && GALLERY_PARTS.some((p) => p.type === partParam)) partSel.value = partParam;
  const steps = params.get('steps');
  // Test hook: more steps per frame, so a headless run with slow software rendering reaches its step.
  const burst = Number(params.get('burst'));
  if (burst > 0) MAX_STEPS_PER_FRAME = burst | 0;
  const stop = params.get('stopat');
  if (stop !== null) stopAt = Math.max(0, Number(stop) | 0);
  if (params.get('beams') === '0') beamsBox.checked = false;
  if (params.get('panels') === '0') panelsBox.checked = false;
  if (params.get('stress') === '0') stressBox.checked = false;
  if (params.get('slow') === '0') slowBox.checked = false;
  if (params.get('follow') === '0') followBox.checked = false;
  const pace = params.get('pace');
  if (pace !== null && [...rateSel.options].some((o) => o.value === pace)) rateSel.value = pace;
  stage.showBeams = beamsBox.checked;
  stage.showNodes = nodesBox.checked;
  stage.showPanels = panelsBox.checked;
  stage.stressColours = stressBox.checked;

  flyBtn.addEventListener('click', () => startScenario('fly'));
  hopBtn.addEventListener('click', () => startScenario('hop', HOP_STEPS));
  rocketBtn.addEventListener('click', () => startScenario('rocket'));
  roverBtn.addEventListener('click', () => startScenario('rover'));
  sleepBox.addEventListener('change', () => {
    if (run && run.player === null) run.sleepAsk = sleepBox.checked;
  });
  stateBox.addEventListener('change', () => {
    syncLegend();
    if (run) {
      refreshState(run);
      drawRun(run);
    }
  });
  for (const b of dropBtns) {
    b.addEventListener('click', () => {
      dropSpeed = Number(b.dataset.speed);
      startScenario('drop');
    });
  }
  spotSel.addEventListener('change', () => run?.kind === 'drop' && startScenario('drop'));
  partSel.addEventListener('change', () => partSel.value !== '' && startScenario('part', 0));
  tDownBtn.addEventListener('click', () => pressThrottle('ArrowDown'));
  tUpBtn.addEventListener('click', () => pressThrottle('ArrowUp'));
  tFullBtn.addEventListener('click', () => pressThrottle('z'));
  tCutBtn.addEventListener('click', () => pressThrottle('x'));
  pauseBtn.addEventListener('click', () => run && setPaused(run.playing));
  stepBtn.addEventListener('click', singleStep);
  saveBtn.addEventListener('click', saveRecording);
  sampleBtn.addEventListener('click', () => void replaySample());
  loadInput.addEventListener('change', () => {
    const file = loadInput.files?.[0];
    loadInput.value = '';
    if (file) void file.arrayBuffer().then((b) => openRecording(new Uint8Array(b), file.name));
  });
  saveCraftBtn.addEventListener('click', () => {
    const part = run?.kind === 'part';
    const built = builder.building ? builder.panel.text() : null;
    const size = download(built !== null ? 'built.craft.json' : part ? `${partSel.value}.craft.json` : craftName, built ?? (part ? galleryCraft(partSel.value) : craftText), 'application/json');
    message = `Saved the craft file (${size} bytes).`;
  });
  loadCraftInput.addEventListener('change', () => {
    const file = loadCraftInput.files?.[0];
    loadCraftInput.value = '';
    if (file) void file.text().then((t) => openCraft(t, file.name));
  });
  for (const [box, layer] of [
    [beamsBox, 'showBeams'],
    [nodesBox, 'showNodes'],
    [panelsBox, 'showPanels'],
  ] as const) {
    box.addEventListener('change', () => {
      stage[layer] = box.checked;
      stage.applyLayers();
      if (run) drawRun(run);
    });
  }
  stressBox.addEventListener('change', () => {
    stage.stressColours = stressBox.checked;
    if (run) drawRun(run);
  });
  resetViewBtn.addEventListener('click', resetView);
  $('build').addEventListener('click', () => (builder.isOpen ? builder.close() : builder.open()));
  window.addEventListener('resize', () => stage.resize());
  window.addEventListener('keydown', (e) => {
    if (e.target instanceof HTMLSelectElement || e.target instanceof HTMLInputElement) return;
    if (e.ctrlKey || e.metaKey || e.altKey) return;
    if (e.key === ' ') {
      if (e.target instanceof HTMLButtonElement) return; // the button's own click does it
      e.preventDefault();
      if (run) setPaused(run.playing);
    } else if (e.key === '.') singleStep();
    else if (e.key === 'r') startScenario(lastStart.kind, lastStart.until);
    else if (pressThrottle(e.key)) e.preventDefault();
  });

  stage.resize();
  const buildParam = params.get('build');
  // Test hook: where a free port's marker is on the page, so a headless browser can click it.
  (window as unknown as { __ribcage: unknown }).__ribcage = { markerScreenPosition: (name: string) => builder.markerAt(name) };
  if (buildParam !== null) {
    builder.open();
    if (buildParam === 'hopper') void builder.demo();
  } else if (params.get('replay') === 'sample') await replaySample(steps === null ? -1 : Math.max(0, Number(steps) | 0));
  else {
    const until = steps !== null ? Math.max(0, Number(steps) | 0) : kind === 'hop' ? HOP_STEPS : kind === 'part' ? 0 : -1;
    if (kind === 'part' && partSel.value === '') partSel.value = GALLERY_PARTS[0]!.type;
    startScenario(kind, until);
    // Test hook: run straight to the step in one go (no frames), so a headless run with slow software rendering gets there.
    if (params.get('jump') === '1' && run && until > 0) {
      advance(run, until - run.step);
      drawRun(run);
      updateText();
    }
  }
  requestAnimationFrame(frame);
}

main().catch((e: unknown) => {
  statusEl.textContent = `could not start: ${String(e)}`;
  console.error(e);
});
