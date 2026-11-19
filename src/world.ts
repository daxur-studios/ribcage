/**
 * Purpose: one simulated world as the host sees it: commands in, a step, events and render data
 * out (layout rule 8). A thin owner of one core host pointer.
 * Value: the editor, the headless tests and the recorder all drive a world through the same
 * four calls (`apply`, `step`, `events`, `snapshot`), so what is tested headlessly is what runs
 * in the page.
 */

import type { ApplyResult, BeamView, BodyView, CoreEvent, EstimateView, HostPtr, JointView, SegmentProps, SegmentView, SleepView, VesselProps, VesselView } from './core.ts';
import { Core } from './core.ts';
import type { CommandBatch } from './commands.ts';

/** Everything a renderer needs for one frame. Copies: safe to keep. */
export interface Snapshot {
  /** Node positions, three `f32` per node slot. */
  positions: Float32Array;
  beams: BeamView;
  nodeFlags: Uint32Array;
}

export class World {
  readonly #core: Core;
  #h: HostPtr;

  constructor(core: Core) {
    this.#core = core;
    this.#h = core.hostNew();
  }

  /** Applies the commands in order, before the next step. */
  apply(batch: CommandBatch): ApplyResult {
    return this.#core.applyCommands(this.#h, batch);
  }

  /** One fixed 60 Hz step; returns the largest substep count used. */
  step(): number {
    return this.#core.step(this.#h);
  }

  /** Events of the last step. */
  events(): CoreEvent[] {
    return this.#core.events(this.#h);
  }

  get stepCount(): bigint {
    return this.#core.stepCount(this.#h);
  }

  get stateHash(): bigint {
    return this.#core.stateHash(this.#h);
  }

  /** The state hash without the segment section: what moves, collides or pushes. */
  get physicsHash(): bigint {
    return this.#core.physicsHash(this.#h);
  }

  /** Segments, seams and each node's segment (copies). */
  segments(): SegmentView {
    return this.#core.segments(this.#h);
  }

  /** Rigid bodies (plan 005, phase 5.2), as copies. */
  bodies(): BodyView {
    return this.#core.bodies(this.#h);
  }

  /** Joint angles and drive torques after the last step (plan 005, phase 5.5). */
  joints(): JointView {
    return this.#core.joints(this.#h);
  }

  /** Switches the stress estimate on or off (read-only; changes no hash, not recorded). */
  setEstimate(on: boolean): void {
    this.#core.setEstimate(this.#h, on);
  }

  /** The stress estimate after the last step (empty when off). */
  estimate(): EstimateView {
    return this.#core.estimate(this.#h);
  }

  /** The sleep state after the last step (phase 5.8; empty before sleeping ran). */
  sleep(): SleepView {
    return this.#core.sleep(this.#h);
  }

  /** Vessels now (phase 5.10; every node in vessel 0 before the first split). */
  vessels(): VesselView {
    return this.#core.vessels(this.#h);
  }

  /** One vessel's mass, momentum and angular momentum about the origin now. */
  vesselProps(vessel: number): VesselProps {
    return this.#core.vesselProps(this.#h, vessel);
  }

  /** One segment's mass, centre and inertia from the current positions, or `null`. */
  segmentProps(segment: number): SegmentProps | null {
    return this.#core.segmentProps(this.#h, segment);
  }

  get nodeCount(): number {
    return this.#core.nodeCount(this.#h);
  }

  get beamCount(): number {
    return this.#core.beamCount(this.#h);
  }

  /** Full-precision positions, three numbers per node (tests and the recorder). */
  positions(): Float64Array {
    return this.#core.positions(this.#h);
  }

  /** One frame's worth of render data. Call per rendered frame, not per step. */
  snapshot(): Snapshot {
    return {
      positions: this.#core.render(this.#h),
      beams: this.#core.beams(this.#h),
      nodeFlags: this.#core.nodeFlags(this.#h),
    };
  }

  /** Ends the world. Using it afterwards is an error. */
  free(): void {
    this.#core.hostFree(this.#h);
    this.#h = 0;
  }
}
