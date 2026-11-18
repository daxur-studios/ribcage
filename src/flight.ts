/**
 * Purpose: the flight scenario for a craft: put a craft file on the landing pad and play a
 * throttle script into it as commands, step by step, the way `liveBridgeCommands` plays a truck.
 * Value: the headless lift-off (milestone 4.7) and the craft page (4.9) run the same code, so
 * what a test measures is what the page shows, and a recorded flight is just the command lists.
 *
 * A script is a list of "from this step, set this throttle" keys. Every thruster of the craft gets
 * the key's throttle (a craft with two engines can be given a script per engine later). The
 * commands carry numbers only, so the same script always gives the same bytes.
 */

import { CommandBatch } from './commands.ts';
import { loadCraft, type LoadedCraft } from './parts/craft-file.ts';
import { compileCraft } from './parts/assemble.ts';
import { compileTerrain, landingField, type Terrain } from './terrain.ts';

/** From step `step` (0-based, applied before that step) every thruster is set to `throttle`, 0 to 1. */
export interface ThrottleKey {
  step: number;
  throttle: number;
}

/** A script is keys in increasing step order; between keys the throttle stays as last set. */
export type ThrottleScript = readonly ThrottleKey[];

/** Checks a script: whole steps in strictly increasing order, throttles from 0 to 1. Throws saying which key is wrong. */
export function checkScript(script: ThrottleScript): void {
  let last = -1;
  for (const [i, k] of script.entries()) {
    if (!Number.isInteger(k.step) || k.step < 0) throw new Error(`throttle key ${i}: step ${k.step} is not a whole number from 0`);
    if (k.step <= last) throw new Error(`throttle key ${i}: step ${k.step} is not after step ${last}`);
    if (!(k.throttle >= 0 && k.throttle <= 1)) throw new Error(`throttle key ${i}: throttle ${k.throttle} is not from 0 to 1`);
    last = k.step;
  }
}

/** A craft placed for flight: its commands for step 0 and what a test or the page needs to read it. */
export interface Flight {
  loaded: LoadedCraft;
  field: Terrain;
  /** The setup commands (list 0): the landing field, then the craft with its thrusters. */
  setup: CommandBatch;
  /** How many thrusters the craft has (the throttle commands set thrusters `0 .. thrusterCount - 1`). */
  thrusterCount: number;
  /** The core's segment number for each craft segment, when the setup carries segment commands. */
  segmentOf: number[] | null;
}

/**
 * Loads a craft file and builds its setup on the landing field. Throws what `loadCraft` throws.
 * `segments` adds the craft's segment commands to the setup (plan 005); it is off here so the
 * committed hard-landing recording, made before segments existed, is still what recording the hop
 * gives byte for byte. The segment tests turn it on and show it changes no physics.
 */
export function prepareFlight(craftText: string, field: Terrain = landingField(), options: { segments?: boolean } = {}): Flight {
  const loaded = loadCraft(craftText);
  const setup = new CommandBatch();
  compileTerrain(setup, field);
  const segmentOf = compileCraft(setup, loaded.craft, { segments: options.segments ?? false });
  return { loaded, field, setup, thrusterCount: loaded.craft.thrusters.length, segmentOf };
}

/**
 * Appends the commands for step `step` of a flight: the throttle of the key that starts on this
 * step, if there is one, set on every thruster. Nothing else is added, so a step with no key
 * adds no records.
 */
export function flightCommands(batch: CommandBatch, flight: Flight, script: ThrottleScript, step: number): void {
  for (const k of script) {
    if (k.step === step) {
      for (let t = 0; t < flight.thrusterCount; t++) batch.setThrottle(t, k.throttle);
      return;
    }
    if (k.step > step) return;
  }
}
