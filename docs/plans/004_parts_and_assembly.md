---
type: Plan
title: Parts and assembly
description: Milestone 4 - tank, truss, landing leg, engine and pod generated from parameters, joined at attachment rings into a craft saved as a craft file, engines with thrust, and a hard landing on rough terrain that snaps a leg (the first playable slice).
timestamp: 2026-11-17T20:00:00.000Z
status: active
record: /docs/engineering-records/004_parts_and_assembly.md
tags: [milestone-4, parts, assembly, craft-file, thrust, landing]
---

# Plan 004: Parts and assembly

Evidence: [ER 004](/docs/engineering-records/004_parts_and_assembly.md). Design: [spec](/docs/research/node-beam-engine-design-spec.md) (parts, attachment rings, craft files, materials), the [rigid-by-default brief](/docs/research/rigid-by-default-design-brief.md) (its milestone 4 rows are binding here: segments, seams, section properties, assembly tree, never-live flag), and the [core layout rules](/docs/research/core-data-layout.md) (still binding, rules 7, 8 and 9 above all).

**What this milestone adds.** Five part generators that turn a few parameters into nodes, beams and skin panels; an assembler that welds parts at attachment rings into one craft; a craft file that saves and loads it exactly; thrusters in the core; and a craft page where a small lander (the "hopper") lifts off, flies, and lands. The test of fun is the last one: land it hard on rough ground and a leg snaps and the craft tips over. Rigid bodies, joints, sleeping and splitting into vessels stay in milestone 5; lift and drag in milestone 6; craft-against-craft contact in milestone 8.

**Scope [assumption]:** still a test bed (Plan 002). One sample craft, a small assembly panel, throttle keys and scripted scenarios, the debug tools from milestone 3. No part editor beyond parameters, no staging, no fuel, no steering.

## Settled before building

These are decided and logged in [ER 004](/docs/engineering-records/004_parts_and_assembly.md) (first dated section). A builder follows them; changing one is a logged decision with a reason, not a silent edit.

- **The rigid engine and pod stay in milestone 5.** In this milestone the engine and the pod are compact, fully braced lattices, with their mass spread over their nodes and every node flagged never-live in the part data. Milestone 5 turns those flagged segments into rigid bodies without touching the generators. The roadmap is unchanged. (The brief recommended moving a minimal rigid body here; see ER 004 for why not.)
- **Generators are TypeScript** (`src/parts/`), pure functions from parameters to a part, compiled to commands like `src/design.ts` and `src/scenes3d.ts`. They use only `+ − × ÷` and `sqrt`, so every engine gives the same bits. Ring node counts come from {4, 6, 8, 12, 16, 24}, and their points are built from exact values and half-angle square roots (multiples of 15° and 22.5°), never from sine or cosine.
- **What a part is.** Nodes (position, mass, radius, segment index), beams (end nodes, material, cross-section area, damping, and a ring-beam flag), skin panels (the 3.8 binding: each vertex on four non-coplanar nodes of its cell), attachment rings and mount patches, seams with their section properties (axial area and bending stiffness at the cut), a never-live flag per segment, one coarse collision capsule (emitted, unused until milestone 8), and thrusters for engines. Every tube is braced with **both** diagonals in every wall quad (ER 001: one diagonal makes a twist-and-crush mechanism).
- **Materials are real, stiffness scaled.** Steel, aluminium and carbon fibre with real density and real yield and break stresses; Young's modulus multiplied by one common scale `s = 0.1` **[assumption]** so the stiffest members fit the substep budget. Yield and break strains are derived from the real stresses and the scaled modulus, so **members yield and break at real forces**; they only stretch ten times further on the way. Numbers in ER 004.
- **Substep budget.** Every default part alone, and the hopper, steps at no more than 24 substeps (the cap stays 32), leaving room for stiffer user parameters. The heaviest to lightest node ratio across a craft is at most 10.
- **Welding happens in the assembler, in TypeScript.** A child's attachment ring is placed against the parent's (axes opposed, centres equal), turned by a whole number of ring steps (the "clock", a permutation, no angles), and its nodes are merged into the parent's: masses add, the merged node keeps the parent's segment, the child's ring beams are dropped (the parent's carry the ring), and the child's beams that touch merged nodes become seam beams. Rings must match in node count and in radius within 1 mm; a mismatch is refused with a reason. Legs use the same weld on a **mount patch** (two adjacent nodes on each of two adjacent rings of a tank or truss). No beam-bridged joins in this milestone.
- **Craft file:** JSON, `"format": "ribcage-craft"`, `"version": 1`; the parts (id, type, parameters), the joins (child, child ring, parent, parent ring or patch, clock), and the root placement (position and a yaw clock in 15° steps). Written in one canonical order with JavaScript's shortest round-trip numbers, so load-then-save gives the same text, and a loaded file compiles to the same command bytes. Unknown fields, unknown part types and later versions are refused with a reason. No damage layer yet (wrecks are not saved).
- **Thrust lives in the core, as commands.** `CMD_ADD_THRUSTER` (9): `a` = tail node, `b` = head node, `f[0]` = maximum thrust in newtons; result: the thruster's index. `CMD_SET_THROTTLE` (10): `a` = thruster, `f[0]` = throttle from 0 to 1. Each step, before the substeps, every thruster with throttle above zero adds `throttle × maximum × unit(head − tail)` through the one force door (`add_force`), half on each node, using positions at the start of the step. A thruster whose node is deleted, or whose two nodes coincide, gives nothing. Thrusters and throttles are in the state hash. `ABI_VERSION` 4; the recording decoder accepts ABI 2, 3 and 4. No gimbal, no fuel flow (mass stays constant).
- **The landing leg** is a tapering four-longeron aluminium lattice from its mount patch to a foot square, with, from the bottom: foot pad nodes with a larger radius, a **sprung bay** (soft, well damped beams that take the first part of a landing), a **crush bay** (yields at a set force, like the crushable cartridges of real lander legs, and breaks at the end of its stroke), and the upper leg (stronger than the crush bay). The core already has everything this needs: per-beam compliance, damping, yield as a force cap, break on total strain in compression. No deployment or actuator (milestone 5's servos).
- **The hopper** (reference craft, `test/fixtures/hopper.craft.json`): pod 1200 kg, tank 2000 kg, engine 800 kg with one 80 kN thruster, four legs at 150 kg each; 4600 kg in all, thrust to weight about 1.77. Shapes and numbers in ER 004.
- **A landing field terrain**: a new arithmetic-only generator in `src/terrain.ts`, 65 × 65 points at 0.5 m (32 m square, small enough for a golden recording), a flat pad, and a rough area of bumps 0.3 to 0.8 m high and 1 to 2 m across beside it.
- **No floating origin.** Hops stay within a few hundred metres of the origin, as in milestone 3.

## Phases

Each phase ends with passing tests (`cargo test`, `pnpm test`, `pnpm typecheck`), ticks the matching check in the [roadmap](/ROADMAP.md) if it proves one, and logs a dated section in the record. "Build with" is the model for that session; mechanical follow-ups (ticks, indexes, renames) can go to Haiku.

Order: 4.1 → 4.2 → 4.3 → 4.4 → 4.5. Phase 4.6 (the core) can run side by side with 4.2 to 4.5 (different files). Then 4.7. Then 4.8 and 4.9 can run side by side (`src/parts/` and tests against `web/`), then 4.10, then 4.11.

Roadmap checks: "generated from parameters" in 4.3; "join at rings, saved and loaded" in 4.5; "lifts off" in 4.7 (headless), seen in 4.9; "test of fun" when 4.8's bands pass and the snap is seen on the page in 4.9, whichever comes second. Each check is a quarter of the milestone.

- [x] **4.1 Materials and the part data model.** Build with: Sonnet.
  `src/parts/material.ts`: the three materials from ER 004 and the scale `s`; a member from a material and a cross-section area gives `EA = s·E·A`, the yield and break strains, and damping from a damping ratio (`c = ζ · 2 · sqrt(k · m_a·m_b / (m_a + m_b))`, `k = EA / L`). `src/parts/part.ts`: the part shape from "Settled" above, a `compilePart` to commands (reusing the `compileStruct` pattern), a mass total, and a budget check that reports the substep count the core's rule will pick for a part (the same `ω·h ≤ c` sum, in TypeScript) and the node-mass ratio. Also adds the plan link to the roadmap's milestone 4 heading (mechanical).
  **Test:** a single steel strut and a single aluminium strut, pulled in the core, yield at `σ_y · A` within 1% and break at their stated strain, in tension and in compression; the reported substep count equals what the core uses on a small test lattice; damping from `ζ` on a two-node spring decays at the expected rate within 5%.
  Demo: skip (nothing to see yet).

- [x] **4.2 Truss and tank generators.** Build with: Sonnet.
  `src/parts/truss.ts` (square or triangular section, length, width, bays, material, member area) and `src/parts/tank.ts` (radius, length, ring node count, rings, material, wall area, mass): ring and longitudinal members, crossed diagonals in every wall quad, in-ring perimeter, chords and diameters so each ring keeps its shape, end rings declared as attachment rings, side mount patches, one segment per bay **[assumption]** with seams and section properties between bays, panels on every wall quad.
  **Test** (`test/parts.test.ts`): for a sweep of parameters each part compiles, its mass equals the stated mass exactly, it is within the substep budget, it rests on flat ground for 10 s with no yield and comes to rest (every node slower than 1 mm/s), a tank squeezed across a diameter springs back (no mechanism), and the same parameters give byte-identical commands twice; ring points are exact (each node at the stated radius within 1e-12 m, and opposite nodes exact negatives).
  Demo: skip (shown in 4.9).

- [x] **4.3 Leg, engine and pod generators.** Build with: Sonnet.
  `src/parts/leg.ts` (length, outward foot offset in metres, foot size, mount patch size, material, sprung-bay stiffness and stroke, crush force and stroke), `src/parts/engine.ts` (top ring, nozzle ring radius, height, mass, maximum thrust; a tail node on the axis at the nozzle and a head node on the axis at the top, both braced to both rings; never-live) and `src/parts/pod.ts` (bottom ring, top ring, height, apex node, mass; never-live). Engine bell and pod shell panels. Ticks the roadmap check "tank, truss, landing leg, engine and pod are generated from parameters" (with 4.2).
  **Test:** 4.2's checks for all three; a leg pressed along its axis at a slowly rising force (a headless press: the mount pinned, a force on the foot) stays elastic below the crush force, then shortens at a nearly constant force (within 10%) over its crush stroke, then the crush bay breaks; the upper leg does not yield before the crush bay does; the engine and pod each carry the hopper's full thrust through their lattices with no yield.
  Demo: skip (the parts gallery is shown in 4.9).

- [x] **4.4 Attachment rings and the assembler.** Build with: Sonnet.
  `src/parts/assemble.ts`: the weld from "Settled", the clock, mount patches, the assembly tree (each part's parent and the seam between them), segment indices and never-live flags carried through, and a reason for every refusal (ring sizes differ, node counts differ, ring already used, a part attached twice, a cycle). Output: one craft (nodes, beams, panels, thrusters, segments, seams, tree) that compiles to commands.
  **Test:** the hopper assembles from its seven parts; node count is the sum of the parts minus the merged nodes, mass is the sum of the parts exactly, merged nodes sit within 1e-9 m of where both parts put them, the child's ring beams are gone and its seam beams are marked; each clock gives the expected rotation of a marked node; every refusal case is refused with its reason and changes nothing; the hopper stands on flat ground for 20 s with no yield and comes to rest, within the substep budget.
  Demo: skip.

- [ ] **4.5 Craft file.** Build with: Sonnet.
  `src/parts/craft-file.ts`: save and load as in "Settled", and `test/fixtures/hopper.craft.json` committed. Ticks the roadmap check "parts join at attachment rings into one craft, saved and loaded as a craft file".
  **Test:** load then save gives the same text, byte for byte; a loaded hopper gives the same command bytes as the one assembled in code, and the same state hash after 600 steps on the landing field; a file with an unknown field, an unknown part type, a later version, a bad join or a non-finite number is refused with a reason.
  Demo: skip.

- [ ] **4.6 Thrusters in the core.** Build with: Sonnet.
  `core/src/thrust.rs` (or inside `world.rs`): the thruster arrays (tail, head, maximum, throttle, flat, one per field), the two commands from "Settled" in `core/src/command.rs` and `src/commands.ts`, the force pass through `add_force` before the step, thrusters in the state hash, `ABI_VERSION` 4, the recording decoder accepting 2, 3 and 4.
  **Test:** two free nodes with a thruster between them, no gravity, accelerate at `T / (m_a + m_b)` within rounding along the head direction; throttle 0 gives no force; a deleted node turns its thruster off; bad records (same node twice, dead node, non-finite or negative thrust, throttle outside 0 to 1, unknown thruster) are rejected and the rest of the batch applies; the committed `weak-bridge.rcrd` (ABI 2) and `tower-collapse.rcrd` (ABI 3) replay to their hashes; a thrusting scene hashes the same natively and in WebAssembly.
  Demo: skip.

- [ ] **4.7 Lift-off, headless.** Build with: Sonnet.
  The landing field in `src/terrain.ts`; a scenario module (`src/flight.ts`) that places a craft file on the pad and plays a throttle script as commands, step by step, like `liveBridgeCommands`. Ticks the roadmap check "engines produce thrust and the craft lifts off" (seen in 4.9).
  **Test** (`test/flight.test.ts`): the hopper rests on the pad; at full throttle from step 60 its pod's height gain after 3 s is within 5% of `½ (T/M − g) t²`, the craft tilts less than 5°, nothing yields; at a throttle just under hover (`T·throttle < M·g`) it stays on the pad; the same script run twice gives the same hash. `scene_stats` (or a Node timing script) reports nodes, beams, substeps, contacts and ms per step for the hopper standing and lifting off.
  Demo: skip (shown in 4.9).

- [ ] **4.8 Hard landing, tuned (test of fun).** Build with: **Opus** (the roadmap's feel-critical test; it sets the leg and landing numbers every later craft inherits).
  Drop tests (the craft placed above the ground at rest, so it touches down at a chosen speed) and one scripted hop: throttle up from the pad, a small sideways "nudge" thruster across the tank at about the centre of mass height carries it over the rough area, throttle down, cut, fall. Tune within the bands in ER 004: leg member areas, sprung-bay stiffness, crush force and stroke, foot spread, bump heights. A committed golden recording of the hop that snaps a leg (`test/fixtures/hard-landing.rcrd`, under 150 KB) replayed by a test and by `pnpm replay`.
  **Test** (`test/landing.test.ts`): standing 20 s on the pad: no yield; touchdown at 2 m/s on the pad: no yield; touchdown at 3 m/s on the pad and on each of five fixed spots of the rough area: no break, and the craft stands (tilt under 15°); touchdown at 7 m/s on the same five rough spots: in at least four, a leg snaps (its foot is no longer joined to the pod through live beams, checked by a connectivity walk in the test) and the craft tips over (engine-tail-to-pod-apex axis more than 60° from vertical, at rest); in every run no node goes below the ground by more than its radius plus 1 cm, no node is clamped, no island freezes, and everything comes to rest within 15 s of touchdown; the golden replays to its hash.
  Demo: skip here (shown in 4.9 and 4.11).

- [ ] **4.9 Craft page.** Build with: Sonnet.
  `web/craft.html` and `web/craft3d.ts`, reusing the milestone 3 modules (`src/render3d.ts` beams and picking, `src/stress-colour.ts`, `src/skin.ts` and `src/panels3d.ts` for tank, bell and pod panels, interpolation, the recorder and player). Where `web/viewer3d.ts` code is needed, move it into a shared module rather than copying it. The landing field, the hopper from its craft file, a parts gallery (each default part on its own), a camera that follows the craft, throttle keys (step up and down, full, cut) recorded as `CMD_SET_THROTTLE`, the scripted hop and drop buttons from 4.8, time controls, Save and Load for recordings and for craft files.
  **Test:** in Node, the key-to-command mapping and the follow camera maths; in the browser (headless), after the scripted hop the page shows the same hash as Node; no console errors. By eye: the hopper lifts off on full throttle, and the hard landing snaps a leg and tips it over, with stress colours on the snapping leg.
  Demo: lift-off and the hard landing, with a slow-motion stress view of the snap (the slice demo; it becomes the milestone demo in 4.11).

- [ ] **4.10 Assembly panel.** Build with: Sonnet.
  A small panel on the craft page: pick a part type and its parameters, click a free ring or mount patch to attach it, turn it by clock steps, remove the last part; refusals shown with their reasons; the craft saved and loaded as a craft file. Parts appear placed one after another (render only).
  **Test:** in Node, the panel's actions produce the same craft file as building the hopper in code; in the browser (headless), assembling the hopper by clicks and flying it gives the same hash as the fixture.
  Demo: assembling the hopper part by part, then launching it.

- [ ] **4.11 Milestone wrap.** Build with: Sonnet.
  Timing of the hopper standing, lifting off and crashing (nodes, beams, contacts, substeps, ms per step, contact share, as ER 003's table), the milestone demo from the [demo plan](/docs/research/demo-and-media-plan.md) ("assemble a rocket from parts, launch, then the hard landing that snaps a leg, with a slow-motion stress view of the snap"), recorded headless, the roadmap percentages, and the record closed with what milestone 5 should know (how much the engine and pod lattices wobble, the mass ratio at seams, and whether the segment and seam data are enough for sleeping).

## Not in this milestone

- Rigid bodies, riding nodes, joints, servos, sleeping and splitting into vessels (milestone 5). The engine and pod are lattices flagged never-live; a broken-off leg stays part of the same island and world, with no vessel bookkeeping.
- Leg deployment, hinges and actuators (milestone 5).
- Lift and drag (milestone 6). No air at all: a falling craft only has gravity.
- Fuel flow and changing mass, staging, decouplers, steering and gimbals.
- Craft-against-craft and part-against-part contact (milestone 8). A broken leg can pass through the tank.
- Beam-bridged joins, struts between parts and docking rings.
- A damage layer in the craft file (wrecks are not saved).
- A floating origin.

## Open questions

- [ ] Mini-game or test bed? Still assumed test bed (Plan 002).
- [ ] Fuel and changing mass: a tank that drains changes the hopper's thrust to weight through a flight. Left out here; wanted before milestone 6's planes, or later?
- [ ] Is one stiffness scale for every material right once rigid bodies arrive (milestone 5), or should sleeping segments make real stiffness affordable? Logged as an oracle candidate in ER 004, not asked.
