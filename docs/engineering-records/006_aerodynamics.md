---
type: EngineeringRecord
title: Aerodynamics
description: Decisions, findings and verification evidence for milestone 6.
timestamp: 2026-11-19T12:00:00.000Z
plans: [/docs/plans/006_aerodynamics.md]
tags: [milestone-6, aerodynamics, wings, flutter, sleeping]
---

# ER 006: Aerodynamics

## 2026-11-19 (planning)

- **[action]** Milestone 6 started. Plan 006 written by Opus from the roadmap's rewritten milestone 6 checks, the spec, the core layout rules and ER 005's closing section, against the code as it stands. Phases: 6.1 air settings, environment channel, never-sleep flag; 6.2 body stations; 6.3 automatic stations on the existing parts; 6.4 wing strips; 6.5 plates and deflection; 6.6 the wing part; 6.7 air does not wake, flight does not sleep; 6.8 stability and flutter; 6.9 reference plane and the comparison with the game model; 6.10 the plane flies and is steered; 6.11 the wing flexes and snaps; 6.12 the page; 6.13 wrap. 6.1 to 6.11 are headless.
- **[decision]** Who builds what: **Opus** for 6.7 (the wake pass and the equivalence test), 6.8 (flutter and the stability of explicit air), 6.10 (dynamic stability of a live plane) and 6.11 (the wing-snap window). **Sonnet** for 6.1 to 6.6, 6.9, 6.12 and 6.13. Mechanical follow-ups to Haiku. This matches the roadmap row ("Sonnet; Opus for flutter and stability").
- **[decision]** Roadmap check to phase: check 1 (elements with automatic fallback) in 6.5 with 6.3; check 2 (plane flies and is steered) in 6.10; check 3 (air does not wake, flight never sleeps) in 6.7; check 4 (wing flexes and snaps) in 6.11. Each is a quarter of the milestone.
- **[decision]** **ROADMAP.md is not edited by this planning commit**, as in milestones 4 and 5. The plan link on the milestone 6 heading goes in 6.1's commit. The plans and records index tables are tool-managed and are not hand-edited here.

### Architecture (option E)

- **[decision]** **Each part generator declares aero elements with node weights**, beside the segments and seams it already emits: wing strips (lift with stall, induced drag), body stations (slender-body lift, crossflow drag, base pressure), plates (airbrakes, flaps). Chosen over per-triangle drag after two earlier Opus reviews: per-triangle pressure gets induced drag, stall and slender-body lift wrong without a panel method, and its cost scales with skin detail. Triangles are kept for buoyancy (milestone 7), heating and hypersonic pressure (later).
- **[decision]** **Anything undeclared gets an automatic body station** from its segment's bounding shape, made in the assembler, so legs, struts and wreckage always get drag and the core sees only declared elements.
- **[decision]** **Air is evaluated in the core**, once per 60 Hz step in ascending element order, from the step's start positions and velocities, own `sin`, `cos`, `atan2` and a new own `exp`. Per-step like thrust: no substep or frame-time dependence. Host-side air was rejected: it would cross the boundary every step and could not see rider velocities cheaply.
- **[decision]** **Control surfaces are deflections on strips** (`CMD_SET_AERO_DEFLECTION`), not hinged lattices, to steer the plane with no new joint work. A servo-driven flap stays possible with milestone 5's joints.
- **[decision]** **Formulas are ported from a consuming game's analytic model as a reference only** (no code copied, no dependency): Prandtl-Glauert subsonic inside a DATCOM lift slope, a stall blend from 14° to 24° into flat-plate coefficients, induced drag `C_L² / (π e A)`, plates, crossflow. Written out in the plan's "Settled" so a session without the game can build them. All coefficients are **[assumption]** until 6.9.
- **[decision]** **Agreement test.** The game keeps its own model for autopilot and path prediction, so for a stiff craft ribcage must agree with it on lift, drag and trim. 6.9 compares the core against `src/aero-reference.ts` (the same formulas on the undeformed craft) and, when the owner supplies them, against numbers from the game's model for the same reference craft (a numbers-only fixture). Until then that check is open, not passed.
- **[decision]** **Later passes are listed, not planned:** Mach regimes past a stub (hold `β` at `M = 0.7`, count elements above 0.85), wing interference and downwash, ground effect, per-triangle pressure.

### Verified in the code before relying on it

- **[finding]** **The force-jump wake** is the "applied forces" loop in `World::sleep_before_solve` in `core/src/sleep.rs` (around line 534): per node it compares `force` with `sleep.pushed` and wakes the body when the change exceeds the node's weight. `step_inner` in `core/src/sim.rs` calls `sleep_before_solve` before `apply_thrust`, so the loop sees only host-commanded forces. A core air pass placed after thrust would not trip it by order alone; the plan still adds `add_env_force` and `env_force` so the rule is a named contract with a test and holds for later environment sources.
- **[finding]** **Rider forces reach the body.** In `core/src/island.rs`, island packing (around line 330) gives a rider node inverse mass zero and adds its `force` to the body's force and `(pos - centre) × force` to its torque. Air on sleeping or never-live segments therefore acts on the body with no new code.
- **[finding]** **The stress estimate sees external forces.** `estimate::take_forces` copies `world.force` after `apply_thrust` and before the islands are built (`core/src/sim.rs`, lines 63 to 66), and `core/src/estimate.rs` subtracts `m g + f + J / dt` per node in its cut. Air applied between `apply_thrust` and `take_forces` therefore flows into every seam's estimated load. 6.2 tests it.
- **[finding]** **No never-sleep flag exists.** `core/src/segment.rs` has only `SEG_NEVER_LIVE` (value 1, always rigid, the opposite of never-sleep) in `SEG_FLAGS_KNOWN`; the only rule that keeps a segment live is the torn-segment rule in `sleep.rs`. ER 005's closing note ("a segment flag exists") was mistaken on this point. 6.1 adds `SEG_NEVER_SLEEP` (value 2).
- **[finding]** No `exp`, `acos` or `asin` in `core/src/maths.rs`; `sin`, `cos`, `sin_cos` and `atan2` exist. Angles are taken with `atan2`; 6.1 adds `exp` for density.
- **[finding]** `ABI_VERSION` is 10 (`core/src/ffi.rs`, `src/core.ts`); command kinds run to 18 (`CMD_BREAK_BEAM`). No wing generator exists in `src/parts/`. Gravity is flat (`gravity_at` in `sim.rs`).

### Sleeping and flight

- **[decision]** **A craft in flight never sleeps:** a vessel with no node in contact range and an element dynamic pressure above 50 Pa **[assumption]** wakes once through the existing wake pass and does not sleep until it is in contact or slow again. Wing segments also carry `SEG_NEVER_SLEEP` from the generator, as ER 005 advised, until 6.11 measures. Standing craft (the pad, the clamp) still sleep. **No threshold in `sleep.rs` is tuned in this milestone.**
- **[decision]** Air loads wake segments only through the estimate reading, never through the force-jump trigger. Decided against the stated goals (performant, semi-realistic, deterministic): a steady stream on a parked craft is not news, and the estimate already sees it.

### Determinism and the goldens

- **[decision]** Air settings and elements enter the state hash only once one exists. The committed recordings send no air command and must replay unchanged at every phase. From 6.3 flight scenarios compile with `aero: true` and `hard-landing-rigid.rcrd` is re-recorded deliberately and logged; `aero: false` keeps the hop at `0x8e49fe2d420d03e2`, and `rigid: false` the all-live `0xd92f2aad01c0e048`.
- **[decision]** ABI bumps per phase that adds command kinds (11 in 6.1), mirrored in `src/commands.ts`, the decoder accepting every earlier ABI.

### ER 004's open note

- **[decision]** ER 004 (phase 4.8) relaxed the rest criterion because "the core has no drag", and fragments on a hinge swing for ever. 6.3 re-runs that window with air on and closes the note with numbers. Expected: quadratic drag at 0.1 to 0.3 m/s is far too small to settle a 50 kg fragment, so the likely close is "not air's job" with joint or hinge damping named for a later phase. Stated here so the outcome is not read as a failure of the air model.

### Open, to be measured

- **[assumption]** The coefficients in the plan's formulas, the 50 Pa flight level, the aero-rate budget of 0.5 at 150 m/s, the wing's substep count staying at or under 19, the plane's size and cruise speed.
- **[assumption]** Explicit per-step air is stable on live wings below the dive speed. 6.8 decides; it is the biggest risk of the milestone.
