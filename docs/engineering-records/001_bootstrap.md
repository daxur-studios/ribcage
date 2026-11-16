---
type: EngineeringRecord
title: Bootstrap ribcage
description: Initial scaffold provenance and verification log for ribcage.
timestamp: 2026-11-16T19:05:47.257Z
plans: [/docs/plans/001_bootstrap.md]
tags: [bootstrap, setup]
---

# ER 001: Bootstrap ribcage

## 2026-11-16

- **[action]** Created ribcage with `ae repo new` and generated initial `ae.repo.json` alignment.
- **[action]** Added Cargo workspace with `core/` (`beam-core`), and saved the design spec under `docs/research/`.
- **[decision]** Used the `node-ts` profile for the TypeScript host; the Rust core is added by hand because `ae repo new` has no Rust profile yet.
- **[verification]** `cargo test` passes (1 test).
- **[decision]** Added `ROADMAP.md` at the repo root as the one page for scope and overall progress: a definition of version 1, weighted milestones, and visible or testable checks per milestone. Overall percent is the weighted sum of ticked checks. Draft; weights and checks await owner review.
- **[decision]** Plan 001 now covers the scaffold and milestone 1 only, split into phases. Milestones 2-9 live in the roadmap and each gets its own plan and record when it starts.
- **[question]** Is the core 3D from the first line (the 2D bridge builder pins everything flat), or generic over dimension as the spec's architecture section implies? Recommended: 3D from the start, so milestone 3 is not a partial rewrite. Blocks milestone 1 phase 1.
- **[decision]** The core is 3D from the first line. The 2D bridge builder runs the same core with everything in one plane. Owner approved. Reason: milestone 3 is then a rendering and terrain job, not a partial solver rewrite.
- **[decision]** Planning happens in two stages. A design brief is written early for the milestones that are costly to redo (5 and 8), covering design, traps, tests and the rules they impose on earlier milestones. The detailed phase plan is written when the milestone starts, against the real code.
- **[action]** Wrote three research documents: `core-data-layout.md` (binding rules for milestone 1), `rigid-by-default-design-brief.md` (milestone 5) and `collision-and-threads-design-brief.md` (milestone 8). Each marks its assumptions.
- **[finding]** The briefs changed four things about earlier milestones: the solver works on packed per-island buffers and never on the world arrays; a beam endpoint is always a node, and a node may ride on a rigid body; generators must emit segment, seam and surface data; and the wake threshold should sit near 25% of yield, below the spec's 30-50%, because a suddenly applied load peaks at about twice its steady value.
- **[question]** Milestone 4 needs a rigid engine and pod, but rigid bodies arrive in milestone 5. Recommended: move the small "rigid body with anchor nodes" piece to the start of milestone 4. Needs owner approval because it changes the roadmap.
- **[finding]** Riskiest milestone 1 item is the cantilever check: a point-mass lattice only approximates a textbook beam, and XPBD stiffness depends on substeps, so the tolerance has to be measured, not assumed.

## 2026-11-16 (phase 1.1)

- **[action]** Phase 1.1 done: `Real` (64-bit) and `Vec3` in `real.rs`; flat-array `World` with stable `u32` indices, generation handles, lowest-first slot reuse and `add_force` in `world.rs`; packed single-island buffer with `build_islands` and `copy_back` in `island.rs`. The old per-node struct and `f32` step constant were replaced to follow layout rules 2 and 3. `cargo test` passes (2 tests). No roadmap check is proven yet; the first needs the fixed step (1.2).
- **[action]** Phase 1.2 done: `sim.rs` has the fixed step (gravity and gathered forces, 4 placeholder substeps, forces cleared, integer step counter) and an FNV-1a state hash over raw bits. Tests prove same inputs give the same hash, different inputs differ, and free fall matches 0.5 g t². `cargo test` passes (5 tests). Roadmap: two milestone 1 checks ticked, milestone 1 at 29%, overall 5%.
- **[decision]** Added `docs/research/demo-and-media-plan.md`: every milestone and worthwhile slice gets a short honest demo, tagged `[demo]` in the record. Milestone 1 demos are small HTML pages replaying JSON traces dumped by the headless core; live wasm and Three.js demos start at milestone 2. Clips stay local under gitignored `artifacts/`. The host framework for milestone 2 is left open.
- **[decision]** Folded a Fable review of the demo plan into it: new step 1.2b (trace dump example plus one HTML player, half a session) before 1.3; stronger per-phase demos; trace format, time boxes, replay labels, scenario registry and bloopers added. Milestone 2 host recommended as a plain Three.js page in this repo, to be confirmed at milestone 2. Asked as a subagent question, not an oracle question.
- **[action]** Phase 1.2b done: `core/examples/trace.rs` (scenario registry, one scenario `hash-split`, versioned JSON trace v1 with header, per-step positions, strain, hash, energy, empty event and command lists) and `demo/player.html` (one loader function, side-by-side panes, scrubber, speed, hash colour, fixed "replay" label with core commit). Traces go to gitignored `artifacts/traces/`. Checked in the browser: runs are identical until step 60, then split by 1e-9 m and the hashes differ. `cargo test` passes (5 tests). Command: `cargo run -p beam-core --example trace -- hash-split`. [demo] hash-split pair, no clip recorded yet.
