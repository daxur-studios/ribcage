---
type: Plan
title: 3D on terrain
description: Milestone 3 - height-map terrain with friction in the core, a tower and a bridge that stand and collapse on it, drawn in Three.js with meshes skinned to the nodes.
timestamp: 2026-11-17T18:00:00.000Z
status: active
record: /docs/engineering-records/003_terrain_3d.md
tags: [milestone-3, terrain, contact, three, skinning]
---

# Plan 003: 3D on terrain

Evidence: [ER 003](/docs/engineering-records/003_terrain_3d.md). Design: [spec](/docs/research/node-beam-engine-design-spec.md) (collision stage 1, terrain, rendering), the [collision-and-threads brief](/docs/research/collision-and-threads-design-brief.md) (its contact model and its milestone 3 row are binding here), and the [core layout rules](/docs/research/core-data-layout.md) (still binding, rules 5, 6 and 8 above all).

**What this milestone adds.** The core gets ground: a height-map terrain and a push-only ground contact with friction, the first stage of the spec's collision. The host gets a Three.js page that shows a tower and a bridge standing on that terrain, collapsing when overloaded or knocked, with stress colours, time controls, record and replay, and panels skinned to the nodes. Everything else (parts, rigid bodies, craft-against-craft contact, surface types, tearing) belongs to later milestones and is not started here.

**Scope [assumption]:** still a test bed (Plan 002). The 3D page has scene and action buttons and the debug tools, no game around them.

## Settled before building

These are decided and logged in [ER 003](/docs/engineering-records/003_terrain_3d.md) (first dated section). A builder follows them; changing one is a logged decision with a reason, not a silent edit.

- **Axes.** `y` is up (gravity is `-y`, as now). The terrain is a height `y = h(x, z)`.
- **Terrain is a grid of heights,** `nx × nz` points, `cell` metres apart, from an origin `(x0, z0)`. Each cell is two triangles split along the diagonal from `(i, j)` to `(i+1, j+1)`, so the surface is flat triangles with an exact plane each, and the renderer draws the very same triangles. Outside the grid the edge heights continue flat. At most 257 × 257 points **[assumption]**. No terrain set means no ground (milestone 2 scenes are unchanged).
- **Terrain arrives as commands** (layout rule 8): one record sets the grid size, origin, cell size and friction (heights all zero), and further records each write six heights. A recording therefore holds its terrain, and "the commands are the whole input" stays true. Heights are in the state hash.
- **Contacts follow the collision brief.** Found once per 60 Hz step per island, before the solve, with a margin; each contact keeps the plane of the triangle under its point for the whole step; depth is re-measured against that plane every substep. The constraint only pushes, is rigid (no compliance), removes old overlap no faster than 2 m/s (the recovery cap), and has Coulomb friction with one coefficient (static equals kinetic) applied as a position correction. No bounce. Contacts are solved after the beams in each substep.
- **One correction kernel** (layout rule 6). The maths that moves endpoints by inverse mass along a direction is pulled out of `solve_distance` into one function; beams and contacts both call it. Beam results must stay bit-identical (the golden recording and the benchmark hash guard it).
- **Beam midpoints touch the ground too** (spec, collision stage 1), as an endpoint that is a weighted mix of the beam's two nodes. This is the "point along a beam" endpoint milestone 8 needs, built once here.
- **Renderer:** Three.js, pinned to one exact version, vendored from `node_modules` into `dist/` by the build script and loaded with an import map. Still no bundler. `WebGPURenderer` (with its built-in WebGL 2 fallback) and TSL for the skinning shader, because the spec puts TSL in milestone 9 and one renderer should last the project. **[assumption]** Check the pinned version's own docs for import paths and TSL names when phase 3.5 starts.
- **Skinned meshes are per-cell panels** (deck slabs, floor slabs, wall panels), each embedded in the nodes of its own lattice cell. Panels then part at cell edges when a structure comes apart, which looks right without the seam tearing of milestone 9.
- **Materials for both scenes** reuse milestone 2's strong member (`EA` 2e8 N, 100 kg per node, damping 5000 N·s/m, yield strain 0.00125, break strain 0.01), which stays under the 32-substep cap. Node radius 0.15 m **[assumption]**.

## Phases

Each phase ends with passing tests (`cargo test`, `pnpm test`, `pnpm typecheck`), ticks the matching check in the [roadmap](/ROADMAP.md) if it proves one, and logs a dated section in the record. "Build with" is the model for that session; mechanical follow-ups (ticks, indexes, renames) can go to Haiku.

Order: 3.1 → 3.2 → 3.3 → 3.4 → 3.5, then 3.6 and 3.8 can run side by side (different files), then 3.7, then 3.9.

- [x] **3.1 Terrain in the core.** Build with: Sonnet.
  `core/src/terrain.rs`: the grid, `height_at(x, z)` and `plane_at(x, z)` (the triangle's unit normal and offset) with the fixed diagonal and the flat edge rule; only `+ − × ÷`, `sqrt` and `floor` (exact by IEEE 754; not on the lint's list). The terrain lives in `World` as read-only data. Two command kinds, `CMD_SET_TERRAIN` and `CMD_TERRAIN_HEIGHTS` (start index, six heights; rejected past the grid end or for a non-finite height), mirrored in `src/commands.ts`. Heights and grid size join the state hash. `ABI_VERSION` 3; the recording decoder also accepts ABI 2 files, because ABI 3 only adds command kinds.
  **Test:** heights and normals are exact on a tilted plane and at grid points, the diagonal split is the documented one, outside the grid is the edge height; bad terrain records are rejected and the rest of the batch applies; the committed `weak-bridge.rcrd` (ABI 2) still replays to `0xb580a6d8faa6610d`; a terrain-only world hashes the same natively and in WebAssembly (`core/tests/ffi.rs`, `test/wasm.test.ts`).
  Demo: skip (nothing to see yet).

- [x] **3.2 Ground contact and friction.** Build with: **Opus** (contact and friction decide how everything rests, slides and lands, and milestone 8 reuses them; costly to redo).
  The shared correction kernel (layout rule 6), with the beam path bit-identical. Contact finding per island at the start of the step (a pure function of the island buffer and the terrain, so it can move to a thread later): every movable node whose gap to the ground is under the margin `|v|·Δt + ½·g·Δt² + 0.01 m` gets a contact holding the plane under it. Contact arrays live in the island buffer, in ascending packed node order. Per substep, after the beams: push each contact out along its normal to `radius` above the plane, but leave any overlap that was already there at the substep's start shrinking by no more than `2 m/s · h`; then cancel the sideways slip of that substep, fully if it is within `μ` times the push, otherwise by `μ` times the push. Pinned nodes are skipped. `StepStats` gains a contact count.
  **Test** (`core/tests/ground.rs`): a dropped node lands at `radius` above flat ground without bounce or sinking (within 1 mm) and stays there; a node spawned 0.5 m deep rises at no more than 2 m/s; a small lattice cube on a slope stays put (drift under 1 mm in 10 s) when `tan θ < μ` and slides with acceleration `g (sin θ − μ cos θ)` within 5% when `tan θ > μ`; a structure resting on the ground for ten simulated minutes gains no energy (as `core/tests/energy.rs`) and does not creep; the benchmark hash and the golden recording are unchanged; a small terrain scene hashes the same natively and in WebAssembly.
  Demo: a cube sticking on a gentle slope beside one sliding on a steep one, from a trace (the 1.2b player is enough).

- [x] **3.3 Beam midpoints on the ground.** Build with: Sonnet.
  The weighted endpoint: a point `b_a · p_a + b_b · p_b` whose resistance to a push is `b_a²·w_a + b_b²·w_b`, each node moving by `b_i · w_i · Δλ` (the kernel from 3.2 takes it; no second copy of the maths). Each live beam's midpoint (weights ½, ½) is checked against the ground like a node, with the smaller of its two end radii, after all node contacts in the list (ascending beam order).
  **Test:** a long beam dropped across a one-cell ridge rests on the ridge instead of passing through it; a beam dropped flat on flat ground lands level (its two ends within 1 mm of each other in height) and its total momentum change matches the ground impulse; 3.2's tests still pass.
  Demo: skip, or the ridge drop if it is quick.

- [x] **3.4 Tower and bridge on terrain, headless.** Build with: Sonnet.
  `src/terrain.ts`: a terrain generator using only arithmetic (so every engine gives the same heights): a flat plain for the tower and a valley with two flat ledges for the bridge, with gentle bumps. `src/scenes3d.ts`: a tower (square lattice, braced faces and floors, feet resting on the ground, not pinned) and a 3D bridge (two Warren trusses with cross bracing and a deck, its ends resting on the ledges, not pinned), each as command lists like `src/design.ts`. Actions as commands: knock (a large sideways force on chosen nodes for a short time), remove a column (delete beams), overload (a heavy roof load on the tower; the rolling truck from milestone 2 on the bridge deck, its weight split over the four deck nodes around it).
  **Test** (`test/scenes3d.test.ts`): both stand for 20 s with no yield or break and come to rest (every node slower than 1 mm/s); each collapses under each action it is meant to (break events, then everything comes to rest within a set time); during a collapse no node goes below the ground by more than its radius plus 1 cm, no node is clamped and no island freezes; the same lists run twice give the same hash. Roadmap check "a tower and a bridge stand on height-map terrain with friction" can be ticked here (seen in 3.5).
  Demo: skip (shown in 3.5 and 3.6).

- [x] **3.5 Three.js page.** Build with: Sonnet.
  `three` and `@types/three` at one exact version; `scripts/build-web.mjs` copies the needed module files into `dist/web/vendor/`; `web/terrain.html` (import map) and `web/viewer3d.ts`. Terrain drawn from the same triangles as the core. Beams as one instanced cylinder mesh, coloured by stress with the 2D editor's scale (moved to a shared `src/stress-colour.ts`, the 2D editor using it too), broken beams hidden; nodes as optional small spheres; orbit camera; a scene picker; Run and Stop. The 2D editor stays as it is.
  **Test:** the pure parts in Node (beam instance transforms from two end points, colour scale shared with 2D); in the browser both scenes stand on the terrain, and after a fixed number of steps the page shows the same hash as Node; no console errors.
  Demo: an orbit around the tower and the bridge standing on the terrain.

- [x] **3.6 Time controls and actions in 3D.** Build with: Sonnet.
  Pause, single step, the five speeds and the step clock from the 2D editor. Drawing interpolates between the last two render buffers (render only; the core is never stepped by frame time). The actions from 3.4 on buttons, with knock aimed by clicking a node (a ray pick against node spheres).
  **Test:** interpolation and picking maths in Node; by hand: each action collapses its scene, speeds match their step rates as in ER 002, and stress colours change as load builds. Ticks the roadmap checks "both collapse believably when overloaded or knocked" (with 3.4's tests) and "the stress colours and time controls work in 3D".
  Demo: the tower knocked over and the bridge failing under the truck, at ¼ speed.

- [x] **3.7 Record and replay in 3D.** Build with: Sonnet.
  The 2.4 recorder and player on the 3D page (Save, Load, Replay sample), terrain included through its commands. A committed golden recording of one collapse (`test/fixtures/tower-collapse.rcrd`, under 150 KB, or a smaller scene if not) replayed by a test and by `pnpm replay`.
  **Test:** the golden replays to its hash in Node and in the browser; a changed command is caught at the next checkpoint.
  Demo: the same collapse twice side by side, identical debris.

- [x] **3.8 Meshes skinned to the nodes.** Build with: **Opus** (the roadmap's one hard part; it is judged by eye and decides how every later part looks).
  Per-cell panels on both scenes. Each panel vertex is bound to four nodes of its cell that are not in one plane, as barycentric weights (exact for any rigid motion and any uniform stretch of that cell). Node positions go to a float texture once per frame; a TSL vertex function places each vertex from its four nodes, and turns the normal by the cofactor of the cell's deformation. The binding is computed in TypeScript (`src/skin.ts`) and tested in Node; the shader only repeats it.
  **Test:** in Node: the rest pose reproduces the mesh exactly, a rotated and moved lattice gives the rotated and moved mesh (within 1e-6 m), a stretched one the stretched mesh; a CPU reference of the shader maths agrees with `src/skin.ts`. By eye: panels bend with the bridge under the truck and tilt and fall with their cells as the tower collapses, with no panel left floating away from its cell. Ticks the roadmap check "meshes are skinned to the nodes and bend with them".
  Demo: the bridge deck flexing under the truck, close up, beams hidden.

- [x] **3.9 Milestone wrap.** Build with: Sonnet.
  Browser timing of both scenes (nodes, beams, contacts, substeps, ms per step), the milestone demo from the [demo plan](/docs/research/demo-and-media-plan.md) (a tower and a bridge collapse from two camera angles, the same collapse twice), the roadmap percentages, and the record closed with what milestone 4 should know (contact cost, anything that looked wrong).

## Not in this milestone

- Node against node, beam against beam and craft against craft contact (milestone 8). Debris from one structure passes through itself and through the other structure. Noted, not fixed.
- Surface types, sinking into soft ground, ruts and craters (milestone 7). One friction coefficient per terrain.
- Mesh tearing along seams, particles and debris (milestone 9).
- A floating origin. Scenes stay within a few hundred metres of the world origin, where 32-bit render positions are still finer than a tenth of a millimetre.

## Open questions

- [ ] Mini-game or test bed? Still assumed test bed (Plan 002).
- [ ] Does a resting pile of fallen beams creep or jitter with one contact pass per substep (brief, trap 5)? 3.2's resting tests and 3.4's "comes to rest" tests are the first look; if they fail, the cure (a second contact pass, or a larger skin) goes back to an Opus session on the contact code.
