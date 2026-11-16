# ribcage roadmap

**TL;DR.** Overall progress: **9%**. Now: milestone 1 (solver core and tools). First playable slice ends at milestone 4.

This page is the single answer to "what are we building, and how far along is it?". The design is the [spec](docs/research/node-beam-engine-design-spec.md). The detailed work for the current milestone lives in its plan.

## What we are building

A real-time node-beam physics engine for a KSP-like building game. Rockets, planes, rovers, buildings and bridges are built from parts, and they flex, bend and break under load. Rust core compiled to WebAssembly, drawn by Three.js in the browser.

**Version 1 is done when** a player can assemble a craft from parts, fly or drive it over ground and water, crash it into terrain or another craft, and watch it bend and tear apart along its seams, at a steady frame rate, with the same inputs always giving the same result.

**Not in version 1:** multiplayer, heat damage, gameplay-driving ball fluids, the game around the engine (career, missions, UI beyond a test bed).

## Progress

Each milestone has a weight (its share of the whole) and a short list of checks. A check is something that can be seen or tested, never "code written". Milestone progress is ticked checks divided by total checks. Overall progress is the weighted sum.

| # | Milestone | Weight | Done | Status |
| --- | --- | --- | --- | --- |
| 0 | Scaffold | 2 | 100% | done |
| 1 | Solver core and tools | 12 | 57% | **now** |
| 2 | 2D bridge builder | 8 | 0% | next |
| 3 | 3D on terrain in Three.js | 10 | 0% | |
| 4 | Parts and assembly (first playable slice) | 12 | 0% | |
| 5 | Rigid by default | 16 | 0% | |
| 6 | Aerodynamics | 10 | 0% | |
| 7 | Ground and water | 10 | 0% | |
| 8 | Craft-against-craft collision and threads | 12 | 0% | |
| 9 | Visual layer | 8 | 0% | |
| | **Overall** | **100** | **9%** | |

## Who plans and builds what

A first guess, corrected as each milestone starts. "Plan" means writing that milestone's plan from the spec. Mechanical work in every milestone (ticking checks, indexes, test scaffolds, renames) goes to Haiku.

| # | Milestone | Plan with | Build with | Why |
| --- | --- | --- | --- | --- |
| 1 | Solver core and tools | planned (Fable) | Opus for the solver and cantilever; Sonnet for layout, hash, benchmark | Numerics that everything else rests on |
| 2 | 2D bridge builder | Sonnet | Sonnet | Known patterns: WebAssembly wiring and a small editor |
| 3 | 3D on terrain | Opus | Sonnet; Opus for mesh skinning | One hard part (skinning), the rest is porting |
| 4 | Parts and assembly | Opus | Sonnet for generators; Opus to tune the hard landing | Many small generators, one feel-critical test |
| 5 | Rigid by default | [brief](docs/research/rigid-by-default-design-brief.md) done (Fable); phases by Opus | Opus | Hardest design: switching without adding energy, merging, splitting |
| 6 | Aerodynamics | Opus | Sonnet; Opus for flutter and stability | Formulas are known, stability is not |
| 7 | Ground and water | Opus | Sonnet | Several independent, well-described features |
| 8 | Collision and threads | [brief](docs/research/collision-and-threads-design-brief.md) done (Fable); phases by Opus | Opus | Threads must give identical results; costly to redo |
| 9 | Visual layer | Sonnet | Sonnet | Visual only, judged by eye |

Every milestone and every worthwhile slice also gets a demo: see the [demo and media plan](docs/research/demo-and-media-plan.md).

Planning has two stages. The costly-to-redo milestones (5 and 8) have an early design brief, and milestone 1 follows the [core layout rules](docs/research/core-data-layout.md) those briefs produced. Every milestone gets its detailed phase plan when it starts. No Fable planning remains nominated; a new nomination needs the owner's approval.

## Milestone checks

### 0. Scaffold

- [x] Repository, Rust core crate and TypeScript host exist, and all test commands pass.

### 1. Solver core and tools ([Plan 001](docs/plans/001_bootstrap.md))

- [x] Nodes and beams live in flat arrays and step at a fixed 60 Hz.
- [x] The same inputs give an identical state hash on every run.
- [x] A cantilever's tip deflection matches the textbook formula within a stated tolerance. (Within 2% with enough substeps; at the rule's count softer by a measured `0.4 · c²`, see ER 001, phase 1.4.)
- [x] A beam loaded below yield returns to rest; past yield it stays bent; past break it is removed and reports an event. (Tests in `core/tests/failure.rs`; see ER 001, phase 1.5.)
- [ ] A resting structure gains no energy over ten simulated minutes.
- [ ] A bad number freezes the structure and reports, instead of spreading.
- [ ] A benchmark scene reports beams, substeps and milliseconds per step.

### 2. 2D bridge builder

- [ ] The core runs in the browser as WebAssembly, stepped from TypeScript.
- [ ] You can place nodes and beams, press run, and pause, single-step and slow down time.
- [ ] A load rolls across the bridge and an under-built bridge breaks.
- [ ] Beams are coloured by stress.
- [ ] A recorded run replays exactly.

### 3. 3D on terrain in Three.js

- [ ] A tower and a bridge stand on height-map terrain with friction.
- [ ] Both collapse believably when overloaded or knocked.
- [ ] Meshes are skinned to the nodes and bend with them.
- [ ] The stress colours and time controls work in 3D.

### 4. Parts and assembly (first playable slice)

- [ ] Tank, truss, landing leg, engine and pod are generated from parameters.
- [ ] Parts join at attachment rings into one craft, saved and loaded as a craft file.
- [ ] Engines produce thrust and the craft lifts off.
- [ ] **Test of fun:** a hard landing on rough terrain snaps a leg and the craft tips over.

### 5. Rigid by default

- [ ] Rigid bodies with anchors carry engines, pods and wheels.
- [ ] Hinges, joint limits, servos and rotors work, and a stalled servo adds no energy.
- [ ] Segments go rigid when barely loaded and wake under load, with momentum conserved across every switch.
- [ ] Touching rigid segments merge into one body, and a tall rocket visibly wobbles less than with everything live.
- [ ] A craft cut in two becomes two vessels with the correct total mass.

### 6. Aerodynamics

- [ ] Lift and drag act per skin triangle, and a simple plane flies and can be steered.
- [ ] A wing flexes under load and snaps in a hard turn.

### 7. Ground and water

- [ ] Rock, sand, mud and snow feel different, and things sink into soft ground.
- [ ] Wheels and skids leave ruts and craters that stay.
- [ ] Craft float on waves with buoyancy and drag, and a breached hull floods.

### 8. Craft-against-craft collision and threads

- [ ] Two craft collide and damage each other.
- [ ] Docked or spawned parts never explode apart.
- [ ] Islands solve on worker threads with the same result as single-threaded, with a single-thread fallback.
- [ ] The benchmark meets the performance budget (budget still to be set).

### 9. Visual layer

- [ ] Meshes tear along seams when the seam beams break, with particles covering the gap.
- [ ] Small wreckage becomes GPU debris.
- [ ] Ball-water splashes, wakes and dust react to the craft (visual only).

## How this page is kept

- When a check is proven, tick it and update the two percentages in the table, in the same commit as the work.
- When a milestone starts, it gets its own plan and engineering record; only the current milestone is planned in detail.
- A new idea goes to one of three places: a check in an existing milestone (it is part of version 1), the "Later" list (it is not), or it replaces something (say what it replaces). Weights are re-balanced only when a milestone is added or removed, with a one-line reason in the engineering record.
- Progress reports use the overall number from this page.

## Later (not in version 1)

- Multiplayer (the north star; no decision here should block it).
- Heat: strength falls with temperature, for re-entry and engine fires.
- Ball fluids that drive gameplay in small contained places.
- Detail levels for distant craft.
- Sequencer and walker-brain controller parts.

## Open questions

Kept in the [spec](docs/research/node-beam-engine-design-spec.md#open-questions). The ones with a deadline:

- Performance budget: needed before milestone 8's last check, useful from milestone 1's benchmark.
- Is the 2D bridge builder a shippable mini-game or a test bed: needed at the start of milestone 2.
- Hosting and the headers threads need: needed before milestone 8.
- Multiplayer model: after version 1, but it sets how strict determinism must stay.
