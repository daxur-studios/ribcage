# ribcage roadmap

**TL;DR.** Overall progress: **59%**. Now: milestones 1 (solver core and tools), 2 (2D bridge builder) are complete, and so is milestone 3 (3D on terrain in Three.js): terrain, ground contact, a tower and a bridge that stand and collapse, the Three.js page with time controls, record and replay and panels skinned to the nodes (see [Plan 003](docs/plans/003_terrain_3d.md)). Milestone 4 (parts and assembly, [Plan 004](docs/plans/004_parts_and_assembly.md)) is complete: the hopper is assembled from parts, lifts off, and its hard landing snaps a leg on the craft page; the assembly panel puts it together by clicks to the same hash, and the demo is recorded. The first playable slice ends here. Milestone 5 (rigid by default, [Plan 005](docs/plans/005_rigid_by_default.md)) is 100% done on its checks (the milestone wrap, 5.13, remains; the page, 5.12, is done): segments, seams, the assembly tree and rigid bodies are in the core, hinges, limits, servos and rotors work with a stalled servo adding no energy, a four-wheeled rover rolls on its rotors (headless), and a read-only stress estimate per seam is validated against the true beam stress and barely loaded segments sleep and wake under load, on by default in flight, telling the same story as all-live in the equivalence test; a craft cut in two becomes two vessels with the right total mass, and a tall rocket on a clamp sleeps into one body and wobbles less under thrust.

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
| 1 | Solver core and tools | 12 | 100% | done |
| 2 | 2D bridge builder | 8 | 100% | done |
| 3 | 3D on terrain in Three.js | 10 | 100% | done |
| 4 | Parts and assembly (first playable slice) | 12 | 100% | done |
| 5 | Rigid by default | 16 | 100% | in progress |
| 6 | Aerodynamics | 10 | 0% | |
| 7 | Ground and water | 10 | 0% | |
| 8 | Craft-against-craft collision and threads | 12 | 0% | |
| 9 | Visual layer | 8 | 0% | |
| | **Overall** | **100** | **59%** | |

## Who plans and builds what

A first guess, corrected as each milestone starts. "Plan" means writing that milestone's plan from the spec. Mechanical work in every milestone (ticking checks, indexes, test scaffolds, renames) goes to Haiku.

| # | Milestone | Plan with | Build with | Why |
| --- | --- | --- | --- | --- |
| 1 | Solver core and tools | planned (Fable) | Opus for the solver and cantilever; Sonnet for layout, hash, benchmark | Numerics that everything else rests on |
| 2 | 2D bridge builder | Sonnet | Sonnet | Known patterns: WebAssembly wiring and a small editor |
| 3 | 3D on terrain | planned (Opus) | Sonnet; Opus for ground contact and mesh skinning | Contact is reused by milestone 8; skinning is judged by eye; the rest is porting |
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
- [x] A beam loaded below yield returns to rest, even when the load arrives suddenly; past its yield force it stays bent and carries no more; past break it is flagged broken (it keeps its slot and stops carrying load) and reports an event. (Tests in `core/tests/failure.rs`; see ER 001, phase 1.5 and its damping fix.)
- [x] A resting structure gains no energy over ten simulated minutes. (After settling, rises stay at rounding level, 8e-14 J on 24 J; `core/tests/energy.rs`, see ER 001, phase 1.6.)
- [x] A bad number freezes the structure and reports, instead of spreading. (Per island; other islands stay bit-identical; `core/tests/guards.rs`.)
- [x] A benchmark scene reports beams, substeps and milliseconds per step. (`cargo run --release -p beam-core --example bench`: 11,136 beams, 17 substeps, roughly 6 to 10 ms per step natively on a noisy machine; see ER 001, phase 1.7. The core's own sine and cosine are in `core/src/maths.rs`.)

### 2. 2D bridge builder ([Plan 002](docs/plans/002_bridge_builder.md))

- [x] The core runs in the browser as WebAssembly, stepped from TypeScript. (The editor page loads it: `pnpm build:web`, then `pnpm serve`. In the browser the reported 11,136-beam scene ends at the native hash, 6.6 ms per step; see ER 002, phase 2.3.)
- [x] You can place nodes and beams, press run, and pause, single-step and slow down time. (`web/editor.ts`: node, beam, pin and delete tools, run, pause, step and five speeds; checked by hand in the browser, see ER 002, phase 2.3.)
- [x] A load rolls across the bridge and an under-built bridge breaks. (Headless, through the command buffer: `test/bridge.test.ts`; a strong truss carries the truck with no yield, a tenth as stiff yields then breaks; see ER 002, phase 2.2. Seen in the editor in 2.3: the same truss runs there and ends at the same hash as in Node.)
- [x] Beams are coloured by stress. (Blue in compression through grey to red at yield, purple once bent past yield, dashed when broken; `web/editor.ts`.)
- [x] A recorded run replays exactly. (`src/recording.ts`: the command list per step plus a hash every 60 steps. The committed weak-bridge recording replays to the same hash in Node (`pnpm replay`, `test/recording.test.ts`) and in the editor; a changed command is caught at the first checkpoint after it; see ER 002, phase 2.4.)

### 3. 3D on terrain in Three.js ([Plan 003](docs/plans/003_terrain_3d.md))

- [x] A tower and a bridge stand on height-map terrain with friction. (`src/terrain.ts`, `src/scenes3d.ts`, `test/scenes3d.test.ts`: both stand 20 s with no yield or break and come to rest, resting on the ground by weight and friction, not pinned; seen on the page in 3.5; see ER 003, phase 3.4.)
- [x] Both collapse believably when overloaded or knocked. (Headless tests in 3.4; seen on the page in 3.6: the tower tips over and lies flat under a knock aimed at a roof corner, the bridge fails under a 50 t truck at the middle of the span; the on-screen hashes equal Node's; see ER 003, phases 3.4 and 3.6.)
- [x] Meshes are skinned to the nodes and bend with them. (`src/skin.ts`, `src/panels3d.ts`, `test/skin.test.ts`: floor, wall and deck panels bound to four nodes of their cell, exact for rotation, movement and stretch, and the shader's 32-bit maths agrees with the reference; seen on the page under WebGPU and WebGL 2: the panels tilt and fall with the knocked tower and sag with the overloaded bridge; see ER 003, phase 3.8.)
- [x] The stress colours and time controls work in 3D. (Pause, single step, five speeds measured against their step rates, click-to-knock, the three action buttons; beams go from blue and grey to red and purple as load builds; `web/viewer3d.ts`, `test/live3d.test.ts`; see ER 003, phase 3.6.)

### 4. Parts and assembly (first playable slice) ([Plan 004](docs/plans/004_parts_and_assembly.md))

- [x] Tank, truss, landing leg, engine and pod are generated from parameters. (`src/parts/`: each part compiles, has its exact stated mass, is inside the budget, rests on flat ground for 10 s with no yield and is byte-identical run twice; the leg stays elastic below its crush force, crushes at about that force and then breaks in the crush bay; the engine and the pod carry the hopper's full thrust with no yield; see ER 004, phases 4.2 and 4.3.)
- [x] Parts join at attachment rings into one craft, saved and loaded as a craft file. (`src/parts/assemble.ts` welds rings and mount patches with a whole-step clock and refuses every bad join with a reason; `src/parts/craft-file.ts` saves one canonical text, so the committed hopper file loads and saves back byte for byte, compiles to the same command bytes as the hopper built in code and gives the same state hash after 600 steps; unknown fields, unknown part types, later versions, bad joins and non-finite numbers are refused with a reason; see ER 004, phases 4.4 and 4.5.)
- [x] Engines produce thrust and the craft lifts off. (Headless: `src/flight.ts` plays a throttle script on the landing field; at full throttle the hopper accelerates at thrust over mass minus gravity within 0.01%, level to under 0.02°, with nothing yielding; eased on just under hover it stays on the pad; the same script gives the same hash; see ER 004, phase 4.7. Seen on the page in 4.9: `web/craft.html` lifts off on full throttle.)
- [x] **Test of fun:** a hard landing on rough terrain snaps a leg and the craft tips over. (Bands in `test/landing.test.ts` pass, ER 004 phases 4.8 and 4.9; the snap is seen on the craft page: the scripted hop loses two legs, shows yielded beams in purple and tips the craft to about 100°, and the page's final hash equals Node's.)

### 5. Rigid by default ([Plan 005](docs/plans/005_rigid_by_default.md))

- [x] Rigid bodies with anchors carry engines, pods and wheels. (Headless: the hopper's engine and pod ride as bodies (ER 005, phase 5.4); the rover's four wheels are rigid bodies on rotor hinges, rest on the pad without creeping and roll at rotor speed times radius within 1% at 1 to 2 rad/s (`test/rover.test.ts`, ER 005, phase 5.6).)
- [x] Hinges, joint limits, servos and rotors work, and a stalled servo adds no energy. (Headless: `core/tests/joint.rs`; a servo stalled ten simulated minutes against a load twice its cap never gains energy, and the drive never exceeds its cap; see ER 005, phase 5.5.)
- [x] Segments go rigid when barely loaded and wake under load, with momentum conserved across every switch. (Headless: `core/src/sleep.rs` (ER 005, phase 5.8); `test/equivalence.test.ts` runs 16 hopper scenes sleeping on and all-live: the same breaks within one beam, the same legs lost, the tilt within 0.5°, and every switch keeps momentum and spin to 1e-12; on by default in flight from phase 5.9.)
- [x] Touching rigid segments merge into one body, and a tall rocket visibly wobbles less than with everything live. (Headless numbers: a 15 m, 7000 kg rocket on a clamp becomes one body from step 180; its tip moves 0.48 against 2.08 mm standing and 0.32 against 4.26 mm under full thrust; `test/tall-rocket.test.ts`, ER 005, phase 5.11. Seen on the page in 5.12: the rocket sits on the clamp with every segment asleep as one body, and a knock wakes the whole craft; the page tests are `test/segment-view.test.ts` and `test/browser.test.ts`.)
- [x] A craft cut in two becomes two vessels with the correct total mass. (Headless: a hopper leg cut off in the air becomes vessel 1, the masses add to 4600 kg within 3 ulp and the momentum changes only by gravity; cut asleep, the body is divided keeping momentum and spin to 1e-12 with no energy added; the hard landing reports the leg it tears off as its own vessel (`test/vessel.test.ts`, `core/tests/vessel.rs`, ER 005, phase 5.10).)

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
