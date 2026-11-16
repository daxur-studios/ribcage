---
type: Research
title: Node-beam physics engine design spec
description: Original design spec for the real-time node-beam physics engine (goals, core model, solver, rigid-by-default, collision, failure, architecture, milestones).
timestamp: 2026-11-16T00:00:00Z
status: active
tags: [spec, physics, design]
---

# Node-beam physics engine: design spec

Nov 16, 2026 · @Brúnó

## Goals and principles

This is a real-time node-beam physics engine for a KSP-like building game, where structures flex, bend and break. It runs in the browser alongside Three.js.

- **Performance over accuracy.** Behaviour should be believable and fun, not engineering-grade.
- **Rigid by default.** Most of a craft is rigid most of the time, and flex appears only under load. The target is far less wobble than KSP.
- **No krakens.** Nothing explodes without a cause the player can see.
- **Deterministic on demand.** Needed for tests and replays now, and for multiplayer later.
- **Multiplayer is the north star.** It is not in the first release, but no decision should block it.
- **Small, testable pieces.** Each module can be checked headless, so AI agents can build it safely.

## Core model

Everything is built from four element types held in flat arrays.

| Element | What it is | Key properties |
| --- | --- | --- |
| Node | A point mass | Position, velocity, mass, radius |
| Beam | A spring between two nodes or anchors | Rest length, stiffness, damping, yield strain, break strain |
| Rigid body | A box, cylinder or capsule, or a merged group of sleeping segments | Position, rotation, mass, inertia, anchor points |
| Skin triangle | A surface between three nodes | Used for rendering, aerodynamics and buoyancy |

Beam variants:

- **Cable:** resists tension only. Used for ropes, stays and parachute lines.
- **Actuator:** its rest length is animated. Used for pistons, landing gear and hinges.
- **Seam:** a beam flagged as a pre-defined break line between segments.

Joints, for robotic parts and walkers:

- **Hinge:** two segments sharing an axis. In a lattice this is two shared nodes, and between rigid bodies it is a hinge constraint.
- **Joint limit:** a bend constraint that stops a hinge at its minimum and maximum angle.
- **Servo:** an actuator across a hinge that drives it to a target angle. It has a torque limit, so it stalls under too much load instead of injecting energy.
- **Rotor:** a free-spinning hinge with a motor. Wheels use this.

A moving joint keeps its neighbouring segments live.

Materials are rows in a table: density, stiffness, yield strength, breaking strain and damping. A beam's stiffness, mass and strength derive from its material and cross-section, so thickness is the player's design dial. All stiffness values are scaled down by one common factor to keep the solver stable, with the ratios between materials preserved. The first set is steel (heavy, bends before failing), aluminium (light, yields sooner) and carbon fibre (light, stiff, snaps without warning).

## Solver and timestep

The solver is XPBD on a fixed timestep, and it is deterministic by default.

- **Fixed step:** the simulation advances at 60 Hz and rendering interpolates between the last two states.
- **Substeps:** each island (one craft or structure, plus anything touching it) gets its own substep count. The count comes from its stiffest beam, its lightest node and its current load.
- **Gravity:** a per-node function, so flat and spherical gravity are both supported with no special handling.
- **Islands in parallel:** islands do not affect each other within a step, so solving them on separate threads gives the same result as solving them in sequence.

Two modes share one code path:

| Mode | Used for | What changes |
| --- | --- | --- |
| Deterministic | Tests, replays, multiplayer | Fixed solve order and rule-based substep counts. Same inputs always give the same result. |
| Fast | Single-player on weak hardware | Substeps and detail may drop when a frame runs over budget, so results vary with machine speed. |

WebAssembly arithmetic gives the same results on every machine, so determinism mostly costs discipline, not speed. The rules are a fixed iteration order, no dependence on frame time, and our own sine and cosine instead of the browser's.

## Rigid by default

Every segment of every part is either live (its nodes and beams are solved) or rigid (its nodes ride on a rigid body), and it switches on load.

- **Segments:** each part is sliced along its length when generated, for example the bays between a wing's ribs. A segment is the smallest unit that switches.
- **Merging:** rigid segments that touch merge into one rigid body. This avoids chains of stiff joints, which is where KSP-style wobble comes from.
- **Anchors:** live segments attach to the rigid body through anchor points, the same mechanism used for engines and wheels.
- **Wake rule:** a segment wakes when its estimated internal stress passes 30-50% of yield. The estimate uses the forces at its anchors plus the inertial load from the body's acceleration.
- **Other wake triggers:** a contact, a thrust change, an actuator moving, docking, or a neighbouring segment under high load.
- **Sleep rule:** a segment goes rigid after about a second of low strain, keeping any permanent bend. The sleep threshold sits below the wake threshold so it does not flicker.
- **Switching:** positions do not change on a switch, and a woken node takes the body's velocity plus its spin contribution. Momentum must be conserved across every switch.
- **Time warp and orbit:** the whole vessel is one rigid body on rails.

Stiffness and sleeping are separate. Geometry and material decide how stiff something looks, and sleeping is only a cost saving applied where a segment is barely straining.

## Collision

Collision is built in four stages, and the first stage alone covers bridges, collapsing buildings and landing legs.

1. **Nodes against terrain and water.** Each node tests against a height function, with friction. Beam midpoints are also sampled so a beam cannot straddle a sharp ridge.
2. **Nodes against static primitives.** Pads, rocks and sleeping buildings are spheres, capsules, boxes and cylinders, tested with signed distance functions.
3. **Body against body.** Surface nodes are spheres and surface beams are capsules. Each part also emits one coarse primitive, so other craft can test against that instead of the fine skin.
4. **Self-collision.** Not planned. If crumpling looks wrong, add it only between separate parts of one craft.

Shared rules:

- Contacts are detected once per 60 Hz step and reused across substeps.
- A spatial hash finds candidate pairs.
- Rigid bodies collide with terrain through a few sample points on their surface.
- Part generators emit collision primitives alongside nodes, because the shape is known at generation time.

## Failure and breaking

Beams bend permanently past their yield strain and are removed past their break strain, and the visible mesh separates only along pre-defined seams.

- **Yield:** the beam's rest length shifts, so the part stays bent.
- **Break:** the beam is deleted and a break event is sent to the game.
- **Seams:** the render mesh is pre-cut along segment boundaries. When the seam beams on a boundary are gone, the two sides separate, with particles and debris covering the gap.
- **Interior breaks:** beams that break inside a segment weaken and deform it but do not cut the mesh.
- **Splitting:** after any break, a connectivity check finds whether the craft is now two or more pieces. Each piece becomes its own vessel with its own rigid groups.
- **Debris:** small pieces that no longer matter to gameplay are handed to the GPU as visual-only debris.
- **Heat:** strength falls with temperature, which gives re-entry and engine-fire failures. This comes after the first slice.

## Environment

Terrain and water are height functions for gameplay physics, and GPU ball simulations are a visual layer on top.

### Terrain

- **Base:** a height function sampled per node, in a local frame near the craft with a floating origin.
- **Surface types:** rock, sand, mud and snow each have stiffness, damping, friction and a bearing strength. Soft surfaces resist with a force that grows with depth and speed, so things sink in.
- **Deformable ground:** a small grid of height offsets sits on top of the base terrain near the action. Where a node presses harder than the bearing strength, the physics lowers the offset there. The result is ruts, skid trenches and craters, and the renderer reads the same grid.

### Water

- **Surface:** an analytic wave function (a sum of Gerstner waves), used by both the physics and the water shader.
- **Buoyancy and drag:** applied per skin triangle, reusing the aerodynamics code with water's density.
- **Flooding:** sealed compartments lose buoyancy when their hull beams break.

### GPU ball water and ball terrain

- **First use:** visuals only. The craft pushes the balls, and the balls do not push the craft. Splashes, wakes, spray and dust come from this layer.
- **Why not gameplay yet:** two-way coupling needs GPU results read back to the CPU, which arrives late and is not deterministic. That conflicts with the multiplayer goal.
- **Later option:** a separate project to make ball fluids drive gameplay in small, contained places such as a fuel tank or a pond.

## Procedural parts and craft format

A part is a function from parameters to physics data, and players never see nodes.

Each generator outputs nodes, beams, skin triangles, segments, seams, attachment rings and collision primitives.

| Part | Built as | Main parameters |
| --- | --- | --- |
| Tank or fuselage | Rings, lengthwise beams, diagonals, interior cross-beams | Length, radius, material, wall thickness |
| Truss | Lattice | Length, width, material |
| Landing leg | Lattice with an actuator | Length, material, stroke |
| Wing | Spars, ribs and skin, tapered | Span, chord, taper, thickness, material |
| Engine, pod | Rigid body | Size, mass, thrust |
| Wheel | Rigid cylinder on a hinge | Radius, width |
| Cable, piston | Single beam variants | Length, strength |

Robotic parts are built from the joints in the core model:

- **Hinge and servo parts:** for arms, cranes, folding wings, deployable panels and walker legs. Parameters are size, angle limits, torque limit and material.
- **Sequencer:** a controller part that plays target angles over time, programmed by the player.
- **Walker brain:** a later controller part that runs an assisted gait, with a rhythm per leg and foot placement targets. It is shared with the creature game.

* **Joining:** parts weld at matching attachment rings, or are bridged with beams. Joints are natural seams.
* **Mass limits:** generators keep node masses within a set ratio of each other, which protects solver stability.
* **Detail levels:** the same parameters can generate a coarser lattice for distant craft.
* **Craft file:** part parameters plus the assembly tree. Damage (broken beams, bent rest lengths) is a separate optional layer.

## Stability guards

Each known cause of a kraken has a specific guard, and a glitch that gets through should look like a break, not a launch into space.

| Cause | Guard |
| --- | --- |
| Stiff beams with a large timestep | XPBD with per-island substeps |
| Heavy part on a light structure | Mass-ratio limits in generators, and heavy items as rigid bodies |
| Chains of stiff joints | Rigid segments merge into one body |
| Precision loss far from the origin | Local frame with a floating origin |
| Parts spawned or docked overlapping | Weld only at rest, and push apart gently before enabling contacts |
| Energy injected on a mode switch | Momentum-conserving switch, covered by tests |
| Undamped ringing or flutter | A minimum damping on every beam |
| A single bad value spreading | Velocity and per-step movement clamps, and any beam stretched past a hard limit breaks |
| Invalid numbers | Detected each step. The island freezes and reports instead of spreading them. |

Servos and motors are torque-limited and their mass ratio to the arm they drive is capped, because a long arm on a small servo is a classic source of instability.

In debug builds, an energy monitor flags any island whose energy rises with no external input.

## Architecture

The engine is a Rust core compiled to WebAssembly, with Three.js reading its output buffers directly.

- **Core:** Rust, built for WebAssembly in the game and natively for tests and benchmarks. It has no rendering or browser dependencies.
- **Threads:** a worker pool solves one island per task. This needs cross-origin isolation headers on the host, so a single-thread fallback is kept.
- **Boundary:** one step call per frame. Inputs go in as a command buffer (thrust, actuator targets, spawn, delete). Outputs come back as buffers of node positions, rigid body transforms and events (break, split, wake, contact).
- **Memory views:** JavaScript views into WebAssembly memory are re-fetched whenever memory grows.
- **Rendering:** part meshes are skinned to node positions in the vertex shader, with positions uploaded once per frame. TSL handles particles, debris and the ball-water visuals.
- **Dimensions:** the 2D bridge builder and the 3D engine share the data layout, solver and failure rules. Only the vector size and the collision code differ.
- **Jolt:** optional. It can keep handling free rigid objects that are not attached to a craft, or be removed once the engine has its own rigid bodies.

## Tools and tests

The debugging tools and headless tests are built before any gameplay feature, because every later step depends on them.

Tools:

- **Stress heat-map:** beams coloured by strain against yield. It doubles as a player feature.
- **State overlay:** which segments are live or rigid, plus contacts and anchors.
- **Time controls:** pause, single-step and slow motion.
- **Record and replay:** inputs are recorded and replayed exactly, which also proves determinism.
- **Benchmark scene:** a fixed stress scene reporting beams, substeps and milliseconds per step.

Headless tests, run natively:

- A cantilever's tip deflection matches the textbook beam formula within a tolerance.
- A resting structure does not gain energy over ten minutes of simulated time.
- The same inputs produce an identical state hash on every run.
- Momentum is conserved across every live-to-rigid and rigid-to-live switch.
- A beam loaded past its break strain breaks, and one loaded below yield returns to its rest shape.
- A craft cut in two becomes two vessels with the correct total mass.

## Milestones

The first vertical slice ends at milestone 4, and its test of fun is: a hard landing on rough terrain snaps a leg and the craft tips over.

1. **Solver core and tools.** Nodes, beams, XPBD, fixed step, yield and break, the headless tests and the benchmark.
2. **2D bridge builder.** Place nodes and beams, run, roll a load across, and watch it break. Includes the stress heat-map.
3. **3D port.** A tower and a bridge collapsing on terrain, rendered in Three.js with skinned meshes.
4. **Parts and assembly.** Tank, truss, leg, rigid engine and pod, joined at rings, with thrust. The hard-landing test must pass here.
5. **Rigid by default.** Rigid bodies with anchors, hinges, joint limits and servos, then per-segment sleeping, merging and splitting into vessels.
6. **Aerodynamics.** Per-triangle lift and drag, with wings that flex and snap in a hard turn.
7. **Ground and water.** Surface types, deformable ground, waves and buoyancy.
8. **Body-against-body collision and threads.** Capsule and sphere contacts between craft, and islands solved on workers.
9. **Visual layer.** Seam tearing with particles, GPU debris and ball-water effects.

Multiplayer, heat and gameplay-driving ball fluids come after this list.

## Open questions

- [ ] Does Jolt stay for free rigid objects, or is it removed once the engine has its own rigid bodies?
- [ ] Where will the game be hosted, and does that host allow the headers that multi-threaded WebAssembly needs?
- [ ] What is the performance budget: how many live beams, on what target hardware, at what frame rate?
- [ ] Should wrecks persist in saves, which decides whether the damage layer is built early?
- [ ] How is planet rotation handled in the local frame near the surface?
- [ ] Is the 2D bridge builder a shippable mini-game or only a test bed?
- [ ] Which multiplayer model is the target (lockstep with shared inputs, or a server sending state)? This sets how strict determinism must be.
