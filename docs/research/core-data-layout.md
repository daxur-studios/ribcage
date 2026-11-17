---
type: Research
title: Core data layout and solver rules
description: Binding rules for how milestone 1 stores nodes and beams and runs the solver, chosen so rigid bodies, collision and threads slot in later without a rewrite.
timestamp: 2026-11-16T23:00:00Z
status: active
tags: [design, solver, data-layout, determinism]
---

# Core data layout and solver rules

**TL;DR.** Milestone 1 can be built so that the two hardest later milestones (rigid by default, and collision with threads) are additions instead of rewrites. That takes nine rules, listed first. Everything after the rules is the reasoning and the detail a builder needs. Items marked **[assumption]** are best guesses made before any code existed and must be re-checked by measurement.

Read with the [spec](node-beam-engine-design-spec.md), the [rigid-by-default brief](rigid-by-default-design-brief.md) and the [collision-and-threads brief](collision-and-threads-design-brief.md).

## The nine rules

1. **3D everywhere.** One vector type with three components. The 2D bridge builder is the same core with every node in one plane.
2. **One number type, behind an alias.** `Real` is 64-bit floating point to start. Nothing in the core names `f32` or `f64` directly except the output buffers.
3. **Flat arrays, one per field.** Adding a field later means adding an array, not changing a struct that other code depends on.
4. **Indices are stable for life.** A node or beam keeps its index until an explicit, deterministic compaction. Deleting sets a flag. Indices are `u32`.
5. **The solver never touches the world.** It works on a packed island buffer built from the world at the start of the step and copied back at the end.
6. **A beam endpoint is always a node.** One function solves a distance constraint between two endpoints. No second copy of that maths anywhere.
7. **Forces go through one door.** Every external force on a node is added through one function and kept until the end of the step.
8. **Events and commands are buffers.** Commands are applied in order at the start of a step. Events are appended in a fixed order during it.
9. **The determinism list is law.** See the section below. Each item has a test or a lint.

## Why these rules

### 1. 3D everywhere

A core written for 2D first would need its vector type, rotation handling and collision changed for milestone 3. Writing it 3D costs about half again as much arithmetic for the bridge builder, which is tiny, and removes that port.

In a planar scene every node starts with zero depth and no force acts in depth, so depth stays exactly zero. A `planar` world flag also zeroes depth position and velocity each substep, as a guard.

### 2. `Real` is 64-bit to start

The solver derives velocity from the change in position over a substep. With 32-bit numbers, a node 100 m from the origin has a position resolution of about 0.008 mm, and at 600 substeps per second that becomes velocity noise of several millimetres per second, larger than gravity's effect in one substep. With 64-bit numbers the problem disappears.

WebAssembly runs 64-bit arithmetic at the same speed as 32-bit, apart from memory use and the width of vector instructions.

**[assumption]** 64-bit is fast enough. If the milestone 1 benchmark shows memory bandwidth is the limit, the alternative is 32-bit positions stored relative to each island's own origin. The alias keeps that switch to one line plus re-running the tests.

Output buffers for rendering are 32-bit, relative to the floating origin.

### 3. Flat arrays, one per field

World storage for milestone 1:

| Array | Per | Meaning |
| --- | --- | --- |
| `pos`, `vel` | node | Position and velocity |
| `inv_mass` | node | One over mass. Zero means pinned in place. |
| `radius` | node | For collision later; stored from the start |
| `force` | node | External force gathered this step (rule 7) |
| `node_flags` | node | Alive, pinned, frozen (set when the node's island hit a bad number; cleared only by `unfreeze_node`). In the state hash. |
| `beam_a`, `beam_b` | beam | Endpoint node indices |
| `rest`, `rest0` | beam | Current and original rest length. Yield moves `rest`; `rest0` never changes. |
| `compliance` | beam | One over stiffness |
| `damping` | beam | Newton-seconds per metre. Never below the global minimum |
| `yield_strain`, `break_strain` | beam | Failure limits |
| `beam_flags` | beam | Alive, broken, and later cable, actuator, seam |
| `last_lambda` | beam | Constraint force from the last substep, for the stress display and for milestone 5 |

Fields for later milestones (segment, rigid body, vessel, material, surface flag) are added as new arrays when needed. Nothing above changes when they arrive.

### 4. Stable indices

The renderer skins meshes to node indices, the game holds beam indices for the stress display, and replays refer to both. If deleting a beam moved another beam into its slot, all three would break. So a broken beam keeps its slot with a `broken` flag, and a free list hands dead slots to new elements in a fixed order.

Handles given to the host are an index plus a generation counter, so a stale handle is detected instead of pointing at a reused slot.

### 5. Packed island buffers

This is the rule that makes threads and sleeping cheap later.

At the start of a step the world is split into islands. For each island the core builds a packed buffer: its live nodes and live beams, copied in ascending index order, with beam endpoints rewritten as positions in the packed arrays. The solver is a pure function of that buffer:

```
solve(island buffer, substep count, static world) -> updated buffer + events
```

After the solve, state is copied back to the world arrays.

- **Threads (milestone 8):** each task owns its buffer, so parallel solving needs no unsafe code and cannot depend on thread timing.
- **Sleeping (milestone 5):** a sleeping segment is simply not copied into the buffer. The "active list" and the packed buffer are the same thing.
- **Speed:** the solver walks contiguous memory. The copy costs far less than the eight or more substeps it feeds.
- **Tests and benchmarks:** they can build a buffer directly.

The index mapping is rebuilt only when an island's membership changes. On other steps only the state is copied.

Islands are connected components, rebuilt every step. Two movable nodes are in the same island when a live beam links them, directly or through other movable nodes. Pinned nodes (inverse mass zero) never join islands: a pinned node is copied into every island that has a beam to it, so two structures bolted to the same anchor stay separate islands. Islands are ordered by their lowest movable node index (the island's root). A movable node with no beams is an island of its own. **[assumption]** A live beam between two pinned nodes belongs to no component; the core puts all such beams in one extra island after the others (root `u32::MAX`, no movable nodes), so they are still checked for break. The substep count is chosen per island.

### 6. One distance-solve function

In milestone 5 a node can ride on a rigid body, and the constraint then needs that body's resistance to being pushed at that point instead of the node's mass. In milestone 8, contacts push on points along a beam, which are weighted mixes of two nodes. Both are small extensions of one function if there is one function. If the distance maths is copied into the beam loop, the contact loop and the joint code, each copy must be extended and tested separately.

So milestone 1 has a single `solve_distance(endpoint a, endpoint b, rest, yield stretch, compliance, substep length)`. In milestone 1 an endpoint is a plain node. `rest` is passed mutably because the yield cap inside the function moves it (see Solver outline). Beam damping is not part of it: it is a separate velocity pass, `damp_beam`, with the same endpoints (changed in the phase 1.5 damping fix).

### 7. One door for forces

Milestone 5 decides when to wake a rigid segment from the forces acting on it, without solving its beams. That needs to know every external force that was applied in the step. If thrust, drag and test loads each wrote to velocity directly, that information would be gone. So all external forces are accumulated per node through `add_force`, applied by the solver, and cleared at the end of the step.

Gravity is a per-node function, as in the spec, evaluated in the solver and reported separately.

### 8. Buffers at the boundary

- **Commands in:** a list of fixed-layout records (spawn, delete, set force, pin, set actuator target), applied in list order before anything else in the step.
- **Events out:** fixed-layout records (break, yield, freeze, and later split, wake, contact). Each island writes its own list during the solve. Lists are joined in island order afterwards. Island order is by lowest movable node index. A freeze event has `index` = the island's lowest movable node and `value` = the substep number (1-based) in which the bad number appeared.
- **Output:** node positions as 32-bit values in one buffer, written once per rendered frame.

Record and replay is then "save the command lists per step", and the determinism test is "replay gives the same hash".

## Determinism list

| Rule | Why | How it is enforced |
| --- | --- | --- |
| Only add, subtract, multiply, divide and square root on `Real` | These give identical bits natively and in WebAssembly. Library functions do not. | A test that scans `core/src` for calls to library maths functions (`core/tests/maths.rs`); the core's own sine and cosine are in `core/src/maths.rs` |
| Own sine, cosine and any other transcendental function | Same reason | The core's maths module is the only one allowed |
| No fused multiply-add | Native hardware and WebAssembly round it differently | No `mul_add` calls; no fast-math flags |
| No iteration over hash maps or hash sets | Their order changes between runs | Use sorted lists or index arrays |
| No randomness in the core | | If ever needed: a counter-based generator seeded per island |
| Time is a step counter | Adding up fractions of a second drifts | An integer step count, with time derived from it |
| Fixed-width integers at the boundary and in the hash | A native build and WebAssembly disagree on the size of the default integer | `u32` and `u64` only |
| Islands, beams and nodes are always visited in ascending index order | | Packed buffers are built in that order |
| No global mutable state | A thread or a second world must not see leftovers | Everything lives in a world value passed in |
| Fast mode changes only the substep count function | One code path | Deterministic mode is the default and the tested one |

The state hash covers the raw bits of positions, velocities, rest lengths and flags, in index order, using a simple hash written in the core.

A bad number is detected at the end of each substep, after the clamp: any movable node with a non-finite position or velocity. The whole island is frozen at once. Every node position goes back to its value at the start of the step, movable velocities are zeroed, beam rest lengths, flags and `last_lambda` go back to their start-of-step values, and the movable nodes get `NODE_FROZEN`. One freeze event is sent (see section 8). From the next step any island containing a frozen node is skipped: no solve, no events, no copy back. Other islands are not affected and stay bit-identical to a run without the bad number. The host releases a node with `World::unfreeze_node`. `node_flags` is part of the hash, so frozen state is too.

**Movement clamp.** The world setting `max_speed` (default 1000 m/s, **[assumption]**) limits node speed. After the damping pass of each substep, a movable node faster than `max_speed` has its velocity scaled to `max_speed` and its position set to its position at the start of that substep plus the clamped velocity times `h`. There is no event; the world counter `clamped_nodes` counts distinct nodes clamped in the step and is reset at the start of each step. A NaN speed is not clamped; the bad-number check handles it.

## Solver outline

Per 60 Hz step, per island, with `n` substeps of length `h = 1 / (60 n)`:

1. For each substep:
   1. Predict: add gravity and gathered forces to velocity, save the position, move by velocity times `h`.
   2. Solve each live beam once, in order, through `solve_distance`.
   3. Derive velocity from the change in position.
   4. Damp each live beam once, in order, through `damp_beam`.
   5. Apply the movement and velocity clamp, then check for bad numbers (freeze).
2. After the last substep: decide break from each beam's total strain, emit the break and first-yield events. (`last_lambda` is stored per substep; yield itself happens inside the substeps.)

One solver pass per substep with many substeps converges better than many passes in one step. This is the "small steps" form of XPBD.

The distance correction for a beam with endpoints `a` and `b`, with `C` the current length minus rest length, `w` the sum of the two inverse masses and `α = compliance / h²`:

```
Δλ = -C / (w + α)
```

Each endpoint moves along the beam direction by its inverse mass times `Δλ`, in opposite senses. The force the spring carries is `-Δλ / h²`; that is `last_lambda`.

**Damping** is an implicit dashpot along the beam, applied to velocity after it is derived. With `v` the relative speed of the ends along the beam and `β = damping · h`, the pass removes `β · v / (1 + β · w)` of momentum from the pair, so the relative speed becomes `v / (1 + β · w)`. It cannot overshoot or reverse the motion at any damping, it is zero for a structure at rest, and `damping` is a physical coefficient (N·s/m) that means the same at any substep count. Its error is first order in `damping · w · h`: a critically damped test beam is about 3 to 5% off the continuous answer at 64 substeps, halving as the substeps double.

(History: phase 1.3 put damping inside `Δλ` as the XPBD paper's `γ = α · β̃ / h`. The phase 1.5 damping fix moved it out, because the yield cap then limited spring and damper together and moved `rest` as if the spring alone were at yield: a critically damped beam knocked at 0.5 m/s yielded at a third of its yield strain. ER 001 has the measurements of the options.)

**Yield** is a force cap inside the substeps, not a per-step rule. A beam's yield force is `F_y = yield_strain · rest0 / compliance`, and the force it carries is `F = -Δλ / h²`, so the correction is clamped to `|Δλ| ≤ F_y · h²` (no clamp when compliance is zero). A beam therefore never carries more than `F_y` in any substep, in tension or in compression, and the excess load goes to its neighbours in the same substep. When the clamp fires the beam yields: `rest` is set from the length the beam has after the clamped move, minus `yield_strain · rest0` in tension and plus it in compression (both ends move along the beam, so no square root is needed), and the `yielded` flag is set. The elastic stiffness never changes: the beam is stiff as before around a new rest length. `last_lambda` saturates at `F_y`. The cap is on the spring only: the damper sits beside it and is not capped, so a fast knock or a sudden load below `F_y` does not yield the beam (tested at a 0.99 `F_y` step load). Because the cap is a force, a held load above it flows: on an undamped beam at the excess force, `(F - F_y) / m` on a lone mass, whatever the step rate or substep count (tested at 8, 16 and 32 substeps); on a damped beam at the speed where the damper takes the excess, `(F - F_y) / damping`.

A beam between two pinned nodes (both inverse masses zero) is never solved, so it never yields; it can still break if the pins are moved apart, since break reads only its length.

**Break** is checked once per 60 Hz step, after the last substep, on total strain `(length - rest0) / rest0`. Past the break value the beam is flagged broken, in tension or compression. Measuring against the original length means a beam that has yielded a long way still reaches its break limit.

Break wins: a beam that breaks in a step gets a break event and no yield event. The yield event is sent once, for a beam whose `yielded` flag was set during that step, and its `value` is the beam's total strain at the end of the step (the same quantity a break event reports). Events are cleared at the start of each step and read by the host after it, in ascending beam order within one island (islands are joined in island order first). A load held above `F_y` is not stopped by the beam: it flows until the load drops, load moves to other beams, or the beam breaks on strain. Hardening (a stiffer beam past yield) is not part of milestone 1. Known mismatch: yield is force based and break is strain based; at the rule's substep count the solver is softer than exact by about `0.4 (ω h)²`, which moves strain and so break, but not the yield force of a statically determinate structure.

**Substep count.** For each beam, `ω = sqrt(stiffness · (inverse mass a + inverse mass b))`. The island's count is the smallest `n` with `ω_max · h ≤ c`, clamped to a minimum and maximum.

`c` is 1, minimum 4, maximum 32. **Measured in phase 1.4:** every scene measured stays bounded past `c = 2.5` (the test truss even at one substep, though it then sags 18 times too far), so `c` is not a stability limit. With one pass per substep, a settled structure is softer than the exact one by about `K · (ω_max · h)²`, with `K` between 0.35 and 0.42 on every scene measured (a chain, a flat truss, square trusses 6 to 24 times as long as they are wide). At the rule's count that is about `0.4 · c²`: 40% at `c = 1`, 10% at `c = 0.5` for twice the cost. `c = 1` is kept for now; the minimum and maximum are still **[assumption]**. The spec also asks the count to depend on current load; that part waits until there is a scene that needs it.

**Stiffness scale.** The spec scales all stiffness down by one factor. The factor is whatever keeps the stiffest common beam within the maximum substep count. **[assumption]** It is set once from the benchmark and then left alone, because every tuned value in the game depends on it.

## The cantilever test

A node-beam lattice is not a textbook beam, so the test compares like with like.

- **Shape:** a long square truss: four lengthwise rails, square frames at regular spacing, a diagonal on every face and across every frame. Length at least ten times the width, so bending dominates.
- **Expected deflection:** the bending term `P L³ / (3 E I)` with `I` from the four rails (rail area times width squared), plus the shear term from the diagonals. The stiffness used is the scaled one.
- **Method:** pin one end, hang a load on the other, settle with a test-side drag force (beam damping zero, since damping adds sag), compare the tip position.
- **Tolerance (measured in phase 1.4):** the formula is within 2% of the exact truss. With enough substeps (128) the solver settles within 2% of the formula. At the rule's count the solver is softer than exact by `K · (ω_max · h)²` with `K` between 0.38 and 0.46, about 43% at `c = 1`; the test checks that law rather than one percentage. The generator and the numbers are in `core/tests/`.

Build up to it in three steps, each a test: one hanging mass stretches its beam by weight over stiffness (this should be near exact), a hanging chain of ten, then the truss.

## What milestone 1 does not decide

- How the core is built and loaded as WebAssembly (milestone 2).
- The performance budget. The benchmark reports numbers; the budget is the owner's call.
- Compaction of dead slots. Not needed until scenes run long enough to fill up.
- Vector instructions and solving beams in parallel within one island. Possible later through colouring; changes results, so it would be a deliberate, versioned change.
