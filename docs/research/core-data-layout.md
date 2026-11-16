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
| `node_flags` | node | Alive, pinned |
| `beam_a`, `beam_b` | beam | Endpoint node indices |
| `rest`, `rest0` | beam | Current and original rest length. Yield moves `rest`; `rest0` never changes. |
| `compliance` | beam | One over stiffness |
| `damping` | beam | Never below the global minimum |
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

In milestone 1 there is one island holding everything. The split into islands is a function that returns one island for now.

### 6. One distance-solve function

In milestone 5 a node can ride on a rigid body, and the constraint then needs that body's resistance to being pushed at that point instead of the node's mass. In milestone 8, contacts push on points along a beam, which are weighted mixes of two nodes. Both are small extensions of one function if there is one function. If the distance maths is copied into the beam loop, the contact loop and the joint code, each copy must be extended and tested separately.

So milestone 1 has a single `solve_distance(endpoint a, endpoint b, rest, compliance, damping, substep length)`. In milestone 1 an endpoint is a plain node.

### 7. One door for forces

Milestone 5 decides when to wake a rigid segment from the forces acting on it, without solving its beams. That needs to know every external force that was applied in the step. If thrust, drag and test loads each wrote to velocity directly, that information would be gone. So all external forces are accumulated per node through `add_force`, applied by the solver, and cleared at the end of the step.

Gravity is a per-node function, as in the spec, evaluated in the solver and reported separately.

### 8. Buffers at the boundary

- **Commands in:** a list of fixed-layout records (spawn, delete, set force, pin, set actuator target), applied in list order before anything else in the step.
- **Events out:** fixed-layout records (break, yield, freeze, and later split, wake, contact). Each island writes its own list during the solve. Lists are joined in island order afterwards. Island order is by lowest node index.
- **Output:** node positions as 32-bit values in one buffer, written once per rendered frame.

Record and replay is then "save the command lists per step", and the determinism test is "replay gives the same hash".

## Determinism list

| Rule | Why | How it is enforced |
| --- | --- | --- |
| Only add, subtract, multiply, divide and square root on `Real` | These give identical bits natively and in WebAssembly. Library functions do not. | A lint or review check for calls to library maths functions in `core/` |
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

A bad number is detected at the end of each substep. The island is frozen (velocities zeroed, solving skipped) and an event is sent. Frozen state is part of the hash.

## Solver outline

Per 60 Hz step, per island, with `n` substeps of length `h = 1 / (60 n)`:

1. For each substep:
   1. Predict: add gravity and gathered forces to velocity, save the position, move by velocity times `h`.
   2. Solve each live beam once, in order, through `solve_distance`.
   3. Derive velocity from the change in position, then apply the movement and velocity clamps.
2. After the last substep: update yield and break from each beam's strain, emit events, store `last_lambda`.

One solver pass per substep with many substeps converges better than many passes in one step. This is the "small steps" form of XPBD.

The distance correction for a beam with endpoints `a` and `b`, with `C` the current length minus rest length, `w` the sum of the two inverse masses, `α = compliance / h²` and `γ = α · damping / h`:

```
Δλ = (-C - γ · (direction · relative movement this substep)) / ((1 + γ) · w + α)
```

Each endpoint moves along the beam direction by its inverse mass times `Δλ`, in opposite senses.

**Yield and break** are checked once per 60 Hz step, not per substep, using two strains:

- **Elastic strain** `(length - rest) / rest0` decides yield. Past the yield value, `rest` moves toward the current length until the elastic strain is back at the yield value. The beam stays bent.
- **Total strain** `(length - rest0) / rest0` decides break. Past the break value, the beam is flagged broken. Measuring against the original length means a beam that has yielded a long way still reaches its break limit.

Both are checked in tension and in compression.

**Substep count.** For each beam, `ω = sqrt(stiffness · (inverse mass a + inverse mass b))`. The island's count is the smallest `n` with `ω_max · h ≤ c`, clamped to a minimum and maximum.

**[assumption]** `c` near 1, minimum 4, maximum 32. The spec also asks the count to depend on current load; that part waits until there is a scene that needs it.

**Stiffness scale.** The spec scales all stiffness down by one factor. The factor is whatever keeps the stiffest common beam within the maximum substep count. **[assumption]** It is set once from the benchmark and then left alone, because every tuned value in the game depends on it.

## The cantilever test

A node-beam lattice is not a textbook beam, so the test compares like with like.

- **Shape:** a long square truss: four lengthwise rails, square frames at regular spacing, a diagonal on every face and across every frame. Length at least ten times the width, so bending dominates.
- **Expected deflection:** the bending term `P L³ / (3 E I)` with `I` from the four rails (rail area times width squared), plus the shear term from the diagonals. The stiffness used is the scaled one.
- **Method:** pin one end, hang a load on the other, run with damping until the movement dies away, compare the tip position.
- **[assumption]** Tolerance 10%. Phase 1.4 measures the real error and how it shrinks as substeps rise, then sets the tolerance from that and records it.

Build up to it in three steps, each a test: one hanging mass stretches its beam by weight over stiffness (this should be near exact), a hanging chain of ten, then the truss.

## What milestone 1 does not decide

- How the core is built and loaded as WebAssembly (milestone 2).
- The performance budget. The benchmark reports numbers; the budget is the owner's call.
- Compaction of dead slots. Not needed until scenes run long enough to fill up.
- Vector instructions and solving beams in parallel within one island. Possible later through colouring; changes results, so it would be a deliberate, versioned change.
