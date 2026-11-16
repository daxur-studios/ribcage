---
type: Research
title: Collision and threads design brief
description: Early design for milestone 8 (craft-against-craft collision and solving islands on worker threads), with its traps, tests and the rules it imposes on earlier milestones.
timestamp: 2026-11-16T23:20:00Z
status: active
tags: [design, collision, threads, determinism, milestone-8]
---

# Collision and threads: design brief

**TL;DR.** Two craft should be able to hit and damage each other, and many craft should be solved on several processor cores with exactly the same result as on one. Collision reuses the solver's one distance function, so contacts work the same against live and rigid parts. Threads are safe because each island is solved from its own packed copy of the data, which milestone 1 already builds. Two things to know early: threads speed up scenes with many craft, not one big craft, and craft built from nodes and beams have gaps that small things can poke through unless the part generators prevent it.

This is a brief, not a plan. When milestone 8 starts, a detailed plan is written from this against the real code. Items marked **[assumption]** were decided before any code existed and must be re-checked.

Read with the [spec](node-beam-engine-design-spec.md), the [core layout rules](core-data-layout.md) and the [rigid-by-default brief](rigid-by-default-design-brief.md).

## What the player should see

- Two craft collide, bend and break where they touch, and bounce or tangle believably.
- Nothing passes through anything else at ordinary speeds.
- Docking, or spawning a part that overlaps another, never flings anything away.
- A scene with many craft or lots of wreckage runs smoothly on a machine with several cores.
- A replay gives the same result on a phone with two cores and a desktop with sixteen.

## Collision shapes

- A surface node is a sphere. A surface beam is a capsule (a cylinder with rounded ends) between its two nodes.
- Each part also has one coarse shape (box, cylinder or capsule) that encloses it.
- Interior nodes and beams do not collide.

## Finding contacts

Done once per 60 Hz step, before the solve, in three levels:

1. **Vessel against vessel:** bounding boxes, grown by a margin.
2. **Part against part:** the coarse shapes of the two vessels' parts.
3. **Surface against surface:** spheres and capsules of the two overlapping parts.

Levels 1 and 3 use a spatial grid. To keep it deterministic the grid is not a hash map: each item gets an integer cell key, the list of keys is sorted, and neighbours are found by walking the sorted list. Candidate pairs are ordered lowest index first and sorted before use.

**Margin.** A pair counts as a contact if the gap is less than the distance the two could close in one step plus a small skin. That lets the contact list be reused across substeps without missing a contact that starts mid-step.

**[assumption]** Margin from relative speed at the start of the step. Very fast, thin objects can still pass through; the engine does not promise otherwise, and the spec's per-step movement clamp bounds how far anything travels.

## Contact constraints

A contact is a distance constraint that only pushes. It goes through the same distance-solve function as beams, with one extension: an endpoint may be a point part-way along a beam, which is a weighted mix of the beam's two nodes.

| Pair | Endpoints |
| --- | --- |
| Sphere against sphere | Node, node |
| Sphere against capsule | Node, point on beam |
| Capsule against capsule | Point on beam, point on beam |

- The contact stores which features touch, where along each beam, and the direction. These are fixed for the step. The depth is re-measured each substep from current positions.
- Because endpoints resolve to nodes, a contact against a rigid part pushes on its body through the riding-node path from the rigid-by-default design. No separate rigid-body collision code is needed for craft.
- **Friction:** after the push, sideways slip at the contact during the substep is cancelled, up to the friction coefficient times the push. This gives sticking and sliding.
- **Bounce:** none by default. Craft are structures, not balls.
- **Recovery speed cap:** a contact removes overlap no faster than a fixed speed. A deep overlap is pushed apart over several steps instead of in one. This is the main guard against parts exploding apart.

**[assumption]** Cap of about 2 m/s.

A contact on a sleeping segment wakes it and its neighbours before the solve, as the rigid-by-default brief requires.

## Islands

An island is everything that can affect each other within one step.

- Start with one island per vessel.
- Join two vessels' islands if there is a contact between them this step, or a beam between them (docked).
- The static world (terrain, pads, sleeping buildings) never joins islands. Two craft resting on the same ground are separate islands.
- Islands are found with a union-find over vessels, after contacts are found and before the solve. An island's identity is its lowest node index.
- Contacts are fixed for the step, so islands cannot start interacting mid-step. That is what makes solving them separately exact, not approximate.
- Each island has its own substep count.

## Step order

| Phase | Runs | What happens |
| --- | --- | --- |
| 1. Commands | In order, one thread | Spawn, delete, thrust, targets |
| 2. Contacts and islands | One thread to start | Find contacts, wake touched segments, build islands and their packed buffers |
| 3. Solve | One task per island, any thread | All substeps for that island |
| 4. Merge | In island order, one thread | Copy state back, join event lists, apply ground changes, yield and break, splits, sleep and wake decisions |
| 5. Output | One thread | Write the render buffers |

## Why threads give the same result

- An island task reads only its own packed buffer and shared data that nothing writes during the solve (terrain, materials, its slice of the contact list).
- It writes only its own buffer, its own event list and its own list of ground changes.
- Nothing is summed, compared or decided across islands during phase 3.
- Phase 4 always runs in island order.

So the result does not depend on how many threads there are, which thread takes which island, or which finishes first. The single-thread fallback runs the same tasks in island order and matches bit for bit.

Two places where this would be easy to break:

- **Deformable ground (milestone 7).** Two islands pressing on the same patch of ground would both write to it. So an island does not write to the ground grid. It reads the grid as it was at the start of the step and records the changes it wants. Phase 4 applies them in island order.
- **Fast mode.** Dropping substeps when a frame runs long depends on machine speed by design. It is never on in deterministic mode, and it changes only the substep count.

## Threads in the browser

- Threads in WebAssembly need shared memory, which browsers allow only on pages served with two cross-origin isolation headers. Whether the host allows those headers is a spec open question, so the single-thread path stays fully supported.
- The main browser thread may not block waiting for workers. So the whole simulation runs in its own worker, and that worker hands islands to a small pool.
- With shared memory, the page reads the output buffers directly. Without it, the simulation worker sends a copy each frame.
- **[assumption]** The Rust and browser tooling for this changes often. Check current documentation when the milestone starts instead of trusting this section's details.

## Traps

1. **Threads do not speed up one big craft.** One craft is one island and one task. Threads help scenes with many craft, wreckage and bases. For a single large craft the gains come from sleeping segments (milestone 5) and, later, from solving non-touching beams side by side within an island. Set the performance budget with this in mind.
2. **Gaps in the net.** A craft's surface is spheres and capsules with open faces between them. A small node from another craft can pass through a face. Generators must size surface node and beam radii so that no gap is wider than the smallest surface sphere in the game. If that makes shapes too fat, add triangle tests for the faces.
3. **Deep overlap at creation.** Spawned or docked parts may overlap. The recovery speed cap handles it, and welds are made at rest with each new beam's rest length set to its current length, so a weld starts with no stored force.
4. **Contact lists that change order.** If contacts are solved in a different order from run to run, results differ. Contacts are sorted by their feature indices before the solve.
5. **Stacks and piles.** Reusing contacts across substeps and solving each once per substep can make resting piles creep or jitter. The resting test below covers it; the usual cures are a small extra pass over contacts or a slightly larger skin.
6. **Big islands from small touches.** A long row of craft each touching the next becomes one island. That is correct, only slow. Nothing to fix, but the benchmark should include it.

## Tests

- Two craft colliding in empty space: total linear and angular momentum agree before and after, to within rounding.
- A craft resting on another for ten simulated minutes gains no energy and does not creep.
- A part spawned overlapping another separates at no more than the recovery speed cap.
- A small probe fired at a hull at a range of speeds up to the stated limit does not pass through.
- The state hash after a many-craft scene is identical with 1, 2 and 8 threads, and with the single-thread fallback.
- Islands joining and separating over a run give the same hash on every run.
- Two craft pressing ruts into the same ground patch give the same ground on every run and every thread count.
- The benchmark scene meets the performance budget, once the owner sets one.

## What this requires of earlier milestones

| Milestone | Requirement |
| --- | --- |
| 1 | The solver is a pure function of a packed island buffer and read-only shared data. No global mutable state anywhere in the core. Events are written per island and joined in island order. Splitting into islands is a function, even while it returns one island. |
| 1 | The distance-solve function takes endpoints, so that "a point along a beam" can be added without touching callers. |
| 2 | The page talks to the simulation through one thin wrapper (step, send commands, read buffers). No other code reaches into the core. Moving the simulation into a worker later then changes only the wrapper. |
| 3 | Terrain is a pure function of position that the solver only reads. Contacts with terrain use the same push-only constraint, recovery cap and friction as here, so milestone 8 adds pairs, not a second contact model. |
| 4 | Generators emit a surface flag per node and beam, radii that meet the gap rule, and one coarse shape per part. |
| 7 | Ground changes are recorded by the island and applied in the merge phase. |

## Open at the time of writing

- The performance budget (live beams, target hardware, frame rate). Needed to judge the last test.
- Hosting and whether the isolation headers are available.
- The multiplayer model. Lockstep needs the bit-for-bit guarantees above on every player's machine; a server sending state needs them only on the server. The design above assumes the stricter case.
