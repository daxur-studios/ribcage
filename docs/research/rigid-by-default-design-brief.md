---
type: Research
title: Rigid by default design brief
description: Early design for milestone 5 (rigid bodies, joints, sleeping, merging, splitting), with its traps, tests and the rules it imposes on earlier milestones.
timestamp: 2026-11-16T23:10:00Z
status: active
tags: [design, rigid-bodies, sleeping, joints, milestone-5]
---

# Rigid by default: design brief

**TL;DR.** A craft should look solid until something loads it hard, then flex and break. The design does this with one idea: a node can ride on a rigid body. Putting a segment to sleep means attaching its nodes to a body; waking it means letting them go. Beams never change. The brief gives the exact formulas that keep momentum unchanged across every switch, a cheap way to estimate stress in a sleeping segment, and five traps. One of them changes the spec: the wake threshold should be about 25% of yield, not 30-50%.

This is a brief, not a plan. When milestone 5 starts, a detailed plan is written from this against the real code. Items marked **[assumption]** were decided before any code existed and must be re-checked.

Read with the [spec](node-beam-engine-design-spec.md) and the [core layout rules](core-data-layout.md).

## What the player should see

- A tall rocket on the pad does not wobble at all.
- Under hard thrust or a rough landing, the loaded part of it visibly flexes, and breaks if overloaded.
- Switching between the two is never visible: no pop, no jolt, no sudden spin.
- A craft with everything allowed to sleep breaks in the same places, at about the same loads, as one with everything live. Sleeping is a cost saving, not a change to the game.

## The one idea: a node can ride on a body

Each node gets a `body` field. Empty means the node is live and solved as a point mass. Otherwise the node rides: its position is the body's position plus the body's rotation applied to a stored local offset.

- **A beam endpoint is still a node.** If the node rides, the single distance-solve function uses the body's resistance at that point and applies its correction to the body's position and rotation. This is the standard XPBD treatment of rigid bodies.
- **The spec's "anchor" is a riding node.** Engines, pods and wheels attach through them. No separate anchor type is needed.
- **A beam with both ends on the same body is skipped.** It is not copied into the island buffer.
- **A beam between a riding node and a live node is solved as normal.** This is how a live segment connects to its rigid neighbours.

For a riding endpoint at offset `r` from the body's centre, with unit direction `n`, the resistance used in place of inverse mass is:

```
w = 1 / mass + (r × n) · inverse_inertia · (r × n)
```

The correction `p = Δλ · n` moves the body's position by `p / mass` and turns it by `inverse_inertia · (r × p)`.

## Rigid body state

| Field | Notes |
| --- | --- |
| Position of the centre of mass, rotation | Rotation as a unit quaternion, re-normalised each substep |
| Linear and angular velocity | |
| Mass, inverse inertia in the body's own frame | A full symmetric 3×3, because merged groups have arbitrary shape |
| Previous position and rotation | The solver derives velocity from the change, as it does for nodes |
| Member segments | Kept sorted |

Bodies are integrated in the same substep loop as nodes: predict, solve, derive velocity.

**[assumption]** The spin term for a tumbling body (angular velocity crossed with angular momentum) is applied in its simple explicit form with a clamp on angular speed. If the energy test shows gain on fast-tumbling debris, switch to the implicit form.

## Segments and bodies

- Every node belongs to exactly one segment. Every beam belongs to one segment or is a seam beam between two.
- Each segment is live or rigid.
- **Bodies are the connected groups of rigid segments** within a vessel, where two segments are connected if they share a seam. They are recomputed, with a small union-find over segments, on any step where a segment switched or a seam broke. A body's identity is the lowest segment index in its group.
- Waking a segment in the middle of a rocket therefore splits one body into two, with no special code.
- Parts that are always rigid (engine, pod, wheel) are segments flagged never-live.

## Putting a segment to sleep

This turns a set of moving point masses into rigid motion. For the set of nodes being made rigid (a segment alone, or a segment joining a body, or two bodies merging, which are all the same sum):

```
mass     M = Σ m
centre   c = Σ m x / M
momentum P = Σ m v                       linear velocity  V = P / M
spin     L = Σ m (x - c) × v
inertia  I = Σ m (|r|² · identity - r rᵀ),  r = x - c
                                          angular velocity ω = I⁻¹ L
```

An existing body enters the sums as its own mass, momentum, spin and inertia, moved to the new centre.

- Linear and angular momentum are unchanged, exactly.
- Kinetic energy can only fall. Rigid motion is the lowest-energy motion with a given momentum and spin, so what is removed is internal vibration. Sleep can never add energy.
- Positions do not change. Each node stores its offset in the body's frame as it is now, so a bent segment stays bent.
- Beam rest lengths and any small leftover strain are kept as they are.

## Waking a segment

- Each released node takes the velocity `V + ω × r`. Positions do not change.
- The remaining body recomputes its mass, centre and inertia from its remaining nodes. Its angular velocity stays `ω` and its linear velocity becomes `V + ω × (new centre - old centre)`.
- Momentum, spin and kinetic energy are all unchanged, exactly, because every piece keeps moving as it was.

## When to wake: estimating stress without solving

A sleeping segment's beams are not solved, so its stress has to be estimated from outside. The estimate is a free-body cut, done once per 60 Hz step per vessel.

1. **Net load per segment.** Sum the external forces on its nodes (thrust, drag, contact, gravity, and the forces from beams that connect it to live nodes), then subtract its mass times its acceleration. Acceleration comes from the body's change in velocity over the step, including the spin terms. Do the same for moments about the segment's centre.
2. **Accumulate along the craft's tree.** The assembly tree gives each segment a parent. Walking from the leaves to the root, each seam receives the summed force and moment of everything beyond it. One pass, proportional to the number of segments.
3. **Convert to strain at each seam.** The part generator supplies each seam's section properties (effective area and bending stiffness of the lattice at that cut). Axial force over area, plus moment over section modulus, gives a strain estimate.
4. **A segment's estimate** is the larger of its two seams' values.

**[assumption]** For craft with closed loops (struts, docked rings) the tree ignores the extra paths, so the estimate reads high. That errs toward waking, which is safe. If it wakes far too much in strutted craft, split loop loads by stiffness.

Triggers that wake a segment regardless of the estimate: a contact on it, a thrust change on it, an actuator or joint on it moving, docking or welding to it, and a neighbour whose estimate is above the wake level.

## Thresholds

| Setting | Value | Reason |
| --- | --- | --- |
| Wake | Estimate above 25% of yield | See trap 1 |
| Sleep | Measured strain below 8% of yield and nearly unchanging, for 60 steps in a row | Well below wake, so no flicker |
| Minimum live time | 30 steps | Stops rapid switching |
| Wake damping | Extra damping in a just-woken segment, fading over about 10 steps | Softens the sudden pick-up of load |

**[assumption]** All four values. They are tuned against the equivalence test below.

## Joints

Joints sit between two bodies or two live segments and are built from the same parts.

- **Hinge:** two point-to-point constraints on the hinge axis, each a zero-length distance constraint between a node on one side and a node on the other. In a live lattice these are two shared nodes, as the spec says.
- **Joint limit:** an angle constraint about the hinge axis, active only past the minimum or maximum.
- **Servo:** an angle constraint toward a target, with a cap on the force it may apply per substep. The cap is the torque limit. When the load exceeds it the servo stalls; it cannot add energy because the correction is always toward the target and bounded. The target moves at a limited rate.
- **Rotor:** a servo on angular speed instead of angle, with the same cap.

Generators cap the mass ratio between a servo's body and the arm it drives, as in the spec.

A moving joint marks its neighbouring segments as triggered, so they stay live while it moves.

## Splitting into vessels

- Runs only on steps where a beam broke, on that island only.
- A union-find over unbroken beams and body membership finds the connected groups of nodes.
- More than one group means the craft has split. Each group becomes a vessel; the one containing the lowest node index keeps the original identity.
- A body cut by the split is divided with the waking formulas, so momentum is unchanged.
- An event per new vessel carries the list of its segments, so the renderer and the game can follow.

## Time warp and orbit

Putting the whole vessel on rails is "force every segment to sleep", using the same formulas. Coming off rails wakes nothing; the normal triggers take over.

## Traps

1. **A suddenly applied load peaks at about twice its steady value.** A segment that wakes at 50% of yield under a load that has just arrived can overshoot to 100% and yield when it should not. So the wake level must stay well under half of yield, and a just-woken segment gets extra damping for a few steps. This is why the table says 25%, below the spec's 30-50%.
2. **The estimate lags by one step.** A hard impact arrives in one step. Contacts therefore wake the touched segment and its neighbours immediately, before the solve, without waiting for the estimate.
3. **Light live nodes against a heavy body.** A live node of a few kilograms on a beam to a body of many tonnes is the mass-ratio problem in a new place. The body's resistance at the point is what the solver sees, which helps, and the generator's mass-ratio limit applies at seams too.
4. **Sleeping must not change the game.** If the estimate is too generous, a leg that should snap stays rigid and survives. The equivalence test below exists to catch this, and it is the most important test in the milestone.
5. **Rigid and live pieces look different.** A rigid rocket is perfectly stiff; a live one sags slightly. With the scaled-down stiffness that sag may be visible. Positions do not jump at a switch, but if the difference is noticeable in play, lower the wake level for long slender parts so they go live earlier.

## Tests

- Linear and angular momentum before and after every sleep, wake, merge and split agree to within rounding.
- Kinetic energy does not rise on any sleep, and is unchanged on any wake.
- A craft switching back and forth under a load near the threshold for ten simulated minutes gains no energy and switches no more often than the minimum live time allows.
- **Equivalence:** a set of drop, thrust and impact scenes run twice, once with sleeping on and once with everything live. The same beams break, within a stated tolerance on the load at which they break.
- **Wobble:** a tall rocket under thrust has far smaller tip movement with sleeping on than with everything live.
- A stalled servo holding a load for ten simulated minutes adds no energy. A joint limit holds at its angle.
- A craft cut in two becomes two vessels whose masses add up to the original.
- The state hash is identical across runs with sleeping on.

## What this requires of earlier milestones

| Milestone | Requirement |
| --- | --- |
| 1 | The solver runs on packed island buffers. One distance-solve function. All external forces go through one accumulator. Each beam's constraint force from the last substep is stored. |
| 3 | Skinning reads the node position buffer and does not care whether a node is live or riding. The core writes riding nodes' positions once per frame. |
| 4 | Generators emit, for every part: a segment per node, the seam beams between segments, each seam's section properties, the assembly tree, and a never-live flag for rigid parts. |
| 4 | See the ordering issue below. |

## Ordering issue with milestone 4

Milestone 4 lists a "rigid engine and pod", but rigid bodies arrive here in milestone 5. A heavy engine built as a lattice on a light truss is the mass-ratio problem the spec warns about.

Recommended: move the smallest piece of this design, a rigid body with riding nodes and no sleeping, to the start of milestone 4. Milestone 5 then keeps joints, sleeping, merging and splitting. This is a roadmap change and needs the owner's approval.

## Open at the time of writing

- Whether wrecks persist in saves (a spec open question) decides whether offsets and bent rest lengths of sleeping segments are written to the craft file.
- Whether an external rigid-body library stays for loose objects. Nothing here depends on it.
