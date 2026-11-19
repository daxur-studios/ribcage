---
type: EngineeringRecord
title: Rigid by default
description: Decisions, findings and verification evidence for milestone 5.
timestamp: 2026-11-18T12:00:00.000Z
plans: [/docs/plans/005_rigid_by_default.md]
tags: [milestone-5, rigid-bodies, joints, sleeping, vessels]
---

# ER 005: Rigid by default

## 2026-11-18 (planning)

- **[action]** Milestone 5 started. Plan 005 written by Opus from the roadmap checks, the rigid-by-default brief, the core layout rules and what milestone 4 left (ER 004, phase 4.11), against the code as it stands (`core/src/solver.rs` already has the `Endpoint` enum the brief needs; segments, seams and the tree exist only in TypeScript today). Phases: 5.1 segments in the core, 5.2 bodies and switch formulas, 5.3 riding endpoints, 5.4 engine and pod rigid, 5.5 joints, 5.6 wheel, 5.7 stress estimate, 5.8 sleep and wake, 5.9 equivalence and tuning, 5.10 vessels, 5.11 tall rocket, 5.12 page, 5.13 wrap. 5.1 to 5.11 are headless.
- **[decision]** Who builds what: **Opus** for 5.2, 5.3, 5.4, 5.5, 5.7, 5.8, 5.9 and 5.10 (new numerics, the shared kernel, energy, the estimate and the equivalence test); **Sonnet** for 5.1 (arrays, commands, hash, ABI and decoder, the 4.6 pattern), 5.6 (a generator on 5.5's joints), 5.11 (an assembly and a measurement), 5.12 (page) and 5.13 (wrap). The roadmap's row says "Opus"; this refines it per phase, and the row is left as is (it names the milestone's hard parts correctly).
- **[decision]** Roadmap check to phase: check 1 (engines, pods, wheels) in 5.6 with 5.4; check 2 (joints, stalled servo) in 5.5; check 3 (sleep and wake with momentum kept) in 5.9, built in 5.8; check 4 (merge, tall rocket wobbles less) in 5.11, seen in 5.12; check 5 (split into vessels with the right mass) in 5.10. Each is a fifth of the milestone.
- **[decision]** **ROADMAP.md is not edited by this planning commit.** Checks and weight (16) stand as written. The plan link on the milestone 5 heading and the TL;DR's "Next is milestone 5" line are left to 5.1's commit, as in milestone 4.

### What ER 004 asked for

- **[decision]** **Segment mass is stored; centre and inertia are computed at the switch.** Masses are constant (no fuel), so a stored sum is exact and cheap. Centre and inertia depend on where the nodes are now, and the brief keeps a bent segment bent by taking the offsets as they are at the switch; storing them would only add a value to keep in step.
- **[decision]** **Seam beams never become joints.** The brief already gives the rule: a beam with both ends on one body is skipped; a beam from a riding node to a live node is solved through the riding endpoint; a beam between live nodes is as now. Classifying by the endpoints' state at the start of the step covers every seam beam with no list to keep. Joints are only what a part declares (hinge, limit, servo, rotor). This answers the second gap in ER 004, 4.11.
- **[decision]** **The substep floor: legs set it while they are live; legs are not retuned here.** A leg alone needs 19 substeps (ER 004, 4.3), the same as the whole hopper, so making the engine and pod rigid lowers nothing while legs are live. The saving comes when the legs and tank sleep: in flight, and standing at rest if contacts wake on arrival only (next item). A hard landing wakes the legs and pays 19 for those steps, which is when the legs are working. Retuning the legs would move 4.8's tuned bands and the golden hop for a saving only during touchdown, so it is not worth it now; 5.4 and 5.9 measure substeps per flight phase so a later milestone can decide with numbers. This answers ER 004's oracle candidate (engine and pod, or legs too): legs too, but through sleeping, not retuning.
- **[decision]** **Contacts wake on arrival or on force above the wake level, not on resting [assumption].** The brief's trap 2 (the estimate lags a step on impact) is met by waking on a new contact or a contact force over the wake level, before the solve. A steady resting contact does not hold a segment awake; otherwise the legs, always on the pad, would stay live and keep 19 substeps on the ground forever. 5.9's drops on the pad and the rough spots are the test that this misses no hard landing.
- **[decision]** **Mass ratio: measured as effective mass at the seam point.** A rigid engine (800 kg) or pod (1200 kg) against a 6.6 kg leg node or a 120 kg tank ring node is what the solver sees through the body's weight `1/M + (r × n) · I⁻¹ · (r × n)` at that point, not the total mass. 5.3 measures a light chain on a heavy free body against the same chain on a pin; 5.4 reports the effective ratio at every hopper seam. The craft node-mass budget of 20 stays a rule on live nodes; a seam rule is added only if those numbers show trouble. Decided against the stated goals (performant, semi-realistic, deterministic) rather than asked.

### Determinism and the goldens

- **[decision]** **Sleeping is a world setting, off by default until 5.9 passes**; never-live parts become rigid by command whatever the setting. New state enters the state hash only when it exists (segment tables once a segment command ran, bodies while one exists, joints once one is added, vessel ids once a split happened). Recordings replay their own command bytes, and the committed ones send none of the new commands, so `weak-bridge.rcrd` (ABI 2), `tower-collapse.rcrd` (ABI 3) and `hard-landing.rcrd` (ABI 4) must replay unchanged at every phase.
- **[decision]** **The live hop's hash `0xd92f2aad01c0e048` at step 1320 stays valid through 5.3** (5.1 emits segment commands and must show they change no physics by this hash). **From 5.4 the flight scenarios default to rigid engine and pod**, and a second golden `hard-landing-rigid.rcrd` is recorded deliberately and logged with its hash; it is re-recorded once more in 5.9 when sleeping becomes the default, also logged. A `rigid: false` option rebuilds the all-live hop, and a test keeps it at `0xd92f2aad01c0e048`.
- **[decision]** **ABI bumps per phase that adds command kinds** (5 in 5.1), each mirrored in `src/commands.ts`, the decoder accepting every earlier ABI.

### Open, to be measured

- **[assumption]** The brief's four thresholds (wake 25% of yield, sleep 8% for 60 steps, 30 steps minimum live, 10 steps of wake damping), the explicit spin term with a clamp, and "a body lives in one island". Tuned or confirmed in 5.2, 5.8 and 5.9.
- **[assumption]** A tall rocket of four to six tanks fits the substep and craft mass budgets on the existing generators (5.11 checks before measuring wobble).

## 2026-11-18 (phase 5.1: segments, seams and the tree in the core)

Built by Sonnet. `cargo test`, `pnpm test` (224 pass, 1 skipped) and `pnpm typecheck` pass. The three goldens replay to their hashes, and the live hop still ends at `0xd92f2aad01c0e048` at step 1320 (as the physics hash; see below).

### Record layouts (ABI 5, command kinds 11 and 12)

- **[decision]** **`CMD_ADD_SEGMENT` (11).** `a` = parent segment, or `u32::MAX` for none; `b` = flags; `f[0]` = axial area, `f[1]` = bending section of the seam to the parent. Result: the new segment's index, numbered in the order added. Rejected, and nothing changed (no segment, no seam, no number used), when: the parent does not exist yet (a parent number at or past the new segment's own is the cycle case: segments are only appended and a parent must already exist, so a cycle cannot form); `b` has any bit other than bit 0 (`SEG_NEVER_LIVE`); area or bending is not finite or is below zero. A root's two numbers are checked the same way and then not stored.
- **[decision]** **`CMD_SET_NODE_SEGMENT` (12).** `a` = node, `b` = segment; result 1. Rejected, and nothing changed, when the node is dead, past the end or pinned (no mass to store), the node already has a segment (a node is assigned once), or the segment does not exist. The node's mass joins the segment's stored mass at assignment.
- **[decision]** **Arrays on the world, one per field.** Per node: `node_segment` (`u32::MAX` for none, also on slot reuse). Per segment: `seg_flags`, `seg_parent`, `seg_seam` (seam to the parent, `u32::MAX` for a root), `seg_mass`. Per seam: `seam_a` (parent segment), `seam_b` (child), `seam_area`, `seam_bending`. A beam's seam is not stored: `beam_seam(beam)` looks up the seam between its two end nodes' segments (none when either end has no segment, both ends share one, or the two are not tree neighbours).
- **[decision]** **Stored mass keeps up.** Added at assignment (`1 / inv_mass`); recomputed from scratch in index order when an assigned node is deleted or its inverse mass is changed (so a pin takes its mass out and the number never drifts). A deleted node leaves its segment, and its slot comes back with none.
- **[decision]** **`segment_centre_and_inertia(segment)`** is read-only and computed from current positions in two index-order passes: the mass-weighted centre, then `Σ m (|r|² 1 − r rᵀ)` about it, as `[xx, yy, zz, xy, xz, yz]` with the products negated. Only add, subtract, multiply and divide. `mass` is the stored mass. `None` for a missing or massless segment. Exposed as `rc_segment_props` (ten `Real`: mass, centre, inertia).
- **[decision]** **Hash.** After terrain and thrusters, only if a segment exists: segment count, then per segment flags, parent, seam and mass bits; seam count, then per seam a, b, area and bending bits; then `node_segment` for every node slot. A world that never ran a segment command hashes as before. **`physics_hash`** (new, `rc_physics_hash`) is the hash without that section, so a test can show segments change no physics.
- **[decision]** **Host.** `ABI_VERSION` 5 in `core/src/ffi.rs` and `src/core.ts`; the decoder reads ABI 2 to 5 (ABI 3, 4 and 5 only added command kinds and hash sections that exist only when their data does). Readers: `rc_segment_count`, `rc_seam_count`, `rc_node_segment_ptr`, `rc_segment_flags_ptr`, `rc_segment_parent_ptr`, `rc_segment_seam_ptr`, `rc_segment_mass_ptr`, `rc_seam_a_ptr`, `rc_seam_b_ptr`, `rc_seam_area_ptr`, `rc_seam_bending_ptr`, `rc_segment_props`, `rc_physics_hash`; in TypeScript `World.segments()`, `World.segmentProps()`, `World.physicsHash`.
- **[decision]** **`compileCraft(batch, craft, { segments })`** (in `src/parts/assemble.ts`, on by default) is `compilePart`, then the segment records, then one `setNodeSegment` per node. The tree comes from the craft's seams: a breadth-first walk from segment 0 gives each segment its parent and the seam's numbers (a segment the seams do not reach becomes a root; a seam closing a loop would be ignored, which cannot happen for an assembly tree). Records go out lowest-numbered-ready first, which for the hopper is craft order, so the core's segment numbers equal the craft's. Seam area and bending are copied to the bit.

### Two things the brief disagreed with itself on

- **[finding]** **The hop's hash cannot be the full state hash once segments exist.** The brief says segments enter the state hash once a segment command has run, and also that the hop with segment commands still ends at `0xd92f2aad01c0e048`. Both cannot hold for one number. Resolved by testing against `physics_hash`: the hop with segment commands ends at **physics hash `0xd92f2aad01c0e048`** and every node position is bit-identical to the run without them; its full state hash is `0x2cc35d25969c8cf4`, the hop's hash plus the segment section. Without segment commands the two hashes are equal and the full hash is the old one.
- **[decision]** **`prepareFlight` keeps segments off by default** (option `{ segments: true }`; `prepareHop` takes the same). Otherwise the test that recording the hop again gives the committed `hard-landing.rcrd` byte for byte would fail, because the segment records would be in the setup list. `compileCraft` itself defaults on, as asked. That recording test now compares with the stored file's ABI field (4), as the other two goldens' tests do; the golden was not re-recorded. Switching flights to segments by default is a deliberate re-record for a later phase, with its hash logged.

### Measured

- **[finding]** **The stored masses sum to 4600.000000000007, not exactly 4600** (4600 − sum = −7.3e-12, 1.6e-15 relative). The craft's own node masses sum to exactly 4600 in TypeScript, but the core only has inverse masses, so each node's mass is `1 / (1 / m)`, which is not always `m`, and the per-segment sums are added in a different order. The test asserts 1e-12 relative and logs the number. Making it exact would need the true mass in the command record, which is not worth a layout change; nothing downstream needs more than 1e-12.
- **[finding]** **Centre and inertia of every hopper segment match a direct TypeScript sum within 1e-12** (relative to the centre's size, at least 1 m, and to the inertia's trace), at rest and after 90 steps.
- **[finding]** **Bad records, both layers.** Rust (`core/tests/segment.rs`) and through the wasm boundary (`test/segment.test.ts`): unknown parent, parent equal to the new number, unknown flag bit, NaN, infinite and negative numbers, node already assigned, unknown segment, unknown node, pinned and dead nodes: each rejected; rejected ones take no number and add no seam, and the good records around them apply.
- **[finding]** **Rust tests added:** stored mass and tree, `beam_seam`, rejections, mass following deletes, mass changes and slot reuse, centre and inertia against a direct sum, a segment with no mass, hash equal to the old one until a segment exists, and `physics_hash` equal with and without segments over 30 steps.

### Not checked

- **[assumption]** Not run in the browser: nothing the page draws changed (segments are off in the flights it prepares).
- **[assumption]** No native-against-wasm comparison of the hash with segments in it (the new section uses the same `mix` helpers as the rest). 5.2 should add the segment commands to that comparison when it adds bodies.
- **[assumption]** A segment has one seam, to its parent. A craft where two parts share one segment pair through more than one join would keep only the first seam found; the assembler cannot make that today.

### For 5.2

5.2 (Opus) can start from the files alone: `core/src/segment.rs` (the arrays and `segment_centre_and_inertia`) and the hop test in `test/segment.test.ts` (the "no physics change" pattern). Keep the segment section last in the hash among today's sections, with bodies appended after it only while a body exists.

## 2026-11-18 (phase 5.2: rigid bodies and the switch formulas)

Built by Opus. `cargo test`, `pnpm test` (225 pass, 1 skipped: the existing reported-benchmark skip) and `pnpm typecheck` pass. The three goldens replay to their hashes, and the live hop still ends at `0xd92f2aad01c0e048` at step 1320. A world with no body hashes exactly as before.

### Record layouts (ABI 6)

- **[decision]** **`CMD_MAKE_RIGID` (13).** `a` = segment, `b` = a second segment to merge into the same body, or `u32::MAX`. Result: the body's slot, or rejected (nothing changed) when a segment does not exist, has no massive node, or the group is degenerate. If either segment already rides a body, that whole body joins the new one.
- **[decision]** **`CMD_RELEASE_RIGID` (14).** `a` = segment; result 1. Rejected when the segment does not exist or is not rigid. The brief allowed it "if the tests need one": they do, so the native-against-wasm hash comparison runs a release through the boundary.
- **[decision]** **Arrays on the world, one per field.** Per node: `node_body` (`NO_BODY` = `u32::MAX`), `node_offset` (body frame). Per segment: `seg_body`. Per body slot: `body_flags` (`BODY_ALIVE` = 1), `body_centre`, `body_rot` (quaternion w, x, y, z), `body_vel`, `body_omega` (world frame), `body_mass`, `body_inertia` and `body_inv_inertia` (body frame, symmetric, six numbers), `body_prev_centre`, `body_prev_rot`. The slot is the lowest segment of the group, so the slot arrays grow lazily to the segment count and no free list is needed. Member segments are not stored as a list: `seg_body` is the membership, walked in segment order.
- **[decision]** **Readers.** `rc_body_slot_count`, `rc_body_flags_ptr`, `rc_body_centre_ptr`, `rc_body_rot_ptr`, `rc_body_vel_ptr`, `rc_body_omega_ptr`, `rc_body_mass_ptr`, `rc_node_body_ptr`, `rc_segment_body_ptr`; in TypeScript `World.bodies()`, and `makeRigid` / `releaseRigid` on `CommandBatch`. `ABI_VERSION` 6 in `core/src/ffi.rs` and `src/core.ts`; the decoder reads ABI 2 to 6.
- **[decision]** **Hash.** While at least one body is alive, `physics_hash` gets a body section after everything else: the live count, then per live slot in ascending order the slot, centre, rotation, velocity, angular velocity, mass and the six inverse-inertia numbers. `state_hash` also adds, after the segment section, `seg_body` for every segment and `node_body` plus `node_offset` for every node. Bodies are physics, so they go in `physics_hash` too; a world with no live body adds nothing to either.
- **[decision]** **Own maths in `core/src/maths.rs`.** `Quat` (product, conjugate, normalise, rotate, inverse rotate, Cayley map, to matrix), `Sym3` (point inertia, determinant, inverse, rotated) and `Mat3` (product, skew, solve by cofactors). Only add, subtract, multiply, divide and `sqrt`; no `mul_add`, no library maths. `Vec3` gained `cross` and negation.

### Formulas as built

- **[decision]** **Make rigid.** Sum over the loose massive nodes of the segments, in node order, and over any body already holding one of them: mass `M`, centre `c = Σ m x / M`, momentum `P = Σ m v`, angular momentum about `c`: `L = Σ m (x − c) × v + Σ_bodies (R I Rᵀ ω + M_b (c_b − c) × V_b)`, inertia about `c` with the parallel-axis term for bodies. Then `V = P / M`, `ω = I⁻¹ L`, rotation reset to identity (offsets are world-frame `x − c`), and each rider's velocity set to `V + ω × r`. Kinetic energy can only fall (the internal relative motion is removed); positions are not touched.
- **[decision]** **Release.** Released nodes take `V + ω × r` at their current world position. A body that keeps some segments is re-summed from its riders (`refresh_body`): mass and centre `c'` from the remaining riders, `V' = V + ω × (c' − c)`, `ω` unchanged, inertia rebuilt. If the remaining group is degenerate it is released whole. If the lowest segment left, the body moves to the new lowest slot.
- **[decision]** **Degeneracy.** A group is refused (or released) when it has no mass or `det(I) ≤ 1e-9 (trace / 3)³`, which catches single nodes and straight rods, whose inverse inertia would blow up.
- **[decision]** **Step.** Each body is its own island in 5.2 and takes the same substep count as a light island: `MIN_SUBSTEPS` (or `n` under `Fixed(n)`), `h = STEP_SECONDS / n`. `step_bodies` runs after the islands are copied back and before forces are cleared. Force and torque are summed over the riders once at step start, in node order. Per substep: predict `V += h (g + F / M)`, `c += h V`, angular kick `Π += h Rᵀ τ` on body-frame momentum, spin term, then the predicted pose goes into `body_prev_*`. Derive is in delta form (`V += (c − c_pred) / h`, `ω += 2 δq_vec / h`), which is exactly zero in 5.2 because nothing corrects a body yet; then `V` is clamped to the world's max speed. Riders' world positions and velocities are written once, after the last substep.
- **[decision]** **Spin term: implicit midpoint by default** (`SpinForm::Midpoint`; `Explicit` and `Implicit` kept behind `World.spin_form` for the measurement). Solve `Π' = cay(−h ω_m) Π` with `ω_m = I⁻¹ (Π + Π') / 2` by three Newton iterations from `Π`, and turn the body by `cay(h ω_m)`. The Cayley map is a rotation, so `|Π|` and world `L` hold to rounding; energy holds to the Newton residual. Cost: one 3×3 solve per iteration.
- **[assumption]** **Angular-speed clamp 100 rad/s** (`DEFAULT_MAX_ANGULAR_SPEED`): if `|ω| > 100`, `ω` is scaled down, which can only remove energy. Picked as well above anything a player can spin a part to by thrust, and below the rate where `h ω` leaves the Cayley map's comfortable range (`h ω ≈ 0.42` at 4 substeps).
- **[decision]** **Islands see riders as fixed.** A riding node is not movable, so it does not seed or join a live island; a beam from a live node to a rider packs the rider with inverse mass 0 (a pinned end that moves with the body); a beam with both ends riding is skipped. No beam pushes a body yet (5.3), and bodies have no ground contact in 5.2.
- **[decision]** **Bad-number guard.** If a body's state is not finite after a substep, it goes back to its step-start centre and rotation with zero velocity and spin, its riders are flagged frozen, and `EVENT_FREEZE` is emitted (index = first rider, value = substep). Same contract as the islands' guard.
- **[decision]** **Riders come and go.** Assigning a node to a rigid segment calls `make_rigid` on it; deleting a rider or changing its inverse mass re-sums its body (`refresh_body`), releasing it whole if it becomes degenerate.

### Measured

- **[finding]** **Free fall.** The drop after `k` substeps equals `g h² k (k + 1) / 2` (symplectic Euler) to 1e-12 and stays within `½ g h t` of `½ g t²`; spin and spin energy are unchanged to 1e-12. The same box as loose nodes falls to within 1e-9 relative of the body: the loose nodes derive velocity from positions, the body integrates it, and the two round differently by about 1e-10.
- **[finding]** **Ten simulated minutes of spin (36,000 steps).** A 1.2 × 0.7 × 0.3 m cuboid about each principal axis and about a tilted axis: angular momentum and energy within 1e-9 relative (the bound the test asserts; seen about 3.5e-11 and 6.8e-11).
- **[finding]** **Intermediate-axis tumble** at 1, 20 and 50 rad/s: the axis flips (the test checks it does), energy stays within 1e-9 and never rises. At 97 rad/s the tumble passes the 100 rad/s clamp, which removes 19% of the energy over ten minutes and 10% of `|L|`; energy never rises (largest gain 1e-15). The test asserts only "no gain" for that clamped case.
- **[finding]** **The open question, answered with numbers** (the same cuboid, ten minutes, energy change):

  | start spin, rad/s | explicit + clamp | implicit (backward Euler) | implicit midpoint |
  |---|---|---|---|
  | 1.1 | +41% (`|L|` +19%) | −26% (`|L|` −14%) | 6.8e-11 |
  | 20 | +2161% (until clamped) | −94% | 6.8e-11 |
  | 51 | +261% (clamped) | −94% | 6.8e-11 |
  | 97 | +1% (clamped; `|L|` +112%) | −94% | −19% (clamp only) |

  The explicit term pumps energy until the clamp holds it at the ceiling: the clamp hides the gain, it does not stop it. Plain implicit is stable but bleeds spin away, which reads as debris that stops tumbling. The midpoint form is the only one that holds both, so it is the default and the explicit form is not used.
- **[finding]** **Cost.** 144,000 substeps of one body, release build: 42 / 43 / 56 ms (explicit / implicit / midpoint) at 1 rad/s, 29 / 32 / 48 ms at higher rates. Midpoint is 1.3× to 1.65× the explicit term, about 0.33 µs per body substep with the world step around it: small against a lattice island.
- **[finding]** **Switch formulas.** Making a moving, spinning, bent lattice rigid keeps linear momentum to 1e-13 and angular momentum to 1e-12 relative, never raises kinetic energy, and moves no node (bit-identical positions). Merging two bodies in two steps keeps the same. Releasing keeps momentum, spin and kinetic energy to the same bounds and moves no node; make then release in the same step is the identity on momentum. Rods and single nodes are refused; riders leaving and joining re-sum the body.
- **[finding]** **Hashes.** A body run hashes the same twice. The native-against-wasm test now runs a scene with segments and bodies through the boundary (a box and a tetrahedron, a beam to the box, a thruster; make rigid at step 10, merge at 40, release one segment at 80, the rest at 100): physics hash `0x9959877922115153` after step 61 (a body alive) and `0xf58545641d0a5073` after 120, equal in Rust and through WebAssembly.

### Not checked

- **[assumption]** The bad-number guard path is written but no test drives a body to a non-finite state.
- **[assumption]** `Fixed(n)` substeps with a body are not tested; only the rule path is.
- **[assumption]** No body contacts, no beam pushing a body, no sleep or wake triggers: all later phases.
- **[assumption]** No measurement with many bodies or a body with many riders; the write-back is one pass over riders per step and was not timed at scale.
- **[assumption]** Not run in the browser: the page draws no body yet.

### For 5.3

5.3 (Opus) can start from the files alone. `predict_body` and `derive_body` in `core/src/body.rs` are already split per substep in delta form, so moving them into the island substep loop (a body joining the island of the live nodes it touches) is a move, not a rewrite; `body_prev_*` holds the predicted pose for the riding-endpoint correction. Beams to riders are packed today as fixed ends (`inv_mass` 0) in `core/src/island.rs`; `Endpoint::Riding` replaces that.

## 2026-11-18 (phase 5.3: riding endpoints in the one kernel)

Built by Opus. `cargo test` (debug and release), `pnpm test` (225 pass, 1 skipped: the existing reported-benchmark skip) and `pnpm typecheck` pass. The three goldens replay to their hashes, the live hop still ends at `0xd92f2aad01c0e048` at step 1320, and a world with no body hashes exactly as before. New tests are in `core/tests/riding.rs`; `core/tests/midpoint.rs` follows the new endpoint API.

### Formulas as built

- **[decision]** **`Endpoint::Riding(body, local)`** in `core/src/solver.rs`, the fourth case of the one kernel. With `r = R · local` and push direction `n`, weight `w = 1/M + (r × n) · I⁻¹ (r × n)` with the world inverse inertia `I⁻¹ = R I_b⁻¹ Rᵀ` (applied as rotate, body-frame multiply, rotate back; no matrix built). A push `p = n Δλ` moves the centre by `p / M` and turns the body by `Δθ = I⁻¹ (r × p)` as a first-order quaternion push `q ← normalize(q + ½ (0, Δθ) ⊗ q)`. Beams, beam damping, ground contacts and beam-midpoint contacts (`Endpoint::Point` with a riding end shares through the body) all go through it.
- **[decision]** **Bodies in the island substep loop.** `predict_body` and `derive_body` run per substep inside the island, around the constraint passes. A body joins the island of the live nodes it touches (every riding node of one body is joined); a body with no live neighbour is an island of its own. Beams with both ends on one body are skipped when the island is packed; packing riders as fixed ends (`inv_mass` 0) is gone.
- **[decision]** **Substep rule through the same weight.** A beam to a riding node is rated by the riding weight, so a light live node on a heavy body is rated by the live node. A lone body takes `MIN_SUBSTEPS` (4), or `n` under `Fixed(n)`.
- **[decision]** **Contacts in two passes: all pushes, then all friction** (`push_ground_contact`, then `ground_friction`). Interleaving push and friction per contact let a body resting on four corners creep 0.117 m over 60 s, because each corner's friction saw the body already turned by the next corner's push. Alternating the contact order each substep was tried and rejected: it turned the rocking into yaw. Node-only worlds give the same bits as before (a node's friction does not depend on another node's push).
- **[decision]** **Friction uses the riding weight along the slip direction**, not along the normal.
- **[decision]** **Damping kicks both the body's velocity and its spin** (`V += p / M`, `ω += I⁻¹ (r × p)`), so a damped beam to a body removes energy from both.
- **[decision]** **Split recovery also shifts the body's predicted pose**, so a contact's penetration recovery does not reappear as velocity in `derive_body`.
- **[decision]** **The body speed clamp sits in the island loop**, after derive, as for nodes.
- **[decision]** **A body freeze reports the island root** (`EVENT_FREEZE` index), the same contract as a live island.
- **[decision]** **Riders are placed once per step, before the break check**, so a beam's strain to a riding node is measured at the body's final pose.
- **[decision]** **The 5.2 spin term is unchanged** (implicit midpoint, 100 rad/s clamp).

### Measured

- **[finding]** **Pendulum.** A live pendulum hung from a pinned heavy body against the same pendulum on a pin: positions differ by at most 9.1e-6 m over 3 s, both at 32 substeps.
- **[finding]** **Mass ratio 800 kg : 6.6 kg.** A light chain hung from a free 800 kg body settles to a stretch of 4.787279999672e-3 m; on a pin, 4.787279999791e-3 m (2.5e-11 relative). Both use 10 substeps; the body drifts 2.6e-4 m. XPBD statics adds up to `h² w` to the compliance, so the test brackets the pin's stretch between the two rather than asserting equality.
- **[finding]** **The open question, 5.3's part.** At 800 : 6.6 (effective ratio over 120) a body converges like a pin, to rounding, at the same substep count. 5.3 shows no need for an effective-mass rule at seams on top of the craft node-mass budget. 5.4 reports the effective ratio at each real hopper seam and closes the question.
- **[finding]** **Cube drop.** The rigid cube and the same cube as a stiff lattice touch down on the same step (24). Rebound: 6.3 cm rigid, 18 cm lattice. Rest height 0.60004 m rigid, 0.59999 m lattice (both within the radius). Energy gain at rest over 60 s: 4e-14 J.
- **[finding]** **Four corners.** Creep 3.1e-11 m and yaw 3.8e-11 rad over 60 s (was 0.117 m before the two-pass contacts).
- **[finding]** **Yield and break.** A beam from a riding end to a live end and one between two live ends both yield at 1902 N and break at 1928 N.
- **[finding]** **Momentum.** A body plus a live lattice with no external force: worst drift 91 rounding units over 10 s. It wanders rather than grows and scales with distance from the origin, so it is rounding. The test bound is 1000 rounding units.
- **[finding]** **`Fixed(n)` with a body** (left unchecked by 5.2). The body centre is off the 32-substep run by 13.5 cm at 1 substep, 6.9 cm at 4 and 4.5 cm at 8; no step froze. The test checks that the gap shrinks as the count rises.
- **[finding]** **Native against WebAssembly body scene.** The beam from node 1 to node 8 now pushes the body, so the scene changes: `BODY_SCENE_61` `0x9959877922115153` → `0x06024ffac786af7b`, `BODY_SCENE_120` `0xf58545641d0a5073` → `0x722722cfee9354f3`. Updated in `core/tests/ffi.rs` and `test/wasm.test.ts`; Rust and WebAssembly agree.

### Not checked

- **[assumption]** The lattice cube rebounds 18 cm against 6.3 cm for the rigid cube. It is not beam damping (600 → 6000 changed the rebound by under 5%); the cause is not traced. The test only checks that both bounce and keep less than a third of the fall. 5.9 (equivalence) should look at it.
- **[assumption]** The body bad-number guard path is still not driven by a test.
- **[assumption]** No timing with many bodies or bodies with many riders.
- **[assumption]** Joints through the riding endpoint are 5.5; not exercised here.
- **[assumption]** Not run in the browser: the page draws no body yet.
- **[oracle-candidate]** Why does a stiff lattice cube keep about three times the rebound of the same cube rigid, when damping is not the cause?

### For 5.4

5.4 (Opus) can start from the files alone. Every beam and contact to a body goes through `Endpoint::Riding`; `CMD_MAKE_RIGID` on the never-live segments at spawn is all `compileCraft(..., { rigid: true })` needs to send. Report the effective mass `1 / w` at each seam node along each seam beam and compare with the 800 : 6.6 case measured here; if every seam is at or below that ratio, close the open question with "no rule needed".

## 2026-11-18 (phase 5.4: the hopper's engine and pod rigid)

Built by Opus. `cargo test`, `pnpm test` (230 pass, 1 skipped: the existing reported-benchmark skip) and `pnpm typecheck` pass. The flight and landing scenarios now spawn the hopper with its engine and pod as rigid bodies. New measurements are in `test/rigid.test.ts`; the bands run rigid in `test/flight.test.ts` and `test/landing.test.ts`.

### Formulas as built

- **[decision]** **`compileCraft(..., { rigid: true })`** appends one `CMD_MAKE_RIGID` per never-live segment after the segment records, in core segment order. A never-live segment whose parent is also never-live is made rigid with the parent, so a chain of never-live segments becomes one body. The hopper has none, so it gets two bodies: the engine (core segment 4, body slot 4, 427.5 kg) and the pod (segment 5, slot 5, 630.2 kg). The setup is accepted with nothing rejected.
- **[decision]** **`rigid` needs the segment commands.** `segments: false` together with `rigid` throws. In the flight options, `segments` defaults to the same value as `rigid`, so `rigid: false` gives the all-live setup byte for byte, and the all-live hop still compiles to `0xd92f2aad01c0e048` at step 1320.
- **[decision]** **The default is rigid** (`prepareFlight`, `prepareDrop`, `prepareHop`, and therefore the craft page and the assembly panel's stand height). The legs are not retuned.
- **[decision]** **The rigid golden.** `test/fixtures/hard-landing-rigid.rcrd` was recorded on purpose with `RIBCAGE_UPDATE_RIGID_FIXTURE=1`. It is 106,876 bytes and ends at hash `0x5a1c383b76d89ea6` at step 1320. The craft page's hop test now checks against it. `hard-landing.rcrd` stays as the all-live regression.
- **[decision]** **Rest band for the 7 m/s pad case only: 0.5 m/s** (`PAD_7_REST_SPEED`; other cases keep 0.25). This case is "recorded, not required". With every leg crushed, the craft stands on the engine's bottom ring plus the feet and keeps a bounded wobble. Rigid moves 0.30 m/s at rest against the old 0.25 band. The other bands are unchanged.

### Measured

- **[finding]** **Hops.**

  | | Hash at 1320 | Broken | Yields | Legs lost | Tilt | Substeps max | ms/step |
  |---|---|---|---|---|---|---|---|
  | All-live | `0xd92f2aad01c0e048` | 43 | 78 | 1001 | 102° | 19 | 1.29 |
  | Rigid | `0x5a1c383b76d89ea6` | 40 | 76 | 1001 | 103° | 19 | 1.45 |

- **[finding]** **4.8's bands, rigid.**
  - Standing and 2 m/s pass.
  - 3 m/s: no beam breaks on any of the five rough spots, and tilt is 0–1°.
  - 7 m/s: all five rough spots lose a leg and tip over (99–104°). rough-a lost 0101 (40 broken), rough-b 1100 (41), rough-c 1010 (34), rough-d 1001 (39), rough-e 0101 (38).
  - The 30 scatter spots and 4.7's lift-off checks pass.
- **[finding]** **The 7 m/s pad case.**
  - 24 beams break and no leg is lost.
  - The craft settles on 7–9 nodes of the engine's bottom ring plus the feet.
  - The pod and engine wobble against each other in xz: about 5 mm all-live and 10 mm rigid.
  - The pod's peak speed stays flat from 40 s to 120 s: 0.18 m/s all-live, 0.29 m/s rigid. It is a limit cycle that does not grow, not an instability.
- **[finding]** **No internal strain.** Over the whole rigid hop, the 36 inner beams of the engine and the 28 of the pod change length by at most 3.3e-15 relative, and their strain ratio stays at or below 1.2e-13 of yield. None of them yields or breaks.
  - "Inner" means both ends are nodes of the body's own segment. The part's nodes that were merged into the tank keep the tank's live segment, so they are not part of the body.
- **[finding]** **Effective mass ratio at the seams, at spawn.** The ratio is the body's `1 / w` along each seam beam against the live node's mass, with `w = 1/M + (r × n) · I⁻¹ (r × n)`. `I` is summed from point masses about the centre, as the core sums it, and the body's mass is checked against the core's.
  - Engine: 40 seam beams, ratio 0.88 to 4.02 (for example, 190.8 kg against a 102.5 kg tank node).
  - Pod: 24 seam beams, ratio 1.84 on every one (about 221 kg against 120.2 kg).
  - The 5.3 case was 800 : 6.6, or 121. Every hopper seam is at least 30 times below it.
- **[finding]** **The open question: no rule needed.** Every seam is far below the ratio 5.3 showed converging like a pin, and the bands pass rigid. The craft node-mass budget (20) on live nodes stays the only rule. The question is closed in the plan.
- **[finding]** **Substeps and ms per step by phase, rigid against all-live** (`test/rigid.test.ts`, after a warm-up run).
  - Standing is steps 60–359 of a craft placed on the pad with the engine off.
  - Lift-off is hop steps 0–59.
  - Coast is from the cut (step 220) to the first ground touch.
  - Touchdown is the 60 steps after the touch.

  | Phase | Steps | All-live substeps (max / mean) | All-live ms | Rigid substeps (max / mean) | Rigid ms |
  |---|---|---|---|---|---|
  | Standing | 300 | 19 / 19.00 | 1.49 | 19 / 19.00 | 1.23 |
  | Lift-off | 60 | 19 / 19.00 | 1.40 | 19 / 19.00 | 1.21 |
  | Coast | 47 | 19 / 19.00 | 1.38 | 19 / 19.00 | 1.19 |
  | Touchdown | 60 | 19 / 19.00 | 1.36 | 19 / 19.00 | 1.15 |

  - The substep counts are identical, as the plan predicted: the live legs set the floor of 19.
  - Timing is noisy from run to run (the probe hop above measured rigid 12% slower; this run measured it 13–17% faster). Read it as "about the same"; skipping 64 of 612 beams is offset by the body work. The saving comes in 5.8, when legs and tank sleep.

### Not checked

- **[assumption]** Timing was measured on one machine in Node, with no repeated runs or spread. The difference between rigid and all-live is within the noise.
- **[assumption]** The seam ratio was computed at spawn only. The body's inertia rotates with it, but the ratio's size depends on `r` and `n`, which change little while the seams hold.
- **[assumption]** The cause of the bounded wobble in the 7 m/s pad case is not traced. It appears all-live too, so it is not caused by the bodies; rigid only doubles its size.
- **[assumption]** Not run in the browser. The page plays the rigid hop by default, but it does not draw the bodies differently until 5.12.
- **[oracle-candidate]** Why does a craft resting on a ring of nodes plus its feet keep a bounded limit cycle (0.18–0.29 m/s peak) instead of settling, when one standing on its feet settles?

### For 5.5

5.5 (Opus) can start from the files alone. The engine and pod are bodies in every flight scenario. Joints go through `Endpoint::Riding` as beams do. `test/rigid.test.ts` shows the per-phase measurement to repeat with joints. Both hop goldens must keep replaying.

## 2026-11-18 (phase 5.5: hinges, limits, servos and rotors)

Built by Opus. `cargo test` (134 pass, 4 ignored), `pnpm test` (230 pass, 1 skipped) and `pnpm typecheck` pass. weak-bridge, tower-collapse, hard-landing and hard-landing-rigid replay to their hashes; the all-live hop still ends at `0xd92f2aad01c0e048` and the rigid hop at `0x5a1c383b76d89ea6`, both at step 1320. ABI 7 (two command kinds and four readers); ABI 6 recordings still read.

### Formulas as built

- **[decision]** **A joint is six endpoints through the one kernel:** `a0, a1, ra` on side A (the frame) and `b0, b1, rb` on side B (the moving part). Each is a node, a pinned node or a rider, so body–body, body–live and live–live are one code path. The hinge is two zero-length `solve_distance` constraints, `a0–b0` and `a1–b1`, with no compliance.
- **[decision]** **The angle** is B's turn about `u = (a1 − a0)/|a1 − a0|`: each reference is projected off the axis (`pA`, `pB`), `raw = atan2(u · (pA × pB), pA · pB)` and `φ = wrap(raw − zero)`, with `zero` measured when the joint is added. The core gains its own `atan2` (an odd series on `[0, tan(π/8)]` after two range halvings; at most about 9e-16 from the library over the test sweep). A reference on the axis line has no angle, so `add_joint` rejects it.
- **[decision]** **The drive and the limit are a couple, not a rotation.** A correction `Δλ` pushes `rb` along `+t_B` and `b0` along `−t_B` by `Δλ / L_B`, and `ra` along `−t_A` and `a0` along `+t_A` by `Δλ / L_A` (`t` the tangent at each reference, `L` its lever), with `Δλ = −C / (w_A/L_A² + w_B/L_B²)`. Equal and opposite pairs keep linear momentum: the free falling pair keeps it to 3.8e-15 of the fall's. Rotating a body side about the hinge line instead was rejected: it breaks momentum for a free pair.
- **[decision]** **Servo:** the aim is `φ0 + clamp(wrap(target − φ0), ±rate · h)`, with `φ0` the angle at the substep's start. **Rotor:** the aim is `φ0 + speed · h`. The drive's `Δλ` summed over the substep is clamped to `±cap · h²`, so its torque (`Σ Δλ / h²`) never exceeds the cap. The aim is relative to `φ0`, so a stalled servo never stores a growing error to release later.
- **[decision]** **Limit:** a hard, uncapped correction toward `min` or `max`, only when `φ` is past it. Limits sit in `−π ≤ min ≤ 0 ≤ max ≤ π`, the angle at add time being 0.
- **[decision]** **`JOINT_PASSES = 8` local passes per substep** over [hinge points, drive, limit]. One pass leaves a body side short: the couple turns a body about its centre, which moves its axis nodes, and the hinge points pull part of the turn back. Measured over 4, 8 and 16 passes:

  | Passes | Door hinge gap (m) | Limit overshoot (rad) | Body servo lag (rad) | Rotor speed error (rad/s) |
  |---|---|---|---|---|
  | 1 | 1.2e-4 | 1.6e-4 | 1.1e-1 | 1.2e-2 |
  | 4 | 5.3e-7 | 2.5e-5 | 2.0e-2 | 2.0e-3 |
  | 8 | 3.7e-9 | 7.9e-6 | 8.7e-3 | 5.5e-4 |
  | 16 | 9.5e-13 | 1.8e-6 | 3.7e-3 | 8.2e-5 |

  Eight passes meet every bound at a small cost (joints are few; each pass is two point constraints and two angle measures). Lattice sides converge in one pass; the cost is only for body sides.
- **[decision]** **Hash:** with one joint or more, `physics_hash_state` adds the count, then per joint its six nodes and generations, mode, min, max, target, rate, cap and zero; `state_hash` adds the per-segment triggered flags. With no joint neither changes, which is why every golden still replays.
- **[decision]** **Triggered:** a joint whose angle moved more than `JOINT_MOVE_EPS = 1e-4` rad in a step marks the segments of `a0` and `b0` (`seg_triggered`, cleared each step, read by 5.8).
- **[decision]** **Commands:** `CMD_ADD_JOINT` (`a = a0`, `b = b0`, `f0..f3 = a1, b1, ra, rb` as whole node numbers, `f4, f5 = min, max`; result: the joint's id) and `CMD_SET_JOINT_TARGET` (`a = joint`, `b = mode`, `f0..f2 = target, rate, cap`). Rejected: dead, repeated or out-of-range nodes, all six on one body, limits outside their ranges, a reference on the axis, unknown joints or modes, non-finite values, a negative rate or cap, a servo target past ±π.

### Measured

- **[finding]** **Door** (one body, 2 m vertical hinge, pushed for 1 s at its far corner along every axis, then free for 4 s; it turns to 3.14 rad): worst hinge gap 3.7e-9 m, worst drift of any door node along the axis 2.3e-9 m. The stated bound is 1e-6 m for both.
- **[finding]** **Limit:** held at 0.3000079 rad under twice gravity's torque (bound 2e-5); one second after the push reverses, at −1.40 rad.
- **[finding]** **Servo** (body arm to 1 rad at 0.5 rad/s, cap 200 N·m): never ahead of `0.5 t`, behind it by at most 8.7e-3 rad (under 2% of the rate); held within 2.9e-5 of 1 rad from 2.5 s. A live arm: tracking 5.0e-4, hold 6e-16.
- **[finding]** **Stall** (arm of 8.30 kg asked for π/2 with a cap of half `M g d`, 23.84 N·m, for 10 minutes): it rests at 0.52364 rad (gravity meets the cap at asin 0.5 = 0.52360) and never rises above 0.550. `E = KE + PE − τ_cap · φ` over minutes 1 to 10 moves within [−1.4e-2, +0.0] J of its value at one minute: it only loses energy. KE + PE moves by 1.2 J over the same window, as the arm settles.
- **[finding]** **Peak drive torque against the cap:** 1.000000000 of the cap at most in every test (servo, stall, rotor, the four pairs), within the 1e-12 rounding slack.
- **[finding]** **Rotor** (wheel, I = 4.025 kg·m², 2 rad/s, cap 5 N·m): within 5.5e-4 rad/s of 2 under a 3 N·m brake; under 8 N·m it slows at 0.7453 rad/s² against `(8 − 5)/I = 0.7454`.
- **[finding]** **Pairs** (two boxes hinged on a shared edge, falling, servo to 0.5 rad at 1 rad/s): at 0.25 s the angle is 0.2402 (body–body), 0.2385 (body–live and live–body) and 0.2386 (live–live); worst hold error from 1 s is 3e-14, 3.7e-4, 2.4e-3 and 1.3e-3 rad. The live lattices have undamped stiff beams, so they ring a little under the drive; the bound is 5e-3.

### Not checked

- **[assumption]** Timing with joints is not measured: no flight scenario has a joint yet (5.6's wheel is the first). With no joint the solver's joint loops are empty and nothing else runs.
- **[assumption]** One hinge per joint pair of sides; chains of joints on one body (a leg with knee and hip) are not tested. Each joint is solved in its island in add order, so a chain converges like any Gauss-Seidel chain.
- **[assumption]** A body side lags its servo rate by up to 2% (5% in the free pair at 0.25 s), from the passes not fully converging. It does not accumulate past the target, and the hold converges.
- **[oracle-candidate]** Is there a cheap effective mass for a body side of a joint (the inertia about the hinge line rather than the centre) that makes one pass converge, while still keeping momentum for a free pair?
- **[assumption]** Not run in the browser; the TypeScript side is covered by `pnpm typecheck` and the replays only.

### For 5.6

5.6 (Sonnet) can start from the files alone. Add the wheel with `Commands.addJoint` (or `World::add_joint`) between the chassis body and the wheel body, then `setJointTarget(joint, JOINT_ROTOR, speed, 0, cap)`; `World.joints()` reads each joint's angle, torque and per-step peak. `core/tests/joint.rs` has the `wheel` and `brake` helpers to copy.

## 2026-11-18 (phase 5.6: a wheel on a rotor)

Built by Sonnet. `cargo test` 134 passed, 4 ignored; `pnpm test` 234 passed, 1 skipped (235); `pnpm typecheck` clean. The four goldens replay and both hops end at their hashes (0xd92f2aad01c0e048 all-live, 0x5a1c383b76d89ea6 rigid, step 1320); no core change.

### As built

- **[decision]** `src/parts/wheel.ts`: a wheel is a ring of 4 to 24 rim nodes in the xy plane, two hub nodes on the axle, beams along the rim, rim to both hubs and hub to hub, one never-live segment. A wheel is rigid when compiled with `rigid: true` (the default), so no rim beam is solved.
- **[decision]** The rim is turned half a node step (`halfTurn`, default on, by half-angle sqrt formulas, since the angle table has no 7.5°). Nodes are spheres against the ground, so a 24-gon with a vertex down balances on one node, an unstable rest that crept with growing speed; turned half a step it rests on two nodes, which is stable. This is what makes the no-creep test pass.
- **[decision]** `src/parts/rover.ts`: the chassis is a generated lattice (8 box corners, 4 axle stubs, each stub braced to its corner, the top node above and the other bottom corner on its side). Wheels are joined by a hinge (chassis corner, stub, top node as reference; wheel hubs, rim node 0 as reference), not by the ring assembler, since a hinge is not a weld.
- **[decision]** Each wheel is its own never-live segment, a child of the chassis with no seam area. `driveRover` gives the +z side's rotors the negative speed, because their hinge axis points the other way; positive speed rolls toward +x.
- **[decision]** Rolling radius is the rim radius, not rim plus node radius: a rim of spheres travels its perimeter per turn, and measured speed does not change with the node radius.

### Measured

- **[finding]** Rest (340 kg chassis, four 30 kg wheels, radius 0.4, 24 rim nodes): creep from 2 s to 3 s is 7.4e-7 m (bound 1e-3 m), 4 bodies.
- **[finding]** Rolling, mean speed over the last two driven seconds against rotor speed × radius (cap 150 N·m per wheel): 1 rad/s 0.3976 against 0.4000 (slip 0.6%); 2 rad/s 0.7952 against 0.8000 (0.6%); 4 rad/s 1.418 against 1.600 (11.4%); 8 rad/s 3.061 against 3.200 (4.3%). Stated bounds: 5% at 2 rad/s, 20% elsewhere.
- **[finding]** Stopping: after the rotors are set to 0 the rover moves at most 2.2e-2 m in the last second (bound 5e-2), 1.2e-3 m at 4 rad/s.
- **[finding]** The drive's peak torque is 1.000000 of the cap in every run, never above.
- **[finding]** Worst rim beam strain ratio 5.4e-13, so the rim beams are not solved.
- **[finding]** Same script twice gives the same hash.
- **[finding]** Cost: 0.33 ms per step with the rover (16 substeps, 4 bodies), measured in Node on this machine; there is no other scenario to compare against here.
- **[finding]** Slip is not monotone in rotor speed (11% at 4, 4% at 8, under 1% at 1 and 2). With the torque pinned at the cap (150 N·m) the rover at 4 rad/s was accelerating, not at steady speed, in the early measurements; the loss is contact slip and polygon rolling resistance, not the rotor (the wheel's relative rotation reaches the target within 2%). I did not rework the solver.

### Not checked

- **[assumption]** Not run on the page; no visual demo (the plan says skip; a rover clip may join 5.12).
- **[assumption]** Steering, suspension and the rough ground are not tested; the rover runs on the flat part of the field only.
- **[assumption]** Only 24-node wheels are tested; the lower counts are built but untested and will be bumpier.
- **[oracle-candidate]** Why does a rolling polygon wheel on a torque-pinned rotor slip 11% at 4 rad/s but 0.6% at 1 to 2? Is it the one friction coefficient with a positional cancel, or the joint passes?

### For 5.7/5.8

- 5.7 (Opus, the stress estimate, read-only) can start now; the rover is a craft with joints to run it on.
- 5.8 should expect a hinge to mark both of its sides' segments as triggered, so a driven wheel is never put to sleep as barely loaded while its rotor moves.

## 2026-11-18 (phase 5.7: the stress estimate, read-only)

Built by Opus. `cargo test` 138 passed, 5 ignored; `pnpm test` 238 passed, 1 skipped (239); `pnpm typecheck` clean. The four goldens replay and both hops end at their hashes with the estimate on (0xd92f2aad01c0e048 all-live, 0x5a1c383b76d89ea6 rigid, step 1320). One full `pnpm test` run of six had one failure I could not reproduce in five reruns; most likely the new timing check under parallel load, since made best-of-three.

### As built

- **[decision]** `core/src/estimate.rs`, off by default. When on, each step it records the velocities before the step, the forces from the one force door (with thrust), and the contact impulses the solver applied, then cuts the assembly tree at every seam: per node `m (v - v_prev) / dt - (m g + force + contact / dt)`, summed per segment, accumulated from leaves to root (a parent's index is always below its child's), moment taken about the seam centre (the mean of the seam beams' midpoints, or the child's centre when there are none). Beams from a segment node to a node with no segment, or a pinned one, are loads from outside.
- **[decision]** The core has no Young's modulus or section per seam (only compliance and yield per beam), so "strain from axial area and bending section" is done as a stiffness spread: the child moves rigidly by (d, θ) against the parent, each seam beam stretches by `g_i · x` with `g_i = [u; (r × u) / L]`, the 6×6 system `Σ k_i g_i g_iᵀ x = -[F; M / L]` is solved (Gaussian elimination with partial pivoting, a 1e-9 trace regulariser), and `T_i = k_i g_i · x`, `k_i = 1 / compliance`. A beam the spread puts past its yield force is held at it and the rest spread again (at most 8 rounds), as the solver's beams yield. The seam's ratio is the worst `|T| / F_yield`; a segment's is the largest of its seams'.
- **[decision]** Read-only by construction: the solver change is output only (the ground push and friction also return what they applied, the old functions are wrappers, bit-identical), and the contact record is sized only when the estimate is on. Turned on by `rc_set_estimate`, not a command, so recordings and hashes are untouched; ABI 7 → 8 adds only the readers (`rc_est_*`), and the decoder accepts 3 to 8.
- **[decision]** Compared against the same seam beams' `last_lambda` (spring only, last substep, capped at yield); a beam with both ends on one body is not solved and is left out of the measurement. Comparisons count steps where the seam carries at least a tenth of its run's peak; below that both are noise around zero.
- **[decision]** A beam that skips a seam (both ends not on the two sides of one seam) is ignored by the spread, so a closed loop reads high, never low (tested with a strut over three seams).
- **[decision]** Per vessel is per world until 5.10 makes vessels.

### Measured

Ratio is estimate / measured. Bounds are in `test/estimate.test.ts` and `core/tests/estimate.rs`.

- **[finding]** Cantilever truss, one segment per frame, static: the seam force equals the weight beyond it plus the tip load within 1%, and equals the beams' own resultant within 1%; the worst ratio per seam is within 12% (bound 15%), reading low.
- **[finding]** Hopper standing (all-live): median 0.950, p5 0.837, p95 1.014; peak -17.6% to +0.7%. Bounds: median 0.9 to 1.05, peak -20% to +5%.
- **[finding]** Hop, powered climb: all-live median 0.940 (p5 0.465), peak -53.4% to +4.6%; rigid median 0.941 (p5 0.818), peak -18.8% to +3.9%. Bounds: median 0.9 to 1.05, peak above -60% (all-live) or -25% (rigid), high side under +10%.
- **[finding]** Drops (all-live): 3 m/s median 0.951, peak -37.0% to 0%; 7 m/s median 0.861, peak -37.4% to +0.1%. Bounds: median 0.8 to 1.05, peak -45% to +5%.
- **[finding]** Hop touchdown: all-live peak high +44.1%, rigid +6.7% (before the yield cap all-live read +124%). Bounds +50% and +10%. Single steps differ more: the estimate is a step average with the dampers, the measurement the last substep's spring.
- **[finding]** Rover with its chassis cut front and rear (16 beams across): with a 4 kN push up on one hub (meant as a rock; see the launch finding below, so read it as a large uneven load) median 0.724, peak -15.3%; driven at 2 rad/s median 0.678, peak 0.390 against 0.416 of yield (-6.1%); stopping median 0.976, peak +3.6%. Bounds: rock and driven peaks -20% to +5%, stopping median 0.9 to 1.1. The driven chassis seam peaks at 0.42 of yield, worth knowing for 5.8.
- **[finding]** Cost on the all-live hop, best of three: about 1.8 ms/step off and 1.85 on, 0 to 5% (bound 25%).
- **[finding]** The cut changes no motion: the rover cut in two ends on the stock rover's positions bit for bit. Its physics hash is not equal, because a body's slot is its lowest segment index and the cut moves the wheels from slots 1-4 to 2-5; the hash covers the slots.

### Not checked

- **[finding]** The free-body cut cannot see self-equilibrated stress. After a beam yields, the stress locked in by the plastic set crosses no cut: the hop's rest after touchdown reads a median of 0.31 (all-live) and 0.19 (rigid) of the measured value. Standing, the rover's indeterminate frame carries a self-balanced stress under its own weight (7.3e-4 of yield) that the estimate reads as 2.5e-5. Both are asserted as known limits.
- **[finding]** The plan's "never below the measured strain by more than a stated margin at any step" is not met: single-step peaks read low by up to 20% (rigid) and 53% (all-live), and p5 of single steps under impact is 0.1 to 0.5. 5.8 should wake on its triggers (contact arrival, thrust change, joint motion) and use the estimate with a margin (a wake level at half the sleep level's distance to yield, say) for the steady case where it is within 10%. I did not rework the solver's damping split to close this.
- **[assumption]** Not run on the page; spin terms (`I ω̇ + ω × I ω`) of rigid bodies enter only through their nodes' velocities, which is exact for riding nodes.
- **[finding]** The "rock" push launches the rover. 2 kN up on each of wheel 0's two hub nodes (4 kN, under the rover's 4.5 kN weight) lifts its centre of mass from 0.58 m to 1.20 m in the first 0.5 s, while the wheels are still on the ground; airborne, it decelerates at -1.1 m/s², which is 4 kN against 460 kg exactly, and it peaks near 4 m. So the push itself is applied right and the extra upward momentum comes in through ground contact with the rigid wheels. This belongs to 5.3/5.6 (contacts on riding nodes), not 5.7, so I did not rework it. The estimate is compared within the same run, so the bounds still hold; the scenario is not a rock. Found while recording the clip.
- **[oracle-candidate]** How should a sleeping segment carry a locked-in plastic stress so a later wake test is not low by it: store the seam's residual λ at sleep and add it to the cut, or a different measure?

### For 5.8

- Read `world.est.seg_ratio` (per segment) after `step`; it is the larger of the segment's seams. Expect it 5 to 20% low at peaks and blind to locked-in stress.
- The estimate needs the contact record on, which costs a little; switch it on with sleeping.


## 2026-11-18 (phase 5.7 follow-up: the rover's "launch" is physics)

Done by Opus, asked to fix the launch found while recording the 5.7 clip. There was nothing to fix in the core; the finding above was wrong. No core change, no hash moved.

- **[finding]** Correction to the 5.7 launch finding. The 4 kN push held on wheel 0's hub tips the rover about the diagonally opposite wheel, which stays on the pad as a pivot while the other three rise. The pivot's normal force is what adds the upward momentum, and it does no work. Over the push, the mechanical energy gained (node kinetic plus potential) is within -1.0% of the push's work (14.3 kJ against 14.5 kJ over 2.3 s), with no point where the gain runs ahead of the work. So the contacts inject no energy.
- **[finding]** The tipping push from the settled contact points, hub nodes and centre of mass is 2.255 kN, about half the 4.51 kN weight, as for lifting one corner of a table. In the sim the rover holds at 2.1 kN (wheel 0 up 13 mm, chassis flex), creeps at 2.2 kN and tips at 2.25 kN. That is within 2% of the static answer.
- **[decision]** `test/rover.test.ts` gains "a corner push tips the rover at about half its weight, and the contacts add no energy". It asserts the following:
  - At 2.1 kN, wheel 0 rises under 3 cm.
  - At 2.3 kN, wheel 0 rises over 0.3 m.
  - At 4 kN, the energy gained is between 97% and 100.5% of the work.
  The 5.7 stress test keeps its 4 kN "rock", since its bounds were measured on it, and the comment now says it flips the rover.
- **[finding]** Not this question, seen on the way: the all-live rover (`compileRover(..., { rigid: false })`) does not rest. Its wheel beams pass 5x yield within five steps of the start, and it bounces with its centre of mass near 1.2 m instead of 0.58 m. No test uses it; the wheels are rigid by default. This belongs to 5.6, and I did not rework it.


## 2026-11-19 (phase 5.8: sleep and wake)

Built by Opus. `cargo test --release` 140 passed, 5 ignored; `pnpm test` 245 passed, 1 skipped (246); `pnpm typecheck` clean. The four goldens replay with sleeping off (weak-bridge at ABI 2, tower-collapse at 3, hard-landing at 4, hard-landing-rigid), and both hops end at their hashes at step 1320 (0xd92f2aad01c0e048 all-live, 0x5a1c383b76d89ea6 rigid). ABI 8 → 9 (`CMD_SET_SLEEPING` = 17, wake and sleep events, `rc_sleep_*` readers); the decoder reads 2 to 9.

### As built

- **[decision]** `core/src/sleep.rs`, off by default, switched on by `CMD_SET_SLEEPING` (recorded, so a replay sleeps the same way). It switches the stress estimate on with it, since the estimate needs the contact record. Turning sleeping off wakes every sleeper with reason 7 (`WAKE_OFF`). The sleep state enters the hash only once the command has run, so every earlier hash is unchanged.
- **[decision]** Managed segments are the live ones and the never-live rigid ones the flight made at spawn. A segment with a joint node never sleeps (5.5's joint motion would wake it every step anyway). Rigid segments the host made by hand are left alone.
- **[decision]** The reading per segment, against yield:
  - a live segment reads its measured ratio (the worst `|λ| / λ_yield` of its beams and seam beams);
  - a never-live rigid segment reads its estimate × 1.25 (`ESTIMATE_MARGIN`);
  - an asleep segment reads the measured value it had when it slept, plus 1.25 × the change in its estimate since then. Its reference estimate is retaken over the first 10 steps after any switch (`SETTLE_STEPS`), because merging a whole stance into one body redistributes the foot loads for about 8 steps (a leg seam's estimate jumps from 0.057 to 0.40 and back).
  The estimate alone could not be used: it reads the standing legs at 0.057 of yield while they carry 0.197 inside the segment (5.7 found it blind inside a segment), and the raw estimate × 1.25 woke the tank the step after the whole craft merged.
- **[decision]** Thresholds: wake over 25% of yield (`WAKE_LEVEL`, the brief's); sleep under 20% (`SLEEP_LEVEL`), and its reading changing by under 1% of yield per step, for 60 steps (`QUIET_STEPS`); live at least 30 steps after a wake (`MIN_LIVE_STEPS`). The brief's 8% sleep level was changed to 20%: the standing legs carry 19.6% of yield under the craft's weight, so at 8% the hopper would never sleep whole. 20% is four fifths of the wake level, which leaves a band of 5% of yield for hysteresis. 5.9 tunes all four against its equivalence test.
- **[decision]** Wake triggers, tested before the solve each step:
  1. contact: a node of the segment comes into the range test it was out of last step, or closes on the ground faster than 0.25 m/s;
  2. impact: its contact force over the wake level;
  3. thrust: a throttle change on a thruster that rides it;
  4. joint: the joint-motion mark from 5.5;
  5. estimate: the reading over the wake level, after the step;
  6. neighbour: a seam neighbour woke or reads over the wake level.
  A trigger spreads to the segment's seam neighbours in the same pass. Host forces are not a direct trigger; the estimate catches them (the cycled-load test below is exactly that case).
- **[decision]** After any switch, bodies are rebuilt by union-find over segments, along unbroken beams between asleep segments, so touching rigid segments merge into one body. Each body is made with 5.2's sum, so the brief's formulas hold on every switch. Events go out in the order of the switch passes (wake pass, then sleep pass), segment order within a pass: rule 8.
- **[decision]** Wake damping: a woken segment's beams get damping × (1 + 4 · left / 10), fading over 10 steps. The factor is a placeholder; 5.9 tunes it.
- **[decision]** Every switch pass writes a `SwitchCheck` (momentum, spin and kinetic energy before and after over the nodes and bodies involved), read by `World.sleep().checks`; both test suites check every one.

### Measured

- **[finding]** Two rigid segments sharing a seam (the ffi body scene, `core/tests/sleep.rs`) become one body of 100 kg (8 × 10 + 4 × 5) by step 59; the thrust cut at step 150 wakes segment 0 with `WAKE_THRUST` and segment 1 with `WAKE_NEIGHBOUR`. The golden 0x8fb3e3c32250148b after 200 steps holds natively, through the boundary, and in WebAssembly.
- **[finding]** The hopper standing on the pad, rigid default: segments 1 to 3 sleep at step 59, the tank's segment 0 at 126, the legs at 307 (reading 0.196). After that the craft is one body, nothing wakes, and the substeps drop from 19 to 4. All-live (`rigid: false`) the engine and pod are not managed and it runs at 17 substeps.
- **[finding]** Switch checks, worst over each run, relative to the craft's momentum at 10 m/s (spin at 20 m) and kinetic energy at 10 m/s:
  - standing: 8 sleeps;
  - hop: 9 sleeps, 9 wakes; momentum 2e-16, spin 2e-17, a wake changes energy by 5e-17;
  - drop at 3 m/s: 8 sleeps; segments 0 to 5 form one body, the legs are still live at 600 steps;
  - drop at 7 m/s: nothing sleeps in 600 steps.
  Every sleep lowers kinetic energy or keeps it (worst +0, the largest fall 6e-10 relative); every wake keeps it to rounding.
- **[finding]** The rigid hop with sleeping on: the tank sleeps at about step 90 and the legs at steps 155 to 174, in powered flight. The throttle step at 150 wakes by neighbour, the cut at 220 by thrust. At touchdown leg 6 wakes on contact at step 266, leg 9 on contact at 267, legs 7 and 8 by neighbour at 267. The first yield is at step 267 with sleeping on and off. The run is not equivalent: 41 breaks and 70 yields, hash 0xc2f0d492f949c2f3, against 40 and 76 off; 24007 substeps against 25080. After the landing the damaged craft reads 0.39 to 0.46 of yield (stress locked in by the plastic set, measured live), so it never sleeps again.
- **[finding]** Ten simulated minutes (36000 steps) standing: 8 sleeps, no wakes, the largest reading 0.196 (98% of the sleep level). The sum of squared node speeds is 3.2e-12 m²/s² in minute 2 and the same in minute 10, so no energy is gained. 3.2 s, 4.13 substeps per step.
- **[finding]** A downward push on the pod of 0.3 × the craft's weight, on 150 steps and off 150, from step 600, for ten minutes: 240 wakes (236 by neighbour, 4 by estimate) and 244 sleeps. The shortest wake-to-sleep is 60 steps, which is `QUIET_STEPS` (above `MIN_LIVE_STEPS`), and no switch adds energy. 40 s, 18.9 substeps per step. At 0.45 × the weight: 123 wakes, the shortest 62 steps. The test runs two minutes by default (48 wakes, 52 sleeps, shortest 60) and ten with `RIBCAGE_SLOW`.
- **[finding]** The open question: a steady resting contact lets the legs sleep without missing the start of a hard landing. It stays in range, so it is no arrival; a foot coming into range, or closing faster than 0.25 m/s, wakes the leg before the solve. Above, the legs woke at step 266 and 267, and the first yield is at 267 either way.

### Not checked

- **[finding]** Equivalence (same breaks, legs lost, tilt) is not checked here: the hop differs by one break and six yields. That is 5.9's test.
- **[finding]** The rigid stance carries its four feet unevenly: four contacts under one body are statically indeterminate, and the solver's split differs from the live craft's. It shows as the settle jump above, and it may move where the legs first yield on a landing.
- **[assumption]** Wake damping (1 + 4 · left / 10) is untested beyond "it runs and the checks pass"; nothing measures whether it helps.
- **[finding]** An asleep body drifts at about 1e-7 m/s on the pad, from the contact's leftover push; it does not grow over ten minutes.
- **[assumption]** Not run on the page.
- **[oracle-candidate]** Should an asleep segment's reading carry the residual λ of its seam beams at sleep (stored and re-added), so a damaged craft can sleep at its locked-in stress without reading falsely high or low on a later load?

### For 5.9

- Tune the four thresholds, the closing speed and the wake damping against the equivalence test, not against this phase's numbers.
- Decide whether the post-landing locked-in readings (0.39 to 0.46) should let a damaged craft sleep (a sleep level over the locked-in value, or the residual-λ idea above).
- The settle window (10 steps) and the 1.25 margin on the estimate are the first knobs if a sleeping craft wakes falsely or late.


## 2026-11-19 (phase 5.9: equivalence and tuning)

Built by Opus. `cargo test` 140 passed, 5 ignored; `pnpm test` 246 passed; `pnpm typecheck` clean. The pre-sleep goldens replay with sleeping off (weak-bridge at ABI 2, tower-collapse at 3, hard-landing at 4), the all-live hop ends at 0xd92f2aad01c0e048 at step 1320, and the 4.8 bands pass with sleeping on. No ABI change (still 9).

### As built

- **[decision]** `src/equivalence.ts` builds 16 scenes and `test/equivalence.test.ts` runs each twice, sleeping on and all-live (40 s). The scenes are the 4.8 drops at 2, 3 and 7 m/s (2 m/s on the pad only, 3 and 7 on the pad and the five rough spots), full throttle, the scripted hop, and a 70 kN side push on the tank for 0.1 s (`KNOCK_FORCE`: 70 kN yields about ten beams and the craft stands, 85 kN breaks some and tips it, 100 kN knocks it over).
- **[decision]** Tolerances, from the scenes' own chaos: moving the all-live 7 m/s rough drop 0.1 to 1 mm sideways already changes up to 2 breaks and 3 yields; the hop is steadier (a 1 to 1000 N push at step 240 changes at most 1 break). So the crash scenes (the hop and 7 m/s on a rough spot) may differ by 2 breaks and 6 extra yields. Every other scene must break and yield the same set exactly. Everywhere: the same legs lost and the tilt at rest within 2°, every switch keeping momentum and spin to 1e-12 with no energy added, and at least one sleep.
- **[decision]** `WAKE_LEVEL` 0.25 → 0.30 and `SLEEP_LEVEL` 0.20 → 0.25. After the knock the legs read about 0.21 and never slept, and the tank's segment 0 kept waking through its neighbour. With the new levels the knocked craft sleeps (standing 11.6 substeps against 19).
- **[decision]** `WAKE_DAMP_BOOST` 4 → 0. The extra damping damped the impact that caused the wake: the hop's legs crushed differently. The code is kept for later tuning.
- **[decision]** New wake rules, all tested before the solve:
  - a contact trigger wakes every segment of its body, not only its tree neighbours;
  - a contact closing faster than 1 m/s (`CRASH_WAKE_SPEED`) wakes the whole craft over its seams, broken or not. A stress wave crosses a few metres of structure in about a millisecond, so the whole craft is loaded within the step;
  - a throttle step over 0.1 (`THROTTLE_CRAFT_WAKE`) wakes the whole craft. The legs had slept under thrust and kept that loaded shape through the engine cut at step 220. This was the hop's main gap: it closed from 8 breaks and 8 extra yields to 1 and 5;
  - a commanded force on a node that changes by more than the node's weight wakes its body (`WAKE_FORCE` = 8, a new reason). At 70 kN the legs had stayed asleep 3 to 4 steps into the knock and yielded 10 against 6. With the force wake, 9 yield against 10 and nothing breaks.
  The commanded force at the last check per node (`Sleep.pushed`) is in the state hash.
- **[decision]** A damaged craft does not sleep. There is no special rule: its yielded beams lock in 0.39 to 0.46 of yield, over `SLEEP_LEVEL`, so it stays live. A sleep ceiling of 0.5 for damaged segments was tried. It made them chatter at rest (sleep, wake, sleep), and each switch changes the island's substep count, so the live legs jolted: a 3 m/s rough-b leg moved at 0.67 m/s at the end and the 4.8 bands failed. Removed.
- **[decision]** Unchanged after trying them in the equivalence test: `QUIET_CHANGE` 0.005, `QUIET_STEPS` 60, `MIN_LIVE_STEPS` 30, `CONTACT_WAKE_SPEED` 0.25 m/s, `SETTLE_STEPS` 10, `ESTIMATE_MARGIN` 1.25. None of them moved a break or a leg.
- **[decision]** Sleeping is on by default in the flight scenarios: `prepareFlight` sends `CMD_SET_SLEEPING` whenever it sends the segment commands, unless `sleeping: false`. The core's own default stays off, so the old recordings replay unchanged.
- **[decision]** `hard-landing-rigid.rcrd` was re-recorded on purpose with sleeping on (106940 bytes, ABI 9). The rigid hop's hash moves from 0x5a1c383b76d89ea6 (5.4 to 5.8, sleeping off) to **0xa08c8113b3a581c7**. The 5.8 sleeping-on value 0xc2f0d492f949c2f3 is gone because the rules changed. The estimate tests keep the sleeping-off rigid hop (0x5a1c383b76d89ea6), since they compare the estimate against the measurement.
- **[decision]** Tests adapted to the default. `test/sleep.test.ts` builds its setups with sleeping off and switches it itself. The 5.8 test "legs asleep in flight wake on contact" is now "wake at the engine cut and are live for the touchdown", since the throttle rule wakes them at step 220 and the coast is too short to sleep again. The first yield is at step 267 either way. The cycle check allows 59 step numbers: a wake before the solve makes its own step live. `test/rigid.test.ts` checks the two spawn bodies with sleeping off, and `test/flight.test.ts` expects the sleeping record last.

### Measured

Off/on, substeps and ms per step:

| Scene | Breaks | Yields (+extra −missed) | Legs lost | Tilt | Notes |
| --- | --- | --- | --- | --- | --- |
| 2 and 3 m/s, pad | 0/0 | 0/0 | none | 0.0° | rest 19/7–7.9 substeps, 2.6/0.4 ms |
| 3 m/s, rough a–e | 0/0 | 1–6, identical sets | none | within 0.3° | rest stays near 19; ms down 0–30% |
| 7 m/s, pad | 24/24 | 56/56 | none | 0.3/0.0° | rest 19/13.4 substeps |
| 7 m/s, rough a–e | 34–41, worst differs by 1 (rough-b) | worst +1 (rough-e) | the same pair each | within 0.4° | rest 17–19 substeps |
| Full throttle | 0/0 | 0/0 | none | 0.6/0.3° | thrust 19/7.5 substeps, 1.76/0.38 ms |
| Hop | 40/41 (differs by 1) | 76/75 (+5 −6) | legs 6 and 9 both runs | 102.5/102.3° | thrust 19/15.5 substeps, 1.98/1.18 ms; coast, touchdown and rest 19/19 |
| Side knock | 0/0 | 10/9 (+0 −1) | none | 1.5/1.6° | standing 19/11.6 substeps, 2.10/0.62 ms; knock and rest 19/19 |

- **[finding]** Every switch in every scene keeps momentum and spin to 1e-12 relative, no sleep adds energy and no wake changes it.
- **[finding]** The saving is in quiet phases: standing, thrust, and a clean pad landing at rest. A rough-ground landing pays nearly full price at rest, because some leg stays loaded over the sleep level.
- **[finding]** `SLEEP_SCENE_200` (the ffi body scene) moved from 0x8fb3e3c32250148b to 0x5b2faa52304df2bb because of the `pushed` hash field and the wider wakes. It is checked natively and in WebAssembly.

### Not checked

- **[assumption]** Only the hopper. The tolerances are stated against its chaos; another craft may need its own.
- **[assumption]** The ms numbers are one run on one machine, so they are noisy (a few phases show sleeping on slower by 0.1 to 0.5 ms with the same substep count).
- **[finding]** The 5.8 contact wake ("asleep legs wake on contact before the first yield") is no longer covered by a hop test, because the legs are awake at touchdown. The rough drops exercise contact wakes (rough-a: 15 wakes), but no test asserts that the wake comes before the first yield there.
- **[assumption]** Not run on the page.
- The all-live rover not resting is 5.6's finding and is untouched.

### For 5.10 and 5.11

- **[finding]** The root of the wreck chatter: the substep count is per island and set by its stiffest live beam, and it changes when segments join or leave a body. A live segment's settled sag depends on that count, so every switch jolts its live neighbours. A per-segment (or per-live-part) count, or holding the count across a switch, would let damaged and partly loaded craft sleep. That matters for 5.11's tall rocket, which will be partly asleep most of the time.
- **[oracle-candidate]** Can the substep count of a mixed rigid/live island be held, or chosen per live part, without breaking determinism or the 4.8 bands? This would let switches stop changing the live segments' settled shape.
- 5.10 splits a body cut by a break. The crash rule (wake the whole craft over seams, broken or not) means a split always happens awake in the hopper scenes, so test a split of a sleeping body directly.

## 2026-11-19 (phase 5.10: splitting into vessels)

Built `core/src/vessel.rs`: after the solve, on a step with a break event or a structural command, a union-find over live nodes finds the connected pieces; more than one piece makes vessels. ABI 9 → 10 (`CMD_BREAK_BEAM` = 18, `EVENT_SPLIT` = 6, `EVENT_VESSEL_SEGMENT` = 7, readers `rc_vessel_*`), mirrored in `src/core.ts`, `src/world.ts` and `src/commands.ts`; the decoder still reads ABI 2 onward.

### Decisions

- **[decision]** Pieces are joined by unbroken beams and live joints only. A body does not join anything by itself, because a rider's beams stay alive, so sleeping can never change what is connected. Segment membership does not join either. The first build did join every node to its segment, and then the hop never split: leg 0 is cut off inside its own segment (the crush bay), not at its seam.
- **[decision]** A segment cut inside belongs to the vessel holding most of its node mass (summed in node order, ties to the lower id), so a leg torn off below its root is reported in the new vessel. The segment is marked torn, and a torn segment never sleeps, since one body over two loose pieces would move them together. A host's explicit `CMD_MAKE_RIGID` on a torn segment is not refused (see Not checked).
- **[decision]** Identity: each piece belongs to the lowest old vessel id among its nodes. Of one vessel's pieces, the one holding its lowest node keeps the id; the others get new ids in piece order. Ids are never reused. Nodes added after a split have no vessel until the next check. A new beam between two vessels merges them at the next check, and the lower id is kept. Commanded breaks and deletes set a pending check; adds do not, so a merge waits for the next cut.
- **[decision]** Nothing runs in a world without segments, and nothing enters the state hash until the first split. So the all-live goldens and the pre-sleep recordings are untouched. The cost: two crafts that never touched share vessel 0 until the first check finds them apart. Craft against craft is milestone 8.
- **[decision]** A body whose segments now lie in more than one vessel is divided with the wake formulas. The segments away from the vessel of the body's lowest segment are released, and each other vessel's share is made rigid again from those exact velocities. Asleep shares settle again (`SETTLE_STEPS`). Each step that divides records one `SwitchCheck` of kind 3.
- **[decision]** Crash-wake and throttle-wake after a split: both stop at a vessel boundary, and so does the neighbour wake. "The whole craft" now means the whole vessel: `sleep_spread_craft` and `sleep_spread` only cross seams whose two segments are in one vessel. A torn-off leg lying on the ground is not woken by the craft's engine.
- **[decision]** The stress estimate is per vessel. In the leaves-to-root pass, a child segment's load is added into its parent only when they are in one vessel.
- **[decision]** A commanded break (`CMD_BREAK_BEAM`) emits no break event. The host knows it asked; the split event still comes.
- **[decision]** Vessel mass is the sum of the vessel's live node masses in node order, not the stored segment masses, because a torn segment's mass has to be divided. With the hopper's fractional node masses, the vessels add to 4600 kg within 1 to 3 ulp (9.1e-13 and 2.7e-12 kg measured), not bit-exactly as the plan asked. Exact sums would need masses stored at the split, which would drift with adds and deletes. The tests hold the sum to 1e-12 relative.
- **[decision]** Goldens re-recorded on purpose:
  - `hard-landing-rigid.rcrd` (106940 bytes, ABI 10): the rigid hop hash moves from 0xa08c8113b3a581c7 to **0x8e49fe2d420d03e2**;
  - the estimate tests' rigid hop with sleeping off: 0x5a1c383b76d89ea6 → **0xc5612d988a0ccf1f**. Here only the vessel ids enter the state hash: the physics hash matches the last commit's build at every step checked through 1320.
  With sleeping on, the physics matches until step 640, where the torn-off leg piece (segment 6) used to fall asleep and now stays live. By the end the median node is 9 mm from the old run; the largest moves are loose leg nodes, up to 0.89 m. Unchanged: the all-live hop 0xd92f2aad01c0e048, `SLEEP_SCENE_200` 0x5b2faa52304df2bb, the pre-sleep recordings, and `test/equivalence.test.ts`, which passes as it stands.

### Measured

| Case | Result |
| --- | --- |
| Sleep scene cut live (native) | vessels 80 + 20 kg, exactly 100; cube keeps vessel 0 |
| Sleep scene cut asleep (native) | body divided in two; \|dp\| 1.1e-13 N·s, \|dL\| 3.2e-13 N·m·s, dE +3.6e-12 J |
| Three rigid cubes, cut between 2 and 3 | 160 + 80 kg; lower two still one body, top cube its own; momentum change over the step is gravity to 1e-9 |
| Hopper leg 0 cut in the air, live | 4478.563 + 121.437 kg (sum +9.1e-13); momentum off gravity's impulse by 1.9e-9 N·s |
| The same cut asleep (step 100 of a 20 m/s drop) | \|dp\| 3.3e-24 N·s, \|dL\| 9.6e-13 N·m·s, dE 0 J; leg on body 6, craft on body 0 |
| Rigid hop, sleeping on and off | split at step 275, leg 0 is vessel 1 (88.8 kg torn piece, the craft 4511.2 kg); matches the beam walk (legs joined 0111) |
| 7 m/s drops, rigid | rough-a, rough-b (leg 1), rough-d (leg 0) split at step 55 to 56; the pad, rough-c and rough-e keep every leg |
| Cut scene golden | `VESSEL_SCENE_200` 0x2235baa7f88d26ad, native and wasm |

Suites: `cargo test` 144 passed, `pnpm test` 251 passed, `pnpm typecheck` clean.

### Not checked

- **[assumption]** The all-live flight (`rigid: false`) sends no segment commands, so it never checks for a split and its hop reports one vessel. The core needs segments to split; a craft without them is one vessel by construction.
- **[assumption]** A host that makes a torn segment rigid by command gets one body over two pieces. Nothing in the repo does this.
- Not run on the page. No demo (skip by plan).
- A split into three or more vessels in one step is covered by the code path (piece order, new ids in sequence) but no test cuts three ways at once.

### For 5.11 to 5.13

- 5.11 (the tall rocket): stages separating are a split. A decoupler should be a beam set that `CMD_BREAK_BEAM` cuts; the upper stage keeps vessel 0 only if it holds the lowest node, so number the craft from the payload down if the payload should keep the identity.
- 5.12 (the page): read `rc_vessel_nodes` and colour or label by vessel; the split event gives the step and the segments for a "stage separated" or "leg lost" notice.
- 5.13: the torn-segment rule keeps a broken leg live forever. With many wrecks lying about that costs substeps; a later phase could re-segment a torn segment into its pieces so each can sleep.
- **[oracle-candidate]** Is "majority of mass" the right owner for a segment cut inside, or should a cut segment be split into new segments at the cut, so that each piece can sleep and carry its own estimate?

## 2026-11-19 (phase 5.11: the tall rocket)

Built `test/fixtures/tall-rocket.craft.json` (engine, five tanks, pod, from the existing generators only), `src/tall-rocket.ts` (scenario and measurements) and `test/tall-rocket.test.ts`. No change to `core/`. The rocket is 147 nodes, 808 beams, 17 segments, 7000 kg exactly (the sum of its parts), 15 m tall, with 160 kN of thrust against a 69 kN weight. Budgets: 13 substeps (cap 24), node-mass ratio 3.06 (cap 10). The clamp is the engine's lowest ring pinned with `setInvMass(n, 0)`; no helper was needed. Scenario: stand 10 s, full thrust 10 s, a 30 kN side knock for 0.2 s on the middle tank's middle ring, engine cut at step 1500, end at step 2100.

### Decisions

- **Knock 30 kN.** A sweep showed 30 kN is the largest round figure that yields no beam in either run; 40 kN yields one or two. The rocket must not break for the two runs to be comparable.
- **Radius 1.25 m, wall area 0.0008 m², member area 0.0006 m².** The first design (0.75 m radius, 3 m tanks) was unstable: with no knock and no thrust the tip drift grew from rounding noise and the engine beams yielded by step ~1950, all-live and sleeping alike. A stiffer wall collapsed faster. Radius 1.5 m was stable, 1.0 m not, so the limit is slenderness (a very slender stack buckles), not sleeping. 15 m by 2.5 m is as tall as this stays stable.
- **Tolerances.** Standing and thrust: sleeping must be no worse than all-live on tip excursion, tip range and stack-length range, and use fewer substeps. Knock and settle: sleeping within 1.5 times all-live (measured 1.02 to 1.08). Switch checks 1e-12. The same run twice gives the same hash.

### Measured (all-live / sleeping on, per phase)

| phase | substeps per step | ms per step | stack length range | tip max | tip range | bodies |
|---|---|---|---|---|---|---|
| standing | 11.0 / 6.0 | 0.96 / 0.29 | 0.08 / 0.00 mm | 2.33 / 1.67 mm | 2.08 / 0.48 mm | 2 / 1-2 |
| thrust | 11.0 / 4.7 | 0.96 / 0.20 | 0.02 / 0.00 mm | 6.15 / 1.16 mm | 4.26 / 0.32 mm | 2 / 1-2 |
| knock | 11.0 / 11.0 | 0.96 / 0.98 | 4.12 / 4.50 mm | 289 / 296 mm | 189 / 196 mm | 2 / 2 |
| settle | 11.0 / 11.0 | 0.97 / 0.96 | 46 / 52 mm | 871 / 899 mm | 544 / 586 mm | 2 / 2 |

Sleeping on: the whole stack is one body from step 180; 31 wakes, 32 sleeps; every switch kept momentum to 1.5e-17 and spin to 1e-17, with no energy added (wake energy 1e-23, relative). No beam yielded or broke in either run, and the rocket stays one vessel.

### Findings

- **Sleeping is much better standing and under thrust:** wobble sideways four to thirteen times smaller, the stack length steady, and half or less the substeps and time.
- **A knock wakes the whole craft and it does not sleep again.** After the knock the rocket sways 0.2 to 0.9 m (low damping), the whole of it awake (two bodies, the pod and engine, as all-live; nothing asleep) through the 10 s of settle. A 7200-step run (120 s) did not return to sleep either, and by then both runs had yielded the same 19 to 42 beams from the sway. So the plan's "settle back to sleep" is not met: the wake is by the throttle step (over 0.1) and the force command, both of which wake the whole craft, and the sway is too big to sleep after. Owner decision, not fixed here.
- **Under the knock and the settle sleeping is slightly worse than all-live** (tip 296 against 289 mm, 899 against 871 mm; about 2 to 8%), as it is on the hop. The cause is the per-island substep rule: the substep count changes when segments join or leave a body, and the jolt shows as a small difference rather than a gain. Not changed; `core/src/sleep.rs` thresholds untouched.
- **Slenderness limit** as above. A tall thin rocket needs a thicker wall or fewer tanks, or the beams buckle.

### Not checked

- Not run on the page; no demo (5.12). Not run with the rocket free (no clamp) or lifting off. Not checked with other counts of tanks or ring nodes. No run beyond 120 s apart from the one 7200-step probe, which was not committed. The goldens (all-live hop `0xd92f2aad01c0e048`, rigid hop `0x8e49fe2d420d03e2`, weak-bridge, tower-collapse, hard-landing) replay in their existing tests, which pass; this phase did not re-assert them.

### For 5.12 and 5.13

- 5.12: `runTallRocket` gives the series (`bodies` and `asleep` every 30 steps) for a panel; the knock is `tallRocketCommands`. Show the all-live and sleeping tips side by side; the knock is the interesting scene, the sway is large.
- 5.13: if the rocket should settle to sleep after a knock, the wake rule needs to wake only the knocked segments, and a quieter threshold after a large sway.
- **[oracle-candidate]** Should a throttle step or a commanded force wake the whole craft, or only the segments it acts on? Waking all prevents any re-sleep here.
