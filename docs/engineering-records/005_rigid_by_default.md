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
