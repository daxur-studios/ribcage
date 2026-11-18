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
