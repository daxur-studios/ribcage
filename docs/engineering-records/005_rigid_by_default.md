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
