---
type: Plan
title: Aerodynamics
description: Milestone 6 - parts declare aero elements (wing strips, body stations, plates) tied to nodes with an automatic body station for anything undeclared, lift and drag per element in the core through an environment-force channel, a craft in flight that never sleeps, a simple plane that flies and is steered, and a wing that flexes and snaps in a hard turn.
timestamp: 2026-11-19T12:00:00.000Z
status: active
record: /docs/engineering-records/006_aerodynamics.md
tags: [milestone-6, aerodynamics, wings, flutter, sleeping]
---

# Plan 006: Aerodynamics

Evidence: [ER 006](/docs/engineering-records/006_aerodynamics.md). Design: the [spec](/docs/research/node-beam-engine-design-spec.md) (skin triangles, the wing part, milestone 6), the [core layout rules](/docs/research/core-data-layout.md) (binding: rules 3, 5, 7, 8 and 9 above all), and what milestone 5 left ([ER 005](/docs/engineering-records/005_rigid_by_default.md), closing section "What milestone 6 should know").

**What this milestone adds.** Air. Each part generator declares aero elements with node weights, the way it already emits segments and seams: **wing strips** (lift with stall, induced and profile drag, a control deflection), **body stations** (axial drag, slender-body lift, crossflow drag, base pressure) and **plates** (airbrakes, flaps). A segment that declares nothing gets an automatic body station from its bounding shape, so legs, struts and wreckage always get drag. The core evaluates every element once per step from its nodes' positions and velocities and pushes the result back onto the same nodes, so a wing that twists changes its own angle of attack and a wing that flexes carries its own load. A craft in flight never sleeps. A small plane is assembled from parts, flies level, is steered by its control surfaces, and loses a wing when pulled too hard at speed.

**Scope [assumption]:** still a test bed. One new part (the wing), one reference plane, the hopper and the tall rocket from milestone 5, the craft page. Air start for the plane (spawned at cruise speed); no take-off run is required. No autopilot, no propellers (thrust is the existing engine), no fuel.

## Settled before building

Decided and logged in [ER 006](/docs/engineering-records/006_aerodynamics.md) (first dated section). A builder follows them; changing one is a logged decision with a reason.

- **Parts declare elements; the core never learns what a part is (option E).** An aero element is a kind, a list of nodes with weights, a few frame nodes and a row of coefficients. The generator knows the shape, so it knows the strips and stations; the assembler sends them as commands, as it sends segments today. **Per-triangle drag is rejected as the main model**: it gets induced drag, stall and slender-body lift wrong without a panel method on top, and its cost grows with skin detail. Skin triangles stay for buoyancy (milestone 7), heating and hypersonic pressure (later).
- **Automatic fallback.** At compile, every segment with no declared element gets one body station from its nodes' bounding shape: axis along the largest extent, frontal area from the other two extents (ellipse), plan area from length times the larger width, bluff coefficients, no slender-body lift, equal weights over the segment's nodes (the force acts at the geometric centre) **[assumption]**. Done in the assembler (TypeScript), never in the core, so the core sees only declared elements.
- **Storage (layout rule 3).** Per element: `aero_kind`, `aero_node_start`, `aero_node_count` into flat `aero_node` / `aero_weight` lists (weights sum to one; no fixed cap), frame nodes `aero_frame[4]` (strip: trailing-edge, leading-edge, span-root, span-tip; station: tail, nose; plate: two edges giving the normal), coefficient arrays one per field, `aero_deflection`. Elements are evaluated in ascending index order. Stable indices; a dead node makes its element inert, never moves another into its slot.
- **When and where.** `core/src/aero.rs`, `apply_aero(world)`, called once per 60 Hz step in `step_inner` (`core/src/sim.rs`) right after `apply_thrust` and **before** `estimate::take_forces`, from the positions and velocities the step starts with. Per-step forces, like thrust: no substep or frame-time dependence. Relative velocity is the weighted node velocity minus the wind. Verified in the code (ER 006): `take_forces` copies `world.force` after thrust, so air loads enter the stress estimate's cut and reach every seam; forces on rider nodes are summed into their body's force and torque in `core/src/island.rs` (the rider branch of island packing), so air acts on sleeping and never-live bodies with no extra code.
- **The environment-force channel.** Rule 7 stays one door: `World::add_env_force(node, f)` adds to the same `force` array the solver reads **and** to a per-node `env_force` array cleared with it. The force-jump wake in `core/src/sleep.rs` (`sleep_before_solve`, the "applied forces" loop comparing `force` with `sleep.pushed`) compares host forces only, `force - env_force`. Today that loop already runs before thrust, so a core-side air pass would not trip it by order alone; the channel makes the rule a named contract with a test, and a later environment source (water, a host gust) uses the same door. Air loads wake segments only through the estimate reading, never through the jump trigger.
- **Never sleep in flight.** Two parts. (1) A new segment flag `SEG_NEVER_SLEEP` (value 2 in `CMD_ADD_SEGMENT`'s flags, in `core/src/segment.rs`; only `SEG_NEVER_LIVE` exists today); the wing generator sets it on every wing segment, as ER 005 advised, until 6.11 has measured otherwise. (2) A vessel **in flight** (no node of it in contact range this step, and the largest dynamic pressure on any of its elements above `FLIGHT_Q` = 50 Pa **[assumption]**, about 9 m/s at sea level) wakes once through the existing wake pass (momentum kept) and none of its sleepable segments sleeps while in flight. Never-live segments stay rigid. A craft on the pad or on a launch clamp still sleeps, so milestone 5's standing saving stays; its in-ascent saving is given up, which the roadmap check asks for. **Thresholds in `sleep.rs` are not retuned in this milestone.**
- **Air.** World settings by `CMD_SET_AIR`: sea-level density (1.225 kg/m³ default), scale height (8500 m; 0 means constant), wind vector (zero), speed of sound (340 m/s). Density `ρ0 · exp(-y / H)` with the core's **own** `exp` added to `core/src/maths.rs` (range reduction and a polynomial, `+ - * /` only, covered by the maths scan test). Gravity stays flat. Air settings and elements enter the state hash only once one exists, so worlds without air hash as before.
- **Formulas** (ported from a consuming game's analytic model, as a reference only, no code copied; coefficients are **[assumption]** until the 6.9 comparison). With `u = -(v_rel)` the air velocity seen by the element, `q = ½ ρ |u|²`, `M = |u| / a`, `β = sqrt(1 - min(M, 0.7)²)`:
  - *Wing strip.* Chord axis `ĉ` (trailing to leading edge), span axis `ŝ`, normal `n̂ = ĉ × ŝ` (sign fixed by declaration order). Simple sweep: lift uses the chordwise part `u_p = u - (u·ŝ) ŝ` and `q_p` from it. `α = atan2(u_p·n̂, -u_p·ĉ)` (own `atan2`), effective `α_e = α + τ · δ` (`δ` the deflection, `τ` flap effectiveness, 0 for a plain strip). Lift slope (DATCOM form, Prandtl-Glauert inside it) `a = 2π A / (2 + sqrt(4 + A² β²))` with `A` the whole wing's aspect ratio. Attached `C_L = a · α_e`; flat plate `C_L,fp = 2 sin α_e cos α_e`, `C_D,fp = 2 sin² α_e`; stall blend `w = smoothstep((|α_e| - 14°) / (24° - 14°))`, `C_L = (1 - w) a α_e + w C_L,fp`; `C_D = C_D0 + (1 - w) C_L² / (π e A) + w C_D,fp`, `e = 0.8`. Lift `q_p S C_L` perpendicular to `u_p` in the chord-normal plane; drag `q_p S C_D` along `u_p`, plus spanwise friction `q_s S C_f` along the spanwise part. No pitching moment for symmetric sections (strip at quarter chord).
  - *Body station.* Axis `x̂` (tail to nose); `α` the angle between `u` and `-x̂` (own `atan2` of the cross and axial parts). Axial drag `q A_f C_A` along `-x̂` (sign of the axial flow); normal force `q A_ref C_N` along the crossflow, `C_N = (A_b / A_ref) sin 2α cos(α/2) + η C_dc (A_p / A_ref) sin² α` (slender-body term on base area `A_b`, crossflow term on plan area `A_p`; `η = 0.7`, `C_dc = 1.2`); base pressure `q A_b C_base` (`C_base = 0.12`) when the station's base is declared and no thruster on it is firing.
  - *Plate.* Normal `n̂`; `α` between `u` and the plate; normal force `q A C_N n̂` toward downstream with `C_N = 1.2 sin α (1 + 0.67 cos α)` (slope 2 at zero, 1.2 broadside), plus friction along the plate; a deploy fraction scales `A`.
  - *Mach stub.* `β` is held at its `M = 0.7` value above 0.7; above `M = 0.85` a world counter `aero_mach_clamped` counts elements per step (like `clamped_nodes`), no event. Wave drag is a later pass.
- **Control surfaces are deflections, not moving lattices [assumption].** `CMD_SET_AERO_DEFLECTION` (element, radians) sets `δ` on a strip or the deploy fraction on a plate, applied from the next step. A servo-driven hinged flap is possible with milestone 5's joints but is not needed to steer; it is a later option.
- **Explicit air, bounded.** Air is a per-step explicit force, so a light node under a big area can over-correct. Each element gets an **aero rate** at a design speed, `r = ρ0 V (∂C/∂α or C) S Δt / m_e` with `m_e` the weighted node mass; the part budget refuses a part above `r = 0.5` at 150 m/s **[assumption]**. 6.8 measures the real bound and whether the one-step lag pumps flutter.
- **Agreement with the consuming game.** The game keeps its own analytic model for autopilot and path prediction. For a stiff craft (every segment rigid) ribcage's summed lift, drag and trim must agree with it. The owner supplies the reference numbers for the 6.9 reference craft as a fixture (numbers only); until then the test compares against `src/aero-reference.ts`, the same formulas summed on the undeformed geometry, and the owner check stays open in the record.
- **Goldens stay valid.** Recordings replay their own command bytes; the committed ones (`weak-bridge.rcrd`, `tower-collapse.rcrd`, `hard-landing.rcrd`, `hard-landing-rigid.rcrd`) send no air command, so they replay unchanged at every phase. `compileCraft` gets an `aero` option, default on in flight scenarios from 6.3; then `hard-landing-rigid.rcrd` is re-recorded deliberately and logged with its new hash, and `aero: false` keeps the 5.10 hop hash `0x8e49fe2d420d03e2` (and `rigid: false` the all-live `0xd92f2aad01c0e048`) as regressions.
- **Commands and ABI.** Each phase that adds command kinds bumps `ABI_VERSION` (11 in 6.1, onward; it is 10 today in `core/src/ffi.rs` and `src/core.ts`), mirrors them in `src/commands.ts`, and keeps the recording decoder accepting every earlier ABI.

## Phases

Each phase ends with passing tests (`cargo test`, `pnpm test`, `pnpm typecheck`), the goldens replaying to their hashes, ticks the matching roadmap check if it proves one, and logs a dated section in the record. "Build with" is the model for the session; mechanical follow-ups go to Haiku. 6.1 to 6.11 are headless; the page comes in 6.12.

Order: 6.1 → 6.2 → 6.3 and 6.4 (side by side) → 6.5 → 6.6 → 6.7 and 6.8 (side by side) → 6.9 → 6.10 → 6.11 → 6.12 → 6.13.

Roadmap checks (a quarter of the milestone each): "parts declare aero elements ... with automatic fallback" in 6.5 (with 6.3); "lift and drag act per element ... a simple plane flies and can be steered" in 6.10; "air forces do not wake sleeping bodies ... a craft in flight never sleeps" in 6.7; "a wing flexes under load and snaps in a hard turn" in 6.11. ER 004's note that rough-landing fragments creep because the core has no drag is closed in 6.3.

- [x] **6.1 Air settings, the environment channel and the never-sleep flag.** Build with: Sonnet (plumbing on the 5.1 pattern).
  `CMD_SET_AIR`; own `exp` in `core/src/maths.rs`; `World::add_env_force` and `env_force` (cleared with `force`); the force-jump loop in `sleep_before_solve` reads `force - env_force` (no threshold change); `SEG_NEVER_SLEEP` added to `SEG_FLAGS_KNOWN` and honoured by the sleep candidate test in `sleep.rs`. ABI 11, `src/commands.ts`, decoder. Adds the plan link to the roadmap's milestone 6 heading (mechanical).
  **Test:** own `exp` within 1e-14 relative of a reference table over the range used, and the maths scan test passes; a sleeping hopper on the pad with a steady `add_env_force` load of twice a node's weight on every node is not woken by the jump trigger, while the same load through `CMD_ADD_FORCE` is; a segment flagged never-sleep stays live through a 60 s rest that sleeps the rest of the craft; density at 0 and 8500 m equals the formula; the goldens replay; a world with no air command hashes as before.
  Done when: the channel and flag exist, are tested, and nothing else moved.
  Demo: skip.

- [x] **6.2 Body stations in the core.** Build with: Sonnet (known formulas, the element arrays).
  `core/src/aero.rs`: the element arrays, `CMD_ADD_AERO_ELEMENT` (kind, node list with weights, frame nodes, coefficients; bad records rejected with the rest of the batch applying), `apply_aero` at the place in "Settled", the station formulas as a pure function, forces through `add_env_force` by weight; elements and air in the hash once one exists. Check that rider nodes' world velocities at step start are the body's `V + ω × r`; if not, the station reads the body's velocity at the weighted point, and the record says which. ABI 12.
  **Test:** the station function against hand values (axial only, 90° crossflow, 10° and 30° with slender-body lift, base pressure on and off); a single-node station falls to terminal speed `sqrt(2 m g / (ρ A_f C_A))` within 0.5%; a station on a rigid body and the same one live give the same force; wind on a resting station gives the formula force; with the estimate on, a tethered station's air load appears in the seam force within 1%; ascending order and the same run twice give the same hash, natively and in WebAssembly; the goldens replay.
  Done when: one element kind is complete end to end.
  Demo: skip.

- [ ] **6.3 Automatic stations and air on the existing parts.** Build with: Sonnet (assembler work and measurements).
  `Part` gains `aero: AeroElement[]`; tank, pod, engine and truss declare stations (pod nose, engine base with base pressure), leg and wheel take the fallback; `compileCraft` emits declared elements and one fallback station per segment that declares none; `aero` option on by default in flight scenarios; `hard-landing-rigid.rcrd` re-recorded deliberately and logged. Closes ER 004's creep note.
  **Test:** every segment of the hopper, the rover and the tall rocket has at least one element; fallback areas match the bounding shapes; the hopper dropped from height reaches the terminal speed of its summed stations within 2%; 4.8's bands and 5.9's equivalence test pass with air on; the 4.8 rest window is re-measured with air on and the record states whether air settles the fragments and the creep (expected: barely, since drag at 0.1 to 0.3 m/s is tiny; if so the note is closed as "not air's job", with the numbers, and joint or hinge damping is named as the fix for a later phase); `aero: false` keeps `0x8e49fe2d420d03e2`; the goldens replay.
  Done when: nothing on a craft is without drag, and ER 004's note is closed with numbers.
  Demo: skip.

- [ ] **6.4 Wing strips in the core.** Build with: Sonnet (the strip formulas from "Settled").
  The strip kind: frame from its four nodes, simple sweep, `α` with own `atan2`, DATCOM slope with Prandtl-Glauert, stall blend, induced and profile drag, spanwise friction, deflection with `τ`, the Mach stub and its counter.
  **Test:** the strip function against a table (`α` from -30° to 90°, `M` 0, 0.5, 0.7, 0.9): `C_L` linear below 14°, peak between 14° and 24°, symmetric in `α`, flat-plate values past 30°; induced drag equals `C_L² / (π e A)` below stall; sweep halves nothing it should not (a strip yawed 30° gets lift from `cos² 30°` of `q`); a rigid flat strip in a stream gives lift and drag equal to the formula within 1e-9 relative; twisting the strip's leading-edge node changes `α` by the twist angle; `aero_mach_clamped` counts above 0.85; deterministic hash; the goldens replay.
  Done when: a strip on its own nodes gives the formula force and follows its own twist.
  Demo: skip.

- [ ] **6.5 Plates and control deflection.** Build with: Sonnet.
  The plate kind and `CMD_SET_AERO_DEFLECTION` (deflection on strips, deploy fraction on plates). Ticks the roadmap check "parts declare aero elements (wing strips, body stations, plates) tied to nodes; anything undeclared gets an automatic body station" (with 6.3).
  **Test:** plate `C_N` against its formula at 0°, 10°, 45° and 90°; an airbrake plate on the hopper raises its terminal-speed drag by the stated amount; a strip's deflection shifts `α_e` by `τ δ` and its lift by the slope times that; a deflection command mid-run acts from the next step and replays to the same hash; the goldens replay.
  Done when: all three kinds exist and can be commanded.
  Demo: skip.

- [ ] **6.6 The wing part.** Build with: Sonnet (a generator on the existing pattern).
  `src/parts/wing.ts` (spars, ribs and skin, tapered: span, chord, taper, thickness, material, from the spec's part table): a segment per rib bay flagged never-sleep, seams at the root and between bays with area and bending, one strip per bay with its weights at the quarter chord and its frame on the bay's corner nodes, optional control strips (aileron, elevator, rudder) with `τ`, a root mount ring; the aero rate from "Settled" added to the part budget.
  **Test:** mass exact, inside the substep and node-mass budgets (substep count reported; it must not exceed the hopper's 19 **[assumption]**), aero rate under 0.5 at 150 m/s; clamped at the root, tip deflection under a uniform load within a stated band of the cantilever formula on its spars (the method of `core/tests/cantilever.rs`); in a stream at 5° on a rigid root, the summed strip lift equals the reference sum within 1%; same build twice is byte-identical; the goldens replay.
  Done when: a wing can be generated, clamped and loaded.
  Demo: skip.

- [ ] **6.7 Air does not wake; flight does not sleep.** Build with: **Opus** (touches the wake pass and the equivalence test).
  The in-flight rule from "Settled" in `sleep.rs` (detection from the existing contact-range record and the elements' dynamic pressure, one wake on entry, no sleep while in flight); wakes and sleeps per second logged per scenario. Ticks the roadmap check "air forces do not wake sleeping bodies by themselves, and a craft in flight never sleeps".
  **Test:** the hopper asleep on the pad in a 20 m/s wind stays asleep unless its estimate passes the wake level (the jump trigger never fires on air); the hopper launched at 30 m/s wakes on leaving the pad and no segment sleeps until it is back in contact; the tall rocket on its clamp still sleeps into one body and its 5.11 standing numbers hold; momentum across every switch to 1e-12; wakes and sleeps per second on a gliding craft are zero while in flight; 5.9's equivalence test passes with air on; the goldens replay.
  Done when: no sleeping state can be reached in flight, and nothing on the ground lost its sleep.
  Demo: skip.

- [ ] **6.8 Stability and flutter of explicit air.** Build with: **Opus** (whether the one-step lag pumps energy is not known).
  A clamped wing in a stream at increasing speed: pluck the tip, measure the decay; with air off, the decay of beam damping alone. Find the speed where the measured damping changes sign, compare to a simple bending-torsion estimate, and decide whether the onset is physical or numerical (it moves with the step rate or the substep count). If numerical below 1.5 times the plane's dive speed, fix it (candidates, in order: frame from predicted positions, splitting the strip's velocity-dependent part into a per-node damping term the solver applies each substep through the one door, a lower aero-rate budget) and log the choice and its cost. Also the aero rate bound measured on light nodes.
  **Test:** below the stated onset a plucked wing's energy never rises over 60 s; the onset changes by under a stated fraction between 8 and 32 forced substeps; a fragment of the lightest node mass in the budget at 300 m/s falls without growth or freeze; the same run twice gives the same hash; the goldens replay.
  Done when: there is a measured, stated speed range where air adds no energy, and the budget matches it.
  Demo: skip (a clip of the plucked wing is made for the record, not committed).

- [ ] **6.9 The reference plane and the comparison with the game model.** Build with: Sonnet (an assembly and a comparison).
  `test/fixtures/plane.craft.json`: pod nose, fuselage (truss or tanks), engine at the tail pointing aft, a wing with ailerons, a tailplane with elevator, a fin with rudder; about 2 t, 10 m span, cruise near 70 m/s at sea level **[assumption]**, inside the budgets. `src/aero-reference.ts`: the same formulas summed on the undeformed geometry as one rigid craft (lift, drag, pitching moment, trim angle and elevator at a speed). Optional fixture of owner-supplied numbers from the consuming game's model.
  **Test:** with every segment rigid, the core's summed lift and drag at `α` from -4° to 12° at 50, 70 and 100 m/s agree with the reference within 2%; pitching moment slope is negative (statically stable) and the trim angle and elevator at 70 m/s agree within 0.5°; with the owner's fixture present, the same tolerances against it, otherwise the test says it was skipped; the goldens replay.
  Done when: ribcage and the analytic reference agree on a stiff craft, and the record says whether the game's own numbers were checked.
  Demo: skip.

- [ ] **6.10 The plane flies and is steered.** Build with: **Opus** (dynamic stability: short period, phugoid, Dutch roll, live wings).
  `src/plane-flight.ts`: air start at trim, a control script (elevator, aileron, rudder, throttle) recorded as commands; flight with the plane live (wings never-sleep). Ticks the roadmap check "lift and drag act per element ... and a simple plane flies and can be steered".
  **Test:** trimmed, it holds altitude within a stated band and heading within 2° for 60 s; an elevator step climbs and recovers; an aileron pulse rolls it and it turns; a rudder pulse yaws it and the Dutch roll decays; no freeze, no clamped node, no yield in level flight; the same script gives the same hash natively and in WebAssembly; substeps and ms per step reported; the goldens replay.
  Done when: the plane flies a scripted circuit headless.
  Demo: skip (shown in 6.12).

- [ ] **6.11 The wing flexes and snaps.** Build with: **Opus** (the strength must sit between a normal turn and a hard one; feel-critical).
  Wing tip deflection measured at 1 g and in steady turns; a hard pull at high speed raises the load factor until the root seam yields and breaks and the wing becomes its own vessel; the same pull at low speed stalls without breaking. Wing material or thickness tuned on the reference plane only. Decide from measurement whether wings keep `SEG_NEVER_SLEEP` (ER 005 asked for this measurement); no change to the sleep thresholds.
  **Test:** tip deflection grows with load factor (stated values at 1, 2 and 4 g); a 4 g turn yields nothing; a pull above a stated load factor at a stated speed breaks the root and splits a vessel, with masses adding to the plane's within 3 ulp; the slow pull stalls (lift falls past 14° to 24°) and breaks nothing; the same script gives the same hash; the goldens replay. Ticks the roadmap check "a wing flexes under load and snaps in a hard turn".
  Done when: the snap and the survivable turn are both tests.
  Demo: skip (shown in 6.12).

- [ ] **6.12 Air on the page.** Build with: Sonnet.
  Craft page: the plane selectable, keyboard controls recorded as deflection and throttle commands, lift and drag arrows per element (a view mode), airspeed, angle of attack, dynamic pressure, load factor and in-flight state in the status line, the split wing shown as a vessel.
  **Test:** in Node, the arrow data from the core's element forces; in the browser (headless), the scripted circuit and the hard pull end on the same hash as Node, no page errors. By eye: the wing bends in the turn and comes off in the hard pull.
  Demo: the plane flies, turns, and snaps a wing in a hard pull (the slice demo), narrated with Kokoro as AGENTS.md says, kept local.

- [ ] **6.13 Milestone wrap.** Build with: Sonnet.
  Timing of the plane, the hopper and the tall rocket with air on and off (nodes, beams, elements, substeps, ms per step, the air pass's own share); the milestone demo from the [demo plan](/docs/research/demo-and-media-plan.md), narrated, kept local; roadmap percentages; the record closed with what milestone 7 should know (water's buoyancy per triangle and drag reusing the element functions with water's density, the aero rate bound, in-flight sleeping cost) and the later passes below.
  Done when: a fresh session can start milestone 7 from the files.

## Later passes (listed, not planned)

- Mach regimes past the stub: transonic drag rise, supersonic lift slope, wave drag.
- Wing interference and downwash (the tail in the wing's wake), wing-body carry-over.
- Ground effect.
- Per-triangle pressure for heating, re-entry and hypersonic flow; buoyancy per triangle is milestone 7.
- Cambered sections and pitching moment per strip; propellers; servo-driven hinged flaps.

## Not in this milestone

- Retuning the sleep thresholds in `core/src/sleep.rs`, and the open milestone 5 wake items (a knock waking the whole craft, the torn-segment rule).
- Take-off and landing runs, wheels with brakes, autopilot (the consuming game keeps its own).
- Water (milestone 7), heat, fuel and staging.

## Open questions

- [ ] Is the one-step lag of explicit air a numerical flutter source below the dive speed (6.8 decides)?
- [ ] Do wings need to stay never-sleep, or can a lower wake level do (6.11 decides from measurement)?
- [ ] Do the owner's numbers from the consuming game's model match ribcage's reference craft (6.9; needs the owner's fixture)?
- [ ] Does air settle ER 004's creeping fragments, or is that joint damping's job (6.3 decides)?

## Open risks

- **Explicit air on live wings (biggest).** Forces from the step's start act through all substeps; a light, flexible wing at speed can over-correct or flutter numerically. 6.8 measures before 6.10 and 6.11 rely on it; the fallback costs a solver change through the one door.
- **Substeps on live wings.** Wings never sleep, so their stiffness and node masses set the plane's count; a wing stiff enough not to flap may raise the floor past the hopper's 19. 6.6 reports it; a heavier, softer wing is the dial.
- **Formula drift from the game's model.** Two analytic models diverge quietly; 6.9 is the guard and depends on the owner supplying numbers.
- **The snap window.** The gap between "survives 4 g" and "snaps in a hard pull" may be narrow on a lattice whose yield is force-based and break strain-based; 6.11 may need a seam design, not just thickness.
- **Losing the ascent saving.** A rocket climbing through air never sleeps; milestone 5's thrust saving goes with it. Measured in 6.13; a later decision.
