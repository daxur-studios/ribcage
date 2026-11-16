//! Phase 1.5 and its follow-up: yield is a force cap inside the substeps, break is checked once
//! per step on total strain.
//!
//! The law under test. A beam of compliance `C` and yield strain `ε_y` on a 1 m rest length
//! carries at most the yield force `F_y = ε_y / C`, in tension and in compression. The solver
//! clamps each substep's correction to `F_y · h²`, so a beam at its cap pulls on its mass with
//! exactly `F_y` in every substep and the mass accelerates at `(F − F_y) / m` whatever the
//! substep count. When the cap fires `rest` follows the length (`length ∓ ε_y`), so the beam
//! keeps the same elastic stiffness and the same yield force around its new rest length.
//!
//! Expected values come from that law (and a small continuous-time reference integrator for the
//! one case that has no closed form), never from measured solver output. Every test mass is
//! 1 kg and every beam 1 m long. Tests that need a clean load cancel gravity with an upward
//! force on the mass, so the net load is the one the test states.

use beam_core::event::{Event, EVENT_BREAK, EVENT_YIELD};
use beam_core::real::{Real, Vec3};
use beam_core::sim::{state_hash, step, step_with_substeps};
use beam_core::solver::MIN_SUBSTEPS;
use beam_core::world::{BeamSpec, World, BEAM_ALIVE, BEAM_BROKEN, BEAM_YIELDED};

const G: Real = 9.81;
const MASS: Real = 1.0;
const OMEGA: Real = 60.0;
/// Compliance that gives the 1 kg mass the natural frequency `OMEGA`.
const C: Real = 1.0 / (MASS * OMEGA * OMEGA);
const YIELD: Real = 0.01;
const BREAK: Real = 0.1;
/// Yield force of the standard beam, in newtons: `YIELD / C`.
const FY: Real = YIELD / C;
const NEVER: Real = 1e9;
/// Substeps for tests that compare against the continuous law. The solver is first order in the
/// substep `h` (semi-implicit Euler, like free fall), so a flow measured against the continuous
/// law is off by about `v h`; at 64 substeps that is under 1% in these scenes, and a test
/// below checks that it halves when the substeps double.
const SUB: u32 = 64;

/// A 1 m beam from a pinned node to a 1 kg mass. `side` is -1 for a mass hanging below (the beam
/// is pulled in tension by a downward load) and +1 for a strut with the mass above it.
fn beam_world(side: Real, yield_strain: Real, break_strain: Real, damping: Real) -> World {
    let mut w = World::new();
    w.add_node(Vec3::new(0.0, 0.0, 0.0), 0.0, 0.1);
    w.add_node(Vec3::new(0.0, side, 0.0), 1.0 / MASS, 0.1);
    w.add_beam(0, 1, BeamSpec { rest: 1.0, compliance: C, damping, yield_strain, break_strain });
    w
}

/// The standard beam hanging below its pinned end, critically damped.
fn hanging(yield_strain: Real, break_strain: Real) -> World {
    beam_world(-1.0, yield_strain, break_strain, 2.0 * MASS * OMEGA)
}

/// Beam length.
fn length(w: &World) -> Real {
    (w.pos[1] - w.pos[0]).length()
}

/// Applies a net load of `f` newtons along the beam (positive pulls the mass away from the
/// pinned end) on top of gravity cancelled out, for the coming step.
fn load(w: &mut World, side: Real, f: Real) {
    load_on(w, 1, side, f);
}

fn load_on(w: &mut World, node: u32, side: Real, f: Real) {
    w.add_force(node, Vec3::new(0.0, side * f + MASS * G, 0.0));
}

/// Downward force that gives a settled elastic stretch of `strain` (times the 1 m rest length).
fn load_for(strain: Real) -> Real {
    strain / C - MASS * G
}

/// Runs `steps` steps, pulling down with `force` for the first `loaded` of them.
fn pull(w: &mut World, force: Real, loaded: u32, steps: u32) -> Vec<Event> {
    let mut events = Vec::new();
    for s in 0..steps {
        if s < loaded {
            w.add_force(1, Vec3::new(0.0, -force, 0.0));
        }
        step(w);
        events.extend(w.events.iter().copied());
    }
    events
}

/// Continuous-time reference for one mass on an elastic, perfectly plastic beam (stiffness
/// `1 / C`, yield force `FY`), under a net load `ext(t)` along the beam, started at rest at zero
/// stretch. Returns `(stretch, plastic set, speed)` at `t_end`. Symplectic Euler at 10 µs.
fn plastic_reference(ext: impl Fn(Real) -> Real, t_end: Real) -> (Real, Real, Real) {
    let (k, dt) = (1.0 / C, 1e-5);
    let (mut x, mut p, mut v) = (0.0, 0.0, 0.0);
    let mut t = 0.0;
    while t < t_end {
        let mut s = k * (x - p);
        if s > FY {
            s = FY;
            p = x - FY / k;
        } else if s < -FY {
            s = -FY;
            p = x + FY / k;
        }
        v += (ext(t) - s) / MASS * dt;
        x += v * dt;
        t += dt;
    }
    (x, p, v)
}

/// Below yield the beam returns to rest: same rest bits, no flag, no event.
#[test]
fn loaded_below_yield_returns_to_rest() {
    let mut w = hanging(YIELD, BREAK);
    let events = pull(&mut w, load_for(0.005), 30, 90);
    assert!(events.is_empty(), "unexpected events {events:?}");
    assert_eq!(w.rest[0], w.rest0[0]);
    assert_eq!(w.beam_flags[0], BEAM_ALIVE);
    let weight_stretch = MASS * G * C;
    assert!((length(&w) - (1.0 + weight_stretch)).abs() < 1e-9, "length {}", length(&w));
}

/// A held load of 1.5 `F_y` on one mass: the beam carries exactly `F_y` every step (`last_lambda`
/// saturates there) and the mass accelerates at `(F - F_y) / m`, the same at 8, 16 and 32
/// substeps, because the cap is a force and not a per-step rule. One yield event, once.
#[test]
fn held_load_past_yield_flows_at_the_excess_force_whatever_the_substeps() {
    let f = 1.5 * FY;
    let a = (f - FY) / MASS;
    for n in [8u32, 16, 32] {
        let mut w = beam_world(-1.0, YIELD, NEVER, 0.0);
        let mut speeds = Vec::new();
        let mut yields = 0;
        for s in 0..40 {
            load(&mut w, -1.0, f);
            step_with_substeps(&mut w, n);
            yields += w.events.iter().filter(|e| e.kind == EVENT_YIELD).count();
            assert!(w.events.iter().all(|e| e.kind == EVENT_YIELD), "{:?}", w.events);
            speeds.push(-w.vel[1].y);
            if s >= 15 {
                let carried = w.last_lambda[0];
                assert!((carried - FY).abs() < 1e-9 * FY, "{n} substeps, step {s}: carries {carried}, F_y {FY}");
            }
        }
        assert_eq!(yields, 1, "{n} substeps");
        assert_eq!(w.beam_flags[0], BEAM_ALIVE | BEAM_YIELDED);
        for s in 15..39 {
            let accel = (speeds[s + 1] - speeds[s]) * 60.0;
            assert!((accel - a).abs() < 1e-8 * a, "{n} substeps, step {s}: accelerates at {accel}, expected {a}");
        }
        // And it is flowing, not holding: the rest length has followed the mass a long way.
        assert!(w.rest[0] - w.rest0[0] > 0.1, "set {}", w.rest[0] - w.rest0[0]);
    }
}

/// Load, then unload: the beam keeps a permanent set, and its elastic stiffness is unchanged.
/// The set has no closed form (it depends on how far the mass flows before the beam stops it),
/// so it is checked against the continuous-time reference. Then, settled with a test-side drag
/// that is zero at rest, the beam sits at its new rest length and a probe load below yield
/// stretches it by exactly `probe · C`: the stiffness did not change.
#[test]
fn load_then_unload_leaves_a_permanent_set_with_unchanged_stiffness() {
    const LOADED: u32 = 12;
    let f = 1.5 * FY;
    let mut w = beam_world(-1.0, YIELD, NEVER, 0.0);
    for s in 0..600 {
        load(&mut w, -1.0, if s < LOADED { f } else { 0.0 });
        if s >= 60 {
            w.add_force(1, w.vel[1] * -30.0); // drag, after the plastic flow has ended
        }
        step_with_substeps(&mut w, SUB);
    }
    let t_load = LOADED as Real / 60.0;
    let (_, set_ref, _) = plastic_reference(|t| if t < t_load { f } else { 0.0 }, 1.0);
    let set = w.rest[0] - w.rest0[0];
    assert!((set - set_ref).abs() < 0.01 * set_ref, "set {set}, reference {set_ref}");
    assert_eq!(w.rest0[0], 1.0);
    assert!(w.vel[1].length() < 1e-9, "not settled: {:?}", w.vel[1]);
    // Unloaded and settled, the length is the rest length.
    assert!((length(&w) - w.rest[0]).abs() < 1e-9, "length {}, rest {}", length(&w), w.rest[0]);
    let before = length(&w);
    // A probe at half the yield force, settled the same way.
    let probe = 0.5 * FY;
    for _ in 0..600 {
        load(&mut w, -1.0, probe);
        w.add_force(1, w.vel[1] * -30.0);
        step_with_substeps(&mut w, SUB);
    }
    let stretch = length(&w) - before;
    assert!((stretch - probe * C).abs() < 1e-6 * probe * C, "probe stretch {stretch}, expected {}", probe * C);
    assert_eq!(w.rest[0] - w.rest0[0], set, "a load below yield must not move the rest length");
}

/// Two parallel bars on one mass, with different yield forces and stiffness. The weaker bar
/// yields first and then carries `F1` while the other takes the rest, so the pair holds any
/// load below `F1 + F2` and collapses above it, at `(F - F1 - F2) / m`.
#[test]
fn parallel_bars_collapse_at_the_sum_of_their_yield_forces() {
    // Bar 1: stiffness 1/C, yield 1 %, so F1 = 36 N. Bar 2: twice as stiff, yield 1.5 %, F2 = 108 N.
    let (c1, c2) = (C, C / 2.0);
    let (f1, f2) = (0.01 / c1, 0.015 / c2);
    let build = || {
        let mut w = World::new();
        w.add_node(Vec3::new(0.0, 0.0, 0.0), 0.0, 0.1);
        w.add_node(Vec3::new(0.0, 0.0, 0.0), 0.0, 0.1);
        w.add_node(Vec3::new(0.0, -1.0, 0.0), 1.0 / MASS, 0.1);
        let bar = |compliance: Real, yield_strain: Real| BeamSpec {
            rest: 1.0,
            compliance,
            damping: 2.0 * (MASS / compliance).sqrt(),
            yield_strain,
            break_strain: NEVER,
        };
        w.add_beam(0, 2, bar(c1, 0.01));
        w.add_beam(1, 2, bar(c2, 0.015));
        w
    };
    // Ramped over two seconds so the structure follows quasi-statically, then held.
    let run = |hold: Real, steps: u32| {
        let mut w = build();
        let mut speeds = Vec::new();
        for s in 0..steps {
            load_on(&mut w, 2, -1.0, hold * (s as Real / 120.0).min(1.0));
            step_with_substeps(&mut w, SUB);
            speeds.push(-w.vel[2].y);
        }
        (w, speeds)
    };

    // Below the sum it holds: still, with the weak bar at its yield force and the strong bar
    // carrying the rest.
    let below = 0.95 * (f1 + f2);
    let (w, _) = run(below, 600);
    assert!(w.vel[2].length() < 1e-3, "still moving: {:?}", w.vel[2]);
    assert_eq!(w.beam_flags[0] & BEAM_YIELDED, BEAM_YIELDED, "the weaker bar must have yielded");
    assert_eq!(w.beam_flags[1] & BEAM_YIELDED, 0, "the stronger bar must not have");
    assert!((w.last_lambda[0] - f1).abs() < 0.03 * f1, "weak bar carries {}, F1 {f1}", w.last_lambda[0]);
    assert!((w.last_lambda[0] + w.last_lambda[1] - below).abs() < 1e-3 * below, "bars carry {} + {}", w.last_lambda[0], w.last_lambda[1]);

    // Above the sum it collapses: both bars at their cap, the mass at the excess force.
    let above = 1.05 * (f1 + f2);
    let (w, speeds) = run(above, 600);
    assert_eq!(w.beam_flags[1] & BEAM_YIELDED, BEAM_YIELDED, "the stronger bar must have yielded");
    for (k, f) in [f1, f2].into_iter().enumerate() {
        assert!((w.last_lambda[k] - f).abs() < 1e-9 * f, "bar {k} carries {}, F_y {f}", w.last_lambda[k]);
    }
    let a = (above - f1 - f2) / MASS;
    for s in 400..599 {
        let accel = (speeds[s + 1] - speeds[s]) * 60.0;
        assert!((accel - a).abs() < 1e-6 * a, "step {s}: accelerates at {accel}, expected {a}");
    }
}

/// Impact: a mass arrives at speed `v0` along the beam with nothing else loading it. It stops
/// when the plastic work has absorbed its kinetic energy. At the stop the elastic part is
/// `F_y² C / 2` (the beam is at its yield force), so the permanent set is
/// `Δ = (m v0² / 2 - F_y² C / 2) / F_y`, and the beam keeps it after the rebound. Tension (a
/// mass thrown away from the pinned end) and compression (a mass driven into it) are the same
/// law with opposite signs, each with one yield event of the matching sign.
#[test]
fn impact_energy_goes_into_plastic_work_in_tension_and_in_compression() {
    let v0: Real = 3.0;
    let expected = (0.5 * MASS * v0 * v0 - 0.5 * FY * FY * C) / FY;
    let mut errors = Vec::new();
    for (side, sign) in [(-1.0, 1.0), (1.0, -1.0)] {
        let mut w = beam_world(side, YIELD, NEVER, 0.0);
        // Tension: moving away from the pinned end is down (side -1); compression: down too
        // (side +1, the mass above the pin).
        w.vel[1] = Vec3::new(0.0, -v0, 0.0);
        let mut events = Vec::new();
        for _ in 0..90 {
            load(&mut w, side, 0.0);
            step_with_substeps(&mut w, SUB);
            events.extend(w.events.iter().copied());
        }
        let set = w.rest[0] - w.rest0[0];
        assert!((set - sign * expected).abs() < 0.01 * expected, "side {side}: set {set}, expected {}", sign * expected);
        assert_eq!(events.len(), 1, "{events:?}");
        assert_eq!((events[0].kind, events[0].index), (EVENT_YIELD, 0));
        assert!(sign * events[0].value > YIELD, "yield strain {} has the wrong sign or size", events[0].value);
        assert_eq!(w.beam_flags[0], BEAM_ALIVE | BEAM_YIELDED);
        assert_eq!(w.rest0[0], 1.0);
    }

    // First order in the substep: doubling the substeps halves the error (here 16 against 64).
    let set_at = |n: u32| {
        let mut w = beam_world(-1.0, YIELD, NEVER, 0.0);
        w.vel[1] = Vec3::new(0.0, -v0, 0.0);
        for _ in 0..90 {
            load(&mut w, -1.0, 0.0);
            step_with_substeps(&mut w, n);
        }
        w.rest[0] - w.rest0[0]
    };
    for n in [16u32, 64] {
        errors.push((expected - set_at(n)).abs());
    }
    assert!(errors[1] < errors[0] / 3.0, "errors at 16 and 64 substeps: {errors:?}");
}

/// Past break: one event on the right step, the beam stops carrying load, its slot stays.
#[test]
fn past_break_breaks_once_and_stops_carrying_load() {
    let mut w = hanging(YIELD, BREAK);
    // One step of a load of 40 yield forces. The beam yields at once and flows; at the end of
    // step 1 it is far past the break strain, which the reference gives.
    let f = 40.0 * FY;
    load(&mut w, -1.0, f);
    step_with_substeps(&mut w, SUB);
    let events = w.events.clone();
    assert_eq!(events.len(), 1, "{events:?}");
    assert_eq!((events[0].kind, events[0].index, events[0].step), (EVENT_BREAK, 0, 1));
    let (x_ref, _, _) = plastic_reference(|_| f, 1.0 / 60.0);
    assert!(x_ref > BREAK, "scene should break on step 1");
    assert!((events[0].value - x_ref).abs() < 0.03 * x_ref, "break strain {}, reference {x_ref}", events[0].value);
    assert_eq!(w.beam_flags[0], BEAM_ALIVE | BEAM_YIELDED | BEAM_BROKEN);
    assert!(!w.beam_alive(0));
    assert_eq!(w.last_lambda[0], 0.0);

    // From here the mass is in free fall: no beam holds it. It leaves with the speed it had at
    // the break, so one second drops it `0.5 g t^2 - v t`.
    let (y0, vy) = (w.pos[1].y, w.vel[1].y);
    let later = pull(&mut w, 0.0, 0, 60);
    assert!(later.is_empty(), "a broken beam must not report again: {later:?}");
    let drop = y0 - w.pos[1].y;
    let expected = 0.5 * G - vy;
    assert!((drop - expected).abs() < 0.01 * expected, "dropped {drop}, expected {expected}");
    assert_eq!(w.last_lambda[0], 0.0);

    // The slot is kept (stable index): the count is unchanged and a new beam does not reuse it.
    assert_eq!(w.beam_count(), 1);
    w.add_node(Vec3::new(1.0, 0.0, 0.0), 1.0, 0.1);
    let fresh = w.add_beam(0, 2, BeamSpec { rest: 1.0, compliance: C, damping: 0.0, yield_strain: YIELD, break_strain: BREAK });
    assert_eq!(fresh.index, 1);
}

/// Break is checked against the original length, in compression too: a strut driven in past the
/// break value breaks, though yield had moved its rest length on the way. A speed of 4 m/s
/// gives the yield event on step 1 (the stroke passes 1 % in about 3 ms) and the break on step
/// 2 (the stroke after 1/60 s is `v t - F_y t² / 2 m` = 0.062 m, after 2/60 s 0.113 m, past
/// the 0.1 m break value).
#[test]
fn break_uses_total_strain_in_compression() {
    let mut w = beam_world(1.0, YIELD, BREAK, 0.0);
    w.vel[1] = Vec3::new(0.0, -4.0, 0.0);
    let mut events = Vec::new();
    for _ in 0..30 {
        load(&mut w, 1.0, 0.0);
        step_with_substeps(&mut w, SUB);
        events.extend(w.events.iter().copied());
    }
    let seen: Vec<(u32, u64)> = events.iter().map(|e| (e.kind, e.step)).collect();
    assert_eq!(seen, vec![(EVENT_YIELD, 1), (EVENT_BREAK, 2)], "{events:?}");
    assert!(events[0].value < -YIELD, "yield strain is negative: {}", events[0].value);
    assert!(events[1].value < -BREAK && events[1].value > -BREAK - 0.03, "break strain {}", events[1].value);
    // The yield moved the rest length in, but the break compared against rest0, not the moved rest.
    assert!(w.rest[0] < 1.0 && w.rest[0] > 0.8 && w.beam_flags[0] & BEAM_BROKEN != 0);
}

/// Events in one step come out in ascending beam order, mixed kinds included. (That is the order
/// inside one island; with several islands the lists are joined in island order first, and
/// there is one island for now.) Three identical beams under the same impact; the same break
/// value 1.1 % just above the yield value 1 % makes beams 0 and 2 break on the step the
/// stroke passes it and beam 1 only yield.
#[test]
fn events_are_in_ascending_beam_order() {
    let mut w = World::new();
    for i in 0..3 {
        w.add_node(Vec3::new(i as Real, 0.0, 0.0), 0.0, 0.1);
        w.add_node(Vec3::new(i as Real, -1.0, 0.0), 1.0, 0.1);
        w.vel[2 * i + 1] = Vec3::new(0.0, -4.0, 0.0);
    }
    let hold = |break_strain| BeamSpec { rest: 1.0, compliance: C, damping: 0.0, yield_strain: YIELD, break_strain };
    w.add_beam(0, 1, hold(0.011));
    w.add_beam(2, 3, hold(0.5));
    w.add_beam(4, 5, hold(0.011));
    step_with_substeps(&mut w, SUB);
    let seen: Vec<(u32, u32)> = w.events.iter().map(|e| (e.index, e.kind)).collect();
    assert_eq!(seen, vec![(0, EVENT_BREAK), (1, EVENT_YIELD), (2, EVENT_BREAK)], "{:?}", w.events);
    assert!(w.events.iter().all(|e| e.step == w.step_count));
}

/// A broken beam is out of the solve and out of the substep rule: with the stiff beam gone the
/// island needs only the minimum count.
#[test]
fn broken_beams_are_skipped_by_the_substep_rule() {
    let mut w = World::new();
    w.add_node(Vec3::new(0.0, 0.0, 0.0), 0.0, 0.1);
    w.add_node(Vec3::new(0.0, -1.0, 0.0), 1.0, 0.1);
    w.add_node(Vec3::new(5.0, 0.0, 0.0), 0.0, 0.1);
    w.add_node(Vec3::new(5.0, -1.0, 0.0), 1.0, 0.1);
    // Stiff (omega 1000 rad/s, 17 substeps) and frail; and soft and strong.
    w.add_beam(0, 1, BeamSpec { rest: 1.0, compliance: 1e-6, damping: 0.0, yield_strain: 5e-8, break_strain: 1e-7 });
    w.add_beam(2, 3, BeamSpec { rest: 1.0, compliance: 0.02, damping: 0.0, yield_strain: 0.5, break_strain: 0.9 });
    assert_eq!(step(&mut w), 17);
    assert!(w.events.iter().any(|e| e.kind == EVENT_BREAK && e.index == 0), "{:?}", w.events);
    assert_eq!(step(&mut w), MIN_SUBSTEPS);
    assert!(w.beam_alive(1));
}

/// A scene that never reaches yield: the cap never fires, so the beam arrays and the hash are
/// those of a solver without a cap. Two proofs: the hash equals the one the previous solver
/// (yield checked once per step, no cap) gave for this scene, recorded before the change; and
/// the same scene with the yield strain raised out of reach (an effectively absent cap) has
/// the same hash. The first stops anyone changing the elastic maths by accident.
#[test]
fn elastic_only_scenes_keep_their_hash() {
    let run = |yield_strain: Real| {
        let mut w = World::new();
        w.add_node(Vec3::new(0.0, 0.0, 0.0), 0.0, 0.1);
        w.add_node(Vec3::new(1.0, 0.1, 0.2), 1.0, 0.1);
        w.add_node(Vec3::new(1.5, 1.0, -0.3), 0.7, 0.1);
        let len = |w: &World, a: usize, b: usize| (w.pos[b] - w.pos[a]).length();
        let spec = |rest: Real, c: Real| BeamSpec { rest, compliance: c, damping: 3.0, yield_strain, break_strain: 0.05 };
        let (l01, l12, l02) = (len(&w, 0, 1), len(&w, 1, 2), len(&w, 0, 2));
        w.add_beam(0, 1, spec(l01, 1e-5));
        w.add_beam(1, 2, spec(l12, 1e-4));
        w.add_beam(0, 2, spec(l02, 1e-5));
        for k in 0..600 {
            if k == 10 {
                w.add_force(2, Vec3::new(5.0, 0.0, 0.0));
            }
            step(&mut w);
            assert!(w.events.is_empty(), "{:?}", w.events);
        }
        assert!(w.beam_flags.iter().all(|&f| f == BEAM_ALIVE), "{:?}", w.beam_flags);
        state_hash(&w)
    };
    assert_eq!(run(0.01), 0x0a36_1cbc_ccec_a95b, "elastic hash changed");
    assert_eq!(run(0.01), run(1e9));
}

/// The same inputs give the same events and the same hash, and the hash sees rest and flags.
#[test]
fn same_inputs_same_events_and_hash() {
    let run = || {
        let mut w = hanging(YIELD, BREAK);
        let mut events = Vec::new();
        for s in 0..120u32 {
            // Grows past yield, then past break.
            if s < 40 {
                w.add_force(1, Vec3::new(0.0, -load_for(0.009 + 0.01 * s as Real), 0.0));
            }
            step(&mut w);
            events.extend(w.events.iter().copied());
        }
        (state_hash(&w), events)
    };
    let (h1, e1) = run();
    let (h2, e2) = run();
    assert_eq!(h1, h2);
    assert_eq!(e1, e2);
    assert!(e1.iter().any(|e| e.kind == EVENT_YIELD) && e1.iter().any(|e| e.kind == EVENT_BREAK), "{e1:?}");

    let base = state_hash(&hanging(YIELD, BREAK));
    let mut moved = hanging(YIELD, BREAK);
    moved.rest[0] += 1e-12;
    assert_ne!(state_hash(&moved), base, "hash must see the rest length");
    for flag in [BEAM_BROKEN, BEAM_YIELDED] {
        let mut flagged = hanging(YIELD, BREAK);
        flagged.beam_flags[0] |= flag;
        assert_ne!(state_hash(&flagged), base, "hash must see flag {flag}");
    }
}
