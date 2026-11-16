//! Phase 1.5: yield and break, checked once per step after the last substep.
//!
//! The solver-loaded tests use a single hanging beam, which is exact at any substep count (ER 001,
//! phases 1.3 and 1.4), so the 40% softness a multi-beam structure shows at the rule's count does
//! not enter. A single beam is also fast: omega = 600 rad/s with critical damping settles in a few
//! milliseconds, so every 60 Hz step ends at the static stretch and the load can be treated as
//! quasi-static. The machine-controlled tests (a pinned grip moved by the test) need no solver.

use beam_core::event::{Event, EVENT_BREAK, EVENT_YIELD};
use beam_core::real::{Real, Vec3};
use beam_core::sim::{state_hash, step};
use beam_core::solver::MIN_SUBSTEPS;
use beam_core::world::{BeamSpec, World, BEAM_ALIVE, BEAM_BROKEN, BEAM_YIELDED};

const G: Real = 9.81;
const MASS: Real = 1.0;
const OMEGA: Real = 600.0;
/// Compliance that gives the hanging mass the natural frequency `OMEGA`.
const C: Real = 1.0 / (MASS * OMEGA * OMEGA);
const YIELD: Real = 0.01;
const BREAK: Real = 0.1;

/// A 1 m beam hanging from a pinned node, with a 1 kg mass, critically damped.
fn hanging(yield_strain: Real, break_strain: Real) -> World {
    let mut w = World::new();
    w.add_node(Vec3::new(0.0, 0.0, 0.0), 0.0, 0.1);
    w.add_node(Vec3::new(0.0, -1.0, 0.0), 1.0 / MASS, 0.1);
    let damping = 2.0 * MASS * OMEGA; // critical for one mass on one spring
    w.add_beam(0, 1, BeamSpec { rest: 1.0, compliance: C, damping, yield_strain, break_strain });
    w
}

/// Downward force that gives a settled elastic stretch of `strain` (times the 1 m rest length).
fn load_for(strain: Real) -> Real {
    strain / C - MASS * G
}

fn length(w: &World) -> Real {
    (w.pos[1] - w.pos[0]).length()
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

/// Past yield the beam stays longer. Under a constant load the beam is perfectly plastic and
/// flows: each step the rest length moves to `length - yield * rest0`, then the solver pulls the
/// length out to `rest + stretch` again, so the rest grows by `stretch - yield * rest0` per step
/// until the load is gone (or the beam breaks). Load for `K` steps, then none: the permanent set
/// is `K * (stretch - yield)` metres. That is the predicted amount, from the rule alone.
#[test]
fn past_yield_stays_longer_by_the_predicted_amount() {
    const K: u32 = 4;
    let stretch = 0.015; // elastic strain of the load, 1.5 times the yield value
    let mut w = hanging(YIELD, BREAK);
    let events = pull(&mut w, load_for(stretch), K, 90);

    // One yield event, on the first loaded step, for the right beam; nothing else.
    assert_eq!(events.len(), 1, "{events:?}");
    let e = events[0];
    assert_eq!((e.kind, e.index, e.step), (EVENT_YIELD, 0, 1));
    assert!((e.value - stretch).abs() < 0.02 * stretch, "yield event strain {}", e.value);

    // The rest length moved by the predicted amount; rest0 never moves.
    let predicted = K as Real * (stretch - YIELD);
    let set = w.rest[0] - w.rest0[0];
    assert!((set - predicted).abs() < 0.01 * predicted, "set {set}, predicted {predicted}");
    assert_eq!(w.rest0[0], 1.0);
    assert_eq!(w.beam_flags[0], BEAM_ALIVE | BEAM_YIELDED);

    // Unloaded and settled, the beam is longer than it started by that set (plus its own weight).
    let longer = length(&w) - (1.0 + MASS * G * C);
    assert!((longer - predicted).abs() < 0.01 * predicted, "longer by {longer}, predicted {predicted}");
}

/// The grip test: both ends pinned, one moved by the test (a testing machine). The beam's length
/// is then exactly the grip distance, so the rest length after each yield is exact: in tension
/// `length - yield`, in compression `length + yield`.
#[test]
fn yield_works_in_tension_and_in_compression() {
    let grip = |to: Real| -> (World, Vec<Event>) {
        let mut w = World::new();
        w.add_node(Vec3::new(0.0, 0.0, 0.0), 0.0, 0.1);
        w.add_node(Vec3::new(1.0, 0.0, 0.0), 0.0, 0.1);
        w.add_beam(0, 1, BeamSpec { rest: 1.0, compliance: 1e-6, damping: 0.0, yield_strain: YIELD, break_strain: BREAK });
        let mut events = Vec::new();
        for s in 1..=100 {
            w.pos[1].x = 1.0 + (to - 1.0) * s as Real / 100.0;
            step(&mut w);
            events.extend(w.events.iter().copied());
        }
        (w, events)
    };
    let (w, ev) = grip(1.04);
    assert!((w.rest[0] - 1.03).abs() < 1e-12, "tension rest {}", w.rest[0]);
    assert_eq!(ev.len(), 1);
    assert_eq!(ev[0].kind, EVENT_YIELD);
    let (w, ev) = grip(0.96);
    assert!((w.rest[0] - 0.97).abs() < 1e-12, "compression rest {}", w.rest[0]);
    assert_eq!(ev.len(), 1);
    assert_eq!(ev[0].kind, EVENT_YIELD);
    assert!(ev[0].value < -YIELD, "compression strain is negative: {}", ev[0].value);

    // Pull to 1.04, then push the grip back to its start: the beam, now longer than its original
    // length, is squeezed past its elastic limit the other way and yields again in compression.
    let mut w = World::new();
    w.add_node(Vec3::new(0.0, 0.0, 0.0), 0.0, 0.1);
    w.add_node(Vec3::new(1.0, 0.0, 0.0), 0.0, 0.1);
    w.add_beam(0, 1, BeamSpec { rest: 1.0, compliance: 1e-6, damping: 0.0, yield_strain: YIELD, break_strain: BREAK });
    for s in 1..=50 {
        w.pos[1].x = 1.0 + 0.04 * s as Real / 50.0;
        step(&mut w);
    }
    for s in 1..=50 {
        w.pos[1].x = 1.04 - 0.04 * s as Real / 50.0;
        step(&mut w);
    }
    assert!((w.rest[0] - 1.01).abs() < 1e-12, "reversed rest {}", w.rest[0]);
    assert_eq!(w.rest0[0], 1.0);
}

/// Past break: one event on the right step, the beam stops carrying load, its slot stays.
#[test]
fn past_break_breaks_once_and_stops_carrying_load() {
    let mut w = hanging(YIELD, BREAK);
    // One step of a load worth twice the break strain, then nothing. The beam is settled to
    // within a fraction of a per mille by the end of that step, so it breaks on step 1.
    let events = pull(&mut w, load_for(2.0 * BREAK), 1, 1);
    assert_eq!(events.len(), 1, "{events:?}");
    assert_eq!((events[0].kind, events[0].index, events[0].step), (EVENT_BREAK, 0, 1));
    assert!((events[0].value - 2.0 * BREAK).abs() < 0.02 * 2.0 * BREAK, "break strain {}", events[0].value);
    assert_eq!(w.beam_flags[0], BEAM_ALIVE | BEAM_BROKEN);
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

/// Break is checked against the original length, in compression too: a grip squeezed past the
/// break value breaks the beam, even though yield had moved its rest length on the way.
#[test]
fn break_uses_total_strain_in_compression() {
    let mut w = World::new();
    w.add_node(Vec3::new(0.0, 0.0, 0.0), 0.0, 0.1);
    w.add_node(Vec3::new(1.0, 0.0, 0.0), 0.0, 0.1);
    w.add_beam(0, 1, BeamSpec { rest: 1.0, compliance: 1e-6, damping: 0.0, yield_strain: YIELD, break_strain: BREAK });
    let mut events = Vec::new();
    for s in 1..=100 {
        w.pos[1].x = 1.0 - 0.15 * s as Real / 100.0;
        step(&mut w);
        events.extend(w.events.iter().copied());
    }
    let kinds: Vec<u32> = events.iter().map(|e| e.kind).collect();
    assert_eq!(kinds, vec![EVENT_YIELD, EVENT_BREAK], "{events:?}");
    assert!(events[1].value < -BREAK && events[1].value > -BREAK - 0.01, "break strain {}", events[1].value);
    // The yield moved the rest length in, but the break compared against rest0, not the moved rest.
    assert!(w.rest[0] > 0.9 && w.beam_flags[0] & BEAM_BROKEN != 0);
}

/// Events in one step come out in ascending beam order, mixed kinds included.
#[test]
fn events_are_in_ascending_beam_order() {
    let mut w = World::new();
    let c = 1e-3;
    for i in 0..3 {
        w.add_node(Vec3::new(i as Real, 0.0, 0.0), 0.0, 0.1);
        w.add_node(Vec3::new(i as Real, -1.0, 0.0), 1.0, 0.1);
    }
    // Identical dynamics (strain 0.0016 after the first step); the same limit of 0.0015 is a
    // break for beams 0 and 2 and a yield for beam 1, so all three trigger on the same step.
    let hold = |yield_strain, break_strain| BeamSpec { rest: 1.0, compliance: c, damping: 0.0, yield_strain, break_strain };
    w.add_beam(0, 1, hold(0.001, 0.0015));
    w.add_beam(2, 3, hold(0.0015, 0.1));
    w.add_beam(4, 5, hold(0.001, 0.0015));
    let mut first = Vec::new();
    for _ in 0..60 {
        step(&mut w);
        if !w.events.is_empty() {
            first = w.events.clone();
            break;
        }
    }
    let seen: Vec<(u32, u32)> = first.iter().map(|e| (e.index, e.kind)).collect();
    assert_eq!(seen, vec![(0, EVENT_BREAK), (1, EVENT_YIELD), (2, EVENT_BREAK)], "{first:?}");
    assert!(first.iter().all(|e| e.step == w.step_count));
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

/// The same inputs give the same events and the same hash, and the hash sees rest and flags.
#[test]
fn same_inputs_same_events_and_hash() {
    let run = || {
        let mut w = hanging(YIELD, BREAK);
        let mut events = Vec::new();
        for s in 0..120u32 {
            // Grows past yield, then past break.
            if s < 12 {
                w.add_force(1, Vec3::new(0.0, -load_for(0.009 + 0.002 * s as Real), 0.0));
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
