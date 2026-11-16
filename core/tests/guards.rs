//! Phase 1.6: islands, the movement clamp and the bad-number freeze.
//!
//! Islands are connected components of movable nodes; pinned nodes join no island (they are
//! copied into every island that has a beam to them). The clamp scales a node faster than
//! `max_speed` down to it and resets its position to match. A non-finite position or velocity
//! freezes the whole island and leaves every other island untouched.
//!
//! Expected values come from the rules above, never from measured output. Where a test compares
//! one run to another it compares raw bits.

#[path = "support/flat_truss.rs"]
#[allow(dead_code)]
mod flat_truss;

use beam_core::event::{EVENT_BREAK, EVENT_FREEZE};
use beam_core::island::build_islands;
use beam_core::real::{Real, Vec3};
use beam_core::sim::{state_hash, step, step_with_substeps};
use beam_core::solver::{substep_count, MIN_SUBSTEPS};
use beam_core::world::{BeamSpec, World, DEFAULT_MAX_SPEED, NODE_FROZEN};
use beam_core::STEP_SECONDS;
use flat_truss::{add_flat_truss, flat_truss_nodes};

fn bits(v: Vec3) -> [u64; 3] {
    [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()]
}

fn spec(compliance: Real, damping: Real) -> BeamSpec {
    BeamSpec { rest: 1.0, compliance, damping, yield_strain: 1.0, break_strain: 10.0 }
}

/// Two separate trusses, A (nodes 0..6) and B (nodes 6..12), two bays each, eight beams each.
/// Nodes 0 and 1 (A) and 6 and 7 (B) are pinned; the lowest movable nodes are 2 and 8.
const A_FIRST: u32 = 0;
const B_FIRST: u32 = 6;
const A_MOVABLE: u32 = 2;
const B_MOVABLE: u32 = 8;
const STRUCT_NODES: u32 = 6;

fn two_trusses() -> World {
    let mut w = World::new();
    add_flat_truss(&mut w, 0.0, 0.0, 2, 1e-5, 20.0);
    add_flat_truss(&mut w, 10.0, 0.0, 2, 1e-5, 20.0);
    assert_eq!(flat_truss_nodes(2), STRUCT_NODES);
    w
}

/// Pushes both trusses sideways at the tips so they are moving, for `steps` steps.
fn run_pushed(w: &mut World, steps: u32) {
    for _ in 0..steps {
        w.add_force(A_FIRST + STRUCT_NODES - 1, Vec3::new(3.0, 0.0, 0.0));
        w.add_force(B_FIRST + STRUCT_NODES - 1, Vec3::new(-3.0, 0.0, 0.0));
        step(w);
    }
}

/// Node `i` of both worlds has the same position, velocity and flags, bit for bit.
fn same_node(a: &World, b: &World, i: u32) -> bool {
    let u = i as usize;
    bits(a.pos[u]) == bits(b.pos[u]) && bits(a.vel[u]) == bits(b.vel[u]) && a.node_flags[u] == b.node_flags[u]
}

/// Beam `i` of both worlds has the same rest length, flags and force, bit for bit.
fn same_beam(a: &World, b: &World, i: u32) -> bool {
    let u = i as usize;
    a.rest[u].to_bits() == b.rest[u].to_bits()
        && a.beam_flags[u] == b.beam_flags[u]
        && a.last_lambda[u].to_bits() == b.last_lambda[u].to_bits()
}

// ---------------------------------------------------------------------------------------------
// Islands
// ---------------------------------------------------------------------------------------------

/// Two separate structures are two islands, in lowest-movable-node order, each with its own
/// nodes in ascending order (its pins among them).
#[test]
fn separate_structures_are_separate_islands_in_lowest_node_order() {
    let w = two_trusses();
    let islands = build_islands(&w);
    assert_eq!(islands.len(), 2);
    assert_eq!(islands[0].root, A_MOVABLE);
    assert_eq!(islands[1].root, B_MOVABLE);
    assert_eq!(islands[0].node_world, (0..6).collect::<Vec<u32>>());
    assert_eq!(islands[1].node_world, (6..12).collect::<Vec<u32>>());
    assert_eq!(islands[0].beam_world, (0..8).collect::<Vec<u32>>());
    assert_eq!(islands[1].beam_world, (8..16).collect::<Vec<u32>>());
}

/// Order is by the lowest MOVABLE node, not by the lowest node: here the island holding node 1
/// (movable) comes before the island holding node 0 (a pin), whose movable node is 2.
#[test]
fn island_order_follows_the_lowest_movable_node_not_the_pins() {
    let mut w = World::new();
    w.add_node(Vec3::new(0.0, 0.0, 0.0), 0.0, 0.1); // 0: pin of X
    w.add_node(Vec3::new(5.0, -1.0, 0.0), 1.0, 0.1); // 1: Y's mass
    w.add_node(Vec3::new(0.0, -1.0, 0.0), 1.0, 0.1); // 2: X's mass
    w.add_node(Vec3::new(5.0, 0.0, 0.0), 0.0, 0.1); // 3: pin of Y
    w.add_beam(0, 2, spec(1e-5, 0.0));
    w.add_beam(1, 3, spec(1e-5, 0.0));
    let islands = build_islands(&w);
    assert_eq!(islands.len(), 2);
    assert_eq!((islands[0].root, islands[1].root), (1, 2));
    assert_eq!(islands[0].node_world, vec![1, 3]);
    assert_eq!(islands[1].node_world, vec![0, 2]);
}

/// A pinned node shared by two structures does not merge them: two islands, both holding the
/// pin, and each behaves bit for bit as it does alone with its own copy of the pin.
#[test]
fn a_shared_pin_does_not_merge_islands() {
    let shared = || {
        let mut w = World::new();
        w.add_node(Vec3::new(0.0, 0.0, 0.0), 0.0, 0.1);
        for k in 1..=2u32 {
            w.add_node(Vec3::new(k as Real, 0.0, 0.0), 1.0, 0.1); // 1, 2: first arm
        }
        for k in 1..=2u32 {
            w.add_node(Vec3::new(-(k as Real), 0.0, 0.0), 1.0, 0.1); // 3, 4: second arm
        }
        w.add_beam(0, 1, spec(1e-5, 10.0));
        w.add_beam(1, 2, spec(1e-5, 10.0));
        w.add_beam(0, 3, spec(1e-5, 10.0));
        w.add_beam(3, 4, spec(1e-5, 10.0));
        w
    };
    let mut w = shared();
    let islands = build_islands(&w);
    assert_eq!(islands.len(), 2);
    assert_eq!((islands[0].root, islands[1].root), (1, 3));
    assert_eq!(islands[0].node_world, vec![0, 1, 2]);
    assert_eq!(islands[1].node_world, vec![0, 3, 4]);

    let mut alone = World::new();
    alone.add_node(Vec3::new(0.0, 0.0, 0.0), 0.0, 0.1);
    alone.add_node(Vec3::new(1.0, 0.0, 0.0), 1.0, 0.1);
    alone.add_node(Vec3::new(2.0, 0.0, 0.0), 1.0, 0.1);
    alone.add_beam(0, 1, spec(1e-5, 10.0));
    alone.add_beam(1, 2, spec(1e-5, 10.0));
    for _ in 0..120 {
        step(&mut w);
        step(&mut alone);
    }
    assert_eq!(bits(w.pos[0]), bits(Vec3::new(0.0, 0.0, 0.0)), "the pin moved");
    for i in 1..=2u32 {
        assert!(same_node(&w, &alone, i), "arm node {i} differs from the same arm alone");
    }
    assert!(w.pos[2].y < -0.1, "the arm should have sagged");
}

/// A beam that breaks and splits a structure gives two islands on the next step; the piece
/// that lost its pin is its own island.
#[test]
fn a_break_that_splits_a_structure_gives_two_islands_next_step() {
    // Pin 0, then nodes 1 to 4 hanging in a chain. Beam 2 (nodes 2-3) is the weak one.
    let mut w = World::new();
    w.add_node(Vec3::new(0.0, 0.0, 0.0), 0.0, 0.1);
    for k in 1..=4u32 {
        w.add_node(Vec3::new(0.0, -(k as Real), 0.0), 1.0, 0.1);
    }
    for k in 0..4u32 {
        let break_strain = if k == 2 { 0.005 } else { 10.0 };
        w.add_beam(k, k + 1, BeamSpec { rest: 1.0, compliance: 1e-5, damping: 0.0, yield_strain: 1.0, break_strain });
    }
    let before = build_islands(&w);
    assert_eq!(before.len(), 1);
    assert_eq!(before[0].node_world, vec![0, 1, 2, 3, 4]);

    w.add_force(4, Vec3::new(0.0, -1e4, 0.0));
    step(&mut w);
    assert_eq!(w.events.len(), 1, "{:?}", w.events);
    assert_eq!((w.events[0].kind, w.events[0].index), (EVENT_BREAK, 2));

    let after = build_islands(&w);
    assert_eq!(after.len(), 2);
    assert_eq!((after[0].root, after[1].root), (1, 3));
    assert_eq!(after[0].node_world, vec![0, 1, 2]);
    assert_eq!(after[1].node_world, vec![3, 4]);
    assert_eq!(after[1].beam_world, vec![3]);
}

/// A movable node with no beams is an island of its own. A beam between two pins belongs to no
/// component: it sits in one extra island after the others, with no movable node.
#[test]
fn lone_nodes_and_pinned_only_beams() {
    let mut w = World::new();
    w.add_node(Vec3::new(0.0, 0.0, 0.0), 0.0, 0.1); // 0 pin
    w.add_node(Vec3::new(1.0, 0.0, 0.0), 0.0, 0.1); // 1 pin
    w.add_node(Vec3::new(3.0, 5.0, 0.0), 1.0, 0.1); // 2 lone mass
    w.add_node(Vec3::new(9.0, 9.0, 0.0), 0.0, 0.1); // 3 lone pin: in no island
    w.add_beam(0, 1, spec(1e-5, 0.0));
    let islands = build_islands(&w);
    assert_eq!(islands.len(), 2);
    assert_eq!(islands[0].root, 2);
    assert_eq!(islands[0].node_world, vec![2]);
    assert_eq!(islands[1].root, u32::MAX);
    assert_eq!(islands[1].node_world, vec![0, 1]);
    assert_eq!(islands[1].beam_world, vec![0]);
    assert_eq!(substep_count(&islands[1]), MIN_SUBSTEPS);
}

/// Each island gets the substep count its own beams ask for, and a loose island next to a
/// stiff one stays at the minimum. `step` returns the largest count used.
#[test]
fn each_island_has_its_own_substep_count() {
    let mut w = World::new();
    // Stiff island first (omega = 1000 rad/s, so 17 substeps), soft island second.
    w.add_node(Vec3::new(0.0, 0.0, 0.0), 0.0, 0.1);
    w.add_node(Vec3::new(1.0, 0.0, 0.0), 1.0, 0.1);
    w.add_node(Vec3::new(5.0, 0.0, 0.0), 0.0, 0.1);
    w.add_node(Vec3::new(6.0, 0.0, 0.0), 1.0, 0.1);
    w.add_beam(0, 1, spec(1e-6, 0.0));
    w.add_beam(2, 3, spec(0.02, 0.0));
    let islands = build_islands(&w);
    assert_eq!(islands.len(), 2);
    assert_eq!(substep_count(&islands[0]), 17);
    assert_eq!(substep_count(&islands[1]), MIN_SUBSTEPS);
    assert_eq!(step(&mut w), 17);
}

/// Events are joined in island order, not in beam order: here island A (lower nodes) owns the
/// higher beam index, and its event comes first.
#[test]
fn events_are_joined_in_island_order() {
    let mut w = World::new();
    w.add_node(Vec3::new(0.0, 0.0, 0.0), 0.0, 0.1); // 0
    w.add_node(Vec3::new(0.0, -1.0, 0.0), 1.0, 0.1); // 1
    w.add_node(Vec3::new(5.0, 0.0, 0.0), 0.0, 0.1); // 2
    w.add_node(Vec3::new(5.0, -1.0, 0.0), 1.0, 0.1); // 3
    let weak = BeamSpec { rest: 1.0, compliance: 1e-5, damping: 0.0, yield_strain: 0.0005, break_strain: 0.001 };
    w.add_beam(2, 3, weak); // beam 0 belongs to island B
    w.add_beam(0, 1, weak); // beam 1 belongs to island A
    w.add_force(1, Vec3::new(0.0, -1000.0, 0.0));
    w.add_force(3, Vec3::new(0.0, -1000.0, 0.0));
    step(&mut w);
    let seen: Vec<(u32, u32)> = w.events.iter().map(|e| (e.kind, e.index)).collect();
    assert_eq!(seen, vec![(EVENT_BREAK, 1), (EVENT_BREAK, 0)]);
}

// ---------------------------------------------------------------------------------------------
// Clamp
// ---------------------------------------------------------------------------------------------

/// A node thrown at 5000 m/s ends the step at `max_speed`, its position consistent with the
/// clamped velocity (start plus velocity times the substep length, exactly, for one substep),
/// and the counter says one node.
#[test]
fn clamp_limits_speed_and_resets_position() {
    assert_eq!(World::new().max_speed, DEFAULT_MAX_SPEED);
    let mut w = World::new();
    let start = Vec3::new(1.0, 2.0, 3.0);
    w.add_node(start, 1.0, 0.1);
    w.vel[0] = Vec3::new(5000.0, 0.0, 0.0);
    step_with_substeps(&mut w, 1);
    let speed = w.vel[0].length();
    assert!(speed <= DEFAULT_MAX_SPEED * (1.0 + 4.0 * Real::EPSILON), "speed {speed}");
    assert!(speed > 0.999 * DEFAULT_MAX_SPEED, "speed {speed}");
    assert!(w.vel[0].x > 0.0 && w.vel[0].y < 0.0, "direction lost: {:?}", w.vel[0]);
    // One substep of length h = STEP_SECONDS: position is the start plus the clamped velocity times h.
    assert_eq!(bits(w.pos[0]), bits(start + w.vel[0] * STEP_SECONDS));
    assert_eq!(w.clamped_nodes, 1);
}

/// With several substeps the node is clamped in each of them but counted once; it never moves
/// farther than `max_speed` times the step; the counter resets on the next step.
#[test]
fn clamp_counts_each_node_once_per_step_and_resets() {
    let mut w = World::new();
    let start = Vec3::new(0.0, 0.0, 0.0);
    w.add_node(start, 1.0, 0.1);
    w.add_node(Vec3::new(10.0, 0.0, 0.0), 1.0, 0.1); // a second, slow, lone node
    w.vel[0] = Vec3::new(0.0, 5000.0, 0.0);
    step(&mut w); // the rule gives a lone island the minimum count
    assert!(w.vel[0].length() <= DEFAULT_MAX_SPEED * (1.0 + 4.0 * Real::EPSILON));
    let moved = (w.pos[0] - start).length();
    assert!(moved <= DEFAULT_MAX_SPEED * STEP_SECONDS * (1.0 + 1e-12), "moved {moved}");
    assert_eq!(w.clamped_nodes, 1, "one node, however many substeps clamped it");
    w.vel[0] = Vec3::ZERO;
    step(&mut w);
    assert_eq!(w.clamped_nodes, 0);
}

/// A scene that stays under `max_speed` gives the same bits whatever the setting.
#[test]
fn a_scene_under_the_limit_is_bit_identical_with_the_clamp_off() {
    let run = |max_speed: Real| {
        let mut w = two_trusses();
        w.max_speed = max_speed;
        let mut fastest: Real = 0.0;
        for _ in 0..300 {
            run_pushed(&mut w, 1);
            assert_eq!(w.clamped_nodes, 0);
            for v in &w.vel {
                fastest = fastest.max(v.length());
            }
        }
        (state_hash(&w), fastest)
    };
    let (h_default, fastest) = run(DEFAULT_MAX_SPEED);
    let (h_huge, _) = run(Real::MAX);
    assert!(fastest > 0.1 && fastest < 0.1 * DEFAULT_MAX_SPEED, "the scene should move, slowly: {fastest}");
    assert_eq!(h_default, h_huge);
}

// ---------------------------------------------------------------------------------------------
// Freeze
// ---------------------------------------------------------------------------------------------

/// How the bad number is put in.
#[derive(Clone, Copy)]
enum Poison {
    Velocity,
    Force,
}

/// Runs two trusses side by side, poisons one node of the `victim` structure (0 is A, 1 is B)
/// and checks the freeze: one event, the victim restored to its start-of-step positions with
/// zero velocity and flagged, the other structure bit-identical to a clean run, the victim
/// staying put and silent afterwards, and `unfreeze_node` releasing it.
fn check_freeze(victim: u32, poison: Poison) {
    let (first, lowest) = if victim == 0 { (A_FIRST, A_MOVABLE) } else { (B_FIRST, B_MOVABLE) };
    let other = if victim == 0 { B_FIRST } else { A_FIRST };
    let range = |f: u32| f..f + STRUCT_NODES;
    let movable = |f: u32| (f + 2)..(f + STRUCT_NODES);

    let mut w = two_trusses();
    let mut clean = two_trusses();
    run_pushed(&mut w, 20);
    run_pushed(&mut clean, 20);
    assert_eq!(state_hash(&w), state_hash(&clean));

    let before_pos: Vec<Vec3> = range(first).map(|i| w.pos[i as usize]).collect();
    let before_rest: Vec<u64> = (0..8).map(|k| w.rest[(first - first + victim * 8 + k) as usize].to_bits()).collect();
    let target = lowest + 3;
    match poison {
        Poison::Velocity => w.vel[target as usize] = Vec3::new(Real::NAN, 0.0, 0.0),
        Poison::Force => w.add_force(target, Vec3::new(0.0, Real::NAN, 0.0)),
    }
    run_pushed(&mut w, 1);
    run_pushed(&mut clean, 1);

    // Exactly one freeze event, for the victim island, at its first substep.
    assert_eq!(w.events.len(), 1, "{:?}", w.events);
    let e = w.events[0];
    assert_eq!((e.kind, e.index, e.step), (EVENT_FREEZE, lowest, w.step_count));
    assert_eq!(e.value, 1.0);

    // The victim is back where the step started, at rest, flagged. Its pins are untouched.
    for (k, i) in range(first).enumerate() {
        assert_eq!(bits(w.pos[i as usize]), bits(before_pos[k]), "node {i} not restored");
        if movable(first).contains(&i) {
            assert_eq!(bits(w.vel[i as usize]), bits(Vec3::ZERO), "node {i} velocity");
            assert!(w.node_frozen(i), "node {i} not flagged");
        } else {
            assert!(!w.node_frozen(i), "pin {i} flagged");
            assert_eq!(bits(w.vel[i as usize]), bits(Vec3::ZERO), "pin {i} velocity");
        }
    }
    for k in 0..8u32 {
        assert_eq!(w.rest[(victim * 8 + k) as usize].to_bits(), before_rest[k as usize]);
    }
    // The other structure is bit-identical to the clean run, and carries no flag.
    for i in range(other) {
        assert!(same_node(&w, &clean, i), "other structure node {i} changed");
        assert!(!w.node_frozen(i));
    }
    for k in 0..8u32 {
        let b = (1 - victim) * 8 + k;
        assert!(same_beam(&w, &clean, b), "other structure beam {b} changed");
    }
    // The flag is part of the hash.
    assert_ne!(state_hash(&w), state_hash(&clean));

    // Later steps: the victim stays put and silent; the other keeps matching the clean run.
    for s in 0..10 {
        run_pushed(&mut w, 1);
        run_pushed(&mut clean, 1);
        assert!(w.events.is_empty(), "step {s}: {:?}", w.events);
        for (k, i) in range(first).enumerate() {
            assert_eq!(bits(w.pos[i as usize]), bits(before_pos[k]), "step {s}: node {i} moved");
            assert_eq!(bits(w.vel[i as usize]), bits(Vec3::ZERO), "step {s}: node {i} velocity");
        }
        for i in range(other) {
            assert!(same_node(&w, &clean, i), "step {s}: other structure node {i} changed");
        }
    }

    // The host can release it: flags clear, the island is solved again and falls.
    for i in movable(first) {
        w.unfreeze_node(i);
        assert_eq!(w.node_flags[i as usize] & NODE_FROZEN, 0);
    }
    run_pushed(&mut w, 5);
    assert!(w.events.is_empty(), "{:?}", w.events);
    assert!(w.pos[target as usize].y < before_pos[(target - first) as usize].y - 1e-4, "released island did not move");
}

#[test]
fn nan_velocity_in_b_freezes_only_b() {
    check_freeze(1, Poison::Velocity);
}

#[test]
fn nan_force_in_b_freezes_only_b() {
    check_freeze(1, Poison::Force);
}

/// The mirror case: the bad number is in A, the structure with the lower indices.
#[test]
fn nan_in_a_freezes_only_a() {
    check_freeze(0, Poison::Velocity);
    check_freeze(0, Poison::Force);
}

/// An infinite speed is not clamped to a number: the clamp scale is zero times infinity, which
/// is NaN, so it ends in a freeze like any other bad number.
#[test]
fn infinite_velocity_freezes_the_island() {
    let mut w = two_trusses();
    w.vel[B_MOVABLE as usize] = Vec3::new(Real::INFINITY, 0.0, 0.0);
    step(&mut w);
    assert_eq!(w.events.len(), 1);
    assert_eq!((w.events[0].kind, w.events[0].index), (EVENT_FREEZE, B_MOVABLE));
    assert!(w.node_frozen(B_MOVABLE) && !w.node_frozen(A_MOVABLE));
    assert!(w.pos.iter().all(|p| p.x.is_finite() && p.y.is_finite() && p.z.is_finite()));
}
