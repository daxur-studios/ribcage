//! Phase 3.3: beam midpoints on the ground.
//!
//! Scenes: a long beam (two nodes joined by one stiff, lightly damped beam) dropped across a
//! one-cell plateau, where only the beam's midpoint can touch the ground, and dropped flat on
//! flat ground, where both ends and the midpoint do. Plus the weighted endpoint in the kernel.
//!
//! The momentum check: every step, the change in the beam's total momentum is gravity plus the
//! ground's impulse. The ground only pushes, so that impulse is never downward, and over the
//! whole run it sums to the weight times the time (the beam ends at rest).

use beam_core::real::{Real, Vec3};
use beam_core::sim::{state_hash, step, step_with_rule};
use beam_core::solver::{apply_correction, Endpoint, SubstepRule};
use beam_core::terrain::Terrain;
use beam_core::world::{BeamSpec, World, NODE_FROZEN};
use beam_core::STEP_SECONDS;

const G: Real = 9.81;
const RADIUS: Real = 0.1;
const MU: Real = 0.8;
const PLATEAU: Real = 0.5;

/// A 2 m wide strip of ground, flat at zero except two grid points (x = ±0.5) raised to
/// [`PLATEAU`]: a one-cell plateau 1 m wide with flanks 1 m wide down to the plain. Constant
/// along z, so every triangle's plane is `y = PLATEAU` on top and slopes by 0.5 on the flanks.
fn ridge_world() -> World {
    let mut t = Terrain::flat(8, 4, -3.5, -1.5, 1.0, MU);
    for j in 0..4 {
        for i in [3, 4] {
            t.heights[i + j * 8] = PLATEAU;
        }
    }
    let mut w = World::new();
    w.terrain = Some(t);
    w
}

fn flat_world() -> World {
    let mut w = World::new();
    w.terrain = Some(Terrain::flat(2, 2, -100.0, -100.0, 200.0, MU));
    w
}

/// A beam 3 m long along x, centred on `x = 0`, ends at `y`, masses `ma` and `mb` kilograms.
fn add_beam(w: &mut World, y: Real, ma: Real, mb: Real) -> [u32; 2] {
    let a = w.add_node(Vec3::new(-1.5, y, 0.0), 1.0 / ma, RADIUS).index;
    let b = w.add_node(Vec3::new(1.5, y, 0.0), 1.0 / mb, RADIUS).index;
    w.add_beam(a, b, BeamSpec { rest: 3.0, compliance: 3.0 / 2e8, damping: 50.0, yield_strain: 1.0, break_strain: 2.0 });
    [a, b]
}

/// Total momentum of the two nodes.
fn momentum(w: &World, nodes: [u32; 2]) -> Vec3 {
    let mut p = Vec3::ZERO;
    for i in nodes {
        p = p + w.vel[i as usize] * (1.0 / w.inv_mass[i as usize]);
    }
    p
}

fn mass(w: &World, nodes: [u32; 2]) -> Real {
    nodes.iter().map(|&i| 1.0 / w.inv_mass[i as usize]).sum()
}

/// One step with the quiet-run checks every test here wants: no events, no clamp, no freeze.
fn quiet_step(w: &mut World) {
    step(w);
    assert!(w.events.is_empty(), "events {:?}", w.events);
    assert_eq!(w.clamped_nodes, 0, "a node was clamped");
    assert!((0..w.node_count()).all(|i| w.node_flags[i as usize] & NODE_FROZEN == 0), "a node froze");
}

/// What a run shows: the lowest the midpoint went, the lowest either end went, and the ground
/// impulse each step.
struct Run {
    min_mid: Real,
    min_end: Real,
    max_level_error: Real,
    steps: u64,
    impulse: Vec3,
    least_push: Real,
    max_sideways: Real,
}

fn run(w: &mut World, nodes: [u32; 2], steps: u64) -> Run {
    let m = mass(w, nodes);
    let mut r = Run {
        min_mid: Real::MAX,
        min_end: Real::MAX,
        max_level_error: 0.0,
        steps,
        impulse: Vec3::ZERO,
        least_push: Real::MAX,
        max_sideways: 0.0,
    };
    for _ in 0..steps {
        let before = momentum(w, nodes);
        quiet_step(w);
        let after = momentum(w, nodes);
        let step_impulse = after - before + Vec3::new(0.0, m * G * STEP_SECONDS, 0.0);
        r.impulse = r.impulse + step_impulse;
        r.least_push = r.least_push.min(step_impulse.y);
        r.max_sideways = r.max_sideways.max(after.x.abs()).max(after.z.abs());
        let (pa, pb) = (w.pos[nodes[0] as usize], w.pos[nodes[1] as usize]);
        r.min_mid = r.min_mid.min((pa.y + pb.y) * 0.5);
        r.min_end = r.min_end.min(pa.y).min(pb.y);
        r.max_level_error = r.max_level_error.max((pa.y - pb.y).abs());
    }
    r
}

/// The impulse over the run is the weight times the time: the beam starts and ends at rest.
fn assert_weight_carried(w: &World, nodes: [u32; 2], r: &Run) {
    let weight_time = mass(w, nodes) * G * STEP_SECONDS * r.steps as Real;
    assert!(
        (r.impulse.y - weight_time).abs() < 1e-6 * weight_time,
        "ground impulse {} N·s, weight times time {weight_time}",
        r.impulse.y
    );
    assert!(r.least_push > -1e-9, "the ground pulled: a step's impulse was {}", r.least_push);
}

#[test]
fn beam_dropped_across_a_ridge_rests_on_it() {
    let mut w = ridge_world();
    let nodes = add_beam(&mut w, 1.5, 1.0, 1.0);
    let r = run(&mut w, nodes, 480);
    let (pa, pb) = (w.pos[nodes[0] as usize], w.pos[nodes[1] as usize]);
    let rest = PLATEAU + RADIUS;
    println!(
        "midpoint lowest {:.6} (rest {rest}), ends {:.6} and {:.6}, level error {:.3e}, sideways momentum {:.3e}",
        r.min_mid, pa.y, pb.y, r.max_level_error, r.max_sideways
    );
    // Without the midpoint contact the ends would land on the plain at 0.1 and the middle of the
    // beam would be inside the ridge.
    assert!(r.min_mid > rest - 1e-3, "the beam went through the ridge: midpoint down to {}", r.min_mid);
    assert!(((pa.y + pb.y) * 0.5 - rest).abs() < 1e-3, "midpoint at {}", (pa.y + pb.y) * 0.5);
    // Balanced on the ridge, level, with both ends in the air.
    assert!(r.max_level_error < 1e-3, "ends differ by {}", r.max_level_error);
    assert!(pa.y > RADIUS + 0.4 && pb.y > RADIUS + 0.4, "an end touched the plain");
    for i in nodes {
        assert!(w.vel[i as usize].length() < 1e-6, "still moving: {:?}", w.vel[i as usize]);
    }
    assert!(r.max_sideways < 1e-9, "slid sideways: momentum {}", r.max_sideways);
    assert_weight_carried(&w, nodes, &r);
    // Only the midpoint touches: no node contact, one beam contact.
    let stats = step_with_rule(&mut w, SubstepRule::DEFAULT);
    assert_eq!((stats.contacts, stats.beam_contacts), (0, 1));
}

#[test]
fn beam_dropped_flat_lands_level_and_the_ground_carries_its_weight() {
    let mut w = flat_world();
    // Unequal masses: the ground's impulse is not split evenly, but the beam still lands level.
    let nodes = add_beam(&mut w, 1.0, 1.0, 3.0);
    let r = run(&mut w, nodes, 360);
    let (pa, pb) = (w.pos[nodes[0] as usize], w.pos[nodes[1] as usize]);
    println!(
        "ends {:.9} and {:.9}, lowest end {:.6}, level error {:.3e}, least push {:.3e}",
        pa.y, pb.y, r.min_end, r.max_level_error, r.least_push
    );
    assert!(r.min_end > RADIUS - 1e-3, "sank to {}", r.min_end);
    assert!(r.max_level_error < 1e-3, "ends differ by {}", r.max_level_error);
    assert!((pa.y - RADIUS).abs() < 1e-3 && (pb.y - RADIUS).abs() < 1e-3, "rest heights {} and {}", pa.y, pb.y);
    for i in nodes {
        assert!(w.vel[i as usize].length() < 1e-6, "still moving: {:?}", w.vel[i as usize]);
    }
    assert!(r.max_sideways < 1e-9, "slid sideways: momentum {}", r.max_sideways);
    assert_weight_carried(&w, nodes, &r);
    // Resting flat: the two ends and the midpoint are all in contact.
    let stats = step_with_rule(&mut w, SubstepRule::DEFAULT);
    assert_eq!((stats.contacts, stats.beam_contacts), (2, 1));
}

/// A beam with a pinned end: the midpoint's push moves the free node alone, the pinned one
/// never moves, and nothing sinks.
#[test]
fn midpoint_of_a_beam_with_a_pinned_end_is_held_by_the_free_node() {
    let mut w = flat_world();
    // The pinned end is above ground; the free end starts under it and is dug out.
    let a = w.add_node(Vec3::new(0.0, 0.5, 0.0), 0.0, RADIUS).index;
    let b = w.add_node(Vec3::new(0.0, -0.2, 0.0), 1.0, RADIUS).index;
    w.add_beam(a, b, BeamSpec { rest: 0.7, compliance: 0.7 / 2e8, damping: 50.0, yield_strain: 1.0, break_strain: 2.0 });
    for _ in 0..120 {
        quiet_step(&mut w);
    }
    // The free node sits where the ground allows it (its own contact), the pinned one is untouched.
    assert_eq!(w.pos[a as usize], Vec3::new(0.0, 0.5, 0.0));
    assert!(w.pos[b as usize].y > RADIUS - 1e-3, "free end under the ground: {}", w.pos[b as usize].y);
}

/// The kernel moves a point on a beam as the plan says: node `i` by `b_i · w_i · Δλ`, so the
/// point itself by its weight times `Δλ`.
#[test]
fn weighted_endpoint_moves_its_nodes_by_their_shares() {
    let inv_mass = [1.0, 0.5];
    let mut pos = vec![Vec3::new(0.0, 0.0, 0.0), Vec3::new(2.0, 0.0, 0.0)];
    let mid = Endpoint::Point(0, 1, 0.5);
    assert_eq!(mid.weight(&inv_mass), 0.25 * 1.0 + 0.25 * 0.5);
    assert_eq!(mid.place(&pos), Vec3::new(1.0, 0.0, 0.0));
    let up = Vec3::new(0.0, 1.0, 0.0);
    apply_correction(&mut pos, &inv_mass, Endpoint::Fixed, mid, up, 0.8);
    assert!((pos[0].y - 0.4).abs() < 1e-15, "node 0 moved {}", pos[0].y);
    assert!((pos[1].y - 0.2).abs() < 1e-15, "node 1 moved {}", pos[1].y);
    assert!((mid.place(&pos).y - 0.3).abs() < 1e-15, "point moved {}", mid.place(&pos).y);
    // A point off the middle: weights 0.75 and 0.25.
    let third = Endpoint::Point(0, 1, 0.25);
    assert_eq!(third.weight(&inv_mass), 0.75 * 0.75 * 1.0 + 0.25 * 0.25 * 0.5);
    // As the first end, the same shares move the other way.
    let mut back = vec![Vec3::ZERO, Vec3::new(2.0, 0.0, 0.0)];
    apply_correction(&mut back, &inv_mass, third, Endpoint::Fixed, up, 1.0);
    assert!((back[0].y + 0.75).abs() < 1e-15 && (back[1].y + 0.125).abs() < 1e-15, "{:?}", back);
}

#[test]
fn the_same_run_twice_gives_the_same_hash() {
    let hash = || {
        let mut w = ridge_world();
        add_beam(&mut w, 1.5, 1.0, 1.0);
        let n = w.node_count();
        assert_eq!(n, 2);
        for _ in 0..240 {
            step(&mut w);
        }
        state_hash(&w)
    };
    assert_eq!(hash(), hash());
}
