//! Phase 3.2: ground contact and friction.
//!
//! Scenes: a lone node on flat ground (landing, deep spawn), and a braced lattice cube (eight
//! 100 kg nodes, all 28 node pairs joined by stiff, lightly damped beams) on a single-plane
//! slope. The slope is a one-cell grid whose four corner heights lie on `y = −tan θ · x`, so
//! both triangles are the same plane and the ground under the cube is exact everywhere.
//!
//! Expected values. A resting contact's push each substep is the gravity drop `g_n · h²`, so its
//! normal force is exactly the weight it carries and the friction limit is `μ` times it: a
//! block sticks when `tan θ < μ` and slides with `a = g (sin θ − μ cos θ)` when `tan θ > μ`.

use beam_core::real::{Real, Vec3};
use beam_core::sim::{gravity_at, state_hash, step, step_with_rule, total_energy};
use beam_core::solver::{SubstepRule, RECOVERY_SPEED};
use beam_core::terrain::Terrain;
use beam_core::world::{BeamSpec, World, NODE_FROZEN};
use beam_core::STEP_SECONDS;

const G: Real = 9.81;
const MU: Real = 0.5;
const RADIUS: Real = 0.1;

fn flat_world(friction: Real) -> World {
    let mut w = World::new();
    w.terrain = Some(Terrain::flat(2, 2, -100.0, -100.0, 200.0, friction));
    w
}

/// A world whose ground is the plane `y = −tan θ · x` (downhill towards +x).
fn slope_world(tan: Real, friction: Real) -> World {
    let mut t = Terrain::flat(2, 2, -100.0, -100.0, 200.0, friction);
    // Index i + j·nx: x = −100 at i = 0, x = +100 at i = 1, for both rows of z.
    t.heights = vec![100.0 * tan, -100.0 * tan, 100.0 * tan, -100.0 * tan];
    let mut w = World::new();
    w.terrain = Some(t);
    w
}

/// The slope frame: unit downhill tangent and unit normal for `tan θ`.
fn frame(tan: Real) -> (Vec3, Vec3) {
    let cos = 1.0 / (1.0 + tan * tan).sqrt();
    let sin = tan * cos;
    (Vec3::new(cos, -sin, 0.0), Vec3::new(sin, cos, 0.0))
}

const CUBE_MASS: Real = 100.0;
const CUBE_RADIUS: Real = 0.15;

/// A 2 m lattice cube resting on the slope (bottom face touching, at `radius` above the
/// plane), centred at `x = 0`. Every pair of its eight nodes is a beam: edges, face diagonals
/// and body diagonals. Steel-like members (`EA` 2e8 N), light damping, no yield.
fn add_cube(w: &mut World, tan: Real) -> Vec<u32> {
    let (u, n) = frame(tan);
    let z = Vec3::new(0.0, 0.0, 1.0);
    let side = 2.0;
    let mut nodes = Vec::new();
    for k in 0..8u32 {
        let (a, b, c) = ((k & 1) as Real, ((k >> 1) & 1) as Real, ((k >> 2) & 1) as Real);
        let p = u * ((a - 0.5) * side) + n * (CUBE_RADIUS + b * side) + z * ((c - 0.5) * side);
        nodes.push(w.add_node(p, 1.0 / CUBE_MASS, CUBE_RADIUS).index);
    }
    for i in 0..8 {
        for j in i + 1..8 {
            let rest = (w.pos[nodes[j] as usize] - w.pos[nodes[i] as usize]).length();
            let spec = BeamSpec { rest, compliance: rest / 2e8, damping: 5000.0, yield_strain: 1.0, break_strain: 2.0 };
            w.add_beam(nodes[i], nodes[j], spec);
        }
    }
    nodes
}

fn centre(w: &World, nodes: &[u32]) -> (Vec3, Vec3) {
    let (mut p, mut v) = (Vec3::ZERO, Vec3::ZERO);
    for &i in nodes {
        p = p + w.pos[i as usize];
        v = v + w.vel[i as usize];
    }
    let k = 1.0 / nodes.len() as Real;
    (p * k, v * k)
}

/// One step, with the quiet-run checks every test here wants: no events, no clamp, no freeze.
fn quiet_step(w: &mut World) {
    step(w);
    assert!(w.events.is_empty(), "events {:?}", w.events);
    assert_eq!(w.clamped_nodes, 0, "a node was clamped");
    assert!((0..w.node_count()).all(|i| w.node_flags[i as usize] & NODE_FROZEN == 0), "a node froze");
}

#[test]
fn dropped_node_lands_at_radius_without_bounce() {
    let mut w = flat_world(MU);
    w.add_node(Vec3::new(0.3, 2.0, -0.7), 1.0, RADIUS);
    let mut landed = None;
    for s in 1..=600 {
        quiet_step(&mut w);
        let (y, vy) = (w.pos[0].y, w.vel[0].y);
        assert!(y > RADIUS - 1e-3, "step {s}: sank to {y}");
        if landed.is_none() && y < RADIUS + 1e-3 {
            landed = Some(s);
        }
        if let Some(l) = landed {
            assert!((y - RADIUS).abs() < 1e-3, "step {s} (landed at {l}): height {y}");
            assert!(vy <= 1e-9, "step {s}: bounced up at {vy} m/s");
        }
    }
    // About 0.62 s of fall from 1.9 m.
    let l = landed.expect("never landed");
    println!("landed at step {l}; rest height {:e}, minus radius {:e}", w.pos[0].y, w.pos[0].y - RADIUS);
    assert!((36..=39).contains(&l), "landed at step {l}");
    assert!((w.pos[0].y - RADIUS).abs() < 1e-9, "rest height {}", w.pos[0].y);
    assert!(w.vel[0].length() < 1e-9, "still moving: {:?}", w.vel[0]);
    assert_eq!((w.pos[0].x, w.pos[0].z), (0.3, -0.7), "slid sideways on flat ground");
}

#[test]
fn node_spawned_deep_rises_no_faster_than_the_cap() {
    let mut w = flat_world(MU);
    w.add_node(Vec3::new(0.0, RADIUS - 0.5, 0.0), 1.0, RADIUS);
    let mut out = None;
    for s in 1..=600 {
        let y0 = w.pos[0].y;
        quiet_step(&mut w);
        let (y, vy) = (w.pos[0].y, w.vel[0].y);
        let rise = (y - y0) / STEP_SECONDS;
        assert!(rise <= RECOVERY_SPEED * (1.0 + 1e-9), "step {s}: rose at {rise} m/s");
        // Split recovery: digging out moves the node but gives it no upward speed.
        assert!(vy <= 1e-9, "step {s}: upward speed {vy}");
        if out.is_none() && y >= RADIUS - 1e-9 {
            out = Some(s);
        }
        if out.is_some() {
            assert!((y - RADIUS).abs() < 1e-3, "step {s}: hopped or sank to {y}");
        }
    }
    // 0.5 m at 2 m/s: 0.25 s, 15 steps.
    let s = out.expect("never got out");
    println!("out at step {s}; height {:e}", w.pos[0].y);
    assert!((15..=16).contains(&s), "got out at step {s}");
}

/// Runs the cube on a `tan θ` slope; returns the centre's position and velocity after each
/// second (index 0 is the start).
fn run_cube(tan: Real, seconds: u32) -> Vec<(Vec3, Vec3)> {
    let mut w = slope_world(tan, MU);
    let nodes = add_cube(&mut w, tan);
    let mut out = vec![centre(&w, &nodes)];
    for s in 1..=seconds * 60 {
        quiet_step(&mut w);
        if s % 60 == 0 {
            out.push(centre(&w, &nodes));
        }
    }
    let stats = step_with_rule(&mut w, SubstepRule::DEFAULT);
    assert_eq!(stats.contacts, 4, "the bottom face should be the only contacts");
    out
}

#[test]
fn cube_sticks_on_a_gentle_slope() {
    let tan = 0.3; // tan θ < μ = 0.5
    let (u, _) = frame(tan);
    let track = run_cube(tan, 10);
    let drift = (track[10].0 - track[0].0).dot(u);
    let creep = (track[10].0 - track[1].0).dot(u);
    println!("downhill drift over 10 s: {drift:.3e} m, after the first second: {creep:.3e} m");
    assert!(drift.abs() < 1e-3, "drifted {drift} m");
    assert!(creep.abs() < 1e-6, "crept {creep} m");
}

#[test]
fn cube_slides_on_a_steep_slope_with_the_coulomb_acceleration() {
    let tan = 0.9; // tan θ > μ = 0.5
    let (u, _) = frame(tan);
    let cos = 1.0 / (1.0 + tan * tan).sqrt();
    let expected = G * (tan * cos - MU * cos);
    let track = run_cube(tan, 2);
    let a = (track[2].1 - track[1].1).dot(u);
    println!("slide acceleration {a:.6} m/s^2, expected {expected:.6}");
    assert!((a - expected).abs() < 0.05 * expected, "acceleration {a}, expected {expected}");
}

/// Sum of the absolute values of the terms of `total_energy` (as `core/tests/energy.rs`).
fn energy_scale(w: &World) -> Real {
    let mut s = 0.0;
    for i in 0..w.node_count() as usize {
        if w.inv_mass[i] == 0.0 {
            continue;
        }
        let m = 1.0 / w.inv_mass[i];
        s += (0.5 * m * w.vel[i].length_squared()).abs() + (m * gravity_at(w.pos[i]).y * w.pos[i].y).abs();
    }
    for b in 0..w.beam_count() as usize {
        let c = (w.pos[w.beam_b[b] as usize] - w.pos[w.beam_a[b] as usize]).length() - w.rest[b];
        s += 0.5 * c * c / w.compliance[b];
    }
    s
}

/// The cube resting on the gentle slope: after settling, energy never rises above its settled
/// value by more than the rounding tolerance of `energy.rs`, and the cube does not creep.
fn rest_on_slope(steps: u64, settle: u64) -> u64 {
    let tan = 0.3;
    let (u, _) = frame(tan);
    let mut w = slope_world(tan, MU);
    let nodes = add_cube(&mut w, tan);
    let (mut settled, mut tolerance, mut max_rise, mut at) = (0.0, 0.0, Real::MIN, 0);
    let mut settled_centre = Vec3::ZERO;
    for s in 1..=steps {
        quiet_step(&mut w);
        let e = total_energy(&w);
        assert!(e.is_finite(), "step {s}: energy {e}");
        if s == settle {
            settled = e;
            tolerance = 1024.0 * Real::EPSILON * energy_scale(&w);
            settled_centre = centre(&w, &nodes).0;
        }
        if s > settle && e - settled > max_rise {
            max_rise = e - settled;
            at = s;
        }
    }
    let creep = (centre(&w, &nodes).0 - settled_centre).length();
    let drift = (centre(&w, &nodes).0).dot(u);
    println!("max rise {max_rise:.3e} J at step {at} (tolerance {tolerance:.3e} J); creep {creep:.3e} m; drift {drift:.3e} m");
    assert!(max_rise <= tolerance, "energy rose by {max_rise:.3e} J at step {at}, tolerance {tolerance:.3e}");
    assert!(creep < 1e-6, "crept {creep} m after settling");
    assert!(drift.abs() < 1e-3, "drifted {drift} m downhill");
    state_hash(&w)
}

/// Fast version for every `cargo test`: 10 s, settled by 5 s.
#[test]
fn resting_cube_gains_no_energy_600_steps() {
    assert_eq!(rest_on_slope(600, 300), rest_on_slope(600, 300), "two runs gave different hashes");
}

/// Ten minutes of simulated time, settled by one minute.
/// Run with `cargo test --release -- --ignored`.
#[test]
#[ignore = "slow in debug; run with `cargo test --release -- --ignored`"]
fn resting_cube_gains_no_energy_in_ten_minutes() {
    rest_on_slope(36_000, 3_600);
}

/// Pinned nodes get no contact, and a world with ground but nothing near it finds none.
#[test]
fn pinned_and_distant_nodes_get_no_contact() {
    let mut w = flat_world(MU);
    w.add_node(Vec3::new(0.0, -1.0, 0.0), 0.0, RADIUS); // pinned, below the ground
    w.add_node(Vec3::new(1.0, 5.0, 0.0), 1.0, RADIUS); // falling, far above
    let stats = step_with_rule(&mut w, SubstepRule::DEFAULT);
    assert_eq!(stats.contacts, 0);
    assert_eq!(w.pos[0], Vec3::new(0.0, -1.0, 0.0), "the ground moved a pinned node");
}
