//! Phase 4.6: thrusters. A push along the line from a tail node to a head node, as plain external
//! force before the step; bad records are rejected like any other; thrusters are in the state hash.

use beam_core::command::*;
use beam_core::real::{Real, Vec3};
use beam_core::sim::{state_hash, step};
use beam_core::world::World;

fn rec(kind: u32, a: u32, b: u32, f: [Real; 6]) -> Command {
    Command { kind, a, b, reserved: 0, f }
}

fn node(x: f64, y: f64, z: f64, mass: f64) -> Command {
    rec(CMD_ADD_NODE, 0, 0, [x, y, z, 1.0 / mass, 0.1, 0.0])
}

fn beam(a: u32, b: u32, rest: f64) -> Command {
    rec(CMD_ADD_BEAM, a, b, [rest, 1e-7, 5.0, 0.2, 0.5, 0.0])
}

fn thruster(tail: u32, head: u32, max: f64) -> Command {
    rec(CMD_ADD_THRUSTER, tail, head, [max, 0.0, 0.0, 0.0, 0.0, 0.0])
}

fn throttle(t: u32, v: f64) -> Command {
    rec(CMD_SET_THROTTLE, t, 0, [v, 0.0, 0.0, 0.0, 0.0, 0.0])
}

fn apply(w: &mut World, cmds: &[Command]) -> (Vec<u32>, u32) {
    let mut r = Vec::new();
    let rejected = apply_commands(w, cmds, &mut r);
    (r, rejected)
}

/// Total momentum, `sum m v`, of every live movable node.
fn momentum(w: &World) -> Vec3 {
    let mut p = Vec3::ZERO;
    for i in 0..w.node_count() as usize {
        if w.node_alive(i as u32) && w.inv_mass[i] != 0.0 {
            p = p + w.vel[i] * (1.0 / w.inv_mass[i]);
        }
    }
    p
}

/// Two unequal masses 2 m apart along x, tied by a stiff beam, thruster from the left to the right
/// one. Gravity pulls along -y only, so along x the pair is in free space.
fn pair(max: f64) -> World {
    let mut w = World::new();
    let cmds = [node(0.0, 100.0, 0.0, 30.0), node(2.0, 100.0, 0.0, 70.0), beam(0, 1, 2.0), thruster(0, 1, max)];
    let (r, rejected) = apply(&mut w, &cmds);
    assert_eq!((rejected, r[3]), (0, 0), "the thruster is number 0");
    w
}

#[test]
fn a_pair_accelerates_at_thrust_over_total_mass() {
    let max = 3000.0;
    let mut w = pair(max);
    apply(&mut w, &[throttle(0, 1.0)]);
    for _ in 0..60 {
        step(&mut w);
    }
    let p = momentum(&w);
    // One second of constant force: momentum is thrust times time (to rounding), along x.
    assert!((p.x - max).abs() < 1e-6 * max, "p.x = {}", p.x);
    assert!(p.z.abs() < 1e-9, "p.z = {}", p.z);
    // Gravity is untouched: 100 kg times g times 1 s.
    assert!((p.y + 100.0 * 9.81).abs() < 1e-6 * 981.0, "p.y = {}", p.y);
    // So the pair's speed along x is thrust over total mass times time.
    let v = p.x / 100.0;
    assert!((v - max / 100.0).abs() < 1e-6, "v = {v}");
    // And it did not stretch the beam much: both ends moved nearly alike.
    let gap = w.pos[1].x - w.pos[0].x;
    assert!((gap - 2.0).abs() < 1e-3, "gap = {gap}");
}

#[test]
fn half_throttle_is_half_the_force_and_direction_follows_the_nodes() {
    let max = 3000.0;
    let mut w = World::new();
    // Tail at the origin, head up and to the side along 3-4-5, so the unit vector is (0.6, 0.8, 0).
    apply(&mut w, &[node(0.0, 100.0, 0.0, 10.0), node(3.0, 104.0, 0.0, 10.0), beam(0, 1, 5.0), thruster(0, 1, max), throttle(0, 0.5)]);
    for _ in 0..30 {
        step(&mut w);
    }
    let p = momentum(&w);
    let t = 0.5; // seconds
    assert!((p.x - 0.5 * max * 0.6 * t).abs() < 1e-3, "p.x = {}", p.x);
    // y: thrust up and gravity down; the body turns a little (its line of thrust passes through
    // its centre, so almost not at all), so a loose bound is enough here.
    let expect_y = 0.5 * max * 0.8 * t - 20.0 * 9.81 * t;
    assert!((p.y - expect_y).abs() < 0.01 * expect_y.abs(), "p.y = {}, expected {expect_y}", p.y);
}

#[test]
fn throttle_zero_gives_no_force() {
    let mut with = pair(5000.0);
    let mut without = World::new();
    apply(&mut without, &[node(0.0, 100.0, 0.0, 30.0), node(2.0, 100.0, 0.0, 70.0), beam(0, 1, 2.0)]);
    for _ in 0..60 {
        step(&mut with);
        step(&mut without);
    }
    assert_eq!(with.pos, without.pos);
    assert_eq!(with.vel, without.vel);
    // The thruster is still part of the state, so the hashes differ.
    assert_ne!(state_hash(&with), state_hash(&without));
}

#[test]
fn a_throttle_stays_until_it_is_set_again() {
    let mut w = pair(3000.0);
    apply(&mut w, &[throttle(0, 1.0)]);
    for _ in 0..30 {
        step(&mut w);
    }
    let half = momentum(&w).x;
    apply(&mut w, &[throttle(0, 0.0)]);
    for _ in 0..30 {
        step(&mut w);
    }
    assert!((momentum(&w).x - half).abs() < 1e-9, "no push after the throttle is cut");
    assert!((half - 1500.0).abs() < 1e-6 * 1500.0, "half a second at full thrust, got {half}");
}

#[test]
fn a_deleted_node_turns_its_thruster_off_for_good() {
    let mut w = World::new();
    apply(&mut w, &[node(0.0, 100.0, 0.0, 10.0), node(1.0, 100.0, 0.0, 10.0), thruster(0, 1, 1000.0), throttle(0, 1.0)]);
    step(&mut w);
    let pushed = w.vel[1].x;
    assert!(pushed > 0.0);
    apply(&mut w, &[rec(CMD_DELETE_NODE, 0, 0, [0.0; 6])]);
    let vx = w.vel[1].x;
    step(&mut w);
    assert_eq!(w.vel[1].x, vx, "no push once the tail is gone");
    // A new node takes the freed slot; the thruster does not come back on it.
    let (r, _) = apply(&mut w, &[node(-5.0, 100.0, 0.0, 10.0)]);
    assert_eq!(r[0], 0, "the new node reuses slot 0");
    let vx = w.vel[1].x;
    step(&mut w);
    assert_eq!(w.vel[1].x, vx);
    assert_eq!(w.vel[0].x, 0.0);
    // The throttle can still be set (the thruster exists), and nothing is pushed.
    assert_eq!(apply(&mut w, &[throttle(0, 0.5)]).1, 0);
}

#[test]
fn two_nodes_on_top_of_each_other_give_no_force_and_no_bad_number() {
    let mut w = World::new();
    apply(&mut w, &[node(0.0, 100.0, 0.0, 10.0), node(0.0, 100.0, 0.0, 10.0), thruster(0, 1, 1000.0), throttle(0, 1.0)]);
    for _ in 0..10 {
        step(&mut w);
    }
    assert_eq!(w.vel[0].x, 0.0);
    assert_eq!(w.vel[1].x, 0.0);
    assert!(w.vel[0].y.is_finite() && w.vel[1].y.is_finite());
    assert!(w.events.is_empty());
}

#[test]
fn bad_records_are_rejected_and_the_rest_of_the_batch_applies() {
    let mut w = World::new();
    let nan = Real::NAN;
    let cmds = [
        node(0.0, 10.0, 0.0, 10.0),            // 0 ok: node 0
        node(1.0, 10.0, 0.0, 10.0),            // 1 ok: node 1
        thruster(0, 0, 100.0),                 // 2 rejected: same node twice
        thruster(0, 7, 100.0),                 // 3 rejected: no such node
        thruster(9, 1, 100.0),                 // 4 rejected: no such node
        thruster(0, 1, nan),                   // 5 rejected: not finite
        thruster(0, 1, Real::INFINITY),        // 6 rejected: not finite
        thruster(0, 1, -5.0),                  // 7 rejected: negative thrust
        thruster(0, 1, 100.0),                 // 8 ok: thruster 0
        throttle(0, 1.5),                      // 9 rejected: above 1
        throttle(0, -0.1),                     // 10 rejected: below 0
        throttle(0, nan),                      // 11 rejected: not a number
        throttle(1, 0.5),                      // 12 rejected: no such thruster
        throttle(0, 0.25),                     // 13 ok
        thruster(1, 0, 0.0),                   // 14 ok: zero thrust is allowed, thruster 1
        rec(CMD_DELETE_NODE, 1, 0, [0.0; 6]),  // 15 ok: node 1 goes
        thruster(0, 1, 100.0),                 // 16 rejected: node 1 is dead
    ];
    let (r, rejected) = apply(&mut w, &cmds);
    let u = REJECTED;
    // Record 12 asks for thruster 1, which does not exist yet, so it is rejected too.
    assert_eq!(r, vec![0, 1, u, u, u, u, u, u, 0, u, u, u, u, 1, 1, 1, u]);
    assert_eq!(rejected, 11);
    assert_eq!(w.thruster_count(), 2);
    assert_eq!(w.thr_throttle[0], 0.25);
    assert_eq!(w.thr_max[0], 100.0);
    assert_eq!(w.thr_max[1], 0.0);
}

#[test]
fn thrusters_are_in_the_hash_and_a_world_without_them_hashes_as_before() {
    let build = |max: f64, thr: f64| {
        let mut w = World::new();
        apply(&mut w, &[node(0.0, 10.0, 0.0, 10.0), node(1.0, 10.0, 0.0, 10.0), beam(0, 1, 1.0), thruster(0, 1, max), throttle(0, thr)]);
        w
    };
    let base = state_hash(&build(100.0, 0.5));
    assert_ne!(base, state_hash(&build(101.0, 0.5)), "the maximum is in the hash");
    assert_ne!(base, state_hash(&build(100.0, 0.6)), "the throttle is in the hash");
    assert_eq!(base, state_hash(&build(100.0, 0.5)));

    // A world with no thrusters mixes nothing extra into its hash. The golden hashes in ffi.rs
    // and the committed recordings (made before thrusters existed) are the real check of that.
}

#[test]
fn the_same_thrusting_run_twice_gives_the_same_hash() {
    let run = || {
        let mut w = pair(4000.0);
        for s in 0..120 {
            if s == 10 {
                apply(&mut w, &[throttle(0, 0.7)]);
            }
            if s == 70 {
                apply(&mut w, &[throttle(0, 0.2)]);
            }
            step(&mut w);
        }
        state_hash(&w)
    };
    assert_eq!(run(), run());
}
