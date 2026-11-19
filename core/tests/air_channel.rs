//! Phase 6.1: the air settings, the environment-force channel and the never-sleep flag.
//!
//! Two small lattice cubes (eight 12.5 kg nodes, all 28 pairs joined) rest on flat ground, one
//! segment each, the second a child of the first. With sleeping on they sleep. A steady push of
//! twice a node's weight is then added on every node, once through the host's door (`CMD_ADD_FORCE`)
//! and once through the environment's (`World::add_env_force`): only the first trips the force-jump
//! wake.

use beam_core::aero::{DEFAULT_RHO0, DEFAULT_SCALE_HEIGHT};
use beam_core::command::{apply_commands, Command, CMD_ADD_FORCE, CMD_SET_AIR, CMD_SET_SLEEPING};
use beam_core::event::EVENT_WAKE;
use beam_core::real::{Real, Vec3};
use beam_core::segment::{SEG_NEVER_LIVE, SEG_NEVER_SLEEP};
use beam_core::sim::{gravity_at, state_hash, step};
use beam_core::sleep::WAKE_FORCE;
use beam_core::terrain::Terrain;
use beam_core::world::{BeamSpec, World};

const NODE_MASS: Real = 12.5;
const RADIUS: Real = 0.1;

fn rec(kind: u32, a: u32, b: u32, f: [Real; 6]) -> Command {
    Command { kind, a, b, reserved: 0, f }
}

/// Two cubes of side 1 m on flat ground, x = 0 and x = 3; segment 0 is the first, segment 1 (a
/// child of 0) the second, with `flags1` on it. Sleeping switched on.
fn scene(flags1: u32) -> World {
    let mut w = World::new();
    w.terrain = Some(Terrain::flat(2, 2, -100.0, -100.0, 200.0, 0.5));
    let mut cmds = Vec::new();
    for (c, x0) in [0.0 as Real, 3.0].into_iter().enumerate() {
        let base = w.node_count();
        for k in 0..8u32 {
            let p = Vec3::new(x0 + (k & 1) as Real, RADIUS + ((k >> 1) & 1) as Real, ((k >> 2) & 1) as Real);
            let n = w.add_node(p, 1.0 / NODE_MASS, RADIUS).index;
            let _ = (c, base, n);
        }
        for i in 0..8u32 {
            for j in (i + 1)..8 {
                let d = (w.pos[(base + i) as usize] - w.pos[(base + j) as usize]).length();
                w.add_beam(
                    base + i,
                    base + j,
                    BeamSpec { rest: d, compliance: 1e-8, damping: 50.0, yield_strain: 0.05, break_strain: 0.1 },
                );
            }
        }
    }
    let mut r = Vec::new();
    let setup = [
        rec(beam_core::command::CMD_ADD_SEGMENT, u32::MAX, 0, [0.0; 6]),
        rec(beam_core::command::CMD_ADD_SEGMENT, 0, flags1, [0.01, 1e-5, 0.0, 0.0, 0.0, 0.0]),
    ];
    cmds.extend(setup);
    for n in 0..16u32 {
        cmds.push(rec(beam_core::command::CMD_SET_NODE_SEGMENT, n, n / 8, [0.0; 6]));
    }
    cmds.push(rec(CMD_SET_SLEEPING, 1, 0, [0.0; 6]));
    apply_commands(&mut w, &cmds, &mut r);
    assert!(!r.contains(&u32::MAX), "a setup record was rejected: {r:?}");
    w
}

fn run(w: &mut World, steps: u32) {
    for _ in 0..steps {
        step(w);
    }
}

fn rel(a: Real, b: Real) -> Real {
    ((a - b) / b).abs()
}

#[test]
fn own_exp_matches_the_library_to_1e_14_relative() {
    use beam_core::maths::exp;
    let mut worst: Real = 0.0;
    // Dense over the range density uses (-30 to 10) and sparser out to the saturation points.
    for i in 0..400_000u32 {
        let x = -30.0 + i as Real * 0.000_100_007_3;
        worst = worst.max(rel(exp(x), x.exp()));
    }
    for i in 0..200_000u32 {
        let x = -700.0 + i as Real * 0.007_000_131;
        worst = worst.max(rel(exp(x), x.exp()));
    }
    assert!(worst < 1e-14, "worst relative error {worst:e}");
    assert_eq!(exp(0.0), 1.0);
    assert!(rel(exp(1.0), std::f64::consts::E) < 1e-15);
    assert!(exp(710.0).is_infinite() && exp(-709.0) == 0.0 && exp(Real::NAN).is_nan());
}

#[test]
fn density_at_height_follows_the_formula_and_the_command_sets_it() {
    let mut w = World::new();
    assert_eq!(w.air.density(0.0), DEFAULT_RHO0, "default sea level");
    let h0 = state_hash(&w);
    let mut r = Vec::new();
    // rho0 1.2, H 8000, wind (3, 0, -2), sound 300.
    apply_commands(&mut w, &[rec(CMD_SET_AIR, 0, 0, [1.2, 8000.0, 3.0, 0.0, -2.0, 300.0])], &mut r);
    assert_eq!(r, vec![1]);
    assert!(w.air.ever && w.air.wind == Vec3::new(3.0, 0.0, -2.0) && w.air.sound_speed == 300.0);
    assert_eq!(w.air.density(0.0), 1.2);
    assert!(rel(w.air.density(8000.0), 1.2 * (-1.0 as Real).exp()) < 1e-14);
    assert!(rel(w.air.density(8500.0), 1.2 * (-8500.0 / 8000.0 as Real).exp()) < 1e-14);
    assert_ne!(state_hash(&w), h0, "the air settings are in the hash once set");
    // Defaults at 0 and 8500 m.
    let mut d = World::new();
    apply_commands(&mut d, &[rec(CMD_SET_AIR, 0, 0, [DEFAULT_RHO0, DEFAULT_SCALE_HEIGHT, 0.0, 0.0, 0.0, 340.0])], &mut r);
    assert_eq!(d.air.density(0.0), DEFAULT_RHO0);
    assert!(rel(d.air.density(8500.0), DEFAULT_RHO0 * (-1.0 as Real).exp()) < 1e-14);
    // Scale height zero: constant density.
    apply_commands(&mut d, &[rec(CMD_SET_AIR, 0, 0, [1.0, 0.0, 0.0, 0.0, 0.0, 340.0])], &mut r);
    assert_eq!(d.air.density(12_345.0), 1.0);
    // Bad records are rejected and change nothing.
    let before = d.air;
    let bad = [
        [-1.0, 8500.0, 0.0, 0.0, 0.0, 340.0],
        [1.2, -1.0, 0.0, 0.0, 0.0, 340.0],
        [1.2, 8500.0, Real::NAN, 0.0, 0.0, 340.0],
        [1.2, 8500.0, 0.0, 0.0, 0.0, 0.0],
        [1.2, 8500.0, 0.0, Real::INFINITY, 0.0, 340.0],
    ];
    for f in bad {
        apply_commands(&mut d, &[rec(CMD_SET_AIR, 0, 0, f)], &mut r);
        assert_eq!(r, vec![u32::MAX], "{f:?}");
    }
    assert_eq!(d.air, before);
}

#[test]
fn a_world_with_no_air_command_hashes_as_before() {
    // The same scene twice, one with a never-sleep flag nobody set: identical bits. (The goldens
    // in the other tests and in `test/` replay to their old hashes; this is the direct check.)
    let mut a = scene(0);
    let mut b = scene(0);
    run(&mut a, 120);
    run(&mut b, 120);
    assert_eq!(state_hash(&a), state_hash(&b));
    assert!(!a.air.ever);
}

/// Puts the scene to sleep and checks it did.
fn sleeping_scene() -> World {
    let mut w = scene(0);
    run(&mut w, 300);
    assert!(w.sleep.asleep.iter().all(|&a| a == 1), "both cubes asleep: {:?}", w.sleep.asleep);
    w
}

/// Steps with the force applied on every node before each step, and returns the segments woken
/// by the force-jump trigger.
fn push_and_count(w: &mut World, through_env: bool, steps: u32) -> Vec<u32> {
    let mut woken = Vec::new();
    for _ in 0..steps {
        for i in 0..w.node_count() {
            let weight = gravity_at(w.pos[i as usize]).length() * NODE_MASS;
            let f = Vec3::new(2.0 * weight, 0.0, 0.0);
            if through_env {
                w.add_env_force(i, f);
            } else {
                let mut r = Vec::new();
                apply_commands(w, &[rec(CMD_ADD_FORCE, i, 0, [f.x, f.y, f.z, 0.0, 0.0, 0.0])], &mut r);
            }
        }
        step(w);
        for e in &w.events {
            if e.kind == EVENT_WAKE && e.value == WAKE_FORCE as Real {
                woken.push(e.index);
            }
        }
    }
    woken
}

#[test]
fn env_force_does_not_trip_the_force_jump_wake_but_a_host_force_does() {
    let mut env = sleeping_scene();
    let woken = push_and_count(&mut env, true, 10);
    assert!(woken.is_empty(), "air-like force tripped the jump trigger: {woken:?}");
    // The force still reached the world: the door adds to `force`, and both are cleared at the end of the step.
    assert!(env.force.iter().all(|f| *f == Vec3::ZERO) && env.env_force.iter().all(|f| *f == Vec3::ZERO));

    let mut host = sleeping_scene();
    let woken = push_and_count(&mut host, false, 10);
    assert!(!woken.is_empty(), "the same load through CMD_ADD_FORCE must wake the cubes");
}

#[test]
fn env_force_reaches_the_solver_like_any_force() {
    let mut w = World::new();
    let n = w.add_node(Vec3::new(0.0, 1000.0, 0.0), 1.0, 0.1).index;
    w.add_env_force(n, Vec3::new(10.0, 0.0, 0.0)); // 1 kg: 10 m/s² for one step
    assert_eq!(w.env_force[n as usize], Vec3::new(10.0, 0.0, 0.0));
    step(&mut w);
    assert!(w.vel[n as usize].x > 0.15 && w.vel[n as usize].x < 0.18, "vx {}", w.vel[n as usize].x);
    assert_eq!(w.env_force[n as usize], Vec3::ZERO, "cleared with force");
    // A node added into a freed slot starts with no environment force either.
    w.add_env_force(n, Vec3::new(1.0, 0.0, 0.0));
    w.delete_node(n);
    let m = w.add_node(Vec3::ZERO, 1.0, 0.1).index;
    assert_eq!(m, n);
    assert_eq!(w.env_force[m as usize], Vec3::ZERO);
}

#[test]
fn a_never_sleep_segment_stays_live_through_a_rest_that_sleeps_the_rest() {
    let mut plain = scene(0);
    let mut flagged = scene(SEG_NEVER_SLEEP);
    assert_eq!(flagged.seg_flags, vec![0, SEG_NEVER_SLEEP]);
    for s in 0..60 * 60 {
        step(&mut plain);
        step(&mut flagged);
        if s > 60 {
            assert_eq!(flagged.sleep.asleep[1], 0, "step {s}: the never-sleep segment slept");
        }
    }
    assert_eq!(plain.sleep.asleep, vec![1, 1], "without the flag both sleep");
    assert_eq!(flagged.sleep.asleep[0], 1, "its neighbour still sleeps: {:?}", flagged.sleep.asleep);
    assert_eq!(flagged.sleep.asleep[1], 0);
    assert_eq!(flagged.seg_body[1], beam_core::body::NO_BODY);
    // Never-live is a different flag, and unknown bits are still refused.
    assert_ne!(SEG_NEVER_LIVE, SEG_NEVER_SLEEP);
    assert!(flagged.add_segment(0, 4, 1.0, 1.0).is_none());
    assert!(flagged.add_segment(0, SEG_NEVER_LIVE | SEG_NEVER_SLEEP, 1.0, 1.0).is_some());
}
