//! Phase 6.2: body stations and the air pass.
//!
//! The station formula against hand values, a falling rod against its terminal speed, wind on a
//! resting rod, rigid against live, the stress estimate seeing the air load, bad records, the
//! hash (determinism, and a world with no element hashes as before) and dead nodes.

#[path = "support/truss.rs"]
mod truss;

use beam_core::aero::{apply_aero, station_force, Station, AERO_STATION, ETA, C_DC};
use beam_core::command::{
    apply_commands, Command, CMD_ADD_AERO_ELEMENT, CMD_ADD_AERO_NODE, CMD_ADD_SEGMENT, CMD_MAKE_RIGID,
    CMD_SET_AERO_FRAME, CMD_SET_AIR, CMD_SET_NODE_SEGMENT,
};
use beam_core::real::{Real, Vec3};
use beam_core::sim::{physics_hash, state_hash, step};
use beam_core::world::{BeamSpec, World};
use truss::{build, TrussSpec};

fn rec(kind: u32, a: u32, b: u32, f: [Real; 6]) -> Command {
    Command { kind, a, b, reserved: 0, f }
}

fn rel(a: Real, b: Real) -> Real {
    ((a - b) / b).abs()
}

fn close(a: Vec3, b: Vec3, tol: Real) -> bool {
    (a - b).length() <= tol
}

const STATION: Station = Station { a_front: 0.8, c_axial: 0.4, a_base: 0.5, a_plan: 2.0, c_base: 0.3 };
const X: Vec3 = Vec3::new(1.0, 0.0, 0.0);

#[test]
fn station_axial_flow() {
    // Flying nose first along +x: the air comes at -x, ua < 0, drag along -x.
    let (rho, v) = (1.2, 50.0);
    let u = Vec3::new(-v, 0.0, 0.0);
    let q = 0.5 * rho * v * v;
    let f = station_force(&STATION, X, u, rho, false);
    assert!(close(f, Vec3::new(-q * 0.8 * 0.4, 0.0, 0.0), 1e-9 * q), "{f:?}");
    // The base pressure adds `A_b C_base` while no thruster fires on it.
    let on = station_force(&STATION, X, u, rho, true);
    assert!(close(on, Vec3::new(-q * (0.8 * 0.4 + 0.5 * 0.3), 0.0, 0.0), 1e-9 * q), "{on:?}");
    // Backwards flight pushes the other way with the same size.
    let back = station_force(&STATION, X, -u, rho, false);
    assert!(close(back, -f, 1e-9 * q));
}

#[test]
fn station_crossflow_at_90_degrees() {
    let (rho, v) = (1.2, 40.0);
    let q = 0.5 * rho * v * v;
    // Wind blows along +z across the x axis: the body feels a force along +z.
    let f = station_force(&STATION, X, Vec3::new(0.0, 0.0, v), rho, true);
    // sin 2a = 0 at 90 degrees, sin^2 a = 1, and no axial part.
    let want = q * ETA * C_DC * STATION.a_plan;
    assert!(close(f, Vec3::new(0.0, 0.0, want), 1e-9 * want), "{f:?} against {want}");
}

#[test]
fn station_slender_body_term_at_10_and_30_degrees() {
    let (rho, v) = (1.2, 60.0);
    let q = 0.5 * rho * v * v;
    for deg in [10.0 as Real, 30.0] {
        let a = deg.to_radians();
        // Nose into the wind at angle `a`: u = v (-cos a, sin a, 0) in the (x, y) plane.
        let u = Vec3::new(-v * a.cos(), v * a.sin(), 0.0);
        let f = station_force(&STATION, X, u, rho, false);
        let axial = -0.5 * rho * (v * a.cos()).powi(2) * 0.8 * 0.4;
        let cn = 0.5 * (2.0 * a).sin() * (0.5 * a).cos() + ETA * C_DC * 2.0 * a.sin().powi(2);
        let want = Vec3::new(axial, q * cn, 0.0);
        assert!(close(f, want, 1e-7 * want.length()), "{deg} deg: {f:?} against {want:?}");
    }
}

#[test]
fn station_gives_nothing_without_airspeed_or_air() {
    assert_eq!(station_force(&STATION, X, Vec3::ZERO, 1.2, true), Vec3::ZERO);
    assert_eq!(station_force(&STATION, X, Vec3::new(-10.0, 0.0, 0.0), 0.0, true), Vec3::ZERO);
}

/// A vertical rod of two nodes (`m` kg in all), a station on both nodes with tail below nose.
fn rod(m: Real, coef: [Real; 6]) -> World {
    let mut w = World::new();
    let a = w.add_node(Vec3::new(0.0, 1000.0, 0.0), 2.0 / m, 0.05).index;
    let b = w.add_node(Vec3::new(0.0, 1001.0, 0.0), 2.0 / m, 0.05).index;
    w.add_beam(a, b, BeamSpec { rest: 1.0, compliance: 1e-8, damping: 50.0, yield_strain: 0.5, break_strain: 1.0 });
    let mut r = Vec::new();
    apply_commands(
        &mut w,
        &[
            rec(CMD_SET_AIR, 0, 0, [1.225, 0.0, 0.0, 0.0, 0.0, 340.0]),
            rec(CMD_ADD_AERO_ELEMENT, AERO_STATION, 0, coef),
            rec(CMD_ADD_AERO_NODE, 0, a, [0.5, 0.0, 0.0, 0.0, 0.0, 0.0]),
            rec(CMD_ADD_AERO_NODE, 0, b, [0.5, 0.0, 0.0, 0.0, 0.0, 0.0]),
            rec(CMD_SET_AERO_FRAME, 0, a, [b as Real, u32::MAX as Real, u32::MAX as Real, 0.0, 0.0, 0.0]),
        ],
        &mut r,
    );
    assert_eq!(r[0], 1);
    assert_eq!(r[1], 0, "the first element has index 0");
    assert!(r[2..].iter().all(|&x| x == 1), "{r:?}");
    w
}

#[test]
fn a_falling_rod_reaches_its_terminal_speed() {
    let m = 100.0;
    let (af, ca) = (1.0, 2.0);
    let mut w = rod(m, [af, ca, 0.0, 0.0, 0.0, 0.0]);
    for _ in 0..1800 {
        step(&mut w);
    }
    let v = -(w.vel[0].y + w.vel[1].y) * 0.5;
    let want = (2.0 * m * 9.81 / (1.225 * af * ca)).sqrt();
    println!("terminal speed {v:.4} against {want:.4}");
    assert!(rel(v, want) < 0.005, "{v} against {want}");
}

#[test]
fn wind_on_a_resting_rod_pushes_by_the_formula() {
    let mut w = rod(100.0, [1.0, 2.0, 0.0, 3.0, 0.0, 0.0]);
    let mut r = Vec::new();
    apply_commands(&mut w, &[rec(CMD_SET_AIR, 0, 0, [1.225, 0.0, 20.0, 0.0, 0.0, 340.0])], &mut r);
    w.vel[0] = Vec3::ZERO;
    w.vel[1] = Vec3::ZERO;
    apply_aero(&mut w);
    let total = w.env_force[0] + w.env_force[1];
    let st = Station { a_front: 1.0, c_axial: 2.0, a_base: 0.0, a_plan: 3.0, c_base: 0.0 };
    let want = station_force(&st, Vec3::new(0.0, 1.0, 0.0), Vec3::new(20.0, 0.0, 0.0), 1.225, false);
    assert!(want.x > 0.0);
    assert!(close(total, want, 1e-9 * want.length()), "{total:?} against {want:?}");
    // The air force went through the environment door: it is in `force` and `env_force` alike.
    assert!(close(w.force[0] + w.force[1], total, 1e-12));
    // Split by weight: equal halves.
    assert!(close(w.env_force[0], w.env_force[1], 1e-12));
}

#[test]
fn the_base_pressure_acts_on_a_base_in_axial_flow() {
    let mut w = rod(100.0, [0.0, 0.0, 1.0, 0.0, 1.0, 0.0]);
    w.vel[0] = Vec3::new(0.0, 30.0, 0.0);
    w.vel[1] = Vec3::new(0.0, 30.0, 0.0);
    apply_aero(&mut w);
    let with_base = w.env_force[0] + w.env_force[1];
    assert!(with_base.y < 0.0, "{with_base:?}");
    let q = 0.5 * 1.225 * 900.0;
    assert!(close(with_base, Vec3::new(0.0, -q, 0.0), 1e-9 * q));
}

#[test]
fn rigid_and_live_give_the_same_force() {
    // A cube with a station on its four top nodes; the rod's frame runs across the top.
    let build_cube = |rigid: bool| -> World {
        let mut w = World::new();
        for k in 0..8u32 {
            let p = Vec3::new((k & 1) as Real, 500.0 + ((k >> 1) & 1) as Real, ((k >> 2) & 1) as Real);
            w.add_node(p, 1.0 / 12.5, 0.1);
        }
        for i in 0..8u32 {
            for j in (i + 1)..8 {
                let d = (w.pos[i as usize] - w.pos[j as usize]).length();
                w.add_beam(i, j, BeamSpec { rest: d, compliance: 1e-8, damping: 50.0, yield_strain: 0.05, break_strain: 0.1 });
            }
        }
        let mut cmds = vec![rec(CMD_ADD_SEGMENT, u32::MAX, 0, [0.0; 6])];
        for n in 0..8u32 {
            cmds.push(rec(CMD_SET_NODE_SEGMENT, n, 0, [0.0; 6]));
        }
        if rigid {
            cmds.push(rec(CMD_MAKE_RIGID, 0, 0, [0.0; 6]));
        }
        cmds.push(rec(CMD_SET_AIR, 0, 0, [1.225, 0.0, 5.0, 0.0, 0.0, 340.0]));
        cmds.push(rec(CMD_ADD_AERO_ELEMENT, AERO_STATION, 0, [0.5, 0.8, 0.0, 1.0, 0.0, 0.0]));
        for n in [2u32, 3, 6, 7] {
            cmds.push(rec(CMD_ADD_AERO_NODE, 0, n, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]));
        }
        cmds.push(rec(CMD_SET_AERO_FRAME, 0, 2, [3.0, u32::MAX as Real, u32::MAX as Real, 0.0, 0.0, 0.0]));
        let mut r = Vec::new();
        apply_commands(&mut w, &cmds, &mut r);
        assert!(!r.contains(&u32::MAX), "{r:?}");
        // Move the cube along +z at 30 m/s for a few steps; then read the pass on the step's start.
        for n in 0..8 {
            w.vel[n] = Vec3::new(0.0, 0.0, 30.0);
        }
        for _ in 0..3 {
            step(&mut w);
        }
        w
    };
    let (mut live, mut rigid) = (build_cube(false), build_cube(true));
    // Equalise: both worlds' nodes share the cube's shape; the pass reads pos and vel only.
    for n in 0..8 {
        rigid.pos[n] = live.pos[n];
        rigid.vel[n] = live.vel[n];
    }
    apply_aero(&mut live);
    apply_aero(&mut rigid);
    let sum = |w: &World| (0..8).fold(Vec3::ZERO, |s, n| s + w.env_force[n]);
    let (a, b) = (sum(&live), sum(&rigid));
    assert!(a.length() > 0.0);
    assert!(close(a, b, 1e-9 * a.length()), "{a:?} against {b:?}");
}

#[test]
fn the_estimate_sees_the_air_load() {
    let spec = TrussSpec { damping: 50.0, ..TrussSpec::standard() };
    let mut w = build(&spec);
    // Cut at the middle: frames 1..=bays one segment each, a chain.
    for i in 1..=spec.bays {
        let parent = if i == 1 { u32::MAX } else { i - 2 };
        let s = w.add_segment(parent, 0, 0.0, 0.0).unwrap();
        for j in 0..4 {
            assert!(w.assign_node_segment(4 * i + j, s));
        }
    }
    let tips = spec.tip_nodes();
    let mut cmds = vec![
        rec(CMD_SET_AIR, 0, 0, [1.225, 0.0, 40.0, 0.0, 0.0, 340.0]),
        rec(CMD_ADD_AERO_ELEMENT, AERO_STATION, 0, [0.0, 0.0, 0.0, 6.0, 0.0, 0.0]),
    ];
    for t in tips {
        cmds.push(rec(CMD_ADD_AERO_NODE, 0, t as u32, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]));
    }
    cmds.push(rec(CMD_SET_AERO_FRAME, 0, tips[0] as u32, [tips[2] as Real, u32::MAX as Real, u32::MAX as Real, 0.0, 0.0, 0.0]));
    let mut r = Vec::new();
    apply_commands(&mut w, &cmds, &mut r);
    assert!(!r.contains(&u32::MAX), "{r:?}");
    w.est.on = true;
    // Settle, then compare one step's seam force with the weight plus the air load.
    for _ in 0..900 {
        step(&mut w);
    }
    // The settled truss is still: the air force is the formula on the tip frame's positions.
    let st = Station { a_front: 0.0, c_axial: 0.0, a_base: 0.0, a_plan: 6.0, c_base: 0.0 };
    let d = w.pos[tips[2]] - w.pos[tips[0]];
    let axis = d * (1.0 / d.length());
    let air = station_force(&st, axis, Vec3::new(40.0, 0.0, 0.0), 1.225, false);
    assert!(air.x > 1.0, "{air:?}");
    let seam = w.est.seam_force.len() - 1;
    let f = w.est.seam_force[seam];
    // The last seam holds the tip frame's own weight; the seam carries the air load away: -air.
    let weight = Vec3::new(0.0, 4.0 * spec.node_mass * 9.81, 0.0);
    let want = weight - air;
    let tol = 0.02 * want.length();
    println!("seam force {f:?}, expected {want:?}");
    assert!(close(f, want, tol), "{f:?} against {want:?}");
}

#[test]
fn bad_records_are_rejected_and_the_rest_apply() {
    let mut w = World::new();
    let a = w.add_node(Vec3::ZERO, 1.0, 0.1).index;
    let b = w.add_node(Vec3::new(0.0, 1.0, 0.0), 1.0, 0.1).index;
    let nan = Real::NAN;
    let none = u32::MAX as Real;
    let cmds = [
        rec(CMD_ADD_AERO_ELEMENT, 9, 0, [1.0; 6]),               // unknown kind
        rec(CMD_ADD_AERO_ELEMENT, AERO_STATION, 0, [nan, 0.0, 0.0, 0.0, 0.0, 0.0]), // not finite
        rec(CMD_ADD_AERO_ELEMENT, AERO_STATION, 0, [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0]), // negative
        rec(CMD_ADD_AERO_ELEMENT, AERO_STATION, 0, [1.0, 1.0, 0.0, 0.0, 0.0, 0.0]), // good: 0
        rec(CMD_ADD_AERO_NODE, 0, a, [0.0; 6]),                  // weight zero
        rec(CMD_ADD_AERO_NODE, 0, 99, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]), // no such node
        rec(CMD_ADD_AERO_NODE, 5, a, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]), // no such element
        rec(CMD_ADD_AERO_NODE, 0, a, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]), // good
        rec(CMD_SET_AERO_FRAME, 0, a, [a as Real, none, none, 0.0, 0.0, 0.0]), // tail = nose
        rec(CMD_SET_AERO_FRAME, 0, a, [none, none, none, 0.0, 0.0, 0.0]),      // no nose
        rec(CMD_SET_AERO_FRAME, 0, a, [0.5, none, none, 0.0, 0.0, 0.0]),       // not whole
        rec(CMD_SET_AERO_FRAME, 0, a, [b as Real, none, none, 0.0, 0.0, 0.0]), // good
    ];
    let mut r = Vec::new();
    apply_commands(&mut w, &cmds, &mut r);
    let ok = [false, false, false, true, false, false, false, true, false, false, false, true];
    for (i, &g) in ok.iter().enumerate() {
        assert_eq!(r[i] != u32::MAX, g, "record {i}: {r:?}");
    }
    assert_eq!(r[3], 0);
    assert_eq!(w.aero_element_count(), 1);
}

#[test]
fn a_dead_node_makes_the_element_inert() {
    let mut w = rod(100.0, [1.0, 2.0, 0.0, 0.0, 0.0, 0.0]);
    w.vel[0] = Vec3::new(0.0, -30.0, 0.0);
    w.vel[1] = Vec3::new(0.0, -30.0, 0.0);
    apply_aero(&mut w);
    assert!(w.env_force[0].length() > 0.0);
    let mut dead = rod(100.0, [1.0, 2.0, 0.0, 0.0, 0.0, 0.0]);
    dead.vel[0] = Vec3::new(0.0, -30.0, 0.0);
    dead.vel[1] = Vec3::new(0.0, -30.0, 0.0);
    dead.delete_node(1);
    apply_aero(&mut dead);
    assert_eq!(dead.env_force[0], Vec3::ZERO);
}

#[test]
fn hash_is_deterministic_and_absent_without_elements() {
    let run = || {
        let mut w = rod(100.0, [1.0, 2.0, 0.5, 1.0, 0.3, 0.0]);
        for _ in 0..120 {
            step(&mut w);
        }
        (state_hash(&w), physics_hash(&w))
    };
    let (a, b) = (run(), run());
    println!("rod hash {:#018x}", a.0);
    assert_eq!(a, b);
    // The element changes the hash; a world with no element hashes the same with or without the
    // empty element arrays.
    let make = || {
        let mut w = World::new();
        w.add_node(Vec3::new(0.0, 5.0, 0.0), 1.0, 0.1);
        w
    };
    let plain = make();
    let h0 = state_hash(&plain);
    let mut with = make();
    let mut r = Vec::new();
    apply_commands(&mut with, &[rec(CMD_ADD_AERO_ELEMENT, AERO_STATION, 0, [1.0, 1.0, 0.0, 0.0, 0.0, 0.0])], &mut r);
    assert_ne!(state_hash(&with), h0);
    assert_eq!(state_hash(&plain), h0);
}
