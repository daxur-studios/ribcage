//! Sleep and wake (plan 005, phase 5.8), headless. A cube (segment 0) and a tetrahedron
//! (segment 1, its child) joined by one beam fall freely with a thruster on the cube; with
//! sleeping on, both sleep and become one body with their summed mass, a thrust cut wakes both,
//! and every switch keeps momentum and spin and never adds kinetic energy. The same records are
//! run in `test/wasm.test.ts`, held to the same golden.

use beam_core::command::{
    apply_commands, Command, CMD_ADD_BEAM, CMD_ADD_NODE, CMD_ADD_SEGMENT, CMD_ADD_THRUSTER, CMD_SET_NODE_SEGMENT, CMD_SET_SLEEPING,
    CMD_SET_THROTTLE,
};
use beam_core::event::{EVENT_SLEEP, EVENT_WAKE};
use beam_core::ffi::*;
use beam_core::real::{Real, Vec3};
use beam_core::sim::{state_hash, step};
use beam_core::sleep::{SwitchCheck, MIN_LIVE_STEPS, QUIET_STEPS, WAKE_NEIGHBOUR, WAKE_THRUST};
use beam_core::world::World;

/// State hash of the sleep scene after 200 steps. `test/wasm.test.ts` holds the same number.
pub const SLEEP_SCENE_200: u64 = 0x8fb3_e3c3_2250_148b;

const STEPS: u32 = 200;
/// The thrust is cut before this step.
const CUT: u32 = 150;

fn rec(kind: u32, a: u32, b: u32, f: [Real; 6]) -> Command {
    Command { kind, a, b, reserved: 0, f }
}

/// The body scene of `tests/ffi.rs` (cube of eight 10 kg nodes, tetrahedron of four 5 kg nodes,
/// one beam between them, a 300 N thruster on the cube) with sleeping switched on.
fn sleep_scene_setup() -> Vec<Command> {
    let beam = |a, b, rest| rec(CMD_ADD_BEAM, a, b, [rest, 1e-5, 20.0, 5.0, 10.0, 0.0]);
    let mut cmds = Vec::new();
    for k in 0..8u32 {
        let x = if k & 1 != 0 { 0.5 } else { -0.5 };
        let y = if k & 2 != 0 { 50.5 } else { 49.5 };
        let z = if k & 4 != 0 { 0.5 } else { -0.5 };
        cmds.push(rec(CMD_ADD_NODE, 0, 0, [x, y, z, 0.1, 0.1, 0.0]));
    }
    for [x, y, z] in [[2.0, 50.0, 0.0], [3.0, 50.0, 0.0], [2.5, 50.0, 0.8660254037844386], [2.5, 50.816496580927726, 0.28867513459481287]] {
        cmds.push(rec(CMD_ADD_NODE, 0, 0, [x, y, z, 0.2, 0.1, 0.0]));
    }
    for i in 0..8u32 {
        for bit in [1u32, 2, 4] {
            if i & bit == 0 {
                cmds.push(beam(i, i | bit, 1.0));
            }
        }
    }
    for (a, b) in [(8, 9), (8, 10), (8, 11), (9, 10), (9, 11), (10, 11)] {
        cmds.push(beam(a, b, 1.0));
    }
    cmds.push(beam(1, 8, 1.5));
    cmds.push(rec(CMD_ADD_SEGMENT, u32::MAX, 0, [0.0; 6]));
    cmds.push(rec(CMD_ADD_SEGMENT, 0, 0, [0.01, 1e-5, 0.0, 0.0, 0.0, 0.0]));
    for n in 0..12u32 {
        cmds.push(rec(CMD_SET_NODE_SEGMENT, n, if n < 8 { 0 } else { 1 }, [0.0; 6]));
    }
    cmds.push(rec(CMD_ADD_THRUSTER, 0, 1, [300.0, 0.0, 0.0, 0.0, 0.0, 0.0]));
    cmds.push(rec(CMD_SET_THROTTLE, 0, 0, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]));
    cmds.push(rec(CMD_SET_SLEEPING, 1, 0, [0.0; 6]));
    cmds
}

fn sleep_scene_at(s: u32) -> Vec<Command> {
    if s == CUT {
        vec![rec(CMD_SET_THROTTLE, 0, 0, [0.0; 6])]
    } else {
        Vec::new()
    }
}

/// `|a − b| ≤ tol`, with a message.
fn close(what: &str, a: Vec3, b: Vec3, tol: Real) {
    let d = (a - b).length();
    assert!(d <= tol, "{what}: {a:?} vs {b:?}, off by {d:e} (allowed {tol:e})");
}

/// The brief's switch checks: momentum and spin to rounding, energy never up on a sleep and
/// unchanged on a wake. Scales: 100 kg at the scene's speeds (under 100 m/s, 60 m from the origin).
fn check_switch(c: &SwitchCheck, step: u32) {
    let (p_scale, l_scale, e_scale) = (100.0 * 100.0, 100.0 * 100.0 * 60.0, 0.5 * 100.0 * 100.0 * 100.0);
    close(&format!("step {step}: momentum"), c.momentum_before, c.momentum_after, 1e-12 * p_scale);
    close(&format!("step {step}: spin"), c.spin_before, c.spin_after, 1e-12 * l_scale);
    let de = c.energy_after - c.energy_before;
    if c.kind == 2.0 {
        assert!(de <= 1e-12 * e_scale, "step {step}: a sleep added {de:e} J");
    } else {
        assert!(de.abs() <= 1e-12 * e_scale, "step {step}: a wake changed the energy by {de:e} J");
    }
}

/// Both segments sleep into one body with their summed mass; the cut wakes both; every switch
/// passes the checks; nothing switches faster than the minimum live time allows.
#[test]
fn two_segments_sharing_a_seam_sleep_into_one_body_and_wake_on_a_thrust_cut() {
    let mut w = World::new();
    let mut results = Vec::new();
    apply_commands(&mut w, &sleep_scene_setup(), &mut results);
    assert!(!results.contains(&u32::MAX), "a record was rejected: {results:?}");
    let mut slept_at = [u32::MAX; 2];
    let mut woke_at = [u32::MAX; 2];
    let mut switches = 0;
    let mut one_body_mass = None;
    for s in 0..STEPS {
        apply_commands(&mut w, &sleep_scene_at(s), &mut results);
        w.events.clear();
        step(&mut w);
        for c in &w.sleep.checks {
            check_switch(c, s);
            switches += 1;
        }
        for e in &w.events {
            if e.kind == EVENT_SLEEP {
                slept_at[e.index as usize] = s;
            }
            if e.kind == EVENT_WAKE {
                let why = if e.index == 0 { WAKE_THRUST } else { WAKE_NEIGHBOUR };
                assert_eq!(e.value, why as Real, "step {s}: segment {} woken by the cut or its neighbour", e.index);
                woke_at[e.index as usize] = s;
            }
        }
        if s == CUT - 1 {
            assert_eq!(w.body_count(), 1, "one body before the cut");
            let b = w.segment_body(0);
            assert_eq!(w.segment_body(1), b, "both segments ride it");
            assert!(w.sleep.asleep.iter().all(|&a| a == 1));
            one_body_mass = Some(w.body_mass[b as usize]);
        }
    }
    let m = one_body_mass.expect("checked before the cut");
    assert!((m - 100.0).abs() <= 1e-12 * 100.0, "summed mass: {m}");
    for k in 0..2 {
        assert!(slept_at[k] + 1 >= MIN_LIVE_STEPS.max(QUIET_STEPS), "segment {k} slept at step {}", slept_at[k]);
        assert_eq!(woke_at[k], CUT, "segment {k} woke on the cut");
    }
    assert_eq!(w.body_count(), 0, "awake after the cut");
    assert!(switches >= 2);
}

/// The scene through the boundary gives the direct world's bits twice over, and the golden the
/// WebAssembly build is held to.
#[test]
fn sleep_scene_is_deterministic_and_matches_the_golden() {
    let run = || {
        let mut w = World::new();
        let mut results = Vec::new();
        apply_commands(&mut w, &sleep_scene_setup(), &mut results);
        for s in 0..STEPS {
            apply_commands(&mut w, &sleep_scene_at(s), &mut results);
            step(&mut w);
        }
        state_hash(&w)
    };
    let direct = run();
    assert_eq!(run(), direct, "the same run twice");
    unsafe {
        let h = rc_host_new();
        let setup = sleep_scene_setup();
        let dst = rc_commands_reserve(h, setup.len() as u32);
        std::ptr::copy_nonoverlapping(setup.as_ptr(), dst, setup.len());
        assert_eq!(rc_apply_commands(h), 0, "no record rejected");
        for s in 0..STEPS {
            let more = sleep_scene_at(s);
            if !more.is_empty() {
                let dst = rc_commands_reserve(h, more.len() as u32);
                std::ptr::copy_nonoverlapping(more.as_ptr(), dst, more.len());
                assert_eq!(rc_apply_commands(h), 0);
            }
            rc_step(h);
        }
        assert_eq!(rc_state_hash(h), direct);
        assert_eq!(rc_sleep_segment_count(h), 2);
        rc_host_free(h);
    }
    assert_eq!(direct, SLEEP_SCENE_200, "golden: {direct:#018x}");
}
