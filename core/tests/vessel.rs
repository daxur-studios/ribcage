//! Splitting into vessels (plan 005, phase 5.10), headless. The sleep scene's cube (segment 0)
//! and tetrahedron (segment 1), joined by one beam, are cut apart live and asleep; a stack of
//! three rigid cubes is cut between the second and third. Each cut gives two vessels with the
//! summed mass, the lowest node keeps vessel 0, momentum and spin are kept, and an asleep cut
//! divides the body without adding energy. The cut scene is run in `test/wasm.test.ts` against
//! the same golden.

use beam_core::body::NO_BODY;
use beam_core::command::{
    apply_commands, Command, CMD_ADD_BEAM, CMD_ADD_NODE, CMD_ADD_SEGMENT, CMD_ADD_THRUSTER, CMD_BREAK_BEAM, CMD_MAKE_RIGID,
    CMD_SET_NODE_SEGMENT, CMD_SET_SLEEPING, CMD_SET_THROTTLE,
};
use beam_core::event::{EVENT_SPLIT, EVENT_VESSEL_SEGMENT};
use beam_core::real::{Real, Vec3};
use beam_core::sim::{state_hash, step};
use beam_core::world::World;

/// State hash of the cut scene (sleeping on, the seam beam cut before step 120) after 200 steps.
/// `test/wasm.test.ts` holds the same number.
pub const VESSEL_SCENE_200: u64 = 0x2235_baa7_f88d_26ad;

const STEPS: u32 = 200;
/// The seam beam is cut before this step: both segments have slept by then (phase 5.8 test).
const CUT_AT: u32 = 120;
/// The seam beam between the cube and the tetrahedron: 12 cube beams and 6 tetrahedron beams first.
const SEAM_BEAM: u32 = 18;

fn rec(kind: u32, a: u32, b: u32, f: [Real; 6]) -> Command {
    Command { kind, a, b, reserved: 0, f }
}

fn beam(a: u32, b: u32, rest: Real) -> Command {
    rec(CMD_ADD_BEAM, a, b, [rest, 1e-5, 20.0, 5.0, 10.0, 0.0])
}

/// The sleep scene of `tests/sleep.rs`: a cube of eight 10 kg nodes and a tetrahedron of four
/// 5 kg nodes, one beam between them, a 300 N thruster on the cube; sleeping as asked.
fn scene_setup(sleeping: bool) -> Vec<Command> {
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
    if sleeping {
        cmds.push(rec(CMD_SET_SLEEPING, 1, 0, [0.0; 6]));
    }
    cmds
}

fn apply(w: &mut World, cmds: &[Command]) {
    let mut results = Vec::new();
    apply_commands(w, cmds, &mut results);
    assert!(!results.contains(&u32::MAX), "a record was rejected: {results:?}");
}

/// Momentum and spin about the origin of every movable node (loose nodes and body riders alike:
/// the copy-back leaves each rider with its body's velocity at its position).
fn totals(w: &World) -> (Vec3, Vec3) {
    let (mut p, mut l) = (Vec3::ZERO, Vec3::ZERO);
    for i in 0..w.pos.len() {
        if w.inv_mass[i] == 0.0 {
            continue;
        }
        let q = w.vel[i] * (1.0 / w.inv_mass[i]);
        p = p + q;
        l = l + w.pos[i].cross(q);
    }
    (p, l)
}

fn close(what: &str, a: Vec3, b: Vec3, tol: Real) {
    let d = (a - b).length();
    assert!(d <= tol, "{what}: {a:?} vs {b:?}, off by {d:e} (allowed {tol:e})");
}

/// The split events of the last step: (new vessel, parent) and the (segment, vessel) records.
fn split_events(w: &World) -> (Vec<(u32, u32)>, Vec<(u32, u32)>) {
    let (mut splits, mut segs) = (Vec::new(), Vec::new());
    for e in &w.events {
        if e.kind == EVENT_SPLIT {
            splits.push((e.index, e.value as u32));
        }
        if e.kind == EVENT_VESSEL_SEGMENT {
            segs.push((e.index, e.value as u32));
        }
    }
    (splits, segs)
}

/// Runs the scene, cutting the seam beam before `CUT_AT`; returns the world after `steps`.
fn run_cut(sleeping: bool, steps: u32, mut at_cut: impl FnMut(&World)) -> World {
    let mut w = World::new();
    apply(&mut w, &scene_setup(sleeping));
    for s in 0..steps {
        if s == CUT_AT {
            apply(&mut w, &[rec(CMD_BREAK_BEAM, SEAM_BEAM, 0, [0.0; 6])]);
        }
        w.events.clear();
        step(&mut w);
        if s == CUT_AT {
            at_cut(&w);
        }
    }
    w
}

/// Common checks right after the cut: two vessels, the cube keeps vessel 0, masses add exactly.
fn check_two_vessels(w: &World) {
    assert_eq!(w.vessel_ids(), 2, "two vessels");
    for n in 0..12u32 {
        assert_eq!(w.node_vessel(n), if n < 8 { 0 } else { 1 }, "node {n}");
    }
    assert_eq!((w.segment_vessel(0), w.segment_vessel(1)), (0, 1));
    let (m0, m1) = (w.vessel_mass(0), w.vessel_mass(1));
    assert_eq!(m0 + m1, 100.0, "masses add to exactly 100 kg ({m0} + {m1})");
    assert!((m0 - 80.0).abs() < 1e-9 && (m1 - 20.0).abs() < 1e-9, "{m0} and {m1}");
    let (splits, segs) = split_events(w);
    assert_eq!(splits, vec![(1, 0)], "one split: vessel 1 out of vessel 0");
    assert_eq!(segs, vec![(1, 1)], "segment 1 listed in vessel 1");
    let (p, l) = totals(w);
    let (p0, l0) = w.vessel_momentum(0);
    let (p1, l1) = w.vessel_momentum(1);
    close("vessel momenta add up", p0 + p1, p, 1e-12 * 100.0 * 100.0);
    close("vessel spins add up", l0 + l1, l, 1e-12 * 100.0 * 100.0 * 60.0);
}

/// Cut live (sleeping off): two vessels, no body anywhere, momentum across the cut step changes
/// only by gravity and thrust.
#[test]
fn a_live_cut_gives_two_vessels_with_the_summed_mass() {
    let mut before = (Vec3::ZERO, Vec3::ZERO);
    let mut w = World::new();
    apply(&mut w, &scene_setup(false));
    for s in 0..CUT_AT {
        step(&mut w);
        if s + 1 == CUT_AT {
            before = totals(&w);
        }
    }
    assert_eq!(w.vessel_ids(), 1, "one vessel before any cut");
    assert_eq!(w.segment_vessel(1), 0);
    apply(&mut w, &[rec(CMD_BREAK_BEAM, SEAM_BEAM, 0, [0.0; 6])]);
    w.events.clear();
    step(&mut w);
    check_two_vessels(&w);
    assert_eq!(w.body_count(), 0, "nothing rigid when live");
    // The cut itself adds no impulse: a step's change in momentum is gravity plus the thrust.
    let (p, _) = totals(&w);
    let dp = p - before.0;
    assert!(dp.length() < (100.0 * 9.81 + 300.0) / 60.0 + 1e-9, "momentum jumped by {dp:?} over the cut step");
}

/// Cut asleep: the one body is divided into the cube's and the tetrahedron's, the division keeps
/// momentum and spin to 1e-12 and adds no energy, and both shares are woken.
#[test]
fn an_asleep_cut_divides_the_body_without_adding_energy() {
    let mut checked = false;
    let mut w = World::new();
    apply(&mut w, &scene_setup(true));
    for _ in 0..CUT_AT {
        step(&mut w);
    }
    assert!(w.sleep.asleep.iter().all(|&a| a == 1), "both asleep before the cut");
    assert_eq!(w.body_count(), 1);
    apply(&mut w, &[rec(CMD_BREAK_BEAM, SEAM_BEAM, 0, [0.0; 6])]);
    w.events.clear();
    step(&mut w);
    check_two_vessels(&w);
    let (b0, b1) = (w.segment_body(0), w.segment_body(1));
    assert!(b0 != NO_BODY && b1 != NO_BODY && b0 != b1, "each vessel rides its own body ({b0}, {b1})");
    for c in &w.vessel.checks {
        assert_eq!(c.kind, 3.0);
        let (p_scale, l_scale, e_scale) = (100.0 * 100.0, 100.0 * 100.0 * 60.0, 0.5 * 100.0 * 100.0 * 100.0);
        close("division momentum", c.momentum_before, c.momentum_after, 1e-12 * p_scale);
        close("division spin", c.spin_before, c.spin_after, 1e-12 * l_scale);
        let de = c.energy_after - c.energy_before;
        assert!(de <= 1e-12 * e_scale, "the division added {de:e} J");
        println!("asleep cut: |dp| {:e}, |dL| {:e}, dE {de:e} J", (c.momentum_after - c.momentum_before).length(), (c.spin_after - c.spin_before).length());
        checked = true;
    }
    assert!(checked, "the division recorded its check");
}

/// Three rigid cubes stacked in one body, cut between the second and third: two vessels, the
/// lower two cubes still one body, the top cube its own.
#[test]
fn a_tall_stack_cut_gives_two_vessels_each_with_its_own_body() {
    let mut cmds = Vec::new();
    for c in 0..3u32 {
        for k in 0..8u32 {
            let x = if k & 1 != 0 { 0.5 } else { -0.5 };
            let y = 50.0 + 1.5 * c as Real + if k & 2 != 0 { 1.0 } else { 0.0 };
            let z = if k & 4 != 0 { 0.5 } else { -0.5 };
            cmds.push(rec(CMD_ADD_NODE, 0, 0, [x, y, z, 0.1, 0.1, 0.0]));
        }
    }
    for c in 0..3u32 {
        for i in 0..8u32 {
            for bit in [1u32, 2, 4] {
                if i & bit == 0 {
                    cmds.push(beam(8 * c + i, 8 * c + (i | bit), 1.0));
                }
            }
        }
    }
    // Between cubes: each top corner (bit 2 set) to the corner above it, 0.5 m.
    let mut seam = [Vec::new(), Vec::new()];
    let mut index = 36u32;
    for c in 0..2u32 {
        for i in [2u32, 3, 6, 7] {
            cmds.push(beam(8 * c + i, 8 * (c + 1) + (i & !2), 0.5));
            seam[c as usize].push(index);
            index += 1;
        }
    }
    cmds.push(rec(CMD_ADD_SEGMENT, u32::MAX, 0, [0.0; 6]));
    cmds.push(rec(CMD_ADD_SEGMENT, 0, 0, [0.01, 1e-5, 0.0, 0.0, 0.0, 0.0]));
    cmds.push(rec(CMD_ADD_SEGMENT, 1, 0, [0.01, 1e-5, 0.0, 0.0, 0.0, 0.0]));
    for n in 0..24u32 {
        cmds.push(rec(CMD_SET_NODE_SEGMENT, n, n / 8, [0.0; 6]));
    }
    cmds.push(rec(CMD_MAKE_RIGID, 0, u32::MAX, [0.0; 6]));
    cmds.push(rec(CMD_MAKE_RIGID, 1, 0, [0.0; 6]));
    cmds.push(rec(CMD_MAKE_RIGID, 2, 0, [0.0; 6]));
    let mut w = World::new();
    apply(&mut w, &cmds);
    for _ in 0..20 {
        step(&mut w);
    }
    assert_eq!(w.body_count(), 1, "one body before the cut");
    let cut: Vec<Command> = seam[1].iter().map(|&b| rec(CMD_BREAK_BEAM, b, 0, [0.0; 6])).collect();
    apply(&mut w, &cut);
    let before = totals(&w);
    w.events.clear();
    step(&mut w);
    assert_eq!(w.vessel_ids(), 2);
    assert_eq!((w.segment_vessel(0), w.segment_vessel(1), w.segment_vessel(2)), (0, 0, 1));
    let (m0, m1) = (w.vessel_mass(0), w.vessel_mass(1));
    assert_eq!(m0 + m1, 240.0, "{m0} + {m1}");
    assert!((m0 - 160.0).abs() < 1e-9 && (m1 - 80.0).abs() < 1e-9);
    let (b0, b1, b2) = (w.segment_body(0), w.segment_body(1), w.segment_body(2));
    assert!(b0 != NO_BODY && b0 == b1, "the lower two cubes are still one body");
    assert!(b2 != NO_BODY && b2 != b0, "the top cube has its own body");
    assert_eq!(w.body_count(), 2);
    let (p, _) = totals(&w);
    let dp = p - before.0;
    assert!((dp.length() - 240.0 * 9.81 / 60.0).abs() < 1e-9, "only gravity over the cut step: {dp:?}");
    for c in &w.vessel.checks {
        close("stack division momentum", c.momentum_before, c.momentum_after, 1e-12 * 240.0 * 100.0);
        close("stack division spin", c.spin_before, c.spin_after, 1e-12 * 240.0 * 100.0 * 60.0);
        assert!(c.energy_after - c.energy_before <= 1e-12 * 0.5 * 240.0 * 1e4);
    }
}

/// Before any cut the state hash is the one without vessels; the cut scene runs to the same bits
/// twice and to the golden the WebAssembly build is held to.
#[test]
fn the_cut_scene_is_deterministic_and_matches_the_golden() {
    let mut w = World::new();
    apply(&mut w, &scene_setup(true));
    assert!(!w.vessel.ever);
    let a = state_hash(&run_cut(true, STEPS, |_| {}));
    let b = state_hash(&run_cut(true, STEPS, |w| check_two_vessels(w)));
    assert_eq!(a, b, "the same run twice");
    println!("VESSEL_SCENE_200 = {a:#018x}");
    assert_eq!(a, VESSEL_SCENE_200);
}
