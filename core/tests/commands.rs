//! Phase 2.2: the command buffer. Records are applied in list order, a record that makes no sense
//! is rejected the same way every time, and the beam output reports strain in yield units.

use beam_core::command::*;
use beam_core::ffi::*;
use beam_core::real::Real;
use beam_core::sim::{state_hash, step};
use beam_core::world::{World, NODE_PINNED};

fn rec(kind: u32, a: u32, b: u32, f: [Real; 6]) -> Command {
    Command { kind, a, b, reserved: 0, f }
}

fn node(x: Real, inv_mass: Real) -> Command {
    rec(CMD_ADD_NODE, 0, 0, [x, 0.0, 0.0, inv_mass, 0.1, 0.0])
}

fn beam(a: u32, b: u32, rest: Real) -> Command {
    rec(CMD_ADD_BEAM, a, b, [rest, 1e-6, 0.5, 0.01, 0.05, 0.0])
}

#[test]
fn record_is_64_bytes() {
    assert_eq!(COMMAND_BYTES, 64);
    assert_eq!(std::mem::size_of::<Command>(), 64);
}

#[test]
fn adds_apply_in_order_and_report_indices() {
    let mut w = World::new();
    let mut r = Vec::new();
    let cmds = [node(0.0, 0.0), node(1.0, 1.0), beam(0, 1, 1.0), rec(CMD_ADD_FORCE, 1, 0, [0.0, -5.0, 0.0, 0.0, 0.0, 0.0])];
    assert_eq!(apply_commands(&mut w, &cmds, &mut r), 0);
    assert_eq!(r, vec![0, 1, 0, 1]);
    assert_eq!((w.node_count(), w.beam_count()), (2, 1));
    assert_eq!(w.force[1].y, -5.0);
}

#[test]
fn nonsense_is_rejected_and_the_rest_still_applies() {
    let mut w = World::new();
    let mut r = Vec::new();
    let nan = Real::NAN;
    let cmds = [
        node(0.0, 0.0),                                                      // 0 ok
        node(1.0, 1.0),                                                      // 1 ok
        node(nan, 1.0),                                                      // rejected: not finite
        node(2.0, -1.0),                                                     // rejected: negative inverse mass
        beam(0, 0, 1.0),                                                     // rejected: same node twice
        beam(0, 9, 1.0),                                                     // rejected: no such node
        beam(0, 1, 0.0),                                                     // rejected: zero rest length
        beam(0, 1, 1.0),                                                     // ok: beam 0
        rec(99, 0, 0, [0.0; 6]),                                             // rejected: unknown kind
        rec(0, 0, 0, [0.0; 6]),                                              // rejected: zeroed record
        rec(CMD_ADD_FORCE, 0, 0, [Real::INFINITY, 0.0, 0.0, 0.0, 0.0, 0.0]), // rejected
        rec(CMD_DELETE_BEAM, 5, 0, [0.0; 6]),                                // rejected: no such beam
        rec(CMD_DELETE_NODE, 1, 0, [0.0; 6]),                                // ok: takes beam 0 with it
        rec(CMD_ADD_FORCE, 1, 0, [0.0, 1.0, 0.0, 0.0, 0.0, 0.0]),            // rejected: node 1 is dead now
        beam(0, 1, 1.0),                                                     // rejected: node 1 is dead now
    ];
    let rejected = apply_commands(&mut w, &cmds, &mut r);
    let want = [0, 1, REJECTED, REJECTED, REJECTED, REJECTED, REJECTED, 0, REJECTED, REJECTED, REJECTED, REJECTED, 1, REJECTED, REJECTED];
    assert_eq!(r, want);
    assert_eq!(rejected, 11);
    assert!(!w.node_alive(1) && !w.beam_alive(0));
    // The same list applied to a second world gives the same results and the same state.
    let mut w2 = World::new();
    let mut r2 = Vec::new();
    apply_commands(&mut w2, &cmds, &mut r2);
    assert_eq!(r, r2);
    assert_eq!(state_hash(&w), state_hash(&w2));
}

#[test]
fn set_inv_mass_pins_and_frees() {
    let mut w = World::new();
    let mut r = Vec::new();
    apply_commands(&mut w, &[node(0.0, 1.0)], &mut r);
    assert_eq!(w.node_flags[0] & NODE_PINNED, 0);
    apply_commands(&mut w, &[rec(CMD_SET_INV_MASS, 0, 0, [0.0; 6])], &mut r);
    assert_ne!(w.node_flags[0] & NODE_PINNED, 0);
    for _ in 0..10 {
        step(&mut w);
    }
    assert_eq!(w.pos[0].y, 0.0, "a pinned node does not fall");
    apply_commands(&mut w, &[rec(CMD_SET_INV_MASS, 0, 0, [0.5, 0.0, 0.0, 0.0, 0.0, 0.0])], &mut r);
    assert_eq!(w.node_flags[0] & NODE_PINNED, 0);
    for _ in 0..10 {
        step(&mut w);
    }
    assert!(w.pos[0].y < 0.0, "a freed node falls");
}

/// Through the boundary: the host writes into the reserved buffer, and the beam output reports
/// the signed strain in yield units.
#[test]
fn boundary_applies_commands_and_reports_strain() {
    unsafe {
        let h = rc_host_new();
        assert_eq!(rc_command_bytes(), 64);
        let cmds = [node(0.0, 0.0), node(1.1, 0.0), beam(0, 1, 1.0), beam(0, 1, 1.1)];
        let p = rc_commands_reserve(h, cmds.len() as u32);
        std::ptr::copy_nonoverlapping(cmds.as_ptr(), p, cmds.len());
        assert_eq!(rc_apply_commands(h), 0);
        let res = std::slice::from_raw_parts(rc_results_ptr(h), cmds.len());
        assert_eq!(res, &[0, 1, 0, 1]);

        let ends = rc_write_beams(h);
        let ratio = std::slice::from_raw_parts(rc_beam_ratio_ptr(h), 2);
        let ends = std::slice::from_raw_parts(ends, 4);
        assert_eq!(ends, &[0, 1, 0, 1]);
        // Beam 0: rest 1.0 stretched to 1.1 is strain 0.1, ten times the 0.01 yield strain.
        assert!((ratio[0] - 10.0).abs() < 1e-4, "{}", ratio[0]);
        assert!(ratio[1].abs() < 1e-4, "{}", ratio[1]);
        let flags = std::slice::from_raw_parts(rc_beam_flags_ptr(h), 2);
        assert_eq!(flags, &[1, 1]);
        rc_host_free(h);
    }
}
