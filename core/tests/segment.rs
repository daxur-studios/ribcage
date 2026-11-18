//! Phase 5.1: segments, seams and the assembly tree. Records, rejections, stored mass, centre and
//! inertia, the seam of a beam, slot reuse and the state hash.

use beam_core::command::*;
use beam_core::real::{Real, Vec3};
use beam_core::segment::{NO_SEGMENT, SEG_NEVER_LIVE};
use beam_core::sim::{physics_hash, state_hash, step};
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

fn seg(parent: u32, flags: u32, area: f64, bending: f64) -> Command {
    rec(CMD_ADD_SEGMENT, parent, flags, [area, bending, 0.0, 0.0, 0.0, 0.0])
}

fn assign(node: u32, segment: u32) -> Command {
    rec(CMD_SET_NODE_SEGMENT, node, segment, [0.0; 6])
}

fn apply(w: &mut World, cmds: &[Command]) -> (Vec<u32>, u32) {
    let mut r = Vec::new();
    let rejected = apply_commands(w, cmds, &mut r);
    (r, rejected)
}

/// Four nodes in a row (masses 10, 20, 30, 40), a beam between neighbours, two segments:
/// 0 = nodes 0 and 1, 1 = nodes 2 and 3, child of 0.
fn two_segments() -> World {
    let mut w = World::new();
    let cmds = [
        node(0.0, 50.0, 0.0, 10.0),
        node(1.0, 50.0, 0.0, 20.0),
        node(2.0, 51.0, 0.0, 30.0),
        node(3.0, 50.0, 1.0, 40.0),
        beam(0, 1, 1.0),
        beam(1, 2, 1.4142135623730951),
        beam(2, 3, 1.4142135623730951),
        seg(NO_SEGMENT, 0, 0.0, 0.0),
        seg(0, SEG_NEVER_LIVE, 0.25, 0.01),
        assign(0, 0),
        assign(1, 0),
        assign(2, 1),
        assign(3, 1),
    ];
    let (r, rejected) = apply(&mut w, &cmds);
    assert_eq!(rejected, 0, "{r:?}");
    assert_eq!((r[7], r[8]), (0, 1), "segments are numbered in the order added");
    w
}

#[test]
fn segments_seams_and_stored_mass() {
    let w = two_segments();
    assert_eq!((w.segment_count(), w.seam_count()), (2, 1));
    assert_eq!(w.seg_parent, vec![NO_SEGMENT, 0]);
    assert_eq!(w.seg_seam, vec![NO_SEGMENT, 0]);
    assert_eq!(w.seg_flags, vec![0, SEG_NEVER_LIVE]);
    assert_eq!((w.seam_a[0], w.seam_b[0]), (0, 1));
    assert_eq!((w.seam_area[0], w.seam_bending[0]), (0.25, 0.01));
    assert_eq!(w.node_segment, vec![0, 0, 1, 1]);
    assert!((w.seg_mass[0] - 30.0).abs() < 1e-12 && (w.seg_mass[1] - 70.0).abs() < 1e-12);
}

#[test]
fn a_beam_knows_its_seam_from_its_ends() {
    let w = two_segments();
    assert_eq!(w.beam_seam(0), None, "both ends in segment 0");
    assert_eq!(w.beam_seam(1), Some(0), "node 1 in segment 0, node 2 in segment 1");
    assert_eq!(w.beam_seam(2), None, "both ends in segment 1");
    assert_eq!(w.beam_seam(99), None);
}

#[test]
fn bad_records_are_rejected_and_the_rest_of_the_batch_applies() {
    let mut w = two_segments();
    let n = f64::NAN;
    let cmds = [
        seg(5, 0, 1.0, 1.0),                  // 0: parent does not exist
        seg(2, 0, 1.0, 1.0),                  // 1: the new segment's own number (a cycle of one)
        seg(3, 0, 1.0, 1.0),                  // 2: a parent that comes later (a longer cycle)
        seg(0, 2, 1.0, 1.0),                  // 3: unknown flag bit
        seg(0, 0, n, 1.0),                    // 4: area is not a number
        seg(0, 0, 1.0, f64::INFINITY),        // 5: bending is infinite
        seg(0, 0, -1.0, 1.0),                 // 6: negative area
        seg(NO_SEGMENT, 0, n, 0.0),           // 7: a root's numbers are checked too
        seg(1, 0, 0.5, 0.02),                 // 8: good, number 2
        node(9.0, 50.0, 0.0, 5.0),            // 9: good, node 4
        assign(4, 7),                         // 10: no such segment
        assign(99, 2),                        // 11: no such node
        assign(0, 2),                         // 12: already in a segment
        assign(4, 2),                         // 13: good
        rec(CMD_ADD_NODE, 0, 0, [10.0, 50.0, 0.0, 0.0, 0.1, 0.0]), // 14: a pinned node, node 5
        assign(5, 2),                         // 15: a pinned node has no mass to store
        rec(CMD_DELETE_NODE, 4, 0, [0.0; 6]), // 16: good, node 4 leaves its segment
        assign(4, 2),                         // 17: a dead node
    ];
    let (r, rejected) = apply(&mut w, &cmds);
    let bad = [0, 1, 2, 3, 4, 5, 6, 7, 10, 11, 12, 15, 17];
    for i in 0..cmds.len() {
        assert_eq!(r[i] == REJECTED, bad.contains(&i), "record {i}: {r:?}");
    }
    assert_eq!(rejected, bad.len() as u32);
    assert_eq!(r[8], 2, "the good segment is number 2, none of the bad ones took a number");
    assert_eq!(w.segment_count(), 3);
    assert_eq!(w.seam_count(), 2, "a rejected segment adds no seam either");
    assert_eq!(w.node_segment[5], NO_SEGMENT);
    assert_eq!(w.seg_mass[2], 0.0, "the one node that joined was deleted again");
}

#[test]
fn stored_mass_follows_deletes_mass_changes_and_slot_reuse() {
    let mut w = two_segments();
    apply(&mut w, &[rec(CMD_SET_INV_MASS, 3, 0, [1.0 / 100.0, 0.0, 0.0, 0.0, 0.0, 0.0])]);
    assert!((w.seg_mass[1] - 130.0).abs() < 1e-12, "{}", w.seg_mass[1]);
    apply(&mut w, &[rec(CMD_DELETE_NODE, 2, 0, [0.0; 6])]);
    assert!((w.seg_mass[1] - 100.0).abs() < 1e-12);
    assert_eq!(w.node_segment[2], NO_SEGMENT);
    // The freed slot comes back with no segment.
    let (r, _) = apply(&mut w, &[node(7.0, 7.0, 7.0, 3.0)]);
    assert_eq!(r[0], 2);
    assert_eq!(w.node_segment[2], NO_SEGMENT);
    assert!((w.seg_mass[1] - 100.0).abs() < 1e-12);
    // Pinning a node takes its mass out.
    apply(&mut w, &[rec(CMD_SET_INV_MASS, 3, 0, [0.0; 6])]);
    assert_eq!(w.seg_mass[1], 0.0);
}

#[test]
fn centre_and_inertia_match_a_direct_sum() {
    let w = two_segments();
    let p = w.segment_centre_and_inertia(1).expect("segment 1 has mass");
    // Nodes 2 and 3: masses 30 and 40 at (2, 51, 0) and (3, 50, 1).
    let (m2, m3) = (30.0, 40.0);
    let c = Vec3::new((m2 * 2.0 + m3 * 3.0) / 70.0, (m2 * 51.0 + m3 * 50.0) / 70.0, (m2 * 0.0 + m3 * 1.0) / 70.0);
    let (r2, r3) = (Vec3::new(2.0, 51.0, 0.0) - c, Vec3::new(3.0, 50.0, 1.0) - c);
    let xx = m2 * (r2.y * r2.y + r2.z * r2.z) + m3 * (r3.y * r3.y + r3.z * r3.z);
    let xy = -(m2 * r2.x * r2.y + m3 * r3.x * r3.y);
    let rel = |a: f64, b: f64| ((a - b) / b.abs().max(1e-300)).abs();
    assert!(rel(p.centre.x, c.x) < 1e-12 && rel(p.centre.y, c.y) < 1e-12 && rel(p.centre.z, c.z) < 1e-12);
    assert!(rel(p.inertia[0], xx) < 1e-12 && rel(p.inertia[3], xy) < 1e-12);
    assert_eq!(p.mass, w.seg_mass[1]);
    assert!(w.segment_centre_and_inertia(2).is_none());
    // Read-only: asking changes nothing.
    assert_eq!(state_hash(&w), state_hash(&two_segments()));
}

#[test]
fn a_segment_with_no_mass_has_no_centre() {
    let mut w = World::new();
    apply(&mut w, &[seg(NO_SEGMENT, 0, 0.0, 0.0)]);
    assert!(w.segment_centre_and_inertia(0).is_none());
}

#[test]
fn segments_enter_the_hash_only_once_they_exist() {
    let mut bare = World::new();
    let cmds = [node(0.0, 50.0, 0.0, 10.0), node(1.0, 50.0, 0.0, 20.0), beam(0, 1, 1.0)];
    apply(&mut bare, &cmds);
    let before = state_hash(&bare);
    assert_eq!(before, physics_hash(&bare), "no segments, no difference");

    let mut tagged = World::new();
    apply(&mut tagged, &cmds);
    apply(&mut tagged, &[seg(NO_SEGMENT, 0, 0.0, 0.0), assign(0, 0)]);
    assert_eq!(physics_hash(&tagged), before, "the physics hash ignores segments");
    assert_ne!(state_hash(&tagged), before, "the full hash sees them");

    // And segments change no physics: the same steps give the same physics hash.
    for _ in 0..30 {
        step(&mut bare);
        step(&mut tagged);
    }
    assert_eq!(physics_hash(&tagged), physics_hash(&bare));
    assert_eq!(physics_hash(&bare), state_hash(&bare));
}
