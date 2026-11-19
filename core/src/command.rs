//! Commands in (layout rule 8): fixed-layout records, applied in list order before the step.
//! Everything the host does to a world while it runs goes through here, so a recording is just
//! "the command list per step" and a replay is the same lists applied again (milestone 2.4).
//!
//! The record has one shape for every kind: two or three indices and up to six numbers. Any bit
//! pattern is a valid record, so the host can fill the buffer from untrusted bytes; a record that
//! makes no sense (a dead node, a non-finite number) is rejected, not trapped, and the rejection
//! is deterministic, so a replay rejects the same ones.

use crate::real::{Real, Vec3};
use crate::terrain::{Terrain, MAX_GRID};
use crate::world::{BeamSpec, World, BEAM_ALIVE};

/// `f[0..5]` = x, y, z, inverse mass (0 pins), radius. Result: the new node's index.
pub const CMD_ADD_NODE: u32 = 1;
/// `a`, `b` = endpoint nodes; `f[0..5]` = rest length, compliance, damping, yield strain, break
/// strain. Result: the new beam's index.
pub const CMD_ADD_BEAM: u32 = 2;
/// `a` = node. Beams touching it go too. Result: 1.
pub const CMD_DELETE_NODE: u32 = 3;
/// `a` = beam. Result: 1.
pub const CMD_DELETE_BEAM: u32 = 4;
/// `a` = node; `f[0..3]` = force in newtons, kept until the end of the coming step. Result: 1.
pub const CMD_ADD_FORCE: u32 = 5;
/// `a` = node; `f[0]` = inverse mass (0 pins, anything else frees). Result: 1.
pub const CMD_SET_INV_MASS: u32 = 6;
/// Sets the terrain (plan 003, phase 3.1): `a` = points along x, `b` = points along z (each 2 to
/// [`MAX_GRID`]); `f[0..4]` = x of the first point, z of the first point, cell size in metres
/// (above zero), friction coefficient (zero or more). All heights start at zero, and any terrain
/// there was is replaced. Result: 1.
pub const CMD_SET_TERRAIN: u32 = 7;
/// Writes heights into the terrain: `a` = index of the first height (`i + j * nx`), `b` = how many
/// (1 to 6), `f[0..b]` = the heights in metres. Rejected, and nothing written, when there is no
/// terrain, the count is out of range, the run goes past the last height or a height is not a
/// finite number. Result: 1.
pub const CMD_TERRAIN_HEIGHTS: u32 = 8;
/// Adds a thruster (plan 004, phase 4.6): `a` = tail node, `b` = head node (distinct live nodes);
/// `f[0]` = maximum thrust in newtons (finite, zero or more). Its throttle starts at zero. Result:
/// the thruster's index.
pub const CMD_ADD_THRUSTER: u32 = 9;
/// `a` = thruster; `f[0]` = throttle, a finite number from 0 to 1. Result: 1.
pub const CMD_SET_THROTTLE: u32 = 10;
/// Adds a segment (plan 005, phase 5.1): `a` = parent segment, or `u32::MAX` for none; `b` = flags
/// (bit 0 is never-live, any other bit rejects the record); `f[0]` = axial area of the seam to the
/// parent in square metres, `f[1]` = its bending section, each finite and zero or more (checked for
/// a root too, then not stored). A parent must already exist: a parent that does not (the new
/// segment's own number is the cycle of length one) is rejected, so no cycle can form. Its stored
/// mass starts at zero. Result: the segment's index (numbered in the order added).
pub const CMD_ADD_SEGMENT: u32 = 11;
/// Puts a node in a segment: `a` = node, `b` = segment. Adds the node's mass to the segment's
/// stored mass. Rejected, and nothing changed, when the node is not live or is pinned (no mass to
/// store), already has a segment, or the segment does not exist. Result: 1.
pub const CMD_SET_NODE_SEGMENT: u32 = 12;
/// Makes a segment rigid (plan 005, phase 5.2): `a` = segment, `b` = a second segment whose body
/// (or which, if loose) joins the same body, or `u32::MAX` for none. Any body either segment
/// already rides is merged in whole. Momentum and angular momentum are kept, no node moves.
/// Rejected, and nothing changed, when a segment does not exist or the group has no mass or a
/// degenerate inertia (a rod, a single node). Result: the body's slot (its lowest segment).
pub const CMD_MAKE_RIGID: u32 = 13;
/// Releases a segment from its body: `a` = segment. Its nodes go loose with the body's velocity
/// at their position; the rest stays rigid with the same spin (or goes loose too, if what is left
/// has no mass or a degenerate inertia). Rejected, and nothing changed, when the segment does not
/// exist or is not rigid. Result: 1.
pub const CMD_RELEASE_RIGID: u32 = 14;

/// Adds a joint (plan 005, phase 5.5; see [`crate::joint`]): `a` = side A's first axis node,
/// `b` = side B's first axis node; `f[0]` = A's second axis node, `f[1]` = B's second axis node,
/// `f[2]` = A's reference node, `f[3]` = B's reference node, each a whole number from 0 to
/// `u32::MAX` written as a float; `f[4]` = lower limit, `f[5]` = upper limit, radians from the
/// current pose, `−π ≤ min ≤ 0 ≤ max ≤ π` (`−π` and `π` for no limit). Rejected, and nothing
/// changed, when a node is not live, two of the six are the same node, all six ride one body, a
/// limit is out of range, or the axis or a reference arm has no length now. The drive starts
/// free. Result: the joint's index (numbered in the order added).
pub const CMD_ADD_JOINT: u32 = 15;
/// Sets a joint's drive: `a` = joint, `b` = mode (0 free, 1 servo, 2 rotor); `f[0]` = target (a
/// servo's angle in `[−π, π]` from the build pose, a rotor's speed in rad/s), `f[1]` = a servo's
/// rate in rad/s, finite and zero or more, `f[2]` = torque cap in newton-metres, finite and zero or
/// more. Result: 1.
pub const CMD_SET_JOINT_TARGET: u32 = 16;
/// Switches sleeping on or off (plan 005, phase 5.8; see [`crate::sleep`]): `a` = 1 on, 0 off
/// (anything else rejects the record). On also switches the stress estimate on, which it reads.
/// Off wakes every segment it put to sleep at the start of the next step. Result: 1.
pub const CMD_SET_SLEEPING: u32 = 17;
/// Breaks a beam as if it had snapped (plan 005, phase 5.10): `a` = beam. It stays in its slot,
/// flagged broken, and carries nothing; the coming step checks whether the structure fell into
/// separate vessels ([`crate::vessel`]). Rejected, and nothing changed, when the beam is not live or
/// is already broken. No break event: events are what a step did. Result: 1.
pub const CMD_BREAK_BEAM: u32 = 18;
/// Result slot value for a rejected record.
pub const REJECTED: u32 = u32::MAX;

/// One command record. `repr(C)`; the TypeScript host writes it field by field (`src/commands.ts`).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Command {
    pub kind: u32,
    pub a: u32,
    pub b: u32,
    pub reserved: u32,
    pub f: [Real; 6],
}

/// Bytes per record (16 for the integers plus six numbers).
pub const COMMAND_BYTES: usize = 16 + 6 * std::mem::size_of::<Real>();
const _: () = assert!(std::mem::size_of::<Command>() == COMMAND_BYTES);

fn finite(v: &[Real]) -> bool {
    v.iter().all(|x| x.is_finite())
}

/// Applies the commands in order. `results[i]` is the new index for an add, 1 for another
/// accepted command and [`REJECTED`] for a rejected one. Returns the number rejected.
pub fn apply_commands(world: &mut World, cmds: &[Command], results: &mut Vec<u32>) -> u32 {
    results.clear();
    let mut rejected = 0;
    for c in cmds {
        let r = apply_one(world, c);
        if r == REJECTED {
            rejected += 1;
        }
        results.push(r);
    }
    rejected
}

fn node_ok(w: &World, i: u32) -> bool {
    i < w.node_count() && w.node_alive(i)
}

fn apply_one(w: &mut World, c: &Command) -> u32 {
    match c.kind {
        CMD_ADD_NODE => {
            if !finite(&c.f[0..5]) || c.f[3] < 0.0 || c.f[4] < 0.0 {
                return REJECTED;
            }
            w.add_node(Vec3::new(c.f[0], c.f[1], c.f[2]), c.f[3], c.f[4])
                .index
        }
        CMD_ADD_BEAM => {
            let [rest, compliance, damping, yield_strain, break_strain, _] = c.f;
            if !node_ok(w, c.a)
                || !node_ok(w, c.b)
                || c.a == c.b
                || !finite(&c.f[0..5])
                || rest <= 0.0
                || compliance < 0.0
                || damping < 0.0
                || yield_strain < 0.0
                || break_strain <= 0.0
            {
                return REJECTED;
            }
            w.add_beam(
                c.a,
                c.b,
                BeamSpec {
                    rest,
                    compliance,
                    damping,
                    yield_strain,
                    break_strain,
                },
            )
            .index
        }
        CMD_DELETE_NODE => {
            if !node_ok(w, c.a) {
                return REJECTED;
            }
            w.delete_node(c.a);
            1
        }
        CMD_DELETE_BEAM => {
            if c.a >= w.beam_count() || w.beam_flags[c.a as usize] & BEAM_ALIVE == 0 {
                return REJECTED;
            }
            w.delete_beam(c.a);
            1
        }
        CMD_ADD_FORCE => {
            if !node_ok(w, c.a) || !finite(&c.f[0..3]) {
                return REJECTED;
            }
            w.add_force(c.a, Vec3::new(c.f[0], c.f[1], c.f[2]));
            1
        }
        CMD_SET_INV_MASS => {
            if !node_ok(w, c.a) || !c.f[0].is_finite() || c.f[0] < 0.0 {
                return REJECTED;
            }
            w.set_inv_mass(c.a, c.f[0]);
            1
        }
        CMD_SET_TERRAIN => {
            let [x0, z0, cell, friction, _, _] = c.f;
            let sizes_ok = (2..=MAX_GRID).contains(&c.a) && (2..=MAX_GRID).contains(&c.b);
            if !sizes_ok || !finite(&c.f[0..4]) || cell <= 0.0 || friction < 0.0 {
                return REJECTED;
            }
            w.terrain = Some(Terrain::flat(c.a, c.b, x0, z0, cell, friction));
            1
        }
        CMD_TERRAIN_HEIGHTS => {
            let Some(t) = w.terrain.as_mut() else {
                return REJECTED;
            };
            let count = c.b as usize;
            let start = c.a as usize;
            if !(1..=6).contains(&count)
                || start + count > t.height_count()
                || !finite(&c.f[0..count])
            {
                return REJECTED;
            }
            t.heights[start..start + count].copy_from_slice(&c.f[0..count]);
            1
        }
        CMD_ADD_THRUSTER => {
            if !node_ok(w, c.a)
                || !node_ok(w, c.b)
                || c.a == c.b
                || !c.f[0].is_finite()
                || c.f[0] < 0.0
            {
                return REJECTED;
            }
            w.add_thruster(c.a, c.b, c.f[0])
        }
        CMD_SET_THROTTLE => {
            if c.a >= w.thruster_count() || !(0.0..=1.0).contains(&c.f[0]) {
                return REJECTED;
            }
            w.thr_throttle[c.a as usize] = c.f[0];
            1
        }
        CMD_ADD_SEGMENT => match w.add_segment(c.a, c.b, c.f[0], c.f[1]) {
            Some(index) => index,
            None => REJECTED,
        },
        CMD_SET_NODE_SEGMENT => {
            if w.assign_node_segment(c.a, c.b) {
                1
            } else {
                REJECTED
            }
        }
        CMD_MAKE_RIGID => {
            let made = if c.b == u32::MAX {
                w.make_rigid(&[c.a])
            } else {
                w.make_rigid(&[c.a, c.b])
            };
            made.unwrap_or(REJECTED)
        }
        CMD_RELEASE_RIGID => {
            if w.segment_body(c.a) != crate::body::NO_BODY && w.release(&[c.a]) {
                1
            } else {
                REJECTED
            }
        }
        CMD_ADD_JOINT => {
            let whole = |x: Real| x >= 0.0 && x < 4_294_967_296.0 && x == (x as u32) as Real;
            if !(whole(c.f[0]) && whole(c.f[1]) && whole(c.f[2]) && whole(c.f[3])) {
                return REJECTED;
            }
            let nodes = [
                c.a,
                c.f[0] as u32,
                c.f[2] as u32,
                c.b,
                c.f[1] as u32,
                c.f[3] as u32,
            ];
            w.add_joint(nodes, c.f[4], c.f[5]).unwrap_or(REJECTED)
        }
        CMD_SET_JOINT_TARGET => {
            if w.set_joint_target(c.a, c.b, c.f[0], c.f[1], c.f[2]) {
                1
            } else {
                REJECTED
            }
        }
        CMD_SET_SLEEPING => {
            if c.a > 1 {
                return REJECTED;
            }
            w.set_sleeping(c.a == 1);
            1
        }
        CMD_BREAK_BEAM => {
            if c.a >= w.beam_count() || !w.beam_alive(c.a) {
                return REJECTED;
            }
            let k = c.a as usize;
            w.beam_flags[k] |= crate::world::BEAM_BROKEN;
            w.last_lambda[k] = 0.0;
            w.vessel_structure_changed();
            1
        }
        _ => REJECTED,
    }
}
