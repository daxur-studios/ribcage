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
            w.add_node(Vec3::new(c.f[0], c.f[1], c.f[2]), c.f[3], c.f[4]).index
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
            w.add_beam(c.a, c.b, BeamSpec { rest, compliance, damping, yield_strain, break_strain }).index
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
            if !(1..=6).contains(&count) || start + count > t.height_count() || !finite(&c.f[0..count]) {
                return REJECTED;
            }
            t.heights[start..start + count].copy_from_slice(&c.f[0..count]);
            1
        }
        _ => REJECTED,
    }
}
