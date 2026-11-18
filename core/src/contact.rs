//! Ground contact finding (plan 003, phases 3.2 and 3.3). Once per step, before the substeps,
//! each island lists the movable nodes and the beam midpoints that may touch the ground during
//! the step and the plane under each. The solve ([`crate::solver::solve_ground_contact`]) then holds each contact against
//! that plane for every substep of the step.
//!
//! A pure function of the island buffer and the terrain, so it can run on a worker thread with
//! the island's solve later.

use crate::island::Island;
use crate::real::Real;
use crate::sim::gravity_at;
use crate::terrain::Terrain;
use crate::STEP_SECONDS;

/// Extra gap, in metres, under which a node gets a contact even if its speed and gravity alone
/// would not bring it to the ground this step: the margin's fixed skin.
pub const CONTACT_SKIN: Real = 0.01;

/// Fills the island's contact arrays for this step, in ascending packed node order.
///
/// A movable node gets a contact when its gap to the ground (distance above the plane under it,
/// minus its radius) is under `|v| · Δt + ½ · |g| · Δt² + CONTACT_SKIN`: as far as it could
/// travel in one step at its present speed under gravity, plus a skin. Beams can still pull a
/// node further than that; the next step catches it, and the push removes any new overlap at
/// once. Pinned nodes get no contact (the ground cannot move them). The plane is taken under the
/// node's position at the step's start and kept for the whole step.
pub fn find_contacts(isl: &mut Island, terrain: &Terrain) {
    isl.contact_node.clear();
    isl.contact_normal.clear();
    isl.contact_offset.clear();
    isl.beam_contact.clear();
    isl.beam_contact_normal.clear();
    isl.beam_contact_offset.clear();
    isl.friction = terrain.friction;
    let dt = STEP_SECONDS;
    for i in 0..isl.pos.len() {
        // A rider (phase 5.3) gets its contact as a riding endpoint of its body.
        if isl.inv_mass[i] == 0.0 && !isl.rides(i) {
            continue;
        }
        let p = isl.pos[i];
        let plane = terrain.plane_at(p.x, p.z);
        let gap = plane.distance(p) - isl.radius[i];
        let margin = isl.vel[i].length() * dt + 0.5 * gravity_at(p).length() * dt * dt + CONTACT_SKIN;
        if gap < margin {
            isl.contact_node.push(i as u32);
            isl.contact_normal.push(plane.normal);
            isl.contact_offset.push(plane.offset);
        }
    }
    // Beam midpoints (phase 3.3), after all node contacts, in ascending beam order. The same
    // test as for a node, with the midpoint's position and speed and the smaller end radius.
    // A beam between two pinned nodes cannot be pushed and gets none.
    for k in 0..isl.beam_a.len() {
        let (a, b) = (isl.beam_a[k] as usize, isl.beam_b[k] as usize);
        if isl.inv_mass[a] == 0.0 && !isl.rides(a) && isl.inv_mass[b] == 0.0 && !isl.rides(b) {
            continue;
        }
        let p = (isl.pos[a] + isl.pos[b]) * 0.5;
        let v = (isl.vel[a] + isl.vel[b]) * 0.5;
        let radius = if isl.radius[a] < isl.radius[b] { isl.radius[a] } else { isl.radius[b] };
        let plane = terrain.plane_at(p.x, p.z);
        let gap = plane.distance(p) - radius;
        let margin = v.length() * dt + 0.5 * gravity_at(p).length() * dt * dt + CONTACT_SKIN;
        if gap < margin {
            isl.beam_contact.push(k as u32);
            isl.beam_contact_normal.push(plane.normal);
            isl.beam_contact_offset.push(plane.offset);
        }
    }
}
