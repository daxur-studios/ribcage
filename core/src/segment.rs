//! Segments, seams and the assembly tree (plan 005, phase 5.1).
//!
//! A segment is a group of nodes that belong together (a tank bay, a leg, an engine). Segments
//! form a tree: each has at most one parent segment, joined to it by one seam that carries the
//! section properties milestone 5 needs (axial area and bending section). The arrays are plain
//! flat vectors on [`World`], one per field (layout rule 3):
//!
//! - per node: `node_segment`, the segment it belongs to, or [`NO_SEGMENT`];
//! - per segment: `seg_flags`, `seg_parent`, `seg_seam` (the seam to the parent) and `seg_mass`;
//! - per seam: `seam_a` (the parent segment), `seam_b` (the child), `seam_area`, `seam_bending`.
//!
//! Segments and seams are only ever added, so an index is stable. A parent must already exist
//! when its child is added, which makes a cycle impossible by construction; a record that names
//! a parent that does not exist yet (its own number included, which is the cycle of length one)
//! is rejected. A beam's seam is not stored: it follows from its two end nodes' segments
//! ([`World::beam_seam`]).
//!
//! **Segments change no physics in this phase.** Nothing in the solver reads these arrays. They
//! enter the state hash only once a segment exists ([`crate::sim::state_hash`]), after
//! everything else, so a world that never runs a segment command hashes as it did before, and
//! [`crate::sim::physics_hash`] is the hash without them.
//!
//! Only add, subtract, multiply and divide appear in the maths here, in a fixed order (index
//! order over the nodes), so the same world gives the same bits natively and in WebAssembly.

use crate::real::{Real, Vec3};
use crate::world::World;

/// A node that belongs to no segment, and a segment with no parent or no seam.
pub const NO_SEGMENT: u32 = u32::MAX;

/// Segment flag: never simulated as a live lattice once rigid bodies exist (engine, pod).
pub const SEG_NEVER_LIVE: u32 = 1;
/// Every flag bit a record may set. A record with any other bit is rejected.
pub const SEG_FLAGS_KNOWN: u32 = SEG_NEVER_LIVE;

/// Mass, centre and inertia of a segment, from the nodes' current positions.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SegmentProps {
    /// The stored mass of the segment ([`World::seg_mass`]).
    pub mass: Real,
    /// Mass-weighted mean of the node positions.
    pub centre: Vec3,
    /// Inertia tensor about the centre in world axes, as `[xx, yy, zz, xy, xz, yz]` with the
    /// products negated (`xy = -Σ m x y`), so the full tensor is `[[xx, xy, xz], [xy, yy, yz],
    /// [xz, yz, zz]]`.
    pub inertia: [Real; 6],
}

impl World {
    pub fn segment_count(&self) -> u32 {
        self.seg_parent.len() as u32
    }

    pub fn seam_count(&self) -> u32 {
        self.seam_a.len() as u32
    }

    /// Adds a segment with `parent` (a segment that already exists, or [`NO_SEGMENT`] for a root)
    /// and, for a child, a seam to it with the given section properties. Returns its index, or
    /// `None` and changes nothing when `parent` does not exist yet, `flags` has an unknown bit,
    /// or `area` or `bending` is not a finite number of zero or more. A root's two numbers are
    /// checked too and then not stored.
    pub fn add_segment(&mut self, parent: u32, flags: u32, area: Real, bending: Real) -> Option<u32> {
        let index = self.segment_count();
        let parent_ok = parent == NO_SEGMENT || parent < index;
        if !parent_ok || flags & !SEG_FLAGS_KNOWN != 0 {
            return None;
        }
        if !(area.is_finite() && area >= 0.0 && bending.is_finite() && bending >= 0.0) {
            return None;
        }
        let seam = if parent == NO_SEGMENT {
            NO_SEGMENT
        } else {
            self.seam_a.push(parent);
            self.seam_b.push(index);
            self.seam_area.push(area);
            self.seam_bending.push(bending);
            self.seam_count() - 1
        };
        self.seg_flags.push(flags);
        self.seg_parent.push(parent);
        self.seg_seam.push(seam);
        self.seg_mass.push(0.0);
        Some(index)
    }

    /// Puts a node in a segment and adds its mass to the segment's stored mass. Refused (and
    /// nothing changes) when the node is not live, has no mass to add (pinned), already has a
    /// segment, or the segment does not exist.
    pub fn assign_node_segment(&mut self, node: u32, segment: u32) -> bool {
        let (n, s) = (node as usize, segment as usize);
        if node >= self.node_count() || !self.node_alive(node) || segment >= self.segment_count() {
            return false;
        }
        if self.node_segment[n] != NO_SEGMENT || self.inv_mass[n] == 0.0 {
            return false;
        }
        self.node_segment[n] = segment;
        self.seg_mass[s] += 1.0 / self.inv_mass[n];
        true
    }

    /// The segment's stored mass again, summed over its live movable nodes in index order. Used
    /// when a node leaves a segment or changes mass, so the stored number never drifts.
    pub(crate) fn recompute_segment_mass(&mut self, segment: u32) {
        let mut total = 0.0;
        for i in 0..self.pos.len() {
            if self.node_segment[i] == segment && self.node_flags[i] & crate::world::NODE_ALIVE != 0 && self.inv_mass[i] != 0.0 {
                total += 1.0 / self.inv_mass[i];
            }
        }
        self.seg_mass[segment as usize] = total;
    }

    /// Called when a node's slot is freed: it leaves its segment.
    pub(crate) fn release_node_segment(&mut self, node: u32) {
        let s = self.node_segment[node as usize];
        if s != NO_SEGMENT {
            self.node_segment[node as usize] = NO_SEGMENT;
            self.recompute_segment_mass(s);
        }
    }

    /// Called after a node's inverse mass changed: its segment's stored mass follows.
    pub(crate) fn segment_mass_changed(&mut self, node: u32) {
        let s = self.node_segment[node as usize];
        if s != NO_SEGMENT {
            self.recompute_segment_mass(s);
        }
    }

    /// The seam between two segments, if one is the other's parent; `None` for the same segment,
    /// for segments with no seam between them, and for [`NO_SEGMENT`].
    pub fn seam_between(&self, a: u32, b: u32) -> Option<u32> {
        let n = self.segment_count();
        if a >= n || b >= n || a == b {
            return None;
        }
        if self.seg_parent[b as usize] == a {
            Some(self.seg_seam[b as usize])
        } else if self.seg_parent[a as usize] == b {
            Some(self.seg_seam[a as usize])
        } else {
            None
        }
    }

    /// The seam a beam crosses, from its end nodes' segments (nothing is stored per beam). `None`
    /// when either end has no segment, both ends are in one segment, or the two segments are not
    /// neighbours in the tree.
    pub fn beam_seam(&self, beam: u32) -> Option<u32> {
        let u = beam as usize;
        if u >= self.beam_a.len() {
            return None;
        }
        self.seam_between(self.node_segment[self.beam_a[u] as usize], self.node_segment[self.beam_b[u] as usize])
    }

    /// Centre and inertia of a segment from the nodes' current positions, summed over its live
    /// movable nodes in index order. `None` for a segment that does not exist or has no mass.
    /// Read-only: nothing is stored (plan 005: mass is stored, the rest is computed at the switch).
    pub fn segment_centre_and_inertia(&self, segment: u32) -> Option<SegmentProps> {
        if segment >= self.segment_count() {
            return None;
        }
        let mut mass = 0.0;
        let mut moment = Vec3::ZERO;
        for i in 0..self.pos.len() {
            if self.node_segment[i] == segment && self.node_flags[i] & crate::world::NODE_ALIVE != 0 && self.inv_mass[i] != 0.0 {
                let m = 1.0 / self.inv_mass[i];
                mass += m;
                moment = moment + self.pos[i] * m;
            }
        }
        if !(mass > 0.0) {
            return None;
        }
        let centre = moment / mass;
        let mut inertia = [0.0; 6];
        for i in 0..self.pos.len() {
            if self.node_segment[i] == segment && self.node_flags[i] & crate::world::NODE_ALIVE != 0 && self.inv_mass[i] != 0.0 {
                let m = 1.0 / self.inv_mass[i];
                let r = self.pos[i] - centre;
                inertia[0] += m * (r.y * r.y + r.z * r.z);
                inertia[1] += m * (r.x * r.x + r.z * r.z);
                inertia[2] += m * (r.x * r.x + r.y * r.y);
                inertia[3] -= m * (r.x * r.y);
                inertia[4] -= m * (r.x * r.z);
                inertia[5] -= m * (r.y * r.z);
            }
        }
        Some(SegmentProps { mass: self.seg_mass[segment as usize], centre, inertia })
    }
}
