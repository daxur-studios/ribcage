//! Splitting into vessels (plan 005, phase 5.10). A vessel is one connected piece of structure:
//! nodes joined by unbroken beams and by live joints. Every node starts in
//! vessel 0; nothing here runs, and no hash changes, until a step in a world with segments breaks
//! a beam (or a command breaks or deletes one) and the structure falls into more than one piece.
//!
//! **The check.** After the solve (and before the stress estimate and sleeping read the step), on
//! a step with a break event or a structural command, a union-find over every live node: unbroken
//! beams and live joints (all six nodes). Rigid bodies do not join anything by themselves (a
//! rider's beams stay alive), so sleeping can never change what is connected. Groups are taken in
//! order of their lowest node (island order).
//!
//! **Segments cut inside.** A break inside a segment (a leg's crush bay giving way) can leave its
//! nodes in two vessels. The segment then belongs to the vessel holding most of its mass (ties to
//! the lower id), so a leg torn off below its root is reported in the new vessel; and it is
//! [`Vessel::torn`]: sleeping never makes it rigid, since one body over two loose pieces would
//! move them together.
//!
//! **Identity.** Each group belongs to the lowest vessel id among its nodes. Of the groups of one
//! vessel, the one holding the lowest node keeps the id; every other group is a new vessel,
//! numbered from [`Vessel::next`] in group order. A group made only of nodes added after the
//! first split (no id yet) is a new vessel too. Ids are never reused.
//!
//! **Bodies.** A rigid body whose segments now lie in more than one vessel is divided with the
//! wake formulas: the segments outside the vessel of the body's lowest segment are released (each
//! node takes `V + ω × r`, the rest keeps `ω`), then each other vessel's share is made rigid
//! again from those exact rigid velocities. Momentum, spin and kinetic energy are kept to
//! rounding; a share that is degenerate (a rod) stays live.
//!
//! Deterministic: integer work in ascending index order, plus the body formulas' `+ - * /`.

use crate::event::{Event, EVENT_SPLIT, EVENT_VESSEL_SEGMENT};
use crate::real::Real;
use crate::segment::NO_SEGMENT;
use crate::sleep::{SwitchCheck, SETTLE_STEPS};
use crate::world::{World, NODE_ALIVE};

/// No vessel: a dead node slot, a node added after the first split and not checked yet, a
/// segment with no live node.
pub const NO_VESSEL: u32 = u32::MAX;

/// The vessel state. In the state hash once [`Vessel::ever`] is set.
#[derive(Clone, Debug, Default)]
pub struct Vessel {
    /// Set by the first check that found more than one vessel. Before that every node is in
    /// vessel 0 and [`Vessel::of`] is empty.
    pub ever: bool,
    /// Vessel per node slot, once `ever` (`NO_VESSEL` for a dead slot). A slot past the end is a
    /// node added since the last check: no vessel yet.
    pub of: Vec<u32>,
    /// Vessel per segment, refreshed by each check once `ever` (`NO_VESSEL` for a segment with no
    /// live node). Empty before.
    pub seg: Vec<u32>,
    /// Per segment, refreshed by each check once `ever`: its live nodes lie in more than one vessel.
    pub torn: Vec<bool>,
    /// The id the next new vessel gets. 1 until the first split.
    pub next: u32,
    /// A command broke or deleted a beam or a node since the last check. Consumed by the step.
    pub pending: bool,
    /// The last step's body divisions, one record (kind 3) per step that divided any.
    pub checks: Vec<SwitchCheck>,
}

impl World {
    /// The vessel a node is in now (0 for every node before the first split).
    pub fn node_vessel(&self, i: u32) -> u32 {
        if !self.vessel.ever {
            return if self.node_alive(i) { 0 } else { NO_VESSEL };
        }
        self.vessel.of.get(i as usize).copied().unwrap_or(NO_VESSEL)
    }

    /// The vessel a segment is in (0 for every segment before the first split).
    pub fn segment_vessel(&self, s: u32) -> u32 {
        if !self.vessel.ever {
            return 0;
        }
        self.vessel.seg.get(s as usize).copied().unwrap_or(NO_VESSEL)
    }

    /// Vessel ids handed out so far: one more than the highest. 1 before the first split.
    pub fn vessel_ids(&self) -> u32 {
        if self.vessel.ever { self.vessel.next } else { 1 }
    }

    /// A vessel's mass, kg: its live massive nodes' masses in node order (a segment cut inside
    /// counts in each vessel for the nodes it has there).
    pub fn vessel_mass(&self, v: u32) -> Real {
        let mut m = 0.0 as Real;
        for i in 0..self.pos.len() as u32 {
            let u = i as usize;
            if self.node_alive(i) && self.inv_mass[u] != 0.0 && self.node_vessel(i) == v {
                m += 1.0 / self.inv_mass[u];
            }
        }
        m
    }

    /// True when the segment's live nodes lie in more than one vessel (never, before a split).
    pub fn segment_torn(&self, s: u32) -> bool {
        self.vessel.ever && self.vessel.torn.get(s as usize).copied().unwrap_or(false)
    }

    /// A vessel's momentum and angular momentum about the origin, from its live massive nodes in
    /// index order (a rider's velocity from its body).
    pub fn vessel_momentum(&self, v: u32) -> (crate::real::Vec3, crate::real::Vec3) {
        use crate::real::Vec3;
        let (mut p, mut l) = (Vec3::ZERO, Vec3::ZERO);
        for i in 0..self.pos.len() as u32 {
            let u = i as usize;
            if !self.node_alive(i) || self.inv_mass[u] == 0.0 || self.node_vessel(i) != v {
                continue;
            }
            let m = 1.0 / self.inv_mass[u];
            let b = self.node_body[u];
            let vel = if b == crate::body::NO_BODY {
                self.vel[u]
            } else {
                let b = b as usize;
                self.body_vel[b] + self.body_omega[b].cross(self.pos[u] - self.body_centre[b])
            };
            p = p + vel * m;
            l = l + self.pos[u].cross(vel * m);
        }
        (p, l)
    }

    /// Called by the commands that cut structure: the next step checks for a split.
    pub(crate) fn vessel_structure_changed(&mut self) {
        self.vessel.pending = true;
    }

    /// Called by `delete_node`: the slot has no vessel until it is used and checked again.
    pub(crate) fn vessel_node_left(&mut self, i: u32) {
        if let Some(v) = self.vessel.of.get_mut(i as usize) {
            *v = NO_VESSEL;
        }
    }

    /// Phase 5.10, after the solve: the split check. No-op without segments, or on a step with no
    /// break and no structural command.
    pub(crate) fn vessel_after_solve(&mut self, step: u64) {
        self.vessel.checks.clear();
        let pending = core::mem::take(&mut self.vessel.pending);
        if self.seg_parent.is_empty() {
            return;
        }
        let broke = self.events.iter().any(|e| e.kind == crate::event::EVENT_BREAK);
        if !broke && !pending {
            return;
        }
        let n = self.pos.len();
        let segs = self.seg_parent.len();
        // The union-find over nodes; a group's root is its lowest node.
        let mut root: Vec<u32> = (0..n as u32).collect();
        fn find(root: &mut [u32], mut i: u32) -> u32 {
            while root[i as usize] != i {
                let up = root[root[i as usize] as usize];
                root[i as usize] = up;
                i = up;
            }
            i
        }
        fn join(root: &mut [u32], a: u32, b: u32) {
            let (ra, rb) = (find(root, a), find(root, b));
            if ra != rb {
                let (lo, hi) = if ra < rb { (ra, rb) } else { (rb, ra) };
                root[hi as usize] = lo;
            }
        }
        let alive = |w: &World, i: u32| w.node_flags[i as usize] & NODE_ALIVE != 0;
        for k in 0..self.beam_a.len() as u32 {
            if self.beam_alive(k) {
                let (a, b) = (self.beam_a[k as usize], self.beam_b[k as usize]);
                if alive(self, a) && alive(self, b) {
                    join(&mut root, a, b);
                }
            }
        }
        for j in 0..self.joint_count() as usize {
            if self.joint_nodes_live(j) {
                let nodes = self.joint_nodes[j];
                for k in 1..6 {
                    join(&mut root, nodes[0], nodes[k]);
                }
            }
        }
        let group: Vec<u32> = (0..n as u32).map(|i| find(&mut root, i)).collect();
        // Each group's old vessel: the lowest id among its nodes.
        let old: Vec<u32> = (0..n as u32)
            .map(|i| if alive(self, i) { self.node_vessel(i) } else { NO_VESSEL })
            .collect();
        let mut owner = vec![NO_VESSEL; n];
        for i in 0..n {
            if alive(self, i as u32) && old[i] < owner[group[i] as usize] {
                owner[group[i] as usize] = old[i];
            }
        }
        // New ids, in group order. `kept[v]`: vessel v's first group already holds the id.
        let mut next = self.vessel_ids();
        let mut id = vec![NO_VESSEL; n];
        let mut kept: Vec<bool> = vec![false; next as usize];
        let mut born: Vec<(u32, u32)> = Vec::new(); // (new id, parent id)
        for r in 0..n {
            if !alive(self, r as u32) || group[r] != r as u32 {
                continue;
            }
            let v = owner[r];
            if v != NO_VESSEL && !kept[v as usize] {
                kept[v as usize] = true;
                id[r] = v;
            } else {
                id[r] = next;
                born.push((next, v));
                next += 1;
            }
        }
        let changed = (0..n).any(|i| alive(self, i as u32) && id[group[i] as usize] != old[i]);
        if born.is_empty() && !changed {
            return;
        }
        self.vessel.ever = true;
        self.vessel.next = next;
        self.vessel.of = (0..n)
            .map(|i| if alive(self, i as u32) { id[group[i] as usize] } else { NO_VESSEL })
            .collect();
        self.vessel_assign_segments(segs);
        self.vessel_divide_bodies();
        // One split record per new vessel, then its segments in ascending order.
        for &(v, parent) in &born {
            self.events.push(Event { kind: EVENT_SPLIT, index: v, step, value: parent as Real });
            for s in 0..segs as u32 {
                if self.vessel.seg[s as usize] == v {
                    self.events.push(Event { kind: EVENT_VESSEL_SEGMENT, index: s, step, value: v as Real });
                }
            }
        }
    }

    /// Each segment's vessel and torn flag (module docs): the vessel with most of its node mass,
    /// summed in node order, ties to the lower id; a segment of massless nodes goes by its lowest.
    fn vessel_assign_segments(&mut self, segs: usize) {
        let ids = self.vessel.next as usize;
        let mut mass = vec![0.0 as Real; segs * ids];
        let mut first = vec![NO_VESSEL; segs];
        let mut torn = vec![false; segs];
        for i in 0..self.pos.len() {
            let (s, v) = (self.node_segment[i], self.vessel.of[i]);
            if s == NO_SEGMENT || v == NO_VESSEL {
                continue;
            }
            let s = s as usize;
            if first[s] == NO_VESSEL {
                first[s] = v;
            } else if first[s] != v {
                torn[s] = true;
            }
            if self.inv_mass[i] != 0.0 {
                mass[s * ids + v as usize] += 1.0 / self.inv_mass[i];
            }
        }
        self.vessel.seg = (0..segs)
            .map(|s| {
                let mut best = NO_VESSEL;
                for v in 0..ids {
                    let m = mass[s * ids + v];
                    if m > 0.0 && (best == NO_VESSEL || m > mass[s * ids + best as usize]) {
                        best = v as u32;
                    }
                }
                if best == NO_VESSEL { first[s] } else { best }
            })
            .collect();
        self.vessel.torn = torn;
    }

    /// Divides every body whose segments lie in more than one vessel (module docs).
    fn vessel_divide_bodies(&mut self) {
        let segs = self.seg_body.len();
        let mut cut: Vec<u32> = Vec::new();
        for b in 0..self.body_slot_count() {
            if !self.body_alive(b) {
                continue;
            }
            let home = self.vessel.seg[b as usize];
            if (0..segs).any(|s| self.seg_body[s] == b && self.vessel.seg[s] != home) {
                cut.push(b);
            }
        }
        if cut.is_empty() {
            return;
        }
        let (p0, l0, e0) = self.sleep_totals();
        for &b in &cut {
            let home = self.vessel.seg[b as usize];
            let members: Vec<u32> = (0..segs as u32).filter(|&s| self.seg_body[s as usize] == b).collect();
            let away: Vec<u32> = members.iter().copied().filter(|&s| self.vessel.seg[s as usize] != home).collect();
            self.release(&away);
            // Each other vessel's share, lowest vessel first, made rigid again.
            let mut vessels: Vec<u32> = away.iter().map(|&s| self.vessel.seg[s as usize]).collect();
            vessels.sort_unstable();
            vessels.dedup();
            for v in vessels {
                let share: Vec<u32> = away.iter().copied().filter(|&s| self.vessel.seg[s as usize] == v).collect();
                let _ = self.make_rigid(&share);
            }
            // A sleeping share's body was just re-formed: it settles again.
            for &s in &members {
                let u = s as usize;
                if u < self.sleep.asleep.len() && self.sleep.asleep[u] != 0 {
                    self.sleep.settle_left[u] = SETTLE_STEPS;
                }
            }
        }
        let (p1, l1, e1) = self.sleep_totals();
        self.vessel.checks.push(SwitchCheck {
            kind: 3.0,
            momentum_before: p0,
            spin_before: l0,
            energy_before: e0,
            momentum_after: p1,
            spin_after: l1,
            energy_after: e1,
        });
    }

    /// True when the seam's two segments are in one vessel (always, before the first split).
    pub(crate) fn seam_in_one_vessel(&self, k: usize) -> bool {
        !self.vessel.ever || self.segment_vessel(self.seam_a[k]) == self.segment_vessel(self.seam_b[k])
    }
}
