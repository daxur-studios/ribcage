//! Sleep and wake (plan 005, phase 5.8). A segment that has carried little and steady load for a
//! while goes to sleep: it becomes rigid, and touching rigid segments are one body. A sleeping
//! segment wakes on a trigger or when the stress estimate (phase 5.7) reads it near its wake
//! level. Off by default; `CMD_SET_SLEEPING` turns it on. With it off nothing here runs and no
//! hash changes.
//!
//! **Readings.** Each step, after the estimate, every segment gets one load reading as a fraction
//! of yield: a live segment's is *measured*, the worst `|last_lambda| / F_yield` over the solved
//! beams touching it (its own and its seams'). A sleeping segment's is the measured reading it
//! fell asleep with plus [`ESTIMATE_MARGIN`] times how far the estimate has moved since: the
//! estimate reads low at single-step peaks and is blind to locked-in stress (ER 005, 5.7), so it
//! is used only for the change, calibrated against the solver at the switch. For
//! [`SETTLE_STEPS`] after any pass that re-formed the bodies, a sleeping segment reads its
//! measured value and the calibration follows the estimate: a rigid stance carries its feet
//! unevenly, so the estimate jumps once at the switch and settles (ER 005, 5.8). A never-live rigid
//! segment's is the estimate times the margin.
//!
//! **Sleep.** A live segment that is allowed to sleep ([`World::sleep_eligible`]), has been live
//! at least [`MIN_LIVE_STEPS`], and has read under [`SLEEP_LEVEL`] and changed by no more than
//! [`QUIET_CHANGE`] per step for [`QUIET_STEPS`] steps in a row goes to sleep at the end of the
//! step.
//!
//! **Wake.** Before the solve (brief trap 2: the estimate lags a step): a node of a segment that
//! comes within contact range of the ground, a node already in range that closes on it faster
//! than [`CONTACT_WAKE_SPEED`], or a thruster on it whose throttle changed. After the step: a
//! joint on it that moved, or a reading at or over [`WAKE_LEVEL`]. Each trigger wakes the
//! segment it is on and that segment's neighbours in the assembly tree (both sides of each seam),
//! whether the triggering segment is live, asleep or never-live; only sleeping segments change.
//!
//! **Bodies.** On any pass that switched something, the bodies are recomputed by a union-find
//! over segments: two rigid segments are joined when an unbroken beam has one end in each, and
//! each group is one body (named by its lowest segment, as before). A body cut by a wake into
//! pieces is released whole and each piece made rigid again; a group that is not exactly one body
//! is made rigid with [`World::make_rigid`], which sums the bodies and loose nodes in it. Both are
//! the brief's switch formulas, so momentum and spin are unchanged and kinetic energy can only
//! fall on a sleep. Each pass records the totals before and after ([`SwitchCheck`]) for the tests.
//!
//! **Wake damping.** A just-woken segment's beams solve with their damping scaled by
//! `1 + WAKE_DAMP_BOOST · left / WAKE_DAMP_STEPS`, `left` counting down from [`WAKE_DAMP_STEPS`].
//!
//! Every value here is an **[assumption]** from the brief, tuned in phase 5.9. Deterministic maths
//! only (`+ - * /`, `sqrt`), every loop in ascending index order.

use crate::body::NO_BODY;
use crate::event::{Event, EVENT_SLEEP, EVENT_WAKE};
use crate::real::{Real, Vec3};
use crate::segment::{NO_SEGMENT, SEG_NEVER_LIVE};
use crate::sim::gravity_at;
use crate::world::{World, BEAM_ALIVE, BEAM_BROKEN, NODE_ALIVE, NODE_FROZEN};
use crate::STEP_SECONDS;

/// Wake when a reading reaches this fraction of yield (brief: 25%).
pub const WAKE_LEVEL: Real = 0.25;
/// Sleep only while the reading stays under this fraction of yield. The brief said 8%; the hopper's
/// legs carry 19.4% of yield standing, so at 8% a craft at rest never sleeps (ER 005, 5.8). 20% is
/// four fifths of the wake level: a sleeping segment wakes once its load rises by a fifth.
pub const SLEEP_LEVEL: Real = 0.20;
/// "Nearly unchanging": the reading moves by no more than this fraction of yield per step.
pub const QUIET_CHANGE: Real = 0.005;
/// Quiet steps in a row before a sleep (brief: 60).
pub const QUIET_STEPS: u32 = 60;
/// A woken segment stays live at least this many steps (brief: 30).
pub const MIN_LIVE_STEPS: u32 = 30;
/// Steps of extra damping after a wake (brief: about 10).
pub const WAKE_DAMP_STEPS: u32 = 10;
/// Extra damping on the first step after a wake, as a multiple of the beam's own.
pub const WAKE_DAMP_BOOST: Real = 4.0;
/// The estimate reads a rigid segment up to 20% low at peaks (ER 005, 5.7): scaled by this.
pub const ESTIMATE_MARGIN: Real = 1.25;
/// Steps after a body change in which a sleeping segment's estimate only recalibrates. The hopper's
/// whole-craft body settles its foot loads in about 8 steps.
pub const SETTLE_STEPS: u32 = 10;
/// A node in contact range closing on the ground faster than this, m/s, wakes its segment before
/// the solve (a contact force jump the estimate would see a step late). A resting node's speed at
/// the start of a step is near zero; one step of gravity is 0.16 m/s.
pub const CONTACT_WAKE_SPEED: Real = 0.25;

/// Wake reasons, the `value` of a wake event.
pub const WAKE_CONTACT: u32 = 1;
pub const WAKE_IMPACT: u32 = 2;
pub const WAKE_THRUST: u32 = 3;
pub const WAKE_JOINT: u32 = 4;
pub const WAKE_ESTIMATE: u32 = 5;
pub const WAKE_NEIGHBOUR: u32 = 6;
/// Sleeping was switched off.
pub const WAKE_OFF: u32 = 7;

/// Totals of the world's movable mass before and after one switch pass: momentum, angular
/// momentum about the origin and kinetic energy. `kind` is 1 for a wake pass, 2 for a sleep pass.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SwitchCheck {
    pub kind: Real,
    pub momentum_before: Vec3,
    pub spin_before: Vec3,
    pub energy_before: Real,
    pub momentum_after: Vec3,
    pub spin_after: Vec3,
    pub energy_after: Real,
}

const _: () = assert!(std::mem::size_of::<SwitchCheck>() == 15 * 8);

/// The sleep switch and its per-segment state. In the state hash once `CMD_SET_SLEEPING` has run.
#[derive(Clone, Debug, Default)]
pub struct Sleep {
    /// Sleeping is on.
    pub on: bool,
    /// `CMD_SET_SLEEPING` has run at least once: the state below is hashed from then.
    pub ever: bool,
    /// The contact record below holds a check from the previous step.
    pub primed: bool,
    /// Per segment: 1 when this module put it to sleep and it is still rigid.
    pub asleep: Vec<u32>,
    /// Per segment: steps live since its last wake (saturating).
    pub live_steps: Vec<u32>,
    /// Per segment: quiet steps in a row.
    pub quiet_steps: Vec<u32>,
    /// Per segment: wake-damping steps left.
    pub damp_left: Vec<u32>,
    /// Per segment: the last step's reading.
    pub ratio: Vec<Real>,
    /// Per segment: the measured reading and the estimate on the step it fell asleep.
    pub ref_measured: Vec<Real>,
    pub ref_estimate: Vec<Real>,
    /// Per segment: settle steps left (see [`SETTLE_STEPS`]).
    pub settle_left: Vec<u32>,
    /// Per thruster: the throttle at the last check.
    pub throttle: Vec<Real>,
    /// Per node: within contact range of the ground at the last check.
    pub near: Vec<bool>,
    /// The switch passes of the last step, in order. Not hashed.
    pub checks: Vec<SwitchCheck>,
}

impl Sleep {
    /// Whether any segment is still in its wake damping.
    pub fn boosting(&self) -> bool {
        self.damp_left.iter().any(|&d| d > 0)
    }

    /// The damping scale of a beam whose ends are in segments `sa` and `sb`.
    pub fn damp_scale(&self, sa: u32, sb: u32) -> Real {
        let mut left = 0;
        for s in [sa, sb] {
            if s != NO_SEGMENT && (s as usize) < self.damp_left.len() && self.damp_left[s as usize] > left {
                left = self.damp_left[s as usize];
            }
        }
        1.0 + WAKE_DAMP_BOOST * left as Real / WAKE_DAMP_STEPS as Real
    }
}

impl World {
    /// `CMD_SET_SLEEPING`. Turning it off wakes every sleeping segment at the start of the next step.
    pub fn set_sleeping(&mut self, on: bool) {
        self.sleep.on = on;
        self.sleep.ever = true;
        if on {
            self.est.on = true;
        }
    }

    fn sleep_size(&mut self) {
        let segs = self.seg_parent.len();
        let sl = &mut self.sleep;
        sl.asleep.resize(segs, 0);
        sl.live_steps.resize(segs, 0);
        sl.quiet_steps.resize(segs, 0);
        sl.damp_left.resize(segs, 0);
        sl.ratio.resize(segs, 0.0);
        sl.ref_measured.resize(segs, 0.0);
        sl.ref_estimate.resize(segs, 0.0);
        sl.settle_left.resize(segs, 0);
        sl.throttle.resize(self.thr_throttle.len(), 0.0);
        sl.near.resize(self.pos.len(), false);
    }

    /// Per segment: may this module put it to sleep? Not never-live, not rigid already, with
    /// mass, no frozen or pinned node, and no joint node (a joint's two sides must stay on two
    /// bodies, and a moving joint wakes it anyway).
    pub fn sleep_eligible(&self) -> Vec<bool> {
        let segs = self.seg_parent.len();
        let mut ok: Vec<bool> = (0..segs)
            .map(|s| self.seg_flags[s] & SEG_NEVER_LIVE == 0 && self.seg_body[s] == NO_BODY)
            .collect();
        let mut mass = vec![false; segs];
        for i in 0..self.pos.len() {
            let s = self.node_segment[i];
            if s == NO_SEGMENT || self.node_flags[i] & NODE_ALIVE == 0 {
                continue;
            }
            if self.inv_mass[i] == 0.0 || self.node_flags[i] & NODE_FROZEN != 0 {
                ok[s as usize] = false;
            } else {
                mass[s as usize] = true;
            }
        }
        for j in 0..self.joint_nodes.len() {
            for &n in &self.joint_nodes[j] {
                let s = self.node_segment[n as usize];
                if s != NO_SEGMENT {
                    ok[s as usize] = false;
                }
            }
        }
        for s in 0..segs {
            ok[s] = ok[s] && mass[s];
        }
        ok
    }

    /// A movable node's velocity: a rider's from its body.
    fn sleep_node_vel(&self, i: usize) -> Vec3 {
        let b = self.node_body[i];
        if b == NO_BODY {
            self.vel[i]
        } else {
            let b = b as usize;
            self.body_vel[b] + self.body_omega[b].cross(self.pos[i] - self.body_centre[b])
        }
    }

    /// Momentum, angular momentum about the origin and kinetic energy of every live node with mass.
    fn sleep_totals(&self) -> (Vec3, Vec3, Real) {
        let (mut p, mut l, mut e) = (Vec3::ZERO, Vec3::ZERO, 0.0 as Real);
        for i in 0..self.pos.len() {
            if self.node_flags[i] & NODE_ALIVE == 0 || self.inv_mass[i] == 0.0 {
                continue;
            }
            let m = 1.0 / self.inv_mass[i];
            let v = self.sleep_node_vel(i);
            p = p + v * m;
            l = l + self.pos[i].cross(v * m);
            e += 0.5 * m * v.length_squared();
        }
        (p, l, e)
    }

    /// Marks `s` and its tree neighbours with `reason` where nothing is marked yet.
    fn sleep_spread(&self, s: usize, reason: u32, out: &mut [u32]) {
        if out[s] == 0 {
            out[s] = reason;
        }
        for k in 0..self.seam_a.len() {
            let (a, b) = (self.seam_a[k] as usize, self.seam_b[k] as usize);
            let other = if a == s {
                b
            } else if b == s {
                a
            } else {
                continue;
            };
            if out[other] == 0 {
                out[other] = WAKE_NEIGHBOUR;
            }
        }
    }

    /// Wakes every sleeping segment with a reason in `wake`, then re-forms the bodies. One pass,
    /// one check record, one event per woken segment in segment order.
    fn sleep_wake_pass(&mut self, wake: &[u32], step: u64) {
        let list: Vec<u32> = (0..wake.len())
            .filter(|&s| wake[s] != 0 && self.sleep.asleep[s] != 0)
            .map(|s| s as u32)
            .collect();
        if list.is_empty() {
            return;
        }
        let (p0, l0, e0) = self.sleep_totals();
        self.release(&list);
        for &s in &list {
            let u = s as usize;
            self.sleep.asleep[u] = 0;
            self.sleep.live_steps[u] = 0;
            self.sleep.quiet_steps[u] = 0;
            self.sleep.damp_left[u] = WAKE_DAMP_STEPS;
            self.events.push(Event { kind: EVENT_WAKE, index: s, step, value: wake[u] as Real });
        }
        let none = vec![false; wake.len()];
        self.sleep_form_bodies(&none);
        self.sleep_settle_all();
        let (p1, l1, e1) = self.sleep_totals();
        self.sleep.checks.push(SwitchCheck {
            kind: 1.0,
            momentum_before: p0,
            spin_before: l0,
            energy_before: e0,
            momentum_after: p1,
            spin_after: l1,
            energy_after: e1,
        });
    }

    /// Every sleeping segment starts a settle window: its bodies were just re-formed.
    fn sleep_settle_all(&mut self) {
        let sl = &mut self.sleep;
        for s in 0..sl.asleep.len() {
            if sl.asleep[s] != 0 {
                sl.settle_left[s] = SETTLE_STEPS;
            }
        }
    }

    /// The union-find: the rigid segments (and `extra`), joined by unbroken beams, become one body
    /// per connected group. Returns nothing; read `seg_body` for the outcome.
    fn sleep_form_bodies(&mut self, extra: &[bool]) {
        let segs = self.seg_parent.len();
        let rigid: Vec<bool> = (0..segs).map(|s| self.seg_body[s] != NO_BODY || extra[s]).collect();
        let mut root: Vec<u32> = (0..segs as u32).collect();
        fn find(root: &mut [u32], mut s: u32) -> u32 {
            while root[s as usize] != s {
                let up = root[root[s as usize] as usize];
                root[s as usize] = up;
                s = up;
            }
            s
        }
        for k in 0..self.beam_a.len() {
            if self.beam_flags[k] & BEAM_ALIVE == 0 || self.beam_flags[k] & BEAM_BROKEN != 0 {
                continue;
            }
            let (sa, sb) = (
                self.node_segment[self.beam_a[k] as usize],
                self.node_segment[self.beam_b[k] as usize],
            );
            if sa == NO_SEGMENT || sb == NO_SEGMENT || sa == sb || !rigid[sa as usize] || !rigid[sb as usize] {
                continue;
            }
            let (ra, rb) = (find(&mut root, sa), find(&mut root, sb));
            if ra != rb {
                // The lower index is the root, so a group's root is its lowest segment.
                let (lo, hi) = if ra < rb { (ra, rb) } else { (rb, ra) };
                root[hi as usize] = lo;
            }
        }
        let group: Vec<u32> = (0..segs as u32).map(|s| find(&mut root, s)).collect();
        // A body whose segments fall in more than one group goes loose whole.
        let mut split = vec![false; segs];
        let mut first_group = vec![u32::MAX; segs];
        for s in 0..segs {
            let b = self.seg_body[s];
            if b == NO_BODY {
                continue;
            }
            let b = b as usize;
            if first_group[b] == u32::MAX {
                first_group[b] = group[s];
            } else if first_group[b] != group[s] {
                split[b] = true;
            }
        }
        let had: Vec<u32> = self.seg_body.clone();
        let release: Vec<u32> = (0..segs as u32)
            .filter(|&s| had[s as usize] != NO_BODY && split[had[s as usize] as usize])
            .collect();
        if !release.is_empty() {
            self.release(&release);
        }
        // Each group that is not already exactly one body is made one, lowest group first.
        for g in 0..segs {
            if !rigid[g] || group[g] != g as u32 {
                continue;
            }
            let members: Vec<u32> = (0..segs as u32).filter(|&s| group[s as usize] == g as u32).collect();
            let b0 = self.seg_body[members[0] as usize];
            let one_body = b0 != NO_BODY
                && members.iter().all(|&s| self.seg_body[s as usize] == b0)
                && (0..segs).all(|s| self.seg_body[s] != b0 || group[s] == g as u32);
            if one_body {
                continue;
            }
            if self.make_rigid(&members).is_none() {
                // A degenerate group: what was rigid before goes back to being rigid on its own,
                // and the segments that were to sleep stay live.
                for &s in &members {
                    if had[s as usize] != NO_BODY && self.seg_body[s as usize] == NO_BODY {
                        let _ = self.make_rigid(&[s]);
                    }
                }
            }
        }
    }

    /// Phase 5.8, at the start of a step: the before-the-solve wakes. No-op unless sleeping was
    /// ever switched on.
    pub(crate) fn sleep_before_solve(&mut self, step: u64) {
        if !self.sleep.ever {
            return;
        }
        self.sleep.checks.clear();
        self.sleep_size();
        let segs = self.seg_parent.len();
        // A segment a command released is not asleep any more.
        for s in 0..segs {
            if self.sleep.asleep[s] != 0 && self.seg_body[s] == NO_BODY {
                self.sleep.asleep[s] = 0;
                self.sleep.live_steps[s] = 0;
                self.sleep.quiet_steps[s] = 0;
            }
        }
        let mut wake = vec![0u32; segs];
        if !self.sleep.on {
            for s in 0..segs {
                if self.sleep.asleep[s] != 0 {
                    wake[s] = WAKE_OFF;
                }
            }
            self.sleep.primed = false;
            self.sleep_wake_pass(&wake, step);
            return;
        }
        self.est.on = true;
        // Contacts: the same range test as `contact::find_contacts`, from the step's start.
        if let Some(terrain) = &self.terrain {
            let dt = STEP_SECONDS;
            let mut trig: Vec<(usize, u32)> = Vec::new();
            for i in 0..self.pos.len() {
                let s = self.node_segment[i];
                if s == NO_SEGMENT || self.node_flags[i] & NODE_ALIVE == 0 || self.inv_mass[i] == 0.0 {
                    continue;
                }
                let p = self.pos[i];
                let v = self.sleep_node_vel(i);
                let plane = terrain.plane_at(p.x, p.z);
                let gap = plane.distance(p) - self.radius[i];
                let margin = v.length() * dt + 0.5 * gravity_at(p).length() * dt * dt + crate::contact::CONTACT_SKIN;
                let near = gap < margin;
                if self.sleep.primed && near {
                    if !self.sleep.near[i] {
                        trig.push((s as usize, WAKE_CONTACT));
                    } else if -v.dot(plane.normal) > CONTACT_WAKE_SPEED {
                        trig.push((s as usize, WAKE_IMPACT));
                    }
                }
                self.sleep.near[i] = near;
            }
            for (s, r) in trig {
                self.sleep_spread(s, r, &mut wake);
            }
        }
        self.sleep.primed = true;
        // Thrust changes.
        for t in 0..self.thr_throttle.len() {
            let now = self.thr_throttle[t];
            if now != self.sleep.throttle[t] {
                for n in [self.thr_tail[t], self.thr_head[t]] {
                    let s = self.node_segment[n as usize];
                    if s != NO_SEGMENT {
                        self.sleep_spread(s as usize, WAKE_THRUST, &mut wake);
                    }
                }
            }
            self.sleep.throttle[t] = now;
        }
        self.sleep_wake_pass(&wake, step);
    }

    /// Phase 5.8, after the estimate: readings, the after-the-step wakes, then sleeps.
    pub(crate) fn sleep_after_step(&mut self, step: u64) {
        if !self.sleep.on {
            return;
        }
        self.sleep_size();
        let segs = self.seg_parent.len();
        // Readings: measured on live segments, calibrated estimate on sleeping ones, the estimate
        // with margin on never-live rigid ones.
        let mut measured = vec![0.0 as Real; segs];
        for k in 0..self.beam_a.len() {
            if self.beam_flags[k] & BEAM_ALIVE == 0 || self.beam_flags[k] & BEAM_BROKEN != 0 {
                continue;
            }
            let (a, b) = (self.beam_a[k] as usize, self.beam_b[k] as usize);
            if self.node_body[a] != NO_BODY && self.node_body[a] == self.node_body[b] {
                continue;
            }
            let strain = if self.yield_strain[k] > 0.0 { self.yield_strain[k] } else { self.break_strain[k] };
            let c = self.compliance[k];
            if !(strain > 0.0) || !(c > 0.0) {
                continue;
            }
            let r = self.last_lambda[k].abs() * c / (strain * self.rest0[k]);
            for s in [self.node_segment[a], self.node_segment[b]] {
                if s != NO_SEGMENT && r > measured[s as usize] {
                    measured[s as usize] = r;
                }
            }
        }
        let mut reading = vec![0.0 as Real; segs];
        for s in 0..segs {
            let e = if s < self.est.seg_ratio.len() { self.est.seg_ratio[s] } else { 0.0 };
            reading[s] = if self.seg_body[s] != NO_BODY && self.sleep.asleep[s] != 0 && self.sleep.settle_left[s] > 0 {
                self.sleep.settle_left[s] -= 1;
                self.sleep.ref_estimate[s] = e;
                self.sleep.ref_measured[s]
            } else if self.seg_body[s] != NO_BODY && self.sleep.asleep[s] != 0 {
                let r = self.sleep.ref_measured[s] + ESTIMATE_MARGIN * (e - self.sleep.ref_estimate[s]);
                if r > 0.0 { r } else { 0.0 }
            } else if self.seg_body[s] != NO_BODY {
                e * ESTIMATE_MARGIN
            } else {
                measured[s]
            };
        }
        let mut wake = vec![0u32; segs];
        for s in 0..segs {
            if s < self.seg_triggered.len() && self.seg_triggered[s] {
                self.sleep_spread(s, WAKE_JOINT, &mut wake);
            }
        }
        for s in 0..segs {
            if reading[s] >= WAKE_LEVEL {
                self.sleep_spread(s, WAKE_ESTIMATE, &mut wake);
            }
        }
        // Counters for the step just taken.
        for s in 0..segs {
            let sl = &mut self.sleep;
            if sl.damp_left[s] > 0 {
                sl.damp_left[s] -= 1;
            }
            if self.seg_body[s] == NO_BODY {
                sl.live_steps[s] = sl.live_steps[s].saturating_add(1);
                let change = reading[s] - sl.ratio[s];
                let quiet = reading[s] < SLEEP_LEVEL && change <= QUIET_CHANGE && change >= -QUIET_CHANGE;
                sl.quiet_steps[s] = if quiet { sl.quiet_steps[s].saturating_add(1) } else { 0 };
            }
            sl.ratio[s] = reading[s];
        }
        self.sleep_wake_pass(&wake, step);
        // Sleeps.
        let ok = self.sleep_eligible();
        let sleepers: Vec<bool> = (0..segs)
            .map(|s| {
                ok[s]
                    && wake[s] == 0
                    && self.sleep.live_steps[s] >= MIN_LIVE_STEPS
                    && self.sleep.quiet_steps[s] >= QUIET_STEPS
            })
            .collect();
        if !sleepers.iter().any(|&x| x) {
            return;
        }
        let (p0, l0, e0) = self.sleep_totals();
        self.sleep_form_bodies(&sleepers);
        for s in 0..segs {
            if !sleepers[s] {
                continue;
            }
            if self.seg_body[s] != NO_BODY {
                self.sleep.asleep[s] = 1;
                self.sleep.ref_measured[s] = reading[s];
                self.events.push(Event { kind: EVENT_SLEEP, index: s as u32, step, value: reading[s] });
            } else {
                // A degenerate group: try again after another full quiet run.
                self.sleep.quiet_steps[s] = 0;
            }
        }
        self.sleep_settle_all();
        let (p1, l1, e1) = self.sleep_totals();
        self.sleep.checks.push(SwitchCheck {
            kind: 2.0,
            momentum_before: p0,
            spin_before: l0,
            energy_before: e0,
            momentum_after: p1,
            spin_after: l1,
            energy_after: e1,
        });
    }
}
