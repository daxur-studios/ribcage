//! Rigid bodies (plan 005, phase 5.2): a group of segments moving as one solid, with a centre,
//! a rotation, a linear and an angular velocity, a mass and an inertia tensor in place of its
//! nodes. The nodes stay in the world arrays and *ride* the body: once per step their position
//! is written back as `centre + R · offset` and their velocity as `V + ω × (x − centre)`, so
//! everything that reads nodes (rendering, the hash, the energy count) keeps working.
//!
//! **Storage** (layout rule 3, flat arrays on [`World`]). A body lives in a *slot*, numbered
//! as the lowest segment of the group, so slots never collide and the arrays are sized by the
//! segment count (they grow when the first body is made). Per slot: flags, centre, rotation
//! (unit quaternion, body frame to world), linear and angular velocity (world frame), mass,
//! inertia and inverse inertia in the body frame (full symmetric 3 by 3), and the pose the
//! last substep predicted (`body_prev_*`, what the velocity is derived against). Per segment:
//! `seg_body`, so a body's member segments are the segments with its slot, in ascending order
//! by construction. Per node: `node_body` and `node_offset` (body frame).
//!
//! **Switching** (the plan's formulas). [`World::make_rigid`] sums the loose nodes and any
//! bodies the group touches: `M = Σm`, `c = Σm x / M`, `P = Σm v`, `L = Σ m (x − c) × v`,
//! `I = Σ m (|r|² 1 − r rᵀ)`, a body entering as its own mass, momentum, spin and inertia
//! moved to the new centre; then `V = P / M`, `ω = I⁻¹ L`. No position moves, momentum and
//! angular momentum are kept, and kinetic energy can only fall (the part not in rigid motion
//! is dropped). [`World::release`] gives each released node `V + ω × r` and recomputes the
//! rest with the same `ω` and `V' = V + ω × (c' − c)`: momentum, spin and energy unchanged.
//!
//! **Integration** (per substep). Gravity and the step's external force push the centre;
//! the external torque and the spin term turn the body ([`SpinForm`]); the rotation is
//! updated with the Cayley map (no sine or cosine). The spin is clamped at
//! [`World::max_angular_speed`]. Velocities are then derived against the predicted pose, which
//! in 5.2 nothing moves (a body touches only itself; beams to bodies come in 5.3).

use crate::event::{Event, EVENT_FREEZE};
use crate::maths::{Mat3, Quat, Sym3};
use crate::real::{Real, Vec3};
use crate::sim::gravity_at;
use crate::solver::MIN_SUBSTEPS;
use crate::world::{World, NODE_ALIVE, NODE_FROZEN};
use crate::STEP_SECONDS;

/// No body: a loose node, a segment that is not rigid.
pub const NO_BODY: u32 = u32::MAX;

/// Body flag: the slot holds a live body.
pub const BODY_ALIVE: u32 = 1;

/// **[assumption]** Largest angular speed a body keeps, rad/s: about 950 rpm, well above any
/// tumble a craft or its debris shows, and `h ω` stays below 0.42 at the four-substep minimum.
pub const DEFAULT_MAX_ANGULAR_SPEED: Real = 100.0;

/// A group whose inertia has `det(I) ≤ DEGENERATE · (trace / 3)³` (a rod, a single node, a
/// line of nodes) is not made rigid: its inverse inertia does not exist or is unbounded.
pub const DEGENERATE: Real = 1e-9;

/// Newton iterations of the midpoint spin solve. Fixed, so the work and the bits are fixed.
pub const SPIN_NEWTON_ITERATIONS: u32 = 3;

/// How the gyroscopic term `−ω × L` of a body's spin is integrated (phase 5.2 measures all
/// three; ER 005).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SpinForm {
    /// Forward Euler on the body-frame momentum, `Π' = Π − h ω × Π`. Always gains energy.
    Explicit,
    /// Backward Euler with one Newton step (the common game-engine "implicit gyroscopic"
    /// form). Always loses energy and angular momentum.
    Implicit,
    /// Implicit midpoint solved by Newton, then the momentum rotated by the same Cayley
    /// rotation that turns the body: keeps `|L|` and the world `L` to rounding and the energy to
    /// the Newton residual. The default.
    Midpoint,
}

/// A body's step-start state, for the bad-number restore.
struct Start {
    centre: Vec3,
    rot: Quat,
}

impl World {
    /// Body slots, live or not (zero until the first body is made).
    pub fn body_slot_count(&self) -> u32 {
        self.body_flags.len() as u32
    }

    pub fn body_alive(&self, slot: u32) -> bool {
        (slot as usize) < self.body_flags.len() && self.body_flags[slot as usize] & BODY_ALIVE != 0
    }

    /// Live bodies.
    pub fn body_count(&self) -> u32 {
        self.body_flags.iter().filter(|&&f| f & BODY_ALIVE != 0).count() as u32
    }

    /// The slot of a segment's body, or [`NO_BODY`].
    pub fn segment_body(&self, segment: u32) -> u32 {
        self.seg_body.get(segment as usize).copied().unwrap_or(NO_BODY)
    }

    /// A body's inertia tensor in the world frame, `R I_b Rᵀ`.
    pub fn body_world_inertia(&self, slot: u32) -> Sym3 {
        let s = slot as usize;
        self.body_inertia[s].rotated(&self.body_rot[s].to_mat3())
    }

    /// A body's angular momentum about its own centre, world frame.
    pub fn body_spin(&self, slot: u32) -> Vec3 {
        self.body_world_inertia(slot).mul_vec(self.body_omega[slot as usize])
    }

    fn grow_body_slots(&mut self) {
        let n = self.seg_body.len();
        if self.body_flags.len() < n {
            self.body_flags.resize(n, 0);
            self.body_centre.resize(n, Vec3::ZERO);
            self.body_rot.resize(n, Quat::IDENTITY);
            self.body_vel.resize(n, Vec3::ZERO);
            self.body_omega.resize(n, Vec3::ZERO);
            self.body_mass.resize(n, 0.0);
            self.body_inertia.resize(n, Sym3::ZERO);
            self.body_inv_inertia.resize(n, Sym3::ZERO);
            self.body_prev_centre.resize(n, Vec3::ZERO);
            self.body_prev_rot.resize(n, Quat::IDENTITY);
        }
    }

    /// A node that counts toward a group: live and with mass. Riding or loose.
    fn massive(&self, i: usize) -> bool {
        self.node_flags[i] & NODE_ALIVE != 0 && self.inv_mass[i] != 0.0
    }

    /// Makes the listed segments, and every segment of any body they touch, one rigid body
    /// (sleep, the plan's formulas). Returns its slot (the group's lowest segment), or `None`
    /// and changes nothing when a segment does not exist, the group has no mass, or its inertia
    /// is degenerate ([`DEGENERATE`]). No node moves.
    pub fn make_rigid(&mut self, segments: &[u32]) -> Option<u32> {
        let ns = self.seg_body.len();
        if segments.is_empty() || segments.iter().any(|&s| s as usize >= ns) {
            return None;
        }
        // The group: the listed segments, then every segment of a body they touch.
        let mut in_group = vec![false; ns];
        let mut touched = vec![false; ns];
        for &s in segments {
            in_group[s as usize] = true;
            if self.seg_body[s as usize] != NO_BODY {
                touched[self.seg_body[s as usize] as usize] = true;
            }
        }
        for s in 0..ns {
            let b = self.seg_body[s];
            if b != NO_BODY && touched[b as usize] {
                in_group[s] = true;
            }
        }
        let in_group_node = |w: &World, i: usize| {
            let s = w.node_segment[i];
            s != crate::segment::NO_SEGMENT && in_group[s as usize]
        };
        // First pass: mass, centre and momentum. Loose nodes in index order, then bodies in
        // slot order.
        let n = self.pos.len();
        let (mut mass, mut moment, mut momentum) = (0.0 as Real, Vec3::ZERO, Vec3::ZERO);
        for i in 0..n {
            if self.massive(i) && self.node_body[i] == NO_BODY && in_group_node(self, i) {
                let m = 1.0 / self.inv_mass[i];
                mass += m;
                moment = moment + self.pos[i] * m;
                momentum = momentum + self.vel[i] * m;
            }
        }
        for b in 0..ns {
            if touched[b] {
                let m = self.body_mass[b];
                mass += m;
                moment = moment + self.body_centre[b] * m;
                momentum = momentum + self.body_vel[b] * m;
            }
        }
        if !(mass > 0.0) {
            return None;
        }
        let centre = moment / mass;
        // Second pass: inertia and angular momentum about the new centre.
        let (mut inertia, mut spin) = (Sym3::ZERO, Vec3::ZERO);
        for i in 0..n {
            if self.massive(i) && self.node_body[i] == NO_BODY && in_group_node(self, i) {
                let m = 1.0 / self.inv_mass[i];
                let r = self.pos[i] - centre;
                inertia = inertia.add(Sym3::point(m, r));
                spin = spin + r.cross(self.vel[i] * m);
            }
        }
        for b in 0..ns {
            if touched[b] {
                let m = self.body_mass[b];
                let d = self.body_centre[b] - centre;
                inertia = inertia.add(self.body_world_inertia(b as u32)).add(Sym3::point(m, d));
                spin = spin + self.body_spin(b as u32) + d.cross(self.body_vel[b] * m);
            }
        }
        let inv = degenerate_or_inverse(inertia)?;
        let vel = momentum / mass;
        let omega = inv.mul_vec(spin);
        if !(centre.is_finite() && vel.is_finite() && omega.is_finite()) {
            return None;
        }
        // Commit: the touched bodies end, the group becomes one body in its lowest slot.
        self.grow_body_slots();
        let slot = (0..ns).find(|&s| in_group[s]).expect("group is not empty") as u32;
        for b in 0..ns {
            if touched[b] {
                self.body_flags[b] = 0;
            }
        }
        for s in 0..ns {
            if in_group[s] {
                self.seg_body[s] = slot;
            }
        }
        for i in 0..n {
            if self.massive(i) && in_group_node(self, i) {
                let r = self.pos[i] - centre;
                self.node_body[i] = slot;
                self.node_offset[i] = r;
                self.vel[i] = vel + omega.cross(r);
            }
        }
        let s = slot as usize;
        self.body_flags[s] = BODY_ALIVE;
        self.body_centre[s] = centre;
        self.body_rot[s] = Quat::IDENTITY;
        self.body_vel[s] = vel;
        self.body_omega[s] = omega;
        self.body_mass[s] = mass;
        self.body_inertia[s] = inertia;
        self.body_inv_inertia[s] = inv;
        self.body_prev_centre[s] = centre;
        self.body_prev_rot[s] = Quat::IDENTITY;
        Some(slot)
    }

    /// Releases the listed segments from their bodies (wake, the plan's formulas). Each released
    /// node takes `V + ω × r`; what stays rigid keeps `ω` and moves its centre ([`World::refresh_body`]).
    /// Segments not in a body are ignored. `false` and nothing changes when a segment does not
    /// exist. No node moves.
    pub fn release(&mut self, segments: &[u32]) -> bool {
        let ns = self.seg_body.len();
        if segments.iter().any(|&s| s as usize >= ns) {
            return false;
        }
        let mut leaving = vec![false; ns];
        let mut touched = vec![false; ns];
        for &s in segments {
            let b = self.seg_body[s as usize];
            if b != NO_BODY {
                leaving[s as usize] = true;
                touched[b as usize] = true;
            }
        }
        for i in 0..self.pos.len() {
            let b = self.node_body[i];
            if b != NO_BODY && touched[b as usize] && leaving[self.node_segment[i] as usize] {
                self.detach_node(i);
            }
        }
        for s in 0..ns {
            if leaving[s] {
                self.seg_body[s] = NO_BODY;
            }
        }
        for b in 0..ns {
            if touched[b] {
                self.refresh_body(b as u32);
            }
        }
        true
    }

    /// The node leaves its body with the body's velocity at its position.
    fn detach_node(&mut self, i: usize) {
        let b = self.node_body[i] as usize;
        let r = self.pos[i] - self.body_centre[b];
        self.vel[i] = self.body_vel[b] + self.body_omega[b].cross(r);
        self.node_body[i] = NO_BODY;
        self.node_offset[i] = Vec3::ZERO;
    }

    /// Recomputes a body from the nodes still riding it, after some left: mass, centre and
    /// inertia from their positions, the same `ω`, and `V' = V + ω × (c' − c)`, so the velocity
    /// of every point is unchanged. The rotation restarts at identity (offsets are re-taken in
    /// the world frame). If the rest has no mass or a degenerate inertia it is released whole.
    /// If the body's lowest segment left, the body moves to its new lowest segment's slot.
    pub(crate) fn refresh_body(&mut self, slot: u32) {
        let s = slot as usize;
        if !self.body_alive(slot) {
            return;
        }
        let n = self.pos.len();
        let (mut mass, mut moment) = (0.0 as Real, Vec3::ZERO);
        for i in 0..n {
            if self.node_body[i] == slot {
                let m = 1.0 / self.inv_mass[i];
                mass += m;
                moment = moment + self.pos[i] * m;
            }
        }
        let fallback = || -> Option<(Vec3, Sym3, Sym3)> { None };
        let mut result = fallback();
        if mass > 0.0 {
            let centre = moment / mass;
            let mut inertia = Sym3::ZERO;
            for i in 0..n {
                if self.node_body[i] == slot {
                    inertia = inertia.add(Sym3::point(1.0 / self.inv_mass[i], self.pos[i] - centre));
                }
            }
            if let Some(inv) = degenerate_or_inverse(inertia) {
                result = Some((centre, inertia, inv));
            }
        }
        let Some((centre, inertia, inv)) = result else {
            // Nothing rigid is left: every rider goes loose with its rigid velocity.
            for i in 0..n {
                if self.node_body[i] == slot {
                    self.detach_node(i);
                }
            }
            for b in self.seg_body.iter_mut() {
                if *b == slot {
                    *b = NO_BODY;
                }
            }
            self.body_flags[s] = 0;
            return;
        };
        let omega = self.body_omega[s];
        let vel = self.body_vel[s] + omega.cross(centre - self.body_centre[s]);
        let new_slot = self.seg_body.iter().position(|&b| b == slot).expect("a live body has a segment") as u32;
        if new_slot != slot {
            for b in self.seg_body.iter_mut() {
                if *b == slot {
                    *b = new_slot;
                }
            }
            self.body_flags[s] = 0;
        }
        for i in 0..n {
            if self.node_body[i] == slot {
                self.node_body[i] = new_slot;
                self.node_offset[i] = self.pos[i] - centre;
            }
        }
        let t = new_slot as usize;
        self.body_flags[t] = BODY_ALIVE;
        self.body_centre[t] = centre;
        self.body_rot[t] = Quat::IDENTITY;
        self.body_vel[t] = vel;
        self.body_omega[t] = omega;
        self.body_mass[t] = mass;
        self.body_inertia[t] = inertia;
        self.body_inv_inertia[t] = inv;
        self.body_prev_centre[t] = centre;
        self.body_prev_rot[t] = Quat::IDENTITY;
    }

    /// Called by `delete_node` after the node's flags are cleared: a rider leaves its body.
    pub(crate) fn body_node_left(&mut self, node: u32) {
        let i = node as usize;
        let b = self.node_body[i];
        if b != NO_BODY {
            self.node_body[i] = NO_BODY;
            self.node_offset[i] = Vec3::ZERO;
            self.refresh_body(b);
        }
    }

    /// Called by `set_inv_mass`: a rider leaves with its rigid velocity and the group is made
    /// rigid again with the node at its new mass (a pinned node stays out). A loose node with
    /// mass in a rigid segment joins the body the same way.
    pub(crate) fn body_node_changed(&mut self, node: u32) {
        let i = node as usize;
        let seg = self.node_segment[i];
        if seg == crate::segment::NO_SEGMENT || self.seg_body[seg as usize] == NO_BODY {
            return;
        }
        let slot = self.seg_body[seg as usize];
        let group: Vec<u32> = (0..self.seg_body.len() as u32).filter(|&s| self.seg_body[s as usize] == slot).collect();
        if self.node_body[i] != NO_BODY {
            self.detach_node(i);
            self.refresh_body(slot);
        }
        self.make_rigid(&group);
    }

    /// Integrates every live body over one step and writes its riders back. Called by the step
    /// after the islands are copied back and before the forces are cleared. `fixed` is the
    /// substep count when the caller fixed one; otherwise a body takes the rule's minimum (a
    /// body alone has no beam to resolve). Bodies go in slot order; a body with a frozen rider
    /// is skipped.
    pub(crate) fn step_bodies(&mut self, fixed: Option<u32>, done: u64) {
        let slots = self.body_flags.len();
        if slots == 0 || !self.body_flags.iter().any(|&f| f & BODY_ALIVE != 0) {
            return;
        }
        // External force and torque per body, from the riders, in node order.
        let mut force = vec![Vec3::ZERO; slots];
        let mut torque = vec![Vec3::ZERO; slots];
        let mut frozen = vec![false; slots];
        let mut first_rider = vec![u32::MAX; slots];
        for i in 0..self.pos.len() {
            let b = self.node_body[i];
            if b == NO_BODY {
                continue;
            }
            let b = b as usize;
            let f = self.force[i];
            force[b] = force[b] + f;
            torque[b] = torque[b] + (self.pos[i] - self.body_centre[b]).cross(f);
            if self.node_flags[i] & NODE_FROZEN != 0 {
                frozen[b] = true;
            }
            if first_rider[b] == u32::MAX {
                first_rider[b] = i as u32;
            }
        }
        let n = fixed.unwrap_or(MIN_SUBSTEPS).max(1);
        let h = STEP_SECONDS / n as Real;
        let mut bad = vec![0u32; slots];
        for b in 0..slots {
            if self.body_flags[b] & BODY_ALIVE == 0 || frozen[b] {
                continue;
            }
            let start = Start { centre: self.body_centre[b], rot: self.body_rot[b] };
            for substep in 1..=n {
                self.predict_body(b, h, force[b], torque[b]);
                self.derive_body(b, h);
                if !self.body_finite(b) {
                    bad[b] = substep;
                    break;
                }
            }
            if bad[b] != 0 {
                self.body_centre[b] = start.centre;
                self.body_rot[b] = start.rot;
                self.body_prev_centre[b] = start.centre;
                self.body_prev_rot[b] = start.rot;
                self.body_vel[b] = Vec3::ZERO;
                self.body_omega[b] = Vec3::ZERO;
                self.events.push(Event { kind: EVENT_FREEZE, index: first_rider[b], step: done, value: bad[b] as Real });
            }
        }
        // Riders follow, once per step, in node order.
        for i in 0..self.pos.len() {
            let b = self.node_body[i];
            if b == NO_BODY || frozen[b as usize] {
                continue;
            }
            let b = b as usize;
            if bad[b] != 0 {
                self.node_flags[i] |= NODE_FROZEN;
            }
            let p = self.body_centre[b] + self.body_rot[b].rotate(self.node_offset[i]);
            self.pos[i] = p;
            self.vel[i] = self.body_vel[b] + self.body_omega[b].cross(p - self.body_centre[b]);
        }
    }

    fn body_finite(&self, b: usize) -> bool {
        self.body_centre[b].is_finite()
            && self.body_rot[b].is_finite()
            && self.body_vel[b].is_finite()
            && self.body_omega[b].is_finite()
    }

    /// One substep's prediction: the linear kick and drift, the torque kick, the spin term and
    /// the rotation update, and the spin clamp. Stores the predicted pose in `body_prev_*`.
    pub(crate) fn predict_body(&mut self, b: usize, h: Real, force: Vec3, torque: Vec3) {
        let mass = self.body_mass[b];
        let c = self.body_centre[b];
        let vel = self.body_vel[b] + (gravity_at(c) + force / mass) * h;
        let centre = c + vel * h;
        let q = self.body_rot[b];
        let (inertia, inv) = (self.body_inertia[b], self.body_inv_inertia[b]);
        // Body-frame momentum after the torque kick.
        let pi0 = inertia.mul_vec(q.inv_rotate(self.body_omega[b])) + q.inv_rotate(torque) * h;
        let (q1, pi1) = match self.spin_form {
            SpinForm::Explicit => {
                let w0 = inv.mul_vec(pi0);
                let pi1 = pi0 - w0.cross(pi0) * h;
                (rotate_by_rate(q, inv.mul_vec(pi1), h), pi1)
            }
            SpinForm::Implicit => {
                let w0 = inv.mul_vec(pi0);
                let iw = inertia.mul_vec(w0);
                // J = I + h (skew(ω) I − skew(I ω)), Δω = −J⁻¹ h ω × I ω.
                let j = inertia.to_mat3().add(&Mat3::skew(w0).mul(&inertia.to_mat3()).sub(&Mat3::skew(iw)).scale(h));
                let w1 = match j.solve(w0.cross(iw) * h) {
                    Some(d) => w0 - d,
                    None => w0,
                };
                (rotate_by_rate(q, w1, h), inertia.mul_vec(w1))
            }
            SpinForm::Midpoint => midpoint_spin(q, pi0, inv, h),
        };
        // The clamp scales the spin, so momentum and rate stay in proportion.
        let mut omega_b = inv.mul_vec(pi1);
        let speed = omega_b.length();
        if speed > self.max_angular_speed {
            omega_b = omega_b * (self.max_angular_speed / speed);
        }
        self.body_vel[b] = vel;
        self.body_centre[b] = centre;
        self.body_rot[b] = q1;
        self.body_omega[b] = q1.rotate(omega_b);
        self.body_prev_centre[b] = centre;
        self.body_prev_rot[b] = q1;
    }

    /// One substep's velocity update from what the constraints did to the predicted pose:
    /// `V += (c − c_pred) / h`, `ω += 2 vec(q ⊗ q_pred⁻¹) / h`. Exactly nothing when nothing
    /// moved the body (always, in 5.2). The linear speed is then clamped at `max_speed`.
    pub(crate) fn derive_body(&mut self, b: usize, h: Real) {
        let (c, q) = (self.body_centre[b], self.body_rot[b]);
        let (cp, qp) = (self.body_prev_centre[b], self.body_prev_rot[b]);
        if c != cp {
            self.body_vel[b] = self.body_vel[b] + (c - cp) / h;
        }
        if q != qp {
            let d = q.delta_vec(qp);
            let d = if q.delta_w(qp) < 0.0 { -d } else { d };
            self.body_omega[b] = self.body_omega[b] + d * (2.0 / h);
        }
        let speed = self.body_vel[b].length();
        if speed > self.max_speed {
            self.body_vel[b] = self.body_vel[b] * (self.max_speed / speed);
        }
    }
}

/// `q` turned by the body-frame rate `ω` for `h`: `normalize(q + ½ h q ⊗ (0, ω))`, the usual
/// first-order quaternion update (used by the explicit and implicit forms).
fn rotate_by_rate(q: Quat, w: Vec3, h: Real) -> Quat {
    let d = q.mul(Quat::new(0.0, w.x, w.y, w.z));
    let k = 0.5 * h;
    Quat::new(q.w + d.w * k, q.x + d.x * k, q.y + d.y * k, q.z + d.z * k).normalize()
}

/// The implicit midpoint rule for torque-free spin in the body frame,
/// `Π' = Π − h ω_m × Π_m` with `Π_m = (Π + Π') / 2`, `ω_m = I⁻¹ Π_m`, solved by
/// [`SPIN_NEWTON_ITERATIONS`] Newton steps from `Π' = Π`. That equation is exactly
/// `Π' = cay(−h ω_m) Π`, so the result is taken as that rotation of `Π`, and the body turns by
/// `cay(h ω_m)`: `|Π|` and the world momentum `R Π` are then kept to rounding whatever the
/// Newton residual, and the energy to that residual. Returns the new rotation and `Π'`.
fn midpoint_spin(q: Quat, pi0: Vec3, inv: Sym3, h: Real) -> (Quat, Vec3) {
    let inv_m = inv.to_mat3();
    let mut pi1 = pi0;
    for _ in 0..SPIN_NEWTON_ITERATIONS {
        let pm = (pi0 + pi1) * 0.5;
        let wm = inv.mul_vec(pm);
        let g = pi1 - pi0 + wm.cross(pm) * h;
        // dG/dΠ' = 1 + (h/2) (skew(ω_m) − skew(Π_m) I⁻¹).
        let j = Mat3::IDENTITY.add(&Mat3::skew(wm).sub(&Mat3::skew(pm).mul(&inv_m)).scale(0.5 * h));
        match j.solve(g) {
            Some(d) => pi1 = pi1 - d,
            None => break,
        }
    }
    let a = inv.mul_vec((pi0 + pi1) * 0.5) * h;
    let dq = Quat::cayley(a);
    (q.mul(dq).normalize(), dq.inv_rotate(pi0))
}

/// `I⁻¹`, or `None` when `I` is degenerate ([`DEGENERATE`]) or not finite.
fn degenerate_or_inverse(i: Sym3) -> Option<Sym3> {
    if !i.is_finite() {
        return None;
    }
    let t = i.trace() / 3.0;
    if !(t > 0.0) || !(i.det() > DEGENERATE * t * t * t) {
        return None;
    }
    i.inverse()
}
