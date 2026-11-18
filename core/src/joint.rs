//! Hinges, joint limits, servos and rotors (plan 005, phase 5.5).
//!
//! A joint is six nodes, three per side: two on the hinge axis and one reference node off it.
//! Side A is `[a0, a1, ra]`, side B is `[b0, b1, rb]`, stored in that order. Each node is whatever
//! it is at the start of the step: a loose node, a pinned node, or a node riding a rigid body, so
//! one joint type connects two bodies, a body and live nodes, or two live lattices, all through
//! the one kernel ([`crate::solver::Endpoint`], layout rule 6). The arrays are flat, one per field,
//! on the world (rule 3), addressed by a stable `u32` index (joints are only ever added).
//!
//! **Hinge.** Two zero-length point constraints, `a0` with `b0` and `a1` with `b1`, each a
//! [`crate::solver::solve_distance`] with rest length zero and zero compliance. They leave one
//! freedom: the turn about the axis.
//!
//! **Angle.** The axis is `u = (a1 − a0) / |a1 − a0|`. Each side's reference arm is its reference
//! node's offset from its first axis node with the part along `u` removed:
//! `p_A = (ra − a0)⊥`, `p_B = (rb − b0)⊥`. The raw angle is `atan2(u · (p_A × p_B), p_A · p_B)`, the
//! turn from A's arm to B's about `u` (the core's own [`crate::maths::atan2`]); the joint angle
//! `φ` is that minus the raw angle when the joint was added, wrapped to `(−π, π]`. So the build
//! pose is angle zero.
//!
//! **Angular correction (limits and drives).** `φ` depends only on the two arms, so its gradient
//! is a couple on each side: `+t_B / L_B` at `rb` and `−t_B / L_B` at `b0`, `−t_A / L_A` at `ra` and
//! `+t_A / L_A` at `a0`, with `t = u × p / |p|` and `L = |p|`. An XPBD step on `C` is
//! `Δλ = −C / ((w_ra + w_a0) / L_A² + (w_rb + w_b0) / L_B²)` (each `w` the kernel's weight of that
//! end along its tangent), applied as two calls of the kernel. The pushes on each side sum to zero
//! force, and the two sides' torques about the axis are equal and opposite, so a drive inside a
//! free assembly keeps its linear and angular momentum. `Δλ / h²` is the torque in newton-metres.
//!
//! **Limit.** One-sided and hard: only when `φ < min` (correct to `min`) or `φ > max` (to `max`),
//! uncapped, after the drive, so the limit wins. `−π ≤ min ≤ 0 ≤ max ≤ π`; `min = −π` and
//! `max = π` together mean no limit.
//!
//! **Servo.** Each substep it aims at `φ₀ + clamp(wrap(target − φ₀), ±rate · h)`, `φ₀` being the
//! angle at the substep's start, so the angle moves toward the target at no more than `rate` and
//! there is no wound-up error when it is held back. **Rotor**: the same on speed, aiming at
//! `φ₀ + speed · h`. Both corrections are capped: `|Δλ| ≤ τ_max · h²`, so the drive torque never
//! exceeds `τ_max` in any substep. A drive is a constraint with a force cap, not a spring, so it
//! stores nothing; stalled against a larger load it gives exactly `τ_max`, a constant torque about
//! a fixed axis, which does work only while the angle moves.
//!
//! **Trigger.** A joint whose angle changes by more than [`JOINT_MOVE_EPS`] over a step marks the
//! segments of `a0` and `b0` as triggered (`World::seg_triggered`), cleared at the start of each
//! step. Phase 5.8 reads it to wake sleeping segments; nothing here reads it.
//!
//! Only add, subtract, multiply, divide and square root on `Real`, plus the core's own `atan2`,
//! in ascending joint order (determinism list).

use crate::maths::{atan2, Quat, PI, TAU};
use crate::real::{Real, Vec3};
use crate::solver::{apply_correction, solve_distance, Endpoint, Mass, Pose};
use crate::world::World;

/// Joint drive mode: no drive (a free hinge, with its limit if it has one).
pub const JOINT_FREE: u32 = 0;
/// Joint drive mode: a servo, toward a target angle at a capped rate.
pub const JOINT_SERVO: u32 = 1;
/// Joint drive mode: a rotor, at a target angular speed.
pub const JOINT_ROTOR: u32 = 2;

/// A joint whose angle changes by more than this over one step, in radians, is moving and marks
/// its segments as triggered: 1e-4 rad per step is 0.006 rad/s at 60 Hz, a third of a degree a
/// second. **[assumption]** for phase 5.8 to tune.
pub const JOINT_MOVE_EPS: Real = 1.0e-4;

/// Wraps an angle in `(−3π, 3π)` to `(−π, π]`.
#[inline(always)]
pub fn wrap(x: Real) -> Real {
    if x > PI {
        x - TAU
    } else if x <= -PI {
        x + TAU
    } else {
        x
    }
}

/// What the angular correction needs: the angle and each side's tangent and arm length.
#[derive(Clone, Copy, Debug)]
pub struct JointGeom {
    /// `φ`, wrapped to `(−π, π]`, zero at the pose the joint was added in.
    pub angle: Real,
    pub tan_a: Vec3,
    pub tan_b: Vec3,
    pub lever_a: Real,
    pub lever_b: Real,
}

/// Raw angle and arms from six positions (`[a0, a1, ra, b0, b1, rb]`). `None` when the axis or an
/// arm has no length (a degenerate joint does nothing that step).
pub fn measure(p: [Vec3; 6], zero: Real) -> Option<JointGeom> {
    let axis = p[1] - p[0];
    let al = axis.length();
    if !(al > 0.0) {
        return None;
    }
    let u = axis / al;
    let da = p[2] - p[0];
    let pa = da - u * u.dot(da);
    let db = p[5] - p[3];
    let pb = db - u * u.dot(db);
    let (la, lb) = (pa.length(), pb.length());
    if !(la > 0.0 && lb > 0.0) {
        return None;
    }
    let raw = atan2(u.dot(pa.cross(pb)), pa.dot(pb));
    Some(JointGeom {
        angle: wrap(raw - zero),
        tan_a: u.cross(pa) / la,
        tan_b: u.cross(pb) / lb,
        lever_a: la,
        lever_b: lb,
    })
}

/// [`measure`] at the ends' current places.
#[inline(always)]
pub fn geometry(
    m: &Mass,
    pos: &[Vec3],
    centre: &[Vec3],
    rot: &[Quat],
    e: &[Endpoint; 6],
    zero: Real,
) -> Option<JointGeom> {
    measure(e.map(|x| x.place(m, pos, centre, rot)), zero)
}

/// One angular correction of `C` (radians) with `Δλ` clamped to `[lo, hi]`, through the kernel.
/// Returns `Δλ`.
fn turn_joint(
    s: &mut Pose,
    m: &Mass,
    e: &[Endpoint; 6],
    g: &JointGeom,
    c: Real,
    lo: Real,
    hi: Real,
) -> Real {
    let wa = e[2].weight(m, s.rot, g.tan_a) + e[0].weight(m, s.rot, g.tan_a);
    let wb = e[5].weight(m, s.rot, g.tan_b) + e[3].weight(m, s.rot, g.tan_b);
    let denom = wa / (g.lever_a * g.lever_a) + wb / (g.lever_b * g.lever_b);
    if !(denom > 0.0) {
        return 0.0;
    }
    let mut dl = -c / denom;
    if dl > hi {
        dl = hi;
    } else if dl < lo {
        dl = lo;
    }
    // Side B: rb along +t_B, b0 along −t_B. Side A: ra along −t_A, a0 along +t_A.
    apply_correction(s, m, e[3], e[5], g.tan_b, dl / g.lever_b);
    apply_correction(s, m, e[2], e[0], g.tan_a, dl / g.lever_a);
    dl
}

/// One joint's settings, as packed into an island.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointSpec {
    pub mode: u32,
    pub min: Real,
    pub max: Real,
    pub target: Real,
    pub rate: Real,
    pub cap: Real,
    pub zero: Real,
}

/// Passes over one joint per substep. One pass leaves a body side short: the angular push turns a
/// body about its centre, which moves its axis nodes off the axis, and the hinge points then pull
/// part of the turn back (a lattice side does not do this, its nodes being independent). A few
/// local passes converge it: see ER 005, phase 5.5, for the measured effect of each count.
pub const JOINT_PASSES: u32 = 8;

/// Solves one joint in a substep: [`JOINT_PASSES`] passes of the two hinge points, then the
/// drive, then the limit. The drive's corrections over the passes are summed and the sum is
/// clamped to `±cap · h²`, so its torque over the substep never exceeds the cap. `phi0` is the
/// angle at the substep's start (NaN if it could not be measured: no drive). Returns the drive's
/// torque in newton-metres (zero with no drive).
pub fn solve_joint(
    s: &mut Pose,
    m: &Mass,
    e: &[Endpoint; 6],
    j: &JointSpec,
    phi0: Real,
    h: Real,
) -> Real {
    let driven = j.mode != JOINT_FREE && j.cap > 0.0 && phi0.is_finite();
    let aim = if j.mode == JOINT_SERVO {
        let step = j.rate * h;
        let mut d = wrap(j.target - phi0);
        if d > step {
            d = step;
        } else if d < -step {
            d = -step;
        }
        phi0 + d
    } else {
        phi0 + j.target * h
    };
    let limited = j.min > -PI || j.max < PI;
    let cap = j.cap * (h * h);
    let mut total: Real = 0.0;
    for _ in 0..JOINT_PASSES {
        let mut rest: Real = 0.0;
        solve_distance(s, m, e[0], e[3], &mut rest, Real::INFINITY, 0.0, h);
        rest = 0.0;
        solve_distance(s, m, e[1], e[4], &mut rest, Real::INFINITY, 0.0, h);
        if driven {
            if let Some(g) = geometry(m, s.pos, s.centre, s.rot, e, j.zero) {
                total += turn_joint(s, m, e, &g, wrap(g.angle - aim), -cap - total, cap - total);
            }
        }
        if limited {
            if let Some(g) = geometry(m, s.pos, s.centre, s.rot, e, j.zero) {
                if g.angle < j.min {
                    turn_joint(
                        s,
                        m,
                        e,
                        &g,
                        g.angle - j.min,
                        Real::NEG_INFINITY,
                        Real::INFINITY,
                    );
                } else if g.angle > j.max {
                    turn_joint(
                        s,
                        m,
                        e,
                        &g,
                        g.angle - j.max,
                        Real::NEG_INFINITY,
                        Real::INFINITY,
                    );
                }
            }
        }
    }
    total / (h * h)
}

impl World {
    pub fn joint_count(&self) -> u32 {
        self.joint_mode.len() as u32
    }

    /// Whether all six of a joint's nodes are the live nodes it was made on.
    pub fn joint_nodes_live(&self, j: usize) -> bool {
        (0..6).all(|k| {
            let i = self.joint_nodes[j][k];
            self.node_alive(i) && self.node_gen[i as usize] == self.joint_gen[j][k]
        })
    }

    /// Adds a joint on `nodes = [a0, a1, ra, b0, b1, rb]` with limits `min` and `max` (radians from
    /// the current pose) and no drive. Returns its index, or `None` and changes nothing when a
    /// node is not live, a side repeats a node, the sides share a node, the limits are not
    /// `−π ≤ min ≤ 0 ≤ max ≤ π`, or the axis or a reference arm has no length now.
    pub fn add_joint(&mut self, nodes: [u32; 6], min: Real, max: Real) -> Option<u32> {
        let n = self.node_count();
        if nodes.iter().any(|&i| i >= n || !self.node_alive(i)) {
            return None;
        }
        for x in 0..6 {
            for y in (x + 1)..6 {
                if nodes[x] == nodes[y] {
                    return None;
                }
            }
        }
        if !(min >= -PI && min <= 0.0 && max >= 0.0 && max <= PI) {
            return None;
        }
        let body = self.node_body[nodes[0] as usize];
        if body != crate::body::NO_BODY && nodes.iter().all(|&i| self.node_body[i as usize] == body)
        {
            return None;
        }
        let zero = measure(nodes.map(|i| self.pos[i as usize]), 0.0)?.angle;
        self.joint_nodes.push(nodes);
        self.joint_gen
            .push(nodes.map(|i| self.node_gen[i as usize]));
        self.joint_mode.push(JOINT_FREE);
        self.joint_min.push(min);
        self.joint_max.push(max);
        self.joint_target.push(0.0);
        self.joint_rate.push(0.0);
        self.joint_cap.push(0.0);
        self.joint_zero.push(zero);
        self.joint_angle.push(0.0);
        self.joint_torque.push(0.0);
        self.joint_peak.push(0.0);
        Some(self.joint_count() - 1)
    }

    /// Sets a joint's drive: `mode` [`JOINT_FREE`], [`JOINT_SERVO`] (target angle in `[−π, π]`
    /// from the build pose, `rate` in rad/s) or [`JOINT_ROTOR`] (target speed in rad/s), with a
    /// torque cap in newton-metres. Returns `false` and changes nothing for an unknown joint or
    /// mode, a non-finite number, a negative rate or cap, or a servo target outside `[−π, π]`.
    pub fn set_joint_target(
        &mut self,
        j: u32,
        mode: u32,
        target: Real,
        rate: Real,
        cap: Real,
    ) -> bool {
        if j >= self.joint_count() || mode > JOINT_ROTOR {
            return false;
        }
        if !(target.is_finite() && rate.is_finite() && cap.is_finite()) || rate < 0.0 || cap < 0.0 {
            return false;
        }
        if mode == JOINT_SERVO && !(-PI..=PI).contains(&target) {
            return false;
        }
        let u = j as usize;
        self.joint_mode[u] = mode;
        self.joint_target[u] = target;
        self.joint_rate[u] = rate;
        self.joint_cap[u] = cap;
        true
    }

    /// A joint's settings.
    pub fn joint_spec(&self, j: usize) -> JointSpec {
        JointSpec {
            mode: self.joint_mode[j],
            min: self.joint_min[j],
            max: self.joint_max[j],
            target: self.joint_target[j],
            rate: self.joint_rate[j],
            cap: self.joint_cap[j],
            zero: self.joint_zero[j],
        }
    }

    /// Start of a step with joints: no segment is triggered yet and no drive has pushed.
    pub(crate) fn joints_begin_step(&mut self) {
        let segs = self.segment_count() as usize;
        self.seg_triggered.clear();
        self.seg_triggered.resize(segs, false);
        for t in self.joint_torque.iter_mut() {
            *t = 0.0;
        }
        for t in self.joint_peak.iter_mut() {
            *t = 0.0;
        }
    }

    /// Marks the segments of a moving joint's two first axis nodes as triggered.
    pub(crate) fn joint_moved(&mut self, j: usize) {
        for k in [0, 3] {
            let s = self.node_segment[self.joint_nodes[j][k] as usize];
            if s != crate::segment::NO_SEGMENT && (s as usize) < self.seg_triggered.len() {
                self.seg_triggered[s as usize] = true;
            }
        }
    }
}
