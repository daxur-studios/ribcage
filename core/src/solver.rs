//! XPBD beam solve in the "small steps" form: one pass over the beams per substep, and many
//! substeps. A pure function of the packed island buffer (layout rule 5); it never reads the
//! world arrays. One distance function for every beam (rule 6). Only add, subtract, multiply,
//! divide and square root on `Real`, nodes and beams in ascending packed order, no fused
//! multiply-add (determinism list). Yield is a force cap on the spring inside the substeps
//! ([`solve_distance`]); damping is a separate velocity pass ([`damp_beam`]); break is checked
//! once after the last substep ([`crate::failure`]).
//! Broken beams are not in the island, so neither the solve nor the substep rule sees them.
//! Stability guards (phase 1.6) sit around the solver, not in it: after each substep's damping
//! pass a node faster than `max_speed` is clamped ([`solve_island`]), and a non-finite position
//! or velocity freezes the island ([`SolveOutput::frozen`]).

use crate::body::{derive_body, predict_body, NO_BODY};
use crate::event::{Event, EVENT_FREEZE};
use crate::failure::check_failure;
use crate::island::Island;
use crate::maths::{Quat, Sym3};
use crate::real::{Real, Vec3};
use crate::sim::gravity_at;
use crate::world::BEAM_YIELDED;
use crate::STEP_SECONDS;

/// The island's [`Mass`], borrowed field by field so the solve can move positions beside it.
macro_rules! mass {
    ($isl:expr) => {
        Mass {
            node: &$isl.inv_mass,
            node_body: &$isl.node_body,
            offset: &$isl.offset,
            body: &$isl.body_inv_mass,
            inertia: &$isl.body_inv_inertia,
        }
    };
}

/// Substep rule constant: the count is the smallest `n` with `ω_max · h ≤ SUBSTEP_C`.
/// Measured in phase 1.4: every scene measured stays bounded past `c = 2.5` (the test
/// truss even at `c = 20`), so `c` is a cost and accuracy dial, not a stability limit. A
/// settled structure reads softer than exact by about `0.4 · (ω_max · h)²`, so about 40%
/// at `c = 1`. Kept at 1 (ER 001, phase 1.4).
pub const SUBSTEP_C: Real = 1.0;
/// Fewest substeps any island gets.
pub const MIN_SUBSTEPS: u32 = 4;
/// Most substeps any island gets. Islands stiffer than this allows run mushier than their
/// beams say; the stiffness scale (spec) exists to keep common beams below this.
pub const MAX_SUBSTEPS: u32 = 32;

/// The rule-based substep count for one island (deterministic mode).
///
/// For each beam `ω² = stiffness · (weight a + weight b)`, with stiffness
/// `1 / compliance`. The count is the smallest `n` in `MIN..=MAX` with `ω_max · h ≤ c`,
/// where `h = STEP_SECONDS / n`. A beam with zero compliance (infinitely stiff) between
/// movable nodes asks for the maximum. A beam between two pinned nodes asks for nothing.
pub fn substep_count(isl: &Island) -> u32 {
    substep_count_with(isl, SubstepRule::DEFAULT)
}

/// The two numbers of the substep rule. [`SubstepRule::DEFAULT`] is the deterministic default
/// and the only one the game uses; other values exist so the benchmark and tests can price
/// and compare settings (ER 001, phase 1.7) without a second code path.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SubstepRule {
    /// The rule constant `c`: the count is the smallest `n` with `ω_max · h ≤ c`.
    pub c: Real,
    /// Most substeps an island may get (the minimum is always [`MIN_SUBSTEPS`]).
    pub max: u32,
}

impl SubstepRule {
    pub const DEFAULT: SubstepRule = SubstepRule { c: SUBSTEP_C, max: MAX_SUBSTEPS };
}

/// [`substep_count`] with an explicit rule. Same arithmetic, so `DEFAULT` gives the same bits.
pub fn substep_count_with(isl: &Island, rule: SubstepRule) -> u32 {
    let max = rule.max.max(MIN_SUBSTEPS);
    let mut omega2_max: Real = 0.0;
    let m = mass!(isl);
    for k in 0..isl.beam_a.len() {
        let w = match (m.end(isl.beam_a[k]), m.end(isl.beam_b[k])) {
            (Endpoint::Node(a), Endpoint::Node(b)) => isl.inv_mass[a as usize] + isl.inv_mass[b as usize],
            // A riding end weighs by the beam's direction at the step's start (phase 5.3).
            (a, b) => {
                let d = b.place(&m, &isl.pos, &isl.body_centre, &isl.body_rot) - a.place(&m, &isl.pos, &isl.body_centre, &isl.body_rot);
                let len = d.length();
                let dir = if len > 0.0 { d / len } else { Vec3::ZERO };
                a.weight(&m, &isl.body_rot, dir) + b.weight(&m, &isl.body_rot, dir)
            }
        };
        if w == 0.0 {
            continue;
        }
        let c = isl.compliance[k];
        if c == 0.0 {
            return max;
        }
        let o2 = w / c;
        if o2 > omega2_max {
            omega2_max = o2;
        }
    }
    let omega_max = omega2_max.sqrt();
    let mut n = MIN_SUBSTEPS;
    while n < max && omega_max * (STEP_SECONDS / n as Real) > rule.c {
        n += 1;
    }
    n
}

/// What the kernel reads to weigh an end: inverse masses of the packed nodes and bodies, and
/// which body each packed node rides (layout rule 6). A rider's own inverse mass is zero in
/// `node`: its mass is in its body. `node_body` may be empty (no bodies: every node is loose).
#[derive(Clone, Copy)]
pub struct Mass<'a> {
    /// Inverse mass per packed node (zero for pinned nodes and riders).
    pub node: &'a [Real],
    /// Packed body each node rides, or [`NO_BODY`]. Empty when nothing rides.
    pub node_body: &'a [u32],
    /// A rider's offset from its body's centre, body frame (per packed node).
    pub offset: &'a [Vec3],
    /// Inverse mass per packed body, `1 / M`.
    pub body: &'a [Real],
    /// Inverse inertia per packed body, body frame.
    pub inertia: &'a [Sym3],
}

impl<'a> Mass<'a> {
    /// Loose nodes only, no bodies.
    pub fn nodes(inv_mass: &'a [Real]) -> Mass<'a> {
        Mass { node: inv_mass, node_body: &[], offset: &[], body: &[], inertia: &[] }
    }

    /// The end packed node `i` is: a point of its body ([`Endpoint::Riding`]) or itself.
    #[inline(always)]
    pub fn end(&self, i: u32) -> Endpoint {
        match self.node_body.get(i as usize) {
            Some(&b) if b != NO_BODY => Endpoint::Riding(b, self.offset[i as usize]),
            _ => Endpoint::Node(i),
        }
    }
}

/// Positions the kernel moves: the packed nodes, and the bodies' centres and rotations.
pub struct Pose<'a> {
    pub pos: &'a mut [Vec3],
    pub centre: &'a mut [Vec3],
    pub rot: &'a mut [Quat],
}

impl<'a> Pose<'a> {
    /// Loose nodes only, no bodies.
    pub fn nodes(pos: &'a mut [Vec3]) -> Pose<'a> {
        Pose { pos, centre: &mut [], rot: &mut [] }
    }
}

/// Velocities the damping pass changes: the packed nodes', and the bodies' linear and angular.
pub struct Vel<'a> {
    pub node: &'a mut [Vec3],
    pub lin: &'a mut [Vec3],
    pub ang: &'a mut [Vec3],
}

/// One end of a correction (layout rule 6). A beam has two node or riding ends; a ground
/// contact has a `Fixed` end (the ground, which never moves) and a node, a riding point or,
/// from phase 3.3, a `Point` on a beam. A riding end (phase 5.3) only changes
/// [`Endpoint::weight`] and how the end moves, never the callers' maths.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Endpoint {
    /// A packed loose node: moves by its own inverse mass.
    Node(u32),
    /// Static ground: infinite mass, never moved.
    Fixed,
    /// The point `(1 - t) · p_a + t · p_b` on the beam between packed nodes `a` and `b`. With
    /// `b_a = 1 - t` and `b_b = t` it resists a push by `b_a² · w_a + b_b² · w_b`, and a push of
    /// `Δλ` moves node `i` by `b_i · w_i · Δλ`, so the point itself moves by `weight · Δλ`. A
    /// part that rides a body takes its share through the body, as [`Endpoint::Riding`].
    Point(u32, u32, Real),
    /// The point at `local` (body frame, from the centre) on packed body `b` (phase 5.3). With
    /// `r = R · local` and the push along unit `n` it resists by
    /// `w = 1/M + (r × n) · I⁻¹ (r × n)`, `I⁻¹ = R I_b⁻¹ Rᵀ`. A push `p = n · Δλ` moves the
    /// centre by `p / M` and turns the body by `Δθ = I⁻¹ (r × p)`, applied as
    /// `q ← normalize(q + ½ (0, Δθ) ⊗ q)`, so to first order the point moves by `w · Δλ` along
    /// `n`, as a node would.
    Riding(u32, Vec3),
}

/// World-frame `I⁻¹ v` of packed body `b` at rotation `q`: `R I_b⁻¹ Rᵀ v`, no matrix built.
#[inline(always)]
fn turn(m: &Mass, q: Quat, b: usize, v: Vec3) -> Vec3 {
    q.rotate(m.inertia[b].mul_vec(q.inv_rotate(v)))
}

impl Endpoint {
    /// The end's resistance to a push along unit `dir`: how far it moves per unit of `Δλ`.
    /// Only a riding end depends on `dir`.
    #[inline(always)]
    pub fn weight(self, m: &Mass, rot: &[Quat], dir: Vec3) -> Real {
        match self {
            Endpoint::Node(i) => m.node[i as usize],
            Endpoint::Fixed => 0.0,
            Endpoint::Point(a, b, t) => {
                let (ba, bb) = (1.0 - t, t);
                ba * ba * m.end(a).weight(m, rot, dir) + bb * bb * m.end(b).weight(m, rot, dir)
            }
            Endpoint::Riding(b, local) => {
                let b = b as usize;
                let q = rot[b];
                let u = q.inv_rotate(q.rotate(local).cross(dir));
                m.body[b] + u.dot(m.inertia[b].mul_vec(u))
            }
        }
    }

    /// The end is, or has a part, riding a body.
    #[inline(always)]
    pub fn touches_body(self, m: &Mass) -> bool {
        match self {
            Endpoint::Riding(..) => true,
            Endpoint::Point(a, b, _) => m.end(a).touches_body(m) || m.end(b).touches_body(m),
            _ => false,
        }
    }

    /// Where the end is (the ground has no place of its own: zero).
    #[inline(always)]
    pub fn place(self, m: &Mass, pos: &[Vec3], centre: &[Vec3], rot: &[Quat]) -> Vec3 {
        match self {
            Endpoint::Node(i) => pos[i as usize],
            Endpoint::Fixed => Vec3::ZERO,
            Endpoint::Point(a, b, t) => {
                m.end(a).place(m, pos, centre, rot) * (1.0 - t) + m.end(b).place(m, pos, centre, rot) * t
            }
            Endpoint::Riding(b, local) => centre[b as usize] + rot[b as usize].rotate(local),
        }
    }

    /// Moves the end by a push of `dir · dl`, added when `plus` is set, else taken: a node by
    /// `dir · (b_i · w_i · dl)`, a body as [`Endpoint::Riding`] says. With `nodes` clear only
    /// bodies move (a contact's recovery shared into the bodies' predicted pose).
    #[inline(always)]
    fn shift(self, s: &mut Pose, m: &Mass, dir: Vec3, dl: Real, plus: bool, nodes: bool) {
        match self {
            Endpoint::Fixed => {}
            Endpoint::Node(i) => {
                if nodes {
                    let i = i as usize;
                    let d = dir * (m.node[i] * dl);
                    s.pos[i] = if plus { s.pos[i] + d } else { s.pos[i] - d };
                }
            }
            Endpoint::Point(a, b, t) => {
                for (i, share) in [(a, 1.0 - t), (b, t)] {
                    match m.end(i) {
                        Endpoint::Node(i) => {
                            if nodes {
                                let i = i as usize;
                                let d = dir * (share * m.node[i] * dl);
                                s.pos[i] = if plus { s.pos[i] + d } else { s.pos[i] - d };
                            }
                        }
                        ride => ride.shift(s, m, dir, share * dl, plus, nodes),
                    }
                }
            }
            Endpoint::Riding(b, local) => {
                let b = b as usize;
                let q = s.rot[b];
                let p = if plus { dir * dl } else { dir * -dl };
                let dt = turn(m, q, b, q.rotate(local).cross(p));
                s.centre[b] = s.centre[b] + p * m.body[b];
                let d = Quat::new(0.0, dt.x, dt.y, dt.z).mul(q);
                s.rot[b] = Quat::new(q.w + 0.5 * d.w, q.x + 0.5 * d.x, q.y + 0.5 * d.y, q.z + 0.5 * d.z).normalize();
            }
        }
    }

    /// The end's velocity (a riding point's is `V + ω × r`).
    #[inline(always)]
    fn velocity(self, m: &Mass, rot: &[Quat], v: &Vel) -> Vec3 {
        match self {
            Endpoint::Node(i) => v.node[i as usize],
            Endpoint::Fixed => Vec3::ZERO,
            Endpoint::Point(a, b, t) => m.end(a).velocity(m, rot, v) * (1.0 - t) + m.end(b).velocity(m, rot, v) * t,
            Endpoint::Riding(b, local) => {
                let b = b as usize;
                v.lin[b] + v.ang[b].cross(rot[b].rotate(local))
            }
        }
    }

    /// Changes the end's velocity by an impulse `dir · j`, added when `plus` is set, else taken.
    #[inline(always)]
    fn kick(self, m: &Mass, rot: &[Quat], v: &mut Vel, dir: Vec3, j: Real, plus: bool) {
        match self {
            Endpoint::Fixed => {}
            Endpoint::Node(i) => {
                let i = i as usize;
                let d = dir * (m.node[i] * j);
                v.node[i] = if plus { v.node[i] + d } else { v.node[i] - d };
            }
            Endpoint::Point(a, b, t) => {
                m.end(a).kick(m, rot, v, dir, (1.0 - t) * j, plus);
                m.end(b).kick(m, rot, v, dir, t * j, plus);
            }
            Endpoint::Riding(b, local) => {
                let b = b as usize;
                let q = rot[b];
                let p = if plus { dir * j } else { dir * -j };
                v.lin[b] = v.lin[b] + p * m.body[b];
                v.ang[b] = v.ang[b] + turn(m, q, b, q.rotate(local).cross(p));
            }
        }
    }
}

/// The correction kernel (layout rule 6): moves `a` by `-dir · (w_a · Δλ)` and `b` by
/// `+dir · (w_b · Δλ)`, each end by its own weight. Every constraint in the solver (beams,
/// ground contacts, friction) moves endpoints only through here. For node ends the arithmetic
/// and its order are exactly those of the milestone 1 beam solve, so beams give the same bits
/// as before.
#[inline(always)]
pub fn apply_correction(s: &mut Pose, m: &Mass, a: Endpoint, b: Endpoint, dir: Vec3, dl: Real) {
    a.shift(s, m, dir, dl, false, true);
    b.shift(s, m, dir, dl, true, true);
}

/// Speed at which a ground contact may remove overlap that was already there when the substep
/// began, in metres per second (the collision brief's recovery cap). Overlap created by this
/// substep's own motion is removed in full.
pub const RECOVERY_SPEED: Real = 2.0;

/// One ground contact for one substep: the push, then friction (plan 003, phases 3.2 and 3.3).
///
/// The contact holds the plane `normal · p = offset` for the whole step. The contact point is
/// `end`: a node, a point riding a body, or a point on a beam. `p` is where it is now in `s`
/// (after the beams), `p0` where it was at the substep's start (`start` holds the nodes' and
/// bodies' start poses), `radius` its sphere. With `d = radius − (normal · p − offset)` the
/// overlap now and `d0` the same at `p0`:
///
/// - **Push.** The overlap allowed to remain is `max(0, d0 − RECOVERY_SPEED · h)`, so new
///   overlap goes at once and old overlap drains at no more than the cap. If the overlap is
///   larger, `Δλ_n = (d − allowed) / w` (rigid: no compliance, `w` the end's weight along the
///   normal), along the normal, push only.
/// - **Split recovery.** The part of the push that removes old overlap (at most
///   `d0 − allowed`) also moves the start pose as the push moved the end, so the velocities
///   derived from `(p − p0) / h` do not include it: a node dug out of the ground rises at up to
///   the cap but does not leave it with that speed and hop. A body's velocity is derived
///   against its predicted pose (`pred`), so that pose takes the same move (phase 5.3). Only
///   the part that stops the end's own motion into the ground changes its velocity.
/// - **Friction.** The end's sideways slip over the substep, `s` (the part of `p − p0` along
///   the plane, after the push), is cancelled by `Δλ_t = min(|s| / w_t, μ · Δλ_n)`, `w_t` the
///   end's weight along the slip: in full if that is within `μ` times the push (sticking),
///   otherwise by `μ` times the push (sliding). One coefficient, static equals kinetic.
///
/// No push means no normal force and so no friction. Returns `Δλ_n` (zero when it did not push).
///
/// The island solve runs the two halves apart ([`push_ground_contact`], then
/// [`ground_friction`]) so that all of a body's pushes come before any of its friction; this
/// function is the two back to back, for one contact alone.
#[allow(clippy::too_many_arguments)]
#[inline]
pub fn solve_ground_contact(
    s: &mut Pose,
    start: &mut Pose,
    pred: &mut Pose,
    m: &Mass,
    end: Endpoint,
    radius: Real,
    normal: Vec3,
    offset: Real,
    mu: Real,
    h: Real,
) -> Real {
    let (dl_n, p0) = push_ground_contact(s, start, pred, m, end, radius, normal, offset, h);
    ground_friction(s, start, m, end, normal, dl_n, p0, mu);
    dl_n
}

/// The push and split recovery of [`solve_ground_contact`]. Returns `Δλ_n` (zero when it did
/// not push) and `p0`, the end's place in `start` before the recovery moved it.
#[allow(clippy::too_many_arguments)]
#[inline]
pub fn push_ground_contact(
    s: &mut Pose,
    start: &mut Pose,
    pred: &mut Pose,
    m: &Mass,
    end: Endpoint,
    radius: Real,
    normal: Vec3,
    offset: Real,
    h: Real,
) -> (Real, Vec3) {
    let ground = Endpoint::Fixed;
    let w = ground.weight(m, s.rot, normal) + end.weight(m, s.rot, normal);
    if w == 0.0 {
        return (0.0, Vec3::ZERO);
    }
    let p0 = end.place(m, start.pos, start.centre, start.rot);
    let d = radius - (normal.dot(end.place(m, s.pos, s.centre, s.rot)) - offset);
    let d0 = radius - (normal.dot(p0) - offset);
    let cap = RECOVERY_SPEED * h;
    let allowed = if d0 > cap { d0 - cap } else { 0.0 };
    #[allow(clippy::neg_cmp_op_on_partial_ord)] // also true for NaN: a bad number is not pushed
    if !(d > allowed) {
        return (0.0, p0);
    }
    let push = d - allowed;
    let dl_n = push / w;
    apply_correction(s, m, ground, end, normal, dl_n);
    let old = d0 - allowed;
    if old > 0.0 {
        let recovered = if push < old { push } else { old };
        end.shift(start, m, normal, recovered / w, true, true);
        end.shift(pred, m, normal, recovered / w, true, false);
    }
    (dl_n, p0)
}

/// The friction of [`solve_ground_contact`], after its push `dl_n` (none without a push). The
/// slip is measured from `p0` for an end of loose nodes (the recovery moved them along the
/// normal only, so that is the same slip), and from the end's place in `start` now for an end
/// that touches a body: the recovery turned the body's start pose, which is not slip.
#[allow(clippy::too_many_arguments)]
#[inline]
pub fn ground_friction(s: &mut Pose, start: &Pose, m: &Mass, end: Endpoint, normal: Vec3, dl_n: Real, p0: Vec3, mu: Real) {
    if dl_n == 0.0 {
        return;
    }
    let ground = Endpoint::Fixed;
    let p0 = if end.touches_body(m) { end.place(m, start.pos, start.centre, start.rot) } else { p0 };
    let moved = end.place(m, s.pos, s.centre, s.rot) - p0;
    let slip = moved - normal * normal.dot(moved);
    let sl = slip.length();
    if sl > 0.0 {
        let along = slip / sl;
        let wt = ground.weight(m, s.rot, along) + end.weight(m, s.rot, along);
        let full = sl / wt;
        let limit = mu * dl_n;
        let dl_t = if full <= limit { full } else { limit };
        apply_correction(s, m, ground, end, along, -dl_t);
    }
}

/// The one distance constraint (layout rule 6). A beam endpoint is a loose node or a point
/// riding a body. Returns `Δλ`; the ends move along the beam through [`apply_correction`], by
/// their weight times `Δλ`, in opposite senses.
///
/// `C` is current length minus rest, `w` the sum of the ends' weights along the beam and
/// `α = compliance / h²`:
///
/// `Δλ = -C / (w + α)`
///
/// The beam's spring force is `F = -Δλ / h²`, in newtons, positive in tension. Damping is not
/// in here: it is a separate velocity pass after each substep ([`damp_beam`]), so this is the
/// spring alone.
///
/// **Yield cap.** `yield_elong` is the elastic stretch at yield (`yield_strain · rest0`), so
/// the yield force is `F_y = yield_elong / compliance` and the largest `|Δλ|` is `F_y · h²`. A
/// larger correction is clamped to that, in either sign, so the spring never carries more than
/// `F_y` in any substep and the excess load goes to its neighbours in the same substep. When
/// the clamp fires the beam yields: `rest` becomes the length after the clamped move, minus
/// `yield_elong` in tension and plus it in compression (both ends move along the beam, so that
/// length is `len + w · Δλ`, no square root), and the function returns `true` so the caller
/// sets the yielded flag. With zero compliance, or an infinite or NaN `yield_elong`, there is
/// no cap. A beam between two pinned nodes (`w = 0`) is skipped: it carries no force and never
/// yields, though break still reads its length.
///
/// Returns `(Δλ, yielded)`.
#[allow(clippy::too_many_arguments)]
pub fn solve_distance(
    s: &mut Pose,
    m: &Mass,
    a: Endpoint,
    b: Endpoint,
    rest: &mut Real,
    yield_elong: Real,
    compliance: Real,
    h: Real,
) -> (Real, bool) {
    let d = b.place(m, s.pos, s.centre, s.rot) - a.place(m, s.pos, s.centre, s.rot);
    let len = d.length();
    if len == 0.0 {
        // No direction to push along. Coincident endpoints are a generator bug; 1.6 guards it.
        return (0.0, false);
    }
    let dir = d / len;
    let w = a.weight(m, s.rot, dir) + b.weight(m, s.rot, dir);
    if w == 0.0 {
        return (0.0, false);
    }
    let alpha = compliance / (h * h);
    let mut dl = -(len - *rest) / (w + alpha);
    let mut yielded = false;
    if compliance > 0.0 {
        let cap = yield_elong / compliance * (h * h);
        if dl < -cap {
            dl = -cap; // tension: the spring is at its yield force
            *rest = (len + w * dl) - yield_elong;
            yielded = true;
        } else if dl > cap {
            dl = cap; // compression
            *rest = (len + w * dl) + yield_elong;
            yielded = true;
        }
    }
    apply_correction(s, m, a, b, dir, dl);
    (dl, yielded)
}

/// Beam damping, as a velocity pass after each substep's velocities are derived: a damper of
/// `damping` newton-seconds per metre beside the spring, acting on the endpoints' relative
/// speed along the beam. Implicit, so it never overshoots: the relative speed `v` becomes
/// `v / (1 + β · w)` with `β = damping · h` and `w` the ends' weights along the beam, by an
/// impulse `β · v / (1 + β · w)` given to each end through its weight (a riding end's body
/// takes it as `V += p / M`, `ω += I⁻¹ (r × p)`). Positions are not touched.
///
/// Why a separate pass and not the XPBD `γ` term inside the constraint (the phase 1.3 form):
/// with one pass per substep the `γ` term sees each substep's predicted move as speed, so a
/// still, loaded structure carries damper force. Elastic beams hid that as extra sag; with
/// yield capping the spring it showed as wrong forces at rest, and a damped beam yielded when
/// spring plus damper passed `F_y`. Here a still structure has zero relative speed and zero
/// damper force, and yield reads the spring alone (ER 001, phase 1.5 damping fix).
#[allow(clippy::too_many_arguments)]
pub fn damp_beam(
    pos: &[Vec3],
    centre: &[Vec3],
    rot: &[Quat],
    v: &mut Vel,
    m: &Mass,
    a: Endpoint,
    b: Endpoint,
    damping: Real,
    h: Real,
) {
    if damping == 0.0 {
        return;
    }
    let d = b.place(m, pos, centre, rot) - a.place(m, pos, centre, rot);
    let len = d.length();
    if len == 0.0 {
        return;
    }
    let dir = d / len;
    let w = a.weight(m, rot, dir) + b.weight(m, rot, dir);
    if w == 0.0 {
        return;
    }
    let vr = dir.dot(b.velocity(m, rot, v) - a.velocity(m, rot, v));
    let beta = damping * h;
    let j = beta * vr / (1.0 + beta * w);
    a.kick(m, rot, v, dir, j, true);
    b.kick(m, rot, v, dir, j, false);
}

/// What one island's solve returns.
#[derive(Debug, PartialEq)]
pub struct SolveOutput {
    /// Events in ascending beam order, `step` left at zero for the caller to stamp. A frozen
    /// island has exactly one, the freeze event.
    pub events: Vec<Event>,
    /// Nodes of this island clamped to `max_speed` at least once this step.
    pub clamped: u32,
    /// The island hit a bad number and was frozen: the caller sets `NODE_FROZEN` on its movable
    /// nodes and riders.
    pub frozen: bool,
}

/// Advances one island by one fixed step in `n` substeps. Per substep: predict (gravity and
/// gathered forces into velocity, move; the bodies by their own integrator,
/// [`crate::body::predict_body`]), solve every beam once in order, then the ground contacts,
/// derive velocity from the change in position (bodies against their predicted pose,
/// [`crate::body::derive_body`]), damp every beam once in order ([`damp_beam`]), then apply
/// the clamps and the bad-number check. Pinned nodes (inverse mass zero) never move. A node
/// riding a body is moved only through its body, as an [`Endpoint::Riding`] end, and is placed
/// once after the last substep (`centre + R · offset`, velocity `V + ω × r`). Yield happens
/// inside the substeps (the force cap in [`solve_distance`], which sets the `YIELDED` flag).
/// After the last substep, break is checked once and the events returned ([`check_failure`]).
///
/// **Movement clamp.** A movable node whose speed is above `max_speed` has its velocity scaled
/// to `max_speed` (one square root, one divide) and its position reset to the substep's start
/// position plus the clamped velocity times `h`, so position and velocity stay consistent. A
/// speed that is NaN is not above anything and is left to the bad-number check. A body's linear
/// speed is scaled the same way, its pose left as it is (phase 5.2).
///
/// **Bad-number freeze.** At the end of every substep, after the clamp, each movable node's
/// position and velocity, and each body's pose and velocities, must be finite. If one is not,
/// the island stops at once: every node's position goes back to its value at the start of the
/// step (pinned nodes included, since a bad number reaches them through zero-weight
/// arithmetic), movable and riding velocities are zeroed, bodies go back to their start pose
/// with no velocity, beam `rest`, flags and `last_lambda` go back to their start-of-step
/// values, and the substep number is reported in a freeze event. Nothing else is checked or
/// emitted.
///
/// `last_lambda` ends up holding each beam's spring force from the last substep, `-Δλ / h²`,
/// in newtons, positive in tension. It never exceeds the beam's yield force, and it does not
/// include the damper (zero for a beam that just broke).
#[allow(clippy::needless_range_loop)] // explicit ascending index order is a determinism rule
pub fn solve_island(isl: &mut Island, n: u32, max_speed: Real) -> SolveOutput {
    let h = STEP_SECONDS / n as Real;
    let nodes = isl.pos.len();
    let beams = isl.beam_a.len();
    let bodies = isl.body_world.len();
    let mut prev = isl.pos.clone();
    let mut prev_centre = isl.body_centre.clone();
    let mut prev_rot = isl.body_rot.clone();
    let flags_before = isl.beam_flags.clone();
    let start_pos = isl.pos.clone();
    let start_vel = isl.vel.clone();
    let start_centre = isl.body_centre.clone();
    let start_rot = isl.body_rot.clone();
    let start_rest = isl.rest.clone();
    let start_lambda = isl.last_lambda.clone();
    let mut was_clamped = vec![false; nodes];
    let mut pushes = vec![(0.0 as Real, Vec3::ZERO); isl.contact_node.len()];
    let mut clamped = 0u32;
    for substep in 1..=n {
        for i in 0..nodes {
            prev[i] = isl.pos[i];
            let w = isl.inv_mass[i];
            if w == 0.0 {
                continue;
            }
            let acc = gravity_at(isl.pos[i]) + isl.force[i] * w;
            isl.vel[i] = isl.vel[i] + acc * h;
            isl.pos[i] = isl.pos[i] + isl.vel[i] * h;
        }
        for b in 0..bodies {
            prev_centre[b] = isl.body_centre[b];
            prev_rot[b] = isl.body_rot[b];
            predict_body(isl, b, h);
        }
        {
            let m = mass!(isl);
            let mut s = Pose { pos: &mut isl.pos, centre: &mut isl.body_centre, rot: &mut isl.body_rot };
            for k in 0..beams {
                // No original length means no strain scale, so no cap (a generator bug 1.6 guards).
                let yield_elong = if isl.rest0[k] == 0.0 { Real::INFINITY } else { isl.yield_strain[k] * isl.rest0[k] };
                let (dl, yielded) = solve_distance(
                    &mut s,
                    &m,
                    m.end(isl.beam_a[k]),
                    m.end(isl.beam_b[k]),
                    &mut isl.rest[k],
                    yield_elong,
                    isl.compliance[k],
                    h,
                );
                if yielded {
                    isl.beam_flags[k] |= BEAM_YIELDED;
                }
                isl.last_lambda[k] = -dl / (h * h);
            }
            // Ground contacts after the beams: node contacts in ascending packed node order,
            // then beam midpoints in ascending beam order (phases 3.2 and 3.3).
            let mut start = Pose { pos: &mut prev, centre: &mut prev_centre, rot: &mut prev_rot };
            let mut pred = Pose { pos: &mut [], centre: &mut isl.body_pred_centre, rot: &mut isl.body_pred_rot };
            // Node contacts in two passes: every push, then every friction (phase 5.3). A
            // loose node's contact touches only that node, so for loose nodes this is the same
            // arithmetic as one pass; for a body's corners it keeps one corner's push from
            // tilting the body under another corner's friction.
            for c in 0..isl.contact_node.len() {
                let i = isl.contact_node[c];
                pushes[c] = push_ground_contact(
                    &mut s,
                    &mut start,
                    &mut pred,
                    &m,
                    m.end(i),
                    isl.radius[i as usize],
                    isl.contact_normal[c],
                    isl.contact_offset[c],
                    h,
                );
            }
            for c in 0..isl.contact_node.len() {
                let (dl_n, p0) = pushes[c];
                ground_friction(&mut s, &start, &m, m.end(isl.contact_node[c]), isl.contact_normal[c], dl_n, p0, isl.friction);
            }
            for c in 0..isl.beam_contact.len() {
                let k = isl.beam_contact[c] as usize;
                let (a, b) = (isl.beam_a[k], isl.beam_b[k]);
                let (ra, rb) = (isl.radius[a as usize], isl.radius[b as usize]);
                solve_ground_contact(
                    &mut s,
                    &mut start,
                    &mut pred,
                    &m,
                    Endpoint::Point(a, b, 0.5),
                    if ra < rb { ra } else { rb },
                    isl.beam_contact_normal[c],
                    isl.beam_contact_offset[c],
                    isl.friction,
                    h,
                );
            }
        }
        for i in 0..nodes {
            if isl.inv_mass[i] == 0.0 {
                continue;
            }
            isl.vel[i] = (isl.pos[i] - prev[i]) / h;
        }
        for b in 0..bodies {
            derive_body(isl, b, h);
        }
        {
            let m = mass!(isl);
            let mut v = Vel { node: &mut isl.vel, lin: &mut isl.body_vel, ang: &mut isl.body_omega };
            for k in 0..beams {
                damp_beam(
                    &isl.pos,
                    &isl.body_centre,
                    &isl.body_rot,
                    &mut v,
                    &m,
                    m.end(isl.beam_a[k]),
                    m.end(isl.beam_b[k]),
                    isl.damping[k],
                    h,
                );
            }
        }
        // Movement and velocity clamp, then the bad-number check, in ascending node order,
        // then the bodies in ascending order.
        let mut bad = false;
        for i in 0..nodes {
            if isl.inv_mass[i] == 0.0 {
                continue;
            }
            let speed = isl.vel[i].length();
            if speed > max_speed {
                isl.vel[i] = isl.vel[i] * (max_speed / speed);
                isl.pos[i] = prev[i] + isl.vel[i] * h;
                if !was_clamped[i] {
                    was_clamped[i] = true;
                    clamped += 1;
                }
            }
            let (p, v) = (isl.pos[i], isl.vel[i]);
            if !(p.x.is_finite() && p.y.is_finite() && p.z.is_finite() && v.x.is_finite() && v.y.is_finite() && v.z.is_finite()) {
                bad = true;
            }
        }
        for b in 0..bodies {
            let speed = isl.body_vel[b].length();
            if speed > max_speed {
                isl.body_vel[b] = isl.body_vel[b] * (max_speed / speed);
            }
            if !(isl.body_centre[b].is_finite() && isl.body_rot[b].is_finite() && isl.body_vel[b].is_finite() && isl.body_omega[b].is_finite()) {
                bad = true;
            }
        }
        if bad {
            for i in 0..nodes {
                isl.pos[i] = start_pos[i];
                isl.vel[i] = if isl.inv_mass[i] != 0.0 || isl.rides(i) { Vec3::ZERO } else { start_vel[i] };
            }
            for b in 0..bodies {
                isl.body_centre[b] = start_centre[b];
                isl.body_rot[b] = start_rot[b];
                isl.body_pred_centre[b] = start_centre[b];
                isl.body_pred_rot[b] = start_rot[b];
                isl.body_vel[b] = Vec3::ZERO;
                isl.body_omega[b] = Vec3::ZERO;
            }
            isl.rest.copy_from_slice(&start_rest);
            isl.beam_flags.copy_from_slice(&flags_before);
            isl.last_lambda.copy_from_slice(&start_lambda);
            let event = Event { kind: EVENT_FREEZE, index: isl.root, step: 0, value: substep as Real };
            return SolveOutput { events: vec![event], clamped, frozen: true };
        }
    }
    // Riders follow their bodies, once per step, in node order (break reads their positions).
    for i in 0..nodes {
        if !isl.rides(i) {
            continue;
        }
        let b = isl.node_body[i] as usize;
        let p = isl.body_centre[b] + isl.body_rot[b].rotate(isl.offset[i]);
        isl.pos[i] = p;
        isl.vel[i] = isl.body_vel[b] + isl.body_omega[b].cross(p - isl.body_centre[b]);
    }
    SolveOutput { events: check_failure(isl, &flags_before), clamped, frozen: false }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::island::build_islands;
    use crate::sim::{state_hash, step, step_with_substeps};
    use crate::world::{BeamSpec, World};

    const G: Real = 9.81;

    fn beam(compliance: Real, damping: Real) -> BeamSpec {
        BeamSpec { rest: 1.0, compliance, damping, yield_strain: 0.01, break_strain: 0.05 }
    }

    fn run(w: &mut World, seconds: u32) {
        for _ in 0..seconds * 60 {
            step(w);
        }
    }

    fn len(w: &World, a: usize, b: usize) -> Real {
        (w.pos[b] - w.pos[a]).length()
    }

    /// One mass hanging from a pinned node, starting at rest length. Returns the settled
    /// stretch. Damping is chosen near half of critical so it settles within a few seconds.
    fn hanging_stretch(mass: Real, compliance: Real) -> Real {
        let mut w = World::new();
        w.add_node(Vec3::new(0.0, 0.0, 0.0), 0.0, 0.1);
        w.add_node(Vec3::new(0.0, -1.0, 0.0), 1.0 / mass, 0.1);
        let critical = 2.0 * (mass / compliance).sqrt();
        w.add_beam(0, 1, beam(compliance, 0.5 * critical));
        run(&mut w, 5);
        assert!(w.vel[1].length() < 1e-6, "still moving: {:?}", w.vel[1]);
        len(&w, 0, 1) - 1.0
    }

    /// A single hanging mass settles at weight times compliance. With one constraint the
    /// XPBD fixed point is exact, so the tolerance is tight.
    #[test]
    fn hanging_mass_stretches_by_weight_over_stiffness() {
        let stretch = hanging_stretch(2.0, 1e-4);
        let expected = 2.0 * G * 1e-4;
        assert!((stretch - expected).abs() < 1e-6 * expected, "stretch {stretch}, expected {expected}");
    }

    /// Ten times stiffer, ten times less stretch.
    #[test]
    fn stiffer_beam_stretches_less() {
        let soft = hanging_stretch(1.0, 1e-4);
        let stiff = hanging_stretch(1.0, 1e-5);
        assert!(stiff < soft);
        let ratio = soft / stiff;
        assert!((ratio - 10.0).abs() < 1e-3, "ratio {ratio}");
    }

    /// A pendulum released level: the rod keeps its length, the bob swings through the
    /// bottom at about the free-fall speed, reaches the far side, and never climbs above
    /// where it started (no energy gain).
    #[test]
    fn pendulum_swings_and_keeps_length() {
        let mut w = World::new();
        w.add_node(Vec3::new(0.0, 0.0, 0.0), 0.0, 0.1);
        w.add_node(Vec3::new(1.0, 0.0, 0.0), 1.0, 0.1);
        w.add_beam(0, 1, beam(1e-7, 0.0));
        let (mut max_speed, mut min_x, mut max_y, mut worst_len) = (0.0 as Real, 1.0 as Real, Real::MIN, 0.0 as Real);
        // A level pendulum of length 1 m has a half period of about 1.18 s.
        for _ in 0..90 {
            step(&mut w);
            max_speed = max_speed.max(w.vel[1].length());
            min_x = min_x.min(w.pos[1].x);
            max_y = max_y.max(w.pos[1].y);
            worst_len = worst_len.max((len(&w, 0, 1) - 1.0).abs());
        }
        let free_fall = (2.0 * G).sqrt();
        assert!(worst_len < 1e-3, "rod length off by {worst_len}");
        assert!((max_speed - free_fall).abs() < 0.02 * free_fall, "bottom speed {max_speed}, free fall {free_fall}");
        assert!(min_x < -0.9, "did not reach the far side: {min_x}");
        assert!(max_y < 1e-6, "climbed above the start: {max_y}");
    }

    /// A chain of ten 0.5 kg masses on beams of compliance 1e-5, damped at a quarter of
    /// one link's critical value, settled for `seconds` at `substeps` (0 means the rule).
    /// Returns each beam's stretch divided by the textbook stretch (weight below it times
    /// compliance), top beam first.
    fn chain_stretch_ratios(substeps: u32, seconds: u32) -> (World, Vec<Real>) {
        let mut w = World::new();
        let (m, c) = (0.5, 1e-5);
        w.add_node(Vec3::new(0.0, 0.0, 0.0), 0.0, 0.1);
        for i in 1..=10 {
            w.add_node(Vec3::new(0.0, -(i as Real), 0.0), 1.0 / m, 0.1);
            w.add_beam(i - 1, i, beam(c, 0.5 * (m / c).sqrt()));
        }
        for _ in 0..seconds * 60 {
            if substeps == 0 {
                step(&mut w);
            } else {
                step_with_substeps(&mut w, substeps);
            }
        }
        let ratios = (0..10usize).map(|k| (len(&w, k, k + 1) - 1.0) / ((10 - k) as Real * m * G * c)).collect();
        (w, ratios)
    }

    /// The chain settles on its line, every beam is in tension, and the load (stretch)
    /// increases toward the top. With one solver pass per substep a chain reads softer than
    /// the textbook, never stiffer, and the excess shrinks as substeps rise (measured on the
    /// top beam: 2.2x at 8 substeps, 1.7x at the rule's 11, 1.15x at 32; 1.05x at 32 with no
    /// damping, so damping adds to the sag). Phase 1.4 measures this properly.
    #[test]
    fn hanging_chain_settles_and_converges_with_substeps() {
        let (w, r) = chain_stretch_ratios(0, 10);
        let mut prev_stretch = Real::MAX;
        for k in 0..10usize {
            let stretch = len(&w, k, k + 1) - 1.0;
            assert!(stretch > 0.0 && stretch < prev_stretch, "beam {k} stretch {stretch}");
            assert!(r[k] >= 1.0 - 1e-9 && r[k] < 2.0, "beam {k} ratio {}", r[k]);
            prev_stretch = stretch;
        }
        for i in 1..=10usize {
            assert!(w.vel[i].length() < 1e-6, "node {i} still moving");
            assert!(w.pos[i].x == 0.0 && w.pos[i].z == 0.0, "chain left its line");
        }
        // Four times the substeps, well under half the excess stretch on the top beam.
        let (_, r8) = chain_stretch_ratios(8, 10);
        let (_, r32) = chain_stretch_ratios(32, 10);
        assert!(r32[0] - 1.0 < 0.5 * (r8[0] - 1.0), "8 substeps {}, 32 substeps {}", r8[0], r32[0]);
    }

    /// Stored beam force is the tension, in newtons: the top beam of a settled hanging mass
    /// carries its weight.
    #[test]
    fn last_lambda_is_tension_in_newtons() {
        let mut w = World::new();
        w.add_node(Vec3::new(0.0, 0.0, 0.0), 0.0, 0.1);
        w.add_node(Vec3::new(0.0, -1.0, 0.0), 1.0 / 3.0, 0.1);
        w.add_beam(0, 1, beam(1e-4, 2.0 * (3.0 / 1e-4 as Real).sqrt()));
        run(&mut w, 5);
        let weight = 3.0 * G;
        assert!((w.last_lambda[0] - weight).abs() < 1e-6 * weight, "force {}", w.last_lambda[0]);
    }

    fn one_beam_island(inv_a: Real, inv_b: Real, compliance: Real) -> World {
        let mut w = World::new();
        w.add_node(Vec3::new(0.0, 0.0, 0.0), inv_a, 0.1);
        w.add_node(Vec3::new(1.0, 0.0, 0.0), inv_b, 0.1);
        w.add_beam(0, 1, beam(compliance, 0.0));
        w
    }

    fn count(w: &World) -> u32 {
        substep_count(&build_islands(w)[0])
    }

    /// The count is the smallest n in 4..=32 with ω_max · h ≤ c.
    #[test]
    fn substep_count_follows_the_rule() {
        // ω = sqrt(1e6 · 1) = 1000 rad/s; n ≥ 1000 / 60 = 16.7, so 17.
        assert_eq!(count(&one_beam_island(0.0, 1.0, 1e-6)), 17);
        // Soft beam: ω = 10, well under the minimum.
        assert_eq!(count(&one_beam_island(1.0, 1.0, 0.02)), MIN_SUBSTEPS);
        // Very stiff: clamped to the maximum.
        assert_eq!(count(&one_beam_island(1.0, 1.0, 1e-9)), MAX_SUBSTEPS);
        // Zero compliance between movable nodes asks for the maximum.
        assert_eq!(count(&one_beam_island(0.0, 1.0, 0.0)), MAX_SUBSTEPS);
        // A beam between two pinned nodes asks for nothing, even with zero compliance.
        assert_eq!(count(&one_beam_island(0.0, 0.0, 0.0)), MIN_SUBSTEPS);
        // The rule's boundary, checked directly across a sweep of stiffnesses.
        let mut compliance: Real = 1e-3;
        for _ in 0..60 {
            compliance *= 0.8;
            let w = one_beam_island(0.0, 1.0, compliance);
            let n = count(&w);
            let omega = (1.0 / compliance).sqrt();
            assert!((MIN_SUBSTEPS..=MAX_SUBSTEPS).contains(&n));
            if n < MAX_SUBSTEPS {
                assert!(omega * STEP_SECONDS / n as Real <= SUBSTEP_C, "n {n} too small");
            }
            if n > MIN_SUBSTEPS {
                assert!(omega * STEP_SECONDS / (n - 1) as Real > SUBSTEP_C, "n {n} not the smallest");
            }
        }
        // The stiffest beam wins, and the step reports the count it used.
        let mut w = one_beam_island(0.0, 1.0, 1e-6);
        w.add_node(Vec3::new(2.0, 0.0, 0.0), 1.0, 0.1);
        w.add_beam(1, 2, beam(0.02, 0.0));
        assert_eq!(count(&w), 17);
        assert_eq!(step(&mut w), 17);
    }

    /// The solver is a pure function of the buffer: the same island solved twice gives the
    /// same bits, and the same scene stepped twice gives the same hash.
    #[test]
    fn same_inputs_same_bits() {
        let make = || {
            let mut w = World::new();
            w.add_node(Vec3::new(0.0, 0.0, 0.0), 0.0, 0.1);
            w.add_node(Vec3::new(1.0, 0.1, 0.2), 1.0, 0.1);
            w.add_node(Vec3::new(1.5, 1.0, -0.3), 0.7, 0.1);
            w.add_beam(0, 1, beam(1e-5, 30.0));
            w.add_beam(1, 2, beam(1e-4, 10.0));
            w.add_beam(0, 2, beam(1e-5, 30.0));
            w
        };
        let mut i1 = build_islands(&make());
        let mut i2 = build_islands(&make());
        solve_island(&mut i1[0], 9, 1000.0);
        solve_island(&mut i2[0], 9, 1000.0);
        assert_eq!(i1, i2);
        let (mut a, mut b) = (make(), make());
        for s in 0..300 {
            if s == 10 {
                a.add_force(2, Vec3::new(5.0, 0.0, 0.0));
                b.add_force(2, Vec3::new(5.0, 0.0, 0.0));
            }
            step(&mut a);
            step(&mut b);
        }
        assert_eq!(state_hash(&a), state_hash(&b));
        // A different substep count is a different run.
        let mut c = make();
        for s in 0..300 {
            if s == 10 {
                c.add_force(2, Vec3::new(5.0, 0.0, 0.0));
            }
            step_with_substeps(&mut c, 5);
        }
        assert_ne!(state_hash(&a), state_hash(&c));
    }
}
