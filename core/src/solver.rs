//! XPBD beam solve in the "small steps" form: one pass over the beams per substep, and many
//! substeps. A pure function of the packed island buffer (layout rule 5); it never reads the
//! world arrays. One distance function for every beam (rule 6). Only add, subtract, multiply,
//! divide and square root on `Real`, nodes and beams in ascending packed order, no fused
//! multiply-add (determinism list). Yield is a force cap on the spring inside the substeps
//! ([`solve_distance`]); damping is a separate velocity pass ([`damp_beam`]); break is checked
//! once after the last substep ([`crate::failure`]).
//! Broken beams are not in the island, so neither the solve nor the substep rule sees them.
//! Movement clamps arrive in 1.6.

use crate::event::Event;
use crate::failure::check_failure;
use crate::island::Island;
use crate::real::{Real, Vec3};
use crate::sim::gravity_at;
use crate::world::BEAM_YIELDED;
use crate::STEP_SECONDS;

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
/// For each beam `ω² = stiffness · (inverse mass a + inverse mass b)`, with stiffness
/// `1 / compliance`. The count is the smallest `n` in `MIN..=MAX` with `ω_max · h ≤ c`,
/// where `h = STEP_SECONDS / n`. A beam with zero compliance (infinitely stiff) between
/// movable nodes asks for the maximum. A beam between two pinned nodes asks for nothing.
pub fn substep_count(isl: &Island) -> u32 {
    let mut omega2_max: Real = 0.0;
    for k in 0..isl.beam_a.len() {
        let w = isl.inv_mass[isl.beam_a[k] as usize] + isl.inv_mass[isl.beam_b[k] as usize];
        if w == 0.0 {
            continue;
        }
        let c = isl.compliance[k];
        if c == 0.0 {
            return MAX_SUBSTEPS;
        }
        let o2 = w / c;
        if o2 > omega2_max {
            omega2_max = o2;
        }
    }
    let omega_max = omega2_max.sqrt();
    let mut n = MIN_SUBSTEPS;
    while n < MAX_SUBSTEPS && omega_max * (STEP_SECONDS / n as Real) > SUBSTEP_C {
        n += 1;
    }
    n
}

/// The one distance constraint (layout rule 6). In milestone 1 an endpoint is a plain node:
/// `a` and `b` index `pos` and `inv_mass`. Returns `Δλ`; each endpoint moves along the beam by
/// its inverse mass times `Δλ`, in opposite senses.
///
/// `C` is current length minus rest, `w` the sum of inverse masses and `α = compliance / h²`:
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
pub fn solve_distance(
    pos: &mut [Vec3],
    inv_mass: &[Real],
    a: usize,
    b: usize,
    rest: &mut Real,
    yield_elong: Real,
    compliance: Real,
    h: Real,
) -> (Real, bool) {
    let wa = inv_mass[a];
    let wb = inv_mass[b];
    let w = wa + wb;
    if w == 0.0 {
        return (0.0, false);
    }
    let d = pos[b] - pos[a];
    let len = d.length();
    if len == 0.0 {
        // No direction to push along. Coincident endpoints are a generator bug; 1.6 guards it.
        return (0.0, false);
    }
    let dir = d / len;
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
    pos[a] = pos[a] - dir * (wa * dl);
    pos[b] = pos[b] + dir * (wb * dl);
    (dl, yielded)
}

/// Beam damping, as a velocity pass after each substep's velocities are derived: a damper of
/// `damping` newton-seconds per metre beside the spring, acting on the endpoints' relative
/// speed along the beam. Implicit, so it never overshoots: the relative speed `v` becomes
/// `v / (1 + β · w)` with `β = damping · h`, by an impulse `β · v / (1 + β · w)` shared by
/// inverse mass. Positions are not touched.
///
/// Why a separate pass and not the XPBD `γ` term inside the constraint (the phase 1.3 form):
/// with one pass per substep the `γ` term sees each substep's predicted move as speed, so a
/// still, loaded structure carries damper force. Elastic beams hid that as extra sag; with
/// yield capping the spring it showed as wrong forces at rest, and a damped beam yielded when
/// spring plus damper passed `F_y`. Here a still structure has zero relative speed and zero
/// damper force, and yield reads the spring alone (ER 001, phase 1.5 damping fix).
pub fn damp_beam(pos: &[Vec3], vel: &mut [Vec3], inv_mass: &[Real], a: usize, b: usize, damping: Real, h: Real) {
    let wa = inv_mass[a];
    let wb = inv_mass[b];
    let w = wa + wb;
    if w == 0.0 || damping == 0.0 {
        return;
    }
    let d = pos[b] - pos[a];
    let len = d.length();
    if len == 0.0 {
        return;
    }
    let dir = d / len;
    let v = dir.dot(vel[b] - vel[a]);
    let beta = damping * h;
    let j = beta * v / (1.0 + beta * w);
    vel[a] = vel[a] + dir * (wa * j);
    vel[b] = vel[b] - dir * (wb * j);
}

/// Advances one island by one fixed step in `n` substeps. Per substep: predict (gravity and
/// gathered forces into velocity, move), solve every beam once in order, derive velocity
/// from the change in position, then damp every beam once in order ([`damp_beam`]). Pinned
/// nodes (inverse mass zero) never move. Yield happens inside the substeps (the force cap in
/// [`solve_distance`], which sets the `YIELDED` flag). After the last substep, break is
/// checked once and the events returned ([`check_failure`]).
///
/// `last_lambda` ends up holding each beam's spring force from the last substep, `-Δλ / h²`,
/// in newtons, positive in tension. It never exceeds the beam's yield force, and it does not
/// include the damper (zero for a beam that just broke).
#[allow(clippy::needless_range_loop)] // explicit ascending index order is a determinism rule
pub fn solve_island(isl: &mut Island, n: u32) -> Vec<Event> {
    let h = STEP_SECONDS / n as Real;
    let nodes = isl.pos.len();
    let beams = isl.beam_a.len();
    let mut prev = isl.pos.clone();
    let flags_before = isl.beam_flags.clone();
    for _ in 0..n {
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
        for k in 0..beams {
            // No original length means no strain scale, so no cap (a generator bug 1.6 guards).
            let yield_elong = if isl.rest0[k] == 0.0 { Real::INFINITY } else { isl.yield_strain[k] * isl.rest0[k] };
            let (dl, yielded) = solve_distance(
                &mut isl.pos,
                &isl.inv_mass,
                isl.beam_a[k] as usize,
                isl.beam_b[k] as usize,
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
        for i in 0..nodes {
            if isl.inv_mass[i] == 0.0 {
                continue;
            }
            isl.vel[i] = (isl.pos[i] - prev[i]) / h;
        }
        for k in 0..beams {
            damp_beam(&isl.pos, &mut isl.vel, &isl.inv_mass, isl.beam_a[k] as usize, isl.beam_b[k] as usize, isl.damping[k], h);
        }
    }
    check_failure(isl, &flags_before)
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
        solve_island(&mut i1[0], 9);
        solve_island(&mut i2[0], 9);
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
