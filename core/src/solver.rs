//! XPBD beam solve in the "small steps" form: one pass over the beams per substep, and many
//! substeps. A pure function of the packed island buffer (layout rule 5); it never reads the
//! world arrays. One distance function for every beam (rule 6). Only add, subtract, multiply,
//! divide and square root on `Real`, nodes and beams in ascending packed order, no fused
//! multiply-add (determinism list). Yield and break arrive in phase 1.5; clamps in 1.6.

use crate::island::Island;
use crate::real::{Real, Vec3};
use crate::sim::gravity_at;
use crate::STEP_SECONDS;

/// Substep rule constant: the count is the smallest `n` with `ω_max · h ≤ SUBSTEP_C`.
/// **[assumption]** `c` near 1, chosen before any measurement. Phase 1.4 measures it.
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
/// `a` and `b` index `pos`, `prev` and `inv_mass`. `prev` holds positions at the start of
/// this substep. Returns `Δλ`; each endpoint moves along the beam by its inverse mass
/// times `Δλ`, in opposite senses.
///
/// `C` is current length minus rest, `w` the sum of inverse masses, `α = compliance / h²`,
/// and `γ = compliance · damping / h` (the XPBD paper's `α̃ β̃ / h` with `β̃ = h² · damping`),
/// so `damping` is a physical coefficient in newton-seconds per metre and means the same
/// thing at any substep count:
///
/// `Δλ = (-C - γ · (direction · relative movement this substep)) / ((1 + γ) · w + α)`
#[allow(clippy::too_many_arguments)]
pub fn solve_distance(
    pos: &mut [Vec3],
    prev: &[Vec3],
    inv_mass: &[Real],
    a: usize,
    b: usize,
    rest: Real,
    compliance: Real,
    damping: Real,
    h: Real,
) -> Real {
    let wa = inv_mass[a];
    let wb = inv_mass[b];
    let w = wa + wb;
    if w == 0.0 {
        return 0.0;
    }
    let d = pos[b] - pos[a];
    let len = d.length();
    if len == 0.0 {
        // No direction to push along. Coincident endpoints are a generator bug; 1.6 guards it.
        return 0.0;
    }
    let dir = d / len;
    let c = len - rest;
    let alpha = compliance / (h * h);
    let gamma = compliance * damping / h;
    let rel = (pos[b] - prev[b]) - (pos[a] - prev[a]);
    let dl = (-c - gamma * dir.dot(rel)) / ((1.0 + gamma) * w + alpha);
    pos[a] = pos[a] - dir * (wa * dl);
    pos[b] = pos[b] + dir * (wb * dl);
    dl
}

/// Advances one island by one fixed step in `n` substeps. Per substep: predict (gravity and
/// gathered forces into velocity, move), solve every beam once in order, derive velocity
/// from the change in position. Pinned nodes (inverse mass zero) never move.
///
/// `last_lambda` ends up holding each beam's constraint force from the last substep,
/// `-Δλ / h²`, in newtons, positive in tension.
#[allow(clippy::needless_range_loop)] // explicit ascending index order is a determinism rule
pub fn solve_island(isl: &mut Island, n: u32) {
    let h = STEP_SECONDS / n as Real;
    let nodes = isl.pos.len();
    let mut prev = isl.pos.clone();
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
        for k in 0..isl.beam_a.len() {
            let dl = solve_distance(
                &mut isl.pos,
                &prev,
                &isl.inv_mass,
                isl.beam_a[k] as usize,
                isl.beam_b[k] as usize,
                isl.rest[k],
                isl.compliance[k],
                isl.damping[k],
                h,
            );
            isl.last_lambda[k] = -dl / (h * h);
        }
        for i in 0..nodes {
            if isl.inv_mass[i] == 0.0 {
                continue;
            }
            isl.vel[i] = (isl.pos[i] - prev[i]) / h;
        }
    }
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
