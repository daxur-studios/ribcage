//! The stress estimate (plan 005, phase 5.7): per seam, the force and moment its parent puts on
//! the child's subtree, and from them an estimated peak load ratio of the seam's own beams.
//! Read-only: it reads the step the solver just took and writes only its own fields, which are
//! not in either hash. Off by default; nothing here runs unless [`Estimate::on`] is set.
//!
//! **The cut.** Per movable node with a segment, the force the rest of the craft put on it over
//! the step is what is left after the outside world: `m (v - v_prev) / dt - (m g + f + J / dt)`,
//! with `f` the step's external force (thrust, host forces) and `J` the step's contact impulse
//! from the solver. Summed over a subtree, internal forces cancel and what is left is the force
//! across the subtree's seam to its parent (plus any beam that skips a seam: loops read high).
//! Moments are summed about the origin and moved to the seam's centre.
//!
//! **The strain step.** The plan's section formula wants E and the extreme-fibre distance, which
//! the core does not have; the seam's beams are here, so the load is spread over them as a stiff
//! plate would spread it: the child moves rigidly by a small `(d, θ)` against the parent, each
//! beam stretches by `u·(d + θ×r)` and pulls back with its own spring rate `1/compliance`. Six
//! unknowns, one small solve per seam. The estimated ratio is the worst beam's `|T| / F_yield`.
//!
//! Deterministic maths only: `+ - * /` and `sqrt` on `Real`, fixed iteration order.

use crate::real::{Real, Vec3};
use crate::segment::NO_SEGMENT;
use crate::world::{World, BEAM_ALIVE, BEAM_BROKEN, NODE_ALIVE, NODE_FROZEN};
use crate::STEP_SECONDS;

/// The estimate's switch, its per-step inputs and its outputs. Not hashed, not recorded.
#[derive(Clone, Debug, Default)]
pub struct Estimate {
    /// When set, every step computes the estimate. Off by default.
    pub on: bool,
    /// Node velocities at the start of the step.
    pub prev_vel: Vec<Vec3>,
    /// External node forces of the step (after thrust went in), before they are cleared.
    pub ext_force: Vec<Vec3>,
    /// Summed contact impulse per node over the step, N·s, from the solver.
    pub contact: Vec<Vec3>,
    /// Per seam: force from the parent side on the child's subtree, newtons (step average).
    pub seam_force: Vec<Vec3>,
    /// Per seam: moment from the parent side on the child's subtree about [`Self::seam_centre`].
    pub seam_moment: Vec<Vec3>,
    /// Per seam: where the moment is taken, the mean of its beams' midpoints (the child segment's
    /// centre when it has no beams).
    pub seam_centre: Vec<Vec3>,
    /// Per seam: estimated worst `|T| / F_yield` over its live beams; 0 with no beams.
    pub seam_ratio: Vec<Real>,
    /// Per seam: measured worst `|last_lambda| / F_yield` over the same beams, the solver's own
    /// spring force (no damper) in the last substep; NaN when none of them is solved as a beam.
    pub seam_measured: Vec<Real>,
    /// Per seam: the measured beams' resultant on the child, `Σ -λ u`, newtons.
    pub seam_measured_force: Vec<Vec3>,
    /// Per segment: the worst estimated ratio over the seams it touches.
    pub seg_ratio: Vec<Real>,
}

/// Called at the start of a step: remembers the velocities. No-op when off.
pub fn begin_step(world: &mut World) {
    if !world.est.on {
        return;
    }
    let n = world.vel.len();
    let mut prev = core::mem::take(&mut world.est.prev_vel);
    prev.clear();
    prev.extend_from_slice(&world.vel);
    world.est.prev_vel = prev;
    world.est.contact.clear();
    world.est.contact.resize(n, Vec3::ZERO);
}

/// Called after thrust went in, before the solve: remembers the external forces.
pub fn take_forces(world: &mut World) {
    if !world.est.on {
        return;
    }
    let mut f = core::mem::take(&mut world.est.ext_force);
    f.clear();
    f.extend_from_slice(&world.force);
    world.est.ext_force = f;
}

/// Called after the islands are copied back: computes the estimate from the step just taken.
pub fn finish_step(world: &mut World) {
    if !world.est.on {
        return;
    }
    let mut est = core::mem::take(&mut world.est);
    compute(world, &mut est);
    world.est = est;
}

fn compute(world: &World, est: &mut Estimate) {
    let segs = world.seg_parent.len();
    let seams = world.seam_a.len();
    let dt = STEP_SECONDS;
    let g = crate::sim::gravity_at(Vec3::ZERO);

    // Per segment: the force and the moment about the origin the rest of the craft puts on it.
    let mut force = vec![Vec3::ZERO; segs];
    let mut moment = vec![Vec3::ZERO; segs];
    for i in 0..world.pos.len() {
        let s = world.node_segment[i];
        if s == NO_SEGMENT || world.node_flags[i] & NODE_ALIVE == 0 || world.node_flags[i] & NODE_FROZEN != 0 {
            continue;
        }
        let w = world.inv_mass[i];
        if w == 0.0 {
            continue;
        }
        let m = 1.0 / w;
        let accel = (world.vel[i] - est.prev_vel[i]) * (m / dt);
        let outside = g * m + est.ext_force[i] + est.contact[i] / dt;
        let r = accel - outside;
        force[s as usize] = force[s as usize] + r;
        moment[s as usize] = moment[s as usize] + world.pos[i].cross(r);
    }
    // A beam from a segment node to a node with no segment (or a pinned one) is outside too:
    // its spring force is taken off, so it does not show up as seam load.
    for k in 0..world.beam_a.len() {
        if world.beam_flags[k] & BEAM_ALIVE == 0 || world.beam_flags[k] & BEAM_BROKEN != 0 {
            continue;
        }
        let (a, b) = (world.beam_a[k] as usize, world.beam_b[k] as usize);
        let (sa, sb) = (world.node_segment[a], world.node_segment[b]);
        let ins = |n: usize, s: u32| s != NO_SEGMENT && world.inv_mass[n] != 0.0;
        let (ia, ib) = (ins(a, sa), ins(b, sb));
        if ia == ib {
            continue;
        }
        let d = world.pos[b] - world.pos[a];
        let len = d.length();
        if !(len > 0.0) {
            continue;
        }
        let u = d / len;
        let lam = world.last_lambda[k];
        // Tension pulls a toward b.
        let (n, s, f) = if ia { (a, sa, u * lam) } else { (b, sb, u * -lam) };
        force[s as usize] = force[s as usize] - f;
        moment[s as usize] = moment[s as usize] - world.pos[n].cross(f);
    }
    // Leaves to root: a parent's index is always lower than its child's.
    for s in (0..segs).rev() {
        let p = world.seg_parent[s];
        if p != NO_SEGMENT {
            force[p as usize] = force[p as usize] + force[s];
            moment[p as usize] = moment[p as usize] + moment[s];
        }
    }

    // The seams' beams, gathered in beam order.
    let mut beams: Vec<Vec<u32>> = vec![Vec::new(); seams];
    for k in 0..world.beam_a.len() {
        if world.beam_flags[k] & BEAM_ALIVE == 0 || world.beam_flags[k] & BEAM_BROKEN != 0 {
            continue;
        }
        if let Some(seam) = world.beam_seam(k as u32) {
            beams[seam as usize].push(k as u32);
        }
    }

    est.seam_force.clear();
    est.seam_moment.clear();
    est.seam_centre.clear();
    est.seam_ratio.clear();
    est.seam_measured.clear();
    est.seam_measured_force.clear();
    for seam in 0..seams {
        let child = world.seam_b[seam] as usize;
        let list = &beams[seam];
        let centre = if list.is_empty() {
            world.segment_centre_and_inertia(child as u32).map(|p| p.centre).unwrap_or(Vec3::ZERO)
        } else {
            let mut c = Vec3::ZERO;
            for &k in list {
                let k = k as usize;
                c = c + (world.pos[world.beam_a[k] as usize] + world.pos[world.beam_b[k] as usize]) * 0.5;
            }
            c / list.len() as Real
        };
        let f = force[child];
        let m = moment[child] - centre.cross(f);
        let (ratio, measured, measured_force) = seam_beams(world, list, child as u32, centre, f, m);
        est.seam_force.push(f);
        est.seam_moment.push(m);
        est.seam_centre.push(centre);
        est.seam_ratio.push(ratio);
        est.seam_measured.push(measured);
        est.seam_measured_force.push(measured_force);
    }
    est.seg_ratio.clear();
    est.seg_ratio.resize(segs, 0.0);
    for seam in 0..seams {
        let r = est.seam_ratio[seam];
        for s in [world.seam_a[seam] as usize, world.seam_b[seam] as usize] {
            if r > est.seg_ratio[s] {
                est.seg_ratio[s] = r;
            }
        }
    }
}

/// The force a beam can take before it yields (or breaks, with no yield set), newtons.
fn yield_force(world: &World, k: usize) -> Real {
    let strain = if world.yield_strain[k] > 0.0 { world.yield_strain[k] } else { world.break_strain[k] };
    let c = world.compliance[k];
    if !(strain > 0.0) || !(c > 0.0) {
        return 0.0;
    }
    strain * world.rest0[k] / c
}

/// The stiffness spread of one seam's load over its beams. Returns the estimated worst ratio, the
/// measured worst ratio and the measured resultant on the child.
fn seam_beams(world: &World, list: &[u32], child: u32, centre: Vec3, f: Vec3, m: Vec3) -> (Real, Real, Vec3) {
    if list.is_empty() {
        return (0.0, Real::NAN, Vec3::ZERO);
    }
    // Per beam: u from the parent end to the child end, r from the centre to the child end.
    let mut us = Vec::with_capacity(list.len());
    let mut rs = Vec::with_capacity(list.len());
    let mut r2 = 0.0;
    for &k in list {
        let k = k as usize;
        let (a, b) = (world.beam_a[k] as usize, world.beam_b[k] as usize);
        let (pa, pc) = if world.node_segment[b] == child { (a, b) } else { (b, a) };
        let d = world.pos[pc] - world.pos[pa];
        let len = d.length();
        let u = if len > 0.0 { d / len } else { Vec3::ZERO };
        let r = world.pos[pc] - centre;
        r2 += r.length_squared();
        us.push(u);
        rs.push(r);
    }
    let l = (r2 / list.len() as Real).sqrt();
    let l = if l > 1e-9 { l } else { 1.0 };
    let gvec = |i: usize| -> [Real; 6] {
        let (u, t) = (us[i], rs[i].cross(us[i]) / l);
        [u.x, u.y, u.z, t.x, t.y, t.z]
    };
    let stiff = |k: usize| -> Real {
        let c = world.compliance[k];
        1.0 / if c > 1e-15 { c } else { 1e-15 }
    };
    // Elastic spread first; a beam the spread puts past its yield force is held at it (it
    // yields, as the solver's beams do) and the rest is spread again over the others. At most a
    // few rounds: each one caps at least one more beam or stops.
    let fys: Vec<Real> = list.iter().map(|&k| yield_force(world, k as usize)).collect();
    let mut capped: Vec<Option<Real>> = vec![None; list.len()];
    let ml = m / l;
    let mut t_all = vec![0.0 as Real; list.len()];
    for _round in 0..8 {
        let mut kk = [[0.0 as Real; 6]; 6];
        let mut rhs = [-f.x, -f.y, -f.z, -ml.x, -ml.y, -ml.z];
        for (i, &k) in list.iter().enumerate() {
            let gi = gvec(i);
            match capped[i] {
                Some(tc) => {
                    for r in 0..6 {
                        rhs[r] -= tc * gi[r];
                    }
                }
                None => {
                    let ki = stiff(k as usize);
                    for r in 0..6 {
                        for c in 0..6 {
                            kk[r][c] += ki * gi[r] * gi[c];
                        }
                    }
                }
            }
        }
        let trace = (0..6).map(|i| kk[i][i]).sum::<Real>();
        for i in 0..6 {
            kk[i][i] += 1e-9 * trace / 6.0 + 1e-300;
        }
        let x = solve6(kk, rhs);
        let mut more = false;
        for (i, &k) in list.iter().enumerate() {
            if let Some(tc) = capped[i] {
                t_all[i] = tc;
                continue;
            }
            let gi = gvec(i);
            let mut e = 0.0;
            for j in 0..6 {
                e += gi[j] * x[j];
            }
            let t = stiff(k as usize) * e;
            t_all[i] = t;
            let fy = fys[i];
            if fy > 0.0 && t.abs() > fy {
                capped[i] = Some(if t > 0.0 { fy } else { -fy });
                more = true;
            }
        }
        if !more || capped.iter().all(|c| c.is_some()) {
            break;
        }
    }
    let mut worst: Real = 0.0;
    let mut measured: Real = Real::NAN;
    let mut measured_force = Vec3::ZERO;
    for (i, &k) in list.iter().enumerate() {
        let k = k as usize;
        let t = t_all[i];
        let fy = fys[i];
        if fy > 0.0 {
            let r = t.abs() / fy;
            if r > worst {
                worst = r;
            }
        }
        // Both ends on one body: the beam is not solved, its λ means nothing.
        let (a, b) = (world.beam_a[k] as usize, world.beam_b[k] as usize);
        let nb = crate::body::NO_BODY;
        let skipped = world.node_body[a] != nb && world.node_body[a] == world.node_body[b];
        if !skipped {
            let lam = world.last_lambda[k];
            measured_force = measured_force - us[i] * lam;
            if fy > 0.0 {
                let r = lam.abs() / fy;
                if measured.is_nan() || r > measured {
                    measured = r;
                }
            } else if measured.is_nan() {
                measured = 0.0;
            }
        }
    }
    (worst, measured, measured_force)
}

/// Gaussian elimination with partial pivoting on a 6×6 system.
fn solve6(mut a: [[Real; 6]; 6], mut b: [Real; 6]) -> [Real; 6] {
    for col in 0..6 {
        let mut piv = col;
        for r in col + 1..6 {
            if a[r][col].abs() > a[piv][col].abs() {
                piv = r;
            }
        }
        a.swap(col, piv);
        b.swap(col, piv);
        let d = a[col][col];
        if d == 0.0 {
            continue;
        }
        for r in col + 1..6 {
            let f = a[r][col] / d;
            for c in col..6 {
                a[r][c] -= f * a[col][c];
            }
            b[r] -= f * b[col];
        }
    }
    let mut x = [0.0; 6];
    for r in (0..6).rev() {
        let mut s = b[r];
        for c in r + 1..6 {
            s -= a[r][c] * x[c];
        }
        x[r] = if a[r][r] != 0.0 { s / a[r][r] } else { 0.0 };
    }
    x
}
