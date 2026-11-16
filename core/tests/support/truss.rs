//! Test-side generator for the long square truss of the cantilever test
//! (docs/research/core-data-layout.md, "The cantilever test"), its textbook tip deflection,
//! and an exact static solve of any world, to separate formula error from solver error.
//! Shared by `tests/cantilever.rs` and `examples/trace.rs` (through `#[path]`); not part of
//! the core library.

#![allow(dead_code)]

use beam_core::real::{Real, Vec3};
use beam_core::sim::gravity_at;
use beam_core::world::{BeamSpec, World};

/// The truss: `bays` square-section bays of length `spacing`, `width` wide and deep, made of
/// one material with axial stiffness `ea` (newtons; a member of length `l` gets compliance
/// `l / ea`). Every node has mass `node_mass`. Frame 0 is pinned; `tip_force` newtons hang
/// straight down on the last frame, a quarter on each of its four nodes.
#[derive(Clone, Copy, Debug)]
pub struct TrussSpec {
    pub width: Real,
    pub spacing: Real,
    pub bays: u32,
    pub ea: Real,
    pub node_mass: Real,
    /// Newton-seconds per metre on every member.
    pub damping: Real,
    pub tip_force: Real,
}

impl TrussSpec {
    /// The truss the cantilever test uses: 0.5 m square section, twelve 0.5 m bays (6 m
    /// long, twelve times the width), 1 kg nodes, EA = 360 kN, a 200 N tip load. Its
    /// shortest members ask the substep rule for 20 substeps at c = 1.
    pub fn standard() -> TrussSpec {
        TrussSpec { width: 0.5, spacing: 0.5, bays: 12, ea: 3.6e5, node_mass: 1.0, damping: 0.0, tip_force: 200.0 }
    }

    pub fn length(&self) -> Real {
        self.spacing * self.bays as Real
    }

    pub fn nodes(&self) -> usize {
        4 * (self.bays as usize + 1)
    }

    /// The four tip node indices.
    pub fn tip_nodes(&self) -> [usize; 4] {
        let t = 4 * self.bays as usize;
        [t, t + 1, t + 2, t + 3]
    }
}

/// Node `4 i + j` sits in frame `i` at `x = i · spacing`; corner `j` runs 0 = bottom near,
/// 1 = top near, 2 = top far, 3 = bottom far (y up, z across).
pub fn corner(spec: &TrussSpec, i: u32, j: u32) -> Vec3 {
    let w = spec.width;
    let (y, z) = match j {
        0 => (0.0, 0.0),
        1 => (w, 0.0),
        2 => (w, w),
        _ => (0.0, w),
    };
    Vec3::new(i as Real * spec.spacing, y, z)
}

/// Members as `(a, b)` node pairs, in the order the world adds them:
/// - per frame `i` (all of them, the pinned root too): the four sides, then one diagonal
///   across the frame (corner 0 to corner 2);
/// - per bay `i` to `i + 1`: the four rails, then one diagonal on each face. The two side
///   faces (near, far) both run bottom at `i` to top at `i + 1`, so they form two matching
///   N-trusses; the top and bottom faces run near at `i` to far at `i + 1`.
pub fn members(spec: &TrussSpec) -> Vec<(u32, u32)> {
    let mut m = Vec::new();
    let n = |i: u32, j: u32| 4 * i + j;
    for i in 0..=spec.bays {
        m.push((n(i, 0), n(i, 1)));
        m.push((n(i, 1), n(i, 2)));
        m.push((n(i, 2), n(i, 3)));
        m.push((n(i, 3), n(i, 0)));
        m.push((n(i, 0), n(i, 2)));
    }
    for i in 0..spec.bays {
        for j in 0..4 {
            m.push((n(i, j), n(i + 1, j)));
        }
        m.push((n(i, 0), n(i + 1, 1))); // near side face
        m.push((n(i, 3), n(i + 1, 2))); // far side face
        m.push((n(i, 1), n(i + 1, 2))); // top face
        m.push((n(i, 0), n(i + 1, 3))); // bottom face
    }
    m
}

fn build_with(spec: &TrussSpec, members: Vec<(u32, u32)>) -> World {
    let mut w = World::new();
    for i in 0..=spec.bays {
        let inv = if i == 0 { 0.0 } else { 1.0 / spec.node_mass };
        for j in 0..4 {
            w.add_node(corner(spec, i, j), inv, 0.02);
        }
    }
    for (a, b) in members {
        let rest = (w.pos[b as usize] - w.pos[a as usize]).length();
        w.add_beam(a, b, BeamSpec { rest, compliance: rest / spec.ea, damping: spec.damping, yield_strain: 1.0, break_strain: 2.0 });
    }
    w
}

/// Builds the truss at rest. Frame 0 is pinned. The tip load is not added here: it goes
/// through `World::add_force` before every step ([`apply_tip_load`]).
pub fn build(spec: &TrussSpec) -> World {
    build_with(spec, members(spec))
}

/// The same truss with its members added in reverse order (tip first), to measure how
/// much the solve order matters.
pub fn build_reversed(spec: &TrussSpec) -> World {
    build_with(spec, members(spec).into_iter().rev().collect())
}

/// The tip load, through the one door for forces. Call before every step.
pub fn apply_tip_load(w: &mut World, spec: &TrussSpec) {
    for t in spec.tip_nodes() {
        w.add_force(t as u32, Vec3::new(0.0, -spec.tip_force / 4.0, 0.0));
    }
}

/// Mean downward movement of the four tip nodes from their rest positions. The mean
/// cancels any small twist of the tip frame.
pub fn tip_deflection(w: &World, spec: &TrussSpec) -> Real {
    let mut d = 0.0;
    for t in spec.tip_nodes() {
        d += corner(spec, spec.bays, (t % 4) as u32).y - w.pos[t].y;
    }
    d / 4.0
}

fn g(spec: &TrussSpec) -> Real {
    -gravity_at(corner(spec, 0, 0)).y
}

/// The textbook tip deflection, positive downward.
///
/// Derivation. The load is a set of point loads: each free frame `i` (at `a = i · s`)
/// carries its own weight `Q_i = 4 m g`, and the tip frame also carries `P`. Linear
/// superposition over the point loads, each contributing a bending and a shear part:
///
/// - **Bending.** A point load `Q` at `a` on a cantilever of length `L` moves the tip by
///   `Q a² (3L − a) / (6 E I)`; at `a = L` this is the familiar `P L³ / (3 E I)`. The
///   section is the four rails, each of area `A`, at `±w/2` from the neutral axis:
///   `I = 4 A (w/2)² = A w²`, so `E I = EA · w²`. Frame and diagonal members carry no
///   bending stress in this model.
/// - **Shear.** The vertical shear `V` is carried by the two side faces, `V/2` each. Each
///   side face is a plane N-truss: per bay, a diagonal of length `d = √(s² + w²)` carries
///   `(V/2)(d / w)` and a vertical of length `w` carries `V/2`. By the unit-load method
///   (real force times unit-load force times `l / EA`, over both faces), one bay deflects
///   `V (d³ / w² + w) / (2 EA)`. A load at frame `i` puts its shear through bays `1..=i`.
///
/// The truss is statically indeterminate (each bay has one more member than it needs), so
/// this formula approximates the truss; [`exact_static_tip`] measures how well.
pub fn textbook_tip(spec: &TrussSpec) -> Real {
    let (s, w, ea) = (spec.spacing, spec.width, spec.ea);
    let l = spec.length();
    let ei = ea * w * w;
    let d = (s * s + w * w).sqrt();
    let shear_per_bay = (d * d * d / (w * w) + w) / (2.0 * ea);
    let mut tip = 0.0;
    for i in 1..=spec.bays {
        let a = i as Real * s;
        let mut q = 4.0 * spec.node_mass * g(spec);
        if i == spec.bays {
            q += spec.tip_force;
        }
        tip += q * a * a * (3.0 * l - a) / (6.0 * ei) + q * i as Real * shear_per_bay;
    }
    tip
}

/// The exact static tip deflection of this truss under the same loads, from
/// [`static_equilibrium`]. Returns `(linear, nonlinear)`, positive downward.
pub fn exact_static_tip(spec: &TrussSpec) -> (Real, Real) {
    let w = build(spec);
    let mut load = vec![Vec3::ZERO; w.pos.len()];
    for t in spec.tip_nodes() {
        load[t] = Vec3::new(0.0, -spec.tip_force / 4.0, 0.0);
    }
    let (lin, nl) = static_equilibrium(&w, &load);
    let tip = |x: &Vec<Vec3>| {
        let mut d = 0.0;
        for t in spec.tip_nodes() {
            d += w.pos[t].y - x[t].y;
        }
        d / 4.0
    };
    (tip(&lin), tip(&nl))
}

/// The exact static equilibrium of any world at rest under gravity plus `load` (newtons per
/// node), by Newton's method on the force balance with the solver's own springs: a beam's
/// tension is `(length − rest) / compliance`. Pinned nodes stay put. After one iteration
/// from the rest shape the positions are the linear (small-deflection) answer; converged,
/// they are the nonlinear equilibrium the solver should settle to. Returns
/// `(linear, nonlinear)` positions. Dense Gaussian elimination with a tiny diagonal
/// regulariser so directions with no stiffness yet (a planar scene's depth, a slack chain's
/// sideways) stay solvable. Test-side only, but still only `+ − × ÷ √`.
pub fn static_equilibrium(w: &World, load: &[Vec3]) -> (Vec<Vec3>, Vec<Vec3>) {
    let nodes = w.pos.len();
    let none = usize::MAX;
    let mut slot = vec![none; nodes];
    let mut free = 0;
    for i in 0..nodes {
        if w.inv_mass[i] != 0.0 {
            slot[i] = 3 * free;
            free += 1;
        }
    }
    let dof = 3 * free;
    let mut x = w.pos.clone();
    let mut linear = Vec::new();
    let mut kmax: Real = 0.0;
    for b in 0..w.rest.len() {
        kmax = kmax.max(1.0 / w.compliance[b]);
    }
    for iter in 0..40 {
        // Residual: net force on each free node. Tangent stiffness: k e eᵀ + (T / l)(I − e eᵀ).
        let mut r = vec![0.0 as Real; dof];
        let mut k = vec![0.0 as Real; dof * dof];
        for i in 0..nodes {
            if slot[i] == none {
                continue;
            }
            let f = gravity_at(x[i]) / w.inv_mass[i] + load[i];
            r[slot[i]] += f.x;
            r[slot[i] + 1] += f.y;
            r[slot[i] + 2] += f.z;
            for c in 0..3 {
                k[(slot[i] + c) * dof + slot[i] + c] += 1e-12 * kmax;
            }
        }
        for m in 0..w.rest.len() {
            let (a, b) = (w.beam_a[m] as usize, w.beam_b[m] as usize);
            let dvec = x[b] - x[a];
            let len = dvec.length();
            let e = [dvec.x / len, dvec.y / len, dvec.z / len];
            let stiff = 1.0 / w.compliance[m];
            let t = stiff * (len - w.rest[m]);
            let (ia, ib) = (slot[a], slot[b]);
            // Force on a is +T e, on b is −T e.
            for c in 0..3 {
                if ia != none {
                    r[ia + c] += t * e[c];
                }
                if ib != none {
                    r[ib + c] -= t * e[c];
                }
            }
            for p in 0..3 {
                for q in 0..3 {
                    let id = if p == q { 1.0 } else { 0.0 };
                    let kpq = stiff * e[p] * e[q] + (t / len) * (id - e[p] * e[q]);
                    if ia != none {
                        k[(ia + p) * dof + ia + q] += kpq;
                    }
                    if ib != none {
                        k[(ib + p) * dof + ib + q] += kpq;
                    }
                    if ia != none && ib != none {
                        k[(ia + p) * dof + ib + q] -= kpq;
                        k[(ib + p) * dof + ia + q] -= kpq;
                    }
                }
            }
        }
        let dx = gauss_solve(&mut k, &mut r, dof);
        let mut step2 = 0.0;
        for i in 0..nodes {
            if slot[i] == none {
                continue;
            }
            let o = slot[i];
            x[i] = x[i] + Vec3::new(dx[o], dx[o + 1], dx[o + 2]);
            step2 += dx[o] * dx[o] + dx[o + 1] * dx[o + 1] + dx[o + 2] * dx[o + 2];
        }
        if iter == 0 {
            linear = x.clone();
        }
        if step2 < 1e-26 {
            break;
        }
    }
    (linear, x)
}

/// Solves `K u = r` in place by Gaussian elimination with partial pivoting.
fn gauss_solve(k: &mut [Real], r: &mut [Real], n: usize) -> Vec<Real> {
    let abs = |v: Real| if v < 0.0 { -v } else { v };
    for col in 0..n {
        let mut piv = col;
        for row in col + 1..n {
            if abs(k[row * n + col]) > abs(k[piv * n + col]) {
                piv = row;
            }
        }
        if piv != col {
            for c in 0..n {
                k.swap(col * n + c, piv * n + c);
            }
            r.swap(col, piv);
        }
        let d = k[col * n + col];
        for row in col + 1..n {
            let f = k[row * n + col] / d;
            if f == 0.0 {
                continue;
            }
            for c in col..n {
                k[row * n + c] -= f * k[col * n + c];
            }
            r[row] -= f * r[col];
        }
    }
    let mut u = vec![0.0; n];
    for row in (0..n).rev() {
        let mut s = r[row];
        for c in row + 1..n {
            s -= k[row * n + c] * u[c];
        }
        u[row] = s / k[row * n + row];
    }
    u
}
