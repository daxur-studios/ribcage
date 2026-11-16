//! Phase 1.4: the cantilever test against the textbook deflection.
//!
//! The scene is the long square truss of `support/truss.rs`, pinned at one end with a load
//! on the other. Three things are compared:
//!
//! - the textbook formula (`textbook_tip`, derivation in its doc comment) against the exact
//!   static answer of the same truss (`exact_static_tip`, Newton's method on the force
//!   balance), which isolates the formula's own error;
//! - the solver's settled tip against both, at many substeps, which isolates the solver's;
//! - how the solver's error shrinks with substeps. With one pass per substep the settled
//!   structure is softer than the exact one by a fraction close to `K · (ω_max · h)²`, with
//!   `K` about 0.4 on every scene measured. At the rule's count `ω_max · h` is at most the
//!   substep constant `c`, so the rule's count reads about `1 + K c²` times the exact sag.
//!
//! Settling: beam damping is zero, so it cannot add sag (it does; see the record), and a
//! test-side drag `−drag · v` goes through `add_force` before each step. It is zero at rest,
//! so it cannot move where the truss settles. It lags one 60 Hz step, so at very high
//! substep counts (above about 100) it pumps the fastest modes; those runs switch it off
//! once the truss has settled.

#[path = "support/truss.rs"]
mod truss;

use beam_core::island::build_islands;
use beam_core::real::{Real, Vec3};
use beam_core::sim::{step, step_with_substeps};
use beam_core::STEP_SECONDS;
use beam_core::solver::substep_count;
use beam_core::world::{BeamSpec, World};
use truss::*;

/// The fastest beam frequency, as the substep rule computes it.
fn omega_max(w: &World) -> Real {
    let mut o: Real = 0.0;
    for b in 0..w.rest.len() {
        let inv = w.inv_mass[w.beam_a[b] as usize] + w.inv_mass[w.beam_b[b] as usize];
        o = o.max((inv / w.compliance[b]).sqrt());
    }
    o
}

/// Runs any world under `load` (newtons per node, added before every step) and a test-side
/// drag of `drag` newton-seconds per metre for the first `drag_secs`, for `seconds` in all;
/// `n` substeps, 0 meaning the rule. Returns the world and its fastest node speed.
fn run(mut w: World, load: &[Vec3], n: u32, seconds: u32, drag: Real, drag_secs: u32) -> (World, Real) {
    for s in 0..seconds * 60 {
        for i in 0..w.pos.len() {
            if w.inv_mass[i] == 0.0 {
                continue;
            }
            w.add_force(i as u32, load[i]);
            if s < drag_secs * 60 {
                let v = w.vel[i];
                w.add_force(i as u32, v * (-drag));
            }
        }
        if n == 0 {
            step(&mut w);
        } else {
            step_with_substeps(&mut w, n);
        }
    }
    let vmax = w.vel.iter().map(|v| v.length()).fold(0.0, Real::max);
    (w, vmax)
}

fn report(name: &str, make: &dyn Fn() -> World, load: Vec<Vec3>, drag: Real, ns: &[u32]) {
    let w0 = make();
    let (_lin, exact) = static_equilibrium(&w0, &load);
    // The node that moves most in the exact answer.
    let mut far = 0;
    let mut dmax = 0.0;
    for i in 0..w0.pos.len() {
        let d = (exact[i] - w0.pos[i]).length();
        if d > dmax {
            dmax = d;
            far = i;
        }
    }
    let om = omega_max(&w0);
    let rule = substep_count(&build_islands(&w0)[0]);
    println!("{name}: omega_max {om:.1} rule n {rule} exact far-node move {dmax:.6}");
    for &n in ns {
        let (w, v) = run(make(), &load, n, 30, drag, 20);
        let d = (w.pos[far] - w0.pos[far]).length();
        let wh = om / (60.0 * n as Real);
        println!("  n {n:>3}  wh {wh:.3}  ratio {:.4}  K {:.3}  vmax {v:.1e}", d / dmax, (d / dmax - 1.0) / (wh * wh));
    }
}

fn tip_load(spec: &TrussSpec) -> Vec<Vec3> {
    let mut load = vec![Vec3::ZERO; spec.nodes()];
    for t in spec.tip_nodes() {
        load[t] = Vec3::new(0.0, -spec.tip_force / 4.0, 0.0);
    }
    load
}

/// The textbook formula against the exact truss. Measured on the standard truss: textbook
/// 0.3318 m, exact linear 0.3283 m (formula 1.1% high), exact nonlinear 0.3268 m (formula
/// 1.5% high; the large deflection stiffens the truss slightly). The gap is the formula's
/// own approximation of an indeterminate truss, so the tolerance is 2%. Of the 0.33 m,
/// the 200 N tip load's `P L³ / (3 E I)` is 0.16 m; the truss's own 471 N of weight and
/// the shear of the diagonals make up the rest, all inside the formula.
#[test]
fn textbook_formula_matches_the_exact_truss() {
    let spec = TrussSpec::standard();
    assert!(spec.length() >= 10.0 * spec.width);
    let text = textbook_tip(&spec);
    let (lin, nl) = exact_static_tip(&spec);
    assert!((text / lin - 1.0).abs() < 0.02, "textbook {text} linear {lin}");
    assert!((text / nl - 1.0).abs() < 0.02, "textbook {text} nonlinear {nl}");
}

/// With enough substeps the solver settles on the textbook tip. Measured at 128 substeps:
/// 1.0% softer than the exact truss, 0.5% short of the textbook (the formula's +1.5% and
/// the solver's +1.0% partly cancel). Tolerance 2%: the formula's own error plus the
/// solver's remaining error at this count, with margin for either sign.
#[test]
fn settles_on_the_textbook_tip_with_enough_substeps() {
    let spec = TrussSpec::standard();
    let (w, vmax) = run(build(&spec), &tip_load(&spec), 128, 6, 10.0, 3);
    let tip = tip_deflection(&w, &spec);
    let text = textbook_tip(&spec);
    assert!(vmax < 1e-3, "still moving at {vmax} m/s");
    assert!((tip / text - 1.0).abs() < 0.02, "tip {tip} textbook {text}");
}

/// The error at the substep rule's count, and how it shrinks. Measured on the standard
/// truss (settled tip over exact nonlinear tip): 1.67 at 16 substeps, 1.43 at the rule's
/// 20, 1.17 at 32, 1.04 at 64, 1.01 at 128. The excess is `K (ω_max h)²` with `K` 0.415 to
/// 0.43 (the chain and flat truss of phase 1.3 give 0.41 and 0.42, square trusses with
/// length 6, 12 and 24 times the width 0.35 to 0.42). The stated tolerance at the rule's
/// count is therefore the law, not a percentage: softer than exact by `K c²` with `K`
/// between 0.38 and 0.46, never stiffer.
#[test]
fn error_shrinks_with_the_square_of_the_substep_length() {
    let spec = TrussSpec::standard();
    let w0 = build(&spec);
    let rule = substep_count(&build_islands(&w0)[0]);
    assert_eq!(rule, 20);
    let om = omega_max(&w0);
    let (_, nl) = exact_static_tip(&spec);
    let mut last_excess = Real::MAX;
    for n in [16u32, rule, 32, 64] {
        let (w, vmax) = run(build(&spec), &tip_load(&spec), n, 15, 10.0, 15);
        assert!(vmax < 1e-6, "{n} substeps still moving at {vmax} m/s");
        let excess = tip_deflection(&w, &spec) / nl - 1.0;
        let wh = om * STEP_SECONDS / n as Real;
        let k = excess / (wh * wh);
        assert!(excess > 0.0 && excess < last_excess, "{n} substeps excess {excess}");
        assert!(k > 0.38 && k < 0.46, "{n} substeps: K {k}");
        last_excess = excess;
    }
}

/// Prints the error constant `K` on the phase 1.3 scenes and on square trusses of three
/// lengths. Run with `cargo test --release --test cantilever -- --ignored --nocapture`.
#[test]
#[ignore]
fn measure_error_constant() {
    let spec = TrussSpec::standard();

    // Ten-link chain from 1.3, no beam damping.
    let mut w = World::new();
    w.add_node(Vec3::new(0.0, 0.0, 0.0), 0.0, 0.1);
    for i in 1..=10u32 {
        w.add_node(Vec3::new(0.0, -(i as Real), 0.0), 2.0, 0.1);
        w.add_beam(i - 1, i, BeamSpec { rest: 1.0, compliance: 1e-5, damping: 0.0, yield_strain: 1.0, break_strain: 2.0 });
    }
    let load = vec![Vec3::ZERO; w.pos.len()];
    let chain = move || { let mut w = World::new(); w.add_node(Vec3::new(0.0, 0.0, 0.0), 0.0, 0.1); for i in 1..=10u32 { w.add_node(Vec3::new(0.0, -(i as Real), 0.0), 2.0, 0.1); w.add_beam(i - 1, i, BeamSpec { rest: 1.0, compliance: 1e-5, damping: 0.0, yield_strain: 1.0, break_strain: 2.0 }); } w };
    report("chain", &chain, load, 2.0, &[4, 8, 11, 16, 32]);

    // Flat 1.3 truss, no beam damping.
    let flat = || {
    let mut w = World::new();
    let bays = 8u32;
    let d = 0.5;
    for i in 0..=bays {
        let x = i as Real * d;
        let inv = if i == 0 { 0.0 } else if i == bays { 0.1 } else { 1.0 };
        w.add_node(Vec3::new(x, 0.0, 0.0), inv, 0.03);
        w.add_node(Vec3::new(x, d, 0.0), inv, 0.03);
    }
    let s = |rest: Real| BeamSpec { rest, compliance: 1e-6, damping: 0.0, yield_strain: 1.0, break_strain: 2.0 };
    let diag = (2.0 as Real).sqrt() * d;
    for i in 0..bays {
        let (b0, t0, b1, t1) = (2 * i, 2 * i + 1, 2 * i + 2, 2 * i + 3);
        w.add_beam(b0, b1, s(d));
        w.add_beam(t0, t1, s(d));
        w.add_beam(b1, t1, s(d));
        w.add_beam(b0, t1, s(diag));
    }
    w
    };
    let load = vec![Vec3::ZERO; flat().pos.len()];
    report("flat truss", &flat, load, 10.0, &[8, 16, 24, 32, 64]);

    // Square trusses of different slenderness.
    for &bays in &[6u32, 12, 24] {
        let sp = TrussSpec { bays, ..spec };
        let w = build(&sp);
        let mut load = vec![Vec3::ZERO; w.pos.len()];
        for t in sp.tip_nodes() {
            load[t] = Vec3::new(0.0, -sp.tip_force / 4.0, 0.0);
        }
        report(&format!("square truss L/b {bays}"), &|| build(&sp), load, 10.0, &[16, 20, 32, 64]);
    }
}
