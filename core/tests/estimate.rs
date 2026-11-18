//! Phase 5.7: the stress estimate. The cantilever truss cut into one segment per frame: the
//! free-body seam force matches the beams' own resultant and the exact static answer, the
//! estimated ratio tracks the measured one, a beam that skips a seam reads high, and switching
//! the estimate on changes no hash.

#[path = "support/truss.rs"]
mod truss;

use beam_core::real::{Real, Vec3};
use beam_core::segment::NO_SEGMENT;
use beam_core::sim::{physics_hash, state_hash, step};
use beam_core::world::{BeamSpec, World};
use truss::{apply_tip_load, build, TrussSpec};

/// The standard truss with damping, cut into segments: frame `i` (1..=bays) is segment `i - 1`,
/// a child of the frame before it. Frame 0 is pinned and has none.
fn cut_truss(spec: &TrussSpec) -> World {
    let mut w = build(spec);
    for i in 1..=spec.bays {
        let parent = if i == 1 { NO_SEGMENT } else { i - 2 };
        let s = w.add_segment(parent, 0, 0.0, 0.0).unwrap();
        for j in 0..4 {
            assert!(w.assign_node_segment(4 * i + j, s));
        }
    }
    w
}

fn spec() -> TrussSpec {
    TrussSpec { damping: 50.0, ..TrussSpec::standard() }
}

/// Runs with the tip load and a test-side drag for the first `drag_steps`.
fn run(w: &mut World, spec: &TrussSpec, steps: u32, drag_steps: u32) {
    for s in 0..steps {
        apply_tip_load(w, spec);
        if s < drag_steps {
            for i in 0..w.pos.len() {
                if w.inv_mass[i] != 0.0 {
                    let v = w.vel[i];
                    w.add_force(i as u32, v * -20.0);
                }
            }
        }
        step(w);
    }
}

fn close(a: Vec3, b: Vec3, tol: Real) -> bool {
    (a - b).length() <= tol
}

#[test]
fn seam_force_is_the_weight_and_load_beyond_it() {
    let spec = spec();
    let mut w = cut_truss(&spec);
    w.est.on = true;
    run(&mut w, &spec, 900, 600);
    let est = &w.est;
    assert_eq!(est.seam_force.len(), (spec.bays - 1) as usize);
    for seam in 0..est.seam_force.len() {
        // Seam `seam` holds frames seam+2 ..= bays: their weight plus the tip load, upward.
        let frames = (spec.bays - 1 - seam as u32) as Real;
        let up = frames * 4.0 * spec.node_mass * 9.81 + spec.tip_force;
        let f = est.seam_force[seam];
        assert!(close(f, Vec3::new(0.0, up, 0.0), 0.01 * up), "seam {seam}: {f:?} against {up}");
        let mf = est.seam_measured_force[seam];
        assert!(close(f, mf, 0.01 * up), "seam {seam}: free body {f:?}, beams {mf:?}");
    }
}

#[test]
fn estimated_ratio_tracks_the_measured_one() {
    let spec = spec();
    let mut w = cut_truss(&spec);
    w.est.on = true;
    run(&mut w, &spec, 900, 600);
    let est = &w.est;
    let mut worst_rel: Real = 0.0;
    for seam in 0..est.seam_ratio.len() {
        let (e, m) = (est.seam_ratio[seam], est.seam_measured[seam]);
        let rel = (e - m).abs() / m;
        worst_rel = worst_rel.max(rel);
        println!("seam {seam:2}: estimate {e:.3e}, measured {m:.3e}, {:+.1}%", 100.0 * (e - m) / m);
    }
    println!("worst relative error {:.1}%", 100.0 * worst_rel);
    assert!(worst_rel < 0.15, "worst {worst_rel}");
}

#[test]
fn a_beam_that_skips_a_seam_reads_high() {
    let spec = spec();
    let mut w = cut_truss(&spec);
    // A stiff strut from frame 2 straight to frame 5, skipping two seams.
    for j in [0u32, 1] {
        let (a, b) = (4 * 2 + j, 4 * 5 + j);
        let rest = (w.pos[b as usize] - w.pos[a as usize]).length();
        w.add_beam(a, b, BeamSpec { rest, compliance: rest / 3.6e6, damping: 50.0, yield_strain: 1.0, break_strain: 2.0 });
    }
    w.est.on = true;
    run(&mut w, &spec, 900, 600);
    let est = &w.est;
    // Seams 1..=3 lie between frames 2 and 5 (seam s joins frame s+1 to s+2).
    for seam in 1..=3 {
        let (e, m) = (est.seam_ratio[seam], est.seam_measured[seam]);
        println!("bypassed seam {seam}: estimate {e:.3e}, measured {m:.3e}");
        assert!(e > m, "seam {seam}: {e} not above {m}");
    }
}

#[test]
fn switching_it_on_changes_no_hash() {
    let spec = spec();
    let mut a = cut_truss(&spec);
    let mut b = cut_truss(&spec);
    b.est.on = true;
    for s in 0..300 {
        run(&mut a, &spec, 1, 0);
        run(&mut b, &spec, 1, 0);
        if s % 50 == 0 {
            assert_eq!(state_hash(&a), state_hash(&b));
        }
    }
    assert_eq!(state_hash(&a), state_hash(&b));
    assert_eq!(physics_hash(&a), physics_hash(&b));
    assert!(!b.est.seam_ratio.is_empty() && a.est.seam_ratio.is_empty());
}
