//! Phase 1.6: a resting structure under gravity never gains energy.
//!
//! Scene: a flat pinned truss (`support/flat_truss.rs`) of 1 kg nodes with light, physical beam
//! damping. Energy is `total_energy`: kinetic plus gravitational plus elastic
//! (`0.5 (length - rest)^2 / compliance` per beam), the same quantity the trace dump writes.
//!
//! What is asserted, and what is not. The structure is released from its unloaded shape, so it
//! first rings down. During that transient the measured energy is NOT monotone: at the rule's
//! substep count the solver is softer than the nominal beams (ER 001, phase 1.4), so the nominal
//! energy formula reads above the starting value for a few steps. Measured on the ten-minute
//! scene (24.5 J at the start): at most 0.27 J above (1.1%) at the rule's 24 substeps, 0.12 J at
//! 32, 0.02 J at 64, and below the start at 256; at a forced 8 substeps, 10.6 J. That is a
//! discretisation effect of the one-pass solver, not a gain of the simulated system, and it
//! shrinks with the substep count. So the assertion is made after the structure has settled:
//! energy must never exceed its value at the end of settling by more than a rounding tolerance.
//!
//! Tolerance, derived from rounding. The energy is a sum of about 3 terms per node and 1 per
//! beam, each evaluated to a relative error of one machine epsilon, so the sum is uncertain by
//! about `EPS * (sum of the absolute terms)`. On top of that the state itself jitters by one ulp
//! per coordinate from step to step, which moves each term by its own sensitivity: a node by
//! `m g ulp(y)`, a beam by `force * ulp(length)` (a force of up to a few hundred newtons on these
//! beams, against 1 N of weight). The tolerance is `2^10 * EPS * (sum of the absolute terms)`,
//! about 3e-12 to 6e-12 J on these scenes. Measured rises after settling: 8e-14 J in both scenes.

#[path = "support/flat_truss.rs"]
#[allow(dead_code)]
mod flat_truss;

use beam_core::real::Real;
use beam_core::sim::{gravity_at, state_hash, step, total_energy};
use beam_core::world::{World, NODE_FROZEN};
use flat_truss::add_flat_truss;

fn scene(bays: u32, damping: Real) -> World {
    let mut w = World::new();
    add_flat_truss(&mut w, 0.0, 0.0, bays, 1e-6, damping);
    w
}

/// Sum of the absolute values of the terms of `total_energy`: the scale rounding acts on.
fn energy_scale(w: &World) -> Real {
    let mut s = 0.0;
    for i in 0..w.node_count() {
        let u = i as usize;
        if w.inv_mass[u] == 0.0 {
            continue;
        }
        let m = 1.0 / w.inv_mass[u];
        s += (0.5 * m * w.vel[u].length_squared()).abs() + (m * gravity_at(w.pos[u]).y * w.pos[u].y).abs();
    }
    for b in 0..w.beam_count() {
        let u = b as usize;
        let c = (w.pos[w.beam_b[u] as usize] - w.pos[w.beam_a[u] as usize]).length() - w.rest[u];
        s += 0.5 * c * c / w.compliance[u];
    }
    s
}

struct Report {
    settled: Real,
    max_rise: Real,
    max_rise_step: u64,
    final_drift: Real,
    tolerance: Real,
    hash: u64,
}

/// Runs `steps` steps; `settle` is the step at whose end the energy is taken as the settled
/// value. Checks the quiet-run conditions every step and returns the measurements.
fn run(bays: u32, damping: Real, steps: u64, settle: u64) -> Report {
    let mut w = scene(bays, damping);
    let (mut settled, mut tolerance) = (0.0, 0.0);
    let (mut max_rise, mut max_rise_step) = (Real::MIN, 0);
    let mut last = 0.0;
    for s in 1..=steps {
        step(&mut w);
        assert!(w.events.is_empty(), "step {s}: events {:?}", w.events);
        assert_eq!(w.clamped_nodes, 0, "step {s}: a node was clamped");
        let e = total_energy(&w);
        assert!(e.is_finite(), "step {s}: energy {e}");
        if s == settle {
            settled = e;
            tolerance = 1024.0 * Real::EPSILON * energy_scale(&w);
        }
        if s > settle {
            if e - settled > max_rise {
                max_rise = e - settled;
                max_rise_step = s;
            }
            last = e;
        }
        if s % 3000 == 0 || s == steps {
            println!("step {s:6}: energy {e:.15e}");
        }
    }
    assert!((0..w.node_count()).all(|i| w.node_flags[i as usize] & NODE_FROZEN == 0), "a node froze");
    let r = Report { settled, max_rise, max_rise_step, final_drift: last - settled, tolerance, hash: state_hash(&w) };
    println!(
        "settled {:.15e} J (step {settle}), max rise {:.3e} J at step {}, final drift {:.3e} J, tolerance {:.3e} J",
        r.settled, r.max_rise, r.max_rise_step, r.final_drift, r.tolerance
    );
    assert!(r.max_rise <= r.tolerance, "energy rose by {:.3e} J at step {}, tolerance {:.3e}", r.max_rise, r.max_rise_step, r.tolerance);
    r
}

/// Fast version for every `cargo test`: 12 beams, 600 steps (10 s), settled by step 450.
#[test]
fn resting_truss_gains_no_energy_600_steps() {
    let a = run(3, 200.0, 600, 450);
    let b = run(3, 200.0, 600, 450);
    assert_eq!(a.hash, b.hash, "two runs gave different hashes");
}

/// Ten minutes of simulated time: 20 beams, 36,000 steps at 60 Hz, settled by two minutes.
/// Run with `cargo test --release -- --ignored`; about 2 s in release.
#[test]
#[ignore = "slow in debug; run with `cargo test --release -- --ignored`"]
fn resting_truss_gains_no_energy_in_ten_minutes() {
    let a = run(5, 20.0, 36_000, 7_200);
    let b = run(5, 20.0, 36_000, 7_200);
    assert_eq!(a.hash, b.hash, "two runs gave different hashes");
}
