//! A small flat truss for the stability tests: `bays` square bays of 0.5 m in the x-y plane, the
//! left two nodes pinned, 1 kg free nodes, one diagonal per bay (so every bay is rigid and the
//! whole thing is a cantilever on its pins). Four beams per bay.

use beam_core::real::{Real, Vec3};
use beam_core::world::{BeamSpec, World};

pub const BAY: Real = 0.5;

/// Adds the truss with its left-bottom corner at `(x0, y0)`. Node order: bottom then top of each
/// frame, left to right, so the first two nodes are the pins. Returns the first node index.
pub fn add_flat_truss(w: &mut World, x0: Real, y0: Real, bays: u32, compliance: Real, damping: Real) -> u32 {
    let first = w.node_count();
    for i in 0..=bays {
        let x = x0 + i as Real * BAY;
        let inv = if i == 0 { 0.0 } else { 1.0 };
        w.add_node(Vec3::new(x, y0, 0.0), inv, 0.03);
        w.add_node(Vec3::new(x, y0 + BAY, 0.0), inv, 0.03);
    }
    let spec = |rest: Real| BeamSpec { rest, compliance, damping, yield_strain: 0.05, break_strain: 0.2 };
    let diag = (2.0 as Real).sqrt() * BAY;
    for i in 0..bays {
        let (b0, t0, b1, t1) = (first + 2 * i, first + 2 * i + 1, first + 2 * i + 2, first + 2 * i + 3);
        w.add_beam(b0, b1, spec(BAY));
        w.add_beam(t0, t1, spec(BAY));
        w.add_beam(b1, t1, spec(BAY));
        w.add_beam(b0, t1, spec(diag));
    }
    first
}

/// Number of nodes the truss adds.
pub fn flat_truss_nodes(bays: u32) -> u32 {
    2 * (bays + 1)
}
