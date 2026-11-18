//! Node-beam physics core. Pure simulation: no rendering, no browser dependencies.
//! Built to WebAssembly for the game and natively for tests and benchmarks.
//! Storage rules: docs/research/core-data-layout.md.

pub mod bench;
pub mod command;
pub mod contact;
pub mod event;
pub mod failure;
pub mod ffi;
pub mod island;
pub mod maths;
pub mod real;
pub mod sim;
pub mod solver;
pub mod terrain;
pub mod thrust;
pub mod world;

use real::Real;

/// Fixed simulation rate in hertz. Rendering interpolates between steps.
pub const STEP_HZ: u32 = 60;

/// Seconds advanced by one fixed step.
pub const STEP_SECONDS: Real = 1.0 / STEP_HZ as Real;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::island::{build_islands, copy_back};
    use crate::real::Vec3;
    use crate::world::{BeamSpec, World};

    #[test]
    fn step_is_one_sixtieth_of_a_second() {
        assert!((STEP_SECONDS * STEP_HZ as Real - 1.0).abs() < 1e-12);
    }

    fn spec() -> BeamSpec {
        BeamSpec { rest: 1.0, compliance: 1e-6, damping: 0.1, yield_strain: 0.01, break_strain: 0.05 }
    }

    /// Dead slots are reused lowest-first with a new generation, and the packed island
    /// skips them, in ascending order, with endpoints rewritten to packed positions.
    #[test]
    fn stable_indices_and_packed_island() {
        let mut w = World::new();
        let n0 = w.add_node(Vec3::new(0.0, 0.0, 0.0), 0.0, 0.1);
        let n1 = w.add_node(Vec3::new(1.0, 0.0, 0.0), 1.0, 0.1);
        let n2 = w.add_node(Vec3::new(2.0, 0.0, 0.0), 1.0, 0.1);
        let b0 = w.add_beam(n0.index, n1.index, spec());
        let b1 = w.add_beam(n1.index, n2.index, spec());

        // Deleting the middle node kills both beams but moves nothing.
        w.delete_node(n1.index);
        assert!(!w.node_handle_valid(n1));
        assert!(!w.beam_handle_valid(b0) && !w.beam_handle_valid(b1));
        assert_eq!(w.node_count(), 3);

        // The freed slot is reused with a new generation; the old handle stays stale.
        let n1b = w.add_node(Vec3::new(5.0, 0.0, 0.0), 1.0, 0.1);
        assert_eq!(n1b.index, n1.index);
        assert_ne!(n1b.generation, n1.generation);
        assert!(!w.node_handle_valid(n1) && w.node_handle_valid(n1b));

        // Delete node 0; the island holds nodes 1 and 2 only, in ascending order.
        w.add_beam(n1b.index, n2.index, spec());
        w.delete_node(n0.index);
        w.add_force(n2.index, Vec3::new(0.0, -3.0, 0.0));
        let mut islands = build_islands(&w);
        assert_eq!(islands.len(), 1);
        let isl = &mut islands[0];
        assert_eq!(isl.node_world, vec![1, 2]);
        assert_eq!(isl.beam_a.len(), 1);
        assert_eq!((isl.beam_a[0], isl.beam_b[0]), (0, 1));
        assert_eq!(isl.force[1], Vec3::new(0.0, -3.0, 0.0));

        // Solved state is copied back to the world.
        isl.pos[1] = Vec3::new(9.0, 9.0, 9.0);
        copy_back(&mut w, &islands[0]);
        assert_eq!(w.pos[n2.index as usize], Vec3::new(9.0, 9.0, 9.0));
    }
}
