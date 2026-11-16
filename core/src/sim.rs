//! The fixed 60 Hz step and the state hash. The solve itself is in [`crate::solver`].
//! Islands and nodes visited in ascending index order (determinism list).

use crate::event::Event;
use crate::island::{build_islands, copy_back};
use crate::real::{Real, Vec3};
use crate::solver::{solve_island, substep_count};
use crate::world::World;

/// Standard gravity as a per-node acceleration (a function so it can vary later).
pub fn gravity_at(_pos: Vec3) -> Vec3 {
    Vec3::new(0.0, -9.81, 0.0)
}

/// Advances the world by one fixed step in deterministic mode: each island gets its
/// rule-based substep count ([`substep_count`]). Returns the largest count any island used.
pub fn step(world: &mut World) -> u32 {
    step_inner(world, None)
}

/// Same as [`step`], but every island uses exactly `substeps`. For demos and tests that
/// compare substep counts; not the deterministic default.
pub fn step_with_substeps(world: &mut World, substeps: u32) -> u32 {
    step_inner(world, Some(substeps.max(1)))
}

fn step_inner(world: &mut World, fixed: Option<u32>) -> u32 {
    world.events.clear();
    let done = world.step_count + 1;
    let mut islands = build_islands(world);
    let mut most = 0;
    for isl in islands.iter_mut() {
        let n = fixed.unwrap_or_else(|| substep_count(isl));
        let events = solve_island(isl, n);
        world.events.extend(events.into_iter().map(|e| Event { step: done, ..e }));
        most = most.max(n);
    }
    for isl in &islands {
        copy_back(world, isl);
    }
    for f in world.force.iter_mut() {
        *f = Vec3::ZERO;
    }
    world.step_count += 1;
    most
}

const FNV_OFFSET: u64 = 0xcbf2_9ce4_8422_2325;
const FNV_PRIME: u64 = 0x0000_0100_0000_01b3;

fn mix(h: &mut u64, v: u64) {
    for k in 0..8 {
        *h ^= (v >> (8 * k)) & 0xff;
        *h = h.wrapping_mul(FNV_PRIME);
    }
}

fn mix_real(h: &mut u64, r: Real) {
    mix(h, r.to_bits());
}

fn mix_vec(h: &mut u64, v: Vec3) {
    mix_real(h, v.x);
    mix_real(h, v.y);
    mix_real(h, v.z);
}

/// FNV-1a over the raw bits of positions, velocities, rest lengths and flags, in index
/// order, plus the step count. Dead slots are included so slot reuse is part of the state.
pub fn state_hash(world: &World) -> u64 {
    let mut h = FNV_OFFSET;
    mix(&mut h, world.step_count);
    mix(&mut h, world.node_count() as u64);
    for i in 0..world.pos.len() {
        mix_vec(&mut h, world.pos[i]);
        mix_vec(&mut h, world.vel[i]);
        mix(&mut h, world.node_flags[i] as u64);
    }
    mix(&mut h, world.beam_count() as u64);
    for i in 0..world.rest.len() {
        mix_real(&mut h, world.rest[i]);
        mix(&mut h, world.beam_flags[i] as u64);
    }
    h
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::world::BeamSpec;

    fn scene() -> World {
        let mut w = World::new();
        let a = w.add_node(Vec3::new(0.0, 10.0, 0.0), 0.0, 0.1);
        let b = w.add_node(Vec3::new(1.0, 10.0, 0.3), 1.0, 0.1);
        let c = w.add_node(Vec3::new(2.0, 11.0, -0.2), 0.5, 0.1);
        let s = BeamSpec { rest: 1.0, compliance: 1e-6, damping: 0.1, yield_strain: 0.01, break_strain: 0.05 };
        w.add_beam(a.index, b.index, s);
        w.add_beam(b.index, c.index, s);
        w
    }

    fn run(steps: u32, push: bool) -> (World, u64) {
        let mut w = scene();
        for s in 0..steps {
            if push && s == 5 {
                w.add_force(2, Vec3::new(1.0, 0.0, 0.0));
            }
            step(&mut w);
        }
        let h = state_hash(&w);
        (w, h)
    }

    #[test]
    fn same_inputs_same_hash() {
        let (_, a) = run(600, true);
        let (_, b) = run(600, true);
        assert_eq!(a, b);
    }

    #[test]
    fn different_inputs_different_hash() {
        let (_, a) = run(600, true);
        let (_, b) = run(600, false);
        assert_ne!(a, b);
    }

    #[test]
    fn gravity_pulls_free_nodes_and_pinned_stay() {
        let mut w = scene();
        let free = w.add_node(Vec3::new(5.0, 10.0, 0.0), 1.0, 0.1).index as usize;
        for s in 0..60 {
            if s == 5 {
                w.add_force(2, Vec3::new(1.0, 0.0, 0.0));
            }
            step(&mut w);
        }
        assert_eq!(w.pos[0], Vec3::new(0.0, 10.0, 0.0));
        // One second of free fall for a node on no beam: about 0.5 g t^2 = 4.9 m.
        let drop = 10.0 - w.pos[free].y;
        assert!((drop - 4.905).abs() < 0.1, "drop was {drop}");
        assert_eq!(w.step_count, 60);
        assert_eq!(w.force[2], Vec3::ZERO);
    }
}
