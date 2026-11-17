//! The fixed 60 Hz step and the state hash. The solve itself is in [`crate::solver`].
//! Islands and nodes visited in ascending index order (determinism list).

use crate::event::Event;
use crate::island::{build_islands, copy_back};
use crate::real::{Real, Vec3};
use crate::solver::{solve_island, substep_count_with, SubstepRule};
use crate::world::{World, NODE_FROZEN};

/// Standard gravity as a per-node acceleration (a function so it can vary later).
pub fn gravity_at(_pos: Vec3) -> Vec3 {
    Vec3::new(0.0, -9.81, 0.0)
}

/// Advances the world by one fixed step in deterministic mode: each island gets its
/// rule-based substep count ([`substep_count`]). Returns the largest count any island used.
pub fn step(world: &mut World) -> u32 {
    step_inner(world, Choice::Rule(SubstepRule::DEFAULT)).max_substeps
}

/// What one step did, for the benchmark and debug overlays. Read-only bookkeeping.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct StepStats {
    /// Islands solved (frozen islands are skipped and not counted).
    pub islands: u32,
    /// Largest substep count any island used.
    pub max_substeps: u32,
    /// Sum over islands of substeps times beams: the number of beam solves done.
    pub beam_solves: u64,
}

/// Same as [`step`] with an explicit substep rule, and a report. `SubstepRule::DEFAULT` is
/// bit-identical to [`step`]; other rules are for pricing and comparing (ER 001, phase 1.7).
pub fn step_with_rule(world: &mut World, rule: SubstepRule) -> StepStats {
    step_inner(world, Choice::Rule(rule))
}

enum Choice {
    Rule(SubstepRule),
    Fixed(u32),
}

/// Same as [`step`], but every island uses exactly `substeps`. For demos and tests that
/// compare substep counts; not the deterministic default.
pub fn step_with_substeps(world: &mut World, substeps: u32) -> u32 {
    step_inner(world, Choice::Fixed(substeps.max(1))).max_substeps
}

fn step_inner(world: &mut World, choice: Choice) -> StepStats {
    world.events.clear();
    world.clamped_nodes = 0;
    let done = world.step_count + 1;
    let mut islands = build_islands(world);
    let mut stats = StepStats::default();
    for isl in islands.iter_mut() {
        // An island holding a frozen node is not solved and reports nothing; its nodes keep
        // their position and zero velocity. Its buffer is dropped without a copy back.
        if isl.node_world.iter().any(|&i| world.node_flags[i as usize] & NODE_FROZEN != 0) {
            isl.skipped = true;
            continue;
        }
        let n = match choice {
            Choice::Rule(rule) => substep_count_with(isl, rule),
            Choice::Fixed(n) => n,
        };
        let out = solve_island(isl, n, world.max_speed);
        world.clamped_nodes += out.clamped;
        if out.frozen {
            for (p, &i) in isl.node_world.iter().enumerate() {
                if isl.inv_mass[p] != 0.0 {
                    world.node_flags[i as usize] |= NODE_FROZEN;
                }
            }
        }
        world.events.extend(out.events.into_iter().map(|e| Event { step: done, ..e }));
        stats.islands += 1;
        stats.max_substeps = stats.max_substeps.max(n);
        stats.beam_solves += n as u64 * isl.beam_a.len() as u64;
    }
    for isl in islands.iter().filter(|isl| !isl.skipped) {
        copy_back(world, isl);
    }
    for f in world.force.iter_mut() {
        *f = Vec3::ZERO;
    }
    world.step_count += 1;
    stats
}

/// Kinetic plus gravitational plus elastic energy of the live world, in joules: for each
/// movable node `0.5 m v^2 + m g y` (zero of potential at `y = 0`), for each live beam with
/// nonzero compliance `0.5 (length - rest)^2 / compliance`. Pinned nodes carry none. Read-only
/// (debug traces and the stability tests); not part of the solve.
pub fn total_energy(w: &World) -> Real {
    let mut e = 0.0;
    for i in 0..w.node_count() {
        let u = i as usize;
        if !w.node_alive(i) || w.inv_mass[u] == 0.0 {
            continue;
        }
        let m = 1.0 / w.inv_mass[u];
        e += 0.5 * m * w.vel[u].length_squared() - m * gravity_at(w.pos[u]).y * w.pos[u].y;
    }
    for b in 0..w.beam_count() {
        let u = b as usize;
        if !w.beam_alive(b) || w.compliance[u] == 0.0 {
            continue;
        }
        let c = (w.pos[w.beam_b[u] as usize] - w.pos[w.beam_a[u] as usize]).length() - w.rest[u];
        e += 0.5 * c * c / w.compliance[u];
    }
    e
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
/// The terrain (size, origin, cell, friction and every height) follows, if there is one.
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
    // The ground, only when there is one: a world with no terrain hashes as it did before
    // terrain existed (the golden recording and the benchmark hashes depend on that). Every
    // height goes in each time; the hash is taken at checkpoints, not per step, so there is no
    // cached checksum to get out of step.
    if let Some(t) = &world.terrain {
        mix(&mut h, t.nx as u64);
        mix(&mut h, t.nz as u64);
        mix_real(&mut h, t.x0);
        mix_real(&mut h, t.z0);
        mix_real(&mut h, t.cell);
        mix_real(&mut h, t.friction);
        for &height in &t.heights {
            mix_real(&mut h, height);
        }
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
