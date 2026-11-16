//! The packed island buffer (layout rule 5): live nodes and live beams copied in ascending
//! index order, beam endpoints rewritten as positions in the packed arrays. The solver
//! ([`crate::solver`]) is a pure function of this buffer. Milestone 1 has one island holding everything.

use crate::real::{Real, Vec3};
use crate::world::World;

#[derive(Default, Debug, PartialEq)]
pub struct Island {
    // Per packed node, ascending world index.
    pub node_world: Vec<u32>,
    pub pos: Vec<Vec3>,
    pub vel: Vec<Vec3>,
    pub inv_mass: Vec<Real>,
    pub force: Vec<Vec3>,
    // Per packed beam, ascending world index. Endpoints are packed node positions.
    pub beam_world: Vec<u32>,
    pub beam_a: Vec<u32>,
    pub beam_b: Vec<u32>,
    pub rest: Vec<Real>,
    pub rest0: Vec<Real>,
    pub compliance: Vec<Real>,
    pub damping: Vec<Real>,
    pub yield_strain: Vec<Real>,
    pub break_strain: Vec<Real>,
    pub beam_flags: Vec<u32>,
    pub last_lambda: Vec<Real>,
}

/// The island split. For now it returns one island with every live node and beam.
pub fn build_islands(world: &World) -> Vec<Island> {
    let mut island = Island::default();
    let mut packed = vec![u32::MAX; world.node_count() as usize];
    for i in 0..world.node_count() {
        if !world.node_alive(i) {
            continue;
        }
        let u = i as usize;
        packed[u] = island.node_world.len() as u32;
        island.node_world.push(i);
        island.pos.push(world.pos[u]);
        island.vel.push(world.vel[u]);
        island.inv_mass.push(world.inv_mass[u]);
        island.force.push(world.force[u]);
    }
    for b in 0..world.beam_count() {
        if !world.beam_alive(b) {
            continue;
        }
        let u = b as usize;
        island.beam_world.push(b);
        island.beam_a.push(packed[world.beam_a[u] as usize]);
        island.beam_b.push(packed[world.beam_b[u] as usize]);
        island.rest.push(world.rest[u]);
        island.rest0.push(world.rest0[u]);
        island.compliance.push(world.compliance[u]);
        island.damping.push(world.damping[u]);
        island.yield_strain.push(world.yield_strain[u]);
        island.break_strain.push(world.break_strain[u]);
        island.beam_flags.push(world.beam_flags[u]);
        island.last_lambda.push(world.last_lambda[u]);
    }
    vec![island]
}

/// Copies solved state back to the world arrays.
pub fn copy_back(world: &mut World, island: &Island) {
    for (p, &w) in island.node_world.iter().enumerate() {
        world.pos[w as usize] = island.pos[p];
        world.vel[w as usize] = island.vel[p];
    }
    for (p, &w) in island.beam_world.iter().enumerate() {
        world.rest[w as usize] = island.rest[p];
        world.beam_flags[w as usize] = island.beam_flags[p];
        world.last_lambda[w as usize] = island.last_lambda[p];
    }
}
