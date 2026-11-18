//! The packed island buffer (layout rule 5): live nodes and live beams copied in ascending
//! index order, beam endpoints rewritten as positions in the packed arrays. The solver
//! ([`crate::solver`]) is a pure function of this buffer. Islands are connected components of movable nodes ([`build_islands`]).

use crate::body::NO_BODY;
use crate::real::{Real, Vec3};
use crate::world::{World, NODE_ALIVE};

#[derive(Default, Debug, PartialEq)]
pub struct Island {
    /// Lowest movable node's world index (`u32::MAX` for the island of pinned-only beams).
    pub root: u32,
    /// Set by the step when the island holds a frozen node and was not solved (not copied back).
    pub skipped: bool,
    // Per packed node, ascending world index.
    pub node_world: Vec<u32>,
    pub pos: Vec<Vec3>,
    pub vel: Vec<Vec3>,
    pub inv_mass: Vec<Real>,
    pub force: Vec<Vec3>,
    /// Contact sphere radius (ground contact, plan 003).
    pub radius: Vec<Real>,
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
    // Ground contacts for this step ([`crate::contact::find_contacts`]), ascending packed node
    // order. Each holds the plane `normal · p = offset` under its node for the whole step.
    pub contact_node: Vec<u32>,
    pub contact_normal: Vec<Vec3>,
    pub contact_offset: Vec<Real>,
    // Beam-midpoint ground contacts (phase 3.3): packed beam index in ascending order, and the
    // plane under the beam's midpoint at the step's start.
    pub beam_contact: Vec<u32>,
    pub beam_contact_normal: Vec<Vec3>,
    pub beam_contact_offset: Vec<Real>,
    /// The ground's friction coefficient `μ` for this step's contacts.
    pub friction: Real,
}

/// The island split: connected components of movable nodes.
///
/// - Two movable nodes are in one island when a live beam joins them, directly or through other
///   movable nodes. A pinned node (inverse mass zero) never joins islands: it is copied into
///   every island that has a beam to it and is never moved.
/// - Islands are ordered by their lowest movable node index (`root`). Inside an island, nodes
///   and beams are in ascending world index order (pinned nodes among them).
/// - A movable node with no beams is an island of its own.
/// - Live beams whose two ends are both pinned belong to no component. If there are any they
///   form one extra island after all the others (`root` is `u32::MAX`, no movable node).
///
/// Union-find with the smaller index as root, so the root is the lowest movable index of the
/// component and no hashing or iteration order enters. Rebuilt every step.
pub fn build_islands(world: &World) -> Vec<Island> {
    let n = world.node_count() as usize;
    // A node riding a rigid body (plan 005) is not movable: the body moves it, once per step.
    let movable = |i: usize| world.node_flags[i] & NODE_ALIVE != 0 && world.inv_mass[i] != 0.0 && world.node_body[i] == NO_BODY;
    let riding = |i: usize| world.node_body[i] != NO_BODY;
    let mut parent: Vec<u32> = (0..n as u32).collect();
    fn find(parent: &mut [u32], mut x: u32) -> u32 {
        while parent[x as usize] != x {
            let up = parent[parent[x as usize] as usize];
            parent[x as usize] = up;
            x = up;
        }
        x
    }
    for b in 0..world.beam_count() {
        if !world.beam_alive(b) {
            continue;
        }
        let (a, c) = (world.beam_a[b as usize] as usize, world.beam_b[b as usize] as usize);
        if movable(a) && movable(c) {
            let (ra, rc) = (find(&mut parent, a as u32), find(&mut parent, c as u32));
            if ra != rc {
                parent[ra.max(rc) as usize] = ra.min(rc);
            }
        }
    }
    // Island ids in ascending root order; roots are visited in ascending index.
    let mut island_of_root = vec![u32::MAX; n];
    let mut count = 0u32;
    for i in 0..n {
        if movable(i) && find(&mut parent, i as u32) == i as u32 {
            island_of_root[i] = count;
            count += 1;
        }
    }
    let mut nodes: Vec<Vec<u32>> = vec![Vec::new(); count as usize];
    let mut beams: Vec<Vec<u32>> = vec![Vec::new(); count as usize];
    let mut fixed_beams: Vec<u32> = Vec::new();
    let mut fixed_nodes: Vec<u32> = Vec::new();
    for i in 0..n {
        if movable(i) {
            let r = find(&mut parent, i as u32);
            nodes[island_of_root[r as usize] as usize].push(i as u32);
        }
    }
    for b in 0..world.beam_count() {
        if !world.beam_alive(b) {
            continue;
        }
        let (a, c) = (world.beam_a[b as usize], world.beam_b[b as usize]);
        let home = if movable(a as usize) {
            Some(a)
        } else if movable(c as usize) {
            Some(c)
        } else {
            None
        };
        match home {
            Some(m) => {
                let r = find(&mut parent, m);
                let id = island_of_root[r as usize] as usize;
                beams[id].push(b);
                for e in [a, c] {
                    if !movable(e as usize) {
                        nodes[id].push(e);
                    }
                }
            }
            // A beam inside a body (or from a body to a pin) is not solved: the body is rigid.
            None if riding(a as usize) || riding(c as usize) => {}
            None => {
                fixed_beams.push(b);
                fixed_nodes.push(a);
                fixed_nodes.push(c);
            }
        }
    }
    let mut packed = vec![u32::MAX; n];
    let mut islands = Vec::with_capacity(count as usize + 1);
    for (id, (ns, bs)) in nodes.iter_mut().zip(beams.iter()).enumerate() {
        ns.sort_unstable();
        ns.dedup();
        let root = *ns.iter().find(|&&i| movable(i as usize)).expect("island has a movable node");
        debug_assert_eq!(island_of_root[root as usize] as usize, id);
        islands.push(pack(world, ns, bs, &mut packed, root));
    }
    if !fixed_beams.is_empty() {
        fixed_nodes.sort_unstable();
        fixed_nodes.dedup();
        islands.push(pack(world, &fixed_nodes, &fixed_beams, &mut packed, u32::MAX));
    }
    islands
}

/// Copies the listed nodes and beams (both ascending) into one packed island.
fn pack(world: &World, nodes: &[u32], beams: &[u32], packed: &mut [u32], root: u32) -> Island {
    let mut island = Island { root, ..Island::default() };
    for &i in nodes {
        let u = i as usize;
        packed[u] = island.node_world.len() as u32;
        island.node_world.push(i);
        island.pos.push(world.pos[u]);
        island.vel.push(world.vel[u]);
        // A rider is a fixed end to the island (phase 5.2: beams do not push bodies yet).
        island.inv_mass.push(if world.node_body[u] != NO_BODY { 0.0 } else { world.inv_mass[u] });
        island.force.push(world.force[u]);
        island.radius.push(world.radius[u]);
    }
    for &b in beams {
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
    island
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
