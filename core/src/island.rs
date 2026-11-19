//! The packed island buffer (layout rule 5): live nodes and live beams copied in ascending
//! index order, beam endpoints rewritten as positions in the packed arrays. The solver
//! ([`crate::solver`]) is a pure function of this buffer. Islands are connected components of
//! movable nodes and rigid bodies ([`build_islands`]). A body is packed with the island of the
//! live nodes it touches, with its riders and its own pose (phase 5.3).

use crate::body::{SpinForm, NO_BODY};
use crate::joint::JointSpec;
use crate::maths::{Quat, Sym3};
use crate::real::{Real, Vec3};
use crate::world::{World, NODE_ALIVE};

#[derive(Default, Debug, PartialEq)]
pub struct Island {
    /// Lowest movable or riding node's world index (`u32::MAX` for the island of pinned-only
    /// beams).
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
    /// Packed body the node rides, or [`NO_BODY`]. A rider's `inv_mass` here is zero: the
    /// solve moves it only through its body.
    pub node_body: Vec<u32>,
    /// A rider's offset from its body's centre, body frame (zero for a loose node).
    pub offset: Vec<Vec3>,
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
    // Per packed body (phase 5.3), ascending slot. Velocities world frame, inertia body frame.
    pub body_world: Vec<u32>,
    pub body_centre: Vec<Vec3>,
    pub body_rot: Vec<Quat>,
    pub body_vel: Vec<Vec3>,
    pub body_omega: Vec<Vec3>,
    pub body_mass: Vec<Real>,
    /// `1 / M`.
    pub body_inv_mass: Vec<Real>,
    pub body_inertia: Vec<Sym3>,
    pub body_inv_inertia: Vec<Sym3>,
    /// The step's external force and torque (about the centre at the step's start), summed
    /// over the riders in ascending node order.
    pub body_force: Vec<Vec3>,
    pub body_torque: Vec<Vec3>,
    /// The pose the last prediction left, which the body's velocity is derived against.
    pub body_pred_centre: Vec<Vec3>,
    pub body_pred_rot: Vec<Quat>,
    /// The world's spin integrator and angular speed limit, for the bodies' prediction.
    pub spin_form: SpinForm,
    pub max_angular_speed: Real,
    // Per packed joint (phase 5.5), ascending world index. Ends are packed node positions in the
    // order `[a0, a1, ra, b0, b1, rb]`. The solve writes the angle at the step's end, the last
    // substep's drive torque, the step's peak drive torque and whether the angle moved.
    pub joint_world: Vec<u32>,
    pub joint_ends: Vec<[u32; 6]>,
    pub joint_spec: Vec<JointSpec>,
    pub joint_angle: Vec<Real>,
    pub joint_torque: Vec<Real>,
    pub joint_peak: Vec<Real>,
    pub joint_moved: Vec<bool>,
    /// Per packed node, the step's summed contact impulse (normal and friction, without the part
    /// that only drained old overlap), in N·s. Sized by the caller to ask for it (the stress
    /// estimate, phase 5.7); left empty, the solver records nothing. Never read by the solve.
    pub contact_impulse: Vec<Vec3>,
}

impl Island {
    /// Packed node `i` rides a body.
    #[inline(always)]
    pub fn rides(&self, i: usize) -> bool {
        self.node_body.get(i).is_some_and(|&b| b != NO_BODY)
    }
}

/// The island split: connected components of movable nodes and rigid bodies.
///
/// - Two movable nodes are in one island when a live beam joins them, directly or through other
///   movable nodes. A pinned node (inverse mass zero) never joins islands: it is copied into
///   every island that has a beam to it and is never moved.
/// - A node riding a rigid body (phase 5.3) joins as a movable node does, and every rider of
///   one body is in one island, so a body is in the island of the live nodes its riders have
///   beams to. A beam with both ends on one body is not packed at all: the body is rigid.
/// - Islands are ordered by their lowest movable or riding node index (`root`). Inside an
///   island, nodes and beams are in ascending world index order (pinned nodes among them), and
///   bodies in ascending slot order.
/// - A movable node with no beams is an island of its own; so is a body with no beam out.
/// - Live beams whose two ends are both pinned belong to no component. If there are any they
///   form one extra island after all the others (`root` is `u32::MAX`, no movable node).
///
/// - A live joint (phase 5.5) joins all its movable nodes, as a beam does; its pinned nodes are
///   copied in. A joint with no movable node, or with all six nodes on one body, is not packed.
///
/// Union-find with the smaller index as root, so the root is the lowest joining index of the
/// component and no hashing or iteration order enters. Rebuilt every step.
pub fn build_islands(world: &World) -> Vec<Island> {
    let n = world.node_count() as usize;
    // A node riding a rigid body joins as a movable node does; the solve moves it through its
    // body. Two ends on one body: a beam inside the body, never packed.
    let movable = |i: usize| {
        world.node_flags[i] & NODE_ALIVE != 0
            && (world.node_body[i] != NO_BODY || world.inv_mass[i] != 0.0)
    };
    let inside = |a: usize, c: usize| {
        world.node_body[a] != NO_BODY && world.node_body[a] == world.node_body[c]
    };
    let mut parent: Vec<u32> = (0..n as u32).collect();
    fn find(parent: &mut [u32], mut x: u32) -> u32 {
        while parent[x as usize] != x {
            let up = parent[parent[x as usize] as usize];
            parent[x as usize] = up;
            x = up;
        }
        x
    }
    fn union(parent: &mut [u32], a: u32, c: u32) {
        let (ra, rc) = (find(parent, a), find(parent, c));
        if ra != rc {
            parent[ra.max(rc) as usize] = ra.min(rc);
        }
    }
    // Every rider of a body with the body's lowest rider, in node order.
    let mut first_rider = vec![u32::MAX; world.body_slot_count() as usize];
    for i in 0..n {
        let s = world.node_body[i];
        if s != NO_BODY && movable(i) {
            if first_rider[s as usize] == u32::MAX {
                first_rider[s as usize] = i as u32;
            } else {
                union(&mut parent, first_rider[s as usize], i as u32);
            }
        }
    }
    for b in 0..world.beam_count() {
        if !world.beam_alive(b) {
            continue;
        }
        let (a, c) = (
            world.beam_a[b as usize] as usize,
            world.beam_b[b as usize] as usize,
        );
        if movable(a) && movable(c) && !inside(a, c) {
            union(&mut parent, a as u32, c as u32);
        }
    }
    // Joints, ascending: which take part, and their movable nodes in one island.
    let joint_on = |j: usize| {
        let ns = world.joint_nodes[j];
        world.joint_nodes_live(j)
            && ns.iter().any(|&i| movable(i as usize))
            && !ns.iter().all(|&i| inside(ns[0] as usize, i as usize))
    };
    for j in 0..world.joint_count() as usize {
        if joint_on(j) {
            let ns = world.joint_nodes[j];
            let first = *ns.iter().find(|&&i| movable(i as usize)).unwrap();
            for &i in &ns {
                if movable(i as usize) {
                    union(&mut parent, first, i);
                }
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
        if inside(a as usize, c as usize) {
            continue; // the body is rigid: a beam inside it carries nothing
        }
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
            None => {
                fixed_beams.push(b);
                fixed_nodes.push(a);
                fixed_nodes.push(c);
            }
        }
    }
    let mut joints: Vec<Vec<u32>> = vec![Vec::new(); count as usize];
    for j in 0..world.joint_count() as usize {
        if joint_on(j) {
            let ns = world.joint_nodes[j];
            let first = *ns.iter().find(|&&i| movable(i as usize)).unwrap();
            let id = island_of_root[find(&mut parent, first) as usize] as usize;
            joints[id].push(j as u32);
            for &i in &ns {
                if !movable(i as usize) {
                    nodes[id].push(i);
                }
            }
        }
    }
    let mut packed = vec![u32::MAX; n];
    let mut islands = Vec::with_capacity(count as usize + 1);
    for (id, (ns, bs)) in nodes.iter_mut().zip(beams.iter()).enumerate() {
        ns.sort_unstable();
        ns.dedup();
        let root = *ns
            .iter()
            .find(|&&i| movable(i as usize))
            .expect("island has a movable node");
        debug_assert_eq!(island_of_root[root as usize] as usize, id);
        let mut isl = pack(world, ns, bs, &mut packed, root);
        pack_joints(world, &mut isl, &joints[id], &packed);
        islands.push(isl);
    }
    if !fixed_beams.is_empty() {
        fixed_nodes.sort_unstable();
        fixed_nodes.dedup();
        islands.push(pack(
            world,
            &fixed_nodes,
            &fixed_beams,
            &mut packed,
            u32::MAX,
        ));
    }
    islands
}

/// Copies the listed nodes and beams (both ascending), and the bodies the nodes ride
/// (ascending slot), into one packed island.
fn pack(world: &World, nodes: &[u32], beams: &[u32], packed: &mut [u32], root: u32) -> Island {
    let mut island = Island {
        root,
        spin_form: world.spin_form,
        max_angular_speed: world.max_angular_speed,
        ..Island::default()
    };
    let mut slots: Vec<u32> = nodes
        .iter()
        .map(|&i| world.node_body[i as usize])
        .filter(|&s| s != NO_BODY)
        .collect();
    slots.sort_unstable();
    slots.dedup();
    for &s in &slots {
        let u = s as usize;
        island.body_world.push(s);
        island.body_centre.push(world.body_centre[u]);
        island.body_rot.push(world.body_rot[u]);
        island.body_vel.push(world.body_vel[u]);
        island.body_omega.push(world.body_omega[u]);
        island.body_mass.push(world.body_mass[u]);
        island.body_inv_mass.push(1.0 / world.body_mass[u]);
        island.body_inertia.push(world.body_inertia[u]);
        island.body_inv_inertia.push(world.body_inv_inertia[u]);
        island.body_force.push(Vec3::ZERO);
        island.body_torque.push(Vec3::ZERO);
        island.body_pred_centre.push(world.body_prev_centre[u]);
        island.body_pred_rot.push(world.body_prev_rot[u]);
    }
    for &i in nodes {
        let u = i as usize;
        packed[u] = island.node_world.len() as u32;
        island.node_world.push(i);
        island.pos.push(world.pos[u]);
        island.vel.push(world.vel[u]);
        island.force.push(world.force[u]);
        island.radius.push(world.radius[u]);
        let s = world.node_body[u];
        if s == NO_BODY {
            island.inv_mass.push(world.inv_mass[u]);
            island.node_body.push(NO_BODY);
            island.offset.push(Vec3::ZERO);
        } else {
            // A rider: its mass is in its body, which takes its force and the force's torque.
            let b = slots.binary_search(&s).expect("a rider's body is packed");
            let f = world.force[u];
            island.inv_mass.push(0.0);
            island.node_body.push(b as u32);
            island.offset.push(world.node_offset[u]);
            island.body_force[b] = island.body_force[b] + f;
            island.body_torque[b] =
                island.body_torque[b] + (world.pos[u] - island.body_centre[b]).cross(f);
        }
    }
    let boost = world.sleep.on && world.sleep.boosting();
    for &b in beams {
        let u = b as usize;
        island.beam_world.push(b);
        island.beam_a.push(packed[world.beam_a[u] as usize]);
        island.beam_b.push(packed[world.beam_b[u] as usize]);
        island.rest.push(world.rest[u]);
        island.rest0.push(world.rest0[u]);
        island.compliance.push(world.compliance[u]);
        // A just-woken segment's beams solve with extra damping (phase 5.8); untouched otherwise.
        island.damping.push(if boost {
            let (sa, sb) = (world.node_segment[world.beam_a[u] as usize], world.node_segment[world.beam_b[u] as usize]);
            world.damping[u] * world.sleep.damp_scale(sa, sb)
        } else {
            world.damping[u]
        });
        island.yield_strain.push(world.yield_strain[u]);
        island.break_strain.push(world.break_strain[u]);
        island.beam_flags.push(world.beam_flags[u]);
        island.last_lambda.push(world.last_lambda[u]);
    }
    island
}

/// Adds the listed joints (ascending) to a packed island, after its nodes.
fn pack_joints(world: &World, island: &mut Island, joints: &[u32], packed: &[u32]) {
    for &j in joints {
        let u = j as usize;
        island.joint_world.push(j);
        island
            .joint_ends
            .push(world.joint_nodes[u].map(|i| packed[i as usize]));
        island.joint_spec.push(world.joint_spec(u));
        island.joint_angle.push(world.joint_angle[u]);
        island.joint_torque.push(0.0);
        island.joint_peak.push(0.0);
        island.joint_moved.push(false);
    }
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
    for (p, &s) in island.body_world.iter().enumerate() {
        let s = s as usize;
        world.body_centre[s] = island.body_centre[p];
        world.body_rot[s] = island.body_rot[p];
        world.body_vel[s] = island.body_vel[p];
        world.body_omega[s] = island.body_omega[p];
        world.body_prev_centre[s] = island.body_pred_centre[p];
        world.body_prev_rot[s] = island.body_pred_rot[p];
    }
    for (p, &j) in island.joint_world.iter().enumerate() {
        let j = j as usize;
        world.joint_angle[j] = island.joint_angle[p];
        world.joint_torque[j] = island.joint_torque[p];
        world.joint_peak[j] = island.joint_peak[p];
        if island.joint_moved[p] {
            world.joint_moved(j);
        }
    }
}
