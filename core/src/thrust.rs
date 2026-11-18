//! Thrusters (plan 004, phase 4.6): a push along the line from a tail node to a head node.
//!
//! A thruster is two node indices, a maximum thrust in newtons and a throttle from 0 to 1, kept in
//! flat arrays on the world (one per field, layout rule 3) and addressed by a stable `u32` index.
//! Each step, before the islands are built, [`apply_thrust`] adds `throttle * maximum` along the
//! unit vector from tail to head to the two nodes, half on each, through [`World::add_force`] (the
//! one force door, rule 7), using the positions the step starts with. So thrust is plain external
//! force: the solver neither knows nor cares where it came from.
//!
//! A thruster whose tail or head has been deleted (even if the slot was reused since: the node
//! generation is kept) gives nothing, and so does one whose two nodes are on top of each other.
//! There is no gimbal and no fuel flow; the mass stays what it is.

use crate::real::Real;
use crate::world::World;

/// Adds the force of every thruster with a throttle above zero, in ascending thruster order.
pub fn apply_thrust(world: &mut World) {
    for t in 0..world.thr_tail.len() {
        let throttle = world.thr_throttle[t];
        if throttle <= 0.0 {
            continue;
        }
        let (tail, head) = (world.thr_tail[t], world.thr_head[t]);
        if !world.thruster_nodes_live(t) {
            continue;
        }
        let d = world.pos[head as usize] - world.pos[tail as usize];
        let len = d.length();
        if !(len > 0.0) || !len.is_finite() {
            continue;
        }
        let half: Real = 0.5 * throttle * world.thr_max[t];
        let push = d * (half / len);
        world.add_force(tail, push);
        world.add_force(head, push);
    }
}
