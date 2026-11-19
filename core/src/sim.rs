//! The fixed 60 Hz step and the state hash. The solve itself is in [`crate::solver`].
//! Islands and nodes visited in ascending index order (determinism list).

use crate::body::NO_BODY;
use crate::contact::find_contacts;
use crate::event::Event;
use crate::island::{build_islands, copy_back};
use crate::real::{Real, Vec3};
use crate::solver::{solve_island, substep_count_with, SubstepRule};
use crate::thrust::apply_thrust;
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
    /// Ground contacts found this step, over all solved islands (each is solved every substep).
    pub contacts: u32,
    /// Beam-midpoint ground contacts found this step (phase 3.3), counted the same way.
    pub beam_contacts: u32,
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
    // Sleep and wake (phase 5.8): the wakes that cannot wait for the estimate, before anything
    // reads the step's start. No-op unless sleeping was ever switched on.
    world.sleep_before_solve(done);
    crate::estimate::begin_step(world);
    // Thrust goes in as ordinary external force, from the positions the step starts with.
    apply_thrust(world);
    crate::aero::apply_aero(world);
    crate::estimate::take_forces(world);
    if world.joint_count() > 0 {
        world.joints_begin_step();
    }
    let mut islands = build_islands(world);
    let mut stats = StepStats::default();
    for isl in islands.iter_mut() {
        // An island holding a frozen node is not solved and reports nothing; its nodes keep
        // their position and zero velocity. Its buffer is dropped without a copy back.
        if isl
            .node_world
            .iter()
            .any(|&i| world.node_flags[i as usize] & NODE_FROZEN != 0)
        {
            isl.skipped = true;
            continue;
        }
        if let Some(terrain) = &world.terrain {
            find_contacts(isl, terrain);
        }
        if world.est.on {
            // Asks the solver to read off each node's contact impulse (output only).
            isl.contact_impulse = vec![Vec3::ZERO; isl.node_world.len()];
        }
        let n = match choice {
            Choice::Rule(rule) => substep_count_with(isl, rule),
            Choice::Fixed(n) => n,
        };
        let out = solve_island(isl, n, world.max_speed);
        world.clamped_nodes += out.clamped;
        if out.frozen {
            for (p, &i) in isl.node_world.iter().enumerate() {
                if isl.inv_mass[p] != 0.0 || isl.rides(p) {
                    world.node_flags[i as usize] |= NODE_FROZEN;
                }
            }
        }
        world
            .events
            .extend(out.events.into_iter().map(|e| Event { step: done, ..e }));
        stats.islands += 1;
        stats.max_substeps = stats.max_substeps.max(n);
        stats.beam_solves += n as u64 * isl.beam_a.len() as u64;
        stats.contacts += isl.contact_node.len() as u32;
        stats.beam_contacts += isl.beam_contact.len() as u32;
    }
    for isl in islands.iter().filter(|isl| !isl.skipped) {
        copy_back(world, isl);
        for (p, &j) in isl.contact_impulse.iter().enumerate() {
            let i = isl.node_world[p] as usize;
            world.est.contact[i] = world.est.contact[i] + j;
        }
    }
    // Splitting into vessels (phase 5.10): before the estimate and sleeping read the step, so both
    // see the vessels it ended with. No-op without segments or without a cut.
    world.vessel_after_solve(done);
    crate::estimate::finish_step(world);
    world.sleep_after_step(done);
    for f in world.force.iter_mut() {
        *f = Vec3::ZERO;
    }
    for f in world.env_force.iter_mut() {
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
/// The terrain (size, origin, cell, friction and every height) follows, if there is one, then
/// the thrusters (ends, maximum and throttle), if there are any, and last the segments and seams
/// (plan 005), if any segment exists, and the body membership while a rigid body is alive. A
/// world that never ran a segment command hashes exactly as it did before segments existed;
/// [`physics_hash`] is the hash without the segment section. The bodies' own state (centre,
/// rotation, velocities, mass, inverse inertia) is in both, only while a body is alive.
pub fn state_hash(world: &World) -> u64 {
    let mut h = physics_hash_state(world);
    // Segments, only once one exists. They change no physics in phase 5.1, so this section is the
    // only difference from `physics_hash`. Masses and section numbers go in as bits; the tree and
    // every node's segment, in index order.
    if !world.seg_parent.is_empty() {
        mix(&mut h, world.seg_parent.len() as u64);
        for s in 0..world.seg_parent.len() {
            mix(&mut h, world.seg_flags[s] as u64);
            mix(&mut h, world.seg_parent[s] as u64);
            mix(&mut h, world.seg_seam[s] as u64);
            mix_real(&mut h, world.seg_mass[s]);
        }
        mix(&mut h, world.seam_a.len() as u64);
        for k in 0..world.seam_a.len() {
            mix(&mut h, world.seam_a[k] as u64);
            mix(&mut h, world.seam_b[k] as u64);
            mix_real(&mut h, world.seam_area[k]);
            mix_real(&mut h, world.seam_bending[k]);
        }
        for i in 0..world.node_segment.len() {
            mix(&mut h, world.node_segment[i] as u64);
        }
    }
    // Rigid bodies, only while one is alive: which segment and node belong to which body and each
    // rider's offset (the body state itself is in `physics_hash_state`).
    if world.body_count() > 0 {
        for s in 0..world.seg_body.len() {
            mix(&mut h, world.seg_body[s] as u64);
        }
        for i in 0..world.node_body.len() {
            mix(&mut h, world.node_body[i] as u64);
            if world.node_body[i] != NO_BODY {
                mix_vec(&mut h, world.node_offset[i]);
            }
        }
    }
    // Sleep state (phase 5.8), only once `CMD_SET_SLEEPING` has run: a world that never sent it
    // hashes as before. The switch checks are outputs and stay out.
    if world.sleep.ever {
        let sl = &world.sleep;
        mix(&mut h, sl.on as u64);
        mix(&mut h, sl.primed as u64);
        mix(&mut h, sl.asleep.len() as u64);
        for s in 0..sl.asleep.len() {
            mix(&mut h, sl.asleep[s] as u64);
            mix(&mut h, sl.live_steps[s] as u64);
            mix(&mut h, sl.quiet_steps[s] as u64);
            mix(&mut h, sl.damp_left[s] as u64);
            mix_real(&mut h, sl.ratio[s]);
            mix_real(&mut h, sl.ref_measured[s]);
            mix_real(&mut h, sl.ref_estimate[s]);
            mix(&mut h, sl.settle_left[s] as u64);
        }
        mix(&mut h, sl.throttle.len() as u64);
        for &t in &sl.throttle {
            mix_real(&mut h, t);
        }
        mix(&mut h, sl.near.len() as u64);
        for &n in &sl.near {
            mix(&mut h, n as u64);
        }
        mix(&mut h, sl.pushed.len() as u64);
        for f in &sl.pushed {
            mix_real(&mut h, f.x);
            mix_real(&mut h, f.y);
            mix_real(&mut h, f.z);
        }
    }
    // Vessels (phase 5.10), only once a split has happened: before it every node is in vessel 0 and
    // the world hashes as before. Every node slot's vessel, in index order, and the next id.
    if world.vessel.ever {
        mix(&mut h, world.vessel.next as u64);
        mix(&mut h, world.pos.len() as u64);
        for i in 0..world.pos.len() as u32 {
            mix(&mut h, world.node_vessel(i) as u64);
        }
    }
    // Which segments a moving joint triggered in the last step, only once a joint exists.
    if world.joint_count() > 0 {
        mix(&mut h, world.seg_triggered.len() as u64);
        for &t in &world.seg_triggered {
            mix(&mut h, t as u64);
        }
    }
    h
}

/// The state hash without the segment section: the hash of everything that moves, collides or
/// pushes. Equal to [`state_hash`] for a world with no segments. The test that segment commands
/// change no physics compares this against the same run without them.
pub fn physics_hash(world: &World) -> u64 {
    physics_hash_state(world)
}

fn physics_hash_state(world: &World) -> u64 {
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
    // Thrusters, only when there are any (a world without them hashes as before phase 4.6). The
    // node generations are not mixed in: they follow from the commands, as node generations
    // already do for the nodes.
    if !world.thr_tail.is_empty() {
        mix(&mut h, world.thr_tail.len() as u64);
        for t in 0..world.thr_tail.len() {
            mix(&mut h, world.thr_tail[t] as u64);
            mix(&mut h, world.thr_head[t] as u64);
            mix_real(&mut h, world.thr_max[t]);
            mix_real(&mut h, world.thr_throttle[t]);
        }
    }
    // Rigid bodies (plan 005), only while one is alive, in slot order: a world that never made
    // one, or released every one, hashes as before. Bodies move, so they are physics.
    let bodies = world.body_count();
    if bodies > 0 {
        mix(&mut h, bodies as u64);
        for b in 0..world.body_slot_count() {
            if !world.body_alive(b) {
                continue;
            }
            let u = b as usize;
            mix(&mut h, b as u64);
            mix_vec(&mut h, world.body_centre[u]);
            let q = world.body_rot[u];
            for r in [q.w, q.x, q.y, q.z] {
                mix_real(&mut h, r);
            }
            mix_vec(&mut h, world.body_vel[u]);
            mix_vec(&mut h, world.body_omega[u]);
            mix_real(&mut h, world.body_mass[u]);
            let k = world.body_inv_inertia[u];
            for r in [k.xx, k.yy, k.zz, k.xy, k.xz, k.yz] {
                mix_real(&mut h, r);
            }
        }
    }
    // Joints (plan 005, phase 5.5), only once one exists: a world that never made one hashes as
    // before. Inputs only, in index order; the angle and torques are outputs of these and the pose.
    if world.joint_count() > 0 {
        mix(&mut h, world.joint_count() as u64);
        for j in 0..world.joint_count() as usize {
            for k in 0..6 {
                mix(&mut h, world.joint_nodes[j][k] as u64);
                mix(&mut h, world.joint_gen[j][k] as u64);
            }
            mix(&mut h, world.joint_mode[j] as u64);
            mix_real(&mut h, world.joint_min[j]);
            mix_real(&mut h, world.joint_max[j]);
            mix_real(&mut h, world.joint_target[j]);
            mix_real(&mut h, world.joint_rate[j]);
            mix_real(&mut h, world.joint_cap[j]);
            mix_real(&mut h, world.joint_zero[j]);
        }
    }
    // Air (plan 006, phase 6.1), only once `CMD_SET_AIR` has run: a world that never sent it
    // hashes as before. Air pushes, so it is physics.
    if world.air.ever {
        let a = &world.air;
        mix_real(&mut h, a.rho0);
        mix_real(&mut h, a.scale_height);
        mix_vec(&mut h, a.wind);
        mix_real(&mut h, a.sound_speed);
    }
    // Aero elements (phase 6.2), only once one exists.
    let el = &world.aero_el;
    if !el.kind.is_empty() {
        mix(&mut h, el.kind.len() as u64);
        for e in 0..el.kind.len() {
            mix(&mut h, el.kind[e] as u64);
            mix(&mut h, el.node_start[e] as u64);
            mix(&mut h, el.node_count[e] as u64);
            for k in 0..4 {
                mix(&mut h, el.frame[e][k] as u64);
                mix(&mut h, el.frame_gen[e][k] as u64);
            }
            for k in 0..6 {
                mix_real(&mut h, el.coef[k][e]);
            }
            mix_real(&mut h, el.deflection[e]);
        }
        for k in 0..el.node.len() {
            mix(&mut h, el.node[k] as u64);
            mix(&mut h, el.node_gen[k] as u64);
            mix_real(&mut h, el.weight[k]);
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
        let s = BeamSpec {
            rest: 1.0,
            compliance: 1e-6,
            damping: 0.1,
            yield_strain: 0.01,
            break_strain: 0.05,
        };
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
