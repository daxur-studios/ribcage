//! World storage: flat arrays, one per field (layout rule 3), stable `u32` indices (rule 4),
//! and one door for external forces (rule 7). The solver never reads these directly;
//! it works on a packed [`crate::island::Island`] (rule 5).

use crate::event::Event;
use crate::real::{Real, Vec3};
use std::cmp::Reverse;
use std::collections::BinaryHeap;

pub const NODE_ALIVE: u32 = 1;
pub const NODE_PINNED: u32 = 2;
/// Set on every movable node of an island that hit a bad number (see [`crate::solver`]). A frozen
/// node keeps its position and zero velocity, and any island containing it is not solved until
/// the host calls [`World::unfreeze_node`]. Part of the state hash.
pub const NODE_FROZEN: u32 = 4;

/// Default for [`World::max_speed`], in metres per second.
///
/// **[assumption]** 1000 m/s: far above anything a bridge or a vehicle reaches at milestone 1
/// scale, and far below the point where a substep of motion breaks the solver. Orbital speeds
/// (KSP scale) are for a floating origin or rigid bodies later, not for this clamp.
pub const DEFAULT_MAX_SPEED: Real = 1000.0;

pub const BEAM_ALIVE: u32 = 1;
pub const BEAM_BROKEN: u32 = 2;
/// Set the first time a beam yields (its `rest` has moved at least once).
pub const BEAM_YIELDED: u32 = 4;

/// Index plus generation. A stale handle (slot reused) is detected instead of aliasing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Handle {
    pub index: u32,
    pub generation: u32,
}

/// Parameters for a new beam.
#[derive(Clone, Copy, Debug)]
pub struct BeamSpec {
    pub rest: Real,
    pub compliance: Real,
    pub damping: Real,
    pub yield_strain: Real,
    pub break_strain: Real,
}

pub struct World {
    // Per node.
    pub pos: Vec<Vec3>,
    pub vel: Vec<Vec3>,
    pub inv_mass: Vec<Real>,
    pub radius: Vec<Real>,
    pub force: Vec<Vec3>,
    pub node_flags: Vec<u32>,
    pub node_gen: Vec<u32>,
    // Per beam.
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
    pub beam_gen: Vec<u32>,
    // Dead slots, handed out lowest index first (deterministic; no hash order).
    free_nodes: BinaryHeap<Reverse<u32>>,
    free_beams: BinaryHeap<Reverse<u32>>,
    /// Events from the last step, in island order then ascending beam order (rule 8). Cleared
    /// at the start of every step; the host reads it after the step.
    pub events: Vec<Event>,
    /// Integer step counter; time is derived from it, never accumulated.
    pub step_count: u64,
    /// Speed limit for every node, in metres per second ([`DEFAULT_MAX_SPEED`]). After each
    /// substep a faster node is scaled down to this speed and its position is reset to match.
    pub max_speed: Real,
    /// Nodes that were clamped to `max_speed` in the last step, each counted once however many
    /// substeps clamped it. Reset every step. A debugging number; there is no event for it.
    pub clamped_nodes: u32,
}

impl Default for World {
    fn default() -> World {
        World {
            pos: Vec::new(),
            vel: Vec::new(),
            inv_mass: Vec::new(),
            radius: Vec::new(),
            force: Vec::new(),
            node_flags: Vec::new(),
            node_gen: Vec::new(),
            beam_a: Vec::new(),
            beam_b: Vec::new(),
            rest: Vec::new(),
            rest0: Vec::new(),
            compliance: Vec::new(),
            damping: Vec::new(),
            yield_strain: Vec::new(),
            break_strain: Vec::new(),
            beam_flags: Vec::new(),
            last_lambda: Vec::new(),
            beam_gen: Vec::new(),
            free_nodes: BinaryHeap::new(),
            free_beams: BinaryHeap::new(),
            events: Vec::new(),
            step_count: 0,
            max_speed: DEFAULT_MAX_SPEED,
            clamped_nodes: 0,
        }
    }
}

impl World {
    pub fn new() -> World {
        World::default()
    }

    pub fn node_frozen(&self, i: u32) -> bool {
        self.node_flags[i as usize] & NODE_FROZEN != 0
    }

    /// Clears the frozen flag. The node's position and zero velocity stay as the freeze left
    /// them; the island is solved again from the next step. The host is responsible for having
    /// fixed whatever produced the bad number.
    pub fn unfreeze_node(&mut self, i: u32) {
        self.node_flags[i as usize] &= !NODE_FROZEN;
    }

    pub fn node_count(&self) -> u32 {
        self.pos.len() as u32
    }

    pub fn beam_count(&self) -> u32 {
        self.rest.len() as u32
    }

    pub fn node_alive(&self, i: u32) -> bool {
        self.node_flags[i as usize] & NODE_ALIVE != 0
    }

    pub fn beam_alive(&self, i: u32) -> bool {
        let f = self.beam_flags[i as usize];
        f & BEAM_ALIVE != 0 && f & BEAM_BROKEN == 0
    }

    /// `inv_mass` of zero pins the node in place.
    pub fn add_node(&mut self, pos: Vec3, inv_mass: Real, radius: Real) -> Handle {
        let flags = NODE_ALIVE | if inv_mass == 0.0 { NODE_PINNED } else { 0 };
        if let Some(Reverse(i)) = self.free_nodes.pop() {
            let u = i as usize;
            self.pos[u] = pos;
            self.vel[u] = Vec3::ZERO;
            self.inv_mass[u] = inv_mass;
            self.radius[u] = radius;
            self.force[u] = Vec3::ZERO;
            self.node_flags[u] = flags;
            Handle { index: i, generation: self.node_gen[u] }
        } else {
            self.pos.push(pos);
            self.vel.push(Vec3::ZERO);
            self.inv_mass.push(inv_mass);
            self.radius.push(radius);
            self.force.push(Vec3::ZERO);
            self.node_flags.push(flags);
            self.node_gen.push(0);
            Handle { index: self.node_count() - 1, generation: 0 }
        }
    }

    /// Changes a live node's inverse mass; zero pins it, anything else frees it. Velocity is
    /// kept (a pinned node's is not used), and a frozen flag is left alone.
    pub fn set_inv_mass(&mut self, i: u32, inv_mass: Real) {
        let u = i as usize;
        self.inv_mass[u] = inv_mass;
        if inv_mass == 0.0 {
            self.node_flags[u] |= NODE_PINNED;
        } else {
            self.node_flags[u] &= !NODE_PINNED;
        }
    }

    /// Both endpoints must be live nodes (a beam endpoint is always a node, rule 6).
    pub fn add_beam(&mut self, a: u32, b: u32, spec: BeamSpec) -> Handle {
        assert!(a != b && self.node_alive(a) && self.node_alive(b), "beam endpoints must be distinct live nodes");
        if let Some(Reverse(i)) = self.free_beams.pop() {
            let u = i as usize;
            self.beam_a[u] = a;
            self.beam_b[u] = b;
            self.rest[u] = spec.rest;
            self.rest0[u] = spec.rest;
            self.compliance[u] = spec.compliance;
            self.damping[u] = spec.damping;
            self.yield_strain[u] = spec.yield_strain;
            self.break_strain[u] = spec.break_strain;
            self.beam_flags[u] = BEAM_ALIVE;
            self.last_lambda[u] = 0.0;
            Handle { index: i, generation: self.beam_gen[u] }
        } else {
            self.beam_a.push(a);
            self.beam_b.push(b);
            self.rest.push(spec.rest);
            self.rest0.push(spec.rest);
            self.compliance.push(spec.compliance);
            self.damping.push(spec.damping);
            self.yield_strain.push(spec.yield_strain);
            self.break_strain.push(spec.break_strain);
            self.beam_flags.push(BEAM_ALIVE);
            self.last_lambda.push(0.0);
            self.beam_gen.push(0);
            Handle { index: self.beam_count() - 1, generation: 0 }
        }
    }

    pub fn node_handle_valid(&self, h: Handle) -> bool {
        (h.index as usize) < self.pos.len() && self.node_gen[h.index as usize] == h.generation && self.node_alive(h.index)
    }

    pub fn beam_handle_valid(&self, h: Handle) -> bool {
        (h.index as usize) < self.rest.len()
            && self.beam_gen[h.index as usize] == h.generation
            && self.beam_flags[h.index as usize] & BEAM_ALIVE != 0
    }

    /// Frees the slot (flag only; no other index moves). Beams touching the node are deleted too.
    pub fn delete_node(&mut self, i: u32) {
        if !self.node_alive(i) {
            return;
        }
        for b in 0..self.beam_count() {
            if self.beam_flags[b as usize] & BEAM_ALIVE != 0 && (self.beam_a[b as usize] == i || self.beam_b[b as usize] == i) {
                self.delete_beam(b);
            }
        }
        self.node_flags[i as usize] = 0;
        self.node_gen[i as usize] = self.node_gen[i as usize].wrapping_add(1);
        self.free_nodes.push(Reverse(i));
    }

    pub fn delete_beam(&mut self, i: u32) {
        if self.beam_flags[i as usize] & BEAM_ALIVE == 0 {
            return;
        }
        self.beam_flags[i as usize] = 0;
        self.beam_gen[i as usize] = self.beam_gen[i as usize].wrapping_add(1);
        self.free_beams.push(Reverse(i));
    }

    /// The one door for external force on a node (rule 7). Kept until the end of the step.
    pub fn add_force(&mut self, node: u32, f: Vec3) {
        let u = node as usize;
        self.force[u] = self.force[u] + f;
    }
}
