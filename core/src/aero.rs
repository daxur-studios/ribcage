//! Air (plan 006): the world's air settings, the aero elements and the pass that turns them into force.
//!
//! Phase 6.1 added the settings and the density function. Phase 6.2 adds the aero **elements** (flat
//! arrays on [`Elements`], layout rule 3), the commands that build them, the body-station formula
//! as a pure function ([`station_force`]) and the pass ([`apply_aero`]) that evaluates every element
//! once per step and pushes the result onto its nodes through [`World::add_env_force`] (rule 7, the
//! environment door). Settings and elements enter the state hash only once they exist, so a world
//! that never sent an air or element command hashes as before.
//!
//! Density is `ρ0 · exp(-y / H)` with the core's own [`crate::maths::exp`] (a library `exp` differs
//! between platforms). Gravity stays flat. Only add, subtract, multiply, divide and square root
//! appear here; the transcendentals are the core's own.
//!
//! **Station axis and flow.** `x̂` runs tail to nose; `u = wind - v` is the air velocity the element
//! sees; `ua = u·x̂`. A craft flying nose first has `ua < 0`. The axial terms use the axial dynamic
//! pressure `½ ρ ua²` (full `q` at zero angle of attack, nothing at 90°), the normal terms the full
//! `q`. The reference area of the plan's `C_N` cancels (`q A_ref C_N` is `q (A_b sin 2α cos(α/2) +
//! η C_dc A_p sin² α)`), so no reference area is stored.

use crate::maths::{atan2, cos, exp, sin};
use crate::real::{Real, Vec3};
use crate::world::World;

/// Sea-level air density in kg/m³ ([`Air::default`]).
pub const DEFAULT_RHO0: Real = 1.225;
/// Density scale height in metres ([`Air::default`]); zero means constant density.
pub const DEFAULT_SCALE_HEIGHT: Real = 8500.0;
/// Speed of sound in m/s ([`Air::default`]).
pub const DEFAULT_SOUND_SPEED: Real = 340.0;

/// The air the world sits in. Read by [`apply_aero`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Air {
    /// `CMD_SET_AIR` has run at least once: the settings are hashed from then.
    pub ever: bool,
    /// Sea-level density, kg/m³ (zero or more).
    pub rho0: Real,
    /// Scale height, metres (zero or more); zero means constant density `rho0`.
    pub scale_height: Real,
    /// Wind velocity, m/s.
    pub wind: Vec3,
    /// Speed of sound, m/s (above zero).
    pub sound_speed: Real,
}

impl Default for Air {
    fn default() -> Air {
        Air {
            ever: false,
            rho0: DEFAULT_RHO0,
            scale_height: DEFAULT_SCALE_HEIGHT,
            wind: Vec3::ZERO,
            sound_speed: DEFAULT_SOUND_SPEED,
        }
    }
}

impl Air {
    /// Air density at height `y` metres: `rho0 · exp(-y / H)`, or `rho0` when `H` is zero.
    pub fn density(&self, y: Real) -> Real {
        if self.scale_height == 0.0 {
            self.rho0
        } else {
            self.rho0 * exp(-y / self.scale_height)
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Elements (phase 6.2)
// ---------------------------------------------------------------------------------------------

/// Element kind: body station (axial drag, slender-body lift, crossflow drag, base pressure).
/// Coefficients: `[A_front, C_A, A_base, A_plan, C_base, 0]`. Strips (6.4) and plates (6.5) take
/// the next kinds.
pub const AERO_STATION: u32 = 1;
/// A frame slot with no node.
pub const NO_FRAME: u32 = u32::MAX;
/// Crossflow efficiency factor `η` in the station's normal coefficient.
pub const ETA: Real = 0.7;
/// Crossflow drag coefficient `C_dc` of a cylinder in the station's normal coefficient.
pub const C_DC: Real = 1.2;

/// The aero elements, one flat array per field (layout rule 3). Indices are stable; a dead node
/// makes its element inert (every node and frame node keeps the generation it had when it was
/// declared), never moves another element into its slot.
#[derive(Clone, Debug, Default)]
pub struct Elements {
    pub kind: Vec<u32>,
    /// Range into [`Self::node`] / [`Self::node_gen`] / [`Self::weight`]; elements own their ranges
    /// in order, so nodes can only be appended to the newest element.
    pub node_start: Vec<u32>,
    pub node_count: Vec<u32>,
    /// Frame nodes (station: tail, nose; the others [`NO_FRAME`]).
    pub frame: Vec<[u32; 4]>,
    pub frame_gen: Vec<[u32; 4]>,
    /// Coefficients, one array per slot (meaning per kind, see the kind constants).
    pub coef: [Vec<Real>; 6],
    /// Control deflection (strips) or deploy fraction (plates); unused by stations.
    pub deflection: Vec<Real>,
    pub node: Vec<u32>,
    pub node_gen: Vec<u32>,
    pub weight: Vec<Real>,
}

/// A body station's coefficients.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Station {
    /// Frontal area, m².
    pub a_front: Real,
    /// Axial drag coefficient on the frontal area.
    pub c_axial: Real,
    /// Base area (slender-body term and base pressure), m².
    pub a_base: Real,
    /// Plan area (crossflow term), m².
    pub a_plan: Real,
    /// Base-pressure coefficient; zero means no base is declared.
    pub c_base: Real,
}

/// The force of a body station on its body, newtons. `axis` is the unit vector tail to nose, `u`
/// the air velocity the station sees (wind minus its velocity), `rho` the density. `base_on` says
/// the base pressure acts (the station has a base and no thruster on it is firing).
///
/// Axial: `½ρ ua² (A_f C_A + A_b C_base)` along the axial flow (`sign(ua) x̂`). Normal:
/// `q (A_b sin 2α cos(α/2) + η C_dc A_p sin² α)` along the crossflow, `α = atan2(|u_perp|, -ua)`,
/// the angle between `u` and `-x̂`. Own `sin`, `cos`, `atan2`; no library transcendentals.
pub fn station_force(c: &Station, axis: Vec3, u: Vec3, rho: Real, base_on: bool) -> Vec3 {
    let speed2 = u.dot(u);
    if !(speed2 > 0.0) {
        return Vec3::ZERO;
    }
    let ua = u.dot(axis);
    let perp = u - axis * ua;
    let q = 0.5 * rho * speed2;
    let qa = 0.5 * rho * ua * ua;
    let sign: Real = if ua > 0.0 {
        1.0
    } else if ua < 0.0 {
        -1.0
    } else {
        0.0
    };
    let base = if base_on { c.a_base * c.c_base } else { 0.0 };
    let mut f = axis * (sign * qa * (c.a_front * c.c_axial + base));
    let pl = perp.length();
    if pl > 0.0 {
        let alpha = atan2(pl, -ua);
        let s = sin(alpha);
        let cn = c.a_base * sin(2.0 * alpha) * cos(0.5 * alpha) + ETA * C_DC * c.a_plan * s * s;
        f = f + perp * (q * cn / pl);
    }
    f
}

impl World {
    pub fn aero_element_count(&self) -> u32 {
        self.aero_el.kind.len() as u32
    }

    /// Adds an element of `kind` with its coefficients; frame and nodes follow in later records
    /// (until both exist it is inert). `None` for an unknown kind or a coefficient that is not a
    /// finite number of zero or more.
    pub fn add_aero_element(&mut self, kind: u32, coef: [Real; 6]) -> Option<u32> {
        if kind != AERO_STATION || coef.iter().any(|c| !(c.is_finite() && *c >= 0.0)) {
            return None;
        }
        let e = &mut self.aero_el;
        e.kind.push(kind);
        e.node_start.push(e.node.len() as u32);
        e.node_count.push(0);
        e.frame.push([NO_FRAME; 4]);
        e.frame_gen.push([0; 4]);
        for k in 0..6 {
            e.coef[k].push(coef[k]);
        }
        e.deflection.push(0.0);
        Some(e.kind.len() as u32 - 1)
    }

    /// Appends a weighted node to the **newest** element. `false` (nothing changed) when `element`
    /// is not the newest, the node is not live, or the weight is not a finite number above zero.
    pub fn add_aero_node(&mut self, element: u32, node: u32, weight: Real) -> bool {
        let n = self.aero_el.kind.len() as u32;
        if n == 0 || element != n - 1 || !self.aero_node_ok(node) || !(weight > 0.0 && weight.is_finite()) {
            return false;
        }
        let gen = self.node_gen[node as usize];
        let e = &mut self.aero_el;
        e.node.push(node);
        e.node_gen.push(gen);
        e.weight.push(weight);
        e.node_count[element as usize] += 1;
        true
    }

    /// Sets an element's frame nodes (`NO_FRAME` for an unused slot). `false` (nothing changed) when
    /// the element does not exist, a given node is not live, or the first two (a station's tail and
    /// nose) are missing or the same.
    pub fn set_aero_frame(&mut self, element: u32, frame: [u32; 4]) -> bool {
        if element >= self.aero_element_count() {
            return false;
        }
        if frame[0] == NO_FRAME || frame[1] == NO_FRAME || frame[0] == frame[1] {
            return false;
        }
        if frame.iter().any(|&f| f != NO_FRAME && !self.aero_node_ok(f)) {
            return false;
        }
        let gen = frame.map(|f| if f == NO_FRAME { 0 } else { self.node_gen[f as usize] });
        self.aero_el.frame[element as usize] = frame;
        self.aero_el.frame_gen[element as usize] = gen;
        true
    }

    fn aero_node_ok(&self, i: u32) -> bool {
        i < self.node_count() && self.node_alive(i)
    }

    fn aero_node_live(&self, i: u32, gen: u32) -> bool {
        self.aero_node_ok(i) && self.node_gen[i as usize] == gen
    }

    /// Whether any firing thruster has an end among the element's nodes.
    fn thruster_fires_on(&self, e: usize, firing: &[(u32, u32)]) -> bool {
        let el = &self.aero_el;
        let (s, n) = (el.node_start[e] as usize, el.node_count[e] as usize);
        for &(tail, head) in firing {
            for k in s..s + n {
                if el.node[k] == tail || el.node[k] == head {
                    return true;
                }
            }
        }
        false
    }
}

/// Adds the air force of every element, in ascending element order, from the positions and
/// velocities the step starts with, through [`World::add_env_force`] split over the element's
/// nodes by weight (normalised here, so the weights need not sum to one). Elements with a dead
/// node, no frame, no nodes, no air or no airspeed give nothing; a non-finite result is dropped.
/// Called once per step after thrust and before the estimate reads the forces.
///
/// A rider node's world velocity at the start of the step is its body's `V + ω × r`: the island
/// solve ends by writing exactly that into `vel` for every rider (`solver.rs`, "Riders follow their
/// bodies"), so the pass reads `vel` as it is.
pub fn apply_aero(world: &mut World) {
    let count = world.aero_el.kind.len();
    if count == 0 {
        return;
    }
    let mut firing: Vec<(u32, u32)> = Vec::new();
    for t in 0..world.thr_tail.len() {
        if world.thr_throttle[t] > 0.0 && world.thruster_nodes_live(t) {
            firing.push((world.thr_tail[t], world.thr_head[t]));
        }
    }
    let air = world.air;
    for e in 0..count {
        let (s, n) = (world.aero_el.node_start[e] as usize, world.aero_el.node_count[e] as usize);
        let frame = world.aero_el.frame[e];
        if n == 0 || frame[0] == NO_FRAME || frame[1] == NO_FRAME {
            continue;
        }
        let fg = world.aero_el.frame_gen[e];
        let mut live = world.aero_node_live(frame[0], fg[0]) && world.aero_node_live(frame[1], fg[1]);
        let (mut wsum, mut py, mut vel) = (0.0 as Real, 0.0 as Real, Vec3::ZERO);
        for k in s..s + n {
            let (i, w) = (world.aero_el.node[k], world.aero_el.weight[k]);
            if !world.aero_node_live(i, world.aero_el.node_gen[k]) {
                live = false;
                break;
            }
            wsum += w;
            py += w * world.pos[i as usize].y;
            vel = vel + world.vel[i as usize] * w;
        }
        if !live || !(wsum > 0.0) {
            continue;
        }
        let rho = air.density(py / wsum);
        if !(rho > 0.0) {
            continue;
        }
        let u = air.wind - vel * (1.0 / wsum);
        let d = world.pos[frame[1] as usize] - world.pos[frame[0] as usize];
        let len = d.length();
        if !(len > 0.0) || !len.is_finite() {
            continue;
        }
        let el = &world.aero_el;
        let station = Station {
            a_front: el.coef[0][e],
            c_axial: el.coef[1][e],
            a_base: el.coef[2][e],
            a_plan: el.coef[3][e],
            c_base: el.coef[4][e],
        };
        let base_on = station.c_base > 0.0 && !world.thruster_fires_on(e, &firing);
        let f = station_force(&station, d * (1.0 / len), u, rho, base_on);
        if !f.is_finite() {
            continue;
        }
        for k in s..s + n {
            let (i, w) = (world.aero_el.node[k], world.aero_el.weight[k]);
            world.add_env_force(i, f * (w / wsum));
        }
    }
}
