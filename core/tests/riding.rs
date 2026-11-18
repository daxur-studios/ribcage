//! Riding endpoints (plan 005, phase 5.3): a rigid body is one more endpoint of the one kernel,
//! so beams, beam damping and ground contacts push and turn it. Each test compares a body with
//! the live or pinned build it stands for; every bound is stated where it is checked, and
//! ER 005 records the measured numbers (run with `--nocapture` to see them).

use beam_core::event::{EVENT_BREAK, EVENT_FREEZE, EVENT_YIELD};
use beam_core::real::{Real, Vec3};
use beam_core::sim::{gravity_at, step, step_with_rule, step_with_substeps};
use beam_core::solver::SubstepRule;
use beam_core::terrain::Terrain;
use beam_core::world::{BeamSpec, World};
use beam_core::STEP_SECONDS;

const NO_PARENT: u32 = u32::MAX;

fn g() -> Vec3 {
    gravity_at(Vec3::ZERO)
}

/// Adds a segment holding the given nodes, each `(position, mass)`, all of `radius`.
fn segment_of(w: &mut World, nodes: &[(Vec3, Real)], radius: Real) -> (u32, Vec<u32>) {
    let s = w.add_segment(NO_PARENT, 0, 0.01, 1e-5).expect("segment");
    let mut ids = Vec::new();
    for &(p, m) in nodes {
        let n = w.add_node(p, 1.0 / m, radius).index;
        assert!(w.assign_node_segment(n, s));
        ids.push(n);
    }
    (s, ids)
}

/// The eight corners `centre + (±a, ±b, ±c)`, each of `mass`.
fn corners(centre: Vec3, a: Real, b: Real, c: Real, mass: Real) -> Vec<(Vec3, Real)> {
    (0..8)
        .map(|k| {
            let sx = if k & 1 != 0 { a } else { -a };
            let sy = if k & 2 != 0 { b } else { -b };
            let sz = if k & 4 != 0 { c } else { -c };
            (centre + Vec3::new(sx, sy, sz), mass)
        })
        .collect()
}

/// A body of `mass` held against gravity: a unit-half cube's corners and one node at the centre
/// of its bottom face (an eighth of the mass each... less a small share for the bottom node).
/// Returns `(world slot, corner nodes, bottom node)`. The holding force is applied by
/// [`hold`] every step, equally on the corners, so it adds no torque.
fn held_body(w: &mut World, centre: Vec3, mass: Real) -> (u32, Vec<u32>, u32) {
    let bottom_mass = mass / 100.0;
    let mut nodes = corners(centre, 0.5, 0.5, 0.5, (mass - bottom_mass) / 8.0);
    nodes.push((centre - Vec3::new(0.0, 0.5, 0.0), bottom_mass));
    let (s, ids) = segment_of(w, &nodes, 0.1);
    let slot = w.make_rigid(&[s]).expect("a cube is not degenerate");
    (slot, ids[..8].to_vec(), ids[8])
}

/// Pushes up on the corners with `weight` in total, an eighth each.
fn hold(w: &mut World, corners: &[u32], weight: Real) {
    for &n in corners {
        w.add_force(n, Vec3::new(0.0, weight / 8.0, 0.0));
    }
}

fn stiff(rest: Real, compliance: Real, damping: Real) -> BeamSpec {
    BeamSpec { rest, compliance, damping, yield_strain: 1e9, break_strain: 1e9 }
}

/// 1. A live pendulum on a held heavy body (10⁶ kg against a 1 kg bob) swings as the same
/// pendulum on a pin: the body's share of the motion is `m / M = 10⁻⁶`, so the bob's path may
/// differ by about that times the swing. Bound: 1e-5 of the 1.5 m arm, over 3 s. The hold
/// carries the bob's weight as well as the body's, so the pair as a whole does not fall (with
/// the body's weight alone it sinks `½ (m g / M) t²`, 4.4e-5 m in 3 s: real, but not the test).
#[test]
fn pendulum_on_a_heavy_body_matches_a_pin() {
    let arm = 1.5;
    let run = |on_body: bool| -> (Vec<Vec3>, u32) {
        let mut w = World::new();
        let centre = Vec3::new(0.0, 10.0, 0.0);
        let mass = 1.0e6;
        let (top, corners) = if on_body {
            let (_, corners, bottom) = held_body(&mut w, centre, mass);
            (bottom, corners)
        } else {
            (w.add_node(centre - Vec3::new(0.0, 0.5, 0.0), 0.0, 0.1).index, Vec::new())
        };
        let start = w.pos[top as usize];
        let bob = w.add_node(start + Vec3::new(arm, 0.0, 0.0), 1.0, 0.1).index;
        w.add_beam(top, bob, stiff(arm, 1e-7, 0.0));
        let mut path = Vec::new();
        let mut most = 0;
        for _ in 0..180 {
            if on_body {
                hold(&mut w, &corners, (mass + 1.0) * g().length());
            }
            most = most.max(step_with_rule(&mut w, SubstepRule::DEFAULT).max_substeps);
            path.push(w.pos[bob as usize] - start);
        }
        (path, most)
    };
    let (body, n_body) = run(true);
    let (pin, n_pin) = run(false);
    let worst = body.iter().zip(&pin).map(|(a, b)| (*a - *b).length()).fold(0.0, Real::max);
    let low = pin.iter().map(|p| p.y).fold(0.0, Real::min);
    eprintln!("pendulum: worst bob offset {worst:e} m over 3 s, lowest {low} m, substeps body {n_body} pin {n_pin}");
    assert!(low < -1.4, "the bob swung down ({low})");
    assert!(worst <= 1e-5 * arm, "bob paths differ by {worst:e} m");
    assert_eq!(n_body, n_pin, "the same substep count");
}

/// The chain of test 2: ten 6.6 kg links 0.5 m apart hanging from `top`. Returns the links.
fn chain_from(w: &mut World, top: u32, links: u32, compliance: Real) -> Vec<u32> {
    let mut prev = top;
    let mut ids = Vec::new();
    for k in 1..=links {
        let p = w.pos[top as usize] - Vec3::new(0.0, 0.5 * k as Real, 0.0);
        let n = w.add_node(p, 1.0 / 6.6, 0.05).index;
        w.add_beam(prev, n, stiff(0.5, compliance, 40.0));
        ids.push(n);
        prev = n;
    }
    ids
}

/// 2. The mass-ratio measurement: a light chain (ten 6.6 kg links, 121 to 1 against the body)
/// on a free 800 kg body, held up by a force equal to the whole weight, and the same chain on a
/// pin. After 10 s both settle to the same stretch, with the same substep count. Bound: the
/// stretch within 1 % of the pin's (the body is free to bob and tilt, the pin is not), and
/// the body's centre within 1 mm of where it started. The body's centre is its centre of mass,
/// 5 mm under the cube's middle because of the bottom node, so the drift is measured from
/// where the centre starts, not from the middle.
#[test]
fn light_chain_on_a_free_heavy_body_matches_a_pin() {
    let (links, compliance) = (10u32, 1e-6);
    let run = |on_body: bool| -> (Real, u32, Real) {
        let mut w = World::new();
        let centre = Vec3::new(0.0, 20.0, 0.0);
        let mass = 800.0;
        let (top, corners, slot) = if on_body {
            let (slot, corners, bottom) = held_body(&mut w, centre, mass);
            (bottom, corners, slot)
        } else {
            (w.add_node(centre - Vec3::new(0.0, 0.5, 0.0), 0.0, 0.1).index, Vec::new(), 0)
        };
        let start = if on_body { w.body_centre[slot as usize] } else { centre };
        let ids = chain_from(&mut w, top, links, compliance);
        let total = (mass + 6.6 * links as Real) * g().length();
        let mut most = 0;
        for _ in 0..600 {
            if on_body {
                hold(&mut w, &corners, total);
            }
            most = most.max(step_with_rule(&mut w, SubstepRule::DEFAULT).max_substeps);
        }
        let last = *ids.last().unwrap();
        let length = (w.pos[top as usize] - w.pos[last as usize]).length();
        let drift = if on_body { (w.body_centre[slot as usize] - start).length() } else { 0.0 };
        (length - 0.5 * links as Real, most, drift)
    };
    let (s_body, n_body, drift) = run(true);
    let (s_pin, n_pin, _) = run(false);
    // Hanging chain: beam k carries (links − k + 1) · 6.6 g, so the stretch is c · 6.6 g · Σk.
    // One XPBD pass per substep with λ starting at zero rests where `C = F (c + h² w)`, not
    // `C = F c`: the solve adds up to `h² w` to each beam's compliance in statics (Gauss-Seidel
    // takes some of it back). So the pin's stretch lies between the two.
    let load = 6.6 * g().length() * (links * (links + 1) / 2) as Real;
    let h = STEP_SECONDS / n_pin as Real;
    let (low, high) = (compliance * load, (compliance + h * h * 2.0 * 6.6_f64.recip()) * load);
    eprintln!(
        "mass ratio 800:6.6 — stretch body {s_body:e} m, pin {s_pin:e} m, statics {low:e}..{high:e} m, \
         substeps body {n_body} pin {n_pin}, body drift {drift:e} m"
    );
    assert!(low <= s_pin && s_pin <= high, "the pinned chain hangs between {low:e} and {high:e}");
    assert!((s_body - s_pin).abs() <= 0.01 * s_pin, "stretch {s_body:e} vs pin {s_pin:e}");
    assert_eq!(n_body, n_pin, "the same substep count");
    assert!(drift <= 1e-3, "the held body drifted {drift:e} m");
}

/// A unit cube of eight 2 kg corner nodes (radius 0.1) with its bottom face at `bottom`.
fn cube_nodes(bottom: Real) -> Vec<(Vec3, Real)> {
    corners(Vec3::new(0.0, bottom + 0.5, 0.0), 0.5, 0.5, 0.5, 2.0)
}

/// Kinetic plus gravitational energy of the listed 2 kg nodes.
fn cube_energy(w: &World, ids: &[u32]) -> Real {
    let gy = g().length();
    ids.iter().map(|&n| 1.0 * w.vel[n as usize].length_squared() + 2.0 * gy * w.pos[n as usize].y).sum()
}

/// The centre of the listed nodes.
fn mean(w: &World, ids: &[u32]) -> Vec3 {
    let mut s = Vec3::ZERO;
    for &n in ids {
        s = s + w.pos[n as usize];
    }
    s / ids.len() as Real
}

/// Drops a cube from 1 m onto flat ground (friction 0.6), rigid or as a lattice of all 28
/// node pairs. Returns the centre's height, the lowest node's height and the energy, per step,
/// over 65 s.
fn drop_cube(rigid: bool) -> (Vec<Real>, Vec<Real>, Vec<Real>) {
    let mut w = World::new();
    w.terrain = Some(Terrain::flat(2, 2, -100.0, -100.0, 200.0, 0.6));
    let (s, ids) = segment_of(&mut w, &cube_nodes(1.0), 0.1);
    // A small tilt so the cube does not land flat on all four corners at once (set before the
    // cube turns rigid: a body takes its velocity and spin from its nodes once, then owns them).
    for &n in &ids {
        let x = w.pos[n as usize].x;
        w.vel[n as usize] = Vec3::new(0.0, 0.3 * x, 0.0);
    }
    if rigid {
        w.make_rigid(&[s]).expect("a cube is not degenerate");
    } else {
        for a in 0..8 {
            for b in a + 1..8 {
                let rest = (w.pos[ids[b] as usize] - w.pos[ids[a] as usize]).length();
                w.add_beam(ids[a], ids[b], stiff(rest, 1e-7, 20.0));
            }
        }
    }
    let (mut height, mut lowest, mut energy) = (Vec::new(), Vec::new(), Vec::new());
    for _ in 0..(65 * 60) {
        step(&mut w);
        height.push(mean(&w, &ids).y);
        lowest.push(ids.iter().map(|&n| w.pos[n as usize].y).fold(Real::MAX, Real::min));
        energy.push(cube_energy(&w, &ids));
    }
    assert!(w.events.iter().all(|e| e.kind != EVENT_FREEZE));
    (height, lowest, energy)
}

/// 3. A rigid cube dropped on flat ground bounces and rests as the same cube built as a stiff
/// lattice: both touch down within a step of each other (a node within 2 cm of its radius off
/// the ground: a stiff lattice corner hits and turns back inside one 16 ms step; the lone body
/// falls on 4 substeps and the lattice on 32, and the integrator's gravity error `g t h / 2`
/// differs by millimetres between them), both bounce (the centre rises more than 1 mm after
/// the touch) and keep less than a third of the 0.9 m fall, the rest
/// height within the node radius (0.1 m) of the lattice's, and no energy gained at rest over
/// 60 s (from 5 s to 65 s, at most 1e-6 of the cube's weight times a metre).
#[test]
fn rigid_cube_lands_and_rests_like_a_stiff_lattice() {
    let (h_rigid, l_rigid, e_rigid) = drop_cube(true);
    let (h_lat, l_lat, e_lat) = drop_cube(false);
    let rebound = |h: &[Real], lowest: &[Real]| {
        let first = lowest.iter().position(|&y| y < 0.1 + 0.02).expect("it lands");
        let low = h[first..first + 30].iter().cloned().fold(Real::MAX, Real::min);
        let high = h[first..first + 120].iter().cloned().fold(Real::MIN, Real::max);
        (high - low, first)
    };
    let (b_rigid, t_rigid) = rebound(&h_rigid, &l_rigid);
    let (b_lat, t_lat) = rebound(&h_lat, &l_lat);
    let rest_rigid = *h_rigid.last().unwrap();
    let rest_lat = *h_lat.last().unwrap();
    let at5 = 5 * 60 - 1;
    let gain_rigid = e_rigid[at5..].iter().cloned().fold(Real::MIN, Real::max) - e_rigid[at5];
    let gain_lat = e_lat[at5..].iter().cloned().fold(Real::MIN, Real::max) - e_lat[at5];
    eprintln!(
        "cube drop — lands at step rigid {t_rigid} lattice {t_lat}; rebound rigid {b_rigid:e} m lattice {b_lat:e} m; \
         rest height rigid {rest_rigid} lattice {rest_lat}; energy gain 5–65 s rigid {gain_rigid:e} J lattice {gain_lat:e} J"
    );
    assert!(t_rigid.abs_diff(t_lat) <= 1, "touch-down step {t_rigid} vs {t_lat}");
    for b in [b_rigid, b_lat] {
        assert!(b > 1e-3 && b < 0.3, "rebound {b} m (rigid {b_rigid}, lattice {b_lat})");
    }
    assert!((rest_rigid - rest_lat).abs() <= 0.1, "rest {rest_rigid} vs {rest_lat}");
    assert!((rest_rigid - 0.6).abs() <= 0.1, "rest height {rest_rigid}, a node radius above half the cube");
    let weight_metre = 16.0 * g().length();
    assert!(gain_rigid <= 1e-6 * weight_metre, "rigid cube gained {gain_rigid:e} J at rest");
}

/// 4. A body resting on its four bottom corners does not creep or spin: over 60 s the centre
/// moves less than 1e-6 m sideways and the body turns less than 1e-6 rad about the vertical.
#[test]
fn body_on_four_corners_does_not_creep_or_spin() {
    let mut w = World::new();
    w.terrain = Some(Terrain::flat(2, 2, -100.0, -100.0, 200.0, 0.6));
    // Off-centre in x and z, and not square, so nothing cancels by symmetry of placement.
    let centre = Vec3::new(3.3, 0.1 + 0.25, -1.7);
    let (s, ids) = segment_of(&mut w, &corners(centre, 0.8, 0.25, 0.45, 5.0), 0.1);
    let slot = w.make_rigid(&[s]).expect("a box is not degenerate") as usize;
    for _ in 0..60 {
        step(&mut w); // settle into the contacts
    }
    let (c0, q0) = (w.body_centre[slot], w.body_rot[slot]);
    let x0 = w.pos[ids[1] as usize] - w.pos[ids[0] as usize];
    for _ in 0..3600 {
        step(&mut w);
    }
    let c1 = w.body_centre[slot];
    let x1 = w.pos[ids[1] as usize] - w.pos[ids[0] as usize];
    let creep = Vec3::new(c1.x - c0.x, 0.0, c1.z - c0.z).length();
    // Yaw from the bottom edge's direction in the ground plane.
    let yaw = (x0.x * x1.z - x0.z * x1.x) / (x0.x * x1.x + x0.z * x1.z);
    let sink = c1.y - c0.y;
    eprintln!("four corners, 60 s — creep {creep:e} m, yaw {yaw:e} rad, height change {sink:e} m, rot {q0:?} → {:?}", w.body_rot[slot]);
    assert!(creep <= 1e-6, "creep {creep:e} m");
    assert!(yaw.abs() <= 1e-6, "yaw {yaw:e} rad");
    assert!(sink.abs() <= 1e-6, "height change {sink:e} m");
}

/// Hangs a 10 kg load under a held 800 kg top on one weak beam, then pulls the load down with a
/// force growing 2 N per step. The top is a riding node (a held body) or a live 800 kg node
/// held the same way. Returns the pull at the step of the yield event and of the break event.
fn pull_until_break(riding: bool) -> (Real, Real) {
    let mut w = World::new();
    let centre = Vec3::new(0.0, 50.0, 0.0);
    let mass = 800.0;
    let (top, corners) = if riding {
        let (_, corners, bottom) = held_body(&mut w, centre, mass);
        (bottom, corners)
    } else {
        let top = w.add_node(centre - Vec3::new(0.0, 0.5, 0.0), 1.0 / mass, 0.1).index;
        (top, vec![top; 8])
    };
    let load = w.add_node(w.pos[top as usize] - Vec3::new(0.0, 1.0, 0.0), 0.1, 0.1).index;
    // F_y = yield_strain · rest0 / compliance = 0.002 / 1e-6 = 2000 N; break at 2 % strain.
    w.add_beam(top, load, BeamSpec { rest: 1.0, compliance: 1e-6, damping: 50.0, yield_strain: 0.002, break_strain: 0.02 });
    let (mut yielded, mut broke) = (Real::NAN, Real::NAN);
    for k in 0..4000 {
        let pull = 2.0 * k as Real;
        let total = (mass + 10.0) * g().length() + pull;
        hold(&mut w, &corners, total);
        w.add_force(load, Vec3::new(0.0, -pull, 0.0));
        step(&mut w);
        for e in &w.events {
            if e.kind == EVENT_YIELD && yielded.is_nan() {
                yielded = pull;
            }
            if e.kind == EVENT_BREAK && broke.is_nan() {
                broke = pull;
            }
        }
        if !broke.is_nan() {
            break;
        }
    }
    (yielded, broke)
}

/// 5. A beam between a riding node and a live node yields and breaks at the same pull as a
/// beam between two live nodes: within one step of the ramp (2 N) of each other, and the yield
/// within 5 % of the yield force less the load's weight (2000 − 98 N).
#[test]
fn riding_end_yields_and_breaks_like_a_live_end() {
    let (y_ride, b_ride) = pull_until_break(true);
    let (y_live, b_live) = pull_until_break(false);
    eprintln!("yield at pull riding {y_ride} N, live {y_live} N; break riding {b_ride} N, live {b_live} N");
    assert!(y_ride.is_finite() && b_ride.is_finite(), "the riding beam yielded and broke");
    assert!((y_ride - y_live).abs() <= 2.0, "yield {y_ride} vs {y_live}");
    assert!((b_ride - b_live).abs() <= 2.0, "break {b_ride} vs {b_live}");
    let expect = 2000.0 - 10.0 * g().length();
    assert!((y_live - expect).abs() <= 0.05 * expect, "yield pull {y_live} vs {expect}");
}

/// The rounding unit of momentum in a step: total mass times `ε |x| / h`, the error of one
/// velocity derived from positions as far out as `|x|` over one substep `h`.
fn rounding_unit(w: &World, m: Real, substeps: u32) -> Real {
    let far = (0..w.node_count() as usize).map(|i| w.pos[i].length()).fold(0.0, Real::max);
    m * Real::EPSILON * far * (substeps as Real / STEP_SECONDS)
}

/// Momentum of every loose live node and every body.
fn momentum(w: &World) -> Vec3 {
    let mut p = Vec3::ZERO;
    for i in 0..w.node_count() as usize {
        if w.node_alive(i as u32) && w.inv_mass[i] != 0.0 && w.node_body[i] == u32::MAX {
            p = p + w.vel[i] / w.inv_mass[i];
        }
    }
    for b in 0..w.body_slot_count() as usize {
        if w.body_alive(b as u32) {
            p = p + w.body_vel[b] * w.body_mass[b];
        }
    }
    p
}

/// A spinning, moving 16 kg box with a live tetrahedron of 1 kg nodes beamed to two of its
/// corners, in the air. Returns `(world, total mass)`.
fn box_and_lattice() -> (World, Real) {
    let mut w = World::new();
    let c = Vec3::new(0.0, 200.0, 0.0);
    let (s, ids) = segment_of(&mut w, &corners(c, 0.6, 0.4, 0.3, 2.0), 0.1);
    for &n in &ids {
        let r = w.pos[n as usize] - c;
        w.vel[n as usize] = Vec3::new(1.0, 2.0, -0.5) + Vec3::new(0.4, 1.3, -0.7).cross(r);
    }
    w.make_rigid(&[s]).expect("a box is not degenerate");
    let tip = [Vec3::new(1.5, 0.0, 0.0), Vec3::new(2.3, 0.6, 0.2), Vec3::new(2.0, -0.5, 0.6), Vec3::new(2.1, 0.1, -0.7)];
    let tet: Vec<u32> = tip.iter().map(|&p| w.add_node(c + p, 1.0, 0.1).index).collect();
    for a in 0..4 {
        for b in a + 1..4 {
            let rest = (w.pos[tet[b] as usize] - w.pos[tet[a] as usize]).length();
            w.add_beam(tet[a], tet[b], stiff(rest, 1e-6, 5.0));
        }
    }
    for &(corner, node) in &[(ids[1], tet[0]), (ids[3], tet[0]), (ids[7], tet[1])] {
        let rest = (w.pos[node as usize] - w.pos[corner as usize]).length();
        // Start the joints stretched 5 %, so the beams work hard from the first step.
        w.add_beam(corner, node, stiff(rest * 0.95, 1e-6, 5.0));
    }
    (w, 16.0 + 4.0)
}

/// Runs the box and lattice for `steps` at `substeps` (0: the rule) and returns the worst
/// `|P − P0 − M g t|` in rounding units ([`rounding_unit`]) and the change of the body's spin.
fn momentum_run(steps: u32, substeps: u32) -> (Real, Real, World) {
    let (mut w, m) = box_and_lattice();
    let p0 = momentum(&w);
    let w0 = w.body_omega[0];
    let mut worst: Real = 0.0;
    for k in 1..=steps {
        let n = if substeps == 0 { step(&mut w) } else { step_with_substeps(&mut w, substeps) };
        assert!(w.events.iter().all(|e| e.kind != EVENT_FREEZE), "{substeps} substeps froze");
        let t = k as Real * STEP_SECONDS;
        let off = (momentum(&w) - p0 - g() * (m * t)).length();
        worst = worst.max(off / rounding_unit(&w, m, n));
    }
    let turned = (w.body_omega[0] - w0).length();
    (worst, turned, w)
}

/// 6. Momentum of a body plus a live lattice changes only by gravity: `P − M g t` stays at its
/// start value to rounding while the beams turn the body. Bound: 1000 rounding units over 10 s
/// (the drift wanders with the rounding of the positions, about 1e-8 kg m/s at 200 m up; it
/// does not grow with time, and it grows with distance from the origin, as rounding does; a
/// missing reaction would show as a share of `M g t`, millions of units).
#[test]
fn body_and_lattice_keep_momentum_less_gravity() {
    let (worst, turned, _) = momentum_run(600, 0);
    eprintln!("momentum: worst |P − P0 − M g t| = {worst:.2} rounding units over 10 s; spin changed by {turned} rad/s");
    assert!(turned > 0.01, "the beams turned the body");
    assert!(worst <= 1000.0, "momentum drift {worst} rounding units");
}

/// 7. A fixed substep count works with a body: the box and lattice of test 6 run 2 s at 1, 4,
/// 8 and 32 substeps without a freeze, keep momentum less gravity to 1000 rounding units at
/// each count, and the body's centre comes closer to the 32-substep run as the count rises
/// (the scene starts with its joints 5 % stretched and spins hard, so the counts differ by
/// centimetres; the check is that the difference shrinks).
#[test]
fn fixed_substeps_work_with_a_body() {
    let mut centres = Vec::new();
    for n in [1u32, 4, 8, 32] {
        let (worst, _, w) = momentum_run(120, n);
        assert!(w.body_centre[0].is_finite());
        assert!(worst <= 1000.0, "{n} substeps: momentum drift {worst} rounding units");
        centres.push(w.body_centre[0]);
    }
    let d1 = (centres[0] - centres[3]).length();
    let d4 = (centres[1] - centres[3]).length();
    let d8 = (centres[2] - centres[3]).length();
    eprintln!("fixed substeps: body centre off the 32-substep run by {d1:e} m at 1, {d4:e} m at 4, {d8:e} m at 8");
    assert!(d8 < d4 && d4 < d1, "{d1} {d4} {d8}");
}
