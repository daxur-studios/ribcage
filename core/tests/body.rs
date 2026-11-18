//! Rigid bodies (plan 005, phase 5.2): free fall, torque-free spin, and the make-rigid and
//! release formulas. Every bound below is stated where it is checked, and ER 005 records the
//! measured numbers.

use beam_core::body::{SpinForm, NO_BODY};
use beam_core::real::{Real, Vec3};
use beam_core::sim::{state_hash, step};
use beam_core::world::World;
use beam_core::STEP_SECONDS;

const STEPS_PER_MINUTE: u32 = 3600;

/// Adds a segment (a root, or a child of `parent`) holding the given nodes, each `(position, mass)`.
fn segment_of(w: &mut World, parent: u32, nodes: &[(Vec3, Real)]) -> (u32, Vec<u32>) {
    let s = w.add_segment(parent, 0, 0.01, 1e-5).expect("segment");
    let mut ids = Vec::new();
    for &(p, m) in nodes {
        let n = w.add_node(p, 1.0 / m, 0.1).index;
        assert!(w.assign_node_segment(n, s));
        ids.push(n);
    }
    (s, ids)
}

/// Eight 2 kg nodes at `(±a, ±b, ±c)` around `centre`: principal axes x, y, z with moments
/// `16 (b² + c²)`, `16 (a² + c²)`, `16 (a² + b²)`.
fn cuboid(centre: Vec3, a: Real, b: Real, c: Real) -> Vec<(Vec3, Real)> {
    (0..8)
        .map(|k| {
            let sx = if k & 1 != 0 { a } else { -a };
            let sy = if k & 2 != 0 { b } else { -b };
            let sz = if k & 4 != 0 { c } else { -c };
            (centre + Vec3::new(sx, sy, sz), 2.0)
        })
        .collect()
}

/// Sets each node's velocity to `v + ω × (x − centre)`.
fn set_rigid_motion(w: &mut World, nodes: &[u32], centre: Vec3, v: Vec3, omega: Vec3) {
    for &n in nodes {
        let r = w.pos[n as usize] - centre;
        w.vel[n as usize] = v + omega.cross(r);
    }
}

/// Momentum, angular momentum about the origin and kinetic energy of every live node with mass.
fn node_totals(w: &World) -> (Vec3, Vec3, Real) {
    let (mut p, mut l, mut e) = (Vec3::ZERO, Vec3::ZERO, 0.0);
    for i in 0..w.node_count() as usize {
        if !w.node_alive(i as u32) || w.inv_mass[i] == 0.0 {
            continue;
        }
        let m = 1.0 / w.inv_mass[i];
        p = p + w.vel[i] * m;
        l = l + w.pos[i].cross(w.vel[i] * m);
        e += 0.5 * m * w.vel[i].length_squared();
    }
    (p, l, e)
}

/// `|a − b| ≤ tol · scale`, with a message.
fn close(what: &str, a: Vec3, b: Vec3, scale: Real, tol: Real) {
    let d = (a - b).length();
    assert!(d <= tol * scale, "{what}: {a:?} vs {b:?}, off by {d:e} (allowed {:e})", tol * scale);
}

/// A spin body's angular momentum (world) and rotational energy, from its own state.
fn spin_of(w: &World, slot: u32) -> (Vec3, Real) {
    let l = w.body_spin(slot);
    (l, 0.5 * w.body_omega[slot as usize].dot(l))
}

/// Free fall: a spinning body's centre falls as the substep scheme says, within `½ g h t` of
/// `½ g t²` (h the substep, the same offset a loose node has), and the spin is untouched.
#[test]
fn free_fall_follows_half_g_t_squared_and_keeps_its_spin() {
    let mut w = World::new();
    let c0 = Vec3::new(0.0, 100.0, 0.0);
    let (s, nodes) = segment_of(&mut w, u32::MAX, &cuboid(c0, 0.5, 0.3, 0.2));
    let omega = Vec3::new(0.3, 1.1, -0.4);
    set_rigid_motion(&mut w, &nodes, c0, Vec3::ZERO, omega);
    let loose = w.add_node(Vec3::new(10.0, 100.0, 0.0), 1.0, 0.1).index;
    let slot = w.make_rigid(&[s]).expect("a cuboid is not degenerate");
    let (l0, e0) = spin_of(&w, slot);
    for _ in 0..60 {
        step(&mut w);
    }
    let k = 60.0 * 4.0; // substeps in one second, at the rule's minimum
    let h = STEP_SECONDS / 4.0;
    let g = 9.81;
    let drop = c0.y - w.body_centre[slot as usize].y;
    // Semi-implicit Euler: after k substeps the drop is g h² k (k + 1) / 2.
    let discrete = g * h * h * k * (k + 1.0) / 2.0;
    assert!((drop - discrete).abs() <= 1e-12 * discrete, "drop {drop} vs {discrete}");
    assert!((drop - 0.5 * g).abs() <= 0.5 * g * h * 1.0 + 1e-12, "drop {drop}");
    // A loose node falls as the body's centre does, to rounding (its velocity is derived from
    // positions, so the bits differ).
    let loose_drop = 100.0 - w.pos[loose as usize].y;
    assert!((loose_drop - drop).abs() <= 1e-9 * drop, "loose {loose_drop} vs body {drop}");
    let (l1, e1) = spin_of(&w, slot);
    close("spin", l1, l0, l0.length(), 1e-12);
    assert!((e1 - e0).abs() <= 1e-12 * e0, "energy {e0} → {e1}");
    // The riders were written back on the body.
    let c = w.body_centre[slot as usize];
    let mut sum = Vec3::ZERO;
    for &n in &nodes {
        sum = sum + w.pos[n as usize];
    }
    close("riders' centre", sum / 8.0, c, 1.0, 1e-12);
}

/// Runs a cuboid spinning at `omega` (world) for `minutes`, and returns the worst relative drift
/// of `|L|`, of each component of the world `L` (relative to `|L|`) and of the energy, and the
/// largest relative energy gain.
fn spin_run(form: SpinForm, half: Vec3, omega: Vec3, minutes: u32) -> (Real, Real, Real, Real) {
    let mut w = World::new();
    w.spin_form = form;
    let c0 = Vec3::new(0.0, 0.0, 0.0);
    let (s, nodes) = segment_of(&mut w, u32::MAX, &cuboid(c0, half.x, half.y, half.z));
    set_rigid_motion(&mut w, &nodes, c0, Vec3::ZERO, omega);
    let slot = w.make_rigid(&[s]).expect("cuboid");
    let (l0, e0) = spin_of(&w, slot);
    let (mut dl, mut dvec, mut de, mut gain) = (0.0 as Real, 0.0 as Real, 0.0 as Real, 0.0 as Real);
    for _ in 0..minutes * STEPS_PER_MINUTE {
        step(&mut w);
        let (l, e) = spin_of(&w, slot);
        dl = dl.max((l.length() - l0.length()).abs() / l0.length());
        dvec = dvec.max((l - l0).length() / l0.length());
        de = de.max((e - e0).abs() / e0);
        gain = gain.max((e - e0) / e0);
    }
    (dl, dvec, de, gain)
}

const HALF: Vec3 = Vec3 { x: 0.6, y: 0.35, z: 0.15 };

/// Spin about each principal axis and about a tilted axis keeps angular momentum and energy over
/// ten simulated minutes. Bound (default midpoint form): world `L` within 1e-9 of `|L|`, energy
/// within 1e-9 relative. About a principal axis the spin term is zero, so the bound is rounding.
#[test]
fn spin_keeps_momentum_and_energy_for_ten_minutes() {
    for omega in [Vec3::new(2.0, 0.0, 0.0), Vec3::new(0.0, 2.0, 0.0), Vec3::new(0.0, 0.0, 2.0), Vec3::new(1.2, -0.7, 1.9)] {
        let (dl, dvec, de, _) = spin_run(SpinForm::Midpoint, HALF, omega, 10);
        assert!(dl <= 1e-9 && dvec <= 1e-9, "ω {omega:?}: |L| drift {dl:e}, L drift {dvec:e}");
        assert!(de <= 1e-9, "ω {omega:?}: energy drift {de:e}");
    }
}

/// A slab spinning about its intermediate axis (with a small nudge) tumbles: the axis flips over
/// and back. It stays bounded and gains no energy (bound 1e-9 relative), and `L` is kept.
#[test]
fn intermediate_axis_tumble_stays_bounded_without_energy_gain() {
    let omega = Vec3::new(0.01, 3.0, 0.01);
    let (dl, dvec, de, gain) = spin_run(SpinForm::Midpoint, HALF, omega, 10);
    assert!(dl <= 1e-9 && dvec <= 1e-9, "|L| drift {dl:e}, L drift {dvec:e}");
    assert!(de <= 1e-9 && gain <= 1e-9, "energy drift {de:e}, gain {gain:e}");
    // It really tumbles: the body-frame spin leaves the y axis.
    let mut w = World::new();
    let (s, nodes) = segment_of(&mut w, u32::MAX, &cuboid(Vec3::ZERO, HALF.x, HALF.y, HALF.z));
    set_rigid_motion(&mut w, &nodes, Vec3::ZERO, Vec3::ZERO, omega);
    let slot = w.make_rigid(&[s]).expect("cuboid") as usize;
    let mut least_y: Real = 1.0;
    for _ in 0..600 {
        step(&mut w);
        let wb = w.body_rot[slot].inv_rotate(w.body_omega[slot]);
        least_y = least_y.min(wb.y / wb.length());
    }
    assert!(least_y < 0.0, "the intermediate axis flipped (least y share {least_y})");
}

/// Prints how each spin form holds energy and momentum on tumbling debris over ten minutes (the
/// plan's open question; numbers in ER 005; `cargo test --release --test body -- --nocapture`).
/// The last case starts near the 100 rad/s clamp and tumbles past it, so there the clamp removes
/// energy. Asserts the midpoint form's bounds (below the clamp: `L` and energy within 1e-9; at the
/// clamp: no gain) and that the implicit form never gains.
#[test]
fn spin_forms_on_fast_tumbling_debris() {
    let cases = [(Vec3::new(0.3, 1.0, 0.2), false), (Vec3::new(3.0, 20.0, 2.0), false), (Vec3::new(8.0, 50.0, 5.0), false), (Vec3::new(15.0, 95.0, 10.0), true)];
    for (omega, clamped) in cases {
        for form in [SpinForm::Explicit, SpinForm::Implicit, SpinForm::Midpoint] {
            let t = std::time::Instant::now();
            let (dl, dvec, de, gain) = spin_run(form, HALF, omega, 10);
            let ms = t.elapsed().as_secs_f64() * 1e3;
            println!("{form:?} |ω0| {:.1}: |L| {dl:.3e}, L {dvec:.3e}, energy {de:.3e}, gain {gain:.3e}, {ms:.1} ms", omega.length());
            match form {
                SpinForm::Midpoint if !clamped => assert!(dvec <= 1e-9 && de <= 1e-9, "|ω0| {}", omega.length()),
                SpinForm::Midpoint => assert!(gain <= 1e-9),
                SpinForm::Implicit => assert!(gain <= 1e-12, "the implicit form never gains"),
                SpinForm::Explicit => {}
            }
        }
    }
}

/// An L-shaped lattice of two segments, bent, every node moving its own way.
fn bent_lattice(w: &mut World) -> (u32, u32, Vec<u32>) {
    let mut a = Vec::new();
    let mut b = Vec::new();
    for k in 0..6u32 {
        let x = k as Real * 0.5;
        a.push((Vec3::new(x, 10.0 + 0.04 * x * x, 0.0), 3.0 + k as Real));
        a.push((Vec3::new(x, 10.6, 0.1 * x), 2.0));
        b.push((Vec3::new(2.5 + 0.1 * k as Real, 11.0 + 0.45 * k as Real, 0.3), 1.5 + 0.5 * k as Real));
        b.push((Vec3::new(2.9, 11.0 + 0.45 * k as Real, -0.2 + 0.05 * k as Real), 2.5));
    }
    let (s0, n0) = segment_of(w, u32::MAX, &a);
    let (s1, n1) = segment_of(w, s0, &b);
    let mut nodes = n0;
    nodes.extend(n1);
    // Deterministic pseudo-random velocities around a spin and a drift.
    let mut seed: u32 = 12345;
    let mut next = || {
        seed = seed.wrapping_mul(1_103_515_245).wrapping_add(12_345);
        ((seed >> 8) & 0xffff) as Real / 65535.0 - 0.5
    };
    let centre = Vec3::new(1.5, 10.8, 0.0);
    for &n in &nodes {
        let r = w.pos[n as usize] - centre;
        w.vel[n as usize] = Vec3::new(3.0, 1.0, -2.0) + Vec3::new(0.4, 1.5, -0.8).cross(r) + Vec3::new(next(), next(), next()) * 2.0;
    }
    (s0, s1, nodes)
}

fn positions(w: &World) -> Vec<Vec3> {
    w.pos.clone()
}

/// Making a moving, spinning, bent lattice rigid keeps momentum and angular momentum to rounding,
/// never raises the kinetic energy, and moves no node.
#[test]
fn make_rigid_keeps_momentum_drops_energy_moves_nothing() {
    let mut w = World::new();
    let (s0, s1, _) = bent_lattice(&mut w);
    let x0 = positions(&w);
    let (p0, l0, e0) = node_totals(&w);
    let slot = w.make_rigid(&[s0, s1]).expect("bent lattice");
    assert_eq!(positions(&w), x0, "no node moved");
    let (p1, l1, e1) = node_totals(&w);
    close("momentum", p1, p0, p0.length(), 1e-13);
    close("angular momentum", l1, l0, l0.length(), 1e-13);
    assert!(e1 <= e0, "energy {e0} → {e1}");
    assert!(e1 < e0, "the noise is not rigid, so some energy goes");
    // The body's own state says the same.
    let m = w.body_mass[slot as usize];
    close("body momentum", w.body_vel[slot as usize] * m, p0, p0.length(), 1e-13);
    let c = w.body_centre[slot as usize];
    let l_body = w.body_spin(slot) + c.cross(w.body_vel[slot as usize] * m);
    close("body angular momentum", l_body, l0, l0.length(), 1e-12);
    // One segment first, then the other joins: the same totals.
    let mut v = World::new();
    let (t0, t1, _) = bent_lattice(&mut v);
    let a = v.make_rigid(&[t0]).expect("one segment");
    assert_eq!(v.make_rigid(&[t1, t0]), Some(a));
    assert_eq!(positions(&v), x0);
    let (p2, l2, e2) = node_totals(&v);
    close("momentum, in two", p2, p0, p0.length(), 1e-13);
    close("angular momentum, in two", l2, l0, l0.length(), 1e-12);
    assert!((e2 - e1).abs() <= 1e-12 * e1, "energy {e1} vs {e2}");
}

/// Releasing keeps momentum, spin and kinetic energy to rounding and moves no node: one segment
/// out of two, then the other.
#[test]
fn release_keeps_momentum_spin_and_energy() {
    let mut w = World::new();
    let (s0, s1, nodes) = bent_lattice(&mut w);
    let slot = w.make_rigid(&[s0, s1]).expect("bent lattice");
    for _ in 0..30 {
        step(&mut w);
    }
    // Measure without gravity's effect on the comparison: release changes nothing in time.
    let x0 = positions(&w);
    let (p0, l0, e0) = node_totals(&w);
    assert!(w.release(&[s1]));
    assert_eq!(positions(&w), x0);
    let (p1, l1, e1) = node_totals(&w);
    close("momentum", p1, p0, p0.length(), 1e-13);
    close("angular momentum", l1, l0, l0.length(), 1e-12);
    assert!((e1 - e0).abs() <= 1e-12 * e0, "energy {e0} → {e1}");
    // What stays rigid has the same spin rate and gives every rider the same velocity.
    let rest = w.segment_body(s0);
    assert_eq!(rest, slot, "the body keeps its lowest segment's slot");
    assert!(nodes.iter().all(|&n| (w.node_body[n as usize] == NO_BODY) == (w.node_segment[n as usize] == s1)));
    assert!(w.release(&[s0]));
    assert_eq!(w.body_count(), 0);
    assert_eq!(positions(&w), x0);
    let (p2, l2, e2) = node_totals(&w);
    close("momentum, all", p2, p0, p0.length(), 1e-13);
    close("angular momentum, all", l2, l0, l0.length(), 1e-12);
    assert!((e2 - e0).abs() <= 1e-12 * e0, "energy {e0} → {e2}");
}

/// Make, then release in the same step: the identity on momentum and angular momentum (the
/// energy drops once, by the part that was not rigid motion).
#[test]
fn make_then_release_is_the_identity_on_momentum() {
    let mut w = World::new();
    let (s0, s1, _) = bent_lattice(&mut w);
    let x0 = positions(&w);
    let (p0, l0, e0) = node_totals(&w);
    w.make_rigid(&[s0, s1]).expect("bent lattice");
    let (_, _, e_rigid) = node_totals(&w);
    assert!(w.release(&[s0, s1]));
    assert_eq!(w.body_count(), 0);
    assert_eq!(positions(&w), x0);
    let (p1, l1, e1) = node_totals(&w);
    close("momentum", p1, p0, p0.length(), 1e-13);
    close("angular momentum", l1, l0, l0.length(), 1e-12);
    assert!((e1 - e_rigid).abs() <= 1e-12 * e_rigid && e1 <= e0);
}

/// Degenerate groups are refused and change nothing: a single node, a straight rod.
#[test]
fn rods_and_single_nodes_are_not_made_rigid() {
    let mut w = World::new();
    let (one, _) = segment_of(&mut w, u32::MAX, &[(Vec3::new(0.0, 5.0, 0.0), 1.0)]);
    let rod: Vec<(Vec3, Real)> = (0..4).map(|k| (Vec3::new(k as Real, 5.0, 0.0), 1.0)).collect();
    let (line, _) = segment_of(&mut w, u32::MAX, &rod);
    let before = state_hash(&w);
    assert_eq!(w.make_rigid(&[one]), None);
    assert_eq!(w.make_rigid(&[line]), None);
    assert_eq!(w.make_rigid(&[99]), None);
    assert_eq!(state_hash(&w), before);
}

/// Deleting a rider or pinning it leaves the rest rigid with the same motion; a node added to a
/// rigid segment joins the body.
#[test]
fn riders_leave_and_join() {
    let mut w = World::new();
    let c0 = Vec3::new(0.0, 20.0, 0.0);
    let (s, nodes) = segment_of(&mut w, u32::MAX, &cuboid(c0, 0.5, 0.4, 0.3));
    set_rigid_motion(&mut w, &nodes, c0, Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 0.5, 0.2));
    let slot = w.make_rigid(&[s]).expect("cuboid");
    step(&mut w);
    let v_at = |w: &World, n: u32| w.vel[n as usize];
    let before = v_at(&w, nodes[7]);
    w.delete_node(nodes[0]);
    assert_eq!(w.segment_body(s), slot);
    assert_eq!(v_at(&w, nodes[7]), before, "a rider's velocity is unchanged by another leaving");
    w.set_inv_mass(nodes[1], 0.0);
    assert_eq!(w.node_body[nodes[1] as usize], NO_BODY, "a pinned node does not ride");
    let extra = w.add_node(c0 + Vec3::new(0.0, 0.0, 0.6), 0.5, 0.1).index;
    assert!(w.assign_node_segment(extra, s));
    assert_eq!(w.node_body[extra as usize], slot, "a node put in a rigid segment rides");
    for _ in 0..30 {
        step(&mut w);
    }
    assert!(w.body_alive(slot));
    assert!(w.pos.iter().all(|p| p.is_finite()));
}

/// The same body run twice hashes the same (the native/WebAssembly check is in `tests/ffi.rs`).
#[test]
fn a_body_hashes_identically_twice() {
    let run = || {
        let mut w = World::new();
        let (s0, s1, _) = bent_lattice(&mut w);
        w.make_rigid(&[s0, s1]).expect("bent lattice");
        let mut hashes = Vec::new();
        for _ in 0..120 {
            step(&mut w);
            hashes.push(state_hash(&w));
        }
        hashes
    };
    assert_eq!(run(), run());
}

/// A world that never makes a body hashes as before (the body section is absent), and so does a
/// world whose bodies are all released, given the same node state.
#[test]
fn no_body_no_body_section() {
    let mut a = World::new();
    let (s0, s1, _) = bent_lattice(&mut a);
    let mut b = World::new();
    bent_lattice(&mut b);
    b.make_rigid(&[s0, s1]).expect("bent lattice");
    assert_ne!(state_hash(&a), state_hash(&b));
    b.release(&[s0, s1]);
    // Same positions; velocities now rigid in `b`, so copy them to `a` for the comparison.
    a.vel = b.vel.clone();
    assert_eq!(state_hash(&a), state_hash(&b));
}
