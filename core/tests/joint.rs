//! Hinges, joint limits, servos and rotors (plan 005, phase 5.5). Every bound is stated where it
//! is checked, and ER 005 records the measured numbers (run with `--nocapture` to see them).
//!
//! The scenes are built from small parts: a list of nodes whose first three are a joint side's
//! two axis nodes and its reference node, made pinned, live (every pair joined by a stiff beam)
//! or one rigid body. Side A is always the frame and side B the moving part, so the joint angle is
//! B's turn about the axis.

use beam_core::command::*;
use beam_core::joint::{wrap, JOINT_ROTOR, JOINT_SERVO};
use beam_core::maths::PI;
use beam_core::real::{Real, Vec3};
use beam_core::sim::{state_hash, step};
use beam_core::world::{BeamSpec, World};
use beam_core::STEP_SECONDS;

const NO_PARENT: u32 = u32::MAX;
const G: Real = 9.81;
/// The drive's torque is `Δλ / h²` with `|Δλ|` clamped to `cap · h²`, so it can round above the
/// cap by a few ulps and no more.
const CAP_SLACK: Real = 1e-12;

#[derive(Clone, Copy, PartialEq, Debug)]
enum Kind {
    Pinned,
    Live,
    Body,
}

/// Adds a part: each `(position, mass)` a node. Returns the node ids in the order given.
fn part(w: &mut World, kind: Kind, nodes: &[(Vec3, Real)]) -> Vec<u32> {
    let seg = if kind == Kind::Body || kind == Kind::Live {
        w.add_segment(NO_PARENT, 0, 0.01, 1e-5)
    } else {
        None
    };
    let mut ids = Vec::new();
    for &(p, m) in nodes {
        let inv = if kind == Kind::Pinned { 0.0 } else { 1.0 / m };
        let n = w.add_node(p, inv, 0.05).index;
        if let Some(s) = seg {
            assert!(w.assign_node_segment(n, s));
        }
        ids.push(n);
    }
    match kind {
        Kind::Live => {
            for x in 0..ids.len() {
                for y in (x + 1)..ids.len() {
                    let rest = (nodes[x].0 - nodes[y].0).length();
                    let spec = BeamSpec {
                        rest,
                        compliance: 1e-9,
                        damping: 0.0,
                        yield_strain: 1e9,
                        break_strain: 1e9,
                    };
                    w.add_beam(ids[x], ids[y], spec);
                }
            }
        }
        Kind::Body => {
            w.make_rigid(&[seg.unwrap()])
                .expect("a part is not degenerate");
        }
        Kind::Pinned => {}
    }
    ids
}

fn joint(w: &mut World, a: &[u32], b: &[u32], min: Real, max: Real) -> u32 {
    w.add_joint([a[0], a[1], a[2], b[0], b[1], b[2]], min, max)
        .expect("joint")
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

/// An arm hanging from a horizontal hinge on the z axis through the origin: axis nodes at
/// `z = ±0.5`, a reference node straight below, and a box of eight 1 kg corners from 0.1 m to
/// 1.1 m below the axis. The frame is three pinned nodes at the same places. Returns
/// `(frame, arm, joint)`.
fn hanging_arm(w: &mut World, arm: Kind) -> (Vec<u32>, Vec<u32>, u32) {
    let (frame, arm) = hanging_parts(w, arm);
    let j = joint(w, &frame, &arm, -PI, PI);
    (frame, arm, j)
}

/// The hanging arm's two parts without the joint: frame nodes 0 to 2, arm nodes 3 to 13.
fn hanging_parts(w: &mut World, arm: Kind) -> (Vec<u32>, Vec<u32>) {
    let axis = [
        Vec3::new(0.0, 0.0, -0.5),
        Vec3::new(0.0, 0.0, 0.5),
        Vec3::new(0.0, -0.6, 0.0),
    ];
    let frame = part(w, Kind::Pinned, &axis.map(|p| (p, 1.0)));
    let mut nodes: Vec<(Vec3, Real)> = axis.iter().map(|&p| (p, 0.1)).collect();
    nodes.extend(corners(Vec3::new(0.0, -0.6, 0.0), 0.15, 0.5, 0.5, 1.0));
    let arm = part(w, arm, &nodes);
    (frame, arm)
}

fn mass_of(w: &World, ids: &[u32]) -> Real {
    ids.iter().map(|&i| 1.0 / w.inv_mass[i as usize]).sum()
}

/// Kinetic plus potential energy of the listed nodes (riders carry their body's motion).
fn energy(w: &World, ids: &[u32]) -> Real {
    ids.iter()
        .map(|&i| {
            let u = i as usize;
            let m = 1.0 / w.inv_mass[u];
            0.5 * m * w.vel[u].length_squared() + m * G * w.pos[u].y
        })
        .sum()
}

fn gap(w: &World, a: u32, b: u32) -> Real {
    (w.pos[a as usize] - w.pos[b as usize]).length()
}

/// 1. A door (one rigid body) on a vertical hinge at its edge, the frame pinned, pushed at its
/// far corner by a force with parts along every axis for one second and then left to swing. It
/// turns about the hinge only. Bounds: each hinge point's gap stays under 1e-6 m, and every door
/// node's height (the coordinate along the axis) stays within 1e-6 m of where it started, over
/// 5 s. The door's weight hangs off the hinge (a moment the two points carry), so this is the
/// hinge under a real off-axis load, not a free spin.
#[test]
fn door_swings_about_its_axis_only() {
    let mut w = World::new();
    let axis = [
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 2.0, 0.0),
        Vec3::new(-1.0, 1.0, 0.0),
    ];
    let frame = part(&mut w, Kind::Pinned, &axis.map(|p| (p, 1.0)));
    let mut nodes = vec![
        (axis[0], 0.5),
        (axis[1], 0.5),
        (Vec3::new(1.0, 1.0, 0.0), 0.5),
    ];
    nodes.extend(corners(Vec3::new(0.5, 1.0, 0.0), 0.5, 1.0, 0.03, 2.0));
    let door = part(&mut w, Kind::Body, &nodes);
    joint(&mut w, &frame, &door, -PI, PI);
    let y0: Vec<Real> = door.iter().map(|&i| w.pos[i as usize].y).collect();
    let far = door[2];
    let (mut worst_gap, mut worst_height): (Real, Real) = (0.0, 0.0);
    let mut turned: Real = 0.0;
    let mut triggered = false;
    for s in 0..300 {
        if s < 60 {
            w.add_force(far, Vec3::new(3.0, -20.0, 15.0));
        }
        step(&mut w);
        worst_gap = worst_gap
            .max(gap(&w, frame[0], door[0]))
            .max(gap(&w, frame[1], door[1]));
        for (k, &i) in door.iter().enumerate() {
            worst_height = worst_height.max((w.pos[i as usize].y - y0[k]).abs());
        }
        turned = turned.max(w.joint_angle[0].abs());
        triggered |= w.seg_triggered[0];
    }
    println!("door: turned {turned:.4} rad, worst hinge gap {worst_gap:.3e} m, worst height drift {worst_height:.3e} m");
    assert!(turned > 1.0, "the push turns the door ({turned})");
    assert!(worst_gap < 1e-6, "hinge gap {worst_gap}");
    assert!(worst_height < 1e-6, "off-axis drift {worst_height}");
    assert!(triggered, "a moving joint marks the door's segment");
}

/// 2. A limit holds under load and releases when the load reverses. The hanging arm (one body)
/// with an upper limit of 0.3 rad, pushed sideways by 50 N at its reference node (30 N·m at the
/// start, twice gravity's 14 N·m at 0.3 rad). Bounds: from 2 s to 4 s the angle stays within 2e-5
/// of 0.3 (the joint's passes leave a body side a few micro-radians past: ER 005, phase 5.5); one second after the push reverses it is below 0.2 and the limit pushes no more.
#[test]
fn limit_holds_under_load_and_releases() {
    let mut w = World::new();
    let (_, arm, j) = hanging_arm(&mut w, Kind::Body);
    w.joint_max[j as usize] = 0.3;
    let (mut lo, mut hi) = (Real::INFINITY, Real::NEG_INFINITY);
    for s in 0..300 {
        w.add_force(
            arm[2],
            Vec3::new(if s < 240 { 50.0 } else { -50.0 }, 0.0, 0.0),
        );
        step(&mut w);
        if (120..240).contains(&s) {
            lo = lo.min(w.joint_angle[0]);
            hi = hi.max(w.joint_angle[0]);
        }
    }
    let after = w.joint_angle[0];
    println!("limit: held in [{lo:.9}, {hi:.9}], one second after reversing {after:.4}");
    assert!(
        (lo - 0.3).abs() < 2e-5 && (hi - 0.3).abs() < 2e-5,
        "held in [{lo}, {hi}]"
    );
    assert!(after < 0.2, "released: {after}");
}

/// Runs the hanging arm with a servo and returns `(angle per step, worst peak/cap)`.
fn servo_run(
    arm: Kind,
    target: Real,
    rate: Real,
    cap: Real,
    steps: u32,
) -> (Vec<Real>, Real, World, Vec<u32>) {
    let mut w = World::new();
    let (_, ids, j) = hanging_arm(&mut w, arm);
    assert!(w.set_joint_target(j, JOINT_SERVO, target, rate, cap));
    let mut angles = Vec::new();
    let mut worst: Real = 0.0;
    for _ in 0..steps {
        step(&mut w);
        angles.push(w.joint_angle[0]);
        worst = worst.max(w.joint_peak[0] / cap);
    }
    (angles, worst, w, ids)
}

/// 3. A servo reaches its target at its rate and holds it. The hanging arm (one body) lifted to
/// 1 rad at 0.5 rad/s with a cap (200 N·m) well above gravity's most (48 N·m). Bounds: the angle
/// is within 2e-2 rad of `0.5 t` until it gets there (it moves at its rate, never faster, lagging
/// it by under 2% on a body side), then within 1e-4 of 1 rad from 2.5 s to 6 s; the drive's torque never exceeds the cap.
#[test]
fn servo_reaches_its_target_at_its_rate_and_holds() {
    let (angles, worst, _, _) = servo_run(Kind::Body, 1.0, 0.5, 200.0, 360);
    let mut track: Real = 0.0;
    let mut hold: Real = 0.0;
    for (s, &a) in angles.iter().enumerate() {
        let t = (s + 1) as Real * STEP_SECONDS;
        if t < 1.95 {
            track = track.max((a - 0.5 * t).abs());
        }
        if t >= 2.5 {
            hold = hold.max((a - 1.0).abs());
        }
    }
    println!("servo: worst tracking error {track:.3e} rad, worst hold error {hold:.3e} rad, peak/cap {worst:.6}");
    assert!(track < 2e-2, "tracking {track}");
    assert!(
        angles
            .iter()
            .enumerate()
            .all(|(s, &a)| a <= 0.5 * (s + 1) as Real * STEP_SECONDS + 1e-9),
        "never faster"
    );
    assert!(hold < 1e-4, "hold {hold}");
    assert!(worst <= 1.0 + CAP_SLACK, "peak/cap {worst}");
}

/// 4. A servo stalled for 10 simulated minutes against a load above its cap gains no energy.
/// The hanging arm (one body) asked to lift to π/2 (horizontal, where gravity needs `M g d`) with
/// a cap of half that: it rises to where gravity's torque meets the cap (30°) and stalls there.
/// With the drive giving at most its cap, `E = KE + PE − τ_cap · φ` is what a constant torque at
/// the cap conserves, and the most the drive could have added. Bounds: over minutes 1 to 10 the
/// arm stays below 0.6 rad (it never gets near the target), `E` never rises above its value at
/// one minute by more than 1e-6 J, and the drive's torque never exceeds its cap.
#[test]
fn stalled_servo_gains_no_energy() {
    let mut w = World::new();
    let (_, arm, j) = hanging_arm(&mut w, Kind::Body);
    let m = mass_of(&w, &arm);
    let c = w.body_centre[w.node_body[arm[0] as usize] as usize];
    let d = -c.y;
    let cap = 0.5 * m * G * d;
    assert!(w.set_joint_target(j, JOINT_SERVO, 0.5 * PI, 0.2, cap));
    let e = |w: &World| energy(w, &arm) - cap * w.joint_angle[0];
    let (mut e0, mut e_max, mut e_min) = (0.0, Real::NEG_INFINITY, Real::INFINITY);
    let (mut worst, mut top): (Real, Real) = (0.0, 0.0);
    let (mut mech_lo, mut mech_hi) = (Real::INFINITY, Real::NEG_INFINITY);
    for s in 0..36_000u32 {
        step(&mut w);
        worst = worst.max(w.joint_peak[0] / cap);
        if s + 1 == 3600 {
            e0 = e(&w);
        }
        if s + 1 >= 3600 {
            let v = e(&w);
            e_max = e_max.max(v);
            e_min = e_min.min(v);
            top = top.max(w.joint_angle[0]);
            let mech = energy(&w, &arm);
            mech_lo = mech_lo.min(mech);
            mech_hi = mech_hi.max(mech);
        }
    }
    let end = w.joint_angle[0];
    println!(
        "stall: M {m:.2} kg, d {d:.4} m, cap {cap:.3} N·m; angle at 10 min {end:.6} (asin 0.5 = {:.6}), highest {top:.6}; \
         E at 1 min {e0:.9} J, E range over minutes 1-10 [{:+.3e}, {:+.3e}] J; KE+PE range {:.3e} J; peak/cap {worst:.9}",
        PI / 6.0,
        e_min - e0,
        e_max - e0,
        mech_hi - mech_lo
    );
    assert!(top < 0.6, "stalls short of the target: {top}");
    assert!(e_max - e0 < 1e-6, "energy gained {}", e_max - e0);
    assert!(worst <= 1.0 + CAP_SLACK, "peak/cap {worst}");
}

/// A wheel (one rigid body) on a vertical hinge, the frame pinned. Returns `(frame, wheel, joint)`.
fn wheel(w: &mut World) -> (Vec<u32>, Vec<u32>, u32) {
    let axis = [
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.2, 0.0),
        Vec3::new(1.0, 0.1, 0.0),
    ];
    let frame = part(w, Kind::Pinned, &axis.map(|p| (p, 1.0)));
    let mut nodes = vec![
        (axis[0], 0.1),
        (axis[1], 0.1),
        (Vec3::new(0.5, 0.1, 0.0), 0.1),
    ];
    nodes.extend(corners(Vec3::new(0.0, 0.1, 0.0), 0.5, 0.05, 0.5, 1.0));
    let ids = part(w, Kind::Body, &nodes);
    let j = joint(w, &frame, &ids, -PI, PI);
    (frame, ids, j)
}

/// A torque about +y of `tau` (a single force at the reference node, tangent to its circle).
fn brake(w: &mut World, node: u32, tau: Real) {
    let p = w.pos[node as usize];
    let r = Vec3::new(p.x, 0.0, p.z);
    let len = r.length();
    let t = Vec3::new(0.0, 1.0, 0.0).cross(r) / len;
    w.add_force(node, t * (tau / len));
}

/// 5. A rotor holds its speed under a load below its cap and slows under one above it. The
/// wheel spun to 2 rad/s with a 5 N·m cap; from 4 s a 3 N·m brake, from 8 s an 8 N·m brake.
/// Bounds: from 5 s to 8 s the speed is within 1e-3 rad/s of 2; from 9 s to 12 s it falls at
/// `(8 − 5) / I` within 5%; the drive's torque never exceeds the cap.
#[test]
fn rotor_holds_speed_below_its_cap_and_slows_above() {
    let mut w = World::new();
    let (_, ids, j) = wheel(&mut w);
    let slot = w.node_body[ids[0] as usize] as usize;
    let inertia_y = {
        let c = w.body_centre[slot];
        ids.iter()
            .map(|&i| {
                let r = w.pos[i as usize] - c;
                (r.x * r.x + r.z * r.z) / w.inv_mass[i as usize]
            })
            .sum::<Real>()
    };
    let cap = 5.0;
    assert!(w.set_joint_target(j, JOINT_ROTOR, 2.0, 0.0, cap));
    let mut prev = w.joint_angle[0];
    let mut hold: Real = 0.0;
    let (mut at9, mut at12) = (0.0, 0.0);
    let mut worst: Real = 0.0;
    for s in 0..720u32 {
        if s >= 480 {
            brake(&mut w, ids[2], -8.0);
        } else if s >= 240 {
            brake(&mut w, ids[2], -3.0);
        }
        step(&mut w);
        let speed = wrap(w.joint_angle[0] - prev) / STEP_SECONDS;
        prev = w.joint_angle[0];
        worst = worst.max(w.joint_peak[0] / cap);
        if (300..480).contains(&s) {
            hold = hold.max((speed - 2.0).abs());
        }
        if s + 1 == 540 {
            at9 = speed;
        }
        if s + 1 == 720 {
            at12 = speed;
        }
    }
    let decel = (at9 - at12) / 3.0;
    let expect = (8.0 - cap) / inertia_y;
    println!(
        "rotor: I {inertia_y:.4} kg·m², speed error under 3 N·m {hold:.3e} rad/s; under 8 N·m {at9:.4} -> {at12:.4} rad/s, \
         slowing {decel:.4} rad/s² (expected {expect:.4}); peak/cap {worst:.9}"
    );
    assert!(hold < 1e-3, "hold {hold}");
    assert!(
        (decel - expect).abs() < 0.05 * expect,
        "slowing {decel}, expected {expect}"
    );
    assert!(worst <= 1.0 + CAP_SLACK, "peak/cap {worst}");
}

/// Two boxes side by side, hinged on their shared edge (the z axis), falling freely; the servo
/// opens B to 0.5 rad at 1 rad/s. Returns the world, the joint's angle per step and each part's ids.
fn pair(a: Kind, b: Kind) -> (World, Vec<Real>, Vec<u32>, Vec<u32>) {
    let mut w = World::new();
    let side = |x: Real| {
        let mut ns = vec![
            (Vec3::new(0.0, 0.0, -0.5), 1.0),
            (Vec3::new(0.0, 0.0, 0.5), 1.0),
            (Vec3::new(x, 0.0, -0.5), 1.0),
        ];
        ns.extend(
            [
                (x, 0.0, 0.5),
                (0.0, 0.3, -0.5),
                (0.0, 0.3, 0.5),
                (x, 0.3, -0.5),
                (x, 0.3, 0.5),
            ]
            .map(|(px, py, pz)| (Vec3::new(px, py + 0.0, pz), 1.0)),
        );
        ns.iter()
            .map(|&(p, m)| (p + Vec3::new(0.0, 100.0, 0.0), m))
            .collect::<Vec<_>>()
    };
    let ia = part(&mut w, a, &side(-1.0));
    let ib = part(&mut w, b, &side(1.0));
    let j = joint(&mut w, &ia, &ib, -PI, PI);
    assert!(w.set_joint_target(j, JOINT_SERVO, 0.5, 1.0, 50.0));
    let mut angles = Vec::new();
    for _ in 0..120 {
        step(&mut w);
        angles.push(w.joint_angle[0]);
    }
    (w, angles, ia, ib)
}

/// 6. Joints work body–body, body–live and live–live (and the drive's torque never exceeds the
/// cap in any of them). Bounds: at 0.25 s the angle is within 2e-2 rad of 0.25 (on its way at its
/// rate, a body side lagging under 5%) and from 1 s on within 5e-3 of 0.5 (undamped live lattices ring a little under the drive); the
/// hinge points stay within 1e-3 m; both segments are triggered while it moves and neither once
/// it holds.
#[test]
fn joints_work_body_body_body_live_and_live_live() {
    for (a, b) in [
        (Kind::Body, Kind::Body),
        (Kind::Body, Kind::Live),
        (Kind::Live, Kind::Body),
        (Kind::Live, Kind::Live),
    ] {
        let (w, angles, ia, ib) = pair(a, b);
        let quarter = angles[14];
        let held = angles[60..]
            .iter()
            .map(|&x| (x - 0.5).abs())
            .fold(0.0, Real::max);
        let gaps = gap(&w, ia[0], ib[0]).max(gap(&w, ia[1], ib[1]));
        println!("{a:?}-{b:?}: angle at 0.25 s {quarter:.5}, worst hold error from 1 s {held:.3e}, final hinge gap {gaps:.3e} m");
        assert!(
            (quarter - 0.25).abs() < 2e-2,
            "{a:?}-{b:?} at 0.25 s: {quarter}"
        );
        assert!(held < 5e-3, "{a:?}-{b:?} hold {held}");
        assert!(gaps < 1e-3, "{a:?}-{b:?} gap {gaps}");
        assert!(w.joint_peak[0] <= 50.0 * (1.0 + CAP_SLACK));
        assert!(
            !w.seg_triggered[0] && !w.seg_triggered[1],
            "{a:?}-{b:?}: still triggered once held"
        );
    }
    // Triggered while moving: one step in.
    let mut w = World::new();
    let ia = part(
        &mut w,
        Kind::Body,
        &corners(Vec3::new(-0.5, 0.15, 0.0), 0.5, 0.15, 0.5, 1.0),
    );
    let ib = part(
        &mut w,
        Kind::Live,
        &corners(Vec3::new(0.5, 0.15, 0.0), 0.5, 0.15, 0.5, 1.0),
    );
    // Corners k = 1 and 5 of A are (0, 0, ∓0.5); k = 0 and 4 of B the same; refs A k=0, B k=1.
    let j = w
        .add_joint([ia[1], ia[5], ia[0], ib[0], ib[4], ib[1]], -PI, PI)
        .expect("joint");
    assert!(w.set_joint_target(j, JOINT_SERVO, 0.5, 1.0, 50.0));
    step(&mut w);
    assert!(
        w.seg_triggered[0] && w.seg_triggered[1],
        "both segments triggered while moving"
    );
}

/// The servo's arm as a live lattice too: the same lift, within 1e-2 rad while tracking.
#[test]
fn servo_drives_a_live_arm() {
    let (angles, worst, _, _) = servo_run(Kind::Live, 1.0, 0.5, 200.0, 360);
    let track = angles[..110]
        .iter()
        .enumerate()
        .map(|(s, &a)| (a - 0.5 * (s + 1) as Real * STEP_SECONDS).abs())
        .fold(0.0, Real::max);
    let hold = angles[150..]
        .iter()
        .map(|&a| (a - 1.0).abs())
        .fold(0.0, Real::max);
    println!("live servo: tracking {track:.3e} rad, hold {hold:.3e} rad, peak/cap {worst:.6}");
    assert!(track < 1e-2 && hold < 1e-3, "tracking {track}, hold {hold}");
    assert!(worst <= 1.0 + CAP_SLACK);
}

/// A drive inside a free assembly is a couple: it keeps the pair's momentum (it falls as one).
#[test]
fn drive_keeps_free_momentum() {
    let (w, _, ia, ib) = pair(Kind::Body, Kind::Body);
    let all: Vec<u32> = ia.iter().chain(ib.iter()).copied().collect();
    let m = mass_of(&w, &all);
    let p = all.iter().fold(Vec3::ZERO, |acc, &i| {
        acc + w.vel[i as usize] * (1.0 / w.inv_mass[i as usize])
    });
    let expect = -G * m * 120.0 * STEP_SECONDS;
    let err = (p - Vec3::new(0.0, expect, 0.0)).length() / (G * m * 2.0);
    println!("free pair: momentum error {err:.3e} of the fall's");
    assert!(err < 1e-9, "momentum error {err}");
}

/// Same commands, same hash: two runs of the body–live pair hash alike.
#[test]
fn joints_are_deterministic() {
    let (a, _, _, _) = pair(Kind::Body, Kind::Live);
    let (b, _, _, _) = pair(Kind::Body, Kind::Live);
    assert_eq!(state_hash(&a), state_hash(&b));
}

fn rec(kind: u32, a: u32, b: u32, f: [Real; 6]) -> Command {
    Command {
        kind,
        a,
        b,
        reserved: 0,
        f,
    }
}

/// The commands: a joint through `CMD_ADD_JOINT`, its drive through `CMD_SET_JOINT_TARGET`, and
/// every malformed record rejected with nothing changed.
#[test]
fn joint_commands_accept_and_reject() {
    let mut w = World::new();
    hanging_parts(&mut w, Kind::Body);
    let before = state_hash(&w);
    let mut r = Vec::new();
    let f = |a1: Real, b1: Real, ra: Real, rb: Real, lo: Real, hi: Real| [a1, b1, ra, rb, lo, hi];
    let bad = [
        rec(CMD_ADD_JOINT, 0, 3, f(1.0, 4.0, 2.0, 5.0, 0.1, 0.0)), // min above zero
        rec(CMD_ADD_JOINT, 0, 3, f(1.0, 4.0, 2.0, 5.0, 0.0, 4.0)), // max above π
        rec(CMD_ADD_JOINT, 0, 3, f(1.0, 4.0, 2.5, 5.0, -1.0, 1.0)), // not a whole node number
        rec(CMD_ADD_JOINT, 0, 3, f(1.0, 4.0, -1.0, 5.0, -1.0, 1.0)), // negative node
        rec(CMD_ADD_JOINT, 0, 3, f(1.0, 4.0, Real::NAN, 5.0, -1.0, 1.0)),
        rec(CMD_ADD_JOINT, 0, 3, f(1.0, 4.0, 2.0, 999.0, -1.0, 1.0)), // no such node
        rec(CMD_ADD_JOINT, 0, 3, f(0.0, 4.0, 2.0, 5.0, -1.0, 1.0)),   // a0 twice
        rec(CMD_ADD_JOINT, 3, 6, f(4.0, 7.0, 5.0, 8.0, -1.0, 1.0)),   // all six on one body
        rec(CMD_SET_JOINT_TARGET, 0, 1, [0.0; 6]),                    // no joint yet
    ];
    apply_commands(&mut w, &bad[..8], &mut r);
    assert!(r.iter().all(|&x| x == REJECTED), "{r:?}");
    apply_commands(&mut w, &bad[8..], &mut r);
    assert!(r.iter().all(|&x| x == REJECTED), "{r:?}");
    assert_eq!(w.joint_count(), 0);
    assert_eq!(state_hash(&w), before, "rejected records change nothing");
    // A reference node on the axis line has no arm, so no angle: rejected.
    let mut w2 = World::new();
    let p = [
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 2.0, 0.0),
    ];
    let fr = part(&mut w2, Kind::Pinned, &p.map(|x| (x, 1.0)));
    let mv = part(
        &mut w2,
        Kind::Live,
        &[(p[0], 1.0), (p[1], 1.0), (Vec3::new(1.0, 0.0, 0.0), 1.0)],
    );
    assert_eq!(
        w2.add_joint([fr[0], fr[1], fr[2], mv[0], mv[1], mv[2]], -1.0, 1.0),
        None,
        "a reference on the axis"
    );
    // The good records.
    let good = [
        rec(CMD_ADD_JOINT, 0, 3, f(1.0, 4.0, 2.0, 5.0, -1.0, 1.0)),
        rec(
            CMD_SET_JOINT_TARGET,
            0,
            JOINT_SERVO,
            [0.5, 1.0, 10.0, 0.0, 0.0, 0.0],
        ),
        rec(
            CMD_SET_JOINT_TARGET,
            0,
            JOINT_ROTOR,
            [-7.0, 0.0, 10.0, 0.0, 0.0, 0.0],
        ),
    ];
    apply_commands(&mut w, &good, &mut r);
    assert_eq!(r, vec![0, 1, 1]);
    assert_ne!(state_hash(&w), before, "a joint enters the hash");
    let wrong = [
        rec(
            CMD_SET_JOINT_TARGET,
            1,
            JOINT_SERVO,
            [0.5, 1.0, 10.0, 0.0, 0.0, 0.0],
        ), // no joint 1
        rec(CMD_SET_JOINT_TARGET, 0, 3, [0.5, 1.0, 10.0, 0.0, 0.0, 0.0]), // no mode 3
        rec(
            CMD_SET_JOINT_TARGET,
            0,
            JOINT_SERVO,
            [4.0, 1.0, 10.0, 0.0, 0.0, 0.0],
        ), // servo past π
        rec(
            CMD_SET_JOINT_TARGET,
            0,
            JOINT_SERVO,
            [0.5, -1.0, 10.0, 0.0, 0.0, 0.0],
        ), // negative rate
        rec(
            CMD_SET_JOINT_TARGET,
            0,
            JOINT_ROTOR,
            [1.0, 0.0, -1.0, 0.0, 0.0, 0.0],
        ), // negative cap
        rec(
            CMD_SET_JOINT_TARGET,
            0,
            JOINT_ROTOR,
            [Real::INFINITY, 0.0, 1.0, 0.0, 0.0, 0.0],
        ),
    ];
    let mid = state_hash(&w);
    apply_commands(&mut w, &wrong, &mut r);
    assert!(r.iter().all(|&x| x == REJECTED), "{r:?}");
    assert_eq!(state_hash(&w), mid);
}

/// A joint whose node is deleted does nothing, even after the slot is reused.
#[test]
fn a_joint_on_a_dead_node_does_nothing() {
    let mut w = World::new();
    let (_, arm, j) = hanging_arm(&mut w, Kind::Live);
    assert!(w.set_joint_target(j, JOINT_SERVO, 1.0, 1.0, 100.0));
    for _ in 0..30 {
        step(&mut w);
    }
    assert!(w.joint_angle[0] > 0.4);
    w.delete_node(arm[2]);
    let reused = w.add_node(Vec3::new(5.0, 5.0, 5.0), 1.0, 0.1).index;
    assert_eq!(reused, arm[2]);
    assert!(!w.joint_nodes_live(0));
    let held = w.joint_angle[0];
    for _ in 0..30 {
        step(&mut w);
    }
    assert_eq!(w.joint_angle[0], held, "a dead joint is not solved");
}
