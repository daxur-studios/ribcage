//! Phase 3.1: the height-grid terrain and its commands.
//!
//! Expected values come from the rules in `core/src/terrain.rs` (fixed diagonal, flat edges, exact
//! plane per triangle), worked out by hand, never from measured output. Heights on a grid whose
//! numbers are exact in binary are compared bit for bit; normals go through one square root, so
//! they are compared within 1e-14.

use beam_core::command::{apply_commands, Command, CMD_SET_TERRAIN, CMD_TERRAIN_HEIGHTS, REJECTED};
use beam_core::real::{Real, Vec3};
use beam_core::sim::state_hash;
use beam_core::terrain::{Terrain, MAX_GRID};
use beam_core::world::World;

fn rec(kind: u32, a: u32, b: u32, f: [Real; 6]) -> Command {
    Command { kind, a, b, reserved: 0, f }
}

fn set(nx: u32, nz: u32, x0: Real, z0: Real, cell: Real, friction: Real) -> Command {
    rec(CMD_SET_TERRAIN, nx, nz, [x0, z0, cell, friction, 0.0, 0.0])
}

fn heights(start: u32, h: &[Real]) -> Command {
    let mut f = [0.0; 6];
    f[..h.len()].copy_from_slice(h);
    rec(CMD_TERRAIN_HEIGHTS, start, h.len() as u32, f)
}

/// Applies a list and returns the results.
fn run(w: &mut World, cmds: &[Command]) -> Vec<u32> {
    let mut results = Vec::new();
    apply_commands(w, cmds, &mut results);
    results
}

/// A terrain with the given heights, written through the same commands the host uses.
fn grid(nx: u32, nz: u32, x0: Real, z0: Real, cell: Real, h: impl Fn(u32, u32) -> Real) -> World {
    let mut w = World::new();
    let mut cmds = vec![set(nx, nz, x0, z0, cell, 0.8)];
    let all: Vec<Real> = (0..nz).flat_map(|j| (0..nx).map(move |i| (i, j))).map(|(i, j)| h(i, j)).collect();
    for (k, chunk) in all.chunks(6).enumerate() {
        cmds.push(heights(k as u32 * 6, chunk));
    }
    let r = run(&mut w, &cmds);
    assert!(r.iter().all(|&x| x != REJECTED), "{r:?}");
    w
}

fn near(a: Vec3, b: Vec3) -> bool {
    (a.x - b.x).abs() < 1e-14 && (a.y - b.y).abs() < 1e-14 && (a.z - b.z).abs() < 1e-14
}

/// A tilted plane `y = 0.25 x + 0.125 z + 3`, sampled at points 2 m apart from `(-4, 1)`: heights
/// are exact everywhere inside, at grid points and between them, and the normal is the plane's.
#[test]
fn heights_and_normals_are_exact_on_a_tilted_plane() {
    let (x0, z0, cell) = (-4.0, 1.0, 2.0);
    let plane = |x: Real, z: Real| 0.25 * x + 0.125 * z + 3.0;
    let w = grid(5, 4, x0, z0, cell, |i, j| plane(x0 + cell * i as Real, z0 + cell * j as Real));
    let t = w.terrain.as_ref().unwrap();
    assert_eq!((t.nx, t.nz, t.cell, t.friction), (5, 4, 2.0, 0.8));

    // Grid points, cell middles, points either side of the diagonal, the far edge.
    for &(x, z) in &[(-4.0, 1.0), (0.0, 3.0), (4.0, 7.0), (-3.0, 1.5), (-1.5, 2.0), (-2.5, 5.5), (3.75, 6.5), (4.0, 1.0), (-4.0, 7.0)] {
        assert_eq!(t.height_at(x, z), plane(x, z), "height at ({x}, {z})");
    }

    // The unit normal of that plane, at points in both triangles of different cells.
    let len = (1.0f64 + 0.25 * 0.25 + 0.125 * 0.125).sqrt();
    let want = Vec3::new(-0.25 / len, 1.0 / len, -0.125 / len);
    for &(x, z) in &[(-3.5, 1.25), (-3.25, 1.75), (0.5, 4.0), (1.75, 3.0), (3.9, 6.9)] {
        let p = t.plane_at(x, z);
        assert!(near(p.normal, want), "normal at ({x}, {z}): {:?}", p.normal);
        // The surface point is on the plane, a point one metre straight up is 1 / len above it.
        assert!(p.distance(Vec3::new(x, plane(x, z), z)).abs() < 1e-13);
        assert!((p.distance(Vec3::new(x, plane(x, z) + 1.0, z)) - 1.0 / len).abs() < 1e-13);
    }
}

/// One cell with heights that are not on one plane: `h00 = 0`, `h10 = 2`, `h01 = 4`, `h11 = 10`
/// (cell 1 m). The documented diagonal runs `(0,0)` to `(1,1)`; the other diagonal would give
/// other numbers at these points.
#[test]
fn the_diagonal_split_is_the_documented_one() {
    let table = [0.0, 2.0, 4.0, 10.0]; // h00, h10, h01, h11 = index i + 2 j
    let w = grid(2, 2, 0.0, 0.0, 1.0, |i, j| table[(i + 2 * j) as usize]);
    let t = w.terrain.as_ref().unwrap();

    // Corners exactly.
    assert_eq!(t.height_at(0.0, 0.0), 0.0);
    assert_eq!(t.height_at(1.0, 0.0), 2.0);
    assert_eq!(t.height_at(0.0, 1.0), 4.0);
    assert_eq!(t.height_at(1.0, 1.0), 10.0);
    // Below the diagonal (x > z): triangle (0,0) (1,0) (1,1): h = 0 + 2 u + 8 v.
    assert_eq!(t.height_at(0.75, 0.25), 3.5);
    // Above it (z > x): triangle (0,0) (1,1) (0,1): h = 6 u + 4 v.
    assert_eq!(t.height_at(0.25, 0.75), 4.5);
    // On the diagonal both triangles agree: the surface is continuous.
    assert_eq!(t.height_at(0.5, 0.5), 5.0);
    assert_eq!(t.height_at(0.125, 0.125), 1.25);

    // The planes: slopes (2, 8) below the diagonal, (6, 4) above it.
    let lo = t.plane_at(0.75, 0.25);
    let l = (1.0f64 + 4.0 + 64.0).sqrt();
    assert!(near(lo.normal, Vec3::new(-2.0 / l, 1.0 / l, -8.0 / l)), "{:?}", lo.normal);
    let hi = t.plane_at(0.25, 0.75);
    let l = (1.0f64 + 36.0 + 16.0).sqrt();
    assert!(near(hi.normal, Vec3::new(-6.0 / l, 1.0 / l, -4.0 / l)), "{:?}", hi.normal);
    // Both planes pass through the shared diagonal.
    assert!(lo.distance(Vec3::new(0.5, 5.0, 0.5)).abs() < 1e-14);
    assert!(hi.distance(Vec3::new(0.5, 5.0, 0.5)).abs() < 1e-14);
}

/// Past an edge the edge heights continue flat: height of the nearest edge point, no slope along
/// the axis the point is past, the other slope kept. Far away and non-finite points still answer.
#[test]
fn outside_the_grid_the_edge_heights_continue_flat() {
    let table = [0.0, 2.0, 4.0, 10.0];
    let w = grid(2, 2, 0.0, 0.0, 1.0, |i, j| table[(i + 2 * j) as usize]);
    let t = w.terrain.as_ref().unwrap();

    // Past the +x edge: the edge column x = 1 runs from 2 (z = 0) to 10 (z = 1).
    assert_eq!(t.height_at(5.0, 0.25), 4.0);
    assert_eq!(t.height_at(1e9, 0.5), 6.0);
    // Past the -x edge: the column x = 0 runs from 0 to 4.
    assert_eq!(t.height_at(-3.0, 0.5), 2.0);
    // Past a z edge: the row z = 0 runs 0 to 2, the row z = 1 runs 4 to 10.
    assert_eq!(t.height_at(0.5, -7.0), 1.0);
    assert_eq!(t.height_at(0.5, 8.0), 7.0);
    // Past a corner: that corner's height.
    assert_eq!(t.height_at(-1.0, -1.0), 0.0);
    assert_eq!(t.height_at(9.0, 9.0), 10.0);
    assert_eq!(t.height_at(9.0, -9.0), 2.0);
    assert_eq!(t.height_at(Real::INFINITY, Real::INFINITY), 10.0);
    // Not a number reads as the first corner.
    assert_eq!(t.height_at(Real::NAN, 0.5), t.height_at(0.0, 0.5));

    // Slopes: zero along the axis the point is past, the cell's slope along the other.
    let l = (1.0f64 + 64.0).sqrt();
    let p = t.plane_at(5.0, 0.25); // past +x, lower triangle clamps to u = 1: z slope 8
    assert!(near(p.normal, Vec3::new(0.0, 1.0 / l, -8.0 / l)), "{:?}", p.normal);
    assert!(p.distance(Vec3::new(5.0, 4.0, 0.25)).abs() < 1e-14);
    let corner = t.plane_at(9.0, 9.0);
    assert!(near(corner.normal, Vec3::new(0.0, 1.0, 0.0)), "{:?}", corner.normal);
    assert!((corner.distance(Vec3::new(9.0, 12.0, 9.0)) - 2.0).abs() < 1e-14);
}

/// A grid is rejected record by record; the rest of the batch applies and nothing half-written
/// is left behind.
#[test]
fn bad_terrain_records_are_rejected_and_the_rest_applies() {
    let mut w = World::new();
    let nan = Real::NAN;
    let cmds = [
        heights(0, &[1.0]),                                 // 0 rejected: no terrain yet
        set(1, 4, 0.0, 0.0, 1.0, 0.5),                      // 1 rejected: one point wide
        set(4, MAX_GRID + 1, 0.0, 0.0, 1.0, 0.5),           // 2 rejected: too big
        set(4, 4, 0.0, 0.0, 0.0, 0.5),                      // 3 rejected: cell not above zero
        set(4, 4, 0.0, 0.0, 1.0, -0.1),                     // 4 rejected: negative friction
        set(4, 4, nan, 0.0, 1.0, 0.5),                      // 5 rejected: origin not a number
        set(4, 4, 0.0, 0.0, Real::INFINITY, 0.5),           // 6 rejected: cell infinite
        set(4, 3, -1.0, 2.0, 0.5, 0.6),                     // 7 ok: 12 heights
        heights(2, &[1.0, 2.0, 3.0]),                       // 8 ok
        heights(11, &[9.0]),                                // 9 ok: the last height
        heights(11, &[9.0, 9.0]),                           // 10 rejected: past the end
        heights(10, &[1.0, 2.0, 3.0]),                      // 11 rejected: past the end
        heights(4, &[5.0, nan]),                            // 12 rejected: not a number, nothing written
        rec(CMD_TERRAIN_HEIGHTS, 0, 0, [0.0; 6]),          // 13 rejected: zero heights
        rec(CMD_TERRAIN_HEIGHTS, 0, 7, [1.0; 6]),           // 14 rejected: more than six
        rec(CMD_TERRAIN_HEIGHTS, u32::MAX, 3, [1.0; 6]),    // 15 rejected: start far past the end
        heights(0, &[0.5]),                                 // 16 ok: later records still apply
    ];
    let r = run(&mut w, &cmds);
    let ok = |i: usize| r[i] == 1;
    let rejected: Vec<usize> = (0..cmds.len()).filter(|&i| r[i] == REJECTED).collect();
    assert_eq!(rejected, vec![0, 1, 2, 3, 4, 5, 6, 10, 11, 12, 13, 14, 15], "{r:?}");
    assert!(ok(7) && ok(8) && ok(9) && ok(16));

    let t = w.terrain.as_ref().unwrap();
    assert_eq!((t.nx, t.nz, t.x0, t.z0, t.cell, t.friction), (4, 3, -1.0, 2.0, 0.5, 0.6));
    let want = [0.5, 0.0, 1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.0];
    assert_eq!(t.heights, want);
}

/// Setting the terrain again replaces it, heights back to zero.
#[test]
fn a_new_terrain_replaces_the_old_one() {
    let mut w = grid(3, 3, 0.0, 0.0, 1.0, |i, j| (i + j) as Real);
    assert_eq!(w.terrain.as_ref().unwrap().height_at(2.0, 2.0), 4.0);
    run(&mut w, &[set(2, 2, 10.0, 10.0, 3.0, 0.2)]);
    let t = w.terrain.as_ref().unwrap();
    assert_eq!((t.nx, t.nz, t.heights.len()), (2, 2, 4));
    assert!(t.heights.iter().all(|&h| h == 0.0));
    assert_eq!(t.height_at(11.0, 11.0), 0.0);
}

/// The ground is part of the state hash: every field of it, every height, and the world with no
/// ground hashes as before (the golden hashes in `ffi.rs` and the recording test guard that).
#[test]
fn the_terrain_is_in_the_state_hash() {
    let base = || grid(4, 4, 0.0, 0.0, 1.0, |i, j| (i * j) as Real * 0.5);
    let h0 = state_hash(&base());
    assert_eq!(h0, state_hash(&base()), "same commands, same hash");
    assert_ne!(h0, state_hash(&World::new()), "ground changes the hash");

    let mut changed = base();
    changed.terrain.as_mut().unwrap().heights[15] += 0.5; // the last height
    assert_ne!(h0, state_hash(&changed));
    let mut changed = base();
    changed.terrain.as_mut().unwrap().heights[0] += 0.5;
    assert_ne!(h0, state_hash(&changed));
    for edit in [
        |t: &mut Terrain| t.friction += 0.1,
        |t: &mut Terrain| t.cell += 0.5,
        |t: &mut Terrain| t.x0 += 1.0,
        |t: &mut Terrain| t.z0 += 1.0,
    ] {
        let mut changed = base();
        edit(changed.terrain.as_mut().unwrap());
        assert_ne!(h0, state_hash(&changed));
    }
    // Same heights in another shape: 2 by 8 is not 4 by 4.
    let mut shaped = World::new();
    run(&mut shaped, &[set(2, 8, 0.0, 0.0, 1.0, 0.8)]);
    let mut square = World::new();
    run(&mut square, &[set(4, 4, 0.0, 0.0, 1.0, 0.8)]);
    assert_ne!(state_hash(&shaped), state_hash(&square));
}

/// The biggest grid is accepted whole: 257 by 257 points through 11,010 height records.
#[test]
fn the_largest_grid_is_accepted() {
    let n = MAX_GRID;
    let w = grid(n, n, 0.0, 0.0, 1.0, |i, j| ((i + j) % 4) as Real);
    let t = w.terrain.as_ref().unwrap();
    assert_eq!(t.heights.len(), (n * n) as usize);
    assert_eq!(t.height_at(256.0, 256.0), 0.0); // (256 + 256) % 4
    assert_eq!(t.height_at(255.0, 256.0), 3.0); // 511 % 4
}
