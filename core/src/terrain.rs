//! Terrain: a grid of heights, `y = h(x, z)`, split into fixed triangles (plan 003, phase 3.1).
//!
//! The grid has `nx` by `nz` points, `cell` metres apart, from the corner `(x0, z0)`. Point
//! `(i, j)` sits at `x = x0 + i * cell`, `z = z0 + j * cell`, and its height is `heights[i + j * nx]`.
//! Every cell is two flat triangles, split along the diagonal from `(i, j)` to `(i + 1, j + 1)`:
//! the one with `u >= v` (where `u`, `v` are the position in the cell, 0 to 1) is
//! `(i, j) (i+1, j) (i+1, j+1)`, the other is `(i, j) (i+1, j+1) (i, j+1)`. Each triangle has an
//! exact plane, so the surface is continuous, the contact code can hold a plane for a whole step,
//! and the renderer draws the very same triangles.
//!
//! Outside the grid the edge heights continue flat: a point past an edge reads the height of the
//! nearest edge point, and its plane has no slope in the direction it is past. A node that leaves
//! the map therefore still has ground.
//!
//! Only `+ - * /`, `sqrt` and `floor` (all exactly specified by IEEE 754), so the same bits come
//! out natively and in WebAssembly. A non-finite coordinate reads as the grid's first corner (the
//! comparisons below are false for NaN); callers that care reject such positions earlier.
//!
//! The terrain is read-only data in [`crate::world::World`]; it changes only through commands
//! ([`crate::command`]), so a recording holds its terrain.

use crate::real::{Real, Vec3};

/// Most grid points along one side. **[assumption]** 257 gives a 256-cell side (a power of two
/// plus one, the usual height-map size) and bounds one terrain at 66,049 heights, 528 KB.
pub const MAX_GRID: u32 = 257;

/// A plane with an upward unit normal: the signed distance of `p` from it is
/// `normal . p - offset`, positive above the ground.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Plane {
    pub normal: Vec3,
    pub offset: Real,
}

impl Plane {
    /// Signed distance of `p` above the plane.
    pub fn distance(&self, p: Vec3) -> Real {
        self.normal.dot(p) - self.offset
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Terrain {
    pub nx: u32,
    pub nz: u32,
    pub x0: Real,
    pub z0: Real,
    pub cell: Real,
    /// One Coulomb coefficient for the whole terrain (static equals kinetic).
    pub friction: Real,
    /// `nx * nz` heights, row after row along `x` (index `i + j * nx`).
    pub heights: Vec<Real>,
}

/// One axis of a lookup: the cell index, the position in the cell (0 to 1) and whether the
/// point was past the grid on this axis.
struct Axis {
    cell: usize,
    frac: Real,
    outside: bool,
}

fn axis(coord: Real, origin: Real, cell: Real, points: u32) -> Axis {
    let last = (points - 1) as Real;
    let mut t = (coord - origin) / cell;
    let mut outside = false;
    // The negated form is on purpose: it is also true for NaN, which reads as the first point.
    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    if !(t >= 0.0) {
        // Before the first point, or not a number.
        outside = t < 0.0;
        t = 0.0;
    } else if t > last {
        outside = true;
        t = last;
    }
    let mut i = t.floor();
    // The last point belongs to the last cell, at its far edge.
    if i > last - 1.0 {
        i = last - 1.0;
    }
    Axis { cell: i as usize, frac: t - i, outside }
}

impl Terrain {
    /// A flat terrain at height zero. The caller has checked the sizes (`2..=MAX_GRID` each).
    pub fn flat(nx: u32, nz: u32, x0: Real, z0: Real, cell: Real, friction: Real) -> Terrain {
        Terrain { nx, nz, x0, z0, cell, friction, heights: vec![0.0; nx as usize * nz as usize] }
    }

    pub fn height_count(&self) -> usize {
        self.heights.len()
    }

    fn h(&self, i: usize, j: usize) -> Real {
        self.heights[i + j * self.nx as usize]
    }

    /// Height and the two slopes (`dh/dx`, `dh/dz`) of the triangle under `(x, z)`, with the
    /// slope zero along an axis the point is past.
    fn locate(&self, x: Real, z: Real) -> (Real, Real, Real) {
        let ax = axis(x, self.x0, self.cell, self.nx);
        let az = axis(z, self.z0, self.cell, self.nz);
        let (i, j) = (ax.cell, az.cell);
        let (h00, h10, h01, h11) = (self.h(i, j), self.h(i + 1, j), self.h(i, j + 1), self.h(i + 1, j + 1));
        // The two edge differences of the triangle, along x and along z.
        let (dx, dz) = if ax.frac >= az.frac { (h10 - h00, h11 - h10) } else { (h11 - h01, h01 - h00) };
        let h = h00 + ax.frac * dx + az.frac * dz;
        let sx = if ax.outside { 0.0 } else { dx / self.cell };
        let sz = if az.outside { 0.0 } else { dz / self.cell };
        (h, sx, sz)
    }

    /// Ground height at `(x, z)`.
    pub fn height_at(&self, x: Real, z: Real) -> Real {
        self.locate(x, z).0
    }

    /// The plane of the triangle under `(x, z)` (the surface itself, extended flat past the edges).
    pub fn plane_at(&self, x: Real, z: Real) -> Plane {
        let (h, sx, sz) = self.locate(x, z);
        let len = (1.0 + sx * sx + sz * sz).sqrt();
        let normal = Vec3::new(-sx / len, 1.0 / len, -sz / len);
        Plane { normal, offset: normal.dot(Vec3::new(x, h, z)) }
    }
}
