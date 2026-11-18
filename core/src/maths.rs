//! The core's own sine and cosine (determinism list: "own sine, cosine and any other
//! transcendental function"). Library `sin` and `cos` differ between platforms and between a
//! native build and WebAssembly; these use only add, subtract, multiply and one cast, so the
//! bits are the same everywhere. This module is the only place in `core/src` allowed to hold
//! such functions; `core/tests/maths.rs` checks that no other file calls a library one.
//!
//! **Method.** Reduce `x` to `r` in `[-π/4, π/4]` and a quadrant `k`, then evaluate the
//! classic fdlibm kernel polynomials (degree 13 for sine, 12 for cosine, Horner form, no
//! fused multiply-add). The reduction is Cody-Waite: `k` is `x · 2/π` rounded to the nearest
//! integer, and `r = x − k · π/2` is taken with `π/2` split into three 33-bit pieces, so
//! each product `k · piece` is exact while `|k| < 2^20`.
//!
//! **Domain and accuracy.** For `|x| ≤ 1e6` radians the absolute error is at most about
//! `1.2e-16` (measured against the platform library, which is itself within one ulp; see
//! `core/tests/maths.rs`). Beyond that, or for a NaN or infinite `x`, the result is NaN, so
//! the bad-number freeze catches a runaway angle instead of returning a wrong one. Keep
//! stored angles wrapped; a joint angle is nowhere near `1e6` rad.

use crate::real::{Real, Vec3};

/// π as a `Real` (a constant, not a function call).
pub const PI: Real = 3.141_592_653_589_793_238_462_643_383_279_5;
/// 2π.
pub const TAU: Real = 2.0 * PI;

/// Largest `|x|` the reduction handles exactly (`k < 2^20` at `|x| · 2/π`, with margin).
const LIMIT: Real = 1.0e6;

const TWO_OVER_PI: Real = 6.366_197_723_675_813_824_33e-1;
// π/2 in three pieces of 33 bits each (fdlibm pio2_1, pio2_2, pio2_3).
const PIO2_1: Real = 1.570_796_326_734_125_614_17;
const PIO2_2: Real = 6.077_100_506_303_965_976_60e-11;
const PIO2_3: Real = 2.022_266_248_711_166_455_80e-21;

// Kernel coefficients (fdlibm k_sin.c and k_cos.c).
const S1: Real = -1.666_666_666_666_663_243_48e-1;
const S2: Real = 8.333_333_333_322_489_461_24e-3;
const S3: Real = -1.984_126_982_985_794_931_34e-4;
const S4: Real = 2.755_731_370_707_006_767_89e-6;
const S5: Real = -2.505_076_025_340_686_341_95e-8;
const S6: Real = 1.589_690_995_211_550_102_21e-10;

const C1: Real = 4.166_666_666_666_660_190_37e-2;
const C2: Real = -1.388_888_888_887_410_957_49e-3;
const C3: Real = 2.480_158_728_947_672_941_78e-5;
const C4: Real = -2.755_731_435_139_066_330_35e-7;
const C5: Real = 2.087_572_321_298_174_827_90e-9;
const C6: Real = -1.135_964_755_778_819_482_65e-11;

/// Sine of `r`, for `|r| ≤ π/4` (a little beyond is fine).
fn sin_kernel(r: Real) -> Real {
    let z = r * r;
    let p = S2 + z * (S3 + z * (S4 + z * (S5 + z * S6)));
    r + (z * r) * (S1 + z * p)
}

/// Cosine of `r`, for `|r| ≤ π/4`. `1 − z/2` is split so the rounding of the big part does not
/// swamp the small correction.
fn cos_kernel(r: Real) -> Real {
    let z = r * r;
    let p = z * (C1 + z * (C2 + z * (C3 + z * (C4 + z * (C5 + z * C6)))));
    let hz = 0.5 * z;
    let w = 1.0 - hz;
    w + (((1.0 - w) - hz) + z * p)
}

/// Reduces `x` to `(k mod 4, r)` with `x = k · π/2 + r` and `|r| ≤ π/4`.
fn reduce(x: Real) -> (i64, Real) {
    let t = x * TWO_OVER_PI;
    // Round to nearest by adding half and truncating toward zero (the cast truncates).
    let k = (t + if t < 0.0 { -0.5 } else { 0.5 }) as i64;
    let kf = k as Real;
    let r = ((x - kf * PIO2_1) - kf * PIO2_2) - kf * PIO2_3;
    (k & 3, r)
}

/// Sine of `x` in radians. NaN for NaN, infinite or `|x| > 1e6`.
pub fn sin(x: Real) -> Real {
    if !(x.abs() <= LIMIT) {
        return Real::NAN;
    }
    if x == 0.0 {
        return x; // keeps the sign of zero, so sine stays odd bit for bit
    }
    let (q, r) = reduce(x);
    match q {
        0 => sin_kernel(r),
        1 => cos_kernel(r),
        2 => -sin_kernel(r),
        _ => -cos_kernel(r),
    }
}

/// Cosine of `x` in radians. NaN for NaN, infinite or `|x| > 1e6`.
pub fn cos(x: Real) -> Real {
    if !(x.abs() <= LIMIT) {
        return Real::NAN;
    }
    let (q, r) = reduce(x);
    match q {
        0 => cos_kernel(r),
        1 => -sin_kernel(r),
        2 => -cos_kernel(r),
        _ => sin_kernel(r),
    }
}

/// `(sin x, cos x)` with one reduction. Bit-identical to calling [`sin`] and [`cos`].
pub fn sin_cos(x: Real) -> (Real, Real) {
    if !(x.abs() <= LIMIT) {
        return (Real::NAN, Real::NAN);
    }
    if x == 0.0 {
        return (x, 1.0);
    }
    let (q, r) = reduce(x);
    let (s, c) = (sin_kernel(r), cos_kernel(r));
    match q {
        0 => (s, c),
        1 => (c, -s),
        2 => (-s, -c),
        _ => (-c, s),
    }
}

// ---------------------------------------------------------------------------------------------
// Rotations and 3 by 3 matrices for rigid bodies (plan 005, phase 5.2). Only add, subtract,
// multiply, divide and square root, each expression in a fixed order, so native and WebAssembly
// give the same bits. No fused multiply-add.
// ---------------------------------------------------------------------------------------------

/// A rotation quaternion `w + xi + yj + zk`. Rigid-body rotations are kept unit length
/// (renormalised after every update); [`Quat::rotate`] assumes it.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quat {
    pub w: Real,
    pub x: Real,
    pub y: Real,
    pub z: Real,
}

impl Default for Quat {
    fn default() -> Quat {
        Quat::IDENTITY
    }
}

impl Quat {
    pub const IDENTITY: Quat = Quat { w: 1.0, x: 0.0, y: 0.0, z: 0.0 };

    pub const fn new(w: Real, x: Real, y: Real, z: Real) -> Quat {
        Quat { w, x, y, z }
    }

    /// The vector part.
    pub fn vec(self) -> Vec3 {
        Vec3::new(self.x, self.y, self.z)
    }

    pub fn conj(self) -> Quat {
        Quat::new(self.w, -self.x, -self.y, -self.z)
    }

    pub fn length_squared(self) -> Real {
        self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z
    }

    /// Scaled to unit length (one square root and four divides).
    pub fn normalize(self) -> Quat {
        let n = self.length_squared().sqrt();
        Quat::new(self.w / n, self.x / n, self.y / n, self.z / n)
    }

    /// The Hamilton product `self ⊗ o` (apply `o` first, then `self`).
    pub fn mul(self, o: Quat) -> Quat {
        Quat::new(
            self.w * o.w - self.x * o.x - self.y * o.y - self.z * o.z,
            self.w * o.x + self.x * o.w + self.y * o.z - self.z * o.y,
            self.w * o.y - self.x * o.z + self.y * o.w + self.z * o.x,
            self.w * o.z + self.x * o.y - self.y * o.x + self.z * o.w,
        )
    }

    /// The vector part of `self ⊗ conj(p)`, the rotation from `p` to `self` (half the angle's
    /// sine times the axis). Written so that `self == p` gives exactly zero in floating point.
    pub fn delta_vec(self, p: Quat) -> Vec3 {
        let (q, pv) = (self.vec(), p.vec());
        let c = q.cross(pv);
        Vec3::new((p.w * q.x - self.w * pv.x) - c.x, (p.w * q.y - self.w * pv.y) - c.y, (p.w * q.z - self.w * pv.z) - c.z)
    }

    /// The scalar part of `self ⊗ conj(p)`.
    pub fn delta_w(self, p: Quat) -> Real {
        self.w * p.w + self.x * p.x + self.y * p.y + self.z * p.z
    }

    /// `v` rotated by this unit quaternion: `v + w t + u × t` with `t = 2 u × v`.
    pub fn rotate(self, v: Vec3) -> Vec3 {
        let u = self.vec();
        let t = u.cross(v) * 2.0;
        v + t * self.w + u.cross(t)
    }

    /// `v` rotated by the inverse of this unit quaternion.
    pub fn inv_rotate(self, v: Vec3) -> Vec3 {
        self.conj().rotate(v)
    }

    /// The Cayley map of a rotation vector `a`: `(1, a/2)` normalised. Its angle is
    /// `2 atan(|a| / 2)`, equal to `|a|` to third order; no sine or cosine needed.
    pub fn cayley(a: Vec3) -> Quat {
        Quat::new(1.0, 0.5 * a.x, 0.5 * a.y, 0.5 * a.z).normalize()
    }

    /// The rotation matrix of this unit quaternion.
    pub fn to_mat3(self) -> Mat3 {
        let (w, x, y, z) = (self.w, self.x, self.y, self.z);
        let (xx, yy, zz) = (x * x, y * y, z * z);
        let (xy, xz, yz, wx, wy, wz) = (x * y, x * z, y * z, w * x, w * y, w * z);
        Mat3 {
            m: [
                [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
                [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
                [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
            ],
        }
    }

    pub fn is_finite(self) -> bool {
        self.w.is_finite() && self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }
}

/// A symmetric 3 by 3 matrix (an inertia tensor or its inverse): the six unique entries, the
/// off-diagonal ones as they stand in the matrix (not negated).
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct Sym3 {
    pub xx: Real,
    pub yy: Real,
    pub zz: Real,
    pub xy: Real,
    pub xz: Real,
    pub yz: Real,
}

impl Sym3 {
    pub const ZERO: Sym3 = Sym3 { xx: 0.0, yy: 0.0, zz: 0.0, xy: 0.0, xz: 0.0, yz: 0.0 };

    pub const fn diag(x: Real, y: Real, z: Real) -> Sym3 {
        Sym3 { xx: x, yy: y, zz: z, xy: 0.0, xz: 0.0, yz: 0.0 }
    }

    pub fn mul_vec(self, v: Vec3) -> Vec3 {
        Vec3::new(
            self.xx * v.x + self.xy * v.y + self.xz * v.z,
            self.xy * v.x + self.yy * v.y + self.yz * v.z,
            self.xz * v.x + self.yz * v.y + self.zz * v.z,
        )
    }

    pub fn add(self, o: Sym3) -> Sym3 {
        Sym3 {
            xx: self.xx + o.xx,
            yy: self.yy + o.yy,
            zz: self.zz + o.zz,
            xy: self.xy + o.xy,
            xz: self.xz + o.xz,
            yz: self.yz + o.yz,
        }
    }

    pub fn trace(self) -> Real {
        self.xx + self.yy + self.zz
    }

    /// The point-mass inertia `m (|r|² 1 − r rᵀ)` about the origin of `r`; also the parallel-axis
    /// term of a body of mass `m` whose centre is at `r`.
    pub fn point(m: Real, r: Vec3) -> Sym3 {
        let (x2, y2, z2) = (r.x * r.x, r.y * r.y, r.z * r.z);
        Sym3 {
            xx: m * (y2 + z2),
            yy: m * (x2 + z2),
            zz: m * (x2 + y2),
            xy: -(m * (r.x * r.y)),
            xz: -(m * (r.x * r.z)),
            yz: -(m * (r.y * r.z)),
        }
    }

    /// The adjugate (symmetric too), entry by entry in the same layout.
    fn adjugate(self) -> Sym3 {
        Sym3 {
            xx: self.yy * self.zz - self.yz * self.yz,
            yy: self.xx * self.zz - self.xz * self.xz,
            zz: self.xx * self.yy - self.xy * self.xy,
            xy: self.xz * self.yz - self.xy * self.zz,
            xz: self.xy * self.yz - self.xz * self.yy,
            yz: self.xy * self.xz - self.xx * self.yz,
        }
    }

    pub fn det(self) -> Real {
        let a = self.adjugate();
        self.xx * a.xx + self.xy * a.xy + self.xz * a.xz
    }

    /// The inverse by the adjugate; `None` when the determinant is zero or not finite.
    pub fn inverse(self) -> Option<Sym3> {
        let a = self.adjugate();
        let det = self.xx * a.xx + self.xy * a.xy + self.xz * a.xz;
        if det == 0.0 || !det.is_finite() {
            return None;
        }
        Some(Sym3 { xx: a.xx / det, yy: a.yy / det, zz: a.zz / det, xy: a.xy / det, xz: a.xz / det, yz: a.yz / det })
    }

    pub fn to_mat3(self) -> Mat3 {
        Mat3 { m: [[self.xx, self.xy, self.xz], [self.xy, self.yy, self.yz], [self.xz, self.yz, self.zz]] }
    }

    /// `R S Rᵀ` for a rotation `R`: an inertia tensor moved from the body frame to the world.
    pub fn rotated(self, r: &Mat3) -> Sym3 {
        let rs = r.mul(&self.to_mat3());
        let e = |i: usize, j: usize| rs.m[i][0] * r.m[j][0] + rs.m[i][1] * r.m[j][1] + rs.m[i][2] * r.m[j][2];
        Sym3 { xx: e(0, 0), yy: e(1, 1), zz: e(2, 2), xy: e(0, 1), xz: e(0, 2), yz: e(1, 2) }
    }

    pub fn is_finite(self) -> bool {
        self.xx.is_finite()
            && self.yy.is_finite()
            && self.zz.is_finite()
            && self.xy.is_finite()
            && self.xz.is_finite()
            && self.yz.is_finite()
    }
}

/// A general 3 by 3 matrix, rows first.
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct Mat3 {
    pub m: [[Real; 3]; 3],
}

impl Mat3 {
    pub const IDENTITY: Mat3 = Mat3 { m: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]] };

    /// The cross-product matrix: `skew(v) u = v × u`.
    pub fn skew(v: Vec3) -> Mat3 {
        Mat3 { m: [[0.0, -v.z, v.y], [v.z, 0.0, -v.x], [-v.y, v.x, 0.0]] }
    }

    pub fn mul_vec(&self, v: Vec3) -> Vec3 {
        let r = &self.m;
        Vec3::new(
            r[0][0] * v.x + r[0][1] * v.y + r[0][2] * v.z,
            r[1][0] * v.x + r[1][1] * v.y + r[1][2] * v.z,
            r[2][0] * v.x + r[2][1] * v.y + r[2][2] * v.z,
        )
    }

    /// `selfᵀ v`.
    pub fn mul_vec_t(&self, v: Vec3) -> Vec3 {
        let r = &self.m;
        Vec3::new(
            r[0][0] * v.x + r[1][0] * v.y + r[2][0] * v.z,
            r[0][1] * v.x + r[1][1] * v.y + r[2][1] * v.z,
            r[0][2] * v.x + r[1][2] * v.y + r[2][2] * v.z,
        )
    }

    pub fn mul(&self, o: &Mat3) -> Mat3 {
        let mut out = Mat3::default();
        for i in 0..3 {
            for j in 0..3 {
                out.m[i][j] = self.m[i][0] * o.m[0][j] + self.m[i][1] * o.m[1][j] + self.m[i][2] * o.m[2][j];
            }
        }
        out
    }

    pub fn add(&self, o: &Mat3) -> Mat3 {
        let mut out = *self;
        for i in 0..3 {
            for j in 0..3 {
                out.m[i][j] = self.m[i][j] + o.m[i][j];
            }
        }
        out
    }

    pub fn sub(&self, o: &Mat3) -> Mat3 {
        let mut out = *self;
        for i in 0..3 {
            for j in 0..3 {
                out.m[i][j] = self.m[i][j] - o.m[i][j];
            }
        }
        out
    }

    pub fn scale(&self, s: Real) -> Mat3 {
        let mut out = *self;
        for row in out.m.iter_mut() {
            for e in row.iter_mut() {
                *e = *e * s;
            }
        }
        out
    }

    /// Solves `self x = b` with the adjugate (Cramer's rule); `None` when the determinant is zero
    /// or not finite.
    pub fn solve(&self, b: Vec3) -> Option<Vec3> {
        let a = &self.m;
        let c00 = a[1][1] * a[2][2] - a[1][2] * a[2][1];
        let c01 = a[1][2] * a[2][0] - a[1][0] * a[2][2];
        let c02 = a[1][0] * a[2][1] - a[1][1] * a[2][0];
        let det = a[0][0] * c00 + a[0][1] * c01 + a[0][2] * c02;
        if det == 0.0 || !det.is_finite() {
            return None;
        }
        let c10 = a[0][2] * a[2][1] - a[0][1] * a[2][2];
        let c11 = a[0][0] * a[2][2] - a[0][2] * a[2][0];
        let c12 = a[0][1] * a[2][0] - a[0][0] * a[2][1];
        let c20 = a[0][1] * a[1][2] - a[0][2] * a[1][1];
        let c21 = a[0][2] * a[1][0] - a[0][0] * a[1][2];
        let c22 = a[0][0] * a[1][1] - a[0][1] * a[1][0];
        // inverse = adjugate / det; the adjugate is the cofactor matrix transposed.
        Some(Vec3::new(
            (c00 * b.x + c10 * b.y + c20 * b.z) / det,
            (c01 * b.x + c11 * b.y + c21 * b.z) / det,
            (c02 * b.x + c12 * b.y + c22 * b.z) / det,
        ))
    }
}
