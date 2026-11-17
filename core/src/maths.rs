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

use crate::real::Real;

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
