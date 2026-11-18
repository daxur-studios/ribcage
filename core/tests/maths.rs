//! Own sine and cosine (phase 1.7). The platform library is the reference here, in tests only;
//! the core itself never calls it. Also the determinism lint: no file in `core/src` other than
//! `maths.rs` may call a library maths function.

use beam_core::maths::{atan2, cos, sin, sin_cos, PI, TAU};
use beam_core::real::Real;
use std::fs;
use std::path::Path;

/// A fixed, irrational-stride grid so the sweep is the same on every run.
fn grid(n: u64, start: Real, stride: Real) -> impl Iterator<Item = Real> {
    (0..n).map(move |i| start + i as Real * stride)
}

fn worst(xs: impl Iterator<Item = Real>) -> (Real, Real) {
    let (mut ws, mut wc) = (0.0 as Real, 0.0 as Real);
    for x in xs {
        ws = ws.max((sin(x) - x.sin()).abs());
        wc = wc.max((cos(x) - x.cos()).abs());
    }
    (ws, wc)
}

/// Within a couple of ulps of the library over the range angles live in, near zero, and out to
/// the edge of the domain. Measured worst case is recorded in ER 001, phase 1.7.
#[test]
fn matches_the_library_over_the_domain() {
    // 400,000 points over [-1000, 1000]
    let (s, c) = worst(grid(400_000, -1000.0, 0.005_000_123_456_789));
    assert!(s < 2.5e-16 && c < 2.5e-16, "sin {s:e}, cos {c:e}");
    // 100,000 points over [-0.8, 0.8] (the kernel range, small angles)
    let (s, c) = worst(grid(100_000, -0.8, 0.000_016_000_000_7));
    assert!(s < 1.2e-16 && c < 1.2e-16, "sin {s:e}, cos {c:e}");
    // Out to the edge of the domain.
    let (s, c) = worst(grid(100_000, -1.0e6, 20.000_123_456_7));
    assert!(s < 2.5e-16 && c < 2.5e-16, "sin {s:e}, cos {c:e}");
}

/// The reduction is exact enough that small results near multiples of π/2 keep their digits:
/// the error is relative to the tiny value, not to one.
#[test]
fn reduction_keeps_small_results_near_multiples_of_half_pi() {
    for k in -2000i64..=2000 {
        let x = k as Real * (PI / 2.0);
        let (s, c) = (sin(x), cos(x));
        let (rs, rc) = (x.sin(), x.cos());
        // Whichever of the two is near zero must agree with the library to a few percent of
        // itself; the library reduces with more bits than ours, but both are below 1e-15.
        let tiny = if k % 2 == 0 { (s, rs) } else { (c, rc) };
        assert!(
            (tiny.0 - tiny.1).abs() < 3e-16,
            "k {k}: {} vs {}",
            tiny.0,
            tiny.1
        );
    }
}

#[test]
fn exact_points_and_symmetry() {
    assert_eq!(sin(0.0), 0.0);
    assert_eq!(cos(0.0), 1.0);
    assert!(sin(-0.0).is_sign_negative() && sin(-0.0) == 0.0);
    for x in grid(20_000, 0.0, 0.173_205_080_7) {
        // Odd and even symmetry hold bit for bit, since the reduction is symmetric.
        assert_eq!(sin(-x).to_bits(), (-sin(x)).to_bits(), "sin at {x}");
        assert_eq!(cos(-x).to_bits(), cos(x).to_bits(), "cos at {x}");
        // Pythagoras to rounding.
        let (s, c) = sin_cos(x);
        assert!((s * s + c * c - 1.0).abs() < 4.5e-16, "at {x}");
    }
    // Quarter and half turns land within an ulp or two of the exact values.
    assert!((sin(PI / 2.0) - 1.0).abs() < 1.2e-16);
    assert!(cos(PI / 2.0).abs() < 1.3e-16);
    assert!(sin(PI).abs() < 1.3e-16);
    assert!((cos(PI) + 1.0).abs() < 1.2e-16);
    assert!(sin(TAU).abs() < 2.5e-16);
}

/// Sum and difference identities tie sine and cosine together without the library.
#[test]
fn angle_addition_holds() {
    // Small arguments, so the rounding of `a + b` itself stays below the tolerance.
    for (a, b) in grid(5_000, -3.0, 0.001_200_13).zip(grid(5_000, -2.0, 0.000_900_17)) {
        let lhs = sin(a + b);
        let rhs = sin(a) * cos(b) + cos(a) * sin(b);
        assert!((lhs - rhs).abs() < 6e-16, "{a} + {b}: {lhs} vs {rhs}");
    }
}

#[test]
fn sin_cos_is_bit_identical_to_the_two_calls() {
    for x in grid(50_000, -300.0, 0.012_345_678_9) {
        let (s, c) = sin_cos(x);
        assert_eq!(s.to_bits(), sin(x).to_bits());
        assert_eq!(c.to_bits(), cos(x).to_bits());
    }
}

/// Outside the domain the answer is NaN, so the bad-number freeze catches a runaway angle.
#[test]
fn out_of_domain_is_nan() {
    for x in [
        Real::NAN,
        Real::INFINITY,
        Real::NEG_INFINITY,
        1.0e6 + 1.0,
        -1.0e7,
        1.0e300,
    ] {
        assert!(sin(x).is_nan() && cos(x).is_nan(), "{x}");
        let (s, c) = sin_cos(x);
        assert!(s.is_nan() && c.is_nan(), "{x}");
    }
    assert!(sin(1.0e6).is_finite() && cos(-1.0e6).is_finite());
}

/// Golden bits. A hash over the raw bits of both functions on a fixed grid. If this changes,
/// the maths changed on purpose or the platform disagrees: either way, a reason to stop. The
/// WebAssembly build must produce the same value (checked when the core runs in the browser,
/// milestone 2).
#[test]
fn golden_bits_do_not_move() {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    let mut mix = |v: u64| {
        for k in 0..8 {
            h ^= (v >> (8 * k)) & 0xff;
            h = h.wrapping_mul(0x0000_0100_0000_01b3);
        }
    };
    for x in grid(20_000, -500.0, 0.050_000_7).chain(grid(2_000, -1.0e6, 1000.000_3)) {
        let (s, c) = sin_cos(x);
        mix(s.to_bits());
        mix(c.to_bits());
    }
    assert_eq!(h, GOLDEN, "golden hash is now {h:#018x}");
}
const GOLDEN: u64 = 0x5a4c_08f0_a35e_d69a;

/// The determinism list says library maths functions are not allowed in the core. Scan the
/// source for calls to them; `maths.rs` is the one file that may mention them (it does not call
/// them, but its docs name them). Comment lines are skipped.
#[test]
fn no_library_maths_in_core_source() {
    let forbidden = [
        ".sin(",
        ".cos(",
        ".tan(",
        ".asin(",
        ".acos(",
        ".atan(",
        ".atan2(",
        ".sinh(",
        ".cosh(",
        ".tanh(",
        ".sin_cos(",
        ".exp(",
        ".exp2(",
        ".exp_m1(",
        ".ln(",
        ".ln_1p(",
        ".log(",
        ".log2(",
        ".log10(",
        ".powf(",
        ".powi(",
        ".cbrt(",
        ".hypot(",
        ".mul_add(",
        "f64::sin",
        "f64::cos",
        "f64::powf",
        "f64::exp",
        "f32::sin",
        "f32::cos",
        "libm::",
    ];
    let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
    let mut checked = 0;
    for entry in fs::read_dir(&dir).expect("core/src") {
        let path = entry.unwrap().path();
        if path.extension().map_or(true, |e| e != "rs") || path.file_name().unwrap() == "maths.rs" {
            continue;
        }
        let text = fs::read_to_string(&path).unwrap();
        for (n, line) in text.lines().enumerate() {
            if line.trim_start().starts_with("//") {
                continue;
            }
            for f in forbidden {
                assert!(
                    !line.contains(f),
                    "{}:{}: library maths call `{f}` (use beam_core::maths)",
                    path.display(),
                    n + 1
                );
            }
        }
        checked += 1;
    }
    assert!(checked >= 8, "scanned {checked} files");
}

/// Prints the worst measured errors (run with `--ignored --nocapture`); the numbers in ER 001.
#[test]
#[ignore = "measurement, prints only"]
fn measure_worst_error() {
    for (name, g) in [
        (
            "[-1000, 1000]",
            grid(2_000_000, -1000.0, 0.001_000_123_456_789),
        ),
        ("[-0.8, 0.8]", grid(2_000_000, -0.8, 0.000_000_8)),
        ("[-1e6, 1e6]", grid(2_000_000, -1.0e6, 1.000_000_3)),
    ] {
        let (s, c) = worst(g);
        println!("{name}: worst abs error sin {s:.3e}, cos {c:.3e}");
    }
}

/// Own `atan2` (phase 5.5, joint angles) against the library, all round the circle and at the
/// axes, with the angle the result of our own sine and cosine read back.
#[test]
fn atan2_matches_the_library() {
    let mut worst: Real = 0.0;
    for x in grid(200_000, -PI, 0.000_031_415_9) {
        let (s, c) = sin_cos(x);
        for r in [1e-3, 1.0, 7.5e3] {
            let (y, xx) = (s * r, c * r);
            worst = worst.max((atan2(y, xx) - y.atan2(xx)).abs());
        }
    }
    assert!(worst < 1e-15, "worst {worst:e}"); // measured 8.9e-16, two ulps near ±π
    assert_eq!(atan2(0.0, 1.0), 0.0);
    assert_eq!(atan2(0.0, -1.0), PI);
    assert_eq!(atan2(0.0, 0.0), 0.0);
    assert!((atan2(1.0, 0.0) - PI / 2.0).abs() < 2.3e-16);
    assert!((atan2(-1.0, 0.0) + PI / 2.0).abs() < 2.3e-16);
    assert!(atan2(Real::NAN, 1.0).is_nan());
}
