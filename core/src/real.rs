//! The one number type and the one vector type of the core (layout rules 1 and 2).
//! Nothing outside this module and the output buffers names `f32` or `f64`.

/// Scalar used by all simulation maths. Switching precision is a one-line change here.
pub type Real = f64;

/// Three components, always. A planar scene keeps `z` at zero.
///
/// `repr(C)`: the host reads arrays of these straight out of WebAssembly memory as three
/// consecutive `Real`s per node ([`crate::ffi`]), so the layout is part of the boundary.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct Vec3 {
    pub x: Real,
    pub y: Real,
    pub z: Real,
}

const _: () = assert!(std::mem::size_of::<Vec3>() == 3 * std::mem::size_of::<Real>());

impl Vec3 {
    pub const ZERO: Vec3 = Vec3 { x: 0.0, y: 0.0, z: 0.0 };

    pub const fn new(x: Real, y: Real, z: Real) -> Vec3 {
        Vec3 { x, y, z }
    }

    /// Only add and multiply, in a fixed order: identical bits natively and in WebAssembly.
    pub fn dot(self, o: Vec3) -> Real {
        self.x * o.x + self.y * o.y + self.z * o.z
    }

    pub fn length_squared(self) -> Real {
        self.dot(self)
    }

    /// `self × o`, each component two products and one subtraction in a fixed order.
    pub fn cross(self, o: Vec3) -> Vec3 {
        Vec3::new(self.y * o.z - self.z * o.y, self.z * o.x - self.x * o.z, self.x * o.y - self.y * o.x)
    }

    /// Whether all three components are finite.
    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }
}

impl core::ops::Neg for Vec3 {
    type Output = Vec3;
    fn neg(self) -> Vec3 {
        Vec3::new(-self.x, -self.y, -self.z)
    }
}

impl core::ops::Add for Vec3 {
    type Output = Vec3;
    fn add(self, o: Vec3) -> Vec3 {
        Vec3::new(self.x + o.x, self.y + o.y, self.z + o.z)
    }
}

impl core::ops::Sub for Vec3 {
    type Output = Vec3;
    fn sub(self, o: Vec3) -> Vec3 {
        Vec3::new(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}

impl core::ops::Mul<Real> for Vec3 {
    type Output = Vec3;
    fn mul(self, s: Real) -> Vec3 {
        Vec3::new(self.x * s, self.y * s, self.z * s)
    }
}

impl core::ops::Div<Real> for Vec3 {
    type Output = Vec3;
    fn div(self, s: Real) -> Vec3 {
        Vec3::new(self.x / s, self.y / s, self.z / s)
    }
}

impl Vec3 {
    /// Square root is correctly rounded in IEEE 754, so it is on the determinism list.
    pub fn length(self) -> Real {
        self.length_squared().sqrt()
    }
}
