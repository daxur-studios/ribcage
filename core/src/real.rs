//! The one number type and the one vector type of the core (layout rules 1 and 2).
//! Nothing outside this module and the output buffers names `f32` or `f64`.

/// Scalar used by all simulation maths. Switching precision is a one-line change here.
pub type Real = f64;

/// Three components, always. A planar scene keeps `z` at zero.
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct Vec3 {
    pub x: Real,
    pub y: Real,
    pub z: Real,
}

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
