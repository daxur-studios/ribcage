//! Node-beam physics core. Pure simulation: no rendering, no browser dependencies.
//! Built to WebAssembly for the game and natively for tests and benchmarks.

/// Fixed simulation rate in hertz. Rendering interpolates between steps.
pub const STEP_HZ: u32 = 60;

/// Seconds advanced by one fixed step.
pub const STEP_SECONDS: f32 = 1.0 / STEP_HZ as f32;

/// A point mass: the smallest element of the node-beam model.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Node {
    pub position: [f32; 3],
    pub velocity: [f32; 3],
    pub mass: f32,
    pub radius: f32,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn step_is_one_sixtieth_of_a_second() {
        assert!((STEP_SECONDS * STEP_HZ as f32 - 1.0).abs() < 1e-6);
    }
}
