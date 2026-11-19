//! Air (plan 006, phase 6.1): the world's air settings and the density they give.
//!
//! Phase 6.1 holds only the settings and the density function; the aero elements and the pass that
//! turns them into force arrive in phase 6.2. The settings are set by `CMD_SET_AIR` and enter the
//! state hash only once that command has run, so a world that never sent it hashes as before.
//!
//! Density is `ρ0 · exp(-y / H)` with the core's own [`crate::maths::exp`] (a library `exp` differs
//! between platforms). Gravity stays flat. Only add, subtract, multiply and divide appear here.

use crate::maths::exp;
use crate::real::{Real, Vec3};

/// Sea-level air density in kg/m³ ([`Air::default`]).
pub const DEFAULT_RHO0: Real = 1.225;
/// Density scale height in metres ([`Air::default`]); zero means constant density.
pub const DEFAULT_SCALE_HEIGHT: Real = 8500.0;
/// Speed of sound in m/s ([`Air::default`]).
pub const DEFAULT_SOUND_SPEED: Real = 340.0;

/// The air the world sits in. Never read by the solver; phase 6.2's air pass reads it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Air {
    /// `CMD_SET_AIR` has run at least once: the settings are hashed from then.
    pub ever: bool,
    /// Sea-level density, kg/m³ (zero or more).
    pub rho0: Real,
    /// Scale height, metres (zero or more); zero means constant density `rho0`.
    pub scale_height: Real,
    /// Wind velocity, m/s.
    pub wind: Vec3,
    /// Speed of sound, m/s (above zero).
    pub sound_speed: Real,
}

impl Default for Air {
    fn default() -> Air {
        Air {
            ever: false,
            rho0: DEFAULT_RHO0,
            scale_height: DEFAULT_SCALE_HEIGHT,
            wind: Vec3::ZERO,
            sound_speed: DEFAULT_SOUND_SPEED,
        }
    }
}

impl Air {
    /// Air density at height `y` metres: `rho0 · exp(-y / H)`, or `rho0` when `H` is zero.
    pub fn density(&self, y: Real) -> Real {
        if self.scale_height == 0.0 {
            self.rho0
        } else {
            self.rho0 * exp(-y / self.scale_height)
        }
    }
}
