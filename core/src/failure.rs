//! Break (layout rules, "Yield and break"), checked once per 60 Hz step after the last
//! substep, on the packed island buffer. Yield is not checked here: it is a force cap inside
//! the substeps ([`crate::solver::solve_distance`]), which moves `rest` and sets the `YIELDED`
//! flag. This module turns that flag into an event and decides break.
//!
//! - Break: total strain `(length - rest0) / rest0` past the break value flags the beam broken,
//!   in tension and in compression. Measured against the original length, so a beam that
//!   yielded a long way still reaches its break limit.
//! - Yield event: sent once, for a beam whose `YIELDED` flag was set during this step
//!   (`flags_before` is the flags at the start of the step). Its `value` is the beam's total
//!   strain at the end of the step, the same quantity a break event reports.
//! - Break wins: a beam that breaks gets a break event and no yield event (its `YIELDED` flag
//!   stays set).
//!
//! Beams are visited in ascending packed order, which is ascending world order, so events
//! come out in ascending beam order within the island; the world joins islands in island order.

use crate::event::{Event, EVENT_BREAK, EVENT_YIELD};
use crate::island::Island;
use crate::real::Real;
use crate::world::{BEAM_BROKEN, BEAM_YIELDED};

/// Decides break for every live beam of the island, reports newly yielded beams, and returns
/// the events, with `step` left at zero for the caller to stamp.
#[allow(clippy::needless_range_loop)] // explicit ascending index order is a determinism rule
pub fn check_failure(isl: &mut Island, flags_before: &[u32]) -> Vec<Event> {
    let mut events = Vec::new();
    for k in 0..isl.beam_a.len() {
        let r0 = isl.rest0[k];
        if r0 == 0.0 {
            continue; // no original length, no strain; a generator bug that 1.6 guards
        }
        let len = (isl.pos[isl.beam_b[k] as usize] - isl.pos[isl.beam_a[k] as usize]).length();
        let index = isl.beam_world[k];
        let total: Real = (len - r0) / r0;
        if total.abs() > isl.break_strain[k] {
            isl.beam_flags[k] |= BEAM_BROKEN;
            isl.last_lambda[k] = 0.0; // a broken beam carries nothing
            events.push(Event { kind: EVENT_BREAK, index, step: 0, value: total });
            continue;
        }
        let newly = isl.beam_flags[k] & BEAM_YIELDED != 0 && flags_before[k] & BEAM_YIELDED == 0;
        if newly {
            events.push(Event { kind: EVENT_YIELD, index, step: 0, value: total });
        }
    }
    events
}
