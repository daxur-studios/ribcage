//! Yield and break (layout rules, "Yield and break"), checked once per 60 Hz step after the
//! last substep, on the packed island buffer. Two strains:
//!
//! - elastic strain `(length - rest) / rest0` decides yield: past the yield value `rest` moves
//!   toward the current length until the elastic strain is back at the yield value (the beam
//!   stays bent);
//! - total strain `(length - rest0) / rest0` decides break: past the break value the beam is
//!   flagged broken. Measured against the original length, so a beam that yielded a long way
//!   still reaches its break limit.
//!
//! Both are checked in tension and in compression. Break wins: a beam that breaks gets a break
//! event and no yield event. A yield event is sent once, the first time; the `YIELDED` flag
//! (part of the state hash) records it. Beams are visited in ascending packed order, which is
//! ascending world order, so events come out in ascending beam order.

use crate::event::{Event, EVENT_BREAK, EVENT_YIELD};
use crate::island::Island;
use crate::real::Real;
use crate::world::{BEAM_BROKEN, BEAM_YIELDED};

/// Updates yield and break for every live beam of the island and returns the events, with
/// `step` left at zero for the caller to stamp.
#[allow(clippy::needless_range_loop)] // explicit ascending index order is a determinism rule
pub fn check_failure(isl: &mut Island) -> Vec<Event> {
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
        let elastic: Real = (len - isl.rest[k]) / r0;
        let limit = isl.yield_strain[k];
        if elastic.abs() > limit {
            isl.rest[k] = if elastic > 0.0 { len - limit * r0 } else { len + limit * r0 };
            if isl.beam_flags[k] & BEAM_YIELDED == 0 {
                isl.beam_flags[k] |= BEAM_YIELDED;
                events.push(Event { kind: EVENT_YIELD, index, step: 0, value: elastic });
            }
        }
    }
    events
}
