//! Events out (layout rule 8): fixed-layout records appended in a fixed order and read by the
//! host after each step. Each island writes its own list during the solve; the lists are joined
//! in island order, and inside an island in ascending beam order. Later kinds (freeze, split,
//! wake, contact) are new constants, not new layouts.

use crate::real::Real;

/// A beam's elastic strain first passed its yield value; its rest length started to move.
pub const EVENT_YIELD: u32 = 1;
/// A beam's total strain passed its break value; it is now flagged broken.
pub const EVENT_BREAK: u32 = 2;

/// One event. `index` is a world index (a beam for yield and break), never a packed one.
/// `step` is the world's step count after the step that produced the event. `value` is the
/// strain that triggered it: elastic strain for a yield, total strain for a break.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Event {
    pub kind: u32,
    pub index: u32,
    pub step: u64,
    pub value: Real,
}

const _: () = assert!(std::mem::size_of::<Event>() == 24);
