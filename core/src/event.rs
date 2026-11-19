//! Events out (layout rule 8): fixed-layout records appended in a fixed order and read by the
//! host after each step. Each island writes its own list during the solve; the lists are joined
//! in island order, and inside an island in ascending beam order. Later kinds (split,
//! wake, contact) are new constants, not new layouts.

use crate::real::Real;

/// A beam first reached its yield force; its rest length started to move.
pub const EVENT_YIELD: u32 = 1;
/// A beam's total strain passed its break value; it is now flagged broken.
pub const EVENT_BREAK: u32 = 2;

/// An island met a bad number (non-finite position or velocity) and was frozen. `index` is the
/// island's lowest movable node (a node, not a beam) and `value` is the substep number (1-based,
/// as a `Real`) at whose end the bad value was found.
pub const EVENT_FREEZE: u32 = 3;

/// A sleeping segment woke (phase 5.8). `index` is the segment; `value` is the reason, a
/// `sleep::WAKE_*` number as a `Real`.
pub const EVENT_WAKE: u32 = 4;
/// A segment went to sleep (phase 5.8). `index` is the segment; `value` is its last reading as a
/// fraction of yield.
pub const EVENT_SLEEP: u32 = 5;
/// The structure fell apart and a new vessel was split off (phase 5.10). `index` is the new
/// vessel's id; `value` is the id of the vessel it came from, as a `Real` (`u32::MAX` for a piece
/// made only of nodes added since the last check). The new vessel's segments follow at once, as
/// [`EVENT_VESSEL_SEGMENT`] records. One per new vessel, in order of each piece's lowest node.
pub const EVENT_SPLIT: u32 = 6;
/// One segment of the vessel just split off (phase 5.10): `index` is the segment; `value` is the
/// vessel's id as a `Real`. In ascending segment order, right after their [`EVENT_SPLIT`].
pub const EVENT_VESSEL_SEGMENT: u32 = 7;

/// One event. `index` is a world index (a beam for yield and break, a node for freeze), never a
/// packed one.
/// `step` is the world's step count after the step that produced the event. `value` is the
/// beam's total strain `(length - rest0) / rest0` at the end of that step, for a yield and a
/// break alike. A freeze event's `value` is the substep number instead.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Event {
    pub kind: u32,
    pub index: u32,
    pub step: u64,
    pub value: Real,
}

const _: () = assert!(std::mem::size_of::<Event>() == 24);
