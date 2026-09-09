// OmniCore logical controller ABI. Button bits and axis indices are stable.
pub const FACE_EAST: u64 = 1 << 0;
pub const FACE_NORTH: u64 = 1 << 1;
pub const SELECT: u64 = 1 << 2;
pub const START: u64 = 1 << 3;
pub const UP: u64 = 1 << 4;
pub const DOWN: u64 = 1 << 5;
pub const LEFT: u64 = 1 << 6;
pub const RIGHT: u64 = 1 << 7;
pub const FACE_SOUTH: u64 = 1 << 8;
pub const FACE_WEST: u64 = 1 << 9;
pub const L1: u64 = 1 << 10;
pub const R1: u64 = 1 << 11;
pub const L2: u64 = 1 << 12;
pub const R2: u64 = 1 << 13;
pub const L3: u64 = 1 << 14;
pub const R3: u64 = 1 << 15;

pub const AXIS_LEFT_X: usize = 0;
pub const AXIS_LEFT_Y: usize = 1;
pub const AXIS_RIGHT_X: usize = 2;
pub const AXIS_RIGHT_Y: usize = 3;
pub const AXIS_LEFT_TRIGGER: usize = 4;
pub const AXIS_RIGHT_TRIGGER: usize = 5;
pub const AXIS_AUX_X: usize = 6;
pub const AXIS_AUX_Y: usize = 7;
pub const AXIS_COUNT: usize = 8;
