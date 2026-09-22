use crate::state::{StateReader, StateWriter};

const CELL_COUNT: usize = 8192;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Phase {
    Command,
    Finish,
    Window,
    Index,
    Direction,
    Move,
    MoveIndex,
    Coordinate,
    Zero,
    Absorb,
    DoubleZero,
    RomStart,
    RomRead,
    PlanarCount,
    PlanarData,
    PlanarRead,
    DecodeCount,
    DecodeLength,
    Decode,
    Origin,
    SearchRange,
    SearchCell,
    Terrain,
    Cost,
    Solve,
    ResultRange,
    ResultCell,
    ResultCost,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum DecodePhase {
    SymbolPrefix,
    SymbolValue,
    TreeSize,
    TreeLength,
    Prefix,
    Suffix,
    CopySize,
    CopyOffset,
}

#[derive(Clone, Copy, Default)]
struct Cell {
    terrain: u8,
    cost: u8,
    weight: i16,
}

#[derive(Clone, Copy, Default)]
struct Ring {
    x: i16,
    y: i16,
    low: u16,
    high: u16,
    radius: u16,
    steps: u16,
    side: u16,
}

#[derive(Clone)]
pub(crate) struct Dsp3 {
    phase: Phase,
    data: u16,
    status: u8,
    index: u16,
    count: u16,
    width: u8,
    height: u8,
    move_x: i16,
    move_y: i16,
    x: u16,
    y: u16,
    pixels: [u8; 8],
    planes: [u8; 8],
    decode_phase: DecodePhase,
    symbols: [u16; 512],
    offsets: [u16; 8],
    lengths: [u8; 8],
    symbols_left: u16,
    outputs_left: u16,
    symbol: u16,
    bit_word: u16,
    partial_bits: u16,

    bits_available: u16,
    bits_needed: u16,
    symbol_prefix: u16,
    tree_entries: u16,
    prefix_bits: u16,
    prefix: u16,
    copy_bits: u16,
    cells: [Cell; CELL_COUNT],
    ring: Ring,
    origin_x: i16,
    origin_y: i16,
    searched_radius: u16,
    returned_radius: u16,
    cell: u16,
}

impl Default for Dsp3 {
    fn default() -> Self {
        Self {
            phase: Phase::Command,
            data: 0x80,
            status: 0x84,
            index: 0,
            count: 0,
            width: 0,

            height: 0,
            move_x: 0,
            move_y: 0,
            x: 0,
            y: 0,
            pixels: [0; 8],
            planes: [0; 8],
            decode_phase: DecodePhase::SymbolPrefix,
            symbols: [0; 512],
            offsets: [0; 8],
            lengths: [0; 8],
            symbols_left: 0,
            outputs_left: 0,
            symbol: 0,
            bit_word: 0,
            partial_bits: 0,
            bits_available: 0,
            bits_needed: 0,
            symbol_prefix: 0,
            tree_entries: 0,
            prefix_bits: 0,
            prefix: 0,
            copy_bits: 0,
            cells: [Cell::default(); CELL_COUNT],
            ring: Ring::default(),

            origin_x: 0,
            origin_y: 0,
            searched_radius: 0,
            returned_radius: 0,
            cell: 0,
        }
    }
}

impl Dsp3 {
    pub(crate) fn reset(&mut self) {
        *self = Self::default();
    }

    pub(crate) fn read(&mut self, address: u16) -> u8 {
        if address >= 0xc000 {
            return self.status;
        }
        if self.status & 0x04 != 0 {
            let value = self.data as u8;
            self.transfer();
            return value;
        }
        let high = self.status & 0x10 != 0;
        self.status ^= 0x10;

        let value = if high {
            (self.data >> 8) as u8
        } else {
            self.data as u8
        };
        if high {
            self.transfer();
        }
        value
    }

    pub(crate) fn write(&mut self, address: u16, value: u8) {
        if address >= 0xc000 {
            return;
        }
        let byte_mode = self.status & 0x04 != 0;
        let high = !byte_mode && self.status & 0x10 != 0;
        if high {
            self.data = (self.data & 0x00ff) | (u16::from(value) << 8);
        } else {
            self.data = (self.data & 0xff00) | u16::from(value);
        }
        if !byte_mode {
            self.status ^= 0x10;
        }
        if byte_mode || high {
            self.transfer();
        }
    }

    fn idle(&mut self) {
        self.phase = Phase::Command;
        self.data = 0x80;
        self.status = 0x84;
    }

    fn linear(&self, x: u16, y: u16) -> u16 {
        let doubled = (u32::from(self.width) * u32::from(y) + u32::from(x)).wrapping_mul(2) as u16;
        ((doubled as i16) >> 1) as u16
    }

    fn data_rom(index: usize) -> u16 {
        const CONFIG: [u16; 19] = [
            0, 15, 1024, 512, 320, 1024, 512, 64, 125, 126, 126, 123, 124, 125, 123, 124, 2, 32, 48,
        ];
        const MASKS: [u16; 5] = [43, 127, 32, 255, 0xff00];
        const HEADERS: [i16; 18] = [
            -66, -63, -59, -54, -48, -41, -33, -24, -14, -3, -57, -44, -30, -15, -53, -36, -18, -18,
        ];
        const BASES: [u16; 18] = [
            0, 1, 3, 6, 10, 15, 21, 28, 36, 45, 55, 0, 12, 25, 39, 0, 16, 33,
        ];
        const TAIL: [u16; 26] = [
            340, 536, 272, 176, 204, 176, 136, 176, 68, 176, 0, 176, 254, 0xff07, 2, 255, 248, 7,
            254, 238, 2047, 512, 239, 0xf800, 0x700, 238,
        ];
        const NEIGHBOR: [i16; 12] = [-1, -1, -1, 0, 0, 1, 1, 1, 1, 0, 0, -1];
        const HEX_STEP: [i16; 12] = [-1, 0, -1, 1, 0, 1, 1, 0, 0, -1, -1, -1];

        let index = index & 1023;
        if index < 16 {
            return 0x8000u16 >> index;
        }
        if index < 24 {
            return 2u16 << (index - 16);
        }

        if (0x18..0x2b).contains(&index) {
            return CONFIG[index - 0x18];
        }
        if (0x2b..0xab).contains(&index) {
            const WAVE: [i16; 128] = [
                0, 13, 25, 38, 50, 62, 74, 86, 98, 109, 121, 132, 142, 152, 162, 172, 181, 190,
                198, 206, 213, 220, 226, 231, 236, 241, 245, 248, 251, 253, 255, 256, 256, 256,
                255, 253, 251, 248, 245, 241, 237, 231, 226, 220, 213, 206, 198, 190, 181, 172,
                162, 153, 142, 132, 121, 110, 98, 86, 74, 62, 50, 38, 25, 13, 0, -13, -25, -37,
                -50, -62, -74, -86, -98, -109, -121, -131, -142, -152, -162, -172, -181, -190,
                -198, -206, -213, -219, -226, -231, -236, -241, -245, -248, -251, -253, -255, -256,
                -256, -256, -255, -253, -251, -248, -245, -241, -237, -232, -226, -220, -213, -206,
                -198, -190, -181, -172, -163, -153, -142, -132, -121, -110, -98, -87, -75, -62,
                -50, -38, -25, -13,
            ];
            return WAVE[index - 0x2b] as u16;
        }
        if (0xab..0xb0).contains(&index) {
            return MASKS[index - 0xab];
        }

        if (0xb0..0x380).contains(&index) {
            let relative = index - 0xb0;
            let bank = relative / 360;
            let in_bank = relative % 360;
            let row = in_bank / 20;
            let column = in_bank % 20;
            if row < 18 {
                if column == 0 {
                    return HEADERS[row] as u16;
                }
                if (1..=row + 1).contains(&column) {
                    return BASES[row] + (column - 1) as u16;
                }
                if column == row + 2 {
                    let adjustment = if bank == 0 { 68 } else { -340 };
                    return (i32::from(BASES[row]) + adjustment) as u16;
                }
            }
            return 0;
        }
        if (0x380..0x39a).contains(&index) {
            return TAIL[index - 0x380];
        }

        if (0x39a..0x3b2).contains(&index) {
            return NEIGHBOR[(index - 0x39a) % 12] as u16;
        }
        if (0x3b2..0x3ca).contains(&index) {
            return HEX_STEP[(index - 0x3b2) % 12] as u16;
        }
        if (0x3cc..0x3d2).contains(&index) {
            return (68 * (index - 0x3cc)) as u16;
        }
        if index >= 0x3d2 {
            return 0xffff;
        }
        0
    }

    fn transfer(&mut self) {
        match self.phase {
            Phase::Command => {
                self.phase = match self.data {
                    0x02 => Phase::Coordinate,
                    0x03 => Phase::Index,
                    0x06 => Phase::Window,

                    0x07 => {
                        self.phase = Phase::Direction;
                        return;
                    }
                    0x0c | 0x0f => Phase::Zero,
                    0x10 => Phase::Absorb,
                    0x18 => Phase::PlanarCount,
                    0x1c => Phase::DoubleZero,
                    0x1e => Phase::SearchRange,
                    0x1f => Phase::RomStart,
                    0x38 => Phase::DecodeCount,
                    0x3e => Phase::Origin,
                    _ => return,
                };
                self.status = 0x80;
                self.index = 0;
            }
            Phase::Finish => self.idle(),
            Phase::Window => {
                self.width = self.data as u8;
                self.height = (self.data >> 8) as u8;
                self.idle();
            }
            Phase::Index => {
                self.data = self.linear(self.data as u8 as u16, (self.data >> 8) as u8 as u16);
                self.phase = Phase::Finish;
            }
            Phase::Direction => {
                let direction = usize::from(self.data);
                self.move_y = Self::data_rom(0x3b2 + direction * 2) as i16;
                self.move_x = Self::data_rom(0x3b3 + direction * 2) as i16;
                self.phase = Phase::Move;
                self.status = 0x80;
            }
            Phase::Move => {
                let x = self.data as u8 as i16;
                self.move_y = self
                    .move_y
                    .wrapping_add((self.data >> 8) as u8 as i16)
                    .wrapping_add(if x & 1 != 0 { self.move_x & 1 } else { 0 });
                self.move_x = self.move_x.wrapping_add(x);
                if self.move_x < 0 {
                    self.move_x = self.move_x.wrapping_add(i16::from(self.width));
                } else if self.move_x >= i16::from(self.width) {
                    self.move_x = self.move_x.wrapping_sub(i16::from(self.width));
                }

                if self.move_y < 0 {
                    self.move_y = self.move_y.wrapping_add(i16::from(self.height));
                } else if self.move_y >= i16::from(self.height) {
                    self.move_y = self.move_y.wrapping_sub(i16::from(self.height));
                }
                self.data = (self.move_x as u16) | ((self.move_y as u16) << 8);
                self.phase = Phase::MoveIndex;
            }
            Phase::MoveIndex => {
                self.data = self.linear(self.move_x as u16, self.move_y as u16);
                self.phase = Phase::Finish;
            }
            Phase::Coordinate => {
                self.index = self.index.wrapping_add(1);
                if self.index == 3 && self.data == 0xffff {
                    self.idle();
                } else if self.index == 4 {
                    self.x = self.data;
                } else if self.index == 5 {
                    self.y = self.data;
                    self.data = 1;
                } else if self.index == 6 {
                    self.data = self.x;
                } else if self.index == 7 {
                    self.data = self.y;
                    self.index = 0;
                }
            }
            Phase::Zero => {
                self.data = 0;
                self.phase = Phase::Finish;
            }
            Phase::Absorb => {
                if self.data == 0xffff {
                    self.idle();
                }
            }
            Phase::DoubleZero => {
                self.index = self.index.wrapping_add(1);
                if self.index >= 3 {
                    self.data = 0;
                }
                if self.index == 4 {
                    self.phase = Phase::Finish;
                }
            }
            Phase::RomStart => {
                self.index = 0;
                self.phase = Phase::RomRead;

                self.data = Self::data_rom(usize::from(self.index));
                self.index += 1;
            }
            Phase::RomRead => {
                self.data = Self::data_rom(usize::from(self.index));
                self.index += 1;
                if self.index == 1024 {
                    self.phase = Phase::Finish;
                }
            }
            Phase::PlanarCount => {
                self.count = self.data;
                self.index = 0;
                self.phase = Phase::PlanarData;
            }
            Phase::PlanarData => {
                let at = usize::from(self.index);
                if at + 1 < self.pixels.len() {
                    self.pixels[at] = self.data as u8;
                    self.pixels[at + 1] = (self.data >> 8) as u8;
                }
                self.index += 2;
                if self.index != 8 {
                    return;
                }
                self.planes.fill(0);
                for x in 0..8usize {
                    for bit in 0..8usize {
                        self.planes[bit] |= ((self.pixels[x] >> bit) & 1) << (7 - x);
                    }
                }
                self.count = self.count.saturating_sub(1);
                self.index = 0;
                self.phase = Phase::PlanarRead;
                self.emit_planar_word();
            }
            Phase::PlanarRead => self.emit_planar_word(),
            Phase::DecodeCount => {
                self.symbols_left = self.data;
                self.phase = Phase::DecodeLength;
            }
            Phase::DecodeLength => {
                self.outputs_left = self.data;
                self.index = 0;
                self.symbol = 0;
                self.bits_available = 0;
                self.bits_needed = 0;
                self.decode_phase = DecodePhase::SymbolPrefix;
                self.phase = Phase::Decode;

                self.status = 0xc0;
                if self.symbols_left == 0
                    || usize::from(self.symbols_left) > self.symbols.len()
                    || self.outputs_left == 0
                {
                    self.idle();
                }
            }
            Phase::Decode => self.decode(),
            Phase::Origin => {
                self.origin_x = self.data as u8 as i16;
                self.origin_y = (self.data >> 8) as u8 as i16;
                self.data = self.linear(self.origin_x as u16, self.origin_y as u16);
                self.cells[usize::from(self.data & 8191)] = Cell {
                    terrain: 0,
                    cost: 255,
                    weight: 0,
                };
                self.searched_radius = 0;
                self.returned_radius = 0;
                self.phase = Phase::Finish;
            }
            Phase::SearchRange => self.start_ring(false),
            Phase::SearchCell => {
                self.status = 0x84;
                self.phase = Phase::Terrain;
            }
            Phase::Terrain => {
                self.cells[usize::from(self.cell & 8191)].terrain = self.data as u8;
                self.phase = Phase::Cost;
            }
            Phase::Cost => {
                let cell = &mut self.cells[usize::from(self.cell & 8191)];
                cell.cost = self.data as u8;
                cell.weight = if self.ring.radius == 1 && cell.terrain & 1 == 0 {
                    i16::from(cell.cost)
                } else {
                    255
                };
                let direction = i32::from(self.ring.side) + 2;
                let mut x = self.ring.x;
                let mut y = self.ring.y;
                self.move_cell(direction, &mut x, &mut y, true);
                self.ring.x = x;
                self.ring.y = y;
                self.ring.steps = self.ring.steps.saturating_sub(1);
                self.emit_cell(false);
            }
            Phase::Solve => {
                self.solve_paths();
                self.phase = Phase::ResultRange;
            }
            Phase::ResultRange => self.start_ring(true),
            Phase::ResultCell => {
                self.data = self.cells[usize::from(self.cell & 8191)].weight as u16;
                let direction = i32::from(self.ring.side) + 2;
                let mut x = self.ring.x;
                let mut y = self.ring.y;
                self.move_cell(direction, &mut x, &mut y, true);
                self.ring.x = x;
                self.ring.y = y;
                self.ring.steps = self.ring.steps.saturating_sub(1);
                self.status = 0x84;
                self.phase = Phase::ResultCost;
            }
            Phase::ResultCost => self.emit_cell(true),
        }
    }

    fn emit_planar_word(&mut self) {
        if self.index >= 8 {
            if self.count == 0 {
                self.idle();
            } else {
                self.index = 0;
                self.phase = Phase::PlanarData;
                self.status = 0x80;
            }
            return;
        }

        let at = usize::from(self.index);
        self.data = u16::from(self.planes[at]) | (u16::from(self.planes[at + 1]) << 8);
        self.index += 2;
        self.phase = Phase::PlanarRead;
        self.status = 0x80;
    }

    fn take_bits(&mut self, count: u16) -> Option<u16> {
        if self.bits_needed == 0 {
            self.bits_needed = count;
            self.partial_bits = 0;
        }

        while self.bits_needed != 0 {
            if self.bits_available == 0 {
                self.status = 0xc0;
                return None;
            }

            self.partial_bits <<= 1;
            if self.bit_word & 0x8000 != 0 {
                self.partial_bits |= 1;
            }
            self.bit_word <<= 1;
            self.bits_available -= 1;
            self.bits_needed -= 1;
        }

        Some(self.partial_bits)
    }

    fn decode(&mut self) {
        if self.status & 0x40 != 0 {
            self.bit_word = self.data;
            self.bits_available = self.bits_available.saturating_add(16);
        }

        loop {
            match self.decode_phase {
                DecodePhase::SymbolPrefix => {
                    if self.symbols_left == 0 {
                        self.index = 0;
                        self.symbol = 0;
                        self.tree_entries = 0;
                        self.decode_phase = DecodePhase::TreeSize;
                        continue;
                    }

                    let Some(command) = self.take_bits(2) else {
                        return;
                    };
                    self.symbol_prefix = command;
                    self.decode_phase = DecodePhase::SymbolValue;
                }
                DecodePhase::SymbolValue => {
                    self.symbol = match self.symbol_prefix {
                        0 => {
                            let Some(value) = self.take_bits(9) else {
                                return;
                            };
                            value
                        }
                        1 => self.symbol.wrapping_add(1),
                        2 => {
                            let Some(value) = self.take_bits(1) else {
                                return;
                            };
                            self.symbol.wrapping_add(2 + value)
                        }
                        3 => {
                            let Some(value) = self.take_bits(4) else {
                                return;
                            };
                            self.symbol.wrapping_add(4 + value)
                        }
                        _ => {
                            self.idle();
                            return;
                        }
                    };

                    let at = usize::from(self.index);
                    if at >= self.symbols.len() {
                        self.idle();
                        return;
                    }
                    self.symbols[at] = self.symbol;
                    self.index += 1;
                    self.symbols_left = self.symbols_left.saturating_sub(1);
                    self.symbol_prefix = u16::MAX;
                    self.decode_phase = DecodePhase::SymbolPrefix;
                }
                DecodePhase::TreeSize => {
                    let Some(value) = self.take_bits(1) else {
                        return;
                    };
                    if value != 0 {
                        self.prefix_bits = 3;
                        self.tree_entries = 8;
                    } else {
                        self.prefix_bits = 2;
                        self.tree_entries = 4;
                    }
                    self.index = 0;
                    self.symbol = 0;
                    self.decode_phase = DecodePhase::TreeLength;
                }
                DecodePhase::TreeLength => {
                    if self.tree_entries == 0 {
                        self.prefix = u16::MAX;
                        self.copy_bits = 0;
                        self.decode_phase = DecodePhase::Prefix;
                        continue;
                    }

                    let Some(length) = self.take_bits(3) else {
                        return;
                    };
                    let length = length + 1;
                    let at = usize::from(self.index);
                    if at >= self.lengths.len() {
                        self.idle();
                        return;
                    }
                    self.lengths[at] = length as u8;
                    self.offsets[at] = self.symbol;
                    self.index += 1;
                    self.symbol = self.symbol.wrapping_add(1u16 << length);
                    self.tree_entries -= 1;
                }
                DecodePhase::Prefix => {
                    if self.outputs_left == 0 {
                        self.idle();
                        return;
                    }
                    let Some(prefix) = self.take_bits(self.prefix_bits) else {
                        return;
                    };
                    if usize::from(prefix) >= self.lengths.len() {
                        self.idle();
                        return;
                    }
                    self.prefix = prefix;
                    self.decode_phase = DecodePhase::Suffix;
                }
                DecodePhase::Suffix => {
                    let base = usize::from(self.prefix);
                    let Some(code) = self.take_bits(u16::from(self.lengths[base])) else {
                        return;
                    };
                    let symbol_index = usize::from(self.offsets[base].wrapping_add(code));
                    if symbol_index >= self.symbols.len() {
                        self.idle();
                        return;
                    }

                    let mut symbol = self.symbols[symbol_index];
                    self.prefix = u16::MAX;
                    if symbol & 0xff00 != 0 {
                        symbol = symbol.wrapping_add(0x7f02);
                        self.data = symbol;
                        self.status = 0x80;
                        self.decode_phase = DecodePhase::CopySize;
                        return;
                    }

                    self.outputs_left = self.outputs_left.saturating_sub(1);
                    self.data = symbol;
                    self.status = 0x80;
                    self.decode_phase = DecodePhase::Prefix;
                    if self.outputs_left == 0 {
                        self.phase = Phase::Finish;
                    }
                    return;
                }
                DecodePhase::CopySize => {
                    let Some(value) = self.take_bits(1) else {
                        return;
                    };
                    self.copy_bits = if value != 0 { 12 } else { 8 };
                    self.decode_phase = DecodePhase::CopyOffset;
                }
                DecodePhase::CopyOffset => {
                    let Some(value) = self.take_bits(self.copy_bits) else {
                        return;
                    };
                    self.outputs_left = self.outputs_left.saturating_sub(1);
                    self.data = value;
                    self.status = 0x80;
                    self.copy_bits = 0;
                    self.decode_phase = DecodePhase::Prefix;
                    if self.outputs_left == 0 {
                        self.phase = Phase::Finish;
                    }
                    return;
                }
            }
        }
    }

    fn start_ring(&mut self, result: bool) {
        let mut low = self.data & 0x00ff;
        let high = (self.data >> 8) & 0x00ff;
        if low == 0 {
            low = 1;
        }

        if result {
            if self.returned_radius >= low {
                low = self.returned_radius.saturating_add(1);
            }
            self.returned_radius = self.returned_radius.max(high);
        } else {
            if self.searched_radius >= low {
                low = self.searched_radius.saturating_add(1);
            }
            self.searched_radius = self.searched_radius.max(high);
        }

        self.ring = Ring {
            x: self.origin_x,
            y: self.origin_y,
            low,
            high,
            radius: low,
            steps: low,
            side: 0,
        };
        for _ in 0..low {
            let mut x = self.ring.x;
            let mut y = self.ring.y;
            self.move_cell(0, &mut x, &mut y, true);
            self.ring.x = x;
            self.ring.y = y;
        }
        self.emit_cell(result);
    }

    fn emit_cell(&mut self, result: bool) {
        loop {
            if self.ring.steps == 0 {
                self.ring.radius = self.ring.radius.saturating_add(1);
                self.ring.steps = self.ring.radius;
                self.ring.x = self.origin_x;
                self.ring.y = self.origin_y;
                for _ in 0..self.ring.radius {
                    let mut x = self.ring.x;
                    let mut y = self.ring.y;
                    self.move_cell(i32::from(self.ring.side), &mut x, &mut y, true);
                    self.ring.x = x;
                    self.ring.y = y;
                }
            }

            if self.ring.radius > self.ring.high {
                self.ring.side = self.ring.side.saturating_add(1);
                if self.ring.side >= 6 {
                    self.data = 0xffff;
                    self.status = 0x80;
                    self.phase = if result { Phase::Finish } else { Phase::Solve };
                    return;
                }

                self.ring.radius = self.ring.low;
                self.ring.steps = self.ring.low;
                self.ring.x = self.origin_x;
                self.ring.y = self.origin_y;
                for _ in 0..self.ring.low {
                    let mut x = self.ring.x;
                    let mut y = self.ring.y;
                    self.move_cell(i32::from(self.ring.side), &mut x, &mut y, true);
                    self.ring.x = x;
                    self.ring.y = y;
                }
                continue;
            }

            self.data = (self.ring.x as u8 as u16) | ((self.ring.y as u8 as u16) << 8);
            self.cell = self.linear(self.ring.x as u16, self.ring.y as u16) & 8191;
            self.status = 0x80;
            self.phase = if result {
                Phase::ResultCell
            } else {
                Phase::SearchCell
            };
            return;
        }
    }

    fn solve_paths(&mut self) {
        let mut x = self.origin_x;
        let mut y = self.origin_y;
        let mut radius = 1u16;

        while radius < self.ring.high {
            y = y.wrapping_sub(1);
            let mut turns = 6u16;
            let mut turn = 5i32;

            while turns != 0 {
                let mut steps = radius;
                while steps != 0 {
                    self.move_cell(turn, &mut x, &mut y, false);
                    if self.in_window(x, y) {
                        let cell_index = usize::from(self.linear(x as u16, y as u16) & 8191);
                        if self.cells[cell_index].cost < 0x80
                            && self.cells[cell_index].terrain < 0x40
                        {
                            let mut path = 255i16;
                            for direction in (1..=6).rev() {
                                let mut neighbor_x = x;
                                let mut neighbor_y = y;
                                self.move_cell(direction, &mut neighbor_x, &mut neighbor_y, false);
                                if self.in_window(neighbor_x, neighbor_y) {
                                    let neighbor_index = usize::from(
                                        self.linear(neighbor_x as u16, neighbor_y as u16) & 8191,
                                    );
                                    let neighbor = self.cells[neighbor_index];
                                    if (neighbor.terrain < 0x80 || neighbor.weight == 0)
                                        && neighbor.weight < path
                                    {
                                        path = neighbor.weight;
                                    }
                                }
                            }
                            if path != 255 {
                                self.cells[cell_index].weight =
                                    path.wrapping_add(i16::from(self.cells[cell_index].cost));
                            }
                        }
                    }
                    steps -= 1;
                }

                turn -= 1;
                if turn == 0 {
                    turn = 6;
                }
                turns -= 1;
            }
            radius += 1;
        }
    }

    fn in_window(&self, x: i16, y: i16) -> bool {
        x >= 0 && y >= 0 && x < i16::from(self.width) && y < i16::from(self.height)
    }

    fn move_cell(&self, direction: i32, x: &mut i16, y: &mut i16, wrap: bool) {
        let direction = usize::try_from(direction).unwrap_or(0) & 7;
        if wrap {
            let data_offset = (0x03b2 + direction * 2) & 0x03ff;
            let add_y = Self::data_rom(data_offset) as i16;
            let add_x = Self::data_rom(data_offset + 1) as i16;
            let old_x = *x as u8 as i16;
            let mut next_y = *y as u8 as i16;
            if old_x & 1 != 0 {
                next_y = next_y.wrapping_add(add_x & 1);
            }
            let mut next_x = add_x.wrapping_add(old_x);
            next_y = add_y.wrapping_add(next_y);

            if self.width != 0 {
                if next_x < 0 {
                    next_x = next_x.wrapping_add(i16::from(self.width));
                } else if next_x >= i16::from(self.width) {
                    next_x = next_x.wrapping_sub(i16::from(self.width));
                }
            }
            if self.height != 0 {
                if next_y < 0 {
                    next_y = next_y.wrapping_add(i16::from(self.height));
                } else if next_y >= i16::from(self.height) {
                    next_y = next_y.wrapping_sub(i16::from(self.height));
                }
            }
            *x = next_x;
            *y = next_y;
            return;
        }

        const HI_ADD: [i16; 16] = [
            0x00, 0xff, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x01, 0x00,
            0xff, 0x00,
        ];
        const LO_ADD: [i16; 8] = [0x00, 0x00, 0x01, 0x01, 0x00, 0xff, 0xff, 0x00];

        let old_x = *x as u8 as i16;
        let mut next_y = *y as u8 as i16;
        let add_x = LO_ADD[direction];
        let add_y = if old_x & 1 != 0 {
            HI_ADD[direction + 8]
        } else {
            HI_ADD[direction]
        };
        if old_x & 1 != 0 {
            next_y = next_y.wrapping_add(add_x & 1);
        }
        *x = add_x.wrapping_add(old_x);
        *y = add_y.wrapping_add(next_y);
    }

    fn phase_code(phase: Phase) -> u8 {
        match phase {
            Phase::Command => 0,
            Phase::Finish => 1,
            Phase::Window => 2,
            Phase::Index => 3,
            Phase::Direction => 4,
            Phase::Move => 5,
            Phase::MoveIndex => 6,
            Phase::Coordinate => 7,
            Phase::Zero => 8,
            Phase::Absorb => 9,
            Phase::DoubleZero => 10,
            Phase::RomStart => 11,
            Phase::RomRead => 12,
            Phase::PlanarCount => 13,
            Phase::PlanarData => 14,
            Phase::PlanarRead => 15,
            Phase::DecodeCount => 16,
            Phase::DecodeLength => 17,
            Phase::Decode => 18,
            Phase::Origin => 19,
            Phase::SearchRange => 20,
            Phase::SearchCell => 21,
            Phase::Terrain => 22,
            Phase::Cost => 23,
            Phase::Solve => 24,
            Phase::ResultRange => 25,
            Phase::ResultCell => 26,
            Phase::ResultCost => 27,
        }
    }

    fn decode_phase_code(phase: DecodePhase) -> u8 {
        match phase {
            DecodePhase::SymbolPrefix => 0,
            DecodePhase::SymbolValue => 1,
            DecodePhase::TreeSize => 2,
            DecodePhase::TreeLength => 3,
            DecodePhase::Prefix => 4,
            DecodePhase::Suffix => 5,
            DecodePhase::CopySize => 6,
            DecodePhase::CopyOffset => 7,
        }
    }

    fn phase_from_code(value: u8) -> Result<Phase, String> {
        Ok(match value {
            0 => Phase::Command,
            1 => Phase::Finish,
            2 => Phase::Window,
            3 => Phase::Index,
            4 => Phase::Direction,
            5 => Phase::Move,
            6 => Phase::MoveIndex,
            7 => Phase::Coordinate,
            8 => Phase::Zero,
            9 => Phase::Absorb,
            10 => Phase::DoubleZero,
            11 => Phase::RomStart,
            12 => Phase::RomRead,
            13 => Phase::PlanarCount,
            14 => Phase::PlanarData,
            15 => Phase::PlanarRead,
            16 => Phase::DecodeCount,
            17 => Phase::DecodeLength,
            18 => Phase::Decode,
            19 => Phase::Origin,
            20 => Phase::SearchRange,
            21 => Phase::SearchCell,
            22 => Phase::Terrain,
            23 => Phase::Cost,
            24 => Phase::Solve,
            25 => Phase::ResultRange,
            26 => Phase::ResultCell,
            27 => Phase::ResultCost,
            _ => return Err("invalid DSP-3 phase in save state".into()),
        })
    }

    fn decode_phase_from_code(value: u8) -> Result<DecodePhase, String> {
        Ok(match value {
            0 => DecodePhase::SymbolPrefix,
            1 => DecodePhase::SymbolValue,
            2 => DecodePhase::TreeSize,
            3 => DecodePhase::TreeLength,
            4 => DecodePhase::Prefix,
            5 => DecodePhase::Suffix,
            6 => DecodePhase::CopySize,
            7 => DecodePhase::CopyOffset,
            _ => return Err("invalid DSP-3 decode phase in save state".into()),
        })
    }

    pub(crate) fn save(&self, out: &mut StateWriter) {
        out.u8(Self::phase_code(self.phase));
        out.u16(self.data);
        out.u8(self.status);
        out.u16(self.index);
        out.u16(self.count);
        out.u8(self.width);
        out.u8(self.height);
        out.u16(self.move_x as u16);
        out.u16(self.move_y as u16);
        out.u16(self.x);
        out.u16(self.y);
        out.blob(&self.pixels);
        out.blob(&self.planes);
        out.u8(Self::decode_phase_code(self.decode_phase));
        for value in self.symbols {
            out.u16(value);
        }
        for value in self.offsets {
            out.u16(value);
        }
        out.blob(&self.lengths);
        out.u16(self.symbols_left);
        out.u16(self.outputs_left);
        out.u16(self.symbol);
        out.u16(self.bit_word);
        out.u16(self.partial_bits);
        out.u16(self.bits_available);
        out.u16(self.bits_needed);
        out.u16(self.symbol_prefix);
        out.u16(self.tree_entries);
        out.u16(self.prefix_bits);
        out.u16(self.prefix);
        out.u16(self.copy_bits);
        for cell in self.cells {
            out.u8(cell.terrain);
            out.u8(cell.cost);
            out.u16(cell.weight as u16);
        }
        out.u16(self.ring.x as u16);
        out.u16(self.ring.y as u16);
        out.u16(self.ring.low);
        out.u16(self.ring.high);
        out.u16(self.ring.radius);
        out.u16(self.ring.steps);
        out.u16(self.ring.side);
        out.u16(self.origin_x as u16);
        out.u16(self.origin_y as u16);
        out.u16(self.searched_radius);
        out.u16(self.returned_radius);
        out.u16(self.cell);
    }

    pub(crate) fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.phase = Self::phase_from_code(input.u8()?)?;
        self.data = input.u16()?;
        self.status = input.u8()?;
        self.index = input.u16()?;
        self.count = input.u16()?;
        self.width = input.u8()?;
        self.height = input.u8()?;
        self.move_x = input.u16()? as i16;
        self.move_y = input.u16()? as i16;
        self.x = input.u16()?;
        self.y = input.u16()?;

        let pixels = input.blob()?;
        if pixels.len() != self.pixels.len() {
            return Err("invalid DSP-3 pixel state length".into());
        }
        self.pixels.copy_from_slice(pixels);

        let planes = input.blob()?;
        if planes.len() != self.planes.len() {
            return Err("invalid DSP-3 plane state length".into());
        }
        self.planes.copy_from_slice(planes);

        self.decode_phase = Self::decode_phase_from_code(input.u8()?)?;
        for value in &mut self.symbols {
            *value = input.u16()?;
        }
        for value in &mut self.offsets {
            *value = input.u16()?;
        }
        let lengths = input.blob()?;
        if lengths.len() != self.lengths.len() {
            return Err("invalid DSP-3 decode-length state".into());
        }
        self.lengths.copy_from_slice(lengths);
        self.symbols_left = input.u16()?;
        self.outputs_left = input.u16()?;
        self.symbol = input.u16()?;
        self.bit_word = input.u16()?;
        self.partial_bits = input.u16()?;
        self.bits_available = input.u16()?;
        self.bits_needed = input.u16()?;
        self.symbol_prefix = input.u16()?;
        self.tree_entries = input.u16()?;
        self.prefix_bits = input.u16()?;
        self.prefix = input.u16()?;
        self.copy_bits = input.u16()?;
        for cell in &mut self.cells {
            cell.terrain = input.u8()?;
            cell.cost = input.u8()?;
            cell.weight = input.u16()? as i16;
        }
        self.ring.x = input.u16()? as i16;
        self.ring.y = input.u16()? as i16;
        self.ring.low = input.u16()?;
        self.ring.high = input.u16()?;
        self.ring.radius = input.u16()?;
        self.ring.steps = input.u16()?;
        self.ring.side = input.u16()?;
        self.origin_x = input.u16()? as i16;
        self.origin_y = input.u16()? as i16;
        self.searched_radius = input.u16()?;
        self.returned_radius = input.u16()?;
        self.cell = input.u16()?;

        if self.bits_available > 16 || self.bits_needed > 16 || self.prefix_bits > 3 {
            return Err("invalid DSP-3 bitstream state".into());
        }
        if usize::from(self.index) > self.symbols.len() {
            return Err("invalid DSP-3 index in save state".into());
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::platform::PlatformId;

    fn write_word(dsp: &mut Dsp3, value: u16) {
        dsp.write(0x8000, value as u8);
        dsp.write(0x8000, (value >> 8) as u8);
    }

    fn read_word(dsp: &mut Dsp3) -> u16 {
        let low = u16::from(dsp.read(0x8000));
        let high = u16::from(dsp.read(0x8000));
        low | (high << 8)
    }

    fn start_command(dsp: &mut Dsp3, command: u8) {
        assert_eq!(dsp.read(0xc000), 0x84);
        dsp.write(0x8000, command);
    }

    #[test]
    fn window_and_linear_index_commands_follow_word_protocol() {
        let mut dsp = Dsp3::default();
        start_command(&mut dsp, 0x06);
        write_word(&mut dsp, 0x0810);
        assert_eq!(dsp.read(0xc000), 0x84);

        dsp.write(0x8000, 0x03);
        write_word(&mut dsp, 0x0203);
        assert_eq!(read_word(&mut dsp), 0x0023);
        assert_eq!(dsp.read(0xc000), 0x84);
    }

    #[test]
    fn generated_data_rom_matches_reference_table_digest() {
        let mut hash = 0xcbf2_9ce4_8422_2325u64;
        for index in 0..1024 {
            for byte in Dsp3::data_rom(index).to_le_bytes() {
                hash ^= u64::from(byte);
                hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
            }
        }
        assert_eq!(hash, 0xd8fc_1773_8f74_dc15);
    }

    #[test]
    fn data_rom_command_streams_known_prefix() {
        let mut dsp = Dsp3::default();
        start_command(&mut dsp, 0x1f);
        assert_eq!(read_word(&mut dsp), 0x001f);
        assert_eq!(read_word(&mut dsp), 0x8000);
        assert_eq!(read_word(&mut dsp), 0x4000);
        assert_eq!(read_word(&mut dsp), 0x2000);
        assert_eq!(read_word(&mut dsp), 0x1000);
    }

    #[test]
    fn planar_conversion_transposes_eight_pixels_into_bitplanes() {
        let mut dsp = Dsp3::default();
        start_command(&mut dsp, 0x18);
        write_word(&mut dsp, 1);
        for value in [0x0100u16, 0x0402, 0x1008, 0x4020] {
            write_word(&mut dsp, value);
        }
        assert_eq!(read_word(&mut dsp), 0x2040);
        assert_eq!(read_word(&mut dsp), 0x0810);
        assert_eq!(read_word(&mut dsp), 0x0204);
        assert_eq!(read_word(&mut dsp), 0x0001);
        assert_eq!(dsp.read(0xc000), 0x84);
    }

    #[test]
    fn decode_command_consumes_symbol_tree_and_emits_literal() {
        let mut dsp = Dsp3::default();
        start_command(&mut dsp, 0x38);
        write_word(&mut dsp, 1);
        write_word(&mut dsp, 1);
        assert_eq!(dsp.read(0xc000), 0xc0);

        let bits = [
            0u8, 0, // literal symbol command
            0, 0, 0, 0, 0, 0, 1, 0, 1, // symbol 5
            0, // two-bit prefix tree
            0, 0, 0, // length 1
            0, 0, 0, // length 1
            0, 0, 0, // length 1
            0, 0, 0, // length 1
            0, 0, // prefix 0
            0, // suffix 0
        ];
        let mut words = [0u16; 2];
        for (index, bit) in bits.into_iter().enumerate() {
            if bit != 0 {
                words[index / 16] |= 1 << (15 - (index % 16));
            }
        }
        write_word(&mut dsp, words[0]);
        assert_eq!(dsp.read(0xc000), 0xc0);
        write_word(&mut dsp, words[1]);
        assert_eq!(dsp.read(0xc000), 0x80);
        assert_eq!(read_word(&mut dsp), 5);
        assert_eq!(dsp.read(0xc000), 0x84);
    }

    #[test]
    fn mapping_state_round_trips_mid_command() {
        let mut dsp = Dsp3::default();
        start_command(&mut dsp, 0x06);
        write_word(&mut dsp, 0x0810);
        dsp.write(0x8000, 0x07);
        write_word(&mut dsp, 2);
        write_word(&mut dsp, 0x0304);

        let mut out = StateWriter::new(PlatformId::Snes, 1);
        dsp.save(&mut out);
        let state = out.finish();

        let expected_status = dsp.read(0xc000);
        let expected = read_word(&mut dsp);

        let mut restored = Dsp3::default();
        let mut input = StateReader::new(&state, PlatformId::Snes, 1).unwrap();
        restored.load(&mut input).unwrap();
        input.finish().unwrap();

        assert_eq!(restored.read(0xc000), expected_status);
        assert_eq!(read_word(&mut restored), expected);
    }
}
