use crate::state::{StateReader, StateWriter};

pub trait RspBus {
    fn read8(&mut self, address: u32) -> u8;
    fn write8(&mut self, address: u32, value: u8);
    fn read_cop0(&mut self, index: u8) -> u32;
    fn write_cop0(&mut self, index: u8, value: u32);

    fn read16(&mut self, address: u32) -> u16 {
        u16::from_be_bytes([self.read8(address), self.read8(address.wrapping_add(1))])
    }
    fn read32(&mut self, address: u32) -> u32 {
        u32::from_be_bytes([
            self.read8(address),
            self.read8(address.wrapping_add(1)),
            self.read8(address.wrapping_add(2)),
            self.read8(address.wrapping_add(3)),
        ])
    }
    fn write16(&mut self, address: u32, value: u16) {
        for (offset, byte) in value.to_be_bytes().into_iter().enumerate() {
            self.write8(address.wrapping_add(offset as u32), byte);
        }
    }
    fn write32(&mut self, address: u32, value: u32) {
        for (offset, byte) in value.to_be_bytes().into_iter().enumerate() {
            self.write8(address.wrapping_add(offset as u32), byte);
        }
    }
}

const ELEMENT_LANES: [[usize; 8]; 16] = [
    [0, 1, 2, 3, 4, 5, 6, 7],
    [0, 1, 2, 3, 4, 5, 6, 7],
    [0, 0, 2, 2, 4, 4, 6, 6],
    [1, 1, 3, 3, 5, 5, 7, 7],
    [0, 0, 0, 0, 4, 4, 4, 4],
    [1, 1, 1, 1, 5, 5, 5, 5],
    [2, 2, 2, 2, 6, 6, 6, 6],
    [3, 3, 3, 3, 7, 7, 7, 7],
    [0; 8],
    [1; 8],
    [2; 8],
    [3; 8],
    [4; 8],
    [5; 8],
    [6; 8],
    [7; 8],
];

const RSP_RECIPROCAL_ROM: [u16; 512] = build_rsp_reciprocal_rom();
const RSP_INVERSE_SQRT_ROM: [u16; 512] = build_rsp_inverse_sqrt_rom();

const fn build_rsp_reciprocal_rom() -> [u16; 512] {
    let mut table = [0u16; 512];
    table[0] = u16::MAX;
    let mut index = 1usize;
    while index < table.len() {
        let divisor = index as u64 + 512;
        let quotient = (1u64 << 34) / divisor;
        table[index] = ((quotient + 1) >> 8) as u16;
        index += 1;
    }
    table
}

const fn build_rsp_inverse_sqrt_rom() -> [u16; 512] {
    let mut table = [0u16; 512];
    let mut index = 0usize;
    while index < table.len() {
        let divisor = (index as u64 + 512) >> ((index & 1) as u32);
        let mut low = 1u64 << 17;
        let mut high = 1u64 << 19;
        while low < high {
            let middle = low + (high - low) / 2;
            if divisor * (middle + 1) * (middle + 1) >= (1u64 << 44) {
                high = middle;
            } else {
                low = middle + 1;
            }
        }
        table[index] = (low >> 1) as u16;
        index += 1;
    }
    table
}
#[derive(Debug, Clone)]
pub struct Rsp {
    pub regs: [u32; 32],
    pub vectors: [[u16; 8]; 32],
    pub pc: u32,
    pub next_pc: u32,
    pub cycles: u64,
    pub running: bool,
    pub broke: bool,
    acc: [i64; 8],
    vco: u16,
    vcc: u16,
    vce: u8,
    pending_branch: Option<u32>,
    branch_delay: bool,
    div_in: u32,
    div_out: u32,
    div_pending: bool,
}

impl Default for Rsp {
    fn default() -> Self {
        Self::new()
    }
}
impl Rsp {
    pub fn new() -> Self {
        Self {
            regs: [0; 32],
            vectors: [[0; 8]; 32],
            pc: 0,
            next_pc: 4,
            cycles: 0,
            running: false,
            broke: false,
            acc: [0; 8],
            vco: 0,
            vcc: 0,
            vce: 0,
            pending_branch: None,
            branch_delay: false,
            div_in: 0,
            div_out: 0,
            div_pending: false,
        }
    }

    pub fn reset(&mut self) {
        *self = Self::new();
    }
    pub fn set_pc(&mut self, value: u32) {
        self.pc = value & 0x0ffc;
        self.next_pc = self.pc.wrapping_add(4) & 0x0ffc;
        self.pending_branch = None;
        self.branch_delay = false;
    }

    pub fn flags(&self) -> (u16, u16, u8) {
        (self.vco, self.vcc, self.vce)
    }

    fn reg(&self, index: u8) -> u32 {
        self.regs[usize::from(index)]
    }
    fn write_reg(&mut self, index: u8, value: u32) {
        if index != 0 {
            self.regs[usize::from(index)] = value;
        }
    }
    fn rs(instruction: u32) -> u8 {
        ((instruction >> 21) & 31) as u8
    }
    fn rt(instruction: u32) -> u8 {
        ((instruction >> 16) & 31) as u8
    }
    fn rd(instruction: u32) -> u8 {
        ((instruction >> 11) & 31) as u8
    }
    fn simm(instruction: u32) -> u32 {
        instruction as u16 as i16 as i32 as u32
    }
    fn vector_byte(&self, reg: usize, byte: usize) -> u8 {
        let lane = (byte & 15) >> 1;
        let word = self.vectors[reg & 31][lane];
        if byte & 1 == 0 {
            (word >> 8) as u8
        } else {
            word as u8
        }
    }
    fn set_vector_byte(&mut self, reg: usize, byte: usize, value: u8) {
        let lane = (byte & 15) >> 1;
        let word = &mut self.vectors[reg & 31][lane];
        if byte & 1 == 0 {
            *word = (*word & 0x00ff) | (u16::from(value) << 8);
        } else {
            *word = (*word & 0xff00) | u16::from(value);
        }
    }
    fn vt_lane(&self, vt: usize, element: usize, lane: usize) -> u16 {
        self.vectors[vt & 31][ELEMENT_LANES[element & 15][lane & 7]]
    }

    fn vector_sources(&self, vs: usize, vt: usize, element: usize) -> ([u16; 8], [u16; 8]) {
        let left = self.vectors[vs & 31];
        let right = core::array::from_fn(|lane| self.vt_lane(vt, element, lane));
        (left, right)
    }

    fn wrap_acc(value: i64) -> i64 {
        let raw = value & 0x0000_ffff_ffff_ffff;
        if raw & 0x0000_8000_0000_0000 != 0 {
            raw | !0x0000_ffff_ffff_ffff
        } else {
            raw
        }
    }
    fn signed_sat(value: i64) -> u16 {
        value.clamp(i64::from(i16::MIN), i64::from(i16::MAX)) as i16 as u16
    }
    fn unsigned_sat(value: i64) -> u16 {
        if value < 0 {
            0
        } else if value > i64::from(i16::MAX) {
            u16::MAX
        } else {
            value as u16
        }
    }

    fn extract_low(acc: i64) -> u16 {
        let middle = acc >> 16;
        if middle > i64::from(i16::MAX) {
            u16::MAX
        } else if middle < i64::from(i16::MIN) {
            0
        } else {
            acc as u16
        }
    }

    fn branch(&mut self, instruction: u32, take: bool) {
        if take {
            let offset = (instruction as u16 as i16 as i32 as u32).wrapping_shl(2);
            self.pending_branch = Some(self.pc.wrapping_add(offset) & 0x0ffc);
            self.branch_delay = true;
        }
    }

    pub fn step<B: RspBus>(&mut self, bus: &mut B) -> u32 {
        if !self.running {
            return 0;
        }
        let instruction = bus.read32(0x1000 | (self.pc & 0x0ffc));
        let apply_branch = self.branch_delay;
        let branch_target = self.pending_branch.take();
        self.branch_delay = false;
        self.pc = self.next_pc;
        self.next_pc = self.next_pc.wrapping_add(4) & 0x0ffc;
        self.execute(bus, instruction);
        if apply_branch {
            if let Some(target) = branch_target {
                self.pc = target;
                self.next_pc = target.wrapping_add(4) & 0x0ffc;
            }
        }
        self.regs[0] = 0;
        self.cycles = self.cycles.wrapping_add(1);
        1
    }
    fn execute<B: RspBus>(&mut self, bus: &mut B, instruction: u32) {
        match instruction >> 26 {
            0x00 => self.special(instruction),
            0x01 => self.regimm(instruction),
            0x02 => self.jump(instruction, false),
            0x03 => self.jump(instruction, true),
            0x04 => self.branch(
                instruction,
                self.reg(Self::rs(instruction)) == self.reg(Self::rt(instruction)),
            ),
            0x05 => self.branch(
                instruction,
                self.reg(Self::rs(instruction)) != self.reg(Self::rt(instruction)),
            ),
            0x06 => self.branch(instruction, (self.reg(Self::rs(instruction)) as i32) <= 0),
            0x07 => self.branch(instruction, (self.reg(Self::rs(instruction)) as i32) > 0),
            0x08 | 0x09 => self.write_reg(
                Self::rt(instruction),
                self.reg(Self::rs(instruction))
                    .wrapping_add(Self::simm(instruction)),
            ),
            0x0a => self.write_reg(
                Self::rt(instruction),
                ((self.reg(Self::rs(instruction)) as i32) < (Self::simm(instruction) as i32))
                    as u32,
            ),
            0x0b => self.write_reg(
                Self::rt(instruction),
                (self.reg(Self::rs(instruction)) < Self::simm(instruction)) as u32,
            ),
            0x0c => self.write_reg(
                Self::rt(instruction),
                self.reg(Self::rs(instruction)) & u32::from(instruction as u16),
            ),
            0x0d => self.write_reg(
                Self::rt(instruction),
                self.reg(Self::rs(instruction)) | u32::from(instruction as u16),
            ),
            0x0e => self.write_reg(
                Self::rt(instruction),
                self.reg(Self::rs(instruction)) ^ u32::from(instruction as u16),
            ),
            0x0f => self.write_reg(Self::rt(instruction), u32::from(instruction as u16) << 16),
            0x10 => self.cop0(bus, instruction),
            0x12 => self.cop2(instruction),
            0x20..=0x25 => self.load(bus, instruction),
            0x28 | 0x29 | 0x2b => self.store(bus, instruction),
            0x32 => self.vector_load(bus, instruction),
            0x3a => self.vector_store(bus, instruction),
            _ => {}
        }
    }
    fn special(&mut self, instruction: u32) {
        let rs = Self::rs(instruction);
        let rt = Self::rt(instruction);
        let rd = Self::rd(instruction);
        let sa = (instruction >> 6) & 31;
        let left = self.reg(rs);
        let right = self.reg(rt);
        match instruction & 63 {
            0x00 => self.write_reg(rd, right << sa),
            0x02 => self.write_reg(rd, right >> sa),
            0x03 => self.write_reg(rd, ((right as i32) >> sa) as u32),
            0x04 => self.write_reg(rd, right << (left & 31)),
            0x06 => self.write_reg(rd, right >> (left & 31)),
            0x07 => self.write_reg(rd, ((right as i32) >> (left & 31)) as u32),
            0x08 => {
                self.pending_branch = Some(left & 0x0ffc);
                self.branch_delay = true;
            }
            0x09 => {
                self.write_reg(if rd == 0 { 31 } else { rd }, self.next_pc);
                self.pending_branch = Some(left & 0x0ffc);
                self.branch_delay = true;
            }
            0x0d => {
                self.running = false;
                self.broke = true;
            }
            0x20 | 0x21 => self.write_reg(rd, left.wrapping_add(right)),
            0x22 | 0x23 => self.write_reg(rd, left.wrapping_sub(right)),
            0x24 => self.write_reg(rd, left & right),
            0x25 => self.write_reg(rd, left | right),
            0x26 => self.write_reg(rd, left ^ right),
            0x27 => self.write_reg(rd, !(left | right)),
            0x2a => self.write_reg(rd, ((left as i32) < (right as i32)) as u32),
            0x2b => self.write_reg(rd, (left < right) as u32),
            _ => {}
        }
    }

    fn regimm(&mut self, instruction: u32) {
        let value = self.reg(Self::rs(instruction)) as i32;
        match Self::rt(instruction) {
            0x00 => self.branch(instruction, value < 0),
            0x01 => self.branch(instruction, value >= 0),
            0x10 => {
                self.write_reg(31, self.next_pc);
                self.branch(instruction, value < 0);
            }
            0x11 => {
                self.write_reg(31, self.next_pc);
                self.branch(instruction, value >= 0);
            }
            _ => {}
        }
    }
    fn jump(&mut self, instruction: u32, link: bool) {
        if link {
            self.write_reg(31, self.next_pc);
        }
        self.pending_branch = Some((instruction << 2) & 0x0ffc);
        self.branch_delay = true;
    }
    fn cop0<B: RspBus>(&mut self, bus: &mut B, instruction: u32) {
        let rt = Self::rt(instruction);
        let rd = Self::rd(instruction);
        match Self::rs(instruction) {
            0x00 => {
                let value = bus.read_cop0(rd);
                self.write_reg(rt, value);
            }
            0x04 => bus.write_cop0(rd, self.reg(rt)),
            _ => {}
        }
    }

    fn cop2(&mut self, instruction: u32) {
        let rs = Self::rs(instruction);
        if rs >= 0x10 {
            self.vector_compute(instruction);
            return;
        }
        let rt = Self::rt(instruction);
        let vd = Self::rd(instruction) as usize;
        let element = ((instruction >> 7) & 15) as usize;
        match rs {
            0x00 => {
                let high = self.vector_byte(vd, element);
                let low = self.vector_byte(vd, element + 1);
                self.write_reg(rt, i16::from_be_bytes([high, low]) as i32 as u32);
            }
            0x02 => {
                let value = match vd & 3 {
                    0 => self.vco,
                    1 => self.vcc,
                    2 => u16::from(self.vce),
                    _ => 0,
                };
                self.write_reg(rt, i16::from_ne_bytes(value.to_ne_bytes()) as i32 as u32);
            }
            0x04 => {
                let bytes = (self.reg(rt) as u16).to_be_bytes();
                self.set_vector_byte(vd, element, bytes[0]);
                if element != 15 {
                    self.set_vector_byte(vd, element + 1, bytes[1]);
                }
            }
            0x06 => match vd & 3 {
                0 => self.vco = self.reg(rt) as u16,
                1 => self.vcc = self.reg(rt) as u16,
                2 => self.vce = self.reg(rt) as u8,
                _ => {}
            },
            _ => {}
        }
    }

    fn read_dmem16<B: RspBus>(bus: &mut B, address: u32) -> u16 {
        u16::from_be_bytes([
            bus.read8(address & 0x0fff),
            bus.read8(address.wrapping_add(1) & 0x0fff),
        ])
    }

    fn read_dmem32<B: RspBus>(bus: &mut B, address: u32) -> u32 {
        u32::from_be_bytes([
            bus.read8(address & 0x0fff),
            bus.read8(address.wrapping_add(1) & 0x0fff),
            bus.read8(address.wrapping_add(2) & 0x0fff),
            bus.read8(address.wrapping_add(3) & 0x0fff),
        ])
    }

    fn write_dmem16<B: RspBus>(bus: &mut B, address: u32, value: u16) {
        for (offset, byte) in value.to_be_bytes().into_iter().enumerate() {
            bus.write8(address.wrapping_add(offset as u32) & 0x0fff, byte);
        }
    }

    fn write_dmem32<B: RspBus>(bus: &mut B, address: u32, value: u32) {
        for (offset, byte) in value.to_be_bytes().into_iter().enumerate() {
            bus.write8(address.wrapping_add(offset as u32) & 0x0fff, byte);
        }
    }

    fn load<B: RspBus>(&mut self, bus: &mut B, instruction: u32) {
        let address = self
            .reg(Self::rs(instruction))
            .wrapping_add(Self::simm(instruction))
            & 0x0fff;
        let rt = Self::rt(instruction);
        let value = match instruction >> 26 {
            0x20 => bus.read8(address) as i8 as i32 as u32,
            0x21 => Self::read_dmem16(bus, address) as i16 as i32 as u32,
            0x23 => Self::read_dmem32(bus, address),
            0x24 => u32::from(bus.read8(address)),
            0x25 => u32::from(Self::read_dmem16(bus, address)),
            _ => return,
        };
        self.write_reg(rt, value);
    }

    fn store<B: RspBus>(&mut self, bus: &mut B, instruction: u32) {
        let address = self
            .reg(Self::rs(instruction))
            .wrapping_add(Self::simm(instruction))
            & 0x0fff;
        let value = self.reg(Self::rt(instruction));
        match instruction >> 26 {
            0x28 => bus.write8(address, value as u8),
            0x29 => Self::write_dmem16(bus, address, value as u16),
            0x2b => Self::write_dmem32(bus, address, value),
            _ => {}
        }
    }

    fn vector_address(&self, instruction: u32, scale: u32) -> u32 {
        let base = self.reg(Self::rs(instruction));
        let raw = (instruction & 0x7f) as u8;
        let offset = ((raw << 1) as i8 >> 1) as i32;
        base.wrapping_add((offset * scale as i32) as u32) & 0x0fff
    }

    fn vector_load<B: RspBus>(&mut self, bus: &mut B, instruction: u32) {
        let vt = Self::rt(instruction) as usize;
        let op = ((instruction >> 11) & 31) as u8;
        let element = ((instruction >> 7) & 15) as usize;
        match op {
            0..=3 => {
                let size = 1usize << op;
                let address = self.vector_address(instruction, size as u32);
                let count = size.min(16 - element);
                for index in 0..count {
                    let value = bus.read8((address.wrapping_add(index as u32)) & 0x0fff);
                    self.set_vector_byte(vt, element + index, value);
                }
            }
            4 => {
                let address = self.vector_address(instruction, 16);
                let count = (16 - (address as usize & 15)).min(16 - element);
                for index in 0..count {
                    let value = bus.read8((address.wrapping_add(index as u32)) & 0x0fff);
                    self.set_vector_byte(vt, element + index, value);
                }
            }
            5 => {
                let address = self.vector_address(instruction, 16);
                let count = address as usize & 15;
                let source = address & !0x0f;
                if element < count {
                    let count = count - element;
                    let target = 16 - (address as usize & 15) + element;
                    for index in 0..count {
                        let value = bus.read8((source.wrapping_add(index as u32)) & 0x0fff);
                        self.set_vector_byte(vt, target + index, value);
                    }
                }
            }
            6 | 7 => self.vector_load_packed(bus, instruction, vt, op, element),
            8 => self.vector_load_half(bus, instruction, vt, element),
            9 => self.vector_load_fractional(bus, instruction, vt, element),
            10 => {}
            11 => self.vector_load_transpose(bus, instruction, vt, element),
            _ => {}
        }
    }

    fn vector_store<B: RspBus>(&mut self, bus: &mut B, instruction: u32) {
        let vt = Self::rt(instruction) as usize;
        let op = ((instruction >> 11) & 31) as u8;
        let element = ((instruction >> 7) & 15) as usize;
        match op {
            0..=3 => {
                let size = 1usize << op;
                let address = self.vector_address(instruction, size as u32);
                for index in 0..size {
                    let value = self.vector_byte(vt, element + index);
                    bus.write8((address.wrapping_add(index as u32)) & 0x0fff, value);
                }
            }
            4 => {
                let address = self.vector_address(instruction, 16);
                let count = 16 - (address as usize & 15);
                for index in 0..count {
                    let value = self.vector_byte(vt, element + index);
                    bus.write8((address.wrapping_add(index as u32)) & 0x0fff, value);
                }
            }
            5 => {
                let address = self.vector_address(instruction, 16);
                let count = address as usize & 15;
                let target = address & !0x0f;
                let source = 16 - count;
                for index in 0..count {
                    let value = self.vector_byte(vt, element + source + index);
                    bus.write8((target.wrapping_add(index as u32)) & 0x0fff, value);
                }
            }
            6 | 7 => self.vector_store_packed(bus, instruction, vt, op, element),
            8 => self.vector_store_half(bus, instruction, vt, element),
            9 => self.vector_store_fractional(bus, instruction, vt, element),
            10 => self.vector_store_word(bus, instruction, vt, element),
            11 => self.vector_store_transpose(bus, instruction, vt, element),
            _ => {}
        }
    }

    fn vector_load_packed<B: RspBus>(
        &mut self,
        bus: &mut B,
        instruction: u32,
        vt: usize,
        op: u8,
        element: usize,
    ) {
        let base = self.vector_address(instruction, 8);
        let address = base & !7;
        let index = (base & 7).wrapping_sub(element as u32);
        let shift = if op == 7 { 7 } else { 8 };
        for lane in 0..8u32 {
            let byte = index.wrapping_add(lane) & 15;
            let value = u16::from(bus.read8((address.wrapping_add(byte)) & 0x0fff));
            self.vectors[vt][lane as usize] = value << shift;
        }
    }

    fn vector_store_packed<B: RspBus>(
        &mut self,
        bus: &mut B,
        instruction: u32,
        vt: usize,
        op: u8,
        element: usize,
    ) {
        let address = self.vector_address(instruction, 8);
        let shifted = op == 7;
        for index in 0..8u32 {
            let offset = element as u32 + index;
            let lane = (offset & 7) as usize;
            let use_high_byte = (offset & 15 < 8) != shifted;
            let value = if use_high_byte {
                self.vector_byte(vt, lane << 1)
            } else {
                (self.vectors[vt][lane] >> 7) as u8
            };
            bus.write8((address.wrapping_add(index)) & 0x0fff, value);
        }
    }

    fn vector_load_half<B: RspBus>(
        &mut self,
        bus: &mut B,
        instruction: u32,
        vt: usize,
        element: usize,
    ) {
        let base = self.vector_address(instruction, 16);
        let address = base & !7;
        let index = (base & 7).wrapping_sub(element as u32);
        for lane in 0..8u32 {
            let byte = index.wrapping_add(lane * 2) & 15;
            let value = u16::from(bus.read8((address.wrapping_add(byte)) & 0x0fff));
            self.vectors[vt][lane as usize] = value << 7;
        }
    }

    fn vector_store_half<B: RspBus>(
        &mut self,
        bus: &mut B,
        instruction: u32,
        vt: usize,
        element: usize,
    ) {
        let base = self.vector_address(instruction, 16);
        let address = base & !7;
        let index = base & 7;
        for lane in 0..8u32 {
            let byte = element + lane as usize * 2;
            let high = u16::from(self.vector_byte(vt, byte & 15));
            let low = u16::from(self.vector_byte(vt, (byte + 1) & 15));
            let value = ((high << 1) | (low >> 7)) as u8;
            let target = index.wrapping_add(lane * 2) & 15;
            bus.write8((address.wrapping_add(target)) & 0x0fff, value);
        }
    }

    fn vector_load_fractional<B: RspBus>(
        &mut self,
        bus: &mut B,
        instruction: u32,
        vt: usize,
        element: usize,
    ) {
        let address = self.vector_address(instruction, 16);
        let aligned = address & !7;
        let misalignment = (address & 7) as i32;
        let element_i32 = element as i32;
        let offsets = [
            -element_i32,
            4 - element_i32,
            8 - element_i32,
            12 - element_i32,
            8 - element_i32,
            12 - element_i32,
            -element_i32,
            4 - element_i32,
        ];
        let mut temp = [0u16; 8];
        for (lane, offset) in offsets.into_iter().enumerate() {
            let byte = (misalignment + offset).rem_euclid(16) as u32;
            temp[lane] = u16::from(bus.read8((aligned.wrapping_add(byte)) & 0x0fff)) << 7;
        }
        let mut bytes = [0u8; 16];
        for (lane, value) in temp.into_iter().enumerate() {
            let [high, low] = value.to_be_bytes();
            bytes[lane * 2] = high;
            bytes[lane * 2 + 1] = low;
        }
        let count = 8.min(16 - element);
        for (byte, value) in bytes.into_iter().enumerate().skip(element).take(count) {
            self.set_vector_byte(vt, byte, value);
        }
    }

    fn vector_store_fractional<B: RspBus>(
        &mut self,
        bus: &mut B,
        instruction: u32,
        vt: usize,
        element: usize,
    ) {
        let address = self.vector_address(instruction, 16);
        let lanes = match element {
            0 | 15 => Some([0, 1, 2, 3]),
            1 => Some([6, 7, 4, 5]),
            4 => Some([1, 2, 3, 0]),
            5 => Some([7, 4, 5, 6]),
            8 => Some([4, 5, 6, 7]),
            11 => Some([3, 0, 1, 2]),
            12 => Some([5, 6, 7, 4]),
            _ => None,
        };
        let misalignment = address & 7;
        let aligned = address & !7;
        for index in 0..4u32 {
            let value = lanes.map_or(0, |source| {
                (self.vectors[vt][source[index as usize]] >> 7) as u8
            });
            let target = aligned.wrapping_add((misalignment + (index << 2)) & 15);
            bus.write8(target & 0x0fff, value);
        }
    }

    fn vector_store_word<B: RspBus>(
        &mut self,
        bus: &mut B,
        instruction: u32,
        vt: usize,
        element: usize,
    ) {
        let address = self.vector_address(instruction, 16);
        let aligned = address & !7;
        let misalignment = address & 7;
        for index in 0..16u32 {
            let target = aligned.wrapping_add((misalignment + index) & 15);
            let value = self.vector_byte(vt, element + index as usize);
            bus.write8(target & 0x0fff, value);
        }
    }

    fn vector_load_transpose<B: RspBus>(
        &mut self,
        bus: &mut B,
        instruction: u32,
        vt: usize,
        element: usize,
    ) {
        let address = self.vector_address(instruction, 16);
        let begin = address & !7;
        let mut pointer = begin + ((element as u32 + (address & 8)) & 15);
        let register_base = vt & !7;
        let mut register_offset = element >> 1;
        for lane in 0..8usize {
            for half in 0..2usize {
                let value = bus.read8(pointer & 0x0fff);
                self.set_vector_byte(register_base + register_offset, lane * 2 + half, value);
                pointer += 1;
                if pointer == begin + 16 {
                    pointer = begin;
                }
            }
            register_offset = (register_offset + 1) & 7;
        }
    }

    fn vector_store_transpose<B: RspBus>(
        &mut self,
        bus: &mut B,
        instruction: u32,
        vt: usize,
        element: usize,
    ) {
        let address = self.vector_address(instruction, 16);
        let register_base = vt & !7;
        let mut source_byte = 16 - (element & !1);
        let mut target_offset = (address & 7).wrapping_sub((element & !1) as u32);
        let aligned = address & !7;
        for register in register_base..register_base + 8 {
            for _ in 0..2 {
                let value = self.vector_byte(register, source_byte & 15);
                bus.write8(aligned.wrapping_add(target_offset & 15) & 0x0fff, value);
                target_offset = target_offset.wrapping_add(1);
                source_byte += 1;
            }
        }
    }

    fn vector_compute(&mut self, instruction: u32) {
        let element = ((instruction >> 21) & 15) as usize;
        let vt = ((instruction >> 16) & 31) as usize;
        let vs = ((instruction >> 11) & 31) as usize;
        let vd = ((instruction >> 6) & 31) as usize;
        match instruction & 63 {
            0x00 => self.vmul(vd, vs, vt, element, true),
            0x01 => self.vmulu(vd, vs, vt, element),
            0x02 => self.vrnd(vd, vs, vt, element, true),
            0x03 => self.vmulq(vd, vs, vt, element),
            0x04 => self.vmudl(vd, vs, vt, element, false),
            0x05 => self.vmudm(vd, vs, vt, element, false),
            0x06 => self.vmudn(vd, vs, vt, element, false),
            0x07 => self.vmudh(vd, vs, vt, element, false),
            0x08 => self.vmul(vd, vs, vt, element, false),
            0x09 => self.vmacu(vd, vs, vt, element),
            0x0a => self.vrnd(vd, vs, vt, element, false),
            0x0b => self.vmacq(vd),
            0x0c => self.vmudl(vd, vs, vt, element, true),
            0x0d => self.vmudm(vd, vs, vt, element, true),
            0x0e => self.vmudn(vd, vs, vt, element, true),
            0x0f => self.vmudh(vd, vs, vt, element, true),
            0x10 => self.vadd(vd, vs, vt, element, false),
            0x11 => self.vsub(vd, vs, vt, element),
            0x13 => self.vabs(vd, vs, vt, element),
            0x14 => self.vadd(vd, vs, vt, element, true),
            0x15 => self.vsubc(vd, vs, vt, element),
            0x12 | 0x16..=0x1c | 0x1e | 0x1f => self.vzero(vd, vs, vt, element),
            0x1d => self.vsaw(vd, element),
            0x20 => self.vcompare(vd, vs, vt, element, 0),
            0x21 => self.vcompare(vd, vs, vt, element, 1),
            0x22 => self.vcompare(vd, vs, vt, element, 2),
            0x23 => self.vcompare(vd, vs, vt, element, 3),
            0x24..=0x26 => self.vclip(vd, vs, vt, element, (instruction & 63) as u8),
            0x27 => self.vmerge(vd, vs, vt, element),
            0x28 => self.vlogic(vd, vs, vt, element, 0),
            0x29 => self.vlogic(vd, vs, vt, element, 1),
            0x2a => self.vlogic(vd, vs, vt, element, 2),
            0x2b => self.vlogic(vd, vs, vt, element, 3),
            0x2c => self.vlogic(vd, vs, vt, element, 4),
            0x2d => self.vlogic(vd, vs, vt, element, 5),
            0x2e | 0x2f => self.vzero(vd, vs, vt, element),
            0x30..=0x36 => self.vscalar(vd, vs, vt, element, (instruction & 63) as u8),
            0x38..=0x3e => self.vzero(vd, vs, vt, element),
            0x37 => {}
            _ => {}
        }
    }
    fn vmul(&mut self, vd: usize, vs: usize, vt: usize, e: usize, initialize: bool) {
        let (left, right) = self.vector_sources(vs, vt, e);
        for lane in 0..8 {
            let product = i64::from(left[lane] as i16) * i64::from(right[lane] as i16) * 2;
            self.acc[lane] = if initialize {
                Self::wrap_acc(product + 0x8000)
            } else {
                Self::wrap_acc(self.acc[lane] + product)
            };
            self.vectors[vd][lane] = Self::signed_sat(self.acc[lane] >> 16);
        }
    }

    fn vmulu(&mut self, vd: usize, vs: usize, vt: usize, e: usize) {
        let (left, right) = self.vector_sources(vs, vt, e);
        for lane in 0..8 {
            let product = i64::from(left[lane] as i16) * i64::from(right[lane] as i16) * 2;
            self.acc[lane] = Self::wrap_acc(product + 0x8000);
            self.vectors[vd][lane] = Self::unsigned_sat(self.acc[lane] >> 16);
        }
    }

    fn vmulq(&mut self, vd: usize, vs: usize, vt: usize, e: usize) {
        let (left, right) = self.vector_sources(vs, vt, e);
        for lane in 0..8 {
            let mut product = i64::from(left[lane] as i16) * i64::from(right[lane] as i16);
            if product < 0 {
                product += 31;
            }
            self.acc[lane] = Self::wrap_acc(product << 16);
            self.vectors[vd][lane] = Self::signed_sat(product >> 1) & !0x000f;
        }
    }

    fn vmacu(&mut self, vd: usize, vs: usize, vt: usize, e: usize) {
        let (left, right) = self.vector_sources(vs, vt, e);
        for lane in 0..8 {
            let product = i64::from(left[lane] as i16) * i64::from(right[lane] as i16) * 2;
            self.acc[lane] = Self::wrap_acc(self.acc[lane] + product);
            self.vectors[vd][lane] = Self::unsigned_sat(self.acc[lane] >> 16);
        }
    }

    fn vrnd(&mut self, vd: usize, vs_field: usize, vt: usize, e: usize, positive: bool) {
        let right: [u16; 8] = core::array::from_fn(|lane| self.vt_lane(vt, e, lane));
        let shift = vs_field & 1 != 0;
        for (lane, &value) in right.iter().enumerate() {
            let mut term = i64::from(value as i16);
            if shift {
                term <<= 16;
            }
            let mut acc = self.acc[lane];
            if (positive && acc >= 0) || (!positive && acc < 0) {
                acc = Self::wrap_acc(acc + term);
            }
            self.acc[lane] = acc;
            self.vectors[vd][lane] = Self::signed_sat(acc >> 16);
        }
    }

    fn vmacq(&mut self, vd: usize) {
        for lane in 0..8 {
            let acc = self.acc[lane];
            let adjusted = if acc & 0x20_0000 == 0 {
                match (acc >> 22).cmp(&0) {
                    core::cmp::Ordering::Less => acc + 0x20_0000,
                    core::cmp::Ordering::Greater => acc - 0x20_0000,
                    core::cmp::Ordering::Equal => acc,
                }
            } else {
                acc
            };
            self.acc[lane] = Self::wrap_acc(adjusted);
            let clamped = if adjusted < 0 {
                if (!adjusted) >> 32 != 0 {
                    0x8000
                } else {
                    (adjusted >> 17) as u16
                }
            } else if adjusted >> 32 != 0 {
                0x7fff
            } else {
                (adjusted >> 17) as u16
            };
            self.vectors[vd][lane] = clamped & 0xfff0;
        }
    }

    fn vmudl(&mut self, vd: usize, vs: usize, vt: usize, e: usize, accumulate: bool) {
        let (left, right) = self.vector_sources(vs, vt, e);
        for lane in 0..8 {
            let product = u32::from(left[lane]) * u32::from(right[lane]);
            let term = i64::from(product >> 16);
            self.acc[lane] = if accumulate {
                Self::wrap_acc(self.acc[lane] + term)
            } else {
                term
            };
            self.vectors[vd][lane] = if accumulate {
                Self::extract_low(self.acc[lane])
            } else {
                self.acc[lane] as u16
            };
        }
    }

    fn vmudm(&mut self, vd: usize, vs: usize, vt: usize, e: usize, accumulate: bool) {
        let (left, right) = self.vector_sources(vs, vt, e);
        for lane in 0..8 {
            let product = i64::from(left[lane] as i16) * i64::from(right[lane]);
            self.acc[lane] = if accumulate {
                Self::wrap_acc(self.acc[lane] + product)
            } else {
                Self::wrap_acc(product)
            };
            self.vectors[vd][lane] = if accumulate {
                Self::signed_sat(self.acc[lane] >> 16)
            } else {
                (self.acc[lane] >> 16) as u16
            };
        }
    }

    fn vmudn(&mut self, vd: usize, vs: usize, vt: usize, e: usize, accumulate: bool) {
        let (left, right) = self.vector_sources(vs, vt, e);
        for lane in 0..8 {
            let product = i64::from(left[lane]) * i64::from(right[lane] as i16);
            self.acc[lane] = if accumulate {
                Self::wrap_acc(self.acc[lane] + product)
            } else {
                Self::wrap_acc(product)
            };
            self.vectors[vd][lane] = if accumulate {
                Self::extract_low(self.acc[lane])
            } else {
                self.acc[lane] as u16
            };
        }
    }

    fn vmudh(&mut self, vd: usize, vs: usize, vt: usize, e: usize, accumulate: bool) {
        let (left, right) = self.vector_sources(vs, vt, e);
        for lane in 0..8 {
            let product = (i64::from(left[lane] as i16) * i64::from(right[lane] as i16)) << 16;
            self.acc[lane] = if accumulate {
                Self::wrap_acc(self.acc[lane] + product)
            } else {
                Self::wrap_acc(product)
            };
            self.vectors[vd][lane] = Self::signed_sat(self.acc[lane] >> 16);
        }
    }

    fn vadd(&mut self, vd: usize, vs: usize, vt: usize, e: usize, set_carry: bool) {
        let (left, right) = self.vector_sources(vs, vt, e);
        let incoming = self.vco;
        let mut carry = 0u16;
        for lane in 0..8 {
            if set_carry {
                let sum = u32::from(left[lane]) + u32::from(right[lane]);
                let low = sum as u16;
                self.set_acc_low(lane, low);
                self.vectors[vd][lane] = low;
                if sum > 0xffff {
                    carry |= 1 << lane;
                }
            } else {
                let add = i32::from((incoming >> lane) & 1);
                let sum = i32::from(left[lane] as i16) + i32::from(right[lane] as i16) + add;
                self.set_acc_low(lane, sum as u16);
                self.vectors[vd][lane] = Self::signed_sat(i64::from(sum));
            }
        }
        self.vco = if set_carry { carry } else { 0 };
    }
    fn vsub(&mut self, vd: usize, vs: usize, vt: usize, e: usize) {
        let (left, right) = self.vector_sources(vs, vt, e);
        let incoming = self.vco;
        for lane in 0..8 {
            let borrow = i32::from((incoming >> lane) & 1);
            let value = i32::from(left[lane] as i16) - i32::from(right[lane] as i16) - borrow;
            self.set_acc_low(lane, value as u16);
            self.vectors[vd][lane] = Self::signed_sat(i64::from(value));
        }
        self.vco = 0;
    }

    fn vsubc(&mut self, vd: usize, vs: usize, vt: usize, e: usize) {
        let (left, right) = self.vector_sources(vs, vt, e);
        let mut low = 0u16;
        let mut high = 0u16;
        for lane in 0..8 {
            let result = left[lane].wrapping_sub(right[lane]);
            self.set_acc_low(lane, result);
            self.vectors[vd][lane] = result;
            if left[lane] < right[lane] {
                low |= 1 << lane;
            }
            if result != 0 {
                high |= 1 << lane;
            }
        }
        self.vco = low | (high << 8);
    }

    fn vabs(&mut self, vd: usize, vs: usize, vt: usize, e: usize) {
        let (left, right) = self.vector_sources(vs, vt, e);
        for lane in 0..8 {
            let sign = left[lane] as i16;
            let value = right[lane] as i16;
            let result = if sign < 0 {
                let raw = value.wrapping_neg() as u16;
                self.set_acc_low(lane, raw);
                if value == i16::MIN {
                    0x7fff
                } else {
                    raw
                }
            } else if sign > 0 {
                self.set_acc_low(lane, right[lane]);
                right[lane]
            } else {
                self.set_acc_low(lane, 0);
                0
            };
            self.vectors[vd][lane] = result;
        }
    }

    fn vsaw(&mut self, vd: usize, element: usize) {
        let shift = match element {
            8 => 32,
            9 => 16,
            10 => 0,
            _ => {
                self.vectors[vd] = [0; 8];
                return;
            }
        };
        for lane in 0..8 {
            self.vectors[vd][lane] = (self.acc[lane] >> shift) as u16;
        }
    }

    fn vcompare(&mut self, vd: usize, vs: usize, vt: usize, e: usize, mode: u8) {
        let (left, right) = self.vector_sources(vs, vt, e);
        let old_vco = self.vco;
        let mut flags = 0u16;
        for lane in 0..8 {
            let left_signed = left[lane] as i16;
            let right_signed = right[lane] as i16;
            let carry = (old_vco >> lane) & 1 != 0;
            let neq = (old_vco >> (lane + 8)) & 1 != 0;
            let select_left = match mode {
                0 => left_signed < right_signed || (left_signed == right_signed && carry && neq),
                1 => left[lane] == right[lane] && !neq,
                2 => left[lane] != right[lane] || neq,
                3 => {
                    left_signed > right_signed || (left_signed == right_signed && (!carry || !neq))
                }
                _ => false,
            };
            if select_left {
                flags |= 1 << lane;
            }
            let result = if select_left { left[lane] } else { right[lane] };
            self.set_acc_low(lane, result);
            self.vectors[vd][lane] = result;
        }
        self.vcc = flags;
        self.vco = 0;
    }

    fn flag(flags: u16, bit: usize) -> bool {
        flags & (1 << bit) != 0
    }

    fn set_flag(flags: &mut u16, bit: usize, value: bool) {
        if value {
            *flags |= 1 << bit;
        } else {
            *flags &= !(1 << bit);
        }
    }

    fn set_vce_flag(flags: &mut u8, bit: usize, value: bool) {
        if value {
            *flags |= 1 << bit;
        } else {
            *flags &= !(1 << bit);
        }
    }

    fn set_acc_low(&mut self, lane: usize, value: u16) {
        self.acc[lane] = (self.acc[lane] & !0xffff) | i64::from(value);
    }

    fn vzero(&mut self, vd: usize, vs: usize, vt: usize, e: usize) {
        let (left, right) = self.vector_sources(vs, vt, e);
        for lane in 0..8 {
            self.set_acc_low(lane, left[lane].wrapping_add(right[lane]));
            self.vectors[vd][lane] = 0;
        }
    }

    fn vclip(&mut self, vd: usize, vs: usize, vt: usize, e: usize, opcode: u8) {
        let (left, right) = self.vector_sources(vs, vt, e);

        for lane in 0..8 {
            let result = match opcode {
                0x24 => self.vcl_lane(lane, left[lane], right[lane]),
                0x25 => self.vch_lane(lane, left[lane], right[lane]),
                0x26 => self.vcr_lane(lane, left[lane], right[lane]),
                _ => unreachable!("RSP clip opcode dispatch must be VCL/VCH/VCR"),
            };
            self.set_acc_low(lane, result);
            self.vectors[vd][lane] = result;
        }

        if opcode != 0x25 {
            self.vco = 0;
            self.vce = 0;
        }
    }

    fn vch_lane(&mut self, lane: usize, left: u16, right: u16) -> u16 {
        let left_signed = left as i16;
        let right_signed = right as i16;

        if (left_signed ^ right_signed) < 0 {
            let sum = left_signed.wrapping_add(right_signed);
            Self::set_flag(&mut self.vcc, lane, sum <= 0);
            Self::set_flag(&mut self.vcc, lane + 8, right_signed < 0);
            Self::set_flag(&mut self.vco, lane, true);
            Self::set_flag(&mut self.vco, lane + 8, sum != 0 && left != !right);
            Self::set_vce_flag(&mut self.vce, lane, sum == -1);
            if sum <= 0 {
                right.wrapping_neg()
            } else {
                left
            }
        } else {
            let difference = left_signed.wrapping_sub(right_signed);
            Self::set_flag(&mut self.vcc, lane, right_signed < 0);
            Self::set_flag(&mut self.vcc, lane + 8, difference >= 0);
            Self::set_flag(&mut self.vco, lane, false);
            Self::set_flag(&mut self.vco, lane + 8, difference != 0);
            Self::set_vce_flag(&mut self.vce, lane, false);
            if difference >= 0 {
                right
            } else {
                left
            }
        }
    }

    fn vcl_lane(&mut self, lane: usize, left: u16, right: u16) -> u16 {
        let vco_low = Self::flag(self.vco, lane);
        let vco_high = Self::flag(self.vco, lane + 8);

        if vco_low {
            if vco_high {
                if Self::flag(self.vcc, lane) {
                    right.wrapping_neg()
                } else {
                    left
                }
            } else {
                let sum = left.wrapping_add(right);
                let carry = u32::from(left) + u32::from(right) != u32::from(sum);
                let select_right = if self.vce & (1 << lane) != 0 {
                    sum == 0 || !carry
                } else {
                    sum == 0 && !carry
                };
                Self::set_flag(&mut self.vcc, lane, select_right);
                if select_right {
                    right.wrapping_neg()
                } else {
                    left
                }
            }
        } else if vco_high {
            if Self::flag(self.vcc, lane + 8) {
                right
            } else {
                left
            }
        } else {
            let select_right = i32::from(left) - i32::from(right) >= 0;
            Self::set_flag(&mut self.vcc, lane + 8, select_right);
            if select_right {
                right
            } else {
                left
            }
        }
    }

    fn vcr_lane(&mut self, lane: usize, left: u16, right: u16) -> u16 {
        let left_signed = left as i16;
        let right_signed = right as i16;

        if (left_signed ^ right_signed) < 0 {
            Self::set_flag(&mut self.vcc, lane + 8, right_signed < 0);
            let select_right = i32::from(left_signed) + i32::from(right_signed) < 0;
            Self::set_flag(&mut self.vcc, lane, select_right);
            if select_right {
                !right
            } else {
                left
            }
        } else {
            Self::set_flag(&mut self.vcc, lane, right_signed < 0);
            let select_right = i32::from(left_signed) - i32::from(right_signed) >= 0;
            Self::set_flag(&mut self.vcc, lane + 8, select_right);
            if select_right {
                right
            } else {
                left
            }
        }
    }

    fn vmerge(&mut self, vd: usize, vs: usize, vt: usize, e: usize) {
        let (left, right) = self.vector_sources(vs, vt, e);
        for lane in 0..8 {
            let value = if self.vcc & (1 << lane) != 0 {
                left[lane]
            } else {
                right[lane]
            };
            self.set_acc_low(lane, value);
            self.vectors[vd][lane] = value;
        }
        self.vco = 0;
    }

    fn vlogic(&mut self, vd: usize, vs: usize, vt: usize, e: usize, mode: u8) {
        let (left, right) = self.vector_sources(vs, vt, e);
        for lane in 0..8 {
            let value = match mode {
                0 => left[lane] & right[lane],
                1 => !(left[lane] & right[lane]),
                2 => left[lane] | right[lane],
                3 => !(left[lane] | right[lane]),
                4 => left[lane] ^ right[lane],
                _ => !(left[lane] ^ right[lane]),
            };
            self.set_acc_low(lane, value);
            self.vectors[vd][lane] = value;
        }
    }

    fn reciprocal_result(input: i32, square_root: bool) -> u32 {
        let sign_mask = input >> 31;
        let mut data = input ^ sign_mask;
        if input > -32768 {
            data -= sign_mask;
        }

        let result = if data == 0 {
            0x7fff_ffff
        } else if input == -32768 {
            0xffff_0000u32 as i32
        } else {
            let shift = (data as u32).leading_zeros();
            let index = ((u64::from(data as u32) << shift) & 0x7fc0_0000) >> 22;
            let entry = if square_root {
                RSP_INVERSE_SQRT_ROM[((index as usize) & 0x1fe) | (shift as usize & 1)]
            } else {
                RSP_RECIPROCAL_ROM[index as usize]
            };
            let normalized = (0x1_0000 | i32::from(entry)) << 14;
            let back_shift = if square_root {
                (31 - shift) >> 1
            } else {
                31 - shift
            };
            (normalized >> back_shift) ^ sign_mask
        };
        result as u32
    }

    fn vscalar(&mut self, vd: usize, de: usize, vt: usize, e: usize, opcode: u8) {
        let right: [u16; 8] = core::array::from_fn(|lane| self.vt_lane(vt, e, lane));
        let reciprocal_source = right[e & 7];
        let destination_lane = de & 7;
        let result = match opcode {
            0x30 | 0x31 | 0x34 | 0x35 => {
                let long = matches!(opcode, 0x31 | 0x35);
                let square_root = opcode >= 0x34;
                let input = if long && self.div_pending {
                    ((self.div_in & 0xffff_0000) | u32::from(reciprocal_source)) as i32
                } else {
                    i32::from(reciprocal_source as i16)
                };
                self.div_out = Self::reciprocal_result(input, square_root);
                self.div_pending = false;
                self.div_out as u16
            }
            0x32 | 0x36 => {
                let previous_high = (self.div_out >> 16) as u16;
                self.div_in = u32::from(reciprocal_source) << 16;
                self.div_pending = true;
                previous_high
            }
            0x33 => right[destination_lane],
            _ => return,
        };

        self.vectors[vd][destination_lane] = result;
        for (lane, value) in right.into_iter().enumerate() {
            self.set_acc_low(lane, value);
        }
    }

    pub fn save(&self, out: &mut StateWriter) {
        for value in self.regs {
            out.u32(value);
        }
        for vector in self.vectors {
            for value in vector {
                out.u16(value);
            }
        }
        out.u32(self.pc);
        out.u32(self.next_pc);
        out.u64(self.cycles);
        out.u8(u8::from(self.running));
        out.u8(u8::from(self.broke));
        for value in self.acc {
            out.u64(value as u64);
        }
        out.u16(self.vco);
        out.u16(self.vcc);
        out.u8(self.vce);
        out.u8(u8::from(self.branch_delay));
        out.u8(u8::from(self.pending_branch.is_some()));
        out.u32(self.pending_branch.unwrap_or(0));
        out.u32(self.div_in);
        out.u32(self.div_out);
        out.u8(u8::from(self.div_pending));
    }

    pub fn load_state(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        for value in &mut self.regs {
            *value = input.u32()?;
        }
        for vector in &mut self.vectors {
            for value in vector {
                *value = input.u16()?;
            }
        }
        self.pc = input.u32()? & 0x0ffc;
        self.next_pc = input.u32()? & 0x0ffc;
        self.cycles = input.u64()?;
        self.running = input.u8()? != 0;
        self.broke = input.u8()? != 0;
        for value in &mut self.acc {
            *value = input.u64()? as i64;
        }
        self.vco = input.u16()?;
        self.vcc = input.u16()?;
        self.vce = input.u8()?;
        self.branch_delay = input.u8()? != 0;
        let has_branch = input.u8()? != 0;
        let branch = input.u32()? & 0x0ffc;
        self.pending_branch = has_branch.then_some(branch);
        self.div_in = input.u32()?;
        self.div_out = input.u32()?;
        self.div_pending = input.u8()? != 0;
        self.regs[0] = 0;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::platform::PlatformId;

    struct Bus {
        bytes: [u8; 0x2000],
        cop0: [u32; 16],
    }
    impl Default for Bus {
        fn default() -> Self {
            Self {
                bytes: [0; 0x2000],
                cop0: [0; 16],
            }
        }
    }
    impl RspBus for Bus {
        fn read8(&mut self, address: u32) -> u8 {
            self.bytes[address as usize & 0x1fff]
        }
        fn write8(&mut self, address: u32, value: u8) {
            self.bytes[address as usize & 0x1fff] = value;
        }
        fn read_cop0(&mut self, index: u8) -> u32 {
            self.cop0[index as usize & 15]
        }
        fn write_cop0(&mut self, index: u8, value: u32) {
            self.cop0[index as usize & 15] = value;
        }
    }
    fn put32(bus: &mut Bus, address: usize, value: u32) {
        bus.bytes[address..address + 4].copy_from_slice(&value.to_be_bytes());
    }
    fn i(op: u32, rs: u8, rt: u8, imm: u16) -> u32 {
        (op << 26) | (u32::from(rs) << 21) | (u32::from(rt) << 16) | u32::from(imm)
    }
    fn vector(op: u32, e: u32, vt: u8, vs: u8, vd: u8) -> u32 {
        (0x12 << 26)
            | (1 << 25)
            | ((e & 15) << 21)
            | (u32::from(vt) << 16)
            | (u32::from(vs) << 11)
            | (u32::from(vd) << 6)
            | (op & 63)
    }

    fn vector_memory(load: bool, base: u8, vt: u8, op: u8, element: u8, offset: i8) -> u32 {
        let primary = if load { 0x32 } else { 0x3a };
        let encoded_offset = (i32::from(offset) & 0x7f) as u32;
        (primary << 26)
            | (u32::from(base) << 21)
            | (u32::from(vt) << 16)
            | (u32::from(op) << 11)
            | (u32::from(element & 15) << 7)
            | encoded_offset
    }

    #[test]
    fn scalar_unaligned_memory_wraps_within_dmem_at_4k_boundary() {
        let mut bus = Bus::default();
        bus.bytes[0x0fff] = 0x12;
        bus.bytes[0x0000] = 0x34;
        bus.bytes[0x0001] = 0x56;
        bus.bytes[0x0002] = 0x78;
        bus.bytes[0x1000..0x1004].copy_from_slice(&[0xaa, 0xbb, 0xcc, 0xdd]);

        let mut rsp = Rsp::new();
        rsp.regs[1] = 0x0fff;

        rsp.load(&mut bus, i(0x25, 1, 2, 0));
        assert_eq!(rsp.regs[2], 0x1234);

        rsp.load(&mut bus, i(0x23, 1, 2, 0));
        assert_eq!(rsp.regs[2], 0x1234_5678);

        rsp.regs[3] = 0xa1b2_c3d4;
        rsp.store(&mut bus, i(0x2b, 1, 3, 0));
        assert_eq!(
            [bus.bytes[0x0fff], bus.bytes[0], bus.bytes[1], bus.bytes[2]],
            [0xa1, 0xb2, 0xc3, 0xd4]
        );
        assert_eq!(&bus.bytes[0x1000..0x1004], &[0xaa, 0xbb, 0xcc, 0xdd]);

        rsp.regs[3] = 0x5566;
        rsp.store(&mut bus, i(0x29, 1, 3, 0));
        assert_eq!([bus.bytes[0x0fff], bus.bytes[0]], [0x55, 0x66]);
        assert_eq!(&bus.bytes[0x1000..0x1004], &[0xaa, 0xbb, 0xcc, 0xdd]);
    }

    #[test]
    fn scalar_branch_delay_and_cop0_execute() {
        let mut bus = Bus::default();
        put32(&mut bus, 0x1000, i(0x0d, 0, 1, 1));
        put32(&mut bus, 0x1004, i(0x04, 1, 1, 1));
        put32(&mut bus, 0x1008, i(0x0d, 0, 2, 2));
        put32(&mut bus, 0x100c, i(0x0d, 0, 3, 3));
        put32(
            &mut bus,
            0x1010,
            (0x10 << 26) | (4 << 21) | (3 << 16) | (5 << 11),
        );
        let mut rsp = Rsp::new();
        rsp.running = true;
        for _ in 0..5 {
            rsp.step(&mut bus);
        }
        assert_eq!(rsp.regs[2], 2);
        assert_eq!(rsp.regs[3], 3);
        assert_eq!(bus.cop0[5], 3);
    }
    #[test]
    fn vector_add_carry_and_broadcast_chain() {
        let mut bus = Bus::default();
        put32(&mut bus, 0x1000, vector(0x14, 8, 2, 1, 3));
        put32(&mut bus, 0x1004, vector(0x10, 8, 2, 3, 4));
        let mut rsp = Rsp::new();
        rsp.running = true;
        rsp.vectors[1] = [0xffff, 1, 2, 3, 4, 5, 6, 7];
        rsp.vectors[2][0] = 1;
        rsp.step(&mut bus);
        assert_eq!(rsp.vectors[3][0], 0);
        assert_eq!(rsp.vco & 1, 1);
        rsp.step(&mut bus);
        assert_eq!(rsp.vectors[4][0], 2);
        assert_eq!(rsp.vco, 0);
    }

    #[test]
    fn vector_clip_high_sets_hardware_flags() {
        let mut bus = Bus::default();
        put32(&mut bus, 0x1000, vector(0x25, 0, 2, 1, 3));
        let mut rsp = Rsp::new();
        rsp.running = true;
        rsp.vectors[1] = [1; 8];
        rsp.vectors[2] = [0xfffe; 8];

        rsp.step(&mut bus);

        assert_eq!(rsp.vectors[3], [2; 8]);
        assert_eq!(rsp.flags(), (0x00ff, 0xffff, 0xff));
    }

    #[test]
    fn vector_clip_low_consumes_flags_and_preserves_accumulator_high_slices() {
        let mut bus = Bus::default();
        put32(&mut bus, 0x1000, vector(0x24, 0, 2, 1, 3));
        let mut rsp = Rsp::new();
        rsp.running = true;
        rsp.vectors[1] = [1; 8];
        rsp.vectors[2] = [0xffff; 8];
        rsp.vco = 0x00ff;
        rsp.vcc = 0x5500;
        rsp.acc = [0x1234_5678_aaaa; 8];

        rsp.step(&mut bus);

        assert_eq!(rsp.vectors[3], [1; 8]);
        assert_eq!(rsp.flags(), (0, 0x5500, 0));
        assert_eq!(rsp.acc[0], 0x1234_5678_0001);
    }

    #[test]
    fn vector_clip_one_complement_snapshots_broadcast_sources_before_alias_writes() {
        let mut bus = Bus::default();
        put32(&mut bus, 0x1000, vector(0x26, 4, 7, 6, 7));
        let mut rsp = Rsp::new();
        rsp.running = true;
        rsp.vectors[6] = [0, 2, 3, 4, 0, 6, 7, 8];
        rsp.vectors[7] = [0xfffe, 0, 0, 0, 0xfffe, 0, 0, 0];
        rsp.vco = 0xffff;
        rsp.vce = 0xff;

        rsp.step(&mut bus);

        assert_eq!(rsp.vectors[7], [1, 2, 3, 4, 1, 6, 7, 8]);
        assert_eq!(rsp.flags(), (0, 0xff11, 0));
    }

    #[test]
    fn vector_compute_snapshots_destructive_broadcast_sources() {
        let mut bus = Bus::default();
        put32(&mut bus, 0x1000, vector(0x10, 4, 7, 6, 7));
        let mut rsp = Rsp::new();
        rsp.running = true;
        rsp.vectors[6] = [1, 2, 3, 4, 5, 6, 7, 8];
        rsp.vectors[7] = [10, 0, 0, 0, 20, 0, 0, 0];

        rsp.step(&mut bus);

        assert_eq!(rsp.vectors[7], [11, 12, 13, 14, 25, 26, 27, 28]);
    }

    #[test]
    fn vmulu_uses_rsp_unsigned_saturation_threshold() {
        let mut bus = Bus::default();
        put32(&mut bus, 0x1000, vector(0x01, 0, 2, 1, 3));
        let mut rsp = Rsp::new();
        rsp.running = true;
        rsp.vectors[1] = [0x8000; 8];
        rsp.vectors[2] = [0x8000; 8];

        rsp.step(&mut bus);

        assert_eq!(rsp.vectors[3], [0xffff; 8]);
    }

    #[test]
    fn vabs_minimum_keeps_raw_accumulator_low_but_saturates_destination() {
        let mut bus = Bus::default();
        put32(&mut bus, 0x1000, vector(0x13, 0, 2, 1, 3));
        let mut rsp = Rsp::new();
        rsp.running = true;
        rsp.vectors[1] = [0xffff; 8];
        rsp.vectors[2] = [0x8000; 8];
        rsp.acc = [0x1234_5678_aaaa; 8];

        rsp.step(&mut bus);

        assert_eq!(rsp.vectors[3], [0x7fff; 8]);
        assert_eq!(rsp.acc[0], 0x1234_5678_8000);
    }

    #[test]
    fn vmad_middle_and_low_results_apply_hardware_extraction_rules() {
        let mut bus = Bus::default();
        put32(&mut bus, 0x1000, vector(0x0d, 0, 2, 1, 3));
        put32(&mut bus, 0x1004, vector(0x0e, 0, 2, 1, 4));
        let mut rsp = Rsp::new();
        rsp.running = true;

        rsp.acc = [0x0000_8000_0000; 8];
        rsp.step(&mut bus);
        assert_eq!(rsp.vectors[3], [0x7fff; 8]);

        rsp.acc = [0x0000_8000_c000; 8];
        rsp.step(&mut bus);
        assert_eq!(rsp.vectors[4], [0xffff; 8]);
    }

    #[test]
    fn vrnd_and_quantized_multiply_family_update_full_accumulator() {
        let mut bus = Bus::default();
        put32(&mut bus, 0x1000, vector(0x02, 0, 2, 1, 3));
        put32(&mut bus, 0x1004, vector(0x0a, 0, 2, 1, 4));
        put32(&mut bus, 0x1008, vector(0x03, 0, 2, 1, 5));
        put32(&mut bus, 0x100c, vector(0x0b, 0, 0, 0, 6));
        let mut rsp = Rsp::new();
        rsp.running = true;
        rsp.vectors[1] = [4, 0xfffc, 4, 0xfffc, 4, 0xfffc, 4, 0xfffc];
        rsp.vectors[2] = [1; 8];

        rsp.step(&mut bus);
        assert_eq!(rsp.vectors[3], [1; 8]);
        assert_eq!(rsp.acc, [0x1_0000; 8]);

        rsp.acc = [-0x1_0000; 8];
        rsp.step(&mut bus);
        assert_eq!(rsp.vectors[4], [0; 8]);
        assert_eq!(rsp.acc, [0; 8]);

        rsp.vectors[2] = [8; 8];
        rsp.step(&mut bus);
        assert_eq!(rsp.vectors[5][0], 0x0010);
        assert_eq!(rsp.vectors[5][1], 0xfff0);
        assert_eq!(rsp.acc[0], 0x20_0000);
        assert_eq!(rsp.acc[1], -0x1_0000);

        rsp.acc = [0x40_0000, -0x40_0000, 0, 0, 0, 0, 0, 0];
        rsp.step(&mut bus);
        assert_eq!(rsp.vectors[6][0], 0x0010);
        assert_eq!(rsp.vectors[6][1], 0xfff0);
        assert_eq!(rsp.acc[0], 0x20_0000);
        assert_eq!(rsp.acc[1], -0x20_0000);
    }

    #[test]
    fn reserved_vector_ops_zero_destination_but_preserve_flags() {
        let mut bus = Bus::default();
        put32(&mut bus, 0x1000, vector(0x12, 4, 7, 6, 7));
        let mut rsp = Rsp::new();
        rsp.running = true;
        rsp.vectors[6] = [1, 2, 3, 4, 5, 6, 7, 8];
        rsp.vectors[7] = [10, 0, 0, 0, 20, 0, 0, 0];
        rsp.acc = [0x1234_5678_0000; 8];
        rsp.vco = 0xaaaa;
        rsp.vcc = 0x5555;
        rsp.vce = 0x5a;

        rsp.step(&mut bus);

        assert_eq!(rsp.vectors[7], [0; 8]);
        assert_eq!(
            rsp.acc,
            [
                0x1234_5678_000b,
                0x1234_5678_000c,
                0x1234_5678_000d,
                0x1234_5678_000e,
                0x1234_5678_0019,
                0x1234_5678_001a,
                0x1234_5678_001b,
                0x1234_5678_001c,
            ]
        );
        assert_eq!(rsp.flags(), (0xaaaa, 0x5555, 0x5a));
    }

    #[test]
    fn compare_preserves_vce_and_merge_clears_vco() {
        let mut bus = Bus::default();
        put32(&mut bus, 0x1000, vector(0x21, 0, 2, 1, 3));
        put32(&mut bus, 0x1004, vector(0x27, 0, 2, 1, 4));
        let mut rsp = Rsp::new();
        rsp.running = true;
        rsp.vectors[1] = [1; 8];
        rsp.vectors[2] = [1; 8];
        rsp.vce = 0xa5;

        rsp.step(&mut bus);
        assert_eq!(rsp.vcc, 0x00ff);
        assert_eq!(rsp.vce, 0xa5);

        rsp.vco = 0xffff;
        rsp.step(&mut bus);
        assert_eq!(rsp.vco, 0);
        assert_eq!(rsp.vce, 0xa5);
    }

    #[test]
    fn reciprocal_roms_match_hardware_reference_entries() {
        assert_eq!(
            &RSP_RECIPROCAL_ROM[..5],
            &[0xffff, 0xff00, 0xfe01, 0xfd04, 0xfc07]
        );
        assert_eq!(
            &RSP_INVERSE_SQRT_ROM[..12],
            &[27145, 65535, 26965, 65280, 26785, 65026, 26607, 64774, 26430, 64523, 26253, 64274,]
        );
    }

    #[test]
    fn reciprocal_high_stages_input_and_returns_previous_high_half() {
        let mut rsp = Rsp::new();
        rsp.vectors[1][0] = 0x1234;
        rsp.div_out = 0xbeef_cafe;

        rsp.vscalar(2, 0, 1, 0, 0x32);

        assert_eq!(rsp.vectors[2][0], 0xbeef);
        assert_eq!(rsp.div_in, 0x1234_0000);
        assert_eq!(rsp.div_out, 0xbeef_cafe);
        assert!(rsp.div_pending);
    }

    #[test]
    fn reciprocal_and_rsqrt_use_element_source_and_exact_rom_results() {
        let mut rsp = Rsp::new();
        rsp.vectors[1] = [2, 4, 8, 16, 32, 64, 128, 256];

        rsp.vscalar(2, 5, 1, 1, 0x30);
        assert_eq!(rsp.div_out, 0x1fff_f000);
        assert_eq!(rsp.vectors[2][5], 0xf000);

        rsp.vscalar(3, 4, 1, 1, 0x34);
        assert_eq!(rsp.div_out, 0x3fff_e000);
        assert_eq!(rsp.vectors[3][4], 0xe000);

        rsp.vectors[1][1] = 0;
        rsp.vscalar(4, 0, 1, 1, 0x30);
        assert_eq!(rsp.div_out, 0x7fff_ffff);
        assert_eq!(rsp.vectors[4][0], 0xffff);
    }

    #[test]
    fn reciprocal_low_consumes_staging_once_and_vmov_preserves_division_latches() {
        let mut rsp = Rsp::new();
        rsp.vectors[1][0] = 0x1234;
        rsp.div_out = 0xabcd_5678;
        rsp.vscalar(2, 0, 1, 0, 0x32);

        rsp.vectors[1][0] = 2;
        rsp.vscalar(3, 1, 1, 0, 0x31);
        assert_eq!(rsp.div_out, 0x0000_0007);
        assert_eq!(rsp.vectors[3][1], 7);
        assert!(!rsp.div_pending);

        rsp.div_out = 0x1122_3344;
        rsp.div_pending = true;
        rsp.vectors[1] = [
            0x1000, 0x1001, 0x1002, 0x1003, 0x1004, 0x1005, 0x1006, 0x1007,
        ];
        rsp.vscalar(4, 3, 1, 0, 0x33);
        assert_eq!(rsp.vectors[4][3], 0x1003);
        assert_eq!(rsp.div_out, 0x1122_3344);
        assert!(rsp.div_pending);
    }

    #[test]
    fn vector_right_load_element_shortens_without_register_wrap() {
        let mut bus = Bus::default();
        for index in 0..8 {
            bus.bytes[0x10 + index] = 0xa0 + index as u8;
        }
        let mut rsp = Rsp::new();
        rsp.regs[1] = 0x18;
        put32(&mut bus, 0x1000, vector_memory(true, 1, 2, 5, 2, 0));
        rsp.running = true;

        rsp.step(&mut bus);

        for byte in 0..10 {
            assert_eq!(rsp.vector_byte(2, byte), 0);
        }
        for index in 0..6 {
            assert_eq!(rsp.vector_byte(2, 10 + index), 0xa0 + index as u8);
        }
    }

    #[test]
    fn vector_packed_and_strided_memory_follow_lane_layout() {
        let mut bus = Bus::default();
        for index in 0..16 {
            bus.bytes[index] = 0x10 + index as u8;
        }
        let mut rsp = Rsp::new();
        put32(&mut bus, 0x1000, vector_memory(true, 0, 2, 6, 0, 0));
        put32(&mut bus, 0x1004, vector_memory(true, 0, 3, 7, 0, 0));
        put32(&mut bus, 0x1008, vector_memory(true, 0, 4, 8, 0, 0));
        rsp.running = true;

        rsp.step(&mut bus);
        rsp.step(&mut bus);
        rsp.step(&mut bus);

        assert_eq!(
            rsp.vectors[2],
            [0x1000, 0x1100, 0x1200, 0x1300, 0x1400, 0x1500, 0x1600, 0x1700]
        );
        assert_eq!(
            rsp.vectors[3],
            [0x0800, 0x0880, 0x0900, 0x0980, 0x0a00, 0x0a80, 0x0b00, 0x0b80]
        );
        assert_eq!(
            rsp.vectors[4],
            [0x0800, 0x0900, 0x0a00, 0x0b00, 0x0c00, 0x0d00, 0x0e00, 0x0f00]
        );

        rsp.regs[1] = 0x100;
        rsp.vectors[5] =
            core::array::from_fn(|lane| ((0x10 + lane as u16) << 8) | (0x80 + lane as u16));
        put32(&mut bus, 0x100c, vector_memory(false, 1, 5, 6, 4, 0));
        rsp.step(&mut bus);
        assert_eq!(
            &bus.bytes[0x100..0x108],
            &[0x14, 0x15, 0x16, 0x17, 0x21, 0x23, 0x25, 0x27]
        );
    }

    #[test]
    fn vector_fractional_load_subtracts_nonzero_element_from_first_source() {
        let mut bus = Bus::default();
        bus.bytes[2] = 1;
        bus.bytes[4] = 0;

        let mut rsp = Rsp::new();
        rsp.regs[1] = 3;
        put32(&mut bus, 0x1000, vector_memory(true, 1, 2, 9, 1, 0));
        rsp.running = true;

        rsp.step(&mut bus);

        assert_eq!(rsp.vector_byte(2, 0), 0);
        assert_eq!(rsp.vector_byte(2, 1), 0x80);
        assert_eq!(rsp.vectors[2][0], 0x0080);
    }

    #[test]
    fn vector_fractional_memory_uses_element_specific_rules() {
        let mut bus = Bus::default();
        for index in 0..16 {
            bus.bytes[index] = 0x10 + index as u8;
        }
        let mut rsp = Rsp::new();
        put32(&mut bus, 0x1000, vector_memory(true, 0, 2, 9, 4, 0));
        put32(&mut bus, 0x1004, vector_memory(false, 0, 3, 9, 2, 0));
        rsp.vectors[3] = core::array::from_fn(|lane| (lane as u16 + 1) << 8);
        rsp.running = true;

        rsp.step(&mut bus);
        assert_eq!(rsp.vectors[2], [0, 0, 0x0a00, 0x0c00, 0x0a00, 0x0c00, 0, 0]);

        bus.bytes[..16].fill(0xee);
        rsp.step(&mut bus);
        assert_eq!(
            [bus.bytes[0], bus.bytes[4], bus.bytes[8], bus.bytes[12]],
            [0, 0, 0, 0]
        );
        assert_eq!(bus.bytes[1], 0xee);
    }

    #[test]
    fn vector_transpose_memory_moves_diagonal_register_group() {
        let mut bus = Bus::default();
        for index in 0..16 {
            bus.bytes[index] = 0x10 + index as u8;
        }
        let mut rsp = Rsp::new();
        put32(&mut bus, 0x1000, vector_memory(true, 0, 0, 11, 0, 0));
        put32(&mut bus, 0x1004, vector_memory(false, 0, 0, 11, 0, 0));
        rsp.running = true;

        rsp.step(&mut bus);
        let diagonal = [
            0x1011, 0x1213, 0x1415, 0x1617, 0x1819, 0x1a1b, 0x1c1d, 0x1e1f,
        ];
        for (register, expected) in diagonal.into_iter().enumerate() {
            assert_eq!(rsp.vectors[register][register], expected);
        }

        for register in 0..8usize {
            for byte in 0..16usize {
                rsp.set_vector_byte(register, byte, (register * 0x10 + byte) as u8);
            }
        }
        bus.bytes[..16].fill(0);
        rsp.step(&mut bus);
        assert_eq!(
            &bus.bytes[..16],
            &[
                0x00, 0x01, 0x12, 0x13, 0x24, 0x25, 0x36, 0x37, 0x48, 0x49, 0x5a, 0x5b, 0x6c, 0x6d,
                0x7e, 0x7f,
            ]
        );
    }

    #[test]
    fn vector_whole_store_rotates_within_eight_byte_window() {
        let mut bus = Bus::default();
        let mut rsp = Rsp::new();
        rsp.regs[1] = 3;
        for byte in 0..16usize {
            rsp.set_vector_byte(2, byte, 0xa0 + byte as u8);
        }
        put32(&mut bus, 0x1000, vector_memory(false, 1, 2, 10, 0, 0));
        rsp.running = true;

        rsp.step(&mut bus);

        assert_eq!(
            &bus.bytes[..16],
            &[
                0xad, 0xae, 0xaf, 0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa,
                0xab, 0xac,
            ]
        );
    }

    #[test]
    fn vector_quad_load_and_store_use_byte_elements() {
        let mut bus = Bus::default();
        for index in 0..16 {
            bus.bytes[0x80 + index] = index as u8;
        }
        let mut rsp = Rsp::new();
        rsp.regs[1] = 0x80;
        let load = (0x32 << 26) | (1 << 21) | (2 << 16) | (4 << 11) | (4 << 7);
        let store = (0x3a << 26) | (1 << 21) | (2 << 16) | (4 << 11) | (4 << 7) | 1;
        put32(&mut bus, 0x1000, load);
        put32(&mut bus, 0x1004, store);
        rsp.running = true;
        rsp.step(&mut bus);
        assert_eq!(rsp.vector_byte(2, 4), 0);
        assert_eq!(rsp.vector_byte(2, 11), 7);
        rsp.step(&mut bus);
        assert_eq!(&bus.bytes[0x90..0x98], &[0, 1, 2, 3, 4, 5, 6, 7]);
    }

    #[test]
    fn break_and_state_round_trip_preserve_vector_pipeline() {
        let mut bus = Bus::default();
        put32(&mut bus, 0x1000, 0x0000_000d);
        let mut rsp = Rsp::new();
        rsp.running = true;
        rsp.vectors[7][3] = 0x55aa;
        rsp.acc[3] = -0x1234_5678;
        rsp.vcc = 0x5a5a;
        rsp.div_in = 0x1122_0000;
        rsp.div_pending = true;
        rsp.step(&mut bus);
        assert!(rsp.broke && !rsp.running);
        let mut writer = StateWriter::new(PlatformId::Nintendo64, 1);
        rsp.save(&mut writer);
        let bytes = writer.finish();
        let mut reader = StateReader::new(&bytes, PlatformId::Nintendo64, 1).unwrap();
        let mut restored = Rsp::new();
        restored.load_state(&mut reader).unwrap();
        reader.finish().unwrap();
        assert_eq!(restored.vectors[7][3], 0x55aa);
        assert_eq!(restored.acc[3], -0x1234_5678);
        assert_eq!(restored.vcc, 0x5a5a);
        assert_eq!(restored.div_in, 0x1122_0000);
        assert!(restored.div_pending && restored.broke && !restored.running);
    }
}
