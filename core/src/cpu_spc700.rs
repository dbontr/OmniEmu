use crate::state::{StateReader, StateWriter};

pub trait Spc700Bus: Send {
    fn read8(&mut self, address: u16) -> u8;
    fn write8(&mut self, address: u16, value: u8);
}

const N: u8 = 0x80;
const V: u8 = 0x40;
const P: u8 = 0x20;
const B: u8 = 0x10;
const H: u8 = 0x08;
const I: u8 = 0x04;
const Z: u8 = 0x02;
const C: u8 = 0x01;

#[derive(Debug, Clone)]
pub struct Spc700 {
    pub a: u8,
    pub x: u8,
    pub y: u8,
    pub sp: u8,
    pub pc: u16,
    pub psw: u8,
    pub cycles: u64,
    pub sleeping: bool,
    pub stopped: bool,
}
impl Default for Spc700 {
    fn default() -> Self {
        Self {
            a: 0,
            x: 0,
            y: 0,
            sp: 0xef,
            pc: 0xffc0,
            psw: 0x02,
            cycles: 0,
            sleeping: false,
            stopped: false,
        }
    }
}

#[derive(Clone, Copy)]
enum Alu {
    Or,
    And,
    Eor,
    Cmp,
    Adc,
    Sbc,
}

impl Spc700 {
    pub fn reset<B: Spc700Bus>(&mut self, bus: &mut B) {
        *self = Self::default();
        self.pc = self.read16(bus, 0xfffe);
    }
    fn set_flag(&mut self, flag: u8, value: bool) {
        if value {
            self.psw |= flag;
        } else {
            self.psw &= !flag;
        }
    }

    fn set_nz(&mut self, value: u8) {
        self.set_flag(N, value & 0x80 != 0);
        self.set_flag(Z, value == 0);
    }

    fn set_nz16(&mut self, value: u16) {
        self.set_flag(N, value & 0x8000 != 0);
        self.set_flag(Z, value == 0);
    }

    fn fetch8<B: Spc700Bus>(&mut self, bus: &mut B) -> u8 {
        let value = bus.read8(self.pc);
        self.pc = self.pc.wrapping_add(1);
        value
    }

    fn fetch16<B: Spc700Bus>(&mut self, bus: &mut B) -> u16 {
        let lo = self.fetch8(bus);
        let hi = self.fetch8(bus);
        u16::from_le_bytes([lo, hi])
    }

    fn read16<B: Spc700Bus>(&self, bus: &mut B, address: u16) -> u16 {
        let lo = bus.read8(address);
        let hi = bus.read8(address.wrapping_add(1));
        u16::from_le_bytes([lo, hi])
    }
    fn write16<B: Spc700Bus>(&self, bus: &mut B, address: u16, value: u16) {
        let [lo, hi] = value.to_le_bytes();
        bus.write8(address, lo);
        bus.write8(address.wrapping_add(1), hi);
    }

    fn dp_base(&self) -> u16 {
        if self.psw & P != 0 {
            0x0100
        } else {
            0
        }
    }

    fn dp(&self, offset: u8) -> u16 {
        self.dp_base() | u16::from(offset)
    }

    fn dp_index(&self, offset: u8, index: u8) -> u16 {
        self.dp(offset.wrapping_add(index))
    }

    fn read_dp16<B: Spc700Bus>(&self, bus: &mut B, offset: u8) -> u16 {
        let lo = bus.read8(self.dp(offset));
        let hi = bus.read8(self.dp(offset.wrapping_add(1)));
        u16::from_le_bytes([lo, hi])
    }

    fn push8<B: Spc700Bus>(&mut self, bus: &mut B, value: u8) {
        bus.write8(0x0100 | u16::from(self.sp), value);
        self.sp = self.sp.wrapping_sub(1);
    }
    fn pop8<B: Spc700Bus>(&mut self, bus: &mut B) -> u8 {
        self.sp = self.sp.wrapping_add(1);
        bus.read8(0x0100 | u16::from(self.sp))
    }

    fn push16<B: Spc700Bus>(&mut self, bus: &mut B, value: u16) {
        let [lo, hi] = value.to_le_bytes();
        self.push8(bus, hi);
        self.push8(bus, lo);
    }

    fn pop16<B: Spc700Bus>(&mut self, bus: &mut B) -> u16 {
        let lo = self.pop8(bus);
        let hi = self.pop8(bus);
        u16::from_le_bytes([lo, hi])
    }

    fn branch(&mut self, offset: u8, take: bool) -> u32 {
        if take {
            self.pc = self.pc.wrapping_add_signed(i16::from(offset as i8));
            4
        } else {
            2
        }
    }

    fn compare(&mut self, left: u8, right: u8) {
        let result = left.wrapping_sub(right);
        self.set_flag(C, left >= right);
        self.set_nz(result);
    }
    fn adc(&mut self, left: u8, right: u8) -> u8 {
        let carry = u16::from(self.psw & C != 0);
        let sum = u16::from(left) + u16::from(right) + carry;
        let result = sum as u8;
        self.set_flag(C, sum > 0xff);
        self.set_flag(H, (left & 0x0f) + (right & 0x0f) + carry as u8 > 0x0f);
        self.set_flag(V, (!(left ^ right) & (left ^ result) & 0x80) != 0);
        self.set_nz(result);
        result
    }

    fn sbc(&mut self, left: u8, right: u8) -> u8 {
        let carry = u16::from(self.psw & C != 0);
        let borrow = 1u16 - carry;
        let difference = u16::from(left).wrapping_sub(u16::from(right) + borrow);
        let result = difference as u8;
        self.set_flag(C, u16::from(left) >= u16::from(right) + borrow);
        self.set_flag(
            H,
            (left & 0x0f) >= (right & 0x0f).wrapping_add(borrow as u8),
        );
        self.set_flag(V, ((left ^ right) & (left ^ result) & 0x80) != 0);
        self.set_nz(result);
        result
    }

    fn alu_value(&mut self, operation: Alu, left: u8, right: u8) -> u8 {
        match operation {
            Alu::Or => {
                let value = left | right;
                self.set_nz(value);
                value
            }
            Alu::And => {
                let value = left & right;
                self.set_nz(value);
                value
            }
            Alu::Eor => {
                let value = left ^ right;
                self.set_nz(value);
                value
            }
            Alu::Cmp => {
                self.compare(left, right);
                left
            }
            Alu::Adc => self.adc(left, right),
            Alu::Sbc => self.sbc(left, right),
        }
    }

    fn operation_for(opcode: u8) -> Option<Alu> {
        Some(match opcode >> 5 {
            0 => Alu::Or,
            1 => Alu::And,
            2 => Alu::Eor,
            3 => Alu::Cmp,
            4 => Alu::Adc,
            5 => Alu::Sbc,
            _ => return None,
        })
    }

    fn indirect_x<B: Spc700Bus>(&self, bus: &mut B, operand: u8) -> u16 {
        self.read_dp16(bus, operand.wrapping_add(self.x))
    }

    fn indirect_y<B: Spc700Bus>(&self, bus: &mut B, operand: u8) -> u16 {
        self.read_dp16(bus, operand).wrapping_add(u16::from(self.y))
    }

    fn apply_alu_to_a(&mut self, operation: Alu, value: u8) {
        let result = self.alu_value(operation, self.a, value);
        if !matches!(operation, Alu::Cmp) {
            self.a = result;
        }
    }
    fn execute_alu_family<B: Spc700Bus>(&mut self, bus: &mut B, opcode: u8) -> Option<u32> {
        let operation = Self::operation_for(opcode)?;
        let low = opcode & 0x0f;
        if !(4..=9).contains(&low) {
            return None;
        }
        let indexed_row = opcode & 0x10 != 0;
        match (low, indexed_row) {
            (4, false) => {
                let dp = self.fetch8(bus);
                let value = bus.read8(self.dp(dp));
                self.apply_alu_to_a(operation, value);
                Some(3)
            }
            (5, false) => {
                let address = self.fetch16(bus);
                let value = bus.read8(address);
                self.apply_alu_to_a(operation, value);
                Some(4)
            }
            (6, false) => {
                let value = bus.read8(self.dp(self.x));
                self.apply_alu_to_a(operation, value);
                Some(3)
            }
            (7, false) => {
                let dp = self.fetch8(bus);
                let address = self.indirect_x(bus, dp);
                let value = bus.read8(address);
                self.apply_alu_to_a(operation, value);
                Some(6)
            }
            (8, false) => {
                let value = self.fetch8(bus);
                self.apply_alu_to_a(operation, value);
                Some(2)
            }
            (4, true) => {
                let dp = self.fetch8(bus);
                let value = bus.read8(self.dp_index(dp, self.x));
                self.apply_alu_to_a(operation, value);
                Some(4)
            }
            (5, true) => {
                let address = self.fetch16(bus).wrapping_add(u16::from(self.x));
                let value = bus.read8(address);
                self.apply_alu_to_a(operation, value);
                Some(5)
            }
            (6, true) => {
                let address = self.fetch16(bus).wrapping_add(u16::from(self.y));
                let value = bus.read8(address);
                self.apply_alu_to_a(operation, value);
                Some(5)
            }
            (7, true) => {
                let dp = self.fetch8(bus);
                let address = self.indirect_y(bus, dp);
                let value = bus.read8(address);
                self.apply_alu_to_a(operation, value);
                Some(6)
            }
            (8, true) => Some(self.alu_dp_immediate(bus, operation)),
            (9, false) => Some(self.alu_dp_dp(bus, operation)),
            (9, true) => Some(self.alu_xy(bus, operation)),
            _ => None,
        }
    }
    fn alu_dp_immediate<B: Spc700Bus>(&mut self, bus: &mut B, operation: Alu) -> u32 {
        let immediate = self.fetch8(bus);
        let destination = self.fetch8(bus);
        let address = self.dp(destination);
        let left = bus.read8(address);
        let result = self.alu_value(operation, left, immediate);
        if !matches!(operation, Alu::Cmp) {
            bus.write8(address, result);
        }
        5
    }

    fn alu_dp_dp<B: Spc700Bus>(&mut self, bus: &mut B, operation: Alu) -> u32 {
        let source = self.fetch8(bus);
        let destination = self.fetch8(bus);
        let right = bus.read8(self.dp(source));
        let address = self.dp(destination);
        let left = bus.read8(address);
        let result = self.alu_value(operation, left, right);
        if !matches!(operation, Alu::Cmp) {
            bus.write8(address, result);
        }
        6
    }

    fn alu_xy<B: Spc700Bus>(&mut self, bus: &mut B, operation: Alu) -> u32 {
        let left_address = self.dp(self.x);
        let left = bus.read8(left_address);
        let right = bus.read8(self.dp(self.y));
        let result = self.alu_value(operation, left, right);
        if !matches!(operation, Alu::Cmp) {
            bus.write8(left_address, result);
        }
        5
    }
    pub fn step<B: Spc700Bus>(&mut self, bus: &mut B) -> u32 {
        if self.stopped || self.sleeping {
            self.cycles = self.cycles.wrapping_add(2);
            return 2;
        }
        let opcode = self.fetch8(bus);
        let used = if opcode & 0x0f == 0x01 {
            self.tcall(bus, opcode >> 4)
        } else if opcode & 0x0f == 0x02 {
            self.set_clear_bit(bus, opcode)
        } else if opcode & 0x0f == 0x03 {
            self.branch_bit(bus, opcode)
        } else if opcode & 0x1f == 0x10 {
            self.conditional_branch(bus, opcode)
        } else if let Some(cycles) = self.execute_alu_family(bus, opcode) {
            cycles
        } else if let Some(cycles) = self.execute_rmw_family(bus, opcode) {
            cycles
        } else {
            self.execute_special(bus, opcode)
        };
        self.cycles = self.cycles.wrapping_add(u64::from(used));
        used
    }

    fn tcall<B: Spc700Bus>(&mut self, bus: &mut B, number: u8) -> u32 {
        self.push16(bus, self.pc);
        let vector = 0xffdeu16.wrapping_sub(u16::from(number) * 2);
        self.pc = self.read16(bus, vector);
        8
    }
    fn set_clear_bit<B: Spc700Bus>(&mut self, bus: &mut B, opcode: u8) -> u32 {
        let offset = self.fetch8(bus);
        let address = self.dp(offset);
        let bit = (opcode >> 5) & 7;
        let mut value = bus.read8(address);
        if opcode & 0x10 == 0 {
            value |= 1 << bit;
        } else {
            value &= !(1 << bit);
        }
        bus.write8(address, value);
        4
    }

    fn branch_bit<B: Spc700Bus>(&mut self, bus: &mut B, opcode: u8) -> u32 {
        let offset = self.fetch8(bus);
        let relative = self.fetch8(bus);
        let bit = (opcode >> 5) & 7;
        let set = bus.read8(self.dp(offset)) & (1 << bit) != 0;
        let take = if opcode & 0x10 == 0 { set } else { !set };
        self.branch(relative, take) + 3
    }

    fn conditional_branch<B: Spc700Bus>(&mut self, bus: &mut B, opcode: u8) -> u32 {
        let relative = self.fetch8(bus);
        let take = match opcode {
            0x10 => self.psw & N == 0,
            0x30 => self.psw & N != 0,
            0x50 => self.psw & V == 0,
            0x70 => self.psw & V != 0,
            0x90 => self.psw & C == 0,
            0xb0 => self.psw & C != 0,
            0xd0 => self.psw & Z == 0,
            0xf0 => self.psw & Z != 0,
            _ => false,
        };
        self.branch(relative, take)
    }
    fn rmw_value(&mut self, kind: u8, value: u8) -> u8 {
        let result = match kind {
            0 => {
                self.set_flag(C, value & 0x80 != 0);
                value << 1
            }
            1 => {
                let carry = u8::from(self.psw & C != 0);
                self.set_flag(C, value & 0x80 != 0);
                (value << 1) | carry
            }
            2 => {
                self.set_flag(C, value & 1 != 0);
                value >> 1
            }
            3 => {
                let carry = u8::from(self.psw & C != 0) << 7;
                self.set_flag(C, value & 1 != 0);
                (value >> 1) | carry
            }
            4 => value.wrapping_sub(1),
            5 => value.wrapping_add(1),
            _ => value,
        };
        self.set_nz(result);
        result
    }

    fn execute_rmw_family<B: Spc700Bus>(&mut self, bus: &mut B, opcode: u8) -> Option<u32> {
        let (kind, mode) = match opcode {
            0x0b => (0, 0),
            0x0c => (0, 1),
            0x1b => (0, 2),
            0x1c => (0, 3),
            0x2b => (1, 0),
            0x2c => (1, 1),
            0x3b => (1, 2),
            0x3c => (1, 3),
            0x4b => (2, 0),
            0x4c => (2, 1),
            0x5b => (2, 2),
            0x5c => (2, 3),
            0x6b => (3, 0),
            0x6c => (3, 1),
            0x7b => (3, 2),
            0x7c => (3, 3),
            0x8b => (4, 0),
            0x8c => (4, 1),
            0x9b => (4, 2),
            0x9c => (4, 3),
            0xab => (5, 0),
            0xac => (5, 1),
            0xbb => (5, 2),
            0xbc => (5, 3),
            _ => return None,
        };
        if mode == 3 {
            self.a = self.rmw_value(kind, self.a);
            return Some(2);
        }
        let address = match mode {
            0 => {
                let dp = self.fetch8(bus);
                self.dp(dp)
            }
            1 => self.fetch16(bus),
            2 => {
                let dp = self.fetch8(bus);
                self.dp_index(dp, self.x)
            }
            _ => unreachable!(),
        };
        let value = bus.read8(address);
        let result = self.rmw_value(kind, value);
        bus.write8(address, result);
        Some(if mode == 1 { 5 } else { 4 })
    }

    fn bit_operand<B: Spc700Bus>(&mut self, bus: &mut B) -> (u16, u8) {
        let encoded = self.fetch16(bus);
        (encoded & 0x1fff, (encoded >> 13) as u8)
    }

    fn call<B: Spc700Bus>(&mut self, bus: &mut B, address: u16) {
        self.push16(bus, self.pc);
        self.pc = address;
    }
    fn ya(&self) -> u16 {
        u16::from(self.a) | (u16::from(self.y) << 8)
    }

    fn set_ya(&mut self, value: u16) {
        self.a = value as u8;
        self.y = (value >> 8) as u8;
    }

    fn execute_special<B: Spc700Bus>(&mut self, bus: &mut B, opcode: u8) -> u32 {
        match opcode {
            0x00 => 2,
            0x0d => {
                self.push8(bus, self.psw);
                4
            }
            0x2d => {
                self.push8(bus, self.a);
                4
            }
            0x4d => {
                self.push8(bus, self.x);
                4
            }
            0x6d => {
                self.push8(bus, self.y);
                4
            }
            0x8e => {
                self.psw = self.pop8(bus);
                4
            }
            0xae => {
                self.a = self.pop8(bus);
                4
            }
            0xce => {
                self.x = self.pop8(bus);
                4
            }
            0xee => {
                self.y = self.pop8(bus);
                4
            }
            0x20 => {
                self.psw &= !P;
                2
            }
            0x40 => {
                self.psw |= P;
                2
            }
            0x60 => {
                self.psw &= !C;
                2
            }
            0x80 => {
                self.psw |= C;
                2
            }
            0xed => {
                self.psw ^= C;
                3
            }
            0xe0 => {
                self.psw &= !(V | H);
                2
            }
            0xa0 => {
                self.psw |= I;
                3
            }
            0xc0 => {
                self.psw &= !I;
                3
            }
            _ => self.execute_special2(bus, opcode),
        }
    }
    fn execute_special2<B: Spc700Bus>(&mut self, bus: &mut B, opcode: u8) -> u32 {
        match opcode {
            0x2f => {
                let rel = self.fetch8(bus);
                self.branch(rel, true)
            }
            0x2e => self.cbne(bus, false),
            0xde => self.cbne(bus, true),
            0x6e => self.dbnz_dp(bus),
            0xfe => {
                let rel = self.fetch8(bus);
                self.y = self.y.wrapping_sub(1);
                self.set_nz(self.y);
                self.branch(rel, self.y != 0) + 2
            }
            0x3f => {
                let address = self.fetch16(bus);
                self.call(bus, address);
                8
            }
            0x4f => {
                let low = self.fetch8(bus);
                self.call(bus, 0xff00 | u16::from(low));
                6
            }
            0x5f => {
                self.pc = self.fetch16(bus);
                3
            }
            0x1f => {
                let base = self.fetch16(bus).wrapping_add(u16::from(self.x));
                self.pc = self.read16(bus, base);
                6
            }
            0x6f => {
                self.pc = self.pop16(bus);
                5
            }
            0x7f => {
                self.psw = self.pop8(bus);
                self.pc = self.pop16(bus);
                6
            }
            0x0f => {
                self.push16(bus, self.pc);
                self.push8(bus, self.psw | B);
                self.psw |= B;
                self.pc = self.read16(bus, 0xffde);
                8
            }
            0xef => {
                self.sleeping = true;
                3
            }
            0xff => {
                self.stopped = true;
                3
            }
            0x0e => self.tset_tclr(bus, true),
            0x4e => self.tset_tclr(bus, false),
            _ => self.execute_special3(bus, opcode),
        }
    }

    fn cbne<B: Spc700Bus>(&mut self, bus: &mut B, indexed: bool) -> u32 {
        let dp = self.fetch8(bus);
        let rel = self.fetch8(bus);
        let address = if indexed {
            self.dp_index(dp, self.x)
        } else {
            self.dp(dp)
        };
        let value = bus.read8(address);
        self.branch(rel, self.a != value) + 3
    }
    fn dbnz_dp<B: Spc700Bus>(&mut self, bus: &mut B) -> u32 {
        let dp = self.fetch8(bus);
        let rel = self.fetch8(bus);
        let address = self.dp(dp);
        let value = bus.read8(address).wrapping_sub(1);
        bus.write8(address, value);
        self.branch(rel, value != 0) + 3
    }

    fn tset_tclr<B: Spc700Bus>(&mut self, bus: &mut B, set: bool) -> u32 {
        let address = self.fetch16(bus);
        let value = bus.read8(address);
        self.set_nz(self.a.wrapping_sub(value));
        bus.write8(address, if set { value | self.a } else { value & !self.a });
        6
    }

    fn execute_special3<B: Spc700Bus>(&mut self, bus: &mut B, opcode: u8) -> u32 {
        match opcode {
            0x0a | 0x2a | 0x4a | 0x6a | 0x8a | 0xaa | 0xca | 0xea => {
                self.bit_instruction(bus, opcode)
            }
            0x1a => self.word_inc_dec(bus, false),
            0x3a => self.word_inc_dec(bus, true),
            0x5a => self.word_compare(bus),
            0x7a => self.word_add_sub(bus, true),
            0x9a => self.word_add_sub(bus, false),
            0xba => self.movw_load(bus),
            0xda => self.movw_store(bus),
            0xcf => {
                let product = u16::from(self.a) * u16::from(self.y);
                self.set_ya(product);
                self.set_nz(self.y);
                9
            }
            0x9e => self.divide(),
            0x9f => {
                self.a = self.a.rotate_left(4);
                self.set_nz(self.a);
                5
            }
            0xdf => {
                self.decimal_adjust(true);
                3
            }
            0xbe => {
                self.decimal_adjust(false);
                3
            }
            _ => self.execute_moves(bus, opcode),
        }
    }
    fn bit_instruction<B: Spc700Bus>(&mut self, bus: &mut B, opcode: u8) -> u32 {
        let (address, bit) = self.bit_operand(bus);
        let mask = 1u8 << bit;
        let value = bus.read8(address);
        let bit_set = value & mask != 0;
        match opcode {
            0x0a => self.set_flag(C, self.psw & C != 0 || bit_set),
            0x2a => self.set_flag(C, self.psw & C != 0 || !bit_set),
            0x4a => self.set_flag(C, self.psw & C != 0 && bit_set),
            0x6a => self.set_flag(C, self.psw & C != 0 && !bit_set),
            0x8a => self.set_flag(C, (self.psw & C != 0) ^ bit_set),
            0xaa => self.set_flag(C, bit_set),
            0xca => {
                let next = if self.psw & C != 0 {
                    value | mask
                } else {
                    value & !mask
                };
                bus.write8(address, next);
            }
            0xea => bus.write8(address, value ^ mask),
            _ => {}
        }
        5
    }

    fn word_inc_dec<B: Spc700Bus>(&mut self, bus: &mut B, increment: bool) -> u32 {
        let dp = self.fetch8(bus);
        let address = self.dp(dp);
        let value = self.read_dp16(bus, dp);
        let result = if increment {
            value.wrapping_add(1)
        } else {
            value.wrapping_sub(1)
        };
        self.write16(bus, address, result);
        self.set_nz16(result);
        6
    }
    fn word_compare<B: Spc700Bus>(&mut self, bus: &mut B) -> u32 {
        let dp = self.fetch8(bus);
        let right = self.read_dp16(bus, dp);
        let left = self.ya();
        let result = left.wrapping_sub(right);
        self.set_flag(C, left >= right);
        self.set_nz16(result);
        4
    }

    fn word_add_sub<B: Spc700Bus>(&mut self, bus: &mut B, add: bool) -> u32 {
        let dp = self.fetch8(bus);
        let right = self.read_dp16(bus, dp);
        let left = self.ya();
        let result = if add {
            left.wrapping_add(right)
        } else {
            left.wrapping_sub(right)
        };
        if add {
            self.set_flag(C, u32::from(left) + u32::from(right) > 0xffff);
            self.set_flag(H, (left & 0x0fff) + (right & 0x0fff) > 0x0fff);
            self.set_flag(V, (!(left ^ right) & (left ^ result) & 0x8000) != 0);
        } else {
            self.set_flag(C, left >= right);
            self.set_flag(H, (left & 0x0fff) >= (right & 0x0fff));
            self.set_flag(V, ((left ^ right) & (left ^ result) & 0x8000) != 0);
        }
        self.set_ya(result);
        self.set_nz16(result);
        5
    }

    fn movw_load<B: Spc700Bus>(&mut self, bus: &mut B) -> u32 {
        let dp = self.fetch8(bus);
        let value = self.read_dp16(bus, dp);
        self.set_ya(value);
        self.set_nz16(value);
        5
    }

    fn movw_store<B: Spc700Bus>(&mut self, bus: &mut B) -> u32 {
        let dp = self.fetch8(bus);
        self.write16(bus, self.dp(dp), self.ya());
        5
    }

    fn divide(&mut self) -> u32 {
        self.set_flag(V, self.y >= self.x);
        self.set_flag(H, (self.y & 0x0f) >= (self.x & 0x0f));
        if self.x == 0 {
            self.a = 0xff;
            self.y = 0xff;
        } else if self.y < self.x.wrapping_mul(2) {
            let ya = self.ya();
            self.a = (ya / u16::from(self.x)) as u8;
            self.y = (ya % u16::from(self.x)) as u8;
        } else {
            let ya = self.ya();
            let numerator = ya.wrapping_sub(u16::from(self.x) << 9);
            let denominator = 0x100u16.wrapping_sub(u16::from(self.x));
            self.a = (0xffu16.wrapping_sub(numerator / denominator)) as u8;
            self.y = (u16::from(self.x) + numerator % denominator) as u8;
        }
        self.set_nz(self.a);
        12
    }
    fn decimal_adjust(&mut self, add: bool) {
        if add {
            if self.psw & C != 0 || self.a > 0x99 {
                self.a = self.a.wrapping_add(0x60);
                self.psw |= C;
            }
            if self.psw & H != 0 || self.a & 0x0f > 9 {
                self.a = self.a.wrapping_add(0x06);
            }
        } else {
            if self.psw & C == 0 || self.a > 0x99 {
                self.a = self.a.wrapping_sub(0x60);
                self.psw &= !C;
            }
            if self.psw & H == 0 || self.a & 0x0f > 9 {
                self.a = self.a.wrapping_sub(0x06);
            }
        }
        self.set_nz(self.a);
    }

    fn execute_moves<B: Spc700Bus>(&mut self, bus: &mut B, opcode: u8) -> u32 {
        match opcode {
            0xc4 => {
                let dp = self.fetch8(bus);
                bus.write8(self.dp(dp), self.a);
                4
            }
            0xc5 => {
                let address = self.fetch16(bus);
                bus.write8(address, self.a);
                5
            }
            0xc6 => {
                bus.write8(self.dp(self.x), self.a);
                4
            }
            0xc7 => {
                let dp = self.fetch8(bus);
                let address = self.indirect_x(bus, dp);
                bus.write8(address, self.a);
                7
            }
            0xd4 => {
                let dp = self.fetch8(bus);
                bus.write8(self.dp_index(dp, self.x), self.a);
                5
            }
            0xd5 => {
                let address = self.fetch16(bus).wrapping_add(u16::from(self.x));
                bus.write8(address, self.a);
                6
            }
            0xd6 => {
                let address = self.fetch16(bus).wrapping_add(u16::from(self.y));
                bus.write8(address, self.a);
                6
            }
            0xd7 => {
                let dp = self.fetch8(bus);
                let address = self.indirect_y(bus, dp);
                bus.write8(address, self.a);
                7
            }
            0xe4 => {
                let dp = self.fetch8(bus);
                self.a = bus.read8(self.dp(dp));
                self.set_nz(self.a);
                3
            }
            0xe5 => {
                let address = self.fetch16(bus);
                self.a = bus.read8(address);
                self.set_nz(self.a);
                4
            }
            0xe6 => {
                self.a = bus.read8(self.dp(self.x));
                self.set_nz(self.a);
                3
            }
            0xe7 => {
                let dp = self.fetch8(bus);
                let address = self.indirect_x(bus, dp);
                self.a = bus.read8(address);
                self.set_nz(self.a);
                6
            }
            0xe8 => {
                self.a = self.fetch8(bus);
                self.set_nz(self.a);
                2
            }
            0xf4 => {
                let dp = self.fetch8(bus);
                self.a = bus.read8(self.dp_index(dp, self.x));
                self.set_nz(self.a);
                4
            }
            0xf5 => {
                let address = self.fetch16(bus).wrapping_add(u16::from(self.x));
                self.a = bus.read8(address);
                self.set_nz(self.a);
                5
            }
            0xf6 => {
                let address = self.fetch16(bus).wrapping_add(u16::from(self.y));
                self.a = bus.read8(address);
                self.set_nz(self.a);
                5
            }
            0xf7 => {
                let dp = self.fetch8(bus);
                let address = self.indirect_y(bus, dp);
                self.a = bus.read8(address);
                self.set_nz(self.a);
                6
            }
            0xbf => {
                self.a = bus.read8(self.dp(self.x));
                self.x = self.x.wrapping_add(1);
                self.set_nz(self.a);
                4
            }
            0xaf => {
                bus.write8(self.dp(self.x), self.a);
                self.x = self.x.wrapping_add(1);
                4
            }
            0x8d => {
                self.y = self.fetch8(bus);
                self.set_nz(self.y);
                2
            }
            0xcd => {
                self.x = self.fetch8(bus);
                self.set_nz(self.x);
                2
            }
            0xf8 => {
                let dp = self.fetch8(bus);
                self.x = bus.read8(self.dp(dp));
                self.set_nz(self.x);
                3
            }
            0xf9 => {
                let dp = self.fetch8(bus);
                self.x = bus.read8(self.dp_index(dp, self.y));
                self.set_nz(self.x);
                4
            }
            0xe9 => {
                let address = self.fetch16(bus);
                self.x = bus.read8(address);
                self.set_nz(self.x);
                4
            }
            0xeb => {
                let dp = self.fetch8(bus);
                self.y = bus.read8(self.dp(dp));
                self.set_nz(self.y);
                3
            }
            0xfb => {
                let dp = self.fetch8(bus);
                self.y = bus.read8(self.dp_index(dp, self.x));
                self.set_nz(self.y);
                4
            }
            0xec => {
                let address = self.fetch16(bus);
                self.y = bus.read8(address);
                self.set_nz(self.y);
                4
            }
            _ => self.execute_moves2(bus, opcode),
        }
    }
    fn execute_moves2<B: Spc700Bus>(&mut self, bus: &mut B, opcode: u8) -> u32 {
        match opcode {
            0xc8 => {
                let value = self.fetch8(bus);
                self.compare(self.x, value);
                2
            }
            0xc9 => {
                let address = self.fetch16(bus);
                bus.write8(address, self.x);
                5
            }
            0xcb => {
                let dp = self.fetch8(bus);
                bus.write8(self.dp(dp), self.y);
                4
            }
            0xcc => {
                let address = self.fetch16(bus);
                bus.write8(address, self.y);
                5
            }
            0xd8 => {
                let dp = self.fetch8(bus);
                bus.write8(self.dp(dp), self.x);
                4
            }
            0xd9 => {
                let dp = self.fetch8(bus);
                bus.write8(self.dp_index(dp, self.y), self.x);
                5
            }
            0xdb => {
                let dp = self.fetch8(bus);
                bus.write8(self.dp_index(dp, self.x), self.y);
                5
            }
            0x8f => {
                let value = self.fetch8(bus);
                let dp = self.fetch8(bus);
                bus.write8(self.dp(dp), value);
                5
            }
            0xfa => {
                let source = self.fetch8(bus);
                let destination = self.fetch8(bus);
                let value = bus.read8(self.dp(source));
                bus.write8(self.dp(destination), value);
                5
            }
            0x1d => {
                self.x = self.x.wrapping_sub(1);
                self.set_nz(self.x);
                2
            }
            0x3d => {
                self.x = self.x.wrapping_add(1);
                self.set_nz(self.x);
                2
            }
            0xdc => {
                self.y = self.y.wrapping_sub(1);
                self.set_nz(self.y);
                2
            }
            0xfc => {
                self.y = self.y.wrapping_add(1);
                self.set_nz(self.y);
                2
            }
            0x5d => {
                self.x = self.a;
                self.set_nz(self.x);
                2
            }
            0x7d => {
                self.a = self.x;
                self.set_nz(self.a);
                2
            }
            0x9d => {
                self.x = self.sp;
                self.set_nz(self.x);
                2
            }
            0xbd => {
                self.sp = self.x;
                2
            }
            0xdd => {
                self.a = self.y;
                self.set_nz(self.a);
                2
            }
            0xfd => {
                self.y = self.a;
                self.set_nz(self.y);
                2
            }
            _ => self.execute_compare_and_misc(bus, opcode),
        }
    }
    fn execute_compare_and_misc<B: Spc700Bus>(&mut self, bus: &mut B, opcode: u8) -> u32 {
        match opcode {
            0x1e => {
                let address = self.fetch16(bus);
                let value = bus.read8(address);
                self.compare(self.x, value);
                4
            }
            0x3e => {
                let dp = self.fetch8(bus);
                let value = bus.read8(self.dp(dp));
                self.compare(self.x, value);
                3
            }
            0x5e => {
                let address = self.fetch16(bus);
                let value = bus.read8(address);
                self.compare(self.y, value);
                4
            }
            0x7e => {
                let dp = self.fetch8(bus);
                let value = bus.read8(self.dp(dp));
                self.compare(self.y, value);
                3
            }
            0xad => {
                let value = self.fetch8(bus);
                self.compare(self.y, value);
                2
            }
            _ => panic!("unhandled SPC700 opcode {opcode:02x}"),
        }
    }

    pub fn save(&self, out: &mut StateWriter) {
        out.u8(self.a);
        out.u8(self.x);
        out.u8(self.y);
        out.u8(self.sp);
        out.u16(self.pc);
        out.u8(self.psw);
        out.u64(self.cycles);
        out.u8(self.sleeping as u8);
        out.u8(self.stopped as u8);
    }

    pub fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.a = input.u8()?;
        self.x = input.u8()?;
        self.y = input.u8()?;
        self.sp = input.u8()?;
        self.pc = input.u16()?;
        self.psw = input.u8()?;
        self.cycles = input.u64()?;
        self.sleeping = input.u8()? != 0;
        self.stopped = input.u8()? != 0;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct Ram([u8; 65536]);
    impl Default for Ram {
        fn default() -> Self {
            Self([0; 65536])
        }
    }
    impl Spc700Bus for Ram {
        fn read8(&mut self, address: u16) -> u8 {
            self.0[usize::from(address)]
        }
        fn write8(&mut self, address: u16, value: u8) {
            self.0[usize::from(address)] = value;
        }
    }

    #[test]
    fn immediate_arithmetic_and_direct_page_execute() {
        let mut bus = Ram::default();
        bus.0[0x200..0x208].copy_from_slice(&[0xe8, 0x10, 0x88, 0x22, 0xc4, 0x40, 0xe4, 0x40]);
        let mut cpu = Spc700 {
            pc: 0x200,
            ..Default::default()
        };
        for _ in 0..4 {
            cpu.step(&mut bus);
        }
        assert_eq!(cpu.a, 0x32);
        assert_eq!(bus.0[0x40], 0x32);
    }
    #[test]
    fn word_math_mul_and_div_execute() {
        let mut bus = Ram::default();
        bus.0[0x20] = 0x34;
        bus.0[0x21] = 0x12;
        bus.0[0x200..0x205].copy_from_slice(&[0xba, 0x20, 0xcf, 0xcd, 0x10]);
        let mut cpu = Spc700 {
            pc: 0x200,
            ..Default::default()
        };
        cpu.step(&mut bus);
        assert_eq!(cpu.ya(), 0x1234);
        cpu.step(&mut bus);
        assert_eq!(cpu.ya(), 0x34 * 0x12);
        cpu.step(&mut bus);
        cpu.step(&mut bus);
        assert_ne!(cpu.a, 0);
    }

    #[test]
    fn every_opcode_has_an_execution_path() {
        for opcode in 0u16..=255 {
            let mut bus = Ram::default();
            bus.0[0x200] = opcode as u8;
            bus.0[0x201] = 0;
            bus.0[0x202] = 0;
            for vector in (0xffc0..=0xfffe).step_by(2) {
                bus.0[vector] = 0x00;
                bus.0[vector + 1] = 0x02;
            }
            let mut cpu = Spc700 {
                pc: 0x200,
                x: 1,
                y: 1,
                ..Default::default()
            };
            let used = cpu.step(&mut bus);
            assert!(used > 0, "opcode {opcode:02x}");
        }
    }
}
