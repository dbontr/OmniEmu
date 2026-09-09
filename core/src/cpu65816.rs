pub trait Bus65816: Send {
    fn read8(&mut self, address: u32) -> u8;
    fn write8(&mut self, address: u32, value: u8);
}

const N: u8 = 0x80;
const V: u8 = 0x40;
const M: u8 = 0x20;
const X: u8 = 0x10;
const D: u8 = 0x08;
const I: u8 = 0x04;
const Z: u8 = 0x02;
const C: u8 = 0x01;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum AddressMode {
    DirectXIndirect,
    StackRelative,
    Direct,
    DirectLongIndirect,
    Absolute,
    Long,
    DirectIndirectY,
    DirectIndirect,
    StackRelativeIndirectY,
    DirectX,
    DirectLongIndirectY,
    AbsoluteY,
    AbsoluteX,
    LongX,
}

#[derive(Debug, Clone)]
pub struct Cpu65816 {
    pub a: u16,
    pub x: u16,
    pub y: u16,
    pub sp: u16,
    pub d: u16,
    pub pc: u16,
    pub pbr: u8,
    pub dbr: u8,
    pub p: u8,
    pub emulation: bool,
    pub cycles: u64,
    pub stopped: bool,
    pub waiting: bool,
}

impl Default for Cpu65816 {
    fn default() -> Self {
        Self {
            a: 0,
            x: 0,
            y: 0,
            sp: 0x01ff,
            d: 0,
            pc: 0,
            pbr: 0,
            dbr: 0,
            p: M | X | I,
            emulation: true,
            cycles: 0,
            stopped: false,
            waiting: false,
        }
    }
}

impl Cpu65816 {
    pub fn reset<B: Bus65816>(&mut self, bus: &mut B) {
        *self = Self::default();
        self.pc = self.read16_bank0(bus, 0xfffc);
        self.cycles = 8;
    }

    pub fn accumulator_is_8_bit(&self) -> bool {
        self.emulation || self.p & M != 0
    }

    pub fn index_is_8_bit(&self) -> bool {
        self.emulation || self.p & X != 0
    }

    fn set_flag(&mut self, flag: u8, enabled: bool) {
        if enabled {
            self.p |= flag;
        } else {
            self.p &= !flag;
        }
    }

    fn normalize_mode(&mut self) {
        if self.emulation {
            self.p |= M | X;
            self.sp = 0x0100 | (self.sp & 0x00ff);
        }
        if self.index_is_8_bit() {
            self.x &= 0x00ff;
            self.y &= 0x00ff;
        }
    }

    fn set_zn8(&mut self, value: u8) {
        self.set_flag(Z, value == 0);
        self.set_flag(N, value & 0x80 != 0);
    }

    fn set_zn16(&mut self, value: u16) {
        self.set_flag(Z, value == 0);
        self.set_flag(N, value & 0x8000 != 0);
    }

    fn program_address(&self) -> u32 {
        (u32::from(self.pbr) << 16) | u32::from(self.pc)
    }

    fn fetch8<B: Bus65816>(&mut self, bus: &mut B) -> u8 {
        let value = bus.read8(self.program_address());
        self.pc = self.pc.wrapping_add(1);
        value
    }

    fn fetch16<B: Bus65816>(&mut self, bus: &mut B) -> u16 {
        let lo = self.fetch8(bus);
        let hi = self.fetch8(bus);
        u16::from_le_bytes([lo, hi])
    }

    fn fetch24<B: Bus65816>(&mut self, bus: &mut B) -> u32 {
        let lo = u32::from(self.fetch8(bus));
        let hi = u32::from(self.fetch8(bus));
        let bank = u32::from(self.fetch8(bus));
        lo | (hi << 8) | (bank << 16)
    }

    fn read16_bank0<B: Bus65816>(&self, bus: &mut B, address: u16) -> u16 {
        let lo = bus.read8(u32::from(address));
        let hi = bus.read8(u32::from(address.wrapping_add(1)));
        u16::from_le_bytes([lo, hi])
    }

    fn read16_same_bank<B: Bus65816>(&self, bus: &mut B, address: u32) -> u16 {
        let bank = address & 0xff0000;
        let low = address as u16;
        let lo = bus.read8(bank | u32::from(low));
        let hi = bus.read8(bank | u32::from(low.wrapping_add(1)));
        u16::from_le_bytes([lo, hi])
    }

    fn read24_bank0<B: Bus65816>(&self, bus: &mut B, address: u16) -> u32 {
        let lo = u32::from(bus.read8(u32::from(address)));
        let hi = u32::from(bus.read8(u32::from(address.wrapping_add(1))));
        let bank = u32::from(bus.read8(u32::from(address.wrapping_add(2))));
        lo | (hi << 8) | (bank << 16)
    }

    fn write16_same_bank<B: Bus65816>(&self, bus: &mut B, address: u32, value: u16) {
        let bank = address & 0xff0000;
        let low = address as u16;
        let [lo, hi] = value.to_le_bytes();
        bus.write8(bank | u32::from(low), lo);
        bus.write8(bank | u32::from(low.wrapping_add(1)), hi);
    }

    fn stack_address(&self) -> u32 {
        u32::from(self.sp)
    }

    fn push8<B: Bus65816>(&mut self, bus: &mut B, value: u8) {
        bus.write8(self.stack_address(), value);
        self.sp = self.sp.wrapping_sub(1);
        if self.emulation {
            self.sp = 0x0100 | (self.sp & 0x00ff);
        }
    }

    fn pull8<B: Bus65816>(&mut self, bus: &mut B) -> u8 {
        self.sp = self.sp.wrapping_add(1);
        if self.emulation {
            self.sp = 0x0100 | (self.sp & 0x00ff);
        }
        bus.read8(self.stack_address())
    }

    fn push16<B: Bus65816>(&mut self, bus: &mut B, value: u16) {
        self.push8(bus, (value >> 8) as u8);
        self.push8(bus, value as u8);
    }

    fn pull16<B: Bus65816>(&mut self, bus: &mut B) -> u16 {
        let lo = self.pull8(bus);
        let hi = self.pull8(bus);
        u16::from_le_bytes([lo, hi])
    }

    fn direct_address(&self, operand: u8) -> u16 {
        self.d.wrapping_add(u16::from(operand))
    }

    fn data_address(&self, address: u16) -> u32 {
        (u32::from(self.dbr) << 16) | u32::from(address)
    }

    fn index_x(&self) -> u16 {
        if self.index_is_8_bit() {
            self.x & 0xff
        } else {
            self.x
        }
    }

    fn index_y(&self) -> u16 {
        if self.index_is_8_bit() {
            self.y & 0xff
        } else {
            self.y
        }
    }

    fn resolve_address<B: Bus65816>(&mut self, bus: &mut B, mode: AddressMode) -> (u32, u32) {
        match mode {
            AddressMode::DirectXIndirect => {
                let operand = self.fetch8(bus);
                let pointer = self.direct_address(operand).wrapping_add(self.index_x());
                let address = self.read16_bank0(bus, pointer);
                (self.data_address(address), 6)
            }
            AddressMode::StackRelative => {
                let operand = self.fetch8(bus);
                (u32::from(self.sp.wrapping_add(u16::from(operand))), 4)
            }
            AddressMode::Direct => {
                let operand = self.fetch8(bus);
                (u32::from(self.direct_address(operand)), 3)
            }
            AddressMode::DirectLongIndirect => {
                let operand = self.fetch8(bus);
                let pointer = self.direct_address(operand);
                (self.read24_bank0(bus, pointer), 6)
            }
            AddressMode::Absolute => {
                let address = self.fetch16(bus);
                (self.data_address(address), 4)
            }
            AddressMode::Long => (self.fetch24(bus), 5),
            AddressMode::DirectIndirectY => {
                let operand = self.fetch8(bus);
                let pointer = self.read16_bank0(bus, self.direct_address(operand));
                let address = pointer.wrapping_add(self.index_y());
                (self.data_address(address), 5)
            }
            AddressMode::DirectIndirect => {
                let operand = self.fetch8(bus);
                let pointer = self.read16_bank0(bus, self.direct_address(operand));
                (self.data_address(pointer), 5)
            }
            AddressMode::StackRelativeIndirectY => {
                let operand = self.fetch8(bus);
                let pointer = self.read16_bank0(bus, self.sp.wrapping_add(u16::from(operand)));
                let address = pointer.wrapping_add(self.index_y());
                (self.data_address(address), 7)
            }
            AddressMode::DirectX => {
                let operand = self.fetch8(bus);
                let address = self.direct_address(operand).wrapping_add(self.index_x());
                (u32::from(address), 4)
            }
            AddressMode::DirectLongIndirectY => {
                let operand = self.fetch8(bus);
                let pointer = self.read24_bank0(bus, self.direct_address(operand));
                ((pointer + u32::from(self.index_y())) & 0x00ff_ffff, 6)
            }
            AddressMode::AbsoluteY => {
                let address = self.fetch16(bus).wrapping_add(self.index_y());
                (self.data_address(address), 4)
            }
            AddressMode::AbsoluteX => {
                let address = self.fetch16(bus).wrapping_add(self.index_x());
                (self.data_address(address), 4)
            }
            AddressMode::LongX => {
                let address = (self.fetch24(bus) + u32::from(self.index_x())) & 0x00ff_ffff;
                (address, 5)
            }
        }
    }

    fn read_m<B: Bus65816>(&self, bus: &mut B, address: u32) -> u16 {
        if self.accumulator_is_8_bit() {
            u16::from(bus.read8(address & 0x00ff_ffff))
        } else {
            self.read16_same_bank(bus, address & 0x00ff_ffff)
        }
    }

    fn write_m<B: Bus65816>(&self, bus: &mut B, address: u32, value: u16) {
        if self.accumulator_is_8_bit() {
            bus.write8(address & 0x00ff_ffff, value as u8);
        } else {
            self.write16_same_bank(bus, address & 0x00ff_ffff, value);
        }
    }

    fn accumulator(&self) -> u16 {
        if self.accumulator_is_8_bit() {
            self.a & 0x00ff
        } else {
            self.a
        }
    }

    fn set_accumulator(&mut self, value: u16) {
        if self.accumulator_is_8_bit() {
            self.a = (self.a & 0xff00) | (value & 0x00ff);
            self.set_zn8(value as u8);
        } else {
            self.a = value;
            self.set_zn16(value);
        }
    }

    fn fetch_m<B: Bus65816>(&mut self, bus: &mut B) -> u16 {
        if self.accumulator_is_8_bit() {
            u16::from(self.fetch8(bus))
        } else {
            self.fetch16(bus)
        }
    }

    fn alu_mode(opcode: u8) -> Option<AddressMode> {
        Some(match opcode & 0x1f {
            0x01 => AddressMode::DirectXIndirect,
            0x03 => AddressMode::StackRelative,
            0x05 => AddressMode::Direct,
            0x07 => AddressMode::DirectLongIndirect,
            0x0d => AddressMode::Absolute,
            0x0f => AddressMode::Long,
            0x11 => AddressMode::DirectIndirectY,
            0x12 => AddressMode::DirectIndirect,
            0x13 => AddressMode::StackRelativeIndirectY,
            0x15 => AddressMode::DirectX,
            0x17 => AddressMode::DirectLongIndirectY,
            0x19 => AddressMode::AbsoluteY,
            0x1d => AddressMode::AbsoluteX,
            0x1f => AddressMode::LongX,
            _ => return None,
        })
    }

    fn is_alu_opcode(opcode: u8) -> bool {
        if Self::alu_mode(opcode).is_some() {
            return true;
        }
        opcode & 0x1f == 0x09 && opcode & 0xe0 != 0x80
    }

    fn compare_m(&mut self, rhs: u16) {
        if self.accumulator_is_8_bit() {
            let lhs = self.a as u8;
            let rhs = rhs as u8;
            self.set_flag(C, lhs >= rhs);
            self.set_zn8(lhs.wrapping_sub(rhs));
        } else {
            self.set_flag(C, self.a >= rhs);
            self.set_zn16(self.a.wrapping_sub(rhs));
        }
    }

    fn bcd_add(lhs: u16, rhs: u16, digits: usize, carry_in: bool) -> (u16, bool) {
        let mut result = 0u16;
        let mut carry = u16::from(carry_in);
        for digit in 0..digits {
            let shift = digit * 4;
            let mut sum = ((lhs >> shift) & 0x0f) + ((rhs >> shift) & 0x0f) + carry;
            if sum > 9 {
                sum += 6;
            }
            carry = u16::from(sum > 0x0f);
            result |= (sum & 0x0f) << shift;
        }
        (result, carry != 0)
    }

    fn bcd_sub(lhs: u16, rhs: u16, digits: usize, carry_in: bool) -> (u16, bool) {
        let mut result = 0u16;
        let mut borrow = if carry_in { 0i16 } else { 1i16 };
        for digit in 0..digits {
            let shift = digit * 4;
            let a = ((lhs >> shift) & 0x0f) as i16;
            let b = ((rhs >> shift) & 0x0f) as i16;
            let raw = a - b - borrow;
            let (value, next_borrow) = if raw < 0 { (raw - 6, 1) } else { (raw, 0) };
            result |= ((value as u16) & 0x0f) << shift;
            borrow = next_borrow;
        }
        (result, borrow == 0)
    }

    fn adc_m(&mut self, rhs: u16) {
        let eight = self.accumulator_is_8_bit();
        let mask = if eight { 0x00ffu32 } else { 0xffffu32 };
        let sign = if eight { 0x0080u32 } else { 0x8000u32 };
        let lhs = u32::from(self.accumulator());
        let rhs = u32::from(rhs) & mask;
        let carry = u32::from(self.p & C != 0);
        let binary = lhs + rhs + carry;
        let binary_result = binary & mask;
        self.set_flag(V, (!(lhs ^ rhs) & (lhs ^ binary_result) & sign) != 0);
        let (result, carry_out) = if self.p & D != 0 {
            let (bcd, c) = Self::bcd_add(
                lhs as u16,
                rhs as u16,
                if eight { 2 } else { 4 },
                carry != 0,
            );
            (bcd, c)
        } else {
            (binary_result as u16, binary > mask)
        };
        self.set_flag(C, carry_out);
        self.set_accumulator(result);
    }

    fn sbc_m(&mut self, rhs: u16) {
        let eight = self.accumulator_is_8_bit();
        let mask = if eight { 0x00ffu32 } else { 0xffffu32 };
        let sign = if eight { 0x0080u32 } else { 0x8000u32 };
        let lhs = u32::from(self.accumulator());
        let rhs = u32::from(rhs) & mask;
        let borrow = u32::from(self.p & C == 0);
        let binary_result = lhs.wrapping_sub(rhs + borrow) & mask;
        self.set_flag(V, ((lhs ^ rhs) & (lhs ^ binary_result) & sign) != 0);
        let (result, carry_out) = if self.p & D != 0 {
            Self::bcd_sub(
                lhs as u16,
                rhs as u16,
                if eight { 2 } else { 4 },
                borrow == 0,
            )
        } else {
            (binary_result as u16, lhs >= rhs + borrow)
        };
        self.set_flag(C, carry_out);
        self.set_accumulator(result);
    }

    fn execute_alu<B: Bus65816>(&mut self, bus: &mut B, opcode: u8) -> u32 {
        let group = opcode & 0xe0;
        let immediate = opcode & 0x1f == 0x09;
        let (value, address, mut cycles) = if immediate {
            (self.fetch_m(bus), None, 2)
        } else {
            let mode = Self::alu_mode(opcode).expect("ALU opcode requires addressing mode");
            let (address, cycles) = self.resolve_address(bus, mode);
            let value = if group == 0x80 {
                0
            } else {
                self.read_m(bus, address)
            };
            (value, Some(address), cycles)
        };
        match group {
            0x00 => self.set_accumulator(self.accumulator() | value),
            0x20 => self.set_accumulator(self.accumulator() & value),
            0x40 => self.set_accumulator(self.accumulator() ^ value),
            0x60 => self.adc_m(value),
            0x80 => self.write_m(
                bus,
                address.expect("STA requires address"),
                self.accumulator(),
            ),
            0xa0 => self.set_accumulator(value),
            0xc0 => self.compare_m(value),
            0xe0 => self.sbc_m(value),
            _ => unreachable!(),
        }
        if !self.accumulator_is_8_bit() {
            cycles += 1;
        }
        cycles
    }

    fn branch(&mut self, offset: i8, condition: bool) -> u32 {
        if !condition {
            return 2;
        }
        self.pc = self.pc.wrapping_add_signed(i16::from(offset));
        3
    }

    fn read_index<B: Bus65816>(&self, bus: &mut B, address: u32) -> u16 {
        if self.index_is_8_bit() {
            u16::from(bus.read8(address & 0x00ff_ffff))
        } else {
            self.read16_same_bank(bus, address & 0x00ff_ffff)
        }
    }

    fn write_index<B: Bus65816>(&self, bus: &mut B, address: u32, value: u16) {
        if self.index_is_8_bit() {
            bus.write8(address & 0x00ff_ffff, value as u8);
        } else {
            self.write16_same_bank(bus, address & 0x00ff_ffff, value);
        }
    }

    fn fetch_index<B: Bus65816>(&mut self, bus: &mut B) -> u16 {
        if self.index_is_8_bit() {
            u16::from(self.fetch8(bus))
        } else {
            self.fetch16(bus)
        }
    }

    fn set_x(&mut self, value: u16) {
        if self.index_is_8_bit() {
            self.x = value & 0xff;
            self.set_zn8(value as u8);
        } else {
            self.x = value;
            self.set_zn16(value);
        }
    }

    fn set_y(&mut self, value: u16) {
        if self.index_is_8_bit() {
            self.y = value & 0xff;
            self.set_zn8(value as u8);
        } else {
            self.y = value;
            self.set_zn16(value);
        }
    }

    fn compare_index(&mut self, lhs: u16, rhs: u16) {
        if self.index_is_8_bit() {
            let lhs = lhs as u8;
            let rhs = rhs as u8;
            self.set_flag(C, lhs >= rhs);
            self.set_zn8(lhs.wrapping_sub(rhs));
        } else {
            self.set_flag(C, lhs >= rhs);
            self.set_zn16(lhs.wrapping_sub(rhs));
        }
    }

    fn rmw_m<B: Bus65816>(&mut self, bus: &mut B, address: u32, operation: u8) {
        let value = self.read_m(bus, address);
        let width8 = self.accumulator_is_8_bit();
        let carry_in = u16::from(self.p & C != 0);
        let (result, carry_out) = match operation {
            0 => (
                value << 1,
                Some(value & if width8 { 0x80 } else { 0x8000 } != 0),
            ),
            1 => (
                (value << 1) | carry_in,
                Some(value & if width8 { 0x80 } else { 0x8000 } != 0),
            ),
            2 => (value >> 1, Some(value & 1 != 0)),
            3 => (
                (value >> 1)
                    | if self.p & C != 0 {
                        if width8 {
                            0x80
                        } else {
                            0x8000
                        }
                    } else {
                        0
                    },
                Some(value & 1 != 0),
            ),
            4 => (value.wrapping_sub(1), None),
            _ => (value.wrapping_add(1), None),
        };
        let result = if width8 { result & 0xff } else { result };
        if let Some(carry) = carry_out {
            self.set_flag(C, carry);
        }
        if width8 {
            self.set_zn8(result as u8);
        } else {
            self.set_zn16(result);
        }
        self.write_m(bus, address, result);
    }

    fn rmw_accumulator(&mut self, operation: u8) {
        let value = self.accumulator();
        let width8 = self.accumulator_is_8_bit();
        let carry_in = u16::from(self.p & C != 0);
        let (result, carry_out) = match operation {
            0 => (
                value << 1,
                Some(value & if width8 { 0x80 } else { 0x8000 } != 0),
            ),
            1 => (
                (value << 1) | carry_in,
                Some(value & if width8 { 0x80 } else { 0x8000 } != 0),
            ),
            2 => (value >> 1, Some(value & 1 != 0)),
            3 => (
                (value >> 1)
                    | if self.p & C != 0 {
                        if width8 {
                            0x80
                        } else {
                            0x8000
                        }
                    } else {
                        0
                    },
                Some(value & 1 != 0),
            ),
            4 => (value.wrapping_sub(1), None),
            _ => (value.wrapping_add(1), None),
        };
        let result = if width8 { result & 0xff } else { result };
        if let Some(carry) = carry_out {
            self.set_flag(C, carry);
        }
        self.set_accumulator(result);
    }

    fn bit_m(&mut self, value: u16, immediate: bool) {
        let accumulator = self.accumulator();
        self.set_flag(Z, accumulator & value == 0);
        if !immediate {
            if self.accumulator_is_8_bit() {
                self.set_flag(N, value & 0x80 != 0);
                self.set_flag(V, value & 0x40 != 0);
            } else {
                self.set_flag(N, value & 0x8000 != 0);
                self.set_flag(V, value & 0x4000 != 0);
            }
        }
    }

    fn enter_interrupt<B: Bus65816>(&mut self, bus: &mut B, vector: u16, software: bool) -> u32 {
        self.waiting = false;
        if self.emulation {
            self.push16(bus, self.pc);
            let pushed = if software {
                self.p | 0x10
            } else {
                self.p & !0x10
            };
            self.push8(bus, pushed);
        } else {
            self.push8(bus, self.pbr);
            self.push16(bus, self.pc);
            self.push8(bus, self.p);
        }
        self.p |= I;
        self.p &= !D;
        self.pbr = 0;
        self.pc = self.read16_bank0(bus, vector);
        if self.emulation {
            7
        } else {
            8
        }
    }

    pub fn nmi<B: Bus65816>(&mut self, bus: &mut B) -> u32 {
        if self.stopped {
            return 0;
        }
        let vector = if self.emulation { 0xfffa } else { 0xffea };
        let used = self.enter_interrupt(bus, vector, false);
        self.cycles = self.cycles.wrapping_add(u64::from(used));
        used
    }

    pub fn irq<B: Bus65816>(&mut self, bus: &mut B) -> u32 {
        if self.stopped {
            return 0;
        }
        if self.waiting {
            self.waiting = false;
        }
        if self.p & I != 0 {
            return 0;
        }
        let vector = if self.emulation { 0xfffe } else { 0xffee };
        let used = self.enter_interrupt(bus, vector, false);
        self.cycles = self.cycles.wrapping_add(u64::from(used));
        used
    }

    pub fn step<B: Bus65816>(&mut self, bus: &mut B) -> u32 {
        if self.stopped {
            return 0;
        }
        if self.waiting {
            self.cycles = self.cycles.wrapping_add(1);
            return 1;
        }
        let opcode = self.fetch8(bus);
        let used = if Self::is_alu_opcode(opcode) {
            self.execute_alu(bus, opcode)
        } else {
            self.execute_other(bus, opcode)
        };
        self.normalize_mode();
        self.cycles = self.cycles.wrapping_add(u64::from(used));
        used
    }

    fn execute_other<B: Bus65816>(&mut self, bus: &mut B, opcode: u8) -> u32 {
        match opcode {
            0x00 => {
                self.fetch8(bus);
                let vector = if self.emulation { 0xfffe } else { 0xffe6 };
                self.enter_interrupt(bus, vector, true)
            }
            0x02 => {
                self.fetch8(bus);
                let vector = if self.emulation { 0xfff4 } else { 0xffe4 };
                self.enter_interrupt(bus, vector, true)
            }
            0x08 => {
                self.push8(bus, self.p | if self.emulation { 0x30 } else { 0 });
                3
            }
            0x04 | 0x0c | 0x14 | 0x1c => {
                let (address, mut cycles) = match opcode {
                    0x04 | 0x14 => self.resolve_address(bus, AddressMode::Direct),
                    _ => self.resolve_address(bus, AddressMode::Absolute),
                };
                let value = self.read_m(bus, address);
                self.set_flag(Z, self.accumulator() & value == 0);
                let result = if matches!(opcode, 0x04 | 0x0c) {
                    value | self.accumulator()
                } else {
                    value & !self.accumulator()
                };
                self.write_m(bus, address, result);
                cycles += 2 + u32::from(!self.accumulator_is_8_bit());
                cycles
            }
            0x06 | 0x0e | 0x16 | 0x1e => {
                let (address, mut cycles) = match opcode {
                    0x06 => self.resolve_address(bus, AddressMode::Direct),
                    0x0e => self.resolve_address(bus, AddressMode::Absolute),
                    0x16 => self.resolve_address(bus, AddressMode::DirectX),
                    _ => self.resolve_address(bus, AddressMode::AbsoluteX),
                };
                self.rmw_m(bus, address, 0);
                cycles += 2 + u32::from(!self.accumulator_is_8_bit());
                cycles
            }
            0x0a => {
                self.rmw_accumulator(0);
                2
            }
            0x0b => {
                self.push16(bus, self.d);
                4
            }
            0x10 => {
                let offset = self.fetch8(bus) as i8;
                self.branch(offset, self.p & N == 0)
            }
            0x18 => {
                self.p &= !C;
                2
            }
            0x1a => {
                self.rmw_accumulator(5);
                2
            }
            0x1b => {
                self.sp = self.a;
                if self.emulation {
                    self.sp = 0x0100 | (self.sp & 0xff);
                }
                2
            }
            0x20 => {
                let target = self.fetch16(bus);
                self.push16(bus, self.pc.wrapping_sub(1));
                self.pc = target;
                6
            }
            0x22 => {
                let target = self.fetch24(bus);
                self.push8(bus, self.pbr);
                self.push16(bus, self.pc.wrapping_sub(1));
                self.pbr = (target >> 16) as u8;
                self.pc = target as u16;
                8
            }
            0x24 | 0x2c | 0x34 | 0x3c => {
                let (address, cycles) = match opcode {
                    0x24 => self.resolve_address(bus, AddressMode::Direct),
                    0x2c => self.resolve_address(bus, AddressMode::Absolute),
                    0x34 => self.resolve_address(bus, AddressMode::DirectX),
                    _ => self.resolve_address(bus, AddressMode::AbsoluteX),
                };
                let value = self.read_m(bus, address);
                self.bit_m(value, false);
                cycles + u32::from(!self.accumulator_is_8_bit())
            }
            0x26 | 0x2e | 0x36 | 0x3e => {
                let (address, mut cycles) = match opcode {
                    0x26 => self.resolve_address(bus, AddressMode::Direct),
                    0x2e => self.resolve_address(bus, AddressMode::Absolute),
                    0x36 => self.resolve_address(bus, AddressMode::DirectX),
                    _ => self.resolve_address(bus, AddressMode::AbsoluteX),
                };
                self.rmw_m(bus, address, 1);
                cycles += 2 + u32::from(!self.accumulator_is_8_bit());
                cycles
            }
            0x28 => {
                self.p = self.pull8(bus);
                self.normalize_mode();
                4
            }
            0x2a => {
                self.rmw_accumulator(1);
                2
            }
            0x2b => {
                self.d = self.pull16(bus);
                self.set_zn16(self.d);
                5
            }
            0x30 => {
                let offset = self.fetch8(bus) as i8;
                self.branch(offset, self.p & N != 0)
            }
            0x38 => {
                self.p |= C;
                2
            }
            0x3a => {
                self.rmw_accumulator(4);
                2
            }
            0x3b => {
                self.a = self.sp;
                self.set_zn16(self.a);
                2
            }
            0x40 => {
                self.p = self.pull8(bus);
                self.pc = self.pull16(bus);
                if !self.emulation {
                    self.pbr = self.pull8(bus);
                }
                self.normalize_mode();
                if self.emulation {
                    6
                } else {
                    7
                }
            }
            0x42 => {
                self.fetch8(bus);
                2
            }
            0x44 | 0x54 => {
                let destination_bank = self.fetch8(bus);
                let source_bank = self.fetch8(bus);
                let source = (u32::from(source_bank) << 16) | u32::from(self.x);
                let destination = (u32::from(destination_bank) << 16) | u32::from(self.y);
                let value = bus.read8(source);
                bus.write8(destination, value);
                self.dbr = destination_bank;
                if opcode == 0x54 {
                    self.x = self.x.wrapping_add(1);
                    self.y = self.y.wrapping_add(1);
                } else {
                    self.x = self.x.wrapping_sub(1);
                    self.y = self.y.wrapping_sub(1);
                }
                self.a = self.a.wrapping_sub(1);
                if self.a != 0xffff {
                    self.pc = self.pc.wrapping_sub(3);
                }
                7
            }
            0x46 | 0x4e | 0x56 | 0x5e => {
                let (address, mut cycles) = match opcode {
                    0x46 => self.resolve_address(bus, AddressMode::Direct),
                    0x4e => self.resolve_address(bus, AddressMode::Absolute),
                    0x56 => self.resolve_address(bus, AddressMode::DirectX),
                    _ => self.resolve_address(bus, AddressMode::AbsoluteX),
                };
                self.rmw_m(bus, address, 2);
                cycles += 2 + u32::from(!self.accumulator_is_8_bit());
                cycles
            }
            0x48 => {
                if self.accumulator_is_8_bit() {
                    self.push8(bus, self.a as u8);
                    3
                } else {
                    self.push16(bus, self.a);
                    4
                }
            }
            0x4a => {
                self.rmw_accumulator(2);
                2
            }
            0x4b => {
                self.push8(bus, self.pbr);
                3
            }
            0x4c => {
                self.pc = self.fetch16(bus);
                3
            }
            0x50 => {
                let offset = self.fetch8(bus) as i8;
                self.branch(offset, self.p & V == 0)
            }
            0x58 => {
                self.p &= !I;
                2
            }
            0x5a => {
                if self.index_is_8_bit() {
                    self.push8(bus, self.y as u8);
                    3
                } else {
                    self.push16(bus, self.y);
                    4
                }
            }
            0x5b => {
                self.d = self.a;
                self.set_zn16(self.d);
                2
            }
            0x5c => {
                let target = self.fetch24(bus);
                self.pbr = (target >> 16) as u8;
                self.pc = target as u16;
                4
            }
            0x60 => {
                self.pc = self.pull16(bus).wrapping_add(1);
                6
            }
            0x62 => {
                let displacement = self.fetch16(bus) as i16;
                let target = self.pc.wrapping_add_signed(displacement);
                self.push16(bus, target);
                6
            }
            0x64 | 0x74 | 0x9c | 0x9e => {
                let (address, mut cycles) = match opcode {
                    0x64 => self.resolve_address(bus, AddressMode::Direct),
                    0x74 => self.resolve_address(bus, AddressMode::DirectX),
                    0x9c => self.resolve_address(bus, AddressMode::Absolute),
                    _ => self.resolve_address(bus, AddressMode::AbsoluteX),
                };
                self.write_m(bus, address, 0);
                if !self.accumulator_is_8_bit() {
                    cycles += 1;
                }
                cycles
            }
            0x66 | 0x6e | 0x76 | 0x7e => {
                let (address, mut cycles) = match opcode {
                    0x66 => self.resolve_address(bus, AddressMode::Direct),
                    0x6e => self.resolve_address(bus, AddressMode::Absolute),
                    0x76 => self.resolve_address(bus, AddressMode::DirectX),
                    _ => self.resolve_address(bus, AddressMode::AbsoluteX),
                };
                self.rmw_m(bus, address, 3);
                cycles += 2 + u32::from(!self.accumulator_is_8_bit());
                cycles
            }
            0x68 => {
                if self.accumulator_is_8_bit() {
                    let value = u16::from(self.pull8(bus));
                    self.set_accumulator(value);
                    4
                } else {
                    let value = self.pull16(bus);
                    self.set_accumulator(value);
                    5
                }
            }
            0x6a => {
                self.rmw_accumulator(3);
                2
            }
            0x6b => {
                self.pc = self.pull16(bus).wrapping_add(1);
                self.pbr = self.pull8(bus);
                6
            }
            0x6c => {
                let pointer = self.fetch16(bus);
                self.pc = self.read16_bank0(bus, pointer);
                5
            }
            0x70 => {
                let offset = self.fetch8(bus) as i8;
                self.branch(offset, self.p & V != 0)
            }
            0x78 => {
                self.p |= I;
                2
            }
            0x7a => {
                let value = if self.index_is_8_bit() {
                    u16::from(self.pull8(bus))
                } else {
                    self.pull16(bus)
                };
                self.set_y(value);
                if self.index_is_8_bit() {
                    4
                } else {
                    5
                }
            }
            0x7b => {
                self.a = self.d;
                self.set_zn16(self.a);
                2
            }
            0x7c => {
                let base = self.fetch16(bus).wrapping_add(self.index_x());
                let pointer = (u32::from(self.pbr) << 16) | u32::from(base);
                self.pc = self.read16_same_bank(bus, pointer);
                6
            }
            0x80 => {
                let offset = self.fetch8(bus) as i8;
                self.branch(offset, true)
            }
            0x82 => {
                let offset = self.fetch16(bus) as i16;
                self.pc = self.pc.wrapping_add_signed(offset);
                4
            }
            0x84 | 0x8c | 0x94 => {
                let (address, mut cycles) = match opcode {
                    0x84 => self.resolve_address(bus, AddressMode::Direct),
                    0x8c => self.resolve_address(bus, AddressMode::Absolute),
                    _ => self.resolve_address(bus, AddressMode::DirectX),
                };
                self.write_index(bus, address, self.y);
                if !self.index_is_8_bit() {
                    cycles += 1;
                }
                cycles
            }
            0x86 | 0x8e | 0x96 => {
                let (address, mut cycles) = if opcode == 0x86 {
                    self.resolve_address(bus, AddressMode::Direct)
                } else if opcode == 0x8e {
                    self.resolve_address(bus, AddressMode::Absolute)
                } else {
                    let operand = self.fetch8(bus);
                    let address = self.direct_address(operand).wrapping_add(self.index_y());
                    (u32::from(address), 4)
                };
                self.write_index(bus, address, self.x);
                if !self.index_is_8_bit() {
                    cycles += 1;
                }
                cycles
            }
            0x88 => {
                self.set_y(self.y.wrapping_sub(1));
                2
            }
            0x89 => {
                let value = self.fetch_m(bus);
                self.bit_m(value, true);
                2 + u32::from(!self.accumulator_is_8_bit())
            }
            0x8a => {
                let value = self.x;
                self.set_accumulator(value);
                2
            }
            0x8b => {
                self.push8(bus, self.dbr);
                3
            }
            0x90 => {
                let offset = self.fetch8(bus) as i8;
                self.branch(offset, self.p & C == 0)
            }
            0x98 => {
                let value = self.y;
                self.set_accumulator(value);
                2
            }
            0x9a => {
                self.sp = self.x;
                if self.emulation {
                    self.sp = 0x0100 | (self.sp & 0x00ff);
                }
                2
            }
            0x9b => {
                self.set_y(self.x);
                2
            }
            0xa0 => {
                let value = self.fetch_index(bus);
                self.set_y(value);
                2 + u32::from(!self.index_is_8_bit())
            }
            0xa2 => {
                let value = self.fetch_index(bus);
                self.set_x(value);
                2 + u32::from(!self.index_is_8_bit())
            }
            0xa4 | 0xac | 0xb4 | 0xbc => {
                let (address, mut cycles) = match opcode {
                    0xa4 => self.resolve_address(bus, AddressMode::Direct),
                    0xac => self.resolve_address(bus, AddressMode::Absolute),
                    0xb4 => self.resolve_address(bus, AddressMode::DirectX),
                    _ => self.resolve_address(bus, AddressMode::AbsoluteX),
                };
                let value = self.read_index(bus, address);
                self.set_y(value);
                if !self.index_is_8_bit() {
                    cycles += 1;
                }
                cycles
            }
            0xa6 | 0xae | 0xb6 | 0xbe => {
                let (address, mut cycles) = match opcode {
                    0xa6 => self.resolve_address(bus, AddressMode::Direct),
                    0xae => self.resolve_address(bus, AddressMode::Absolute),
                    0xb6 => {
                        let operand = self.fetch8(bus);
                        let address = self.direct_address(operand).wrapping_add(self.index_y());
                        (u32::from(address), 4)
                    }
                    _ => self.resolve_address(bus, AddressMode::AbsoluteY),
                };
                let value = self.read_index(bus, address);
                self.set_x(value);
                if !self.index_is_8_bit() {
                    cycles += 1;
                }
                cycles
            }
            0xa8 => {
                self.set_y(self.accumulator());
                2
            }
            0xaa => {
                self.set_x(self.accumulator());
                2
            }
            0xab => {
                self.dbr = self.pull8(bus);
                self.set_zn8(self.dbr);
                4
            }
            0xb0 => {
                let offset = self.fetch8(bus) as i8;
                self.branch(offset, self.p & C != 0)
            }
            0xb8 => {
                self.p &= !V;
                2
            }
            0xba => {
                self.set_x(self.sp);
                2
            }
            0xbb => {
                self.set_x(self.y);
                2
            }
            0xc0 => {
                let value = self.fetch_index(bus);
                self.compare_index(self.y, value);
                2 + u32::from(!self.index_is_8_bit())
            }
            0xc2 => {
                let mask = self.fetch8(bus);
                self.p &= !mask;
                self.normalize_mode();
                3
            }
            0xc4 | 0xcc => {
                let (address, mut cycles) = if opcode == 0xc4 {
                    self.resolve_address(bus, AddressMode::Direct)
                } else {
                    self.resolve_address(bus, AddressMode::Absolute)
                };
                let value = self.read_index(bus, address);
                self.compare_index(self.y, value);
                if !self.index_is_8_bit() {
                    cycles += 1;
                }
                cycles
            }
            0xc6 | 0xce | 0xd6 | 0xde => {
                let (address, mut cycles) = match opcode {
                    0xc6 => self.resolve_address(bus, AddressMode::Direct),
                    0xce => self.resolve_address(bus, AddressMode::Absolute),
                    0xd6 => self.resolve_address(bus, AddressMode::DirectX),
                    _ => self.resolve_address(bus, AddressMode::AbsoluteX),
                };
                self.rmw_m(bus, address, 4);
                cycles += 2 + u32::from(!self.accumulator_is_8_bit());
                cycles
            }
            0xc8 => {
                self.set_y(self.y.wrapping_add(1));
                2
            }
            0xca => {
                self.set_x(self.x.wrapping_sub(1));
                2
            }
            0xcb => {
                self.waiting = true;
                3
            }
            0xd0 => {
                let offset = self.fetch8(bus) as i8;
                self.branch(offset, self.p & Z == 0)
            }
            0xd4 => {
                let operand = self.fetch8(bus);
                let pointer = self.read16_bank0(bus, self.direct_address(operand));
                self.push16(bus, pointer);
                6
            }
            0xd8 => {
                self.p &= !D;
                2
            }
            0xda => {
                if self.index_is_8_bit() {
                    self.push8(bus, self.x as u8);
                    3
                } else {
                    self.push16(bus, self.x);
                    4
                }
            }
            0xdb => {
                self.stopped = true;
                3
            }
            0xdc => {
                let pointer = self.fetch16(bus);
                let target = self.read24_bank0(bus, pointer);
                self.pbr = (target >> 16) as u8;
                self.pc = target as u16;
                6
            }
            0xe0 => {
                let value = self.fetch_index(bus);
                self.compare_index(self.x, value);
                2 + u32::from(!self.index_is_8_bit())
            }
            0xe2 => {
                let mask = self.fetch8(bus);
                self.p |= mask;
                self.normalize_mode();
                3
            }
            0xe4 | 0xec => {
                let (address, mut cycles) = if opcode == 0xe4 {
                    self.resolve_address(bus, AddressMode::Direct)
                } else {
                    self.resolve_address(bus, AddressMode::Absolute)
                };
                let value = self.read_index(bus, address);
                self.compare_index(self.x, value);
                if !self.index_is_8_bit() {
                    cycles += 1;
                }
                cycles
            }
            0xe6 | 0xee | 0xf6 | 0xfe => {
                let (address, mut cycles) = match opcode {
                    0xe6 => self.resolve_address(bus, AddressMode::Direct),
                    0xee => self.resolve_address(bus, AddressMode::Absolute),
                    0xf6 => self.resolve_address(bus, AddressMode::DirectX),
                    _ => self.resolve_address(bus, AddressMode::AbsoluteX),
                };
                self.rmw_m(bus, address, 5);
                cycles += 2 + u32::from(!self.accumulator_is_8_bit());
                cycles
            }
            0xe8 => {
                self.set_x(self.x.wrapping_add(1));
                2
            }
            0xea => 2,
            0xeb => {
                self.a = self.a.rotate_left(8);
                self.set_zn8(self.a as u8);
                3
            }
            0xf0 => {
                let offset = self.fetch8(bus) as i8;
                self.branch(offset, self.p & Z != 0)
            }
            0xf4 => {
                let value = self.fetch16(bus);
                self.push16(bus, value);
                5
            }
            0xf8 => {
                self.p |= D;
                2
            }
            0xfa => {
                let value = if self.index_is_8_bit() {
                    u16::from(self.pull8(bus))
                } else {
                    self.pull16(bus)
                };
                self.set_x(value);
                if self.index_is_8_bit() {
                    4
                } else {
                    5
                }
            }
            0xfb => {
                let old_carry = self.p & C != 0;
                self.set_flag(C, self.emulation);
                self.emulation = old_carry;
                self.normalize_mode();
                2
            }
            0xfc => {
                let base = self.fetch16(bus).wrapping_add(self.index_x());
                let pointer = (u32::from(self.pbr) << 16) | u32::from(base);
                let target = self.read16_same_bank(bus, pointer);
                self.push16(bus, self.pc.wrapping_sub(1));
                self.pc = target;
                8
            }
            _ => {
                self.stopped = true;
                0
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct TestBus {
        memory: Vec<u8>,
    }

    impl Default for TestBus {
        fn default() -> Self {
            Self {
                memory: vec![0; 1 << 24],
            }
        }
    }

    impl Bus65816 for TestBus {
        fn read8(&mut self, address: u32) -> u8 {
            self.memory[(address & 0x00ff_ffff) as usize]
        }

        fn write8(&mut self, address: u32, value: u8) {
            self.memory[(address & 0x00ff_ffff) as usize] = value;
        }
    }

    fn cpu_with_program(program: &[u8]) -> (Cpu65816, TestBus) {
        let mut bus = TestBus::default();
        bus.memory[0xfffc] = 0x00;
        bus.memory[0xfffd] = 0x80;
        bus.memory[0x8000..0x8000 + program.len()].copy_from_slice(program);
        let mut cpu = Cpu65816::default();
        cpu.reset(&mut bus);
        (cpu, bus)
    }

    #[test]
    fn every_opcode_has_an_execution_path() {
        let mut bus = TestBus::default();
        bus.memory[0xfffc] = 0x00;
        bus.memory[0xfffd] = 0x80;
        for opcode in 0u16..=255 {
            bus.memory[0x8000..0x8010].fill(0);
            bus.memory[0x8000] = opcode as u8;
            let mut cpu = Cpu65816::default();
            cpu.reset(&mut bus);
            let used = cpu.step(&mut bus);
            assert!(used > 0, "opcode {opcode:02X} has no execution path");
        }
    }

    #[test]
    fn native_mode_switches_accumulator_and_index_widths() {
        let program = [
            0xfb, 0xc2, 0x30, 0xa9, 0x34, 0x12, 0xa2, 0x78, 0x56, 0x18, 0x69, 0x01, 0x00, 0x8d,
            0x00, 0x20, 0xe2, 0x20, 0xa9, 0xaa, 0x8d, 0x02, 0x20, 0xdb,
        ];
        let (mut cpu, mut bus) = cpu_with_program(&program);
        while !cpu.stopped {
            cpu.step(&mut bus);
        }
        assert!(!cpu.emulation);
        assert_eq!(cpu.x, 0x5678);
        assert_eq!(bus.memory[0x2000], 0x35);
        assert_eq!(bus.memory[0x2001], 0x12);
        assert_eq!(bus.memory[0x2002], 0xaa);
        assert_eq!(cpu.a, 0x12aa);
    }

    #[test]
    fn long_addressing_reaches_full_24_bit_bus() {
        let program = [
            0xfb, 0xc2, 0x20, 0xa9, 0xef, 0xbe, 0x8f, 0x34, 0x12, 0x7e, 0xa9, 0x00, 0x00, 0xaf,
            0x34, 0x12, 0x7e, 0xdb,
        ];
        let (mut cpu, mut bus) = cpu_with_program(&program);
        while !cpu.stopped {
            cpu.step(&mut bus);
        }
        assert_eq!(bus.memory[0x7e1234], 0xef);
        assert_eq!(bus.memory[0x7e1235], 0xbe);
        assert_eq!(cpu.a, 0xbeef);
    }

    #[test]
    fn decimal_mode_handles_sixteen_bit_bcd() {
        let program = [
            0xfb, 0xc2, 0x20, 0xf8, 0x18, 0xa9, 0x99, 0x09, 0x69, 0x01, 0x00, 0xdb,
        ];
        let (mut cpu, mut bus) = cpu_with_program(&program);
        while !cpu.stopped {
            cpu.step(&mut bus);
        }
        assert_eq!(cpu.a, 0x1000);
        assert_eq!(cpu.p & C, 0);
    }

    #[test]
    fn block_move_copies_until_accumulator_underflows() {
        let program = [
            0xfb, 0xc2, 0x30, 0xa2, 0x00, 0x10, 0xa0, 0x00, 0x20, 0xa9, 0x02, 0x00, 0x54, 0x7f,
            0x7e, 0xdb,
        ];
        let (mut cpu, mut bus) = cpu_with_program(&program);
        bus.memory[0x7e1000..0x7e1003].copy_from_slice(&[0x11, 0x22, 0x33]);
        while !cpu.stopped {
            cpu.step(&mut bus);
        }
        assert_eq!(&bus.memory[0x7f2000..0x7f2003], &[0x11, 0x22, 0x33]);
        assert_eq!(cpu.a, 0xffff);
    }

    #[test]
    fn native_nmi_pushes_program_bank_and_returns_with_rti() {
        let program = [0xfb, 0xea, 0xea, 0xdb];
        let (mut cpu, mut bus) = cpu_with_program(&program);
        cpu.step(&mut bus);
        bus.memory[0xffea] = 0x00;
        bus.memory[0xffeb] = 0x90;
        bus.memory[0x9000] = 0x40;
        let interrupted_pc = cpu.pc;
        assert_eq!(cpu.nmi(&mut bus), 8);
        assert_eq!(cpu.pc, 0x9000);
        cpu.step(&mut bus);
        assert_eq!(cpu.pc, interrupted_pc);
        assert_eq!(cpu.pbr, 0);
    }
}
