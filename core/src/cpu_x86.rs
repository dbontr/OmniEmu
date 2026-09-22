use crate::state::{StateReader, StateWriter};

const CF: u32 = 1 << 0;
const PF: u32 = 1 << 2;
const AF: u32 = 1 << 4;
const ZF: u32 = 1 << 6;
const SF: u32 = 1 << 7;
const IF: u32 = 1 << 9;
const DF: u32 = 1 << 10;
const OF: u32 = 1 << 11;

pub const EAX: usize = 0;
pub const ECX: usize = 1;
pub const EDX: usize = 2;
pub const EBX: usize = 3;
pub const ESP: usize = 4;
pub const EBP: usize = 5;
pub const ESI: usize = 6;
pub const EDI: usize = 7;

pub trait X86Bus {
    fn read8(&mut self, address: u32) -> u8;
    fn write8(&mut self, address: u32, value: u8);

    fn read16(&mut self, address: u32) -> u16 {
        u16::from_le_bytes([self.read8(address), self.read8(address.wrapping_add(1))])
    }

    fn write16(&mut self, address: u32, value: u16) {
        for (offset, byte) in value.to_le_bytes().into_iter().enumerate() {
            self.write8(address.wrapping_add(offset as u32), byte);
        }
    }
    fn read32(&mut self, address: u32) -> u32 {
        let mut bytes = [0; 4];
        for (offset, byte) in bytes.iter_mut().enumerate() {
            *byte = self.read8(address.wrapping_add(offset as u32));
        }
        u32::from_le_bytes(bytes)
    }

    fn write32(&mut self, address: u32, value: u32) {
        for (offset, byte) in value.to_le_bytes().into_iter().enumerate() {
            self.write8(address.wrapping_add(offset as u32), byte);
        }
    }

    fn io_read8(&mut self, _port: u16) -> u8 {
        0xff
    }

    fn io_write8(&mut self, _port: u16, _value: u8) {}

    fn io_read16(&mut self, port: u16) -> u16 {
        u16::from_le_bytes([self.io_read8(port), self.io_read8(port.wrapping_add(1))])
    }

    fn io_write16(&mut self, port: u16, value: u16) {
        for (offset, byte) in value.to_le_bytes().into_iter().enumerate() {
            self.io_write8(port.wrapping_add(offset as u16), byte);
        }
    }

    fn io_read32(&mut self, port: u16) -> u32 {
        let mut bytes = [0; 4];
        for (offset, byte) in bytes.iter_mut().enumerate() {
            *byte = self.io_read8(port.wrapping_add(offset as u16));
        }
        u32::from_le_bytes(bytes)
    }

    fn io_write32(&mut self, port: u16, value: u32) {
        for (offset, byte) in value.to_le_bytes().into_iter().enumerate() {
            self.io_write8(port.wrapping_add(offset as u16), byte);
        }
    }

    fn read64(&mut self, address: u32) -> u64 {
        let low = u64::from(self.read32(address));
        let high = u64::from(self.read32(address.wrapping_add(4)));
        low | (high << 32)
    }

    fn write64(&mut self, address: u32, value: u64) {
        self.write32(address, value as u32);
        self.write32(address.wrapping_add(4), (value >> 32) as u32);
    }

    fn read128(&mut self, address: u32) -> u128 {
        let low = u128::from(self.read64(address));
        let high = u128::from(self.read64(address.wrapping_add(8)));
        low | (high << 64)
    }

    fn write128(&mut self, address: u32, value: u128) {
        self.write64(address, value as u64);
        self.write64(address.wrapping_add(8), (value >> 64) as u64);
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Operand {
    Register(usize),
    Memory(u32),
}

#[derive(Clone, Copy, Default)]
struct Prefixes {
    operand16: bool,
    rep: bool,
    repne: bool,
    segment_base: u32,
}
#[derive(Clone)]
pub struct X86Cpu {
    pub regs: [u32; 8],
    pub eip: u32,
    pub eflags: u32,
    pub cycles: u64,
    pub fs_base: u32,
    pub gs_base: u32,
    pub cr0: u32,
    pub cr2: u32,
    pub cr3: u32,
    pub cr4: u32,
    pub tsc: u64,
    pub xmm: [u128; 8],
    pub mxcsr: u32,
    halted: bool,
    interrupt_pending: bool,
    invalid_opcode: bool,
}

impl Default for X86Cpu {
    fn default() -> Self {
        Self::new()
    }
}

impl X86Cpu {
    pub fn new() -> Self {
        Self {
            regs: [0; 8],
            eip: 0,
            eflags: 0x2,
            cycles: 0,
            fs_base: 0,
            gs_base: 0,
            cr0: 1,
            cr2: 0,
            cr3: 0,
            cr4: 0,
            tsc: 0,
            xmm: [0; 8],
            mxcsr: 0x1f80,
            halted: false,
            interrupt_pending: false,
            invalid_opcode: false,
        }
    }
    pub fn reset_to(&mut self, eip: u32) {
        *self = Self::new();
        self.eip = eip;
    }

    pub fn set_interrupt_pending(&mut self, pending: bool) {
        self.interrupt_pending = pending;
        if pending {
            self.halted = false;
        }
    }

    pub fn halted(&self) -> bool {
        self.halted
    }

    pub fn invalid_opcode(&self) -> bool {
        self.invalid_opcode
    }

    pub fn step<B: X86Bus>(&mut self, bus: &mut B) {
        if self.halted {
            self.cycles = self.cycles.wrapping_add(1);
            self.tsc = self.tsc.wrapping_add(1);
            return;
        }
        self.invalid_opcode = false;
        let prefixes = self.fetch_prefixes(bus);
        let opcode = self.fetch8(bus);
        self.execute_opcode(bus, opcode, prefixes);
        self.eflags |= 0x2;
        self.cycles = self.cycles.wrapping_add(1);
        self.tsc = self.tsc.wrapping_add(1);
    }

    fn fetch_prefixes<B: X86Bus>(&mut self, bus: &mut B) -> Prefixes {
        let mut prefixes = Prefixes::default();
        loop {
            match bus.read8(self.eip) {
                0x66 => prefixes.operand16 = true,
                0xf3 => prefixes.rep = true,
                0xf2 => prefixes.repne = true,
                0x64 => prefixes.segment_base = self.fs_base,
                0x65 => prefixes.segment_base = self.gs_base,
                0x2e | 0x36 | 0x3e | 0x26 => prefixes.segment_base = 0,
                0xf0 => {}
                _ => break,
            }
            self.eip = self.eip.wrapping_add(1);
        }
        prefixes
    }
    fn fetch8<B: X86Bus>(&mut self, bus: &mut B) -> u8 {
        let value = bus.read8(self.eip);
        self.eip = self.eip.wrapping_add(1);
        value
    }

    fn fetch16<B: X86Bus>(&mut self, bus: &mut B) -> u16 {
        let value = bus.read16(self.eip);
        self.eip = self.eip.wrapping_add(2);
        value
    }

    fn fetch32<B: X86Bus>(&mut self, bus: &mut B) -> u32 {
        let value = bus.read32(self.eip);
        self.eip = self.eip.wrapping_add(4);
        value
    }

    fn reg8(&self, index: usize) -> u8 {
        if index < 4 {
            self.regs[index] as u8
        } else {
            (self.regs[index - 4] >> 8) as u8
        }
    }

    fn set_reg8(&mut self, index: usize, value: u8) {
        if index < 4 {
            self.regs[index] = (self.regs[index] & !0xff) | u32::from(value);
        } else {
            let register = index - 4;
            self.regs[register] = (self.regs[register] & !0xff00) | (u32::from(value) << 8);
        }
    }
    fn decode_modrm<B: X86Bus>(&mut self, bus: &mut B, prefixes: Prefixes) -> (usize, Operand) {
        let modrm = self.fetch8(bus);
        let mode = modrm >> 6;
        let reg = ((modrm >> 3) & 7) as usize;
        let rm = (modrm & 7) as usize;
        if mode == 3 {
            return (reg, Operand::Register(rm));
        }

        let mut base = 0u32;
        let mut index = 0u32;
        let mut needs_disp32 = false;
        if rm == 4 {
            let sib = self.fetch8(bus);
            let scale = 1u32 << (sib >> 6);
            let index_reg = ((sib >> 3) & 7) as usize;
            let base_reg = (sib & 7) as usize;
            if index_reg != ESP {
                index = self.regs[index_reg].wrapping_mul(scale);
            }
            if mode == 0 && base_reg == EBP {
                needs_disp32 = true;
            } else {
                base = self.regs[base_reg];
            }
        } else if mode == 0 && rm == EBP {
            needs_disp32 = true;
        } else {
            base = self.regs[rm];
        }
        let displacement = match mode {
            0 if needs_disp32 => self.fetch32(bus),
            0 => 0,
            1 => i32::from(self.fetch8(bus) as i8) as u32,
            2 => self.fetch32(bus),
            _ => 0,
        };
        let address = prefixes
            .segment_base
            .wrapping_add(base)
            .wrapping_add(index)
            .wrapping_add(displacement);
        (reg, Operand::Memory(address))
    }

    fn read_operand8<B: X86Bus>(&self, bus: &mut B, operand: Operand) -> u8 {
        match operand {
            Operand::Register(index) => self.reg8(index),
            Operand::Memory(address) => bus.read8(address),
        }
    }

    fn write_operand8<B: X86Bus>(&mut self, bus: &mut B, operand: Operand, value: u8) {
        match operand {
            Operand::Register(index) => self.set_reg8(index, value),
            Operand::Memory(address) => bus.write8(address, value),
        }
    }

    fn read_operand16<B: X86Bus>(&self, bus: &mut B, operand: Operand) -> u16 {
        match operand {
            Operand::Register(index) => self.regs[index] as u16,
            Operand::Memory(address) => bus.read16(address),
        }
    }
    fn write_operand16<B: X86Bus>(&mut self, bus: &mut B, operand: Operand, value: u16) {
        match operand {
            Operand::Register(index) => {
                self.regs[index] = (self.regs[index] & 0xffff_0000) | u32::from(value);
            }
            Operand::Memory(address) => bus.write16(address, value),
        }
    }

    fn read_operand32<B: X86Bus>(&self, bus: &mut B, operand: Operand) -> u32 {
        match operand {
            Operand::Register(index) => self.regs[index],
            Operand::Memory(address) => bus.read32(address),
        }
    }

    fn write_operand32<B: X86Bus>(&mut self, bus: &mut B, operand: Operand, value: u32) {
        match operand {
            Operand::Register(index) => self.regs[index] = value,
            Operand::Memory(address) => bus.write32(address, value),
        }
    }

    fn read_xmm128<B: X86Bus>(&self, bus: &mut B, operand: Operand) -> u128 {
        match operand {
            Operand::Register(index) => self.xmm[index],
            Operand::Memory(address) => bus.read128(address),
        }
    }

    fn write_xmm128<B: X86Bus>(&mut self, bus: &mut B, operand: Operand, value: u128) {
        match operand {
            Operand::Register(index) => self.xmm[index] = value,
            Operand::Memory(address) => bus.write128(address, value),
        }
    }

    fn read_xmm32<B: X86Bus>(&self, bus: &mut B, operand: Operand) -> u32 {
        match operand {
            Operand::Register(index) => self.xmm[index] as u32,
            Operand::Memory(address) => bus.read32(address),
        }
    }

    fn xmm_u32(value: u128, lane: usize) -> u32 {
        (value >> (lane * 32)) as u32
    }

    fn xmm_set_u32(value: &mut u128, lane: usize, lane_value: u32) {
        let shift = lane * 32;
        let mask = u128::from(u32::MAX) << shift;
        *value = (*value & !mask) | (u128::from(lane_value) << shift);
    }

    fn sse_binary_f32(lhs: f32, rhs: f32, opcode: u8) -> f32 {
        match opcode {
            0x58 => lhs + rhs,
            0x59 => lhs * rhs,
            0x5c => lhs - rhs,
            0x5e => lhs / rhs,
            _ => unreachable!(),
        }
    }

    fn sse_min_f32(lhs: f32, rhs: f32) -> f32 {
        if lhs.is_nan() || rhs.is_nan() || lhs == rhs {
            rhs
        } else if lhs < rhs {
            lhs
        } else {
            rhs
        }
    }

    fn sse_max_f32(lhs: f32, rhs: f32) -> f32 {
        if lhs.is_nan() || rhs.is_nan() || lhs == rhs {
            rhs
        } else if lhs > rhs {
            lhs
        } else {
            rhs
        }
    }

    fn sse_compare_f32(lhs: f32, rhs: f32, predicate: u8) -> bool {
        let unordered = lhs.is_nan() || rhs.is_nan();
        match predicate & 7 {
            0 => !unordered && lhs == rhs,
            1 => !unordered && lhs < rhs,
            2 => !unordered && lhs <= rhs,
            3 => unordered,
            4 => unordered || lhs != rhs,
            5 => unordered || lhs >= rhs,
            6 => unordered || lhs > rhs,
            _ => !unordered,
        }
    }

    fn sse_f32_to_i32(&mut self, value: f32, truncate: bool) -> u32 {
        if !value.is_finite() {
            self.mxcsr |= 1;
            return 0x8000_0000;
        }
        let rounded = if truncate {
            value.trunc()
        } else {
            match (self.mxcsr >> 13) & 3 {
                0 => value.round_ties_even(),
                1 => value.floor(),
                2 => value.ceil(),
                _ => value.trunc(),
            }
        };
        if rounded < i32::MIN as f32 || rounded >= 2_147_483_648.0 {
            self.mxcsr |= 1;
            0x8000_0000
        } else {
            rounded as i32 as u32
        }
    }

    fn push32<B: X86Bus>(&mut self, bus: &mut B, value: u32) {
        self.regs[ESP] = self.regs[ESP].wrapping_sub(4);
        bus.write32(self.regs[ESP], value);
    }

    fn pop32<B: X86Bus>(&mut self, bus: &mut B) -> u32 {
        let value = bus.read32(self.regs[ESP]);
        self.regs[ESP] = self.regs[ESP].wrapping_add(4);
        value
    }

    fn parity(value: u8) -> bool {
        value.count_ones().is_multiple_of(2)
    }
    fn set_szp(&mut self, value: u32, bits: u32) {
        let mask = if bits == 32 {
            u32::MAX
        } else {
            (1u32 << bits) - 1
        };
        let value = value & mask;
        let sign = 1u32 << (bits - 1);
        self.eflags &= !(SF | ZF | PF);
        if value == 0 {
            self.eflags |= ZF;
        }
        if value & sign != 0 {
            self.eflags |= SF;
        }
        if Self::parity(value as u8) {
            self.eflags |= PF;
        }
    }

    fn logic_flags(&mut self, value: u32, bits: u32) {
        self.eflags &= !(CF | OF);
        self.set_szp(value, bits);
    }

    fn add_flags(&mut self, lhs: u32, rhs: u32, result: u32, bits: u32) {
        let mask = if bits == 32 {
            u32::MAX
        } else {
            (1u32 << bits) - 1
        };
        let sign = 1u32 << (bits - 1);
        let wide = u64::from(lhs & mask) + u64::from(rhs & mask);
        self.eflags &= !(CF | AF | OF);
        if wide > u64::from(mask) {
            self.eflags |= CF;
        }
        if (lhs ^ rhs ^ result) & 0x10 != 0 {
            self.eflags |= AF;
        }
        if (!(lhs ^ rhs) & (lhs ^ result) & sign) != 0 {
            self.eflags |= OF;
        }
        self.set_szp(result, bits);
    }

    fn adc_flags(&mut self, lhs: u32, rhs: u32, carry: u32, result: u32, bits: u32) {
        let mask = if bits == 32 {
            u32::MAX
        } else {
            (1u32 << bits) - 1
        };
        let wide = u64::from(lhs & mask) + u64::from(rhs & mask) + u64::from(carry);
        self.eflags &= !(CF | AF | OF);
        if wide > u64::from(mask) {
            self.eflags |= CF;
        }
        if (lhs & 0x0f) + (rhs & 0x0f) + carry > 0x0f {
            self.eflags |= AF;
        }
        let lhs_signed = match bits {
            8 => i64::from(lhs as u8 as i8),
            16 => i64::from(lhs as u16 as i16),
            _ => i64::from(lhs as i32),
        };
        let rhs_signed = match bits {
            8 => i64::from(rhs as u8 as i8),
            16 => i64::from(rhs as u16 as i16),
            _ => i64::from(rhs as i32),
        };
        let signed_result = lhs_signed + rhs_signed + i64::from(carry);
        let min = -(1i64 << (bits - 1));
        let max = (1i64 << (bits - 1)) - 1;
        if signed_result < min || signed_result > max {
            self.eflags |= OF;
        }
        self.set_szp(result, bits);
    }

    fn sub_flags(&mut self, lhs: u32, rhs: u32, result: u32, bits: u32) {
        let mask = if bits == 32 {
            u32::MAX
        } else {
            (1u32 << bits) - 1
        };
        let sign = 1u32 << (bits - 1);
        self.eflags &= !(CF | AF | OF);
        if lhs & mask < rhs & mask {
            self.eflags |= CF;
        }
        if (lhs ^ rhs ^ result) & 0x10 != 0 {
            self.eflags |= AF;
        }
        if ((lhs ^ rhs) & (lhs ^ result) & sign) != 0 {
            self.eflags |= OF;
        }
        self.set_szp(result, bits);
    }

    fn sbb_flags(&mut self, lhs: u32, rhs: u32, carry: u32, result: u32, bits: u32) {
        let mask = if bits == 32 {
            u32::MAX
        } else {
            (1u32 << bits) - 1
        };
        let subtrahend = u64::from(rhs & mask) + u64::from(carry);
        self.eflags &= !(CF | AF | OF);
        if u64::from(lhs & mask) < subtrahend {
            self.eflags |= CF;
        }
        if (lhs & 0x0f) < (rhs & 0x0f).wrapping_add(carry) {
            self.eflags |= AF;
        }
        let lhs_signed = match bits {
            8 => i64::from(lhs as u8 as i8),
            16 => i64::from(lhs as u16 as i16),
            _ => i64::from(lhs as i32),
        };
        let rhs_signed = match bits {
            8 => i64::from(rhs as u8 as i8),
            16 => i64::from(rhs as u16 as i16),
            _ => i64::from(rhs as i32),
        };
        let signed_result = lhs_signed - rhs_signed - i64::from(carry);
        let min = -(1i64 << (bits - 1));
        let max = (1i64 << (bits - 1)) - 1;
        if signed_result < min || signed_result > max {
            self.eflags |= OF;
        }
        self.set_szp(result, bits);
    }
    fn condition(&self, code: u8) -> bool {
        let cf = self.eflags & CF != 0;
        let pf = self.eflags & PF != 0;
        let zf = self.eflags & ZF != 0;
        let sf = self.eflags & SF != 0;
        let of = self.eflags & OF != 0;
        match code & 0x0f {
            0x0 => of,
            0x1 => !of,
            0x2 => cf,
            0x3 => !cf,
            0x4 => zf,
            0x5 => !zf,
            0x6 => cf || zf,
            0x7 => !cf && !zf,
            0x8 => sf,
            0x9 => !sf,
            0xa => pf,
            0xb => !pf,
            0xc => sf != of,
            0xd => sf == of,
            0xe => zf || sf != of,
            _ => !zf && sf == of,
        }
    }

    fn execute_group1_32<B: X86Bus>(&mut self, bus: &mut B, op: usize, dst: Operand, rhs: u32) {
        let lhs = self.read_operand32(bus, dst);
        let carry = u32::from(self.eflags & CF != 0);
        let result = match op {
            0 => lhs.wrapping_add(rhs),
            1 => lhs | rhs,
            2 => lhs.wrapping_add(rhs).wrapping_add(carry),
            3 => lhs.wrapping_sub(rhs).wrapping_sub(carry),
            4 => lhs & rhs,
            5 | 7 => lhs.wrapping_sub(rhs),
            6 => lhs ^ rhs,
            _ => unreachable!(),
        };
        match op {
            0 => self.add_flags(lhs, rhs, result, 32),
            1 | 4 | 6 => self.logic_flags(result, 32),
            2 => self.adc_flags(lhs, rhs, carry, result, 32),
            3 => self.sbb_flags(lhs, rhs, carry, result, 32),
            5 | 7 => self.sub_flags(lhs, rhs, result, 32),
            _ => {}
        }
        if op != 7 {
            self.write_operand32(bus, dst, result);
        }
    }

    fn execute_group1_16<B: X86Bus>(&mut self, bus: &mut B, op: usize, dst: Operand, rhs: u16) {
        let lhs = self.read_operand16(bus, dst);
        let carry = u16::from(self.eflags & CF != 0);
        let result = match op {
            0 => lhs.wrapping_add(rhs),
            1 => lhs | rhs,
            2 => lhs.wrapping_add(rhs).wrapping_add(carry),
            3 => lhs.wrapping_sub(rhs).wrapping_sub(carry),
            4 => lhs & rhs,
            5 | 7 => lhs.wrapping_sub(rhs),
            6 => lhs ^ rhs,
            _ => unreachable!(),
        };
        match op {
            0 => self.add_flags(u32::from(lhs), u32::from(rhs), u32::from(result), 16),
            1 | 4 | 6 => self.logic_flags(u32::from(result), 16),
            2 => self.adc_flags(
                u32::from(lhs),
                u32::from(rhs),
                u32::from(carry),
                u32::from(result),
                16,
            ),
            3 => self.sbb_flags(
                u32::from(lhs),
                u32::from(rhs),
                u32::from(carry),
                u32::from(result),
                16,
            ),
            5 | 7 => self.sub_flags(u32::from(lhs), u32::from(rhs), u32::from(result), 16),
            _ => {}
        }
        if op != 7 {
            self.write_operand16(bus, dst, result);
        }
    }

    fn execute_group1_8<B: X86Bus>(&mut self, bus: &mut B, op: usize, dst: Operand, rhs: u8) {
        let lhs = self.read_operand8(bus, dst);
        let carry = u8::from(self.eflags & CF != 0);
        let result = match op {
            0 => lhs.wrapping_add(rhs),
            1 => lhs | rhs,
            2 => lhs.wrapping_add(rhs).wrapping_add(carry),
            3 => lhs.wrapping_sub(rhs).wrapping_sub(carry),
            4 => lhs & rhs,
            5 | 7 => lhs.wrapping_sub(rhs),
            6 => lhs ^ rhs,
            _ => unreachable!(),
        };
        match op {
            0 => self.add_flags(u32::from(lhs), u32::from(rhs), u32::from(result), 8),
            1 | 4 | 6 => self.logic_flags(u32::from(result), 8),
            2 => self.adc_flags(
                u32::from(lhs),
                u32::from(rhs),
                u32::from(carry),
                u32::from(result),
                8,
            ),
            3 => self.sbb_flags(
                u32::from(lhs),
                u32::from(rhs),
                u32::from(carry),
                u32::from(result),
                8,
            ),
            5 | 7 => self.sub_flags(u32::from(lhs), u32::from(rhs), u32::from(result), 8),
            _ => {}
        }
        if op != 7 {
            self.write_operand8(bus, dst, result);
        }
    }

    fn execute_shift32<B: X86Bus>(&mut self, bus: &mut B, op: usize, dst: Operand, count: u32) {
        let count = count & 0x1f;
        if count == 0 {
            return;
        }
        let lhs = self.read_operand32(bus, dst);
        let result = match op {
            0 => lhs.rotate_left(count),
            1 => lhs.rotate_right(count),
            4 | 6 => lhs.wrapping_shl(count),
            5 => lhs.wrapping_shr(count),
            7 => ((lhs as i32) >> count) as u32,
            _ => {
                self.invalid_opcode = true;
                return;
            }
        };
        self.eflags &= !(CF | OF);
        let carry = match op {
            0 => result & 1 != 0,
            1 => result & 0x8000_0000 != 0,
            4 | 6 => lhs & (1u32 << (32 - count)) != 0,
            5 | 7 => lhs & (1u32 << (count - 1)) != 0,
            _ => false,
        };
        if carry {
            self.eflags |= CF;
        }
        self.set_szp(result, 32);
        self.write_operand32(bus, dst, result);
    }

    fn string_step(&self, width: u32) -> u32 {
        if self.eflags & DF != 0 {
            0u32.wrapping_sub(width)
        } else {
            width
        }
    }

    fn execute_string<B: X86Bus>(&mut self, bus: &mut B, opcode: u8, prefixes: Prefixes) {
        let iterations = if prefixes.rep || prefixes.repne {
            self.regs[ECX]
        } else {
            1
        };
        let mut remaining = iterations;
        while remaining != 0 {
            match opcode {
                0x6c => {
                    let value = bus.io_read8(self.regs[EDX] as u16);
                    bus.write8(self.regs[EDI], value);
                    self.regs[EDI] = self.regs[EDI].wrapping_add(self.string_step(1));
                }
                0x6d => {
                    let width = if prefixes.operand16 { 2 } else { 4 };
                    let port = self.regs[EDX] as u16;
                    if width == 2 {
                        let value = bus.io_read16(port);
                        bus.write16(self.regs[EDI], value);
                    } else {
                        let value = bus.io_read32(port);
                        bus.write32(self.regs[EDI], value);
                    }
                    self.regs[EDI] = self.regs[EDI].wrapping_add(self.string_step(width));
                }
                0x6e => {
                    let value = bus.read8(prefixes.segment_base.wrapping_add(self.regs[ESI]));
                    bus.io_write8(self.regs[EDX] as u16, value);
                    self.regs[ESI] = self.regs[ESI].wrapping_add(self.string_step(1));
                }
                0x6f => {
                    let width = if prefixes.operand16 { 2 } else { 4 };
                    let address = prefixes.segment_base.wrapping_add(self.regs[ESI]);
                    let port = self.regs[EDX] as u16;
                    if width == 2 {
                        let value = bus.read16(address);
                        bus.io_write16(port, value);
                    } else {
                        let value = bus.read32(address);
                        bus.io_write32(port, value);
                    }
                    self.regs[ESI] = self.regs[ESI].wrapping_add(self.string_step(width));
                }
                0xa4 => {
                    let value = bus.read8(prefixes.segment_base.wrapping_add(self.regs[ESI]));
                    bus.write8(self.regs[EDI], value);
                    let step = self.string_step(1);
                    self.regs[ESI] = self.regs[ESI].wrapping_add(step);
                    self.regs[EDI] = self.regs[EDI].wrapping_add(step);
                }
                0xa5 => {
                    let width = if prefixes.operand16 { 2 } else { 4 };
                    if width == 2 {
                        let value = bus.read16(prefixes.segment_base.wrapping_add(self.regs[ESI]));
                        bus.write16(self.regs[EDI], value);
                    } else {
                        let value = bus.read32(prefixes.segment_base.wrapping_add(self.regs[ESI]));
                        bus.write32(self.regs[EDI], value);
                    }
                    let step = self.string_step(width);
                    self.regs[ESI] = self.regs[ESI].wrapping_add(step);
                    self.regs[EDI] = self.regs[EDI].wrapping_add(step);
                }
                0xaa => {
                    bus.write8(self.regs[EDI], self.reg8(EAX));
                    self.regs[EDI] = self.regs[EDI].wrapping_add(self.string_step(1));
                }
                0xab => {
                    let width = if prefixes.operand16 { 2 } else { 4 };
                    if width == 2 {
                        bus.write16(self.regs[EDI], self.regs[EAX] as u16);
                    } else {
                        bus.write32(self.regs[EDI], self.regs[EAX]);
                    }
                    self.regs[EDI] = self.regs[EDI].wrapping_add(self.string_step(width));
                }
                0xac => {
                    let value = bus.read8(prefixes.segment_base.wrapping_add(self.regs[ESI]));
                    self.set_reg8(EAX, value);
                    self.regs[ESI] = self.regs[ESI].wrapping_add(self.string_step(1));
                }
                0xad => {
                    let width = if prefixes.operand16 { 2 } else { 4 };
                    if width == 2 {
                        let value = bus.read16(prefixes.segment_base.wrapping_add(self.regs[ESI]));
                        self.regs[EAX] = (self.regs[EAX] & 0xffff_0000) | u32::from(value);
                    } else {
                        self.regs[EAX] =
                            bus.read32(prefixes.segment_base.wrapping_add(self.regs[ESI]));
                    }
                    self.regs[ESI] = self.regs[ESI].wrapping_add(self.string_step(width));
                }
                _ => {
                    self.invalid_opcode = true;
                    break;
                }
            }
            remaining -= 1;
            if prefixes.rep || prefixes.repne {
                self.regs[ECX] = remaining;
            }
        }
    }

    fn execute_sse<B: X86Bus>(&mut self, bus: &mut B, opcode: u8, prefixes: Prefixes) -> bool {
        let scalar = prefixes.rep && !prefixes.repne && !prefixes.operand16;
        let packed = !prefixes.rep && !prefixes.repne && !prefixes.operand16;
        match opcode {
            0x10 | 0x28 => {
                let (reg, src) = self.decode_modrm(bus, prefixes);
                if scalar && opcode == 0x10 {
                    let value = self.read_xmm32(bus, src);
                    if matches!(src, Operand::Memory(_)) {
                        self.xmm[reg] = u128::from(value);
                    } else {
                        Self::xmm_set_u32(&mut self.xmm[reg], 0, value);
                    }
                } else if packed {
                    self.xmm[reg] = self.read_xmm128(bus, src);
                } else {
                    self.invalid_opcode = true;
                }
                true
            }
            0x11 | 0x29 => {
                let (reg, dst) = self.decode_modrm(bus, prefixes);
                if scalar && opcode == 0x11 {
                    let value = self.xmm[reg] as u32;
                    match dst {
                        Operand::Register(index) => {
                            Self::xmm_set_u32(&mut self.xmm[index], 0, value)
                        }
                        Operand::Memory(address) => bus.write32(address, value),
                    }
                } else if packed {
                    self.write_xmm128(bus, dst, self.xmm[reg]);
                } else {
                    self.invalid_opcode = true;
                }
                true
            }
            0x14 | 0x15 if packed => {
                let (reg, src) = self.decode_modrm(bus, prefixes);
                let lhs = self.xmm[reg];
                let rhs = self.read_xmm128(bus, src);
                let mut result = 0u128;
                let lanes = if opcode == 0x14 {
                    [
                        Self::xmm_u32(lhs, 0),
                        Self::xmm_u32(rhs, 0),
                        Self::xmm_u32(lhs, 1),
                        Self::xmm_u32(rhs, 1),
                    ]
                } else {
                    [
                        Self::xmm_u32(lhs, 2),
                        Self::xmm_u32(rhs, 2),
                        Self::xmm_u32(lhs, 3),
                        Self::xmm_u32(rhs, 3),
                    ]
                };
                for (lane, value) in lanes.into_iter().enumerate() {
                    Self::xmm_set_u32(&mut result, lane, value);
                }
                self.xmm[reg] = result;
                true
            }
            0x18 => {
                let _ = self.decode_modrm(bus, prefixes);
                true
            }
            0x2a => {
                let (reg, src) = self.decode_modrm(bus, prefixes);
                if scalar {
                    let value = self.read_operand32(bus, src) as i32 as f32;
                    Self::xmm_set_u32(&mut self.xmm[reg], 0, value.to_bits());
                } else {
                    self.invalid_opcode = true;
                }
                true
            }
            0x2c | 0x2d => {
                let (reg, src) = self.decode_modrm(bus, prefixes);
                if scalar {
                    let value = f32::from_bits(self.read_xmm32(bus, src));
                    self.regs[reg] = self.sse_f32_to_i32(value, opcode == 0x2c);
                } else {
                    self.invalid_opcode = true;
                }
                true
            }
            0x2e | 0x2f => {
                let (reg, src) = self.decode_modrm(bus, prefixes);
                if packed {
                    let lhs = f32::from_bits(self.xmm[reg] as u32);
                    let rhs = f32::from_bits(self.read_xmm32(bus, src));
                    self.eflags &= !(CF | PF | AF | ZF | SF | OF);
                    if lhs.is_nan() || rhs.is_nan() {
                        self.eflags |= CF | PF | ZF;
                    } else if lhs < rhs {
                        self.eflags |= CF;
                    } else if lhs == rhs {
                        self.eflags |= ZF;
                    }
                } else {
                    self.invalid_opcode = true;
                }
                true
            }
            0x50 => {
                let (reg, src) = self.decode_modrm(bus, prefixes);
                if packed {
                    let value = self.read_xmm128(bus, src);
                    let mut mask = 0u32;
                    for lane in 0..4 {
                        mask |= (Self::xmm_u32(value, lane) >> 31) << lane;
                    }
                    self.regs[reg] = mask;
                } else {
                    self.invalid_opcode = true;
                }
                true
            }
            0x51..=0x53 => {
                let (reg, src) = self.decode_modrm(bus, prefixes);
                if packed || scalar {
                    let source = self.read_xmm128(bus, src);
                    let mut result = self.xmm[reg];
                    let count = if scalar { 1 } else { 4 };
                    for lane in 0..count {
                        let value = f32::from_bits(Self::xmm_u32(source, lane));
                        let output = match opcode {
                            0x51 => value.sqrt(),
                            0x52 => 1.0 / value.sqrt(),
                            _ => 1.0 / value,
                        };
                        Self::xmm_set_u32(&mut result, lane, output.to_bits());
                    }
                    self.xmm[reg] = result;
                } else {
                    self.invalid_opcode = true;
                }
                true
            }
            0x54..=0x57 => {
                let (reg, src) = self.decode_modrm(bus, prefixes);
                if packed {
                    let lhs = self.xmm[reg];
                    let rhs = self.read_xmm128(bus, src);
                    self.xmm[reg] = match opcode {
                        0x54 => lhs & rhs,
                        0x55 => !lhs & rhs,
                        0x56 => lhs | rhs,
                        _ => lhs ^ rhs,
                    };
                } else {
                    self.invalid_opcode = true;
                }
                true
            }
            0x58 | 0x59 | 0x5c | 0x5d | 0x5e | 0x5f => {
                let (reg, src) = self.decode_modrm(bus, prefixes);
                if packed || scalar {
                    let source = self.read_xmm128(bus, src);
                    let mut result = self.xmm[reg];
                    let count = if scalar { 1 } else { 4 };
                    for lane in 0..count {
                        let lhs = f32::from_bits(Self::xmm_u32(result, lane));
                        let rhs = f32::from_bits(Self::xmm_u32(source, lane));
                        let output = match opcode {
                            0x5d => Self::sse_min_f32(lhs, rhs),
                            0x5f => Self::sse_max_f32(lhs, rhs),
                            _ => Self::sse_binary_f32(lhs, rhs, opcode),
                        };
                        Self::xmm_set_u32(&mut result, lane, output.to_bits());
                    }
                    self.xmm[reg] = result;
                } else {
                    self.invalid_opcode = true;
                }
                true
            }
            0x77 if packed => true,
            0xae => {
                let (group, operand) = self.decode_modrm(bus, prefixes);
                match (group, operand) {
                    (2, Operand::Memory(address)) => self.mxcsr = bus.read32(address) & 0xffff,
                    (3, Operand::Memory(address)) => bus.write32(address, self.mxcsr),
                    (7, Operand::Register(_)) => {}
                    _ => self.invalid_opcode = true,
                }
                true
            }
            0xc2 => {
                let (reg, src) = self.decode_modrm(bus, prefixes);
                let predicate = self.fetch8(bus);
                if packed || scalar {
                    let source = self.read_xmm128(bus, src);
                    let mut result = self.xmm[reg];
                    let count = if scalar { 1 } else { 4 };
                    for lane in 0..count {
                        let lhs = f32::from_bits(Self::xmm_u32(result, lane));
                        let rhs = f32::from_bits(Self::xmm_u32(source, lane));
                        Self::xmm_set_u32(
                            &mut result,
                            lane,
                            if Self::sse_compare_f32(lhs, rhs, predicate) {
                                u32::MAX
                            } else {
                                0
                            },
                        );
                    }
                    self.xmm[reg] = result;
                } else {
                    self.invalid_opcode = true;
                }
                true
            }
            0xc6 => {
                let (reg, src) = self.decode_modrm(bus, prefixes);
                let control = self.fetch8(bus);
                if packed {
                    let lhs = self.xmm[reg];
                    let rhs = self.read_xmm128(bus, src);
                    let mut result = 0u128;
                    Self::xmm_set_u32(&mut result, 0, Self::xmm_u32(lhs, usize::from(control & 3)));
                    Self::xmm_set_u32(
                        &mut result,
                        1,
                        Self::xmm_u32(lhs, usize::from((control >> 2) & 3)),
                    );
                    Self::xmm_set_u32(
                        &mut result,
                        2,
                        Self::xmm_u32(rhs, usize::from((control >> 4) & 3)),
                    );
                    Self::xmm_set_u32(
                        &mut result,
                        3,
                        Self::xmm_u32(rhs, usize::from((control >> 6) & 3)),
                    );
                    self.xmm[reg] = result;
                } else {
                    self.invalid_opcode = true;
                }
                true
            }
            _ => false,
        }
    }

    fn execute_two_byte<B: X86Bus>(&mut self, bus: &mut B, prefixes: Prefixes) {
        let opcode = self.fetch8(bus);
        if self.execute_sse(bus, opcode, prefixes) {
            return;
        }
        match opcode {
            0x1f => {
                let _ = self.decode_modrm(bus, prefixes);
            }
            0x31 => {
                self.regs[EAX] = self.tsc as u32;
                self.regs[EDX] = (self.tsc >> 32) as u32;
            }
            0xa2 => {
                let leaf = self.regs[EAX];
                match leaf {
                    0 => {
                        self.regs[EAX] = 1;
                        self.regs[EBX] = u32::from_le_bytes(*b"Genu");
                        self.regs[EDX] = u32::from_le_bytes(*b"ineI");
                        self.regs[ECX] = u32::from_le_bytes(*b"ntel");
                    }
                    1 => {
                        self.regs[EAX] = 0x0000_0686;
                        self.regs[EBX] = 0;
                        self.regs[ECX] = 0;
                        self.regs[EDX] = (1 << 0)
                            | (1 << 4)
                            | (1 << 5)
                            | (1 << 8)
                            | (1 << 15)
                            | (1 << 23)
                            | (1 << 24)
                            | (1 << 25);
                    }
                    _ => {
                        self.regs[EAX] = 0;
                        self.regs[EBX] = 0;
                        self.regs[ECX] = 0;
                        self.regs[EDX] = 0;
                    }
                }
            }
            0xb6 | 0xb7 | 0xbe | 0xbf => {
                let (reg, src) = self.decode_modrm(bus, prefixes);
                let value = match opcode {
                    0xb6 => u32::from(self.read_operand8(bus, src)),
                    0xb7 => u32::from(self.read_operand16(bus, src)),
                    0xbe => (self.read_operand8(bus, src) as i8 as i32) as u32,
                    _ => (self.read_operand16(bus, src) as i16 as i32) as u32,
                };
                self.regs[reg] = value;
            }
            0x40..=0x4f => {
                let (reg, src) = self.decode_modrm(bus, prefixes);
                if self.condition(opcode & 0x0f) {
                    if prefixes.operand16 {
                        let value = self.read_operand16(bus, src);
                        self.regs[reg] = (self.regs[reg] & 0xffff_0000) | u32::from(value);
                    } else {
                        self.regs[reg] = self.read_operand32(bus, src);
                    }
                }
            }
            0xaf => {
                let (reg, src) = self.decode_modrm(bus, prefixes);
                if prefixes.operand16 {
                    let lhs = self.regs[reg] as i16 as i32;
                    let rhs = self.read_operand16(bus, src) as i16 as i32;
                    let wide = lhs * rhs;
                    self.regs[reg] = (self.regs[reg] & 0xffff_0000) | u32::from(wide as u16);
                    self.eflags &= !(CF | OF);
                    if wide != wide as i16 as i32 {
                        self.eflags |= CF | OF;
                    }
                } else {
                    let lhs = self.regs[reg] as i32 as i64;
                    let rhs = self.read_operand32(bus, src) as i32 as i64;
                    let wide = lhs * rhs;
                    self.regs[reg] = wide as u32;
                    self.eflags &= !(CF | OF);
                    if wide != wide as i32 as i64 {
                        self.eflags |= CF | OF;
                    }
                }
            }
            0x80..=0x8f => {
                let displacement = self.fetch32(bus) as i32;
                if self.condition(opcode & 0x0f) {
                    self.eip = self.eip.wrapping_add_signed(displacement);
                }
            }
            0x90..=0x9f => {
                let (_, dst) = self.decode_modrm(bus, prefixes);
                self.write_operand8(bus, dst, u8::from(self.condition(opcode & 0x0f)));
            }
            0xbc | 0xbd => {
                let (reg, src) = self.decode_modrm(bus, prefixes);
                if prefixes.operand16 {
                    let value = self.read_operand16(bus, src);
                    if value == 0 {
                        self.eflags |= ZF;
                    } else {
                        self.eflags &= !ZF;
                        let result = if opcode == 0xbc {
                            value.trailing_zeros()
                        } else {
                            15 - value.leading_zeros()
                        };
                        self.regs[reg] = (self.regs[reg] & 0xffff_0000) | result;
                    }
                } else {
                    let value = self.read_operand32(bus, src);
                    if value == 0 {
                        self.eflags |= ZF;
                    } else {
                        self.eflags &= !ZF;
                        self.regs[reg] = if opcode == 0xbc {
                            value.trailing_zeros()
                        } else {
                            31 - value.leading_zeros()
                        };
                    }
                }
            }
            0xc8..=0xcf => {
                let reg = usize::from(opcode - 0xc8);
                self.regs[reg] = self.regs[reg].swap_bytes();
            }
            _ => self.invalid_opcode = true,
        }
    }

    fn execute_opcode<B: X86Bus>(&mut self, bus: &mut B, opcode: u8, prefixes: Prefixes) {
        match opcode {
            0x0f => self.execute_two_byte(bus, prefixes),
            0x40..=0x47 => {
                let reg = (opcode - 0x40) as usize;
                let old_cf = self.eflags & CF;
                let lhs = self.regs[reg];
                let result = lhs.wrapping_add(1);
                self.add_flags(lhs, 1, result, 32);
                self.eflags = (self.eflags & !CF) | old_cf;
                self.regs[reg] = result;
            }
            0x48..=0x4f => {
                let reg = (opcode - 0x48) as usize;
                let old_cf = self.eflags & CF;
                let lhs = self.regs[reg];
                let result = lhs.wrapping_sub(1);
                self.sub_flags(lhs, 1, result, 32);
                self.eflags = (self.eflags & !CF) | old_cf;
                self.regs[reg] = result;
            }
            0x50..=0x57 => self.push32(bus, self.regs[(opcode - 0x50) as usize]),
            0x58..=0x5f => {
                let reg = (opcode - 0x58) as usize;
                self.regs[reg] = self.pop32(bus);
            }
            0x68 => {
                let value = self.fetch32(bus);
                self.push32(bus, value);
            }
            0x69 | 0x6b => {
                let (reg, src) = self.decode_modrm(bus, prefixes);
                if prefixes.operand16 {
                    let lhs = self.read_operand16(bus, src) as i16 as i32;
                    let rhs = if opcode == 0x69 {
                        self.fetch16(bus) as i16 as i32
                    } else {
                        self.fetch8(bus) as i8 as i32
                    };
                    let wide = lhs * rhs;
                    self.regs[reg] = (self.regs[reg] & 0xffff_0000) | u32::from(wide as u16);
                    self.eflags &= !(CF | OF);
                    if wide != wide as i16 as i32 {
                        self.eflags |= CF | OF;
                    }
                } else {
                    let lhs = self.read_operand32(bus, src) as i32 as i64;
                    let rhs = if opcode == 0x69 {
                        self.fetch32(bus) as i32 as i64
                    } else {
                        self.fetch8(bus) as i8 as i64
                    };
                    let wide = lhs * rhs;
                    self.regs[reg] = wide as u32;
                    self.eflags &= !(CF | OF);
                    if wide != wide as i32 as i64 {
                        self.eflags |= CF | OF;
                    }
                }
            }
            0x6a => {
                let value = self.fetch8(bus) as i8 as i32 as u32;
                self.push32(bus, value);
            }
            0x6c..=0x6f => self.execute_string(bus, opcode, prefixes),
            0x70..=0x7f => {
                let displacement = self.fetch8(bus) as i8 as i32;
                if self.condition(opcode & 0x0f) {
                    self.eip = self.eip.wrapping_add_signed(displacement);
                }
            }
            0x86 | 0x87 => {
                let (reg, rm) = self.decode_modrm(bus, prefixes);
                if opcode == 0x86 {
                    let lhs = self.reg8(reg);
                    let rhs = self.read_operand8(bus, rm);
                    self.set_reg8(reg, rhs);
                    self.write_operand8(bus, rm, lhs);
                } else if prefixes.operand16 {
                    let lhs = self.regs[reg] as u16;
                    let rhs = self.read_operand16(bus, rm);
                    self.regs[reg] = (self.regs[reg] & 0xffff_0000) | u32::from(rhs);
                    self.write_operand16(bus, rm, lhs);
                } else {
                    let lhs = self.regs[reg];
                    let rhs = self.read_operand32(bus, rm);
                    self.regs[reg] = rhs;
                    self.write_operand32(bus, rm, lhs);
                }
            }
            0x88..=0x8b => {
                let (reg, rm) = self.decode_modrm(bus, prefixes);
                match opcode {
                    0x88 => self.write_operand8(bus, rm, self.reg8(reg)),
                    0x89 if prefixes.operand16 => {
                        self.write_operand16(bus, rm, self.regs[reg] as u16)
                    }
                    0x89 => self.write_operand32(bus, rm, self.regs[reg]),
                    0x8a => self.set_reg8(reg, self.read_operand8(bus, rm)),
                    0x8b if prefixes.operand16 => {
                        let value = self.read_operand16(bus, rm);
                        self.regs[reg] = (self.regs[reg] & 0xffff_0000) | u32::from(value);
                    }
                    _ => self.regs[reg] = self.read_operand32(bus, rm),
                }
            }
            0x8d => {
                let (reg, rm) = self.decode_modrm(bus, prefixes);
                if let Operand::Memory(address) = rm {
                    self.regs[reg] = address;
                } else {
                    self.invalid_opcode = true;
                }
            }
            0x90 => {}
            0x91..=0x97 => {
                let reg = (opcode - 0x90) as usize;
                self.regs.swap(EAX, reg);
            }
            0x98 => {
                if prefixes.operand16 {
                    let value = self.reg8(EAX) as i8 as i16 as u16;
                    self.regs[EAX] = (self.regs[EAX] & 0xffff_0000) | u32::from(value);
                } else {
                    self.regs[EAX] = self.regs[EAX] as u16 as i16 as i32 as u32;
                }
            }
            0x99 => {
                if prefixes.operand16 {
                    let value = if self.regs[EAX] & 0x8000 != 0 {
                        0xffff
                    } else {
                        0
                    };
                    self.regs[EDX] = (self.regs[EDX] & 0xffff_0000) | value;
                } else {
                    self.regs[EDX] = if self.regs[EAX] & 0x8000_0000 != 0 {
                        u32::MAX
                    } else {
                        0
                    };
                }
            }
            0x9c => self.push32(bus, self.eflags),
            0x9d => self.eflags = self.pop32(bus) | 0x2,
            0xa0 => {
                let address = prefixes.segment_base.wrapping_add(self.fetch32(bus));
                self.set_reg8(EAX, bus.read8(address));
            }
            0xa1 => {
                let address = prefixes.segment_base.wrapping_add(self.fetch32(bus));
                if prefixes.operand16 {
                    let value = bus.read16(address);
                    self.regs[EAX] = (self.regs[EAX] & 0xffff_0000) | u32::from(value);
                } else {
                    self.regs[EAX] = bus.read32(address);
                }
            }
            0xa2 => {
                let address = prefixes.segment_base.wrapping_add(self.fetch32(bus));
                bus.write8(address, self.reg8(EAX));
            }
            0xa3 => {
                let address = prefixes.segment_base.wrapping_add(self.fetch32(bus));
                if prefixes.operand16 {
                    bus.write16(address, self.regs[EAX] as u16);
                } else {
                    bus.write32(address, self.regs[EAX]);
                }
            }
            0xa4 | 0xa5 | 0xaa | 0xab | 0xac | 0xad => self.execute_string(bus, opcode, prefixes),
            0xa8 => {
                let rhs = self.fetch8(bus);
                self.logic_flags(u32::from(self.reg8(EAX) & rhs), 8);
            }
            0xa9 => {
                if prefixes.operand16 {
                    let rhs = self.fetch16(bus);
                    self.logic_flags(u32::from(self.regs[EAX] as u16 & rhs), 16);
                } else {
                    let rhs = self.fetch32(bus);
                    self.logic_flags(self.regs[EAX] & rhs, 32);
                }
            }
            0xb0..=0xb7 => {
                let reg = (opcode - 0xb0) as usize;
                let value = self.fetch8(bus);
                self.set_reg8(reg, value);
            }
            0xb8..=0xbf => {
                let reg = (opcode - 0xb8) as usize;
                if prefixes.operand16 {
                    let value = self.fetch16(bus);
                    self.regs[reg] = (self.regs[reg] & 0xffff_0000) | u32::from(value);
                } else {
                    self.regs[reg] = self.fetch32(bus);
                }
            }
            0xc2 => {
                let adjust = u32::from(self.fetch16(bus));
                self.eip = self.pop32(bus);
                self.regs[ESP] = self.regs[ESP].wrapping_add(adjust);
            }
            0xc3 => self.eip = self.pop32(bus),
            0xc6 | 0xc7 => {
                let (group, dst) = self.decode_modrm(bus, prefixes);
                if group != 0 {
                    self.invalid_opcode = true;
                    return;
                }
                if opcode == 0xc6 {
                    let value = self.fetch8(bus);
                    self.write_operand8(bus, dst, value);
                } else if prefixes.operand16 {
                    let value = self.fetch16(bus);
                    self.write_operand16(bus, dst, value);
                } else {
                    let value = self.fetch32(bus);
                    self.write_operand32(bus, dst, value);
                }
            }
            0xc9 => {
                self.regs[ESP] = self.regs[EBP];
                self.regs[EBP] = self.pop32(bus);
            }
            0xcc => self.invalid_opcode = true,
            0xcd => {
                let _ = self.fetch8(bus);
                self.invalid_opcode = true;
            }
            0xe8 => {
                let displacement = self.fetch32(bus) as i32;
                let return_address = self.eip;
                self.push32(bus, return_address);
                self.eip = self.eip.wrapping_add_signed(displacement);
            }
            0xe9 => {
                let displacement = self.fetch32(bus) as i32;
                self.eip = self.eip.wrapping_add_signed(displacement);
            }
            0xeb => {
                let displacement = self.fetch8(bus) as i8 as i32;
                self.eip = self.eip.wrapping_add_signed(displacement);
            }
            0xe4 => {
                let port = u16::from(self.fetch8(bus));
                let value = bus.io_read8(port);
                self.set_reg8(EAX, value);
            }
            0xe5 => {
                let port = u16::from(self.fetch8(bus));
                if prefixes.operand16 {
                    let value = bus.io_read16(port);
                    self.regs[EAX] = (self.regs[EAX] & 0xffff_0000) | u32::from(value);
                } else {
                    self.regs[EAX] = bus.io_read32(port);
                }
            }
            0xe6 => {
                let port = u16::from(self.fetch8(bus));
                bus.io_write8(port, self.reg8(EAX));
            }
            0xe7 => {
                let port = u16::from(self.fetch8(bus));
                if prefixes.operand16 {
                    bus.io_write16(port, self.regs[EAX] as u16);
                } else {
                    bus.io_write32(port, self.regs[EAX]);
                }
            }
            0xec => {
                let value = bus.io_read8(self.regs[EDX] as u16);
                self.set_reg8(EAX, value);
            }
            0xed => {
                let port = self.regs[EDX] as u16;
                if prefixes.operand16 {
                    let value = bus.io_read16(port);
                    self.regs[EAX] = (self.regs[EAX] & 0xffff_0000) | u32::from(value);
                } else {
                    self.regs[EAX] = bus.io_read32(port);
                }
            }
            0xee => bus.io_write8(self.regs[EDX] as u16, self.reg8(EAX)),
            0xef => {
                let port = self.regs[EDX] as u16;
                if prefixes.operand16 {
                    bus.io_write16(port, self.regs[EAX] as u16);
                } else {
                    bus.io_write32(port, self.regs[EAX]);
                }
            }
            0xf4 => self.halted = true,
            0x00 | 0x08 | 0x10 | 0x18 | 0x20 | 0x28 | 0x30 | 0x38 => {
                let (reg, dst) = self.decode_modrm(bus, prefixes);
                let op = usize::from((opcode >> 3) & 7);
                self.execute_group1_8(bus, op, dst, self.reg8(reg));
            }
            0x01 | 0x09 | 0x11 | 0x19 | 0x21 | 0x29 | 0x31 | 0x39 => {
                let (reg, dst) = self.decode_modrm(bus, prefixes);
                let op = usize::from((opcode >> 3) & 7);
                if prefixes.operand16 {
                    self.execute_group1_16(bus, op, dst, self.regs[reg] as u16);
                } else {
                    self.execute_group1_32(bus, op, dst, self.regs[reg]);
                }
            }
            0x02 | 0x0a | 0x12 | 0x1a | 0x22 | 0x2a | 0x32 | 0x3a => {
                let (reg, src) = self.decode_modrm(bus, prefixes);
                let op = usize::from((opcode >> 3) & 7);
                let rhs = self.read_operand8(bus, src);
                self.execute_group1_8(bus, op, Operand::Register(reg), rhs);
            }
            0x03 | 0x0b | 0x13 | 0x1b | 0x23 | 0x2b | 0x33 | 0x3b => {
                let (reg, src) = self.decode_modrm(bus, prefixes);
                let op = usize::from((opcode >> 3) & 7);
                if prefixes.operand16 {
                    let rhs = self.read_operand16(bus, src);
                    self.execute_group1_16(bus, op, Operand::Register(reg), rhs);
                } else {
                    let rhs = self.read_operand32(bus, src);
                    self.execute_group1_32(bus, op, Operand::Register(reg), rhs);
                }
            }
            0x04 | 0x0c | 0x14 | 0x1c | 0x24 | 0x2c | 0x34 | 0x3c => {
                let rhs = self.fetch8(bus);
                let op = usize::from((opcode >> 3) & 7);
                self.execute_group1_8(bus, op, Operand::Register(EAX), rhs);
            }
            0x05 | 0x0d | 0x15 | 0x1d | 0x25 | 0x2d | 0x35 | 0x3d => {
                let op = usize::from((opcode >> 3) & 7);
                if prefixes.operand16 {
                    let rhs = self.fetch16(bus);
                    self.execute_group1_16(bus, op, Operand::Register(EAX), rhs);
                } else {
                    let rhs = self.fetch32(bus);
                    self.execute_group1_32(bus, op, Operand::Register(EAX), rhs);
                }
            }
            0x80 => {
                let (op, dst) = self.decode_modrm(bus, prefixes);
                let rhs = self.fetch8(bus);
                self.execute_group1_8(bus, op, dst, rhs);
            }
            0x81 | 0x83 => {
                let (op, dst) = self.decode_modrm(bus, prefixes);
                if prefixes.operand16 {
                    let rhs = if opcode == 0x81 {
                        self.fetch16(bus)
                    } else {
                        self.fetch8(bus) as i8 as i16 as u16
                    };
                    self.execute_group1_16(bus, op, dst, rhs);
                } else {
                    let rhs = if opcode == 0x81 {
                        self.fetch32(bus)
                    } else {
                        self.fetch8(bus) as i8 as i32 as u32
                    };
                    self.execute_group1_32(bus, op, dst, rhs);
                }
            }
            0x84 | 0x85 => {
                let (reg, src) = self.decode_modrm(bus, prefixes);
                if opcode == 0x84 {
                    let value = self.reg8(reg) & self.read_operand8(bus, src);
                    self.logic_flags(u32::from(value), 8);
                } else if prefixes.operand16 {
                    let value = self.regs[reg] as u16 & self.read_operand16(bus, src);
                    self.logic_flags(u32::from(value), 16);
                } else {
                    let value = self.regs[reg] & self.read_operand32(bus, src);
                    self.logic_flags(value, 32);
                }
            }
            0xc1 | 0xd1 | 0xd3 => {
                let (op, dst) = self.decode_modrm(bus, prefixes);
                let count = match opcode {
                    0xc1 => u32::from(self.fetch8(bus)),
                    0xd1 => 1,
                    _ => self.regs[ECX] & 0xff,
                };
                self.execute_shift32(bus, op, dst, count);
            }
            0xf6 | 0xf7 => {
                let (op, dst) = self.decode_modrm(bus, prefixes);
                if opcode == 0xf6 {
                    self.execute_group3_8(bus, op, dst);
                } else {
                    self.execute_group3_32(bus, op, dst);
                }
            }
            0xf8 => self.eflags &= !CF,
            0xf9 => self.eflags |= CF,
            0xfa => self.eflags &= !IF,
            0xfb => self.eflags |= IF,
            0xfc => self.eflags &= !DF,
            0xfd => self.eflags |= DF,
            0xff => {
                let (op, dst) = self.decode_modrm(bus, prefixes);
                let value = self.read_operand32(bus, dst);
                match op {
                    0 => {
                        let old_cf = self.eflags & CF;
                        let r = value.wrapping_add(1);
                        self.add_flags(value, 1, r, 32);
                        self.eflags = (self.eflags & !CF) | old_cf;
                        self.write_operand32(bus, dst, r);
                    }
                    1 => {
                        let old_cf = self.eflags & CF;
                        let r = value.wrapping_sub(1);
                        self.sub_flags(value, 1, r, 32);
                        self.eflags = (self.eflags & !CF) | old_cf;
                        self.write_operand32(bus, dst, r);
                    }
                    2 => {
                        let ret = self.eip;
                        self.push32(bus, ret);
                        self.eip = value;
                    }
                    4 => self.eip = value,
                    6 => self.push32(bus, value),
                    _ => self.invalid_opcode = true,
                }
            }
            _ => self.invalid_opcode = true,
        }
    }

    fn execute_group3_8<B: X86Bus>(&mut self, bus: &mut B, op: usize, dst: Operand) {
        let value = self.read_operand8(bus, dst);
        match op {
            0 => {
                let rhs = self.fetch8(bus);
                self.logic_flags(u32::from(value & rhs), 8);
            }
            2 => self.write_operand8(bus, dst, !value),
            3 => {
                let result = 0u8.wrapping_sub(value);
                self.sub_flags(0, u32::from(value), u32::from(result), 8);
                self.write_operand8(bus, dst, result);
            }
            4 => {
                let result = u16::from(self.reg8(EAX)) * u16::from(value);
                self.regs[EAX] = (self.regs[EAX] & !0xffff) | u32::from(result);
                self.eflags &= !(CF | OF);
                if result > 0xff {
                    self.eflags |= CF | OF;
                }
            }
            5 => {
                let result = i16::from(self.reg8(EAX) as i8) * i16::from(value as i8);
                self.regs[EAX] = (self.regs[EAX] & !0xffff) | u32::from(result as u16);
                self.eflags &= !(CF | OF);
                if result != result as i8 as i16 {
                    self.eflags |= CF | OF;
                }
            }
            6 if value != 0 => {
                let dividend = self.regs[EAX] as u16;
                let quotient = dividend / u16::from(value);
                if quotient <= 0xff {
                    self.set_reg8(EAX, quotient as u8);
                    self.set_reg8(4, (dividend % u16::from(value)) as u8);
                } else {
                    self.invalid_opcode = true;
                }
            }
            7 if value != 0 => {
                let dividend = self.regs[EAX] as u16 as i16;
                let divisor = value as i8 as i16;
                let quotient = dividend / divisor;
                if (-128..=127).contains(&quotient) {
                    self.set_reg8(EAX, quotient as i8 as u8);
                    self.set_reg8(4, (dividend % divisor) as i8 as u8);
                } else {
                    self.invalid_opcode = true;
                }
            }
            6 | 7 => self.invalid_opcode = true,
            _ => self.invalid_opcode = true,
        }
    }

    fn execute_group3_32<B: X86Bus>(&mut self, bus: &mut B, op: usize, dst: Operand) {
        let value = self.read_operand32(bus, dst);
        match op {
            0 => {
                let rhs = self.fetch32(bus);
                self.logic_flags(value & rhs, 32);
            }
            2 => self.write_operand32(bus, dst, !value),
            3 => {
                let result = 0u32.wrapping_sub(value);
                self.sub_flags(0, value, result, 32);
                self.write_operand32(bus, dst, result);
            }
            4 => {
                let result = u64::from(self.regs[EAX]) * u64::from(value);
                self.regs[EAX] = result as u32;
                self.regs[EDX] = (result >> 32) as u32;
                self.eflags &= !(CF | OF);
                if self.regs[EDX] != 0 {
                    self.eflags |= CF | OF;
                }
            }
            5 => {
                let result = (self.regs[EAX] as i32 as i64) * (value as i32 as i64);
                self.regs[EAX] = result as u32;
                self.regs[EDX] = (result >> 32) as u32;
                self.eflags &= !(CF | OF);
                if result != result as i32 as i64 {
                    self.eflags |= CF | OF;
                }
            }
            6 if value != 0 => {
                let dividend = (u64::from(self.regs[EDX]) << 32) | u64::from(self.regs[EAX]);
                let quotient = dividend / u64::from(value);
                if quotient <= u64::from(u32::MAX) {
                    self.regs[EAX] = quotient as u32;
                    self.regs[EDX] = (dividend % u64::from(value)) as u32;
                } else {
                    self.invalid_opcode = true;
                }
            }
            7 if value != 0 => {
                let dividend = (((u64::from(self.regs[EDX]) << 32) | u64::from(self.regs[EAX]))
                    as i64) as i128;
                let divisor = value as i32 as i128;
                let quotient = dividend / divisor;
                if (i32::MIN as i128..=i32::MAX as i128).contains(&quotient) {
                    self.regs[EAX] = quotient as i32 as u32;
                    self.regs[EDX] = (dividend % divisor) as i32 as u32;
                } else {
                    self.invalid_opcode = true;
                }
            }
            6 | 7 => self.invalid_opcode = true,
            _ => self.invalid_opcode = true,
        }
    }

    pub fn save(&self, out: &mut StateWriter) {
        for reg in self.regs {
            out.u32(reg);
        }
        out.u32(self.eip);
        out.u32(self.eflags);
        out.u64(self.cycles);
        out.u32(self.fs_base);
        out.u32(self.gs_base);
        out.u32(self.cr0);
        out.u32(self.cr2);
        out.u32(self.cr3);
        out.u32(self.cr4);
        out.u64(self.tsc);
        for value in self.xmm {
            out.u64(value as u64);
            out.u64((value >> 64) as u64);
        }
        out.u32(self.mxcsr);
        out.u8(self.halted as u8);
        out.u8(self.interrupt_pending as u8);
        out.u8(self.invalid_opcode as u8);
    }

    pub fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        for reg in &mut self.regs {
            *reg = input.u32()?;
        }
        self.eip = input.u32()?;
        self.eflags = input.u32()? | 0x2;
        self.cycles = input.u64()?;
        self.fs_base = input.u32()?;
        self.gs_base = input.u32()?;
        self.cr0 = input.u32()?;
        self.cr2 = input.u32()?;
        self.cr3 = input.u32()?;
        self.cr4 = input.u32()?;
        self.tsc = input.u64()?;
        for value in &mut self.xmm {
            let low = u128::from(input.u64()?);
            let high = u128::from(input.u64()?);
            *value = low | (high << 64);
        }
        self.mxcsr = input.u32()?;
        self.halted = input.u8()? != 0;
        self.interrupt_pending = input.u8()? != 0;
        self.invalid_opcode = input.u8()? != 0;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::platform::PlatformId;

    struct RamBus {
        bytes: Vec<u8>,
        ports: Vec<u8>,
    }
    impl RamBus {
        fn new(size: usize) -> Self {
            Self {
                bytes: vec![0; size],
                ports: vec![0; 1 << 16],
            }
        }
    }
    impl X86Bus for RamBus {
        fn read8(&mut self, address: u32) -> u8 {
            self.bytes[address as usize % self.bytes.len()]
        }
        fn write8(&mut self, address: u32, value: u8) {
            let index = address as usize % self.bytes.len();
            self.bytes[index] = value;
        }
        fn io_read8(&mut self, port: u16) -> u8 {
            self.ports[usize::from(port)]
        }
        fn io_write8(&mut self, port: u16, value: u8) {
            self.ports[usize::from(port)] = value;
        }
    }

    #[test]
    fn executes_integer_stack_and_control_flow() {
        let mut bus = RamBus::new(0x2000);
        let code = [
            0xb8, 5, 0, 0, 0,    // mov eax,5
            0x50, // push eax
            0x5b, // pop ebx
            0x83, 0xc3, 7, // add ebx,7
            0x83, 0xfb, 12, // cmp ebx,12
            0x75, 2,    // jne +2
            0x40, // inc eax
            0xf4, // hlt
        ];
        bus.bytes[..code.len()].copy_from_slice(&code);
        let mut cpu = X86Cpu::new();
        cpu.regs[ESP] = 0x1800;
        while !cpu.halted() && !cpu.invalid_opcode() {
            cpu.step(&mut bus);
        }
        assert_eq!(cpu.regs[EAX], 6);
        assert_eq!(cpu.regs[EBX], 12);
        assert_eq!(cpu.regs[ESP], 0x1800);
        assert!(!cpu.invalid_opcode());
    }

    #[test]
    fn modrm_sib_and_rep_movsd_copy_memory() {
        let mut bus = RamBus::new(0x3000);
        bus.bytes[0x900..0x90c].copy_from_slice(&[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]);
        let code = [
            0xbe, 0x00, 0x09, 0, 0, // mov esi,0x900
            0xbf, 0x00, 0x0a, 0, 0, // mov edi,0xa00
            0xb9, 3, 0, 0, 0, // mov ecx,3
            0xf3, 0xa5, // rep movsd
            0x8b, 0x44, 0x24, 0x08, // mov eax,[esp+8]
            0xf4,
        ];
        bus.bytes[..code.len()].copy_from_slice(&code);
        bus.write32(0x1008, 0x1234_5678);
        let mut cpu = X86Cpu::new();
        cpu.regs[ESP] = 0x1000;
        while !cpu.halted() && !cpu.invalid_opcode() {
            cpu.step(&mut bus);
        }
        assert_eq!(&bus.bytes[0xa00..0xa0c], &bus.bytes[0x900..0x90c]);
        assert_eq!(cpu.regs[EAX], 0x1234_5678);
        assert_eq!(cpu.regs[ECX], 0);
    }

    #[test]
    fn port_io_supports_immediate_dx_and_operand_size_forms() {
        let mut bus = RamBus::new(0x1000);
        bus.ports[0x80] = 0x5a;
        bus.ports[0x84..0x88].copy_from_slice(&0x1234_5678u32.to_le_bytes());
        bus.ports[0xc006..0xc00a].copy_from_slice(&0xa1b2_c3d4u32.to_le_bytes());
        let code = [
            0xe4, 0x80, // in al,0x80
            0xe6, 0x90, // out 0x90,al
            0xe5, 0x84, // in eax,0x84
            0xe7, 0x94, // out 0x94,eax
            0xba, 0x06, 0xc0, 0x00, 0x00, // mov edx,0xc006
            0xec, // in al,dx
            0xee, // out dx,al
            0xed, // in eax,dx
            0x66, 0xed, // in ax,dx
            0x66, 0xef, // out dx,ax
            0xf4,
        ];
        bus.bytes[..code.len()].copy_from_slice(&code);
        let mut cpu = X86Cpu::new();
        while !cpu.halted() && !cpu.invalid_opcode() {
            cpu.step(&mut bus);
        }
        assert!(!cpu.invalid_opcode());
        assert_eq!(bus.ports[0x90], 0x5a);
        assert_eq!(&bus.ports[0x94..0x98], &0x1234_5678u32.to_le_bytes());
        assert_eq!(cpu.regs[EAX], 0xa1b2_c3d4);
        assert_eq!(&bus.ports[0xc006..0xc008], &0xc3d4u16.to_le_bytes());
    }

    #[test]
    fn adc_sbb_preserve_carry_borrow_and_auxiliary_flags_across_widths() {
        let mut bus = RamBus::new(0x1000);
        let code = [
            0xb8, 0xff, 0xff, 0xff, 0xff, // mov eax,-1
            0xf9, // stc
            0x15, 0, 0, 0, 0, // adc eax,0
            0x1d, 0, 0, 0, 0, // sbb eax,0
            0xb0, 0x0f, // mov al,0x0f
            0x04, 0x01, // add al,1
            0x66, 0xb8, 0xff, 0xff, // mov ax,0xffff
            0xf9, // stc
            0x66, 0x15, 0, 0, // adc ax,0
            0xf4,
        ];
        bus.bytes[..code.len()].copy_from_slice(&code);
        let mut cpu = X86Cpu::new();

        cpu.step(&mut bus);
        cpu.step(&mut bus);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[EAX], 0);
        assert_ne!(cpu.eflags & CF, 0);
        assert_ne!(cpu.eflags & ZF, 0);

        cpu.step(&mut bus);
        assert_eq!(cpu.regs[EAX], u32::MAX);
        assert_ne!(cpu.eflags & CF, 0);

        cpu.step(&mut bus);
        cpu.step(&mut bus);
        assert_eq!(cpu.reg8(EAX), 0x10);
        assert_ne!(cpu.eflags & AF, 0);

        cpu.step(&mut bus);
        cpu.step(&mut bus);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[EAX] as u16, 0);
        assert_ne!(cpu.eflags & CF, 0);
        assert_ne!(cpu.eflags & ZF, 0);
        assert!(!cpu.invalid_opcode());
    }

    #[test]
    fn compiler_integer_forms_cover_cmov_imul_bswap_xchg_and_nop() {
        let mut bus = RamBus::new(0x2000);
        let code = [
            0xb8, 3, 0, 0, 0, // mov eax,3
            0xbb, 0xf9, 0xff, 0xff, 0xff, // mov ebx,-7
            0x69, 0xc8, 5, 0, 0, 0, // imul ecx,eax,5
            0x6b, 0xd3, 0xfe, // imul edx,ebx,-2
            0x83, 0xf9, 15, // cmp ecx,15
            0x0f, 0x44, 0xf2, // cmove esi,edx
            0x0f, 0xce, // bswap esi
            0xbf, 0x44, 0x33, 0x22, 0x11, // mov edi,0x11223344
            0x87, 0xf8, // xchg eax,edi
            0x0f, 0x1f, 0x00, // nop dword [eax]
            0xf4,
        ];
        bus.bytes[..code.len()].copy_from_slice(&code);
        let mut cpu = X86Cpu::new();
        while !cpu.halted() && !cpu.invalid_opcode() {
            cpu.step(&mut bus);
        }
        assert!(!cpu.invalid_opcode());
        assert_eq!(cpu.regs[ECX], 15);
        assert_eq!(cpu.regs[EDX], 14);
        assert_eq!(cpu.regs[ESI], 0x0e00_0000);
        assert_eq!(cpu.regs[EAX], 0x1122_3344);
        assert_eq!(cpu.regs[EDI], 3);
    }

    #[test]
    fn operand_size_override_updates_only_low_word_for_integer_forms() {
        let mut bus = RamBus::new(0x2000);
        let code = [
            0xb8, 0x00, 0x00, 0xaa, 0xaa, // mov eax,0xaaaa0000
            0xbb, 0x02, 0x00, 0xbb, 0xbb, // mov ebx,0xbbbb0002
            0x66, 0xb8, 0x01, 0x00, // mov ax,1
            0x66, 0x01, 0xd8, // add ax,bx
            0x66, 0x89, 0xc1, // mov cx,ax
            0x66, 0x85, 0xc9, // test cx,cx
            0xf4,
        ];
        bus.bytes[..code.len()].copy_from_slice(&code);
        let mut cpu = X86Cpu::new();
        while !cpu.halted() && !cpu.invalid_opcode() {
            cpu.step(&mut bus);
        }
        assert!(!cpu.invalid_opcode());
        assert_eq!(cpu.regs[EAX], 0xaaaa_0003);
        assert_eq!(cpu.regs[ECX] as u16, 3);
        assert_eq!(cpu.eflags & ZF, 0);
    }

    #[test]
    fn cpuid_rdtsc_and_state_are_deterministic() {
        let mut bus = RamBus::new(0x1000);
        bus.bytes[..6].copy_from_slice(&[0x0f, 0xa2, 0x0f, 0x31, 0x90, 0xf4]);
        let mut cpu = X86Cpu::new();
        cpu.regs[EAX] = 0;
        cpu.tsc = 0x1122_3344_5566_7788;
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[EBX], u32::from_le_bytes(*b"Genu"));
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[EAX], 0x5566_7789);
        assert_eq!(cpu.regs[EDX], 0x1122_3344);
        cpu.fs_base = 0x4000;
        cpu.gs_base = 0x5000;
        let mut writer = StateWriter::new(PlatformId::Xbox, 91);
        cpu.save(&mut writer);
        let state = writer.finish();
        let mut reader = StateReader::new(&state, PlatformId::Xbox, 91).unwrap();
        let mut restored = X86Cpu::new();
        restored.load(&mut reader).unwrap();
        reader.finish().unwrap();
        assert_eq!(restored.regs, cpu.regs);
        assert_eq!(restored.eip, cpu.eip);
        assert_eq!(restored.eflags, cpu.eflags);
        assert_eq!(restored.tsc, cpu.tsc);
        assert_eq!(restored.fs_base, cpu.fs_base);
        assert_eq!(restored.gs_base, cpu.gs_base);
    }
}
