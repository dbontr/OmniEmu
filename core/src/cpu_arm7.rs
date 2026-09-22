use crate::state::{StateReader, StateWriter};

const N: u32 = 1 << 31;
const Z: u32 = 1 << 30;
const C: u32 = 1 << 29;
const V: u32 = 1 << 28;
const I: u32 = 1 << 7;
const F: u32 = 1 << 6;
const T: u32 = 1 << 5;
const MODE_MASK: u32 = 0x1f;
const USER: u32 = 0x10;
const FIQ: u32 = 0x11;
const IRQ: u32 = 0x12;
const SVC: u32 = 0x13;
const ABT: u32 = 0x17;
const UND: u32 = 0x1b;
const SYS: u32 = 0x1f;

pub trait Arm7Bus {
    fn read8(&mut self, address: u32) -> u8;
    fn write8(&mut self, address: u32, value: u8);

    fn read16(&mut self, address: u32) -> u16 {
        u16::from_le_bytes([self.read8(address), self.read8(address.wrapping_add(1))])
    }
    fn write16(&mut self, address: u32, value: u16) {
        let [low, high] = value.to_le_bytes();
        self.write8(address, low);
        self.write8(address.wrapping_add(1), high);
    }

    fn read32(&mut self, address: u32) -> u32 {
        u32::from_le_bytes([
            self.read8(address),
            self.read8(address.wrapping_add(1)),
            self.read8(address.wrapping_add(2)),
            self.read8(address.wrapping_add(3)),
        ])
    }
    fn write32(&mut self, address: u32, value: u32) {
        for (offset, byte) in value.to_le_bytes().into_iter().enumerate() {
            self.write8(address.wrapping_add(offset as u32), byte);
        }
    }
}

#[derive(Clone, Debug)]
pub struct Arm7 {
    pub r: [u32; 16],
    pub cpsr: u32,
    pub cycles: u64,
    banked_sp_lr: [[u32; 2]; 6],
    user_r8_12: [u32; 5],
    fiq_r8_12: [u32; 5],
    spsr: [u32; 5],
    halted: bool,
}

impl Default for Arm7 {
    fn default() -> Self {
        Self {
            r: [0; 16],
            cpsr: SVC | I | F,
            cycles: 0,
            banked_sp_lr: [[0; 2]; 6],
            user_r8_12: [0; 5],
            fiq_r8_12: [0; 5],
            spsr: [0; 5],
            halted: false,
        }
    }
}

impl Arm7 {
    pub fn reset(&mut self, pc: u32, stack: u32) {
        *self = Self::default();
        self.r[15] = pc & !3;
        self.r[13] = stack;
        self.banked_sp_lr[Self::bank_index(SVC)] = [stack, 0];
    }

    fn bank_index(mode: u32) -> usize {
        match mode & MODE_MASK {
            FIQ => 1,
            IRQ => 2,
            SVC => 3,
            ABT => 4,
            UND => 5,
            _ => 0,
        }
    }

    fn spsr_index(mode: u32) -> Option<usize> {
        Some(match mode & MODE_MASK {
            FIQ => 0,
            IRQ => 1,
            SVC => 2,
            ABT => 3,
            UND => 4,
            _ => return None,
        })
    }
    fn switch_mode(&mut self, new_mode: u32) {
        let old_mode = self.cpsr & MODE_MASK;
        if old_mode == new_mode {
            return;
        }
        let old_bank = Self::bank_index(old_mode);
        self.banked_sp_lr[old_bank] = [self.r[13], self.r[14]];
        if old_mode == FIQ {
            self.fiq_r8_12.copy_from_slice(&self.r[8..13]);
            self.r[8..13].copy_from_slice(&self.user_r8_12);
        } else if new_mode == FIQ {
            self.user_r8_12.copy_from_slice(&self.r[8..13]);
            self.r[8..13].copy_from_slice(&self.fiq_r8_12);
        }
        self.cpsr = (self.cpsr & !MODE_MASK) | new_mode;
        let new_bank = Self::bank_index(new_mode);
        [self.r[13], self.r[14]] = self.banked_sp_lr[new_bank];
    }

    fn current_spsr(&self) -> Option<u32> {
        Self::spsr_index(self.cpsr).map(|index| self.spsr[index])
    }

    fn set_current_spsr(&mut self, value: u32) {
        if let Some(index) = Self::spsr_index(self.cpsr) {
            self.spsr[index] = value;
        }
    }
    fn valid_mode(mode: u32) -> bool {
        matches!(mode & MODE_MASK, USER | FIQ | IRQ | SVC | ABT | UND | SYS)
    }

    fn write_cpsr(&mut self, value: u32, mask: u32) {
        let candidate = (self.cpsr & !mask) | (value & mask);
        let requested_mode = candidate & MODE_MASK;
        if mask & MODE_MASK != 0 && Self::valid_mode(requested_mode) {
            self.switch_mode(requested_mode);
        }
        let mode = self.cpsr & MODE_MASK;
        self.cpsr = (candidate & !MODE_MASK) | mode;
    }

    fn enter_exception(&mut self, mode: u32, vector: u32, return_address: u32) {
        let old = self.cpsr;
        self.switch_mode(mode);
        self.set_current_spsr(old);
        self.r[14] = return_address;
        self.cpsr |= I;
        if mode == FIQ {
            self.cpsr |= F;
        }
        self.cpsr &= !T;
        self.r[15] = vector;
        self.halted = false;
    }

    pub fn irq(&mut self) -> u32 {
        if self.cpsr & I != 0 {
            return 0;
        }
        let return_address = self.r[15].wrapping_add(4);
        self.enter_exception(IRQ, 0x18, return_address);
        self.cycles = self.cycles.wrapping_add(4);
        4
    }

    pub fn fiq(&mut self) -> u32 {
        if self.cpsr & F != 0 {
            return 0;
        }
        let return_address = self.r[15].wrapping_add(4);
        self.enter_exception(FIQ, 0x1c, return_address);
        self.cycles = self.cycles.wrapping_add(4);
        4
    }

    fn condition_passed(&self, condition: u32) -> bool {
        let n = self.cpsr & N != 0;
        let z = self.cpsr & Z != 0;
        let c = self.cpsr & C != 0;
        let v = self.cpsr & V != 0;
        match condition {
            0x0 => z,
            0x1 => !z,
            0x2 => c,
            0x3 => !c,
            0x4 => n,
            0x5 => !n,
            0x6 => v,
            0x7 => !v,
            0x8 => c && !z,
            0x9 => !c || z,
            0xa => n == v,
            0xb => n != v,
            0xc => !z && n == v,
            0xd => z || n != v,
            0xe => true,
            _ => false,
        }
    }

    fn read_reg(&self, index: usize) -> u32 {
        if index == 15 {
            self.r[15].wrapping_add(4)
        } else {
            self.r[index]
        }
    }

    fn read_thumb_reg(&self, index: usize) -> u32 {
        if index == 15 {
            self.r[15].wrapping_add(2)
        } else {
            self.r[index]
        }
    }

    fn branch_exchange(&mut self, target: u32) {
        if target & 1 != 0 {
            self.cpsr |= T;
            self.r[15] = target & !1;
        } else {
            self.cpsr &= !T;
            self.r[15] = target & !3;
        }
    }
    fn set_nz(&mut self, value: u32) {
        self.cpsr = if value & 0x8000_0000 != 0 {
            self.cpsr | N
        } else {
            self.cpsr & !N
        };
        self.cpsr = if value == 0 {
            self.cpsr | Z
        } else {
            self.cpsr & !Z
        };
    }

    fn set_carry(&mut self, value: bool) {
        self.cpsr = if value { self.cpsr | C } else { self.cpsr & !C };
    }

    fn set_overflow(&mut self, value: bool) {
        self.cpsr = if value { self.cpsr | V } else { self.cpsr & !V };
    }

    fn add_with_carry(a: u32, b: u32, carry: bool) -> (u32, bool, bool) {
        let wide = u64::from(a) + u64::from(b) + u64::from(carry);
        let result = wide as u32;
        let overflow = ((a ^ result) & (b ^ result) & 0x8000_0000) != 0;
        (result, wide >> 32 != 0, overflow)
    }

    fn shift(value: u32, kind: u32, amount: u32, carry_in: bool, register: bool) -> (u32, bool) {
        if register && amount == 0 {
            return (value, carry_in);
        }
        match kind {
            0 => match amount {
                0 => (value, carry_in),
                1..=31 => (value << amount, value >> (32 - amount) & 1 != 0),
                32 => (0, value & 1 != 0),
                _ => (0, false),
            },
            1 => {
                let amount = if !register && amount == 0 { 32 } else { amount };
                match amount {
                    1..=31 => (value >> amount, value >> (amount - 1) & 1 != 0),
                    32 => (0, value >> 31 != 0),
                    _ => (0, false),
                }
            }
            2 => {
                let amount = if !register && amount == 0 { 32 } else { amount };
                match amount {
                    1..=31 => (
                        ((value as i32) >> amount) as u32,
                        value >> (amount - 1) & 1 != 0,
                    ),
                    _ => {
                        let sign = value >> 31 != 0;
                        (if sign { u32::MAX } else { 0 }, sign)
                    }
                }
            }
            _ => {
                if !register && amount == 0 {
                    let result = (u32::from(carry_in) << 31) | (value >> 1);
                    return (result, value & 1 != 0);
                }
                let amount = amount & 31;
                if amount == 0 {
                    (value, value >> 31 != 0)
                } else {
                    (value.rotate_right(amount), value >> (amount - 1) & 1 != 0)
                }
            }
        }
    }
    fn operand2(&self, opcode: u32) -> (u32, bool) {
        let carry_in = self.cpsr & C != 0;
        if opcode & (1 << 25) != 0 {
            let immediate = opcode & 0xff;
            let rotate = ((opcode >> 8) & 0x0f) * 2;
            let result = immediate.rotate_right(rotate);
            return (
                result,
                if rotate == 0 {
                    carry_in
                } else {
                    result >> 31 != 0
                },
            );
        }
        let rm = (opcode & 0x0f) as usize;
        let value = self.read_reg(rm);
        let kind = (opcode >> 5) & 3;
        let by_register = opcode & (1 << 4) != 0;
        let amount = if by_register {
            let rs = ((opcode >> 8) & 0x0f) as usize;
            self.read_reg(rs) & 0xff
        } else {
            (opcode >> 7) & 0x1f
        };
        Self::shift(value, kind, amount, carry_in, by_register)
    }

    fn write_pc_or_reg(&mut self, rd: usize, value: u32, restore_status: bool) {
        if rd == 15 {
            if restore_status {
                if let Some(status) = self.current_spsr() {
                    self.write_cpsr(status, u32::MAX);
                }
            }
            self.r[15] = if self.cpsr & T != 0 {
                value & !1
            } else {
                value & !3
            };
        } else {
            self.r[rd] = value;
        }
    }
    fn data_processing(&mut self, opcode: u32) -> bool {
        let operation = (opcode >> 21) & 0x0f;
        let set_flags = opcode & (1 << 20) != 0;
        let rn = ((opcode >> 16) & 0x0f) as usize;
        let rd = ((opcode >> 12) & 0x0f) as usize;
        let a = self.read_reg(rn);
        let (b, shifter_carry) = self.operand2(opcode);
        let carry_in = self.cpsr & C != 0;
        let (result, carry, overflow, arithmetic) = match operation {
            0x0 => (a & b, shifter_carry, false, false),
            0x1 => (a ^ b, shifter_carry, false, false),
            0x2 => {
                let (r, c, v) = Self::add_with_carry(a, !b, true);
                (r, c, v, true)
            }
            0x3 => {
                let (r, c, v) = Self::add_with_carry(b, !a, true);
                (r, c, v, true)
            }
            0x4 => {
                let (r, c, v) = Self::add_with_carry(a, b, false);
                (r, c, v, true)
            }
            0x5 => {
                let (r, c, v) = Self::add_with_carry(a, b, carry_in);
                (r, c, v, true)
            }
            0x6 => {
                let (r, c, v) = Self::add_with_carry(a, !b, carry_in);
                (r, c, v, true)
            }
            0x7 => {
                let (r, c, v) = Self::add_with_carry(b, !a, carry_in);
                (r, c, v, true)
            }
            0x8 => (a & b, shifter_carry, false, false),
            0x9 => (a ^ b, shifter_carry, false, false),
            0xa => {
                let (r, c, v) = Self::add_with_carry(a, !b, true);
                (r, c, v, true)
            }
            0xb => {
                let (r, c, v) = Self::add_with_carry(a, b, false);
                (r, c, v, true)
            }
            0xc => (a | b, shifter_carry, false, false),
            0xd => (b, shifter_carry, false, false),
            0xe => (a & !b, shifter_carry, false, false),
            0xf => (!b, shifter_carry, false, false),
            _ => unreachable!(),
        };
        let test_only = (0x8..=0xb).contains(&operation);
        if set_flags || test_only {
            self.set_nz(result);
            self.set_carry(carry);
            if arithmetic {
                self.set_overflow(overflow);
            }
        }
        if !test_only {
            self.write_pc_or_reg(rd, result, set_flags && rd == 15);
        }
        true
    }

    fn arm_branch_exchange(&mut self, opcode: u32) -> bool {
        if opcode & 0x0fff_fff0 != 0x012f_ff10 {
            return false;
        }
        let rm = (opcode & 0x0f) as usize;
        self.branch_exchange(self.read_reg(rm));
        true
    }

    fn multiply(&mut self, opcode: u32) -> bool {
        if opcode & 0x0fc0_00f0 != 0x0000_0090 {
            return false;
        }
        let accumulate = opcode & (1 << 21) != 0;
        let set_flags = opcode & (1 << 20) != 0;
        let rd = ((opcode >> 16) & 0x0f) as usize;
        let rn = ((opcode >> 12) & 0x0f) as usize;
        let rs = ((opcode >> 8) & 0x0f) as usize;
        let rm = (opcode & 0x0f) as usize;
        let mut result = self.read_reg(rm).wrapping_mul(self.read_reg(rs));
        if accumulate {
            result = result.wrapping_add(self.read_reg(rn));
        }
        self.write_pc_or_reg(rd, result, false);
        if set_flags {
            self.set_nz(result);
        }
        true
    }

    fn is_multiply_long(opcode: u32) -> bool {
        opcode & 0x0f80_00f0 == 0x0080_0090
    }

    fn multiply_long(&mut self, opcode: u32) {
        let signed = opcode & (1 << 22) != 0;
        let accumulate = opcode & (1 << 21) != 0;
        let set_flags = opcode & (1 << 20) != 0;
        let rd_hi = ((opcode >> 16) & 0x0f) as usize;
        let rd_lo = ((opcode >> 12) & 0x0f) as usize;
        let rs = ((opcode >> 8) & 0x0f) as usize;
        let rm = (opcode & 0x0f) as usize;
        let product = if signed {
            (i64::from(self.read_reg(rm) as i32) * i64::from(self.read_reg(rs) as i32)) as u64
        } else {
            u64::from(self.read_reg(rm)) * u64::from(self.read_reg(rs))
        };
        let existing = (u64::from(self.r[rd_hi]) << 32) | u64::from(self.r[rd_lo]);
        let result = if accumulate {
            product.wrapping_add(existing)
        } else {
            product
        };
        self.r[rd_hi] = (result >> 32) as u32;
        self.r[rd_lo] = result as u32;
        if set_flags {
            self.cpsr = if result >> 63 != 0 {
                self.cpsr | N
            } else {
                self.cpsr & !N
            };
            self.cpsr = if result == 0 {
                self.cpsr | Z
            } else {
                self.cpsr & !Z
            };
        }
    }

    fn status_transfer(&mut self, opcode: u32) -> bool {
        if opcode & 0x0fbf_0fff == 0x010f_0000 {
            let rd = ((opcode >> 12) & 0x0f) as usize;
            let source_spsr = opcode & (1 << 22) != 0;
            let value = if source_spsr {
                self.current_spsr().unwrap_or(self.cpsr)
            } else {
                self.cpsr
            };
            self.write_pc_or_reg(rd, value, false);
            return true;
        }
        let immediate = opcode & (1 << 25) != 0;
        let register_form = opcode & 0x0fb0_fff0 == 0x0120_f000;
        let immediate_form = opcode & 0x0fb0_f000 == 0x0320_f000;
        if !register_form && !immediate_form {
            return false;
        }
        let field_mask = (opcode >> 16) & 0x0f;
        let mut mask = 0u32;
        if field_mask & 1 != 0 {
            mask |= 0x0000_00ff;
        }
        if field_mask & 2 != 0 {
            mask |= 0x0000_ff00;
        }
        if field_mask & 4 != 0 {
            mask |= 0x00ff_0000;
        }
        if field_mask & 8 != 0 {
            mask |= 0xff00_0000;
        }
        let value = if immediate {
            let rotate = ((opcode >> 8) & 0x0f) * 2;
            (opcode & 0xff).rotate_right(rotate)
        } else {
            self.read_reg((opcode & 0x0f) as usize)
        };
        if opcode & (1 << 22) != 0 {
            if let Some(index) = Self::spsr_index(self.cpsr) {
                self.spsr[index] = (self.spsr[index] & !mask) | (value & mask);
            }
        } else {
            self.write_cpsr(value, mask);
        }
        true
    }
    fn swap<B: Arm7Bus>(&mut self, bus: &mut B, opcode: u32) -> bool {
        if opcode & 0x0fb0_0ff0 != 0x0100_0090 {
            return false;
        }
        let byte = opcode & (1 << 22) != 0;
        let rn = ((opcode >> 16) & 0x0f) as usize;
        let rd = ((opcode >> 12) & 0x0f) as usize;
        let rm = (opcode & 0x0f) as usize;
        let address = self.read_reg(rn);
        let source = self.read_reg(rm);
        let previous = if byte {
            let value = u32::from(bus.read8(address));
            bus.write8(address, source as u8);
            value
        } else {
            let value = bus.read32(address & !3);
            bus.write32(address & !3, source);
            value
        };
        self.write_pc_or_reg(rd, previous, false);
        true
    }

    fn is_halfword_transfer(opcode: u32) -> bool {
        opcode & 0x0e00_0090 == 0x0000_0090 && opcode & 0x60 != 0
    }

    fn halfword_transfer<B: Arm7Bus>(&mut self, bus: &mut B, opcode: u32) -> bool {
        let pre = opcode & (1 << 24) != 0;
        let up = opcode & (1 << 23) != 0;
        let immediate = opcode & (1 << 22) != 0;
        let writeback = opcode & (1 << 21) != 0;
        let load = opcode & (1 << 20) != 0;
        let rn = ((opcode >> 16) & 0x0f) as usize;
        let rd = ((opcode >> 12) & 0x0f) as usize;
        let operation = (opcode >> 5) & 3;
        if !load && operation != 1 {
            return false;
        }
        let offset = if immediate {
            ((opcode >> 4) & 0xf0) | (opcode & 0x0f)
        } else {
            self.read_reg((opcode & 0x0f) as usize)
        };
        let base = self.read_reg(rn);
        let indexed = if up {
            base.wrapping_add(offset)
        } else {
            base.wrapping_sub(offset)
        };
        let address = if pre { indexed } else { base };
        if load {
            let value = match operation {
                1 => u32::from(bus.read16(address & !1)),
                2 => i32::from(bus.read8(address) as i8) as u32,
                3 => i32::from(bus.read16(address & !1) as i16) as u32,
                _ => return false,
            };
            self.write_pc_or_reg(rd, value, false);
        } else {
            bus.write16(address & !1, self.read_reg(rd) as u16);
        }
        if (!pre || writeback) && rn != 15 {
            self.r[rn] = indexed;
        }
        true
    }

    fn transfer_offset(&self, opcode: u32) -> u32 {
        if opcode & (1 << 25) == 0 {
            return opcode & 0x0fff;
        }
        let rm = (opcode & 0x0f) as usize;
        let kind = (opcode >> 5) & 3;
        let amount = (opcode >> 7) & 0x1f;
        Self::shift(self.read_reg(rm), kind, amount, self.cpsr & C != 0, false).0
    }
    fn single_transfer<B: Arm7Bus>(&mut self, bus: &mut B, opcode: u32) -> bool {
        let pre = opcode & (1 << 24) != 0;
        let up = opcode & (1 << 23) != 0;
        let byte = opcode & (1 << 22) != 0;
        let writeback = opcode & (1 << 21) != 0;
        let load = opcode & (1 << 20) != 0;
        let rn = ((opcode >> 16) & 0x0f) as usize;
        let rd = ((opcode >> 12) & 0x0f) as usize;
        let base = self.read_reg(rn);
        let offset = self.transfer_offset(opcode);
        let indexed = if up {
            base.wrapping_add(offset)
        } else {
            base.wrapping_sub(offset)
        };
        let address = if pre { indexed } else { base };
        if load {
            let value = if byte {
                u32::from(bus.read8(address))
            } else {
                bus.read32(address & !3).rotate_right((address & 3) * 8)
            };
            self.write_pc_or_reg(rd, value, false);
        } else if byte {
            bus.write8(address, self.read_reg(rd) as u8);
        } else {
            bus.write32(address & !3, self.read_reg(rd));
        }
        if (!pre || writeback) && rn != 15 {
            self.r[rn] = indexed;
        }
        true
    }
    fn block_transfer<B: Arm7Bus>(&mut self, bus: &mut B, opcode: u32) -> bool {
        let pre = opcode & (1 << 24) != 0;
        let up = opcode & (1 << 23) != 0;
        let restore = opcode & (1 << 22) != 0;
        let writeback = opcode & (1 << 21) != 0;
        let load = opcode & (1 << 20) != 0;
        let rn = ((opcode >> 16) & 0x0f) as usize;
        let registers = opcode & 0xffff;
        let count = registers.count_ones().max(1);
        let base = self.read_reg(rn);
        let mut address = if up {
            base.wrapping_add(if pre { 4 } else { 0 })
        } else {
            base.wrapping_sub(count * 4)
                .wrapping_add(if pre { 0 } else { 4 })
        };
        for register in 0..16 {
            if registers & (1 << register) == 0 {
                continue;
            }
            if load {
                let value = bus.read32(address);
                self.write_pc_or_reg(register, value, restore && register == 15);
            } else {
                bus.write32(address, self.read_reg(register));
            }
            address = address.wrapping_add(4);
        }
        if writeback && rn != 15 {
            self.r[rn] = if up {
                base.wrapping_add(count * 4)
            } else {
                base.wrapping_sub(count * 4)
            };
        }
        true
    }
    fn branch(&mut self, opcode: u32) -> u32 {
        let raw = (opcode & 0x00ff_ffff) << 2;
        let offset = ((raw << 6) as i32) >> 6;
        if opcode & (1 << 24) != 0 {
            self.r[14] = self.r[15];
        }
        self.r[15] = self.r[15].wrapping_add(4).wrapping_add_signed(offset) & !3;
        3
    }

    fn thumb_shift_add_sub(&mut self, opcode: u16) -> u32 {
        if opcode & 0x1800 != 0x1800 {
            let kind = u32::from((opcode >> 11) & 3);
            let amount = u32::from((opcode >> 6) & 0x1f);
            let rs = usize::from((opcode >> 3) & 7);
            let rd = usize::from(opcode & 7);
            let (result, carry) = Self::shift(self.r[rs], kind, amount, self.cpsr & C != 0, false);
            self.r[rd] = result;
            self.set_nz(result);
            self.set_carry(carry);
            return 1;
        }
        let immediate = opcode & (1 << 10) != 0;
        let subtract = opcode & (1 << 9) != 0;
        let operand = usize::from((opcode >> 6) & 7);
        let rs = usize::from((opcode >> 3) & 7);
        let rd = usize::from(opcode & 7);
        let right = if immediate {
            operand as u32
        } else {
            self.r[operand]
        };
        let (result, carry, overflow) = if subtract {
            Self::add_with_carry(self.r[rs], !right, true)
        } else {
            Self::add_with_carry(self.r[rs], right, false)
        };
        self.r[rd] = result;
        self.set_nz(result);
        self.set_carry(carry);
        self.set_overflow(overflow);
        1
    }

    fn thumb_immediate(&mut self, opcode: u16) -> u32 {
        let operation = (opcode >> 11) & 3;
        let rd = usize::from((opcode >> 8) & 7);
        let immediate = u32::from(opcode as u8);
        match operation {
            0 => {
                self.r[rd] = immediate;
                self.set_nz(immediate);
            }
            1 => {
                let (result, carry, overflow) = Self::add_with_carry(self.r[rd], !immediate, true);
                self.set_nz(result);
                self.set_carry(carry);
                self.set_overflow(overflow);
            }
            2 | 3 => {
                let (result, carry, overflow) = if operation == 2 {
                    Self::add_with_carry(self.r[rd], immediate, false)
                } else {
                    Self::add_with_carry(self.r[rd], !immediate, true)
                };
                self.r[rd] = result;
                self.set_nz(result);
                self.set_carry(carry);
                self.set_overflow(overflow);
            }
            _ => unreachable!(),
        }
        1
    }

    fn thumb_alu(&mut self, opcode: u16) -> u32 {
        let operation = (opcode >> 6) & 0x0f;
        let rs = usize::from((opcode >> 3) & 7);
        let rd = usize::from(opcode & 7);
        let a = self.r[rd];
        let b = self.r[rs];
        match operation {
            0x0 | 0x1 | 0xc | 0xe | 0xf => {
                let result = match operation {
                    0x0 => a & b,
                    0x1 => a ^ b,
                    0xc => a | b,
                    0xe => a & !b,
                    _ => !b,
                };
                self.r[rd] = result;
                self.set_nz(result);
            }
            0x2..=0x4 | 0x7 => {
                let kind = match operation {
                    0x2 => 0,
                    0x3 => 1,
                    0x4 => 2,
                    _ => 3,
                };
                let (result, carry) = Self::shift(a, kind, b & 0xff, self.cpsr & C != 0, true);
                self.r[rd] = result;
                self.set_nz(result);
                self.set_carry(carry);
            }
            0x5 | 0x6 => {
                let (result, carry, overflow) = if operation == 0x5 {
                    Self::add_with_carry(a, b, self.cpsr & C != 0)
                } else {
                    Self::add_with_carry(a, !b, self.cpsr & C != 0)
                };
                self.r[rd] = result;
                self.set_nz(result);
                self.set_carry(carry);
                self.set_overflow(overflow);
            }
            0x8 | 0xa | 0xb => {
                let (result, carry, overflow, arithmetic) = match operation {
                    0x8 => (a & b, self.cpsr & C != 0, false, false),
                    0xa => {
                        let (r, c, v) = Self::add_with_carry(a, !b, true);
                        (r, c, v, true)
                    }
                    _ => {
                        let (r, c, v) = Self::add_with_carry(a, b, false);
                        (r, c, v, true)
                    }
                };
                self.set_nz(result);
                if arithmetic {
                    self.set_carry(carry);
                    self.set_overflow(overflow);
                }
            }
            0x9 => {
                let (result, carry, overflow) = Self::add_with_carry(0, !b, true);
                self.r[rd] = result;
                self.set_nz(result);
                self.set_carry(carry);
                self.set_overflow(overflow);
            }
            0xd => {
                let result = a.wrapping_mul(b);
                self.r[rd] = result;
                self.set_nz(result);
                return 2;
            }
            _ => unreachable!(),
        }
        1
    }

    fn thumb_hi_register(&mut self, opcode: u16) -> u32 {
        let operation = (opcode >> 8) & 3;
        let rd = usize::from(opcode & 7) + if opcode & (1 << 7) != 0 { 8 } else { 0 };
        let rs = usize::from((opcode >> 3) & 7) + if opcode & (1 << 6) != 0 { 8 } else { 0 };
        let left = self.read_thumb_reg(rd);
        let right = self.read_thumb_reg(rs);
        match operation {
            0 => {
                let result = left.wrapping_add(right);
                if rd == 15 {
                    self.r[15] = result & !1;
                } else {
                    self.r[rd] = result;
                }
            }
            1 => {
                let (result, carry, overflow) = Self::add_with_carry(left, !right, true);
                self.set_nz(result);
                self.set_carry(carry);
                self.set_overflow(overflow);
            }
            2 => {
                if rd == 15 {
                    self.r[15] = right & !1;
                } else {
                    self.r[rd] = right;
                }
            }
            3 => {
                self.branch_exchange(right);
                return 3;
            }
            _ => unreachable!(),
        }
        1
    }

    fn thumb_register_transfer<B: Arm7Bus>(&mut self, bus: &mut B, opcode: u16) -> u32 {
        let operation = (opcode >> 9) & 7;
        let ro = usize::from((opcode >> 6) & 7);
        let rb = usize::from((opcode >> 3) & 7);
        let rd = usize::from(opcode & 7);
        let address = self.r[rb].wrapping_add(self.r[ro]);
        match operation {
            0 => bus.write32(address & !3, self.r[rd]),
            1 => bus.write16(address & !1, self.r[rd] as u16),
            2 => bus.write8(address, self.r[rd] as u8),
            3 => self.r[rd] = i32::from(bus.read8(address) as i8) as u32,
            4 => {
                self.r[rd] = bus.read32(address & !3).rotate_right((address & 3) * 8);
            }
            5 => self.r[rd] = u32::from(bus.read16(address & !1)),
            6 => self.r[rd] = u32::from(bus.read8(address)),
            7 => self.r[rd] = i32::from(bus.read16(address & !1) as i16) as u32,
            _ => unreachable!(),
        }
        if operation >= 3 {
            3
        } else {
            2
        }
    }

    fn thumb_literal_load<B: Arm7Bus>(&mut self, bus: &mut B, opcode: u16) -> u32 {
        let rd = usize::from((opcode >> 8) & 7);
        let base = self.r[15].wrapping_add(2) & !3;
        let address = base.wrapping_add(u32::from(opcode as u8) * 4);
        self.r[rd] = bus.read32(address);
        3
    }

    fn thumb_immediate_transfer<B: Arm7Bus>(&mut self, bus: &mut B, opcode: u16) -> u32 {
        let byte = opcode & (1 << 12) != 0;
        let load = opcode & (1 << 11) != 0;
        let offset = u32::from((opcode >> 6) & 0x1f) * if byte { 1 } else { 4 };
        let rb = usize::from((opcode >> 3) & 7);
        let rd = usize::from(opcode & 7);
        let address = self.r[rb].wrapping_add(offset);
        if load {
            self.r[rd] = if byte {
                u32::from(bus.read8(address))
            } else {
                bus.read32(address & !3).rotate_right((address & 3) * 8)
            };
            3
        } else {
            if byte {
                bus.write8(address, self.r[rd] as u8);
            } else {
                bus.write32(address & !3, self.r[rd]);
            }
            2
        }
    }

    fn thumb_halfword_immediate<B: Arm7Bus>(&mut self, bus: &mut B, opcode: u16) -> u32 {
        let load = opcode & (1 << 11) != 0;
        let offset = u32::from((opcode >> 6) & 0x1f) * 2;
        let rb = usize::from((opcode >> 3) & 7);
        let rd = usize::from(opcode & 7);
        let address = self.r[rb].wrapping_add(offset) & !1;
        if load {
            self.r[rd] = u32::from(bus.read16(address));
            3
        } else {
            bus.write16(address, self.r[rd] as u16);
            2
        }
    }

    fn thumb_sp_transfer<B: Arm7Bus>(&mut self, bus: &mut B, opcode: u16) -> u32 {
        let load = opcode & (1 << 11) != 0;
        let rd = usize::from((opcode >> 8) & 7);
        let address = self.r[13].wrapping_add(u32::from(opcode as u8) * 4);
        if load {
            self.r[rd] = bus.read32(address & !3).rotate_right((address & 3) * 8);
            3
        } else {
            bus.write32(address & !3, self.r[rd]);
            2
        }
    }

    fn thumb_load_address(&mut self, opcode: u16) -> u32 {
        let rd = usize::from((opcode >> 8) & 7);
        let offset = u32::from(opcode as u8) * 4;
        let base = if opcode & (1 << 11) != 0 {
            self.r[13]
        } else {
            self.r[15].wrapping_add(2) & !3
        };
        self.r[rd] = base.wrapping_add(offset);
        1
    }

    fn thumb_adjust_sp(&mut self, opcode: u16) -> u32 {
        let offset = u32::from(opcode & 0x7f) * 4;
        self.r[13] = if opcode & 0x80 != 0 {
            self.r[13].wrapping_sub(offset)
        } else {
            self.r[13].wrapping_add(offset)
        };
        1
    }

    fn thumb_push_pop<B: Arm7Bus>(&mut self, bus: &mut B, opcode: u16) -> u32 {
        let pop = opcode & (1 << 11) != 0;
        let extra = opcode & (1 << 8) != 0;
        let list = opcode as u8;
        let count = list.count_ones() + u32::from(extra);
        if pop {
            let mut address = self.r[13];
            for register in 0..8 {
                if list & (1 << register) != 0 {
                    self.r[register] = bus.read32(address);
                    address = address.wrapping_add(4);
                }
            }
            if extra {
                self.r[15] = bus.read32(address) & !1;
                address = address.wrapping_add(4);
            }
            self.r[13] = address;
        } else {
            let mut address = self.r[13].wrapping_sub(count * 4);
            self.r[13] = address;
            for register in 0..8 {
                if list & (1 << register) != 0 {
                    bus.write32(address, self.r[register]);
                    address = address.wrapping_add(4);
                }
            }
            if extra {
                bus.write32(address, self.r[14]);
            }
        }
        2 + count
    }

    fn thumb_multiple<B: Arm7Bus>(&mut self, bus: &mut B, opcode: u16) -> u32 {
        let load = opcode & (1 << 11) != 0;
        let rb = usize::from((opcode >> 8) & 7);
        let list = opcode as u8;
        if list == 0 {
            return self.undefined();
        }
        let base = self.r[rb];
        let mut address = base;
        for register in 0..8 {
            if list & (1 << register) == 0 {
                continue;
            }
            if load {
                self.r[register] = bus.read32(address);
            } else {
                bus.write32(address, self.r[register]);
            }
            address = address.wrapping_add(4);
        }
        self.r[rb] = base.wrapping_add(list.count_ones() * 4);
        2 + list.count_ones()
    }

    fn thumb_conditional_branch(&mut self, opcode: u16) -> u32 {
        let condition = u32::from((opcode >> 8) & 0x0f);
        if condition == 0x0f {
            let return_address = self.r[15];
            self.enter_exception(SVC, 0x08, return_address);
            return 3;
        }
        if condition == 0x0e {
            return self.undefined();
        }
        if self.condition_passed(condition) {
            let offset = i32::from(opcode as u8 as i8) * 2;
            self.r[15] = self.r[15].wrapping_add(2).wrapping_add_signed(offset) & !1;
            3
        } else {
            1
        }
    }

    fn thumb_unconditional_branch(&mut self, opcode: u16) -> u32 {
        let raw = i32::from(opcode & 0x07ff);
        let signed = if raw & 0x0400 != 0 {
            raw | !0x07ff
        } else {
            raw
        };
        self.r[15] = self.r[15].wrapping_add(2).wrapping_add_signed(signed * 2) & !1;
        3
    }

    fn step_thumb<B: Arm7Bus>(&mut self, bus: &mut B) -> u32 {
        let current = self.r[15] & !1;
        let opcode = bus.read16(current);
        self.r[15] = current.wrapping_add(2);
        let used = if opcode & 0xe000 == 0x0000 {
            self.thumb_shift_add_sub(opcode)
        } else if opcode & 0xe000 == 0x2000 {
            self.thumb_immediate(opcode)
        } else if opcode & 0xfc00 == 0x4000 {
            self.thumb_alu(opcode)
        } else if opcode & 0xfc00 == 0x4400 {
            self.thumb_hi_register(opcode)
        } else if opcode & 0xf800 == 0x4800 {
            self.thumb_literal_load(bus, opcode)
        } else if opcode & 0xf000 == 0x5000 {
            self.thumb_register_transfer(bus, opcode)
        } else if opcode & 0xe000 == 0x6000 {
            self.thumb_immediate_transfer(bus, opcode)
        } else if opcode & 0xf000 == 0x8000 {
            self.thumb_halfword_immediate(bus, opcode)
        } else if opcode & 0xf000 == 0x9000 {
            self.thumb_sp_transfer(bus, opcode)
        } else if opcode & 0xf000 == 0xa000 {
            self.thumb_load_address(opcode)
        } else if opcode & 0xff00 == 0xb000 {
            self.thumb_adjust_sp(opcode)
        } else if opcode & 0xf600 == 0xb400 {
            self.thumb_push_pop(bus, opcode)
        } else if opcode & 0xf000 == 0xc000 {
            self.thumb_multiple(bus, opcode)
        } else if opcode & 0xf000 == 0xd000 {
            self.thumb_conditional_branch(opcode)
        } else if opcode & 0xf800 == 0xe000 {
            self.thumb_unconditional_branch(opcode)
        } else if opcode & 0xf800 == 0xf000 {
            let raw = i32::from(opcode & 0x07ff);
            let high = if raw & 0x0400 != 0 {
                raw | !0x07ff
            } else {
                raw
            };
            self.r[14] = self.r[15].wrapping_add(2).wrapping_add_signed(high << 12);
            1
        } else if opcode & 0xf800 == 0xf800 {
            let target = self.r[14].wrapping_add(u32::from(opcode & 0x07ff) << 1);
            self.r[14] = self.r[15] | 1;
            self.r[15] = target & !1;
            3
        } else {
            self.undefined()
        };
        self.cycles = self.cycles.wrapping_add(u64::from(used));
        used
    }

    fn undefined(&mut self) -> u32 {
        let return_address = self.r[15];
        self.enter_exception(UND, 0x04, return_address);
        4
    }

    pub fn step<B: Arm7Bus>(&mut self, bus: &mut B) -> u32 {
        if self.halted {
            self.cycles = self.cycles.wrapping_add(1);
            return 1;
        }
        if self.cpsr & T != 0 {
            return self.step_thumb(bus);
        }
        let current = self.r[15];
        let opcode = bus.read32(current);
        self.r[15] = current.wrapping_add(4);
        if !self.condition_passed(opcode >> 28) {
            self.cycles = self.cycles.wrapping_add(1);
            return 1;
        }
        let used = if opcode & 0x0f00_0000 == 0x0f00_0000 {
            let return_address = self.r[15];
            self.enter_exception(SVC, 0x08, return_address);
            3
        } else if self.arm_branch_exchange(opcode) {
            3
        } else if opcode & 0x0e00_0000 == 0x0a00_0000 {
            self.branch(opcode)
        } else if opcode & 0x0e00_0000 == 0x0800_0000 {
            self.block_transfer(bus, opcode);
            2 + u32::max(1, (opcode & 0xffff).count_ones())
        } else if opcode & 0x0c00_0000 == 0x0400_0000 {
            let load = opcode & (1 << 20) != 0;
            self.single_transfer(bus, opcode);
            if load {
                3
            } else {
                2
            }
        } else if Self::is_halfword_transfer(opcode) {
            let load = opcode & (1 << 20) != 0;
            if self.halfword_transfer(bus, opcode) {
                if load {
                    3
                } else {
                    2
                }
            } else {
                self.undefined()
            }
        } else if Self::is_multiply_long(opcode) {
            self.multiply_long(opcode);
            3
        } else if self.status_transfer(opcode) {
            1
        } else if self.swap(bus, opcode) {
            4
        } else if self.multiply(opcode) {
            2
        } else if opcode & 0x0c00_0000 == 0 {
            self.data_processing(opcode);
            1
        } else {
            self.undefined()
        };
        self.cycles = self.cycles.wrapping_add(u64::from(used));
        used
    }

    pub fn save(&self, out: &mut StateWriter) {
        for value in self.r {
            out.u32(value);
        }
        out.u32(self.cpsr);
        out.u64(self.cycles);
        for bank in self.banked_sp_lr {
            out.u32(bank[0]);
            out.u32(bank[1]);
        }
        for value in self.user_r8_12 {
            out.u32(value);
        }
        for value in self.fiq_r8_12 {
            out.u32(value);
        }
        for value in self.spsr {
            out.u32(value);
        }
        out.u8(u8::from(self.halted));
    }
    pub fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        for value in &mut self.r {
            *value = input.u32()?;
        }
        self.cpsr = input.u32()?;
        if !Self::valid_mode(self.cpsr) {
            return Err("ARM7 state contains an invalid processor mode".into());
        }
        self.cycles = input.u64()?;
        for bank in &mut self.banked_sp_lr {
            bank[0] = input.u32()?;
            bank[1] = input.u32()?;
        }
        for value in &mut self.user_r8_12 {
            *value = input.u32()?;
        }
        for value in &mut self.fiq_r8_12 {
            *value = input.u32()?;
        }
        for value in &mut self.spsr {
            *value = input.u32()?;
        }
        self.halted = input.u8()? != 0;
        self.r[15] &= if self.cpsr & T != 0 { !1 } else { !3 };
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::platform::PlatformId;

    struct TestBus {
        data: Vec<u8>,
    }

    impl TestBus {
        fn new() -> Self {
            Self {
                data: vec![0; 0x2000],
            }
        }
        fn half(&mut self, address: usize, value: u16) {
            self.data[address..address + 2].copy_from_slice(&value.to_le_bytes());
        }
        fn word(&mut self, address: usize, value: u32) {
            self.data[address..address + 4].copy_from_slice(&value.to_le_bytes());
        }
    }
    impl Arm7Bus for TestBus {
        fn read8(&mut self, address: u32) -> u8 {
            self.data[address as usize % self.data.len()]
        }
        fn write8(&mut self, address: u32, value: u8) {
            let index = address as usize % self.data.len();
            self.data[index] = value;
        }
    }

    fn arm_half(load: bool, operation: u32, rn: u32, rd: u32, offset: u32) -> u32 {
        0xe1c0_0090
            | (u32::from(load) << 20)
            | (rn << 16)
            | (rd << 12)
            | ((offset & 0xf0) << 4)
            | (operation << 5)
            | (offset & 0x0f)
    }

    fn arm_long_mul(signed: bool, rd_hi: u32, rd_lo: u32, rs: u32, rm: u32) -> u32 {
        0xe080_0090 | (u32::from(signed) << 22) | (rd_hi << 16) | (rd_lo << 12) | (rs << 8) | rm
    }

    #[test]
    fn arithmetic_conditions_and_branch_pipeline_execute() {
        let mut bus = TestBus::new();
        for (index, opcode) in [
            0xe3a0_0005,
            0xe3a0_1007,
            0xe080_2001,
            0xe352_000c,
            0x03a0_3001,
            0x13a0_3002,
            0xea00_0000,
            0xe3a0_4009,
            0xe3a0_4004,
        ]
        .into_iter()
        .enumerate()
        {
            bus.word(index * 4, opcode);
        }
        let mut cpu = Arm7::default();
        for _ in 0..8 {
            cpu.step(&mut bus);
        }
        assert_eq!(cpu.r[2], 12);
        assert_eq!(cpu.r[3], 1);
        assert_eq!(cpu.r[4], 4);
        assert!(cpu.cpsr & Z != 0);
    }
    #[test]
    fn load_store_block_transfer_and_unaligned_load_work() {
        let mut bus = TestBus::new();
        for (index, opcode) in [
            0xe3a0_0c01,
            0xe3a0_1044,
            0xe580_1000,
            0xe590_2000,
            0xe8a0_0006,
            0xe240_0008,
            0xe890_0018,
        ]
        .into_iter()
        .enumerate()
        {
            bus.word(index * 4, opcode);
        }
        let mut cpu = Arm7::default();
        for _ in 0..7 {
            cpu.step(&mut bus);
        }
        assert_eq!(bus.read32(0x100), 0x44);
        assert_eq!(cpu.r[2], 0x44);
        assert_eq!(cpu.r[3], 0x44);
        assert_eq!(cpu.r[4], 0x44);
        bus.word(0x180, 0x1122_3344);
        bus.word(0x40, 0xe591_5001);
        cpu.r[1] = 0x180;
        cpu.r[15] = 0x40;
        cpu.step(&mut bus);
        assert_eq!(cpu.r[5], 0x4411_2233);
    }

    #[test]
    fn multiply_exceptions_and_banked_modes_are_preserved() {
        let mut bus = TestBus::new();
        bus.word(0x100, 0xe002_0190);
        bus.word(0x104, 0xef00_0001);
        let mut cpu = Arm7::default();
        cpu.reset(0x100, 0x1f00);
        cpu.r[0] = 9;
        cpu.r[1] = 7;
        cpu.step(&mut bus);
        assert_eq!(cpu.r[2], 63);
        cpu.step(&mut bus);
        assert_eq!(cpu.r[15], 0x08);
        assert_eq!(cpu.r[14], 0x108);
        cpu.r[13] = 0x1800;
        cpu.switch_mode(FIQ);
        cpu.r[8] = 0xfeed_beef;
        cpu.r[13] = 0x1700;
        cpu.switch_mode(SVC);
        assert_eq!(cpu.r[13], 0x1800);
        cpu.switch_mode(FIQ);
        assert_eq!(cpu.r[8], 0xfeed_beef);
        assert_eq!(cpu.r[13], 0x1700);
        cpu.cpsr &= !I;
        assert_eq!(cpu.irq(), 4);
        assert_eq!(cpu.cpsr & MODE_MASK, IRQ);
        assert_eq!(cpu.r[15], 0x18);
    }

    #[test]
    fn bx_enters_thumb_and_thumb_alu_returns_to_arm() {
        let mut bus = TestBus::new();
        bus.word(0, 0xe12f_ff11);
        bus.half(0x100, 0x2005);
        bus.half(0x102, 0x3007);
        bus.half(0x104, 0x280c);
        bus.half(0x106, 0x4710);
        bus.word(0x200, 0xe3a0_3009);
        let mut cpu = Arm7::default();
        cpu.r[1] = 0x101;
        cpu.r[2] = 0x200;
        assert_ne!(cpu.step(&mut bus), 0);
        assert_ne!(cpu.cpsr & T, 0);
        assert_eq!(cpu.r[15], 0x100);
        for _ in 0..3 {
            assert_ne!(cpu.step(&mut bus), 0);
        }
        assert_eq!(cpu.r[0], 12);
        assert_ne!(cpu.cpsr & Z, 0);
        assert_ne!(cpu.step(&mut bus), 0);
        assert_eq!(cpu.cpsr & T, 0);
        assert_eq!(cpu.r[15], 0x200);
        assert_ne!(cpu.step(&mut bus), 0);
        assert_eq!(cpu.r[3], 9);
    }

    #[test]
    fn armv4_halfword_signed_transfers_and_long_multiply_execute() {
        let mut bus = TestBus::new();
        bus.word(0, arm_half(false, 1, 0, 1, 2));
        bus.word(4, arm_half(true, 3, 0, 2, 2));
        bus.word(8, arm_half(true, 2, 0, 3, 4));
        bus.word(12, arm_long_mul(true, 5, 4, 7, 6));
        bus.data[0x304] = 0x80;
        let mut cpu = Arm7::default();
        cpu.r[0] = 0x300;
        cpu.r[1] = 0x0000_ff80;
        cpu.r[6] = 0xffff_fffe;
        cpu.r[7] = 3;
        for _ in 0..4 {
            assert_ne!(cpu.step(&mut bus), 0);
        }
        assert_eq!(bus.read16(0x302), 0xff80);
        assert_eq!(cpu.r[2], 0xffff_ff80);
        assert_eq!(cpu.r[3], 0xffff_ff80);
        assert_eq!(cpu.r[5], 0xffff_ffff);
        assert_eq!(cpu.r[4], 0xffff_fffa);
    }

    #[test]
    fn thumb_memory_stack_and_pop_pc_execute() {
        let mut bus = TestBus::new();
        for (address, opcode) in [
            (0x100, 0x6001),
            (0x102, 0x6802),
            (0x104, 0xb506),
            (0x106, 0x2100),
            (0x108, 0x2200),
            (0x10a, 0xbd06),
            (0x118, 0x2309),
        ] {
            bus.half(address, opcode);
        }
        let mut cpu = Arm7::default();
        cpu.cpsr |= T;
        cpu.r[15] = 0x100;
        cpu.r[0] = 0x300;
        cpu.r[1] = 0xa5a5_5a5a;
        cpu.r[13] = 0x800;
        cpu.r[14] = 0x119;
        for _ in 0..6 {
            assert_ne!(cpu.step(&mut bus), 0);
        }
        assert_eq!(cpu.r[15], 0x118);
        assert_eq!(cpu.r[1], 0xa5a5_5a5a);
        assert_eq!(cpu.r[2], 0xa5a5_5a5a);
        assert_eq!(cpu.r[13], 0x800);
        assert_ne!(cpu.step(&mut bus), 0);
        assert_eq!(cpu.r[3], 9);
    }

    #[test]
    fn state_round_trip_preserves_all_execution_banks() {
        let mut cpu = Arm7::default();
        cpu.r[0] = 0x1234_5678;
        cpu.r[15] = 0x400;
        cpu.cycles = 99;
        cpu.switch_mode(IRQ);
        cpu.r[13] = 0x1e00;
        cpu.set_current_spsr(0xa000_0013);
        cpu.cpsr |= T;
        let mut out = StateWriter::new(PlatformId::Dreamcast, 1);
        cpu.save(&mut out);
        let bytes = out.finish();
        let mut input = StateReader::new(&bytes, PlatformId::Dreamcast, 1).unwrap();
        let mut restored = Arm7::default();
        restored.load(&mut input).unwrap();
        input.finish().unwrap();
        assert_eq!(restored.r, cpu.r);
        assert_eq!(restored.cpsr, cpu.cpsr);
        assert_eq!(restored.cycles, cpu.cycles);
        assert_eq!(restored.banked_sp_lr, cpu.banked_sp_lr);
        assert_eq!(restored.user_r8_12, cpu.user_r8_12);
        assert_eq!(restored.fiq_r8_12, cpu.fiq_r8_12);
        assert_eq!(restored.spsr, cpu.spsr);
    }
}
