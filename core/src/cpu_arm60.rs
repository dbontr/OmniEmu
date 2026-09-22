use crate::state::{StateReader, StateWriter};

const N: u32 = 1 << 31;
const Z: u32 = 1 << 30;
const C: u32 = 1 << 29;
const V: u32 = 1 << 28;
const I: u32 = 1 << 7;
const F: u32 = 1 << 6;
const MODE_MASK: u32 = 0x1f;
const USER: u32 = 0x10;
const FIQ: u32 = 0x11;
const IRQ: u32 = 0x12;
const SVC: u32 = 0x13;
const ABT: u32 = 0x17;
const UND: u32 = 0x1b;
const SYS: u32 = 0x1f;

pub trait Arm60Bus {
    fn read8(&mut self, address: u32) -> u8;
    fn write8(&mut self, address: u32, value: u8);

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
pub struct Arm60 {
    pub r: [u32; 16],
    pub cpsr: u32,
    pub cycles: u64,
    banked_sp_lr: [[u32; 2]; 6],
    user_r8_12: [u32; 5],
    fiq_r8_12: [u32; 5],
    spsr: [u32; 5],
    halted: bool,
}

impl Default for Arm60 {
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

impl Arm60 {
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
            self.r[15] = value & !3;
            if restore_status {
                if let Some(status) = self.current_spsr() {
                    self.write_cpsr(status, u32::MAX);
                }
            }
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
    fn swap<B: Arm60Bus>(&mut self, bus: &mut B, opcode: u32) -> bool {
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

    fn transfer_offset(&self, opcode: u32) -> u32 {
        if opcode & (1 << 25) == 0 {
            return opcode & 0x0fff;
        }
        let rm = (opcode & 0x0f) as usize;
        let kind = (opcode >> 5) & 3;
        let amount = (opcode >> 7) & 0x1f;
        Self::shift(self.read_reg(rm), kind, amount, self.cpsr & C != 0, false).0
    }
    fn single_transfer<B: Arm60Bus>(&mut self, bus: &mut B, opcode: u32) -> bool {
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
    fn block_transfer<B: Arm60Bus>(&mut self, bus: &mut B, opcode: u32) -> bool {
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

    fn undefined(&mut self) -> u32 {
        let return_address = self.r[15];
        self.enter_exception(UND, 0x04, return_address);
        4
    }

    pub fn step<B: Arm60Bus>(&mut self, bus: &mut B) -> u32 {
        if self.halted {
            self.cycles = self.cycles.wrapping_add(1);
            return 1;
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
            return Err("ARM60 state contains an invalid processor mode".into());
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
        self.r[15] &= !3;
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
        fn word(&mut self, address: usize, value: u32) {
            self.data[address..address + 4].copy_from_slice(&value.to_le_bytes());
        }
    }
    impl Arm60Bus for TestBus {
        fn read8(&mut self, address: u32) -> u8 {
            self.data[address as usize % self.data.len()]
        }
        fn write8(&mut self, address: u32, value: u8) {
            let index = address as usize % self.data.len();
            self.data[index] = value;
        }
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
        let mut cpu = Arm60::default();
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
        let mut cpu = Arm60::default();
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
        let mut cpu = Arm60::default();
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
    fn state_round_trip_preserves_all_execution_banks() {
        let mut cpu = Arm60::default();
        cpu.r[0] = 0x1234_5678;
        cpu.r[15] = 0x400;
        cpu.cycles = 99;
        cpu.switch_mode(IRQ);
        cpu.r[13] = 0x1e00;
        cpu.set_current_spsr(0xa000_0013);
        let mut out = StateWriter::new(PlatformId::ThreeDo, 1);
        cpu.save(&mut out);
        let bytes = out.finish();
        let mut input = StateReader::new(&bytes, PlatformId::ThreeDo, 1).unwrap();
        let mut restored = Arm60::default();
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
