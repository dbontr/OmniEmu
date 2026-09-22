use crate::state::{StateReader, StateWriter};

pub trait AArch64Bus {
    fn read8(&mut self, address: u64) -> u8;
    fn write8(&mut self, address: u64, value: u8);
    fn exclusive_epoch(&self) -> u64 {
        0
    }

    fn read32(&mut self, address: u64) -> u32 {
        let mut bytes = [0; 4];
        for (index, byte) in bytes.iter_mut().enumerate() {
            *byte = self.read8(address.wrapping_add(index as u64));
        }
        u32::from_le_bytes(bytes)
    }
    fn write32(&mut self, address: u64, value: u32) {
        for (index, byte) in value.to_le_bytes().into_iter().enumerate() {
            self.write8(address.wrapping_add(index as u64), byte);
        }
    }
    fn read64(&mut self, address: u64) -> u64 {
        let mut bytes = [0; 8];
        for (index, byte) in bytes.iter_mut().enumerate() {
            *byte = self.read8(address.wrapping_add(index as u64));
        }
        u64::from_le_bytes(bytes)
    }
    fn write64(&mut self, address: u64, value: u64) {
        for (index, byte) in value.to_le_bytes().into_iter().enumerate() {
            self.write8(address.wrapping_add(index as u64), byte);
        }
    }
}

const N: u32 = 1 << 31;
const Z: u32 = 1 << 30;
const C: u32 = 1 << 29;
const V: u32 = 1 << 28;

#[derive(Clone, Copy)]
struct ExclusiveReservation {
    address: u64,
    size: u8,
    epoch: u64,
}

#[derive(Clone)]
pub struct AArch64Cpu {
    pub x: [u64; 31],
    pub sp: u64,
    pub pc: u64,
    pub nzcv: u32,
    pub cycles: u64,
    pub tpidr_el0: u64,
    pub tpidrro_el0: u64,
    counter_frequency: u64,
    counter_cycle_rate: u64,
    exclusive: Option<ExclusiveReservation>,
    halted: bool,
    invalid_instruction: bool,
    pub last_svc: Option<u16>,
    pending_svc: Option<u16>,
}

impl Default for AArch64Cpu {
    fn default() -> Self {
        Self::new()
    }
}

impl AArch64Cpu {
    pub fn new() -> Self {
        Self {
            x: [0; 31],
            sp: 0,
            pc: 0,
            nzcv: 0,
            cycles: 0,
            tpidr_el0: 0,
            tpidrro_el0: 0,
            counter_frequency: 1,
            counter_cycle_rate: 1,
            exclusive: None,
            halted: false,
            invalid_instruction: false,
            last_svc: None,
            pending_svc: None,
        }
    }

    pub fn reset_to(&mut self, pc: u64) {
        let counter_frequency = self.counter_frequency;
        let counter_cycle_rate = self.counter_cycle_rate;
        *self = Self::new();
        self.counter_frequency = counter_frequency;
        self.counter_cycle_rate = counter_cycle_rate;
        self.pc = pc;
    }
    pub fn halted(&self) -> bool {
        self.halted
    }
    pub fn invalid_instruction(&self) -> bool {
        self.invalid_instruction
    }
    pub fn pending_svc(&self) -> Option<u16> {
        self.pending_svc
    }
    pub fn take_pending_svc(&mut self) -> Option<u16> {
        self.pending_svc.take()
    }
    pub fn halt(&mut self) {
        self.halted = true;
    }
    pub fn configure_system_counter(
        &mut self,
        frequency: u64,
        cycle_rate: u64,
    ) -> Result<(), String> {
        if frequency == 0 || cycle_rate == 0 {
            return Err("AArch64 system-counter rates must be non-zero".into());
        }
        self.counter_frequency = frequency;
        self.counter_cycle_rate = cycle_rate;
        Ok(())
    }

    fn system_counter(&self) -> u64 {
        (u128::from(self.cycles) * u128::from(self.counter_frequency)
            / u128::from(self.counter_cycle_rate))
        .min(u128::from(u64::MAX)) as u64
    }

    fn xreg(&self, reg: usize) -> u64 {
        if reg == 31 {
            0
        } else {
            self.x[reg]
        }
    }
    fn set_xreg(&mut self, reg: usize, value: u64) {
        if reg != 31 {
            self.x[reg] = value;
        }
    }
    fn base_reg(&self, reg: usize) -> u64 {
        if reg == 31 {
            self.sp
        } else {
            self.x[reg]
        }
    }
    fn set_base_reg(&mut self, reg: usize, value: u64) {
        if reg == 31 {
            self.sp = value;
        } else {
            self.x[reg] = value;
        }
    }
    fn sign_extend(value: u64, bits: u32) -> i64 {
        ((value << (64 - bits)) as i64) >> (64 - bits)
    }

    fn set_nz(&mut self, value: u64, bits: u32) {
        self.nzcv &= !(N | Z);
        let mask = if bits == 32 {
            u32::MAX as u64
        } else {
            u64::MAX
        };
        let value = value & mask;
        if value == 0 {
            self.nzcv |= Z;
        }
        if value & (1u64 << (bits - 1)) != 0 {
            self.nzcv |= N;
        }
    }

    fn set_add_flags(&mut self, lhs: u64, rhs: u64, result: u64, bits: u32) {
        self.nzcv = 0;
        self.set_nz(result, bits);
        if bits == 32 {
            let lhs = lhs as u32;
            let rhs = rhs as u32;
            let result = result as u32;
            if lhs.overflowing_add(rhs).1 {
                self.nzcv |= C;
            }
            if ((!(lhs ^ rhs) & (lhs ^ result)) & 0x8000_0000) != 0 {
                self.nzcv |= V;
            }
        } else {
            if lhs.overflowing_add(rhs).1 {
                self.nzcv |= C;
            }
            if ((!(lhs ^ rhs) & (lhs ^ result)) & 0x8000_0000_0000_0000) != 0 {
                self.nzcv |= V;
            }
        }
    }

    fn set_sub_flags(&mut self, lhs: u64, rhs: u64, result: u64, bits: u32) {
        self.nzcv = 0;
        self.set_nz(result, bits);
        let mask = if bits == 32 {
            u32::MAX as u64
        } else {
            u64::MAX
        };
        let lhs = lhs & mask;
        let rhs = rhs & mask;
        let result = result & mask;
        if lhs >= rhs {
            self.nzcv |= C;
        }
        let sign = 1u64 << (bits - 1);
        if (((lhs ^ rhs) & (lhs ^ result)) & sign) != 0 {
            self.nzcv |= V;
        }
    }
    fn condition(&self, cond: u32) -> bool {
        let n = self.nzcv & N != 0;
        let z = self.nzcv & Z != 0;
        let c = self.nzcv & C != 0;
        let v = self.nzcv & V != 0;
        match cond & 0xf {
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

    fn shifted(value: u64, shift: u32, amount: u32, bits: u32) -> u64 {
        let amount = amount & (bits - 1);
        let mask = if bits == 32 {
            u32::MAX as u64
        } else {
            u64::MAX
        };
        let value = value & mask;
        match shift {
            0 => (value << amount) & mask,
            1 => value >> amount,
            2 if bits == 32 => ((value as u32 as i32) >> amount) as u32 as u64,
            2 => ((value as i64) >> amount) as u64,
            _ => value,
        }
    }

    fn width_mask(bits: u32) -> u64 {
        if bits == 32 {
            u64::from(u32::MAX)
        } else {
            u64::MAX
        }
    }

    fn conditional_compare(&mut self, instruction: u32, immediate: bool) {
        let bits = if instruction >> 31 != 0 { 64 } else { 32 };
        let mask = Self::width_mask(bits);
        let negative = instruction & (1 << 30) == 0;
        let condition = (instruction >> 12) & 0xf;
        if !self.condition(condition) {
            self.nzcv = (instruction & 0xf) << 28;
            return;
        }
        let rn = ((instruction >> 5) & 31) as usize;
        let lhs = self.xreg(rn) & mask;
        let rhs = if immediate {
            u64::from((instruction >> 16) & 31)
        } else {
            self.xreg(((instruction >> 16) & 31) as usize) & mask
        };
        let result = if negative {
            lhs.wrapping_add(rhs)
        } else {
            lhs.wrapping_sub(rhs)
        } & mask;
        if negative {
            self.set_add_flags(lhs, rhs, result, bits);
        } else {
            self.set_sub_flags(lhs, rhs, result, bits);
        }
    }

    fn test_bit_branch(&mut self, instruction: u32, current_pc: u64) {
        let bit = ((instruction >> 26) & 0x20) | ((instruction >> 19) & 0x1f);
        let value = self.xreg((instruction & 31) as usize);
        let nonzero = instruction & (1 << 24) != 0;
        let displacement = Self::sign_extend(u64::from((instruction >> 5) & 0x3fff), 14) << 2;
        if ((value >> bit) & 1 != 0) == nonzero {
            self.pc = current_pc.wrapping_add_signed(displacement);
        }
    }

    fn conditional_select(&mut self, instruction: u32) {
        let bits = if instruction >> 31 != 0 { 64 } else { 32 };
        let mask = Self::width_mask(bits);
        let op = (instruction >> 30) & 1;
        let op2 = (instruction >> 10) & 1;
        let rm = ((instruction >> 16) & 31) as usize;
        let rn = ((instruction >> 5) & 31) as usize;
        let rd = (instruction & 31) as usize;
        let value = if self.condition((instruction >> 12) & 0xf) {
            self.xreg(rn)
        } else {
            match (op, op2) {
                (0, 0) => self.xreg(rm),
                (0, 1) => self.xreg(rm).wrapping_add(1),
                (1, 0) => !self.xreg(rm),
                (1, 1) => (0u64).wrapping_sub(self.xreg(rm)),
                _ => unreachable!(),
            }
        } & mask;
        self.set_xreg(rd, value);
    }

    fn data_processing_two_source(&mut self, instruction: u32) {
        let bits = if instruction >> 31 != 0 { 64 } else { 32 };
        let mask = Self::width_mask(bits);
        let rm = ((instruction >> 16) & 31) as usize;
        let rn = ((instruction >> 5) & 31) as usize;
        let rd = (instruction & 31) as usize;
        let rhs = self.xreg(rm) & mask;
        let lhs = self.xreg(rn) & mask;
        let value = match (instruction >> 10) & 0x3f {
            2 => lhs.checked_div(rhs).unwrap_or(0),
            3 => {
                if rhs == 0 {
                    0
                } else if bits == 32 {
                    (lhs as u32 as i32).wrapping_div(rhs as u32 as i32) as u32 as u64
                } else {
                    (lhs as i64).wrapping_div(rhs as i64) as u64
                }
            }
            8 => (lhs << (rhs as u32 & (bits - 1))) & mask,
            9 => lhs >> (rhs as u32 & (bits - 1)),
            10 if bits == 32 => ((lhs as u32 as i32) >> (rhs as u32 & 31)) as u32 as u64,
            10 => ((lhs as i64) >> (rhs as u32 & 63)) as u64,
            11 => lhs.rotate_right(rhs as u32 & (bits - 1)) & mask,
            _ => {
                self.invalid_instruction = true;
                0
            }
        };
        if !self.invalid_instruction {
            self.set_xreg(rd, value & mask);
        }
    }

    fn data_processing_three_source(&mut self, instruction: u32) {
        let bits = if instruction >> 31 != 0 { 64 } else { 32 };
        let mask = Self::width_mask(bits);
        let rm = ((instruction >> 16) & 31) as usize;
        let ra = ((instruction >> 10) & 31) as usize;
        let rn = ((instruction >> 5) & 31) as usize;
        let rd = (instruction & 31) as usize;
        let product = (self.xreg(rn) & mask).wrapping_mul(self.xreg(rm) & mask) & mask;
        let accumulator = self.xreg(ra) & mask;
        let value = if instruction & (1 << 15) != 0 {
            accumulator.wrapping_sub(product)
        } else {
            accumulator.wrapping_add(product)
        } & mask;
        self.set_xreg(rd, value);
    }

    fn bitfield_shift_alias(&mut self, instruction: u32) {
        let bits = if instruction >> 31 != 0 { 64 } else { 32 };
        let mask = Self::width_mask(bits);
        let opc = (instruction >> 29) & 3;
        let n = (instruction >> 22) & 1;
        let immr = (instruction >> 16) & 0x3f;
        let imms = (instruction >> 10) & 0x3f;
        let rn = ((instruction >> 5) & 31) as usize;
        let rd = (instruction & 31) as usize;
        if n != u32::from(bits == 64) || immr >= bits || imms >= bits || !matches!(opc, 0 | 2) {
            self.invalid_instruction = true;
            return;
        }
        let source = self.xreg(rn) & mask;
        let value = if imms == bits - 1 {
            if opc == 0 {
                if bits == 32 {
                    ((source as u32 as i32) >> immr) as u32 as u64
                } else {
                    ((source as i64) >> immr) as u64
                }
            } else {
                source >> immr
            }
        } else if opc == 2 && imms.wrapping_add(1) == immr {
            (source << (bits - immr)) & mask
        } else {
            self.invalid_instruction = true;
            0
        };
        if !self.invalid_instruction {
            self.set_xreg(rd, value & mask);
        }
    }

    fn unscaled_load_store<B: AArch64Bus>(&mut self, bus: &mut B, instruction: u32) {
        let size = 1u64 << ((instruction >> 30) & 3);
        let load = instruction & (1 << 22) != 0;
        let mode = (instruction >> 10) & 3;
        if mode == 2 {
            self.invalid_instruction = true;
            return;
        }
        let offset = Self::sign_extend(u64::from((instruction >> 12) & 0x1ff), 9);
        let rn = ((instruction >> 5) & 31) as usize;
        let rt = (instruction & 31) as usize;
        let base = self.base_reg(rn);
        let address = if mode == 3 {
            base.wrapping_add_signed(offset)
        } else {
            base
        };
        if load {
            let value = match size {
                1 => u64::from(bus.read8(address)),
                2 => u64::from(u16::from_le_bytes([
                    bus.read8(address),
                    bus.read8(address + 1),
                ])),
                4 => u64::from(bus.read32(address)),
                8 => bus.read64(address),
                _ => unreachable!(),
            };
            self.set_xreg(rt, value);
        } else {
            let value = self.xreg(rt);
            match size {
                1 => bus.write8(address, value as u8),
                2 => {
                    for (index, byte) in (value as u16).to_le_bytes().into_iter().enumerate() {
                        bus.write8(address + index as u64, byte);
                    }
                }
                4 => bus.write32(address, value as u32),
                8 => bus.write64(address, value),
                _ => unreachable!(),
            }
        }
        if matches!(mode, 1 | 3) {
            self.set_base_reg(rn, base.wrapping_add_signed(offset));
        }
    }

    fn system_register(&mut self, instruction: u32) -> bool {
        let rt = (instruction & 31) as usize;
        match instruction & 0xffff_ffe0 {
            0xd53b_e020 => self.set_xreg(rt, self.system_counter()),
            0xd53b_e000 => self.set_xreg(rt, self.counter_frequency),
            0xd53b_d060 => self.set_xreg(rt, self.tpidrro_el0),
            0xd53b_d040 => self.set_xreg(rt, self.tpidr_el0),
            0xd51b_d040 => self.tpidr_el0 = self.xreg(rt),
            _ => return false,
        }
        true
    }

    fn load_exclusive<B: AArch64Bus>(&mut self, bus: &mut B, instruction: u32, bits: u32) {
        let rn = ((instruction >> 5) & 31) as usize;
        let rt = (instruction & 31) as usize;
        let address = self.base_reg(rn);
        let value = if bits == 64 {
            bus.read64(address)
        } else {
            u64::from(bus.read32(address))
        };
        self.set_xreg(rt, value);
        self.exclusive = Some(ExclusiveReservation {
            address,
            size: (bits / 8) as u8,
            epoch: bus.exclusive_epoch(),
        });
    }

    fn store_exclusive<B: AArch64Bus>(&mut self, bus: &mut B, instruction: u32, bits: u32) {
        let rs = ((instruction >> 16) & 31) as usize;
        let rn = ((instruction >> 5) & 31) as usize;
        let rt = (instruction & 31) as usize;
        let address = self.base_reg(rn);
        let size = (bits / 8) as u8;
        let success = self.exclusive.is_some_and(|reservation| {
            reservation.address == address
                && reservation.size == size
                && reservation.epoch == bus.exclusive_epoch()
        });
        self.exclusive = None;
        if success {
            if bits == 64 {
                bus.write64(address, self.xreg(rt));
            } else {
                bus.write32(address, self.xreg(rt) as u32);
            }
        }
        self.set_xreg(rs, u64::from(!success));
    }

    pub fn step<B: AArch64Bus>(&mut self, bus: &mut B) {
        if self.halted {
            self.cycles = self.cycles.wrapping_add(1);
            return;
        }
        self.invalid_instruction = false;
        let instruction = bus.read32(self.pc);
        let current_pc = self.pc;
        self.pc = self.pc.wrapping_add(4);

        if instruction == 0xd503_201f {
            // NOP
        } else if self.system_register(instruction) {
        } else if matches!(
            instruction,
            0xd503_3fbf
                | 0xd503_3bbf
                | 0xd503_39bf
                | 0xd503_3abf
                | 0xd503_3f9f
                | 0xd503_3b9f
                | 0xd503_3fdf
        ) {
            // The interpreter is strongly ordered, so architectural barriers require no host action.
        } else if instruction == 0xd503_3f5f {
            self.exclusive = None;
        } else if instruction & 0xffff_fc00 == 0x885f_7c00
            || instruction & 0xffff_fc00 == 0x885f_fc00
        {
            self.load_exclusive(bus, instruction, 32);
        } else if instruction & 0xffff_fc00 == 0xc85f_7c00
            || instruction & 0xffff_fc00 == 0xc85f_fc00
        {
            self.load_exclusive(bus, instruction, 64);
        } else if instruction & 0xffe0_fc00 == 0x8800_7c00
            || instruction & 0xffe0_fc00 == 0x8800_fc00
        {
            self.store_exclusive(bus, instruction, 32);
        } else if instruction & 0xffe0_fc00 == 0xc800_7c00
            || instruction & 0xffe0_fc00 == 0xc800_fc00
        {
            self.store_exclusive(bus, instruction, 64);
        } else if instruction & 0xffff_fc00 == 0x88df_fc00 {
            let rn = ((instruction >> 5) & 31) as usize;
            let rt = (instruction & 31) as usize;
            let value = u64::from(bus.read32(self.base_reg(rn)));
            self.set_xreg(rt, value);
        } else if instruction & 0xffff_fc00 == 0xc8df_fc00 {
            let rn = ((instruction >> 5) & 31) as usize;
            let rt = (instruction & 31) as usize;
            let value = bus.read64(self.base_reg(rn));
            self.set_xreg(rt, value);
        } else if instruction & 0xffff_fc00 == 0x889f_fc00 {
            let rn = ((instruction >> 5) & 31) as usize;
            let rt = (instruction & 31) as usize;
            bus.write32(self.base_reg(rn), self.xreg(rt) as u32);
        } else if instruction & 0xffff_fc00 == 0xc89f_fc00 {
            let rn = ((instruction >> 5) & 31) as usize;
            let rt = (instruction & 31) as usize;
            bus.write64(self.base_reg(rn), self.xreg(rt));
        } else if instruction & 0xffe0_001f == 0xd440_0000 {
            self.halted = true;
        } else if instruction & 0xffe0_001f == 0xd400_0001 {
            let svc = ((instruction >> 5) & 0xffff) as u16;
            self.last_svc = Some(svc);
            self.pending_svc = Some(svc);
        } else if instruction & 0xfc00_0000 == 0x1400_0000 {
            let displacement = Self::sign_extend(u64::from(instruction & 0x03ff_ffff), 26) << 2;
            self.pc = current_pc.wrapping_add_signed(displacement);
        } else if instruction & 0xfc00_0000 == 0x9400_0000 {
            let displacement = Self::sign_extend(u64::from(instruction & 0x03ff_ffff), 26) << 2;
            self.x[30] = self.pc;
            self.pc = current_pc.wrapping_add_signed(displacement);
        } else if instruction & 0xffff_fc1f == 0xd61f_0000 {
            let rn = ((instruction >> 5) & 31) as usize;
            self.pc = self.xreg(rn);
        } else if instruction & 0xffff_fc1f == 0xd63f_0000 {
            let rn = ((instruction >> 5) & 31) as usize;
            let target = self.xreg(rn);
            self.x[30] = self.pc;
            self.pc = target;
        } else if instruction & 0xffff_fc1f == 0xd65f_0000 {
            let rn = ((instruction >> 5) & 31) as usize;
            self.pc = self.xreg(rn);
        } else if instruction & 0x7e00_0000 == 0x3400_0000 {
            let sf = instruction >> 31 != 0;
            let nonzero = instruction & (1 << 24) != 0;
            let rt = (instruction & 31) as usize;
            let value = if sf {
                self.xreg(rt)
            } else {
                self.xreg(rt) & u64::from(u32::MAX)
            };
            let displacement = Self::sign_extend(u64::from((instruction >> 5) & 0x7ffff), 19) << 2;
            if (value != 0) == nonzero {
                self.pc = current_pc.wrapping_add_signed(displacement);
            }
        } else if instruction & 0x7e00_0000 == 0x3600_0000 {
            self.test_bit_branch(instruction, current_pc);
        } else if instruction & 0xff00_0010 == 0x5400_0000 {
            let displacement = Self::sign_extend(u64::from((instruction >> 5) & 0x7ffff), 19) << 2;
            if self.condition(instruction & 0xf) {
                self.pc = current_pc.wrapping_add_signed(displacement);
            }
        } else if instruction & 0x1fe0_0800 == 0x1a40_0800 {
            self.conditional_compare(instruction, true);
        } else if instruction & 0x1fe0_0800 == 0x1a40_0000 {
            self.conditional_compare(instruction, false);
        } else if instruction & 0x1f80_0000 == 0x1280_0000 {
            let sf = instruction >> 31 != 0;
            let opc = (instruction >> 29) & 3;
            let hw = (instruction >> 21) & 3;
            if !sf && hw >= 2 {
                self.invalid_instruction = true;
            } else {
                let rd = (instruction & 31) as usize;
                let shift = hw * 16;
                let immediate = u64::from((instruction >> 5) & 0xffff) << shift;
                let width_mask = if sf { u64::MAX } else { u64::from(u32::MAX) };
                let value = match opc {
                    0 => (!immediate) & width_mask,
                    2 => immediate,
                    3 => (self.xreg(rd) & !(0xffffu64 << shift)) | immediate,
                    _ => {
                        self.invalid_instruction = true;
                        0
                    }
                } & width_mask;
                if !self.invalid_instruction {
                    self.set_xreg(rd, value);
                }
            }
        } else if instruction & 0x1f80_0000 == 0x1300_0000 {
            self.bitfield_shift_alias(instruction);
        } else if instruction & 0x1f00_0000 == 0x1000_0000 {
            let page = instruction >> 31 != 0;
            let rd = (instruction & 31) as usize;
            let imm =
                (u64::from((instruction >> 5) & 0x7ffff) << 2) | u64::from((instruction >> 29) & 3);
            let displacement = Self::sign_extend(imm, 21);
            let value = if page {
                (current_pc & !0xfff).wrapping_add_signed(displacement << 12)
            } else {
                current_pc.wrapping_add_signed(displacement)
            };
            self.set_xreg(rd, value);
        } else if instruction & 0x1f00_0000 == 0x1100_0000 {
            let sf = instruction >> 31 != 0;
            let subtract = instruction & (1 << 30) != 0;
            let set_flags = instruction & (1 << 29) != 0;
            let shift = if instruction & (1 << 22) != 0 { 12 } else { 0 };
            let immediate = u64::from((instruction >> 10) & 0xfff) << shift;
            let rn = ((instruction >> 5) & 31) as usize;
            let rd = (instruction & 31) as usize;
            let lhs = self.base_reg(rn);
            let width_mask = if sf { u64::MAX } else { u64::from(u32::MAX) };
            let lhs = lhs & width_mask;
            let rhs = immediate & width_mask;
            let result = if subtract {
                lhs.wrapping_sub(rhs)
            } else {
                lhs.wrapping_add(rhs)
            } & width_mask;
            if set_flags {
                if subtract {
                    self.set_sub_flags(lhs, rhs, result, if sf { 64 } else { 32 });
                } else {
                    self.set_add_flags(lhs, rhs, result, if sf { 64 } else { 32 });
                }
                self.set_xreg(rd, result);
            } else {
                self.set_base_reg(rd, result);
            }
        } else if instruction & 0x1f20_0000 == 0x0b00_0000 {
            let sf = instruction >> 31 != 0;
            let subtract = instruction & (1 << 30) != 0;
            let set_flags = instruction & (1 << 29) != 0;
            let shift_type = (instruction >> 22) & 3;
            let rm = ((instruction >> 16) & 31) as usize;
            let amount = (instruction >> 10) & 0x3f;
            let rn = ((instruction >> 5) & 31) as usize;
            let rd = (instruction & 31) as usize;
            let bits = if sf { 64 } else { 32 };
            if amount >= bits || shift_type == 3 {
                self.invalid_instruction = true;
            } else {
                let mask = if sf { u64::MAX } else { u64::from(u32::MAX) };
                let lhs = self.base_reg(rn) & mask;
                let rhs = Self::shifted(self.xreg(rm), shift_type, amount, bits) & mask;
                let result = if subtract {
                    lhs.wrapping_sub(rhs)
                } else {
                    lhs.wrapping_add(rhs)
                } & mask;
                if set_flags {
                    if subtract {
                        self.set_sub_flags(lhs, rhs, result, bits);
                    } else {
                        self.set_add_flags(lhs, rhs, result, bits);
                    }
                    self.set_xreg(rd, result);
                } else {
                    self.set_base_reg(rd, result);
                }
            }
        } else if instruction & 0x1f20_0000 == 0x0a00_0000 {
            let sf = instruction >> 31 != 0;
            let opc = (instruction >> 29) & 3;
            let shift_type = (instruction >> 22) & 3;
            let invert = instruction & (1 << 21) != 0;
            let rm = ((instruction >> 16) & 31) as usize;
            let amount = (instruction >> 10) & 0x3f;
            let rn = ((instruction >> 5) & 31) as usize;
            let rd = (instruction & 31) as usize;
            let bits = if sf { 64 } else { 32 };
            if amount >= bits || shift_type == 3 {
                self.invalid_instruction = true;
            } else {
                let mask = if sf { u64::MAX } else { u64::from(u32::MAX) };
                let lhs = self.xreg(rn) & mask;
                let mut rhs = Self::shifted(self.xreg(rm), shift_type, amount, bits) & mask;
                if invert {
                    rhs = !rhs & mask;
                }
                let result = match opc {
                    0 => lhs & rhs,
                    1 => lhs | rhs,
                    2 => lhs ^ rhs,
                    _ => lhs & rhs,
                };
                self.set_xreg(rd, result);
                if opc == 3 {
                    self.nzcv &= !(N | Z | C | V);
                    self.set_nz(result, bits);
                }
            }
        } else if instruction & 0x1fe0_0800 == 0x1a80_0000 {
            self.conditional_select(instruction);
        } else if instruction & 0x1fe0_0000 == 0x1ac0_0000 {
            self.data_processing_two_source(instruction);
        } else if instruction & 0x1fe0_0000 == 0x1b00_0000 {
            self.data_processing_three_source(instruction);
        } else if instruction & 0x3b20_0000 == 0x3800_0000 {
            self.unscaled_load_store(bus, instruction);
        } else if instruction & 0xffc0_0000 == 0xf900_0000
            || instruction & 0xffc0_0000 == 0xf940_0000
            || instruction & 0xffc0_0000 == 0xb900_0000
            || instruction & 0xffc0_0000 == 0xb940_0000
            || instruction & 0xffc0_0000 == 0x3900_0000
            || instruction & 0xffc0_0000 == 0x3940_0000
            || instruction & 0xffc0_0000 == 0x7900_0000
            || instruction & 0xffc0_0000 == 0x7940_0000
        {
            let load = instruction & (1 << 22) != 0;
            let size_code = (instruction >> 30) & 3;
            let bytes = 1u64 << size_code;
            let imm = u64::from((instruction >> 10) & 0xfff) * bytes;
            let rn = ((instruction >> 5) & 31) as usize;
            let rt = (instruction & 31) as usize;
            let address = self.base_reg(rn).wrapping_add(imm);
            if load {
                let value = match bytes {
                    1 => u64::from(bus.read8(address)),
                    2 => u64::from(u16::from_le_bytes([
                        bus.read8(address),
                        bus.read8(address + 1),
                    ])),
                    4 => u64::from(bus.read32(address)),
                    8 => bus.read64(address),
                    _ => 0,
                };
                self.set_xreg(rt, value);
            } else {
                let value = self.xreg(rt);
                match bytes {
                    1 => bus.write8(address, value as u8),
                    2 => {
                        for (offset, byte) in (value as u16).to_le_bytes().into_iter().enumerate() {
                            bus.write8(address + offset as u64, byte);
                        }
                    }
                    4 => bus.write32(address, value as u32),
                    8 => bus.write64(address, value),
                    _ => {}
                }
            }
        } else if instruction & 0x3b00_0000 == 0x1800_0000 {
            let opc = instruction >> 30;
            let rt = (instruction & 31) as usize;
            let displacement = Self::sign_extend(u64::from((instruction >> 5) & 0x7ffff), 19) << 2;
            let address = current_pc.wrapping_add_signed(displacement);
            match opc {
                0 => self.set_xreg(rt, u64::from(bus.read32(address))),
                1 => self.set_xreg(rt, bus.read64(address)),
                2 => self.set_xreg(rt, (bus.read32(address) as i32 as i64) as u64),
                _ => self.invalid_instruction = true,
            }
        } else if instruction & 0x3a00_0000 == 0x2800_0000 && instruction & 0x0400_0000 == 0 {
            let opc = instruction >> 30;
            let load = instruction & (1 << 22) != 0;
            let mode = (instruction >> 23) & 3;
            let scale = match opc {
                0 => 4u64,
                2 => 8u64,
                _ => {
                    self.invalid_instruction = true;
                    1
                }
            };
            if !self.invalid_instruction {
                let offset =
                    Self::sign_extend(u64::from((instruction >> 15) & 0x7f), 7) * scale as i64;
                let rt2 = ((instruction >> 10) & 31) as usize;
                let rn = ((instruction >> 5) & 31) as usize;
                let rt = (instruction & 31) as usize;
                let base = self.base_reg(rn);
                let address = if mode == 3 {
                    base.wrapping_add_signed(offset)
                } else {
                    base
                };
                let address = if mode == 2 {
                    base.wrapping_add_signed(offset)
                } else {
                    address
                };
                if load {
                    if scale == 8 {
                        self.set_xreg(rt, bus.read64(address));
                        self.set_xreg(rt2, bus.read64(address + 8));
                    } else {
                        self.set_xreg(rt, u64::from(bus.read32(address)));
                        self.set_xreg(rt2, u64::from(bus.read32(address + 4)));
                    }
                } else if scale == 8 {
                    bus.write64(address, self.xreg(rt));
                    bus.write64(address + 8, self.xreg(rt2));
                } else {
                    bus.write32(address, self.xreg(rt) as u32);
                    bus.write32(address + 4, self.xreg(rt2) as u32);
                }
                if mode == 1 || mode == 3 {
                    self.set_base_reg(rn, base.wrapping_add_signed(offset));
                }
            }
        } else {
            self.invalid_instruction = true;
        }
        self.cycles = self.cycles.wrapping_add(1);
    }

    pub fn save(&self, out: &mut StateWriter) {
        for reg in self.x {
            out.u64(reg);
        }
        out.u64(self.sp);
        out.u64(self.pc);
        out.u32(self.nzcv);
        out.u64(self.cycles);
        out.u64(self.tpidr_el0);
        out.u64(self.tpidrro_el0);
        out.u64(self.counter_frequency);
        out.u64(self.counter_cycle_rate);
        match self.exclusive {
            Some(reservation) => {
                out.u8(1);
                out.u64(reservation.address);
                out.u8(reservation.size);
                out.u64(reservation.epoch);
            }
            None => out.u8(0),
        }
        out.u8(self.halted as u8);
        out.u8(self.invalid_instruction as u8);
        for svc in [self.last_svc, self.pending_svc] {
            match svc {
                Some(value) => {
                    out.u8(1);
                    out.u16(value);
                }
                None => out.u8(0),
            }
        }
    }

    pub fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        for reg in &mut self.x {
            *reg = input.u64()?;
        }
        self.sp = input.u64()?;
        self.pc = input.u64()?;
        self.nzcv = input.u32()?;
        self.cycles = input.u64()?;
        self.tpidr_el0 = input.u64()?;
        self.tpidrro_el0 = input.u64()?;
        self.counter_frequency = input.u64()?;
        self.counter_cycle_rate = input.u64()?;
        if self.counter_frequency == 0 || self.counter_cycle_rate == 0 {
            return Err("AArch64 state has an invalid system-counter rate".into());
        }
        self.exclusive = if input.u8()? != 0 {
            let address = input.u64()?;
            let size = input.u8()?;
            if !matches!(size, 4 | 8) {
                return Err("AArch64 state has an invalid exclusive reservation size".into());
            }
            Some(ExclusiveReservation {
                address,
                size,
                epoch: input.u64()?,
            })
        } else {
            None
        };
        self.halted = input.u8()? != 0;
        self.invalid_instruction = input.u8()? != 0;
        self.last_svc = if input.u8()? != 0 {
            Some(input.u16()?)
        } else {
            None
        };
        self.pending_svc = if input.u8()? != 0 {
            Some(input.u16()?)
        } else {
            None
        };
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::platform::PlatformId;

    struct Bus {
        bytes: Vec<u8>,
    }
    impl AArch64Bus for Bus {
        fn read8(&mut self, address: u64) -> u8 {
            self.bytes[address as usize % self.bytes.len()]
        }
        fn write8(&mut self, address: u64, value: u8) {
            let index = address as usize % self.bytes.len();
            self.bytes[index] = value;
        }
    }

    struct EpochBus {
        bytes: Vec<u8>,
        epoch: u64,
    }

    impl AArch64Bus for EpochBus {
        fn read8(&mut self, address: u64) -> u8 {
            self.bytes[address as usize % self.bytes.len()]
        }
        fn write8(&mut self, address: u64, value: u8) {
            let index = address as usize % self.bytes.len();
            self.bytes[index] = value;
            self.epoch = self.epoch.wrapping_add(1);
        }
        fn exclusive_epoch(&self) -> u64 {
            self.epoch
        }
    }

    fn movz_x(rd: u32, imm: u16) -> u32 {
        0xd280_0000 | (u32::from(imm) << 5) | rd
    }
    fn str_x(rt: u32, rn: u32) -> u32 {
        0xf900_0000 | (rn << 5) | rt
    }
    fn ldr_x(rt: u32, rn: u32) -> u32 {
        0xf940_0000 | (rn << 5) | rt
    }
    fn svc(id: u16) -> u32 {
        0xd400_0001 | (u32::from(id) << 5)
    }

    #[test]
    fn executes_little_endian_load_store_and_exit_svc() {
        let program = [
            movz_x(0, 0x2000),
            movz_x(1, 0x1234),
            str_x(1, 0),
            ldr_x(2, 0),
            svc(7),
        ];
        let mut bus = Bus {
            bytes: vec![0; 0x4000],
        };
        for (index, instruction) in program.iter().enumerate() {
            bus.bytes[index * 4..index * 4 + 4].copy_from_slice(&instruction.to_le_bytes());
        }
        let mut cpu = AArch64Cpu::new();
        while cpu.pending_svc().is_none() && !cpu.invalid_instruction() {
            cpu.step(&mut bus);
        }
        assert_eq!(bus.read64(0x2000), 0x1234);
        assert_eq!(cpu.x[2], 0x1234);
        assert_eq!(cpu.last_svc, Some(7));
        assert!(!cpu.invalid_instruction());
    }

    #[test]
    fn pair_pre_post_index_preserves_stack_and_registers() {
        let mut bus = Bus {
            bytes: vec![0; 0x4000],
        };
        let program = [0xa9bf_7bfd, 0xd280_2460, 0xa8c1_7bfd, svc(7)];
        for (index, instruction) in program.iter().enumerate() {
            bus.bytes[index * 4..index * 4 + 4].copy_from_slice(&instruction.to_le_bytes());
        }
        let mut cpu = AArch64Cpu::new();
        cpu.sp = 0x3000;
        cpu.x[29] = 0x1122_3344_5566_7788;
        cpu.x[30] = 0x8877_6655_4433_2211;
        while cpu.pending_svc().is_none() && !cpu.invalid_instruction() {
            cpu.step(&mut bus);
        }
        assert_eq!(cpu.sp, 0x3000);
        assert_eq!(cpu.x[29], 0x1122_3344_5566_7788);
        assert_eq!(cpu.x[30], 0x8877_6655_4433_2211);
        assert!(!cpu.invalid_instruction());
    }

    #[test]
    fn conditional_compare_matches_compiler_flag_semantics() {
        let program: [u32; 3] = [0xba41_1824, 0xfa43_0040, svc(7)];
        let mut bus = Bus {
            bytes: vec![0; 0x100],
        };
        for (index, instruction) in program.iter().enumerate() {
            bus.bytes[index * 4..index * 4 + 4].copy_from_slice(&instruction.to_le_bytes());
        }
        let mut cpu = AArch64Cpu::new();
        cpu.x[1] = u64::MAX;
        cpu.x[2] = 5;
        cpu.x[3] = 7;
        cpu.step(&mut bus);
        assert_eq!(cpu.nzcv, Z | C);
        cpu.step(&mut bus);
        assert_eq!(cpu.nzcv, N);
        cpu.step(&mut bus);
        assert_eq!(cpu.pending_svc(), Some(7));
        assert!(!cpu.halted());
        assert!(!cpu.invalid_instruction());
    }

    #[test]
    fn test_bit_branches_and_bitfield_shift_aliases_execute() {
        let program: [u32; 6] = [
            0x3638_0044,
            0xd503_201f,
            0xb740_0045,
            0xd503_201f,
            0xd37b_e8e6,
            0xd34b_fd28,
        ];
        let mut bus = Bus {
            bytes: vec![0; 0x100],
        };
        for (index, instruction) in program.iter().enumerate() {
            bus.bytes[index * 4..index * 4 + 4].copy_from_slice(&instruction.to_le_bytes());
        }
        let mut cpu = AArch64Cpu::new();
        cpu.x[4] = 0;
        cpu.x[5] = 1u64 << 40;
        cpu.x[7] = 3;
        cpu.x[9] = 0x8000;
        cpu.step(&mut bus);
        assert_eq!(cpu.pc, 8);
        cpu.step(&mut bus);
        assert_eq!(cpu.pc, 16);
        cpu.step(&mut bus);
        cpu.step(&mut bus);
        assert_eq!(cpu.x[6], 96);
        assert_eq!(cpu.x[8], 0x10);
        assert!(!cpu.invalid_instruction());
    }

    #[test]
    fn arithmetic_shift_alias_sign_extends() {
        let program: [u32; 2] = [0x934d_fd6a, svc(7)];
        let mut bus = Bus {
            bytes: vec![0; 0x100],
        };
        for (index, instruction) in program.iter().enumerate() {
            bus.bytes[index * 4..index * 4 + 4].copy_from_slice(&instruction.to_le_bytes());
        }
        let mut cpu = AArch64Cpu::new();
        cpu.x[11] = (-8192i64) as u64;
        cpu.step(&mut bus);
        assert_eq!(cpu.x[10], u64::MAX);
        assert!(!cpu.invalid_instruction());
    }

    #[test]
    fn conditional_select_variants_execute() {
        let program: [u32; 5] = [0x9a8e_01ac, 0x9a91_160f, 0xda89_b107, 0xda8c_a56a, svc(7)];
        let mut bus = Bus {
            bytes: vec![0; 0x100],
        };
        for (index, instruction) in program.iter().enumerate() {
            bus.bytes[index * 4..index * 4 + 4].copy_from_slice(&instruction.to_le_bytes());
        }
        let mut cpu = AArch64Cpu::new();
        cpu.nzcv = Z;
        cpu.x[13] = 0x1111;
        cpu.x[14] = 0x2222;
        cpu.x[16] = 0x3333;
        cpu.x[17] = 0x4444;
        cpu.x[8] = 0x5555;
        cpu.x[9] = 0x00ff;
        cpu.x[11] = 0x7777;
        cpu.x[12] = 0x8888;
        while cpu.pending_svc().is_none() && !cpu.invalid_instruction() {
            cpu.step(&mut bus);
        }
        assert_eq!(cpu.x[12], 0x1111);
        assert_eq!(cpu.x[15], 0x4445);
        assert_eq!(cpu.x[7], !0x00ffu64);
        assert_eq!(cpu.x[10], 0x7777);
        assert!(!cpu.invalid_instruction());
    }

    #[test]
    fn multiply_divide_and_variable_shifts_execute() {
        let program: [u32; 8] = [
            0x9b03_1041,
            0x1b07_a0c5,
            0x9ad7_0ed5,
            0x9ada_0b38,
            0x9ac3_2041,
            0x9ac6_24a4,
            0x9ac9_2907,
            svc(7),
        ];
        let mut bus = Bus {
            bytes: vec![0; 0x100],
        };
        for (index, instruction) in program.iter().enumerate() {
            bus.bytes[index * 4..index * 4 + 4].copy_from_slice(&instruction.to_le_bytes());
        }
        let mut cpu = AArch64Cpu::new();
        cpu.x[2] = 6;
        cpu.x[3] = 7;
        cpu.x[4] = 5;
        cpu.step(&mut bus);
        assert_eq!(cpu.x[1], 47);

        cpu.x[6] = 9;
        cpu.x[7] = 3;
        cpu.x[8] = 100;
        cpu.step(&mut bus);
        assert_eq!(cpu.x[5] as u32, 73);

        cpu.x[22] = (-84i64) as u64;
        cpu.x[23] = 7;
        cpu.step(&mut bus);
        assert_eq!(cpu.x[21], (-12i64) as u64);

        cpu.x[25] = 100;
        cpu.x[26] = 9;
        cpu.step(&mut bus);
        assert_eq!(cpu.x[24], 11);

        cpu.x[2] = 3;
        cpu.x[3] = 4;
        cpu.step(&mut bus);
        assert_eq!(cpu.x[1], 48);

        cpu.x[5] = 0x100;
        cpu.x[6] = 3;
        cpu.step(&mut bus);
        assert_eq!(cpu.x[4], 0x20);

        cpu.x[8] = (-256i64) as u64;
        cpu.x[9] = 7;
        cpu.step(&mut bus);
        assert_eq!(cpu.x[7], (-2i64) as u64);

        cpu.step(&mut bus);
        assert_eq!(cpu.pending_svc(), Some(7));
        assert!(!cpu.halted());
        assert!(!cpu.invalid_instruction());
    }

    #[test]
    fn scalar_pre_post_index_memory_preserves_base_updates() {
        let program: [u32; 3] = [0xf81f_0fe0, 0xf841_07e1, svc(7)];
        let mut bus = Bus {
            bytes: vec![0; 0x4000],
        };
        for (index, instruction) in program.iter().enumerate() {
            bus.bytes[index * 4..index * 4 + 4].copy_from_slice(&instruction.to_le_bytes());
        }
        let mut cpu = AArch64Cpu::new();
        cpu.sp = 0x3000;
        cpu.x[0] = 0x1122_3344_5566_7788;
        cpu.step(&mut bus);
        assert_eq!(cpu.sp, 0x2ff0);
        assert_eq!(bus.read64(0x2ff0), 0x1122_3344_5566_7788);
        cpu.step(&mut bus);
        assert_eq!(cpu.x[1], 0x1122_3344_5566_7788);
        assert_eq!(cpu.sp, 0x3000);
        cpu.step(&mut bus);
        assert_eq!(cpu.pending_svc(), Some(7));
        assert!(!cpu.halted());
        assert!(!cpu.invalid_instruction());
    }

    #[test]
    fn system_registers_counter_tls_and_barriers_execute() {
        let program = [
            0xd51b_d044,
            0xd53b_d045,
            0xd53b_d066,
            0xd53b_e027,
            0xd53b_e008,
            0xd503_3bbf,
            0xd503_3b9f,
            0xd503_3fdf,
            svc(7),
        ];
        let mut bus = Bus {
            bytes: vec![0; 0x100],
        };
        for (index, instruction) in program.iter().enumerate() {
            bus.bytes[index * 4..index * 4 + 4].copy_from_slice(&instruction.to_le_bytes());
        }
        let mut cpu = AArch64Cpu::new();
        cpu.configure_system_counter(16, 5).unwrap();
        cpu.reset_to(0);
        cpu.x[4] = 0x1122_3344_5566_7788;
        cpu.tpidrro_el0 = 0x8877_6655_4433_2211;
        while cpu.pending_svc().is_none() && !cpu.invalid_instruction() {
            cpu.step(&mut bus);
        }
        assert_eq!(cpu.x[5], 0x1122_3344_5566_7788);
        assert_eq!(cpu.x[6], 0x8877_6655_4433_2211);
        assert_eq!(cpu.x[7], 9);
        assert_eq!(cpu.x[8], 16);
        assert_eq!(cpu.pending_svc(), Some(7));
        assert!(!cpu.invalid_instruction());
    }

    #[test]
    fn exclusive_and_acquire_release_memory_operations_execute() {
        let program = [
            0x885f_fc20,
            0x8802_fc23,
            0x885f_fc20,
            0x8802_fc23,
            0x88df_fc24,
            0x889f_fc25,
            0xc85f_7ce6,
            0xc808_7ce9,
            0xd503_3f5f,
            svc(7),
        ];
        let mut bus = EpochBus {
            bytes: vec![0; 0x400],
            epoch: 0,
        };
        for (index, instruction) in program.iter().enumerate() {
            bus.bytes[index * 4..index * 4 + 4].copy_from_slice(&instruction.to_le_bytes());
        }
        bus.bytes[0x100..0x104].copy_from_slice(&5u32.to_le_bytes());
        bus.bytes[0x180..0x188].copy_from_slice(&0x1020_3040_5060_7080u64.to_le_bytes());
        let mut cpu = AArch64Cpu::new();
        cpu.x[1] = 0x100;
        cpu.x[3] = 9;
        cpu.x[5] = 0x1234_5678;
        cpu.x[7] = 0x180;
        cpu.x[9] = 0xaabb_ccdd_eeff_0011;

        cpu.step(&mut bus);
        assert_eq!(cpu.x[0], 5);
        bus.write8(0x200, 0xaa);
        cpu.step(&mut bus);
        assert_eq!(cpu.x[2], 1);
        assert_eq!(bus.read32(0x100), 5);

        cpu.step(&mut bus);
        cpu.step(&mut bus);
        assert_eq!(cpu.x[2], 0);
        assert_eq!(bus.read32(0x100), 9);
        cpu.step(&mut bus);
        assert_eq!(cpu.x[4], 9);
        cpu.step(&mut bus);
        assert_eq!(bus.read32(0x100), 0x1234_5678);

        cpu.step(&mut bus);
        assert_eq!(cpu.x[6], 0x1020_3040_5060_7080);
        cpu.step(&mut bus);
        assert_eq!(cpu.x[8], 0);
        assert_eq!(bus.read64(0x180), 0xaabb_ccdd_eeff_0011);
        cpu.step(&mut bus);
        cpu.step(&mut bus);
        assert_eq!(cpu.pending_svc(), Some(7));
        assert!(!cpu.invalid_instruction());
    }

    #[test]
    fn state_round_trip_preserves_aarch64_registers() {
        let mut cpu = AArch64Cpu::new();
        cpu.x[0] = 0x1122_3344_5566_7788;
        cpu.sp = 0x0071_0010_0000;
        cpu.pc = 0x0071_0000_0080;
        cpu.nzcv = N | C;
        cpu.tpidr_el0 = 0x1000_2000;
        cpu.tpidrro_el0 = 0x3000_4000;
        cpu.configure_system_counter(19_200_000, 6_000_000).unwrap();
        cpu.exclusive = Some(ExclusiveReservation {
            address: 0x5000,
            size: 8,
            epoch: 77,
        });
        cpu.last_svc = Some(6);
        cpu.pending_svc = Some(0x29);
        let mut writer = StateWriter::new(PlatformId::Switch, 55);
        cpu.save(&mut writer);
        let state = writer.finish();
        let mut reader = StateReader::new(&state, PlatformId::Switch, 55).unwrap();
        let mut restored = AArch64Cpu::new();
        restored.load(&mut reader).unwrap();
        reader.finish().unwrap();
        assert_eq!(restored.x, cpu.x);
        assert_eq!(restored.sp, cpu.sp);
        assert_eq!(restored.pc, cpu.pc);
        assert_eq!(restored.nzcv, cpu.nzcv);
        assert_eq!(restored.tpidr_el0, cpu.tpidr_el0);
        assert_eq!(restored.tpidrro_el0, cpu.tpidrro_el0);
        assert_eq!(restored.counter_frequency, cpu.counter_frequency);
        assert_eq!(restored.counter_cycle_rate, cpu.counter_cycle_rate);
        let restored_exclusive = restored.exclusive.unwrap();
        assert_eq!(restored_exclusive.address, 0x5000);
        assert_eq!(restored_exclusive.size, 8);
        assert_eq!(restored_exclusive.epoch, 77);
        assert_eq!(restored.last_svc, cpu.last_svc);
        assert_eq!(restored.pending_svc, cpu.pending_svc);
    }
}
