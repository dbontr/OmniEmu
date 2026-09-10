use crate::state::{StateReader, StateWriter};

pub trait MipsBus: Send {
    fn read8(&mut self, address: u32) -> u8;
    fn write8(&mut self, address: u32, value: u8);

    fn read16(&mut self, address: u32) -> u16 {
        u16::from_le_bytes([self.read8(address), self.read8(address.wrapping_add(1))])
    }

    fn read32(&mut self, address: u32) -> u32 {
        u32::from_le_bytes([
            self.read8(address),
            self.read8(address.wrapping_add(1)),
            self.read8(address.wrapping_add(2)),
            self.read8(address.wrapping_add(3)),
        ])
    }

    fn write16(&mut self, address: u32, value: u16) {
        for (offset, byte) in value.to_le_bytes().into_iter().enumerate() {
            self.write8(address.wrapping_add(offset as u32), byte);
        }
    }

    fn write32(&mut self, address: u32, value: u32) {
        for (offset, byte) in value.to_le_bytes().into_iter().enumerate() {
            self.write8(address.wrapping_add(offset as u32), byte);
        }
    }
}
const COP0_BADVADDR: usize = 8;
const COP0_STATUS: usize = 12;
const COP0_CAUSE: usize = 13;
const COP0_EPC: usize = 14;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Exception {
    Interrupt = 0,
    AddressLoad = 4,
    AddressStore = 5,
    Syscall = 8,
    Break = 9,
    ReservedInstruction = 10,
    CoprocessorUnusable = 11,
    Overflow = 12,
}

#[derive(Debug, Clone)]
pub struct MipsR3000 {
    pub regs: [u32; 32],
    pub hi: u32,
    pub lo: u32,
    pub pc: u32,
    pub next_pc: u32,
    pub cop0: [u32; 32],
    pub cycles: u64,
    current_pc: u32,
    exception_raised: bool,
    branch_delay_next: bool,
    in_delay_slot: bool,
    pending_load: Option<(u8, u32)>,
    next_load: Option<(u8, u32)>,
    written_reg: Option<u8>,
}
impl Default for MipsR3000 {
    fn default() -> Self {
        Self {
            regs: [0; 32],
            hi: 0,
            lo: 0,
            pc: 0xbfc0_0000,
            next_pc: 0xbfc0_0004,
            cop0: [0; 32],
            cycles: 0,
            current_pc: 0xbfc0_0000,
            exception_raised: false,
            branch_delay_next: false,
            in_delay_slot: false,
            pending_load: None,
            next_load: None,
            written_reg: None,
        }
    }
}

impl MipsR3000 {
    pub fn reset(&mut self) {
        *self = Self::default();
        self.cop0[COP0_STATUS] = 1 << 22;
    }

    pub fn set_irq_line(&mut self, line: u8, asserted: bool) {
        if line >= 8 {
            return;
        }
        let bit = 8 + u32::from(line);
        if asserted {
            self.cop0[COP0_CAUSE] |= 1 << bit;
        } else {
            self.cop0[COP0_CAUSE] &= !(1 << bit);
        }
    }
    fn interrupts_pending(&self) -> bool {
        let status = self.cop0[COP0_STATUS];
        let cause = self.cop0[COP0_CAUSE];
        status & 1 != 0 && status & cause & 0x0000_ff00 != 0
    }

    fn reg(&self, index: u8) -> u32 {
        self.regs[usize::from(index)]
    }

    fn write_reg(&mut self, index: u8, value: u32) {
        if index != 0 {
            self.regs[usize::from(index)] = value;
            self.written_reg = Some(index);
        }
    }

    fn schedule_load(&mut self, index: u8, value: u32) {
        if index != 0 {
            self.next_load = Some((index, value));
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
    fn shamt(instruction: u32) -> u32 {
        (instruction >> 6) & 31
    }
    fn imm(instruction: u32) -> u16 {
        instruction as u16
    }
    fn simm(instruction: u32) -> u32 {
        (instruction as u16 as i16 as i32) as u32
    }

    fn branch_target(&self, instruction: u32) -> u32 {
        self.pc
            .wrapping_add(Self::simm(instruction).wrapping_shl(2))
    }
    fn take_exception(&mut self, exception: Exception, bad_address: Option<u32>) {
        self.exception_raised = true;
        let current = self.current_pc;
        let mut cause = self.cop0[COP0_CAUSE] & 0x0000_ff00;
        cause |= (exception as u32) << 2;
        let epc = if self.in_delay_slot {
            cause |= 1 << 31;
            current.wrapping_sub(4)
        } else {
            current
        };
        self.cop0[COP0_CAUSE] = cause;
        self.cop0[COP0_EPC] = epc;
        if let Some(address) = bad_address {
            self.cop0[COP0_BADVADDR] = address;
        }
        let status = self.cop0[COP0_STATUS];
        self.cop0[COP0_STATUS] = (status & !0x3f) | ((status << 2) & 0x3f);
        let vector = if status & (1 << 22) != 0 {
            0xbfc0_0180
        } else {
            0x8000_0080
        };
        self.pc = vector;
        self.next_pc = vector.wrapping_add(4);
        self.branch_delay_next = false;
        self.in_delay_slot = false;
        self.pending_load = None;
        self.next_load = None;
    }

    fn overflow_add(left: u32, right: u32) -> Option<u32> {
        (left as i32).checked_add(right as i32).map(|v| v as u32)
    }

    fn overflow_sub(left: u32, right: u32) -> Option<u32> {
        (left as i32).checked_sub(right as i32).map(|v| v as u32)
    }
    pub fn step<B: MipsBus>(&mut self, bus: &mut B) -> u32 {
        if self.interrupts_pending() {
            self.current_pc = self.pc;
            self.in_delay_slot = false;
            self.take_exception(Exception::Interrupt, None);
            self.cycles = self.cycles.wrapping_add(1);
            return 1;
        }
        let instruction_pc = self.pc;
        self.current_pc = instruction_pc;
        self.in_delay_slot = self.branch_delay_next;
        self.branch_delay_next = false;
        if instruction_pc & 3 != 0 {
            self.take_exception(Exception::AddressLoad, Some(instruction_pc));
            self.cycles = self.cycles.wrapping_add(1);
            return 1;
        }
        let instruction = bus.read32(instruction_pc);
        self.pc = self.next_pc;
        self.next_pc = self.next_pc.wrapping_add(4);
        self.written_reg = None;
        self.next_load = None;
        self.exception_raised = false;
        let old_load = self.pending_load.take();
        self.execute(bus, instruction);
        if !self.exception_raised {
            if let Some((reg, value)) = old_load {
                if self.written_reg != Some(reg) {
                    self.regs[usize::from(reg)] = value;
                }
            }
            self.pending_load = self.next_load.take();
        }
        self.regs[0] = 0;
        self.cycles = self.cycles.wrapping_add(1);
        1
    }
    fn execute<B: MipsBus>(&mut self, bus: &mut B, instruction: u32) {
        match instruction >> 26 {
            0x00 => self.execute_special(bus, instruction),
            0x01 => self.execute_regimm(instruction),
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
            0x08 => self.addi(instruction, true),
            0x09 => self.addi(instruction, false),
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
                self.reg(Self::rs(instruction)) & u32::from(Self::imm(instruction)),
            ),
            0x0d => self.write_reg(
                Self::rt(instruction),
                self.reg(Self::rs(instruction)) | u32::from(Self::imm(instruction)),
            ),
            0x0e => self.write_reg(
                Self::rt(instruction),
                self.reg(Self::rs(instruction)) ^ u32::from(Self::imm(instruction)),
            ),
            0x0f => self.write_reg(
                Self::rt(instruction),
                u32::from(Self::imm(instruction)) << 16,
            ),
            0x10 => self.execute_cop0(instruction),
            0x20..=0x26 => self.load_memory(bus, instruction),
            0x28..=0x2e => self.store(bus, instruction),
            0x30..=0x33 | 0x38..=0x3b => self.take_exception(Exception::CoprocessorUnusable, None),
            _ => self.take_exception(Exception::ReservedInstruction, None),
        }
    }
    fn execute_special<B: MipsBus>(&mut self, _bus: &mut B, instruction: u32) {
        let rs = Self::rs(instruction);
        let rt = Self::rt(instruction);
        let rd = Self::rd(instruction);
        let shamt = Self::shamt(instruction);
        match instruction & 0x3f {
            0x00 => self.write_reg(rd, self.reg(rt) << shamt),
            0x02 => self.write_reg(rd, self.reg(rt) >> shamt),
            0x03 => self.write_reg(rd, ((self.reg(rt) as i32) >> shamt) as u32),
            0x04 => self.write_reg(rd, self.reg(rt) << (self.reg(rs) & 31)),
            0x06 => self.write_reg(rd, self.reg(rt) >> (self.reg(rs) & 31)),
            0x07 => self.write_reg(rd, ((self.reg(rt) as i32) >> (self.reg(rs) & 31)) as u32),
            0x08 => {
                self.next_pc = self.reg(rs);
                self.branch_delay_next = true;
            }
            0x09 => {
                let link = if rd == 0 { 31 } else { rd };
                self.write_reg(link, self.next_pc);
                self.next_pc = self.reg(rs);
                self.branch_delay_next = true;
            }
            0x0c => self.take_exception(Exception::Syscall, None),
            0x0d => self.take_exception(Exception::Break, None),
            0x10 => self.write_reg(rd, self.hi),
            0x11 => self.hi = self.reg(rs),
            0x12 => self.write_reg(rd, self.lo),
            0x13 => self.lo = self.reg(rs),
            _ => self.execute_special_alu(instruction),
        }
    }
    fn execute_special_alu(&mut self, instruction: u32) {
        let rs = Self::rs(instruction);
        let rt = Self::rt(instruction);
        let rd = Self::rd(instruction);
        let left = self.reg(rs);
        let right = self.reg(rt);
        match instruction & 0x3f {
            0x18 => {
                let result = i64::from(left as i32) * i64::from(right as i32);
                self.lo = result as u32;
                self.hi = (result >> 32) as u32;
            }
            0x19 => {
                let result = u64::from(left) * u64::from(right);
                self.lo = result as u32;
                self.hi = (result >> 32) as u32;
            }
            0x1a => self.divide_signed(left, right),
            0x1b => self.divide_unsigned(left, right),
            0x20 => {
                if let Some(value) = Self::overflow_add(left, right) {
                    self.write_reg(rd, value);
                } else {
                    self.take_exception(Exception::Overflow, None);
                }
            }
            0x21 => self.write_reg(rd, left.wrapping_add(right)),
            0x22 => {
                if let Some(value) = Self::overflow_sub(left, right) {
                    self.write_reg(rd, value);
                } else {
                    self.take_exception(Exception::Overflow, None);
                }
            }
            0x23 => self.write_reg(rd, left.wrapping_sub(right)),
            0x24 => self.write_reg(rd, left & right),
            0x25 => self.write_reg(rd, left | right),
            0x26 => self.write_reg(rd, left ^ right),
            0x27 => self.write_reg(rd, !(left | right)),
            0x2a => self.write_reg(rd, ((left as i32) < (right as i32)) as u32),
            0x2b => self.write_reg(rd, (left < right) as u32),
            _ => self.take_exception(Exception::ReservedInstruction, None),
        }
    }
    fn divide_signed(&mut self, left: u32, right: u32) {
        let dividend = left as i32;
        let divisor = right as i32;
        if divisor == 0 {
            self.lo = if dividend >= 0 { u32::MAX } else { 1 };
            self.hi = left;
        } else if dividend == i32::MIN && divisor == -1 {
            self.lo = i32::MIN as u32;
            self.hi = 0;
        } else {
            self.lo = (dividend / divisor) as u32;
            self.hi = (dividend % divisor) as u32;
        }
    }

    fn divide_unsigned(&mut self, left: u32, right: u32) {
        if let Some(quotient) = left.checked_div(right) {
            self.lo = quotient;
            self.hi = left % right;
        } else {
            self.lo = u32::MAX;
            self.hi = left;
        }
    }

    fn execute_regimm(&mut self, instruction: u32) {
        let rs = self.reg(Self::rs(instruction)) as i32;
        let rt = Self::rt(instruction);
        let (take, link) = match rt {
            0x00 => (rs < 0, false),
            0x01 => (rs >= 0, false),
            0x10 => (rs < 0, true),
            0x11 => (rs >= 0, true),
            _ => {
                self.take_exception(Exception::ReservedInstruction, None);
                return;
            }
        };
        if link {
            self.write_reg(31, self.next_pc);
        }
        self.branch(instruction, take);
    }
    fn branch(&mut self, instruction: u32, take: bool) {
        if take {
            self.next_pc = self.branch_target(instruction);
        }
        self.branch_delay_next = true;
    }

    fn jump(&mut self, instruction: u32, link: bool) {
        if link {
            self.write_reg(31, self.next_pc);
        }
        self.next_pc = (self.pc & 0xf000_0000) | ((instruction & 0x03ff_ffff) << 2);
        self.branch_delay_next = true;
    }

    fn addi(&mut self, instruction: u32, checked: bool) {
        let left = self.reg(Self::rs(instruction));
        let right = Self::simm(instruction);
        if checked {
            if let Some(value) = Self::overflow_add(left, right) {
                self.write_reg(Self::rt(instruction), value);
            } else {
                self.take_exception(Exception::Overflow, None);
            }
        } else {
            self.write_reg(Self::rt(instruction), left.wrapping_add(right));
        }
    }

    fn execute_cop0(&mut self, instruction: u32) {
        let rs = Self::rs(instruction);
        let rt = Self::rt(instruction);
        let rd = usize::from(Self::rd(instruction));
        match rs {
            0x00 => self.schedule_load(rt, self.cop0[rd]),
            0x04 => self.cop0[rd] = self.reg(rt),
            0x10 if instruction & 0x3f == 0x10 => {
                let status = self.cop0[COP0_STATUS];
                self.cop0[COP0_STATUS] = (status & !0x0f) | ((status >> 2) & 0x0f);
            }
            _ => self.take_exception(Exception::ReservedInstruction, None),
        }
    }

    fn effective_address(&self, instruction: u32) -> u32 {
        self.reg(Self::rs(instruction))
            .wrapping_add(Self::simm(instruction))
    }

    fn load_memory<B: MipsBus>(&mut self, bus: &mut B, instruction: u32) {
        let opcode = instruction >> 26;
        let rt = Self::rt(instruction);
        let address = self.effective_address(instruction);
        match opcode {
            0x20 => self.schedule_load(rt, (bus.read8(address) as i8 as i32) as u32),
            0x21 => {
                if address & 1 != 0 {
                    self.take_exception(Exception::AddressLoad, Some(address));
                } else {
                    self.schedule_load(rt, (bus.read16(address) as i16 as i32) as u32);
                }
            }
            0x22 => {
                let aligned = address & !3;
                let memory = bus.read32(aligned);
                let old = self.reg(rt);
                let value = match address & 3 {
                    0 => (old & 0x00ff_ffff) | (memory << 24),
                    1 => (old & 0x0000_ffff) | (memory << 16),
                    2 => (old & 0x0000_00ff) | (memory << 8),
                    _ => memory,
                };
                self.schedule_load(rt, value);
            }
            0x23 => {
                if address & 3 != 0 {
                    self.take_exception(Exception::AddressLoad, Some(address));
                } else {
                    self.schedule_load(rt, bus.read32(address));
                }
            }
            0x24 => self.schedule_load(rt, u32::from(bus.read8(address))),
            0x25 => {
                if address & 1 != 0 {
                    self.take_exception(Exception::AddressLoad, Some(address));
                } else {
                    self.schedule_load(rt, u32::from(bus.read16(address)));
                }
            }
            0x26 => {
                let aligned = address & !3;
                let memory = bus.read32(aligned);
                let old = self.reg(rt);
                let value = match address & 3 {
                    0 => memory,
                    1 => (old & 0xff00_0000) | (memory >> 8),
                    2 => (old & 0xffff_0000) | (memory >> 16),
                    _ => (old & 0xffff_ff00) | (memory >> 24),
                };
                self.schedule_load(rt, value);
            }
            _ => self.take_exception(Exception::ReservedInstruction, None),
        }
    }

    fn store<B: MipsBus>(&mut self, bus: &mut B, instruction: u32) {
        let opcode = instruction >> 26;
        let value = self.reg(Self::rt(instruction));
        let address = self.effective_address(instruction);
        match opcode {
            0x28 => bus.write8(address, value as u8),
            0x29 => {
                if address & 1 != 0 {
                    self.take_exception(Exception::AddressStore, Some(address));
                } else {
                    bus.write16(address, value as u16);
                }
            }
            0x2a => {
                let aligned = address & !3;
                let memory = bus.read32(aligned);
                let merged = match address & 3 {
                    0 => (memory & 0xffff_ff00) | (value >> 24),
                    1 => (memory & 0xffff_0000) | (value >> 16),
                    2 => (memory & 0xff00_0000) | (value >> 8),
                    _ => value,
                };
                bus.write32(aligned, merged);
            }
            0x2b => {
                if address & 3 != 0 {
                    self.take_exception(Exception::AddressStore, Some(address));
                } else {
                    bus.write32(address, value);
                }
            }
            0x2e => {
                let aligned = address & !3;
                let memory = bus.read32(aligned);
                let merged = match address & 3 {
                    0 => value,
                    1 => (memory & 0x0000_00ff) | (value << 8),
                    2 => (memory & 0x0000_ffff) | (value << 16),
                    _ => (memory & 0x00ff_ffff) | (value << 24),
                };
                bus.write32(aligned, merged);
            }
            _ => self.take_exception(Exception::ReservedInstruction, None),
        }
    }
    pub fn save(&self, out: &mut StateWriter) {
        for reg in self.regs {
            out.u32(reg);
        }
        out.u32(self.hi);
        out.u32(self.lo);
        out.u32(self.pc);
        out.u32(self.next_pc);
        for reg in self.cop0 {
            out.u32(reg);
        }
        out.u64(self.cycles);
        out.u32(self.current_pc);
        out.u8(self.branch_delay_next as u8);
        out.u8(self.in_delay_slot as u8);
        match self.pending_load {
            Some((reg, value)) => {
                out.u8(1);
                out.u8(reg);
                out.u32(value);
            }
            None => out.u8(0),
        }
    }

    pub fn load_state(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        for reg in &mut self.regs {
            *reg = input.u32()?;
        }
        self.hi = input.u32()?;
        self.lo = input.u32()?;
        self.pc = input.u32()?;
        self.next_pc = input.u32()?;
        for reg in &mut self.cop0 {
            *reg = input.u32()?;
        }
        self.cycles = input.u64()?;
        self.current_pc = input.u32()?;
        self.branch_delay_next = input.u8()? != 0;
        self.in_delay_slot = input.u8()? != 0;
        self.pending_load = if input.u8()? != 0 {
            let reg = input.u8()?;
            if reg >= 32 {
                return Err("invalid MIPS delayed-load register".into());
            }
            Some((reg, input.u32()?))
        } else {
            None
        };
        self.next_load = None;
        self.written_reg = None;
        self.exception_raised = false;
        self.regs[0] = 0;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct Ram {
        bytes: Vec<u8>,
    }

    impl Ram {
        fn new() -> Self {
            Self {
                bytes: vec![0; 0x20_000],
            }
        }
        fn put32(&mut self, address: u32, value: u32) {
            let index = address as usize;
            self.bytes[index..index + 4].copy_from_slice(&value.to_le_bytes());
        }
    }
    impl MipsBus for Ram {
        fn read8(&mut self, address: u32) -> u8 {
            self.bytes[address as usize % self.bytes.len()]
        }
        fn write8(&mut self, address: u32, value: u8) {
            let index = address as usize % self.bytes.len();
            self.bytes[index] = value;
        }
    }

    fn r(rs: u8, rt: u8, rd: u8, shamt: u8, funct: u8) -> u32 {
        (u32::from(rs) << 21)
            | (u32::from(rt) << 16)
            | (u32::from(rd) << 11)
            | (u32::from(shamt) << 6)
            | u32::from(funct)
    }

    fn i(op: u8, rs: u8, rt: u8, imm: u16) -> u32 {
        (u32::from(op) << 26) | (u32::from(rs) << 21) | (u32::from(rt) << 16) | u32::from(imm)
    }

    fn cpu_at_zero() -> MipsR3000 {
        MipsR3000 {
            pc: 0,
            next_pc: 4,
            current_pc: 0,
            ..Default::default()
        }
    }

    #[test]
    fn arithmetic_branch_delay_and_link_execute() {
        let mut bus = Ram::new();
        bus.put32(0, i(0x09, 0, 1, 5));
        bus.put32(4, i(0x09, 0, 2, 7));
        bus.put32(8, r(1, 2, 3, 0, 0x21));
        bus.put32(12, i(0x04, 3, 0, 2));
        bus.put32(12, i(0x04, 3, 3, 2));
        bus.put32(16, i(0x09, 0, 4, 1));
        bus.put32(20, i(0x09, 0, 4, 2));
        bus.put32(24, i(0x09, 0, 5, 9));
        bus.put32(28, (0x03u32 << 26) | (40 >> 2));
        bus.put32(32, i(0x09, 0, 6, 3));
        bus.put32(40, i(0x09, 0, 7, 4));
        let mut cpu = cpu_at_zero();
        for _ in 0..9 {
            cpu.step(&mut bus);
        }
        assert_eq!(cpu.reg(3), 12);
        assert_eq!(cpu.reg(4), 1);
        assert_eq!(cpu.reg(5), 9);
        assert_eq!(cpu.reg(6), 3);
        assert_eq!(cpu.reg(31), 36);
        assert_eq!(cpu.reg(7), 4);
    }

    #[test]
    fn load_delay_and_unaligned_merge_are_modeled() {
        let mut bus = Ram::new();
        bus.put32(0x100, 0x4433_2211);
        bus.put32(0, i(0x23, 0, 1, 0x100));
        bus.put32(4, r(1, 0, 2, 0, 0x21));
        bus.put32(8, r(1, 0, 3, 0, 0x21));
        let mut cpu = cpu_at_zero();
        cpu.step(&mut bus);
        cpu.step(&mut bus);
        assert_eq!(cpu.reg(2), 0);
        cpu.step(&mut bus);
        assert_eq!(cpu.reg(3), 0x4433_2211);
        cpu.pc = 0x20;
        cpu.next_pc = 0x24;
        cpu.regs[4] = 0xaabb_ccdd;
        bus.put32(0x20, i(0x22, 0, 4, 0x101));
        bus.put32(0x24, 0);
        cpu.step(&mut bus);
        cpu.step(&mut bus);
        assert_eq!(cpu.reg(4), 0x2211_ccdd);
        cpu.regs[4] = 0xaabb_ccdd;
        cpu.pc = 0x30;
        cpu.next_pc = 0x34;
        bus.put32(0x30, i(0x26, 0, 4, 0x102));
        bus.put32(0x34, 0);
        cpu.step(&mut bus);
        cpu.step(&mut bus);
        assert_eq!(cpu.reg(4), 0xaabb_4433);
    }

    #[test]
    fn exception_in_delay_slot_sets_bd_and_branch_epc() {
        let mut bus = Ram::new();
        bus.put32(0, i(0x04, 0, 0, 1));
        bus.put32(4, 0x0000_000c);
        let mut cpu = cpu_at_zero();
        cpu.cop0[COP0_STATUS] = 0;
        cpu.step(&mut bus);
        cpu.step(&mut bus);
        assert_ne!(cpu.cop0[COP0_CAUSE] & (1 << 31), 0);
        assert_eq!(cpu.cop0[COP0_EPC], 0);
        assert_eq!(
            (cpu.cop0[COP0_CAUSE] >> 2) & 0x1f,
            Exception::Syscall as u32
        );
        assert_eq!(cpu.pc, 0x8000_0080);
    }
    #[test]
    fn cop0_interrupt_and_rfe_restore_status_stack() {
        let mut bus = Ram::new();
        let mut cpu = cpu_at_zero();
        cpu.cop0[COP0_STATUS] = 1 | (1 << 10);
        cpu.set_irq_line(2, true);
        cpu.step(&mut bus);
        assert_eq!(
            (cpu.cop0[COP0_CAUSE] >> 2) & 0x1f,
            Exception::Interrupt as u32
        );
        assert_eq!(cpu.cop0[COP0_EPC], 0);
        let stacked = cpu.cop0[COP0_STATUS];
        cpu.pc = 0x100;
        cpu.next_pc = 0x104;
        bus.put32(0x100, 0x4200_0010);
        cpu.cop0[COP0_CAUSE] = 0;
        cpu.step(&mut bus);
        assert_eq!(cpu.cop0[COP0_STATUS] & 0x0f, (stacked >> 2) & 0x0f);
    }

    #[test]
    fn state_round_trip_preserves_pipeline_state() {
        let mut cpu = cpu_at_zero();
        cpu.regs[1] = 0x1234_5678;
        cpu.hi = 7;
        cpu.lo = 9;
        cpu.pending_load = Some((2, 0xaabb_ccdd));
        let mut out = StateWriter::new(crate::platform::PlatformId::PlayStation, 7);
        cpu.save(&mut out);
        let bytes = out.finish();
        let mut restored = MipsR3000::default();
        let mut input =
            StateReader::new(&bytes, crate::platform::PlatformId::PlayStation, 7).unwrap();
        restored.load_state(&mut input).unwrap();
        input.finish().unwrap();
        assert_eq!(restored.regs[1], 0x1234_5678);
        assert_eq!(restored.pending_load, Some((2, 0xaabb_ccdd)));
        assert_eq!((restored.hi, restored.lo), (7, 9));
    }
}
