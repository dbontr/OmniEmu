use crate::state::{StateReader, StateWriter};

pub trait MipsBus: Send {
    fn read8(&mut self, address: u32) -> u8;
    fn write8(&mut self, address: u32, value: u8);

    fn fetch32(&mut self, address: u32) -> u32 {
        self.read32(address)
    }

    fn instruction_bus_error(&mut self, _address: u32) -> bool {
        false
    }

    fn data_bus_error(&mut self, _address: u32, _size: u8, _write: bool) -> bool {
        false
    }

    fn load_cycles(&mut self, _address: u32, _size: u8) -> u32 {
        1
    }

    fn set_cache_isolated(&mut self, _isolated: bool) {}

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

    fn store8(&mut self, address: u32, source: u32) {
        self.write8(address, source as u8);
    }

    fn store16(&mut self, address: u32, source: u32) {
        self.write16(address, source as u16);
    }

    fn store_left(&mut self, address: u32, source: u32) {
        let aligned = address & !3;
        let memory = self.read32(aligned);
        let merged = match address & 3 {
            0 => (memory & 0xffff_ff00) | (source >> 24),
            1 => (memory & 0xffff_0000) | (source >> 16),
            2 => (memory & 0xff00_0000) | (source >> 8),
            _ => source,
        };
        self.write32(aligned, merged);
    }

    fn store_right(&mut self, address: u32, source: u32) {
        let aligned = address & !3;
        let memory = self.read32(aligned);
        let merged = match address & 3 {
            0 => source,
            1 => (memory & 0x0000_00ff) | (source << 8),
            2 => (memory & 0x0000_ffff) | (source << 16),
            _ => (memory & 0x00ff_ffff) | (source << 24),
        };
        self.write32(aligned, merged);
    }

    fn cop2_present(&self) -> bool {
        false
    }
    fn cop2_read_data(&mut self, _register: u8) -> u32 {
        0
    }
    fn cop2_write_data(&mut self, _register: u8, _value: u32) {}
    fn cop2_write_data_at(&mut self, register: u8, value: u32, _cycle: u64) {
        self.cop2_write_data(register, value);
    }
    fn cop2_read_control(&mut self, _register: u8) -> u32 {
        0
    }
    fn cop2_write_control(&mut self, _register: u8, _value: u32) {}
    fn cop2_write_control_at(&mut self, register: u8, value: u32, _cycle: u64) {
        self.cop2_write_control(register, value);
    }
    fn cop2_command(&mut self, _instruction: u32) -> u32 {
        1
    }
    fn cop2_command_at(&mut self, instruction: u32, _cycle: u64) -> u32 {
        self.cop2_command(instruction)
    }
    fn cop2_advance_to(&mut self, _cycle: u64) {}
}
const COP0_TAR: usize = 6;
const COP0_BADVADDR: usize = 8;
const COP0_STATUS: usize = 12;
const COP0_CAUSE: usize = 13;
const COP0_EPC: usize = 14;
const COP0_PRID: usize = 15;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Exception {
    Interrupt = 0,
    AddressLoad = 4,
    AddressStore = 5,
    BusErrorInstruction = 6,
    BusErrorData = 7,
    Syscall = 8,
    Break = 9,
    ReservedInstruction = 10,
    CoprocessorUnusable = 11,
    Overflow = 12,
}

#[derive(Debug, Clone, Copy)]
struct PendingCop2Write {
    control: bool,
    register: u8,
    value: u32,
    ready_cycle: u64,
}

#[derive(Debug, Clone, Copy)]
struct PendingCop2Enable {
    previous: bool,
    enabled: bool,
    ready_cycle: u64,
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
    branch_target_next: Option<u32>,
    delay_slot_target: Option<u32>,
    in_delay_slot: bool,
    pending_load: Option<(u8, u32)>,
    next_load: Option<(u8, u32)>,
    written_reg: Option<u8>,
    step_cycles: u32,
    cop2_busy_until: u64,
    pending_cop2_writes: Vec<PendingCop2Write>,
    pending_cop2_enable: Option<PendingCop2Enable>,
}
impl Default for MipsR3000 {
    fn default() -> Self {
        let mut cop0 = [0; 32];
        cop0[COP0_PRID] = 2;
        Self {
            regs: [0; 32],
            hi: 0,
            lo: 0,
            pc: 0xbfc0_0000,
            next_pc: 0xbfc0_0004,
            cop0,
            cycles: 0,
            current_pc: 0xbfc0_0000,
            exception_raised: false,
            branch_delay_next: false,
            branch_target_next: None,
            delay_slot_target: None,
            in_delay_slot: false,
            pending_load: None,
            next_load: None,
            written_reg: None,
            step_cycles: 1,
            cop2_busy_until: 0,
            pending_cop2_writes: Vec::new(),
            pending_cop2_enable: None,
        }
    }
}

impl MipsR3000 {
    pub fn reset(&mut self) {
        *self = Self::default();
        self.cop0[COP0_STATUS] = 1 << 22;
    }

    pub fn enter_program(&mut self, pc: u32) {
        self.pc = pc;
        self.next_pc = pc.wrapping_add(4);
        self.current_pc = pc;
        self.exception_raised = false;
        self.branch_delay_next = false;
        self.branch_target_next = None;
        self.delay_slot_target = None;
        self.in_delay_slot = false;
        self.pending_load = None;
        self.next_load = None;
        self.written_reg = None;
        self.step_cycles = 0;
        self.cop2_busy_until = self.cycles;
        self.pending_cop2_writes.clear();
        self.pending_cop2_enable = None;
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

    fn commit_pending_load(&mut self) {
        if let Some((reg, value)) = self.pending_load.take() {
            self.regs[usize::from(reg)] = value;
        }
    }

    fn cop2_write_delay(control: bool, register: u8) -> u64 {
        if !control && matches!(register & 31, 28 | 30) {
            3
        } else {
            2
        }
    }

    fn queue_cop2_write(&mut self, control: bool, register: u8, value: u32) {
        self.pending_cop2_writes.push(PendingCop2Write {
            control,
            register: register & 31,
            value,
            ready_cycle: self
                .cycles
                .wrapping_add(Self::cop2_write_delay(control, register)),
        });
    }

    fn commit_cop2_writes<B: MipsBus>(&mut self, bus: &mut B, through_cycle: u64) {
        let mut index = 0;
        while index < self.pending_cop2_writes.len() {
            if self.pending_cop2_writes[index].ready_cycle <= through_cycle {
                let pending = self.pending_cop2_writes.remove(index);
                if pending.control {
                    bus.cop2_write_control_at(pending.register, pending.value, pending.ready_cycle);
                } else {
                    bus.cop2_write_data_at(pending.register, pending.value, pending.ready_cycle);
                }
            } else {
                index += 1;
            }
        }
    }

    fn cop2_wait_cycles(&self) -> u32 {
        self.cop2_busy_until
            .saturating_sub(self.cycles)
            .min(u64::from(u32::MAX)) as u32
    }

    fn effective_cop2_enabled(&self) -> bool {
        match self.pending_cop2_enable {
            Some(pending) if self.cycles < pending.ready_cycle => pending.previous,
            Some(pending) => pending.enabled,
            None => self.cop0[COP0_STATUS] & (1 << 30) != 0,
        }
    }

    fn commit_cop2_enable(&mut self) {
        if self
            .pending_cop2_enable
            .is_some_and(|pending| pending.ready_cycle <= self.cycles)
        {
            self.pending_cop2_enable = None;
        }
    }

    fn write_cop0(&mut self, register: usize, value: u32) {
        if register == COP0_STATUS {
            let previous = self.effective_cop2_enabled();
            let enabled = value & (1 << 30) != 0;
            self.cop0[register] = value;
            self.pending_cop2_enable = if enabled != previous {
                Some(PendingCop2Enable {
                    previous,
                    enabled,
                    ready_cycle: self.cycles.wrapping_add(2),
                })
            } else {
                None
            };
        } else if register == COP0_CAUSE {
            self.cop0[register] = (self.cop0[register] & !0x0000_0300) | (value & 0x0000_0300);
        } else if register != COP0_PRID {
            self.cop0[register] = value;
        }
    }

    fn cop0_register_invalid(register: usize) -> bool {
        matches!(register, 0 | 1 | 2 | 4 | 10)
    }

    fn cop0_access_allowed(&self, register: usize) -> bool {
        register >= 16
            || self.cop0[COP0_STATUS] & (1 << 1) == 0
            || self.cop0[COP0_STATUS] & (1 << 28) != 0
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
            if let Some(target) = self.delay_slot_target {
                cause |= 1 << 30;
                self.cop0[COP0_TAR] = target;
            }
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
        self.branch_target_next = None;
        self.delay_slot_target = None;
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

    fn user_mode(&self) -> bool {
        self.cop0[COP0_STATUS] & (1 << 1) != 0
    }

    fn data_address_error(&mut self, address: u32, write: bool) -> bool {
        if self.user_mode() && address >= 0x8000_0000 {
            self.take_exception(
                if write {
                    Exception::AddressStore
                } else {
                    Exception::AddressLoad
                },
                Some(address),
            );
            true
        } else {
            false
        }
    }

    fn data_bus_fault<B: MipsBus>(
        &mut self,
        bus: &mut B,
        address: u32,
        size: u8,
        write: bool,
    ) -> bool {
        if bus.data_bus_error(address, size, write) {
            self.take_exception(Exception::BusErrorData, None);
            true
        } else {
            false
        }
    }

    fn charge_load_cycles<B: MipsBus>(&mut self, bus: &mut B, address: u32, size: u8) {
        self.step_cycles = self.step_cycles.max(bus.load_cycles(address, size).max(1));
    }

    pub fn step<B: MipsBus>(&mut self, bus: &mut B) -> u32 {
        self.commit_cop2_enable();
        self.commit_cop2_writes(bus, self.cycles);
        bus.cop2_advance_to(self.cycles);
        bus.set_cache_isolated(self.cop0[COP0_STATUS] & (1 << 16) != 0);
        if self.interrupts_pending() {
            self.current_pc = self.pc;
            self.in_delay_slot = self.branch_delay_next;
            self.delay_slot_target = if self.in_delay_slot {
                self.branch_target_next
            } else {
                None
            };
            if self.pc & 3 == 0
                && bus.cop2_present()
                && self.effective_cop2_enabled()
                && self.cop2_wait_cycles() == 0
                && !bus.instruction_bus_error(self.pc)
            {
                let instruction = bus.fetch32(self.pc);
                if instruction & 0xfe00_0000 == 0x4a00_0000 {
                    let latency = bus.cop2_command_at(instruction, self.cycles).max(1);
                    self.cop2_busy_until = self.cycles.wrapping_add(u64::from(latency));
                }
            }
            self.commit_pending_load();
            self.take_exception(Exception::Interrupt, None);
            self.cycles = self.cycles.wrapping_add(1);
            return 1;
        }
        let instruction_pc = self.pc;
        self.current_pc = instruction_pc;
        self.in_delay_slot = self.branch_delay_next;
        self.delay_slot_target = if self.in_delay_slot {
            self.branch_target_next.take()
        } else {
            None
        };
        self.branch_delay_next = false;
        if instruction_pc & 3 != 0 || (self.user_mode() && instruction_pc >= 0x8000_0000) {
            self.commit_pending_load();
            self.take_exception(Exception::AddressLoad, Some(instruction_pc));
            self.cycles = self.cycles.wrapping_add(1);
            return 1;
        }
        if bus.instruction_bus_error(instruction_pc) {
            self.commit_pending_load();
            self.take_exception(Exception::BusErrorInstruction, None);
            self.cycles = self.cycles.wrapping_add(1);
            return 1;
        }
        let instruction = bus.fetch32(instruction_pc);
        self.pc = self.next_pc;
        self.next_pc = self.next_pc.wrapping_add(4);
        self.written_reg = None;
        self.next_load = None;
        self.exception_raised = false;
        self.step_cycles = 1;
        let old_load = self.pending_load.take();
        self.execute(bus, instruction);
        if let Some((reg, value)) = old_load {
            if self.written_reg != Some(reg) {
                self.regs[usize::from(reg)] = value;
            }
        }
        if !self.exception_raised {
            self.pending_load = self.next_load.take();
        }
        self.regs[0] = 0;
        self.cycles = self.cycles.wrapping_add(u64::from(self.step_cycles));
        self.step_cycles
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
            0x11 => self.cop_unusable(1),
            0x12 => self.execute_cop2(bus, instruction),
            0x13 => self.cop_unusable(3),
            0x20..=0x26 => self.load_memory(bus, instruction),
            0x28..=0x2e => self.store(bus, instruction),
            0x30 | 0x38 => self.cop_unusable(0),
            0x31 | 0x39 => self.cop_unusable(1),
            0x32 => self.load_cop2(bus, instruction),
            0x33 | 0x3b => self.cop_unusable(3),
            0x3a => self.store_cop2(bus, instruction),
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
                let target = self.reg(rs);
                self.next_pc = target;
                self.branch_target_next = Some(target);
                self.branch_delay_next = true;
            }
            0x09 => {
                self.write_reg(rd, self.next_pc);
                let target = self.reg(rs);
                self.next_pc = target;
                self.branch_target_next = Some(target);
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
        self.branch_target_next = if take {
            let target = self.branch_target(instruction);
            self.next_pc = target;
            Some(target)
        } else {
            None
        };
        self.branch_delay_next = true;
    }

    fn jump(&mut self, instruction: u32, link: bool) {
        if link {
            self.write_reg(31, self.next_pc);
        }
        let target = (self.pc & 0xf000_0000) | ((instruction & 0x03ff_ffff) << 2);
        self.next_pc = target;
        self.branch_target_next = Some(target);
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
            0x00 => {
                if Self::cop0_register_invalid(rd) {
                    self.take_exception(Exception::ReservedInstruction, None);
                } else if !self.cop0_access_allowed(rd) {
                    self.cop_unusable(0);
                } else {
                    self.schedule_load(rt, self.cop0[rd]);
                }
            }
            0x04 => {
                if Self::cop0_register_invalid(rd) {
                    self.take_exception(Exception::ReservedInstruction, None);
                } else if !self.cop0_access_allowed(rd) {
                    self.cop_unusable(0);
                } else {
                    self.write_cop0(rd, self.reg(rt));
                }
            }
            0x10 if instruction & 0x3f == 0x10 => {
                if !self.cop0_access_allowed(COP0_STATUS) {
                    self.cop_unusable(0);
                    return;
                }
                let status = self.cop0[COP0_STATUS];
                self.cop0[COP0_STATUS] = (status & !0x0f) | ((status >> 2) & 0x0f);
            }
            _ => self.take_exception(Exception::ReservedInstruction, None),
        }
    }

    fn cop_unusable(&mut self, coprocessor: u8) {
        self.take_exception(Exception::CoprocessorUnusable, None);
        self.cop0[COP0_CAUSE] =
            (self.cop0[COP0_CAUSE] & !(3 << 28)) | (u32::from(coprocessor & 3) << 28);
    }

    fn cop2_usable<B: MipsBus>(&mut self, bus: &B) -> bool {
        if bus.cop2_present() && self.effective_cop2_enabled() {
            true
        } else {
            self.cop_unusable(2);
            false
        }
    }

    fn execute_cop2<B: MipsBus>(&mut self, bus: &mut B, instruction: u32) {
        if !self.cop2_usable(bus) {
            return;
        }
        let rs = Self::rs(instruction);
        let rt = Self::rt(instruction);
        let rd = Self::rd(instruction);
        match rs {
            0x00 | 0x02 => {
                let wait = self.cop2_wait_cycles();
                let access_cycle = self.cycles.wrapping_add(u64::from(wait));
                self.commit_cop2_writes(bus, access_cycle);
                bus.cop2_advance_to(access_cycle);
                let value = if rs == 0x00 {
                    bus.cop2_read_data(rd)
                } else {
                    bus.cop2_read_control(rd)
                };
                self.schedule_load(rt, value);
                self.step_cycles = wait.saturating_add(1);
            }
            0x04 => self.queue_cop2_write(false, rd, self.reg(rt)),
            0x06 => self.queue_cop2_write(true, rd, self.reg(rt)),
            0x08 if rt <= 1 => self.branch(instruction, rt == 0),
            0x10..=0x1f => {
                let wait = self.cop2_wait_cycles();
                let issue_cycle = self.cycles.wrapping_add(u64::from(wait));
                self.commit_cop2_writes(bus, issue_cycle);
                bus.cop2_advance_to(issue_cycle);
                let latency = bus.cop2_command_at(instruction, issue_cycle).max(1);
                self.cop2_busy_until = issue_cycle.wrapping_add(u64::from(latency));
                self.step_cycles = wait.saturating_add(1);
            }
            _ => self.take_exception(Exception::ReservedInstruction, None),
        }
    }

    fn load_cop2<B: MipsBus>(&mut self, bus: &mut B, instruction: u32) {
        if !self.cop2_usable(bus) {
            return;
        }
        let address = self.effective_address(instruction);
        if address & 3 != 0 {
            self.take_exception(Exception::AddressLoad, Some(address));
            return;
        }
        if self.data_address_error(address, false) || self.data_bus_fault(bus, address, 4, false) {
            return;
        }
        self.charge_load_cycles(bus, address, 4);
        let value = bus.read32(address);
        self.queue_cop2_write(false, Self::rt(instruction), value);
    }

    fn store_cop2<B: MipsBus>(&mut self, bus: &mut B, instruction: u32) {
        if !self.cop2_usable(bus) {
            return;
        }
        let address = self.effective_address(instruction);
        if address & 3 != 0 {
            self.take_exception(Exception::AddressStore, Some(address));
            return;
        }
        if self.data_address_error(address, true) || self.data_bus_fault(bus, address, 4, true) {
            return;
        }
        let wait = self.cop2_wait_cycles();
        let access_cycle = self.cycles.wrapping_add(u64::from(wait));
        self.commit_cop2_writes(bus, access_cycle);
        bus.cop2_advance_to(access_cycle);
        let value = bus.cop2_read_data(Self::rt(instruction));
        bus.write32(address, value);
        self.step_cycles = wait.saturating_add(1);
    }

    fn effective_address(&self, instruction: u32) -> u32 {
        self.reg(Self::rs(instruction))
            .wrapping_add(Self::simm(instruction))
    }

    fn load_memory<B: MipsBus>(&mut self, bus: &mut B, instruction: u32) {
        let opcode = instruction >> 26;
        let rt = Self::rt(instruction);
        let address = self.effective_address(instruction);
        if self.data_address_error(address, false) {
            return;
        }
        match opcode {
            0x20 => {
                if !self.data_bus_fault(bus, address, 1, false) {
                    self.charge_load_cycles(bus, address, 1);
                    self.schedule_load(rt, (bus.read8(address) as i8 as i32) as u32);
                }
            }
            0x21 => {
                if address & 1 != 0 {
                    self.take_exception(Exception::AddressLoad, Some(address));
                } else if !self.data_bus_fault(bus, address, 2, false) {
                    self.charge_load_cycles(bus, address, 2);
                    self.schedule_load(rt, (bus.read16(address) as i16 as i32) as u32);
                }
            }
            0x22 => {
                let aligned = address & !3;
                if self.data_bus_fault(bus, aligned, 4, false) {
                    return;
                }
                self.charge_load_cycles(bus, aligned, 4);
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
                } else if !self.data_bus_fault(bus, address, 4, false) {
                    self.charge_load_cycles(bus, address, 4);
                    self.schedule_load(rt, bus.read32(address));
                }
            }
            0x24 => {
                if !self.data_bus_fault(bus, address, 1, false) {
                    self.charge_load_cycles(bus, address, 1);
                    self.schedule_load(rt, u32::from(bus.read8(address)));
                }
            }
            0x25 => {
                if address & 1 != 0 {
                    self.take_exception(Exception::AddressLoad, Some(address));
                } else if !self.data_bus_fault(bus, address, 2, false) {
                    self.charge_load_cycles(bus, address, 2);
                    self.schedule_load(rt, u32::from(bus.read16(address)));
                }
            }
            0x26 => {
                let aligned = address & !3;
                if self.data_bus_fault(bus, aligned, 4, false) {
                    return;
                }
                self.charge_load_cycles(bus, aligned, 4);
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
        if self.data_address_error(address, true) {
            return;
        }
        match opcode {
            0x28 => {
                if !self.data_bus_fault(bus, address, 1, true) {
                    bus.store8(address, value);
                }
            }
            0x29 => {
                if address & 1 != 0 {
                    self.take_exception(Exception::AddressStore, Some(address));
                } else if !self.data_bus_fault(bus, address, 2, true) {
                    bus.store16(address, value);
                }
            }
            0x2a => {
                let aligned = address & !3;
                if self.data_bus_fault(bus, aligned, 4, true) {
                    return;
                }
                bus.store_left(address, value);
            }
            0x2b => {
                if address & 3 != 0 {
                    self.take_exception(Exception::AddressStore, Some(address));
                } else if !self.data_bus_fault(bus, address, 4, true) {
                    bus.write32(address, value);
                }
            }
            0x2e => {
                let aligned = address & !3;
                if self.data_bus_fault(bus, aligned, 4, true) {
                    return;
                }
                bus.store_right(address, value);
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
        match self.branch_target_next {
            Some(target) => {
                out.u8(1);
                out.u32(target);
            }
            None => out.u8(0),
        }
        match self.delay_slot_target {
            Some(target) => {
                out.u8(1);
                out.u32(target);
            }
            None => out.u8(0),
        }
        match self.pending_load {
            Some((reg, value)) => {
                out.u8(1);
                out.u8(reg);
                out.u32(value);
            }
            None => out.u8(0),
        }
        out.u64(self.cop2_busy_until);
        out.u32(self.pending_cop2_writes.len() as u32);
        for pending in &self.pending_cop2_writes {
            out.u8(pending.control as u8);
            out.u8(pending.register);
            out.u32(pending.value);
            out.u64(pending.ready_cycle);
        }
        match self.pending_cop2_enable {
            Some(pending) => {
                out.u8(1);
                out.u8(pending.previous as u8);
                out.u8(pending.enabled as u8);
                out.u64(pending.ready_cycle);
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
        self.branch_target_next = if input.u8()? != 0 {
            Some(input.u32()?)
        } else {
            None
        };
        self.delay_slot_target = if input.u8()? != 0 {
            Some(input.u32()?)
        } else {
            None
        };
        self.pending_load = if input.u8()? != 0 {
            let reg = input.u8()?;
            if reg >= 32 {
                return Err("invalid MIPS delayed-load register".into());
            }
            Some((reg, input.u32()?))
        } else {
            None
        };
        self.cop2_busy_until = input.u64()?;
        let pending_count = input.u32()? as usize;
        if pending_count > 32 {
            return Err("invalid MIPS COP2 pending-write count".into());
        }
        self.pending_cop2_writes.clear();
        for _ in 0..pending_count {
            let control = input.u8()? != 0;
            let register = input.u8()?;
            if register >= 32 {
                return Err("invalid MIPS COP2 pending-write register".into());
            }
            self.pending_cop2_writes.push(PendingCop2Write {
                control,
                register,
                value: input.u32()?,
                ready_cycle: input.u64()?,
            });
        }
        self.pending_cop2_enable = if input.u8()? != 0 {
            Some(PendingCop2Enable {
                previous: input.u8()? != 0,
                enabled: input.u8()? != 0,
                ready_cycle: input.u64()?,
            })
        } else {
            None
        };
        self.next_load = None;
        self.written_reg = None;
        self.exception_raised = false;
        self.step_cycles = 1;
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

    struct Cop2Ram {
        ram: Ram,
        data: [u32; 32],
        control: [u32; 32],
        command: u32,
        command_cycles: u32,
    }

    impl Cop2Ram {
        fn new() -> Self {
            Self {
                ram: Ram::new(),
                data: [0; 32],
                control: [0; 32],
                command: 0,
                command_cycles: 1,
            }
        }
    }

    impl MipsBus for Cop2Ram {
        fn read8(&mut self, address: u32) -> u8 {
            self.ram.read8(address)
        }
        fn write8(&mut self, address: u32, value: u8) {
            self.ram.write8(address, value);
        }
        fn cop2_present(&self) -> bool {
            true
        }
        fn cop2_read_data(&mut self, register: u8) -> u32 {
            self.data[usize::from(register & 31)]
        }
        fn cop2_write_data(&mut self, register: u8, value: u32) {
            self.data[usize::from(register & 31)] = value;
        }
        fn cop2_read_control(&mut self, register: u8) -> u32 {
            self.control[usize::from(register & 31)]
        }
        fn cop2_write_control(&mut self, register: u8, value: u32) {
            self.control[usize::from(register & 31)] = value;
        }
        fn cop2_command(&mut self, instruction: u32) -> u32 {
            self.command = instruction;
            self.command_cycles
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
    fn pending_load_retires_when_interrupt_or_following_exception_occurs() {
        let mut bus = Ram::new();
        bus.put32(0x100, 0x4433_2211);
        bus.put32(0, i(0x23, 0, 1, 0x100));
        bus.put32(4, 0);
        let mut cpu = cpu_at_zero();
        cpu.cop0[COP0_STATUS] = 1 | (1 << 10);
        cpu.step(&mut bus);
        assert_eq!(cpu.reg(1), 0);
        cpu.set_irq_line(2, true);
        cpu.step(&mut bus);
        assert_eq!(cpu.reg(1), 0x4433_2211);
        assert_eq!(
            (cpu.cop0[COP0_CAUSE] >> 2) & 0x1f,
            Exception::Interrupt as u32
        );

        bus.put32(4, 0x0000_000c);
        let mut cpu = cpu_at_zero();
        cpu.step(&mut bus);
        assert_eq!(cpu.reg(1), 0);
        cpu.step(&mut bus);
        assert_eq!(cpu.reg(1), 0x4433_2211);
        assert_eq!(
            (cpu.cop0[COP0_CAUSE] >> 2) & 0x1f,
            Exception::Syscall as u32
        );
    }

    #[test]
    fn unaligned_stores_merge_without_alignment_exception() {
        let mut bus = Ram::new();
        bus.put32(0x100, 0x1122_3344);
        bus.put32(0, i(0x2a, 0, 1, 0x101));
        bus.put32(4, i(0x2e, 0, 2, 0x106));
        let mut cpu = cpu_at_zero();
        cpu.regs[1] = 0xaabb_ccdd;
        cpu.regs[2] = 0x5566_7788;

        cpu.step(&mut bus);
        assert_eq!(bus.read32(0x100), 0x1122_aabb);
        bus.put32(0x104, 0x99aa_bbcc);
        cpu.step(&mut bus);
        assert_eq!(bus.read32(0x104), 0x7788_bbcc);
        assert_eq!((cpu.cop0[COP0_CAUSE] >> 2) & 0x1f, 0);
    }

    #[test]
    fn interrupt_before_jump_delay_slot_sets_bd_and_branch_epc() {
        let mut bus = Ram::new();
        bus.put32(0, r(1, 0, 0, 0, 0x08));
        bus.put32(4, i(0x09, 0, 2, 7));
        let mut cpu = cpu_at_zero();
        cpu.regs[1] = 0x20;
        cpu.cop0[COP0_STATUS] = 1 | (1 << 10);

        cpu.step(&mut bus);
        assert_eq!(cpu.pc, 4);
        cpu.set_irq_line(2, true);
        cpu.step(&mut bus);

        assert_eq!(
            (cpu.cop0[COP0_CAUSE] >> 2) & 0x1f,
            Exception::Interrupt as u32
        );
        assert_ne!(cpu.cop0[COP0_CAUSE] & (1 << 31), 0);
        assert_ne!(cpu.cop0[COP0_CAUSE] & (1 << 30), 0);
        assert_eq!(cpu.cop0[COP0_EPC], 0);
        assert_eq!(cpu.cop0[COP0_TAR], 0x20);
        assert_eq!(cpu.reg(2), 0);
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
        assert_ne!(cpu.cop0[COP0_CAUSE] & (1 << 30), 0);
        assert_eq!(cpu.cop0[COP0_TAR], 8);
        assert_eq!(cpu.cop0[COP0_EPC], 0);
        assert_eq!(
            (cpu.cop0[COP0_CAUSE] >> 2) & 0x1f,
            Exception::Syscall as u32
        );
        assert_eq!(cpu.pc, 0x8000_0080);
    }
    #[test]
    fn untaken_branch_delay_exception_leaves_bt_clear_and_tar_unchanged() {
        let mut bus = Ram::new();
        bus.put32(0, i(0x05, 0, 0, 1));
        bus.put32(4, 0x0000_000c);
        let mut cpu = cpu_at_zero();
        cpu.cop0[COP0_TAR] = 0x1234_5678;
        cpu.step(&mut bus);
        cpu.step(&mut bus);
        assert_ne!(cpu.cop0[COP0_CAUSE] & (1 << 31), 0);
        assert_eq!(cpu.cop0[COP0_CAUSE] & (1 << 30), 0);
        assert_eq!(cpu.cop0[COP0_TAR], 0x1234_5678);
    }

    #[test]
    fn user_mode_rejects_kernel_instruction_and_data_addresses() {
        let mut bus = Ram::new();
        let mut cpu = cpu_at_zero();
        cpu.pc = 0x8000_0000;
        cpu.next_pc = 0x8000_0004;
        cpu.cop0[COP0_STATUS] = 1 << 1;
        cpu.step(&mut bus);
        assert_eq!(
            (cpu.cop0[COP0_CAUSE] >> 2) & 0x1f,
            Exception::AddressLoad as u32
        );
        assert_eq!(cpu.cop0[COP0_BADVADDR], 0x8000_0000);

        bus.put32(0, i(0x23, 2, 1, 0));
        let mut cpu = cpu_at_zero();
        cpu.cop0[COP0_STATUS] = 1 << 1;
        cpu.regs[2] = 0x8000_0100;
        cpu.step(&mut bus);
        assert_eq!(
            (cpu.cop0[COP0_CAUSE] >> 2) & 0x1f,
            Exception::AddressLoad as u32
        );
        assert_eq!(cpu.cop0[COP0_BADVADDR], 0x8000_0100);

        bus.put32(0, i(0x2b, 2, 1, 0));
        let mut cpu = cpu_at_zero();
        cpu.cop0[COP0_STATUS] = 1 << 1;
        cpu.regs[2] = 0x8000_0200;
        cpu.step(&mut bus);
        assert_eq!(
            (cpu.cop0[COP0_CAUSE] >> 2) & 0x1f,
            Exception::AddressStore as u32
        );
        assert_eq!(cpu.cop0[COP0_BADVADDR], 0x8000_0200);
    }

    #[test]
    fn jalr_uses_encoded_destination_register_without_implicit_ra_fallback() {
        let mut bus = Ram::new();
        bus.put32(0, r(1, 0, 0, 0, 0x09));
        bus.put32(4, 0);
        bus.put32(0x20, 0);
        let mut cpu = cpu_at_zero();
        cpu.regs[1] = 0x20;
        cpu.regs[31] = 0xfeed_beef;

        cpu.step(&mut bus);
        assert_eq!(cpu.reg(31), 0xfeed_beef);
        cpu.step(&mut bus);
        assert_eq!(cpu.pc, 0x20);
    }

    #[test]
    fn unusable_coprocessors_report_their_number_in_cause_ce() {
        for (instruction, coprocessor) in [
            (0x11u32 << 26, 1u32),
            (0x13u32 << 26, 3u32),
            (0x30u32 << 26, 0u32),
            (0x31u32 << 26, 1u32),
            (0x33u32 << 26, 3u32),
            (0x38u32 << 26, 0u32),
            (0x39u32 << 26, 1u32),
            (0x3bu32 << 26, 3u32),
        ] {
            let mut bus = Ram::new();
            bus.put32(0, instruction);
            let mut cpu = cpu_at_zero();
            cpu.step(&mut bus);
            assert_eq!(
                (cpu.cop0[COP0_CAUSE] >> 2) & 0x1f,
                Exception::CoprocessorUnusable as u32
            );
            assert_eq!((cpu.cop0[COP0_CAUSE] >> 28) & 3, coprocessor);
        }
    }

    #[test]
    fn cop0_prid_cause_mask_and_invalid_registers_follow_ps1_rules() {
        let mut cpu = cpu_at_zero();
        assert_eq!(cpu.cop0[COP0_PRID], 2);
        cpu.cop0[COP0_CAUSE] = (1 << 31) | (2 << 28) | (1 << 10) | (9 << 2) | (1 << 9);
        let preserved = cpu.cop0[COP0_CAUSE];
        cpu.write_cop0(COP0_CAUSE, 1 << 8);
        assert_eq!(cpu.cop0[COP0_CAUSE], (preserved & !0x0000_0300) | (1 << 8));
        cpu.write_cop0(COP0_PRID, 0xffff_ffff);
        assert_eq!(cpu.cop0[COP0_PRID], 2);

        let mut bus = Ram::new();
        bus.put32(0, (0x10u32 << 26) | (1 << 16));
        cpu = cpu_at_zero();
        cpu.step(&mut bus);
        assert_eq!(
            (cpu.cop0[COP0_CAUSE] >> 2) & 0x1f,
            Exception::ReservedInstruction as u32
        );
    }

    #[test]
    fn cop0_access_in_user_mode_requires_cu0_for_real_registers() {
        let mfc0_status = (0x10u32 << 26) | (1 << 16) | ((COP0_STATUS as u32) << 11);
        let mut bus = Ram::new();
        bus.put32(0, mfc0_status);
        let mut cpu = cpu_at_zero();
        cpu.cop0[COP0_STATUS] = 1 << 1;
        cpu.step(&mut bus);
        assert_eq!(
            (cpu.cop0[COP0_CAUSE] >> 2) & 0x1f,
            Exception::CoprocessorUnusable as u32
        );
        assert_eq!((cpu.cop0[COP0_CAUSE] >> 28) & 3, 0);

        let mut cpu = cpu_at_zero();
        cpu.cop0[COP0_STATUS] = (1 << 1) | (1 << 28);
        cpu.step(&mut bus);
        assert_eq!(cpu.pending_load, Some((1, (1 << 1) | (1 << 28))));
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
    fn cop2_register_reads_use_load_delay_and_unusable_sets_ce() {
        let mut bus = Cop2Ram::new();
        bus.data[3] = 0x1234_5678;
        bus.ram.put32(0, 0x4801_1800);
        bus.ram.put32(4, r(1, 0, 2, 0, 0x21));
        bus.ram.put32(8, r(1, 0, 3, 0, 0x21));
        let mut cpu = cpu_at_zero();
        cpu.cop0[COP0_STATUS] = 1 << 30;
        cpu.step(&mut bus);
        cpu.step(&mut bus);
        assert_eq!(cpu.reg(2), 0);
        cpu.step(&mut bus);
        assert_eq!(cpu.reg(3), 0x1234_5678);

        let mut cpu = cpu_at_zero();
        cpu.cop0[COP0_STATUS] = 0;
        cpu.step(&mut bus);
        assert_eq!(
            (cpu.cop0[COP0_CAUSE] >> 2) & 0x1f,
            Exception::CoprocessorUnusable as u32
        );
        assert_eq!((cpu.cop0[COP0_CAUSE] >> 28) & 3, 2);
    }

    #[test]
    fn cop2_memory_transfers_and_commands_overlap_cpu_execution() {
        let mut bus = Cop2Ram::new();
        bus.ram.put32(0x100, 0xaabb_ccdd);
        bus.ram.put32(0, i(0x32, 0, 5, 0x100));
        bus.ram.put32(4, 0);
        bus.ram.put32(8, i(0x3a, 0, 5, 0x104));
        bus.ram.put32(12, 0x4a00_0001);
        bus.ram.put32(16, i(0x09, 0, 1, 7));
        bus.ram.put32(20, i(0x3a, 0, 5, 0x108));
        bus.command_cycles = 15;
        let mut cpu = cpu_at_zero();
        cpu.cop0[COP0_STATUS] = 1 << 30;

        assert_eq!(cpu.step(&mut bus), 1);
        assert_eq!(bus.data[5], 0);
        assert_eq!(cpu.step(&mut bus), 1);
        assert_eq!(cpu.step(&mut bus), 1);
        assert_eq!(bus.ram.read32(0x104), 0xaabb_ccdd);
        assert_eq!(cpu.step(&mut bus), 1);
        assert_eq!(bus.command, 0x4a00_0001);
        assert_eq!(cpu.cop2_busy_until, 18);
        assert_eq!(cpu.step(&mut bus), 1);
        assert_eq!(cpu.reg(1), 7);
        assert_eq!(cpu.step(&mut bus), 14);
        assert_eq!(bus.ram.read32(0x108), 0xaabb_ccdd);
        assert_eq!(cpu.cycles, 19);
    }

    #[test]
    fn cop2_enable_from_status_write_takes_two_cycles() {
        let mtc0_status = (0x10 << 26) | (0x04 << 21) | (1 << 16) | (12 << 11);

        let mut immediate_bus = Cop2Ram::new();
        immediate_bus.ram.put32(0, mtc0_status);
        immediate_bus.ram.put32(4, 0x4a00_0001);
        let mut immediate_cpu = cpu_at_zero();
        immediate_cpu.regs[1] = 1 << 30;
        immediate_cpu.step(&mut immediate_bus);
        immediate_cpu.step(&mut immediate_bus);
        assert_eq!(
            (immediate_cpu.cop0[COP0_CAUSE] >> 2) & 0x1f,
            Exception::CoprocessorUnusable as u32
        );

        let mut delayed_bus = Cop2Ram::new();
        delayed_bus.ram.put32(0, mtc0_status);
        delayed_bus.ram.put32(4, 0);
        delayed_bus.ram.put32(8, 0x4a00_0001);
        let mut delayed_cpu = cpu_at_zero();
        delayed_cpu.regs[1] = 1 << 30;
        delayed_cpu.step(&mut delayed_bus);
        delayed_cpu.step(&mut delayed_bus);
        delayed_cpu.step(&mut delayed_bus);
        assert_eq!(delayed_bus.command, 0x4a00_0001);
    }

    #[test]
    fn interrupt_on_cop2_command_still_issues_gte_work() {
        let mut bus = Cop2Ram::new();
        bus.ram.put32(0, 0x4a00_0001);
        bus.command_cycles = 15;
        let mut cpu = cpu_at_zero();
        cpu.cop0[COP0_STATUS] = (1 << 30) | (1 << 10) | 1;
        cpu.set_irq_line(2, true);

        assert_eq!(cpu.step(&mut bus), 1);
        assert_eq!(bus.command, 0x4a00_0001);
        assert_eq!(cpu.cop2_busy_until, 15);
        assert_eq!(cpu.cop0[COP0_EPC], 0);
        assert_eq!(
            (cpu.cop0[COP0_CAUSE] >> 2) & 0x1f,
            Exception::Interrupt as u32
        );
    }

    #[test]
    fn cop2_false_branch_is_taken_and_true_branch_is_not() {
        let mut bus = Cop2Ram::new();
        bus.ram.put32(0, 0x4900_0002);
        bus.ram.put32(4, i(0x09, 0, 1, 1));
        bus.ram.put32(8, i(0x09, 0, 1, 2));
        bus.ram.put32(12, i(0x09, 0, 2, 3));
        let mut cpu = cpu_at_zero();
        cpu.cop0[COP0_STATUS] = 1 << 30;
        cpu.step(&mut bus);
        cpu.step(&mut bus);
        cpu.step(&mut bus);
        assert_eq!(cpu.reg(1), 1);
        assert_eq!(cpu.reg(2), 3);
    }

    #[test]
    fn state_round_trip_preserves_pipeline_state() {
        let mut cpu = cpu_at_zero();
        cpu.regs[1] = 0x1234_5678;
        cpu.hi = 7;
        cpu.lo = 9;
        cpu.cycles = 5;
        cpu.branch_delay_next = true;
        cpu.branch_target_next = Some(0x8000_1234);
        cpu.delay_slot_target = Some(0x8000_5678);
        cpu.pending_load = Some((2, 0xaabb_ccdd));
        cpu.cop2_busy_until = 20;
        cpu.queue_cop2_write(false, 4, 0xdead_beef);
        cpu.pending_cop2_enable = Some(PendingCop2Enable {
            previous: false,
            enabled: true,
            ready_cycle: 9,
        });
        let mut out = StateWriter::new(crate::platform::PlatformId::PlayStation, 7);
        cpu.save(&mut out);
        let bytes = out.finish();
        let mut restored = MipsR3000::default();
        let mut input =
            StateReader::new(&bytes, crate::platform::PlatformId::PlayStation, 7).unwrap();
        restored.load_state(&mut input).unwrap();
        input.finish().unwrap();
        assert_eq!(restored.regs[1], 0x1234_5678);
        assert!(restored.branch_delay_next);
        assert_eq!(restored.branch_target_next, Some(0x8000_1234));
        assert_eq!(restored.delay_slot_target, Some(0x8000_5678));
        assert_eq!(restored.pending_load, Some((2, 0xaabb_ccdd)));
        assert_eq!(restored.cop2_busy_until, 20);
        assert_eq!(restored.pending_cop2_writes.len(), 1);
        assert_eq!(restored.pending_cop2_writes[0].register, 4);
        assert_eq!(restored.pending_cop2_writes[0].value, 0xdead_beef);
        assert_eq!(restored.pending_cop2_writes[0].ready_cycle, 7);
        let pending_enable = restored.pending_cop2_enable.expect("pending COP2 enable");
        assert!(!pending_enable.previous);
        assert!(pending_enable.enabled);
        assert_eq!(pending_enable.ready_cycle, 9);
        assert_eq!((restored.hi, restored.lo), (7, 9));
    }
}
