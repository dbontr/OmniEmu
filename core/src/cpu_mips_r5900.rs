use crate::state::{StateReader, StateWriter};

pub trait R5900Bus: Send {
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
    fn read64(&mut self, address: u32) -> u64 {
        u64::from_le_bytes([
            self.read8(address),
            self.read8(address.wrapping_add(1)),
            self.read8(address.wrapping_add(2)),
            self.read8(address.wrapping_add(3)),
            self.read8(address.wrapping_add(4)),
            self.read8(address.wrapping_add(5)),
            self.read8(address.wrapping_add(6)),
            self.read8(address.wrapping_add(7)),
        ])
    }
    fn read128(&mut self, address: u32) -> u128 {
        u128::from_le_bytes(std::array::from_fn(|offset| {
            self.read8(address.wrapping_add(offset as u32))
        }))
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
    fn write64(&mut self, address: u32, value: u64) {
        for (offset, byte) in value.to_le_bytes().into_iter().enumerate() {
            self.write8(address.wrapping_add(offset as u32), byte);
        }
    }
    fn write128(&mut self, address: u32, value: u128) {
        for (offset, byte) in value.to_le_bytes().into_iter().enumerate() {
            self.write8(address.wrapping_add(offset as u32), byte);
        }
    }
}

const COP0_INDEX: usize = 0;
const COP0_RANDOM: usize = 1;
const COP0_ENTRY_LO0: usize = 2;
const COP0_ENTRY_LO1: usize = 3;
const COP0_CONTEXT: usize = 4;
const COP0_PAGE_MASK: usize = 5;
const COP0_WIRED: usize = 6;
const COP0_BADVADDR: usize = 8;
const COP0_COUNT: usize = 9;
const COP0_ENTRY_HI: usize = 10;
const COP0_COMPARE: usize = 11;
const COP0_STATUS: usize = 12;
const COP0_CAUSE: usize = 13;
const COP0_EPC: usize = 14;
const COP0_PRID: usize = 15;
const COP0_CONFIG: usize = 16;
const COP0_XCONTEXT: usize = 20;
const COP0_ERROREPC: usize = 30;

const STATUS_IE: u64 = 1;
const STATUS_EXL: u64 = 1 << 1;
const STATUS_ERL: u64 = 1 << 2;
const STATUS_EIE: u64 = 1 << 16;
const STATUS_BEV: u64 = 1 << 22;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Exception {
    Interrupt = 0,
    TlbLoad = 2,
    TlbStore = 3,
    AddressLoad = 4,
    AddressStore = 5,
    Syscall = 8,
    Break = 9,
    ReservedInstruction = 10,
    CoprocessorUnusable = 11,
    Overflow = 12,
    Trap = 13,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
struct TlbEntry {
    page_mask: u32,
    entry_hi: u64,
    entry_lo0: u64,
    entry_lo1: u64,
}
#[derive(Debug, Clone)]
pub struct MipsR5900 {
    pub regs: [u128; 32],
    pub hi: u128,
    pub lo: u128,
    pub hi1: u64,
    pub lo1: u64,
    pub sa: u32,
    pub pc: u64,
    pub next_pc: u64,
    pub cop0: [u64; 32],
    pub cycles: u64,
    tlb: [TlbEntry; 48],
    current_pc: u64,
    branch_delay_next: bool,
    in_delay_slot: bool,
    exception_raised: bool,
    count_phase: bool,
    hle_direct_map: bool,
}

impl Default for MipsR5900 {
    fn default() -> Self {
        let mut cop0 = [0u64; 32];
        cop0[COP0_STATUS] = 0x7040_0004;
        cop0[COP0_PRID] = 0x0000_2e20;
        cop0[COP0_CONFIG] = 0x0000_0440;
        cop0[COP0_RANDOM] = 47;
        Self {
            regs: [0; 32],
            hi: 0,
            lo: 0,
            hi1: 0,
            lo1: 0,
            sa: 0,
            pc: 0xffff_ffff_bfc0_0000,
            next_pc: 0xffff_ffff_bfc0_0004,
            cop0,
            cycles: 0,
            tlb: [TlbEntry::default(); 48],
            current_pc: 0xffff_ffff_bfc0_0000,
            branch_delay_next: false,
            in_delay_slot: false,
            exception_raised: false,
            count_phase: false,
            hle_direct_map: false,
        }
    }
}
impl MipsR5900 {
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    pub fn reset_to(&mut self, pc: u64) {
        self.reset();
        self.pc = pc;
        self.next_pc = pc.wrapping_add(4);
        self.current_pc = pc;
        self.cop0[COP0_STATUS] &= !STATUS_ERL;
    }

    pub fn set_hle_direct_map(&mut self, enabled: bool) {
        self.hle_direct_map = enabled;
    }

    pub fn set_irq_line(&mut self, line: u8, asserted: bool) {
        if line >= 8 {
            return;
        }
        let bit = 8 + u32::from(line);
        if asserted {
            self.cop0[COP0_CAUSE] |= 1u64 << bit;
        } else {
            self.cop0[COP0_CAUSE] &= !(1u64 << bit);
        }
    }

    fn interrupts_pending(&self) -> bool {
        let status = self.cop0[COP0_STATUS];
        let cause = self.cop0[COP0_CAUSE];
        status & STATUS_IE != 0
            && status & STATUS_EIE != 0
            && status & (STATUS_EXL | STATUS_ERL) == 0
            && status & cause & 0x0000_ff00 != 0
    }

    fn reg(&self, index: u8) -> u64 {
        self.regs[usize::from(index)] as u64
    }
    fn reg128(&self, index: u8) -> u128 {
        self.regs[usize::from(index)]
    }
    fn write_reg(&mut self, index: u8, value: u64) {
        if index != 0 {
            let register = &mut self.regs[usize::from(index)];
            *register = (*register & (!0u128 << 64)) | u128::from(value);
        }
    }
    fn write_reg128(&mut self, index: u8, value: u128) {
        if index != 0 {
            self.regs[usize::from(index)] = value;
        }
    }
    fn write_reg_low32(&mut self, index: u8, value: u32) {
        if index != 0 {
            let register = &mut self.regs[usize::from(index)];
            *register = (*register & !u128::from(u32::MAX)) | u128::from(value);
        }
    }
    fn hi64(&self) -> u64 {
        self.hi as u64
    }
    fn lo64(&self) -> u64 {
        self.lo as u64
    }
    fn set_hi64(&mut self, value: u64) {
        self.hi = (self.hi & (!0u128 << 64)) | u128::from(value);
    }
    fn set_lo64(&mut self, value: u64) {
        self.lo = (self.lo & (!0u128 << 64)) | u128::from(value);
    }

    fn sign32(value: u32) -> u64 {
        value as i32 as i64 as u64
    }
    fn simm(instruction: u32) -> u64 {
        instruction as u16 as i16 as i64 as u64
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

    fn canonical32(address: u64) -> Option<u32> {
        let low = address as u32;
        let sign = if low & 0x8000_0000 != 0 {
            u64::MAX << 32
        } else {
            0
        };
        ((address & 0xffff_ffff_0000_0000) == sign).then_some(low)
    }

    fn direct_physical(address: u64) -> Option<u32> {
        let low = Self::canonical32(address)?;
        match low {
            0x7000_0000..=0x7000_3fff => Some(low),
            0x8000_0000..=0xbfff_ffff => Some(low & 0x1fff_ffff),
            _ => None,
        }
    }

    fn tlb_page_size(entry: &TlbEntry) -> u64 {
        u64::from((entry.page_mask | 0x1fff).wrapping_add(1))
    }
    fn translate(&mut self, address: u64, store: bool) -> Result<u32, Exception> {
        if self.hle_direct_map {
            if let Some(low) = Self::canonical32(address).filter(|low| *low <= 0x1fff_ffff) {
                return Ok(low);
            }
        }
        if let Some(physical) = Self::direct_physical(address) {
            return Ok(physical);
        }
        let Some(virtual32) = Self::canonical32(address) else {
            self.set_tlb_fault_state(address);
            return Err(if store {
                Exception::TlbStore
            } else {
                Exception::TlbLoad
            });
        };
        let asid = self.cop0[COP0_ENTRY_HI] as u8;
        for entry in self.tlb {
            let pair_size = Self::tlb_page_size(&entry);
            let mask = !(pair_size - 1);
            let vpn_match = (u64::from(virtual32) & mask) == (entry.entry_hi & 0xffff_ffff & mask);
            let global = entry.entry_lo0 & 1 != 0 && entry.entry_lo1 & 1 != 0;
            if !vpn_match || (!global && entry.entry_hi as u8 != asid) {
                continue;
            }
            let half = pair_size >> 1;
            let lo = if u64::from(virtual32) & half == 0 {
                entry.entry_lo0
            } else {
                entry.entry_lo1
            };
            if lo & 0x02 == 0 {
                self.set_tlb_fault_state(address);
                return Err(if store {
                    Exception::TlbStore
                } else {
                    Exception::TlbLoad
                });
            }
            if store && lo & 0x04 == 0 {
                self.set_tlb_fault_state(address);
                return Err(Exception::TlbStore);
            }
            let page_offset = u64::from(virtual32) & (half - 1);
            let pfn = (lo >> 6) << 12;
            return Ok(((pfn | page_offset) & 0x1fff_ffff) as u32);
        }
        self.set_tlb_fault_state(address);
        Err(if store {
            Exception::TlbStore
        } else {
            Exception::TlbLoad
        })
    }
    fn set_tlb_fault_state(&mut self, address: u64) {
        self.cop0[COP0_BADVADDR] = address;
        let virtual32 = address as u32;
        self.cop0[COP0_CONTEXT] =
            (self.cop0[COP0_CONTEXT] & 0xffff_ffff_ff80_000f) | (u64::from(virtual32 >> 13) << 4);
        self.cop0[COP0_XCONTEXT] =
            (self.cop0[COP0_XCONTEXT] & 0xffff_ffff_8000_000f) | (u64::from(virtual32 >> 13) << 4);
        self.cop0[COP0_ENTRY_HI] =
            (self.cop0[COP0_ENTRY_HI] & 0xff) | (u64::from(virtual32) & 0xffff_e000);
    }

    fn take_exception(&mut self, exception: Exception, bad_address: Option<u64>) {
        self.exception_raised = true;
        if let Some(address) = bad_address {
            self.cop0[COP0_BADVADDR] = address;
        }
        let status = self.cop0[COP0_STATUS];
        let mut cause = self.cop0[COP0_CAUSE] & 0x0000_ff00;
        cause |= (exception as u64) << 2;
        let epc = if self.in_delay_slot {
            cause |= 1u64 << 31;
            self.current_pc.wrapping_sub(4)
        } else {
            self.current_pc
        };
        self.cop0[COP0_CAUSE] = cause;
        if status & STATUS_EXL == 0 {
            self.cop0[COP0_EPC] = epc;
        }
        self.cop0[COP0_STATUS] = status | STATUS_EXL;
        let tlb_refill = matches!(exception, Exception::TlbLoad | Exception::TlbStore)
            && status & STATUS_EXL == 0;
        let vector = if status & STATUS_BEV != 0 {
            if tlb_refill {
                0xffff_ffff_bfc0_0200
            } else if exception == Exception::Interrupt {
                0xffff_ffff_bfc0_0400
            } else {
                0xffff_ffff_bfc0_0380
            }
        } else if tlb_refill {
            0xffff_ffff_8000_0000
        } else if exception == Exception::Interrupt {
            0xffff_ffff_8000_0200
        } else {
            0xffff_ffff_8000_0180
        };
        self.pc = vector;
        self.next_pc = vector.wrapping_add(4);
        self.branch_delay_next = false;
        self.in_delay_slot = false;
    }

    fn tick_count(&mut self) {
        self.count_phase = !self.count_phase;
        if self.count_phase {
            self.cop0[COP0_COUNT] = self.cop0[COP0_COUNT].wrapping_add(1) & 0xffff_ffff;
            if self.cop0[COP0_COUNT] as u32 == self.cop0[COP0_COMPARE] as u32 {
                self.cop0[COP0_CAUSE] |= 1 << 15;
            }
        }
        let wired = (self.cop0[COP0_WIRED] as usize).min(47);
        let random = self.cop0[COP0_RANDOM] as usize;
        self.cop0[COP0_RANDOM] = if random <= wired {
            47
        } else {
            (random - 1) as u64
        };
    }

    fn branch_target(&self, instruction: u32) -> u64 {
        self.pc
            .wrapping_add((instruction as u16 as i16 as i64 as u64) << 2)
    }
    pub fn step<B: R5900Bus>(&mut self, bus: &mut B) -> u32 {
        if self.interrupts_pending() {
            self.current_pc = self.pc;
            self.in_delay_slot = false;
            self.take_exception(Exception::Interrupt, None);
            self.finish_cycle();
            return 1;
        }
        self.current_pc = self.pc;
        self.in_delay_slot = self.branch_delay_next;
        self.branch_delay_next = false;
        if self.pc & 3 != 0 {
            self.take_exception(Exception::AddressLoad, Some(self.pc));
            self.finish_cycle();
            return 1;
        }
        let physical = match self.translate(self.pc, false) {
            Ok(address) => address,
            Err(exception) => {
                self.take_exception(exception, Some(self.pc));
                self.finish_cycle();
                return 1;
            }
        };
        let instruction = bus.read32(physical);
        self.pc = self.next_pc;
        self.next_pc = self.next_pc.wrapping_add(4);
        self.exception_raised = false;
        self.execute(bus, instruction);
        self.regs[0] = 0;
        self.finish_cycle();
        1
    }

    fn finish_cycle(&mut self) {
        self.cycles = self.cycles.wrapping_add(1);
        self.tick_count();
    }
    fn execute<B: R5900Bus>(&mut self, bus: &mut B, instruction: u32) {
        match instruction >> 26 {
            0x00 => self.execute_special(instruction),
            0x01 => self.execute_regimm(instruction),
            0x02 => self.jump(instruction, false),
            0x03 => self.jump(instruction, true),
            0x04 => self.branch(
                instruction,
                self.reg(Self::rs(instruction)) == self.reg(Self::rt(instruction)),
                false,
            ),
            0x05 => self.branch(
                instruction,
                self.reg(Self::rs(instruction)) != self.reg(Self::rt(instruction)),
                false,
            ),
            0x06 => self.branch(
                instruction,
                (self.reg(Self::rs(instruction)) as i64) <= 0,
                false,
            ),
            0x07 => self.branch(
                instruction,
                (self.reg(Self::rs(instruction)) as i64) > 0,
                false,
            ),
            0x08 => self.addi32(instruction, true),
            0x09 => self.addi32(instruction, false),
            0x0a => self.write_reg(
                Self::rt(instruction),
                ((self.reg(Self::rs(instruction)) as i64) < (Self::simm(instruction) as i64))
                    as u64,
            ),
            0x0b => self.write_reg(
                Self::rt(instruction),
                (self.reg(Self::rs(instruction)) < Self::simm(instruction)) as u64,
            ),
            0x0c => self.write_reg(
                Self::rt(instruction),
                self.reg(Self::rs(instruction)) & u64::from(instruction as u16),
            ),
            0x0d => self.write_reg(
                Self::rt(instruction),
                self.reg(Self::rs(instruction)) | u64::from(instruction as u16),
            ),
            0x0e => self.write_reg(
                Self::rt(instruction),
                self.reg(Self::rs(instruction)) ^ u64::from(instruction as u16),
            ),
            0x0f => self.write_reg(
                Self::rt(instruction),
                Self::sign32(u32::from(instruction as u16) << 16),
            ),
            0x10 => self.execute_cop0(instruction),
            0x11 => self.take_exception(Exception::CoprocessorUnusable, None),
            0x12 => self.take_exception(Exception::CoprocessorUnusable, None),
            0x14 => self.branch(
                instruction,
                self.reg(Self::rs(instruction)) == self.reg(Self::rt(instruction)),
                true,
            ),
            0x15 => self.branch(
                instruction,
                self.reg(Self::rs(instruction)) != self.reg(Self::rt(instruction)),
                true,
            ),
            0x16 => self.branch(
                instruction,
                (self.reg(Self::rs(instruction)) as i64) <= 0,
                true,
            ),
            0x17 => self.branch(
                instruction,
                (self.reg(Self::rs(instruction)) as i64) > 0,
                true,
            ),
            0x18 => self.daddi(instruction, true),
            0x19 => self.daddi(instruction, false),
            0x1a | 0x1b | 0x1e | 0x20..=0x27 | 0x37 => self.load(bus, instruction),
            0x1c => self.execute_mmi(instruction),
            0x1f | 0x28..=0x2e | 0x3f => self.store(bus, instruction),
            0x2f | 0x33 => {}
            0x31 | 0x39 => self.take_exception(Exception::CoprocessorUnusable, None),
            0x36 | 0x3e => self.take_exception(Exception::CoprocessorUnusable, None),
            _ => self.take_exception(Exception::ReservedInstruction, None),
        }
    }
    fn write32_result(&mut self, rd: u8, value: u32) {
        self.write_reg(rd, Self::sign32(value));
    }

    fn execute_special(&mut self, instruction: u32) {
        let rs = Self::rs(instruction);
        let rt = Self::rt(instruction);
        let rd = Self::rd(instruction);
        let sa = Self::shamt(instruction);
        let left = self.reg(rs);
        let right = self.reg(rt);
        match instruction & 0x3f {
            0x00 => self.write32_result(rd, (right as u32) << sa),
            0x02 => self.write32_result(rd, (right as u32) >> sa),
            0x03 => self.write32_result(rd, ((right as u32 as i32) >> sa) as u32),
            0x04 => self.write32_result(rd, (right as u32) << ((left as u32) & 31)),
            0x06 => self.write32_result(rd, (right as u32) >> ((left as u32) & 31)),
            0x07 => self.write32_result(rd, ((right as u32 as i32) >> ((left as u32) & 31)) as u32),
            0x08 => {
                self.next_pc = left;
                self.branch_delay_next = true;
            }
            0x09 => {
                self.write_reg(if rd == 0 { 31 } else { rd }, self.next_pc);
                self.next_pc = left;
                self.branch_delay_next = true;
            }
            0x0a => {
                if right == 0 {
                    self.write_reg(rd, left);
                }
            }
            0x0b => {
                if right != 0 {
                    self.write_reg(rd, left);
                }
            }
            0x0c => self.take_exception(Exception::Syscall, None),
            0x0d => self.take_exception(Exception::Break, None),
            0x0f => {}
            0x10 => self.write_reg(rd, self.hi64()),
            0x11 => self.set_hi64(left),
            0x12 => self.write_reg(rd, self.lo64()),
            0x13 => self.set_lo64(left),
            0x14 => self.write_reg(rd, right << (left & 63)),
            0x16 => self.write_reg(rd, right >> (left & 63)),
            0x17 => self.write_reg(rd, ((right as i64) >> (left & 63)) as u64),
            0x18 => {
                let value = i64::from(left as u32 as i32) * i64::from(right as u32 as i32);
                self.set_lo64(Self::sign32(value as u32));
                self.set_hi64(Self::sign32((value >> 32) as u32));
                self.write32_result(rd, value as u32);
            }
            0x19 => {
                let value = u64::from(left as u32) * u64::from(right as u32);
                self.set_lo64(Self::sign32(value as u32));
                self.set_hi64(Self::sign32((value >> 32) as u32));
                self.write32_result(rd, value as u32);
            }
            0x1a => self.div32(left as u32, right as u32, true),
            0x1b => self.div32(left as u32, right as u32, false),
            0x1c..=0x1f => self.take_exception(Exception::ReservedInstruction, None),
            0x20 => self.add32(rd, left as u32, right as u32, true),
            0x21 => self.add32(rd, left as u32, right as u32, false),
            0x22 => self.sub32(rd, left as u32, right as u32, true),
            0x23 => self.sub32(rd, left as u32, right as u32, false),
            0x24 => self.write_reg(rd, left & right),
            0x25 => self.write_reg(rd, left | right),
            0x26 => self.write_reg(rd, left ^ right),
            0x27 => self.write_reg(rd, !(left | right)),
            0x28 => self.write_reg(rd, u64::from(self.sa)),
            0x29 => self.sa = left as u32,
            0x2a => self.write_reg(rd, ((left as i64) < (right as i64)) as u64),
            0x2b => self.write_reg(rd, (left < right) as u64),
            0x2c => self.dadd(rd, left, right, true),
            0x2d => self.dadd(rd, left, right, false),
            0x2e => self.dsub(rd, left, right, true),
            0x2f => self.dsub(rd, left, right, false),
            0x30 => self.trap((left as i64) >= (right as i64)),
            0x31 => self.trap(left >= right),
            0x32 => self.trap((left as i64) < (right as i64)),
            0x33 => self.trap(left < right),
            0x34 => self.trap(left == right),
            0x36 => self.trap(left != right),
            0x38 => self.write_reg(rd, right << sa),
            0x3a => self.write_reg(rd, right >> sa),
            0x3b => self.write_reg(rd, ((right as i64) >> sa) as u64),
            0x3c => self.write_reg(rd, right << (sa + 32)),
            0x3e => self.write_reg(rd, right >> (sa + 32)),
            0x3f => self.write_reg(rd, ((right as i64) >> (sa + 32)) as u64),
            _ => self.take_exception(Exception::ReservedInstruction, None),
        }
    }

    fn trap(&mut self, condition: bool) {
        if condition {
            self.take_exception(Exception::Trap, None);
        }
    }

    fn add32(&mut self, rd: u8, left: u32, right: u32, trap: bool) {
        if trap {
            if let Some(value) = (left as i32).checked_add(right as i32) {
                self.write32_result(rd, value as u32);
            } else {
                self.take_exception(Exception::Overflow, None);
            }
        } else {
            self.write32_result(rd, left.wrapping_add(right));
        }
    }

    fn sub32(&mut self, rd: u8, left: u32, right: u32, trap: bool) {
        if trap {
            if let Some(value) = (left as i32).checked_sub(right as i32) {
                self.write32_result(rd, value as u32);
            } else {
                self.take_exception(Exception::Overflow, None);
            }
        } else {
            self.write32_result(rd, left.wrapping_sub(right));
        }
    }

    fn dadd(&mut self, rd: u8, left: u64, right: u64, trap: bool) {
        if trap {
            if let Some(value) = (left as i64).checked_add(right as i64) {
                self.write_reg(rd, value as u64);
            } else {
                self.take_exception(Exception::Overflow, None);
            }
        } else {
            self.write_reg(rd, left.wrapping_add(right));
        }
    }

    fn dsub(&mut self, rd: u8, left: u64, right: u64, trap: bool) {
        if trap {
            if let Some(value) = (left as i64).checked_sub(right as i64) {
                self.write_reg(rd, value as u64);
            } else {
                self.take_exception(Exception::Overflow, None);
            }
        } else {
            self.write_reg(rd, left.wrapping_sub(right));
        }
    }
    fn div32(&mut self, left: u32, right: u32, signed: bool) {
        let (lo, hi) = if signed {
            let dividend = left as i32;
            let divisor = right as i32;
            if divisor == 0 {
                (
                    Self::sign32(if dividend >= 0 { u32::MAX } else { 1 }),
                    Self::sign32(left),
                )
            } else if dividend == i32::MIN && divisor == -1 {
                (Self::sign32(i32::MIN as u32), 0)
            } else {
                (
                    Self::sign32((dividend / divisor) as u32),
                    Self::sign32((dividend % divisor) as u32),
                )
            }
        } else if let Some(quotient) = left.checked_div(right) {
            (Self::sign32(quotient), Self::sign32(left % right))
        } else {
            (Self::sign32(u32::MAX), Self::sign32(left))
        };
        self.set_lo64(lo);
        self.set_hi64(hi);
    }

    fn addi32(&mut self, instruction: u32, trap: bool) {
        let rt = Self::rt(instruction);
        self.add32(
            rt,
            self.reg(Self::rs(instruction)) as u32,
            Self::simm(instruction) as u32,
            trap,
        );
    }

    fn daddi(&mut self, instruction: u32, trap: bool) {
        let rt = Self::rt(instruction);
        self.dadd(
            rt,
            self.reg(Self::rs(instruction)),
            Self::simm(instruction),
            trap,
        );
    }
    fn branch(&mut self, instruction: u32, take: bool, likely: bool) {
        if take {
            self.next_pc = self.branch_target(instruction);
            self.branch_delay_next = true;
        } else if likely {
            self.pc = self.next_pc;
            self.next_pc = self.next_pc.wrapping_add(4);
            self.branch_delay_next = false;
        }
    }

    fn jump(&mut self, instruction: u32, link: bool) {
        if link {
            self.write_reg(31, self.next_pc);
        }
        let region = self.pc & 0xffff_ffff_f000_0000;
        self.next_pc = region | (u64::from(instruction & 0x03ff_ffff) << 2);
        self.branch_delay_next = true;
    }

    fn execute_regimm(&mut self, instruction: u32) {
        let raw = self.reg(Self::rs(instruction));
        let value = raw as i64;
        let immediate = Self::simm(instruction);
        let rt = Self::rt(instruction);
        let branch = match rt {
            0x00 => Some((value < 0, false, false)),
            0x01 => Some((value >= 0, false, false)),
            0x02 => Some((value < 0, true, false)),
            0x03 => Some((value >= 0, true, false)),
            0x08 => {
                self.trap(value >= immediate as i64);
                None
            }
            0x09 => {
                self.trap(raw >= immediate);
                None
            }
            0x0a => {
                self.trap(value < immediate as i64);
                None
            }
            0x0b => {
                self.trap(raw < immediate);
                None
            }
            0x0c => {
                self.trap(value == immediate as i64);
                None
            }
            0x0e => {
                self.trap(value != immediate as i64);
                None
            }
            0x10 => Some((value < 0, false, true)),
            0x11 => Some((value >= 0, false, true)),
            0x12 => Some((value < 0, true, true)),
            0x13 => Some((value >= 0, true, true)),
            0x18 => {
                self.sa = ((raw as u32 & 0x0f) ^ (instruction as u16 as u32 & 0x0f)) & 0x0f;
                None
            }
            0x19 => {
                self.sa = (((raw as u32 & 0x07) ^ (instruction as u16 as u32 & 0x07)) << 1) & 0x0f;
                None
            }
            _ => {
                self.take_exception(Exception::ReservedInstruction, None);
                return;
            }
        };
        if let Some((take, likely, link)) = branch {
            if link {
                self.write_reg(31, self.next_pc);
            }
            self.branch(instruction, take, likely);
        }
    }
    fn execute_cop0(&mut self, instruction: u32) {
        let rs = (instruction >> 21) & 31;
        let rt = Self::rt(instruction);
        let rd = usize::from(Self::rd(instruction));
        match rs {
            0x00 => self.write_reg(rt, Self::sign32(self.cop0[rd] as u32)),
            0x04 => self.write_cop0(rd, Self::sign32(self.reg(rt) as u32)),
            0x08 => self.take_exception(Exception::ReservedInstruction, None),
            0x10 => match instruction & 0x3f {
                0x01 => self.tlbr(),
                0x02 => self.tlbwi(),
                0x06 => self.tlbwr(),
                0x08 => self.tlbp(),
                0x18 => self.eret(),
                0x38 => self.cop0[COP0_STATUS] |= STATUS_EIE,
                0x39 => self.cop0[COP0_STATUS] &= !STATUS_EIE,
                _ => self.take_exception(Exception::ReservedInstruction, None),
            },
            _ => self.take_exception(Exception::CoprocessorUnusable, None),
        }
    }

    fn write_cop0(&mut self, rd: usize, value: u64) {
        let value = u64::from(value as u32);
        match rd {
            COP0_INDEX => self.cop0[rd] = value & 0x8000_003f,
            COP0_RANDOM => {}
            COP0_ENTRY_LO0 | COP0_ENTRY_LO1 => self.cop0[rd] = value & 0x3fff_ffff,
            COP0_CONTEXT => self.cop0[rd] = value,
            COP0_PAGE_MASK => self.cop0[rd] = value & 0x01ff_e000,
            COP0_WIRED => {
                self.cop0[rd] = value.min(47);
                self.cop0[COP0_RANDOM] = 47;
            }
            COP0_COUNT => self.cop0[rd] = value & 0xffff_ffff,
            COP0_ENTRY_HI => self.cop0[rd] = value & 0xffff_ffff_ffff_e0ff,
            COP0_COMPARE => {
                self.cop0[rd] = value & 0xffff_ffff;
                self.cop0[COP0_CAUSE] &= !(1 << 15);
            }
            COP0_CAUSE => {
                let pending = self.cop0[rd] & !0x0300;
                self.cop0[rd] = pending | (value & 0x0300);
            }
            COP0_STATUS => self.cop0[rd] = value & 0xff57_ffff,
            COP0_EPC | COP0_ERROREPC => self.cop0[rd] = value,
            COP0_CONFIG => {
                self.cop0[rd] = (self.cop0[rd] & !0x0000_0007) | (value & 0x0000_0007);
            }
            COP0_PRID | COP0_BADVADDR => {}
            _ => self.cop0[rd] = value,
        }
    }

    fn tlb_index(&self) -> usize {
        (self.cop0[COP0_INDEX] as usize) % 48
    }

    fn write_tlb_at(&mut self, index: usize) {
        self.tlb[index % 48] = TlbEntry {
            page_mask: self.cop0[COP0_PAGE_MASK] as u32,
            entry_hi: self.cop0[COP0_ENTRY_HI],
            entry_lo0: self.cop0[COP0_ENTRY_LO0],
            entry_lo1: self.cop0[COP0_ENTRY_LO1],
        };
    }

    fn tlbwi(&mut self) {
        self.write_tlb_at(self.tlb_index());
    }

    fn tlbwr(&mut self) {
        self.write_tlb_at((self.cop0[COP0_RANDOM] as usize).min(47));
    }
    fn tlbr(&mut self) {
        let entry = self.tlb[self.tlb_index()];
        self.cop0[COP0_PAGE_MASK] = u64::from(entry.page_mask);
        self.cop0[COP0_ENTRY_HI] = entry.entry_hi;
        self.cop0[COP0_ENTRY_LO0] = entry.entry_lo0;
        self.cop0[COP0_ENTRY_LO1] = entry.entry_lo1;
    }

    fn tlbp(&mut self) {
        let target_hi = self.cop0[COP0_ENTRY_HI];
        let target_asid = target_hi as u8;
        self.cop0[COP0_INDEX] = 0x8000_0000;
        for (index, entry) in self.tlb.iter().copied().enumerate() {
            let pair_size = Self::tlb_page_size(&entry);
            let mask = !(pair_size - 1);
            let global = entry.entry_lo0 & 1 != 0 && entry.entry_lo1 & 1 != 0;
            if (entry.entry_hi & mask) == (target_hi & mask)
                && (global || entry.entry_hi as u8 == target_asid)
            {
                self.cop0[COP0_INDEX] = index as u64;
                break;
            }
        }
    }

    fn eret(&mut self) {
        if self.cop0[COP0_STATUS] & STATUS_ERL != 0 {
            self.pc = self.cop0[COP0_ERROREPC];
            self.cop0[COP0_STATUS] &= !STATUS_ERL;
        } else {
            self.pc = self.cop0[COP0_EPC];
            self.cop0[COP0_STATUS] &= !STATUS_EXL;
        }
        self.next_pc = self.pc.wrapping_add(4);
        self.branch_delay_next = false;
        self.in_delay_slot = false;
    }
    fn effective_address(&self, instruction: u32) -> u64 {
        self.reg(Self::rs(instruction))
            .wrapping_add(Self::simm(instruction))
    }

    fn physical(&mut self, address: u64, align: u64, store: bool) -> Option<u32> {
        if address & (align - 1) != 0 {
            self.take_exception(
                if store {
                    Exception::AddressStore
                } else {
                    Exception::AddressLoad
                },
                Some(address),
            );
            return None;
        }
        match self.translate(address, store) {
            Ok(physical) => Some(physical),
            Err(exception) => {
                self.take_exception(exception, Some(address));
                None
            }
        }
    }

    fn load<B: R5900Bus>(&mut self, bus: &mut B, instruction: u32) {
        let opcode = instruction >> 26;
        let rt = Self::rt(instruction);
        let address = self.effective_address(instruction);
        match opcode {
            0x1e => {
                let aligned = address & !0x0f;
                let Some(physical) = self.physical(aligned, 16, false) else {
                    return;
                };
                self.write_reg128(rt, bus.read128(physical));
            }
            0x20 | 0x24 => {
                let Some(physical) = self.physical(address, 1, false) else {
                    return;
                };
                let byte = bus.read8(physical);
                self.write_reg(
                    rt,
                    if opcode == 0x20 {
                        byte as i8 as i64 as u64
                    } else {
                        u64::from(byte)
                    },
                );
            }
            0x21 | 0x25 => {
                let Some(physical) = self.physical(address, 2, false) else {
                    return;
                };
                let word = bus.read16(physical);
                self.write_reg(
                    rt,
                    if opcode == 0x21 {
                        word as i16 as i64 as u64
                    } else {
                        u64::from(word)
                    },
                );
            }
            0x22 => self.load_word_left(bus, rt, address),
            0x23 | 0x27 => {
                let Some(physical) = self.physical(address, 4, false) else {
                    return;
                };
                let word = bus.read32(physical);
                self.write_reg(
                    rt,
                    if opcode == 0x27 {
                        u64::from(word)
                    } else {
                        Self::sign32(word)
                    },
                );
            }
            0x26 => self.load_word_right(bus, rt, address),
            0x1a => self.load_double_left(bus, rt, address),
            0x1b => self.load_double_right(bus, rt, address),
            0x37 => {
                let Some(physical) = self.physical(address, 8, false) else {
                    return;
                };
                self.write_reg(rt, bus.read64(physical));
            }
            _ => self.take_exception(Exception::ReservedInstruction, None),
        }
    }

    fn load_word_left<B: R5900Bus>(&mut self, bus: &mut B, rt: u8, address: u64) {
        let aligned = address & !3;
        let Some(physical) = self.physical(aligned, 4, false) else {
            return;
        };
        let memory = bus.read32(physical);
        let lane = (address & 3) as usize;
        let masks = [0x00ff_ffff, 0x0000_ffff, 0x0000_00ff, 0];
        let shifts = [24, 16, 8, 0];
        let value = (self.reg(rt) as u32 & masks[lane]) | memory.wrapping_shl(shifts[lane]);
        self.write_reg(rt, Self::sign32(value));
    }
    fn load_word_right<B: R5900Bus>(&mut self, bus: &mut B, rt: u8, address: u64) {
        let aligned = address & !3;
        let Some(physical) = self.physical(aligned, 4, false) else {
            return;
        };
        let memory = bus.read32(physical);
        let lane = (address & 3) as usize;
        let masks = [0, 0xff00_0000, 0xffff_0000, 0xffff_ff00];
        let shifts = [0, 8, 16, 24];
        let value = (self.reg(rt) as u32 & masks[lane]) | (memory >> shifts[lane]);
        if lane == 0 {
            self.write_reg(rt, Self::sign32(value));
        } else {
            self.write_reg_low32(rt, value);
        }
    }

    fn load_double_left<B: R5900Bus>(&mut self, bus: &mut B, rt: u8, address: u64) {
        let aligned = address & !7;
        let Some(physical) = self.physical(aligned, 8, false) else {
            return;
        };
        let memory = bus.read64(physical);
        let lane = (address & 7) as usize;
        let masks = [
            0x00ff_ffff_ffff_ffff,
            0x0000_ffff_ffff_ffff,
            0x0000_00ff_ffff_ffff,
            0x0000_0000_ffff_ffff,
            0x0000_0000_00ff_ffff,
            0x0000_0000_0000_ffff,
            0x0000_0000_0000_00ff,
            0,
        ];
        let shifts = [56, 48, 40, 32, 24, 16, 8, 0];
        self.write_reg(
            rt,
            (self.reg(rt) & masks[lane]) | memory.wrapping_shl(shifts[lane]),
        );
    }

    fn load_double_right<B: R5900Bus>(&mut self, bus: &mut B, rt: u8, address: u64) {
        let aligned = address & !7;
        let Some(physical) = self.physical(aligned, 8, false) else {
            return;
        };
        let memory = bus.read64(physical);
        let lane = (address & 7) as usize;
        let masks = [
            0,
            0xff00_0000_0000_0000,
            0xffff_0000_0000_0000,
            0xffff_ff00_0000_0000,
            0xffff_ffff_0000_0000,
            0xffff_ffff_ff00_0000,
            0xffff_ffff_ffff_0000,
            0xffff_ffff_ffff_ff00,
        ];
        let shifts = [0, 8, 16, 24, 32, 40, 48, 56];
        self.write_reg(rt, (self.reg(rt) & masks[lane]) | (memory >> shifts[lane]));
    }
    fn store<B: R5900Bus>(&mut self, bus: &mut B, instruction: u32) {
        let opcode = instruction >> 26;
        let rt = Self::rt(instruction);
        let address = self.effective_address(instruction);
        let value = self.reg(rt);
        match opcode {
            0x1f => {
                let aligned = address & !0x0f;
                let Some(physical) = self.physical(aligned, 16, true) else {
                    return;
                };
                bus.write128(physical, self.reg128(rt));
            }
            0x28 => {
                let Some(physical) = self.physical(address, 1, true) else {
                    return;
                };
                bus.write8(physical, value as u8);
            }
            0x29 => {
                let Some(physical) = self.physical(address, 2, true) else {
                    return;
                };
                bus.write16(physical, value as u16);
            }
            0x2a => self.store_word_left(bus, address, value as u32),
            0x2b => {
                let Some(physical) = self.physical(address, 4, true) else {
                    return;
                };
                bus.write32(physical, value as u32);
            }
            0x2c => self.store_double_left(bus, address, value),
            0x2d => self.store_double_right(bus, address, value),
            0x2e => self.store_word_right(bus, address, value as u32),
            0x3f => {
                let Some(physical) = self.physical(address, 8, true) else {
                    return;
                };
                bus.write64(physical, value);
            }
            _ => self.take_exception(Exception::ReservedInstruction, None),
        }
    }

    fn store_word_left<B: R5900Bus>(&mut self, bus: &mut B, address: u64, value: u32) {
        let aligned = address & !3;
        let Some(physical) = self.physical(aligned, 4, true) else {
            return;
        };
        let memory = bus.read32(physical);
        let lane = (address & 3) as usize;
        let masks = [0xffff_ff00, 0xffff_0000, 0xff00_0000, 0];
        let shifts = [24, 16, 8, 0];
        bus.write32(physical, (memory & masks[lane]) | (value >> shifts[lane]));
    }

    fn store_word_right<B: R5900Bus>(&mut self, bus: &mut B, address: u64, value: u32) {
        let aligned = address & !3;
        let Some(physical) = self.physical(aligned, 4, true) else {
            return;
        };
        let memory = bus.read32(physical);
        let lane = (address & 3) as usize;
        let masks = [0, 0x0000_00ff, 0x0000_ffff, 0x00ff_ffff];
        let shifts = [0, 8, 16, 24];
        bus.write32(
            physical,
            (memory & masks[lane]) | value.wrapping_shl(shifts[lane]),
        );
    }
    fn store_double_left<B: R5900Bus>(&mut self, bus: &mut B, address: u64, value: u64) {
        let aligned = address & !7;
        let Some(physical) = self.physical(aligned, 8, true) else {
            return;
        };
        let memory = bus.read64(physical);
        let lane = (address & 7) as usize;
        let masks = [
            0xffff_ffff_ffff_ff00,
            0xffff_ffff_ffff_0000,
            0xffff_ffff_ff00_0000,
            0xffff_ffff_0000_0000,
            0xffff_ff00_0000_0000,
            0xffff_0000_0000_0000,
            0xff00_0000_0000_0000,
            0,
        ];
        let shifts = [56, 48, 40, 32, 24, 16, 8, 0];
        bus.write64(physical, (memory & masks[lane]) | (value >> shifts[lane]));
    }

    fn store_double_right<B: R5900Bus>(&mut self, bus: &mut B, address: u64, value: u64) {
        let aligned = address & !7;
        let Some(physical) = self.physical(aligned, 8, true) else {
            return;
        };
        let memory = bus.read64(physical);
        let lane = (address & 7) as usize;
        let masks = [
            0,
            0x0000_0000_0000_00ff,
            0x0000_0000_0000_ffff,
            0x0000_0000_00ff_ffff,
            0x0000_0000_ffff_ffff,
            0x0000_00ff_ffff_ffff,
            0x0000_ffff_ffff_ffff,
            0x00ff_ffff_ffff_ffff,
        ];
        let shifts = [0, 8, 16, 24, 32, 40, 48, 56];
        bus.write64(
            physical,
            (memory & masks[lane]) | value.wrapping_shl(shifts[lane]),
        );
    }

    pub fn save(&self, out: &mut StateWriter) {
        for value in self.regs {
            out.u128(value);
        }
        out.u128(self.hi);
        out.u128(self.lo);
        out.u64(self.hi1);
        out.u64(self.lo1);
        out.u32(self.sa);
        out.u64(self.pc);
        out.u64(self.next_pc);
        for value in self.cop0 {
            out.u64(value);
        }
        out.u64(self.cycles);
        for entry in self.tlb {
            out.u32(entry.page_mask);
            out.u64(entry.entry_hi);
            out.u64(entry.entry_lo0);
            out.u64(entry.entry_lo1);
        }
        out.u64(self.current_pc);
        out.u8(u8::from(self.branch_delay_next));
        out.u8(u8::from(self.in_delay_slot));
        out.u8(u8::from(self.exception_raised));
        out.u8(u8::from(self.count_phase));
        out.u8(u8::from(self.hle_direct_map));
    }

    pub fn load_state(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        for value in &mut self.regs {
            *value = input.u128()?;
        }
        self.hi = input.u128()?;
        self.lo = input.u128()?;
        self.hi1 = input.u64()?;
        self.lo1 = input.u64()?;
        self.sa = input.u32()? & 0x0f;
        self.pc = input.u64()?;
        self.next_pc = input.u64()?;
        for value in &mut self.cop0 {
            *value = input.u64()?;
        }
        self.cycles = input.u64()?;
        for entry in &mut self.tlb {
            entry.page_mask = input.u32()? & 0x01ff_e000;
            entry.entry_hi = input.u64()?;
            entry.entry_lo0 = input.u64()? & 0x3fff_ffff;
            entry.entry_lo1 = input.u64()? & 0x3fff_ffff;
        }
        self.current_pc = input.u64()?;
        self.branch_delay_next = input.u8()? != 0;
        self.in_delay_slot = input.u8()? != 0;
        self.exception_raised = input.u8()? != 0;
        self.count_phase = input.u8()? != 0;
        self.hle_direct_map = input.u8()? != 0;
        self.regs[0] = 0;
        Ok(())
    }
}

impl MipsR5900 {
    fn lane8(value: u128, lane: usize) -> u8 {
        (value >> (lane * 8)) as u8
    }
    fn lane16(value: u128, lane: usize) -> u16 {
        (value >> (lane * 16)) as u16
    }
    fn lane32(value: u128, lane: usize) -> u32 {
        (value >> (lane * 32)) as u32
    }
    fn with_lane8(output: &mut u128, lane: usize, value: u8) {
        *output |= u128::from(value) << (lane * 8);
    }
    fn with_lane16(output: &mut u128, lane: usize, value: u16) {
        *output |= u128::from(value) << (lane * 16);
    }
    fn with_lane32(output: &mut u128, lane: usize, value: u32) {
        *output |= u128::from(value) << (lane * 32);
    }

    fn execute_mmi(&mut self, instruction: u32) {
        let function = instruction & 0x3f;
        let rs = Self::rs(instruction);
        let rt = Self::rt(instruction);
        let rd = Self::rd(instruction);
        match function {
            0x00 => self.madd(rd, rs, rt, true, false),
            0x01 => self.madd(rd, rs, rt, false, false),
            0x08 => self.execute_mmi0(instruction),
            0x09 => self.execute_mmi2(instruction),
            0x10 => self.write_reg(rd, self.hi1),
            0x11 => self.hi1 = self.reg(rs),
            0x12 => self.write_reg(rd, self.lo1),
            0x13 => self.lo1 = self.reg(rs),
            0x18 => self.mult1(rd, rs, rt, true),
            0x19 => self.mult1(rd, rs, rt, false),
            0x1a => self.div1(rs, rt, true),
            0x1b => self.div1(rs, rt, false),
            0x20 => self.madd(rd, rs, rt, true, true),
            0x21 => self.madd(rd, rs, rt, false, true),
            0x28 => self.execute_mmi1(instruction),
            0x29 => self.execute_mmi3(instruction),
            0x34 | 0x36 | 0x37 | 0x3c | 0x3e | 0x3f => self.mmi_shift(instruction),
            _ => self.take_exception(Exception::ReservedInstruction, None),
        }
    }

    fn madd(&mut self, rd: u8, rs: u8, rt: u8, signed: bool, bank1: bool) {
        let left = self.reg(rs) as u32;
        let right = self.reg(rt) as u32;
        let (old_hi, old_lo) = if bank1 {
            (self.hi1, self.lo1)
        } else {
            (self.hi64(), self.lo64())
        };
        let accumulator = (u64::from(old_hi as u32) << 32) | u64::from(old_lo as u32);
        let value = if signed {
            (accumulator as i64).wrapping_add(i64::from(left as i32) * i64::from(right as i32))
                as u64
        } else {
            accumulator.wrapping_add(u64::from(left) * u64::from(right))
        };
        let lo = Self::sign32(value as u32);
        let hi = Self::sign32((value >> 32) as u32);
        if bank1 {
            self.lo1 = lo;
            self.hi1 = hi;
        } else {
            self.set_lo64(lo);
            self.set_hi64(hi);
        }
        self.write_reg(rd, lo);
    }

    fn mult1(&mut self, rd: u8, rs: u8, rt: u8, signed: bool) {
        let left = self.reg(rs) as u32;
        let right = self.reg(rt) as u32;
        let value = if signed {
            (i64::from(left as i32) * i64::from(right as i32)) as u64
        } else {
            u64::from(left) * u64::from(right)
        };
        self.lo1 = Self::sign32(value as u32);
        self.hi1 = Self::sign32((value >> 32) as u32);
        self.write_reg(rd, self.lo1);
    }

    fn div1(&mut self, rs: u8, rt: u8, signed: bool) {
        let left = self.reg(rs) as u32;
        let right = self.reg(rt) as u32;
        let (lo, hi) = if signed {
            let dividend = left as i32;
            let divisor = right as i32;
            if divisor == 0 {
                (
                    Self::sign32(if dividend < 0 { 1 } else { u32::MAX }),
                    Self::sign32(left),
                )
            } else if dividend == i32::MIN && divisor == -1 {
                (Self::sign32(i32::MIN as u32), 0)
            } else {
                (
                    Self::sign32((dividend / divisor) as u32),
                    Self::sign32((dividend % divisor) as u32),
                )
            }
        } else if let Some(quotient) = left.checked_div(right) {
            (Self::sign32(quotient), Self::sign32(left % right))
        } else {
            (Self::sign32(u32::MAX), Self::sign32(left))
        };
        self.lo1 = lo;
        self.hi1 = hi;
    }

    fn execute_mmi0(&mut self, instruction: u32) {
        let sub = (instruction >> 6) & 0x1f;
        let rs = self.reg128(Self::rs(instruction));
        let rt = self.reg128(Self::rt(instruction));
        let rd = Self::rd(instruction);
        let mut out = 0u128;
        match sub {
            0x00..=0x03 => {
                for lane in 0..4 {
                    let a = Self::lane32(rs, lane);
                    let b = Self::lane32(rt, lane);
                    let value = match sub {
                        0x00 => a.wrapping_add(b),
                        0x01 => a.wrapping_sub(b),
                        0x02 => {
                            if (a as i32) > (b as i32) {
                                u32::MAX
                            } else {
                                0
                            }
                        }
                        _ => {
                            if (a as i32) > (b as i32) {
                                a
                            } else {
                                b
                            }
                        }
                    };
                    Self::with_lane32(&mut out, lane, value);
                }
            }
            0x04..=0x07 => {
                for lane in 0..8 {
                    let a = Self::lane16(rs, lane);
                    let b = Self::lane16(rt, lane);
                    let value = match sub {
                        0x04 => a.wrapping_add(b),
                        0x05 => a.wrapping_sub(b),
                        0x06 => {
                            if (a as i16) > (b as i16) {
                                u16::MAX
                            } else {
                                0
                            }
                        }
                        _ => {
                            if (a as i16) > (b as i16) {
                                a
                            } else {
                                b
                            }
                        }
                    };
                    Self::with_lane16(&mut out, lane, value);
                }
            }
            0x08..=0x0a => {
                for lane in 0..16 {
                    let a = Self::lane8(rs, lane);
                    let b = Self::lane8(rt, lane);
                    let value = match sub {
                        0x08 => a.wrapping_add(b),
                        0x09 => a.wrapping_sub(b),
                        _ => {
                            if (a as i8) > (b as i8) {
                                u8::MAX
                            } else {
                                0
                            }
                        }
                    };
                    Self::with_lane8(&mut out, lane, value);
                }
            }
            0x12 => {
                for lane in 0..2 {
                    Self::with_lane32(&mut out, lane * 2, Self::lane32(rt, lane));
                    Self::with_lane32(&mut out, lane * 2 + 1, Self::lane32(rs, lane));
                }
            }
            0x13 => {
                for lane in 0..2 {
                    Self::with_lane32(&mut out, lane, Self::lane32(rt, lane * 2));
                    Self::with_lane32(&mut out, lane + 2, Self::lane32(rs, lane * 2));
                }
            }
            0x16 => {
                for lane in 0..4 {
                    Self::with_lane16(&mut out, lane * 2, Self::lane16(rt, lane));
                    Self::with_lane16(&mut out, lane * 2 + 1, Self::lane16(rs, lane));
                }
            }
            0x17 => {
                for lane in 0..4 {
                    Self::with_lane16(&mut out, lane, Self::lane16(rt, lane * 2));
                    Self::with_lane16(&mut out, lane + 4, Self::lane16(rs, lane * 2));
                }
            }
            0x1a => {
                for lane in 0..8 {
                    Self::with_lane8(&mut out, lane * 2, Self::lane8(rt, lane));
                    Self::with_lane8(&mut out, lane * 2 + 1, Self::lane8(rs, lane));
                }
            }
            0x1b => {
                for lane in 0..8 {
                    Self::with_lane8(&mut out, lane, Self::lane8(rt, lane * 2));
                    Self::with_lane8(&mut out, lane + 8, Self::lane8(rs, lane * 2));
                }
            }
            _ => {
                self.take_exception(Exception::ReservedInstruction, None);
                return;
            }
        }
        self.write_reg128(rd, out);
    }

    fn execute_mmi1(&mut self, instruction: u32) {
        let sub = (instruction >> 6) & 0x1f;
        let rs = self.reg128(Self::rs(instruction));
        let rt = self.reg128(Self::rt(instruction));
        let rd = Self::rd(instruction);
        let mut out = 0u128;
        match sub {
            0x02 | 0x03 => {
                for lane in 0..4 {
                    let a = Self::lane32(rs, lane);
                    let b = Self::lane32(rt, lane);
                    let value = if sub == 0x02 {
                        if a == b {
                            u32::MAX
                        } else {
                            0
                        }
                    } else if (a as i32) < (b as i32) {
                        a
                    } else {
                        b
                    };
                    Self::with_lane32(&mut out, lane, value);
                }
            }
            0x06 | 0x07 => {
                for lane in 0..8 {
                    let a = Self::lane16(rs, lane);
                    let b = Self::lane16(rt, lane);
                    let value = if sub == 0x06 {
                        if a == b {
                            u16::MAX
                        } else {
                            0
                        }
                    } else if (a as i16) < (b as i16) {
                        a
                    } else {
                        b
                    };
                    Self::with_lane16(&mut out, lane, value);
                }
            }
            0x0a => {
                for lane in 0..16 {
                    Self::with_lane8(
                        &mut out,
                        lane,
                        if Self::lane8(rs, lane) == Self::lane8(rt, lane) {
                            u8::MAX
                        } else {
                            0
                        },
                    );
                }
            }
            0x12 => {
                for lane in 0..2 {
                    Self::with_lane32(&mut out, lane * 2, Self::lane32(rt, lane + 2));
                    Self::with_lane32(&mut out, lane * 2 + 1, Self::lane32(rs, lane + 2));
                }
            }
            0x16 => {
                for lane in 0..4 {
                    Self::with_lane16(&mut out, lane * 2, Self::lane16(rt, lane + 4));
                    Self::with_lane16(&mut out, lane * 2 + 1, Self::lane16(rs, lane + 4));
                }
            }
            0x1a => {
                for lane in 0..8 {
                    Self::with_lane8(&mut out, lane * 2, Self::lane8(rt, lane + 8));
                    Self::with_lane8(&mut out, lane * 2 + 1, Self::lane8(rs, lane + 8));
                }
            }
            0x1b => {
                let shift = (self.sa & 0x0f) * 8;
                out = if shift == 0 {
                    rt
                } else {
                    (rt >> shift) | (rs << (128 - shift))
                };
            }
            _ => {
                self.take_exception(Exception::ReservedInstruction, None);
                return;
            }
        }
        self.write_reg128(rd, out);
    }

    fn execute_mmi2(&mut self, instruction: u32) {
        let sub = (instruction >> 6) & 0x1f;
        let rs = self.reg128(Self::rs(instruction));
        let rt = self.reg128(Self::rt(instruction));
        let rd = Self::rd(instruction);
        match sub {
            0x0e => self.write_reg128(rd, (rs << 64) | u128::from(rt as u64)),
            0x12 => self.write_reg128(rd, rs & rt),
            0x13 => self.write_reg128(rd, rs ^ rt),
            _ => self.take_exception(Exception::ReservedInstruction, None),
        }
    }

    fn execute_mmi3(&mut self, instruction: u32) {
        let sub = (instruction >> 6) & 0x1f;
        let rs = self.reg128(Self::rs(instruction));
        let rt = self.reg128(Self::rt(instruction));
        let rd = Self::rd(instruction);
        match sub {
            0x0e => self.write_reg128(rd, (rs >> 64) | (rt & (!0u128 << 64))),
            0x12 => self.write_reg128(rd, rs | rt),
            0x13 => self.write_reg128(rd, !(rs | rt)),
            _ => self.take_exception(Exception::ReservedInstruction, None),
        }
    }

    fn mmi_shift(&mut self, instruction: u32) {
        let function = instruction & 0x3f;
        let rt = self.reg128(Self::rt(instruction));
        let rd = Self::rd(instruction);
        let amount = (instruction >> 6) & 0x1f;
        let mut out = 0u128;
        if function < 0x3c {
            let shift = amount & 0x0f;
            for lane in 0..8 {
                let value = Self::lane16(rt, lane);
                let result = match function {
                    0x34 => value.wrapping_shl(shift),
                    0x36 => value >> shift,
                    _ => ((value as i16) >> shift) as u16,
                };
                Self::with_lane16(&mut out, lane, result);
            }
        } else {
            let shift = amount & 0x1f;
            for lane in 0..4 {
                let value = Self::lane32(rt, lane);
                let result = match function {
                    0x3c => value.wrapping_shl(shift),
                    0x3e => value >> shift,
                    _ => ((value as i32) >> shift) as u32,
                };
                Self::with_lane32(&mut out, lane, result);
            }
        }
        self.write_reg128(rd, out);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::platform::PlatformId;

    struct TestBus {
        bytes: Vec<u8>,
    }

    impl TestBus {
        fn new() -> Self {
            Self {
                bytes: vec![0; 32 * 1024 * 1024],
            }
        }
        fn put32(&mut self, address: usize, value: u32) {
            self.bytes[address..address + 4].copy_from_slice(&value.to_le_bytes());
        }
        fn put128(&mut self, address: usize, value: u128) {
            self.bytes[address..address + 16].copy_from_slice(&value.to_le_bytes());
        }
    }

    impl R5900Bus for TestBus {
        fn read8(&mut self, address: u32) -> u8 {
            self.bytes[address as usize]
        }
        fn write8(&mut self, address: u32, value: u8) {
            self.bytes[address as usize] = value;
        }
    }

    fn i(op: u32, rs: u8, rt: u8, imm: u16) -> u32 {
        (op << 26) | (u32::from(rs) << 21) | (u32::from(rt) << 16) | u32::from(imm)
    }
    fn r(rs: u8, rt: u8, rd: u8, sa: u32, function: u32) -> u32 {
        (u32::from(rs) << 21)
            | (u32::from(rt) << 16)
            | (u32::from(rd) << 11)
            | ((sa & 31) << 6)
            | (function & 63)
    }
    fn mmi(rs: u8, rt: u8, rd: u8, sub: u32, function: u32) -> u32 {
        (0x1c << 26)
            | (u32::from(rs) << 21)
            | (u32::from(rt) << 16)
            | (u32::from(rd) << 11)
            | ((sub & 31) << 6)
            | (function & 63)
    }

    #[test]
    fn reset_state_and_scalar_writes_preserve_upper_gpr() {
        let mut bus = TestBus::new();
        bus.put32(0x100, i(0x0d, 1, 1, 0x00ff));
        let mut cpu = MipsR5900::default();
        assert_eq!(cpu.pc, 0xffff_ffff_bfc0_0000);
        assert_eq!(cpu.cop0[COP0_STATUS], 0x7040_0004);
        assert_eq!(cpu.cop0[COP0_PRID], 0x2e20);
        assert_eq!(cpu.cop0[COP0_CONFIG], 0x440);
        assert_eq!(cpu.cop0[COP0_RANDOM], 47);
        cpu.reset_to(0xffff_ffff_8000_0100);
        cpu.regs[1] = (0xaabb_ccdd_eeff_0011u128 << 64) | 0x1000;
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[1] >> 64, 0xaabb_ccdd_eeff_0011);
        assert_eq!(cpu.regs[1] as u64, 0x10ff);
    }

    #[test]
    fn little_endian_quadword_and_merge_memory_execute() {
        let mut bus = TestBus::new();
        let quad = 0x1122_3344_5566_7788_99aa_bbcc_ddee_ff00u128;
        bus.put128(0x200, quad);
        bus.put32(0x100, i(0x1e, 1, 2, 7));
        bus.put32(0x104, i(0x1f, 1, 2, 0x27));
        let mut cpu = MipsR5900::default();
        cpu.reset_to(0xffff_ffff_8000_0100);
        cpu.regs[1] = 0xffff_ffff_8000_0201;
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[2], quad);
        cpu.step(&mut bus);
        assert_eq!(bus.read128(0x220), quad);

        bus.write32(0x300, 0x1122_3344);
        cpu.regs[3] = (0xfeed_face_cafe_beefu128 << 64) | 0xdead_beef_dead_beefu128;
        cpu.load_word_left(&mut bus, 3, 0xffff_ffff_8000_0303);
        assert_eq!(cpu.regs[3] >> 64, 0xfeed_face_cafe_beef);
        assert_eq!(cpu.regs[3] as u64, 0x1122_3344);
        cpu.regs[4] = (0x1234_5678_9abc_def0u128 << 64) | 0xaaaa_bbbb_cccc_dddd;
        cpu.load_word_right(&mut bus, 4, 0xffff_ffff_8000_0301);
        assert_eq!(cpu.regs[4] >> 64, 0x1234_5678_9abc_def0);
        assert_eq!(cpu.regs[4] as u64 >> 32, 0xaaaa_bbbb);
    }

    #[test]
    fn mmi_packed_logic_interleave_and_copy_execute() {
        let mut cpu = MipsR5900::default();
        cpu.regs[1] = 0x0000_0004_0000_0003_0000_0002_0000_0001;
        cpu.regs[2] = 0x0000_0028_0000_001e_0000_0014_0000_000a;
        cpu.execute_mmi(mmi(1, 2, 3, 0x00, 0x08));
        assert_eq!(cpu.regs[3], 0x0000_002c_0000_0021_0000_0016_0000_000b);
        cpu.execute_mmi(mmi(1, 2, 4, 0x12, 0x08));
        assert_eq!(MipsR5900::lane32(cpu.regs[4], 0), 10);
        assert_eq!(MipsR5900::lane32(cpu.regs[4], 1), 1);
        cpu.execute_mmi(mmi(1, 2, 5, 0x12, 0x09));
        assert_eq!(cpu.regs[5], cpu.regs[1] & cpu.regs[2]);

        cpu.regs[6] = 0xaaaa_bbbb_cccc_dddd_1111_2222_3333_4444;
        cpu.regs[7] = 0xeeee_ffff_0000_1111_5555_6666_7777_8888;
        cpu.execute_mmi(mmi(6, 7, 8, 0x0e, 0x29));
        assert_eq!(cpu.regs[8], 0xeeee_ffff_0000_1111_aaaa_bbbb_cccc_dddd);
    }

    #[test]
    fn sa_qfsrv_and_secondary_hilo_execute() {
        let mut cpu = MipsR5900::default();
        cpu.regs[1] = 0x0f;
        cpu.execute_regimm(i(0x01, 1, 0x18, 3));
        assert_eq!(cpu.sa, 12);
        cpu.regs[2] = 0x0011_2233_4455_6677_8899_aabb_ccdd_eeff;
        cpu.regs[3] = 0xffee_ddcc_bbaa_9988_7766_5544_3322_1100;
        cpu.sa = 1;
        cpu.execute_mmi(mmi(2, 3, 4, 0x1b, 0x28));
        assert_eq!(cpu.regs[4], (cpu.regs[3] >> 8) | (cpu.regs[2] << 120));

        cpu.regs[5] = 7;
        cpu.regs[6] = 9;
        cpu.execute_mmi(mmi(5, 6, 7, 0, 0x18));
        assert_eq!(cpu.lo1 as u32, 63);
        assert_eq!(cpu.regs[7] as u32, 63);
        cpu.execute_special(r(0, 0, 8, 0, 0x12));
        assert_eq!(cpu.regs[8] as u64, cpu.lo64());
    }

    #[test]
    fn tlb_and_interrupt_vectors_use_ee_rules() {
        let mut bus = TestBus::new();
        let mut cpu = MipsR5900::default();
        cpu.cop0[COP0_INDEX] = 47;
        cpu.cop0[COP0_ENTRY_HI] = 0x0040_0001;
        cpu.cop0[COP0_ENTRY_LO0] = (3 << 6) | 0x07;
        cpu.cop0[COP0_ENTRY_LO1] = (4 << 6) | 0x07;
        cpu.tlbwi();
        assert_eq!(cpu.tlb[47].entry_hi, 0x0040_0001);
        assert_eq!(cpu.translate(0x0040_0010, false).unwrap(), 0x3010);

        cpu.reset_to(0xffff_ffff_8000_0100);
        cpu.cop0[COP0_STATUS] = STATUS_IE | STATUS_EIE | (1 << 10);
        cpu.set_irq_line(2, true);
        cpu.step(&mut bus);
        assert_eq!(cpu.pc, 0xffff_ffff_8000_0200);
        assert_eq!(cpu.cop0[COP0_EPC], 0xffff_ffff_8000_0100);
        assert_ne!(cpu.cop0[COP0_STATUS] & STATUS_EXL, 0);
    }

    #[test]
    fn trap_and_ei_di_extensions_execute() {
        let mut cpu = MipsR5900::default();
        cpu.reset_to(0xffff_ffff_8000_0100);
        cpu.current_pc = cpu.pc;
        cpu.regs[1] = 5;
        cpu.regs[2] = 5;
        cpu.execute_special(r(1, 2, 0, 0, 0x34));
        assert_eq!((cpu.cop0[COP0_CAUSE] >> 2) & 0x1f, Exception::Trap as u64);
        cpu.cop0[COP0_STATUS] &= !STATUS_EIE;
        cpu.execute_cop0((0x10 << 26) | (0x10 << 21) | 0x38);
        assert_ne!(cpu.cop0[COP0_STATUS] & STATUS_EIE, 0);
        cpu.execute_cop0((0x10 << 26) | (0x10 << 21) | 0x39);
        assert_eq!(cpu.cop0[COP0_STATUS] & STATUS_EIE, 0);
    }

    #[test]
    fn state_round_trip_preserves_128_bit_and_secondary_state() {
        let mut cpu = MipsR5900::default();
        cpu.reset_to(0xffff_ffff_8000_1234);
        cpu.regs[1] = 0x1122_3344_5566_7788_99aa_bbcc_ddee_ff00;
        cpu.hi = 0x8877_6655_4433_2211_0011_2233_4455_6677;
        cpu.lo = 0x0123_4567_89ab_cdef_fedc_ba98_7654_3210;
        cpu.hi1 = 0x1234_5678_9abc_def0;
        cpu.lo1 = 0xfedc_ba98_7654_3210;
        cpu.sa = 13;
        cpu.cop0[COP0_INDEX] = 47;
        cpu.cop0[COP0_ENTRY_HI] = 0x0040_0007;
        cpu.cop0[COP0_ENTRY_LO0] = (0x123 << 6) | 0x07;
        cpu.cop0[COP0_ENTRY_LO1] = (0x124 << 6) | 0x07;
        cpu.tlbwi();
        cpu.branch_delay_next = true;
        cpu.count_phase = true;
        let mut writer = StateWriter::new(PlatformId::PlayStation2, 1);
        cpu.save(&mut writer);
        let bytes = writer.finish();
        let mut reader = StateReader::new(&bytes, PlatformId::PlayStation2, 1).unwrap();
        let mut restored = MipsR5900::default();
        restored.load_state(&mut reader).unwrap();
        reader.finish().unwrap();
        assert_eq!(restored.regs, cpu.regs);
        assert_eq!(restored.hi, cpu.hi);
        assert_eq!(restored.lo, cpu.lo);
        assert_eq!(restored.hi1, cpu.hi1);
        assert_eq!(restored.lo1, cpu.lo1);
        assert_eq!(restored.sa, cpu.sa);
        assert_eq!(restored.tlb[47], cpu.tlb[47]);
        assert!(restored.branch_delay_next && restored.count_phase);
    }
}
