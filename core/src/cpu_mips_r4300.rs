use crate::state::{StateReader, StateWriter};

pub trait Mips64Bus: Send {
    fn read8(&mut self, address: u32) -> u8;
    fn write8(&mut self, address: u32, value: u8);

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
    fn read64(&mut self, address: u32) -> u64 {
        u64::from_be_bytes([
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
    fn write64(&mut self, address: u32, value: u64) {
        for (offset, byte) in value.to_be_bytes().into_iter().enumerate() {
            self.write8(address.wrapping_add(offset as u32), byte);
        }
    }

    fn write_sized(&mut self, address: u32, width: u8, value: u64) {
        match width {
            1 => self.write8(address, value as u8),
            2 => self.write16(address, value as u16),
            4 => self.write32(address, value as u32),
            8 => self.write64(address, value),
            _ => {}
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
const COP0_LLADDR: usize = 17;
const COP0_XCONTEXT: usize = 20;
const COP0_ERROREPC: usize = 30;

const STATUS_IE: u64 = 1;
const STATUS_EXL: u64 = 1 << 1;
const STATUS_ERL: u64 = 1 << 2;
const STATUS_KSU_MASK: u64 = 3 << 3;
const STATUS_UX: u64 = 1 << 5;
const STATUS_SX: u64 = 1 << 6;
const STATUS_KX: u64 = 1 << 7;
const STATUS_BEV: u64 = 1 << 22;
const STATUS_FR: u64 = 1 << 26;
const STATUS_CU0: u64 = 1 << 28;
const STATUS_CU1: u64 = 1 << 29;

const FCR31_RM_MASK: u32 = 0x0000_0003;
const FCR31_FLAG_MASK: u32 = 0x0000_007c;
const FCR31_CAUSE_MASK: u32 = 0x0003_f000;
const FCR31_CONDITION: u32 = 1 << 23;
const FCR31_FS: u32 = 1 << 24;
const FCR0_REVISION: u32 = 0x0000_0a00;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CpuMode {
    Kernel,
    Supervisor,
    User,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum VirtualSegment {
    Mapped,
    Direct(u32),
    Unused,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Exception {
    Interrupt,
    TlbModification,
    TlbLoadMiss,
    TlbLoadInvalid,
    TlbStoreMiss,
    TlbStoreInvalid,
    AddressLoad,
    AddressStore,
    Syscall,
    Break,
    ReservedInstruction,
    CoprocessorUnusable,
    Overflow,
    Trap,
    FloatingPoint,
}

impl Exception {
    const fn code(self) -> u64 {
        match self {
            Self::Interrupt => 0,
            Self::TlbModification => 1,
            Self::TlbLoadMiss | Self::TlbLoadInvalid => 2,
            Self::TlbStoreMiss | Self::TlbStoreInvalid => 3,
            Self::AddressLoad => 4,
            Self::AddressStore => 5,
            Self::Syscall => 8,
            Self::Break => 9,
            Self::ReservedInstruction => 10,
            Self::CoprocessorUnusable => 11,
            Self::Overflow => 12,
            Self::Trap => 13,
            Self::FloatingPoint => 15,
        }
    }

    const fn is_tlb_refill(self) -> bool {
        matches!(self, Self::TlbLoadMiss | Self::TlbStoreMiss)
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
struct TlbEntry {
    page_mask: u32,
    entry_hi: u64,
    entry_lo0: u64,
    entry_lo1: u64,
}
#[derive(Debug, Clone)]
pub struct MipsR4300 {
    pub regs: [u64; 32],
    pub hi: u64,
    pub lo: u64,
    pub pc: u64,
    pub next_pc: u64,
    pub cop0: [u64; 32],
    pub fpr: [u64; 32],
    pub fcr0: u32,
    pub fcr31: u32,
    pub cycles: u64,
    tlb: [TlbEntry; 32],
    current_pc: u64,
    branch_delay_next: bool,
    in_delay_slot: bool,
    exception_raised: bool,
    llbit: bool,
    count_phase: bool,
}

impl Default for MipsR4300 {
    fn default() -> Self {
        let mut cop0 = [0u64; 32];
        cop0[COP0_STATUS] = STATUS_BEV | STATUS_ERL;
        cop0[COP0_PRID] = 0x0000_0b22;
        cop0[COP0_CONFIG] = 0x7006_e463;
        cop0[COP0_RANDOM] = 31;
        Self {
            regs: [0; 32],
            hi: 0,
            lo: 0,
            pc: 0xffff_ffff_bfc0_0000,
            next_pc: 0xffff_ffff_bfc0_0004,
            cop0,
            fpr: [0; 32],
            fcr0: FCR0_REVISION,
            fcr31: 0,
            cycles: 0,
            tlb: [TlbEntry::default(); 32],
            current_pc: 0xffff_ffff_bfc0_0000,
            branch_delay_next: false,
            in_delay_slot: false,
            exception_raised: false,
            llbit: false,
            count_phase: false,
        }
    }
}
impl MipsR4300 {
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
            && status & (STATUS_EXL | STATUS_ERL) == 0
            && status & cause & 0x0000_ff00 != 0
    }

    fn reg(&self, index: u8) -> u64 {
        self.regs[usize::from(index)]
    }
    fn write_reg(&mut self, index: u8, value: u64) {
        if index != 0 {
            self.regs[usize::from(index)] = value;
        }
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

    fn cpu_mode(&self) -> CpuMode {
        let status = self.cop0[COP0_STATUS];
        if status & (STATUS_EXL | STATUS_ERL) != 0 {
            return CpuMode::Kernel;
        }
        match (status & STATUS_KSU_MASK) >> 3 {
            0 => CpuMode::Kernel,
            1 => CpuMode::Supervisor,
            _ => CpuMode::User,
        }
    }

    fn extended_addressing(&self) -> bool {
        let status = self.cop0[COP0_STATUS];
        match self.cpu_mode() {
            CpuMode::Kernel => status & STATUS_KX != 0,
            CpuMode::Supervisor => status & STATUS_SX != 0,
            CpuMode::User => status & STATUS_UX != 0,
        }
    }

    fn require_64_bit_integer(&mut self) -> bool {
        if self.cpu_mode() == CpuMode::Kernel || self.extended_addressing() {
            true
        } else {
            self.take_exception(Exception::ReservedInstruction, None);
            false
        }
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

    fn segment_32(&self, address: u64) -> VirtualSegment {
        let Some(virtual32) = Self::canonical32(address) else {
            return VirtualSegment::Unused;
        };
        match self.cpu_mode() {
            CpuMode::Kernel => match virtual32 {
                0x0000_0000..=0x7fff_ffff => VirtualSegment::Mapped,
                0x8000_0000..=0xbfff_ffff => VirtualSegment::Direct(virtual32 & 0x1fff_ffff),
                _ => VirtualSegment::Mapped,
            },
            CpuMode::Supervisor => match virtual32 {
                0x0000_0000..=0x7fff_ffff | 0xc000_0000..=0xdfff_ffff => VirtualSegment::Mapped,
                _ => VirtualSegment::Unused,
            },
            CpuMode::User => match virtual32 {
                0x0000_0000..=0x7fff_ffff => VirtualSegment::Mapped,
                _ => VirtualSegment::Unused,
            },
        }
    }

    fn segment_64(&self, address: u64) -> VirtualSegment {
        match self.cpu_mode() {
            CpuMode::Kernel => match address {
                0x0000_0000_0000_0000..=0x0000_00ff_ffff_ffff => VirtualSegment::Mapped,
                0x0000_0100_0000_0000..=0x3fff_ffff_ffff_ffff => VirtualSegment::Unused,
                0x4000_0000_0000_0000..=0x4000_00ff_ffff_ffff => VirtualSegment::Mapped,
                0x4000_0100_0000_0000..=0x7fff_ffff_ffff_ffff => VirtualSegment::Unused,
                0x8000_0000_0000_0000..=0x8000_0000_ffff_ffff => {
                    VirtualSegment::Direct(address as u32)
                }
                0x8000_0001_0000_0000..=0x87ff_ffff_ffff_ffff => VirtualSegment::Unused,
                0x8800_0000_0000_0000..=0x8800_0000_ffff_ffff => {
                    VirtualSegment::Direct(address as u32)
                }
                0x8800_0001_0000_0000..=0x8fff_ffff_ffff_ffff => VirtualSegment::Unused,
                0x9000_0000_0000_0000..=0x9000_0000_ffff_ffff => {
                    VirtualSegment::Direct(address as u32)
                }
                0x9000_0001_0000_0000..=0x97ff_ffff_ffff_ffff => VirtualSegment::Unused,
                0x9800_0000_0000_0000..=0x9800_0000_ffff_ffff => {
                    VirtualSegment::Direct(address as u32)
                }
                0x9800_0001_0000_0000..=0x9fff_ffff_ffff_ffff => VirtualSegment::Unused,
                0xa000_0000_0000_0000..=0xa000_0000_ffff_ffff => {
                    VirtualSegment::Direct(address as u32)
                }
                0xa000_0001_0000_0000..=0xa7ff_ffff_ffff_ffff => VirtualSegment::Unused,
                0xa800_0000_0000_0000..=0xa800_0000_ffff_ffff => {
                    VirtualSegment::Direct(address as u32)
                }
                0xa800_0001_0000_0000..=0xafff_ffff_ffff_ffff => VirtualSegment::Unused,
                0xb000_0000_0000_0000..=0xb000_0000_ffff_ffff => {
                    VirtualSegment::Direct(address as u32)
                }
                0xb000_0001_0000_0000..=0xb7ff_ffff_ffff_ffff => VirtualSegment::Unused,
                0xb800_0000_0000_0000..=0xb800_0000_ffff_ffff => {
                    VirtualSegment::Direct(address as u32)
                }
                0xb800_0001_0000_0000..=0xbfff_ffff_ffff_ffff => VirtualSegment::Unused,
                0xc000_0000_0000_0000..=0xc000_00ff_7fff_ffff => VirtualSegment::Mapped,
                0xc000_0100_0000_0000..=0xffff_ffff_7fff_ffff => VirtualSegment::Unused,
                0xffff_ffff_8000_0000..=0xffff_ffff_bfff_ffff => {
                    VirtualSegment::Direct((address as u32) & 0x1fff_ffff)
                }
                0xffff_ffff_c000_0000..=0xffff_ffff_ffff_ffff => VirtualSegment::Mapped,
                _ => VirtualSegment::Unused,
            },
            CpuMode::Supervisor => match address {
                0x0000_0000_0000_0000..=0x0000_00ff_ffff_ffff => VirtualSegment::Mapped,
                0x0000_0100_0000_0000..=0x3fff_ffff_ffff_ffff => VirtualSegment::Unused,
                0x4000_0000_0000_0000..=0x4000_00ff_ffff_ffff => VirtualSegment::Mapped,
                0x4000_0100_0000_0000..=0xffff_ffff_bfff_ffff => VirtualSegment::Unused,
                0xffff_ffff_c000_0000..=0xffff_ffff_dfff_ffff => VirtualSegment::Mapped,
                _ => VirtualSegment::Unused,
            },
            CpuMode::User => match address {
                0x0000_0000_0000_0000..=0x0000_00ff_ffff_ffff => VirtualSegment::Mapped,
                _ => VirtualSegment::Unused,
            },
        }
    }

    fn virtual_segment(&self, address: u64) -> VirtualSegment {
        if self.extended_addressing() {
            self.segment_64(address)
        } else {
            self.segment_32(address)
        }
    }

    fn tlb_page_size(entry: &TlbEntry) -> u64 {
        u64::from((entry.page_mask | 0x1fff).wrapping_add(1))
    }
    fn translate(&mut self, address: u64, store: bool) -> Result<u32, Exception> {
        match self.virtual_segment(address) {
            VirtualSegment::Direct(physical) => return Ok(physical),
            VirtualSegment::Unused => {
                return Err(if store {
                    Exception::AddressStore
                } else {
                    Exception::AddressLoad
                });
            }
            VirtualSegment::Mapped => {}
        }

        const VPN_MASK_40: u64 = 0x0000_00ff_ffff_e000;
        let asid = self.cop0[COP0_ENTRY_HI] as u8;
        let region = address >> 62;
        for entry in self.tlb {
            let pair_size = Self::tlb_page_size(&entry);
            let mask = (!(pair_size - 1)) & VPN_MASK_40;
            let vpn_match = (address & mask) == (entry.entry_hi & mask);
            let region_match = (entry.entry_hi >> 62) == region;
            let global = entry.entry_lo0 & 1 != 0 && entry.entry_lo1 & 1 != 0;
            if !vpn_match || !region_match || (!global && entry.entry_hi as u8 != asid) {
                continue;
            }

            let half = pair_size >> 1;
            let lo = if address & half == 0 {
                entry.entry_lo0
            } else {
                entry.entry_lo1
            };
            if lo & 0x02 == 0 {
                self.set_tlb_fault_state(address);
                return Err(if store {
                    Exception::TlbStoreInvalid
                } else {
                    Exception::TlbLoadInvalid
                });
            }
            if store && lo & 0x04 == 0 {
                self.set_tlb_fault_state(address);
                return Err(Exception::TlbModification);
            }

            let page_offset = address & (half - 1);
            let physical_base = ((lo >> 6) << 12) & 0xffff_ffff;
            return Ok((physical_base | page_offset) as u32);
        }

        self.set_tlb_fault_state(address);
        Err(if store {
            Exception::TlbStoreMiss
        } else {
            Exception::TlbLoadMiss
        })
    }
    fn set_tlb_fault_state(&mut self, address: u64) {
        self.cop0[COP0_BADVADDR] = address;

        let context_base = self.cop0[COP0_CONTEXT] & 0xffff_ffff_ff80_0000;
        let context_bad_vpn = ((address >> 13) & 0x7ffff) << 4;
        self.cop0[COP0_CONTEXT] = context_base | context_bad_vpn;

        let xcontext_base = self.cop0[COP0_XCONTEXT] & 0xffff_fffe_0000_0000;
        let xcontext_bad_vpn = ((address >> 13) & 0x07ff_ffff) << 4;
        let xcontext_region = ((address >> 62) & 3) << 31;
        self.cop0[COP0_XCONTEXT] = xcontext_base | xcontext_region | xcontext_bad_vpn;

        let asid = self.cop0[COP0_ENTRY_HI] & 0xff;
        let region = address & 0xc000_0000_0000_0000;
        let vpn = address & 0x0000_00ff_ffff_e000;
        self.cop0[COP0_ENTRY_HI] = region | vpn | asid;
    }

    fn take_exception(&mut self, exception: Exception, bad_address: Option<u64>) {
        self.exception_raised = true;
        if let Some(address) = bad_address {
            self.cop0[COP0_BADVADDR] = address;
        }
        let status = self.cop0[COP0_STATUS];
        let mut cause = self.cop0[COP0_CAUSE] & 0x0000_ff00;
        cause |= exception.code() << 2;
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
        let tlb_refill = exception.is_tlb_refill() && status & STATUS_EXL == 0;
        let vector = if status & STATUS_BEV != 0 {
            if tlb_refill {
                0xffff_ffff_bfc0_0200
            } else {
                0xffff_ffff_bfc0_0380
            }
        } else if tlb_refill {
            0xffff_ffff_8000_0000
        } else {
            0xffff_ffff_8000_0180
        };
        self.pc = vector;
        self.next_pc = vector.wrapping_add(4);
        self.branch_delay_next = false;
        self.in_delay_slot = false;
        self.llbit = false;
    }

    fn tick_count(&mut self) {
        self.count_phase = !self.count_phase;
        if self.count_phase {
            self.cop0[COP0_COUNT] = self.cop0[COP0_COUNT].wrapping_add(1) & 0xffff_ffff;
            if self.cop0[COP0_COUNT] as u32 == self.cop0[COP0_COMPARE] as u32 {
                self.cop0[COP0_CAUSE] |= 1 << 15;
            }
        }
        let wired = (self.cop0[COP0_WIRED] as usize).min(31);
        let random = self.cop0[COP0_RANDOM] as usize;
        self.cop0[COP0_RANDOM] = if random <= wired {
            31
        } else {
            (random - 1) as u64
        };
    }

    fn branch_target(&self, instruction: u32) -> u64 {
        self.pc
            .wrapping_add((instruction as u16 as i16 as i64 as u64) << 2)
    }
    pub fn step<B: Mips64Bus>(&mut self, bus: &mut B) -> u32 {
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
    fn execute<B: Mips64Bus>(&mut self, bus: &mut B, instruction: u32) {
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
            0x11 => self.execute_cop1(instruction),
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
            0x1a | 0x1b | 0x20..=0x27 | 0x30 | 0x31 | 0x34..=0x37 => self.load(bus, instruction),
            0x28..=0x2f | 0x38..=0x3f => self.store(bus, instruction),
            _ => self.take_exception(Exception::ReservedInstruction, None),
        }
    }
    fn write32_result(&mut self, rd: u8, value: u32) {
        self.write_reg(rd, Self::sign32(value));
    }

    fn write64_integer_result(&mut self, rd: u8, value: u64) {
        if self.require_64_bit_integer() {
            self.write_reg(rd, value);
        }
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
            0x0c => self.take_exception(Exception::Syscall, None),
            0x0d => self.take_exception(Exception::Break, None),
            0x0f => {}
            0x10 => self.write_reg(rd, self.hi),
            0x11 => self.hi = left,
            0x12 => self.write_reg(rd, self.lo),
            0x13 => self.lo = left,
            0x14 => self.write64_integer_result(rd, right << (left & 63)),
            0x16 => self.write64_integer_result(rd, right >> (left & 63)),
            0x17 => self.write64_integer_result(rd, ((right as i64) >> (left & 63)) as u64),
            0x18 => {
                let value = i64::from(left as u32 as i32) * i64::from(right as u32 as i32);
                self.lo = Self::sign32(value as u32);
                self.hi = Self::sign32((value >> 32) as u32);
            }
            0x19 => {
                let value = u64::from(left as u32) * u64::from(right as u32);
                self.lo = Self::sign32(value as u32);
                self.hi = Self::sign32((value >> 32) as u32);
            }
            0x1a => self.div32(left as u32, right as u32, true),
            0x1b => self.div32(left as u32, right as u32, false),
            0x1c => self.dmult(left, right, true),
            0x1d => self.dmult(left, right, false),
            0x1e => self.ddiv(left, right, true),
            0x1f => self.ddiv(left, right, false),
            0x20 => self.add32(rd, left as u32, right as u32, true),
            0x21 => self.add32(rd, left as u32, right as u32, false),
            0x22 => self.sub32(rd, left as u32, right as u32, true),
            0x23 => self.sub32(rd, left as u32, right as u32, false),
            0x24 => self.write_reg(rd, left & right),
            0x25 => self.write_reg(rd, left | right),
            0x26 => self.write_reg(rd, left ^ right),
            0x27 => self.write_reg(rd, !(left | right)),
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
            0x38 => self.write64_integer_result(rd, right << sa),
            0x3a => self.write64_integer_result(rd, right >> sa),
            0x3b => self.write64_integer_result(rd, ((right as i64) >> sa) as u64),
            0x3c => self.write64_integer_result(rd, right << (sa + 32)),
            0x3e => self.write64_integer_result(rd, right >> (sa + 32)),
            0x3f => self.write64_integer_result(rd, ((right as i64) >> (sa + 32)) as u64),
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
        if !self.require_64_bit_integer() {
            return;
        }
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
        if !self.require_64_bit_integer() {
            return;
        }
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
        if signed {
            let dividend = left as i32;
            let divisor = right as i32;
            if divisor == 0 {
                self.lo = Self::sign32(if dividend >= 0 { u32::MAX } else { 1 });
                self.hi = Self::sign32(left);
            } else if dividend == i32::MIN && divisor == -1 {
                self.lo = Self::sign32(i32::MIN as u32);
                self.hi = 0;
            } else {
                self.lo = Self::sign32((dividend / divisor) as u32);
                self.hi = Self::sign32((dividend % divisor) as u32);
            }
        } else if let Some(quotient) = left.checked_div(right) {
            self.lo = Self::sign32(quotient);
            self.hi = Self::sign32(left % right);
        } else {
            self.lo = Self::sign32(u32::MAX);
            self.hi = Self::sign32(left);
        }
    }

    fn dmult(&mut self, left: u64, right: u64, signed: bool) {
        if !self.require_64_bit_integer() {
            return;
        }
        let product = if signed {
            (left as i64 as i128).wrapping_mul(right as i64 as i128) as u128
        } else {
            u128::from(left).wrapping_mul(u128::from(right))
        };
        self.lo = product as u64;
        self.hi = (product >> 64) as u64;
    }
    fn ddiv(&mut self, left: u64, right: u64, signed: bool) {
        if !self.require_64_bit_integer() {
            return;
        }
        if signed {
            let dividend = left as i64;
            let divisor = right as i64;
            if divisor == 0 {
                self.lo = if dividend >= 0 { u64::MAX } else { 1 };
                self.hi = left;
            } else if dividend == i64::MIN && divisor == -1 {
                self.lo = i64::MIN as u64;
                self.hi = 0;
            } else {
                self.lo = (dividend / divisor) as u64;
                self.hi = (dividend % divisor) as u64;
            }
        } else if let Some(quotient) = left.checked_div(right) {
            self.lo = quotient;
            self.hi = left % right;
        } else {
            self.lo = u64::MAX;
            self.hi = left;
        }
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
        if !self.require_64_bit_integer() {
            return;
        }
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
        let register = self.reg(Self::rs(instruction));
        let value = register as i64;
        let immediate = Self::simm(instruction);
        let signed_immediate = immediate as i64;
        let rt = Self::rt(instruction);
        match rt {
            0x08 => {
                self.trap(value >= signed_immediate);
                return;
            }
            0x09 => {
                self.trap(register >= immediate);
                return;
            }
            0x0a => {
                self.trap(value < signed_immediate);
                return;
            }
            0x0b => {
                self.trap(register < immediate);
                return;
            }
            0x0c => {
                self.trap(value == signed_immediate);
                return;
            }
            0x0e => {
                self.trap(value != signed_immediate);
                return;
            }
            _ => {}
        }

        let (take, likely, link) = match rt {
            0x00 => (value < 0, false, false),
            0x01 => (value >= 0, false, false),
            0x02 => (value < 0, true, false),
            0x03 => (value >= 0, true, false),
            0x10 => (value < 0, false, true),
            0x11 => (value >= 0, false, true),
            0x12 => (value < 0, true, true),
            0x13 => (value >= 0, true, true),
            _ => {
                self.take_exception(Exception::ReservedInstruction, None);
                return;
            }
        };
        if link {
            self.write_reg(31, self.next_pc);
        }
        self.branch(instruction, take, likely);
    }
    fn execute_cop0(&mut self, instruction: u32) {
        if self.cpu_mode() != CpuMode::Kernel && self.cop0[COP0_STATUS] & STATUS_CU0 == 0 {
            self.take_coprocessor_unusable(0);
            return;
        }

        let rs = (instruction >> 21) & 31;
        let rt = Self::rt(instruction);
        let rd = usize::from(Self::rd(instruction));
        match rs {
            0x00 => self.write_reg(rt, Self::sign32(self.cop0[rd] as u32)),
            0x01 => {
                if self.cpu_mode() != CpuMode::Kernel && !self.extended_addressing() {
                    self.take_exception(Exception::ReservedInstruction, None);
                } else {
                    self.write_reg(rt, self.cop0[rd]);
                }
            }
            0x04 => self.write_cop0(rd, Self::sign32(self.reg(rt) as u32)),
            0x05 => {
                if self.cpu_mode() != CpuMode::Kernel && !self.extended_addressing() {
                    self.take_exception(Exception::ReservedInstruction, None);
                } else {
                    self.write_cop0(rd, self.reg(rt));
                }
            }
            0x10 => match instruction & 0x3f {
                0x01 => self.tlbr(),
                0x02 => self.tlbwi(),
                0x06 => self.tlbwr(),
                0x08 => self.tlbp(),
                0x18 => self.eret(),
                _ => self.take_exception(Exception::ReservedInstruction, None),
            },
            _ => self.take_exception(Exception::ReservedInstruction, None),
        }
    }

    fn take_coprocessor_unusable(&mut self, coprocessor: u8) {
        self.take_exception(Exception::CoprocessorUnusable, None);
        self.cop0[COP0_CAUSE] =
            (self.cop0[COP0_CAUSE] & !(3u64 << 28)) | (u64::from(coprocessor & 3) << 28);
    }

    fn cop1_usable(&self) -> bool {
        self.cop0[COP0_STATUS] & STATUS_CU1 != 0
    }

    fn fpr_word(&self, index: u8) -> u32 {
        let index = usize::from(index & 31);
        if self.cop0[COP0_STATUS] & STATUS_FR != 0 {
            self.fpr[index] as u32
        } else {
            let pair = index & !1;
            if index & 1 == 0 {
                self.fpr[pair] as u32
            } else {
                (self.fpr[pair] >> 32) as u32
            }
        }
    }

    fn write_fpr_word(&mut self, index: u8, value: u32) {
        let index = usize::from(index & 31);
        if self.cop0[COP0_STATUS] & STATUS_FR != 0 {
            self.fpr[index] = (self.fpr[index] & 0xffff_ffff_0000_0000) | u64::from(value);
        } else {
            let pair = index & !1;
            if index & 1 == 0 {
                self.fpr[pair] = (self.fpr[pair] & 0xffff_ffff_0000_0000) | u64::from(value);
            } else {
                self.fpr[pair] =
                    (self.fpr[pair] & 0x0000_0000_ffff_ffff) | (u64::from(value) << 32);
            }
        }
    }

    fn fpr_double(&self, index: u8) -> Option<u64> {
        let index = usize::from(index & 31);
        if self.cop0[COP0_STATUS] & STATUS_FR == 0 && index & 1 != 0 {
            None
        } else {
            Some(self.fpr[index])
        }
    }

    fn write_fpr_double(&mut self, index: u8, value: u64) -> bool {
        let index = usize::from(index & 31);
        if self.cop0[COP0_STATUS] & STATUS_FR == 0 && index & 1 != 0 {
            return false;
        }
        self.fpr[index] = value;
        true
    }

    fn execute_cop1(&mut self, instruction: u32) {
        if !self.cop1_usable() {
            self.take_coprocessor_unusable(1);
            return;
        }
        let rs = (instruction >> 21) & 31;
        let rt = Self::rt(instruction);
        let fs = Self::rd(instruction);
        match rs {
            0x00 => self.write_reg(rt, Self::sign32(self.fpr_word(fs))),
            0x01 => {
                let Some(value) = self.fpr_double(fs) else {
                    self.take_exception(Exception::ReservedInstruction, None);
                    return;
                };
                self.write_reg(rt, value);
            }
            0x02 => {
                let value = match fs {
                    0 => self.fcr0,
                    31 => self.fcr31,
                    _ => {
                        self.take_exception(Exception::ReservedInstruction, None);
                        return;
                    }
                };
                self.write_reg(rt, Self::sign32(value));
            }
            0x04 => self.write_fpr_word(fs, self.reg(rt) as u32),
            0x05 => {
                if !self.write_fpr_double(fs, self.reg(rt)) {
                    self.take_exception(Exception::ReservedInstruction, None);
                }
            }
            0x06 => {
                if fs != 31 {
                    self.take_exception(Exception::ReservedInstruction, None);
                    return;
                }
                self.fcr31 = self.reg(rt) as u32
                    & (FCR31_RM_MASK
                        | FCR31_FLAG_MASK
                        | FCR31_CAUSE_MASK
                        | FCR31_CONDITION
                        | FCR31_FS);
            }
            0x08 => {
                if rt > 3 {
                    self.take_exception(Exception::ReservedInstruction, None);
                    return;
                }
                let condition = self.fcr31 & FCR31_CONDITION != 0;
                let take = condition == (rt & 1 != 0);
                self.branch(instruction, take, rt & 2 != 0);
            }
            0x10 | 0x11 | 0x14 | 0x15 => self.execute_cop1_arithmetic(instruction, rs),
            _ => self.take_exception(Exception::ReservedInstruction, None),
        }
    }

    fn execute_cop1_arithmetic(&mut self, instruction: u32, format: u32) {
        let ft = Self::rt(instruction);
        let fs = Self::rd(instruction);
        let fd = ((instruction >> 6) & 31) as u8;
        let function = instruction & 0x3f;
        self.fcr31 &= !FCR31_CAUSE_MASK;

        if (0x30..=0x3f).contains(&function) {
            self.cop1_compare(format, fs, ft, function as u8);
            return;
        }

        match function {
            0x00..=0x07 => self.cop1_basic_float(format, fs, ft, fd, function as u8),
            0x08..=0x0f => self.cop1_round_to_integer(format, fs, fd, function as u8),
            0x20 => self.cop1_convert_to_single(format, fs, fd),
            0x21 => self.cop1_convert_to_double(format, fs, fd),
            0x24 => self.cop1_convert_to_word(format, fs, fd, None),
            0x25 => self.cop1_convert_to_long(format, fs, fd, None),
            _ => self.take_exception(Exception::ReservedInstruction, None),
        }
    }

    fn read_float_value(&self, format: u32, index: u8) -> Option<f64> {
        match format {
            0x10 => Some(f64::from(f32::from_bits(self.fpr_word(index)))),
            0x11 => self.fpr_double(index).map(f64::from_bits),
            0x14 => Some(f64::from(self.fpr_word(index) as i32)),
            0x15 => self.fpr_double(index).map(|value| value as i64 as f64),
            _ => None,
        }
    }

    fn cop1_basic_float(&mut self, format: u32, fs: u8, ft: u8, fd: u8, function: u8) {
        if !matches!(format, 0x10 | 0x11) {
            self.take_exception(Exception::ReservedInstruction, None);
            return;
        }
        let Some(left) = self.read_float_value(format, fs) else {
            self.take_exception(Exception::ReservedInstruction, None);
            return;
        };
        let right = self.read_float_value(format, ft).unwrap_or(0.0);
        let result = match function {
            0x00 => left + right,
            0x01 => left - right,
            0x02 => left * right,
            0x03 => {
                if right == 0.0 && left != 0.0 && left.is_finite() {
                    self.raise_fpu_condition(3);
                    if self.exception_raised {
                        return;
                    }
                }
                left / right
            }
            0x04 => {
                if left < 0.0 {
                    self.raise_fpu_condition(4);
                    if self.exception_raised {
                        return;
                    }
                }
                left.sqrt()
            }
            0x05 => left.abs(),
            0x06 => left,
            0x07 => -left,
            _ => unreachable!(),
        };
        if result.is_nan() && !left.is_nan() && !right.is_nan() {
            self.raise_fpu_condition(4);
            if self.exception_raised {
                return;
            }
        }
        if format == 0x10 {
            self.write_fpr_word(fd, (result as f32).to_bits());
        } else if !self.write_fpr_double(fd, result.to_bits()) {
            self.take_exception(Exception::ReservedInstruction, None);
        }
    }

    fn cop1_compare(&mut self, format: u32, fs: u8, ft: u8, function: u8) {
        if !matches!(format, 0x10 | 0x11) {
            self.take_exception(Exception::ReservedInstruction, None);
            return;
        }
        let Some(left) = self.read_float_value(format, fs) else {
            self.take_exception(Exception::ReservedInstruction, None);
            return;
        };
        let Some(right) = self.read_float_value(format, ft) else {
            self.take_exception(Exception::ReservedInstruction, None);
            return;
        };
        let predicate = function & 0x0f;
        let unordered = left.is_nan() || right.is_nan();
        if unordered && predicate & 0x08 != 0 {
            self.raise_fpu_condition(4);
            if self.exception_raised {
                return;
            }
        }
        let condition = (unordered && predicate & 0x01 != 0)
            || (!unordered
                && ((left == right && predicate & 0x02 != 0)
                    || (left < right && predicate & 0x04 != 0)));
        if condition {
            self.fcr31 |= FCR31_CONDITION;
        } else {
            self.fcr31 &= !FCR31_CONDITION;
        }
    }

    fn cop1_round_to_integer(&mut self, format: u32, fs: u8, fd: u8, function: u8) {
        if !matches!(format, 0x10 | 0x11) {
            self.take_exception(Exception::ReservedInstruction, None);
            return;
        }
        let Some(value) = self.read_float_value(format, fs) else {
            self.take_exception(Exception::ReservedInstruction, None);
            return;
        };
        let rounding = match function & 3 {
            0 => 0,
            1 => 1,
            2 => 2,
            _ => 3,
        };
        if function <= 0x0b {
            self.write_integer_conversion(fd, value, true, rounding);
        } else {
            self.write_integer_conversion(fd, value, false, rounding);
        }
    }

    fn cop1_convert_to_single(&mut self, format: u32, fs: u8, fd: u8) {
        let Some(value) = self.read_float_value(format, fs) else {
            self.take_exception(Exception::ReservedInstruction, None);
            return;
        };
        self.write_fpr_word(fd, (value as f32).to_bits());
    }

    fn cop1_convert_to_double(&mut self, format: u32, fs: u8, fd: u8) {
        let Some(value) = self.read_float_value(format, fs) else {
            self.take_exception(Exception::ReservedInstruction, None);
            return;
        };
        if !self.write_fpr_double(fd, value.to_bits()) {
            self.take_exception(Exception::ReservedInstruction, None);
        }
    }

    fn cop1_convert_to_word(&mut self, format: u32, fs: u8, fd: u8, rounding: Option<u32>) {
        if matches!(format, 0x14 | 0x15) {
            self.take_exception(Exception::ReservedInstruction, None);
            return;
        }
        let Some(value) = self.read_float_value(format, fs) else {
            self.take_exception(Exception::ReservedInstruction, None);
            return;
        };
        self.write_integer_conversion(
            fd,
            value,
            false,
            rounding.unwrap_or(self.fcr31 & FCR31_RM_MASK),
        );
    }

    fn cop1_convert_to_long(&mut self, format: u32, fs: u8, fd: u8, rounding: Option<u32>) {
        if matches!(format, 0x14 | 0x15) {
            self.take_exception(Exception::ReservedInstruction, None);
            return;
        }
        let Some(value) = self.read_float_value(format, fs) else {
            self.take_exception(Exception::ReservedInstruction, None);
            return;
        };
        self.write_integer_conversion(
            fd,
            value,
            true,
            rounding.unwrap_or(self.fcr31 & FCR31_RM_MASK),
        );
    }

    fn write_integer_conversion(&mut self, fd: u8, value: f64, long: bool, rounding: u32) {
        let rounded = match rounding & 3 {
            0 => value.round_ties_even(),
            1 => value.trunc(),
            2 => value.ceil(),
            _ => value.floor(),
        };
        if long {
            if !rounded.is_finite()
                || rounded < i64::MIN as f64
                || rounded >= 9_223_372_036_854_775_808.0
            {
                self.raise_fpu_condition(4);
                if !self.exception_raised {
                    let _ = self.write_fpr_double(fd, i64::MIN as u64);
                }
                return;
            }
            if !self.write_fpr_double(fd, (rounded as i64) as u64) {
                self.take_exception(Exception::ReservedInstruction, None);
            }
        } else {
            if !rounded.is_finite() || rounded < f64::from(i32::MIN) || rounded >= 2_147_483_648.0 {
                self.raise_fpu_condition(4);
                if !self.exception_raised {
                    self.write_fpr_word(fd, i32::MIN as u32);
                }
                return;
            }
            self.write_fpr_word(fd, rounded as i32 as u32);
        }
    }

    fn raise_fpu_condition(&mut self, condition: u32) {
        let flag = 1u32 << (2 + condition);
        let enable = 1u32 << (7 + condition);
        let cause = 1u32 << (12 + condition);
        self.fcr31 |= cause;
        if self.fcr31 & enable != 0 {
            self.take_exception(Exception::FloatingPoint, None);
        } else {
            self.fcr31 |= flag;
        }
    }

    fn write_cop0(&mut self, rd: usize, value: u64) {
        match rd {
            COP0_INDEX => self.cop0[rd] = value & 0x8000_003f,
            COP0_RANDOM => {}
            COP0_ENTRY_LO0 | COP0_ENTRY_LO1 => self.cop0[rd] = value & 0x3fff_ffff,
            COP0_CONTEXT => self.cop0[rd] = value,
            COP0_PAGE_MASK => self.cop0[rd] = value & 0x01ff_e000,
            COP0_WIRED => {
                self.cop0[rd] = value & 0x3f;
                self.cop0[COP0_RANDOM] = 31;
            }
            COP0_COUNT => self.cop0[rd] = value & 0xffff_ffff,
            COP0_ENTRY_HI => self.cop0[rd] = value & 0xc000_00ff_ffff_e0ff,
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
                const WRITABLE: u64 = 0x0f00_800f;
                self.cop0[rd] = (self.cop0[rd] & !WRITABLE) | (value & WRITABLE);
            }
            COP0_LLADDR => self.cop0[rd] = value & 0xffff_ffff,
            COP0_PRID | COP0_BADVADDR => {}
            _ => self.cop0[rd] = value,
        }
    }

    fn tlb_index(&self) -> usize {
        (self.cop0[COP0_INDEX] as usize) & 31
    }

    fn write_tlb_at(&mut self, index: usize) {
        let mut page_mask = self.cop0[COP0_PAGE_MASK] as u32 & 0x0155_4000;
        page_mask |= page_mask >> 1;
        self.tlb[index & 31] = TlbEntry {
            page_mask,
            entry_hi: self.cop0[COP0_ENTRY_HI] & 0xc000_00ff_ffff_e0ff,
            entry_lo0: self.cop0[COP0_ENTRY_LO0],
            entry_lo1: self.cop0[COP0_ENTRY_LO1],
        };
    }

    fn tlbwi(&mut self) {
        self.write_tlb_at(self.tlb_index());
    }

    fn tlbwr(&mut self) {
        self.write_tlb_at((self.cop0[COP0_RANDOM] as usize).min(31));
    }
    fn tlbr(&mut self) {
        let entry = self.tlb[self.tlb_index()];
        self.cop0[COP0_PAGE_MASK] = u64::from(entry.page_mask);
        self.cop0[COP0_ENTRY_HI] = entry.entry_hi;
        self.cop0[COP0_ENTRY_LO0] = entry.entry_lo0;
        self.cop0[COP0_ENTRY_LO1] = entry.entry_lo1;
    }

    fn tlbp(&mut self) {
        const VPN_MASK_40: u64 = 0x0000_00ff_ffff_e000;
        let target_hi = self.cop0[COP0_ENTRY_HI];
        let target_asid = target_hi as u8;
        let target_region = target_hi >> 62;
        self.cop0[COP0_INDEX] = 0x8000_0000;
        for (index, entry) in self.tlb.iter().copied().enumerate() {
            let pair_size = Self::tlb_page_size(&entry);
            let mask = (!(pair_size - 1)) & VPN_MASK_40;
            let global = entry.entry_lo0 & 1 != 0 && entry.entry_lo1 & 1 != 0;
            if (entry.entry_hi & mask) == (target_hi & mask)
                && entry.entry_hi >> 62 == target_region
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
        self.llbit = false;
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

    fn load<B: Mips64Bus>(&mut self, bus: &mut B, instruction: u32) {
        let opcode = instruction >> 26;
        let rt = Self::rt(instruction);
        if matches!(opcode, 0x1a | 0x1b | 0x34 | 0x37) && !self.require_64_bit_integer() {
            return;
        }
        let address = self.effective_address(instruction);
        match opcode {
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
            0x23 | 0x27 | 0x30 => {
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
                if opcode == 0x30 {
                    self.llbit = true;
                    self.cop0[COP0_LLADDR] = u64::from(physical >> 4);
                }
            }
            0x26 => self.load_word_right(bus, rt, address),
            0x1a => self.load_double_left(bus, rt, address),
            0x1b => self.load_double_right(bus, rt, address),
            0x31 => {
                if !self.cop1_usable() {
                    self.take_coprocessor_unusable(1);
                    return;
                }
                let Some(physical) = self.physical(address, 4, false) else {
                    return;
                };
                self.write_fpr_word(rt, bus.read32(physical));
            }
            0x34 | 0x37 => {
                let Some(physical) = self.physical(address, 8, false) else {
                    return;
                };
                self.write_reg(rt, bus.read64(physical));
                if opcode == 0x34 {
                    self.llbit = true;
                    self.cop0[COP0_LLADDR] = u64::from(physical >> 4);
                }
            }
            0x35 => {
                if !self.cop1_usable() {
                    self.take_coprocessor_unusable(1);
                    return;
                }
                let Some(physical) = self.physical(address, 8, false) else {
                    return;
                };
                if !self.write_fpr_double(rt, bus.read64(physical)) {
                    self.take_exception(Exception::ReservedInstruction, None);
                }
            }
            0x36 => self.take_coprocessor_unusable(2),
            _ => self.take_exception(Exception::ReservedInstruction, None),
        }
    }

    fn load_word_left<B: Mips64Bus>(&mut self, bus: &mut B, rt: u8, address: u64) {
        let aligned = address & !3;
        let Some(physical) = self.physical(aligned, 4, false) else {
            return;
        };
        let memory = bus.read32(physical);
        let shift = ((address & 3) * 8) as u32;
        let preserve = if shift == 0 { 0 } else { (1u32 << shift) - 1 };
        self.write_reg(
            rt,
            Self::sign32((self.reg(rt) as u32 & preserve) | memory.wrapping_shl(shift)),
        );
    }
    fn load_word_right<B: Mips64Bus>(&mut self, bus: &mut B, rt: u8, address: u64) {
        let aligned = address & !3;
        let Some(physical) = self.physical(aligned, 4, false) else {
            return;
        };
        let memory = bus.read32(physical);
        let shift = ((3 - (address & 3)) * 8) as u32;
        let preserve = if shift == 0 {
            0
        } else {
            u32::MAX << (32 - shift)
        };
        self.write_reg(
            rt,
            Self::sign32((self.reg(rt) as u32 & preserve) | (memory >> shift)),
        );
    }

    fn load_double_left<B: Mips64Bus>(&mut self, bus: &mut B, rt: u8, address: u64) {
        let aligned = address & !7;
        let Some(physical) = self.physical(aligned, 8, false) else {
            return;
        };
        let memory = bus.read64(physical);
        let shift = ((address & 7) * 8) as u32;
        let preserve = if shift == 0 { 0 } else { (1u64 << shift) - 1 };
        self.write_reg(rt, (self.reg(rt) & preserve) | memory.wrapping_shl(shift));
    }

    fn load_double_right<B: Mips64Bus>(&mut self, bus: &mut B, rt: u8, address: u64) {
        let aligned = address & !7;
        let Some(physical) = self.physical(aligned, 8, false) else {
            return;
        };
        let memory = bus.read64(physical);
        let shift = ((7 - (address & 7)) * 8) as u32;
        let preserve = if shift == 0 {
            0
        } else {
            u64::MAX << (64 - shift)
        };
        self.write_reg(rt, (self.reg(rt) & preserve) | (memory >> shift));
    }
    fn store<B: Mips64Bus>(&mut self, bus: &mut B, instruction: u32) {
        let opcode = instruction >> 26;
        let rt = Self::rt(instruction);
        if matches!(opcode, 0x2c | 0x2d | 0x3c | 0x3f) && !self.require_64_bit_integer() {
            return;
        }
        let address = self.effective_address(instruction);
        let value = self.reg(rt);
        match opcode {
            0x28 => {
                let Some(physical) = self.physical(address, 1, true) else {
                    return;
                };
                bus.write_sized(physical, 1, value);
                self.llbit = false;
            }
            0x29 => {
                let Some(physical) = self.physical(address, 2, true) else {
                    return;
                };
                bus.write_sized(physical, 2, value);
                self.llbit = false;
            }
            0x2a => self.store_word_left(bus, address, value as u32),
            0x2b => {
                let Some(physical) = self.physical(address, 4, true) else {
                    return;
                };
                bus.write_sized(physical, 4, value);
                self.llbit = false;
            }
            0x2c => self.store_double_left(bus, address, value),
            0x2d => self.store_double_right(bus, address, value),
            0x2e => self.store_word_right(bus, address, value as u32),
            0x2f => self.cache(address),
            0x38 => self.store_conditional(bus, rt, address, false),
            0x39 => {
                if !self.cop1_usable() {
                    self.take_coprocessor_unusable(1);
                    return;
                }
                let Some(physical) = self.physical(address, 4, true) else {
                    return;
                };
                bus.write_sized(physical, 4, u64::from(self.fpr_word(rt)));
                self.llbit = false;
            }
            0x3c => self.store_conditional(bus, rt, address, true),
            0x3d => {
                if !self.cop1_usable() {
                    self.take_coprocessor_unusable(1);
                    return;
                }
                let Some(bits) = self.fpr_double(rt) else {
                    self.take_exception(Exception::ReservedInstruction, None);
                    return;
                };
                let Some(physical) = self.physical(address, 8, true) else {
                    return;
                };
                bus.write_sized(physical, 8, bits);
                self.llbit = false;
            }
            0x3f => {
                let Some(physical) = self.physical(address, 8, true) else {
                    return;
                };
                bus.write_sized(physical, 8, value);
                self.llbit = false;
            }
            0x3a | 0x3e => self.take_coprocessor_unusable(2),
            _ => self.take_exception(Exception::ReservedInstruction, None),
        }
    }
    fn cache(&mut self, address: u64) {
        let _ = self.physical(address, 4, false);
    }

    fn store_conditional<B: Mips64Bus>(&mut self, bus: &mut B, rt: u8, address: u64, double: bool) {
        let align = if double { 8 } else { 4 };
        let Some(physical) = self.physical(address, align, true) else {
            return;
        };
        let value = self.reg(rt);
        let success = self.llbit && self.cop0[COP0_LLADDR] == u64::from(physical >> 4);
        self.llbit = false;
        if success {
            bus.write_sized(physical, if double { 8 } else { 4 }, value);
        }
        self.write_reg(rt, u64::from(success));
    }

    fn store_word_left<B: Mips64Bus>(&mut self, bus: &mut B, address: u64, value: u32) {
        let aligned = address & !3;
        let Some(physical) = self.physical(aligned, 4, true) else {
            return;
        };
        let memory = bus.read32(physical);
        let shift = ((address & 3) * 8) as u32;
        let preserve = if shift == 0 {
            0
        } else {
            u32::MAX << (32 - shift)
        };
        bus.write32(physical, (memory & preserve) | (value >> shift));
        self.llbit = false;
    }

    fn store_word_right<B: Mips64Bus>(&mut self, bus: &mut B, address: u64, value: u32) {
        let aligned = address & !3;
        let Some(physical) = self.physical(aligned, 4, true) else {
            return;
        };
        let memory = bus.read32(physical);
        let shift = ((3 - (address & 3)) * 8) as u32;
        let preserve = if shift == 0 { 0 } else { (1u32 << shift) - 1 };
        bus.write32(physical, (memory & preserve) | value.wrapping_shl(shift));
        self.llbit = false;
    }
    fn store_double_left<B: Mips64Bus>(&mut self, bus: &mut B, address: u64, value: u64) {
        let aligned = address & !7;
        let Some(physical) = self.physical(aligned, 8, true) else {
            return;
        };
        let memory = bus.read64(physical);
        let shift = ((address & 7) * 8) as u32;
        let preserve = if shift == 0 {
            0
        } else {
            u64::MAX << (64 - shift)
        };
        bus.write64(physical, (memory & preserve) | (value >> shift));
        self.llbit = false;
    }

    fn store_double_right<B: Mips64Bus>(&mut self, bus: &mut B, address: u64, value: u64) {
        let aligned = address & !7;
        let Some(physical) = self.physical(aligned, 8, true) else {
            return;
        };
        let memory = bus.read64(physical);
        let shift = ((7 - (address & 7)) * 8) as u32;
        let preserve = if shift == 0 { 0 } else { (1u64 << shift) - 1 };
        bus.write64(physical, (memory & preserve) | value.wrapping_shl(shift));
        self.llbit = false;
    }

    pub fn save(&self, out: &mut StateWriter) {
        for value in self.regs {
            out.u64(value);
        }
        out.u64(self.hi);
        out.u64(self.lo);
        out.u64(self.pc);
        out.u64(self.next_pc);
        for value in self.cop0 {
            out.u64(value);
        }
        for value in self.fpr {
            out.u64(value);
        }
        out.u32(self.fcr0);
        out.u32(self.fcr31);
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
        out.u8(u8::from(self.llbit));
        out.u8(u8::from(self.count_phase));
    }

    pub fn load_state(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        for value in &mut self.regs {
            *value = input.u64()?;
        }
        self.hi = input.u64()?;
        self.lo = input.u64()?;
        self.pc = input.u64()?;
        self.next_pc = input.u64()?;
        for value in &mut self.cop0 {
            *value = input.u64()?;
        }
        for value in &mut self.fpr {
            *value = input.u64()?;
        }
        self.fcr0 = input.u32()?;
        self.fcr31 = input.u32()?
            & (FCR31_RM_MASK | FCR31_FLAG_MASK | FCR31_CAUSE_MASK | FCR31_CONDITION | FCR31_FS);
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
        self.llbit = input.u8()? != 0;
        self.count_phase = input.u8()? != 0;
        self.regs[0] = 0;
        Ok(())
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
                bytes: vec![0; 16 * 1024 * 1024],
            }
        }
        fn put32(&mut self, address: usize, value: u32) {
            self.bytes[address..address + 4].copy_from_slice(&value.to_be_bytes());
        }
    }

    impl Mips64Bus for TestBus {
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

    fn c1(format: u32, ft: u8, fs: u8, fd: u8, function: u32) -> u32 {
        (0x11 << 26)
            | ((format & 31) << 21)
            | (u32::from(ft) << 16)
            | (u32::from(fs) << 11)
            | (u32::from(fd) << 6)
            | (function & 63)
    }

    #[test]
    fn cop0_prid_and_config_match_vr4300_revision_and_writable_fields() {
        let mut cpu = MipsR4300::default();
        assert_eq!(cpu.cop0[COP0_PRID], 0x0000_0b22);
        assert_eq!(cpu.cop0[COP0_CONFIG], 0x7006_e463);

        cpu.write_cop0(COP0_PRID, 0xffff_ffff_ffff_ffff);
        assert_eq!(cpu.cop0[COP0_PRID], 0x0000_0b22);

        const WRITABLE: u64 = 0x0f00_800f;
        let fixed = cpu.cop0[COP0_CONFIG] & !WRITABLE;
        cpu.write_cop0(COP0_CONFIG, 0);
        assert_eq!(cpu.cop0[COP0_CONFIG], fixed);
        cpu.write_cop0(COP0_CONFIG, u64::MAX);
        assert_eq!(cpu.cop0[COP0_CONFIG], fixed | WRITABLE);
        assert_eq!((cpu.cop0[COP0_CONFIG] >> 28) & 7, 7);
        assert_eq!((cpu.cop0[COP0_CONFIG] >> 16) & 0xff, 0x06);
    }

    #[test]
    fn cop0_privilege_and_doubleword_moves_follow_vr4300_mode_rules() {
        let cop0 = |rs: u32, rt: u8, rd: u8, function: u32| {
            (0x10u32 << 26)
                | ((rs & 31) << 21)
                | (u32::from(rt) << 16)
                | (u32::from(rd) << 11)
                | (function & 63)
        };

        let mut denied = MipsR4300::default();
        denied.reset_to(0xffff_ffff_8000_0100);
        denied.cop0[COP0_STATUS] = 2 << 3;
        denied.current_pc = denied.pc;
        denied.regs[2] = 0xfeed_face;
        denied.execute_cop0(cop0(0x00, 2, COP0_EPC as u8, 0));
        assert_eq!(denied.regs[2], 0xfeed_face);
        assert_eq!(
            (denied.cop0[COP0_CAUSE] >> 2) & 0x1f,
            Exception::CoprocessorUnusable.code()
        );
        assert_eq!((denied.cop0[COP0_CAUSE] >> 28) & 3, 0);

        let mut user32 = MipsR4300::default();
        user32.reset_to(0xffff_ffff_8000_0100);
        user32.cop0[COP0_STATUS] = (2 << 3) | STATUS_CU0;
        user32.cop0[COP0_EPC] = 0x0123_4567_89ab_cdef;
        user32.execute_cop0(cop0(0x00, 2, COP0_EPC as u8, 0));
        assert_eq!(user32.regs[2], 0xffff_ffff_89ab_cdef);

        user32.current_pc = user32.pc;
        user32.exception_raised = false;
        user32.execute_cop0(cop0(0x01, 3, COP0_EPC as u8, 0));
        assert_eq!(user32.regs[3], 0);
        assert_eq!(
            (user32.cop0[COP0_CAUSE] >> 2) & 0x1f,
            Exception::ReservedInstruction.code()
        );

        let mut user64 = MipsR4300::default();
        user64.reset_to(0xffff_ffff_8000_0100);
        user64.cop0[COP0_STATUS] = (2 << 3) | STATUS_CU0 | STATUS_UX;
        user64.cop0[COP0_EPC] = 0x0123_4567_89ab_cdef;
        user64.execute_cop0(cop0(0x01, 3, COP0_EPC as u8, 0));
        assert_eq!(user64.regs[3], 0x0123_4567_89ab_cdef);

        user64.regs[4] = 0xfedc_ba98_7654_3210;
        user64.execute_cop0(cop0(0x05, 4, COP0_EPC as u8, 0));
        assert_eq!(user64.cop0[COP0_EPC], 0xfedc_ba98_7654_3210);
    }

    #[test]
    fn invalid_cop0_encoding_is_reserved_instruction_not_coprocessor_unusable() {
        let mut cpu = MipsR4300::default();
        cpu.reset_to(0xffff_ffff_8000_0100);
        cpu.cop0[COP0_STATUS] = 0;
        cpu.current_pc = cpu.pc;
        let instruction = (0x10u32 << 26) | (0x02 << 21);
        cpu.execute_cop0(instruction);
        assert_eq!(
            (cpu.cop0[COP0_CAUSE] >> 2) & 0x1f,
            Exception::ReservedInstruction.code()
        );
        assert_eq!((cpu.cop0[COP0_CAUSE] >> 28) & 3, 0);
    }

    #[test]
    fn trap_register_instructions_raise_exc_code_thirteen_only_when_true() {
        let cases = [
            (0x30, 5u64, 3u64, true),
            (0x30, 2, 3, false),
            (0x31, u64::MAX, 1, true),
            (0x31, 1, u64::MAX, false),
            (0x32, (-5i64) as u64, (-3i64) as u64, true),
            (0x32, 4, 3, false),
            (0x33, 1, 2, true),
            (0x33, 2, 1, false),
            (0x34, 0x1234, 0x1234, true),
            (0x34, 0x1234, 0x5678, false),
            (0x36, 0x1234, 0x5678, true),
            (0x36, 0x1234, 0x1234, false),
        ];

        for (function, left, right, should_trap) in cases {
            let mut cpu = MipsR4300::default();
            cpu.reset_to(0xffff_ffff_8000_0100);
            cpu.current_pc = cpu.pc;
            cpu.regs[1] = left;
            cpu.regs[2] = right;
            cpu.execute_special(r(1, 2, 0, 0, function));

            assert_eq!(
                cpu.exception_raised, should_trap,
                "function {function:#04x}, left {left:#018x}, right {right:#018x}"
            );
            if should_trap {
                assert_eq!((cpu.cop0[COP0_CAUSE] >> 2) & 0x1f, Exception::Trap.code());
                assert_eq!(cpu.cop0[COP0_EPC], 0xffff_ffff_8000_0100);
            }
        }
    }

    #[test]
    fn trap_immediate_instructions_sign_extend_the_sixteen_bit_immediate() {
        let cases = [
            (0x08, 5u64, 3u16, true),
            (0x08, 2, 3, false),
            (0x09, 0, 0xffff, false),
            (0x09, u64::MAX, 0xffff, true),
            (0x0a, (-5i64) as u64, (-3i16) as u16, true),
            (0x0a, 4, 3, false),
            (0x0b, 0, 0xffff, true),
            (0x0b, u64::MAX, 0xffff, false),
            (0x0c, (-1i64) as u64, 0xffff, true),
            (0x0c, 1, 0xffff, false),
            (0x0e, 1, 0xffff, true),
            (0x0e, (-1i64) as u64, 0xffff, false),
        ];

        for (subopcode, value, immediate, should_trap) in cases {
            let mut cpu = MipsR4300::default();
            cpu.reset_to(0xffff_ffff_8000_0100);
            cpu.current_pc = cpu.pc;
            cpu.regs[1] = value;
            cpu.execute_regimm(i(0x01, 1, subopcode, immediate));

            assert_eq!(
                cpu.exception_raised, should_trap,
                "subopcode {subopcode:#04x}, value {value:#018x}, immediate {immediate:#06x}"
            );
            if should_trap {
                assert_eq!((cpu.cop0[COP0_CAUSE] >> 2) & 0x1f, Exception::Trap.code());
            }
        }
    }

    #[test]
    fn cache_instruction_preserves_address_and_tlb_fault_semantics() {
        let mut bus = TestBus::new();

        let mut kernel = MipsR4300::default();
        kernel.reset_to(0xffff_ffff_8000_0100);
        kernel.cop0[COP0_STATUS] = 0;
        kernel.regs[1] = 0xffff_ffff_8000_0200;
        kernel.store(&mut bus, i(0x2f, 1, 0, 0));
        assert!(!kernel.exception_raised);

        let mut misaligned = MipsR4300::default();
        misaligned.reset_to(0xffff_ffff_8000_0100);
        misaligned.cop0[COP0_STATUS] = 0;
        misaligned.current_pc = misaligned.pc;
        misaligned.regs[1] = 0xffff_ffff_8000_0201;
        misaligned.store(&mut bus, i(0x2f, 1, 0, 0));
        assert_eq!(
            (misaligned.cop0[COP0_CAUSE] >> 2) & 0x1f,
            Exception::AddressLoad.code()
        );

        let mut user_kernel_segment = MipsR4300::default();
        user_kernel_segment.reset_to(0x0040_0000);
        user_kernel_segment.cop0[COP0_STATUS] = 2 << 3;
        user_kernel_segment.current_pc = user_kernel_segment.pc;
        user_kernel_segment.regs[1] = 0xffff_ffff_8000_0200;
        user_kernel_segment.store(&mut bus, i(0x2f, 1, 0, 0));
        assert_eq!(
            (user_kernel_segment.cop0[COP0_CAUSE] >> 2) & 0x1f,
            Exception::AddressLoad.code()
        );

        let mut tlb_miss = MipsR4300::default();
        tlb_miss.reset_to(0xffff_ffff_8000_0100);
        tlb_miss.cop0[COP0_STATUS] = 0;
        tlb_miss.current_pc = tlb_miss.pc;
        tlb_miss.regs[1] = 0x0040_0000;
        tlb_miss.store(&mut bus, i(0x2f, 1, 0, 0));
        assert_eq!(
            (tlb_miss.cop0[COP0_CAUSE] >> 2) & 0x1f,
            Exception::TlbLoadMiss.code()
        );
        assert_eq!(tlb_miss.pc, 0xffff_ffff_8000_0000);
    }

    #[test]
    fn sixty_four_bit_integer_legality_follows_vr4300_execution_mode() {
        let daddu = r(1, 2, 3, 0, 0x2d);

        let mut kernel = MipsR4300::default();
        kernel.reset_to(0xffff_ffff_8000_0100);
        kernel.cop0[COP0_STATUS] = 0;
        kernel.regs[1] = 0x1_0000_0000;
        kernel.regs[2] = 7;
        kernel.execute_special(daddu);
        assert_eq!(kernel.regs[3], 0x1_0000_0007);
        assert!(!kernel.exception_raised);

        let mut user32 = MipsR4300::default();
        user32.reset_to(0xffff_ffff_8000_0100);
        user32.cop0[COP0_STATUS] = 2 << 3;
        user32.current_pc = user32.pc;
        user32.regs[1] = 1;
        user32.regs[2] = 2;
        user32.execute_special(daddu);
        assert_eq!(user32.regs[3], 0);
        assert_eq!(
            (user32.cop0[COP0_CAUSE] >> 2) & 0x1f,
            Exception::ReservedInstruction.code()
        );

        let mut user64 = MipsR4300::default();
        user64.reset_to(0xffff_ffff_8000_0100);
        user64.cop0[COP0_STATUS] = (2 << 3) | STATUS_UX;
        user64.regs[1] = 4;
        user64.regs[2] = 5;
        user64.execute_special(daddu);
        assert_eq!(user64.regs[3], 9);
        assert!(!user64.exception_raised);

        let mut supervisor32 = MipsR4300::default();
        supervisor32.reset_to(0xffff_ffff_8000_0100);
        supervisor32.cop0[COP0_STATUS] = 1 << 3;
        supervisor32.current_pc = supervisor32.pc;
        supervisor32.execute_special(daddu);
        assert_eq!(
            (supervisor32.cop0[COP0_CAUSE] >> 2) & 0x1f,
            Exception::ReservedInstruction.code()
        );

        let mut supervisor64 = MipsR4300::default();
        supervisor64.reset_to(0xffff_ffff_8000_0100);
        supervisor64.cop0[COP0_STATUS] = (1 << 3) | STATUS_SX;
        supervisor64.regs[1] = 10;
        supervisor64.regs[2] = 20;
        supervisor64.execute_special(daddu);
        assert_eq!(supervisor64.regs[3], 30);

        let mut exception_kernel = MipsR4300::default();
        exception_kernel.reset_to(0xffff_ffff_8000_0100);
        exception_kernel.cop0[COP0_STATUS] = (2 << 3) | STATUS_EXL;
        exception_kernel.regs[1] = 11;
        exception_kernel.regs[2] = 12;
        exception_kernel.execute_special(daddu);
        assert_eq!(exception_kernel.regs[3], 23);
    }

    #[test]
    fn sixty_four_bit_integer_memory_ops_fault_before_address_translation_in_32bit_user_mode() {
        let mut bus = TestBus::new();
        let mut cpu = MipsR4300::default();
        cpu.reset_to(0xffff_ffff_8000_0100);
        cpu.cop0[COP0_STATUS] = 2 << 3;
        cpu.current_pc = cpu.pc;
        cpu.regs[1] = 0xffff_ffff_8000_0300;
        cpu.regs[2] = 0x1122_3344_5566_7788;

        cpu.load(&mut bus, i(0x37, 1, 3, 0));
        assert_eq!(cpu.regs[3], 0);
        assert_eq!(
            (cpu.cop0[COP0_CAUSE] >> 2) & 0x1f,
            Exception::ReservedInstruction.code()
        );

        cpu.exception_raised = false;
        cpu.cop0[COP0_STATUS] = 2 << 3;
        cpu.current_pc = cpu.pc;
        cpu.store(&mut bus, i(0x3f, 1, 2, 0));
        assert_eq!(&bus.bytes[0x300..0x308], &[0; 8]);
        assert_eq!(
            (cpu.cop0[COP0_CAUSE] >> 2) & 0x1f,
            Exception::ReservedInstruction.code()
        );
    }

    #[test]
    fn sixty_four_bit_arithmetic_and_branch_likely_execute() {
        let mut bus = TestBus::new();
        let words = [
            i(0x19, 0, 1, 0xffff),
            r(0, 1, 2, 8, 0x38),
            i(0x14, 0, 1, 1),
            i(0x0d, 0, 3, 0x1234),
            i(0x0d, 0, 4, 0x5678),
        ];
        for (index, word) in words.into_iter().enumerate() {
            bus.put32(0x100 + index * 4, word);
        }
        let mut cpu = MipsR4300::default();
        cpu.reset_to(0xffff_ffff_8000_0100);
        for _ in 0..4 {
            cpu.step(&mut bus);
        }
        assert_eq!(cpu.regs[1], u64::MAX);
        assert_eq!(cpu.regs[2], u64::MAX << 8);
        assert_eq!(cpu.regs[3], 0);
        assert_eq!(cpu.regs[4], 0x5678);
    }
    #[test]
    fn big_endian_memory_unaligned_pairs_and_llsc_work() {
        let mut bus = TestBus::new();
        let words = [
            i(0x3f, 6, 2, 0),
            i(0x37, 6, 3, 0),
            i(0x2a, 1, 4, 0),
            i(0x2e, 1, 4, 3),
            i(0x22, 1, 5, 0),
            i(0x26, 1, 5, 3),
            i(0x30, 6, 7, 0),
            i(0x38, 6, 7, 0),
        ];
        for (index, word) in words.into_iter().enumerate() {
            bus.put32(0x100 + index * 4, word);
        }
        let mut cpu = MipsR4300::default();
        cpu.reset_to(0xffff_ffff_8000_0100);
        cpu.regs[1] = 0xffff_ffff_8000_0201;
        cpu.regs[2] = 0x1122_3344_5566_7788;
        cpu.regs[4] = 0x1122_3344;
        cpu.regs[6] = 0xffff_ffff_8000_0300;
        for _ in 0..6 {
            cpu.step(&mut bus);
        }
        assert_eq!(cpu.regs[3], 0x1122_3344_5566_7788);
        assert_eq!(&bus.bytes[0x201..0x205], &[0x11, 0x22, 0x33, 0x44]);
        assert_eq!(cpu.regs[5], 0x1122_3344);
        cpu.step(&mut bus);
        cpu.regs[7] = 0x5566_7788;
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[7], 1);
        assert_eq!(&bus.bytes[0x300..0x304], &[0x55, 0x66, 0x77, 0x88]);
    }
    #[test]
    fn cop1_single_double_arithmetic_compare_and_conversion_execute() {
        let mut cpu = MipsR4300::default();
        cpu.reset_to(0xffff_ffff_8000_0100);
        cpu.cop0[COP0_STATUS] |= STATUS_CU1 | STATUS_FR;
        cpu.write_fpr_word(1, 1.5f32.to_bits());
        cpu.write_fpr_word(2, 2.25f32.to_bits());
        cpu.execute_cop1(c1(0x10, 2, 1, 3, 0x00));
        assert_eq!(f32::from_bits(cpu.fpr_word(3)), 3.75);
        cpu.execute_cop1(c1(0x10, 2, 1, 0, 0x34));
        assert_ne!(cpu.fcr31 & FCR31_CONDITION, 0);
        cpu.execute_cop1(c1(0x10, 0, 3, 4, 0x0d));
        assert_eq!(cpu.fpr_word(4) as i32, 3);

        assert!(cpu.write_fpr_double(5, 9.0f64.to_bits()));
        assert!(cpu.write_fpr_double(6, 4.0f64.to_bits()));
        cpu.execute_cop1(c1(0x11, 6, 5, 7, 0x03));
        assert_eq!(f64::from_bits(cpu.fpr_double(7).unwrap()), 2.25);
        cpu.execute_cop1(c1(0x11, 0, 7, 8, 0x04));
        assert_eq!(f64::from_bits(cpu.fpr_double(8).unwrap()), 1.5);
    }

    #[test]
    fn cop1_memory_moves_and_fr0_word_aliasing_work() {
        let mut bus = TestBus::new();
        bus.put32(0x300, 1.25f32.to_bits());
        bus.bytes[0x308..0x310].copy_from_slice(&6.5f64.to_bits().to_be_bytes());
        let mut cpu = MipsR4300::default();
        cpu.reset_to(0xffff_ffff_8000_0100);
        cpu.cop0[COP0_STATUS] |= STATUS_CU1 | STATUS_FR;
        cpu.regs[1] = 0xffff_ffff_8000_0300;
        cpu.load(&mut bus, i(0x31, 1, 2, 0));
        cpu.load(&mut bus, i(0x35, 1, 4, 8));
        assert_eq!(f32::from_bits(cpu.fpr_word(2)), 1.25);
        assert_eq!(f64::from_bits(cpu.fpr_double(4).unwrap()), 6.5);
        cpu.regs[1] = 0xffff_ffff_8000_0320;
        cpu.store(&mut bus, i(0x39, 1, 2, 0));
        cpu.store(&mut bus, i(0x3d, 1, 4, 8));
        assert_eq!(
            u32::from_be_bytes(bus.bytes[0x320..0x324].try_into().unwrap()),
            1.25f32.to_bits()
        );
        assert_eq!(
            u64::from_be_bytes(bus.bytes[0x328..0x330].try_into().unwrap()),
            6.5f64.to_bits()
        );

        cpu.cop0[COP0_STATUS] &= !STATUS_FR;
        cpu.write_fpr_word(20, 0x1122_3344);
        cpu.write_fpr_word(21, 0x5566_7788);
        assert_eq!(cpu.fpr[20], 0x5566_7788_1122_3344);
    }

    #[test]
    fn cop1_unusable_sets_cause_coprocessor_field() {
        let mut cpu = MipsR4300::default();
        cpu.reset_to(0xffff_ffff_8000_0100);
        cpu.execute_cop1(c1(0x10, 2, 1, 3, 0x00));
        assert_eq!(
            (cpu.cop0[COP0_CAUSE] >> 2) & 0x1f,
            Exception::CoprocessorUnusable.code()
        );
        assert_eq!((cpu.cop0[COP0_CAUSE] >> 28) & 3, 1);
    }

    #[test]
    fn extended_addressing_supports_xkphys_and_region_aware_forty_bit_tlb_entries() {
        const VPN_MASK_40: u64 = 0x0000_00ff_ffff_e000;

        let mut kernel = MipsR4300::default();
        kernel.cop0[COP0_STATUS] = STATUS_KX;
        assert_eq!(
            kernel.translate(0x9000_0000_1234_5678, false),
            Ok(0x1234_5678)
        );
        assert_eq!(
            kernel.translate(0x9000_0001_1234_5678, false),
            Err(Exception::AddressLoad)
        );

        let mut user = MipsR4300::default();
        user.cop0[COP0_STATUS] = (2 << 3) | STATUS_UX;
        let user_address = 0x0000_0001_0040_0010u64;
        user.cop0[COP0_INDEX] = 0;
        user.cop0[COP0_ENTRY_HI] = (user_address & VPN_MASK_40) | 0x33;
        user.cop0[COP0_ENTRY_LO0] = (0x12345u64 << 6) | 0x07;
        user.cop0[COP0_ENTRY_LO1] = (0x12346u64 << 6) | 0x07;
        user.tlbwi();
        assert_eq!(user.translate(user_address, false), Ok(0x1234_5010));
        assert_eq!(
            user.translate(user_address + 0x1000, false),
            Ok(0x1234_6010)
        );

        let mut region = MipsR4300::default();
        region.cop0[COP0_STATUS] = STATUS_KX;
        let xksseg = 0x4000_0001_0040_0010u64;
        region.cop0[COP0_INDEX] = 0;
        region.cop0[COP0_ENTRY_HI] = (xksseg & (0xc000_0000_0000_0000 | VPN_MASK_40)) | 0x44;
        region.cop0[COP0_ENTRY_LO0] = (0x34567u64 << 6) | 0x07;
        region.cop0[COP0_ENTRY_LO1] = (0x34568u64 << 6) | 0x07;
        region.tlbwi();
        assert_eq!(region.translate(xksseg, false), Ok(0x3456_7010));
        assert_eq!(
            region.translate(xksseg & 0x3fff_ffff_ffff_ffff, false),
            Err(Exception::TlbLoadMiss)
        );

        region.cop0[COP0_ENTRY_HI] = (xksseg & (0xc000_0000_0000_0000 | VPN_MASK_40)) | 0x44;
        region.tlbp();
        assert_eq!(region.cop0[COP0_INDEX], 0);
    }

    #[test]
    fn tlb_faults_distinguish_miss_invalid_and_modification_and_capture_region() {
        const VPN_MASK_40: u64 = 0x0000_00ff_ffff_e000;
        let mut cpu = MipsR4300::default();
        cpu.cop0[COP0_STATUS] = STATUS_KX;
        cpu.cop0[COP0_ENTRY_HI] = 0x5a;

        let address = 0x4000_0002_0080_0010u64;
        assert_eq!(cpu.translate(address, false), Err(Exception::TlbLoadMiss));
        assert_eq!(cpu.cop0[COP0_BADVADDR], address);
        assert_eq!(cpu.cop0[COP0_ENTRY_HI] >> 62, 1);
        assert_eq!(cpu.cop0[COP0_ENTRY_HI] & VPN_MASK_40, address & VPN_MASK_40);
        assert_eq!((cpu.cop0[COP0_XCONTEXT] >> 31) & 3, 1);

        cpu.cop0[COP0_INDEX] = 0;
        cpu.cop0[COP0_ENTRY_HI] = (address & (0xc000_0000_0000_0000 | VPN_MASK_40)) | 0x5a;
        cpu.cop0[COP0_ENTRY_LO0] = 0x07 & !0x02;
        cpu.cop0[COP0_ENTRY_LO1] = 0x07;
        cpu.tlbwi();
        assert_eq!(
            cpu.translate(address, false),
            Err(Exception::TlbLoadInvalid)
        );

        cpu.cop0[COP0_ENTRY_LO0] = (0x200u64 << 6) | 0x03;
        cpu.tlbwi();
        assert_eq!(
            cpu.translate(address, true),
            Err(Exception::TlbModification)
        );
    }

    #[test]
    fn tlb_miss_uses_refill_vector_but_invalid_and_modification_use_general_vector() {
        let mut miss = MipsR4300::default();
        miss.cop0[COP0_STATUS] = 0;
        miss.current_pc = 0xffff_ffff_8000_1000;
        miss.take_exception(Exception::TlbLoadMiss, Some(0x0040_0000));
        assert_eq!(miss.pc, 0xffff_ffff_8000_0000);
        assert_eq!((miss.cop0[COP0_CAUSE] >> 2) & 0x1f, 2);

        let mut invalid = MipsR4300::default();
        invalid.cop0[COP0_STATUS] = 0;
        invalid.current_pc = 0xffff_ffff_8000_1000;
        invalid.take_exception(Exception::TlbLoadInvalid, Some(0x0040_0000));
        assert_eq!(invalid.pc, 0xffff_ffff_8000_0180);
        assert_eq!((invalid.cop0[COP0_CAUSE] >> 2) & 0x1f, 2);

        let mut modification = MipsR4300::default();
        modification.cop0[COP0_STATUS] = 0;
        modification.current_pc = 0xffff_ffff_8000_1000;
        modification.take_exception(Exception::TlbModification, Some(0x0040_0000));
        assert_eq!(modification.pc, 0xffff_ffff_8000_0180);
        assert_eq!((modification.cop0[COP0_CAUSE] >> 2) & 0x1f, 1);
    }

    #[test]
    fn privilege_modes_enforce_the_vr4300_32bit_segment_map() {
        let mut kernel = MipsR4300::default();
        kernel.cop0[COP0_STATUS] = 0;
        assert_eq!(kernel.translate(0xffff_ffff_8000_1234, false), Ok(0x1234));
        assert_eq!(kernel.translate(0xffff_ffff_a000_5678, true), Ok(0x5678));

        let mut supervisor = MipsR4300::default();
        supervisor.cop0[COP0_STATUS] = 1 << 3;
        supervisor.cop0[COP0_INDEX] = 0;
        supervisor.cop0[COP0_ENTRY_HI] = 0xffff_ffff_c000_0001;
        supervisor.cop0[COP0_ENTRY_LO0] = (3 << 6) | 0x07;
        supervisor.cop0[COP0_ENTRY_LO1] = (4 << 6) | 0x07;
        supervisor.tlbwi();
        assert_eq!(
            supervisor.translate(0xffff_ffff_c000_0010, false),
            Ok(0x3010)
        );
        assert_eq!(
            supervisor.translate(0xffff_ffff_8000_0010, false),
            Err(Exception::AddressLoad)
        );
        assert_eq!(
            supervisor.translate(0xffff_ffff_e000_0010, true),
            Err(Exception::AddressStore)
        );

        let mut user = MipsR4300::default();
        user.cop0[COP0_STATUS] = 2 << 3;
        user.cop0[COP0_INDEX] = 0;
        user.cop0[COP0_ENTRY_HI] = 0x0040_0001;
        user.cop0[COP0_ENTRY_LO0] = (5 << 6) | 0x07;
        user.cop0[COP0_ENTRY_LO1] = (6 << 6) | 0x07;
        user.tlbwi();
        assert_eq!(user.translate(0x0040_0010, false), Ok(0x5010));
        assert_eq!(
            user.translate(0xffff_ffff_8000_0010, false),
            Err(Exception::AddressLoad)
        );
        assert_eq!(
            user.translate(0xffff_ffff_c000_0010, false),
            Err(Exception::AddressLoad)
        );

        user.cop0[COP0_STATUS] |= STATUS_UX;
        assert_eq!(
            user.translate(0xffff_ffff_8000_0010, false),
            Err(Exception::AddressLoad)
        );
    }

    #[test]
    fn privilege_segment_faults_raise_address_exception_with_bad_vaddr() {
        let mut cpu = MipsR4300::default();
        cpu.reset_to(0x0040_0000);
        cpu.cop0[COP0_STATUS] = 2 << 3;
        cpu.current_pc = cpu.pc;
        let address = 0xffff_ffff_8000_1000;
        assert_eq!(cpu.physical(address, 4, false), None);
        assert_eq!(
            (cpu.cop0[COP0_CAUSE] >> 2) & 0x1f,
            Exception::AddressLoad.code()
        );
        assert_eq!(cpu.cop0[COP0_BADVADDR], address);
        assert_eq!(cpu.cop0[COP0_EPC], 0x0040_0000);
    }

    #[test]
    fn tlb_maps_even_and_odd_pages_and_enforces_dirty() {
        let mut bus = TestBus::new();
        bus.put32(0x3000, i(0x0d, 0, 8, 0x55aa));
        let mut cpu = MipsR4300::default();
        cpu.reset_to(0x0040_0000);
        cpu.cop0[COP0_INDEX] = 0;
        cpu.cop0[COP0_ENTRY_HI] = 0x0040_0001;
        cpu.cop0[COP0_ENTRY_LO0] = (3 << 6) | 0x07;
        cpu.cop0[COP0_ENTRY_LO1] = (4 << 6) | 0x03;
        cpu.tlbwi();
        assert_eq!(cpu.translate(0x0040_0010, false).unwrap(), 0x3010);
        assert_eq!(cpu.translate(0x0040_1010, false).unwrap(), 0x4010);
        assert_eq!(
            cpu.translate(0x0040_1010, true),
            Err(Exception::TlbModification)
        );
        cpu.cop0[COP0_ENTRY_HI] = 0x0040_0001;
        cpu.pc = 0x0040_0000;
        cpu.next_pc = 0x0040_0004;
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[8], 0x55aa);
    }

    #[test]
    fn count_compare_interrupt_and_eret_follow_cop0_state() {
        let mut bus = TestBus::new();
        bus.put32(0x100, 0);
        bus.put32(0x104, 0);
        let mut cpu = MipsR4300::default();
        cpu.reset_to(0xffff_ffff_8000_0100);
        cpu.cop0[COP0_STATUS] = STATUS_IE | (1 << 15);
        cpu.cop0[COP0_COMPARE] = 1;
        cpu.step(&mut bus);
        assert_ne!(cpu.cop0[COP0_CAUSE] & (1 << 15), 0);
        cpu.step(&mut bus);
        assert_ne!(cpu.cop0[COP0_STATUS] & STATUS_EXL, 0);
        assert_eq!(cpu.cop0[COP0_EPC], 0xffff_ffff_8000_0104);
        cpu.eret();
        assert_eq!(cpu.pc, 0xffff_ffff_8000_0104);
        assert_eq!(cpu.cop0[COP0_STATUS] & STATUS_EXL, 0);
    }
    #[test]
    fn state_round_trip_preserves_tlb_pipeline_and_llbit() {
        let mut cpu = MipsR4300::default();
        cpu.reset_to(0xffff_ffff_8000_1234);
        cpu.regs[1] = 0x1122_3344_5566_7788;
        cpu.hi = 0x8877_6655_4433_2211;
        cpu.lo = 0x0123_4567_89ab_cdef;
        cpu.cop0[COP0_ENTRY_HI] = 0x0040_0007;
        cpu.cop0[COP0_ENTRY_LO0] = (0x123 << 6) | 0x07;
        cpu.cop0[COP0_ENTRY_LO1] = (0x124 << 6) | 0x07;
        cpu.cop0[COP0_INDEX] = 3;
        cpu.fpr[7] = 0x400a_0000_0000_0000;
        cpu.fcr31 = FCR31_CONDITION | 2;
        cpu.tlbwi();
        cpu.llbit = true;
        cpu.branch_delay_next = true;
        cpu.count_phase = true;
        let mut writer = StateWriter::new(PlatformId::Nintendo64, 1);
        cpu.save(&mut writer);
        let bytes = writer.finish();
        let mut reader = StateReader::new(&bytes, PlatformId::Nintendo64, 1).unwrap();
        let mut restored = MipsR4300::default();
        restored.load_state(&mut reader).unwrap();
        reader.finish().unwrap();
        assert_eq!(restored.regs, cpu.regs);
        assert_eq!(restored.hi, cpu.hi);
        assert_eq!(restored.lo, cpu.lo);
        assert_eq!(restored.pc, cpu.pc);
        assert_eq!(restored.fpr, cpu.fpr);
        assert_eq!(restored.fcr31, cpu.fcr31);
        assert_eq!(restored.tlb[3], cpu.tlb[3]);
        assert!(restored.llbit && restored.branch_delay_next && restored.count_phase);
    }
}
