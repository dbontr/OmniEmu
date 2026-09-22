use crate::bus::Bus8;
use crate::cpu6502::Mos6502;
use crate::state::{StateReader, StateWriter};

const C: u8 = 0x01;
const Z: u8 = 0x02;
const I: u8 = 0x04;
const D: u8 = 0x08;
const B: u8 = 0x10;
const T: u8 = 0x20;
const V: u8 = 0x40;
const N: u8 = 0x80;

const IRQ2_VECTOR: u16 = 0xfff6;
const IRQ1_VECTOR: u16 = 0xfff8;
const TIMER_VECTOR: u16 = 0xfffa;
const NMI_VECTOR: u16 = 0xfffc;
const RESET_VECTOR: u16 = 0xfffe;
const TIMER_PERIOD: u32 = 1024;

pub trait Huc6280Bus: Send {
    fn read8(&mut self, address: u32) -> u8;
    fn write8(&mut self, address: u32, value: u8);

    fn write_st(&mut self, port: u8, value: u8) {
        self.write8(0x1fe000 + u32::from(port), value);
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HucInterrupt {
    Irq2,
    Irq1,
    Timer,
    Nmi,
}

impl HucInterrupt {
    const fn vector(self) -> u16 {
        match self {
            Self::Irq2 => IRQ2_VECTOR,
            Self::Irq1 => IRQ1_VECTOR,
            Self::Timer => TIMER_VECTOR,
            Self::Nmi => NMI_VECTOR,
        }
    }

    const fn mask(self) -> u8 {
        match self {
            Self::Irq2 => 0x01,
            Self::Irq1 => 0x02,
            Self::Timer => 0x04,
            Self::Nmi => 0,
        }
    }
}

#[derive(Debug, Clone)]
struct InternalIo {
    timer_reload: u8,
    timer_value: u8,
    timer_enabled: bool,
    timer_phase: u32,
    irq_mask: u8,
    irq_pending: u8,
}

impl Default for InternalIo {
    fn default() -> Self {
        Self {
            timer_reload: 0,
            timer_value: 0,
            timer_enabled: false,
            timer_phase: TIMER_PERIOD,
            irq_mask: 0,
            irq_pending: 0,
        }
    }
}

impl InternalIo {
    fn read(&self, address: u32) -> Option<u8> {
        match address {
            0x1fec00..=0x1fefff => Some(if address & 1 == 0 {
                self.timer_value & 0x7f
            } else {
                u8::from(self.timer_enabled)
            }),
            0x1ff400..=0x1ff7ff => Some(match address & 3 {
                2 => self.irq_mask & 0x07,
                3 => self.irq_pending & 0x07,
                _ => 0,
            }),
            _ => None,
        }
    }

    fn write(&mut self, address: u32, value: u8) -> bool {
        match address {
            0x1fec00..=0x1fefff => {
                if address & 1 == 0 {
                    self.timer_reload = value & 0x7f;
                } else {
                    let enabled = value & 1 != 0;
                    if enabled && !self.timer_enabled {
                        self.timer_value = self.timer_reload;
                        self.timer_phase = TIMER_PERIOD;
                    }
                    self.timer_enabled = enabled;
                }
                true
            }
            0x1ff400..=0x1ff7ff => {
                match address & 3 {
                    2 => self.irq_mask = value & 0x07,
                    3 => self.irq_pending &= !0x04,
                    _ => {}
                }
                true
            }
            _ => false,
        }
    }

    fn tick(&mut self, clocks: u32) {
        if !self.timer_enabled {
            return;
        }
        let mut clocks = clocks;
        while clocks >= self.timer_phase {
            clocks -= self.timer_phase;
            self.timer_phase = TIMER_PERIOD;
            if self.timer_value == 0 {
                self.timer_value = self.timer_reload;
                self.irq_pending |= 0x04;
            } else {
                self.timer_value = self.timer_value.wrapping_sub(1) & 0x7f;
            }
        }
        self.timer_phase -= clocks;
    }

    fn request(&mut self, interrupt: HucInterrupt) {
        self.irq_pending |= interrupt.mask();
    }

    fn clear(&mut self, interrupt: HucInterrupt) {
        self.irq_pending &= !interrupt.mask();
    }

    fn highest_pending(&self) -> Option<HucInterrupt> {
        if self.irq_pending & 0x04 != 0 && self.irq_mask & 0x04 == 0 {
            Some(HucInterrupt::Timer)
        } else if self.irq_pending & 0x02 != 0 && self.irq_mask & 0x02 == 0 {
            Some(HucInterrupt::Irq1)
        } else if self.irq_pending & 0x01 != 0 && self.irq_mask & 0x01 == 0 {
            Some(HucInterrupt::Irq2)
        } else {
            None
        }
    }

    fn save(&self, out: &mut StateWriter) {
        out.u8(self.timer_reload);
        out.u8(self.timer_value);
        out.u8(u8::from(self.timer_enabled));
        out.u32(self.timer_phase);
        out.u8(self.irq_mask);
        out.u8(self.irq_pending);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.timer_reload = input.u8()? & 0x7f;
        self.timer_value = input.u8()? & 0x7f;
        self.timer_enabled = input.u8()? != 0;
        self.timer_phase = input.u32()?.clamp(1, TIMER_PERIOD);
        self.irq_mask = input.u8()? & 0x07;
        self.irq_pending = input.u8()? & 0x07;
        Ok(())
    }
}

struct MappedBus<'a, B> {
    bus: &'a mut B,
    mpr: [u8; 8],
    internal: &'a mut InternalIo,
}

impl<B: Huc6280Bus> MappedBus<'_, B> {
    fn translate(&self, address: u16) -> u32 {
        (u32::from(self.mpr[usize::from(address >> 13)]) << 13) | u32::from(address & 0x1fff)
    }

    fn read_logical(&mut self, address: u16) -> u8 {
        let physical = self.translate(address);
        self.internal
            .read(physical)
            .unwrap_or_else(|| self.bus.read8(physical))
    }

    fn write_logical(&mut self, address: u16, value: u8) {
        let physical = self.translate(address);
        if !self.internal.write(physical, value) {
            self.bus.write8(physical, value);
        }
    }
}

impl<B: Huc6280Bus> Bus8 for MappedBus<'_, B> {
    fn read8(&mut self, address: u16) -> u8 {
        self.read_logical(address)
    }

    fn write8(&mut self, address: u16, value: u8) {
        self.write_logical(address, value);
    }
}

#[derive(Debug, Clone)]
pub struct HuC6280 {
    pub core: Mos6502,
    pub mpr: [u8; 8],
    pub high_speed: bool,
    internal: InternalIo,
    master_clocks: u64,
}

impl Default for HuC6280 {
    fn default() -> Self {
        let mut core = Mos6502::default();
        core.set_page_bases(0x2000, 0x2100);
        core.p = I | B;
        core.sp = 0xff;
        Self {
            core,
            mpr: [0; 8],
            high_speed: false,
            internal: InternalIo::default(),
            master_clocks: 0,
        }
    }
}

impl HuC6280 {
    pub fn reset<B: Huc6280Bus>(&mut self, bus: &mut B) {
        *self = Self::default();
        let low = self.read_logical(bus, RESET_VECTOR);
        let high = self.read_logical(bus, RESET_VECTOR.wrapping_add(1));
        self.core.pc = u16::from_le_bytes([low, high]);
    }

    pub fn pc(&self) -> u16 {
        self.core.pc
    }
    pub fn master_clocks(&self) -> u64 {
        self.master_clocks
    }
    pub fn irq_mask(&self) -> u8 {
        self.internal.irq_mask
    }
    pub fn irq_pending(&self) -> u8 {
        self.internal.irq_pending
    }

    pub fn request_irq(&mut self, interrupt: HucInterrupt) {
        self.internal.request(interrupt);
    }
    pub fn clear_irq(&mut self, interrupt: HucInterrupt) {
        self.internal.clear(interrupt);
    }

    pub fn translate(&self, address: u16) -> u32 {
        (u32::from(self.mpr[usize::from(address >> 13)]) << 13) | u32::from(address & 0x1fff)
    }

    fn speed_multiplier(&self) -> u32 {
        if self.high_speed {
            1
        } else {
            4
        }
    }

    fn read_logical<B: Huc6280Bus>(&mut self, bus: &mut B, address: u16) -> u8 {
        let physical = self.translate(address);
        self.internal
            .read(physical)
            .unwrap_or_else(|| bus.read8(physical))
    }

    fn write_logical<B: Huc6280Bus>(&mut self, bus: &mut B, address: u16, value: u8) {
        let physical = self.translate(address);
        if !self.internal.write(physical, value) {
            bus.write8(physical, value);
        }
    }

    fn fetch8<B: Huc6280Bus>(&mut self, bus: &mut B) -> u8 {
        let value = self.read_logical(bus, self.core.pc);
        self.core.pc = self.core.pc.wrapping_add(1);
        value
    }

    fn fetch16<B: Huc6280Bus>(&mut self, bus: &mut B) -> u16 {
        let low = self.fetch8(bus);
        let high = self.fetch8(bus);
        u16::from_le_bytes([low, high])
    }

    fn direct_address(offset: u8) -> u16 {
        0x2000 | u16::from(offset)
    }
    fn direct_x(&self, offset: u8) -> u16 {
        Self::direct_address(offset.wrapping_add(self.core.x))
    }

    fn read_direct_pointer<B: Huc6280Bus>(&mut self, bus: &mut B, offset: u8) -> u16 {
        let low = self.read_logical(bus, Self::direct_address(offset));
        let high = self.read_logical(bus, Self::direct_address(offset.wrapping_add(1)));
        u16::from_le_bytes([low, high])
    }

    fn push8<B: Huc6280Bus>(&mut self, bus: &mut B, value: u8) {
        self.write_logical(bus, 0x2100 | u16::from(self.core.sp), value);
        self.core.sp = self.core.sp.wrapping_sub(1);
    }

    fn pop8<B: Huc6280Bus>(&mut self, bus: &mut B) -> u8 {
        self.core.sp = self.core.sp.wrapping_add(1);
        self.read_logical(bus, 0x2100 | u16::from(self.core.sp))
    }

    fn push16<B: Huc6280Bus>(&mut self, bus: &mut B, value: u16) {
        self.push8(bus, (value >> 8) as u8);
        self.push8(bus, value as u8);
    }

    fn pop16<B: Huc6280Bus>(&mut self, bus: &mut B) -> u16 {
        let low = self.pop8(bus);
        let high = self.pop8(bus);
        u16::from_le_bytes([low, high])
    }

    fn flag(&self, flag: u8) -> bool {
        self.core.p & flag != 0
    }

    fn set_flag(&mut self, flag: u8, enabled: bool) {
        if enabled {
            self.core.p |= flag;
        } else {
            self.core.p &= !flag;
        }
    }

    fn clear_t(&mut self) {
        self.core.p &= !T;
    }

    fn set_zn(&mut self, value: u8) {
        self.clear_t();
        self.set_flag(Z, value == 0);
        self.set_flag(N, value & 0x80 != 0);
    }

    fn compare(&mut self, left: u8, right: u8) {
        let result = left.wrapping_sub(right);
        self.clear_t();
        self.set_flag(C, left >= right);
        self.set_flag(Z, result == 0);
        self.set_flag(N, result & 0x80 != 0);
    }

    fn adc_value(&mut self, left: u8, right: u8) -> u8 {
        let carry = u16::from(self.flag(C));
        let binary = u16::from(left) + u16::from(right) + carry;
        let binary_result = binary as u8;
        if self.flag(D) {
            let mut low = u16::from(left & 0x0f) + u16::from(right & 0x0f) + carry;
            if low > 9 {
                low += 6;
            }
            let mut high = u16::from(left >> 4) + u16::from(right >> 4) + u16::from(low > 0x0f);
            if high > 9 {
                high += 6;
            }
            let result = (((high << 4) | (low & 0x0f)) & 0xff) as u8;
            self.set_flag(C, high > 0x0f);
            self.set_zn(result);
            return result;
        }
        self.set_flag(C, binary > 0xff);
        self.set_flag(V, (!(left ^ right) & (left ^ binary_result) & 0x80) != 0);
        self.set_zn(binary_result);
        binary_result
    }

    fn sbc_value(&mut self, left: u8, right: u8) -> u8 {
        let borrow = i16::from(!self.flag(C));
        let binary = i16::from(left) - i16::from(right) - borrow;
        let binary_result = binary as u8;
        if self.flag(D) {
            let mut low = i16::from(left & 0x0f) - i16::from(right & 0x0f) - borrow;
            let mut high = i16::from(left >> 4) - i16::from(right >> 4);
            if low < 0 {
                low -= 6;
                high -= 1;
            }
            if high < 0 {
                high -= 6;
            }
            let result = (((high << 4) & 0xf0) | (low & 0x0f)) as u8;
            self.set_flag(C, binary >= 0);
            self.set_zn(result);
            return result;
        }
        self.set_flag(C, binary >= 0);
        self.set_flag(V, ((left ^ binary_result) & (left ^ right) & 0x80) != 0);
        self.set_zn(binary_result);
        binary_result
    }

    fn finish_special(&mut self, cycles: u32) -> u32 {
        self.core.cycles = self.core.cycles.wrapping_add(u64::from(cycles));
        self.finish_clocks(cycles)
    }

    fn finish_clocks(&mut self, cycles: u32) -> u32 {
        let clocks = cycles.saturating_mul(self.speed_multiplier());
        self.internal.tick(clocks);
        self.master_clocks = self.master_clocks.wrapping_add(u64::from(clocks));
        clocks
    }

    fn take_interrupt<B: Huc6280Bus>(&mut self, bus: &mut B, interrupt: HucInterrupt) -> u32 {
        if interrupt != HucInterrupt::Nmi && self.flag(I) {
            return 0;
        }
        self.push16(bus, self.core.pc);
        self.push8(bus, self.core.p & !B);
        self.core.p = (self.core.p & !D) | I;
        let low = self.read_logical(bus, interrupt.vector());
        let high = self.read_logical(bus, interrupt.vector().wrapping_add(1));
        self.core.pc = u16::from_le_bytes([low, high]);
        self.internal.clear(interrupt);
        self.finish_special(7)
    }

    pub fn service_interrupts<B: Huc6280Bus>(&mut self, bus: &mut B) -> u32 {
        if self.flag(I) {
            return 0;
        }
        self.internal
            .highest_pending()
            .map_or(0, |interrupt| self.take_interrupt(bus, interrupt))
    }

    fn delegate<B: Huc6280Bus>(&mut self, bus: &mut B) -> u32 {
        let before = self.core.cycles;
        let mut mapped = MappedBus {
            bus,
            mpr: self.mpr,
            internal: &mut self.internal,
        };
        let cycles = self.core.step(&mut mapped);
        debug_assert_eq!(cycles, self.core.cycles.wrapping_sub(before) as u32);
        self.clear_t();
        self.finish_clocks(cycles)
    }

    fn t_capable(opcode: u8) -> bool {
        matches!(
            opcode,
            0x01 | 0x05
                | 0x09
                | 0x0d
                | 0x11
                | 0x12
                | 0x15
                | 0x19
                | 0x1d
                | 0x21
                | 0x25
                | 0x29
                | 0x2d
                | 0x31
                | 0x32
                | 0x35
                | 0x39
                | 0x3d
                | 0x41
                | 0x45
                | 0x49
                | 0x4d
                | 0x51
                | 0x52
                | 0x55
                | 0x59
                | 0x5d
                | 0x61
                | 0x65
                | 0x69
                | 0x6d
                | 0x71
                | 0x72
                | 0x75
                | 0x79
                | 0x7d
                | 0xe1
                | 0xe5
                | 0xe9
                | 0xed
                | 0xf1
                | 0xf2
                | 0xf5
                | 0xf9
                | 0xfd
        )
    }

    fn t_cycles(opcode: u8) -> u32 {
        match opcode & 0x1f {
            0x01 => 7,
            0x05 | 0x15 => 4,
            0x09 => 2,
            0x0d | 0x19 | 0x1d => 5,
            0x11 | 0x12 => 7,
            _ => 2,
        }
    }

    fn execute_t_alu<B: Huc6280Bus>(&mut self, bus: &mut B, opcode: u8) -> u32 {
        if opcode & 0x1f == 0x12 {
            self.fetch8(bus);
            let pointer = self.fetch8(bus);
            let address = self.read_direct_pointer(bus, pointer);
            let source = self.read_logical(bus, address);
            let target_address = Self::direct_address(self.core.x);
            let target = self.read_logical(bus, target_address);
            let result = match opcode & 0xe0 {
                0x00 => {
                    let value = target | source;
                    self.set_zn(value);
                    value
                }
                0x20 => {
                    let value = target & source;
                    self.set_zn(value);
                    value
                }
                0x40 => {
                    let value = target ^ source;
                    self.set_zn(value);
                    value
                }
                0x60 => self.adc_value(target, source),
                0xe0 => self.sbc_value(target, source),
                _ => target,
            };
            self.write_logical(bus, target_address, result);
            return self.finish_special(Self::t_cycles(opcode) + 3);
        }
        let original_a = self.core.a;
        let target_address = Self::direct_address(self.core.x);
        let target = self.read_logical(bus, target_address);
        self.core.a = target;
        let before = self.core.cycles;
        let mut mapped = MappedBus {
            bus,
            mpr: self.mpr,
            internal: &mut self.internal,
        };
        let _ = self.core.step(&mut mapped);
        let result = self.core.a;
        self.core.a = original_a;
        self.write_logical(bus, target_address, result);
        let cycles = Self::t_cycles(opcode) + 3;
        self.core.cycles = before.wrapping_add(u64::from(cycles));
        self.clear_t();
        self.finish_clocks(cycles)
    }

    fn zero_indirect<B: Huc6280Bus>(&mut self, bus: &mut B, opcode: u8) -> u32 {
        let pointer = self.fetch8(bus);
        let address = self.read_direct_pointer(bus, pointer);
        let value = self.read_logical(bus, address);
        match opcode {
            0x12 => {
                self.core.a |= value;
                self.set_zn(self.core.a);
            }
            0x32 => {
                self.core.a &= value;
                self.set_zn(self.core.a);
            }
            0x52 => {
                self.core.a ^= value;
                self.set_zn(self.core.a);
            }
            0x72 => {
                self.core.a = self.adc_value(self.core.a, value);
            }
            0x92 => self.write_logical(bus, address, self.core.a),
            0xb2 => {
                self.core.a = value;
                self.set_zn(value);
            }
            0xd2 => self.compare(self.core.a, value),
            0xf2 => {
                self.core.a = self.sbc_value(self.core.a, value);
            }
            _ => unreachable!(),
        }
        self.clear_t();
        self.finish_special(7)
    }

    fn bit_value(&mut self, value: u8, immediate: bool) {
        self.clear_t();
        self.set_flag(Z, self.core.a & value == 0);
        if !immediate {
            self.set_flag(N, value & 0x80 != 0);
            self.set_flag(V, value & 0x40 != 0);
        }
    }

    fn test_and_modify<B: Huc6280Bus>(&mut self, bus: &mut B, reset: bool, absolute: bool) -> u32 {
        let address = if absolute {
            self.fetch16(bus)
        } else {
            Self::direct_address(self.fetch8(bus))
        };
        let value = self.read_logical(bus, address);
        self.clear_t();
        self.set_flag(N, value & 0x80 != 0);
        self.set_flag(V, value & 0x40 != 0);
        let modified = if reset {
            value & !self.core.a
        } else {
            value | self.core.a
        };
        self.set_flag(Z, modified == 0);
        self.write_logical(bus, address, modified);
        self.finish_special(if absolute { 7 } else { 6 })
    }

    fn tst<B: Huc6280Bus>(&mut self, bus: &mut B, indexed: bool, absolute: bool) -> u32 {
        let immediate = self.fetch8(bus);
        let address = if absolute {
            let base = self.fetch16(bus);
            if indexed {
                base.wrapping_add(u16::from(self.core.x))
            } else {
                base
            }
        } else {
            let offset = self.fetch8(bus);
            if indexed {
                self.direct_x(offset)
            } else {
                Self::direct_address(offset)
            }
        };
        let value = self.read_logical(bus, address);
        self.clear_t();
        self.set_flag(N, value & 0x80 != 0);
        self.set_flag(V, value & 0x40 != 0);
        self.set_flag(Z, value & immediate == 0);
        self.finish_special(if absolute { 8 } else { 7 })
    }

    fn bit_branch<B: Huc6280Bus>(&mut self, bus: &mut B, opcode: u8) -> u32 {
        let offset = self.fetch8(bus);
        let value = self.read_logical(bus, Self::direct_address(offset));
        let relative = self.fetch8(bus) as i8;
        let bit = (opcode >> 4) & 7;
        let set_branch = opcode & 0x80 != 0;
        let condition = (value & (1 << bit) != 0) == set_branch;
        self.clear_t();
        if condition {
            self.core.pc = self.core.pc.wrapping_add_signed(i16::from(relative));
        }
        self.finish_special(if condition { 8 } else { 6 })
    }

    fn bit_memory<B: Huc6280Bus>(&mut self, bus: &mut B, opcode: u8) -> u32 {
        let offset = self.fetch8(bus);
        let address = Self::direct_address(offset);
        let value = self.read_logical(bus, address);
        let bit = (opcode >> 4) & 7;
        let result = if opcode & 0x80 != 0 {
            value | (1 << bit)
        } else {
            value & !(1 << bit)
        };
        self.write_logical(bus, address, result);
        self.clear_t();
        self.finish_special(7)
    }

    fn block_transfer<B: Huc6280Bus>(&mut self, bus: &mut B, opcode: u8) -> u32 {
        let source = self.fetch16(bus);
        let destination = self.fetch16(bus);
        let encoded = self.fetch16(bus);
        let length = if encoded == 0 {
            65_536u32
        } else {
            u32::from(encoded)
        };
        for index in 0..length {
            let (source_address, destination_address) = match opcode {
                0x73 => (
                    source.wrapping_add(index as u16),
                    destination.wrapping_add(index as u16),
                ),
                0xc3 => (
                    source.wrapping_sub(index as u16),
                    destination.wrapping_sub(index as u16),
                ),
                0xd3 => (source.wrapping_add(index as u16), destination),
                0xe3 => (
                    source.wrapping_add(index as u16),
                    destination.wrapping_add((index & 1) as u16),
                ),
                0xf3 => (
                    source.wrapping_add((index & 1) as u16),
                    destination.wrapping_add(index as u16),
                ),
                _ => unreachable!(),
            };
            let value = self.read_logical(bus, source_address);
            self.write_logical(bus, destination_address, value);
        }
        self.clear_t();
        self.finish_special(17u32.saturating_add(length.saturating_mul(6)))
    }

    fn mapper_transfer<B: Huc6280Bus>(&mut self, bus: &mut B, to_accumulator: bool) -> u32 {
        let mask = self.fetch8(bus);
        if to_accumulator {
            for index in 0..8 {
                if mask & (1 << index) != 0 {
                    self.core.a = self.mpr[index];
                }
            }
        } else {
            for index in 0..8 {
                if mask & (1 << index) != 0 {
                    self.mpr[index] = self.core.a;
                }
            }
        }
        self.clear_t();
        self.finish_special(if to_accumulator { 4 } else { 5 })
    }

    fn speed_change(&mut self, high_speed: bool) -> u32 {
        let old_multiplier = self.speed_multiplier();
        self.core.cycles = self.core.cycles.wrapping_add(3);
        let clocks = 3 * old_multiplier;
        self.internal.tick(clocks);
        self.master_clocks = self.master_clocks.wrapping_add(u64::from(clocks));
        self.high_speed = high_speed;
        self.clear_t();
        clocks
    }

    fn special_opcode(opcode: u8) -> bool {
        matches!(
            opcode,
            0x00 | 0x02
                | 0x03
                | 0x04
                | 0x08
                | 0x0c
                | 0x12
                | 0x13
                | 0x14
                | 0x1a
                | 0x1c
                | 0x22
                | 0x23
                | 0x28
                | 0x32
                | 0x33
                | 0x34
                | 0x3a
                | 0x3c
                | 0x40
                | 0x42
                | 0x43
                | 0x44
                | 0x52
                | 0x53
                | 0x54
                | 0x5a
                | 0x5c
                | 0x62
                | 0x63
                | 0x64
                | 0x6c
                | 0x72
                | 0x73
                | 0x74
                | 0x7a
                | 0x7c
                | 0x80
                | 0x82
                | 0x83
                | 0x89
                | 0x92
                | 0x93
                | 0x9c
                | 0xa3
                | 0x9e
                | 0xb2
                | 0xb3
                | 0xc2
                | 0xc3
                | 0xd2
                | 0xd3
                | 0xd4
                | 0xda
                | 0xdc
                | 0xe2
                | 0xe3
                | 0xf2
                | 0xf3
                | 0xf4
                | 0xfa
                | 0xfc
                | 0x0b
                | 0x2b
                | 0x4b
                | 0x6b
                | 0x8b
                | 0xab
                | 0xcb
                | 0xeb
                | 0x1b
                | 0x3b
                | 0x5b
                | 0x7b
                | 0x9b
                | 0xbb
                | 0xdb
                | 0xfb
        ) || opcode & 0x0f == 0x07
            || opcode & 0x0f == 0x0f
    }

    fn execute_special<B: Huc6280Bus>(&mut self, bus: &mut B, opcode: u8) -> u32 {
        if opcode & 0x0f == 0x07 {
            return self.bit_memory(bus, opcode);
        }
        if opcode & 0x0f == 0x0f {
            return self.bit_branch(bus, opcode);
        }
        match opcode {
            0x00 => {
                self.core.pc = self.core.pc.wrapping_add(1);
                self.push16(bus, self.core.pc);
                self.push8(bus, self.core.p | B);
                self.core.p = (self.core.p & !(D | T)) | I | B;
                let low = self.read_logical(bus, IRQ2_VECTOR);
                let high = self.read_logical(bus, IRQ2_VECTOR.wrapping_add(1));
                self.core.pc = u16::from_le_bytes([low, high]);
                self.finish_special(8)
            }
            0x08 => {
                self.push8(bus, self.core.p | B);
                self.clear_t();
                self.finish_special(3)
            }
            0x28 => {
                self.core.p = self.pop8(bus) | B;
                self.finish_special(4)
            }
            0x40 => {
                self.core.p = self.pop8(bus) | B;
                self.core.pc = self.pop16(bus);
                self.finish_special(7)
            }
            0x02 => {
                std::mem::swap(&mut self.core.x, &mut self.core.y);
                self.set_zn(self.core.x);
                self.finish_special(3)
            }
            0x22 => {
                std::mem::swap(&mut self.core.a, &mut self.core.x);
                self.set_zn(self.core.a);
                self.finish_special(3)
            }
            0x42 => {
                std::mem::swap(&mut self.core.a, &mut self.core.y);
                self.set_zn(self.core.a);
                self.finish_special(3)
            }
            0x62 => {
                self.core.a = 0;
                self.set_zn(0);
                self.finish_special(2)
            }
            0x82 => {
                self.core.x = 0;
                self.set_zn(0);
                self.finish_special(2)
            }
            0xc2 => {
                self.core.y = 0;
                self.set_zn(0);
                self.finish_special(2)
            }
            0x1a => {
                self.core.a = self.core.a.wrapping_add(1);
                self.set_zn(self.core.a);
                self.finish_special(2)
            }
            0x3a => {
                self.core.a = self.core.a.wrapping_sub(1);
                self.set_zn(self.core.a);
                self.finish_special(2)
            }
            0x5a => {
                self.push8(bus, self.core.y);
                self.clear_t();
                self.finish_special(3)
            }
            0x7a => {
                self.core.y = self.pop8(bus);
                self.set_zn(self.core.y);
                self.finish_special(4)
            }
            0xda => {
                self.push8(bus, self.core.x);
                self.clear_t();
                self.finish_special(3)
            }
            0xfa => {
                self.core.x = self.pop8(bus);
                self.set_zn(self.core.x);
                self.finish_special(4)
            }
            0x03 | 0x13 | 0x23 => {
                let value = self.fetch8(bus);
                let port = match opcode {
                    0x03 => 0,
                    0x13 => 2,
                    _ => 3,
                };
                bus.write_st(port, value);
                self.clear_t();
                self.finish_special(5)
            }
            0x43 => self.mapper_transfer(bus, true),
            0x53 => self.mapper_transfer(bus, false),
            0x54 => self.speed_change(false),
            0xd4 => self.speed_change(true),
            0xf4 => {
                self.core.p |= T;
                self.finish_special(2)
            }
            0x44 => {
                let relative = self.fetch8(bus) as i8;
                self.push16(bus, self.core.pc.wrapping_sub(1));
                self.core.pc = self.core.pc.wrapping_add_signed(i16::from(relative));
                self.clear_t();
                self.finish_special(8)
            }
            0x80 => {
                let relative = self.fetch8(bus) as i8;
                self.core.pc = self.core.pc.wrapping_add_signed(i16::from(relative));
                self.clear_t();
                self.finish_special(4)
            }
            0x04 => self.test_and_modify(bus, false, false),
            0x0c => self.test_and_modify(bus, false, true),
            0x14 => self.test_and_modify(bus, true, false),
            0x1c => self.test_and_modify(bus, true, true),
            0x34 => {
                let offset = self.fetch8(bus);
                let address = self.direct_x(offset);
                let value = self.read_logical(bus, address);
                self.bit_value(value, false);
                self.finish_special(4)
            }
            0x3c => {
                let address = self.fetch16(bus).wrapping_add(u16::from(self.core.x));
                let value = self.read_logical(bus, address);
                self.bit_value(value, false);
                self.finish_special(5)
            }
            0x89 => {
                let value = self.fetch8(bus);
                self.bit_value(value, true);
                self.finish_special(2)
            }
            0x64 | 0x74 | 0x9c | 0x9e => {
                let address = match opcode {
                    0x64 => Self::direct_address(self.fetch8(bus)),
                    0x74 => {
                        let offset = self.fetch8(bus);
                        self.direct_x(offset)
                    }
                    0x9c => self.fetch16(bus),
                    _ => self.fetch16(bus).wrapping_add(u16::from(self.core.x)),
                };
                self.write_logical(bus, address, 0);
                self.clear_t();
                self.finish_special(if opcode >= 0x9c { 5 } else { 4 })
            }
            0x6c => {
                let pointer = self.fetch16(bus);
                let low = self.read_logical(bus, pointer);
                let high = self.read_logical(bus, pointer.wrapping_add(1));
                self.core.pc = u16::from_le_bytes([low, high]);
                self.clear_t();
                self.finish_special(7)
            }
            0x7c => {
                let pointer = self.fetch16(bus).wrapping_add(u16::from(self.core.x));
                let low = self.read_logical(bus, pointer);
                let high = self.read_logical(bus, pointer.wrapping_add(1));
                self.core.pc = u16::from_le_bytes([low, high]);
                self.clear_t();
                self.finish_special(7)
            }
            0x12 | 0x32 | 0x52 | 0x72 | 0x92 | 0xb2 | 0xd2 | 0xf2 => {
                self.zero_indirect(bus, opcode)
            }
            0x83 => self.tst(bus, false, false),
            0xa3 => self.tst(bus, true, false),
            0x93 => self.tst(bus, false, true),
            0xb3 => self.tst(bus, true, true),
            0x73 | 0xc3 | 0xd3 | 0xe3 | 0xf3 => self.block_transfer(bus, opcode),
            0x33 | 0x63 | 0x5c | 0xdc | 0xfc | 0xe2 | 0x0b | 0x2b | 0x4b | 0x6b | 0x8b | 0xab
            | 0xcb | 0xeb | 0x1b | 0x3b | 0x5b | 0x7b | 0x9b | 0xbb | 0xdb | 0xfb => {
                self.clear_t();
                self.finish_special(2)
            }
            _ => unreachable!(),
        }
    }

    pub fn step<B: Huc6280Bus>(&mut self, bus: &mut B) -> u32 {
        let interrupt_clocks = self.service_interrupts(bus);
        if interrupt_clocks != 0 {
            return interrupt_clocks;
        }
        if self.core.stopped {
            return self.finish_special(2);
        }
        let opcode = self.read_logical(bus, self.core.pc);
        if self.flag(T) && Self::t_capable(opcode) {
            return self.execute_t_alu(bus, opcode);
        }
        if Self::special_opcode(opcode) {
            let fetched = self.fetch8(bus);
            debug_assert_eq!(fetched, opcode);
            return self.execute_special(bus, opcode);
        }
        self.delegate(bus)
    }

    pub fn save(&self, out: &mut StateWriter) {
        out.u16(self.core.pc);
        out.u8(self.core.sp);
        out.u8(self.core.a);
        out.u8(self.core.x);
        out.u8(self.core.y);
        out.u8(self.core.p);
        out.u64(self.core.cycles);
        out.u8(u8::from(self.core.stopped));
        out.blob(&self.mpr);
        out.u8(u8::from(self.high_speed));
        self.internal.save(out);
        out.u64(self.master_clocks);
    }

    pub fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let mut core = Mos6502::default();
        core.set_page_bases(0x2000, 0x2100);
        core.pc = input.u16()?;
        core.sp = input.u8()?;
        core.a = input.u8()?;
        core.x = input.u8()?;
        core.y = input.u8()?;
        core.p = input.u8()?;
        core.cycles = input.u64()?;
        core.stopped = input.u8()? != 0;
        self.core = core;
        let mpr = input.blob()?;
        if mpr.len() != 8 {
            return Err("HuC6280 state has invalid MPR count".into());
        }
        self.mpr.copy_from_slice(mpr);
        self.high_speed = input.u8()? != 0;
        self.internal.load(input)?;
        self.master_clocks = input.u64()?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::platform::PlatformId;

    struct TestBus {
        bytes: Vec<u8>,
        writes: Vec<(u32, u8)>,
    }

    impl Default for TestBus {
        fn default() -> Self {
            Self {
                bytes: vec![0; 1 << 21],
                writes: Vec::new(),
            }
        }
    }

    impl Huc6280Bus for TestBus {
        fn read8(&mut self, address: u32) -> u8 {
            self.bytes[address as usize]
        }
        fn write8(&mut self, address: u32, value: u8) {
            self.bytes[address as usize] = value;
            self.writes.push((address, value));
        }
    }

    fn reset_to(bus: &mut TestBus, address: u16) -> HuC6280 {
        let [low, high] = address.to_le_bytes();
        bus.bytes[0x1ffe] = low;
        bus.bytes[0x1fff] = high;
        let mut cpu = HuC6280::default();
        cpu.reset(bus);
        cpu
    }

    #[test]
    fn reset_and_mpr_translation_match_huc6280_map() {
        let mut bus = TestBus::default();
        let mut cpu = reset_to(&mut bus, 0x4000);
        assert_eq!(cpu.pc(), 0x4000);
        assert!(!cpu.high_speed);
        assert_eq!(cpu.translate(0x5abc), 0x1abc);
        bus.bytes[..8].copy_from_slice(&[0xa9, 0xf8, 0x53, 0x02, 0xa9, 0x5a, 0x85, 0x10]);
        for _ in 0..4 {
            cpu.step(&mut bus);
        }
        assert_eq!(cpu.mpr[1], 0xf8);
        assert_eq!(bus.bytes[0x1f0010], 0x5a);
        assert_eq!(bus.bytes[0x0010], 0);
    }

    #[test]
    fn t_flag_targets_direct_page_x_without_replacing_accumulator() {
        let mut bus = TestBus::default();
        let mut cpu = reset_to(&mut bus, 0x4000);
        bus.bytes[..7].copy_from_slice(&[0xa2, 0x10, 0xa9, 0x03, 0xf4, 0x69, 0x02]);
        bus.bytes[0x10] = 5;
        for _ in 0..4 {
            cpu.step(&mut bus);
        }
        assert_eq!(cpu.core.a, 3);
        assert_eq!(bus.bytes[0x10], 7);
        assert_eq!(cpu.core.p & T, 0);
    }

    #[test]
    fn block_transfer_and_st_ports_use_real_physical_targets() {
        let mut bus = TestBus::default();
        let mut cpu = reset_to(&mut bus, 0x4000);
        cpu.mpr[2] = 1;
        cpu.mpr[3] = 2;
        bus.bytes[0x2000..0x2009]
            .copy_from_slice(&[0x73, 0x00, 0x50, 0x00, 0x60, 0x03, 0x00, 0x03, 0x2a]);
        bus.bytes[0x3000..0x3003].copy_from_slice(&[1, 2, 3]);
        cpu.step(&mut bus);
        assert_eq!(&bus.bytes[0x4000..0x4003], &[1, 2, 3]);
        cpu.step(&mut bus);
        assert!(bus.writes.contains(&(0x1fe000, 0x2a)));
    }

    #[test]
    fn interrupt_priority_timer_and_state_are_deterministic() {
        let mut bus = TestBus::default();
        let mut cpu = reset_to(&mut bus, 0x4000);
        bus.bytes[0x1ffa] = 0x34;
        bus.bytes[0x1ffb] = 0x12;
        cpu.core.p &= !I;
        cpu.request_irq(HucInterrupt::Irq2);
        cpu.request_irq(HucInterrupt::Irq1);
        cpu.request_irq(HucInterrupt::Timer);
        assert_ne!(cpu.service_interrupts(&mut bus), 0);
        assert_eq!(cpu.pc(), 0x1234);
        assert_eq!(cpu.irq_pending(), 0x03);

        let mut writer = StateWriter::new(PlatformId::HomePong, 99);
        cpu.save(&mut writer);
        let bytes = writer.finish();
        let mut reader = StateReader::new(&bytes, PlatformId::HomePong, 99).unwrap();
        let mut restored = HuC6280::default();
        restored.load(&mut reader).unwrap();
        reader.finish().unwrap();
        assert_eq!(restored.pc(), cpu.pc());
        assert_eq!(restored.mpr, cpu.mpr);
        assert_eq!(restored.irq_pending(), cpu.irq_pending());
        assert_eq!(restored.master_clocks(), cpu.master_clocks());
    }

    #[test]
    fn internal_timer_registers_raise_timer_irq() {
        let mut bus = TestBus::default();
        let mut cpu = reset_to(&mut bus, 0x4000);
        cpu.mpr[6] = 0xff;
        cpu.write_logical(&mut bus, 0xcc00, 0);
        cpu.write_logical(&mut bus, 0xcc01, 1);
        cpu.internal.tick(TIMER_PERIOD);
        assert_eq!(cpu.irq_pending() & 0x04, 0x04);
        cpu.write_logical(&mut bus, 0xd403, 0);
        assert_eq!(cpu.irq_pending() & 0x04, 0);
    }
}
