use crate::state::{StateReader, StateWriter};

const Z: u8 = 1;
const C: u8 = 2;
const N: u8 = 4;

pub trait JaguarRiscBus {
    fn read8(&mut self, address: u32) -> u8;
    fn write8(&mut self, address: u32, value: u8);

    fn read16(&mut self, address: u32) -> u16 {
        u16::from_be_bytes([self.read8(address), self.read8(address.wrapping_add(1))])
    }
    fn write16(&mut self, address: u32, value: u16) {
        let [high, low] = value.to_be_bytes();
        self.write8(address, high);
        self.write8(address.wrapping_add(1), low);
    }
    fn read32(&mut self, address: u32) -> u32 {
        (u32::from(self.read16(address)) << 16) | u32::from(self.read16(address.wrapping_add(2)))
    }
    fn write32(&mut self, address: u32, value: u32) {
        self.write16(address, (value >> 16) as u16);
        self.write16(address.wrapping_add(2), value as u16);
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JaguarRiscKind {
    Gpu,
    Dsp,
}

#[derive(Clone, Debug)]
pub struct JaguarRisc {
    pub regs: [u32; 32],
    pub alt: [u32; 32],
    pub pc: u32,
    pub cycles: u64,
    pub running: bool,
    flags: u8,
    accum: i64,
    high_data: u32,
    remain: u32,
    modulo: u32,
    matrix_control: u32,
    matrix_pointer: u32,
    data_organization: u32,
    div_control: u32,
    reg_page: bool,
    bank_one_active: bool,
    interrupt_mask: u8,
    interrupt_latch: u8,
    interrupt_imask: bool,
    pending_branch: Option<u32>,
    kind: JaguarRiscKind,
}

impl JaguarRisc {
    pub fn new(kind: JaguarRiscKind) -> Self {
        Self {
            regs: [0; 32],
            alt: [0; 32],
            pc: 0,
            cycles: 0,
            running: false,
            flags: 0,
            accum: 0,
            high_data: 0,
            remain: 0,
            modulo: 0,
            matrix_control: 0,
            matrix_pointer: 0,
            data_organization: 0,
            div_control: 0,
            reg_page: false,
            bank_one_active: false,
            interrupt_mask: 0,
            interrupt_latch: 0,
            interrupt_imask: false,
            pending_branch: None,
            kind,
        }
    }

    pub fn reset(&mut self, pc: u32) {
        let kind = self.kind;
        *self = Self::new(kind);
        self.pc = pc;
    }

    pub fn kind(&self) -> JaguarRiscKind {
        self.kind
    }
    pub fn zero(&self) -> bool {
        self.flags & Z != 0
    }
    pub fn carry(&self) -> bool {
        self.flags & C != 0
    }
    pub fn negative(&self) -> bool {
        self.flags & N != 0
    }
    pub fn flags_word(&self) -> u32 {
        let mut value = u32::from(self.flags);
        value |= u32::from(self.interrupt_imask) << 3;
        value |= u32::from(self.interrupt_mask & 0x1f) << 4;
        if self.reg_page {
            value |= 0x4000;
        }
        if self.kind == JaguarRiscKind::Dsp && self.interrupt_mask & 0x20 != 0 {
            value |= 1 << 16;
        }
        value
    }

    fn update_register_bank(&mut self) {
        let bank_one = self.reg_page && !self.interrupt_imask;
        if bank_one != self.bank_one_active {
            std::mem::swap(&mut self.regs, &mut self.alt);
            self.bank_one_active = bank_one;
        }
    }

    pub fn set_flags_word(&mut self, value: u32) {
        self.flags = value as u8 & (Z | C | N);
        self.reg_page = value & 0x4000 != 0;
        if value & 0x08 == 0 {
            self.interrupt_imask = false;
        }
        self.interrupt_mask = ((value >> 4) as u8) & 0x1f;
        self.interrupt_latch &= !(((value >> 9) as u8) & 0x1f);
        if self.kind == JaguarRiscKind::Dsp {
            self.interrupt_mask |= (((value >> 16) as u8) & 1) << 5;
            self.interrupt_latch &= !((((value >> 17) as u8) & 1) << 5);
        } else {
            self.interrupt_mask &= 0x1f;
            self.interrupt_latch &= 0x1f;
        }
        self.update_register_bank();
    }

    pub fn set_interrupt_line(&mut self, line: u8, asserted: bool) {
        let max_line = if self.kind == JaguarRiscKind::Dsp {
            5
        } else {
            4
        };
        if line > max_line {
            return;
        }
        let mask = 1u8 << line;
        self.interrupt_latch &= !mask;
        if asserted && self.interrupt_mask & mask != 0 {
            self.interrupt_latch |= mask;
        }
    }

    pub fn interrupt_latch(&self) -> u8 {
        self.interrupt_latch
    }

    fn service_interrupt<B: JaguarRiscBus>(&mut self, bus: &mut B) {
        if self.interrupt_imask || !self.running || self.pending_branch.is_some() {
            return;
        }
        let pending = self.interrupt_latch & self.interrupt_mask;
        let Some(line) = (0u8..=5).rev().find(|line| pending & (1 << line) != 0) else {
            return;
        };

        self.interrupt_imask = true;
        self.update_register_bank();
        self.regs[31] = self.regs[31].wrapping_sub(4);
        bus.write32(self.regs[31], self.pc.wrapping_sub(2));
        self.pc = match self.kind {
            JaguarRiscKind::Gpu => 0x00f0_3000u32,
            JaguarRiscKind::Dsp => 0x00f1_b000u32,
        }
        .wrapping_add(u32::from(line) * 0x10);
        self.regs[30] = self.pc;
    }
    pub fn matrix_control(&self) -> u32 {
        self.matrix_control
    }
    pub fn set_matrix_control(&mut self, value: u32) {
        self.matrix_control = value;
    }
    pub fn matrix_pointer(&self) -> u32 {
        self.matrix_pointer
    }
    pub fn set_matrix_pointer(&mut self, value: u32) {
        self.matrix_pointer = value & !3;
    }
    pub fn data_organization(&self) -> u32 {
        self.data_organization
    }
    pub fn set_data_organization(&mut self, value: u32) {
        self.data_organization = value;
    }
    pub fn modulo(&self) -> u32 {
        self.modulo
    }
    pub fn set_modulo(&mut self, value: u32) {
        self.modulo = value;
    }
    pub fn div_control(&self) -> u32 {
        self.div_control
    }
    pub fn set_div_control(&mut self, value: u32) {
        self.div_control = value;
    }
    pub fn high_data(&self) -> u32 {
        self.high_data
    }
    pub fn set_high_data(&mut self, value: u32) {
        self.high_data = value;
    }
    pub fn remain(&self) -> u32 {
        self.remain
    }
    pub fn set_remain(&mut self, value: u32) {
        self.remain = value;
    }

    fn set_zn(&mut self, value: u32) {
        self.flags &= !(Z | N);
        if value == 0 {
            self.flags |= Z;
        }
        if value & 0x8000_0000 != 0 {
            self.flags |= N;
        }
    }

    fn set_add_flags(&mut self, a: u32, b: u32, result: u32) {
        self.set_zn(result);
        if u64::from(a) + u64::from(b) > u64::from(u32::MAX) {
            self.flags |= C;
        } else {
            self.flags &= !C;
        }
    }
    fn set_sub_flags(&mut self, a: u32, b: u32, result: u32) {
        self.set_zn(result);
        if b > a {
            self.flags |= C;
        } else {
            self.flags &= !C;
        }
    }

    fn normalized_accum(&self, value: i64) -> i64 {
        if self.kind == JaguarRiscKind::Dsp {
            const MASK: i64 = (1i64 << 40) - 1;
            let raw = value & MASK;
            if raw & (1i64 << 39) != 0 {
                raw | !MASK
            } else {
                raw
            }
        } else {
            i64::from(value as i32)
        }
    }

    fn set_accum(&mut self, value: i64) {
        self.accum = self.normalized_accum(value);
    }

    fn quick(value: usize) -> u32 {
        if value == 0 {
            32
        } else {
            value as u32
        }
    }

    fn signed_quick(value: usize) -> i32 {
        if value & 0x10 != 0 {
            value as i32 - 32
        } else {
            value as i32
        }
    }

    fn branch_condition(&self, condition: usize) -> bool {
        let z = self.zero();
        let selected = if condition & 0x10 != 0 {
            self.negative()
        } else {
            self.carry()
        };
        if condition & 1 != 0 && z {
            return false;
        }
        if condition & 2 != 0 && !z {
            return false;
        }
        if condition & 4 != 0 && selected {
            return false;
        }
        if condition & 8 != 0 && !selected {
            return false;
        }
        true
    }

    fn schedule_branch(&mut self, target: u32, in_delay_slot: bool) {
        if !in_delay_slot {
            self.pending_branch = Some(target);
        }
    }
    fn shift_dynamic(&mut self, amount: i32, value: u32, arithmetic: bool) -> u32 {
        let result = if amount < 0 {
            let left = amount.unsigned_abs();
            if left >= 32 {
                0
            } else {
                value << left
            }
        } else if arithmetic {
            if amount >= 32 {
                if value & 0x8000_0000 != 0 {
                    u32::MAX
                } else {
                    0
                }
            } else {
                ((value as i32) >> amount) as u32
            }
        } else if amount >= 32 {
            0
        } else {
            value >> amount
        };
        self.flags = (self.flags & !C)
            | if amount < 0 {
                if value >> 31 != 0 {
                    C
                } else {
                    0
                }
            } else if value & 1 != 0 {
                C
            } else {
                0
            };
        self.set_zn(result);
        result
    }

    fn rotate(&mut self, amount: u32, value: u32) -> u32 {
        let result = value.rotate_right(amount & 31);
        self.flags = (self.flags & !C) | if value >> 31 != 0 { C } else { 0 };
        self.set_zn(result);
        result
    }

    fn normi(value: u32) -> u32 {
        let mut shifted = value;
        let mut count = 0i32;
        if shifted != 0 {
            while shifted & 0xffc0_0000 == 0 {
                shifted <<= 1;
                count -= 1;
            }
            while shifted & 0xff80_0000 != 0 {
                shifted >>= 1;
                count += 1;
            }
        }
        count as u32
    }
    fn saturate_unsigned(&mut self, bits: u32, value: u32) -> u32 {
        let maximum = if bits == 32 {
            u32::MAX
        } else {
            (1u32 << bits) - 1
        };
        let result = if (value as i32) < 0 {
            0
        } else {
            value.min(maximum)
        };
        self.set_zn(result);
        result
    }

    fn saturate_signed(&mut self, bits: u32, value: u32) -> u32 {
        let signed = value as i32 as i64;
        let maximum = (1i64 << (bits - 1)) - 1;
        let minimum = -(1i64 << (bits - 1));
        let result = signed.clamp(minimum, maximum) as i32 as u32;
        self.set_zn(result);
        result
    }

    fn load_long<B: JaguarRiscBus>(bus: &mut B, address: u32) -> u32 {
        bus.read32(address & !3)
    }

    fn store_long<B: JaguarRiscBus>(bus: &mut B, address: u32, value: u32) {
        bus.write32(address & !3, value);
    }

    fn is_local_address(&self, address: u32) -> bool {
        match self.kind {
            JaguarRiscKind::Gpu => (0x00f0_3000..=0x00f0_3fff).contains(&address),
            JaguarRiscKind::Dsp => (0x00f1_b000..=0x00f1_cfff).contains(&address),
        }
    }

    fn matrix_multiply<B: JaguarRiscBus>(&mut self, bus: &mut B, first: usize) -> u32 {
        let count = (self.matrix_control & 0x0f) as usize;
        let mut address = self.matrix_pointer;
        let mut sum = 0i64;
        for i in 0..count {
            let packed = self.alt[(first + (i >> 1)) & 31];
            let vector = if i & 1 == 0 {
                packed as i16
            } else {
                (packed >> 16) as i16
            };
            let matrix = bus.read16(address.wrapping_add(2)) as i16;
            sum = sum.wrapping_add(i64::from(vector) * i64::from(matrix));
            address = address.wrapping_add(if self.matrix_control & 0x10 != 0 {
                (count as u32) * 4
            } else {
                4
            });
        }
        sum as u32
    }

    pub fn step<B: JaguarRiscBus>(&mut self, bus: &mut B) -> u32 {
        if !self.running {
            self.cycles = self.cycles.wrapping_add(1);
            return 1;
        }
        self.service_interrupt(bus);
        let delayed_target = self.pending_branch.take();
        let in_delay_slot = delayed_target.is_some();
        let current = self.pc;
        let opcode = bus.read16(current);
        self.pc = self.pc.wrapping_add(2);
        let index = usize::from(opcode >> 10);
        let first = usize::from((opcode >> 5) & 0x1f);
        let second = usize::from(opcode & 0x1f);
        let rm = self.regs[first];
        let rn = self.regs[second];
        let mut used = 1u32;

        match index {
            0 => {
                let result = rn.wrapping_add(rm);
                self.set_add_flags(rn, rm, result);
                self.regs[second] = result;
            }
            1 => {
                let wide = u64::from(rn) + u64::from(rm) + u64::from(self.carry());
                let result = wide as u32;
                self.flags = (self.flags & !C) | if wide >> 32 != 0 { C } else { 0 };
                self.set_zn(result);
                self.regs[second] = result;
            }
            2 | 3 => {
                let addend = Self::quick(first);
                let result = rn.wrapping_add(addend);
                if index == 2 {
                    self.set_add_flags(rn, addend, result);
                }
                self.regs[second] = result;
            }
            4 => {
                let result = rn.wrapping_sub(rm);
                self.set_sub_flags(rn, rm, result);
                self.regs[second] = result;
            }
            5 => {
                let wide = u64::from(rn) + u64::from(!rm) + u64::from(!self.carry());
                let result = wide as u32;
                self.flags = (self.flags & !C) | if (wide >> 32) & 1 == 0 { C } else { 0 };
                self.set_zn(result);
                self.regs[second] = result;
            }
            6 | 7 => {
                let subtrahend = Self::quick(first);
                let result = rn.wrapping_sub(subtrahend);
                if index == 6 {
                    self.set_sub_flags(rn, subtrahend, result);
                }
                self.regs[second] = result;
            }
            8 => {
                let result = rn.wrapping_neg();
                self.set_sub_flags(0, rn, result);
                self.regs[second] = result;
            }
            9..=12 => {
                let result = match index {
                    9 => rn & rm,
                    10 => rn | rm,
                    11 => rn ^ rm,
                    _ => !rn,
                };
                self.regs[second] = result;
                self.set_zn(result);
            }
            13 => {
                self.flags = (self.flags & !Z) | if rn & (1u32 << first) == 0 { Z } else { 0 };
            }
            14 => {
                let result = rn | (1u32 << first);
                self.regs[second] = result;
                self.set_zn(result);
            }
            15 => {
                let result = rn & !(1u32 << first);
                self.regs[second] = result;
                self.set_zn(result);
            }
            16 => {
                let result = u32::from(rm as u16) * u32::from(rn as u16);
                self.regs[second] = result;
                self.set_zn(result);
            }
            17 => {
                let result = i32::from(rm as i16).wrapping_mul(i32::from(rn as i16)) as u32;
                self.regs[second] = result;
                self.set_zn(result);
            }
            18 => {
                let result = i64::from((rm as i16) as i32) * i64::from((rn as i16) as i32);
                self.set_accum(result);
                self.set_zn(result as u32);
            }
            19 => {
                self.regs[second] = self.accum as u32;
            }
            20 => {
                let product = i64::from((rm as i16) as i32) * i64::from((rn as i16) as i32);
                self.set_accum(self.accum.wrapping_add(product));
            }
            21 => {
                let mut quotient = rn;
                let mut remainder = 0u32;
                if self.div_control & 1 != 0 {
                    quotient <<= 16;
                    remainder = rn >> 16;
                }
                for _ in 0..32 {
                    let sign = remainder & 0x8000_0000;
                    remainder = (remainder << 1) | (quotient >> 31);
                    remainder = if sign != 0 {
                        remainder.wrapping_add(rm)
                    } else {
                        remainder.wrapping_sub(rm)
                    };
                    quotient = (quotient << 1) | ((!remainder) >> 31);
                }
                self.regs[second] = quotient;
                self.remain = remainder;
                used = 16;
            }
            22 => {
                let negative = rn & 0x8000_0000 != 0;
                self.flags = (self.flags & !C) | if negative { C } else { 0 };
                let result = if negative { rn.wrapping_neg() } else { rn };
                self.regs[second] = result;
                self.set_zn(result);
            }
            23 => {
                self.regs[second] = self.shift_dynamic(rm as i32, rn, false);
            }
            24 => {
                let amount = 32u32.saturating_sub(first as u32);
                let result = if amount >= 32 { 0 } else { rn << amount };
                self.flags = (self.flags & !C) | if rn >> 31 != 0 { C } else { 0 };
                self.regs[second] = result;
                self.set_zn(result);
            }
            25 => {
                let amount = Self::quick(first);
                let result = if amount >= 32 { 0 } else { rn >> amount };
                self.flags = (self.flags & !C) | if rn & 1 != 0 { C } else { 0 };
                self.regs[second] = result;
                self.set_zn(result);
            }
            26 => {
                self.regs[second] = self.shift_dynamic(rm as i32, rn, true);
            }
            27 => {
                let amount = Self::quick(first);
                let result = if amount >= 32 {
                    if rn & 0x8000_0000 != 0 {
                        u32::MAX
                    } else {
                        0
                    }
                } else {
                    ((rn as i32) >> amount) as u32
                };
                self.flags = (self.flags & !C) | if rn & 1 != 0 { C } else { 0 };
                self.regs[second] = result;
                self.set_zn(result);
            }
            28 => {
                self.regs[second] = self.rotate(rm & 31, rn);
            }
            29 => {
                self.regs[second] = self.rotate(Self::quick(first) & 31, rn);
            }
            30 => {
                let result = rn.wrapping_sub(rm);
                self.set_sub_flags(rn, rm, result);
            }
            31 => {
                let immediate = Self::signed_quick(first) as u32;
                let result = rn.wrapping_sub(immediate);
                self.set_sub_flags(rn, immediate, result);
            }
            32 => {
                if self.kind == JaguarRiscKind::Gpu {
                    self.regs[second] = self.saturate_unsigned(8, rn);
                } else {
                    let subtrahend = Self::quick(first);
                    let raw = rn.wrapping_sub(subtrahend);
                    let result = (raw & !self.modulo) | (rn & self.modulo);
                    self.set_sub_flags(rn, subtrahend, result);
                    self.regs[second] = result;
                }
            }
            33 => {
                self.regs[second] = if self.kind == JaguarRiscKind::Gpu {
                    self.saturate_unsigned(16, rn)
                } else {
                    self.saturate_signed(16, rn)
                };
            }
            34 => self.regs[second] = rm,
            35 => self.regs[second] = first as u32,
            36 => self.alt[second] = rm,
            37 => self.regs[second] = self.alt[first],
            38 => {
                let low = u32::from(bus.read16(self.pc));
                let high = u32::from(bus.read16(self.pc.wrapping_add(2)));
                self.pc = self.pc.wrapping_add(4);
                self.regs[second] = low | (high << 16);
                used = 3;
            }
            39 => {
                let value = if self.is_local_address(rm) {
                    Self::load_long(bus, rm)
                } else {
                    u32::from(bus.read8(rm))
                };
                self.regs[second] = value;
            }
            40 => {
                let value = if self.is_local_address(rm) {
                    Self::load_long(bus, rm)
                } else {
                    u32::from(bus.read16(rm & !1))
                };
                self.regs[second] = value;
            }
            41 => self.regs[second] = Self::load_long(bus, rm),
            42 => {
                if self.kind == JaguarRiscKind::Gpu {
                    let base = rm & !7;
                    self.high_data = bus.read32(base);
                    self.regs[second] = bus.read32(base.wrapping_add(4));
                } else {
                    let upper = self.accum >> 32;
                    let value = if upper < -1 {
                        0x8000_0000
                    } else if upper > 0 {
                        0x7fff_ffff
                    } else {
                        rn
                    };
                    self.regs[second] = value;
                    self.set_zn(value);
                }
            }
            43 | 44 => {
                let base = self.regs[if index == 43 { 14 } else { 15 }];
                let address = base.wrapping_add(Self::quick(first) << 2);
                self.regs[second] = Self::load_long(bus, address);
            }
            45 => {
                if self.is_local_address(rm) {
                    Self::store_long(bus, rm, rn & 0xff);
                } else {
                    bus.write8(rm, rn as u8);
                }
            }
            46 => {
                if self.is_local_address(rm) {
                    Self::store_long(bus, rm, rn & 0xffff);
                } else {
                    bus.write16(rm & !1, rn as u16);
                }
            }
            47 => Self::store_long(bus, rm, rn),
            48 => {
                if self.kind == JaguarRiscKind::Gpu {
                    let base = rm & !7;
                    bus.write32(base, self.high_data);
                    bus.write32(base.wrapping_add(4), rn);
                } else {
                    let value = rn.reverse_bits();
                    self.regs[second] = value;
                    self.set_zn(value);
                }
            }
            49 | 50 => {
                let base = self.regs[if index == 49 { 14 } else { 15 }];
                let address = base.wrapping_add(Self::quick(first) << 2);
                Self::store_long(bus, address, rn);
            }
            51 => self.regs[second] = current,
            52 => {
                if self.branch_condition(second) {
                    self.schedule_branch(rm, in_delay_slot);
                }
            }
            53 => {
                if self.branch_condition(second) {
                    let offset = Self::signed_quick(first).wrapping_mul(2) as u32;
                    let target = self.pc.wrapping_add(offset);
                    self.schedule_branch(target, in_delay_slot);
                }
            }
            54 => {
                let value = self.matrix_multiply(bus, first);
                self.regs[second] = value;
                self.set_zn(value);
            }
            55 => {
                let value = (((rm as i32) >> 8) as u32 & 0xff80_0000) | (rm & 0x007f_ffff);
                self.regs[second] = value;
                self.set_zn(value);
            }
            56 => {
                let value = Self::normi(rm);
                self.regs[second] = value;
                self.set_zn(value);
            }
            57 => {}
            58 | 59 => {
                let base = self.regs[if index == 58 { 14 } else { 15 }];
                self.regs[second] = Self::load_long(bus, base.wrapping_add(rm));
            }
            60 | 61 => {
                let base = self.regs[if index == 60 { 14 } else { 15 }];
                Self::store_long(bus, base.wrapping_add(rm), rn);
            }
            62 => {
                if self.kind == JaguarRiscKind::Gpu {
                    self.regs[second] = self.saturate_unsigned(24, rn);
                }
            }
            63 => {
                if self.kind == JaguarRiscKind::Gpu {
                    let value = if first == 0 {
                        ((rn >> 10) & 0x0000_f000) | ((rn >> 5) & 0x0000_0f00) | (rn & 0xff)
                    } else {
                        ((rn & 0x0000_f000) << 10) | ((rn & 0x0000_0f00) << 5) | (rn & 0xff)
                    };
                    self.regs[second] = value;
                } else {
                    let addend = Self::quick(first);
                    let raw = rn.wrapping_add(addend);
                    let value = (raw & !self.modulo) | (rn & self.modulo);
                    self.regs[second] = value;
                    self.set_add_flags(rn, addend, value);
                }
            }
            _ => unreachable!(),
        }

        if let Some(target) = delayed_target {
            self.pc = target;
        }
        self.cycles = self.cycles.wrapping_add(u64::from(used));
        used
    }
}

impl JaguarRisc {
    pub fn save(&self, out: &mut StateWriter) {
        for value in self.regs {
            out.u32(value);
        }
        for value in self.alt {
            out.u32(value);
        }
        out.u32(self.pc);
        out.u64(self.cycles);
        out.u8(u8::from(self.running));
        out.u8(self.flags);
        out.u64(self.accum as u64);
        out.u32(self.high_data);
        out.u32(self.remain);
        out.u32(self.modulo);
        out.u32(self.matrix_control);
        out.u32(self.matrix_pointer);
        out.u32(self.data_organization);
        out.u32(self.div_control);
        out.u8(u8::from(self.reg_page));
        out.u8(u8::from(self.bank_one_active));
        out.u8(self.interrupt_mask);
        out.u8(self.interrupt_latch);
        out.u8(u8::from(self.interrupt_imask));
        out.u8(u8::from(self.pending_branch.is_some()));
        out.u32(self.pending_branch.unwrap_or(0));
    }

    pub fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        for value in &mut self.regs {
            *value = input.u32()?;
        }
        for value in &mut self.alt {
            *value = input.u32()?;
        }
        self.pc = input.u32()?;
        self.cycles = input.u64()?;
        self.running = input.u8()? != 0;
        self.flags = input.u8()? & (Z | C | N);
        self.accum = self.normalized_accum(input.u64()? as i64);
        self.high_data = input.u32()?;
        self.remain = input.u32()?;
        self.modulo = input.u32()?;
        self.matrix_control = input.u32()?;
        self.matrix_pointer = input.u32()? & !3;
        self.data_organization = input.u32()?;
        self.div_control = input.u32()?;
        self.reg_page = input.u8()? != 0;
        self.bank_one_active = input.u8()? != 0;
        self.interrupt_mask = input.u8()?;
        self.interrupt_latch = input.u8()?;
        self.interrupt_imask = input.u8()? != 0;
        let valid_interrupts = if self.kind == JaguarRiscKind::Dsp {
            0x3f
        } else {
            0x1f
        };
        if self.interrupt_mask & !valid_interrupts != 0
            || self.interrupt_latch & !valid_interrupts != 0
        {
            return Err("Jaguar RISC state contains invalid interrupt bits".into());
        }
        let pending = input.u8()? != 0;
        let target = input.u32()?;
        self.pending_branch = pending.then_some(target);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::platform::PlatformId;
    use std::collections::BTreeMap;

    #[derive(Default)]
    struct TestBus {
        data: BTreeMap<u32, u8>,
    }

    impl JaguarRiscBus for TestBus {
        fn read8(&mut self, address: u32) -> u8 {
            self.data.get(&address).copied().unwrap_or(0)
        }
        fn write8(&mut self, address: u32, value: u8) {
            self.data.insert(address, value);
        }
    }

    fn opcode(index: u16, first: u16, second: u16) -> u16 {
        (index << 10) | ((first & 31) << 5) | (second & 31)
    }

    fn write_opcode(bus: &mut TestBus, address: u32, value: u16) {
        bus.write16(address, value);
    }

    #[test]
    fn arithmetic_flags_and_register_pages_execute() {
        let mut bus = TestBus::default();
        write_opcode(&mut bus, 0, opcode(0, 1, 2));
        write_opcode(&mut bus, 2, opcode(35, 31, 3));
        write_opcode(&mut bus, 4, opcode(6, 2, 3));
        let mut cpu = JaguarRisc::new(JaguarRiscKind::Gpu);
        cpu.running = true;
        cpu.regs[1] = 5;
        cpu.regs[2] = 7;
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[2], 12);
        assert!(!cpu.zero() && !cpu.negative());
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[3], 31);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[3], 29);

        cpu.set_flags_word(0x4000 | u32::from(Z));
        assert_eq!(cpu.regs[2], 0);
        assert_eq!(cpu.alt[2], 12);
        assert!(cpu.zero());
        cpu.regs[4] = 0x1234_5678;
        cpu.set_flags_word(0);
        assert_eq!(cpu.regs[2], 12);
        assert_eq!(cpu.alt[4], 0x1234_5678);
    }

    #[test]
    fn taken_branch_executes_exactly_one_delay_slot() {
        let mut bus = TestBus::default();
        write_opcode(&mut bus, 0, opcode(52, 1, 0));
        write_opcode(&mut bus, 2, opcode(35, 7, 2));
        write_opcode(&mut bus, 0x10, opcode(35, 9, 2));
        let mut cpu = JaguarRisc::new(JaguarRiscKind::Gpu);
        cpu.running = true;
        cpu.regs[1] = 0x10;

        cpu.step(&mut bus);
        assert_eq!(cpu.pc, 2);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[2], 7);
        assert_eq!(cpu.pc, 0x10);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[2], 9);
    }

    #[test]
    fn local_loads_and_phrase_transfers_follow_jaguar_bus_rules() {
        let mut bus = TestBus::default();
        write_opcode(&mut bus, 0, opcode(39, 1, 2));
        write_opcode(&mut bus, 2, opcode(39, 3, 4));
        write_opcode(&mut bus, 4, opcode(42, 5, 6));
        write_opcode(&mut bus, 6, opcode(48, 7, 8));
        bus.write8(0x100, 0xab);
        bus.write32(0x00f0_3000, 0x1122_3344);
        bus.write32(0x200, 0xaabb_ccdd);
        bus.write32(0x204, 0x0102_0304);

        let mut cpu = JaguarRisc::new(JaguarRiscKind::Gpu);
        cpu.running = true;
        cpu.regs[1] = 0x100;
        cpu.regs[3] = 0x00f0_3001;
        cpu.regs[5] = 0x200;
        cpu.regs[7] = 0x300;
        cpu.regs[8] = 0x5566_7788;
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[2], 0xab);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[4], 0x1122_3344);
        cpu.step(&mut bus);
        assert_eq!(cpu.high_data(), 0xaabb_ccdd);
        assert_eq!(cpu.regs[6], 0x0102_0304);
        cpu.set_high_data(0x99aa_bbcc);
        cpu.step(&mut bus);
        assert_eq!(bus.read32(0x300), 0x99aa_bbcc);
        assert_eq!(bus.read32(0x304), 0x5566_7788);
    }

    #[test]
    fn dsp_variant_opcodes_use_modulo_mirror_and_signed_saturation() {
        let mut bus = TestBus::default();
        write_opcode(&mut bus, 0, opcode(32, 1, 2));
        write_opcode(&mut bus, 2, opcode(63, 1, 2));
        write_opcode(&mut bus, 4, opcode(48, 0, 3));
        write_opcode(&mut bus, 6, opcode(33, 0, 4));
        write_opcode(&mut bus, 8, opcode(42, 0, 5));

        let mut cpu = JaguarRisc::new(JaguarRiscKind::Dsp);
        cpu.running = true;
        cpu.set_modulo(0xf0);
        cpu.regs[2] = 0x120;
        cpu.regs[3] = 1;
        cpu.regs[4] = 0x0001_0000;
        cpu.set_accum(1i64 << 32);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[2], 0x12f);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[2], 0x120);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[3], 0x8000_0000);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[4], 0x7fff);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[5], 0x7fff_ffff);
    }

    #[test]
    fn interrupt_entry_prioritizes_high_line_and_forces_bank_zero() {
        let mut bus = TestBus::default();
        write_opcode(&mut bus, 0x00f0_3030, opcode(57, 0, 0));

        let mut cpu = JaguarRisc::new(JaguarRiscKind::Gpu);
        cpu.running = true;
        cpu.pc = 0x00f0_3040;
        cpu.regs[31] = 0x400;
        cpu.set_flags_word(0x4000 | (1 << 5) | (1 << 7));
        assert!(cpu.bank_one_active);

        cpu.set_interrupt_line(1, true);
        cpu.set_interrupt_line(3, true);
        cpu.step(&mut bus);

        assert!(!cpu.bank_one_active);
        assert!(cpu.interrupt_imask);
        assert_eq!(cpu.pc, 0x00f0_3032);
        assert_eq!(cpu.regs[30], 0x00f0_3030);
        assert_eq!(cpu.regs[31], 0x3fc);
        assert_eq!(bus.read32(0x3fc), 0x00f0_303e);
        assert_eq!(cpu.interrupt_latch & 0x0a, 0x0a);

        cpu.set_flags_word(0x4000 | (1 << 5) | (1 << 7) | (1 << 12));
        assert!(!cpu.interrupt_imask);
        assert!(cpu.bank_one_active);
        assert_eq!(cpu.interrupt_latch & 0x08, 0);
        assert_ne!(cpu.interrupt_latch & 0x02, 0);
    }

    #[test]
    fn dsp_sixth_interrupt_line_uses_extended_flags_bits() {
        let mut bus = TestBus::default();
        write_opcode(&mut bus, 0x00f1_b050, opcode(57, 0, 0));

        let mut cpu = JaguarRisc::new(JaguarRiscKind::Dsp);
        cpu.running = true;
        cpu.pc = 0x00f1_b100;
        cpu.regs[31] = 0x500;
        cpu.set_flags_word(1 << 16);
        assert_ne!(cpu.flags_word() & (1 << 16), 0);

        cpu.set_interrupt_line(5, true);
        cpu.step(&mut bus);
        assert_eq!(cpu.pc, 0x00f1_b052);
        assert_eq!(cpu.regs[30], 0x00f1_b050);
        assert_eq!(bus.read32(0x4fc), 0x00f1_b0fe);
        assert_ne!(cpu.interrupt_latch() & 0x20, 0);

        cpu.set_flags_word((1 << 16) | (1 << 17));
        assert_eq!(cpu.interrupt_latch() & 0x20, 0);
        assert!(!cpu.interrupt_imask);
    }

    #[test]
    fn every_gpu_and_dsp_opcode_has_an_execution_path() {
        for kind in [JaguarRiscKind::Gpu, JaguarRiscKind::Dsp] {
            for index in 0u16..64 {
                let mut bus = TestBus::default();
                write_opcode(&mut bus, 0, opcode(index, 1, 2));
                bus.write32(0x100, 0x1234_5678);
                let mut cpu = JaguarRisc::new(kind);
                cpu.running = true;
                cpu.regs[1] = 0x100;
                cpu.regs[2] = 4;
                let used = cpu.step(&mut bus);
                assert!(used > 0, "opcode {index} did not consume time for {kind:?}");
                assert!(cpu.cycles > 0);
            }
        }
    }

    #[test]
    fn state_round_trip_preserves_execution_and_control_state() {
        let mut cpu = JaguarRisc::new(JaguarRiscKind::Gpu);
        cpu.running = true;
        cpu.pc = 0x00f0_3000;
        cpu.cycles = 0x1234_5678;
        cpu.regs[3] = 0x1111_2222;
        cpu.alt[4] = 0x3333_4444;
        cpu.set_flags_word(0x4000 | u32::from(Z | C));
        cpu.set_accum(-12345);
        cpu.set_high_data(0xaabb_ccdd);
        cpu.set_remain(0x0102_0304);
        cpu.set_modulo(0x0000_0ff0);
        cpu.set_matrix_control(0x13);
        cpu.set_matrix_pointer(0x00f0_3003);
        cpu.set_data_organization(0x7654_3210);
        cpu.set_div_control(1);
        cpu.pending_branch = Some(0x00f0_3010);

        let mut writer = StateWriter::new(PlatformId::Jaguar, 3);
        cpu.save(&mut writer);
        let bytes = writer.finish();
        let mut restored = JaguarRisc::new(JaguarRiscKind::Gpu);
        let mut reader = StateReader::new(&bytes, PlatformId::Jaguar, 3).unwrap();
        restored.load(&mut reader).unwrap();
        reader.finish().unwrap();
        assert_eq!(restored.regs, cpu.regs);
        assert_eq!(restored.alt, cpu.alt);
        assert_eq!(restored.pc, cpu.pc);
        assert_eq!(restored.cycles, cpu.cycles);
        assert_eq!(restored.running, cpu.running);
        assert_eq!(restored.flags_word(), cpu.flags_word());
        assert_eq!(restored.accum, cpu.accum);
        assert_eq!(restored.high_data(), cpu.high_data());
        assert_eq!(restored.remain(), cpu.remain());
        assert_eq!(restored.modulo(), cpu.modulo());
        assert_eq!(restored.matrix_control(), cpu.matrix_control());
        assert_eq!(restored.matrix_pointer(), cpu.matrix_pointer());
        assert_eq!(restored.data_organization(), cpu.data_organization());
        assert_eq!(restored.div_control(), cpu.div_control());
        assert_eq!(restored.pending_branch, cpu.pending_branch);
    }
}
