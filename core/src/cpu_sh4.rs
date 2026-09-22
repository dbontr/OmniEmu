use crate::state::{StateReader, StateWriter};

const T: u32 = 1 << 0;
const S: u32 = 1 << 1;
const I_MASK: u32 = 0x0000_00f0;
const Q: u32 = 1 << 8;
const M: u32 = 1 << 9;
const FD: u32 = 1 << 15;
const BL: u32 = 1 << 28;
const RB: u32 = 1 << 29;
const MD: u32 = 1 << 30;
const SR_MASK: u32 = T | S | I_MASK | Q | M | FD | BL | RB | MD;
const FPSCR_PR: u32 = 1 << 19;
const FPSCR_SZ: u32 = 1 << 20;
const FPSCR_FR: u32 = 1 << 21;
const FPSCR_MASK: u32 = 0x003f_ffff;

pub trait Sh4Bus {
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
pub struct Sh4 {
    pub r: [u32; 16],
    pub r_bank: [u32; 8],
    pub pc: u32,
    pub pr: u32,
    pub gbr: u32,
    pub vbr: u32,
    pub dbr: u32,
    pub mach: u32,
    pub macl: u32,
    pub sr: u32,
    pub ssr: u32,
    pub spc: u32,
    pub sgr: u32,
    pub fr: [u32; 16],
    pub xf: [u32; 16],
    pub fpul: u32,
    pub fpscr: u32,
    pub expevt: u32,
    pub intevt: u32,
    pub tra: u32,
    pub cycles: u64,
    sleeping: bool,
    pending_branch: Option<u32>,
}

impl Default for Sh4 {
    fn default() -> Self {
        Self {
            r: [0; 16],
            r_bank: [0; 8],
            pc: 0,
            pr: 0,
            gbr: 0,
            vbr: 0,
            dbr: 0,
            mach: 0,
            macl: 0,
            sr: MD | RB | BL | I_MASK,
            ssr: 0,
            spc: 0,
            sgr: 0,
            fr: [0; 16],
            xf: [0; 16],
            fpul: 0,
            fpscr: 0x0004_0001,
            expevt: 0,
            intevt: 0,
            tra: 0,
            cycles: 0,
            sleeping: false,
            pending_branch: None,
        }
    }
}

impl Sh4 {
    pub fn reset(&mut self, pc: u32, vbr: u32, stack: u32) {
        *self = Self::default();
        self.pc = pc;
        self.vbr = vbr;
        self.r[15] = stack;
    }

    fn swap_gpr_bank(&mut self) {
        for index in 0..8 {
            std::mem::swap(&mut self.r[index], &mut self.r_bank[index]);
        }
    }

    fn set_sr(&mut self, value: u32) {
        let old_bank = self.sr & (MD | RB) == (MD | RB);
        let next = value & SR_MASK;
        let new_bank = next & (MD | RB) == (MD | RB);
        if old_bank != new_bank {
            self.swap_gpr_bank();
        }
        self.sr = next;
    }

    fn set_fpscr(&mut self, value: u32) {
        let next = value & FPSCR_MASK;
        if (self.fpscr ^ next) & FPSCR_FR != 0 {
            std::mem::swap(&mut self.fr, &mut self.xf);
        }
        self.fpscr = next;
    }

    fn fr32(&self, index: usize) -> f32 {
        f32::from_bits(self.fr[index & 15])
    }

    fn set_fr32(&mut self, index: usize, value: f32) {
        self.fr[index & 15] = value.to_bits();
    }

    fn t(&self) -> bool {
        self.sr & T != 0
    }

    fn set_t(&mut self, value: bool) {
        if value {
            self.sr |= T
        } else {
            self.sr &= !T
        }
    }

    fn set_bit(&mut self, mask: u32, value: bool) {
        if value {
            self.sr |= mask
        } else {
            self.sr &= !mask
        }
    }

    fn sign8(value: u8) -> u32 {
        i32::from(value as i8) as u32
    }

    fn sign12(value: u16) -> i32 {
        let raw = i32::from(value & 0x0fff);
        if raw & 0x0800 != 0 {
            raw | !0x0fff
        } else {
            raw
        }
    }

    fn branch_target8(pc_after_fetch: u32, disp: u8) -> u32 {
        let delta = i32::from(disp as i8) * 2;
        pc_after_fetch.wrapping_add(2).wrapping_add_signed(delta)
    }

    fn branch_target12(pc_after_fetch: u32, disp: u16) -> u32 {
        pc_after_fetch
            .wrapping_add(2)
            .wrapping_add_signed(Self::sign12(disp) * 2)
    }

    fn schedule_branch(&mut self, target: u32, in_delay_slot: bool) -> bool {
        if in_delay_slot {
            return false;
        }
        self.pending_branch = Some(target);
        true
    }

    pub fn interrupt<B: Sh4Bus>(&mut self, _bus: &mut B, level: u8, event: u16) -> u32 {
        let current = ((self.sr & I_MASK) >> 4) as u8;
        if level <= current || self.sr & BL != 0 {
            return 0;
        }
        self.sleeping = false;
        self.pending_branch = None;
        self.ssr = self.sr;
        self.spc = self.pc;
        self.sgr = self.r[15];
        self.intevt = u32::from(event);
        let next = (self.sr & !I_MASK) | (u32::from(level.min(15)) << 4) | MD | RB | BL;
        self.set_sr(next);
        self.pc = self.vbr.wrapping_add(0x600);
        self.cycles = self.cycles.wrapping_add(5);
        5
    }

    pub fn step<B: Sh4Bus>(&mut self, bus: &mut B) -> u32 {
        if self.sleeping {
            self.cycles = self.cycles.wrapping_add(1);
            return 1;
        }
        let delayed_target = self.pending_branch.take();
        let in_delay_slot = delayed_target.is_some();
        let opcode = bus.read16(self.pc);
        self.pc = self.pc.wrapping_add(2);
        let n = usize::from((opcode >> 8) & 0x0f);
        let m = usize::from((opcode >> 4) & 0x0f);
        let mut used = 1u32;
        let ok = match opcode >> 12 {
            0x0 => self.group0(bus, opcode, n, m, in_delay_slot, &mut used),
            0x1 => {
                let disp = u32::from(opcode & 0x0f) * 4;
                bus.write32(self.r[n].wrapping_add(disp), self.r[m]);
                true
            }
            0x2 => self.group2(bus, opcode, n, m, &mut used),
            0x3 => self.group3(opcode, n, m, &mut used),
            0x4 => self.group4(bus, opcode, n, m, in_delay_slot, &mut used),
            0x5 => {
                let disp = u32::from(opcode & 0x0f) * 4;
                self.r[n] = bus.read32(self.r[m].wrapping_add(disp));
                true
            }
            0x6 => self.group6(bus, opcode, n, m),
            0x7 => {
                self.r[n] = self.r[n].wrapping_add(Self::sign8(opcode as u8));
                true
            }
            0x8 => self.group8(bus, opcode, n, in_delay_slot, &mut used),
            0x9 => {
                let address = self
                    .pc
                    .wrapping_add(2)
                    .wrapping_add(u32::from(opcode as u8) * 2);
                self.r[n] = i32::from(bus.read16(address) as i16) as u32;
                true
            }
            0xa => {
                let target = Self::branch_target12(self.pc, opcode);
                used = 2;
                self.schedule_branch(target, in_delay_slot)
            }
            0xb => {
                self.pr = self.pc.wrapping_add(2);
                let target = Self::branch_target12(self.pc, opcode);
                used = 2;
                self.schedule_branch(target, in_delay_slot)
            }
            0xc => self.group_c(bus, opcode, &mut used),
            0xd => {
                let base = self.pc.wrapping_add(2) & !3;
                self.r[n] = bus.read32(base.wrapping_add(u32::from(opcode as u8) * 4));
                true
            }
            0xe => {
                self.r[n] = Self::sign8(opcode as u8);
                true
            }
            0xf => self.group_f(bus, opcode, n, m),
            _ => false,
        };
        if !ok {
            return 0;
        }
        if let Some(target) = delayed_target {
            self.pc = target;
        }
        self.cycles = self.cycles.wrapping_add(u64::from(used));
        used
    }

    fn group0<B: Sh4Bus>(
        &mut self,
        bus: &mut B,
        opcode: u16,
        n: usize,
        m: usize,
        in_delay: bool,
        used: &mut u32,
    ) -> bool {
        if opcode == 0x0008 {
            self.set_t(false);
            return true;
        }
        if opcode == 0x0018 {
            self.set_t(true);
            return true;
        }
        if opcode == 0x0009 {
            return true;
        }
        if opcode == 0x0019 {
            self.sr &= !(M | Q | T);
            return true;
        }
        if opcode == 0x0028 {
            self.mach = 0;
            self.macl = 0;
            return true;
        }
        if opcode == 0x001b {
            self.sleeping = true;
            *used = 3;
            return true;
        }
        if opcode == 0x000b {
            *used = 2;
            return self.schedule_branch(self.pr, in_delay);
        }
        if opcode == 0x002b {
            if in_delay || self.sr & MD == 0 {
                return false;
            }
            let target = self.spc;
            let restored = self.ssr;
            self.set_sr(restored);
            *used = 4;
            return self.schedule_branch(target, false);
        }
        match opcode & 0x00ff {
            0x02 => self.r[n] = self.sr,
            0x12 => self.r[n] = self.gbr,
            0x22 => self.r[n] = self.vbr,
            0x0a => self.r[n] = self.mach,
            0x1a => self.r[n] = self.macl,
            0x2a => self.r[n] = self.pr,
            0x29 => self.r[n] = u32::from(self.t()),
            0x03 => {
                self.pr = self.pc.wrapping_add(2);
                *used = 2;
                return self
                    .schedule_branch(self.pc.wrapping_add(2).wrapping_add(self.r[n]), in_delay);
            }
            0x23 => {
                *used = 2;
                return self
                    .schedule_branch(self.pc.wrapping_add(2).wrapping_add(self.r[n]), in_delay);
            }
            _ => match opcode & 0x000f {
                0x4 => bus.write8(self.r[n].wrapping_add(self.r[0]), self.r[m] as u8),
                0x5 => bus.write16(self.r[n].wrapping_add(self.r[0]), self.r[m] as u16),
                0x6 => bus.write32(self.r[n].wrapping_add(self.r[0]), self.r[m]),
                0x7 => self.macl = self.r[n].wrapping_mul(self.r[m]),
                0xc => {
                    self.r[n] = i32::from(bus.read8(self.r[m].wrapping_add(self.r[0])) as i8) as u32
                }
                0xd => {
                    self.r[n] =
                        i32::from(bus.read16(self.r[m].wrapping_add(self.r[0])) as i16) as u32
                }
                0xe => self.r[n] = bus.read32(self.r[m].wrapping_add(self.r[0])),
                0xf => {
                    let a = bus.read32(self.r[m]) as i32 as i64;
                    let b = bus.read32(self.r[n]) as i32 as i64;
                    self.r[m] = self.r[m].wrapping_add(4);
                    self.r[n] = self.r[n].wrapping_add(4);
                    let acc = ((u64::from(self.mach) << 32) | u64::from(self.macl)) as i64;
                    let value = acc.wrapping_add(a.wrapping_mul(b)) as u64;
                    self.mach = (value >> 32) as u32;
                    self.macl = value as u32;
                    *used = 3;
                }
                _ => return false,
            },
        }
        true
    }

    fn group2<B: Sh4Bus>(
        &mut self,
        bus: &mut B,
        opcode: u16,
        n: usize,
        m: usize,
        used: &mut u32,
    ) -> bool {
        match opcode & 0x000f {
            0x0 => bus.write8(self.r[n], self.r[m] as u8),
            0x1 => bus.write16(self.r[n], self.r[m] as u16),
            0x2 => bus.write32(self.r[n], self.r[m]),
            0x4 => {
                self.r[n] = self.r[n].wrapping_sub(1);
                bus.write8(self.r[n], self.r[m] as u8);
            }
            0x5 => {
                self.r[n] = self.r[n].wrapping_sub(2);
                bus.write16(self.r[n], self.r[m] as u16);
            }
            0x6 => {
                self.r[n] = self.r[n].wrapping_sub(4);
                bus.write32(self.r[n], self.r[m]);
            }
            0x7 => {
                self.set_bit(Q, self.r[n] & 0x8000_0000 != 0);
                self.set_bit(M, self.r[m] & 0x8000_0000 != 0);
                self.set_t((self.sr & Q != 0) == (self.sr & M != 0));
            }
            0x8 => self.set_t(self.r[n] & self.r[m] == 0),
            0x9 => self.r[n] &= self.r[m],
            0xa => self.r[n] ^= self.r[m],
            0xb => self.r[n] |= self.r[m],
            0xc => {
                let x = self.r[n] ^ self.r[m];
                self.set_t((0..4).any(|shift| ((x >> (shift * 8)) & 0xff) == 0));
            }
            0xd => self.r[n] = (self.r[n] >> 16) | (self.r[m] << 16),
            0xe => self.macl = u32::from(self.r[n] as u16) * u32::from(self.r[m] as u16),
            0xf => {
                self.macl = ((self.r[n] as i16 as i32).wrapping_mul(self.r[m] as i16 as i32)) as u32
            }
            _ => return false,
        }
        *used = 1;
        true
    }

    fn group3(&mut self, opcode: u16, n: usize, m: usize, used: &mut u32) -> bool {
        match opcode & 0x000f {
            0x0 => self.set_t(self.r[n] == self.r[m]),
            0x2 => self.set_t(self.r[n] >= self.r[m]),
            0x3 => self.set_t((self.r[n] as i32) >= (self.r[m] as i32)),
            0x4 => self.div1(n, m),
            0x5 => {
                let value = u64::from(self.r[n]) * u64::from(self.r[m]);
                self.mach = (value >> 32) as u32;
                self.macl = value as u32;
                *used = 2;
            }
            0x6 => self.set_t(self.r[n] > self.r[m]),
            0x7 => self.set_t((self.r[n] as i32) > (self.r[m] as i32)),
            0x8 => self.r[n] = self.r[n].wrapping_sub(self.r[m]),
            0xa => {
                let borrow = u32::from(self.t());
                let (v1, b1) = self.r[n].overflowing_sub(self.r[m]);
                let (v2, b2) = v1.overflowing_sub(borrow);
                self.r[n] = v2;
                self.set_t(b1 || b2);
            }
            0xb => {
                let a = self.r[n] as i32;
                let b = self.r[m] as i32;
                let value = a.wrapping_sub(b);
                self.r[n] = value as u32;
                self.set_t(((a ^ b) & (a ^ value)) < 0);
            }
            0xc => self.r[n] = self.r[n].wrapping_add(self.r[m]),
            0xd => {
                let value = (self.r[n] as i32 as i64).wrapping_mul(self.r[m] as i32 as i64) as u64;
                self.mach = (value >> 32) as u32;
                self.macl = value as u32;
                *used = 2;
            }
            0xe => {
                let carry = u32::from(self.t());
                let (v1, c1) = self.r[n].overflowing_add(self.r[m]);
                let (v2, c2) = v1.overflowing_add(carry);
                self.r[n] = v2;
                self.set_t(c1 || c2);
            }
            0xf => {
                let a = self.r[n] as i32;
                let b = self.r[m] as i32;
                let value = a.wrapping_add(b);
                self.r[n] = value as u32;
                self.set_t(((a ^ value) & (b ^ value)) < 0);
            }
            _ => return false,
        }
        true
    }

    fn group4<B: Sh4Bus>(
        &mut self,
        bus: &mut B,
        opcode: u16,
        n: usize,
        m: usize,
        in_delay: bool,
        used: &mut u32,
    ) -> bool {
        if opcode & 0x000f == 0x000f {
            let a = bus.read16(self.r[m]) as i16 as i32 as i64;
            let b = bus.read16(self.r[n]) as i16 as i32 as i64;
            self.r[m] = self.r[m].wrapping_add(2);
            self.r[n] = self.r[n].wrapping_add(2);
            let acc = ((u64::from(self.mach) << 32) | u64::from(self.macl)) as i64;
            let value = acc.wrapping_add(a.wrapping_mul(b)) as u64;
            self.mach = (value >> 32) as u32;
            self.macl = value as u32;
            *used = 3;
            return true;
        }
        match opcode & 0x00ff {
            0x00 | 0x20 => {
                let carry = self.r[n] & 0x8000_0000 != 0;
                self.r[n] <<= 1;
                self.set_t(carry);
            }
            0x01 => {
                let carry = self.r[n] & 1 != 0;
                self.r[n] >>= 1;
                self.set_t(carry);
            }
            0x02 => {
                self.r[n] = self.r[n].wrapping_sub(4);
                bus.write32(self.r[n], self.mach);
            }
            0x03 => {
                self.r[n] = self.r[n].wrapping_sub(4);
                bus.write32(self.r[n], self.sr);
            }
            0x04 => {
                let carry = self.r[n] & 0x8000_0000 != 0;
                self.r[n] = self.r[n].rotate_left(1);
                self.set_t(carry);
            }
            0x05 => {
                let carry = self.r[n] & 1 != 0;
                self.r[n] = self.r[n].rotate_right(1);
                self.set_t(carry);
            }
            0x06 => {
                self.mach = bus.read32(self.r[n]);
                self.r[n] = self.r[n].wrapping_add(4);
            }
            0x07 => {
                let value = bus.read32(self.r[n]);
                self.r[n] = self.r[n].wrapping_add(4);
                self.set_sr(value);
            }
            0x08 => self.r[n] <<= 2,
            0x09 => self.r[n] >>= 2,
            0x0a => self.mach = self.r[n],
            0x0b => {
                self.pr = self.pc.wrapping_add(2);
                *used = 2;
                return self.schedule_branch(self.r[n], in_delay);
            }
            0x0e => self.set_sr(self.r[n]),
            0x10 => {
                self.r[n] = self.r[n].wrapping_sub(1);
                self.set_t(self.r[n] == 0);
            }
            0x11 => self.set_t((self.r[n] as i32) >= 0),
            0x12 => {
                self.r[n] = self.r[n].wrapping_sub(4);
                bus.write32(self.r[n], self.macl);
            }
            0x13 => {
                self.r[n] = self.r[n].wrapping_sub(4);
                bus.write32(self.r[n], self.gbr);
            }
            0x15 => self.set_t((self.r[n] as i32) > 0),
            0x16 => {
                self.macl = bus.read32(self.r[n]);
                self.r[n] = self.r[n].wrapping_add(4);
            }
            0x17 => {
                self.gbr = bus.read32(self.r[n]);
                self.r[n] = self.r[n].wrapping_add(4);
            }
            0x18 => self.r[n] <<= 8,
            0x19 => self.r[n] >>= 8,
            0x1a => self.macl = self.r[n],
            0x1b => {
                let value = bus.read8(self.r[n]);
                self.set_t(value == 0);
                bus.write8(self.r[n], value | 0x80);
                *used = 4;
            }
            0x1e => self.gbr = self.r[n],
            0x21 => {
                let carry = self.r[n] & 1 != 0;
                self.r[n] = ((self.r[n] as i32) >> 1) as u32;
                self.set_t(carry);
            }
            0x22 => {
                self.r[n] = self.r[n].wrapping_sub(4);
                bus.write32(self.r[n], self.pr);
            }
            0x23 => {
                self.r[n] = self.r[n].wrapping_sub(4);
                bus.write32(self.r[n], self.vbr);
            }
            0x24 => {
                let old_t = u32::from(self.t());
                let carry = self.r[n] & 0x8000_0000 != 0;
                self.r[n] = (self.r[n] << 1) | old_t;
                self.set_t(carry);
            }
            0x25 => {
                let old_t = u32::from(self.t());
                let carry = self.r[n] & 1 != 0;
                self.r[n] = (self.r[n] >> 1) | (old_t << 31);
                self.set_t(carry);
            }
            0x26 => {
                self.pr = bus.read32(self.r[n]);
                self.r[n] = self.r[n].wrapping_add(4);
            }
            0x27 => {
                self.vbr = bus.read32(self.r[n]);
                self.r[n] = self.r[n].wrapping_add(4);
            }
            0x28 => self.r[n] <<= 16,
            0x29 => self.r[n] >>= 16,
            0x2a => self.pr = self.r[n],
            0x2b => {
                *used = 2;
                return self.schedule_branch(self.r[n], in_delay);
            }
            0x2e => self.vbr = self.r[n],
            _ => return false,
        }
        true
    }

    fn group6<B: Sh4Bus>(&mut self, bus: &mut B, opcode: u16, n: usize, m: usize) -> bool {
        match opcode & 0x000f {
            0x0 => self.r[n] = i32::from(bus.read8(self.r[m]) as i8) as u32,
            0x1 => self.r[n] = i32::from(bus.read16(self.r[m]) as i16) as u32,
            0x2 => self.r[n] = bus.read32(self.r[m]),
            0x3 => self.r[n] = self.r[m],
            0x4 => {
                let value = i32::from(bus.read8(self.r[m]) as i8) as u32;
                if n != m {
                    self.r[m] = self.r[m].wrapping_add(1);
                }
                self.r[n] = value;
            }
            0x5 => {
                let value = i32::from(bus.read16(self.r[m]) as i16) as u32;
                if n != m {
                    self.r[m] = self.r[m].wrapping_add(2);
                }
                self.r[n] = value;
            }
            0x6 => {
                let value = bus.read32(self.r[m]);
                if n != m {
                    self.r[m] = self.r[m].wrapping_add(4);
                }
                self.r[n] = value;
            }
            0x7 => self.r[n] = !self.r[m],
            0x8 => {
                self.r[n] = (self.r[m] & 0xffff_0000)
                    | ((self.r[m] & 0xff) << 8)
                    | ((self.r[m] >> 8) & 0xff)
            }
            0x9 => self.r[n] = self.r[m].rotate_left(16),
            0xa => {
                let carry = u32::from(self.t());
                let (v1, b1) = 0u32.overflowing_sub(self.r[m]);
                let (v2, b2) = v1.overflowing_sub(carry);
                self.r[n] = v2;
                self.set_t(b1 || b2);
            }
            0xb => self.r[n] = 0u32.wrapping_sub(self.r[m]),
            0xc => self.r[n] = self.r[m] & 0xff,
            0xd => self.r[n] = self.r[m] & 0xffff,
            0xe => self.r[n] = i32::from(self.r[m] as u8 as i8) as u32,
            0xf => self.r[n] = i32::from(self.r[m] as u16 as i16) as u32,
            _ => return false,
        }
        true
    }

    fn group8<B: Sh4Bus>(
        &mut self,
        bus: &mut B,
        opcode: u16,
        _n: usize,
        in_delay: bool,
        used: &mut u32,
    ) -> bool {
        let reg = usize::from((opcode >> 4) & 0x0f);
        let disp4 = u32::from(opcode & 0x0f);
        match opcode >> 8 {
            0x80 => bus.write8(self.r[reg].wrapping_add(disp4), self.r[0] as u8),
            0x81 => bus.write16(self.r[reg].wrapping_add(disp4 * 2), self.r[0] as u16),
            0x84 => self.r[0] = i32::from(bus.read8(self.r[reg].wrapping_add(disp4)) as i8) as u32,
            0x85 => {
                self.r[0] = i32::from(bus.read16(self.r[reg].wrapping_add(disp4 * 2)) as i16) as u32
            }
            0x88 => self.set_t(self.r[0] == Self::sign8(opcode as u8)),
            0x89 => {
                if self.t() {
                    self.pc = Self::branch_target8(self.pc, opcode as u8);
                    *used = 3;
                }
            }
            0x8b => {
                if !self.t() {
                    self.pc = Self::branch_target8(self.pc, opcode as u8);
                    *used = 3;
                }
            }
            0x8d => {
                if self.t() {
                    *used = 2;
                    return self
                        .schedule_branch(Self::branch_target8(self.pc, opcode as u8), in_delay);
                }
            }
            0x8f => {
                if !self.t() {
                    *used = 2;
                    return self
                        .schedule_branch(Self::branch_target8(self.pc, opcode as u8), in_delay);
                }
            }
            _ => return false,
        }
        true
    }

    fn group_c<B: Sh4Bus>(&mut self, bus: &mut B, opcode: u16, used: &mut u32) -> bool {
        let imm = opcode as u8;
        let disp = u32::from(imm);
        match opcode >> 8 {
            0xc0 => bus.write8(self.gbr.wrapping_add(disp), self.r[0] as u8),
            0xc1 => bus.write16(self.gbr.wrapping_add(disp * 2), self.r[0] as u16),
            0xc2 => bus.write32(self.gbr.wrapping_add(disp * 4), self.r[0]),
            0xc3 => {
                self.tra = disp << 2;
                self.ssr = self.sr;
                self.spc = self.pc;
                self.sgr = self.r[15];
                self.expevt = 0x160;
                self.set_sr(self.sr | MD | RB | BL);
                self.pc = self.vbr.wrapping_add(0x100);
                self.pending_branch = None;
                *used = 8;
            }
            0xc4 => self.r[0] = i32::from(bus.read8(self.gbr.wrapping_add(disp)) as i8) as u32,
            0xc5 => {
                self.r[0] = i32::from(bus.read16(self.gbr.wrapping_add(disp * 2)) as i16) as u32
            }
            0xc6 => self.r[0] = bus.read32(self.gbr.wrapping_add(disp * 4)),
            0xc7 => self.r[0] = (self.pc.wrapping_add(2) & !3).wrapping_add(disp * 4),
            0xc8 => self.set_t(self.r[0] & u32::from(imm) == 0),
            0xc9 => self.r[0] &= u32::from(imm),
            0xca => self.r[0] ^= u32::from(imm),
            0xcb => self.r[0] |= u32::from(imm),
            0xcc..=0xcf => {
                let address = self.gbr.wrapping_add(self.r[0]);
                let value = bus.read8(address);
                match opcode >> 8 {
                    0xcc => self.set_t(value & imm == 0),
                    0xcd => bus.write8(address, value & imm),
                    0xce => bus.write8(address, value ^ imm),
                    _ => bus.write8(address, value | imm),
                }
            }
            _ => return false,
        }
        true
    }

    fn div1(&mut self, n: usize, m: usize) {
        let old_q = self.sr & Q != 0;
        let q = self.r[n] & 0x8000_0000 != 0;
        self.set_bit(Q, q);
        self.r[n] = (self.r[n] << 1) | u32::from(self.t());
        let m_bit = self.sr & M != 0;
        let before = self.r[n];
        let q_after = match (old_q, m_bit) {
            (false, false) => {
                self.r[n] = self.r[n].wrapping_sub(self.r[m]);
                let carry = self.r[n] > before;
                if !q {
                    carry
                } else {
                    !carry
                }
            }
            (false, true) => {
                self.r[n] = self.r[n].wrapping_add(self.r[m]);
                let carry = self.r[n] < before;
                if !q {
                    !carry
                } else {
                    carry
                }
            }
            (true, false) => {
                self.r[n] = self.r[n].wrapping_add(self.r[m]);
                let carry = self.r[n] < before;
                if !q {
                    carry
                } else {
                    !carry
                }
            }
            (true, true) => {
                self.r[n] = self.r[n].wrapping_sub(self.r[m]);
                let carry = self.r[n] > before;
                if !q {
                    !carry
                } else {
                    carry
                }
            }
        };
        self.set_bit(Q, q_after);
        self.set_t(q_after == m_bit);
    }

    pub fn save(&self, out: &mut StateWriter) {
        for value in self.r {
            out.u32(value);
        }
        for value in self.r_bank {
            out.u32(value);
        }
        for value in [
            self.pc, self.pr, self.gbr, self.vbr, self.dbr, self.mach, self.macl, self.sr,
            self.ssr, self.spc, self.sgr,
        ] {
            out.u32(value);
        }
        for value in self.fr {
            out.u32(value);
        }
        for value in self.xf {
            out.u32(value);
        }
        for value in [self.fpul, self.fpscr, self.expevt, self.intevt, self.tra] {
            out.u32(value);
        }
        out.u64(self.cycles);
        out.u8(u8::from(self.sleeping));
        out.u8(u8::from(self.pending_branch.is_some()));
        out.u32(self.pending_branch.unwrap_or(0));
    }

    pub fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        for value in &mut self.r {
            *value = input.u32()?;
        }
        for value in &mut self.r_bank {
            *value = input.u32()?;
        }
        self.pc = input.u32()?;
        self.pr = input.u32()?;
        self.gbr = input.u32()?;
        self.vbr = input.u32()?;
        self.dbr = input.u32()?;
        self.mach = input.u32()?;
        self.macl = input.u32()?;
        self.sr = input.u32()? & SR_MASK;
        self.ssr = input.u32()?;
        self.spc = input.u32()?;
        self.sgr = input.u32()?;
        for value in &mut self.fr {
            *value = input.u32()?;
        }
        for value in &mut self.xf {
            *value = input.u32()?;
        }
        self.fpul = input.u32()?;
        self.fpscr = input.u32()? & FPSCR_MASK;
        self.expevt = input.u32()?;
        self.intevt = input.u32()?;
        self.tra = input.u32()?;
        self.cycles = input.u64()?;
        self.sleeping = input.u8()? != 0;
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

    struct TestBus {
        data: Vec<u8>,
    }

    impl TestBus {
        fn new() -> Self {
            Self {
                data: vec![0; 0x2000],
            }
        }
        fn word(&mut self, address: usize, value: u16) {
            self.data[address..address + 2].copy_from_slice(&value.to_le_bytes());
        }
        fn long(&mut self, address: usize, value: u32) {
            self.data[address..address + 4].copy_from_slice(&value.to_le_bytes());
        }
    }

    impl Sh4Bus for TestBus {
        fn read8(&mut self, address: u32) -> u8 {
            self.data.get(address as usize).copied().unwrap_or(0xff)
        }
        fn write8(&mut self, address: u32, value: u8) {
            if let Some(slot) = self.data.get_mut(address as usize) {
                *slot = value;
            }
        }
    }

    #[test]
    fn arithmetic_memory_and_little_endian_bus_execute() {
        let mut bus = TestBus::new();
        bus.long(0x120, 0x1122_3344);
        assert_eq!(bus.read32(0x120), 0x1122_3344);
        bus.word(0, 0xe105);
        bus.word(2, 0xe207);
        bus.word(4, 0x321c);
        bus.word(6, 0x2322);
        let mut cpu = Sh4::default();
        cpu.r[3] = 0x100;
        for _ in 0..4 {
            assert_ne!(cpu.step(&mut bus), 0);
        }
        assert_eq!(cpu.r[2], 12);
        assert_eq!(&bus.data[0x100..0x104], &[12, 0, 0, 0]);
    }

    #[test]
    fn delayed_branch_executes_slot_once() {
        let mut bus = TestBus::new();
        bus.word(0, 0xe101);
        bus.word(2, 0xa001);
        bus.word(4, 0xe202);
        bus.word(6, 0xe303);
        bus.word(8, 0xe404);
        let mut cpu = Sh4::default();
        assert_ne!(cpu.step(&mut bus), 0);
        assert_ne!(cpu.step(&mut bus), 0);
        assert_eq!(cpu.pc, 4);
        assert_ne!(cpu.step(&mut bus), 0);
        assert_eq!(cpu.pc, 8);
        assert_eq!(cpu.r[2], 2);
        assert_eq!(cpu.r[3], 0);
        assert_ne!(cpu.step(&mut bus), 0);
        assert_eq!(cpu.r[4], 4);
    }

    #[test]
    fn trap_and_rte_use_sh4_saved_status_registers() {
        let mut bus = TestBus::new();
        bus.word(0, 0xc320);
        bus.word(0x200, 0x002b);
        bus.word(0x202, 0x0009);
        let mut cpu = Sh4 {
            vbr: 0x100,
            ..Default::default()
        };
        cpu.set_sr(0x51);
        cpu.r[15] = 0x800;
        assert_ne!(cpu.step(&mut bus), 0);
        assert_eq!(cpu.pc, 0x200);
        assert_eq!(cpu.spc, 2);
        assert_eq!(cpu.ssr, 0x51);
        assert_eq!(cpu.sgr, 0x800);
        assert_eq!(cpu.tra, 0x80);
        assert_eq!(cpu.expevt, 0x160);
        assert_ne!(cpu.step(&mut bus), 0);
        assert_ne!(cpu.step(&mut bus), 0);
        assert_eq!(cpu.pc, 2);
        assert_eq!(cpu.sr, 0x51);
        assert_eq!(cpu.r[15], 0x800);
    }

    #[test]
    fn cpu_state_round_trip_is_deterministic() {
        let mut cpu = Sh4::default();
        cpu.r[2] = 0x1234_5678;
        cpu.r_bank[2] = 0x89ab_cdef;
        cpu.pc = 0x8c00_0100;
        cpu.vbr = 0x8c00_0000;
        cpu.pr = 0x8c00_0200;
        cpu.set_sr(MD | RB | 0x21);
        cpu.ssr = 0x51;
        cpu.spc = 0x8c00_0400;
        cpu.fr[3] = 1.25f32.to_bits();
        cpu.xf[7] = (-2.5f32).to_bits();
        cpu.fpul = 17;
        cpu.fpscr = FPSCR_FR | FPSCR_SZ;
        cpu.cycles = 99;
        cpu.pending_branch = Some(0x8c00_0300);
        let mut out = StateWriter::new(PlatformId::Dreamcast, 1);
        cpu.save(&mut out);
        let bytes = out.finish();
        let mut input = StateReader::new(&bytes, PlatformId::Dreamcast, 1).unwrap();
        let mut restored = Sh4::default();
        restored.load(&mut input).unwrap();
        input.finish().unwrap();
        assert_eq!(restored.r[2], cpu.r[2]);
        assert_eq!(restored.pc, cpu.pc);
        assert_eq!(restored.vbr, cpu.vbr);
        assert_eq!(restored.pr, cpu.pr);
        assert_eq!(restored.sr, cpu.sr);
        assert_eq!(restored.r_bank, cpu.r_bank);
        assert_eq!(restored.ssr, cpu.ssr);
        assert_eq!(restored.spc, cpu.spc);
        assert_eq!(restored.fr, cpu.fr);
        assert_eq!(restored.xf, cpu.xf);
        assert_eq!(restored.fpul, cpu.fpul);
        assert_eq!(restored.fpscr, cpu.fpscr);
        assert_eq!(restored.cycles, cpu.cycles);
        assert_eq!(restored.pending_branch, cpu.pending_branch);
    }
}

impl Sh4 {
    fn dr64(&self, index: usize) -> f64 {
        let even = index & 0x0e;
        let bits = (u64::from(self.fr[even]) << 32) | u64::from(self.fr[even + 1]);
        f64::from_bits(bits)
    }

    fn set_dr64(&mut self, index: usize, value: f64) {
        let even = index & 0x0e;
        let bits = value.to_bits();
        self.fr[even] = (bits >> 32) as u32;
        self.fr[even + 1] = bits as u32;
    }

    fn fmov_load<B: Sh4Bus>(&mut self, bus: &mut B, n: usize, address: u32) {
        if self.fpscr & FPSCR_SZ == 0 {
            self.fr[n] = bus.read32(address);
        } else {
            let even = n & 0x0e;
            self.fr[even] = bus.read32(address);
            self.fr[even + 1] = bus.read32(address.wrapping_add(4));
        }
    }
    fn fmov_store<B: Sh4Bus>(&self, bus: &mut B, m: usize, address: u32) {
        if self.fpscr & FPSCR_SZ == 0 {
            bus.write32(address, self.fr[m]);
        } else {
            let even = m & 0x0e;
            bus.write32(address, self.fr[even]);
            bus.write32(address.wrapping_add(4), self.fr[even + 1]);
        }
    }

    fn fpu_binary(&mut self, opcode: u16, n: usize, m: usize) -> bool {
        if self.fpscr & FPSCR_PR == 0 {
            let left = self.fr32(n);
            let right = self.fr32(m);
            let value = match opcode & 0x0f {
                0x0 => left + right,
                0x1 => left - right,
                0x2 => left * right,
                0x3 => left / right,
                _ => return false,
            };
            self.set_fr32(n, value);
            true
        } else {
            if n & 1 != 0 || m & 1 != 0 {
                return false;
            }
            let left = self.dr64(n);
            let right = self.dr64(m);
            let value = match opcode & 0x0f {
                0x0 => left + right,
                0x1 => left - right,
                0x2 => left * right,
                0x3 => left / right,
                _ => return false,
            };
            self.set_dr64(n, value);
            true
        }
    }

    fn group_f<B: Sh4Bus>(&mut self, bus: &mut B, opcode: u16, n: usize, m: usize) -> bool {
        if self.sr & FD != 0 {
            return false;
        }
        match opcode & 0x0f {
            0x0..=0x3 => self.fpu_binary(opcode, n, m),
            0x4 | 0x5 => {
                if self.fpscr & FPSCR_PR == 0 {
                    let left = self.fr32(n);
                    let right = self.fr32(m);
                    self.set_t(if opcode & 0x0f == 4 {
                        left == right
                    } else {
                        left > right
                    });
                } else {
                    if n & 1 != 0 || m & 1 != 0 {
                        return false;
                    }
                    let left = self.dr64(n);
                    let right = self.dr64(m);
                    self.set_t(if opcode & 0x0f == 4 {
                        left == right
                    } else {
                        left > right
                    });
                }
                true
            }
            0x6 => {
                let address = self.r[0].wrapping_add(self.r[m]);
                self.fmov_load(bus, n, address);
                true
            }
            0x7 => {
                let address = self.r[0].wrapping_add(self.r[n]);
                self.fmov_store(bus, m, address);
                true
            }
            0x8 => {
                self.fmov_load(bus, n, self.r[m]);
                true
            }
            0x9 => {
                let width = if self.fpscr & FPSCR_SZ == 0 { 4 } else { 8 };
                let address = self.r[m];
                self.fmov_load(bus, n, address);
                self.r[m] = self.r[m].wrapping_add(width);
                true
            }
            0xa => {
                self.fmov_store(bus, m, self.r[n]);
                true
            }
            0xb => {
                let width = if self.fpscr & FPSCR_SZ == 0 { 4 } else { 8 };
                self.r[n] = self.r[n].wrapping_sub(width);
                self.fmov_store(bus, m, self.r[n]);
                true
            }
            0xd => self.fpu_special(opcode, n),
            0xe if self.fpscr & FPSCR_PR == 0 => {
                let value = self.fr32(0) * self.fr32(m) + self.fr32(n);
                self.set_fr32(n, value);
                true
            }
            _ => false,
        }
    }

    fn fpu_special(&mut self, opcode: u16, n: usize) -> bool {
        match opcode & 0x00ff {
            0x0d => self.fr[n] = self.fpul,
            0x1d => self.fpul = self.fr[n],
            0x2d => {
                if self.fpscr & FPSCR_PR == 0 {
                    self.set_fr32(n, self.fpul as i32 as f32);
                } else if n & 1 == 0 {
                    self.set_dr64(n, self.fpul as i32 as f64);
                } else {
                    return false;
                }
            }
            0x3d => {
                let value = if self.fpscr & FPSCR_PR == 0 {
                    f64::from(self.fr32(n))
                } else if n & 1 == 0 {
                    self.dr64(n)
                } else {
                    return false;
                };
                self.fpul = if value.is_nan() {
                    0x8000_0000
                } else if value >= i32::MAX as f64 {
                    i32::MAX as u32
                } else if value <= i32::MIN as f64 {
                    i32::MIN as u32
                } else {
                    value.trunc() as i32 as u32
                };
            }
            0x4d => self.fr[n] ^= 0x8000_0000,
            0x5d => self.fr[n] &= 0x7fff_ffff,
            0x6d => {
                if self.fpscr & FPSCR_PR == 0 {
                    self.set_fr32(n, self.fr32(n).sqrt());
                } else if n & 1 == 0 {
                    self.set_dr64(n, self.dr64(n).sqrt());
                } else {
                    return false;
                }
            }
            0x7d if self.fpscr & FPSCR_PR == 0 => {
                self.set_fr32(n, 1.0 / self.fr32(n).sqrt());
            }
            0x8d if self.fpscr & FPSCR_PR == 0 => self.fr[n] = 0,
            0x9d if self.fpscr & FPSCR_PR == 0 => self.fr[n] = 1.0f32.to_bits(),
            0xad if self.fpscr & FPSCR_PR != 0 && n & 1 == 0 => {
                self.set_dr64(n, f64::from(f32::from_bits(self.fpul)));
            }
            0xbd if self.fpscr & FPSCR_PR != 0 && n & 1 == 0 => {
                self.fpul = (self.dr64(n) as f32).to_bits();
            }
            0xed if self.fpscr & FPSCR_PR == 0 => {
                let vn = ((opcode >> 10) as usize & 3) * 4;
                let vm = ((opcode >> 8) as usize & 3) * 4;
                let mut dot = 0.0f32;
                for lane in 0..4 {
                    dot += self.fr32(vn + lane) * self.fr32(vm + lane);
                }
                self.set_fr32(vn + 3, dot);
            }
            0xfd if opcode == 0xf3fd && self.fpscr & FPSCR_PR == 0 => {
                self.set_fpscr(self.fpscr ^ FPSCR_SZ);
            }
            0xfd if opcode == 0xfbfd && self.fpscr & FPSCR_PR == 0 => {
                self.set_fpscr(self.fpscr ^ FPSCR_FR);
            }
            0xfd if opcode & 0x03ff == 0x01fd && self.fpscr & FPSCR_PR == 0 => {
                let base = ((opcode >> 10) as usize & 3) * 4;
                let input = [
                    self.fr32(base),
                    self.fr32(base + 1),
                    self.fr32(base + 2),
                    self.fr32(base + 3),
                ];
                let mut output = [0.0f32; 4];
                for (row, result) in output.iter_mut().enumerate() {
                    for (column, input_value) in input.iter().copied().enumerate() {
                        *result += f32::from_bits(self.xf[row * 4 + column]) * input_value;
                    }
                }
                for (lane, value) in output.into_iter().enumerate() {
                    self.set_fr32(base + lane, value);
                }
            }
            0xfd if opcode & 0x0100 == 0 && self.fpscr & FPSCR_PR == 0 => {
                let base = n & 0x0e;
                let angle = f64::from(self.fpul as u16) * std::f64::consts::TAU / 65536.0;
                self.set_fr32(base, angle.sin() as f32);
                self.set_fr32(base + 1, angle.cos() as f32);
            }
            _ => return false,
        }
        true
    }
}
#[cfg(test)]
mod sh4_extension_tests {
    use super::*;

    struct Bus {
        bytes: [u8; 256],
    }

    impl Default for Bus {
        fn default() -> Self {
            Self { bytes: [0; 256] }
        }
    }

    impl Sh4Bus for Bus {
        fn read8(&mut self, address: u32) -> u8 {
            self.bytes[address as usize & 0xff]
        }

        fn write8(&mut self, address: u32, value: u8) {
            self.bytes[address as usize & 0xff] = value;
        }
    }
    #[test]
    fn privileged_register_bank_switches_are_physical() {
        let mut cpu = Sh4::default();
        cpu.r[0] = 0x1111_1111;
        cpu.r_bank[0] = 0x2222_2222;
        cpu.set_sr(0);
        assert_eq!(cpu.r[0], 0x2222_2222);
        assert_eq!(cpu.r_bank[0], 0x1111_1111);
        cpu.set_sr(MD | RB);
        assert_eq!(cpu.r[0], 0x1111_1111);
        assert_eq!(cpu.r_bank[0], 0x2222_2222);
    }

    #[test]
    fn single_precision_fpu_and_fpul_conversions_execute() {
        let mut cpu = Sh4::default();
        let mut bus = Bus::default();
        cpu.set_sr(cpu.sr & !FD);
        cpu.set_fr32(1, 1.5);
        cpu.set_fr32(2, 2.5);
        assert!(cpu.group_f(&mut bus, 0xf210, 2, 1));
        assert_eq!(cpu.fr32(2), 4.0);
        assert!(cpu.group_f(&mut bus, 0xf212, 2, 1));
        assert_eq!(cpu.fr32(2), 6.0);
        assert!(cpu.group_f(&mut bus, 0xf23d, 2, 3));
        assert_eq!(cpu.fpul, 6);
        cpu.fpul = (-7i32) as u32;
        assert!(cpu.group_f(&mut bus, 0xf32d, 3, 2));
        assert_eq!(cpu.fr32(3), -7.0);
    }

    #[test]
    fn fpu_bank_width_and_sincos_controls_execute() {
        let mut cpu = Sh4::default();
        let mut bus = Bus::default();
        cpu.set_sr(cpu.sr & !FD);
        cpu.fr[0] = 0x1111_1111;
        cpu.xf[0] = 0x2222_2222;
        assert!(cpu.group_f(&mut bus, 0xfbfd, 11, 15));
        assert_eq!(cpu.fr[0], 0x2222_2222);
        assert_eq!(cpu.xf[0], 0x1111_1111);
        assert!(cpu.group_f(&mut bus, 0xf3fd, 3, 15));
        assert_ne!(cpu.fpscr & FPSCR_SZ, 0);
        cpu.set_fpscr(cpu.fpscr & !(FPSCR_SZ | FPSCR_FR));
        cpu.fpul = 0x4000;
        assert!(cpu.group_f(&mut bus, 0xf0fd, 0, 15));
        assert!((cpu.fr32(0) - 1.0).abs() < 0.00001);
        assert!(cpu.fr32(1).abs() < 0.00001);
    }

    #[test]
    fn interrupt_uses_saved_registers_and_fixed_vector() {
        let mut cpu = Sh4::default();
        let mut bus = Bus::default();
        cpu.set_sr(0x20);
        cpu.pc = 0x8c01_0000;
        cpu.vbr = 0x8c00_0000;
        cpu.r[15] = 0x8d00_0000;
        assert_eq!(cpu.interrupt(&mut bus, 6, 0x320), 5);
        assert_eq!(cpu.spc, 0x8c01_0000);
        assert_eq!(cpu.ssr, 0x20);
        assert_eq!(cpu.sgr, 0x8d00_0000);
        assert_eq!(cpu.intevt, 0x320);
        assert_eq!(cpu.pc, 0x8c00_0600);
        assert_ne!(cpu.sr & (MD | RB | BL), 0);
    }
}
