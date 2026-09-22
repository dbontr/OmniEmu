use crate::state::{StateReader, StateWriter};

pub trait Bus68000: Send {
    fn read8(&mut self, address: u32) -> u8;
    fn write8(&mut self, address: u32, value: u8);
    fn tas_write8(&mut self, address: u32, value: u8) {
        self.write8(address, value);
    }

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
}

const C: u16 = 0x0001;
const V: u16 = 0x0002;
const Z: u16 = 0x0004;
const N: u16 = 0x0008;
const X: u16 = 0x0010;
const S: u16 = 0x2000;
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Size {
    Byte,
    Word,
    Long,
}

impl Size {
    fn mask(self) -> u32 {
        match self {
            Self::Byte => 0xff,
            Self::Word => 0xffff,
            Self::Long => 0xffff_ffff,
        }
    }
    fn sign(self) -> u32 {
        match self {
            Self::Byte => 0x80,
            Self::Word => 0x8000,
            Self::Long => 0x8000_0000,
        }
    }
    fn bytes(self, register: usize) -> u32 {
        match self {
            Self::Byte if register == 7 => 2,
            Self::Byte => 1,
            Self::Word => 2,
            Self::Long => 4,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Ea {
    Data(usize),
    Address(usize),
    Memory(u32),
    Immediate(u32),
}

#[derive(Debug, Clone)]
pub struct M68000 {
    pub d: [u32; 8],
    pub a: [u32; 8],
    pub pc: u32,
    pub sr: u16,
    pub cycles: u64,
    pub stopped: bool,
    usp: u32,
    ssp: u32,
}
impl Default for M68000 {
    fn default() -> Self {
        Self {
            d: [0; 8],
            a: [0; 8],
            pc: 0,
            sr: 0x2700,
            cycles: 0,
            stopped: false,
            usp: 0,
            ssp: 0,
        }
    }
}

impl M68000 {
    pub fn reset<B: Bus68000>(&mut self, bus: &mut B) {
        *self = Self::default();
        self.a[7] = bus.read32(0) & 0x00ff_ffff;
        self.ssp = self.a[7];
        self.pc = bus.read32(4) & 0x00ff_ffff;
        self.cycles = 40;
    }

    fn mask_address(address: u32) -> u32 {
        address & 0x00ff_ffff
    }

    fn fetch16<B: Bus68000>(&mut self, bus: &mut B) -> u16 {
        let value = bus.read16(Self::mask_address(self.pc));
        self.pc = Self::mask_address(self.pc.wrapping_add(2));
        value
    }

    fn fetch32<B: Bus68000>(&mut self, bus: &mut B) -> u32 {
        let high = u32::from(self.fetch16(bus));
        let low = u32::from(self.fetch16(bus));
        (high << 16) | low
    }

    fn set_flag(&mut self, flag: u16, enabled: bool) {
        if enabled {
            self.sr |= flag;
        } else {
            self.sr &= !flag;
        }
    }

    fn flag(&self, flag: u16) -> bool {
        self.sr & flag != 0
    }

    fn set_sr(&mut self, value: u16) {
        let was_supervisor = self.sr & S != 0;
        let will_be_supervisor = value & S != 0;
        if was_supervisor != will_be_supervisor {
            if was_supervisor {
                self.ssp = Self::mask_address(self.a[7]);
                self.a[7] = Self::mask_address(self.usp);
            } else {
                self.usp = Self::mask_address(self.a[7]);
                self.a[7] = Self::mask_address(self.ssp);
            }
        }
        self.sr = value;
    }

    fn set_nz(&mut self, value: u32, size: Size) {
        let value = value & size.mask();
        self.set_flag(N, value & size.sign() != 0);
        self.set_flag(Z, value == 0);
        self.set_flag(V, false);
        self.set_flag(C, false);
    }

    fn sign_extend(value: u32, size: Size) -> u32 {
        match size {
            Size::Byte => (value as u8 as i8 as i32) as u32,
            Size::Word => (value as u16 as i16 as i32) as u32,
            Size::Long => value,
        }
    }

    fn size_from_field(field: u16) -> Option<Size> {
        match field & 3 {
            0 => Some(Size::Byte),
            1 => Some(Size::Word),
            2 => Some(Size::Long),
            _ => None,
        }
    }

    fn write_data_register(&mut self, reg: usize, value: u32, size: Size) {
        self.d[reg] = match size {
            Size::Byte => (self.d[reg] & !0xff) | (value & 0xff),
            Size::Word => (self.d[reg] & !0xffff) | (value & 0xffff),
            Size::Long => value,
        };
    }

    fn index_value(&self, extension: u16) -> u32 {
        let reg = usize::from((extension >> 12) & 7);
        let value = if extension & 0x8000 != 0 {
            self.a[reg]
        } else {
            self.d[reg]
        };
        if extension & 0x0800 != 0 {
            value
        } else {
            (value as u16 as i16 as i32) as u32
        }
    }

    fn resolve_ea<B: Bus68000>(
        &mut self,
        bus: &mut B,
        mode: u16,
        reg: usize,
        size: Size,
        immediate: bool,
    ) -> Option<Ea> {
        Some(match mode {
            0 => Ea::Data(reg),
            1 => Ea::Address(reg),
            2 => Ea::Memory(Self::mask_address(self.a[reg])),
            3 => {
                let address = Self::mask_address(self.a[reg]);
                self.a[reg] = Self::mask_address(self.a[reg].wrapping_add(size.bytes(reg)));
                Ea::Memory(address)
            }
            4 => {
                self.a[reg] = Self::mask_address(self.a[reg].wrapping_sub(size.bytes(reg)));
                Ea::Memory(self.a[reg])
            }
            5 => {
                let displacement = self.fetch16(bus) as i16 as i32 as u32;
                Ea::Memory(Self::mask_address(self.a[reg].wrapping_add(displacement)))
            }
            6 => {
                let extension = self.fetch16(bus);
                let displacement = extension as u8 as i8 as i32 as u32;
                let index = self.index_value(extension);
                Ea::Memory(Self::mask_address(
                    self.a[reg].wrapping_add(displacement).wrapping_add(index),
                ))
            }
            7 => match reg {
                0 => Ea::Memory(Self::mask_address(self.fetch16(bus) as i16 as i32 as u32)),
                1 => Ea::Memory(Self::mask_address(self.fetch32(bus))),
                2 => {
                    let base = self.pc;
                    let disp = self.fetch16(bus) as i16 as i32 as u32;
                    Ea::Memory(Self::mask_address(base.wrapping_add(disp)))
                }
                3 => {
                    let base = self.pc;
                    let ext = self.fetch16(bus);
                    let disp = ext as u8 as i8 as i32 as u32;
                    Ea::Memory(Self::mask_address(
                        base.wrapping_add(disp).wrapping_add(self.index_value(ext)),
                    ))
                }
                4 if immediate => Ea::Immediate(match size {
                    Size::Byte => u32::from(self.fetch16(bus) as u8),
                    Size::Word => u32::from(self.fetch16(bus)),
                    Size::Long => self.fetch32(bus),
                }),
                _ => return None,
            },
            _ => return None,
        })
    }
    fn read_ea<B: Bus68000>(&self, bus: &mut B, ea: Ea, size: Size) -> u32 {
        match ea {
            Ea::Data(reg) => self.d[reg] & size.mask(),
            Ea::Address(reg) => self.a[reg] & size.mask(),
            Ea::Immediate(value) => value & size.mask(),
            Ea::Memory(address) => match size {
                Size::Byte => u32::from(bus.read8(address)),
                Size::Word => u32::from(bus.read16(address)),
                Size::Long => bus.read32(address),
            },
        }
    }

    fn write_ea<B: Bus68000>(&mut self, bus: &mut B, ea: Ea, size: Size, value: u32) -> bool {
        match ea {
            Ea::Data(reg) => self.write_data_register(reg, value, size),
            Ea::Address(reg) => {
                self.a[reg] = match size {
                    Size::Word => Self::sign_extend(value, Size::Word),
                    _ => value,
                } & 0x00ff_ffff;
            }
            Ea::Memory(address) => match size {
                Size::Byte => bus.write8(address, value as u8),
                Size::Word => bus.write16(address, value as u16),
                Size::Long => bus.write32(address, value),
            },
            Ea::Immediate(_) => return false,
        }
        true
    }

    fn movem_register(&self, index: usize) -> u32 {
        if index < 8 {
            self.d[index]
        } else {
            self.a[index - 8]
        }
    }

    fn set_movem_register(&mut self, index: usize, value: u32) {
        if index < 8 {
            self.d[index] = value;
        } else {
            self.a[index - 8] = Self::mask_address(value);
        }
    }

    fn push16<B: Bus68000>(&mut self, bus: &mut B, value: u16) {
        self.a[7] = Self::mask_address(self.a[7].wrapping_sub(2));
        bus.write16(self.a[7], value);
    }

    fn push32<B: Bus68000>(&mut self, bus: &mut B, value: u32) {
        self.a[7] = Self::mask_address(self.a[7].wrapping_sub(4));
        bus.write32(self.a[7], value);
    }
    fn pop16<B: Bus68000>(&mut self, bus: &mut B) -> u16 {
        let value = bus.read16(self.a[7]);
        self.a[7] = Self::mask_address(self.a[7].wrapping_add(2));
        value
    }

    fn pop32<B: Bus68000>(&mut self, bus: &mut B) -> u32 {
        let value = bus.read32(self.a[7]);
        self.a[7] = Self::mask_address(self.a[7].wrapping_add(4));
        value
    }

    fn condition(&self, code: u16) -> bool {
        let n = self.flag(N);
        let z = self.flag(Z);
        let v = self.flag(V);
        let c = self.flag(C);
        match code & 0x0f {
            0 => true,
            1 => false,
            2 => !c && !z,
            3 => c || z,
            4 => !c,
            5 => c,
            6 => !z,
            7 => z,
            8 => !v,
            9 => v,
            10 => !n,
            11 => n,
            12 => n == v,
            13 => n != v,
            14 => !z && n == v,
            _ => z || n != v,
        }
    }
    fn add_value(&mut self, lhs: u32, rhs: u32, size: Size) -> u32 {
        let mask = size.mask();
        let sign = size.sign();
        let a = lhs & mask;
        let b = rhs & mask;
        let wide = u64::from(a) + u64::from(b);
        let result = wide as u32 & mask;
        let carry = wide > u64::from(mask);
        let overflow = (!(a ^ b) & (a ^ result) & sign) != 0;
        self.set_flag(N, result & sign != 0);
        self.set_flag(Z, result == 0);
        self.set_flag(V, overflow);
        self.set_flag(C, carry);
        self.set_flag(X, carry);
        result
    }

    fn sub_value(&mut self, lhs: u32, rhs: u32, size: Size) -> u32 {
        let mask = size.mask();
        let sign = size.sign();
        let a = lhs & mask;
        let b = rhs & mask;
        let result = a.wrapping_sub(b) & mask;
        let borrow = b > a;
        let overflow = ((a ^ b) & (a ^ result) & sign) != 0;
        self.set_flag(N, result & sign != 0);
        self.set_flag(Z, result == 0);
        self.set_flag(V, overflow);
        self.set_flag(C, borrow);
        self.set_flag(X, borrow);
        result
    }

    fn cmp_value(&mut self, lhs: u32, rhs: u32, size: Size) {
        let old_x = self.sr & X;
        self.sub_value(lhs, rhs, size);
        self.sr = (self.sr & !X) | old_x;
    }

    pub fn interrupt<B: Bus68000>(&mut self, bus: &mut B, level: u8, vector: u8) -> u32 {
        let level = level.min(7);
        let current = ((self.sr >> 8) & 7) as u8;
        if level <= current && level != 7 {
            return 0;
        }
        self.stopped = false;
        let old_sr = self.sr;
        self.set_sr(old_sr | S);
        self.push32(bus, self.pc);
        self.push16(bus, old_sr);
        self.sr = (self.sr & !0x0700) | (u16::from(level) << 8);
        self.pc = bus.read32(u32::from(vector) * 4) & 0x00ff_ffff;
        self.cycles += 44;
        44
    }

    pub fn save(&self, out: &mut StateWriter) {
        for value in self.d {
            out.u32(value);
        }
        for value in self.a {
            out.u32(value);
        }
        let (usp, ssp) = if self.sr & S != 0 {
            (self.usp, self.a[7])
        } else {
            (self.a[7], self.ssp)
        };
        out.u32(Self::mask_address(usp));
        out.u32(Self::mask_address(ssp));
        out.u32(self.pc);
        out.u16(self.sr);
        out.u64(self.cycles);
        out.u8(self.stopped as u8);
    }

    pub fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        for value in &mut self.d {
            *value = input.u32()?;
        }
        for value in &mut self.a {
            *value = input.u32()? & 0x00ff_ffff;
        }
        self.usp = input.u32()? & 0x00ff_ffff;
        self.ssp = input.u32()? & 0x00ff_ffff;
        self.pc = input.u32()? & 0x00ff_ffff;
        self.sr = input.u16()?;
        self.a[7] = if self.sr & S != 0 { self.ssp } else { self.usp };
        self.cycles = input.u64()?;
        self.stopped = input.u8()? != 0;
        Ok(())
    }

    pub fn step<B: Bus68000>(&mut self, bus: &mut B) -> u32 {
        if self.stopped {
            return 0;
        }
        let opcode = self.fetch16(bus);
        let used = self.execute(bus, opcode);
        self.cycles = self.cycles.wrapping_add(u64::from(used));
        used
    }

    fn execute<B: Bus68000>(&mut self, bus: &mut B, opcode: u16) -> u32 {
        if opcode & 0xf000 == 0x6000 {
            return self.branch(bus, opcode);
        }
        if opcode & 0xf100 == 0x7000 {
            let reg = usize::from((opcode >> 9) & 7);
            let value = opcode as u8 as i8 as i32 as u32;
            self.d[reg] = value;
            self.set_nz(value, Size::Long);
            return 4;
        }
        if matches!(opcode >> 12, 1..=3) {
            return self.move_instruction(bus, opcode);
        }
        self.execute_misc(bus, opcode)
    }
    fn branch<B: Bus68000>(&mut self, bus: &mut B, opcode: u16) -> u32 {
        let condition = (opcode >> 8) & 0x0f;
        let base = self.pc;
        let byte = opcode as u8;
        let displacement = if byte == 0 {
            self.fetch16(bus) as i16 as i32
        } else {
            byte as i8 as i32
        };
        let take = condition <= 1 || self.condition(condition);
        if !take {
            return 8;
        }
        if condition == 1 {
            self.push32(bus, self.pc);
        }
        self.pc = Self::mask_address(base.wrapping_add(displacement as u32));
        if condition == 1 {
            18
        } else {
            10
        }
    }

    fn check_bounds<B: Bus68000>(&mut self, bus: &mut B, opcode: u16) -> u32 {
        let Some(upper) = self.word_data_source(bus, opcode) else {
            return 0;
        };
        let register = usize::from((opcode >> 9) & 7);
        let value = self.d[register] as u16 as i16;
        let upper = upper as i16;
        if value < 0 || value > upper {
            self.set_flag(N, value < 0);
            let _ = self.exception(bus, 6);
            40
        } else {
            10
        }
    }

    fn negate_bcd<B: Bus68000>(&mut self, bus: &mut B, opcode: u16) -> u32 {
        let Some(ea) = self.data_alterable_ea(bus, opcode, Size::Byte) else {
            return 0;
        };
        let value = self.read_ea(bus, ea, Size::Byte) as u8;
        let extend = u16::from(self.flag(X));
        let low_borrow = u16::from(value & 0x0f) + extend != 0;
        let borrow = u16::from(value) + extend != 0;
        let correction = u8::from(low_borrow) * 0x06 + u8::from(borrow) * 0x60;
        let result = 0u8
            .wrapping_sub(value)
            .wrapping_sub(extend as u8)
            .wrapping_sub(correction);
        let old_zero = self.flag(Z);
        self.set_flag(Z, old_zero && result == 0);
        self.set_flag(C, borrow);
        self.set_flag(X, borrow);
        if !self.write_ea(bus, ea, Size::Byte, u32::from(result)) {
            return 0;
        }
        if matches!(ea, Ea::Data(_)) {
            6
        } else {
            8
        }
    }

    fn movem_instruction<B: Bus68000>(&mut self, bus: &mut B, opcode: u16) -> u32 {
        let load = opcode & 0x0400 != 0;
        let size = if opcode & 0x0040 != 0 {
            Size::Long
        } else {
            Size::Word
        };
        let bytes = size.bytes(0);
        let mode = (opcode >> 3) & 7;
        let ea_reg = usize::from(opcode & 7);
        let mask = self.fetch16(bus);
        let mut transferred = 0u32;

        if mode == 4 {
            if load {
                return 0;
            }
            let original = self.a[ea_reg];
            let mut address = original;
            for bit in 0..16usize {
                if mask & (1u16 << bit) == 0 {
                    continue;
                }
                address = Self::mask_address(address.wrapping_sub(bytes));
                let register = 15 - bit;
                let value = if register == 8 + ea_reg {
                    original
                } else {
                    self.movem_register(register)
                };
                match size {
                    Size::Word => bus.write16(address, value as u16),
                    Size::Long => bus.write32(address, value),
                    Size::Byte => unreachable!(),
                }
                transferred += 1;
            }
            self.a[ea_reg] = address;
            return 8 + transferred * if size == Size::Long { 8 } else { 4 };
        }

        if mode == 3 && !load {
            return 0;
        }
        let postincrement = mode == 3;
        let mut address = match mode {
            2 | 3 => self.a[ea_reg],
            5 => {
                let displacement = self.fetch16(bus) as i16 as i32 as u32;
                Self::mask_address(self.a[ea_reg].wrapping_add(displacement))
            }
            6 => {
                let extension = self.fetch16(bus);
                let displacement = extension as u8 as i8 as i32 as u32;
                Self::mask_address(
                    self.a[ea_reg]
                        .wrapping_add(displacement)
                        .wrapping_add(self.index_value(extension)),
                )
            }
            7 => match ea_reg {
                0 => Self::mask_address(self.fetch16(bus) as i16 as i32 as u32),
                1 => Self::mask_address(self.fetch32(bus)),
                2 if load => {
                    let base = self.pc;
                    let displacement = self.fetch16(bus) as i16 as i32 as u32;
                    Self::mask_address(base.wrapping_add(displacement))
                }
                3 if load => {
                    let base = self.pc;
                    let extension = self.fetch16(bus);
                    let displacement = extension as u8 as i8 as i32 as u32;
                    Self::mask_address(
                        base.wrapping_add(displacement)
                            .wrapping_add(self.index_value(extension)),
                    )
                }
                _ => return 0,
            },
            _ => return 0,
        };

        for register in 0..16usize {
            if mask & (1u16 << register) == 0 {
                continue;
            }
            if load {
                let value = match size {
                    Size::Word => Self::sign_extend(u32::from(bus.read16(address)), Size::Word),
                    Size::Long => bus.read32(address),
                    Size::Byte => unreachable!(),
                };
                self.set_movem_register(register, value);
            } else {
                let value = self.movem_register(register);
                match size {
                    Size::Word => bus.write16(address, value as u16),
                    Size::Long => bus.write32(address, value),
                    Size::Byte => unreachable!(),
                }
            }
            address = Self::mask_address(address.wrapping_add(bytes));
            transferred += 1;
        }
        if postincrement {
            self.a[ea_reg] = address;
        }
        let base = if load { 12 } else { 8 };
        base + transferred * if size == Size::Long { 8 } else { 4 }
    }

    fn move_instruction<B: Bus68000>(&mut self, bus: &mut B, opcode: u16) -> u32 {
        let size = match opcode >> 12 {
            1 => Size::Byte,
            2 => Size::Long,
            3 => Size::Word,
            _ => return 0,
        };
        let src_mode = (opcode >> 3) & 7;
        let src_reg = usize::from(opcode & 7);
        let dest_mode = (opcode >> 6) & 7;
        let dest_reg = usize::from((opcode >> 9) & 7);
        let Some(source) = self.resolve_ea(bus, src_mode, src_reg, size, true) else {
            return 0;
        };
        let value = self.read_ea(bus, source, size);
        if dest_mode == 1 {
            if size == Size::Byte {
                return 0;
            }
            self.a[dest_reg] = Self::mask_address(Self::sign_extend(value, size));
            return 8;
        }
        let Some(dest) = self.resolve_ea(bus, dest_mode, dest_reg, size, false) else {
            return 0;
        };
        if !self.write_ea(bus, dest, size, value) {
            return 0;
        }
        self.set_nz(value, size);
        8
    }

    fn execute_misc<B: Bus68000>(&mut self, bus: &mut B, opcode: u16) -> u32 {
        match opcode {
            0x4e71 => return 4,
            0x4e75 => {
                self.pc = Self::mask_address(self.pop32(bus));
                return 16;
            }
            0x4e73 => {
                if self.sr & S == 0 {
                    return self.exception(bus, 8);
                }
                let restored_sr = self.pop16(bus);
                let restored_pc = Self::mask_address(self.pop32(bus));
                self.set_sr(restored_sr);
                self.pc = restored_pc;
                return 20;
            }
            0x4e72 => {
                if self.sr & S == 0 {
                    return self.exception(bus, 8);
                }
                let next_sr = self.fetch16(bus);
                self.set_sr(next_sr);
                self.stopped = true;
                return 4;
            }
            0x4e70 => {
                return if self.sr & S == 0 {
                    self.exception(bus, 8)
                } else {
                    132
                };
            }
            0x4e76 => {
                return if self.flag(V) {
                    self.exception(bus, 7)
                } else {
                    4
                };
            }
            0x4e77 => {
                let ccr = self.pop16(bus) & 0x001f;
                self.sr = (self.sr & !0x001f) | ccr;
                self.pc = Self::mask_address(self.pop32(bus));
                return 20;
            }
            0x4afc => return self.exception(bus, 4),
            _ => {}
        }
        if opcode & 0xfff0 == 0x4e40 {
            return self.exception(bus, 32 + (opcode & 0x0f) as u8);
        }
        if opcode & 0xfff0 == 0x4e60 {
            if self.sr & S == 0 {
                return self.exception(bus, 8);
            }
            let reg = usize::from(opcode & 7);
            if opcode & 0x0008 == 0 {
                self.usp = Self::mask_address(self.a[reg]);
            } else {
                self.a[reg] = Self::mask_address(self.usp);
            }
            return 4;
        }
        if opcode & 0xfff8 == 0x4e50 {
            let reg = usize::from(opcode & 7);
            let displacement = self.fetch16(bus) as i16 as i32 as u32;
            self.push32(bus, self.a[reg]);
            self.a[reg] = self.a[7];
            self.a[7] = Self::mask_address(self.a[7].wrapping_add(displacement));
            return 16;
        }
        if opcode & 0xfff8 == 0x4e58 {
            let reg = usize::from(opcode & 7);
            self.a[7] = self.a[reg];
            self.a[reg] = self.pop32(bus);
            return 12;
        }
        if opcode & 0xffc0 == 0x4800 {
            return self.negate_bcd(bus, opcode);
        }
        if opcode & 0xfb80 == 0x4880 && ((opcode >> 3) & 7) >= 2 {
            return self.movem_instruction(bus, opcode);
        }
        if opcode & 0xfff8 == 0x4840 {
            let reg = usize::from(opcode & 7);
            let value = self.d[reg].rotate_left(16);
            self.d[reg] = value;
            self.set_nz(value, Size::Long);
            return 4;
        }
        if opcode & 0xfff8 == 0x4880 {
            let reg = usize::from(opcode & 7);
            let value = (self.d[reg] as u8 as i8 as i16) as u16;
            self.write_data_register(reg, u32::from(value), Size::Word);
            self.set_nz(u32::from(value), Size::Word);
            return 4;
        }
        if opcode & 0xfff8 == 0x48c0 {
            let reg = usize::from(opcode & 7);
            let value = (self.d[reg] as u16 as i16 as i32) as u32;
            self.d[reg] = value;
            self.set_nz(value, Size::Long);
            return 4;
        }
        if opcode & 0xffc0 == 0x4e80 {
            let Some(ea) = self.resolve_ea(
                bus,
                (opcode >> 3) & 7,
                usize::from(opcode & 7),
                Size::Long,
                false,
            ) else {
                return 0;
            };
            let Ea::Memory(target) = ea else { return 0 };
            self.push32(bus, self.pc);
            self.pc = target;
            return 16;
        }
        if opcode & 0xffc0 == 0x4ec0 {
            let Some(ea) = self.resolve_ea(
                bus,
                (opcode >> 3) & 7,
                usize::from(opcode & 7),
                Size::Long,
                false,
            ) else {
                return 0;
            };
            let Ea::Memory(target) = ea else { return 0 };
            self.pc = target;
            return 10;
        }
        if opcode & 0xf1c0 == 0x4180 {
            return self.check_bounds(bus, opcode);
        }
        if opcode & 0xf1c0 == 0x41c0 {
            let reg = usize::from((opcode >> 9) & 7);
            let Some(Ea::Memory(address)) = self.resolve_ea(
                bus,
                (opcode >> 3) & 7,
                usize::from(opcode & 7),
                Size::Long,
                false,
            ) else {
                return 0;
            };
            self.a[reg] = address;
            return 8;
        }
        if opcode & 0xffc0 == 0x4840 {
            let Some(Ea::Memory(address)) = self.resolve_ea(
                bus,
                (opcode >> 3) & 7,
                usize::from(opcode & 7),
                Size::Long,
                false,
            ) else {
                return 0;
            };
            self.push32(bus, address);
            return 12;
        }
        if opcode & 0xf0f8 == 0x50c8 {
            let condition = (opcode >> 8) & 0x0f;
            let reg = usize::from(opcode & 7);
            let base = self.pc;
            let displacement = self.fetch16(bus) as i16 as i32 as u32;
            if self.condition(condition) {
                return 12;
            }
            let count = (self.d[reg] as u16).wrapping_sub(1);
            self.d[reg] = (self.d[reg] & 0xffff_0000) | u32::from(count);
            if count != 0xffff {
                self.pc = Self::mask_address(base.wrapping_add(displacement));
                return 10;
            }
            return 14;
        }
        if opcode & 0xf0c0 == 0x50c0 {
            let condition = (opcode >> 8) & 0x0f;
            let Some(ea) = self.resolve_ea(
                bus,
                (opcode >> 3) & 7,
                usize::from(opcode & 7),
                Size::Byte,
                false,
            ) else {
                return 0;
            };
            let value = if self.condition(condition) {
                0xff
            } else {
                0x00
            };
            if !self.write_ea(bus, ea, Size::Byte, value) {
                return 0;
            }
            return 8;
        }
        if opcode & 0xf000 == 0x5000 {
            return self.quick_arithmetic(bus, opcode);
        }
        if opcode & 0xf138 == 0x0108 {
            return self.movep_instruction(bus, opcode);
        }
        if opcode & 0xff00 == 0x0800 {
            return self.bit_instruction(bus, opcode, true);
        }
        if opcode & 0xf000 == 0x0000 && opcode & 0x0100 != 0 {
            return self.bit_instruction(bus, opcode, false);
        }
        if matches!(opcode, 0x003c | 0x007c | 0x023c | 0x027c | 0x0a3c | 0x0a7c) {
            return self.immediate_status_instruction(bus, opcode);
        }
        if opcode & 0xffc0 == 0x40c0 {
            return self.move_from_sr(bus, opcode);
        }
        if opcode & 0xffc0 == 0x44c0 {
            return self.move_to_status(bus, opcode, false);
        }
        if opcode & 0xffc0 == 0x46c0 {
            return self.move_to_status(bus, opcode, true);
        }
        if opcode & 0xffc0 == 0x4ac0 {
            return self.test_and_set(bus, opcode);
        }
        if opcode & 0xff00 == 0x4000 {
            return self.unary_data_instruction(bus, opcode, 0);
        }
        if opcode & 0xff00 == 0x4200 {
            return self.clear_instruction(bus, opcode);
        }
        if opcode & 0xff00 == 0x4400 {
            return self.unary_data_instruction(bus, opcode, 1);
        }
        if opcode & 0xff00 == 0x4600 {
            return self.unary_data_instruction(bus, opcode, 2);
        }
        if opcode & 0xff00 == 0x4a00 {
            return self.test_instruction(bus, opcode);
        }
        if matches!(
            opcode & 0xff00,
            0x0000 | 0x0200 | 0x0400 | 0x0600 | 0x0a00 | 0x0c00
        ) {
            return self.immediate_instruction(bus, opcode);
        }
        if opcode & 0xf1f0 == 0x8100 {
            return self.bcd_extended(bus, opcode, true);
        }
        if opcode & 0xf1f0 == 0xc100 {
            return self.bcd_extended(bus, opcode, false);
        }
        if opcode & 0xf130 == 0x9100 {
            return self.extended_arithmetic(bus, opcode, true);
        }
        if opcode & 0xf130 == 0xd100 {
            return self.extended_arithmetic(bus, opcode, false);
        }
        if opcode & 0xf138 == 0xb108 {
            return self.compare_memory(bus, opcode);
        }
        if opcode & 0xf1f8 == 0xc140 || opcode & 0xf1f8 == 0xc148 || opcode & 0xf1f8 == 0xc188 {
            return self.exchange_registers(opcode);
        }
        let opmode = (opcode >> 6) & 7;
        if opcode >> 12 == 0x8 && matches!(opmode, 3 | 7) {
            return self.divide_word(bus, opcode, opmode == 7);
        }
        if opcode >> 12 == 0xc && matches!(opmode, 3 | 7) {
            return self.multiply_word(bus, opcode, opmode == 7);
        }
        if opcode & 0xf8c0 == 0xe0c0 {
            return self.shift_memory(bus, opcode);
        }
        match opcode >> 12 {
            0x8 => self.binary_register_op(bus, opcode, 0),
            0x9 => self.binary_register_op(bus, opcode, 1),
            0xb => self.compare_or_eor(bus, opcode),
            0xc => self.binary_register_op(bus, opcode, 2),
            0xd => self.binary_register_op(bus, opcode, 3),
            0xe => self.shift_register(opcode),
            _ => {
                self.stopped = true;
                0
            }
        }
    }

    fn exception<B: Bus68000>(&mut self, bus: &mut B, vector: u8) -> u32 {
        let old_sr = self.sr;
        self.set_sr(old_sr | S);
        self.push32(bus, self.pc);
        self.push16(bus, old_sr);
        self.pc = bus.read32(u32::from(vector) * 4) & 0x00ff_ffff;
        34
    }

    fn clear_instruction<B: Bus68000>(&mut self, bus: &mut B, opcode: u16) -> u32 {
        let Some(size) = Self::size_from_field((opcode >> 6) & 3) else {
            return 0;
        };
        let Some(ea) = self.data_alterable_ea(bus, opcode, size) else {
            return 0;
        };
        if !self.write_ea(bus, ea, size, 0) {
            return 0;
        }
        self.set_flag(N, false);
        self.set_flag(Z, true);
        self.set_flag(V, false);
        self.set_flag(C, false);
        8
    }

    fn test_instruction<B: Bus68000>(&mut self, bus: &mut B, opcode: u16) -> u32 {
        let Some(size) = Self::size_from_field((opcode >> 6) & 3) else {
            return 0;
        };
        let Some(ea) = self.data_alterable_ea(bus, opcode, size) else {
            return 0;
        };
        let value = self.read_ea(bus, ea, size);
        self.set_nz(value, size);
        4
    }
    fn quick_arithmetic<B: Bus68000>(&mut self, bus: &mut B, opcode: u16) -> u32 {
        let mut quick = u32::from((opcode >> 9) & 7);
        if quick == 0 {
            quick = 8;
        }
        let Some(size) = Self::size_from_field((opcode >> 6) & 3) else {
            return 0;
        };
        let mode = (opcode >> 3) & 7;
        let reg = usize::from(opcode & 7);
        if mode == 1 {
            if size == Size::Byte {
                return 0;
            }
            self.a[reg] = if opcode & 0x0100 != 0 {
                Self::mask_address(self.a[reg].wrapping_sub(quick))
            } else {
                Self::mask_address(self.a[reg].wrapping_add(quick))
            };
            return 8;
        }
        let Some(ea) = self.resolve_ea(bus, mode, reg, size, false) else {
            return 0;
        };
        let old = self.read_ea(bus, ea, size);
        let result = if opcode & 0x0100 != 0 {
            self.sub_value(old, quick, size)
        } else {
            self.add_value(old, quick, size)
        };
        if !self.write_ea(bus, ea, size, result) {
            return 0;
        }
        8
    }

    fn immediate_value<B: Bus68000>(&mut self, bus: &mut B, size: Size) -> u32 {
        match size {
            Size::Byte => u32::from(self.fetch16(bus) as u8),
            Size::Word => u32::from(self.fetch16(bus)),
            Size::Long => self.fetch32(bus),
        }
    }
    fn immediate_instruction<B: Bus68000>(&mut self, bus: &mut B, opcode: u16) -> u32 {
        let Some(size) = Self::size_from_field((opcode >> 6) & 3) else {
            return 0;
        };
        let immediate = self.immediate_value(bus, size);
        let Some(ea) = self.data_alterable_ea(bus, opcode, size) else {
            return 0;
        };
        let old = self.read_ea(bus, ea, size);
        let operation = opcode & 0xff00;
        if operation == 0x0c00 {
            self.cmp_value(old, immediate, size);
            return 8;
        }
        let result = match operation {
            0x0000 => {
                let value = old | immediate;
                self.set_nz(value, size);
                value
            }
            0x0200 => {
                let value = old & immediate;
                self.set_nz(value, size);
                value
            }
            0x0400 => self.sub_value(old, immediate, size),
            0x0600 => self.add_value(old, immediate, size),
            0x0a00 => {
                let value = old ^ immediate;
                self.set_nz(value, size);
                value
            }
            _ => return 0,
        };
        if !self.write_ea(bus, ea, size, result) {
            return 0;
        }
        8
    }
    fn data_alterable_ea<B: Bus68000>(
        &mut self,
        bus: &mut B,
        opcode: u16,
        size: Size,
    ) -> Option<Ea> {
        let mode = (opcode >> 3) & 7;
        let reg = usize::from(opcode & 7);
        if mode == 1 || (mode == 7 && reg > 1) {
            return None;
        }
        self.resolve_ea(bus, mode, reg, size, false)
    }

    fn memory_alterable_ea<B: Bus68000>(
        &mut self,
        bus: &mut B,
        opcode: u16,
        size: Size,
    ) -> Option<Ea> {
        let mode = (opcode >> 3) & 7;
        let reg = usize::from(opcode & 7);
        if mode < 2 || (mode == 7 && reg > 1) {
            return None;
        }
        self.resolve_ea(bus, mode, reg, size, false)
    }

    fn word_data_source<B: Bus68000>(&mut self, bus: &mut B, opcode: u16) -> Option<u16> {
        let mode = (opcode >> 3) & 7;
        let reg = usize::from(opcode & 7);
        if mode == 1 || (mode == 7 && reg > 4) {
            return None;
        }
        let ea = self.resolve_ea(bus, mode, reg, Size::Word, true)?;
        Some(self.read_ea(bus, ea, Size::Word) as u16)
    }

    fn exchange_registers(&mut self, opcode: u16) -> u32 {
        let rx = usize::from((opcode >> 9) & 7);
        let ry = usize::from(opcode & 7);
        match opcode & 0xf1f8 {
            0xc140 => self.d.swap(rx, ry),
            0xc148 => self.a.swap(rx, ry),
            0xc188 => std::mem::swap(&mut self.d[rx], &mut self.a[ry]),
            _ => unreachable!(),
        }
        6
    }

    fn multiply_word<B: Bus68000>(&mut self, bus: &mut B, opcode: u16, signed: bool) -> u32 {
        let Some(source) = self.word_data_source(bus, opcode) else {
            return 0;
        };
        let reg = usize::from((opcode >> 9) & 7);
        let result = if signed {
            let lhs = self.d[reg] as u16 as i16 as i32;
            let rhs = source as i16 as i32;
            lhs.wrapping_mul(rhs) as u32
        } else {
            u32::from(self.d[reg] as u16) * u32::from(source)
        };
        self.d[reg] = result;
        self.set_nz(result, Size::Long);
        70
    }

    fn divide_word<B: Bus68000>(&mut self, bus: &mut B, opcode: u16, signed: bool) -> u32 {
        let Some(source) = self.word_data_source(bus, opcode) else {
            return 0;
        };
        if source == 0 {
            return self.exception(bus, 5);
        }
        let reg = usize::from((opcode >> 9) & 7);
        if signed {
            let dividend = i64::from(self.d[reg] as i32);
            let divisor = i64::from(source as i16);
            let quotient = dividend / divisor;
            if !(-32768..=32767).contains(&quotient) {
                self.set_flag(V, true);
                self.set_flag(C, false);
                return 158;
            }
            let remainder = dividend % divisor;
            let quotient_word = quotient as i16 as u16;
            let remainder_word = remainder as i16 as u16;
            self.d[reg] = (u32::from(remainder_word) << 16) | u32::from(quotient_word);
            self.set_flag(N, quotient < 0);
            self.set_flag(Z, quotient == 0);
        } else {
            let dividend = u64::from(self.d[reg]);
            let divisor = u64::from(source);
            let quotient = dividend / divisor;
            if quotient > u64::from(u16::MAX) {
                self.set_flag(V, true);
                self.set_flag(C, false);
                return 140;
            }
            let remainder = dividend % divisor;
            self.d[reg] = ((remainder as u32) << 16) | quotient as u32;
            self.set_flag(N, quotient & 0x8000 != 0);
            self.set_flag(Z, quotient == 0);
        }
        self.set_flag(V, false);
        self.set_flag(C, false);
        if signed {
            158
        } else {
            140
        }
    }

    fn movep_instruction<B: Bus68000>(&mut self, bus: &mut B, opcode: u16) -> u32 {
        let data_reg = usize::from((opcode >> 9) & 7);
        let address_reg = usize::from(opcode & 7);
        let opmode = (opcode >> 6) & 7;
        let displacement = self.fetch16(bus) as i16 as i32 as u32;
        let address = Self::mask_address(self.a[address_reg].wrapping_add(displacement));
        match opmode {
            4 => {
                let value = (u16::from(bus.read8(address)) << 8)
                    | u16::from(bus.read8(Self::mask_address(address.wrapping_add(2))));
                self.write_data_register(data_reg, u32::from(value), Size::Word);
                16
            }
            5 => {
                let mut value = 0u32;
                for offset in 0..4u32 {
                    value = (value << 8)
                        | u32::from(
                            bus.read8(Self::mask_address(address.wrapping_add(offset * 2))),
                        );
                }
                self.d[data_reg] = value;
                24
            }
            6 => {
                let value = self.d[data_reg] as u16;
                bus.write8(address, (value >> 8) as u8);
                bus.write8(Self::mask_address(address.wrapping_add(2)), value as u8);
                16
            }
            7 => {
                let value = self.d[data_reg];
                for offset in 0..4u32 {
                    let shift = 24 - offset * 8;
                    bus.write8(
                        Self::mask_address(address.wrapping_add(offset * 2)),
                        (value >> shift) as u8,
                    );
                }
                24
            }
            _ => 0,
        }
    }

    fn bit_instruction<B: Bus68000>(
        &mut self,
        bus: &mut B,
        opcode: u16,
        immediate_bit: bool,
    ) -> u32 {
        let operation = usize::from((opcode >> 6) & 3);
        let mode = (opcode >> 3) & 7;
        let reg = usize::from(opcode & 7);
        if mode == 1 {
            return 0;
        }
        let bit_number = if immediate_bit {
            u32::from(self.fetch16(bus) as u8)
        } else {
            self.d[usize::from((opcode >> 9) & 7)]
        };
        let data_register = mode == 0;
        let size = if data_register {
            Size::Long
        } else {
            Size::Byte
        };
        let ea = if operation == 0 {
            let max_reg = if immediate_bit { 3 } else { 4 };
            if mode == 7 && reg > max_reg {
                return 0;
            }
            let Some(ea) = self.resolve_ea(bus, mode, reg, size, !immediate_bit) else {
                return 0;
            };
            ea
        } else {
            let Some(ea) = self.data_alterable_ea(bus, opcode, size) else {
                return 0;
            };
            ea
        };
        let modulo = if data_register { 32 } else { 8 };
        let bit = bit_number % modulo;
        let mask = 1u32 << bit;
        let value = self.read_ea(bus, ea, size);
        self.set_flag(Z, value & mask == 0);
        let result = match operation {
            0 => None,
            1 => Some(value ^ mask),
            2 => Some(value & !mask),
            3 => Some(value | mask),
            _ => unreachable!(),
        };
        if let Some(result) = result {
            if !self.write_ea(bus, ea, size, result) {
                return 0;
            }
        }
        if data_register {
            8
        } else {
            12
        }
    }

    fn unary_data_instruction<B: Bus68000>(&mut self, bus: &mut B, opcode: u16, kind: u8) -> u32 {
        let Some(size) = Self::size_from_field((opcode >> 6) & 3) else {
            return 0;
        };
        let Some(ea) = self.data_alterable_ea(bus, opcode, size) else {
            return 0;
        };
        let old = self.read_ea(bus, ea, size) & size.mask();
        let result = match kind {
            0 => {
                let old_z = self.flag(Z);
                let extend = u32::from(self.flag(X));
                let result = 0u32.wrapping_sub(old).wrapping_sub(extend) & size.mask();
                let borrow = old != 0 || extend != 0;
                self.set_flag(N, result & size.sign() != 0);
                self.set_flag(Z, old_z && result == 0);
                self.set_flag(V, old == size.sign() && extend == 0);
                self.set_flag(C, borrow);
                self.set_flag(X, borrow);
                result
            }
            1 => self.sub_value(0, old, size),
            2 => {
                let result = !old & size.mask();
                self.set_nz(result, size);
                result
            }
            _ => unreachable!(),
        };
        if !self.write_ea(bus, ea, size, result) {
            return 0;
        }
        if matches!(ea, Ea::Data(_)) {
            4
        } else {
            8
        }
    }

    fn test_and_set<B: Bus68000>(&mut self, bus: &mut B, opcode: u16) -> u32 {
        let Some(ea) = self.data_alterable_ea(bus, opcode, Size::Byte) else {
            return 0;
        };
        let value = self.read_ea(bus, ea, Size::Byte) as u8;
        self.set_flag(N, value & 0x80 != 0);
        self.set_flag(Z, value == 0);
        self.set_flag(V, false);
        self.set_flag(C, false);
        match ea {
            Ea::Data(reg) => self.write_data_register(reg, u32::from(value | 0x80), Size::Byte),
            Ea::Memory(address) => bus.tas_write8(address, value | 0x80),
            _ => return 0,
        }
        if matches!(ea, Ea::Data(_)) {
            4
        } else {
            10
        }
    }

    fn immediate_status_instruction<B: Bus68000>(&mut self, bus: &mut B, opcode: u16) -> u32 {
        let sr_target = opcode & 0x0040 != 0;
        if sr_target && self.sr & S == 0 {
            return self.exception(bus, 8);
        }
        let value = self.fetch16(bus);
        let operation = opcode & 0xff00;
        if sr_target {
            let next = match operation {
                0x0000 => self.sr | value,
                0x0200 => self.sr & value,
                0x0a00 => self.sr ^ value,
                _ => return 0,
            };
            self.set_sr(next);
            20
        } else {
            let current = self.sr & 0x001f;
            let operand = value & 0x001f;
            let ccr = match operation {
                0x0000 => current | operand,
                0x0200 => current & operand,
                0x0a00 => current ^ operand,
                _ => return 0,
            };
            self.sr = (self.sr & !0x001f) | ccr;
            20
        }
    }

    fn move_from_sr<B: Bus68000>(&mut self, bus: &mut B, opcode: u16) -> u32 {
        let Some(ea) = self.data_alterable_ea(bus, opcode, Size::Word) else {
            return 0;
        };
        if !self.write_ea(bus, ea, Size::Word, u32::from(self.sr)) {
            return 0;
        }
        if matches!(ea, Ea::Data(_)) {
            6
        } else {
            8
        }
    }

    fn move_to_status<B: Bus68000>(&mut self, bus: &mut B, opcode: u16, sr_target: bool) -> u32 {
        if sr_target && self.sr & S == 0 {
            return self.exception(bus, 8);
        }
        let Some(value) = self.word_data_source(bus, opcode) else {
            return 0;
        };
        if sr_target {
            self.set_sr(value);
        } else {
            self.sr = (self.sr & !0x001f) | (value & 0x001f);
        }
        12
    }

    fn bcd_extended<B: Bus68000>(&mut self, bus: &mut B, opcode: u16, subtract: bool) -> u32 {
        let destination = usize::from((opcode >> 9) & 7);
        let source = usize::from(opcode & 7);
        let memory = opcode & 0x0008 != 0;
        let (lhs, rhs, destination_ea) = if memory {
            self.a[source] =
                Self::mask_address(self.a[source].wrapping_sub(Size::Byte.bytes(source)));
            let rhs = self.read_ea(bus, Ea::Memory(self.a[source]), Size::Byte) as u8;
            self.a[destination] =
                Self::mask_address(self.a[destination].wrapping_sub(Size::Byte.bytes(destination)));
            let ea = Ea::Memory(self.a[destination]);
            (self.read_ea(bus, ea, Size::Byte) as u8, rhs, Some(ea))
        } else {
            (self.d[destination] as u8, self.d[source] as u8, None)
        };
        let extend = u16::from(self.flag(X));
        let (result, carry) = if subtract {
            let low_borrow = u16::from(lhs & 0x0f) < u16::from(rhs & 0x0f) + extend;
            let borrow = u16::from(lhs) < u16::from(rhs) + extend;
            let correction = u8::from(low_borrow) * 0x06 + u8::from(borrow) * 0x60;
            (
                lhs.wrapping_sub(rhs)
                    .wrapping_sub(extend as u8)
                    .wrapping_sub(correction),
                borrow,
            )
        } else {
            let mut adjusted = u16::from(lhs) + u16::from(rhs) + extend;
            if u16::from(lhs & 0x0f) + u16::from(rhs & 0x0f) + extend > 9 {
                adjusted += 0x06;
            }
            let carry = adjusted > 0x99;
            if carry {
                adjusted += 0x60;
            }
            (adjusted as u8, carry)
        };
        let old_zero = self.flag(Z);
        self.set_flag(Z, old_zero && result == 0);
        self.set_flag(C, carry);
        self.set_flag(X, carry);
        if let Some(ea) = destination_ea {
            if !self.write_ea(bus, ea, Size::Byte, u32::from(result)) {
                return 0;
            }
        } else {
            self.write_data_register(destination, u32::from(result), Size::Byte);
        }
        if memory {
            18
        } else {
            6
        }
    }

    fn signed_operand(value: u32, size: Size) -> i64 {
        match size {
            Size::Byte => i64::from(value as u8 as i8),
            Size::Word => i64::from(value as u16 as i16),
            Size::Long => i64::from(value as i32),
        }
    }

    fn extended_arithmetic<B: Bus68000>(
        &mut self,
        bus: &mut B,
        opcode: u16,
        subtract: bool,
    ) -> u32 {
        let Some(size) = Self::size_from_field((opcode >> 6) & 3) else {
            return 0;
        };
        let destination = usize::from((opcode >> 9) & 7);
        let source = usize::from(opcode & 7);
        let memory = opcode & 0x0008 != 0;
        let (lhs, rhs, destination_ea) = if memory {
            self.a[source] = Self::mask_address(self.a[source].wrapping_sub(size.bytes(source)));
            let rhs = self.read_ea(bus, Ea::Memory(self.a[source]), size);
            self.a[destination] =
                Self::mask_address(self.a[destination].wrapping_sub(size.bytes(destination)));
            let ea = Ea::Memory(self.a[destination]);
            (self.read_ea(bus, ea, size), rhs, Some(ea))
        } else {
            (
                self.d[destination] & size.mask(),
                self.d[source] & size.mask(),
                None,
            )
        };
        let extend = u64::from(self.flag(X));
        let mask = size.mask();
        let sign = size.sign();
        let result = if subtract {
            lhs.wrapping_sub(rhs).wrapping_sub(extend as u32) & mask
        } else {
            lhs.wrapping_add(rhs).wrapping_add(extend as u32) & mask
        };
        let carry = if subtract {
            u64::from(lhs) < u64::from(rhs) + extend
        } else {
            u64::from(lhs) + u64::from(rhs) + extend > u64::from(mask)
        };
        let signed_result = if subtract {
            Self::signed_operand(lhs, size) - Self::signed_operand(rhs, size) - extend as i64
        } else {
            Self::signed_operand(lhs, size) + Self::signed_operand(rhs, size) + extend as i64
        };
        let signed_limit = i64::from(sign);
        let overflow = signed_result < -signed_limit || signed_result >= signed_limit;
        let old_zero = self.flag(Z);
        self.set_flag(N, result & sign != 0);
        self.set_flag(Z, old_zero && result == 0);
        self.set_flag(V, overflow);
        self.set_flag(C, carry);
        self.set_flag(X, carry);
        if let Some(ea) = destination_ea {
            if !self.write_ea(bus, ea, size, result) {
                return 0;
            }
        } else {
            self.write_data_register(destination, result, size);
        }
        if memory {
            if size == Size::Long {
                30
            } else {
                18
            }
        } else if size == Size::Long {
            6
        } else {
            4
        }
    }

    fn compare_memory<B: Bus68000>(&mut self, bus: &mut B, opcode: u16) -> u32 {
        let Some(size) = Self::size_from_field((opcode >> 6) & 3) else {
            return 0;
        };
        let destination = usize::from((opcode >> 9) & 7);
        let source = usize::from(opcode & 7);
        let rhs = self.read_ea(bus, Ea::Memory(self.a[source]), size);
        self.a[source] = Self::mask_address(self.a[source].wrapping_add(size.bytes(source)));
        let lhs = self.read_ea(bus, Ea::Memory(self.a[destination]), size);
        self.a[destination] =
            Self::mask_address(self.a[destination].wrapping_add(size.bytes(destination)));
        self.cmp_value(lhs, rhs, size);
        if size == Size::Long {
            20
        } else {
            12
        }
    }

    fn binary_register_op<B: Bus68000>(&mut self, bus: &mut B, opcode: u16, kind: u8) -> u32 {
        let reg = usize::from((opcode >> 9) & 7);
        let opmode = (opcode >> 6) & 7;
        let mode = (opcode >> 3) & 7;
        let ea_reg = usize::from(opcode & 7);
        if matches!(opmode, 3 | 7) && matches!(kind, 1 | 3) {
            let size = if opmode == 3 { Size::Word } else { Size::Long };
            let Some(ea) = self.resolve_ea(bus, mode, ea_reg, size, true) else {
                return 0;
            };
            let mut value = self.read_ea(bus, ea, size);
            if size == Size::Word {
                value = Self::sign_extend(value, size);
            }
            self.a[reg] = if kind == 1 {
                Self::mask_address(self.a[reg].wrapping_sub(value))
            } else {
                Self::mask_address(self.a[reg].wrapping_add(value))
            };
            return 8;
        }
        let (size, direction_to_ea) = match opmode {
            0 => (Size::Byte, false),
            1 => (Size::Word, false),
            2 => (Size::Long, false),
            4 => (Size::Byte, true),
            5 => (Size::Word, true),
            6 => (Size::Long, true),
            _ => return 0,
        };
        let Some(ea) = self.resolve_ea(bus, mode, ea_reg, size, true) else {
            return 0;
        };
        let ea_value = self.read_ea(bus, ea, size);
        let d_value = self.d[reg] & size.mask();
        let (lhs, rhs) = if direction_to_ea {
            (ea_value, d_value)
        } else {
            (d_value, ea_value)
        };
        let result = match kind {
            0 => {
                let v = lhs | rhs;
                self.set_nz(v, size);
                v
            }
            1 => self.sub_value(lhs, rhs, size),
            2 => {
                let v = lhs & rhs;
                self.set_nz(v, size);
                v
            }
            3 => self.add_value(lhs, rhs, size),
            _ => unreachable!(),
        };
        if direction_to_ea {
            if !self.write_ea(bus, ea, size, result) {
                return 0;
            }
        } else {
            self.write_data_register(reg, result, size);
        }
        8
    }
    fn compare_or_eor<B: Bus68000>(&mut self, bus: &mut B, opcode: u16) -> u32 {
        let reg = usize::from((opcode >> 9) & 7);
        let opmode = (opcode >> 6) & 7;
        let mode = (opcode >> 3) & 7;
        let ea_reg = usize::from(opcode & 7);
        if matches!(opmode, 0..=2) {
            let size = [Size::Byte, Size::Word, Size::Long][opmode as usize];
            let Some(ea) = self.resolve_ea(bus, mode, ea_reg, size, true) else {
                return 0;
            };
            let value = self.read_ea(bus, ea, size);
            self.cmp_value(self.d[reg], value, size);
            return 6;
        }
        if matches!(opmode, 3 | 7) {
            let size = if opmode == 3 { Size::Word } else { Size::Long };
            let Some(ea) = self.resolve_ea(bus, mode, ea_reg, size, true) else {
                return 0;
            };
            let mut value = self.read_ea(bus, ea, size);
            if size == Size::Word {
                value = Self::sign_extend(value, size);
            }
            self.cmp_value(self.a[reg], value, Size::Long);
            return 6;
        }
        let size = match opmode {
            4 => Size::Byte,
            5 => Size::Word,
            6 => Size::Long,
            _ => return 0,
        };
        let Some(ea) = self.resolve_ea(bus, mode, ea_reg, size, false) else {
            return 0;
        };
        let value = self.read_ea(bus, ea, size) ^ (self.d[reg] & size.mask());
        self.set_nz(value, size);
        if !self.write_ea(bus, ea, size, value) {
            return 0;
        }
        8
    }
    fn shift_memory<B: Bus68000>(&mut self, bus: &mut B, opcode: u16) -> u32 {
        let Some(ea) = self.memory_alterable_ea(bus, opcode, Size::Word) else {
            return 0;
        };
        let kind = (opcode >> 9) & 3;
        let left = opcode & 0x0100 != 0;
        let old = self.read_ea(bus, ea, Size::Word) as u16;
        let extend = self.flag(X);
        let (result, carry, overflow) = if left {
            let carry = old & 0x8000 != 0;
            let mut value = old << 1;
            if (kind == 2 && extend) || (kind == 3 && carry) {
                value |= 1;
            }
            (value, carry, kind == 0 && (old ^ value) & 0x8000 != 0)
        } else {
            let carry = old & 1 != 0;
            let mut value = old >> 1;
            if (kind == 0 && old & 0x8000 != 0) || (kind == 2 && extend) || (kind == 3 && carry) {
                value |= 0x8000;
            }
            (value, carry, false)
        };
        if !self.write_ea(bus, ea, Size::Word, u32::from(result)) {
            return 0;
        }
        if kind != 3 {
            self.set_flag(X, carry);
        }
        self.set_flag(N, result & 0x8000 != 0);
        self.set_flag(Z, result == 0);
        self.set_flag(V, overflow);
        self.set_flag(C, carry);
        12
    }

    fn shift_register(&mut self, opcode: u16) -> u32 {
        let size = match (opcode >> 6) & 3 {
            0 => Size::Byte,
            1 => Size::Word,
            2 => Size::Long,
            _ => return 0,
        };
        let dest = usize::from(opcode & 7);
        let register_count = opcode & 0x0020 != 0;
        let mut count = if register_count {
            self.d[usize::from((opcode >> 9) & 7)] & 0x3f
        } else {
            let encoded = u32::from((opcode >> 9) & 7);
            if encoded == 0 {
                8
            } else {
                encoded
            }
        };
        let left = opcode & 0x0100 != 0;
        let kind = (opcode >> 3) & 3;
        let mask = size.mask();
        let sign = size.sign();
        let mut value = self.d[dest] & mask;
        let mut carry = false;
        let mut overflow = false;
        while count != 0 {
            count -= 1;
            if left {
                let old_sign = value & sign;
                carry = old_sign != 0;
                value = (value << 1) & mask;
                if kind == 0 && (value & sign) != old_sign {
                    overflow = true;
                }
                if kind == 2 && self.flag(X) {
                    value |= 1;
                }
                if kind == 3 && carry {
                    value |= 1;
                }
            } else {
                carry = value & 1 != 0;
                let old_sign = value & sign;
                value >>= 1;
                if kind == 0 {
                    value |= old_sign;
                }
                if kind == 2 && self.flag(X) {
                    value |= sign;
                }
                if kind == 3 && carry {
                    value |= sign;
                }
            }
            if kind != 3 {
                self.set_flag(X, carry);
            }
        }
        self.write_data_register(dest, value, size);
        self.set_flag(N, value & sign != 0);
        self.set_flag(Z, value == 0);
        self.set_flag(V, kind == 0 && left && overflow);
        self.set_flag(C, carry);
        6
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct TestBus {
        bytes: Vec<u8>,
    }
    impl TestBus {
        fn new() -> Self {
            Self {
                bytes: vec![0; 0x10000],
            }
        }
        fn put16(&mut self, address: usize, value: u16) {
            self.bytes[address..address + 2].copy_from_slice(&value.to_be_bytes());
        }
        fn put32(&mut self, address: usize, value: u32) {
            self.bytes[address..address + 4].copy_from_slice(&value.to_be_bytes());
        }
        fn boot(&mut self, pc: u32) {
            self.put32(0, 0x0000_8000);
            self.put32(4, pc);
        }
    }
    impl Bus68000 for TestBus {
        fn read8(&mut self, address: u32) -> u8 {
            self.bytes[address as usize]
        }
        fn write8(&mut self, address: u32, value: u8) {
            self.bytes[address as usize] = value;
        }
    }

    #[test]
    fn reset_moveq_immediate_arithmetic_and_compare_execute() {
        let mut bus = TestBus::new();
        bus.boot(0x0100);
        let words = [
            0x7005, 0x0680, 0x0000, 0x0003, 0x0c80, 0x0000, 0x0008, 0x4e71,
        ];
        for (index, word) in words.into_iter().enumerate() {
            bus.put16(0x100 + index * 2, word);
        }
        let mut cpu = M68000::default();
        cpu.reset(&mut bus);
        assert_eq!(cpu.a[7], 0x8000);
        assert_eq!(cpu.pc, 0x0100);
        assert_ne!(cpu.step(&mut bus), 0);
        assert_ne!(cpu.step(&mut bus), 0);
        assert_eq!(cpu.d[0], 8);
        assert_ne!(cpu.step(&mut bus), 0);
        assert!(cpu.flag(Z));
    }
    #[test]
    fn move_long_and_memory_indirect_use_big_endian_bus() {
        let mut bus = TestBus::new();
        bus.boot(0x0100);
        let words = [0x223c, 0x1234, 0x5678, 0x2081, 0x4e71];
        for (index, word) in words.into_iter().enumerate() {
            bus.put16(0x100 + index * 2, word);
        }
        let mut cpu = M68000::default();
        cpu.reset(&mut bus);
        cpu.a[0] = 0x0200;
        assert_ne!(cpu.step(&mut bus), 0);
        assert_eq!(cpu.d[1], 0x1234_5678);
        assert_ne!(cpu.step(&mut bus), 0);
        assert_eq!(&bus.bytes[0x200..0x204], &[0x12, 0x34, 0x56, 0x78]);
    }

    #[test]
    fn jsr_and_rts_round_trip_stack_and_pc() {
        let mut bus = TestBus::new();
        bus.boot(0x0100);
        bus.put16(0x100, 0x4eb9);
        bus.put32(0x102, 0x0000_0120);
        bus.put16(0x106, 0x4e71);
        bus.put16(0x120, 0x742a);
        bus.put16(0x122, 0x4e75);
        let mut cpu = M68000::default();
        cpu.reset(&mut bus);
        assert_ne!(cpu.step(&mut bus), 0);
        assert_eq!(cpu.pc, 0x0120);
        assert_eq!(cpu.a[7], 0x7ffc);
        assert_ne!(cpu.step(&mut bus), 0);
        assert_eq!(cpu.d[2], 42);
        assert_ne!(cpu.step(&mut bus), 0);
        assert_eq!(cpu.pc, 0x0106);
        assert_eq!(cpu.a[7], 0x8000);
    }
    #[test]
    fn condition_branch_and_dbcc_update_control_flow() {
        let mut bus = TestBus::new();
        bus.boot(0x0100);
        let words = [
            0x7000, 0x0c00, 0x0000, 0x6702, 0x7201, 0x7202, 0x51c8, 0xfffc,
        ];
        for (index, word) in words.into_iter().enumerate() {
            bus.put16(0x100 + index * 2, word);
        }
        let mut cpu = M68000::default();
        cpu.reset(&mut bus);
        cpu.step(&mut bus);
        cpu.step(&mut bus);
        assert!(cpu.flag(Z));
        cpu.step(&mut bus);
        assert_eq!(cpu.pc, 0x010a);
        cpu.step(&mut bus);
        assert_eq!(cpu.d[1], 2);
        cpu.d[0] = 1;
        cpu.step(&mut bus);
        assert_eq!(cpu.d[0] & 0xffff, 0);
    }

    #[test]
    fn supervisor_and_user_stack_banks_round_trip_through_exception_and_rte() {
        let mut bus = TestBus::new();
        bus.boot(0x0100);
        bus.put32(32 * 4, 0x0000_0300);
        bus.put16(0x100, 0x4e60); // MOVE A0,USP
        bus.put16(0x102, 0x46fc); // MOVE #0,SR
        bus.put16(0x104, 0x0000);
        bus.put16(0x106, 0x4e40); // TRAP #0
        bus.put16(0x108, 0x4e71);
        bus.put16(0x300, 0x4e69); // MOVE USP,A1
        bus.put16(0x302, 0x4e73); // RTE
        let mut cpu = M68000::default();
        cpu.reset(&mut bus);
        cpu.a[0] = 0x0000_7000;

        assert_eq!(cpu.step(&mut bus), 4);
        assert_eq!(cpu.usp, 0x0000_7000);
        assert_eq!(cpu.a[7], 0x0000_8000);

        assert_eq!(cpu.step(&mut bus), 12);
        assert_eq!(cpu.sr & S, 0);
        assert_eq!(cpu.a[7], 0x0000_7000);
        assert_eq!(cpu.ssp, 0x0000_8000);

        assert_eq!(cpu.step(&mut bus), 34);
        assert_eq!(cpu.pc, 0x0000_0300);
        assert_eq!(cpu.sr & S, S);
        assert_eq!(cpu.a[7], 0x0000_7ffa);
        assert_eq!(bus.read16(0x7ffa), 0x0000);
        assert_eq!(bus.read32(0x7ffc), 0x0000_0108);

        assert_eq!(cpu.step(&mut bus), 4);
        assert_eq!(cpu.a[1], 0x0000_7000);
        assert_eq!(cpu.step(&mut bus), 20);
        assert_eq!(cpu.pc, 0x0000_0108);
        assert_eq!(cpu.sr & S, 0);
        assert_eq!(cpu.a[7], 0x0000_7000);
        assert_eq!(cpu.ssp, 0x0000_8000);
    }

    #[test]
    fn move_usp_is_privileged_in_user_mode() {
        let mut bus = TestBus::new();
        bus.boot(0x0100);
        bus.put32(8 * 4, 0x0000_0340);
        bus.put16(0x100, 0x4e60); // MOVE A0,USP
        bus.put16(0x102, 0x46fc); // MOVE #0,SR
        bus.put16(0x104, 0x0000);
        bus.put16(0x106, 0x4e68); // MOVE USP,A0
        let mut cpu = M68000::default();
        cpu.reset(&mut bus);
        cpu.a[0] = 0x0000_7000;

        cpu.step(&mut bus);
        cpu.step(&mut bus);
        assert_eq!(cpu.sr & S, 0);
        assert_eq!(cpu.step(&mut bus), 34);
        assert_eq!(cpu.pc, 0x0000_0340);
        assert_eq!(cpu.sr & S, S);
        assert_eq!(cpu.a[7], 0x0000_7ffa);
        assert_eq!(bus.read32(0x7ffc), 0x0000_0108);
    }

    #[test]
    fn chk_and_nbcd_cover_bounds_and_decimal_negation() {
        let mut bus = TestBus::new();
        bus.boot(0x0100);
        bus.put32(6 * 4, 0x0000_0300);
        bus.put16(0x100, 0x41bc); // CHK.W #10,D0
        bus.put16(0x102, 10);
        bus.put16(0x104, 0x41bc); // CHK.W #10,D0
        bus.put16(0x106, 10);
        bus.put16(0x300, 0x4801); // NBCD D1
        bus.put16(0x302, 0x4810); // NBCD (A0)
        let mut cpu = M68000::default();
        cpu.reset(&mut bus);

        cpu.d[0] = 5;
        assert_eq!(cpu.step(&mut bus), 10);
        assert_eq!(cpu.pc, 0x0104);

        cpu.d[0] = 0xffff_ffff;
        assert_eq!(cpu.step(&mut bus), 40);
        assert_eq!(cpu.pc, 0x0300);
        assert!(cpu.flag(N));

        cpu.d[1] = 1;
        cpu.sr = (cpu.sr & !(X | C)) | Z;
        assert_eq!(cpu.step(&mut bus), 6);
        assert_eq!(cpu.d[1] & 0xff, 0x99);
        assert!(cpu.flag(C));
        assert!(cpu.flag(X));
        assert!(!cpu.flag(Z));

        cpu.a[0] = 0x0400;
        bus.write8(0x0400, 1);
        cpu.sr = (cpu.sr & !(X | C)) | Z;
        assert_eq!(cpu.step(&mut bus), 8);
        assert_eq!(bus.read8(0x0400), 0x99);
        assert!(cpu.flag(C));
        assert!(cpu.flag(X));
    }

    #[test]
    fn decimal_extended_arithmetic_supports_register_and_predecrement_forms() {
        let mut bus = TestBus::new();
        bus.boot(0x0100);
        bus.put16(0x100, 0xc300); // ABCD D0,D1
        bus.put16(0x102, 0x8300); // SBCD D0,D1
        bus.put16(0x104, 0xc308); // ABCD -(A0),-(A1)
        let mut cpu = M68000::default();
        cpu.reset(&mut bus);

        cpu.d[0] = 0x45;
        cpu.d[1] = 0x54;
        cpu.sr = (cpu.sr & !(X | C)) | Z;
        assert_eq!(cpu.step(&mut bus), 6);
        assert_eq!(cpu.d[1] & 0xff, 0x99);
        assert!(!cpu.flag(C));
        assert!(!cpu.flag(X));
        assert!(!cpu.flag(Z));

        cpu.d[0] = 0x01;
        cpu.d[1] = 0x00;
        cpu.sr = (cpu.sr & !(X | C)) | Z;
        assert_eq!(cpu.step(&mut bus), 6);
        assert_eq!(cpu.d[1] & 0xff, 0x99);
        assert!(cpu.flag(C));
        assert!(cpu.flag(X));
        assert!(!cpu.flag(Z));

        cpu.a[0] = 0x0300;
        cpu.a[1] = 0x0400;
        bus.write8(0x02ff, 0x01);
        bus.write8(0x03ff, 0x99);
        cpu.sr = (cpu.sr & !(X | C)) | Z;
        assert_eq!(cpu.step(&mut bus), 18);
        assert_eq!(cpu.a[0], 0x02ff);
        assert_eq!(cpu.a[1], 0x03ff);
        assert_eq!(bus.read8(0x03ff), 0x00);
        assert!(cpu.flag(C));
        assert!(cpu.flag(X));
        assert!(cpu.flag(Z));
    }

    #[test]
    fn extended_arithmetic_and_cmpm_cover_register_and_memory_forms() {
        let mut bus = TestBus::new();
        bus.boot(0x0100);
        bus.put16(0x100, 0xd340); // ADDX.W D0,D1
        bus.put16(0x102, 0x9380); // SUBX.L D0,D1
        bus.put16(0x104, 0xd348); // ADDX.W -(A0),-(A1)
        bus.put16(0x106, 0xb348); // CMPM.W (A0)+,(A1)+
        let mut cpu = M68000::default();
        cpu.reset(&mut bus);

        cpu.d[0] = 1;
        cpu.d[1] = 0x0000_7fff;
        cpu.sr |= Z;
        assert_eq!(cpu.step(&mut bus), 4);
        assert_eq!(cpu.d[1], 0x0000_8000);
        assert!(cpu.flag(N));
        assert!(cpu.flag(V));
        assert!(!cpu.flag(C));
        assert!(!cpu.flag(Z));

        cpu.d[0] = 1;
        cpu.d[1] = 0;
        cpu.sr |= X | Z;
        assert_eq!(cpu.step(&mut bus), 6);
        assert_eq!(cpu.d[1], 0xffff_fffe);
        assert!(cpu.flag(N));
        assert!(cpu.flag(C));
        assert!(cpu.flag(X));
        assert!(!cpu.flag(Z));

        cpu.a[0] = 0x0300;
        cpu.a[1] = 0x0400;
        bus.put16(0x02fe, 2);
        bus.put16(0x03fe, 3);
        cpu.sr |= X | Z;
        assert_eq!(cpu.step(&mut bus), 18);
        assert_eq!(cpu.a[0], 0x02fe);
        assert_eq!(cpu.a[1], 0x03fe);
        assert_eq!(bus.read16(0x03fe), 6);
        assert!(!cpu.flag(C));
        assert!(!cpu.flag(Z));

        cpu.a[0] = 0x0500;
        cpu.a[1] = 0x0600;
        bus.put16(0x0500, 0x1234);
        bus.put16(0x0600, 0x1234);
        cpu.sr |= X;
        assert_eq!(cpu.step(&mut bus), 12);
        assert_eq!(cpu.a[0], 0x0502);
        assert_eq!(cpu.a[1], 0x0602);
        assert!(cpu.flag(Z));
        assert!(cpu.flag(X));
    }

    #[test]
    fn movep_and_bit_operations_preserve_line_zero_decode_boundaries() {
        let mut bus = TestBus::new();
        bus.boot(0x0100);
        bus.put16(0x100, 0x01c8); // MOVEP.L D0,(4,A0)
        bus.put16(0x102, 0x0004);
        bus.put16(0x104, 0x0348); // MOVEP.L (4,A0),D1
        bus.put16(0x106, 0x0004);
        bus.put16(0x108, 0x08c2); // BSET #3,D2
        bus.put16(0x10a, 0x0003);
        bus.put16(0x10c, 0x0751); // BCHG D3,(A1)
        bus.put16(0x10e, 0x0711); // BTST D3,(A1)
        let mut cpu = M68000::default();
        cpu.reset(&mut bus);
        cpu.d[0] = 0x1122_3344;
        cpu.d[2] = 0;
        cpu.d[3] = 1;
        cpu.a[0] = 0x0200;
        cpu.a[1] = 0x0300;
        bus.write8(0x0300, 0x02);

        assert_ne!(cpu.step(&mut bus), 0);
        assert_eq!(bus.read8(0x0204), 0x11);
        assert_eq!(bus.read8(0x0206), 0x22);
        assert_eq!(bus.read8(0x0208), 0x33);
        assert_eq!(bus.read8(0x020a), 0x44);
        assert_ne!(cpu.step(&mut bus), 0);
        assert_eq!(cpu.d[1], 0x1122_3344);
        assert_ne!(cpu.step(&mut bus), 0);
        assert_eq!(cpu.d[2], 0x0000_0008);
        assert!(cpu.flag(Z));
        assert_ne!(cpu.step(&mut bus), 0);
        assert_eq!(bus.read8(0x0300), 0x00);
        assert!(!cpu.flag(Z));
        assert_ne!(cpu.step(&mut bus), 0);
        assert!(cpu.flag(Z));
    }

    #[test]
    fn memory_tas_uses_bus_specific_write_cycle() {
        let mut bus = TestBus::new();
        bus.boot(0x0100);
        bus.put16(0x0100, 0x4ad0); // TAS (A0)
        bus.write8(0x0200, 0x01);
        let mut cpu = M68000::default();
        cpu.reset(&mut bus);
        cpu.a[0] = 0x0200;

        assert_eq!(cpu.step(&mut bus), 10);
        assert_eq!(bus.read8(0x0200), 0x81);
        assert!(!cpu.flag(Z));
        assert!(!cpu.flag(N));
    }
    #[test]
    fn status_unary_and_tas_instructions_execute_without_decoder_aliases() {
        let mut bus = TestBus::new();
        bus.boot(0x0100);
        let words = [
            0x003c, 0x0011, // ORI #$11,CCR
            0x023c, 0x000f, // ANDI #$0f,CCR
            0x0a3c, 0x0005, // EORI #$05,CCR
            0x40c0, // MOVE SR,D0
            0x44fc, 0x0011, // MOVE #$11,CCR
            0x46fc, 0x2700, // MOVE #$2700,SR
            0x4441, // NEG.W D1
            0x4602, // NOT.B D2
            0x4ac3, // TAS D3
        ];
        for (index, word) in words.into_iter().enumerate() {
            bus.put16(0x100 + index * 2, word);
        }
        let mut cpu = M68000::default();
        cpu.reset(&mut bus);
        cpu.d[0] = 0xaaaa_0000;
        cpu.d[1] = 1;
        cpu.d[2] = 0x1234_560f;
        cpu.d[3] = 1;

        cpu.step(&mut bus);
        assert_eq!(cpu.sr & 0x1f, 0x11);
        cpu.step(&mut bus);
        assert_eq!(cpu.sr & 0x1f, 0x01);
        cpu.step(&mut bus);
        assert_eq!(cpu.sr & 0x1f, 0x04);
        cpu.step(&mut bus);
        assert_eq!(cpu.d[0], 0xaaaa_2704);
        cpu.step(&mut bus);
        assert_eq!(cpu.sr & 0x1f, 0x11);
        cpu.step(&mut bus);
        assert_eq!(cpu.sr, 0x2700);
        cpu.step(&mut bus);
        assert_eq!(cpu.d[1] & 0xffff, 0xffff);
        assert!(cpu.flag(N) && cpu.flag(C) && cpu.flag(X));
        cpu.step(&mut bus);
        assert_eq!(cpu.d[2], 0x1234_56f0);
        assert!(cpu.flag(N));
        cpu.step(&mut bus);
        assert_eq!(cpu.d[3] & 0xff, 0x81);
        assert!(!cpu.flag(N));
        assert!(!cpu.flag(Z));
    }

    #[test]
    fn explicit_illegal_opcode_uses_vector_four_instead_of_stopping_host() {
        let mut bus = TestBus::new();
        bus.boot(0x0100);
        bus.put32(4 * 4, 0x0000_0300);
        bus.put16(0x100, 0x4afc);
        let mut cpu = M68000::default();
        cpu.reset(&mut bus);

        assert_ne!(cpu.step(&mut bus), 0);
        assert_eq!(cpu.pc, 0x0300);
        assert!(!cpu.stopped);
    }

    #[test]
    fn multiply_divide_and_exchange_cover_common_compiler_sequences() {
        let mut bus = TestBus::new();
        bus.boot(0x0100);
        let words = [
            0xc0fc, 0x0007, // MULU.W #7,D0
            0xc3fc, 0xfffe, // MULS.W #-2,D1
            0x80fc, 0x0005, // DIVU.W #5,D0
            0x83fc, 0xfffe, // DIVS.W #-2,D1
            0xc141, // EXG D0,D1
            0xc149, // EXG A0,A1
            0xc58a, // EXG D2,A2
        ];
        for (index, word) in words.into_iter().enumerate() {
            bus.put16(0x100 + index * 2, word);
        }
        let mut cpu = M68000::default();
        cpu.reset(&mut bus);
        cpu.d[0] = 6;
        cpu.d[1] = 3;
        cpu.d[2] = 0x0012_3456;
        cpu.a[0] = 0x0000_1111;
        cpu.a[1] = 0x0000_2222;
        cpu.a[2] = 0x0000_3333;

        assert_ne!(cpu.step(&mut bus), 0);
        assert_eq!(cpu.d[0], 42);
        assert_ne!(cpu.step(&mut bus), 0);
        assert_eq!(cpu.d[1], 0xffff_fffa);
        assert_ne!(cpu.step(&mut bus), 0);
        assert_eq!(cpu.d[0], 0x0002_0008);
        assert_ne!(cpu.step(&mut bus), 0);
        assert_eq!(cpu.d[1], 3);
        assert_ne!(cpu.step(&mut bus), 0);
        assert_eq!(cpu.d[0], 3);
        assert_eq!(cpu.d[1], 0x0002_0008);
        assert_ne!(cpu.step(&mut bus), 0);
        assert_eq!(cpu.a[0], 0x0000_2222);
        assert_eq!(cpu.a[1], 0x0000_1111);
        assert_ne!(cpu.step(&mut bus), 0);
        assert_eq!(cpu.d[2], 0x0000_3333);
        assert_eq!(cpu.a[2], 0x0012_3456);
    }

    #[test]
    fn divide_overflow_preserves_destination_and_sets_overflow() {
        let mut bus = TestBus::new();
        bus.boot(0x0100);
        bus.put16(0x100, 0x80fc); // DIVU.W #1,D0
        bus.put16(0x102, 0x0001);
        let mut cpu = M68000::default();
        cpu.reset(&mut bus);
        cpu.d[0] = 0x0001_0000;
        cpu.sr |= X;

        assert_ne!(cpu.step(&mut bus), 0);
        assert_eq!(cpu.d[0], 0x0001_0000);
        assert!(cpu.flag(V));
        assert!(!cpu.flag(C));
        assert!(cpu.flag(X));
    }

    #[test]
    fn movem_long_predecrement_and_postincrement_round_trip() {
        let mut bus = TestBus::new();
        bus.boot(0x0100);
        bus.put16(0x100, 0x48e7); // MOVEM.L D0-D1/A0-A1,-(A7)
        bus.put16(0x102, 0xc0c0);
        bus.put16(0x104, 0x4cdf); // MOVEM.L (A7)+,D0-D1/A0-A1
        bus.put16(0x106, 0x0303);
        let mut cpu = M68000::default();
        cpu.reset(&mut bus);
        cpu.d[0] = 0x1122_3344;
        cpu.d[1] = 0x5566_7788;
        cpu.a[0] = 0x0012_3456;
        cpu.a[1] = 0x0065_4321;

        assert_ne!(cpu.step(&mut bus), 0);
        assert_eq!(cpu.a[7], 0x7ff0);
        assert_eq!(bus.read32(0x7ff0), 0x1122_3344);
        assert_eq!(bus.read32(0x7ff4), 0x5566_7788);
        assert_eq!(bus.read32(0x7ff8), 0x0012_3456);
        assert_eq!(bus.read32(0x7ffc), 0x0065_4321);

        cpu.d[0] = 0;
        cpu.d[1] = 0;
        cpu.a[0] = 0;
        cpu.a[1] = 0;
        assert_ne!(cpu.step(&mut bus), 0);
        assert_eq!(cpu.d[0], 0x1122_3344);
        assert_eq!(cpu.d[1], 0x5566_7788);
        assert_eq!(cpu.a[0], 0x0012_3456);
        assert_eq!(cpu.a[1], 0x0065_4321);
        assert_eq!(cpu.a[7], 0x8000);
    }

    #[test]
    fn movem_preserves_mc68000_address_register_update_rules() {
        let mut bus = TestBus::new();
        bus.boot(0x0100);
        bus.put16(0x100, 0x48e7); // MOVEM.L A7,-(A7)
        bus.put16(0x102, 0x0001);
        bus.put16(0x104, 0x4cdf); // MOVEM.L (A7)+,A7
        bus.put16(0x106, 0x8000);
        let mut cpu = M68000::default();
        cpu.reset(&mut bus);

        assert_ne!(cpu.step(&mut bus), 0);
        assert_eq!(cpu.a[7], 0x7ffc);
        assert_eq!(bus.read32(0x7ffc), 0x0000_8000);
        bus.write32(0x7ffc, 0x0012_3456);
        assert_ne!(cpu.step(&mut bus), 0);
        assert_eq!(cpu.a[7], 0x8000);
    }

    #[test]
    fn movem_word_load_sign_extends_registers() {
        let mut bus = TestBus::new();
        bus.boot(0x0100);
        bus.put16(0x100, 0x4c98); // MOVEM.W (A0)+,D0
        bus.put16(0x102, 0x0001);
        bus.put16(0x0200, 0x8001);
        let mut cpu = M68000::default();
        cpu.reset(&mut bus);
        cpu.a[0] = 0x0200;

        assert_ne!(cpu.step(&mut bus), 0);
        assert_eq!(cpu.d[0], 0xffff_8001);
        assert_eq!(cpu.a[0], 0x0202);
    }

    #[test]
    fn memory_shift_and_rotate_forms_execute_word_operands() {
        let mut bus = TestBus::new();
        bus.boot(0x0100);
        bus.put16(0x100, 0xe1d0); // ASL.W (A0)
        bus.put16(0x102, 0xe2d0); // LSR.W (A0)
        bus.put16(0x104, 0xe5d0); // ROXL.W (A0)
        bus.put16(0x106, 0xe6d0); // ROR.W (A0)
        bus.put16(0x0200, 0x8001);
        let mut cpu = M68000::default();
        cpu.reset(&mut bus);
        cpu.a[0] = 0x0200;

        assert_ne!(cpu.step(&mut bus), 0);
        assert_eq!(bus.read16(0x0200), 0x0002);
        assert!(cpu.flag(C));
        assert!(cpu.flag(X));
        assert!(cpu.flag(V));

        assert_ne!(cpu.step(&mut bus), 0);
        assert_eq!(bus.read16(0x0200), 0x0001);
        assert!(!cpu.flag(C));
        assert!(!cpu.flag(X));
        assert!(!cpu.flag(V));

        cpu.sr |= X;
        assert_ne!(cpu.step(&mut bus), 0);
        assert_eq!(bus.read16(0x0200), 0x0003);
        assert!(!cpu.flag(C));
        assert!(!cpu.flag(X));

        assert_ne!(cpu.step(&mut bus), 0);
        assert_eq!(bus.read16(0x0200), 0x8001);
        assert!(cpu.flag(C));
        assert!(!cpu.flag(X));
    }

    #[test]
    fn register_shift_updates_result_and_flags() {
        let mut bus = TestBus::new();
        bus.boot(0x0100);
        bus.put16(0x100, 0x7001);
        bus.put16(0x102, 0xe588); // LSL.L #2,D0
        let mut cpu = M68000::default();
        cpu.reset(&mut bus);
        cpu.step(&mut bus);
        assert_ne!(cpu.step(&mut bus), 0);
        assert_eq!(cpu.d[0], 4);
        assert!(!cpu.flag(Z));
    }
}
