use crate::state::{StateReader, StateWriter};

pub trait Bus68000: Send {
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
        }
    }
}

impl M68000 {
    pub fn reset<B: Bus68000>(&mut self, bus: &mut B) {
        *self = Self::default();
        self.a[7] = bus.read32(0) & 0x00ff_ffff;
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
        self.push32(bus, self.pc);
        self.push16(bus, self.sr);
        self.sr |= S;
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
        self.pc = input.u32()? & 0x00ff_ffff;
        self.sr = input.u16()?;
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
                self.sr = self.pop16(bus);
                self.pc = Self::mask_address(self.pop32(bus));
                return 20;
            }
            0x4e72 => {
                if self.sr & S == 0 {
                    return self.exception(bus, 8);
                }
                self.sr = self.fetch16(bus);
                self.stopped = true;
                return 4;
            }
            0x4e70 => return 132,
            _ => {}
        }
        if opcode & 0xfff0 == 0x4e40 {
            return self.exception(bus, 32 + (opcode & 0x0f) as u8);
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
        if opcode & 0xff00 == 0x4200 {
            return self.clear_instruction(bus, opcode);
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
        self.sr |= S;
        self.push32(bus, self.pc);
        self.push16(bus, old_sr);
        self.pc = bus.read32(u32::from(vector) * 4) & 0x00ff_ffff;
        34
    }

    fn clear_instruction<B: Bus68000>(&mut self, bus: &mut B, opcode: u16) -> u32 {
        let Some(size) = Self::size_from_field((opcode >> 6) & 3) else {
            return 0;
        };
        let Some(ea) =
            self.resolve_ea(bus, (opcode >> 3) & 7, usize::from(opcode & 7), size, false)
        else {
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
        let Some(ea) =
            self.resolve_ea(bus, (opcode >> 3) & 7, usize::from(opcode & 7), size, false)
        else {
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
        let Some(ea) =
            self.resolve_ea(bus, (opcode >> 3) & 7, usize::from(opcode & 7), size, false)
        else {
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
        while count != 0 {
            count -= 1;
            if left {
                carry = value & sign != 0;
                value = (value << 1) & mask;
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
        self.set_flag(V, false);
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
