use crate::bus::Bus8;

const FLAG_CARRY: u8 = 1 << 0;
const FLAG_ZERO: u8 = 1 << 1;
const FLAG_INTERRUPT: u8 = 1 << 2;
const FLAG_DECIMAL: u8 = 1 << 3;
const FLAG_BREAK: u8 = 1 << 4;
const FLAG_UNUSED: u8 = 1 << 5;
const FLAG_OVERFLOW: u8 = 1 << 6;
const FLAG_NEGATIVE: u8 = 1 << 7;
#[derive(Debug, Clone)]
pub struct Mos6502 {
    pub pc: u16,
    pub sp: u8,
    pub a: u8,
    pub x: u8,
    pub y: u8,
    pub p: u8,
    pub cycles: u64,
    pub stopped: bool,
    decimal_supported: bool,
}

impl Default for Mos6502 {
    fn default() -> Self {
        Self {
            pc: 0,
            sp: 0xfd,
            a: 0,
            x: 0,
            y: 0,
            p: FLAG_INTERRUPT | FLAG_UNUSED,
            cycles: 0,
            stopped: false,
            decimal_supported: true,
        }
    }
}
impl Mos6502 {
    pub fn reset<B: Bus8>(&mut self, bus: &mut B) {
        self.sp = 0xfd;
        self.p = FLAG_INTERRUPT | FLAG_UNUSED;
        self.pc = bus.read16(0xfffc);
        self.stopped = false;
        self.cycles = 7;
    }

    pub fn set_decimal_supported(&mut self, supported: bool) {
        self.decimal_supported = supported;
        if !supported {
            self.p &= !FLAG_DECIMAL;
        }
    }

    pub fn irq<B: Bus8>(&mut self, bus: &mut B) {
        if self.p & FLAG_INTERRUPT != 0 {
            return;
        }
        self.push16(bus, self.pc);
        self.push8(bus, (self.p & !FLAG_BREAK) | FLAG_UNUSED);
        self.p |= FLAG_INTERRUPT;
        self.pc = bus.read16(0xfffe);
        self.cycles += 7;
    }

    pub fn nmi<B: Bus8>(&mut self, bus: &mut B) {
        self.push16(bus, self.pc);
        self.push8(bus, (self.p & !FLAG_BREAK) | FLAG_UNUSED);
        self.p |= FLAG_INTERRUPT;
        self.pc = bus.read16(0xfffa);
        self.cycles += 7;
    }
    fn fetch8<B: Bus8>(&mut self, bus: &mut B) -> u8 {
        let value = bus.read8(self.pc);
        self.pc = self.pc.wrapping_add(1);
        value
    }
    fn fetch16<B: Bus8>(&mut self, bus: &mut B) -> u16 {
        let lo = self.fetch8(bus) as u16;
        let hi = self.fetch8(bus) as u16;
        lo | (hi << 8)
    }
    fn zp<B: Bus8>(&mut self, bus: &mut B) -> u16 {
        self.fetch8(bus) as u16
    }
    fn zpx<B: Bus8>(&mut self, bus: &mut B) -> u16 {
        self.fetch8(bus).wrapping_add(self.x) as u16
    }
    fn zpy<B: Bus8>(&mut self, bus: &mut B) -> u16 {
        self.fetch8(bus).wrapping_add(self.y) as u16
    }
    fn abs<B: Bus8>(&mut self, bus: &mut B) -> u16 {
        self.fetch16(bus)
    }
    fn absx<B: Bus8>(&mut self, bus: &mut B) -> (u16, bool) {
        let base = self.fetch16(bus);
        let addr = base.wrapping_add(self.x as u16);
        (addr, base & 0xff00 != addr & 0xff00)
    }
    fn absy<B: Bus8>(&mut self, bus: &mut B) -> (u16, bool) {
        let base = self.fetch16(bus);
        let addr = base.wrapping_add(self.y as u16);
        (addr, base & 0xff00 != addr & 0xff00)
    }
    fn indx<B: Bus8>(&mut self, bus: &mut B) -> u16 {
        let ptr = self.fetch8(bus).wrapping_add(self.x);
        let lo = bus.read8(ptr as u16) as u16;
        let hi = bus.read8(ptr.wrapping_add(1) as u16) as u16;
        lo | (hi << 8)
    }
    fn indy<B: Bus8>(&mut self, bus: &mut B) -> (u16, bool) {
        let ptr = self.fetch8(bus);
        let lo = bus.read8(ptr as u16) as u16;
        let hi = bus.read8(ptr.wrapping_add(1) as u16) as u16;
        let base = lo | (hi << 8);
        let addr = base.wrapping_add(self.y as u16);
        (addr, base & 0xff00 != addr & 0xff00)
    }
    fn indirect_bug<B: Bus8>(&mut self, bus: &mut B, pointer: u16) -> u16 {
        let lo = bus.read8(pointer) as u16;
        let hi_addr = (pointer & 0xff00) | pointer.wrapping_add(1) & 0x00ff;
        let hi = bus.read8(hi_addr) as u16;
        lo | (hi << 8)
    }
    fn push8<B: Bus8>(&mut self, bus: &mut B, value: u8) {
        bus.write8(0x0100 | self.sp as u16, value);
        self.sp = self.sp.wrapping_sub(1);
    }
    fn pop8<B: Bus8>(&mut self, bus: &mut B) -> u8 {
        self.sp = self.sp.wrapping_add(1);
        bus.read8(0x0100 | self.sp as u16)
    }
    fn push16<B: Bus8>(&mut self, bus: &mut B, value: u16) {
        self.push8(bus, (value >> 8) as u8);
        self.push8(bus, value as u8);
    }
    fn pop16<B: Bus8>(&mut self, bus: &mut B) -> u16 {
        let lo = self.pop8(bus) as u16;
        let hi = self.pop8(bus) as u16;
        lo | (hi << 8)
    }
    fn set_flag(&mut self, flag: u8, value: bool) {
        if value {
            self.p |= flag
        } else {
            self.p &= !flag
        }
    }
    fn set_zn(&mut self, value: u8) {
        self.set_flag(FLAG_ZERO, value == 0);
        self.set_flag(FLAG_NEGATIVE, value & 0x80 != 0);
    }
    fn compare(&mut self, lhs: u8, rhs: u8) {
        let result = lhs.wrapping_sub(rhs);
        self.set_flag(FLAG_CARRY, lhs >= rhs);
        self.set_zn(result);
    }
    fn adc(&mut self, value: u8) {
        let carry = u16::from(self.p & FLAG_CARRY != 0);
        let sum = self.a as u16 + value as u16 + carry;
        let binary_result = sum as u8;
        let overflow = (!(self.a ^ value) & (self.a ^ binary_result) & 0x80) != 0;
        if self.decimal_supported && self.p & FLAG_DECIMAL != 0 {
            let mut low = u16::from(self.a & 0x0f) + u16::from(value & 0x0f) + carry;
            if low > 9 {
                low += 6;
            }
            let mut high = u16::from(self.a >> 4) + u16::from(value >> 4) + u16::from(low > 0x0f);
            if high > 9 {
                high += 6;
            }
            self.a = (((high << 4) | (low & 0x0f)) & 0xff) as u8;
            self.set_flag(FLAG_CARRY, high > 0x0f);
            self.set_flag(FLAG_OVERFLOW, overflow);
            self.set_zn(self.a);
            return;
        }
        self.set_flag(FLAG_CARRY, sum > 0xff);
        self.set_flag(FLAG_OVERFLOW, overflow);
        self.a = binary_result;
        self.set_zn(self.a);
    }

    fn sbc(&mut self, value: u8) {
        if self.decimal_supported && self.p & FLAG_DECIMAL != 0 {
            let borrow = i16::from(self.p & FLAG_CARRY == 0);
            let binary = self.a as i16 - value as i16 - borrow;
            let binary_result = binary as u8;
            let overflow = ((self.a ^ binary_result) & (self.a ^ value) & 0x80) != 0;
            let mut low = i16::from(self.a & 0x0f) - i16::from(value & 0x0f) - borrow;
            let mut high = i16::from(self.a >> 4) - i16::from(value >> 4);
            if low < 0 {
                low -= 6;
                high -= 1;
            }
            if high < 0 {
                high -= 6;
            }
            self.a = (((high << 4) & 0xf0) | (low & 0x0f)) as u8;
            self.set_flag(FLAG_CARRY, binary >= 0);
            self.set_flag(FLAG_OVERFLOW, overflow);
            self.set_zn(self.a);
            return;
        }
        self.adc(value ^ 0xff);
    }
    fn branch<B: Bus8>(&mut self, bus: &mut B, condition: bool) -> u32 {
        let offset = self.fetch8(bus) as i8;
        if !condition {
            return 2;
        }
        let old = self.pc;
        self.pc = self.pc.wrapping_add_signed(offset as i16);
        3 + u32::from(old & 0xff00 != self.pc & 0xff00)
    }
    fn asl(&mut self, value: u8) -> u8 {
        self.set_flag(FLAG_CARRY, value & 0x80 != 0);
        let result = value << 1;
        self.set_zn(result);
        result
    }
    fn lsr(&mut self, value: u8) -> u8 {
        self.set_flag(FLAG_CARRY, value & 1 != 0);
        let result = value >> 1;
        self.set_zn(result);
        result
    }
    fn rol(&mut self, value: u8) -> u8 {
        let carry = u8::from(self.p & FLAG_CARRY != 0);
        self.set_flag(FLAG_CARRY, value & 0x80 != 0);
        let result = (value << 1) | carry;
        self.set_zn(result);
        result
    }
    fn ror(&mut self, value: u8) -> u8 {
        let carry = if self.p & FLAG_CARRY != 0 { 0x80 } else { 0 };
        self.set_flag(FLAG_CARRY, value & 1 != 0);
        let result = (value >> 1) | carry;
        self.set_zn(result);
        result
    }

    pub fn step<B: Bus8>(&mut self, bus: &mut B) -> u32 {
        if self.stopped {
            return 0;
        }
        let opcode = self.fetch8(bus);
        let used = self.execute(bus, opcode);
        self.cycles += used as u64;
        used
    }
    fn read_zp<B: Bus8>(&mut self, bus: &mut B) -> u8 {
        let a = self.zp(bus);
        bus.read8(a)
    }
    fn read_zpx<B: Bus8>(&mut self, bus: &mut B) -> u8 {
        let a = self.zpx(bus);
        bus.read8(a)
    }
    fn read_zpy<B: Bus8>(&mut self, bus: &mut B) -> u8 {
        let a = self.zpy(bus);
        bus.read8(a)
    }
    fn read_abs<B: Bus8>(&mut self, bus: &mut B) -> u8 {
        let a = self.abs(bus);
        bus.read8(a)
    }
    fn read_absx<B: Bus8>(&mut self, bus: &mut B) -> (u8, bool) {
        let (a, crossed) = self.absx(bus);
        (bus.read8(a), crossed)
    }
    fn read_absy<B: Bus8>(&mut self, bus: &mut B) -> (u8, bool) {
        let (a, crossed) = self.absy(bus);
        (bus.read8(a), crossed)
    }
    fn read_indx<B: Bus8>(&mut self, bus: &mut B) -> u8 {
        let a = self.indx(bus);
        bus.read8(a)
    }
    fn read_indy<B: Bus8>(&mut self, bus: &mut B) -> (u8, bool) {
        let (a, crossed) = self.indy(bus);
        (bus.read8(a), crossed)
    }

    fn unofficial_rmw_address<B: Bus8>(&mut self, bus: &mut B, opcode: u8) -> (u16, u32) {
        match opcode & 0x1f {
            0x03 => (self.indx(bus), 8),
            0x07 => (self.zp(bus), 5),
            0x0f => (self.abs(bus), 6),
            0x13 => (self.indy(bus).0, 8),
            0x17 => (self.zpx(bus), 6),
            0x1b => (self.absy(bus).0, 7),
            0x1f => (self.absx(bus).0, 7),
            _ => unreachable!("not an unofficial read-modify-write opcode"),
        }
    }

    fn lax(&mut self, value: u8) {
        self.a = value;
        self.x = value;
        self.set_zn(value);
    }

    fn execute_unofficial_rmw<B: Bus8>(&mut self, bus: &mut B, opcode: u8) -> u32 {
        let (address, cycles) = self.unofficial_rmw_address(bus, opcode);
        let value = bus.read8(address);
        let group = opcode & 0xe0;
        let result = match group {
            0x00 => self.asl(value),
            0x20 => self.rol(value),
            0x40 => self.lsr(value),
            0x60 => self.ror(value),
            0xc0 => value.wrapping_sub(1),
            0xe0 => value.wrapping_add(1),
            _ => unreachable!("not an unofficial read-modify-write group"),
        };
        bus.write8(address, result);
        match group {
            0x00 => {
                self.a |= result;
                self.set_zn(self.a);
            }
            0x20 => {
                self.a &= result;
                self.set_zn(self.a);
            }
            0x40 => {
                self.a ^= result;
                self.set_zn(self.a);
            }
            0x60 => self.adc(result),
            0xc0 => self.compare(self.a, result),
            0xe0 => self.sbc(result),
            _ => unreachable!(),
        }
        cycles
    }

    fn nop_absx<B: Bus8>(&mut self, bus: &mut B) -> u32 {
        let (_, crossed) = self.read_absx(bus);
        4 + u32::from(crossed)
    }

    fn execute<B: Bus8>(&mut self, bus: &mut B, opcode: u8) -> u32 {
        match opcode {
            0x00 => {
                self.pc = self.pc.wrapping_add(1);
                self.push16(bus, self.pc);
                self.push8(bus, self.p | FLAG_BREAK | FLAG_UNUSED);
                self.p |= FLAG_INTERRUPT;
                self.pc = bus.read16(0xfffe);
                7
            }
            0xea => 2,
            // Stable NMOS 6502 undocumented NOPs used by commercial software.
            0x1a | 0x3a | 0x5a | 0x7a | 0xda | 0xfa => 2,
            0x80 | 0x82 | 0x89 | 0xc2 | 0xe2 => {
                self.fetch8(bus);
                2
            }
            0x04 | 0x44 | 0x64 => {
                self.read_zp(bus);
                3
            }
            0x14 | 0x34 | 0x54 | 0x74 | 0xd4 | 0xf4 => {
                self.read_zpx(bus);
                4
            }
            0x0c => {
                self.read_abs(bus);
                4
            }
            0x1c | 0x3c | 0x5c | 0x7c | 0xdc | 0xfc => self.nop_absx(bus),

            // LAX: load A and X together.
            0xab => {
                let value = self.fetch8(bus);
                self.lax(value);
                2
            }
            0xa7 => {
                let value = self.read_zp(bus);
                self.lax(value);
                3
            }
            0xb7 => {
                let value = self.read_zpy(bus);
                self.lax(value);
                4
            }
            0xaf => {
                let value = self.read_abs(bus);
                self.lax(value);
                4
            }
            0xbf => {
                let (value, crossed) = self.read_absy(bus);
                self.lax(value);
                4 + u32::from(crossed)
            }
            0xa3 => {
                let value = self.read_indx(bus);
                self.lax(value);
                6
            }
            0xb3 => {
                let (value, crossed) = self.read_indy(bus);
                self.lax(value);
                5 + u32::from(crossed)
            }

            // SAX: store A & X.
            0x87 => {
                let address = self.zp(bus);
                bus.write8(address, self.a & self.x);
                3
            }
            0x97 => {
                let address = self.zpy(bus);
                bus.write8(address, self.a & self.x);
                4
            }
            0x8f => {
                let address = self.abs(bus);
                bus.write8(address, self.a & self.x);
                4
            }
            0x83 => {
                let address = self.indx(bus);
                bus.write8(address, self.a & self.x);
                6
            }

            // SLO/RLA/SRE/RRA/DCP/ISC read-modify-write families.
            0x03 | 0x07 | 0x0f | 0x13 | 0x17 | 0x1b | 0x1f | 0x23 | 0x27 | 0x2f | 0x33 | 0x37
            | 0x3b | 0x3f | 0x43 | 0x47 | 0x4f | 0x53 | 0x57 | 0x5b | 0x5f | 0x63 | 0x67 | 0x6f
            | 0x73 | 0x77 | 0x7b | 0x7f | 0xc3 | 0xc7 | 0xcf | 0xd3 | 0xd7 | 0xdb | 0xdf | 0xe3
            | 0xe7 | 0xef | 0xf3 | 0xf7 | 0xfb | 0xff => self.execute_unofficial_rmw(bus, opcode),

            // Immediate undocumented ALU forms with stable behavior.
            0x0b | 0x2b => {
                self.a &= self.fetch8(bus);
                self.set_zn(self.a);
                self.set_flag(FLAG_CARRY, self.a & 0x80 != 0);
                2
            }
            0x4b => {
                self.a &= self.fetch8(bus);
                self.a = self.lsr(self.a);
                2
            }
            0x6b => {
                let value = self.a & self.fetch8(bus);
                let carry_in = if self.p & FLAG_CARRY != 0 { 0x80 } else { 0 };
                self.a = (value >> 1) | carry_in;
                self.set_zn(self.a);
                self.set_flag(FLAG_CARRY, self.a & 0x40 != 0);
                self.set_flag(FLAG_OVERFLOW, ((self.a >> 6) ^ (self.a >> 5)) & 1 != 0);
                2
            }
            0xcb => {
                let value = self.fetch8(bus);
                let lhs = self.a & self.x;
                self.x = lhs.wrapping_sub(value);
                self.set_flag(FLAG_CARRY, lhs >= value);
                self.set_zn(self.x);
                2
            }
            0xeb => {
                let value = self.fetch8(bus);
                self.sbc(value);
                2
            }
            0xbb => {
                let (value, crossed) = self.read_absy(bus);
                let result = value & self.sp;
                self.a = result;
                self.x = result;
                self.sp = result;
                self.set_zn(result);
                4 + u32::from(crossed)
            }

            // Loads
            0xa9 => {
                self.a = self.fetch8(bus);
                self.set_zn(self.a);
                2
            }
            0xa5 => {
                self.a = self.read_zp(bus);
                self.set_zn(self.a);
                3
            }
            0xb5 => {
                self.a = self.read_zpx(bus);
                self.set_zn(self.a);
                4
            }
            0xad => {
                self.a = self.read_abs(bus);
                self.set_zn(self.a);
                4
            }
            0xbd => {
                let (v, c) = self.read_absx(bus);
                self.a = v;
                self.set_zn(v);
                4 + u32::from(c)
            }
            0xb9 => {
                let (v, c) = self.read_absy(bus);
                self.a = v;
                self.set_zn(v);
                4 + u32::from(c)
            }
            0xa1 => {
                self.a = self.read_indx(bus);
                self.set_zn(self.a);
                6
            }
            0xb1 => {
                let (v, c) = self.read_indy(bus);
                self.a = v;
                self.set_zn(v);
                5 + u32::from(c)
            }
            0xa2 => {
                self.x = self.fetch8(bus);
                self.set_zn(self.x);
                2
            }
            0xa6 => {
                self.x = self.read_zp(bus);
                self.set_zn(self.x);
                3
            }
            0xb6 => {
                self.x = self.read_zpy(bus);
                self.set_zn(self.x);
                4
            }
            0xae => {
                self.x = self.read_abs(bus);
                self.set_zn(self.x);
                4
            }
            0xbe => {
                let (v, c) = self.read_absy(bus);
                self.x = v;
                self.set_zn(v);
                4 + u32::from(c)
            }
            0xa0 => {
                self.y = self.fetch8(bus);
                self.set_zn(self.y);
                2
            }
            0xa4 => {
                self.y = self.read_zp(bus);
                self.set_zn(self.y);
                3
            }
            0xb4 => {
                self.y = self.read_zpx(bus);
                self.set_zn(self.y);
                4
            }
            0xac => {
                self.y = self.read_abs(bus);
                self.set_zn(self.y);
                4
            }
            0xbc => {
                let (v, c) = self.read_absx(bus);
                self.y = v;
                self.set_zn(v);
                4 + u32::from(c)
            }
            // Stores
            0x85 => {
                let a = self.zp(bus);
                bus.write8(a, self.a);
                3
            }
            0x95 => {
                let a = self.zpx(bus);
                bus.write8(a, self.a);
                4
            }
            0x8d => {
                let a = self.abs(bus);
                bus.write8(a, self.a);
                4
            }
            0x9d => {
                let (a, _) = self.absx(bus);
                bus.write8(a, self.a);
                5
            }
            0x99 => {
                let (a, _) = self.absy(bus);
                bus.write8(a, self.a);
                5
            }
            0x81 => {
                let a = self.indx(bus);
                bus.write8(a, self.a);
                6
            }
            0x91 => {
                let (a, _) = self.indy(bus);
                bus.write8(a, self.a);
                6
            }
            0x86 => {
                let a = self.zp(bus);
                bus.write8(a, self.x);
                3
            }
            0x96 => {
                let a = self.zpy(bus);
                bus.write8(a, self.x);
                4
            }
            0x8e => {
                let a = self.abs(bus);
                bus.write8(a, self.x);
                4
            }
            0x84 => {
                let a = self.zp(bus);
                bus.write8(a, self.y);
                3
            }
            0x94 => {
                let a = self.zpx(bus);
                bus.write8(a, self.y);
                4
            }
            0x8c => {
                let a = self.abs(bus);
                bus.write8(a, self.y);
                4
            }

            // Register transfers
            0xaa => {
                self.x = self.a;
                self.set_zn(self.x);
                2
            }
            0xa8 => {
                self.y = self.a;
                self.set_zn(self.y);
                2
            }
            0x8a => {
                self.a = self.x;
                self.set_zn(self.a);
                2
            }
            0x98 => {
                self.a = self.y;
                self.set_zn(self.a);
                2
            }
            0xba => {
                self.x = self.sp;
                self.set_zn(self.x);
                2
            }
            0x9a => {
                self.sp = self.x;
                2
            }
            // ADC / SBC
            0x69 => {
                let v = self.fetch8(bus);
                self.adc(v);
                2
            }
            0x65 => {
                let v = self.read_zp(bus);
                self.adc(v);
                3
            }
            0x75 => {
                let v = self.read_zpx(bus);
                self.adc(v);
                4
            }
            0x6d => {
                let v = self.read_abs(bus);
                self.adc(v);
                4
            }
            0x7d => {
                let (v, c) = self.read_absx(bus);
                self.adc(v);
                4 + u32::from(c)
            }
            0x79 => {
                let (v, c) = self.read_absy(bus);
                self.adc(v);
                4 + u32::from(c)
            }
            0x61 => {
                let v = self.read_indx(bus);
                self.adc(v);
                6
            }
            0x71 => {
                let (v, c) = self.read_indy(bus);
                self.adc(v);
                5 + u32::from(c)
            }
            0xe9 => {
                let v = self.fetch8(bus);
                self.sbc(v);
                2
            }
            0xe5 => {
                let v = self.read_zp(bus);
                self.sbc(v);
                3
            }
            0xf5 => {
                let v = self.read_zpx(bus);
                self.sbc(v);
                4
            }
            0xed => {
                let v = self.read_abs(bus);
                self.sbc(v);
                4
            }
            0xfd => {
                let (v, c) = self.read_absx(bus);
                self.sbc(v);
                4 + u32::from(c)
            }
            0xf9 => {
                let (v, c) = self.read_absy(bus);
                self.sbc(v);
                4 + u32::from(c)
            }
            0xe1 => {
                let v = self.read_indx(bus);
                self.sbc(v);
                6
            }
            0xf1 => {
                let (v, c) = self.read_indy(bus);
                self.sbc(v);
                5 + u32::from(c)
            }
            // AND
            0x29 => {
                self.a &= self.fetch8(bus);
                self.set_zn(self.a);
                2
            }
            0x25 => {
                self.a &= self.read_zp(bus);
                self.set_zn(self.a);
                3
            }
            0x35 => {
                self.a &= self.read_zpx(bus);
                self.set_zn(self.a);
                4
            }
            0x2d => {
                self.a &= self.read_abs(bus);
                self.set_zn(self.a);
                4
            }
            0x3d => {
                let (v, c) = self.read_absx(bus);
                self.a &= v;
                self.set_zn(self.a);
                4 + u32::from(c)
            }
            0x39 => {
                let (v, c) = self.read_absy(bus);
                self.a &= v;
                self.set_zn(self.a);
                4 + u32::from(c)
            }
            0x21 => {
                self.a &= self.read_indx(bus);
                self.set_zn(self.a);
                6
            }
            0x31 => {
                let (v, c) = self.read_indy(bus);
                self.a &= v;
                self.set_zn(self.a);
                5 + u32::from(c)
            }
            // ORA
            0x09 => {
                self.a |= self.fetch8(bus);
                self.set_zn(self.a);
                2
            }
            0x05 => {
                self.a |= self.read_zp(bus);
                self.set_zn(self.a);
                3
            }
            0x15 => {
                self.a |= self.read_zpx(bus);
                self.set_zn(self.a);
                4
            }
            0x0d => {
                self.a |= self.read_abs(bus);
                self.set_zn(self.a);
                4
            }
            0x1d => {
                let (v, c) = self.read_absx(bus);
                self.a |= v;
                self.set_zn(self.a);
                4 + u32::from(c)
            }
            0x19 => {
                let (v, c) = self.read_absy(bus);
                self.a |= v;
                self.set_zn(self.a);
                4 + u32::from(c)
            }
            0x01 => {
                self.a |= self.read_indx(bus);
                self.set_zn(self.a);
                6
            }
            0x11 => {
                let (v, c) = self.read_indy(bus);
                self.a |= v;
                self.set_zn(self.a);
                5 + u32::from(c)
            }
            // EOR
            0x49 => {
                self.a ^= self.fetch8(bus);
                self.set_zn(self.a);
                2
            }
            0x45 => {
                self.a ^= self.read_zp(bus);
                self.set_zn(self.a);
                3
            }
            0x55 => {
                self.a ^= self.read_zpx(bus);
                self.set_zn(self.a);
                4
            }
            0x4d => {
                self.a ^= self.read_abs(bus);
                self.set_zn(self.a);
                4
            }
            0x5d => {
                let (v, c) = self.read_absx(bus);
                self.a ^= v;
                self.set_zn(self.a);
                4 + u32::from(c)
            }
            0x59 => {
                let (v, c) = self.read_absy(bus);
                self.a ^= v;
                self.set_zn(self.a);
                4 + u32::from(c)
            }
            0x41 => {
                self.a ^= self.read_indx(bus);
                self.set_zn(self.a);
                6
            }
            0x51 => {
                let (v, c) = self.read_indy(bus);
                self.a ^= v;
                self.set_zn(self.a);
                5 + u32::from(c)
            }
            // Compare
            0xc9 => {
                let v = self.fetch8(bus);
                self.compare(self.a, v);
                2
            }
            0xc5 => {
                let v = self.read_zp(bus);
                self.compare(self.a, v);
                3
            }
            0xd5 => {
                let v = self.read_zpx(bus);
                self.compare(self.a, v);
                4
            }
            0xcd => {
                let v = self.read_abs(bus);
                self.compare(self.a, v);
                4
            }
            0xdd => {
                let (v, c) = self.read_absx(bus);
                self.compare(self.a, v);
                4 + u32::from(c)
            }
            0xd9 => {
                let (v, c) = self.read_absy(bus);
                self.compare(self.a, v);
                4 + u32::from(c)
            }
            0xc1 => {
                let v = self.read_indx(bus);
                self.compare(self.a, v);
                6
            }
            0xd1 => {
                let (v, c) = self.read_indy(bus);
                self.compare(self.a, v);
                5 + u32::from(c)
            }
            0xe0 => {
                let v = self.fetch8(bus);
                self.compare(self.x, v);
                2
            }
            0xe4 => {
                let v = self.read_zp(bus);
                self.compare(self.x, v);
                3
            }
            0xec => {
                let v = self.read_abs(bus);
                self.compare(self.x, v);
                4
            }
            0xc0 => {
                let v = self.fetch8(bus);
                self.compare(self.y, v);
                2
            }
            0xc4 => {
                let v = self.read_zp(bus);
                self.compare(self.y, v);
                3
            }
            0xcc => {
                let v = self.read_abs(bus);
                self.compare(self.y, v);
                4
            }
            // BIT
            0x24 => {
                let v = self.read_zp(bus);
                self.bit(v);
                3
            }
            0x2c => {
                let v = self.read_abs(bus);
                self.bit(v);
                4
            }

            // Increment/decrement registers
            0xe8 => {
                self.x = self.x.wrapping_add(1);
                self.set_zn(self.x);
                2
            }
            0xc8 => {
                self.y = self.y.wrapping_add(1);
                self.set_zn(self.y);
                2
            }
            0xca => {
                self.x = self.x.wrapping_sub(1);
                self.set_zn(self.x);
                2
            }
            0x88 => {
                self.y = self.y.wrapping_sub(1);
                self.set_zn(self.y);
                2
            }

            // Branches
            0x10 => self.branch(bus, self.p & FLAG_NEGATIVE == 0),
            0x30 => self.branch(bus, self.p & FLAG_NEGATIVE != 0),
            0x50 => self.branch(bus, self.p & FLAG_OVERFLOW == 0),
            0x70 => self.branch(bus, self.p & FLAG_OVERFLOW != 0),
            0x90 => self.branch(bus, self.p & FLAG_CARRY == 0),
            0xb0 => self.branch(bus, self.p & FLAG_CARRY != 0),
            0xd0 => self.branch(bus, self.p & FLAG_ZERO == 0),
            0xf0 => self.branch(bus, self.p & FLAG_ZERO != 0),
            // INC / DEC memory
            0xe6 => {
                let a = self.zp(bus);
                let v = bus.read8(a).wrapping_add(1);
                bus.write8(a, v);
                self.set_zn(v);
                5
            }
            0xf6 => {
                let a = self.zpx(bus);
                let v = bus.read8(a).wrapping_add(1);
                bus.write8(a, v);
                self.set_zn(v);
                6
            }
            0xee => {
                let a = self.abs(bus);
                let v = bus.read8(a).wrapping_add(1);
                bus.write8(a, v);
                self.set_zn(v);
                6
            }
            0xfe => {
                let (a, _) = self.absx(bus);
                let v = bus.read8(a).wrapping_add(1);
                bus.write8(a, v);
                self.set_zn(v);
                7
            }
            0xc6 => {
                let a = self.zp(bus);
                let v = bus.read8(a).wrapping_sub(1);
                bus.write8(a, v);
                self.set_zn(v);
                5
            }
            0xd6 => {
                let a = self.zpx(bus);
                let v = bus.read8(a).wrapping_sub(1);
                bus.write8(a, v);
                self.set_zn(v);
                6
            }
            0xce => {
                let a = self.abs(bus);
                let v = bus.read8(a).wrapping_sub(1);
                bus.write8(a, v);
                self.set_zn(v);
                6
            }
            0xde => {
                let (a, _) = self.absx(bus);
                let v = bus.read8(a).wrapping_sub(1);
                bus.write8(a, v);
                self.set_zn(v);
                7
            }

            // ASL / LSR
            0x0a => {
                self.a = self.asl(self.a);
                2
            }
            0x06 => {
                let a = self.zp(bus);
                let v = self.asl(bus.read8(a));
                bus.write8(a, v);
                5
            }
            0x16 => {
                let a = self.zpx(bus);
                let v = self.asl(bus.read8(a));
                bus.write8(a, v);
                6
            }
            0x0e => {
                let a = self.abs(bus);
                let v = self.asl(bus.read8(a));
                bus.write8(a, v);
                6
            }
            0x1e => {
                let (a, _) = self.absx(bus);
                let v = self.asl(bus.read8(a));
                bus.write8(a, v);
                7
            }
            0x4a => {
                self.a = self.lsr(self.a);
                2
            }
            0x46 => {
                let a = self.zp(bus);
                let v = self.lsr(bus.read8(a));
                bus.write8(a, v);
                5
            }
            0x56 => {
                let a = self.zpx(bus);
                let v = self.lsr(bus.read8(a));
                bus.write8(a, v);
                6
            }
            0x4e => {
                let a = self.abs(bus);
                let v = self.lsr(bus.read8(a));
                bus.write8(a, v);
                6
            }
            0x5e => {
                let (a, _) = self.absx(bus);
                let v = self.lsr(bus.read8(a));
                bus.write8(a, v);
                7
            }
            // ROL / ROR
            0x2a => {
                self.a = self.rol(self.a);
                2
            }
            0x26 => {
                let a = self.zp(bus);
                let v = self.rol(bus.read8(a));
                bus.write8(a, v);
                5
            }
            0x36 => {
                let a = self.zpx(bus);
                let v = self.rol(bus.read8(a));
                bus.write8(a, v);
                6
            }
            0x2e => {
                let a = self.abs(bus);
                let v = self.rol(bus.read8(a));
                bus.write8(a, v);
                6
            }
            0x3e => {
                let (a, _) = self.absx(bus);
                let v = self.rol(bus.read8(a));
                bus.write8(a, v);
                7
            }
            0x6a => {
                self.a = self.ror(self.a);
                2
            }
            0x66 => {
                let a = self.zp(bus);
                let v = self.ror(bus.read8(a));
                bus.write8(a, v);
                5
            }
            0x76 => {
                let a = self.zpx(bus);
                let v = self.ror(bus.read8(a));
                bus.write8(a, v);
                6
            }
            0x6e => {
                let a = self.abs(bus);
                let v = self.ror(bus.read8(a));
                bus.write8(a, v);
                6
            }
            0x7e => {
                let (a, _) = self.absx(bus);
                let v = self.ror(bus.read8(a));
                bus.write8(a, v);
                7
            }

            // Jumps / subroutines
            0x4c => {
                self.pc = self.fetch16(bus);
                3
            }
            0x6c => {
                let p = self.fetch16(bus);
                self.pc = self.indirect_bug(bus, p);
                5
            }
            0x20 => {
                let target = self.fetch16(bus);
                self.push16(bus, self.pc.wrapping_sub(1));
                self.pc = target;
                6
            }
            0x60 => {
                self.pc = self.pop16(bus).wrapping_add(1);
                6
            }
            0x40 => {
                self.p = (self.pop8(bus) & !FLAG_BREAK) | FLAG_UNUSED;
                self.pc = self.pop16(bus);
                6
            }
            // Stack
            0x48 => {
                self.push8(bus, self.a);
                3
            }
            0x68 => {
                self.a = self.pop8(bus);
                self.set_zn(self.a);
                4
            }
            0x08 => {
                self.push8(bus, self.p | FLAG_BREAK | FLAG_UNUSED);
                3
            }
            0x28 => {
                self.p = (self.pop8(bus) & !FLAG_BREAK) | FLAG_UNUSED;
                4
            }

            // Processor flags
            0x18 => {
                self.p &= !FLAG_CARRY;
                2
            }
            0x38 => {
                self.p |= FLAG_CARRY;
                2
            }
            0x58 => {
                self.p &= !FLAG_INTERRUPT;
                2
            }
            0x78 => {
                self.p |= FLAG_INTERRUPT;
                2
            }
            0xb8 => {
                self.p &= !FLAG_OVERFLOW;
                2
            }
            0xd8 => {
                self.p &= !FLAG_DECIMAL;
                2
            }
            0xf8 => {
                self.p |= FLAG_DECIMAL;
                2
            }

            _ => {
                self.stopped = true;
                0
            }
        }
    }

    fn bit(&mut self, value: u8) {
        self.set_flag(FLAG_ZERO, self.a & value == 0);
        self.set_flag(FLAG_OVERFLOW, value & 0x40 != 0);
        self.set_flag(FLAG_NEGATIVE, value & 0x80 != 0);
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use crate::bus::Ram64k;

    #[test]
    fn executes_arithmetic_branch_and_store() {
        let mut bus = Ram64k::new();
        bus.load(
            0x8000,
            &[
                0xa9, 0x10, 0x69, 0x05, 0x8d, 0x00, 0x02, 0xa2, 0x02, 0xca, 0xd0, 0xfd,
            ],
        );
        bus.bytes_mut()[0xfffc] = 0x00;
        bus.bytes_mut()[0xfffd] = 0x80;
        let mut cpu = Mos6502::default();
        cpu.reset(&mut bus);
        for _ in 0..7 {
            cpu.step(&mut bus);
        }
        assert_eq!(bus.bytes()[0x0200], 0x15);
        assert_eq!(cpu.x, 0);
        assert!(cpu.p & FLAG_ZERO != 0);
    }

    #[test]
    fn jsr_and_rts_restore_program_counter() {
        let mut bus = Ram64k::new();
        bus.load(
            0x8000,
            &[0x20, 0x06, 0x80, 0xa9, 0x2a, 0xea, 0xa9, 0x11, 0x60],
        );
        bus.bytes_mut()[0xfffc] = 0x00;
        bus.bytes_mut()[0xfffd] = 0x80;
        let mut cpu = Mos6502::default();
        cpu.reset(&mut bus);
        cpu.step(&mut bus);
        cpu.step(&mut bus);
        cpu.step(&mut bus);
        cpu.step(&mut bus);
        assert_eq!(cpu.a, 0x2a);
        assert_eq!(cpu.pc, 0x8005);
    }
    #[test]
    fn stable_undocumented_lax_sax_dcp_and_isc_execute() {
        let mut bus = Ram64k::new();
        bus.load(
            0x8000,
            &[
                0xa9, 0x3c, 0xa2, 0x0f, 0x87, 0x10, 0xa7, 0x10, 0xc7, 0x10, 0xe7, 0x10,
            ],
        );
        bus.bytes_mut()[0xfffc] = 0x00;
        bus.bytes_mut()[0xfffd] = 0x80;
        let mut cpu = Mos6502::default();
        cpu.reset(&mut bus);
        for _ in 0..6 {
            assert_ne!(cpu.step(&mut bus), 0);
        }
        assert_eq!(bus.bytes()[0x0010], 0x0c);
        assert_eq!(cpu.a, 0);
        assert_eq!(cpu.x, 0x0c);
        assert_ne!(cpu.p & FLAG_ZERO, 0);
        assert_ne!(cpu.p & FLAG_CARRY, 0);
    }

    #[test]
    fn stable_undocumented_rmw_and_nops_preserve_execution() {
        let mut bus = Ram64k::new();
        bus.load(
            0x8000,
            &[
                0xa9, 0x01, 0x07, 0x20, 0x47, 0x20, 0x67, 0x20, 0x80, 0xaa, 0x1a,
            ],
        );
        bus.bytes_mut()[0x0020] = 0x81;
        bus.bytes_mut()[0xfffc] = 0x00;
        bus.bytes_mut()[0xfffd] = 0x80;
        let mut cpu = Mos6502::default();
        cpu.reset(&mut bus);
        for _ in 0..6 {
            assert_ne!(cpu.step(&mut bus), 0);
        }
        assert_eq!(bus.bytes()[0x0020], 0x00);
        assert_eq!(cpu.a, 0x03);
        assert_eq!(cpu.pc, 0x800b);
        assert!(!cpu.stopped);
    }
    #[test]
    fn nmos_decimal_adc_and_sbc_are_available_but_can_be_disabled() {
        let mut bus = Ram64k::new();
        bus.load(
            0x8000,
            &[
                0xf8, 0x18, 0xa9, 0x45, 0x69, 0x55, 0x38, 0xa9, 0x50, 0xe9, 0x01,
            ],
        );
        bus.bytes_mut()[0xfffc] = 0x00;
        bus.bytes_mut()[0xfffd] = 0x80;
        let mut cpu = Mos6502::default();
        cpu.reset(&mut bus);
        for _ in 0..4 {
            cpu.step(&mut bus);
        }
        assert_eq!(cpu.a, 0x00);
        assert_ne!(cpu.p & FLAG_CARRY, 0);
        for _ in 0..3 {
            cpu.step(&mut bus);
        }
        assert_eq!(cpu.a, 0x49);
        assert_ne!(cpu.p & FLAG_CARRY, 0);

        let mut binary = Mos6502::default();
        binary.set_decimal_supported(false);
        binary.reset(&mut bus);
        for _ in 0..4 {
            binary.step(&mut bus);
        }
        assert_eq!(binary.a, 0x9a);
        assert_eq!(binary.p & FLAG_CARRY, 0);
        assert_ne!(binary.p & FLAG_DECIMAL, 0);
    }
}
