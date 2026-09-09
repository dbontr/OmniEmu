pub trait Z80Bus: Send {
    fn mem_read(&mut self, address: u16) -> u8;
    fn mem_write(&mut self, address: u16, value: u8);
    fn io_read(&mut self, _port: u16) -> u8 {
        0xff
    }
    fn io_write(&mut self, _port: u16, _value: u8) {}
}

const S: u8 = 0x80;
const Z: u8 = 0x40;
const Y: u8 = 0x20;
const H: u8 = 0x10;
const X: u8 = 0x08;
const PV: u8 = 0x04;
const N: u8 = 0x02;
const C: u8 = 0x01;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum IndexMode {
    Ix,
    Iy,
}

#[derive(Debug, Clone)]
pub struct Z80 {
    pub a: u8,
    pub f: u8,
    pub b: u8,
    pub c: u8,
    pub d: u8,
    pub e: u8,
    pub h: u8,
    pub l: u8,
    pub a2: u8,
    pub f2: u8,
    pub b2: u8,
    pub c2: u8,
    pub d2: u8,
    pub e2: u8,
    pub h2: u8,
    pub l2: u8,
    pub ix: u16,
    pub iy: u16,
    pub sp: u16,
    pub pc: u16,
    pub i: u8,
    pub r: u8,
    pub iff1: bool,
    pub iff2: bool,
    pub interrupt_mode: u8,
    pub halted: bool,
    pub cycles: u64,
    ei_delay: u8,
}

impl Default for Z80 {
    fn default() -> Self {
        Self {
            a: 0xff,
            f: 0xff,
            b: 0,
            c: 0,
            d: 0,
            e: 0,
            h: 0,
            l: 0,
            a2: 0,
            f2: 0,
            b2: 0,
            c2: 0,
            d2: 0,
            e2: 0,
            h2: 0,
            l2: 0,
            ix: 0,
            iy: 0,
            sp: 0xffff,
            pc: 0,
            i: 0,
            r: 0,
            iff1: false,
            iff2: false,
            interrupt_mode: 0,
            halted: false,
            cycles: 0,
            ei_delay: 0,
        }
    }
}

impl Z80 {
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    pub fn step<B: Z80Bus>(&mut self, bus: &mut B) -> u32 {
        if self.halted {
            self.bump_r();
            self.cycles = self.cycles.wrapping_add(4);
            return 4;
        }
        let opcode = self.fetch_opcode(bus);
        let used = match opcode {
            0xcb => {
                let op = self.fetch_opcode(bus);
                self.execute_cb(bus, op, None, None)
            }
            0xed => {
                let op = self.fetch_opcode(bus);
                self.execute_ed(bus, op)
            }
            0xdd => self.execute_index_prefix(bus, IndexMode::Ix),
            0xfd => self.execute_index_prefix(bus, IndexMode::Iy),
            _ => self.execute_base(bus, opcode),
        };
        if self.ei_delay > 0 {
            self.ei_delay -= 1;
        }
        self.cycles = self.cycles.wrapping_add(used as u64);
        used
    }

    pub fn irq<B: Z80Bus>(&mut self, bus: &mut B, vector: u8) -> u32 {
        if !self.iff1 || self.ei_delay != 0 {
            return 0;
        }
        self.halted = false;
        self.iff1 = false;
        self.iff2 = false;
        self.bump_r();
        let used = match self.interrupt_mode {
            2 => {
                self.push16(bus, self.pc);
                let table = (u16::from(self.i) << 8) | u16::from(vector);
                self.pc = self.read16(bus, table);
                19
            }
            _ => {
                self.push16(bus, self.pc);
                self.pc = 0x0038;
                13
            }
        };
        self.cycles = self.cycles.wrapping_add(used as u64);
        used
    }

    pub fn nmi<B: Z80Bus>(&mut self, bus: &mut B) -> u32 {
        self.halted = false;
        self.iff2 = self.iff1;
        self.iff1 = false;
        self.push16(bus, self.pc);
        self.pc = 0x0066;
        self.bump_r();
        self.cycles = self.cycles.wrapping_add(11);
        11
    }

    fn bump_r(&mut self) {
        self.r = (self.r & 0x80) | (self.r.wrapping_add(1) & 0x7f);
    }
    fn fetch_opcode<B: Z80Bus>(&mut self, bus: &mut B) -> u8 {
        let value = bus.mem_read(self.pc);
        self.pc = self.pc.wrapping_add(1);
        self.bump_r();
        value
    }
    fn fetch8<B: Z80Bus>(&mut self, bus: &mut B) -> u8 {
        let value = bus.mem_read(self.pc);
        self.pc = self.pc.wrapping_add(1);
        value
    }
    fn fetch16<B: Z80Bus>(&mut self, bus: &mut B) -> u16 {
        let lo = self.fetch8(bus);
        let hi = self.fetch8(bus);
        u16::from_le_bytes([lo, hi])
    }
    fn read16<B: Z80Bus>(&mut self, bus: &mut B, address: u16) -> u16 {
        u16::from_le_bytes([bus.mem_read(address), bus.mem_read(address.wrapping_add(1))])
    }
    fn write16<B: Z80Bus>(&mut self, bus: &mut B, address: u16, value: u16) {
        let [lo, hi] = value.to_le_bytes();
        bus.mem_write(address, lo);
        bus.mem_write(address.wrapping_add(1), hi);
    }
    fn push16<B: Z80Bus>(&mut self, bus: &mut B, value: u16) {
        let [lo, hi] = value.to_le_bytes();
        self.sp = self.sp.wrapping_sub(1);
        bus.mem_write(self.sp, hi);
        self.sp = self.sp.wrapping_sub(1);
        bus.mem_write(self.sp, lo);
    }
    fn pop16<B: Z80Bus>(&mut self, bus: &mut B) -> u16 {
        let lo = bus.mem_read(self.sp);
        self.sp = self.sp.wrapping_add(1);
        let hi = bus.mem_read(self.sp);
        self.sp = self.sp.wrapping_add(1);
        u16::from_le_bytes([lo, hi])
    }
    fn af(&self) -> u16 {
        u16::from_be_bytes([self.a, self.f])
    }
    fn bc(&self) -> u16 {
        u16::from_be_bytes([self.b, self.c])
    }
    fn de(&self) -> u16 {
        u16::from_be_bytes([self.d, self.e])
    }
    fn hl(&self) -> u16 {
        u16::from_be_bytes([self.h, self.l])
    }
    fn set_af(&mut self, value: u16) {
        let [a, f] = value.to_be_bytes();
        self.a = a;
        self.f = f;
    }
    fn set_bc(&mut self, value: u16) {
        let [b, c] = value.to_be_bytes();
        self.b = b;
        self.c = c;
    }
    fn set_de(&mut self, value: u16) {
        let [d, e] = value.to_be_bytes();
        self.d = d;
        self.e = e;
    }
    fn set_hl(&mut self, value: u16) {
        let [h, l] = value.to_be_bytes();
        self.h = h;
        self.l = l;
    }

    fn pair(&self, p: u8) -> u16 {
        match p & 3 {
            0 => self.bc(),
            1 => self.de(),
            2 => self.hl(),
            _ => self.sp,
        }
    }
    fn set_pair(&mut self, p: u8, value: u16) {
        match p & 3 {
            0 => self.set_bc(value),
            1 => self.set_de(value),
            2 => self.set_hl(value),
            _ => self.sp = value,
        }
    }
    fn pair2(&self, p: u8) -> u16 {
        match p & 3 {
            0 => self.bc(),
            1 => self.de(),
            2 => self.hl(),
            _ => self.af(),
        }
    }
    fn set_pair2(&mut self, p: u8, value: u16) {
        match p & 3 {
            0 => self.set_bc(value),
            1 => self.set_de(value),
            2 => self.set_hl(value),
            _ => self.set_af(value),
        }
    }
    fn condition(&self, y: u8) -> bool {
        match y & 7 {
            0 => self.f & Z == 0,
            1 => self.f & Z != 0,
            2 => self.f & C == 0,
            3 => self.f & C != 0,
            4 => self.f & PV == 0,
            5 => self.f & PV != 0,
            6 => self.f & S == 0,
            _ => self.f & S != 0,
        }
    }

    fn read_reg<B: Z80Bus>(&mut self, bus: &mut B, r: u8) -> u8 {
        match r & 7 {
            0 => self.b,
            1 => self.c,
            2 => self.d,
            3 => self.e,
            4 => self.h,
            5 => self.l,
            6 => bus.mem_read(self.hl()),
            _ => self.a,
        }
    }
    fn write_reg<B: Z80Bus>(&mut self, bus: &mut B, r: u8, value: u8) {
        match r & 7 {
            0 => self.b = value,
            1 => self.c = value,
            2 => self.d = value,
            3 => self.e = value,
            4 => self.h = value,
            5 => self.l = value,
            6 => bus.mem_write(self.hl(), value),
            _ => self.a = value,
        }
    }

    fn parity(value: u8) -> bool {
        value.count_ones().is_multiple_of(2)
    }
    fn sz53(value: u8) -> u8 {
        (value & (S | Y | X)) | if value == 0 { Z } else { 0 }
    }
    fn sz53p(value: u8) -> u8 {
        Self::sz53(value) | if Self::parity(value) { PV } else { 0 }
    }

    fn add8(&mut self, value: u8, carry: bool) {
        let ci = u16::from(carry && self.f & C != 0);
        let a = self.a;
        let sum = u16::from(a) + u16::from(value) + ci;
        let result = sum as u8;
        self.f = Self::sz53(result)
            | if ((a ^ value ^ result) & 0x10) != 0 {
                H
            } else {
                0
            }
            | if (!(a ^ value) & (a ^ result) & 0x80) != 0 {
                PV
            } else {
                0
            }
            | if sum > 0xff { C } else { 0 };
        self.a = result;
    }
    fn sub8(&mut self, value: u8, carry: bool) {
        let ci = i16::from(carry && self.f & C != 0);
        let a = self.a;
        let diff = i16::from(a) - i16::from(value) - ci;
        let result = diff as u8;
        self.f = Self::sz53(result)
            | N
            | if ((a ^ value ^ result) & 0x10) != 0 {
                H
            } else {
                0
            }
            | if ((a ^ value) & (a ^ result) & 0x80) != 0 {
                PV
            } else {
                0
            }
            | if diff < 0 { C } else { 0 };
        self.a = result;
    }
    fn cp8(&mut self, value: u8) {
        let a = self.a;
        let diff = i16::from(a) - i16::from(value);
        let result = diff as u8;
        self.f = (result & S)
            | if result == 0 { Z } else { 0 }
            | (value & (Y | X))
            | N
            | if ((a ^ value ^ result) & 0x10) != 0 {
                H
            } else {
                0
            }
            | if ((a ^ value) & (a ^ result) & 0x80) != 0 {
                PV
            } else {
                0
            }
            | if diff < 0 { C } else { 0 };
    }
    fn logic8(&mut self, op: u8, value: u8) {
        self.a = match op {
            4 => self.a & value,
            5 => self.a ^ value,
            _ => self.a | value,
        };
        self.f = Self::sz53p(self.a) | if op == 4 { H } else { 0 };
    }
    fn inc8(&mut self, value: u8) -> u8 {
        let result = value.wrapping_add(1);
        let carry = self.f & C;
        self.f = carry
            | Self::sz53(result)
            | if value & 0x0f == 0x0f { H } else { 0 }
            | if value == 0x7f { PV } else { 0 };
        result
    }
    fn dec8(&mut self, value: u8) -> u8 {
        let result = value.wrapping_sub(1);
        let carry = self.f & C;
        self.f = carry
            | Self::sz53(result)
            | N
            | if value & 0x0f == 0 { H } else { 0 }
            | if value == 0x80 { PV } else { 0 };
        result
    }
    fn alu(&mut self, op: u8, value: u8) {
        match op & 7 {
            0 => self.add8(value, false),
            1 => self.add8(value, true),
            2 => self.sub8(value, false),
            3 => self.sub8(value, true),
            4..=6 => self.logic8(op & 7, value),
            _ => self.cp8(value),
        }
    }

    fn add_hl(&mut self, value: u16) {
        let hl = self.hl();
        let sum = u32::from(hl) + u32::from(value);
        let result = sum as u16;
        self.f = (self.f & (S | Z | PV))
            | ((result >> 8) as u8 & (Y | X))
            | if ((hl ^ value ^ result) & 0x1000) != 0 {
                H
            } else {
                0
            }
            | if sum > 0xffff { C } else { 0 };
        self.set_hl(result);
    }
    fn adc_sbc_hl(&mut self, value: u16, subtract: bool) {
        let hl = self.hl();
        let carry = u32::from(self.f & C != 0);
        let result = if subtract {
            u32::from(hl).wrapping_sub(u32::from(value) + carry)
        } else {
            u32::from(hl) + u32::from(value) + carry
        } as u16;
        let carry_out = if subtract {
            u32::from(hl) < u32::from(value) + carry
        } else {
            u32::from(hl) + u32::from(value) + carry > 0xffff
        };
        let overflow = if subtract {
            ((hl ^ value) & (hl ^ result) & 0x8000) != 0
        } else {
            (!(hl ^ value) & (hl ^ result) & 0x8000) != 0
        };
        self.f = ((result >> 8) as u8 & (S | Y | X))
            | if result == 0 { Z } else { 0 }
            | if ((hl ^ value ^ result) & 0x1000) != 0 {
                H
            } else {
                0
            }
            | if overflow { PV } else { 0 }
            | if subtract { N } else { 0 }
            | if carry_out { C } else { 0 };
        self.set_hl(result);
    }

    fn execute_base<B: Z80Bus>(&mut self, bus: &mut B, opcode: u8) -> u32 {
        let x = opcode >> 6;
        let y = (opcode >> 3) & 7;
        let z = opcode & 7;
        let p = y >> 1;
        let q = y & 1;
        match x {
            0 => match z {
                0 => match y {
                    0 => 4,
                    1 => {
                        std::mem::swap(&mut self.a, &mut self.a2);
                        std::mem::swap(&mut self.f, &mut self.f2);
                        4
                    }
                    2 => {
                        let d = self.fetch8(bus) as i8;
                        self.b = self.b.wrapping_sub(1);
                        if self.b != 0 {
                            self.pc = self.pc.wrapping_add_signed(i16::from(d));
                            13
                        } else {
                            8
                        }
                    }
                    3 => {
                        let d = self.fetch8(bus) as i8;
                        self.pc = self.pc.wrapping_add_signed(i16::from(d));
                        12
                    }
                    _ => {
                        let d = self.fetch8(bus) as i8;
                        if self.condition(y - 4) {
                            self.pc = self.pc.wrapping_add_signed(i16::from(d));
                            12
                        } else {
                            7
                        }
                    }
                },
                1 => {
                    if q == 0 {
                        let value = self.fetch16(bus);
                        self.set_pair(p, value);
                        10
                    } else {
                        let value = self.pair(p);
                        self.add_hl(value);
                        11
                    }
                }
                2 => {
                    if q == 0 {
                        match p {
                            0 => bus.mem_write(self.bc(), self.a),
                            1 => bus.mem_write(self.de(), self.a),
                            2 => {
                                let a = self.fetch16(bus);
                                self.write16(bus, a, self.hl());
                            }
                            _ => {
                                let a = self.fetch16(bus);
                                bus.mem_write(a, self.a);
                            }
                        }
                    } else {
                        match p {
                            0 => self.a = bus.mem_read(self.bc()),
                            1 => self.a = bus.mem_read(self.de()),
                            2 => {
                                let a = self.fetch16(bus);
                                let v = self.read16(bus, a);
                                self.set_hl(v);
                            }
                            _ => {
                                let a = self.fetch16(bus);
                                self.a = bus.mem_read(a);
                            }
                        }
                    }
                    match p {
                        0 | 1 => 7,
                        2 => 16,
                        _ => 13,
                    }
                }
                3 => {
                    let value = if q == 0 {
                        self.pair(p).wrapping_add(1)
                    } else {
                        self.pair(p).wrapping_sub(1)
                    };
                    self.set_pair(p, value);
                    6
                }
                4 => {
                    if y == 6 {
                        let a = self.hl();
                        let v = bus.mem_read(a);
                        let r = self.inc8(v);
                        bus.mem_write(a, r);
                        11
                    } else {
                        let v = self.read_reg(bus, y);
                        let r = self.inc8(v);
                        self.write_reg(bus, y, r);
                        4
                    }
                }
                5 => {
                    if y == 6 {
                        let a = self.hl();
                        let v = bus.mem_read(a);
                        let r = self.dec8(v);
                        bus.mem_write(a, r);
                        11
                    } else {
                        let v = self.read_reg(bus, y);
                        let r = self.dec8(v);
                        self.write_reg(bus, y, r);
                        4
                    }
                }
                6 => {
                    let v = self.fetch8(bus);
                    self.write_reg(bus, y, v);
                    if y == 6 {
                        10
                    } else {
                        7
                    }
                }
                _ => self.execute_misc(y),
            },
            1 => {
                if y == 6 && z == 6 {
                    self.halted = true;
                    4
                } else {
                    let v = self.read_reg(bus, z);
                    self.write_reg(bus, y, v);
                    if y == 6 || z == 6 {
                        7
                    } else {
                        4
                    }
                }
            }
            2 => {
                let value = self.read_reg(bus, z);
                self.alu(y, value);
                if z == 6 {
                    7
                } else {
                    4
                }
            }
            _ => match z {
                0 => {
                    if self.condition(y) {
                        self.pc = self.pop16(bus);
                        11
                    } else {
                        5
                    }
                }
                1 => {
                    if q == 0 {
                        let value = self.pop16(bus);
                        self.set_pair2(p, value);
                        10
                    } else {
                        match p {
                            0 => {
                                self.pc = self.pop16(bus);
                                10
                            }
                            1 => {
                                std::mem::swap(&mut self.b, &mut self.b2);
                                std::mem::swap(&mut self.c, &mut self.c2);
                                std::mem::swap(&mut self.d, &mut self.d2);
                                std::mem::swap(&mut self.e, &mut self.e2);
                                std::mem::swap(&mut self.h, &mut self.h2);
                                std::mem::swap(&mut self.l, &mut self.l2);
                                4
                            }
                            2 => {
                                self.pc = self.hl();
                                4
                            }
                            _ => {
                                self.sp = self.hl();
                                6
                            }
                        }
                    }
                }
                2 => {
                    let address = self.fetch16(bus);
                    if self.condition(y) {
                        self.pc = address;
                    }
                    10
                }
                3 => self.execute_z3(bus, y),
                4 => {
                    let address = self.fetch16(bus);
                    if self.condition(y) {
                        self.push16(bus, self.pc);
                        self.pc = address;
                        17
                    } else {
                        10
                    }
                }
                5 => {
                    if q == 0 {
                        let value = self.pair2(p);
                        self.push16(bus, value);
                        11
                    } else {
                        match p {
                            0 => {
                                let address = self.fetch16(bus);
                                self.push16(bus, self.pc);
                                self.pc = address;
                                17
                            }
                            _ => 4,
                        }
                    }
                }
                6 => {
                    let value = self.fetch8(bus);
                    self.alu(y, value);
                    7
                }
                _ => {
                    self.push16(bus, self.pc);
                    self.pc = u16::from(y) * 8;
                    11
                }
            },
        }
    }

    fn execute_z3<B: Z80Bus>(&mut self, bus: &mut B, y: u8) -> u32 {
        match y {
            0 => {
                let address = self.fetch16(bus);
                self.pc = address;
                10
            }
            1 => {
                let op = self.fetch_opcode(bus);
                4 + self.execute_cb(bus, op, None, None)
            }
            2 => {
                let low = self.fetch8(bus);
                let port = (u16::from(self.a) << 8) | u16::from(low);
                bus.io_write(port, self.a);
                11
            }
            3 => {
                let low = self.fetch8(bus);
                let port = (u16::from(self.a) << 8) | u16::from(low);
                self.a = bus.io_read(port);
                11
            }
            4 => {
                let memory = self.read16(bus, self.sp);
                let hl = self.hl();
                self.write16(bus, self.sp, hl);
                self.set_hl(memory);
                19
            }
            5 => {
                let de = self.de();
                self.set_de(self.hl());
                self.set_hl(de);
                4
            }
            6 => {
                self.iff1 = false;
                self.iff2 = false;
                self.ei_delay = 0;
                4
            }
            _ => {
                self.iff1 = true;
                self.iff2 = true;
                self.ei_delay = 2;
                4
            }
        }
    }
    fn execute_misc(&mut self, y: u8) -> u32 {
        match y {
            0 => {
                let carry = self.a >> 7;
                self.a = self.a.rotate_left(1);
                self.f = (self.f & (S | Z | PV)) | (self.a & (Y | X)) | carry;
            }
            1 => {
                let carry = self.a & 1;
                self.a = self.a.rotate_right(1);
                self.f = (self.f & (S | Z | PV)) | (self.a & (Y | X)) | carry;
            }
            2 => {
                let old_c = u8::from(self.f & C != 0);
                let new_c = self.a >> 7;
                self.a = (self.a << 1) | old_c;
                self.f = (self.f & (S | Z | PV)) | (self.a & (Y | X)) | new_c;
            }
            3 => {
                let old_c = if self.f & C != 0 { 0x80 } else { 0 };
                let new_c = self.a & 1;
                self.a = (self.a >> 1) | old_c;
                self.f = (self.f & (S | Z | PV)) | (self.a & (Y | X)) | new_c;
            }
            4 => self.daa(),
            5 => {
                self.a = !self.a;
                self.f = (self.f & (S | Z | PV | C)) | H | N | (self.a & (Y | X));
            }
            6 => {
                self.f = (self.f & (S | Z | PV)) | C | (self.a & (Y | X));
            }
            _ => {
                let old_c = self.f & C;
                self.f =
                    (self.f & (S | Z | PV)) | (self.a & (Y | X)) | if old_c != 0 { H } else { C };
            }
        }
        4
    }
    fn daa(&mut self) {
        let before = self.a;
        let subtract = self.f & N != 0;
        let mut correction = 0u8;
        let mut carry = self.f & C != 0;
        if self.f & H != 0 || (!subtract && self.a & 0x0f > 9) {
            correction |= 0x06;
        }
        if carry || (!subtract && self.a > 0x99) {
            correction |= 0x60;
            carry = true;
        }
        self.a = if subtract {
            self.a.wrapping_sub(correction)
        } else {
            self.a.wrapping_add(correction)
        };
        self.f = (self.f & N)
            | Self::sz53p(self.a)
            | if (before ^ self.a ^ correction) & 0x10 != 0 {
                H
            } else {
                0
            }
            | if carry { C } else { 0 };
    }

    fn rotate_shift(&mut self, y: u8, value: u8) -> u8 {
        let old_c = u8::from(self.f & C != 0);
        let (result, carry) = match y & 7 {
            0 => (value.rotate_left(1), value >> 7),
            1 => (value.rotate_right(1), value & 1),
            2 => ((value << 1) | old_c, value >> 7),
            3 => ((value >> 1) | (old_c << 7), value & 1),
            4 => (value << 1, value >> 7),
            5 => ((value >> 1) | (value & 0x80), value & 1),
            6 => ((value << 1) | 1, value >> 7),
            _ => (value >> 1, value & 1),
        };
        self.f = Self::sz53p(result) | carry;
        result
    }
    fn execute_cb<B: Z80Bus>(
        &mut self,
        bus: &mut B,
        opcode: u8,
        indexed: Option<u16>,
        _mode: Option<IndexMode>,
    ) -> u32 {
        let x = opcode >> 6;
        let y = (opcode >> 3) & 7;
        let z = opcode & 7;
        let memory = indexed.is_some() || z == 6;
        let address = indexed.or_else(|| if z == 6 { Some(self.hl()) } else { None });
        let value = if let Some(address) = address {
            bus.mem_read(address)
        } else {
            self.read_reg(bus, z)
        };
        match x {
            0 => {
                let result = self.rotate_shift(y, value);
                if let Some(address) = address {
                    bus.mem_write(address, result);
                }
                if z != 6 {
                    self.write_reg(bus, z, result);
                }
            }
            1 => {
                let mask = 1u8 << y;
                let zero = value & mask == 0;
                self.f = (self.f & C)
                    | H
                    | (value & (Y | X))
                    | if zero { Z | PV } else { 0 }
                    | if y == 7 && !zero { S } else { 0 };
            }
            2 | 3 => {
                let mask = 1u8 << y;
                let result = if x == 2 { value & !mask } else { value | mask };
                if let Some(address) = address {
                    bus.mem_write(address, result);
                }
                if z != 6 {
                    self.write_reg(bus, z, result);
                }
            }
            _ => unreachable!(),
        }
        if indexed.is_some() {
            if x == 1 {
                20
            } else {
                23
            }
        } else if memory {
            15
        } else {
            8
        }
    }

    fn execute_ed<B: Z80Bus>(&mut self, bus: &mut B, opcode: u8) -> u32 {
        let x = opcode >> 6;
        let y = (opcode >> 3) & 7;
        let z = opcode & 7;
        let p = y >> 1;
        let q = y & 1;
        if x != 1 {
            return self.execute_block(bus, opcode);
        }
        match z {
            0 => {
                let value = bus.io_read(self.bc());
                if y != 6 {
                    self.write_reg(bus, y, value);
                }
                self.f = (self.f & C) | Self::sz53p(value);
                12
            }
            1 => {
                let value = if y == 6 { 0 } else { self.read_reg(bus, y) };
                bus.io_write(self.bc(), value);
                12
            }
            2 => {
                let value = self.pair(p);
                self.adc_sbc_hl(value, q == 0);
                15
            }
            3 => {
                let address = self.fetch16(bus);
                if q == 0 {
                    let value = self.pair(p);
                    self.write16(bus, address, value);
                } else {
                    let value = self.read16(bus, address);
                    self.set_pair(p, value);
                }
                20
            }
            4 => {
                let value = self.a;
                self.a = 0;
                self.sub8(value, false);
                8
            }
            5 => {
                self.pc = self.pop16(bus);
                self.iff1 = self.iff2;
                14
            }
            6 => {
                self.interrupt_mode = match y {
                    0 | 1 | 4 | 5 => 0,
                    2 | 6 => 1,
                    _ => 2,
                };
                8
            }
            _ => self.execute_ed_z7(bus, y),
        }
    }
    fn execute_ed_z7<B: Z80Bus>(&mut self, bus: &mut B, y: u8) -> u32 {
        match y {
            0 => {
                self.i = self.a;
                9
            }
            1 => {
                self.r = self.a;
                9
            }
            2 | 3 => {
                self.a = if y == 2 { self.i } else { self.r };
                self.f = (self.f & C) | Self::sz53(self.a) | if self.iff2 { PV } else { 0 };
                9
            }
            4 => {
                let address = self.hl();
                let value = bus.mem_read(address);
                let low_a = self.a & 0x0f;
                bus.mem_write(address, (low_a << 4) | (value >> 4));
                self.a = (self.a & 0xf0) | (value & 0x0f);
                self.f = (self.f & C) | Self::sz53p(self.a);
                18
            }
            5 => {
                let address = self.hl();
                let value = bus.mem_read(address);
                let low_a = self.a & 0x0f;
                bus.mem_write(address, (value << 4) | low_a);
                self.a = (self.a & 0xf0) | (value >> 4);
                self.f = (self.f & C) | Self::sz53p(self.a);
                18
            }
            _ => 8,
        }
    }

    fn execute_block<B: Z80Bus>(&mut self, bus: &mut B, opcode: u8) -> u32 {
        let decrement = matches!(
            opcode,
            0xa8 | 0xa9 | 0xaa | 0xab | 0xb8 | 0xb9 | 0xba | 0xbb
        );
        let repeat = opcode & 0x10 != 0;
        let step = if decrement { -1 } else { 1 };
        match opcode & 0x07 {
            0 => {
                let value = bus.mem_read(self.hl());
                bus.mem_write(self.de(), value);
                self.set_hl(self.hl().wrapping_add_signed(step));
                self.set_de(self.de().wrapping_add_signed(step));
                self.set_bc(self.bc().wrapping_sub(1));
                let sum = self.a.wrapping_add(value);
                self.f = (self.f & (S | Z | C))
                    | if self.bc() != 0 { PV } else { 0 }
                    | (sum & X)
                    | ((sum & 0x02) << 4);
                if repeat && self.bc() != 0 {
                    self.pc = self.pc.wrapping_sub(2);
                    21
                } else {
                    16
                }
            }
            1 => {
                let value = bus.mem_read(self.hl());
                let result = self.a.wrapping_sub(value);
                let half = (self.a ^ value ^ result) & 0x10 != 0;
                self.set_hl(self.hl().wrapping_add_signed(step));
                self.set_bc(self.bc().wrapping_sub(1));
                let carry = self.f & C;
                let adjust = result.wrapping_sub(u8::from(half));
                self.f = carry
                    | N
                    | (result & S)
                    | if result == 0 { Z } else { 0 }
                    | if half { H } else { 0 }
                    | if self.bc() != 0 { PV } else { 0 }
                    | (adjust & X)
                    | ((adjust & 0x02) << 4);
                if repeat && self.bc() != 0 && result != 0 {
                    self.pc = self.pc.wrapping_sub(2);
                    21
                } else {
                    16
                }
            }
            2 => {
                let port = self.bc();
                let value = bus.io_read(port);
                bus.mem_write(self.hl(), value);
                self.set_hl(self.hl().wrapping_add_signed(step));
                self.b = self.b.wrapping_sub(1);
                let k = u16::from(value) + u16::from(self.c.wrapping_add_signed(step as i8));
                self.f = Self::sz53(self.b)
                    | if value & 0x80 != 0 { N } else { 0 }
                    | if k > 0xff { H | C } else { 0 }
                    | if Self::parity(((k as u8) & 7) ^ self.b) {
                        PV
                    } else {
                        0
                    };
                if repeat && self.b != 0 {
                    self.pc = self.pc.wrapping_sub(2);
                    21
                } else {
                    16
                }
            }
            3 => {
                let value = bus.mem_read(self.hl());
                bus.io_write(self.bc(), value);
                self.set_hl(self.hl().wrapping_add_signed(step));
                self.b = self.b.wrapping_sub(1);
                let k = u16::from(value) + u16::from(self.l);
                self.f = Self::sz53(self.b)
                    | if value & 0x80 != 0 { N } else { 0 }
                    | if k > 0xff { H | C } else { 0 }
                    | if Self::parity(((k as u8) & 7) ^ self.b) {
                        PV
                    } else {
                        0
                    };
                if repeat && self.b != 0 {
                    self.pc = self.pc.wrapping_sub(2);
                    21
                } else {
                    16
                }
            }
            _ => 8,
        }
    }

    fn index_value(&self, mode: IndexMode) -> u16 {
        match mode {
            IndexMode::Ix => self.ix,
            IndexMode::Iy => self.iy,
        }
    }
    fn set_index_value(&mut self, mode: IndexMode, value: u16) {
        match mode {
            IndexMode::Ix => self.ix = value,
            IndexMode::Iy => self.iy = value,
        }
    }
    fn read_index_reg<B: Z80Bus>(
        &mut self,
        bus: &mut B,
        r: u8,
        mode: IndexMode,
        address: Option<u16>,
        memory_form: bool,
    ) -> u8 {
        let index = self.index_value(mode);
        match r & 7 {
            0 => self.b,
            1 => self.c,
            2 => self.d,
            3 => self.e,
            4 if !memory_form => (index >> 8) as u8,
            5 if !memory_form => index as u8,
            4 => self.h,
            5 => self.l,
            6 => bus.mem_read(address.expect("indexed memory operand requires address")),
            _ => self.a,
        }
    }
    fn write_index_reg<B: Z80Bus>(
        &mut self,
        bus: &mut B,
        r: u8,
        mode: IndexMode,
        address: Option<u16>,
        memory_form: bool,
        value: u8,
    ) {
        match r & 7 {
            0 => self.b = value,
            1 => self.c = value,
            2 => self.d = value,
            3 => self.e = value,
            4 if !memory_form => {
                let low = self.index_value(mode) & 0x00ff;
                self.set_index_value(mode, (u16::from(value) << 8) | low);
            }
            5 if !memory_form => {
                let high = self.index_value(mode) & 0xff00;
                self.set_index_value(mode, high | u16::from(value));
            }
            4 => self.h = value,
            5 => self.l = value,
            6 => bus.mem_write(
                address.expect("indexed memory operand requires address"),
                value,
            ),
            _ => self.a = value,
        }
    }
    fn execute_index_prefix<B: Z80Bus>(&mut self, bus: &mut B, mut mode: IndexMode) -> u32 {
        let opcode = self.fetch_opcode(bus);
        if opcode == 0xdd || opcode == 0xfd {
            mode = if opcode == 0xdd {
                IndexMode::Ix
            } else {
                IndexMode::Iy
            };
            return 4 + self.execute_index_prefix(bus, mode);
        }
        if opcode == 0xed {
            let op = self.fetch_opcode(bus);
            return 4 + self.execute_ed(bus, op);
        }
        if opcode == 0xcb {
            let displacement = self.fetch8(bus) as i8;
            let op = self.fetch_opcode(bus);
            let address = self
                .index_value(mode)
                .wrapping_add_signed(i16::from(displacement));
            return self.execute_cb(bus, op, Some(address), Some(mode));
        }

        let x = opcode >> 6;
        let y = (opcode >> 3) & 7;
        let z = opcode & 7;
        let p = y >> 1;
        let q = y & 1;

        if x == 1 {
            if y == 6 && z == 6 {
                self.halted = true;
                return 8;
            }
            let memory = y == 6 || z == 6;
            let address = if memory {
                let d = self.fetch8(bus) as i8;
                Some(self.index_value(mode).wrapping_add_signed(i16::from(d)))
            } else {
                None
            };
            let value = self.read_index_reg(bus, z, mode, address, memory);
            self.write_index_reg(bus, y, mode, address, memory, value);
            return if memory { 19 } else { 8 };
        }
        if x == 2 {
            let memory = z == 6;
            let address = if memory {
                let d = self.fetch8(bus) as i8;
                Some(self.index_value(mode).wrapping_add_signed(i16::from(d)))
            } else {
                None
            };
            let value = self.read_index_reg(bus, z, mode, address, memory);
            self.alu(y, value);
            return if memory { 19 } else { 8 };
        }

        if x == 0 {
            match z {
                1 if q == 0 && p == 2 => {
                    let value = self.fetch16(bus);
                    self.set_index_value(mode, value);
                    return 14;
                }
                1 if q == 1 => {
                    let rhs = if p == 2 {
                        self.index_value(mode)
                    } else {
                        self.pair(p)
                    };
                    self.add_index(mode, rhs);
                    return 15;
                }
                2 if p == 2 => {
                    let address = self.fetch16(bus);
                    if q == 0 {
                        self.write16(bus, address, self.index_value(mode));
                    } else {
                        let value = self.read16(bus, address);
                        self.set_index_value(mode, value);
                    }
                    return 20;
                }
                3 if p == 2 => {
                    let value = if q == 0 {
                        self.index_value(mode).wrapping_add(1)
                    } else {
                        self.index_value(mode).wrapping_sub(1)
                    };
                    self.set_index_value(mode, value);
                    return 10;
                }
                4 | 5 if matches!(y, 4..=6) => {
                    let memory = y == 6;
                    let address = if memory {
                        let d = self.fetch8(bus) as i8;
                        Some(self.index_value(mode).wrapping_add_signed(i16::from(d)))
                    } else {
                        None
                    };
                    let value = self.read_index_reg(bus, y, mode, address, memory);
                    let result = if z == 4 {
                        self.inc8(value)
                    } else {
                        self.dec8(value)
                    };
                    self.write_index_reg(bus, y, mode, address, memory, result);
                    return if memory { 23 } else { 8 };
                }
                6 if matches!(y, 4..=6) => {
                    if y == 6 {
                        let d = self.fetch8(bus) as i8;
                        let address = self.index_value(mode).wrapping_add_signed(i16::from(d));
                        let value = self.fetch8(bus);
                        bus.mem_write(address, value);
                        return 19;
                    }
                    let value = self.fetch8(bus);
                    self.write_index_reg(bus, y, mode, None, false, value);
                    return 11;
                }
                _ => {}
            }
        }

        if x == 3 {
            match (z, q, p, y) {
                (1, 0, 2, _) => {
                    let value = self.pop16(bus);
                    self.set_index_value(mode, value);
                    return 14;
                }
                (1, 1, 2, _) => {
                    self.pc = self.index_value(mode);
                    return 8;
                }
                (1, 1, 3, _) => {
                    self.sp = self.index_value(mode);
                    return 10;
                }
                (3, _, _, 4) => {
                    let memory = self.read16(bus, self.sp);
                    let index = self.index_value(mode);
                    self.write16(bus, self.sp, index);
                    self.set_index_value(mode, memory);
                    return 23;
                }
                (5, 0, 2, _) => {
                    self.push16(bus, self.index_value(mode));
                    return 15;
                }
                _ => {}
            }
        }
        4 + self.execute_base(bus, opcode)
    }

    fn add_index(&mut self, mode: IndexMode, value: u16) {
        let index = self.index_value(mode);
        let sum = u32::from(index) + u32::from(value);
        let result = sum as u16;
        self.f = (self.f & (S | Z | PV))
            | ((result >> 8) as u8 & (Y | X))
            | if ((index ^ value ^ result) & 0x1000) != 0 {
                H
            } else {
                0
            }
            | if sum > 0xffff { C } else { 0 };
        self.set_index_value(mode, result);
    }
    pub fn save(&self, out: &mut crate::state::StateWriter) {
        for value in [
            self.a, self.f, self.b, self.c, self.d, self.e, self.h, self.l,
        ] {
            out.u8(value);
        }
        for value in [
            self.a2, self.f2, self.b2, self.c2, self.d2, self.e2, self.h2, self.l2,
        ] {
            out.u8(value);
        }
        out.u16(self.ix);
        out.u16(self.iy);
        out.u16(self.sp);
        out.u16(self.pc);
        out.u8(self.i);
        out.u8(self.r);
        out.u8(self.iff1 as u8);
        out.u8(self.iff2 as u8);
        out.u8(self.interrupt_mode);
        out.u8(self.halted as u8);
        out.u64(self.cycles);
        out.u8(self.ei_delay);
    }

    pub fn load(&mut self, input: &mut crate::state::StateReader<'_>) -> Result<(), String> {
        self.a = input.u8()?;
        self.f = input.u8()?;
        self.b = input.u8()?;
        self.c = input.u8()?;
        self.d = input.u8()?;
        self.e = input.u8()?;
        self.h = input.u8()?;
        self.l = input.u8()?;
        self.a2 = input.u8()?;
        self.f2 = input.u8()?;
        self.b2 = input.u8()?;
        self.c2 = input.u8()?;
        self.d2 = input.u8()?;
        self.e2 = input.u8()?;
        self.h2 = input.u8()?;
        self.l2 = input.u8()?;
        self.ix = input.u16()?;
        self.iy = input.u16()?;
        self.sp = input.u16()?;
        self.pc = input.u16()?;
        self.i = input.u8()?;
        self.r = input.u8()?;
        self.iff1 = input.u8()? != 0;
        self.iff2 = input.u8()? != 0;
        self.interrupt_mode = input.u8()?;
        self.halted = input.u8()? != 0;
        self.cycles = input.u64()?;
        self.ei_delay = input.u8()?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct TestBus {
        memory: [u8; 65536],
        input: u8,
        output: Vec<(u16, u8)>,
    }
    impl Default for TestBus {
        fn default() -> Self {
            Self {
                memory: [0; 65536],
                input: 0xa5,
                output: Vec::new(),
            }
        }
    }
    impl Z80Bus for TestBus {
        fn mem_read(&mut self, address: u16) -> u8 {
            self.memory[address as usize]
        }
        fn mem_write(&mut self, address: u16, value: u8) {
            self.memory[address as usize] = value;
        }
        fn io_read(&mut self, _port: u16) -> u8 {
            self.input
        }
        fn io_write(&mut self, port: u16, value: u8) {
            self.output.push((port, value));
        }
    }

    fn run(cpu: &mut Z80, bus: &mut TestBus, steps: usize) {
        for _ in 0..steps {
            cpu.step(bus);
        }
    }

    #[test]
    fn executes_base_arithmetic_branch_and_memory() {
        let mut bus = TestBus::default();
        bus.memory[..18].copy_from_slice(&[
            0x31, 0x00, 0xff, 0x3e, 0x10, 0x06, 0x03, 0x80, 0x10, 0xfd, 0x21, 0x00, 0x80, 0x77,
            0x76, 0, 0, 0,
        ]);
        let mut cpu = Z80::default();
        run(&mut cpu, &mut bus, 12);
        assert!(cpu.halted);
        assert_eq!(cpu.a, 0x16);
        assert_eq!(bus.memory[0x8000], 0x16);
    }
    #[test]
    fn cb_and_indexed_cb_operate_on_registers_and_memory() {
        let mut bus = TestBus::default();
        bus.memory[..16].copy_from_slice(&[
            0x06, 0x81, 0xcb, 0x00, 0xcb, 0x78, 0xdd, 0x21, 0x00, 0x90, 0xdd, 0xcb, 0x02, 0xc6,
            0x76, 0,
        ]);
        bus.memory[0x9002] = 0x10;
        let mut cpu = Z80::default();
        run(&mut cpu, &mut bus, 6);
        assert_eq!(cpu.b, 0x03);
        assert_eq!(cpu.f & Z, Z);
        assert_eq!(bus.memory[0x9002], 0x11);
    }

    #[test]
    fn ed_block_copy_and_io_paths_execute() {
        let mut bus = TestBus::default();
        bus.memory[..20].copy_from_slice(&[
            0x21, 0x00, 0x80, 0x11, 0x00, 0x90, 0x01, 0x03, 0x00, 0xed, 0xb0, 0x3e, 0x55, 0xd3,
            0x20, 0xdb, 0x20, 0x76, 0, 0,
        ]);
        bus.memory[0x8000..0x8003].copy_from_slice(&[1, 2, 3]);
        let mut cpu = Z80::default();
        for _ in 0..16 {
            if cpu.halted {
                break;
            }
            cpu.step(&mut bus);
        }
        assert_eq!(&bus.memory[0x9000..0x9003], &[1, 2, 3]);
        assert_eq!(bus.output.last().map(|entry| entry.1), Some(0x55));
        assert_eq!(cpu.a, 0xa5);
    }
    #[test]
    fn indexed_loads_and_interrupt_mode_two_work() {
        let mut bus = TestBus::default();
        bus.memory[..12].copy_from_slice(&[
            0xdd, 0x21, 0x00, 0x80, 0xdd, 0x36, 0x05, 0x7c, 0xdd, 0x7e, 0x05, 0x76,
        ]);
        let mut cpu = Z80::default();
        run(&mut cpu, &mut bus, 4);
        assert_eq!(bus.memory[0x8005], 0x7c);
        assert_eq!(cpu.a, 0x7c);
        cpu.halted = false;
        cpu.iff1 = true;
        cpu.interrupt_mode = 2;
        cpu.i = 0x90;
        bus.memory[0x9034] = 0x78;
        bus.memory[0x9035] = 0x56;
        let old_pc = cpu.pc;
        assert_eq!(cpu.irq(&mut bus, 0x34), 19);
        assert_eq!(cpu.pc, 0x5678);
        assert_eq!(bus.memory[cpu.sp as usize], old_pc as u8);
    }
}
