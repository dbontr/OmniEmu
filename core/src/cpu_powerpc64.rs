use crate::state::{StateReader, StateWriter};

const XER_CA: u32 = 1 << 29;
const XER_OV: u32 = 1 << 30;
const XER_SO: u32 = 1 << 31;

pub trait PowerPc64Bus {
    fn read8(&mut self, address: u64) -> u8;
    fn write8(&mut self, address: u64, value: u8);

    fn read16(&mut self, address: u64) -> u16 {
        u16::from_be_bytes([self.read8(address), self.read8(address.wrapping_add(1))])
    }
    fn write16(&mut self, address: u64, value: u16) {
        for (index, byte) in value.to_be_bytes().into_iter().enumerate() {
            self.write8(address.wrapping_add(index as u64), byte);
        }
    }
    fn read32(&mut self, address: u64) -> u32 {
        let mut bytes = [0; 4];
        for (index, byte) in bytes.iter_mut().enumerate() {
            *byte = self.read8(address.wrapping_add(index as u64));
        }
        u32::from_be_bytes(bytes)
    }
    fn write32(&mut self, address: u64, value: u32) {
        for (index, byte) in value.to_be_bytes().into_iter().enumerate() {
            self.write8(address.wrapping_add(index as u64), byte);
        }
    }
    fn read64(&mut self, address: u64) -> u64 {
        let mut bytes = [0; 8];
        for (index, byte) in bytes.iter_mut().enumerate() {
            *byte = self.read8(address.wrapping_add(index as u64));
        }
        u64::from_be_bytes(bytes)
    }
    fn write64(&mut self, address: u64, value: u64) {
        for (index, byte) in value.to_be_bytes().into_iter().enumerate() {
            self.write8(address.wrapping_add(index as u64), byte);
        }
    }
}
#[derive(Clone)]
pub struct PowerPc64 {
    pub gpr: [u64; 32],
    pub pc: u64,
    pub lr: u64,
    pub ctr: u64,
    pub cr: u32,
    pub xer: u32,
    pub msr: u64,
    pub cycles: u64,
    halted: bool,
    invalid_instruction: bool,
}

impl Default for PowerPc64 {
    fn default() -> Self {
        Self::new()
    }
}

impl PowerPc64 {
    pub fn new() -> Self {
        Self {
            gpr: [0; 32],
            pc: 0,
            lr: 0,
            ctr: 0,
            cr: 0,
            xer: 0,
            msr: 0,
            cycles: 0,
            halted: false,
            invalid_instruction: false,
        }
    }

    pub fn reset_to(&mut self, pc: u64) {
        *self = Self::new();
        self.pc = pc;
    }
    pub fn halted(&self) -> bool {
        self.halted
    }
    pub fn invalid_instruction(&self) -> bool {
        self.invalid_instruction
    }
    fn sext16(value: u32) -> u64 {
        (value as u16 as i16 as i64) as u64
    }
    fn d_address(&self, instruction: u32) -> u64 {
        let ra = ((instruction >> 16) & 31) as usize;
        let base = if ra == 0 { 0 } else { self.gpr[ra] };
        base.wrapping_add(Self::sext16(instruction))
    }

    fn set_cr_field(&mut self, field: usize, relation: std::cmp::Ordering) {
        let mut nibble = match relation {
            std::cmp::Ordering::Less => 0x8,
            std::cmp::Ordering::Greater => 0x4,
            std::cmp::Ordering::Equal => 0x2,
        };
        if self.xer & XER_SO != 0 {
            nibble |= 1;
        }
        let shift = 28 - field * 4;
        self.cr = (self.cr & !(0xf << shift)) | (nibble << shift);
    }

    fn record_cr0(&mut self, value: u64) {
        self.set_cr_field(0, (value as i64).cmp(&0));
    }

    fn set_ca(&mut self, active: bool) {
        if active {
            self.xer |= XER_CA;
        } else {
            self.xer &= !XER_CA;
        }
    }

    fn maybe_record(&mut self, instruction: u32, value: u64) {
        if instruction & 1 != 0 {
            self.record_cr0(value);
        }
    }

    fn branch_condition(&mut self, bo: u32, bi: u32) -> bool {
        let ctr_ok = if bo & 0x04 != 0 {
            true
        } else {
            self.ctr = self.ctr.wrapping_sub(1);
            (self.ctr != 0) ^ (bo & 0x02 != 0)
        };
        let cr_bit = (self.cr >> (31 - (bi & 31))) & 1 != 0;
        let cond_ok = bo & 0x10 != 0 || cr_bit == (bo & 0x08 != 0);
        ctr_ok && cond_ok
    }

    pub fn step<B: PowerPc64Bus>(&mut self, bus: &mut B) {
        if self.halted {
            self.cycles = self.cycles.wrapping_add(1);
            return;
        }
        self.invalid_instruction = false;
        let instruction = bus.read32(self.pc);
        let current_pc = self.pc;
        self.pc = self.pc.wrapping_add(4);
        let opcode = instruction >> 26;
        let rt = ((instruction >> 21) & 31) as usize;
        let ra = ((instruction >> 16) & 31) as usize;
        let rb = ((instruction >> 11) & 31) as usize;
        match opcode {
            7 => self.mulli(instruction),
            8 => self.subfic(instruction),
            10 => self.compare_immediate(instruction, false),
            11 => self.compare_immediate(instruction, true),
            12 => self.addic(instruction, false),
            13 => self.addic(instruction, true),
            14 => {
                let base = if ra == 0 { 0 } else { self.gpr[ra] };
                self.gpr[rt] = base.wrapping_add(Self::sext16(instruction));
            }
            15 => {
                let base = if ra == 0 { 0 } else { self.gpr[ra] };
                self.gpr[rt] = base.wrapping_add(Self::sext16(instruction) << 16);
            }
            16 => {
                let bo = (instruction >> 21) & 31;
                let bi = (instruction >> 16) & 31;
                let displacement = (((instruction & 0xfffc) as i16) as i64) as u64;
                if self.branch_condition(bo, bi) {
                    if instruction & 1 != 0 {
                        self.lr = self.pc;
                    }
                    self.pc = if instruction & 2 != 0 {
                        displacement
                    } else {
                        current_pc.wrapping_add(displacement)
                    };
                }
            }
            19 => self.execute_opcode19(instruction),
            18 => {
                let displacement = (((instruction & 0x03ff_fffc) << 6) as i32 >> 6) as i64 as u64;
                if instruction & 1 != 0 {
                    self.lr = self.pc;
                }
                self.pc = if instruction & 2 != 0 {
                    displacement
                } else {
                    current_pc.wrapping_add(displacement)
                };
            }
            20 => self.rotate_word(instruction, true, false),
            21 => self.rotate_word(instruction, false, false),
            23 => self.rotate_word(instruction, false, true),
            24 => self.gpr[ra] = self.gpr[rt] | u64::from(instruction & 0xffff),
            25 => self.gpr[ra] = self.gpr[rt] | (u64::from(instruction & 0xffff) << 16),
            26 => self.gpr[ra] = self.gpr[rt] ^ u64::from(instruction & 0xffff),
            27 => self.gpr[ra] = self.gpr[rt] ^ (u64::from(instruction & 0xffff) << 16),
            28 => {
                self.gpr[ra] = self.gpr[rt] & u64::from(instruction & 0xffff);
                self.record_cr0(self.gpr[ra]);
            }
            29 => {
                self.gpr[ra] = self.gpr[rt] & (u64::from(instruction & 0xffff) << 16);
                self.record_cr0(self.gpr[ra]);
            }
            32..=47 => self.load_store_immediate(bus, instruction, opcode),
            58 => {
                let base = if ra == 0 { 0 } else { self.gpr[ra] };
                let disp = (((instruction & 0xfffc) as u16 as i16) as i64) as u64;
                let address = base.wrapping_add(disp);
                match instruction & 3 {
                    0 => self.gpr[rt] = bus.read64(address),
                    1 if ra != 0 && ra != rt => {
                        self.gpr[rt] = bus.read64(address);
                        self.gpr[ra] = address;
                    }
                    2 => self.gpr[rt] = (bus.read32(address) as i32 as i64) as u64,
                    _ => self.invalid_instruction = true,
                }
            }
            62 => {
                let base = if ra == 0 { 0 } else { self.gpr[ra] };
                let disp = (((instruction & 0xfffc) as u16 as i16) as i64) as u64;
                let address = base.wrapping_add(disp);
                match instruction & 3 {
                    0 => bus.write64(address, self.gpr[rt]),
                    1 if ra != 0 => {
                        bus.write64(address, self.gpr[rt]);
                        self.gpr[ra] = address;
                    }
                    _ => self.invalid_instruction = true,
                }
            }
            31 => self.execute_xo(bus, instruction, rt, ra, rb),
            0 if instruction == 0 => self.halted = true,
            _ => self.invalid_instruction = true,
        }
        self.cycles = self.cycles.wrapping_add(1);
    }

    fn compare_immediate(&mut self, instruction: u32, signed: bool) {
        let field = ((instruction >> 23) & 7) as usize;
        let long = instruction & (1 << 21) != 0;
        let ra = ((instruction >> 16) & 31) as usize;
        let relation = if signed {
            if long {
                (self.gpr[ra] as i64).cmp(&(instruction as u16 as i16 as i64))
            } else {
                (self.gpr[ra] as u32 as i32).cmp(&(instruction as u16 as i16 as i32))
            }
        } else if long {
            self.gpr[ra].cmp(&u64::from(instruction & 0xffff))
        } else {
            (self.gpr[ra] as u32).cmp(&(instruction & 0xffff))
        };
        self.set_cr_field(field, relation);
    }

    fn mulli(&mut self, instruction: u32) {
        let rt = ((instruction >> 21) & 31) as usize;
        let ra = ((instruction >> 16) & 31) as usize;
        let rhs = instruction as u16 as i16 as i64;
        self.gpr[rt] = (self.gpr[ra] as i64).wrapping_mul(rhs) as u64;
    }

    fn subfic(&mut self, instruction: u32) {
        let rt = ((instruction >> 21) & 31) as usize;
        let ra = ((instruction >> 16) & 31) as usize;
        let rhs = Self::sext16(instruction);
        let (partial, carry1) = (!self.gpr[ra]).overflowing_add(rhs);
        let (result, carry2) = partial.overflowing_add(1);
        self.gpr[rt] = result;
        self.set_ca(carry1 || carry2);
    }

    fn addic(&mut self, instruction: u32, record: bool) {
        let rt = ((instruction >> 21) & 31) as usize;
        let ra = ((instruction >> 16) & 31) as usize;
        let (result, carry) = self.gpr[ra].overflowing_add(Self::sext16(instruction));
        self.gpr[rt] = result;
        self.set_ca(carry);
        if record {
            self.record_cr0(result);
        }
    }

    fn mask32(mb: u32, me: u32) -> u32 {
        let mut mask = 0u32;
        for bit in 0..32 {
            let architecture_bit = 31 - bit;
            let selected = if mb <= me {
                architecture_bit >= mb && architecture_bit <= me
            } else {
                architecture_bit >= mb || architecture_bit <= me
            };
            if selected {
                mask |= 1 << bit;
            }
        }
        mask
    }

    fn rotate_word(&mut self, instruction: u32, insert: bool, variable: bool) {
        let rs = ((instruction >> 21) & 31) as usize;
        let ra = ((instruction >> 16) & 31) as usize;
        let shift = if variable {
            (self.gpr[((instruction >> 11) & 31) as usize] & 31) as u32
        } else {
            (instruction >> 11) & 31
        };
        let mask = Self::mask32((instruction >> 6) & 31, (instruction >> 1) & 31);
        let rotated = (self.gpr[rs] as u32).rotate_left(shift);
        let value = if insert {
            (self.gpr[ra] as u32 & !mask) | (rotated & mask)
        } else {
            rotated & mask
        };
        self.gpr[ra] = u64::from(value);
        self.maybe_record(instruction, self.gpr[ra]);
    }

    fn load_store_immediate<B: PowerPc64Bus>(
        &mut self,
        bus: &mut B,
        instruction: u32,
        opcode: u32,
    ) {
        let rt = ((instruction >> 21) & 31) as usize;
        let ra = ((instruction >> 16) & 31) as usize;
        let address = self.d_address(instruction);
        match opcode {
            32 | 33 => self.gpr[rt] = u64::from(bus.read32(address)),
            34 | 35 => self.gpr[rt] = u64::from(bus.read8(address)),
            36 | 37 => bus.write32(address, self.gpr[rt] as u32),
            38 | 39 => bus.write8(address, self.gpr[rt] as u8),
            40 | 41 => self.gpr[rt] = u64::from(bus.read16(address)),
            42 | 43 => self.gpr[rt] = (bus.read16(address) as i16 as i64) as u64,
            44 | 45 => bus.write16(address, self.gpr[rt] as u16),
            46 => {
                let mut cursor = address;
                for reg in rt..32 {
                    self.gpr[reg] = u64::from(bus.read32(cursor));
                    cursor = cursor.wrapping_add(4);
                }
            }
            47 => {
                let mut cursor = address;
                for reg in rt..32 {
                    bus.write32(cursor, self.gpr[reg] as u32);
                    cursor = cursor.wrapping_add(4);
                }
            }
            _ => unreachable!(),
        }
        if matches!(opcode, 33 | 35 | 37 | 39 | 41 | 43 | 45) {
            if ra == 0 || (matches!(opcode, 33 | 35 | 41 | 43) && ra == rt) {
                self.invalid_instruction = true;
            } else {
                self.gpr[ra] = address;
            }
        }
    }

    fn cr_bit(&self, bit: u32) -> bool {
        self.cr & (1 << (31 - (bit & 31))) != 0
    }

    fn set_cr_bit(&mut self, bit: u32, value: bool) {
        let mask = 1 << (31 - (bit & 31));
        if value {
            self.cr |= mask;
        } else {
            self.cr &= !mask;
        }
    }

    fn execute_opcode19(&mut self, instruction: u32) {
        let xo = (instruction >> 1) & 0x3ff;
        match xo {
            0 => {
                let dst = ((instruction >> 23) & 7) as usize;
                let src = ((instruction >> 18) & 7) as usize;
                let nibble = (self.cr >> (28 - src * 4)) & 0xf;
                let shift = 28 - dst * 4;
                self.cr = (self.cr & !(0xf << shift)) | (nibble << shift);
            }
            16 => {
                let target = self.lr & !3;
                let bo = (instruction >> 21) & 31;
                let bi = (instruction >> 16) & 31;
                let next = self.pc;
                if self.branch_condition(bo, bi) {
                    self.pc = target;
                }
                if instruction & 1 != 0 {
                    self.lr = next;
                }
            }
            33 | 129 | 193 | 225 | 257 | 289 | 417 | 449 => {
                let bt = (instruction >> 21) & 31;
                let a = self.cr_bit((instruction >> 16) & 31);
                let b = self.cr_bit((instruction >> 11) & 31);
                let value = match xo {
                    33 => !(a || b),
                    129 => a && !b,
                    193 => a ^ b,
                    225 => !(a && b),
                    257 => a && b,
                    289 => a == b,
                    417 => a || !b,
                    449 => a || b,
                    _ => unreachable!(),
                };
                self.set_cr_bit(bt, value);
            }
            150 => {}
            528 => {
                let target = self.ctr & !3;
                let bo = (instruction >> 21) & 31;
                let bi = (instruction >> 16) & 31;
                let next = self.pc;
                if self.branch_condition(bo | 0x04, bi) {
                    self.pc = target;
                }
                if instruction & 1 != 0 {
                    self.lr = next;
                }
            }
            _ => self.invalid_instruction = true,
        }
    }

    fn execute_xo<B: PowerPc64Bus>(
        &mut self,
        bus: &mut B,
        instruction: u32,
        rt: usize,
        ra: usize,
        rb: usize,
    ) {
        let xo = (instruction >> 1) & 0x3ff;
        match xo {
            0 | 32 => self.compare_register(instruction, xo == 0),
            8 => self.subfc(instruction, rt, ra, rb),
            10 => self.addc(instruction, rt, ra, rb),
            19 => self.gpr[rt] = u64::from(self.cr),
            21 | 23 | 53 | 55 | 87 | 119 | 279 | 311 | 341 | 343 | 373 | 375 => {
                self.load_indexed(bus, instruction, xo, rt, ra, rb)
            }
            26 => {
                self.gpr[ra] = u64::from((self.gpr[rt] as u32).leading_zeros());
                self.maybe_record(instruction, self.gpr[ra]);
            }
            28 | 60 | 124 | 284 | 316 | 412 | 444 | 476 => {
                self.logical_register(instruction, xo, rt, ra, rb)
            }
            40 => {
                self.gpr[rt] = self.gpr[rb].wrapping_sub(self.gpr[ra]);
                self.maybe_record(instruction, self.gpr[rt]);
            }
            58 => {
                self.gpr[ra] = u64::from(self.gpr[rt].leading_zeros());
                self.maybe_record(instruction, self.gpr[ra]);
            }
            83 => self.gpr[rt] = self.msr,
            104 => {
                self.gpr[rt] = (0u64).wrapping_sub(self.gpr[ra]);
                self.maybe_record(instruction, self.gpr[rt]);
            }
            136 => self.subfe(instruction, rt, ra, rb),
            138 => self.adde(instruction, rt, ra, rb),
            144 => self.move_to_cr(instruction, rt),
            146 => self.msr = self.gpr[rt],
            149 | 151 | 181 | 183 | 215 | 247 | 407 | 439 => {
                self.store_indexed(bus, instruction, xo, rt, ra, rb)
            }
            233 => {
                self.gpr[rt] = (self.gpr[ra] as i64).wrapping_mul(self.gpr[rb] as i64) as u64;
                self.maybe_record(instruction, self.gpr[rt]);
            }
            235 => {
                let value = (self.gpr[ra] as u32 as i32).wrapping_mul(self.gpr[rb] as u32 as i32);
                self.gpr[rt] = (value as i64) as u64;
                self.maybe_record(instruction, self.gpr[rt]);
            }
            266 => {
                self.gpr[rt] = self.gpr[ra].wrapping_add(self.gpr[rb]);
                self.maybe_record(instruction, self.gpr[rt]);
            }
            339 => {
                let spr = ((instruction >> 16) & 0x1f) | ((instruction >> 6) & 0x3e0);
                self.gpr[rt] = match spr {
                    1 => self.xer as u64,
                    8 => self.lr,
                    9 => self.ctr,
                    _ => 0,
                };
            }
            371 => self.gpr[rt] = self.cycles,
            457 => self.divide(instruction, rt, ra, rb, false, true),
            459 => self.divide(instruction, rt, ra, rb, false, false),
            467 => {
                let spr = ((instruction >> 16) & 0x1f) | ((instruction >> 6) & 0x3e0);
                match spr {
                    1 => self.xer = self.gpr[rt] as u32,
                    8 => self.lr = self.gpr[rt],
                    9 => self.ctr = self.gpr[rt],
                    _ => {}
                }
            }
            489 => self.divide(instruction, rt, ra, rb, true, true),
            491 => self.divide(instruction, rt, ra, rb, true, false),
            24 | 27 | 536 | 539 | 792 | 794 | 824 | 826 | 827 => {
                self.shift(instruction, xo, rt, ra, rb)
            }
            598 | 854 => {}
            922 => {
                self.gpr[ra] = (self.gpr[rt] as u16 as i16 as i64) as u64;
                self.maybe_record(instruction, self.gpr[ra]);
            }
            954 => {
                self.gpr[ra] = (self.gpr[rt] as u8 as i8 as i64) as u64;
                self.maybe_record(instruction, self.gpr[ra]);
            }
            986 => {
                self.gpr[ra] = (self.gpr[rt] as u32 as i32 as i64) as u64;
                self.maybe_record(instruction, self.gpr[ra]);
            }
            _ => self.invalid_instruction = true,
        }
    }

    fn compare_register(&mut self, instruction: u32, signed: bool) {
        let field = ((instruction >> 23) & 7) as usize;
        let long = instruction & (1 << 21) != 0;
        let ra = ((instruction >> 16) & 31) as usize;
        let rb = ((instruction >> 11) & 31) as usize;
        let relation = if signed {
            if long {
                (self.gpr[ra] as i64).cmp(&(self.gpr[rb] as i64))
            } else {
                (self.gpr[ra] as u32 as i32).cmp(&(self.gpr[rb] as u32 as i32))
            }
        } else if long {
            self.gpr[ra].cmp(&self.gpr[rb])
        } else {
            (self.gpr[ra] as u32).cmp(&(self.gpr[rb] as u32))
        };
        self.set_cr_field(field, relation);
    }

    fn addc(&mut self, instruction: u32, rt: usize, ra: usize, rb: usize) {
        let (value, carry) = self.gpr[ra].overflowing_add(self.gpr[rb]);
        self.gpr[rt] = value;
        self.set_ca(carry);
        self.maybe_record(instruction, value);
    }

    fn subfc(&mut self, instruction: u32, rt: usize, ra: usize, rb: usize) {
        let value = self.gpr[rb].wrapping_sub(self.gpr[ra]);
        self.gpr[rt] = value;
        self.set_ca(self.gpr[rb] >= self.gpr[ra]);
        self.maybe_record(instruction, value);
    }

    fn adde(&mut self, instruction: u32, rt: usize, ra: usize, rb: usize) {
        let carry_in = u64::from(self.xer & XER_CA != 0);
        let (partial, carry1) = self.gpr[ra].overflowing_add(self.gpr[rb]);
        let (value, carry2) = partial.overflowing_add(carry_in);
        self.gpr[rt] = value;
        self.set_ca(carry1 || carry2);
        self.maybe_record(instruction, value);
    }

    fn subfe(&mut self, instruction: u32, rt: usize, ra: usize, rb: usize) {
        let carry_in = u64::from(self.xer & XER_CA != 0);
        let (partial, carry1) = self.gpr[rb].overflowing_add(!self.gpr[ra]);
        let (value, carry2) = partial.overflowing_add(carry_in);
        self.gpr[rt] = value;
        self.set_ca(carry1 || carry2);
        self.maybe_record(instruction, value);
    }

    fn move_to_cr(&mut self, instruction: u32, rs: usize) {
        let mask = (instruction >> 12) & 0xff;
        for field in 0..8 {
            if mask & (1 << (7 - field)) != 0 {
                let shift = 28 - field * 4;
                let nibble = (self.gpr[rs] as u32 >> shift) & 0xf;
                self.cr = (self.cr & !(0xf << shift)) | (nibble << shift);
            }
        }
    }

    fn logical_register(&mut self, instruction: u32, xo: u32, rs: usize, ra: usize, rb: usize) {
        let lhs = self.gpr[rs];
        let rhs = self.gpr[rb];
        let value = match xo {
            28 => lhs & rhs,
            60 => lhs & !rhs,
            124 => !(lhs | rhs),
            284 => !(lhs ^ rhs),
            316 => lhs ^ rhs,
            412 => lhs | !rhs,
            444 => lhs | rhs,
            476 => !(lhs & rhs),
            _ => unreachable!(),
        };
        self.gpr[ra] = value;
        self.maybe_record(instruction, value);
    }

    fn indexed_address(&self, ra: usize, rb: usize) -> u64 {
        let base = if ra == 0 { 0 } else { self.gpr[ra] };
        base.wrapping_add(self.gpr[rb])
    }

    fn load_indexed<B: PowerPc64Bus>(
        &mut self,
        bus: &mut B,
        _instruction: u32,
        xo: u32,
        rt: usize,
        ra: usize,
        rb: usize,
    ) {
        let address = self.indexed_address(ra, rb);
        self.gpr[rt] = match xo {
            21 | 53 => bus.read64(address),
            23 | 55 => u64::from(bus.read32(address)),
            87 | 119 => u64::from(bus.read8(address)),
            279 | 311 => u64::from(bus.read16(address)),
            341 | 373 => (bus.read32(address) as i32 as i64) as u64,
            343 | 375 => (bus.read16(address) as i16 as i64) as u64,
            _ => unreachable!(),
        };
        if matches!(xo, 53 | 55 | 119 | 311 | 373 | 375) {
            if ra == 0 || ra == rt {
                self.invalid_instruction = true;
            } else {
                self.gpr[ra] = address;
            }
        }
    }

    fn store_indexed<B: PowerPc64Bus>(
        &mut self,
        bus: &mut B,
        _instruction: u32,
        xo: u32,
        rs: usize,
        ra: usize,
        rb: usize,
    ) {
        let address = self.indexed_address(ra, rb);
        match xo {
            149 | 181 => bus.write64(address, self.gpr[rs]),
            151 | 183 => bus.write32(address, self.gpr[rs] as u32),
            215 | 247 => bus.write8(address, self.gpr[rs] as u8),
            407 | 439 => bus.write16(address, self.gpr[rs] as u16),
            _ => unreachable!(),
        }
        if matches!(xo, 181 | 183 | 247 | 439) {
            if ra == 0 {
                self.invalid_instruction = true;
            } else {
                self.gpr[ra] = address;
            }
        }
    }

    fn divide(
        &mut self,
        instruction: u32,
        rt: usize,
        ra: usize,
        rb: usize,
        signed: bool,
        doubleword: bool,
    ) {
        let (value, overflow) = if doubleword && signed {
            let lhs = self.gpr[ra] as i64;
            let rhs = self.gpr[rb] as i64;
            if rhs == 0 || (lhs == i64::MIN && rhs == -1) {
                (0, true)
            } else {
                (lhs.wrapping_div(rhs) as u64, false)
            }
        } else if doubleword {
            let rhs = self.gpr[rb];
            match self.gpr[ra].checked_div(rhs) {
                Some(value) => (value, false),
                None => (0, true),
            }
        } else if signed {
            let lhs = self.gpr[ra] as u32 as i32;
            let rhs = self.gpr[rb] as u32 as i32;
            if rhs == 0 || (lhs == i32::MIN && rhs == -1) {
                (0, true)
            } else {
                ((lhs.wrapping_div(rhs) as i64) as u64, false)
            }
        } else {
            let lhs = self.gpr[ra] as u32;
            let rhs = self.gpr[rb] as u32;
            match lhs.checked_div(rhs) {
                Some(value) => (u64::from(value), false),
                None => (0, true),
            }
        };
        self.gpr[rt] = value;
        if instruction & 0x400 != 0 {
            self.xer &= !XER_OV;
            if overflow {
                self.xer |= XER_OV | XER_SO;
            }
        }
        self.maybe_record(instruction, value);
    }

    fn shift(&mut self, instruction: u32, xo: u32, rs: usize, ra: usize, rb: usize) {
        let source = self.gpr[rs];
        let value = match xo {
            24 => {
                let amount = (self.gpr[rb] & 0x3f) as u32;
                if amount >= 32 {
                    0
                } else {
                    u64::from((source as u32) << amount)
                }
            }
            27 => {
                let amount = (self.gpr[rb] & 0x7f) as u32;
                if amount >= 64 {
                    0
                } else {
                    source << amount
                }
            }
            536 => {
                let amount = (self.gpr[rb] & 0x3f) as u32;
                if amount >= 32 {
                    0
                } else {
                    u64::from((source as u32) >> amount)
                }
            }
            539 => {
                let amount = (self.gpr[rb] & 0x7f) as u32;
                if amount >= 64 {
                    0
                } else {
                    source >> amount
                }
            }
            792 => self.shift_right_arithmetic_word(source, (self.gpr[rb] & 0x3f) as u32),
            794 => self.shift_right_arithmetic_double(source, (self.gpr[rb] & 0x7f) as u32),
            824 => self.shift_right_arithmetic_word(source, (instruction >> 11) & 31),
            826 | 827 => {
                let amount = ((instruction >> 11) & 31) | ((instruction & 2) << 4);
                self.shift_right_arithmetic_double(source, amount)
            }
            _ => unreachable!(),
        };
        self.gpr[ra] = value;
        self.maybe_record(instruction, value);
    }

    fn shift_right_arithmetic_word(&mut self, source: u64, amount: u32) -> u64 {
        let signed = source as u32 as i32;
        let value = if amount >= 32 {
            if signed < 0 {
                u64::MAX
            } else {
                0
            }
        } else {
            (signed >> amount) as i64 as u64
        };
        let shifted_out = if amount == 0 {
            false
        } else if amount >= 32 {
            signed < 0 && signed != 0
        } else {
            signed < 0 && (source as u32 & ((1u32 << amount) - 1)) != 0
        };
        self.set_ca(shifted_out);
        value
    }

    fn shift_right_arithmetic_double(&mut self, source: u64, amount: u32) -> u64 {
        let signed = source as i64;
        let value = if amount >= 64 {
            if signed < 0 {
                u64::MAX
            } else {
                0
            }
        } else {
            (signed >> amount) as u64
        };
        let shifted_out = if amount == 0 {
            false
        } else if amount >= 64 {
            signed < 0 && source != 0
        } else {
            signed < 0 && (source & ((1u64 << amount) - 1)) != 0
        };
        self.set_ca(shifted_out);
        value
    }

    pub fn save(&self, out: &mut StateWriter) {
        for reg in self.gpr {
            out.u64(reg);
        }
        out.u64(self.pc);
        out.u64(self.lr);
        out.u64(self.ctr);
        out.u32(self.cr);
        out.u32(self.xer);
        out.u64(self.msr);
        out.u64(self.cycles);
        out.u8(self.halted as u8);
        out.u8(self.invalid_instruction as u8);
    }

    pub fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        for reg in &mut self.gpr {
            *reg = input.u64()?;
        }
        self.pc = input.u64()?;
        self.lr = input.u64()?;
        self.ctr = input.u64()?;
        self.cr = input.u32()?;
        self.xer = input.u32()?;
        self.msr = input.u64()?;
        self.cycles = input.u64()?;
        self.halted = input.u8()? != 0;
        self.invalid_instruction = input.u8()? != 0;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::platform::PlatformId;

    struct Bus {
        bytes: Vec<u8>,
    }
    impl PowerPc64Bus for Bus {
        fn read8(&mut self, address: u64) -> u8 {
            self.bytes[address as usize % self.bytes.len()]
        }
        fn write8(&mut self, address: u64, value: u8) {
            let index = address as usize % self.bytes.len();
            self.bytes[index] = value;
        }
    }
    fn addi(rt: u32, ra: u32, imm: u16) -> u32 {
        (14 << 26) | (rt << 21) | (ra << 16) | u32::from(imm)
    }
    fn std(rs: u32, ra: u32, disp: u16) -> u32 {
        (62 << 26) | (rs << 21) | (ra << 16) | (u32::from(disp) & 0xfffc)
    }

    #[test]
    fn executes_64_bit_load_store_and_branch() {
        let mut bus = Bus {
            bytes: vec![0; 0x4000],
        };
        let program = [addi(3, 0, 0x1200), addi(4, 0, 0x1234), std(4, 3, 0), 0];
        for (index, instruction) in program.iter().enumerate() {
            bus.bytes[index * 4..index * 4 + 4].copy_from_slice(&instruction.to_be_bytes());
        }
        let mut cpu = PowerPc64::new();
        while !cpu.halted() && !cpu.invalid_instruction() {
            cpu.step(&mut bus);
        }
        assert_eq!(bus.read64(0x1200), 0x1234);
        assert_eq!(cpu.pc, 16);
    }

    #[test]
    fn immediate_byte_halfword_memory_and_sign_extension_execute() {
        let mut bus = Bus {
            bytes: vec![0; 0x4000],
        };
        let program: [u32; 5] = [0x88a6_0004, 0xa8e8_0006, 0x992a_0008, 0xb16c_000a, 0];
        for (index, instruction) in program.iter().enumerate() {
            bus.bytes[index * 4..index * 4 + 4].copy_from_slice(&instruction.to_be_bytes());
        }
        bus.bytes[0x1004] = 0xab;
        bus.bytes[0x1106..0x1108].copy_from_slice(&0xff80u16.to_be_bytes());
        let mut cpu = PowerPc64::new();
        cpu.gpr[6] = 0x1000;
        cpu.gpr[8] = 0x1100;
        cpu.gpr[9] = 0x1234;
        cpu.gpr[10] = 0x1200;
        cpu.gpr[11] = 0xbeef;
        cpu.gpr[12] = 0x1300;
        while !cpu.halted() && !cpu.invalid_instruction() {
            cpu.step(&mut bus);
        }
        assert_eq!(cpu.gpr[5], 0xab);
        assert_eq!(cpu.gpr[7], (-128i64) as u64);
        assert_eq!(bus.bytes[0x1208], 0x34);
        assert_eq!(&bus.bytes[0x130a..0x130c], &0xbeefu16.to_be_bytes());
        assert!(!cpu.invalid_instruction());
    }

    #[test]
    fn indexed_doubleword_memory_executes() {
        let mut bus = Bus {
            bytes: vec![0; 0x4000],
        };
        let program: [u32; 3] = [0x7dae_782a, 0x7e11_912a, 0];
        for (index, instruction) in program.iter().enumerate() {
            bus.bytes[index * 4..index * 4 + 4].copy_from_slice(&instruction.to_be_bytes());
        }
        bus.bytes[0x1420..0x1428].copy_from_slice(&0x1122_3344_5566_7788u64.to_be_bytes());
        let mut cpu = PowerPc64::new();
        cpu.gpr[14] = 0x1400;
        cpu.gpr[15] = 0x20;
        cpu.gpr[16] = 0xaabb_ccdd_eeff_0123;
        cpu.gpr[17] = 0x1500;
        cpu.gpr[18] = 0x30;
        while !cpu.halted() && !cpu.invalid_instruction() {
            cpu.step(&mut bus);
        }
        assert_eq!(cpu.gpr[13], 0x1122_3344_5566_7788);
        assert_eq!(bus.read64(0x1530), 0xaabb_ccdd_eeff_0123);
        assert!(!cpu.invalid_instruction());
    }

    #[test]
    fn doubleword_arithmetic_shift_count_and_sign_extension_execute() {
        let mut bus = Bus {
            bytes: vec![0; 0x4000],
        };
        let program: [u32; 9] = [
            0x7e74_a9d2,
            0x7ed7_c3d2,
            0x7f3a_db92,
            0x7c83_2836,
            0x7ce6_4436,
            0x7d49_3e74,
            0x7d8b_0074,
            0x7dcd_07b4,
            0,
        ];
        for (index, instruction) in program.iter().enumerate() {
            bus.bytes[index * 4..index * 4 + 4].copy_from_slice(&instruction.to_be_bytes());
        }
        let mut cpu = PowerPc64::new();
        cpu.gpr[20] = 9;
        cpu.gpr[21] = 7;
        cpu.gpr[23] = (-84i64) as u64;
        cpu.gpr[24] = 7;
        cpu.gpr[26] = 100;
        cpu.gpr[27] = 9;
        cpu.gpr[4] = 3;
        cpu.gpr[5] = 4;
        cpu.gpr[7] = 0x100;
        cpu.gpr[8] = 3;
        cpu.gpr[10] = (-256i64) as u64;
        cpu.gpr[12] = 0x1000;
        cpu.gpr[14] = 0x8000_0001;
        while !cpu.halted() && !cpu.invalid_instruction() {
            cpu.step(&mut bus);
        }
        assert_eq!(cpu.gpr[19], 63);
        assert_eq!(cpu.gpr[22], (-12i64) as u64);
        assert_eq!(cpu.gpr[25], 11);
        assert_eq!(cpu.gpr[3], 48);
        assert_eq!(cpu.gpr[6], 0x20);
        assert_eq!(cpu.gpr[9], (-2i64) as u64);
        assert_eq!(cpu.gpr[11], 51);
        assert_eq!(cpu.gpr[13], 0xffff_ffff_8000_0001);
        assert!(!cpu.invalid_instruction());
    }

    #[test]
    fn compare_fields_and_branch_to_link_register_execute() {
        let mut bus = Bus {
            bytes: vec![0; 0x100],
        };
        let program: [u32; 3] = [0x7d2f_8000, 0x7db1_9040, 0x4e80_0020];
        for (index, instruction) in program.iter().enumerate() {
            bus.bytes[index * 4..index * 4 + 4].copy_from_slice(&instruction.to_be_bytes());
        }
        let mut cpu = PowerPc64::new();
        cpu.gpr[15] = u64::MAX;
        cpu.gpr[16] = 1;
        cpu.gpr[17] = 5;
        cpu.gpr[18] = 2;
        cpu.lr = 0x20;
        for _ in 0..3 {
            cpu.step(&mut bus);
        }
        assert_eq!((cpu.cr >> 20) & 0xf, 0x8);
        assert_eq!((cpu.cr >> 16) & 0xf, 0x4);
        assert_eq!(cpu.pc, 0x20);
        cpu.step(&mut bus);
        assert!(cpu.halted());
        assert!(!cpu.invalid_instruction());
    }

    #[test]
    fn state_round_trip_preserves_64_bit_registers() {
        let mut cpu = PowerPc64::new();
        cpu.gpr[7] = 0x1122_3344_5566_7788;
        cpu.pc = 0x8000_0001_0000_1000;
        cpu.lr = 0xfeed_face_cafe_beef;
        let mut writer = StateWriter::new(PlatformId::Xbox360, 77);
        cpu.save(&mut writer);
        let state = writer.finish();
        let mut reader = StateReader::new(&state, PlatformId::Xbox360, 77).unwrap();
        let mut restored = PowerPc64::new();
        restored.load(&mut reader).unwrap();
        reader.finish().unwrap();
        assert_eq!(restored.gpr, cpu.gpr);
        assert_eq!(restored.pc, cpu.pc);
        assert_eq!(restored.lr, cpu.lr);
    }
}
