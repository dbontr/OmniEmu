use crate::state::{StateReader, StateWriter};

const XER_SO: u32 = 1 << 31;
const XER_OV: u32 = 1 << 30;
const XER_CA: u32 = 1 << 29;

const MSR_EE: u32 = 0x0000_8000;
const MSR_PR: u32 = 0x0000_4000;
const MSR_FP: u32 = 0x0000_2000;
const MSR_FE0: u32 = 0x0000_0800;
const MSR_SE: u32 = 0x0000_0400;
const MSR_BE: u32 = 0x0000_0200;
const MSR_FE1: u32 = 0x0000_0100;
const MSR_IP: u32 = 0x0000_0040;
const MSR_IR: u32 = 0x0000_0020;
const MSR_DR: u32 = 0x0000_0010;
const MSR_RI: u32 = 0x0000_0002;
const MSR_LE: u32 = 0x0000_0001;

const FPSCR_FX: u32 = 1 << 31;
const FPSCR_FEX: u32 = 1 << 30;
const FPSCR_VX: u32 = 1 << 29;
const FPSCR_OX: u32 = 1 << 28;
const FPSCR_UX: u32 = 1 << 27;
const FPSCR_ZX: u32 = 1 << 26;
const FPSCR_XX: u32 = 1 << 25;
const FPSCR_VXSNAN: u32 = 1 << 24;
const FPSCR_VXISI: u32 = 1 << 23;
const FPSCR_VXIDI: u32 = 1 << 22;
const FPSCR_VXZDZ: u32 = 1 << 21;
const FPSCR_VXIMZ: u32 = 1 << 20;
const FPSCR_VXVC: u32 = 1 << 19;
const FPSCR_FR: u32 = 1 << 18;
const FPSCR_FI: u32 = 1 << 17;
const FPSCR_FPRF_MASK: u32 = 0x1f << 12;
const FPSCR_FPCC_MASK: u32 = 0x0f << 12;
const FPSCR_VXSOFT: u32 = 1 << 10;
const FPSCR_VXSQRT: u32 = 1 << 9;
const FPSCR_VXCVI: u32 = 1 << 8;
const FPSCR_VE: u32 = 1 << 7;
const FPSCR_OE: u32 = 1 << 6;
const FPSCR_UE: u32 = 1 << 5;
const FPSCR_ZE: u32 = 1 << 4;
const FPSCR_XE: u32 = 1 << 3;
const FPSCR_NI: u32 = 1 << 2;
const FPSCR_VX_ANY: u32 = FPSCR_VXSNAN
    | FPSCR_VXISI
    | FPSCR_VXIDI
    | FPSCR_VXZDZ
    | FPSCR_VXIMZ
    | FPSCR_VXVC
    | FPSCR_VXSOFT
    | FPSCR_VXSQRT
    | FPSCR_VXCVI;

const DOUBLE_SIGN: u64 = 1 << 63;
const DOUBLE_EXP: u64 = 0x7ff << 52;
const DOUBLE_FRAC: u64 = (1 << 52) - 1;
const DOUBLE_QBIT: u64 = 1 << 51;
const CANONICAL_QNAN: u64 = DOUBLE_EXP | DOUBLE_QBIT;

// Gekko reciprocal-estimate interpolation constants. These are hardware lookup data,
// cross-checked against independent MIT-licensed console-validation work; see REFERENCES.md.
const FRES_ESTIMATE: [(u32, i32); 32] = [
    (0x7ff800, 0x3e1),
    (0x783800, 0x3a7),
    (0x70ea00, 0x371),
    (0x6a0800, 0x340),
    (0x638800, 0x313),
    (0x5d6200, 0x2ea),
    (0x579000, 0x2c4),
    (0x520800, 0x2a0),
    (0x4cc800, 0x27f),
    (0x47ca00, 0x261),
    (0x430800, 0x245),
    (0x3e8000, 0x22a),
    (0x3a2c00, 0x212),
    (0x360800, 0x1fb),
    (0x321400, 0x1e5),
    (0x2e4a00, 0x1d1),
    (0x2aa800, 0x1be),
    (0x272c00, 0x1ac),
    (0x23d600, 0x19b),
    (0x209e00, 0x18b),
    (0x1d8800, 0x17c),
    (0x1a9000, 0x16e),
    (0x17ae00, 0x15b),
    (0x14f800, 0x15b),
    (0x124400, 0x143),
    (0x0fbe00, 0x143),
    (0x0d3800, 0x12d),
    (0x0ade00, 0x12d),
    (0x088400, 0x11a),
    (0x065000, 0x11a),
    (0x041c00, 0x108),
    (0x020c00, 0x106),
];

const FRSQRTE_ESTIMATE: [(u32, i32); 32] = [
    (0x1a7e800, -0x568),
    (0x17cb800, -0x4f3),
    (0x1552800, -0x48d),
    (0x130c000, -0x435),
    (0x10f2000, -0x3e7),
    (0x0eff000, -0x3a2),
    (0x0d2e000, -0x365),
    (0x0b7c000, -0x32e),
    (0x09e5000, -0x2fc),
    (0x0867000, -0x2d0),
    (0x06ff000, -0x2a8),
    (0x05ab800, -0x283),
    (0x046a000, -0x261),
    (0x0339800, -0x243),
    (0x0218800, -0x226),
    (0x0105800, -0x20b),
    (0x3ffa000, -0x7a4),
    (0x3c29000, -0x700),
    (0x38aa000, -0x670),
    (0x3572000, -0x5f2),
    (0x3279000, -0x584),
    (0x2fb7000, -0x524),
    (0x2d26000, -0x4cc),
    (0x2ac0000, -0x47e),
    (0x2881000, -0x43a),
    (0x2665000, -0x3fa),
    (0x2468000, -0x3c2),
    (0x2287000, -0x38e),
    (0x20c1000, -0x35e),
    (0x1f12000, -0x332),
    (0x1d79000, -0x30a),
    (0x1bf4000, -0x2e6),
];

fn is_signaling_nan(value: f64) -> bool {
    let bits = value.to_bits();
    bits & DOUBLE_EXP == DOUBLE_EXP && bits & DOUBLE_FRAC != 0 && bits & DOUBLE_QBIT == 0
}

fn quiet_nan(value: f64) -> f64 {
    f64::from_bits(value.to_bits() | DOUBLE_QBIT)
}

fn classify_f64(value: f64) -> u32 {
    let bits = value.to_bits();
    let sign = bits & DOUBLE_SIGN != 0;
    let exponent = bits & DOUBLE_EXP;
    let fraction = bits & DOUBLE_FRAC;
    if exponent == DOUBLE_EXP {
        if fraction != 0 {
            0x11
        } else if sign {
            0x09
        } else {
            0x05
        }
    } else if exponent == 0 {
        if fraction == 0 {
            if sign {
                0x12
            } else {
                0x02
            }
        } else if sign {
            0x18
        } else {
            0x14
        }
    } else if sign {
        0x08
    } else {
        0x04
    }
}

fn classify_f32(value: f32) -> u32 {
    let bits = value.to_bits();
    let sign = bits >> 31 != 0;
    let exponent = bits & 0x7f80_0000;
    let fraction = bits & 0x007f_ffff;
    if exponent == 0x7f80_0000 {
        if fraction != 0 {
            0x11
        } else if sign {
            0x09
        } else {
            0x05
        }
    } else if exponent == 0 {
        if fraction == 0 {
            if sign {
                0x12
            } else {
                0x02
            }
        } else if sign {
            0x18
        } else {
            0x14
        }
    } else if sign {
        0x08
    } else {
        0x04
    }
}

#[derive(Clone, Copy)]
struct FpArithmeticResult {
    value: f64,
    invalid: bool,
}

fn force_25_bit(value: f64) -> f64 {
    let mut bits = value.to_bits();
    let exponent = bits & DOUBLE_EXP;
    let fraction = bits & DOUBLE_FRAC;
    if exponent == 0 && fraction != 0 {
        let shift = fraction.leading_zeros().saturating_sub(11);
        let keep_mask = ((0xffff_ffff_f800_0000u64 as i64) >> shift) as u64;
        let round_bit = 0x0800_0000u64 >> shift;
        bits = (bits & keep_mask).wrapping_add(bits & round_bit);
    } else {
        bits = (bits & 0xffff_ffff_f800_0000).wrapping_add(bits & 0x0800_0000);
    }
    f64::from_bits(bits)
}

fn next_up_f32(value: f32) -> f32 {
    if value.is_nan() || value == f32::INFINITY {
        return value;
    }
    if value == 0.0 {
        return f32::from_bits(1);
    }
    let bits = value.to_bits();
    if value.is_sign_positive() {
        f32::from_bits(bits + 1)
    } else {
        f32::from_bits(bits - 1)
    }
}

fn next_down_f32(value: f32) -> f32 {
    if value.is_nan() || value == f32::NEG_INFINITY {
        return value;
    }
    if value == 0.0 {
        return f32::from_bits(0x8000_0001);
    }
    let bits = value.to_bits();
    if value.is_sign_positive() {
        f32::from_bits(bits - 1)
    } else {
        f32::from_bits(bits + 1)
    }
}

fn gekko_reciprocal_estimate(value: f64) -> f64 {
    let bits = value.to_bits();
    let mantissa = bits & DOUBLE_FRAC;
    let sign = bits & DOUBLE_SIGN;
    let exponent = bits & DOUBLE_EXP;

    if mantissa == 0 && exponent == 0 {
        return f64::from_bits(sign | DOUBLE_EXP);
    }
    if exponent == DOUBLE_EXP {
        return if mantissa == 0 {
            f64::from_bits(sign)
        } else {
            quiet_nan(value)
        };
    }
    if exponent < (895u64 << 52) {
        return f64::from(f32::MAX).copysign(value);
    }
    if exponent >= (1149u64 << 52) {
        return f64::from_bits(sign);
    }

    let output_exponent = (0x7fdu64 << 52) - exponent;
    let index = (mantissa >> 37) as usize;
    let (base, decrement) = FRES_ESTIMATE[index / 1024];
    let interpolation = index % 1024;
    let fraction = i64::from(base) - (i64::from(decrement) * interpolation as i64 + 1) / 2;
    f64::from_bits(sign | output_exponent | ((fraction as u64) << 29))
}

fn gekko_reciprocal_sqrt_estimate(value: f64) -> f64 {
    let bits = value.to_bits();
    let mut mantissa = bits & DOUBLE_FRAC;
    let sign = bits & DOUBLE_SIGN;
    let mut exponent = (bits & DOUBLE_EXP) as i64;

    if mantissa == 0 && exponent == 0 {
        return f64::from_bits(sign | DOUBLE_EXP);
    }
    if exponent == DOUBLE_EXP as i64 {
        if mantissa == 0 {
            return if sign != 0 {
                f64::from_bits(CANONICAL_QNAN)
            } else {
                0.0
            };
        }
        return quiet_nan(value);
    }
    if sign != 0 {
        return f64::from_bits(CANONICAL_QNAN);
    }

    if exponent == 0 {
        while mantissa & (1 << 52) == 0 {
            exponent -= 1i64 << 52;
            mantissa <<= 1;
        }
        mantissa &= DOUBLE_FRAC;
        exponent += 1i64 << 52;
    }

    let exponent_lsb = (exponent as u64) & (1 << 52);
    let output_exponent =
        ((0x3ffi64 << 52) - (exponent - (0x3fei64 << 52)) / 2) & DOUBLE_EXP as i64;
    let index = ((exponent_lsb | mantissa) >> 37) as usize;
    let (base, decrement) = FRSQRTE_ESTIMATE[index / 2048];
    let interpolation = index % 2048;
    let fraction = i64::from(base) + i64::from(decrement) * interpolation as i64;
    f64::from_bits(output_exponent as u64 | ((fraction as u64) << 26))
}

const HID2_PSE: u32 = 1 << 29;
const HID2_LSQE: u32 = 1 << 31;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PowerPcException {
    SystemReset,
    MachineCheck,
    DataStorage,
    InstructionStorage,
    ExternalInterrupt,
    Alignment,
    Program,
    FloatingPointUnavailable,
    Decrementer,
    SystemCall,
    Trace,
}

pub trait PowerPcBus {
    fn read8(&mut self, address: u32) -> u8;
    fn write8(&mut self, address: u32, value: u8);

    fn read16(&mut self, address: u32) -> u16 {
        u16::from_be_bytes([self.read8(address), self.read8(address.wrapping_add(1))])
    }

    fn write16(&mut self, address: u32, value: u16) {
        let [a, b] = value.to_be_bytes();
        self.write8(address, a);
        self.write8(address.wrapping_add(1), b);
    }

    fn read32(&mut self, address: u32) -> u32 {
        let mut bytes = [0; 4];
        for (index, byte) in bytes.iter_mut().enumerate() {
            *byte = self.read8(address.wrapping_add(index as u32));
        }
        u32::from_be_bytes(bytes)
    }

    fn write32(&mut self, address: u32, value: u32) {
        for (index, byte) in value.to_be_bytes().into_iter().enumerate() {
            self.write8(address.wrapping_add(index as u32), byte);
        }
    }

    fn read64(&mut self, address: u32) -> u64 {
        let mut bytes = [0; 8];
        for (index, byte) in bytes.iter_mut().enumerate() {
            *byte = self.read8(address.wrapping_add(index as u32));
        }
        u64::from_be_bytes(bytes)
    }

    fn write64(&mut self, address: u32, value: u64) {
        for (index, byte) in value.to_be_bytes().into_iter().enumerate() {
            self.write8(address.wrapping_add(index as u32), byte);
        }
    }
}

#[derive(Clone)]
pub struct PowerPc750 {
    pub gpr: [u32; 32],
    pub fpr: [u64; 32],
    pub paired1: [u64; 32],
    pub gqr: [u32; 8],
    pub pc: u32,
    pub lr: u32,
    pub ctr: u32,
    pub cr: u32,
    pub xer: u32,
    pub msr: u32,
    pub srr0: u32,
    pub srr1: u32,
    pub dar: u32,
    pub dsisr: u32,
    pub decrementer: u32,
    pub time_base: u64,
    pub hid0: u32,
    pub hid2: u32,
    pub fpscr: u32,
    pub cycles: u64,
    reservation: Option<u32>,
    external_interrupt: bool,
    halted: bool,
    exception_raised: bool,
}

impl Default for PowerPc750 {
    fn default() -> Self {
        Self::new()
    }
}

impl PowerPc750 {
    pub fn new() -> Self {
        Self {
            gpr: [0; 32],
            fpr: [0; 32],
            paired1: [0; 32],
            gqr: [0; 8],
            pc: 0xfff0_0100,
            lr: 0,
            ctr: 0,
            cr: 0,
            xer: 0,
            msr: 0,
            srr0: 0,
            srr1: 0,
            dar: 0,
            dsisr: 0,
            decrementer: 0xffff_ffff,
            time_base: 0,
            hid0: 0,
            hid2: 0,
            fpscr: 0,
            cycles: 0,
            reservation: None,
            external_interrupt: false,
            halted: false,
            exception_raised: false,
        }
    }

    pub fn reset_to(&mut self, pc: u32) {
        *self = Self::new();
        self.pc = pc & !3;
    }

    pub fn set_external_interrupt(&mut self, pending: bool) {
        self.external_interrupt = pending;
        if pending {
            self.halted = false;
        }
    }

    pub fn halted(&self) -> bool {
        self.halted
    }

    pub fn step<B: PowerPcBus>(&mut self, bus: &mut B) -> u32 {
        self.exception_raised = false;
        if self.external_interrupt && self.msr & MSR_EE != 0 {
            self.take_exception(PowerPcException::ExternalInterrupt, self.pc);
            self.tick_time(1);
            return 1;
        }
        if self.halted {
            self.tick_time(1);
            return 1;
        }
        if self.pc & 3 != 0 {
            self.dar = self.pc;
            self.take_exception(PowerPcException::Alignment, self.pc);
            self.tick_time(1);
            return 1;
        }
        let current = self.pc;
        let instruction = bus.read32(current);
        self.pc = current.wrapping_add(4);
        self.execute(bus, instruction, current);
        self.tick_time(1);
        1
    }

    fn tick_time(&mut self, cycles: u32) {
        self.cycles = self.cycles.wrapping_add(u64::from(cycles));
        self.time_base = self.time_base.wrapping_add(u64::from(cycles));
        let previous = self.decrementer;
        self.decrementer = self.decrementer.wrapping_sub(cycles);
        if previous < cycles && self.msr & MSR_EE != 0 && !self.exception_raised {
            self.take_exception(PowerPcException::Decrementer, self.pc);
        }
    }

    fn execute<B: PowerPcBus>(&mut self, bus: &mut B, instruction: u32, current: u32) {
        let opcode = instruction >> 26;
        match opcode {
            3 => self.trap_immediate(instruction),
            7 => self.mulli(instruction),
            8 => self.subfic(instruction),
            10 => self.compare_immediate(instruction, false),
            11 => self.compare_immediate(instruction, true),
            12 => self.addic(instruction, false),
            13 => self.addic(instruction, true),
            14 => self.addi(instruction, false),
            15 => self.addi(instruction, true),
            16 => self.branch_conditional(instruction, current),
            17 => self.take_exception(PowerPcException::SystemCall, current),
            18 => self.branch_immediate(instruction, current),
            19 => self.execute_opcode19(instruction, current),
            20 => self.rotate_mask_insert(instruction),
            21 => self.rotate_mask(instruction, false),
            23 => self.rotate_mask(instruction, true),
            24 => self.logical_immediate(instruction, 0),
            25 => self.logical_immediate(instruction, 1),
            26 => self.logical_immediate(instruction, 2),
            27 => self.logical_immediate(instruction, 3),
            28 => self.logical_immediate(instruction, 4),
            29 => self.logical_immediate(instruction, 5),
            4 => self.execute_paired_single(bus, instruction),
            31 => self.execute_opcode31(bus, instruction),
            32..=47 => self.load_store_immediate(bus, instruction, opcode),
            48..=55 => self.load_store_float(bus, instruction, opcode),
            56 | 57 | 60 | 61 => self.load_store_paired(bus, instruction, opcode),
            59 => self.execute_float_single(instruction),
            63 => self.execute_float_double(instruction),
            _ => self.take_exception(PowerPcException::Program, current),
        }
    }

    fn gpr(&self, index: usize) -> u32 {
        self.gpr[index & 31]
    }

    fn base(&self, index: usize) -> u32 {
        if index == 0 {
            0
        } else {
            self.gpr(index)
        }
    }

    fn rd(instruction: u32) -> usize {
        ((instruction >> 21) & 31) as usize
    }

    fn ra(instruction: u32) -> usize {
        ((instruction >> 16) & 31) as usize
    }

    fn rb(instruction: u32) -> usize {
        ((instruction >> 11) & 31) as usize
    }

    fn simm(instruction: u32) -> i32 {
        i32::from(instruction as i16)
    }

    fn addi(&mut self, instruction: u32, shifted: bool) {
        let rd = Self::rd(instruction);
        let ra = Self::ra(instruction);
        let immediate = if shifted {
            (instruction & 0xffff) << 16
        } else {
            Self::simm(instruction) as u32
        };
        self.gpr[rd] = self.base(ra).wrapping_add(immediate);
    }

    fn mulli(&mut self, instruction: u32) {
        let rd = Self::rd(instruction);
        let ra = Self::ra(instruction);
        self.gpr[rd] = (self.gpr(ra) as i32).wrapping_mul(Self::simm(instruction)) as u32;
    }

    fn subfic(&mut self, instruction: u32) {
        let rd = Self::rd(instruction);
        let ra = Self::ra(instruction);
        let immediate = Self::simm(instruction) as u32;
        let source = self.gpr(ra);
        let (result, carry1) = (!source).overflowing_add(immediate);
        let (result, carry2) = result.overflowing_add(1);
        self.gpr[rd] = result;
        self.set_ca(carry1 || carry2);
    }

    fn addic(&mut self, instruction: u32, record: bool) {
        let rd = Self::rd(instruction);
        let ra = Self::ra(instruction);
        let (result, carry) = self.gpr(ra).overflowing_add(Self::simm(instruction) as u32);
        self.gpr[rd] = result;
        self.set_ca(carry);
        if record {
            self.record_cr0(result);
        }
    }

    fn compare_immediate(&mut self, instruction: u32, signed: bool) {
        let field = ((instruction >> 23) & 7) as usize;
        let ra = Self::ra(instruction);
        let lhs = self.gpr(ra);
        let rhs = if signed {
            Self::simm(instruction) as u32
        } else {
            instruction & 0xffff
        };
        self.set_compare_field(field, lhs, rhs, signed);
    }

    fn logical_immediate(&mut self, instruction: u32, operation: u8) {
        let rs = Self::rd(instruction);
        let ra = Self::ra(instruction);
        let mut immediate = instruction & 0xffff;
        if matches!(operation, 1 | 3 | 5) {
            immediate <<= 16;
        }
        let result = match operation {
            0 | 1 => self.gpr(rs) | immediate,
            2 | 3 => self.gpr(rs) ^ immediate,
            4 | 5 => self.gpr(rs) & immediate,
            _ => unreachable!(),
        };
        self.gpr[ra] = result;
        if matches!(operation, 4 | 5) {
            self.record_cr0(result);
        }
    }

    fn rotate_mask_insert(&mut self, instruction: u32) {
        let rs = Self::rd(instruction);
        let ra = Self::ra(instruction);
        let shift = (instruction >> 11) & 31;
        let mb = (instruction >> 6) & 31;
        let me = (instruction >> 1) & 31;
        let mask = Self::ppc_mask(mb, me);
        let rotated = self.gpr(rs).rotate_left(shift);
        let result = (self.gpr(ra) & !mask) | (rotated & mask);
        self.gpr[ra] = result;
        if instruction & 1 != 0 {
            self.record_cr0(result);
        }
    }

    fn rotate_mask(&mut self, instruction: u32, variable: bool) {
        let rs = Self::rd(instruction);
        let ra = Self::ra(instruction);
        let shift = if variable {
            self.gpr(Self::rb(instruction)) & 31
        } else {
            (instruction >> 11) & 31
        };
        let mb = (instruction >> 6) & 31;
        let me = (instruction >> 1) & 31;
        let result = self.gpr(rs).rotate_left(shift) & Self::ppc_mask(mb, me);
        self.gpr[ra] = result;
        if instruction & 1 != 0 {
            self.record_cr0(result);
        }
    }

    fn ppc_mask(mb: u32, me: u32) -> u32 {
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

    fn branch_immediate(&mut self, instruction: u32, current: u32) {
        let offset = ((instruction & 0x03ff_fffc) as i32) << 6 >> 6;
        let absolute = instruction & 2 != 0;
        if instruction & 1 != 0 {
            self.lr = current.wrapping_add(4);
        }
        self.pc = if absolute {
            offset as u32
        } else {
            current.wrapping_add(offset as u32)
        } & !3;
    }

    fn branch_conditional(&mut self, instruction: u32, current: u32) {
        let bo = ((instruction >> 21) & 31) as u8;
        let bi = ((instruction >> 16) & 31) as u8;
        let offset = ((instruction & 0xfffc) as i16) as i32;
        let absolute = instruction & 2 != 0;
        let link = instruction & 1 != 0;
        if self.branch_condition(bo, bi) {
            self.pc = if absolute {
                offset as u32
            } else {
                current.wrapping_add(offset as u32)
            } & !3;
        }
        if link {
            self.lr = current.wrapping_add(4);
        }
    }

    fn branch_condition(&mut self, bo: u8, bi: u8) -> bool {
        let ctr_ok = if bo & 0x04 != 0 {
            true
        } else {
            self.ctr = self.ctr.wrapping_sub(1);
            (self.ctr != 0) ^ (bo & 0x02 != 0)
        };
        let condition_ok = if bo & 0x10 != 0 {
            true
        } else {
            self.cr_bit(bi) == (bo & 0x08 != 0)
        };
        ctr_ok && condition_ok
    }

    fn execute_opcode19(&mut self, instruction: u32, current: u32) {
        let xo = (instruction >> 1) & 0x3ff;
        match xo {
            16 => {
                let target = self.lr & !3;
                let bo = ((instruction >> 21) & 31) as u8;
                let bi = ((instruction >> 16) & 31) as u8;
                let link = instruction & 1 != 0;
                if self.branch_condition(bo, bi) {
                    self.pc = target;
                }
                if link {
                    self.lr = current.wrapping_add(4);
                }
            }
            33 | 129 | 193 | 225 | 257 | 289 | 417 | 449 => self.cr_logical(instruction, xo),
            50 => {
                self.pc = self.srr0 & !3;
                self.msr = self.srr1;
            }
            150 => {}
            528 => {
                let target = self.ctr & !3;
                let bo = ((instruction >> 21) & 31) as u8;
                let bi = ((instruction >> 16) & 31) as u8;
                let link = instruction & 1 != 0;
                if self.branch_condition(bo | 0x04, bi) {
                    self.pc = target;
                }
                if link {
                    self.lr = current.wrapping_add(4);
                }
            }
            _ => self.take_exception(PowerPcException::Program, current),
        }
    }

    fn cr_logical(&mut self, instruction: u32, xo: u32) {
        let bt = ((instruction >> 21) & 31) as u8;
        let ba = ((instruction >> 16) & 31) as u8;
        let bb = ((instruction >> 11) & 31) as u8;
        let a = self.cr_bit(ba);
        let b = self.cr_bit(bb);
        let result = match xo {
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
        self.set_cr_bit(bt, result);
    }

    fn execute_opcode31<B: PowerPcBus>(&mut self, bus: &mut B, instruction: u32) {
        let xo = (instruction >> 1) & 0x3ff;
        match xo {
            0 => self.compare_register(instruction, true),
            4 => self.trap_register(instruction),
            8 => self.subfc(instruction),
            10 => self.addc(instruction),
            19 => self.gpr[Self::rd(instruction)] = self.cr,
            20 => self.load_reserve(bus, instruction),
            23 | 55 | 87 | 119 | 279 | 311 | 343 | 375 => self.load_indexed(bus, instruction, xo),
            24 | 536 | 792 | 824 => self.shift_word(instruction, xo),
            26 => {
                let rd = Self::ra(instruction);
                let value = self.gpr(Self::rd(instruction)).leading_zeros();
                self.gpr[rd] = value;
                if instruction & 1 != 0 {
                    self.record_cr0(value);
                }
            }
            28 | 60 | 124 | 284 | 316 | 412 | 444 | 476 => self.logical_register(instruction, xo),
            32 => self.compare_register(instruction, false),
            40 => self.subf(instruction),
            54 | 86 | 246 | 278 | 470 | 598 | 854 | 982 => {
                self.cache_or_ordering(bus, instruction, xo)
            }
            83 => self.gpr[Self::rd(instruction)] = self.msr,
            136 => self.subfe(instruction),
            138 => self.adde(instruction),
            144 => self.move_to_cr(instruction),
            146 => self.msr = self.gpr(Self::rd(instruction)),
            150 => self.store_conditional(bus, instruction),
            151 | 183 | 215 | 247 | 407 | 439 => self.store_indexed(bus, instruction, xo),
            235 => self.multiply_low(instruction),
            266 => self.add(instruction),
            339 => self.move_from_spr(instruction),
            371 => self.move_from_time_base(instruction),
            459 => self.divide_word(instruction, false),
            467 => self.move_to_spr(instruction),
            491 => self.divide_word(instruction, true),
            534 | 790 => self.load_byte_reverse(bus, instruction, xo),
            662 | 918 => self.store_byte_reverse(bus, instruction, xo),
            922 | 954 => self.extend_sign(instruction, xo),
            1014 => self.cache_zero(bus, instruction),
            _ => self.take_exception(PowerPcException::Program, self.pc.wrapping_sub(4)),
        }
    }

    fn compare_register(&mut self, instruction: u32, signed: bool) {
        let field = ((instruction >> 23) & 7) as usize;
        let ra = Self::ra(instruction);
        let rb = Self::rb(instruction);
        self.set_compare_field(field, self.gpr(ra), self.gpr(rb), signed);
    }

    fn set_compare_field(&mut self, field: usize, lhs: u32, rhs: u32, signed: bool) {
        let relation = if signed {
            (lhs as i32).cmp(&(rhs as i32))
        } else {
            lhs.cmp(&rhs)
        };
        let nibble = match relation {
            std::cmp::Ordering::Less => 0x8,
            std::cmp::Ordering::Greater => 0x4,
            std::cmp::Ordering::Equal => 0x2,
        } | u32::from(self.xer & XER_SO != 0);
        self.set_cr_field(field, nibble);
    }

    fn add(&mut self, instruction: u32) {
        let rd = Self::rd(instruction);
        let ra = Self::ra(instruction);
        let rb = Self::rb(instruction);
        let lhs = self.gpr(ra);
        let rhs = self.gpr(rb);
        let result = lhs.wrapping_add(rhs);
        self.gpr[rd] = result;
        self.update_overflow(instruction, lhs as i32, rhs as i32, result as i32, false);
        self.maybe_record(instruction, result);
    }

    fn addc(&mut self, instruction: u32) {
        let rd = Self::rd(instruction);
        let lhs = self.gpr(Self::ra(instruction));
        let rhs = self.gpr(Self::rb(instruction));
        let (result, carry) = lhs.overflowing_add(rhs);
        self.gpr[rd] = result;
        self.set_ca(carry);
        self.update_overflow(instruction, lhs as i32, rhs as i32, result as i32, false);
        self.maybe_record(instruction, result);
    }

    fn adde(&mut self, instruction: u32) {
        let rd = Self::rd(instruction);
        let lhs = self.gpr(Self::ra(instruction));
        let rhs = self.gpr(Self::rb(instruction));
        let carry_in = u32::from(self.xer & XER_CA != 0);
        let (partial, carry1) = lhs.overflowing_add(rhs);
        let (result, carry2) = partial.overflowing_add(carry_in);
        self.gpr[rd] = result;
        self.set_ca(carry1 || carry2);
        self.maybe_record(instruction, result);
    }

    fn subf(&mut self, instruction: u32) {
        let rd = Self::rd(instruction);
        let lhs = self.gpr(Self::ra(instruction));
        let rhs = self.gpr(Self::rb(instruction));
        let result = rhs.wrapping_sub(lhs);
        self.gpr[rd] = result;
        self.update_overflow(instruction, rhs as i32, lhs as i32, result as i32, true);
        self.maybe_record(instruction, result);
    }

    fn subfc(&mut self, instruction: u32) {
        let rd = Self::rd(instruction);
        let lhs = self.gpr(Self::ra(instruction));
        let rhs = self.gpr(Self::rb(instruction));
        let result = rhs.wrapping_sub(lhs);
        self.gpr[rd] = result;
        self.set_ca(rhs >= lhs);
        self.maybe_record(instruction, result);
    }

    fn subfe(&mut self, instruction: u32) {
        let rd = Self::rd(instruction);
        let lhs = self.gpr(Self::ra(instruction));
        let rhs = self.gpr(Self::rb(instruction));
        let borrow = u32::from(self.xer & XER_CA == 0);
        let (partial, under1) = rhs.overflowing_sub(lhs);
        let (result, under2) = partial.overflowing_sub(borrow);
        self.gpr[rd] = result;
        self.set_ca(!(under1 || under2));
        self.maybe_record(instruction, result);
    }

    fn multiply_low(&mut self, instruction: u32) {
        let rd = Self::rd(instruction);
        let lhs = self.gpr(Self::ra(instruction)) as i32;
        let rhs = self.gpr(Self::rb(instruction)) as i32;
        let result = lhs.wrapping_mul(rhs) as u32;
        self.gpr[rd] = result;
        self.maybe_record(instruction, result);
    }

    fn divide_word(&mut self, instruction: u32, signed: bool) {
        let rd = Self::rd(instruction);
        let lhs = self.gpr(Self::ra(instruction));
        let rhs = self.gpr(Self::rb(instruction));
        let result = if signed {
            (lhs as i32).checked_div(rhs as i32).unwrap_or(0) as u32
        } else {
            lhs.checked_div(rhs).unwrap_or(0)
        };
        self.gpr[rd] = result;
        self.maybe_record(instruction, result);
    }

    fn logical_register(&mut self, instruction: u32, xo: u32) {
        let rs = Self::rd(instruction);
        let ra = Self::ra(instruction);
        let rb = Self::rb(instruction);
        let a = self.gpr(rs);
        let b = self.gpr(rb);
        let result = match xo {
            28 => a & b,
            60 => a & !b,
            124 => !(a | b),
            284 => !(a ^ b),
            316 => a ^ b,
            412 => a | !b,
            444 => a | b,
            476 => !(a & b),
            _ => unreachable!(),
        };
        self.gpr[ra] = result;
        self.maybe_record(instruction, result);
    }

    fn shift_word(&mut self, instruction: u32, xo: u32) {
        let rs = Self::rd(instruction);
        let ra = Self::ra(instruction);
        let rb = Self::rb(instruction);
        let source = self.gpr(rs);
        let (result, carry) = match xo {
            24 => {
                let shift = self.gpr(rb) & 0x3f;
                (if shift >= 32 { 0 } else { source << shift }, false)
            }
            536 => {
                let shift = self.gpr(rb) & 0x3f;
                (if shift >= 32 { 0 } else { source >> shift }, false)
            }
            792 => {
                let shift = self.gpr(rb) & 0x3f;
                let signed = source as i32;
                let result = if shift >= 32 {
                    (signed >> 31) as u32
                } else {
                    (signed >> shift) as u32
                };
                let mask = if shift == 0 {
                    0
                } else if shift >= 32 {
                    u32::MAX
                } else {
                    (1u32 << shift) - 1
                };
                (result, signed < 0 && source & mask != 0)
            }
            824 => {
                let shift = (instruction >> 11) & 31;
                let signed = source as i32;
                let result = (signed >> shift) as u32;
                let mask = if shift == 0 { 0 } else { (1u32 << shift) - 1 };
                (result, signed < 0 && source & mask != 0)
            }
            _ => unreachable!(),
        };
        self.gpr[ra] = result;
        if matches!(xo, 792 | 824) {
            self.set_ca(carry);
        }
        self.maybe_record(instruction, result);
    }

    fn extend_sign(&mut self, instruction: u32, xo: u32) {
        let rs = Self::rd(instruction);
        let ra = Self::ra(instruction);
        let result = if xo == 922 {
            i32::from(self.gpr(rs) as i16) as u32
        } else {
            i32::from(self.gpr(rs) as i8) as u32
        };
        self.gpr[ra] = result;
        self.maybe_record(instruction, result);
    }

    fn trap_immediate(&mut self, instruction: u32) {
        let to = (instruction >> 21) & 31;
        let ra = Self::ra(instruction);
        let lhs = self.gpr(ra);
        let rhs = Self::simm(instruction) as u32;
        if Self::trap_condition(to, lhs, rhs) {
            self.take_exception(PowerPcException::Program, self.pc.wrapping_sub(4));
        }
    }

    fn trap_register(&mut self, instruction: u32) {
        let to = (instruction >> 21) & 31;
        let lhs = self.gpr(Self::ra(instruction));
        let rhs = self.gpr(Self::rb(instruction));
        if Self::trap_condition(to, lhs, rhs) {
            self.take_exception(PowerPcException::Program, self.pc.wrapping_sub(4));
        }
    }

    fn trap_condition(to: u32, lhs: u32, rhs: u32) -> bool {
        (to & 0x10 != 0 && (lhs as i32) < (rhs as i32))
            || (to & 0x08 != 0 && (lhs as i32) > (rhs as i32))
            || (to & 0x04 != 0 && lhs == rhs)
            || (to & 0x02 != 0 && lhs < rhs)
            || (to & 0x01 != 0 && lhs > rhs)
    }

    fn load_store_immediate<B: PowerPcBus>(&mut self, bus: &mut B, instruction: u32, opcode: u32) {
        let rd = Self::rd(instruction);
        let ra = Self::ra(instruction);
        let address = self.base(ra).wrapping_add(Self::simm(instruction) as u32);
        match opcode {
            32 | 33 => {
                if !self.aligned(address, 4, false) {
                    return;
                }
                self.gpr[rd] = bus.read32(address);
                if opcode == 33 {
                    self.update_base(ra, address);
                }
            }
            34 | 35 => {
                self.gpr[rd] = u32::from(bus.read8(address));
                if opcode == 35 {
                    self.update_base(ra, address);
                }
            }
            36 | 37 => {
                if !self.aligned(address, 4, true) {
                    return;
                }
                bus.write32(address, self.gpr(rd));
                self.reservation = None;
                if opcode == 37 {
                    self.update_base(ra, address);
                }
            }
            38 | 39 => {
                bus.write8(address, self.gpr(rd) as u8);
                self.reservation = None;
                if opcode == 39 {
                    self.update_base(ra, address);
                }
            }
            40..=43 => {
                if !self.aligned(address, 2, false) {
                    return;
                }
                let value = bus.read16(address);
                self.gpr[rd] = if matches!(opcode, 42 | 43) {
                    i32::from(value as i16) as u32
                } else {
                    u32::from(value)
                };
                if matches!(opcode, 41 | 43) {
                    self.update_base(ra, address);
                }
            }
            44 | 45 => {
                if !self.aligned(address, 2, true) {
                    return;
                }
                bus.write16(address, self.gpr(rd) as u16);
                self.reservation = None;
                if opcode == 45 {
                    self.update_base(ra, address);
                }
            }
            46 => {
                if !self.aligned(address, 4, false) {
                    return;
                }
                let mut cursor = address;
                for register in rd..32 {
                    self.gpr[register] = bus.read32(cursor);
                    cursor = cursor.wrapping_add(4);
                }
            }
            47 => {
                if !self.aligned(address, 4, true) {
                    return;
                }
                let mut cursor = address;
                for register in rd..32 {
                    bus.write32(cursor, self.gpr(register));
                    cursor = cursor.wrapping_add(4);
                }
                self.reservation = None;
            }
            _ => unreachable!(),
        }
    }

    fn load_store_float<B: PowerPcBus>(&mut self, bus: &mut B, instruction: u32, opcode: u32) {
        if self.msr & MSR_FP == 0 {
            self.take_exception(
                PowerPcException::FloatingPointUnavailable,
                self.pc.wrapping_sub(4),
            );
            return;
        }
        let fr = Self::rd(instruction);
        let ra = Self::ra(instruction);
        let address = self.base(ra).wrapping_add(Self::simm(instruction) as u32);
        match opcode {
            48 | 49 => {
                if !self.aligned(address, 4, false) {
                    return;
                }
                self.fpr[fr] = f64::from(f32::from_bits(bus.read32(address))).to_bits();
                if opcode == 49 {
                    self.update_base(ra, address);
                }
            }
            50 | 51 => {
                if !self.aligned(address, 8, false) {
                    return;
                }
                self.fpr[fr] = bus.read64(address);
                if opcode == 51 {
                    self.update_base(ra, address);
                }
            }
            52 | 53 => {
                if !self.aligned(address, 4, true) {
                    return;
                }
                bus.write32(address, (f64::from_bits(self.fpr[fr]) as f32).to_bits());
                self.reservation = None;
                if opcode == 53 {
                    self.update_base(ra, address);
                }
            }
            54 | 55 => {
                if !self.aligned(address, 8, true) {
                    return;
                }
                bus.write64(address, self.fpr[fr]);
                self.reservation = None;
                if opcode == 55 {
                    self.update_base(ra, address);
                }
            }
            _ => unreachable!(),
        }
    }

    fn load_indexed<B: PowerPcBus>(&mut self, bus: &mut B, instruction: u32, xo: u32) {
        let rd = Self::rd(instruction);
        let ra = Self::ra(instruction);
        let address = self.base(ra).wrapping_add(self.gpr(Self::rb(instruction)));
        match xo {
            23 | 55 => {
                if !self.aligned(address, 4, false) {
                    return;
                }
                self.gpr[rd] = bus.read32(address);
            }
            87 | 119 => self.gpr[rd] = u32::from(bus.read8(address)),
            279 | 311 => {
                if !self.aligned(address, 2, false) {
                    return;
                }
                self.gpr[rd] = u32::from(bus.read16(address));
            }
            343 | 375 => {
                if !self.aligned(address, 2, false) {
                    return;
                }
                self.gpr[rd] = i32::from(bus.read16(address) as i16) as u32;
            }
            _ => unreachable!(),
        }
        if matches!(xo, 55 | 119 | 311 | 375) {
            self.update_base(ra, address);
        }
    }

    fn store_indexed<B: PowerPcBus>(&mut self, bus: &mut B, instruction: u32, xo: u32) {
        let rs = Self::rd(instruction);
        let ra = Self::ra(instruction);
        let address = self.base(ra).wrapping_add(self.gpr(Self::rb(instruction)));
        match xo {
            151 | 183 => {
                if !self.aligned(address, 4, true) {
                    return;
                }
                bus.write32(address, self.gpr(rs));
            }
            215 | 247 => bus.write8(address, self.gpr(rs) as u8),
            407 | 439 => {
                if !self.aligned(address, 2, true) {
                    return;
                }
                bus.write16(address, self.gpr(rs) as u16);
            }
            _ => unreachable!(),
        }
        self.reservation = None;
        if matches!(xo, 183 | 247 | 439) {
            self.update_base(ra, address);
        }
    }

    fn load_byte_reverse<B: PowerPcBus>(&mut self, bus: &mut B, instruction: u32, xo: u32) {
        let rd = Self::rd(instruction);
        let address = self
            .base(Self::ra(instruction))
            .wrapping_add(self.gpr(Self::rb(instruction)));
        self.gpr[rd] = if xo == 534 {
            if !self.aligned(address, 4, false) {
                return;
            }
            bus.read32(address).swap_bytes()
        } else {
            if !self.aligned(address, 2, false) {
                return;
            }
            u32::from(bus.read16(address).swap_bytes())
        };
    }

    fn store_byte_reverse<B: PowerPcBus>(&mut self, bus: &mut B, instruction: u32, xo: u32) {
        let rs = Self::rd(instruction);
        let address = self
            .base(Self::ra(instruction))
            .wrapping_add(self.gpr(Self::rb(instruction)));
        if xo == 662 {
            if !self.aligned(address, 4, true) {
                return;
            }
            bus.write32(address, self.gpr(rs).swap_bytes());
        } else {
            if !self.aligned(address, 2, true) {
                return;
            }
            bus.write16(address, (self.gpr(rs) as u16).swap_bytes());
        }
        self.reservation = None;
    }

    fn load_reserve<B: PowerPcBus>(&mut self, bus: &mut B, instruction: u32) {
        let rd = Self::rd(instruction);
        let address = self
            .base(Self::ra(instruction))
            .wrapping_add(self.gpr(Self::rb(instruction)));
        if !self.aligned(address, 4, false) {
            return;
        }
        self.gpr[rd] = bus.read32(address);
        self.reservation = Some(address & !31);
    }

    fn store_conditional<B: PowerPcBus>(&mut self, bus: &mut B, instruction: u32) {
        let rs = Self::rd(instruction);
        let address = self
            .base(Self::ra(instruction))
            .wrapping_add(self.gpr(Self::rb(instruction)));
        if !self.aligned(address, 4, true) {
            return;
        }
        let success = self.reservation == Some(address & !31);
        self.reservation = None;
        if success {
            bus.write32(address, self.gpr(rs));
        }
        self.set_cr_field(
            0,
            if success { 0x2 } else { 0 } | u32::from(self.xer & XER_SO != 0),
        );
    }

    fn move_from_spr(&mut self, instruction: u32) {
        let rd = Self::rd(instruction);
        let spr = Self::decode_spr(instruction);
        self.gpr[rd] = match spr {
            1 => self.xer,
            8 => self.lr,
            9 => self.ctr,
            18 => self.dsisr,
            19 => self.dar,
            22 => self.decrementer,
            26 => self.srr0,
            27 => self.srr1,
            912..=919 => self.gqr[usize::from(spr - 912)],
            920 => self.hid2,
            1008 => self.hid0,
            _ => 0,
        };
    }

    fn move_to_spr(&mut self, instruction: u32) {
        let rs = Self::rd(instruction);
        let value = self.gpr(rs);
        match Self::decode_spr(instruction) {
            1 => self.xer = value,
            8 => self.lr = value,
            9 => self.ctr = value,
            22 => self.decrementer = value,
            26 => self.srr0 = value,
            27 => self.srr1 = value,
            912..=919 => self.gqr[usize::from(Self::decode_spr(instruction) - 912)] = value,
            920 => self.hid2 = value,
            1008 => self.hid0 = value,
            _ => {}
        }
    }

    fn move_from_time_base(&mut self, instruction: u32) {
        let rd = Self::rd(instruction);
        self.gpr[rd] = match Self::decode_spr(instruction) {
            268 => self.time_base as u32,
            269 => (self.time_base >> 32) as u32,
            _ => 0,
        };
    }

    fn decode_spr(instruction: u32) -> u16 {
        let encoded = ((instruction >> 11) & 0x3ff) as u16;
        ((encoded & 0x1f) << 5) | ((encoded >> 5) & 0x1f)
    }

    fn move_to_cr(&mut self, instruction: u32) {
        let rs = Self::rd(instruction);
        let mask = ((instruction >> 12) & 0xff) as u8;
        for field in 0..8 {
            if mask & (1 << (7 - field)) != 0 {
                let shift = (7 - field) * 4;
                self.cr = (self.cr & !(0xf << shift)) | (self.gpr(rs) & (0xf << shift));
            }
        }
    }

    fn cache_or_ordering<B: PowerPcBus>(&mut self, _bus: &mut B, _instruction: u32, _xo: u32) {
        // Cache maintenance and ordering are architecturally visible but do not need a host cache
        // operation in the reference interpreter. Machine memory/device ordering remains explicit.
    }

    fn cache_zero<B: PowerPcBus>(&mut self, bus: &mut B, instruction: u32) {
        let address = self
            .base(Self::ra(instruction))
            .wrapping_add(self.gpr(Self::rb(instruction)))
            & !31;
        for offset in (0..32).step_by(4) {
            bus.write32(address.wrapping_add(offset), 0);
        }
        self.reservation = None;
    }

    fn require_paired(&mut self, quantized: bool) -> bool {
        if !self.require_fpu() {
            return false;
        }
        if self.hid2 & HID2_PSE == 0 || (quantized && self.hid2 & HID2_LSQE == 0) {
            self.take_exception(PowerPcException::Program, self.pc.wrapping_sub(4));
            return false;
        }
        true
    }

    fn paired(&self, index: usize) -> (f64, f64) {
        (
            f64::from_bits(self.fpr[index & 31]),
            f64::from_bits(self.paired1[index & 31]),
        )
    }

    fn set_paired(&mut self, index: usize, first: f64, second: f64) {
        self.fpr[index & 31] = f64::from(first as f32).to_bits();
        self.paired1[index & 31] = f64::from(second as f32).to_bits();
    }

    fn set_paired_raw(&mut self, index: usize, first: u64, second: u64) {
        self.fpr[index & 31] = first;
        self.paired1[index & 31] = second;
    }

    fn update_fp_exception_summary(&mut self) {
        if self.fpscr & FPSCR_VX_ANY != 0 {
            self.fpscr |= FPSCR_VX;
        } else {
            self.fpscr &= !FPSCR_VX;
        }
        let enabled = (self.fpscr & FPSCR_VX != 0 && self.fpscr & FPSCR_VE != 0)
            || (self.fpscr & FPSCR_OX != 0 && self.fpscr & FPSCR_OE != 0)
            || (self.fpscr & FPSCR_UX != 0 && self.fpscr & FPSCR_UE != 0)
            || (self.fpscr & FPSCR_ZX != 0 && self.fpscr & FPSCR_ZE != 0)
            || (self.fpscr & FPSCR_XX != 0 && self.fpscr & FPSCR_XE != 0);
        if enabled {
            self.fpscr |= FPSCR_FEX;
        } else {
            self.fpscr &= !FPSCR_FEX;
        }
    }

    fn set_fp_exception(&mut self, mask: u32) {
        if self.fpscr & mask != mask {
            self.fpscr |= FPSCR_FX;
        }
        self.fpscr |= mask;
        self.update_fp_exception_summary();
    }

    fn clear_fifr(&mut self) {
        self.fpscr &= !(FPSCR_FR | FPSCR_FI);
    }

    fn force_single(&self, value: f64) -> f32 {
        let magnitude = value.to_bits() & !DOUBLE_SIGN;
        if self.fpscr & FPSCR_NI != 0 && magnitude < 0x3810_0000_0000_0000 {
            return f32::from_bits(if value.is_sign_negative() {
                0x8000_0000
            } else {
                0
            });
        }

        let nearest = value as f32;
        if value.is_nan() || value.is_infinite() || f64::from(nearest) == value {
            return nearest;
        }
        let nearest64 = f64::from(nearest);
        let rounded = match self.fpscr & 3 {
            0 => nearest,
            1 => {
                if value.is_sign_positive() && nearest64 > value {
                    next_down_f32(nearest)
                } else if value.is_sign_negative() && nearest64 < value {
                    next_up_f32(nearest)
                } else {
                    nearest
                }
            }
            2 => {
                if nearest64 < value {
                    next_up_f32(nearest)
                } else {
                    nearest
                }
            }
            _ => {
                if nearest64 > value {
                    next_down_f32(nearest)
                } else {
                    nearest
                }
            }
        };
        if self.fpscr & FPSCR_NI != 0
            && rounded.to_bits() & 0x7f80_0000 == 0
            && rounded.to_bits() & 0x007f_ffff != 0
        {
            f32::from_bits(rounded.to_bits() & 0x8000_0000)
        } else {
            rounded
        }
    }

    fn set_paired_single(&mut self, index: usize, first: f32, second: f32) {
        self.fpr[index & 31] = f64::from(first).to_bits();
        self.paired1[index & 31] = f64::from(second).to_bits();
    }

    fn trap_invalid_if_enabled(&mut self, invalid: bool) -> bool {
        if invalid && self.fpscr & FPSCR_VE != 0 {
            self.take_exception(PowerPcException::Program, self.pc.wrapping_sub(4));
            true
        } else {
            false
        }
    }

    fn fp_add_result(&mut self, left: f64, right: f64) -> FpArithmeticResult {
        let mut invalid = false;
        if is_signaling_nan(left) || is_signaling_nan(right) {
            self.set_fp_exception(FPSCR_VXSNAN);
            invalid = true;
        }
        if left.is_nan() {
            self.clear_fifr();
            return FpArithmeticResult {
                value: quiet_nan(left),
                invalid,
            };
        }
        if right.is_nan() {
            self.clear_fifr();
            return FpArithmeticResult {
                value: quiet_nan(right),
                invalid,
            };
        }
        let value = left + right;
        if value.is_nan() {
            self.set_fp_exception(FPSCR_VXISI);
            self.clear_fifr();
            return FpArithmeticResult {
                value: f64::from_bits(CANONICAL_QNAN),
                invalid: true,
            };
        }
        if left.is_infinite() || right.is_infinite() {
            self.clear_fifr();
        }
        FpArithmeticResult { value, invalid }
    }

    fn fp_sub_result(&mut self, left: f64, right: f64) -> FpArithmeticResult {
        let mut invalid = false;
        if is_signaling_nan(left) || is_signaling_nan(right) {
            self.set_fp_exception(FPSCR_VXSNAN);
            invalid = true;
        }
        if left.is_nan() {
            self.clear_fifr();
            return FpArithmeticResult {
                value: quiet_nan(left),
                invalid,
            };
        }
        if right.is_nan() {
            self.clear_fifr();
            return FpArithmeticResult {
                value: quiet_nan(right),
                invalid,
            };
        }
        let value = left - right;
        if value.is_nan() {
            self.set_fp_exception(FPSCR_VXISI);
            self.clear_fifr();
            return FpArithmeticResult {
                value: f64::from_bits(CANONICAL_QNAN),
                invalid: true,
            };
        }
        if left.is_infinite() || right.is_infinite() {
            self.clear_fifr();
        }
        FpArithmeticResult { value, invalid }
    }

    fn fp_mul_result(&mut self, left: f64, right: f64) -> FpArithmeticResult {
        let mut invalid = false;
        if is_signaling_nan(left) || is_signaling_nan(right) {
            self.set_fp_exception(FPSCR_VXSNAN);
            invalid = true;
        }
        if left.is_nan() {
            self.clear_fifr();
            return FpArithmeticResult {
                value: quiet_nan(left),
                invalid,
            };
        }
        if right.is_nan() {
            self.clear_fifr();
            return FpArithmeticResult {
                value: quiet_nan(right),
                invalid,
            };
        }
        let value = left * right;
        if value.is_nan() {
            self.set_fp_exception(FPSCR_VXIMZ);
            self.clear_fifr();
            return FpArithmeticResult {
                value: f64::from_bits(CANONICAL_QNAN),
                invalid: true,
            };
        }
        FpArithmeticResult { value, invalid }
    }

    fn fp_single_mul_result(&mut self, left: f64, right: f64) -> FpArithmeticResult {
        let mut invalid = false;
        if is_signaling_nan(left) || is_signaling_nan(right) {
            self.set_fp_exception(FPSCR_VXSNAN);
            invalid = true;
        }
        if left.is_nan() {
            self.clear_fifr();
            return FpArithmeticResult {
                value: quiet_nan(left),
                invalid,
            };
        }
        if right.is_nan() {
            self.clear_fifr();
            return FpArithmeticResult {
                value: quiet_nan(right),
                invalid,
            };
        }
        let value = left * force_25_bit(right);
        if value.is_nan() {
            self.set_fp_exception(FPSCR_VXIMZ);
            self.clear_fifr();
            return FpArithmeticResult {
                value: f64::from_bits(CANONICAL_QNAN),
                invalid: true,
            };
        }
        FpArithmeticResult { value, invalid }
    }

    fn fp_div_result(&mut self, left: f64, right: f64) -> (FpArithmeticResult, bool) {
        let mut invalid = false;
        if is_signaling_nan(left) || is_signaling_nan(right) {
            self.set_fp_exception(FPSCR_VXSNAN);
            invalid = true;
        }
        if left.is_nan() {
            self.clear_fifr();
            return (
                FpArithmeticResult {
                    value: quiet_nan(left),
                    invalid,
                },
                false,
            );
        }
        if right.is_nan() {
            self.clear_fifr();
            return (
                FpArithmeticResult {
                    value: quiet_nan(right),
                    invalid,
                },
                false,
            );
        }
        if right == 0.0 {
            if left == 0.0 {
                self.set_fp_exception(FPSCR_VXZDZ);
                self.clear_fifr();
                return (
                    FpArithmeticResult {
                        value: f64::from_bits(CANONICAL_QNAN),
                        invalid: true,
                    },
                    false,
                );
            }
            self.set_fp_exception(FPSCR_ZX);
            return (
                FpArithmeticResult {
                    value: left / right,
                    invalid,
                },
                true,
            );
        }
        if left.is_infinite() && right.is_infinite() {
            self.set_fp_exception(FPSCR_VXIDI);
            self.clear_fifr();
            return (
                FpArithmeticResult {
                    value: f64::from_bits(CANONICAL_QNAN),
                    invalid: true,
                },
                false,
            );
        }
        (
            FpArithmeticResult {
                value: left / right,
                invalid,
            },
            false,
        )
    }

    fn fp_madd_result(
        &mut self,
        a: f64,
        c: f64,
        b: f64,
        subtract: bool,
        single: bool,
    ) -> FpArithmeticResult {
        let mut invalid = false;
        if is_signaling_nan(a) || is_signaling_nan(b) || is_signaling_nan(c) {
            self.set_fp_exception(FPSCR_VXSNAN);
            invalid = true;
        }
        if a.is_nan() {
            self.clear_fifr();
            return FpArithmeticResult {
                value: quiet_nan(a),
                invalid,
            };
        }
        if b.is_nan() {
            self.clear_fifr();
            return FpArithmeticResult {
                value: quiet_nan(b),
                invalid,
            };
        }
        if c.is_nan() {
            self.clear_fifr();
            return FpArithmeticResult {
                value: quiet_nan(c),
                invalid,
            };
        }

        let c_effective = if single { force_25_bit(c) } else { c };
        let addend = if subtract { -b } else { b };
        let mut value = a.mul_add(c_effective, addend);

        if single && value.is_finite() {
            let bits = value.to_bits();
            if bits & 0x1fff_ffff == 0x1000_0000 {
                let product_side = addend - value;
                let recovered_addend = value + product_side;
                let product_error = a.mul_add(c_effective, product_side);
                let addend_error = addend - recovered_addend;
                let residual = product_error + addend_error;
                if residual != 0.0 {
                    value = f64::from_bits(if (residual > 0.0) == (value > 0.0) {
                        bits + 1
                    } else {
                        bits - 1
                    });
                }
            }
        }

        if value.is_nan() {
            self.clear_fifr();
            let product_invalid = (a.is_infinite() && c == 0.0) || (c.is_infinite() && a == 0.0);
            self.set_fp_exception(if product_invalid {
                FPSCR_VXIMZ
            } else {
                FPSCR_VXISI
            });
            return FpArithmeticResult {
                value: f64::from_bits(CANONICAL_QNAN),
                invalid: true,
            };
        }
        if a.is_infinite() || b.is_infinite() || c.is_infinite() {
            self.clear_fifr();
        }
        FpArithmeticResult { value, invalid }
    }

    fn update_fprf_double(&mut self, value: f64) {
        self.fpscr = (self.fpscr & !FPSCR_FPRF_MASK) | (classify_f64(value) << 12);
    }

    fn update_fprf_single(&mut self, value: f32) {
        self.fpscr = (self.fpscr & !FPSCR_FPRF_MASK) | (classify_f32(value) << 12);
    }

    fn update_fpcc(&mut self, value: u32) {
        self.fpscr = (self.fpscr & !FPSCR_FPCC_MASK) | ((value & 0xf) << 12);
    }

    fn record_fp_rc(&mut self, instruction: u32) {
        if instruction & 1 != 0 {
            self.set_cr_field(1, (self.fpscr >> 28) & 0xf);
        }
    }

    fn paired_record(&mut self, instruction: u32) {
        self.record_fp_rc(instruction);
    }

    fn quant_scale(scale: u32, load: bool) -> f64 {
        let exponent = if scale < 32 {
            if load {
                -(scale as i32)
            } else {
                scale as i32
            }
        } else if load {
            (64 - scale) as i32
        } else {
            scale as i32 - 64
        };
        2.0f64.powi(exponent)
    }

    fn load_quantized<B: PowerPcBus>(
        &mut self,
        bus: &mut B,
        rd: usize,
        address: u32,
        gqr_index: usize,
        scalar: bool,
    ) {
        let gqr = self.gqr[gqr_index & 7];
        let kind = (gqr >> 16) & 7;
        let scale = (gqr >> 24) & 0x3f;
        let factor = Self::quant_scale(scale, true);
        let pair = match kind {
            0 => {
                let first = f64::from(f32::from_bits(bus.read32(address)));
                let second = if scalar {
                    1.0
                } else {
                    f64::from(f32::from_bits(bus.read32(address.wrapping_add(4))))
                };
                (first, second)
            }
            4 => {
                let first = f64::from(bus.read8(address)) * factor;
                let second = if scalar {
                    1.0
                } else {
                    f64::from(bus.read8(address.wrapping_add(1))) * factor
                };
                (first, second)
            }
            5 => {
                let first = f64::from(bus.read16(address)) * factor;
                let second = if scalar {
                    1.0
                } else {
                    f64::from(bus.read16(address.wrapping_add(2))) * factor
                };
                (first, second)
            }
            6 => {
                let first = f64::from(bus.read8(address) as i8) * factor;
                let second = if scalar {
                    1.0
                } else {
                    f64::from(bus.read8(address.wrapping_add(1)) as i8) * factor
                };
                (first, second)
            }
            7 => {
                let first = f64::from(bus.read16(address) as i16) * factor;
                let second = if scalar {
                    1.0
                } else {
                    f64::from(bus.read16(address.wrapping_add(2)) as i16) * factor
                };
                (first, second)
            }
            _ => {
                self.take_exception(PowerPcException::Program, self.pc.wrapping_sub(4));
                return;
            }
        };
        self.set_paired(rd, pair.0, pair.1);
    }

    fn clamp_quantized(value: f64, factor: f64, min: f64, max: f64) -> f64 {
        (f64::from(value as f32) * factor).clamp(min, max).trunc()
    }

    fn store_quantized<B: PowerPcBus>(
        &mut self,
        bus: &mut B,
        rs: usize,
        address: u32,
        gqr_index: usize,
        scalar: bool,
    ) {
        let gqr = self.gqr[gqr_index & 7];
        let kind = gqr & 7;
        let scale = (gqr >> 8) & 0x3f;
        let factor = Self::quant_scale(scale, false);
        let (first, second) = self.paired(rs);
        match kind {
            0 => {
                bus.write32(address, (first as f32).to_bits());
                if !scalar {
                    bus.write32(address.wrapping_add(4), (second as f32).to_bits());
                }
            }
            4 => {
                bus.write8(
                    address,
                    Self::clamp_quantized(first, factor, 0.0, 255.0) as u8,
                );
                if !scalar {
                    bus.write8(
                        address.wrapping_add(1),
                        Self::clamp_quantized(second, factor, 0.0, 255.0) as u8,
                    );
                }
            }
            5 => {
                bus.write16(
                    address,
                    Self::clamp_quantized(first, factor, 0.0, 65535.0) as u16,
                );
                if !scalar {
                    bus.write16(
                        address.wrapping_add(2),
                        Self::clamp_quantized(second, factor, 0.0, 65535.0) as u16,
                    );
                }
            }
            6 => {
                bus.write8(
                    address,
                    Self::clamp_quantized(first, factor, -128.0, 127.0) as i8 as u8,
                );
                if !scalar {
                    bus.write8(
                        address.wrapping_add(1),
                        Self::clamp_quantized(second, factor, -128.0, 127.0) as i8 as u8,
                    );
                }
            }
            7 => {
                bus.write16(
                    address,
                    Self::clamp_quantized(first, factor, -32768.0, 32767.0) as i16 as u16,
                );
                if !scalar {
                    bus.write16(
                        address.wrapping_add(2),
                        Self::clamp_quantized(second, factor, -32768.0, 32767.0) as i16 as u16,
                    );
                }
            }
            _ => self.take_exception(PowerPcException::Program, self.pc.wrapping_sub(4)),
        }
    }

    fn load_store_paired<B: PowerPcBus>(&mut self, bus: &mut B, instruction: u32, opcode: u32) {
        if !self.require_paired(true) {
            return;
        }
        let reg = Self::rd(instruction);
        let ra = Self::ra(instruction);
        let scalar = instruction & (1 << 15) != 0;
        let gqr = ((instruction >> 12) & 7) as usize;
        let displacement = ((instruction << 20) as i32 >> 20) as u32;
        let update = matches!(opcode, 57 | 61);
        if update && ra == 0 {
            self.take_exception(PowerPcException::Program, self.pc.wrapping_sub(4));
            return;
        }
        let address = self.base(ra).wrapping_add(displacement);
        if matches!(opcode, 56 | 57) {
            self.load_quantized(bus, reg, address, gqr, scalar);
        } else {
            self.store_quantized(bus, reg, address, gqr, scalar);
        }
        if update && !self.exception_raised {
            self.gpr[ra] = address;
        }
    }

    fn indexed_paired<B: PowerPcBus>(&mut self, bus: &mut B, instruction: u32, subop: u32) {
        if !self.require_paired(true) {
            return;
        }
        let reg = Self::rd(instruction);
        let ra = Self::ra(instruction);
        let rb = Self::rb(instruction);
        let scalar = instruction & (1 << 10) != 0;
        let gqr = ((instruction >> 7) & 7) as usize;
        let update = matches!(subop, 38 | 39);
        if update && ra == 0 {
            self.take_exception(PowerPcException::Program, self.pc.wrapping_sub(4));
            return;
        }
        let address = self.base(ra).wrapping_add(self.gpr(rb));
        if matches!(subop, 6 | 38) {
            self.load_quantized(bus, reg, address, gqr, scalar);
        } else {
            self.store_quantized(bus, reg, address, gqr, scalar);
        }
        if update && !self.exception_raised {
            self.gpr[ra] = address;
        }
    }

    fn execute_paired_single<B: PowerPcBus>(&mut self, bus: &mut B, instruction: u32) {
        let sub10 = (instruction >> 1) & 0x3ff;
        if sub10 == 1014 {
            self.cache_zero(bus, instruction);
            return;
        }
        let sub6 = (instruction >> 1) & 0x3f;
        if matches!(sub6, 6 | 7 | 38 | 39) {
            self.indexed_paired(bus, instruction, sub6);
            return;
        }
        if !self.require_paired(false) {
            return;
        }
        if self.execute_paired_special(instruction, sub10) {
            return;
        }
        self.execute_paired_arithmetic(instruction, (instruction >> 1) & 0x1f);
    }

    fn paired_compare(&mut self, instruction: u32, lane: usize, ordered: bool) {
        let field = ((instruction >> 23) & 7) as usize;
        let (a0, a1) = self.paired(Self::ra(instruction));
        let (b0, b1) = self.paired(Self::rb(instruction));
        let (a, b) = if lane == 0 { (a0, b0) } else { (a1, b1) };
        let nibble = if a.is_nan() || b.is_nan() {
            let signaling = is_signaling_nan(a) || is_signaling_nan(b);
            if signaling {
                self.set_fp_exception(FPSCR_VXSNAN);
            }
            if ordered && (!signaling || self.fpscr & FPSCR_VE == 0) {
                self.set_fp_exception(FPSCR_VXVC);
            }
            0x1
        } else if a < b {
            0x8
        } else if a > b {
            0x4
        } else {
            0x2
        };
        self.update_fpcc(nibble);
        self.set_cr_field(field, nibble);
    }

    fn execute_paired_special(&mut self, instruction: u32, sub10: u32) -> bool {
        let fd = Self::rd(instruction);
        let fa = Self::ra(instruction);
        let fb = Self::rb(instruction);
        match sub10 {
            0 => self.paired_compare(instruction, 0, false),
            32 => self.paired_compare(instruction, 0, true),
            64 => self.paired_compare(instruction, 1, false),
            96 => self.paired_compare(instruction, 1, true),
            40 => {
                self.set_paired_raw(fd, self.fpr[fb] ^ (1 << 63), self.paired1[fb] ^ (1 << 63));
                self.paired_record(instruction);
            }
            72 => {
                self.set_paired_raw(fd, self.fpr[fb], self.paired1[fb]);
                self.paired_record(instruction);
            }
            136 => {
                self.set_paired_raw(fd, self.fpr[fb] | (1 << 63), self.paired1[fb] | (1 << 63));
                self.paired_record(instruction);
            }
            264 => {
                self.set_paired_raw(fd, self.fpr[fb] & !(1 << 63), self.paired1[fb] & !(1 << 63));
                self.paired_record(instruction);
            }
            528 | 560 | 592 | 624 => {
                let (a0, a1) = (self.fpr[fa], self.paired1[fa]);
                let (b0, b1) = (self.fpr[fb], self.paired1[fb]);
                let pair = match sub10 {
                    528 => (a0, b0),
                    560 => (a0, b1),
                    592 => (a1, b0),
                    _ => (a1, b1),
                };
                self.set_paired_raw(fd, pair.0, pair.1);
                self.paired_record(instruction);
            }
            _ => return false,
        }
        true
    }

    fn execute_paired_arithmetic(&mut self, instruction: u32, sub5: u32) {
        let fd = Self::rd(instruction);
        let fa = Self::ra(instruction);
        let fb = Self::rb(instruction);
        let fc = ((instruction >> 6) & 31) as usize;
        let (a0, a1) = self.paired(fa);
        let (b0, b1) = self.paired(fb);
        let (c0, c1) = self.paired(fc);

        if sub5 == 23 {
            let first = if a0 >= -0.0 { c0 } else { b0 };
            let second = if a1 >= -0.0 { c1 } else { b1 };
            self.set_paired_raw(fd, first.to_bits(), second.to_bits());
            self.paired_record(instruction);
            return;
        }

        if sub5 == 24 {
            if b0 == 0.0 || b1 == 0.0 {
                self.set_fp_exception(FPSCR_ZX);
                self.clear_fifr();
            }
            if b0.is_nan() || b0.is_infinite() || b1.is_nan() || b1.is_infinite() {
                self.clear_fifr();
            }
            if is_signaling_nan(b0) || is_signaling_nan(b1) {
                self.set_fp_exception(FPSCR_VXSNAN);
            }
            let first = gekko_reciprocal_estimate(b0);
            let second = gekko_reciprocal_estimate(b1);
            self.set_paired_raw(fd, first.to_bits(), second.to_bits());
            self.update_fprf_single(first as f32);
            self.paired_record(instruction);
            return;
        }

        let pair = match sub5 {
            10 => {
                let sum = self.fp_add_result(a0, b1);
                (self.force_single(sum.value), self.force_single(c1))
            }
            11 => {
                let sum = self.fp_add_result(a0, b1);
                (self.force_single(c0), self.force_single(sum.value))
            }
            12 | 13 => {
                let scalar = if sub5 == 12 { c0 } else { c1 };
                let first = self.fp_single_mul_result(a0, scalar);
                let second = self.fp_single_mul_result(a1, scalar);
                (
                    self.force_single(first.value),
                    self.force_single(second.value),
                )
            }
            14 | 15 => {
                let scalar = if sub5 == 14 { c0 } else { c1 };
                let first = self.fp_madd_result(a0, scalar, b0, false, true);
                let second = self.fp_madd_result(a1, scalar, b1, false, true);
                (
                    self.force_single(first.value),
                    self.force_single(second.value),
                )
            }
            18 => {
                let (first, _) = self.fp_div_result(a0, b0);
                let (second, _) = self.fp_div_result(a1, b1);
                (
                    self.force_single(first.value),
                    self.force_single(second.value),
                )
            }
            20 => {
                let first = self.fp_sub_result(a0, b0);
                let second = self.fp_sub_result(a1, b1);
                (
                    self.force_single(first.value),
                    self.force_single(second.value),
                )
            }
            21 => {
                let first = self.fp_add_result(a0, b0);
                let second = self.fp_add_result(a1, b1);
                (
                    self.force_single(first.value),
                    self.force_single(second.value),
                )
            }
            25 => {
                let first = self.fp_single_mul_result(a0, c0);
                let second = self.fp_single_mul_result(a1, c1);
                (
                    self.force_single(first.value),
                    self.force_single(second.value),
                )
            }
            26 => {
                if b0 == 0.0 || b1 == 0.0 {
                    self.set_fp_exception(FPSCR_ZX);
                    self.clear_fifr();
                }
                if b0 < 0.0 || b1 < 0.0 {
                    self.set_fp_exception(FPSCR_VXSQRT);
                    self.clear_fifr();
                }
                if b0.is_nan() || b0.is_infinite() || b1.is_nan() || b1.is_infinite() {
                    self.clear_fifr();
                }
                if is_signaling_nan(b0) || is_signaling_nan(b1) {
                    self.set_fp_exception(FPSCR_VXSNAN);
                }
                (
                    self.force_single(gekko_reciprocal_sqrt_estimate(b0)),
                    self.force_single(gekko_reciprocal_sqrt_estimate(b1)),
                )
            }
            28..=31 => {
                let subtract = matches!(sub5, 28 | 30);
                let first = self.fp_madd_result(a0, c0, b0, subtract, true);
                let second = self.fp_madd_result(a1, c1, b1, subtract, true);
                let first_value = if matches!(sub5, 30 | 31) && !first.value.is_nan() {
                    -first.value
                } else {
                    first.value
                };
                let second_value = if matches!(sub5, 30 | 31) && !second.value.is_nan() {
                    -second.value
                } else {
                    second.value
                };
                (
                    self.force_single(first_value),
                    self.force_single(second_value),
                )
            }
            _ => {
                self.take_exception(PowerPcException::Program, self.pc.wrapping_sub(4));
                return;
            }
        };

        self.set_paired_single(fd, pair.0, pair.1);
        let reported = if sub5 == 11 { pair.1 } else { pair.0 };
        self.update_fprf_single(reported);
        self.paired_record(instruction);
    }

    fn execute_float_single(&mut self, instruction: u32) {
        if !self.require_fpu() {
            return;
        }
        let fd = Self::rd(instruction);
        let fa = Self::ra(instruction);
        let fb = Self::rb(instruction);
        let fc = ((instruction >> 6) & 31) as usize;
        let xo = (instruction >> 1) & 31;
        let b_full = f64::from_bits(self.fpr[fb]);
        if xo == 24 {
            if b_full == 0.0 {
                self.set_fp_exception(FPSCR_ZX);
                self.clear_fifr();
                if self.fpscr & FPSCR_ZE != 0 {
                    self.take_exception(PowerPcException::Program, self.pc.wrapping_sub(4));
                    return;
                }
            } else if is_signaling_nan(b_full) {
                self.set_fp_exception(FPSCR_VXSNAN);
                self.clear_fifr();
                if self.fpscr & FPSCR_VE != 0 {
                    self.take_exception(PowerPcException::Program, self.pc.wrapping_sub(4));
                    return;
                }
            } else if b_full.is_nan() || b_full.is_infinite() {
                self.clear_fifr();
            }
            let result = gekko_reciprocal_estimate(b_full);
            self.set_paired_raw(fd, result.to_bits(), result.to_bits());
            self.float_record_single(instruction, result as f32);
            return;
        }

        let a = f64::from_bits(self.fpr[fa]);
        let b = b_full;
        let c = f64::from_bits(self.fpr[fc]);
        let result = match xo {
            18 => {
                let value = self.float_div(a, b);
                if self.exception_raised {
                    return;
                }
                self.force_single(value)
            }
            20 => {
                let result = self.fp_sub_result(a, b);
                if self.trap_invalid_if_enabled(result.invalid) {
                    return;
                }
                self.force_single(result.value)
            }
            21 => {
                let result = self.fp_add_result(a, b);
                if self.trap_invalid_if_enabled(result.invalid) {
                    return;
                }
                self.force_single(result.value)
            }
            22 => {
                let value = self.float_sqrt(b);
                if self.exception_raised {
                    return;
                }
                self.force_single(value)
            }
            25 => {
                let result = self.fp_single_mul_result(a, c);
                if self.trap_invalid_if_enabled(result.invalid) {
                    return;
                }
                self.force_single(result.value)
            }
            28..=31 => {
                let subtract = matches!(xo, 28 | 30);
                let result = self.fp_madd_result(a, c, b, subtract, true);
                if self.trap_invalid_if_enabled(result.invalid) {
                    return;
                }
                let value = if matches!(xo, 30 | 31) && !result.value.is_nan() {
                    -result.value
                } else {
                    result.value
                };
                self.force_single(value)
            }
            _ => {
                self.take_exception(PowerPcException::Program, self.pc.wrapping_sub(4));
                return;
            }
        };
        self.set_paired_single(fd, result, result);
        self.float_record_single(instruction, result);
    }

    fn execute_float_double(&mut self, instruction: u32) {
        if !self.require_fpu() {
            return;
        }
        let fd = Self::rd(instruction);
        let fa = Self::ra(instruction);
        let fb = Self::rb(instruction);
        let fc = ((instruction >> 6) & 31) as usize;
        let xo10 = (instruction >> 1) & 0x3ff;
        let xo5 = xo10 & 0x1f;
        let xo = if matches!(xo5, 18 | 20 | 21 | 23 | 25 | 28 | 29 | 30 | 31) {
            xo5
        } else {
            xo10
        };
        let a = f64::from_bits(self.fpr[fa]);
        let b = f64::from_bits(self.fpr[fb]);
        let c = f64::from_bits(self.fpr[fc]);
        match xo {
            0 | 32 => {
                let field = ((instruction >> 23) & 7) as usize;
                let ordered = xo == 32;
                let nibble = if a.is_nan() || b.is_nan() {
                    let signaling = is_signaling_nan(a) || is_signaling_nan(b);
                    if signaling {
                        self.set_fp_exception(FPSCR_VXSNAN);
                    }
                    if ordered && (!signaling || self.fpscr & FPSCR_VE == 0) {
                        self.set_fp_exception(FPSCR_VXVC);
                    }
                    0x1
                } else if a < b {
                    0x8
                } else if a > b {
                    0x4
                } else {
                    0x2
                };
                self.update_fpcc(nibble);
                self.set_cr_field(field, nibble);
            }
            12 => {
                let result = b as f32;
                self.set_paired(fd, f64::from(result), f64::from(result));
                self.float_record_single(instruction, result);
            }
            14 | 15 => {
                let rounded = if xo == 15 {
                    b.trunc()
                } else {
                    self.round_fpscr(b)
                };
                let word = if rounded.is_finite()
                    && rounded >= f64::from(i32::MIN)
                    && rounded < 2_147_483_648.0
                {
                    rounded as i32
                } else {
                    self.set_fp_exception(FPSCR_VXCVI);
                    self.clear_fifr();
                    if self.fpscr & FPSCR_VE != 0 {
                        self.take_exception(PowerPcException::Program, self.pc.wrapping_sub(4));
                        return;
                    }
                    if b.is_nan() || b.is_sign_negative() {
                        i32::MIN
                    } else {
                        i32::MAX
                    }
                };
                self.fpr[fd] = u64::from(word as u32);
                self.record_fp_rc(instruction);
            }
            18 => {
                let result = self.float_div(a, b);
                if self.exception_raised {
                    return;
                }
                self.fpr[fd] = result.to_bits();
                self.float_record_double(instruction, result);
            }
            20 => {
                let result = self.fp_sub_result(a, b);
                if self.trap_invalid_if_enabled(result.invalid) {
                    return;
                }
                self.fpr[fd] = result.value.to_bits();
                self.float_record_double(instruction, result.value);
            }
            21 => {
                let result = self.fp_add_result(a, b);
                if self.trap_invalid_if_enabled(result.invalid) {
                    return;
                }
                self.fpr[fd] = result.value.to_bits();
                self.float_record_double(instruction, result.value);
            }
            22 => {
                let result = self.float_sqrt(b);
                if self.exception_raised {
                    return;
                }
                self.fpr[fd] = result.to_bits();
                self.float_record_double(instruction, result);
            }
            23 => {
                let result = if a >= 0.0 { c } else { b };
                self.fpr[fd] = result.to_bits();
                self.record_fp_rc(instruction);
            }
            25 => {
                let result = self.fp_mul_result(a, c);
                if self.trap_invalid_if_enabled(result.invalid) {
                    return;
                }
                self.fpr[fd] = result.value.to_bits();
                self.float_record_double(instruction, result.value);
            }
            26 => {
                if b == 0.0 {
                    self.set_fp_exception(FPSCR_ZX);
                    self.clear_fifr();
                    if self.fpscr & FPSCR_ZE != 0 {
                        self.take_exception(PowerPcException::Program, self.pc.wrapping_sub(4));
                        return;
                    }
                } else {
                    if is_signaling_nan(b) {
                        self.set_fp_exception(FPSCR_VXSNAN);
                    }
                    if b < 0.0 {
                        self.set_fp_exception(FPSCR_VXSQRT);
                    }
                    if b < 0.0 || b.is_nan() || b.is_infinite() {
                        self.clear_fifr();
                    }
                    if self.fpscr & FPSCR_VE != 0 && (is_signaling_nan(b) || b < 0.0) {
                        self.take_exception(PowerPcException::Program, self.pc.wrapping_sub(4));
                        return;
                    }
                }
                let result = gekko_reciprocal_sqrt_estimate(b);
                self.fpr[fd] = result.to_bits();
                self.float_record_double(instruction, result);
            }
            28..=31 => {
                let subtract = matches!(xo, 28 | 30);
                let result = self.fp_madd_result(a, c, b, subtract, false);
                if self.trap_invalid_if_enabled(result.invalid) {
                    return;
                }
                let value = if matches!(xo, 30 | 31) && !result.value.is_nan() {
                    -result.value
                } else {
                    result.value
                };
                self.fpr[fd] = value.to_bits();
                self.float_record_double(instruction, value);
            }
            40 => {
                self.fpr[fd] = (-b).to_bits();
                self.record_fp_rc(instruction);
            }
            72 => {
                self.fpr[fd] = self.fpr[fb];
                self.record_fp_rc(instruction);
            }
            136 => {
                self.fpr[fd] = (-b.abs()).to_bits();
                self.record_fp_rc(instruction);
            }
            264 => {
                self.fpr[fd] = b.abs().to_bits();
                self.record_fp_rc(instruction);
            }
            583 => {
                self.fpr[fd] = u64::from(self.fpscr);
                self.record_fp_rc(instruction);
            }
            711 => {
                let mask = ((instruction >> 17) & 0xff) as u8;
                let source = self.fpr[fb] as u32;
                for field in 0..8 {
                    if mask & (1 << (7 - field)) != 0 {
                        let shift = (7 - field) * 4;
                        self.fpscr = (self.fpscr & !(0xf << shift)) | (source & (0xf << shift));
                    }
                }
                self.update_fp_exception_summary();
                self.record_fp_rc(instruction);
            }
            _ => self.take_exception(PowerPcException::Program, self.pc.wrapping_sub(4)),
        }
    }

    fn require_fpu(&mut self) -> bool {
        if self.msr & MSR_FP != 0 {
            true
        } else {
            self.take_exception(
                PowerPcException::FloatingPointUnavailable,
                self.pc.wrapping_sub(4),
            );
            false
        }
    }

    fn float_div<T: Into<f64>>(&mut self, left: T, right: T) -> f64 {
        let (result, zero_divide) = self.fp_div_result(left.into(), right.into());
        if result.invalid && self.fpscr & FPSCR_VE != 0 || zero_divide && self.fpscr & FPSCR_ZE != 0
        {
            self.take_exception(PowerPcException::Program, self.pc.wrapping_sub(4));
        }
        result.value
    }

    fn float_sqrt<T: Into<f64>>(&mut self, value: T) -> f64 {
        let value = value.into();
        if is_signaling_nan(value) {
            self.set_fp_exception(FPSCR_VXSNAN);
            self.clear_fifr();
            if self.fpscr & FPSCR_VE != 0 {
                self.take_exception(PowerPcException::Program, self.pc.wrapping_sub(4));
                return f64::from_bits(CANONICAL_QNAN);
            }
            return quiet_nan(value);
        }
        if value.is_nan() {
            self.clear_fifr();
            return quiet_nan(value);
        }
        if value < 0.0 {
            self.set_fp_exception(FPSCR_VXSQRT);
            self.clear_fifr();
            if self.fpscr & FPSCR_VE != 0 {
                self.take_exception(PowerPcException::Program, self.pc.wrapping_sub(4));
            }
            return f64::from_bits(CANONICAL_QNAN);
        }
        value.sqrt()
    }

    fn round_fpscr(&self, value: f64) -> f64 {
        match self.fpscr & 3 {
            0 => value.round_ties_even(),
            1 => value.trunc(),
            2 => value.ceil(),
            _ => value.floor(),
        }
    }

    fn float_record_double(&mut self, instruction: u32, value: f64) {
        self.update_fprf_double(value);
        self.record_fp_rc(instruction);
    }

    fn float_record_single(&mut self, instruction: u32, value: f32) {
        self.update_fprf_single(value);
        self.record_fp_rc(instruction);
    }

    fn aligned(&mut self, address: u32, width: u32, write: bool) -> bool {
        if address & (width - 1) == 0 {
            true
        } else {
            self.dar = address;
            self.dsisr = if write { 1 << 25 } else { 0 };
            self.take_exception(PowerPcException::Alignment, self.pc.wrapping_sub(4));
            false
        }
    }

    fn update_base(&mut self, ra: usize, address: u32) {
        if ra == 0 {
            self.take_exception(PowerPcException::Program, self.pc.wrapping_sub(4));
        } else {
            self.gpr[ra] = address;
        }
    }

    fn set_ca(&mut self, carry: bool) {
        if carry {
            self.xer |= XER_CA;
        } else {
            self.xer &= !XER_CA;
        }
    }

    fn update_overflow(
        &mut self,
        instruction: u32,
        lhs: i32,
        rhs: i32,
        result: i32,
        subtract: bool,
    ) {
        if instruction & (1 << 10) == 0 {
            return;
        }
        let overflow = if subtract {
            ((lhs ^ rhs) & (lhs ^ result)) < 0
        } else {
            ((lhs ^ result) & (rhs ^ result)) < 0
        };
        if overflow {
            self.xer |= XER_OV | XER_SO;
        } else {
            self.xer &= !XER_OV;
        }
    }

    fn maybe_record(&mut self, instruction: u32, result: u32) {
        if instruction & 1 != 0 {
            self.record_cr0(result);
        }
    }

    fn record_cr0(&mut self, result: u32) {
        let nibble = (if (result as i32) < 0 {
            0x8
        } else if result > 0 {
            0x4
        } else {
            0x2
        }) | u32::from(self.xer & XER_SO != 0);
        self.set_cr_field(0, nibble);
    }

    fn set_cr_field(&mut self, field: usize, value: u32) {
        let shift = (7 - (field & 7)) * 4;
        self.cr = (self.cr & !(0xf << shift)) | ((value & 0xf) << shift);
    }

    fn cr_bit(&self, bit: u8) -> bool {
        self.cr & (1 << (31 - u32::from(bit & 31))) != 0
    }

    fn set_cr_bit(&mut self, bit: u8, value: bool) {
        let mask = 1 << (31 - u32::from(bit & 31));
        if value {
            self.cr |= mask;
        } else {
            self.cr &= !mask;
        }
    }

    fn take_exception(&mut self, exception: PowerPcException, address: u32) {
        self.exception_raised = true;
        self.halted = false;
        self.srr0 = address;
        self.srr1 = self.msr;
        self.msr &= !(MSR_EE
            | MSR_PR
            | MSR_FP
            | MSR_FE0
            | MSR_SE
            | MSR_BE
            | MSR_FE1
            | MSR_IR
            | MSR_DR
            | MSR_RI
            | MSR_LE);
        let vector = match exception {
            PowerPcException::SystemReset => 0x0100,
            PowerPcException::MachineCheck => 0x0200,
            PowerPcException::DataStorage => 0x0300,
            PowerPcException::InstructionStorage => 0x0400,
            PowerPcException::ExternalInterrupt => 0x0500,
            PowerPcException::Alignment => 0x0600,
            PowerPcException::Program => 0x0700,
            PowerPcException::FloatingPointUnavailable => 0x0800,
            PowerPcException::Decrementer => 0x0900,
            PowerPcException::SystemCall => 0x0c00,
            PowerPcException::Trace => 0x0d00,
        };
        self.pc = if self.srr1 & MSR_IP != 0 {
            0xfff0_0000 | vector
        } else {
            vector
        };
        self.reservation = None;
    }

    pub fn save(&self, out: &mut StateWriter) {
        for value in self.gpr {
            out.u32(value);
        }
        for value in self.fpr {
            out.u64(value);
        }
        for value in self.paired1 {
            out.u64(value);
        }
        for value in self.gqr {
            out.u32(value);
        }
        out.u32(self.pc);
        out.u32(self.lr);
        out.u32(self.ctr);
        out.u32(self.cr);
        out.u32(self.xer);
        out.u32(self.msr);
        out.u32(self.srr0);
        out.u32(self.srr1);
        out.u32(self.dar);
        out.u32(self.dsisr);
        out.u32(self.decrementer);
        out.u64(self.time_base);
        out.u32(self.hid0);
        out.u32(self.hid2);
        out.u32(self.fpscr);
        out.u64(self.cycles);
        out.u8(self.reservation.is_some() as u8);
        out.u32(self.reservation.unwrap_or(0));
        out.u8(self.external_interrupt as u8);
        out.u8(self.halted as u8);
    }

    pub fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        for value in &mut self.gpr {
            *value = input.u32()?;
        }
        for value in &mut self.fpr {
            *value = input.u64()?;
        }
        for value in &mut self.paired1 {
            *value = input.u64()?;
        }
        for value in &mut self.gqr {
            *value = input.u32()?;
        }
        self.pc = input.u32()? & !3;
        self.lr = input.u32()?;
        self.ctr = input.u32()?;
        self.cr = input.u32()?;
        self.xer = input.u32()?;
        self.msr = input.u32()?;
        self.srr0 = input.u32()?;
        self.srr1 = input.u32()?;
        self.dar = input.u32()?;
        self.dsisr = input.u32()?;
        self.decrementer = input.u32()?;
        self.time_base = input.u64()?;
        self.hid0 = input.u32()?;
        self.hid2 = input.u32()?;
        self.fpscr = input.u32()?;
        self.cycles = input.u64()?;
        let has_reservation = input.u8()? != 0;
        let reservation = input.u32()? & !31;
        self.reservation = has_reservation.then_some(reservation);
        self.external_interrupt = input.u8()? != 0;
        self.halted = input.u8()? != 0;
        self.exception_raised = false;
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
                bytes: vec![0; 0x4000],
            }
        }
        fn put32(&mut self, address: u32, value: u32) {
            let start = address as usize;
            self.bytes[start..start + 4].copy_from_slice(&value.to_be_bytes());
        }
    }

    impl PowerPcBus for TestBus {
        fn read8(&mut self, address: u32) -> u8 {
            self.bytes[address as usize]
        }
        fn write8(&mut self, address: u32, value: u8) {
            self.bytes[address as usize] = value;
        }
    }

    fn d(op: u32, rd: u8, ra: u8, immediate: u16) -> u32 {
        (op << 26) | (u32::from(rd) << 21) | (u32::from(ra) << 16) | u32::from(immediate)
    }

    fn x(rd: u8, ra: u8, rb: u8, xo: u32, rc: bool) -> u32 {
        (31 << 26)
            | (u32::from(rd) << 21)
            | (u32::from(ra) << 16)
            | (u32::from(rb) << 11)
            | (xo << 1)
            | u32::from(rc)
    }

    fn spr_x(rd: u8, spr: u16, xo: u32) -> u32 {
        let encoded = ((spr & 0x1f) << 5) | ((spr >> 5) & 0x1f);
        (31 << 26) | (u32::from(rd) << 21) | (u32::from(encoded) << 11) | (xo << 1)
    }

    fn psq(op: u32, reg: u8, ra: u8, displacement: i16, gqr: u8, scalar: bool) -> u32 {
        (op << 26)
            | (u32::from(reg) << 21)
            | (u32::from(ra) << 16)
            | (u32::from(scalar) << 15)
            | (u32::from(gqr & 7) << 12)
            | (u32::from(displacement as u16) & 0x0fff)
    }

    fn ps(fd: u8, fa: u8, fb: u8, fc: u8, sub5: u32) -> u32 {
        (4 << 26)
            | (u32::from(fd) << 21)
            | (u32::from(fa) << 16)
            | (u32::from(fb) << 11)
            | (u32::from(fc) << 6)
            | (sub5 << 1)
    }

    fn ps_special(fd: u8, fa: u8, fb: u8, sub10: u32) -> u32 {
        (4 << 26)
            | (u32::from(fd) << 21)
            | (u32::from(fa) << 16)
            | (u32::from(fb) << 11)
            | (sub10 << 1)
    }

    fn fp_a(op: u32, fd: u8, fa: u8, fb: u8, fc: u8, xo: u32) -> u32 {
        (op << 26)
            | (u32::from(fd) << 21)
            | (u32::from(fa) << 16)
            | (u32::from(fb) << 11)
            | (u32::from(fc) << 6)
            | (xo << 1)
    }

    fn fp_compare(field: u8, fa: u8, fb: u8, ordered: bool) -> u32 {
        (63 << 26)
            | (u32::from(field & 7) << 23)
            | (u32::from(fa) << 16)
            | (u32::from(fb) << 11)
            | ((u32::from(ordered) * 32) << 1)
    }

    #[test]
    fn integer_branch_memory_and_condition_register_execute() {
        let mut bus = TestBus::new();
        bus.put32(0x100, d(14, 3, 0, 10));
        bus.put32(0x104, d(14, 4, 0, 20));
        bus.put32(0x108, x(5, 3, 4, 266, true));
        bus.put32(0x10c, d(36, 5, 0, 0x200));
        bus.put32(0x110, d(32, 6, 0, 0x200));
        bus.put32(0x114, (11 << 26) | (u32::from(6u8) << 16) | 30);
        bus.put32(0x118, (16 << 26) | (0x0c << 21) | (2 << 16) | 8);
        bus.put32(0x11c, d(14, 7, 0, 1));
        bus.put32(0x120, d(14, 7, 0, 2));
        let mut cpu = PowerPc750::new();
        cpu.reset_to(0x100);
        for _ in 0..8 {
            cpu.step(&mut bus);
        }
        assert_eq!(cpu.gpr[5], 30);
        assert_eq!(cpu.gpr[6], 30);
        assert_eq!(bus.read32(0x200), 30);
        assert_eq!(cpu.gpr[7], 2);
    }

    #[test]
    fn rotate_reservation_spr_and_byte_reverse_execute() {
        let mut bus = TestBus::new();
        let mut cpu = PowerPc750::new();
        cpu.reset_to(0x100);
        cpu.gpr[3] = 0x1234_5678;
        cpu.execute(
            &mut bus,
            (21 << 26) | (3 << 21) | (4 << 16) | (8 << 11) | (8 << 6) | (23 << 1),
            0x100,
        );
        assert_eq!(cpu.gpr[4], 0x0056_7800);
        cpu.gpr[5] = 0x220;
        bus.write32(0x220, 0xaabb_ccdd);
        cpu.execute(&mut bus, x(6, 0, 5, 20, false), 0x104);
        assert_eq!(cpu.gpr[6], 0xaabb_ccdd);
        cpu.gpr[6] = 0xdead_beef;
        cpu.execute(&mut bus, x(6, 0, 5, 150, true), 0x108);
        assert_eq!(bus.read32(0x220), 0xdead_beef);
        assert_ne!(cpu.cr & 0x2000_0000, 0);
        cpu.gpr[8] = 0x8877_6655;
        cpu.gpr[9] = 0x240;
        cpu.execute(&mut bus, x(8, 0, 9, 662, false), 0x10c);
        assert_eq!(bus.read32(0x240), 0x5566_7788);
        cpu.lr = 0x1234_5678;
        cpu.execute(&mut bus, spr_x(10, 8, 339), 0x110);
        assert_eq!(cpu.gpr[10], 0x1234_5678);
    }

    #[test]
    fn floating_load_arithmetic_store_and_unavailable_exception_work() {
        let mut bus = TestBus::new();
        bus.write32(0x300, 1.5f32.to_bits());
        bus.write32(0x304, 2.25f32.to_bits());
        let mut cpu = PowerPc750::new();
        cpu.reset_to(0x100);
        cpu.msr |= MSR_FP;
        cpu.execute(&mut bus, d(48, 1, 0, 0x300), 0x100);
        cpu.execute(&mut bus, d(48, 2, 0, 0x304), 0x104);
        let fadds = (59 << 26) | (3 << 21) | (1 << 16) | (2 << 11) | (21 << 1);
        cpu.execute(&mut bus, fadds, 0x108);
        assert_eq!(f64::from_bits(cpu.fpr[3]), 3.75);
        cpu.execute(&mut bus, d(52, 3, 0, 0x308), 0x10c);
        assert_eq!(f32::from_bits(bus.read32(0x308)), 3.75);
        cpu.msr &= !MSR_FP;
        cpu.pc = 0x114;
        cpu.execute(&mut bus, fadds, 0x110);
        assert_eq!(cpu.srr0, 0x110);
        assert_eq!(cpu.pc & 0xffff, 0x0800);
    }

    #[test]
    fn gekko_quantized_load_store_and_spr_controls_execute() {
        let mut bus = TestBus::new();
        bus.write16(0x300, 100);
        bus.write16(0x302, 200);
        let mut cpu = PowerPc750::new();
        cpu.reset_to(0x100);
        cpu.msr |= MSR_FP;
        cpu.hid2 = HID2_PSE | HID2_LSQE;
        cpu.gqr[1] = 5 | (1 << 8) | (5 << 16) | (1 << 24);
        cpu.execute(&mut bus, psq(56, 2, 0, 0x300, 1, false), 0x100);
        assert_eq!(cpu.paired(2), (50.0, 100.0));
        cpu.execute(&mut bus, psq(60, 2, 0, 0x320, 1, false), 0x104);
        assert_eq!(bus.read16(0x320), 100);
        assert_eq!(bus.read16(0x322), 200);

        cpu.gpr[3] = 0x1122_3344;
        cpu.execute(&mut bus, spr_x(3, 913, 467), 0x108);
        cpu.execute(&mut bus, spr_x(4, 913, 339), 0x10c);
        assert_eq!(cpu.gqr[1], 0x1122_3344);
        assert_eq!(cpu.gpr[4], 0x1122_3344);
    }

    #[test]
    fn gekko_paired_arithmetic_merge_and_legality_execute() {
        let mut bus = TestBus::new();
        let mut cpu = PowerPc750::new();
        cpu.reset_to(0x100);
        cpu.msr |= MSR_FP;
        cpu.hid2 = HID2_PSE | HID2_LSQE;
        cpu.set_paired(1, 1.5, -2.0);
        cpu.set_paired(2, 2.5, 4.0);
        cpu.execute(&mut bus, ps(3, 1, 2, 0, 21), 0x100);
        assert_eq!(cpu.paired(3), (4.0, 2.0));
        cpu.execute(&mut bus, ps_special(4, 1, 2, 592), 0x104);
        assert_eq!(cpu.paired(4), (-2.0, 2.5));

        cpu.hid2 = 0;
        cpu.pc = 0x10c;
        cpu.execute(&mut bus, ps(5, 1, 2, 0, 21), 0x108);
        assert_eq!(cpu.srr0, 0x108);
        assert_eq!(cpu.pc & 0xffff, 0x0700);
    }

    #[test]
    fn gekko_estimate_tables_and_special_values_match_hardware_shape() {
        assert_eq!(
            gekko_reciprocal_estimate(1.0).to_bits(),
            0x3fef_ff00_0000_0000
        );
        assert_eq!(
            gekko_reciprocal_sqrt_estimate(1.0).to_bits(),
            0x3fef_fe80_0000_0000
        );
        assert_eq!(
            gekko_reciprocal_estimate(-0.0).to_bits(),
            f64::NEG_INFINITY.to_bits()
        );
        assert_eq!(
            gekko_reciprocal_sqrt_estimate(-0.0).to_bits(),
            f64::NEG_INFINITY.to_bits()
        );
        assert_eq!(
            gekko_reciprocal_estimate(f64::INFINITY).to_bits(),
            0.0f64.to_bits()
        );
        assert_eq!(
            gekko_reciprocal_sqrt_estimate(f64::INFINITY).to_bits(),
            0.0f64.to_bits()
        );
        assert_eq!(
            gekko_reciprocal_estimate(f64::from_bits(1)).to_bits(),
            f64::from(f32::MAX).to_bits()
        );
        assert_eq!(
            gekko_reciprocal_estimate(f64::from_bits(0x7fef_ffff_ffff_ffff)).to_bits(),
            0.0f64.to_bits()
        );
        assert_eq!(
            gekko_reciprocal_sqrt_estimate(f64::from_bits(1)).to_bits(),
            0x617f_fe80_0000_0000
        );
        assert_eq!(
            gekko_reciprocal_sqrt_estimate(f64::from_bits(0x7fef_ffff_ffff_ffff)).to_bits(),
            0x1ff0_0008_2c00_0000
        );
        let signaling = f64::from_bits(0x7ff0_0000_0000_0123);
        assert!(is_signaling_nan(signaling));
        assert_eq!(
            gekko_reciprocal_estimate(signaling).to_bits(),
            0x7ff8_0000_0000_0123
        );
        assert_eq!(
            gekko_reciprocal_sqrt_estimate(signaling).to_bits(),
            0x7ff8_0000_0000_0123
        );
        assert_eq!(
            gekko_reciprocal_sqrt_estimate(-1.0).to_bits(),
            CANONICAL_QNAN
        );
    }

    #[test]
    fn gekko_scalar_and_paired_estimates_use_hardware_approximations() {
        let mut bus = TestBus::new();
        let mut cpu = PowerPc750::new();
        cpu.reset_to(0x100);
        cpu.msr |= MSR_FP;
        cpu.hid2 = HID2_PSE | HID2_LSQE;

        cpu.fpr[1] = 1.0f64.to_bits();
        cpu.execute(&mut bus, fp_a(59, 2, 0, 1, 0, 24), 0x100);
        assert_eq!(cpu.fpr[2], 0x3fef_ff00_0000_0000);
        assert_eq!(cpu.paired1[2], cpu.fpr[2]);

        cpu.paired1[3] = 0x1122_3344_5566_7788;
        cpu.execute(&mut bus, fp_a(63, 3, 0, 1, 0, 26), 0x104);
        assert_eq!(cpu.fpr[3], 0x3fef_fe80_0000_0000);
        assert_eq!(cpu.paired1[3], 0x1122_3344_5566_7788);

        cpu.set_paired(4, 1.0, 4.0);
        cpu.execute(&mut bus, ps(5, 0, 4, 0, 24), 0x108);
        let res = cpu.paired(5);
        assert_eq!(res.0.to_bits(), gekko_reciprocal_estimate(1.0).to_bits());
        assert_eq!(
            res.1.to_bits(),
            f64::from(gekko_reciprocal_estimate(4.0) as f32).to_bits()
        );

        cpu.execute(&mut bus, ps(6, 0, 4, 0, 26), 0x10c);
        let rsqrt = cpu.paired(6);
        assert_eq!(
            rsqrt.0.to_bits(),
            f64::from(gekko_reciprocal_sqrt_estimate(1.0) as f32).to_bits()
        );
        assert_eq!(
            rsqrt.1.to_bits(),
            f64::from(gekko_reciprocal_sqrt_estimate(4.0) as f32).to_bits()
        );
    }

    #[test]
    fn gekko_single_precision_ops_read_fb_and_fill_both_lanes() {
        let mut bus = TestBus::new();
        let mut cpu = PowerPc750::new();
        cpu.reset_to(0x100);
        cpu.msr |= MSR_FP;
        cpu.fpr[1] = 144.0f64.to_bits();
        cpu.fpr[2] = 9.0f64.to_bits();

        cpu.execute(&mut bus, fp_a(59, 3, 1, 2, 0, 22), 0x100);
        assert_eq!(cpu.paired(3), (3.0, 3.0));

        cpu.fpr[2] = 3.75f64.to_bits();
        cpu.execute(&mut bus, fp_a(63, 4, 1, 2, 0, 12), 0x104);
        assert_eq!(cpu.paired(4), (3.75, 3.75));
    }

    #[test]
    fn gekko_estimate_exceptions_preserve_destination_when_enabled() {
        let mut bus = TestBus::new();
        let mut cpu = PowerPc750::new();
        cpu.reset_to(0x100);
        cpu.msr |= MSR_FP;
        cpu.fpscr = FPSCR_VE;
        cpu.fpr[1] = 0x7ff0_0000_0000_0001;
        cpu.fpr[2] = 0x0123_4567_89ab_cdef;
        cpu.paired1[2] = 0xfedc_ba98_7654_3210;
        cpu.pc = 0x104;
        cpu.execute(&mut bus, fp_a(59, 2, 0, 1, 0, 24), 0x100);
        assert_eq!(cpu.fpr[2], 0x0123_4567_89ab_cdef);
        assert_eq!(cpu.paired1[2], 0xfedc_ba98_7654_3210);
        assert_ne!(cpu.fpscr & (FPSCR_FX | FPSCR_VX), 0);
        assert_eq!(cpu.srr0, 0x100);
        assert_eq!(cpu.pc & 0xffff, 0x0700);
    }

    #[test]
    fn gekko_fprf_classification_and_rc_follow_fpscr() {
        assert_eq!(classify_f64(f64::NAN), 0x11);
        assert_eq!(classify_f64(f64::NEG_INFINITY), 0x09);
        assert_eq!(classify_f64(-1.0), 0x08);
        assert_eq!(classify_f64(f64::from_bits(DOUBLE_SIGN | 1)), 0x18);
        assert_eq!(classify_f64(-0.0), 0x12);
        assert_eq!(classify_f64(0.0), 0x02);
        assert_eq!(classify_f64(f64::from_bits(1)), 0x14);
        assert_eq!(classify_f64(1.0), 0x04);
        assert_eq!(classify_f64(f64::INFINITY), 0x05);
        assert_eq!(classify_f32(f32::from_bits(1)), 0x14);

        let mut bus = TestBus::new();
        let mut cpu = PowerPc750::new();
        cpu.reset_to(0x100);
        cpu.msr |= MSR_FP;
        cpu.fpscr = FPSCR_OX;
        cpu.fpr[1] = 1.0f64.to_bits();
        cpu.fpr[2] = 2.0f64.to_bits();
        cpu.execute(&mut bus, fp_a(63, 3, 1, 2, 0, 21) | 1, 0x100);
        assert_eq!(f64::from_bits(cpu.fpr[3]), 3.0);
        assert_eq!((cpu.fpscr & FPSCR_FPRF_MASK) >> 12, 0x04);
        assert_eq!((cpu.cr >> 24) & 0xf, 0x1);

        cpu.fpscr = FPSCR_OX | (0x18 << 12);
        cpu.fpr[1] = 1.0f64.to_bits();
        cpu.fpr[2] = 5.0f64.to_bits();
        cpu.fpr[4] = 7.0f64.to_bits();
        cpu.execute(&mut bus, fp_a(63, 3, 1, 2, 4, 23) | 1, 0x104);
        assert_eq!(f64::from_bits(cpu.fpr[3]), 7.0);
        assert_eq!((cpu.fpscr & FPSCR_FPRF_MASK) >> 12, 0x18);
        assert_eq!((cpu.cr >> 24) & 0xf, 0x1);
    }

    #[test]
    fn gekko_float_compares_distinguish_quiet_and_signaling_nan() {
        let qnan = f64::from_bits(0x7ff8_0000_0000_0123);
        let snan = f64::from_bits(0x7ff0_0000_0000_0123);
        let mut bus = TestBus::new();

        let mut unordered_quiet = PowerPc750::new();
        unordered_quiet.reset_to(0x100);
        unordered_quiet.msr |= MSR_FP;
        unordered_quiet.fpr[1] = qnan.to_bits();
        unordered_quiet.fpr[2] = 1.0f64.to_bits();
        unordered_quiet.execute(&mut bus, fp_compare(3, 1, 2, false), 0x100);
        assert_eq!((unordered_quiet.cr >> 16) & 0xf, 0x1);
        assert_eq!((unordered_quiet.fpscr & FPSCR_FPCC_MASK) >> 12, 0x1);
        assert_eq!(unordered_quiet.fpscr & FPSCR_VX_ANY, 0);

        let mut unordered_signal = PowerPc750::new();
        unordered_signal.reset_to(0x100);
        unordered_signal.msr |= MSR_FP;
        unordered_signal.fpr[1] = snan.to_bits();
        unordered_signal.fpr[2] = 1.0f64.to_bits();
        unordered_signal.execute(&mut bus, fp_compare(2, 1, 2, false), 0x100);
        assert_ne!(unordered_signal.fpscr & FPSCR_VXSNAN, 0);
        assert_eq!(unordered_signal.fpscr & FPSCR_VXVC, 0);
        assert_eq!(unordered_signal.fpscr & FPSCR_FEX, 0);
        assert_ne!(unordered_signal.fpscr & (FPSCR_FX | FPSCR_VX), 0);

        let mut ordered_quiet = PowerPc750::new();
        ordered_quiet.reset_to(0x100);
        ordered_quiet.msr |= MSR_FP;
        ordered_quiet.fpr[1] = qnan.to_bits();
        ordered_quiet.fpr[2] = 1.0f64.to_bits();
        ordered_quiet.execute(&mut bus, fp_compare(1, 1, 2, true), 0x100);
        assert_ne!(ordered_quiet.fpscr & FPSCR_VXVC, 0);
        assert_eq!(ordered_quiet.fpscr & FPSCR_VXSNAN, 0);

        let mut ordered_signal_enabled = PowerPc750::new();
        ordered_signal_enabled.reset_to(0x100);
        ordered_signal_enabled.msr |= MSR_FP;
        ordered_signal_enabled.fpscr = FPSCR_VE;
        ordered_signal_enabled.fpr[1] = snan.to_bits();
        ordered_signal_enabled.fpr[2] = 1.0f64.to_bits();
        ordered_signal_enabled.execute(&mut bus, fp_compare(4, 1, 2, true), 0x100);
        assert_ne!(ordered_signal_enabled.fpscr & FPSCR_VXSNAN, 0);
        assert_eq!(ordered_signal_enabled.fpscr & FPSCR_VXVC, 0);
        assert_ne!(ordered_signal_enabled.fpscr & FPSCR_FEX, 0);
    }

    #[test]
    fn gekko_enabled_divide_and_sqrt_exceptions_preserve_destination() {
        let mut bus = TestBus::new();

        let mut invalid_divide = PowerPc750::new();
        invalid_divide.reset_to(0x100);
        invalid_divide.msr |= MSR_FP;
        invalid_divide.fpscr = FPSCR_VE;
        invalid_divide.fpr[1] = 0.0f64.to_bits();
        invalid_divide.fpr[2] = 0.0f64.to_bits();
        invalid_divide.fpr[3] = 0x0123_4567_89ab_cdef;
        invalid_divide.pc = 0x104;
        invalid_divide.execute(&mut bus, fp_a(63, 3, 1, 2, 0, 18), 0x100);
        assert_eq!(invalid_divide.fpr[3], 0x0123_4567_89ab_cdef);
        assert_ne!(invalid_divide.fpscr & FPSCR_VXZDZ, 0);
        assert_ne!(invalid_divide.fpscr & FPSCR_FEX, 0);
        assert_eq!(invalid_divide.pc & 0xffff, 0x0700);

        let mut zero_divide = PowerPc750::new();
        zero_divide.reset_to(0x100);
        zero_divide.msr |= MSR_FP;
        zero_divide.fpscr = FPSCR_ZE;
        zero_divide.fpr[1] = 1.0f64.to_bits();
        zero_divide.fpr[2] = 0.0f64.to_bits();
        zero_divide.fpr[3] = 0xfedc_ba98_7654_3210;
        zero_divide.pc = 0x104;
        zero_divide.execute(&mut bus, fp_a(63, 3, 1, 2, 0, 18), 0x100);
        assert_eq!(zero_divide.fpr[3], 0xfedc_ba98_7654_3210);
        assert_ne!(zero_divide.fpscr & FPSCR_ZX, 0);
        assert_ne!(zero_divide.fpscr & FPSCR_FEX, 0);

        let mut invalid_sqrt = PowerPc750::new();
        invalid_sqrt.reset_to(0x100);
        invalid_sqrt.msr |= MSR_FP;
        invalid_sqrt.fpscr = FPSCR_VE;
        invalid_sqrt.fpr[2] = (-1.0f64).to_bits();
        invalid_sqrt.fpr[3] = 0xaaaa_bbbb_cccc_dddd;
        invalid_sqrt.pc = 0x104;
        invalid_sqrt.execute(&mut bus, fp_a(63, 3, 0, 2, 0, 22), 0x100);
        assert_eq!(invalid_sqrt.fpr[3], 0xaaaa_bbbb_cccc_dddd);
        assert_ne!(invalid_sqrt.fpscr & FPSCR_VXSQRT, 0);
        assert_ne!(invalid_sqrt.fpscr & FPSCR_FEX, 0);
    }

    #[test]
    fn gekko_integer_conversion_saturates_and_sets_vxcvi() {
        let mut bus = TestBus::new();
        let mut cpu = PowerPc750::new();
        cpu.reset_to(0x100);
        cpu.msr |= MSR_FP;

        cpu.fpr[1] = f64::INFINITY.to_bits();
        cpu.execute(&mut bus, fp_a(63, 2, 0, 1, 0, 15), 0x100);
        assert_eq!(cpu.fpr[2] as u32, i32::MAX as u32);
        assert_ne!(cpu.fpscr & FPSCR_VXCVI, 0);

        cpu.fpscr = 0;
        cpu.fpr[1] = f64::NEG_INFINITY.to_bits();
        cpu.execute(&mut bus, fp_a(63, 2, 0, 1, 0, 15), 0x104);
        assert_eq!(cpu.fpr[2] as u32, i32::MIN as u32);
        assert_ne!(cpu.fpscr & FPSCR_VXCVI, 0);
    }

    #[test]
    fn gekko_single_rounding_modes_ni_and_force25_follow_fpscr() {
        let mut cpu = PowerPc750::new();
        let halfway = 1.0 + 2.0f64.powi(-24);

        cpu.fpscr = 0;
        assert_eq!(cpu.force_single(halfway).to_bits(), 0x3f80_0000);
        cpu.fpscr = 2;
        assert_eq!(cpu.force_single(halfway).to_bits(), 0x3f80_0001);
        cpu.fpscr = 1;
        assert_eq!(cpu.force_single(halfway).to_bits(), 0x3f80_0000);
        cpu.fpscr = 3;
        assert_eq!(cpu.force_single(-halfway).to_bits(), 0xbf80_0001);

        let min_subnormal = f64::from(f32::from_bits(1));
        cpu.fpscr = 0;
        assert_eq!(cpu.force_single(min_subnormal).to_bits(), 1);
        cpu.fpscr = FPSCR_NI;
        assert_eq!(cpu.force_single(min_subnormal).to_bits(), 0);

        let force25_tie = f64::from_bits(1.0f64.to_bits() | (1 << 27));
        assert_eq!(
            force_25_bit(force25_tie).to_bits(),
            1.0f64.to_bits() | (1 << 28)
        );
    }

    #[test]
    fn gekko_single_ops_keep_fpr_precision_and_fma_rounds_once() {
        let mut bus = TestBus::new();
        let mut cpu = PowerPc750::new();
        cpu.reset_to(0x100);
        cpu.msr |= MSR_FP;

        cpu.fpr[1] = (1.0 + 2.0f64.powi(-24)).to_bits();
        cpu.fpr[2] = 2.0f64.powi(-25).to_bits();
        cpu.execute(&mut bus, fp_a(59, 3, 1, 2, 0, 21), 0x100);
        assert_eq!((f64::from_bits(cpu.fpr[3]) as f32).to_bits(), 0x3f80_0001);
        assert_eq!(cpu.paired1[3], cpu.fpr[3]);

        cpu.fpr[1] = f64::from(f32::from_bits(0x4248_0000)).to_bits();
        cpu.fpr[2] = f64::from(f32::from_bits(0x1b1c_72a0)).to_bits();
        cpu.fpr[3] = f64::from(f32::from_bits(0xbc88_cc38)).to_bits();
        cpu.execute(&mut bus, fp_a(59, 4, 1, 2, 3, 29), 0x104);
        assert_eq!((f64::from_bits(cpu.fpr[4]) as f32).to_bits(), 0xbf55_bf17);

        cpu.fpr[1] = (1.0 + 2.0f64.powi(-27)).to_bits();
        cpu.fpr[2] = (-1.0f64).to_bits();
        cpu.fpr[3] = (1.0 - 2.0f64.powi(-27)).to_bits();
        cpu.execute(&mut bus, fp_a(63, 4, 1, 2, 3, 29), 0x108);
        assert_eq!(cpu.fpr[4], (-(2.0f64.powi(-54))).to_bits());
    }

    #[test]
    fn gekko_arithmetic_invalid_flags_and_original_snan_survive_frc_rounding() {
        let mut bus = TestBus::new();

        let mut single_snan = PowerPc750::new();
        single_snan.reset_to(0x100);
        single_snan.msr |= MSR_FP;
        single_snan.fpscr = FPSCR_VE;
        single_snan.fpr[1] = 1.0f64.to_bits();
        single_snan.fpr[3] = 0x7ff0_0000_0000_0001;
        single_snan.fpr[4] = 0x0123_4567_89ab_cdef;
        single_snan.pc = 0x104;
        single_snan.execute(&mut bus, fp_a(59, 4, 1, 0, 3, 25), 0x100);
        assert_eq!(single_snan.fpr[4], 0x0123_4567_89ab_cdef);
        assert_ne!(single_snan.fpscr & FPSCR_VXSNAN, 0);
        assert_ne!(single_snan.fpscr & FPSCR_FEX, 0);

        let mut invalid = PowerPc750::new();
        invalid.reset_to(0x100);
        invalid.msr |= MSR_FP;
        invalid.fpr[1] = f64::INFINITY.to_bits();
        invalid.fpr[2] = f64::NEG_INFINITY.to_bits();
        invalid.execute(&mut bus, fp_a(63, 3, 1, 2, 0, 21), 0x100);
        assert_ne!(invalid.fpscr & FPSCR_VXISI, 0);
        assert!(f64::from_bits(invalid.fpr[3]).is_nan());

        invalid.fpscr = 0;
        invalid.fpr[1] = f64::INFINITY.to_bits();
        invalid.fpr[3] = 0.0f64.to_bits();
        invalid.execute(&mut bus, fp_a(63, 4, 1, 0, 3, 25), 0x104);
        assert_ne!(invalid.fpscr & FPSCR_VXIMZ, 0);

        invalid.fpscr = 0;
        invalid.fpr[2] = 1.0f64.to_bits();
        invalid.execute(&mut bus, fp_a(63, 4, 1, 2, 3, 29), 0x108);
        assert_ne!(invalid.fpscr & FPSCR_VXIMZ, 0);
    }

    #[test]
    fn gekko_paired_select_preserves_raw_lanes_and_divide_sets_invalid_subflags() {
        let mut bus = TestBus::new();
        let mut cpu = PowerPc750::new();
        cpu.reset_to(0x100);
        cpu.msr |= MSR_FP;
        cpu.hid2 = HID2_PSE | HID2_LSQE;

        let c0 = 4.0 + 2.0f64.powi(-40);
        let b1 = 3.0 + 2.0f64.powi(-40);
        cpu.set_paired_raw(1, 1.0f64.to_bits(), (-1.0f64).to_bits());
        cpu.set_paired_raw(2, (2.0 + 2.0f64.powi(-40)).to_bits(), b1.to_bits());
        cpu.set_paired_raw(3, c0.to_bits(), (5.0 + 2.0f64.powi(-40)).to_bits());
        cpu.execute(&mut bus, ps(4, 1, 2, 3, 23), 0x100);
        assert_eq!(cpu.fpr[4], c0.to_bits());
        assert_eq!(cpu.paired1[4], b1.to_bits());

        cpu.fpscr = 0;
        cpu.set_paired_raw(1, 0.0f64.to_bits(), f64::INFINITY.to_bits());
        cpu.set_paired_raw(2, 0.0f64.to_bits(), f64::INFINITY.to_bits());
        cpu.execute(&mut bus, ps(5, 1, 2, 0, 18), 0x104);
        assert_ne!(cpu.fpscr & FPSCR_VXZDZ, 0);
        assert_ne!(cpu.fpscr & FPSCR_VXIDI, 0);
        let divided = cpu.paired(5);
        assert!(divided.0.is_nan());
        assert!(divided.1.is_nan());
    }

    #[test]
    fn decrementer_external_interrupt_and_state_round_trip_are_deterministic() {
        let mut bus = TestBus::new();
        bus.put32(0x100, d(24, 0, 0, 0));
        let mut cpu = PowerPc750::new();
        cpu.reset_to(0x100);
        cpu.msr = MSR_EE | MSR_FP;
        cpu.decrementer = 0;
        cpu.step(&mut bus);
        assert_eq!(cpu.pc & 0xffff, 0x0900);
        cpu.reset_to(0x100);
        cpu.msr = MSR_EE;
        cpu.set_external_interrupt(true);
        cpu.step(&mut bus);
        assert_eq!(cpu.pc & 0xffff, 0x0500);
        cpu.gpr[3] = 0x1234_5678;
        cpu.fpr[4] = 9.5f64.to_bits();
        cpu.paired1[4] = (-2.25f64).to_bits();
        cpu.gqr[2] = 0x1234_5678;
        cpu.hid2 = 0xe000_0000;
        cpu.time_base = 0x1122_3344_5566_7788;
        cpu.lr = 0x9000;
        let mut writer = StateWriter::new(PlatformId::GameCube, 77);
        cpu.save(&mut writer);
        let state = writer.finish();
        let mut reader = StateReader::new(&state, PlatformId::GameCube, 77).unwrap();
        let mut restored = PowerPc750::new();
        restored.load(&mut reader).unwrap();
        reader.finish().unwrap();
        assert_eq!(restored.gpr, cpu.gpr);
        assert_eq!(restored.fpr, cpu.fpr);
        assert_eq!(restored.paired1, cpu.paired1);
        assert_eq!(restored.gqr, cpu.gqr);
        assert_eq!(restored.hid2, cpu.hid2);
        assert_eq!(restored.time_base, cpu.time_base);
        assert_eq!(restored.lr, cpu.lr);
        assert_eq!(restored.pc, cpu.pc);
    }
}
