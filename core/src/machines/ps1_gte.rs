use crate::state::{StateReader, StateWriter};

const FLAG_ERROR_MASK: u32 = 0x7f87_e000;
const MAC44_MAX: i64 = (1i64 << 43) - 1;
const MAC44_MIN: i64 = -(1i64 << 43);

#[derive(Clone)]
struct PendingGteCommand {
    instruction: u32,
    issue_cycle: u64,
    complete_cycle: u64,
    data: [u32; 32],
    control: [u32; 32],
}

#[derive(Clone, Default)]
pub struct Ps1Gte {
    data: [u32; 32],
    control: [u32; 32],
    pending_command: Option<PendingGteCommand>,
}

impl Ps1Gte {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn reset(&mut self) {
        *self = Self::default();
    }
    fn s16(value: u32) -> i64 {
        i64::from(value as u16 as i16)
    }

    fn signed32(value: u32) -> i64 {
        i64::from(value as i32)
    }

    fn ir(&self, index: usize) -> i64 {
        Self::s16(self.data[8 + index])
    }

    fn mac(&self, index: usize) -> i64 {
        Self::signed32(self.data[24 + index])
    }

    fn set_flag(&mut self, bit: u32) {
        self.control[31] |= 1 << bit;
    }

    fn finish_flags(&mut self) {
        self.control[31] &= 0x7fff_f000;
        if self.control[31] & FLAG_ERROR_MASK != 0 {
            self.control[31] |= 1 << 31;
        } else {
            self.control[31] &= !(1 << 31);
        }
    }
    fn set_mac_overflow(&mut self, index: usize, value: i64) {
        let positive = [30, 29, 28][index - 1];
        let negative = [27, 26, 25][index - 1];
        if value > MAC44_MAX {
            self.set_flag(positive);
        }
        if value < MAC44_MIN {
            self.set_flag(negative);
        }
    }

    fn write_ir(&mut self, index: usize, value: i64, lm: bool, set_flag: bool) -> i64 {
        let minimum = if lm { 0 } else { i64::from(i16::MIN) };
        let maximum = i64::from(i16::MAX);
        let saturated = value.clamp(minimum, maximum);
        if set_flag && saturated != value {
            self.set_flag([24, 23, 22][index - 1]);
        }
        self.data[8 + index] = u32::from(saturated as i16 as u16);
        saturated
    }

    fn set_ir(&mut self, index: usize, value: i64, lm: bool) -> i64 {
        self.write_ir(index, value, lm, true)
    }

    fn set_mac_ir(&mut self, raw: [i64; 3], shift: u32, lm: bool) -> [i64; 3] {
        let mut result = [0i64; 3];
        for index in 1..=3 {
            self.set_mac_overflow(index, raw[index - 1]);
            let value = raw[index - 1] >> shift;
            self.data[24 + index] = value as i32 as u32;
            result[index - 1] = self.set_ir(index, value, lm);
        }
        result
    }
    fn set_mac0(&mut self, value: i64) {
        if value > i64::from(i32::MAX) {
            self.set_flag(16);
        }
        if value < i64::from(i32::MIN) {
            self.set_flag(15);
        }
        self.data[24] = value as i32 as u32;
    }

    fn saturate_sz(&mut self, value: i64) -> u16 {
        let saturated = value.clamp(0, i64::from(u16::MAX));
        if saturated != value {
            self.set_flag(18);
        }
        saturated as u16
    }

    fn saturate_screen(&mut self, value: i64, x_axis: bool) -> i16 {
        let saturated = value.clamp(-0x400, 0x3ff);
        if saturated != value {
            self.set_flag(if x_axis { 14 } else { 13 });
        }
        saturated as i16
    }

    fn saturate_ir0(&mut self, value: i64) -> u16 {
        let saturated = value.clamp(0, 0x1000);
        if saturated != value {
            self.set_flag(12);
        }
        saturated as u16
    }
    fn matrix(&self, base: usize) -> [[i64; 3]; 3] {
        let r0 = self.control[base];
        let r1 = self.control[base + 1];
        let r2 = self.control[base + 2];
        let r3 = self.control[base + 3];
        let r4 = self.control[base + 4];
        [
            [Self::s16(r0), Self::s16(r0 >> 16), Self::s16(r1)],
            [Self::s16(r1 >> 16), Self::s16(r2), Self::s16(r2 >> 16)],
            [Self::s16(r3), Self::s16(r3 >> 16), Self::s16(r4)],
        ]
    }

    fn vector(&self, index: usize) -> [i64; 3] {
        if index == 3 {
            return [self.ir(1), self.ir(2), self.ir(3)];
        }
        let xy = self.data[index * 2];
        [
            Self::s16(xy),
            Self::s16(xy >> 16),
            Self::s16(self.data[index * 2 + 1]),
        ]
    }

    fn translation(&self, cv: usize) -> [i64; 3] {
        let base = match cv {
            0 => Some(5),
            1 => Some(13),
            2 => Some(21),
            _ => None,
        };
        base.map_or([0; 3], |base| {
            std::array::from_fn(|index| Self::signed32(self.control[base + index]))
        })
    }
    fn multiply_raw(matrix: [[i64; 3]; 3], vector: [i64; 3], add: [i64; 3]) -> [i64; 3] {
        std::array::from_fn(|row| {
            add[row] * 0x1000
                + matrix[row][0] * vector[0]
                + matrix[row][1] * vector[1]
                + matrix[row][2] * vector[2]
        })
    }

    fn push_sz(&mut self, value: u16) {
        self.data[16] = self.data[17];
        self.data[17] = self.data[18];
        self.data[18] = self.data[19];
        self.data[19] = u32::from(value);
    }

    fn push_sxy(&mut self, x: i16, y: i16) {
        self.data[12] = self.data[13];
        self.data[13] = self.data[14];
        self.data[14] = u32::from(x as u16) | (u32::from(y as u16) << 16);
    }

    fn sxy(&self, index: usize) -> (i64, i64) {
        let value = self.data[12 + index];
        (Self::s16(value), Self::s16(value >> 16))
    }

    fn pack_irgb(&self) -> u32 {
        let channel = |index| (self.ir(index) / 0x80).clamp(0, 0x1f) as u32;
        channel(1) | (channel(2) << 5) | (channel(3) << 10)
    }
    pub fn read_data(&self, register: u8) -> u32 {
        match usize::from(register & 31) {
            1 | 3 | 5 | 8..=11 => Self::s16(self.data[usize::from(register & 31)]) as i32 as u32,
            7 | 16..=19 => self.data[usize::from(register & 31)] & 0xffff,
            15 => self.data[14],
            28 | 29 => self.pack_irgb(),
            index => self.data[index],
        }
    }

    fn write_data_regs(data: &mut [u32; 32], register: u8, value: u32) {
        let index = usize::from(register & 31);
        match index {
            1 | 3 | 5 | 7..=11 | 16..=19 => data[index] = value & 0xffff,
            15 => {
                data[12] = data[13];
                data[13] = data[14];
                data[14] = value;
            }
            28 => {
                data[9] = (value & 0x1f) * 0x80;
                data[10] = ((value >> 5) & 0x1f) * 0x80;
                data[11] = ((value >> 10) & 0x1f) * 0x80;
            }
            29 | 31 => {}
            30 => {
                data[30] = value;
                data[31] = if value & 0x8000_0000 == 0 {
                    value.leading_zeros()
                } else {
                    (!value).leading_zeros()
                };
            }
            _ => data[index] = value,
        }
    }

    pub fn write_data(&mut self, register: u8, value: u32) {
        Self::write_data_regs(&mut self.data, register, value);
    }
    pub fn read_control(&self, register: u8) -> u32 {
        let index = usize::from(register & 31);
        match index {
            4 | 12 | 20 | 27 | 29 | 30 => Self::s16(self.control[index]) as i32 as u32,
            26 => Self::s16(self.control[26]) as i32 as u32,
            31 => {
                let value = self.control[31] & 0x7fff_f000;
                if value & FLAG_ERROR_MASK != 0 {
                    value | (1 << 31)
                } else {
                    value
                }
            }
            _ => self.control[index],
        }
    }

    fn write_control_regs(control: &mut [u32; 32], register: u8, value: u32) {
        let index = usize::from(register & 31);
        control[index] = match index {
            4 | 12 | 20 | 26 | 27 | 29 | 30 => value & 0xffff,
            31 => value & 0x7fff_f000,
            _ => value,
        };
        if index == 31 {
            let value = control[31] & 0x7fff_f000;
            control[31] = if value & FLAG_ERROR_MASK != 0 {
                value | (1 << 31)
            } else {
                value
            };
        }
    }

    pub fn write_control(&mut self, register: u8, value: u32) {
        Self::write_control_regs(&mut self.control, register, value);
    }

    fn color_channels(value: u32) -> [i64; 3] {
        [
            i64::from(value & 0xff),
            i64::from((value >> 8) & 0xff),
            i64::from((value >> 16) & 0xff),
        ]
    }
    fn unr_entry(index: u32) -> u32 {
        let denominator = index + 0x100;
        ((0x40000 / denominator).div_ceil(2).saturating_sub(0x101)).min(0xff)
    }

    fn divide(&mut self, h: u16, sz: u16) -> u32 {
        if sz == 0 || u32::from(h) >= u32::from(sz) * 2 {
            self.set_flag(17);
            return 0x1ffff;
        }
        let z = sz.leading_zeros();
        let n = u64::from(h) << z;
        let d = u64::from(sz) << z;
        let index = ((d - 0x7fc0) >> 7) as u32;
        let u = u64::from(Self::unr_entry(index) + 0x101);
        let d = (0x0200_0080u64 - d * u) >> 8;
        let d = (0x80u64 + d * u) >> 8;
        (((n * d + 0x8000) >> 16).min(0x1ffff)) as u32
    }

    fn push_color(&mut self) {
        let code = self.data[6] & 0xff00_0000;
        let mut packed = code;
        for index in 0..3 {
            let value = self.mac(index + 1) / 16;
            let saturated = value.clamp(0, 255);
            if saturated != value {
                self.set_flag(21 - index as u32);
            }
            packed |= (saturated as u32) << (index * 8);
        }
        self.data[20] = self.data[21];
        self.data[21] = self.data[22];
        self.data[22] = packed;
    }
    fn perspective(&mut self, vector_index: usize, sf: bool, lm: bool) {
        let raw = Self::multiply_raw(
            self.matrix(0),
            self.vector(vector_index),
            self.translation(0),
        );
        let shift = if sf { 12 } else { 0 };
        let mut ir = [0i64; 3];
        for index in 1..=3 {
            self.set_mac_overflow(index, raw[index - 1]);
            let value = raw[index - 1] >> shift;
            self.data[24 + index] = value as i32 as u32;
            ir[index - 1] = if index == 3 {
                self.write_ir(index, value, lm, false)
            } else {
                self.set_ir(index, value, lm)
            };
        }
        let ir3_flag_value = raw[2] >> 12;
        if ir3_flag_value != ir3_flag_value.clamp(i64::from(i16::MIN), i64::from(i16::MAX)) {
            self.set_flag(22);
        }
        let sz = self.saturate_sz(raw[2] >> 12);
        self.push_sz(sz);
        let quotient = i64::from(self.divide(self.control[26] as u16, sz));

        self.set_mac0(quotient * ir[0] + Self::signed32(self.control[24]));
        let x = self.saturate_screen(Self::signed32(self.data[24]) >> 16, true);
        self.set_mac0(quotient * ir[1] + Self::signed32(self.control[25]));
        let y = self.saturate_screen(Self::signed32(self.data[24]) >> 16, false);
        self.push_sxy(x, y);

        let dqa = Self::s16(self.control[27]);
        self.set_mac0(quotient * dqa + Self::signed32(self.control[28]));
        let ir0 = self.saturate_ir0(Self::signed32(self.data[24]) >> 12);
        self.data[8] = u32::from(ir0);
    }

    fn nclip(&mut self) {
        let (x0, y0) = self.sxy(0);
        let (x1, y1) = self.sxy(1);
        let (x2, y2) = self.sxy(2);
        self.set_mac0(x0 * y1 + x1 * y2 + x2 * y0 - x0 * y2 - x1 * y0 - x2 * y1);
    }
    fn average_z(&mut self, four: bool) {
        let scale = Self::s16(self.control[if four { 30 } else { 29 }]);
        let start = if four { 16 } else { 17 };
        let sum: i64 = self.data[start..20]
            .iter()
            .map(|value| i64::from(value & 0xffff))
            .sum();
        let raw = scale * sum;
        self.set_mac0(raw);
        self.data[7] = u32::from(self.saturate_sz(raw >> 12));
    }

    fn garbage_matrix(&self) -> [[i64; 3]; 3] {
        let red = i64::from(self.data[6] & 0xff) << 4;
        let rt = self.matrix(0);
        [
            [-red, red, self.ir(0)],
            [rt[0][2], rt[0][2], rt[0][2]],
            [rt[1][1], rt[1][1], rt[1][1]],
        ]
    }

    fn mvmva(&mut self, instruction: u32) {
        let sf = instruction & (1 << 19) != 0;
        let mx = ((instruction >> 17) & 3) as usize;
        let v = ((instruction >> 15) & 3) as usize;
        let cv = ((instruction >> 13) & 3) as usize;
        let lm = instruction & (1 << 10) != 0;
        let matrix = match mx {
            0 => self.matrix(0),
            1 => self.matrix(8),
            2 => self.matrix(16),
            _ => self.garbage_matrix(),
        };
        let vector = self.vector(v);
        let raw = if cv == 2 {
            std::array::from_fn(|row| matrix[row][1] * vector[1] + matrix[row][2] * vector[2])
        } else {
            Self::multiply_raw(matrix, vector, self.translation(cv))
        };
        self.set_mac_ir(raw, if sf { 12 } else { 0 }, lm);
    }
    fn square(&mut self, instruction: u32) {
        let shift = if instruction & (1 << 19) != 0 { 12 } else { 0 };
        let raw = [self.ir(1).pow(2), self.ir(2).pow(2), self.ir(3).pow(2)];
        self.set_mac_ir(raw, shift, true);
    }

    fn outer_product(&mut self, instruction: u32) {
        let shift = if instruction & (1 << 19) != 0 { 12 } else { 0 };
        let lm = instruction & (1 << 10) != 0;
        let rt = self.matrix(0);
        let ir = [self.ir(1), self.ir(2), self.ir(3)];
        let diagonal = [rt[0][0], rt[1][1], rt[2][2]];
        let raw = [
            ir[2] * diagonal[1] - ir[1] * diagonal[2],
            ir[0] * diagonal[2] - ir[2] * diagonal[0],
            ir[1] * diagonal[0] - ir[0] * diagonal[1],
        ];
        self.set_mac_ir(raw, shift, lm);
    }

    fn light_vector(&mut self, vector: [i64; 3], shift: u32, lm: bool) {
        let raw = Self::multiply_raw(self.matrix(8), vector, [0; 3]);
        self.set_mac_ir(raw, shift, lm);
    }

    fn color_matrix(&mut self, shift: u32, lm: bool) {
        let raw = Self::multiply_raw(
            self.matrix(16),
            [self.ir(1), self.ir(2), self.ir(3)],
            self.translation(1),
        );
        self.set_mac_ir(raw, shift, lm);
    }
    fn primary_color_raw(&self, color: u32) -> [i64; 3] {
        let rgb = Self::color_channels(color);
        [
            rgb[0] * self.ir(1) * 16,
            rgb[1] * self.ir(2) * 16,
            rgb[2] * self.ir(3) * 16,
        ]
    }

    fn depth_cue_raw(&mut self, mac: [i64; 3], shift: u32, lm: bool) {
        let far = self.translation(2);
        let ir0 = self.ir(0);
        let mut delta = [0i64; 3];
        for index in 1..=3 {
            delta[index - 1] = self.set_ir(
                index,
                ((far[index - 1] << 12) - mac[index - 1]) >> shift,
                false,
            );
        }
        let raw = std::array::from_fn(|index| delta[index] * ir0 + mac[index]);
        self.set_mac_ir(raw, shift, lm);
    }

    fn normal_color(&mut self, vector: [i64; 3], instruction: u32, mode: u8) {
        let shift = if instruction & (1 << 19) != 0 { 12 } else { 0 };
        let lm = instruction & (1 << 10) != 0;
        self.light_vector(vector, shift, lm);
        self.color_matrix(shift, lm);
        if mode != 0 {
            let primary = self.primary_color_raw(self.data[6]);
            if mode == 2 {
                self.depth_cue_raw(primary, shift, lm);
            } else {
                self.set_mac_ir(primary, shift, lm);
            }
        }
        self.push_color();
    }
    fn color_color(&mut self, instruction: u32, depth: bool) {
        let shift = if instruction & (1 << 19) != 0 { 12 } else { 0 };
        let lm = instruction & (1 << 10) != 0;
        self.color_matrix(shift, lm);
        let primary = self.primary_color_raw(self.data[6]);
        if depth {
            self.depth_cue_raw(primary, shift, lm);
        } else {
            self.set_mac_ir(primary, shift, lm);
        }
        self.push_color();
    }

    fn depth_color(&mut self, instruction: u32, source: u8) {
        let shift = if instruction & (1 << 19) != 0 { 12 } else { 0 };
        let lm = instruction & (1 << 10) != 0;
        let raw = match source {
            0 => self.primary_color_raw(self.data[6]),
            1 => {
                let rgb = Self::color_channels(self.data[6]);
                [rgb[0] << 16, rgb[1] << 16, rgb[2] << 16]
            }
            2 => [self.ir(1) << 12, self.ir(2) << 12, self.ir(3) << 12],
            _ => [0; 3],
        };
        self.depth_cue_raw(raw, shift, lm);
        self.push_color();
    }

    fn dpct(&mut self, instruction: u32) {
        for _ in 0..3 {
            let rgb = Self::color_channels(self.data[20]);
            let raw = [rgb[0] << 16, rgb[1] << 16, rgb[2] << 16];
            let shift = if instruction & (1 << 19) != 0 { 12 } else { 0 };
            let lm = instruction & (1 << 10) != 0;
            self.depth_cue_raw(raw, shift, lm);
            self.push_color();
        }
    }
    fn general_interpolation(&mut self, instruction: u32, preserve_mac: bool) {
        let shift = if instruction & (1 << 19) != 0 { 12 } else { 0 };
        let lm = instruction & (1 << 10) != 0;
        let ir0 = self.ir(0);
        let raw = std::array::from_fn(|index| {
            let base = if preserve_mac {
                self.mac(index + 1) << shift
            } else {
                0
            };
            base + self.ir(index + 1) * ir0
        });
        self.set_mac_ir(raw, shift, lm);
        self.push_color();
    }

    fn command_cycles(opcode: u8) -> u32 {
        match opcode {
            0x01 => 15,
            0x06 | 0x10 | 0x11 | 0x12 | 0x29 => 8,
            0x0c => 6,
            0x13 => 19,
            0x14 => 13,
            0x16 => 44,
            0x1b => 17,
            0x1c => 11,
            0x1e => 14,
            0x20 => 30,
            0x28 | 0x3d | 0x3e => 5,
            0x2a => 17,
            0x2d => 5,
            0x2e => 6,
            0x30 => 23,
            0x3f => 39,
            _ => 1,
        }
    }
    fn execute_command_immediate(&mut self, instruction: u32) -> u32 {
        self.control[31] = 0;
        let opcode = (instruction & 0x3f) as u8;
        match opcode {
            0x01 => self.perspective(
                0,
                instruction & (1 << 19) != 0,
                instruction & (1 << 10) != 0,
            ),
            0x06 => self.nclip(),
            0x0c => self.outer_product(instruction),
            0x10 => self.depth_color(instruction, 1),
            0x11 => self.depth_color(instruction, 2),
            0x12 => self.mvmva(instruction),
            0x13 => self.normal_color(self.vector(0), instruction, 2),
            0x14 => self.color_color(instruction, true),
            0x16 => {
                for index in 0..3 {
                    self.normal_color(self.vector(index), instruction, 2);
                }
            }
            0x1b => self.normal_color(self.vector(0), instruction, 1),
            0x1c => self.color_color(instruction, false),
            0x1e => self.normal_color(self.vector(0), instruction, 0),
            0x20 => {
                for index in 0..3 {
                    self.normal_color(self.vector(index), instruction, 0);
                }
            }
            0x28 => self.square(instruction),
            0x29 => self.depth_color(instruction, 0),
            0x2a => self.dpct(instruction),
            0x2d => self.average_z(false),
            0x2e => self.average_z(true),
            0x30 => {
                for index in 0..3 {
                    self.perspective(
                        index,
                        instruction & (1 << 19) != 0,
                        instruction & (1 << 10) != 0,
                    );
                }
            }
            0x3d => self.general_interpolation(instruction, false),
            0x3e => self.general_interpolation(instruction, true),
            0x3f => {
                for index in 0..3 {
                    self.normal_color(self.vector(index), instruction, 1);
                }
            }
            _ => {}
        }
        self.finish_flags();
        Self::command_cycles(opcode)
    }

    fn perspective_data_latch_n(opcode: u8, register: u8) -> Option<u64> {
        match (opcode, register & 31) {
            (0x01, 0 | 1) => Some(0),
            (0x30, 0 | 1 | 3 | 5) => Some(0),
            (0x30, 2) => Some(3),
            (0x30, 4) => Some(2),
            _ => None,
        }
    }

    fn perspective_control_latch_n(opcode: u8, register: u8) -> Option<u64> {
        let register = register & 31;
        match opcode {
            0x01 => match register {
                0..=7 | 25 => Some(0),
                24 | 26 => Some(1),
                27 => Some(4),
                28 => Some(3),
                _ => None,
            },
            0x30 => match register {
                0 => Some(2),
                1 | 2 | 6 | 25 => Some(4),
                3 | 4 => Some(0),
                5 | 7 => Some(1),
                24 | 26 => Some(5),
                27 => Some(7),
                28 => Some(6),
                _ => None,
            },
            _ => None,
        }
    }

    fn deferred_data_latch_n(instruction: u32, register: u8) -> Option<u64> {
        let opcode = (instruction & 0x3f) as u8;
        if let Some(boundary) = Self::perspective_data_latch_n(opcode, register) {
            return Some(boundary);
        }
        match (opcode, register & 31) {
            (0x06, 12) => Some(0),
            (0x06, 13 | 14) => Some(1),
            (0x10, 6) => Some(0),
            (0x10, 8) => Some(1),
            (0x13 | 0x1b, 0) => Some(0),
            (0x13 | 0x1b, 1) => Some(1),
            (0x13 | 0x1b, 6) => Some(3),
            (0x1e, 0 | 1) => Some(0),
            (0x16, 0..=2) => Some(0),
            (0x16, 3) => Some(1),
            (0x16, 4) => Some(3),
            (0x16, 5) => Some(4),
            (0x16, 6) => Some(15),
            (0x20, 0 | 2) => Some(0),
            (0x20, 1) => Some(2),
            (0x20, 3 | 4) => Some(1),
            (0x20, 5) => Some(3),
            (0x2d, 17..=19) => Some(0),
            (0x2e, 16..=19) => Some(0),
            (0x3f, 0 | 2) => Some(0),
            (0x3f, 1) => Some(2),
            (0x3f, 3) => Some(1),
            (0x3f, 4 | 5) => Some(3),
            _ => None,
        }
    }

    fn deferred_control_latch_n(instruction: u32, register: u8) -> Option<u64> {
        let opcode = (instruction & 0x3f) as u8;
        if let Some(boundary) = Self::perspective_control_latch_n(opcode, register) {
            return Some(boundary);
        }
        let register = register & 31;
        match opcode {
            0x10 => match register {
                21..=23 => Some(0),
                _ => None,
            },
            0x13 | 0x1b | 0x1e => match register {
                8..=12 => Some(0),
                13 => Some(0),
                14 => Some(2),
                15 | 17 | 18 => Some(1),
                16 | 19 => Some(2),
                20 => Some(3),
                21 if opcode == 0x13 => Some(2),
                22 if opcode == 0x13 => Some(3),
                23 if opcode == 0x13 => Some(4),
                _ => None,
            },
            0x16 => match register {
                8 => Some(1),
                9 => Some(0),
                10 => Some(3),
                11 | 12 => Some(2),
                13..=15 => Some(7),
                16 | 20 => Some(5),
                17 => Some(4),
                18 | 19 => Some(7),
                21 => Some(13),
                22 | 23 => Some(14),
                _ => None,
            },
            0x20 => match register {
                8 | 9 | 11 | 12 => Some(0),
                10 => Some(3),
                13 => Some(8),
                14 | 15 => Some(9),
                16 | 20 => Some(6),
                17 => Some(3),
                18 | 19 => Some(8),
                _ => None,
            },
            0x2d if register == 29 => Some(0),
            0x2e if register == 30 => Some(0),
            0x3f => match register {
                8 | 11 | 12 => Some(1),
                9 => Some(0),
                10 => Some(3),
                13 | 15 => Some(9),
                14 => Some(6),
                16 | 20 => Some(5),
                17 => Some(3),
                18 => Some(8),
                19 => Some(7),
                _ => None,
            },
            _ => None,
        }
    }

    fn command_has_nonaliasing_latch_model(instruction: u32) -> bool {
        matches!(
            instruction & 0x3f,
            0x01 | 0x06 | 0x10 | 0x13 | 0x16 | 0x1b | 0x1e | 0x20 | 0x2d | 0x2e | 0x30 | 0x3f
        )
    }

    fn write_reaches_pending_input(issue_cycle: u64, latch_n: u64, write_cycle: u64) -> bool {
        write_cycle < issue_cycle.saturating_add(3).saturating_add(latch_n)
    }

    pub fn write_data_at(&mut self, register: u8, value: u32, cycle: u64) {
        self.write_data(register, value);
        let Some(pending) = self.pending_command.as_mut() else {
            return;
        };
        let Some(latch_n) = Self::deferred_data_latch_n(pending.instruction, register) else {
            return;
        };
        if Self::write_reaches_pending_input(pending.issue_cycle, latch_n, cycle) {
            Self::write_data_regs(&mut pending.data, register, value);
        }
    }

    pub fn write_control_at(&mut self, register: u8, value: u32, cycle: u64) {
        self.write_control(register, value);
        let Some(pending) = self.pending_command.as_mut() else {
            return;
        };
        let Some(latch_n) = Self::deferred_control_latch_n(pending.instruction, register) else {
            return;
        };
        if Self::write_reaches_pending_input(pending.issue_cycle, latch_n, cycle) {
            Self::write_control_regs(&mut pending.control, register, value);
        }
    }

    fn commit_deferred_result(&mut self, instruction: u32, result: &Ps1Gte) {
        match instruction & 0x3f {
            0x01 | 0x30 => {
                for index in 8..=14 {
                    self.data[index] = result.data[index];
                }
                for index in 16..=19 {
                    self.data[index] = result.data[index];
                }
                for index in 24..=27 {
                    self.data[index] = result.data[index];
                }
            }
            0x06 => self.data[24] = result.data[24],
            0x10 | 0x13 | 0x16 | 0x1b | 0x1e | 0x20 | 0x3f => {
                for index in 9..=11 {
                    self.data[index] = result.data[index];
                }
                for index in 20..=22 {
                    self.data[index] = result.data[index];
                }
                for index in 25..=27 {
                    self.data[index] = result.data[index];
                }
            }
            0x2d | 0x2e => {
                self.data[7] = result.data[7];
                self.data[24] = result.data[24];
            }
            _ => {}
        }
        self.control[31] = result.control[31];
    }

    pub fn advance_to(&mut self, cycle: u64) {
        let should_complete = self
            .pending_command
            .as_ref()
            .is_some_and(|pending| pending.complete_cycle <= cycle);
        if !should_complete {
            return;
        }
        let Some(pending) = self.pending_command.take() else {
            return;
        };
        let mut result = Ps1Gte {
            data: pending.data,
            control: pending.control,
            pending_command: None,
        };
        result.execute_command_immediate(pending.instruction);
        self.commit_deferred_result(pending.instruction, &result);
    }

    pub fn command_at(&mut self, instruction: u32, issue_cycle: u64) -> u32 {
        self.advance_to(issue_cycle);
        let opcode = (instruction & 0x3f) as u8;
        let cycles = Self::command_cycles(opcode);
        if Self::command_has_nonaliasing_latch_model(instruction) {
            self.pending_command = Some(PendingGteCommand {
                instruction,
                issue_cycle,
                complete_cycle: issue_cycle.saturating_add(u64::from(cycles)),
                data: self.data,
                control: self.control,
            });
            cycles
        } else {
            self.execute_command_immediate(instruction)
        }
    }

    pub fn command(&mut self, instruction: u32) -> u32 {
        self.execute_command_immediate(instruction)
    }

    pub fn save(&self, out: &mut StateWriter) {
        for value in self.data {
            out.u32(value);
        }
        for value in self.control {
            out.u32(value);
        }
        match &self.pending_command {
            Some(pending) => {
                out.u8(1);
                out.u32(pending.instruction);
                out.u64(pending.issue_cycle);
                out.u64(pending.complete_cycle);
                for value in pending.data {
                    out.u32(value);
                }
                for value in pending.control {
                    out.u32(value);
                }
            }
            None => out.u8(0),
        }
    }

    pub fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        for value in &mut self.data {
            *value = input.u32()?;
        }
        for value in &mut self.control {
            *value = input.u32()?;
        }
        self.pending_command = if input.u8()? != 0 {
            let instruction = input.u32()?;
            let issue_cycle = input.u64()?;
            let complete_cycle = input.u64()?;
            if !Self::command_has_nonaliasing_latch_model(instruction)
                || complete_cycle < issue_cycle
            {
                return Err("invalid pending PlayStation GTE command state".into());
            }
            let mut data = [0u32; 32];
            for value in &mut data {
                *value = input.u32()?;
            }
            let mut control = [0u32; 32];
            for value in &mut control {
                *value = input.u32()?;
            }
            Some(PendingGteCommand {
                instruction,
                issue_cycle,
                complete_cycle,
                data,
                control,
            })
        } else {
            None
        };
        self.data[1] &= 0xffff;
        self.data[3] &= 0xffff;
        self.data[5] &= 0xffff;
        self.data[7] &= 0xffff;
        for index in 8..=11 {
            self.data[index] &= 0xffff;
        }
        for index in 16..=19 {
            self.data[index] &= 0xffff;
        }
        self.control[31] &= 0x7fff_f000;
        self.finish_flags();
        Ok(())
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use crate::platform::PlatformId;

    fn set_identity(gte: &mut Ps1Gte, base: u8) {
        gte.write_control(base, 0x0000_1000);
        gte.write_control(base + 1, 0);
        gte.write_control(base + 2, 0x0000_1000);
        gte.write_control(base + 3, 0);
        gte.write_control(base + 4, 0x0000_1000);
    }

    #[test]
    fn special_register_access_matches_gte_aliases() {
        let mut gte = Ps1Gte::new();
        gte.write_data(1, 0x0000_ffff);
        assert_eq!(gte.read_data(1), 0xffff_ffff);
        gte.write_data(15, 0x0002_0001);
        assert_eq!(gte.read_data(14), 0x0002_0001);
        assert_eq!(gte.read_data(15), 0x0002_0001);

        gte.write_data(28, 0x7c1f);
        assert_eq!(gte.read_data(9), 0x0f80);
        assert_eq!(gte.read_data(10), 0);
        assert_eq!(gte.read_data(11), 0x0f80);
        assert_eq!(gte.read_data(29), 0x7c1f);

        gte.write_data(30, 0x00ff_ffff);
        assert_eq!(gte.read_data(31), 8);
        gte.write_data(30, 0xffff_ffff);
        assert_eq!(gte.read_data(31), 32);
    }
    #[test]
    fn control_register_widths_include_h_read_sign_extension_bug() {
        let mut gte = Ps1Gte::new();
        gte.write_control(26, 0x1234_8000);
        assert_eq!(gte.read_control(26), 0xffff_8000);
        gte.write_control(27, 0x1234_ffff);
        assert_eq!(gte.read_control(27), 0xffff_ffff);
        gte.write_control(31, 0xffff_ffff);
        assert_eq!(gte.read_control(31), 0xffff_f000);
    }

    #[test]
    fn mvmva_identity_preserves_vector_with_fraction_shift() {
        let mut gte = Ps1Gte::new();
        set_identity(&mut gte, 0);
        gte.write_data(
            0,
            u32::from(0x0200u16) | (u32::from((-0x100i16) as u16) << 16),
        );
        gte.write_data(1, 0x0000_0400);
        assert_eq!(gte.command((1 << 19) | 0x12), 8);
        assert_eq!(gte.read_data(9), 0x0000_0200);
        assert_eq!(gte.read_data(10), 0xffff_ff00);
        assert_eq!(gte.read_data(11), 0x0000_0400);
    }
    #[test]
    fn rtps_identity_projects_and_updates_depth_and_screen_fifos() {
        let mut gte = Ps1Gte::new();
        set_identity(&mut gte, 0);
        gte.write_data(
            0,
            u32::from(0x0100u16) | (u32::from((-0x80i16) as u16) << 16),
        );
        gte.write_data(1, 0x0000_1000);
        gte.write_control(26, 0x1000);

        assert_eq!(gte.command((1 << 19) | 0x01), 15);
        assert_eq!(gte.read_data(19), 0x1000);
        assert_eq!(gte.read_data(14), 0xff80_0100);
        assert_eq!(gte.read_data(9), 0x0000_0100);
        assert_eq!(gte.read_data(10), 0xffff_ff80);
        assert_eq!(gte.read_data(11), 0x0000_1000);
        assert_eq!(gte.read_data(8), 0);
    }

    #[test]
    fn rtps_dqa_write_respects_measured_input_latch_boundary() {
        let run = |write_cycle| {
            let mut gte = Ps1Gte::new();
            set_identity(&mut gte, 0);
            gte.write_data(0, 0);
            gte.write_data(1, 0x1000);
            gte.write_control(26, 0x1000);
            assert_eq!(gte.command_at((1 << 19) | 0x01, 100), 15);
            gte.write_control_at(27, 0x1000, write_cycle);
            gte.advance_to(115);
            (gte.read_data(8), gte.read_control(27))
        };

        let (early_ir0, early_dqa) = run(106);
        assert_eq!(early_ir0, 0x1000);
        assert_eq!(early_dqa, 0x1000);

        let (late_ir0, late_dqa) = run(107);
        assert_eq!(late_ir0, 0);
        assert_eq!(late_dqa, 0x1000);
    }

    #[test]
    fn rtpt_vxy1_write_respects_measured_input_latch_boundary() {
        let run = |write_cycle| {
            let mut gte = Ps1Gte::new();
            set_identity(&mut gte, 0);
            for index in 0..3 {
                gte.write_data((index * 2) as u8, 0);
                gte.write_data((index * 2 + 1) as u8, 0x1000);
            }
            gte.write_control(26, 0x1000);
            assert_eq!(gte.command_at((1 << 19) | 0x30, 200), 23);
            gte.write_data_at(2, 0x0100, write_cycle);
            gte.advance_to(223);
            (gte.read_data(13) & 0xffff, gte.read_data(2))
        };

        let (early_sx1, early_vxy1) = run(205);
        assert_eq!(early_sx1, 0x0100);
        assert_eq!(early_vxy1, 0x0100);

        let (late_sx1, late_vxy1) = run(206);
        assert_eq!(late_sx1, 0);
        assert_eq!(late_vxy1, 0x0100);
    }

    #[test]
    fn rtps_ir3_flag_uses_unshifted_depth_rule_independent_of_lm() {
        let mut gte = Ps1Gte::new();
        set_identity(&mut gte, 0);
        gte.write_data(0, 0);
        gte.write_data(1, u32::from((-1i16) as u16));
        gte.write_control(26, 1);
        gte.command((1 << 19) | (1 << 10) | 0x01);
        assert_eq!(gte.read_data(11), 0);
        assert_eq!(gte.read_control(31) & (1 << 22), 0);

        gte.reset();
        set_identity(&mut gte, 0);
        gte.write_data(0, 0);
        gte.write_data(1, 0x0100);
        gte.command(0x01);
        assert_eq!(gte.read_data(11), 0x7fff);
        assert_eq!(gte.read_control(31) & (1 << 22), 0);
    }

    #[test]
    fn nct_identity_matrices_fill_color_fifo_in_vertex_order() {
        let mut gte = Ps1Gte::new();
        set_identity(&mut gte, 8);
        set_identity(&mut gte, 16);
        gte.write_data(6, 0x2400_0000);
        for (vector, value) in [0x0400u16, 0x0800, 0x0c00].into_iter().enumerate() {
            gte.write_data(
                (vector * 2) as u8,
                u32::from(value) | (u32::from(value) << 16),
            );
            gte.write_data((vector * 2 + 1) as u8, u32::from(value));
        }
        assert_eq!(gte.command((1 << 19) | (1 << 10) | 0x20), 30);
        assert_eq!(gte.read_data(20), 0x2440_4040);
        assert_eq!(gte.read_data(21), 0x2480_8080);
        assert_eq!(gte.read_data(22), 0x24c0_c0c0);
    }

    #[test]
    fn dpcs_preserves_color_when_far_color_matches_source() {
        let mut gte = Ps1Gte::new();
        gte.write_data(6, 0x2c30_2010);
        gte.write_data(8, 0x1000);
        gte.write_control(21, 0x0100);
        gte.write_control(22, 0x0200);
        gte.write_control(23, 0x0300);
        assert_eq!(gte.command((1 << 19) | 0x10), 8);
        assert_eq!(gte.read_data(22), 0x2c30_2010);
    }

    #[test]
    fn nclip_and_average_z_use_fifo_values() {
        let mut gte = Ps1Gte::new();
        gte.write_data(12, 0x0000_0000);
        gte.write_data(13, 0x0000_0004);
        gte.write_data(14, 0x0003_0000);
        assert_eq!(gte.command(0x06), 8);
        assert_eq!(gte.read_data(24), 12);
        gte.write_data(17, 0x0100);
        gte.write_data(18, 0x0200);
        gte.write_data(19, 0x0300);
        gte.write_control(29, 0x0555);
        assert_eq!(gte.command(0x2d), 5);
        assert_eq!(gte.read_data(7), 0x01ff);
    }

    #[test]
    fn nclip_sxy1_write_respects_measured_input_latch_boundary() {
        let run = |write_cycle| {
            let mut gte = Ps1Gte::new();
            gte.write_data(12, 0x0000_0000);
            gte.write_data(13, 0x0000_0004);
            gte.write_data(14, 0x0003_0000);
            assert_eq!(gte.command_at(0x06, 100), 8);
            gte.write_data_at(13, 0x0000_0008, write_cycle);
            gte.advance_to(108);
            (gte.read_data(24), gte.read_data(13))
        };

        let (early_mac0, early_sxy1) = run(103);
        assert_eq!(early_mac0, 24);
        assert_eq!(early_sxy1, 8);

        let (late_mac0, late_sxy1) = run(104);
        assert_eq!(late_mac0, 12);
        assert_eq!(late_sxy1, 8);
    }

    #[test]
    fn nct_vz2_write_respects_measured_input_latch_boundary() {
        let run = |write_cycle| {
            let mut gte = Ps1Gte::new();
            set_identity(&mut gte, 8);
            set_identity(&mut gte, 16);
            gte.write_data(6, 0x2400_0000);
            for vector in 0..3 {
                gte.write_data((vector * 2) as u8, 0x0400_0400);
                gte.write_data((vector * 2 + 1) as u8, 0x0400);
            }
            assert_eq!(gte.command_at((1 << 19) | (1 << 10) | 0x20, 100), 30);
            gte.write_data_at(5, 0x0800, write_cycle);
            gte.advance_to(130);
            (gte.read_data(22) & 0x00ff_0000, gte.read_data(5))
        };

        let (early_blue, early_vz2) = run(105);
        assert_eq!(early_blue, 0x0080_0000);
        assert_eq!(early_vz2, 0x0800);

        let (late_blue, late_vz2) = run(106);
        assert_eq!(late_blue, 0x0040_0000);
        assert_eq!(late_vz2, 0x0800);
    }

    #[test]
    fn saturation_flags_follow_error_summary_mask() {
        let mut gte = Ps1Gte::new();
        set_identity(&mut gte, 8);
        set_identity(&mut gte, 16);
        gte.write_data(6, 0x2400_0000);
        gte.write_data(0, 0x2000_2000);
        gte.write_data(1, 0x2000);
        gte.command((1 << 19) | (1 << 10) | 0x1e);
        let color_flags = gte.read_control(31);
        assert_eq!(
            color_flags & ((1 << 21) | (1 << 20) | (1 << 19)),
            (1 << 21) | (1 << 20) | (1 << 19)
        );
        assert_eq!(color_flags & (1 << 31), 0);

        gte.write_data(9, 0x7fff);
        gte.write_data(10, 0x7fff);
        gte.write_data(11, 0x7fff);
        gte.command(0x28);
        let ir_flags = gte.read_control(31);
        assert_ne!(ir_flags & ((1 << 24) | (1 << 23) | (1 << 22)), 0);
        assert_ne!(ir_flags & (1 << 31), 0);
    }

    #[test]
    fn divide_matches_documented_unr_path_and_overflow_flag() {
        let mut gte = Ps1Gte::new();
        assert_eq!(gte.divide(0x1000, 0x1000), 0x1_0000);
        assert_eq!(gte.read_control(31) & (1 << 17), 0);
        assert_eq!(gte.divide(0x1000, 0x0800), 0x1_ffff);
        assert_ne!(gte.read_control(31) & (1 << 17), 0);
    }

    #[test]
    fn pending_rtps_state_round_trip_preserves_latched_inputs() {
        let mut gte = Ps1Gte::new();
        set_identity(&mut gte, 0);
        gte.write_data(0, 0);
        gte.write_data(1, 0x1000);
        gte.write_control(26, 0x1000);
        assert_eq!(gte.command_at((1 << 19) | 0x01, 100), 15);
        gte.write_control_at(27, 0x1000, 106);

        let mut out = StateWriter::new(PlatformId::PlayStation, 1);
        gte.save(&mut out);
        let bytes = out.finish();
        let mut input = StateReader::new(&bytes, PlatformId::PlayStation, 1).unwrap();
        let mut restored = Ps1Gte::new();
        restored.load(&mut input).unwrap();
        input.finish().unwrap();
        restored.advance_to(115);

        assert_eq!(restored.read_data(8), 0x1000);
        assert_eq!(restored.read_control(27), 0x1000);
    }

    #[test]
    fn state_round_trip_preserves_registers() {
        let mut gte = Ps1Gte::new();
        gte.write_data(0, 0x1234_5678);
        gte.write_data(28, 0x4210);
        gte.write_control(5, 0x89ab_cdef);
        gte.write_control(31, 1 << 24);
        let mut out = StateWriter::new(PlatformId::PlayStation, 1);
        gte.save(&mut out);
        let bytes = out.finish();
        let mut input = StateReader::new(&bytes, PlatformId::PlayStation, 1).unwrap();
        let mut restored = Ps1Gte::new();
        restored.load(&mut input).unwrap();
        input.finish().unwrap();
        assert_eq!(restored.read_data(0), 0x1234_5678);
        assert_eq!(restored.read_data(28), 0x4210);
        assert_eq!(restored.read_control(5), 0x89ab_cdef);
        assert_ne!(restored.read_control(31) & (1 << 24), 0);
    }
}
