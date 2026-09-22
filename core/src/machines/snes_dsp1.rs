use crate::state::{StateReader, StateWriter};

const PARAMETER_CAPACITY: usize = 16;
const OUTPUT_CAPACITY: usize = 2048;

#[derive(Clone, Copy, Default)]
struct ProjectionState {
    fx: i16,
    fy: i16,
    fz: i16,
    lfe: i16,
    les: i16,
    aas: i16,
    azs: i16,
    centre_x: f64,
    centre_y: f64,
    centre_z: f64,
    eye_x: f64,
    eye_y: f64,
    eye_z: f64,
    sin_aas: f64,
    cos_aas: f64,
    sin_azs: f64,
    cos_azs: f64,
}
#[derive(Clone)]
pub(crate) struct Dsp1 {
    waiting_for_command: bool,
    command: u8,
    expected: u8,
    input_len: u8,
    parameters: [u8; PARAMETER_CAPACITY],
    output: [u8; OUTPUT_CAPACITY],
    output_len: u16,
    output_index: u16,
    matrices: [[[i32; 3]; 3]; 3],
    projection: ProjectionState,
    raster_vs: i16,
    raster_streaming: bool,
}

impl Default for Dsp1 {
    fn default() -> Self {
        Self {
            waiting_for_command: true,
            command: 0,
            expected: 0,
            input_len: 0,
            parameters: [0; PARAMETER_CAPACITY],
            output: [0; OUTPUT_CAPACITY],
            output_len: 0,
            output_index: 0,
            matrices: [[[0; 3]; 3]; 3],
            projection: ProjectionState::default(),
            raster_vs: 0,
            raster_streaming: false,
        }
    }
}

impl Dsp1 {
    pub(crate) fn status(&self) -> u8 {
        0x80
    }

    pub(crate) fn reset(&mut self) {
        *self = Self::default();
    }

    pub(crate) fn write_data(&mut self, value: u8) {
        if self.raster_streaming && self.output_index < self.output_len {
            self.output_len = 0;
            self.output_index = 0;
            self.raster_streaming = false;
            self.waiting_for_command = true;
        }
        if self.waiting_for_command {
            if value == 0x80 {
                return;
            }
            let command = Self::canonical_command(value);
            let Some(expected) = Self::parameter_bytes(command) else {
                return;
            };
            self.command = command;
            self.expected = expected;
            self.input_len = 0;
            self.waiting_for_command = expected == 0;
            self.output_len = 0;
            self.output_index = 0;
            self.raster_streaming = false;
            if expected == 0 {
                self.execute();
            }
            return;
        }

        if usize::from(self.input_len) < self.parameters.len() {
            self.parameters[usize::from(self.input_len)] = value;
        }
        self.input_len = self.input_len.saturating_add(1);
        if self.input_len >= self.expected {
            self.waiting_for_command = true;
            self.execute();
        }
    }

    pub(crate) fn read_data(&mut self) -> u8 {
        if self.output_index >= self.output_len {
            if self.raster_streaming {
                self.emit_raster();
            } else {
                return 0x80;
            }
        }
        let value = self.output[usize::from(self.output_index)];
        self.output_index += 1;
        value
    }

    fn canonical_command(command: u8) -> u8 {
        match command {
            0x30 => 0x10,
            0x24 => 0x04,
            0x2c => 0x0c,
            0x3c => 0x1c,
            0x12 | 0x22 | 0x32 => 0x02,
            0x1a | 0x2a | 0x3a => 0x0a,
            0x16 | 0x26 | 0x36 => 0x06,
            0x1e | 0x2e | 0x3e => 0x0e,
            0x05 | 0x31 | 0x35 => 0x01,
            0x15 => 0x11,
            0x25 => 0x21,
            0x09 | 0x39 | 0x3d => 0x0d,
            0x19 => 0x1d,
            0x29 => 0x2d,
            0x33 => 0x03,
            0x3b => 0x0b,
            0x34 => 0x14,
            0x07 => 0x0f,
            0x27 => 0x2f,
            0x17 | 0x37 | 0x3f => 0x1f,
            other => other,
        }
    }

    fn parameter_bytes(command: u8) -> Option<u8> {
        let words = match command {
            0x00 | 0x10 | 0x20 | 0x04 | 0x0e => 2,
            0x08 | 0x28 | 0x0c | 0x06 | 0x0d | 0x1d | 0x2d | 0x03 | 0x13 | 0x23 | 0x0b | 0x1b
            | 0x2b => 3,
            0x18 | 0x38 | 0x01 | 0x11 | 0x21 => 4,
            0x1c | 0x14 => 6,
            0x02 => 7,
            0x0a | 0x0f | 0x2f | 0x1f => 1,
            _ => return None,
        };
        Some(words * 2)
    }

    fn word(&self, index: usize) -> i16 {
        let offset = index * 2;
        i16::from_le_bytes([self.parameters[offset], self.parameters[offset + 1]])
    }

    fn set_output_words(&mut self, words: &[i16]) {
        self.output_len = (words.len() * 2) as u16;
        self.output_index = 0;
        for (index, word) in words.iter().enumerate() {
            let bytes = word.to_le_bytes();
            self.output[index * 2] = bytes[0];
            self.output[index * 2 + 1] = bytes[1];
        }
    }

    fn angle(angle: i16) -> f64 {
        f64::from(angle) * std::f64::consts::TAU / 65_536.0
    }

    fn clamp_i16(value: f64) -> i16 {
        value
            .round()
            .clamp(f64::from(i16::MIN), f64::from(i16::MAX)) as i16
    }

    fn q15(value: f64) -> i32 {
        (value * 32768.0).round().clamp(-32768.0, 32767.0) as i32
    }

    fn mul_q15(a: i32, b: i32) -> i32 {
        ((i64::from(a) * i64::from(b)) >> 15) as i32
    }

    fn sin_q15(angle: i16) -> i32 {
        Self::q15(Self::angle(angle).sin())
    }

    fn cos_q15(angle: i16) -> i32 {
        Self::q15(Self::angle(angle).cos())
    }

    fn set_matrix(&mut self, matrix: usize, scale: i16, zr: i16, yr: i16, xr: i16) {
        let scale = f64::from(scale) / 2.0;
        let (sz, cz) = Self::angle(zr).sin_cos();
        let (sy, cy) = Self::angle(yr).sin_cos();
        let (sx, cx) = Self::angle(xr).sin_cos();
        let m = &mut self.matrices[matrix];
        m[0][0] = (scale * cz * cy).round() as i32;
        m[0][1] = (-scale * sz * cy).round() as i32;
        m[0][2] = (scale * sy).round() as i32;
        m[1][0] = (scale * (sz * cx + cz * sx * sy)).round() as i32;
        m[1][1] = (scale * (cz * cx - sz * sx * sy)).round() as i32;
        m[1][2] = (-scale * sx * cy).round() as i32;
        m[2][0] = (scale * (sz * sx - cz * cx * sy)).round() as i32;
        m[2][1] = (scale * (cz * sx + sz * cx * sy)).round() as i32;
        m[2][2] = (scale * cx * cy).round() as i32;
    }

    fn objective(&self, matrix: usize, x: i16, y: i16, z: i16) -> [i16; 3] {
        let vector = [i64::from(x), i64::from(y), i64::from(z)];
        let mut result = [0i16; 3];
        for (row, output) in result.iter_mut().enumerate() {
            let sum = vector
                .iter()
                .enumerate()
                .map(|(column, value)| value * i64::from(self.matrices[matrix][row][column]))
                .sum::<i64>();
            *output = (sum >> 15) as i16;
        }
        result
    }

    fn subjective(&self, matrix: usize, f: i16, l: i16, u: i16) -> [i16; 3] {
        let vector = [i64::from(f), i64::from(l), i64::from(u)];
        let mut result = [0i16; 3];
        for (column, output) in result.iter_mut().enumerate() {
            let sum = vector
                .iter()
                .enumerate()
                .map(|(row, value)| value * i64::from(self.matrices[matrix][row][column]))
                .sum::<i64>();
            *output = (sum >> 15) as i16;
        }
        result
    }

    fn scalar(&self, matrix: usize, x: i16, y: i16, z: i16) -> i16 {
        self.objective(matrix, x, y, z)[0]
    }

    fn configure_projection(&mut self, args: [i16; 7]) -> [i16; 4] {
        let [fx, fy, fz, lfe, les, aas, azs] = args;
        let mut p = ProjectionState {
            fx,
            fy,
            fz,
            lfe,
            les,
            aas,
            azs,
            ..ProjectionState::default()
        };
        let (sin_aas, cos_aas) = Self::angle(aas).sin_cos();
        let (sin_azs, cos_azs) = Self::angle(azs).sin_cos();
        p.sin_aas = sin_aas;
        p.cos_aas = cos_aas;
        p.sin_azs = sin_azs;
        p.cos_azs = cos_azs;
        let nx = -sin_azs * sin_aas;
        let ny = sin_azs * cos_aas;
        let nz = cos_azs;
        p.centre_x = f64::from(fx) + f64::from(lfe) * nx;
        p.centre_y = f64::from(fy) + f64::from(lfe) * ny;
        p.centre_z = f64::from(fz) + f64::from(lfe) * nz;
        p.eye_x = p.centre_x - f64::from(les) * nx;
        p.eye_y = p.centre_y - f64::from(les) * ny;
        p.eye_z = p.centre_z - f64::from(les) * nz;
        let vva = if sin_azs.abs() > 1.0e-9 {
            -f64::from(les) * cos_azs / sin_azs
        } else {
            0.0
        };
        let result = [
            0,
            Self::clamp_i16(vva),
            Self::clamp_i16(p.centre_x),
            Self::clamp_i16(p.centre_y),
        ];
        self.projection = p;
        result
    }

    fn raster(&self, vs: i16) -> [i16; 4] {
        let p = self.projection;
        let denominator = f64::from(vs) * p.sin_azs + f64::from(p.les) * p.cos_azs;
        if denominator.abs() < 1.0e-9 {
            return [0; 4];
        }
        let scale = p.centre_z / denominator;
        let sec = if p.cos_azs.abs() > 1.0e-9 {
            1.0 / p.cos_azs
        } else {
            0.0
        };
        [
            Self::clamp_i16(scale * p.cos_aas * 32768.0),
            Self::clamp_i16(-scale * sec * p.sin_aas * 32768.0),
            Self::clamp_i16(scale * p.sin_aas * 32768.0),
            Self::clamp_i16(scale * sec * p.cos_aas * 32768.0),
        ]
    }

    fn project(&self, x: i16, y: i16, z: i16) -> [i16; 3] {
        let p = self.projection;
        let rx = f64::from(x) - p.eye_x;
        let ry = f64::from(y) - p.eye_y;
        let rz = f64::from(z) - p.eye_z;
        let nx = -p.sin_azs * p.sin_aas;
        let ny = p.sin_azs * p.cos_aas;
        let nz = p.cos_azs;
        let denominator = f64::from(p.les) - (rx * nx + ry * ny + rz * nz);
        if denominator.abs() < 1.0e-9 {
            return [0, 0, i16::MAX];
        }
        let scale = f64::from(p.les) / denominator;
        let horizontal = rx * p.cos_aas + ry * p.sin_aas;
        let vertical =
            rx * (-p.cos_azs * p.sin_aas) + ry * (p.cos_azs * p.cos_aas) - rz * p.sin_azs;
        [
            Self::clamp_i16(horizontal * scale),
            Self::clamp_i16(vertical * scale),
            Self::clamp_i16(scale * 256.0),
        ]
    }

    fn target(&self, h: i16, v: i16) -> [i16; 2] {
        let p = self.projection;
        let denominator = f64::from(v) * p.sin_azs + f64::from(p.les) * p.cos_azs;
        if denominator.abs() < 1.0e-9 {
            return [i16::MAX, i16::MAX];
        }
        let scale = p.centre_z / denominator;
        let sec = if p.cos_azs.abs() > 1.0e-9 {
            1.0 / p.cos_azs
        } else {
            0.0
        };
        let x =
            p.centre_x + scale * f64::from(h) * p.cos_aas - scale * sec * f64::from(v) * p.sin_aas;
        let y =
            p.centre_y - scale * f64::from(h) * p.sin_aas + scale * sec * f64::from(v) * p.cos_aas;
        [Self::clamp_i16(x), Self::clamp_i16(y)]
    }

    fn inverse(coefficient: i16, exponent: i16) -> [i16; 2] {
        if coefficient == 0 {
            return [0x7fff, 0x002f];
        }
        let value = (f64::from(coefficient) / 32768.0) * 2f64.powi(i32::from(exponent));
        let inverse = 1.0 / value;
        if !inverse.is_finite() {
            return [0x7fff, 0x002f];
        }
        let output_exponent = inverse.abs().log2().floor() as i32 + 1;
        let normalized = inverse / 2f64.powi(output_exponent);
        [
            Self::clamp_i16(normalized * 32768.0),
            output_exponent.clamp(i32::from(i16::MIN), i32::from(i16::MAX)) as i16,
        ]
    }

    fn emit_raster(&mut self) {
        let values = self.raster(self.raster_vs);
        self.raster_vs = self.raster_vs.wrapping_add(1);
        self.set_output_words(&values);
        self.raster_streaming = true;
    }

    fn execute(&mut self) {
        self.raster_streaming = false;
        match self.command {
            0x00 | 0x20 => {
                let product = (i32::from(self.word(0)) * i32::from(self.word(1))) >> 15;
                let product = product + i32::from(self.command == 0x20);
                self.set_output_words(&[product as i16]);
            }
            0x10 => {
                let result = Self::inverse(self.word(0), self.word(1));
                self.set_output_words(&result);
            }
            0x04 => {
                let angle = self.word(0);
                let radius = i32::from(self.word(1));
                let sin = Self::mul_q15(radius, Self::sin_q15(angle));
                let cos = Self::mul_q15(radius, Self::cos_q15(angle));
                self.set_output_words(&[sin as i16, cos as i16]);
            }
            0x08 => {
                let x = i64::from(self.word(0));
                let y = i64::from(self.word(1));
                let z = i64::from(self.word(2));
                let length = ((x * x + y * y + z * z) << 1) as u32;
                self.set_output_words(&[length as u16 as i16, (length >> 16) as u16 as i16]);
            }
            0x18 | 0x38 => {
                let x = i64::from(self.word(0));
                let y = i64::from(self.word(1));
                let z = i64::from(self.word(2));
                let radius = i64::from(self.word(3));
                let mut difference = ((x * x + y * y + z * z - radius * radius) >> 15) as i32;
                if self.command == 0x38 {
                    difference = difference.wrapping_add(1);
                }
                self.set_output_words(&[difference as i16]);
            }
            0x28 => {
                let x = f64::from(self.word(0));
                let y = f64::from(self.word(1));
                let z = f64::from(self.word(2));
                self.set_output_words(&[Self::clamp_i16((x * x + y * y + z * z).sqrt())]);
            }
            0x0c => {
                let angle = Self::angle(self.word(0));
                let (sin, cos) = angle.sin_cos();
                let x = f64::from(self.word(1));
                let y = f64::from(self.word(2));
                self.set_output_words(&[
                    Self::clamp_i16(y * sin + x * cos),
                    Self::clamp_i16(y * cos - x * sin),
                ]);
            }
            0x1c => {
                let (sz, cz) = Self::angle(self.word(0)).sin_cos();
                let (sy, cy) = Self::angle(self.word(1)).sin_cos();
                let (sx, cx) = Self::angle(self.word(2)).sin_cos();
                let mut x = f64::from(self.word(3));
                let mut y = f64::from(self.word(4));
                let mut z = f64::from(self.word(5));
                (x, y) = (y * sz + x * cz, y * cz - x * sz);
                (z, x) = (x * sy + z * cy, x * cy - z * sy);
                (y, z) = (z * sx + y * cx, z * cx - y * sx);
                self.set_output_words(&[
                    Self::clamp_i16(x),
                    Self::clamp_i16(y),
                    Self::clamp_i16(z),
                ]);
            }
            0x02 => {
                let args = [
                    self.word(0),
                    self.word(1),
                    self.word(2),
                    self.word(3),
                    self.word(4),
                    self.word(5),
                    self.word(6),
                ];
                let result = self.configure_projection(args);
                self.set_output_words(&result);
            }
            0x0a => {
                self.raster_vs = self.word(0);
                self.emit_raster();
            }
            0x06 => {
                let result = self.project(self.word(0), self.word(1), self.word(2));
                self.set_output_words(&result);
            }
            0x0e => {
                let result = self.target(self.word(0), self.word(1));
                self.set_output_words(&result);
            }
            0x01 | 0x11 | 0x21 => {
                let matrix = usize::from(self.command >> 4);
                let args = [self.word(0), self.word(1), self.word(2), self.word(3)];
                self.set_matrix(matrix, args[0], args[1], args[2], args[3]);
                self.output_len = 0;
                self.output_index = 0;
            }
            0x0d | 0x1d | 0x2d => {
                let matrix = usize::from(self.command >> 4);
                let result = self.objective(matrix, self.word(0), self.word(1), self.word(2));
                self.set_output_words(&result);
            }
            0x03 | 0x13 | 0x23 => {
                let matrix = usize::from(self.command >> 4);
                let result = self.subjective(matrix, self.word(0), self.word(1), self.word(2));
                self.set_output_words(&result);
            }
            0x0b | 0x1b | 0x2b => {
                let matrix = usize::from(self.command >> 4);
                let result = self.scalar(matrix, self.word(0), self.word(1), self.word(2));
                self.set_output_words(&[result]);
            }
            0x14 => {
                let zr = self.word(0);
                let xr = self.word(1);
                let yr = self.word(2);
                let u = f64::from(self.word(3));
                let f = f64::from(self.word(4));
                let l = f64::from(self.word(5));
                let (sy, cy) = Self::angle(yr).sin_cos();
                let (sx, cx) = Self::angle(xr).sin_cos();
                let z_delta = if cx.abs() > 1.0e-9 {
                    (u * cy - f * sy) / cx
                } else {
                    0.0
                };
                let x_delta = u * sy + f * cy;
                let y_delta =
                    -(u * cy + f * sy) * if cx.abs() > 1.0e-9 { sx / cx } else { 0.0 } + l;
                self.set_output_words(&[
                    zr.wrapping_add(Self::clamp_i16(z_delta)),
                    xr.wrapping_add(Self::clamp_i16(x_delta)),
                    yr.wrapping_add(Self::clamp_i16(y_delta)),
                ]);
            }
            0x0f => self.set_output_words(&[0]),
            0x2f => self.set_output_words(&[0x0100]),
            0x1f => {
                self.output.fill(0);
                self.output_len = OUTPUT_CAPACITY as u16;
                self.output_index = 0;
            }
            _ => {
                self.output_len = 0;
                self.output_index = 0;
            }
        }
    }

    pub(crate) fn save(&self, out: &mut StateWriter) {
        out.u8(u8::from(self.waiting_for_command));
        out.u8(self.command);
        out.u8(self.expected);
        out.u8(self.input_len);
        out.blob(&self.parameters);
        out.blob(&self.output);
        out.u16(self.output_len);
        out.u16(self.output_index);
        for matrix in self.matrices {
            for row in matrix {
                for value in row {
                    out.u32(value as u32);
                }
            }
        }
        for value in [
            self.projection.fx,
            self.projection.fy,
            self.projection.fz,
            self.projection.lfe,
            self.projection.les,
            self.projection.aas,
            self.projection.azs,
        ] {
            out.u16(value as u16);
        }
        out.u16(self.raster_vs as u16);
        out.u8(u8::from(self.raster_streaming));
    }

    pub(crate) fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.waiting_for_command = input.u8()? != 0;
        self.command = input.u8()?;
        self.expected = input.u8()?.min(PARAMETER_CAPACITY as u8);
        self.input_len = input.u8()?.min(PARAMETER_CAPACITY as u8);
        let parameters = input.blob()?;
        if parameters.len() != self.parameters.len() {
            return Err("invalid SNES DSP-1 parameter state length".into());
        }
        self.parameters.copy_from_slice(parameters);
        let output = input.blob()?;
        if output.len() != self.output.len() {
            return Err("invalid SNES DSP-1 output state length".into());
        }
        self.output.copy_from_slice(output);
        self.output_len = input.u16()?.min(OUTPUT_CAPACITY as u16);
        self.output_index = input.u16()?.min(self.output_len);
        for matrix in &mut self.matrices {
            for row in matrix {
                for value in row {
                    *value = input.u32()? as i32;
                }
            }
        }
        let fx = input.u16()? as i16;
        let fy = input.u16()? as i16;
        let fz = input.u16()? as i16;
        let lfe = input.u16()? as i16;
        let les = input.u16()? as i16;
        let aas = input.u16()? as i16;
        let azs = input.u16()? as i16;
        let _ = self.configure_projection([fx, fy, fz, lfe, les, aas, azs]);
        self.raster_vs = input.u16()? as i16;
        self.raster_streaming = input.u8()? != 0;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn command(dsp: &mut Dsp1, opcode: u8, parameters: &[i16], outputs: usize) -> Vec<i16> {
        dsp.write_data(opcode);
        for parameter in parameters {
            for byte in parameter.to_le_bytes() {
                dsp.write_data(byte);
            }
        }
        (0..outputs)
            .map(|_| i16::from_le_bytes([dsp.read_data(), dsp.read_data()]))
            .collect()
    }

    #[test]
    fn multiply_triangle_and_vector_commands_follow_protocol() {
        let mut dsp = Dsp1::default();
        assert_eq!(command(&mut dsp, 0x00, &[0x4000, 0x4000], 1), vec![0x2000]);
        let trig = command(&mut dsp, 0x04, &[0x4000, 0x4000], 2);
        assert!((i32::from(trig[0]) - 0x4000).abs() <= 1);
        assert!(i32::from(trig[1]).abs() <= 1);
        assert_eq!(command(&mut dsp, 0x08, &[3, 4, 0], 2), vec![50, 0]);
        assert_eq!(command(&mut dsp, 0x28, &[3, 4, 0], 1), vec![5]);
    }

    #[test]
    fn matrices_support_objective_subjective_and_scalar_paths() {
        let mut dsp = Dsp1::default();
        let _ = command(&mut dsp, 0x01, &[0x7fff, 0, 0, 0], 0);
        let objective = command(&mut dsp, 0x0d, &[1000, -2000, 3000], 3);
        assert!((i32::from(objective[0]) - 500).abs() <= 2);
        assert!((i32::from(objective[1]) + 1000).abs() <= 2);
        assert!((i32::from(objective[2]) - 1500).abs() <= 2);
        let subjective = command(&mut dsp, 0x03, &objective, 3);
        assert!((i32::from(subjective[0]) - 250).abs() <= 3);
        let scalar = command(&mut dsp, 0x0b, &[1000, 0, 0], 1);
        assert!((i32::from(scalar[0]) - 500).abs() <= 2);
    }

    #[test]
    fn raster_stream_continues_until_interrupted() {
        let mut dsp = Dsp1::default();
        let _ = command(&mut dsp, 0x02, &[0, 0, 1024, 0, 256, 0, 0x1000], 4);
        dsp.write_data(0x0a);
        for byte in 0i16.to_le_bytes() {
            dsp.write_data(byte);
        }
        let first: Vec<u8> = (0..8).map(|_| dsp.read_data()).collect();
        assert_eq!(first.len(), 8);
        assert_eq!(dsp.raster_vs, 1);
        let second: Vec<u8> = (0..8).map(|_| dsp.read_data()).collect();
        assert_eq!(second.len(), 8);
        assert_eq!(dsp.raster_vs, 2);
        dsp.write_data(0x00);
        assert!(!dsp.raster_streaming);
    }

    #[test]
    fn state_round_trip_preserves_pending_output() {
        let mut dsp = Dsp1::default();
        let _ = command(&mut dsp, 0x01, &[0x6000, 0x1000, 0x2000, 0x3000], 0);
        dsp.write_data(0x00);
        for word in [0x4000i16, 0x2000] {
            for byte in word.to_le_bytes() {
                dsp.write_data(byte);
            }
        }
        let mut out = StateWriter::new(crate::platform::PlatformId::Snes, 123);
        dsp.save(&mut out);
        let bytes = out.finish();

        let first = dsp.read_data();
        let mut restored = Dsp1::default();
        let mut input = StateReader::new(&bytes, crate::platform::PlatformId::Snes, 123).unwrap();
        restored.load(&mut input).unwrap();
        input.finish().unwrap();
        assert_eq!(restored.read_data(), first);
        assert_eq!(restored.matrices, dsp.matrices);
    }
}
