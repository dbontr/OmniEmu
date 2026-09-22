use crate::state::{StateReader, StateWriter};

const BUFFER_SIZE: usize = 512;

#[derive(Clone)]
pub(crate) struct Dsp2 {
    waiting_for_command: bool,
    command: u8,
    expected: u16,
    input_len: u16,
    parameters: [u8; BUFFER_SIZE],
    output: [u8; BUFFER_SIZE],
    output_len: u16,
    output_index: u16,
    transparent: u8,
    op05_has_len: bool,
    op05_len: u8,
    op06_has_len: bool,
    op06_len: u8,
    op0d_has_len: bool,
    op0d_in_len: u8,
    op0d_out_len: u8,
}

impl Default for Dsp2 {
    fn default() -> Self {
        Self {
            waiting_for_command: true,
            command: 0,
            expected: 0,
            input_len: 0,
            parameters: [0; BUFFER_SIZE],
            output: [0; BUFFER_SIZE],
            output_len: 0,
            output_index: 0,
            transparent: 0,
            op05_has_len: false,
            op05_len: 0,
            op06_has_len: false,
            op06_len: 0,
            op0d_has_len: false,
            op0d_in_len: 0,
            op0d_out_len: 0,
        }
    }
}

impl Dsp2 {
    pub(crate) fn reset(&mut self) {
        *self = Self::default();
    }

    pub(crate) fn read_data(&mut self) -> u8 {
        if self.output_index >= self.output_len {
            return 0xff;
        }
        let value = self.output[usize::from(self.output_index)];
        self.output_index += 1;
        if self.output_index >= self.output_len {
            self.output_len = 0;
            self.output_index = 0;
        }
        value
    }

    pub(crate) fn write_data(&mut self, value: u8) {
        if self.waiting_for_command {
            self.command = value;
            self.input_len = 0;
            self.output_len = 0;
            self.output_index = 0;
            self.expected = match value {
                0x01 => 32,
                0x03 | 0x05 | 0x06 => 1,
                0x09 => 4,
                0x0d => 2,
                _ => 0,
            };
            self.waiting_for_command = self.expected == 0;
            if self.expected == 0 {
                self.finish_phase();
            }
            return;
        }

        let index = usize::from(self.input_len);
        if index < self.parameters.len() {
            self.parameters[index] = value;
        }
        self.input_len = self.input_len.saturating_add(1);
        if self.input_len >= self.expected {
            self.waiting_for_command = true;
            self.finish_phase();
        }
    }

    fn finish_phase(&mut self) {
        self.output_index = 0;
        match self.command {
            0x01 => {
                self.output_len = 32;
                self.op01();
            }
            0x03 => self.transparent = self.parameters[0],
            0x05 => {
                if self.op05_has_len {
                    self.op05_has_len = false;
                    self.output_len = u16::from(self.op05_len);
                    self.op05();
                } else {
                    self.op05_len = self.parameters[0];
                    self.input_len = 0;
                    self.expected = u16::from(self.op05_len) * 2;
                    self.op05_has_len = true;
                    self.waiting_for_command = self.op05_len == 0;
                }
            }
            0x06 => {
                if self.op06_has_len {
                    self.op06_has_len = false;
                    self.output_len = u16::from(self.op06_len);
                    self.op06();
                } else {
                    self.op06_len = self.parameters[0];
                    self.input_len = 0;
                    self.expected = u16::from(self.op06_len);
                    self.op06_has_len = true;
                    self.waiting_for_command = self.op06_len == 0;
                }
            }
            0x09 => {
                self.output_len = 4;
                self.op09();
            }
            0x0d => {
                if self.op0d_has_len {
                    self.op0d_has_len = false;
                    self.output_len = u16::from(self.op0d_out_len);
                    self.op0d();
                } else {
                    self.op0d_in_len = self.parameters[0];
                    self.op0d_out_len = self.parameters[1];
                    self.input_len = 0;
                    self.expected = u16::from(self.op0d_in_len).div_ceil(2);
                    self.op0d_has_len = true;
                    self.waiting_for_command = self.expected == 0 || self.op0d_out_len == 0;
                }
            }
            _ => {}
        }
    }

    fn op01(&mut self) {
        for block in 0..8usize {
            let input = block * 4;
            let c0 = self.parameters[input];
            let c1 = self.parameters[input + 1];
            let c2 = self.parameters[input + 2];
            let c3 = self.parameters[input + 3];
            let output = block * 2;

            self.output[output] = (c0 & 0x10) << 3
                | (c0 & 0x01) << 6
                | (c1 & 0x10) << 1
                | (c1 & 0x01) << 4
                | (c2 & 0x10) >> 1
                | (c2 & 0x01) << 2
                | (c3 & 0x10) >> 3
                | (c3 & 0x01);
            self.output[output + 1] = (c0 & 0x20) << 2
                | (c0 & 0x02) << 5
                | (c1 & 0x20)
                | (c1 & 0x02) << 3
                | (c2 & 0x20) >> 2
                | (c2 & 0x02) << 1
                | (c3 & 0x20) >> 4
                | (c3 & 0x02) >> 1;
            self.output[16 + output] = (c0 & 0x40) << 1
                | (c0 & 0x04) << 4
                | (c1 & 0x40) >> 1
                | (c1 & 0x04) << 2
                | (c2 & 0x40) >> 3
                | (c2 & 0x04)
                | (c3 & 0x40) >> 5
                | (c3 & 0x04) >> 2;
            self.output[17 + output] = (c0 & 0x80)
                | (c0 & 0x08) << 3
                | (c1 & 0x80) >> 2
                | (c1 & 0x08) << 1
                | (c2 & 0x80) >> 4
                | (c2 & 0x08) >> 1
                | (c3 & 0x80) >> 6
                | (c3 & 0x08) >> 3;
        }
    }

    fn op05(&mut self) {
        let len = usize::from(self.op05_len);
        let transparent = self.transparent & 0x0f;
        for index in 0..len {
            let base = self.parameters[index];
            let overlay = self.parameters[len + index];
            let high = if overlay >> 4 == transparent {
                base & 0xf0
            } else {
                overlay & 0xf0
            };
            let low = if overlay & 0x0f == transparent {
                base & 0x0f
            } else {
                overlay & 0x0f
            };
            self.output[index] = high | low;
        }
    }

    fn op06(&mut self) {
        let len = usize::from(self.op06_len);
        for index in 0..len {
            let value = self.parameters[index];
            self.output[len - 1 - index] = value.rotate_left(4);
        }
    }

    fn op09(&mut self) {
        let a = u32::from(u16::from_le_bytes([self.parameters[0], self.parameters[1]]));
        let b = u32::from(u16::from_le_bytes([self.parameters[2], self.parameters[3]]));
        self.output[..4].copy_from_slice(&a.wrapping_mul(b).to_le_bytes());
    }

    fn op0d(&mut self) {
        let input_len = u32::from(self.op0d_in_len);
        let output_len = usize::from(self.op0d_out_len);
        if output_len == 0 {
            return;
        }
        let multiplier = if input_len <= u32::from(self.op0d_out_len) {
            0x1_0000
        } else {
            (input_len << 17) / (u32::from(self.op0d_out_len) * 2 + 1)
        };
        let mut pixel_location = 0u32;
        let mut pixels = [0u8; BUFFER_SIZE * 2];
        for pixel in pixels.iter_mut().take(output_len * 2) {
            let source = (pixel_location >> 16) as usize;
            *pixel = if source & 1 != 0 {
                self.parameters[source >> 1] & 0x0f
            } else {
                self.parameters[source >> 1] >> 4
            };
            pixel_location = pixel_location.wrapping_add(multiplier);
        }
        for index in 0..output_len {
            self.output[index] = (pixels[index * 2] << 4) | pixels[index * 2 + 1];
        }
    }

    pub(crate) fn save(&self, out: &mut StateWriter) {
        out.u8(u8::from(self.waiting_for_command));
        out.u8(self.command);
        out.u16(self.expected);
        out.u16(self.input_len);
        out.blob(&self.parameters);
        out.blob(&self.output);
        out.u16(self.output_len);
        out.u16(self.output_index);
        for value in [
            self.transparent,
            u8::from(self.op05_has_len),
            self.op05_len,
            u8::from(self.op06_has_len),
            self.op06_len,
            u8::from(self.op0d_has_len),
            self.op0d_in_len,
            self.op0d_out_len,
        ] {
            out.u8(value);
        }
    }

    pub(crate) fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.waiting_for_command = input.u8()? != 0;
        self.command = input.u8()?;
        self.expected = input.u16()?.min(BUFFER_SIZE as u16);
        self.input_len = input.u16()?.min(self.expected);
        let parameters = input.blob()?;
        if parameters.len() != self.parameters.len() {
            return Err("invalid SNES DSP-2 parameter state length".into());
        }
        self.parameters.copy_from_slice(parameters);
        let output = input.blob()?;
        if output.len() != self.output.len() {
            return Err("invalid SNES DSP-2 output state length".into());
        }
        self.output.copy_from_slice(output);
        self.output_len = input.u16()?.min(BUFFER_SIZE as u16);
        self.output_index = input.u16()?.min(self.output_len);
        self.transparent = input.u8()?;
        self.op05_has_len = input.u8()? != 0;
        self.op05_len = input.u8()?;
        self.op06_has_len = input.u8()? != 0;
        self.op06_len = input.u8()?;
        self.op0d_has_len = input.u8()? != 0;
        self.op0d_in_len = input.u8()?;
        self.op0d_out_len = input.u8()?;
        Ok(())
    }
}
#[cfg(test)]
mod tests {
    use super::*;

    fn send(dsp: &mut Dsp2, command: u8, bytes: &[u8], output: usize) -> Vec<u8> {
        dsp.write_data(command);
        for &byte in bytes {
            dsp.write_data(byte);
        }
        (0..output).map(|_| dsp.read_data()).collect()
    }

    #[test]
    fn bitmap_conversion_and_multiply_follow_protocol() {
        let mut dsp = Dsp2::default();
        let mut bitmap = [0u8; 32];
        bitmap[0] = 0x11;
        let converted = send(&mut dsp, 0x01, &bitmap, 32);
        assert_eq!(converted[0], 0xc0);
        assert!(converted[1..].iter().all(|&value| value == 0));

        let product = send(&mut dsp, 0x09, &[0x34, 0x12, 0x02, 0x00], 4);
        assert_eq!(u32::from_le_bytes(product.try_into().unwrap()), 0x2468);
        assert_eq!(dsp.read_data(), 0xff);
    }

    #[test]
    fn transparent_overlay_and_reverse_are_exact() {
        let mut dsp = Dsp2::default();
        let _ = send(&mut dsp, 0x03, &[0x03], 0);

        dsp.write_data(0x05);
        dsp.write_data(2);
        for byte in [0xab, 0xcd, 0x3e, 0xf3] {
            dsp.write_data(byte);
        }
        assert_eq!([dsp.read_data(), dsp.read_data()], [0xae, 0xfd]);

        dsp.write_data(0x06);
        dsp.write_data(3);
        for byte in [0x12, 0x34, 0x56] {
            dsp.write_data(byte);
        }
        assert_eq!(
            [dsp.read_data(), dsp.read_data(), dsp.read_data()],
            [0x65, 0x43, 0x21]
        );
    }

    #[test]
    fn fixed_point_scale_handles_downscale_and_upscale() {
        let mut dsp = Dsp2::default();

        dsp.write_data(0x0d);
        dsp.write_data(4);
        dsp.write_data(2);
        for byte in [0x12, 0x34] {
            dsp.write_data(byte);
        }
        assert_eq!([dsp.read_data(), dsp.read_data()], [0x12, 0x40]);

        dsp.write_data(0x0d);
        dsp.write_data(2);
        dsp.write_data(4);
        dsp.write_data(0xab);
        assert_eq!(
            [
                dsp.read_data(),
                dsp.read_data(),
                dsp.read_data(),
                dsp.read_data()
            ],
            [0xab, 0x04, 0x00, 0x00]
        );
    }

    #[test]
    fn state_round_trip_preserves_dynamic_packet_phase() {
        let mut dsp = Dsp2::default();
        dsp.write_data(0x05);
        dsp.write_data(2);
        dsp.write_data(0x12);
        dsp.write_data(0x34);

        let mut out = StateWriter::new(crate::platform::PlatformId::Snes, 321);
        dsp.save(&mut out);
        let bytes = out.finish();

        let mut restored = Dsp2::default();
        let mut input = StateReader::new(&bytes, crate::platform::PlatformId::Snes, 321).unwrap();
        restored.load(&mut input).unwrap();
        input.finish().unwrap();

        for byte in [0x56, 0x78] {
            dsp.write_data(byte);
            restored.write_data(byte);
        }
        assert_eq!(
            [restored.read_data(), restored.read_data()],
            [dsp.read_data(), dsp.read_data()]
        );
    }
}
