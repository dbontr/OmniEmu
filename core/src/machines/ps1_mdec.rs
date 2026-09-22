use std::collections::VecDeque;

use crate::state::{StateReader, StateWriter};

const ZIGZAG: [usize; 64] = [
    0, 8, 1, 2, 9, 16, 24, 17, 10, 3, 4, 11, 18, 25, 32, 40, 33, 26, 19, 12, 5, 6, 13, 20, 27, 34,
    41, 48, 56, 49, 42, 35, 28, 21, 14, 7, 15, 22, 29, 36, 43, 50, 57, 58, 51, 44, 37, 30, 23, 31,
    38, 45, 52, 59, 60, 53, 46, 39, 47, 54, 61, 62, 55, 63,
];

pub struct Ps1Mdec {
    quant_y: [u8; 64],
    quant_uv: [u8; 64],
    scale: [i16; 64],
    parameters: Vec<u32>,
    output: VecDeque<u32>,
    command: u32,
    remaining_words: u16,
    dma_in_enabled: bool,
    dma_out_enabled: bool,
    current_block: u8,
    reset_latched: bool,
}

impl Default for Ps1Mdec {
    fn default() -> Self {
        Self::new()
    }
}

impl Ps1Mdec {
    pub fn new() -> Self {
        Self {
            quant_y: [0; 64],
            quant_uv: [0; 64],
            scale: [0; 64],
            parameters: Vec::with_capacity(256),
            output: VecDeque::with_capacity(1024),
            command: 0,
            remaining_words: 0,
            dma_in_enabled: false,
            dma_out_enabled: false,
            current_block: 4,
            reset_latched: true,
        }
    }

    pub fn reset(&mut self) {
        self.parameters.clear();
        self.output.clear();
        self.command = 0;
        self.remaining_words = 0;
        self.current_block = 4;
        self.reset_latched = true;
    }

    pub fn write_control(&mut self, value: u32) {
        self.dma_in_enabled = value & (1 << 30) != 0;
        self.dma_out_enabled = value & (1 << 29) != 0;
        if value & (1 << 31) != 0 {
            self.reset();
        }
    }

    pub fn status(&self) -> u32 {
        let mut status = 0u32;
        if self.output.is_empty() {
            status |= 1 << 31;
        }
        if self.remaining_words == 0 && self.command != 0 {
            status |= 1 << 30;
        }
        if self.remaining_words != 0 {
            status |= 1 << 29;
        }
        if self.dma_in_enabled && self.remaining_words != 0 {
            status |= 1 << 28;
        }
        if self.dma_out_enabled && !self.output.is_empty() {
            status |= 1 << 27;
        }
        status |= (self.command >> 2) & 0x0780_0000;
        status |= u32::from(self.current_block & 7) << 16;
        status
            | if self.reset_latched {
                0
            } else if self.remaining_words == 0 {
                0xffff
            } else {
                u32::from(self.remaining_words - 1)
            }
    }

    pub fn write_data(&mut self, value: u32) {
        if self.remaining_words == 0 {
            self.begin_command(value);
            return;
        }
        self.parameters.push(value);
        self.remaining_words -= 1;
        if self.remaining_words == 0 {
            self.execute_command();
        }
    }

    pub fn read_data(&mut self) -> u32 {
        self.output.pop_front().unwrap_or(0)
    }

    pub fn can_dma_in(&self) -> bool {
        self.dma_in_enabled && self.remaining_words != 0
    }

    pub fn can_dma_out(&self) -> bool {
        self.dma_out_enabled && !self.output.is_empty()
    }

    fn begin_command(&mut self, command: u32) {
        self.command = command;
        self.reset_latched = false;
        self.parameters.clear();
        self.current_block = 4;
        self.remaining_words = match command >> 29 {
            1 => command as u16,
            2 => {
                if command & 1 != 0 {
                    32
                } else {
                    16
                }
            }
            3 => 32,
            _ => 0,
        };
        if self.remaining_words == 0 {
            self.execute_command();
        }
    }

    fn execute_command(&mut self) {
        match self.command >> 29 {
            1 => self.decode_stream(),
            2 => self.load_quant_tables(),
            3 => self.load_scale_table(),
            _ => {}
        }
        self.parameters.clear();
    }

    fn parameter_bytes(&self) -> Vec<u8> {
        self.parameters
            .iter()
            .flat_map(|word| word.to_le_bytes())
            .collect()
    }

    fn load_quant_tables(&mut self) {
        let bytes = self.parameter_bytes();
        if bytes.len() >= 64 {
            self.quant_y.copy_from_slice(&bytes[..64]);
        }
        if self.command & 1 != 0 && bytes.len() >= 128 {
            self.quant_uv.copy_from_slice(&bytes[64..128]);
        }
    }

    fn load_scale_table(&mut self) {
        let bytes = self.parameter_bytes();
        let mut wire = [0i16; 64];
        for (index, chunk) in bytes.as_chunks::<2>().0.iter().take(64).enumerate() {
            wire[index] = i16::from_le_bytes(*chunk);
        }
        for y in 0..8usize {
            for x in 0..8usize {
                self.scale[y * 8 + x] = (wire[x * 8 + y] >> 3) << 3;
            }
        }
    }

    fn decode_stream(&mut self) {
        let halfwords: Vec<u16> = self
            .parameters
            .iter()
            .flat_map(|word| [*word as u16, (*word >> 16) as u16])
            .collect();
        let mut cursor = 0usize;
        let depth = ((self.command >> 27) & 3) as u8;
        let signed = self.command & (1 << 26) != 0;
        let set_bit15 = self.command & (1 << 25) != 0;
        while cursor < halfwords.len() {
            let before = cursor;
            if depth < 2 {
                let Some(block) = self.decode_block(&halfwords, &mut cursor, self.quant_y) else {
                    break;
                };
                self.current_block = 4;
                self.emit_mono(&block, depth, signed);
            } else if !self.decode_color_macroblock(
                &halfwords,
                &mut cursor,
                depth,
                signed,
                set_bit15,
            ) {
                break;
            }
            if cursor <= before {
                break;
            }
        }
        self.current_block = if self.output.is_empty() { 4 } else { 0 };
    }

    fn decode_color_macroblock(
        &mut self,
        source: &[u16],
        cursor: &mut usize,
        depth: u8,
        signed: bool,
        set_bit15: bool,
    ) -> bool {
        let Some(cr) = self.decode_block(source, cursor, self.quant_uv) else {
            return false;
        };
        let Some(cb) = self.decode_block(source, cursor, self.quant_uv) else {
            return false;
        };
        let mut rgb = [[0u8; 3]; 256];
        for block_index in 0..4usize {
            let Some(y_block) = self.decode_block(source, cursor, self.quant_y) else {
                return false;
            };
            let origin_x = (block_index & 1) * 8;
            let origin_y = (block_index >> 1) * 8;
            for y in 0..8usize {
                for x in 0..8usize {
                    let px = origin_x + x;
                    let py = origin_y + y;
                    let chroma_index = (px / 2) + (py / 2) * 8;
                    rgb[px + py * 16] = Self::yuv_pixel(
                        y_block[x + y * 8],
                        cr[chroma_index],
                        cb[chroma_index],
                        signed,
                    );
                }
            }
        }
        self.emit_rgb(&rgb, depth, set_bit15);
        true
    }

    fn signed_ten(value: u16) -> i32 {
        let value = i32::from(value & 0x03ff);
        if value & 0x0200 != 0 {
            value - 0x0400
        } else {
            value
        }
    }

    fn decode_block(
        &self,
        source: &[u16],
        cursor: &mut usize,
        quant: [u8; 64],
    ) -> Option<[i32; 64]> {
        while *cursor < source.len() && source[*cursor] == 0xfe00 {
            *cursor += 1;
        }
        let first = *source.get(*cursor)?;
        *cursor += 1;
        let q_scale = i32::from(first >> 10);
        let mut block = [0i32; 64];
        let mut k = 0usize;
        let coefficient = Self::signed_ten(first);
        let bias = if coefficient == 0 {
            0
        } else if coefficient < 0 {
            8
        } else {
            -8
        };
        let value = if q_scale == 0 {
            coefficient << 5
        } else {
            ((coefficient * i32::from(quant[0])) << 4) + bias
        }
        .clamp(-0x4000, 0x3fff);
        block[ZIGZAG[0]] = value;

        while let Some(word) = source.get(*cursor).copied() {
            *cursor += 1;
            k = k.saturating_add(usize::from(word >> 10) + 1);
            if k < 64 {
                let coefficient = Self::signed_ten(word);
                let scale_quant = q_scale * i32::from(quant[k]);
                let bias = if coefficient == 0 {
                    0
                } else if coefficient < 0 {
                    8
                } else {
                    -8
                };
                let value = if scale_quant == 0 {
                    coefficient << 5
                } else {
                    (((coefficient * scale_quant) >> 3) << 4) + bias
                }
                .clamp(-0x4000, 0x3fff);
                block[ZIGZAG[k]] = value;
            }
            if k >= 63 {
                break;
            }
        }
        self.idct(&mut block);
        Some(block)
    }

    fn idct_row(values: &[i32], matrix: &[i16]) -> i32 {
        let mut sum = 0i64;
        for index in 0..8usize {
            sum += i64::from(values[index]) * i64::from(matrix[index]);
        }
        ((sum + 0x2_0000) >> 18) as i32
    }

    fn idct(&self, block: &mut [i32; 64]) {
        let mut intermediate = [0i32; 64];
        for x in 0..8usize {
            for y in 0..8usize {
                intermediate[y * 8 + x] =
                    Self::idct_row(&block[x * 8..x * 8 + 8], &self.scale[y * 8..y * 8 + 8]);
            }
        }
        for x in 0..8usize {
            for y in 0..8usize {
                let value = Self::idct_row(
                    &intermediate[x * 8..x * 8 + 8],
                    &self.scale[y * 8..y * 8 + 8],
                );
                let nine_bit = value & 0x01ff;
                let signed = if nine_bit & 0x0100 != 0 {
                    nine_bit - 0x0200
                } else {
                    nine_bit
                };
                block[x * 8 + y] = signed.clamp(-128, 127);
            }
        }
    }

    fn sample_byte(value: i32, signed: bool) -> u8 {
        let signed_value = (value & 0x1ff).clamp(0, 0x1ff);
        let signed_value = if signed_value & 0x100 != 0 {
            signed_value - 0x200
        } else {
            signed_value
        }
        .clamp(-128, 127);
        if signed {
            signed_value as i8 as u8
        } else {
            (signed_value + 128) as u8
        }
    }

    fn channel_byte(value: i32, signed: bool) -> u8 {
        let nine_bit = value & 0x01ff;
        let signed_value = if nine_bit & 0x0100 != 0 {
            nine_bit - 0x0200
        } else {
            nine_bit
        }
        .clamp(-128, 127);
        if signed {
            signed_value as i8 as u8
        } else {
            (signed_value + 128) as u8
        }
    }

    fn yuv_pixel(y: i32, cr: i32, cb: i32, signed: bool) -> [u8; 3] {
        let red = y + ((cr * 359 + 0x80) >> 8);
        let green = y + ((((-88 * cb) & !0x1f) + ((-183 * cr) & !0x07) + 0x80) >> 8);
        let blue = y + ((cb * 454 + 0x80) >> 8);
        [
            Self::channel_byte(red, signed),
            Self::channel_byte(green, signed),
            Self::channel_byte(blue, signed),
        ]
    }

    fn mono4_sample(value: u8, _signed: bool) -> u8 {
        value >> 4
    }

    fn rgb8_to_5(value: u8) -> u16 {
        ((u16::from(value) + 4) >> 3).min(0x1f)
    }

    fn emit_mono(&mut self, block: &[i32; 64], depth: u8, signed: bool) {
        let pixels: Vec<u8> = block
            .iter()
            .map(|value| Self::sample_byte(*value, signed))
            .collect();
        if depth == 0 {
            let mut packed = Vec::with_capacity(32);
            for pair in pixels.as_chunks::<2>().0 {
                packed.push(
                    Self::mono4_sample(pair[0], signed)
                        | (Self::mono4_sample(pair[1], signed) << 4),
                );
            }
            self.push_bytes(&packed);
        } else {
            self.push_bytes(&pixels);
        }
    }

    fn emit_rgb(&mut self, rgb: &[[u8; 3]; 256], depth: u8, set_bit15: bool) {
        let mut bytes = Vec::with_capacity(if depth == 3 { 512 } else { 768 });
        if depth == 3 {
            for pixel in rgb {
                let value = Self::rgb8_to_5(pixel[0])
                    | (Self::rgb8_to_5(pixel[1]) << 5)
                    | (Self::rgb8_to_5(pixel[2]) << 10)
                    | (u16::from(set_bit15) << 15);
                bytes.extend_from_slice(&value.to_le_bytes());
            }
        } else {
            for pixel in rgb {
                bytes.extend_from_slice(pixel);
            }
        }
        self.push_bytes(&bytes);
    }

    fn push_bytes(&mut self, bytes: &[u8]) {
        for chunk in bytes.chunks(4) {
            let mut word = [0u8; 4];
            word[..chunk.len()].copy_from_slice(chunk);
            self.output.push_back(u32::from_le_bytes(word));
        }
    }

    pub fn save(&self, out: &mut StateWriter) {
        out.blob(&self.quant_y);
        out.blob(&self.quant_uv);
        for value in self.scale {
            out.u16(value as u16);
        }
        out.u32(self.parameters.len() as u32);
        for value in &self.parameters {
            out.u32(*value);
        }
        out.u32(self.output.len() as u32);
        for value in &self.output {
            out.u32(*value);
        }
        out.u32(self.command);
        out.u16(self.remaining_words);
        out.u8(self.dma_in_enabled as u8);
        out.u8(self.dma_out_enabled as u8);
        out.u8(self.current_block);
        out.u8(self.reset_latched as u8);
    }

    pub fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let quant_y = input.blob()?;
        let quant_uv = input.blob()?;
        if quant_y.len() != 64 || quant_uv.len() != 64 {
            return Err("invalid PlayStation MDEC quantization table state".into());
        }
        self.quant_y.copy_from_slice(quant_y);
        self.quant_uv.copy_from_slice(quant_uv);
        for value in &mut self.scale {
            *value = input.u16()? as i16;
        }
        let parameter_len = input.u32()? as usize;
        if parameter_len > 0x10000 {
            return Err("invalid PlayStation MDEC parameter state".into());
        }
        self.parameters.clear();
        for _ in 0..parameter_len {
            self.parameters.push(input.u32()?);
        }
        let output_len = input.u32()? as usize;
        if output_len > 0x100000 {
            return Err("invalid PlayStation MDEC output state".into());
        }
        self.output.clear();
        for _ in 0..output_len {
            self.output.push_back(input.u32()?);
        }
        self.command = input.u32()?;
        self.remaining_words = input.u16()?;
        self.dma_in_enabled = input.u8()? != 0;
        self.dma_out_enabled = input.u8()? != 0;
        self.current_block = input.u8()? & 7;
        self.reset_latched = input.u8()? != 0;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::platform::PlatformId;

    const SCALE: [u16; 64] = [
        0x5a82, 0x5a82, 0x5a82, 0x5a82, 0x5a82, 0x5a82, 0x5a82, 0x5a82, 0x7d8a, 0x6a6d, 0x471c,
        0x18f8, 0xe707, 0xb8e3, 0x9592, 0x8275, 0x7641, 0x30fb, 0xcf04, 0x89be, 0x89be, 0xcf04,
        0x30fb, 0x7641, 0x6a6d, 0xe707, 0x8275, 0xb8e3, 0x471c, 0x7d8a, 0x18f8, 0x9592, 0x5a82,
        0xa57d, 0xa57d, 0x5a82, 0x5a82, 0xa57d, 0xa57d, 0x5a82, 0x471c, 0x8275, 0x18f8, 0x6a6d,
        0x9592, 0xe707, 0x7d8a, 0xb8e3, 0x30fb, 0x89be, 0x7641, 0xcf04, 0xcf04, 0x7641, 0x89be,
        0x30fb, 0x18f8, 0xb8e3, 0x6a6d, 0x8275, 0x7d8a, 0x9592, 0x471c, 0xe707,
    ];

    fn load_scale(mdec: &mut Ps1Mdec) {
        mdec.write_data(3 << 29);
        for pair in SCALE.as_chunks::<2>().0 {
            mdec.write_data(u32::from(pair[0]) | (u32::from(pair[1]) << 16));
        }
    }

    #[test]
    fn four_bit_mono_uses_high_nibble_without_rounding() {
        assert_eq!(Ps1Mdec::mono4_sample(0, false), 0);
        assert_eq!(Ps1Mdec::mono4_sample(8, false), 0);
        assert_eq!(Ps1Mdec::mono4_sample(15, false), 0);
        assert_eq!(Ps1Mdec::mono4_sample(16, false), 1);
        assert_eq!(Ps1Mdec::mono4_sample(200, false), 12);
        assert_eq!(Ps1Mdec::mono4_sample(232, false), 14);
        assert_eq!(Ps1Mdec::mono4_sample(255, false), 15);
        assert_eq!(Ps1Mdec::mono4_sample(0xf8, true), 0x0f);
    }

    #[test]
    fn yuv_conversion_uses_hardware_rounding_and_signed_nine_bit_wrap() {
        assert_eq!(Ps1Mdec::yuv_pixel(0, 2, 1, false), [131, 126, 130]);
        assert_eq!(Ps1Mdec::yuv_pixel(0, 2, 1, true), [3, 254, 2]);
        assert_eq!(Ps1Mdec::yuv_pixel(127, 127, 0, false), [0, 164, 255]);
    }

    #[test]
    fn fifteen_bit_output_rounds_eight_bit_channels() {
        assert_eq!(Ps1Mdec::rgb8_to_5(0), 0);
        assert_eq!(Ps1Mdec::rgb8_to_5(3), 0);
        assert_eq!(Ps1Mdec::rgb8_to_5(4), 1);
        assert_eq!(Ps1Mdec::rgb8_to_5(251), 31);
        assert_eq!(Ps1Mdec::rgb8_to_5(252), 31);
        assert_eq!(Ps1Mdec::rgb8_to_5(255), 31);
    }

    #[test]
    fn control_and_status_expose_dma_requests_and_reset() {
        let mut mdec = Ps1Mdec::new();
        assert_eq!(mdec.status(), 0x8004_0000);
        mdec.write_control((1 << 30) | (1 << 29));
        mdec.write_data((1 << 29) | (1 << 27) | 1);
        assert!(mdec.can_dma_in());
        assert_ne!(mdec.status() & (1 << 28), 0);
        assert_eq!((mdec.status() >> 25) & 3, 1);
        mdec.write_data(0xfe00_0008);
        assert!(mdec.can_dma_out());
        assert_ne!(mdec.status() & (1 << 27), 0);
        mdec.write_control(1 << 31);
        assert!(mdec.output.is_empty());
        assert_eq!(mdec.remaining_words, 0);
        assert_eq!(mdec.status(), 0x8004_0000);
    }

    #[test]
    fn scale_table_and_mono_decode_produce_eight_bit_block() {
        let mut mdec = Ps1Mdec::new();
        load_scale(&mut mdec);
        let mut expected_scale = [0i16; 64];
        for y in 0..8usize {
            for x in 0..8usize {
                expected_scale[y * 8 + x] = ((SCALE[x * 8 + y] as i16) >> 3) << 3;
            }
        }
        assert_eq!(mdec.scale, expected_scale);
        mdec.write_data((1 << 29) | (1 << 27) | 1);
        mdec.write_data(0xfe00_0010);
        assert_eq!(mdec.output.len(), 16);
        assert!(mdec.output.iter().any(|word| *word != 0));
    }

    #[test]
    fn colored_fifteen_bit_macroblock_is_emitted_in_raster_order() {
        let mut mdec = Ps1Mdec::new();
        load_scale(&mut mdec);
        mdec.write_data((1 << 29) | (3 << 27) | 6);
        for _ in 0..6 {
            mdec.write_data(0xfe00_0000);
        }
        assert_eq!(mdec.output.len(), 128);
        assert_eq!(mdec.output[0], 0x4210_4210);
    }

    #[test]
    fn state_round_trip_preserves_tables_command_and_fifo() {
        let mut first = Ps1Mdec::new();
        load_scale(&mut first);
        first.write_control((1 << 30) | (1 << 29));
        first.write_data((1 << 29) | (1 << 27) | 1);
        first.write_data(0xfe00_0010);
        let _ = first.read_data();
        let mut writer = StateWriter::new(PlatformId::PlayStation, 1);
        first.save(&mut writer);
        let bytes = writer.finish();
        let mut reader = StateReader::new(&bytes, PlatformId::PlayStation, 1).unwrap();
        let mut second = Ps1Mdec::new();
        second.load(&mut reader).unwrap();
        reader.finish().unwrap();
        let mut left = StateWriter::new(PlatformId::PlayStation, 1);
        let mut right = StateWriter::new(PlatformId::PlayStation, 1);
        first.save(&mut left);
        second.save(&mut right);
        assert_eq!(left.finish(), right.finish());
    }
}
