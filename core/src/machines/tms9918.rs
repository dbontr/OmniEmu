use crate::kernel::VideoBuffer;
use crate::state::{StateReader, StateWriter};

const WIDTH: u32 = 256;
const HEIGHT: u32 = 192;

const PALETTE: [[u8; 4]; 16] = [
    [0, 0, 0, 255],
    [0, 0, 0, 255],
    [33, 200, 66, 255],
    [94, 220, 120, 255],
    [84, 85, 237, 255],
    [125, 118, 252, 255],
    [212, 82, 77, 255],
    [66, 235, 245, 255],
    [252, 85, 84, 255],
    [255, 121, 120, 255],
    [212, 193, 84, 255],
    [230, 206, 128, 255],
    [33, 176, 59, 255],
    [201, 91, 186, 255],
    [204, 204, 204, 255],
    [255, 255, 255, 255],
];

pub struct Tms9918 {
    vram: [u8; 0x4000],
    regs: [u8; 8],
    address: u16,
    latch: Option<u8>,
    read_buffer: u8,
    status: u8,
    scanline: u16,
    dot: u16,
    dot_half: u8,
    frame: u64,
    irq: bool,
    irq_edge: bool,
    video: VideoBuffer,
}

impl Default for Tms9918 {
    fn default() -> Self {
        Self {
            vram: [0; 0x4000],
            regs: [0; 8],
            address: 0,
            latch: None,
            read_buffer: 0,
            status: 0,
            scanline: 0,
            dot: 0,
            dot_half: 0,
            frame: 0,
            irq: false,
            irq_edge: false,
            video: VideoBuffer::new(WIDTH, HEIGHT),
        }
    }
}

impl Tms9918 {
    pub fn reset(&mut self) {
        let vram = self.vram;
        *self = Self::default();
        self.vram = vram;
    }

    pub fn video(&self) -> &VideoBuffer {
        &self.video
    }

    #[cfg(test)]
    pub fn irq_pending(&self) -> bool {
        self.irq
    }

    fn update_irq(&mut self) {
        let next = self.status & 0x80 != 0 && self.regs[1] & 0x20 != 0;
        if next && !self.irq {
            self.irq_edge = true;
        }
        self.irq = next;
    }

    pub fn take_irq_edge(&mut self) -> bool {
        let edge = self.irq_edge;
        self.irq_edge = false;
        edge
    }

    pub fn write_control(&mut self, value: u8) {
        let Some(first) = self.latch.take() else {
            self.latch = Some(value);
            return;
        };
        if value & 0x80 != 0 {
            let register = usize::from(value & 7);
            self.regs[register] = first;
            self.update_irq();
            return;
        }
        self.address = ((u16::from(value & 0x3f) << 8) | u16::from(first)) & 0x3fff;
        if value & 0x40 == 0 {
            self.read_buffer = self.vram[self.address as usize];
            self.address = (self.address + 1) & 0x3fff;
        }
    }

    pub fn write_data(&mut self, value: u8) {
        self.vram[self.address as usize] = value;
        self.address = (self.address + 1) & 0x3fff;
        self.read_buffer = value;
        self.latch = None;
    }

    pub fn read_data(&mut self) -> u8 {
        let value = self.read_buffer;
        self.read_buffer = self.vram[self.address as usize];
        self.address = (self.address + 1) & 0x3fff;
        self.latch = None;
        value
    }

    pub fn read_status(&mut self) -> u8 {
        let value = self.status;
        self.status &= 0x1f;
        self.irq_edge = false;
        self.latch = None;
        self.update_irq();
        value
    }

    pub fn tick_cpu_cycles(&mut self, cycles: u32) {
        let scaled = cycles.saturating_mul(3) + u32::from(self.dot_half);
        self.dot_half = (scaled & 1) as u8;
        let mut dots = u32::from(self.dot) + scaled / 2;
        while dots >= 342 {
            dots -= 342;
            self.scanline = self.scanline.wrapping_add(1);
            if self.scanline == 192 {
                self.status |= 0x80;
                self.frame = self.frame.wrapping_add(1);
                self.render_frame();
                self.update_irq();
            }
            if self.scanline >= 262 {
                self.scanline = 0;
            }
        }
        self.dot = dots as u16;
    }

    pub fn frame(&self) -> u64 {
        self.frame
    }

    fn render_frame(&mut self) {
        let backdrop = PALETTE[usize::from(self.regs[7] & 0x0f)];
        self.video.clear(backdrop);
        if self.regs[1] & 0x40 == 0 {
            return;
        }
        if self.regs[0] & 0x02 != 0 {
            self.render_graphics_two();
        } else {
            self.render_graphics_one();
        }
        self.render_sprites();
    }

    fn render_graphics_one(&mut self) {
        let name_base = (usize::from(self.regs[2] & 0x0f) << 10) & 0x3fff;
        let pattern_base = (usize::from(self.regs[4] & 0x07) << 11) & 0x3fff;
        let color_base = (usize::from(self.regs[3]) << 6) & 0x3fff;
        for tile_y in 0..24usize {
            for tile_x in 0..32usize {
                let name = self.vram[(name_base + tile_y * 32 + tile_x) & 0x3fff];
                let colors = self.vram[(color_base + usize::from(name / 8)) & 0x3fff];
                let fg = colors >> 4;
                let bg = colors & 0x0f;
                for row in 0..8usize {
                    let pattern = self.vram[(pattern_base + usize::from(name) * 8 + row) & 0x3fff];
                    self.draw_pattern_row(tile_x * 8, tile_y * 8 + row, pattern, fg, bg);
                }
            }
        }
    }

    fn render_graphics_two(&mut self) {
        let name_base = (usize::from(self.regs[2] & 0x0f) << 10) & 0x3fff;
        let pattern_base = if self.regs[4] & 0x04 != 0 { 0x2000 } else { 0 };
        let color_base = if self.regs[3] & 0x80 != 0 { 0x2000 } else { 0 };
        for tile_y in 0..24usize {
            let third = tile_y / 8;
            for tile_x in 0..32usize {
                let name = self.vram[(name_base + tile_y * 32 + tile_x) & 0x3fff];
                for row in 0..8usize {
                    let index = third * 0x800 + usize::from(name) * 8 + row;
                    let pattern = self.vram[(pattern_base + index) & 0x3fff];
                    let colors = self.vram[(color_base + index) & 0x3fff];
                    self.draw_pattern_row(
                        tile_x * 8,
                        tile_y * 8 + row,
                        pattern,
                        colors >> 4,
                        colors & 0x0f,
                    );
                }
            }
        }
    }

    fn draw_pattern_row(&mut self, x: usize, y: usize, pattern: u8, fg: u8, bg: u8) {
        for bit in 0..8usize {
            let color = if pattern & (0x80 >> bit) != 0 { fg } else { bg };
            let color = if color == 0 {
                self.regs[7] & 0x0f
            } else {
                color
            };
            let offset = (y * WIDTH as usize + x + bit) * 4;
            self.video.pixels_mut()[offset..offset + 4]
                .copy_from_slice(&PALETTE[usize::from(color & 0x0f)]);
        }
    }

    fn render_sprites(&mut self) {
        let attr_base = (usize::from(self.regs[5] & 0x7f) << 7) & 0x3fff;
        let pattern_base = (usize::from(self.regs[6] & 0x07) << 11) & 0x3fff;
        let size = if self.regs[1] & 0x02 != 0 { 16usize } else { 8 };
        let zoom = if self.regs[1] & 0x01 != 0 { 2usize } else { 1 };
        let mut occupied = vec![false; (WIDTH * HEIGHT) as usize];
        let mut count = [0u8; HEIGHT as usize];
        for sprite in 0..32usize {
            let base = (attr_base + sprite * 4) & 0x3fff;
            let raw_y = self.vram[base];
            if raw_y == 0xd0 {
                break;
            }
            let mut y = i32::from(raw_y) + 1;
            if raw_y > 0xd0 {
                y -= 256;
            }
            let mut x = i32::from(self.vram[(base + 1) & 0x3fff]);
            let mut pattern = self.vram[(base + 2) & 0x3fff];
            let color = self.vram[(base + 3) & 0x3fff];
            if color & 0x80 != 0 {
                x -= 32;
            }
            if size == 16 {
                pattern &= 0xfc;
            }
            for source_y in 0..size {
                let tile_row = source_y / 8;
                let row = source_y & 7;
                let pattern_number = usize::from(pattern) + tile_row * 2;
                for zy in 0..zoom {
                    let screen_y = y + (source_y * zoom + zy) as i32;
                    if !(0..HEIGHT as i32).contains(&screen_y) {
                        continue;
                    }
                    let line = screen_y as usize;
                    if count[line] >= 4 {
                        self.status = (self.status & 0xe0) | 0x40 | (sprite as u8 & 0x1f);
                        continue;
                    }
                    for source_x in 0..size {
                        let tile_col = source_x / 8;
                        let bit = 7 - (source_x & 7);
                        let tile = pattern_number + tile_col;
                        let byte = self.vram[(pattern_base + tile * 8 + row) & 0x3fff];
                        if byte & (1 << bit) == 0 || color & 0x0f == 0 {
                            continue;
                        }
                        for zx in 0..zoom {
                            let screen_x = x + (source_x * zoom + zx) as i32;
                            if !(0..WIDTH as i32).contains(&screen_x) {
                                continue;
                            }
                            let pixel = line * WIDTH as usize + screen_x as usize;
                            if occupied[pixel] {
                                self.status |= 0x20;
                            }
                            occupied[pixel] = true;
                            let offset = pixel * 4;
                            self.video.pixels_mut()[offset..offset + 4]
                                .copy_from_slice(&PALETTE[usize::from(color & 0x0f)]);
                        }
                    }
                    count[line] = count[line].saturating_add(1);
                }
            }
        }
    }

    pub fn save(&self, out: &mut StateWriter) {
        out.blob(&self.vram);
        out.blob(&self.regs);
        out.u16(self.address);
        out.u8(self.latch.unwrap_or(0));
        out.u8(self.latch.is_some() as u8);
        out.u8(self.read_buffer);
        out.u8(self.status);
        out.u16(self.scanline);
        out.u16(self.dot);
        out.u8(self.dot_half);
        out.u64(self.frame);
        out.u8(self.irq as u8);
        out.u8(self.irq_edge as u8);
    }

    pub fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let vram = input.blob()?;
        if vram.len() != self.vram.len() {
            return Err("invalid TMS9918 VRAM length".into());
        }
        self.vram.copy_from_slice(vram);
        let regs = input.blob()?;
        if regs.len() != self.regs.len() {
            return Err("invalid TMS9918 register length".into());
        }
        self.regs.copy_from_slice(regs);
        self.address = input.u16()? & 0x3fff;
        let latch = input.u8()?;
        self.latch = if input.u8()? != 0 { Some(latch) } else { None };
        self.read_buffer = input.u8()?;
        self.status = input.u8()?;
        self.scanline = input.u16()?;
        self.dot = input.u16()?;
        self.dot_half = input.u8()?;
        self.frame = input.u64()?;
        self.irq = input.u8()? != 0;
        self.irq_edge = input.u8()? != 0;
        self.render_frame();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn set_register(vdp: &mut Tms9918, register: u8, value: u8) {
        vdp.write_control(value);
        vdp.write_control(0x80 | register);
    }

    #[test]
    fn graphics_two_renders_and_vblank_interrupts() {
        let mut vdp = Tms9918::default();
        set_register(&mut vdp, 0, 0x02);
        set_register(&mut vdp, 1, 0x60);
        set_register(&mut vdp, 2, 0x06);
        set_register(&mut vdp, 3, 0x80);
        set_register(&mut vdp, 4, 0x00);
        set_register(&mut vdp, 7, 0x01);
        vdp.vram[0x1800] = 0;
        vdp.vram[0x0000] = 0xff;
        vdp.vram[0x2000] = 0xf1;
        vdp.tick_cpu_cycles((342 * 192 * 2) / 3 + 4);
        assert!(vdp.frame() >= 1);
        assert!(vdp.irq_pending());
        let pixel = &vdp.video().pixels()[..4];
        assert_eq!(pixel, &PALETTE[15]);
        let status = vdp.read_status();
        assert_ne!(status & 0x80, 0);
        assert!(!vdp.irq_pending());
    }

    #[test]
    fn state_round_trip_preserves_vram_and_registers() {
        let mut vdp = Tms9918::default();
        set_register(&mut vdp, 7, 0x0f);
        vdp.vram[0x1234] = 0x5a;
        let mut writer = StateWriter::new(crate::platform::PlatformId::ColecoVision, 91);
        vdp.save(&mut writer);
        let bytes = writer.finish();
        let mut restored = Tms9918::default();
        let mut reader =
            StateReader::new(&bytes, crate::platform::PlatformId::ColecoVision, 91).unwrap();
        restored.load(&mut reader).unwrap();
        reader.finish().unwrap();
        assert_eq!(restored.vram[0x1234], 0x5a);
        assert_eq!(restored.regs[7], 0x0f);
    }
}
