use crate::kernel::VideoBuffer;
use crate::state::{StateReader, StateWriter};

const WIDTH: u32 = 256;
const HEIGHT: u32 = 224;
const VRAM_BYTES: usize = 64 * 1024;
const OAM_BYTES: usize = 544;

pub struct SnesPpu {
    regs: [u8; 0x40],
    vram: Vec<u8>,
    cgram: [u16; 256],
    oam: [u8; OAM_BYTES],
    video: VideoBuffer,
    vmaddr: u16,
    vmain: u8,
    cgaddr: u8,
    cg_latch: u8,
    cg_high: bool,
    oam_addr: u16,
    cycle: u16,
    scanline: u16,
    frame: u64,
    vblank: bool,
    nmi_pending: bool,
}

impl Default for SnesPpu {
    fn default() -> Self {
        Self::new()
    }
}
impl SnesPpu {
    pub fn new() -> Self {
        let mut video = VideoBuffer::new(WIDTH, HEIGHT);
        video.clear([0, 0, 0, 255]);
        Self {
            regs: [0; 0x40],
            vram: vec![0; VRAM_BYTES],
            cgram: [0; 256],
            oam: [0; OAM_BYTES],
            video,
            vmaddr: 0,
            vmain: 0,
            cgaddr: 0,
            cg_latch: 0,
            cg_high: false,
            oam_addr: 0,
            cycle: 0,
            scanline: 0,
            frame: 0,
            vblank: false,
            nmi_pending: false,
        }
    }

    pub fn reset(&mut self) {
        *self = Self::new();
    }

    pub fn video(&self) -> &VideoBuffer {
        &self.video
    }
    pub fn frame(&self) -> u64 {
        self.frame
    }

    pub fn in_vblank(&self) -> bool {
        self.vblank
    }

    pub fn take_nmi(&mut self) -> bool {
        let pending = self.nmi_pending;
        self.nmi_pending = false;
        pending
    }

    fn vram_increment(&self) -> u16 {
        match self.vmain & 0x03 {
            0 => 1,
            1 => 32,
            2 | 3 => 128,
            _ => 1,
        }
    }

    fn increment_vram(&mut self) {
        self.vmaddr = self.vmaddr.wrapping_add(self.vram_increment());
    }
    fn vram_byte_index(&self, high: bool) -> usize {
        (usize::from(self.vmaddr) * 2 + usize::from(high)) & 0xffff
    }

    pub fn write(&mut self, address: u16, value: u8) {
        let reg = (address & 0x3f) as usize;
        if reg < self.regs.len() {
            self.regs[reg] = value;
        }
        match address {
            0x2102 => self.oam_addr = (self.oam_addr & 0x100) | u16::from(value),
            0x2103 => self.oam_addr = (self.oam_addr & 0xff) | (u16::from(value & 1) << 8),
            0x2104 => {
                let index = usize::from(self.oam_addr) % OAM_BYTES;
                self.oam[index] = value;
                self.oam_addr = self.oam_addr.wrapping_add(1) % OAM_BYTES as u16;
            }
            0x2115 => self.vmain = value,
            0x2116 => self.vmaddr = (self.vmaddr & 0xff00) | u16::from(value),
            0x2117 => self.vmaddr = (self.vmaddr & 0x00ff) | (u16::from(value) << 8),
            0x2118 => {
                let index = self.vram_byte_index(false);
                self.vram[index] = value;
                if self.vmain & 0x80 == 0 {
                    self.increment_vram();
                }
            }
            0x2119 => {
                let index = self.vram_byte_index(true);
                self.vram[index] = value;
                if self.vmain & 0x80 != 0 {
                    self.increment_vram();
                }
            }
            0x2121 => {
                self.cgaddr = value;
                self.cg_high = false;
            }
            0x2122 => {
                if !self.cg_high {
                    self.cg_latch = value;
                    self.cg_high = true;
                } else {
                    self.cgram[usize::from(self.cgaddr)] =
                        u16::from(self.cg_latch) | (u16::from(value & 0x7f) << 8);
                    self.cgaddr = self.cgaddr.wrapping_add(1);
                    self.cg_high = false;
                }
            }
            _ => {}
        }
    }

    pub fn read(&mut self, address: u16) -> u8 {
        match address {
            0x2138 => {
                let index = usize::from(self.oam_addr) % OAM_BYTES;
                let value = self.oam[index];
                self.oam_addr = self.oam_addr.wrapping_add(1) % OAM_BYTES as u16;
                value
            }
            0x2139 => {
                let value = self.vram[self.vram_byte_index(false)];
                if self.vmain & 0x80 == 0 {
                    self.increment_vram();
                }
                value
            }
            0x213a => {
                let value = self.vram[self.vram_byte_index(true)];
                if self.vmain & 0x80 != 0 {
                    self.increment_vram();
                }
                value
            }
            0x213b => {
                let color = self.cgram[usize::from(self.cgaddr)];
                if !self.cg_high {
                    self.cg_high = true;
                    color as u8
                } else {
                    self.cg_high = false;
                    self.cgaddr = self.cgaddr.wrapping_add(1);
                    (color >> 8) as u8
                }
            }
            0x213c => (self.cycle & 0xff) as u8,
            0x213d => (self.scanline & 0xff) as u8,
            0x213e => 0x01,
            0x213f => u8::from(self.vblank) << 7,
            _ => 0,
        }
    }

    pub fn tick_dots(&mut self, dots: u32) {
        for _ in 0..dots {
            self.cycle += 1;
            if self.cycle >= 341 {
                self.cycle = 0;
                self.scanline += 1;
                if self.scanline == 225 {
                    self.vblank = true;
                    self.render_frame();
                    self.frame = self.frame.wrapping_add(1);
                    self.nmi_pending = true;
                } else if self.scanline >= 262 {
                    self.scanline = 0;
                    self.vblank = false;
                }
            }
        }
    }

    fn color_rgba(&self, color: u16) -> [u8; 4] {
        let brightness = u16::from(self.regs[0] & 0x0f);
        let scale = |component: u16| -> u8 {
            let five = u32::from(component & 0x1f);
            let value = five * 255 * u32::from(brightness) / (31 * 15);
            value.min(255) as u8
        };
        [scale(color), scale(color >> 5), scale(color >> 10), 255]
    }
    fn render_frame(&mut self) {
        if self.regs[0] & 0x80 != 0 {
            self.video.clear([0, 0, 0, 255]);
            return;
        }
        let backdrop = self.color_rgba(self.cgram[0]);
        self.video.clear(backdrop);
        if self.regs[0x2c] & 0x01 == 0 {
            return;
        }
        let mode = self.regs[0x05] & 0x07;
        if mode > 1 {
            return;
        }
        self.render_bg1(mode);
    }

    fn read_vram16(&self, address: usize) -> u16 {
        let lo = self.vram[address & 0xffff];
        let hi = self.vram[(address + 1) & 0xffff];
        u16::from_le_bytes([lo, hi])
    }

    fn render_bg1(&mut self, mode: u8) {
        let map_base = (usize::from(self.regs[0x07] & 0xfc) << 9) & 0xffff;
        let tile_base = (usize::from(self.regs[0x0b] & 0x0f) << 13) & 0xffff;
        let bpp = if mode == 0 { 2usize } else { 4usize };
        let tile_bytes = bpp * 8;
        for y in 0..HEIGHT as usize {
            for x in 0..WIDTH as usize {
                let tile_x = x / 8;
                let tile_y = y / 8;
                let map_index = tile_y * 32 + tile_x;
                let entry = self.read_vram16(map_base + map_index * 2);
                let tile = usize::from(entry & 0x03ff);
                let palette = usize::from((entry >> 10) & 0x07);
                let hflip = entry & 0x4000 != 0;
                let vflip = entry & 0x8000 != 0;
                let mut px = x & 7;
                let mut py = y & 7;
                if hflip {
                    px = 7 - px;
                }
                if vflip {
                    py = 7 - py;
                }
                let base = (tile_base + tile * tile_bytes) & 0xffff;
                let bit = 7 - px;
                let mut color_index = 0u8;
                for plane in 0..bpp {
                    let group = plane / 2;
                    let byte = self.vram[(base + group * 16 + py * 2 + plane % 2) & 0xffff];
                    color_index |= ((byte >> bit) & 1) << plane;
                }
                if color_index == 0 {
                    continue;
                }
                let index = if mode == 0 {
                    palette * 4 + usize::from(color_index)
                } else {
                    palette * 16 + usize::from(color_index)
                };
                let rgba = self.color_rgba(self.cgram[index & 0xff]);
                let offset = (y * WIDTH as usize + x) * 4;
                self.video.pixels_mut()[offset..offset + 4].copy_from_slice(&rgba);
            }
        }
    }

    pub fn save(&self, out: &mut StateWriter) {
        out.blob(&self.regs);
        out.blob(&self.vram);
        for color in self.cgram {
            out.u16(color);
        }
        out.blob(&self.oam);
        out.u16(self.vmaddr);
        out.u8(self.vmain);
        out.u8(self.cgaddr);
        out.u8(self.cg_latch);
        out.u8(self.cg_high as u8);
        out.u16(self.oam_addr);
        out.u16(self.cycle);
        out.u16(self.scanline);
        out.u64(self.frame);
        out.u8(self.vblank as u8);
        out.u8(self.nmi_pending as u8);
    }

    pub fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let regs = input.blob()?;
        if regs.len() != self.regs.len() {
            return Err("invalid SNES PPU register state length".into());
        }
        self.regs.copy_from_slice(regs);
        let vram = input.blob()?;
        if vram.len() != self.vram.len() {
            return Err("invalid SNES VRAM state length".into());
        }
        self.vram.copy_from_slice(vram);
        for color in &mut self.cgram {
            *color = input.u16()?;
        }
        let oam = input.blob()?;
        if oam.len() != self.oam.len() {
            return Err("invalid SNES OAM state length".into());
        }
        self.oam.copy_from_slice(oam);
        self.vmaddr = input.u16()?;
        self.vmain = input.u8()?;
        self.cgaddr = input.u8()?;
        self.cg_latch = input.u8()?;
        self.cg_high = input.u8()? != 0;
        self.oam_addr = input.u16()?;
        self.cycle = input.u16()?;
        self.scanline = input.u16()?;
        self.frame = input.u64()?;
        self.vblank = input.u8()? != 0;
        self.nmi_pending = input.u8()? != 0;
        self.render_frame();
        Ok(())
    }

    #[cfg(test)]
    pub(crate) fn force_render(&mut self) {
        self.render_frame();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cgram_backdrop_renders_with_brightness() {
        let mut ppu = SnesPpu::new();
        ppu.write(0x2100, 0x0f);
        ppu.write(0x2121, 0);
        ppu.write(0x2122, 0x1f);
        ppu.write(0x2122, 0x00);
        ppu.force_render();
        assert_eq!(&ppu.video().pixels()[..4], &[255, 0, 0, 255]);
    }

    #[test]
    fn mode1_bg1_tile_reads_vram_and_palette() {
        let mut ppu = SnesPpu::new();
        ppu.write(0x2100, 0x0f);
        ppu.write(0x2105, 1);
        ppu.write(0x2107, 0x04);
        ppu.write(0x210b, 0x00);
        ppu.write(0x212c, 0x01);
        ppu.cgram[1] = 0x03e0;
        for row in 0..8 {
            ppu.vram[row * 2] = 0xff;
        }
        ppu.vram[0x800] = 0;
        ppu.vram[0x801] = 0;
        ppu.force_render();
        assert!(ppu.video().pixels()[1] > 200);
    }
}
