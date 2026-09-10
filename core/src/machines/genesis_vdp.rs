use crate::kernel::VideoBuffer;
use crate::state::{StateReader, StateWriter};

const WIDTH: u32 = 320;
const HEIGHT: u32 = 224;
const LINES_PER_FRAME: u16 = 262;
const CPU_CYCLES_PER_LINE: u16 = 488;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct Pixel {
    color: u8,
    priority: bool,
}

pub struct GenesisVdp {
    vram: Box<[u8; 0x10000]>,
    cram: [u16; 64],
    vsram: [u16; 40],
    regs: [u8; 24],
    address: u16,
    code: u8,
    control_first: Option<u16>,
    read_buffer: u16,
    status: u16,
    line: u16,
    cycle: u16,
    frame: u64,
    h_counter: u8,
    hint_pending: bool,
    vint_pending: bool,
    video: VideoBuffer,
}
impl Default for GenesisVdp {
    fn default() -> Self {
        Self {
            vram: Box::new([0; 0x10000]),
            cram: [0; 64],
            vsram: [0; 40],
            regs: [0; 24],
            address: 0,
            code: 0,
            control_first: None,
            read_buffer: 0,
            status: 0x3400,
            line: 0,
            cycle: 0,
            frame: 0,
            h_counter: 0,
            hint_pending: false,
            vint_pending: false,
            video: VideoBuffer::new(WIDTH, HEIGHT),
        }
    }
}

impl GenesisVdp {
    pub fn reset(&mut self) {
        *self = Self::default();
        self.video.clear([0, 0, 0, 255]);
    }

    pub fn video(&self) -> &VideoBuffer {
        &self.video
    }
    pub fn frame(&self) -> u64 {
        self.frame
    }
    pub fn irq_level(&self) -> u8 {
        if self.vint_pending && self.regs[1] & 0x20 != 0 {
            6
        } else if self.hint_pending && self.regs[0] & 0x10 != 0 {
            4
        } else {
            0
        }
    }

    pub fn acknowledge_irq(&mut self, level: u8) {
        if level >= 6 {
            self.vint_pending = false;
        }
        if level >= 4 {
            self.hint_pending = false;
        }
    }
    pub fn read_status(&mut self) -> u16 {
        self.control_first = None;
        let mut status = self.status;
        if self.line >= HEIGHT as u16 {
            status |= 0x0008;
        }
        if self.cycle >= 430 {
            status |= 0x0004;
        }
        status
    }

    pub fn read_hv_counter(&self) -> u16 {
        let h = ((u32::from(self.cycle) * 256) / u32::from(CPU_CYCLES_PER_LINE)) as u8;
        ((self.line as u8 as u16) << 8) | u16::from(h)
    }

    pub fn write_control(&mut self, word: u16) {
        if word & 0xc000 == 0x8000 {
            let register = usize::from((word >> 8) & 0x1f);
            if register < self.regs.len() {
                self.regs[register] = word as u8;
                if register == 10 {
                    self.h_counter = self.regs[10];
                }
            }
            self.control_first = None;
            return;
        }
        let Some(first) = self.control_first.take() else {
            self.control_first = Some(word);
            return;
        };
        self.address = (first & 0x3fff) | ((word & 0x0003) << 14);
        self.code = (((first >> 14) & 3) | ((word >> 2) & 0x3c)) as u8;
        if self.code & 0x0f == 0 {
            self.read_buffer = self.read_target();
        }
    }

    fn auto_increment(&mut self) {
        self.address = self.address.wrapping_add(u16::from(self.regs[15].max(1)));
    }
    fn read_target(&self) -> u16 {
        match self.code & 0x0f {
            0x00 => {
                let a = usize::from(self.address & 0xfffe);
                u16::from_be_bytes([self.vram[a], self.vram[(a + 1) & 0xffff]])
            }
            0x08 => self.cram[usize::from(self.address >> 1) & 0x3f],
            0x04 => self.vsram[usize::from(self.address >> 1) % self.vsram.len()],
            _ => 0xffff,
        }
    }

    pub fn read_data(&mut self) -> u16 {
        let value = self.read_buffer;
        self.read_buffer = self.read_target();
        self.auto_increment();
        value
    }

    pub fn write_data(&mut self, word: u16) {
        match self.code & 0x0f {
            0x01 => {
                let a = usize::from(self.address & 0xfffe);
                let [high, low] = word.to_be_bytes();
                self.vram[a] = high;
                self.vram[(a + 1) & 0xffff] = low;
            }
            0x03 => {
                self.cram[usize::from(self.address >> 1) & 0x3f] = word & 0x0eee;
            }
            0x05 => {
                let index = usize::from(self.address >> 1) % self.vsram.len();
                self.vsram[index] = word & 0x07ff;
            }
            _ => {}
        }
        self.auto_increment();
        self.control_first = None;
    }
    pub fn tick_cpu_cycles(&mut self, cycles: u32) {
        let mut clocks = u32::from(self.cycle) + cycles;
        while clocks >= u32::from(CPU_CYCLES_PER_LINE) {
            clocks -= u32::from(CPU_CYCLES_PER_LINE);
            self.advance_scanline();
        }
        self.cycle = clocks as u16;
    }

    fn advance_scanline(&mut self) {
        if self.line < HEIGHT as u16 {
            if self.h_counter == 0 {
                self.h_counter = self.regs[10];
                if self.regs[0] & 0x10 != 0 {
                    self.hint_pending = true;
                }
            } else {
                self.h_counter = self.h_counter.wrapping_sub(1);
            }
        }
        self.line += 1;
        if self.line == HEIGHT as u16 {
            self.status |= 0x0008;
            self.vint_pending = true;
            self.render_frame();
            self.frame = self.frame.wrapping_add(1);
        }
        if self.line >= LINES_PER_FRAME {
            self.line = 0;
            self.status &= !0x0008;
            self.h_counter = self.regs[10];
        }
    }

    fn active_width(&self) -> usize {
        if self.regs[12] & 1 != 0 {
            320
        } else {
            256
        }
    }

    fn plane_cells(code: u8) -> usize {
        match code & 3 {
            0 => 32,
            1 => 64,
            3 => 128,
            _ => 32,
        }
    }

    fn read_vram_word(&self, address: usize) -> u16 {
        let a = address & 0xffff;
        u16::from_be_bytes([self.vram[a], self.vram[(a + 1) & 0xffff]])
    }
    fn cram_color(&self, index: u8) -> [u8; 4] {
        let word = self.cram[usize::from(index & 0x3f)];
        let r = ((word >> 1) & 7) as u8;
        let g = ((word >> 5) & 7) as u8;
        let b = ((word >> 9) & 7) as u8;
        [r * 36, g * 36, b * 36, 255]
    }

    fn signed_scroll(value: u16) -> i32 {
        let value = value & 0x07ff;
        if value & 0x0400 != 0 {
            i32::from(value) - 0x800
        } else {
            i32::from(value)
        }
    }

    fn hscroll(&self, plane: usize, line: usize) -> i32 {
        let base = usize::from(self.regs[13] & 0x3f) << 10;
        let mode = self.regs[11] & 3;
        let offset = match mode {
            0 => plane * 2,
            2 => (line & !7) * 4 + plane * 2,
            3 => line * 4 + plane * 2,
            _ => plane * 2,
        };
        Self::signed_scroll(self.read_vram_word(base + offset))
    }

    fn vscroll(&self, plane: usize) -> i32 {
        Self::signed_scroll(self.vsram[plane.min(1)])
    }

    fn tile_pixel(&self, descriptor: u16, x: usize, y: usize) -> Pixel {
        let tile = usize::from(descriptor & 0x07ff);
        let flip_x = descriptor & 0x0800 != 0;
        let flip_y = descriptor & 0x1000 != 0;
        let palette = ((descriptor >> 13) & 3) as u8;
        let sx = if flip_x { 7 - x } else { x };
        let sy = if flip_y { 7 - y } else { y };
        let byte = self.vram[(tile * 32 + sy * 4 + sx / 2) & 0xffff];
        let nibble = if sx & 1 == 0 { byte >> 4 } else { byte & 0x0f };
        Pixel {
            color: palette * 16 + nibble,
            priority: descriptor & 0x8000 != 0,
        }
    }
    fn plane_pixel(&self, plane: usize, x: usize, y: usize) -> Pixel {
        let base = if plane == 0 {
            usize::from(self.regs[2] & 0x38) << 10
        } else {
            usize::from(self.regs[4] & 0x07) << 13
        };
        let cells_x = Self::plane_cells(self.regs[16]);
        let cells_y = Self::plane_cells(self.regs[16] >> 4);
        let width = cells_x * 8;
        let height = cells_y * 8;
        let world_x = (x as i32 - self.hscroll(plane, y)).rem_euclid(width as i32) as usize;
        let world_y = (y as i32 + self.vscroll(plane)).rem_euclid(height as i32) as usize;
        let cell_x = world_x / 8;
        let cell_y = world_y / 8;
        let entry = base + (cell_y * cells_x + cell_x) * 2;
        let descriptor = self.read_vram_word(entry);
        self.tile_pixel(descriptor, world_x & 7, world_y & 7)
    }

    fn overlay(dst: &mut Pixel, src: Pixel) {
        if src.color & 0x0f == 0 {
            return;
        }
        if src.priority || !dst.priority {
            *dst = src;
        }
    }

    fn render_planes(&self, pixels: &mut [Pixel], width: usize) {
        for y in 0..HEIGHT as usize {
            for x in 0..width {
                let index = y * WIDTH as usize + x;
                let b = self.plane_pixel(1, x, y);
                Self::overlay(&mut pixels[index], b);
                let a = self.plane_pixel(0, x, y);
                Self::overlay(&mut pixels[index], a);
            }
        }
    }
    fn render_sprites(&mut self, pixels: &mut [Pixel], width: usize) {
        let table = usize::from(self.regs[5] & if width == 320 { 0x7e } else { 0x7f }) << 9;
        let mut occupied = vec![false; WIDTH as usize * HEIGHT as usize];
        let mut link = 0usize;
        let max_sprites = if width == 320 { 80 } else { 64 };
        for _ in 0..max_sprites {
            let base = (table + link * 8) & 0xffff;
            let y_word = self.read_vram_word(base);
            let size_link = self.read_vram_word(base + 2);
            let attribute = self.read_vram_word(base + 4);
            let x_word = self.read_vram_word(base + 6);
            let cells_x = usize::from((size_link >> 10) & 3) + 1;
            let cells_y = usize::from((size_link >> 8) & 3) + 1;
            let x0 = i32::from(x_word & 0x01ff) - 128;
            let y0 = i32::from(y_word & 0x03ff) - 128;
            let flip_x = attribute & 0x0800 != 0;
            let flip_y = attribute & 0x1000 != 0;
            let palette = ((attribute >> 13) & 3) as u8;
            let priority = attribute & 0x8000 != 0;
            let first_tile = usize::from(attribute & 0x07ff);
            self.draw_sprite(
                pixels,
                &mut occupied,
                width,
                x0,
                y0,
                cells_x,
                cells_y,
                first_tile,
                palette,
                priority,
                flip_x,
                flip_y,
            );
            let next = usize::from(size_link & 0x007f);
            if next == 0 || next == link {
                break;
            }
            link = next;
        }
    }
    #[allow(clippy::too_many_arguments)]
    fn draw_sprite(
        &mut self,
        pixels: &mut [Pixel],
        occupied: &mut [bool],
        active_width: usize,
        x0: i32,
        y0: i32,
        cells_x: usize,
        cells_y: usize,
        first_tile: usize,
        palette: u8,
        priority: bool,
        flip_x: bool,
        flip_y: bool,
    ) {
        let sprite_width = cells_x * 8;
        let sprite_height = cells_y * 8;
        for sy in 0..sprite_height {
            let source_y = if flip_y { sprite_height - 1 - sy } else { sy };
            let tile_y = source_y / 8;
            let row = source_y & 7;
            for sx in 0..sprite_width {
                let screen_x = x0 + sx as i32;
                let screen_y = y0 + sy as i32;
                if screen_x < 0
                    || screen_y < 0
                    || screen_x >= active_width as i32
                    || screen_y >= HEIGHT as i32
                {
                    continue;
                }
                let source_x = if flip_x { sprite_width - 1 - sx } else { sx };
                let tile_x = source_x / 8;
                let column = source_x & 7;
                let tile = first_tile + tile_x * cells_y + tile_y;
                let byte = self.vram[(tile * 32 + row * 4 + column / 2) & 0xffff];
                let nibble = if column & 1 == 0 {
                    byte >> 4
                } else {
                    byte & 0x0f
                };
                if nibble == 0 {
                    continue;
                }
                let index = screen_y as usize * WIDTH as usize + screen_x as usize;
                if occupied[index] {
                    self.status |= 0x0020;
                }
                occupied[index] = true;
                Self::overlay(
                    &mut pixels[index],
                    Pixel {
                        color: palette * 16 + nibble,
                        priority,
                    },
                );
            }
        }
    }
    fn render_frame(&mut self) {
        self.status &= !0x0060;
        let background = self.regs[7] & 0x3f;
        let mut pixels = vec![
            Pixel {
                color: background,
                priority: false
            };
            WIDTH as usize * HEIGHT as usize
        ];
        let active_width = self.active_width();
        if self.regs[1] & 0x40 != 0 {
            self.render_planes(&mut pixels, active_width);
            self.render_sprites(&mut pixels, active_width);
        }
        self.video.clear([0, 0, 0, 255]);
        for y in 0..HEIGHT as usize {
            for x in 0..active_width {
                let rgba = self.cram_color(pixels[y * WIDTH as usize + x].color);
                let offset = (y * WIDTH as usize + x) * 4;
                self.video.pixels_mut()[offset..offset + 4].copy_from_slice(&rgba);
            }
        }
    }

    pub fn save(&self, out: &mut StateWriter) {
        out.blob(self.vram.as_slice());
        for word in self.cram {
            out.u16(word);
        }
        for word in self.vsram {
            out.u16(word);
        }
        out.blob(&self.regs);
        out.u16(self.address);
        out.u8(self.code);
        out.u8(self.control_first.is_some() as u8);
        out.u16(self.control_first.unwrap_or(0));
        out.u16(self.read_buffer);
        out.u16(self.status);
        out.u16(self.line);
        out.u16(self.cycle);
        out.u64(self.frame);
        out.u8(self.h_counter);
        out.u8(self.hint_pending as u8);
        out.u8(self.vint_pending as u8);
    }
    pub fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let vram = input.blob()?;
        if vram.len() != self.vram.len() {
            return Err("invalid Genesis VRAM state length".into());
        }
        self.vram.copy_from_slice(vram);
        for word in &mut self.cram {
            *word = input.u16()?;
        }
        for word in &mut self.vsram {
            *word = input.u16()?;
        }
        let regs = input.blob()?;
        if regs.len() != self.regs.len() {
            return Err("invalid Genesis VDP register state length".into());
        }
        self.regs.copy_from_slice(regs);
        self.address = input.u16()?;
        self.code = input.u8()?;
        let has_first = input.u8()? != 0;
        let first = input.u16()?;
        self.control_first = has_first.then_some(first);
        self.read_buffer = input.u16()?;
        self.status = input.u16()?;
        self.line = input.u16()?;
        self.cycle = input.u16()?;
        self.frame = input.u64()?;
        self.h_counter = input.u8()?;
        self.hint_pending = input.u8()? != 0;
        self.vint_pending = input.u8()? != 0;
        self.render_frame();
        Ok(())
    }
}
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn command_port_writes_vram_cram_and_vsram() {
        let mut vdp = GenesisVdp::default();
        vdp.regs[15] = 2;
        vdp.write_control(0x4000);
        vdp.write_control(0x0000);
        vdp.write_data(0x1234);
        assert_eq!(&vdp.vram[..2], &[0x12, 0x34]);
        vdp.write_control(0xc000);
        vdp.write_control(0x0000);
        vdp.write_data(0x0eee);
        assert_eq!(vdp.cram[0], 0x0eee);
        vdp.write_control(0x4000);
        vdp.write_control(0x0010);
        vdp.write_data(0x0123);
        assert_eq!(vdp.vsram[0], 0x0123);
    }

    #[test]
    fn timing_raises_vblank_interrupt_and_advances_frame() {
        let mut vdp = GenesisVdp::default();
        vdp.regs[1] = 0x20;
        vdp.tick_cpu_cycles(u32::from(CPU_CYCLES_PER_LINE) * HEIGHT);
        assert_eq!(vdp.frame(), 1);
        assert_eq!(vdp.irq_level(), 6);
        vdp.acknowledge_irq(6);
        assert_eq!(vdp.irq_level(), 0);
    }
    #[test]
    fn plane_tile_renders_from_vram_and_cram() {
        let mut vdp = GenesisVdp::default();
        vdp.regs[1] = 0x40;
        vdp.regs[2] = 0x08;
        vdp.regs[4] = 0x01;
        vdp.regs[12] = 1;
        vdp.regs[16] = 0;
        vdp.cram[1] = 0x000e;
        for row in 0..8usize {
            for pair in 0..4usize {
                vdp.vram[32 + row * 4 + pair] = 0x11;
            }
        }
        let plane_a = usize::from(vdp.regs[2] & 0x38) << 10;
        vdp.vram[plane_a] = 0x00;
        vdp.vram[plane_a + 1] = 0x01;
        vdp.render_frame();
        let first = &vdp.video.pixels()[..4];
        assert!(first[0] > 0);
        assert_eq!(first[3], 255);
    }

    #[test]
    fn state_round_trip_preserves_vram_registers_and_frame() {
        let mut vdp = GenesisVdp::default();
        vdp.vram[0x1234] = 0xa5;
        vdp.regs[7] = 5;
        vdp.tick_cpu_cycles(u32::from(CPU_CYCLES_PER_LINE) * HEIGHT);
        let mut out = StateWriter::new(crate::platform::PlatformId::Genesis, 1);
        vdp.save(&mut out);
        let bytes = out.finish();
        let mut restored = GenesisVdp::default();
        let mut input = StateReader::new(&bytes, crate::platform::PlatformId::Genesis, 1).unwrap();
        restored.load(&mut input).unwrap();
        input.finish().unwrap();
        assert_eq!(restored.vram[0x1234], 0xa5);
        assert_eq!(restored.regs[7], 5);
        assert_eq!(restored.frame(), 1);
    }
}
