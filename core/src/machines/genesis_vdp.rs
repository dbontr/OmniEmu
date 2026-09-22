use crate::kernel::VideoBuffer;
use crate::state::{StateReader, StateWriter};

const WIDTH: u32 = 320;
const HEIGHT: u32 = 224;
const NTSC_LINES_PER_FRAME: u16 = 262;
const PAL_LINES_PER_FRAME: u16 = 313;
const CPU_CYCLES_PER_LINE: u16 = 488;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct Pixel {
    color: u8,
    priority: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct MemoryDmaState {
    source: u32,
    remaining: u32,
    transferred: u32,
    cycle_fraction: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum InternalDmaKind {
    Fill,
    Copy,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct InternalDmaState {
    kind: InternalDmaKind,
    source: u16,
    fill: u8,
    remaining: u32,
    transferred: u32,
    slot_phase: u32,
}

pub struct GenesisVdp {
    pal: bool,
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
    memory_dma: Option<MemoryDmaState>,
    internal_dma: Option<InternalDmaState>,
    video: VideoBuffer,
}
impl Default for GenesisVdp {
    fn default() -> Self {
        Self::new(false)
    }
}

impl GenesisVdp {
    pub fn new(pal: bool) -> Self {
        Self {
            pal,
            vram: Box::new([0; 0x10000]),
            cram: [0; 64],
            vsram: [0; 40],
            regs: [0; 24],
            address: 0,
            code: 0,
            control_first: None,
            read_buffer: 0,
            status: 0x3400 | u16::from(pal),
            line: 0,
            cycle: 0,
            frame: 0,
            h_counter: 0,
            hint_pending: false,
            vint_pending: false,
            memory_dma: None,
            internal_dma: None,
            video: VideoBuffer::new(WIDTH, HEIGHT),
        }
    }

    fn lines_per_frame(&self) -> u16 {
        if self.pal {
            PAL_LINES_PER_FRAME
        } else {
            NTSC_LINES_PER_FRAME
        }
    }

    fn v_counter(&self) -> u8 {
        if self.pal {
            match self.line {
                0..=255 => self.line as u8,
                256..=258 => (self.line - 256) as u8,
                _ => (self.line - 57) as u8,
            }
        } else if self.line <= 234 {
            self.line as u8
        } else {
            (self.line - 6) as u8
        }
    }

    pub fn reset(&mut self) {
        let pal = self.pal;
        *self = Self::new(pal);
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
        (u16::from(self.v_counter()) << 8) | u16::from(h)
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
        if self.code & 0x20 == 0 || self.regs[1] & 0x10 == 0 {
            return;
        }
        match self.regs[23] >> 6 {
            2 => {
                if self.code & 0x0f == 0x01 {
                    self.status |= 0x0002;
                    self.memory_dma = None;
                    self.internal_dma = None;
                }
            }
            3 => {
                if self.code & 0x0f == 0x00 {
                    self.status |= 0x0002;
                    self.memory_dma = None;
                    self.internal_dma = Some(InternalDmaState {
                        kind: InternalDmaKind::Copy,
                        source: u16::from(self.regs[21]) | (u16::from(self.regs[22]) << 8),
                        fill: 0,
                        remaining: self.dma_length(),
                        transferred: 0,
                        slot_phase: 0,
                    });
                }
            }
            _ if matches!(self.code & 0x0f, 0x01 | 0x03 | 0x05) => {
                self.status |= 0x0002;
                self.internal_dma = None;
                self.memory_dma = Some(MemoryDmaState {
                    source: self.dma_source_address(),
                    remaining: self.dma_length(),
                    transferred: 0,
                    cycle_fraction: 0,
                });
            }
            _ => {}
        }
    }

    fn dma_length(&self) -> u32 {
        let length = u32::from(self.regs[19]) | (u32::from(self.regs[20]) << 8);
        if length == 0 {
            0x1_0000
        } else {
            length
        }
    }

    fn dma_source_address(&self) -> u32 {
        let words = u32::from(self.regs[21])
            | (u32::from(self.regs[22]) << 8)
            | (u32::from(self.regs[23] & 0x7f) << 16);
        (words << 1) & 0x00ff_ffff
    }

    fn complete_dma(&mut self, transfers: u32) {
        let source = u32::from(self.regs[21]) | (u32::from(self.regs[22]) << 8);
        let end = source.wrapping_add(transfers) as u16;
        self.regs[21] = end as u8;
        self.regs[22] = (end >> 8) as u8;
        self.regs[19] = 0;
        self.regs[20] = 0;
        self.status &= !0x0002;
    }

    fn memory_dma_words_per_line(&self) -> u32 {
        let h40 = self.regs[12] & 1 != 0;
        let display_active = self.regs[1] & 0x40 != 0 && self.line < HEIGHT as u16;
        let external_slots = match (display_active, h40) {
            (true, false) => 16,
            (true, true) => 18,
            (false, false) => 166,
            (false, true) => 204,
        };
        if self.code & 0x0f == 0x01 {
            external_slots / 2
        } else {
            external_slots
        }
    }

    pub(super) fn memory_dma_pending(&self) -> bool {
        self.memory_dma.is_some()
    }

    pub(super) fn memory_dma_source(&self) -> Option<u32> {
        self.memory_dma.map(|state| state.source)
    }

    pub(super) fn service_memory_dma_word(&mut self, word: u16) -> Option<u32> {
        let mut state = self.memory_dma?;
        let rate = u64::from(self.memory_dma_words_per_line().max(1));
        let cost_fp = (u64::from(CPU_CYCLES_PER_LINE) << 16) / rate;
        let accumulated = u64::from(state.cycle_fraction) + cost_fp;
        let cycles = ((accumulated >> 16) as u32).max(1);
        state.cycle_fraction = (accumulated & 0xffff) as u32;

        self.write_target_word(word);
        self.auto_increment();
        state.source = state.source.wrapping_add(2) & 0x00ff_ffff;
        state.remaining = state.remaining.saturating_sub(1);
        state.transferred = state.transferred.saturating_add(1);
        if state.remaining == 0 {
            self.complete_dma(state.transferred);
            self.memory_dma = None;
        } else {
            self.memory_dma = Some(state);
        }
        Some(cycles)
    }

    fn internal_dma_transfers_per_line(&self, kind: InternalDmaKind) -> u32 {
        let h40 = self.regs[12] & 1 != 0;
        let display_active = self.regs[1] & 0x40 != 0 && self.line < HEIGHT as u16;
        match (kind, display_active, h40) {
            (InternalDmaKind::Fill, true, false) => 15,
            (InternalDmaKind::Fill, true, true) => 17,
            (InternalDmaKind::Fill, false, false) => 166,
            (InternalDmaKind::Fill, false, true) => 204,
            (InternalDmaKind::Copy, true, false) => 8,
            (InternalDmaKind::Copy, true, true) => 9,
            (InternalDmaKind::Copy, false, false) => 83,
            (InternalDmaKind::Copy, false, true) => 102,
        }
    }

    fn finish_internal_dma(&mut self, state: InternalDmaState) {
        self.regs[21] = state.source as u8;
        self.regs[22] = (state.source >> 8) as u8;
        self.regs[19] = 0;
        self.regs[20] = 0;
        self.status &= !0x0002;
        self.internal_dma = None;
    }

    fn begin_vram_fill(&mut self, word: u16) {
        let [high, low] = word.to_be_bytes();
        if let Some(mut state) = self.internal_dma {
            if state.kind == InternalDmaKind::Fill {
                self.write_target_word(word);
                self.auto_increment();
                state.fill = high;
                self.internal_dma = Some(state);
                return;
            }
        }

        let aligned = usize::from(self.address & 0xfffe);
        if self.address & 1 == 0 {
            self.vram[aligned] = low;
            self.vram[(aligned + 1) & 0xffff] = high;
        } else {
            self.vram[aligned] = high;
            self.vram[(aligned + 1) & 0xffff] = low;
        }
        self.auto_increment();
        self.internal_dma = Some(InternalDmaState {
            kind: InternalDmaKind::Fill,
            source: u16::from(self.regs[21]) | (u16::from(self.regs[22]) << 8),
            fill: high,
            remaining: self.dma_length(),
            transferred: 0,
            slot_phase: 0,
        });
    }

    fn tick_internal_dma(&mut self, cycles: u32) {
        let Some(mut state) = self.internal_dma else {
            return;
        };
        let rate = u64::from(self.internal_dma_transfers_per_line(state.kind));
        let accumulated = u64::from(state.slot_phase) + u64::from(cycles) * rate;
        let transfers =
            (accumulated / u64::from(CPU_CYCLES_PER_LINE)).min(u64::from(state.remaining)) as u32;
        state.slot_phase = (accumulated % u64::from(CPU_CYCLES_PER_LINE)) as u32;

        for _ in 0..transfers {
            let value = match state.kind {
                InternalDmaKind::Fill => state.fill,
                InternalDmaKind::Copy => self.vram[usize::from(state.source)],
            };
            self.vram[usize::from(self.address)] = value;
            state.source = state.source.wrapping_add(1);
            self.auto_increment();
            state.remaining -= 1;
            state.transferred += 1;
        }

        if state.remaining == 0 {
            self.finish_internal_dma(state);
        } else {
            self.internal_dma = Some(state);
        }
    }

    fn auto_increment(&mut self) {
        self.address = self.address.wrapping_add(u16::from(self.regs[15]));
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

    fn write_target_word(&mut self, word: u16) {
        match self.code & 0x0f {
            0x01 => {
                let a = usize::from(self.address & 0xfffe);
                let [high, low] = word.to_be_bytes();
                if self.address & 1 == 0 {
                    self.vram[a] = high;
                    self.vram[(a + 1) & 0xffff] = low;
                } else {
                    self.vram[a] = low;
                    self.vram[(a + 1) & 0xffff] = high;
                }
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
    }

    pub fn read_data(&mut self) -> u16 {
        let value = self.read_buffer;
        self.auto_increment();
        self.read_buffer = self.read_target();
        value
    }

    pub fn write_data(&mut self, word: u16) {
        if self.status & 0x0002 != 0 && self.code & 0x20 != 0 && self.regs[23] >> 6 == 2 {
            self.begin_vram_fill(word);
        } else {
            self.write_target_word(word);
            self.auto_increment();
        }
        self.control_first = None;
    }
    pub fn tick_cpu_cycles(&mut self, cycles: u32) {
        let mut remaining = cycles;
        while remaining != 0 {
            let until_scanline = u32::from(CPU_CYCLES_PER_LINE - self.cycle);
            let step = remaining.min(until_scanline);
            self.tick_internal_dma(step);
            self.cycle += step as u16;
            remaining -= step;
            if self.cycle == CPU_CYCLES_PER_LINE {
                self.cycle = 0;
                self.advance_scanline();
            }
        }
    }

    fn advance_scanline(&mut self) {
        if self.line < HEIGHT as u16 {
            if self.line == 0 {
                self.status &= !0x0060;
                self.video.clear([0, 0, 0, 255]);
            }
            self.render_scanline(self.line as usize);
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
            self.frame = self.frame.wrapping_add(1);
        }
        if self.line >= self.lines_per_frame() {
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

    fn vscroll(&self, plane: usize, x: usize) -> i32 {
        let index = if self.regs[11] & 0x04 != 0 {
            (x / 16).min(19) * 2 + plane.min(1)
        } else {
            plane.min(1)
        };
        Self::signed_scroll(self.vsram[index])
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
        let world_y = (y as i32 + self.vscroll(plane, x)).rem_euclid(height as i32) as usize;
        let cell_x = world_x / 8;
        let cell_y = world_y / 8;
        let entry = base + (cell_y * cells_x + cell_x) * 2;
        let descriptor = self.read_vram_word(entry);
        self.tile_pixel(descriptor, world_x & 7, world_y & 7)
    }

    fn window_active(&self, x: usize, y: usize) -> bool {
        let horizontal_boundary = usize::from(self.regs[17] & 0x1f) * 16;
        let vertical_boundary = usize::from(self.regs[18] & 0x1f) * 8;
        let horizontal = if self.regs[17] & 0x80 != 0 {
            x >= horizontal_boundary
        } else {
            x < horizontal_boundary
        };
        let vertical = if self.regs[18] & 0x80 != 0 {
            y >= vertical_boundary
        } else {
            y < vertical_boundary
        };
        horizontal || vertical
    }

    fn window_pixel(&self, x: usize, y: usize, active_width: usize) -> Pixel {
        let register_mask = if active_width == 320 { 0x3c } else { 0x3e };
        let base = usize::from(self.regs[3] & register_mask) << 10;
        let cells_per_row = if active_width == 320 { 64 } else { 32 };
        let cell_x = x / 8;
        let cell_y = y / 8;
        let descriptor = self.read_vram_word(base + (cell_y * cells_per_row + cell_x) * 2);
        self.tile_pixel(descriptor, x & 7, y & 7)
    }

    fn overlay(dst: &mut Pixel, src: Pixel) {
        if src.color & 0x0f == 0 {
            return;
        }
        if src.priority || !dst.priority {
            *dst = src;
        }
    }

    fn render_plane_scanline(&self, pixels: &mut [Pixel], width: usize, y: usize) {
        for (x, pixel) in pixels.iter_mut().take(width).enumerate() {
            let b = self.plane_pixel(1, x, y);
            Self::overlay(pixel, b);
            let a = if self.window_active(x, y) {
                self.window_pixel(x, y, width)
            } else {
                self.plane_pixel(0, x, y)
            };
            Self::overlay(pixel, a);
        }
    }
    fn render_sprite_scanline(&mut self, pixels: &mut [Pixel], width: usize, y: usize) {
        let table = usize::from(self.regs[5] & if width == 320 { 0x7e } else { 0x7f }) << 9;
        let max_sprites = if width == 320 { 80 } else { 64 };
        let max_line_sprites = if width == 320 { 20 } else { 16 };
        let max_line_dots = width;
        let mut occupied = vec![false; width];
        let mut line_sprites = 0usize;
        let mut line_dots = 0usize;
        let mut link = 0usize;
        for _ in 0..max_sprites {
            let base = (table + link * 8) & 0xffff;
            let y_word = self.read_vram_word(base);
            let size_link = self.read_vram_word(base + 2);
            let attribute = self.read_vram_word(base + 4);
            let x_word = self.read_vram_word(base + 6);
            let cells_x = usize::from((size_link >> 10) & 3) + 1;
            let cells_y = usize::from((size_link >> 8) & 3) + 1;
            let sprite_width = cells_x * 8;
            let sprite_height = cells_y * 8;
            let x0 = i32::from(x_word & 0x01ff) - 128;
            let y0 = i32::from(y_word & 0x03ff) - 128;
            let screen_y = y as i32;
            let on_line = screen_y >= y0 && screen_y < y0 + sprite_height as i32;
            if on_line {
                if line_sprites >= max_line_sprites {
                    self.status |= 0x0040;
                    break;
                }
                line_sprites += 1;
                let remaining = max_line_dots.saturating_sub(line_dots);
                let visible_dots = sprite_width.min(remaining);
                let truncated = visible_dots < sprite_width;
                if truncated {
                    self.status |= 0x0040;
                }
                let flip_x = attribute & 0x0800 != 0;
                let flip_y = attribute & 0x1000 != 0;
                let palette = ((attribute >> 13) & 3) as u8;
                let priority = attribute & 0x8000 != 0;
                let first_tile = usize::from(attribute & 0x07ff);
                let source_y = if flip_y {
                    sprite_height - 1 - (screen_y - y0) as usize
                } else {
                    (screen_y - y0) as usize
                };
                let tile_y = source_y / 8;
                let row = source_y & 7;
                for sx in 0..visible_dots {
                    let screen_x = x0 + sx as i32;
                    if screen_x < 0 || screen_x >= width as i32 {
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
                    let index = screen_x as usize;
                    if occupied[index] {
                        self.status |= 0x0020;
                        continue;
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
                line_dots = line_dots.saturating_add(visible_dots);
                if truncated {
                    break;
                }
            }
            let next = usize::from(size_link & 0x007f);
            if next == 0 || next == link {
                break;
            }
            link = next;
        }
    }
    fn render_scanline(&mut self, y: usize) {
        let background = self.regs[7] & 0x3f;
        let mut pixels = vec![
            Pixel {
                color: background,
                priority: false,
            };
            WIDTH as usize
        ];
        let active_width = self.active_width();
        if self.regs[1] & 0x40 != 0 {
            self.render_plane_scanline(&mut pixels, active_width, y);
            self.render_sprite_scanline(&mut pixels, active_width, y);
        }
        let row_start = y * WIDTH as usize * 4;
        self.video.pixels_mut()[row_start..row_start + WIDTH as usize * 4].fill(0);
        for (x, pixel) in pixels.iter().take(active_width).enumerate() {
            let rgba = self.cram_color(pixel.color);
            let offset = row_start + x * 4;
            self.video.pixels_mut()[offset..offset + 4].copy_from_slice(&rgba);
        }
        for x in active_width..WIDTH as usize {
            self.video.pixels_mut()[row_start + x * 4 + 3] = 255;
        }
    }

    fn render_frame(&mut self) {
        self.status &= !0x0060;
        self.video.clear([0, 0, 0, 255]);
        for y in 0..HEIGHT as usize {
            self.render_scanline(y);
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
        out.u8(self.memory_dma.is_some() as u8);
        let dma = self.memory_dma.unwrap_or_default();
        out.u32(dma.source);
        out.u32(dma.remaining);
        out.u32(dma.transferred);
        out.u32(dma.cycle_fraction);
        out.u8(self.internal_dma.is_some() as u8);
        if let Some(dma) = self.internal_dma {
            out.u8(match dma.kind {
                InternalDmaKind::Fill => 1,
                InternalDmaKind::Copy => 2,
            });
            out.u16(dma.source);
            out.u8(dma.fill);
            out.u32(dma.remaining);
            out.u32(dma.transferred);
            out.u32(dma.slot_phase);
        } else {
            out.u8(0);
            out.u16(0);
            out.u8(0);
            out.u32(0);
            out.u32(0);
            out.u32(0);
        }
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
        self.status = (input.u16()? & !1) | u16::from(self.pal);
        self.line = input.u16()? % self.lines_per_frame();
        self.cycle = input.u16()? % CPU_CYCLES_PER_LINE;
        self.frame = input.u64()?;
        self.h_counter = input.u8()?;
        self.hint_pending = input.u8()? != 0;
        self.vint_pending = input.u8()? != 0;
        let has_dma = input.u8()? != 0;
        let dma = MemoryDmaState {
            source: input.u32()? & 0x00ff_ffff,
            remaining: input.u32()?,
            transferred: input.u32()?,
            cycle_fraction: input.u32()? & 0xffff,
        };
        self.memory_dma = if has_dma && dma.remaining != 0 {
            self.status |= 0x0002;
            Some(dma)
        } else {
            None
        };
        let has_internal_dma = input.u8()? != 0;
        let internal_kind = match input.u8()? {
            1 => Some(InternalDmaKind::Fill),
            2 => Some(InternalDmaKind::Copy),
            _ => None,
        };
        let internal_dma = InternalDmaState {
            kind: internal_kind.unwrap_or(InternalDmaKind::Fill),
            source: input.u16()?,
            fill: input.u8()?,
            remaining: input.u32()?,
            transferred: input.u32()?,
            slot_phase: input.u32()? % u32::from(CPU_CYCLES_PER_LINE),
        };
        if has_dma && has_internal_dma {
            return Err("Genesis VDP state cannot contain two simultaneous DMA engines".into());
        }
        self.internal_dma = if has_internal_dma && internal_dma.remaining != 0 {
            let Some(kind) = internal_kind else {
                return Err("Genesis VDP state has an invalid internal DMA kind".into());
            };
            self.status |= 0x0002;
            Some(InternalDmaState {
                kind,
                ..internal_dma
            })
        } else {
            None
        };
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
    fn sequential_reads_advance_before_prefetching_the_next_word() {
        let mut vdp = GenesisVdp::default();
        vdp.regs[15] = 2;
        vdp.vram[..4].copy_from_slice(&[0x11, 0x22, 0x33, 0x44]);
        vdp.write_control(0x0000);
        vdp.write_control(0x0000);
        assert_eq!(vdp.read_data(), 0x1122);
        assert_eq!(vdp.read_data(), 0x3344);
        assert_eq!(vdp.address, 4);
    }

    #[test]
    fn zero_increment_and_odd_address_word_writes_follow_vdp_addressing() {
        let mut vdp = GenesisVdp::default();
        vdp.regs[15] = 0;
        vdp.write_control(0x4000);
        vdp.write_control(0x0000);
        vdp.write_data(0x1234);
        vdp.write_data(0x5678);
        assert_eq!(&vdp.vram[..2], &[0x56, 0x78]);
        assert_eq!(vdp.address, 0);

        vdp.write_control(0x4001);
        vdp.write_control(0x0000);
        vdp.write_data(0x1234);
        assert_eq!(&vdp.vram[..2], &[0x34, 0x12]);
    }

    #[test]
    fn dma_copy_moves_bytes_and_updates_counters() {
        let mut vdp = GenesisVdp::default();
        vdp.regs[1] = 0x10;
        vdp.regs[15] = 1;
        vdp.regs[19] = 3;
        vdp.regs[21] = 0x10;
        vdp.regs[23] = 0xc0;
        vdp.vram[0x10..0x13].copy_from_slice(&[0x11, 0x22, 0x33]);

        vdp.write_control(0x0020);
        vdp.write_control(0x00c0);
        assert_ne!(vdp.status & 0x0002, 0);
        assert_eq!(&vdp.vram[0x20..0x23], &[0, 0, 0]);
        vdp.tick_cpu_cycles(u32::from(CPU_CYCLES_PER_LINE));
        assert_eq!(&vdp.vram[0x20..0x23], &[0x11, 0x22, 0x33]);
        assert_eq!(vdp.address, 0x23);
        assert_eq!(vdp.regs[21], 0x13);
        assert_eq!(vdp.regs[19], 0);
        assert_eq!(vdp.status & 0x0002, 0);
    }

    #[test]
    fn dma_fill_waits_for_data_word_and_repeats_high_byte() {
        let mut vdp = GenesisVdp::default();
        vdp.regs[1] = 0x10;
        vdp.regs[15] = 1;
        vdp.regs[19] = 3;
        vdp.regs[23] = 0x80;

        vdp.write_control(0x4000);
        vdp.write_control(0x0080);
        assert_ne!(vdp.status & 0x0002, 0);
        vdp.write_data(0xabcd);
        assert_eq!(&vdp.vram[..4], &[0xcd, 0xab, 0x00, 0x00]);
        assert_ne!(vdp.status & 0x0002, 0);
        vdp.tick_cpu_cycles(u32::from(CPU_CYCLES_PER_LINE));
        assert_eq!(&vdp.vram[..4], &[0xcd, 0xab, 0xab, 0xab]);
        assert_eq!(vdp.address, 4);
        assert_eq!(vdp.regs[19], 0);
        assert_eq!(vdp.status & 0x0002, 0);
    }

    #[test]
    fn memory_dma_request_uses_word_source_and_vdp_destination_semantics() {
        let mut vdp = GenesisVdp::default();
        vdp.regs[1] = 0x10;
        vdp.regs[15] = 2;
        vdp.regs[19] = 2;
        vdp.regs[21] = 0x00;
        vdp.regs[22] = 0x01;
        vdp.regs[23] = 0x00;

        vdp.write_control(0x4020);
        vdp.write_control(0x0080);
        assert_eq!(vdp.memory_dma_source(), Some(0x0200));
        assert!(vdp.memory_dma_pending());
        let first_cycles = vdp.service_memory_dma_word(0x1122).unwrap();
        assert!(first_cycles > 0);
        assert_eq!(vdp.memory_dma_source(), Some(0x0202));
        let second_cycles = vdp.service_memory_dma_word(0x3344).unwrap();
        assert!(second_cycles > 0);
        assert!(!vdp.memory_dma_pending());
        assert_eq!(&vdp.vram[0x20..0x24], &[0x11, 0x22, 0x33, 0x44]);
        assert_eq!(vdp.address, 0x24);
        assert_eq!(vdp.regs[21], 0x02);
        assert_eq!(vdp.regs[22], 0x01);
        assert_eq!(vdp.regs[19], 0);
        assert_eq!(vdp.status & 0x0002, 0);
    }

    #[test]
    fn memory_dma_bandwidth_changes_between_active_and_inactive_display() {
        let mut active = GenesisVdp::default();
        active.regs[1] = 0x50;
        active.regs[12] = 1;
        active.regs[15] = 2;
        active.regs[19] = 1;
        active.write_control(0x4000);
        active.write_control(0x0080);
        let active_cycles = active.service_memory_dma_word(0x1234).unwrap();

        let mut inactive = GenesisVdp::default();
        inactive.regs[1] = 0x10;
        inactive.regs[12] = 1;
        inactive.regs[15] = 2;
        inactive.regs[19] = 1;
        inactive.write_control(0x4000);
        inactive.write_control(0x0080);
        let inactive_cycles = inactive.service_memory_dma_word(0x1234).unwrap();

        assert_eq!(active_cycles, 54);
        assert_eq!(inactive_cycles, 4);
        assert!(inactive_cycles < active_cycles);
    }

    #[test]
    fn pending_memory_dma_survives_state_round_trip() {
        let mut vdp = GenesisVdp::default();
        vdp.regs[1] = 0x50;
        vdp.regs[12] = 1;
        vdp.regs[15] = 2;
        vdp.regs[19] = 3;
        vdp.regs[21] = 0x00;
        vdp.regs[22] = 0x01;
        vdp.write_control(0x4000);
        vdp.write_control(0x0080);
        let _ = vdp.service_memory_dma_word(0x1122).unwrap();
        let expected_dma = vdp.memory_dma.unwrap();

        let mut out = StateWriter::new(crate::platform::PlatformId::Genesis, 99);
        vdp.save(&mut out);
        let bytes = out.finish();
        let mut restored = GenesisVdp::default();
        let mut input = StateReader::new(&bytes, crate::platform::PlatformId::Genesis, 99).unwrap();
        restored.load(&mut input).unwrap();
        input.finish().unwrap();

        assert_eq!(restored.memory_dma, Some(expected_dma));
        assert_ne!(restored.status & 0x0002, 0);
        let _ = restored.service_memory_dma_word(0x3344).unwrap();
        let _ = restored.service_memory_dma_word(0x5566).unwrap();
        assert_eq!(&restored.vram[..6], &[0x11, 0x22, 0x33, 0x44, 0x55, 0x66]);
        assert!(!restored.memory_dma_pending());
    }

    #[test]
    fn pending_fill_survives_state_round_trip_without_extra_state_fields() {
        let mut vdp = GenesisVdp::default();
        vdp.regs[1] = 0x10;
        vdp.regs[15] = 1;
        vdp.regs[19] = 2;
        vdp.regs[23] = 0x80;
        vdp.write_control(0x4000);
        vdp.write_control(0x0080);
        assert_ne!(vdp.status & 0x0002, 0);

        let mut out = StateWriter::new(crate::platform::PlatformId::Genesis, 99);
        vdp.save(&mut out);
        let bytes = out.finish();
        let mut restored = GenesisVdp::default();
        let mut input = StateReader::new(&bytes, crate::platform::PlatformId::Genesis, 99).unwrap();
        restored.load(&mut input).unwrap();
        input.finish().unwrap();

        restored.write_data(0x5aa5);
        assert_eq!(&restored.vram[..3], &[0xa5, 0x5a, 0x00]);
        assert_ne!(restored.status & 0x0002, 0);
        restored.tick_cpu_cycles(u32::from(CPU_CYCLES_PER_LINE));
        assert_eq!(&restored.vram[..3], &[0xa5, 0x5a, 0x5a]);
        assert_eq!(restored.status & 0x0002, 0);
        assert_eq!(restored.regs[19], 0);
    }

    #[test]
    fn pal_and_ntsc_v_counters_follow_224_line_sequences() {
        let mut ntsc = GenesisVdp::new(false);
        ntsc.line = 234;
        assert_eq!((ntsc.read_hv_counter() >> 8) as u8, 0xea);
        ntsc.line = 235;
        assert_eq!((ntsc.read_hv_counter() >> 8) as u8, 0xe5);
        ntsc.line = 261;
        assert_eq!((ntsc.read_hv_counter() >> 8) as u8, 0xff);
        assert_eq!(ntsc.read_status() & 1, 0);

        let mut pal = GenesisVdp::new(true);
        assert_ne!(pal.read_status() & 1, 0);
        pal.line = 255;
        assert_eq!((pal.read_hv_counter() >> 8) as u8, 0xff);
        pal.line = 256;
        assert_eq!((pal.read_hv_counter() >> 8) as u8, 0x00);
        pal.line = 258;
        assert_eq!((pal.read_hv_counter() >> 8) as u8, 0x02);
        pal.line = 259;
        assert_eq!((pal.read_hv_counter() >> 8) as u8, 0xca);
        pal.line = 312;
        assert_eq!((pal.read_hv_counter() >> 8) as u8, 0xff);
        pal.cycle = CPU_CYCLES_PER_LINE - 1;
        pal.tick_cpu_cycles(1);
        assert_eq!(pal.line, 0);
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
    fn linked_sprite_order_wins_before_sprite_priority_bits() {
        let mut vdp = GenesisVdp::default();
        vdp.regs[1] = 0x40;
        vdp.regs[2] = 0x20;
        vdp.regs[4] = 0x07;
        vdp.regs[5] = 0x60;
        vdp.regs[12] = 1;
        vdp.cram[1] = 0x000e;
        vdp.cram[2] = 0x00e0;
        for row in 0..8usize {
            vdp.vram[32 + row * 4..32 + row * 4 + 4].fill(0x11);
            vdp.vram[64 + row * 4..64 + row * 4 + 4].fill(0x22);
        }
        let table = usize::from(vdp.regs[5] & 0x7e) << 9;
        vdp.vram[table..table + 2].copy_from_slice(&128u16.to_be_bytes());
        vdp.vram[table + 2..table + 4].copy_from_slice(&1u16.to_be_bytes());
        vdp.vram[table + 4..table + 6].copy_from_slice(&1u16.to_be_bytes());
        vdp.vram[table + 6..table + 8].copy_from_slice(&128u16.to_be_bytes());
        let second = table + 8;
        vdp.vram[second..second + 2].copy_from_slice(&128u16.to_be_bytes());
        vdp.vram[second + 2..second + 4].copy_from_slice(&0u16.to_be_bytes());
        vdp.vram[second + 4..second + 6].copy_from_slice(&0x8002u16.to_be_bytes());
        vdp.vram[second + 6..second + 8].copy_from_slice(&128u16.to_be_bytes());

        vdp.render_frame();
        assert_eq!(&vdp.video.pixels()[0..3], &[252, 0, 0]);
        assert_ne!(vdp.status & 0x0020, 0);
    }

    #[test]
    fn sprite_scanline_budget_sets_overflow_and_hides_excess_entries() {
        let mut vdp = GenesisVdp::default();
        vdp.regs[1] = 0x40;
        vdp.regs[2] = 0x20;
        vdp.regs[4] = 0x07;
        vdp.regs[5] = 0x60;
        vdp.regs[12] = 1;
        vdp.cram[1] = 0x000e;
        for row in 0..8usize {
            vdp.vram[32 + row * 4..32 + row * 4 + 4].fill(0x11);
        }
        let table = usize::from(vdp.regs[5] & 0x7e) << 9;
        for sprite in 0..21usize {
            let base = table + sprite * 8;
            let next = if sprite == 20 { 0 } else { sprite + 1 };
            vdp.vram[base..base + 2].copy_from_slice(&128u16.to_be_bytes());
            vdp.vram[base + 2..base + 4].copy_from_slice(&(next as u16).to_be_bytes());
            vdp.vram[base + 4..base + 6].copy_from_slice(&1u16.to_be_bytes());
            vdp.vram[base + 6..base + 8]
                .copy_from_slice(&(128u16 + sprite as u16 * 8).to_be_bytes());
        }

        vdp.render_frame();
        assert_ne!(vdp.status & 0x0040, 0);
        let twentieth = 19 * 8 * 4;
        assert_eq!(&vdp.video.pixels()[twentieth..twentieth + 3], &[252, 0, 0]);
        let excess = 20 * 8 * 4;
        assert_eq!(&vdp.video.pixels()[excess..excess + 3], &[0, 0, 0]);
    }

    #[test]
    fn vertical_scroll_can_change_for_each_two_cell_column() {
        let mut vdp = GenesisVdp::default();
        vdp.regs[1] = 0x40;
        vdp.regs[2] = 0x08;
        vdp.regs[4] = 0x07;
        vdp.regs[11] = 0x04;
        vdp.regs[12] = 1;
        vdp.regs[16] = 0;
        vdp.cram[1] = 0x000e;
        vdp.cram[2] = 0x00e0;
        for row in 0..8usize {
            vdp.vram[32 + row * 4..32 + row * 4 + 4].fill(0x11);
            vdp.vram[64 + row * 4..64 + row * 4 + 4].fill(0x22);
        }
        let plane_a = usize::from(vdp.regs[2] & 0x38) << 10;
        for column in 0..32usize {
            let top = plane_a + column * 2;
            let next = plane_a + (32 + column) * 2;
            vdp.vram[top..top + 2].copy_from_slice(&1u16.to_be_bytes());
            vdp.vram[next..next + 2].copy_from_slice(&2u16.to_be_bytes());
        }
        vdp.vsram[0] = 0;
        vdp.vsram[2] = 8;
        vdp.render_frame();

        assert_eq!(&vdp.video.pixels()[0..3], &[252, 0, 0]);
        let second_group = 16 * 4;
        assert_eq!(
            &vdp.video.pixels()[second_group..second_group + 3],
            &[0, 252, 0]
        );
    }

    #[test]
    fn window_plane_replaces_scroll_a_on_horizontal_and_vertical_sides() {
        let mut vdp = GenesisVdp::default();
        vdp.regs[1] = 0x40;
        vdp.regs[2] = 0x08;
        vdp.regs[3] = 0x0c;
        vdp.regs[4] = 0x07;
        vdp.regs[12] = 1;
        vdp.regs[16] = 0;
        vdp.cram[1] = 0x000e;
        vdp.cram[2] = 0x00e0;
        for row in 0..8usize {
            vdp.vram[32 + row * 4..32 + row * 4 + 4].fill(0x11);
            vdp.vram[64 + row * 4..64 + row * 4 + 4].fill(0x22);
        }
        let plane_a = usize::from(vdp.regs[2] & 0x38) << 10;
        for cell in 0..(32usize * 28) {
            let offset = plane_a + cell * 2;
            vdp.vram[offset..offset + 2].copy_from_slice(&1u16.to_be_bytes());
        }
        let window = usize::from(vdp.regs[3] & 0x3c) << 10;
        for cell in 0..64usize {
            let offset = window + cell * 2;
            vdp.vram[offset..offset + 2].copy_from_slice(&2u16.to_be_bytes());
        }

        vdp.regs[17] = 0x01;
        vdp.regs[18] = 0x00;
        vdp.render_frame();
        assert_eq!(&vdp.video.pixels()[0..3], &[0, 252, 0]);
        let scroll_a = 24 * 4;
        assert_eq!(&vdp.video.pixels()[scroll_a..scroll_a + 3], &[252, 0, 0]);

        vdp.regs[17] = 0x00;
        vdp.regs[18] = 0x01;
        vdp.render_frame();
        let top = 24 * 4;
        assert_eq!(&vdp.video.pixels()[top..top + 3], &[0, 252, 0]);
        let below = (16 * WIDTH as usize + 24) * 4;
        assert_eq!(&vdp.video.pixels()[below..below + 3], &[252, 0, 0]);
    }

    #[test]
    fn scanline_rendering_preserves_mid_frame_background_changes() {
        let mut vdp = GenesisVdp::default();
        vdp.cram[1] = 0x000e;
        vdp.cram[2] = 0x00e0;
        vdp.regs[7] = 1;
        vdp.tick_cpu_cycles(u32::from(CPU_CYCLES_PER_LINE));
        vdp.regs[7] = 2;
        vdp.tick_cpu_cycles(u32::from(CPU_CYCLES_PER_LINE));

        assert_eq!(&vdp.video.pixels()[0..3], &[252, 0, 0]);
        let second_line = WIDTH as usize * 4;
        assert_eq!(
            &vdp.video.pixels()[second_line..second_line + 3],
            &[0, 252, 0]
        );
    }

    #[test]
    fn sprite_dot_budget_can_overflow_before_sprite_count_limit() {
        let mut vdp = GenesisVdp::default();
        vdp.regs[1] = 0x40;
        vdp.regs[5] = 0x60;
        vdp.regs[12] = 0;
        vdp.cram[1] = 0x000e;
        let table = usize::from(vdp.regs[5] & 0x7f) << 9;
        for sprite in 0..9usize {
            let base = table + sprite * 8;
            let next = if sprite == 8 { 0 } else { sprite + 1 };
            vdp.vram[base..base + 2].copy_from_slice(&128u16.to_be_bytes());
            let size_link = 0x0c00u16 | next as u16;
            vdp.vram[base + 2..base + 4].copy_from_slice(&size_link.to_be_bytes());
            let tile = if sprite == 8 { 1 } else { 0 };
            vdp.vram[base + 4..base + 6].copy_from_slice(&(tile as u16).to_be_bytes());
            vdp.vram[base + 6..base + 8].copy_from_slice(&128u16.to_be_bytes());
        }
        for row in 0..8usize {
            vdp.vram[32 + row * 4..32 + row * 4 + 4].fill(0x11);
        }

        vdp.render_frame();
        assert_ne!(vdp.status & 0x0040, 0);
        assert_eq!(&vdp.video.pixels()[0..3], &[0, 0, 0]);
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
