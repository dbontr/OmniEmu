use crate::kernel::VideoBuffer;
use crate::state::{StateReader, StateWriter};

const WIDTH: u32 = 256;
const HEIGHT: u32 = 224;
const VRAM_BYTES: usize = 64 * 1024;
const OAM_BYTES: usize = 544;
const SCREEN_PIXELS: usize = WIDTH as usize * HEIGHT as usize;

#[derive(Clone, Copy)]
struct ScreenPixel {
    color: u16,
    layer: u8,
    math_allowed: bool,
    transparent: bool,
}

impl ScreenPixel {
    fn backdrop(color: u16) -> Self {
        Self {
            color,
            layer: 5,
            math_allowed: true,
            transparent: true,
        }
    }
}

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
    oam_reload_addr: u16,
    oam_latch: u8,
    bg_scroll_x: [u16; 4],
    bg_scroll_y: [u16; 4],
    bgofs_latch: u8,
    bghofs_latch: u8,
    mode7_latch: u8,
    mode7_scroll: [i16; 2],
    mode7_matrix: [i16; 4],
    mode7_center: [i16; 2],
    fixed_color: u16,
    main_screen: Vec<ScreenPixel>,
    sub_screen: Vec<ScreenPixel>,
    rendering_subscreen: bool,
    cycle: u16,
    scanline: u16,
    frame: u64,
    vblank: bool,
    nmi_pending: bool,
    range_over: bool,
    time_over: bool,
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
            oam_reload_addr: 0,
            oam_latch: 0,
            bg_scroll_x: [0; 4],
            bg_scroll_y: [0; 4],
            bgofs_latch: 0,
            bghofs_latch: 0,
            mode7_latch: 0,
            mode7_scroll: [0; 2],
            mode7_matrix: [0x0100, 0, 0, 0x0100],
            mode7_center: [0; 2],
            fixed_color: 0,
            main_screen: vec![ScreenPixel::backdrop(0); SCREEN_PIXELS],
            sub_screen: vec![ScreenPixel::backdrop(0); SCREEN_PIXELS],
            rendering_subscreen: false,
            cycle: 0,
            scanline: 0,
            frame: 0,
            vblank: false,
            nmi_pending: false,
            range_over: false,
            time_over: false,
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

    pub fn scanline(&self) -> u16 {
        self.scanline
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

    fn reload_oam_address(&mut self) {
        let word_address = u16::from(self.regs[0x02]) | (u16::from(self.regs[0x03] & 1) << 8);
        self.oam_reload_addr = (word_address << 1) % OAM_BYTES as u16;
        self.oam_addr = self.oam_reload_addr;
    }

    fn write_oam_data(&mut self, value: u8) {
        let index = usize::from(self.oam_addr) % OAM_BYTES;
        if index < 512 {
            if index & 1 == 0 {
                self.oam_latch = value;
            } else {
                self.oam[index - 1] = self.oam_latch;
                self.oam[index] = value;
            }
        } else {
            self.oam[index] = value;
        }
        self.oam_addr = self.oam_addr.wrapping_add(1) % OAM_BYTES as u16;
    }

    fn sign_extend_13(value: u16) -> i16 {
        ((value << 3) as i16) >> 3
    }

    fn write_mode7_word(&mut self, value: u8) -> u16 {
        let word = (u16::from(value) << 8) | u16::from(self.mode7_latch);
        self.mode7_latch = value;
        word
    }

    fn write_mode7_register(&mut self, address: u16, value: u8) {
        let word = self.write_mode7_word(value);
        match address {
            0x210d => self.mode7_scroll[0] = Self::sign_extend_13(word),
            0x210e => self.mode7_scroll[1] = Self::sign_extend_13(word),
            0x211b => self.mode7_matrix[0] = word as i16,
            0x211c => self.mode7_matrix[1] = word as i16,
            0x211d => self.mode7_matrix[2] = word as i16,
            0x211e => self.mode7_matrix[3] = word as i16,
            0x211f => self.mode7_center[0] = Self::sign_extend_13(word),
            0x2120 => self.mode7_center[1] = Self::sign_extend_13(word),
            _ => {}
        }
    }

    fn mode7_product(&self) -> u32 {
        let factor_b = (self.mode7_matrix[1] >> 8) as i8 as i32;
        let product = i32::from(self.mode7_matrix[0]) * factor_b;
        product as u32 & 0x00ff_ffff
    }

    fn write_bg_scroll(&mut self, address: u16, value: u8) {
        let (background, horizontal) = match address {
            0x210d => (0, true),
            0x210e => (0, false),
            0x210f => (1, true),
            0x2110 => (1, false),
            0x2111 => (2, true),
            0x2112 => (2, false),
            0x2113 => (3, true),
            0x2114 => (3, false),
            _ => return,
        };
        if horizontal {
            self.bg_scroll_x[background] = ((u16::from(value) << 8)
                | (u16::from(self.bgofs_latch) & !7)
                | (u16::from(self.bghofs_latch) & 7))
                & 0x03ff;
            self.bgofs_latch = value;
            self.bghofs_latch = value;
        } else {
            self.bg_scroll_y[background] =
                ((u16::from(value) << 8) | u16::from(self.bgofs_latch)) & 0x03ff;
            self.bgofs_latch = value;
        }
    }

    pub fn write(&mut self, address: u16, value: u8) {
        let reg = (address & 0x3f) as usize;
        if reg < self.regs.len() {
            self.regs[reg] = value;
        }
        match address {
            0x2102 | 0x2103 => self.reload_oam_address(),
            0x2104 => self.write_oam_data(value),
            0x210d | 0x210e => {
                self.write_bg_scroll(address, value);
                self.write_mode7_register(address, value);
            }
            0x210f..=0x2114 => self.write_bg_scroll(address, value),
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
            0x211b..=0x2120 => self.write_mode7_register(address, value),
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
            0x2132 => {
                let color = u16::from(value & 0x1f);
                if value & 0x20 != 0 {
                    self.fixed_color = (self.fixed_color & !0x001f) | color;
                }
                if value & 0x40 != 0 {
                    self.fixed_color = (self.fixed_color & !0x03e0) | (color << 5);
                }
                if value & 0x80 != 0 {
                    self.fixed_color = (self.fixed_color & !0x7c00) | (color << 10);
                }
            }
            _ => {}
        }
    }

    pub fn read(&mut self, address: u16) -> u8 {
        match address {
            0x2134 => self.mode7_product() as u8,
            0x2135 => (self.mode7_product() >> 8) as u8,
            0x2136 => (self.mode7_product() >> 16) as u8,
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
            0x213e => (u8::from(self.time_over) << 7) | (u8::from(self.range_over) << 6) | 0x01,
            0x213f => u8::from(self.vblank) << 7,
            _ => 0,
        }
    }

    pub fn tick_dots(&mut self, dots: u32) {
        for _ in 0..dots {
            self.cycle += 1;
            if self.cycle >= 341 {
                self.cycle = 0;
                let completed_scanline = self.scanline;
                if (1..=HEIGHT as u16).contains(&completed_scanline) {
                    let y = usize::from(completed_scanline - 1);
                    let (_, range_over, time_over) = self.obj_scanline_sliver_masks(y);
                    self.range_over |= range_over;
                    self.time_over |= time_over;
                    self.render_scanline(y);
                }
                self.scanline += 1;
                if self.scanline == 225 {
                    self.vblank = true;
                    if self.regs[0] & 0x80 == 0 {
                        self.oam_addr = self.oam_reload_addr;
                    }
                    self.frame = self.frame.wrapping_add(1);
                    self.nmi_pending = true;
                } else if self.scanline >= 262 {
                    self.scanline = 0;
                    self.vblank = false;
                    self.range_over = false;
                    self.time_over = false;
                    self.reset_frame_surface();
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
    fn high_resolution_output(&self) -> bool {
        matches!(self.regs[0x05] & 7, 5 | 6) || self.regs[0x33] & 0x08 != 0
    }

    fn reset_frame_surface(&mut self) {
        self.video.resize(WIDTH, HEIGHT);
        self.video.clear([0, 0, 0, 255]);
    }

    fn promote_video_to_hires(&mut self, completed_rows: usize) {
        if self.video.width() == 512 {
            return;
        }
        self.video.resize(512, HEIGHT);
        let source_stride = WIDTH as usize * 4;
        let target_stride = 512usize * 4;
        let pixels = self.video.pixels_mut();
        for y in (0..completed_rows).rev() {
            let source = y * source_stride;
            let target = y * target_stride;
            pixels.copy_within(source..source + source_stride, target);
            for x in (0..WIDTH as usize).rev() {
                let source_pixel = target + x * 4;
                let rgba = [
                    pixels[source_pixel],
                    pixels[source_pixel + 1],
                    pixels[source_pixel + 2],
                    pixels[source_pixel + 3],
                ];
                let output = target + x * 8;
                pixels[output..output + 4].copy_from_slice(&rgba);
                pixels[output + 4..output + 8].copy_from_slice(&rgba);
            }
        }
    }

    fn render_scanline(&mut self, y: usize) {
        if y >= HEIGHT as usize {
            return;
        }
        let high_resolution = self.high_resolution_output();
        if high_resolution {
            self.promote_video_to_hires(y);
        }
        let start = y * WIDTH as usize;
        let end = start + WIDTH as usize;
        self.main_screen[start..end].fill(ScreenPixel::backdrop(self.cgram[0]));
        self.sub_screen[start..end].fill(ScreenPixel::backdrop(self.fixed_color));

        if self.regs[0] & 0x80 == 0 {
            self.rendering_subscreen = false;
            self.render_active_mode(y);
            self.rendering_subscreen = true;
            self.render_active_mode(y);
            self.rendering_subscreen = false;
        }

        for x in 0..WIDTH as usize {
            let index = start + x;
            let main = if self.regs[0] & 0x80 != 0 {
                0
            } else {
                self.compose_pixel(index, x)
            };
            if self.video.width() == 512 {
                if high_resolution {
                    let sub = if self.regs[0] & 0x80 != 0 {
                        0
                    } else {
                        self.sub_screen[index].color
                    };
                    self.write_video_color(x * 2, y, sub);
                    self.write_video_color(x * 2 + 1, y, main);
                } else {
                    self.write_video_color(x * 2, y, main);
                    self.write_video_color(x * 2 + 1, y, main);
                }
            } else {
                self.write_video_color(x, y, main);
            }
        }
    }

    fn render_frame(&mut self) {
        self.reset_frame_surface();
        for y in 0..HEIGHT as usize {
            self.render_scanline(y);
        }
    }

    fn render_active_mode(&mut self, y: usize) {
        match self.regs[0x05] & 0x07 {
            0 => self.render_mode0(y),
            1 => self.render_mode1(y),
            2..=5 => self.render_mode2_to_4(y),
            6 => self.render_mode6(y),
            _ => self.render_mode7(y),
        }
    }

    fn direct_color(pixel: u8, palette: u8) -> u16 {
        let red = (u16::from(pixel & 0x07) << 2) | (u16::from(palette & 0x01) << 1);
        let green = (u16::from((pixel >> 3) & 0x07) << 2) | (u16::from((palette >> 1) & 0x01) << 1);
        let blue = (u16::from((pixel >> 6) & 0x03) << 3) | (u16::from((palette >> 2) & 0x01) << 2);
        red | (green << 5) | (blue << 10)
    }

    fn mode7_sample(&self, screen_x: usize, screen_y: usize) -> Option<u8> {
        let settings = self.regs[0x1a];
        let sx = if settings & 0x01 != 0 {
            255 - screen_x as i32
        } else {
            screen_x as i32
        };
        let sy = if settings & 0x02 != 0 {
            255 - screen_y as i32
        } else {
            screen_y as i32
        };
        let x_center = i32::from(self.mode7_center[0]);
        let y_center = i32::from(self.mode7_center[1]);
        let dx = sx + i32::from(self.mode7_scroll[0]) - x_center;
        let dy = sy + i32::from(self.mode7_scroll[1]) - y_center;
        let tx = (i32::from(self.mode7_matrix[0]) * dx
            + i32::from(self.mode7_matrix[1]) * dy
            + (x_center << 8))
            >> 8;
        let ty = (i32::from(self.mode7_matrix[2]) * dx
            + i32::from(self.mode7_matrix[3]) * dy
            + (y_center << 8))
            >> 8;
        let outside = !(0..1024).contains(&tx) || !(0..1024).contains(&ty);
        let repeat_disabled = settings & 0x80 != 0;
        if outside && repeat_disabled && settings & 0x40 == 0 {
            return None;
        }

        let pixel_x = (tx & 7) as usize;
        let pixel_y = (ty & 7) as usize;
        let tile = if outside && repeat_disabled {
            0usize
        } else {
            let wrapped_x = (tx & 0x03ff) as usize;
            let wrapped_y = (ty & 0x03ff) as usize;
            let map_word = (wrapped_y / 8) * 128 + wrapped_x / 8;
            usize::from(self.vram[(map_word * 2) & 0xffff])
        };
        let pixel_word = tile * 64 + pixel_y * 8 + pixel_x;
        Some(self.vram[((pixel_word * 2) + 1) & 0xffff])
    }

    fn read_vram16(&self, address: usize) -> u16 {
        let lo = self.vram[address & 0xffff];
        let hi = self.vram[(address + 1) & 0xffff];
        u16::from_le_bytes([lo, hi])
    }

    fn bg_bpp(&self, background: usize) -> Option<usize> {
        match self.regs[0x05] & 7 {
            0 => Some(2),
            1 => match background {
                0 | 1 => Some(4),
                2 => Some(2),
                _ => None,
            },
            2 => (background < 2).then_some(4),
            3 => match background {
                0 => Some(8),
                1 => Some(4),
                _ => None,
            },
            4 => match background {
                0 => Some(8),
                1 => Some(2),
                _ => None,
            },
            5 => match background {
                0 => Some(4),
                1 => Some(2),
                _ => None,
            },
            6 => (background == 0).then_some(4),
            _ => None,
        }
    }

    fn bg_tile_base(&self, background: usize) -> usize {
        let register = self.regs[0x0b + background / 2];
        let nibble = if background & 1 == 0 {
            register & 0x0f
        } else {
            register >> 4
        };
        (usize::from(nibble) << 13) & 0xffff
    }

    fn tile_pixel(&self, base: usize, bpp: usize, x: usize, y: usize) -> u8 {
        let bit = 7 - (x & 7);
        let mut color = 0u8;
        for plane in 0..bpp {
            let group = plane / 2;
            let byte = self.vram[(base + group * 16 + (y & 7) * 2 + plane % 2) & 0xffff];
            color |= ((byte >> bit) & 1) << plane;
        }
        color
    }

    fn mosaic_coordinates(&self, background: usize, x: usize, y: usize) -> (usize, usize) {
        let mosaic = self.regs[0x06];
        if mosaic & (1 << background) == 0 {
            return (x, y);
        }
        let size = usize::from(mosaic >> 4) + 1;
        (x / size * size, y / size * size)
    }

    fn window_mask(&self, layer: usize, x: usize) -> bool {
        let (select, logic) = match layer {
            0 => (self.regs[0x23] & 0x0f, self.regs[0x2a] & 0x03),
            1 => (self.regs[0x23] >> 4, (self.regs[0x2a] >> 2) & 0x03),
            2 => (self.regs[0x24] & 0x0f, (self.regs[0x2a] >> 4) & 0x03),
            3 => (self.regs[0x24] >> 4, (self.regs[0x2a] >> 6) & 0x03),
            4 => (self.regs[0x25] & 0x0f, self.regs[0x2b] & 0x03),
            5 => (self.regs[0x25] >> 4, (self.regs[0x2b] >> 2) & 0x03),
            _ => return false,
        };
        let eval = |left: u8, right: u8, enabled: bool, inverted: bool| {
            if !enabled {
                return false;
            }
            let inside = usize::from(left) <= x && x <= usize::from(right);
            inside ^ inverted
        };
        let window1 = eval(
            self.regs[0x26],
            self.regs[0x27],
            select & 0x02 != 0,
            select & 0x01 != 0,
        );
        let window2 = eval(
            self.regs[0x28],
            self.regs[0x29],
            select & 0x08 != 0,
            select & 0x04 != 0,
        );
        match (select & 0x02 != 0, select & 0x08 != 0) {
            (false, false) => false,
            (true, false) => window1,
            (false, true) => window2,
            (true, true) => match logic {
                0 => window1 || window2,
                1 => window1 && window2,
                2 => window1 ^ window2,
                _ => window1 == window2,
            },
        }
    }

    fn screen_layer_enabled(&self, layer: usize) -> bool {
        let register = if self.rendering_subscreen { 0x2d } else { 0x2c };
        self.regs[register] & (1 << layer) != 0
    }

    fn screen_window_masks_layer(&self, layer: usize, x: usize) -> bool {
        let register = if self.rendering_subscreen { 0x2f } else { 0x2e };
        self.regs[register] & (1 << layer) != 0 && self.window_mask(layer, x)
    }

    fn color_window_region_matches(&self, region: u8, x: usize) -> bool {
        let inside = self.window_mask(5, x);
        match region & 3 {
            0 => false,
            1 => !inside,
            2 => inside,
            _ => true,
        }
    }

    fn blend_color(&self, main: u16, addend: u16, allow_half: bool) -> u16 {
        let subtract = self.regs[0x31] & 0x80 != 0;
        let half = allow_half && self.regs[0x31] & 0x40 != 0;
        let blend = |main: u16, addend: u16| {
            let value = if subtract {
                i32::from(main) - i32::from(addend)
            } else {
                i32::from(main) + i32::from(addend)
            };
            let value = if half { value / 2 } else { value };
            value.clamp(0, 31) as u16
        };
        let red = blend(main & 0x1f, addend & 0x1f);
        let green = blend((main >> 5) & 0x1f, (addend >> 5) & 0x1f);
        let blue = blend((main >> 10) & 0x1f, (addend >> 10) & 0x1f);
        red | (green << 5) | (blue << 10)
    }

    #[cfg(test)]
    fn apply_fixed_color_math(
        &self,
        color: u16,
        x: usize,
        layer: usize,
        math_allowed: bool,
    ) -> u16 {
        if !math_allowed
            || self.regs[0x30] & 0x02 != 0
            || self.regs[0x31] & (1 << layer) == 0
            || self.color_window_region_matches((self.regs[0x30] >> 4) & 3, x)
        {
            color
        } else {
            self.blend_color(color, self.fixed_color, true)
        }
    }

    fn compose_pixel(&self, index: usize, x: usize) -> u16 {
        let main = self.main_screen[index];
        let main_black_region = (self.regs[0x30] >> 6) & 3;
        let main_blacked = self.color_window_region_matches(main_black_region, x);
        let color = if main_blacked { 0 } else { main.color };
        let math_mask_region = (self.regs[0x30] >> 4) & 3;
        if !main.math_allowed
            || self.regs[0x31] & (1 << main.layer) == 0
            || self.color_window_region_matches(math_mask_region, x)
        {
            return color;
        }
        let use_subscreen = self.regs[0x30] & 0x02 != 0;
        let sub = self.sub_screen[index];
        let addend = if use_subscreen {
            sub.color
        } else {
            self.fixed_color
        };
        let allow_half = !main_blacked && !(use_subscreen && sub.transparent);
        self.blend_color(color, addend, allow_half)
    }

    fn tilemap_entry(&self, screen: u8, map_x: usize, map_y: usize) -> u16 {
        let map_width = if screen & 1 != 0 { 64usize } else { 32usize };
        let map_height = if screen & 2 != 0 { 64usize } else { 32usize };
        let map_x = map_x % map_width;
        let map_y = map_y % map_height;
        let screen_x = map_x / 32;
        let screen_y = map_y / 32;
        let screens_per_row = if map_width == 64 { 2 } else { 1 };
        let screen_index = screen_y * screens_per_row + screen_x;
        let map_base = (usize::from(screen & 0xfc) << 9) & 0xffff;
        let address = map_base + screen_index * 0x800 + (((map_y & 31) * 32 + (map_x & 31)) * 2);
        self.read_vram16(address)
    }

    fn offset_map_entry(&self, column: usize, vertical: bool) -> u16 {
        let screen = self.regs[0x09];
        let map_width = if screen & 1 != 0 { 64usize } else { 32usize };
        let map_height = if screen & 2 != 0 { 64usize } else { 32usize };
        let map_x = (column + usize::from(self.bg_scroll_x[2] >> 3)) % map_width;
        let map_y = (usize::from(self.bg_scroll_y[2] >> 3) + usize::from(vertical)) % map_height;
        self.tilemap_entry(screen, map_x, map_y)
    }

    fn offset_scroll(&self, background: usize, x: usize) -> (usize, usize) {
        let mode = self.regs[0x05] & 7;
        let high_resolution = mode == 6;
        let mut horizontal =
            usize::from(self.bg_scroll_x[background]) * if high_resolution { 2 } else { 1 };
        let mut vertical = usize::from(self.bg_scroll_y[background]);
        if !matches!(mode, 2 | 4 | 6) || background > 1 || (mode == 6 && background != 0) {
            return (horizontal, vertical);
        }
        let visible_column = x / 8;
        if visible_column == 0 {
            return (horizontal, vertical);
        }
        let enable = if background == 0 { 0x2000 } else { 0x4000 };
        let horizontal_entry = self.offset_map_entry(visible_column - 1, false);
        if horizontal_entry & enable != 0 {
            if mode == 4 && horizontal_entry & 0x8000 != 0 {
                vertical = usize::from(horizontal_entry & 0x03ff);
            } else {
                horizontal = (horizontal & 7) | usize::from(horizontal_entry & 0x03f8);
            }
        }
        if mode != 4 {
            let vertical_entry = self.offset_map_entry(visible_column - 1, true);
            if vertical_entry & enable != 0 {
                vertical = usize::from(vertical_entry & 0x03ff);
            }
        }
        (horizontal, vertical)
    }

    fn bg_pixel(&self, background: usize, x: usize, y: usize) -> Option<(u16, bool)> {
        let (x, y) = self.mosaic_coordinates(background, x, y);
        let mode = self.regs[0x05] & 7;
        let bpp = self.bg_bpp(background)?;
        let screen = self.regs[0x07 + background];
        let high_resolution = matches!(mode, 5 | 6);
        let large_tile = self.regs[0x05] & (0x10 << background) != 0;
        let tile_width = if high_resolution || large_tile {
            16usize
        } else {
            8usize
        };
        let tile_height = if large_tile { 16usize } else { 8usize };
        let map_width = if screen & 1 != 0 { 64usize } else { 32usize };
        let map_height = if screen & 2 != 0 { 64usize } else { 32usize };
        let source_x = if high_resolution {
            x * 2 + usize::from(!self.rendering_subscreen)
        } else {
            x
        };
        let (scroll_x, scroll_y) = self.offset_scroll(background, x);
        let world_x = (source_x + scroll_x) % (map_width * tile_width);
        let world_y = (y + 1 + scroll_y) % (map_height * tile_height);
        let map_x = world_x / tile_width;
        let map_y = world_y / tile_height;
        let entry = self.tilemap_entry(screen, map_x, map_y);
        let priority = entry & 0x2000 != 0;
        let palette = usize::from((entry >> 10) & 7);
        let mut pixel_x = world_x % tile_width;
        let mut pixel_y = world_y % tile_height;
        if entry & 0x4000 != 0 {
            pixel_x = tile_width - 1 - pixel_x;
        }
        if entry & 0x8000 != 0 {
            pixel_y = tile_height - 1 - pixel_y;
        }
        let tile = (usize::from(entry & 0x03ff) + pixel_x / 8 + (pixel_y / 8) * 16) & 0x03ff;
        let tile_base = (self.bg_tile_base(background) + tile * bpp * 8) & 0xffff;
        let color = self.tile_pixel(tile_base, bpp, pixel_x, pixel_y);
        if color == 0 {
            return None;
        }
        let output_color = if bpp == 8
            && background == 0
            && matches!(mode, 3 | 4)
            && self.regs[0x30] & 0x01 != 0
        {
            Self::direct_color(color, palette as u8)
        } else {
            let cgram_index = if bpp == 8 {
                usize::from(color)
            } else if mode == 0 {
                background * 32 + palette * (1usize << bpp) + usize::from(color)
            } else {
                palette * (1usize << bpp) + usize::from(color)
            };
            self.cgram[cgram_index & 0xff]
        };
        Some((output_color, priority))
    }

    fn write_video_color(&mut self, x: usize, y: usize, color: u16) {
        let rgba = self.color_rgba(color);
        let width = self.video.width() as usize;
        let offset = (y * width + x) * 4;
        self.video.pixels_mut()[offset..offset + 4].copy_from_slice(&rgba);
    }

    fn write_layer_color(
        &mut self,
        x: usize,
        y: usize,
        color: u16,
        layer: usize,
        math_allowed: bool,
    ) {
        let index = y * WIDTH as usize + x;
        let pixel = ScreenPixel {
            color,
            layer: layer as u8,
            math_allowed,
            transparent: false,
        };
        if self.rendering_subscreen {
            self.sub_screen[index] = pixel;
        } else {
            self.main_screen[index] = pixel;
        }
    }

    fn write_layer_pixel(
        &mut self,
        x: usize,
        y: usize,
        cgram_index: u8,
        layer: usize,
        math_allowed: bool,
    ) {
        self.write_layer_color(
            x,
            y,
            self.cgram[usize::from(cgram_index)],
            layer,
            math_allowed,
        );
    }

    fn draw_bg_priority(&mut self, background: usize, high: bool, y: usize) {
        if !self.screen_layer_enabled(background) || self.bg_bpp(background).is_none() {
            return;
        }
        for x in 0..WIDTH as usize {
            if self.screen_window_masks_layer(background, x) {
                continue;
            }
            if let Some((color, priority)) = self.bg_pixel(background, x, y) {
                if priority == high {
                    self.write_layer_color(x, y, color, background, true);
                }
            }
        }
    }

    fn obj_size_pair(&self) -> ((usize, usize), (usize, usize)) {
        match (self.regs[0x01] >> 5) & 7 {
            0 => ((8, 8), (16, 16)),
            1 => ((8, 8), (32, 32)),
            2 => ((8, 8), (64, 64)),
            3 => ((16, 16), (32, 32)),
            4 => ((16, 16), (64, 64)),
            5 => ((32, 32), (64, 64)),
            6 => ((16, 32), (32, 64)),
            _ => ((16, 32), (32, 32)),
        }
    }

    fn first_sprite(&self) -> usize {
        if self.regs[0x03] & 0x80 != 0 {
            usize::from(self.oam_reload_addr / 4) & 0x7f
        } else {
            0
        }
    }

    fn obj_geometry(&self, sprite: usize) -> (i16, usize, usize, usize) {
        let base = sprite * 4;
        let high = self.oam[512 + sprite / 4];
        let packed = (high >> ((sprite & 3) * 2)) & 3;
        let x = i16::from(self.oam[base]) - if packed & 1 != 0 { 256 } else { 0 };
        let y = usize::from(self.oam[base + 1]);
        let (small, large) = self.obj_size_pair();
        let (width, height) = if packed & 2 != 0 { large } else { small };
        (x, y, width, height)
    }

    fn obj_scanline_sliver_masks(&self, screen_y: usize) -> ([u8; 128], bool, bool) {
        let first = self.first_sprite();
        let mut selected = [0usize; 32];
        let mut selected_count = 0usize;
        let mut sprite_overflow = false;

        for position in 0..128usize {
            let sprite = (first + position) & 0x7f;
            let (x, y, width, height) = self.obj_geometry(sprite);
            let dy = screen_y.wrapping_sub(y) & 0xff;
            if dy >= height {
                continue;
            }
            let horizontally_counted = x == -256 || (x < WIDTH as i16 && x + width as i16 > 0);
            if !horizontally_counted {
                continue;
            }
            if selected_count == selected.len() {
                sprite_overflow = true;
                break;
            }
            selected[selected_count] = sprite;
            selected_count += 1;
        }

        let mut masks = [0u8; 128];
        let mut sliver_count = 0usize;
        let mut sliver_overflow = false;
        'sprites: for selected_index in (0..selected_count).rev() {
            let sprite = selected[selected_index];
            let (x, _, width, _) = self.obj_geometry(sprite);
            for sliver in 0..width / 8 {
                let sliver_x = x + (sliver * 8) as i16;
                let counted = x == -256 || (sliver_x < WIDTH as i16 && sliver_x + 8 > 0);
                if !counted {
                    continue;
                }
                if sliver_count == 34 {
                    sliver_overflow = true;
                    break 'sprites;
                }
                masks[sprite] |= 1 << sliver;
                sliver_count += 1;
            }
        }
        (masks, sprite_overflow, sliver_overflow)
    }

    fn draw_obj_priority(&mut self, priority: u8, screen_y: usize) {
        if !self.screen_layer_enabled(4) {
            return;
        }
        let first = self.first_sprite();
        let name_base = usize::from(self.regs[0x01] & 7) << 14;
        let name_select = (usize::from((self.regs[0x01] >> 3) & 3) + 1) << 13;
        let (sliver_masks, _, _) = self.obj_scanline_sliver_masks(screen_y);

        for position in (0..128usize).rev() {
            let sprite = (first + position) & 0x7f;
            let sliver_mask = sliver_masks[sprite];
            if sliver_mask == 0 {
                continue;
            }
            let base = sprite * 4;
            let attributes = self.oam[base + 3];
            if ((attributes >> 4) & 3) != priority {
                continue;
            }
            let (x, y, width, height) = self.obj_geometry(sprite);
            let dy = screen_y.wrapping_sub(y) & 0xff;
            if dy >= height {
                continue;
            }
            let tile_page = if attributes & 1 != 0 {
                name_base + name_select
            } else {
                name_base
            };
            let palette = usize::from((attributes >> 1) & 7);
            let hflip = attributes & 0x40 != 0;
            let vflip = attributes & 0x80 != 0;
            let source_y = if vflip { height - 1 - dy } else { dy };
            let tile_y = source_y / 8;
            let pixel_y = source_y & 7;

            for sliver in 0..width / 8 {
                if sliver_mask & (1 << sliver) == 0 {
                    continue;
                }
                for sliver_x in 0..8usize {
                    let dx = sliver * 8 + sliver_x;
                    let screen_x = x + dx as i16;
                    if !(0..WIDTH as i16).contains(&screen_x) {
                        continue;
                    }
                    if self.screen_window_masks_layer(4, screen_x as usize) {
                        continue;
                    }
                    let source_x = if hflip { width - 1 - dx } else { dx };
                    let tile_x = source_x / 8;
                    let pixel_x = source_x & 7;
                    let tile = (usize::from(self.oam[base + 2]) + tile_x + tile_y * 16) & 0xff;
                    let tile_base = (tile_page + tile * 32) & 0xffff;
                    let color = self.tile_pixel(tile_base, 4, pixel_x, pixel_y);
                    if color == 0 {
                        continue;
                    }
                    let cgram = 128 + palette * 16 + usize::from(color);
                    self.write_layer_pixel(
                        screen_x as usize,
                        screen_y,
                        cgram as u8,
                        4,
                        palette >= 4,
                    );
                }
            }
        }
    }

    fn render_mode0(&mut self, y: usize) {
        self.draw_bg_priority(3, false, y);
        self.draw_bg_priority(2, false, y);
        self.draw_obj_priority(0, y);
        self.draw_bg_priority(3, true, y);
        self.draw_bg_priority(2, true, y);
        self.draw_obj_priority(1, y);
        self.draw_bg_priority(1, false, y);
        self.draw_bg_priority(0, false, y);
        self.draw_obj_priority(2, y);
        self.draw_bg_priority(1, true, y);
        self.draw_bg_priority(0, true, y);
        self.draw_obj_priority(3, y);
    }

    fn render_mode1(&mut self, y: usize) {
        self.draw_bg_priority(2, false, y);
        self.draw_obj_priority(0, y);
        if self.regs[0x05] & 0x08 == 0 {
            self.draw_bg_priority(2, true, y);
        }
        self.draw_obj_priority(1, y);
        self.draw_bg_priority(1, false, y);
        self.draw_bg_priority(0, false, y);
        self.draw_obj_priority(2, y);
        self.draw_bg_priority(1, true, y);
        self.draw_bg_priority(0, true, y);
        self.draw_obj_priority(3, y);
        if self.regs[0x05] & 0x08 != 0 {
            self.draw_bg_priority(2, true, y);
        }
    }

    fn render_mode2_to_4(&mut self, y: usize) {
        self.draw_bg_priority(1, false, y);
        self.draw_obj_priority(0, y);
        self.draw_bg_priority(0, false, y);
        self.draw_obj_priority(1, y);
        self.draw_bg_priority(1, true, y);
        self.draw_obj_priority(2, y);
        self.draw_bg_priority(0, true, y);
        self.draw_obj_priority(3, y);
    }

    fn render_mode6(&mut self, y: usize) {
        self.draw_obj_priority(0, y);
        self.draw_bg_priority(0, false, y);
        self.draw_obj_priority(1, y);
        self.draw_obj_priority(2, y);
        self.draw_bg_priority(0, true, y);
        self.draw_obj_priority(3, y);
    }

    fn draw_mode7_bg1(&mut self, y: usize) {
        if !self.screen_layer_enabled(0) {
            return;
        }
        let direct_color = self.regs[0x30] & 0x01 != 0;
        for x in 0..WIDTH as usize {
            if self.screen_window_masks_layer(0, x) {
                continue;
            }
            let (sample_x, sample_y) = self.mosaic_coordinates(0, x, y);
            let Some(pixel) = self.mode7_sample(sample_x, sample_y) else {
                continue;
            };
            if pixel == 0 {
                continue;
            }
            if direct_color {
                self.write_layer_color(x, y, Self::direct_color(pixel, 0), 0, true);
            } else {
                self.write_layer_pixel(x, y, pixel, 0, true);
            }
        }
    }

    fn draw_mode7_extbg(&mut self, high: bool, y: usize) {
        if self.regs[0x33] & 0x40 == 0 || !self.screen_layer_enabled(1) {
            return;
        }
        for x in 0..WIDTH as usize {
            if self.screen_window_masks_layer(1, x) {
                continue;
            }
            let (sample_x, sample_y) = self.mosaic_coordinates(1, x, y);
            let Some(pixel) = self.mode7_sample(sample_x, sample_y) else {
                continue;
            };
            if pixel == 0 || (pixel & 0x80 != 0) != high {
                continue;
            }
            self.write_layer_pixel(x, y, pixel & 0x7f, 1, true);
        }
    }

    fn render_mode7(&mut self, y: usize) {
        self.draw_mode7_extbg(false, y);
        self.draw_obj_priority(0, y);
        self.draw_mode7_bg1(y);
        self.draw_obj_priority(1, y);
        self.draw_mode7_extbg(true, y);
        self.draw_obj_priority(2, y);
        self.draw_obj_priority(3, y);
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
        out.u16(self.oam_reload_addr);
        out.u8(self.oam_latch);
        for value in self.bg_scroll_x {
            out.u16(value);
        }
        for value in self.bg_scroll_y {
            out.u16(value);
        }
        out.u8(self.bgofs_latch);
        out.u8(self.bghofs_latch);
        out.u8(self.mode7_latch);
        for value in self.mode7_scroll {
            out.u16(value as u16);
        }
        for value in self.mode7_matrix {
            out.u16(value as u16);
        }
        for value in self.mode7_center {
            out.u16(value as u16);
        }
        out.u16(self.fixed_color);
        out.u16(self.cycle);
        out.u16(self.scanline);
        out.u64(self.frame);
        out.u8(self.vblank as u8);
        out.u8(self.nmi_pending as u8);
        out.u8(self.range_over as u8);
        out.u8(self.time_over as u8);
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
        self.oam_reload_addr = input.u16()?;
        self.oam_latch = input.u8()?;
        for value in &mut self.bg_scroll_x {
            *value = input.u16()? & 0x03ff;
        }
        for value in &mut self.bg_scroll_y {
            *value = input.u16()? & 0x03ff;
        }
        self.bgofs_latch = input.u8()?;
        self.bghofs_latch = input.u8()?;
        self.mode7_latch = input.u8()?;
        for value in &mut self.mode7_scroll {
            *value = input.u16()? as i16;
        }
        for value in &mut self.mode7_matrix {
            *value = input.u16()? as i16;
        }
        for value in &mut self.mode7_center {
            *value = input.u16()? as i16;
        }
        self.fixed_color = input.u16()? & 0x7fff;
        self.cycle = input.u16()?;
        self.scanline = input.u16()?;
        self.frame = input.u64()?;
        self.vblank = input.u8()? != 0;
        self.nmi_pending = input.u8()? != 0;
        self.range_over = input.u8()? != 0;
        self.time_over = input.u8()? != 0;
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
    fn completed_scanlines_preserve_the_ppu_state_active_for_that_line() {
        let mut ppu = SnesPpu::new();
        ppu.write(0x2100, 0x0f);
        ppu.cgram[0] = 0x001f;
        ppu.tick_dots(341 * 2);
        ppu.cgram[0] = 0x03e0;
        ppu.tick_dots(341);

        assert_eq!(&ppu.video().pixels()[..3], &[255, 0, 0]);
        let row1 = ppu.video().width() as usize * 4;
        assert_eq!(&ppu.video().pixels()[row1..row1 + 3], &[0, 255, 0]);
    }

    #[test]
    fn mid_frame_hires_promotion_preserves_completed_low_resolution_rows() {
        let mut ppu = SnesPpu::new();
        ppu.write(0x2100, 0x0f);
        ppu.cgram[0] = 0x001f;
        ppu.tick_dots(341 * 2);
        ppu.write(0x2105, 5);
        ppu.write(0x2132, 0x5f);
        ppu.tick_dots(341);

        assert_eq!(ppu.video().width(), 512);
        assert_eq!(&ppu.video().pixels()[..3], &[255, 0, 0]);
        assert_eq!(&ppu.video().pixels()[4..7], &[255, 0, 0]);
        let row1 = ppu.video().width() as usize * 4;
        assert_eq!(&ppu.video().pixels()[row1..row1 + 3], &[0, 255, 0]);
        assert_eq!(&ppu.video().pixels()[row1 + 4..row1 + 7], &[255, 0, 0]);
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

    #[test]
    fn oam_uses_word_address_and_two_write_low_table_latch() {
        let mut ppu = SnesPpu::new();
        ppu.write(0x2102, 1);
        ppu.write(0x2103, 0);
        assert_eq!(ppu.oam_addr, 2);
        ppu.write(0x2104, 0xaa);
        assert_eq!(ppu.oam[2], 0);
        ppu.write(0x2104, 0x55);
        assert_eq!(&ppu.oam[2..4], &[0xaa, 0x55]);

        ppu.write(0x2102, 0);
        ppu.write(0x2103, 1);
        assert_eq!(ppu.oam_addr, 512);
        ppu.write(0x2104, 0x03);
        assert_eq!(ppu.oam[512], 0x03);
    }

    #[test]
    fn background_scroll_selects_adjacent_tile() {
        let mut ppu = SnesPpu::new();
        ppu.write(0x2100, 0x0f);
        ppu.write(0x2105, 0);
        ppu.write(0x2107, 0x04);
        ppu.write(0x212c, 0x01);
        ppu.cgram[1] = 0x001f;
        for row in 0..8 {
            ppu.vram[16 + row * 2] = 0xff;
        }
        ppu.vram[0x802] = 1;
        ppu.vram[0x803] = 0;
        ppu.write(0x210d, 8);
        ppu.write(0x210d, 0);
        assert_eq!(ppu.bg_scroll_x[0], 8);
        ppu.force_render();
        assert!(ppu.video().pixels()[0] > 200);
    }

    #[test]
    fn mosaic_repeats_each_background_blocks_upper_left_pixel() {
        let mut ppu = SnesPpu::new();
        ppu.write(0x2100, 0x0f);
        ppu.write(0x2105, 0);
        ppu.write(0x2106, 0x31);
        ppu.write(0x2107, 0x04);
        ppu.write(0x212c, 0x01);
        ppu.cgram[1] = 0x001f;
        ppu.cgram[2] = 0x03e0;
        ppu.vram[2] = 0x80;
        ppu.vram[3] = 0x40;
        ppu.vram[10] = 0x00;
        ppu.vram[11] = 0x80;
        ppu.force_render();

        for y in 0..4usize {
            for x in 0..4usize {
                let offset = (y * WIDTH as usize + x) * 4;
                assert_eq!(&ppu.video().pixels()[offset..offset + 3], &[255, 0, 0]);
            }
        }
        let offset = 4 * WIDTH as usize * 4;
        assert_eq!(&ppu.video().pixels()[offset..offset + 3], &[0, 255, 0]);
        assert_eq!(ppu.mosaic_coordinates(1, 3, 3), (3, 3));
    }

    #[test]
    fn main_screen_window_masks_selected_background_pixels() {
        let mut ppu = SnesPpu::new();
        ppu.write(0x2100, 0x0f);
        ppu.write(0x2105, 0);
        ppu.write(0x2107, 0x04);
        ppu.write(0x2123, 0x02);
        ppu.write(0x2126, 2);
        ppu.write(0x2127, 4);
        ppu.write(0x212c, 0x01);
        ppu.write(0x212e, 0x01);
        ppu.cgram[1] = 0x001f;
        for row in 0..8usize {
            ppu.vram[row * 2] = 0xff;
        }
        ppu.force_render();

        let red = |x: usize| ppu.video().pixels()[x * 4] > 200;
        assert!(red(1));
        assert!(!red(2));
        assert!(!red(4));
        assert!(red(5));
    }

    #[test]
    fn window_logic_combines_and_inverts_both_window_devices() {
        let mut ppu = SnesPpu::new();
        ppu.write(0x2123, 0x0a);
        ppu.write(0x2126, 2);
        ppu.write(0x2127, 5);
        ppu.write(0x2128, 4);
        ppu.write(0x2129, 7);
        ppu.write(0x212a, 0x01);
        assert!(!ppu.window_mask(0, 3));
        assert!(ppu.window_mask(0, 4));
        assert!(ppu.window_mask(0, 5));
        assert!(!ppu.window_mask(0, 6));

        ppu.write(0x2123, 0x03);
        assert!(ppu.window_mask(0, 1));
        assert!(!ppu.window_mask(0, 3));
        assert!(ppu.window_mask(0, 8));
    }

    #[test]
    fn fixed_color_port_and_math_follow_five_bit_channel_rules() {
        let mut ppu = SnesPpu::new();
        ppu.write(0x2132, 0x3f);
        ppu.write(0x2132, 0x4f);
        ppu.write(0x2132, 0x83);
        assert_eq!(ppu.fixed_color, 0x0dff);

        ppu.fixed_color = 10;
        ppu.write(0x2131, 0x01);
        assert_eq!(ppu.apply_fixed_color_math(10, 0, 0, true), 20);
        ppu.fixed_color = 31;
        ppu.write(0x2131, 0x41);
        assert_eq!(ppu.apply_fixed_color_math(31, 0, 0, true), 31);
        ppu.fixed_color = 15;
        ppu.write(0x2131, 0x81);
        assert_eq!(ppu.apply_fixed_color_math(10, 0, 0, true), 0);
        assert_eq!(ppu.apply_fixed_color_math(10, 0, 0, false), 10);
    }

    #[test]
    fn color_window_can_disable_fixed_color_math_inside_window() {
        let mut ppu = SnesPpu::new();
        ppu.fixed_color = 10;
        ppu.write(0x2125, 0x20);
        ppu.write(0x2126, 2);
        ppu.write(0x2127, 4);
        ppu.write(0x2130, 0x20);
        ppu.write(0x2131, 0x01);
        assert_eq!(ppu.apply_fixed_color_math(10, 1, 0, true), 20);
        assert_eq!(ppu.apply_fixed_color_math(10, 3, 0, true), 10);
        ppu.write(0x2130, 0x02);
        assert_eq!(ppu.apply_fixed_color_math(10, 1, 0, true), 10);
    }

    #[test]
    fn fixed_color_math_is_applied_to_rendered_background_and_backdrop() {
        let mut ppu = SnesPpu::new();
        ppu.write(0x2100, 0x0f);
        ppu.write(0x2105, 0);
        ppu.write(0x2107, 0x04);
        ppu.write(0x212c, 0x01);
        ppu.write(0x2131, 0x21);
        ppu.write(0x2132, 0x5f);
        ppu.cgram[0] = 0x001f;
        ppu.cgram[1] = 0x001f;
        for row in 0..8usize {
            ppu.vram[row * 2] = 0xff;
        }
        ppu.vram[0x802] = 1;
        ppu.vram[0x803] = 0;
        ppu.force_render();

        assert_eq!(&ppu.video().pixels()[..3], &[255, 255, 0]);
        let backdrop = 8 * 4;
        assert_eq!(
            &ppu.video().pixels()[backdrop..backdrop + 3],
            &[255, 255, 0]
        );
    }

    #[test]
    fn subscreen_layer_blends_with_main_backdrop_and_honors_tsw() {
        let mut ppu = SnesPpu::new();
        ppu.write(0x2100, 0x0f);
        ppu.write(0x2105, 0);
        ppu.write(0x2107, 0x04);
        ppu.write(0x2123, 0x02);
        ppu.write(0x2126, 2);
        ppu.write(0x2127, 4);
        ppu.write(0x212d, 0x01);
        ppu.write(0x212f, 0x01);
        ppu.write(0x2130, 0x02);
        ppu.write(0x2131, 0x20);
        ppu.write(0x2132, 0x9f);
        ppu.cgram[0] = 0x001f;
        ppu.cgram[1] = 0x03e0;
        for row in 0..8usize {
            ppu.vram[row * 2] = 0xff;
        }
        ppu.force_render();

        assert_eq!(&ppu.video().pixels()[4..7], &[255, 255, 0]);
        let masked = 3 * 4;
        assert_eq!(&ppu.video().pixels()[masked..masked + 3], &[255, 0, 255]);
    }

    #[test]
    fn color_window_can_black_main_and_make_subscreen_addend_transparent() {
        let mut ppu = SnesPpu::new();
        ppu.write(0x2100, 0x0f);
        ppu.write(0x2125, 0x20);
        ppu.write(0x2126, 2);
        ppu.write(0x2127, 4);
        ppu.cgram[0] = 0x001f;
        ppu.write(0x2130, 0x80);
        ppu.force_render();
        assert_eq!(&ppu.video().pixels()[4..7], &[255, 0, 0]);
        assert_eq!(&ppu.video().pixels()[12..15], &[0, 0, 0]);

        ppu.write(0x2107, 0x04);
        ppu.write(0x212d, 0x01);
        ppu.write(0x2130, 0x22);
        ppu.write(0x2131, 0x20);
        ppu.cgram[1] = 0x03e0;
        for row in 0..8usize {
            ppu.vram[row * 2] = 0xff;
        }
        ppu.force_render();
        assert_eq!(&ppu.video().pixels()[4..7], &[255, 255, 0]);
        assert_eq!(&ppu.video().pixels()[12..15], &[255, 0, 0]);
    }

    #[test]
    fn fixed_color_math_remains_available_in_high_resolution_mode() {
        let mut ppu = SnesPpu::new();
        ppu.write(0x2100, 0x0f);
        ppu.write(0x2105, 5);
        ppu.write(0x2131, 0x20);
        ppu.write(0x2132, 0x5f);
        ppu.cgram[0] = 0x001f;
        ppu.force_render();
        assert_eq!(ppu.video().width(), 512);
        assert_eq!(&ppu.video().pixels()[4..7], &[255, 255, 0]);
    }

    #[test]
    fn mode5_deinterleaves_sub_and_main_pixels_into_512_columns() {
        let mut ppu = SnesPpu::new();
        ppu.write(0x2100, 0x0f);
        ppu.write(0x2105, 5);
        ppu.write(0x2107, 0x04);
        ppu.write(0x212c, 0x01);
        ppu.write(0x212d, 0x01);
        ppu.cgram[1] = 0x001f;
        ppu.cgram[2] = 0x03e0;
        for row in 0..8usize {
            ppu.vram[row * 2] = 0x80;
            ppu.vram[row * 2 + 1] = 0x40;
        }
        ppu.force_render();

        assert_eq!(ppu.video().width(), 512);
        assert_eq!(&ppu.video().pixels()[..3], &[255, 0, 0]);
        assert_eq!(&ppu.video().pixels()[4..7], &[0, 255, 0]);
    }

    #[test]
    fn pseudo_hires_interleaves_fixed_subscreen_and_main_backdrop() {
        let mut ppu = SnesPpu::new();
        ppu.write(0x2100, 0x0f);
        ppu.write(0x2133, 0x08);
        ppu.write(0x2132, 0x9f);
        ppu.cgram[0] = 0x001f;
        ppu.force_render();

        assert_eq!(ppu.video().width(), 512);
        assert_eq!(&ppu.video().pixels()[..3], &[0, 0, 255]);
        assert_eq!(&ppu.video().pixels()[4..7], &[255, 0, 0]);
    }

    #[test]
    fn half_color_is_suppressed_for_transparent_subscreen_and_forced_black() {
        let mut ppu = SnesPpu::new();
        ppu.fixed_color = 10;
        ppu.main_screen[0] = ScreenPixel {
            color: 10,
            layer: 0,
            math_allowed: true,
            transparent: false,
        };
        ppu.sub_screen[0] = ScreenPixel::backdrop(10);
        ppu.write(0x2130, 0x02);
        ppu.write(0x2131, 0x41);
        assert_eq!(ppu.compose_pixel(0, 0), 20);

        ppu.sub_screen[0] = ScreenPixel {
            color: 10,
            layer: 0,
            math_allowed: true,
            transparent: false,
        };
        assert_eq!(ppu.compose_pixel(0, 0), 10);

        ppu.write(0x2125, 0x20);
        ppu.write(0x2126, 0);
        ppu.write(0x2127, 0);
        ppu.write(0x2130, 0x80);
        assert_eq!(ppu.compose_pixel(0, 0), 10);
    }

    #[test]
    fn mode2_offset_map_changes_rendered_second_column() {
        let mut ppu = SnesPpu::new();
        ppu.write(0x2100, 0x0f);
        ppu.write(0x2105, 2);
        ppu.write(0x2107, 0x04);
        ppu.write(0x2109, 0x08);
        ppu.write(0x212c, 0x01);
        ppu.cgram[1] = 0x001f;
        ppu.cgram[2] = 0x03e0;
        for row in 0..8usize {
            ppu.vram[row * 2] = 0xff;
            ppu.vram[32 + row * 2 + 1] = 0xff;
        }
        ppu.vram[0x804..0x806].copy_from_slice(&1u16.to_le_bytes());
        ppu.vram[0x1000..0x1002].copy_from_slice(&0x2008u16.to_le_bytes());
        ppu.force_render();

        assert_eq!(&ppu.video().pixels()[..3], &[255, 0, 0]);
        let second_column = 8 * 4;
        assert_eq!(
            &ppu.video().pixels()[second_column..second_column + 3],
            &[0, 255, 0]
        );
    }

    #[test]
    fn mode2_offset_map_overrides_second_column_horizontal_and_vertical_scroll() {
        let mut ppu = SnesPpu::new();
        ppu.write(0x2105, 2);
        ppu.write(0x2109, 0x08);
        ppu.bg_scroll_x[0] = 3;
        ppu.bg_scroll_y[0] = 5;
        ppu.vram[0x1000..0x1002].copy_from_slice(&0x2018u16.to_le_bytes());
        ppu.vram[0x1040..0x1042].copy_from_slice(&0x2028u16.to_le_bytes());

        assert_eq!(ppu.offset_scroll(0, 0), (3, 5));
        assert_eq!(ppu.offset_scroll(0, 8), (27, 40));
    }

    #[test]
    fn mode2_offset_map_uses_eight_pixel_rows_even_when_bg3_tiles_are_large() {
        let mut ppu = SnesPpu::new();
        ppu.write(0x2105, 0x42);
        ppu.write(0x2109, 0x08);
        ppu.bg_scroll_x[0] = 3;
        ppu.bg_scroll_y[0] = 5;
        ppu.vram[0x1000..0x1002].copy_from_slice(&0x2018u16.to_le_bytes());
        ppu.vram[0x1040..0x1042].copy_from_slice(&0x2028u16.to_le_bytes());

        assert_eq!(ppu.offset_scroll(0, 8), (27, 40));
    }

    #[test]
    fn mode4_offset_entry_selects_vertical_or_horizontal_override() {
        let mut ppu = SnesPpu::new();
        ppu.write(0x2105, 4);
        ppu.write(0x2109, 0x08);
        ppu.bg_scroll_x[0] = 5;
        ppu.bg_scroll_y[0] = 7;
        ppu.vram[0x1000..0x1002].copy_from_slice(&0xa030u16.to_le_bytes());
        assert_eq!(ppu.offset_scroll(0, 8), (5, 48));

        ppu.vram[0x1000..0x1002].copy_from_slice(&0x2028u16.to_le_bytes());
        assert_eq!(ppu.offset_scroll(0, 8), (45, 7));
    }

    #[test]
    fn mode6_offset_scroll_uses_hires_horizontal_units() {
        let mut ppu = SnesPpu::new();
        ppu.write(0x2105, 6);
        ppu.write(0x2109, 0x08);
        ppu.bg_scroll_x[0] = 3;
        ppu.bg_scroll_y[0] = 9;
        ppu.vram[0x1000..0x1002].copy_from_slice(&0x2018u16.to_le_bytes());
        assert_eq!(ppu.offset_scroll(0, 0), (6, 9));
        assert_eq!(ppu.offset_scroll(0, 8), (30, 9));
    }

    #[test]
    fn mode7_mosaic_uses_bg1_enable_and_supplied_origin() {
        let mut ppu = SnesPpu::new();
        ppu.write(0x2100, 0x0f);
        ppu.write(0x2105, 7);
        ppu.write(0x2106, 0x11);
        ppu.write(0x212c, 0x01);
        ppu.cgram[5] = 0x001f;
        ppu.cgram[6] = 0x03e0;
        ppu.vram[0] = 1;
        ppu.vram[(64 * 2) + 1] = 5;
        ppu.vram[(65 * 2) + 1] = 6;
        ppu.vram[(66 * 2) + 1] = 6;
        ppu.force_render();

        assert_eq!(&ppu.video().pixels()[..3], &[255, 0, 0]);
        assert_eq!(&ppu.video().pixels()[4..7], &[255, 0, 0]);
        assert_eq!(&ppu.video().pixels()[8..11], &[0, 255, 0]);
    }

    #[test]
    fn mode3_direct_color_uses_pixel_and_tile_palette_bits_without_cgram() {
        let mut ppu = SnesPpu::new();
        ppu.write(0x2100, 0x0f);
        ppu.write(0x2105, 3);
        ppu.write(0x2107, 0x04);
        ppu.write(0x212c, 0x01);
        ppu.write(0x2130, 0x01);
        let pixel = 0b10_101_011u8;
        let palette = 0b101u8;
        ppu.vram[0x800..0x802].copy_from_slice(&(u16::from(palette) << 10).to_le_bytes());
        for row in 0..8usize {
            for plane in 0..8usize {
                if pixel & (1 << plane) != 0 {
                    ppu.vram[(plane / 2) * 16 + row * 2 + plane % 2] = 0x80;
                }
            }
        }
        ppu.cgram[usize::from(pixel)] = 0x001f;
        ppu.force_render();

        assert_eq!(SnesPpu::direct_color(pixel, palette), 0x528e);
        assert_eq!(ppu.main_screen[0].color, 0x528e);
        assert_ne!(ppu.main_screen[0].color, ppu.cgram[usize::from(pixel)]);
    }

    #[test]
    fn mode3_bg1_decodes_eight_bit_tiles() {
        let mut ppu = SnesPpu::new();
        ppu.write(0x2100, 0x0f);
        ppu.write(0x2105, 3);
        ppu.write(0x2107, 0x04);
        ppu.write(0x212c, 0x01);
        ppu.cgram[1] = 0x03e0;
        for row in 0..8 {
            ppu.vram[row * 2] = 0xff;
        }
        ppu.force_render();
        assert!(ppu.video().pixels()[1] > 200);
    }

    #[test]
    fn stat77_latches_range_and_time_over_until_end_of_vblank() {
        let mut range_ppu = SnesPpu::new();
        for sprite in 0..128usize {
            range_ppu.oam[sprite * 4 + 1] = 224;
        }
        for sprite in 0..33usize {
            range_ppu.oam[sprite * 4] = (sprite * 7) as u8;
            range_ppu.oam[sprite * 4 + 1] = 0;
        }
        range_ppu.scanline = 1;
        range_ppu.cycle = 340;
        range_ppu.tick_dots(1);
        assert_eq!(range_ppu.read(0x213e) & 0xc0, 0x40);

        let mut time_ppu = SnesPpu::new();
        time_ppu.write(0x2101, 0x60);
        for sprite in 0..128usize {
            time_ppu.oam[sprite * 4 + 1] = 224;
        }
        for sprite in 0..18usize {
            time_ppu.oam[sprite * 4] = (sprite * 8) as u8;
            time_ppu.oam[sprite * 4 + 1] = 0;
        }
        time_ppu.scanline = 1;
        time_ppu.cycle = 340;
        time_ppu.tick_dots(1);
        assert_eq!(time_ppu.read(0x213e) & 0xc0, 0x80);
        assert_eq!(time_ppu.read(0x213e) & 0x0f, 0x01);

        time_ppu.scanline = 261;
        time_ppu.cycle = 340;
        time_ppu.vblank = true;
        time_ppu.tick_dots(1);
        assert_eq!(time_ppu.read(0x213e) & 0xc0, 0);
    }

    #[test]
    fn obj_scanline_keeps_first_32_sprites_and_counts_x_minus_256() {
        let mut ppu = SnesPpu::new();
        for sprite in 0..128usize {
            ppu.oam[sprite * 4 + 1] = 224;
        }
        for sprite in 0..33usize {
            ppu.oam[sprite * 4 + 1] = 0;
        }
        ppu.oam[512] |= 0x01;

        let (masks, sprite_overflow, sliver_overflow) = ppu.obj_scanline_sliver_masks(0);
        assert!(sprite_overflow);
        assert!(!sliver_overflow);
        assert_ne!(masks[31], 0);
        assert_eq!(masks[32], 0);
    }

    #[test]
    fn obj_sliver_budget_is_allocated_in_reverse_sprite_order() {
        let mut ppu = SnesPpu::new();
        ppu.write(0x2101, 0x60);
        for sprite in 0..128usize {
            ppu.oam[sprite * 4 + 1] = 224;
        }
        for sprite in 0..32usize {
            ppu.oam[sprite * 4 + 1] = 0;
        }

        let (masks, sprite_overflow, sliver_overflow) = ppu.obj_scanline_sliver_masks(0);
        assert!(!sprite_overflow);
        assert!(sliver_overflow);
        assert_eq!(masks[14], 0);
        assert_eq!(masks[15], 0x03);
        assert_eq!(masks[31], 0x03);
    }

    #[test]
    fn obj_sliver_budget_changes_rendered_top_sprite() {
        let mut ppu = SnesPpu::new();
        ppu.write(0x2100, 0x0f);
        ppu.write(0x2101, 0x60);
        ppu.write(0x212c, 0x10);
        for sprite in 0..128usize {
            ppu.oam[sprite * 4 + 1] = 224;
        }
        for sprite in 0..32usize {
            ppu.oam[sprite * 4 + 1] = 0;
            ppu.oam[sprite * 4 + 3] = 0x02;
        }
        ppu.oam[3] = 0x00;
        ppu.cgram[129] = 0x001f;
        ppu.cgram[145] = 0x03e0;
        for row in 0..8usize {
            ppu.vram[row * 2] = 0xff;
        }
        ppu.force_render();

        assert_eq!(&ppu.video().pixels()[..3], &[0, 255, 0]);
    }

    #[test]
    fn obj_scanline_limit_follows_priority_rotation_start() {
        let mut ppu = SnesPpu::new();
        for sprite in 0..128usize {
            ppu.oam[sprite * 4 + 1] = 224;
        }
        for sprite in 40..=72usize {
            ppu.oam[sprite * 4 + 1] = 0;
        }
        ppu.write(0x2102, 80);
        ppu.write(0x2103, 0x80);
        assert_eq!(ppu.first_sprite(), 40);

        let (masks, sprite_overflow, sliver_overflow) = ppu.obj_scanline_sliver_masks(0);
        assert!(sprite_overflow);
        assert!(!sliver_overflow);
        assert_ne!(masks[40], 0);
        assert_ne!(masks[71], 0);
        assert_eq!(masks[72], 0);
    }

    #[test]
    fn obj_uses_sprite_palette_and_four_bpp_tile_data() {
        let mut ppu = SnesPpu::new();
        ppu.write(0x2100, 0x0f);
        ppu.write(0x2101, 0);
        ppu.write(0x212c, 0x10);
        for sprite in 1..128 {
            ppu.oam[sprite * 4 + 1] = 224;
        }
        ppu.oam[0] = 0;
        ppu.oam[1] = 0;
        ppu.oam[2] = 0;
        ppu.oam[3] = 0x30;
        ppu.cgram[129] = 0x7c00;
        for row in 0..8 {
            ppu.vram[row * 2] = 0xff;
        }
        ppu.force_render();
        assert!(ppu.video().pixels()[2] > 200);
    }

    #[test]
    fn mode7_identity_matrix_reads_split_vram_tilemap_and_pixels() {
        let mut ppu = SnesPpu::new();
        ppu.write(0x2100, 0x0f);
        ppu.write(0x2105, 7);
        ppu.write(0x212c, 0x01);
        ppu.cgram[5] = 0x03e0;
        ppu.vram[0] = 1;
        for x in 0..8 {
            ppu.vram[((64 + x) * 2) + 1] = 5;
        }
        ppu.force_render();
        assert!(ppu.video().pixels()[1] > 200);
    }

    #[test]
    fn mode7_write_latch_and_signed_multiplier_follow_register_contract() {
        let mut ppu = SnesPpu::new();
        ppu.write(0x211b, 0x00);
        ppu.write(0x211b, 0x01);
        ppu.write(0x211c, 0x00);
        ppu.write(0x211c, 0x02);
        assert_eq!(ppu.mode7_matrix[0], 0x0100);
        assert_eq!(ppu.mode7_matrix[1], 0x0200);
        assert_eq!(ppu.read(0x2134), 0x00);
        assert_eq!(ppu.read(0x2135), 0x02);
        assert_eq!(ppu.read(0x2136), 0x00);
    }

    #[test]
    fn mode7_non_repeating_area_can_be_transparent_or_tile_zero() {
        let mut ppu = SnesPpu::new();
        ppu.write(0x2100, 0x0f);
        ppu.write(0x2105, 7);
        ppu.write(0x212c, 0x01);
        ppu.cgram[0] = 0x03e0;
        ppu.cgram[2] = 0x001f;
        ppu.cgram[3] = 0x7c00;
        ppu.vram[127 * 2] = 1;
        ppu.vram[((64 + 7) * 2) + 1] = 2;
        ppu.vram[1] = 3;
        ppu.write(0x210d, 0xff);
        ppu.write(0x210d, 0x03);
        ppu.write(0x211a, 0x80);
        ppu.force_render();
        assert!(ppu.video().pixels()[0] > 200);
        assert!(ppu.video().pixels()[5] > 200);

        ppu.write(0x211a, 0xc0);
        ppu.force_render();
        assert!(ppu.video().pixels()[6] > 200);
    }

    #[test]
    fn mode7_extbg_high_pixel_composes_above_sprite_priority_one() {
        let mut ppu = SnesPpu::new();
        ppu.write(0x2100, 0x0f);
        ppu.write(0x2105, 7);
        ppu.write(0x2101, 2);
        ppu.write(0x212c, 0x13);
        ppu.write(0x2133, 0x40);
        ppu.cgram[1] = 0x03e0;
        ppu.cgram[129] = 0x7c00;
        ppu.cgram[145] = 0x001f;
        ppu.vram[0] = 0;
        ppu.vram[1] = 0x81;
        for row in 0..8 {
            ppu.vram[0x8000 + row * 2] = 0xff;
        }
        for sprite in 1..128 {
            ppu.oam[sprite * 4 + 1] = 224;
        }
        ppu.oam[0] = 0;
        ppu.oam[1] = 0;
        ppu.oam[2] = 0;
        ppu.oam[3] = 0x12;
        ppu.force_render();
        let pixel = &ppu.video().pixels()[..4];
        assert!(pixel[1] > 200);
        assert!(pixel[0] < 40);
        assert!(pixel[2] < 40);
    }

    #[test]
    fn mode0_background_priority_can_cover_low_priority_obj() {
        let mut ppu = SnesPpu::new();
        ppu.write(0x2100, 0x0f);
        ppu.write(0x2105, 0);
        ppu.write(0x2107, 0x04);
        ppu.write(0x212c, 0x11);
        ppu.cgram[1] = 0x03e0;
        ppu.cgram[129] = 0x001f;
        for row in 0..8 {
            ppu.vram[row * 2] = 0xff;
        }
        for sprite in 1..128 {
            ppu.oam[sprite * 4 + 1] = 224;
        }
        ppu.oam[3] = 0x00;
        ppu.force_render();
        let pixel = &ppu.video().pixels()[..4];
        assert!(pixel[1] > 200);
        assert!(pixel[0] < 40);

        ppu.oam[3] = 0x30;
        ppu.force_render();
        let pixel = &ppu.video().pixels()[..4];
        assert!(pixel[0] > 200);
        assert!(pixel[1] < 40);
    }
}
