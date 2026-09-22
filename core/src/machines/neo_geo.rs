use crate::cpu68000::{Bus68000, M68000};
use crate::cpu_z80::{Z80Bus, Z80};
use crate::input::{DOWN, FACE_EAST, FACE_NORTH, FACE_SOUTH, FACE_WEST, LEFT, RIGHT, START, UP};
use crate::kernel::{AudioBuffer, InputState, VideoBuffer};
use crate::machine::Machine;
use crate::platform::PlatformId;
use crate::state::{StateReader, StateWriter};

const WIDTH: u32 = 320;
const HEIGHT: u32 = 224;
const FRAME_RATE_NUM: u64 = 24_000_000;
const FRAME_RATE_DEN: u64 = 1536 * 264;
const FRAME_RATE: f64 = FRAME_RATE_NUM as f64 / FRAME_RATE_DEN as f64;
const M68K_CLOCK: u64 = 12_000_000;
const Z80_CLOCK: u64 = 4_000_000;
const AUDIO_RATE: u32 = 48_000;
const STATE_VERSION: u32 = 3;
const NEO_HEADER: usize = 4096;
const SYSTEM_ROM_SIZE: usize = 128 * 1024;

#[derive(Debug, Clone)]
struct NeoCartridge {
    p: Vec<u8>,
    s: Vec<u8>,
    m: Vec<u8>,
    v1: Vec<u8>,
    v2: Vec<u8>,
    c: Vec<u8>,
}

impl NeoCartridge {
    fn u32le(bytes: &[u8], offset: usize) -> Result<u32, String> {
        let value = bytes
            .get(offset..offset + 4)
            .ok_or_else(|| "Neo Geo .neo header is truncated".to_string())?;
        Ok(u32::from_le_bytes(value.try_into().unwrap()))
    }

    fn from_neo(bytes: &[u8]) -> Result<Self, String> {
        if bytes.len() < NEO_HEADER {
            return Err("Neo Geo .neo file is smaller than its 4 KiB header".into());
        }
        if &bytes[..4] != b"NEO\x01" {
            return Err("Neo Geo content must use the NEO1 unified cartridge format".into());
        }
        let sizes = [
            Self::u32le(bytes, 4)?,
            Self::u32le(bytes, 8)?,
            Self::u32le(bytes, 12)?,
            Self::u32le(bytes, 16)?,
            Self::u32le(bytes, 20)?,
            Self::u32le(bytes, 24)?,
        ];
        if sizes[0] == 0 {
            return Err("Neo Geo .neo file has no 68K P ROM section".into());
        }
        let total = sizes
            .iter()
            .try_fold(NEO_HEADER as u64, |sum, size| {
                sum.checked_add(u64::from(*size)).ok_or(())
            })
            .map_err(|()| "Neo Geo .neo section sizes overflow".to_string())?;
        if total > bytes.len() as u64 {
            return Err("Neo Geo .neo sections extend past the file".into());
        }
        let mut cursor = NEO_HEADER;
        let mut take = |size: u32| {
            let end = cursor + size as usize;
            let section = bytes[cursor..end].to_vec();
            cursor = end;
            section
        };
        let p = take(sizes[0]);
        let s = take(sizes[1]);
        let m = take(sizes[2]);
        let v1 = take(sizes[3]);
        let v2 = take(sizes[4]);
        let c = take(sizes[5]);
        Ok(Self { p, s, m, v1, v2, c })
    }

    fn adpcm_b_rom(&self) -> &[u8] {
        if self.v2.is_empty() {
            &self.v1
        } else {
            &self.v2
        }
    }
}

fn neo_color(value: u16) -> [u8; 4] {
    let common = u8::from(value & 0x8000 != 0);
    let red = (((value >> 8) & 0x0f) as u8) << 1 | ((value >> 14) as u8 & 1);
    let green = (((value >> 4) & 0x0f) as u8) << 1 | ((value >> 13) as u8 & 1);
    let blue = ((value & 0x0f) as u8) << 1 | ((value >> 12) as u8 & 1);
    let expand = |component: u8| ((u16::from(component) * 255 + 15) / 31) as u8;
    let darken = |component: u8| {
        if common != 0 {
            component.saturating_sub(component / 8)
        } else {
            component
        }
    };
    [
        darken(expand(red)),
        darken(expand(green)),
        darken(expand(blue)),
        255,
    ]
}

struct NeoVideo {
    vram: Box<[u16; 0x8800]>,
    address: u16,
    modulo: i16,
    mode: u16,
    timer_reload: u32,
    timer_counter: u32,
    palette: [Box<[u16; 4096]>; 2],
    palette_bank: usize,
    frame: u64,
    frame_phase: u128,
    vblank_pending: bool,
    timer_pending: bool,
    irq3_pending: bool,
    shadow: bool,
    cart_fix: bool,
    video: VideoBuffer,
}

impl Default for NeoVideo {
    fn default() -> Self {
        Self {
            vram: Box::new([0; 0x8800]),
            address: 0,
            modulo: 1,
            mode: 0,
            timer_reload: 0,
            timer_counter: 0,
            palette: [Box::new([0; 4096]), Box::new([0; 4096])],
            palette_bank: 0,
            frame: 0,
            frame_phase: 0,
            vblank_pending: false,
            timer_pending: false,
            irq3_pending: false,
            shadow: false,
            cart_fix: true,
            video: VideoBuffer::new(WIDTH, HEIGHT),
        }
    }
}

impl NeoVideo {
    fn read_register(&self, address: u32) -> u16 {
        match address & 0x0e {
            0x00 => self.vram[usize::from(self.address).min(0x87ff)],
            0x02 => self.vram[usize::from(self.address).min(0x87ff)],
            0x04 => self.modulo as u16,
            0x06 => self.mode,
            0x08 => (self.timer_reload >> 16) as u16,
            0x0a => self.timer_reload as u16,
            _ => 0xffff,
        }
    }

    fn write_register(&mut self, address: u32, value: u16) {
        match address & 0x0e {
            0x00 => self.address = value,
            0x02 => {
                let index = usize::from(self.address);
                if index < self.vram.len() {
                    self.vram[index] = value;
                }
                self.address = self.address.wrapping_add_signed(self.modulo);
            }
            0x04 => self.modulo = value as i16,
            0x06 => self.mode = value,
            0x08 => self.timer_reload = (self.timer_reload & 0xffff) | (u32::from(value) << 16),
            0x0a => {
                self.timer_reload = (self.timer_reload & 0xffff0000) | u32::from(value);
                if self.timer_counter == 0 {
                    self.timer_counter = self.timer_reload;
                }
            }
            0x0c => {
                if value & 0x04 != 0 {
                    self.vblank_pending = false;
                }
                if value & 0x02 != 0 {
                    self.timer_pending = false;
                }
                if value & 0x01 != 0 {
                    self.irq3_pending = false;
                }
            }
            _ => {}
        }
    }

    fn irq_level(&self) -> u8 {
        if self.vblank_pending {
            1
        } else if self.timer_pending {
            2
        } else if self.irq3_pending {
            3
        } else {
            0
        }
    }

    fn palette_write8(&mut self, offset: usize, value: u8) {
        let word_index = (offset >> 1) & 0x0fff;
        let replicated = u16::from(value) * 0x0101;
        self.palette[self.palette_bank][word_index] = replicated;
    }

    fn palette_write16(&mut self, offset: usize, value: u16) {
        self.palette[self.palette_bank][(offset >> 1) & 0x0fff] = value;
    }

    fn palette_read8(&self, offset: usize) -> u8 {
        let value = self.palette[self.palette_bank][(offset >> 1) & 0x0fff];
        if offset & 1 == 0 {
            (value >> 8) as u8
        } else {
            value as u8
        }
    }

    fn color(&self, palette: usize, index: usize) -> [u8; 4] {
        let mut color = neo_color(
            self.palette[self.palette_bank][((palette & 0xff) * 16 + (index & 15)) & 0x0fff],
        );
        if self.shadow {
            color[0] >>= 1;
            color[1] >>= 1;
            color[2] >>= 1;
        }
        color
    }

    fn fix_pixel(srom: &[u8], tile: usize, x: usize, y: usize) -> u8 {
        let base = tile.saturating_mul(32);
        if base + 32 > srom.len() {
            return 0;
        }
        let pair = x / 2;
        let half = usize::from(pair < 2);
        let column = pair & 1;
        let address = base + (half << 4) + (column << 3) + y;
        let byte = srom[address];
        if x & 1 == 0 {
            byte & 0x0f
        } else {
            byte >> 4
        }
    }

    fn sprite_pixel(crom: &[u8], tile: usize, x: usize, y: usize) -> u8 {
        let base = tile.saturating_mul(128);
        if base + 128 > crom.len() {
            return 0;
        }
        let block = match (x >= 8, y >= 8) {
            (true, false) => 0,
            (true, true) => 1,
            (false, false) => 2,
            (false, true) => 3,
        };
        let row = y & 7;
        let pixel = x & 7;
        let source = block * 32 + row * 4;
        let bp0 = crom[base + source];
        let bp2 = crom[base + source + 1];
        let bp1 = crom[base + source + 2];
        let bp3 = crom[base + source + 3];
        let bit = 7 - pixel;
        ((bp0 >> bit) & 1)
            | (((bp1 >> bit) & 1) << 1)
            | (((bp2 >> bit) & 1) << 2)
            | (((bp3 >> bit) & 1) << 3)
    }

    fn render_sprites(&mut self, cart: &NeoCartridge) {
        let mut line_usage = [0u8; HEIGHT as usize];
        for sprite in 1..=380usize {
            let scb3 = self.vram[0x8200 + sprite];
            let scb4 = self.vram[0x8400 + sprite];
            let height_tiles = usize::from(scb3 & 0x003f).min(32);
            if height_tiles == 0 {
                continue;
            }
            let x = i32::from(scb4 >> 7) - 0x18;
            let y = (496i32 - i32::from(scb3 >> 7)) & 0x1ff;
            let x_shrink = usize::from((self.vram[0x8000 + sprite] >> 8) & 0x0f) + 1;
            for tile_row in 0..height_tiles {
                let entry = sprite * 64 + tile_row * 2;
                if entry + 1 >= 0x7000 {
                    break;
                }
                let low = u32::from(self.vram[entry]);
                let attr = self.vram[entry + 1];
                let tile = (low | (u32::from(attr & 0x00f0) << 12)) as usize;
                let palette = usize::from(attr >> 8);
                let flip_x = attr & 0x0001 != 0;
                let flip_y = attr & 0x0002 != 0;
                for sy in 0..16usize {
                    let py = (y + (tile_row * 16 + sy) as i32) & 0x1ff;
                    if py >= HEIGHT as i32 {
                        continue;
                    }
                    if line_usage[py as usize] >= 96 {
                        continue;
                    }
                    for dx in 0..x_shrink {
                        let sx = dx * 16 / x_shrink;
                        let source_x = if flip_x { 15 - sx } else { sx };
                        let source_y = if flip_y { 15 - sy } else { sy };
                        let color_index = Self::sprite_pixel(&cart.c, tile, source_x, source_y);
                        if color_index == 0 {
                            continue;
                        }
                        let px = x + dx as i32;
                        if !(0..WIDTH as i32).contains(&px) {
                            continue;
                        }
                        let color = self.color(palette, usize::from(color_index));
                        let offset = (py as usize * WIDTH as usize + px as usize) * 4;
                        self.video.pixels_mut()[offset..offset + 4].copy_from_slice(&color);
                    }
                    line_usage[py as usize] = line_usage[py as usize].saturating_add(1);
                }
            }
        }
    }

    fn render_fix(&mut self, cart: &NeoCartridge) {
        let srom = &cart.s;
        for x_tile in 0..40usize {
            for y_tile in 0..28usize {
                let map_index = 0x7000 + x_tile * 32 + y_tile + 2;
                let entry = self.vram[map_index];
                let tile = usize::from(entry & 0x0fff);
                let palette = usize::from(entry >> 12) & 0x0f;
                for y in 0..8usize {
                    for x in 0..8usize {
                        let index = Self::fix_pixel(srom, tile, x, y);
                        if index == 0 {
                            continue;
                        }
                        let color = self.color(palette, usize::from(index));
                        let px = x_tile * 8 + x;
                        let py = y_tile * 8 + y;
                        let offset = (py * WIDTH as usize + px) * 4;
                        self.video.pixels_mut()[offset..offset + 4].copy_from_slice(&color);
                    }
                }
            }
        }
    }

    fn render(&mut self, cart: &NeoCartridge) {
        self.video.clear(self.color(255, 15));
        self.render_sprites(cart);
        if self.cart_fix {
            self.render_fix(cart);
        }
    }

    fn tick(&mut self, cycles: u32, cart: &NeoCartridge) {
        if self.mode & 0x0010 != 0 && self.timer_reload != 0 {
            let ticks = cycles.saturating_mul(2);
            if ticks >= self.timer_counter && self.timer_counter != 0 {
                self.timer_pending = true;
                self.timer_counter = self.timer_reload;
            } else {
                self.timer_counter = self.timer_counter.saturating_sub(ticks);
            }
        }
        let denominator = u128::from(M68K_CLOCK) * u128::from(FRAME_RATE_DEN);
        self.frame_phase += u128::from(cycles) * u128::from(FRAME_RATE_NUM);
        while self.frame_phase >= denominator {
            self.frame_phase -= denominator;
            self.frame = self.frame.wrapping_add(1);
            self.vblank_pending = true;
            self.render(cart);
        }
    }

    fn save(&self, out: &mut StateWriter) {
        out.u16(self.address);
        out.u16(self.modulo as u16);
        out.u16(self.mode);
        out.u32(self.timer_reload);
        out.u32(self.timer_counter);
        out.u8(self.palette_bank as u8);
        out.u64(self.frame);
        out.u128(self.frame_phase);
        out.u8(u8::from(self.vblank_pending));
        out.u8(u8::from(self.timer_pending));
        out.u8(u8::from(self.irq3_pending));
        out.u8(u8::from(self.shadow));
        out.u8(u8::from(self.cart_fix));
        for word in self.vram.iter() {
            out.u16(*word);
        }
        for bank in &self.palette {
            for word in bank.iter() {
                out.u16(*word);
            }
        }
    }

    fn load(&mut self, input: &mut StateReader<'_>, cart: &NeoCartridge) -> Result<(), String> {
        self.address = input.u16()?;
        self.modulo = input.u16()? as i16;
        self.mode = input.u16()?;
        self.timer_reload = input.u32()?;
        self.timer_counter = input.u32()?;
        self.palette_bank = usize::from(input.u8()? & 1);
        self.frame = input.u64()?;
        self.frame_phase = input.u128()?;
        self.vblank_pending = input.u8()? != 0;
        self.timer_pending = input.u8()? != 0;
        self.irq3_pending = input.u8()? != 0;
        self.shadow = input.u8()? != 0;
        self.cart_fix = input.u8()? != 0;
        for word in self.vram.iter_mut() {
            *word = input.u16()?;
        }
        for bank in &mut self.palette {
            for word in bank.iter_mut() {
                *word = input.u16()?;
            }
        }
        self.render(cart);
        Ok(())
    }
}

#[derive(Clone, Copy)]
struct AdpcmAChannel {
    playing: bool,
    high_nibble: bool,
    current_byte: u8,
    address: u32,
    accumulator: i32,
    step_index: i32,
}

impl Default for AdpcmAChannel {
    fn default() -> Self {
        Self {
            playing: false,
            high_nibble: true,
            current_byte: 0,
            address: 0,
            accumulator: 0,
            step_index: 0,
        }
    }
}

impl AdpcmAChannel {
    const STEPS: [i32; 49] = [
        16, 17, 19, 21, 23, 25, 28, 31, 34, 37, 41, 45, 50, 55, 60, 66, 73, 80, 88, 97, 107, 118,
        130, 143, 157, 173, 190, 209, 230, 253, 279, 307, 337, 371, 408, 449, 494, 544, 598, 658,
        724, 796, 876, 963, 1060, 1166, 1282, 1411, 1552,
    ];
    const STEP_DELTA: [i32; 8] = [-1, -1, -1, -1, 2, 5, 7, 9];

    fn key_on(&mut self, start: u32) {
        self.playing = true;
        self.high_nibble = true;
        self.current_byte = 0;
        self.address = start & 0x00ff_ffff;
        self.accumulator = 0;
        self.step_index = 0;
    }

    fn key_off(&mut self) {
        self.playing = false;
        self.accumulator = 0;
    }

    fn clock(&mut self, rom: &[u8], end_exclusive: u32) -> bool {
        if !self.playing {
            return false;
        }
        let nibble = if self.high_nibble {
            if ((self.address ^ end_exclusive) & 0x000f_ffff) == 0 {
                self.key_off();
                return true;
            }
            let Some(&byte) = rom.get(self.address as usize) else {
                self.key_off();
                return true;
            };
            self.current_byte = byte;
            self.address = self.address.wrapping_add(1) & 0x00ff_ffff;
            self.high_nibble = false;
            byte >> 4
        } else {
            self.high_nibble = true;
            self.current_byte & 0x0f
        };
        let magnitude = i32::from(nibble & 7);
        let mut delta = (2 * magnitude + 1) * Self::STEPS[self.step_index as usize] / 8;
        if nibble & 8 != 0 {
            delta = -delta;
        }
        self.accumulator = (self.accumulator + delta) & 0x0fff;
        self.step_index = (self.step_index + Self::STEP_DELTA[magnitude as usize]).clamp(0, 48);
        false
    }

    fn mix(&self, pan_level: u8, total_level: u8) -> (f32, f32) {
        if !self.playing {
            return (0.0, 0.0);
        }
        let volume = i32::from((pan_level & 0x1f) ^ 0x1f) + i32::from((total_level & 0x3f) ^ 0x3f);
        if volume >= 63 {
            return (0.0, 0.0);
        }
        let multiplier = 15 - (volume & 7);
        let shift = 5 + (volume >> 3);
        let signed = if self.accumulator & 0x0800 != 0 {
            self.accumulator - 0x1000
        } else {
            self.accumulator
        };
        let value = ((signed * 16 * multiplier) >> shift) as f32 / 32768.0;
        (
            if pan_level & 0x80 != 0 { value } else { 0.0 },
            if pan_level & 0x40 != 0 { value } else { 0.0 },
        )
    }

    fn save(&self, out: &mut StateWriter) {
        out.u8(u8::from(self.playing));
        out.u8(u8::from(self.high_nibble));
        out.u8(self.current_byte);
        out.u32(self.address);
        out.u32(self.accumulator as u32);
        out.u32(self.step_index as u32);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.playing = input.u8()? != 0;
        self.high_nibble = input.u8()? != 0;
        self.current_byte = input.u8()?;
        self.address = input.u32()? & 0x00ff_ffff;
        self.accumulator = input.u32()? as i32 & 0x0fff;
        self.step_index = (input.u32()? as i32).clamp(0, 48);
        Ok(())
    }
}

#[derive(Clone, Copy)]
struct AdpcmBChannel {
    playing: bool,
    high_nibble: bool,
    current_byte: u8,
    address: u32,
    position: u32,
    accumulator: i32,
    previous: i32,
    step: i32,
    eos: bool,
}

impl Default for AdpcmBChannel {
    fn default() -> Self {
        Self {
            playing: false,
            high_nibble: true,
            current_byte: 0,
            address: 0,
            position: 0,
            accumulator: 0,
            previous: 0,
            step: 127,
            eos: false,
        }
    }
}

impl AdpcmBChannel {
    const STEP_SCALE: [i32; 8] = [57, 57, 57, 57, 77, 102, 128, 153];

    fn start(&mut self, start: u32) {
        self.playing = true;
        self.high_nibble = true;
        self.current_byte = 0;
        self.address = start & 0x00ff_ffff;
        self.position = 0;
        self.accumulator = 0;
        self.previous = 0;
        self.step = 127;
        self.eos = false;
    }

    fn reset(&mut self) {
        *self = Self::default();
    }

    fn clock(&mut self, regs: &[u8; 256], rom: &[u8]) {
        if !self.playing {
            return;
        }
        let delta_n = u32::from(u16::from_le_bytes([regs[0x19], regs[0x1a]]));
        if delta_n == 0 {
            return;
        }
        let next = self.position + delta_n;
        self.position = next & 0xffff;
        if next < 0x1_0000 {
            return;
        }
        let nibble = if self.high_nibble {
            let Some(&byte) = rom.get(self.address as usize) else {
                self.playing = false;
                self.eos = true;
                return;
            };
            self.current_byte = byte;
            self.high_nibble = false;
            byte >> 4
        } else {
            self.high_nibble = true;
            let nibble = self.current_byte & 0x0f;
            let end_exclusive = (u32::from(u16::from_le_bytes([regs[0x14], regs[0x15]])) + 1) << 8;
            if self.address.wrapping_add(1) >= end_exclusive {
                if regs[0x10] & 0x10 != 0 {
                    let start = u32::from(u16::from_le_bytes([regs[0x12], regs[0x13]])) << 8;
                    self.address = start;
                } else {
                    self.playing = false;
                    self.eos = true;
                }
            } else {
                self.address = self.address.wrapping_add(1) & 0x00ff_ffff;
            }
            nibble
        };
        self.previous = self.accumulator;
        let magnitude = i32::from(nibble & 7);
        let mut delta = (2 * magnitude + 1) * self.step / 8;
        if nibble & 8 != 0 {
            delta = -delta;
        }
        self.accumulator = (self.accumulator + delta).clamp(-32768, 32767);
        self.step = (self.step * Self::STEP_SCALE[magnitude as usize] / 64).clamp(127, 24576);
    }

    fn mix(&self, regs: &[u8; 256]) -> (f32, f32) {
        if !self.playing || regs[0x10] & 0x08 != 0 {
            return (0.0, 0.0);
        }
        let fraction = self.position as i64;
        let interpolated = ((i64::from(self.previous) * (65536 - fraction)
            + i64::from(self.accumulator) * fraction)
            >> 16) as f32;
        let value = interpolated / 32768.0 * (f32::from(regs[0x1b]) / 255.0) * 0.5;
        let pan = regs[0x11];
        (
            if pan & 0x80 != 0 { value } else { 0.0 },
            if pan & 0x40 != 0 { value } else { 0.0 },
        )
    }

    fn save(&self, out: &mut StateWriter) {
        out.u8(u8::from(self.playing));
        out.u8(u8::from(self.high_nibble));
        out.u8(self.current_byte);
        out.u32(self.address);
        out.u32(self.position);
        out.u32(self.accumulator as u32);
        out.u32(self.previous as u32);
        out.u32(self.step as u32);
        out.u8(u8::from(self.eos));
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.playing = input.u8()? != 0;
        self.high_nibble = input.u8()? != 0;
        self.current_byte = input.u8()?;
        self.address = input.u32()? & 0x00ff_ffff;
        self.position = input.u32()? & 0xffff;
        self.accumulator = input.u32()? as i32;
        self.previous = input.u32()? as i32;
        self.step = (input.u32()? as i32).clamp(127, 24576);
        self.eos = input.u8()? != 0;
        Ok(())
    }
}

#[derive(Clone)]
struct Ym2610Foundation {
    address_a: u8,
    address_b: u8,
    regs_a: [u8; 256],
    regs_b: [u8; 256],
    tone_phase: [u64; 3],
    tone_level: [bool; 3],
    noise_phase: u64,
    noise_lfsr: u32,
    envelope_phase: u64,
    envelope_level: u8,
    envelope_up: bool,
    sample_phase: u64,
    adpcm_a_phase: u64,
    adpcm_b_phase: u64,
    adpcm_a: [AdpcmAChannel; 6],
    adpcm_b: AdpcmBChannel,
    eos_status: u8,
    flag_mask: u8,
    samples: Vec<(f32, f32)>,
    timer_a_phase: u64,
    timer_b_phase: u64,
    irq_pending: bool,
}

impl Default for Ym2610Foundation {
    fn default() -> Self {
        let mut regs_b = [0u8; 256];
        regs_b[0x08..=0x0d].fill(0xdf);
        Self {
            address_a: 0,
            address_b: 0,
            regs_a: [0; 256],
            regs_b,
            tone_phase: [0; 3],
            tone_level: [false; 3],
            noise_phase: 0,
            noise_lfsr: 0x1ffff,
            envelope_phase: 0,
            envelope_level: 0,
            envelope_up: true,
            sample_phase: 0,
            adpcm_a_phase: 0,
            adpcm_b_phase: 0,
            adpcm_a: [AdpcmAChannel::default(); 6],
            adpcm_b: AdpcmBChannel::default(),
            eos_status: 0,
            flag_mask: 0xbf,
            samples: Vec::with_capacity(1024),
            timer_a_phase: 0,
            timer_b_phase: 0,
            irq_pending: false,
        }
    }
}

impl Ym2610Foundation {
    const YM_CLOCK: u64 = 8_000_000;
    const SSG_CLOCK: u64 = Self::YM_CLOCK / 4;

    fn begin_frame(&mut self) {
        self.samples.clear();
    }

    fn read(&self, port: u16) -> u8 {
        match port & 3 {
            0 => u8::from(self.irq_pending),
            1 => {
                if self.address_a < 0x0e {
                    self.regs_a[usize::from(self.address_a)]
                } else {
                    0
                }
            }
            2 => self.eos_status & self.flag_mask,
            _ => 0,
        }
    }

    fn write(&mut self, port: u16, value: u8) {
        match port & 3 {
            0 => self.address_a = value,
            1 => {
                let address = self.address_a;
                if address == 0x10 {
                    let forced = (value | 0x20) & !0x40;
                    self.regs_a[0x10] = forced;
                    if forced & 0x01 != 0 {
                        self.adpcm_b.reset();
                    } else if forced & 0x80 != 0 {
                        let start =
                            u32::from(u16::from_le_bytes([self.regs_a[0x12], self.regs_a[0x13]]))
                                << 8;
                        self.adpcm_b.start(start);
                    }
                } else if address == 0x1c {
                    self.flag_mask = !value & 0xbf;
                    self.eos_status &= !(value & 0xbf);
                } else {
                    self.regs_a[usize::from(address)] = value;
                }
                if address == 0x0d {
                    self.envelope_level = if value & 0x04 != 0 { 0 } else { 15 };
                    self.envelope_up = value & 0x04 != 0;
                    self.envelope_phase = 0;
                }
                if address == 0x27 && value & 0x30 != 0 {
                    self.irq_pending = false;
                }
            }
            2 => self.address_b = value,
            _ => {
                let address = self.address_b;
                self.regs_b[usize::from(address)] = value;
                if address == 0x00 {
                    let key_on = value & 0x80 == 0;
                    for channel in 0..6 {
                        if value & (1 << channel) == 0 {
                            continue;
                        }
                        if key_on {
                            let start = u32::from(self.regs_b[0x10 + channel])
                                | (u32::from(self.regs_b[0x18 + channel]) << 8);
                            self.adpcm_a[channel].key_on(start << 8);
                        } else {
                            self.adpcm_a[channel].key_off();
                        }
                    }
                }
            }
        }
    }

    fn tone_period(&self, channel: usize) -> u64 {
        let low = u16::from(self.regs_a[channel * 2]);
        let high = u16::from(self.regs_a[channel * 2 + 1] & 0x0f);
        u64::from((low | (high << 8)).max(1))
    }

    fn envelope_period(&self) -> u64 {
        u64::from(u16::from_le_bytes([self.regs_a[0x0b], self.regs_a[0x0c]]).max(1))
    }

    fn advance_generators(&mut self) {
        for channel in 0..3 {
            self.tone_phase[channel] += Self::SSG_CLOCK;
            let threshold = self.tone_period(channel) * 16 * u64::from(AUDIO_RATE);
            while self.tone_phase[channel] >= threshold {
                self.tone_phase[channel] -= threshold;
                self.tone_level[channel] = !self.tone_level[channel];
            }
        }
        let noise_period = u64::from((self.regs_a[6] & 0x1f).max(1));
        self.noise_phase += Self::SSG_CLOCK;
        let noise_threshold = noise_period * 16 * u64::from(AUDIO_RATE);
        while self.noise_phase >= noise_threshold {
            self.noise_phase -= noise_threshold;
            let feedback = (self.noise_lfsr ^ (self.noise_lfsr >> 3)) & 1;
            self.noise_lfsr = (self.noise_lfsr >> 1) | (feedback << 16);
        }
        self.envelope_phase += Self::SSG_CLOCK;
        let envelope_threshold = self.envelope_period() * 256 * u64::from(AUDIO_RATE);
        while self.envelope_phase >= envelope_threshold {
            self.envelope_phase -= envelope_threshold;
            let shape = self.regs_a[0x0d] & 0x0f;
            if self.envelope_up {
                if self.envelope_level < 15 {
                    self.envelope_level += 1;
                } else if shape & 0x08 == 0 || shape & 0x01 != 0 {
                    self.envelope_up = false;
                } else if shape & 0x02 != 0 {
                    self.envelope_up = false;
                    self.envelope_level = 14;
                } else {
                    self.envelope_level = 0;
                }
            } else if self.envelope_level > 0 {
                self.envelope_level -= 1;
            } else if shape & 0x08 != 0 && shape & 0x01 == 0 {
                if shape & 0x02 != 0 {
                    self.envelope_up = true;
                    self.envelope_level = 1;
                } else {
                    self.envelope_level = 15;
                }
            }
        }
    }

    fn ssg_sample(&self) -> f32 {
        let mixer = self.regs_a[7];
        let noise = self.noise_lfsr & 1 != 0;
        let mut mix = 0.0f32;
        for channel in 0..3 {
            let tone_allowed = mixer & (1 << channel) == 0;
            let noise_allowed = mixer & (1 << (channel + 3)) == 0;
            let gate = (!tone_allowed || self.tone_level[channel]) && (!noise_allowed || noise);
            if !gate {
                continue;
            }
            let volume_reg = self.regs_a[8 + channel];
            let level = if volume_reg & 0x10 != 0 {
                self.envelope_level
            } else {
                volume_reg & 0x0f
            };
            mix += f32::from(level) / 15.0;
        }
        mix / 3.0 * 0.22
    }

    fn emit_sample(&mut self, adpcm_a_rom: &[u8], adpcm_b_rom: &[u8]) {
        self.advance_generators();
        self.adpcm_a_phase += Self::YM_CLOCK;
        let adpcm_a_threshold = 432 * u64::from(AUDIO_RATE);
        while self.adpcm_a_phase >= adpcm_a_threshold {
            self.adpcm_a_phase -= adpcm_a_threshold;
            for channel in 0..6 {
                let end = (u32::from(self.regs_b[0x20 + channel])
                    | (u32::from(self.regs_b[0x28 + channel]) << 8))
                    .wrapping_add(1)
                    << 8;
                if self.adpcm_a[channel].clock(adpcm_a_rom, end) {
                    self.eos_status |= 1 << channel;
                }
            }
        }
        self.adpcm_b_phase += Self::YM_CLOCK;
        let adpcm_b_threshold = 144 * u64::from(AUDIO_RATE);
        while self.adpcm_b_phase >= adpcm_b_threshold {
            self.adpcm_b_phase -= adpcm_b_threshold;
            self.adpcm_b.clock(&self.regs_a, adpcm_b_rom);
        }
        if self.adpcm_b.eos {
            self.eos_status |= 0x80;
        }
        let ssg = self.ssg_sample();
        let mut left = ssg;
        let mut right = ssg;
        for channel in 0..6 {
            let (l, r) = self.adpcm_a[channel].mix(self.regs_b[0x08 + channel], self.regs_b[0x01]);
            left += l;
            right += r;
        }
        let (l, r) = self.adpcm_b.mix(&self.regs_a);
        left += l;
        right += r;
        self.samples
            .push((left.clamp(-1.0, 1.0), right.clamp(-1.0, 1.0)));
    }

    fn tick(&mut self, z80_cycles: u32, adpcm_a_rom: &[u8], adpcm_b_rom: &[u8]) {
        self.sample_phase = self
            .sample_phase
            .saturating_add(u64::from(z80_cycles) * u64::from(AUDIO_RATE));
        while self.sample_phase >= Z80_CLOCK {
            self.sample_phase -= Z80_CLOCK;
            self.emit_sample(adpcm_a_rom, adpcm_b_rom);
        }
        if self.regs_a[0x27] & 0x01 != 0 {
            let period = 1024u64
                .saturating_sub(u64::from(
                    u16::from(self.regs_a[0x24]) << 2 | u16::from(self.regs_a[0x25] & 3),
                ))
                .max(1);
            self.timer_a_phase += u64::from(z80_cycles) * 2;
            if self.timer_a_phase >= period * 12 {
                self.timer_a_phase %= period * 12;
                self.irq_pending = true;
            }
        }
        if self.regs_a[0x27] & 0x02 != 0 {
            let period = 256u64.saturating_sub(u64::from(self.regs_a[0x26])).max(1);
            self.timer_b_phase += u64::from(z80_cycles) * 2;
            if self.timer_b_phase >= period * 192 {
                self.timer_b_phase %= period * 192;
                self.irq_pending = true;
            }
        }
    }

    fn save(&self, out: &mut StateWriter) {
        out.u8(self.address_a);
        out.u8(self.address_b);
        out.blob(&self.regs_a);
        out.blob(&self.regs_b);
        for phase in self.tone_phase {
            out.u64(phase);
        }
        for level in self.tone_level {
            out.u8(u8::from(level));
        }
        out.u64(self.noise_phase);
        out.u32(self.noise_lfsr);
        out.u64(self.envelope_phase);
        out.u8(self.envelope_level);
        out.u8(u8::from(self.envelope_up));
        out.u64(self.sample_phase);
        out.u64(self.adpcm_a_phase);
        out.u64(self.adpcm_b_phase);
        for channel in &self.adpcm_a {
            channel.save(out);
        }
        self.adpcm_b.save(out);
        out.u8(self.eos_status);
        out.u8(self.flag_mask);
        out.u64(self.timer_a_phase);
        out.u64(self.timer_b_phase);
        out.u8(u8::from(self.irq_pending));
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.address_a = input.u8()?;
        self.address_b = input.u8()?;
        let a = input.blob()?;
        if a.len() != 256 {
            return Err("Neo Geo YM2610 A register state is invalid".into());
        }
        self.regs_a.copy_from_slice(a);
        let b = input.blob()?;
        if b.len() != 256 {
            return Err("Neo Geo YM2610 B register state is invalid".into());
        }
        self.regs_b.copy_from_slice(b);
        for phase in &mut self.tone_phase {
            *phase = input.u64()?;
        }
        for level in &mut self.tone_level {
            *level = input.u8()? != 0;
        }
        self.noise_phase = input.u64()?;
        self.noise_lfsr = input.u32()? & 0x1ffff;
        self.envelope_phase = input.u64()?;
        self.envelope_level = input.u8()?.min(15);
        self.envelope_up = input.u8()? != 0;
        self.sample_phase = input.u64()? % Z80_CLOCK;
        self.adpcm_a_phase = input.u64()? % (432 * u64::from(AUDIO_RATE));
        self.adpcm_b_phase = input.u64()? % (144 * u64::from(AUDIO_RATE));
        for channel in &mut self.adpcm_a {
            channel.load(input)?;
        }
        self.adpcm_b.load(input)?;
        self.eos_status = input.u8()?;
        self.flag_mask = input.u8()? & 0xbf;
        self.timer_a_phase = input.u64()?;
        self.timer_b_phase = input.u64()?;
        self.irq_pending = input.u8()? != 0;
        self.samples.clear();
        Ok(())
    }
}

struct NeoZ80Bus<'a> {
    mrom: &'a [u8],
    ram: &'a mut [u8; 0x800],
    banks: &'a mut [u32; 4],
    command: &'a mut u8,
    command_pending: &'a mut bool,
    reply: &'a mut u8,
    nmi_enabled: &'a mut bool,
    ym: &'a mut Ym2610Foundation,
}

impl NeoZ80Bus<'_> {
    fn banked_read(&self, address: u16) -> u8 {
        let (slot, window_start, mask) = match address {
            0x8000..=0xbfff => (3, 0x8000u16, 0x3fffu32),
            0xc000..=0xdfff => (2, 0xc000u16, 0x1fffu32),
            0xe000..=0xefff => (1, 0xe000u16, 0x0fffu32),
            0xf000..=0xf7ff => (0, 0xf000u16, 0x07ffu32),
            _ => return 0xff,
        };
        let offset =
            self.banks[slot].wrapping_add(u32::from(address - window_start) & mask) as usize;
        self.mrom.get(offset).copied().unwrap_or(0xff)
    }
}

impl Z80Bus for NeoZ80Bus<'_> {
    fn mem_read(&mut self, address: u16) -> u8 {
        match address {
            0x0000..=0x7fff => self.mrom.get(usize::from(address)).copied().unwrap_or(0xff),
            0x8000..=0xf7ff => self.banked_read(address),
            0xf800..=0xffff => self.ram[usize::from(address & 0x07ff)],
        }
    }

    fn mem_write(&mut self, address: u16, value: u8) {
        if address >= 0xf800 {
            self.ram[usize::from(address & 0x07ff)] = value;
        }
    }

    fn io_read(&mut self, port: u16) -> u8 {
        match port as u8 {
            0x00 => {
                *self.command_pending = false;
                *self.command
            }
            0x04..=0x07 => self.ym.read(port),
            0x08..=0x0b => {
                let slot = usize::from((port as u8) - 0x08);
                let width = [0x0800u32, 0x1000, 0x2000, 0x4000][slot];
                self.banks[slot] = u32::from(port >> 8).saturating_mul(width);
                0
            }
            0x0c => *self.reply,
            _ => 0xff,
        }
    }

    fn io_write(&mut self, port: u16, value: u8) {
        match port as u8 {
            0x04..=0x07 => self.ym.write(port, value),
            0x08 => *self.nmi_enabled = true,
            0x18 => *self.nmi_enabled = false,
            0x0c => *self.reply = value,
            _ => {}
        }
    }
}

struct NeoSound {
    cpu: Z80,
    ram: [u8; 0x800],
    banks: [u32; 4],
    command: u8,
    command_pending: bool,
    reply: u8,
    nmi_enabled: bool,
    nmi_queued: bool,
    ym: Ym2610Foundation,
    cycle_phase: u64,
}

impl Default for NeoSound {
    fn default() -> Self {
        Self {
            cpu: Z80::default(),
            ram: [0; 0x800],
            banks: [0; 4],
            command: 0,
            command_pending: false,
            reply: 0,
            nmi_enabled: true,
            nmi_queued: false,
            ym: Ym2610Foundation::default(),
            cycle_phase: 0,
        }
    }
}

impl NeoSound {
    fn queue_command(&mut self, command: u8) {
        self.command = command;
        self.command_pending = true;
        self.nmi_queued = true;
    }

    fn run_for_68k_cycles(&mut self, cycles_68k: u32, cart: &NeoCartridge) {
        self.cycle_phase += u64::from(cycles_68k) * Z80_CLOCK;
        let budget = self.cycle_phase / M68K_CLOCK;
        self.cycle_phase %= M68K_CLOCK;
        let target = self.cpu.cycles.saturating_add(budget);
        let (cpu, ram, banks, command, command_pending, reply, nmi_enabled, nmi_queued, ym) = (
            &mut self.cpu,
            &mut self.ram,
            &mut self.banks,
            &mut self.command,
            &mut self.command_pending,
            &mut self.reply,
            &mut self.nmi_enabled,
            &mut self.nmi_queued,
            &mut self.ym,
        );
        let mut bus = NeoZ80Bus {
            mrom: &cart.m,
            ram,
            banks,
            command,
            command_pending,
            reply,
            nmi_enabled,
            ym,
        };
        if *nmi_queued && *bus.nmi_enabled {
            let cycles = cpu.nmi(&mut bus);
            bus.ym.tick(cycles, &cart.v1, cart.adpcm_b_rom());
            *nmi_queued = false;
        }
        while cpu.cycles < target {
            let cycles = if bus.ym.irq_pending {
                cpu.irq(&mut bus, 0xff)
            } else {
                0
            };
            let cycles = if cycles == 0 {
                cpu.step(&mut bus)
            } else {
                cycles
            };
            bus.ym.tick(cycles, &cart.v1, cart.adpcm_b_rom());
        }
    }

    fn begin_frame(&mut self) {
        self.ym.begin_frame();
    }

    fn save(&self, out: &mut StateWriter) {
        self.cpu.save(out);
        out.blob(&self.ram);
        for bank in self.banks {
            out.u32(bank);
        }
        out.u8(self.command);
        out.u8(u8::from(self.command_pending));
        out.u8(self.reply);
        out.u8(u8::from(self.nmi_enabled));
        out.u8(u8::from(self.nmi_queued));
        self.ym.save(out);
        out.u64(self.cycle_phase);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.cpu.load(input)?;
        let ram = input.blob()?;
        if ram.len() != self.ram.len() {
            return Err("Neo Geo Z80 RAM state is invalid".into());
        }
        self.ram.copy_from_slice(ram);
        for bank in &mut self.banks {
            *bank = input.u32()?;
        }
        self.command = input.u8()?;
        self.command_pending = input.u8()? != 0;
        self.reply = input.u8()?;
        self.nmi_enabled = input.u8()? != 0;
        self.nmi_queued = input.u8()? != 0;
        self.ym.load(input)?;
        self.cycle_phase = input.u64()? % M68K_CLOCK;
        Ok(())
    }
}

struct NeoBus {
    cart: NeoCartridge,
    bios: Box<[u8; SYSTEM_ROM_SIZE]>,
    work_ram: Box<[u8; 0x10000]>,
    backup_ram: Box<[u8; 0x10000]>,
    memory_card: [u8; 0x800],
    video: NeoVideo,
    sound: NeoSound,
    bios_vectors: bool,
    bank_base: usize,
    input: InputState,
}

impl NeoBus {
    fn new(cart: NeoCartridge, bios: &[u8]) -> Result<Self, String> {
        if bios.len() != SYSTEM_ROM_SIZE {
            return Err(format!(
                "Neo Geo BIOS must be exactly {SYSTEM_ROM_SIZE} bytes, got {}",
                bios.len()
            ));
        }
        let mut bios_image = Box::new([0; SYSTEM_ROM_SIZE]);
        bios_image.copy_from_slice(bios);
        Ok(Self {
            cart,
            bios: bios_image,
            work_ram: Box::new([0; 0x10000]),
            backup_ram: Box::new([0; 0x10000]),
            memory_card: [0xff; 0x800],
            video: NeoVideo::default(),
            sound: NeoSound::default(),
            bios_vectors: true,
            bank_base: 0x100000,
            input: InputState::default(),
        })
    }

    fn set_input(&mut self, input: &InputState) {
        self.input = input.clone();
    }

    fn p_byte(&self, address: usize) -> u8 {
        self.cart.p.get(address).copied().unwrap_or(0xff)
    }

    fn input_port(&self, port: u32) -> u8 {
        let buttons = self.input.buttons[0];
        let mut value = 0xffu8;
        match port {
            0x300000 => {
                if buttons & UP != 0 {
                    value &= !0x01;
                }
                if buttons & DOWN != 0 {
                    value &= !0x02;
                }
                if buttons & LEFT != 0 {
                    value &= !0x04;
                }
                if buttons & RIGHT != 0 {
                    value &= !0x08;
                }
                if buttons & FACE_SOUTH != 0 {
                    value &= !0x10;
                }
                if buttons & FACE_EAST != 0 {
                    value &= !0x20;
                }
                if buttons & FACE_WEST != 0 {
                    value &= !0x40;
                }
                if buttons & FACE_NORTH != 0 {
                    value &= !0x80;
                }
            }
            0x300001 if buttons & START != 0 => {
                value &= !0x01;
            }
            _ => {}
        }
        value
    }

    fn switch_latch(&mut self, address: u32) {
        match address {
            0x3a0003 => self.bios_vectors = true,
            0x3a0013 => self.bios_vectors = false,
            0x3a000b => self.video.cart_fix = false,
            0x3a001b => self.video.cart_fix = true,
            0x3a000f => self.video.palette_bank = 0,
            0x3a001f => self.video.palette_bank = 1,
            0x3a0001 => self.video.shadow = true,
            0x3a0011 => self.video.shadow = false,
            _ => {}
        }
    }

    fn read_word_direct(&mut self, address: u32) -> Option<u16> {
        match address {
            0x320000 => {
                Some(u16::from(self.sound.reply) << 8 | u16::from(!self.sound.command_pending))
            }
            0x3c0000..=0x3c000f => Some(self.video.read_register(address)),
            0x400000..=0x401fff => {
                let offset = address as usize - 0x400000;
                let high = self.video.palette_read8(offset);
                let low = self.video.palette_read8(offset + 1);
                Some(u16::from_be_bytes([high, low]))
            }
            _ => None,
        }
    }

    fn write_word_direct(&mut self, address: u32, value: u16) -> bool {
        match address {
            0x320000 => {
                self.sound.queue_command((value >> 8) as u8);
                true
            }
            0x3c0000..=0x3c000f => {
                self.video.write_register(address, value);
                true
            }
            0x400000..=0x401fff => {
                self.video
                    .palette_write16(address as usize - 0x400000, value);
                true
            }
            _ => false,
        }
    }

    fn tick(&mut self, cycles: u32) {
        self.video.tick(cycles, &self.cart);
        self.sound.run_for_68k_cycles(cycles, &self.cart);
    }

    fn save(&self, out: &mut StateWriter) {
        out.blob(self.work_ram.as_ref());
        out.blob(self.backup_ram.as_ref());
        out.blob(&self.memory_card);
        self.video.save(out);
        self.sound.save(out);
        out.u8(u8::from(self.bios_vectors));
        out.u32(self.bank_base as u32);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let work = input.blob()?;
        if work.len() != self.work_ram.len() {
            return Err("Neo Geo work RAM state is invalid".into());
        }
        self.work_ram.copy_from_slice(work);
        let backup = input.blob()?;
        if backup.len() != self.backup_ram.len() {
            return Err("Neo Geo backup RAM state is invalid".into());
        }
        self.backup_ram.copy_from_slice(backup);
        let card = input.blob()?;
        if card.len() != self.memory_card.len() {
            return Err("Neo Geo memory card state is invalid".into());
        }
        self.memory_card.copy_from_slice(card);
        self.video.load(input, &self.cart)?;
        self.sound.load(input)?;
        self.bios_vectors = input.u8()? != 0;
        self.bank_base = input.u32()? as usize;
        Ok(())
    }
}

impl Bus68000 for NeoBus {
    fn read8(&mut self, address: u32) -> u8 {
        let address = address & 0x00ff_ffff;
        match address {
            0x000000..=0x00007f if self.bios_vectors => self.bios[address as usize],
            0x000000..=0x0fffff => self.p_byte(address as usize),
            0x100000..=0x10ffff => self.work_ram[address as usize & 0xffff],
            0x200000..=0x2fffff => self.p_byte(self.bank_base + (address as usize & 0x0fffff)),
            0x300000..=0x300001 => self.input_port(address),
            0x320000 => self.sound.reply,
            0x320001 => u8::from(!self.sound.command_pending),
            0x3c0000..=0x3c000f => {
                let word = self.video.read_register(address & !1);
                if address & 1 == 0 {
                    (word >> 8) as u8
                } else {
                    word as u8
                }
            }
            0x400000..=0x401fff => self.video.palette_read8(address as usize - 0x400000),
            0x800000..=0x800fff => self.memory_card[(address as usize >> 1) & 0x07ff],
            0xc00000..=0xc1ffff => self.bios[(address as usize - 0xc00000) & (SYSTEM_ROM_SIZE - 1)],
            0xd00000..=0xd0ffff => self.backup_ram[address as usize & 0xffff],
            _ => 0xff,
        }
    }

    fn write8(&mut self, address: u32, value: u8) {
        let address = address & 0x00ff_ffff;
        match address {
            0x100000..=0x10ffff => self.work_ram[address as usize & 0xffff] = value,
            0x2ffff0..=0x2fffff => self.bank_base = 0x100000 + (usize::from(value) * 0x100000),
            0x320000 => self.sound.queue_command(value),
            0x3a0000..=0x3a001f => self.switch_latch(address),
            0x400000..=0x401fff => self
                .video
                .palette_write8(address as usize - 0x400000, value),
            0x800000..=0x800fff => self.memory_card[(address as usize >> 1) & 0x07ff] = value,
            0xd00000..=0xd0ffff => self.backup_ram[address as usize & 0xffff] = value,
            _ => {}
        }
    }

    fn read16(&mut self, address: u32) -> u16 {
        if let Some(value) = self.read_word_direct(address & !1) {
            value
        } else {
            u16::from_be_bytes([self.read8(address), self.read8(address.wrapping_add(1))])
        }
    }

    fn write16(&mut self, address: u32, value: u16) {
        if !self.write_word_direct(address & !1, value) {
            let [high, low] = value.to_be_bytes();
            self.write8(address, high);
            self.write8(address.wrapping_add(1), low);
        }
    }

    fn read32(&mut self, address: u32) -> u32 {
        (u32::from(self.read16(address)) << 16) | u32::from(self.read16(address.wrapping_add(2)))
    }

    fn write32(&mut self, address: u32, value: u32) {
        self.write16(address, (value >> 16) as u16);
        self.write16(address.wrapping_add(2), value as u16);
    }
}

pub struct NeoGeoMachine {
    main: M68000,
    bus: NeoBus,
    audio: AudioBuffer,
}

impl NeoGeoMachine {
    pub fn from_images(game: &[u8], bios: &[u8]) -> Result<Self, String> {
        let cart = NeoCartridge::from_neo(game)?;
        let mut bus = NeoBus::new(cart, bios)?;
        let mut main = M68000::default();
        main.reset(&mut bus);
        Ok(Self {
            main,
            bus,
            audio: AudioBuffer::new(AUDIO_RATE, 2),
        })
    }

    fn flush_audio(&mut self) {
        self.audio.begin_frame();
        for &(left, right) in &self.bus.sound.ym.samples {
            self.audio.push_stereo(left, right);
        }
    }
}

impl Machine for NeoGeoMachine {
    fn platform(&self) -> PlatformId {
        PlatformId::NeoGeo
    }

    fn reset(&mut self) {
        let cart = self.bus.cart.clone();
        let bios = self.bus.bios.to_vec();
        self.bus = NeoBus::new(cart, &bios).expect("existing Neo Geo images must remain valid");
        self.main.reset(&mut self.bus);
        self.audio.begin_frame();
    }

    fn run_frame(&mut self, input: &InputState) {
        self.bus.set_input(input);
        self.bus.sound.begin_frame();
        let target = self.bus.video.frame.wrapping_add(1);
        let mut instructions = 0usize;
        while self.bus.video.frame != target && instructions < 2_000_000 {
            let cycles = self.main.step(&mut self.bus);
            let cycles = if cycles == 0 { 4 } else { cycles };
            self.bus.tick(cycles);
            let level = self.bus.video.irq_level();
            if level != 0 {
                let irq_cycles = self.main.interrupt(&mut self.bus, level, 24 + level);
                if irq_cycles != 0 {
                    self.bus.tick(irq_cycles);
                }
            }
            instructions += 1;
        }
        self.flush_audio();
    }

    fn frame_rate(&self) -> f64 {
        FRAME_RATE
    }
    fn video(&self) -> &VideoBuffer {
        &self.bus.video.video
    }
    fn audio(&self) -> &AudioBuffer {
        &self.audio
    }

    fn save_state(&self) -> Result<Vec<u8>, String> {
        let mut out = StateWriter::new(PlatformId::NeoGeo, STATE_VERSION);
        self.main.save(&mut out);
        self.bus.save(&mut out);
        Ok(out.finish())
    }

    fn load_state(&mut self, bytes: &[u8]) -> Result<(), String> {
        let mut input = StateReader::new(bytes, PlatformId::NeoGeo, STATE_VERSION)?;
        self.main.load(&mut input)?;
        self.bus.load(&mut input)?;
        input.finish()?;
        self.audio.begin_frame();
        Ok(())
    }

    fn persistent_len(&self, kind: crate::resources::ResourceKind, slot: u32) -> usize {
        if slot != 0 {
            return 0;
        }
        match kind {
            crate::resources::ResourceKind::Storage => self.bus.backup_ram.len(),
            crate::resources::ResourceKind::MemoryCard => self.bus.memory_card.len(),
            _ => 0,
        }
    }

    fn read_persistent(
        &self,
        kind: crate::resources::ResourceKind,
        slot: u32,
        out: &mut [u8],
    ) -> Result<(), String> {
        let source: &[u8] = match (kind, slot) {
            (crate::resources::ResourceKind::Storage, 0) => self.bus.backup_ram.as_ref(),
            (crate::resources::ResourceKind::MemoryCard, 0) => &self.bus.memory_card,
            _ => return Err("Neo Geo persistent resource slot is not supported".into()),
        };
        if out.len() != source.len() {
            return Err("Neo Geo persistent resource length does not match".into());
        }
        out.copy_from_slice(source);
        Ok(())
    }

    fn write_persistent(
        &mut self,
        kind: crate::resources::ResourceKind,
        slot: u32,
        data: &[u8],
    ) -> Result<(), String> {
        let target: &mut [u8] = match (kind, slot) {
            (crate::resources::ResourceKind::Storage, 0) => self.bus.backup_ram.as_mut(),
            (crate::resources::ResourceKind::MemoryCard, 0) => &mut self.bus.memory_card,
            _ => return Err("Neo Geo persistent resource slot is not supported".into()),
        };
        if data.len() != target.len() {
            return Err("Neo Geo persistent resource length does not match".into());
        }
        target.copy_from_slice(data);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::resources::ResourceKind;

    fn synthetic_mrom() -> Vec<u8> {
        let mut m = vec![0; 0x10000];
        let program = [
            0x3e, 0x00, 0xd3, 0x04, 0x3e, 0x20, 0xd3, 0x05, 0x3e, 0x01, 0xd3, 0x04, 0x3e, 0x00,
            0xd3, 0x05, 0x3e, 0x07, 0xd3, 0x04, 0x3e, 0x3e, 0xd3, 0x05, 0x3e, 0x08, 0xd3, 0x04,
            0x3e, 0x0f, 0xd3, 0x05, 0x76,
        ];
        m[..program.len()].copy_from_slice(&program);
        m[0x66..0x6d].copy_from_slice(&[0xdb, 0x00, 0xd3, 0x0c, 0xed, 0x45, 0x00]);
        m
    }

    fn synthetic_neo() -> Vec<u8> {
        let mut p_rom = vec![0x4e, 0x71, 0x60, 0xfc];
        p_rom.resize(0x10000, 0xff);
        p_rom[..4].copy_from_slice(&0x00aa55ccu32.to_be_bytes());
        let mut s_rom = vec![0u8; 0x10000];
        s_rom[32..64].fill(0xff);
        let m_rom = synthetic_mrom();
        let v1 = vec![0u8; 0x10000];
        let v2 = Vec::<u8>::new();
        let c_rom = vec![0u8; 0x40000];
        let sections: [&[u8]; 6] = [&p_rom, &s_rom, &m_rom, &v1, &v2, &c_rom];
        let mut output = vec![0u8; NEO_HEADER];
        output[..4].copy_from_slice(b"NEO\x01");
        for (index, section) in sections.iter().enumerate() {
            output[4 + index * 4..8 + index * 4]
                .copy_from_slice(&(section.len() as u32).to_le_bytes());
        }
        output[40..44].copy_from_slice(&0x0123u32.to_le_bytes());
        for section in sections {
            output.extend_from_slice(section);
        }
        output
    }

    fn synthetic_bios() -> Vec<u8> {
        let mut bios = vec![0x4e; SYSTEM_ROM_SIZE];
        bios[0..4].copy_from_slice(&0x0010ff00u32.to_be_bytes());
        bios[4..8].copy_from_slice(&0x00c00100u32.to_be_bytes());
        for vector in 24..=27usize {
            bios[vector * 4..vector * 4 + 4].copy_from_slice(&0x00c00100u32.to_be_bytes());
        }
        bios[0x100..0x104].copy_from_slice(&[0x4e, 0x71, 0x60, 0xfc]);
        bios
    }

    #[test]
    fn neo1_parser_splits_sections_and_metadata() {
        let image = synthetic_neo();
        let cart = NeoCartridge::from_neo(&image).unwrap();
        assert_eq!(cart.p.len(), 0x10000);
        assert_eq!(cart.s.len(), 0x10000);
        assert_eq!(cart.m.len(), 0x10000);
        assert_eq!(cart.v1.len(), 0x10000);
        assert!(cart.v2.is_empty());
        assert_eq!(cart.c.len(), 0x40000);
        assert_eq!(NeoCartridge::u32le(&image, 40).unwrap(), 0x0123);
        assert!(NeoCartridge::from_neo(b"bad").is_err());
    }

    #[test]
    fn bios_and_cartridge_vector_latch_switches_low_vectors() {
        let cart = NeoCartridge::from_neo(&synthetic_neo()).unwrap();
        let mut bus = NeoBus::new(cart, &synthetic_bios()).unwrap();
        assert_eq!(bus.read32(0), 0x0010ff00);
        bus.write8(0x3a0013, 0);
        assert_eq!(bus.read32(0), 0x00aa55cc);
        assert_eq!(bus.read32(0xc00000), 0x0010ff00);
    }

    #[test]
    fn sound_command_nmi_replies_and_ssg_generates_audio() {
        let cart = NeoCartridge::from_neo(&synthetic_neo()).unwrap();
        let mut sound = NeoSound::default();
        sound.begin_frame();
        sound.run_for_68k_cycles(5_000, &cart);
        assert!(sound.ym.samples.iter().any(|&(l, r)| l != 0.0 || r != 0.0));
        sound.queue_command(0x5a);
        sound.run_for_68k_cycles(1_000, &cart);
        assert_eq!(sound.reply, 0x5a);
        assert!(!sound.command_pending);
    }

    #[test]
    fn ym2610_adpcm_a_consumes_v1_samples_and_reports_end() {
        let mut ym = Ym2610Foundation::default();
        let v1 = vec![0x17; 0x100];
        let empty = Vec::<u8>::new();

        for (address, value) in [
            (0x01, 0x3f),
            (0x08, 0xdf),
            (0x10, 0x00),
            (0x18, 0x00),
            (0x20, 0x00),
            (0x28, 0x00),
            (0x00, 0x01),
        ] {
            ym.write(2, address);
            ym.write(3, value);
        }
        ym.begin_frame();
        ym.tick(120_000, &v1, &empty);

        assert!(ym
            .samples
            .iter()
            .any(|&(left, right)| left.abs() > 0.0001 || right.abs() > 0.0001));
        assert_ne!(ym.eos_status & 0x01, 0);
        assert!(!ym.adpcm_a[0].playing);
    }

    #[test]
    fn ym2610_adpcm_b_consumes_v2_samples_and_reports_end() {
        let mut ym = Ym2610Foundation::default();
        let empty = Vec::<u8>::new();
        let v2 = vec![0x17; 0x100];

        for (address, value) in [
            (0x11, 0xc0),
            (0x12, 0x00),
            (0x13, 0x00),
            (0x14, 0x00),
            (0x15, 0x00),
            (0x19, 0xff),
            (0x1a, 0xff),
            (0x1b, 0xff),
            (0x10, 0x80),
        ] {
            ym.write(0, address);
            ym.write(1, value);
        }
        ym.begin_frame();
        ym.tick(80_000, &empty, &v2);

        assert!(ym
            .samples
            .iter()
            .any(|&(left, right)| left.abs() > 0.0001 || right.abs() > 0.0001));
        assert_ne!(ym.eos_status & 0x80, 0);
        assert!(!ym.adpcm_b.playing);
    }

    #[test]
    fn fix_layer_and_palette_render_inside_machine_frame() {
        let mut machine = NeoGeoMachine::from_images(&synthetic_neo(), &synthetic_bios()).unwrap();
        machine.bus.video.palette[0][0xfff] = 0;
        machine.bus.video.palette[0][0x01f] = 0x0f00;
        machine.bus.video.vram[0x7002] = 0x1001;
        machine.run_frame(&InputState::default());
        assert_eq!(machine.video().width(), WIDTH);
        assert_eq!(machine.video().height(), HEIGHT);
        assert!(machine
            .video()
            .pixels()
            .as_chunks::<4>()
            .0
            .iter()
            .any(|pixel| pixel[0] > pixel[1] && pixel[0] > pixel[2]));
        assert!(machine
            .audio()
            .samples()
            .iter()
            .any(|sample| sample.abs() > 0.0001));
    }

    #[test]
    fn persistent_storage_and_state_round_trip_are_deterministic() {
        let mut machine = NeoGeoMachine::from_images(&synthetic_neo(), &synthetic_bios()).unwrap();
        machine.run_frame(&InputState::default());
        machine.bus.work_ram[7] = 0x44;
        machine.bus.backup_ram[9] = 0x55;
        machine.bus.memory_card[11] = 0x66;
        let saved = machine.save_state().unwrap();
        machine.bus.work_ram[7] = 0;
        machine.load_state(&saved).unwrap();
        assert_eq!(machine.bus.work_ram[7], 0x44);
        assert_eq!(machine.bus.backup_ram[9], 0x55);
        assert_eq!(machine.bus.memory_card[11], 0x66);
        assert_eq!(machine.save_state().unwrap(), saved);
        assert_eq!(machine.persistent_len(ResourceKind::Storage, 0), 0x10000);
        assert_eq!(machine.persistent_len(ResourceKind::MemoryCard, 0), 0x800);
    }
}
