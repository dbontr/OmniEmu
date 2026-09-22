use crate::cd_image::{DiscImage, TrackKind};
use crate::cpu_huc6280::{HuC6280, Huc6280Bus, HucInterrupt};
use crate::input::{
    DOWN, FACE_EAST, FACE_NORTH, FACE_SOUTH, FACE_WEST, L1, LEFT, R1, RIGHT, SELECT, START, UP,
};
use crate::kernel::{AudioBuffer, InputState, VideoBuffer};
use crate::machine::Machine;
use crate::platform::PlatformId;
use crate::resources::{ResourceBlob, ResourceKind, RESOURCE_PENDING};
use crate::state::{StateReader, StateWriter};

const WIDTH: u32 = 256;
const HEIGHT: u32 = 240;
const SCANLINES: u32 = 262;
const CPU_CLOCK_HZ: u64 = 7_159_090;
const FRAME_RATE_NUM: u64 = 59_826;
const FRAME_RATE_DEN: u64 = 1_000;
const FRAME_RATE: f64 = FRAME_RATE_NUM as f64 / FRAME_RATE_DEN as f64;
const AUDIO_RATE: u32 = 48_000;
const CDDA_RATE: u32 = 44_100;
const CDDA_FRAMES_PER_SECTOR: usize = 588;
const CDDA_OFF: u8 = 0;
const CDDA_PLAYING: u8 = 1;
const CDDA_PAUSED: u8 = 2;
const STATE_VERSION: u32 = 10;
const STANDARD_HUCARD_MAX: usize = 1024 * 1024;
const SF2_HUCARD_SIZE: usize = 0x280000;
const POPULOUS_HUCARD_CRC32: u32 = 0x083c_956a;
const POPULOUS_RAM_SIZE: usize = 32 * 1024;
const MAX_HUCARD: usize = SF2_HUCARD_SIZE + 512;
const SYSTEM_CARD_SIZE: usize = 256 * 1024;
const CD_RAM_SIZE: usize = 64 * 1024;
const SUPER_CD_RAM_SIZE: usize = 192 * 1024;
const BACKUP_RAM_SIZE: usize = 2 * 1024;
const ADPCM_RAM_SIZE: usize = 64 * 1024;
const ARCADE_CARD_RAM_SIZE: usize = 2 * 1024 * 1024;
const PCE_CD_CLOCK_HZ: u64 = 9_216_000;
const ADPCM_NIBBLE_RATE: u64 = PCE_CD_CLOCK_HZ / 6 / 48;
const ADPCM_PLAY_FLAG: u8 = 0x08;
const ADPCM_STOP_FLAG: u8 = 0x01;
const CD_IRQ_TRANSFER_READY: u8 = 0x40;
const CD_IRQ_TRANSFER_DONE: u8 = 0x20;
const CD_IRQ_SAMPLE_FULL_PLAY: u8 = 0x08;
const MIX_GAIN_ONE: u32 = 1 << 16;

fn pce_color(value: u16) -> [u8; 4] {
    let r = ((value >> 3) & 7) as u8 * 36;
    let g = (value & 7) as u8 * 36;
    let b = ((value >> 6) & 7) as u8 * 36;
    [r, g, b, 255]
}

struct HuC6260 {
    control: u8,
    address: u16,
    colors: [u16; 512],
}

impl Default for HuC6260 {
    fn default() -> Self {
        Self {
            control: 0,
            address: 0,
            colors: [0; 512],
        }
    }
}

impl HuC6260 {
    fn read(&mut self, port: u32) -> u8 {
        match port & 7 {
            0 => self.control,
            2 => self.address as u8,
            3 => (self.address >> 8) as u8,
            4 => self.colors[usize::from(self.address & 0x01ff)] as u8,
            5 => {
                let value = (self.colors[usize::from(self.address & 0x01ff)] >> 8) as u8;
                self.address = (self.address + 1) & 0x01ff;
                value
            }
            _ => 0xff,
        }
    }

    fn write(&mut self, port: u32, value: u8) {
        match port & 7 {
            0 => self.control = value,
            2 => self.address = (self.address & 0x0100) | u16::from(value),
            3 => self.address = (self.address & 0x00ff) | (u16::from(value & 1) << 8),
            4 => {
                let slot = &mut self.colors[usize::from(self.address & 0x01ff)];
                *slot = (*slot & 0x0100) | u16::from(value);
            }
            5 => {
                let slot = &mut self.colors[usize::from(self.address & 0x01ff)];
                *slot = (*slot & 0x00ff) | (u16::from(value & 1) << 8);
                self.address = (self.address + 1) & 0x01ff;
            }
            _ => {}
        }
    }

    fn rgba(&self, index: usize) -> [u8; 4] {
        pce_color(self.colors[index & 0x01ff])
    }

    fn save(&self, out: &mut StateWriter) {
        out.u8(self.control);
        out.u16(self.address);
        for color in self.colors {
            out.u16(color);
        }
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.control = input.u8()?;
        self.address = input.u16()? & 0x01ff;
        for color in &mut self.colors {
            *color = input.u16()? & 0x01ff;
        }
        Ok(())
    }
}

struct HuC6270 {
    vram: Box<[u16; 0x8000]>,
    satb: [u16; 256],
    regs: [u16; 20],
    select: u8,
    low_latch: u8,
    read_latch: u16,
    status: u8,
    scanline: u16,
    line_phase: u128,
    frame: u64,
    irq_pending: bool,
    video: VideoBuffer,
    pixel_codes: Vec<u16>,
}

impl Default for HuC6270 {
    fn default() -> Self {
        Self {
            vram: Box::new([0; 0x8000]),
            satb: [0; 256],
            regs: [0; 20],
            select: 0,
            low_latch: 0,
            read_latch: 0,
            status: 0,
            scanline: 0,
            line_phase: 0,
            frame: 0,
            irq_pending: false,
            video: VideoBuffer::new(WIDTH, HEIGHT),
            pixel_codes: vec![0; (WIDTH * HEIGHT) as usize],
        }
    }
}

impl HuC6270 {
    fn vram_increment(&self) -> u16 {
        match (self.regs[5] >> 11) & 3 {
            0 => 1,
            1 => 32,
            2 => 64,
            _ => 128,
        }
    }

    fn update_irq(&mut self) {
        let cr = self.regs[5];
        self.irq_pending = (self.status & 0x20 != 0 && cr & 0x0008 != 0)
            || (self.status & 0x04 != 0 && cr & 0x0004 != 0)
            || (self.status & 0x02 != 0 && cr & 0x0002 != 0)
            || (self.status & 0x01 != 0 && cr & 0x0001 != 0)
            || (self.status & 0x18 != 0 && self.regs[15] & 0x0003 != 0);
    }

    fn read(&mut self, port: u32) -> u8 {
        match port & 3 {
            0 => {
                let status = self.status;
                self.status &= 0x40;
                self.update_irq();
                status
            }
            2 => {
                let address = usize::from(self.regs[1] & 0x7fff);
                self.read_latch = self.vram[address];
                self.read_latch as u8
            }
            3 => {
                let value = (self.read_latch >> 8) as u8;
                self.regs[1] = self.regs[1].wrapping_add(self.vram_increment()) & 0x7fff;
                value
            }
            _ => 0xff,
        }
    }

    fn write(&mut self, port: u32, value: u8) {
        match port & 3 {
            0 => self.select = value & 0x1f,
            2 => self.low_latch = value,
            3 => {
                let word = u16::from_le_bytes([self.low_latch, value]);
                self.write_register(self.select, word);
            }
            _ => {}
        }
    }

    fn write_register(&mut self, register: u8, value: u16) {
        let index = usize::from(register);
        if index >= self.regs.len() {
            return;
        }
        self.regs[index] = value;
        match register {
            0 | 1 => self.regs[index] &= 0x7fff,
            2 => {
                let address = usize::from(self.regs[0] & 0x7fff);
                self.vram[address] = value;
                self.regs[0] = self.regs[0].wrapping_add(self.vram_increment()) & 0x7fff;
            }
            18 => self.vram_dma(),
            19 => self.satb_dma(),
            _ => {}
        }
        self.update_irq();
    }

    fn vram_dma(&mut self) {
        let mut source = self.regs[16] & 0x7fff;
        let mut destination = self.regs[17] & 0x7fff;
        let count = u32::from(self.regs[18]) + 1;
        let source_delta: i32 = if self.regs[15] & 0x0004 != 0 { -1 } else { 1 };
        let destination_delta: i32 = if self.regs[15] & 0x0008 != 0 { -1 } else { 1 };
        for _ in 0..count {
            self.vram[usize::from(destination)] = self.vram[usize::from(source)];
            source = ((i32::from(source) + source_delta) & 0x7fff) as u16;
            destination = ((i32::from(destination) + destination_delta) & 0x7fff) as u16;
        }
        self.regs[16] = source;
        self.regs[17] = destination;
        self.regs[18] = 0xffff;
        self.status |= 0x10;
        self.update_irq();
    }

    fn satb_dma(&mut self) {
        let source = usize::from(self.regs[19] & 0x7fff);
        for index in 0..256 {
            self.satb[index] = self.vram[(source + index) & 0x7fff];
        }
        self.status |= 0x08;
        self.update_irq();
    }

    fn bat_dimensions(&self) -> (usize, usize) {
        let width = match (self.regs[9] >> 4) & 3 {
            0 => 32,
            1 => 64,
            _ => 128,
        };
        let height = if self.regs[9] & 0x0040 != 0 { 64 } else { 32 };
        (width, height)
    }

    fn background_pixel(&self, x: usize, y: usize) -> (u8, usize) {
        let (bat_width, bat_height) = self.bat_dimensions();
        let world_x = (x + usize::from(self.regs[7] & 0x03ff)) % (bat_width * 8);
        let world_y = (y + usize::from(self.regs[8] & 0x03ff)) % (bat_height * 8);
        let tile_x = world_x / 8;
        let tile_y = world_y / 8;
        let entry = self.vram[(tile_y * bat_width + tile_x) & 0x7fff];
        let tile = usize::from(entry & 0x0fff);
        let palette = usize::from((entry >> 12) & 0x0f);
        let row = world_y & 7;
        let bit = 7 - (world_x & 7);
        let p01 = self.vram[(tile * 16 + row) & 0x7fff];
        let p23 = self.vram[(tile * 16 + 8 + row) & 0x7fff];
        let color = ((p01 >> bit) & 1)
            | (((p01 >> (bit + 8)) & 1) << 1)
            | (((p23 >> bit) & 1) << 2)
            | (((p23 >> (bit + 8)) & 1) << 3);
        (color as u8, palette)
    }

    fn render_background(&mut self, vce: &HuC6260) {
        for y in 0..HEIGHT as usize {
            for x in 0..WIDTH as usize {
                let (color, palette) = self.background_pixel(x, y);
                let palette_index = palette * 16 + usize::from(color);
                let rgba = vce.rgba(palette_index);
                let pixel_index = y * WIDTH as usize + x;
                let offset = pixel_index * 4;
                self.pixel_codes[pixel_index] = palette_index as u16;
                self.video.pixels_mut()[offset..offset + 4].copy_from_slice(&rgba);
            }
        }
    }

    fn sprite_pixel(&self, pattern: usize, x: usize, y: usize) -> u8 {
        let tile_x = x / 8;
        let tile_y = y / 8;
        let tile = pattern * 4 + tile_y * 2 + tile_x;
        let row = y & 7;
        let bit = 7 - (x & 7);
        let p01 = self.vram[(tile * 16 + row) & 0x7fff];
        let p23 = self.vram[(tile * 16 + 8 + row) & 0x7fff];
        (((p01 >> bit) & 1)
            | (((p01 >> (bit + 8)) & 1) << 1)
            | (((p23 >> bit) & 1) << 2)
            | (((p23 >> (bit + 8)) & 1) << 3)) as u8
    }

    fn render_sprites(&mut self, vce: &HuC6260) {
        let mut occupied = vec![false; (WIDTH * HEIGHT) as usize];
        let mut line_count = [0u8; HEIGHT as usize];
        for sprite in (0..64).rev() {
            let base = sprite * 4;
            let raw_y = self.satb[base] & 0x03ff;
            let raw_x = self.satb[base + 1] & 0x03ff;
            let pattern = usize::from((self.satb[base + 2] >> 1) & 0x03ff);
            let attr = self.satb[base + 3];
            let x = i32::from(raw_x) - 32;
            let y = i32::from(raw_y) - 64;
            let palette = usize::from(attr & 0x000f);
            let flip_x = attr & 0x0800 != 0;
            let flip_y = attr & 0x8000 != 0;
            let behind = attr & 0x0080 == 0;
            for sy in 0..16usize {
                let py = y + sy as i32;
                if !(0..HEIGHT as i32).contains(&py) {
                    continue;
                }
                let line = py as usize;
                if line_count[line] >= 16 {
                    self.status |= 0x02;
                    continue;
                }
                for sx in 0..16usize {
                    let px = x + sx as i32;
                    if !(0..WIDTH as i32).contains(&px) {
                        continue;
                    }
                    let source_x = if flip_x { 15 - sx } else { sx };
                    let source_y = if flip_y { 15 - sy } else { sy };
                    let color = self.sprite_pixel(pattern, source_x, source_y);
                    if color == 0 {
                        continue;
                    }
                    let index = line * WIDTH as usize + px as usize;
                    if occupied[index] {
                        self.status |= 0x01;
                    }
                    occupied[index] = true;
                    if behind && self.background_pixel(px as usize, line).0 != 0 {
                        continue;
                    }
                    let palette_index = 256 + palette * 16 + usize::from(color);
                    let rgba = vce.rgba(palette_index);
                    let offset = index * 4;
                    self.pixel_codes[index] = palette_index as u16;
                    self.video.pixels_mut()[offset..offset + 4].copy_from_slice(&rgba);
                }
                line_count[line] = line_count[line].saturating_add(1);
            }
        }
        self.update_irq();
    }

    fn render(&mut self, vce: &HuC6260) {
        self.video.clear(vce.rgba(0));
        let cr = self.regs[5];
        let blank_code = if cr & 0x0080 != 0 { 0 } else { 0x0100 };
        self.pixel_codes.fill(blank_code);
        if cr & 0x0080 != 0 {
            self.render_background(vce);
        }
        if cr & 0x0040 != 0 {
            self.render_sprites(vce);
        }
    }

    fn tick(&mut self, clocks: u32, vce: &HuC6260) {
        let denominator = u128::from(CPU_CLOCK_HZ) * u128::from(FRAME_RATE_DEN);
        self.line_phase += u128::from(clocks) * u128::from(FRAME_RATE_NUM) * u128::from(SCANLINES);
        while self.line_phase >= denominator {
            self.line_phase -= denominator;
            self.scanline = self.scanline.wrapping_add(1);
            let raster = self.regs[6] & 0x03ff;
            if raster != 0 && self.scanline.wrapping_add(64) == raster {
                self.status |= 0x04;
                self.update_irq();
            }
            if self.scanline == HEIGHT as u16 {
                self.status |= 0x20;
                self.frame = self.frame.wrapping_add(1);
                self.render(vce);
                self.update_irq();
            }
            if self.scanline >= SCANLINES as u16 {
                self.scanline = 0;
            }
        }
    }

    fn save(&self, out: &mut StateWriter) {
        out.u8(self.select);
        out.u8(self.low_latch);
        out.u16(self.read_latch);
        out.u8(self.status);
        out.u16(self.scanline);
        out.u128(self.line_phase);
        out.u64(self.frame);
        out.u8(u8::from(self.irq_pending));
        for register in self.regs {
            out.u16(register);
        }
        for word in self.vram.iter() {
            out.u16(*word);
        }
        for word in self.satb {
            out.u16(word);
        }
    }

    fn load(&mut self, input: &mut StateReader<'_>, vce: &HuC6260) -> Result<(), String> {
        self.select = input.u8()? & 0x1f;
        self.low_latch = input.u8()?;
        self.read_latch = input.u16()?;
        self.status = input.u8()?;
        self.scanline = input.u16()? % SCANLINES as u16;
        self.line_phase = input.u128()?;
        self.frame = input.u64()?;
        self.irq_pending = input.u8()? != 0;
        for register in &mut self.regs {
            *register = input.u16()?;
        }
        for word in self.vram.iter_mut() {
            *word = input.u16()?;
        }
        for word in &mut self.satb {
            *word = input.u16()?;
        }
        self.render(vce);
        Ok(())
    }
}

#[derive(Clone, Copy)]
struct VpcPriority {
    prio_type: u8,
    dev0_enabled: bool,
    dev1_enabled: bool,
}

impl Default for VpcPriority {
    fn default() -> Self {
        Self {
            prio_type: 0,
            dev0_enabled: true,
            dev1_enabled: false,
        }
    }
}

#[derive(Default)]
struct HuC6202 {
    priorities: [VpcPriority; 4],
    windows: [u16; 2],
    io_device: bool,
}

impl HuC6202 {
    fn priority_nibble(priority: VpcPriority) -> u8 {
        (priority.prio_type << 2)
            | u8::from(priority.dev0_enabled)
            | (u8::from(priority.dev1_enabled) << 1)
    }

    fn set_priority_nibble(&mut self, index: usize, value: u8) {
        self.priorities[index] = VpcPriority {
            prio_type: (value >> 2) & 3,
            dev0_enabled: value & 1 != 0,
            dev1_enabled: value & 2 != 0,
        };
    }

    fn read(&self, offset: u32) -> u8 {
        match offset & 7 {
            0 => {
                Self::priority_nibble(self.priorities[0])
                    | (Self::priority_nibble(self.priorities[1]) << 4)
            }
            1 => {
                Self::priority_nibble(self.priorities[2])
                    | (Self::priority_nibble(self.priorities[3]) << 4)
            }
            2 => self.windows[0] as u8,
            3 => (self.windows[0] >> 8) as u8,
            4 => self.windows[1] as u8,
            5 => (self.windows[1] >> 8) as u8,
            _ => 0xff,
        }
    }

    fn write(&mut self, offset: u32, value: u8) {
        match offset & 7 {
            0 => {
                self.set_priority_nibble(0, value & 0x0f);
                self.set_priority_nibble(1, value >> 4);
            }
            1 => {
                self.set_priority_nibble(2, value & 0x0f);
                self.set_priority_nibble(3, value >> 4);
            }
            2 => self.windows[0] = (self.windows[0] & 0x0300) | u16::from(value),
            3 => self.windows[0] = (self.windows[0] & 0x00ff) | (u16::from(value & 3) << 8),
            4 => self.windows[1] = (self.windows[1] & 0x0300) | u16::from(value),
            5 => self.windows[1] = (self.windows[1] & 0x00ff) | (u16::from(value & 3) << 8),
            6 => self.io_device = value & 1 != 0,
            _ => {}
        }
    }

    fn compose_code(&self, x: usize, data0: u16, data1: u16) -> u16 {
        const SPRITE_BASE: u16 = 0x0100;
        if data0 == SPRITE_BASE && data1 == SPRITE_BASE {
            return 0;
        }
        let mut map = 0usize;
        let x = x as u16;
        if self.windows[0] < 0x40 || x > self.windows[0] {
            map |= 1;
        }
        if self.windows[1] < 0x40 || x > self.windows[1] {
            map |= 2;
        }
        let priority = self.priorities[map];
        let dev0 = priority.dev0_enabled && data0 != SPRITE_BASE;
        let dev1 = priority.dev1_enabled && data1 != SPRITE_BASE;
        match (dev0, dev1) {
            (true, false) => data0,
            (false, true) => data1,
            (false, false) => 0,
            (true, true) => match priority.prio_type {
                1 => {
                    if data0 > SPRITE_BASE {
                        data0
                    } else if data1 > SPRITE_BASE {
                        data1
                    } else if data0 & 0x0f != 0 {
                        data0
                    } else {
                        data1
                    }
                }
                2 => {
                    if data1 > SPRITE_BASE {
                        if data0 > SPRITE_BASE || data0 & 0x0f != 0 {
                            data0
                        } else {
                            data1
                        }
                    } else if data0 > SPRITE_BASE {
                        data1
                    } else if data0 & 0x0f != 0 {
                        data0
                    } else {
                        data1
                    }
                }
                _ => {
                    if data0 & 0x0f != 0 {
                        data0
                    } else {
                        data1
                    }
                }
            },
        }
    }

    fn save(&self, out: &mut StateWriter) {
        out.u8(self.read(0));
        out.u8(self.read(1));
        out.u16(self.windows[0]);
        out.u16(self.windows[1]);
        out.u8(u8::from(self.io_device));
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.write(0, input.u8()?);
        self.write(1, input.u8()?);
        self.windows[0] = input.u16()? & 0x03ff;
        self.windows[1] = input.u16()? & 0x03ff;
        self.io_device = input.u8()? != 0;
        Ok(())
    }
}

#[derive(Clone)]
struct PcePsgChannel {
    frequency: u16,
    control: u8,
    balance: u8,
    wave: [u8; 32],
    wave_write: u8,
    wave_position: u8,
    phase: u64,
    dda: u8,
    noise: u8,
    noise_phase: u64,
    noise_lfsr: u32,
}

impl Default for PcePsgChannel {
    fn default() -> Self {
        Self {
            frequency: 1,
            control: 0,
            balance: 0xff,
            wave: [16; 32],
            wave_write: 0,
            wave_position: 0,
            phase: 0,
            dda: 16,
            noise: 0,
            noise_phase: 0,
            noise_lfsr: 0x1ffff,
        }
    }
}

struct PcePsg {
    selected: usize,
    global_balance: u8,
    lfo_frequency: u8,
    lfo_control: u8,
    sample_phase: u64,
    channels: [PcePsgChannel; 6],
    samples: Vec<(f32, f32)>,
}

impl Default for PcePsg {
    fn default() -> Self {
        Self {
            selected: 0,
            global_balance: 0xff,
            lfo_frequency: 0,
            lfo_control: 0,
            sample_phase: 0,
            channels: std::array::from_fn(|_| PcePsgChannel::default()),
            samples: Vec::with_capacity(1024),
        }
    }
}

impl PcePsg {
    fn begin_frame(&mut self) {
        self.samples.clear();
    }

    fn read(&self, port: u32) -> u8 {
        match port & 0x0f {
            0 => self.selected as u8,
            1 => self.global_balance,
            2 => self.channels[self.selected].frequency as u8,
            3 => (self.channels[self.selected].frequency >> 8) as u8,
            4 => self.channels[self.selected].control,
            5 => self.channels[self.selected].balance,
            6 => self.channels[self.selected].dda,
            7 => self.channels[self.selected].noise,
            8 => self.lfo_frequency,
            9 => self.lfo_control,
            _ => 0xff,
        }
    }

    fn write(&mut self, port: u32, value: u8) {
        match port & 0x0f {
            0 => self.selected = usize::from(value & 7).min(5),
            1 => self.global_balance = value,
            2 => {
                let channel = &mut self.channels[self.selected];
                channel.frequency = (channel.frequency & 0x0f00) | u16::from(value);
            }
            3 => {
                let channel = &mut self.channels[self.selected];
                channel.frequency = (channel.frequency & 0x00ff) | (u16::from(value & 0x0f) << 8);
            }
            4 => {
                let channel = &mut self.channels[self.selected];
                if channel.control & 0x40 == 0 && value & 0x40 != 0 {
                    channel.wave_position = 0;
                }
                channel.control = value;
            }
            5 => self.channels[self.selected].balance = value,
            6 => {
                let channel = &mut self.channels[self.selected];
                let sample = value & 0x1f;
                if channel.control & 0x40 != 0 {
                    channel.dda = sample;
                } else if channel.control & 0x80 == 0 {
                    channel.wave[usize::from(channel.wave_write & 31)] = sample;
                    channel.wave_write = channel.wave_write.wrapping_add(1) & 31;
                }
            }
            7 => self.channels[self.selected].noise = value,
            8 => self.lfo_frequency = value,
            9 => self.lfo_control = value,
            _ => {}
        }
    }

    fn balance_gain(balance: u8, left: bool) -> f32 {
        let nibble = if left { balance >> 4 } else { balance & 0x0f };
        f32::from(nibble) / 15.0
    }

    fn channel_sample(channel: &mut PcePsgChannel) -> f32 {
        if channel.control & 0x80 == 0 {
            return 0.0;
        }
        if channel.control & 0x40 != 0 {
            return (f32::from(channel.dda) - 15.5) / 15.5;
        }
        let period = u64::from(channel.frequency.max(1));
        channel.phase = channel.phase.saturating_add(3_579_545);
        let threshold = period
            .saturating_mul(32)
            .saturating_mul(u64::from(AUDIO_RATE));
        while channel.phase >= threshold {
            channel.phase -= threshold;
            channel.wave_position = channel.wave_position.wrapping_add(1) & 31;
        }
        let sample = channel.wave[usize::from(channel.wave_position)];
        (f32::from(sample) - 15.5) / 15.5
    }

    fn noise_sample(channel: &mut PcePsgChannel) -> Option<f32> {
        if channel.noise & 0x80 == 0 {
            return None;
        }
        let period = u64::from(32u8.wrapping_sub(channel.noise & 0x1f).max(1));
        channel.noise_phase = channel.noise_phase.saturating_add(3_579_545);
        let threshold = period
            .saturating_mul(64)
            .saturating_mul(u64::from(AUDIO_RATE));
        while channel.noise_phase >= threshold {
            channel.noise_phase -= threshold;
            let feedback = (channel.noise_lfsr ^ (channel.noise_lfsr >> 3)) & 1;
            channel.noise_lfsr = (channel.noise_lfsr >> 1) | (feedback << 16);
        }
        Some(if channel.noise_lfsr & 1 != 0 {
            1.0
        } else {
            -1.0
        })
    }

    fn emit_sample(&mut self) {
        let mut left = 0.0f32;
        let mut right = 0.0f32;
        for (index, channel) in self.channels.iter_mut().enumerate() {
            let mut sample = Self::channel_sample(channel);
            if index >= 4 {
                if let Some(noise) = Self::noise_sample(channel) {
                    sample = noise;
                }
            }
            let volume = f32::from(channel.control & 0x1f) / 31.0;
            left += sample * volume * Self::balance_gain(channel.balance, true);
            right += sample * volume * Self::balance_gain(channel.balance, false);
        }
        left *= Self::balance_gain(self.global_balance, true) / 6.0;
        right *= Self::balance_gain(self.global_balance, false) / 6.0;
        self.samples
            .push((left.clamp(-1.0, 1.0), right.clamp(-1.0, 1.0)));
    }

    fn tick(&mut self, clocks: u32) {
        self.sample_phase = self
            .sample_phase
            .saturating_add(u64::from(clocks) * u64::from(AUDIO_RATE));
        while self.sample_phase >= CPU_CLOCK_HZ {
            self.sample_phase -= CPU_CLOCK_HZ;
            self.emit_sample();
        }
    }

    fn save(&self, out: &mut StateWriter) {
        out.u8(self.selected as u8);
        out.u8(self.global_balance);
        out.u8(self.lfo_frequency);
        out.u8(self.lfo_control);
        out.u64(self.sample_phase);
        for channel in &self.channels {
            out.u16(channel.frequency);
            out.u8(channel.control);
            out.u8(channel.balance);
            out.blob(&channel.wave);
            out.u8(channel.wave_write);
            out.u8(channel.wave_position);
            out.u64(channel.phase);
            out.u8(channel.dda);
            out.u8(channel.noise);
            out.u64(channel.noise_phase);
            out.u32(channel.noise_lfsr);
        }
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.selected = usize::from(input.u8()?).min(5);
        self.global_balance = input.u8()?;
        self.lfo_frequency = input.u8()?;
        self.lfo_control = input.u8()?;
        self.sample_phase = input.u64()? % CPU_CLOCK_HZ;
        for channel in &mut self.channels {
            channel.frequency = input.u16()? & 0x0fff;
            channel.control = input.u8()?;
            channel.balance = input.u8()?;
            let wave = input.blob()?;
            if wave.len() != 32 {
                return Err("PC Engine PSG state has invalid waveform length".into());
            }
            channel.wave.copy_from_slice(wave);
            channel.wave_write = input.u8()? & 31;
            channel.wave_position = input.u8()? & 31;
            channel.phase = input.u64()?;
            channel.dda = input.u8()? & 31;
            channel.noise = input.u8()?;
            channel.noise_phase = input.u64()?;
            channel.noise_lfsr = input.u32()? & 0x1ffff;
        }
        self.samples.clear();
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum PceCdPhase {
    BusFree,
    Command,
    DataIn,
    Status,
    MessageIn,
}

impl PceCdPhase {
    fn encode(self) -> u8 {
        match self {
            Self::BusFree => 0,
            Self::Command => 1,
            Self::DataIn => 2,
            Self::Status => 3,
            Self::MessageIn => 4,
        }
    }

    fn decode(value: u8) -> Result<Self, String> {
        match value {
            0 => Ok(Self::BusFree),
            1 => Ok(Self::Command),
            2 => Ok(Self::DataIn),
            3 => Ok(Self::Status),
            4 => Ok(Self::MessageIn),
            _ => Err("PC Engine CD state has invalid SCSI phase".into()),
        }
    }
}

fn pce_crc32_ieee(bytes: &[u8]) -> u32 {
    let mut crc = 0xffff_ffffu32;
    for &byte in bytes {
        crc ^= u32::from(byte);
        for _ in 0..8 {
            let mask = 0u32.wrapping_sub(crc & 1);
            crc = (crc >> 1) ^ (0xedb8_8320 & mask);
        }
    }
    !crc
}

#[derive(Clone, Copy)]
struct PceFade {
    gain: u32,
    start: u32,
    target: u32,
    elapsed: u64,
    duration: u64,
}

impl Default for PceFade {
    fn default() -> Self {
        Self {
            gain: MIX_GAIN_ONE,
            start: MIX_GAIN_ONE,
            target: MIX_GAIN_ONE,
            elapsed: 0,
            duration: 0,
        }
    }
}

impl PceFade {
    fn start(&mut self, from: u32, target: u32, duration: u64) {
        self.gain = from.min(MIX_GAIN_ONE);
        self.start = self.gain;
        self.target = target.min(MIX_GAIN_ONE);
        self.elapsed = 0;
        self.duration = duration.max(1);
    }

    fn tick(&mut self, clocks: u32) {
        if self.duration == 0 {
            return;
        }
        self.elapsed = self
            .elapsed
            .saturating_add(u64::from(clocks))
            .min(self.duration);
        let start = i64::from(self.start);
        let delta = i64::from(self.target) - start;
        self.gain = (start + delta * self.elapsed as i64 / self.duration as i64) as u32;
        if self.elapsed == self.duration {
            self.gain = self.target;
            self.start = self.target;
            self.elapsed = 0;
            self.duration = 0;
        }
    }

    fn factor(self) -> f32 {
        self.gain as f32 / MIX_GAIN_ONE as f32
    }

    fn save(self, out: &mut StateWriter) {
        out.u32(self.gain);
        out.u32(self.start);
        out.u32(self.target);
        out.u64(self.elapsed);
        out.u64(self.duration);
    }

    fn load(input: &mut StateReader<'_>) -> Result<Self, String> {
        let fade = Self {
            gain: input.u32()?,
            start: input.u32()?,
            target: input.u32()?,
            elapsed: input.u64()?,
            duration: input.u64()?,
        };
        if fade.gain > MIX_GAIN_ONE || fade.start > MIX_GAIN_ONE || fade.target > MIX_GAIN_ONE {
            return Err("PC Engine CD state has invalid fader gain".into());
        }
        if fade.duration == 0 {
            if fade.elapsed != 0 {
                return Err("PC Engine CD state has invalid idle fader state".into());
            }
        } else if fade.duration > CPU_CLOCK_HZ * 5 || fade.elapsed > fade.duration {
            return Err("PC Engine CD state has invalid fader position".into());
        }
        Ok(fade)
    }
}

#[derive(Clone, Copy, Default)]
struct ArcadeCardPort {
    ctrl: u8,
    base_addr: u32,
    addr_offset: u16,
    addr_inc: u16,
}

struct ArcadeCard {
    ram: Box<[u8]>,
    ports: [ArcadeCardPort; 4],
    shift: u32,
    shift_reg: u8,
    rotate_reg: u8,
}

impl ArcadeCardPort {
    fn signed_offset(&self) -> u32 {
        u32::from(self.addr_offset) + if self.ctrl & 0x08 != 0 { 0xff0000 } else { 0 }
    }

    fn ram_addr(&self) -> usize {
        let address = if self.ctrl & 0x02 != 0 {
            self.base_addr.wrapping_add(self.signed_offset())
        } else {
            self.base_addr
        };
        (address & 0x1fffff) as usize
    }

    fn addr_increment(&mut self) {
        if self.ctrl & 0x01 == 0 {
            return;
        }
        if self.ctrl & 0x10 != 0 {
            self.base_addr = self.base_addr.wrapping_add(u32::from(self.addr_inc)) & 0xffffff;
        } else {
            self.addr_offset = self.addr_offset.wrapping_add(self.addr_inc);
        }
    }

    fn adjust_addr(&mut self) {
        self.base_addr = self.base_addr.wrapping_add(self.signed_offset()) & 0xffffff;
    }
}

impl Default for ArcadeCard {
    fn default() -> Self {
        Self {
            ram: vec![0; ARCADE_CARD_RAM_SIZE].into_boxed_slice(),
            ports: [ArcadeCardPort::default(); 4],
            shift: 0,
            shift_reg: 0,
            rotate_reg: 0,
        }
    }
}

impl ArcadeCard {
    fn reset_registers(&mut self) {
        self.ports = [ArcadeCardPort::default(); 4];
        self.shift = 0;
        self.shift_reg = 0;
        self.rotate_reg = 0;
    }

    fn read_reg(&mut self, offset: u8) -> u8 {
        if offset & 0xe0 == 0xe0 {
            return match offset & 0x1f {
                0x00 => self.shift as u8,
                0x01 => (self.shift >> 8) as u8,
                0x02 => (self.shift >> 16) as u8,
                0x03 => (self.shift >> 24) as u8,
                0x04 => self.shift_reg,
                0x05 => self.rotate_reg,
                0x1c | 0x1d => 0x00,
                0x1e => 0x10,
                0x1f => 0x51,
                _ => 0,
            };
        }
        let port = &mut self.ports[usize::from((offset & 0x30) >> 4)];
        match offset & 0x8f {
            0x00 | 0x01 => {
                let value = self.ram[port.ram_addr()];
                port.addr_increment();
                value
            }
            0x02 => port.base_addr as u8,
            0x03 => (port.base_addr >> 8) as u8,
            0x04 => (port.base_addr >> 16) as u8,
            0x05 => port.addr_offset as u8,
            0x06 => (port.addr_offset >> 8) as u8,
            0x07 => port.addr_inc as u8,
            0x08 => (port.addr_inc >> 8) as u8,
            0x09 => port.ctrl,
            0x0a..=0x0f => 0,
            _ => 0xff,
        }
    }

    fn write_reg(&mut self, offset: u8, data: u8) {
        if offset & 0xe0 == 0xe0 {
            match offset & 0x0f {
                0 => self.shift = u32::from(data) | (self.shift & 0xffffff00),
                1 => self.shift = (u32::from(data) << 8) | (self.shift & 0xffff00ff),
                2 => self.shift = (u32::from(data) << 16) | (self.shift & 0xff00ffff),
                3 => self.shift = (u32::from(data) << 24) | (self.shift & 0x00ffffff),
                4 => {
                    self.shift_reg = data & 0x0f;
                    if self.shift_reg != 0 {
                        self.shift = if self.shift_reg < 8 {
                            self.shift << self.shift_reg
                        } else {
                            self.shift >> (16 - self.shift_reg)
                        };
                    }
                }
                5 => {
                    self.rotate_reg = data & 0x0f;
                    if self.rotate_reg != 0 {
                        self.shift = if self.rotate_reg < 8 {
                            self.shift.rotate_left(u32::from(self.rotate_reg))
                        } else {
                            self.shift.rotate_right(u32::from(16 - self.rotate_reg))
                        };
                    }
                }
                _ => {}
            }
            return;
        }
        let port = &mut self.ports[usize::from((offset & 0x30) >> 4)];
        match offset & 0x8f {
            0x00 | 0x01 => {
                let address = port.ram_addr();
                self.ram[address] = data;
                port.addr_increment();
            }
            0x02 => port.base_addr = u32::from(data) | (port.base_addr & 0xffff00),
            0x03 => port.base_addr = (u32::from(data) << 8) | (port.base_addr & 0xff00ff),
            0x04 => port.base_addr = (u32::from(data) << 16) | (port.base_addr & 0x00ffff),
            0x05 => {
                port.addr_offset = u16::from(data) | (port.addr_offset & 0xff00);
                if port.ctrl & 0x60 == 0x20 {
                    port.adjust_addr();
                }
            }
            0x06 => {
                port.addr_offset = (u16::from(data) << 8) | (port.addr_offset & 0x00ff);
                if port.ctrl & 0x60 == 0x40 {
                    port.adjust_addr();
                }
            }
            0x07 => port.addr_inc = u16::from(data) | (port.addr_inc & 0xff00),
            0x08 => port.addr_inc = (u16::from(data) << 8) | (port.addr_inc & 0x00ff),
            0x09 => port.ctrl = data & 0x7f,
            0x0a if port.ctrl & 0x60 == 0x60 => port.adjust_addr(),
            _ => {}
        }
    }

    fn read_window(&mut self, offset: usize) -> u8 {
        self.read_reg(((offset & 0x6000) >> 9) as u8)
    }

    fn write_window(&mut self, offset: usize, data: u8) {
        self.write_reg(((offset & 0x6000) >> 9) as u8, data);
    }

    fn save(&self, out: &mut StateWriter) {
        out.blob(&self.ram);
        for port in self.ports {
            out.u8(port.ctrl);
            out.u32(port.base_addr);
            out.u16(port.addr_offset);
            out.u16(port.addr_inc);
        }
        out.u32(self.shift);
        out.u8(self.shift_reg);
        out.u8(self.rotate_reg);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let ram = input.blob()?;
        if ram.len() != ARCADE_CARD_RAM_SIZE {
            return Err("PC Engine Arcade Card state has invalid DRAM length".into());
        }
        self.ram.copy_from_slice(ram);
        for port in &mut self.ports {
            port.ctrl = input.u8()?;
            port.base_addr = input.u32()?;
            port.addr_offset = input.u16()?;
            port.addr_inc = input.u16()?;
            if port.ctrl > 0x7f || port.base_addr > 0xffffff {
                return Err("PC Engine Arcade Card state has invalid port state".into());
            }
        }
        self.shift = input.u32()?;
        self.shift_reg = input.u8()?;
        self.rotate_reg = input.u8()?;
        if self.shift_reg > 0x0f || self.rotate_reg > 0x0f {
            return Err("PC Engine Arcade Card state has invalid shift state".into());
        }
        Ok(())
    }
}

struct PceCdInterface {
    disc: DiscImage,
    cd_ram: Box<[u8; CD_RAM_SIZE]>,
    super_ram: Box<[u8; SUPER_CD_RAM_SIZE]>,
    super_system: bool,
    us_system_card: bool,
    backup_ram: Box<[u8; BACKUP_RAM_SIZE]>,
    backup_locked: bool,
    adpcm_ram: Box<[u8; ADPCM_RAM_SIZE]>,
    arcade_card: ArcadeCard,
    phase: PceCdPhase,
    cdc_data: u8,
    irq_mask: u8,
    irq_status: u8,
    reset_reg: u8,
    bram_status: u8,
    busy: bool,
    req: bool,
    cd: bool,
    io: bool,
    msg: bool,
    ack: bool,
    command: [u8; 10],
    command_len: usize,
    command_expected: usize,
    data: Vec<u8>,
    data_index: usize,
    read_lba: u32,
    read_remaining: u16,
    cdda_status: u8,
    cdda_play_mode: u8,
    cdda_start_lba: u32,
    cdda_end_lba: u32,
    cdda_lba: u32,
    cdda_sample_index: usize,
    cdda_clock_phase: u64,
    cdda_source_phase: u32,
    cdda_sector: Box<[u8; 2352]>,
    cdda_current: [i16; 2],
    cdda_samples: Vec<(f32, f32)>,
    cdda_fade: PceFade,
    adpcm_latch: u16,
    adpcm_read_ptr: u16,
    adpcm_read_delay: u8,
    adpcm_write_ptr: u16,
    adpcm_write_delay: u8,
    adpcm_length: u16,
    adpcm_play_addr: u16,
    adpcm_end_addr: u16,
    adpcm_low_nibble: bool,
    adpcm_clock_divider: u8,
    adpcm_clock_phase: u64,
    adpcm_signal: i16,
    adpcm_step: u8,
    adpcm_current: i16,
    adpcm_samples: Vec<(f32, f32)>,
    adpcm_fade: PceFade,
    adpcm_control: u8,
    adpcm_dma_control: u8,
    adpcm_status: u8,
    fader_control: u8,
}

impl PceCdInterface {
    fn new(disc: ResourceBlob, bios_crc: u32) -> Result<Self, String> {
        let mut backup_ram = Box::new([0; BACKUP_RAM_SIZE]);
        backup_ram[..8].copy_from_slice(&[b'H', b'U', b'B', b'M', 0x00, 0x88, 0x10, 0x80]);
        Ok(Self {
            disc: DiscImage::new(disc)?,
            cd_ram: Box::new([0; CD_RAM_SIZE]),
            super_ram: Box::new([0; SUPER_CD_RAM_SIZE]),
            super_system: matches!(bios_crc, 0x6d9a_73ef | 0x2b5b_75fe),
            us_system_card: bios_crc == 0x2b5b_75fe,
            backup_ram,
            backup_locked: true,
            adpcm_ram: Box::new([0; ADPCM_RAM_SIZE]),
            arcade_card: ArcadeCard::default(),
            phase: PceCdPhase::BusFree,
            cdc_data: 0,
            irq_mask: 0,
            irq_status: 0,
            reset_reg: 0,
            bram_status: 0,
            busy: false,
            req: false,
            cd: false,
            io: false,
            msg: false,
            ack: false,
            command: [0; 10],
            command_len: 0,
            command_expected: 0,
            data: Vec::new(),
            data_index: 0,
            read_lba: 0,
            read_remaining: 0,
            cdda_status: CDDA_OFF,
            cdda_play_mode: 0,
            cdda_start_lba: 0,
            cdda_end_lba: 0,
            cdda_lba: 0,
            cdda_sample_index: CDDA_FRAMES_PER_SECTOR,
            cdda_clock_phase: 0,
            cdda_source_phase: 0,
            cdda_sector: Box::new([0; 2352]),
            cdda_current: [0; 2],
            cdda_samples: Vec::with_capacity(1024),
            cdda_fade: PceFade::default(),
            adpcm_latch: 0,
            adpcm_read_ptr: 0,
            adpcm_read_delay: 0,
            adpcm_write_ptr: 0,
            adpcm_write_delay: 0,
            adpcm_length: 0,
            adpcm_play_addr: 0,
            adpcm_end_addr: 0,
            adpcm_low_nibble: false,
            adpcm_clock_divider: 1,
            adpcm_clock_phase: 0,
            adpcm_signal: 0,
            adpcm_step: 0,
            adpcm_current: 0,
            adpcm_samples: Vec::with_capacity(1024),
            adpcm_fade: PceFade::default(),
            adpcm_control: 0,
            adpcm_dma_control: 0,
            adpcm_status: ADPCM_STOP_FLAG,
            fader_control: 0xff,
        })
    }

    fn reset_runtime(&mut self) {
        self.cd_ram.fill(0);
        self.super_ram.fill(0);
        self.adpcm_ram.fill(0);
        self.arcade_card.reset_registers();
        self.phase = PceCdPhase::BusFree;
        self.cdc_data = 0;
        self.irq_mask = 0;
        self.irq_status = 0;
        self.reset_reg = 0;
        self.bram_status = 0;
        self.backup_locked = true;
        self.busy = false;
        self.req = false;
        self.cd = false;
        self.io = false;
        self.msg = false;
        self.ack = false;
        self.command = [0; 10];
        self.command_len = 0;
        self.command_expected = 0;
        self.data.clear();
        self.data_index = 0;
        self.read_lba = 0;
        self.read_remaining = 0;
        self.cdda_status = CDDA_OFF;
        self.cdda_play_mode = 0;
        self.cdda_start_lba = 0;
        self.cdda_end_lba = 0;
        self.cdda_lba = 0;
        self.cdda_sample_index = CDDA_FRAMES_PER_SECTOR;
        self.cdda_clock_phase = 0;
        self.cdda_source_phase = 0;
        self.cdda_sector.fill(0);
        self.cdda_current = [0; 2];
        self.cdda_samples.clear();
        self.cdda_fade = PceFade::default();
        self.adpcm_latch = 0;
        self.adpcm_read_ptr = 0;
        self.adpcm_read_delay = 0;
        self.adpcm_write_ptr = 0;
        self.adpcm_write_delay = 0;
        self.adpcm_length = 0;
        self.adpcm_play_addr = 0;
        self.adpcm_end_addr = 0;
        self.adpcm_low_nibble = false;
        self.adpcm_clock_divider = 1;
        self.adpcm_clock_phase = 0;
        self.adpcm_signal = 0;
        self.adpcm_step = 0;
        self.adpcm_current = 0;
        self.adpcm_samples.clear();
        self.adpcm_fade = PceFade::default();
        self.adpcm_control = 0;
        self.adpcm_dma_control = 0;
        self.adpcm_status = ADPCM_STOP_FLAG;
        self.fader_control = 0xff;
    }

    fn system_card_register(&self, offset: u8) -> u8 {
        match offset & 7 {
            1 => 0xaa,
            2 => 0x55,
            3 => 0x00,
            5 => {
                if self.us_system_card {
                    0x55
                } else {
                    0xaa
                }
            }
            6 => {
                if self.us_system_card {
                    0xaa
                } else {
                    0x55
                }
            }
            7 => 0x03,
            _ => 0x00,
        }
    }

    fn irq_pending(&self) -> bool {
        self.irq_mask & self.irq_status & 0x7c != 0
    }

    fn scsi_status(&self) -> u8 {
        u8::from(self.busy) << 7
            | u8::from(self.req) << 6
            | u8::from(self.msg) << 5
            | u8::from(self.cd) << 4
            | u8::from(self.io) << 3
    }

    fn select(&mut self) {
        self.phase = PceCdPhase::Command;
        self.busy = true;
        self.req = true;
        self.cd = true;
        self.io = false;
        self.msg = false;
        self.ack = false;
        self.command_len = 0;
        self.command_expected = 0;
        self.irq_status &= !0x70;
    }

    fn bus_free(&mut self) {
        self.phase = PceCdPhase::BusFree;
        self.busy = false;
        self.req = false;
        self.cd = false;
        self.io = false;
        self.msg = false;
        self.ack = false;
    }

    fn read_reg(&mut self, offset: u8) -> u8 {
        match offset & 0x0f {
            0x00 => self.scsi_status(),
            0x01 => self.cdc_data,
            0x02 => self.irq_mask,
            0x03 => {
                self.backup_locked = true;
                let status = self.irq_status & 0x6e;
                self.irq_status ^= 0x02;
                status
            }
            0x04 => self.reset_reg,
            0x05 | 0x06 => {
                let channel = if self.irq_status & 0x02 != 0 { 0 } else { 1 };
                self.cdda_current[channel].to_le_bytes()[usize::from(offset - 0x05)]
            }
            0x07 => (self.bram_status & 0x7f) | (u8::from(!self.backup_locked) * 0x80),
            0x08 => self.read_cd_data_port(),
            0x0a => {
                if self.adpcm_read_delay != 0 {
                    self.adpcm_read_delay -= 1;
                    0
                } else {
                    let value = self.adpcm_ram[usize::from(self.adpcm_read_ptr)];
                    self.adpcm_read_ptr = self.adpcm_read_ptr.wrapping_add(1);
                    value
                }
            }
            0x0b => self.adpcm_dma_control,
            0x0c => self.adpcm_status,
            0x0d => self.adpcm_control,
            _ => 0xff,
        }
    }

    fn write_reg(&mut self, offset: u8, value: u8) {
        match offset & 0x0f {
            0x00 => self.select(),
            0x01 => self.cdc_data = value,
            0x02 => {
                self.irq_mask = value;
                self.set_ack(value & 0x80 != 0);
            }
            0x04 => {
                if value & 0x02 != 0 {
                    self.bus_free();
                    self.command_len = 0;
                    self.data.clear();
                    self.data_index = 0;
                    self.read_remaining = 0;
                    self.irq_status &= !0x60;
                }
                self.reset_reg = value;
            }
            0x07 => {
                if value & 0x80 != 0 {
                    self.backup_locked = false;
                }
                self.bram_status = value;
            }
            0x08 => self.adpcm_latch = (self.adpcm_latch & 0xff00) | u16::from(value),
            0x09 => self.adpcm_latch = (self.adpcm_latch & 0x00ff) | (u16::from(value) << 8),
            0x0a => {
                if self.adpcm_write_delay != 0 {
                    self.adpcm_write_delay -= 1;
                } else {
                    self.adpcm_ram[usize::from(self.adpcm_write_ptr)] = value;
                    self.adpcm_write_ptr = self.adpcm_write_ptr.wrapping_add(1);
                }
            }
            0x0b => {
                self.adpcm_dma_control = value;
                if value & 3 != 0 {
                    self.adpcm_status |= 0x04;
                }
            }
            0x0d => self.write_adpcm_control(value),
            0x0e => self.adpcm_clock_divider = 0x10 - (value & 0x0f),
            0x0f => self.write_fader_control(value),
            _ => {}
        }
    }

    fn stop_adpcm_playback(&mut self, interrupt: bool) {
        self.adpcm_status |= ADPCM_STOP_FLAG;
        self.adpcm_status &= !ADPCM_PLAY_FLAG;
        if interrupt {
            self.irq_status |= CD_IRQ_SAMPLE_FULL_PLAY;
        }
        self.adpcm_control &= !0x60;
        self.adpcm_low_nibble = false;
        self.adpcm_current = 0;
    }

    fn start_adpcm_playback(&mut self) {
        self.adpcm_play_addr = self.adpcm_read_ptr;
        self.adpcm_end_addr = self.adpcm_read_ptr.wrapping_add(self.adpcm_length);
        self.adpcm_low_nibble = false;
        self.adpcm_clock_phase = 0;
        self.adpcm_signal = 0;
        self.adpcm_step = 0;
        self.adpcm_current = 0;
        self.adpcm_status &= !ADPCM_STOP_FLAG;
        self.adpcm_status |= ADPCM_PLAY_FLAG;
        self.irq_status &= !0x0c;
    }

    fn write_adpcm_control(&mut self, value: u8) {
        if self.adpcm_control & 0x80 != 0 && value & 0x80 == 0 {
            self.adpcm_read_ptr = 0;
            self.adpcm_read_delay = 0;
            self.adpcm_write_ptr = 0;
            self.adpcm_write_delay = 0;
            self.adpcm_play_addr = 0;
            self.adpcm_end_addr = 0;
            self.adpcm_signal = 0;
            self.adpcm_step = 0;
            self.stop_adpcm_playback(false);
        }

        if value & 0x40 != 0 && self.adpcm_control & 0x40 == 0 {
            self.start_adpcm_playback();
        } else if value & 0x40 == 0 {
            self.stop_adpcm_playback(false);
            if value & 0x20 == 0 {
                self.irq_status &= !0x0c;
            }
        }

        if value & 0x10 != 0 {
            self.adpcm_length = self.adpcm_latch;
        }
        if value & 0x08 != 0 {
            self.adpcm_read_ptr = self.adpcm_latch;
            self.adpcm_read_delay = 2;
        }
        if value & 0x02 != 0 {
            self.adpcm_write_ptr = self.adpcm_latch;
            self.adpcm_write_delay = value & 1;
        }
        self.adpcm_control = value;
    }

    fn fade_cycles(milliseconds: u64) -> u64 {
        CPU_CLOCK_HZ.saturating_mul(milliseconds) / 1_000
    }

    fn write_fader_control(&mut self, value: u8) {
        if self.fader_control == value {
            return;
        }
        match value & 0x0f {
            0x00 => {
                self.cdda_fade
                    .start(0, MIX_GAIN_ONE, Self::fade_cycles(100));
                self.adpcm_fade
                    .start(0, MIX_GAIN_ONE, Self::fade_cycles(100));
            }
            0x01 => self
                .cdda_fade
                .start(0, MIX_GAIN_ONE, Self::fade_cycles(100)),
            0x08 | 0x0c => {
                self.cdda_fade
                    .start(MIX_GAIN_ONE, 0, Self::fade_cycles(1_500));
                self.adpcm_fade
                    .start(0, MIX_GAIN_ONE, Self::fade_cycles(100));
            }
            0x09 => self
                .cdda_fade
                .start(MIX_GAIN_ONE, 0, Self::fade_cycles(5_000)),
            0x0a => self
                .adpcm_fade
                .start(MIX_GAIN_ONE, 0, Self::fade_cycles(5_000)),
            0x0d => self
                .cdda_fade
                .start(MIX_GAIN_ONE, 0, Self::fade_cycles(1_500)),
            0x0e => self
                .adpcm_fade
                .start(MIX_GAIN_ONE, 0, Self::fade_cycles(1_500)),
            _ => {}
        }
        self.fader_control = value;
    }

    fn backup_read(&self, offset: usize) -> u8 {
        if self.backup_locked {
            0xff
        } else {
            self.backup_ram[offset & (BACKUP_RAM_SIZE - 1)]
        }
    }

    fn backup_write(&mut self, offset: usize, value: u8) {
        if !self.backup_locked {
            self.backup_ram[offset & (BACKUP_RAM_SIZE - 1)] = value;
        }
    }
}

impl PceCdInterface {
    fn command_size(opcode: u8) -> usize {
        match opcode {
            0x00 | 0x03 | 0x08 => 6,
            0xd8 | 0xd9 | 0xda | 0xdd | 0xde => 10,
            _ => 6,
        }
    }

    fn set_ack(&mut self, value: bool) {
        if value == self.ack {
            return;
        }
        if value {
            if self.req {
                match self.phase {
                    PceCdPhase::Command => {
                        if self.command_len < self.command.len() {
                            self.command[self.command_len] = self.cdc_data;
                            self.command_len += 1;
                            if self.command_len == 1 {
                                self.command_expected = Self::command_size(self.command[0]);
                            }
                        }
                        self.req = false;
                    }
                    PceCdPhase::DataIn => {
                        if self.data_index < self.data.len() {
                            self.data_index += 1;
                        }
                        self.req = false;
                    }
                    PceCdPhase::Status | PceCdPhase::MessageIn => self.req = false,
                    PceCdPhase::BusFree => {}
                }
            }
        } else if self.ack {
            self.advance_after_ack();
        }
        self.ack = value;
    }

    fn advance_after_ack(&mut self) {
        match self.phase {
            PceCdPhase::Command => {
                if self.command_len != 0 && self.command_len >= self.command_expected {
                    self.execute_command();
                } else if self.busy {
                    self.req = true;
                }
            }
            PceCdPhase::DataIn => self.advance_data_phase(),
            PceCdPhase::Status => {
                self.phase = PceCdPhase::MessageIn;
                self.msg = true;
                self.cd = true;
                self.io = true;
                self.cdc_data = 0;
                self.req = true;
            }
            PceCdPhase::MessageIn => self.bus_free(),
            PceCdPhase::BusFree => {}
        }
    }

    fn read_cd_data_port(&mut self) -> u8 {
        let value = self.cdc_data;
        if self.phase == PceCdPhase::DataIn && self.req && !self.ack {
            if self.data_index < self.data.len() {
                self.data_index += 1;
            }
            self.req = false;
            self.advance_data_phase();
        }
        value
    }

    fn advance_data_phase(&mut self) {
        if self.data_index < self.data.len() {
            self.cdc_data = self.data[self.data_index];
            self.req = true;
            return;
        }
        self.data.clear();
        self.data_index = 0;
        if self.read_remaining != 0 {
            self.try_load_read_sector();
        } else {
            self.finish_data_transfer();
        }
    }

    fn begin_data(&mut self, data: Vec<u8>) {
        self.phase = PceCdPhase::DataIn;
        self.cd = false;
        self.io = true;
        self.msg = false;
        self.data = data;
        self.data_index = 0;
        self.irq_status |= CD_IRQ_TRANSFER_READY;
        if let Some(&first) = self.data.first() {
            self.cdc_data = first;
            self.req = true;
        } else {
            self.req = false;
            self.finish_data_transfer();
        }
    }

    fn finish_data_transfer(&mut self) {
        self.irq_status &= !CD_IRQ_TRANSFER_READY;
        self.irq_status |= CD_IRQ_TRANSFER_DONE;
        self.reply_status(true);
    }

    fn reply_status(&mut self, ok: bool) {
        self.phase = PceCdPhase::Status;
        self.cd = true;
        self.io = true;
        self.msg = false;
        self.cdc_data = if ok { 0x00 } else { 0x01 };
        self.req = true;
    }

    fn try_load_read_sector(&mut self) {
        if self.read_remaining == 0 {
            self.finish_data_transfer();
            return;
        }
        let mut sector = [0u8; 2048];
        match self.disc.read_user_sector(self.read_lba, &mut sector) {
            Ok(()) => {
                self.read_lba = self.read_lba.wrapping_add(1);
                self.read_remaining -= 1;
                self.phase = PceCdPhase::DataIn;
                self.cd = false;
                self.io = true;
                self.msg = false;
                self.data = sector.to_vec();
                self.data_index = 0;
                self.cdc_data = self.data[0];
                self.req = true;
                self.irq_status |= CD_IRQ_TRANSFER_READY;
            }
            Err(error) if error == RESOURCE_PENDING => {
                self.phase = PceCdPhase::DataIn;
                self.cd = false;
                self.io = true;
                self.msg = false;
                self.req = false;
            }
            Err(_) => {
                self.read_remaining = 0;
                self.irq_status &= !CD_IRQ_TRANSFER_READY;
                self.reply_status(false);
            }
        }
    }

    fn execute_command(&mut self) {
        let opcode = self.command[0];
        self.req = false;
        match opcode {
            0x00 => self.reply_status(true),
            0x03 => {
                let length = usize::from(self.command[4]).min(18);
                let mut sense = vec![0; length];
                if !sense.is_empty() {
                    sense[0] = 0x70;
                }
                if sense.len() > 7 {
                    sense[7] = (sense.len() - 8) as u8;
                }
                self.begin_data(sense);
            }
            0x08 => {
                self.read_lba = (u32::from(self.command[1] & 0x1f) << 16)
                    | (u32::from(self.command[2]) << 8)
                    | u32::from(self.command[3]);
                self.read_remaining = if self.command[4] == 0 {
                    256
                } else {
                    u16::from(self.command[4])
                };
                self.try_load_read_sector();
            }
            0xd8 => self.command_audio_start(),
            0xd9 => self.command_audio_end(),
            0xda => self.command_audio_pause(),
            0xdd => self.command_subq(),
            0xde => self.command_directory_info(),
            _ => self.reply_status(false),
        }
        self.command_len = 0;
        self.command_expected = 0;
    }

    fn bcd(value: u32) -> u8 {
        (((value / 10) % 10) << 4 | (value % 10)) as u8
    }

    fn frames_msf(frames: u32) -> [u8; 3] {
        let minutes = frames / (75 * 60);
        let seconds = (frames / 75) % 60;
        let frame = frames % 75;
        [Self::bcd(minutes), Self::bcd(seconds), Self::bcd(frame)]
    }

    fn lba_msf(lba: u32) -> [u8; 3] {
        Self::frames_msf(lba.saturating_add(150))
    }

    fn decode_bcd(value: u8) -> Option<u32> {
        let high = value >> 4;
        let low = value & 0x0f;
        (high <= 9 && low <= 9).then_some(u32::from(high * 10 + low))
    }

    fn audio_command_position(&self) -> Option<u32> {
        match self.command[9] & 0xc0 {
            0x00 => Some(
                (u32::from(self.command[3]) << 16)
                    | (u32::from(self.command[4]) << 8)
                    | u32::from(self.command[5]),
            ),
            0x40 => {
                let minute = Self::decode_bcd(self.command[2])?;
                let second = Self::decode_bcd(self.command[3]).filter(|value| *value < 60)?;
                let frame = Self::decode_bcd(self.command[4]).filter(|value| *value < 75)?;
                Some((minute * 60 * 75 + second * 75 + frame).saturating_sub(150))
            }
            0x80 => {
                let track = u8::try_from(Self::decode_bcd(self.command[2])?).ok()?;
                self.disc.track_start(track)
            }
            _ => None,
        }
    }

    fn command_audio_start(&mut self) {
        let Some(start) = self.audio_command_position() else {
            self.reply_status(false);
            return;
        };
        self.cdda_start_lba = start.min(self.disc.sectors);
        self.cdda_lba = self.cdda_start_lba;
        self.cdda_sample_index = CDDA_FRAMES_PER_SECTOR;
        self.cdda_source_phase = 0;
        self.cdda_play_mode = self.command[1] & 3;
        if self.cdda_play_mode == 0 {
            self.cdda_status = CDDA_PAUSED;
        } else {
            self.cdda_end_lba = self.disc.sectors;
            self.cdda_status = CDDA_PLAYING;
        }
        self.reply_status(true);
    }

    fn command_audio_end(&mut self) {
        let Some(end) = self.audio_command_position() else {
            self.reply_status(false);
            return;
        };
        self.cdda_end_lba = end.min(self.disc.sectors);
        self.cdda_play_mode = self.command[1] & 3;
        if self.cdda_play_mode == 0 || self.cdda_end_lba <= self.cdda_start_lba {
            self.cdda_status = CDDA_OFF;
        } else {
            self.cdda_lba = self.cdda_start_lba;
            self.cdda_sample_index = CDDA_FRAMES_PER_SECTOR;
            self.cdda_source_phase = 0;
            self.cdda_status = CDDA_PLAYING;
        }
        self.reply_status(true);
    }

    fn command_audio_pause(&mut self) {
        if self.cdda_status == CDDA_PLAYING {
            self.cdda_status = CDDA_PAUSED;
        }
        self.reply_status(true);
    }

    fn command_subq(&mut self) {
        let active_lba = if self.cdda_status == CDDA_OFF {
            self.read_lba
        } else if self.cdda_sample_index < CDDA_FRAMES_PER_SECTOR {
            self.cdda_lba.saturating_sub(1)
        } else {
            self.cdda_lba
        };
        let lba = active_lba.min(self.disc.sectors);
        let absolute = Self::lba_msf(lba);
        let (track_number, control, relative) =
            self.disc
                .track_for_lba(lba)
                .map_or((0xaau8, 0x41, [0; 3]), |track| {
                    let relative_lba = lba.saturating_sub(track.start_lba);
                    (
                        track.number,
                        0x01 | if track.kind == TrackKind::Data {
                            0x40
                        } else {
                            0
                        },
                        Self::frames_msf(relative_lba),
                    )
                });
        let status = match self.cdda_status {
            CDDA_PLAYING => 0,
            CDDA_PAUSED => 2,
            _ => 3,
        };
        self.begin_data(vec![
            status,
            control,
            if track_number == 0xaa {
                0xaa
            } else {
                Self::bcd(u32::from(track_number))
            },
            0x01,
            relative[0],
            relative[1],
            relative[2],
            absolute[0],
            absolute[1],
            absolute[2],
        ]);
    }

    fn command_directory_info(&mut self) {
        match self.command[1] {
            0x00 => self.begin_data(vec![
                Self::bcd(u32::from(self.disc.first_track())),
                Self::bcd(u32::from(self.disc.last_track())),
            ]),
            0x01 => self.begin_data(Self::lba_msf(self.disc.sectors).to_vec()),
            0x02 => {
                let (lba, kind) = if self.command[2] == 0xaa {
                    (self.disc.sectors, TrackKind::Data)
                } else {
                    let Some(track_number) = Self::decode_bcd(self.command[2])
                        .and_then(|number| u8::try_from(number).ok())
                    else {
                        self.reply_status(false);
                        return;
                    };
                    let Some(track) = self
                        .disc
                        .tracks
                        .iter()
                        .find(|track| track.number == track_number)
                    else {
                        self.reply_status(false);
                        return;
                    };
                    (track.start_lba, track.kind)
                };
                let mut response = Self::lba_msf(lba).to_vec();
                response.push(if kind == TrackKind::Data { 0x04 } else { 0x00 });
                self.begin_data(response);
            }
            _ => self.reply_status(false),
        }
    }

    fn begin_frame(&mut self) {
        self.cdda_samples.clear();
        self.adpcm_samples.clear();
    }

    fn decode_adpcm_nibble(&mut self, nibble: u8) {
        const STEPS: [i32; 49] = [
            16, 17, 19, 21, 23, 25, 28, 31, 34, 37, 41, 45, 50, 55, 60, 66, 73, 80, 88, 97, 107,
            118, 130, 143, 157, 173, 190, 209, 230, 253, 279, 307, 337, 371, 408, 449, 494, 544,
            598, 658, 724, 796, 876, 963, 1060, 1166, 1282, 1411, 1552,
        ];
        const INDEX_SHIFT: [i8; 8] = [-1, -1, -1, -1, 2, 4, 6, 8];

        let step = STEPS[usize::from(self.adpcm_step)];
        let mut difference = step / 8;
        if nibble & 0x04 != 0 {
            difference += step;
        }
        if nibble & 0x02 != 0 {
            difference += step / 2;
        }
        if nibble & 0x01 != 0 {
            difference += step / 4;
        }
        let signal = if nibble & 0x08 != 0 {
            i32::from(self.adpcm_signal) - difference
        } else {
            i32::from(self.adpcm_signal) + difference
        }
        .clamp(-2048, 2047);
        let next_step =
            i16::from(self.adpcm_step) + i16::from(INDEX_SHIFT[usize::from(nibble & 7)]);
        self.adpcm_signal = signal as i16;
        self.adpcm_step = next_step.clamp(0, 48) as u8;
        self.adpcm_current = ((signal & !3) * 8) as i16;
    }

    fn tick_adpcm_decoder(&mut self, clocks: u32) {
        if self.adpcm_status & ADPCM_PLAY_FLAG == 0 {
            return;
        }
        self.adpcm_clock_phase = self
            .adpcm_clock_phase
            .saturating_add(u64::from(clocks) * ADPCM_NIBBLE_RATE);
        let threshold = CPU_CLOCK_HZ * u64::from(self.adpcm_clock_divider.max(1));
        while self.adpcm_clock_phase >= threshold && self.adpcm_status & ADPCM_PLAY_FLAG != 0 {
            self.adpcm_clock_phase -= threshold;
            let byte = self.adpcm_ram[usize::from(self.adpcm_play_addr)];
            let low = self.adpcm_low_nibble;
            let nibble = if low { byte & 0x0f } else { byte >> 4 };
            self.decode_adpcm_nibble(nibble);
            if low {
                self.adpcm_low_nibble = false;
                if self.adpcm_play_addr == self.adpcm_end_addr {
                    self.stop_adpcm_playback(true);
                } else {
                    self.adpcm_play_addr = self.adpcm_play_addr.wrapping_add(1);
                }
            } else {
                self.adpcm_low_nibble = true;
            }
        }
    }

    fn finish_cdda_playback(&mut self) {
        if self.cdda_play_mode == 1 && self.cdda_start_lba < self.cdda_end_lba {
            self.cdda_lba = self.cdda_start_lba;
            self.cdda_sample_index = CDDA_FRAMES_PER_SECTOR;
            self.cdda_source_phase = 0;
            return;
        }
        if self.cdda_play_mode == 2 {
            self.irq_status |= CD_IRQ_TRANSFER_DONE;
        }
        self.cdda_status = CDDA_OFF;
        self.cdda_current = [0; 2];
    }

    fn load_cdda_sector(&mut self) -> bool {
        if self.cdda_lba >= self.cdda_end_lba || self.cdda_lba >= self.disc.sectors {
            self.finish_cdda_playback();
            return false;
        }
        if self
            .disc
            .track_for_lba(self.cdda_lba)
            .is_none_or(|track| track.kind != TrackKind::Audio)
        {
            self.finish_cdda_playback();
            return false;
        }
        match self
            .disc
            .read_raw_sector(self.cdda_lba, self.cdda_sector.as_mut())
        {
            Ok(()) => {
                self.cdda_lba = self.cdda_lba.saturating_add(1);
                self.cdda_sample_index = 0;
                true
            }
            Err(error) if error == RESOURCE_PENDING => false,
            Err(_) => {
                self.cdda_status = CDDA_OFF;
                self.cdda_current = [0; 2];
                false
            }
        }
    }

    fn next_cdda_source_sample(&mut self) -> Option<[i16; 2]> {
        if self.cdda_sample_index >= CDDA_FRAMES_PER_SECTOR && !self.load_cdda_sector() {
            return None;
        }
        let offset = self.cdda_sample_index * 4;
        let sample = [
            i16::from_le_bytes([self.cdda_sector[offset], self.cdda_sector[offset + 1]]),
            i16::from_le_bytes([self.cdda_sector[offset + 2], self.cdda_sector[offset + 3]]),
        ];
        self.cdda_sample_index += 1;
        Some(sample)
    }

    fn tick_cdda(&mut self, clocks: u32) {
        self.cdda_fade.tick(clocks);
        self.adpcm_fade.tick(clocks);
        self.tick_adpcm_decoder(clocks);
        self.cdda_clock_phase = self
            .cdda_clock_phase
            .saturating_add(u64::from(clocks) * u64::from(AUDIO_RATE));
        while self.cdda_clock_phase >= CPU_CLOCK_HZ {
            self.cdda_clock_phase -= CPU_CLOCK_HZ;
            let cdda_gain = self.cdda_fade.factor();
            if self.cdda_status == CDDA_PLAYING {
                if self.cdda_sample_index >= CDDA_FRAMES_PER_SECTOR {
                    self.cdda_current = self.next_cdda_source_sample().unwrap_or([0; 2]);
                }
                self.cdda_samples.push((
                    f32::from(self.cdda_current[0]) / 32768.0 * cdda_gain,
                    f32::from(self.cdda_current[1]) / 32768.0 * cdda_gain,
                ));
                self.cdda_source_phase = self.cdda_source_phase.saturating_add(CDDA_RATE);
                while self.cdda_source_phase >= AUDIO_RATE {
                    self.cdda_source_phase -= AUDIO_RATE;
                    self.cdda_current = self.next_cdda_source_sample().unwrap_or([0; 2]);
                }
            } else {
                self.cdda_samples.push((0.0, 0.0));
            }

            let adpcm_sample =
                f32::from(self.adpcm_current) / 32768.0 * 0.5 * self.adpcm_fade.factor();
            self.adpcm_samples.push((adpcm_sample, adpcm_sample));
        }
    }

    fn tick(&mut self, clocks: u32) {
        if self.phase == PceCdPhase::DataIn
            && self.data.is_empty()
            && self.read_remaining != 0
            && !self.req
        {
            self.try_load_read_sector();
        }
        if self.adpcm_dma_control & 3 != 0 && self.phase == PceCdPhase::DataIn {
            let mut transferred = false;
            for _ in 0..32 {
                if !self.req || self.data_index >= self.data.len() {
                    break;
                }
                let value = self.read_cd_data_port();
                self.adpcm_ram[usize::from(self.adpcm_write_ptr)] = value;
                self.adpcm_write_ptr = self.adpcm_write_ptr.wrapping_add(1);
                transferred = true;
            }
            if transferred {
                self.adpcm_status &= !0x04;
            }
            if self.phase != PceCdPhase::DataIn {
                self.adpcm_dma_control &= !3;
            }
        }
        self.tick_cdda(clocks);
    }
}

impl PceCdInterface {
    fn save(&self, out: &mut StateWriter) {
        out.blob(self.cd_ram.as_slice());
        out.blob(self.super_ram.as_slice());
        out.blob(self.backup_ram.as_slice());
        out.blob(self.adpcm_ram.as_slice());
        self.arcade_card.save(out);
        out.u8(self.backup_locked as u8);
        out.u8(self.phase.encode());
        out.u8(self.cdc_data);
        out.u8(self.irq_mask);
        out.u8(self.irq_status);
        out.u8(self.reset_reg);
        out.u8(self.bram_status);
        out.u8(self.busy as u8);
        out.u8(self.req as u8);
        out.u8(self.cd as u8);
        out.u8(self.io as u8);
        out.u8(self.msg as u8);
        out.u8(self.ack as u8);
        out.blob(&self.command);
        out.u8(self.command_len as u8);
        out.u8(self.command_expected as u8);
        out.blob(&self.data);
        out.u32(self.data_index as u32);
        out.u32(self.read_lba);
        out.u16(self.read_remaining);
        out.u8(self.cdda_status);
        out.u8(self.cdda_play_mode);
        out.u32(self.cdda_start_lba);
        out.u32(self.cdda_end_lba);
        out.u32(self.cdda_lba);
        out.u16(self.cdda_sample_index as u16);
        out.u64(self.cdda_clock_phase);
        out.u32(self.cdda_source_phase);
        out.blob(self.cdda_sector.as_slice());
        out.u16(self.cdda_current[0] as u16);
        out.u16(self.cdda_current[1] as u16);
        self.cdda_fade.save(out);
        out.u16(self.adpcm_latch);
        out.u16(self.adpcm_read_ptr);
        out.u8(self.adpcm_read_delay);
        out.u16(self.adpcm_write_ptr);
        out.u8(self.adpcm_write_delay);
        out.u16(self.adpcm_length);
        out.u16(self.adpcm_play_addr);
        out.u16(self.adpcm_end_addr);
        out.u8(self.adpcm_low_nibble as u8);
        out.u8(self.adpcm_clock_divider);
        out.u64(self.adpcm_clock_phase);
        out.u16(self.adpcm_signal as u16);
        out.u8(self.adpcm_step);
        out.u16(self.adpcm_current as u16);
        self.adpcm_fade.save(out);
        out.u8(self.adpcm_control);
        out.u8(self.adpcm_dma_control);
        out.u8(self.adpcm_status);
        out.u8(self.fader_control);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let cd_ram = input.blob()?;
        if cd_ram.len() != CD_RAM_SIZE {
            return Err("PC Engine CD state has invalid CD RAM length".into());
        }
        self.cd_ram.copy_from_slice(cd_ram);
        let super_ram = input.blob()?;
        if super_ram.len() != SUPER_CD_RAM_SIZE {
            return Err("PC Engine CD state has invalid Super CD RAM length".into());
        }
        self.super_ram.copy_from_slice(super_ram);
        let backup_ram = input.blob()?;
        if backup_ram.len() != BACKUP_RAM_SIZE {
            return Err("PC Engine CD state has invalid backup RAM length".into());
        }
        self.backup_ram.copy_from_slice(backup_ram);
        let adpcm_ram = input.blob()?;
        if adpcm_ram.len() != ADPCM_RAM_SIZE {
            return Err("PC Engine CD state has invalid ADPCM RAM length".into());
        }
        self.adpcm_ram.copy_from_slice(adpcm_ram);
        self.arcade_card.load(input)?;
        self.backup_locked = input.u8()? != 0;
        self.phase = PceCdPhase::decode(input.u8()?)?;
        self.cdc_data = input.u8()?;
        self.irq_mask = input.u8()?;
        self.irq_status = input.u8()?;
        self.reset_reg = input.u8()?;
        self.bram_status = input.u8()?;
        self.busy = input.u8()? != 0;
        self.req = input.u8()? != 0;
        self.cd = input.u8()? != 0;
        self.io = input.u8()? != 0;
        self.msg = input.u8()? != 0;
        self.ack = input.u8()? != 0;
        let command = input.blob()?;
        if command.len() != self.command.len() {
            return Err("PC Engine CD state has invalid command buffer length".into());
        }
        self.command.copy_from_slice(command);
        self.command_len = usize::from(input.u8()?);
        self.command_expected = usize::from(input.u8()?);
        if self.command_len > self.command.len() || self.command_expected > self.command.len() {
            return Err("PC Engine CD state has invalid command position".into());
        }
        self.data = input.blob()?.to_vec();
        if self.data.len() > 2048 {
            return Err("PC Engine CD state has oversized transfer buffer".into());
        }
        self.data_index = input.u32()? as usize;
        if self.data_index > self.data.len() {
            return Err("PC Engine CD state has invalid transfer position".into());
        }
        self.read_lba = input.u32()?;
        self.read_remaining = input.u16()?;
        self.cdda_status = input.u8()?;
        if self.cdda_status > CDDA_PAUSED {
            return Err("PC Engine CD state has invalid CD-DA status".into());
        }
        self.cdda_play_mode = input.u8()? & 3;
        self.cdda_start_lba = input.u32()?;
        self.cdda_end_lba = input.u32()?;
        self.cdda_lba = input.u32()?;
        self.cdda_sample_index = usize::from(input.u16()?);
        if self.cdda_sample_index > CDDA_FRAMES_PER_SECTOR {
            return Err("PC Engine CD state has invalid CD-DA sample position".into());
        }
        self.cdda_clock_phase = input.u64()? % CPU_CLOCK_HZ;
        self.cdda_source_phase = input.u32()? % AUDIO_RATE;
        let cdda_sector = input.blob()?;
        if cdda_sector.len() != self.cdda_sector.len() {
            return Err("PC Engine CD state has invalid CD-DA sector length".into());
        }
        self.cdda_sector.copy_from_slice(cdda_sector);
        self.cdda_current = [input.u16()? as i16, input.u16()? as i16];
        self.cdda_samples.clear();
        self.cdda_fade = PceFade::load(input)?;
        self.adpcm_latch = input.u16()?;
        self.adpcm_read_ptr = input.u16()?;
        self.adpcm_read_delay = input.u8()?;
        if self.adpcm_read_delay > 2 {
            return Err("PC Engine CD state has invalid ADPCM read delay".into());
        }
        self.adpcm_write_ptr = input.u16()?;
        self.adpcm_write_delay = input.u8()?;
        if self.adpcm_write_delay > 1 {
            return Err("PC Engine CD state has invalid ADPCM write delay".into());
        }
        self.adpcm_length = input.u16()?;
        self.adpcm_play_addr = input.u16()?;
        self.adpcm_end_addr = input.u16()?;
        self.adpcm_low_nibble = input.u8()? != 0;
        self.adpcm_clock_divider = input.u8()?;
        if !(1..=16).contains(&self.adpcm_clock_divider) {
            return Err("PC Engine CD state has invalid ADPCM clock divider".into());
        }
        let adpcm_threshold = CPU_CLOCK_HZ * u64::from(self.adpcm_clock_divider);
        self.adpcm_clock_phase = input.u64()? % adpcm_threshold;
        self.adpcm_signal = input.u16()? as i16;
        if !(-2048..=2047).contains(&self.adpcm_signal) {
            return Err("PC Engine CD state has invalid ADPCM decoder signal".into());
        }
        self.adpcm_step = input.u8()?;
        if self.adpcm_step > 48 {
            return Err("PC Engine CD state has invalid ADPCM decoder step".into());
        }
        self.adpcm_current = input.u16()? as i16;
        self.adpcm_samples.clear();
        self.adpcm_fade = PceFade::load(input)?;
        self.adpcm_control = input.u8()?;
        self.adpcm_dma_control = input.u8()?;
        self.adpcm_status = input.u8()?;
        self.fader_control = input.u8()?;
        Ok(())
    }
}

struct PceBus {
    rom: Vec<u8>,
    sf2_bank: u8,
    populous_ram: Option<Box<[u8; POPULOUS_RAM_SIZE]>>,
    ram: [u8; 0x2000],
    vdc: HuC6270,
    vdc2: Option<HuC6270>,
    vpc: Option<HuC6202>,
    video: VideoBuffer,
    vce: HuC6260,
    psg: PcePsg,
    joy_output: u8,
    joy_six_button_phase: bool,
    input: InputState,
    cd: Option<PceCdInterface>,
}

impl PceBus {
    fn from_rom(image: &[u8]) -> Result<Self, String> {
        Self::from_hucard(image, false)
    }

    fn from_supergrafx_rom(image: &[u8]) -> Result<Self, String> {
        Self::from_hucard(image, true)
    }

    fn from_hucard(image: &[u8], supergrafx: bool) -> Result<Self, String> {
        let system = if supergrafx {
            "SuperGrafx"
        } else {
            "PC Engine"
        };
        if image.is_empty() {
            return Err(format!("{system} requires a non-empty HuCard image"));
        }
        if image.len() > MAX_HUCARD {
            return Err(format!(
                "{system} HuCard exceeds {MAX_HUCARD} byte staging limit"
            ));
        }
        let image = if image.len() > 512 && image.len() % 0x2000 == 512 {
            &image[512..]
        } else {
            image
        };
        if image.is_empty() || (image.len() > STANDARD_HUCARD_MAX && image.len() != SF2_HUCARD_SIZE)
        {
            return Err(format!(
                "{system} HuCard payload must be at most {STANDARD_HUCARD_MAX} bytes or the {SF2_HUCARD_SIZE}-byte Street Fighter II layout"
            ));
        }
        let populous = pce_crc32_ieee(image) == POPULOUS_HUCARD_CRC32;
        Ok(Self {
            rom: image.to_vec(),
            sf2_bank: 0,
            populous_ram: populous.then(|| Box::new([0; POPULOUS_RAM_SIZE])),
            ram: [0; 0x2000],
            vdc: HuC6270::default(),
            vdc2: supergrafx.then(HuC6270::default),
            vpc: supergrafx.then(HuC6202::default),
            video: VideoBuffer::new(WIDTH, HEIGHT),
            vce: HuC6260::default(),
            psg: PcePsg::default(),
            joy_output: 3,
            joy_six_button_phase: false,
            input: InputState::default(),
            cd: None,
        })
    }

    fn from_bios_and_disc(bios: &[u8], disc: ResourceBlob) -> Result<Self, String> {
        let bios = if bios.len() == SYSTEM_CARD_SIZE + 512 {
            &bios[512..]
        } else {
            bios
        };
        if bios.len() != SYSTEM_CARD_SIZE {
            return Err(format!(
                "PC Engine CD System Card BIOS must be {SYSTEM_CARD_SIZE} bytes; got {}",
                bios.len()
            ));
        }
        let bios_crc = pce_crc32_ieee(bios);
        Ok(Self {
            rom: bios.to_vec(),
            sf2_bank: 0,
            populous_ram: None,
            ram: [0; 0x2000],
            vdc: HuC6270::default(),
            vdc2: None,
            vpc: None,
            video: VideoBuffer::new(WIDTH, HEIGHT),
            vce: HuC6260::default(),
            psg: PcePsg::default(),
            joy_output: 3,
            joy_six_button_phase: false,
            input: InputState::default(),
            cd: Some(PceCdInterface::new(disc, bios_crc)?),
        })
    }

    fn reset(&mut self) {
        self.sf2_bank = 0;
        self.ram = [0; 0x2000];
        self.vdc = HuC6270::default();
        if self.vdc2.is_some() {
            self.vdc2 = Some(HuC6270::default());
            self.vpc = Some(HuC6202::default());
        }
        self.video.clear([0, 0, 0, 255]);
        self.vce = HuC6260::default();
        self.psg = PcePsg::default();
        self.joy_output = 3;
        self.joy_six_button_phase = false;
        self.input = InputState::default();
        if let Some(cd) = &mut self.cd {
            cd.reset_runtime();
        }
    }

    fn set_input(&mut self, input: &InputState) {
        self.input = input.clone();
    }

    fn six_button_pad(&self) -> bool {
        self.cd.is_none() && self.rom.len() == SF2_HUCARD_SIZE
    }

    fn write_joy_output(&mut self, value: u8) {
        let next = value & 3;
        if self.six_button_pad() {
            if self.joy_output & 0x02 == 0 && next & 0x02 != 0 {
                self.joy_six_button_phase = !self.joy_six_button_phase;
            }
        } else {
            self.joy_six_button_phase = false;
        }
        self.joy_output = next;
    }

    fn joy_read(&self) -> u8 {
        if self.joy_output & 0x02 != 0 {
            return 0x30;
        }
        let buttons = self.input.buttons[0];
        let mut nibble = 0x0f;
        if self.six_button_pad() && self.joy_six_button_phase {
            if self.joy_output & 0x01 != 0 {
                return 0x30;
            }
            if buttons & FACE_WEST != 0 {
                nibble &= !0x01;
            }
            if buttons & FACE_NORTH != 0 {
                nibble &= !0x02;
            }
            if buttons & L1 != 0 {
                nibble &= !0x04;
            }
            if buttons & R1 != 0 {
                nibble &= !0x08;
            }
        } else if self.joy_output & 0x01 != 0 {
            if buttons & UP != 0 {
                nibble &= !0x01;
            }
            if buttons & RIGHT != 0 {
                nibble &= !0x02;
            }
            if buttons & DOWN != 0 {
                nibble &= !0x04;
            }
            if buttons & LEFT != 0 {
                nibble &= !0x08;
            }
        } else {
            if buttons & FACE_SOUTH != 0 {
                nibble &= !0x01;
            }
            if buttons & FACE_EAST != 0 {
                nibble &= !0x02;
            }
            if buttons & SELECT != 0 {
                nibble &= !0x04;
            }
            if buttons & START != 0 {
                nibble &= !0x08;
            }
        }
        0x30 | nibble
    }

    fn hucard_read(&self, address: u32) -> u8 {
        let address = address as usize;
        if self.rom.len() == SF2_HUCARD_SIZE && address >= 0x080000 {
            let bank_base = 0x080000 + usize::from(self.sf2_bank) * 0x080000;
            return self.rom[bank_base + (address & 0x07ffff)];
        }
        self.rom[address % self.rom.len()]
    }

    fn compose_video(&mut self) {
        if let (Some(vdc2), Some(vpc)) = (self.vdc2.as_ref(), self.vpc.as_ref()) {
            let output = self.video.pixels_mut();
            for index in 0..self.vdc.pixel_codes.len() {
                let x = index % WIDTH as usize;
                let code =
                    vpc.compose_code(x, self.vdc.pixel_codes[index], vdc2.pixel_codes[index]);
                let rgba = self.vce.rgba(usize::from(code & 0x01ff));
                let offset = index * 4;
                output[offset..offset + 4].copy_from_slice(&rgba);
            }
        } else {
            self.video
                .pixels_mut()
                .copy_from_slice(self.vdc.video.pixels());
        }
    }

    fn tick(&mut self, clocks: u32) {
        self.psg.tick(clocks);
        self.vdc.tick(clocks, &self.vce);
        if let Some(vdc2) = &mut self.vdc2 {
            vdc2.tick(clocks, &self.vce);
        }
        if let Some(cd) = &mut self.cd {
            cd.tick(clocks);
        }
    }

    fn save(&self, out: &mut StateWriter) {
        out.blob(&self.ram);
        out.u8(self.sf2_bank);
        out.u8(u8::from(self.populous_ram.is_some()));
        if let Some(populous_ram) = &self.populous_ram {
            out.blob(populous_ram.as_slice());
        }
        self.vce.save(out);
        self.vdc.save(out);
        out.u8(u8::from(self.vdc2.is_some()));
        if let (Some(vdc2), Some(vpc)) = (&self.vdc2, &self.vpc) {
            vdc2.save(out);
            vpc.save(out);
        }
        self.psg.save(out);
        out.u8(self.joy_output);
        out.u8(u8::from(self.joy_six_button_phase));
        out.u8(self.cd.is_some() as u8);
        if let Some(cd) = &self.cd {
            cd.save(out);
        }
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let ram = input.blob()?;
        if ram.len() != self.ram.len() {
            return Err("PC Engine state has invalid work RAM length".into());
        }
        self.ram.copy_from_slice(ram);
        self.sf2_bank = input.u8()?;
        if self.sf2_bank > 3 || (self.rom.len() != SF2_HUCARD_SIZE && self.sf2_bank != 0) {
            return Err("PC Engine state has invalid Street Fighter II bank state".into());
        }
        let state_has_populous_ram = input.u8()? != 0;
        if state_has_populous_ram != self.populous_ram.is_some() {
            return Err("PC Engine state HuCard RAM profile differs from machine".into());
        }
        if let Some(populous_ram) = &mut self.populous_ram {
            let saved_ram = input.blob()?;
            if saved_ram.len() != POPULOUS_RAM_SIZE {
                return Err("PC Engine state has invalid Populous RAM length".into());
            }
            populous_ram.copy_from_slice(saved_ram);
        }
        self.vce.load(input)?;
        self.vdc.load(input, &self.vce)?;
        let state_has_vpc = input.u8()? != 0;
        if state_has_vpc != self.vdc2.is_some() || state_has_vpc != self.vpc.is_some() {
            return Err("PC Engine state video profile differs from machine".into());
        }
        if let (Some(vdc2), Some(vpc)) = (&mut self.vdc2, &mut self.vpc) {
            vdc2.load(input, &self.vce)?;
            vpc.load(input)?;
        }
        self.psg.load(input)?;
        self.joy_output = input.u8()? & 3;
        self.joy_six_button_phase = input.u8()? != 0;
        if self.joy_six_button_phase && !self.six_button_pad() {
            return Err(
                "PC Engine state has six-button phase on a two-button controller profile".into(),
            );
        }
        let state_has_cd = input.u8()? != 0;
        if state_has_cd != self.cd.is_some() {
            return Err("PC Engine state media profile differs from machine".into());
        }
        if let Some(cd) = &mut self.cd {
            cd.load(input)?;
        }
        self.compose_video();
        Ok(())
    }
}

impl Huc6280Bus for PceBus {
    fn read8(&mut self, address: u32) -> u8 {
        if self.cd.is_none() && (0x080000..=0x087fff).contains(&address) {
            if let Some(populous_ram) = &self.populous_ram {
                return populous_ram[address as usize - 0x080000];
            }
        }
        if let Some(cd) = &mut self.cd {
            match address {
                0x000000..=0x07ffff => return self.rom[address as usize % self.rom.len()],
                0x080000..=0x087fff => {
                    return cd.arcade_card.read_window(address as usize - 0x080000)
                }
                0x0d0000..=0x0fffff if cd.super_system => {
                    return cd.super_ram[address as usize - 0x0d0000]
                }
                0x100000..=0x10ffff => return cd.cd_ram[address as usize - 0x100000],
                0x1ee000..=0x1ee7ff => return cd.backup_read(address as usize - 0x1ee000),
                0x1ffa00..=0x1ffaff => return cd.arcade_card.read_reg((address & 0xff) as u8),
                0x1ff800..=0x1ffbff => {
                    let canonical = address & !0x330;
                    if cd.super_system && (0x1ff8c0..=0x1ff8c7).contains(&canonical) {
                        return cd.system_card_register((canonical - 0x1ff8c0) as u8);
                    }
                    return cd.read_reg((address & 0x0f) as u8);
                }
                _ => {}
            }
        } else if address <= 0x0fffff {
            return self.hucard_read(address);
        }
        match address {
            0x1f0000..=0x1f7fff => self.ram[address as usize & 0x1fff],
            0x1fe000..=0x1fe3ff if self.vpc.is_some() => match address & 0x1f {
                0x00..=0x07 => self.vdc.read(address),
                0x08..=0x0f => self.vpc.as_ref().unwrap().read(address),
                0x10..=0x17 => self.vdc2.as_mut().unwrap().read(address),
                _ => 0xff,
            },
            0x1fe000..=0x1fe3ff => self.vdc.read(address),
            0x1fe400..=0x1fe7ff => self.vce.read(address),
            0x1fe800..=0x1febff => self.psg.read(address),
            0x1ff000..=0x1ff3ff => self.joy_read(),
            _ => 0xff,
        }
    }

    fn write8(&mut self, address: u32, value: u8) {
        if self.cd.is_none() && (0x080000..=0x087fff).contains(&address) {
            if let Some(populous_ram) = &mut self.populous_ram {
                populous_ram[address as usize - 0x080000] = value;
                return;
            }
        }
        if self.cd.is_none()
            && self.rom.len() == SF2_HUCARD_SIZE
            && (0x001ff0..=0x001ff3).contains(&address)
        {
            self.sf2_bank = (address - 0x001ff0) as u8;
            return;
        }
        if let Some(cd) = &mut self.cd {
            match address {
                0x080000..=0x087fff => {
                    cd.arcade_card
                        .write_window(address as usize - 0x080000, value);
                    return;
                }
                0x0d0000..=0x0fffff if cd.super_system => {
                    cd.super_ram[address as usize - 0x0d0000] = value;
                    return;
                }
                0x100000..=0x10ffff => {
                    cd.cd_ram[address as usize - 0x100000] = value;
                    return;
                }
                0x1ee000..=0x1ee7ff => {
                    cd.backup_write(address as usize - 0x1ee000, value);
                    return;
                }
                0x1ffa00..=0x1ffaff => {
                    cd.arcade_card.write_reg((address & 0xff) as u8, value);
                    return;
                }
                0x1ff800..=0x1ffbff => {
                    cd.write_reg((address & 0x0f) as u8, value);
                    return;
                }
                _ => {}
            }
        }
        match address {
            0x1f0000..=0x1f7fff => self.ram[address as usize & 0x1fff] = value,
            0x1fe000..=0x1fe3ff if self.vpc.is_some() => match address & 0x1f {
                0x00..=0x07 => self.vdc.write(address, value),
                0x08..=0x0f => self.vpc.as_mut().unwrap().write(address, value),
                0x10..=0x17 => self.vdc2.as_mut().unwrap().write(address, value),
                _ => {}
            },
            0x1fe000..=0x1fe3ff => self.vdc.write(address, value),
            0x1fe400..=0x1fe7ff => self.vce.write(address, value),
            0x1fe800..=0x1febff => self.psg.write(address, value),
            0x1ff000..=0x1ff3ff => self.write_joy_output(value),
            _ => {}
        }
    }

    fn write_st(&mut self, port: u8, value: u8) {
        let use_secondary = self.vpc.as_ref().is_some_and(|vpc| vpc.io_device);
        if use_secondary {
            self.vdc2.as_mut().unwrap().write(u32::from(port), value);
        } else {
            self.vdc.write(u32::from(port), value);
        }
    }
}

pub struct PcEngineMachine {
    platform: PlatformId,
    cpu: HuC6280,
    bus: PceBus,
    audio: AudioBuffer,
}

impl PcEngineMachine {
    pub fn from_rom(image: &[u8]) -> Result<Self, String> {
        Ok(Self::from_bus(
            PlatformId::PcEngine,
            PceBus::from_rom(image)?,
        ))
    }

    pub fn from_supergrafx_rom(image: &[u8]) -> Result<Self, String> {
        Ok(Self::from_bus(
            PlatformId::SuperGrafx,
            PceBus::from_supergrafx_rom(image)?,
        ))
    }

    pub fn from_bios_and_disc(bios: &[u8], disc: ResourceBlob) -> Result<Self, String> {
        Ok(Self::from_bus(
            PlatformId::PcEngine,
            PceBus::from_bios_and_disc(bios, disc)?,
        ))
    }

    fn from_bus(platform: PlatformId, mut bus: PceBus) -> Self {
        let mut cpu = HuC6280::default();
        cpu.reset(&mut bus);
        Self {
            platform,
            cpu,
            bus,
            audio: AudioBuffer::new(AUDIO_RATE, 2),
        }
    }

    fn flush_audio(&mut self) {
        self.audio.begin_frame();
        let cdda = self
            .bus
            .cd
            .as_ref()
            .map(|cd| cd.cdda_samples.as_slice())
            .unwrap_or(&[]);
        let adpcm = self
            .bus
            .cd
            .as_ref()
            .map(|cd| cd.adpcm_samples.as_slice())
            .unwrap_or(&[]);
        let frames = self.bus.psg.samples.len().max(cdda.len()).max(adpcm.len());
        for index in 0..frames {
            let (psg_left, psg_right) =
                self.bus.psg.samples.get(index).copied().unwrap_or_default();
            let (cdda_left, cdda_right) = cdda.get(index).copied().unwrap_or_default();
            let (adpcm_left, adpcm_right) = adpcm.get(index).copied().unwrap_or_default();
            self.audio.push_stereo(
                (psg_left + cdda_left + adpcm_left).clamp(-1.0, 1.0),
                (psg_right + cdda_right + adpcm_right).clamp(-1.0, 1.0),
            );
        }
    }
}

impl Machine for PcEngineMachine {
    fn platform(&self) -> PlatformId {
        self.platform
    }

    fn reset(&mut self) {
        self.bus.reset();
        self.cpu.reset(&mut self.bus);
        self.audio.begin_frame();
    }

    fn run_frame(&mut self, input: &InputState) {
        self.bus.set_input(input);
        self.bus.psg.begin_frame();
        if let Some(cd) = &mut self.bus.cd {
            cd.begin_frame();
        }
        let target = self.bus.vdc.frame.wrapping_add(1);
        let mut instructions = 0usize;
        while self.bus.vdc.frame != target && instructions < 1_000_000 {
            let clocks = self.cpu.step(&mut self.bus);
            self.bus.tick(clocks);
            let video_irq = self.bus.vdc.irq_pending
                || self.bus.vdc2.as_ref().is_some_and(|vdc| vdc.irq_pending);
            if video_irq {
                self.cpu.request_irq(HucInterrupt::Irq1);
            } else {
                self.cpu.clear_irq(HucInterrupt::Irq1);
            }
            if self
                .bus
                .cd
                .as_ref()
                .is_some_and(PceCdInterface::irq_pending)
            {
                self.cpu.request_irq(HucInterrupt::Irq2);
            } else {
                self.cpu.clear_irq(HucInterrupt::Irq2);
            }
            instructions += 1;
        }
        self.bus.compose_video();
        self.flush_audio();
    }

    fn frame_rate(&self) -> f64 {
        FRAME_RATE
    }
    fn video(&self) -> &VideoBuffer {
        &self.bus.video
    }
    fn audio(&self) -> &AudioBuffer {
        &self.audio
    }

    fn save_state(&self) -> Result<Vec<u8>, String> {
        let mut out = StateWriter::new(self.platform, STATE_VERSION);
        self.cpu.save(&mut out);
        self.bus.save(&mut out);
        Ok(out.finish())
    }

    fn load_state(&mut self, bytes: &[u8]) -> Result<(), String> {
        let mut input = StateReader::new(bytes, self.platform, STATE_VERSION)?;
        self.cpu.load(&mut input)?;
        self.bus.load(&mut input)?;
        input.finish()?;
        self.audio.begin_frame();
        Ok(())
    }

    fn persistent_len(&self, kind: ResourceKind, slot: u32) -> usize {
        if kind == ResourceKind::Storage && slot == 0 && self.bus.cd.is_some() {
            BACKUP_RAM_SIZE
        } else {
            0
        }
    }

    fn read_persistent(&self, kind: ResourceKind, slot: u32, out: &mut [u8]) -> Result<(), String> {
        let len = self.persistent_len(kind, slot);
        if len == 0 {
            return Err("PC Engine backup storage slot is unavailable".into());
        }
        if out.len() != len {
            return Err(format!(
                "persistent output has {} bytes; expected {len}",
                out.len()
            ));
        }
        out.copy_from_slice(self.bus.cd.as_ref().unwrap().backup_ram.as_slice());
        Ok(())
    }

    fn write_persistent(
        &mut self,
        kind: ResourceKind,
        slot: u32,
        data: &[u8],
    ) -> Result<(), String> {
        let len = self.persistent_len(kind, slot);
        if len == 0 {
            return Err("PC Engine backup storage slot is unavailable".into());
        }
        if data.len() != len {
            return Err(format!(
                "persistent input has {} bytes; expected {len}",
                data.len()
            ));
        }
        self.bus
            .cd
            .as_mut()
            .unwrap()
            .backup_ram
            .copy_from_slice(data);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn synthetic_disc(sectors: usize) -> Vec<u8> {
        let mut disc = vec![0; sectors * 2048];
        for sector in 0..sectors {
            disc[sector * 2048..(sector + 1) * 2048].fill((sector as u8).wrapping_add(0x40));
        }
        disc
    }

    fn cue_container(cue: &str, files: &[(&str, Vec<u8>)]) -> ResourceBlob {
        let cue_bytes = cue.as_bytes();
        let directory_len: usize = files.iter().map(|(name, _)| 2 + name.len() + 16).sum();
        let data_start = 16 + cue_bytes.len() + directory_len;
        let total_len = data_start + files.iter().map(|(_, data)| data.len()).sum::<usize>();
        let mut bytes = vec![0u8; total_len];
        bytes[..8].copy_from_slice(&crate::cd_image::CUE_CONTAINER_MAGIC);
        bytes[8..12].copy_from_slice(&(cue_bytes.len() as u32).to_le_bytes());
        bytes[12..16].copy_from_slice(&(files.len() as u32).to_le_bytes());
        bytes[16..16 + cue_bytes.len()].copy_from_slice(cue_bytes);
        let mut directory_cursor = 16 + cue_bytes.len();
        let mut data_cursor = data_start;
        for (name, data) in files {
            let name_bytes = name.as_bytes();
            bytes[directory_cursor..directory_cursor + 2]
                .copy_from_slice(&(name_bytes.len() as u16).to_le_bytes());
            directory_cursor += 2;
            bytes[directory_cursor..directory_cursor + name_bytes.len()]
                .copy_from_slice(name_bytes);
            directory_cursor += name_bytes.len();
            bytes[directory_cursor..directory_cursor + 8]
                .copy_from_slice(&(data.len() as u64).to_le_bytes());
            bytes[directory_cursor + 8..directory_cursor + 16]
                .copy_from_slice(&(data_cursor as u64).to_le_bytes());
            directory_cursor += 16;
            bytes[data_cursor..data_cursor + data.len()].copy_from_slice(data);
            data_cursor += data.len();
        }
        ResourceBlob::from_bytes(&bytes)
    }

    fn cd_send_command(cd: &mut PceCdInterface, command: &[u8]) {
        const MASK: u8 = CD_IRQ_TRANSFER_READY | CD_IRQ_TRANSFER_DONE;
        cd.write_reg(0x00, 0);
        cd.write_reg(0x02, MASK);
        for &byte in command {
            assert_ne!(cd.read_reg(0x00) & 0x40, 0);
            cd.write_reg(0x01, byte);
            cd.write_reg(0x02, MASK | 0x80);
            cd.write_reg(0x02, MASK);
        }
    }

    fn cd_finish_status(cd: &mut PceCdInterface) {
        const MASK: u8 = CD_IRQ_TRANSFER_READY | CD_IRQ_TRANSFER_DONE;
        assert_eq!(cd.phase, PceCdPhase::Status);
        assert_eq!(cd.read_reg(0x01), 0);
        cd.write_reg(0x02, MASK | 0x80);
        cd.write_reg(0x02, MASK);
        assert_eq!(cd.phase, PceCdPhase::MessageIn);
        assert_eq!(cd.read_reg(0x01), 0);
        cd.write_reg(0x02, MASK | 0x80);
        cd.write_reg(0x02, MASK);
        assert_eq!(cd.phase, PceCdPhase::BusFree);
    }

    fn synthetic_hucard() -> Vec<u8> {
        let mut rom = vec![0xea; 0x2000];
        let mut code = vec![
            0xd4, 0xa9, 0xf8, 0x53, 0x02, 0xa9, 0xff, 0x53, 0x40, 0xa9, 0x01, 0x8d, 0x02, 0xc4,
            0xa9, 0x00, 0x8d, 0x03, 0xc4, 0xa9, 0x38, 0x8d, 0x04, 0xc4, 0xa9, 0x00, 0x8d, 0x05,
            0xc4, 0x03, 0x00, 0x13, 0x00, 0x23, 0x00, 0x03, 0x02, 0x13, 0x01, 0x23, 0x00, 0x03,
            0x00, 0x13, 0x10, 0x23, 0x00, 0x03, 0x02,
        ];
        for _ in 0..8 {
            code.extend_from_slice(&[0x13, 0xff, 0x23, 0x00]);
        }
        for _ in 0..8 {
            code.extend_from_slice(&[0x13, 0x00, 0x23, 0x00]);
        }
        code.extend_from_slice(&[
            0x03, 0x05, 0x13, 0x80, 0x23, 0x00, 0xa9, 0x00, 0x8d, 0x00, 0xc8, 0xa9, 0xff, 0x8d,
            0x01, 0xc8, 0xa9, 0x20, 0x8d, 0x02, 0xc8, 0xa9, 0x00, 0x8d, 0x03, 0xc8, 0xa9, 0xff,
            0x8d, 0x05, 0xc8,
        ]);
        for index in 0..32 {
            code.extend_from_slice(&[0xa9, if index & 1 == 0 { 0 } else { 31 }, 0x8d, 0x06, 0xc8]);
        }
        code.extend_from_slice(&[0xa9, 0x9f, 0x8d, 0x04, 0xc8, 0x80, 0xfe]);
        rom[..code.len()].copy_from_slice(&code);
        rom[0x1ffe] = 0x00;
        rom[0x1fff] = 0xe0;
        rom
    }

    fn synthetic_sf2_hucard() -> Vec<u8> {
        let mut rom = vec![0; SF2_HUCARD_SIZE];
        rom[0x100] = 0x5a;
        for bank in 0..4usize {
            rom[0x080000 + bank * 0x080000] = 0x40 + bank as u8;
            rom[0x080123 + bank * 0x080000] = 0x80 + bank as u8;
        }
        rom
    }

    #[test]
    fn arcade_card_maps_ports_windows_shift_rotate_and_state() {
        let cd =
            PceCdInterface::new(ResourceBlob::from_bytes(&synthetic_disc(2)), 0x6d9a_73ef).unwrap();
        let mut bus = PceBus {
            rom: vec![0xea; SYSTEM_CARD_SIZE],
            sf2_bank: 0,
            populous_ram: None,
            ram: [0; 0x2000],
            vdc: HuC6270::default(),
            vdc2: None,
            vpc: None,
            video: VideoBuffer::new(WIDTH, HEIGHT),
            vce: HuC6260::default(),
            psg: PcePsg::default(),
            joy_output: 3,
            joy_six_button_phase: false,
            input: InputState::default(),
            cd: Some(cd),
        };

        assert_eq!(bus.read8(0x1ffafe), 0x10);
        assert_eq!(bus.read8(0x1ffaff), 0x51);
        assert_eq!(bus.read8(0x1ffa0a), 0);

        bus.write8(0x1ffa02, 0x34);
        bus.write8(0x1ffa03, 0x12);
        bus.write8(0x1ffa04, 0x00);
        bus.write8(0x1ffa07, 0x01);
        bus.write8(0x1ffa08, 0x00);
        bus.write8(0x1ffa09, 0x11);
        bus.write8(0x080000, 0x5a);
        bus.write8(0x080000, 0xa5);
        assert_eq!(bus.read8(0x1ffa02), 0x36);
        bus.write8(0x1ffa02, 0x34);
        bus.write8(0x1ffa09, 0x00);
        assert_eq!(bus.read8(0x080000), 0x5a);

        bus.write8(0x1ffa12, 0x00);
        bus.write8(0x1ffa13, 0x01);
        bus.write8(0x1ffa15, 0xff);
        bus.write8(0x1ffa16, 0xff);
        bus.write8(0x1ffa19, 0x0a);
        bus.write8(0x082000, 0x77);
        bus.write8(0x1ffa15, 0x00);
        bus.write8(0x1ffa16, 0x00);
        bus.write8(0x1ffa19, 0x00);
        bus.write8(0x1ffa12, 0xff);
        bus.write8(0x1ffa13, 0x00);
        assert_eq!(bus.read8(0x082000), 0x77);

        for (offset, value) in [0x78, 0x56, 0x34, 0x12].into_iter().enumerate() {
            bus.write8(0x1ffae0 + offset as u32, value);
        }
        bus.write8(0x1ffae4, 1);
        assert_eq!(bus.read8(0x1ffae0), 0xf0);
        assert_eq!(bus.read8(0x1ffae3), 0x24);
        bus.write8(0x1ffae5, 0x0f);
        assert_eq!(bus.read8(0x1ffae0), 0x78);
        assert_eq!(bus.read8(0x1ffae3), 0x12);

        let mut writer = StateWriter::new(PlatformId::PcEngine, 99);
        bus.cd.as_ref().unwrap().save(&mut writer);
        let state = writer.finish();
        let mut restored =
            PceCdInterface::new(ResourceBlob::from_bytes(&synthetic_disc(2)), 0x6d9a_73ef).unwrap();
        let mut reader = StateReader::new(&state, PlatformId::PcEngine, 99).unwrap();
        restored.load(&mut reader).unwrap();
        reader.finish().unwrap();
        assert_eq!(restored.arcade_card.read_reg(0xfe), 0x10);
        assert_eq!(restored.arcade_card.read_reg(0xff), 0x51);
        assert_eq!(restored.arcade_card.read_reg(0x00), 0x5a);
        assert_eq!(restored.arcade_card.read_reg(0x10), 0x77);
        assert_eq!(restored.arcade_card.read_reg(0xe0), 0x78);
        assert_eq!(restored.arcade_card.read_reg(0xe3), 0x12);
    }

    #[test]
    fn system_card_three_maps_super_ram_cd_ram_and_locked_backup_ram() {
        let cd =
            PceCdInterface::new(ResourceBlob::from_bytes(&synthetic_disc(2)), 0x6d9a_73ef).unwrap();
        assert!(cd.super_system);
        assert!(!cd.us_system_card);
        let mut bus = PceBus {
            rom: vec![0xea; SYSTEM_CARD_SIZE],
            sf2_bank: 0,
            populous_ram: None,
            ram: [0; 0x2000],
            vdc: HuC6270::default(),
            vdc2: None,
            vpc: None,
            video: VideoBuffer::new(WIDTH, HEIGHT),
            vce: HuC6260::default(),
            psg: PcePsg::default(),
            joy_output: 3,
            joy_six_button_phase: false,
            input: InputState::default(),
            cd: Some(cd),
        };

        bus.write8(0x0d0000, 0x5a);
        bus.write8(0x100000, 0xa5);
        assert_eq!(bus.read8(0x0d0000), 0x5a);
        assert_eq!(bus.read8(0x100000), 0xa5);
        assert_eq!(bus.read8(0x1ff8c1), 0xaa);
        assert_eq!(bus.read8(0x1ff8c2), 0x55);
        assert_eq!(bus.read8(0x1ff8c5), 0xaa);
        assert_eq!(bus.read8(0x1ff8c6), 0x55);
        assert_eq!(bus.read8(0x1ff8c7), 0x03);

        assert_eq!(bus.read8(0x1ee000), 0xff);
        bus.write8(0x1ee000, 0x33);
        assert_eq!(bus.read8(0x1ee000), 0xff);
        bus.write8(0x1ff807, 0x80);
        bus.write8(0x1ee000, 0x33);
        assert_eq!(bus.read8(0x1ee000), 0x33);
        let _ = bus.read8(0x1ff803);
        assert_eq!(bus.read8(0x1ee000), 0xff);
    }

    #[test]
    fn cd_read_six_transfers_sector_through_scsi_register_handshake() {
        let mut cd =
            PceCdInterface::new(ResourceBlob::from_bytes(&synthetic_disc(3)), 0x6d9a_73ef).unwrap();
        cd_send_command(&mut cd, &[0x08, 0x00, 0x00, 0x01, 0x01, 0x00]);
        assert_eq!(cd.phase, PceCdPhase::DataIn);
        assert_ne!(cd.irq_status & CD_IRQ_TRANSFER_READY, 0);
        assert!(cd.irq_pending());

        let mut sector = [0u8; 2048];
        for byte in &mut sector {
            *byte = cd.read_reg(0x08);
        }
        assert!(sector.iter().all(|byte| *byte == 0x41));
        assert_eq!(cd.phase, PceCdPhase::Status);
        assert_eq!(cd.irq_status & CD_IRQ_TRANSFER_READY, 0);
        assert_ne!(cd.irq_status & CD_IRQ_TRANSFER_DONE, 0);
        cd_finish_status(&mut cd);
    }

    #[test]
    fn adpcm_dma_moves_cd_data_and_clears_transfer_active_status() {
        let mut cd =
            PceCdInterface::new(ResourceBlob::from_bytes(&synthetic_disc(2)), 0x6d9a_73ef).unwrap();
        cd.adpcm_write_ptr = 0x2000;
        cd_send_command(&mut cd, &[0x08, 0x00, 0x00, 0x01, 0x01, 0x00]);
        assert_eq!(cd.phase, PceCdPhase::DataIn);
        cd.write_reg(0x0b, 1);
        assert_ne!(cd.adpcm_status & 0x04, 0);
        cd.tick(1);
        assert_eq!(cd.adpcm_status & 0x04, 0);
        assert_eq!(cd.adpcm_ram[0x2000], 0x41);
        assert_eq!(cd.adpcm_write_ptr, 0x2020);
        assert_ne!(cd.adpcm_dma_control & 3, 0);
    }

    #[test]
    fn cd_read_six_waits_for_streaming_sector_and_resumes_after_hydration() {
        let disc = ResourceBlob::streaming(2 * 2048, 2).unwrap();
        let mut hydration = disc.clone();
        hydration.write(0, &[0; 16]).unwrap();
        let mut cd = PceCdInterface::new(disc.clone(), 0x6d9a_73ef).unwrap();
        cd_send_command(&mut cd, &[0x08, 0x00, 0x00, 0x01, 0x01, 0x00]);
        assert_eq!(cd.phase, PceCdPhase::DataIn);
        assert!(!cd.req);
        assert_eq!(disc.pending_range(), Some((2048, 4096)));

        hydration.write(2048, &vec![0x7c; 2048]).unwrap();
        cd.tick(1);
        assert!(cd.req);
        assert_eq!(cd.read_reg(0x08), 0x7c);
        for _ in 1..2048 {
            let _ = cd.read_reg(0x08);
        }
        assert_eq!(cd.phase, PceCdPhase::Status);
    }

    #[test]
    fn raw_2352_disc_exposes_mode_one_user_payload() {
        let mut raw = vec![0; 2352];
        raw[0] = 0;
        raw[1..11].fill(0xff);
        raw[11] = 0;
        raw[15] = 1;
        raw[16..16 + 2048].fill(0x6d);
        let disc = DiscImage::new(ResourceBlob::from_bytes(&raw)).unwrap();
        assert!(disc.is_raw_sector(0));
        let mut sector = [0u8; 2048];
        disc.read_user_sector(0, &mut sector).unwrap();
        assert!(sector.iter().all(|byte| *byte == 0x6d));
    }

    #[test]
    fn cue_multitrack_directory_and_cdda_playback_are_live() {
        let data = vec![0x5a; 2 * 2048];
        let mut audio = vec![0u8; 2 * 2352];
        let left = 0x2000i16.to_le_bytes();
        let right = (-0x2000i16).to_le_bytes();
        for frame in audio.as_chunks_mut::<4>().0 {
            frame[..2].copy_from_slice(&left);
            frame[2..].copy_from_slice(&right);
        }
        let cue = "FILE \"data.bin\" BINARY\n  TRACK 01 MODE1/2048\n    INDEX 01 00:00:00\nFILE \"audio.bin\" BINARY\n  TRACK 02 AUDIO\n    INDEX 01 00:00:00\n";
        let mut cd = PceCdInterface::new(
            cue_container(cue, &[("data.bin", data), ("audio.bin", audio)]),
            0x6d9a_73ef,
        )
        .unwrap();
        assert_eq!(cd.disc.first_track(), 1);
        assert_eq!(cd.disc.last_track(), 2);
        assert_eq!(cd.disc.track_start(2), Some(2));
        assert_eq!(cd.disc.track_for_lba(2).unwrap().kind, TrackKind::Audio);

        cd_send_command(&mut cd, &[0xde, 0x02, 0x02, 0, 0, 0, 0, 0, 0, 0]);
        let directory = std::array::from_fn::<_, 4, _>(|_| cd.read_reg(0x08));
        assert_eq!(directory, [0x00, 0x02, 0x02, 0x00]);
        cd_finish_status(&mut cd);

        cd_send_command(&mut cd, &[0xd8, 0x01, 0x02, 0, 0, 0, 0, 0, 0, 0x80]);
        cd_finish_status(&mut cd);
        cd.begin_frame();
        cd.tick((CPU_CLOCK_HZ / 100) as u32);
        assert_eq!(cd.cdda_status, CDDA_PLAYING);
        assert!(cd
            .cdda_samples
            .iter()
            .any(|(left, right)| *left > 0.20 && *right < -0.20));
    }

    #[test]
    fn adpcm_cpu_port_honors_read_and_write_pipeline_delays() {
        let mut cd =
            PceCdInterface::new(ResourceBlob::from_bytes(&synthetic_disc(1)), 0x6d9a_73ef).unwrap();
        cd.adpcm_ram[0x1234] = 0x5a;
        cd.write_reg(0x08, 0x34);
        cd.write_reg(0x09, 0x12);
        cd.write_reg(0x0d, 0x08);
        assert_eq!(cd.read_reg(0x0a), 0);
        assert_eq!(cd.read_reg(0x0a), 0);
        assert_eq!(cd.read_reg(0x0a), 0x5a);
        assert_eq!(cd.adpcm_read_ptr, 0x1235);

        cd.write_reg(0x08, 0x00);
        cd.write_reg(0x09, 0x20);
        cd.write_reg(0x0d, 0x03);
        cd.write_reg(0x0a, 0xaa);
        cd.write_reg(0x0a, 0xbb);
        assert_eq!(cd.adpcm_ram[0x2000], 0xbb);
        assert_eq!(cd.adpcm_write_ptr, 0x2001);
    }

    #[test]
    fn adpcm_playback_rate_decode_and_full_irq_are_live() {
        let mut fast =
            PceCdInterface::new(ResourceBlob::from_bytes(&synthetic_disc(1)), 0x6d9a_73ef).unwrap();
        fast.adpcm_ram[..8].fill(0x77);
        fast.adpcm_length = 7;
        fast.write_reg(0x0e, 0x0f);
        fast.write_adpcm_control(0x40);
        fast.begin_frame();
        fast.tick(1_000);
        assert_eq!(fast.adpcm_clock_divider, 1);
        assert!(fast.adpcm_play_addr >= 2);
        assert!(fast.adpcm_signal > 0);
        assert!(fast
            .adpcm_samples
            .iter()
            .any(|(left, right)| { left.abs() > 0.0001 && (*left - *right).abs() < f32::EPSILON }));

        let mut slow =
            PceCdInterface::new(ResourceBlob::from_bytes(&synthetic_disc(1)), 0x6d9a_73ef).unwrap();
        slow.adpcm_ram[..8].fill(0x77);
        slow.adpcm_length = 7;
        slow.write_reg(0x0e, 0x00);
        slow.write_adpcm_control(0x40);
        slow.tick(1_000);
        assert_eq!(slow.adpcm_clock_divider, 16);
        assert_eq!(slow.adpcm_play_addr, 0);
        assert_eq!(slow.adpcm_signal, 0);

        let mut ending =
            PceCdInterface::new(ResourceBlob::from_bytes(&synthetic_disc(1)), 0x6d9a_73ef).unwrap();
        ending.adpcm_ram[0] = 0x77;
        ending.write_reg(0x0e, 0x0f);
        ending.write_adpcm_control(0x40);
        ending.tick(500);
        assert_ne!(ending.adpcm_status & ADPCM_STOP_FLAG, 0);
        assert_eq!(ending.adpcm_status & ADPCM_PLAY_FLAG, 0);
        assert_ne!(ending.irq_status & CD_IRQ_SAMPLE_FULL_PLAY, 0);
    }

    #[test]
    fn adpcm_and_cdda_faders_follow_programmed_durations() {
        let mut cd =
            PceCdInterface::new(ResourceBlob::from_bytes(&synthetic_disc(1)), 0x6d9a_73ef).unwrap();
        cd.write_reg(0x0f, 0x0d);
        let short = PceCdInterface::fade_cycles(1_500);
        cd.cdda_fade.tick((short / 2) as u32);
        assert!((cd.cdda_fade.gain as i64 - i64::from(MIX_GAIN_ONE / 2)).abs() <= 1);
        cd.cdda_fade.tick((short - short / 2) as u32);
        assert_eq!(cd.cdda_fade.gain, 0);

        cd.write_reg(0x0f, 0x00);
        assert_eq!(cd.cdda_fade.gain, 0);
        assert_eq!(cd.adpcm_fade.gain, 0);
        let fade_in = PceCdInterface::fade_cycles(100);
        cd.cdda_fade.tick(fade_in as u32);
        cd.adpcm_fade.tick(fade_in as u32);
        assert_eq!(cd.cdda_fade.gain, MIX_GAIN_ONE);
        assert_eq!(cd.adpcm_fade.gain, MIX_GAIN_ONE);
    }

    #[test]
    fn adpcm_state_round_trip_preserves_future_audio_and_irq_progress() {
        let mut bios = vec![0xea; SYSTEM_CARD_SIZE];
        bios[0x1ffe] = 0x00;
        bios[0x1fff] = 0xe0;
        let mut machine = PcEngineMachine::from_bios_and_disc(
            &bios,
            ResourceBlob::from_bytes(&synthetic_disc(2)),
        )
        .unwrap();
        {
            let cd = machine.bus.cd.as_mut().unwrap();
            cd.adpcm_ram[..8].fill(0x77);
            cd.adpcm_length = 7;
            cd.write_reg(0x0e, 0x0f);
            cd.write_reg(0x0f, 0x0e);
            cd.write_adpcm_control(0x40);
            cd.begin_frame();
            cd.tick(250);
        }
        let state = machine.save_state().unwrap();

        let expected = {
            let cd = machine.bus.cd.as_mut().unwrap();
            cd.begin_frame();
            cd.tick(1_000);
            (
                cd.adpcm_play_addr,
                cd.adpcm_low_nibble,
                cd.adpcm_signal,
                cd.adpcm_step,
                cd.adpcm_status,
                cd.irq_status,
                cd.adpcm_fade.gain,
                cd.adpcm_samples.clone(),
            )
        };
        machine.load_state(&state).unwrap();
        let actual = {
            let cd = machine.bus.cd.as_mut().unwrap();
            cd.begin_frame();
            cd.tick(1_000);
            (
                cd.adpcm_play_addr,
                cd.adpcm_low_nibble,
                cd.adpcm_signal,
                cd.adpcm_step,
                cd.adpcm_status,
                cd.irq_status,
                cd.adpcm_fade.gain,
                cd.adpcm_samples.clone(),
            )
        };
        assert_eq!(actual, expected);
    }

    #[test]
    fn cd_state_and_backup_persistence_round_trip() {
        let mut bios = vec![0xea; SYSTEM_CARD_SIZE];
        bios[0x1ffe] = 0x00;
        bios[0x1fff] = 0xe0;
        let mut machine = PcEngineMachine::from_bios_and_disc(
            &bios,
            ResourceBlob::from_bytes(&synthetic_disc(2)),
        )
        .unwrap();

        assert_eq!(
            machine.persistent_len(ResourceKind::Storage, 0),
            BACKUP_RAM_SIZE
        );
        let mut backup = [0u8; BACKUP_RAM_SIZE];
        backup[0] = 0x51;
        backup[BACKUP_RAM_SIZE - 1] = 0xa7;
        machine
            .write_persistent(ResourceKind::Storage, 0, &backup)
            .unwrap();
        machine.bus.cd.as_mut().unwrap().cd_ram[0x1234] = 0x5a;
        machine.bus.cd.as_mut().unwrap().adpcm_ram[0x4321] = 0xa5;
        machine.bus.cd.as_mut().unwrap().irq_mask = CD_IRQ_TRANSFER_DONE;
        machine.bus.cd.as_mut().unwrap().irq_status = CD_IRQ_TRANSFER_DONE;

        let state = machine.save_state().unwrap();
        machine.bus.cd.as_mut().unwrap().cd_ram[0x1234] = 0;
        machine.bus.cd.as_mut().unwrap().adpcm_ram[0x4321] = 0;
        machine.bus.cd.as_mut().unwrap().backup_ram.fill(0);
        machine.load_state(&state).unwrap();

        assert_eq!(machine.bus.cd.as_ref().unwrap().cd_ram[0x1234], 0x5a);
        assert_eq!(machine.bus.cd.as_ref().unwrap().adpcm_ram[0x4321], 0xa5);
        assert!(machine.bus.cd.as_ref().unwrap().irq_pending());
        let mut restored = [0u8; BACKUP_RAM_SIZE];
        machine
            .read_persistent(ResourceKind::Storage, 0, &mut restored)
            .unwrap();
        assert_eq!(restored, backup);
    }

    #[test]
    fn populous_hucard_ram_overlay_is_read_write_and_stateful() {
        let rom = vec![0xea; 512 * 1024];
        let mut machine = PcEngineMachine::from_rom(&rom).unwrap();
        machine.bus.populous_ram = Some(Box::new([0; POPULOUS_RAM_SIZE]));
        machine.bus.write8(0x080000, 0x5a);
        machine.bus.write8(0x087fff, 0xa5);
        assert_eq!(machine.bus.read8(0x080000), 0x5a);
        assert_eq!(machine.bus.read8(0x087fff), 0xa5);

        let state = machine.save_state().unwrap();
        machine.bus.write8(0x080000, 0);
        machine.bus.write8(0x087fff, 0);
        machine.load_state(&state).unwrap();
        assert_eq!(machine.bus.read8(0x080000), 0x5a);
        assert_eq!(machine.bus.read8(0x087fff), 0xa5);
    }

    #[test]
    fn street_fighter_two_hucard_switches_512k_window_and_saves_bank() {
        let rom = synthetic_sf2_hucard();
        let mut bus = PceBus::from_rom(&rom).unwrap();
        assert_eq!(bus.read8(0x000100), 0x5a);
        assert_eq!(bus.read8(0x080000), 0x40);
        assert_eq!(bus.read8(0x080123), 0x80);
        bus.write8(0x001ff3, 0xaa);
        assert_eq!(bus.sf2_bank, 3);
        assert_eq!(bus.read8(0x080000), 0x43);
        assert_eq!(bus.read8(0x080123), 0x83);
        assert_eq!(bus.read8(0x000100), 0x5a);

        let mut machine = PcEngineMachine::from_rom(&rom).unwrap();
        machine.bus.write8(0x001ff2, 0);
        let state = machine.save_state().unwrap();
        machine.bus.write8(0x001ff0, 0);
        assert_eq!(machine.bus.read8(0x080000), 0x40);
        machine.load_state(&state).unwrap();
        assert_eq!(machine.bus.sf2_bank, 2);
        assert_eq!(machine.bus.read8(0x080000), 0x42);
    }

    #[test]
    fn synthetic_hucard_runs_cpu_vdc_vce_and_psg() {
        let rom = synthetic_hucard();
        let mut machine = PcEngineMachine::from_rom(&rom).unwrap();
        machine.run_frame(&InputState::default());
        assert_eq!(machine.video().width(), WIDTH);
        assert_eq!(machine.video().height(), HEIGHT);
        assert!(machine
            .video()
            .pixels()
            .as_chunks::<4>()
            .0
            .iter()
            .any(|pixel| pixel[0] > pixel[1]));
        assert_eq!(machine.audio().sample_rate(), AUDIO_RATE);
        assert!(machine
            .audio()
            .samples()
            .iter()
            .any(|sample| sample.abs() > 0.001));
    }

    #[test]
    fn supergrafx_vpc_routes_st_ports_and_memory_mapped_vdcs() {
        let rom = synthetic_hucard();
        let mut bus = PceBus::from_supergrafx_rom(&rom).unwrap();

        bus.write8(0x1fe000, 5);
        assert_eq!(bus.vdc.select, 5);
        bus.write8(0x1fe00e, 1);
        assert!(bus.vpc.as_ref().unwrap().io_device);
        bus.write_st(0, 7);
        assert_eq!(bus.vdc.select, 5);
        assert_eq!(bus.vdc2.as_ref().unwrap().select, 7);

        bus.write8(0x1fe010, 9);
        assert_eq!(bus.vdc2.as_ref().unwrap().select, 9);
        bus.write8(0x1fe008, 0x33);
        assert_eq!(bus.read8(0x1fe008), 0x33);
    }

    #[test]
    fn supergrafx_vpc_windows_and_priority_compose_dual_vdc_output() {
        let rom = synthetic_hucard();
        let mut bus = PceBus::from_supergrafx_rom(&rom).unwrap();
        bus.vce.colors[1] = 0x0038;
        bus.vce.colors[0x101] = 0x01c0;
        bus.vdc.pixel_codes[10] = 1;
        bus.vdc.pixel_codes[200] = 1;
        let vdc2 = bus.vdc2.as_mut().unwrap();
        vdc2.pixel_codes[10] = 0x0101;
        vdc2.pixel_codes[200] = 0x0101;
        let vpc = bus.vpc.as_mut().unwrap();
        vpc.windows = [100, 100];
        vpc.priorities[0] = VpcPriority {
            prio_type: 0,
            dev0_enabled: true,
            dev1_enabled: true,
        };
        vpc.priorities[3] = VpcPriority {
            prio_type: 1,
            dev0_enabled: true,
            dev1_enabled: true,
        };

        bus.compose_video();
        assert_eq!(&bus.video.pixels()[40..44], &pce_color(0x0038));
        assert_eq!(&bus.video.pixels()[800..804], &pce_color(0x01c0));
    }

    #[test]
    fn supergrafx_state_round_trip_preserves_vpc_and_second_vdc() {
        let rom = synthetic_hucard();
        let mut machine = PcEngineMachine::from_supergrafx_rom(&rom).unwrap();
        machine.bus.vdc2.as_mut().unwrap().vram[0x123] = 0xabcd;
        machine.bus.vpc.as_mut().unwrap().write(0, 0x33);
        machine.bus.vpc.as_mut().unwrap().windows = [77, 155];
        let state = machine.save_state().unwrap();

        machine.bus.vdc2.as_mut().unwrap().vram[0x123] = 0;
        machine.bus.vpc.as_mut().unwrap().write(0, 0x11);
        machine.bus.vpc.as_mut().unwrap().windows = [0, 0];
        machine.load_state(&state).unwrap();
        assert_eq!(machine.platform(), PlatformId::SuperGrafx);
        assert_eq!(machine.bus.vdc2.as_ref().unwrap().vram[0x123], 0xabcd);
        assert_eq!(machine.bus.vpc.as_ref().unwrap().read(0), 0x33);
        assert_eq!(machine.bus.vpc.as_ref().unwrap().windows, [77, 155]);

        let mut pc_engine = PcEngineMachine::from_rom(&rom).unwrap();
        assert!(pc_engine.load_state(&state).is_err());
    }

    #[test]
    fn street_fighter_two_six_button_pad_alternates_on_clr_pulses() {
        let rom = synthetic_sf2_hucard();
        let mut bus = PceBus::from_rom(&rom).unwrap();
        let mut input = InputState::default();
        input.buttons[0] = UP | LEFT | FACE_SOUTH | SELECT | FACE_WEST | L1;
        bus.set_input(&input);

        bus.write_joy_output(1);
        bus.write_joy_output(3);
        bus.write_joy_output(1);
        assert!(bus.joy_six_button_phase);
        assert_eq!(bus.joy_read() & 0x0f, 0x00);
        bus.write_joy_output(0);
        assert_eq!(bus.joy_read() & 0x0f, 0x0a);

        bus.write_joy_output(1);
        bus.write_joy_output(3);
        bus.write_joy_output(1);
        assert!(!bus.joy_six_button_phase);
        assert_eq!(bus.joy_read() & 0x0f, 0x06);
        bus.write_joy_output(0);
        assert_eq!(bus.joy_read() & 0x0f, 0x0a);

        let mut machine = PcEngineMachine::from_rom(&rom).unwrap();
        machine.bus.write_joy_output(1);
        machine.bus.write_joy_output(3);
        machine.bus.write_joy_output(1);
        let state = machine.save_state().unwrap();
        machine.bus.write_joy_output(3);
        assert!(!machine.bus.joy_six_button_phase);
        machine.load_state(&state).unwrap();
        assert!(machine.bus.joy_six_button_phase);
        assert_eq!(machine.bus.joy_output, 1);
    }

    #[test]
    fn joypad_port_mux_is_active_low() {
        let rom = synthetic_hucard();
        let mut bus = PceBus::from_rom(&rom).unwrap();
        let mut input = InputState::default();
        input.buttons[0] = UP | FACE_SOUTH;
        bus.set_input(&input);
        bus.joy_output = 1;
        assert_eq!(bus.joy_read() & 0x0f, 0x0e);
        bus.joy_output = 0;
        assert_eq!(bus.joy_read() & 0x0f, 0x0e);
        bus.joy_output = 3;
        assert_eq!(bus.joy_read() & 0x0f, 0);
    }

    #[test]
    fn vram_dma_and_satb_dma_complete_and_raise_status() {
        let mut vdc = HuC6270::default();
        vdc.vram[0x100] = 0x1234;
        vdc.vram[0x101] = 0xabcd;
        vdc.regs[16] = 0x100;
        vdc.regs[17] = 0x200;
        vdc.regs[18] = 1;
        vdc.vram_dma();
        assert_eq!(vdc.vram[0x200], 0x1234);
        assert_eq!(vdc.vram[0x201], 0xabcd);
        assert_ne!(vdc.status & 0x10, 0);
        vdc.regs[19] = 0x200;
        vdc.satb_dma();
        assert_eq!(vdc.satb[0], 0x1234);
        assert_ne!(vdc.status & 0x08, 0);
    }

    #[test]
    fn state_round_trip_preserves_cpu_video_audio_and_ram() {
        let rom = synthetic_hucard();
        let mut machine = PcEngineMachine::from_rom(&rom).unwrap();
        machine.run_frame(&InputState::default());
        machine.bus.ram[7] = 0x5a;
        let saved = machine.save_state().unwrap();
        machine.bus.ram[7] = 0;
        machine.run_frame(&InputState::default());
        machine.load_state(&saved).unwrap();
        assert_eq!(machine.bus.ram[7], 0x5a);
        assert_eq!(machine.save_state().unwrap(), saved);
    }

    #[test]
    fn optional_512_byte_copier_header_is_removed() {
        let rom = synthetic_hucard();
        let mut headered = vec![0u8; 512];
        headered.extend_from_slice(&rom);
        let bus = PceBus::from_rom(&headered).unwrap();
        assert_eq!(bus.rom, rom);
    }
}
