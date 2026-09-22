use super::sn76489::Sn76489;
use super::ym2413::Ym2413;
use crate::cpu_z80::{Z80Bus, Z80};
use crate::input::{
    AXIS_AUX_X, AXIS_AUX_Y, DOWN, FACE_EAST, FACE_SOUTH, LEFT, POINTER_CLICK, POINTER_TOUCH, RIGHT,
    SELECT, START, UP,
};
use crate::kernel::{AudioBuffer, InputState, VideoBuffer};
use crate::machine::Machine;
use crate::platform::PlatformId;
use crate::resources::ResourceKind;
use crate::state::{StateReader, StateWriter};

const WIDTH: u32 = 256;
const HEIGHT: u32 = 192;
const EXTENDED_HEIGHT: u32 = 224;
const FULL_HEIGHT: u32 = 240;
const CPU_CLOCK: f64 = 3_579_545.0;
const FRAME_RATE: f64 = 59.922_743;
const STATE_VERSION: u32 = 7;
const LEGACY_PALETTE: [u8; 16] = [
    0x00, 0x00, 0x08, 0x0c, 0x10, 0x30, 0x01, 0x3c, 0x02, 0x03, 0x05, 0x0f, 0x04, 0x33, 0x15, 0x3f,
];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum LegacyMode {
    GraphicsOne,
    Text,
    GraphicsTwo,
    Multicolor,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum MapperKind {
    Sega,
    Codemasters,
    Korean,
    Janggun,
    Msx,
    Nemesis,
}

struct SegaCartridge {
    rom: Vec<u8>,
    sram: Vec<u8>,
    mapper: MapperKind,
    control: u8,
    pages: [u8; 3],
    janggun_banks: [u8; 4],
    janggun_reverse: [bool; 4],
    msx_banks: [u8; 4],
}

impl SegaCartridge {
    fn new(rom: &[u8]) -> Result<Self, String> {
        if rom.len() < 1024 {
            return Err("Master System cartridge image is too small".into());
        }
        let image = if rom.len() > 512 && rom.len() % 0x4000 == 512 {
            &rom[512..]
        } else {
            rom
        };
        let mapper = if Self::has_codemasters_header(image) {
            MapperKind::Codemasters
        } else if Self::crc32_ieee(image) == 0x1929_49d5 {
            MapperKind::Janggun
        } else if image.len() >= 0x2000
            && Self::crc16_ccitt(&image[image.len() - 0x2000..]) == 0xee05
        {
            MapperKind::Nemesis
        } else if Self::has_msx_mapper_signature(image) {
            MapperKind::Msx
        } else if Self::has_korean_mapper_signature(image) {
            MapperKind::Korean
        } else {
            MapperKind::Sega
        };
        let (pages, sram_len) = match mapper {
            MapperKind::Sega => ([0, 1, 2], 0x8000),
            MapperKind::Codemasters => ([0, 1, 0], 0x2000),
            MapperKind::Korean | MapperKind::Janggun | MapperKind::Msx | MapperKind::Nemesis => {
                ([0, 1, 2], 0)
            }
        };
        Ok(Self {
            rom: image.to_vec(),
            sram: vec![0; sram_len],
            mapper,
            control: 0,
            pages,
            janggun_banks: [2, 3, 4, 5],
            janggun_reverse: [false; 4],
            msx_banks: [2, 3, 4, 5],
        })
    }

    fn has_codemasters_header(image: &[u8]) -> bool {
        if image.len() < 0x8000 {
            return false;
        }
        let header = &image[0x7fe0..0x7ff0];
        let bank_count = usize::from(header[0]);
        if bank_count == 0 || bank_count > image.len().div_ceil(0x4000) {
            return false;
        }
        if header[10..].iter().any(|value| *value != 0) {
            return false;
        }
        let checksum = u16::from_le_bytes([header[6], header[7]]);
        let complement = u16::from_le_bytes([header[8], header[9]]);
        if checksum.wrapping_add(complement) != 0 {
            return false;
        }
        let bcd = |value: u8| -> Option<u8> {
            let high = value >> 4;
            let low = value & 0x0f;
            (high <= 9 && low <= 9).then_some(high * 10 + low)
        };
        matches!(bcd(header[1]), Some(1..=31))
            && matches!(bcd(header[2]), Some(1..=12))
            && bcd(header[3]).is_some()
            && matches!(bcd(header[4]), Some(0..=23))
            && matches!(bcd(header[5]), Some(0..=59))
    }

    fn crc32_ieee(image: &[u8]) -> u32 {
        let mut crc = 0xffff_ffffu32;
        for &byte in image {
            crc ^= u32::from(byte);
            for _ in 0..8 {
                let mask = 0u32.wrapping_sub(crc & 1);
                crc = (crc >> 1) ^ (0xedb8_8320 & mask);
            }
        }
        !crc
    }

    fn crc16_ccitt(image: &[u8]) -> u16 {
        let mut crc = 0xffffu16;
        for &byte in image {
            crc ^= u16::from(byte) << 8;
            for _ in 0..8 {
                crc = if crc & 0x8000 != 0 {
                    (crc << 1) ^ 0x1021
                } else {
                    crc << 1
                };
            }
        }
        crc
    }

    fn has_sega_header(image: &[u8]) -> bool {
        [0x1ff0usize, 0x3ff0, 0x7ff0].into_iter().any(|offset| {
            image
                .get(offset..offset + 8)
                .is_some_and(|header| header == b"TMR SEGA")
        })
    }

    fn has_msx_mapper_signature(image: &[u8]) -> bool {
        if image.len() <= 0x10000 || Self::has_sega_header(image) {
            return false;
        }
        let mut registers = 0u8;
        for window in image.windows(3) {
            if window[0] == 0x32 && window[2] == 0 && window[1] <= 3 {
                registers |= 1 << window[1];
            }
        }
        registers.count_ones() >= 2
    }

    fn has_korean_mapper_signature(image: &[u8]) -> bool {
        image.len() > 0xc000
            && !Self::has_sega_header(image)
            && image.windows(3).any(|window| window == [0x32, 0x00, 0xa0])
    }

    fn reset_mapping(&mut self) {
        self.control = 0;
        self.pages = match self.mapper {
            MapperKind::Sega
            | MapperKind::Korean
            | MapperKind::Janggun
            | MapperKind::Msx
            | MapperKind::Nemesis => [0, 1, 2],
            MapperKind::Codemasters => [0, 1, 0],
        };
        self.janggun_banks = [2, 3, 4, 5];
        self.janggun_reverse = [false; 4];
        self.msx_banks = [2, 3, 4, 5];
    }

    fn bank_count(&self) -> usize {
        self.rom.len().div_ceil(0x4000).max(1)
    }

    fn rom_byte(&self, bank: u8, offset: usize) -> u8 {
        let bank = usize::from(bank) % self.bank_count();
        self.rom[(bank * 0x4000 + offset) % self.rom.len()]
    }

    fn rom_byte_8k(&self, bank: u8, offset: usize) -> u8 {
        let bank_count = self.rom.len().div_ceil(0x2000).max(1);
        let bank = usize::from(bank) % bank_count;
        self.rom[(bank * 0x2000 + offset) % self.rom.len()]
    }

    fn read(&self, address: u16) -> u8 {
        match self.mapper {
            MapperKind::Sega => match address {
                0x0000..=0x03ff => self.rom[address as usize % self.rom.len()],
                0x0400..=0x3fff => self.rom_byte(self.pages[0], address as usize & 0x3fff),
                0x4000..=0x7fff => self.rom_byte(self.pages[1], address as usize & 0x3fff),
                0x8000..=0xbfff if self.control & 0x08 != 0 => {
                    let bank = usize::from((self.control >> 2) & 1);
                    self.sram[bank * 0x4000 + (address as usize & 0x3fff)]
                }
                0x8000..=0xbfff => self.rom_byte(self.pages[2], address as usize & 0x3fff),
                _ => 0xff,
            },
            MapperKind::Codemasters => match address {
                0x0000..=0x3fff => self.rom_byte(self.pages[0], address as usize),
                0x4000..=0x7fff => self.rom_byte(self.pages[1] & 0x7f, address as usize & 0x3fff),
                0xa000..=0xbfff if self.pages[1] & 0x80 != 0 => {
                    self.sram[address as usize & 0x1fff]
                }
                0x8000..=0xbfff => self.rom_byte(self.pages[2], address as usize & 0x3fff),
                _ => 0xff,
            },
            MapperKind::Korean => match address {
                0x0000..=0x3fff => self.rom_byte(0, address as usize),
                0x4000..=0x7fff => self.rom_byte(1, address as usize & 0x3fff),
                0x8000..=0xbfff => self.rom_byte(self.pages[2], address as usize & 0x3fff),
                _ => 0xff,
            },
            MapperKind::Msx | MapperKind::Nemesis => match address {
                0x0000..=0x1fff if self.mapper == MapperKind::Nemesis => {
                    self.rom[self.rom.len() - 0x2000 + usize::from(address)]
                }
                0x0000..=0x3fff if self.mapper == MapperKind::Msx => {
                    self.rom[usize::from(address) % self.rom.len()]
                }
                0x2000..=0x3fff => self.rom_byte_8k(1, usize::from(address) & 0x1fff),
                0x4000..=0x5fff => {
                    self.rom_byte_8k(self.msx_banks[0], usize::from(address) & 0x1fff)
                }
                0x6000..=0x7fff => {
                    self.rom_byte_8k(self.msx_banks[1], usize::from(address) & 0x1fff)
                }
                0x8000..=0x9fff => {
                    self.rom_byte_8k(self.msx_banks[2], usize::from(address) & 0x1fff)
                }
                0xa000..=0xbfff => {
                    self.rom_byte_8k(self.msx_banks[3], usize::from(address) & 0x1fff)
                }
                _ => 0xff,
            },
            MapperKind::Janggun => {
                if address < 0x4000 {
                    return self.rom[address as usize % self.rom.len()];
                }
                let slot = usize::from((address - 0x4000) / 0x2000);
                if slot >= self.janggun_banks.len() {
                    return 0xff;
                }
                let value =
                    self.rom_byte_8k(self.janggun_banks[slot], usize::from(address) & 0x1fff);
                if self.janggun_reverse[slot] {
                    value.reverse_bits()
                } else {
                    value
                }
            }
        }
    }

    fn write(&mut self, address: u16, value: u8) {
        match self.mapper {
            MapperKind::Sega => {
                if (0x8000..=0xbfff).contains(&address) && self.control & 0x08 != 0 {
                    let bank = usize::from((self.control >> 2) & 1);
                    let index = bank * 0x4000 + (address as usize & 0x3fff);
                    self.sram[index] = value;
                }
            }
            MapperKind::Codemasters => match address {
                0x0000..=0x3fff => self.pages[0] = value,
                0x4000..=0x7fff => self.pages[1] = value,
                0xa000..=0xbfff if self.pages[1] & 0x80 != 0 => {
                    self.sram[address as usize & 0x1fff] = value;
                }
                0x8000..=0xbfff => self.pages[2] = value,
                _ => {}
            },
            MapperKind::Korean if address == 0xa000 => self.pages[2] = value,
            MapperKind::Korean => {}
            MapperKind::Msx | MapperKind::Nemesis => match address {
                0x0000 => self.msx_banks[2] = value,
                0x0001 => self.msx_banks[3] = value,
                0x0002 => self.msx_banks[0] = value,
                0x0003 => self.msx_banks[1] = value,
                _ => {}
            },
            MapperKind::Janggun => match address {
                0x4000 => {
                    self.janggun_banks[0] = value & 0x3f;
                    self.janggun_reverse[0] = value & 0x80 != 0;
                }
                0x6000 => {
                    self.janggun_banks[1] = value & 0x3f;
                    self.janggun_reverse[1] = value & 0x80 != 0;
                }
                0x8000 => {
                    self.janggun_banks[2] = value & 0x3f;
                    self.janggun_reverse[2] = value & 0x80 != 0;
                }
                0xa000 => {
                    self.janggun_banks[3] = value & 0x3f;
                    self.janggun_reverse[3] = value & 0x80 != 0;
                }
                _ => {}
            },
        }
    }

    fn write_mapper(&mut self, address: u16, value: u8) {
        match self.mapper {
            MapperKind::Sega => match address {
                0xfffc => self.control = value,
                0xfffd => self.pages[0] = value,
                0xfffe => self.pages[1] = value,
                0xffff => self.pages[2] = value,
                _ => {}
            },
            MapperKind::Janggun if matches!(address, 0xfffe | 0xffff) => {
                let base = (value & 0x1f) << 1;
                let slot = if address == 0xfffe { 0 } else { 2 };
                self.janggun_banks[slot] = base;
                self.janggun_banks[slot + 1] = base | 1;
                let reverse = value & 0xc0 != 0;
                self.janggun_reverse[slot] = reverse;
                self.janggun_reverse[slot + 1] = reverse;
            }
            MapperKind::Codemasters
            | MapperKind::Korean
            | MapperKind::Janggun
            | MapperKind::Msx
            | MapperKind::Nemesis => {}
        }
    }

    fn save(&self, out: &mut StateWriter) {
        out.u8(self.control);
        for page in self.pages {
            out.u8(page);
        }
        for bank in self.janggun_banks {
            out.u8(bank);
        }
        for reverse in self.janggun_reverse {
            out.u8(reverse as u8);
        }
        for bank in self.msx_banks {
            out.u8(bank);
        }
        out.blob(&self.sram);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.control = input.u8()?;
        for page in &mut self.pages {
            *page = input.u8()?;
        }
        for bank in &mut self.janggun_banks {
            *bank = input.u8()? & 0x3f;
        }
        for reverse in &mut self.janggun_reverse {
            *reverse = input.u8()? != 0;
        }
        for bank in &mut self.msx_banks {
            *bank = input.u8()?;
        }
        let sram = input.blob()?;
        if sram.len() != self.sram.len() {
            return Err("Master System save state has invalid cartridge RAM length".into());
        }
        self.sram.copy_from_slice(sram);
        Ok(())
    }
}

struct SmsVdp {
    vram: [u8; 0x4000],
    cram: [u8; 32],
    regs: [u8; 16],
    address: u16,
    code: u8,
    control_latch: Option<u8>,
    read_buffer: u8,
    status: u8,
    line_counter: u8,
    line_irq_pending: bool,
    dot: u16,
    dot_half: u8,
    h_counter_latch: u8,
    scanline: u16,
    frame: u64,
    irq_pending: bool,
    video: VideoBuffer,
    bg_priority: Vec<bool>,
}

impl Default for SmsVdp {
    fn default() -> Self {
        Self {
            vram: [0; 0x4000],
            cram: [0; 32],
            regs: [0; 16],
            address: 0,
            code: 0,
            control_latch: None,
            read_buffer: 0,
            status: 0,
            line_counter: 0xff,
            line_irq_pending: false,
            dot: 0,
            dot_half: 0,
            h_counter_latch: 0,
            scanline: 0,
            frame: 0,
            irq_pending: false,
            video: VideoBuffer::new(WIDTH, HEIGHT),
            bg_priority: vec![false; (WIDTH * FULL_HEIGHT) as usize],
        }
    }
}

impl SmsVdp {
    fn reset(&mut self) {
        let vram = self.vram;
        let cram = self.cram;
        *self = Self::default();
        self.vram = vram;
        self.cram = cram;
    }

    fn mode4_enabled(&self) -> bool {
        self.regs[0] & 0x04 != 0
    }

    fn legacy_mode(&self) -> LegacyMode {
        if self.regs[1] & 0x10 != 0 {
            LegacyMode::Text
        } else if self.regs[0] & 0x02 != 0 {
            LegacyMode::GraphicsTwo
        } else if self.regs[1] & 0x08 != 0 {
            LegacyMode::Multicolor
        } else {
            LegacyMode::GraphicsOne
        }
    }

    fn extended_height_mode(&self) -> bool {
        self.mode4_enabled()
            && self.regs[0] & 0x02 != 0
            && matches!(self.regs[1] & 0x18, 0x08 | 0x10)
    }

    fn active_height(&self) -> u32 {
        if !self.mode4_enabled() || self.regs[0] & 0x02 == 0 {
            return HEIGHT;
        }
        match self.regs[1] & 0x18 {
            0x10 => EXTENDED_HEIGHT,
            0x08 => FULL_HEIGHT,
            _ => HEIGHT,
        }
    }

    fn name_table_base(&self) -> usize {
        if self.extended_height_mode() {
            ((usize::from(self.regs[2] & 0x0c) << 10) + 0x0700) & 0x3fff
        } else {
            (usize::from(self.regs[2] & 0x0e) << 10) & 0x3fff
        }
    }

    fn write_control(&mut self, value: u8) {
        let Some(first) = self.control_latch.take() else {
            self.control_latch = Some(value);
            return;
        };
        self.code = value >> 6;
        if self.code == 2 {
            let register = (value & 0x0f) as usize;
            self.regs[register] = first;
            self.update_irq();
            return;
        }
        self.address = ((u16::from(value & 0x3f) << 8) | u16::from(first)) & 0x3fff;
        if self.code == 0 {
            self.read_buffer = self.vram[self.address as usize];
            self.address = (self.address + 1) & 0x3fff;
        }
    }

    fn write_data(&mut self, value: u8) {
        match self.code {
            3 => self.cram[(self.address as usize) & 0x1f] = value & 0x3f,
            _ => self.vram[self.address as usize] = value,
        }
        self.address = (self.address + 1) & 0x3fff;
        self.read_buffer = value;
        self.control_latch = None;
    }

    fn read_data(&mut self) -> u8 {
        let value = self.read_buffer;
        self.read_buffer = self.vram[self.address as usize];
        self.address = (self.address + 1) & 0x3fff;
        self.control_latch = None;
        value
    }

    fn read_status(&mut self) -> u8 {
        let value = self.status;
        self.status = 0;
        self.line_irq_pending = false;
        self.irq_pending = false;
        self.control_latch = None;
        value
    }

    fn update_irq(&mut self) {
        let frame_irq = self.status & 0x80 != 0 && self.regs[1] & 0x20 != 0;
        let line_irq = self.line_irq_pending && self.regs[0] & 0x10 != 0;
        self.irq_pending = frame_irq || line_irq;
    }

    fn clock_line_counter(&mut self) {
        if u32::from(self.scanline) < self.active_height() {
            if self.line_counter == 0 {
                self.line_counter = self.regs[10];
                self.line_irq_pending = true;
            } else {
                self.line_counter = self.line_counter.wrapping_sub(1);
            }
        } else {
            self.line_counter = self.regs[10];
        }
        self.update_irq();
    }

    fn tick_cpu_cycles(&mut self, cycles: u32) {
        let scaled = cycles.saturating_mul(3) + u32::from(self.dot_half);
        self.dot_half = (scaled & 1) as u8;
        let mut dots = u32::from(self.dot) + scaled / 2;
        while dots >= 342 {
            dots -= 342;
            self.latch_sprite_status_for_scanline(usize::from(self.scanline));
            self.scanline = self.scanline.wrapping_add(1);
            self.clock_line_counter();
            if self.scanline == self.active_height() as u16 {
                self.status |= 0x80;
                self.frame = self.frame.wrapping_add(1);
                self.render_frame();
                self.update_irq();
            }
            if self.scanline >= 262 {
                self.scanline = 0;
                self.line_counter = self.regs[10];
                self.update_irq();
            }
        }
        self.dot = dots as u16;
    }

    fn v_counter(&self) -> u8 {
        if self.active_height() == FULL_HEIGHT {
            return self.scanline as u8;
        }
        let wrap_start = if self.active_height() == EXTENDED_HEIGHT {
            0xeb
        } else {
            0xdb
        };
        if self.scanline < wrap_start {
            self.scanline as u8
        } else {
            self.scanline.wrapping_sub(6) as u8
        }
    }

    fn current_h_counter(&self) -> u8 {
        let half_dot = self.dot / 2;
        if half_dot <= 0x93 {
            half_dot as u8
        } else {
            (half_dot + 0x55) as u8
        }
    }

    fn latch_h_counter(&mut self) {
        self.h_counter_latch = self.current_h_counter();
    }

    fn h_counter(&self) -> u8 {
        self.h_counter_latch
    }

    fn render_frame(&mut self) {
        self.video.resize(WIDTH, self.active_height());
        self.bg_priority.fill(false);
        if self.mode4_enabled() {
            let backdrop = sms_color(self.cram[16 + usize::from(self.regs[7] & 0x0f)]);
            self.video.clear(backdrop);
            if self.regs[1] & 0x40 == 0 {
                return;
            }
            self.render_background();
            self.render_sprites();
            self.mask_left_column();
        } else {
            let backdrop = self.legacy_color(self.regs[7] & 0x0f);
            self.video.clear(backdrop);
            if self.regs[1] & 0x40 == 0 {
                return;
            }
            self.render_legacy_background();
            if self.legacy_mode() != LegacyMode::Text {
                self.render_legacy_sprites();
            }
        }
    }

    fn render_background(&mut self) {
        let name_base = self.name_table_base();
        let base_scroll_x = usize::from(self.regs[8]);
        let base_scroll_y = usize::from(self.regs[9]);
        let tilemap_height = if self.extended_height_mode() {
            256
        } else {
            224
        };
        for y in 0..self.active_height() as usize {
            let scroll_x = if self.regs[0] & 0x40 != 0 && y < 16 {
                0
            } else {
                base_scroll_x
            };
            for x in 0..WIDTH as usize {
                let scroll_y = if self.regs[0] & 0x80 != 0 && x >= 192 {
                    0
                } else {
                    base_scroll_y
                };
                let world_y = (y + scroll_y) % tilemap_height;
                let tile_y = world_y / 8;
                let row = world_y & 7;
                let world_x = (x + 256 - scroll_x) & 0xff;
                let tile_x = world_x / 8;
                let entry = (name_base + (tile_y * 32 + tile_x) * 2) & 0x3fff;
                let low = self.vram[entry];
                let high = self.vram[(entry + 1) & 0x3fff];
                let tile = u16::from(low) | (u16::from(high & 1) << 8);
                let flip_x = high & 0x02 != 0;
                let flip_y = high & 0x04 != 0;
                let palette = usize::from(high & 0x08 != 0);
                let source_y = if flip_y { 7 - row } else { row };
                let source_x = if flip_x {
                    world_x & 7
                } else {
                    7 - (world_x & 7)
                };
                let pattern = (usize::from(tile) * 32 + source_y * 4) & 0x3fff;
                let mut color = 0u8;
                for plane in 0..4 {
                    color |= ((self.vram[(pattern + plane) & 0x3fff] >> source_x) & 1) << plane;
                }
                let cram_index = palette * 16 + usize::from(color);
                let rgba = sms_color(self.cram[cram_index]);
                let pixel = y * WIDTH as usize + x;
                self.bg_priority[pixel] = high & 0x10 != 0 && color != 0;
                let offset = pixel * 4;
                self.video.pixels_mut()[offset..offset + 4].copy_from_slice(&rgba);
            }
        }
    }

    fn legacy_color(&self, index: u8) -> [u8; 4] {
        let index = if index & 0x0f == 0 {
            self.regs[7] & 0x0f
        } else {
            index & 0x0f
        };
        sms_color(LEGACY_PALETTE[usize::from(index)])
    }

    fn draw_legacy_pattern_row(
        &mut self,
        x: usize,
        y: usize,
        pattern: u8,
        foreground: u8,
        background: u8,
    ) {
        for bit in 0..8usize {
            let color = if pattern & (0x80 >> bit) != 0 {
                foreground
            } else {
                background
            };
            let offset = (y * WIDTH as usize + x + bit) * 4;
            let rgba = self.legacy_color(color);
            self.video.pixels_mut()[offset..offset + 4].copy_from_slice(&rgba);
        }
    }

    fn render_legacy_background(&mut self) {
        match self.legacy_mode() {
            LegacyMode::GraphicsOne => self.render_legacy_graphics_one(),
            LegacyMode::Text => self.render_legacy_text(),
            LegacyMode::GraphicsTwo => self.render_legacy_graphics_two(),
            LegacyMode::Multicolor => self.render_legacy_multicolor(),
        }
    }

    fn render_legacy_graphics_one(&mut self) {
        let name_base = (usize::from(self.regs[2] & 0x0f) << 10) & 0x3fff;
        let pattern_base = (usize::from(self.regs[4] & 0x07) << 11) & 0x3fff;
        let color_base = (usize::from(self.regs[3]) << 6) & 0x3fff;
        for tile_y in 0..24usize {
            for tile_x in 0..32usize {
                let name = self.vram[(name_base + tile_y * 32 + tile_x) & 0x3fff];
                let colors = self.vram[(color_base + usize::from(name / 8)) & 0x3fff];
                for row in 0..8usize {
                    let pattern = self.vram[(pattern_base + usize::from(name) * 8 + row) & 0x3fff];
                    self.draw_legacy_pattern_row(
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

    fn render_legacy_text(&mut self) {
        let name_base = (usize::from(self.regs[2] & 0x0f) << 10) & 0x3fff;
        let pattern_base = (usize::from(self.regs[4] & 0x07) << 11) & 0x3fff;
        let foreground = self.regs[7] >> 4;
        let background = self.regs[7] & 0x0f;
        for tile_y in 0..24usize {
            for tile_x in 0..40usize {
                let name = self.vram[(name_base + tile_y * 40 + tile_x) & 0x3fff];
                for row in 0..8usize {
                    let pattern = self.vram[(pattern_base + usize::from(name) * 8 + row) & 0x3fff];
                    for column in 0..6usize {
                        let color = if pattern & (0x80 >> column) != 0 {
                            foreground
                        } else {
                            background
                        };
                        let x = 8 + tile_x * 6 + column;
                        let offset = ((tile_y * 8 + row) * WIDTH as usize + x) * 4;
                        let rgba = self.legacy_color(color);
                        self.video.pixels_mut()[offset..offset + 4].copy_from_slice(&rgba);
                    }
                }
            }
        }
    }

    fn render_legacy_graphics_two(&mut self) {
        let name_base = (usize::from(self.regs[2] & 0x0f) << 10) & 0x3fff;
        let pattern_base = if self.regs[4] & 0x04 != 0 { 0x2000 } else { 0 };
        let pattern_mask = (usize::from(self.regs[4] & 0x03) << 11) | 0x07ff;
        let color_base = if self.regs[3] & 0x80 != 0 { 0x2000 } else { 0 };
        let color_mask = (usize::from(self.regs[3] & 0x7f) << 6) | 0x003f;
        for tile_y in 0..24usize {
            let third = tile_y / 8;
            for tile_x in 0..32usize {
                let name = self.vram[(name_base + tile_y * 32 + tile_x) & 0x3fff];
                let pattern_number = third * 0x100 + usize::from(name);
                for row in 0..8usize {
                    let index = pattern_number * 8 + row;
                    let pattern = self.vram[(pattern_base | (index & pattern_mask)) & 0x3fff];
                    let colors = self.vram[(color_base | (index & color_mask)) & 0x3fff];
                    self.draw_legacy_pattern_row(
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

    fn render_legacy_multicolor(&mut self) {
        let name_base = (usize::from(self.regs[2] & 0x0f) << 10) & 0x3fff;
        let pattern_base = (usize::from(self.regs[4] & 0x07) << 11) & 0x3fff;
        for tile_y in 0..24usize {
            let byte_offset = (tile_y & 3) * 2;
            for tile_x in 0..32usize {
                let name = self.vram[(name_base + tile_y * 32 + tile_x) & 0x3fff];
                let upper =
                    self.vram[(pattern_base + usize::from(name) * 8 + byte_offset) & 0x3fff];
                let lower =
                    self.vram[(pattern_base + usize::from(name) * 8 + byte_offset + 1) & 0x3fff];
                for y_in_tile in 0..8usize {
                    let colors = if y_in_tile < 4 { upper } else { lower };
                    for x_in_tile in 0..8usize {
                        let color = if x_in_tile < 4 {
                            colors >> 4
                        } else {
                            colors & 0x0f
                        };
                        let x = tile_x * 8 + x_in_tile;
                        let y = tile_y * 8 + y_in_tile;
                        let offset = (y * WIDTH as usize + x) * 4;
                        let rgba = self.legacy_color(color);
                        self.video.pixels_mut()[offset..offset + 4].copy_from_slice(&rgba);
                    }
                }
            }
        }
    }

    fn mask_left_column(&mut self) {
        if self.regs[0] & 0x20 == 0 {
            return;
        }
        let backdrop = sms_color(self.cram[16 + usize::from(self.regs[7] & 0x0f)]);
        for y in 0..self.active_height() as usize {
            for x in 0..8usize {
                let pixel = y * WIDTH as usize + x;
                self.bg_priority[pixel] = false;
                let offset = pixel * 4;
                self.video.pixels_mut()[offset..offset + 4].copy_from_slice(&backdrop);
            }
        }
    }

    fn sprite_line_entries(&self, screen_y: usize) -> ([usize; 8], usize, bool) {
        let base = (usize::from(self.regs[5] & 0x7e) << 7) & 0x3fff;
        let sprite_height = if self.regs[1] & 0x02 != 0 { 16 } else { 8 };
        let zoom = if self.regs[1] & 0x01 != 0 { 2 } else { 1 };
        let mut entries = [0usize; 8];
        let mut count = 0usize;
        for sprite in 0..64usize {
            let raw_y = self.vram[(base + sprite) & 0x3fff];
            if raw_y == 0xd0 && self.active_height() == HEIGHT {
                break;
            }
            let mut y = i32::from(raw_y) + 1;
            if raw_y >= 0xe0 {
                y -= 256;
            }
            let delta = screen_y as i32 - y;
            if delta < 0 || delta >= sprite_height * zoom {
                continue;
            }
            if count == entries.len() {
                return (entries, count, true);
            }
            entries[count] = sprite;
            count += 1;
        }
        (entries, count, false)
    }

    fn sprite_line_pixel(&self, sprite: usize, screen_y: usize, screen_x: usize) -> Option<u8> {
        let base = (usize::from(self.regs[5] & 0x7e) << 7) & 0x3fff;
        let pair = (base + 0x80 + sprite * 2) & 0x3fff;
        let raw_y = self.vram[(base + sprite) & 0x3fff];
        let mut y = i32::from(raw_y) + 1;
        if raw_y >= 0xe0 {
            y -= 256;
        }
        let mut x = i32::from(self.vram[pair]);
        if self.regs[0] & 0x08 != 0 {
            x -= 8;
        }
        let sprite_height = if self.regs[1] & 0x02 != 0 { 16 } else { 8 };
        let zoom = if self.regs[1] & 0x01 != 0 { 2 } else { 1 };
        let source_y = (screen_y as i32 - y) / zoom;
        let source_x = (screen_x as i32 - x) / zoom;
        if !(0..sprite_height).contains(&source_y) || !(0..8).contains(&source_x) {
            return None;
        }
        let mut tile = self.vram[(pair + 1) & 0x3fff];
        if sprite_height == 16 {
            tile &= 0xfe;
        }
        let tile_offset = source_y as usize / 8;
        let row = source_y as usize & 7;
        let pattern_base = if self.regs[6] & 0x04 != 0 { 0x2000 } else { 0 };
        let pattern =
            (pattern_base + usize::from(tile.wrapping_add(tile_offset as u8)) * 32 + row * 4)
                & 0x3fff;
        let bit = 7 - source_x as usize;
        let mut color = 0u8;
        for plane in 0..4 {
            color |= ((self.vram[(pattern + plane) & 0x3fff] >> bit) & 1) << plane;
        }
        (color != 0).then_some(color)
    }

    fn legacy_sprite_line_entries(&self, screen_y: usize) -> ([usize; 4], usize, Option<usize>) {
        let base = (usize::from(self.regs[5] & 0x7f) << 7) & 0x3fff;
        let sprite_size = if self.regs[1] & 0x02 != 0 { 16 } else { 8 };
        let zoom = if self.regs[1] & 0x01 != 0 { 2 } else { 1 };
        let mut entries = [0usize; 4];
        let mut count = 0usize;
        for sprite in 0..32usize {
            let raw_y = self.vram[(base + sprite * 4) & 0x3fff];
            if raw_y == 0xd0 {
                break;
            }
            let mut y = i32::from(raw_y) + 1;
            if raw_y > 0xd0 {
                y -= 256;
            }
            let delta = screen_y as i32 - y;
            if delta < 0 || delta >= sprite_size * zoom {
                continue;
            }
            if count == entries.len() {
                return (entries, count, Some(sprite));
            }
            entries[count] = sprite;
            count += 1;
        }
        (entries, count, None)
    }

    fn legacy_sprite_pixel(&self, sprite: usize, screen_y: usize, screen_x: usize) -> Option<u8> {
        let attr_base = (usize::from(self.regs[5] & 0x7f) << 7) & 0x3fff;
        let base = (attr_base + sprite * 4) & 0x3fff;
        let raw_y = self.vram[base];
        let mut y = i32::from(raw_y) + 1;
        if raw_y > 0xd0 {
            y -= 256;
        }
        let color = self.vram[(base + 3) & 0x3fff];
        let mut x = i32::from(self.vram[(base + 1) & 0x3fff]);
        if color & 0x80 != 0 {
            x -= 32;
        }
        let size = if self.regs[1] & 0x02 != 0 { 16 } else { 8 };
        let zoom = if self.regs[1] & 0x01 != 0 { 2 } else { 1 };
        let source_y = (screen_y as i32 - y) / zoom;
        let source_x = (screen_x as i32 - x) / zoom;
        if !(0..size).contains(&source_y) || !(0..size).contains(&source_x) {
            return None;
        }
        let mut pattern = self.vram[(base + 2) & 0x3fff];
        if size == 16 {
            pattern &= 0xfc;
        }
        let tile = usize::from(pattern) + (source_y as usize / 8) * 2 + source_x as usize / 8;
        let row = source_y as usize & 7;
        let bit = 7 - (source_x as usize & 7);
        let pattern_base = (usize::from(self.regs[6] & 0x07) << 11) & 0x3fff;
        let byte = self.vram[(pattern_base + tile * 8 + row) & 0x3fff];
        let color = color & 0x0f;
        (color != 0 && byte & (1 << bit) != 0).then_some(color)
    }

    fn render_legacy_sprites(&mut self) {
        for y in 0..HEIGHT as usize {
            let (entries, count, _) = self.legacy_sprite_line_entries(y);
            for x in 0..WIDTH as usize {
                let Some(color) = entries[..count]
                    .iter()
                    .find_map(|sprite| self.legacy_sprite_pixel(*sprite, y, x))
                else {
                    continue;
                };
                let rgba = self.legacy_color(color);
                let offset = (y * WIDTH as usize + x) * 4;
                self.video.pixels_mut()[offset..offset + 4].copy_from_slice(&rgba);
            }
        }
    }

    fn latch_sprite_status_for_scanline(&mut self, screen_y: usize) {
        if screen_y >= self.active_height() as usize || self.regs[1] & 0x40 == 0 {
            return;
        }

        if self.mode4_enabled() {
            let (entries, count, overflow) = self.sprite_line_entries(screen_y);
            if overflow {
                self.status |= 0x40;
            }
            let mut occupied = [false; WIDTH as usize];
            for &sprite in &entries[..count] {
                for (x, occupied_pixel) in occupied.iter_mut().enumerate() {
                    if self.sprite_line_pixel(sprite, screen_y, x).is_none() {
                        continue;
                    }
                    if *occupied_pixel {
                        self.status |= 0x20;
                        return;
                    }
                    *occupied_pixel = true;
                }
            }
            return;
        }

        if self.legacy_mode() == LegacyMode::Text {
            return;
        }
        let (entries, count, overflow_sprite) = self.legacy_sprite_line_entries(screen_y);
        if let Some(sprite) = overflow_sprite {
            self.status = (self.status & 0xe0) | 0x40 | (sprite as u8 & 0x1f);
        }
        let mut occupied = [false; WIDTH as usize];
        for &sprite in &entries[..count] {
            for (x, occupied_pixel) in occupied.iter_mut().enumerate() {
                if self.legacy_sprite_pixel(sprite, screen_y, x).is_none() {
                    continue;
                }
                if *occupied_pixel {
                    self.status |= 0x20;
                    return;
                }
                *occupied_pixel = true;
            }
        }
    }

    fn render_sprites(&mut self) {
        for y in 0..self.active_height() as usize {
            let (entries, count, _) = self.sprite_line_entries(y);
            for x in 0..WIDTH as usize {
                let Some(color) = entries[..count]
                    .iter()
                    .find_map(|sprite| self.sprite_line_pixel(*sprite, y, x))
                else {
                    continue;
                };
                let pixel = y * WIDTH as usize + x;
                if self.bg_priority[pixel] {
                    continue;
                }
                let rgba = sms_color(self.cram[16 + usize::from(color)]);
                let offset = pixel * 4;
                self.video.pixels_mut()[offset..offset + 4].copy_from_slice(&rgba);
            }
        }
    }

    fn save(&self, out: &mut StateWriter) {
        out.blob(&self.vram);
        out.blob(&self.cram);
        out.blob(&self.regs);
        out.u16(self.address);
        out.u8(self.code);
        out.u8(self.control_latch.unwrap_or(0));
        out.u8(self.control_latch.is_some() as u8);
        out.u8(self.read_buffer);
        out.u8(self.status);
        out.u8(self.line_counter);
        out.u8(self.line_irq_pending as u8);
        out.u16(self.dot);
        out.u8(self.dot_half);
        out.u8(self.h_counter_latch);
        out.u16(self.scanline);
        out.u64(self.frame);
        out.u8(self.irq_pending as u8);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let vram = input.blob()?;
        if vram.len() != self.vram.len() {
            return Err("invalid SMS VRAM state length".into());
        }
        self.vram.copy_from_slice(vram);
        let cram = input.blob()?;
        if cram.len() != self.cram.len() {
            return Err("invalid SMS CRAM state length".into());
        }
        self.cram.copy_from_slice(cram);
        let regs = input.blob()?;
        if regs.len() != self.regs.len() {
            return Err("invalid SMS VDP register state length".into());
        }
        self.regs.copy_from_slice(regs);
        self.address = input.u16()? & 0x3fff;
        self.code = input.u8()? & 3;
        let latch = input.u8()?;
        self.control_latch = if input.u8()? != 0 { Some(latch) } else { None };
        self.read_buffer = input.u8()?;
        self.status = input.u8()?;
        self.line_counter = input.u8()?;
        self.line_irq_pending = input.u8()? != 0;
        self.dot = input.u16()?;
        self.dot_half = input.u8()?;
        self.h_counter_latch = input.u8()?;
        self.scanline = input.u16()?;
        self.frame = input.u64()?;
        self.irq_pending = input.u8()? != 0;
        self.render_frame();
        Ok(())
    }
}

fn sms_color(value: u8) -> [u8; 4] {
    let r = (value & 3) * 85;
    let g = ((value >> 2) & 3) * 85;
    let b = ((value >> 4) & 3) * 85;
    [r, g, b, 255]
}

struct SmsBus {
    ram: [u8; 0x2000],
    cartridge: SegaCartridge,
    vdp: SmsVdp,
    psg: Sn76489,
    fm: Ym2413,
    fm_present: bool,
    audio_control: u8,
    port_dc: u8,
    port_dd: u8,
    io_control: u8,
    phaser_active: [bool; 2],
    phaser_x: [i16; 2],
    phaser_y: [i16; 2],
    phaser_sensor_high: [bool; 2],
}

impl SmsBus {
    fn new(cartridge: SegaCartridge) -> Self {
        Self::with_fm(cartridge, true)
    }

    fn with_fm(cartridge: SegaCartridge, fm_present: bool) -> Self {
        Self {
            ram: [0; 0x2000],
            cartridge,
            vdp: SmsVdp::default(),
            psg: Sn76489::default(),
            fm: Ym2413::default(),
            fm_present,
            audio_control: 0,
            port_dc: 0xff,
            port_dd: 0xff,
            io_control: 0xff,
            phaser_active: [false; 2],
            phaser_x: [0; 2],
            phaser_y: [0; 2],
            phaser_sensor_high: [true; 2],
        }
    }

    fn pointer_coordinate(axis: i16, extent: u32) -> i16 {
        let normalized = i64::from(axis) - i64::from(i16::MIN);
        let span = i64::from(extent.saturating_sub(1));
        ((normalized * span) / i64::from(u16::MAX)) as i16
    }

    fn set_inputs(&mut self, input: &InputState) {
        let p1 = input.buttons[0];
        let p2 = input.buttons[1];
        self.port_dc = 0xff;
        self.port_dd = 0xff;
        self.phaser_active = [p1 & POINTER_TOUCH != 0, p2 & POINTER_TOUCH != 0];
        let active_height = self.vdp.active_height();
        for player in 0..2 {
            self.phaser_x[player] = Self::pointer_coordinate(input.axes[player][AXIS_AUX_X], WIDTH);
            self.phaser_y[player] =
                Self::pointer_coordinate(input.axes[player][AXIS_AUX_Y], active_height);
        }

        if self.phaser_active[0] {
            Self::clear_if_pressed(&mut self.port_dc, 4, p1 & (POINTER_CLICK | FACE_SOUTH) != 0);
        } else {
            Self::clear_if_pressed(&mut self.port_dc, 0, p1 & UP != 0);
            Self::clear_if_pressed(&mut self.port_dc, 1, p1 & DOWN != 0);
            Self::clear_if_pressed(&mut self.port_dc, 2, p1 & LEFT != 0);
            Self::clear_if_pressed(&mut self.port_dc, 3, p1 & RIGHT != 0);
            Self::clear_if_pressed(&mut self.port_dc, 4, p1 & FACE_SOUTH != 0);
            Self::clear_if_pressed(&mut self.port_dc, 5, p1 & FACE_EAST != 0);
        }

        if self.phaser_active[1] {
            Self::clear_if_pressed(&mut self.port_dd, 2, p2 & (POINTER_CLICK | FACE_SOUTH) != 0);
        } else {
            Self::clear_if_pressed(&mut self.port_dc, 6, p2 & UP != 0);
            Self::clear_if_pressed(&mut self.port_dc, 7, p2 & DOWN != 0);
            Self::clear_if_pressed(&mut self.port_dd, 0, p2 & LEFT != 0);
            Self::clear_if_pressed(&mut self.port_dd, 1, p2 & RIGHT != 0);
            Self::clear_if_pressed(&mut self.port_dd, 2, p2 & FACE_SOUTH != 0);
            Self::clear_if_pressed(&mut self.port_dd, 3, p2 & FACE_EAST != 0);
        }

        Self::clear_if_pressed(&mut self.port_dd, 4, p1 & SELECT != 0);
        self.update_light_phasers();
    }

    fn clear_if_pressed(value: &mut u8, bit: u8, pressed: bool) {
        if pressed {
            *value &= !(1 << bit);
        }
    }

    fn th_level(control: u8, direction_bit: u8, output_bit: u8, input_high: bool) -> bool {
        if control & (1 << direction_bit) != 0 {
            input_high
        } else {
            control & (1 << output_bit) != 0
        }
    }

    fn th_input(&self, player: usize) -> bool {
        if self.phaser_active[player] {
            self.phaser_sensor_high[player]
        } else {
            true
        }
    }

    fn th_line_level(&self, player: usize) -> bool {
        let (direction_bit, output_bit) = if player == 0 { (1, 5) } else { (3, 7) };
        Self::th_level(
            self.io_control,
            direction_bit,
            output_bit,
            self.th_input(player),
        )
    }

    fn sync_th_lines(&mut self) {
        for (bit, high) in [(6, self.th_line_level(0)), (7, self.th_line_level(1))] {
            if high {
                self.port_dd |= 1 << bit;
            } else {
                self.port_dd &= !(1 << bit);
            }
        }
    }

    fn write_io_control(&mut self, value: u8) {
        let old_a = self.th_line_level(0);
        let old_b = self.th_line_level(1);
        self.io_control = value;
        let new_a = self.th_line_level(0);
        let new_b = self.th_line_level(1);
        if (old_a && !new_a) || (old_b && !new_b) {
            self.vdp.latch_h_counter();
        }
        self.sync_th_lines();
    }

    fn update_light_phasers(&mut self) {
        let beam_x = i16::from(self.vdp.current_h_counter()) * 2;
        let beam_y = self.vdp.scanline as i16;
        let active_height = self.vdp.active_height() as i16;

        for player in 0..2 {
            let old_line = self.th_line_level(player);
            let sensor_high = if self.phaser_active[player] && beam_y < active_height {
                let dx = i32::from(self.phaser_x[player]) - i32::from(beam_x);
                let dy = i32::from(self.phaser_y[player]) - i32::from(beam_y);
                dx.abs() > 60 || dy.abs() > 5
            } else {
                true
            };
            self.phaser_sensor_high[player] = sensor_high;
            let new_line = self.th_line_level(player);
            if old_line && !new_line {
                self.vdp.latch_h_counter();
            }
        }
        self.sync_th_lines();
    }

    fn psg_output_enabled(&self) -> bool {
        !self.fm_present || matches!(self.audio_control & 3, 0 | 3)
    }

    fn fm_output_enabled(&self) -> bool {
        self.fm_present && self.audio_control & 1 != 0
    }

    fn tick(&mut self, cycles: u32) {
        self.vdp.tick_cpu_cycles(cycles);
        self.update_light_phasers();
        self.psg.tick_cpu_cycles(cycles);
        if self.fm_present {
            self.fm.tick_cpu_cycles(cycles);
        }
    }

    fn save(&self, out: &mut StateWriter) {
        out.blob(&self.ram);
        self.cartridge.save(out);
        self.vdp.save(out);
        self.psg.save(out);
        out.u8(self.fm_present as u8);
        out.u8(self.audio_control);
        self.fm.save(out);
        out.u8(self.port_dc);
        out.u8(self.port_dd);
        out.u8(self.io_control);
        for active in self.phaser_active {
            out.u8(active as u8);
        }
        for value in self.phaser_x {
            out.u16(value as u16);
        }
        for value in self.phaser_y {
            out.u16(value as u16);
        }
        for high in self.phaser_sensor_high {
            out.u8(high as u8);
        }
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let ram = input.blob()?;
        if ram.len() != self.ram.len() {
            return Err("invalid Master System RAM state length".into());
        }
        self.ram.copy_from_slice(ram);
        self.cartridge.load(input)?;
        self.vdp.load(input)?;
        self.psg.load(input)?;
        let fm_present = input.u8()? != 0;
        if fm_present != self.fm_present {
            return Err("Master System save state FM hardware profile differs from machine".into());
        }
        self.audio_control = input.u8()? & 3;
        self.fm.load(input)?;
        self.port_dc = input.u8()?;
        self.port_dd = input.u8()?;
        self.io_control = input.u8()?;
        for active in &mut self.phaser_active {
            *active = input.u8()? != 0;
        }
        for value in &mut self.phaser_x {
            *value = input.u16()? as i16;
            if !(0..WIDTH as i16).contains(value) {
                return Err("invalid Master System Light Phaser X target".into());
            }
        }
        for value in &mut self.phaser_y {
            *value = input.u16()? as i16;
            if !(0..FULL_HEIGHT as i16).contains(value) {
                return Err("invalid Master System Light Phaser Y target".into());
            }
        }
        for high in &mut self.phaser_sensor_high {
            *high = input.u8()? != 0;
        }
        Ok(())
    }
}

impl Z80Bus for SmsBus {
    fn mem_read(&mut self, address: u16) -> u8 {
        match address {
            0x0000..=0xbfff => self.cartridge.read(address),
            0xc000..=0xffff => self.ram[(address as usize) & 0x1fff],
        }
    }

    fn mem_write(&mut self, address: u16, value: u8) {
        match address {
            0x0000..=0xbfff => self.cartridge.write(address, value),
            0xc000..=0xffff => {
                self.ram[(address as usize) & 0x1fff] = value;
                if address >= 0xfffc {
                    self.cartridge.write_mapper(address, value);
                }
            }
        }
    }

    fn io_read(&mut self, port: u16) -> u8 {
        let low = port as u8;
        match low {
            0x40..=0x7f if low & 1 == 0 => self.vdp.v_counter(),
            0x40..=0x7f => self.vdp.h_counter(),
            0x80..=0xbf if low & 1 == 0 => self.vdp.read_data(),
            0x80..=0xbf => self.vdp.read_status(),
            0xf0 | 0xf1 if self.fm_present => 0xff,
            0xf2 if self.fm_present => self.audio_control & 3,
            0xf2 => 0x02,
            0xc0..=0xff if low & 1 == 0 => self.port_dc,
            0xc0..=0xff => self.port_dd,
            _ => 0xff,
        }
    }

    fn io_write(&mut self, port: u16, value: u8) {
        let low = port as u8;
        match low {
            0x3f => self.write_io_control(value),
            0x40..=0x7f => self.psg.write(value),
            0x80..=0xbf if low & 1 == 0 => self.vdp.write_data(value),
            0x80..=0xbf => self.vdp.write_control(value),
            0xf0 if self.fm_present => self.fm.select_register(value),
            0xf1 if self.fm_present => self.fm.write_data(value),
            0xf2 if self.fm_present => self.audio_control = value & 3,
            _ => {}
        }
    }
}

pub struct MasterSystemMachine {
    cpu: Z80,
    bus: SmsBus,
    audio: AudioBuffer,
    powered: bool,
    pause_pressed: bool,
}

impl MasterSystemMachine {
    pub fn from_rom(rom: &[u8]) -> Result<Self, String> {
        let cartridge = SegaCartridge::new(rom)?;
        let bus = SmsBus::new(cartridge);
        Ok(Self {
            cpu: Z80::default(),
            bus,
            audio: AudioBuffer::new(48_000, 2),
            powered: true,
            pause_pressed: false,
        })
    }

    fn clock_instruction(&mut self) -> u32 {
        let cycles = self.cpu.step(&mut self.bus);
        self.bus.tick(cycles);
        if self.bus.vdp.irq_pending {
            let irq_cycles = self.cpu.irq(&mut self.bus, 0xff);
            if irq_cycles != 0 {
                self.bus.tick(irq_cycles);
            }
        }
        cycles
    }

    fn flush_audio(&mut self) {
        self.audio.begin_frame();
        let psg_enabled = self.bus.psg_output_enabled();
        let fm_enabled = self.bus.fm_output_enabled();
        let psg = self.bus.psg.samples();
        let fm = self.bus.fm.samples();
        for index in 0..psg.len().max(fm.len()) {
            let psg_sample = if psg_enabled {
                psg.get(index).copied().unwrap_or(0.0)
            } else {
                0.0
            };
            let fm_sample = if fm_enabled {
                fm.get(index).copied().unwrap_or(0.0)
            } else {
                0.0
            };
            let sample = (psg_sample + fm_sample).clamp(-1.0, 1.0);
            self.audio.push_stereo(sample, sample);
        }
    }
}

impl Machine for MasterSystemMachine {
    fn platform(&self) -> PlatformId {
        PlatformId::MasterSystem
    }

    fn reset(&mut self) {
        self.cpu.reset();
        self.bus.ram = [0; 0x2000];
        self.bus.cartridge.reset_mapping();
        self.bus.vdp.reset();
        self.bus.psg.reset();
        self.bus.fm.reset();
        self.bus.audio_control = 0;
        self.bus.port_dc = 0xff;
        self.bus.port_dd = 0xff;
        self.bus.io_control = 0xff;
        self.powered = true;
        self.pause_pressed = false;
        self.audio.begin_frame();
    }

    fn run_frame(&mut self, input: &InputState) {
        self.bus.set_inputs(input);
        self.bus.psg.begin_frame();
        self.bus.fm.begin_frame();
        let target = self.bus.vdp.frame.wrapping_add(1);
        let pause_pressed = input.buttons[0] & START != 0;
        if pause_pressed && !self.pause_pressed {
            let cycles = self.cpu.nmi(&mut self.bus);
            self.bus.tick(cycles);
        }
        self.pause_pressed = pause_pressed;
        let cycle_budget = (CPU_CLOCK / FRAME_RATE * 2.0).ceil() as u64;
        let deadline = self.cpu.cycles.saturating_add(cycle_budget);
        while self.bus.vdp.frame != target && self.cpu.cycles < deadline {
            self.clock_instruction();
        }
        if self.bus.vdp.frame != target {
            self.powered = false;
        }
        self.flush_audio();
    }

    fn frame_rate(&self) -> f64 {
        FRAME_RATE
    }

    fn video(&self) -> &VideoBuffer {
        &self.bus.vdp.video
    }

    fn audio(&self) -> &AudioBuffer {
        &self.audio
    }

    fn save_state(&self) -> Result<Vec<u8>, String> {
        let mut out = StateWriter::new(PlatformId::MasterSystem, STATE_VERSION);
        self.cpu.save(&mut out);
        self.bus.save(&mut out);
        out.u8(self.powered as u8);
        out.u8(self.pause_pressed as u8);
        Ok(out.finish())
    }

    fn load_state(&mut self, bytes: &[u8]) -> Result<(), String> {
        let mut input = StateReader::new(bytes, PlatformId::MasterSystem, STATE_VERSION)?;
        self.cpu.load(&mut input)?;
        self.bus.load(&mut input)?;
        self.powered = input.u8()? != 0;
        self.pause_pressed = input.u8()? != 0;
        self.audio.begin_frame();
        input.finish()
    }

    fn persistent_len(&self, kind: ResourceKind, slot: u32) -> usize {
        if kind == ResourceKind::Storage && slot == 0 {
            self.bus.cartridge.sram.len()
        } else {
            0
        }
    }

    fn read_persistent(&self, kind: ResourceKind, slot: u32, out: &mut [u8]) -> Result<(), String> {
        let len = self.persistent_len(kind, slot);
        if len == 0 {
            return Err("Master System cartridge storage slot is unavailable".into());
        }
        if out.len() != len {
            return Err(format!(
                "persistent output has {} bytes; expected {len}",
                out.len()
            ));
        }
        out.copy_from_slice(&self.bus.cartridge.sram);
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
            return Err("Master System cartridge storage slot is unavailable".into());
        }
        if data.len() != len {
            return Err(format!(
                "persistent input has {} bytes; expected {len}",
                data.len()
            ));
        }
        self.bus.cartridge.sram.copy_from_slice(data);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn synthetic_rom() -> Vec<u8> {
        let mut rom = vec![0; 0x8000];
        let program: &[u8] = &[
            0xf3, 0x31, 0xf0, 0xdf, 0x3e, 0x04, 0xd3, 0xbf, 0x3e, 0x80, 0xd3, 0xbf, 0x3e, 0xc0,
            0xd3, 0xbf, 0x3e, 0x81, 0xd3, 0xbf, 0x3e, 0xff, 0xd3, 0xbf, 0x3e, 0x82, 0xd3, 0xbf,
            0x3e, 0xff, 0xd3, 0xbf, 0x3e, 0x85, 0xd3, 0xbf, 0x3e, 0xfb, 0xd3, 0xbf, 0x3e, 0x86,
            0xd3, 0xbf, 0x3e, 0x01, 0xd3, 0xbf, 0x3e, 0xc0, 0xd3, 0xbf, 0x3e, 0x03, 0xd3, 0xbe,
            0x3e, 0x00, 0xd3, 0xbf, 0x3e, 0x40, 0xd3, 0xbf, 0x21, 0x00, 0x01, 0x06, 0x20, 0x7e,
            0xd3, 0xbe, 0x23, 0x10, 0xfa, 0xc3, 0x4b, 0x00,
        ];
        rom[..program.len()].copy_from_slice(program);
        for row in 0..8usize {
            let base = 0x100 + row * 4;
            rom[base] = 0xff;
        }
        rom
    }

    fn pause_test_rom() -> Vec<u8> {
        let mut rom = vec![0; 0x8000];
        rom[..3].copy_from_slice(&[0xc3, 0x00, 0x00]);
        let handler = [0x3a, 0x00, 0xc0, 0x3c, 0x32, 0x00, 0xc0, 0xed, 0x45];
        rom[0x66..0x66 + handler.len()].copy_from_slice(&handler);
        rom
    }

    #[test]
    fn pause_button_generates_one_nmi_per_press_edge_and_state_preserves_latch() {
        let mut machine = MasterSystemMachine::from_rom(&pause_test_rom()).unwrap();
        let mut input = InputState::default();
        input.buttons[0] = START;
        machine.run_frame(&input);
        assert_eq!(machine.bus.ram[0], 1);

        machine.run_frame(&input);
        assert_eq!(machine.bus.ram[0], 1);
        input.buttons[0] = 0;
        machine.run_frame(&input);
        input.buttons[0] = START;
        machine.run_frame(&input);
        assert_eq!(machine.bus.ram[0], 2);

        let state = machine.save_state().unwrap();
        machine.load_state(&state).unwrap();
        machine.run_frame(&input);
        assert_eq!(machine.bus.ram[0], 2);
    }

    #[test]
    fn reset_button_is_active_low_on_port_dd_bit_four() {
        let cartridge = SegaCartridge::new(&vec![0; 0x8000]).unwrap();
        let mut bus = SmsBus::new(cartridge);
        let mut input = InputState::default();

        bus.set_inputs(&input);
        assert_eq!(bus.io_read(0xdd) & 0x10, 0x10);

        input.buttons[0] = SELECT;
        bus.set_inputs(&input);
        assert_eq!(bus.port_dd & 0x10, 0);
        assert_eq!(bus.io_read(0xdd) & 0x10, 0);

        input.buttons[0] = 0;
        bus.set_inputs(&input);
        assert_eq!(bus.io_read(0xdd) & 0x10, 0x10);
    }

    #[test]
    fn h_counter_latches_on_th_falling_edge_and_skips_sync_range() {
        let cartridge = SegaCartridge::new(&vec![0; 0x8000]).unwrap();
        let mut bus = SmsBus::new(cartridge);

        bus.vdp.dot = 294;
        bus.write_io_control(0xfd);
        bus.write_io_control(0xdd);
        assert_eq!(bus.vdp.h_counter(), 0x93);
        assert_eq!(bus.io_read(0xdd) & 0x40, 0);

        bus.vdp.dot = 296;
        bus.write_io_control(0xdd);
        assert_eq!(bus.vdp.h_counter(), 0x93);
        bus.write_io_control(0xfd);
        bus.write_io_control(0xdd);
        assert_eq!(bus.vdp.h_counter(), 0xe9);

        bus.vdp.dot = 0;
        bus.write_io_control(0xfd);
        bus.write_io_control(0xdd);
        assert_eq!(bus.io_read(0x7f), 0);
    }

    #[test]
    fn light_phaser_drives_tl_th_and_latches_hcounter_on_optical_edge() {
        let cartridge = SegaCartridge::new(&vec![0; 0x8000]).unwrap();
        let mut bus = SmsBus::new(cartridge);
        bus.vdp.dot = 200;
        bus.vdp.scanline = 0;

        let mut input = InputState::default();
        input.buttons[0] = POINTER_TOUCH | POINTER_CLICK;
        input.axes[0][AXIS_AUX_X] = i16::MIN;
        input.axes[0][AXIS_AUX_Y] = i16::MIN;
        bus.set_inputs(&input);

        assert_eq!(bus.io_read(0xdc) & 0x10, 0);
        assert_ne!(bus.io_read(0xdd) & 0x40, 0);

        bus.vdp.dot = 0;
        bus.update_light_phasers();
        assert_eq!(bus.io_read(0xdd) & 0x40, 0);
        assert_eq!(bus.vdp.h_counter(), 0);

        bus.vdp.scanline = 6;
        bus.update_light_phasers();
        assert_ne!(bus.io_read(0xdd) & 0x40, 0);

        input.buttons[0] = POINTER_TOUCH;
        bus.set_inputs(&input);
        assert_ne!(bus.io_read(0xdc) & 0x10, 0);

        let mut out = StateWriter::new(PlatformId::MasterSystem, STATE_VERSION);
        bus.save(&mut out);
        let state = out.finish();

        let cartridge = SegaCartridge::new(&vec![0; 0x8000]).unwrap();
        let mut restored = SmsBus::new(cartridge);
        let mut reader = StateReader::new(&state, PlatformId::MasterSystem, STATE_VERSION).unwrap();
        restored.load(&mut reader).unwrap();
        reader.finish().unwrap();

        assert!(restored.phaser_active[0]);
        assert!(!restored.phaser_active[1]);
        assert_eq!(restored.phaser_x[0], 0);
        assert_eq!(restored.phaser_y[0], 0);
        assert_eq!(restored.phaser_sensor_high[0], bus.phaser_sensor_high[0]);
    }

    #[test]
    fn fm_ports_drive_ym2413_and_audio_control_selects_chip_outputs() {
        let cartridge = SegaCartridge::new(&vec![0; 0x8000]).unwrap();
        let mut bus = SmsBus::new(cartridge);
        assert_eq!(bus.io_read(0xf2) & 3, 0);
        assert!(bus.psg_output_enabled());
        assert!(!bus.fm_output_enabled());

        for (register, value) in [(0x30, 0x10), (0x10, 0x80), (0x20, 0x17)] {
            bus.io_write(0xf0, register);
            bus.io_write(0xf1, value);
        }
        bus.fm.begin_frame();
        bus.tick((CPU_CLOCK as u32) / 20);
        assert!(bus.fm.samples().iter().any(|sample| sample.abs() > 0.0001));

        for (control, psg, fm) in [
            (0, true, false),
            (1, false, true),
            (2, false, false),
            (3, true, true),
        ] {
            bus.io_write(0xf2, control);
            assert_eq!(bus.io_read(0xf2) & 3, control);
            assert_eq!(bus.psg_output_enabled(), psg);
            assert_eq!(bus.fm_output_enabled(), fm);
        }

        let cartridge = SegaCartridge::new(&vec![0; 0x8000]).unwrap();
        let mut no_fm = SmsBus::with_fm(cartridge, false);
        no_fm.io_write(0xf2, 3);
        assert_eq!(no_fm.io_read(0xf2) & 3, 2);
        assert!(no_fm.psg_output_enabled());
        assert!(!no_fm.fm_output_enabled());
    }

    #[test]
    fn synthetic_cartridge_runs_z80_vdp_and_psg_frame() {
        let rom = synthetic_rom();
        let mut machine = MasterSystemMachine::from_rom(&rom).unwrap();
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
        assert!(!machine.audio().samples().is_empty());
    }

    #[test]
    fn save_state_round_trip_restores_cpu_and_video_state() {
        let rom = synthetic_rom();
        let mut machine = MasterSystemMachine::from_rom(&rom).unwrap();
        machine.run_frame(&InputState::default());
        let saved = machine.save_state().unwrap();
        let pc = machine.cpu.pc;
        let frame = machine.bus.vdp.frame;
        machine.run_frame(&InputState::default());
        machine.load_state(&saved).unwrap();
        assert_eq!(machine.cpu.pc, pc);
        assert_eq!(machine.bus.vdp.frame, frame);
    }

    #[test]
    fn sega_mapper_switches_rom_and_cartridge_ram() {
        let mut rom = vec![0; 0x10000];
        for bank in 0..4usize {
            rom[bank * 0x4000] = bank as u8;
        }
        let mut cart = SegaCartridge::new(&rom).unwrap();
        assert_eq!(cart.read(0x4000), 1);
        cart.write_mapper(0xfffe, 3);
        assert_eq!(cart.read(0x4000), 3);
        cart.write_mapper(0xfffc, 0x08);
        cart.write(0x8000, 0x5a);
        assert_eq!(cart.read(0x8000), 0x5a);
        cart.write_mapper(0xfffc, 0x0c);
        cart.write(0x8000, 0xa5);
        assert_eq!(cart.read(0x8000), 0xa5);
        cart.write_mapper(0xfffc, 0x08);
        assert_eq!(cart.read(0x8000), 0x5a);
    }

    #[test]
    fn codemasters_header_selects_mapper_and_full_slot_bank_registers() {
        let mut rom = vec![0; 0x10000];
        for bank in 0..4usize {
            rom[bank * 0x4000..(bank + 1) * 0x4000].fill(bank as u8);
        }
        rom[0x7fe0..0x7ff0].copy_from_slice(&[
            0x04, 0x15, 0x09, 0x93, 0x10, 0x59, 0x34, 0x12, 0xcc, 0xed, 0, 0, 0, 0, 0, 0,
        ]);

        let mut cart = SegaCartridge::new(&rom).unwrap();
        assert_eq!(cart.mapper, MapperKind::Codemasters);
        assert_eq!(cart.sram.len(), 0x2000);
        assert_eq!(cart.read(0x0000), 0);
        assert_eq!(cart.read(0x4000), 1);
        assert_eq!(cart.read(0x8000), 0);

        cart.write(0x0000, 2);
        cart.write(0x4000, 3);
        cart.write(0x8000, 3);
        assert_eq!(cart.read(0x0000), 2);
        assert_eq!(cart.read(0x4000), 3);
        assert_eq!(cart.read(0x8000), 3);

        cart.write(0x4000, 0x83);
        cart.write(0xa000, 0x5a);
        assert_eq!(cart.read(0x4000), 3);
        assert_eq!(cart.read(0x8000), 3);
        assert_eq!(cart.read(0xa000), 0x5a);
        cart.write(0x4000, 3);
        assert_eq!(cart.read(0xa000), 3);

        cart.reset_mapping();
        assert_eq!(cart.read(0x0000), 0);
        assert_eq!(cart.read(0x4000), 1);
        assert_eq!(cart.read(0x8000), 0);
    }

    #[test]
    fn korean_mapper_signature_selects_a000_slot_two_banking() {
        let mut rom = vec![0; 0x10000];
        for bank in 0..4usize {
            rom[bank * 0x4000..(bank + 1) * 0x4000].fill(bank as u8);
        }
        rom[0x100..0x103].copy_from_slice(&[0x32, 0x00, 0xa0]);

        let mut cart = SegaCartridge::new(&rom).unwrap();
        assert_eq!(cart.mapper, MapperKind::Korean);
        assert!(cart.sram.is_empty());
        assert_eq!(cart.read(0x0000), 0);
        assert_eq!(cart.read(0x4000), 1);
        assert_eq!(cart.read(0x8000), 2);
        cart.write(0xa000, 3);
        assert_eq!(cart.read(0x8000), 3);
        assert_eq!(cart.read(0xa000), 3);
        cart.reset_mapping();
        assert_eq!(cart.read(0x8000), 2);

        rom[0x7ff0..0x7ff8].copy_from_slice(b"TMR SEGA");
        assert_eq!(SegaCartridge::new(&rom).unwrap().mapper, MapperKind::Sega);
    }

    #[test]
    fn crc32_ieee_matches_standard_check_value() {
        assert_eq!(SegaCartridge::crc32_ieee(b"123456789"), 0xcbf4_3926);
    }

    #[test]
    fn janggun_mapper_switches_eight_kib_banks_reverses_bits_and_restores_state() {
        let mut rom = vec![0; 64 * 0x2000];
        for bank in 0..64usize {
            rom[bank * 0x2000..(bank + 1) * 0x2000].fill((bank as u8) << 1 | 1);
        }
        let mut cart = SegaCartridge {
            rom,
            sram: Vec::new(),
            mapper: MapperKind::Janggun,
            control: 0,
            pages: [0, 1, 2],
            janggun_banks: [2, 3, 4, 5],
            janggun_reverse: [false; 4],
            msx_banks: [2, 3, 4, 5],
        };

        assert_eq!(cart.read(0x4000), 5);
        assert_eq!(cart.read(0x6000), 7);
        assert_eq!(cart.read(0x8000), 9);
        assert_eq!(cart.read(0xa000), 11);

        cart.write(0x4000, 0x86);
        assert_eq!(cart.read(0x4000), 13u8.reverse_bits());
        cart.write_mapper(0xfffe, 0x45);
        assert_eq!(cart.read(0x4000), 21u8.reverse_bits());
        assert_eq!(cart.read(0x6000), 23u8.reverse_bits());
        cart.write_mapper(0xffff, 0x07);
        assert_eq!(cart.read(0x8000), 29);
        assert_eq!(cart.read(0xa000), 31);

        let mut writer = StateWriter::new(PlatformId::MasterSystem, 99);
        cart.save(&mut writer);
        let bytes = writer.finish();
        let mut restored = SegaCartridge {
            rom: cart.rom.clone(),
            sram: Vec::new(),
            mapper: MapperKind::Janggun,
            control: 0,
            pages: [0, 1, 2],
            janggun_banks: [2, 3, 4, 5],
            janggun_reverse: [false; 4],
            msx_banks: [2, 3, 4, 5],
        };
        let mut reader = StateReader::new(&bytes, PlatformId::MasterSystem, 99).unwrap();
        restored.load(&mut reader).unwrap();
        reader.finish().unwrap();
        assert_eq!(restored.janggun_banks, cart.janggun_banks);
        assert_eq!(restored.janggun_reverse, cart.janggun_reverse);
        assert_eq!(restored.read(0x4000), cart.read(0x4000));
        assert_eq!(restored.read(0x8000), cart.read(0x8000));

        restored.reset_mapping();
        assert_eq!(restored.janggun_banks, [2, 3, 4, 5]);
        assert_eq!(restored.janggun_reverse, [false; 4]);
    }

    #[test]
    fn crc16_ccitt_matches_standard_check_value() {
        assert_eq!(SegaCartridge::crc16_ccitt(b"123456789"), 0x29b1);
    }

    #[test]
    fn msx_mapper_detects_register_writes_and_maps_four_eight_kib_windows() {
        let mut rom = vec![0; 16 * 0x2000];
        for bank in 0..16usize {
            rom[bank * 0x2000..(bank + 1) * 0x2000].fill(bank as u8);
        }
        rom[0x100..0x103].copy_from_slice(&[0x32, 0x00, 0x00]);
        rom[0x110..0x113].copy_from_slice(&[0x32, 0x02, 0x00]);

        let mut cart = SegaCartridge::new(&rom).unwrap();
        assert_eq!(cart.mapper, MapperKind::Msx);
        assert_eq!(cart.read(0x0000), 0);
        assert_eq!(cart.read(0x2000), 1);
        assert_eq!(cart.read(0x4000), 2);
        assert_eq!(cart.read(0x6000), 3);
        assert_eq!(cart.read(0x8000), 4);
        assert_eq!(cart.read(0xa000), 5);

        cart.write(0x0000, 6);
        cart.write(0x0001, 7);
        cart.write(0x0002, 8);
        cart.write(0x0003, 9);
        assert_eq!(cart.read(0x4000), 8);
        assert_eq!(cart.read(0x6000), 9);
        assert_eq!(cart.read(0x8000), 6);
        assert_eq!(cart.read(0xa000), 7);

        let mut writer = StateWriter::new(PlatformId::MasterSystem, 99);
        cart.save(&mut writer);
        let bytes = writer.finish();
        let mut restored = SegaCartridge::new(&rom).unwrap();
        let mut reader = StateReader::new(&bytes, PlatformId::MasterSystem, 99).unwrap();
        restored.load(&mut reader).unwrap();
        reader.finish().unwrap();
        assert_eq!(restored.msx_banks, [8, 9, 6, 7]);
        assert_eq!(restored.read(0xa000), 7);

        restored.reset_mapping();
        assert_eq!(restored.msx_banks, [2, 3, 4, 5]);
    }

    #[test]
    fn nemesis_mapper_fixes_final_boot_page_and_switches_shared_msx_windows() {
        let mut rom = vec![0; 16 * 0x2000];
        for bank in 0..16usize {
            rom[bank * 0x2000..(bank + 1) * 0x2000].fill(bank as u8);
        }
        let mut cart = SegaCartridge {
            rom,
            sram: Vec::new(),
            mapper: MapperKind::Nemesis,
            control: 0,
            pages: [0, 1, 2],
            janggun_banks: [2, 3, 4, 5],
            janggun_reverse: [false; 4],
            msx_banks: [2, 3, 4, 5],
        };

        assert_eq!(cart.read(0x0000), 15);
        assert_eq!(cart.read(0x2000), 1);
        assert_eq!(cart.read(0x4000), 2);
        assert_eq!(cart.read(0x6000), 3);
        assert_eq!(cart.read(0x8000), 4);
        assert_eq!(cart.read(0xa000), 5);

        cart.write(0x0002, 10);
        cart.write(0x0003, 11);
        cart.write(0x0000, 12);
        cart.write(0x0001, 13);
        assert_eq!(cart.read(0x4000), 10);
        assert_eq!(cart.read(0x6000), 11);
        assert_eq!(cart.read(0x8000), 12);
        assert_eq!(cart.read(0xa000), 13);
        assert_eq!(cart.read(0x0000), 15);
        assert_eq!(cart.read(0x2000), 1);
    }

    #[test]
    fn left_column_mask_applies_after_sprite_composition() {
        let mut vdp = SmsVdp::default();
        vdp.regs[0] = 0x04;
        vdp.regs[1] = 0x40;
        vdp.regs[5] = 0x7e;
        vdp.cram[18] = 0x0c;
        vdp.vram[0x3f00] = 0xff;
        vdp.vram[0x3f01] = 0xd0;
        vdp.vram[0x3f80] = 0;
        vdp.vram[0x3f81] = 1;
        vdp.vram[33] = 0x80;

        vdp.render_frame();
        assert_eq!(&vdp.video.pixels()[..4], &[0, 255, 0, 255]);

        vdp.regs[0] |= 0x20;
        vdp.render_frame();
        assert_eq!(&vdp.video.pixels()[..4], &[0, 0, 0, 255]);
    }

    #[test]
    fn lower_numbered_sprite_wins_overlap_and_sets_collision() {
        let mut vdp = SmsVdp::default();
        vdp.regs[0] = 0x04;
        vdp.regs[1] = 0x40;
        vdp.regs[5] = 0x7e;
        vdp.cram[17] = 0x03;
        vdp.cram[18] = 0x0c;
        vdp.vram[0x3f00] = 0xff;
        vdp.vram[0x3f01] = 0xff;
        vdp.vram[0x3f02] = 0xd0;
        vdp.vram[0x3f80] = 0;
        vdp.vram[0x3f81] = 1;
        vdp.vram[0x3f82] = 0;
        vdp.vram[0x3f83] = 2;
        vdp.vram[32] = 0x80;
        vdp.vram[65] = 0x80;

        vdp.render_frame();
        assert_eq!(&vdp.video.pixels()[..4], &[255, 0, 0, 255]);
        assert_eq!(vdp.status & 0x20, 0);
        vdp.latch_sprite_status_for_scanline(0);
        assert_ne!(vdp.status & 0x20, 0);
    }

    #[test]
    fn sprite_status_latches_at_scanline_end_and_status_read_clears_it() {
        let mut vdp = SmsVdp::default();
        vdp.regs[0] = 0x04;
        vdp.regs[1] = 0x40;
        vdp.regs[5] = 0x7e;
        for sprite in 0..9usize {
            vdp.vram[0x3f00 + sprite] = 0xff;
            vdp.vram[0x3f80 + sprite * 2] = 0;
            vdp.vram[0x3f81 + sprite * 2] = 1;
        }
        vdp.vram[0x3f09] = 0xd0;
        vdp.vram[32] = 0x80;

        assert_eq!(vdp.status & 0x60, 0);
        vdp.tick_cpu_cycles(228);
        assert_eq!(vdp.scanline, 1);
        assert_eq!(vdp.status & 0x60, 0x60);
        assert_eq!(vdp.read_status() & 0x60, 0x60);
        assert_eq!(vdp.status & 0x60, 0);
    }

    #[test]
    fn mode4_priority_background_pixels_cover_sprites_but_color_zero_does_not() {
        let mut vdp = SmsVdp::default();
        vdp.regs[0] = 0x04;
        vdp.regs[1] = 0x40;
        vdp.regs[2] = 0x0e;
        vdp.regs[5] = 0x7e;
        vdp.cram[1] = 0x03;
        vdp.cram[17] = 0x0c;
        vdp.vram[0x3800] = 0;
        vdp.vram[0x3801] = 0x10;
        vdp.vram[0] = 0x80;
        vdp.vram[0x3f00] = 0xff;
        vdp.vram[0x3f01] = 0xd0;
        vdp.vram[0x3f80] = 0;
        vdp.vram[0x3f81] = 1;
        vdp.vram[32] = 0x80;

        vdp.render_frame();
        assert_eq!(&vdp.video.pixels()[..4], &[255, 0, 0, 255]);

        vdp.vram[0x3801] = 0;
        vdp.render_frame();
        assert_eq!(&vdp.video.pixels()[..4], &[0, 255, 0, 255]);

        vdp.vram[0x3801] = 0x10;
        vdp.vram[0] = 0;
        vdp.render_frame();
        assert_eq!(&vdp.video.pixels()[..4], &[0, 255, 0, 255]);
    }

    #[test]
    fn legacy_vdp_sprites_render_with_lower_number_priority() {
        let mut vdp = SmsVdp::default();
        vdp.regs[1] = 0x40;
        vdp.regs[5] = 0x3e;
        let base = 0x1f00usize;
        vdp.vram[base] = 0xff;
        vdp.vram[base + 1] = 0;
        vdp.vram[base + 2] = 1;
        vdp.vram[base + 3] = 2;
        vdp.vram[base + 4] = 0xd0;
        vdp.vram[8] = 0x80;

        vdp.render_frame();
        assert_eq!(&vdp.video.pixels()[..4], &[0, 170, 0, 255]);
    }

    #[test]
    fn legacy_vdp_sprite_collision_and_fifth_sprite_status_latch() {
        let mut vdp = SmsVdp::default();
        vdp.regs[1] = 0x40;
        vdp.regs[5] = 0x3e;
        let base = 0x1f00usize;
        for sprite in 0..5usize {
            let entry = base + sprite * 4;
            vdp.vram[entry] = 0xff;
            vdp.vram[entry + 1] = 0;
            vdp.vram[entry + 2] = 1;
            vdp.vram[entry + 3] = 2;
        }
        vdp.vram[base + 5 * 4] = 0xd0;
        vdp.vram[8] = 0x80;

        vdp.latch_sprite_status_for_scanline(0);
        assert_eq!(vdp.status & 0x60, 0x60);
        assert_eq!(vdp.status & 0x1f, 4);
    }

    #[test]
    fn extended_224_mode_uses_sms2_geometry_timing_and_sprite_rules() {
        let mut vdp = SmsVdp::default();
        vdp.regs[0] = 0x06;
        vdp.regs[1] = 0x50;
        vdp.regs[2] = 0x0c;
        vdp.regs[5] = 0x7e;
        vdp.cram[1] = 0x03;
        vdp.cram[17] = 0x0c;
        for sprite in 0..64usize {
            vdp.vram[0x3f00 + sprite] = 0xdf;
        }
        vdp.vram[0x3700] = 0;
        vdp.vram[0x3701] = 0;
        vdp.vram[0] = 0x80;
        vdp.vram[0x3f00] = 0xd0;
        vdp.vram[0x3f80] = 0;
        vdp.vram[0x3f81] = 1;
        vdp.vram[32] = 0x80;

        assert_eq!(vdp.active_height(), EXTENDED_HEIGHT);
        assert_eq!(vdp.name_table_base(), 0x3700);
        vdp.render_frame();
        assert_eq!(vdp.video.height(), EXTENDED_HEIGHT);
        assert_eq!(&vdp.video.pixels()[..4], &[255, 0, 0, 255]);
        let sprite_pixel = 209usize * WIDTH as usize * 4;
        assert_eq!(
            &vdp.video.pixels()[sprite_pixel..sprite_pixel + 4],
            &[0, 255, 0, 255]
        );

        vdp.status = 0;
        vdp.frame = 0;
        vdp.scanline = 223;
        vdp.dot = 341;
        vdp.dot_half = 0;
        vdp.tick_cpu_cycles(1);
        assert_eq!(vdp.scanline, 224);
        assert_eq!(vdp.frame, 1);
        assert_ne!(vdp.status & 0x80, 0);

        vdp.scanline = 234;
        assert_eq!(vdp.v_counter(), 0xea);
        vdp.scanline = 235;
        assert_eq!(vdp.v_counter(), 0xe5);
    }

    #[test]
    fn extended_240_mode_uses_full_height_tilemap_and_counter_domain() {
        let mut vdp = SmsVdp::default();
        vdp.regs[0] = 0x06;
        vdp.regs[1] = 0x48;
        vdp.regs[2] = 0x0c;
        vdp.cram[1] = 0x03;
        vdp.vram[0x3700] = 0;
        vdp.vram[0x3701] = 0;
        vdp.vram[0] = 0x80;

        assert_eq!(vdp.active_height(), FULL_HEIGHT);
        assert!(vdp.extended_height_mode());
        assert_eq!(vdp.name_table_base(), 0x3700);
        vdp.render_frame();
        assert_eq!(vdp.video.height(), FULL_HEIGHT);

        vdp.status = 0;
        vdp.frame = 0;
        vdp.scanline = 239;
        vdp.dot = 341;
        vdp.dot_half = 0;
        vdp.tick_cpu_cycles(1);
        assert_eq!(vdp.scanline, 240);
        assert_eq!(vdp.frame, 1);
        assert_ne!(vdp.status & 0x80, 0);

        vdp.scanline = 255;
        assert_eq!(vdp.v_counter(), 0xff);
        vdp.scanline = 256;
        assert_eq!(vdp.v_counter(), 0x00);
        vdp.scanline = 261;
        assert_eq!(vdp.v_counter(), 0x05);
    }

    #[test]
    fn line_interrupt_counter_asserts_and_status_acknowledges_it() {
        let mut vdp = SmsVdp::default();
        vdp.regs[0] = 0x14;
        vdp.regs[10] = 1;
        vdp.line_counter = 1;
        vdp.tick_cpu_cycles(456);
        assert!(vdp.line_irq_pending);
        assert!(vdp.irq_pending);
        let _ = vdp.read_status();
        assert!(!vdp.line_irq_pending);
        assert!(!vdp.irq_pending);
    }
}
