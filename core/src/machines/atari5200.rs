use super::pokey::Pokey;
use crate::bus::Bus8;
use crate::cpu6502::Mos6502;
use crate::input::{
    AXIS_LEFT_X, AXIS_LEFT_Y, DOWN, FACE_EAST, FACE_SOUTH, KEYPAD_0, KEYPAD_1, KEYPAD_2, KEYPAD_3,
    KEYPAD_4, KEYPAD_5, KEYPAD_6, KEYPAD_7, KEYPAD_8, KEYPAD_9, KEYPAD_HASH, KEYPAD_STAR, LEFT,
    PAUSE, RESET, RIGHT, SELECT, START, UP,
};
use crate::kernel::{AudioBuffer, InputState, VideoBuffer};
use crate::machine::Machine;
use crate::platform::PlatformId;
use crate::state::{StateReader, StateWriter};

const WIDTH: u32 = 320;
const HEIGHT: u32 = 192;
const CPU_HZ: u64 = 1_789_790;
const FRAME_RATE: f64 = 59.94;
const CPU_CYCLES_PER_LINE: u16 = 114;
const SCANLINES: u16 = 262;
const STATE_VERSION: u32 = 4;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Atari5200Mapper {
    Linear32K,
    TwoChip16K,
    OneChip16K,
    Standard8K,
    Standard4K,
    BountyBob40K,
    BountyBob40KAlt,
    SuperCart,
}

const KNOWN_TWO_CHIP_16K_CRC32: &[u32] = &[
    0x0480_7705,
    0x04b2_99a4,
    0x0af1_9345,
    0x0f99_6184,
    0x10f3_3c90,
    0x1d1c_ee27,
    0x2a64_0143,
    0x2c67_6662,
    0x4019_ecec,
    0x4336_c2cc,
    0x536a_70fe,
    0x5998_3c40,
    0x69f2_3548,
    0x6a68_7f9c,
    0x73b5_b6fb,
    0x752f_5efd,
    0x75f5_66df,
    0x7d81_9a9f,
    0x84df_4925,
    0x8873_ef51,
    0x931a_454a,
    0xa18a_9a40,
    0xa976_06ab,
    0xabc2_d1e4,
    0xaea6_d2c2,
    0xb3b8_e314,
    0xb68d_61e8,
    0xb8fa_aec3,
    0xbd52_623b,
    0xbfd3_0c01,
    0xc597_c087,
    0xcfd4_a7f9,
    0xce07_d9ad,
    0xd9ae_4518,
    0xe8b1_30c4,
    0xecbd_1853,
    0xecfa_624f,
    0xf1f4_2bbd,
    0xfd54_1c80,
];

fn crc32_ieee(bytes: &[u8]) -> u32 {
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

fn raw_16k_mapper(crc32: u32) -> Atari5200Mapper {
    if KNOWN_TWO_CHIP_16K_CRC32.contains(&crc32) {
        Atari5200Mapper::TwoChip16K
    } else {
        Atari5200Mapper::OneChip16K
    }
}

struct Atari5200Cartridge {
    rom: Vec<u8>,
    mapper: Atari5200Mapper,
    bounty_bank_4000: u8,
    bounty_bank_5000: u8,
    super_bank: u8,
}

impl Atari5200Cartridge {
    fn new(image: &[u8]) -> Result<Self, String> {
        let (mapper, rom, expected) = if image.len() >= 16 && &image[..4] == b"CART" {
            let cart_type = u32::from_be_bytes(image[4..8].try_into().unwrap());
            let (mapper, expected) = match cart_type {
                4 => (Atari5200Mapper::Linear32K, 0x8000),
                6 => (Atari5200Mapper::TwoChip16K, 0x4000),
                7 => (Atari5200Mapper::BountyBob40K, 0xa000),
                16 => (Atari5200Mapper::OneChip16K, 0x4000),
                19 => (Atari5200Mapper::Standard8K, 0x2000),
                20 => (Atari5200Mapper::Standard4K, 0x1000),
                71 => (Atari5200Mapper::SuperCart, 0x10000),
                72 => (Atari5200Mapper::SuperCart, 0x20000),
                73 => (Atari5200Mapper::SuperCart, 0x40000),
                74 => (Atari5200Mapper::SuperCart, 0x80000),
                159 => (Atari5200Mapper::BountyBob40KAlt, 0xa000),
                _ => return Err(format!("unsupported Atari 5200 CART type {cart_type}")),
            };
            (mapper, &image[16..], expected)
        } else {
            let mapper = match image.len() {
                0x1000 => Atari5200Mapper::Standard4K,
                0x2000 => Atari5200Mapper::Standard8K,
                0x4000 => raw_16k_mapper(crc32_ieee(image)),
                0x8000 => Atari5200Mapper::Linear32K,
                0xa000 => Atari5200Mapper::BountyBob40K,
                0x10000 | 0x20000 | 0x40000 | 0x80000 => Atari5200Mapper::SuperCart,
                _ => {
                    return Err(
                        "Atari 5200 cartridge must be a supported 4/8/16/32/40/64/128/256/512 KiB raw image or CART container"
                            .into(),
                    )
                }
            };
            (mapper, image, image.len())
        };

        if rom.len() != expected {
            return Err(format!(
                "Atari 5200 cartridge payload is {} bytes; mapper requires {expected} bytes",
                rom.len()
            ));
        }

        Ok(Self {
            rom: rom.to_vec(),
            mapper,
            bounty_bank_4000: 0,
            bounty_bank_5000: 0,
            super_bank: 0,
        })
    }

    fn reset(&mut self) {
        self.bounty_bank_4000 = 0;
        self.bounty_bank_5000 = 0;
        self.super_bank = 0;
    }

    fn select_bounty_bank(&mut self, address: u16) {
        match address {
            0x4ff6..=0x4ff9 => self.bounty_bank_4000 = (address - 0x4ff6) as u8,
            0x5ff6..=0x5ff9 => self.bounty_bank_5000 = (address - 0x5ff6) as u8,
            _ => {}
        }
    }

    fn select_super_bank(&mut self, address: u16) {
        if !(0xbfc0..=0xbfff).contains(&address) {
            return;
        }
        let mut bank = self.super_bank;
        match address & 0x30 {
            0x00 => bank = (bank & 0x03) | ((address as u8) & 0x0c),
            0x10 => bank = (bank & 0x0c) | (((address as u8) & 0x0c) >> 2),
            _ => bank = 0x0f,
        }
        let mask = (self.rom.len() / 0x8000).saturating_sub(1) as u8;
        self.super_bank = bank & mask;
    }

    fn read(&mut self, address: u16) -> u8 {
        if self.mapper == Atari5200Mapper::SuperCart {
            self.select_super_bank(address);
        }
        if matches!(
            self.mapper,
            Atari5200Mapper::BountyBob40K | Atari5200Mapper::BountyBob40KAlt
        ) {
            self.select_bounty_bank(address);
        }
        match self.mapper {
            Atari5200Mapper::Linear32K => self.rom[usize::from(address - 0x4000)],
            Atari5200Mapper::TwoChip16K => match address {
                0x4000..=0x7fff => self.rom[usize::from(address - 0x4000) & 0x1fff],
                0x8000..=0xbfff => self.rom[0x2000 + (usize::from(address - 0x8000) & 0x1fff)],
                _ => 0xff,
            },
            Atari5200Mapper::OneChip16K => {
                if address < 0x8000 {
                    0xff
                } else {
                    self.rom[usize::from(address - 0x8000) & 0x3fff]
                }
            }
            Atari5200Mapper::Standard8K => {
                if address < 0x8000 {
                    0xff
                } else {
                    self.rom[usize::from(address - 0x8000) & 0x1fff]
                }
            }
            Atari5200Mapper::Standard4K => {
                if address < 0x8000 {
                    0xff
                } else {
                    self.rom[usize::from(address - 0x8000) & 0x0fff]
                }
            }
            Atari5200Mapper::BountyBob40K => match address {
                0x4000..=0x4fff => {
                    let base = usize::from(self.bounty_bank_4000) * 0x1000;
                    self.rom[base + usize::from(address - 0x4000)]
                }
                0x5000..=0x5fff => {
                    let base = (4 + usize::from(self.bounty_bank_5000)) * 0x1000;
                    self.rom[base + usize::from(address - 0x5000)]
                }
                0x8000..=0xbfff => self.rom[0x8000 + (usize::from(address - 0x8000) & 0x1fff)],
                _ => 0xff,
            },
            Atari5200Mapper::BountyBob40KAlt => match address {
                0x4000..=0x4fff => {
                    let base = 0x2000 + usize::from(self.bounty_bank_4000) * 0x1000;
                    self.rom[base + usize::from(address - 0x4000)]
                }
                0x5000..=0x5fff => {
                    let base = 0x6000 + usize::from(self.bounty_bank_5000) * 0x1000;
                    self.rom[base + usize::from(address - 0x5000)]
                }
                0x8000..=0xbfff => self.rom[usize::from(address - 0x8000) & 0x1fff],
                _ => 0xff,
            },
            Atari5200Mapper::SuperCart => {
                if !(0x4000..=0xbfff).contains(&address) {
                    0xff
                } else {
                    self.rom[usize::from(self.super_bank) * 0x8000 + usize::from(address - 0x4000)]
                }
            }
        }
    }

    fn write(&mut self, address: u16, _value: u8) {
        if self.mapper == Atari5200Mapper::SuperCart {
            self.select_super_bank(address);
        }
        if matches!(
            self.mapper,
            Atari5200Mapper::BountyBob40K | Atari5200Mapper::BountyBob40KAlt
        ) {
            self.select_bounty_bank(address);
        }
    }

    fn save(&self, out: &mut StateWriter) {
        out.u8(self.bounty_bank_4000);
        out.u8(self.bounty_bank_5000);
        out.u8(self.super_bank);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.bounty_bank_4000 = input.u8()?.min(3);
        self.bounty_bank_5000 = input.u8()?.min(3);
        let super_bank = input.u8()?;
        if self.mapper == Atari5200Mapper::SuperCart
            && usize::from(super_bank) >= self.rom.len() / 0x8000
        {
            return Err("Atari 5200 state contains an invalid SuperCart bank".into());
        }
        self.super_bank = super_bank;
        Ok(())
    }
}

#[derive(Clone)]
struct Gtia {
    write_regs: [u8; 32],
    triggers: [bool; 4],
    console: u8,
}

impl Default for Gtia {
    fn default() -> Self {
        Self {
            write_regs: [0; 32],
            triggers: [false; 4],
            console: 0x07,
        }
    }
}

impl Gtia {
    fn reset(&mut self) {
        *self = Self::default();
    }

    fn set_inputs(&mut self, input: &InputState) {
        for player in 0..4 {
            self.triggers[player] = input.buttons[player] & FACE_SOUTH != 0;
        }
        self.console = 0x07;
        if input.buttons[0] & START != 0 {
            self.console &= !0x01;
        }
        if input.buttons[0] & SELECT != 0 {
            self.console &= !0x02;
        }
    }

    fn read(&self, address: u16) -> u8 {
        match address & 0x1f {
            0x10..=0x13 => u8::from(!self.triggers[(address as usize) & 3]),
            0x1f => self.console,
            _ => 0,
        }
    }

    fn write(&mut self, address: u16, value: u8) {
        self.write_regs[(address as usize) & 0x1f] = value;
    }

    fn playfield_color(&self, index: usize) -> u8 {
        match index & 3 {
            0 => self.write_regs[0x16],
            1 => self.write_regs[0x17],
            2 => self.write_regs[0x18],
            _ => self.write_regs[0x19],
        }
    }

    fn background_color(&self) -> u8 {
        self.write_regs[0x1a]
    }

    fn save(&self, out: &mut StateWriter) {
        out.blob(&self.write_regs);
        for trigger in self.triggers {
            out.u8(trigger as u8);
        }
        out.u8(self.console);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let regs = input.blob()?;
        if regs.len() != self.write_regs.len() {
            return Err("GTIA state has invalid register length".into());
        }
        self.write_regs.copy_from_slice(regs);
        for trigger in &mut self.triggers {
            *trigger = input.u8()? != 0;
        }
        self.console = input.u8()?;
        Ok(())
    }
}

struct Antic {
    regs: [u8; 16],
    scanline: u16,
    cycle_in_line: u16,
    frame: u64,
    nmi_status: u8,
    nmi_pending: bool,
    wsync: bool,
    video: VideoBuffer,
}

impl Default for Antic {
    fn default() -> Self {
        Self {
            regs: [0; 16],
            scanline: 0,
            cycle_in_line: 0,
            frame: 0,
            nmi_status: 0,
            nmi_pending: false,
            wsync: false,
            video: VideoBuffer::new(WIDTH, HEIGHT),
        }
    }
}

impl Antic {
    fn reset(&mut self) {
        *self = Self::default();
    }

    fn read(&self, address: u16) -> u8 {
        match address & 0x0f {
            0x0b => (self.scanline / 2) as u8,
            0x0f => self.nmi_status,
            _ => 0xff,
        }
    }

    fn write(&mut self, address: u16, value: u8) {
        let register = (address & 0x0f) as usize;
        match register {
            0x0a => self.wsync = true,
            0x0f => {
                self.nmi_status = 0;
                self.nmi_pending = false;
            }
            _ => self.regs[register] = value,
        }
    }

    fn take_wsync(&mut self) -> bool {
        std::mem::take(&mut self.wsync)
    }

    fn cycles_until_line_end(&self) -> u32 {
        u32::from(
            CPU_CYCLES_PER_LINE
                .saturating_sub(self.cycle_in_line)
                .max(1),
        )
    }

    fn take_nmi(&mut self) -> bool {
        std::mem::take(&mut self.nmi_pending)
    }

    fn tick(&mut self, cycles: u32) -> bool {
        let mut frame_ready = false;
        let mut total = u32::from(self.cycle_in_line) + cycles;
        while total >= u32::from(CPU_CYCLES_PER_LINE) {
            total -= u32::from(CPU_CYCLES_PER_LINE);
            self.scanline = self.scanline.wrapping_add(1);
            if self.scanline == HEIGHT as u16 {
                frame_ready = true;
            }
            if self.scanline == 248 {
                self.nmi_status |= 0x40;
                if self.regs[0x0e] & 0x40 != 0 {
                    self.nmi_pending = true;
                }
            }
            if self.scanline >= SCANLINES {
                self.scanline = 0;
                self.frame = self.frame.wrapping_add(1);
            }
        }
        self.cycle_in_line = total as u16;
        frame_ready
    }

    fn dma_read(ram: &[u8; 0x4000], address: u16) -> u8 {
        ram[(address as usize) & 0x3fff]
    }

    fn mode_height(mode: u8) -> usize {
        match mode {
            2 | 4 | 6 => 8,
            3 => 10,
            5 | 7 => 16,
            8 => 8,
            9 | 10 => 4,
            11 | 13 => 2,
            _ => 1,
        }
    }

    fn mode_columns(mode: u8) -> usize {
        match mode {
            6 | 7 => 20,
            _ => 40,
        }
    }

    fn render_text_row(
        &mut self,
        ram: &[u8; 0x4000],
        gtia: &Gtia,
        mode: u8,
        screen: u16,
        y_start: usize,
    ) {
        let columns = Self::mode_columns(mode);
        let height = Self::mode_height(mode);
        let scale = WIDTH as usize / (columns * 8);
        let charset = u16::from(self.regs[9]) << 8;
        for row in 0..height.min(HEIGHT as usize - y_start) {
            let glyph_row = row & 7;
            for column in 0..columns {
                let character = Self::dma_read(ram, screen.wrapping_add(column as u16));
                let glyph_addr = charset
                    .wrapping_add(u16::from(character & 0x7f) * 8)
                    .wrapping_add(glyph_row as u16);
                let glyph = Self::dma_read(ram, glyph_addr);
                let color_index = if mode >= 4 {
                    usize::from(character >> 6)
                } else {
                    2
                };
                let foreground = atari_color(gtia.playfield_color(color_index));
                let background = atari_color(gtia.background_color());
                for bit in 0..8usize {
                    let rgba = if glyph & (0x80 >> bit) != 0 {
                        foreground
                    } else {
                        background
                    };
                    let x0 = (column * 8 + bit) * scale;
                    for dx in 0..scale {
                        let x = x0 + dx;
                        if x < WIDTH as usize {
                            let offset = ((y_start + row) * WIDTH as usize + x) * 4;
                            self.video.pixels_mut()[offset..offset + 4].copy_from_slice(&rgba);
                        }
                    }
                }
            }
        }
    }

    fn render_bitmap_row(
        &mut self,
        ram: &[u8; 0x4000],
        gtia: &Gtia,
        mode: u8,
        screen: u16,
        y_start: usize,
    ) {
        let height = Self::mode_height(mode).min(HEIGHT as usize - y_start);
        let high_resolution = mode == 15;
        for row in 0..height {
            for byte_index in 0..40usize {
                let value = Self::dma_read(ram, screen.wrapping_add(byte_index as u16));
                if high_resolution {
                    for bit in 0..8usize {
                        let set = value & (0x80 >> bit) != 0;
                        let rgba = if set {
                            atari_color(gtia.playfield_color(2))
                        } else {
                            atari_color(gtia.background_color())
                        };
                        let x = byte_index * 8 + bit;
                        let offset = ((y_start + row) * WIDTH as usize + x) * 4;
                        self.video.pixels_mut()[offset..offset + 4].copy_from_slice(&rgba);
                    }
                } else {
                    for pixel in 0..4usize {
                        let shift = 6 - pixel * 2;
                        let color = usize::from((value >> shift) & 3);
                        let rgba = if color == 0 {
                            atari_color(gtia.background_color())
                        } else {
                            atari_color(gtia.playfield_color(color - 1))
                        };
                        let x0 = (byte_index * 4 + pixel) * 2;
                        for dx in 0..2usize {
                            let x = x0 + dx;
                            let offset = ((y_start + row) * WIDTH as usize + x) * 4;
                            self.video.pixels_mut()[offset..offset + 4].copy_from_slice(&rgba);
                        }
                    }
                }
            }
        }
    }

    fn render_frame(&mut self, ram: &[u8; 0x4000], gtia: &Gtia) {
        self.video.clear(atari_color(gtia.background_color()));
        if self.regs[0] & 0x20 == 0 {
            return;
        }
        let mut display = u16::from(self.regs[2]) | (u16::from(self.regs[3]) << 8);
        let mut screen = 0u16;
        let mut y = 0usize;
        let mut guard = 0usize;
        while y < HEIGHT as usize && guard < 1024 {
            guard += 1;
            let instruction = Self::dma_read(ram, display);
            display = display.wrapping_add(1);
            let mode = instruction & 0x0f;
            if mode == 0 {
                let lines = usize::from((instruction >> 4) & 7) + 1;
                y = (y + lines).min(HEIGHT as usize);
                continue;
            }
            if mode == 1 {
                let lo = Self::dma_read(ram, display);
                let hi = Self::dma_read(ram, display.wrapping_add(1));
                display = u16::from_le_bytes([lo, hi]);
                if instruction & 0x40 != 0 {
                    break;
                }
                continue;
            }
            if instruction & 0x40 != 0 {
                let lo = Self::dma_read(ram, display);
                let hi = Self::dma_read(ram, display.wrapping_add(1));
                screen = u16::from_le_bytes([lo, hi]);
                display = display.wrapping_add(2);
            }
            if (2..=7).contains(&mode) {
                self.render_text_row(ram, gtia, mode, screen, y);
            } else {
                self.render_bitmap_row(ram, gtia, mode, screen, y);
            }
            let bytes = Self::mode_columns(mode) as u16;
            screen = screen.wrapping_add(bytes);
            y = (y + Self::mode_height(mode)).min(HEIGHT as usize);
        }
    }

    fn save(&self, out: &mut StateWriter) {
        out.blob(&self.regs);
        out.u16(self.scanline);
        out.u16(self.cycle_in_line);
        out.u64(self.frame);
        out.u8(self.nmi_status);
        out.u8(self.nmi_pending as u8);
        out.u8(self.wsync as u8);
        out.blob(self.video.pixels());
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let regs = input.blob()?;
        if regs.len() != self.regs.len() {
            return Err("ANTIC state has invalid register length".into());
        }
        self.regs.copy_from_slice(regs);
        self.scanline = input.u16()? % SCANLINES;
        self.cycle_in_line = input.u16()? % CPU_CYCLES_PER_LINE;
        self.frame = input.u64()?;
        self.nmi_status = input.u8()?;
        self.nmi_pending = input.u8()? != 0;
        self.wsync = input.u8()? != 0;
        let video = input.blob()?;
        if video.len() != self.video.pixels().len() {
            return Err("ANTIC state has invalid framebuffer length".into());
        }
        self.video.pixels_mut().copy_from_slice(video);
        Ok(())
    }
}

fn atari_color(value: u8) -> [u8; 4] {
    let hue = f32::from(value >> 4) / 16.0;
    let luma = f32::from(value & 0x0e) / 14.0;
    let saturation = if value & 0xf0 == 0 { 0.0 } else { 0.72 };
    let h = hue * 6.0;
    let c = luma * saturation;
    let x = c * (1.0 - ((h % 2.0) - 1.0).abs());
    let (r1, g1, b1) = match h as u8 % 6 {
        0 => (c, x, 0.0),
        1 => (x, c, 0.0),
        2 => (0.0, c, x),
        3 => (0.0, x, c),
        4 => (x, 0.0, c),
        _ => (c, 0.0, x),
    };
    let m = luma - c;
    [
        ((r1 + m).clamp(0.0, 1.0) * 255.0) as u8,
        ((g1 + m).clamp(0.0, 1.0) * 255.0) as u8,
        ((b1 + m).clamp(0.0, 1.0) * 255.0) as u8,
        255,
    ]
}

struct Atari5200Bus {
    ram: [u8; 0x4000],
    cartridge: Atari5200Cartridge,
    bios: [u8; 0x0800],
    gtia: Gtia,
    antic: Antic,
    pokey: Pokey,
}

impl Atari5200Bus {
    fn new(cartridge: Atari5200Cartridge, bios: &[u8]) -> Result<Self, String> {
        if bios.len() != 0x0800 {
            return Err(format!(
                "Atari 5200 BIOS must be exactly 2048 bytes, got {}",
                bios.len()
            ));
        }
        let mut bios_bytes = [0u8; 0x0800];
        bios_bytes.copy_from_slice(bios);
        Ok(Self {
            ram: [0; 0x4000],
            cartridge,
            bios: bios_bytes,
            gtia: Gtia::default(),
            antic: Antic::default(),
            pokey: Pokey::new(CPU_HZ),
        })
    }

    fn reset_devices(&mut self) {
        self.cartridge.reset();
        self.gtia.reset();
        self.antic.reset();
        self.pokey.reset();
    }

    fn tick(&mut self, cycles: u32) {
        self.pokey.tick(cycles);
        if self.antic.tick(cycles) {
            self.antic.render_frame(&self.ram, &self.gtia);
        }
    }

    fn set_inputs(&mut self, input: &InputState) {
        self.gtia.set_inputs(input);
        let pots_enabled = self.gtia.write_regs[0x1f] & 0x04 != 0;
        for player in 0..4usize {
            let mut x = input.axes[player][AXIS_LEFT_X];
            let mut y = input.axes[player][AXIS_LEFT_Y];
            let buttons = input.buttons[player];
            if buttons & LEFT != 0 {
                x = i16::MIN;
            } else if buttons & RIGHT != 0 {
                x = i16::MAX;
            }
            if buttons & UP != 0 {
                y = i16::MIN;
            } else if buttons & DOWN != 0 {
                y = i16::MAX;
            }
            let (x_pot, y_pot) = if pots_enabled {
                (axis_to_pot(x), axis_to_pot(y))
            } else {
                (228, 228)
            };
            self.pokey.set_pot(player * 2, x_pot);
            self.pokey.set_pot(player * 2 + 1, y_pot);
        }

        let selected = usize::from(self.gtia.write_regs[0x1f] & 0x03);
        let buttons = input.buttons[selected];
        let keypad = [
            (KEYPAD_1, 0x1e),
            (KEYPAD_2, 0x1c),
            (KEYPAD_3, 0x1a),
            (START, 0x18),
            (KEYPAD_4, 0x16),
            (KEYPAD_5, 0x14),
            (KEYPAD_6, 0x12),
            (PAUSE | SELECT, 0x10),
            (KEYPAD_7, 0x0e),
            (KEYPAD_8, 0x0c),
            (KEYPAD_9, 0x0a),
            (RESET, 0x08),
            (KEYPAD_STAR, 0x06),
            (KEYPAD_0, 0x04),
            (KEYPAD_HASH, 0x02),
        ]
        .into_iter()
        .find_map(|(button, code)| (buttons & button != 0).then_some(code))
        .unwrap_or(0xff);
        let top_button = buttons & FACE_EAST != 0;
        self.pokey.set_keyboard_state(keypad, top_button);
    }

    fn save(&self, out: &mut StateWriter) {
        out.blob(&self.ram);
        self.cartridge.save(out);
        self.gtia.save(out);
        self.antic.save(out);
        self.pokey.save(out);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let ram = input.blob()?;
        if ram.len() != self.ram.len() {
            return Err("Atari 5200 state has invalid RAM length".into());
        }
        self.ram.copy_from_slice(ram);
        self.cartridge.load(input)?;
        self.gtia.load(input)?;
        self.antic.load(input)?;
        self.pokey.load(input)
    }
}

fn axis_to_pot(value: i16) -> u8 {
    let normalized = i32::from(value) - i32::from(i16::MIN);
    ((normalized * 228) / 65_535).clamp(0, 228) as u8
}

impl Bus8 for Atari5200Bus {
    fn read8(&mut self, address: u16) -> u8 {
        match address {
            0x0000..=0x3fff => self.ram[address as usize],
            0x4000..=0xbfff => self.cartridge.read(address),
            0xc000..=0xc0ff => self.gtia.read(address),
            0xd400..=0xd4ff => self.antic.read(address),
            0xe800..=0xe8ff => self.pokey.read(address),
            0xf800..=0xffff => self.bios[(address as usize) & 0x07ff],
            _ => 0xff,
        }
    }

    fn write8(&mut self, address: u16, value: u8) {
        match address {
            0x0000..=0x3fff => self.ram[address as usize] = value,
            0x4000..=0xbfff => self.cartridge.write(address, value),
            0xc000..=0xc0ff => self.gtia.write(address, value),
            0xd400..=0xd4ff => self.antic.write(address, value),
            0xe800..=0xe8ff => self.pokey.write(address, value),
            _ => {}
        }
    }
}

pub struct Atari5200Machine {
    cpu: Mos6502,
    bus: Atari5200Bus,
    audio: AudioBuffer,
    powered: bool,
}

impl Atari5200Machine {
    pub fn new(rom: &[u8], bios: &[u8]) -> Result<Self, String> {
        let cartridge = Atari5200Cartridge::new(rom)?;
        let mut bus = Atari5200Bus::new(cartridge, bios)?;
        let mut cpu = Mos6502::default();
        cpu.reset(&mut bus);
        Ok(Self {
            cpu,
            bus,
            audio: AudioBuffer::new(48_000, 2),
            powered: true,
        })
    }

    fn clock_instruction(&mut self) -> u32 {
        let cycles = self.cpu.step(&mut self.bus);
        if cycles == 0 {
            return 0;
        }
        self.bus.tick(cycles);
        if self.bus.antic.take_wsync() {
            let stall = self.bus.antic.cycles_until_line_end();
            self.cpu.cycles = self.cpu.cycles.saturating_add(u64::from(stall));
            self.bus.tick(stall);
        }
        if self.bus.antic.take_nmi() {
            let before = self.cpu.cycles;
            self.cpu.nmi(&mut self.bus);
            self.bus.tick((self.cpu.cycles - before) as u32);
        }
        if self.bus.pokey.irq_pending() {
            let before = self.cpu.cycles;
            self.cpu.irq(&mut self.bus);
            self.bus.tick((self.cpu.cycles - before) as u32);
        }
        cycles
    }

    fn flush_audio(&mut self) {
        self.audio.begin_frame();
        let samples = self.bus.pokey.take_samples();
        for sample in samples {
            self.audio.push_stereo(sample, sample);
        }
        if self.audio.samples().is_empty() {
            self.audio.push_stereo(0.0, 0.0);
        }
    }

    fn save_cpu(&self, out: &mut StateWriter) {
        out.u16(self.cpu.pc);
        out.u8(self.cpu.sp);
        out.u8(self.cpu.a);
        out.u8(self.cpu.x);
        out.u8(self.cpu.y);
        out.u8(self.cpu.p);
        out.u64(self.cpu.cycles);
        out.u8(self.cpu.stopped as u8);
    }

    fn load_cpu(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.cpu.pc = input.u16()?;
        self.cpu.sp = input.u8()?;
        self.cpu.a = input.u8()?;
        self.cpu.x = input.u8()?;
        self.cpu.y = input.u8()?;
        self.cpu.p = input.u8()?;
        self.cpu.cycles = input.u64()?;
        self.cpu.stopped = input.u8()? != 0;
        Ok(())
    }
}

impl Machine for Atari5200Machine {
    fn platform(&self) -> PlatformId {
        PlatformId::Atari5200
    }

    fn reset(&mut self) {
        self.bus.reset_devices();
        self.cpu.reset(&mut self.bus);
        self.powered = true;
        self.audio.begin_frame();
    }

    fn run_frame(&mut self, input: &InputState) {
        self.bus.set_inputs(input);
        let target = self.bus.antic.frame.wrapping_add(1);
        let deadline = self
            .cpu
            .cycles
            .saturating_add((CPU_HZ as f64 / FRAME_RATE * 2.0) as u64);
        while self.bus.antic.frame != target && self.cpu.cycles < deadline {
            if self.clock_instruction() == 0 {
                self.powered = false;
                break;
            }
        }
        self.flush_audio();
    }

    fn frame_rate(&self) -> f64 {
        FRAME_RATE
    }

    fn video(&self) -> &VideoBuffer {
        &self.bus.antic.video
    }

    fn audio(&self) -> &AudioBuffer {
        &self.audio
    }

    fn save_state(&self) -> Result<Vec<u8>, String> {
        let mut out = StateWriter::new(PlatformId::Atari5200, STATE_VERSION);
        self.save_cpu(&mut out);
        self.bus.save(&mut out);
        out.u8(self.powered as u8);
        Ok(out.finish())
    }

    fn load_state(&mut self, bytes: &[u8]) -> Result<(), String> {
        let mut input = StateReader::new(bytes, PlatformId::Atari5200, STATE_VERSION)?;
        self.load_cpu(&mut input)?;
        self.bus.load(&mut input)?;
        self.powered = input.u8()? != 0;
        self.audio.begin_frame();
        input.finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn synthetic_bios() -> Vec<u8> {
        let mut bios = vec![0xea; 0x800];
        let program = [
            0x78, 0xd8, 0xa9, 0x22, 0x8d, 0x16, 0xc0, 0xa9, 0x84, 0x8d, 0x1a, 0xc0, 0xa9, 0x20,
            0x8d, 0x00, 0xd4, 0xa9, 0x00, 0x8d, 0x02, 0xd4, 0xa9, 0x20, 0x8d, 0x03, 0xd4, 0xa9,
            0x40, 0x8d, 0x0e, 0xd4, 0x4c, 0x00, 0x40,
        ];
        bios[..program.len()].copy_from_slice(&program);
        bios[0x7fa..0x800].copy_from_slice(&[0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8]);
        bios
    }

    fn synthetic_cart() -> Vec<u8> {
        let mut cart = vec![0xea; 0x8000];
        let code = [
            0xa9, 0x08, 0x8d, 0x00, 0xe8, 0xa9, 0xaf, 0x8d, 0x01, 0xe8, 0x4c, 0x0a, 0x40,
        ];
        cart[..code.len()].copy_from_slice(&code);
        cart
    }

    fn cart_container(cart_type: u32, payload: &[u8]) -> Vec<u8> {
        let mut image = vec![0; 16 + payload.len()];
        image[..4].copy_from_slice(b"CART");
        image[4..8].copy_from_slice(&cart_type.to_be_bytes());
        image[16..].copy_from_slice(payload);
        image
    }

    fn install_display_list(machine: &mut Atari5200Machine) {
        let list = 0x2000usize;
        let screen = 0x2100usize;
        machine.bus.ram[list] = 0x4f;
        machine.bus.ram[list + 1] = screen as u8;
        machine.bus.ram[list + 2] = (screen >> 8) as u8;
        for row in 1..HEIGHT as usize {
            machine.bus.ram[list + 2 + row] = 0x0f;
        }
        let end = list + 2 + HEIGHT as usize;
        machine.bus.ram[end] = 0x41;
        machine.bus.ram[end + 1] = list as u8;
        machine.bus.ram[end + 2] = (list >> 8) as u8;
        for row in 0..HEIGHT as usize {
            for byte in 0..40usize {
                machine.bus.ram[screen + row * 40 + byte] =
                    if (row + byte) & 1 == 0 { 0xaa } else { 0x55 };
            }
        }
    }

    #[test]
    fn raw_16k_mapper_uses_known_commercial_crc_profiles() {
        assert_eq!(crc32_ieee(b"123456789"), 0xcbf4_3926);
        assert_eq!(raw_16k_mapper(0x536a_70fe), Atari5200Mapper::TwoChip16K);
        assert_eq!(raw_16k_mapper(0xf43e_7cd0), Atari5200Mapper::OneChip16K);
    }

    #[test]
    fn standard_cartridge_sizes_follow_5200_mirroring_rules() {
        let mut four = vec![0; 0x1000];
        four[0] = 0x41;
        four[0x0fff] = 0x4f;
        let mut cart = Atari5200Cartridge::new(&four).unwrap();
        assert_eq!(cart.read(0x4000), 0xff);
        assert_eq!(cart.read(0x8000), 0x41);
        assert_eq!(cart.read(0x9000), 0x41);
        assert_eq!(cart.read(0xa000), 0x41);
        assert_eq!(cart.read(0xbfff), 0x4f);

        let mut eight = vec![0; 0x2000];
        eight[0] = 0x81;
        eight[0x1fff] = 0x8f;
        let mut cart = Atari5200Cartridge::new(&eight).unwrap();
        assert_eq!(cart.read(0x4000), 0xff);
        assert_eq!(cart.read(0x8000), 0x81);
        assert_eq!(cart.read(0xa000), 0x81);
        assert_eq!(cart.read(0xbfff), 0x8f);

        let mut one_chip = vec![0; 0x4000];
        one_chip[0] = 0x16;
        one_chip[0x3fff] = 0x1f;
        let mut cart = Atari5200Cartridge::new(&one_chip).unwrap();
        assert_eq!(cart.read(0x4000), 0xff);
        assert_eq!(cart.read(0x8000), 0x16);
        assert_eq!(cart.read(0xbfff), 0x1f);
    }

    #[test]
    fn cart_type_six_maps_two_chip_16k_halves_into_mirrored_windows() {
        let mut payload = vec![0x11; 0x4000];
        payload[0x2000..].fill(0x22);
        let image = cart_container(6, &payload);
        let mut cart = Atari5200Cartridge::new(&image).unwrap();

        assert_eq!(cart.mapper, Atari5200Mapper::TwoChip16K);
        assert_eq!(cart.read(0x4000), 0x11);
        assert_eq!(cart.read(0x6000), 0x11);
        assert_eq!(cart.read(0x8000), 0x22);
        assert_eq!(cart.read(0xa000), 0x22);
    }

    #[test]
    fn bounty_bob_40k_accesses_switch_both_windows_and_state_round_trips() {
        let mut rom = vec![0; 0xa000];
        for bank in 0..8usize {
            rom[bank * 0x1000..(bank + 1) * 0x1000].fill(bank as u8);
        }
        rom[0x8000..0x9000].fill(0x80);
        rom[0x9000..0xa000].fill(0x81);

        let mut cart = Atari5200Cartridge::new(&rom).unwrap();
        assert_eq!(cart.mapper, Atari5200Mapper::BountyBob40K);
        assert_eq!(cart.read(0x4000), 0);
        assert_eq!(cart.read(0x5000), 4);

        assert_eq!(cart.read(0x4ff8), 2);
        assert_eq!(cart.read(0x4000), 2);
        cart.write(0x5ff9, 0);
        assert_eq!(cart.read(0x5000), 7);

        assert_eq!(cart.read(0x8000), 0x80);
        assert_eq!(cart.read(0x9000), 0x81);
        assert_eq!(cart.read(0xa000), 0x80);
        assert_eq!(cart.read(0xb000), 0x81);

        let mut writer = StateWriter::new(PlatformId::Atari5200, STATE_VERSION);
        cart.save(&mut writer);
        let state = writer.finish();
        let mut restored = Atari5200Cartridge::new(&rom).unwrap();
        let mut reader = StateReader::new(&state, PlatformId::Atari5200, STATE_VERSION).unwrap();
        restored.load(&mut reader).unwrap();
        reader.finish().unwrap();
        assert_eq!(restored.read(0x4000), 2);
        assert_eq!(restored.read(0x5000), 7);

        restored.reset();
        assert_eq!(restored.read(0x4000), 0);
        assert_eq!(restored.read(0x5000), 4);
    }

    #[test]
    fn cart_type_159_uses_alternate_bounty_bob_rom_wiring() {
        let mut payload = vec![0; 0xa000];
        for bank in 0..10usize {
            payload[bank * 0x1000..(bank + 1) * 0x1000].fill(bank as u8);
        }
        let image = cart_container(159, &payload);
        let mut cart = Atari5200Cartridge::new(&image).unwrap();

        assert_eq!(cart.mapper, Atari5200Mapper::BountyBob40KAlt);
        assert_eq!(cart.read(0x4000), 2);
        assert_eq!(cart.read(0x5000), 6);
        assert_eq!(cart.read(0x8000), 0);
        assert_eq!(cart.read(0x9000), 1);
        assert_eq!(cart.read(0xa000), 0);
        assert_eq!(cart.read(0xb000), 1);

        assert_eq!(cart.read(0x4ff9), 5);
        cart.write(0x5ff8, 0);
        assert_eq!(cart.read(0x4000), 5);
        assert_eq!(cart.read(0x5000), 8);
    }

    #[test]
    fn supercart_types_71_through_74_bank_full_thirty_two_kib_window() {
        for (cart_type, banks) in [(71u32, 2usize), (72, 4), (73, 8), (74, 16)] {
            let mut payload = vec![0; banks * 0x8000];
            for bank in 0..banks {
                payload[bank * 0x8000..(bank + 1) * 0x8000].fill(bank as u8);
            }
            let image = cart_container(cart_type, &payload);
            let mut cart = Atari5200Cartridge::new(&image).unwrap();
            assert_eq!(cart.mapper, Atari5200Mapper::SuperCart);
            assert_eq!(cart.read(0x4000), 0);
            assert_eq!(cart.read(0xbfbf), 0);

            let target = banks - 1;
            let high = 0xbfc0 | ((target as u16) & 0x0c);
            let low = 0xbfd0 | (((target as u16) & 0x03) << 2);
            let _ = cart.read(high);
            cart.write(low, 0);
            assert_eq!(usize::from(cart.super_bank), target);
            assert_eq!(cart.read(0x4000), target as u8);
            assert_eq!(cart.read(0xbfff), target as u8);

            cart.reset();
            assert_eq!(cart.read(0x4000), 0);
            assert_eq!(cart.read(0xbfe0), target as u8);
            assert_eq!(usize::from(cart.super_bank), target);

            let mut writer = StateWriter::new(PlatformId::Atari5200, STATE_VERSION);
            cart.save(&mut writer);
            let state = writer.finish();
            cart.reset();
            let mut reader =
                StateReader::new(&state, PlatformId::Atari5200, STATE_VERSION).unwrap();
            cart.load(&mut reader).unwrap();
            reader.finish().unwrap();
            assert_eq!(usize::from(cart.super_bank), target);
            assert_eq!(cart.read(0x4000), target as u8);
        }
    }

    #[test]
    fn raw_supercart_sizes_select_bank_switched_mapper() {
        for banks in [2usize, 4, 8, 16] {
            let mut payload = vec![0; banks * 0x8000];
            for bank in 0..banks {
                payload[bank * 0x8000..(bank + 1) * 0x8000].fill((0x40 + bank) as u8);
            }
            let mut cart = Atari5200Cartridge::new(&payload).unwrap();
            assert_eq!(cart.mapper, Atari5200Mapper::SuperCart);
            assert_eq!(cart.read(0x4000), 0x40);

            let target = banks - 1;
            let high = 0xbfc0 | ((target as u16) & 0x0c);
            let low = 0xbfd0 | (((target as u16) & 0x03) << 2);
            cart.write(high, 0);
            cart.write(low, 0);
            assert_eq!(usize::from(cart.super_bank), target);
            assert_eq!(cart.read(0x4000), (0x40 + target) as u8);
        }
    }

    #[test]
    fn supercart_container_sizes_are_type_specific() {
        let payload = vec![0; 0x20000];
        let error = match Atari5200Cartridge::new(&cart_container(71, &payload)) {
            Ok(_) => panic!("type 71 must reject a 128 KiB payload"),
            Err(error) => error,
        };
        assert!(error.contains("mapper requires 65536 bytes"));
    }

    #[test]
    fn synthetic_machine_runs_cpu_antic_gtia_and_pokey() {
        let mut machine = Atari5200Machine::new(&synthetic_cart(), &synthetic_bios()).unwrap();
        install_display_list(&mut machine);
        machine.run_frame(&InputState::default());
        assert_eq!(machine.video().width(), WIDTH);
        assert_eq!(machine.video().height(), HEIGHT);
        assert!(machine.video().pixels().iter().any(|&byte| byte != 0));
        assert!(machine.audio().samples().iter().any(|&sample| sample > 0.0));
        assert!(machine.powered);
    }

    #[test]
    fn analog_axes_feed_pokey_pot_inputs() {
        let mut machine = Atari5200Machine::new(&synthetic_cart(), &synthetic_bios()).unwrap();
        let mut input = InputState::default();
        input.axes[0][AXIS_LEFT_X] = i16::MIN;
        input.axes[0][AXIS_LEFT_Y] = i16::MAX;
        machine.bus.gtia.write_regs[0x1f] = 0x04;
        machine.bus.set_inputs(&input);
        assert_eq!(machine.bus.pokey.read(0), 0);
        assert_eq!(machine.bus.pokey.read(1), 228);

        machine.bus.gtia.write_regs[0x1f] = 0;
        machine.bus.set_inputs(&input);
        assert_eq!(machine.bus.pokey.read(0), 228);
        assert_eq!(machine.bus.pokey.read(1), 228);
    }

    #[test]
    fn keypad_reports_all_5200_codes_for_selected_controller() {
        let mut machine = Atari5200Machine::new(&synthetic_cart(), &synthetic_bios()).unwrap();
        let cases = [
            (KEYPAD_1, 0x1e),
            (KEYPAD_2, 0x1c),
            (KEYPAD_3, 0x1a),
            (START, 0x18),
            (KEYPAD_4, 0x16),
            (KEYPAD_5, 0x14),
            (KEYPAD_6, 0x12),
            (PAUSE, 0x10),
            (KEYPAD_7, 0x0e),
            (KEYPAD_8, 0x0c),
            (KEYPAD_9, 0x0a),
            (RESET, 0x08),
            (KEYPAD_STAR, 0x06),
            (KEYPAD_0, 0x04),
            (KEYPAD_HASH, 0x02),
        ];
        for (button, code) in cases {
            let mut input = InputState::default();
            input.buttons[0] = button;
            machine.bus.gtia.write_regs[0x1f] = 0;
            machine.bus.set_inputs(&input);
            assert_eq!(machine.bus.pokey.read(0x09), code);
        }

        let mut input = InputState::default();
        input.buttons[0] = KEYPAD_1;
        input.buttons[2] = KEYPAD_7;
        machine.bus.gtia.write_regs[0x1f] = 2;
        machine.bus.set_inputs(&input);
        assert_eq!(machine.bus.pokey.read(0x09), 0x0e);

        let mut input = InputState::default();
        input.buttons[0] = SELECT;
        machine.bus.gtia.write_regs[0x1f] = 0;
        machine.bus.set_inputs(&input);
        assert_eq!(machine.bus.pokey.read(0x09), 0x10);
    }

    #[test]
    fn selected_top_trigger_drives_pokey_shift_control_and_break_irq() {
        let mut machine = Atari5200Machine::new(&synthetic_cart(), &synthetic_bios()).unwrap();
        machine.bus.pokey.write(0x0e, 0x80);
        machine.bus.gtia.write_regs[0x1f] = 0;

        let mut input = InputState::default();
        input.buttons[0] = KEYPAD_1 | FACE_EAST;
        machine.bus.set_inputs(&input);

        assert_eq!(machine.bus.pokey.read(0x09), 0xde);
        assert_eq!(machine.bus.pokey.read(0x0f) & 0x08, 0);
        assert_eq!(machine.bus.pokey.read(0x0e) & 0x80, 0);
        assert!(machine.bus.pokey.irq_pending());

        input.buttons[0] = 0;
        machine.bus.set_inputs(&input);
        assert_eq!(machine.bus.pokey.read(0x0f) & 0x08, 0x08);
    }

    #[test]
    fn save_state_round_trip_restores_cpu_and_video() {
        let mut machine = Atari5200Machine::new(&synthetic_cart(), &synthetic_bios()).unwrap();
        install_display_list(&mut machine);
        machine.run_frame(&InputState::default());
        let state = machine.save_state().unwrap();
        let pc = machine.cpu.pc;
        let frame = machine.bus.antic.frame;
        let pixel = machine.video().pixels()[0];
        machine.cpu.pc = 0x1234;
        machine.bus.antic.video.pixels_mut()[0] ^= 0xff;
        machine.load_state(&state).unwrap();
        assert_eq!(machine.cpu.pc, pc);
        assert_eq!(machine.bus.antic.frame, frame);
        assert_eq!(machine.video().pixels()[0], pixel);
    }
}
