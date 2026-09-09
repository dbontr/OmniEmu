use super::pokey::Pokey;
use crate::bus::Bus8;
use crate::cpu6502::Mos6502;
use crate::input::{DOWN, FACE_SOUTH, LEFT, RIGHT, SELECT, START, UP};
use crate::kernel::{AudioBuffer, InputState, VideoBuffer};
use crate::machine::Machine;
use crate::platform::PlatformId;
use crate::state::{StateReader, StateWriter};

const WIDTH: u32 = 320;
const HEIGHT: u32 = 240;
const VISIBLE_TOP: u16 = 16;
const VISIBLE_LINES: u16 = 240;
const SCANLINES: u16 = 262;
const CPU_CYCLES_PER_LINE: u16 = 114;
const CPU_HZ: u64 = 1_789_772;
const SAMPLE_RATE: u64 = 48_000;
const FRAME_RATE: f64 = 59.94;
const STATE_VERSION: u32 = 1;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Mapper {
    Linear,
    SuperGame,
}

#[derive(Debug, Clone, Copy, Default)]
struct CartFeatures {
    rom_at_4000: bool,
    bank6_at_4000: bool,
    ram_at_4000: bool,
    pokey_at_4000: bool,
    pokey_at_450: bool,
    pokey_at_440: bool,
    pokey_at_800: bool,
}

struct Atari7800Cartridge {
    rom: Vec<u8>,
    mapper: Mapper,
    features: CartFeatures,
    bank: usize,
    ram: Vec<u8>,
}

impl Atari7800Cartridge {
    fn parse(image: &[u8]) -> Result<Self, String> {
        if image.is_empty() {
            return Err("Atari 7800 cartridge image is empty".into());
        }
        let has_header = image.len() >= 128 && image.get(1..10) == Some(b"ATARI7800");
        let (rom, cart_type, mapper_hint) = if has_header {
            let declared = u32::from_be_bytes(image[49..53].try_into().unwrap()) as usize;
            let payload = &image[128..];
            if declared != 0 && declared > payload.len() {
                return Err("A78 header declares more ROM data than the image contains".into());
            }
            let len = if declared == 0 {
                payload.len()
            } else {
                declared
            };
            let cart_type = u16::from_be_bytes([image[53], image[54]]);
            let mapper = if image[0] >= 4 { Some(image[64]) } else { None };
            (&payload[..len], cart_type, mapper)
        } else {
            (image, 0, None)
        };
        if rom.is_empty() || rom.len() > 2 * 1024 * 1024 {
            return Err(format!("unsupported Atari 7800 ROM size {}", rom.len()));
        }
        let legacy_mapper = if cart_type & (1 << 8) != 0 {
            2
        } else if cart_type & (1 << 9) != 0 {
            3
        } else if cart_type & (1 << 12) != 0 {
            4
        } else if cart_type & (1 << 1) != 0 {
            1
        } else {
            0
        };
        let mapper_code = mapper_hint.unwrap_or({
            if !has_header && rom.len() > 0xc000 {
                1
            } else {
                legacy_mapper
            }
        });
        let mapper = match mapper_code {
            0 => Mapper::Linear,
            1 => Mapper::SuperGame,
            other => {
                return Err(format!(
                    "Atari 7800 A78 mapper {other} is not implemented yet"
                ))
            }
        };
        let mut features = CartFeatures {
            pokey_at_4000: cart_type & (1 << 0) != 0,
            ram_at_4000: cart_type & (1 << 2) != 0,
            rom_at_4000: cart_type & (1 << 3) != 0,
            bank6_at_4000: cart_type & (1 << 4) != 0,
            pokey_at_450: cart_type & (1 << 6) != 0,
            pokey_at_440: cart_type & (1 << 10) != 0,
            pokey_at_800: cart_type & (1 << 15) != 0,
        };
        if cart_type & ((1 << 13) | (1 << 14)) != 0 {
            return Err("Atari 7800 bankset/halt-banked cartridges are not implemented yet".into());
        }
        if has_header && image[0] >= 4 {
            let options = image[65];
            if options & 0x80 != 0 {
                return Err("Atari 7800 v4 bankset cartridges are not implemented yet".into());
            }
            let option = options & 0x07;
            features.ram_at_4000 |= matches!(option, 1 | 2 | 3 | 6);
            features.rom_at_4000 |= matches!(option, 4 | 5);
            match u16::from_be_bytes([image[66], image[67]]) & 0x07 {
                1 => features.pokey_at_440 = true,
                2 => features.pokey_at_450 = true,
                3 => {
                    features.pokey_at_440 = true;
                    features.pokey_at_450 = true;
                }
                4 => features.pokey_at_800 = true,
                5 => features.pokey_at_4000 = true,
                _ => {}
            }
        }
        let ram = if features.ram_at_4000 {
            vec![0; 0x4000]
        } else {
            Vec::new()
        };
        Ok(Self {
            rom: rom.to_vec(),
            mapper,
            features,
            bank: 0,
            ram,
        })
    }

    fn bank_count(&self) -> usize {
        (self.rom.len() / 0x4000).max(1)
    }

    fn linear_read(&self, address: u16) -> u8 {
        let len = self.rom.len();
        let start = 0x10000usize.saturating_sub(len);
        let address = address as usize;
        if address < start {
            0xff
        } else {
            self.rom[(address - start) % len]
        }
    }

    fn supergame_base_bank(&self) -> usize {
        usize::from(self.features.rom_at_4000)
    }

    fn read(&self, address: u16) -> u8 {
        if self.mapper == Mapper::Linear {
            return self.linear_read(address);
        }
        let banks = self.bank_count();
        match address {
            0x4000..=0x7fff if !self.ram.is_empty() => {
                self.ram[(address as usize - 0x4000) % self.ram.len()]
            }
            0x4000..=0x7fff if self.features.rom_at_4000 && banks > 0 => {
                self.rom[(address as usize - 0x4000) % 0x4000]
            }
            0x4000..=0x7fff if self.features.bank6_at_4000 && banks > 6 => {
                self.rom[6 * 0x4000 + (address as usize & 0x3fff)]
            }
            0x4000..=0x7fff => 0xff,
            0x8000..=0xbfff => {
                let base = self.supergame_base_bank();
                let switchable = banks.saturating_sub(base).max(1);
                let bank = base + (self.bank % switchable);
                self.rom[(bank * 0x4000 + (address as usize & 0x3fff)) % self.rom.len()]
            }
            0xc000..=0xffff => {
                let bank = banks.saturating_sub(1);
                self.rom[(bank * 0x4000 + (address as usize & 0x3fff)) % self.rom.len()]
            }
            _ => 0xff,
        }
    }

    fn write(&mut self, address: u16, value: u8) {
        if !self.ram.is_empty() && (0x4000..=0x7fff).contains(&address) {
            let index = (address as usize - 0x4000) % self.ram.len();
            self.ram[index] = value;
            return;
        }
        if self.mapper == Mapper::SuperGame && (0x8000..=0xbfff).contains(&address) {
            self.bank = usize::from(value & 7);
        }
    }

    fn save(&self, out: &mut StateWriter) {
        out.u32(self.bank as u32);
        out.blob(&self.ram);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let bank = input.u32()? as usize;
        if bank >= self.bank_count().max(1) {
            return Err("Atari 7800 save state has invalid cartridge bank".into());
        }
        self.bank = bank;
        let ram = input.blob()?;
        if ram.len() != self.ram.len() {
            return Err("Atari 7800 save state has invalid cartridge RAM length".into());
        }
        self.ram.copy_from_slice(ram);
        Ok(())
    }
}

#[derive(Clone)]
struct Pia6532 {
    swcha: u8,
    swchb: u8,
    porta_out: u8,
    portb_out: u8,
    ddra: u8,
    ddrb: u8,
    timer: u8,
    divider: u16,
    phase: u16,
    irq: bool,
}

impl Default for Pia6532 {
    fn default() -> Self {
        Self {
            swcha: 0xff,
            swchb: 0xff,
            porta_out: 0xff,
            portb_out: 0xff,
            ddra: 0,
            ddrb: 0,
            timer: 0xff,
            divider: 1,
            phase: 1,
            irq: false,
        }
    }
}

impl Pia6532 {
    fn reset(&mut self) {
        *self = Self::default();
    }

    fn set_inputs(&mut self, input: &InputState) {
        self.swcha = 0xff;
        self.swchb = 0xff;
        for player in 0..2usize {
            let buttons = input.buttons[player];
            let base = if player == 0 { 4 } else { 0 };
            clear_active_low(&mut self.swcha, base, buttons & UP != 0);
            clear_active_low(&mut self.swcha, base + 1, buttons & DOWN != 0);
            clear_active_low(&mut self.swcha, base + 2, buttons & LEFT != 0);
            clear_active_low(&mut self.swcha, base + 3, buttons & RIGHT != 0);
        }
        clear_active_low(&mut self.swchb, 0, input.buttons[0] & START != 0);
        clear_active_low(&mut self.swchb, 1, input.buttons[0] & SELECT != 0);
    }

    fn tick(&mut self, cycles: u32) {
        for _ in 0..cycles {
            if self.phase > 1 {
                self.phase -= 1;
                continue;
            }
            self.phase = self.divider.max(1);
            if self.timer == 0 {
                self.timer = 0xff;
                self.divider = 1;
                self.phase = 1;
                self.irq = true;
            } else {
                self.timer = self.timer.wrapping_sub(1);
            }
        }
    }

    fn read(&mut self, address: u16) -> u8 {
        match address & 0x1f {
            0x00 => (self.swcha & !self.ddra) | (self.porta_out & self.ddra),
            0x01 => self.ddra,
            0x02 => (self.swchb & !self.ddrb) | (self.portb_out & self.ddrb),
            0x03 => self.ddrb,
            0x04 => {
                self.irq = false;
                self.timer
            }
            0x05 => {
                if self.irq {
                    0x80
                } else {
                    0
                }
            }
            _ => 0xff,
        }
    }

    fn write(&mut self, address: u16, value: u8) {
        match address & 0x1f {
            0x00 => self.porta_out = value,
            0x01 => self.ddra = value,
            0x02 => self.portb_out = value,
            0x03 => self.ddrb = value,
            0x14 => self.set_timer(value, 1),
            0x15 => self.set_timer(value, 8),
            0x16 => self.set_timer(value, 64),
            0x17 => self.set_timer(value, 1024),
            _ => {}
        }
    }

    fn set_timer(&mut self, value: u8, divider: u16) {
        self.timer = value;
        self.divider = divider;
        self.phase = divider;
        self.irq = false;
    }

    fn save(&self, out: &mut StateWriter) {
        out.u8(self.swcha);
        out.u8(self.swchb);
        out.u8(self.porta_out);
        out.u8(self.portb_out);
        out.u8(self.ddra);
        out.u8(self.ddrb);
        out.u8(self.timer);
        out.u16(self.divider);
        out.u16(self.phase);
        out.u8(self.irq as u8);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.swcha = input.u8()?;
        self.swchb = input.u8()?;
        self.porta_out = input.u8()?;
        self.portb_out = input.u8()?;
        self.ddra = input.u8()?;
        self.ddrb = input.u8()?;
        self.timer = input.u8()?;
        self.divider = input.u16()?.max(1);
        self.phase = input.u16()?.max(1);
        self.irq = input.u8()? != 0;
        Ok(())
    }
}

fn clear_active_low(value: &mut u8, bit: u8, pressed: bool) {
    if pressed {
        *value &= !(1 << bit);
    }
}

#[derive(Debug, Clone, Copy, Default)]
struct TiaAudioChannel {
    control: u8,
    frequency: u8,
    volume: u8,
    counter: u16,
    phase: bool,
}

#[derive(Clone)]
struct Tia7800 {
    regs: [u8; 32],
    fire: [bool; 2],
    channels: [TiaAudioChannel; 2],
    lfsr: u16,
    sample_phase: u64,
    samples: Vec<f32>,
}

impl Default for Tia7800 {
    fn default() -> Self {
        Self {
            regs: [0; 32],
            fire: [false; 2],
            channels: [TiaAudioChannel::default(); 2],
            lfsr: 0x1ff,
            sample_phase: 0,
            samples: Vec::with_capacity(1024),
        }
    }
}

impl Tia7800 {
    fn reset(&mut self) {
        let fire = self.fire;
        *self = Self::default();
        self.fire = fire;
    }

    fn set_inputs(&mut self, input: &InputState) {
        self.fire[0] = input.buttons[0] & FACE_SOUTH != 0;
        self.fire[1] = input.buttons[1] & FACE_SOUTH != 0;
    }

    fn read(&self, address: u16) -> u8 {
        match address & 0x1f {
            0x0c => {
                if self.fire[0] {
                    0x00
                } else {
                    0x80
                }
            }
            0x0d => {
                if self.fire[1] {
                    0x00
                } else {
                    0x80
                }
            }
            _ => 0,
        }
    }

    fn write(&mut self, address: u16, value: u8) {
        let register = (address & 0x1f) as usize;
        self.regs[register] = value;
        match register {
            0x15 => self.channels[0].control = value & 0x0f,
            0x16 => self.channels[1].control = value & 0x0f,
            0x17 => self.channels[0].frequency = value & 0x1f,
            0x18 => self.channels[1].frequency = value & 0x1f,
            0x19 => self.channels[0].volume = value & 0x0f,
            0x1a => self.channels[1].volume = value & 0x0f,
            _ => {}
        }
    }

    fn clock_lfsr(&mut self) {
        let feedback = ((self.lfsr >> 8) ^ (self.lfsr >> 4)) & 1;
        self.lfsr = ((self.lfsr << 1) | feedback) & 0x01ff;
        if self.lfsr == 0 {
            self.lfsr = 0x01ff;
        }
    }

    fn tick(&mut self, cycles: u32) {
        for _ in 0..cycles {
            self.clock_lfsr();
            for channel in &mut self.channels {
                if channel.counter == 0 {
                    channel.counter = u16::from(channel.frequency) + 1;
                    channel.phase = if channel.control & 0x08 != 0 {
                        self.lfsr & 1 != 0
                    } else {
                        !channel.phase
                    };
                } else {
                    channel.counter -= 1;
                }
            }
            self.sample_phase += SAMPLE_RATE;
            if self.sample_phase >= CPU_HZ {
                self.sample_phase -= CPU_HZ;
                self.samples.push(self.mix());
            }
        }
    }

    fn mix(&self) -> f32 {
        let sum: f32 = self
            .channels
            .iter()
            .map(|channel| {
                if channel.phase || channel.control == 0 {
                    f32::from(channel.volume) / 15.0
                } else {
                    0.0
                }
            })
            .sum();
        (sum * 0.35).clamp(0.0, 1.0)
    }

    fn take_samples(&mut self) -> Vec<f32> {
        std::mem::take(&mut self.samples)
    }

    fn save(&self, out: &mut StateWriter) {
        out.blob(&self.regs);
        for fire in self.fire {
            out.u8(fire as u8);
        }
        for channel in self.channels {
            out.u8(channel.control);
            out.u8(channel.frequency);
            out.u8(channel.volume);
            out.u16(channel.counter);
            out.u8(channel.phase as u8);
        }
        out.u16(self.lfsr);
        out.u64(self.sample_phase);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let regs = input.blob()?;
        if regs.len() != self.regs.len() {
            return Err("Atari 7800 TIA state has invalid register length".into());
        }
        self.regs.copy_from_slice(regs);
        for fire in &mut self.fire {
            *fire = input.u8()? != 0;
        }
        for channel in &mut self.channels {
            channel.control = input.u8()?;
            channel.frequency = input.u8()?;
            channel.volume = input.u8()?;
            channel.counter = input.u16()?;
            channel.phase = input.u8()? != 0;
        }
        self.lfsr = input.u16()?.max(1) & 0x01ff;
        self.sample_phase = input.u64()?;
        self.samples.clear();
        Ok(())
    }
}

struct Maria {
    regs: [u8; 32],
    scanline: u16,
    cycle_in_line: u16,
    frame: u64,
    wsync: bool,
    dli_pending: bool,
    dli_lines: Vec<u16>,
    last_dli_scanline: Option<u16>,
    video: VideoBuffer,
}

impl Default for Maria {
    fn default() -> Self {
        Self {
            regs: [0; 32],
            scanline: 0,
            cycle_in_line: 0,
            frame: 0,
            wsync: false,
            dli_pending: false,
            dli_lines: Vec::new(),
            last_dli_scanline: None,
            video: VideoBuffer::new(WIDTH, HEIGHT),
        }
    }
}

impl Maria {
    fn reset(&mut self) {
        *self = Self::default();
    }

    fn read(&self, address: u16) -> u8 {
        match address & 0x1f {
            0x08 => {
                if self.scanline < VISIBLE_TOP || self.scanline >= VISIBLE_TOP + VISIBLE_LINES {
                    0x80
                } else {
                    0x00
                }
            }
            _ => 0xff,
        }
    }

    fn write(&mut self, address: u16, value: u8) {
        let register = (address & 0x1f) as usize;
        if register == 0x04 {
            self.wsync = true;
        } else if register != 0x08 {
            self.regs[register] = value;
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

    fn tick(&mut self, cycles: u32) -> bool {
        let mut render_needed = false;
        let mut total = u32::from(self.cycle_in_line) + cycles;
        while total >= u32::from(CPU_CYCLES_PER_LINE) {
            total -= u32::from(CPU_CYCLES_PER_LINE);
            self.scanline = self.scanline.wrapping_add(1);
            if self.scanline == VISIBLE_TOP {
                render_needed = true;
            }
            if self.scanline >= SCANLINES {
                self.scanline = 0;
                self.frame = self.frame.wrapping_add(1);
            }
        }
        self.cycle_in_line = total as u16;
        render_needed
    }

    fn dma_enabled(&self) -> bool {
        self.regs[0x1c] & 0x60 == 0x40
    }

    fn display_list_list(&self) -> u16 {
        u16::from(self.regs[0x10]) | (u16::from(self.regs[0x0c]) << 8)
    }

    fn palette_color(&self, palette: usize, color: usize) -> [u8; 4] {
        if color == 0 {
            return atari7800_color(self.regs[0]);
        }
        let index = (palette & 7) * 4 + color.min(3);
        atari7800_color(self.regs[index & 0x1f])
    }

    fn dma_read(ram: &[u8; 0x4000], cart: &Atari7800Cartridge, address: u16) -> u8 {
        if address < 0x4000 {
            ram[address as usize]
        } else {
            cart.read(address)
        }
    }

    fn render_frame(&mut self, ram: &[u8; 0x4000], cart: &Atari7800Cartridge) {
        self.video.clear(atari7800_color(self.regs[0]));
        self.dli_lines.clear();
        if !self.dma_enabled() {
            return;
        }
        let mut dll = self.display_list_list();
        let mut y = 0usize;
        for _ in 0..256usize {
            if y >= HEIGHT as usize {
                break;
            }
            let flags = Self::dma_read(ram, cart, dll);
            let dl_hi = Self::dma_read(ram, cart, dll.wrapping_add(1));
            let dl_lo = Self::dma_read(ram, cart, dll.wrapping_add(2));
            dll = dll.wrapping_add(3);
            let display_list = u16::from_le_bytes([dl_lo, dl_hi]);
            let zone_height = usize::from(flags & 0x1f) + 1;
            for row in 0..zone_height {
                if y + row >= HEIGHT as usize {
                    break;
                }
                self.render_display_list(ram, cart, display_list, row, y + row);
            }
            if flags & 0x80 != 0 {
                let line = (y + zone_height.saturating_sub(1)).min(HEIGHT as usize - 1);
                self.dli_lines.push(line as u16);
            }
            y += zone_height;
        }
    }

    fn render_display_list(
        &mut self,
        ram: &[u8; 0x4000],
        cart: &Atari7800Cartridge,
        mut pointer: u16,
        row: usize,
        y: usize,
    ) {
        for _ in 0..128usize {
            let low = Self::dma_read(ram, cart, pointer);
            let control = Self::dma_read(ram, cart, pointer.wrapping_add(1));
            if control == 0 {
                break;
            }
            let extended = control & 0x40 != 0;
            let high = Self::dma_read(ram, cart, pointer.wrapping_add(2));
            let (palette_width, hpos, header_len, indirect) = if extended {
                (
                    Self::dma_read(ram, cart, pointer.wrapping_add(3)),
                    Self::dma_read(ram, cart, pointer.wrapping_add(4)),
                    5u16,
                    control & 0x20 != 0,
                )
            } else {
                (
                    control,
                    Self::dma_read(ram, cart, pointer.wrapping_add(3)),
                    4u16,
                    false,
                )
            };
            let encoded_width = usize::from(palette_width & 0x1f);
            let width = if encoded_width == 0 {
                32
            } else {
                32 - encoded_width
            };
            let palette = usize::from(palette_width >> 5);
            let base = u16::from_le_bytes([low, high]);
            let mode = self.regs[0x1c] & 0x03;
            for byte_index in 0..width {
                let row_offset = row.saturating_mul(width).saturating_add(byte_index);
                let address = base.wrapping_add(row_offset as u16);
                let mut value = Self::dma_read(ram, cart, address);
                if indirect {
                    let char_base = u16::from(self.regs[0x14]) << 8;
                    let glyph = char_base
                        .wrapping_add(u16::from(value) * 8)
                        .wrapping_add((row & 7) as u16);
                    value = Self::dma_read(ram, cart, glyph);
                }
                let x = usize::from(hpos) * 2 + byte_index * 8;
                if mode == 0 {
                    self.draw_160_byte(value, palette, x, y);
                } else {
                    self.draw_320_byte(value, palette, x, y);
                }
            }
            pointer = pointer.wrapping_add(header_len);
        }
    }

    fn draw_160_byte(&mut self, value: u8, palette: usize, x: usize, y: usize) {
        for pixel in 0..4usize {
            let shift = 6 - pixel * 2;
            let color = usize::from((value >> shift) & 3);
            if color == 0 && self.regs[0x1c] & 0x04 == 0 {
                continue;
            }
            let rgba = self.palette_color(palette, color);
            for dx in 0..2usize {
                let screen_x = x + pixel * 2 + dx;
                if screen_x < WIDTH as usize && y < HEIGHT as usize {
                    let offset = (y * WIDTH as usize + screen_x) * 4;
                    self.video.pixels_mut()[offset..offset + 4].copy_from_slice(&rgba);
                }
            }
        }
    }

    fn draw_320_byte(&mut self, value: u8, palette: usize, x: usize, y: usize) {
        for bit in 0..8usize {
            if value & (0x80 >> bit) == 0 && self.regs[0x1c] & 0x04 == 0 {
                continue;
            }
            let color = if value & (0x80 >> bit) != 0 { 2 } else { 0 };
            let rgba = self.palette_color(palette, color);
            let screen_x = x + bit;
            if screen_x < WIDTH as usize && y < HEIGHT as usize {
                let offset = (y * WIDTH as usize + screen_x) * 4;
                self.video.pixels_mut()[offset..offset + 4].copy_from_slice(&rgba);
            }
        }
    }

    fn update_dli(&mut self) {
        if !(VISIBLE_TOP..VISIBLE_TOP + VISIBLE_LINES).contains(&self.scanline) {
            self.last_dli_scanline = None;
            return;
        }
        let visible = self.scanline - VISIBLE_TOP;
        if self.last_dli_scanline == Some(self.scanline) {
            return;
        }
        if self.dli_lines.contains(&visible) {
            self.dli_pending = true;
            self.last_dli_scanline = Some(self.scanline);
        }
    }

    fn take_dli(&mut self) -> bool {
        std::mem::take(&mut self.dli_pending)
    }

    fn save(&self, out: &mut StateWriter) {
        out.blob(&self.regs);
        out.u16(self.scanline);
        out.u16(self.cycle_in_line);
        out.u64(self.frame);
        out.u8(self.wsync as u8);
        out.u8(self.dli_pending as u8);
        out.u32(self.dli_lines.len() as u32);
        for line in &self.dli_lines {
            out.u16(*line);
        }
        out.u16(self.last_dli_scanline.unwrap_or(u16::MAX));
        out.blob(self.video.pixels());
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let regs = input.blob()?;
        if regs.len() != self.regs.len() {
            return Err("MARIA state has invalid register length".into());
        }
        self.regs.copy_from_slice(regs);
        self.scanline = input.u16()? % SCANLINES;
        self.cycle_in_line = input.u16()? % CPU_CYCLES_PER_LINE;
        self.frame = input.u64()?;
        self.wsync = input.u8()? != 0;
        self.dli_pending = input.u8()? != 0;
        let dli_len = input.u32()? as usize;
        if dli_len > HEIGHT as usize {
            return Err("MARIA state has too many DLI lines".into());
        }
        self.dli_lines.clear();
        for _ in 0..dli_len {
            self.dli_lines.push(input.u16()?);
        }
        let last = input.u16()?;
        self.last_dli_scanline = if last == u16::MAX { None } else { Some(last) };
        let video = input.blob()?;
        if video.len() != self.video.pixels().len() {
            return Err("MARIA state has invalid framebuffer length".into());
        }
        self.video.pixels_mut().copy_from_slice(video);
        Ok(())
    }
}

fn atari7800_color(value: u8) -> [u8; 4] {
    let hue = f32::from(value >> 4) / 16.0;
    let luma = f32::from(value & 0x0e) / 14.0;
    let saturation = if value & 0xf0 == 0 { 0.0 } else { 0.74 };
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

struct Atari7800Bus {
    ram: [u8; 0x4000],
    cart: Atari7800Cartridge,
    pia: Pia6532,
    tia: Tia7800,
    maria: Maria,
    pokey: Pokey,
}

impl Atari7800Bus {
    fn new(cart: Atari7800Cartridge) -> Self {
        Self {
            ram: [0; 0x4000],
            cart,
            pia: Pia6532::default(),
            tia: Tia7800::default(),
            maria: Maria::default(),
            pokey: Pokey::new(CPU_HZ),
        }
    }

    fn reset_devices(&mut self) {
        self.pia.reset();
        self.tia.reset();
        self.maria.reset();
        self.pokey.reset();
        self.cart.bank = 0;
    }

    fn set_inputs(&mut self, input: &InputState) {
        self.pia.set_inputs(input);
        self.tia.set_inputs(input);
    }

    fn pokey_address(&self, address: u16) -> Option<u16> {
        let features = self.cart.features;
        let mapped = (features.pokey_at_4000 && (0x4000..=0x7fff).contains(&address))
            || (features.pokey_at_450 && (0x0450..=0x045f).contains(&address))
            || (features.pokey_at_440 && (0x0440..=0x044f).contains(&address))
            || (features.pokey_at_800 && (0x0800..=0x080f).contains(&address));
        mapped.then_some(address & 0x0f)
    }

    fn canonical_ram(address: u16) -> Option<usize> {
        match address {
            0x0040..=0x00ff
            | 0x0140..=0x01ff
            | 0x1800..=0x203f
            | 0x2100..=0x213f
            | 0x2200..=0x27ff => Some(address as usize),
            0x2040..=0x20ff => Some(0x0040 + (address as usize - 0x2040)),
            0x2140..=0x21ff => Some(0x0140 + (address as usize - 0x2140)),
            0x2800..=0x3fff => Some(0x2000 + ((address as usize - 0x2800) & 0x07ff)),
            _ => None,
        }
    }

    fn tick(&mut self, cycles: u32) {
        self.pia.tick(cycles);
        self.tia.tick(cycles);
        if self.pokey_enabled() {
            self.pokey.tick(cycles);
        }
        if self.maria.tick(cycles) {
            self.maria.render_frame(&self.ram, &self.cart);
        }
        self.maria.update_dli();
    }

    fn pokey_enabled(&self) -> bool {
        let features = self.cart.features;
        features.pokey_at_4000
            || features.pokey_at_450
            || features.pokey_at_440
            || features.pokey_at_800
    }

    fn irq_pending(&self) -> bool {
        self.pia.irq || (self.pokey_enabled() && self.pokey.irq_pending())
    }

    fn save(&self, out: &mut StateWriter) {
        out.blob(&self.ram);
        self.cart.save(out);
        self.pia.save(out);
        self.tia.save(out);
        self.maria.save(out);
        self.pokey.save(out);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let ram = input.blob()?;
        if ram.len() != self.ram.len() {
            return Err("Atari 7800 save state has invalid RAM length".into());
        }
        self.ram.copy_from_slice(ram);
        self.cart.load(input)?;
        self.pia.load(input)?;
        self.tia.load(input)?;
        self.maria.load(input)?;
        self.pokey.load(input)
    }
}

impl Bus8 for Atari7800Bus {
    fn read8(&mut self, address: u16) -> u8 {
        if let Some(pokey) = self.pokey_address(address) {
            return self.pokey.read(pokey);
        }
        if let Some(index) = Self::canonical_ram(address) {
            return self.ram[index];
        }
        match address {
            0x0000..=0x001f => self.tia.read(address),
            0x0020..=0x003f => self.maria.read(address - 0x20),
            0x0100..=0x011f => self.tia.read(address - 0x0100),
            0x0120..=0x013f => self.maria.read(address - 0x0120),
            0x0280..=0x02ff => self.pia.read(address),
            0x4000..=0xffff => self.cart.read(address),
            _ => 0xff,
        }
    }

    fn write8(&mut self, address: u16, value: u8) {
        if let Some(pokey) = self.pokey_address(address) {
            self.pokey.write(pokey, value);
            return;
        }
        if let Some(index) = Self::canonical_ram(address) {
            self.ram[index] = value;
            return;
        }
        match address {
            0x0000..=0x001f => self.tia.write(address, value),
            0x0020..=0x003f => self.maria.write(address - 0x20, value),
            0x0100..=0x011f => self.tia.write(address - 0x0100, value),
            0x0120..=0x013f => self.maria.write(address - 0x0120, value),
            0x0280..=0x02ff => self.pia.write(address, value),
            0x4000..=0xffff => self.cart.write(address, value),
            _ => {}
        }
    }
}

pub struct Atari7800Machine {
    cpu: Mos6502,
    bus: Atari7800Bus,
    audio: AudioBuffer,
    powered: bool,
}

impl Atari7800Machine {
    pub fn from_image(image: &[u8]) -> Result<Self, String> {
        let cart = Atari7800Cartridge::parse(image)?;
        let mut bus = Atari7800Bus::new(cart);
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
        if self.bus.maria.take_wsync() {
            let stall = self.bus.maria.cycles_until_line_end();
            self.cpu.cycles = self.cpu.cycles.saturating_add(u64::from(stall));
            self.bus.tick(stall);
        }
        if self.bus.maria.take_dli() {
            let before = self.cpu.cycles;
            self.cpu.nmi(&mut self.bus);
            self.bus.tick((self.cpu.cycles - before) as u32);
        }
        if self.bus.irq_pending() {
            let before = self.cpu.cycles;
            self.cpu.irq(&mut self.bus);
            self.bus.tick((self.cpu.cycles - before) as u32);
        }
        cycles
    }

    fn flush_audio(&mut self) {
        self.audio.begin_frame();
        let tia = self.bus.tia.take_samples();
        let pokey = if self.bus.pokey_enabled() {
            self.bus.pokey.take_samples()
        } else {
            Vec::new()
        };
        let count = tia.len().max(pokey.len());
        for index in 0..count {
            let tia_sample = tia.get(index).copied().unwrap_or(0.0);
            let pokey_sample = pokey.get(index).copied().unwrap_or(0.0);
            let sample = (tia_sample + pokey_sample * 0.75).clamp(0.0, 1.0);
            self.audio.push_stereo(sample, sample);
        }
        if count == 0 {
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

impl Machine for Atari7800Machine {
    fn platform(&self) -> PlatformId {
        PlatformId::Atari7800
    }

    fn reset(&mut self) {
        self.bus.reset_devices();
        self.cpu.reset(&mut self.bus);
        self.powered = true;
        self.audio.begin_frame();
    }

    fn run_frame(&mut self, input: &InputState) {
        self.bus.set_inputs(input);
        let target = self.bus.maria.frame.wrapping_add(1);
        let deadline = self
            .cpu
            .cycles
            .saturating_add((CPU_HZ as f64 / FRAME_RATE * 2.0) as u64);
        while self.bus.maria.frame != target && self.cpu.cycles < deadline {
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
        &self.bus.maria.video
    }

    fn audio(&self) -> &AudioBuffer {
        &self.audio
    }

    fn save_state(&self) -> Result<Vec<u8>, String> {
        let mut out = StateWriter::new(PlatformId::Atari7800, STATE_VERSION);
        self.save_cpu(&mut out);
        self.bus.save(&mut out);
        out.u8(self.powered as u8);
        Ok(out.finish())
    }

    fn load_state(&mut self, bytes: &[u8]) -> Result<(), String> {
        let mut input = StateReader::new(bytes, PlatformId::Atari7800, STATE_VERSION)?;
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

    fn synthetic_rom() -> Vec<u8> {
        let mut rom = vec![0xea; 0xc000];
        let code = [
            0x78, 0xd8, 0xa9, 0x00, 0x85, 0x20, 0xa9, 0x2e, 0x85, 0x21, 0xa9, 0x4e, 0x85, 0x22,
            0xa9, 0x6e, 0x85, 0x23, 0xa9, 0x00, 0x85, 0x30, 0xa9, 0x18, 0x85, 0x2c, 0xa9, 0x40,
            0x85, 0x3c, 0xa9, 0x04, 0x85, 0x15, 0xa9, 0x08, 0x85, 0x17, 0xa9, 0x0f, 0x85, 0x19,
            0x4c, 0x2a, 0x40,
        ];
        rom[..code.len()].copy_from_slice(&code);
        rom[0xbffa..0xc000].copy_from_slice(&[0x00, 0x40, 0x00, 0x40, 0x00, 0x40]);
        rom
    }

    fn install_display(machine: &mut Atari7800Machine) {
        let dll = 0x1800usize;
        let display_list = 0x1900usize;
        let graphics = 0x1a00usize;
        for zone in 0..30usize {
            let offset = dll + zone * 3;
            machine.bus.ram[offset] = 0x07;
            machine.bus.ram[offset + 1] = (display_list >> 8) as u8;
            machine.bus.ram[offset + 2] = display_list as u8;
        }
        machine.bus.ram[display_list] = graphics as u8;
        machine.bus.ram[display_list + 1] = 0x01;
        machine.bus.ram[display_list + 2] = (graphics >> 8) as u8;
        machine.bus.ram[display_list + 3] = 8;
        machine.bus.ram[display_list + 4] = 0;
        machine.bus.ram[display_list + 5] = 0;
        for row in 0..8usize {
            for byte in 0..31usize {
                machine.bus.ram[graphics + row * 31 + byte] = match (row + byte) & 3 {
                    0 => 0x55,
                    1 => 0xaa,
                    2 => 0xff,
                    _ => 0x11,
                };
            }
        }
    }

    #[test]
    fn synthetic_machine_runs_sally_maria_pia_and_tia_audio() {
        let mut machine = Atari7800Machine::from_image(&synthetic_rom()).unwrap();
        install_display(&mut machine);
        machine.run_frame(&InputState::default());
        assert_eq!(machine.video().width(), WIDTH);
        assert_eq!(machine.video().height(), HEIGHT);
        let pixels = machine.video().pixels();
        assert_eq!(machine.bus.maria.display_list_list(), 0x1800);
        assert!(pixels
            .iter()
            .enumerate()
            .any(|(index, &value)| index % 4 != 3 && value != 0));
        assert!(machine.audio().samples().iter().any(|&sample| sample > 0.0));
        assert!(machine.powered);
    }

    #[test]
    fn supergame_a78_header_switches_16k_bank() {
        let mut image = vec![0; 128 + 8 * 0x4000];
        image[0] = 4;
        image[1..10].copy_from_slice(b"ATARI7800");
        image[49..53].copy_from_slice(&(8u32 * 0x4000).to_be_bytes());
        image[53..55].copy_from_slice(&0x0002u16.to_be_bytes());
        image[64] = 1;
        for bank in 0..8usize {
            image[128 + bank * 0x4000..128 + (bank + 1) * 0x4000].fill(bank as u8);
        }
        let mut cart = Atari7800Cartridge::parse(&image).unwrap();
        assert_eq!(cart.read(0x8000), 0);
        assert_eq!(cart.read(0xc000), 7);
        cart.write(0x8000, 3);
        assert_eq!(cart.read(0x8000), 3);
    }

    #[test]
    fn save_state_round_trip_restores_cpu_ram_and_video() {
        let mut machine = Atari7800Machine::from_image(&synthetic_rom()).unwrap();
        install_display(&mut machine);
        machine.run_frame(&InputState::default());
        let state = machine.save_state().unwrap();
        let pc = machine.cpu.pc;
        let frame = machine.bus.maria.frame;
        let pixel = machine.video().pixels()[0];
        machine.cpu.pc = 0x1234;
        machine.bus.ram[0x1800] ^= 0xff;
        machine.bus.maria.video.pixels_mut()[0] ^= 0xff;
        machine.load_state(&state).unwrap();
        assert_eq!(machine.cpu.pc, pc);
        assert_eq!(machine.bus.maria.frame, frame);
        assert_eq!(machine.video().pixels()[0], pixel);
    }
}
