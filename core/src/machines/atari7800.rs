use super::pokey::Pokey;
use crate::bus::Bus8;
use crate::cpu6502::Mos6502;
use crate::input::{DOWN, FACE_EAST, FACE_SOUTH, LEFT, RIGHT, SELECT, START, UP};
use crate::kernel::{AudioBuffer, InputState, VideoBuffer};
use crate::machine::Machine;
use crate::platform::PlatformId;
use crate::state::{StateReader, StateWriter};

const WIDTH: u32 = 320;
const HEIGHT: u32 = 240;
const VISIBLE_TOP: u16 = 16;
const VISIBLE_LINES: u16 = 240;
const NTSC_SCANLINES: u16 = 262;
const PAL_SCANLINES: u16 = 312;
const CPU_CYCLES_PER_LINE: u16 = 114;
const NTSC_CPU_HZ: u64 = 1_789_772;
const PAL_CPU_HZ: u64 = 1_773_447;
const SAMPLE_RATE: u64 = 48_000;
const STATE_VERSION: u32 = 5;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum TvRegion {
    Ntsc,
    Pal,
}

impl TvRegion {
    fn cpu_hz(self) -> u64 {
        match self {
            Self::Ntsc => NTSC_CPU_HZ,
            Self::Pal => PAL_CPU_HZ,
        }
    }

    fn scanlines(self) -> u16 {
        match self {
            Self::Ntsc => NTSC_SCANLINES,
            Self::Pal => PAL_SCANLINES,
        }
    }

    fn frame_rate(self) -> f64 {
        self.cpu_hz() as f64 / (f64::from(CPU_CYCLES_PER_LINE) * f64::from(self.scanlines()))
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Mapper {
    Linear,
    SuperGame,
    Activision,
    Absolute,
    Souper,
}

#[derive(Debug, Clone, Copy, Default)]
struct CartFeatures {
    bankset: bool,
    rom_at_4000: bool,
    second_last_at_4000: bool,
    ram_at_4000: bool,
    mirror_ram_a8: bool,
    banked_ram_x2: bool,
    halt_banked_ram: bool,
    pokey_at_4000: bool,
    pokey_at_450: bool,
    pokey_at_440: bool,
    pokey_at_800: bool,
}

struct Atari7800Cartridge {
    rom: Vec<u8>,
    mapper: Mapper,
    region: TvRegion,
    features: CartFeatures,
    bank: usize,
    ram_bank: usize,
    ram: Vec<u8>,
    souper_chr_bank: [u8; 2],
    souper_mode: u8,
    souper_ram_page_bank: [u8; 2],
    souper_audio_command: u8,
    souper_audio_request: bool,
}

impl Atari7800Cartridge {
    fn parse(image: &[u8]) -> Result<Self, String> {
        if image.is_empty() {
            return Err("Atari 7800 cartridge image is empty".into());
        }
        let has_header = image.len() >= 128 && image.get(1..10) == Some(b"ATARI7800");
        let region = if has_header && image[57] & 1 != 0 {
            TvRegion::Pal
        } else {
            TvRegion::Ntsc
        };
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
        } else if cart_type & (1 << 5) != 0 || cart_type & (1 << 1) != 0 {
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
            2 => Mapper::Activision,
            3 => Mapper::Absolute,
            4 => Mapper::Souper,
            other => {
                return Err(format!(
                    "Atari 7800 A78 mapper {other} is not implemented yet"
                ))
            }
        };
        match mapper {
            Mapper::Activision if rom.len() != 8 * 0x4000 => {
                return Err("Atari 7800 Activision mapper requires 128 KiB ROM".into())
            }
            Mapper::Absolute if rom.len() != 4 * 0x4000 => {
                return Err("Atari 7800 Absolute mapper requires 64 KiB ROM".into())
            }
            Mapper::Souper if rom.len() != 32 * 0x4000 => {
                return Err("Atari 7800 Souper mapper requires 512 KiB ROM".into())
            }
            _ => {}
        }
        let mut features = CartFeatures {
            bankset: cart_type & (1 << 13) != 0,
            pokey_at_4000: cart_type & (1 << 0) != 0,
            ram_at_4000: cart_type & (1 << 2) != 0,
            mirror_ram_a8: cart_type & (1 << 7) != 0,
            banked_ram_x2: cart_type & (1 << 5) != 0,
            halt_banked_ram: cart_type & (1 << 14) != 0,
            rom_at_4000: cart_type & (1 << 3) != 0,
            second_last_at_4000: cart_type & (1 << 4) != 0,
            pokey_at_450: cart_type & (1 << 6) != 0,
            pokey_at_440: cart_type & (1 << 10) != 0,
            pokey_at_800: cart_type & (1 << 15) != 0,
        };
        if features.mirror_ram_a8 || features.banked_ram_x2 || features.halt_banked_ram {
            features.ram_at_4000 = true;
        }
        if has_header && image[0] >= 4 {
            let options = image[65];
            features.bankset |= options & 0x80 != 0;
            let option = options & 0x07;
            features.ram_at_4000 |= matches!(option, 1 | 2 | 3 | 6);
            features.mirror_ram_a8 |= option == 2;
            features.halt_banked_ram |= option == 3;
            features.banked_ram_x2 |= option == 6;
            features.rom_at_4000 |= option == 4;
            features.second_last_at_4000 |= option == 5;
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
        if features.banked_ram_x2 && mapper != Mapper::SuperGame {
            return Err("Atari 7800 EXRAM/X2 requires SuperGame mapping".into());
        }
        if features.bankset && features.banked_ram_x2 {
            return Err(
                "Atari 7800 Bankset + EXRAM/X2 is not a supported cartridge profile".into(),
            );
        }
        if features.bankset {
            if features.rom_at_4000 {
                return Err("Atari 7800 Bankset does not support EXROM at $4000".into());
            }
            if features.second_last_at_4000 && mapper != Mapper::SuperGame {
                return Err("Atari 7800 Bankset EXFIX requires SuperGame mapping".into());
            }
            if !matches!(mapper, Mapper::Linear | Mapper::SuperGame) {
                return Err(
                    "Atari 7800 bankset is only valid with linear or SuperGame mapping".into(),
                );
            }
            if rom.len() % 2 != 0 {
                return Err(
                    "Atari 7800 bankset ROM must contain equal CPU and MARIA halves".into(),
                );
            }
            let half = rom.len() / 2;
            match mapper {
                Mapper::Linear if half > 0xc000 => {
                    return Err(
                        "Atari 7800 linear bankset half exceeds the 48 KiB CPU window".into(),
                    )
                }
                Mapper::SuperGame if half < 0x8000 || half % 0x4000 != 0 => {
                    return Err(
                        "Atari 7800 SuperGame bankset halves must contain whole 16 KiB banks"
                            .into(),
                    )
                }
                _ => {}
            }
        }
        if features.second_last_at_4000 && rom.len() < 2 * 0x4000 {
            return Err("Atari 7800 EXFIX requires at least two 16 KiB ROM banks".into());
        }

        let ram = if mapper == Mapper::Souper || features.halt_banked_ram || features.banked_ram_x2
        {
            vec![0; 0x8000]
        } else if features.mirror_ram_a8 {
            vec![0; 0x2000]
        } else if features.ram_at_4000 {
            vec![0; 0x4000]
        } else {
            Vec::new()
        };
        Ok(Self {
            rom: rom.to_vec(),
            mapper,
            region,
            features,
            bank: 0,
            ram_bank: 0,
            ram,
            souper_chr_bank: [0; 2],
            souper_mode: 0,
            souper_ram_page_bank: [0; 2],
            souper_audio_command: 0,
            souper_audio_request: true,
        })
    }

    fn reset_mapping(&mut self) {
        self.bank = 0;
        self.ram_bank = 0;
        self.souper_chr_bank = [0; 2];
        self.souper_mode = 0;
        self.souper_ram_page_bank = [0; 2];
        self.souper_audio_command = 0;
        self.souper_audio_request = true;
    }

    fn view_bounds(&self, dma: bool) -> (usize, usize) {
        if self.features.bankset {
            let half = self.rom.len() / 2;
            (if dma { half } else { 0 }, half)
        } else {
            (0, self.rom.len())
        }
    }

    fn bank_count(&self) -> usize {
        let (_, len) = self.view_bounds(false);
        (len / 0x4000).max(1)
    }

    fn ram_index_4000(&self, address: u16, dma: bool) -> usize {
        let offset = usize::from(address - 0x4000);
        if self.mapper == Mapper::Souper {
            let mut page = offset >> 12;
            if self.souper_mode & 0x04 != 0 {
                if (0x6000..0x7000).contains(&address) {
                    page = usize::from(self.souper_ram_page_bank[0]);
                } else if (0x7000..0x8000).contains(&address) {
                    page = usize::from(self.souper_ram_page_bank[1]);
                }
            }
            (page << 12) | (offset & 0x0fff)
        } else if self.features.halt_banked_ram && dma {
            0x4000 + offset
        } else if self.features.banked_ram_x2 {
            self.ram_bank * 0x4000 + offset
        } else if self.features.mirror_ram_a8 {
            (offset & 0x00ff) | ((offset & 0x3e00) >> 1)
        } else {
            offset
        }
    }

    fn linear_read_view(&self, address: u16, dma: bool) -> u8 {
        let (view_base, len) = self.view_bounds(dma);
        let start = 0x10000usize.saturating_sub(len);
        let address = address as usize;
        if address < start {
            0xff
        } else {
            self.rom[view_base + ((address - start) % len)]
        }
    }

    fn supergame_base_bank(&self) -> usize {
        usize::from(self.features.rom_at_4000 && !self.features.bankset)
    }

    fn owns_4000_read_window(&self) -> bool {
        if !self.ram.is_empty() {
            return true;
        }
        match self.mapper {
            Mapper::Linear => {
                let (_, len) = self.view_bounds(false);
                0x10000usize.saturating_sub(len) <= 0x4000
            }
            Mapper::Activision | Mapper::Absolute | Mapper::Souper => true,
            Mapper::SuperGame => {
                self.features.bankset
                    || self.features.rom_at_4000
                    || self.features.second_last_at_4000
            }
        }
    }

    fn owns_4000_write_window(&self) -> bool {
        !self.ram.is_empty()
    }

    fn read(&self, address: u16) -> u8 {
        self.read_view(address, false)
    }

    fn read_dma(&self, address: u16) -> u8 {
        self.read_view(address, true)
    }

    fn read_view(&self, address: u16, dma: bool) -> u8 {
        let (view_base, view_len) = self.view_bounds(dma);
        if !self.ram.is_empty() && (0x4000..=0x7fff).contains(&address) {
            return self.ram[self.ram_index_4000(address, dma)];
        }
        match self.mapper {
            Mapper::Linear => return self.linear_read_view(address, dma),
            Mapper::Activision => {
                let (bank, offset) = match address {
                    0x4000..=0x5fff => (6usize, 0x2000 + usize::from(address - 0x4000)),
                    0x6000..=0x7fff => (6usize, usize::from(address - 0x6000)),
                    0x8000..=0x9fff => (7usize, 0x2000 + usize::from(address - 0x8000)),
                    0xa000..=0xdfff => (self.bank & 7, usize::from(address - 0xa000)),
                    0xe000..=0xffff => (7usize, usize::from(address - 0xe000)),
                    _ => return 0xff,
                };
                return self.rom[bank * 0x4000 + offset];
            }
            Mapper::Absolute => {
                return match address {
                    0x4000..=0x7fff => {
                        self.rom[(self.bank & 1) * 0x4000 + usize::from(address - 0x4000)]
                    }
                    0x8000..=0xffff => self.rom[0x8000 + usize::from(address - 0x8000)],
                    _ => 0xff,
                };
            }
            Mapper::Souper => {
                if dma && self.souper_mode & 0x01 != 0 {
                    if address >= 0xc000 {
                        let ram_address = address - 0x8000;
                        return self.ram[self.ram_index_4000(ram_address, true)];
                    }
                    if self.souper_mode & 0x02 != 0 {
                        if (0x8000..0xa000).contains(&address) {
                            return self.rom[31 * 0x4000 + usize::from(address - 0x8000)];
                        }
                        if (0xa000..0xc000).contains(&address) {
                            let page = self.souper_chr_bank[usize::from(address & 0x80 != 0)];
                            let chr_offset =
                                (((usize::from(page) & 0xfe) << 4) | usize::from(page & 1)) << 7;
                            let index = (usize::from(address) & 0x0f7f) | chr_offset;
                            return self.rom[index];
                        }
                    }
                }
                return match address {
                    0x8000..=0xbfff => {
                        self.rom[(self.bank & 31) * 0x4000 + usize::from(address - 0x8000)]
                    }
                    0xc000..=0xffff => self.rom[31 * 0x4000 + usize::from(address - 0xc000)],
                    _ => 0xff,
                };
            }
            Mapper::SuperGame => {}
        }
        let banks = (view_len / 0x4000).max(1);
        match address {
            0x4000..=0x7fff if self.features.bankset && banks >= 2 => {
                let bank = banks - 2;
                self.rom[view_base + bank * 0x4000 + (address as usize & 0x3fff)]
            }
            0x4000..=0x7fff if self.features.rom_at_4000 && banks > 0 => {
                self.rom[view_base + (address as usize - 0x4000) % 0x4000]
            }
            0x4000..=0x7fff if self.features.second_last_at_4000 && banks >= 2 => {
                let bank = banks - 2;
                self.rom[view_base + bank * 0x4000 + (address as usize & 0x3fff)]
            }
            0x4000..=0x7fff => 0xff,
            0x8000..=0xbfff => {
                let base = self.supergame_base_bank();
                let switchable = banks.saturating_sub(base).max(1);
                let bank = base + (self.bank % switchable);
                self.rom[view_base + bank * 0x4000 + (address as usize & 0x3fff)]
            }
            0xc000..=0xffff => {
                let bank = banks.saturating_sub(1);
                self.rom[view_base + bank * 0x4000 + (address as usize & 0x3fff)]
            }
            _ => 0xff,
        }
    }

    fn write(&mut self, address: u16, value: u8) {
        if self.features.halt_banked_ram && (0xc000..=0xffff).contains(&address) {
            let index = 0x4000 + usize::from(address - 0xc000);
            self.ram[index] = value;
            return;
        }
        if !self.ram.is_empty() && (0x4000..=0x7fff).contains(&address) {
            let index = self.ram_index_4000(address, false);
            self.ram[index] = value;
            return;
        }
        match self.mapper {
            Mapper::SuperGame if (0x8000..=0xbfff).contains(&address) => {
                if self.features.banked_ram_x2 {
                    self.ram_bank = usize::from((value >> 5) & 1);
                    self.bank = usize::from(value & 0x1f) % self.bank_count();
                } else {
                    self.bank = if self.features.bankset {
                        usize::from(value & 0x0f) % self.bank_count()
                    } else {
                        usize::from(value & 7)
                    };
                }
            }
            Mapper::Activision if (0xff80..=0xff87).contains(&address) => {
                self.bank = usize::from(address & 7);
            }
            Mapper::Absolute if address == 0x8000 => match value {
                1 => self.bank = 0,
                2 => self.bank = 1,
                _ => {}
            },
            Mapper::Souper if address >= 0x8000 => match address & 7 {
                0 => self.bank = usize::from(value & 31),
                1 => self.souper_chr_bank[0] = value,
                2 => self.souper_chr_bank[1] = value,
                3 => self.souper_mode = value & 7,
                4 => self.souper_ram_page_bank[0] = value & 7,
                5 => self.souper_ram_page_bank[1] = value & 7,
                7 => {
                    self.souper_audio_command = value;
                    self.souper_audio_request = !self.souper_audio_request;
                }
                _ => {}
            },
            _ => {}
        }
    }

    fn save(&self, out: &mut StateWriter) {
        out.u32(self.bank as u32);
        out.u8(self.ram_bank as u8);
        out.blob(&self.ram);
        for bank in self.souper_chr_bank {
            out.u8(bank);
        }
        out.u8(self.souper_mode);
        for bank in self.souper_ram_page_bank {
            out.u8(bank);
        }
        out.u8(self.souper_audio_command);
        out.u8(self.souper_audio_request as u8);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let bank = input.u32()? as usize;
        if bank >= self.bank_count().max(1) {
            return Err("Atari 7800 save state has invalid cartridge bank".into());
        }
        self.bank = bank;
        let ram_bank = usize::from(input.u8()?);
        if ram_bank >= 2 {
            return Err("Atari 7800 save state has invalid cartridge RAM bank".into());
        }
        self.ram_bank = ram_bank;
        let ram = input.blob()?;
        if ram.len() != self.ram.len() {
            return Err("Atari 7800 save state has invalid cartridge RAM length".into());
        }
        self.ram.copy_from_slice(ram);
        for bank in &mut self.souper_chr_bank {
            *bank = input.u8()?;
        }
        let souper_mode = input.u8()?;
        if souper_mode & !7 != 0 {
            return Err("Atari 7800 save state has invalid Souper mode".into());
        }
        self.souper_mode = souper_mode;
        for bank in &mut self.souper_ram_page_bank {
            *bank = input.u8()?;
            if *bank >= 8 {
                return Err("Atari 7800 save state has invalid Souper RAM page".into());
            }
        }
        self.souper_audio_command = input.u8()?;
        self.souper_audio_request = input.u8()? != 0;
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

    fn two_button_mode(&self, player: usize) -> bool {
        let bit = if player == 0 { 2 } else { 4 };
        let mask = 1 << bit;
        self.ddrb & mask != 0 && self.portb_out & mask == 0
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
    cpu_hz: u64,
    regs: [u8; 32],
    fire: [bool; 2],
    proline_buttons: [[bool; 2]; 2],
    channels: [TiaAudioChannel; 2],
    lfsr: u16,
    sample_phase: u64,
    samples: Vec<f32>,
}

impl Default for Tia7800 {
    fn default() -> Self {
        Self::new(NTSC_CPU_HZ)
    }
}

impl Tia7800 {
    fn new(cpu_hz: u64) -> Self {
        Self {
            cpu_hz,
            regs: [0; 32],
            fire: [false; 2],
            proline_buttons: [[false; 2]; 2],
            channels: [TiaAudioChannel::default(); 2],
            lfsr: 0x1ff,
            sample_phase: 0,
            samples: Vec::with_capacity(1024),
        }
    }

    fn reset(&mut self) {
        let cpu_hz = self.cpu_hz;
        let fire = self.fire;
        let proline_buttons = self.proline_buttons;
        *self = Self::new(cpu_hz);
        self.fire = fire;
        self.proline_buttons = proline_buttons;
    }

    fn set_inputs(&mut self, input: &InputState) {
        for player in 0..2 {
            let buttons = input.buttons[player];
            self.proline_buttons[player] = [buttons & FACE_EAST != 0, buttons & FACE_SOUTH != 0];
            self.fire[player] = self.proline_buttons[player][0] || self.proline_buttons[player][1];
        }
    }

    fn read(&self, address: u16, two_button_mode: [bool; 2]) -> u8 {
        match address & 0x1f {
            0x08 if two_button_mode[0] => u8::from(self.proline_buttons[0][0]) << 7,
            0x09 if two_button_mode[0] => u8::from(self.proline_buttons[0][1]) << 7,
            0x0a if two_button_mode[1] => u8::from(self.proline_buttons[1][0]) << 7,
            0x0b if two_button_mode[1] => u8::from(self.proline_buttons[1][1]) << 7,
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
            if self.sample_phase >= self.cpu_hz {
                self.sample_phase -= self.cpu_hz;
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
        for player in self.proline_buttons {
            for pressed in player {
                out.u8(pressed as u8);
            }
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
        for player in &mut self.proline_buttons {
            for pressed in player {
                *pressed = input.u8()? != 0;
            }
        }
        for channel in &mut self.channels {
            channel.control = input.u8()?;
            channel.frequency = input.u8()?;
            channel.volume = input.u8()?;
            channel.counter = input.u16()?;
            channel.phase = input.u8()? != 0;
        }
        self.lfsr = input.u16()?.max(1) & 0x01ff;
        self.sample_phase = input.u64()? % self.cpu_hz;
        self.samples.clear();
        Ok(())
    }
}

struct Maria {
    scanlines: u16,
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
        Self::new(NTSC_SCANLINES)
    }
}

impl Maria {
    fn new(scanlines: u16) -> Self {
        Self {
            scanlines,
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

    fn reset(&mut self) {
        let scanlines = self.scanlines;
        *self = Self::new(scanlines);
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
            if self.scanline >= self.scanlines {
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
            cart.read_dma(address)
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
        self.scanline = input.u16()? % self.scanlines;
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
        let region = cart.region;
        let cpu_hz = region.cpu_hz();
        Self {
            ram: [0; 0x4000],
            cart,
            pia: Pia6532::default(),
            tia: Tia7800::new(cpu_hz),
            maria: Maria::new(region.scanlines()),
            pokey: Pokey::new(cpu_hz),
        }
    }

    fn reset_devices(&mut self) {
        self.pia.reset();
        self.tia.reset();
        self.maria.reset();
        self.pokey.reset();
        self.cart.reset_mapping();
    }

    fn set_inputs(&mut self, input: &InputState) {
        self.pia.set_inputs(input);
        self.tia.set_inputs(input);
    }

    fn pokey_address(&self, address: u16, write: bool) -> Option<u16> {
        let features = self.cart.features;
        let at_4000 = features.pokey_at_4000
            && (0x4000..=0x7fff).contains(&address)
            && if write {
                !self.cart.owns_4000_write_window()
            } else {
                !self.cart.owns_4000_read_window()
            };
        let mapped = at_4000
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
        if let Some(pokey) = self.pokey_address(address, false) {
            return self.pokey.read(pokey);
        }
        if let Some(index) = Self::canonical_ram(address) {
            return self.ram[index];
        }
        let two_button_mode = [self.pia.two_button_mode(0), self.pia.two_button_mode(1)];
        match address {
            0x0000..=0x001f => self.tia.read(address, two_button_mode),
            0x0020..=0x003f => self.maria.read(address - 0x20),
            0x0100..=0x011f => self.tia.read(address - 0x0100, two_button_mode),
            0x0120..=0x013f => self.maria.read(address - 0x0120),
            0x0280..=0x02ff => self.pia.read(address),
            0x4000..=0xffff => self.cart.read(address),
            _ => 0xff,
        }
    }

    fn write8(&mut self, address: u16, value: u8) {
        if let Some(pokey) = self.pokey_address(address, true) {
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
        let region = self.bus.cart.region;
        let deadline = self
            .cpu
            .cycles
            .saturating_add((region.cpu_hz() as f64 / region.frame_rate() * 2.0) as u64);
        while self.bus.maria.frame != target && self.cpu.cycles < deadline {
            if self.clock_instruction() == 0 {
                self.powered = false;
                break;
            }
        }
        self.flush_audio();
    }

    fn frame_rate(&self) -> f64 {
        self.bus.cart.region.frame_rate()
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

    fn synthetic_a78(region: TvRegion) -> Vec<u8> {
        let rom = synthetic_rom();
        let mut image = vec![0; 128 + rom.len()];
        image[0] = 4;
        image[1..10].copy_from_slice(b"ATARI7800");
        image[49..53].copy_from_slice(&(rom.len() as u32).to_be_bytes());
        image[57] = u8::from(region == TvRegion::Pal);
        image[64] = 0;
        image[128..].copy_from_slice(&rom);
        image
    }

    fn synthetic_souper(version: u8) -> Vec<u8> {
        let mut image = vec![0; 128 + 32 * 0x4000];
        image[0] = version;
        image[1..10].copy_from_slice(b"ATARI7800");
        image[49..53].copy_from_slice(&(32u32 * 0x4000).to_be_bytes());
        if version >= 4 {
            image[64] = 4;
        } else {
            image[53..55].copy_from_slice(&(1u16 << 12).to_be_bytes());
        }
        for bank in 0..32usize {
            image[128 + bank * 0x4000..128 + (bank + 1) * 0x4000].fill(bank as u8);
        }
        image[128 + 0x2000] = 0xa4;
        image[128 + 0x2080] = 0xb5;
        image
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
    fn a78_region_selects_pal_machine_timing_and_survives_state_restore() {
        let image = synthetic_a78(TvRegion::Pal);
        let cart = Atari7800Cartridge::parse(&image).unwrap();
        assert_eq!(cart.region, TvRegion::Pal);

        let mut machine = Atari7800Machine::from_image(&image).unwrap();
        assert_eq!(machine.bus.tia.cpu_hz, PAL_CPU_HZ);
        assert_eq!(machine.bus.maria.scanlines, PAL_SCANLINES);
        assert!((machine.frame_rate() - TvRegion::Pal.frame_rate()).abs() < f64::EPSILON);
        assert!(machine.frame_rate() > 49.0 && machine.frame_rate() < 51.0);

        machine.bus.maria.scanline = PAL_SCANLINES - 1;
        machine.bus.maria.cycle_in_line = CPU_CYCLES_PER_LINE - 1;
        let frame = machine.bus.maria.frame;
        machine.bus.maria.tick(1);
        assert_eq!(machine.bus.maria.scanline, 0);
        assert_eq!(machine.bus.maria.frame, frame + 1);

        machine.bus.maria.scanline = 300;
        machine.bus.tia.sample_phase = PAL_CPU_HZ - 1;
        let state = machine.save_state().unwrap();
        machine.bus.maria.scanline = 0;
        machine.bus.tia.sample_phase = 0;
        machine.load_state(&state).unwrap();
        assert_eq!(machine.bus.maria.scanline, 300);
        assert_eq!(machine.bus.tia.cpu_hz, PAL_CPU_HZ);
        assert_eq!(machine.bus.tia.sample_phase, PAL_CPU_HZ - 1);

        let ntsc = Atari7800Machine::from_image(&synthetic_a78(TvRegion::Ntsc)).unwrap();
        assert_eq!(ntsc.bus.tia.cpu_hz, NTSC_CPU_HZ);
        assert_eq!(ntsc.bus.maria.scanlines, NTSC_SCANLINES);
        assert!(ntsc.frame_rate() > machine.frame_rate());
    }

    #[test]
    fn proline_two_button_mode_routes_independent_buttons_through_tia_inputs() {
        let mut machine = Atari7800Machine::from_image(&synthetic_rom()).unwrap();
        let mut input = InputState::default();
        input.buttons[0] = FACE_EAST;
        input.buttons[1] = FACE_SOUTH;
        machine.bus.set_inputs(&input);

        assert_eq!(machine.bus.read8(0x0008), 0);
        assert_eq!(machine.bus.read8(0x0009), 0);
        assert_eq!(machine.bus.read8(0x000c), 0);
        assert_eq!(machine.bus.read8(0x000d), 0);

        machine.bus.write8(0x0283, 0x14);
        machine.bus.write8(0x0282, 0x00);
        assert_eq!(machine.bus.read8(0x0008), 0x80);
        assert_eq!(machine.bus.read8(0x0009), 0x00);
        assert_eq!(machine.bus.read8(0x000a), 0x00);
        assert_eq!(machine.bus.read8(0x000b), 0x80);
        assert_eq!(machine.bus.read8(0x000c), 0x00);
        assert_eq!(machine.bus.read8(0x000d), 0x00);

        input.buttons[0] = FACE_SOUTH;
        input.buttons[1] = FACE_EAST;
        machine.bus.set_inputs(&input);
        assert_eq!(machine.bus.read8(0x0108), 0x00);
        assert_eq!(machine.bus.read8(0x0109), 0x80);
        assert_eq!(machine.bus.read8(0x010a), 0x80);
        assert_eq!(machine.bus.read8(0x010b), 0x00);

        let state = machine.save_state().unwrap();
        machine.bus.write8(0x0282, 0x14);
        machine.bus.set_inputs(&InputState::default());
        assert_eq!(machine.bus.read8(0x0009), 0x00);

        machine.load_state(&state).unwrap();
        assert_eq!(machine.bus.read8(0x0009), 0x80);
        assert_eq!(machine.bus.read8(0x000a), 0x80);

        machine.bus.write8(0x0282, 0x04);
        assert_eq!(machine.bus.read8(0x0008), 0x00);
        assert_eq!(machine.bus.read8(0x0009), 0x00);
        assert_eq!(machine.bus.read8(0x000c), 0x00);
        assert_eq!(machine.bus.read8(0x000a), 0x80);
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
    fn souper_v4_and_legacy_headers_select_mapper_and_reset_layout() {
        for version in [3u8, 4] {
            let mut cart = Atari7800Cartridge::parse(&synthetic_souper(version)).unwrap();
            assert_eq!(cart.mapper, Mapper::Souper);
            assert_eq!(cart.ram.len(), 0x8000);
            assert_eq!(cart.read(0x8000), 0);
            assert_eq!(cart.read(0xc000), 31);

            cart.write(0x9230, 5);
            assert_eq!(cart.read(0x8000), 5);
            cart.write(0x8003, 7);
            cart.write(0x8004, 6);
            cart.write(0x8005, 7);
            cart.write(0x8007, 0x9a);
            assert!(!cart.souper_audio_request);

            cart.reset_mapping();
            assert_eq!(cart.read(0x8000), 0);
            assert_eq!(cart.souper_mode, 0);
            assert_eq!(cart.souper_ram_page_bank, [0; 2]);
            assert_eq!(cart.souper_audio_command, 0);
            assert!(cart.souper_audio_request);
        }
    }

    #[test]
    fn souper_exram_pages_and_repeated_register_decode_follow_hardware() {
        let mut cart = Atari7800Cartridge::parse(&synthetic_souper(4)).unwrap();

        cart.write(0x6123, 0x11);
        cart.write(0x9004, 6);
        cart.write(0xa005, 7);
        cart.write(0xb003, 4);
        cart.write(0x6123, 0x66);
        cart.write(0x7456, 0x77);
        assert_eq!(cart.read(0x6123), 0x66);
        assert_eq!(cart.read(0x7456), 0x77);

        cart.write(0xc003, 0);
        assert_eq!(cart.read(0x6123), 0x11);
        assert_eq!(cart.read(0x7456), 0);

        assert!(cart.souper_audio_request);
        cart.write(0xd007, 0x4c);
        assert_eq!(cart.souper_audio_command, 0x4c);
        assert!(!cart.souper_audio_request);
        cart.write(0xffff, 0x5d);
        assert_eq!(cart.souper_audio_command, 0x5d);
        assert!(cart.souper_audio_request);
    }

    #[test]
    fn souper_maria_mft_chr_remaps_fixed_rom_character_rom_and_exram() {
        let mut cart = Atari7800Cartridge::parse(&synthetic_souper(4)).unwrap();

        cart.write(0x4000, 0x44);
        cart.write(0x8000, 5);
        cart.write(0x8003, 1);
        assert_eq!(cart.read_dma(0x8000), 5);
        assert_eq!(cart.read_dma(0xc000), 0x44);

        cart.write(0x8001, 4);
        cart.write(0x8002, 5);
        cart.write(0x8003, 3);
        assert_eq!(cart.read_dma(0x8000), 31);
        assert_eq!(cart.read_dma(0x9fff), 31);
        assert_eq!(cart.read_dma(0xa000), 0xa4);
        assert_eq!(cart.read_dma(0xa080), 0xb5);
        assert_eq!(cart.read_dma(0xc000), 0x44);

        assert_eq!(cart.read(0x8000), 5);
        assert_eq!(cart.read(0xc000), 31);
    }

    #[test]
    fn souper_mapping_and_exram_round_trip_through_state() {
        let mut cart = Atari7800Cartridge::parse(&synthetic_souper(4)).unwrap();
        cart.write(0x8000, 7);
        cart.write(0x8001, 0x24);
        cart.write(0x8002, 0x35);
        cart.write(0x8004, 6);
        cart.write(0x8005, 7);
        cart.write(0x8003, 7);
        cart.write(0x6123, 0x66);
        cart.write(0x7456, 0x77);
        cart.write(0x8007, 0xa5);

        let mut out = StateWriter::new(PlatformId::Atari7800, STATE_VERSION);
        cart.save(&mut out);
        let state = out.finish();

        cart.reset_mapping();
        cart.write(0x6123, 0);
        cart.write(0x7456, 0);

        let mut input = StateReader::new(&state, PlatformId::Atari7800, STATE_VERSION).unwrap();
        cart.load(&mut input).unwrap();
        input.finish().unwrap();

        assert_eq!(cart.bank, 7);
        assert_eq!(cart.souper_chr_bank, [0x24, 0x35]);
        assert_eq!(cart.souper_mode, 7);
        assert_eq!(cart.souper_ram_page_bank, [6, 7]);
        assert_eq!(cart.souper_audio_command, 0xa5);
        assert!(!cart.souper_audio_request);
        assert_eq!(cart.read(0x6123), 0x66);
        assert_eq!(cart.read(0x7456), 0x77);
    }

    #[test]
    fn v4_exram_a8_mirrors_address_bit_eight_into_eight_kib_ram() {
        let mut image = vec![0; 128 + 0xc000];
        image[0] = 4;
        image[1..10].copy_from_slice(b"ATARI7800");
        image[49..53].copy_from_slice(&0xc000u32.to_be_bytes());
        image[64] = 0;
        image[65] = 2;
        image[128..].fill(0xa5);

        let mut cart = Atari7800Cartridge::parse(&image).unwrap();
        assert!(cart.features.mirror_ram_a8);
        assert_eq!(cart.ram.len(), 0x2000);

        cart.write(0x4000, 0x11);
        assert_eq!(cart.read(0x4000), 0x11);
        assert_eq!(cart.read(0x4100), 0x11);
        cart.write(0x4200, 0x22);
        assert_eq!(cart.read(0x4200), 0x22);
        assert_eq!(cart.read(0x4300), 0x22);
        assert_eq!(cart.read(0x4000), 0x11);

        cart.write(0x7eff, 0x33);
        assert_eq!(cart.read(0x7fff), 0x33);
        assert_eq!(cart.read_dma(0x7fff), 0x33);
    }

    #[test]
    fn legacy_mirror_ram_flag_uses_exram_a8_mapping() {
        let mut image = vec![0; 128 + 0xc000];
        image[0] = 3;
        image[1..10].copy_from_slice(b"ATARI7800");
        image[49..53].copy_from_slice(&0xc000u32.to_be_bytes());
        image[53..55].copy_from_slice(&(1u16 << 7).to_be_bytes());

        let mut cart = Atari7800Cartridge::parse(&image).unwrap();
        assert!(cart.features.mirror_ram_a8);
        assert_eq!(cart.ram.len(), 0x2000);
        cart.write(0x55aa, 0x6c);
        assert_eq!(cart.read(0x54aa), 0x6c);
    }

    #[test]
    fn bankset_with_standard_ram_keeps_shared_ram_ahead_of_both_rom_views() {
        let mut image = vec![0; 128 + 16 * 0x4000];
        image[0] = 4;
        image[1..10].copy_from_slice(b"ATARI7800");
        image[49..53].copy_from_slice(&(16u32 * 0x4000).to_be_bytes());
        image[64] = 1;
        image[65] = 0x80 | 1;
        for bank in 0..16usize {
            image[128 + bank * 0x4000..128 + (bank + 1) * 0x4000].fill(bank as u8);
        }

        let mut cart = Atari7800Cartridge::parse(&image).unwrap();
        assert!(cart.features.bankset);
        assert!(cart.features.ram_at_4000);
        assert!(!cart.features.halt_banked_ram);
        assert_eq!(cart.ram.len(), 0x4000);

        cart.write(0x4000, 0x5a);
        assert_eq!(cart.read(0x4000), 0x5a);
        assert_eq!(cart.read_dma(0x4000), 0x5a);
        assert_eq!(cart.read(0xc000), 7);
        assert_eq!(cart.read_dma(0xc000), 15);
    }

    #[test]
    fn v4_exram_x2_banks_ram_on_supergame_bit_five() {
        let mut image = vec![0; 128 + 32 * 0x4000];
        image[0] = 4;
        image[1..10].copy_from_slice(b"ATARI7800");
        image[49..53].copy_from_slice(&(32u32 * 0x4000).to_be_bytes());
        image[64] = 1;
        image[65] = 6;
        for bank in 0..32usize {
            image[128 + bank * 0x4000..128 + (bank + 1) * 0x4000].fill(bank as u8);
        }

        let mut cart = Atari7800Cartridge::parse(&image).unwrap();
        assert!(cart.features.banked_ram_x2);
        assert_eq!(cart.ram.len(), 0x8000);
        assert_eq!(cart.ram_bank, 0);

        cart.write(0x4000, 0x11);
        cart.write(0x8000, 0x21);
        assert_eq!(cart.ram_bank, 1);
        assert_eq!(cart.read(0x4000), 0x00);
        assert_eq!(cart.read(0x8000), 1);
        assert_eq!(cart.read(0xc000), 31);
        cart.write(0x4000, 0x22);

        cart.write(0x8000, 0x02);
        assert_eq!(cart.ram_bank, 0);
        assert_eq!(cart.read(0x4000), 0x11);
        assert_eq!(cart.read(0x8000), 2);

        cart.write(0x8000, 0x21);
        assert_eq!(cart.read(0x4000), 0x22);
        let mut out = StateWriter::new(PlatformId::Atari7800, STATE_VERSION);
        cart.save(&mut out);
        let state = out.finish();

        cart.write(0x8000, 0x02);
        cart.write(0x4000, 0x55);
        let mut input = StateReader::new(&state, PlatformId::Atari7800, STATE_VERSION).unwrap();
        cart.load(&mut input).unwrap();
        input.finish().unwrap();
        assert_eq!(cart.bank, 1);
        assert_eq!(cart.ram_bank, 1);
        assert_eq!(cart.read(0x4000), 0x22);
    }

    #[test]
    fn legacy_exram_x2_flag_implies_supergame_mapping() {
        let mut image = vec![0; 128 + 8 * 0x4000];
        image[0] = 3;
        image[1..10].copy_from_slice(b"ATARI7800");
        image[49..53].copy_from_slice(&(8u32 * 0x4000).to_be_bytes());
        image[53..55].copy_from_slice(&(1u16 << 5).to_be_bytes());

        let mut cart = Atari7800Cartridge::parse(&image).unwrap();
        assert_eq!(cart.mapper, Mapper::SuperGame);
        assert!(cart.features.banked_ram_x2);
        cart.write(0x8000, 0x21);
        assert_eq!(cart.ram_bank, 1);
        assert_eq!(cart.bank, 1);
    }

    #[test]
    fn exram_x2_rejects_linear_and_bankset_profiles() {
        let mut linear = vec![0; 128 + 0xc000];
        linear[0] = 4;
        linear[1..10].copy_from_slice(b"ATARI7800");
        linear[49..53].copy_from_slice(&0xc000u32.to_be_bytes());
        linear[64] = 0;
        linear[65] = 6;
        let error = match Atari7800Cartridge::parse(&linear) {
            Ok(_) => panic!("linear EXRAM/X2 must be rejected"),
            Err(error) => error,
        };
        assert!(error.contains("requires SuperGame"));

        let mut bankset = vec![0; 128 + 8 * 0x4000];
        bankset[0] = 4;
        bankset[1..10].copy_from_slice(b"ATARI7800");
        bankset[49..53].copy_from_slice(&(8u32 * 0x4000).to_be_bytes());
        bankset[64] = 1;
        bankset[65] = 0x80 | 6;
        let error = match Atari7800Cartridge::parse(&bankset) {
            Ok(_) => panic!("Bankset + EXRAM/X2 must be rejected"),
            Err(error) => error,
        };
        assert!(error.contains("Bankset + EXRAM/X2"));
    }

    #[test]
    fn linear_v4_ram_at_4000_overrides_rom_for_cpu_and_maria() {
        let mut image = vec![0; 128 + 0xc000];
        image[0] = 4;
        image[1..10].copy_from_slice(b"ATARI7800");
        image[49..53].copy_from_slice(&0xc000u32.to_be_bytes());
        image[64] = 0;
        image[65] = 1;
        image[128..].fill(0xa5);

        let mut cart = Atari7800Cartridge::parse(&image).unwrap();
        assert!(cart.features.ram_at_4000);
        assert!(!cart.features.halt_banked_ram);
        assert_eq!(cart.ram.len(), 0x4000);
        assert_eq!(cart.read(0x4000), 0);
        assert_eq!(cart.read_dma(0x4000), 0);

        cart.write(0x4000, 0x5a);
        cart.write(0x7fff, 0xc3);
        assert_eq!(cart.read(0x4000), 0x5a);
        assert_eq!(cart.read_dma(0x4000), 0x5a);
        assert_eq!(cart.read(0x7fff), 0xc3);
        assert_eq!(cart.read(0x8000), 0xa5);
    }

    #[test]
    fn legacy_halt_banked_ram_selects_cpu_and_maria_banks() {
        let mut image = vec![0; 128 + 0xc000];
        image[0] = 3;
        image[1..10].copy_from_slice(b"ATARI7800");
        image[49..53].copy_from_slice(&0xc000u32.to_be_bytes());
        image[53..55].copy_from_slice(&(1u16 << 14).to_be_bytes());
        image[128..].fill(0x77);

        let mut cart = Atari7800Cartridge::parse(&image).unwrap();
        assert!(cart.features.halt_banked_ram);
        assert_eq!(cart.ram.len(), 0x8000);

        cart.write(0x4000, 0x11);
        cart.write(0xc000, 0x22);
        assert_eq!(cart.read(0x4000), 0x11);
        assert_eq!(cart.read_dma(0x4000), 0x22);
        assert_eq!(cart.read(0xc000), 0x77);
        assert_eq!(cart.read_dma(0xc000), 0x77);
    }

    #[test]
    fn v4_bankset_m2_ram_keeps_cpu_and_maria_ram_separate() {
        let mut image = vec![0; 128 + 16 * 0x4000];
        image[0] = 4;
        image[1..10].copy_from_slice(b"ATARI7800");
        image[49..53].copy_from_slice(&(16u32 * 0x4000).to_be_bytes());
        image[64] = 1;
        image[65] = 0x80 | 3;
        for bank in 0..16usize {
            image[128 + bank * 0x4000..128 + (bank + 1) * 0x4000].fill(bank as u8);
        }

        let mut cart = Atari7800Cartridge::parse(&image).unwrap();
        assert!(cart.features.bankset);
        assert!(cart.features.halt_banked_ram);
        assert_eq!(cart.ram.len(), 0x8000);

        cart.write(0x4000, 0x31);
        cart.write(0x7fff, 0x32);
        cart.write(0xc000, 0x41);
        cart.write(0xffff, 0x42);

        assert_eq!(cart.read(0x4000), 0x31);
        assert_eq!(cart.read(0x7fff), 0x32);
        assert_eq!(cart.read_dma(0x4000), 0x41);
        assert_eq!(cart.read_dma(0x7fff), 0x42);
        assert_eq!(cart.read(0xc000), 7);
        assert_eq!(cart.read_dma(0xc000), 15);

        cart.write(0x8000, 3);
        assert_eq!(cart.read(0x8000), 3);
        assert_eq!(cart.read_dma(0x8000), 11);
        assert_eq!(cart.read(0x4000), 0x31);
        assert_eq!(cart.read_dma(0x4000), 0x41);

        let mut out = StateWriter::new(PlatformId::Atari7800, STATE_VERSION);
        cart.save(&mut out);
        let state = out.finish();
        cart.write(0x4000, 0);
        cart.write(0xc000, 0);
        let mut input = StateReader::new(&state, PlatformId::Atari7800, STATE_VERSION).unwrap();
        cart.load(&mut input).unwrap();
        input.finish().unwrap();
        assert_eq!(cart.read(0x4000), 0x31);
        assert_eq!(cart.read_dma(0x4000), 0x41);
    }

    #[test]
    fn linear_bankset_exposes_independent_cpu_and_maria_rom_halves() {
        let mut image = vec![0; 128 + 0x10000];
        image[0] = 3;
        image[1..10].copy_from_slice(b"ATARI7800");
        image[49..53].copy_from_slice(&0x10000u32.to_be_bytes());
        image[53..55].copy_from_slice(&(1u16 << 13).to_be_bytes());
        image[128..128 + 0x8000].fill(0x11);
        image[128 + 0x8000..].fill(0x22);

        let cart = Atari7800Cartridge::parse(&image).unwrap();
        assert!(cart.features.bankset);
        assert_eq!(cart.mapper, Mapper::Linear);
        assert_eq!(cart.read(0x7fff), 0xff);
        assert_eq!(cart.read(0x8000), 0x11);
        assert_eq!(cart.read(0xffff), 0x11);
        assert_eq!(cart.read_dma(0x7fff), 0xff);
        assert_eq!(cart.read_dma(0x8000), 0x22);
        assert_eq!(cart.read_dma(0xffff), 0x22);
    }

    #[test]
    fn bankset_exfix_headers_use_second_last_bank_at_4000() {
        for version in [3u8, 4] {
            let mut image = vec![0; 128 + 16 * 0x4000];
            image[0] = version;
            image[1..10].copy_from_slice(b"ATARI7800");
            image[49..53].copy_from_slice(&(16u32 * 0x4000).to_be_bytes());
            if version >= 4 {
                image[64] = 1;
                image[65] = 0x80 | 5;
            } else {
                let cart_type = (1u16 << 1) | (1u16 << 4) | (1u16 << 13);
                image[53..55].copy_from_slice(&cart_type.to_be_bytes());
            }
            for bank in 0..16usize {
                image[128 + bank * 0x4000..128 + (bank + 1) * 0x4000].fill(bank as u8);
            }

            let cart = Atari7800Cartridge::parse(&image).unwrap();
            assert!(cart.features.bankset);
            assert!(cart.features.second_last_at_4000);
            assert_eq!(cart.mapper, Mapper::SuperGame);
            assert_eq!(cart.read(0x4000), 6);
            assert_eq!(cart.read_dma(0x4000), 14);
            assert_eq!(cart.read(0x8000), 0);
            assert_eq!(cart.read_dma(0x8000), 8);
            assert_eq!(cart.read(0xc000), 7);
            assert_eq!(cart.read_dma(0xc000), 15);
        }
    }

    #[test]
    fn bankset_exrom_profiles_fail_closed_as_invalid_hardware_combinations() {
        for version in [3u8, 4] {
            let mut image = vec![0; 128 + 16 * 0x4000];
            image[0] = version;
            image[1..10].copy_from_slice(b"ATARI7800");
            image[49..53].copy_from_slice(&(16u32 * 0x4000).to_be_bytes());
            if version >= 4 {
                image[64] = 1;
                image[65] = 0x80 | 4;
            } else {
                let cart_type = (1u16 << 1) | (1u16 << 3) | (1u16 << 13);
                image[53..55].copy_from_slice(&cart_type.to_be_bytes());
            }

            let error = match Atari7800Cartridge::parse(&image) {
                Ok(_) => panic!("Bankset + EXROM must be rejected"),
                Err(error) => error,
            };
            assert!(error.contains("does not support EXROM"));
        }
    }

    #[test]
    fn supergame_bankset_keeps_cpu_and_maria_banks_in_lockstep() {
        let mut image = vec![0; 128 + 16 * 0x4000];
        image[0] = 4;
        image[1..10].copy_from_slice(b"ATARI7800");
        image[49..53].copy_from_slice(&(16u32 * 0x4000).to_be_bytes());
        image[64] = 1;
        image[65] = 0x80;
        for bank in 0..16usize {
            image[128 + bank * 0x4000..128 + (bank + 1) * 0x4000].fill(bank as u8);
        }

        let mut cart = Atari7800Cartridge::parse(&image).unwrap();
        assert!(cart.features.bankset);
        assert_eq!(cart.mapper, Mapper::SuperGame);

        assert_eq!(cart.read(0x4000), 6);
        assert_eq!(cart.read(0x8000), 0);
        assert_eq!(cart.read(0xc000), 7);
        assert_eq!(cart.read_dma(0x4000), 14);
        assert_eq!(cart.read_dma(0x8000), 8);
        assert_eq!(cart.read_dma(0xc000), 15);

        cart.write(0x8000, 3);
        assert_eq!(cart.read(0x8000), 3);
        assert_eq!(cart.read_dma(0x8000), 11);

        cart.write(0x8000, 15);
        assert_eq!(cart.read(0x8000), 7);
        assert_eq!(cart.read_dma(0x8000), 15);
    }

    #[test]
    fn activision_mapper_matches_double_dragon_and_rampage_bank_layout() {
        let mut image = vec![0; 128 + 8 * 0x4000];
        image[0] = 4;
        image[1..10].copy_from_slice(b"ATARI7800");
        image[49..53].copy_from_slice(&(8u32 * 0x4000).to_be_bytes());
        image[53..55].copy_from_slice(&(1u16 << 8).to_be_bytes());
        image[64] = 2;
        for bank in 0..8usize {
            let base = 128 + bank * 0x4000;
            image[base..base + 0x2000].fill((bank as u8) << 1);
            image[base + 0x2000..base + 0x4000].fill(((bank as u8) << 1) | 1);
        }

        let mut cart = Atari7800Cartridge::parse(&image).unwrap();
        assert_eq!(cart.read(0x4000), 13);
        assert_eq!(cart.read(0x6000), 12);
        assert_eq!(cart.read(0x8000), 15);
        assert_eq!(cart.read(0xa000), 0);
        assert_eq!(cart.read(0xe000), 14);
        cart.write(0xff85, 0xff);
        assert_eq!(cart.read(0xa000), 10);
        assert_eq!(cart.read(0xdfff), 11);
        cart.write(0xff87, 0);
        assert_eq!(cart.read(0xa000), 14);
    }

    #[test]
    fn absolute_mapper_matches_f18_hornet_bank_layout() {
        let mut image = vec![0; 128 + 4 * 0x4000];
        image[0] = 4;
        image[1..10].copy_from_slice(b"ATARI7800");
        image[49..53].copy_from_slice(&(4u32 * 0x4000).to_be_bytes());
        image[53..55].copy_from_slice(&(1u16 << 9).to_be_bytes());
        image[64] = 3;
        for bank in 0..4usize {
            image[128 + bank * 0x4000..128 + (bank + 1) * 0x4000].fill(bank as u8);
        }

        let mut cart = Atari7800Cartridge::parse(&image).unwrap();
        assert_eq!(cart.read(0x4000), 0);
        assert_eq!(cart.read(0x8000), 2);
        assert_eq!(cart.read(0xc000), 3);
        cart.write(0x8000, 2);
        assert_eq!(cart.read(0x4000), 1);
        cart.write(0x8000, 0xff);
        assert_eq!(cart.read(0x4000), 1);
        cart.write(0x8000, 1);
        assert_eq!(cart.read(0x4000), 0);
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
