use crate::cpu65816::{Bus65816, Cpu65816};
use crate::input::{
    DOWN, FACE_EAST, FACE_NORTH, FACE_SOUTH, FACE_WEST, L1, LEFT, R1, RIGHT, SELECT, START, UP,
};
use crate::kernel::{AudioBuffer, InputState, VideoBuffer};
use crate::machine::Machine;
use crate::platform::PlatformId;
use crate::resources::ResourceKind;
use crate::state::{StateReader, StateWriter};

use super::snes_ppu::SnesPpu;

const FRAME_RATE: f64 = 60.098_813_897;
const STATE_VERSION: u32 = 1;
const WRAM_SIZE: usize = 128 * 1024;
const SAMPLE_RATE: u32 = 48_000;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SnesMap {
    LoRom,
    HiRom,
}

struct SnesCartridge {
    rom: Vec<u8>,
    sram: Vec<u8>,
    map: SnesMap,
}
impl SnesCartridge {
    fn parse(image: &[u8]) -> Result<Self, String> {
        if image.len() < 0x8000 {
            return Err("SNES image is too small".into());
        }
        let rom = if image.len() % 0x8000 == 512 {
            image[512..].to_vec()
        } else {
            image.to_vec()
        };
        let lo_score = Self::header_score(&rom, 0x7fc0, SnesMap::LoRom);
        let hi_score = Self::header_score(&rom, 0xffc0, SnesMap::HiRom);
        let map = if hi_score > lo_score {
            SnesMap::HiRom
        } else {
            SnesMap::LoRom
        };
        let header = match map {
            SnesMap::LoRom => 0x7fc0,
            SnesMap::HiRom => 0xffc0,
        };
        let sram = if header + 0x19 <= rom.len() {
            let exponent = rom[header + 0x18];
            let size = if exponent == 0 {
                0
            } else {
                1024usize.checked_shl(exponent.into()).unwrap_or(0)
            };
            vec![0; size.min(512 * 1024)]
        } else {
            Vec::new()
        };
        Ok(Self { rom, sram, map })
    }

    fn header_score(rom: &[u8], offset: usize, expected: SnesMap) -> i32 {
        if offset + 0x40 > rom.len() {
            return i32::MIN / 2;
        }
        let mode = rom[offset + 0x15];
        let mut score = 0;
        let mode_is_hi = mode & 1 != 0;
        if mode_is_hi == matches!(expected, SnesMap::HiRom) {
            score += 4;
        } else {
            score -= 4;
        }
        let reset = u16::from_le_bytes([rom[offset + 0x3c], rom[offset + 0x3d]]);
        if reset >= 0x8000 {
            score += 3;
        }
        let complement = u16::from_le_bytes([rom[offset + 0x1c], rom[offset + 0x1d]]);
        let checksum = u16::from_le_bytes([rom[offset + 0x1e], rom[offset + 0x1f]]);
        if checksum ^ complement == 0xffff {
            score += 2;
        }
        score
    }

    fn read(&self, address: u32) -> u8 {
        if let Some(index) = self.sram_index(address) {
            return self.sram[index];
        }
        let bank = ((address >> 16) & 0xff) as usize;
        let offset = (address & 0xffff) as usize;
        let index = match self.map {
            SnesMap::LoRom if offset >= 0x8000 => ((bank & 0x7f) * 0x8000) + (offset & 0x7fff),
            SnesMap::HiRom if (0x40..=0x7d).contains(&bank) || bank >= 0xc0 => {
                ((bank & 0x3f) * 0x10000) + offset
            }
            SnesMap::HiRom if offset >= 0x8000 => ((bank & 0x3f) * 0x10000) + offset,
            _ => return 0xff,
        };
        self.rom[index % self.rom.len()]
    }

    fn write(&mut self, address: u32, value: u8) {
        if let Some(index) = self.sram_index(address) {
            self.sram[index] = value;
        }
    }
    fn sram_index(&self, address: u32) -> Option<usize> {
        if self.sram.is_empty() {
            return None;
        }
        let bank = ((address >> 16) & 0xff) as usize;
        let offset = (address & 0xffff) as usize;
        let raw = match self.map {
            SnesMap::LoRom
                if ((0x70..=0x7d).contains(&bank) || bank >= 0xf0) && offset < 0x8000 =>
            {
                ((bank & 0x0f) * 0x8000) + offset
            }
            SnesMap::HiRom
                if ((0x20..=0x3f).contains(&bank) || (0xa0..=0xbf).contains(&bank))
                    && (0x6000..0x8000).contains(&offset) =>
            {
                ((bank & 0x1f) * 0x2000) + (offset - 0x6000)
            }
            _ => return None,
        };
        Some(raw % self.sram.len())
    }

    fn persistent_len(&self) -> usize {
        self.sram.len()
    }
    fn persistent(&self) -> &[u8] {
        &self.sram
    }
    fn set_persistent(&mut self, data: &[u8]) -> Result<(), String> {
        if data.len() != self.sram.len() {
            return Err("SNES SRAM length mismatch".into());
        }
        self.sram.copy_from_slice(data);
        Ok(())
    }
}

struct SnesBus {
    cartridge: SnesCartridge,
    wram: Vec<u8>,
    ppu: SnesPpu,
    apu_ports: [u8; 4],
    dma: [[u8; 16]; 8],
    hdma_enable: u8,
    hdma_active: u8,
    hdma_lines: [u8; 8],
    hdma_repeat: [bool; 8],
    hdma_transfer: [bool; 8],
    nmitimen: u8,
    rdnmi: bool,
    nmi_pending: bool,
    joy_strobe: bool,
    joy_latched: [u16; 2],
    joy_shift: [u16; 2],
    wram_addr: u32,
    dot_phase: u32,
}

impl SnesBus {
    fn new(cartridge: SnesCartridge) -> Self {
        Self {
            cartridge,
            wram: vec![0; WRAM_SIZE],
            ppu: SnesPpu::new(),
            apu_ports: [0; 4],
            dma: [[0; 16]; 8],
            hdma_enable: 0,
            hdma_active: 0,
            hdma_lines: [0; 8],
            hdma_repeat: [false; 8],
            hdma_transfer: [false; 8],
            nmitimen: 0,
            rdnmi: false,
            nmi_pending: false,
            joy_strobe: false,
            joy_latched: [0; 2],
            joy_shift: [0; 2],
            wram_addr: 0,
            dot_phase: 0,
        }
    }

    fn reset(&mut self) {
        self.ppu.reset();
        self.apu_ports = [0; 4];
        self.dma = [[0; 16]; 8];
        self.hdma_enable = 0;
        self.hdma_active = 0;
        self.hdma_lines = [0; 8];
        self.hdma_repeat = [false; 8];
        self.hdma_transfer = [false; 8];
        self.nmitimen = 0;
        self.rdnmi = false;
        self.nmi_pending = false;
        self.joy_strobe = false;
        self.joy_latched = [0; 2];
        self.joy_shift = [0; 2];
        self.wram_addr = 0;
        self.dot_phase = 0;
    }

    fn set_inputs(&mut self, input: &InputState) {
        for player in 0..2 {
            self.joy_latched[player] = snes_buttons(input.buttons[player]);
        }
        if self.joy_strobe {
            self.joy_shift = self.joy_latched;
        }
    }

    fn set_strobe(&mut self, enabled: bool) {
        if enabled || self.joy_strobe {
            self.joy_shift = self.joy_latched;
        }
        self.joy_strobe = enabled;
    }

    fn read_joy(&mut self, player: usize) -> u8 {
        if self.joy_strobe {
            return (self.joy_latched[player] & 1) as u8;
        }
        let value = (self.joy_shift[player] & 1) as u8;
        self.joy_shift[player] = (self.joy_shift[player] >> 1) | 0x8000;
        value
    }
    fn tick_cpu(&mut self, cycles: u32) {
        self.dot_phase = self.dot_phase.saturating_add(cycles.saturating_mul(3));
        let dots = self.dot_phase / 2;
        self.dot_phase %= 2;
        if dots != 0 {
            let before = self.ppu.scanline();
            self.ppu.tick_dots(dots);
            let after = self.ppu.scanline();
            if after != before {
                if after == 0 {
                    self.init_hdma();
                }
                if before < 225 {
                    self.run_hdma_line();
                }
            }
        }
        if self.ppu.take_nmi() {
            self.rdnmi = true;
            if self.nmitimen & 0x80 != 0 {
                self.nmi_pending = true;
            }
        }
    }

    fn take_nmi(&mut self) -> bool {
        let pending = self.nmi_pending;
        self.nmi_pending = false;
        pending
    }

    fn low_bank(bank: u8) -> bool {
        bank <= 0x3f || (0x80..=0xbf).contains(&bank)
    }

    fn wram_index(address: u32) -> Option<usize> {
        let bank = ((address >> 16) & 0xff) as u8;
        let offset = (address & 0xffff) as usize;
        match bank {
            0x7e => Some(offset),
            0x7f => Some(0x10000 + offset),
            _ if Self::low_bank(bank) && offset < 0x2000 => Some(offset),
            _ => None,
        }
    }
    fn read_low_io(&mut self, offset: u16) -> Option<u8> {
        Some(match offset {
            0x2100..=0x213f => self.ppu.read(offset),
            0x2140..=0x2143 => self.apu_ports[usize::from(offset - 0x2140)],
            0x2180 => {
                let value = self.wram[self.wram_addr as usize & (WRAM_SIZE - 1)];
                self.wram_addr = (self.wram_addr + 1) & 0x1ffff;
                value
            }
            0x4016 => self.read_joy(0),
            0x4017 => self.read_joy(1),
            0x4210 => {
                let value = if self.rdnmi { 0x80 } else { 0 };
                self.rdnmi = false;
                value | 0x02
            }
            0x4212 => {
                if self.ppu.in_vblank() {
                    0x80
                } else {
                    0
                }
            }
            0x4218 => self.joy_latched[0] as u8,
            0x4219 => (self.joy_latched[0] >> 8) as u8,
            0x421a => self.joy_latched[1] as u8,
            0x421b => (self.joy_latched[1] >> 8) as u8,
            0x4300..=0x437f => {
                let channel = usize::from((offset - 0x4300) >> 4);
                let reg = usize::from(offset & 0x0f);
                self.dma[channel][reg]
            }
            _ => return None,
        })
    }

    fn write_low_io(&mut self, offset: u16, value: u8) -> bool {
        match offset {
            0x2100..=0x213f => self.ppu.write(offset, value),
            0x2140..=0x2143 => self.apu_ports[usize::from(offset - 0x2140)] = value,
            0x2180 => {
                let index = self.wram_addr as usize & (WRAM_SIZE - 1);
                self.wram[index] = value;
                self.wram_addr = (self.wram_addr + 1) & 0x1ffff;
            }
            0x2181 => self.wram_addr = (self.wram_addr & 0x1ff00) | u32::from(value),
            0x2182 => self.wram_addr = (self.wram_addr & 0x100ff) | (u32::from(value) << 8),
            0x2183 => self.wram_addr = (self.wram_addr & 0x0ffff) | (u32::from(value & 1) << 16),
            0x4016 => self.set_strobe(value & 1 != 0),
            0x4200 => self.nmitimen = value,
            0x420b => self.run_dma(value),
            0x420c => {
                self.hdma_enable = value;
                if self.ppu.scanline() == 0 {
                    self.init_hdma();
                }
            }
            0x4300..=0x437f => {
                let channel = usize::from((offset - 0x4300) >> 4);
                let reg = usize::from(offset & 0x0f);
                self.dma[channel][reg] = value;
            }
            _ => return false,
        }
        true
    }

    fn dma_b_offset(mode: u8, index: usize) -> u8 {
        const PATTERNS: [[u8; 4]; 8] = [
            [0, 0, 0, 0],
            [0, 1, 0, 1],
            [0, 0, 0, 0],
            [0, 0, 1, 1],
            [0, 1, 2, 3],
            [0, 1, 0, 1],
            [0, 0, 0, 0],
            [0, 0, 1, 1],
        ];
        PATTERNS[usize::from(mode & 7)][index & 3]
    }

    fn dma_pattern_len(mode: u8) -> usize {
        match mode & 7 {
            0 => 1,
            1 | 2 | 6 => 2,
            _ => 4,
        }
    }

    fn run_dma(&mut self, mask: u8) {
        for channel in 0..8 {
            if mask & (1 << channel) == 0 {
                continue;
            }
            let regs = self.dma[channel];
            let control = regs[0];
            let bbase = 0x2100u16.wrapping_add(u16::from(regs[1]));
            let bank = u32::from(regs[4]) << 16;
            let mut aoffset = u16::from_le_bytes([regs[2], regs[3]]);
            let encoded = u16::from_le_bytes([regs[5], regs[6]]);
            let count = if encoded == 0 {
                65_536usize
            } else {
                usize::from(encoded)
            };
            let fixed = control & 0x08 != 0;
            let decrement = control & 0x10 != 0;
            let reverse = control & 0x80 != 0;
            let pattern_len = Self::dma_pattern_len(control);
            for index in 0..count {
                let b =
                    bbase.wrapping_add(u16::from(Self::dma_b_offset(control, index % pattern_len)));
                let a = bank | u32::from(aoffset);
                if reverse {
                    let value = self.read8(u32::from(b));
                    self.write8(a, value);
                } else {
                    let value = self.read8(a);
                    self.write8(u32::from(b), value);
                }
                if !fixed {
                    aoffset = if decrement {
                        aoffset.wrapping_sub(1)
                    } else {
                        aoffset.wrapping_add(1)
                    };
                }
            }
            self.dma[channel][2..4].copy_from_slice(&aoffset.to_le_bytes());
            self.dma[channel][5] = 0;
            self.dma[channel][6] = 0;
        }
    }

    fn init_hdma(&mut self) {
        self.hdma_active = self.hdma_enable;
        self.hdma_lines = [0; 8];
        self.hdma_repeat = [false; 8];
        self.hdma_transfer = [false; 8];
        for channel in 0..8 {
            if self.hdma_active & (1 << channel) == 0 {
                continue;
            }
            self.dma[channel][8] = self.dma[channel][2];
            self.dma[channel][9] = self.dma[channel][3];
            self.dma[channel][10] = 0;
        }
    }

    fn hdma_table_read(&mut self, channel: usize) -> u8 {
        let bank = u32::from(self.dma[channel][4]) << 16;
        let address = u16::from_le_bytes([self.dma[channel][8], self.dma[channel][9]]);
        let value = self.read8(bank | u32::from(address));
        let next = address.wrapping_add(1).to_le_bytes();
        self.dma[channel][8] = next[0];
        self.dma[channel][9] = next[1];
        value
    }

    fn reload_hdma_entry(&mut self, channel: usize) -> bool {
        let descriptor = self.hdma_table_read(channel);
        if descriptor == 0 {
            self.hdma_active &= !(1 << channel);
            self.dma[channel][10] = 0;
            return false;
        }
        self.hdma_repeat[channel] = descriptor > 0x80;
        self.hdma_lines[channel] = if descriptor == 0x80 {
            128
        } else {
            descriptor & 0x7f
        };
        self.hdma_transfer[channel] = true;
        self.dma[channel][10] = descriptor;
        if self.dma[channel][0] & 0x40 != 0 {
            let lo = self.hdma_table_read(channel);
            let hi = self.hdma_table_read(channel);
            self.dma[channel][5] = lo;
            self.dma[channel][6] = hi;
        }
        true
    }

    fn hdma_source_read(&mut self, channel: usize, indirect: bool) -> u8 {
        if !indirect {
            return self.hdma_table_read(channel);
        }
        let address = u16::from_le_bytes([self.dma[channel][5], self.dma[channel][6]]);
        let full = (u32::from(self.dma[channel][7]) << 16) | u32::from(address);
        let value = self.read8(full);
        let next = address.wrapping_add(1).to_le_bytes();
        self.dma[channel][5] = next[0];
        self.dma[channel][6] = next[1];
        value
    }

    fn update_hdma_counter_register(&mut self, channel: usize) {
        let remaining = self.hdma_lines[channel];
        self.dma[channel][10] = if remaining == 128 {
            0x80
        } else if self.hdma_repeat[channel] {
            0x80 | remaining
        } else {
            remaining
        };
    }

    fn run_hdma_line(&mut self) {
        for channel in 0..8 {
            if self.hdma_active & (1 << channel) == 0 {
                continue;
            }
            if self.hdma_lines[channel] == 0 && !self.reload_hdma_entry(channel) {
                continue;
            }
            if self.hdma_transfer[channel] {
                self.transfer_hdma_pattern(channel);
            }
            self.hdma_lines[channel] = self.hdma_lines[channel].saturating_sub(1);
            self.hdma_transfer[channel] = self.hdma_repeat[channel];
            self.update_hdma_counter_register(channel);
        }
    }

    fn transfer_hdma_pattern(&mut self, channel: usize) {
        let control = self.dma[channel][0];
        let bbase = 0x2100u16.wrapping_add(u16::from(self.dma[channel][1]));
        let indirect = control & 0x40 != 0;
        let reverse = control & 0x80 != 0;
        let count = Self::dma_pattern_len(control);
        for index in 0..count {
            let b = bbase.wrapping_add(u16::from(Self::dma_b_offset(control, index)));
            if reverse {
                let value = self.read8(u32::from(b));
                if indirect {
                    let address = u16::from_le_bytes([self.dma[channel][5], self.dma[channel][6]]);
                    let full = (u32::from(self.dma[channel][7]) << 16) | u32::from(address);
                    self.write8(full, value);
                    let next = address.wrapping_add(1).to_le_bytes();
                    self.dma[channel][5] = next[0];
                    self.dma[channel][6] = next[1];
                } else {
                    let bank = u32::from(self.dma[channel][4]) << 16;
                    let address = u16::from_le_bytes([self.dma[channel][8], self.dma[channel][9]]);
                    self.write8(bank | u32::from(address), value);
                    let next = address.wrapping_add(1).to_le_bytes();
                    self.dma[channel][8] = next[0];
                    self.dma[channel][9] = next[1];
                }
            } else {
                let value = self.hdma_source_read(channel, indirect);
                self.write8(u32::from(b), value);
            }
        }
    }

    fn save(&self, out: &mut StateWriter) {
        out.blob(&self.wram);
        out.blob(&self.cartridge.sram);
        self.ppu.save(out);
        out.blob(&self.apu_ports);
        for channel in self.dma {
            out.blob(&channel);
        }
        out.u8(self.hdma_enable);
        out.u8(self.hdma_active);
        out.blob(&self.hdma_lines);
        for value in self.hdma_repeat {
            out.u8(value as u8);
        }
        for value in self.hdma_transfer {
            out.u8(value as u8);
        }
        out.u8(self.nmitimen);
        out.u8(self.rdnmi as u8);
        out.u8(self.nmi_pending as u8);
        out.u8(self.joy_strobe as u8);
        out.u16(self.joy_latched[0]);
        out.u16(self.joy_latched[1]);
        out.u16(self.joy_shift[0]);
        out.u16(self.joy_shift[1]);
        out.u32(self.wram_addr);
        out.u32(self.dot_phase);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let wram = input.blob()?;
        if wram.len() != self.wram.len() {
            return Err("invalid SNES WRAM state length".into());
        }
        self.wram.copy_from_slice(wram);
        let sram = input.blob()?;
        if sram.len() != self.cartridge.sram.len() {
            return Err("invalid SNES SRAM state length".into());
        }
        self.cartridge.sram.copy_from_slice(sram);
        self.ppu.load(input)?;
        let ports = input.blob()?;
        if ports.len() != self.apu_ports.len() {
            return Err("invalid SNES APU port state length".into());
        }
        self.apu_ports.copy_from_slice(ports);
        for channel in &mut self.dma {
            let bytes = input.blob()?;
            if bytes.len() != channel.len() {
                return Err("invalid SNES DMA state length".into());
            }
            channel.copy_from_slice(bytes);
        }
        self.hdma_enable = input.u8()?;
        self.hdma_active = input.u8()?;
        let lines = input.blob()?;
        if lines.len() != 8 {
            return Err("invalid SNES HDMA line state length".into());
        }
        self.hdma_lines.copy_from_slice(lines);
        for value in &mut self.hdma_repeat {
            *value = input.u8()? != 0;
        }
        for value in &mut self.hdma_transfer {
            *value = input.u8()? != 0;
        }
        self.nmitimen = input.u8()?;
        self.rdnmi = input.u8()? != 0;
        self.nmi_pending = input.u8()? != 0;
        self.joy_strobe = input.u8()? != 0;
        self.joy_latched = [input.u16()?, input.u16()?];
        self.joy_shift = [input.u16()?, input.u16()?];
        self.wram_addr = input.u32()? & 0x1ffff;
        self.dot_phase = input.u32()? % 2;
        Ok(())
    }
}

impl Bus65816 for SnesBus {
    fn read8(&mut self, address: u32) -> u8 {
        let address = address & 0x00ff_ffff;
        if let Some(index) = Self::wram_index(address) {
            return self.wram[index];
        }
        let bank = ((address >> 16) & 0xff) as u8;
        let offset = (address & 0xffff) as u16;
        if Self::low_bank(bank) {
            if let Some(value) = self.read_low_io(offset) {
                return value;
            }
        }
        self.cartridge.read(address)
    }

    fn write8(&mut self, address: u32, value: u8) {
        let address = address & 0x00ff_ffff;
        if let Some(index) = Self::wram_index(address) {
            self.wram[index] = value;
            return;
        }
        let bank = ((address >> 16) & 0xff) as u8;
        let offset = (address & 0xffff) as u16;
        if Self::low_bank(bank) && self.write_low_io(offset, value) {
            return;
        }
        self.cartridge.write(address, value);
    }
}

fn snes_buttons(buttons: u64) -> u16 {
    let mut out = 0u16;
    let map = [
        (FACE_SOUTH, 0),
        (FACE_WEST, 1),
        (SELECT, 2),
        (START, 3),
        (UP, 4),
        (DOWN, 5),
        (LEFT, 6),
        (RIGHT, 7),
        (FACE_EAST, 8),
        (FACE_NORTH, 9),
        (L1, 10),
        (R1, 11),
    ];
    for (mask, bit) in map {
        if buttons & mask != 0 {
            out |= 1 << bit;
        }
    }
    out
}

pub struct SnesMachine {
    cpu: Cpu65816,
    bus: SnesBus,
    audio: AudioBuffer,
    powered: bool,
}

impl SnesMachine {
    pub fn from_rom(image: &[u8]) -> Result<Self, String> {
        let cartridge = SnesCartridge::parse(image)?;
        let mut bus = SnesBus::new(cartridge);
        let mut cpu = Cpu65816::default();
        cpu.reset(&mut bus);
        Ok(Self {
            cpu,
            bus,
            audio: AudioBuffer::new(SAMPLE_RATE, 2),
            powered: true,
        })
    }

    fn clock_instruction(&mut self) -> u32 {
        let used = self.cpu.step(&mut self.bus);
        if used == 0 {
            return 0;
        }
        self.bus.tick_cpu(used);
        if self.bus.take_nmi() {
            let interrupt = self.cpu.nmi(&mut self.bus);
            self.bus.tick_cpu(interrupt);
        }
        used
    }

    fn silence_frame(&mut self) {
        self.audio.begin_frame();
        let frames = (f64::from(SAMPLE_RATE) / FRAME_RATE).round() as usize;
        for _ in 0..frames {
            self.audio.push_stereo(0.0, 0.0);
        }
    }

    fn save_cpu(&self, out: &mut StateWriter) {
        out.u16(self.cpu.a);
        out.u16(self.cpu.x);
        out.u16(self.cpu.y);
        out.u16(self.cpu.sp);
        out.u16(self.cpu.d);
        out.u16(self.cpu.pc);
        out.u8(self.cpu.pbr);
        out.u8(self.cpu.dbr);
        out.u8(self.cpu.p);
        out.u8(self.cpu.emulation as u8);
        out.u64(self.cpu.cycles);
        out.u8(self.cpu.stopped as u8);
        out.u8(self.cpu.waiting as u8);
    }

    fn load_cpu(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.cpu.a = input.u16()?;
        self.cpu.x = input.u16()?;
        self.cpu.y = input.u16()?;
        self.cpu.sp = input.u16()?;
        self.cpu.d = input.u16()?;
        self.cpu.pc = input.u16()?;
        self.cpu.pbr = input.u8()?;
        self.cpu.dbr = input.u8()?;
        self.cpu.p = input.u8()?;
        self.cpu.emulation = input.u8()? != 0;
        self.cpu.cycles = input.u64()?;
        self.cpu.stopped = input.u8()? != 0;
        self.cpu.waiting = input.u8()? != 0;
        Ok(())
    }
}

impl Machine for SnesMachine {
    fn platform(&self) -> PlatformId {
        PlatformId::Snes
    }

    fn reset(&mut self) {
        self.bus.reset();
        self.cpu.reset(&mut self.bus);
        self.powered = true;
        self.silence_frame();
    }

    fn run_frame(&mut self, input: &InputState) {
        self.bus.set_inputs(input);
        let target = self.bus.ppu.frame().wrapping_add(1);
        let deadline = self.cpu.cycles.saturating_add(200_000);
        while self.bus.ppu.frame() != target && self.cpu.cycles < deadline {
            if self.clock_instruction() == 0 {
                self.powered = false;
                break;
            }
        }
        if self.bus.ppu.frame() != target {
            self.powered = false;
        }
        self.silence_frame();
    }

    fn frame_rate(&self) -> f64 {
        FRAME_RATE
    }
    fn video(&self) -> &VideoBuffer {
        self.bus.ppu.video()
    }
    fn audio(&self) -> &AudioBuffer {
        &self.audio
    }

    fn save_state(&self) -> Result<Vec<u8>, String> {
        let mut out = StateWriter::new(PlatformId::Snes, STATE_VERSION);
        self.save_cpu(&mut out);
        self.bus.save(&mut out);
        out.u8(self.powered as u8);
        Ok(out.finish())
    }

    fn load_state(&mut self, bytes: &[u8]) -> Result<(), String> {
        let mut input = StateReader::new(bytes, PlatformId::Snes, STATE_VERSION)?;
        self.load_cpu(&mut input)?;
        self.bus.load(&mut input)?;
        self.powered = input.u8()? != 0;
        self.silence_frame();
        input.finish()
    }

    fn persistent_len(&self, kind: ResourceKind, slot: u32) -> usize {
        if kind == ResourceKind::Storage && slot == 0 {
            self.bus.cartridge.persistent_len()
        } else {
            0
        }
    }

    fn read_persistent(&self, kind: ResourceKind, slot: u32, out: &mut [u8]) -> Result<(), String> {
        if kind != ResourceKind::Storage || slot != 0 {
            return Err("SNES persistent resource is Storage slot 0".into());
        }
        if out.len() != self.bus.cartridge.persistent_len() {
            return Err("SNES persistent output length mismatch".into());
        }
        out.copy_from_slice(self.bus.cartridge.persistent());
        Ok(())
    }

    fn write_persistent(
        &mut self,
        kind: ResourceKind,
        slot: u32,
        data: &[u8],
    ) -> Result<(), String> {
        if kind != ResourceKind::Storage || slot != 0 {
            return Err("SNES persistent resource is Storage slot 0".into());
        }
        self.bus.cartridge.set_persistent(data)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn synthetic_lorom() -> Vec<u8> {
        let mut rom = vec![0xea; 0x8000];
        let program = [
            0x78, 0xa9, 0x0f, 0x8d, 0x00, 0x21, 0xa9, 0x00, 0x8d, 0x21, 0x21, 0xa9, 0x1f, 0x8d,
            0x22, 0x21, 0xa9, 0x00, 0x8d, 0x22, 0x21, 0x80, 0xfe,
        ];
        rom[..program.len()].copy_from_slice(&program);
        let header = 0x7fc0;
        rom[header + 0x15] = 0x20;
        rom[header + 0x18] = 1;
        rom[header + 0x1c..header + 0x20].copy_from_slice(&[0xff, 0xff, 0x00, 0x00]);
        rom[0x7ffc..0x8000].copy_from_slice(&[0x00, 0x80, 0x00, 0x80]);
        rom
    }

    #[test]
    fn lorom_machine_executes_65816_and_renders_ppu_backdrop() {
        let rom = synthetic_lorom();
        let mut machine = SnesMachine::from_rom(&rom).unwrap();
        machine.run_frame(&InputState::default());
        assert!(machine.powered);
        assert_eq!(machine.video().width(), 256);
        assert_eq!(machine.video().height(), 224);
        assert!(machine.video().pixels()[0] > 200);
        assert_eq!(machine.audio().sample_rate(), SAMPLE_RATE);
    }

    #[test]
    fn controller_serializes_standard_snes_buttons() {
        let rom = synthetic_lorom();
        let mut machine = SnesMachine::from_rom(&rom).unwrap();
        let mut input = InputState::default();
        input.buttons[0] = FACE_SOUTH | START | RIGHT | R1;
        machine.bus.set_inputs(&input);
        machine.bus.set_strobe(true);
        machine.bus.set_strobe(false);
        let mut bits = 0u16;
        for bit in 0..12 {
            bits |= u16::from(machine.bus.read_joy(0)) << bit;
        }
        assert_eq!(
            bits & ((1 << 0) | (1 << 3) | (1 << 7) | (1 << 11)),
            (1 << 0) | (1 << 3) | (1 << 7) | (1 << 11)
        );
    }

    #[test]
    fn state_and_sram_round_trip() {
        let rom = synthetic_lorom();
        let mut machine = SnesMachine::from_rom(&rom).unwrap();
        assert_eq!(machine.persistent_len(ResourceKind::Storage, 0), 2048);
        let data = vec![0x5a; 2048];
        machine
            .write_persistent(ResourceKind::Storage, 0, &data)
            .unwrap();
        machine.run_frame(&InputState::default());
        let saved = machine.save_state().unwrap();
        let pc = machine.cpu.pc;
        machine.bus.wram[0x1234] = 0xa5;
        machine.run_frame(&InputState::default());
        machine.load_state(&saved).unwrap();
        assert_eq!(machine.cpu.pc, pc);
        let mut persisted = vec![0; 2048];
        machine
            .read_persistent(ResourceKind::Storage, 0, &mut persisted)
            .unwrap();
        assert_eq!(persisted, data);
    }

    #[test]
    fn general_dma_moves_wram_into_ppu_b_bus() {
        let rom = synthetic_lorom();
        let mut machine = SnesMachine::from_rom(&rom).unwrap();
        machine.bus.wram[0] = 0x1f;
        machine.bus.wram[1] = 0x00;
        machine.bus.ppu.write(0x2100, 0x0f);
        machine.bus.write_low_io(0x4300, 0x00);
        machine.bus.write_low_io(0x4301, 0x22);
        machine.bus.write_low_io(0x4302, 0x00);
        machine.bus.write_low_io(0x4303, 0x00);
        machine.bus.write_low_io(0x4304, 0x7e);
        machine.bus.write_low_io(0x4305, 0x02);
        machine.bus.write_low_io(0x4306, 0x00);
        machine.bus.write_low_io(0x420b, 0x01);
        machine.bus.ppu.force_render();
        assert!(machine.bus.ppu.video().pixels()[0] > 200);
        assert_eq!(&machine.bus.dma[0][5..7], &[0, 0]);
    }

    #[test]
    fn hdma_repeat_table_updates_cgram_each_line() {
        let rom = synthetic_lorom();
        let mut machine = SnesMachine::from_rom(&rom).unwrap();
        machine.bus.ppu.write(0x2121, 0);
        machine.bus.ppu.write(0x2122, 0x1f);
        machine.bus.ppu.write(0x2122, 0x00);
        let table = [0x82, 0x0f, 0x80, 0x00];
        machine.bus.wram[0x1000..0x1000 + table.len()].copy_from_slice(&table);
        machine.bus.dma[0][0] = 0x00;
        machine.bus.dma[0][1] = 0x00;
        machine.bus.dma[0][2] = 0x00;
        machine.bus.dma[0][3] = 0x10;
        machine.bus.dma[0][4] = 0x7e;
        machine.bus.hdma_enable = 1;
        machine.bus.init_hdma();
        machine.bus.run_hdma_line();
        machine.bus.ppu.force_render();
        assert!(machine.bus.ppu.video().pixels()[0] > 200);
        machine.bus.run_hdma_line();
        machine.bus.ppu.force_render();
        assert_eq!(&machine.bus.ppu.video().pixels()[..3], &[0, 0, 0]);
        assert_eq!(machine.bus.hdma_lines[0], 0);
    }

    #[test]
    fn hirom_maps_full_banks() {
        let mut rom = vec![0; 0x10000];
        rom[0xffd5] = 0x21;
        rom[0xfffc] = 0x00;
        rom[0xfffd] = 0x80;
        rom[0x8000] = 0x42;
        let cart = SnesCartridge::parse(&rom).unwrap();
        assert_eq!(cart.map, SnesMap::HiRom);
        assert_eq!(cart.read(0x008000), 0x42);
    }
}
