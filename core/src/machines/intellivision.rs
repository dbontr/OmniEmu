use crate::clock::{ClockDomain, ClockRate};
use crate::cpu_cp1610::{Cp1610, Cp1610Bus};
use crate::input::{
    AXIS_LEFT_X, AXIS_LEFT_Y, DOWN, FACE_EAST, FACE_NORTH, FACE_SOUTH, KEYPAD_0, KEYPAD_1,
    KEYPAD_2, KEYPAD_3, KEYPAD_4, KEYPAD_5, KEYPAD_6, KEYPAD_7, KEYPAD_8, KEYPAD_9, KEYPAD_CLEAR,
    KEYPAD_ENTER, LEFT, RIGHT, UP,
};
use crate::kernel::{AudioBuffer, InputState, VideoBuffer};
use crate::machine::Machine;
use crate::platform::PlatformId;
use crate::state::{StateReader, StateWriter};

const CPU_CLOCK_HZ: u64 = 894_886;
const FRAME_RATE_NUM: u64 = 5_992;
const FRAME_RATE_DEN: u64 = 100;
const FRAME_RATE: f64 = FRAME_RATE_NUM as f64 / FRAME_RATE_DEN as f64;
const AUDIO_RATE: u32 = 48_000;
const LOGICAL_WIDTH: usize = 160;
const LOGICAL_HEIGHT: usize = 96;
const WIDTH: u32 = 320;
const HEIGHT: u32 = 192;
const STATE_VERSION: u32 = 2;
const ICART_READ: u8 = 0x01;
const ICART_WRITE: u8 = 0x02;
const ICART_NARROW: u8 = 0x04;
const ICART_BANKSW: u8 = 0x08;
const PSG_WRITE_MASKS: [u8; 14] = [
    0xff, 0xff, 0xff, 0xff, 0x0f, 0x0f, 0x0f, 0xff, 0xff, 0x1f, 0x0f, 0x3f, 0x3f, 0x3f,
];
const PSG_VOLUME_LEVELS: [u16; 16] = [
    0, 92, 128, 192, 256, 384, 512, 768, 1024, 1536, 2048, 3072, 4096, 6144, 8192, 10922,
];
const PSG_TICK_DENOMINATOR: u64 = 4 * AUDIO_RATE as u64;

const PALETTE: [[u8; 4]; 16] = [
    [0x00, 0x00, 0x00, 0xff],
    [0x00, 0x2d, 0x9c, 0xff],
    [0xd0, 0x02, 0x18, 0xff],
    [0xc9, 0x7b, 0x47, 0xff],
    [0x00, 0x73, 0x08, 0xff],
    [0x00, 0xa8, 0x16, 0xff],
    [0xfa, 0xea, 0x27, 0xff],
    [0xff, 0xff, 0xff, 0xff],
    [0xa7, 0xa7, 0xa7, 0xff],
    [0x5a, 0xe2, 0xe2, 0xff],
    [0xff, 0x8b, 0x2a, 0xff],
    [0x8a, 0x4a, 0x24, 0xff],
    [0xff, 0x70, 0xa8, 0xff],
    [0x73, 0x83, 0xff, 0xff],
    [0xb7, 0xe3, 0x3a, 0xff],
    [0x9a, 0x4c, 0xb8, 0xff],
];

const DISC_CODES: [u8; 16] = [
    0x04, 0x14, 0x16, 0x06, 0x02, 0x12, 0x13, 0x03, 0x01, 0x11, 0x19, 0x09, 0x08, 0x18, 0x1c, 0x0c,
];

#[derive(Debug, Clone)]
struct Cartridge {
    words: Box<[u16; 65536]>,
    attributes: [u8; 256],
    bank_offsets: [u16; 32],
}

impl Cartridge {
    fn from_bytes(bytes: &[u8]) -> Result<Self, String> {
        let mut cartridge = Self {
            words: Box::new([0xffff; 65536]),
            attributes: [0; 256],
            bank_offsets: [0; 32],
        };
        if bytes.is_empty() {
            return Err("Intellivision cartridge is empty".into());
        }
        if bytes[0] == 0xa8 || (bytes[0] & !0x20) == 0x41 {
            cartridge.load_intellicart(bytes)?;
        } else {
            cartridge.load_flat(bytes)?;
        }
        Ok(cartridge)
    }

    fn load_word(&mut self, address: u16, value: u16) {
        self.words[address as usize] = value;
    }

    fn intellicart_crc16(bytes: &[u8]) -> u16 {
        let mut crc = 0xffffu16;
        for &byte in bytes {
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

    fn load_intellicart(&mut self, bytes: &[u8]) -> Result<(), String> {
        if bytes.len() < 3
            || (bytes[0] != 0xa8 && (bytes[0] & !0x20) != 0x41)
            || bytes[2] != !bytes[1]
        {
            return Err("invalid Intellivision Intellicart header".into());
        }
        let segment_count = bytes[1] as usize;
        if !(1..=32).contains(&segment_count) {
            return Err("Intellicart segment count must be between 1 and 32".into());
        }

        let mut cursor = 3usize;
        for _ in 0..segment_count {
            let segment_start = cursor;
            if cursor + 2 > bytes.len() {
                return Err("truncated Intellicart segment header".into());
            }
            let start_page = bytes[cursor];
            let end_page = bytes[cursor + 1];
            cursor += 2;
            if end_page < start_page {
                return Err("Intellicart segment has a reversed address range".into());
            }
            let start = u16::from(start_page) << 8;
            let words = (usize::from(end_page - start_page) + 1) * 256;
            let data_bytes = words
                .checked_mul(2)
                .ok_or_else(|| "Intellicart segment size overflow".to_string())?;
            let crc_offset = cursor
                .checked_add(data_bytes)
                .ok_or_else(|| "Intellicart segment size overflow".to_string())?;
            if crc_offset + 2 > bytes.len() {
                return Err("truncated Intellicart segment payload".into());
            }
            let expected_crc = u16::from_be_bytes([bytes[crc_offset], bytes[crc_offset + 1]]);
            let actual_crc = Self::intellicart_crc16(&bytes[segment_start..crc_offset]);
            if expected_crc != actual_crc {
                return Err("Intellicart segment CRC mismatch".into());
            }
            for index in 0..words {
                let offset = cursor + index * 2;
                let value = u16::from_be_bytes([bytes[offset], bytes[offset + 1]]);
                self.load_word(start.wrapping_add(index as u16), value);
            }
            cursor = crc_offset + 2;
        }

        if cursor + 50 > bytes.len() {
            return Err("truncated Intellicart attribute table".into());
        }
        let expected_crc = u16::from_be_bytes([bytes[cursor + 48], bytes[cursor + 49]]);
        let actual_crc = Self::intellicart_crc16(&bytes[cursor..cursor + 48]);
        if expected_crc != actual_crc {
            return Err("Intellicart attribute-table CRC mismatch".into());
        }

        for group in 0..32usize {
            let shift = (group & 1) * 4;
            let attr = (bytes[cursor + (group >> 1)] >> shift) & 0x0f;
            let fine_index = (group >> 1) | ((group & 1) << 4);
            let fine = bytes[cursor + 16 + fine_index];
            let low = usize::from((fine >> 4) & 7);
            let high = usize::from(fine & 7) + 1;
            if high < low {
                return Err("Intellicart attribute table has a reversed fine range".into());
            }
            for fine_page in low..high {
                self.attributes[(group << 3) + fine_page] |= attr;
            }
        }
        Ok(())
    }

    fn load_flat(&mut self, bytes: &[u8]) -> Result<(), String> {
        if !bytes.len().is_multiple_of(2) {
            return Err("flat Intellivision cartridge must contain 16-bit words".into());
        }
        let words = bytes.len() / 2;
        if words > 0x4000 {
            return Err("flat Intellivision cartridge exceeds the supported fixed map".into());
        }
        let ranges = [(0x5000u16, 0x2000usize), (0xd000, 0x1000), (0xf000, 0x1000)];
        let mut source = 0usize;
        for (start, capacity) in ranges {
            let count = (words - source).min(capacity);
            for index in 0..count {
                let offset = (source + index) * 2;
                let address = start.wrapping_add(index as u16);
                self.load_word(
                    address,
                    u16::from_be_bytes([bytes[offset], bytes[offset + 1]]),
                );
                self.attributes[usize::from(address >> 8)] |= ICART_READ;
            }
            source += count;
            if source == words {
                break;
            }
        }
        Ok(())
    }

    fn translated_address(&self, address: u16) -> u16 {
        let attr = self.attributes[usize::from(address >> 8)];
        if attr & ICART_BANKSW != 0 {
            address.wrapping_add(self.bank_offsets[usize::from(address >> 11)])
        } else {
            address
        }
    }

    fn read(&self, address: u16) -> Option<u16> {
        let attr = self.attributes[usize::from(address >> 8)];
        if attr & ICART_READ == 0 {
            return None;
        }
        let value = self.words[self.translated_address(address) as usize];
        Some(if attr & ICART_NARROW != 0 {
            value & 0x00ff
        } else {
            value
        })
    }

    fn write(&mut self, address: u16, value: u16) -> bool {
        if (0x0040..=0x005f).contains(&address)
            && self.attributes.iter().any(|attr| attr & ICART_BANKSW != 0)
        {
            let target = ((address & 0x000f) << 12) | ((address & 0x0010) << 7);
            let attr = self.attributes[usize::from(target >> 8)];
            if attr & ICART_BANKSW != 0 {
                let table = usize::from(target >> 11);
                let base = target & !0x0700;
                self.bank_offsets[table] = value.wrapping_shl(8).wrapping_sub(base);
            }
            return true;
        }

        let attr = self.attributes[usize::from(address >> 8)];
        if attr & ICART_WRITE == 0 {
            return false;
        }
        let translated = self.translated_address(address) as usize;
        self.words[translated] = if attr & ICART_NARROW != 0 {
            value & 0x00ff
        } else {
            value
        };
        true
    }

    fn save(&self, out: &mut StateWriter) {
        for offset in self.bank_offsets {
            out.u16(offset);
        }
        let writable_pages = self
            .attributes
            .iter()
            .filter(|attr| **attr & ICART_WRITE != 0)
            .count();
        out.u16(writable_pages as u16);
        for (page, attr) in self.attributes.iter().copied().enumerate() {
            if attr & ICART_WRITE == 0 {
                continue;
            }
            out.u8(page as u8);
            let start = page << 8;
            for value in &self.words[start..start + 0x100] {
                out.u16(*value);
            }
        }
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        for offset in &mut self.bank_offsets {
            *offset = input.u16()?;
        }
        let expected_pages: Vec<usize> = self
            .attributes
            .iter()
            .enumerate()
            .filter_map(|(page, attr)| (attr & ICART_WRITE != 0).then_some(page))
            .collect();
        let page_count = usize::from(input.u16()?);
        if page_count != expected_pages.len() {
            return Err(
                "Intellivision state cartridge RAM map differs from loaded cartridge".into(),
            );
        }
        for expected_page in expected_pages {
            let page = usize::from(input.u8()?);
            if page != expected_page {
                return Err(
                    "Intellivision state cartridge RAM pages differ from loaded cartridge".into(),
                );
            }
            let start = page << 8;
            for value in &mut self.words[start..start + 0x100] {
                *value = input.u16()?;
            }
        }
        Ok(())
    }
}

struct IntellivisionBus {
    stic: [u16; 0x40],
    scratch: [u8; 0xf0],
    psg: [u8; 0x10],
    system_ram: [u16; 0x160],
    exec: [u16; 0x1000],
    grom: [u8; 0x800],
    gram: [u8; 0x200],
    cartridge: Cartridge,
    controllers: [u8; 2],
    display_enabled: bool,
    color_stack_mode: bool,
    psg_envelope_generation: u64,
}

impl IntellivisionBus {
    fn new(game: &[u8], exec: &[u8], grom: &[u8]) -> Result<Self, String> {
        if exec.len() != 0x2000 {
            return Err(format!(
                "Intellivision EXEC must be exactly 8192 bytes, got {}",
                exec.len()
            ));
        }
        if grom.len() != 0x0800 {
            return Err(format!(
                "Intellivision GROM must be exactly 2048 bytes, got {}",
                grom.len()
            ));
        }
        let mut exec_words = [0u16; 0x1000];
        for (index, word) in exec_words.iter_mut().enumerate() {
            *word = u16::from_be_bytes([exec[index * 2], exec[index * 2 + 1]]);
        }
        let mut grom_bytes = [0u8; 0x800];
        grom_bytes.copy_from_slice(grom);
        let mut stic = [0u16; 0x40];
        stic[0x28..=0x2c].fill(0);
        Ok(Self {
            stic,
            scratch: [0; 0xf0],
            psg: [0; 0x10],
            system_ram: [0; 0x160],
            exec: exec_words,
            grom: grom_bytes,
            gram: [0; 0x200],
            cartridge: Cartridge::from_bytes(game)?,
            controllers: [0xff; 2],
            display_enabled: false,
            color_stack_mode: true,
            psg_envelope_generation: 0,
        })
    }

    fn set_inputs(&mut self, input: &InputState) {
        for player in 0..2 {
            let buttons = input.buttons[player];
            let mut asserted = 0u8;
            if buttons & UP != 0 {
                asserted |= 0x04;
            }
            if buttons & DOWN != 0 {
                asserted |= 0x01;
            }
            if buttons & LEFT != 0 {
                asserted |= 0x08;
            }
            if buttons & RIGHT != 0 {
                asserted |= 0x02;
            }
            if buttons & FACE_NORTH != 0 {
                asserted |= 0xa0;
            }
            if buttons & FACE_SOUTH != 0 {
                asserted |= 0x60;
            }
            if buttons & FACE_EAST != 0 {
                asserted |= 0xc0;
            }
            let keypad = [
                (KEYPAD_1, 0x81),
                (KEYPAD_2, 0x41),
                (KEYPAD_3, 0x21),
                (KEYPAD_4, 0x82),
                (KEYPAD_5, 0x42),
                (KEYPAD_6, 0x22),
                (KEYPAD_7, 0x84),
                (KEYPAD_8, 0x44),
                (KEYPAD_9, 0x24),
                (KEYPAD_CLEAR, 0x88),
                (KEYPAD_0, 0x48),
                (KEYPAD_ENTER, 0x28),
            ];
            for (button, code) in keypad {
                if buttons & button != 0 {
                    asserted |= code;
                }
            }

            let x = input.axes[player][AXIS_LEFT_X];
            let y = input.axes[player][AXIS_LEFT_Y];
            if x.unsigned_abs() > 7000 || y.unsigned_abs() > 7000 {
                let angle = (f64::from(y)).atan2(f64::from(x));
                let clockwise_from_north =
                    (angle + std::f64::consts::FRAC_PI_2).rem_euclid(std::f64::consts::TAU);
                let index = ((clockwise_from_north / std::f64::consts::TAU) * 16.0 + 0.5).floor()
                    as usize
                    & 15;
                asserted |= DISC_CODES[index];
            }
            self.controllers[player] = !asserted;
        }
        self.psg[0x0e] = self.controllers[1];
        self.psg[0x0f] = self.controllers[0];
    }

    fn read_stic(&mut self, register: usize) -> u16 {
        if register == 0x21 {
            self.color_stack_mode = true;
        }
        match register {
            0x00..=0x07 => self.stic[register] | 0x3800,
            0x08..=0x0f => self.stic[register] | 0x3000,
            0x10..=0x17 => self.stic[register] & 0x3fff,
            0x18..=0x1f => {
                let self_bit = 1u16 << (register - 0x18);
                (self.stic[register] & 0x03ff & !self_bit) | 0x3c00
            }
            0x20..=0x27 | 0x2d..=0x2f | 0x33..=0x3f => 0x3fff,
            0x28..=0x2c => self.stic[register] | 0x3ff0,
            0x30..=0x31 => self.stic[register] | 0x3ff8,
            0x32 => self.stic[register] | 0x3ffc,
            _ => self.stic[register],
        }
    }

    fn write_stic(&mut self, register: usize, value: u16) {
        match register {
            0x00..=0x07 => self.stic[register] = value & 0x07ff,
            0x08..=0x0f => self.stic[register] = value & 0x0fff,
            0x10..=0x17 => self.stic[register] = value & 0x3fff,
            0x18..=0x1f => {
                let self_bit = 1u16 << (register - 0x18);
                self.stic[register] = value & 0x03ff & !self_bit;
            }
            0x20 => self.display_enabled = true,
            0x21 => self.color_stack_mode = false,
            0x22..=0x27 | 0x2d..=0x2f | 0x33..=0x3f => {}
            0x28..=0x2c => self.stic[register] = value & 0x000f,
            0x30..=0x31 => self.stic[register] = value & 0x0007,
            0x32 => self.stic[register] = value & 0x0003,
            _ => self.stic[register] = value,
        }
    }

    fn save(&self, out: &mut StateWriter) {
        for value in self.stic {
            out.u16(value);
        }
        out.blob(&self.scratch);
        out.blob(&self.psg);
        for value in self.system_ram {
            out.u16(value);
        }
        out.blob(&self.gram);
        self.cartridge.save(out);
        out.u8(self.controllers[0]);
        out.u8(self.controllers[1]);
        out.u8(u8::from(self.display_enabled));
        out.u8(u8::from(self.color_stack_mode));
        out.u64(self.psg_envelope_generation);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        for value in &mut self.stic {
            *value = input.u16()?;
        }
        let scratch = input.blob()?;
        if scratch.len() != self.scratch.len() {
            return Err("invalid Intellivision scratch RAM state".into());
        }
        self.scratch.copy_from_slice(scratch);
        let psg = input.blob()?;
        if psg.len() != self.psg.len() {
            return Err("invalid Intellivision PSG state".into());
        }
        self.psg.copy_from_slice(psg);
        for value in &mut self.system_ram {
            *value = input.u16()?;
        }
        let gram = input.blob()?;
        if gram.len() != self.gram.len() {
            return Err("invalid Intellivision GRAM state".into());
        }
        self.gram.copy_from_slice(gram);
        self.cartridge.load(input)?;
        self.controllers[0] = input.u8()?;
        self.controllers[1] = input.u8()?;
        self.display_enabled = input.u8()? != 0;
        self.color_stack_mode = input.u8()? != 0;
        self.psg_envelope_generation = input.u64()?;
        Ok(())
    }
}

impl Cp1610Bus for IntellivisionBus {
    fn read(&mut self, address: u16) -> u16 {
        match address {
            0x0000..=0x003f => self.read_stic(address as usize),
            0x0100..=0x01ef => u16::from(self.scratch[address as usize - 0x0100]),
            0x01f0..=0x01ff => u16::from(self.psg[address as usize - 0x01f0]),
            0x0200..=0x035f => self.system_ram[address as usize - 0x0200],
            0x1000..=0x1fff => self.exec[address as usize - 0x1000],
            0x3000..=0x37ff => u16::from(self.grom[address as usize - 0x3000]),
            0x3800..=0x39ff => u16::from(self.gram[address as usize & 0x01ff]),
            0x7800..=0x79ff | 0xb800..=0xb9ff | 0xf800..=0xf9ff => self
                .cartridge
                .read(address)
                .unwrap_or_else(|| u16::from(self.gram[address as usize & 0x01ff])),
            _ => self.cartridge.read(address).unwrap_or(0xffff),
        }
    }

    fn write(&mut self, address: u16, value: u16) {
        match address {
            0x0000..=0x003f => self.write_stic(address as usize, value),
            0x0100..=0x01ef => self.scratch[address as usize - 0x0100] = value as u8,
            0x01f0..=0x01fd => {
                let register = address as usize - 0x01f0;
                self.psg[register] = value as u8 & PSG_WRITE_MASKS[register];
                if register == 0x0a {
                    self.psg_envelope_generation = self.psg_envelope_generation.wrapping_add(1);
                }
            }
            0x01fe..=0x01ff => self.psg[address as usize - 0x01f0] = value as u8,
            0x0200..=0x035f => self.system_ram[address as usize - 0x0200] = value,
            0x3800..=0x39ff => self.gram[address as usize & 0x01ff] = value as u8,
            0x7800..=0x79ff | 0xb800..=0xb9ff | 0xf800..=0xf9ff => {
                if !self.cartridge.write(address, value) && self.cartridge.read(address).is_none() {
                    self.gram[address as usize & 0x01ff] = value as u8;
                }
            }
            _ => {
                let _ = self.cartridge.write(address, value);
            }
        }
    }
}

#[derive(Debug, Clone)]
struct PsgState {
    tick_phase: u64,
    sample_phase: u64,
    tone_count: [i32; 3],
    tone_output: [bool; 3],
    noise_count: i32,
    noise_lfsr: u32,
    envelope_count: i32,
    envelope_level: i8,
    envelope_step: i8,
    envelope_generation: u64,
}

impl Default for PsgState {
    fn default() -> Self {
        Self {
            tick_phase: 0,
            sample_phase: 0,
            tone_count: [0; 3],
            tone_output: [false; 3],
            noise_count: 0,
            noise_lfsr: 0x10004,
            envelope_count: 0,
            envelope_level: 0,
            envelope_step: 0,
            envelope_generation: 0,
        }
    }
}

impl PsgState {
    fn tone_period(registers: &[u8; 0x10], channel: usize) -> i32 {
        let raw = u16::from(registers[channel]) | (u16::from(registers[4 + channel] & 0x0f) << 8);
        if raw == 0 {
            0x1000
        } else {
            i32::from(raw)
        }
    }

    fn noise_period(registers: &[u8; 0x10]) -> i32 {
        let raw = i32::from(registers[9] & 0x1f) * 2;
        if raw == 0 {
            0x40
        } else {
            raw
        }
    }

    fn envelope_period(registers: &[u8; 0x10]) -> i32 {
        let raw = (i32::from(registers[3]) | (i32::from(registers[7]) << 8)) * 2;
        if raw == 0 {
            0x20000
        } else {
            raw
        }
    }

    fn trigger_envelope(&mut self, registers: &[u8; 0x10], generation: u64) {
        if self.envelope_generation == generation {
            return;
        }
        self.envelope_generation = generation;
        let attack = registers[10] & 0x04 != 0;
        self.envelope_level = if attack { 0 } else { 15 };
        self.envelope_step = if attack { 1 } else { -1 };
        self.envelope_count = Self::envelope_period(registers);
    }

    fn advance_envelope(&mut self, shape: u8) {
        if self.envelope_step == 0 {
            return;
        }
        self.envelope_level += self.envelope_step;
        if (0..=15).contains(&self.envelope_level) {
            return;
        }

        let continue_envelope = shape & 0x08 != 0;
        let attack = shape & 0x04 != 0;
        let alternate = shape & 0x02 != 0;
        let hold = shape & 0x01 != 0;
        if hold {
            self.envelope_step = 0;
            self.envelope_level = if alternate {
                if attack {
                    0
                } else {
                    15
                }
            } else if attack {
                15
            } else {
                0
            };
        } else if alternate {
            self.envelope_step = -self.envelope_step;
            self.envelope_level = (self.envelope_level + self.envelope_step).rem_euclid(16);
        } else {
            self.envelope_level = if attack { 0 } else { 15 };
        }
        if !continue_envelope {
            self.envelope_level = 0;
            self.envelope_step = 0;
        }
    }

    fn tick(&mut self, registers: &[u8; 0x10]) {
        for channel in 0..3 {
            self.tone_count[channel] -= 1;
            if self.tone_count[channel] <= 0 {
                self.tone_output[channel] = !self.tone_output[channel];
                self.tone_count[channel] += Self::tone_period(registers, channel);
            }
        }

        self.noise_count -= 1;
        if self.noise_count <= 0 {
            let feedback = if self.noise_lfsr & 1 != 0 { 0x10004 } else { 0 };
            self.noise_lfsr = (self.noise_lfsr >> 1) ^ feedback;
            self.noise_count += Self::noise_period(registers);
        }

        if self.envelope_count > 0 {
            self.envelope_count -= 1;
            if self.envelope_count == 0 {
                self.advance_envelope(registers[10] & 0x0f);
                self.envelope_count = Self::envelope_period(registers);
            }
        }
    }

    fn sample(&self, registers: &[u8; 0x10]) -> f32 {
        let mut sum = 0u32;
        for channel in 0..3 {
            let tone_open = registers[8] & (1 << channel) != 0 || self.tone_output[channel];
            let noise_open = registers[8] & (1 << (channel + 3)) != 0 || self.noise_lfsr & 1 != 0;
            if !(tone_open && noise_open) {
                continue;
            }
            let volume = registers[11 + channel];
            let envelope = (volume >> 4) & 0x03;
            let level = match envelope {
                0 => volume & 0x0f,
                1 => (self.envelope_level.max(0) as u8) >> 2,
                2 => (self.envelope_level.max(0) as u8) >> 1,
                _ => self.envelope_level.max(0) as u8,
            };
            sum += u32::from(PSG_VOLUME_LEVELS[usize::from(level & 0x0f)]);
        }
        (sum as f32 / 32768.0).clamp(0.0, 1.0)
    }

    fn save(&self, out: &mut StateWriter) {
        out.u64(self.tick_phase);
        out.u64(self.sample_phase);
        for count in self.tone_count {
            out.u32(count as u32);
        }
        for output in self.tone_output {
            out.u8(u8::from(output));
        }
        out.u32(self.noise_count as u32);
        out.u32(self.noise_lfsr);
        out.u32(self.envelope_count as u32);
        out.u8(self.envelope_level as u8);
        out.u8(self.envelope_step as u8);
        out.u64(self.envelope_generation);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.tick_phase = input.u64()?;
        self.sample_phase = input.u64()?;
        for count in &mut self.tone_count {
            *count = input.u32()? as i32;
        }
        for output in &mut self.tone_output {
            *output = input.u8()? != 0;
        }
        self.noise_count = input.u32()? as i32;
        self.noise_lfsr = input.u32()?;
        self.envelope_count = input.u32()? as i32;
        self.envelope_level = input.u8()? as i8;
        self.envelope_step = input.u8()? as i8;
        self.envelope_generation = input.u64()?;
        Ok(())
    }
}

pub struct IntellivisionMachine {
    cpu: Cp1610,
    bus: IntellivisionBus,
    video: VideoBuffer,
    audio: AudioBuffer,
    psg_state: PsgState,
    frame_clock: ClockDomain,
    powered: bool,
}

impl IntellivisionMachine {
    pub fn from_images(game: &[u8], exec: &[u8], grom: &[u8]) -> Result<Self, String> {
        Ok(Self {
            cpu: Cp1610::default(),
            bus: IntellivisionBus::new(game, exec, grom)?,
            video: VideoBuffer::new(WIDTH, HEIGHT),
            audio: AudioBuffer::new(AUDIO_RATE, 2),
            psg_state: PsgState::default(),
            frame_clock: ClockDomain::new(
                ClockRate::rational(FRAME_RATE_NUM, FRAME_RATE_DEN),
                ClockRate::hz(CPU_CLOCK_HZ),
            ),
            powered: true,
        })
    }

    fn pattern_byte(&self, gram: bool, card: usize, row: usize) -> u8 {
        if gram {
            self.bus.gram[(card & 0x3f) * 8 + (row & 7)]
        } else {
            self.bus.grom[(card & 0xff) * 8 + (row & 7)]
        }
    }

    fn background_colors(&self, word: u16, stack: &mut usize) -> (usize, usize, bool, usize) {
        if self.bus.color_stack_mode {
            if word & 0x2000 != 0 {
                *stack = (*stack + 1) & 3;
            }
            let foreground = usize::from((word & 7) | ((word >> 8) & 8));
            let background = usize::from(self.bus.stic[0x28 + *stack] & 0x0f);
            let gram = word & 0x0800 != 0;
            let card = usize::from((word >> 3) & if gram { 0x3f } else { 0xff });
            (foreground, background, gram, card)
        } else {
            let foreground = usize::from(word & 7);
            let background = usize::from(
                ((word >> 9) & 1)
                    | (((word >> 10) & 1) << 1)
                    | (((word >> 13) & 1) << 2)
                    | (((word >> 12) & 1) << 3),
            );
            let gram = word & 0x0800 != 0;
            let card = usize::from((word >> 3) & 0x3f);
            (foreground, background, gram, card)
        }
    }

    fn render(&mut self) {
        let border = usize::from(self.bus.stic[0x2c] & 0x0f);
        self.video.clear(PALETTE[border]);
        if !self.bus.display_enabled {
            return;
        }
        let mut logical = vec![border as u8; LOGICAL_WIDTH * LOGICAL_HEIGHT];
        let mut foreground_mask = vec![false; logical.len()];
        let mut stack = 0usize;
        for card_y in 0..12 {
            for card_x in 0..20 {
                let word = self.bus.system_ram[card_y * 20 + card_x];
                let (foreground, background, gram, card) = self.background_colors(word, &mut stack);
                for row in 0..8 {
                    let pattern = self.pattern_byte(gram, card, row);
                    for column in 0..8 {
                        let index = (card_y * 8 + row) * LOGICAL_WIDTH + card_x * 8 + column;
                        let set = pattern & (0x80 >> column) != 0;
                        logical[index] = if set {
                            foreground as u8
                        } else {
                            background as u8
                        };
                        foreground_mask[index] = set;
                    }
                }
            }
        }

        let mut rectangles = [(0usize, 0usize, 0usize, 0usize, false); 8];
        for (mob, rectangle) in rectangles.iter_mut().enumerate() {
            let xreg = self.bus.stic[mob];
            let yreg = self.bus.stic[8 + mob];
            let attr = self.bus.stic[16 + mob];
            if xreg & 0x0200 == 0 {
                continue;
            }
            let x = usize::from(xreg & 0xff).min(LOGICAL_WIDTH - 1);
            let y = usize::from(yreg & 0x7f).min(LOGICAL_HEIGHT - 1);
            let xscale = if xreg & 0x0400 != 0 { 2 } else { 1 };
            let yscale = 1usize << usize::from((yreg >> 8) & 0x03);
            let width = 8 * xscale;
            let height = 8 * yscale;
            let gram = attr & 0x0800 != 0;
            let card = usize::from((attr >> 3) & if gram { 0x3f } else { 0xff });
            let color = usize::from((attr & 7) | ((attr >> 8) & 8));
            let xflip = yreg & 0x0400 != 0;
            let yflip = yreg & 0x0800 != 0;
            let behind_foreground = attr & 0x2000 != 0;
            for row in 0..8 {
                let pattern_row = if yflip { 7 - row } else { row };
                let pattern = self.pattern_byte(gram, card, pattern_row);
                for column in 0..8 {
                    let pattern_column = if xflip { 7 - column } else { column };
                    if pattern & (0x80 >> pattern_column) == 0 {
                        continue;
                    }
                    for sy in 0..yscale {
                        for sx in 0..xscale {
                            let px = x + column * xscale + sx;
                            let py = y + row * yscale + sy;
                            if px >= LOGICAL_WIDTH || py >= LOGICAL_HEIGHT {
                                continue;
                            }
                            let index = py * LOGICAL_WIDTH + px;
                            if behind_foreground && foreground_mask[index] {
                                continue;
                            }
                            logical[index] = color as u8;
                        }
                    }
                }
            }
            *rectangle = (x, y, width, height, xreg & 0x0100 != 0);
        }

        for a in 0..8 {
            if !rectangles[a].4 {
                continue;
            }
            for b in (a + 1)..8 {
                if !rectangles[b].4 {
                    continue;
                }
                let (ax, ay, aw, ah, _) = rectangles[a];
                let (bx, by, bw, bh, _) = rectangles[b];
                if ax < bx + bw && ax + aw > bx && ay < by + bh && ay + ah > by {
                    self.bus.stic[0x18 + a] |= 1 << b;
                    self.bus.stic[0x18 + b] |= 1 << a;
                }
            }
        }

        let pixels = self.video.pixels_mut();
        for y in 0..LOGICAL_HEIGHT {
            for x in 0..LOGICAL_WIDTH {
                let color = PALETTE[logical[y * LOGICAL_WIDTH + x] as usize & 15];
                for dy in 0..2 {
                    for dx in 0..2 {
                        let output_x = x * 2 + dx;
                        let output_y = y * 2 + dy;
                        let offset = (output_y * WIDTH as usize + output_x) * 4;
                        pixels[offset..offset + 4].copy_from_slice(&color);
                    }
                }
            }
        }
    }

    fn synthesize_audio(&mut self) {
        self.audio.begin_frame();
        self.psg_state
            .trigger_envelope(&self.bus.psg, self.bus.psg_envelope_generation);
        self.psg_state.sample_phase += u64::from(AUDIO_RATE) * FRAME_RATE_DEN;
        let samples = self.psg_state.sample_phase / FRAME_RATE_NUM;
        self.psg_state.sample_phase %= FRAME_RATE_NUM;
        for _ in 0..samples {
            self.psg_state.tick_phase += CPU_CLOCK_HZ;
            while self.psg_state.tick_phase >= PSG_TICK_DENOMINATOR {
                self.psg_state.tick_phase -= PSG_TICK_DENOMINATOR;
                self.psg_state.tick(&self.bus.psg);
            }
            let sample = self.psg_state.sample(&self.bus.psg);
            self.audio.push_stereo(sample, sample);
        }
    }
}

impl Machine for IntellivisionMachine {
    fn platform(&self) -> PlatformId {
        PlatformId::Intellivision
    }

    fn reset(&mut self) {
        self.cpu.reset();
        self.bus.scratch.fill(0);
        self.bus.system_ram.fill(0);
        self.bus.gram.fill(0);
        self.bus.stic.fill(0);
        self.bus.psg.fill(0);
        self.bus.display_enabled = false;
        self.bus.color_stack_mode = true;
        self.bus.psg_envelope_generation = 0;
        self.psg_state = PsgState::default();
        self.frame_clock = ClockDomain::new(
            ClockRate::rational(FRAME_RATE_NUM, FRAME_RATE_DEN),
            ClockRate::hz(CPU_CLOCK_HZ),
        );
        self.powered = true;
        self.video.clear(PALETTE[0]);
        self.audio.begin_frame();
    }

    fn run_frame(&mut self, input: &InputState) {
        self.bus.set_inputs(input);
        let _ = self.cpu.interrupt(&mut self.bus);
        self.frame_clock.advance(1);
        let deadline = self.frame_clock.cycles();
        while self.cpu.cycles < deadline {
            self.cpu.step(&mut self.bus);
        }
        self.render();
        self.synthesize_audio();
    }

    fn frame_rate(&self) -> f64 {
        FRAME_RATE
    }
    fn video(&self) -> &VideoBuffer {
        &self.video
    }
    fn audio(&self) -> &AudioBuffer {
        &self.audio
    }

    fn save_state(&self) -> Result<Vec<u8>, String> {
        let mut out = StateWriter::new(PlatformId::Intellivision, STATE_VERSION);
        self.cpu.save(&mut out);
        self.bus.save(&mut out);
        self.psg_state.save(&mut out);
        let (frame_phase, _) = self.frame_clock.phase();
        out.u128(frame_phase);
        out.u64(self.frame_clock.cycles());
        out.u8(u8::from(self.powered));
        Ok(out.finish())
    }

    fn load_state(&mut self, bytes: &[u8]) -> Result<(), String> {
        let mut input = StateReader::new(bytes, PlatformId::Intellivision, STATE_VERSION)?;
        self.cpu.load(&mut input)?;
        self.bus.load(&mut input)?;
        self.psg_state.load(&mut input)?;
        let frame_phase = input.u128()?;
        let frame_cycles = input.u64()?;
        self.frame_clock.restore(frame_phase, frame_cycles)?;
        self.powered = input.u8()? != 0;
        input.finish()?;
        self.render();
        self.audio.begin_frame();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn build_intellicart(
        segments: &[(u8, Vec<u16>)],
        attributes: &[(usize, u8, usize, usize)],
    ) -> Vec<u8> {
        assert!(!segments.is_empty() && segments.len() <= 32);
        let mut rom = vec![0xa8, segments.len() as u8, !(segments.len() as u8)];
        for (page, words) in segments {
            assert_eq!(words.len(), 0x100);
            let start = rom.len();
            rom.push(*page);
            rom.push(*page);
            for word in words {
                rom.extend_from_slice(&word.to_be_bytes());
            }
            let crc = Cartridge::intellicart_crc16(&rom[start..]);
            rom.extend_from_slice(&crc.to_be_bytes());
        }

        let mut table = [0u8; 48];
        for &(group, attr, low, high) in attributes {
            assert!(group < 32 && low <= high && high < 8 && attr <= 0x0f);
            let shift = (group & 1) * 4;
            table[group >> 1] |= attr << shift;
            let fine_index = (group >> 1) | ((group & 1) << 4);
            table[16 + fine_index] = ((low as u8) << 4) | high as u8;
        }
        let crc = Cartridge::intellicart_crc16(&table);
        rom.extend_from_slice(&table);
        rom.extend_from_slice(&crc.to_be_bytes());
        rom
    }

    fn synthetic_images() -> (Vec<u8>, Vec<u8>, Vec<u8>) {
        let mut exec = vec![0u8; 0x2000];
        let startup = [0x0004u16, 0x0050, 0x0000];
        for (index, word) in startup.into_iter().enumerate() {
            exec[index * 2..index * 2 + 2].copy_from_slice(&word.to_be_bytes());
        }
        let mut grom = vec![0u8; 0x800];
        for (row, byte) in grom.iter_mut().take(8).enumerate() {
            *byte = if row == 0 || row == 7 { 0xff } else { 0x81 };
        }
        let program = [
            0x02b8u16, 0x0001, 0x0240, 0x0020, 0x02b8, 0x0007, 0x0240, 0x002c, 0x02b8, 0x0007,
            0x0240, 0x0200, 0x0000,
        ];
        let mut game = Vec::with_capacity(program.len() * 2);
        for word in program {
            game.extend_from_slice(&word.to_be_bytes());
        }
        (game, exec, grom)
    }

    #[test]
    fn synthetic_cartridge_runs_cp1610_and_stic_graph() {
        let (game, exec, grom) = synthetic_images();
        let mut machine = IntellivisionMachine::from_images(&game, &exec, &grom).unwrap();
        machine.run_frame(&InputState::default());
        assert_eq!(machine.cpu.pc(), 0x500d);
        assert!(machine.bus.display_enabled);
        assert_eq!(machine.video().width(), WIDTH);
        assert_eq!(machine.video().height(), HEIGHT);
        assert!(machine.video().pixels().iter().any(|byte| *byte != 0));
        assert!(!machine.audio().samples().is_empty());
    }

    #[test]
    fn controller_encoding_is_active_low_and_uses_real_keypad_masks() {
        let (game, exec, grom) = synthetic_images();
        let mut machine = IntellivisionMachine::from_images(&game, &exec, &grom).unwrap();
        let cases = [
            (KEYPAD_1, 0x81),
            (KEYPAD_2, 0x41),
            (KEYPAD_3, 0x21),
            (KEYPAD_4, 0x82),
            (KEYPAD_5, 0x42),
            (KEYPAD_6, 0x22),
            (KEYPAD_7, 0x84),
            (KEYPAD_8, 0x44),
            (KEYPAD_9, 0x24),
            (KEYPAD_CLEAR, 0x88),
            (KEYPAD_0, 0x48),
            (KEYPAD_ENTER, 0x28),
        ];
        for (button, code) in cases {
            let mut input = InputState::default();
            input.buttons[0] = button;
            machine.bus.set_inputs(&input);
            assert_eq!(machine.bus.psg[0x0f], !code);
            assert_eq!(machine.bus.psg[0x0e], 0xff);
        }

        let mut input = InputState::default();
        input.buttons[0] = UP | KEYPAD_5 | KEYPAD_ENTER;
        machine.bus.set_inputs(&input);
        assert_eq!(machine.bus.psg[0x0f], !(0x04 | 0x42 | 0x28));
    }

    #[test]
    fn intellicart_segment_crc_attributes_and_trailing_tags_are_honored() {
        let words: Vec<u16> = (0..256u16).collect();
        let mut rom = build_intellicart(&[(0x50, words)], &[(10, ICART_READ, 0, 0)]);
        rom.extend_from_slice(b"metadata-tag");

        let cart = Cartridge::from_bytes(&rom).unwrap();
        assert_eq!(cart.read(0x5000), Some(0));
        assert_eq!(cart.read(0x50ff), Some(255));
        assert_eq!(cart.read(0x5100), None);

        let mut bad_segment = rom.clone();
        bad_segment[5] ^= 1;
        assert!(Cartridge::from_bytes(&bad_segment)
            .unwrap_err()
            .contains("segment CRC"));

        let mut bad_table =
            build_intellicart(&[(0x50, (0..256u16).collect())], &[(10, ICART_READ, 0, 0)]);
        let table_byte = bad_table.len() - 50;
        bad_table[table_byte] ^= 1;
        assert!(Cartridge::from_bytes(&bad_table)
            .unwrap_err()
            .contains("attribute-table CRC"));
    }

    #[test]
    fn intellicart_writable_narrow_memory_round_trips_through_machine_state() {
        let rom = build_intellicart(
            &[(0xd0, vec![0x1234; 0x100])],
            &[(26, ICART_READ | ICART_WRITE | ICART_NARROW, 0, 0)],
        );
        let (_, exec, grom) = synthetic_images();
        let mut machine = IntellivisionMachine::from_images(&rom, &exec, &grom).unwrap();

        assert_eq!(machine.bus.read(0xd000), 0x0034);
        machine.bus.write(0xd000, 0xabcd);
        assert_eq!(machine.bus.read(0xd000), 0x00cd);

        let state = machine.save_state().unwrap();
        machine.bus.write(0xd000, 0x0011);
        assert_eq!(machine.bus.read(0xd000), 0x0011);
        machine.load_state(&state).unwrap();
        assert_eq!(machine.bus.read(0xd000), 0x00cd);
    }

    #[test]
    fn intellicart_bank_switch_register_remaps_two_kib_window() {
        let rom = build_intellicart(
            &[(0x50, vec![0x1111; 0x100]), (0x60, vec![0x2222; 0x100])],
            &[(10, ICART_READ | ICART_BANKSW, 0, 0)],
        );
        let mut cart = Cartridge::from_bytes(&rom).unwrap();

        assert_eq!(cart.read(0x5000), Some(0x1111));
        assert!(cart.write(0x0045, 0x0060));
        assert_eq!(cart.read(0x5000), Some(0x2222));

        assert!(cart.write(0x0045, 0x0050));
        assert_eq!(cart.read(0x5000), Some(0x1111));
    }

    #[test]
    fn state_round_trip_restores_cpu_ram_stic_and_psg() {
        let (game, exec, grom) = synthetic_images();
        let mut machine = IntellivisionMachine::from_images(&game, &exec, &grom).unwrap();
        machine.run_frame(&InputState::default());
        machine.bus.system_ram[7] = 0x1234;
        machine.bus.psg[11] = 9;
        let saved = machine.save_state().unwrap();
        machine.bus.system_ram[7] = 0;
        machine.load_state(&saved).unwrap();
        assert_eq!(machine.bus.system_ram[7], 0x1234);
        assert_eq!(machine.bus.psg[11], 9);
        assert_eq!(machine.save_state().unwrap(), saved);
    }

    #[test]
    fn stic_masks_reserved_registers_collision_bits_and_gram_aliases() {
        let (game, exec, grom) = synthetic_images();
        let mut machine = IntellivisionMachine::from_images(&game, &exec, &grom).unwrap();
        machine.bus.write_stic(0x18, 0x03ff);
        assert_eq!(machine.bus.read_stic(0x18) & 0x03ff, 0x03fe);
        machine.bus.write_stic(0x20, 0);
        assert_eq!(machine.bus.read_stic(0x20), 0x3fff);
        machine.bus.write_stic(0x22, 0);
        assert_eq!(machine.bus.read_stic(0x22), 0x3fff);

        machine.bus.write(0x7805, 0x00ab);
        assert_eq!(machine.bus.read(0x3805), 0x00ab);
        assert_eq!(machine.bus.read(0xb805), 0x00ab);
        assert_eq!(machine.bus.read(0xf805), 0x00ab);
    }

    #[test]
    fn psg_register_masks_and_envelope_writes_follow_ay38914_contract() {
        let (game, exec, grom) = synthetic_images();
        let mut machine = IntellivisionMachine::from_images(&game, &exec, &grom).unwrap();
        machine.bus.write(0x01f4, 0x00ff);
        machine.bus.write(0x01f9, 0x00ff);
        machine.bus.write(0x01fb, 0x00ff);
        assert_eq!(machine.bus.psg[4], 0x0f);
        assert_eq!(machine.bus.psg[9], 0x1f);
        assert_eq!(machine.bus.psg[11], 0x3f);
        assert_eq!(machine.bus.psg_envelope_generation, 0);
        machine.bus.write(0x01fa, 0x000e);
        assert_eq!(machine.bus.psg[10], 0x0e);
        assert_eq!(machine.bus.psg_envelope_generation, 1);
    }

    #[test]
    fn psg_noise_envelope_and_fractional_frame_pacing_are_deterministic() {
        let mut noise = PsgState::default();
        let mut registers = [0u8; 0x10];
        registers[8] = 0x01;
        registers[9] = 1;
        registers[11] = 15;
        let mut saw_silence = false;
        let mut saw_noise = false;
        for _ in 0..128 {
            noise.tick(&registers);
            let sample = noise.sample(&registers);
            saw_silence |= sample == 0.0;
            saw_noise |= sample > 0.0;
        }
        assert!(saw_silence && saw_noise);

        let mut envelope = PsgState::default();
        registers = [0; 0x10];
        registers[3] = 1;
        registers[8] = 0x09;
        registers[10] = 0x0e;
        registers[11] = 0x3f;
        envelope.trigger_envelope(&registers, 1);
        let mut minimum = envelope.envelope_level;
        let mut maximum = envelope.envelope_level;
        for _ in 0..80 {
            envelope.tick(&registers);
            minimum = minimum.min(envelope.envelope_level);
            maximum = maximum.max(envelope.envelope_level);
        }
        assert_eq!((minimum, maximum), (0, 15));

        let (game, exec, grom) = synthetic_images();
        let mut machine = IntellivisionMachine::from_images(&game, &exec, &grom).unwrap();
        let mut produced = 0usize;
        for _ in 0..100 {
            machine.synthesize_audio();
            produced += machine.audio.samples().len() / 2;
        }
        let expected = (100 * u64::from(AUDIO_RATE) * FRAME_RATE_DEN / FRAME_RATE_NUM) as usize;
        assert_eq!(produced, expected);
    }

    #[test]
    fn cpu_clock_budget_is_exact_over_complete_frame_rate_periods() {
        let mut clock = ClockDomain::new(
            ClockRate::rational(FRAME_RATE_NUM, FRAME_RATE_DEN),
            ClockRate::hz(CPU_CLOCK_HZ),
        );
        for _ in 0..FRAME_RATE_NUM {
            clock.advance(1);
        }
        assert_eq!(clock.cycles(), CPU_CLOCK_HZ * FRAME_RATE_DEN);
        assert_eq!(clock.phase().0, 0);
    }
}
