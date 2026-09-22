use crate::cd_image::{DiscImage, DiscLayout};
use crate::cpu68000::{Bus68000, M68000};
use crate::cpu_z80::{Z80Bus, Z80};
use crate::kernel::{AudioBuffer, InputState, VideoBuffer};
use crate::machine::Machine;
use crate::platform::PlatformId;
use crate::resources::{ResourceBlob, ResourceKind, RESOURCE_PENDING};
use crate::state::{StateReader, StateWriter};

use super::genesis::GenesisIo;
use super::genesis_vdp::GenesisVdp;
use super::sn76489::Sn76489;
use super::ym2612::Ym2612;

const MAIN_HZ: u64 = 7_670_454;
const SUB_HZ: u64 = 12_500_000;
const Z80_HZ: u64 = 3_579_545;
const FRAME_RATE: f64 = 59.922_743;
const AUDIO_RATE: u32 = 48_000;
const BOOT_ROM_SIZE: usize = 0x20000;
const PRG_RAM_SIZE: usize = 0x80000;
const WORD_RAM_SIZE: usize = 0x40000;
const BRAM_SIZE: usize = 0x2000;
const STATE_VERSION: u32 = 11;
const CDDA_RATE: u32 = 44_100;
const CDDA_FRAMES_PER_SECTOR: usize = 588;

const CDD_STOP: u8 = 0x00;
const CDD_PLAY: u8 = 0x01;
const CDD_PAUSE: u8 = 0x04;
const CDD_OPEN: u8 = 0x05;
const CDD_TOC: u8 = 0x09;
const CDD_END: u8 = 0x0c;

#[derive(Clone)]
struct SegaCdDisc {
    image: DiscImage,
    sectors: u32,
}

impl SegaCdDisc {
    fn new(blob: ResourceBlob) -> Result<Self, String> {
        let image = DiscImage::new(blob)?;
        let sectors = image.sectors;
        Ok(Self { image, sectors })
    }

    fn read_user_sector(&self, lba: u32, out: &mut [u8; 2352]) -> Result<usize, String> {
        if lba >= self.sectors {
            return Err("Sega CD sector read exceeded the disc image".into());
        }
        let Some(track) = self.image.track_for_lba(lba) else {
            out[..2048].fill(0);
            return Ok(2048);
        };
        match track.layout {
            DiscLayout::Iso2048 => {
                let target: &mut [u8; 2048] = (&mut out[..2048])
                    .try_into()
                    .map_err(|_| "Sega CD sector buffer size mismatch".to_string())?;
                self.image.read_user_sector(lba, target)?;
                Ok(2048)
            }
            DiscLayout::Raw2352Mode1 | DiscLayout::Raw2352Mode2 => {
                let mut raw = [0u8; 2352];
                self.image.read_raw_sector(lba, &mut raw)?;
                match raw[15] {
                    1 => {
                        out[..2048].copy_from_slice(&raw[16..16 + 2048]);
                        Ok(2048)
                    }
                    2 => {
                        let form2 = raw[18] & 0x20 != 0;
                        let len = if form2 { 2324 } else { 2048 };
                        out[..len].copy_from_slice(&raw[24..24 + len]);
                        Ok(len)
                    }
                    mode => Err(format!(
                        "Sega CD raw sector uses unsupported CD-ROM mode {mode}"
                    )),
                }
            }
            DiscLayout::Raw2352Audio => {
                self.image.read_raw_sector(lba, out)?;
                Ok(2352)
            }
        }
    }
}

struct SegaCdDrive {
    disc: SegaCdDisc,
    current_lba: u32,
    sector: Box<[u8; 2352]>,
    data_pos: usize,
    data_len: usize,
    reading: bool,
    sector_phase: u64,
    sector_ready: bool,
    end_of_disc: bool,
    status: u8,
    cdda_sector: Box<[u8; 2352]>,
    cdda_sample_index: usize,
    cdda_clock_phase: u64,
    cdda_source_phase: u32,
    cdda_current: [i16; 2],
    cdda_samples: Vec<(f32, f32)>,
}

impl SegaCdDrive {
    fn new(disc: ResourceBlob) -> Result<Self, String> {
        Ok(Self {
            disc: SegaCdDisc::new(disc)?,
            current_lba: 0,
            sector: Box::new([0; 2352]),
            data_pos: 0,
            data_len: 0,
            reading: false,
            sector_phase: 0,
            sector_ready: false,
            end_of_disc: false,
            status: CDD_TOC,
            cdda_sector: Box::new([0; 2352]),
            cdda_sample_index: CDDA_FRAMES_PER_SECTOR,
            cdda_clock_phase: 0,
            cdda_source_phase: 0,
            cdda_current: [0; 2],
            cdda_samples: Vec::new(),
        })
    }

    fn reset_cdda_position(&mut self) {
        self.cdda_sample_index = CDDA_FRAMES_PER_SECTOR;
        self.cdda_source_phase = 0;
        self.cdda_current = [0; 2];
    }

    fn begin_frame(&mut self) {
        self.cdda_samples.clear();
    }

    fn audio_path_active(&self) -> bool {
        if !self.reading || self.status != CDD_PLAY {
            return false;
        }
        if self.cdda_sample_index < CDDA_FRAMES_PER_SECTOR {
            return true;
        }
        self.disc
            .image
            .track_for_lba(self.current_lba)
            .is_some_and(|track| track.kind == crate::cd_image::TrackKind::Audio)
    }

    fn load_cdda_sector(&mut self) -> bool {
        if self.current_lba >= self.disc.sectors {
            self.reading = false;
            self.end_of_disc = true;
            self.status = CDD_END;
            return false;
        }
        if self
            .disc
            .image
            .track_for_lba(self.current_lba)
            .is_none_or(|track| track.kind != crate::cd_image::TrackKind::Audio)
        {
            return false;
        }
        match self
            .disc
            .image
            .read_raw_sector(self.current_lba, self.cdda_sector.as_mut())
        {
            Ok(()) => {
                self.current_lba = self.current_lba.saturating_add(1);
                self.cdda_sample_index = 0;
                true
            }
            Err(error) if error == RESOURCE_PENDING => false,
            Err(_) => {
                self.reading = false;
                self.end_of_disc = true;
                self.status = CDD_END;
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

    fn tick_cdda(&mut self, sub_cycles: u32) {
        self.cdda_clock_phase = self
            .cdda_clock_phase
            .saturating_add(u64::from(sub_cycles) * u64::from(AUDIO_RATE));
        while self.cdda_clock_phase >= SUB_HZ {
            self.cdda_clock_phase -= SUB_HZ;
            if self.audio_path_active() {
                if self.cdda_sample_index >= CDDA_FRAMES_PER_SECTOR {
                    self.cdda_current = self.next_cdda_source_sample().unwrap_or([0; 2]);
                }
                self.cdda_samples.push((
                    f32::from(self.cdda_current[0]) / 32768.0,
                    f32::from(self.cdda_current[1]) / 32768.0,
                ));
                self.cdda_source_phase = self.cdda_source_phase.saturating_add(CDDA_RATE);
                while self.cdda_source_phase >= AUDIO_RATE {
                    self.cdda_source_phase -= AUDIO_RATE;
                    self.cdda_current = self.next_cdda_source_sample().unwrap_or([0; 2]);
                }
            } else {
                self.cdda_samples.push((0.0, 0.0));
            }
        }
    }

    fn start_read(&mut self, lba: u32) {
        self.current_lba = lba;
        self.data_pos = 0;
        self.data_len = 0;
        self.reading = lba < self.disc.sectors;
        self.sector_phase = if self.reading { SUB_HZ / 75 } else { 0 };
        self.sector_ready = false;
        self.end_of_disc = !self.reading;
        self.status = if self.reading { CDD_PLAY } else { CDD_END };
        self.reset_cdda_position();
    }

    fn seek(&mut self, lba: u32) {
        self.current_lba = lba.min(self.disc.sectors);
        self.data_pos = 0;
        self.data_len = 0;
        self.reading = false;
        self.sector_phase = 0;
        self.sector_ready = false;
        self.end_of_disc = lba >= self.disc.sectors;
        self.status = if self.end_of_disc { CDD_END } else { CDD_PAUSE };
        self.reset_cdda_position();
    }

    fn pause(&mut self) {
        if self.status == CDD_PLAY {
            self.reading = false;
            self.status = CDD_PAUSE;
        }
    }

    fn resume(&mut self) {
        if self.status == CDD_PAUSE
            && (self.cdda_sample_index < CDDA_FRAMES_PER_SECTOR
                || self.current_lba < self.disc.sectors)
        {
            self.reading = true;
            self.status = CDD_PLAY;
        }
    }

    fn stop(&mut self) {
        self.current_lba = 0;
        self.reading = false;
        self.data_pos = 0;
        self.data_len = 0;
        self.sector_phase = 0;
        self.sector_ready = false;
        self.end_of_disc = false;
        self.status = CDD_TOC;
        self.reset_cdda_position();
    }

    fn tick(&mut self, sub_cycles: u32) {
        let audio_path = self.audio_path_active();
        self.tick_cdda(sub_cycles);
        if !self.reading || audio_path {
            return;
        }
        self.sector_phase = self.sector_phase.saturating_add(u64::from(sub_cycles));
        if self.sector_ready {
            return;
        }
        let period = SUB_HZ / 75;
        if self.sector_phase >= period {
            self.sector_phase -= period;
            if self.current_lba >= self.disc.sectors {
                self.reading = false;
                self.end_of_disc = true;
                self.status = CDD_END;
                return;
            }
            match self
                .disc
                .read_user_sector(self.current_lba, &mut self.sector)
            {
                Ok(len) => {
                    self.data_pos = 0;
                    self.data_len = len;
                    self.sector_ready = true;
                    self.current_lba = self.current_lba.saturating_add(1);
                }
                Err(_) => {
                    // ResourceBlob records the missing range for browser hydration.
                    self.sector_phase = period;
                }
            }
        }
    }

    fn read_host_word(&mut self) -> u16 {
        if self.data_pos >= self.data_len {
            self.sector_ready = false;
            return 0;
        }
        let high = self.sector[self.data_pos];
        let low = self.sector.get(self.data_pos + 1).copied().unwrap_or(0);
        self.data_pos = (self.data_pos + 2).min(self.data_len);
        if self.data_pos >= self.data_len {
            self.sector_ready = false;
        }
        u16::from_be_bytes([high, low])
    }

    fn save(&self, out: &mut StateWriter) {
        out.u32(self.current_lba);
        out.blob(self.sector.as_slice());
        out.u32(self.data_pos as u32);
        out.u32(self.data_len as u32);
        out.u8(u8::from(self.reading));
        out.u64(self.sector_phase);
        out.u8(u8::from(self.sector_ready));
        out.u8(u8::from(self.end_of_disc));
        out.u8(self.status);
        out.blob(self.cdda_sector.as_slice());
        out.u16(self.cdda_sample_index as u16);
        out.u64(self.cdda_clock_phase);
        out.u32(self.cdda_source_phase);
        out.u16(self.cdda_current[0] as u16);
        out.u16(self.cdda_current[1] as u16);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.current_lba = input.u32()?.min(self.disc.sectors);
        let sector = input.blob()?;
        if sector.len() != 2352 {
            return Err("Sega CD state has invalid CDC sector length".into());
        }
        self.sector.copy_from_slice(sector);
        self.data_pos = (input.u32()? as usize).min(2352);
        self.data_len = (input.u32()? as usize).min(2352);
        self.reading = input.u8()? != 0;
        self.sector_phase = input.u64()? % (SUB_HZ / 75);
        self.sector_ready = input.u8()? != 0;
        self.end_of_disc = input.u8()? != 0;
        self.status = input.u8()?;
        if !matches!(
            self.status,
            CDD_STOP | CDD_PLAY | CDD_PAUSE | CDD_OPEN | CDD_TOC | CDD_END
        ) {
            return Err("Sega CD state has invalid CDD status".into());
        }
        let cdda_sector = input.blob()?;
        if cdda_sector.len() != 2352 {
            return Err("Sega CD state has invalid CD-DA sector length".into());
        }
        self.cdda_sector.copy_from_slice(cdda_sector);
        self.cdda_sample_index = usize::from(input.u16()?);
        if self.cdda_sample_index > CDDA_FRAMES_PER_SECTOR {
            return Err("Sega CD state has invalid CD-DA sample index".into());
        }
        self.cdda_clock_phase = input.u64()? % SUB_HZ;
        self.cdda_source_phase = input.u32()? % AUDIO_RATE;
        self.cdda_current = [input.u16()? as i16, input.u16()? as i16];
        self.cdda_samples.clear();
        Ok(())
    }
}

#[derive(Clone, Copy)]
struct PcmChannel {
    env: u8,
    pan: u8,
    step: u16,
    loop_start: u16,
    start: u8,
    address: u32,
}

impl Default for PcmChannel {
    fn default() -> Self {
        Self {
            env: 0,
            pan: 0xff,
            step: 0,
            loop_start: 0,
            start: 0,
            address: 0,
        }
    }
}

struct SegaCdPcm {
    channels: [PcmChannel; 8],
    ram: Box<[u8; 0x10000]>,
    enabled: bool,
    status: u8,
    selected: usize,
    bank: u8,
    clock_phase: u64,
    output_phase: u64,
    current: (f32, f32),
    samples: Vec<(f32, f32)>,
}

impl Default for SegaCdPcm {
    fn default() -> Self {
        Self {
            channels: [PcmChannel::default(); 8],
            ram: Box::new([0; 0x10000]),
            enabled: false,
            status: 0,
            selected: 0,
            bank: 0,
            clock_phase: 0,
            output_phase: 0,
            current: (0.0, 0.0),
            samples: Vec::with_capacity(1024),
        }
    }
}

impl SegaCdPcm {
    const SUB_CYCLES_PER_PCM_CLOCK: u64 = 384;

    fn begin_frame(&mut self) {
        self.samples.clear();
    }

    fn read(&self, address: u16) -> u8 {
        let address = address & 0x1fff;
        if address >= 0x1000 {
            return self.ram[(usize::from(self.bank) << 12) | usize::from(address & 0x0fff)];
        }
        let channel = self.channels[self.selected];
        match address {
            0x00 => channel.env,
            0x01 => channel.pan,
            0x02 => channel.step as u8,
            0x03 => (channel.step >> 8) as u8,
            0x04 => channel.loop_start as u8,
            0x05 => (channel.loop_start >> 8) as u8,
            0x06 => channel.start,
            0x07 => (u8::from(self.enabled) << 7) | 0x40 | self.selected as u8,
            0x08 => !self.status,
            _ => 0,
        }
    }

    fn write(&mut self, address: u16, value: u8) {
        let address = address & 0x1fff;
        if address >= 0x1000 {
            let offset = (usize::from(self.bank) << 12) | usize::from(address & 0x0fff);
            self.ram[offset] = value;
            return;
        }
        match address {
            0x00 => self.channels[self.selected].env = value,
            0x01 => self.channels[self.selected].pan = value,
            0x02 => {
                let ch = &mut self.channels[self.selected];
                ch.step = (ch.step & 0xff00) | u16::from(value);
            }
            0x03 => {
                let ch = &mut self.channels[self.selected];
                ch.step = (ch.step & 0x00ff) | (u16::from(value) << 8);
            }
            0x04 => {
                let ch = &mut self.channels[self.selected];
                ch.loop_start = (ch.loop_start & 0xff00) | u16::from(value);
            }
            0x05 => {
                let ch = &mut self.channels[self.selected];
                ch.loop_start = (ch.loop_start & 0x00ff) | (u16::from(value) << 8);
            }
            0x06 => {
                let ch = &mut self.channels[self.selected];
                ch.start = value;
                if self.status & (1 << self.selected) == 0 {
                    ch.address = u32::from(value) << 19;
                }
            }
            0x07 => {
                self.enabled = value & 0x80 != 0;
                if value & 0x40 != 0 {
                    self.selected = usize::from(value & 7);
                } else {
                    self.bank = value & 0x0f;
                }
            }
            0x08 => {
                self.status = !value;
                for channel in 0..8 {
                    if value & (1 << channel) != 0 {
                        self.channels[channel].address =
                            u32::from(self.channels[channel].start) << 19;
                    }
                }
            }
            _ => {}
        }
    }

    fn clock_chip(&mut self) {
        if !self.enabled {
            self.current = (0.0, 0.0);
            return;
        }
        let mut left = 0i32;
        let mut right = 0i32;
        for index in 0..8 {
            if self.status & (1 << index) == 0 {
                continue;
            }
            let channel = &mut self.channels[index];
            let mut sample = self.ram[((channel.address >> 11) & 0xffff) as usize];
            if sample == 0xff {
                channel.address = u32::from(channel.loop_start) << 11;
                sample = self.ram[usize::from(channel.loop_start)];
            } else {
                channel.address = channel.address.wrapping_add(u32::from(channel.step));
            }
            if sample == 0xff {
                continue;
            }
            let magnitude = i32::from(sample & 0x7f);
            let signed = if sample & 0x80 != 0 {
                magnitude
            } else {
                -magnitude
            };
            left += (signed * i32::from(channel.env) * i32::from(channel.pan & 0x0f)) >> 5;
            right += (signed * i32::from(channel.env) * i32::from(channel.pan >> 4)) >> 5;
        }
        self.current = (
            (left.clamp(-32768, 32767) as f32) / 32768.0,
            (right.clamp(-32768, 32767) as f32) / 32768.0,
        );
    }

    fn tick(&mut self, sub_cycles: u32) {
        self.clock_phase = self.clock_phase.saturating_add(u64::from(sub_cycles));
        while self.clock_phase >= Self::SUB_CYCLES_PER_PCM_CLOCK {
            self.clock_phase -= Self::SUB_CYCLES_PER_PCM_CLOCK;
            self.clock_chip();
            self.output_phase = self
                .output_phase
                .saturating_add(Self::SUB_CYCLES_PER_PCM_CLOCK * u64::from(AUDIO_RATE));
            while self.output_phase >= SUB_HZ {
                self.output_phase -= SUB_HZ;
                self.samples.push(self.current);
            }
        }
    }

    fn save(&self, out: &mut StateWriter) {
        for channel in self.channels {
            out.u8(channel.env);
            out.u8(channel.pan);
            out.u16(channel.step);
            out.u16(channel.loop_start);
            out.u8(channel.start);
            out.u32(channel.address);
        }
        out.blob(self.ram.as_slice());
        out.u8(u8::from(self.enabled));
        out.u8(self.status);
        out.u8(self.selected as u8);
        out.u8(self.bank);
        out.u64(self.clock_phase);
        out.u64(self.output_phase);
        out.u32(self.current.0.to_bits());
        out.u32(self.current.1.to_bits());
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        for channel in &mut self.channels {
            channel.env = input.u8()?;
            channel.pan = input.u8()?;
            channel.step = input.u16()?;
            channel.loop_start = input.u16()?;
            channel.start = input.u8()?;
            channel.address = input.u32()?;
        }
        let ram = input.blob()?;
        if ram.len() != self.ram.len() {
            return Err("Sega CD PCM state has invalid wave RAM length".into());
        }
        self.ram.copy_from_slice(ram);
        self.enabled = input.u8()? != 0;
        self.status = input.u8()?;
        self.selected = usize::from(input.u8()? & 7);
        self.bank = input.u8()? & 0x0f;
        self.clock_phase = input.u64()? % Self::SUB_CYCLES_PER_PCM_CLOCK;
        self.output_phase = input.u64()? % SUB_HZ;
        self.current = (f32::from_bits(input.u32()?), f32::from_bits(input.u32()?));
        self.samples.clear();
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, Default)]
struct SegaCdGraphics {
    active: bool,
    line_credit: i64,
    cycles_per_line: u64,
    dot_mask: u32,
    stamp_shift: u8,
    map_shift: u8,
    map_base: usize,
    trace_offset: usize,
    buffer_start: u32,
    buffer_offset: u32,
}

impl SegaCdGraphics {
    fn read_word(word_ram: &[u8], offset: usize) -> u16 {
        let offset = offset & (WORD_RAM_SIZE - 1);
        u16::from_be_bytes([
            word_ram[offset],
            word_ram[(offset + 1) & (WORD_RAM_SIZE - 1)],
        ])
    }

    fn merge_priority(previous: u8, next: u8, mode: u8) -> u8 {
        match mode & 3 {
            0 => next,
            1 => {
                let high = if previous & 0xf0 != 0 {
                    previous & 0xf0
                } else {
                    next & 0xf0
                };
                let low = if previous & 0x0f != 0 {
                    previous & 0x0f
                } else {
                    next & 0x0f
                };
                high | low
            }
            2 => {
                let high = if next & 0xf0 != 0 {
                    next & 0xf0
                } else {
                    previous & 0xf0
                };
                let low = if next & 0x0f != 0 {
                    next & 0x0f
                } else {
                    previous & 0x0f
                };
                high | low
            }
            _ => previous,
        }
    }

    fn start(&mut self, gate: &mut SegaCdGate) {
        let config = gate.byte(0x59);
        let (dot_mask, stamp_shift, map_shift, map_mask) = match (config >> 1) & 3 {
            0 => (0x07ffff, 15, 4, 0x3fe00usize),
            1 => (0x07ffff, 16, 3, 0x3ff80usize),
            2 => (0x7fffff, 15, 8, 0x20000usize),
            _ => (0x7fffff, 16, 7, 0x38000usize),
        };
        self.dot_mask = dot_mask;
        self.stamp_shift = stamp_shift;
        self.map_shift = map_shift;
        self.map_base = (usize::from(gate.regs[0x5a >> 1]) << 2) & map_mask;
        self.trace_offset = (usize::from(gate.regs[0x66 >> 1]) << 2) & (WORD_RAM_SIZE - 8);
        self.buffer_offset = ((u32::from(gate.byte(0x5d) & 0x1f) + 1) << 6).saturating_sub(7);
        self.buffer_start =
            ((u32::from(gate.regs[0x5e >> 1]) << 3) & 0x7ffc0) + u32::from(gate.byte(0x61) & 0x3f);
        let width = u64::from(gate.regs[0x62 >> 1] & 0x01ff);
        let pixel_offset = u64::from(gate.byte(0x61) & 3);
        self.cycles_per_line = 12 * (4 + 2 * width + ((width + pixel_offset + 3) >> 2));
        self.cycles_per_line = self.cycles_per_line.max(1);
        self.line_credit = 0;
        self.active = true;
        gate.set_byte(0x58, 0x80);
    }

    fn transformed_pixel(
        &self,
        word_ram: &[u8],
        stamp_data: u16,
        xpos: u32,
        ypos: u32,
        stamp_32: bool,
    ) -> u8 {
        let stamp_mask = if stamp_32 { 0x07fc } else { 0x07ff };
        let stamp_base = u32::from(stamp_data & stamp_mask) << 8;
        if stamp_base == 0 {
            return 0;
        }

        let size_mask = if stamp_32 { 31u32 } else { 15u32 };
        let mut x = (xpos >> 11) & size_mask;
        let mut y = (ypos >> 11) & size_mask;
        let transform = (stamp_data >> 13) & 7;
        if transform & 4 != 0 {
            x ^= size_mask;
        }
        if transform & 2 != 0 {
            x ^= size_mask;
            y ^= size_mask;
        }
        if transform & 1 != 0 {
            let old_x = x;
            x = y ^ size_mask;
            y = old_x;
        }

        let cells = if stamp_32 { 4 } else { 2 };
        let cell = (y >> 3) + (x >> 3) * cells;
        let pixel = (x & 7) + (y & 7) * 8;
        let pixel_index = stamp_base | (cell << 6) | pixel;
        let packed = word_ram[((pixel_index >> 1) as usize) & (WORD_RAM_SIZE - 1)];
        if pixel_index & 1 != 0 {
            packed & 0x0f
        } else {
            packed >> 4
        }
    }

    fn render_line(&mut self, gate: &SegaCdGate, word_ram: &mut [u8]) {
        let mut trace = self.trace_offset;
        let mut xpos = u32::from(Self::read_word(word_ram, trace)) << 8;
        trace = (trace + 2) & (WORD_RAM_SIZE - 1);
        let mut ypos = u32::from(Self::read_word(word_ram, trace)) << 8;
        trace = (trace + 2) & (WORD_RAM_SIZE - 1);
        let xoffset = i32::from(Self::read_word(word_ram, trace) as i16) as u32;
        trace = (trace + 2) & (WORD_RAM_SIZE - 1);
        let yoffset = i32::from(Self::read_word(word_ram, trace) as i16) as u32;
        trace = (trace + 2) & (WORD_RAM_SIZE - 1);
        self.trace_offset = trace;

        let config = gate.byte(0x59);
        let repeat = config & 1 != 0;
        let stamp_32 = config & 2 != 0;
        let width = u32::from(gate.regs[0x62 >> 1] & 0x01ff);
        let priority = (gate.memory_mode() >> 3) & 3;
        let mut buffer_index = self.buffer_start;

        for _ in 0..width {
            if repeat {
                xpos &= self.dot_mask;
                ypos &= self.dot_mask;
            } else {
                xpos &= 0x00ff_ffff;
                ypos &= 0x00ff_ffff;
            }

            let pixel = if (xpos | ypos) & !self.dot_mask != 0 {
                0
            } else {
                let map_index =
                    (xpos >> self.stamp_shift) | ((ypos >> self.stamp_shift) << self.map_shift);
                let stamp_data =
                    Self::read_word(word_ram, self.map_base + (map_index as usize * 2));
                self.transformed_pixel(word_ram, stamp_data, xpos, ypos, stamp_32)
            };

            let byte_index = ((buffer_index >> 1) as usize) & (WORD_RAM_SIZE - 1);
            let previous = word_ram[byte_index];
            let next = if buffer_index & 1 != 0 {
                (previous & 0xf0) | pixel
            } else {
                (pixel << 4) | (previous & 0x0f)
            };
            word_ram[byte_index] = Self::merge_priority(previous, next, priority);

            if buffer_index & 7 != 7 {
                buffer_index = buffer_index.wrapping_add(1);
            } else {
                buffer_index = buffer_index.wrapping_add(self.buffer_offset);
            }
            xpos = xpos.wrapping_add(xoffset);
            ypos = ypos.wrapping_add(yoffset);
        }

        self.buffer_start = self.buffer_start.wrapping_add(8);
    }

    fn tick(&mut self, cycles: u32, gate: &mut SegaCdGate, word_ram: &mut [u8]) {
        if !self.active {
            return;
        }
        if gate.one_meg_mode() || !gate.sub_has_word_ram() {
            return;
        }

        self.line_credit = self.line_credit.saturating_add(i64::from(cycles));
        while self.line_credit > 0 && self.active {
            let remaining = gate.byte(0x65);
            if remaining == 0 {
                self.active = false;
                gate.set_byte(0x58, 0);
                if gate.byte(0x33) & 0x02 != 0 {
                    gate.pending |= 1 << 1;
                }
                break;
            }

            self.render_line(gate, word_ram);
            gate.set_byte(0x65, remaining - 1);
            self.line_credit = self.line_credit.saturating_sub(self.cycles_per_line as i64);

            if remaining == 1 {
                self.active = false;
                gate.set_byte(0x58, 0);
                if gate.byte(0x33) & 0x02 != 0 {
                    gate.pending |= 1 << 1;
                }
            }
        }
    }

    fn save(&self, out: &mut StateWriter) {
        out.u8(u8::from(self.active));
        out.u64(self.line_credit as u64);
        out.u64(self.cycles_per_line);
        out.u32(self.dot_mask);
        out.u8(self.stamp_shift);
        out.u8(self.map_shift);
        out.u32(self.map_base as u32);
        out.u32(self.trace_offset as u32);
        out.u32(self.buffer_start);
        out.u32(self.buffer_offset);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.active = input.u8()? != 0;
        self.line_credit = input.u64()? as i64;
        self.cycles_per_line = input.u64()?.max(1);
        self.dot_mask = input.u32()?;
        self.stamp_shift = input.u8()?;
        self.map_shift = input.u8()?;
        self.map_base = input.u32()? as usize & (WORD_RAM_SIZE - 1);
        self.trace_offset = input.u32()? as usize & (WORD_RAM_SIZE - 1);
        self.buffer_start = input.u32()?;
        self.buffer_offset = input.u32()?;
        if self.active
            && (!matches!(self.dot_mask, 0x07ffff | 0x7fffff)
                || !matches!(self.stamp_shift, 15 | 16)
                || !matches!(self.map_shift, 3 | 4 | 7 | 8))
        {
            return Err("Sega CD state has invalid graphics-ASIC geometry".into());
        }
        Ok(())
    }
}

struct SegaCdGate {
    regs: [u16; 0x100],
    pending: u8,
    timer_remaining: u64,
    sub_reset_pulse: bool,
}

impl Default for SegaCdGate {
    fn default() -> Self {
        let mut regs = [0u16; 0x100];
        regs[0x00 >> 1] = 0x0002;
        regs[0x02 >> 1] = 0x0001;
        regs[0x08 >> 1] = 0xffff;
        regs[0x0a >> 1] = 0xffff;
        regs[0x36 >> 1] = 0x0100;
        regs[0x40 >> 1] = 0x000f;
        regs[0x42 >> 1] = 0xffff;
        regs[0x44 >> 1] = 0xffff;
        regs[0x46 >> 1] = 0xffff;
        regs[0x48 >> 1] = 0xffff;
        regs[0x4a >> 1] = 0xffff;
        Self {
            regs,
            pending: 0,
            timer_remaining: 0,
            sub_reset_pulse: false,
        }
    }
}

impl SegaCdGate {
    fn byte(&self, offset: u16) -> u8 {
        let word = self.regs[usize::from((offset & 0x01ff) >> 1)];
        if offset & 1 == 0 {
            (word >> 8) as u8
        } else {
            word as u8
        }
    }

    fn set_byte(&mut self, offset: u16, value: u8) {
        let slot = &mut self.regs[usize::from((offset & 0x01ff) >> 1)];
        if offset & 1 == 0 {
            *slot = (*slot & 0x00ff) | (u16::from(value) << 8);
        } else {
            *slot = (*slot & 0xff00) | u16::from(value);
        }
    }

    fn main_write(&mut self, offset: u16, value: u8) {
        let offset = offset & 0x3f;
        match offset {
            0x00 | 0x01 => {
                let previous = self.byte(0x01);
                self.set_byte(offset, value);
                let current = self.byte(0x01);
                if previous & 0x01 == 0 && current & 0x01 != 0 {
                    self.sub_reset_pulse = true;
                }
                if current & 0x01 == 0 {
                    self.set_byte(0x01, 0x02);
                }
            }
            _ => self.set_byte(offset, value),
        }
    }

    fn sub_write(&mut self, offset: u16, value: u8) {
        let offset = offset & 0x01ff;
        match offset {
            0x0e | 0x0f => self.set_byte(0x0f, value),
            0x10..=0x1f => {}
            0x20..=0x2f => self.set_byte(offset, value),
            0x30 | 0x31 => {
                self.set_byte(0x31, value);
                self.timer_remaining = u64::from(value) * 384;
            }
            0x32 | 0x33 => {
                self.set_byte(0x33, value);
                self.pending &= 0xfd | (value & 0x02);
            }
            0x58 => {}
            0x59 => self.set_byte(0x59, value & 0x07),
            0x5c => {}
            0x5d => self.set_byte(0x5d, value & 0x1f),
            0x60 => {}
            0x61 => self.set_byte(0x61, value & 0x3f),
            0x62 => self.set_byte(0x62, value & 0x01),
            0x64 => {}
            _ => self.set_byte(offset, value),
        }
    }

    fn sub_running(&self) -> bool {
        let control = self.byte(0x01);
        control & 0x01 != 0 && control & 0x02 == 0
    }
    fn main_prg_access(&self) -> bool {
        self.byte(0x01) & 0x02 != 0
    }
    fn memory_mode(&self) -> u8 {
        self.byte(0x03)
    }
    fn prg_bank(&self) -> usize {
        usize::from((self.memory_mode() >> 6) & 3)
    }
    fn one_meg_mode(&self) -> bool {
        self.memory_mode() & 0x04 != 0
    }
    fn main_has_word_ram(&self) -> bool {
        self.one_meg_mode() || self.memory_mode() & 0x01 != 0
    }
    fn sub_has_word_ram(&self) -> bool {
        self.one_meg_mode() || !self.main_has_word_ram()
    }
    fn main_word_offset(&self, address: u32) -> usize {
        if self.one_meg_mode() {
            usize::from(self.memory_mode() & 1) * 0x20000 + (address as usize & 0x1ffff)
        } else {
            address as usize & 0x3ffff
        }
    }
    fn sub_word_offset(&self, address: u32) -> usize {
        if self.one_meg_mode() {
            usize::from((self.memory_mode() & 1) ^ 1) * 0x20000 + (address as usize & 0x1ffff)
        } else {
            address as usize & 0x3ffff
        }
    }

    fn tick(&mut self, sub_cycles: u32) {
        if self.timer_remaining == 0 {
            return;
        }
        let used = u64::from(sub_cycles);
        if used >= self.timer_remaining {
            let reload = u64::from(self.byte(0x31)) * 384;
            self.timer_remaining = reload;
            self.pending |= 1 << 3;
        } else {
            self.timer_remaining -= used;
        }
    }

    fn highest_sub_irq(&self) -> u8 {
        let enabled = self.byte(0x33);
        for level in (1..=5).rev() {
            if self.pending & enabled & (1 << level) != 0 {
                return level;
            }
        }
        0
    }
    fn acknowledge_sub_irq(&mut self, level: u8) {
        self.pending &= !(1 << level.min(7));
    }

    fn save(&self, out: &mut StateWriter) {
        for register in self.regs {
            out.u16(register);
        }
        out.u8(self.pending);
        out.u64(self.timer_remaining);
        out.u8(u8::from(self.sub_reset_pulse));
    }
    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        for register in &mut self.regs {
            *register = input.u16()?;
        }
        self.pending = input.u8()?;
        self.timer_remaining = input.u64()?;
        self.sub_reset_pulse = input.u8()? != 0;
        Ok(())
    }
}

struct SegaCdBoard {
    bootrom: Vec<u8>,
    prg_ram: Box<[u8; PRG_RAM_SIZE]>,
    word_ram: Box<[u8; WORD_RAM_SIZE]>,
    bram: Box<[u8; BRAM_SIZE]>,
    main_ram: Box<[u8; 0x10000]>,
    z80_ram: Box<[u8; 0x2000]>,
    io: GenesisIo,
    vdp: GenesisVdp,
    ym: Ym2612,
    psg: Sn76489,
    pcm: SegaCdPcm,
    gate: SegaCdGate,
    graphics: SegaCdGraphics,
    drive: SegaCdDrive,
    z80_bus_requested: bool,
    z80_running: bool,
    z80_bank: u32,
}

impl SegaCdBoard {
    fn new(bootrom: &[u8], disc: ResourceBlob) -> Result<Self, String> {
        if bootrom.len() != BOOT_ROM_SIZE {
            return Err(format!(
                "Sega CD BIOS must be exactly {BOOT_ROM_SIZE} bytes"
            ));
        }
        Ok(Self {
            bootrom: bootrom.to_vec(),
            prg_ram: Box::new([0; PRG_RAM_SIZE]),
            word_ram: Box::new([0; WORD_RAM_SIZE]),
            bram: Box::new([0; BRAM_SIZE]),
            main_ram: Box::new([0; 0x10000]),
            z80_ram: Box::new([0; 0x2000]),
            io: GenesisIo::default(),
            vdp: GenesisVdp::default(),
            ym: Ym2612::new(MAIN_HZ),
            psg: Sn76489::default(),
            pcm: SegaCdPcm::default(),
            gate: SegaCdGate::default(),
            graphics: SegaCdGraphics::default(),
            drive: SegaCdDrive::new(disc)?,
            z80_bus_requested: false,
            z80_running: false,
            z80_bank: 0,
        })
    }

    fn reset_devices(&mut self) {
        self.vdp.reset();
        self.ym.reset();
        self.psg.reset();
        self.pcm = SegaCdPcm::default();
        self.gate = SegaCdGate::default();
        self.graphics = SegaCdGraphics::default();
        self.io = GenesisIo::default();
        self.drive.stop();
        self.z80_bus_requested = false;
        self.z80_running = false;
        self.z80_bank = 0;
    }

    fn read_vdp_word(&mut self, address: u32) -> u16 {
        match address & 0x1c {
            0x00 => self.vdp.read_data(),
            0x04 => self.vdp.read_status(),
            0x08 => self.vdp.read_hv_counter(),
            _ => 0xffff,
        }
    }

    fn write_vdp_word(&mut self, address: u32, value: u16) {
        match address & 0x1c {
            0x00 => self.vdp.write_data(value),
            0x04 => self.vdp.write_control(value),
            0x10 => self.psg.write(value as u8),
            _ => {}
        }
    }

    fn dma_read_main8(&self, address: u32) -> u8 {
        let address = address & 0x00ff_ffff;
        match address {
            0x000000..=0x1fffff => {
                let local = address & 0x3ffff;
                if local < 0x20000 {
                    self.bootrom[local as usize & 0x1ffff]
                } else if self.gate.main_prg_access() {
                    let base = self.gate.prg_bank() * 0x20000;
                    self.prg_ram[base + (local as usize & 0x1ffff)]
                } else {
                    0xff
                }
            }
            0x200000..=0x3fffff if self.gate.main_has_word_ram() => {
                self.word_ram[self.gate.main_word_offset(address)]
            }
            0xff0000..=0xffffff => self.main_ram[address as usize & 0xffff],
            _ => 0xff,
        }
    }

    fn dma_read_main_word(&self, address: u32) -> u16 {
        let address = address & 0x00ff_fffe;
        u16::from_be_bytes([
            self.dma_read_main8(address),
            self.dma_read_main8(address.wrapping_add(1)),
        ])
    }

    fn service_vdp_dma(&mut self) -> Option<u32> {
        let source = self.vdp.memory_dma_source()?;
        let word = self.dma_read_main_word(source);
        self.vdp.service_memory_dma_word(word)
    }

    fn cdd_digit(&self, offset: u16) -> Option<u32> {
        let digit = self.gate.byte(offset) & 0x0f;
        (digit <= 9).then_some(u32::from(digit))
    }

    fn cdd_command_lba(&self) -> Option<u32> {
        let minute = self.cdd_digit(0x44)? * 10 + self.cdd_digit(0x45)?;
        let second = self.cdd_digit(0x46)? * 10 + self.cdd_digit(0x47)?;
        let frame = self.cdd_digit(0x48)? * 10 + self.cdd_digit(0x49)?;
        if second >= 60 || frame >= 75 {
            return None;
        }
        Some((minute * 60 * 75 + second * 75 + frame).saturating_sub(150))
    }

    fn cdd_set_pair(&mut self, offset: u16, value: u32) {
        let value = value.min(99);
        self.gate.set_byte(offset, (value / 10) as u8);
        self.gate.set_byte(offset + 1, (value % 10) as u8);
    }

    fn cdd_set_msf(&mut self, lba: u32) {
        let absolute = lba.saturating_add(150);
        self.cdd_set_pair(0x3a, (absolute / 75) / 60);
        self.cdd_set_pair(0x3c, (absolute / 75) % 60);
        self.cdd_set_pair(0x3e, absolute % 75);
    }

    fn cdd_position_lba(&self) -> u32 {
        if self.drive.sector_ready || self.drive.cdda_sample_index < CDDA_FRAMES_PER_SECTOR {
            self.drive.current_lba.saturating_sub(1)
        } else {
            self.drive.current_lba
        }
    }

    fn cdd_track_at(&self, lba: u32) -> Option<&crate::cd_image::DiscTrack> {
        let lba = lba.min(self.drive.disc.sectors.saturating_sub(1));
        self.drive.disc.image.track_for_lba(lba)
    }

    fn cdd_track_flags(&self, lba: u32) -> u8 {
        if self
            .cdd_track_at(lba)
            .is_some_and(|track| track.kind == crate::cd_image::TrackKind::Data)
        {
            0x04
        } else {
            0
        }
    }

    fn cdd_clear_payload(&mut self) {
        for offset in 0x3a..=0x40 {
            self.gate.set_byte(offset, 0);
        }
    }

    fn cdd_set_checksum(&mut self) {
        let sum = [0x38u16, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f, 0x40]
            .into_iter()
            .fold(0u8, |sum, offset| sum.wrapping_add(self.gate.byte(offset)));
        self.gate.set_byte(0x41, (!sum) & 0x0f);
    }

    fn cdd_report_position(&mut self, request: u8, relative: bool) {
        let position = self.cdd_position_lba();
        let lba = if relative {
            self.cdd_track_at(position)
                .map_or(position, |track| position.saturating_sub(track.start_lba))
        } else {
            position
        };
        self.gate.set_byte(0x38, self.drive.status);
        self.gate.set_byte(0x39, request);
        if relative {
            self.cdd_set_pair(0x3a, (lba / 75) / 60);
            self.cdd_set_pair(0x3c, (lba / 75) % 60);
            self.cdd_set_pair(0x3e, lba % 75);
        } else {
            self.cdd_set_msf(lba);
        }
        self.gate.set_byte(0x40, self.cdd_track_flags(position));
    }

    fn cdd_report_toc(&mut self, request: u8) {
        self.cdd_clear_payload();
        match request {
            0x00 => self.cdd_report_position(0x00, false),
            0x01 => self.cdd_report_position(0x01, true),
            0x02 => {
                let position = self.cdd_position_lba();
                let track = self
                    .cdd_track_at(position)
                    .map_or(0xaa, |track| track.number);
                self.gate.set_byte(0x38, self.drive.status);
                self.gate.set_byte(0x39, 0x02);
                if track == 0xaa {
                    self.gate.set_byte(0x3a, 0x0a);
                    self.gate.set_byte(0x3b, 0x0a);
                } else {
                    self.cdd_set_pair(0x3a, u32::from(track));
                }
            }
            0x03 => {
                self.gate.set_byte(0x38, self.drive.status);
                self.gate.set_byte(0x39, 0x03);
                self.cdd_set_msf(self.drive.disc.sectors);
            }
            0x04 => {
                self.gate.set_byte(0x38, self.drive.status);
                self.gate.set_byte(0x39, 0x04);
                let first = self
                    .drive
                    .disc
                    .image
                    .tracks
                    .first()
                    .map_or(1, |track| track.number);
                let last = self
                    .drive
                    .disc
                    .image
                    .tracks
                    .last()
                    .map_or(first, |track| track.number);
                self.cdd_set_pair(0x3a, u32::from(first));
                self.cdd_set_pair(0x3c, u32::from(last));
            }
            0x05 => {
                let track_number = self.gate.byte(0x46).min(9) * 10 + self.gate.byte(0x47).min(9);
                self.gate.set_byte(0x38, self.drive.status);
                self.gate.set_byte(0x39, 0x05);
                if let Some(track) = self
                    .drive
                    .disc
                    .image
                    .tracks
                    .iter()
                    .find(|track| track.number == track_number)
                    .cloned()
                {
                    self.cdd_set_msf(track.start_lba);
                    if track.kind == crate::cd_image::TrackKind::Data {
                        self.gate.set_byte(0x3e, self.gate.byte(0x3e) | 0x08);
                    }
                    self.gate.set_byte(0x40, track.number % 10);
                }
            }
            0x06 => {
                self.gate.set_byte(0x38, self.drive.status);
                self.gate.set_byte(0x39, 0x06);
            }
            _ => {
                self.gate.set_byte(0x38, self.drive.status);
                self.gate.set_byte(0x39, request);
            }
        }
    }

    fn process_cdd_command(&mut self) {
        let command = self.gate.byte(0x42) & 0x0f;
        match command {
            0x00 => {
                self.cdd_clear_payload();
                self.cdd_report_position(0x00, false);
            }
            0x01 => {
                self.drive.stop();
                self.cdd_clear_payload();
                self.gate.set_byte(0x38, CDD_STOP);
                self.gate.set_byte(0x39, 0x0f);
            }
            0x02 => self.cdd_report_toc(self.gate.byte(0x45) & 0x0f),
            0x03 => {
                if let Some(lba) = self.cdd_command_lba() {
                    self.drive.start_read(lba);
                }
                self.cdd_clear_payload();
                self.cdd_report_position(0x00, false);
            }
            0x04 => {
                if let Some(lba) = self.cdd_command_lba() {
                    self.drive.seek(lba);
                }
                self.cdd_clear_payload();
                self.cdd_report_position(0x00, false);
            }
            0x06 => {
                self.drive.pause();
                self.gate.set_byte(0x38, self.drive.status);
            }
            0x07 => {
                self.drive.resume();
                self.gate.set_byte(0x38, self.drive.status);
            }
            0x0c => {
                self.drive.stop();
                self.cdd_clear_payload();
                self.gate.set_byte(0x38, CDD_STOP);
                self.gate.set_byte(0x39, 0x0f);
            }
            0x0d => {
                self.drive.stop();
                self.drive.status = CDD_OPEN;
                self.cdd_clear_payload();
                self.gate.set_byte(0x38, CDD_OPEN);
                self.gate.set_byte(0x39, 0x0f);
            }
            _ => self.gate.set_byte(0x38, self.drive.status),
        }
        self.cdd_set_checksum();
        if self.gate.byte(0x37) & 0x04 != 0 {
            self.gate.pending |= 1 << 4;
        }
    }

    fn tick_main_cycles(&mut self, cycles: u32) {
        self.vdp.tick_cpu_cycles(cycles);
        self.ym.tick(cycles);
    }

    fn tick_sub_cycles(&mut self, cycles: u32) {
        self.gate.tick(cycles);
        self.graphics
            .tick(cycles, &mut self.gate, self.word_ram.as_mut_slice());
        let was_ready = self.drive.sector_ready;
        self.drive.tick(cycles);
        if !was_ready && self.drive.sector_ready {
            self.gate.pending |= 1 << 5;
        }
        self.pcm.tick(cycles);
    }

    fn zbank_address(&self, address: u16) -> u32 {
        ((self.z80_bank & 0x01ff) << 15) | u32::from(address & 0x7fff)
    }

    fn zbank_read(&mut self, address: u16) -> u8 {
        let target = self.zbank_address(address) & 0x00ff_ffff;
        match target {
            0x000000..=0x1fffff => {
                let local = target & 0x3ffff;
                if local < 0x20000 {
                    self.bootrom[local as usize & 0x1ffff]
                } else if self.gate.main_prg_access() {
                    let base = self.gate.prg_bank() * 0x20000;
                    self.prg_ram[base + (local as usize & 0x1ffff)]
                } else {
                    0xff
                }
            }
            0x200000..=0x3fffff if self.gate.main_has_word_ram() => {
                self.word_ram[self.gate.main_word_offset(target)]
            }
            0xff0000..=0xffffff => self.main_ram[target as usize & 0xffff],
            _ => 0xff,
        }
    }

    fn zbank_write(&mut self, address: u16, value: u8) {
        let target = self.zbank_address(address) & 0x00ff_ffff;
        match target {
            0x020000..=0x1fffff if self.gate.main_prg_access() => {
                let local = target & 0x3ffff;
                if local >= 0x20000 {
                    let base = self.gate.prg_bank() * 0x20000;
                    self.prg_ram[base + (local as usize & 0x1ffff)] = value;
                }
            }
            0x200000..=0x3fffff if self.gate.main_has_word_ram() => {
                let offset = self.gate.main_word_offset(target);
                self.word_ram[offset] = value;
            }
            0xff0000..=0xffffff => self.main_ram[target as usize & 0xffff] = value,
            _ => {}
        }
    }

    fn save(&self, out: &mut StateWriter) {
        out.blob(self.prg_ram.as_slice());
        out.blob(self.word_ram.as_slice());
        out.blob(self.bram.as_slice());
        out.blob(self.main_ram.as_slice());
        out.blob(self.z80_ram.as_slice());
        self.vdp.save(out);
        self.ym.save(out);
        self.psg.save(out);
        self.pcm.save(out);
        self.gate.save(out);
        self.graphics.save(out);
        self.drive.save(out);
        out.u8(u8::from(self.z80_bus_requested));
        out.u8(u8::from(self.z80_running));
        out.u32(self.z80_bank);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        for (target, label) in [
            (self.prg_ram.as_mut_slice(), "PRG RAM"),
            (self.word_ram.as_mut_slice(), "Word RAM"),
            (self.bram.as_mut_slice(), "backup RAM"),
            (self.main_ram.as_mut_slice(), "main RAM"),
            (self.z80_ram.as_mut_slice(), "Z80 RAM"),
        ] {
            let data = input.blob()?;
            if data.len() != target.len() {
                return Err(format!("Sega CD state has invalid {label} length"));
            }
            target.copy_from_slice(data);
        }
        self.vdp.load(input)?;
        self.ym.load(input)?;
        self.psg.load(input)?;
        self.pcm.load(input)?;
        self.gate.load(input)?;
        self.graphics.load(input)?;
        self.drive.load(input)?;
        self.z80_bus_requested = input.u8()? != 0;
        self.z80_running = input.u8()? != 0;
        self.z80_bank = input.u32()? & 0x01ff;
        Ok(())
    }
}

struct SegaCdMainBus<'a> {
    board: &'a mut SegaCdBoard,
}

impl SegaCdMainBus<'_> {
    fn read_main8(&mut self, address: u32) -> u8 {
        let address = address & 0x00ff_ffff;
        match address {
            0x000000..=0x1fffff => {
                let local = address & 0x3ffff;
                if local < 0x20000 {
                    self.board.bootrom[local as usize & 0x1ffff]
                } else if self.board.gate.main_prg_access() {
                    let base = self.board.gate.prg_bank() * 0x20000;
                    self.board.prg_ram[base + (local as usize & 0x1ffff)]
                } else {
                    0xff
                }
            }
            0x200000..=0x3fffff if self.board.gate.main_has_word_ram() => {
                self.board.word_ram[self.board.gate.main_word_offset(address)]
            }
            0xa00000..=0xa01fff if self.board.z80_bus_requested => {
                self.board.z80_ram[address as usize & 0x1fff]
            }
            0xa04000..=0xa04003 if self.board.z80_bus_requested => self.board.ym.read_status(),
            0xa10000..=0xa1001f => self.board.io.read(address - 0xa10000),
            0xa11100..=0xa11101 => u8::from(!self.board.z80_bus_requested),
            0xa11200..=0xa11201 => u8::from(self.board.z80_running),
            0xa12000..=0xa120ff => self.board.gate.byte((address - 0xa12000) as u16),
            0xc00000..=0xc0001f => {
                let word = self.board.read_vdp_word(address & !1);
                if address & 1 == 0 {
                    (word >> 8) as u8
                } else {
                    word as u8
                }
            }
            0xff0000..=0xffffff => self.board.main_ram[address as usize & 0xffff],
            _ => 0xff,
        }
    }

    fn write_main8(&mut self, address: u32, value: u8) {
        let address = address & 0x00ff_ffff;
        match address {
            0x020000..=0x1fffff if self.board.gate.main_prg_access() => {
                let local = address & 0x3ffff;
                if local >= 0x20000 {
                    let base = self.board.gate.prg_bank() * 0x20000;
                    self.board.prg_ram[base + (local as usize & 0x1ffff)] = value;
                }
            }
            0x200000..=0x3fffff if self.board.gate.main_has_word_ram() => {
                let offset = self.board.gate.main_word_offset(address);
                self.board.word_ram[offset] = value;
            }
            0xa00000..=0xa01fff if self.board.z80_bus_requested => {
                self.board.z80_ram[address as usize & 0x1fff] = value
            }
            0xa04000..=0xa04003 if self.board.z80_bus_requested => {
                self.board.ym.write_port((address & 3) as u8, value)
            }
            0xa06000..=0xa060ff if self.board.z80_bus_requested => {
                self.board.z80_bank =
                    ((self.board.z80_bank >> 1) | (u32::from(value & 1) << 8)) & 0x01ff
            }
            0xa10000..=0xa1001f => self.board.io.write(address - 0xa10000, value),
            0xa11100..=0xa11101 => self.board.z80_bus_requested = value & 1 != 0,
            0xa11200..=0xa11201 => self.board.z80_running = value & 1 != 0,
            0xa12000..=0xa120ff => self
                .board
                .gate
                .main_write((address - 0xa12000) as u16, value),
            0xc00000..=0xc0001f => self
                .board
                .write_vdp_word(address & !1, u16::from(value) * 0x0101),
            0xff0000..=0xffffff => self.board.main_ram[address as usize & 0xffff] = value,
            _ => {}
        }
    }
}

impl Bus68000 for SegaCdMainBus<'_> {
    fn read8(&mut self, address: u32) -> u8 {
        self.read_main8(address)
    }
    fn write8(&mut self, address: u32, value: u8) {
        self.write_main8(address, value);
    }
    fn tas_write8(&mut self, _address: u32, _value: u8) {}
    fn read16(&mut self, address: u32) -> u16 {
        let address = address & 0x00ff_ffff;
        match address {
            0xc00000..=0xc0001f => self.board.read_vdp_word(address),
            _ => u16::from_be_bytes([
                self.read_main8(address),
                self.read_main8(address.wrapping_add(1)),
            ]),
        }
    }
    fn write16(&mut self, address: u32, value: u16) {
        let address = address & 0x00ff_ffff;
        match address {
            0xa11100 => self.board.z80_bus_requested = value & 0x0100 != 0,
            0xa11200 => self.board.z80_running = value & 0x0100 != 0,
            0xc00000..=0xc0001f => self.board.write_vdp_word(address, value),
            _ => {
                let [high, low] = value.to_be_bytes();
                self.write_main8(address, high);
                self.write_main8(address.wrapping_add(1), low);
            }
        }
    }
}

struct SegaCdSubBus<'a> {
    board: &'a mut SegaCdBoard,
}

impl SegaCdSubBus<'_> {
    fn read_sub8(&mut self, address: u32) -> u8 {
        let local = address & 0x000f_ffff;
        match local {
            0x00000..=0x7ffff => self.board.prg_ram[local as usize],
            0x80000..=0xbffff if self.board.gate.sub_has_word_ram() => {
                self.board.word_ram[self.board.gate.sub_word_offset(local)]
            }
            0xe0000..=0xeffff if local & 1 != 0 => self.board.bram[(local as usize >> 1) & 0x1fff],
            0xf0000..=0xf7fff => self.board.pcm.read(((local >> 1) & 0x1fff) as u16),
            0xf8000..=0xfffff => self.board.gate.byte((local & 0x01ff) as u16),
            _ => 0xff,
        }
    }

    fn write_sub8(&mut self, address: u32, value: u8) {
        let local = address & 0x000f_ffff;
        match local {
            0x00000..=0x7ffff => self.board.prg_ram[local as usize] = value,
            0x80000..=0xbffff if self.board.gate.sub_has_word_ram() => {
                let offset = self.board.gate.sub_word_offset(local);
                self.board.word_ram[offset] = value;
            }
            0xe0000..=0xeffff if local & 1 != 0 => {
                self.board.bram[(local as usize >> 1) & 0x1fff] = value
            }
            0xf0000..=0xf7fff => self.board.pcm.write(((local >> 1) & 0x1fff) as u16, value),
            0xf8000..=0xfffff => {
                let offset = (local & 0x01ff) as u16;
                self.board.gate.sub_write(offset, value);
                if matches!(offset, 0x4a | 0x4b) {
                    self.board.process_cdd_command();
                }
                if offset == 0x67 {
                    self.board.graphics.start(&mut self.board.gate);
                }
            }
            _ => {}
        }
    }
}

impl Bus68000 for SegaCdSubBus<'_> {
    fn read8(&mut self, address: u32) -> u8 {
        self.read_sub8(address)
    }
    fn write8(&mut self, address: u32, value: u8) {
        self.write_sub8(address, value);
    }
    fn read16(&mut self, address: u32) -> u16 {
        let local = address & 0x000f_ffff;
        if (0xf8000..=0xfffff).contains(&local) && (local & 0x01fe) == 0x0008 {
            return self.board.drive.read_host_word();
        }
        u16::from_be_bytes([
            self.read_sub8(address),
            self.read_sub8(address.wrapping_add(1)),
        ])
    }
}

struct SegaCdZ80Bridge<'a> {
    board: &'a mut SegaCdBoard,
}

impl Z80Bus for SegaCdZ80Bridge<'_> {
    fn mem_read(&mut self, address: u16) -> u8 {
        match address {
            0x0000..=0x3fff => self.board.z80_ram[usize::from(address) & 0x1fff],
            0x4000..=0x4003 => self.board.ym.read_status(),
            0x8000..=0xffff => self.board.zbank_read(address),
            _ => 0xff,
        }
    }
    fn mem_write(&mut self, address: u16, value: u8) {
        match address {
            0x0000..=0x3fff => self.board.z80_ram[usize::from(address) & 0x1fff] = value,
            0x4000..=0x4003 => self.board.ym.write_port((address & 3) as u8, value),
            0x6000..=0x60ff => {
                self.board.z80_bank =
                    ((self.board.z80_bank >> 1) | (u32::from(value & 1) << 8)) & 0x01ff
            }
            0x7f00..=0x7fff if address & 0x1f == 0x11 => self.board.psg.write(value),
            0x8000..=0xffff => self.board.zbank_write(address, value),
            _ => {}
        }
    }
}

pub struct SegaCdMachine {
    main_cpu: M68000,
    sub_cpu: M68000,
    z80: Z80,
    board: SegaCdBoard,
    audio: AudioBuffer,
    sub_phase: u64,
    sub_credit: i64,
    z80_phase: u64,
    z80_credit: i64,
    powered: bool,
}

impl SegaCdMachine {
    pub fn from_bios_and_disc(bootrom: &[u8], disc: ResourceBlob) -> Result<Self, String> {
        let mut board = SegaCdBoard::new(bootrom, disc)?;
        let mut main_cpu = M68000::default();
        main_cpu.reset(&mut SegaCdMainBus { board: &mut board });
        let mut sub_cpu = M68000::default();
        sub_cpu.reset(&mut SegaCdSubBus { board: &mut board });
        let mut z80 = Z80::default();
        z80.reset();
        Ok(Self {
            main_cpu,
            sub_cpu,
            z80,
            board,
            audio: AudioBuffer::new(AUDIO_RATE, 2),
            sub_phase: 0,
            sub_credit: 0,
            z80_phase: 0,
            z80_credit: 0,
            powered: true,
        })
    }

    fn run_sub_for_main_cycles(&mut self, main_cycles: u32) {
        self.sub_phase = self
            .sub_phase
            .saturating_add(u64::from(main_cycles) * SUB_HZ);
        let due = self.sub_phase / MAIN_HZ;
        self.sub_phase %= MAIN_HZ;
        self.sub_credit = self.sub_credit.saturating_add(due as i64);

        if self.board.gate.sub_reset_pulse {
            self.sub_cpu.reset(&mut SegaCdSubBus {
                board: &mut self.board,
            });
            self.board.gate.sub_reset_pulse = false;
        }
        if !self.board.gate.sub_running() {
            self.sub_credit = self.sub_credit.min(0);
            return;
        }
        while self.sub_credit > 0 {
            let used = self.sub_cpu.step(&mut SegaCdSubBus {
                board: &mut self.board,
            });
            if used == 0 {
                break;
            }
            self.sub_credit -= i64::from(used);
            self.board.tick_sub_cycles(used);
            let level = self.board.gate.highest_sub_irq();
            if level != 0 {
                let interrupt_cycles = self.sub_cpu.interrupt(
                    &mut SegaCdSubBus {
                        board: &mut self.board,
                    },
                    level,
                    24 + level,
                );
                if interrupt_cycles != 0 {
                    self.board.gate.acknowledge_sub_irq(level);
                    self.board.tick_sub_cycles(interrupt_cycles);
                    self.sub_credit -= i64::from(interrupt_cycles);
                }
            }
            if self.sub_credit < -128 {
                self.sub_credit = -128;
            }
        }
    }

    fn run_z80_for_main_cycles(&mut self, main_cycles: u32) {
        self.z80_phase = self
            .z80_phase
            .saturating_add(u64::from(main_cycles) * Z80_HZ);
        let due = self.z80_phase / MAIN_HZ;
        self.z80_phase %= MAIN_HZ;
        self.z80_credit = self.z80_credit.saturating_add(due as i64);
        self.board.psg.tick_cpu_cycles(due as u32);
        if !self.board.z80_running {
            self.z80.reset();
            self.z80_credit = 0;
            return;
        }
        if self.board.z80_bus_requested {
            return;
        }
        while self.z80_credit > 0 {
            let used = self.z80.step(&mut SegaCdZ80Bridge {
                board: &mut self.board,
            });
            if used == 0 {
                break;
            }
            self.z80_credit -= i64::from(used);
            if self.z80_credit < -32 {
                self.z80_credit = -32;
            }
        }
    }

    fn clock_main_instruction(&mut self) -> u32 {
        if let Some(used) = self.board.service_vdp_dma() {
            self.main_cpu.cycles = self.main_cpu.cycles.wrapping_add(u64::from(used));
            self.board.tick_main_cycles(used);
            self.run_sub_for_main_cycles(used);
            self.run_z80_for_main_cycles(used);
            return used;
        }

        let used = self.main_cpu.step(&mut SegaCdMainBus {
            board: &mut self.board,
        });
        if used == 0 {
            return 0;
        }
        self.board.tick_main_cycles(used);
        self.run_sub_for_main_cycles(used);
        self.run_z80_for_main_cycles(used);
        let level = self.board.vdp.irq_level();
        if level != 0 {
            let interrupt_cycles = self.main_cpu.interrupt(
                &mut SegaCdMainBus {
                    board: &mut self.board,
                },
                level,
                24 + level,
            );
            if interrupt_cycles != 0 {
                self.board.vdp.acknowledge_irq(level);
                self.board.tick_main_cycles(interrupt_cycles);
                self.run_sub_for_main_cycles(interrupt_cycles);
                self.run_z80_for_main_cycles(interrupt_cycles);
            }
        }
        used
    }

    fn begin_audio_frame(&mut self) {
        self.board.ym.begin_frame();
        self.board.psg.begin_frame();
        self.board.pcm.begin_frame();
        self.board.drive.begin_frame();
        self.audio.begin_frame();
    }

    fn flush_audio(&mut self) {
        self.audio.begin_frame();
        let fm = self.board.ym.samples();
        let psg = self.board.psg.samples();
        let pcm = &self.board.pcm.samples;
        let cdda = &self.board.drive.cdda_samples;
        let len = fm.len().max(psg.len()).max(pcm.len()).max(cdda.len());
        for index in 0..len {
            let (fm_l, fm_r) = fm.get(index).copied().unwrap_or((0.0, 0.0));
            let tone = psg.get(index).copied().unwrap_or(0.0) * 0.35;
            let (pcm_l, pcm_r) = pcm.get(index).copied().unwrap_or((0.0, 0.0));
            let (cdda_l, cdda_r) = cdda.get(index).copied().unwrap_or((0.0, 0.0));
            self.audio.push_stereo(
                (fm_l + tone + pcm_l * 0.55 + cdda_l).clamp(-1.0, 1.0),
                (fm_r + tone + pcm_r * 0.55 + cdda_r).clamp(-1.0, 1.0),
            );
        }
    }
}

impl Machine for SegaCdMachine {
    fn platform(&self) -> PlatformId {
        PlatformId::SegaCd
    }

    fn reset(&mut self) {
        self.board.reset_devices();
        self.main_cpu.reset(&mut SegaCdMainBus {
            board: &mut self.board,
        });
        self.sub_cpu.reset(&mut SegaCdSubBus {
            board: &mut self.board,
        });
        self.z80.reset();
        self.sub_phase = 0;
        self.sub_credit = 0;
        self.z80_phase = 0;
        self.z80_credit = 0;
        self.powered = true;
        self.begin_audio_frame();
    }

    fn run_frame(&mut self, input: &InputState) {
        self.board.io.set_input(input);
        self.begin_audio_frame();
        let target = self.board.vdp.frame().wrapping_add(1);
        let deadline = self
            .main_cpu
            .cycles
            .saturating_add((MAIN_HZ as f64 / FRAME_RATE * 2.0).ceil() as u64);
        while self.board.vdp.frame() != target && self.main_cpu.cycles < deadline {
            if self.clock_main_instruction() == 0 {
                self.powered = false;
                break;
            }
        }
        if self.board.vdp.frame() != target {
            self.powered = false;
        }
        self.flush_audio();
    }

    fn frame_rate(&self) -> f64 {
        FRAME_RATE
    }
    fn video(&self) -> &VideoBuffer {
        self.board.vdp.video()
    }
    fn audio(&self) -> &AudioBuffer {
        &self.audio
    }

    fn save_state(&self) -> Result<Vec<u8>, String> {
        let mut out = StateWriter::new(PlatformId::SegaCd, STATE_VERSION);
        self.main_cpu.save(&mut out);
        self.sub_cpu.save(&mut out);
        self.z80.save(&mut out);
        self.board.save(&mut out);
        out.u64(self.sub_phase);
        out.u64(self.sub_credit as u64);
        out.u64(self.z80_phase);
        out.u64(self.z80_credit as u64);
        out.u8(u8::from(self.powered));
        Ok(out.finish())
    }

    fn load_state(&mut self, bytes: &[u8]) -> Result<(), String> {
        let mut input = StateReader::new(bytes, PlatformId::SegaCd, STATE_VERSION)?;
        self.main_cpu.load(&mut input)?;
        self.sub_cpu.load(&mut input)?;
        self.z80.load(&mut input)?;
        self.board.load(&mut input)?;
        self.sub_phase = input.u64()? % MAIN_HZ;
        self.sub_credit = input.u64()? as i64;
        self.z80_phase = input.u64()? % MAIN_HZ;
        self.z80_credit = input.u64()? as i64;
        self.powered = input.u8()? != 0;
        self.begin_audio_frame();
        input.finish()
    }

    fn persistent_len(&self, kind: ResourceKind, slot: u32) -> usize {
        if kind == ResourceKind::Storage && slot == 0 {
            self.board.bram.len()
        } else {
            0
        }
    }

    fn read_persistent(&self, kind: ResourceKind, slot: u32, out: &mut [u8]) -> Result<(), String> {
        if kind != ResourceKind::Storage || slot != 0 {
            return Err("Sega CD persistent resource is Storage slot 0".into());
        }
        if out.len() != self.board.bram.len() {
            return Err("Sega CD backup RAM output length mismatch".into());
        }
        out.copy_from_slice(self.board.bram.as_slice());
        Ok(())
    }

    fn write_persistent(
        &mut self,
        kind: ResourceKind,
        slot: u32,
        data: &[u8],
    ) -> Result<(), String> {
        if kind != ResourceKind::Storage || slot != 0 {
            return Err("Sega CD persistent resource is Storage slot 0".into());
        }
        if data.len() != self.board.bram.len() {
            return Err("Sega CD backup RAM input length mismatch".into());
        }
        self.board.bram.copy_from_slice(data);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn write_word(bytes: &mut [u8], offset: usize, value: u16) {
        bytes[offset..offset + 2].copy_from_slice(&value.to_be_bytes());
    }

    fn write_long(bytes: &mut [u8], offset: usize, value: u32) {
        bytes[offset..offset + 4].copy_from_slice(&value.to_be_bytes());
    }

    fn write_dot(bytes: &mut [u8], pixel_index: u32, value: u8) {
        let offset = ((pixel_index >> 1) as usize) & (bytes.len() - 1);
        if pixel_index & 1 == 0 {
            bytes[offset] = (bytes[offset] & 0x0f) | ((value & 0x0f) << 4);
        } else {
            bytes[offset] = (bytes[offset] & 0xf0) | (value & 0x0f);
        }
    }

    fn synthetic_bios() -> Vec<u8> {
        let mut bios = vec![0xff; BOOT_ROM_SIZE];
        write_long(&mut bios, 0, 0x00ff_ff00);
        write_long(&mut bios, 4, 0x0000_0200);
        let words = [
            0x13fc, 0x0084, 0x00c0, 0x0011, 0x13fc, 0x0010, 0x00c0, 0x0011, 0x13fc, 0x0090, 0x00c0,
            0x0011, 0x60fe,
        ];
        for (index, word) in words.into_iter().enumerate() {
            write_word(&mut bios, 0x200 + index * 2, word);
        }
        bios
    }

    fn synthetic_disc() -> ResourceBlob {
        let mut bytes = vec![0u8; 2048 * 4];
        for sector in 0..4usize {
            bytes[sector * 2048..(sector + 1) * 2048].fill((sector + 1) as u8);
        }
        ResourceBlob::from_bytes(&bytes)
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

    fn pcm_wave(data: &[u8]) -> Vec<u8> {
        let mut wave = vec![0u8; 44 + data.len()];
        wave[0..4].copy_from_slice(b"RIFF");
        wave[4..8].copy_from_slice(&(36u32 + data.len() as u32).to_le_bytes());
        wave[8..12].copy_from_slice(b"WAVE");
        wave[12..16].copy_from_slice(b"fmt ");
        wave[16..20].copy_from_slice(&16u32.to_le_bytes());
        wave[20..22].copy_from_slice(&1u16.to_le_bytes());
        wave[22..24].copy_from_slice(&2u16.to_le_bytes());
        wave[24..28].copy_from_slice(&44_100u32.to_le_bytes());
        wave[28..32].copy_from_slice(&176_400u32.to_le_bytes());
        wave[32..34].copy_from_slice(&4u16.to_le_bytes());
        wave[34..36].copy_from_slice(&16u16.to_le_bytes());
        wave[36..40].copy_from_slice(b"data");
        wave[40..44].copy_from_slice(&(data.len() as u32).to_le_bytes());
        wave[44..].copy_from_slice(data);
        wave
    }

    #[test]
    fn graphics_asic_register_path_renders_stamp_and_completes_level1() {
        let mut board = SegaCdBoard::new(&synthetic_bios(), synthetic_disc()).unwrap();
        board.word_ram[0x80..0x84].copy_from_slice(&[0x12, 0x34, 0x56, 0x78]);
        write_word(board.word_ram.as_mut_slice(), 0x200, 0x0001);
        write_word(board.word_ram.as_mut_slice(), 0x400, 0x0000);
        write_word(board.word_ram.as_mut_slice(), 0x402, 0x0000);
        write_word(board.word_ram.as_mut_slice(), 0x404, 0x0800);
        write_word(board.word_ram.as_mut_slice(), 0x406, 0x0000);

        {
            let mut bus = SegaCdSubBus { board: &mut board };
            bus.write8(0xff8003, 0x00);
            bus.write8(0xff8033, 0x02);
            bus.write16(0xff8058, 0x0000);
            bus.write16(0xff805a, 0x0080);
            bus.write16(0xff805c, 0x0000);
            bus.write16(0xff805e, 0x0200);
            bus.write16(0xff8060, 0x0000);
            bus.write16(0xff8062, 0x0008);
            bus.write16(0xff8064, 0x0001);
            bus.write16(0xff8066, 0x0100);
        }

        assert!(board.graphics.active);
        assert_eq!(board.gate.byte(0x58), 0x80);
        assert_eq!(board.gate.byte(0x65), 1);
        board.tick_sub_cycles(1);
        assert_eq!(&board.word_ram[0x800..0x804], &[0x12, 0x34, 0x56, 0x78]);
        assert!(!board.graphics.active);
        assert_eq!(board.gate.byte(0x58), 0);
        assert_eq!(board.gate.byte(0x65), 0);
        assert_ne!(board.gate.pending & (1 << 1), 0);
    }

    #[test]
    fn graphics_asic_halts_when_main_owns_2m_word_ram_then_resumes() {
        let mut board = SegaCdBoard::new(&synthetic_bios(), synthetic_disc()).unwrap();
        write_word(board.word_ram.as_mut_slice(), 0x200, 0x0001);
        write_dot(board.word_ram.as_mut_slice(), 0x100, 0x0d);
        write_word(board.word_ram.as_mut_slice(), 0x400, 0x0000);
        write_word(board.word_ram.as_mut_slice(), 0x402, 0x0000);
        write_word(board.word_ram.as_mut_slice(), 0x404, 0x0000);
        write_word(board.word_ram.as_mut_slice(), 0x406, 0x0000);

        board.gate.set_byte(0x03, 0x01);
        board.gate.regs[0x5a >> 1] = 0x0080;
        board.gate.regs[0x5e >> 1] = 0x0200;
        board.gate.regs[0x62 >> 1] = 1;
        board.gate.regs[0x64 >> 1] = 1;
        board.gate.regs[0x66 >> 1] = 0x0100;
        board.graphics.start(&mut board.gate);
        board.tick_sub_cycles(100_000);
        assert!(board.graphics.active);
        assert_eq!(board.gate.byte(0x65), 1);
        assert_eq!(board.word_ram[0x800] >> 4, 0);

        board.gate.set_byte(0x03, 0x00);
        board.tick_sub_cycles(1);
        assert!(!board.graphics.active);
        assert_eq!(board.word_ram[0x800] >> 4, 0x0d);
    }

    #[test]
    fn graphics_asic_geometry_transform_and_priority_modes_match_stamp_rules() {
        let mut gate = SegaCdGate::default();
        gate.regs[0x5a >> 1] = 0xffff;
        gate.regs[0x62 >> 1] = 8;
        gate.regs[0x64 >> 1] = 1;
        for (config, dot_mask, stamp_shift, map_shift, map_base) in [
            (0x00, 0x07ffff, 15, 4, 0x3fe00usize),
            (0x02, 0x07ffff, 16, 3, 0x3ff80usize),
            (0x04, 0x7fffff, 15, 8, 0x20000usize),
            (0x06, 0x7fffff, 16, 7, 0x38000usize),
        ] {
            gate.set_byte(0x59, config);
            let mut graphics = SegaCdGraphics::default();
            graphics.start(&mut gate);
            assert_eq!(graphics.dot_mask, dot_mask);
            assert_eq!(graphics.stamp_shift, stamp_shift);
            assert_eq!(graphics.map_shift, map_shift);
            assert_eq!(graphics.map_base, map_base);
        }

        let graphics = SegaCdGraphics::default();
        let mut word_ram = vec![0u8; WORD_RAM_SIZE];
        write_dot(&mut word_ram, 0x400, 1);
        write_dot(&mut word_ram, 0x707, 2);
        assert_eq!(graphics.transformed_pixel(&word_ram, 0x0004, 0, 0, true), 1);
        assert_eq!(graphics.transformed_pixel(&word_ram, 0x0005, 0, 0, true), 1);
        assert_eq!(graphics.transformed_pixel(&word_ram, 0x8004, 0, 0, true), 2);

        assert_eq!(SegaCdGraphics::merge_priority(0xa5, 0xc0, 0), 0xc0);
        assert_eq!(SegaCdGraphics::merge_priority(0xa5, 0xc0, 1), 0xa5);
        assert_eq!(SegaCdGraphics::merge_priority(0xa5, 0xc0, 2), 0xc5);
        assert_eq!(SegaCdGraphics::merge_priority(0xa5, 0xc0, 3), 0xa5);
    }

    #[test]
    fn disc_reads_iso_sectors_without_whole_image_copy() {
        let disc = SegaCdDisc::new(synthetic_disc()).unwrap();
        let mut sector = [0u8; 2352];
        let len = disc.read_user_sector(2, &mut sector).unwrap();
        assert_eq!(len, 2048);
        assert!(sector[..len].iter().all(|byte| *byte == 3));
    }

    #[test]
    fn multitrack_cue_routes_data_and_wave_audio_tracks() {
        let mut data = vec![0u8; 2048 * 2];
        data[..2048].fill(0x11);
        data[2048..].fill(0x22);
        let audio = vec![0x33u8; 2352];
        let cue = concat!(
            "FILE \"data.bin\" BINARY\n",
            "  TRACK 01 MODE1/2048\n",
            "    INDEX 01 00:00:00\n",
            "FILE \"audio.wav\" WAVE\n",
            "  TRACK 02 AUDIO\n",
            "    INDEX 01 00:00:00\n",
        );
        let disc = SegaCdDisc::new(cue_container(
            cue,
            &[("data.bin", data), ("audio.wav", pcm_wave(&audio))],
        ))
        .unwrap();

        assert_eq!(disc.image.tracks.len(), 2);
        assert_eq!(disc.image.tracks[0].number, 1);
        assert_eq!(disc.image.tracks[0].start_lba, 0);
        assert_eq!(disc.image.tracks[1].number, 2);
        assert_eq!(disc.image.tracks[1].start_lba, 2);
        assert_eq!(disc.image.tracks[1].kind, crate::cd_image::TrackKind::Audio);
        assert_eq!(disc.sectors, 3);

        let mut sector = [0u8; 2352];
        let len = disc.read_user_sector(0, &mut sector).unwrap();
        assert_eq!(len, 2048);
        assert!(sector[..len].iter().all(|byte| *byte == 0x11));
        let len = disc.read_user_sector(1, &mut sector).unwrap();
        assert_eq!(len, 2048);
        assert!(sector[..len].iter().all(|byte| *byte == 0x22));
        let len = disc.read_user_sector(2, &mut sector).unwrap();
        assert_eq!(len, 2352);
        assert!(sector[..len].iter().all(|byte| *byte == 0x33));
    }

    #[test]
    fn cdd_commands_use_digit_nibbles_and_report_multitrack_toc() {
        let mut data = vec![0u8; 2048 * 2];
        data[..2048].fill(0x11);
        data[2048..].fill(0x22);
        let audio = vec![0x33u8; 2352];
        let cue = concat!(
            "FILE \"data.bin\" BINARY\n",
            "  TRACK 01 MODE1/2048\n",
            "    INDEX 01 00:00:00\n",
            "FILE \"audio.wav\" WAVE\n",
            "  TRACK 02 AUDIO\n",
            "    INDEX 01 00:00:00\n",
        );
        let disc = cue_container(cue, &[("data.bin", data), ("audio.wav", pcm_wave(&audio))]);
        let mut board = SegaCdBoard::new(&synthetic_bios(), disc).unwrap();

        board.gate.set_byte(0x42, 0x02);
        board.gate.set_byte(0x45, 0x04);
        board.process_cdd_command();
        assert_eq!(board.gate.byte(0x38), CDD_TOC);
        assert_eq!(board.gate.byte(0x39), 0x04);
        assert_eq!((board.gate.byte(0x3a), board.gate.byte(0x3b)), (0, 1));
        assert_eq!((board.gate.byte(0x3c), board.gate.byte(0x3d)), (0, 2));
        let checksum_sum = [0x38u16, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f, 0x40]
            .into_iter()
            .fold(0u8, |sum, offset| sum.wrapping_add(board.gate.byte(offset)));
        assert_eq!(board.gate.byte(0x41), (!checksum_sum) & 0x0f);

        for (offset, digit) in [
            (0x44, 0),
            (0x45, 0),
            (0x46, 0),
            (0x47, 2),
            (0x48, 0),
            (0x49, 2),
        ] {
            board.gate.set_byte(offset, digit);
        }
        board.gate.set_byte(0x42, 0x03);
        board.process_cdd_command();
        assert_eq!(board.drive.current_lba, 2);
        assert!(board.drive.reading);
        assert_eq!(board.drive.status, CDD_PLAY);
        assert_eq!(board.gate.byte(0x38), CDD_PLAY);
        assert_eq!(board.gate.byte(0x40), 0x00);

        board.gate.set_byte(0x42, 0x06);
        board.process_cdd_command();
        assert!(!board.drive.reading);
        assert_eq!(board.drive.current_lba, 2);
        assert_eq!(board.drive.status, CDD_PAUSE);

        board.gate.set_byte(0x42, 0x07);
        board.process_cdd_command();
        assert!(board.drive.reading);
        assert_eq!(board.drive.status, CDD_PLAY);

        board.gate.set_byte(0x42, 0x02);
        board.gate.set_byte(0x45, 0x05);
        board.gate.set_byte(0x46, 0x00);
        board.gate.set_byte(0x47, 0x02);
        board.process_cdd_command();
        assert_eq!(board.gate.byte(0x39), 0x05);
        assert_eq!((board.gate.byte(0x3a), board.gate.byte(0x3b)), (0, 0));
        assert_eq!((board.gate.byte(0x3c), board.gate.byte(0x3d)), (0, 2));
        assert_eq!((board.gate.byte(0x3e), board.gate.byte(0x3f)), (0, 2));
        assert_eq!(board.gate.byte(0x40), 2);
    }

    #[test]
    fn cdda_streams_wave_audio_at_48khz_and_pause_resume_preserves_position() {
        let mut audio = Vec::with_capacity(2352);
        for _ in 0..CDDA_FRAMES_PER_SECTOR {
            audio.extend_from_slice(&8192i16.to_le_bytes());
            audio.extend_from_slice(&(-8192i16).to_le_bytes());
        }
        let cue = concat!(
            "FILE \"audio.wav\" WAVE\n",
            "  TRACK 01 AUDIO\n",
            "    INDEX 01 00:00:00\n",
        );
        let disc = cue_container(cue, &[("audio.wav", pcm_wave(&audio))]);
        let mut drive = SegaCdDrive::new(disc).unwrap();
        drive.start_read(0);
        drive.begin_frame();

        let millisecond = (SUB_HZ / 1000) as u32;
        drive.tick(millisecond);
        assert!((47..=49).contains(&drive.cdda_samples.len()));
        assert!(drive
            .cdda_samples
            .iter()
            .all(|sample| (sample.0 - 0.25).abs() < 0.0001 && (sample.1 + 0.25).abs() < 0.0001));
        assert_eq!(drive.current_lba, 1);
        assert!(drive.cdda_sample_index > 0);
        assert!(drive.cdda_sample_index < CDDA_FRAMES_PER_SECTOR);

        let paused_index = drive.cdda_sample_index;
        drive.pause();
        drive.begin_frame();
        drive.tick(millisecond);
        assert_eq!(drive.cdda_sample_index, paused_index);
        assert!(drive
            .cdda_samples
            .iter()
            .all(|sample| *sample == (0.0, 0.0)));

        drive.resume();
        assert!(drive.reading);
        assert_eq!(drive.status, CDD_PLAY);
        drive.begin_frame();
        drive.tick(millisecond);
        assert!(drive.cdda_sample_index > paused_index);
        assert!(drive.cdda_samples.iter().any(|sample| sample.0 != 0.0));

        let mut writer = StateWriter::new(PlatformId::SegaCd, 99);
        drive.save(&mut writer);
        let state = writer.finish();
        let expected_index = drive.cdda_sample_index;
        let expected_current = drive.cdda_current;
        let mut restored =
            SegaCdDrive::new(cue_container(cue, &[("audio.wav", pcm_wave(&audio))])).unwrap();
        let mut reader = StateReader::new(&state, PlatformId::SegaCd, 99).unwrap();
        restored.load(&mut reader).unwrap();
        reader.finish().unwrap();
        assert_eq!(restored.cdda_sample_index, expected_index);
        assert_eq!(restored.cdda_current, expected_current);
        assert!(restored.cdda_samples.is_empty());
    }

    #[test]
    fn raw_disc_extracts_mode1_and_mode2_form_payloads() {
        let mut bytes = vec![0u8; 2352 * 3];
        for sector in 0..3usize {
            let base = sector * 2352;
            bytes[base + 1..base + 11].fill(0xff);
            bytes[base + 11] = 0;
        }

        bytes[15] = 1;
        bytes[16..16 + 2048].fill(0x11);

        let mode2_form1 = 2352;
        bytes[mode2_form1 + 15] = 2;
        bytes[mode2_form1 + 24..mode2_form1 + 24 + 2048].fill(0x22);

        let mode2_form2 = 2352 * 2;
        bytes[mode2_form2 + 15] = 2;
        bytes[mode2_form2 + 18] = 0x20;
        bytes[mode2_form2 + 22] = 0x20;
        bytes[mode2_form2 + 24..mode2_form2 + 24 + 2324].fill(0x33);

        let disc = SegaCdDisc::new(ResourceBlob::from_bytes(&bytes)).unwrap();
        assert_eq!(disc.image.tracks[0].layout, DiscLayout::Raw2352Mode1);
        let mut sector = [0u8; 2352];

        let len = disc.read_user_sector(0, &mut sector).unwrap();
        assert_eq!(len, 2048);
        assert!(sector[..len].iter().all(|byte| *byte == 0x11));

        let len = disc.read_user_sector(1, &mut sector).unwrap();
        assert_eq!(len, 2048);
        assert!(sector[..len].iter().all(|byte| *byte == 0x22));

        let len = disc.read_user_sector(2, &mut sector).unwrap();
        assert_eq!(len, 2324);
        assert!(sector[..len].iter().all(|byte| *byte == 0x33));
    }

    #[test]
    fn streaming_disc_reports_missing_sector_range() {
        let blob = ResourceBlob::streaming((2048 * 8) as u64, 2).unwrap();
        let disc = SegaCdDisc::new(blob.clone()).unwrap();
        let mut sector = [0u8; 2352];
        assert!(disc.read_user_sector(3, &mut sector).is_err());
        assert_eq!(blob.pending_range(), Some((6144, 8192)));
    }

    #[test]
    fn drive_preserves_unread_sector_and_rejects_out_of_range_seek() {
        let mut drive = SegaCdDrive::new(synthetic_disc()).unwrap();
        let period = (SUB_HZ / 75) as u32;
        drive.start_read(0);
        drive.tick(period * 3);
        assert!(drive.sector_ready);
        assert_eq!(drive.current_lba, 1);
        assert_eq!(drive.read_host_word(), 0x0101);

        for _ in 1..1024 {
            drive.read_host_word();
        }
        assert!(!drive.sector_ready);
        drive.tick(0);
        assert!(drive.sector_ready);
        assert_eq!(drive.current_lba, 2);
        assert_eq!(drive.read_host_word(), 0x0202);

        drive.start_read(99);
        assert!(!drive.reading);
        assert!(drive.end_of_disc);
        assert!(!drive.sector_ready);
        assert_eq!(drive.current_lba, 99);
    }

    #[test]
    fn pcm_channel_uses_wave_ram_loop_and_stereo_pan() {
        let mut pcm = SegaCdPcm::default();
        pcm.write(0x07, 0x00);
        for index in 0..32u16 {
            pcm.write(0x1000 + index, 0xc0 + (index as u8 & 0x1f));
        }
        pcm.write(0x07, 0xc0);
        pcm.write(0x00, 0xff);
        pcm.write(0x01, 0xff);
        pcm.write(0x02, 0x00);
        pcm.write(0x03, 0x08);
        pcm.write(0x04, 0x00);
        pcm.write(0x05, 0x00);
        pcm.write(0x06, 0x00);
        pcm.write(0x08, 0xfe);
        pcm.begin_frame();
        pcm.tick(384 * 64);
        assert!(pcm
            .samples
            .iter()
            .any(|&(left, right)| left > 0.0 && right > 0.0));
    }

    #[test]
    fn gate_array_switches_word_ram_and_releases_sub_cpu() {
        let mut gate = SegaCdGate::default();
        assert!(gate.main_prg_access());
        assert!(gate.main_has_word_ram());
        assert!(!gate.sub_has_word_ram());
        gate.main_write(0x03, 0x00);
        assert!(!gate.main_has_word_ram());
        assert!(gate.sub_has_word_ram());
        gate.main_write(0x01, 0x01);
        assert!(gate.sub_running());
        assert!(gate.sub_reset_pulse);
    }

    #[test]
    fn vdp_dma_reads_main_work_ram_through_sega_cd_bus() {
        let mut machine =
            SegaCdMachine::from_bios_and_disc(&synthetic_bios(), synthetic_disc()).unwrap();
        machine.board.main_ram[..4].copy_from_slice(&[0x11, 0x22, 0x33, 0x44]);
        {
            let mut bus = SegaCdMainBus {
                board: &mut machine.board,
            };
            for (register, value) in [
                (1u16, 0x10u16),
                (15, 0x02),
                (19, 0x02),
                (20, 0x00),
                (21, 0x00),
                (22, 0x80),
                (23, 0x7f),
            ] {
                bus.write16(0x00c0_0004, 0x8000 | (register << 8) | value);
            }
            bus.write16(0x00c0_0004, 0x4100);
            bus.write16(0x00c0_0004, 0x0080);
            assert_ne!(bus.read16(0x00c0_0004) & 0x0002, 0);
        }

        let pc = machine.main_cpu.pc;
        assert_ne!(machine.clock_main_instruction(), 0);
        assert_eq!(machine.main_cpu.pc, pc);
        assert!(machine.board.vdp.memory_dma_pending());
        assert_ne!(machine.clock_main_instruction(), 0);
        assert_eq!(machine.main_cpu.pc, pc);
        assert!(!machine.board.vdp.memory_dma_pending());

        let mut bus = SegaCdMainBus {
            board: &mut machine.board,
        };
        bus.write16(0x00c0_0004, 0x0100);
        bus.write16(0x00c0_0004, 0x0000);
        assert_eq!(bus.read16(0x00c0_0000), 0x1122);
        assert_eq!(bus.read16(0x00c0_0000), 0x3344);
        assert_eq!(bus.read16(0x00c0_0004) & 0x0002, 0);
    }

    #[test]
    fn sub_cpu_runs_from_shared_prg_ram_after_main_release() {
        let mut machine =
            SegaCdMachine::from_bios_and_disc(&synthetic_bios(), synthetic_disc()).unwrap();
        write_long(machine.board.prg_ram.as_mut_slice(), 0, 0x0007_ff00);
        write_long(machine.board.prg_ram.as_mut_slice(), 4, 0x0000_0100);
        let words = [0x13fc, 0x005a, 0x00ff, 0x8021, 0x60fe];
        for (index, word) in words.into_iter().enumerate() {
            write_word(
                machine.board.prg_ram.as_mut_slice(),
                0x100 + index * 2,
                word,
            );
        }
        machine.board.gate.main_write(0x03, 0x00);
        machine.board.gate.main_write(0x01, 0x01);
        machine.run_sub_for_main_cycles(2_000);
        assert_eq!(machine.board.gate.byte(0x21), 0x5a);
    }

    #[test]
    fn synthetic_bios_runs_genesis_video_audio_and_cd_state() {
        let mut machine =
            SegaCdMachine::from_bios_and_disc(&synthetic_bios(), synthetic_disc()).unwrap();
        machine.run_frame(&InputState::default());
        assert!(machine.powered);
        assert_eq!(machine.video().width(), 320);
        assert_eq!(machine.video().height(), 224);
        assert_eq!(machine.board.vdp.frame(), 1);
        assert!(machine
            .audio()
            .samples()
            .iter()
            .any(|sample| sample.abs() > 0.001));
        machine.board.bram[7] = 0x5a;
        machine.board.gate.set_byte(0x03, 0x01);
        machine.board.gate.regs[0x5a >> 1] = 0x0080;
        machine.board.gate.regs[0x62 >> 1] = 8;
        machine.board.gate.regs[0x64 >> 1] = 2;
        machine.board.gate.regs[0x66 >> 1] = 0x0100;
        machine.board.graphics.start(&mut machine.board.gate);
        let saved = machine.save_state().unwrap();
        machine.board.bram[7] = 0;
        machine.board.graphics = SegaCdGraphics::default();
        machine.load_state(&saved).unwrap();
        assert_eq!(machine.board.bram[7], 0x5a);
        assert!(machine.board.graphics.active);
        assert_eq!(machine.board.graphics.trace_offset, 0x400);
        assert_eq!(machine.board.gate.byte(0x58), 0x80);
        assert_eq!(machine.save_state().unwrap(), saved);
        assert_eq!(machine.persistent_len(ResourceKind::Storage, 0), BRAM_SIZE);
    }
}
