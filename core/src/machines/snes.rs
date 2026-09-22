use crate::cpu65816::{Bus65816, Cpu65816};
use crate::input::{
    DOWN, FACE_EAST, FACE_NORTH, FACE_SOUTH, FACE_WEST, L1, LEFT, R1, RIGHT, SELECT, START, UP,
};
use crate::kernel::{AudioBuffer, InputState, VideoBuffer};
use crate::machine::Machine;
use crate::platform::PlatformId;
use crate::resources::ResourceKind;
use crate::state::{StateReader, StateWriter};

use super::snes_apu::SnesApu;
use super::snes_dsp1::Dsp1;
use super::snes_dsp2::Dsp2;
use super::snes_dsp3::Dsp3;
use super::snes_ppu::SnesPpu;
use super::snes_srtc::Srtc;

const FRAME_RATE: f64 = 60.098_813_897;
const STATE_VERSION: u32 = 14;
const WRAM_SIZE: usize = 128 * 1024;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(clippy::enum_variant_names)]
enum SnesMap {
    LoRom,
    HiRom,
    ExHiRom,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SnesCoprocessor {
    None,
    Dsp1,
    Dsp2,
    Dsp3,
    Srtc,
    Obc1,
    Sa1,
}

struct Sa1State {
    iram: Vec<u8>,
    ccnt: u8,
    sie: u8,
    crv: u16,
    cnv: u16,
    civ: u16,
    scnt: u8,
    cie: u8,
    snv: u16,
    siv: u16,
    timer_control: u8,
    hcnt: u16,
    vcnt: u16,
    timer_h: u16,
    timer_v: u16,
    banks: [u8; 4],
    bmaps: u8,
    bmap: u8,
    swen: bool,
    cwen: bool,
    bwp: u8,
    siwp: u8,
    ciwp: u8,
    dcnt: u8,
    cdma: u8,
    dsa: u32,
    dda: u32,
    dtc: u16,
    bbf: bool,
    brf: [u8; 16],
    mcnt: u8,
    ma: u16,
    mb: u16,
    mr: u64,
    overflow: bool,
    vbd: u8,
    va: u32,
    vbit: u8,
    cpu_irq_flag: bool,
    chdma_irq_flag: bool,
    sa1_irq_flag: bool,
    timer_irq_flag: bool,
    dma_irq_flag: bool,
    sa1_nmi_flag: bool,
    release_pending: bool,
}

impl Default for Sa1State {
    fn default() -> Self {
        Self {
            iram: vec![0; 0x800],
            ccnt: 0x20,
            sie: 0,
            crv: 0,
            cnv: 0,
            civ: 0,
            scnt: 0,
            cie: 0,
            snv: 0,
            siv: 0,
            timer_control: 0,
            hcnt: 0,
            vcnt: 0,
            timer_h: 0,
            timer_v: 0,
            banks: [0, 1, 2, 3],
            bmaps: 0,
            bmap: 0,
            swen: false,
            cwen: false,
            bwp: 0x0f,
            siwp: 0,
            ciwp: 0,
            dcnt: 0,
            cdma: 0,
            dsa: 0,
            dda: 0,
            dtc: 0,
            bbf: false,
            brf: [0; 16],
            mcnt: 0,
            ma: 0,
            mb: 0,
            mr: 0,
            overflow: false,
            vbd: 0,
            va: 0,
            vbit: 0,
            cpu_irq_flag: false,
            chdma_irq_flag: false,
            sa1_irq_flag: false,
            timer_irq_flag: false,
            dma_irq_flag: false,
            sa1_nmi_flag: false,
            release_pending: false,
        }
    }
}

impl Sa1State {
    fn reset(&mut self) {
        *self = Self::default();
    }

    fn active(&self) -> bool {
        self.ccnt & 0x60 == 0
    }

    fn main_irq_pending(&self) -> bool {
        (self.sie & 0x20 != 0 && self.chdma_irq_flag) || (self.sie & 0x80 != 0 && self.cpu_irq_flag)
    }

    fn sa1_nmi_pending(&self) -> bool {
        self.cie & 0x10 != 0 && self.sa1_nmi_flag
    }

    fn sa1_irq_pending(&self) -> bool {
        (self.cie & 0x20 != 0 && self.dma_irq_flag)
            || (self.cie & 0x40 != 0 && self.timer_irq_flag)
            || (self.cie & 0x80 != 0 && self.sa1_irq_flag)
    }

    fn cpu_status(&self) -> u8 {
        (self.scnt & 0x5f)
            | if self.chdma_irq_flag { 0x20 } else { 0 }
            | if self.cpu_irq_flag { 0x80 } else { 0 }
    }

    fn sa1_status(&self) -> u8 {
        (self.ccnt & 0x0f)
            | if self.sa1_nmi_flag { 0x10 } else { 0 }
            | if self.dma_irq_flag { 0x20 } else { 0 }
            | if self.timer_irq_flag { 0x40 } else { 0 }
            | if self.sa1_irq_flag { 0x80 } else { 0 }
    }

    fn execute_arithmetic(&mut self) {
        if self.mcnt & 0x02 != 0 {
            let product = i64::from(self.ma as i16) * i64::from(self.mb as i16);
            let sum = (self.mr & 0x00ff_ffff_ffff) as i64 + product;
            self.overflow = !(-(1i64 << 39)..(1i64 << 39)).contains(&sum);
            self.mr = (sum as u64) & 0x00ff_ffff_ffff;
            self.mb = 0;
        } else if self.mcnt & 0x01 == 0 {
            self.mr = u64::from((i32::from(self.ma as i16) * i32::from(self.mb as i16)) as u32);
            self.mb = 0;
        } else {
            if self.mb == 0 {
                self.mr = 0;
            } else {
                let dividend = i32::from(self.ma as i16);
                let divisor = i32::from(self.mb);
                let mut remainder = dividend % divisor;
                if remainder < 0 {
                    remainder += divisor;
                }
                let quotient = (dividend - remainder) / divisor;
                self.mr = (u64::from(remainder as u16) << 16) | u64::from(quotient as u16);
            }
            self.ma = 0;
            self.mb = 0;
        }
    }

    fn tick_timer(&mut self, cycles: u32) {
        for _ in 0..cycles {
            if self.timer_control & 0x80 == 0 {
                self.timer_h = self.timer_h.wrapping_add(2);
                if self.timer_h >= 1364 {
                    self.timer_h -= 1364;
                    self.timer_v = (self.timer_v + 1) % 262;
                }
            } else {
                self.timer_h = self.timer_h.wrapping_add(2);
                if self.timer_h >= 2048 {
                    self.timer_h -= 2048;
                    self.timer_v = (self.timer_v + 1) & 0x01ff;
                }
            }
            let h_match = self.timer_h == self.hcnt.wrapping_mul(4);
            let v_match = self.timer_v == self.vcnt;
            let fire = match self.timer_control & 3 {
                0 => false,
                1 => h_match,
                2 => v_match && self.timer_h == 0,
                _ => h_match && v_match,
            };
            if fire {
                self.timer_irq_flag = true;
            }
        }
    }

    fn save(&self, out: &mut StateWriter) {
        out.blob(&self.iram);
        for value in [
            self.ccnt,
            self.sie,
            self.scnt,
            self.cie,
            self.timer_control,
            self.bmaps,
            self.bmap,
            self.bwp,
            self.siwp,
            self.ciwp,
            self.dcnt,
            self.cdma,
            self.bbf as u8,
            self.mcnt,
            self.overflow as u8,
            self.vbd,
            self.vbit,
            self.cpu_irq_flag as u8,
            self.chdma_irq_flag as u8,
            self.sa1_irq_flag as u8,
            self.timer_irq_flag as u8,
            self.dma_irq_flag as u8,
            self.sa1_nmi_flag as u8,
            self.release_pending as u8,
            self.swen as u8,
            self.cwen as u8,
        ] {
            out.u8(value);
        }
        for value in self.banks {
            out.u8(value);
        }
        for value in [
            self.crv,
            self.cnv,
            self.civ,
            self.snv,
            self.siv,
            self.hcnt,
            self.vcnt,
            self.timer_h,
            self.timer_v,
            self.dtc,
            self.ma,
            self.mb,
        ] {
            out.u16(value);
        }
        out.u32(self.dsa);
        out.u32(self.dda);
        out.u32(self.va);
        out.u64(self.mr);
        out.blob(&self.brf);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let iram = input.blob()?;
        if iram.len() != 0x800 {
            return Err("invalid SA-1 I-RAM state length".into());
        }
        self.iram.copy_from_slice(iram);
        self.ccnt = input.u8()?;
        self.sie = input.u8()?;
        self.scnt = input.u8()?;
        self.cie = input.u8()?;
        self.timer_control = input.u8()?;
        self.bmaps = input.u8()?;
        self.bmap = input.u8()?;
        self.bwp = input.u8()?;
        self.siwp = input.u8()?;
        self.ciwp = input.u8()?;
        self.dcnt = input.u8()?;
        self.cdma = input.u8()?;
        self.bbf = input.u8()? != 0;
        self.mcnt = input.u8()?;
        self.overflow = input.u8()? != 0;
        self.vbd = input.u8()?;
        self.vbit = input.u8()?;
        self.cpu_irq_flag = input.u8()? != 0;
        self.chdma_irq_flag = input.u8()? != 0;
        self.sa1_irq_flag = input.u8()? != 0;
        self.timer_irq_flag = input.u8()? != 0;
        self.dma_irq_flag = input.u8()? != 0;
        self.sa1_nmi_flag = input.u8()? != 0;
        self.release_pending = input.u8()? != 0;
        self.swen = input.u8()? != 0;
        self.cwen = input.u8()? != 0;
        for value in &mut self.banks {
            *value = input.u8()?;
        }
        self.crv = input.u16()?;
        self.cnv = input.u16()?;
        self.civ = input.u16()?;
        self.snv = input.u16()?;
        self.siv = input.u16()?;
        self.hcnt = input.u16()?;
        self.vcnt = input.u16()?;
        self.timer_h = input.u16()?;
        self.timer_v = input.u16()?;
        self.dtc = input.u16()?;
        self.ma = input.u16()?;
        self.mb = input.u16()?;
        self.dsa = input.u32()? & 0x00ff_ffff;
        self.dda = input.u32()? & 0x00ff_ffff;
        self.va = input.u32()? & 0x00ff_ffff;
        self.mr = input.u64()? & 0x00ff_ffff_ffff;
        let brf = input.blob()?;
        if brf.len() != 16 {
            return Err("invalid SA-1 bitmap-register state length".into());
        }
        self.brf.copy_from_slice(brf);
        Ok(())
    }
}

struct SnesCartridge {
    rom: Vec<u8>,
    sram: Vec<u8>,
    map: SnesMap,
    coprocessor: SnesCoprocessor,
    dsp1: Option<Dsp1>,
    dsp2: Option<Dsp2>,
    dsp3: Option<Dsp3>,
    srtc: Option<Srtc>,
    sa1: Option<Sa1State>,
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
        let candidates = [
            (SnesMap::LoRom, 0x7fc0usize),
            (SnesMap::HiRom, 0xffc0usize),
            (SnesMap::ExHiRom, 0x40ffc0usize),
        ];
        let (map, header) = candidates
            .into_iter()
            .max_by_key(|(map, offset)| Self::header_score(&rom, *offset, *map))
            .unwrap();
        let chipset = if header + 0x17 <= rom.len() {
            rom[header + 0x16]
        } else {
            0
        };
        let map_mode = rom.get(header + 0x15).copied().unwrap_or(0) & 0x3f;
        let srtc = chipset == 0x55 && map_mode == 0x35;
        let dsp2 = chipset == 0x05 && map_mode == 0x20;
        let dsp3 =
            chipset == 0x05 && map_mode == 0x30 && rom.get(header + 0x2a).copied() == Some(0xb2);
        let dsp1 = match chipset {
            0x03 => map_mode != 0x30,
            0x05 => !dsp2 && !dsp3,
            _ => false,
        };
        let coprocessor = if srtc {
            SnesCoprocessor::Srtc
        } else if dsp2 {
            SnesCoprocessor::Dsp2
        } else if dsp3 {
            SnesCoprocessor::Dsp3
        } else if dsp1 {
            SnesCoprocessor::Dsp1
        } else {
            match chipset >> 4 {
                0x02 => SnesCoprocessor::Obc1,
                0x03 => SnesCoprocessor::Sa1,
                _ => SnesCoprocessor::None,
            }
        };
        let sram_size = if coprocessor == SnesCoprocessor::Obc1 {
            0x2000
        } else if header + 0x19 <= rom.len() {
            let exponent = rom[header + 0x18];
            if exponent == 0 {
                0
            } else {
                1024usize
                    .checked_shl(exponent.into())
                    .unwrap_or(0)
                    .min(512 * 1024)
            }
        } else {
            0
        };
        let sram = vec![0; sram_size];
        let dsp1 = (coprocessor == SnesCoprocessor::Dsp1).then(Dsp1::default);
        let dsp2 = (coprocessor == SnesCoprocessor::Dsp2).then(Dsp2::default);
        let dsp3 = (coprocessor == SnesCoprocessor::Dsp3).then(Dsp3::default);
        let srtc = (coprocessor == SnesCoprocessor::Srtc).then(Srtc::default);
        let sa1 = (coprocessor == SnesCoprocessor::Sa1).then(Sa1State::default);
        Ok(Self {
            rom,
            sram,
            map,
            coprocessor,
            dsp1,
            dsp2,
            dsp3,
            srtc,
            sa1,
        })
    }

    fn header_score(rom: &[u8], offset: usize, expected: SnesMap) -> i32 {
        if offset + 0x40 > rom.len() {
            return i32::MIN / 2;
        }
        let mode = rom[offset + 0x15] & 0x0f;
        let mut score = 0;
        let mode_matches = match expected {
            SnesMap::LoRom => matches!(mode, 0 | 2 | 3),
            SnesMap::HiRom => mode == 1,
            SnesMap::ExHiRom => mode == 5,
        };
        if mode_matches {
            score += 6;
        } else {
            score -= 6;
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

    fn mirror_rom_index(mut address: usize, mut size: usize) -> usize {
        if size == 0 {
            return 0;
        }
        let mut base = 0usize;
        let mut mask = 1usize << 23;
        while address >= size {
            while address & mask == 0 {
                mask >>= 1;
            }
            address -= mask;
            if size > mask {
                size -= mask;
                base += mask;
            }
            mask >>= 1;
        }
        base + address
    }

    fn sa1_low_bank(address: u32) -> bool {
        let bank = ((address >> 16) & 0xff) as u8;
        bank <= 0x3f || (0x80..=0xbf).contains(&bank)
    }

    fn sa1_io_register(address: u32) -> Option<u16> {
        let offset = (address & 0xffff) as u16;
        (Self::sa1_low_bank(address) && (0x2200..=0x23ff).contains(&offset)).then_some(offset)
    }

    fn sa1_rom_index(&self, address: u32) -> Option<usize> {
        let sa1 = self.sa1.as_ref()?;
        let bank = ((address >> 16) & 0xff) as u8;
        let offset = (address & 0xffff) as u16;
        let translated = if Self::sa1_low_bank(address) && offset >= 0x8000 {
            ((address & 0x800000) >> 2) | ((address & 0x3f0000) >> 1) | (address & 0x007fff)
        } else if bank >= 0xc0 {
            address
        } else {
            return None;
        };
        let logical = translated & 0x003f_ffff;
        let segment = ((logical >> 20) & 3) as usize;
        let bank_control = sa1.banks[segment];
        let physical = if translated < 0x0040_0000 && bank_control & 0x80 == 0 {
            logical
        } else {
            (u32::from(bank_control & 7) << 20) | (logical & 0x000f_ffff)
        };
        Some(Self::mirror_rom_index(physical as usize, self.rom.len()))
    }

    fn sa1_bwram_index(&self, address: usize) -> Option<usize> {
        (!self.sram.is_empty()).then(|| Self::mirror_rom_index(address, self.sram.len()))
    }

    fn sa1_bwram_protected(&self, address: usize) -> bool {
        let Some(sa1) = self.sa1.as_ref() else {
            return false;
        };
        !sa1.swen
            && !sa1.cwen
            && address
                < (0x100usize)
                    .checked_shl(u32::from(sa1.bwp))
                    .unwrap_or(usize::MAX)
    }

    fn sa1_bitmap_read(&self, address: usize) -> u8 {
        let Some(sa1) = self.sa1.as_ref() else {
            return 0xff;
        };
        if self.sram.is_empty() {
            return 0xff;
        }
        if !sa1.bbf {
            let packed = self.sram[Self::mirror_rom_index(address >> 1, self.sram.len())];
            if address & 1 == 0 {
                packed & 0x0f
            } else {
                packed >> 4
            }
        } else {
            let packed = self.sram[Self::mirror_rom_index(address >> 2, self.sram.len())];
            (packed >> ((address & 3) * 2)) & 3
        }
    }

    fn sa1_bitmap_write(&mut self, address: usize, value: u8) {
        if self.sram.is_empty() {
            return;
        }
        let Some(sa1) = self.sa1.as_ref() else {
            return;
        };
        if !sa1.bbf {
            let index = Self::mirror_rom_index(address >> 1, self.sram.len());
            let shift = (address & 1) * 4;
            let mask = 0x0fu8 << shift;
            self.sram[index] = (self.sram[index] & !mask) | ((value & 0x0f) << shift);
        } else {
            let index = Self::mirror_rom_index(address >> 2, self.sram.len());
            let shift = (address & 3) * 2;
            let mask = 3u8 << shift;
            self.sram[index] = (self.sram[index] & !mask) | ((value & 3) << shift);
        }
    }

    fn sa1_cpu_read(&self, address: u32) -> Option<u8> {
        let sa1 = self.sa1.as_ref()?;
        if let Some(register) = Self::sa1_io_register(address) {
            return Some(match register {
                0x2300 => sa1.cpu_status(),
                0x230e => 0xff,
                _ => 0xff,
            });
        }

        if address == 0x00ffea && sa1.scnt & 0x10 != 0 {
            return Some(sa1.snv as u8);
        }
        if address == 0x00ffeb && sa1.scnt & 0x10 != 0 {
            return Some((sa1.snv >> 8) as u8);
        }
        if address == 0x00ffee && sa1.scnt & 0x40 != 0 {
            return Some(sa1.siv as u8);
        }
        if address == 0x00ffef && sa1.scnt & 0x40 != 0 {
            return Some((sa1.siv >> 8) as u8);
        }

        let bank = ((address >> 16) & 0xff) as u8;
        let offset = (address & 0xffff) as usize;
        if Self::sa1_low_bank(address) && (0x3000..0x3800).contains(&offset) {
            return Some(sa1.iram[offset & 0x07ff]);
        }
        if Self::sa1_low_bank(address) && (0x6000..0x8000).contains(&offset) {
            let raw = usize::from(sa1.bmaps & 0x1f) * 0x2000 + (offset & 0x1fff);
            return Some(
                self.sa1_bwram_index(raw)
                    .map_or(0xff, |index| self.sram[index]),
            );
        }
        if (0x40..=0x4f).contains(&bank) {
            let raw = ((usize::from(bank) - 0x40) << 16) | offset;
            return Some(
                self.sa1_bwram_index(raw)
                    .map_or(0xff, |index| self.sram[index]),
            );
        }
        self.sa1_rom_index(address).map(|index| self.rom[index])
    }

    fn sa1_cpu_write_io(&mut self, register: u16, value: u8) {
        let mut run_dma = false;
        {
            let Some(sa1) = self.sa1.as_mut() else {
                return;
            };
            match register {
                0x2200 => {
                    if sa1.ccnt & 0x20 != 0 && value & 0x20 == 0 {
                        sa1.release_pending = true;
                    }
                    sa1.ccnt = value;
                    if value & 0x10 != 0 {
                        sa1.sa1_nmi_flag = true;
                    }
                    if value & 0x80 != 0 {
                        sa1.sa1_irq_flag = true;
                    }
                }
                0x2201 => sa1.sie = value & 0xa0,
                0x2202 => {
                    if value & 0x20 != 0 {
                        sa1.chdma_irq_flag = false;
                    }
                    if value & 0x80 != 0 {
                        sa1.cpu_irq_flag = false;
                    }
                }
                0x2203 => sa1.crv = (sa1.crv & 0xff00) | u16::from(value),
                0x2204 => sa1.crv = (sa1.crv & 0x00ff) | (u16::from(value) << 8),
                0x2205 => sa1.cnv = (sa1.cnv & 0xff00) | u16::from(value),
                0x2206 => sa1.cnv = (sa1.cnv & 0x00ff) | (u16::from(value) << 8),
                0x2207 => sa1.civ = (sa1.civ & 0xff00) | u16::from(value),
                0x2208 => sa1.civ = (sa1.civ & 0x00ff) | (u16::from(value) << 8),
                0x2220..=0x2223 => sa1.banks[usize::from(register - 0x2220)] = value & 0x87,
                0x2224 => sa1.bmaps = value & 0x1f,
                0x2226 => sa1.swen = value & 0x80 != 0,
                0x2228 => sa1.bwp = value & 0x0f,
                0x2229 => sa1.siwp = value,
                0x2231 => sa1.cdma = value,
                0x2232 => sa1.dsa = (sa1.dsa & 0xffff00) | u32::from(value),
                0x2233 => sa1.dsa = (sa1.dsa & 0xff00ff) | (u32::from(value) << 8),
                0x2234 => sa1.dsa = (sa1.dsa & 0x00ffff) | (u32::from(value) << 16),
                0x2235 => sa1.dda = (sa1.dda & 0xffff00) | u32::from(value),
                0x2236 => {
                    sa1.dda = (sa1.dda & 0xff00ff) | (u32::from(value) << 8);
                    run_dma = sa1.dcnt & 0xa4 == 0x80;
                }
                0x2237 => {
                    sa1.dda = (sa1.dda & 0x00ffff) | (u32::from(value) << 16);
                    run_dma = sa1.dcnt & 0xa4 == 0x84;
                }
                _ => {}
            }
        }
        if run_dma {
            self.sa1_run_dma();
        }
    }

    fn sa1_cpu_write(&mut self, address: u32, value: u8) -> bool {
        if self.sa1.is_none() {
            return false;
        }
        if let Some(register) = Self::sa1_io_register(address) {
            self.sa1_cpu_write_io(register, value);
            return true;
        }
        let bank = ((address >> 16) & 0xff) as u8;
        let offset = (address & 0xffff) as usize;
        if Self::sa1_low_bank(address) && (0x3000..0x3800).contains(&offset) {
            let group = (offset >> 8) & 7;
            if self
                .sa1
                .as_ref()
                .is_some_and(|sa1| sa1.siwp & (1 << group) != 0)
            {
                self.sa1.as_mut().unwrap().iram[offset & 0x07ff] = value;
            }
            return true;
        }
        let raw = if Self::sa1_low_bank(address) && (0x6000..0x8000).contains(&offset) {
            usize::from(self.sa1.as_ref().unwrap().bmaps & 0x1f) * 0x2000 + (offset & 0x1fff)
        } else if (0x40..=0x4f).contains(&bank) {
            ((usize::from(bank) - 0x40) << 16) | offset
        } else {
            return self.sa1_rom_index(address).is_some();
        };
        if !self.sa1_bwram_protected(raw) {
            if let Some(index) = self.sa1_bwram_index(raw) {
                self.sram[index] = value;
            }
        }
        true
    }

    fn sa1_read_io(&mut self, register: u16) -> u8 {
        match register {
            0x2301 => self.sa1.as_ref().map_or(0xff, Sa1State::sa1_status),
            0x2302 => self.sa1.as_ref().map_or(0xff, |sa1| sa1.hcnt as u8),
            0x2303 => self.sa1.as_ref().map_or(0xff, |sa1| (sa1.hcnt >> 8) as u8),
            0x2304 => self.sa1.as_ref().map_or(0xff, |sa1| sa1.vcnt as u8),
            0x2305 => self.sa1.as_ref().map_or(0xff, |sa1| (sa1.vcnt >> 8) as u8),
            0x2306..=0x230a => {
                let shift = u32::from(register - 0x2306) * 8;
                self.sa1
                    .as_ref()
                    .map_or(0xff, |sa1| (sa1.mr >> shift) as u8)
            }
            0x230b => self
                .sa1
                .as_ref()
                .map_or(0, |sa1| if sa1.overflow { 0x80 } else { 0 }),
            _ => 0xff,
        }
    }

    fn sa1_write_io(&mut self, register: u16, value: u8) {
        let mut run_dma = false;
        {
            let Some(sa1) = self.sa1.as_mut() else {
                return;
            };
            match register {
                0x2209 => {
                    sa1.scnt = value;
                    if value & 0x80 != 0 {
                        sa1.cpu_irq_flag = true;
                    }
                }
                0x220a => sa1.cie = value & 0xf0,
                0x220b => {
                    if value & 0x10 != 0 {
                        sa1.sa1_nmi_flag = false;
                    }
                    if value & 0x20 != 0 {
                        sa1.dma_irq_flag = false;
                    }
                    if value & 0x40 != 0 {
                        sa1.timer_irq_flag = false;
                    }
                    if value & 0x80 != 0 {
                        sa1.sa1_irq_flag = false;
                    }
                }
                0x220c => sa1.snv = (sa1.snv & 0xff00) | u16::from(value),
                0x220d => sa1.snv = (sa1.snv & 0x00ff) | (u16::from(value) << 8),
                0x220e => sa1.siv = (sa1.siv & 0xff00) | u16::from(value),
                0x220f => sa1.siv = (sa1.siv & 0x00ff) | (u16::from(value) << 8),
                0x2210 => sa1.timer_control = value,
                0x2211 => {
                    sa1.hcnt = 0;
                    sa1.vcnt = 0;
                }
                0x2212 => sa1.hcnt = (sa1.hcnt & 0xff00) | u16::from(value),
                0x2213 => sa1.hcnt = (sa1.hcnt & 0x00ff) | (u16::from(value) << 8),
                0x2214 => sa1.vcnt = (sa1.vcnt & 0xff00) | u16::from(value),
                0x2215 => sa1.vcnt = (sa1.vcnt & 0x00ff) | (u16::from(value) << 8),
                0x2225 => sa1.bmap = value,
                0x2227 => sa1.cwen = value & 0x80 != 0,
                0x222a => sa1.ciwp = value,
                0x2230 => sa1.dcnt = value,
                0x2231 => sa1.cdma = value,
                0x2232 => sa1.dsa = (sa1.dsa & 0xffff00) | u32::from(value),
                0x2233 => sa1.dsa = (sa1.dsa & 0xff00ff) | (u32::from(value) << 8),
                0x2234 => sa1.dsa = (sa1.dsa & 0x00ffff) | (u32::from(value) << 16),
                0x2235 => sa1.dda = (sa1.dda & 0xffff00) | u32::from(value),
                0x2236 => {
                    sa1.dda = (sa1.dda & 0xff00ff) | (u32::from(value) << 8);
                    run_dma = sa1.dcnt & 0xa4 == 0x80;
                }
                0x2237 => {
                    sa1.dda = (sa1.dda & 0x00ffff) | (u32::from(value) << 16);
                    run_dma = sa1.dcnt & 0xa4 == 0x84;
                }
                0x2238 => sa1.dtc = (sa1.dtc & 0xff00) | u16::from(value),
                0x2239 => sa1.dtc = (sa1.dtc & 0x00ff) | (u16::from(value) << 8),
                0x223f => sa1.bbf = value & 0x80 != 0,
                0x2240..=0x224f => sa1.brf[usize::from(register - 0x2240)] = value,
                0x2250 => {
                    sa1.mcnt = value & 3;
                    if value & 2 != 0 {
                        sa1.mr = 0;
                        sa1.overflow = false;
                    }
                }
                0x2251 => sa1.ma = (sa1.ma & 0xff00) | u16::from(value),
                0x2252 => sa1.ma = (sa1.ma & 0x00ff) | (u16::from(value) << 8),
                0x2253 => sa1.mb = (sa1.mb & 0xff00) | u16::from(value),
                0x2254 => {
                    sa1.mb = (sa1.mb & 0x00ff) | (u16::from(value) << 8);
                    sa1.execute_arithmetic();
                }
                0x2258 => sa1.vbd = value,
                0x2259 => sa1.va = (sa1.va & 0xffff00) | u32::from(value),
                0x225a => sa1.va = (sa1.va & 0xff00ff) | (u32::from(value) << 8),
                0x225b => {
                    sa1.va = (sa1.va & 0x00ffff) | (u32::from(value) << 16);
                    sa1.vbit = 0;
                }
                _ => {}
            }
        }
        if run_dma {
            self.sa1_run_dma();
        }
    }

    fn sa1_read_rom_byte(&self, address: u32) -> u8 {
        self.sa1_rom_index(address)
            .map_or(0xff, |index| self.rom[index])
    }

    fn sa1_run_dma(&mut self) {
        let Some(sa1) = self.sa1.as_ref() else {
            return;
        };
        if sa1.dcnt & 0xa0 != 0x80 {
            return;
        }
        let source_kind = sa1.dcnt & 3;
        let destination_kind = (sa1.dcnt >> 2) & 1;
        let mut source = sa1.dsa;
        let mut destination = sa1.dda;
        let count = sa1.dtc;
        for _ in 0..count {
            let value = match source_kind {
                0 => self.sa1_read_rom_byte(source),
                1 => self
                    .sa1_bwram_index(source as usize)
                    .map_or(0xff, |index| self.sram[index]),
                2 => self.sa1.as_ref().unwrap().iram[source as usize & 0x07ff],
                _ => 0xff,
            };
            if destination_kind == 0 {
                self.sa1.as_mut().unwrap().iram[destination as usize & 0x07ff] = value;
            } else if let Some(index) = self.sa1_bwram_index(destination as usize) {
                self.sram[index] = value;
            }
            source = source.wrapping_add(1) & 0x00ff_ffff;
            destination = destination.wrapping_add(1) & 0x00ff_ffff;
        }
        let sa1 = self.sa1.as_mut().unwrap();
        sa1.dsa = source;
        sa1.dda = destination;
        sa1.dtc = 0;
        sa1.dma_irq_flag = true;
    }

    fn sa1_read8(&mut self, address: u32) -> u8 {
        let address = address & 0x00ff_ffff;
        let Some(sa1) = self.sa1.as_ref() else {
            return 0xff;
        };
        match address & 0xffff {
            0xffea | 0xfffa => return sa1.cnv as u8,
            0xffeb | 0xfffb => return (sa1.cnv >> 8) as u8,
            0xffee | 0xfffe => return sa1.civ as u8,
            0xffef | 0xffff => return (sa1.civ >> 8) as u8,
            _ => {}
        }
        if let Some(register) = Self::sa1_io_register(address) {
            return self.sa1_read_io(register);
        }
        let bank = ((address >> 16) & 0xff) as u8;
        let offset = (address & 0xffff) as usize;
        if Self::sa1_low_bank(address) && (offset < 0x0800 || (0x3000..0x3800).contains(&offset)) {
            return self.sa1.as_ref().unwrap().iram[offset & 0x07ff];
        }
        if Self::sa1_low_bank(address) && (0x6000..0x8000).contains(&offset) {
            let map = self.sa1.as_ref().unwrap().bmap;
            let raw = usize::from(map & if map & 0x80 == 0 { 0x1f } else { 0x7f }) * 0x2000
                + (offset & 0x1fff);
            return if map & 0x80 == 0 {
                self.sa1_bwram_index(raw)
                    .map_or(0xff, |index| self.sram[index])
            } else {
                self.sa1_bitmap_read(raw)
            };
        }
        if (0x40..=0x5f).contains(&bank) {
            let raw = ((usize::from(bank) - 0x40) << 16) | offset;
            return self
                .sa1_bwram_index(raw)
                .map_or(0xff, |index| self.sram[index]);
        }
        if (0x60..=0x6f).contains(&bank) {
            let raw = ((usize::from(bank) - 0x60) << 16) | offset;
            return self.sa1_bitmap_read(raw);
        }
        self.sa1_read_rom_byte(address)
    }

    fn sa1_write8(&mut self, address: u32, value: u8) {
        let address = address & 0x00ff_ffff;
        if self.sa1.is_none() {
            return;
        }
        if let Some(register) = Self::sa1_io_register(address) {
            self.sa1_write_io(register, value);
            return;
        }
        let bank = ((address >> 16) & 0xff) as u8;
        let offset = (address & 0xffff) as usize;
        if Self::sa1_low_bank(address) && (offset < 0x0800 || (0x3000..0x3800).contains(&offset)) {
            let group = (offset >> 8) & 7;
            if self.sa1.as_ref().unwrap().ciwp & (1 << group) != 0 {
                self.sa1.as_mut().unwrap().iram[offset & 0x07ff] = value;
            }
            return;
        }
        if Self::sa1_low_bank(address) && (0x6000..0x8000).contains(&offset) {
            let map = self.sa1.as_ref().unwrap().bmap;
            let raw = usize::from(map & if map & 0x80 == 0 { 0x1f } else { 0x7f }) * 0x2000
                + (offset & 0x1fff);
            if self.sa1_bwram_protected(raw) {
                return;
            }
            if map & 0x80 == 0 {
                if let Some(index) = self.sa1_bwram_index(raw) {
                    self.sram[index] = value;
                }
            } else {
                self.sa1_bitmap_write(raw, value);
            }
            return;
        }
        if (0x40..=0x5f).contains(&bank) {
            let raw = ((usize::from(bank) - 0x40) << 16) | offset;
            if !self.sa1_bwram_protected(raw) {
                if let Some(index) = self.sa1_bwram_index(raw) {
                    self.sram[index] = value;
                }
            }
        } else if (0x60..=0x6f).contains(&bank) {
            let raw = ((usize::from(bank) - 0x60) << 16) | offset;
            if !self.sa1_bwram_protected(raw) {
                self.sa1_bitmap_write(raw, value);
            }
        }
    }

    fn obc1_window(&self, address: u32) -> bool {
        if self.coprocessor != SnesCoprocessor::Obc1 {
            return false;
        }
        let bank = ((address >> 16) & 0xff) as usize;
        let offset = (address & 0xffff) as usize;
        let low_bank = bank <= 0x3f || (0x80..=0xbf).contains(&bank);
        let extended_bank = matches!(bank, 0x70 | 0x71 | 0xf0 | 0xf1);
        (low_bank && (0x6000..0x8000).contains(&offset))
            || (extended_bank && ((0x6000..0x8000).contains(&offset) || offset >= 0xe000))
    }

    fn obc1_read(&self, address: u32) -> Option<u8> {
        if !self.obc1_window(address) {
            return None;
        }
        let offset = (address as usize) & 0x1fff;
        let object = usize::from(self.sram[0x1ff6] & 0x7f);
        let base = if self.sram[0x1ff5] & 1 != 0 {
            0x1800
        } else {
            0x1c00
        };
        let index = match offset {
            0x1ff0..=0x1ff3 => base + (object << 2) + (offset - 0x1ff0),
            0x1ff4 => base + (object >> 2) + 0x200,
            _ => offset,
        };
        Some(self.sram[index])
    }

    fn obc1_write(&mut self, address: u32, value: u8) -> bool {
        if !self.obc1_window(address) {
            return false;
        }
        let offset = (address as usize) & 0x1fff;
        let selector = self.sram[0x1ff6];
        let object = usize::from(selector & 0x7f);
        let shift = usize::from(selector & 3) << 1;
        let base = if self.sram[0x1ff5] & 1 != 0 {
            0x1800
        } else {
            0x1c00
        };
        match offset {
            0x1ff0..=0x1ff3 => {
                self.sram[base + (object << 2) + (offset - 0x1ff0)] = value;
            }
            0x1ff4 => {
                let index = base + (object >> 2) + 0x200;
                let mask = 3u8 << shift;
                self.sram[index] = (self.sram[index] & !mask) | ((value & 3) << shift);
            }
            _ => self.sram[offset] = value,
        }
        true
    }

    fn dsp1_port(&self, address: u32) -> Option<bool> {
        if self.coprocessor != SnesCoprocessor::Dsp1 {
            return None;
        }
        let bank = ((address >> 16) & 0xff) as u8;
        let offset = (address & 0xffff) as u16;
        match self.map {
            SnesMap::LoRom
                if self.rom.len() > 0x100000
                    && ((0x60..=0x6f).contains(&bank) || (0xe0..=0xef).contains(&bank))
                    && offset < 0x8000 =>
            {
                Some(offset >= 0x4000)
            }
            SnesMap::LoRom
                if self.rom.len() <= 0x100000
                    && ((0x20..=0x3f).contains(&bank) || (0xa0..=0xbf).contains(&bank))
                    && offset >= 0x8000 =>
            {
                Some(offset >= 0xc000)
            }
            SnesMap::HiRom
                if (bank <= 0x1f || (0x80..=0x9f).contains(&bank))
                    && (0x6000..0x8000).contains(&offset) =>
            {
                Some(offset >= 0x7000)
            }
            _ => None,
        }
    }

    fn dsp2_port(&self, address: u32) -> bool {
        if self.coprocessor != SnesCoprocessor::Dsp2 {
            return false;
        }
        let bank = ((address >> 16) & 0xff) as u8;
        let offset = (address & 0xffff) as u16;
        ((0x20..=0x3f).contains(&bank) || (0xa0..=0xbf).contains(&bank))
            && ((0x6000..=0x6fff).contains(&offset) || (0x8000..=0xbfff).contains(&offset))
    }

    fn dsp3_port(&self, address: u32) -> Option<u16> {
        if self.coprocessor != SnesCoprocessor::Dsp3 {
            return None;
        }
        let bank = ((address >> 16) & 0xff) as u8;
        let offset = (address & 0xffff) as u16;
        (((0x20..=0x3f).contains(&bank) || (0xa0..=0xbf).contains(&bank)) && offset >= 0x8000)
            .then_some(offset)
    }

    fn read(&mut self, address: u32) -> u8 {
        if let Some(offset) = self.dsp3_port(address) {
            return self.dsp3.as_mut().unwrap().read(offset);
        }
        if self.dsp2_port(address) {
            return self.dsp2.as_mut().unwrap().read_data();
        }
        if let Some(status) = self.dsp1_port(address) {
            let dsp1 = self.dsp1.as_mut().unwrap();
            return if status {
                dsp1.status()
            } else {
                dsp1.read_data()
            };
        }
        if let Some(value) = self.sa1_cpu_read(address) {
            return value;
        }
        if let Some(value) = self.obc1_read(address) {
            return value;
        }
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
            SnesMap::ExHiRom if bank >= 0xc0 => ((bank & 0x3f) * 0x10000) + offset,
            SnesMap::ExHiRom if (0x40..=0x7d).contains(&bank) => {
                0x400000 + ((bank & 0x3f) * 0x10000) + offset
            }
            SnesMap::ExHiRom if (0x80..=0xbf).contains(&bank) && offset >= 0x8000 => {
                ((bank & 0x3f) * 0x10000) + offset
            }
            SnesMap::ExHiRom if bank <= 0x3f && offset >= 0x8000 => {
                0x400000 + bank * 0x10000 + offset
            }
            _ => return 0xff,
        };
        self.rom[Self::mirror_rom_index(index, self.rom.len())]
    }

    fn write(&mut self, address: u32, value: u8) {
        if let Some(offset) = self.dsp3_port(address) {
            self.dsp3.as_mut().unwrap().write(offset, value);
            return;
        }
        if self.dsp2_port(address) {
            self.dsp2.as_mut().unwrap().write_data(value);
            return;
        }
        if let Some(status) = self.dsp1_port(address) {
            if !status {
                self.dsp1.as_mut().unwrap().write_data(value);
            }
            return;
        }
        if self.sa1_cpu_write(address, value) {
            return;
        }
        if self.obc1_write(address, value) {
            return;
        }
        if let Some(index) = self.sram_index(address) {
            self.sram[index] = value;
        }
    }
    fn sram_index(&self, address: u32) -> Option<usize> {
        if self.sram.is_empty()
            || matches!(
                self.coprocessor,
                SnesCoprocessor::Obc1 | SnesCoprocessor::Sa1
            )
        {
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
            SnesMap::HiRom | SnesMap::ExHiRom
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

struct Sa1Bus<'a> {
    cartridge: &'a mut SnesCartridge,
}

impl Bus65816 for Sa1Bus<'_> {
    fn read8(&mut self, address: u32) -> u8 {
        self.cartridge.sa1_read8(address)
    }

    fn write8(&mut self, address: u32, value: u8) {
        self.cartridge.sa1_write8(address, value);
    }
}

struct SnesBus {
    cartridge: SnesCartridge,
    wram: Vec<u8>,
    ppu: SnesPpu,
    apu: SnesApu,
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
            apu: SnesApu::new(),
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
        self.apu.reset();
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
        if let Some(dsp1) = &mut self.cartridge.dsp1 {
            dsp1.reset();
        }
        if let Some(dsp2) = &mut self.cartridge.dsp2 {
            dsp2.reset();
        }
        if let Some(dsp3) = &mut self.cartridge.dsp3 {
            dsp3.reset();
        }
        if let Some(srtc) = &mut self.cartridge.srtc {
            srtc.reset();
        }
        if let Some(sa1) = &mut self.cartridge.sa1 {
            sa1.reset();
        }
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
        self.apu.tick_main_cycles(cycles);
        if let Some(srtc) = &mut self.cartridge.srtc {
            srtc.tick(cycles);
        }
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
            0x2800 if self.cartridge.srtc.is_some() => self.cartridge.srtc.as_mut().unwrap().read(),
            0x2140..=0x2143 => self.apu.read_cpu_port(usize::from(offset - 0x2140)),
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
            0x2801 if self.cartridge.srtc.is_some() => {
                self.cartridge.srtc.as_mut().unwrap().write(value)
            }
            0x2140..=0x2143 => self.apu.write_cpu_port(usize::from(offset - 0x2140), value),
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
        out.u8(self.cartridge.dsp1.is_some() as u8);
        if let Some(dsp1) = &self.cartridge.dsp1 {
            dsp1.save(out);
        }
        out.u8(self.cartridge.dsp2.is_some() as u8);
        if let Some(dsp2) = &self.cartridge.dsp2 {
            dsp2.save(out);
        }
        out.u8(self.cartridge.dsp3.is_some() as u8);
        if let Some(dsp3) = &self.cartridge.dsp3 {
            dsp3.save(out);
        }
        out.u8(self.cartridge.srtc.is_some() as u8);
        if let Some(srtc) = &self.cartridge.srtc {
            srtc.save(out);
        }
        out.u8(self.cartridge.sa1.is_some() as u8);
        if let Some(sa1) = &self.cartridge.sa1 {
            sa1.save(out);
        }
        self.ppu.save(out);
        self.apu.save(out);
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
        let has_dsp1 = input.u8()? != 0;
        if has_dsp1 != self.cartridge.dsp1.is_some() {
            return Err("SNES save state does not match DSP-1 cartridge".into());
        }
        if let Some(dsp1) = &mut self.cartridge.dsp1 {
            dsp1.load(input)?;
        }
        let has_dsp2 = input.u8()? != 0;
        if has_dsp2 != self.cartridge.dsp2.is_some() {
            return Err("SNES save state does not match DSP-2 cartridge".into());
        }
        if let Some(dsp2) = &mut self.cartridge.dsp2 {
            dsp2.load(input)?;
        }
        let has_dsp3 = input.u8()? != 0;
        if has_dsp3 != self.cartridge.dsp3.is_some() {
            return Err("SNES save state does not match DSP-3 cartridge".into());
        }
        if let Some(dsp3) = &mut self.cartridge.dsp3 {
            dsp3.load(input)?;
        }
        let has_srtc = input.u8()? != 0;
        if has_srtc != self.cartridge.srtc.is_some() {
            return Err("SNES save state does not match S-RTC cartridge".into());
        }
        if let Some(srtc) = &mut self.cartridge.srtc {
            srtc.load(input)?;
        }
        let has_sa1 = input.u8()? != 0;
        if has_sa1 != self.cartridge.sa1.is_some() {
            return Err("SNES save state does not match cartridge coprocessor".into());
        }
        if let Some(sa1) = &mut self.cartridge.sa1 {
            sa1.load(input)?;
        }
        self.ppu.load(input)?;
        self.apu.load(input)?;
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
    sa1_cpu: Option<Cpu65816>,
    sa1_credit: i32,
    bus: SnesBus,
    audio: AudioBuffer,
    powered: bool,
}

impl SnesMachine {
    pub fn from_rom(image: &[u8]) -> Result<Self, String> {
        let cartridge = SnesCartridge::parse(image)?;
        let has_sa1 = cartridge.coprocessor == SnesCoprocessor::Sa1;
        let mut bus = SnesBus::new(cartridge);
        let mut cpu = Cpu65816::default();
        cpu.reset(&mut bus);
        let audio_rate = bus.apu.sample_rate();
        Ok(Self {
            cpu,
            sa1_cpu: has_sa1.then(Cpu65816::default),
            sa1_credit: 0,
            bus,
            audio: AudioBuffer::new(audio_rate, 2),
            powered: true,
        })
    }

    fn clock_sa1(&mut self, main_cycles: u32) {
        let Some(cpu) = &mut self.sa1_cpu else {
            return;
        };
        let release = self.bus.cartridge.sa1.as_mut().and_then(|sa1| {
            if sa1.release_pending {
                sa1.release_pending = false;
                Some(sa1.crv)
            } else {
                None
            }
        });
        if let Some(vector) = release {
            *cpu = Cpu65816::default();
            cpu.pc = vector;
            cpu.pbr = 0;
            cpu.dbr = 0;
            cpu.cycles = 8;
            self.sa1_credit = 0;
        }

        let active = self
            .bus
            .cartridge
            .sa1
            .as_ref()
            .is_some_and(Sa1State::active);
        if !active {
            self.sa1_credit = 0;
            return;
        }

        self.sa1_credit = self.sa1_credit.saturating_add(
            i32::try_from(main_cycles)
                .unwrap_or(i32::MAX)
                .saturating_mul(3),
        );
        while self.sa1_credit > 0 {
            let (nmi_pending, irq_pending) = self
                .bus
                .cartridge
                .sa1
                .as_ref()
                .map(|sa1| (sa1.sa1_nmi_pending(), sa1.sa1_irq_pending()))
                .unwrap_or((false, false));
            let mut bus = Sa1Bus {
                cartridge: &mut self.bus.cartridge,
            };
            let used = if nmi_pending {
                let used = cpu.nmi(&mut bus);
                if used != 0 {
                    if let Some(sa1) = &mut bus.cartridge.sa1 {
                        sa1.sa1_nmi_flag = false;
                    }
                }
                used
            } else if irq_pending {
                let interrupt = cpu.irq(&mut bus);
                if interrupt == 0 {
                    cpu.step(&mut bus)
                } else {
                    interrupt
                }
            } else {
                cpu.step(&mut bus)
            };
            if used == 0 {
                self.sa1_credit = 0;
                break;
            }
            if let Some(sa1) = &mut self.bus.cartridge.sa1 {
                sa1.tick_timer(used);
            }
            self.sa1_credit = self.sa1_credit.saturating_sub(used as i32);
        }
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
        if self
            .bus
            .cartridge
            .sa1
            .as_ref()
            .is_some_and(Sa1State::main_irq_pending)
        {
            let interrupt = self.cpu.irq(&mut self.bus);
            self.bus.tick_cpu(interrupt);
        }
        self.clock_sa1(used);
        used
    }

    fn flush_audio(&mut self) {
        self.audio.begin_frame();
        let samples = self.bus.apu.take_samples();
        let (pairs, _) = samples.as_chunks::<2>();
        for pair in pairs {
            self.audio.push_stereo(pair[0], pair[1]);
        }
    }

    fn save_cpu_state(cpu: &Cpu65816, out: &mut StateWriter) {
        out.u16(cpu.a);
        out.u16(cpu.x);
        out.u16(cpu.y);
        out.u16(cpu.sp);
        out.u16(cpu.d);
        out.u16(cpu.pc);
        out.u8(cpu.pbr);
        out.u8(cpu.dbr);
        out.u8(cpu.p);
        out.u8(cpu.emulation as u8);
        out.u64(cpu.cycles);
        out.u8(cpu.stopped as u8);
        out.u8(cpu.waiting as u8);
    }

    fn load_cpu_state(cpu: &mut Cpu65816, input: &mut StateReader<'_>) -> Result<(), String> {
        cpu.a = input.u16()?;
        cpu.x = input.u16()?;
        cpu.y = input.u16()?;
        cpu.sp = input.u16()?;
        cpu.d = input.u16()?;
        cpu.pc = input.u16()?;
        cpu.pbr = input.u8()?;
        cpu.dbr = input.u8()?;
        cpu.p = input.u8()?;
        cpu.emulation = input.u8()? != 0;
        cpu.cycles = input.u64()?;
        cpu.stopped = input.u8()? != 0;
        cpu.waiting = input.u8()? != 0;
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
        self.sa1_cpu = self.bus.cartridge.sa1.is_some().then(Cpu65816::default);
        self.sa1_credit = 0;
        self.powered = true;
        self.flush_audio();
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
        self.flush_audio();
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
        Self::save_cpu_state(&self.cpu, &mut out);
        out.u8(self.sa1_cpu.is_some() as u8);
        if let Some(cpu) = &self.sa1_cpu {
            Self::save_cpu_state(cpu, &mut out);
        }
        out.u32(self.sa1_credit as u32);
        self.bus.save(&mut out);
        out.u8(self.powered as u8);
        Ok(out.finish())
    }

    fn load_state(&mut self, bytes: &[u8]) -> Result<(), String> {
        let mut input = StateReader::new(bytes, PlatformId::Snes, STATE_VERSION)?;
        Self::load_cpu_state(&mut self.cpu, &mut input)?;
        let has_sa1_cpu = input.u8()? != 0;
        if has_sa1_cpu != self.sa1_cpu.is_some() {
            return Err("SNES save state does not match SA-1 CPU configuration".into());
        }
        if let Some(cpu) = &mut self.sa1_cpu {
            Self::load_cpu_state(cpu, &mut input)?;
        }
        self.sa1_credit = input.u32()? as i32;
        self.bus.load(&mut input)?;
        self.powered = input.u8()? != 0;
        self.flush_audio();
        input.finish()
    }

    fn persistent_len(&self, kind: ResourceKind, slot: u32) -> usize {
        match (kind, slot) {
            (ResourceKind::Storage, 0) => self.bus.cartridge.persistent_len(),
            (ResourceKind::Storage, 1) => self
                .bus
                .cartridge
                .srtc
                .as_ref()
                .map_or(0, |srtc| srtc.persistent().len()),
            _ => 0,
        }
    }

    fn read_persistent(&self, kind: ResourceKind, slot: u32, out: &mut [u8]) -> Result<(), String> {
        let source = match (kind, slot) {
            (ResourceKind::Storage, 0) => self.bus.cartridge.persistent(),
            (ResourceKind::Storage, 1) => self
                .bus
                .cartridge
                .srtc
                .as_ref()
                .map(Srtc::persistent)
                .ok_or_else(|| "SNES cartridge has no S-RTC persistent storage".to_string())?,
            _ => return Err("unsupported SNES persistent resource".into()),
        };
        if out.len() != source.len() {
            return Err("SNES persistent output length mismatch".into());
        }
        out.copy_from_slice(source);
        Ok(())
    }

    fn write_persistent(
        &mut self,
        kind: ResourceKind,
        slot: u32,
        data: &[u8],
    ) -> Result<(), String> {
        match (kind, slot) {
            (ResourceKind::Storage, 0) => self.bus.cartridge.set_persistent(data),
            (ResourceKind::Storage, 1) => self
                .bus
                .cartridge
                .srtc
                .as_mut()
                .ok_or_else(|| "SNES cartridge has no S-RTC persistent storage".to_string())?
                .set_persistent(data),
            _ => Err("unsupported SNES persistent resource".into()),
        }
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

    fn synthetic_dsp1(map: SnesMap, size: usize) -> Vec<u8> {
        let mut rom = vec![0xea; size];
        let header = match map {
            SnesMap::LoRom => 0x7fc0,
            SnesMap::HiRom => 0xffc0,
            SnesMap::ExHiRom => panic!("DSP-1 does not use ExHiROM in this test"),
        };
        rom[header + 0x15] = match map {
            SnesMap::LoRom => 0x20,
            SnesMap::HiRom => 0x21,
            SnesMap::ExHiRom => unreachable!(),
        };
        rom[header + 0x16] = 0x03;
        rom[header + 0x18] = 1;
        rom[header + 0x1c..header + 0x20].copy_from_slice(&[0xff, 0xff, 0x00, 0x00]);
        rom[header + 0x3c..header + 0x3e].copy_from_slice(&[0x00, 0x80]);
        rom
    }

    fn synthetic_dsp2() -> Vec<u8> {
        let mut rom = synthetic_dsp1(SnesMap::LoRom, 0x8000);
        rom[0x7fc0 + 0x16] = 0x05;
        rom
    }

    fn synthetic_dsp3() -> Vec<u8> {
        let mut rom = synthetic_dsp1(SnesMap::LoRom, 0x8000);
        let header = 0x7fc0;
        rom[header + 0x15] = 0x30;
        rom[header + 0x16] = 0x05;
        rom[header + 0x2a] = 0xb2;
        rom
    }

    fn synthetic_srtc() -> Vec<u8> {
        let mut rom = vec![0xea; 0x600000];
        let header = 0x40ffc0;
        rom[header + 0x15] = 0x35;
        rom[header + 0x16] = 0x55;
        rom[header + 0x18] = 1;
        rom[header + 0x1c..header + 0x20].copy_from_slice(&[0xff, 0xff, 0x00, 0x00]);
        rom[header + 0x3c..header + 0x3e].copy_from_slice(&[0x00, 0x80]);
        rom
    }

    fn synthetic_sa1() -> Vec<u8> {
        let mut rom = vec![0xea; 0x8000];
        let main_program = [
            0x78, 0xa9, 0x00, 0x8d, 0x03, 0x22, 0xa9, 0x90, 0x8d, 0x04, 0x22, 0xa9, 0x00, 0x8d,
            0x00, 0x22, 0x80, 0xfe,
        ];
        rom[..main_program.len()].copy_from_slice(&main_program);
        let sa1_program = [
            0xa9, 0x80, 0x8d, 0x27, 0x22, 0xa9, 0x5a, 0x8d, 0x00, 0x60, 0x80, 0xfe,
        ];
        rom[0x1000..0x1000 + sa1_program.len()].copy_from_slice(&sa1_program);
        let header = 0x7fc0;
        rom[header + 0x15] = 0x23;
        rom[header + 0x16] = 0x34;
        rom[header + 0x18] = 5;
        rom[header + 0x1c..header + 0x20].copy_from_slice(&[0xff, 0xff, 0x00, 0x00]);
        rom[0x7ffc..0x7ffe].copy_from_slice(&[0x00, 0x80]);
        rom
    }

    #[test]
    fn srtc_persistent_storage_uses_dedicated_slot_one() {
        let rom = synthetic_srtc();
        let mut machine = SnesMachine::from_rom(&rom).unwrap();
        assert_eq!(machine.bus.cartridge.coprocessor, SnesCoprocessor::Srtc);
        assert_eq!(machine.persistent_len(ResourceKind::Storage, 1), 20);

        let rtc = [5, 4, 3, 2, 1, 1, 2, 1, 9, 9, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        machine
            .write_persistent(ResourceKind::Storage, 1, &rtc)
            .unwrap();

        let mut restored = [0u8; 20];
        machine
            .read_persistent(ResourceKind::Storage, 1, &mut restored)
            .unwrap();
        assert_eq!(restored, rtc);
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
        assert_eq!(machine.audio().sample_rate(), 32_000);
    }

    #[test]
    fn dsp_header_detection_distinguishes_dsp1_dsp2_and_dsp3() {
        let mut dsp1 = synthetic_dsp1(SnesMap::LoRom, 0x8000);
        assert_eq!(
            SnesCartridge::parse(&dsp1).unwrap().coprocessor,
            SnesCoprocessor::Dsp1
        );

        dsp1[0x7fc0 + 0x15] = 0x30;
        assert_eq!(
            SnesCartridge::parse(&dsp1).unwrap().coprocessor,
            SnesCoprocessor::None
        );

        let dsp2 = synthetic_dsp2();
        assert_eq!(
            SnesCartridge::parse(&dsp2).unwrap().coprocessor,
            SnesCoprocessor::Dsp2
        );

        let dsp3 = synthetic_dsp3();
        assert_eq!(
            SnesCartridge::parse(&dsp3).unwrap().coprocessor,
            SnesCoprocessor::Dsp3
        );
    }

    #[test]
    fn dsp3_data_and_status_ports_are_mirrored_across_documented_lorom_windows() {
        let mut cart = SnesCartridge::parse(&synthetic_dsp3()).unwrap();
        assert_eq!(cart.coprocessor, SnesCoprocessor::Dsp3);
        assert_eq!(cart.dsp3_port(0x20_8000), Some(0x8000));
        assert_eq!(cart.dsp3_port(0x3f_bfff), Some(0xbfff));
        assert_eq!(cart.dsp3_port(0xa0_c000), Some(0xc000));
        assert_eq!(cart.dsp3_port(0xbf_ffff), Some(0xffff));
        assert_eq!(cart.dsp3_port(0x20_7fff), None);
        assert_eq!(cart.dsp3_port(0x40_8000), None);
        assert_eq!(cart.read(0x20_c000), 0x84);

        cart.write(0x20_8000, 0x06);
        assert_eq!(cart.read(0xa0_c000), 0x80);
        cart.write(0xa0_8000, 0x10);
        cart.write(0xbf_bfff, 0x08);
        assert_eq!(cart.read(0x3f_c000), 0x84);

        cart.write(0x20_8000, 0x03);
        cart.write(0xa0_8000, 0x03);
        cart.write(0xbf_bfff, 0x02);
        let result = u16::from_le_bytes([cart.read(0x20_8000), cart.read(0xa0_bfff)]);
        assert_eq!(result, 0x0023);
        assert_eq!(cart.read(0xbf_c000), 0x84);
    }

    #[test]
    fn dsp3_partial_word_round_trips_through_machine_state() {
        let rom = synthetic_dsp3();
        let mut machine = SnesMachine::from_rom(&rom).unwrap();
        let data = 0x20_8000;

        machine.bus.cartridge.write(data, 0x06);
        machine.bus.cartridge.write(data, 0x10);
        machine.bus.cartridge.write(data, 0x08);
        machine.bus.cartridge.write(data, 0x03);
        machine.bus.cartridge.write(data, 0x03);
        let state = machine.save_state().unwrap();

        machine.bus.cartridge.write(data, 0x02);
        let expected = [
            machine.bus.cartridge.read(data),
            machine.bus.cartridge.read(data),
        ];

        machine.load_state(&state).unwrap();
        machine.bus.cartridge.write(data, 0x02);
        let restored = [
            machine.bus.cartridge.read(data),
            machine.bus.cartridge.read(data),
        ];
        assert_eq!(restored, expected);
        assert_eq!(u16::from_le_bytes(restored), 0x0023);
    }

    #[test]
    fn dsp2_data_port_is_mirrored_across_documented_lorom_windows() {
        let mut cart = SnesCartridge::parse(&synthetic_dsp2()).unwrap();
        assert_eq!(cart.coprocessor, SnesCoprocessor::Dsp2);
        assert!(cart.dsp2_port(0x20_6000));
        assert!(cart.dsp2_port(0x3f_6fff));
        assert!(cart.dsp2_port(0xa0_8000));
        assert!(cart.dsp2_port(0xbf_bfff));
        assert!(!cart.dsp2_port(0x20_7000));
        assert!(!cart.dsp2_port(0x20_c000));

        cart.write(0x20_6000, 0x09);
        for byte in [0x34, 0x12, 0x02, 0x00] {
            cart.write(0xa0_8000, byte);
        }
        let result = [
            cart.read(0x20_8000),
            cart.read(0x3f_6000),
            cart.read(0xa0_bfff),
            cart.read(0xbf_8000),
        ];
        assert_eq!(u32::from_le_bytes(result), 0x2468);
        assert_eq!(cart.read(0x20_6000), 0xff);
    }

    #[test]
    fn dsp2_pending_result_round_trips_through_machine_state() {
        let rom = synthetic_dsp2();
        let mut machine = SnesMachine::from_rom(&rom).unwrap();
        let data = 0x20_6000;
        machine.bus.cartridge.write(data, 0x09);
        for byte in [0x78, 0x56, 0x02, 0x00] {
            machine.bus.cartridge.write(data, byte);
        }

        let state = machine.save_state().unwrap();
        let expected = [
            machine.bus.cartridge.read(data),
            machine.bus.cartridge.read(data),
            machine.bus.cartridge.read(data),
            machine.bus.cartridge.read(data),
        ];
        machine.load_state(&state).unwrap();
        let restored = [
            machine.bus.cartridge.read(data),
            machine.bus.cartridge.read(data),
            machine.bus.cartridge.read(data),
            machine.bus.cartridge.read(data),
        ];
        assert_eq!(restored, expected);
        assert_eq!(u32::from_le_bytes(restored), 0xacf0);
    }

    #[test]
    fn dsp1_command_and_status_ports_cover_all_cartridge_layouts() {
        fn exercise(mut cart: SnesCartridge, data: u32, status: u32) {
            assert_eq!(cart.coprocessor, SnesCoprocessor::Dsp1);
            assert_eq!(cart.read(status), 0x80);
            cart.write(data, 0x00);
            for word in [0x4000i16, 0x4000] {
                for byte in word.to_le_bytes() {
                    cart.write(data, byte);
                }
            }
            let result = i16::from_le_bytes([cart.read(data), cart.read(data)]);
            assert_eq!(result, 0x2000);
        }

        exercise(
            SnesCartridge::parse(&synthetic_dsp1(SnesMap::LoRom, 0x8000)).unwrap(),
            0x20_8000,
            0x20_c000,
        );
        exercise(
            SnesCartridge::parse(&synthetic_dsp1(SnesMap::LoRom, 0x180000)).unwrap(),
            0x60_0000,
            0x60_4000,
        );
        exercise(
            SnesCartridge::parse(&synthetic_dsp1(SnesMap::HiRom, 0x10000)).unwrap(),
            0x00_6000,
            0x00_7000,
        );
    }

    #[test]
    fn dsp1_pending_result_round_trips_through_machine_state() {
        let rom = synthetic_dsp1(SnesMap::LoRom, 0x8000);
        let mut machine = SnesMachine::from_rom(&rom).unwrap();
        let data = 0x20_8000;
        machine.bus.cartridge.write(data, 0x00);
        for word in [0x6000i16, 0x2000] {
            for byte in word.to_le_bytes() {
                machine.bus.cartridge.write(data, byte);
            }
        }

        let state = machine.save_state().unwrap();
        let expected = [
            machine.bus.cartridge.read(data),
            machine.bus.cartridge.read(data),
        ];
        machine.load_state(&state).unwrap();
        let restored = [
            machine.bus.cartridge.read(data),
            machine.bus.cartridge.read(data),
        ];
        assert_eq!(restored, expected);
        assert_eq!(i16::from_le_bytes(restored), 0x1800);
    }

    #[test]
    fn sa1_second_65816_releases_from_crv_and_writes_bwram() {
        let rom = synthetic_sa1();
        let mut machine = SnesMachine::from_rom(&rom).unwrap();
        assert_eq!(machine.bus.cartridge.coprocessor, SnesCoprocessor::Sa1);
        assert!(machine.sa1_cpu.is_some());

        for _ in 0..64 {
            assert_ne!(machine.clock_instruction(), 0);
        }

        assert_eq!(machine.bus.cartridge.sram[0], 0x5a);
        let sa1 = machine.bus.cartridge.sa1.as_ref().unwrap();
        assert!(sa1.active());
        assert!(!sa1.release_pending);
        assert!(machine.sa1_cpu.as_ref().unwrap().cycles > 8);
    }

    #[test]
    fn sa1_arithmetic_and_normal_dma_use_hardware_register_paths() {
        let rom = synthetic_sa1();
        let mut cart = SnesCartridge::parse(&rom).unwrap();

        cart.sa1_write8(0x002250, 0x00);
        cart.sa1_write8(0x002251, 0xfd);
        cart.sa1_write8(0x002252, 0xff);
        cart.sa1_write8(0x002253, 0x07);
        cart.sa1_write8(0x002254, 0x00);
        let product = u32::from(cart.sa1_read8(0x002306))
            | (u32::from(cart.sa1_read8(0x002307)) << 8)
            | (u32::from(cart.sa1_read8(0x002308)) << 16)
            | (u32::from(cart.sa1_read8(0x002309)) << 24);
        assert_eq!(product, 0xffff_ffeb);

        cart.sa1_write8(0x002230, 0x84);
        cart.sa1_write8(0x002238, 4);
        cart.sa1_write8(0x002239, 0);
        cart.sa1_write8(0x002232, 0x00);
        cart.sa1_write8(0x002233, 0x80);
        cart.sa1_write8(0x002234, 0x00);
        cart.sa1_write8(0x002235, 0x00);
        cart.sa1_write8(0x002236, 0x00);
        cart.sa1_write8(0x002237, 0x00);
        assert_eq!(&cart.sram[..4], &rom[..4]);
        assert!(cart.sa1.as_ref().unwrap().dma_irq_flag);
        assert_eq!(cart.sa1.as_ref().unwrap().dtc, 0);
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
    fn hdma_register_updates_apply_to_the_following_scanline_only() {
        let rom = synthetic_lorom();
        let mut machine = SnesMachine::from_rom(&rom).unwrap();
        machine.bus.ppu.write(0x2121, 0);
        machine.bus.ppu.write(0x2122, 0x1f);
        machine.bus.ppu.write(0x2122, 0x00);
        let table = [0x82, 0x0f, 0x00, 0x00];
        machine.bus.wram[0x1000..0x1000 + table.len()].copy_from_slice(&table);
        machine.bus.dma[0][0] = 0x00;
        machine.bus.dma[0][1] = 0x00;
        machine.bus.dma[0][2] = 0x00;
        machine.bus.dma[0][3] = 0x10;
        machine.bus.dma[0][4] = 0x7e;
        machine.bus.hdma_enable = 1;
        machine.bus.init_hdma();

        machine.bus.ppu.tick_dots(341);
        machine.bus.run_hdma_line();
        machine.bus.ppu.tick_dots(341);
        machine.bus.run_hdma_line();
        machine.bus.ppu.tick_dots(341);

        assert_eq!(&machine.bus.ppu.video().pixels()[..3], &[255, 0, 0]);
        let row1 = machine.bus.ppu.video().width() as usize * 4;
        assert_eq!(
            &machine.bus.ppu.video().pixels()[row1..row1 + 3],
            &[0, 0, 0]
        );
    }

    #[test]
    fn hirom_maps_full_banks() {
        let mut rom = vec![0; 0x10000];
        rom[0xffd5] = 0x21;
        rom[0xfffc] = 0x00;
        rom[0xfffd] = 0x80;
        rom[0x8000] = 0x42;
        let mut cart = SnesCartridge::parse(&rom).unwrap();
        assert_eq!(cart.map, SnesMap::HiRom);
        assert_eq!(cart.read(0x008000), 0x42);
    }

    #[test]
    fn exhirom_detects_header_past_four_megabytes_and_maps_both_halves() {
        let mut rom = vec![0; 0x600000];
        let header = 0x40ffc0;
        rom[header + 0x15] = 0x25;
        rom[header + 0x1c..header + 0x20].copy_from_slice(&[0xff, 0xff, 0x00, 0x00]);
        rom[header + 0x3c..header + 0x3e].copy_from_slice(&[0x00, 0x80]);

        rom[0x000000] = 0x11;
        rom[0x008000] = 0x22;
        rom[0x400000] = 0x33;
        rom[0x408000] = 0x44;

        let mut cart = SnesCartridge::parse(&rom).unwrap();
        assert_eq!(cart.map, SnesMap::ExHiRom);
        assert_eq!(cart.read(0xc00000), 0x11);
        assert_eq!(cart.read(0x808000), 0x22);
        assert_eq!(cart.read(0x400000), 0x33);
        assert_eq!(cart.read(0x008000), 0x44);
    }

    #[test]
    fn non_power_of_two_roms_follow_address_line_mirroring() {
        let mut rom = vec![0; 0x300000];
        rom[0xffd5] = 0x21;
        rom[0xffdc..0xffe0].copy_from_slice(&[0xff, 0xff, 0x00, 0x00]);
        rom[0xfffc..0xfffe].copy_from_slice(&[0x00, 0x80]);
        rom[0x000000] = 0xa5;
        rom[0x200000] = 0x5a;

        let mut cart = SnesCartridge::parse(&rom).unwrap();
        assert_eq!(cart.map, SnesMap::HiRom);
        assert_eq!(cart.read(0xf00000), 0x5a);
        assert_ne!(cart.read(0xf00000), rom[0]);
    }

    #[test]
    fn obc1_object_window_updates_selected_object_and_packed_attributes() {
        let mut rom = vec![0; 0x8000];
        let header = 0x7fc0;
        rom[header + 0x15] = 0x30;
        rom[header + 0x16] = 0x25;
        rom[header + 0x1c..header + 0x20].copy_from_slice(&[0xff, 0xff, 0x00, 0x00]);
        rom[0x7ffc..0x7ffe].copy_from_slice(&[0x00, 0x80]);

        let mut cart = SnesCartridge::parse(&rom).unwrap();
        assert_eq!(cart.coprocessor, SnesCoprocessor::Obc1);
        assert_eq!(cart.persistent_len(), 0x2000);

        cart.write(0x007ff5, 1);
        cart.write(0x007ff6, 5);
        cart.write(0x007ff0, 0x12);
        cart.write(0x007ff1, 0x34);
        cart.write(0x007ff2, 0x56);
        cart.write(0x007ff3, 0x78);
        cart.write(0x007ff4, 3);

        let object = 0x1800 + (5 << 2);
        assert_eq!(&cart.sram[object..object + 4], &[0x12, 0x34, 0x56, 0x78]);
        let attribute = 0x1800 + (5 >> 2) + 0x200;
        assert_eq!(cart.sram[attribute] & 0x0c, 0x0c);
        assert_eq!(cart.read(0x007ff0), 0x12);
        assert_eq!(cart.read(0x007ff4), cart.sram[attribute]);

        cart.write(0x706000, 0xa5);
        assert_eq!(cart.read(0xf06000), 0xa5);
    }
}
