use std::collections::VecDeque;

use crate::cd_image::DiscImage;
use crate::cpu_arm60::{Arm60, Arm60Bus};
use crate::input::{
    DOWN, FACE_EAST, FACE_NORTH, FACE_SOUTH, L1, LEFT, R1, RIGHT, SELECT, START, UP,
};
use crate::kernel::{AudioBuffer, InputState, VideoBuffer};
use crate::machine::Machine;
use crate::platform::PlatformId;
use crate::resources::{ResourceBlob, ResourceKind};
use crate::state::{StateReader, StateWriter};

const CPU_HZ: u64 = 12_500_000;
const AUDIO_RATE: u64 = 44_100;
const SECTORS_PER_SECOND: u64 = 150;
const FRAME_RATE: u64 = 60;
const WIDTH: usize = 640;
const HEIGHT: usize = 480;
const BIOS_SIZE: usize = 1024 * 1024;
const DRAM_SIZE: usize = 2 * 1024 * 1024;
const VRAM_SIZE: usize = 2 * 1024 * 1024;
const NVRAM_SIZE: usize = 32 * 1024;
const STATE_VERSION: u32 = 2;

struct ThreeDoCd {
    disc: Option<DiscImage>,
    lba: u32,
    reading: bool,
    phase: u64,
    fifo: VecDeque<u8>,
    irq: bool,
}

impl ThreeDoCd {
    fn new(disc: Option<ResourceBlob>) -> Result<Self, String> {
        Ok(Self {
            disc: disc.map(DiscImage::new).transpose()?,
            lba: 0,
            reading: false,
            phase: 0,
            fifo: VecDeque::new(),
            irq: false,
        })
    }

    fn reset(&mut self) {
        self.lba = 0;
        self.reading = false;
        self.phase = 0;
        self.fifo.clear();
        self.irq = false;
    }

    fn start_read(&mut self, lba: u32) {
        self.lba = lba;
        self.reading = true;
        self.irq = false;
    }
    fn tick(&mut self, cycles: u32) {
        if !self.reading || self.fifo.len() > 4096 {
            return;
        }
        self.phase = self
            .phase
            .saturating_add(u64::from(cycles) * SECTORS_PER_SECOND);
        while self.phase >= CPU_HZ {
            self.phase -= CPU_HZ;
            let Some(disc) = &self.disc else {
                self.reading = false;
                break;
            };
            let mut data = [0u8; 2048];
            if disc.read_user_sector(self.lba, &mut data).is_err() {
                break;
            }
            self.fifo.extend(data);
            self.lba = self.lba.wrapping_add(1);
            self.irq = true;
            if self.fifo.len() > 4096 {
                break;
            }
        }
    }

    fn pop(&mut self) -> u8 {
        self.fifo.pop_front().unwrap_or(0)
    }

    fn status(&self) -> u32 {
        u32::from(self.disc.is_some()) | (u32::from(self.reading) << 1) | (u32::from(self.irq) << 2)
    }

    fn save(&self, out: &mut StateWriter) {
        out.u32(self.lba);
        out.u8(u8::from(self.reading));
        out.u64(self.phase);
        out.u8(u8::from(self.irq));
        out.u32(self.fifo.len() as u32);
        for byte in &self.fifo {
            out.u8(*byte);
        }
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.lba = input.u32()?;
        self.reading = input.u8()? != 0;
        self.phase = input.u64()? % CPU_HZ;
        self.irq = input.u8()? != 0;
        self.fifo.clear();
        let len = input.u32()?.min(8192);
        for _ in 0..len {
            self.fifo.push_back(input.u8()?);
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Default)]
struct DsppVoice {
    start: u32,
    end: u32,
    position: u32,
    step: u32,
    left: u8,
    right: u8,
    enabled: bool,
}
#[derive(Default)]
struct ThreeDoDspp {
    voices: [DsppVoice; 16],
    sample_phase: u64,
    samples: Vec<(f32, f32)>,
}

impl ThreeDoDspp {
    fn begin_frame(&mut self) {
        self.samples.clear();
    }

    fn configure(&mut self, voice: usize, register: usize, value: u32) {
        let Some(slot) = self.voices.get_mut(voice) else {
            return;
        };
        match register {
            0 => {
                slot.start = value & 0x001f_fffe;
                if !slot.enabled {
                    slot.position = slot.start << 16;
                }
            }
            1 => slot.end = value & 0x001f_fffe,
            2 => slot.step = value.max(1),
            3 => {
                slot.left = (value & 0xff) as u8;
                slot.right = ((value >> 8) & 0xff) as u8;
            }
            4 => {
                slot.enabled = value & 1 != 0;
                if slot.enabled {
                    slot.position = slot.start << 16;
                }
            }
            _ => {}
        }
    }

    fn sample(&mut self, dram: &[u8]) -> (f32, f32) {
        let mut left = 0.0f32;
        let mut right = 0.0f32;
        for voice in &mut self.voices {
            if !voice.enabled || voice.end <= voice.start || dram.len() < 2 {
                continue;
            }
            let address = ((voice.position >> 16) as usize) & (dram.len() - 1);
            let next = (address + 1) & (dram.len() - 1);
            let raw = i16::from_le_bytes([dram[address], dram[next]]);
            let sample = f32::from(raw) / 32768.0;
            left += sample * (f32::from(voice.left) / 255.0);
            right += sample * (f32::from(voice.right) / 255.0);
            voice.position = voice.position.wrapping_add(voice.step.max(1));
            if voice.position >> 16 >= voice.end {
                voice.position = voice.start << 16;
            }
        }
        (left.clamp(-1.0, 1.0), right.clamp(-1.0, 1.0))
    }

    fn tick(&mut self, cycles: u32, dram: &[u8]) {
        self.sample_phase = self
            .sample_phase
            .saturating_add(u64::from(cycles) * AUDIO_RATE);
        while self.sample_phase >= CPU_HZ {
            self.sample_phase -= CPU_HZ;
            let sample = self.sample(dram);
            self.samples.push(sample);
        }
    }

    fn save(&self, out: &mut StateWriter) {
        out.u64(self.sample_phase);
        for voice in self.voices {
            out.u32(voice.start);
            out.u32(voice.end);
            out.u32(voice.position);
            out.u32(voice.step);
            out.u8(voice.left);
            out.u8(voice.right);
            out.u8(u8::from(voice.enabled));
        }
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.sample_phase = input.u64()? % CPU_HZ;
        for voice in &mut self.voices {
            voice.start = input.u32()?;
            voice.end = input.u32()?;
            voice.position = input.u32()?;
            voice.step = input.u32()?.max(1);
            voice.left = input.u8()?;
            voice.right = input.u8()?;
            voice.enabled = input.u8()? != 0;
        }
        self.samples.clear();
        Ok(())
    }
}

struct ThreeDoClio {
    regs: Box<[u8; 0x4000]>,
    irq_status: u32,
    irq_enable: u32,
    pad: [u8; 2],
    dspp: ThreeDoDspp,
}
impl Default for ThreeDoClio {
    fn default() -> Self {
        let mut regs = Box::new([0u8; 0x4000]);
        regs[0x28] = 1;
        Self {
            regs,
            irq_status: 0,
            irq_enable: 0,
            pad: [0; 2],
            dspp: ThreeDoDspp::default(),
        }
    }
}

impl ThreeDoClio {
    fn reset(&mut self) {
        *self = Self::default();
    }

    fn set_input(&mut self, input: &InputState) {
        let buttons = input.buttons[0];
        self.pad[0] = u8::from(buttons & FACE_SOUTH != 0)
            | (u8::from(buttons & LEFT != 0) << 1)
            | (u8::from(buttons & RIGHT != 0) << 2)
            | (u8::from(buttons & UP != 0) << 3)
            | (u8::from(buttons & DOWN != 0) << 4);
        self.pad[1] = (u8::from(buttons & FACE_EAST != 0) << 7)
            | (u8::from(buttons & FACE_NORTH != 0) << 6)
            | (u8::from(buttons & START != 0) << 5)
            | (u8::from(buttons & SELECT != 0) << 4)
            | (u8::from(buttons & R1 != 0) << 3)
            | (u8::from(buttons & L1 != 0) << 2);
    }

    fn raise(&mut self, bit: u32) {
        self.irq_status |= bit;
    }
    fn irq_pending(&self) -> bool {
        self.irq_status & self.irq_enable != 0
    }
    fn read32(&self, offset: usize) -> u32 {
        match offset & !3 {
            0x0000 => 0x0202_0000,
            0x0028 => u32::from_le_bytes(self.regs[0x28..0x2c].try_into().unwrap()),
            0x0040 => self.irq_status,
            0x0044 => self.irq_enable,
            0x0100 => u32::from(self.pad[0]) | (u32::from(self.pad[1]) << 8),
            offset if (0x2000..0x2800).contains(&offset) => {
                let voice = (offset - 0x2000) / 0x20;
                let register = ((offset - 0x2000) % 0x20) / 4;
                let Some(slot) = self.dspp.voices.get(voice) else {
                    return 0;
                };
                match register {
                    0 => slot.start,
                    1 => slot.end,
                    2 => slot.step,
                    3 => u32::from(slot.left) | (u32::from(slot.right) << 8),
                    4 => u32::from(slot.enabled),
                    _ => 0,
                }
            }
            offset => {
                let end = (offset + 4).min(self.regs.len());
                if end - offset != 4 {
                    0
                } else {
                    u32::from_le_bytes(self.regs[offset..end].try_into().unwrap())
                }
            }
        }
    }

    fn write32(&mut self, offset: usize, value: u32) {
        let offset = offset & !3;
        match offset {
            0x0028 => self.regs[0x28..0x2c].copy_from_slice(&value.to_le_bytes()),
            0x0040 => self.irq_status &= !value,
            0x0044 => self.irq_enable = value,
            offset if (0x2000..0x2800).contains(&offset) => {
                let voice = (offset - 0x2000) / 0x20;
                let register = ((offset - 0x2000) % 0x20) / 4;
                self.dspp.configure(voice, register, value);
            }
            offset if offset + 4 <= self.regs.len() => {
                self.regs[offset..offset + 4].copy_from_slice(&value.to_le_bytes())
            }
            _ => {}
        }
    }

    fn save(&self, out: &mut StateWriter) {
        out.blob(self.regs.as_ref());
        out.u32(self.irq_status);
        out.u32(self.irq_enable);
        out.u8(self.pad[0]);
        out.u8(self.pad[1]);
        self.dspp.save(out);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let regs = input.blob()?;
        if regs.len() != self.regs.len() {
            return Err("3DO CLIO state size mismatch".into());
        }
        self.regs.copy_from_slice(regs);
        self.irq_status = input.u32()?;
        self.irq_enable = input.u32()?;
        self.pad = [input.u8()?, input.u8()?];
        self.dspp.load(input)
    }
}

struct ThreeDoMadam {
    regs: Box<[u8; 0x800]>,
    cel_pending: bool,
}

impl Default for ThreeDoMadam {
    fn default() -> Self {
        let mut regs = Box::new([0u8; 0x800]);
        regs[0..4].copy_from_slice(&0x0102_0200u32.to_le_bytes());
        regs[4..8].copy_from_slice(&0x0000_0051u32.to_le_bytes());
        Self {
            regs,
            cel_pending: false,
        }
    }
}

impl ThreeDoMadam {
    fn reset(&mut self) {
        *self = Self::default();
    }

    fn read32(&self, offset: usize) -> u32 {
        let offset = offset & !3;
        if offset + 4 > self.regs.len() {
            return 0;
        }
        u32::from_le_bytes(self.regs[offset..offset + 4].try_into().unwrap())
    }

    fn write32(&mut self, offset: usize, value: u32) {
        let offset = offset & !3;
        if offset + 4 <= self.regs.len() {
            self.regs[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
        }
        if offset == 0x0100 {
            self.cel_pending = true;
        }
        if offset == 0x0104 {
            self.cel_pending = false;
        }
    }
    fn read_source(dram: &[u8], vram: &[u8], address: u32) -> u16 {
        let address = address as usize;
        if address + 1 < dram.len() {
            u16::from_le_bytes([dram[address], dram[address + 1]])
        } else if (0x0020_0000..0x0040_0000).contains(&address) {
            let offset = (address - 0x0020_0000) & (vram.len() - 1);
            u16::from_le_bytes([vram[offset], vram[(offset + 1) & (vram.len() - 1)]])
        } else {
            0
        }
    }

    fn run_cel(&mut self, dram: &[u8], vram: &mut [u8]) -> bool {
        if !self.cel_pending {
            return false;
        }
        let clip = self.read32(0x0134);
        let width = (clip & 0xffff).clamp(1, 320) as usize;
        let height = ((clip >> 16) & 0xffff).clamp(1, 240) as usize;
        let source = self.read32(0x0138);
        let destination = self.read32(0x013c);
        for y in 0..height {
            for x in 0..width {
                let pixel = y * width + x;
                let color = Self::read_source(dram, vram, source.wrapping_add((pixel * 2) as u32));
                let dest = destination.wrapping_sub(0x0020_0000) as usize + pixel * 2;
                if dest + 1 < vram.len() {
                    vram[dest..dest + 2].copy_from_slice(&color.to_le_bytes());
                }
            }
        }
        self.cel_pending = false;
        true
    }
    fn save(&self, out: &mut StateWriter) {
        out.blob(self.regs.as_ref());
        out.u8(u8::from(self.cel_pending));
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let regs = input.blob()?;
        if regs.len() != self.regs.len() {
            return Err("3DO MADAM state size mismatch".into());
        }
        self.regs.copy_from_slice(regs);
        self.cel_pending = input.u8()? != 0;
        Ok(())
    }
}

struct ThreeDoBoard {
    bios: Box<[u8]>,
    dram: Box<[u8]>,
    vram: Box<[u8]>,
    nvram: Box<[u8]>,
    overlay: bool,
    madam: ThreeDoMadam,
    clio: ThreeDoClio,
    cd: ThreeDoCd,
    video: VideoBuffer,
    frame: u64,
}

impl ThreeDoBoard {
    fn new(bios: &[u8], disc: Option<ResourceBlob>) -> Result<Self, String> {
        if bios.len() != BIOS_SIZE {
            return Err("3DO requires a 1 MiB BIOS image".into());
        }
        let mut bios_mem = vec![0u8; BIOS_SIZE].into_boxed_slice();
        bios_mem.copy_from_slice(bios);
        Ok(Self {
            bios: bios_mem,
            dram: vec![0; DRAM_SIZE].into_boxed_slice(),
            vram: vec![0; VRAM_SIZE].into_boxed_slice(),
            nvram: vec![0; NVRAM_SIZE].into_boxed_slice(),
            overlay: true,
            madam: ThreeDoMadam::default(),
            clio: ThreeDoClio::default(),
            cd: ThreeDoCd::new(disc)?,
            video: VideoBuffer::new(WIDTH as u32, HEIGHT as u32),
            frame: 0,
        })
    }

    fn reset(&mut self) {
        self.dram.fill(0);
        self.vram.fill(0);
        self.overlay = true;
        self.madam.reset();
        self.clio.reset();
        self.cd.reset();
        self.video.clear([0, 0, 0, 255]);
        self.frame = 0;
    }

    fn memory_read8(&self, address: u32) -> u8 {
        match address {
            0x0000_0000..=0x001f_ffff => {
                if self.overlay && (address as usize) < BIOS_SIZE {
                    self.bios[address as usize]
                } else {
                    self.dram[address as usize & (DRAM_SIZE - 1)]
                }
            }
            0x0020_0000..=0x003f_ffff => {
                self.vram[(address as usize - 0x0020_0000) & (VRAM_SIZE - 1)]
            }
            0x0300_0000..=0x030f_ffff => {
                self.bios[(address as usize - 0x0300_0000) & (BIOS_SIZE - 1)]
            }
            0x0314_0000..=0x0317_ffff => {
                self.nvram[(address as usize - 0x0314_0000) & (NVRAM_SIZE - 1)]
            }
            0x0320_0000..=0x0320_ffff => {
                self.vram[(address as usize - 0x0320_0000) & (VRAM_SIZE - 1)]
            }
            _ => 0,
        }
    }

    fn memory_write8(&mut self, address: u32, value: u8) {
        match address {
            0x0000_0000..=0x001f_ffff => {
                if self.overlay && (address as usize) < BIOS_SIZE {
                    self.overlay = false;
                }
                self.dram[address as usize & (DRAM_SIZE - 1)] = value;
            }
            0x0020_0000..=0x003f_ffff => {
                self.vram[(address as usize - 0x0020_0000) & (VRAM_SIZE - 1)] = value
            }
            0x0314_0000..=0x0317_ffff => {
                self.nvram[(address as usize - 0x0314_0000) & (NVRAM_SIZE - 1)] = value
            }
            0x0320_0000..=0x0320_ffff => {
                self.vram[(address as usize - 0x0320_0000) & (VRAM_SIZE - 1)] = value
            }
            _ => {}
        }
    }

    fn read8(&mut self, address: u32) -> u8 {
        match address {
            0x0330_0000..=0x0330_07ff => {
                let word = self
                    .madam
                    .read32((address as usize - 0x0330_0000) & !3)
                    .to_le_bytes();
                word[(address & 3) as usize]
            }
            0x0340_0000..=0x0340_3fff => {
                let word = self
                    .clio
                    .read32((address as usize - 0x0340_0000) & !3)
                    .to_le_bytes();
                word[(address & 3) as usize]
            }
            0x0318_0000..=0x0318_0003 => self.cd.status().to_le_bytes()[(address & 3) as usize],
            0x0318_0004..=0x0318_0007 => self.cd.lba.to_le_bytes()[(address & 3) as usize],
            0x0318_0008..=0x0318_000b => {
                (self.cd.fifo.len() as u32).to_le_bytes()[(address & 3) as usize]
            }
            0x0318_000c..=0x0318_000f => self.cd.pop(),
            _ => self.memory_read8(address),
        }
    }

    fn write_device_byte(current: u32, address: u32, value: u8) -> u32 {
        let mut bytes = current.to_le_bytes();
        bytes[(address & 3) as usize] = value;
        u32::from_le_bytes(bytes)
    }

    fn write8(&mut self, address: u32, value: u8) {
        match address {
            0x0330_0000..=0x0330_07ff => {
                let offset = (address as usize - 0x0330_0000) & !3;
                let value = Self::write_device_byte(self.madam.read32(offset), address, value);
                self.write32(0x0330_0000 + offset as u32, value);
            }
            0x0340_0000..=0x0340_3fff => {
                let offset = (address as usize - 0x0340_0000) & !3;
                let value = Self::write_device_byte(self.clio.read32(offset), address, value);
                self.write32(0x0340_0000 + offset as u32, value);
            }
            0x0318_0000..=0x0318_000b => {
                let aligned = address & !3;
                let current = self.read32(aligned);
                let value = Self::write_device_byte(current, address, value);
                self.write32(aligned, value);
            }
            _ => self.memory_write8(address, value),
        }
    }

    fn read32(&mut self, address: u32) -> u32 {
        let aligned = address & !3;
        match aligned {
            0x0318_0000 => self.cd.status(),
            0x0318_0004 => self.cd.lba,
            0x0318_0008 => self.cd.fifo.len().min(u32::MAX as usize) as u32,
            0x0318_000c => {
                u32::from_le_bytes([self.cd.pop(), self.cd.pop(), self.cd.pop(), self.cd.pop()])
            }
            0x0330_0000..=0x0330_07ff => self.madam.read32((aligned - 0x0330_0000) as usize),
            0x0340_0000..=0x0340_3fff => self.clio.read32((aligned - 0x0340_0000) as usize),
            _ => u32::from_le_bytes([
                self.memory_read8(aligned),
                self.memory_read8(aligned + 1),
                self.memory_read8(aligned + 2),
                self.memory_read8(aligned + 3),
            ]),
        }
    }

    fn write32(&mut self, address: u32, value: u32) {
        let aligned = address & !3;
        match aligned {
            0x0318_0000 => {
                if value & 1 != 0 {
                    self.cd.start_read(self.cd.lba);
                } else {
                    self.cd.reading = false;
                }
                if value & 4 != 0 {
                    self.cd.irq = false;
                }
            }
            0x0318_0004 => self.cd.lba = value,
            0x0330_0000..=0x0330_07ff => {
                self.madam.write32((aligned - 0x0330_0000) as usize, value);
                if self.madam.run_cel(self.dram.as_ref(), self.vram.as_mut()) {
                    self.clio.raise(1 << 2);
                }
            }
            0x0340_0000..=0x0340_3fff => self.clio.write32((aligned - 0x0340_0000) as usize, value),
            _ => {
                for (offset, byte) in value.to_le_bytes().into_iter().enumerate() {
                    self.memory_write8(aligned + offset as u32, byte);
                }
            }
        }
    }

    fn tick(&mut self, cycles: u32) {
        self.cd.tick(cycles);
        if self.cd.irq {
            self.clio.raise(1 << 1);
        }
        self.clio.dspp.tick(cycles, self.dram.as_ref());
    }

    fn render(&mut self) {
        self.video.clear([0, 0, 0, 255]);
        for y in 0..240usize {
            for x in 0..320usize {
                let source = (y * 320 + x) * 2;
                let word = u16::from_le_bytes([self.vram[source], self.vram[source + 1]]);
                let r = (((word >> 10) & 0x1f) * 255 / 31) as u8;
                let g = (((word >> 5) & 0x1f) * 255 / 31) as u8;
                let b = ((word & 0x1f) * 255 / 31) as u8;
                for dy in 0..2 {
                    for dx in 0..2 {
                        let target = ((y * 2 + dy) * WIDTH + x * 2 + dx) * 4;
                        self.video.pixels_mut()[target..target + 4]
                            .copy_from_slice(&[r, g, b, 255]);
                    }
                }
            }
        }
    }

    fn end_frame(&mut self) {
        self.render();
        self.clio.raise(1);
        self.frame = self.frame.wrapping_add(1);
    }

    fn save(&self, out: &mut StateWriter) {
        out.blob(self.dram.as_ref());
        out.blob(self.vram.as_ref());
        out.blob(self.nvram.as_ref());
        out.u8(u8::from(self.overlay));
        self.madam.save(out);
        self.clio.save(out);
        self.cd.save(out);
        out.u64(self.frame);
    }
    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        for (name, target) in [
            ("DRAM", self.dram.as_mut()),
            ("VRAM", self.vram.as_mut()),
            ("NVRAM", self.nvram.as_mut()),
        ] {
            let data = input.blob()?;
            if data.len() != target.len() {
                return Err(format!("3DO {name} state size mismatch"));
            }
            target.copy_from_slice(data);
        }
        self.overlay = input.u8()? != 0;
        self.madam.load(input)?;
        self.clio.load(input)?;
        self.cd.load(input)?;
        self.frame = input.u64()?;
        self.render();
        Ok(())
    }
}

struct ThreeDoBus<'a> {
    board: &'a mut ThreeDoBoard,
}

impl Arm60Bus for ThreeDoBus<'_> {
    fn read8(&mut self, address: u32) -> u8 {
        self.board.read8(address)
    }
    fn write8(&mut self, address: u32, value: u8) {
        self.board.write8(address, value);
    }
    fn read32(&mut self, address: u32) -> u32 {
        self.board.read32(address)
    }
    fn write32(&mut self, address: u32, value: u32) {
        self.board.write32(address, value);
    }
}

pub struct ThreeDoMachine {
    cpu: Arm60,
    board: ThreeDoBoard,
    audio: AudioBuffer,
    frame_phase: u64,
    powered: bool,
}
impl ThreeDoMachine {
    pub fn from_bios_and_disc(bios: &[u8], disc: Option<ResourceBlob>) -> Result<Self, String> {
        let board = ThreeDoBoard::new(bios, disc)?;
        let mut cpu = Arm60::default();
        cpu.reset(0, 0);
        Ok(Self {
            cpu,
            board,
            audio: AudioBuffer::new(AUDIO_RATE as u32, 2),
            frame_phase: 0,
            powered: true,
        })
    }

    fn clock_cpu(&mut self) -> u32 {
        if self.board.clio.irq_pending() {
            let used = self.cpu.fiq();
            if used != 0 {
                self.board.tick(used);
                return used;
            }
        }
        let used = {
            let mut bus = ThreeDoBus {
                board: &mut self.board,
            };
            self.cpu.step(&mut bus)
        };
        self.board.tick(used);
        used
    }

    fn flush_audio(&mut self) {
        self.audio.begin_frame();
        for &(left, right) in &self.board.clio.dspp.samples {
            self.audio.push_stereo(left, right);
        }
    }
    fn run_budget(&mut self, target: u64) {
        let deadline = target.saturating_add(64);
        while self.cpu.cycles < target && self.cpu.cycles < deadline {
            if self.clock_cpu() == 0 {
                self.powered = false;
                break;
            }
        }
        if self.cpu.cycles < target {
            self.powered = false;
        }
    }

    fn begin_frame(&mut self) {
        self.board.clio.dspp.begin_frame();
        self.audio.begin_frame();
    }
}

impl Machine for ThreeDoMachine {
    fn platform(&self) -> PlatformId {
        PlatformId::ThreeDo
    }

    fn reset(&mut self) {
        self.board.reset();
        self.cpu.reset(0, 0);
        self.frame_phase = 0;
        self.powered = true;
        self.begin_frame();
    }

    fn run_frame(&mut self, input: &InputState) {
        if !self.powered {
            return;
        }
        self.board.clio.set_input(input);
        self.begin_frame();
        self.frame_phase = self.frame_phase.saturating_add(CPU_HZ);
        let budget = self.frame_phase / FRAME_RATE;
        self.frame_phase %= FRAME_RATE;
        let target = self.cpu.cycles.saturating_add(budget);
        self.run_budget(target);
        if self.powered {
            self.board.end_frame();
        }
        self.flush_audio();
    }

    fn frame_rate(&self) -> f64 {
        FRAME_RATE as f64
    }
    fn video(&self) -> &VideoBuffer {
        &self.board.video
    }
    fn audio(&self) -> &AudioBuffer {
        &self.audio
    }

    fn save_state(&self) -> Result<Vec<u8>, String> {
        let mut out = StateWriter::new(PlatformId::ThreeDo, STATE_VERSION);
        self.cpu.save(&mut out);
        self.board.save(&mut out);
        out.u64(self.frame_phase);
        out.u8(u8::from(self.powered));
        Ok(out.finish())
    }

    fn load_state(&mut self, bytes: &[u8]) -> Result<(), String> {
        let mut input = StateReader::new(bytes, PlatformId::ThreeDo, STATE_VERSION)?;
        self.cpu.load(&mut input)?;
        self.board.load(&mut input)?;
        self.frame_phase = input.u64()? % FRAME_RATE;
        self.powered = input.u8()? != 0;
        input.finish()?;
        self.audio.begin_frame();
        Ok(())
    }

    fn persistent_len(&self, kind: ResourceKind, slot: u32) -> usize {
        if kind == ResourceKind::Storage && slot == 0 {
            NVRAM_SIZE
        } else {
            0
        }
    }

    fn read_persistent(&self, kind: ResourceKind, slot: u32, out: &mut [u8]) -> Result<(), String> {
        if kind != ResourceKind::Storage || slot != 0 {
            return Err("3DO persistent resource is Storage slot 0".into());
        }
        if out.len() != NVRAM_SIZE {
            return Err("3DO NVRAM buffer has the wrong size".into());
        }
        out.copy_from_slice(self.board.nvram.as_ref());
        Ok(())
    }

    fn write_persistent(
        &mut self,
        kind: ResourceKind,
        slot: u32,
        data: &[u8],
    ) -> Result<(), String> {
        if kind != ResourceKind::Storage || slot != 0 {
            return Err("3DO persistent resource is Storage slot 0".into());
        }
        if data.len() != NVRAM_SIZE {
            return Err("3DO NVRAM image has the wrong size".into());
        }
        self.board.nvram.copy_from_slice(data);
        Ok(())
    }
}
#[cfg(test)]
mod tests {
    use super::*;

    fn synthetic_bios() -> Vec<u8> {
        let mut bios = vec![0; BIOS_SIZE];
        bios[..4].copy_from_slice(&0xeaff_fffeu32.to_le_bytes());
        bios
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

    #[test]
    fn boot_overlay_switches_to_dram_on_low_memory_write() {
        let bios = synthetic_bios();
        let mut board = ThreeDoBoard::new(&bios, None).unwrap();
        assert_eq!(board.read32(0), 0xeaff_fffe);
        board.write32(0, 0xe3a0_002a);
        assert!(!board.overlay);
        assert_eq!(board.read32(0), 0xe3a0_002a);
        assert_eq!(board.memory_read8(0x0300_0000), 0xfe);
    }

    #[test]
    fn clio_controller_and_dspp_pcm_are_live() {
        let bios = synthetic_bios();
        let mut board = ThreeDoBoard::new(&bios, None).unwrap();
        let mut input = InputState::default();
        input.buttons[0] = FACE_SOUTH | LEFT | START | R1;
        board.clio.set_input(&input);
        let pad = board.clio.read32(0x0100);
        assert_eq!(pad & 0x03, 0x03);
        assert_ne!(pad & (1 << 13), 0);
        assert_ne!(pad & (1 << 11), 0);
        board.dram[..8].copy_from_slice(&[0xff, 0x7f, 0x00, 0x40, 0x00, 0xc0, 0x01, 0x80]);
        board.clio.dspp.configure(0, 0, 0);
        board.clio.dspp.configure(0, 1, 8);
        board.clio.dspp.configure(0, 2, 1 << 16);
        board.clio.dspp.configure(0, 3, 0xffff);
        board.clio.dspp.configure(0, 4, 1);
        board
            .clio
            .dspp
            .tick((CPU_HZ / AUDIO_RATE * 8) as u32, board.dram.as_ref());
        assert!(board
            .clio
            .dspp
            .samples
            .iter()
            .any(|&(l, r)| l.abs() > 0.1 && r.abs() > 0.1));
    }

    #[test]
    fn madam_cel_reaches_the_display_surface() {
        let bios = synthetic_bios();
        let mut board = ThreeDoBoard::new(&bios, None).unwrap();
        board.dram[0x100..0x102].copy_from_slice(&0x7c1fu16.to_le_bytes());
        board.write32(0x0330_0134, 0x0001_0001);
        board.write32(0x0330_0138, 0x0000_0100);
        board.write32(0x0330_013c, 0x0020_0000);
        board.write32(0x0330_0100, 1);
        board.end_frame();
        let pixels = board.video.pixels();
        assert!(pixels[0] > 200 && pixels[2] > 200);
        assert_ne!(board.clio.irq_status & (1 << 2), 0);
    }

    #[test]
    fn streaming_cd_requests_missing_sector_and_resumes() {
        let mut producer = ResourceBlob::streaming(4 * 2048, 2).unwrap();
        let shared = producer.clone();
        let mut cd = ThreeDoCd::new(Some(shared.clone())).unwrap();
        cd.start_read(1);
        cd.tick((CPU_HZ / SECTORS_PER_SECOND + 1) as u32);
        assert_eq!(shared.pending_range(), Some((2048, 4096)));
        producer.write(2048, &vec![0x6c; 2048]).unwrap();
        cd.tick((CPU_HZ / SECTORS_PER_SECOND + 1) as u32);
        assert_eq!(cd.fifo.len(), 2048);
        assert_eq!(cd.pop(), 0x6c);
        assert_eq!(cd.fifo.len(), 2047);
    }

    #[test]
    fn multitrack_cue_reads_data_across_separate_track_files() {
        let cue = concat!(
            r#"FILE "track01.bin" BINARY"#,
            "\n",
            "  TRACK 01 MODE1/2048\n",
            "    INDEX 01 00:00:00\n",
            r#"FILE "track02.bin" BINARY"#,
            "\n",
            "  TRACK 02 MODE1/2048\n",
            "    INDEX 01 00:00:00\n",
        );
        let track1 = vec![0x31; 2048];
        let track2 = vec![0x72; 2048];
        let blob = cue_container(cue, &[("track01.bin", track1), ("track02.bin", track2)]);
        let mut cd = ThreeDoCd::new(Some(blob)).unwrap();

        cd.start_read(0);
        cd.tick((CPU_HZ / SECTORS_PER_SECOND + 1) as u32);
        assert_eq!(cd.fifo.len(), 2048);
        assert_eq!(cd.pop(), 0x31);

        cd.fifo.clear();
        cd.start_read(1);
        cd.tick((CPU_HZ / SECTORS_PER_SECOND + 1) as u32);
        assert_eq!(cd.fifo.len(), 2048);
        assert_eq!(cd.pop(), 0x72);
    }

    #[test]
    fn machine_frame_state_and_nvram_round_trip() {
        let bios = synthetic_bios();
        let mut machine = ThreeDoMachine::from_bios_and_disc(&bios, None).unwrap();
        machine.run_frame(&InputState::default());
        assert!(machine.powered);
        assert_eq!(machine.video().width(), WIDTH as u32);
        assert_eq!(machine.video().height(), HEIGHT as u32);
        assert_eq!(machine.audio().sample_rate(), AUDIO_RATE as u32);
        assert!(!machine.audio().samples().is_empty());
        let saved = machine.save_state().unwrap();
        let frame = machine.board.frame;
        let pc = machine.cpu.r[15];
        machine.run_frame(&InputState::default());
        machine.load_state(&saved).unwrap();
        assert_eq!(machine.board.frame, frame);
        assert_eq!(machine.cpu.r[15], pc);
        let persistent = vec![0x39; NVRAM_SIZE];
        machine
            .write_persistent(ResourceKind::Storage, 0, &persistent)
            .unwrap();
        let mut restored = vec![0; NVRAM_SIZE];
        machine
            .read_persistent(ResourceKind::Storage, 0, &mut restored)
            .unwrap();
        assert_eq!(restored, persistent);
    }
}
