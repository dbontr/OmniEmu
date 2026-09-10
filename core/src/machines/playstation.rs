use std::collections::VecDeque;

use crate::cpu_mips_r3000::{MipsBus, MipsR3000};
use crate::input::{
    DOWN, FACE_EAST, FACE_NORTH, FACE_SOUTH, FACE_WEST, L1, L2, LEFT, R1, R2, RIGHT, SELECT, START,
    UP,
};
use crate::kernel::{AudioBuffer, InputState, VideoBuffer};
use crate::machine::Machine;
use crate::platform::PlatformId;
use crate::resources::ResourceKind;
use crate::state::{StateReader, StateWriter};

use super::ps1_gpu::Ps1Gpu;

const RAM_SIZE: usize = 2 * 1024 * 1024;
const BIOS_SIZE: usize = 512 * 1024;
const SCRATCH_SIZE: usize = 1024;
const MEMORY_CARD_SIZE: usize = 128 * 1024;
const FRAME_RATE: f64 = 59.94;
const AUDIO_RATE: u32 = 44_100;
const STATE_VERSION: u32 = 1;

#[derive(Clone, Copy, Default)]
struct DmaChannel {
    base: u32,
    block: u32,
    control: u32,
}

struct Ps1Bus {
    ram: Vec<u8>,
    scratch: [u8; SCRATCH_SIZE],
    bios: Vec<u8>,
    gpu: Ps1Gpu,
    irq_status: u32,
    irq_mask: u32,
    dma: [DmaChannel; 7],
    dpcr: u32,
    dicr: u32,
    timers: [u32; 9],
    spu: [u16; 256],
    pad_buttons: [u16; 2],
    sio_rx: VecDeque<u8>,
    sio_mode: u16,
    sio_control: u16,
    sio_baud: u16,
    sio_phase: u8,
    memory_card: Vec<u8>,
    cache_control: u32,
}

impl Ps1Bus {
    fn new(bios: &[u8]) -> Result<Self, String> {
        if bios.len() != BIOS_SIZE {
            return Err(format!(
                "PlayStation BIOS must be exactly {BIOS_SIZE} bytes, got {}",
                bios.len()
            ));
        }
        Ok(Self {
            ram: vec![0; RAM_SIZE],
            scratch: [0; SCRATCH_SIZE],
            bios: bios.to_vec(),
            gpu: Ps1Gpu::new(),
            irq_status: 0,
            irq_mask: 0,
            dma: [DmaChannel::default(); 7],
            dpcr: 0x0765_4321,
            dicr: 0,
            timers: [0; 9],
            spu: [0; 256],
            pad_buttons: [0xffff; 2],
            sio_rx: VecDeque::new(),
            sio_mode: 0,
            sio_control: 0,
            sio_baud: 0,
            sio_phase: 0,
            memory_card: vec![0; MEMORY_CARD_SIZE],
            cache_control: 0,
        })
    }

    fn reset(&mut self) {
        self.ram.fill(0);
        self.scratch.fill(0);
        self.gpu.reset();
        self.irq_status = 0;
        self.irq_mask = 0;
        self.dma = [DmaChannel::default(); 7];
        self.dpcr = 0x0765_4321;
        self.dicr = 0;
        self.timers = [0; 9];
        self.spu.fill(0);
        self.sio_rx.clear();
        self.sio_mode = 0;
        self.sio_control = 0;
        self.sio_baud = 0;
        self.sio_phase = 0;
        self.cache_control = 0;
    }
    fn set_inputs(&mut self, input: &InputState) {
        for player in 0..2 {
            let mut state = 0xffffu16;
            let buttons = input.buttons[player];
            let map = [
                (SELECT, 0),
                (START, 3),
                (UP, 4),
                (RIGHT, 5),
                (DOWN, 6),
                (LEFT, 7),
                (L2, 8),
                (R2, 9),
                (L1, 10),
                (R1, 11),
                (FACE_NORTH, 12),
                (FACE_EAST, 13),
                (FACE_SOUTH, 14),
                (FACE_WEST, 15),
            ];
            for (mask, bit) in map {
                if buttons & mask != 0 {
                    state &= !(1 << bit);
                }
            }
            self.pad_buttons[player] = state;
        }
    }

    fn physical(address: u32) -> u32 {
        address & 0x1fff_ffff
    }

    fn ram_index(physical: u32) -> Option<usize> {
        if physical < 0x0080_0000 {
            Some(physical as usize & (RAM_SIZE - 1))
        } else {
            None
        }
    }

    fn read_memory8(&mut self, physical: u32) -> u8 {
        if let Some(index) = Self::ram_index(physical) {
            return self.ram[index];
        }
        match physical {
            0x1f80_0000..=0x1f80_03ff => self.scratch[(physical - 0x1f80_0000) as usize],
            0x1fc0_0000..=0x1fc7_ffff => self.bios[(physical - 0x1fc0_0000) as usize],
            0x1f80_1040 => self.sio_rx.pop_front().unwrap_or(0xff),
            0x1f80_1044 => 0x05,
            0x1f80_1800..=0x1f80_1803 => 0,
            address if (0x1f80_1c00..=0x1f80_1dff).contains(&address) => {
                let word = self.spu[((address - 0x1f80_1c00) >> 1) as usize];
                if address & 1 == 0 {
                    word as u8
                } else {
                    (word >> 8) as u8
                }
            }
            _ => 0,
        }
    }

    fn write_memory8(&mut self, physical: u32, value: u8) {
        if let Some(index) = Self::ram_index(physical) {
            self.ram[index] = value;
            return;
        }
        match physical {
            0x1f80_0000..=0x1f80_03ff => self.scratch[(physical - 0x1f80_0000) as usize] = value,
            0x1f80_1040 => self.sio_write(value),
            0x1f80_1800..=0x1f80_1803 => {}
            address if (0x1f80_1c00..=0x1f80_1dff).contains(&address) => {
                let index = ((address - 0x1f80_1c00) >> 1) as usize;
                if address & 1 == 0 {
                    self.spu[index] = (self.spu[index] & 0xff00) | u16::from(value);
                } else {
                    self.spu[index] = (self.spu[index] & 0x00ff) | (u16::from(value) << 8);
                }
            }
            _ => {}
        }
    }
    fn sio_write(&mut self, value: u8) {
        let response = match self.sio_phase {
            0 => {
                if value == 0x01 {
                    self.sio_phase = 1;
                }
                0xff
            }
            1 => {
                self.sio_phase = if value == 0x42 { 2 } else { 0 };
                if value == 0x42 {
                    0x41
                } else {
                    0xff
                }
            }
            2 => {
                self.sio_phase = 3;
                0x5a
            }
            3 => {
                self.sio_phase = 4;
                self.pad_buttons[0] as u8
            }
            _ => {
                self.sio_phase = 0;
                (self.pad_buttons[0] >> 8) as u8
            }
        };
        self.sio_rx.push_back(response);
        self.irq_status |= 1 << 7;
    }

    fn read_word_le(&mut self, physical: u32) -> u32 {
        u32::from_le_bytes([
            self.read_memory8(physical),
            self.read_memory8(physical.wrapping_add(1)),
            self.read_memory8(physical.wrapping_add(2)),
            self.read_memory8(physical.wrapping_add(3)),
        ])
    }

    fn write_word_le(&mut self, physical: u32, value: u32) {
        for (offset, byte) in value.to_le_bytes().into_iter().enumerate() {
            self.write_memory8(physical.wrapping_add(offset as u32), byte);
        }
    }
    fn dma_register(&self, physical: u32) -> Option<u32> {
        if !(0x1f80_1080..=0x1f80_10ef).contains(&physical) {
            return None;
        }
        let relative = physical - 0x1f80_1080;
        let channel = (relative / 0x10) as usize;
        if channel >= 7 {
            return None;
        }
        Some(match relative & 0x0f {
            0x0 => self.dma[channel].base,
            0x4 => self.dma[channel].block,
            0x8 => self.dma[channel].control,
            _ => 0,
        })
    }

    fn write_dma_register(&mut self, physical: u32, value: u32) -> bool {
        if !(0x1f80_1080..=0x1f80_10ef).contains(&physical) {
            return false;
        }
        let relative = physical - 0x1f80_1080;
        let channel = (relative / 0x10) as usize;
        if channel >= 7 {
            return false;
        }
        match relative & 0x0f {
            0x0 => self.dma[channel].base = value & 0x00ff_ffff,
            0x4 => self.dma[channel].block = value,
            0x8 => {
                self.dma[channel].control = value;
                if value & (1 << 24) != 0 {
                    self.run_dma(channel);
                }
            }
            _ => {}
        }
        true
    }
    fn run_dma(&mut self, channel: usize) {
        match channel {
            2 => self.run_gpu_dma(),
            6 => self.run_otc_dma(),
            _ => {}
        }
        self.dma[channel].control &= !(1 << 24);
        self.dicr |= 1 << (24 + channel);
        if self.dicr & (1 << 23) != 0 && self.dicr & (1 << (16 + channel)) != 0 {
            self.dicr |= 1 << 31;
            self.irq_status |= 1 << 3;
        }
    }

    fn run_gpu_dma(&mut self) {
        let control = self.dma[2].control;
        let direction_to_gpu = control & 1 != 0;
        let sync = (control >> 9) & 3;
        if sync == 2 && direction_to_gpu {
            self.run_gpu_linked_list();
            return;
        }
        let block = self.dma[2].block;
        let words = match sync {
            1 => (block & 0xffff).max(1).saturating_mul((block >> 16).max(1)),
            _ => (block & 0xffff).max(1),
        };
        let step = if control & 2 != 0 { -4i32 } else { 4i32 };
        let mut address = self.dma[2].base & 0x001f_fffc;
        for _ in 0..words.min(4_000_000) {
            if direction_to_gpu {
                let value = self.read_word_le(address);
                self.gpu.gp0(value);
            } else {
                let value = self.gpu.gpuread();
                self.write_word_le(address, value);
            }
            address = address.wrapping_add_signed(step) & 0x001f_fffc;
        }
        self.dma[2].base = address;
    }

    fn run_gpu_linked_list(&mut self) {
        let mut address = self.dma[2].base & 0x001f_fffc;
        let mut packets = 0usize;
        while packets < 100_000 {
            let header = self.read_word_le(address);
            let count = (header >> 24) as usize;
            let mut word_address = address.wrapping_add(4) & 0x001f_fffc;
            for _ in 0..count {
                let word = self.read_word_le(word_address);
                self.gpu.gp0(word);
                word_address = word_address.wrapping_add(4) & 0x001f_fffc;
            }
            packets += 1;
            if header & 0x0080_0000 != 0 {
                break;
            }
            let next = header & 0x001f_fffc;
            if next == address {
                break;
            }
            address = next;
        }
        self.dma[2].base = address;
    }
    fn run_otc_dma(&mut self) {
        let count = (self.dma[6].block & 0xffff).clamp(1, 0x20_000);
        let mut address = self.dma[6].base & 0x001f_fffc;
        for index in 0..count {
            let value = if index + 1 == count {
                0x00ff_ffff
            } else {
                address.wrapping_sub(4) & 0x001f_ffff
            };
            self.write_word_le(address, value);
            address = address.wrapping_sub(4) & 0x001f_fffc;
        }
        self.dma[6].base = address;
    }

    fn tick(&mut self, cpu_cycles: u32) {
        let before = self.gpu.frame();
        self.gpu.tick_cpu_cycles(cpu_cycles);
        if self.gpu.frame() != before {
            self.irq_status |= 1;
        }
        if self.gpu.irq_pending() {
            self.irq_status |= 1 << 1;
        }
        self.timers[0] = self.timers[0].wrapping_add(cpu_cycles);
        self.timers[3] = self.timers[3].wrapping_add(cpu_cycles);
        self.timers[6] = self.timers[6].wrapping_add(cpu_cycles);
    }

    fn irq_pending(&self) -> bool {
        self.irq_status & self.irq_mask != 0
    }
    fn read_mmio32(&mut self, physical: u32) -> Option<u32> {
        if let Some(value) = self.dma_register(physical) {
            return Some(value);
        }
        Some(match physical {
            0x1f80_1070 => self.irq_status,
            0x1f80_1074 => self.irq_mask,
            0x1f80_10f0 => self.dpcr,
            0x1f80_10f4 => self.dicr,
            0x1f80_1810 => self.gpu.gpuread(),
            0x1f80_1814 => self.gpu.status(),
            address if (0x1f80_1100..=0x1f80_1128).contains(&address) => {
                let index = ((address - 0x1f80_1100) / 4) as usize;
                self.timers.get(index).copied().unwrap_or(0)
            }
            0x1ffe_0130 => self.cache_control,
            _ => return None,
        })
    }

    fn write_mmio32(&mut self, physical: u32, value: u32) -> bool {
        if self.write_dma_register(physical, value) {
            return true;
        }
        match physical {
            0x1f80_1070 => self.irq_status &= value,
            0x1f80_1074 => self.irq_mask = value & 0x7ff,
            0x1f80_10f0 => self.dpcr = value,
            0x1f80_10f4 => {
                let acknowledge = (value >> 24) & 0x7f;
                self.dicr =
                    (self.dicr & 0x7f00_0000 & !(acknowledge << 24)) | (value & 0x00ff_ffff);
            }
            0x1f80_1810 => self.gpu.gp0(value),
            0x1f80_1814 => self.gpu.gp1(value),
            address if (0x1f80_1100..=0x1f80_1128).contains(&address) => {
                let index = ((address - 0x1f80_1100) / 4) as usize;
                if let Some(register) = self.timers.get_mut(index) {
                    *register = value;
                }
            }
            0x1ffe_0130 => self.cache_control = value,
            _ => return false,
        }
        true
    }

    fn save(&self, out: &mut StateWriter) {
        out.blob(&self.ram);
        out.blob(&self.scratch);
        self.gpu.save(out);
        out.u32(self.irq_status);
        out.u32(self.irq_mask);
        for channel in self.dma {
            out.u32(channel.base);
            out.u32(channel.block);
            out.u32(channel.control);
        }
        out.u32(self.dpcr);
        out.u32(self.dicr);
        for value in self.timers {
            out.u32(value);
        }
        for value in self.spu {
            out.u16(value);
        }
        out.u16(self.pad_buttons[0]);
        out.u16(self.pad_buttons[1]);
        out.u32(self.sio_rx.len() as u32);
        for value in &self.sio_rx {
            out.u8(*value);
        }
        out.u16(self.sio_mode);
        out.u16(self.sio_control);
        out.u16(self.sio_baud);
        out.u8(self.sio_phase);
        out.blob(&self.memory_card);
        out.u32(self.cache_control);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let ram = input.blob()?;
        if ram.len() != self.ram.len() {
            return Err("invalid PlayStation RAM state length".into());
        }
        self.ram.copy_from_slice(ram);
        let scratch = input.blob()?;
        if scratch.len() != self.scratch.len() {
            return Err("invalid PlayStation scratch state length".into());
        }
        self.scratch.copy_from_slice(scratch);
        self.gpu.load(input)?;
        self.irq_status = input.u32()?;
        self.irq_mask = input.u32()?;
        for channel in &mut self.dma {
            channel.base = input.u32()?;
            channel.block = input.u32()?;
            channel.control = input.u32()?;
        }
        self.dpcr = input.u32()?;
        self.dicr = input.u32()?;
        for value in &mut self.timers {
            *value = input.u32()?;
        }
        for value in &mut self.spu {
            *value = input.u16()?;
        }
        self.pad_buttons = [input.u16()?, input.u16()?];
        let rx_len = input.u32()? as usize;
        if rx_len > 64 {
            return Err("invalid PlayStation SIO queue length".into());
        }
        self.sio_rx.clear();
        for _ in 0..rx_len {
            self.sio_rx.push_back(input.u8()?);
        }
        self.sio_mode = input.u16()?;
        self.sio_control = input.u16()?;
        self.sio_baud = input.u16()?;
        self.sio_phase = input.u8()?.min(4);
        let card = input.blob()?;
        if card.len() != self.memory_card.len() {
            return Err("invalid PlayStation memory-card state length".into());
        }
        self.memory_card.copy_from_slice(card);
        self.cache_control = input.u32()?;
        Ok(())
    }
}

impl MipsBus for Ps1Bus {
    fn read8(&mut self, address: u32) -> u8 {
        self.read_memory8(Self::physical(address))
    }

    fn write8(&mut self, address: u32, value: u8) {
        self.write_memory8(Self::physical(address), value);
    }

    fn read16(&mut self, address: u32) -> u16 {
        let physical = Self::physical(address);
        match physical {
            0x1f80_1048 => self.sio_mode,
            0x1f80_104a => self.sio_control,
            0x1f80_104e => self.sio_baud,
            _ => u16::from_le_bytes([
                self.read_memory8(physical),
                self.read_memory8(physical.wrapping_add(1)),
            ]),
        }
    }
    fn write16(&mut self, address: u32, value: u16) {
        let physical = Self::physical(address);
        match physical {
            0x1f80_1048 => self.sio_mode = value,
            0x1f80_104a => {
                self.sio_control = value;
                if value & (1 << 6) != 0 {
                    self.sio_rx.clear();
                    self.sio_phase = 0;
                }
            }
            0x1f80_104e => self.sio_baud = value,
            _ => {
                let [lo, hi] = value.to_le_bytes();
                self.write_memory8(physical, lo);
                self.write_memory8(physical.wrapping_add(1), hi);
            }
        }
    }

    fn read32(&mut self, address: u32) -> u32 {
        let physical = Self::physical(address);
        self.read_mmio32(physical)
            .unwrap_or_else(|| self.read_word_le(physical))
    }

    fn write32(&mut self, address: u32, value: u32) {
        let physical = Self::physical(address);
        if !self.write_mmio32(physical, value) {
            self.write_word_le(physical, value);
        }
    }
}

pub struct PlayStationMachine {
    cpu: MipsR3000,
    bus: Ps1Bus,
    audio: AudioBuffer,
    powered: bool,
}
impl PlayStationMachine {
    pub fn from_bios(bios: &[u8]) -> Result<Self, String> {
        let mut bus = Ps1Bus::new(bios)?;
        let mut cpu = MipsR3000::default();
        cpu.reset();
        let _ = bus.read8(cpu.pc);
        Ok(Self {
            cpu,
            bus,
            audio: AudioBuffer::new(AUDIO_RATE, 2),
            powered: true,
        })
    }

    fn clock_instruction(&mut self) -> u32 {
        self.cpu.set_irq_line(2, self.bus.irq_pending());
        let used = self.cpu.step(&mut self.bus);
        self.bus.tick(used);
        used
    }

    fn flush_audio(&mut self) {
        self.audio.begin_frame();
        let frames = (f64::from(AUDIO_RATE) / FRAME_RATE).round() as usize;
        for _ in 0..frames {
            self.audio.push_stereo(0.0, 0.0);
        }
    }
}

impl Machine for PlayStationMachine {
    fn platform(&self) -> PlatformId {
        PlatformId::PlayStation
    }

    fn reset(&mut self) {
        self.bus.reset();
        self.cpu.reset();
        self.powered = true;
        self.flush_audio();
    }

    fn run_frame(&mut self, input: &InputState) {
        self.bus.set_inputs(input);
        let target = self.bus.gpu.frame().wrapping_add(1);
        let deadline = self.cpu.cycles.saturating_add(700_000);
        while self.bus.gpu.frame() != target && self.cpu.cycles < deadline {
            if self.clock_instruction() == 0 {
                self.powered = false;
                break;
            }
        }
        if self.bus.gpu.frame() != target {
            self.powered = false;
        }
        self.flush_audio();
    }

    fn frame_rate(&self) -> f64 {
        FRAME_RATE
    }
    fn video(&self) -> &VideoBuffer {
        self.bus.gpu.video()
    }
    fn audio(&self) -> &AudioBuffer {
        &self.audio
    }

    fn save_state(&self) -> Result<Vec<u8>, String> {
        let mut out = StateWriter::new(PlatformId::PlayStation, STATE_VERSION);
        self.cpu.save(&mut out);
        self.bus.save(&mut out);
        out.u8(self.powered as u8);
        Ok(out.finish())
    }

    fn load_state(&mut self, bytes: &[u8]) -> Result<(), String> {
        let mut input = StateReader::new(bytes, PlatformId::PlayStation, STATE_VERSION)?;
        self.cpu.load_state(&mut input)?;
        self.bus.load(&mut input)?;
        self.powered = input.u8()? != 0;
        self.flush_audio();
        input.finish()
    }
    fn persistent_len(&self, kind: ResourceKind, slot: u32) -> usize {
        if kind == ResourceKind::Storage && slot == 0 {
            self.bus.memory_card.len()
        } else {
            0
        }
    }

    fn read_persistent(&self, kind: ResourceKind, slot: u32, out: &mut [u8]) -> Result<(), String> {
        if kind != ResourceKind::Storage || slot != 0 {
            return Err("PlayStation memory card is Storage slot 0".into());
        }
        if out.len() != self.bus.memory_card.len() {
            return Err("PlayStation memory-card output length mismatch".into());
        }
        out.copy_from_slice(&self.bus.memory_card);
        Ok(())
    }

    fn write_persistent(
        &mut self,
        kind: ResourceKind,
        slot: u32,
        data: &[u8],
    ) -> Result<(), String> {
        if kind != ResourceKind::Storage || slot != 0 {
            return Err("PlayStation memory card is Storage slot 0".into());
        }
        if data.len() != self.bus.memory_card.len() {
            return Err("PlayStation memory-card input length mismatch".into());
        }
        self.bus.memory_card.copy_from_slice(data);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn synthetic_bios() -> Vec<u8> {
        let mut bios = vec![0; BIOS_SIZE];
        let words: [u32; 15] = [
            0x3c08_1f80,
            0x3508_1814,
            0x3c09_0300,
            0xad09_0000,
            0x2508_fffc,
            0x3c09_0200,
            0x3529_00ff,
            0xad09_0000,
            0x2409_0000,
            0xad09_0000,
            0x3c09_00f0,
            0x3529_0140,
            0xad09_0000,
            0x0bf0_000d,
            0x0000_0000,
        ];
        for (index, word) in words.into_iter().enumerate() {
            bios[index * 4..index * 4 + 4].copy_from_slice(&word.to_le_bytes());
        }
        bios
    }

    #[test]
    fn synthetic_bios_boots_mips_and_renders_gpu_frame() {
        let bios = synthetic_bios();
        let mut machine = PlayStationMachine::from_bios(&bios).unwrap();
        machine.run_frame(&InputState::default());
        assert!(machine.powered);
        assert_eq!(machine.video().width(), 320);
        assert_eq!(machine.video().height(), 240);
        assert!(machine.video().pixels()[0] > 200);
        assert_eq!(machine.audio().sample_rate(), AUDIO_RATE);
        assert_eq!(machine.bus.gpu.frame(), 1);
    }

    #[test]
    fn linked_list_dma_feeds_gpu_commands() {
        let bios = synthetic_bios();
        let mut bus = Ps1Bus::new(&bios).unwrap();
        bus.write_word_le(0x100, (3 << 24) | 0x0080_0000);
        bus.write_word_le(0x104, 0x0200_ff00);
        bus.write_word_le(0x108, (4 << 16) | 8);
        bus.write_word_le(0x10c, (2 << 16) | 16);
        bus.dma[2].base = 0x100;
        bus.dma[2].control = 1 | (2 << 9) | (1 << 24);
        bus.run_dma(2);
        assert_ne!(bus.gpu.vram_pixel(8, 4), 0);
        assert_eq!(bus.dma[2].control & (1 << 24), 0);
    }

    #[test]
    fn digital_pad_serial_protocol_reports_active_low_buttons() {
        let bios = synthetic_bios();
        let mut bus = Ps1Bus::new(&bios).unwrap();
        let mut input = InputState::default();
        input.buttons[0] = START | FACE_SOUTH;
        bus.set_inputs(&input);
        for byte in [0x01, 0x42, 0x00, 0x00, 0x00] {
            bus.sio_write(byte);
        }
        let responses: Vec<u8> = bus.sio_rx.drain(..).collect();
        assert_eq!(&responses[..3], &[0xff, 0x41, 0x5a]);
        let buttons = u16::from_le_bytes([responses[3], responses[4]]);
        assert_eq!(buttons & (1 << 3), 0);
        assert_eq!(buttons & (1 << 14), 0);
    }

    #[test]
    fn state_and_memory_card_round_trip() {
        let bios = synthetic_bios();
        let mut machine = PlayStationMachine::from_bios(&bios).unwrap();
        let card = vec![0x5a; MEMORY_CARD_SIZE];
        machine
            .write_persistent(ResourceKind::Storage, 0, &card)
            .unwrap();
        machine.bus.ram[0x1234] = 0xa5;
        machine.bus.gpu.gp0(0x0200_00ff);
        machine.bus.gpu.gp0(0);
        machine.bus.gpu.gp0((2 << 16) | 2);
        let saved = machine.save_state().unwrap();
        machine.bus.ram[0x1234] = 0;
        machine.load_state(&saved).unwrap();
        assert_eq!(machine.bus.ram[0x1234], 0xa5);
        let mut restored = vec![0; MEMORY_CARD_SIZE];
        machine
            .read_persistent(ResourceKind::Storage, 0, &mut restored)
            .unwrap();
        assert_eq!(restored, card);
        assert_ne!(machine.bus.gpu.vram_pixel(0, 0), 0);
    }
}
