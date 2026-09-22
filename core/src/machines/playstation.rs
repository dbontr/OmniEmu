use std::collections::VecDeque;

use crate::cpu_mips_r3000::{MipsBus, MipsR3000};
use crate::input::{
    AXIS_LEFT_X, AXIS_LEFT_Y, AXIS_RIGHT_X, AXIS_RIGHT_Y, DOWN, FACE_EAST, FACE_NORTH, FACE_SOUTH,
    FACE_WEST, L1, L2, L3, LEFT, R1, R2, R3, RIGHT, SELECT, START, UP,
};
use crate::kernel::{AudioBuffer, InputState, VideoBuffer};
use crate::machine::Machine;
use crate::platform::PlatformId;
use crate::resources::{ResourceBlob, ResourceKind};
use crate::state::{StateReader, StateWriter};

use super::ps1_cdrom::Ps1CdRom;
use super::ps1_gpu::Ps1Gpu;
use super::ps1_gte::Ps1Gte;
use super::ps1_mdec::Ps1Mdec;
use super::ps1_spu::Ps1Spu;

const RAM_SIZE: usize = 2 * 1024 * 1024;
const BIOS_SIZE: usize = 512 * 1024;
const PS_EXE_HEADER_SIZE: usize = 0x800;
const PS_EXE_DIRECT_STACK: u32 = 0x801f_fff0;
const SCRATCH_SIZE: usize = 1024;
const MEMORY_CARD_SIZE: usize = 128 * 1024;
const DEFAULT_MEMORY_CONTROL: [u32; 9] = [
    0x1f00_0000,
    0x1f80_2000,
    0x0013_243f,
    0x0000_3022,
    0x0013_243f,
    0x2009_31e1,
    0x0002_0843,
    0x0007_0777,
    0x0003_1125,
];
const DEFAULT_RAM_SIZE: u32 = 0x0000_0b88;
const AUDIO_RATE: u32 = 44_100;
const CPU_CLOCK_HZ: u64 = 33_868_800;
const STATE_VERSION: u32 = 23;

#[derive(Clone, Copy, Default)]
struct DmaChannel {
    base: u32,
    block: u32,
    control: u32,
}

#[derive(Clone, Copy, Default)]
struct GpuDmaProgress {
    remaining_words: u32,
    linked_remaining: u8,
    linked_next: u32,
    linked_header: u32,
    linked_active: bool,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum GpuDmaResult {
    Complete,
    Stalled,
    Fault,
}

impl GpuDmaProgress {
    fn save(self, out: &mut StateWriter) {
        out.u32(self.remaining_words);
        out.u8(self.linked_remaining);
        out.u32(self.linked_next);
        out.u32(self.linked_header);
        out.u8(self.linked_active as u8);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.remaining_words = input.u32()?;
        self.linked_remaining = input.u8()?;
        self.linked_next = input.u32()? & 0x00ff_ffff;
        self.linked_header = input.u32()? & 0x00ff_fffc;
        self.linked_active = input.u8()? != 0;
        Ok(())
    }
}

#[derive(Clone, Copy)]
struct Ps1Timer {
    counter: u16,
    mode: u16,
    target: u16,
    clock_phase: u64,
    hold_clocks: u8,
    irq_fired: bool,
    sync_seen: bool,
    pulse_pending: bool,
}

impl Default for Ps1Timer {
    fn default() -> Self {
        Self {
            counter: 0,
            mode: 1 << 10,
            target: 0,
            clock_phase: 0,
            hold_clocks: 0,
            irq_fired: false,
            sync_seen: false,
            pulse_pending: false,
        }
    }
}

#[derive(Clone, Copy)]
struct TimerSyncSignals {
    was_hblank: bool,
    is_hblank: bool,
    hblank_edge: bool,
    was_vblank: bool,
    is_vblank: bool,
    vblank_edge: bool,
}

impl Ps1Timer {
    fn write_counter(&mut self, value: u16) {
        self.counter = value;
        self.hold_clocks = 2;
    }

    fn write_mode(&mut self, value: u16) {
        self.mode = (value & 0x03ff) | (1 << 10);
        self.counter = 0;
        self.clock_phase = 0;
        self.hold_clocks = 2;
        self.irq_fired = false;
        self.sync_seen = false;
        self.pulse_pending = false;
    }

    fn read_mode(&mut self) -> u16 {
        let value = self.mode & 0x1fff;
        self.mode &= !((1 << 11) | (1 << 12));
        value
    }

    fn write_target(&mut self, value: u16) {
        self.target = value;
    }

    fn reset_on_sync_edge(&mut self) {
        self.counter = 0;
        self.hold_clocks = 0;
    }

    fn fire_irq(&mut self) -> bool {
        if self.mode & (1 << 6) == 0 && self.irq_fired {
            return false;
        }
        self.irq_fired = true;
        if self.mode & (1 << 7) != 0 {
            let was_high = self.mode & (1 << 10) != 0;
            self.mode ^= 1 << 10;
            was_high && self.mode & (1 << 10) == 0
        } else {
            self.mode &= !(1 << 10);
            self.pulse_pending = true;
            true
        }
    }

    fn tick(&mut self, ticks: u32) -> bool {
        let mut irq = false;
        for _ in 0..ticks {
            if self.pulse_pending {
                self.mode |= 1 << 10;
                self.pulse_pending = false;
            }
            if self.hold_clocks != 0 {
                self.hold_clocks -= 1;
                continue;
            }

            let next = self.counter.wrapping_add(1);
            self.counter = next;
            let reached_target = next == self.target;
            let reached_ffff = next == 0xffff;
            if reached_target {
                self.mode |= 1 << 11;
            }
            if reached_ffff {
                self.mode |= 1 << 12;
            }

            let request_irq = (reached_target && self.mode & (1 << 4) != 0)
                || (reached_ffff && self.mode & (1 << 5) != 0);
            if request_irq {
                irq |= self.fire_irq();
            }
            if reached_target && self.mode & (1 << 3) != 0 {
                self.counter = 0;
                self.hold_clocks = 1;
            }
        }
        irq
    }

    fn save(&self, out: &mut StateWriter) {
        out.u16(self.counter);
        out.u16(self.mode);
        out.u16(self.target);
        out.u64(self.clock_phase);
        out.u8(self.hold_clocks);
        out.u8(self.irq_fired as u8);
        out.u8(self.sync_seen as u8);
        out.u8(self.pulse_pending as u8);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.counter = input.u16()?;
        self.mode = input.u16()? & 0x1fff;
        self.target = input.u16()?;
        self.clock_phase = input.u64()?;
        self.hold_clocks = input.u8()?.min(2);
        self.irq_fired = input.u8()? != 0;
        self.sync_seen = input.u8()? != 0;
        self.pulse_pending = input.u8()? != 0;
        Ok(())
    }
}

struct Ps1Bus {
    ram: Vec<u8>,
    scratch: [u8; SCRATCH_SIZE],
    bios: Vec<u8>,
    gpu: Ps1Gpu,
    gte: Ps1Gte,
    mdec: Ps1Mdec,
    cdrom: Ps1CdRom,
    irq_status: u32,
    irq_mask: u32,
    dma: [DmaChannel; 7],
    dpcr: u32,
    dicr: u32,
    mdec_dma_remaining: [u32; 2],
    gpu_dma: GpuDmaProgress,
    timers: [Ps1Timer; 3],
    spu: Ps1Spu,
    pad_buttons: [u16; 2],
    pad_axes: [[u8; 4]; 2],
    pad_analog: [bool; 2],
    pad_config: [bool; 2],
    pad_mode_locked: [bool; 2],
    pad_rumble_map: [[u8; 6]; 2],
    pad_rumble_small: [bool; 2],
    pad_rumble_large: [u8; 2],
    sio_rx: VecDeque<u8>,
    sio_mode: u16,
    sio_control: u16,
    sio_baud: u16,
    sio_phase: u16,
    sio_device: u8,
    sio_command: u8,
    sio_parameter: u8,
    card_flag: u8,
    card_address: u16,
    card_checksum: u8,
    card_write_result: u8,
    card_write_buffer: [u8; 128],
    memory_card: Vec<u8>,
    memory_control: [u32; 9],
    ram_size: u32,
    cache_control: u32,
    cache_isolated: bool,
    icache_words: [u32; 1024],
    icache_tags: [u32; 256],
    icache_valid: [u8; 256],
}

impl Ps1Bus {
    fn new_with_disc(bios: &[u8], disc: Option<ResourceBlob>) -> Result<Self, String> {
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
            gte: Ps1Gte::new(),
            mdec: Ps1Mdec::new(),
            cdrom: Ps1CdRom::new(disc)?,
            irq_status: 0,
            irq_mask: 0,
            dma: [DmaChannel::default(); 7],
            dpcr: 0x0765_4321,
            dicr: 0,
            mdec_dma_remaining: [0; 2],
            gpu_dma: GpuDmaProgress::default(),
            timers: [Ps1Timer::default(); 3],
            spu: Ps1Spu::new(),
            pad_buttons: [0xffff; 2],
            pad_axes: [[0x80; 4]; 2],
            pad_analog: [false; 2],
            pad_config: [false; 2],
            pad_mode_locked: [false; 2],
            pad_rumble_map: [[0xff; 6]; 2],
            pad_rumble_small: [false; 2],
            pad_rumble_large: [0; 2],
            sio_rx: VecDeque::new(),
            sio_mode: 0,
            sio_control: 0,
            sio_baud: 0,
            sio_phase: 0,
            sio_device: 0,
            sio_command: 0,
            sio_parameter: 0,
            card_flag: 0x08,
            card_address: 0,
            card_checksum: 0,
            card_write_result: 0x47,
            card_write_buffer: [0; 128],
            memory_card: Self::formatted_memory_card(),
            memory_control: DEFAULT_MEMORY_CONTROL,
            ram_size: DEFAULT_RAM_SIZE,
            cache_control: 0,
            cache_isolated: false,
            icache_words: [0; 1024],
            icache_tags: [0; 256],
            icache_valid: [0; 256],
        })
    }

    fn reset(&mut self) {
        self.ram.fill(0);
        self.scratch.fill(0);
        self.gpu.reset();
        self.gte.reset();
        self.mdec.reset();
        self.cdrom.reset();
        self.irq_status = 0;
        self.irq_mask = 0;
        self.dma = [DmaChannel::default(); 7];
        self.dpcr = 0x0765_4321;
        self.dicr = 0;
        self.mdec_dma_remaining = [0; 2];
        self.gpu_dma = GpuDmaProgress::default();
        self.timers = [Ps1Timer::default(); 3];
        self.spu.reset();
        self.pad_analog = [false; 2];
        self.pad_config = [false; 2];
        self.pad_mode_locked = [false; 2];
        self.pad_rumble_map = [[0xff; 6]; 2];
        self.pad_rumble_small = [false; 2];
        self.pad_rumble_large = [0; 2];
        self.sio_rx.clear();
        self.sio_mode = 0;
        self.sio_control = 0;
        self.sio_baud = 0;
        self.reset_sio_transaction();
        self.memory_control = DEFAULT_MEMORY_CONTROL;
        self.ram_size = DEFAULT_RAM_SIZE;
        self.cache_control = 0;
        self.cache_isolated = false;
        self.icache_words.fill(0);
        self.icache_tags.fill(0);
        self.icache_valid.fill(0);
    }

    fn formatted_memory_card() -> Vec<u8> {
        fn checksum_frame(frame: &mut [u8]) {
            frame[127] = frame[..127]
                .iter()
                .fold(0u8, |checksum, byte| checksum ^ byte);
        }

        let mut card = vec![0u8; MEMORY_CARD_SIZE];
        card[0] = b'M';
        card[1] = b'C';
        checksum_frame(&mut card[..128]);
        for sector in 1..=15usize {
            let frame = &mut card[sector * 128..(sector + 1) * 128];
            frame[..4].copy_from_slice(&0x0000_00a0u32.to_le_bytes());
            frame[8..10].copy_from_slice(&0xffffu16.to_le_bytes());
            checksum_frame(frame);
        }
        for sector in 16..=35usize {
            let frame = &mut card[sector * 128..(sector + 1) * 128];
            frame[..4].fill(0xff);
            checksum_frame(frame);
        }
        for sector in 36..=62usize {
            card[sector * 128..(sector + 1) * 128].fill(0xff);
        }
        let header = card[..128].to_vec();
        card[63 * 128..64 * 128].copy_from_slice(&header);
        card
    }

    fn reset_sio_transaction(&mut self) {
        self.sio_phase = 0;
        self.sio_device = 0;
        self.sio_command = 0;
        self.sio_parameter = 0;
        self.card_address = 0;
        self.card_checksum = 0;
        self.card_write_result = 0x47;
        self.card_write_buffer.fill(0);
    }

    fn sio_status(&self) -> u16 {
        let mut status = 0x0005u16;
        if !self.sio_rx.is_empty() {
            status |= 1 << 1;
        }
        if self.sio_phase != 0 {
            status |= 1 << 7;
        }
        if self.irq_status & (1 << 7) != 0 {
            status |= 1 << 9;
        }
        status
    }

    fn card_sector_offset(&self) -> Option<usize> {
        (self.card_address < 1024).then_some(usize::from(self.card_address) * 128)
    }

    fn card_read_checksum(&self) -> u8 {
        let mut checksum = (self.card_address >> 8) as u8 ^ self.card_address as u8;
        if let Some(offset) = self.card_sector_offset() {
            for byte in &self.memory_card[offset..offset + 128] {
                checksum ^= *byte;
            }
        }
        checksum
    }

    fn set_inputs(&mut self, input: &InputState) {
        for player in 0..2 {
            let mut state = 0xffffu16;
            let buttons = input.buttons[player];
            let map = [
                (SELECT, 0),
                (L3, 1),
                (R3, 2),
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
            let axis = |value: i16| -> u8 { ((i32::from(value) + 32_768) >> 8) as u8 };
            self.pad_axes[player] = [
                axis(input.axes[player][AXIS_RIGHT_X]),
                axis(input.axes[player][AXIS_RIGHT_Y]),
                axis(input.axes[player][AXIS_LEFT_X]),
                axis(input.axes[player][AXIS_LEFT_Y]),
            ];
        }
    }

    fn physical(address: u32) -> u32 {
        address & 0x1fff_ffff
    }

    fn partial_store_latches_word(physical: u32) -> bool {
        if matches!(
            physical,
            0x1f80_1000..=0x1f80_1023
                | 0x1f80_1060..=0x1f80_1063
                | 0x1f80_1070..=0x1f80_1077
                | 0x1f80_10f0..=0x1f80_10f7
                | 0x1f80_1100..=0x1f80_112b
                | 0x1f80_1810..=0x1f80_1817
                | 0x1f80_1820..=0x1f80_1827
        ) {
            return true;
        }
        if (0x1f80_1080..=0x1f80_10ef).contains(&physical) {
            return matches!(physical & 0x0f, 0..=3 | 8..=15);
        }
        false
    }

    fn icache_location(address: u32) -> (usize, usize, u32) {
        let physical = Self::physical(address);
        let line = ((physical >> 4) & 0xff) as usize;
        let word = ((physical >> 2) & 3) as usize;
        (line, line * 4 + word, physical & 0xffff_f000)
    }

    fn icache_enabled(&self) -> bool {
        self.cache_control & (1 << 11) != 0
    }

    fn isolated_tag_mode(&self) -> bool {
        self.icache_enabled() && self.cache_control & (1 << 2) != 0
    }

    fn isolated_cache_read32(&self, address: u32) -> u32 {
        let (line, word_index, tag) = Self::icache_location(address);
        if self.isolated_tag_mode() {
            let matches = u32::from(self.icache_tags[line] == tag) << 4;
            (self.icache_words[word_index] & !0x1f)
                | u32::from(self.icache_valid[line] & 0x0f)
                | matches
        } else if self.icache_enabled() {
            self.icache_words[word_index]
        } else {
            0
        }
    }

    fn isolated_cache_write32(&mut self, address: u32, value: u32) {
        let (line, word_index, tag) = Self::icache_location(address);
        if self.isolated_tag_mode() {
            self.icache_tags[line] = tag;
            self.icache_valid[line] = value as u8 & 0x0f;
        } else if self.icache_enabled() {
            self.icache_words[word_index] = value;
        }
    }

    fn ram_index(physical: u32) -> Option<usize> {
        if physical < 0x0100_0000 {
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
            0x1f80_1044 => self.sio_status() as u8,
            0x1f80_1045 => (self.sio_status() >> 8) as u8,
            0x1f80_1800..=0x1f80_1803 => self.cdrom.read_register((physical - 0x1f80_1800) as u8),
            address if (0x1f80_1c00..=0x1f80_1e7f).contains(&address) => self.spu.read8(address),
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
            0x1f80_1800..=0x1f80_1803 => {
                self.cdrom
                    .write_register((physical - 0x1f80_1800) as u8, value);
                self.service_dma();
            }
            address if (0x1f80_1c00..=0x1f80_1e7f).contains(&address) => {
                self.spu.write8(address, value)
            }
            _ => {}
        }
    }
    fn update_pad_rumble(&mut self, player: usize, index: usize, value: u8) {
        match self.pad_rumble_map[player][index] {
            0x00 => self.pad_rumble_small[player] = value & 1 != 0,
            0x01 => self.pad_rumble_large[player] = value,
            _ => {}
        }
    }

    fn finish_pad_normal_command(&mut self, player: usize) {
        if self.sio_command == 0x43 && self.sio_parameter == 1 {
            self.pad_config[player] = true;
        }
        self.reset_sio_transaction();
    }

    fn sio_pad_normal_exchange(&mut self, player: usize, value: u8) -> (u8, bool) {
        let analog = self.pad_analog[player];
        match self.sio_phase {
            2 => {
                self.sio_phase = 3;
                (0x5a, true)
            }
            3 => {
                if self.sio_command == 0x43 {
                    self.sio_parameter = value;
                } else {
                    self.update_pad_rumble(player, 0, value);
                }
                self.sio_phase = 4;
                (self.pad_buttons[player] as u8, true)
            }
            4 => {
                if self.sio_command == 0x42 {
                    self.update_pad_rumble(player, 1, value);
                }
                let response = (self.pad_buttons[player] >> 8) as u8;
                if analog {
                    self.sio_phase = 5;
                    (response, true)
                } else {
                    self.finish_pad_normal_command(player);
                    (response, false)
                }
            }
            5..=8 => {
                if self.sio_command == 0x42 {
                    self.update_pad_rumble(player, usize::from(self.sio_phase - 3), value);
                }
                let response = self.pad_axes[player][usize::from(self.sio_phase - 5)];
                if self.sio_phase == 8 {
                    self.finish_pad_normal_command(player);
                    (response, false)
                } else {
                    self.sio_phase += 1;
                    (response, true)
                }
            }
            _ => {
                self.reset_sio_transaction();
                (0xff, false)
            }
        }
    }

    fn pad_config_reply(&mut self, player: usize, value: u8) -> u8 {
        let phase = self.sio_phase;
        let index = usize::from(phase.saturating_sub(3));
        match self.sio_command {
            0x42 => {
                if index < 6 {
                    self.update_pad_rumble(player, index, value);
                }
                match phase {
                    3 => self.pad_buttons[player] as u8,
                    4 => (self.pad_buttons[player] >> 8) as u8,
                    5..=8 => self.pad_axes[player][usize::from(phase - 5)],
                    _ => 0,
                }
            }
            0x43 => {
                if phase == 3 {
                    self.sio_parameter = value;
                }
                0
            }
            0x44 => {
                if phase == 3 && value <= 1 {
                    self.pad_analog[player] = value != 0;
                } else if phase == 4 {
                    self.pad_mode_locked[player] = value & 3 == 3;
                }
                0
            }
            0x45 => match phase {
                3 => 0x01,
                4 => 0x02,
                5 => u8::from(self.pad_analog[player]),
                6 => 0x02,
                7 => 0x01,
                _ => 0x00,
            },
            0x46 => {
                if phase == 3 {
                    self.sio_parameter = value;
                    0
                } else {
                    let values = match self.sio_parameter {
                        0 => [0, 0, 1, 2, 0, 10],
                        1 => [0, 0, 1, 1, 1, 20],
                        _ => [0; 6],
                    };
                    values[index]
                }
            }
            0x47 => [0, 0, 2, 0, 1, 0][index],
            0x48 => {
                if phase == 3 {
                    self.sio_parameter = value;
                }
                if phase == 7 && self.sio_parameter <= 1 {
                    1
                } else {
                    0
                }
            }
            0x4c => {
                if phase == 3 {
                    self.sio_parameter = value;
                }
                if phase == 6 {
                    match self.sio_parameter {
                        0 => 4,
                        1 => 7,
                        _ => 0,
                    }
                } else {
                    0
                }
            }
            0x4d => {
                let response = self.pad_rumble_map[player][index];
                self.pad_rumble_map[player][index] = value;
                response
            }
            _ => 0,
        }
    }

    fn sio_pad_config_exchange(&mut self, player: usize, value: u8) -> (u8, bool) {
        match self.sio_phase {
            2 => {
                self.sio_phase = 3;
                (0x5a, true)
            }
            3..=8 => {
                let response = self.pad_config_reply(player, value);
                if self.sio_phase == 8 {
                    if self.sio_command == 0x43 && self.sio_parameter == 0 {
                        self.pad_config[player] = false;
                    }
                    self.reset_sio_transaction();
                    (response, false)
                } else {
                    self.sio_phase += 1;
                    (response, true)
                }
            }
            _ => {
                self.reset_sio_transaction();
                (0xff, false)
            }
        }
    }

    fn ram_window_bus_error(&self, address: u32, size: u8) -> bool {
        let physical = Self::physical(address);
        if physical >= 0x0100_0000 {
            return false;
        }
        let last = physical.saturating_add(u32::from(size.saturating_sub(1)));
        last >= self.ram_window_size()
    }

    fn instruction_cache_hit(&self, address: u32) -> bool {
        if address >= 0xa000_0000 || !self.icache_enabled() {
            return false;
        }
        let (line, word_index, tag) = Self::icache_location(address);
        let bit = 1u8 << (word_index & 3);
        self.icache_tags[line] == tag && self.icache_valid[line] & bit != 0
    }

    fn fetch_instruction32(&mut self, address: u32) -> u32 {
        let cached_region = address < 0xa000_0000;
        if !cached_region || !self.icache_enabled() {
            return self.read_word_le(Self::physical(address));
        }

        let physical = Self::physical(address);
        let (line, word_index, tag) = Self::icache_location(address);
        let word = word_index & 3;
        let bit = 1u8 << word;
        let tag_matches = self.icache_tags[line] == tag;
        if tag_matches && self.icache_valid[line] & bit != 0 {
            return self.icache_words[word_index];
        }

        let line_base = physical & !0x0f;
        if !tag_matches {
            self.icache_tags[line] = tag;
            self.icache_valid[line] = 0;
            let end_word = if word == 0 && ((self.cache_control >> 8) & 3) == 0 {
                1
            } else {
                3
            };
            for fill_word in word..=end_word {
                self.icache_words[line * 4 + fill_word] =
                    self.read_word_le(line_base + (fill_word as u32) * 4);
                self.icache_valid[line] |= 1 << fill_word;
            }
        } else {
            for fill_word in 0..=3usize {
                self.icache_words[line * 4 + fill_word] =
                    self.read_word_le(line_base + (fill_word as u32) * 4);
            }
            self.icache_valid[line] = 0x0f;
        }
        self.icache_words[word_index]
    }

    fn sio_pad_exchange(&mut self, value: u8) -> (u8, bool) {
        let player = usize::from(self.sio_control & (1 << 13) != 0);
        if self.sio_phase == 1 {
            let config = self.pad_config[player];
            let accepted = if config {
                (0x40..=0x4f).contains(&value)
            } else {
                matches!(value, 0x42 | 0x43)
            };
            if !accepted {
                self.reset_sio_transaction();
                return (0xff, false);
            }
            self.sio_command = value;
            self.sio_phase = 2;
            let id = if config {
                0xf3
            } else if self.pad_analog[player] {
                0x73
            } else {
                0x41
            };
            return (id, true);
        }
        if self.pad_config[player] {
            self.sio_pad_config_exchange(player, value)
        } else {
            self.sio_pad_normal_exchange(player, value)
        }
    }

    fn sio_card_read_exchange(&mut self, value: u8) -> (u8, bool) {
        match self.sio_phase {
            2 => {
                self.sio_phase = 3;
                (0x5a, true)
            }
            3 => {
                self.sio_phase = 4;
                (0x5d, true)
            }
            4 => {
                self.card_address = u16::from(value) << 8;
                self.sio_phase = 5;
                (0x00, true)
            }
            5 => {
                let response = (self.card_address >> 8) as u8;
                self.card_address |= u16::from(value);
                self.sio_phase = 6;
                (response, true)
            }
            6 => {
                self.sio_phase = 7;
                (0x5c, true)
            }
            7 => {
                self.sio_phase = 8;
                (0x5d, true)
            }
            8 => {
                self.sio_phase = 9;
                let response = self
                    .card_sector_offset()
                    .map_or(0xff, |_| (self.card_address >> 8) as u8);
                (response, true)
            }
            9 => {
                let valid = self.card_sector_offset().is_some();
                let response = if valid { self.card_address as u8 } else { 0xff };
                if valid {
                    self.sio_phase = 10;
                    (response, true)
                } else {
                    self.reset_sio_transaction();
                    (response, false)
                }
            }
            10..=137 => {
                let index = usize::from(self.sio_phase - 10);
                let offset = self.card_sector_offset().expect("validated card sector") + index;
                let response = self.memory_card[offset];
                self.sio_phase += 1;
                (response, true)
            }
            138 => {
                self.sio_phase = 139;
                (self.card_read_checksum(), true)
            }
            139 => {
                self.reset_sio_transaction();
                (0x47, false)
            }
            _ => {
                self.reset_sio_transaction();
                (0xff, false)
            }
        }
    }

    fn sio_card_write_exchange(&mut self, value: u8) -> (u8, bool) {
        match self.sio_phase {
            2 => {
                self.sio_phase = 3;
                (0x5a, true)
            }
            3 => {
                self.sio_phase = 4;
                (0x5d, true)
            }
            4 => {
                self.card_address = u16::from(value) << 8;
                self.sio_phase = 5;
                (0x00, true)
            }
            5 => {
                let response = (self.card_address >> 8) as u8;
                self.card_address |= u16::from(value);
                self.card_checksum = (self.card_address >> 8) as u8 ^ self.card_address as u8;
                self.card_write_result = if self.card_sector_offset().is_some() {
                    0x47
                } else {
                    0xff
                };
                self.sio_phase = 6;
                (response, true)
            }
            6..=133 => {
                let index = usize::from(self.sio_phase - 6);
                let response = if index == 0 {
                    self.card_address as u8
                } else {
                    self.card_write_buffer[index - 1]
                };
                self.card_write_buffer[index] = value;
                self.card_checksum ^= value;
                self.sio_phase += 1;
                (response, true)
            }
            134 => {
                let response = self.card_write_buffer[127];
                if self.card_write_result != 0xff {
                    if value == self.card_checksum {
                        if let Some(offset) = self.card_sector_offset() {
                            self.memory_card[offset..offset + 128]
                                .copy_from_slice(&self.card_write_buffer);
                            self.card_flag &= !0x08;
                        }
                    } else {
                        self.card_write_result = 0x4e;
                    }
                }
                self.sio_phase = 135;
                (response, true)
            }
            135 => {
                self.sio_phase = 136;
                (0x5c, true)
            }
            136 => {
                self.sio_phase = 137;
                (0x5d, true)
            }
            137 => {
                let response = self.card_write_result;
                self.reset_sio_transaction();
                (response, false)
            }
            _ => {
                self.reset_sio_transaction();
                (0xff, false)
            }
        }
    }

    fn sio_card_id_exchange(&mut self) -> (u8, bool) {
        let (response, more) = match self.sio_phase {
            2 => (0x5a, true),
            3 => (0x5d, true),
            4 => (0x5c, true),
            5 => (0x5d, true),
            6 => (0x04, true),
            7 | 8 => (0x00, true),
            9 => (0x80, false),
            _ => (0xff, false),
        };
        if more {
            self.sio_phase += 1;
        } else {
            self.reset_sio_transaction();
        }
        (response, more)
    }

    fn sio_card_exchange(&mut self, value: u8) -> (u8, bool) {
        if self.sio_phase == 1 {
            let response = self.card_flag;
            self.sio_command = value;
            if matches!(value, 0x52 | 0x57 | 0x53) {
                self.sio_phase = 2;
                return (response, true);
            }
            self.reset_sio_transaction();
            return (response, false);
        }
        match self.sio_command {
            0x52 => self.sio_card_read_exchange(value),
            0x57 => self.sio_card_write_exchange(value),
            0x53 => self.sio_card_id_exchange(),
            _ => {
                self.reset_sio_transaction();
                (0xff, false)
            }
        }
    }

    fn sio_write(&mut self, value: u8) {
        let (response, acknowledge) = if self.sio_phase == 0 {
            match value {
                0x01 => {
                    self.sio_device = 0x01;
                    self.sio_phase = 1;
                    (0xff, true)
                }
                0x81 if self.sio_control & (1 << 13) == 0 => {
                    self.sio_device = 0x81;
                    self.sio_phase = 1;
                    (0xff, true)
                }
                _ => (0xff, false),
            }
        } else {
            match self.sio_device {
                0x01 => self.sio_pad_exchange(value),
                0x81 => self.sio_card_exchange(value),
                _ => {
                    self.reset_sio_transaction();
                    (0xff, false)
                }
            }
        };
        if self.sio_rx.len() >= 8 {
            self.sio_rx.pop_back();
        }
        self.sio_rx.push_back(response);
        if acknowledge {
            self.irq_status |= 1 << 7;
        }
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

    fn dma_channel_enabled(&self, channel: usize) -> bool {
        self.dpcr & (1 << (channel * 4 + 3)) != 0
    }

    fn dma_channel_ready(&mut self, channel: usize) -> bool {
        if channel >= self.dma.len()
            || self.dma[channel].control & (1 << 24) == 0
            || !self.dma_channel_enabled(channel)
        {
            return false;
        }
        match channel {
            0 => self.mdec.can_dma_in(),
            1 => self.mdec.can_dma_out(),
            2 => {
                let control = self.dma[2].control;
                let direction_to_gpu = control & 1 != 0;
                let sync = (control >> 9) & 3;
                let transfer_in_progress = if sync == 2 {
                    self.gpu_dma.linked_active
                } else {
                    self.gpu_dma.remaining_words != 0
                };
                if transfer_in_progress {
                    if direction_to_gpu {
                        self.gpu.can_accept_gp0_word()
                    } else {
                        self.gpu.can_read_vram_word()
                    }
                } else {
                    control & (1 << 28) != 0 || self.gpu.dma_request()
                }
            }
            3 => self.cdrom.dma_words_available() >= Self::dma_count16(self.dma[3].block) as usize,
            5 => false,
            6 => self.dma[6].control & (1 << 28) != 0,
            _ => true,
        }
    }

    fn update_dma_master_irq(&mut self) {
        let previous = self.dicr & (1 << 31) != 0;
        let active = self.dicr & (1 << 15) != 0
            || (self.dicr & (1 << 23) != 0 && self.dicr & 0x7f00_0000 != 0);
        if active {
            self.dicr |= 1 << 31;
        } else {
            self.dicr &= !(1 << 31);
        }
        if !previous && active {
            self.irq_status |= 1 << 3;
        }
    }

    fn write_dicr(&mut self, value: u32) {
        let flags = (self.dicr & 0x7f00_0000) & !(value & 0x7f00_0000);
        self.dicr = (value & 0x00ff_807f) | flags;
        self.update_dma_master_irq();
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
                if channel < 2 {
                    self.mdec_dma_remaining[channel] = 0;
                } else if channel == 2 {
                    self.gpu_dma = GpuDmaProgress::default();
                }
                self.dma[channel].control = if channel == 6 {
                    (value & ((1 << 24) | (1 << 28) | (1 << 30))) | 2
                } else {
                    value
                };
                if self.dma[channel].control & (1 << 24) != 0 {
                    self.service_dma();
                }
            }
            _ => {}
        }
        true
    }
    fn dma_priority(&self, channel: usize) -> u32 {
        (self.dpcr >> (channel * 4)) & 7
    }

    fn dma_count16(value: u32) -> u32 {
        let count = value & 0xffff;
        if count == 0 {
            0x1_0000
        } else {
            count
        }
    }

    fn ram_window_size(&self) -> u32 {
        match (self.ram_size >> 9) & 7 {
            0 => 0x0010_0000,
            1 => 0x0040_0000,
            2 => 0x0020_0000,
            3 => 0x0080_0000,
            4 => 0x0020_0000,
            5 => 0x0080_0000,
            6 => 0x0040_0000,
            _ => 0x0100_0000,
        }
    }

    fn dma_bus_error(&mut self) {
        self.dicr |= 1 << 15;
        self.update_dma_master_irq();
    }

    fn dma_read_ram_word(&mut self, address: u32) -> Option<u32> {
        let address = address & 0x00ff_fffc;
        if address >= self.ram_window_size() {
            self.dma_bus_error();
            None
        } else {
            Some(self.read_word_le(address))
        }
    }

    fn dma_write_ram_word(&mut self, address: u32, value: u32) -> bool {
        let address = address & 0x00ff_fffc;
        if address >= self.ram_window_size() {
            self.dma_bus_error();
            false
        } else {
            self.write_word_le(address, value);
            true
        }
    }

    fn dma_advance(address: u32, step: i32) -> u32 {
        address.wrapping_add_signed(step) & 0x00ff_fffc
    }

    fn service_dma(&mut self) {
        loop {
            let mut selected = None;
            for channel in 0..self.dma.len() {
                if !self.dma_channel_ready(channel) {
                    continue;
                }
                selected = match selected {
                    None => Some(channel),
                    Some(current) => {
                        let priority = self.dma_priority(channel);
                        let current_priority = self.dma_priority(current);
                        if priority < current_priority
                            || (priority == current_priority && channel > current)
                        {
                            Some(channel)
                        } else {
                            Some(current)
                        }
                    }
                };
            }
            let Some(channel) = selected else {
                break;
            };
            if !self.run_dma(channel) {
                break;
            }
        }
    }

    fn run_dma(&mut self, channel: usize) -> bool {
        if !self.dma_channel_ready(channel) {
            return false;
        }
        self.dma[channel].control &= !(1 << 28);
        let (completed, stalled) = match channel {
            0 => self.run_mdec_in_dma(),
            1 => self.run_mdec_out_dma(),
            2 => match self.run_gpu_dma() {
                GpuDmaResult::Complete => (true, false),
                GpuDmaResult::Stalled => (false, true),
                GpuDmaResult::Fault => (false, false),
            },
            3 => (self.run_cdrom_dma(), false),
            4 => (self.run_spu_dma(), false),
            6 => (self.run_otc_dma(), false),
            _ => return false,
        };
        if stalled {
            self.update_dma_master_irq();
            return false;
        }
        self.dma[channel].control &= !(1 << 24);
        if completed {
            if (self.dma[channel].control >> 9) & 3 == 1 {
                self.dma[channel].block &= 0xffff;
            }
            if self.dicr & (1 << 23) != 0 && self.dicr & (1 << (16 + channel)) != 0 {
                self.dicr |= 1 << (24 + channel);
            }
        }
        self.update_dma_master_irq();
        true
    }

    fn service_mdec_dma(&mut self) {
        self.service_dma();
    }

    fn run_mdec_in_dma(&mut self) -> (bool, bool) {
        if !self.mdec.can_dma_in() {
            return (false, true);
        }
        let sync = (self.dma[0].control >> 9) & 3;
        let block_size = Self::dma_count16(self.dma[0].block);
        let step = if self.dma[0].control & 2 != 0 {
            -4i32
        } else {
            4i32
        };
        let mut address = self.dma[0].base & 0x00ff_fffc;
        let mut remaining = self.mdec_dma_remaining[0];
        let mut budget = 4_000_000u32;
        if remaining == 0 {
            remaining = block_size;
        }
        loop {
            while remaining != 0 && budget != 0 && self.mdec.can_dma_in() {
                let Some(value) = self.dma_read_ram_word(address) else {
                    self.dma[0].base = address;
                    self.mdec_dma_remaining[0] = remaining;
                    return (false, false);
                };
                self.mdec.write_data(value);
                address = Self::dma_advance(address, step);
                remaining -= 1;
                budget -= 1;
            }
            self.dma[0].base = address;
            self.mdec_dma_remaining[0] = remaining;
            if remaining != 0 {
                return (false, true);
            }
            if sync != 1 {
                return (true, false);
            }
            let blocks = Self::dma_count16(self.dma[0].block >> 16);
            let remaining_blocks = blocks - 1;
            self.dma[0].block = (self.dma[0].block & 0xffff) | ((remaining_blocks & 0xffff) << 16);
            if remaining_blocks == 0 {
                return (true, false);
            }
            if budget == 0 || !self.mdec.can_dma_in() {
                return (false, true);
            }
            remaining = block_size;
            self.mdec_dma_remaining[0] = remaining;
        }
    }

    fn run_mdec_out_dma(&mut self) -> (bool, bool) {
        if !self.mdec.can_dma_out() {
            return (false, true);
        }
        let sync = (self.dma[1].control >> 9) & 3;
        let block_size = Self::dma_count16(self.dma[1].block);
        let step = if self.dma[1].control & 2 != 0 {
            -4i32
        } else {
            4i32
        };
        let mut address = self.dma[1].base & 0x00ff_fffc;
        let mut remaining = self.mdec_dma_remaining[1];
        let mut budget = 4_000_000u32;
        if remaining == 0 {
            remaining = block_size;
        }
        loop {
            while remaining != 0 && budget != 0 && self.mdec.can_dma_out() {
                let value = self.mdec.read_data();
                if !self.dma_write_ram_word(address, value) {
                    self.dma[1].base = address;
                    self.mdec_dma_remaining[1] = remaining;
                    return (false, false);
                }
                address = Self::dma_advance(address, step);
                remaining -= 1;
                budget -= 1;
            }
            self.dma[1].base = address;
            self.mdec_dma_remaining[1] = remaining;
            if remaining != 0 {
                return (false, true);
            }
            if sync != 1 {
                return (true, false);
            }
            let blocks = Self::dma_count16(self.dma[1].block >> 16);
            let remaining_blocks = blocks - 1;
            self.dma[1].block = (self.dma[1].block & 0xffff) | ((remaining_blocks & 0xffff) << 16);
            if remaining_blocks == 0 {
                return (true, false);
            }
            if budget == 0 || !self.mdec.can_dma_out() {
                return (false, true);
            }
            remaining = block_size;
            self.mdec_dma_remaining[1] = remaining;
        }
    }

    fn run_gpu_dma(&mut self) -> GpuDmaResult {
        let control = self.dma[2].control;
        let direction_to_gpu = control & 1 != 0;
        let sync = (control >> 9) & 3;
        if sync == 2 {
            return if direction_to_gpu {
                self.run_gpu_linked_list()
            } else {
                GpuDmaResult::Fault
            };
        }
        if self.gpu_dma.remaining_words == 0 {
            self.gpu_dma.remaining_words = Self::dma_count16(self.dma[2].block).min(4_000_000);
        }
        let step = if control & 2 != 0 { -4i32 } else { 4i32 };
        let mut address = self.dma[2].base & 0x00ff_fffc;
        while self.gpu_dma.remaining_words != 0 {
            if direction_to_gpu {
                if !self.gpu.can_accept_gp0_word() {
                    self.dma[2].base = address;
                    return GpuDmaResult::Stalled;
                }
                let Some(value) = self.dma_read_ram_word(address) else {
                    self.dma[2].base = address;
                    self.gpu_dma = GpuDmaProgress::default();
                    return GpuDmaResult::Fault;
                };
                if !self.gpu.gp0(value) {
                    self.dma[2].base = address;
                    return GpuDmaResult::Stalled;
                }
            } else {
                if !self.gpu.can_read_vram_word() {
                    self.dma[2].base = address;
                    return GpuDmaResult::Stalled;
                }
                let value = self.gpu.gpuread();
                if !self.dma_write_ram_word(address, value) {
                    self.dma[2].base = address;
                    self.gpu_dma = GpuDmaProgress::default();
                    return GpuDmaResult::Fault;
                }
            }
            address = Self::dma_advance(address, step);
            self.gpu_dma.remaining_words -= 1;
        }
        self.dma[2].base = address;
        if sync == 1 {
            let blocks = Self::dma_count16(self.dma[2].block >> 16);
            let remaining_blocks = blocks - 1;
            self.dma[2].block = (self.dma[2].block & 0xffff) | ((remaining_blocks & 0xffff) << 16);
            self.gpu_dma.remaining_words = 0;
            if remaining_blocks != 0 {
                return GpuDmaResult::Stalled;
            }
        }
        self.gpu_dma = GpuDmaProgress::default();
        GpuDmaResult::Complete
    }

    fn run_gpu_linked_list(&mut self) -> GpuDmaResult {
        if !self.gpu_dma.linked_active {
            let header_address = self.dma[2].base & 0x00ff_fffc;
            let Some(header) = self.dma_read_ram_word(header_address) else {
                self.dma[2].base = header_address;
                self.gpu_dma = GpuDmaProgress::default();
                return GpuDmaResult::Fault;
            };
            self.gpu_dma.linked_header = header_address;
            self.gpu_dma.linked_remaining = (header >> 24) as u8;
            self.gpu_dma.linked_next = header & 0x00ff_ffff;
            self.gpu_dma.linked_active = true;
            self.dma[2].base = Self::dma_advance(header_address, 4);
        }

        while self.gpu_dma.linked_remaining != 0 {
            if !self.gpu.can_accept_gp0_word() {
                return GpuDmaResult::Stalled;
            }
            let word_address = self.dma[2].base & 0x00ff_fffc;
            let Some(word) = self.dma_read_ram_word(word_address) else {
                self.dma[2].base = word_address;
                self.gpu_dma = GpuDmaProgress::default();
                return GpuDmaResult::Fault;
            };
            if !self.gpu.gp0(word) {
                return GpuDmaResult::Stalled;
            }
            self.dma[2].base = Self::dma_advance(word_address, 4);
            self.gpu_dma.linked_remaining -= 1;
        }

        let next_raw = self.gpu_dma.linked_next;
        let header_address = self.gpu_dma.linked_header;
        self.gpu_dma.linked_active = false;
        if next_raw == 0x00ff_ffff {
            self.dma[2].base = 0x00ff_fffc;
            self.gpu_dma = GpuDmaProgress::default();
            return GpuDmaResult::Complete;
        }
        let next = next_raw & 0x00ff_fffc;
        if next == header_address {
            self.dma[2].base = header_address;
            self.gpu_dma = GpuDmaProgress::default();
            return GpuDmaResult::Complete;
        }
        self.dma[2].base = next;
        GpuDmaResult::Stalled
    }
    fn run_cdrom_dma(&mut self) -> bool {
        let words = Self::dma_count16(self.dma[3].block).min(4_000_000);
        let mut address = self.dma[3].base & 0x00ff_fffc;
        for _ in 0..words {
            let value = self.cdrom.dma_read_word();
            if !self.dma_write_ram_word(address, value) {
                return false;
            }
            address = Self::dma_advance(address, 4);
        }
        true
    }

    fn run_spu_dma(&mut self) -> bool {
        let control = self.dma[4].control;
        let direction_from_ram = control & 1 != 0;
        let sync = (control >> 9) & 3;
        let block = self.dma[4].block;
        let words = match sync {
            1 => Self::dma_count16(block).saturating_mul(Self::dma_count16(block >> 16)),
            _ => Self::dma_count16(block),
        }
        .min(0x20_000);
        let step = if control & 2 != 0 { -4i32 } else { 4i32 };
        let mut address = self.dma[4].base & 0x00ff_fffc;
        for _ in 0..words {
            if direction_from_ram {
                let Some(value) = self.dma_read_ram_word(address) else {
                    self.dma[4].base = address;
                    return false;
                };
                self.spu.transfer_write_halfword(value as u16);
                self.spu.transfer_write_halfword((value >> 16) as u16);
            } else {
                let lo = u32::from(self.spu.transfer_read_halfword());
                let hi = u32::from(self.spu.transfer_read_halfword());
                if !self.dma_write_ram_word(address, lo | (hi << 16)) {
                    self.dma[4].base = address;
                    return false;
                }
            }
            address = Self::dma_advance(address, step);
        }
        self.dma[4].base = address;
        true
    }

    fn run_otc_dma(&mut self) -> bool {
        let count = Self::dma_count16(self.dma[6].block).min(0x1_0000);
        let mut address = self.dma[6].base & 0x00ff_fffc;
        for index in 0..count {
            let value = if index + 1 == count {
                0x00ff_ffff
            } else {
                address.wrapping_sub(4) & 0x00ff_ffff
            };
            if !self.dma_write_ram_word(address, value) {
                return false;
            }
            address = Self::dma_advance(address, -4);
        }
        true
    }

    fn timer_source_ticks(&mut self, index: usize, cpu_cycles: u32, hblank_edge: bool) -> u32 {
        let source = (self.timers[index].mode >> 8) & 3;
        match index {
            0 if source & 1 != 0 => {
                let numerator = u64::from(cpu_cycles).saturating_mul(self.gpu.video_clock_hz());
                let denominator = CPU_CLOCK_HZ.saturating_mul(self.gpu.dot_clock_divisor());
                let timer = &mut self.timers[index];
                timer.clock_phase = timer.clock_phase.saturating_add(numerator);
                let ticks = timer.clock_phase / denominator;
                timer.clock_phase %= denominator;
                ticks.min(u64::from(u32::MAX)) as u32
            }
            1 if source & 1 != 0 => u32::from(hblank_edge),
            2 if source & 2 != 0 => {
                let timer = &mut self.timers[index];
                timer.clock_phase = timer.clock_phase.saturating_add(u64::from(cpu_cycles));
                let ticks = timer.clock_phase / 8;
                timer.clock_phase %= 8;
                ticks.min(u64::from(u32::MAX)) as u32
            }
            _ => cpu_cycles,
        }
    }

    fn tick_timer(&mut self, index: usize, cpu_cycles: u32, signals: TimerSyncSignals) {
        let mut ticks = self.timer_source_ticks(index, cpu_cycles, signals.hblank_edge);
        let mode = self.timers[index].mode;
        if mode & 1 != 0 {
            let sync_mode = (mode >> 1) & 3;
            match index {
                0 => match sync_mode {
                    0 => {
                        if signals.was_hblank && signals.is_hblank {
                            ticks = 0;
                        }
                    }
                    1 => {
                        if signals.hblank_edge {
                            self.timers[index].reset_on_sync_edge();
                        }
                    }
                    2 => {
                        if signals.hblank_edge {
                            self.timers[index].reset_on_sync_edge();
                        }
                        if !signals.is_hblank {
                            ticks = 0;
                        }
                    }
                    _ => {
                        if !self.timers[index].sync_seen {
                            if signals.hblank_edge {
                                self.timers[index].sync_seen = true;
                            }
                            ticks = 0;
                        }
                    }
                },
                1 => match sync_mode {
                    0 => {
                        if signals.was_vblank && signals.is_vblank {
                            ticks = 0;
                        }
                    }
                    1 => {
                        if signals.vblank_edge {
                            self.timers[index].reset_on_sync_edge();
                        }
                    }
                    2 => {
                        if signals.vblank_edge {
                            self.timers[index].reset_on_sync_edge();
                        }
                        if !signals.is_vblank {
                            ticks = 0;
                        }
                    }
                    _ => {
                        if !self.timers[index].sync_seen {
                            if signals.vblank_edge {
                                self.timers[index].sync_seen = true;
                            }
                            ticks = 0;
                        }
                    }
                },
                2 if sync_mode == 0 || sync_mode == 3 => ticks = 0,
                _ => {}
            }
        }
        if self.timers[index].tick(ticks) {
            self.irq_status |= 1 << (4 + index);
        }
    }

    fn tick(&mut self, cpu_cycles: u32) {
        self.cdrom.tick_cpu_cycles(cpu_cycles);
        if self.cdrom.irq_pending() {
            self.irq_status |= 1 << 2;
        }
        self.service_dma();
        self.spu.queue_cd_audio(self.cdrom.drain_audio());
        self.spu.tick_cpu_cycles(cpu_cycles);
        if self.spu.irq_pending() {
            self.irq_status |= 1 << 9;
        }

        let was_hblank = self.gpu.in_hblank();
        let was_vblank = self.gpu.in_vblank();
        self.gpu.tick_cpu_cycles(cpu_cycles);
        let is_hblank = self.gpu.in_hblank();
        let is_vblank = self.gpu.in_vblank();
        let hblank_edge = !was_hblank && is_hblank;
        let vblank_edge = !was_vblank && is_vblank;
        if vblank_edge {
            self.irq_status |= 1;
        }
        if self.gpu.irq_pending() {
            self.irq_status |= 1 << 1;
        }

        let signals = TimerSyncSignals {
            was_hblank,
            is_hblank,
            hblank_edge,
            was_vblank,
            is_vblank,
            vblank_edge,
        };
        for index in 0..3 {
            self.tick_timer(index, cpu_cycles, signals);
        }
    }

    fn irq_pending(&self) -> bool {
        self.irq_status & self.irq_mask != 0
    }
    fn read_mmio32(&mut self, physical: u32) -> Option<u32> {
        if (0x1f80_1000..=0x1f80_1020).contains(&physical) && physical & 3 == 0 {
            return Some(self.memory_control[((physical - 0x1f80_1000) / 4) as usize]);
        }
        if let Some(value) = self.dma_register(physical) {
            return Some(value);
        }
        if (0x1f80_1100..=0x1f80_1128).contains(&physical) {
            let offset = physical - 0x1f80_1100;
            let timer_index = (offset / 0x10) as usize;
            let register = offset & 0x0f;
            let timer = self.timers.get_mut(timer_index)?;
            return match register {
                0x0 => Some(u32::from(timer.counter)),
                0x4 => Some(u32::from(timer.read_mode())),
                0x8 => Some(u32::from(timer.target)),
                _ => None,
            };
        }
        Some(match physical {
            0x1f80_1060 => self.ram_size,
            0x1f80_1070 => self.irq_status,
            0x1f80_1074 => self.irq_mask,
            0x1f80_10f0 => self.dpcr,
            0x1f80_10f4 => self.dicr,
            0x1f80_1810 => self.gpu.gpuread(),
            0x1f80_1814 => self.gpu.status(),
            0x1f80_1820 => self.mdec.read_data(),
            0x1f80_1824 => self.mdec.status(),
            address if (0x1f80_1c00..=0x1f80_1e7c).contains(&address) => {
                u32::from(self.spu.read16(address))
                    | (u32::from(self.spu.read16(address + 2)) << 16)
            }
            0x1ffe_0130 => self.cache_control,
            _ => return None,
        })
    }

    fn write_mmio32(&mut self, physical: u32, value: u32) -> bool {
        if (0x1f80_1000..=0x1f80_1020).contains(&physical) && physical & 3 == 0 {
            let index = ((physical - 0x1f80_1000) / 4) as usize;
            self.memory_control[index] = value;
            return true;
        }
        if self.write_dma_register(physical, value) {
            return true;
        }
        if (0x1f80_1100..=0x1f80_1128).contains(&physical) {
            let offset = physical - 0x1f80_1100;
            let timer_index = (offset / 0x10) as usize;
            let register = offset & 0x0f;
            let Some(timer) = self.timers.get_mut(timer_index) else {
                return false;
            };
            match register {
                0x0 => timer.write_counter(value as u16),
                0x4 => timer.write_mode(value as u16),
                0x8 => timer.write_target(value as u16),
                _ => return false,
            }
            return true;
        }
        match physical {
            0x1f80_1060 => self.ram_size = value,
            0x1f80_1070 => self.irq_status &= value,
            0x1f80_1074 => self.irq_mask = value & 0x7ff,
            0x1f80_10f0 => {
                self.dpcr = value;
                self.service_dma();
            }
            0x1f80_10f4 => self.write_dicr(value),
            0x1f80_1810 => {
                let _ = self.gpu.gp0(value);
            }
            0x1f80_1814 => self.gpu.gp1(value),
            0x1f80_1820 => {
                self.mdec.write_data(value);
                self.service_mdec_dma();
            }
            0x1f80_1824 => {
                self.mdec.write_control(value);
                self.service_mdec_dma();
            }
            address if (0x1f80_1c00..=0x1f80_1e7c).contains(&address) => {
                self.spu.write16(address, value as u16);
                self.spu.write16(address + 2, (value >> 16) as u16);
            }
            0x1ffe_0130 => self.cache_control = value & !((1 << 6) | (1 << 10)),
            _ => return false,
        }
        true
    }

    fn save(&self, out: &mut StateWriter) {
        out.blob(&self.ram);
        out.blob(&self.scratch);
        self.gpu.save(out);
        self.gte.save(out);
        self.mdec.save(out);
        self.cdrom.save(out);
        out.u32(self.irq_status);
        out.u32(self.irq_mask);
        for channel in self.dma {
            out.u32(channel.base);
            out.u32(channel.block);
            out.u32(channel.control);
        }
        out.u32(self.dpcr);
        out.u32(self.dicr);
        for remaining in self.mdec_dma_remaining {
            out.u32(remaining);
        }
        self.gpu_dma.save(out);
        for timer in self.timers {
            timer.save(out);
        }
        self.spu.save(out);
        for player in 0..2 {
            out.u16(self.pad_buttons[player]);
            for axis in self.pad_axes[player] {
                out.u8(axis);
            }
            out.u8(self.pad_analog[player] as u8);
            out.u8(self.pad_config[player] as u8);
            out.u8(self.pad_mode_locked[player] as u8);
            for mapping in self.pad_rumble_map[player] {
                out.u8(mapping);
            }
            out.u8(self.pad_rumble_small[player] as u8);
            out.u8(self.pad_rumble_large[player]);
        }
        out.u32(self.sio_rx.len() as u32);
        for value in &self.sio_rx {
            out.u8(*value);
        }
        out.u16(self.sio_mode);
        out.u16(self.sio_control);
        out.u16(self.sio_baud);
        out.u16(self.sio_phase);
        out.u8(self.sio_device);
        out.u8(self.sio_command);
        out.u8(self.sio_parameter);
        out.u8(self.card_flag);
        out.u16(self.card_address);
        out.u8(self.card_checksum);
        out.u8(self.card_write_result);
        out.blob(&self.card_write_buffer);
        out.blob(&self.memory_card);
        for register in self.memory_control {
            out.u32(register);
        }
        out.u32(self.ram_size);
        out.u32(self.cache_control);
        out.u8(self.cache_isolated as u8);
        for word in self.icache_words {
            out.u32(word);
        }
        for tag in self.icache_tags {
            out.u32(tag);
        }
        for valid in self.icache_valid {
            out.u8(valid);
        }
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
        self.gte.load(input)?;
        self.mdec.load(input)?;
        self.cdrom.load(input)?;
        self.irq_status = input.u32()?;
        self.irq_mask = input.u32()?;
        for channel in &mut self.dma {
            channel.base = input.u32()?;
            channel.block = input.u32()?;
            channel.control = input.u32()?;
        }
        self.dpcr = input.u32()?;
        self.dicr = input.u32()?;
        for remaining in &mut self.mdec_dma_remaining {
            *remaining = input.u32()?;
        }
        self.gpu_dma.load(input)?;
        for timer in &mut self.timers {
            timer.load(input)?;
        }
        self.spu.load(input)?;
        for player in 0..2 {
            self.pad_buttons[player] = input.u16()?;
            for axis in &mut self.pad_axes[player] {
                *axis = input.u8()?;
            }
            self.pad_analog[player] = input.u8()? != 0;
            self.pad_config[player] = input.u8()? != 0;
            self.pad_mode_locked[player] = input.u8()? != 0;
            for mapping in &mut self.pad_rumble_map[player] {
                *mapping = input.u8()?;
            }
            self.pad_rumble_small[player] = input.u8()? != 0;
            self.pad_rumble_large[player] = input.u8()?;
        }
        let rx_len = input.u32()? as usize;
        if rx_len > 8 {
            return Err("invalid PlayStation SIO queue length".into());
        }
        self.sio_rx.clear();
        for _ in 0..rx_len {
            self.sio_rx.push_back(input.u8()?);
        }
        self.sio_mode = input.u16()?;
        self.sio_control = input.u16()?;
        self.sio_baud = input.u16()?;
        self.sio_phase = input.u16()?;
        self.sio_device = input.u8()?;
        self.sio_command = input.u8()?;
        self.sio_parameter = input.u8()?;
        self.card_flag = input.u8()?;
        self.card_address = input.u16()?;
        self.card_checksum = input.u8()?;
        self.card_write_result = input.u8()?;
        let write_buffer = input.blob()?;
        if write_buffer.len() != self.card_write_buffer.len() {
            return Err("invalid PlayStation memory-card write buffer length".into());
        }
        self.card_write_buffer.copy_from_slice(write_buffer);
        if self.sio_phase > 139 || !matches!(self.sio_device, 0 | 0x01 | 0x81) {
            return Err("invalid PlayStation SIO transaction state".into());
        }
        let card = input.blob()?;
        if card.len() != self.memory_card.len() {
            return Err("invalid PlayStation memory-card state length".into());
        }
        self.memory_card.copy_from_slice(card);
        for register in &mut self.memory_control {
            *register = input.u32()?;
        }
        self.ram_size = input.u32()?;
        self.cache_control = input.u32()?;
        self.cache_isolated = input.u8()? != 0;
        for word in &mut self.icache_words {
            *word = input.u32()?;
        }
        for tag in &mut self.icache_tags {
            *tag = input.u32()?;
        }
        for valid in &mut self.icache_valid {
            *valid = input.u8()? & 0x0f;
        }
        Ok(())
    }
}

impl MipsBus for Ps1Bus {
    fn read8(&mut self, address: u32) -> u8 {
        if self.cache_isolated {
            let aligned = address & !3;
            let shift = (address & 3) * 8;
            return (self.isolated_cache_read32(aligned) >> shift) as u8;
        }
        self.read_memory8(Self::physical(address))
    }

    fn write8(&mut self, address: u32, value: u8) {
        if self.cache_isolated {
            let aligned = address & !3;
            let shift = (address & 3) * 8;
            let mask = !(0xffu32 << shift);
            let merged = (self.isolated_cache_read32(aligned) & mask) | (u32::from(value) << shift);
            self.isolated_cache_write32(aligned, merged);
            return;
        }
        self.write_memory8(Self::physical(address), value);
    }

    fn fetch32(&mut self, address: u32) -> u32 {
        self.fetch_instruction32(address)
    }

    fn instruction_bus_error(&mut self, address: u32) -> bool {
        !self.instruction_cache_hit(address) && self.ram_window_bus_error(address, 4)
    }

    fn data_bus_error(&mut self, address: u32, size: u8, _write: bool) -> bool {
        !self.cache_isolated && self.ram_window_bus_error(address, size)
    }

    fn load_cycles(&mut self, address: u32, _size: u8) -> u32 {
        if self.cache_isolated {
            return 1;
        }
        let physical = Self::physical(address);
        if physical < self.ram_window_size() {
            return 7;
        }
        if (0x1f80_0000..=0x1f80_03ff).contains(&physical) {
            return 1;
        }
        if (0x1fc0_0000..0x1fc8_0000).contains(&physical) {
            return 27;
        }
        if matches!(
            physical,
            0x1f80_1000..=0x1f80_1023
                | 0x1f80_1060..=0x1f80_1077
                | 0x1f80_1080..=0x1f80_10f7
                | 0x1f80_1100..=0x1f80_112b
        ) {
            return 5;
        }
        1
    }

    fn set_cache_isolated(&mut self, isolated: bool) {
        self.cache_isolated = isolated;
    }

    fn read16(&mut self, address: u32) -> u16 {
        if self.cache_isolated {
            return u16::from_le_bytes([self.read8(address), self.read8(address.wrapping_add(1))]);
        }
        let physical = Self::physical(address);
        if (0x1f80_1100..=0x1f80_1128).contains(&physical) {
            let offset = physical - 0x1f80_1100;
            let timer_index = (offset / 0x10) as usize;
            let register = offset & 0x0f;
            if let Some(timer) = self.timers.get_mut(timer_index) {
                return match register {
                    0x0 => timer.counter,
                    0x4 => timer.read_mode(),
                    0x8 => timer.target,
                    _ => 0,
                };
            }
        }
        match physical {
            0x1f80_1044 => self.sio_status(),
            0x1f80_1070 => self.irq_status as u16,
            0x1f80_1074 => self.irq_mask as u16,
            0x1f80_1048 => self.sio_mode,
            0x1f80_104a => self.sio_control,
            0x1f80_104e => self.sio_baud,
            address if (0x1f80_1c00..=0x1f80_1e7e).contains(&address) => {
                self.spu.read16(address & !1)
            }
            _ => u16::from_le_bytes([
                self.read_memory8(physical),
                self.read_memory8(physical.wrapping_add(1)),
            ]),
        }
    }
    fn write16(&mut self, address: u32, value: u16) {
        if self.cache_isolated {
            let [lo, hi] = value.to_le_bytes();
            self.write8(address, lo);
            self.write8(address.wrapping_add(1), hi);
            return;
        }
        let physical = Self::physical(address);
        if (0x1f80_1100..=0x1f80_1128).contains(&physical) {
            let offset = physical - 0x1f80_1100;
            let timer_index = (offset / 0x10) as usize;
            let register = offset & 0x0f;
            if let Some(timer) = self.timers.get_mut(timer_index) {
                match register {
                    0x0 => timer.write_counter(value),
                    0x4 => timer.write_mode(value),
                    0x8 => timer.write_target(value),
                    _ => {}
                }
                return;
            }
        }
        match physical {
            0x1f80_1048 => self.sio_mode = value,
            0x1f80_1070 => self.irq_status &= u32::from(value),
            0x1f80_1074 => self.irq_mask = u32::from(value) & 0x7ff,
            0x1f80_104a => {
                let previous = self.sio_control;
                if value & (1 << 4) != 0 {
                    self.irq_status &= !(1 << 7);
                }
                self.sio_control = value & !((1 << 4) | (1 << 6));
                if value & (1 << 6) != 0 {
                    self.sio_rx.clear();
                    self.reset_sio_transaction();
                } else if self.sio_control & (1 << 1) == 0
                    || (previous ^ self.sio_control) & (1 << 13) != 0
                {
                    self.reset_sio_transaction();
                }
            }
            0x1f80_104e => self.sio_baud = value,
            address if (0x1f80_1c00..=0x1f80_1e7e).contains(&address) => {
                self.spu.write16(address & !1, value)
            }
            _ => {
                let [lo, hi] = value.to_le_bytes();
                self.write_memory8(physical, lo);
                self.write_memory8(physical.wrapping_add(1), hi);
            }
        }
    }

    fn read32(&mut self, address: u32) -> u32 {
        if self.cache_isolated {
            return self.isolated_cache_read32(address);
        }
        let physical = Self::physical(address);
        self.read_mmio32(physical)
            .unwrap_or_else(|| self.read_word_le(physical))
    }

    fn write32(&mut self, address: u32, value: u32) {
        if self.cache_isolated {
            self.isolated_cache_write32(address, value);
            return;
        }
        let physical = Self::physical(address);
        if !self.write_mmio32(physical, value) {
            self.write_word_le(physical, value);
        }
    }

    fn store8(&mut self, address: u32, source: u32) {
        if self.cache_isolated {
            self.write8(address, source as u8);
            return;
        }
        let physical = Self::physical(address);
        if Self::partial_store_latches_word(physical) {
            let shift = (physical & 3) * 8;
            self.write32(physical & !3, source.wrapping_shl(shift));
            return;
        }
        if (0x1f80_1084..=0x1f80_10e7).contains(&physical) && physical & 0x0f <= 7 {
            let aligned = physical & !3;
            let shift = (physical & 3) * 8;
            let mask = !(0xffu32 << shift);
            let merged = (self.read32(aligned) & mask) | ((source & 0xff) << shift);
            self.write32(aligned, merged);
            return;
        }
        if (0x1f80_1c00..=0x1f80_1e7f).contains(&physical) {
            if physical & 1 == 0 {
                self.spu.write16(physical, source as u16);
            }
            return;
        }
        if matches!(physical & !1, 0x1f80_1048 | 0x1f80_104a | 0x1f80_104e) {
            let shift = (physical & 1) * 8;
            self.write16(physical & !1, source.wrapping_shl(shift) as u16);
            return;
        }
        self.write8(address, source as u8);
    }

    fn store16(&mut self, address: u32, source: u32) {
        if self.cache_isolated {
            self.write16(address, source as u16);
            return;
        }
        let physical = Self::physical(address);
        if Self::partial_store_latches_word(physical) {
            let shift = (physical & 2) * 8;
            self.write32(physical & !3, source.wrapping_shl(shift));
            return;
        }
        if (0x1f80_1084..=0x1f80_10e7).contains(&physical) && physical & 0x0f <= 7 {
            let aligned = physical & !3;
            let shift = (physical & 2) * 8;
            let mask = !(0xffffu32 << shift);
            let merged = (self.read32(aligned) & mask) | ((source & 0xffff) << shift);
            self.write32(aligned, merged);
            return;
        }
        self.write16(address, source as u16);
    }

    fn store_left(&mut self, address: u32, source: u32) {
        let physical = Self::physical(address);
        let aligned = physical & !3;
        let offset = physical & 3;
        if self.cache_isolated {
            let memory = self.isolated_cache_read32(aligned);
            let merged = match offset {
                0 => (memory & 0xffff_ff00) | (source >> 24),
                1 => (memory & 0xffff_0000) | (source >> 16),
                2 => (memory & 0xff00_0000) | (source >> 8),
                _ => source,
            };
            self.isolated_cache_write32(aligned, merged);
            return;
        }
        if Self::partial_store_latches_word(physical) {
            self.write32(aligned, source >> ((3 - offset) * 8));
            return;
        }
        if (0x1f80_1c00..=0x1f80_1e7f).contains(&physical) {
            match offset {
                0 => self.spu.write16(aligned, (source >> 24) as u16),
                1 => self.spu.write16(aligned, (source >> 16) as u16),
                2 => self.spu.write16(aligned, (source >> 8) as u16),
                _ => self.write32(aligned, source),
            }
            return;
        }
        let memory = self.read32(aligned);
        let merged = match offset {
            0 => (memory & 0xffff_ff00) | (source >> 24),
            1 => (memory & 0xffff_0000) | (source >> 16),
            2 => (memory & 0xff00_0000) | (source >> 8),
            _ => source,
        };
        self.write32(aligned, merged);
    }

    fn store_right(&mut self, address: u32, source: u32) {
        let physical = Self::physical(address);
        let aligned = physical & !3;
        let offset = physical & 3;
        if self.cache_isolated {
            let memory = self.isolated_cache_read32(aligned);
            let merged = match offset {
                0 => source,
                1 => (memory & 0x0000_00ff) | (source << 8),
                2 => (memory & 0x0000_ffff) | (source << 16),
                _ => (memory & 0x00ff_ffff) | (source << 24),
            };
            self.isolated_cache_write32(aligned, merged);
            return;
        }
        if Self::partial_store_latches_word(physical) {
            self.write32(aligned, source << (offset * 8));
            return;
        }
        if (0x1f80_1c00..=0x1f80_1e7f).contains(&physical) {
            match offset {
                0 => self.write32(aligned, source),
                1 => self.spu.write16(aligned, (source << 8) as u16),
                2 => self.spu.write16(aligned + 2, source as u16),
                _ => {}
            }
            return;
        }
        let memory = self.read32(aligned);
        let merged = match offset {
            0 => source,
            1 => (memory & 0x0000_00ff) | (source << 8),
            2 => (memory & 0x0000_ffff) | (source << 16),
            _ => (memory & 0x00ff_ffff) | (source << 24),
        };
        self.write32(aligned, merged);
    }

    fn cop2_present(&self) -> bool {
        true
    }

    fn cop2_read_data(&mut self, register: u8) -> u32 {
        self.gte.read_data(register)
    }

    fn cop2_write_data(&mut self, register: u8, value: u32) {
        self.gte.write_data(register, value);
    }

    fn cop2_write_data_at(&mut self, register: u8, value: u32, cycle: u64) {
        self.gte.write_data_at(register, value, cycle);
    }

    fn cop2_read_control(&mut self, register: u8) -> u32 {
        self.gte.read_control(register)
    }

    fn cop2_write_control(&mut self, register: u8, value: u32) {
        self.gte.write_control(register, value);
    }

    fn cop2_write_control_at(&mut self, register: u8, value: u32, cycle: u64) {
        self.gte.write_control_at(register, value, cycle);
    }

    fn cop2_command(&mut self, instruction: u32) -> u32 {
        self.gte.command(instruction)
    }

    fn cop2_command_at(&mut self, instruction: u32, cycle: u64) -> u32 {
        self.gte.command_at(instruction, cycle)
    }

    fn cop2_advance_to(&mut self, cycle: u64) {
        self.gte.advance_to(cycle);
    }
}

pub struct PlayStationMachine {
    cpu: MipsR3000,
    bus: Ps1Bus,
    audio: AudioBuffer,
    powered: bool,
    boot_executable: Option<Vec<u8>>,
}
impl PlayStationMachine {
    pub fn from_bios(bios: &[u8]) -> Result<Self, String> {
        Self::from_bios_and_disc(bios, None)
    }

    pub fn from_bios_and_disc(bios: &[u8], disc: Option<ResourceBlob>) -> Result<Self, String> {
        let mut bus = Ps1Bus::new_with_disc(bios, disc)?;
        let mut cpu = MipsR3000::default();
        cpu.reset();
        let _ = bus.read8(cpu.pc);
        Ok(Self {
            cpu,
            bus,
            audio: AudioBuffer::new(AUDIO_RATE, 2),
            powered: true,
            boot_executable: None,
        })
    }

    pub fn from_bios_and_executable(bios: &[u8], executable: &[u8]) -> Result<Self, String> {
        Self::validate_ps_exe(executable)?;
        let mut machine = Self::from_bios(bios)?;
        Self::prepare_executable_boot(&mut machine.cpu, &mut machine.bus, executable)?;
        machine.boot_executable = Some(executable.to_vec());
        Ok(machine)
    }

    fn ps_exe_ram_range(
        address: u32,
        length: u32,
        label: &str,
    ) -> Result<std::ops::Range<usize>, String> {
        let start = Ps1Bus::physical(address);
        let end = start
            .checked_add(length)
            .ok_or_else(|| format!("PS-X EXE {label} range overflows address space"))?;
        if end > RAM_SIZE as u32 {
            return Err(format!("PS-X EXE {label} range is outside 2 MiB main RAM"));
        }
        Ok(start as usize..end as usize)
    }

    fn validate_ps_exe(executable: &[u8]) -> Result<(), String> {
        if executable.len() < PS_EXE_HEADER_SIZE {
            return Err("PS-X EXE is smaller than its 0x800-byte header".into());
        }
        if &executable[..8] != b"PS-X EXE" {
            return Err("PlayStation executable is missing the PS-X EXE magic".into());
        }
        let word =
            |offset: usize| u32::from_le_bytes(executable[offset..offset + 4].try_into().unwrap());
        let entry = word(0x10);
        let load_address = word(0x18);
        let text_size = word(0x1c);
        let bss_address = word(0x28);
        let bss_size = word(0x2c);

        if entry & 3 != 0 || Ps1Bus::physical(entry) >= RAM_SIZE as u32 {
            return Err("PS-X EXE entry point is not a word-aligned main-RAM address".into());
        }
        if !(text_size as usize).is_multiple_of(PS_EXE_HEADER_SIZE) {
            return Err("PS-X EXE body size is not a multiple of 0x800 bytes".into());
        }
        let body_end = PS_EXE_HEADER_SIZE
            .checked_add(text_size as usize)
            .ok_or_else(|| "PS-X EXE body size overflows host address space".to_string())?;
        if body_end > executable.len() {
            return Err("PS-X EXE body is shorter than the size declared in its header".into());
        }
        Self::ps_exe_ram_range(load_address, text_size, "body")?;
        if bss_size != 0 {
            if bss_address & 3 != 0 || bss_size & 3 != 0 {
                return Err("PS-X EXE BSS range must be word aligned".into());
            }
            Self::ps_exe_ram_range(bss_address, bss_size, "BSS")?;
        }
        Ok(())
    }

    fn bios_vectors_ready(bus: &Ps1Bus) -> bool {
        [0xa0usize, 0xb0, 0xc0]
            .into_iter()
            .all(|offset| u32::from_le_bytes(bus.ram[offset..offset + 4].try_into().unwrap()) != 0)
    }

    fn initialize_bios_kernel(cpu: &mut MipsR3000, bus: &mut Ps1Bus) -> bool {
        let frame_cycles = bus.gpu.cpu_cycles_per_frame();
        let vector_deadline = cpu.cycles.saturating_add(frame_cycles.saturating_mul(4));
        let ready_deadline = cpu.cycles.saturating_add(frame_cycles.saturating_mul(16));
        let mut vectors_ready_at = None;
        while cpu.cycles < ready_deadline {
            if vectors_ready_at.is_none() && Self::bios_vectors_ready(bus) {
                vectors_ready_at = Some(cpu.cycles);
            }
            if vectors_ready_at
                .is_some_and(|ready| cpu.cycles.saturating_sub(ready) >= frame_cycles)
            {
                return true;
            }
            if vectors_ready_at.is_none() && cpu.cycles >= vector_deadline {
                break;
            }
            cpu.set_irq_line(2, bus.irq_pending());
            let used = cpu.step(bus);
            if used == 0 {
                break;
            }
            bus.tick(used);
        }
        vectors_ready_at.is_some_and(|ready| cpu.cycles.saturating_sub(ready) >= frame_cycles)
    }

    fn prepare_executable_boot(
        cpu: &mut MipsR3000,
        bus: &mut Ps1Bus,
        executable: &[u8],
    ) -> Result<(), String> {
        if !Self::initialize_bios_kernel(cpu, bus) {
            bus.reset();
            cpu.reset();
        }
        Self::load_ps_exe(cpu, bus, executable)
    }

    fn load_ps_exe(cpu: &mut MipsR3000, bus: &mut Ps1Bus, executable: &[u8]) -> Result<(), String> {
        Self::validate_ps_exe(executable)?;
        let word =
            |offset: usize| u32::from_le_bytes(executable[offset..offset + 4].try_into().unwrap());
        let entry = word(0x10);
        let gp = word(0x14);
        let load_address = word(0x18);
        let text_size = word(0x1c);
        let bss_address = word(0x28);
        let bss_size = word(0x2c);
        let stack_base = word(0x30);
        let stack_offset = word(0x34);
        let body_end = PS_EXE_HEADER_SIZE + text_size as usize;
        let text_range = Self::ps_exe_ram_range(load_address, text_size, "body")?;
        bus.ram[text_range].copy_from_slice(&executable[PS_EXE_HEADER_SIZE..body_end]);

        if bss_size != 0 {
            let bss_range = Self::ps_exe_ram_range(bss_address, bss_size, "BSS")?;
            bus.ram[bss_range].fill(0);
        }

        let stack = if stack_base == 0 {
            PS_EXE_DIRECT_STACK
        } else {
            stack_base.wrapping_add(stack_offset)
        };
        cpu.enter_program(entry);
        cpu.regs[4] = 1;
        cpu.regs[5] = 0;
        cpu.regs[28] = gp;
        cpu.regs[29] = stack;
        cpu.regs[30] = stack;
        cpu.regs[0] = 0;
        Ok(())
    }

    pub fn has_disc(&self) -> bool {
        self.bus.cdrom.has_disc()
    }

    fn clock_instruction(&mut self) -> u32 {
        self.cpu.set_irq_line(2, self.bus.irq_pending());
        let used = self.cpu.step(&mut self.bus);
        self.bus.tick(used);
        used
    }

    fn flush_audio(&mut self) {
        self.audio.begin_frame();
        let samples = self.bus.spu.drain_output();
        for frame in samples.as_chunks::<2>().0 {
            self.audio.push_stereo(frame[0], frame[1]);
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
        self.powered = match self.boot_executable.as_deref() {
            Some(executable) => {
                Self::prepare_executable_boot(&mut self.cpu, &mut self.bus, executable).is_ok()
            }
            None => true,
        };
        self.flush_audio();
    }

    fn run_frame(&mut self, input: &InputState) {
        self.bus.set_inputs(input);
        let target = self.bus.gpu.frame().wrapping_add(1);
        let deadline = self
            .cpu
            .cycles
            .saturating_add(self.bus.gpu.cpu_cycles_per_frame().saturating_add(20_000));
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
        self.bus.gpu.frame_rate()
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
        let words: [u32; 18] = [
            0x3c08_1f80,
            0x3508_1814,
            0x3c09_0800,
            0x3529_0001,
            0xad09_0000,
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
            0x0bf0_0010,
            0x0000_0000,
        ];
        for (index, word) in words.into_iter().enumerate() {
            bios[index * 4..index * 4 + 4].copy_from_slice(&word.to_le_bytes());
        }
        bios
    }

    fn synthetic_ps_exe() -> Vec<u8> {
        let mut executable = vec![0u8; PS_EXE_HEADER_SIZE * 2];
        executable[..8].copy_from_slice(b"PS-X EXE");
        for (offset, value) in [
            (0x10, 0x8001_0000u32),
            (0x14, 0x8001_1000),
            (0x18, 0x8001_0000),
            (0x1c, PS_EXE_HEADER_SIZE as u32),
            (0x28, 0x8001_07f0),
            (0x2c, 16),
        ] {
            executable[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
        }
        executable[PS_EXE_HEADER_SIZE..].fill(0x5a);
        executable
    }

    fn sio_exchange(bus: &mut Ps1Bus, value: u8) -> u8 {
        bus.sio_write(value);
        bus.sio_rx.pop_front().expect("SIO exchange response")
    }

    fn sio_transaction(bus: &mut Ps1Bus, bytes: &[u8]) -> Vec<u8> {
        bytes
            .iter()
            .copied()
            .map(|byte| sio_exchange(bus, byte))
            .collect()
    }

    fn synthetic_xa_disc() -> ResourceBlob {
        let mut raw = vec![0u8; 2352];
        raw[0] = 0;
        raw[1..11].fill(0xff);
        raw[11] = 0;
        raw[15] = 2;
        raw[16..20].copy_from_slice(&[1, 2, 0x64, 0x01]);
        raw[20..24].copy_from_slice(&[1, 2, 0x64, 0x01]);
        for group in 0..18 {
            let start = 24 + group * 128;
            raw[start + 16..start + 128].fill(0x11);
        }
        ResourceBlob::from_bytes(&raw)
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
    fn ps_exe_loader_initializes_cpu_ram_bss_and_default_stack() {
        let bios = synthetic_bios();
        let executable = synthetic_ps_exe();
        let mut machine = PlayStationMachine::from_bios_and_executable(&bios, &executable).unwrap();

        assert_eq!(machine.cpu.pc, 0x8001_0000);
        assert_eq!(machine.cpu.next_pc, 0x8001_0004);
        assert_eq!(machine.cpu.regs[4], 1);
        assert_eq!(machine.cpu.regs[5], 0);
        assert_eq!(machine.cpu.regs[28], 0x8001_1000);
        assert_eq!(machine.cpu.regs[29], PS_EXE_DIRECT_STACK);
        assert_eq!(machine.cpu.regs[30], PS_EXE_DIRECT_STACK);
        assert_eq!(machine.bus.ram[0x1_0000], 0x5a);
        assert!(machine.bus.ram[0x1_07f0..0x1_0800]
            .iter()
            .all(|&byte| byte == 0));

        machine.bus.ram[0x1_0000] = 0;
        machine.cpu.pc = 0x1234;
        machine.reset();
        assert!(machine.powered);
        assert_eq!(machine.cpu.pc, 0x8001_0000);
        assert_eq!(machine.bus.ram[0x1_0000], 0x5a);
    }

    #[test]
    fn ps_exe_loader_rejects_malformed_or_out_of_ram_images() {
        let bios = synthetic_bios();
        assert!(PlayStationMachine::from_bios_and_executable(&bios, &[0; 32]).is_err());

        let mut bad_magic = synthetic_ps_exe();
        bad_magic[0] = b'X';
        assert!(PlayStationMachine::from_bios_and_executable(&bios, &bad_magic).is_err());

        let mut short_body = synthetic_ps_exe();
        short_body.pop();
        assert!(PlayStationMachine::from_bios_and_executable(&bios, &short_body).is_err());

        let mut outside_ram = synthetic_ps_exe();
        outside_ram[0x18..0x1c].copy_from_slice(&0x801f_fc00u32.to_le_bytes());
        assert!(PlayStationMachine::from_bios_and_executable(&bios, &outside_ram).is_err());
    }

    #[test]
    fn machine_factory_routes_ps_exe_game_resources_to_direct_loader() {
        let bios = synthetic_bios();
        let mut executable = synthetic_ps_exe();
        executable[0x18..0x1c].copy_from_slice(&0x801f_fc00u32.to_le_bytes());
        let mut resources = crate::resources::ResourceStore::default();
        resources.insert_bytes(ResourceKind::Bios, 0, &bios);
        resources.insert_bytes(ResourceKind::Game, 0, &executable);

        let result = crate::machine::create_machine(PlatformId::PlayStation, &resources);
        let error = match result {
            Ok(_) => panic!("malformed PS-X EXE should be rejected by the direct loader"),
            Err(error) => error,
        };
        assert!(error.contains("PS-X EXE body range is outside 2 MiB main RAM"));
    }

    #[test]
    fn gte_cop2_executes_through_playstation_bus_with_load_delay() {
        let bios = synthetic_bios();
        let mut bus = Ps1Bus::new_with_disc(&bios, None).unwrap();
        for (register, value) in [(0, 0x1000), (1, 0), (2, 0x1000), (3, 0), (4, 0x1000)] {
            bus.gte.write_control(register, value);
        }
        bus.gte.write_data(0, 0xff80_0100);
        bus.gte.write_data(1, 0x1000);
        bus.gte.write_control(26, 0x1000);
        bus.write_word_le(0, 0x4a08_0001);
        bus.write_word_le(4, 0x4802_7000);
        bus.write_word_le(8, 0x0040_1821);
        bus.write_word_le(12, 0x0040_2021);
        let mut cpu = MipsR3000::default();
        cpu.pc = 0;
        cpu.next_pc = 4;
        cpu.cop0[12] = 1 << 30;
        assert_eq!(cpu.step(&mut bus), 1);
        assert_eq!(cpu.step(&mut bus), 15);
        assert_eq!(cpu.step(&mut bus), 1);
        assert_eq!(cpu.regs[3], 0);
        assert_eq!(cpu.step(&mut bus), 1);
        assert_eq!(cpu.regs[4], 0xff80_0100);
    }

    #[test]
    fn cpu_mtc2_arrival_cycle_controls_rtps_dqa_latch() {
        let run = |nops: usize| {
            let bios = synthetic_bios();
            let mut bus = Ps1Bus::new_with_disc(&bios, None).unwrap();
            for (register, value) in [(0, 0x1000), (1, 0), (2, 0x1000), (3, 0), (4, 0x1000)] {
                bus.gte.write_control(register, value);
            }
            bus.gte.write_data(0, 0);
            bus.gte.write_data(1, 0x1000);
            bus.gte.write_control(26, 0x1000);

            let mut address = 0u32;
            bus.write_word_le(address, 0x4a08_0001);
            address += 4;
            for _ in 0..nops {
                bus.write_word_le(address, 0);
                address += 4;
            }
            bus.write_word_le(address, 0x48c1_d800);
            address += 4;
            bus.write_word_le(address, 0x4802_4000);

            let mut cpu = MipsR3000::default();
            cpu.pc = 0;
            cpu.next_pc = 4;
            cpu.cop0[12] = 1 << 30;
            cpu.regs[1] = 0x1000;
            for _ in 0..nops + 3 {
                cpu.step(&mut bus);
            }
            bus.gte.read_data(8)
        };

        assert_eq!(run(3), 0x1000);
        assert_eq!(run(4), 0);
    }

    #[test]
    fn instruction_cache_keeps_stale_code_until_tag_invalidation() {
        let bios = synthetic_bios();
        let mut bus = Ps1Bus::new_with_disc(&bios, None).unwrap();
        bus.cache_control = (1 << 11) | (1 << 8);
        bus.write_word_le(0, 0x2401_0001);
        assert_eq!(bus.fetch_instruction32(0x8000_0000), 0x2401_0001);

        bus.write_word_le(0, 0x2401_0002);
        assert_eq!(bus.fetch_instruction32(0x8000_0000), 0x2401_0001);
        assert_eq!(bus.fetch_instruction32(0xa000_0000), 0x2401_0002);

        bus.cache_control = (1 << 11) | (1 << 2);
        bus.cache_isolated = true;
        MipsBus::write32(&mut bus, 0x0000_0000, 0);
        bus.cache_isolated = false;
        bus.cache_control = (1 << 11) | (1 << 8);
        assert_eq!(bus.fetch_instruction32(0x8000_0000), 0x2401_0002);
    }

    #[test]
    fn isolated_cache_flush_writes_do_not_modify_main_ram() {
        let bios = synthetic_bios();
        let mut bus = Ps1Bus::new_with_disc(&bios, None).unwrap();
        bus.write_word_le(0x100, 0xdead_beef);

        bus.cache_control = (1 << 11) | (1 << 2);
        bus.cache_isolated = true;
        MipsBus::write32(&mut bus, 0x100, 0);
        assert_eq!(bus.read_word_le(0x100), 0xdead_beef);
        let (line, _, _) = Ps1Bus::icache_location(0x100);
        assert_eq!(bus.icache_valid[line], 0);

        bus.cache_control = 1 << 11;
        MipsBus::write32(&mut bus, 0x100, 0x1122_3344);
        assert_eq!(MipsBus::read32(&mut bus, 0x100), 0x1122_3344);
        assert_eq!(bus.read_word_le(0x100), 0xdead_beef);
    }

    #[test]
    fn measured_load_cycle_classes_reach_r3000_step_timing() {
        let bios = synthetic_bios();
        let mut bus = Ps1Bus::new_with_disc(&bios, None).unwrap();
        assert_eq!(MipsBus::load_cycles(&mut bus, 0xa000_0100, 4), 7);
        assert_eq!(MipsBus::load_cycles(&mut bus, 0x1f80_0000, 4), 1);
        assert_eq!(MipsBus::load_cycles(&mut bus, 0x1f80_1070, 4), 5);
        assert_eq!(MipsBus::load_cycles(&mut bus, 0xbfc0_0000, 4), 27);

        bus.write_word_le(0, 0x8c01_0100);
        bus.write_word_le(4, 0x8c41_0000);
        bus.write_word_le(8, 0x8c41_0000);
        bus.write_word_le(0x100, 0x1234_5678);
        let mut cpu = MipsR3000::default();
        cpu.pc = 0xa000_0000;
        cpu.next_pc = 0xa000_0004;
        assert_eq!(cpu.step(&mut bus), 7);

        cpu.regs[2] = 0x1f80_1070;
        assert_eq!(cpu.step(&mut bus), 5);
        cpu.regs[2] = 0xbfc0_0000;
        assert_eq!(cpu.step(&mut bus), 27);
    }

    #[test]
    fn ram_size_bus_errors_reach_cpu_without_overwriting_badvaddr() {
        let bios = synthetic_bios();
        let mut bus = Ps1Bus::new_with_disc(&bios, None).unwrap();
        bus.ram_size = 0x0000_0888;

        let mut cpu = MipsR3000::default();
        cpu.pc = 0xa020_0000;
        cpu.next_pc = 0xa020_0004;
        cpu.cop0[8] = 0xfeed_beef;
        cpu.step(&mut bus);
        assert_eq!((cpu.cop0[13] >> 2) & 0x1f, 6);
        assert_eq!(cpu.cop0[8], 0xfeed_beef);

        bus.write_word_le(0, 0x8c41_0000);
        let mut cpu = MipsR3000::default();
        cpu.pc = 0xa000_0000;
        cpu.next_pc = 0xa000_0004;
        cpu.regs[2] = 0xa020_0000;
        cpu.cop0[8] = 0xcafe_babe;
        cpu.step(&mut bus);
        assert_eq!((cpu.cop0[13] >> 2) & 0x1f, 7);
        assert_eq!(cpu.cop0[8], 0xcafe_babe);
    }

    #[test]
    fn cached_instruction_survives_ram_window_shrink_but_uncached_fetch_faults() {
        let bios = synthetic_bios();
        let mut bus = Ps1Bus::new_with_disc(&bios, None).unwrap();
        bus.cache_control = (1 << 11) | (1 << 8);
        bus.write_word_le(0x0020_0000, 0x2401_0042);
        assert_eq!(bus.fetch_instruction32(0x8020_0000), 0x2401_0042);

        bus.ram_size = 0x0000_0888;
        assert!(!MipsBus::instruction_bus_error(&mut bus, 0x8020_0000));
        assert!(MipsBus::instruction_bus_error(&mut bus, 0xa020_0000));

        let mut cpu = MipsR3000::default();
        cpu.pc = 0x8020_0000;
        cpu.next_pc = 0x8020_0004;
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[1], 0x42);
    }

    #[test]
    fn cpu_isc_store_targets_instruction_cache_not_ram() {
        let bios = synthetic_bios();
        let mut bus = Ps1Bus::new_with_disc(&bios, None).unwrap();
        bus.cache_control = 1 << 11;
        bus.write_word_le(0, 0xac01_0100);
        bus.write_word_le(0x100, 0xcafe_babe);
        let mut cpu = MipsR3000::default();
        cpu.pc = 0xa000_0000;
        cpu.next_pc = 0xa000_0004;
        cpu.regs[1] = 0x1122_3344;
        cpu.cop0[12] = 1 << 16;
        cpu.step(&mut bus);
        assert_eq!(bus.read_word_le(0x100), 0xcafe_babe);
        let (_, word_index, _) = Ps1Bus::icache_location(0x100);
        assert_eq!(bus.icache_words[word_index], 0x1122_3344);
    }

    #[test]
    fn irq_and_timer_halfword_mmio_follow_native_register_widths() {
        let bios = synthetic_bios();
        let mut bus = Ps1Bus::new_with_disc(&bios, None).unwrap();
        bus.irq_status = 0x7ff;
        MipsBus::write16(&mut bus, 0x1f80_1070, 0x07fe);
        MipsBus::write16(&mut bus, 0x1f80_1074, 0x0456);
        assert_eq!(bus.irq_status, 0x07fe);
        assert_eq!(MipsBus::read16(&mut bus, 0x1f80_1070), 0x07fe);
        assert_eq!(MipsBus::read16(&mut bus, 0x1f80_1074), 0x0456);

        MipsBus::write16(&mut bus, 0x1f80_1108, 0x3456);
        MipsBus::write16(&mut bus, 0x1f80_1100, 0x1234);
        assert_eq!(MipsBus::read16(&mut bus, 0x1f80_1108), 0x3456);
        assert_eq!(MipsBus::read16(&mut bus, 0x1f80_1100), 0x1234);
    }

    #[test]
    fn memory_control_and_cache_configuration_registers_round_trip() {
        let bios = synthetic_bios();
        let mut bus = Ps1Bus::new_with_disc(&bios, None).unwrap();
        assert_eq!(MipsBus::read32(&mut bus, 0x1f80_1010), 0x0013_243f);
        assert_eq!(MipsBus::read32(&mut bus, 0x1f80_1060), DEFAULT_RAM_SIZE);

        MipsBus::write32(&mut bus, 0x1f80_1010, 0x00aa_55cc);
        MipsBus::write32(&mut bus, 0x1f80_1060, 0x0000_0888);
        MipsBus::write32(&mut bus, 0xfffe_0130, u32::MAX);
        assert_eq!(MipsBus::read32(&mut bus, 0x1f80_1010), 0x00aa_55cc);
        assert_eq!(MipsBus::read32(&mut bus, 0x1f80_1060), 0x0000_0888);
        assert_eq!(
            MipsBus::read32(&mut bus, 0xfffe_0130) & ((1 << 6) | (1 << 10)),
            0
        );
    }

    #[test]
    fn partial_mmio_stores_follow_ps1_bus_width_rules() {
        let bios = synthetic_bios();
        let mut bus = Ps1Bus::new_with_disc(&bios, None).unwrap();
        bus.write_word_le(0, 0xa441_00f0);
        bus.write_word_le(4, 0xa041_00f1);
        bus.write_word_le(8, 0xa441_0086);
        bus.write_word_le(12, 0xa041_0c00);
        bus.dma[0].block = 0x1122_3344;
        let mut cpu = MipsR3000::default();
        cpu.pc = 0xa000_0000;
        cpu.next_pc = 0xa000_0004;
        cpu.regs[1] = 0xaabb_ccdd;
        cpu.regs[2] = 0x1f80_1000;

        cpu.step(&mut bus);
        assert_eq!(bus.dpcr, 0xaabb_ccdd);
        cpu.step(&mut bus);
        assert_eq!(bus.dpcr, 0xbbcc_dd00);
        cpu.step(&mut bus);
        assert_eq!(bus.dma[0].block, 0xccdd_3344);
        cpu.step(&mut bus);
        assert_eq!(bus.spu.read16(0x1f80_1c00), 0xccdd);
    }

    #[test]
    fn swl_swr_follow_on_die_dma_and_spu_bus_strobes() {
        let bios = synthetic_bios();
        let mut bus = Ps1Bus::new_with_disc(&bios, None).unwrap();
        let source = 0xaabb_ccdd;

        bus.dpcr = 0x1122_3344;
        MipsBus::store_left(&mut bus, 0x1f80_10f1, source);
        assert_eq!(bus.dpcr, 0x0000_aabb);
        bus.dpcr = 0x1122_3344;
        MipsBus::store_right(&mut bus, 0x1f80_10f2, source);
        assert_eq!(bus.dpcr, 0xccdd_0000);

        bus.dma[0].block = 0x1122_3344;
        MipsBus::store_left(&mut bus, 0x1f80_1085, source);
        assert_eq!(bus.dma[0].block, 0x1122_aabb);
        bus.dma[0].block = 0x1122_3344;
        MipsBus::store_right(&mut bus, 0x1f80_1086, source);
        assert_eq!(bus.dma[0].block, 0xccdd_3344);

        bus.spu.write16(0x1f80_1c00, 0x1111);
        bus.spu.write16(0x1f80_1c02, 0x2222);
        MipsBus::store_left(&mut bus, 0x1f80_1c02, source);
        assert_eq!(bus.spu.read16(0x1f80_1c00), 0xbbcc);
        assert_eq!(bus.spu.read16(0x1f80_1c02), 0x2222);
        MipsBus::store_right(&mut bus, 0x1f80_1c02, source);
        assert_eq!(bus.spu.read16(0x1f80_1c00), 0xbbcc);
        assert_eq!(bus.spu.read16(0x1f80_1c02), 0xccdd);
        MipsBus::store_right(&mut bus, 0x1f80_1c03, 0x1234_5678);
        assert_eq!(bus.spu.read16(0x1f80_1c00), 0xbbcc);
        assert_eq!(bus.spu.read16(0x1f80_1c02), 0xccdd);
        MipsBus::store_left(&mut bus, 0x1f80_1c03, source);
        assert_eq!(bus.spu.read16(0x1f80_1c00), 0xccdd);
        assert_eq!(bus.spu.read16(0x1f80_1c02), 0xaabb);
    }

    #[test]
    fn dma_waits_for_dpcr_enable_before_starting_pending_channel() {
        let bios = synthetic_bios();
        let mut bus = Ps1Bus::new_with_disc(&bios, None).unwrap();
        bus.write_word_le(0x100, 0x1234_5678);
        bus.dma[6].base = 0x100;
        bus.dma[6].block = 1;
        bus.dma[6].control = 0x1100_0002;

        bus.service_dma();
        assert_eq!(bus.read_word_le(0x100), 0x1234_5678);
        assert_ne!(bus.dma[6].control & (1 << 24), 0);

        assert!(bus.write_mmio32(0x1f80_10f0, bus.dpcr | (1 << 27)));
        assert_eq!(bus.read_word_le(0x100), 0x00ff_ffff);
        assert_eq!(bus.dma[6].control & ((1 << 24) | (1 << 28)), 0);
    }

    #[test]
    fn dicr_completion_flags_and_master_irq_follow_hardware_gating() {
        let bios = synthetic_bios();
        let mut bus = Ps1Bus::new_with_disc(&bios, None).unwrap();
        bus.dpcr |= 1 << 27;
        bus.dma[6].base = 0x100;
        bus.dma[6].block = 1;
        bus.dma[6].control = 0x1100_0002;
        bus.service_dma();
        assert_eq!(bus.dicr & ((1 << 30) | (1 << 31)), 0);
        assert_eq!(bus.irq_status & (1 << 3), 0);

        bus.write_dicr((1 << 23) | (1 << 22));
        bus.dma[6].base = 0x104;
        bus.dma[6].block = 1;
        bus.dma[6].control = 0x1100_0002;
        bus.service_dma();
        assert_ne!(bus.dicr & (1 << 30), 0);
        assert_ne!(bus.dicr & (1 << 31), 0);
        assert_ne!(bus.irq_status & (1 << 3), 0);

        bus.write_dicr((1 << 23) | (1 << 22) | (1 << 30));
        assert_eq!(bus.dicr & ((1 << 30) | (1 << 31)), 0);
        assert_ne!(bus.irq_status & (1 << 3), 0);
        bus.irq_status &= !(1 << 3);
        bus.write_dicr(1 << 15);
        assert_ne!(bus.dicr & (1 << 31), 0);
        assert_ne!(bus.irq_status & (1 << 3), 0);
    }

    #[test]
    fn dma_bus_error_respects_ram_size_window() {
        let bios = synthetic_bios();
        let mut bus = Ps1Bus::new_with_disc(&bios, None).unwrap();
        assert_eq!(Ps1Bus::dma_count16(0), 0x1_0000);
        bus.dpcr |= 1 << 27;
        bus.ram_size = 0x0000_0888;
        bus.dma[6].base = 0x0020_0000;
        bus.dma[6].block = 1;
        bus.dma[6].control = 0x1100_0002;
        bus.service_dma();
        assert_ne!(bus.dicr & (1 << 15), 0);
        assert_ne!(bus.dicr & (1 << 31), 0);
        assert_ne!(bus.irq_status & (1 << 3), 0);
        assert_eq!(bus.dicr & (1 << 30), 0);
        assert_eq!(bus.dma[6].control & (1 << 24), 0);

        bus.write_dicr(0);
        bus.irq_status &= !(1 << 3);
        bus.ram_size = 0x0000_0b88;
        bus.dma[6].base = 0x0060_0000;
        bus.dma[6].block = 1;
        bus.dma[6].control = 0x1100_0002;
        bus.service_dma();
        assert_eq!(bus.dicr & (1 << 15), 0);
        assert_eq!(bus.read_word_le(0), 0x00ff_ffff);
    }

    #[test]
    fn mdec_out_dma_preserves_partial_request_block_until_more_output_arrives() {
        let bios = synthetic_bios();
        let mut bus = Ps1Bus::new_with_disc(&bios, None).unwrap();
        let decode_mono_block = |mdec: &mut Ps1Mdec| {
            mdec.write_data((1 << 29) | (1 << 27) | 1);
            mdec.write_data(0xfe00_0000);
        };
        bus.mdec.write_control(1 << 29);
        decode_mono_block(&mut bus.mdec);
        bus.dpcr |= 1 << 7;
        bus.dma[1].base = 0x400;
        bus.dma[1].block = (1 << 16) | 0x20;
        bus.dma[1].control = (1 << 9) | (1 << 24);

        assert!(!bus.run_dma(1));
        assert_ne!(bus.dma[1].control & (1 << 24), 0);
        assert_eq!(bus.dma[1].base, 0x440);
        assert_eq!(bus.mdec_dma_remaining[1], 16);
        assert_eq!(bus.read_word_le(0x400), 0x8080_8080);

        decode_mono_block(&mut bus.mdec);
        bus.service_dma();
        assert_eq!(bus.dma[1].control & (1 << 24), 0);
        assert_eq!(bus.dma[1].base, 0x480);
        assert_eq!(bus.mdec_dma_remaining[1], 0);
        assert_eq!(bus.dma[1].block >> 16, 0);
        assert_eq!(bus.read_word_le(0x47c), 0x8080_8080);
    }

    #[test]
    fn block_gpu_dma_stalls_on_full_fifo_and_resumes_from_madr() {
        let bios = synthetic_bios();
        let mut bus = Ps1Bus::new_with_disc(&bios, None).unwrap();
        bus.write_word_le(0x200, 0x0200_00ff);
        bus.write_word_le(0x204, 0);
        bus.write_word_le(0x208, (1 << 16) | 16);
        for index in 3..20u32 {
            bus.write_word_le(0x200 + index * 4, 0);
        }
        bus.gpu.gp1(0x0400_0002);
        bus.dpcr |= 1 << 11;
        bus.write_dicr((1 << 23) | (1 << 18));
        bus.dma[2].base = 0x200;
        bus.dma[2].block = 20;
        bus.dma[2].control = 1 | (1 << 24);

        assert!(!bus.run_dma(2));
        assert_ne!(bus.dma[2].control & (1 << 24), 0);
        assert_eq!(bus.gpu_dma.remaining_words, 1);
        assert_eq!(bus.dma[2].base, 0x24c);
        assert_eq!(bus.dicr & (1 << 26), 0);

        bus.gpu.tick_cpu_cycles(29);
        bus.service_dma();
        assert_eq!(bus.dma[2].control & (1 << 24), 0);
        assert_eq!(bus.gpu_dma.remaining_words, 0);
        assert_eq!(bus.dma[2].base, 0x250);
        assert_ne!(bus.dicr & (1 << 26), 0);
    }

    #[test]
    fn sync_one_gpu_dma_reacquires_request_between_blocks() {
        let bios = synthetic_bios();
        let mut bus = Ps1Bus::new_with_disc(&bios, None).unwrap();
        bus.write_word_le(0x300, 0x0200_00ff);
        bus.write_word_le(0x304, (4 << 16) | 8);
        bus.write_word_le(0x308, (1 << 16) | 1);
        bus.write_word_le(0x30c, 0x0200_ff00);
        bus.write_word_le(0x310, (8 << 16) | 8);
        bus.write_word_le(0x314, (1 << 16) | 1);
        bus.gpu.gp1(0x0400_0002);
        bus.dpcr |= 1 << 11;
        bus.write_dicr((1 << 23) | (1 << 18));
        bus.dma[2].base = 0x300;
        bus.dma[2].block = (2 << 16) | 3;
        bus.dma[2].control = 1 | (1 << 9) | (1 << 24);

        assert!(!bus.run_dma(2));
        assert_ne!(bus.dma[2].control & (1 << 24), 0);
        assert_eq!(bus.dma[2].block >> 16, 1);
        assert_eq!(bus.dma[2].base, 0x30c);
        assert_eq!(bus.dicr & (1 << 26), 0);
        assert_ne!(bus.gpu.vram_pixel(8, 4), 0);

        bus.service_dma();
        assert_eq!(bus.dma[2].base, 0x30c);
        assert_ne!(bus.dma[2].control & (1 << 24), 0);

        bus.gpu.tick_cpu_cycles(29);
        bus.service_dma();
        assert_eq!(bus.dma[2].control & (1 << 24), 0);
        assert_eq!(bus.dma[2].block >> 16, 0);
        assert_eq!(bus.dma[2].base, 0x318);
        assert_ne!(bus.gpu.vram_pixel(8, 8), 0);
        assert_ne!(bus.dicr & (1 << 26), 0);
        assert_ne!(bus.dicr & (1 << 31), 0);
    }

    #[test]
    fn linked_list_dma_feeds_gpu_commands() {
        let bios = synthetic_bios();
        let mut bus = Ps1Bus::new_with_disc(&bios, None).unwrap();
        bus.write_word_le(0x100, (3 << 24) | 0x00ff_ffff);
        bus.write_word_le(0x104, 0x0200_ff00);
        bus.write_word_le(0x108, (4 << 16) | 8);
        bus.write_word_le(0x10c, (2 << 16) | 16);
        bus.gpu.gp1(0x0400_0002);
        bus.dpcr |= 1 << 11;
        bus.dma[2].base = 0x100;
        bus.dma[2].control = 1 | (2 << 9) | (1 << 24);
        assert!(bus.run_dma(2));
        assert_ne!(bus.gpu.vram_pixel(8, 4), 0);
        assert_eq!(bus.dma[2].control & (1 << 24), 0);
    }

    #[test]
    fn linked_list_dma_stalls_on_full_gpu_fifo_and_resumes() {
        let bios = synthetic_bios();
        let mut bus = Ps1Bus::new_with_disc(&bios, None).unwrap();
        bus.write_word_le(0x100, (20 << 24) | 0x00ff_ffff);
        bus.write_word_le(0x104, 0x0200_00ff);
        bus.write_word_le(0x108, 0);
        bus.write_word_le(0x10c, (1 << 16) | 16);
        for index in 3..20u32 {
            bus.write_word_le(0x104 + index * 4, 0);
        }
        bus.gpu.gp1(0x0400_0002);
        bus.dpcr |= 1 << 11;
        bus.write_dicr((1 << 23) | (1 << 18));
        bus.dma[2].base = 0x100;
        bus.dma[2].control = 1 | (2 << 9) | (1 << 24);

        assert!(!bus.run_dma(2));
        assert_ne!(bus.dma[2].control & (1 << 24), 0);
        assert_eq!(bus.gpu_dma.linked_remaining, 1);
        assert_eq!(bus.dma[2].base, 0x150);
        assert_eq!(bus.dicr & (1 << 26), 0);

        bus.gpu.tick_cpu_cycles(29);
        bus.service_dma();
        assert_eq!(bus.dma[2].control & (1 << 24), 0);
        assert_eq!(bus.gpu_dma.linked_remaining, 0);
        assert_eq!(bus.dma[2].base, 0x00ff_fffc);
        assert_ne!(bus.dicr & (1 << 26), 0);
        assert_ne!(bus.dicr & (1 << 31), 0);
    }

    #[test]
    fn mdec_dma_round_trip_moves_decoded_macroblock_through_ram() {
        let bios = synthetic_bios();
        let mut bus = Ps1Bus::new_with_disc(&bios, None).unwrap();
        assert!(bus.write_mmio32(0x1f80_1824, (1 << 30) | (1 << 29)));
        assert!(bus.write_mmio32(0x1f80_1820, (1 << 29) | (1 << 27) | 1));
        assert!(bus.write_mmio32(0x1f80_10f0, bus.dpcr | (1 << 3) | (1 << 7)));
        assert!(bus.write_mmio32(0x1f80_10f4, (1 << 23) | (1 << 16) | (1 << 17)));
        bus.write_word_le(0x1000, 0xfe00_0010);
        assert!(bus.write_mmio32(0x1f80_1090, 0x2000));
        assert!(bus.write_mmio32(0x1f80_1094, (1 << 16) | 16));
        assert!(bus.write_mmio32(0x1f80_1098, (1 << 9) | (1 << 24)));
        assert_ne!(bus.dma[1].control & (1 << 24), 0);

        assert!(bus.write_mmio32(0x1f80_1080, 0x1000));
        assert!(bus.write_mmio32(0x1f80_1084, (1 << 16) | 1));
        assert!(bus.write_mmio32(0x1f80_1088, 1 | (1 << 9) | (1 << 24)));
        assert_eq!(bus.read_word_le(0x2000), 0x8080_8080);
        assert!(!bus.mdec.can_dma_out());
        assert_eq!(bus.dma[0].base, 0x1004);
        assert_eq!(bus.dma[1].base, 0x2040);
        assert_ne!(bus.dicr & ((1 << 24) | (1 << 25)), 0);
    }

    #[test]
    fn spu_dma_moves_words_between_main_and_sound_ram() {
        let bios = synthetic_bios();
        let mut bus = Ps1Bus::new_with_disc(&bios, None).unwrap();
        bus.write_word_le(0x0200, 0xdead_beef);
        bus.spu.write16(0x1f80_1da6, 0x0100);
        bus.dma[4].base = 0x0200;
        bus.dma[4].block = 1;
        bus.dma[4].control = 1;
        bus.run_spu_dma();
        bus.spu.write16(0x1f80_1da6, 0x0100);
        assert_eq!(bus.spu.transfer_read_halfword(), 0xbeef);
        assert_eq!(bus.spu.transfer_read_halfword(), 0xdead);

        bus.spu.write16(0x1f80_1da6, 0x0100);
        bus.write_word_le(0x0300, 0);
        bus.dma[4].base = 0x0300;
        bus.dma[4].block = 1;
        bus.dma[4].control = 0;
        bus.run_spu_dma();
        assert_eq!(bus.read_word_le(0x0300), 0xdead_beef);
    }

    #[test]
    fn spu_current_volume_registers_are_visible_through_cpu_mmio() {
        let bios = synthetic_bios();
        let mut bus = Ps1Bus::new_with_disc(&bios, None).unwrap();
        bus.spu.write16(0x1f80_1c00, 0x2000);
        bus.spu.write16(0x1f80_1c02, 0x1000);
        assert_eq!(MipsBus::read16(&mut bus, 0x1f80_1e00), 0x4000);
        assert_eq!(MipsBus::read16(&mut bus, 0x1f80_1e02), 0x2000);
        assert_eq!(bus.read_mmio32(0x1f80_1e00), Some(0x2000_4000));
    }

    #[test]
    fn cdrom_dma_waits_for_bfrd_and_preserves_burst_madr() {
        let bios = synthetic_bios();
        let mut disc = vec![0u8; 4 * 2048];
        for sector in 0..4usize {
            disc[sector * 2048] = sector as u8;
            disc[sector * 2048 + 1] = 0xa5;
        }
        let mut bus = Ps1Bus::new_with_disc(&bios, Some(ResourceBlob::from_bytes(&disc))).unwrap();

        bus.cdrom.write_register(0, 1);
        bus.cdrom.write_register(2, 0x1f);
        bus.cdrom.write_register(0, 0);
        for value in [0x00, 0x02, 0x01] {
            bus.cdrom.write_register(2, value);
        }
        bus.cdrom.write_register(1, 0x02);
        bus.cdrom.write_register(0, 1);
        bus.cdrom.write_register(3, 0x1f);
        bus.cdrom.write_register(0, 0);
        bus.cdrom.write_register(1, 0x06);

        assert!(bus.write_mmio32(0x1f80_10f0, bus.dpcr | (1 << 15)));
        assert!(bus.write_mmio32(0x1f80_10f4, (1 << 23) | (1 << 19)));
        assert!(bus.write_mmio32(0x1f80_10b0, 0x1000));
        assert!(bus.write_mmio32(0x1f80_10b4, 0x0000_0200));
        assert!(bus.write_mmio32(0x1f80_10b8, 0x1100_0000));
        assert_ne!(bus.dma[3].control & (1 << 24), 0);

        bus.tick(1);
        assert_ne!(bus.dma[3].control & (1 << 24), 0);
        bus.write_memory8(0x1f80_1803, 0x80);
        assert_eq!(bus.read_word_le(0x1000), 0x0000_a501);
        assert_eq!(bus.dma[3].control & (1 << 24), 0);
        assert_eq!(bus.dma[3].base, 0x1000);
        assert_ne!(bus.dicr & (1 << 27), 0);
        assert_ne!(bus.dicr & (1 << 31), 0);
    }

    #[test]
    fn cdda_sector_reaches_spu_mixer_through_drive_and_bus() {
        let bios = synthetic_bios();
        let mut raw = vec![0u8; 2 * 2352];
        for frame in raw.as_chunks_mut::<4>().0 {
            frame[..2].copy_from_slice(&0x3000i16.to_le_bytes());
            frame[2..].copy_from_slice(&(-0x1800i16).to_le_bytes());
        }
        let mut bus = Ps1Bus::new_with_disc(&bios, Some(ResourceBlob::from_bytes(&raw))).unwrap();
        bus.spu.write16(0x1f80_1d80, 0x3fff);
        bus.spu.write16(0x1f80_1d82, 0x3fff);
        bus.spu.write16(0x1f80_1db0, 0x7fff);
        bus.spu.write16(0x1f80_1db2, 0x7fff);
        bus.spu.write16(0x1f80_1daa, 0xc001);
        bus.cdrom.write_register(1, 0x03);
        bus.tick(1);
        bus.tick(800);
        let mixed = bus.spu.drain_output();
        assert!(mixed.len() >= 2);
        assert!(mixed[0] > 0.30);
        assert!(mixed[1] < -0.10);
    }

    #[test]
    fn xa_sector_reaches_spu_mixer_through_drive_and_bus() {
        let bios = synthetic_bios();
        let mut bus = Ps1Bus::new_with_disc(&bios, Some(synthetic_xa_disc())).unwrap();
        bus.spu.write16(0x1f80_1d80, 0x3fff);
        bus.spu.write16(0x1f80_1d82, 0x3fff);
        bus.spu.write16(0x1f80_1db0, 0x7fff);
        bus.spu.write16(0x1f80_1db2, 0x7fff);
        bus.spu.write16(0x1f80_1daa, 0xc001);
        bus.cdrom.write_register(2, 0x40);
        bus.cdrom.write_register(1, 0x0e);
        bus.cdrom.write_register(0, 1);
        bus.cdrom.write_register(3, 0x1f);
        bus.cdrom.write_register(0, 0);
        bus.cdrom.write_register(1, 0x06);
        bus.tick(1);
        bus.tick(40_000);
        let mixed = bus.spu.drain_output();
        assert!(mixed.len() >= 40);
        assert!(mixed.iter().any(|sample| sample.abs() > 0.001));
        assert_eq!(bus.cdrom.dma_read_word(), 0);
    }

    #[test]
    fn digital_pad_serial_protocol_reports_active_low_buttons() {
        let bios = synthetic_bios();
        let mut bus = Ps1Bus::new_with_disc(&bios, None).unwrap();
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
    fn dualshock_config_enables_analog_axes_and_rumble_mapping() {
        let bios = synthetic_bios();
        let mut bus = Ps1Bus::new_with_disc(&bios, None).unwrap();
        let mut input = InputState::default();
        input.buttons[0] = L3 | R3;
        input.axes[0][AXIS_RIGHT_X] = i16::MIN;
        input.axes[0][AXIS_RIGHT_Y] = i16::MAX;
        input.axes[0][AXIS_LEFT_X] = 0;
        input.axes[0][AXIS_LEFT_Y] = -16_384;
        bus.set_inputs(&input);

        assert_eq!(
            sio_transaction(&mut bus, &[0x01, 0x42, 0, 0, 0]),
            [0xff, 0x41, 0x5a, 0xf9, 0xff]
        );
        assert_eq!(
            sio_transaction(&mut bus, &[0x01, 0x43, 0, 1, 0]),
            [0xff, 0x41, 0x5a, 0xf9, 0xff]
        );
        assert!(bus.pad_config[0]);

        assert_eq!(
            sio_transaction(&mut bus, &[0x01, 0x44, 0, 1, 3, 0, 0, 0, 0]),
            [0xff, 0xf3, 0x5a, 0, 0, 0, 0, 0, 0]
        );
        assert!(bus.pad_analog[0]);
        assert!(bus.pad_mode_locked[0]);

        assert_eq!(
            sio_transaction(
                &mut bus,
                &[0x01, 0x4d, 0, 0x00, 0x01, 0xff, 0xff, 0xff, 0xff],
            ),
            [0xff, 0xf3, 0x5a, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff]
        );
        assert_eq!(bus.pad_rumble_map[0], [0, 1, 0xff, 0xff, 0xff, 0xff]);

        assert_eq!(
            sio_transaction(&mut bus, &[0x01, 0x45, 0, 0, 0, 0, 0, 0, 0]),
            [0xff, 0xf3, 0x5a, 0x01, 0x02, 0x01, 0x02, 0x01, 0]
        );
        assert_eq!(
            sio_transaction(&mut bus, &[0x01, 0x43, 0, 0, 0, 0, 0, 0, 0]),
            [0xff, 0xf3, 0x5a, 0, 0, 0, 0, 0, 0]
        );
        assert!(!bus.pad_config[0]);

        assert_eq!(
            sio_transaction(&mut bus, &[0x01, 0x42, 0, 1, 0x80, 0, 0, 0, 0]),
            [0xff, 0x73, 0x5a, 0xf9, 0xff, 0x00, 0xff, 0x80, 0x40]
        );
        assert!(bus.pad_rumble_small[0]);
        assert_eq!(bus.pad_rumble_large[0], 0x80);
    }

    #[test]
    fn dualshock_config_capability_commands_match_reference_responses() {
        let bios = synthetic_bios();
        let mut bus = Ps1Bus::new_with_disc(&bios, None).unwrap();
        sio_transaction(&mut bus, &[0x01, 0x43, 0, 1, 0]);
        assert!(bus.pad_config[0]);

        assert_eq!(
            sio_transaction(&mut bus, &[0x01, 0x46, 0, 0, 0, 0, 0, 0, 0]),
            [0xff, 0xf3, 0x5a, 0, 0, 1, 2, 0, 10]
        );
        assert_eq!(
            sio_transaction(&mut bus, &[0x01, 0x46, 0, 1, 0, 0, 0, 0, 0]),
            [0xff, 0xf3, 0x5a, 0, 0, 1, 1, 1, 20]
        );
        assert_eq!(
            sio_transaction(&mut bus, &[0x01, 0x47, 0, 0, 0, 0, 0, 0, 0]),
            [0xff, 0xf3, 0x5a, 0, 0, 2, 0, 1, 0]
        );
        assert_eq!(
            sio_transaction(&mut bus, &[0x01, 0x48, 0, 0, 0, 0, 0, 0, 0]),
            [0xff, 0xf3, 0x5a, 0, 0, 0, 0, 1, 0]
        );
        assert_eq!(
            sio_transaction(&mut bus, &[0x01, 0x4c, 0, 0, 0, 0, 0, 0, 0]),
            [0xff, 0xf3, 0x5a, 0, 0, 0, 4, 0, 0]
        );
    }

    #[test]
    fn formatted_memory_card_has_valid_header_and_free_directory() {
        let bios = synthetic_bios();
        let bus = Ps1Bus::new_with_disc(&bios, None).unwrap();
        assert_eq!(&bus.memory_card[..2], b"MC");
        assert_eq!(bus.memory_card[127], 0x0e);
        for sector in 1..=15usize {
            let frame = &bus.memory_card[sector * 128..(sector + 1) * 128];
            assert_eq!(&frame[..4], &0x0000_00a0u32.to_le_bytes());
            assert_eq!(&frame[8..10], &0xffffu16.to_le_bytes());
            assert_eq!(
                frame[..127].iter().fold(0u8, |sum, byte| sum ^ byte),
                frame[127]
            );
        }
    }

    #[test]
    fn memory_card_sio_write_then_read_round_trips_sector_and_checksum() {
        let bios = synthetic_bios();
        let mut bus = Ps1Bus::new_with_disc(&bios, None).unwrap();
        let sector = 0x012u16;
        let msb = (sector >> 8) as u8;
        let lsb = sector as u8;
        let data = std::array::from_fn::<_, 128, _>(|index| (index as u8).wrapping_mul(3));
        let checksum = data.iter().fold(msb ^ lsb, |sum, byte| sum ^ byte);

        assert_eq!(sio_exchange(&mut bus, 0x81), 0xff);
        assert_eq!(sio_exchange(&mut bus, 0x57), 0x08);
        assert_eq!(sio_exchange(&mut bus, 0x00), 0x5a);
        assert_eq!(sio_exchange(&mut bus, 0x00), 0x5d);
        assert_eq!(sio_exchange(&mut bus, msb), 0x00);
        assert_eq!(sio_exchange(&mut bus, lsb), msb);
        for (index, byte) in data.iter().copied().enumerate() {
            let response = sio_exchange(&mut bus, byte);
            assert_eq!(response, if index == 0 { lsb } else { data[index - 1] });
        }
        assert_eq!(sio_exchange(&mut bus, checksum), data[127]);
        assert_eq!(sio_exchange(&mut bus, 0x00), 0x5c);
        assert_eq!(sio_exchange(&mut bus, 0x00), 0x5d);
        assert_eq!(sio_exchange(&mut bus, 0x00), 0x47);
        assert_eq!(bus.card_flag & 0x08, 0);

        assert_eq!(sio_exchange(&mut bus, 0x81), 0xff);
        assert_eq!(sio_exchange(&mut bus, 0x52), 0x00);
        assert_eq!(sio_exchange(&mut bus, 0x00), 0x5a);
        assert_eq!(sio_exchange(&mut bus, 0x00), 0x5d);
        assert_eq!(sio_exchange(&mut bus, msb), 0x00);
        assert_eq!(sio_exchange(&mut bus, lsb), msb);
        assert_eq!(sio_exchange(&mut bus, 0x00), 0x5c);
        assert_eq!(sio_exchange(&mut bus, 0x00), 0x5d);
        assert_eq!(sio_exchange(&mut bus, 0x00), msb);
        assert_eq!(sio_exchange(&mut bus, 0x00), lsb);
        for expected in data {
            assert_eq!(sio_exchange(&mut bus, 0x00), expected);
        }
        assert_eq!(sio_exchange(&mut bus, 0x00), checksum);
        assert_eq!(sio_exchange(&mut bus, 0x00), 0x47);
    }

    #[test]
    fn memory_card_rejects_bad_checksum_and_slot_two_is_absent() {
        let bios = synthetic_bios();
        let mut bus = Ps1Bus::new_with_disc(&bios, None).unwrap();
        let before = bus.memory_card[128..256].to_vec();
        assert_eq!(sio_exchange(&mut bus, 0x81), 0xff);
        assert_eq!(sio_exchange(&mut bus, 0x57), 0x08);
        assert_eq!(sio_exchange(&mut bus, 0), 0x5a);
        assert_eq!(sio_exchange(&mut bus, 0), 0x5d);
        assert_eq!(sio_exchange(&mut bus, 0), 0);
        assert_eq!(sio_exchange(&mut bus, 1), 0);
        for _ in 0..128 {
            sio_exchange(&mut bus, 0x5a);
        }
        sio_exchange(&mut bus, 0xff);
        assert_eq!(sio_exchange(&mut bus, 0), 0x5c);
        assert_eq!(sio_exchange(&mut bus, 0), 0x5d);
        assert_eq!(sio_exchange(&mut bus, 0), 0x4e);
        assert_eq!(&bus.memory_card[128..256], before);

        bus.sio_control = 1 << 13;
        bus.irq_status &= !(1 << 7);
        assert_eq!(sio_exchange(&mut bus, 0x81), 0xff);
        assert_eq!(bus.sio_phase, 0);
        assert_eq!(bus.irq_status & (1 << 7), 0);
    }

    #[test]
    fn sio_status_and_port_select_reflect_rx_fifo_and_second_pad() {
        let bios = synthetic_bios();
        let mut bus = Ps1Bus::new_with_disc(&bios, None).unwrap();
        bus.pad_buttons[1] = 0x1234;
        assert_eq!(bus.sio_status() & (1 << 1), 0);
        bus.sio_write(0x01);
        assert_ne!(bus.sio_status() & (1 << 1), 0);
        assert_eq!(bus.sio_rx.pop_front(), Some(0xff));
        assert_eq!(bus.sio_status() & (1 << 1), 0);
        bus.reset_sio_transaction();
        bus.sio_control = 1 << 13;
        for byte in [0x01, 0x42, 0, 0, 0] {
            bus.sio_write(byte);
        }
        let responses: Vec<_> = bus.sio_rx.drain(..).collect();
        assert_eq!(&responses[1..], &[0x41, 0x5a, 0x34, 0x12]);
    }

    #[test]
    fn disc_controller_state_round_trip_keeps_loaded_media() {
        let bios = synthetic_bios();
        let disc = ResourceBlob::from_bytes(&vec![0x5a; 4 * 2048]);
        let mut first = PlayStationMachine::from_bios_and_disc(&bios, Some(disc.clone())).unwrap();
        assert!(first.has_disc());
        first.bus.cdrom.write_register(1, 0x06);
        first.bus.cdrom.tick_cpu_cycles(1);
        let saved = first.save_state().unwrap();
        let mut second = PlayStationMachine::from_bios_and_disc(&bios, Some(disc)).unwrap();
        second.load_state(&saved).unwrap();
        assert!(second.has_disc());
        assert_eq!(second.save_state().unwrap(), saved);
    }

    #[test]
    fn root_timer_target_reset_irq_and_status_flags_follow_mode() {
        let mut timer = Ps1Timer::default();
        timer.write_target(3);
        timer.write_mode((1 << 3) | (1 << 4) | (1 << 6));
        assert!(!timer.tick(4));
        assert_eq!(timer.counter, 2);
        assert!(timer.tick(1));
        assert_eq!(timer.counter, 0);
        let mode = timer.read_mode();
        assert_ne!(mode & (1 << 11), 0);
        assert_eq!(timer.read_mode() & (1 << 11), 0);
        assert!(!timer.tick(1));
        assert!(timer.tick(3));
    }

    #[test]
    fn timer_mmio_maps_all_three_counters_and_routes_target_irq() {
        let bios = synthetic_bios();
        let mut bus = Ps1Bus::new_with_disc(&bios, None).unwrap();
        assert!(bus.write_mmio32(0x1f80_1128, 0x1234));
        assert_eq!(bus.read_mmio32(0x1f80_1128), Some(0x1234));

        assert!(bus.write_mmio32(0x1f80_1108, 3));
        assert!(bus.write_mmio32(0x1f80_1104, (1 << 3) | (1 << 4) | (1 << 6)));
        bus.tick(5);
        assert_ne!(bus.irq_status & (1 << 4), 0);
        assert_eq!(bus.read_mmio32(0x1f80_1100), Some(0));
        assert_ne!(bus.read_mmio32(0x1f80_1104).unwrap() & (1 << 11), 0);
        assert_eq!(bus.read_mmio32(0x1f80_1104).unwrap() & (1 << 11), 0);
    }

    #[test]
    fn timer_two_system_clock_divide_by_eight_preserves_fractional_phase() {
        let bios = synthetic_bios();
        let mut bus = Ps1Bus::new_with_disc(&bios, None).unwrap();
        assert!(bus.write_mmio32(0x1f80_1124, 2 << 8));
        bus.tick(16);
        assert_eq!(bus.read_mmio32(0x1f80_1120), Some(0));
        bus.tick(7);
        assert_eq!(bus.read_mmio32(0x1f80_1120), Some(0));
        bus.tick(1);
        assert_eq!(bus.read_mmio32(0x1f80_1120), Some(1));
    }

    #[test]
    fn timer_one_hblank_clock_uses_gpu_blank_edges() {
        let bios = synthetic_bios();
        let mut bus = Ps1Bus::new_with_disc(&bios, None).unwrap();
        bus.gpu.gp1(0x0600_1000);
        assert!(bus.write_mmio32(0x1f80_1114, 1 << 8));
        for _ in 0..10_000 {
            bus.tick(1);
            if bus.timers[1].counter != 0 {
                break;
            }
        }
        assert_ne!(bus.timers[1].counter, 0);
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
        machine.bus.gte.write_data(0, 0x1234_5678);
        machine.bus.gte.write_control(5, 0x89ab_cdef);
        machine.bus.memory_control[4] = 0x00aa_55cc;
        machine.bus.ram_size = 0x0888;
        machine.bus.cache_control = (1 << 11) | (1 << 8);
        machine.bus.write_word_le(0x400, 0x2401_0077);
        assert_eq!(machine.bus.fetch_instruction32(0x8000_0400), 0x2401_0077);
        machine.bus.write_mmio32(0x1f80_1128, 0x2345);
        machine.bus.write_mmio32(0x1f80_1124, 2 << 8);
        machine.bus.tick(40);
        sio_transaction(&mut machine.bus, &[0x01, 0x43, 0, 1, 0]);
        sio_transaction(&mut machine.bus, &[0x01, 0x44, 0, 1, 3, 0, 0, 0, 0]);
        sio_transaction(
            &mut machine.bus,
            &[0x01, 0x4d, 0, 0, 1, 0xff, 0xff, 0xff, 0xff],
        );
        sio_transaction(&mut machine.bus, &[0x01, 0x43, 0, 0, 0, 0, 0, 0, 0]);
        machine.bus.sio_write(0x01);
        let timer_counter = machine.bus.timers[2].counter;
        let timer_phase = machine.bus.timers[2].clock_phase;
        let saved = machine.save_state().unwrap();
        machine.bus.ram[0x1234] = 0;
        machine.bus.timers[2] = Ps1Timer::default();
        machine.load_state(&saved).unwrap();
        assert_eq!(machine.bus.ram[0x1234], 0xa5);
        assert_eq!(machine.bus.timers[2].target, 0x2345);
        assert_eq!(machine.bus.timers[2].counter, timer_counter);
        assert_eq!(machine.bus.timers[2].clock_phase, timer_phase);
        assert_eq!(machine.bus.gte.read_data(0), 0x1234_5678);
        assert_eq!(machine.bus.gte.read_control(5), 0x89ab_cdef);
        assert_eq!(machine.bus.memory_control[4], 0x00aa_55cc);
        assert_eq!(machine.bus.ram_size, 0x0888);
        assert_eq!(machine.bus.cache_control, (1 << 11) | (1 << 8));
        let (cache_line, cache_word, cache_tag) = Ps1Bus::icache_location(0x400);
        assert_eq!(machine.bus.icache_words[cache_word], 0x2401_0077);
        assert_eq!(machine.bus.icache_tags[cache_line], cache_tag);
        assert_ne!(machine.bus.icache_valid[cache_line], 0);
        assert!(machine.bus.pad_analog[0]);
        assert!(machine.bus.pad_mode_locked[0]);
        assert_eq!(
            machine.bus.pad_rumble_map[0],
            [0, 1, 0xff, 0xff, 0xff, 0xff]
        );
        assert_eq!(machine.bus.sio_phase, 1);
        assert_eq!(machine.bus.sio_device, 0x01);
        assert_eq!(machine.bus.sio_rx.pop_front(), Some(0xff));
        assert_eq!(sio_exchange(&mut machine.bus, 0x42), 0x73);
        machine.bus.reset_sio_transaction();
        let mut restored = vec![0; MEMORY_CARD_SIZE];
        machine
            .read_persistent(ResourceKind::Storage, 0, &mut restored)
            .unwrap();
        assert_eq!(restored, card);
        assert_ne!(machine.bus.gpu.vram_pixel(0, 0), 0);
    }
}
