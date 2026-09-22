use std::collections::VecDeque;

use crate::cpu68000::Bus68000;
use crate::cpu_sh2::{Sh2, Sh2Bus, Sh2Divu, Sh2Dmac, Sh2Frt, Sh2Intc, Sh2Sci, Sh2Wdt};
use crate::kernel::{AudioBuffer, InputState, VideoBuffer};
use crate::machine::Machine;
use crate::platform::PlatformId;
use crate::resources::ResourceKind;
use crate::state::{StateReader, StateWriter};

use super::genesis::{GenesisBus, GenesisMachine, FRAME_RATE, M68K_HZ};

const SH2_HZ: u64 = 23_011_360;
const AUDIO_RATE: u64 = 48_000;
const STATE_VERSION: u32 = 14;
const SDRAM_SIZE: usize = 0x40000;
const FRAMEBUFFER_SIZE: usize = 0x20000;
const PWM_FIFO_DEPTH: usize = 3;
const WIDTH: u32 = 320;
const HEIGHT: u32 = 224;
const MAIN_CYCLES_PER_LINE: u64 = 488;
const IRQ_PWM: u8 = 1 << 0;
const IRQ_CMD: u8 = 1 << 1;
const IRQ_H: u8 = 1 << 2;
const IRQ_V: u8 = 1 << 3;
const IRQ_VRES: u8 = 1 << 4;

#[derive(Clone, Copy, Debug)]
struct MarsHeader {
    source: u32,
    destination: u32,
    size: u32,
    master_entry: u32,
    slave_entry: u32,
    master_vbr: u32,
    slave_vbr: u32,
}

impl MarsHeader {
    fn be32(image: &[u8], offset: usize) -> Result<u32, String> {
        let bytes = image
            .get(offset..offset + 4)
            .ok_or_else(|| "32X cartridge MARS header is truncated".to_string())?;
        Ok(u32::from_be_bytes(bytes.try_into().unwrap()))
    }

    fn parse(image: &[u8]) -> Result<(Self, std::ops::Range<usize>), String> {
        if image.len() < 0x3f0 {
            return Err("32X cartridge is too small to contain the MARS module header".into());
        }
        let source = Self::be32(image, 0x3d4)?;
        let destination = Self::be32(image, 0x3d8)?;
        let size = Self::be32(image, 0x3dc)?;
        let master_entry = Self::be32(image, 0x3e0)?;
        let slave_entry = Self::be32(image, 0x3e4)?;
        let master_vbr = Self::be32(image, 0x3e8)?;
        let slave_vbr = Self::be32(image, 0x3ec)?;
        let source_end = source
            .checked_add(size)
            .ok_or_else(|| "32X MARS source range overflows".to_string())?;
        if source_end as usize > image.len() {
            return Err("32X MARS source range extends past the cartridge image".into());
        }
        let destination_offset = destination.checked_sub(0x0600_0000).unwrap_or(destination);
        let destination_end = destination_offset
            .checked_add(size)
            .ok_or_else(|| "32X MARS destination range overflows".to_string())?;
        if destination_end as usize > SDRAM_SIZE {
            return Err("32X MARS program does not fit the 256 KiB SDRAM".into());
        }
        if [master_entry, slave_entry, master_vbr, slave_vbr]
            .iter()
            .any(|value| value & 1 != 0)
        {
            return Err("32X SH-2 entry and vector addresses must be word aligned".into());
        }
        Ok((
            Self {
                source,
                destination: destination_offset,
                size,
                master_entry,
                slave_entry,
                master_vbr,
                slave_vbr,
            },
            source as usize..source_end as usize,
        ))
    }
}

impl Sh2Dmac {
    fn channel0_dreq_ready(&self) -> bool {
        let channel = self.channels[0];
        channel.sar == 0x2000_4012
            && channel.chcr & 0xfff8 == 0x44e0
            && channel.chcr & 0x0001 != 0
            && channel.chcr & 0x0002 == 0
            && self.drcr[0] == 0
    }

    fn channel0_transfer_complete(&mut self) {
        self.channels[0].tcr = 0;
        self.channels[0].chcr |= 0x0002;
    }

    fn channel1_pwm_transfer_size(&self) -> Option<u32> {
        let channel = self.channels[1];
        if channel.chcr & 0xc000 != 0
            || channel.chcr & 0x00f8 != 0x00e0
            || channel.chcr & 0x0001 == 0
            || channel.chcr & 0x0002 != 0
            || self.drcr[1] != 0
            || ((channel.chcr >> 12) & 3) == 3
        {
            return None;
        }
        match channel.chcr & 0x0f00 {
            0x0400 => Some(2),
            0x0800 => Some(4),
            _ => None,
        }
    }

    fn channel1_transfer_complete(&mut self) {
        self.channels[1].tcr = 0;
        self.channels[1].chcr |= 0x0002;
    }
}

#[derive(Clone, Copy, Default)]
struct PwmChannel {
    last: u16,
}

struct MarsPwm {
    control: u16,
    cycle: u16,
    left: VecDeque<u16>,
    right: VecDeque<u16>,
    channels: [PwmChannel; 2],
    chip_phase: u64,
    timer_count: u8,
    sample_phase: u64,
    samples: Vec<(f32, f32)>,
}

impl Default for MarsPwm {
    fn default() -> Self {
        Self {
            control: 0,
            cycle: 0x100,
            left: VecDeque::with_capacity(PWM_FIFO_DEPTH),
            right: VecDeque::with_capacity(PWM_FIFO_DEPTH),
            channels: [PwmChannel::default(); 2],
            chip_phase: 0,
            timer_count: 0,
            sample_phase: 0,
            samples: Vec::with_capacity(1024),
        }
    }
}

impl MarsPwm {
    fn begin_frame(&mut self) {
        self.samples.clear();
    }

    fn fifo_status(queue: &VecDeque<u16>) -> u16 {
        (if queue.len() >= PWM_FIFO_DEPTH {
            1 << 15
        } else {
            0
        }) | (if queue.is_empty() { 1 << 14 } else { 0 })
    }

    fn read(&self, offset: u32) -> u16 {
        match offset & 0x0e {
            0x00 => self.control,
            0x02 => self.cycle,
            0x04 => Self::fifo_status(&self.left),
            0x06 => Self::fifo_status(&self.right),
            0x08 => Self::fifo_status(&self.left) | Self::fifo_status(&self.right),
            _ => 0,
        }
    }

    fn push(queue: &mut VecDeque<u16>, value: u16) {
        if queue.len() < PWM_FIFO_DEPTH {
            queue.push_back(value & 0x0fff);
        }
    }

    fn can_accept(&self, offset: u32) -> bool {
        match offset & 0x0e {
            0x04 => self.left.len() < PWM_FIFO_DEPTH,
            0x06 => self.right.len() < PWM_FIFO_DEPTH,
            0x08 => self.left.len() < PWM_FIFO_DEPTH && self.right.len() < PWM_FIFO_DEPTH,
            _ => false,
        }
    }

    fn write(&mut self, offset: u32, value: u16) {
        match offset & 0x0e {
            0x00 => self.control = value,
            0x02 => self.cycle = value & 0x0fff,
            0x04 => Self::push(&mut self.left, value),
            0x06 => Self::push(&mut self.right, value),
            0x08 => {
                Self::push(&mut self.left, value);
                Self::push(&mut self.right, value);
            }
            _ => {}
        }
    }

    fn clock_once(&mut self) {
        if let Some(value) = self.left.pop_front() {
            self.channels[0].last = value;
        }
        if let Some(value) = self.right.pop_front() {
            self.channels[1].last = value;
        }
    }

    fn period_cycles(&self) -> u64 {
        let raw = self.cycle & 0x0fff;
        if raw == 0 {
            4095
        } else {
            u64::from(raw - 1)
        }
    }

    fn current_sample(&self) -> (f32, f32) {
        let period_cycles = self.period_cycles();
        if period_cycles == 0 {
            return (0.0, 0.0);
        }
        let period = period_cycles as f32;
        let convert = |value: u16| {
            let raw = value & 0x0fff;
            let width = if raw == 0 { 4095.0 } else { f32::from(raw - 1) };
            ((width.min(period - 1.0) / (period - 1.0).max(1.0)) * 2.0 - 1.0) * 0.35
        };
        let left_mode = self.control & 3;
        let right_mode = (self.control >> 2) & 3;
        let left = match left_mode {
            1 => convert(self.channels[0].last),
            2 => convert(self.channels[1].last),
            _ => 0.0,
        };
        let right = match right_mode {
            1 => convert(self.channels[1].last),
            2 => convert(self.channels[0].last),
            _ => 0.0,
        };
        (left, right)
    }

    fn tick(&mut self, cycles: u32) -> u32 {
        let mut timer_events = 0u32;
        let period = self.period_cycles();
        for _ in 0..cycles {
            if self.control & 0x000f != 0 && period != 0 {
                self.chip_phase += 1;
                if self.chip_phase >= period {
                    self.chip_phase = 0;
                    self.clock_once();
                    self.timer_count = self.timer_count.wrapping_add(1);
                    let tm = ((self.control >> 8) & 0x0f) as u8;
                    let interval = if tm == 0 { 16 } else { tm };
                    if self.timer_count >= interval {
                        self.timer_count = 0;
                        timer_events = timer_events.saturating_add(1);
                    }
                }
            }
            self.sample_phase += AUDIO_RATE;
            if self.sample_phase >= SH2_HZ {
                self.sample_phase -= SH2_HZ;
                self.samples.push(self.current_sample());
            }
        }
        timer_events
    }

    fn save(&self, out: &mut StateWriter) {
        out.u16(self.control);
        out.u16(self.cycle);
        out.u64(self.chip_phase);
        out.u8(self.timer_count);
        out.u64(self.sample_phase);
        out.u16(self.channels[0].last);
        out.u16(self.channels[1].last);
        out.u8(self.left.len() as u8);
        for value in &self.left {
            out.u16(*value);
        }
        out.u8(self.right.len() as u8);
        for value in &self.right {
            out.u16(*value);
        }
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.control = input.u16()?;
        self.cycle = input.u16()? & 0x0fff;
        self.chip_phase = input.u64()? % self.period_cycles().max(1);
        self.timer_count = input.u8()? & 0x0f;
        self.sample_phase = input.u64()? % SH2_HZ;
        self.channels[0].last = input.u16()?;
        self.channels[1].last = input.u16()?;
        self.left.clear();
        let left = input.u8()?;
        if usize::from(left) > PWM_FIFO_DEPTH {
            return Err("32X PWM state left FIFO length is invalid".into());
        }
        for _ in 0..left {
            self.left.push_back(input.u16()? & 0x0fff);
        }
        self.right.clear();
        let right = input.u8()?;
        if usize::from(right) > PWM_FIFO_DEPTH {
            return Err("32X PWM state right FIFO length is invalid".into());
        }
        for _ in 0..right {
            self.right.push_back(input.u16()? & 0x0fff);
        }
        self.samples.clear();
        Ok(())
    }
}

struct MarsBoard {
    header: MarsHeader,
    sdram: Box<[u8; SDRAM_SIZE]>,
    framebuffers: [Box<[u8; FRAMEBUFFER_SIZE]>; 2],
    palette: [u16; 256],
    comm: [u16; 8],
    adapter_control: u16,
    interrupt_command: u16,
    rom_bank: u8,
    dreq_control: u16,
    dreq_source: u32,
    dreq_destination: u32,
    dreq_length: u16,
    dreq_remaining: u32,
    dreq_fifo: VecDeque<u16>,
    sh_mask: [u16; 2],
    h_count: u8,
    h_phase: u64,
    pending: [u8; 2],
    sh_divu: [Sh2Divu; 2],
    sh_dmac: [Sh2Dmac; 2],
    sh_frt: [Sh2Frt; 2],
    sh_intc: [Sh2Intc; 2],
    sh_sci: [Sh2Sci; 2],
    sh_wdt: [Sh2Wdt; 2],
    bitmap_mode: u16,
    screen_shift: u16,
    fill_length: u8,
    fill_start: u16,
    fill_data: u16,
    fill_busy_cycles: u64,
    display_buffer: usize,
    requested_display_buffer: usize,
    vblank: bool,
    pwm: MarsPwm,
    pwm_clock_phase: u64,
    pwm_dreq_pending: [u32; 2],
    video: VideoBuffer,
}

impl MarsBoard {
    fn new(rom: &[u8]) -> Result<Self, String> {
        let (header, source) = MarsHeader::parse(rom)?;
        let mut board = Self {
            header,
            sdram: Box::new([0; SDRAM_SIZE]),
            framebuffers: [
                Box::new([0; FRAMEBUFFER_SIZE]),
                Box::new([0; FRAMEBUFFER_SIZE]),
            ],
            palette: [0; 256],
            comm: [0; 8],
            adapter_control: 0x0080,
            interrupt_command: 0,
            rom_bank: 0,
            dreq_control: 0,
            dreq_source: 0,
            dreq_destination: 0,
            dreq_length: 0,
            dreq_remaining: 0,
            dreq_fifo: VecDeque::with_capacity(4),
            sh_mask: [0; 2],
            h_count: 0,
            h_phase: 0,
            pending: [0; 2],
            sh_divu: [Sh2Divu::default(); 2],
            sh_dmac: [Sh2Dmac::default(); 2],
            sh_frt: [Sh2Frt::default(); 2],
            sh_intc: [Sh2Intc::default(); 2],
            sh_sci: [Sh2Sci::default(); 2],
            sh_wdt: [Sh2Wdt::default(); 2],
            bitmap_mode: 0,
            screen_shift: 0,
            fill_length: 0,
            fill_start: 0,
            fill_data: 0,
            fill_busy_cycles: 0,
            display_buffer: 0,
            requested_display_buffer: 0,
            vblank: false,
            pwm: MarsPwm::default(),
            pwm_clock_phase: 0,
            pwm_dreq_pending: [0; 2],
            video: VideoBuffer::new(WIDTH, HEIGHT),
        };
        board.load_program(rom, source)?;
        board.reset_handshake();
        Ok(board)
    }

    fn load_program(&mut self, rom: &[u8], source: std::ops::Range<usize>) -> Result<(), String> {
        let destination = self.header.destination as usize;
        let size = self.header.size as usize;
        let target = self
            .sdram
            .get_mut(destination..destination + size)
            .ok_or_else(|| "32X MARS destination is outside SDRAM".to_string())?;
        if source.len() != target.len() {
            return Err("32X MARS source and destination lengths disagree".into());
        }
        target.copy_from_slice(&rom[source]);
        Ok(())
    }

    fn reset_handshake(&mut self) {
        self.comm = [0; 8];
        self.comm[0] = u16::from_be_bytes(*b"M_");
        self.comm[1] = u16::from_be_bytes(*b"OK");
        self.comm[2] = u16::from_be_bytes(*b"S_");
        self.comm[3] = u16::from_be_bytes(*b"OK");
        self.adapter_control = 0x0080;
        self.interrupt_command = 0;
        self.rom_bank = 0;
        self.dreq_control = 0;
        self.dreq_source = 0;
        self.dreq_destination = 0;
        self.dreq_length = 0;
        self.dreq_remaining = 0;
        self.dreq_fifo.clear();
        self.sh_mask = [0; 2];
        self.h_count = 0;
        self.h_phase = 0;
        self.pending = [0; 2];
        self.sh_divu = [Sh2Divu::default(); 2];
        self.sh_dmac = [Sh2Dmac::default(); 2];
        self.sh_frt = [Sh2Frt::default(); 2];
        self.sh_intc = [Sh2Intc::default(); 2];
        self.sh_sci = [Sh2Sci::default(); 2];
        self.sh_wdt = [Sh2Wdt::default(); 2];
        self.bitmap_mode = 0;
        self.screen_shift = 0;
        self.fill_length = 0;
        self.fill_start = 0;
        self.fill_data = 0;
        self.fill_busy_cycles = 0;
        self.display_buffer = 0;
        self.requested_display_buffer = 0;
        self.vblank = false;
        self.pwm = MarsPwm::default();
        self.pwm_clock_phase = 0;
        self.pwm_dreq_pending = [0; 2];
        for buffer in &mut self.framebuffers {
            buffer.fill(0);
        }
        self.palette.fill(0);
        self.video.clear([0, 0, 0, 255]);
    }

    fn sh_boot_ready(&self) -> bool {
        self.adapter_control & 0x0003 == 0x0003 && self.comm[..4].iter().all(|value| *value == 0)
    }

    fn fm(&self) -> bool {
        self.adapter_control & 0x8000 != 0
    }

    fn dreq_active(&self) -> bool {
        self.dreq_control & 0x0004 != 0
    }

    fn dreq_full(&self) -> bool {
        self.dreq_fifo.len() >= 4
    }

    fn dreq_empty(&self) -> bool {
        self.dreq_fifo.is_empty()
    }

    fn dreq_count_register(&self) -> u16 {
        if self.dreq_remaining == 0x1_0000 {
            0
        } else {
            self.dreq_remaining as u16
        }
    }

    fn set_dreq_control(&mut self, value: u16) {
        let was_active = self.dreq_active();
        self.dreq_control = value & 0x0005;
        let active = self.dreq_active();
        if active && !was_active {
            self.dreq_remaining = if self.dreq_length == 0 {
                0x1_0000
            } else {
                u32::from(self.dreq_length)
            };
            self.dreq_fifo.clear();
        } else if !active {
            self.dreq_remaining = 0;
            self.dreq_fifo.clear();
        }
    }

    fn set_dreq_length(&mut self, value: u16) {
        self.dreq_length = value & 0xfffc;
        if self.dreq_active() {
            self.dreq_remaining = if self.dreq_length == 0 {
                0x1_0000
            } else {
                u32::from(self.dreq_length)
            };
        }
    }

    fn push_dreq_word(&mut self, value: u16) {
        if self.dreq_active() && !self.dreq_full() {
            self.dreq_fifo.push_back(value);
        }
    }

    fn pop_dreq_word(&mut self) -> u16 {
        if !self.dreq_active() {
            return 0;
        }
        let Some(value) = self.dreq_fifo.pop_front() else {
            return 0;
        };
        if self.dreq_remaining != 0 {
            self.dreq_remaining -= 1;
            if self.dreq_remaining == 0 {
                self.dreq_control &= !0x0004;
                self.dreq_fifo.clear();
            }
        }
        value
    }

    fn main_dreq_control(&self) -> u16 {
        self.dreq_control | (u16::from(self.dreq_full()) << 7)
    }

    fn sh_dreq_control(&self) -> u16 {
        self.dreq_control
            | (u16::from(self.dreq_full()) << 15)
            | (u16::from(self.dreq_empty()) << 14)
    }

    fn main_system_read16(&self, offset: u32) -> u16 {
        match offset & 0x3e {
            0x00 => self.adapter_control,
            0x02 => self.interrupt_command,
            0x04 => u16::from(self.rom_bank & 3),
            0x06 => self.main_dreq_control(),
            0x08 => (self.dreq_source >> 16) as u16,
            0x0a => self.dreq_source as u16,
            0x0c => (self.dreq_destination >> 16) as u16,
            0x0e => self.dreq_destination as u16,
            0x10 => self.dreq_count_register(),
            0x20..=0x2e => self.comm[((offset - 0x20) >> 1) as usize],
            0x30..=0x38 => self.pwm.read(offset - 0x30),
            _ => 0,
        }
    }

    fn main_system_write16(&mut self, offset: u32, value: u16) {
        match offset & 0x3e {
            0x00 => {
                let sticky = self.adapter_control & 0x0003;
                let requested = value & 0x8003;
                self.adapter_control = 0x0080 | requested | sticky;
            }
            0x02 => {
                self.interrupt_command |= value & 3;
                if value & 1 != 0 {
                    self.pending[0] |= IRQ_CMD;
                }
                if value & 2 != 0 {
                    self.pending[1] |= IRQ_CMD;
                }
            }
            0x04 => self.rom_bank = (value & 3) as u8,
            0x06 => self.set_dreq_control(value),
            0x08 => self.dreq_source = (u32::from(value) << 16) | (self.dreq_source & 0xffff),
            0x0a => self.dreq_source = (self.dreq_source & 0xffff_0000) | u32::from(value & 0xfffe),
            0x0c => {
                self.dreq_destination = (u32::from(value) << 16) | (self.dreq_destination & 0xffff)
            }
            0x0e => {
                self.dreq_destination =
                    (self.dreq_destination & 0xffff_0000) | u32::from(value & 0xfffe)
            }
            0x10 => self.set_dreq_length(value),
            0x12 => self.push_dreq_word(value),
            0x20..=0x2e => self.comm[((offset - 0x20) >> 1) as usize] = value,
            0x30..=0x38 => self.pwm.write(offset - 0x30, value),
            _ => {}
        }
    }

    fn sh_system_read16(&mut self, side: usize, offset: u32) -> u16 {
        match offset & 0x3e {
            0x00 => {
                (self.adapter_control & 0x8000)
                    | ((self.adapter_control & 1) << 9)
                    | (self.sh_mask[side] & 0x008f)
            }
            0x04 => u16::from(self.h_count),
            0x06 => self.sh_dreq_control(),
            0x08 => (self.dreq_source >> 16) as u16,
            0x0a => self.dreq_source as u16,
            0x0c => (self.dreq_destination >> 16) as u16,
            0x0e => self.dreq_destination as u16,
            0x10 => self.dreq_count_register(),
            0x12 => self.pop_dreq_word(),
            0x20..=0x2e => self.comm[((offset - 0x20) >> 1) as usize],
            0x30..=0x38 => self.pwm.read(offset - 0x30),
            _ => 0,
        }
    }

    fn sh_system_write16(&mut self, side: usize, offset: u32, value: u16) {
        match offset & 0x3e {
            0x00 => {
                self.sh_mask[side] = value & 0x008f;
                if value & 0x8000 != 0 {
                    self.adapter_control |= 0x8000;
                } else {
                    self.adapter_control &= !0x8000;
                }
            }
            0x04 => self.h_count = value as u8,
            0x08 => self.dreq_source = (u32::from(value) << 16) | (self.dreq_source & 0xffff),
            0x0a => self.dreq_source = (self.dreq_source & 0xffff_0000) | u32::from(value & 0xfffe),
            0x0c => {
                self.dreq_destination = (u32::from(value) << 16) | (self.dreq_destination & 0xffff)
            }
            0x0e => {
                self.dreq_destination =
                    (self.dreq_destination & 0xffff_0000) | u32::from(value & 0xfffe)
            }
            0x14 => self.pending[side] &= !IRQ_VRES,
            0x16 => self.pending[side] &= !IRQ_V,
            0x18 => self.pending[side] &= !IRQ_H,
            0x1a => {
                self.pending[side] &= !IRQ_CMD;
                self.interrupt_command &= !(1 << side);
            }
            0x1c => self.pending[side] &= !IRQ_PWM,
            0x20..=0x2e => self.comm[((offset - 0x20) >> 1) as usize] = value,
            0x30..=0x38 => self.pwm.write(offset - 0x30, value),
            _ => {}
        }
    }

    fn interrupt(&self, side: usize) -> Option<(u8, u8)> {
        let pending = self.pending[side];
        let mask = self.sh_mask[side];
        let external = if pending & IRQ_VRES != 0 {
            Some((14, 71))
        } else if pending & IRQ_V != 0 && mask & 0x08 != 0 {
            Some((12, 70))
        } else if pending & IRQ_H != 0 && mask & 0x04 != 0 {
            Some((10, 69))
        } else if pending & IRQ_CMD != 0 && mask & 0x02 != 0 {
            Some((8, 68))
        } else if pending & IRQ_PWM != 0 && mask & 0x01 != 0 {
            Some((6, 67))
        } else {
            None
        };
        let mut selected = external;
        for candidate in [
            self.sh_dmac[side].interrupt(&self.sh_intc[side]),
            self.sh_frt[side].interrupt(&self.sh_intc[side]),
            self.sh_sci[side].interrupt(&self.sh_intc[side]),
            self.sh_wdt[side].interrupt(&self.sh_intc[side]),
            self.sh_divu[side].interrupt(&self.sh_intc[side]),
        ]
        .into_iter()
        .flatten()
        {
            if selected.is_none_or(|current| candidate.0 > current.0) {
                selected = Some(candidate);
            }
        }
        selected
    }

    fn tick_main_cycles(&mut self, main_cycles: u32) {
        self.h_phase = self.h_phase.saturating_add(u64::from(main_cycles));
        let interval = MAIN_CYCLES_PER_LINE * (u64::from(self.h_count) + 1);
        while self.h_phase >= interval {
            self.h_phase -= interval;
            self.pending[0] |= IRQ_H;
            self.pending[1] |= IRQ_H;
        }
        self.pwm_clock_phase = self
            .pwm_clock_phase
            .saturating_add(u64::from(main_cycles) * SH2_HZ);
        let clocks = self.pwm_clock_phase / M68K_HZ;
        self.pwm_clock_phase %= M68K_HZ;
        if clocks != 0 {
            let sh_clocks = clocks.min(u64::from(u32::MAX)) as u32;
            if self.sh_boot_ready() {
                for frt in &mut self.sh_frt {
                    frt.tick(sh_clocks);
                }
                for wdt in &mut self.sh_wdt {
                    wdt.tick(sh_clocks);
                }
                let master_tx = self.sh_sci[0].tick(sh_clocks);
                let slave_tx = self.sh_sci[1].tick(sh_clocks);
                if let Some((value, multiprocessor)) = master_tx {
                    self.sh_sci[1].receive(value, multiprocessor);
                }
                if let Some((value, multiprocessor)) = slave_tx {
                    self.sh_sci[0].receive(value, multiprocessor);
                }
            }
            self.fill_busy_cycles = self.fill_busy_cycles.saturating_sub(clocks);
            let timer_events = self.pwm.tick(sh_clocks);
            if timer_events != 0 {
                self.pending[0] |= IRQ_PWM;
                self.pending[1] |= IRQ_PWM;
                if self.pwm.control & 0x0080 != 0 {
                    for pending in &mut self.pwm_dreq_pending {
                        *pending = pending.saturating_add(timer_events);
                    }
                }
            }
        }
    }

    fn on_vblank(&mut self) {
        self.vblank = true;
        self.display_buffer = self.requested_display_buffer & 1;
        self.pending[0] |= IRQ_V;
        self.pending[1] |= IRQ_V;
    }

    fn end_vblank(&mut self) {
        self.vblank = false;
    }

    fn read_vdp16(&self, offset: u32) -> u16 {
        match offset & 0x0e {
            0x00 => self.bitmap_mode | 0x8000,
            0x02 => self.screen_shift,
            0x04 => u16::from(self.fill_length),
            0x06 => self.fill_start,
            0x08 => self.fill_data,
            0x0a => {
                (u16::from(self.vblank) << 15)
                    | 0x2000
                    | (u16::from(self.fill_busy_cycles != 0) << 1)
                    | self.display_buffer as u16
            }
            _ => 0,
        }
    }

    fn write_vdp16(&mut self, offset: u32, value: u16) {
        match offset & 0x0e {
            0x00 => self.bitmap_mode = value & 0x00c3,
            0x02 => self.screen_shift = value & 1,
            0x04 => self.fill_length = value as u8,
            0x06 => self.fill_start = value,
            0x08 => {
                self.fill_data = value;
                let draw = self.display_buffer ^ 1;
                let page = self.fill_start & 0xff00;
                let start = self.fill_start & 0x00ff;
                let words = u16::from(self.fill_length) + 1;
                for index in 0..words {
                    let word = page | start.wrapping_add(index) & 0x00ff;
                    let offset = usize::from(word) * 2;
                    if offset + 1 < FRAMEBUFFER_SIZE {
                        self.framebuffers[draw][offset..offset + 2]
                            .copy_from_slice(&value.to_be_bytes());
                    }
                }
                self.fill_start = page | start.wrapping_add(u16::from(self.fill_length)) & 0x00ff;
                self.fill_busy_cycles = 7 + 3 * u64::from(words);
            }
            0x0a => self.requested_display_buffer = usize::from(value & 1),
            _ => {}
        }
    }

    fn palette_read16(&self, offset: u32) -> u16 {
        self.palette[((offset >> 1) & 0xff) as usize]
    }

    fn palette_write16(&mut self, offset: u32, value: u16) {
        self.palette[((offset >> 1) & 0xff) as usize] = value;
    }

    fn framebuffer_read8(&self, offset: usize) -> u8 {
        self.framebuffers[self.display_buffer ^ 1][offset & (FRAMEBUFFER_SIZE - 1)]
    }

    fn framebuffer_write8(&mut self, offset: usize, value: u8, _overwrite: bool) {
        if value == 0 {
            return;
        }
        let draw = self.display_buffer ^ 1;
        self.framebuffers[draw][offset & (FRAMEBUFFER_SIZE - 1)] = value;
    }

    fn framebuffer_write16(&mut self, offset: usize, value: u16, overwrite: bool) {
        let draw = self.display_buffer ^ 1;
        let base = offset & (FRAMEBUFFER_SIZE - 1);
        let [high, low] = value.to_be_bytes();
        if !overwrite || high != 0 {
            self.framebuffers[draw][base] = high;
        }
        let next = (base + 1) & (FRAMEBUFFER_SIZE - 1);
        if !overwrite || low != 0 {
            self.framebuffers[draw][next] = low;
        }
    }

    fn color(value: u16) -> [u8; 4] {
        let expand = |v: u16| ((v * 255 + 15) / 31) as u8;
        [
            expand(value & 0x1f),
            expand((value >> 5) & 0x1f),
            expand((value >> 10) & 0x1f),
            255,
        ]
    }

    fn overlay_pixel(
        video: &mut VideoBuffer,
        bitmap_mode: u16,
        base: &[u8],
        x: usize,
        y: usize,
        value: u16,
    ) {
        let through = value & 0x8000 != 0;
        let mars_priority = bitmap_mode & 0x0080 != 0;
        let mars_front = mars_priority ^ through;
        let offset = (y * WIDTH as usize + x) * 4;
        let base_transparent = base[offset..offset + 3]
            .iter()
            .all(|component| *component == 0);
        if mars_front || base_transparent {
            video.pixels_mut()[offset..offset + 4].copy_from_slice(&Self::color(value));
        }
    }

    fn line_offset(buffer: &[u8], y: usize) -> Option<usize> {
        let table = y.checked_mul(2)?;
        let bytes: [u8; 2] = buffer.get(table..table + 2)?.try_into().ok()?;
        Some(usize::from(u16::from_be_bytes(bytes)) * 2)
    }

    fn render(&mut self, base: &VideoBuffer) {
        if self.video.width() != WIDTH || self.video.height() != HEIGHT {
            self.video.resize(WIDTH, HEIGHT);
        }
        let source = base.pixels();
        if source.len() == self.video.pixels().len() {
            self.video.pixels_mut().copy_from_slice(source);
        } else {
            self.video.clear([0, 0, 0, 255]);
        }
        let mode = self.bitmap_mode & 3;
        if mode == 0 {
            return;
        }
        let frame: &[u8] = self.framebuffers[self.display_buffer].as_slice();
        let bitmap_mode = self.bitmap_mode;
        for y in 0..HEIGHT as usize {
            let Some(mut cursor) = Self::line_offset(frame, y) else {
                continue;
            };
            match mode {
                1 => {
                    let shift = usize::from(self.screen_shift & 1);
                    for x in 0..WIDTH as usize {
                        let index = frame.get(cursor + x + shift).copied().unwrap_or(0);
                        let color = self.palette[usize::from(index)];
                        Self::overlay_pixel(&mut self.video, bitmap_mode, source, x, y, color);
                    }
                }
                2 => {
                    for x in 0..WIDTH as usize {
                        let Some(bytes) = frame.get(cursor..cursor + 2) else {
                            break;
                        };
                        let color = u16::from_be_bytes([bytes[0], bytes[1]]);
                        Self::overlay_pixel(&mut self.video, bitmap_mode, source, x, y, color);
                        cursor += 2;
                    }
                }
                3 => {
                    let mut x = 0usize;
                    while x < WIDTH as usize {
                        let Some(bytes) = frame.get(cursor..cursor + 2) else {
                            break;
                        };
                        cursor += 2;
                        let run = usize::from(bytes[0]) + 1;
                        let color = self.palette[usize::from(bytes[1])];
                        for px in x..(x + run).min(WIDTH as usize) {
                            Self::overlay_pixel(&mut self.video, bitmap_mode, source, px, y, color);
                        }
                        x = x.saturating_add(run);
                    }
                }
                _ => unreachable!(),
            }
        }
    }

    fn save(&self, out: &mut StateWriter) {
        out.blob(self.sdram.as_slice());
        out.blob(self.framebuffers[0].as_slice());
        out.blob(self.framebuffers[1].as_slice());
        for value in self.palette {
            out.u16(value);
        }
        for value in self.comm {
            out.u16(value);
        }
        out.u16(self.adapter_control);
        out.u16(self.interrupt_command);
        out.u8(self.rom_bank);
        out.u16(self.dreq_control);
        out.u32(self.dreq_source);
        out.u32(self.dreq_destination);
        out.u16(self.dreq_length);
        out.u32(self.dreq_remaining);
        out.u8(self.dreq_fifo.len() as u8);
        for value in &self.dreq_fifo {
            out.u16(*value);
        }
        out.u16(self.sh_mask[0]);
        out.u16(self.sh_mask[1]);
        out.u8(self.h_count);
        out.u64(self.h_phase);
        out.u8(self.pending[0]);
        out.u8(self.pending[1]);
        self.sh_divu[0].save(out);
        self.sh_divu[1].save(out);
        self.sh_dmac[0].save(out);
        self.sh_dmac[1].save(out);
        self.sh_frt[0].save(out);
        self.sh_frt[1].save(out);
        self.sh_intc[0].save(out);
        self.sh_intc[1].save(out);
        self.sh_sci[0].save(out);
        self.sh_sci[1].save(out);
        self.sh_wdt[0].save(out);
        self.sh_wdt[1].save(out);
        out.u16(self.bitmap_mode);
        out.u16(self.screen_shift);
        out.u8(self.fill_length);
        out.u16(self.fill_start);
        out.u16(self.fill_data);
        out.u64(self.fill_busy_cycles);
        out.u8(self.display_buffer as u8);
        out.u8(self.requested_display_buffer as u8);
        out.u8(u8::from(self.vblank));
        out.u64(self.pwm_clock_phase);
        out.u32(self.pwm_dreq_pending[0]);
        out.u32(self.pwm_dreq_pending[1]);
        self.pwm.save(out);
    }

    fn load(&mut self, input: &mut StateReader<'_>, base: &VideoBuffer) -> Result<(), String> {
        let sdram = input.blob()?;
        if sdram.len() != SDRAM_SIZE {
            return Err("32X SDRAM state size mismatch".into());
        }
        self.sdram.copy_from_slice(sdram);
        for index in 0..2 {
            let frame = input.blob()?;
            if frame.len() != FRAMEBUFFER_SIZE {
                return Err("32X framebuffer state size mismatch".into());
            }
            self.framebuffers[index].copy_from_slice(frame);
        }
        for value in &mut self.palette {
            *value = input.u16()?;
        }
        for value in &mut self.comm {
            *value = input.u16()?;
        }
        self.adapter_control = (input.u16()? & 0x8083) | 0x0080;
        self.interrupt_command = input.u16()? & 3;
        self.rom_bank = input.u8()? & 3;
        self.dreq_control = input.u16()? & 0x0005;
        self.dreq_source = input.u32()? & !1;
        self.dreq_destination = input.u32()? & !1;
        self.dreq_length = input.u16()? & 0xfffc;
        self.dreq_remaining = input.u32()?;
        if self.dreq_remaining > 0x1_0000 {
            return Err("32X DREQ state remaining count is invalid".into());
        }
        self.dreq_fifo.clear();
        let dreq_words = input.u8()?;
        if dreq_words > 4 {
            return Err("32X DREQ state FIFO length is invalid".into());
        }
        for _ in 0..dreq_words {
            self.dreq_fifo.push_back(input.u16()?);
        }
        if !self.dreq_active() && (self.dreq_remaining != 0 || !self.dreq_fifo.is_empty()) {
            return Err("32X DREQ state is inconsistent with inactive CPU-write mode".into());
        }
        if self.dreq_active()
            && (self.dreq_remaining == 0 || self.dreq_fifo.len() as u32 > self.dreq_remaining)
        {
            return Err("32X DREQ state has an invalid active transfer count".into());
        }
        self.sh_mask[0] = input.u16()? & 0x008f;
        self.sh_mask[1] = input.u16()? & 0x008f;
        self.h_count = input.u8()?;
        self.h_phase = input.u64()?;
        self.pending[0] = input.u8()?;
        self.pending[1] = input.u8()?;
        self.sh_divu[0].load(input)?;
        self.sh_divu[1].load(input)?;
        self.sh_dmac[0].load(input)?;
        self.sh_dmac[1].load(input)?;
        self.sh_frt[0].load(input)?;
        self.sh_frt[1].load(input)?;
        self.sh_intc[0].load(input)?;
        self.sh_intc[1].load(input)?;
        self.sh_sci[0].load(input)?;
        self.sh_sci[1].load(input)?;
        self.sh_wdt[0].load(input)?;
        self.sh_wdt[1].load(input)?;
        self.bitmap_mode = input.u16()? & 0x00c3;
        self.screen_shift = input.u16()? & 1;
        self.fill_length = input.u8()?;
        self.fill_start = input.u16()?;
        self.fill_data = input.u16()?;
        self.fill_busy_cycles = input.u64()?;
        self.display_buffer = usize::from(input.u8()? & 1);
        self.requested_display_buffer = usize::from(input.u8()? & 1);
        self.vblank = input.u8()? != 0;
        self.pwm_clock_phase = input.u64()? % M68K_HZ;
        self.pwm_dreq_pending[0] = input.u32()?;
        self.pwm_dreq_pending[1] = input.u32()?;
        self.pwm.load(input)?;
        self.render(base);
        Ok(())
    }
}

struct Sega32xMainBus<'a> {
    genesis: &'a mut GenesisBus,
    mars: &'a mut MarsBoard,
}

impl Sega32xMainBus<'_> {
    fn register_read8(value: u16, address: u32) -> u8 {
        if address & 1 == 0 {
            (value >> 8) as u8
        } else {
            value as u8
        }
    }

    fn merged_byte(old: u16, address: u32, value: u8) -> u16 {
        if address & 1 == 0 {
            (old & 0x00ff) | (u16::from(value) << 8)
        } else {
            (old & 0xff00) | u16::from(value)
        }
    }
}

impl Bus68000 for Sega32xMainBus<'_> {
    fn read8(&mut self, address: u32) -> u8 {
        let address = address & 0x00ff_ffff;
        match address {
            0x840000..=0x85ffff if !self.mars.fm() => {
                self.mars.framebuffer_read8((address - 0x840000) as usize)
            }
            0x860000..=0x87ffff if !self.mars.fm() => {
                self.mars.framebuffer_read8((address - 0x860000) as usize)
            }
            0x900000..=0x9fffff => {
                let banked = u32::from(self.mars.rom_bank) * 0x100000 + (address - 0x900000);
                self.genesis
                    .cartridge
                    .rom()
                    .get(banked as usize)
                    .copied()
                    .unwrap_or(0xff)
            }
            0xa15100..=0xa1513f => {
                let word = self.mars.main_system_read16((address - 0xa15100) & !1);
                Self::register_read8(word, address)
            }
            0xa15180..=0xa1518b if !self.mars.fm() => {
                let word = self.mars.read_vdp16((address - 0xa15180) & !1);
                Self::register_read8(word, address)
            }
            0xa15200..=0xa153ff if !self.mars.fm() => {
                let word = self.mars.palette_read16((address - 0xa15200) & !1);
                Self::register_read8(word, address)
            }
            _ => <GenesisBus as Bus68000>::read8(self.genesis, address),
        }
    }

    fn write8(&mut self, address: u32, value: u8) {
        let address = address & 0x00ff_ffff;
        match address {
            0x840000..=0x85ffff if !self.mars.fm() => {
                self.mars
                    .framebuffer_write8((address - 0x840000) as usize, value, false)
            }
            0x860000..=0x87ffff if !self.mars.fm() => {
                self.mars
                    .framebuffer_write8((address - 0x860000) as usize, value, true)
            }
            0xa15100..=0xa1513f => {
                let offset = (address - 0xa15100) & !1;
                let old = self.mars.main_system_read16(offset);
                self.mars
                    .main_system_write16(offset, Self::merged_byte(old, address, value));
            }
            0xa15180..=0xa1518b if !self.mars.fm() => {
                let offset = (address - 0xa15180) & !1;
                let old = self.mars.read_vdp16(offset);
                self.mars
                    .write_vdp16(offset, Self::merged_byte(old, address, value));
            }
            0xa15200..=0xa153ff if !self.mars.fm() => {
                let offset = (address - 0xa15200) & !1;
                let old = self.mars.palette_read16(offset);
                self.mars
                    .palette_write16(offset, Self::merged_byte(old, address, value));
            }
            _ => <GenesisBus as Bus68000>::write8(self.genesis, address, value),
        }
    }

    fn read16(&mut self, address: u32) -> u16 {
        let address = address & 0x00ff_ffff;
        match address {
            0xa15100..=0xa1513e => self.mars.main_system_read16(address - 0xa15100),
            0xa15180..=0xa1518a if !self.mars.fm() => self.mars.read_vdp16(address - 0xa15180),
            0xa15200..=0xa153fe if !self.mars.fm() => self.mars.palette_read16(address - 0xa15200),
            0x840000..=0x87fffe | 0x900000..=0x9ffffe => {
                u16::from_be_bytes([self.read8(address), self.read8(address.wrapping_add(1))])
            }
            _ => <GenesisBus as Bus68000>::read16(self.genesis, address),
        }
    }

    fn write16(&mut self, address: u32, value: u16) {
        let address = address & 0x00ff_ffff;
        match address {
            0xa15100..=0xa1513e => self.mars.main_system_write16(address - 0xa15100, value),
            0xa15180..=0xa1518a if !self.mars.fm() => {
                self.mars.write_vdp16(address - 0xa15180, value)
            }
            0xa15200..=0xa153fe if !self.mars.fm() => {
                self.mars.palette_write16(address - 0xa15200, value)
            }
            0x840000..=0x85fffe if !self.mars.fm() => {
                self.mars
                    .framebuffer_write16((address - 0x840000) as usize, value, false)
            }
            0x860000..=0x87fffe if !self.mars.fm() => {
                self.mars
                    .framebuffer_write16((address - 0x860000) as usize, value, true)
            }
            _ => <GenesisBus as Bus68000>::write16(self.genesis, address, value),
        }
    }

    fn read32(&mut self, address: u32) -> u32 {
        (u32::from(self.read16(address)) << 16) | u32::from(self.read16(address.wrapping_add(2)))
    }

    fn write32(&mut self, address: u32, value: u32) {
        self.write16(address, (value >> 16) as u16);
        self.write16(address.wrapping_add(2), value as u16);
    }
}

struct Sega32xShBus<'a> {
    mars: &'a mut MarsBoard,
    rom: &'a [u8],
    side: usize,
}

impl Sega32xShBus<'_> {
    fn normalized(address: u32) -> u32 {
        address & 0x1fff_ffff
    }

    fn read_word_register(&mut self, address: u32) -> Option<u16> {
        let address = Self::normalized(address);
        match address {
            0x00004000..=0x0000403f => Some(
                self.mars
                    .sh_system_read16(self.side, (address - 0x4000) & !1),
            ),
            0x00004100..=0x0000410b if self.mars.fm() => {
                Some(self.mars.read_vdp16((address - 0x4100) & !1))
            }
            0x00004200..=0x000043ff if self.mars.fm() => {
                Some(self.mars.palette_read16((address - 0x4200) & !1))
            }
            _ => None,
        }
    }

    fn write_word_register(&mut self, address: u32, value: u16) -> bool {
        let address = Self::normalized(address);
        match address {
            0x00004000..=0x0000403f => {
                self.mars
                    .sh_system_write16(self.side, (address - 0x4000) & !1, value)
            }
            0x00004100..=0x0000410b if self.mars.fm() => {
                self.mars.write_vdp16((address - 0x4100) & !1, value)
            }
            0x00004200..=0x000043ff if self.mars.fm() => {
                self.mars.palette_write16((address - 0x4200) & !1, value)
            }
            _ => return false,
        }
        true
    }

    fn peripheral_read32(&self, address: u32) -> Option<u32> {
        self.mars.sh_divu[self.side]
            .read32(address)
            .or_else(|| self.mars.sh_dmac[self.side].read32(address))
    }

    fn peripheral_write32(&mut self, address: u32, value: u32) -> bool {
        if self.mars.sh_divu[self.side].write32(address, value) {
            true
        } else {
            self.mars.sh_dmac[self.side].write32(address, value)
        }
    }

    fn peripheral_read8(&mut self, address: u32) -> Option<u8> {
        if let Some(value) = self.mars.sh_sci[self.side].read8(address) {
            return Some(value);
        }
        if let Some(value) = self.mars.sh_intc[self.side].read8(address) {
            return Some(value);
        }
        if let Some(value) = self.mars.sh_wdt[self.side].read8(address) {
            return Some(value);
        }
        if let Some(value) = self.mars.sh_frt[self.side].read8(address) {
            return Some(value);
        }
        if let Some(value) = self.mars.sh_dmac[self.side].read_drcr(address) {
            return Some(value);
        }
        let word = self.peripheral_read32(address)?;
        let shift = (3 - (address & 3)) * 8;
        Some((word >> shift) as u8)
    }

    fn peripheral_read16(&mut self, address: u32) -> Option<u16> {
        if (0xffff_fe00..=0xffff_fe04).contains(&address) {
            let high = self.mars.sh_sci[self.side].read8(address)?;
            let low = self.mars.sh_sci[self.side].read8(address.wrapping_add(1))?;
            return Some(u16::from_be_bytes([high, low]));
        }
        if let Some(value) = self.mars.sh_intc[self.side].read16(address) {
            return Some(value);
        }
        if let (Some(high), Some(low)) = (
            self.mars.sh_wdt[self.side].read8(address),
            self.mars.sh_wdt[self.side].read8(address.wrapping_add(1)),
        ) {
            return Some(u16::from_be_bytes([high, low]));
        }
        if let (Some(high), Some(low)) = (
            self.mars.sh_frt[self.side].read8(address),
            self.mars.sh_frt[self.side].read8(address.wrapping_add(1)),
        ) {
            return Some(u16::from_be_bytes([high, low]));
        }
        if address & 3 == 3 {
            return None;
        }
        let word = self.peripheral_read32(address)?;
        let shift = (2 - (address & 2)) * 8;
        Some((word >> shift) as u16)
    }

    fn peripheral_write8(&mut self, address: u32, value: u8) -> bool {
        if self.mars.sh_sci[self.side].write8(address, value) {
            return true;
        }
        if self.mars.sh_intc[self.side].write8(address, value) {
            return true;
        }
        if self.mars.sh_wdt[self.side].write8(address, value) {
            return true;
        }
        if self.mars.sh_frt[self.side].write8(address, value) {
            return true;
        }
        if self.mars.sh_dmac[self.side].write_drcr(address, value) {
            return true;
        }
        let aligned = address & !3;
        let Some(old) = self.peripheral_read32(aligned) else {
            return false;
        };
        let shift = (3 - (address & 3)) * 8;
        let mask = 0xffu32 << shift;
        self.peripheral_write32(aligned, (old & !mask) | (u32::from(value) << shift))
    }

    fn peripheral_write16(&mut self, address: u32, value: u16) -> bool {
        if (0xffff_fe00..=0xffff_fe04).contains(&address) {
            let [high, low] = value.to_be_bytes();
            let high_written = self.mars.sh_sci[self.side].write8(address, high);
            let low_written = self.mars.sh_sci[self.side].write8(address.wrapping_add(1), low);
            return high_written && low_written;
        }
        if self.mars.sh_wdt[self.side].write16(address, value) {
            return true;
        }
        if self.mars.sh_intc[self.side].write16(address, value) {
            return true;
        }
        if self.mars.sh_frt[self.side].read8(address).is_some()
            && self.mars.sh_frt[self.side]
                .read8(address.wrapping_add(1))
                .is_some()
        {
            let [high, low] = value.to_be_bytes();
            let _ = self.mars.sh_frt[self.side].write8(address, high);
            let _ = self.mars.sh_frt[self.side].write8(address.wrapping_add(1), low);
            return true;
        }
        if address & 3 == 3 {
            return false;
        }
        let aligned = address & !3;
        let Some(old) = self.peripheral_read32(aligned) else {
            return false;
        };
        let shift = (2 - (address & 2)) * 8;
        let mask = 0xffffu32 << shift;
        self.peripheral_write32(aligned, (old & !mask) | (u32::from(value) << shift))
    }

    fn service_dreq_dma0(&mut self) -> u32 {
        if !self.mars.dreq_active()
            || self.mars.dreq_empty()
            || !self.mars.sh_dmac[self.side].channel0_dreq_ready()
        {
            return 0;
        }

        let destination = self.mars.sh_dmac[self.side].channels[0].dar;
        let value = self.mars.pop_dreq_word();
        self.write16(destination, value);

        let channel = &mut self.mars.sh_dmac[self.side].channels[0];
        channel.dar = channel.dar.wrapping_add(2);
        let remaining = if channel.tcr == 0 {
            0x0100_0000
        } else {
            channel.tcr
        };
        if remaining == 1 {
            self.mars.sh_dmac[self.side].channel0_transfer_complete();
        } else {
            channel.tcr = (remaining - 1) & 0x00ff_ffff;
        }
        2
    }

    fn service_pwm_dma1(&mut self) -> u32 {
        if self.mars.pwm_dreq_pending[self.side] == 0 {
            return 0;
        }
        let Some(size) = self.mars.sh_dmac[self.side].channel1_pwm_transfer_size() else {
            return 0;
        };
        let channel = self.mars.sh_dmac[self.side].channels[1];
        let pwm_offset = match channel.dar {
            0x2000_4034 => 0x04,
            0x2000_4036 => 0x06,
            0x2000_4038 => 0x08,
            _ => return 0,
        };
        if (size == 2 && !self.mars.pwm.can_accept(pwm_offset))
            || (size == 4 && (channel.dar != 0x2000_4034 || !self.mars.pwm.can_accept(0x08)))
        {
            return 0;
        }

        if size == 2 {
            let value = self.read16(channel.sar);
            self.write16(channel.dar, value);
        } else {
            let value = self.read32(channel.sar);
            self.write32(channel.dar, value);
        }
        self.mars.pwm_dreq_pending[self.side] -= 1;

        let source_mode = (channel.chcr >> 12) & 3;
        let dma = &mut self.mars.sh_dmac[self.side].channels[1];
        match source_mode {
            1 => dma.sar = dma.sar.wrapping_add(size),
            2 => dma.sar = dma.sar.wrapping_sub(size),
            _ => {}
        }
        let remaining = if dma.tcr == 0 { 0x0100_0000 } else { dma.tcr };
        if remaining == 1 {
            self.mars.sh_dmac[self.side].channel1_transfer_complete();
        } else {
            dma.tcr = (remaining - 1) & 0x00ff_ffff;
        }
        if size == 4 {
            3
        } else {
            2
        }
    }

    fn service_auto_dma(&mut self) -> u32 {
        for index in 0..2 {
            let Some(size) = self.mars.sh_dmac[self.side].auto_transfer_size(index) else {
                continue;
            };
            let channel = self.mars.sh_dmac[self.side].channels[index];
            match size {
                1 => {
                    let value = self.read8(channel.sar);
                    self.write8(channel.dar, value);
                }
                2 => {
                    let value = self.read16(channel.sar);
                    self.write16(channel.dar, value);
                }
                4 => {
                    let value = self.read32(channel.sar);
                    self.write32(channel.dar, value);
                }
                16 => {
                    for offset in [0u32, 4, 8, 12] {
                        let value = self.read32(channel.sar.wrapping_add(offset));
                        self.write32(channel.dar.wrapping_add(offset), value);
                    }
                }
                _ => unreachable!(),
            }

            let source_mode = (channel.chcr >> 12) & 3;
            let destination_mode = (channel.chcr >> 14) & 3;
            let dma = &mut self.mars.sh_dmac[self.side].channels[index];
            match source_mode {
                1 => dma.sar = dma.sar.wrapping_add(size),
                2 => dma.sar = dma.sar.wrapping_sub(size),
                _ => {}
            }
            match destination_mode {
                1 => dma.dar = dma.dar.wrapping_add(size),
                2 => dma.dar = dma.dar.wrapping_sub(size),
                _ => {}
            }
            let remaining = if dma.tcr == 0 { 0x0100_0000 } else { dma.tcr };
            if remaining == 1 {
                self.mars.sh_dmac[self.side].transfer_complete(index);
            } else {
                dma.tcr = (remaining - 1) & 0x00ff_ffff;
            }
            return match size {
                1 | 2 => 2,
                4 => 3,
                16 => 8,
                _ => unreachable!(),
            };
        }
        0
    }
}

impl Sh2Bus for Sega32xShBus<'_> {
    fn read8(&mut self, address: u32) -> u8 {
        if address >= 0xffff_fe00 {
            return self.peripheral_read8(address).unwrap_or(0);
        }
        if let Some(word) = self.read_word_register(address) {
            return if address & 1 == 0 {
                (word >> 8) as u8
            } else {
                word as u8
            };
        }
        let address = Self::normalized(address);
        match address {
            0x02000000..=0x023fffff => self
                .rom
                .get((address - 0x02000000) as usize)
                .copied()
                .unwrap_or(0xff),
            0x04000000..=0x0401ffff => self.mars.framebuffer_read8((address - 0x04000000) as usize),
            0x04020000..=0x0403ffff => self.mars.framebuffer_read8((address - 0x04020000) as usize),
            0x06000000..=0x0603ffff => self.mars.sdram[(address - 0x06000000) as usize],
            _ => 0,
        }
    }

    fn write8(&mut self, address: u32, value: u8) {
        if address >= 0xffff_fe00 {
            let _ = self.peripheral_write8(address, value);
            return;
        }
        let normalized = Self::normalized(address);
        if matches!(normalized, 0x00004000..=0x000043ff) {
            let old = self.read_word_register(address).unwrap_or(0);
            let merged = if address & 1 == 0 {
                (old & 0x00ff) | (u16::from(value) << 8)
            } else {
                (old & 0xff00) | u16::from(value)
            };
            let _ = self.write_word_register(address, merged);
            return;
        }
        match normalized {
            0x04000000..=0x0401ffff if self.mars.fm() => {
                self.mars
                    .framebuffer_write8((normalized - 0x04000000) as usize, value, false)
            }
            0x04020000..=0x0403ffff if self.mars.fm() => {
                self.mars
                    .framebuffer_write8((normalized - 0x04020000) as usize, value, true)
            }
            0x06000000..=0x0603ffff => self.mars.sdram[(normalized - 0x06000000) as usize] = value,
            _ => {}
        }
    }

    fn read16(&mut self, address: u32) -> u16 {
        if address >= 0xffff_fe00 {
            return self.peripheral_read16(address).unwrap_or(0);
        }
        if let Some(word) = self.read_word_register(address) {
            return word;
        }
        u16::from_be_bytes([self.read8(address), self.read8(address.wrapping_add(1))])
    }

    fn write16(&mut self, address: u32, value: u16) {
        if address >= 0xffff_fe00 {
            let _ = self.peripheral_write16(address, value);
            return;
        }
        if self.write_word_register(address, value) {
            return;
        }
        let normalized = Self::normalized(address);
        match normalized {
            0x04000000..=0x0401fffe if self.mars.fm() => {
                self.mars
                    .framebuffer_write16((normalized - 0x04000000) as usize, value, false);
                return;
            }
            0x04020000..=0x0403fffe if self.mars.fm() => {
                self.mars
                    .framebuffer_write16((normalized - 0x04020000) as usize, value, true);
                return;
            }
            _ => {}
        }
        let [high, low] = value.to_be_bytes();
        self.write8(address, high);
        self.write8(address.wrapping_add(1), low);
    }

    fn read32(&mut self, address: u32) -> u32 {
        if address >= 0xffff_fe00 {
            return self.peripheral_read32(address).unwrap_or(0);
        }
        (u32::from(self.read16(address)) << 16) | u32::from(self.read16(address.wrapping_add(2)))
    }

    fn write32(&mut self, address: u32, value: u32) {
        if address >= 0xffff_fe00 {
            let _ = self.peripheral_write32(address, value);
            return;
        }
        self.write16(address, (value >> 16) as u16);
        self.write16(address.wrapping_add(2), value as u16);
    }
}

impl MarsBoard {
    fn reset_from_rom(&mut self, rom: &[u8]) -> Result<(), String> {
        self.sdram.fill(0);
        let start = self.header.source as usize;
        let end = start
            .checked_add(self.header.size as usize)
            .ok_or_else(|| "32X MARS source range overflows during reset".to_string())?;
        self.load_program(rom, start..end)?;
        self.reset_handshake();
        Ok(())
    }
}

pub struct Sega32xMachine {
    base: GenesisMachine,
    sh2: [Sh2; 2],
    mars: MarsBoard,
    audio: AudioBuffer,
    sh_phase: u64,
    sh_credit: [i64; 2],
    sh_faulted: [bool; 2],
    powered: bool,
}

impl Sega32xMachine {
    pub fn from_rom(image: &[u8]) -> Result<Self, String> {
        let base = GenesisMachine::from_rom(image)?;
        let mars = MarsBoard::new(base.bus.cartridge.rom())?;
        let mut machine = Self {
            base,
            sh2: [Sh2::default(), Sh2::default()],
            mars,
            audio: AudioBuffer::new(AUDIO_RATE as u32, 2),
            sh_phase: 0,
            sh_credit: [0; 2],
            sh_faulted: [false; 2],
            powered: true,
        };
        machine.reset_sh2();
        machine.mars.render(machine.base.bus.vdp.video());
        Ok(machine)
    }

    fn reset_sh2(&mut self) {
        self.sh2[0].reset(
            self.mars.header.master_entry,
            self.mars.header.master_vbr,
            0x0603_fff0,
        );
        self.sh2[1].reset(
            self.mars.header.slave_entry,
            self.mars.header.slave_vbr,
            0x0603_eff0,
        );
        self.sh_phase = 0;
        self.sh_credit = [0; 2];
        self.sh_faulted = [false; 2];
    }

    fn run_sh2_side(&mut self, side: usize) {
        if !self.mars.sh_boot_ready() || self.sh_faulted[side] {
            self.sh_credit[side] = 0;
            return;
        }
        while self.sh_credit[side] > 0 {
            let dma_used = {
                let rom = self.base.bus.cartridge.rom();
                let mut bus = Sega32xShBus {
                    mars: &mut self.mars,
                    rom,
                    side,
                };
                let dreq0 = bus.service_dreq_dma0();
                if dreq0 != 0 {
                    dreq0
                } else {
                    bus.service_pwm_dma1()
                }
            };
            if dma_used != 0 {
                self.sh_credit[side] -= i64::from(dma_used);
                continue;
            }

            let interrupt = self.mars.interrupt(side);
            if let Some((level, vector)) = interrupt {
                let rom = self.base.bus.cartridge.rom();
                let mut bus = Sega32xShBus {
                    mars: &mut self.mars,
                    rom,
                    side,
                };
                let used = self.sh2[side].interrupt(&mut bus, level, vector);
                if used != 0 {
                    self.sh_credit[side] -= i64::from(used);
                    continue;
                }
            }
            let rom = self.base.bus.cartridge.rom();
            let mut bus = Sega32xShBus {
                mars: &mut self.mars,
                rom,
                side,
            };
            let mut used = self.sh2[side].step(&mut bus);
            if used == 0 {
                self.sh_faulted[side] = true;
                self.sh_credit[side] = 0;
                break;
            }
            used = used.saturating_add(bus.service_auto_dma());
            self.sh_credit[side] -= i64::from(used);
            if self.sh_credit[side] < -32 {
                self.sh_credit[side] = -32;
            }
        }
    }

    fn advance_time(&mut self, main_cycles: u32) {
        self.base.bus.tick_main_cycles(main_cycles);
        self.base.run_z80_for_main_cycles(main_cycles);
        self.mars.tick_main_cycles(main_cycles);
        self.sh_phase = self
            .sh_phase
            .saturating_add(u64::from(main_cycles) * SH2_HZ);
        let sh_cycles = self.sh_phase / M68K_HZ;
        self.sh_phase %= M68K_HZ;
        for credit in &mut self.sh_credit {
            *credit = credit.saturating_add(sh_cycles as i64);
        }
        self.run_sh2_side(0);
        self.run_sh2_side(1);
    }

    fn clock_main_instruction(&mut self) -> u32 {
        let frame_before = self.base.bus.vdp.frame();
        if let Some(used) = self.base.bus.service_vdp_dma() {
            self.base.main_cpu.cycles = self.base.main_cpu.cycles.wrapping_add(u64::from(used));
            self.advance_time(used);
            if self.base.bus.vdp.frame() != frame_before {
                self.mars.on_vblank();
                self.mars.render(self.base.bus.vdp.video());
                self.mars.end_vblank();
            }
            return used;
        }

        let used = {
            let mut bus = Sega32xMainBus {
                genesis: &mut self.base.bus,
                mars: &mut self.mars,
            };
            self.base.main_cpu.step(&mut bus)
        };
        if used == 0 {
            self.powered = false;
            return 0;
        }
        self.advance_time(used);
        let level = self.base.bus.vdp.irq_level();
        if level != 0 {
            let interrupt_cycles = {
                let mut bus = Sega32xMainBus {
                    genesis: &mut self.base.bus,
                    mars: &mut self.mars,
                };
                self.base.main_cpu.interrupt(&mut bus, level, 24 + level)
            };
            if interrupt_cycles != 0 {
                self.base.bus.vdp.acknowledge_irq(level);
                self.advance_time(interrupt_cycles);
            }
        }
        if self.base.bus.vdp.frame() != frame_before {
            self.mars.on_vblank();
            self.mars.render(self.base.bus.vdp.video());
            self.mars.end_vblank();
        }
        used
    }

    fn begin_audio_frame(&mut self) {
        self.base.begin_audio_frame();
        self.mars.pwm.begin_frame();
        self.audio.begin_frame();
    }

    fn flush_audio(&mut self) {
        self.base.flush_audio();
        self.audio.begin_frame();
        let base = self.base.audio.samples();
        let pwm = &self.mars.pwm.samples;
        let count = (base.len() / 2).max(pwm.len());
        for index in 0..count {
            let left = base.get(index * 2).copied().unwrap_or(0.0);
            let right = base.get(index * 2 + 1).copied().unwrap_or(0.0);
            let (pwm_left, pwm_right) = pwm.get(index).copied().unwrap_or((0.0, 0.0));
            self.audio.push_stereo(
                (left + pwm_left).clamp(-1.0, 1.0),
                (right + pwm_right).clamp(-1.0, 1.0),
            );
        }
    }
}

impl Machine for Sega32xMachine {
    fn platform(&self) -> PlatformId {
        PlatformId::Sega32x
    }

    fn reset(&mut self) {
        self.base.reset();
        if self
            .mars
            .reset_from_rom(self.base.bus.cartridge.rom())
            .is_err()
        {
            self.powered = false;
            return;
        }
        self.reset_sh2();
        self.powered = true;
        self.begin_audio_frame();
        self.mars.render(self.base.bus.vdp.video());
    }

    fn run_frame(&mut self, input: &InputState) {
        if !self.powered {
            return;
        }
        self.base.bus.io.set_input(input);
        self.begin_audio_frame();
        let target = self.base.bus.vdp.frame().wrapping_add(1);
        let deadline = self
            .base
            .main_cpu
            .cycles
            .saturating_add((M68K_HZ as f64 / FRAME_RATE * 2.0).ceil() as u64);
        while self.base.bus.vdp.frame() != target && self.base.main_cpu.cycles < deadline {
            if self.clock_main_instruction() == 0 {
                break;
            }
        }
        if self.base.bus.vdp.frame() != target {
            self.powered = false;
        }
        self.flush_audio();
    }

    fn frame_rate(&self) -> f64 {
        FRAME_RATE
    }

    fn video(&self) -> &VideoBuffer {
        &self.mars.video
    }

    fn audio(&self) -> &AudioBuffer {
        &self.audio
    }

    fn save_state(&self) -> Result<Vec<u8>, String> {
        let mut out = StateWriter::new(PlatformId::Sega32x, STATE_VERSION);
        out.blob(&self.base.save_state()?);
        self.sh2[0].save(&mut out);
        self.sh2[1].save(&mut out);
        self.mars.save(&mut out);
        out.u64(self.sh_phase);
        out.u64(self.sh_credit[0] as u64);
        out.u64(self.sh_credit[1] as u64);
        out.u8(u8::from(self.sh_faulted[0]));
        out.u8(u8::from(self.sh_faulted[1]));
        out.u8(u8::from(self.powered));
        Ok(out.finish())
    }

    fn load_state(&mut self, bytes: &[u8]) -> Result<(), String> {
        let mut input = StateReader::new(bytes, PlatformId::Sega32x, STATE_VERSION)?;
        self.base.load_state(input.blob()?)?;
        self.sh2[0].load(&mut input)?;
        self.sh2[1].load(&mut input)?;
        self.mars.load(&mut input, self.base.bus.vdp.video())?;
        self.sh_phase = input.u64()? % M68K_HZ;
        self.sh_credit[0] = input.u64()? as i64;
        self.sh_credit[1] = input.u64()? as i64;
        self.sh_faulted[0] = input.u8()? != 0;
        self.sh_faulted[1] = input.u8()? != 0;
        self.powered = input.u8()? != 0;
        self.begin_audio_frame();
        input.finish()
    }

    fn persistent_len(&self, kind: ResourceKind, slot: u32) -> usize {
        self.base.persistent_len(kind, slot)
    }

    fn read_persistent(&self, kind: ResourceKind, slot: u32, out: &mut [u8]) -> Result<(), String> {
        self.base.read_persistent(kind, slot, out)
    }

    fn write_persistent(
        &mut self,
        kind: ResourceKind,
        slot: u32,
        data: &[u8],
    ) -> Result<(), String> {
        self.base.write_persistent(kind, slot, data)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn word(image: &mut [u8], offset: usize, value: u16) {
        image[offset..offset + 2].copy_from_slice(&value.to_be_bytes());
    }

    fn long(image: &mut [u8], offset: usize, value: u32) {
        image[offset..offset + 4].copy_from_slice(&value.to_be_bytes());
    }

    fn synthetic_rom() -> Vec<u8> {
        let mut rom = vec![0xff; 0x20000];
        long(&mut rom, 0, 0x00ff_ff00);
        long(&mut rom, 4, 0x0000_0400);
        rom[0x100..0x108].copy_from_slice(b"SEGA 32X");
        rom[0x3c0..0x3c9].copy_from_slice(b"OMNICORE ");
        long(&mut rom, 0x3d0, 1);
        long(&mut rom, 0x3d4, 0x0000_1000);
        long(&mut rom, 0x3d8, 0x0000_0000);
        long(&mut rom, 0x3dc, 0x0000_0400);
        long(&mut rom, 0x3e0, 0x0600_0000);
        long(&mut rom, 0x3e4, 0x0600_0020);
        long(&mut rom, 0x3e8, 0x0600_0200);
        long(&mut rom, 0x3ec, 0x0600_0240);

        let main_words = [
            0x33fc, 0x0003, 0x00a1, 0x5100, 0x33fc, 0x0000, 0x00a1, 0x5120, 0x33fc, 0x0000, 0x00a1,
            0x5122, 0x33fc, 0x0000, 0x00a1, 0x5124, 0x33fc, 0x0000, 0x00a1, 0x5126, 0x33fc, 0x001f,
            0x00a1, 0x5202, 0x33fc, 0x0081, 0x00a1, 0x5180, 0x33fc, 0x0001, 0x00a1, 0x518a, 0x33fc,
            0x0100, 0x0084, 0x0000, 0x13fc, 0x0001, 0x0084, 0x0200, 0x60fe,
        ];
        for (index, value) in main_words.into_iter().enumerate() {
            word(&mut rom, 0x400 + index * 2, value);
        }

        let master = [0xd103, 0xe05a, 0x2102, 0xaffe, 0x0009];
        for (index, value) in master.into_iter().enumerate() {
            word(&mut rom, 0x1000 + index * 2, value);
        }
        long(&mut rom, 0x1010, 0x0600_0100);
        let slave = [0xd103, 0xe066, 0x2102, 0xaffe, 0x0009];
        for (index, value) in slave.into_iter().enumerate() {
            word(&mut rom, 0x1020 + index * 2, value);
        }
        long(&mut rom, 0x1030, 0x0600_0104);
        rom
    }

    #[test]
    fn mars_header_loads_application_into_sdram() {
        let rom = synthetic_rom();
        let board = MarsBoard::new(&rom).unwrap();
        assert_eq!(board.header.master_entry, 0x0600_0000);
        assert_eq!(board.header.slave_entry, 0x0600_0020);
        assert_eq!(&board.sdram[..10], &rom[0x1000..0x100a]);
        assert_eq!(board.comm[0], 0x4d5f);
        assert_eq!(board.comm[1], 0x4f4b);
    }

    #[test]
    fn main_and_sh2_register_views_share_state_and_interrupts() {
        let rom = synthetic_rom();
        let mut board = MarsBoard::new(&rom).unwrap();
        board.main_system_write16(0, 0x0003);
        board.main_system_write16(0x20, 0x1234);
        assert_eq!(board.sh_system_read16(0, 0x20), 0x1234);
        board.sh_system_write16(0, 0, 0x0002);
        board.main_system_write16(2, 1);
        assert_eq!(board.interrupt(0), Some((8, 68)));
        board.sh_system_write16(0, 0x1a, 0);
        assert_eq!(board.interrupt(0), None);
        assert_eq!(board.interrupt_command & 1, 0);
    }

    #[test]
    fn adapter_ren_is_asserted_and_read_only_on_main_bus() {
        let mut machine = Sega32xMachine::from_rom(&synthetic_rom()).unwrap();
        let mut bus = Sega32xMainBus {
            genesis: &mut machine.base.bus,
            mars: &mut machine.mars,
        };

        assert_eq!(bus.read8(0xa15101) & 0x80, 0x80);
        bus.write8(0xa15101, 0x00);
        assert_eq!(bus.read8(0xa15101) & 0x80, 0x80);
        bus.write16(0xa15100, 0x0003);
        assert_eq!(bus.read16(0xa15100) & 0x0083, 0x0083);
        bus.write16(0xa15100, 0x0000);
        assert_eq!(bus.read16(0xa15100) & 0x0080, 0x0080);
    }

    #[test]
    fn dreq_fifo_shares_registers_tracks_full_empty_and_terminates_length() {
        let rom = synthetic_rom();
        let mut board = MarsBoard::new(&rom).unwrap();
        board.main_system_write16(0x08, 0x1234);
        board.main_system_write16(0x0a, 0x5679);
        board.main_system_write16(0x0c, 0x2600);
        board.main_system_write16(0x0e, 0x0101);
        board.main_system_write16(0x10, 0x0007);
        assert_eq!(board.dreq_source, 0x1234_5678);
        assert_eq!(board.dreq_destination, 0x2600_0100);
        assert_eq!(board.dreq_length, 4);

        board.main_system_write16(0x06, 0x0005);
        assert_eq!(board.main_system_read16(0x06) & 0x0085, 0x0005);
        assert_eq!(board.sh_system_read16(0, 0x06) & 0xc005, 0x4005);
        assert_eq!(board.main_system_read16(0x10), 4);

        for value in [0x1111, 0x2222, 0x3333, 0x4444] {
            board.main_system_write16(0x12, value);
        }
        board.main_system_write16(0x12, 0x5555);
        assert_eq!(board.main_system_read16(0x06) & 0x0080, 0x0080);
        assert_eq!(board.sh_system_read16(0, 0x06) & 0xc000, 0x8000);
        assert_eq!(board.dreq_fifo.len(), 4);

        for (remaining, expected) in [(3, 0x1111), (2, 0x2222), (1, 0x3333)] {
            assert_eq!(board.sh_system_read16(0, 0x12), expected);
            assert_eq!(board.main_system_read16(0x10), remaining);
        }
        assert_eq!(board.sh_system_read16(0, 0x12), 0x4444);
        assert_eq!(board.main_system_read16(0x10), 0);
        assert_eq!(board.main_system_read16(0x06) & 0x0004, 0);
        assert_eq!(board.sh_system_read16(0, 0x06) & 0x4000, 0x4000);
        assert!(board.dreq_fifo.is_empty());
        assert_eq!(board.dreq_control & 1, 1);
    }

    #[test]
    fn channel0_dmac_moves_dreq_fifo_words_into_sdram() {
        let rom = synthetic_rom();
        let mut board = MarsBoard::new(&rom).unwrap();
        board.main_system_write16(0x10, 4);
        board.main_system_write16(0x06, 0x0004);
        for value in [0x1122, 0x3344, 0x5566, 0x7788] {
            board.main_system_write16(0x12, value);
        }

        {
            let mut bus = Sega32xShBus {
                mars: &mut board,
                rom: &rom,
                side: 0,
            };
            bus.write32(0xffff_ff80, 0x2000_4012);
            bus.write32(0xffff_ff84, 0x0600_0100);
            bus.write32(0xffff_ff88, 4);
            bus.write32(0xffff_ff8c, 0x0000_44e1);
            bus.write8(0xffff_fe71, 0);

            for _ in 0..4 {
                assert_eq!(bus.service_dreq_dma0(), 2);
            }
            assert_eq!(bus.service_dreq_dma0(), 0);
            assert_eq!(bus.read32(0xffff_ff84), 0x0600_0108);
            assert_eq!(bus.read32(0xffff_ff88), 0);
            assert_eq!(bus.read32(0xffff_ff8c) & 3, 3);
        }

        assert_eq!(
            &board.sdram[0x100..0x108],
            &[0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88]
        );
        assert!(!board.dreq_active());
        assert!(board.dreq_fifo.is_empty());
    }

    #[test]
    fn channel1_dmac_feeds_pwm_fifo_with_word_and_long_transfers() {
        let rom = synthetic_rom();
        let mut board = MarsBoard::new(&rom).unwrap();
        board.sdram[0x200..0x208]
            .copy_from_slice(&[0x01, 0x23, 0x04, 0x56, 0x07, 0x89, 0x0a, 0xbc]);
        board.pwm_dreq_pending[0] = 4;

        {
            let mut bus = Sega32xShBus {
                mars: &mut board,
                rom: &rom,
                side: 0,
            };
            bus.write32(0xffff_ff90, 0x0600_0200);
            bus.write32(0xffff_ff94, 0x2000_4034);
            bus.write32(0xffff_ff98, 4);
            bus.write32(0xffff_ff9c, 0x0000_14e1);
            bus.write8(0xffff_fe72, 0);

            assert_eq!(bus.service_pwm_dma1(), 2);
            assert_eq!(bus.service_pwm_dma1(), 2);
            assert_eq!(bus.service_pwm_dma1(), 2);
            assert_eq!(bus.service_pwm_dma1(), 0);
        }
        assert_eq!(board.pwm.left.len(), PWM_FIFO_DEPTH);
        assert_eq!(board.pwm.read(0x04) & 0x8000, 0x8000);
        board.pwm.clock_once();
        assert_eq!(board.pwm.channels[0].last, 0x0123);

        {
            let mut bus = Sega32xShBus {
                mars: &mut board,
                rom: &rom,
                side: 0,
            };
            assert_eq!(bus.service_pwm_dma1(), 2);
            assert_eq!(bus.read32(0xffff_ff90), 0x0600_0208);
            assert_eq!(bus.read32(0xffff_ff98), 0);
            assert_eq!(bus.read32(0xffff_ff9c) & 3, 3);
        }
        assert_eq!(
            board.pwm.left.iter().copied().collect::<Vec<_>>(),
            vec![0x0456, 0x0789, 0x0abc]
        );
        assert_eq!(board.pwm_dreq_pending[0], 0);

        board.pwm.left.clear();
        board.pwm.right.clear();
        board.pwm_dreq_pending[0] = 1;
        board.sdram[0x240..0x244].copy_from_slice(&[0x01, 0x11, 0x02, 0x22]);
        {
            let mut bus = Sega32xShBus {
                mars: &mut board,
                rom: &rom,
                side: 0,
            };
            bus.write32(0xffff_ff90, 0x0600_0240);
            bus.write32(0xffff_ff94, 0x2000_4034);
            bus.write32(0xffff_ff98, 1);
            bus.write32(0xffff_ff9c, 0x0000_18e1);
            assert_eq!(bus.service_pwm_dma1(), 3);
            assert_eq!(bus.read32(0xffff_ff90), 0x0600_0244);
        }
        assert_eq!(board.pwm.left.front(), Some(&0x0111));
        assert_eq!(board.pwm.right.front(), Some(&0x0222));
    }

    #[test]
    fn dmac_completion_uses_ipra_user_vector_and_priority_arbitration() {
        let rom = synthetic_rom();
        let mut board = MarsBoard::new(&rom).unwrap();
        board.sdram[0x300..0x302].copy_from_slice(&0x0555u16.to_be_bytes());
        board.pwm_dreq_pending[0] = 1;

        {
            let mut bus = Sega32xShBus {
                mars: &mut board,
                rom: &rom,
                side: 0,
            };
            bus.write32(0xffff_ff90, 0x0600_0300);
            bus.write32(0xffff_ff94, 0x2000_4034);
            bus.write32(0xffff_ff98, 1);
            bus.write32(0xffff_ff9c, 0x0000_14e5);
            bus.write32(0xffff_ffa8, 66);
            bus.write16(0xffff_fee2, 0x0400);
            assert_eq!(bus.service_pwm_dma1(), 2);
            assert_eq!(bus.read32(0xffff_ff9c) & 7, 7);
            assert_eq!(bus.read32(0xffff_ffa8), 66);
            assert_eq!(bus.read16(0xffff_fee2) & 0x0f00, 0x0400);
        }
        assert_eq!(board.interrupt(0), Some((4, 66)));

        board.sh_mask[0] |= 0x01;
        board.pending[0] |= IRQ_PWM;
        assert_eq!(board.interrupt(0), Some((6, 67)));
        board.pending[0] &= !IRQ_PWM;

        {
            let mut bus = Sega32xShBus {
                mars: &mut board,
                rom: &rom,
                side: 0,
            };
            bus.write32(0xffff_ff9c, 0x0000_14e5);
        }
        assert_eq!(board.interrupt(0), None);
    }

    #[test]
    fn auto_request_dmac_copies_memory_with_size_and_address_modes() {
        let rom = synthetic_rom();
        let mut board = MarsBoard::new(&rom).unwrap();
        board.sdram[0x100..0x108]
            .copy_from_slice(&[0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88]);

        {
            let mut bus = Sega32xShBus {
                mars: &mut board,
                rom: &rom,
                side: 0,
            };
            bus.write32(0xffff_ff80, 0x0600_0100);
            bus.write32(0xffff_ff84, 0x0600_0180);
            bus.write32(0xffff_ff88, 4);
            bus.write32(0xffff_ff8c, 0x0000_56e1);
            assert_eq!(bus.service_auto_dma(), 0);
            bus.write32(0xffff_ffb0, 1);
            for _ in 0..4 {
                assert_eq!(bus.service_auto_dma(), 2);
            }
            assert_eq!(bus.service_auto_dma(), 0);
            assert_eq!(bus.read32(0xffff_ff80), 0x0600_0108);
            assert_eq!(bus.read32(0xffff_ff84), 0x0600_0188);
            assert_eq!(bus.read32(0xffff_ff88), 0);
            assert_eq!(bus.read32(0xffff_ff8c) & 3, 3);
        }
        assert_eq!(&board.sdram[0x180..0x188], &board.sdram[0x100..0x108]);

        for (index, value) in (0x20u8..0x30).enumerate() {
            board.sdram[0x200 + index] = value;
        }
        {
            let mut bus = Sega32xShBus {
                mars: &mut board,
                rom: &rom,
                side: 0,
            };
            bus.write32(0xffff_ff90, 0x0600_0200);
            bus.write32(0xffff_ff94, 0x0600_0280);
            bus.write32(0xffff_ff98, 1);
            bus.write32(0xffff_ff9c, 0x0000_5ee1);
            assert_eq!(bus.service_auto_dma(), 8);
            assert_eq!(bus.read32(0xffff_ff90), 0x0600_0210);
            assert_eq!(bus.read32(0xffff_ff94), 0x0600_0290);
        }
        assert_eq!(&board.sdram[0x280..0x290], &board.sdram[0x200..0x210]);
    }

    #[test]
    fn packed_framebuffer_overlays_genesis_and_honors_vblank_switch() {
        let rom = synthetic_rom();
        let mut board = MarsBoard::new(&rom).unwrap();
        let base = VideoBuffer::new(WIDTH, HEIGHT);
        board.palette[1] = 0x001f;
        board.bitmap_mode = 0x0081;
        board.framebuffers[1][0..2].copy_from_slice(&0x0100u16.to_be_bytes());
        board.framebuffers[1][0x200] = 1;
        board.requested_display_buffer = 1;
        board.on_vblank();
        board.render(&base);
        let pixel = &board.video.pixels()[..4];
        assert!(pixel[0] > pixel[1] && pixel[0] > pixel[2]);
        assert_eq!(board.display_buffer, 1);
    }

    #[test]
    fn auto_fill_wraps_page_updates_last_address_and_reports_fen_busy() {
        let rom = synthetic_rom();
        let mut board = MarsBoard::new(&rom).unwrap();
        board.write_vdp16(0x04, 0x0010);
        board.write_vdp16(0x06, 0x20fe);
        board.write_vdp16(0x08, 0x1234);

        assert_eq!(board.fill_start, 0x200e);
        assert_ne!(board.read_vdp16(0x0a) & 0x0002, 0);
        assert_eq!(
            &board.framebuffers[1][0x41fc..0x4200],
            &[0x12, 0x34, 0x12, 0x34]
        );
        assert_eq!(&board.framebuffers[1][0x4000..0x4002], &[0x12, 0x34]);
        assert_eq!(&board.framebuffers[1][0x401c..0x401e], &[0x12, 0x34]);
        assert_eq!(&board.framebuffers[1][0x401e..0x4020], &[0x00, 0x00]);

        board.tick_main_cycles(100);
        assert_eq!(board.read_vdp16(0x0a) & 0x0002, 0);
    }

    #[test]
    fn framebuffer_zero_write_semantics_match_byte_and_word_windows() {
        let rom = synthetic_rom();
        let mut board = MarsBoard::new(&rom).unwrap();

        board.framebuffer_write8(0x40, 0x7a, false);
        board.framebuffer_write8(0x40, 0x00, false);
        assert_eq!(board.framebuffer_read8(0x40), 0x7a);
        board.framebuffer_write8(0x40, 0x00, true);
        assert_eq!(board.framebuffer_read8(0x40), 0x7a);

        board.framebuffer_write16(0x40, 0x0000, false);
        assert_eq!(board.framebuffer_read8(0x40), 0x00);
        assert_eq!(board.framebuffer_read8(0x41), 0x00);

        board.framebuffer_write16(0x40, 0x1234, false);
        board.framebuffer_write16(0x40, 0x5600, true);
        assert_eq!(board.framebuffer_read8(0x40), 0x56);
        assert_eq!(board.framebuffer_read8(0x41), 0x34);
        board.framebuffer_write16(0x40, 0x0078, true);
        assert_eq!(board.framebuffer_read8(0x40), 0x56);
        assert_eq!(board.framebuffer_read8(0x41), 0x78);
        board.framebuffer_write16(0x40, 0x0000, true);
        assert_eq!(board.framebuffer_read8(0x40), 0x56);
        assert_eq!(board.framebuffer_read8(0x41), 0x78);
    }

    #[test]
    fn sh2_sci_links_master_to_slave_with_receive_interrupt() {
        let rom = synthetic_rom();
        let mut board = MarsBoard::new(&rom).unwrap();
        board.adapter_control |= 0x0003;
        board.comm[..4].fill(0);
        {
            let mut master = Sega32xShBus {
                mars: &mut board,
                rom: &rom,
                side: 0,
            };
            master.write8(0xffff_fe00, 0x00);
            master.write8(0xffff_fe01, 0x00);
            master.write8(0xffff_fe02, 0x20);
            assert_eq!(master.read8(0xffff_fe04) & 0x80, 0x80);
            master.write8(0xffff_fe03, 0x5a);
            master.write8(0xffff_fe04, 0x04);
        }
        {
            let mut slave = Sega32xShBus {
                mars: &mut board,
                rom: &rom,
                side: 1,
            };
            slave.write8(0xffff_fe00, 0x00);
            slave.write8(0xffff_fe01, 0x00);
            slave.write8(0xffff_fe02, 0x50);
            slave.write16(0xffff_fe60, 0x7000);
            slave.write16(0xffff_fe62, 0x6061);
        }

        board.tick_main_cycles(1_000);
        {
            let mut slave = Sega32xShBus {
                mars: &mut board,
                rom: &rom,
                side: 1,
            };
            assert_eq!(slave.read8(0xffff_fe04) & 0x40, 0x40);
            assert_eq!(slave.read8(0xffff_fe05), 0x5a);
        }
        assert_eq!(board.interrupt(1), Some((7, 0x61)));
    }

    #[test]
    fn sh2_frt_registers_count_and_raise_programmable_interrupt() {
        let rom = synthetic_rom();
        let mut board = MarsBoard::new(&rom).unwrap();
        board.adapter_control |= 0x0003;
        board.comm[..4].fill(0);
        {
            let mut bus = Sega32xShBus {
                mars: &mut board,
                rom: &rom,
                side: 0,
            };
            bus.write8(0xffff_fe14, 0x00);
            bus.write8(0xffff_fe15, 0x03);
            bus.write8(0xffff_fe11, 0x01);
            bus.write8(0xffff_fe10, 0x08);
            bus.write8(0xffff_fe60, 0x05);
            bus.write8(0xffff_fe67, 0x66);
            bus.write8(0xffff_fe16, 0x00);
        }

        board.tick_main_cycles(9);
        assert_eq!(board.sh_frt[0].read8(0xffff_fe11).unwrap() & 0x08, 0x08);
        assert_eq!(board.sh_frt[1].read8(0xffff_fe11).unwrap() & 0x08, 0);
        assert_eq!(board.interrupt(0), Some((5, 0x66)));
    }

    #[test]
    fn sh2_wdt_interval_timer_uses_keyed_writes_and_shared_intc() {
        let rom = synthetic_rom();
        let mut board = MarsBoard::new(&rom).unwrap();
        {
            let mut bus = Sega32xShBus {
                mars: &mut board,
                rom: &rom,
                side: 0,
            };
            bus.write16(0xffff_fee2, 0x0050);
            bus.write16(0xffff_fee4, 0x6300);
            bus.write16(0xffff_fe80, 0x5aff);
            bus.write16(0xffff_fe80, 0xa53e);
        }
        board.sh_wdt[0].tick(4096);
        assert_eq!(board.interrupt(0), Some((5, 0x63)));
        {
            let mut bus = Sega32xShBus {
                mars: &mut board,
                rom: &rom,
                side: 0,
            };
            assert_eq!(bus.read8(0xffff_fe80) & 0x80, 0x80);
            bus.write16(0xffff_fe80, 0xa53e);
        }
        assert_eq!(board.interrupt(0), None);
        assert_eq!(board.sh_wdt[1].read8(0xffff_fe80), Some(0x18));
    }

    #[test]
    fn sh2_division_unit_supports_signed_math_mirrors_and_per_cpu_state() {
        let rom = synthetic_rom();
        let mut board = MarsBoard::new(&rom).unwrap();
        {
            let mut bus = Sega32xShBus {
                mars: &mut board,
                rom: &rom,
                side: 0,
            };
            bus.write32(0xffff_ff00, 7);
            bus.write32(0xffff_ff04, 100);
            assert_eq!(bus.read32(0xffff_ff04), 14);
            assert_eq!(bus.read32(0xffff_ff10), 2);
            assert_eq!(bus.read32(0xffff_ff14), 14);
            assert_eq!(bus.read32(0xffff_ff18), 2);
            assert_eq!(bus.read32(0xffff_ff1c), 14);

            bus.write32(0xffff_ff04, (-100i32) as u32);
            assert_eq!(bus.read32(0xffff_ff04) as i32, -14);
            assert_eq!(bus.read32(0xffff_ff10) as i32, -2);

            bus.write32(0xffff_ff00, 3);
            bus.write32(0xffff_ff10, 1);
            bus.write32(0xffff_ff14, 0);
            assert_eq!(bus.read32(0xffff_ff14), 0x5555_5555);
            assert_eq!(bus.read32(0xffff_ff10), 1);

            bus.write32(0xffff_ff08, 0);
            bus.write32(0xffff_ff00, 1);
            bus.write32(0xffff_ff10, 0x7fff_ffff);
            bus.write32(0xffff_ff14, 0xffff_ffff);
            assert_eq!(bus.read32(0xffff_ff14), 0x7fff_ffff);
            assert_eq!(bus.read32(0xffff_ff10), 0x7fff_ffff);
            assert_eq!(bus.read32(0xffff_ff08) & 1, 1);

            bus.write16(0xffff_ff00, 0);
            bus.write16(0xffff_ff02, 5);
            bus.write32(0xffff_ff04, 42);
            assert_eq!(bus.read16(0xffff_ff06), 8);
            assert_eq!(bus.read16(0xffff_ff12), 2);
        }

        {
            let mut bus = Sega32xShBus {
                mars: &mut board,
                rom: &rom,
                side: 0,
            };
            bus.write16(0xffff_fee2, 0x0007);
            bus.write32(0xffff_ff0c, 0x61);
            bus.write32(0xffff_ff08, 0x0000_0002);
            bus.write32(0xffff_ff00, 0);
            bus.write32(0xffff_ff04, 1);
        }
        assert_eq!(board.interrupt(0), Some((7, 0x61)));
        assert_eq!(board.sh_divu[0].dvsr, 0);
        assert_eq!(board.sh_divu[1].dvsr, 0);
        assert_eq!(board.sh_divu[1].dvdntl, 0);
    }

    #[test]
    fn dual_sh2_machine_runs_application_and_32x_video() {
        let mut machine = Sega32xMachine::from_rom(&synthetic_rom()).unwrap();
        machine.run_frame(&InputState::default());
        assert!(machine.powered);
        assert!(!machine.sh_faulted[0]);
        assert!(!machine.sh_faulted[1]);
        assert_eq!(machine.mars.sdram[0x103], 0x5a);
        assert_eq!(machine.mars.sdram[0x107], 0x66);
        assert_eq!(machine.video().width(), WIDTH);
        assert!(machine.video().pixels()[0] > machine.video().pixels()[1]);
    }

    #[test]
    fn state_round_trip_preserves_both_sh2_and_mars_memory() {
        let mut machine = Sega32xMachine::from_rom(&synthetic_rom()).unwrap();
        machine.run_frame(&InputState::default());
        machine.mars.sh_divu[0].dvsr = 7;
        machine.mars.sh_divu[0].divide32(100);
        machine.mars.sh_divu[1].vcrdiv = 0x5a;
        machine.mars.sh_frt[0].write8(0xffff_fe12, 0x12);
        machine.mars.sh_frt[0].write8(0xffff_fe13, 0x34);
        machine.mars.sh_frt[1].write8(0xffff_fe12, 0x56);
        machine.mars.sh_frt[1].write8(0xffff_fe13, 0x78);
        machine.mars.sh_sci[0].write8(0xffff_fe01, 0x12);
        machine.mars.sh_sci[1].write8(0xffff_fe01, 0x34);
        let saved = machine.save_state().unwrap();
        let master_pc = machine.sh2[0].pc;
        machine.mars.sdram[0x103] = 0;
        machine.mars.sh_divu = [Sh2Divu::default(); 2];
        machine.mars.sh_frt = [Sh2Frt::default(); 2];
        machine.mars.sh_sci = [Sh2Sci::default(); 2];
        machine.sh2[0].pc = 0;
        machine.load_state(&saved).unwrap();
        assert_eq!(machine.mars.sdram[0x103], 0x5a);
        assert_eq!(machine.sh2[0].pc, master_pc);
        assert_eq!(machine.mars.sh_divu[0].dvsr, 7);
        assert_eq!(machine.mars.sh_divu[0].dvdntl, 14);
        assert_eq!(machine.mars.sh_divu[0].dvdnth, 2);
        assert_eq!(machine.mars.sh_divu[1].vcrdiv, 0x5a);
        assert_eq!(machine.mars.sh_frt[0].read8(0xffff_fe12), Some(0x12));
        assert_eq!(machine.mars.sh_frt[0].read8(0xffff_fe13), Some(0x34));
        assert_eq!(machine.mars.sh_frt[1].read8(0xffff_fe12), Some(0x56));
        assert_eq!(machine.mars.sh_frt[1].read8(0xffff_fe13), Some(0x78));
        assert_eq!(machine.mars.sh_sci[0].read8(0xffff_fe01), Some(0x12));
        assert_eq!(machine.mars.sh_sci[1].read8(0xffff_fe01), Some(0x34));
        assert_eq!(machine.save_state().unwrap(), saved);
    }

    #[test]
    fn pwm_cycle_encoding_fifo_depth_and_rtp_dreq_match_hardware() {
        let mut pwm = MarsPwm {
            cycle: 0,
            ..Default::default()
        };
        assert_eq!(pwm.period_cycles(), 4095);
        pwm.cycle = 1;
        assert_eq!(pwm.period_cycles(), 0);
        pwm.control = 0x0185;
        assert_eq!(pwm.tick(32), 0);
        pwm.cycle = 2;
        assert_eq!(pwm.period_cycles(), 1);
        for value in [0x0111, 0x0222, 0x0333, 0x0444] {
            pwm.write(0x04, value);
        }
        assert_eq!(pwm.left.len(), PWM_FIFO_DEPTH);
        assert_eq!(pwm.read(0x04) & 0x8000, 0x8000);

        let rom = synthetic_rom();
        let mut board = MarsBoard::new(&rom).unwrap();
        board.pwm.control = 0x0185;
        board.pwm.cycle = 2;
        board.tick_main_cycles(1);
        assert_ne!(board.pending[0] & IRQ_PWM, 0);
        assert_ne!(board.pending[1] & IRQ_PWM, 0);
        assert_ne!(board.pwm_dreq_pending[0], 0);
        assert_eq!(board.pwm_dreq_pending[0], board.pwm_dreq_pending[1]);

        let before = board.pwm_dreq_pending;
        board.pwm.control &= !0x0080;
        board.tick_main_cycles(1);
        assert_eq!(board.pwm_dreq_pending, before);
    }

    #[test]
    fn pwm_fifo_generates_stereo_samples_and_timer_request() {
        let mut pwm = MarsPwm {
            control: 0x0505,
            cycle: 0x20,
            ..Default::default()
        };
        pwm.write(4, 0x18);
        pwm.write(6, 0x08);
        pwm.begin_frame();
        assert_ne!(pwm.tick(600), 0);
        assert!(!pwm.samples.is_empty());
        assert!(pwm.samples.iter().any(|(left, right)| left != right));
    }
}
