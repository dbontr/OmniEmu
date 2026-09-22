use super::nes_apu::Apu;
use crate::bus::Bus8;
use crate::cpu6502::Mos6502;
use crate::input::{
    AXIS_AUX_X, AXIS_AUX_Y, DOWN, FACE_EAST, FACE_SOUTH, LEFT, POINTER_CLICK, POINTER_TOUCH, RIGHT,
    SELECT, START, UP,
};
use crate::kernel::{AudioBuffer, InputState, VideoBuffer};
use crate::machine::Machine;
use crate::platform::PlatformId;
use crate::resources::ResourceKind;
use crate::state::{StateReader, StateWriter};

const WIDTH: u32 = 256;
const HEIGHT: u32 = 240;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum NesRegion {
    Ntsc,
    Pal,
    Dendy,
}

impl NesRegion {
    fn from_header(rom: &[u8], nes2: bool) -> Self {
        if nes2 {
            match rom[12] & 0x03 {
                1 => Self::Pal,
                3 => Self::Dendy,
                _ => Self::Ntsc,
            }
        } else if rom[9] & 1 != 0 {
            Self::Pal
        } else {
            Self::Ntsc
        }
    }

    fn cpu_hz(self) -> u64 {
        match self {
            Self::Ntsc => 1_789_773,
            Self::Pal => 1_662_607,
            Self::Dendy => 1_773_448,
        }
    }

    fn cpu_clock(self) -> f64 {
        match self {
            Self::Ntsc => 1_789_773.0,
            Self::Pal => 26_601_712.5 / 16.0,
            Self::Dendy => 26_601_712.5 / 15.0,
        }
    }

    fn frame_rate(self) -> f64 {
        match self {
            Self::Ntsc => 60.0988,
            Self::Pal | Self::Dendy => (26_601_712.5 / 5.0) / (312.0 * 341.0),
        }
    }

    fn total_scanlines(self) -> u16 {
        match self {
            Self::Ntsc => 262,
            Self::Pal | Self::Dendy => 312,
        }
    }

    fn pre_render_scanline(self) -> u16 {
        self.total_scanlines() - 1
    }

    fn vblank_scanline(self) -> u16 {
        match self {
            Self::Dendy => 291,
            Self::Ntsc | Self::Pal => 241,
        }
    }

    fn ppu_ratio(self) -> (u8, u8) {
        match self {
            Self::Pal => (16, 5),
            Self::Ntsc | Self::Dendy => (3, 1),
        }
    }

    fn uses_pal_apu(self) -> bool {
        matches!(self, Self::Pal)
    }

    fn state_id(self) -> u8 {
        match self {
            Self::Ntsc => 0,
            Self::Pal => 1,
            Self::Dendy => 3,
        }
    }
}

fn ram_size_from_shift(shift: u8) -> usize {
    if shift == 0 {
        0
    } else {
        64usize << shift
    }
}

fn nes2_rom_size(lsb: u8, msb: u8, unit: usize) -> Result<usize, String> {
    if msb != 0x0f {
        return ((usize::from(msb) << 8) | usize::from(lsb))
            .checked_mul(unit)
            .ok_or_else(|| "NES 2.0 ROM size overflows host address space".to_string());
    }
    let exponent = u32::from(lsb >> 2);
    let multiplier = usize::from((lsb & 3) * 2 + 1);
    1usize
        .checked_shl(exponent)
        .and_then(|base| base.checked_mul(multiplier))
        .ok_or_else(|| "NES 2.0 exponent ROM size exceeds host address space".to_string())
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Mirroring {
    Horizontal,
    Vertical,
    SingleScreen0,
    SingleScreen1,
    FourScreen,
}

#[derive(Clone, Debug)]
struct Mmc1 {
    shift: u8,
    bits: u8,
    control: u8,
    chr0: u8,
    chr1: u8,
    prg: u8,
}
impl Default for Mmc1 {
    fn default() -> Self {
        Self {
            shift: 0,
            bits: 0,
            control: 0x0c,
            chr0: 0,
            chr1: 0,
            prg: 0,
        }
    }
}

#[derive(Clone, Debug)]
struct Mmc3 {
    bank_select: u8,
    regs: [u8; 8],
    mirror_horizontal: bool,
    prg_ram_protect: u8,
    irq_latch: u8,
    irq_counter: u8,
    irq_reload: bool,
    irq_enabled: bool,
    irq_pending: bool,
    a12_high: bool,
    a12_low_cycles: u16,
    mc_acc_fall_counter: u8,
}
impl Default for Mmc3 {
    fn default() -> Self {
        Self {
            bank_select: 0,
            regs: [0; 8],
            mirror_horizontal: false,
            prg_ram_protect: 0x80,
            irq_latch: 0,
            irq_counter: 0,
            irq_reload: false,
            irq_enabled: false,
            irq_pending: false,
            a12_high: false,
            a12_low_cycles: 0,
            mc_acc_fall_counter: 0,
        }
    }
}

#[derive(Clone, Debug, Default)]
struct Rambo1 {
    bank_select: u8,
    regs: [u8; 16],
    mirror_horizontal: bool,
    irq_latch: u8,
    irq_counter: u8,
    irq_reload: bool,
    irq_cycle_mode: bool,
    irq_enabled: bool,
    irq_pending: bool,
    irq_prescaler: u8,
    irq_delay: u8,
    a12_high: bool,
    a12_low_cycles: u16,
}

#[derive(Clone, Debug)]
struct Eeprom24c02 {
    data: [u8; 256],
    mode: u8,
    next_mode: u8,
    shift: u8,
    address: u8,
    bits: u8,
    output: bool,
    prev_scl: bool,
    prev_sda: bool,
    ack_clocked: bool,
    master_ack: bool,
}

impl Default for Eeprom24c02 {
    fn default() -> Self {
        Self {
            data: [0xff; 256],
            mode: 0,
            next_mode: 0,
            shift: 0,
            address: 0,
            bits: 0,
            output: true,
            prev_scl: false,
            prev_sda: true,
            ack_clocked: false,
            master_ack: false,
        }
    }
}

impl Eeprom24c02 {
    const IDLE: u8 = 0;
    const CHIP_ADDRESS: u8 = 1;
    const MEMORY_ADDRESS: u8 = 2;
    const WRITE_DATA: u8 = 3;
    const SEND_ACK: u8 = 4;
    const READ_DATA: u8 = 5;
    const WAIT_MASTER_ACK: u8 = 6;

    fn enter_ack(&mut self, next_mode: u8) {
        self.mode = Self::SEND_ACK;
        self.next_mode = next_mode;
        self.bits = 0;
        self.shift = 0;
        self.output = false;
        self.ack_clocked = false;
    }

    fn begin_read(&mut self) {
        self.mode = Self::READ_DATA;
        self.bits = 0;
        self.shift = self.data[usize::from(self.address)];
        self.output = self.shift & 0x80 != 0;
    }

    fn write_lines(&mut self, scl: bool, sda: bool) {
        if self.prev_scl && scl && self.prev_sda && !sda {
            self.mode = Self::CHIP_ADDRESS;
            self.bits = 0;
            self.shift = 0;
            self.output = true;
        } else if self.prev_scl && scl && !self.prev_sda && sda {
            self.mode = Self::IDLE;
            self.output = true;
        } else if !self.prev_scl && scl {
            match self.mode {
                Self::CHIP_ADDRESS | Self::MEMORY_ADDRESS | Self::WRITE_DATA => {
                    self.shift = (self.shift << 1) | u8::from(sda);
                    self.bits = self.bits.saturating_add(1);
                }
                Self::SEND_ACK => self.ack_clocked = true,
                Self::WAIT_MASTER_ACK => self.master_ack = !sda,
                _ => {}
            }
        } else if self.prev_scl && !scl {
            match self.mode {
                Self::CHIP_ADDRESS if self.bits == 8 => {
                    if self.shift & 0xfe == 0xa0 {
                        let next = if self.shift & 1 != 0 {
                            Self::READ_DATA
                        } else {
                            Self::MEMORY_ADDRESS
                        };
                        self.enter_ack(next);
                    } else {
                        self.mode = Self::IDLE;
                        self.output = true;
                    }
                }
                Self::MEMORY_ADDRESS if self.bits == 8 => {
                    self.address = self.shift;
                    self.enter_ack(Self::WRITE_DATA);
                }
                Self::WRITE_DATA if self.bits == 8 => {
                    self.data[usize::from(self.address)] = self.shift;
                    self.address = (self.address & 0xf8) | (self.address.wrapping_add(1) & 7);
                    self.enter_ack(Self::WRITE_DATA);
                }
                Self::SEND_ACK if self.ack_clocked => {
                    self.output = true;
                    if self.next_mode == Self::READ_DATA {
                        self.begin_read();
                    } else {
                        self.mode = self.next_mode;
                        self.bits = 0;
                        self.shift = 0;
                    }
                }
                Self::READ_DATA => {
                    self.bits = self.bits.saturating_add(1);
                    if self.bits >= 8 {
                        self.mode = Self::WAIT_MASTER_ACK;
                        self.output = true;
                        self.master_ack = false;
                    } else {
                        self.output = self.shift & (0x80 >> self.bits) != 0;
                    }
                }
                Self::WAIT_MASTER_ACK => {
                    if self.master_ack {
                        self.address = self.address.wrapping_add(1);
                        self.begin_read();
                    } else {
                        self.mode = Self::IDLE;
                        self.output = true;
                    }
                }
                _ => {}
            }
        }
        self.prev_scl = scl;
        self.prev_sda = sda;
    }

    fn write_x24c01_lines(&mut self, scl: bool, sda: bool) {
        if self.prev_scl && scl && self.prev_sda && !sda {
            self.mode = Self::CHIP_ADDRESS;
            self.bits = 0;
            self.shift = 0;
            self.output = true;
        } else if self.prev_scl && scl && !self.prev_sda && sda {
            self.mode = Self::IDLE;
            self.output = true;
        } else if !self.prev_scl && scl {
            match self.mode {
                Self::CHIP_ADDRESS | Self::WRITE_DATA => {
                    self.shift = (self.shift << 1) | u8::from(sda);
                    self.bits = self.bits.saturating_add(1);
                }
                Self::SEND_ACK => self.ack_clocked = true,
                Self::WAIT_MASTER_ACK => self.master_ack = !sda,
                _ => {}
            }
        } else if self.prev_scl && !scl {
            match self.mode {
                Self::CHIP_ADDRESS if self.bits == 8 => {
                    self.address = (self.shift >> 1) & 0x7f;
                    let next = if self.shift & 1 != 0 {
                        Self::READ_DATA
                    } else {
                        Self::WRITE_DATA
                    };
                    self.enter_ack(next);
                }
                Self::WRITE_DATA if self.bits == 8 => {
                    self.data[usize::from(self.address)] = self.shift;
                    self.address = (self.address & 0x7c) | (self.address.wrapping_add(1) & 3);
                    self.enter_ack(Self::WRITE_DATA);
                }
                Self::SEND_ACK if self.ack_clocked => {
                    self.output = true;
                    if self.next_mode == Self::READ_DATA {
                        self.begin_read();
                    } else {
                        self.mode = self.next_mode;
                        self.bits = 0;
                        self.shift = 0;
                    }
                }
                Self::READ_DATA => {
                    self.bits = self.bits.saturating_add(1);
                    if self.bits >= 8 {
                        self.mode = Self::WAIT_MASTER_ACK;
                        self.output = true;
                        self.master_ack = false;
                    } else {
                        self.output = self.shift & (0x80 >> self.bits) != 0;
                    }
                }
                Self::WAIT_MASTER_ACK => {
                    if self.master_ack {
                        self.address = self.address.wrapping_add(1) & 0x7f;
                        self.begin_read();
                    } else {
                        self.mode = Self::IDLE;
                        self.output = true;
                    }
                }
                _ => {}
            }
        }
        self.prev_scl = scl;
        self.prev_sda = sda;
    }
}

#[derive(Clone, Debug, Default)]
struct BandaiFcg {
    chr: [u8; 8],
    prg: u8,
    mirroring: u8,
    irq_latch: u16,
    irq_counter: u16,
    irq_enabled: bool,
    irq_pending: bool,
    eeprom_present: bool,
    eeprom: Eeprom24c02,
    external_eeprom_present: bool,
    external_eeprom: Eeprom24c02,
    external_scl: bool,
    barcode_line: bool,
}

#[derive(Clone, Debug, Default)]
struct Namco210 {
    chr: [u8; 8],
    prg: [u8; 3],
    ram_enabled: bool,
    mirroring: u8,
    is_175: bool,
}

#[derive(Clone, Debug)]
struct Namco163 {
    chr: [u8; 8],
    nametable: [u8; 4],
    prg: [u8; 3],
    chr_disable_low: bool,
    chr_disable_high: bool,
    sound_disabled: bool,
    ram: [u8; 128],
    ram_address: u8,
    ram_autoincrement: bool,
    wram_protect: u8,
    irq_counter: u16,
    irq_enabled: bool,
    irq_pending: bool,
    audio_divider: u8,
    audio_channel: u8,
    audio_output: f32,
    sound_capable: bool,
    internal_battery: bool,
    submapper: u8,
}

impl Default for Namco163 {
    fn default() -> Self {
        Self {
            chr: [0; 8],
            nametable: [0xe0, 0xe1, 0xe0, 0xe1],
            prg: [0, 1, 2],
            chr_disable_low: false,
            chr_disable_high: false,
            sound_disabled: true,
            ram: [0; 128],
            ram_address: 0,
            ram_autoincrement: false,
            wram_protect: 0xff,
            irq_counter: 0,
            irq_enabled: false,
            irq_pending: false,
            audio_divider: 0,
            audio_channel: 0,
            audio_output: 0.0,
            sound_capable: true,
            internal_battery: false,
            submapper: 0,
        }
    }
}

impl Namco163 {
    fn advance_ram_address(&mut self) {
        if self.ram_autoincrement && self.ram_address < 0x7f {
            self.ram_address += 1;
        }
    }

    fn read_data(&mut self) -> u8 {
        let value = self.ram[usize::from(self.ram_address)];
        self.advance_ram_address();
        value
    }

    fn write_data(&mut self, value: u8) {
        self.ram[usize::from(self.ram_address)] = value;
        self.advance_ram_address();
    }

    fn audio_gain(&self) -> f32 {
        if !self.sound_capable {
            return 0.0;
        }
        let db = match self.submapper {
            3 => 12.0,
            4 => 16.5,
            5 => 18.75,
            _ => 15.0,
        };
        let full_apu_pulse = (95.88 / (8128.0 / 15.0 + 100.0)) * 1.35;
        full_apu_pulse * 10.0_f32.powf(db / 20.0) / 120.0
    }

    fn tick_audio(&mut self) {
        if !self.sound_capable || self.sound_disabled {
            self.audio_output = 0.0;
            return;
        }
        self.audio_divider += 1;
        if self.audio_divider < 15 {
            return;
        }
        self.audio_divider = 0;

        let channels = ((self.ram[0x7f] >> 4) & 7) + 1;
        self.audio_channel %= channels;
        let base = 0x78usize - usize::from(self.audio_channel) * 8;
        let frequency = u32::from(self.ram[base])
            | (u32::from(self.ram[base + 2]) << 8)
            | (u32::from(self.ram[base + 4] & 3) << 16);
        let mut phase = u32::from(self.ram[base + 1])
            | (u32::from(self.ram[base + 3]) << 8)
            | (u32::from(self.ram[base + 5]) << 16);
        let length = 256u32 - u32::from(self.ram[base + 4] & 0xfc);
        phase = (phase + frequency) % (length << 16);
        self.ram[base + 1] = phase as u8;
        self.ram[base + 3] = (phase >> 8) as u8;
        self.ram[base + 5] = (phase >> 16) as u8;

        let wave_address = (u32::from(self.ram[base + 6]) + (phase >> 16)) as u8;
        let packed = self.ram[usize::from(wave_address >> 1)];
        let sample = if wave_address & 1 == 0 {
            packed & 0x0f
        } else {
            packed >> 4
        };
        let volume = self.ram[base + 7] & 0x0f;
        let raw = (i16::from(sample) - 8) * i16::from(volume);
        self.audio_output = raw as f32 * self.audio_gain();
        self.audio_channel = (self.audio_channel + 1) % channels;
    }
}

#[derive(Clone, Debug)]
struct Mmc2 {
    chr_fd: [u8; 2],
    chr_fe: [u8; 2],
    latch_fe: [bool; 2],
    mirror_horizontal: bool,
}

impl Default for Mmc2 {
    fn default() -> Self {
        Self {
            chr_fd: [0; 2],
            chr_fe: [0; 2],
            latch_fe: [true; 2],
            mirror_horizontal: false,
        }
    }
}

#[derive(Clone, Debug)]
struct Vrc4 {
    prg: [u8; 2],
    chr: [u16; 8],
    mirroring: u8,
    swap_mode: bool,
    wram_enable: bool,
    latch_6000: u8,
    irq_latch: u8,
    irq_counter: u8,
    irq_prescaler: i16,
    irq_cycle_mode: bool,
    irq_enabled: bool,
    irq_enable_after_ack: bool,
    irq_pending: bool,
}

impl Default for Vrc4 {
    fn default() -> Self {
        Self {
            prg: [0; 2],
            chr: [0; 8],
            mirroring: 0,
            swap_mode: false,
            wram_enable: false,
            latch_6000: 0,
            irq_latch: 0,
            irq_counter: 0,
            irq_prescaler: 341,
            irq_cycle_mode: false,
            irq_enabled: false,
            irq_enable_after_ack: false,
            irq_pending: false,
        }
    }
}

#[derive(Clone, Debug)]
struct Vrc6Pulse {
    volume: u8,
    duty: u8,
    mode: bool,
    period: u16,
    timer: u16,
    step: u8,
    enabled: bool,
}

impl Default for Vrc6Pulse {
    fn default() -> Self {
        Self {
            volume: 0,
            duty: 0,
            mode: false,
            period: 0,
            timer: 0,
            step: 15,
            enabled: false,
        }
    }
}

impl Vrc6Pulse {
    fn write_control(&mut self, value: u8) {
        self.volume = value & 0x0f;
        self.duty = (value >> 4) & 7;
        self.mode = value & 0x80 != 0;
    }

    fn write_period_low(&mut self, value: u8) {
        self.period = (self.period & 0x0f00) | u16::from(value);
    }

    fn write_period_high(&mut self, value: u8) {
        self.period = (self.period & 0x00ff) | (u16::from(value & 0x0f) << 8);
        self.enabled = value & 0x80 != 0;
        if !self.enabled {
            self.step = 15;
        }
    }

    fn tick(&mut self, shift: u8, halted: bool) {
        if !self.enabled || halted {
            return;
        }
        if self.timer == 0 {
            self.timer = self.period >> shift;
            self.step = self.step.wrapping_sub(1) & 0x0f;
        } else {
            self.timer -= 1;
        }
    }

    fn output(&self) -> u8 {
        if self.enabled && (self.mode || self.step <= self.duty) {
            self.volume
        } else {
            0
        }
    }
}

#[derive(Clone, Debug, Default)]
struct Vrc6Saw {
    rate: u8,
    period: u16,
    timer: u16,
    step: u8,
    accumulator: u8,
    enabled: bool,
}

impl Vrc6Saw {
    fn write_rate(&mut self, value: u8) {
        self.rate = value & 0x3f;
    }

    fn write_period_low(&mut self, value: u8) {
        self.period = (self.period & 0x0f00) | u16::from(value);
    }

    fn write_period_high(&mut self, value: u8) {
        self.period = (self.period & 0x00ff) | (u16::from(value & 0x0f) << 8);
        self.enabled = value & 0x80 != 0;
        if !self.enabled {
            self.step = 0;
            self.accumulator = 0;
        }
    }

    fn tick(&mut self, shift: u8, halted: bool) {
        if !self.enabled {
            self.step = 0;
            self.accumulator = 0;
            return;
        }
        if halted {
            return;
        }
        if self.timer == 0 {
            self.timer = self.period >> shift;
            self.step = (self.step + 1) % 14;
            if self.step == 0 {
                self.accumulator = 0;
            } else if self.step & 1 == 0 {
                self.accumulator = self.accumulator.wrapping_add(self.rate);
            }
        } else {
            self.timer -= 1;
        }
    }

    fn output(&self) -> u8 {
        if self.enabled {
            self.accumulator >> 3
        } else {
            0
        }
    }
}

#[derive(Clone, Debug)]
struct Vrc6 {
    prg16: u8,
    prg8: u8,
    chr: [u8; 8],
    ppu_control: u8,
    pulses: [Vrc6Pulse; 2],
    saw: Vrc6Saw,
    frequency_control: u8,
    irq_latch: u8,
    irq_counter: u8,
    irq_prescaler: i16,
    irq_cycle_mode: bool,
    irq_enabled: bool,
    irq_enable_after_ack: bool,
    irq_pending: bool,
}

impl Default for Vrc6 {
    fn default() -> Self {
        Self {
            prg16: 0,
            prg8: 0,
            chr: [0; 8],
            ppu_control: 0x20,
            pulses: [Vrc6Pulse::default(), Vrc6Pulse::default()],
            saw: Vrc6Saw::default(),
            frequency_control: 0,
            irq_latch: 0,
            irq_counter: 0,
            irq_prescaler: 341,
            irq_cycle_mode: false,
            irq_enabled: false,
            irq_enable_after_ack: false,
            irq_pending: false,
        }
    }
}

impl Vrc6 {
    fn frequency_shift(&self) -> u8 {
        if self.frequency_control & 0x04 != 0 {
            8
        } else if self.frequency_control & 0x02 != 0 {
            4
        } else {
            0
        }
    }

    fn tick_audio(&mut self) {
        let halted = self.frequency_control & 1 != 0;
        let shift = self.frequency_shift();
        for pulse in &mut self.pulses {
            pulse.tick(shift, halted);
        }
        self.saw.tick(shift, halted);
    }

    fn audio_output(&self) -> f32 {
        let dac = u16::from(self.pulses[0].output())
            + u16::from(self.pulses[1].output())
            + u16::from(self.saw.output());
        let full_apu_pulse = (95.88 / (8128.0 / 15.0 + 100.0)) * 1.35;
        -(dac as f32) * (full_apu_pulse / 15.0)
    }
}

const VRC7_PATCHES: [[u8; 8]; 16] = [
    [0; 8],
    [0x03, 0x21, 0x05, 0x06, 0xe8, 0x81, 0x42, 0x27],
    [0x13, 0x41, 0x14, 0x0d, 0xd8, 0xf6, 0x23, 0x12],
    [0x11, 0x11, 0x08, 0x08, 0xfa, 0xb2, 0x20, 0x12],
    [0x31, 0x61, 0x0c, 0x07, 0xa8, 0x64, 0x61, 0x27],
    [0x32, 0x21, 0x1e, 0x06, 0xe1, 0x76, 0x01, 0x28],
    [0x02, 0x01, 0x06, 0x00, 0xa3, 0xe2, 0xf4, 0xf4],
    [0x21, 0x61, 0x1d, 0x07, 0x82, 0x81, 0x11, 0x07],
    [0x23, 0x21, 0x22, 0x17, 0xa2, 0x72, 0x01, 0x17],
    [0x35, 0x11, 0x25, 0x00, 0x40, 0x73, 0x72, 0x01],
    [0xb5, 0x01, 0x0f, 0x0f, 0xa8, 0xa5, 0x51, 0x02],
    [0x17, 0xc1, 0x24, 0x07, 0xf8, 0xf8, 0x22, 0x12],
    [0x71, 0x23, 0x11, 0x06, 0x65, 0x74, 0x18, 0x16],
    [0x01, 0x02, 0xd3, 0x05, 0xc9, 0x95, 0x03, 0x02],
    [0x61, 0x63, 0x0c, 0x00, 0x94, 0xc0, 0x33, 0xf6],
    [0x21, 0x72, 0x0d, 0x00, 0xc1, 0xd5, 0x56, 0x06],
];

#[derive(Clone, Debug, Default)]
struct Vrc7Channel {
    mod_phase: f32,
    carrier_phase: f32,
    envelope: f32,
    feedback: f32,
    key_on: bool,
    attacking: bool,
}

#[derive(Clone, Debug)]
struct Vrc7 {
    prg: [u8; 3],
    chr: [u8; 8],
    control: u8,
    irq_latch: u8,
    irq_counter: u8,
    irq_prescaler: i16,
    irq_cycle_mode: bool,
    irq_enabled: bool,
    irq_enable_after_ack: bool,
    irq_pending: bool,
    selected_audio_register: u8,
    audio_registers: [u8; 0x40],
    channels: [Vrc7Channel; 6],
    audio_divider: u8,
    lfo_phase: f32,
    audio_output: f32,
    sound_capable: bool,
}

impl Default for Vrc7 {
    fn default() -> Self {
        Self {
            prg: [0, 1, 2],
            chr: [0; 8],
            control: 0,
            irq_latch: 0,
            irq_counter: 0,
            irq_prescaler: 341,
            irq_cycle_mode: false,
            irq_enabled: false,
            irq_enable_after_ack: false,
            irq_pending: false,
            selected_audio_register: 0,
            audio_registers: [0; 0x40],
            channels: std::array::from_fn(|_| Vrc7Channel::default()),
            audio_divider: 0,
            lfo_phase: 0.0,
            audio_output: 0.0,
            sound_capable: true,
        }
    }
}

impl Vrc7 {
    fn reset_audio(&mut self) {
        self.selected_audio_register = 0;
        self.audio_registers = [0; 0x40];
        self.channels = std::array::from_fn(|_| Vrc7Channel::default());
        self.audio_divider = 0;
        self.audio_output = 0.0;
    }

    fn write_control(&mut self, value: u8) {
        self.control = value;
        if value & 0x40 != 0 {
            self.reset_audio();
        }
    }

    fn select_audio_register(&mut self, value: u8) {
        if self.sound_capable && self.control & 0x40 == 0 {
            self.selected_audio_register = value;
        }
    }

    fn write_audio_register(&mut self, value: u8) {
        if !self.sound_capable || self.control & 0x40 != 0 {
            return;
        }
        let register = usize::from(self.selected_audio_register);
        if register < self.audio_registers.len() {
            self.audio_registers[register] = value;
        }
    }

    fn patch(&self, channel: usize) -> [u8; 8] {
        let instrument = usize::from(self.audio_registers[0x30 + channel] >> 4);
        if instrument == 0 {
            std::array::from_fn(|index| self.audio_registers[index])
        } else {
            VRC7_PATCHES[instrument]
        }
    }
}

impl Vrc7 {
    fn tick_audio(&mut self, cpu_clock: f32) {
        if !self.sound_capable || self.control & 0x40 != 0 {
            self.audio_output = 0.0;
            return;
        }
        self.audio_divider = self.audio_divider.wrapping_add(1);
        if self.audio_divider < 36 {
            return;
        }
        self.audio_divider = 0;
        let dt = 36.0 / cpu_clock;
        self.lfo_phase = (self.lfo_phase + 6.1 * dt).fract();
        let vibrato = (std::f32::consts::TAU * self.lfo_phase).sin();
        let tremolo = (std::f32::consts::TAU * self.lfo_phase * 0.61).sin();
        let multipliers = [
            0.5, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 10.0, 12.0, 12.0, 15.0, 15.0,
        ];
        let mut mixed = 0.0;
        for channel in 0..6 {
            let patch = self.patch(channel);
            let low = self.audio_registers[0x10 + channel];
            let high = self.audio_registers[0x20 + channel];
            let fnum = u16::from(low) | (u16::from(high & 1) << 8);
            let block = (high >> 1) & 7;
            let key_on = high & 0x10 != 0;
            let volume = self.audio_registers[0x30 + channel] & 0x0f;
            let state = &mut self.channels[channel];

            if key_on && !state.key_on {
                state.mod_phase = 0.0;
                state.carrier_phase = 0.0;
                state.envelope = 0.0;
                state.feedback = 0.0;
                state.attacking = true;
            }
            let attack = patch[5] >> 4;
            let decay = patch[5] & 0x0f;
            let sustain = patch[7] >> 4;
            let release = patch[7] & 0x0f;
            let sustain_gain = 10.0_f32.powf(-(f32::from(sustain) * 3.0) / 20.0);
            if key_on {
                if state.attacking {
                    let seconds = 0.003 + f32::from(15 - attack) * 0.018;
                    state.envelope = (state.envelope + dt / seconds).min(1.0);
                    if state.envelope >= 1.0 {
                        state.attacking = false;
                    }
                } else if state.envelope > sustain_gain {
                    let seconds = 0.04 + f32::from(15 - decay) * 0.08;
                    state.envelope = (state.envelope - dt / seconds).max(sustain_gain);
                }
            } else {
                state.attacking = false;
                let seconds = 0.04 + f32::from(15 - release) * 0.1;
                state.envelope = (state.envelope - dt / seconds).max(0.0);
            }
            state.key_on = key_on;
            if state.envelope == 0.0 || fnum == 0 {
                continue;
            }

            let mut frequency = f32::from(fnum) * 2.0_f32.powi(i32::from(block));
            frequency *= (cpu_clock * 2.0) / (72.0 * 524_288.0);
            if patch[0] & 0x40 != 0 || patch[1] & 0x40 != 0 {
                frequency *= 1.0 + vibrato * 0.004;
            }
            let mod_mul = multipliers[usize::from(patch[0] & 0x0f)];
            let carrier_mul = multipliers[usize::from(patch[1] & 0x0f)];
            state.mod_phase = (state.mod_phase + frequency * mod_mul * dt).fract();
            state.carrier_phase = (state.carrier_phase + frequency * carrier_mul * dt).fract();
            let feedback_gain = f32::from(patch[3] & 7) * 0.16;
            let mod_level = 10.0_f32.powf(-(f32::from(patch[2] & 0x3f) * 0.75) / 20.0);
            let mod_sample =
                (std::f32::consts::TAU * state.mod_phase + state.feedback * feedback_gain).sin();
            state.feedback = mod_sample;
            let carrier =
                (std::f32::consts::TAU * state.carrier_phase + mod_sample * mod_level * 4.0).sin();
            let volume_gain = 10.0_f32.powf(-(f32::from(volume) * 3.0) / 20.0);
            let tremolo_gain = if patch[1] & 0x80 != 0 {
                0.92 + tremolo * 0.08
            } else {
                1.0
            };
            mixed += carrier * state.envelope * volume_gain * tremolo_gain;
        }
        self.audio_output = (mixed * 0.08).clamp(-0.6, 0.6);
    }
}

#[derive(Clone, Debug, Default)]
struct Vrc1 {
    prg: [u8; 3],
    chr: [u8; 2],
    mirror_horizontal: bool,
}

#[derive(Clone, Debug, Default)]
struct Vrc3 {
    irq_latch: u16,
    irq_counter: u16,
    irq_enable_after_ack: bool,
    irq_enabled: bool,
    irq_mode_8bit: bool,
    irq_pending: bool,
}

#[derive(Clone, Debug, Default)]
struct IremH3001 {
    prg: [u8; 2],
    chr: [u8; 8],
    swap_prg: bool,
    mirroring: u8,
    irq_reload: u16,
    irq_counter: u16,
    irq_enabled: bool,
    irq_pending: bool,
}

#[derive(Clone, Debug, Default)]
struct IremG101 {
    prg: [u8; 2],
    chr: [u8; 8],
    swap_prg: bool,
    mirror_horizontal: bool,
}

#[derive(Clone, Debug, Default)]
struct TaitoTc0190 {
    prg: [u8; 2],
    chr_2k: [u8; 2],
    chr_1k: [u8; 4],
    mirror_horizontal: bool,
    irq_latch: u8,
    irq_counter: u8,
    irq_reload: bool,
    irq_enabled: bool,
    irq_pending: bool,
    irq_delay: u8,
    a12_high: bool,
    a12_low_cycles: u16,
}

#[derive(Clone, Debug, Default)]
struct Sunsoft3 {
    chr_2k: [u8; 4],
    prg: u8,
    mirroring: u8,
    irq_counter: u16,
    irq_low_next: bool,
    irq_enabled: bool,
    irq_pending: bool,
}

#[derive(Clone, Debug, Default)]
struct Sunsoft4 {
    chr_2k: [u8; 4],
    nametable_chr: [u8; 2],
    nametable_control: u8,
    prg_control: u8,
}

#[derive(Clone, Debug, Default)]
struct TaitoX1005 {
    prg: [u8; 3],
    chr: [u8; 6],
    mirror_horizontal: bool,
    ram_enabled: bool,
}

#[derive(Clone, Debug, Default)]
struct TaitoX1017 {
    prg: [u8; 3],
    chr: [u8; 6],
    chr_inverted: bool,
    mirror_vertical: bool,
    ram_enabled: [bool; 3],
    irq_latch: u8,
    irq_counter: u16,
    irq_control: u8,
    irq_pending: bool,
}

#[derive(Clone, Debug, Default)]
struct JalecoJf17 {
    prg: u8,
    chr: u8,
    control: u8,
}

#[derive(Clone, Debug, Default)]
struct JalecoSs88006 {
    prg: [u8; 3],
    chr: [u8; 8],
    ram_control: u8,
    irq_reload: u16,
    irq_counter: u16,
    irq_control: u8,
    irq_pending: bool,
    mirroring: u8,
    sound_control: u8,
}

#[derive(Clone, Debug)]
struct Sunsoft5b {
    selected: u8,
    writes_disabled: bool,
    regs: [u8; 16],
    divider: u8,
    tone_counter: [u16; 3],
    tone_high: [bool; 3],
    noise_counter: u8,
    noise_lfsr: u32,
    envelope_counter: u16,
    envelope_step: u8,
    envelope_attack_mask: u8,
    envelope_hold: bool,
    envelope_alternate: bool,
    envelope_holding: bool,
}

impl Default for Sunsoft5b {
    fn default() -> Self {
        Self {
            selected: 0,
            writes_disabled: false,
            regs: [0; 16],
            divider: 0,
            tone_counter: [0; 3],
            tone_high: [false; 3],
            noise_counter: 0,
            noise_lfsr: 0x1ffff,
            envelope_counter: 0,
            envelope_step: 0x1f,
            envelope_attack_mask: 0,
            envelope_hold: true,
            envelope_alternate: false,
            envelope_holding: false,
        }
    }
}

impl Sunsoft5b {
    const VOLUME: [f32; 32] = [
        0.0,
        0.0,
        0.00668344,
        0.00794328,
        0.00944061,
        0.01122018,
        0.01333521,
        0.01584893,
        0.01883649,
        0.02238721,
        0.02660725,
        0.03162278,
        0.03758374,
        0.04466836,
        0.05308844,
        0.06309573,
        0.07498942,
        0.08912509,
        0.10592537,
        0.12589254,
        0.14962357,
        0.17782794,
        0.211_348_9,
        0.25118864,
        0.29853826,
        0.354_813_4,
        0.421_696_5,
        0.501_187_2,
        0.595_662_1,
        0.70794578,
        0.84139514,
        1.0,
    ];

    fn select(&mut self, value: u8) {
        self.selected = value & 0x0f;
        self.writes_disabled = value & 0xf0 != 0;
    }

    fn write(&mut self, value: u8) {
        if self.writes_disabled {
            return;
        }
        let register = usize::from(self.selected);
        self.regs[register] = match register {
            1 | 3 | 5 | 13 => value & 0x0f,
            6 => value & 0x1f,
            8..=10 => value & 0x1f,
            _ => value,
        };
        if register == 13 {
            self.reset_envelope(self.regs[13]);
        }
    }

    fn reset_envelope(&mut self, shape: u8) {
        let continue_flag = shape & 0x08 != 0;
        let attack = shape & 0x04 != 0;
        self.envelope_hold = shape & 0x01 != 0;
        self.envelope_alternate = shape & 0x02 != 0;
        if !continue_flag {
            self.envelope_hold = true;
            self.envelope_alternate = attack;
        }
        self.envelope_attack_mask = if attack { 0x1f } else { 0 };
        self.envelope_step = 0x1f;
        self.envelope_counter = 0;
        self.envelope_holding = false;
    }

    fn tone_period(&self, channel: usize) -> u16 {
        let low = u16::from(self.regs[channel * 2]);
        let high = u16::from(self.regs[channel * 2 + 1] & 0x0f);
        (low | (high << 8)).max(1)
    }

    fn envelope_period(&self) -> u16 {
        u16::from_le_bytes([self.regs[11], self.regs[12]]).max(1)
    }

    fn clock_envelope(&mut self) {
        if self.envelope_holding {
            return;
        }
        if self.envelope_step != 0 {
            self.envelope_step -= 1;
            return;
        }
        if self.envelope_hold {
            if self.envelope_alternate {
                self.envelope_attack_mask ^= 0x1f;
            }
            self.envelope_holding = true;
        } else {
            if self.envelope_alternate {
                self.envelope_attack_mask ^= 0x1f;
            }
            self.envelope_step = 0x1f;
        }
    }

    fn tick(&mut self) {
        self.divider = self.divider.wrapping_add(1) & 0x1f;
        if self.divider & 0x0f == 0 {
            for channel in 0..3 {
                self.tone_counter[channel] = self.tone_counter[channel].wrapping_add(1);
                if self.tone_counter[channel] >= self.tone_period(channel) {
                    self.tone_counter[channel] = 0;
                    self.tone_high[channel] = !self.tone_high[channel];
                }
            }
            self.envelope_counter = self.envelope_counter.wrapping_add(1);
            if self.envelope_counter >= self.envelope_period() {
                self.envelope_counter = 0;
                self.clock_envelope();
            }
        }
        if self.divider == 0 {
            self.noise_counter = self.noise_counter.wrapping_add(1);
            let period = (self.regs[6] & 0x1f).max(1);
            if self.noise_counter >= period {
                self.noise_counter = 0;
                let feedback = (self.noise_lfsr ^ (self.noise_lfsr >> 3)) & 1;
                self.noise_lfsr = (self.noise_lfsr >> 1) | (feedback << 16);
            }
        }
    }

    fn envelope_output(&self) -> u8 {
        self.envelope_step ^ self.envelope_attack_mask
    }

    fn output(&self) -> f32 {
        let mixer = self.regs[7];
        let noise_high = self.noise_lfsr & 1 != 0;
        let mut sum = 0.0;
        for channel in 0..3 {
            let tone_disabled = mixer & (1 << channel) != 0;
            let noise_disabled = mixer & (1 << (channel + 3)) != 0;
            if !(tone_disabled || self.tone_high[channel]) || !(noise_disabled || noise_high) {
                continue;
            }
            let volume = self.regs[8 + channel];
            let level = if volume & 0x10 != 0 {
                self.envelope_output()
            } else {
                let fixed = volume & 0x0f;
                if fixed == 0 {
                    0
                } else {
                    fixed * 2 + 1
                }
            };
            sum += Self::VOLUME[usize::from(level)];
        }
        -sum * 0.18
    }
}

#[derive(Clone, Debug, Default)]
struct Fme7 {
    command: u8,
    chr: [u8; 8],
    prg: [u8; 3],
    bank_6000: u8,
    mirroring: u8,
    irq_counter: u16,
    irq_counter_enabled: bool,
    irq_output_enabled: bool,
    irq_pending: bool,
    audio: Sunsoft5b,
}

#[derive(Clone, Copy, Debug)]
struct Mmc5Pulse {
    control: u8,
    period: u16,
    timer: u16,
    step: u8,
    length: u8,
    envelope_start: bool,
    envelope_divider: u8,
    envelope_decay: u8,
}

impl Default for Mmc5Pulse {
    fn default() -> Self {
        Self {
            control: 0,
            period: 0,
            timer: 1,
            step: 0,
            length: 0,
            envelope_start: false,
            envelope_divider: 0,
            envelope_decay: 0,
        }
    }
}

impl Mmc5Pulse {
    const DUTY: [[bool; 8]; 4] = [
        [false, true, false, false, false, false, false, false],
        [false, true, true, false, false, false, false, false],
        [false, true, true, true, true, false, false, false],
        [true, false, false, true, true, true, true, true],
    ];
    const LENGTH: [u8; 32] = [
        10, 254, 20, 2, 40, 4, 80, 6, 160, 8, 60, 10, 14, 12, 26, 14, 12, 16, 24, 18, 48, 20, 96,
        22, 192, 24, 72, 26, 16, 28, 32, 30,
    ];

    fn write_control(&mut self, value: u8) {
        self.control = value;
    }

    fn write_period_low(&mut self, value: u8) {
        self.period = (self.period & 0x0700) | u16::from(value);
    }

    fn write_period_high(&mut self, value: u8, enabled: bool) {
        self.period = (self.period & 0x00ff) | (u16::from(value & 7) << 8);
        if enabled {
            self.length = Self::LENGTH[usize::from(value >> 3)];
        }
        self.step = 0;
        self.envelope_start = true;
    }

    fn tick_timer(&mut self) {
        if self.timer > 1 {
            self.timer -= 1;
        } else {
            self.timer = self.period.saturating_add(1).saturating_mul(2).max(1);
            self.step = (self.step + 1) & 7;
        }
    }

    fn clock_quarter_frame(&mut self) {
        let period = self.control & 0x0f;
        if self.envelope_start {
            self.envelope_start = false;
            self.envelope_decay = 15;
            self.envelope_divider = period;
        } else if self.envelope_divider == 0 {
            self.envelope_divider = period;
            if self.envelope_decay != 0 {
                self.envelope_decay -= 1;
            } else if self.control & 0x20 != 0 {
                self.envelope_decay = 15;
            }
        } else {
            self.envelope_divider -= 1;
        }
        if self.control & 0x20 == 0 && self.length != 0 {
            self.length -= 1;
        }
    }

    fn output(&self, enabled: bool) -> f32 {
        if !enabled
            || self.length == 0
            || !Self::DUTY[usize::from(self.control >> 6)][usize::from(self.step)]
        {
            return 0.0;
        }
        let volume = if self.control & 0x10 != 0 {
            self.control & 0x0f
        } else {
            self.envelope_decay
        };
        -(f32::from(volume) / 15.0) * 0.12
    }
}

#[derive(Clone, Debug)]
struct Mmc5 {
    prg_mode: u8,
    chr_mode: u8,
    prg_protect: [u8; 2],
    exram_mode: u8,
    nametable_map: u8,
    fill_tile: u8,
    fill_color: u8,
    prg_ram_bank: u8,
    prg: [u8; 4],
    chr_sprite: [u16; 8],
    chr_bg: [u16; 4],
    chr_upper: u8,
    chr_io_bg: bool,
    split_control: u8,
    split_scroll: u8,
    split_bank: u8,
    irq_compare: u8,
    irq_counter: u8,
    irq_enabled: bool,
    irq_pending: bool,
    in_frame: bool,
    multiplicand: u8,
    multiplier: u8,
    pulses: [Mmc5Pulse; 2],
    pulse_enable: u8,
    audio_divider: u16,
    pcm_mode: bool,
    pcm_irq_enabled: bool,
    pcm_irq_pending: bool,
    pcm: u8,
}

impl Default for Mmc5 {
    fn default() -> Self {
        Self {
            prg_mode: 3,
            chr_mode: 3,
            prg_protect: [1, 2],
            exram_mode: 3,
            nametable_map: 0,
            fill_tile: 0,
            fill_color: 0,
            prg_ram_bank: 0,
            prg: [0, 0, 0, 0xff],
            chr_sprite: [0; 8],
            chr_bg: [0; 4],
            chr_upper: 0,
            chr_io_bg: false,
            split_control: 0,
            split_scroll: 0,
            split_bank: 0,
            irq_compare: 0,
            irq_counter: 0,
            irq_enabled: false,
            irq_pending: false,
            in_frame: false,
            multiplicand: 0,
            multiplier: 0,
            pulses: [Mmc5Pulse::default(), Mmc5Pulse::default()],
            pulse_enable: 0,
            audio_divider: 0,
            pcm_mode: false,
            pcm_irq_enabled: false,
            pcm_irq_pending: false,
            pcm: 0,
        }
    }
}

impl Mmc5 {
    fn ram_write_enabled(&self) -> bool {
        self.prg_protect[0] & 3 == 2 && self.prg_protect[1] & 3 == 1
    }

    fn latch_chr_bank(&self, value: u8) -> u16 {
        match self.chr_mode & 3 {
            3 => (u16::from(self.chr_upper & 3) << 8) | u16::from(value),
            2 => {
                (u16::from(self.chr_upper & 1) << 9)
                    | (u16::from(value) << 1)
                    | u16::from(value & 1)
            }
            1 => (u16::from(value) << 2) | (u16::from(value & 1) * 3),
            _ => (u16::from(value & 0x7f) << 3) | (u16::from(value & 1) * 7),
        }
    }

    fn tick_audio(&mut self) {
        for pulse in &mut self.pulses {
            pulse.tick_timer();
        }
        self.audio_divider += 1;
        if self.audio_divider >= 7424 {
            self.audio_divider = 0;
            for pulse in &mut self.pulses {
                pulse.clock_quarter_frame();
            }
        }
    }

    fn audio_output(&self) -> f32 {
        let pulse = self.pulses[0].output(self.pulse_enable & 1 != 0)
            + self.pulses[1].output(self.pulse_enable & 2 != 0);
        let pcm = if self.pcm == 0 {
            0.0
        } else {
            -((f32::from(self.pcm) / 255.0) - 0.5) * 0.10
        };
        pulse + pcm
    }
}

struct Cartridge {
    prg_rom: Vec<u8>,
    prg_ram: Vec<u8>,
    mapper: u16,
    submapper: u8,
    region: NesRegion,
    prg_bank: usize,
    base_mirroring: Mirroring,
    four_screen: bool,
    chr_banks_1k: usize,
    chr_is_ram: bool,
    simple_reg: u8,
    oeka_inner_chr: u8,
    oeka_last_dd: u8,
    cnrom185_reads_remaining: u8,
    nina_chr: [u8; 2],
    battery_len: usize,
    mmc1: Mmc1,
    mmc2: Mmc2,
    mmc3: Mmc3,
    rambo1: Rambo1,
    bandai: BandaiFcg,
    namco163: Namco163,
    namco210: Namco210,
    mmc5: Mmc5,
    vrc1: Vrc1,
    vrc3: Vrc3,
    irem_h3001: IremH3001,
    irem_g101: IremG101,
    taito_tc0190: TaitoTc0190,
    sunsoft3: Sunsoft3,
    sunsoft4: Sunsoft4,
    taito_x1005: TaitoX1005,
    taito_x1017: TaitoX1017,
    jaleco_jf17: JalecoJf17,
    jaleco_d7756_control: u8,
    jaleco_ss88006: JalecoSs88006,
    vrc4: Vrc4,
    vrc6: Vrc6,
    vrc7: Vrc7,
    fme7: Fme7,
}

impl Cartridge {
    fn parse(rom: &[u8]) -> Result<(Self, Ppu), String> {
        if rom.len() < 16 || &rom[..4] != b"NES\x1a" {
            return Err("NES image is not an iNES/NES 2.0 file".into());
        }
        let flags6 = rom[6];
        let flags7 = rom[7];
        let nes2 = flags7 & 0x0c == 0x08;
        let region = NesRegion::from_header(rom, nes2);
        let prg_size = if nes2 {
            nes2_rom_size(rom[4], rom[9] & 0x0f, 16 * 1024)?
        } else {
            usize::from(rom[4]) * 16 * 1024
        };
        let chr_size = if nes2 {
            nes2_rom_size(rom[5], rom[9] >> 4, 8 * 1024)?
        } else {
            usize::from(rom[5]) * 8 * 1024
        };
        let submapper = if nes2 { rom[8] >> 4 } else { 0 };
        let mapper = u16::from(flags6 >> 4)
            | u16::from(flags7 & 0xf0)
            | if nes2 {
                u16::from(rom[8] & 0x0f) << 8
            } else {
                0
            };
        if !matches!(
            mapper,
            0..=5
                | 7
                | 9
                | 10
                | 11
                | 13
                | 16
                | 18
                | 19
                | 21
                | 22
                | 23
                | 24
                | 25
                | 26
                | 32
                | 33
                | 34
                | 37
                | 47
                | 48
                | 64
                | 65
                | 66
                | 67
                | 68
                | 69
                | 70
                | 71
                | 72
                | 73
                | 75
                | 76
                | 78
                | 79
                | 80
                | 82
                | 85
                | 86
                | 87
                | 88
                | 89
                | 92
                | 93
                | 94
                | 95
                | 96
                | 97
                | 101
                | 113
                | 118
                | 119
                | 140
                | 146
                | 152
                | 153
                | 154
                | 155
                | 157
                | 158
                | 159
                | 180
                | 184
                | 185
                | 206
                | 207
                | 210
                | 232
                | 241
                | 552
        ) {
            return Err(format!(
                "NES mapper {mapper} is not implemented in OmniCore yet"
            ));
        }
        if mapper == 32 && submapper > 1 {
            return Err(format!("Irem G-101 submapper {submapper} is not defined"));
        }
        if mapper == 68 && submapper > 1 {
            return Err(format!("Sunsoft-4 submapper {submapper} is not defined"));
        }
        if mapper == 78 && !matches!(submapper, 0 | 1 | 3) {
            return Err(format!("mapper 78 submapper {submapper} is not defined"));
        }
        if mapper == 3 && submapper > 2 {
            return Err(format!("CNROM submapper {submapper} is not defined"));
        }
        if mapper == 185 && !matches!(submapper, 0 | 4 | 5 | 6 | 7) {
            return Err(format!(
                "CNROM mapper 185 submapper {submapper} is not defined"
            ));
        }
        if mapper == 16 && !matches!(submapper, 0 | 4 | 5) {
            return Err(format!(
                "Bandai mapper 16 submapper {submapper} is deprecated; use its dedicated mapper ID"
            ));
        }
        if mapper == 19 && submapper > 5 {
            return Err(format!("Namco 163 submapper {submapper} is not defined"));
        }
        if mapper == 85 && submapper > 2 {
            return Err(format!("VRC7 submapper {submapper} is not defined"));
        }
        if mapper == 210 && submapper > 2 {
            return Err(format!(
                "Namco mapper 210 submapper {submapper} is not defined"
            ));
        }
        let trainer = if flags6 & 0x04 != 0 { 512 } else { 0 };
        if matches!(mapper, 37 | 47) && trainer != 0 {
            return Err(format!(
                "Nintendo MMC3 multicart mapper {mapper} cannot use an iNES trainer because $6000-$7FFF is the outer bank register"
            ));
        }
        if matches!(mapper, 157 | 159) && trainer != 0 {
            return Err(format!(
                "Bandai mapper {mapper} cannot use an iNES trainer because $6000-$7FFF is a peripheral read port"
            ));
        }
        let offset = 16 + trainer;
        match mapper {
            0 | 3 | 185 if prg_size != 16 * 1024 && prg_size != 32 * 1024 => {
                return Err(format!(
                    "mapper {mapper} requires 16 or 32 KiB PRG ROM, got {prg_size} bytes"
                ));
            }
            5 if !(128 * 1024..=1024 * 1024).contains(&prg_size)
                || !prg_size.is_multiple_of(8 * 1024) =>
            {
                return Err(format!(
                    "MMC5 requires 128-1024 KiB of 8 KiB-banked PRG ROM, got {prg_size} bytes"
                ));
            }
            9 if prg_size < 32 * 1024 || !prg_size.is_multiple_of(8 * 1024) => {
                return Err(format!(
                    "MMC2 requires 8 KiB PRG banks and at least 32 KiB, got {prg_size} bytes"
                ));
            }
            16 | 157 | 159
                if !(32 * 1024..=256 * 1024).contains(&prg_size)
                    || !prg_size.is_multiple_of(16 * 1024) =>
            {
                return Err(format!(
                    "Bandai FCG requires 32-256 KiB of 16 KiB-banked PRG ROM, got {prg_size} bytes"
                ));
            }
            18 if !(32 * 1024..=512 * 1024).contains(&prg_size)
                || !prg_size.is_multiple_of(8 * 1024) =>
            {
                return Err(format!(
                    "Jaleco SS88006 requires 32-512 KiB of 8 KiB-banked PRG ROM, got {prg_size} bytes"
                ));
            }
            19 if !(32 * 1024..=512 * 1024).contains(&prg_size)
                || !prg_size.is_multiple_of(8 * 1024) =>
            {
                return Err(format!(
                    "Namco 163 requires 32-512 KiB of 8 KiB-banked PRG ROM, got {prg_size} bytes"
                ));
            }
            21 | 22 | 23 | 25 if prg_size < 32 * 1024 || !prg_size.is_multiple_of(8 * 1024) => {
                return Err(format!(
                    "VRC2/VRC4 mapper {mapper} requires 8 KiB PRG banks and at least 32 KiB, got {prg_size} bytes"
                ));
            }
            24 | 26 if prg_size < 32 * 1024 || !prg_size.is_multiple_of(8 * 1024) => {
                return Err(format!(
                    "VRC6 mapper {mapper} requires 8 KiB PRG banks and at least 32 KiB, got {prg_size} bytes"
                ));
            }
            32 if !(32 * 1024..=256 * 1024).contains(&prg_size)
                || !prg_size.is_multiple_of(8 * 1024) =>
            {
                return Err(format!(
                    "Irem G-101 requires 32-256 KiB of 8 KiB-banked PRG ROM, got {prg_size} bytes"
                ));
            }
            37 | 47 if prg_size != 256 * 1024 => {
                return Err(format!(
                    "Nintendo MMC3 multicart mapper {mapper} requires exactly 256 KiB PRG ROM, got {prg_size} bytes"
                ));
            }
            33 | 48
                if !(32 * 1024..=512 * 1024).contains(&prg_size)
                    || !prg_size.is_multiple_of(8 * 1024) =>
            {
                return Err(format!(
                    "Taito TC0190/TC0690 mapper {mapper} requires 32-512 KiB of 8 KiB-banked PRG ROM, got {prg_size} bytes"
                ));
            }
            80 | 207
                if !(32 * 1024..=512 * 1024).contains(&prg_size)
                    || !prg_size.is_multiple_of(8 * 1024) =>
            {
                return Err(format!(
                    "Taito X1-005 mapper {mapper} requires 32-512 KiB of 8 KiB-banked PRG ROM, got {prg_size} bytes"
                ));
            }
            82 if !(32 * 1024..=128 * 1024).contains(&prg_size)
                || !prg_size.is_multiple_of(8 * 1024) =>
            {
                return Err(format!(
                    "Taito X1-017 mapper 82 requires 32-128 KiB of 8 KiB-banked PRG ROM, got {prg_size} bytes"
                ));
            }
            552 if !(32 * 1024..=512 * 1024).contains(&prg_size)
                || !prg_size.is_multiple_of(8 * 1024) =>
            {
                return Err(format!(
                    "Taito X1-017 mapper 552 requires 32-512 KiB of 8 KiB-banked PRG ROM, got {prg_size} bytes"
                ));
            }
            65 if !(32 * 1024..=512 * 1024).contains(&prg_size)
                || !prg_size.is_multiple_of(8 * 1024) =>
            {
                return Err(format!(
                    "Irem H3001 requires 32-512 KiB of 8 KiB-banked PRG ROM, got {prg_size} bytes"
                ));
            }
            64 | 158
                if !(32 * 1024..=256 * 1024).contains(&prg_size)
                    || !prg_size.is_multiple_of(8 * 1024) =>
            {
                return Err(format!(
                    "RAMBO-1 mapper {mapper} requires 32-256 KiB of 8 KiB-banked PRG ROM, got {prg_size} bytes"
                ));
            }
            67 if !(32 * 1024..=256 * 1024).contains(&prg_size)
                || !prg_size.is_multiple_of(16 * 1024) =>
            {
                return Err(format!(
                    "Sunsoft-3 requires 32-256 KiB of 16 KiB-banked PRG ROM, got {prg_size} bytes"
                ));
            }
            68 if !(32 * 1024..=256 * 1024).contains(&prg_size)
                || !prg_size.is_multiple_of(16 * 1024) =>
            {
                return Err(format!(
                    "Sunsoft-4 requires 32-256 KiB of 16 KiB-banked PRG ROM, got {prg_size} bytes"
                ));
            }
            70 if !(32 * 1024..=256 * 1024).contains(&prg_size)
                || !prg_size.is_multiple_of(16 * 1024) =>
            {
                return Err(format!(
                    "Bandai 74161/32 mapper 70 requires 32-256 KiB of 16 KiB-banked PRG ROM, got {prg_size} bytes"
                ));
            }
            152 if !(32 * 1024..=128 * 1024).contains(&prg_size)
                || !prg_size.is_multiple_of(16 * 1024) =>
            {
                return Err(format!(
                    "Bandai 74161/32 mapper 152 requires 32-128 KiB of 16 KiB-banked PRG ROM, got {prg_size} bytes"
                ));
            }
            153 if prg_size != 256 * 1024 && prg_size != 512 * 1024 => {
                return Err(format!(
                    "Bandai LZ93D50 mapper 153 requires 256 or 512 KiB PRG ROM, got {prg_size} bytes"
                ));
            }
            78 if !(32 * 1024..=128 * 1024).contains(&prg_size)
                || !prg_size.is_multiple_of(16 * 1024) =>
            {
                return Err(format!(
                    "mapper 78 requires 32-128 KiB of 16 KiB-banked PRG ROM, got {prg_size} bytes"
                ));
            }
            72 if !(32 * 1024..=128 * 1024).contains(&prg_size)
                || !prg_size.is_multiple_of(16 * 1024) =>
            {
                return Err(format!(
                    "Jaleco JF-17 requires 32-128 KiB of 16 KiB-banked PRG ROM, got {prg_size} bytes"
                ));
            }
            92 if !(32 * 1024..=256 * 1024).contains(&prg_size)
                || !prg_size.is_multiple_of(16 * 1024) =>
            {
                return Err(format!(
                    "Jaleco JF-19 requires 32-256 KiB of 16 KiB-banked PRG ROM, got {prg_size} bytes"
                ));
            }
            89 | 93
                if !(32 * 1024..=128 * 1024).contains(&prg_size)
                    || !prg_size.is_multiple_of(16 * 1024) =>
            {
                return Err(format!(
                    "Sunsoft-2 mapper {mapper} requires 32-128 KiB of 16 KiB-banked PRG ROM, got {prg_size} bytes"
                ));
            }
            96 if prg_size != 128 * 1024 => {
                return Err(format!(
                    "Oeka Kids mapper 96 requires exactly 128 KiB PRG ROM, got {prg_size} bytes"
                ));
            }
            97 if !(32 * 1024..=512 * 1024).contains(&prg_size)
                || !prg_size.is_multiple_of(16 * 1024) =>
            {
                return Err(format!(
                    "Irem TAM-S1 mapper 97 requires 32-512 KiB of 16 KiB-banked PRG ROM, got {prg_size} bytes"
                ));
            }
            69 if prg_size < 32 * 1024 || !prg_size.is_multiple_of(8 * 1024) => {
                return Err(format!(
                    "FME-7 requires 8 KiB PRG banks and at least 32 KiB, got {prg_size} bytes"
                ));
            }
            85 if !(32 * 1024..=512 * 1024).contains(&prg_size)
                || !prg_size.is_multiple_of(8 * 1024) =>
            {
                return Err(format!(
                    "VRC7 requires 32-512 KiB of 8 KiB-banked PRG ROM, got {prg_size} bytes"
                ));
            }
            86 if prg_size != 128 * 1024 => {
                return Err(format!(
                    "Jaleco JF-13 mapper 86 requires exactly 128 KiB PRG ROM, got {prg_size} bytes"
                ));
            }
            75 | 76 | 88 | 95 | 154 | 206
                if prg_size < 32 * 1024 || !prg_size.is_multiple_of(8 * 1024) =>
            {
                return Err(format!(
                    "mapper {mapper} requires 8 KiB PRG banks and at least 32 KiB, got {prg_size} bytes"
                ));
            }
            1 | 2 | 4 | 10 | 71 | 73 | 94 | 118 | 119 | 155 | 180
                if prg_size < 32 * 1024 || !prg_size.is_multiple_of(16 * 1024) =>
            {
                return Err(format!(
                    "mapper {mapper} requires 16 KiB PRG banks and at least 32 KiB, got {prg_size} bytes"
                ));
            }
            79 | 146
                if !(32 * 1024..=64 * 1024).contains(&prg_size)
                    || !prg_size.is_multiple_of(32 * 1024) =>
            {
                return Err(format!(
                    "mapper {mapper} requires 32 or 64 KiB of 32 KiB-banked PRG ROM, got {prg_size} bytes"
                ));
            }
            113 if !(32 * 1024..=256 * 1024).contains(&prg_size)
                || !prg_size.is_multiple_of(32 * 1024) =>
            {
                return Err(format!(
                    "mapper 113 requires 32-256 KiB of 32 KiB-banked PRG ROM, got {prg_size} bytes"
                ));
            }
            232 if !(64 * 1024..=256 * 1024).contains(&prg_size)
                || !prg_size.is_multiple_of(64 * 1024) =>
            {
                return Err(format!(
                    "mapper 232 requires 64-256 KiB of PRG ROM, got {prg_size} bytes"
                ));
            }
            7 | 11 | 34 | 66 | 140 | 241
                if prg_size < 32 * 1024 || !prg_size.is_multiple_of(32 * 1024) =>
            {
                return Err(format!(
                    "mapper {mapper} requires 32 KiB PRG banks, got {prg_size} bytes"
                ));
            }
            13 if prg_size != 32 * 1024 => {
                return Err(format!(
                    "CPROM requires 32 KiB PRG ROM, got {prg_size} bytes"
                ));
            }
            87 | 101 | 184 if prg_size != 16 * 1024 && prg_size != 32 * 1024 => {
                return Err(format!(
                    "mapper {mapper} requires 16 or 32 KiB fixed PRG ROM, got {prg_size} bytes"
                ));
            }
            210 if !(32 * 1024..=512 * 1024).contains(&prg_size)
                || !prg_size.is_multiple_of(8 * 1024) =>
            {
                return Err(format!(
                    "Namco 175/340 requires 32-512 KiB of 8 KiB-banked PRG ROM, got {prg_size} bytes"
                ));
            }
            _ => {}
        }
        if matches!(mapper, 76 | 88 | 95 | 154 | 206) && prg_size > 128 * 1024 {
            return Err(format!(
                "Namco 108-family mapper {mapper} supports at most 128 KiB PRG ROM"
            ));
        }
        if mapper == 3 && chr_size == 0 {
            return Err("CNROM requires CHR ROM".into());
        }
        if mapper == 185 && chr_size != 8 * 1024 {
            return Err("CNROM mapper 185 requires exactly 8 KiB CHR ROM".into());
        }
        if matches!(mapper, 9 | 10) && (chr_size < 8 * 1024 || !chr_size.is_multiple_of(4 * 1024)) {
            return Err(format!("mapper {mapper} requires 4 KiB-banked CHR ROM"));
        }
        if mapper == 5
            && chr_size != 0
            && (!(8 * 1024..=1024 * 1024).contains(&chr_size) || !chr_size.is_multiple_of(1024))
        {
            return Err("MMC5 requires CHR RAM or up to 1024 KiB of banked CHR ROM".into());
        }
        if matches!(mapper, 16 | 159)
            && (!(8 * 1024..=256 * 1024).contains(&chr_size) || !chr_size.is_multiple_of(1024))
        {
            return Err("Bandai FCG requires 8-256 KiB of 1 KiB-banked CHR ROM".into());
        }
        if matches!(mapper, 153 | 157) && chr_size != 0 {
            return Err(format!("Bandai mapper {mapper} requires 8 KiB CHR RAM"));
        }
        if mapper == 18
            && (!(8 * 1024..=256 * 1024).contains(&chr_size) || !chr_size.is_multiple_of(1024))
        {
            return Err("Jaleco SS88006 requires 8-256 KiB of 1 KiB-banked CHR ROM".into());
        }
        if mapper == 19
            && chr_size != 0
            && (!(8 * 1024..=256 * 1024).contains(&chr_size) || !chr_size.is_multiple_of(1024))
        {
            return Err("Namco 163 requires CHR RAM or 8-256 KiB of 1 KiB-banked CHR ROM".into());
        }
        if matches!(mapper, 24 | 26) && (chr_size < 8 * 1024 || !chr_size.is_multiple_of(1024)) {
            return Err(format!(
                "VRC6 mapper {mapper} requires 1 KiB-banked CHR ROM"
            ));
        }
        if mapper == 75 && (chr_size < 8 * 1024 || !chr_size.is_multiple_of(4 * 1024)) {
            return Err("VRC1 requires 4 KiB-banked CHR ROM".into());
        }
        if mapper == 32
            && (!(8 * 1024..=256 * 1024).contains(&chr_size) || !chr_size.is_multiple_of(1024))
        {
            return Err("Irem G-101 requires 8-256 KiB of 1 KiB-banked CHR ROM".into());
        }
        if matches!(mapper, 37 | 47) && chr_size != 256 * 1024 {
            return Err(format!(
                "Nintendo MMC3 multicart mapper {mapper} requires exactly 256 KiB CHR ROM"
            ));
        }
        if matches!(mapper, 33 | 48)
            && chr_size != 0
            && (!(8 * 1024..=512 * 1024).contains(&chr_size) || !chr_size.is_multiple_of(1024))
        {
            return Err(format!(
                "Taito TC0190/TC0690 mapper {mapper} requires CHR RAM or 8-512 KiB of banked CHR ROM"
            ));
        }
        if mapper == 80
            && (!(8 * 1024..=256 * 1024).contains(&chr_size) || !chr_size.is_multiple_of(1024))
        {
            return Err("Taito X1-005 mapper 80 requires 8-256 KiB of 1 KiB-banked CHR ROM".into());
        }
        if mapper == 207
            && (!(8 * 1024..=128 * 1024).contains(&chr_size) || !chr_size.is_multiple_of(1024))
        {
            return Err(
                "Taito X1-005 mapper 207 requires 8-128 KiB of 1 KiB-banked CHR ROM".into(),
            );
        }
        if matches!(mapper, 82 | 552)
            && (!(8 * 1024..=256 * 1024).contains(&chr_size) || !chr_size.is_multiple_of(1024))
        {
            return Err(format!(
                "Taito X1-017 mapper {mapper} requires 8-256 KiB of 1 KiB-banked CHR ROM"
            ));
        }
        if mapper == 67
            && chr_size != 0
            && (!(8 * 1024..=128 * 1024).contains(&chr_size) || !chr_size.is_multiple_of(2 * 1024))
        {
            return Err("Sunsoft-3 requires CHR RAM or 8-128 KiB of 2 KiB-banked CHR ROM".into());
        }
        if mapper == 68
            && (!(8 * 1024..=256 * 1024).contains(&chr_size) || !chr_size.is_multiple_of(2 * 1024))
        {
            return Err("Sunsoft-4 requires 8-256 KiB of 2 KiB-banked CHR ROM".into());
        }
        if matches!(mapper, 70 | 152)
            && chr_size != 0
            && (!(8 * 1024..=128 * 1024).contains(&chr_size) || !chr_size.is_multiple_of(8 * 1024))
        {
            return Err(format!(
                "Bandai 74161/32 mapper {mapper} requires CHR RAM or 8-128 KiB of 8 KiB-banked CHR ROM"
            ));
        }
        if mapper == 78
            && chr_size != 0
            && (!(8 * 1024..=128 * 1024).contains(&chr_size) || !chr_size.is_multiple_of(8 * 1024))
        {
            return Err("mapper 78 requires CHR RAM or 8-128 KiB of 8 KiB-banked CHR ROM".into());
        }
        if matches!(mapper, 72 | 92)
            && (!(8 * 1024..=128 * 1024).contains(&chr_size) || !chr_size.is_multiple_of(8 * 1024))
        {
            return Err(format!(
                "Jaleco JF mapper {mapper} requires 8-128 KiB of 8 KiB-banked CHR ROM"
            ));
        }
        if mapper == 89
            && (!(8 * 1024..=128 * 1024).contains(&chr_size) || !chr_size.is_multiple_of(8 * 1024))
        {
            return Err("Sunsoft-2 mapper 89 requires 8-128 KiB of 8 KiB-banked CHR ROM".into());
        }
        if mapper == 93 && chr_size != 0 {
            return Err("Sunsoft-2 mapper 93 requires CHR RAM".into());
        }
        if mapper == 96 && chr_size != 0 {
            return Err("Oeka Kids mapper 96 requires 32 KiB CHR RAM".into());
        }
        if mapper == 97 && chr_size != 0 && chr_size != 8 * 1024 {
            return Err("Irem TAM-S1 mapper 97 supports fixed 8 KiB CHR ROM or CHR RAM".into());
        }
        if mapper == 65
            && chr_size != 0
            && (!(8 * 1024..=256 * 1024).contains(&chr_size) || !chr_size.is_multiple_of(1024))
        {
            return Err("Irem H3001 requires CHR RAM or 8-256 KiB of 1 KiB-banked CHR ROM".into());
        }
        if matches!(mapper, 64 | 158)
            && (!(8 * 1024..=256 * 1024).contains(&chr_size) || !chr_size.is_multiple_of(1024))
        {
            return Err(format!(
                "RAMBO-1 mapper {mapper} requires 8-256 KiB of 1 KiB-banked CHR ROM"
            ));
        }
        if mapper == 76
            && (!(8 * 1024..=128 * 1024).contains(&chr_size) || !chr_size.is_multiple_of(2 * 1024))
        {
            return Err("Namco 109 requires up to 128 KiB of 2 KiB-banked CHR ROM".into());
        }
        if matches!(mapper, 88 | 154)
            && (!(8 * 1024..=128 * 1024).contains(&chr_size) || !chr_size.is_multiple_of(1024))
        {
            return Err(format!(
                "mapper {mapper} requires up to 128 KiB of banked CHR ROM"
            ));
        }
        if matches!(mapper, 95 | 206)
            && (!(8 * 1024..=64 * 1024).contains(&chr_size) || !chr_size.is_multiple_of(1024))
        {
            return Err(format!(
                "mapper {mapper} requires up to 64 KiB of banked CHR ROM"
            ));
        }
        if mapper == 85
            && chr_size != 0
            && (!(8 * 1024..=256 * 1024).contains(&chr_size) || !chr_size.is_multiple_of(1024))
        {
            return Err("VRC7 requires CHR RAM or 8-256 KiB of 1 KiB-banked CHR ROM".into());
        }
        if mapper == 119
            && (!(8 * 1024..=128 * 1024).contains(&chr_size) || !chr_size.is_multiple_of(1024))
        {
            return Err("TQROM requires 8-128 KiB of 1 KiB-banked CHR ROM".into());
        }
        if mapper == 210
            && (!(8 * 1024..=256 * 1024).contains(&chr_size) || !chr_size.is_multiple_of(1024))
        {
            return Err("Namco 175/340 requires 8-256 KiB of 1 KiB-banked CHR ROM".into());
        }
        if matches!(mapper, 79 | 146)
            && (!(8 * 1024..=64 * 1024).contains(&chr_size) || !chr_size.is_multiple_of(8 * 1024))
        {
            return Err(format!(
                "mapper {mapper} requires 8-64 KiB of 8 KiB-banked CHR ROM"
            ));
        }
        if mapper == 113
            && (!(8 * 1024..=128 * 1024).contains(&chr_size) || !chr_size.is_multiple_of(8 * 1024))
        {
            return Err("mapper 113 requires 8-128 KiB of 8 KiB-banked CHR ROM".into());
        }
        if matches!(mapper, 13 | 71 | 73 | 94 | 180 | 232 | 241) && chr_size != 0 {
            return Err(format!("mapper {mapper} requires CHR RAM"));
        }
        if mapper == 86 && chr_size != 64 * 1024 {
            return Err("Jaleco JF-13 mapper 86 requires exactly 64 KiB CHR ROM".into());
        }
        if matches!(mapper, 87 | 101)
            && (chr_size < 16 * 1024 || !chr_size.is_multiple_of(8 * 1024))
        {
            return Err(format!("mapper {mapper} requires banked CHR ROM"));
        }
        if mapper == 140 && (chr_size == 0 || !chr_size.is_multiple_of(8 * 1024)) {
            return Err("mapper 140 requires 8 KiB-banked CHR ROM".into());
        }
        if mapper == 184 && chr_size < 32 * 1024 {
            return Err("mapper 184 requires at least 32 KiB CHR ROM".into());
        }
        if rom.len() < offset + prg_size + chr_size {
            return Err("NES image is truncated".into());
        }
        let prg_rom = rom[offset..offset + prg_size].to_vec();
        let chr_start = offset + prg_size;
        let chr_ram_size = if chr_size == 0 {
            let declared = if nes2 {
                ram_size_from_shift(rom[11] & 0x0f) + ram_size_from_shift(rom[11] >> 4)
            } else {
                0
            };
            let size = declared.max(match mapper {
                13 => 16 * 1024,
                96 => 32 * 1024,
                _ => 8 * 1024,
            });
            if matches!(mapper, 153 | 157) && size != 8 * 1024 {
                return Err(format!(
                    "Bandai mapper {mapper} requires exactly 8 KiB CHR RAM, got {size} bytes"
                ));
            }
            size
        } else {
            0
        };
        let tqrom_chr_ram_size = if mapper == 119 {
            let declared = if nes2 {
                ram_size_from_shift(rom[11] & 0x0f) + ram_size_from_shift(rom[11] >> 4)
            } else {
                0
            };
            if declared != 0 && declared != 8 * 1024 {
                return Err(format!(
                    "TQROM requires 8 KiB CHR RAM, got {declared} bytes"
                ));
            }
            8 * 1024
        } else {
            0
        };
        let chr = if chr_size == 0 {
            vec![0; chr_ram_size]
        } else {
            rom[chr_start..chr_start + chr_size].to_vec()
        };
        let chr_banks_1k = ((if chr_size == 0 {
            chr_ram_size
        } else {
            chr_size
        }) / 0x400)
            .max(1);
        let four_screen = flags6 & 0x08 != 0;
        let mirroring = if four_screen {
            Mirroring::FourScreen
        } else if flags6 & 1 != 0 {
            Mirroring::Vertical
        } else {
            Mirroring::Horizontal
        };
        let mut ppu = Ppu::new(chr, chr_size == 0, mirroring);
        if tqrom_chr_ram_size != 0 {
            ppu.append_chr_ram(tqrom_chr_ram_size);
        }
        ppu.region = region;
        let (mut prg_ram_size, mut battery_len) = if nes2 {
            let volatile = ram_size_from_shift(rom[10] & 0x0f);
            let nonvolatile = ram_size_from_shift(rom[10] >> 4);
            let total = volatile.saturating_add(nonvolatile);
            let battery = if nonvolatile != 0 {
                nonvolatile
            } else if flags6 & 0x02 != 0 {
                total
            } else {
                0
            };
            (total, battery)
        } else {
            let total = usize::from(rom[8].max(1)) * 8 * 1024;
            (total, if flags6 & 0x02 != 0 { total } else { 0 })
        };
        let namco210_is_175 = mapper == 210
            && match submapper {
                1 => true,
                2 => false,
                _ if nes2 => prg_ram_size != 0,
                _ => flags6 & 0x02 != 0 || rom[8] != 0,
            };
        if mapper == 210 && !namco210_is_175 {
            prg_ram_size = 0;
            battery_len = 0;
        }
        let bandai_external_eeprom_present = mapper == 157
            && if nes2 {
                let volatile = ram_size_from_shift(rom[10] & 0x0f);
                let nonvolatile = ram_size_from_shift(rom[10] >> 4);
                if volatile != 0 || !matches!(nonvolatile, 0 | 128) {
                    return Err(format!(
                        "Bandai mapper 157 allows only an optional 128-byte external EEPROM, got {volatile} volatile and {nonvolatile} nonvolatile bytes"
                    ));
                }
                nonvolatile == 128
            } else {
                true
            };
        if mapper == 159 && nes2 {
            let volatile = ram_size_from_shift(rom[10] & 0x0f);
            let nonvolatile = ram_size_from_shift(rom[10] >> 4);
            if volatile != 0 || !matches!(nonvolatile, 0 | 128) {
                return Err(format!(
                    "Bandai mapper 159 requires 128 bytes of PRG-NVRAM and no PRG-RAM, got {volatile} volatile and {nonvolatile} nonvolatile bytes"
                ));
            }
        }
        let bandai_eeprom_present = matches!(mapper, 157 | 159)
            || (mapper == 16
                && if nes2 {
                    matches!(submapper, 0 | 5) && battery_len == 256
                } else {
                    flags6 & 0x02 != 0
                });
        if mapper == 5 && !nes2 {
            // Legacy iNES did not reliably describe ExROM RAM population. A 64 KiB
            // superset preserves the banked-RAM behavior expected by licensed MMC5 games.
            prg_ram_size = prg_ram_size.max(64 * 1024);
        }
        if mapper == 153 {
            prg_ram_size = 8 * 1024;
            battery_len = 8 * 1024;
        }
        if matches!(mapper, 119 | 185) {
            prg_ram_size = 0;
            battery_len = 0;
        }
        if matches!(
            mapper,
            37 | 47 | 48 | 67 | 70 | 72 | 78 | 86 | 87 | 89 | 92 | 93 | 96 | 97 | 101 | 152
        ) && trainer == 0
        {
            prg_ram_size = 0;
            battery_len = 0;
        }
        if matches!(mapper, 80 | 207) {
            prg_ram_size = 128;
            battery_len = if battery_len != 0 || flags6 & 0x02 != 0 {
                128
            } else {
                0
            };
        }
        if matches!(mapper, 82 | 552) {
            prg_ram_size = 5 * 1024;
            battery_len = if battery_len != 0 || flags6 & 0x02 != 0 {
                5 * 1024
            } else {
                0
            };
        }
        if matches!(mapper, 16 | 64 | 157 | 158 | 159) && trainer == 0 {
            prg_ram_size = 0;
            if mapper == 157 {
                battery_len = 0;
            }
        }
        if mapper == 4 && submapper == 1 {
            prg_ram_size = prg_ram_size.max(1024);
        } else if trainer != 0 {
            prg_ram_size = prg_ram_size.max(8 * 1024);
        }
        let mut prg_ram = vec![0; prg_ram_size];
        if trainer != 0 {
            if mapper == 4 && submapper == 1 {
                prg_ram[..512].copy_from_slice(&rom[16..16 + 512]);
            } else {
                prg_ram[0x1000..0x1200].copy_from_slice(&rom[16..16 + 512]);
            }
        }
        let cartridge = Self {
            prg_rom,
            prg_ram,
            mapper,
            submapper,
            region,
            prg_bank: 0,
            base_mirroring: mirroring,
            four_screen,
            chr_banks_1k,
            chr_is_ram: chr_size == 0,
            simple_reg: if (mapper == 97 && matches!(mirroring, Mirroring::Vertical))
                || (mapper == 71 && submapper == 1)
            {
                0x80
            } else if mapper == 232 {
                0x18
            } else {
                0
            },
            oeka_inner_chr: 0,
            oeka_last_dd: 0xff,
            cnrom185_reads_remaining: u8::from(mapper == 185 && submapper == 0) * 2,
            nina_chr: [0, 1],
            battery_len: if matches!(mapper, 16 | 64 | 157 | 158 | 159) {
                0
            } else {
                battery_len
            },
            mmc1: Mmc1::default(),
            mmc2: Mmc2::default(),
            mmc3: Mmc3::default(),
            rambo1: Rambo1 {
                mirror_horizontal: matches!(mirroring, Mirroring::Horizontal),
                ..Rambo1::default()
            },
            bandai: BandaiFcg {
                mirroring: u8::from(matches!(mirroring, Mirroring::Horizontal)),
                eeprom_present: bandai_eeprom_present,
                external_eeprom_present: bandai_external_eeprom_present,
                ..BandaiFcg::default()
            },
            namco163: Namco163 {
                sound_capable: mapper == 19 && !matches!(submapper, 1 | 2),
                internal_battery: mapper == 19 && flags6 & 0x02 != 0,
                submapper,
                ..Namco163::default()
            },
            namco210: Namco210 {
                prg: [0, 1, 2],
                mirroring: match mirroring {
                    Mirroring::Vertical => 1,
                    Mirroring::Horizontal => 3,
                    Mirroring::SingleScreen1 => 2,
                    _ => 0,
                },
                is_175: namco210_is_175,
                ..Namco210::default()
            },
            mmc5: Mmc5::default(),
            vrc1: Vrc1::default(),
            vrc3: Vrc3::default(),
            irem_h3001: IremH3001 {
                prg: [0, 1],
                mirroring: if matches!(mirroring, Mirroring::Horizontal) {
                    2
                } else {
                    0
                },
                ..IremH3001::default()
            },
            irem_g101: IremG101::default(),
            taito_tc0190: TaitoTc0190 {
                mirror_horizontal: matches!(mirroring, Mirroring::Horizontal),
                ..TaitoTc0190::default()
            },
            sunsoft3: Sunsoft3 {
                mirroring: match mirroring {
                    Mirroring::Vertical => 0,
                    Mirroring::Horizontal => 1,
                    Mirroring::SingleScreen0 => 2,
                    Mirroring::SingleScreen1 => 3,
                    Mirroring::FourScreen => 0,
                },
                ..Sunsoft3::default()
            },
            sunsoft4: Sunsoft4 {
                nametable_control: match mirroring {
                    Mirroring::Vertical => 0,
                    Mirroring::Horizontal => 1,
                    Mirroring::SingleScreen0 => 2,
                    Mirroring::SingleScreen1 => 3,
                    Mirroring::FourScreen => 0,
                },
                ..Sunsoft4::default()
            },
            taito_x1005: TaitoX1005 {
                mirror_horizontal: matches!(mirroring, Mirroring::Horizontal),
                ..TaitoX1005::default()
            },
            taito_x1017: TaitoX1017 {
                mirror_vertical: matches!(mirroring, Mirroring::Vertical),
                ..TaitoX1017::default()
            },
            jaleco_jf17: JalecoJf17::default(),
            jaleco_d7756_control: 0,
            jaleco_ss88006: JalecoSs88006 {
                prg: [0, 1, 2],
                chr: [0, 1, 2, 3, 4, 5, 6, 7],
                mirroring: u8::from(matches!(mirroring, Mirroring::Vertical)),
                ..JalecoSs88006::default()
            },
            vrc4: Vrc4::default(),
            vrc6: Vrc6::default(),
            vrc7: Vrc7 {
                sound_capable: mapper == 85 && submapper != 1,
                ..Vrc7::default()
            },
            fme7: Fme7::default(),
        };
        cartridge.sync_ppu(&mut ppu);
        Ok((cartridge, ppu))
    }

    fn is_vrc2(&self) -> bool {
        self.mapper == 22
            || (self.mapper == 23 && self.submapper == 3)
            || (self.mapper == 25 && self.submapper == 3)
    }

    fn is_vrc4(&self) -> bool {
        matches!(self.mapper, 21 | 23 | 25) && !self.is_vrc2()
    }

    fn is_vrc6(&self) -> bool {
        matches!(self.mapper, 24 | 26)
    }

    fn vrc2_uses_latch(&self) -> bool {
        self.mapper == 22 || (self.mapper == 23 && self.submapper == 3)
    }

    fn vrc_subaddress(&self, address: u16) -> u8 {
        let bit_pair =
            |a0: u8, a1: u8| -> u8 { (((address >> a0) & 1) | (((address >> a1) & 1) << 1)) as u8 };
        match (self.mapper, self.submapper) {
            (22, _) | (25, 1 | 3) => bit_pair(1, 0),
            (21, 1) => bit_pair(1, 2),
            (21, 2) => bit_pair(6, 7),
            (23, 1 | 3) => bit_pair(0, 1),
            (23, 2) => bit_pair(2, 3),
            (25, 2) => bit_pair(3, 2),
            (21, 0) => {
                if address & 0x00c0 != 0 {
                    bit_pair(6, 7)
                } else {
                    bit_pair(1, 2)
                }
            }
            (23, 0) => {
                if address & 0x000c != 0 {
                    bit_pair(2, 3)
                } else {
                    bit_pair(0, 1)
                }
            }
            (25, 0) => {
                if address & 0x000c != 0 {
                    bit_pair(3, 2)
                } else {
                    bit_pair(1, 0)
                }
            }
            _ => 0,
        }
    }

    fn vrc6_subaddress(&self, address: u16) -> u8 {
        let low = (address & 3) as u8;
        if self.mapper == 26 {
            (low & !3) | ((low & 1) << 1) | ((low & 2) >> 1)
        } else {
            low
        }
    }

    fn read_prg(&self, address: u16) -> u8 {
        debug_assert!(address >= 0x8000);
        match self.mapper {
            1 | 155 => self.read_mmc1_prg(address),
            16 | 153 | 157 | 159 => self.read_bandai_prg(address),
            18 => self.read_jaleco_ss88006_prg(address),
            19 => self.read_namco163_prg(address),
            21 | 22 | 23 | 25 => self.read_vrc_prg(address),
            24 | 26 => self.read_vrc6_prg(address),
            32 => self.read_irem_g101_prg(address),
            65 => self.read_irem_h3001_prg(address),
            85 => self.read_vrc7_prg(address),
            33 | 48 => self.read_taito_tc0190_prg(address),
            64 | 158 => self.read_rambo1_prg(address),
            67 => self.read_sunsoft3_prg(address),
            68 => self.read_sunsoft4_prg(address),
            69 => self.read_fme7_prg(address),
            73 => self.read_vrc3_prg(address),
            75 => self.read_vrc1_prg(address),
            76 | 88 | 95 | 154 | 206 => self.read_namco108_prg(address),
            80 | 207 => self.read_taito_x1005_prg(address),
            82 | 552 => self.read_taito_x1017_prg(address),
            210 => self.read_namco210_prg(address),
            9 => {
                let banks = self.prg_rom.len() / 0x2000;
                let slot = (address as usize - 0x8000) / 0x2000;
                let bank = if slot == 0 {
                    self.prg_bank % banks
                } else {
                    banks - 4 + slot
                };
                self.prg_rom[bank * 0x2000 + (address as usize & 0x1fff)]
            }
            10 => {
                let banks = self.prg_rom.len() / 0x4000;
                let bank = if address < 0xc000 {
                    self.prg_bank % banks
                } else {
                    banks - 1
                };
                self.prg_rom[bank * 0x4000 + (address as usize & 0x3fff)]
            }
            2 | 71 | 94 => {
                let banks = self.prg_rom.len() / 0x4000;
                let bank = if address < 0xc000 {
                    self.prg_bank % banks
                } else {
                    banks - 1
                };
                self.prg_rom[bank * 0x4000 + (address as usize & 0x3fff)]
            }
            72 | 92 => {
                let banks = self.prg_rom.len() / 0x4000;
                let selected = usize::from(self.jaleco_jf17.prg) % banks;
                let bank = match (self.mapper, address < 0xc000) {
                    (72, true) => selected,
                    (72, false) => banks - 1,
                    (92, true) => 0,
                    (92, false) => selected,
                    _ => unreachable!(),
                };
                self.prg_rom[bank * 0x4000 + (address as usize & 0x3fff)]
            }
            97 => {
                let banks = self.prg_rom.len() / 0x4000;
                let bank = if address < 0xc000 {
                    banks - 1
                } else {
                    usize::from(self.simple_reg & 0x1f) % banks
                };
                self.prg_rom[bank * 0x4000 + (address as usize & 0x3fff)]
            }
            70 | 78 | 89 | 93 | 152 => {
                let banks = self.prg_rom.len() / 0x4000;
                let selected = if self.mapper == 78 {
                    self.simple_reg & 7
                } else if matches!(self.mapper, 89 | 93) || self.bandai_74161_one_screen() {
                    (self.simple_reg >> 4) & 7
                } else {
                    self.simple_reg >> 4
                };
                let bank = if address < 0xc000 {
                    usize::from(selected) % banks
                } else {
                    banks - 1
                };
                self.prg_rom[bank * 0x4000 + (address as usize & 0x3fff)]
            }
            180 => {
                let banks = self.prg_rom.len() / 0x4000;
                let bank = if address < 0xc000 {
                    0
                } else {
                    self.prg_bank % banks
                };
                self.prg_rom[bank * 0x4000 + (address as usize & 0x3fff)]
            }
            4 | 37 | 47 | 118 | 119 => self.read_mmc3_prg(address),
            5 => self.read_mmc5_prg(address),
            7 | 11 | 34 | 66 | 79 | 86 | 96 | 113 | 140 | 146 | 241 => {
                let banks = self.prg_rom.len() / 0x8000;
                let bank = match self.mapper {
                    7 => self.simple_reg as usize & 0x0f,
                    11 | 96 => self.simple_reg as usize & 0x03,
                    34 | 241 => self.prg_bank,
                    66 | 86 | 140 => (self.simple_reg as usize >> 4) & 0x03,
                    79 | 146 => (self.simple_reg as usize >> 3) & 0x01,
                    113 => (self.simple_reg as usize >> 3) & 0x07,
                    _ => unreachable!(),
                } % banks;
                self.prg_rom[bank * 0x8000 + (address as usize - 0x8000)]
            }
            232 => self.read_camerica_quattro_prg(address),
            _ => {
                let mut index = address as usize - 0x8000;
                if self.prg_rom.len() == 16 * 1024 {
                    index &= 0x3fff;
                }
                self.prg_rom[index]
            }
        }
    }

    fn mmc5_prg_mapping(&self, address: u16) -> (bool, usize) {
        let slot = (address as usize - 0x8000) / 0x2000;
        let (register, bank, forced_rom) = match self.mmc5.prg_mode & 3 {
            0 => {
                let register = self.mmc5.prg[3];
                (register, usize::from(register & 0x7c) + slot, true)
            }
            1 => {
                if slot < 2 {
                    let register = self.mmc5.prg[1];
                    (register, usize::from(register & 0x7e) + slot, false)
                } else {
                    let register = self.mmc5.prg[3];
                    (register, usize::from(register & 0x7e) + slot - 2, true)
                }
            }
            2 => match slot {
                0 | 1 => {
                    let register = self.mmc5.prg[1];
                    (register, usize::from(register & 0x7e) + slot, false)
                }
                2 => {
                    let register = self.mmc5.prg[2];
                    (register, usize::from(register & 0x7f), false)
                }
                _ => {
                    let register = self.mmc5.prg[3];
                    (register, usize::from(register & 0x7f), true)
                }
            },
            _ => {
                let register = self.mmc5.prg[slot];
                (register, usize::from(register & 0x7f), slot == 3)
            }
        };
        (forced_rom || register & 0x80 != 0, bank)
    }

    fn read_mmc5_prg(&self, address: u16) -> u8 {
        let (rom, bank) = self.mmc5_prg_mapping(address);
        let offset = address as usize & 0x1fff;
        if rom {
            let banks = self.prg_rom.len() / 0x2000;
            self.prg_rom[(bank % banks) * 0x2000 + offset]
        } else if self.prg_ram.is_empty() {
            0xff
        } else {
            let banks = (self.prg_ram.len() / 0x2000).max(1);
            self.prg_ram[(bank % banks) * 0x2000 + offset]
        }
    }

    fn write_mmc5_prg(&mut self, address: u16, value: u8) {
        let (rom, bank) = self.mmc5_prg_mapping(address);
        if rom || !self.mmc5.ram_write_enabled() || self.prg_ram.is_empty() {
            return;
        }
        let banks = (self.prg_ram.len() / 0x2000).max(1);
        let index = (bank % banks) * 0x2000 + (address as usize & 0x1fff);
        self.prg_ram[index] = value;
    }

    fn read_vrc_prg(&self, address: u16) -> u8 {
        let banks = self.prg_rom.len() / 0x2000;
        let last = banks - 1;
        let second_last = banks - 2;
        let slot = (address as usize - 0x8000) / 0x2000;
        let bank = if self.is_vrc2() {
            match slot {
                0 => usize::from(self.vrc4.prg[0]) % banks,
                1 => usize::from(self.vrc4.prg[1]) % banks,
                2 => second_last,
                3 => last,
                _ => unreachable!(),
            }
        } else {
            match (self.vrc4.swap_mode, slot) {
                (false, 0) | (true, 2) => usize::from(self.vrc4.prg[0]) % banks,
                (_, 1) => usize::from(self.vrc4.prg[1]) % banks,
                (false, 2) | (true, 0) => second_last,
                (_, 3) => last,
                _ => unreachable!(),
            }
        };
        self.prg_rom[bank * 0x2000 + (address as usize & 0x1fff)]
    }

    fn read_vrc6_prg(&self, address: u16) -> u8 {
        let banks_8k = self.prg_rom.len() / 0x2000;
        let bank = match address {
            0x8000..=0xbfff => {
                let base = (usize::from(self.vrc6.prg16) * 2) % banks_8k;
                base + usize::from(address >= 0xa000)
            }
            0xc000..=0xdfff => usize::from(self.vrc6.prg8) % banks_8k,
            0xe000..=0xffff => banks_8k - 1,
            _ => unreachable!(),
        };
        self.prg_rom[bank * 0x2000 + (address as usize & 0x1fff)]
    }

    fn read_vrc7_prg(&self, address: u16) -> u8 {
        let banks = self.prg_rom.len() / 0x2000;
        let slot = (address as usize - 0x8000) / 0x2000;
        let bank = if slot < 3 {
            usize::from(self.vrc7.prg[slot] & 0x3f) % banks
        } else {
            banks - 1
        };
        self.prg_rom[bank * 0x2000 + (address as usize & 0x1fff)]
    }

    fn read_irem_h3001_prg(&self, address: u16) -> u8 {
        let banks = self.prg_rom.len() / 0x2000;
        let slot = (address as usize - 0x8000) / 0x2000;
        let bank = match (self.irem_h3001.swap_prg, slot) {
            (false, 0) | (true, 2) => usize::from(self.irem_h3001.prg[0]) % banks,
            (_, 1) => usize::from(self.irem_h3001.prg[1]) % banks,
            (false, 2) | (true, 0) => banks - 2,
            (_, 3) => banks - 1,
            _ => unreachable!(),
        };
        self.prg_rom[bank * 0x2000 + (address as usize & 0x1fff)]
    }

    fn read_irem_g101_prg(&self, address: u16) -> u8 {
        let banks = self.prg_rom.len() / 0x2000;
        let slot = (address as usize - 0x8000) / 0x2000;
        let swap = self.submapper == 0 && self.irem_g101.swap_prg;
        let bank = match (swap, slot) {
            (false, 0) | (true, 2) => usize::from(self.irem_g101.prg[0] & 0x1f) % banks,
            (_, 1) => usize::from(self.irem_g101.prg[1] & 0x1f) % banks,
            (false, 2) | (true, 0) => banks - 2,
            (_, 3) => banks - 1,
            _ => unreachable!(),
        };
        self.prg_rom[bank * 0x2000 + (address as usize & 0x1fff)]
    }

    fn read_taito_tc0190_prg(&self, address: u16) -> u8 {
        let banks = self.prg_rom.len() / 0x2000;
        let slot = (address as usize - 0x8000) / 0x2000;
        let bank = match slot {
            0 => usize::from(self.taito_tc0190.prg[0] & 0x3f) % banks,
            1 => usize::from(self.taito_tc0190.prg[1] & 0x3f) % banks,
            2 => banks - 2,
            3 => banks - 1,
            _ => unreachable!(),
        };
        self.prg_rom[bank * 0x2000 + (address as usize & 0x1fff)]
    }

    fn read_sunsoft3_prg(&self, address: u16) -> u8 {
        let banks = self.prg_rom.len() / 0x4000;
        let bank = if address < 0xc000 {
            usize::from(self.sunsoft3.prg & 0x0f) % banks
        } else {
            banks - 1
        };
        self.prg_rom[bank * 0x4000 + (address as usize & 0x3fff)]
    }

    fn read_sunsoft4_prg(&self, address: u16) -> u8 {
        let banks = self.prg_rom.len() / 0x4000;
        let bank = if address < 0xc000 {
            if self.submapper == 1 && self.sunsoft4.prg_control & 0x08 == 0 {
                return 0xff;
            }
            usize::from(self.sunsoft4.prg_control & 0x0f) % banks
        } else {
            banks - 1
        };
        self.prg_rom[bank * 0x4000 + (address as usize & 0x3fff)]
    }

    fn read_taito_x1005_prg(&self, address: u16) -> u8 {
        let banks = self.prg_rom.len() / 0x2000;
        let slot = (address as usize - 0x8000) / 0x2000;
        let bank = if slot < 3 {
            usize::from(self.taito_x1005.prg[slot] & 0x3f) % banks
        } else {
            banks - 1
        };
        self.prg_rom[bank * 0x2000 + (address as usize & 0x1fff)]
    }

    fn taito_x1017_prg_bank(&self, value: u8) -> usize {
        if self.mapper == 82 {
            usize::from((value >> 2) & 0x0f)
        } else {
            usize::from((value & 0x3f).reverse_bits() >> 2)
        }
    }

    fn read_taito_x1017_prg(&self, address: u16) -> u8 {
        let banks = self.prg_rom.len() / 0x2000;
        let slot = (address as usize - 0x8000) / 0x2000;
        let bank = if slot < 3 {
            self.taito_x1017_prg_bank(self.taito_x1017.prg[slot]) % banks
        } else {
            banks - 1
        };
        self.prg_rom[bank * 0x2000 + (address as usize & 0x1fff)]
    }

    fn read_jaleco_ss88006_prg(&self, address: u16) -> u8 {
        let banks = self.prg_rom.len() / 0x2000;
        let slot = (address as usize - 0x8000) / 0x2000;
        let bank = if slot < 3 {
            usize::from(self.jaleco_ss88006.prg[slot] & 0x3f) % banks
        } else {
            banks - 1
        };
        self.prg_rom[bank * 0x2000 + (address as usize & 0x1fff)]
    }

    fn read_fme7_prg(&self, address: u16) -> u8 {
        let banks = self.prg_rom.len() / 0x2000;
        let slot = (address as usize - 0x8000) / 0x2000;
        let bank = if slot < 3 {
            usize::from(self.fme7.prg[slot] & 0x3f) % banks
        } else {
            banks - 1
        };
        self.prg_rom[bank * 0x2000 + (address as usize & 0x1fff)]
    }

    fn read_namco108_prg(&self, address: u16) -> u8 {
        let banks = self.prg_rom.len() / 0x2000;
        let slot = (address as usize - 0x8000) / 0x2000;
        let bank = if self.mapper == 206 && self.submapper == 1 && self.prg_rom.len() == 0x8000 {
            slot
        } else {
            match slot {
                0 => usize::from(self.mmc3.regs[6] & 0x0f) % banks,
                1 => usize::from(self.mmc3.regs[7] & 0x0f) % banks,
                2 => banks - 2,
                3 => banks - 1,
                _ => unreachable!(),
            }
        };
        self.prg_rom[bank * 0x2000 + (address as usize & 0x1fff)]
    }

    fn read_vrc3_prg(&self, address: u16) -> u8 {
        let banks = self.prg_rom.len() / 0x4000;
        let bank = if address < 0xc000 {
            self.prg_bank % banks
        } else {
            banks - 1
        };
        self.prg_rom[bank * 0x4000 + (address as usize & 0x3fff)]
    }

    fn read_vrc1_prg(&self, address: u16) -> u8 {
        let banks = self.prg_rom.len() / 0x2000;
        let slot = (address as usize - 0x8000) / 0x2000;
        let bank = if slot < 3 {
            usize::from(self.vrc1.prg[slot] & 0x0f) % banks
        } else {
            banks - 1
        };
        self.prg_rom[bank * 0x2000 + (address as usize & 0x1fff)]
    }

    fn read_camerica_quattro_prg(&self, address: u16) -> u8 {
        let banks = self.prg_rom.len() / 0x4000;
        let mut outer = (self.simple_reg >> 3) & 3;
        if self.submapper == 1 {
            outer = ((outer & 1) << 1) | ((outer >> 1) & 1);
        }
        let outer_base = usize::from(outer) * 4;
        let inner = self.prg_bank & 3;
        let bank = if address < 0xc000 {
            outer_base + inner
        } else {
            outer_base + 3
        } % banks;
        self.prg_rom[bank * 0x4000 + (address as usize & 0x3fff)]
    }

    fn mmc1_outer_prg_base(&self) -> usize {
        if self.prg_rom.len() > 256 * 1024 && self.chr_banks_1k <= 8 {
            usize::from((self.mmc1.chr0 >> 4) & 1) * 16
        } else if self.mapper == 155 && self.mmc1.prg & 0x10 != 0 {
            usize::from((self.mmc1.prg >> 3) & 1) * 8
        } else {
            0
        }
    }

    fn mmc1_prg_selected_bank(&self) -> usize {
        let low_mask = if self.mapper == 155 && self.mmc1.prg & 0x10 != 0 {
            0x07
        } else {
            0x0f
        };
        self.mmc1_outer_prg_base() + usize::from(self.mmc1.prg & low_mask)
    }

    fn read_mmc1_prg(&self, address: u16) -> u8 {
        let banks = self.prg_rom.len() / 0x4000;
        if self.submapper == 5 && banks == 2 {
            return self.prg_rom[address as usize - 0x8000];
        }

        let selected = self.mmc1_prg_selected_bank() % banks;
        let outer_base = self.mmc1_outer_prg_base() % banks;
        let outer_span = if self.prg_rom.len() > 256 * 1024 && self.chr_banks_1k <= 8 {
            16
        } else if self.mapper == 155 && self.mmc1.prg & 0x10 != 0 {
            8
        } else {
            banks
        };
        let outer_last = (outer_base + outer_span.saturating_sub(1)).min(banks - 1);
        let mode = (self.mmc1.control >> 2) & 3;
        let bank = match mode {
            0 | 1 => {
                let pair = selected & !1;
                (pair + usize::from(address >= 0xc000)) % banks
            }
            2 => {
                if address < 0xc000 {
                    outer_base
                } else {
                    selected
                }
            }
            _ => {
                if address < 0xc000 {
                    selected
                } else {
                    outer_last
                }
            }
        };
        self.prg_rom[bank * 0x4000 + (address as usize & 0x3fff)]
    }
    fn read_rambo1_prg(&self, address: u16) -> u8 {
        let banks = self.prg_rom.len() / 0x2000;
        let last = banks - 1;
        let r6 = usize::from(self.rambo1.regs[6]) % banks;
        let r7 = usize::from(self.rambo1.regs[7]) % banks;
        let rf = usize::from(self.rambo1.regs[15]) % banks;
        let slot = (address as usize - 0x8000) / 0x2000;
        let bank = match (self.rambo1.bank_select & 0x40 != 0, slot) {
            (false, 0) => r6,
            (false, 1) => r7,
            (false, 2) => rf,
            (false, 3) => last,
            (true, 0) => rf,
            (true, 1) => r7,
            (true, 2) => r6,
            (true, 3) => last,
            _ => unreachable!(),
        };
        self.prg_rom[bank * 0x2000 + (address as usize & 0x1fff)]
    }

    fn read_bandai_prg(&self, address: u16) -> u8 {
        let banks = self.prg_rom.len() / 0x4000;
        let bank = if self.mapper == 153 {
            let outer = usize::from(self.simple_reg & 1) * 16;
            let inner = if address < 0xc000 {
                usize::from(self.bandai.prg & 0x0f)
            } else {
                0x0f
            };
            (outer | inner) % banks
        } else if address < 0xc000 {
            usize::from(self.bandai.prg & 0x0f) % banks
        } else {
            banks - 1
        };
        self.prg_rom[bank * 0x4000 + (address as usize & 0x3fff)]
    }

    fn write_bandai153(&mut self, address: u16, value: u8) {
        match address & 0x000f {
            0x0..=0x3 => self.simple_reg = (self.simple_reg & !1) | (value & 1),
            0x8..=0xc => self.write_bandai(address, value),
            0xd => self.simple_reg = (self.simple_reg & !0x20) | (value & 0x20),
            _ => {}
        }
    }

    fn write_bandai(&mut self, address: u16, value: u8) {
        let direct_counter = address < 0x8000;
        match address & 0x000f {
            0x0..=0x7 => self.bandai.chr[(address & 7) as usize] = value,
            0x8 => self.bandai.prg = value & 0x0f,
            0x9 => self.bandai.mirroring = value & 3,
            0xa => {
                self.bandai.irq_pending = false;
                self.bandai.irq_enabled = value & 1 != 0;
                if !direct_counter {
                    self.bandai.irq_counter = self.bandai.irq_latch;
                }
                if self.bandai.irq_enabled && self.bandai.irq_counter == 0 {
                    self.bandai.irq_pending = true;
                }
            }
            0xb => {
                if direct_counter {
                    self.bandai.irq_counter = (self.bandai.irq_counter & 0xff00) | u16::from(value);
                } else {
                    self.bandai.irq_latch = (self.bandai.irq_latch & 0xff00) | u16::from(value);
                }
            }
            0xc => {
                if direct_counter {
                    self.bandai.irq_counter =
                        (self.bandai.irq_counter & 0x00ff) | (u16::from(value) << 8);
                } else {
                    self.bandai.irq_latch =
                        (self.bandai.irq_latch & 0x00ff) | (u16::from(value) << 8);
                }
            }
            0xd if !direct_counter && self.bandai.eeprom_present => {
                let scl = value & 0x20 != 0;
                let sda = if value & 0x80 != 0 {
                    true
                } else {
                    value & 0x40 != 0
                };
                if self.mapper == 159 {
                    self.bandai.eeprom.write_x24c01_lines(scl, sda);
                } else {
                    self.bandai.eeprom.write_lines(scl, sda);
                }
            }
            _ => {}
        }
    }

    fn write_datach(&mut self, address: u16, value: u8) {
        match address & 0x000f {
            0x0..=0x7 => {
                self.bandai.external_scl = value & 0x08 != 0;
                if self.bandai.external_eeprom_present {
                    let sda = self.bandai.external_eeprom.prev_sda;
                    self.bandai
                        .external_eeprom
                        .write_x24c01_lines(self.bandai.external_scl, sda);
                }
            }
            0x8..=0xc => self.write_bandai(address, value),
            0xd => {
                let sda = value & 0x80 != 0 || value & 0x40 != 0;
                self.bandai.eeprom.write_lines(value & 0x20 != 0, sda);
                if self.bandai.external_eeprom_present {
                    self.bandai
                        .external_eeprom
                        .write_x24c01_lines(self.bandai.external_scl, sda);
                }
            }
            _ => {}
        }
    }

    fn read_namco163_prg(&self, address: u16) -> u8 {
        let banks = self.prg_rom.len() / 0x2000;
        let slot = (address as usize - 0x8000) / 0x2000;
        let bank = if slot < 3 {
            usize::from(self.namco163.prg[slot] & 0x3f) % banks
        } else {
            banks - 1
        };
        self.prg_rom[bank * 0x2000 + (address as usize & 0x1fff)]
    }

    fn write_namco163(&mut self, address: u16, value: u8) {
        match address {
            0x8000..=0xbfff => {
                let slot = (address as usize - 0x8000) / 0x0800;
                self.namco163.chr[slot] = value;
            }
            0xc000..=0xdfff => {
                let slot = (address as usize - 0xc000) / 0x0800;
                self.namco163.nametable[slot] = value;
            }
            0xe000..=0xe7ff => {
                self.namco163.prg[0] = value & 0x3f;
                self.namco163.sound_disabled = value & 0x40 != 0;
                if self.namco163.sound_disabled {
                    self.namco163.audio_output = 0.0;
                }
            }
            0xe800..=0xefff => {
                self.namco163.prg[1] = value & 0x3f;
                self.namco163.chr_disable_low = value & 0x40 != 0;
                self.namco163.chr_disable_high = value & 0x80 != 0;
            }
            0xf000..=0xf7ff => self.namco163.prg[2] = value & 0x3f,
            0xf800..=0xffff => {
                self.namco163.wram_protect = value;
                self.namco163.ram_address = value & 0x7f;
                self.namco163.ram_autoincrement = value & 0x80 != 0;
            }
            _ => {}
        }
    }

    fn read_namco210_prg(&self, address: u16) -> u8 {
        let banks = self.prg_rom.len() / 0x2000;
        let slot = (address as usize - 0x8000) / 0x2000;
        let bank = if slot < 3 {
            usize::from(self.namco210.prg[slot] & 0x3f) % banks
        } else {
            banks - 1
        };
        self.prg_rom[bank * 0x2000 + (address as usize & 0x1fff)]
    }

    fn write_namco210(&mut self, address: u16, value: u8) {
        match address {
            0x8000..=0xbfff => {
                let slot = (address as usize - 0x8000) / 0x0800;
                self.namco210.chr[slot] = value;
            }
            0xc000..=0xc7ff if self.namco210.is_175 => {
                self.namco210.ram_enabled = value & 1 != 0;
            }
            0xe000..=0xe7ff => {
                self.namco210.prg[0] = value & 0x3f;
                if !self.namco210.is_175 {
                    self.namco210.mirroring = value >> 6;
                }
            }
            0xe800..=0xefff => self.namco210.prg[1] = value & 0x3f,
            0xf000..=0xf7ff => self.namco210.prg[2] = value & 0x3f,
            _ => {}
        }
    }

    fn mmc3_prg_bank_index(&self, inner: usize) -> usize {
        match self.mapper {
            37 => {
                let outer = self.simple_reg & 7;
                let a16 = (outer & 3 == 3) || (outer & 4 != 0 && inner & 8 != 0);
                (usize::from(outer & 4) << 2) | (usize::from(a16) << 3) | (inner & 7)
            }
            47 => (usize::from(self.simple_reg & 1) << 4) | (inner & 0x0f),
            _ => inner,
        }
    }

    fn read_mmc3_prg(&self, address: u16) -> u8 {
        let banks = self.prg_rom.len() / 0x2000;
        let inner_banks = if matches!(self.mapper, 37 | 47) {
            16
        } else {
            banks
        };
        let last = inner_banks - 1;
        let second_last = inner_banks - 2;
        let r6 = self.mmc3.regs[6] as usize % inner_banks;
        let r7 = self.mmc3.regs[7] as usize % inner_banks;
        let slot = (address as usize - 0x8000) / 0x2000;
        let prg_mode = self.mmc3.bank_select & 0x40 != 0;
        let inner = match (prg_mode, slot) {
            (false, 0) => r6,
            (false, 1) => r7,
            (false, 2) => second_last,
            (false, 3) => last,
            (true, 0) => second_last,
            (true, 1) => r7,
            (true, 2) => r6,
            (true, 3) => last,
            _ => unreachable!(),
        };
        let bank = self.mmc3_prg_bank_index(inner) % banks;
        self.prg_rom[bank * 0x2000 + (address as usize & 0x1fff)]
    }

    fn is_nina_001(&self) -> bool {
        self.mapper == 34 && (self.submapper == 1 || (self.submapper == 0 && self.chr_banks_1k > 8))
    }

    fn bandai_74161_one_screen(&self) -> bool {
        self.mapper == 152 || (self.mapper == 70 && self.four_screen)
    }

    fn mapper78_holy_diver_wiring(&self) -> bool {
        self.mapper == 78 && (self.submapper == 3 || (self.submapper == 0 && self.four_screen))
    }

    fn bus_conflicted_value(&self, address: u16, value: u8) -> u8 {
        if self.mapper == 185
            || (self.mapper == 3 && self.submapper == 2)
            || matches!(self.mapper, 70 | 72 | 78 | 89 | 92 | 93 | 96 | 152)
        {
            value & self.read_prg(address)
        } else {
            value
        }
    }

    fn cnrom185_chr_enabled(&self) -> bool {
        if self.mapper != 185 || self.submapper == 0 {
            return true;
        }
        (self.prg_bank & 3) == usize::from(self.submapper - 4)
    }

    fn filter_cnrom185_ppudata_read(&mut self, value: u8) -> u8 {
        if self.mapper == 185 && self.submapper == 0 && self.cnrom185_reads_remaining != 0 {
            self.cnrom185_reads_remaining -= 1;
            0xff
        } else {
            value
        }
    }

    fn write_mapper(&mut self, address: u16, value: u8) {
        match self.mapper {
            1 | 155 => self.write_mmc1(address, value),
            2 => self.prg_bank = value as usize,
            18 => self.write_jaleco_ss88006(address, value),
            3 | 185 => self.prg_bank = usize::from(self.bus_conflicted_value(address, value)),
            70 | 78 | 89 | 93 | 96 | 152 => {
                self.simple_reg = self.bus_conflicted_value(address, value)
            }
            72 | 92 => self.write_jaleco_jf17(address, value),
            4 | 37 | 47 | 118 | 119 => self.write_mmc3(address, value),
            5 => self.write_mmc5_prg(address, value),
            9 | 10 => self.write_mmc2(address, value),
            21 | 22 | 23 | 25 => self.write_vrc(address, value),
            24 | 26 => self.write_vrc6(address, value),
            32 => self.write_irem_g101(address, value),
            65 => self.write_irem_h3001(address, value),
            85 => self.write_vrc7(address, value),
            86 => self.write_jaleco_jf13(address, value),
            33 | 48 => self.write_taito_tc0190(address, value),
            64 | 158 => self.write_rambo1(address, value),
            67 => self.write_sunsoft3(address, value),
            68 => self.write_sunsoft4(address, value),
            69 => self.write_fme7(address, value),
            73 => self.write_vrc3(address, value),
            75 => self.write_vrc1(address, value),
            76 | 88 | 95 | 154 | 206 => self.write_namco108(address, value),
            210 => self.write_namco210(address, value),
            7 | 11 | 66 => self.simple_reg = value,
            13 => self.simple_reg = value & 3,
            16 if matches!(self.submapper, 0 | 5) => self.write_bandai(address, value),
            153 => self.write_bandai153(address, value),
            157 => self.write_datach(address, value),
            159 => self.write_bandai(address, value),
            19 => self.write_namco163(address, value),
            34 if !self.is_nina_001() => self.prg_bank = value as usize,
            71 => match address {
                0x8000..=0x9fff
                    if self.submapper == 1 || (self.submapper == 0 && address >= 0x9000) =>
                {
                    self.simple_reg = 0x80 | (value & 0x10)
                }
                0xc000..=0xffff => self.prg_bank = value as usize,
                _ => {}
            },
            94 => self.prg_bank = usize::from((value >> 2) & 0x0f),
            97 if address < 0xc000 => self.simple_reg = value,
            180 => self.prg_bank = value as usize,
            232 => match address {
                0x8000..=0xbfff => self.simple_reg = value,
                0xc000..=0xffff => self.prg_bank = usize::from(value & 3),
                _ => {}
            },
            241 => self.prg_bank = value as usize,
            _ => {}
        }
    }

    fn write_mmc5_register(&mut self, address: u16, value: u8) -> bool {
        match address {
            0x5000 => self.mmc5.pulses[0].write_control(value),
            0x5002 => self.mmc5.pulses[0].write_period_low(value),
            0x5003 => self.mmc5.pulses[0].write_period_high(value, self.mmc5.pulse_enable & 1 != 0),
            0x5004 => self.mmc5.pulses[1].write_control(value),
            0x5006 => self.mmc5.pulses[1].write_period_low(value),
            0x5007 => self.mmc5.pulses[1].write_period_high(value, self.mmc5.pulse_enable & 2 != 0),
            0x5010 => {
                self.mmc5.pcm_mode = value & 1 != 0;
                self.mmc5.pcm_irq_enabled = value & 0x80 != 0;
            }
            0x5011 if !self.mmc5.pcm_mode => {
                if value == 0 {
                    self.mmc5.pcm_irq_pending = true;
                } else {
                    self.mmc5.pcm = value;
                    self.mmc5.pcm_irq_pending = false;
                }
            }
            0x5011 => {}
            0x5015 => {
                self.mmc5.pulse_enable = value & 3;
                for channel in 0..2 {
                    if self.mmc5.pulse_enable & (1 << channel) == 0 {
                        self.mmc5.pulses[channel].length = 0;
                    }
                }
            }
            0x5100 => self.mmc5.prg_mode = value & 3,
            0x5101 => self.mmc5.chr_mode = value & 3,
            0x5102 => self.mmc5.prg_protect[0] = value & 3,
            0x5103 => self.mmc5.prg_protect[1] = value & 3,
            0x5104 => self.mmc5.exram_mode = value & 3,
            0x5105 => self.mmc5.nametable_map = value,
            0x5106 => self.mmc5.fill_tile = value,
            0x5107 => self.mmc5.fill_color = value & 3,
            0x5113 => self.mmc5.prg_ram_bank = value & 0x0f,
            0x5114..=0x5117 => self.mmc5.prg[usize::from(address - 0x5114)] = value,
            0x5120..=0x5127 => {
                let index = usize::from(address - 0x5120);
                self.mmc5.chr_sprite[index] = self.mmc5.latch_chr_bank(value);
                self.mmc5.chr_io_bg = false;
            }
            0x5128..=0x512b => {
                let index = usize::from(address - 0x5128);
                self.mmc5.chr_bg[index] = self.mmc5.latch_chr_bank(value);
                self.mmc5.chr_io_bg = true;
            }
            0x5130 => self.mmc5.chr_upper = value & 3,
            0x5200 => self.mmc5.split_control = value & 0xdf,
            0x5201 => self.mmc5.split_scroll = value,
            0x5202 => self.mmc5.split_bank = value,
            0x5203 => self.mmc5.irq_compare = value,
            0x5204 => self.mmc5.irq_enabled = value & 0x80 != 0,
            0x5205 => self.mmc5.multiplicand = value,
            0x5206 => self.mmc5.multiplier = value,
            _ => return false,
        }
        true
    }

    fn read_expansion_mapper(&mut self, address: u16) -> Option<u8> {
        match self.mapper {
            5 => match address {
                0x5010 => {
                    let value =
                        u8::from(self.mmc5.pcm_irq_enabled && self.mmc5.pcm_irq_pending) << 7;
                    self.mmc5.pcm_irq_pending = false;
                    Some(value)
                }
                0x5015 => Some(
                    u8::from(self.mmc5.pulses[0].length != 0)
                        | (u8::from(self.mmc5.pulses[1].length != 0) << 1),
                ),
                0x5204 => {
                    let value = (u8::from(self.mmc5.irq_pending) << 7)
                        | (u8::from(self.mmc5.in_frame) << 6);
                    self.mmc5.irq_pending = false;
                    Some(value)
                }
                0x5205 => Some(
                    (u16::from(self.mmc5.multiplicand) * u16::from(self.mmc5.multiplier)) as u8,
                ),
                0x5206 => Some(
                    ((u16::from(self.mmc5.multiplicand) * u16::from(self.mmc5.multiplier)) >> 8)
                        as u8,
                ),
                _ => None,
            },
            19 => match address {
                0x4800..=0x4fff => Some(self.namco163.read_data()),
                0x5000..=0x57ff => Some(self.namco163.irq_counter as u8),
                0x5800..=0x5fff => Some(
                    ((self.namco163.irq_counter >> 8) as u8 & 0x7f)
                        | (u8::from(self.namco163.irq_enabled) << 7),
                ),
                _ => None,
            },
            _ => None,
        }
    }

    fn write_expansion_mapper(&mut self, address: u16, value: u8) -> bool {
        match self.mapper {
            5 => self.write_mmc5_register(address, value),
            19 => match address {
                0x4800..=0x4fff => {
                    self.namco163.write_data(value);
                    true
                }
                0x5000..=0x57ff => {
                    self.namco163.irq_counter =
                        (self.namco163.irq_counter & 0x7f00) | u16::from(value);
                    self.namco163.irq_pending = false;
                    true
                }
                0x5800..=0x5fff => {
                    self.namco163.irq_counter =
                        (self.namco163.irq_counter & 0x00ff) | (u16::from(value & 0x7f) << 8);
                    self.namco163.irq_enabled = value & 0x80 != 0;
                    self.namco163.irq_pending = false;
                    true
                }
                _ => false,
            },
            79 | 113 | 146 if address & 0x0100 != 0 => {
                self.simple_reg = value;
                true
            }
            _ => false,
        }
    }

    fn write_low_mapper(&mut self, address: u16, value: u8) -> bool {
        match self.mapper {
            16 if matches!(self.submapper, 0 | 4) => {
                self.write_bandai(address, value);
                true
            }
            34 if self.is_nina_001() && matches!(address, 0x7ffd..=0x7fff) => {
                self.write_ram(address, value);
                match address {
                    0x7ffd => self.prg_bank = value as usize,
                    0x7ffe => self.nina_chr[0] = value,
                    0x7fff => self.nina_chr[1] = value,
                    _ => unreachable!(),
                }
                true
            }
            80 | 207 if (0x7e00..=0x7eff).contains(&address) => {
                self.write_taito_x1005(address, value)
            }
            82 | 552 if (0x7ef0..=0x7eff).contains(&address) => {
                self.write_taito_x1017(address, value)
            }
            87 | 101 | 140 | 184 if (0x6000..=0x7fff).contains(&address) => {
                self.simple_reg = value;
                true
            }
            _ if self.vrc2_uses_latch() && (0x6000..=0x6fff).contains(&address) => {
                self.vrc4.latch_6000 = value & 1;
                true
            }
            _ => false,
        }
    }
    fn write_mmc1(&mut self, address: u16, value: u8) {
        if value & 0x80 != 0 {
            self.mmc1.shift = 0;
            self.mmc1.bits = 0;
            self.mmc1.control |= 0x0c;
            return;
        }
        self.mmc1.shift |= (value & 1) << self.mmc1.bits;
        self.mmc1.bits += 1;
        if self.mmc1.bits < 5 {
            return;
        }
        let data = self.mmc1.shift & 0x1f;
        match address {
            0x8000..=0x9fff => self.mmc1.control = data,
            0xa000..=0xbfff => self.mmc1.chr0 = data,
            0xc000..=0xdfff => self.mmc1.chr1 = data,
            0xe000..=0xffff => self.mmc1.prg = data,
            _ => {}
        }
        self.mmc1.shift = 0;
        self.mmc1.bits = 0;
    }

    fn write_mmc2(&mut self, address: u16, value: u8) {
        match address & 0xf000 {
            0xa000 => self.prg_bank = usize::from(value & 0x0f),
            0xb000 => self.mmc2.chr_fd[0] = value & 0x1f,
            0xc000 => self.mmc2.chr_fe[0] = value & 0x1f,
            0xd000 => self.mmc2.chr_fd[1] = value & 0x1f,
            0xe000 => self.mmc2.chr_fe[1] = value & 0x1f,
            0xf000 => self.mmc2.mirror_horizontal = value & 1 != 0,
            _ => {}
        }
    }

    fn write_irem_h3001(&mut self, address: u16, value: u8) {
        match address & 0xf007 {
            0x8000..=0x8007 => self.irem_h3001.prg[0] = value,
            0x9000 => self.irem_h3001.swap_prg = value & 0x80 != 0,
            0x9001 => self.irem_h3001.mirroring = (value >> 6) & 3,
            0x9003 => {
                self.irem_h3001.irq_enabled = value & 0x80 != 0;
                self.irem_h3001.irq_pending = false;
            }
            0x9004 => {
                self.irem_h3001.irq_counter = self.irem_h3001.irq_reload;
                self.irem_h3001.irq_pending = false;
            }
            0x9005 => {
                self.irem_h3001.irq_reload =
                    (self.irem_h3001.irq_reload & 0x00ff) | (u16::from(value) << 8);
            }
            0x9006 => {
                self.irem_h3001.irq_reload =
                    (self.irem_h3001.irq_reload & 0xff00) | u16::from(value);
            }
            0xa000..=0xa007 => self.irem_h3001.prg[1] = value,
            0xb000..=0xb007 => self.irem_h3001.chr[usize::from(address & 7)] = value,
            _ => {}
        }
    }

    fn write_irem_g101(&mut self, address: u16, value: u8) {
        match address & 0xf000 {
            0x8000 => self.irem_g101.prg[0] = value & 0x1f,
            0x9000 if self.submapper == 0 => {
                self.irem_g101.mirror_horizontal = value & 1 != 0;
                self.irem_g101.swap_prg = value & 2 != 0;
            }
            0xa000 => self.irem_g101.prg[1] = value & 0x1f,
            0xb000 => self.irem_g101.chr[usize::from(address & 7)] = value,
            _ => {}
        }
    }

    fn write_sunsoft3(&mut self, address: u16, value: u8) {
        match address & 0xf800 {
            0x8800 => self.sunsoft3.chr_2k[0] = value & 0x3f,
            0x9800 => self.sunsoft3.chr_2k[1] = value & 0x3f,
            0xa800 => self.sunsoft3.chr_2k[2] = value & 0x3f,
            0xb800 => self.sunsoft3.chr_2k[3] = value & 0x3f,
            0xc800 => {
                if self.sunsoft3.irq_low_next {
                    self.sunsoft3.irq_counter =
                        (self.sunsoft3.irq_counter & 0xff00) | u16::from(value);
                } else {
                    self.sunsoft3.irq_counter =
                        (self.sunsoft3.irq_counter & 0x00ff) | (u16::from(value) << 8);
                }
                self.sunsoft3.irq_low_next = !self.sunsoft3.irq_low_next;
            }
            0xd800 => {
                self.sunsoft3.irq_enabled = value & 0x10 != 0;
                self.sunsoft3.irq_low_next = false;
            }
            0xe800 => self.sunsoft3.mirroring = value & 3,
            0xf800 => self.sunsoft3.prg = value & 0x0f,
            _ if address & 0x8800 == 0x8000 => self.sunsoft3.irq_pending = false,
            _ => {}
        }
    }

    fn write_sunsoft4(&mut self, address: u16, value: u8) {
        match address >> 12 {
            0x8..=0xb => self.sunsoft4.chr_2k[usize::from((address >> 12) - 8)] = value,
            0xc => self.sunsoft4.nametable_chr[0] = value & 0x7f,
            0xd => self.sunsoft4.nametable_chr[1] = value & 0x7f,
            0xe => self.sunsoft4.nametable_control = value & 0x13,
            0xf => self.sunsoft4.prg_control = value & 0x1f,
            _ => {}
        }
    }

    fn write_taito_tc0190(&mut self, address: u16, value: u8) {
        if self.mapper == 48 {
            match address & 0xe003 {
                0x8000 => self.taito_tc0190.prg[0] = value & 0x3f,
                0x8001 => self.taito_tc0190.prg[1] = value & 0x3f,
                0x8002 => self.taito_tc0190.chr_2k[0] = value,
                0x8003 => self.taito_tc0190.chr_2k[1] = value,
                0xa000..=0xa003 => {
                    self.taito_tc0190.chr_1k[usize::from(address & 3)] = value;
                }
                0xc000 => self.taito_tc0190.irq_latch = value ^ 0xff,
                0xc001 => self.taito_tc0190.irq_reload = true,
                0xc002 => self.taito_tc0190.irq_enabled = true,
                0xc003 => {
                    self.taito_tc0190.irq_enabled = false;
                    self.taito_tc0190.irq_pending = false;
                    self.taito_tc0190.irq_delay = 0;
                }
                0xe000 => self.taito_tc0190.mirror_horizontal = value & 0x40 != 0,
                _ => {}
            }
            return;
        }
        match address & 0xa003 {
            0x8000 => {
                self.taito_tc0190.prg[0] = value & 0x3f;
                self.taito_tc0190.mirror_horizontal = value & 0x40 != 0;
            }
            0x8001 => self.taito_tc0190.prg[1] = value & 0x3f,
            0x8002 => self.taito_tc0190.chr_2k[0] = value,
            0x8003 => self.taito_tc0190.chr_2k[1] = value,
            0xa000..=0xa003 => {
                self.taito_tc0190.chr_1k[usize::from(address & 3)] = value;
            }
            _ => {}
        }
    }

    fn write_taito_x1005(&mut self, address: u16, value: u8) -> bool {
        let register = address | 0x0080;
        match register {
            0x7ef0..=0x7ef5 => {
                self.taito_x1005.chr[usize::from(register - 0x7ef0)] = value;
            }
            0x7ef6 if self.mapper == 80 => {
                self.taito_x1005.mirror_horizontal = value & 1 != 0;
            }
            0x7ef7 => return false,
            0x7ef8 | 0x7ef9 => self.taito_x1005.ram_enabled = value == 0xa3,
            0x7efa | 0x7efb => self.taito_x1005.prg[0] = value & 0x3f,
            0x7efc | 0x7efd => self.taito_x1005.prg[1] = value & 0x3f,
            0x7efe | 0x7eff => self.taito_x1005.prg[2] = value & 0x3f,
            _ => return false,
        }
        true
    }

    fn taito_x1017_control_reload(&self) -> u16 {
        if self.taito_x1017.irq_latch == 0 {
            17
        } else {
            (u16::from(self.taito_x1017.irq_latch) + 2) * 16
        }
    }

    fn taito_x1017_ack_reload(&self) -> u16 {
        if self.taito_x1017.irq_latch == 0 {
            1
        } else {
            (u16::from(self.taito_x1017.irq_latch) + 1) * 16
        }
    }

    fn write_taito_x1017(&mut self, address: u16, value: u8) -> bool {
        match address {
            0x7ef0..=0x7ef5 => {
                self.taito_x1017.chr[usize::from(address - 0x7ef0)] = value;
            }
            0x7ef6 => {
                self.taito_x1017.mirror_vertical = value & 1 != 0;
                self.taito_x1017.chr_inverted = value & 2 != 0;
            }
            0x7ef7 => self.taito_x1017.ram_enabled[0] = value == 0xca,
            0x7ef8 => self.taito_x1017.ram_enabled[1] = value == 0x69,
            0x7ef9 => self.taito_x1017.ram_enabled[2] = value == 0x84,
            0x7efa..=0x7efc => {
                self.taito_x1017.prg[usize::from(address - 0x7efa)] = value;
            }
            0x7efd => self.taito_x1017.irq_latch = value,
            0x7efe => {
                self.taito_x1017.irq_control = value & 7;
                if value & 1 == 0 {
                    self.taito_x1017.irq_counter = self.taito_x1017_control_reload();
                }
            }
            0x7eff => {
                self.taito_x1017.irq_pending = false;
                self.taito_x1017.irq_counter = self.taito_x1017_ack_reload();
            }
            _ => return false,
        }
        true
    }

    fn write_jaleco_jf17(&mut self, address: u16, value: u8) {
        let value = self.bus_conflicted_value(address, value);
        let rising = value & !self.jaleco_jf17.control;
        let bank = value & 0x0f;
        if rising & 0x80 != 0 {
            self.jaleco_jf17.prg = if self.mapper == 72 { bank & 7 } else { bank };
        }
        if rising & 0x40 != 0 {
            self.jaleco_jf17.chr = bank;
        }
        self.jaleco_jf17.control = value & 0xf0;
    }

    fn write_jaleco_jf13(&mut self, address: u16, value: u8) {
        match address & 0xf000 {
            0x6000 | 0xe000 => self.simple_reg = value,
            0x7000 | 0xf000 => self.jaleco_d7756_control = value & 0x3f,
            _ => {}
        }
    }

    fn write_jaleco_ss88006(&mut self, address: u16, value: u8) {
        let nibble = value & 0x0f;
        match address & 0xf003 {
            0x8000 | 0x8001 => {
                let high = address & 1 != 0;
                let mask = if high { 0x0f } else { 0x30 };
                let data = if high { (nibble & 3) << 4 } else { nibble };
                self.jaleco_ss88006.prg[0] = (self.jaleco_ss88006.prg[0] & mask) | data;
            }
            0x8002 | 0x8003 => {
                let high = address & 1 != 0;
                let mask = if high { 0x0f } else { 0x30 };
                let data = if high { (nibble & 3) << 4 } else { nibble };
                self.jaleco_ss88006.prg[1] = (self.jaleco_ss88006.prg[1] & mask) | data;
            }
            0x9000 | 0x9001 => {
                let high = address & 1 != 0;
                let mask = if high { 0x0f } else { 0x30 };
                let data = if high { (nibble & 3) << 4 } else { nibble };
                self.jaleco_ss88006.prg[2] = (self.jaleco_ss88006.prg[2] & mask) | data;
            }
            0x9002 => self.jaleco_ss88006.ram_control = value & 3,
            0xa000..=0xd003 => {
                let group = usize::from((address >> 12) - 0x0a);
                let pair = usize::from((address >> 1) & 1);
                let bank = group * 2 + pair;
                if bank < 8 {
                    if address & 1 == 0 {
                        self.jaleco_ss88006.chr[bank] =
                            (self.jaleco_ss88006.chr[bank] & 0xf0) | nibble;
                    } else {
                        self.jaleco_ss88006.chr[bank] =
                            (self.jaleco_ss88006.chr[bank] & 0x0f) | (nibble << 4);
                    }
                }
            }
            0xe000..=0xe003 => {
                let shift = ((address & 3) * 4) as u32;
                let mask = !(0x0fu16 << shift);
                self.jaleco_ss88006.irq_reload =
                    (self.jaleco_ss88006.irq_reload & mask) | (u16::from(nibble) << shift);
            }
            0xf000 => {
                self.jaleco_ss88006.irq_counter = self.jaleco_ss88006.irq_reload;
                self.jaleco_ss88006.irq_pending = false;
            }
            0xf001 => {
                self.jaleco_ss88006.irq_control = value & 0x0f;
                self.jaleco_ss88006.irq_pending = false;
            }
            0xf002 => self.jaleco_ss88006.mirroring = value & 3,
            0xf003 => self.jaleco_ss88006.sound_control = value,
            _ => {}
        }
    }

    fn write_namco108(&mut self, address: u16, value: u8) {
        if self.mapper == 154 {
            self.simple_reg = value & 0x40;
        }
        match address & 0xe001 {
            0x8000 => self.mmc3.bank_select = value & 7,
            0x8001 => {
                let register = usize::from(self.mmc3.bank_select & 7);
                if self.mapper == 76 && register < 2 {
                    return;
                }
                self.mmc3.regs[register] = if register < 6 {
                    value & 0x3f
                } else {
                    value & 0x0f
                };
            }
            _ => {}
        }
    }

    fn write_vrc1(&mut self, address: u16, value: u8) {
        match address & 0xf000 {
            0x8000 => self.vrc1.prg[0] = value & 0x0f,
            0x9000 => {
                self.vrc1.mirror_horizontal = value & 0x01 != 0;
                self.vrc1.chr[0] = (self.vrc1.chr[0] & 0x0f) | ((value & 0x02) << 3);
                self.vrc1.chr[1] = (self.vrc1.chr[1] & 0x0f) | ((value & 0x04) << 2);
            }
            0xa000 => self.vrc1.prg[1] = value & 0x0f,
            0xc000 => self.vrc1.prg[2] = value & 0x0f,
            0xe000 => self.vrc1.chr[0] = (self.vrc1.chr[0] & 0x10) | (value & 0x0f),
            0xf000 => self.vrc1.chr[1] = (self.vrc1.chr[1] & 0x10) | (value & 0x0f),
            _ => {}
        }
    }

    fn write_vrc3(&mut self, address: u16, value: u8) {
        match address & 0xf000 {
            0x8000 => {
                self.vrc3.irq_latch = (self.vrc3.irq_latch & 0xfff0) | u16::from(value & 0x0f)
            }
            0x9000 => {
                self.vrc3.irq_latch =
                    (self.vrc3.irq_latch & 0xff0f) | (u16::from(value & 0x0f) << 4)
            }
            0xa000 => {
                self.vrc3.irq_latch =
                    (self.vrc3.irq_latch & 0xf0ff) | (u16::from(value & 0x0f) << 8)
            }
            0xb000 => {
                self.vrc3.irq_latch =
                    (self.vrc3.irq_latch & 0x0fff) | (u16::from(value & 0x0f) << 12)
            }
            0xc000 => {
                self.vrc3.irq_pending = false;
                self.vrc3.irq_enable_after_ack = value & 0x01 != 0;
                self.vrc3.irq_enabled = value & 0x02 != 0;
                self.vrc3.irq_mode_8bit = value & 0x04 != 0;
                if self.vrc3.irq_enabled {
                    self.vrc3.irq_counter = self.vrc3.irq_latch;
                }
            }
            0xd000 => {
                self.vrc3.irq_pending = false;
                self.vrc3.irq_enabled = self.vrc3.irq_enable_after_ack;
            }
            0xf000 => self.prg_bank = usize::from(value & 0x07),
            _ => {}
        }
    }

    fn write_vrc(&mut self, address: u16, value: u8) {
        let sub = self.vrc_subaddress(address);
        match address & 0xf000 {
            0x8000 => self.vrc4.prg[0] = value & 0x1f,
            0x9000 => {
                if self.is_vrc2() {
                    self.vrc4.mirroring = value & 1;
                } else if sub == 0 {
                    self.vrc4.mirroring = value & 3;
                } else if sub == 2 {
                    self.vrc4.wram_enable = value & 1 != 0;
                    self.vrc4.swap_mode = value & 2 != 0;
                }
            }
            0xa000 => self.vrc4.prg[1] = value & 0x1f,
            0xb000..=0xe000 => {
                let group = usize::from(((address >> 12) & 0x0f) as u8 - 0x0b);
                let bank = group * 2 + usize::from(sub >> 1);
                if bank < self.vrc4.chr.len() {
                    if sub & 1 == 0 {
                        self.vrc4.chr[bank] =
                            (self.vrc4.chr[bank] & !0x000f) | u16::from(value & 0x0f);
                    } else {
                        let high_mask = if self.is_vrc2() { 0x0f } else { 0x1f };
                        self.vrc4.chr[bank] =
                            (self.vrc4.chr[bank] & 0x000f) | (u16::from(value & high_mask) << 4);
                    }
                }
            }
            0xf000 if self.is_vrc4() => match sub {
                0 => self.vrc4.irq_latch = (self.vrc4.irq_latch & 0xf0) | (value & 0x0f),
                1 => self.vrc4.irq_latch = (self.vrc4.irq_latch & 0x0f) | ((value & 0x0f) << 4),
                2 => {
                    self.vrc4.irq_pending = false;
                    self.vrc4.irq_cycle_mode = value & 0x04 != 0;
                    self.vrc4.irq_enabled = value & 0x02 != 0;
                    self.vrc4.irq_enable_after_ack = value & 0x01 != 0;
                    self.vrc4.irq_prescaler = 341;
                    if self.vrc4.irq_enabled {
                        self.vrc4.irq_counter = self.vrc4.irq_latch;
                    }
                }
                3 => {
                    self.vrc4.irq_pending = false;
                    self.vrc4.irq_enabled = self.vrc4.irq_enable_after_ack;
                }
                _ => unreachable!(),
            },
            _ => {}
        }
    }

    fn write_vrc6(&mut self, address: u16, value: u8) {
        let sub = self.vrc6_subaddress(address);
        match address & 0xf000 {
            0x8000 => self.vrc6.prg16 = value & 0x0f,
            0x9000 => match sub {
                0 => self.vrc6.pulses[0].write_control(value),
                1 => self.vrc6.pulses[0].write_period_low(value),
                2 => self.vrc6.pulses[0].write_period_high(value),
                3 => self.vrc6.frequency_control = value & 7,
                _ => unreachable!(),
            },
            0xa000 => match sub {
                0 => self.vrc6.pulses[1].write_control(value),
                1 => self.vrc6.pulses[1].write_period_low(value),
                2 => self.vrc6.pulses[1].write_period_high(value),
                _ => {}
            },
            0xb000 => match sub {
                0 => self.vrc6.saw.write_rate(value),
                1 => self.vrc6.saw.write_period_low(value),
                2 => self.vrc6.saw.write_period_high(value),
                3 => self.vrc6.ppu_control = value,
                _ => unreachable!(),
            },
            0xc000 => self.vrc6.prg8 = value & 0x1f,
            0xd000 => self.vrc6.chr[usize::from(sub)] = value,
            0xe000 => self.vrc6.chr[4 + usize::from(sub)] = value,
            0xf000 => match sub {
                0 => self.vrc6.irq_latch = value,
                1 => {
                    self.vrc6.irq_pending = false;
                    self.vrc6.irq_cycle_mode = value & 0x04 != 0;
                    self.vrc6.irq_enabled = value & 0x02 != 0;
                    self.vrc6.irq_enable_after_ack = value & 0x01 != 0;
                    self.vrc6.irq_prescaler = 341;
                    if self.vrc6.irq_enabled {
                        self.vrc6.irq_counter = self.vrc6.irq_latch;
                    }
                }
                2 => {
                    self.vrc6.irq_pending = false;
                    self.vrc6.irq_enabled = self.vrc6.irq_enable_after_ack;
                }
                _ => {}
            },
            _ => {}
        }
    }

    fn vrc7_secondary_register(&self, address: u16) -> bool {
        match self.submapper {
            1 => address & 0x0018 == 0x0008,
            2 => address & 0x0018 == 0x0010,
            _ => matches!(address & 0x0018, 0x0008 | 0x0010),
        }
    }

    fn write_vrc7(&mut self, address: u16, value: u8) {
        if address == 0x9010 {
            self.vrc7.select_audio_register(value);
            return;
        }
        if address == 0x9030 {
            self.vrc7.write_audio_register(value);
            return;
        }

        let primary = address & 0x0018 == 0;
        let secondary = self.vrc7_secondary_register(address);
        match address & 0xf000 {
            0x8000 if primary => self.vrc7.prg[0] = value & 0x3f,
            0x8000 if secondary => self.vrc7.prg[1] = value & 0x3f,
            0x9000 if primary => self.vrc7.prg[2] = value & 0x3f,
            0xa000 | 0xb000 | 0xc000 | 0xd000 if primary || secondary => {
                let pair = usize::from(((address >> 12) & 0x0f) as u8 - 0x0a) * 2;
                let slot = pair + usize::from(secondary);
                self.vrc7.chr[slot] = value;
            }
            0xe000 if primary => self.vrc7.write_control(value),
            0xe000 if secondary => self.vrc7.irq_latch = value,
            0xf000 if primary => {
                self.vrc7.irq_pending = false;
                self.vrc7.irq_cycle_mode = value & 0x04 != 0;
                self.vrc7.irq_enabled = value & 0x02 != 0;
                self.vrc7.irq_enable_after_ack = value & 0x01 != 0;
                self.vrc7.irq_prescaler = 341;
                if self.vrc7.irq_enabled {
                    self.vrc7.irq_counter = self.vrc7.irq_latch;
                }
            }
            0xf000 if secondary => {
                self.vrc7.irq_pending = false;
                self.vrc7.irq_enabled = self.vrc7.irq_enable_after_ack;
            }
            _ => {}
        }
    }

    fn write_fme7(&mut self, address: u16, value: u8) {
        match address {
            0x8000..=0x9fff => self.fme7.command = value & 0x0f,
            0xa000..=0xbfff => match self.fme7.command {
                0..=7 => self.fme7.chr[usize::from(self.fme7.command)] = value,
                8 => self.fme7.bank_6000 = value,
                9..=11 => self.fme7.prg[usize::from(self.fme7.command - 9)] = value & 0x3f,
                12 => self.fme7.mirroring = value & 3,
                13 => {
                    self.fme7.irq_pending = false;
                    self.fme7.irq_counter_enabled = value & 0x80 != 0;
                    self.fme7.irq_output_enabled = value & 0x01 != 0;
                }
                14 => self.fme7.irq_counter = (self.fme7.irq_counter & 0xff00) | u16::from(value),
                15 => {
                    self.fme7.irq_counter =
                        (self.fme7.irq_counter & 0x00ff) | (u16::from(value) << 8)
                }
                _ => unreachable!(),
            },
            0xc000..=0xdfff => self.fme7.audio.select(value),
            0xe000..=0xffff => self.fme7.audio.write(value),
            _ => {}
        }
    }

    fn write_rambo1(&mut self, address: u16, value: u8) {
        match address & 0xe001 {
            0x8000 => self.rambo1.bank_select = value,
            0x8001 => {
                let register = usize::from(self.rambo1.bank_select & 0x0f);
                if matches!(register, 0..=9 | 15) {
                    self.rambo1.regs[register] = value;
                }
            }
            0xa000 if self.mapper == 64 && !self.four_screen => {
                self.rambo1.mirror_horizontal = value & 1 != 0;
            }
            0xc000 => self.rambo1.irq_latch = value,
            0xc001 => {
                self.rambo1.irq_counter = 0;
                self.rambo1.irq_reload = true;
                self.rambo1.irq_cycle_mode = value & 1 != 0;
                self.rambo1.irq_prescaler = 0;
                self.rambo1.irq_delay = 0;
            }
            0xe000 => {
                self.rambo1.irq_enabled = false;
                self.rambo1.irq_pending = false;
                self.rambo1.irq_delay = 0;
            }
            0xe001 => self.rambo1.irq_enabled = true,
            _ => {}
        }
    }

    fn write_mmc3(&mut self, address: u16, value: u8) {
        match address & 0xe001 {
            0x8000 => {
                self.mmc3.bank_select = value;
                if self.submapper == 1 && value & 0x20 == 0 {
                    self.mmc3.prg_ram_protect = 0;
                }
            }
            0x8001 => self.mmc3.regs[(self.mmc3.bank_select & 7) as usize] = value,
            0xa000
                if matches!(self.mapper, 4 | 37 | 47 | 119)
                    && !self.four_screen
                    && self.submapper != 2 =>
            {
                self.mmc3.mirror_horizontal = value & 1 != 0;
            }
            0xa001 if self.submapper != 1 || self.mmc3.bank_select & 0x20 != 0 => {
                self.mmc3.prg_ram_protect = value;
            }
            0xc000 => self.mmc3.irq_latch = value,
            0xc001 => {
                self.mmc3.irq_reload = true;
                if self.submapper == 3 {
                    self.mmc3.mc_acc_fall_counter = 0;
                }
            }
            0xe000 => {
                self.mmc3.irq_enabled = false;
                self.mmc3.irq_pending = false;
            }
            0xe001 => self.mmc3.irq_enabled = true,
            _ => {}
        }
    }

    fn sync_ppu(&self, ppu: &mut Ppu) {
        ppu.set_mirroring(self.mapper_mirroring());
        ppu.set_ciram_page_map(None);
        ppu.set_nametable_chr_map(None);
        ppu.chr_access_disabled = (self.mapper == 185 && !self.cnrom185_chr_enabled())
            || (self.mapper == 93 && self.simple_reg & 1 == 0);
        if self.mapper == 5 {
            ppu.set_mmc5(
                &self.mmc5,
                self.mmc5_chr_map(false),
                self.mmc5_chr_map(true),
            );
        } else if self.mapper == 19 {
            ppu.set_namco163(&self.namco163);
        } else {
            ppu.set_chr_map(self.chr_map());
            if self.mapper == 68 && self.sunsoft4.nametable_control & 0x10 != 0 {
                ppu.set_nametable_chr_map(Some(self.sunsoft4_nametable_chr_map()));
            } else if self.mapper == 95 {
                ppu.set_ciram_page_map(Some(self.namco3425_ciram_page_map()));
            } else if self.mapper == 118 {
                ppu.set_ciram_page_map(Some(self.mmc3_ciram_page_map()));
            } else if self.mapper == 158 {
                ppu.set_ciram_page_map(Some(self.rambo1_ciram_page_map()));
            } else if self.mapper == 207 {
                ppu.set_ciram_page_map(Some(self.taito_x1005_ciram_page_map()));
            }
        }
    }

    fn mmc3_chr_raw_map(&self) -> [u8; 8] {
        let r = self.mmc3.regs;
        let normal = [
            r[0] & 0xfe,
            r[0] | 1,
            r[1] & 0xfe,
            r[1] | 1,
            r[2],
            r[3],
            r[4],
            r[5],
        ];
        let inverted = [
            r[2],
            r[3],
            r[4],
            r[5],
            r[0] & 0xfe,
            r[0] | 1,
            r[1] & 0xfe,
            r[1] | 1,
        ];
        if self.mmc3.bank_select & 0x80 != 0 {
            inverted
        } else {
            normal
        }
    }

    fn mmc3_chr_bank_index(&self, raw: u8) -> usize {
        match self.mapper {
            37 => usize::from(((self.simple_reg & 4) << 5) | (raw & 0x7f)),
            47 => usize::from(((self.simple_reg & 1) << 7) | (raw & 0x7f)),
            _ => usize::from(raw),
        }
    }

    fn tqrom_chr_bank(&self, raw: u8) -> usize {
        let rom_banks = self.chr_banks_1k.max(1);
        if raw & 0x40 != 0 {
            rom_banks + usize::from(raw & 0x07)
        } else {
            let bank = usize::from(raw & 0x3f) | (usize::from(raw & 0x80) >> 1);
            bank % rom_banks
        }
    }

    fn mmc3_ciram_page_map(&self) -> [u8; 4] {
        let raw = self.mmc3_chr_raw_map();
        [raw[0] >> 7, raw[1] >> 7, raw[2] >> 7, raw[3] >> 7]
    }

    fn namco3425_ciram_page_map(&self) -> [u8; 4] {
        let lower = (self.mmc3.regs[0] >> 5) & 1;
        let upper = (self.mmc3.regs[1] >> 5) & 1;
        [lower, lower, upper, upper]
    }

    fn sunsoft4_nametable_chr_map(&self) -> [usize; 4] {
        let count = self.chr_bank_count_1k();
        let low = usize::from(0x80 | self.sunsoft4.nametable_chr[0]) % count;
        let high = usize::from(0x80 | self.sunsoft4.nametable_chr[1]) % count;
        match self.sunsoft4.nametable_control & 3 {
            0 => [low, high, low, high],
            1 => [low, low, high, high],
            2 => [low; 4],
            _ => [high; 4],
        }
    }

    fn taito_x1005_ciram_page_map(&self) -> [u8; 4] {
        let lower = self.taito_x1005.chr[0] >> 7;
        let upper = self.taito_x1005.chr[1] >> 7;
        [lower, lower, upper, upper]
    }

    fn rambo1_chr_raw_map(&self) -> [u8; 8] {
        let r = &self.rambo1.regs;
        let full_1k = self.rambo1.bank_select & 0x20 != 0;
        let lower = if full_1k {
            [r[0], r[8], r[1], r[9]]
        } else {
            [
                r[0] & 0xfe,
                (r[0] & 0xfe).wrapping_add(1),
                r[1] & 0xfe,
                (r[1] & 0xfe).wrapping_add(1),
            ]
        };
        let upper = [r[2], r[3], r[4], r[5]];
        if self.rambo1.bank_select & 0x80 != 0 {
            [
                upper[0], upper[1], upper[2], upper[3], lower[0], lower[1], lower[2], lower[3],
            ]
        } else {
            [
                lower[0], lower[1], lower[2], lower[3], upper[0], upper[1], upper[2], upper[3],
            ]
        }
    }

    fn rambo1_ciram_page_map(&self) -> [u8; 4] {
        let raw = self.rambo1_chr_raw_map();
        [raw[0] >> 7, raw[1] >> 7, raw[2] >> 7, raw[3] >> 7]
    }

    fn mmc5_chr_map(&self, sprite: bool) -> [usize; 8] {
        let count = self.chr_bank_count_1k();
        let regs: &[u16] = if sprite {
            &self.mmc5.chr_sprite
        } else {
            &self.mmc5.chr_bg
        };
        let mut map = [0usize; 8];
        let mut place = |start: usize, size: usize, register: u16| {
            let base = usize::from(register) & !(size - 1);
            for offset in 0..size {
                map[start + offset] = (base + offset) % count;
            }
        };
        match (self.mmc5.chr_mode & 3, sprite) {
            (3, true) => {
                for (slot, &register) in regs.iter().take(8).enumerate() {
                    place(slot, 1, register);
                }
            }
            (3, false) => {
                for (slot, &register) in regs.iter().take(4).cycle().take(8).enumerate() {
                    place(slot, 1, register);
                }
            }
            (2, true) => {
                for pair in 0..4 {
                    place(pair * 2, 2, regs[pair * 2 + 1]);
                }
            }
            (2, false) => {
                place(0, 2, regs[1]);
                place(2, 2, regs[3]);
                place(4, 2, regs[1]);
                place(6, 2, regs[3]);
            }
            (1, true) => {
                place(0, 4, regs[3]);
                place(4, 4, regs[7]);
            }
            (1, false) => {
                place(0, 4, regs[3]);
                place(4, 4, regs[3]);
            }
            (0, true) => place(0, 8, regs[7]),
            (0, false) => place(0, 8, regs[3]),
            _ => unreachable!(),
        }
        map
    }

    fn mapper_mirroring(&self) -> Mirroring {
        if self.bandai_74161_one_screen() {
            return if self.simple_reg & 0x80 != 0 {
                Mirroring::SingleScreen1
            } else {
                Mirroring::SingleScreen0
            };
        }
        if self.mapper == 78 {
            return if self.mapper78_holy_diver_wiring() {
                if self.simple_reg & 0x08 != 0 {
                    Mirroring::Vertical
                } else {
                    Mirroring::Horizontal
                }
            } else if self.simple_reg & 0x08 != 0 {
                Mirroring::SingleScreen1
            } else {
                Mirroring::SingleScreen0
            };
        }
        if self.mapper == 89 {
            return if self.simple_reg & 0x08 != 0 {
                Mirroring::SingleScreen1
            } else {
                Mirroring::SingleScreen0
            };
        }
        if self.mapper == 96 {
            return Mirroring::Vertical;
        }
        if self.mapper == 97 {
            return if self.simple_reg & 0x80 != 0 {
                Mirroring::Vertical
            } else {
                Mirroring::Horizontal
            };
        }
        if self.four_screen {
            return Mirroring::FourScreen;
        }
        match self.mapper {
            1 | 155 if self.submapper == 7 => self.base_mirroring,
            1 | 155 => match self.mmc1.control & 3 {
                0 => Mirroring::SingleScreen0,
                1 => Mirroring::SingleScreen1,
                2 => Mirroring::Vertical,
                _ => Mirroring::Horizontal,
            },
            4 if self.submapper == 2 => self.base_mirroring,
            4 | 37 | 47 | 119 => {
                if self.mmc3.mirror_horizontal {
                    Mirroring::Horizontal
                } else {
                    Mirroring::Vertical
                }
            }
            7 => {
                if self.simple_reg & 0x10 != 0 {
                    Mirroring::SingleScreen1
                } else {
                    Mirroring::SingleScreen0
                }
            }
            9 | 10 => {
                if self.mmc2.mirror_horizontal {
                    Mirroring::Horizontal
                } else {
                    Mirroring::Vertical
                }
            }
            16 | 153 | 157 | 159 => match self.bandai.mirroring & 3 {
                0 => Mirroring::Vertical,
                1 => Mirroring::Horizontal,
                2 => Mirroring::SingleScreen0,
                _ => Mirroring::SingleScreen1,
            },
            18 => match self.jaleco_ss88006.mirroring & 3 {
                0 => Mirroring::Horizontal,
                1 => Mirroring::Vertical,
                2 => Mirroring::SingleScreen0,
                _ => Mirroring::SingleScreen1,
            },
            21 | 22 | 23 | 25 => match self.vrc4.mirroring & if self.is_vrc2() { 1 } else { 3 } {
                0 => Mirroring::Vertical,
                1 => Mirroring::Horizontal,
                2 => Mirroring::SingleScreen0,
                _ => Mirroring::SingleScreen1,
            },
            24 | 26 => match (self.vrc6.ppu_control >> 2) & 3 {
                0 => Mirroring::Vertical,
                1 => Mirroring::Horizontal,
                2 => Mirroring::SingleScreen0,
                _ => Mirroring::SingleScreen1,
            },
            32 if self.submapper == 1 => Mirroring::SingleScreen1,
            32 => {
                if self.irem_g101.mirror_horizontal {
                    Mirroring::Horizontal
                } else {
                    Mirroring::Vertical
                }
            }
            33 | 48 => {
                if self.taito_tc0190.mirror_horizontal {
                    Mirroring::Horizontal
                } else {
                    Mirroring::Vertical
                }
            }
            64 => {
                if self.rambo1.mirror_horizontal {
                    Mirroring::Horizontal
                } else {
                    Mirroring::Vertical
                }
            }
            65 => match self.irem_h3001.mirroring & 3 {
                0 => Mirroring::Vertical,
                2 => Mirroring::Horizontal,
                _ => Mirroring::SingleScreen0,
            },
            67 => match self.sunsoft3.mirroring & 3 {
                0 => Mirroring::Vertical,
                1 => Mirroring::Horizontal,
                2 => Mirroring::SingleScreen0,
                _ => Mirroring::SingleScreen1,
            },
            68 => match self.sunsoft4.nametable_control & 3 {
                0 => Mirroring::Vertical,
                1 => Mirroring::Horizontal,
                2 => Mirroring::SingleScreen0,
                _ => Mirroring::SingleScreen1,
            },
            69 => match self.fme7.mirroring & 3 {
                0 => Mirroring::Vertical,
                1 => Mirroring::Horizontal,
                2 => Mirroring::SingleScreen0,
                _ => Mirroring::SingleScreen1,
            },
            75 => {
                if self.vrc1.mirror_horizontal {
                    Mirroring::Horizontal
                } else {
                    Mirroring::Vertical
                }
            }
            80 => {
                if self.taito_x1005.mirror_horizontal {
                    Mirroring::Horizontal
                } else {
                    Mirroring::Vertical
                }
            }
            82 | 552 => {
                if self.taito_x1017.mirror_vertical {
                    Mirroring::Vertical
                } else {
                    Mirroring::Horizontal
                }
            }
            85 => match self.vrc7.control & 3 {
                0 => Mirroring::Vertical,
                1 => Mirroring::Horizontal,
                2 => Mirroring::SingleScreen0,
                _ => Mirroring::SingleScreen1,
            },
            113 => {
                if self.simple_reg & 0x80 != 0 {
                    Mirroring::Vertical
                } else {
                    Mirroring::Horizontal
                }
            }
            154 => {
                if self.simple_reg & 0x40 != 0 {
                    Mirroring::SingleScreen1
                } else {
                    Mirroring::SingleScreen0
                }
            }
            210 if self.namco210.is_175 => self.base_mirroring,
            210 => match self.namco210.mirroring & 3 {
                0 => Mirroring::SingleScreen0,
                1 => Mirroring::Vertical,
                2 => Mirroring::SingleScreen1,
                _ => Mirroring::Horizontal,
            },
            71 if self.simple_reg & 0x80 != 0 => {
                if self.simple_reg & 0x10 != 0 {
                    Mirroring::SingleScreen1
                } else {
                    Mirroring::SingleScreen0
                }
            }
            _ => self.base_mirroring,
        }
    }

    fn chr_map(&self) -> [usize; 8] {
        let count = self.chr_bank_count_1k();
        let mut map = [0usize; 8];
        match self.mapper {
            1 | 155 => {
                if self.mmc1.control & 0x10 == 0 {
                    let base = ((self.mmc1.chr0 & 0x1e) as usize * 4) % count;
                    for (i, slot) in map.iter_mut().enumerate() {
                        *slot = (base + i) % count;
                    }
                } else {
                    let lower = self.mmc1.chr0 as usize * 4;
                    let upper = self.mmc1.chr1 as usize * 4;
                    for i in 0..4 {
                        map[i] = (lower + i) % count;
                        map[i + 4] = (upper + i) % count;
                    }
                }
            }
            3 => {
                let base = (self.prg_bank * 8) % count;
                for (i, slot) in map.iter_mut().enumerate() {
                    *slot = (base + i) % count;
                }
            }
            13 => {
                for (i, slot) in map.iter_mut().take(4).enumerate() {
                    *slot = i % count;
                }
                let upper = usize::from(self.simple_reg & 3) * 4;
                for i in 0..4 {
                    map[i + 4] = (upper + i) % count;
                }
            }
            16 | 159 => {
                for (slot, mapped) in map.iter_mut().enumerate() {
                    *mapped = usize::from(self.bandai.chr[slot]) % count;
                }
            }
            18 => {
                for (slot, mapped) in map.iter_mut().enumerate() {
                    *mapped = usize::from(self.jaleco_ss88006.chr[slot]) % count;
                }
            }
            19 => {
                for (slot, mapped) in map.iter_mut().enumerate() {
                    *mapped = usize::from(self.namco163.chr[slot]) % count;
                }
            }
            34 if self.is_nina_001() => {
                for half in 0..2 {
                    let base = usize::from(self.nina_chr[half]) * 4;
                    for i in 0..4 {
                        map[half * 4 + i] = (base + i) % count;
                    }
                }
            }
            9 | 10 => {
                for half in 0..2 {
                    let bank = if self.mmc2.latch_fe[half] {
                        self.mmc2.chr_fe[half]
                    } else {
                        self.mmc2.chr_fd[half]
                    };
                    let base = usize::from(bank) * 4;
                    for i in 0..4 {
                        map[half * 4 + i] = (base + i) % count;
                    }
                }
            }
            21 | 22 | 23 | 25 => {
                for (slot, mapped) in map.iter_mut().enumerate() {
                    let bank = if self.mapper == 22 {
                        self.vrc4.chr[slot] >> 1
                    } else {
                        self.vrc4.chr[slot]
                    };
                    *mapped = usize::from(bank) % count;
                }
            }
            24 | 26 => {
                for (slot, mapped) in map.iter_mut().enumerate() {
                    *mapped = usize::from(self.vrc6.chr[slot]) % count;
                }
            }
            32 => {
                for (slot, mapped) in map.iter_mut().enumerate() {
                    *mapped = usize::from(self.irem_g101.chr[slot]) % count;
                }
            }
            65 => {
                for (slot, mapped) in map.iter_mut().enumerate() {
                    *mapped = usize::from(self.irem_h3001.chr[slot]) % count;
                }
            }
            33 | 48 => {
                for pair in 0..2 {
                    let base = usize::from(self.taito_tc0190.chr_2k[pair]) * 2;
                    map[pair * 2] = base % count;
                    map[pair * 2 + 1] = (base + 1) % count;
                }
                for slot in 0..4 {
                    map[slot + 4] = usize::from(self.taito_tc0190.chr_1k[slot]) % count;
                }
            }
            80 | 207 => {
                let chr_mask = if self.mapper == 207 { 0x7f } else { 0xff };
                for pair in 0..2 {
                    let base = usize::from(self.taito_x1005.chr[pair] & chr_mask & 0xfe);
                    map[pair * 2] = base % count;
                    map[pair * 2 + 1] = (base + 1) % count;
                }
                for slot in 0..4 {
                    map[slot + 4] = usize::from(self.taito_x1005.chr[slot + 2] & chr_mask) % count;
                }
            }
            82 | 552 => {
                let two_k_start = if self.taito_x1017.chr_inverted { 4 } else { 0 };
                let one_k_start = if self.taito_x1017.chr_inverted { 0 } else { 4 };
                for pair in 0..2 {
                    let base = usize::from(self.taito_x1017.chr[pair] & 0xfe);
                    map[two_k_start + pair * 2] = base % count;
                    map[two_k_start + pair * 2 + 1] = (base + 1) % count;
                }
                for slot in 0..4 {
                    map[one_k_start + slot] = usize::from(self.taito_x1017.chr[slot + 2]) % count;
                }
            }
            64 | 158 => {
                for (slot, bank) in map.iter_mut().zip(self.rambo1_chr_raw_map()) {
                    *slot = usize::from(bank) % count;
                }
            }
            67 => {
                for pair in 0..4 {
                    let base = usize::from(self.sunsoft3.chr_2k[pair]) * 2;
                    map[pair * 2] = base % count;
                    map[pair * 2 + 1] = (base + 1) % count;
                }
            }
            68 => {
                for pair in 0..4 {
                    let base = usize::from(self.sunsoft4.chr_2k[pair]) * 2;
                    map[pair * 2] = base % count;
                    map[pair * 2 + 1] = (base + 1) % count;
                }
            }
            69 => {
                for (slot, mapped) in map.iter_mut().enumerate() {
                    *mapped = usize::from(self.fme7.chr[slot]) % count;
                }
            }
            75 => {
                for half in 0..2 {
                    let base = usize::from(self.vrc1.chr[half]) * 4;
                    for i in 0..4 {
                        map[half * 4 + i] = (base + i) % count;
                    }
                }
            }
            85 => {
                for (slot, mapped) in map.iter_mut().enumerate() {
                    *mapped = usize::from(self.vrc7.chr[slot]) % count;
                }
            }
            76 => {
                for pair in 0..4 {
                    let base = usize::from(self.mmc3.regs[pair + 2]) * 2;
                    map[pair * 2] = base % count;
                    map[pair * 2 + 1] = (base + 1) % count;
                }
            }
            88 | 95 | 154 | 206 => {
                let lower0 = usize::from(self.mmc3.regs[0] & 0x3e);
                let lower1 = usize::from(self.mmc3.regs[1] & 0x3e);
                map[0] = lower0 % count;
                map[1] = (lower0 + 1) % count;
                map[2] = lower1 % count;
                map[3] = (lower1 + 1) % count;
                let upper_half = usize::from(matches!(self.mapper, 88 | 154) && count > 64) * 64;
                for slot in 0..4 {
                    map[slot + 4] =
                        (upper_half + usize::from(self.mmc3.regs[slot + 2] & 0x3f)) % count;
                }
            }
            210 => {
                for (slot, mapped) in map.iter_mut().enumerate() {
                    *mapped = usize::from(self.namco210.chr[slot]) % count;
                }
            }
            72 | 92 => {
                let base = usize::from(self.jaleco_jf17.chr) * 8;
                for (i, slot) in map.iter_mut().enumerate() {
                    *slot = (base + i) % count;
                }
            }
            96 => {
                let outer = usize::from((self.simple_reg >> 2) & 1) * 4;
                let lower = (outer + usize::from(self.oeka_inner_chr & 3)) * 4;
                let upper = (outer + 3) * 4;
                for i in 0..4 {
                    map[i] = (lower + i) % count;
                    map[i + 4] = (upper + i) % count;
                }
            }
            11 | 66 | 70 | 78 | 79 | 86 | 87 | 89 | 101 | 113 | 140 | 146 | 152 => {
                let bank = match self.mapper {
                    11 | 78 => self.simple_reg >> 4,
                    66 | 140 => self.simple_reg & 3,
                    86 => (self.simple_reg & 3) | ((self.simple_reg >> 4) & 4),
                    70 | 152 => self.simple_reg & 0x0f,
                    79 | 146 => self.simple_reg & 7,
                    87 => ((self.simple_reg & 1) << 1) | ((self.simple_reg >> 1) & 1),
                    101 => self.simple_reg,
                    89 => (self.simple_reg & 7) | ((self.simple_reg >> 4) & 8),
                    113 => (self.simple_reg & 7) | ((self.simple_reg >> 3) & 8),
                    _ => unreachable!(),
                };
                let base = bank as usize * 8;
                for (i, slot) in map.iter_mut().enumerate() {
                    *slot = (base + i) % count;
                }
            }
            184 => {
                let lower = usize::from(self.simple_reg & 7) * 4;
                let upper = usize::from((self.simple_reg >> 4) & 7) * 4;
                for i in 0..4 {
                    map[i] = (lower + i) % count;
                    map[i + 4] = (upper + i) % count;
                }
            }
            119 => {
                for (slot, raw) in map.iter_mut().zip(self.mmc3_chr_raw_map()) {
                    *slot = self.tqrom_chr_bank(raw);
                }
            }
            37 | 47 => {
                for (slot, raw) in map.iter_mut().zip(self.mmc3_chr_raw_map()) {
                    *slot = self.mmc3_chr_bank_index(raw) % count;
                }
            }
            4 | 118 => {
                for (slot, bank) in map.iter_mut().zip(self.mmc3_chr_raw_map()) {
                    *slot = usize::from(bank) % count;
                }
            }
            _ => {
                for (i, slot) in map.iter_mut().enumerate() {
                    *slot = i % count;
                }
            }
        }
        map
    }

    fn chr_bank_count_1k(&self) -> usize {
        self.chr_banks_1k.max(1)
    }
    fn clock_rambo1_irq(&mut self) {
        if self.rambo1.irq_reload {
            self.rambo1.irq_counter = if self.rambo1.irq_latch == 0 {
                0
            } else {
                self.rambo1.irq_latch | 1
            };
            self.rambo1.irq_reload = false;
        } else if self.rambo1.irq_counter == 0 {
            self.rambo1.irq_counter = self.rambo1.irq_latch;
        } else {
            self.rambo1.irq_counter = self.rambo1.irq_counter.wrapping_sub(1);
        }
        if self.rambo1.irq_counter == 0 && self.rambo1.irq_enabled {
            self.rambo1.irq_delay = 2;
        }
    }

    fn tick_rambo1_cpu_irq(&mut self) {
        if self.rambo1.irq_cycle_mode {
            self.rambo1.irq_prescaler = self.rambo1.irq_prescaler.wrapping_add(1);
            if self.rambo1.irq_prescaler >= 4 {
                self.rambo1.irq_prescaler = 0;
                self.clock_rambo1_irq();
            }
        }
        if self.rambo1.irq_delay != 0 {
            self.rambo1.irq_delay -= 1;
            if self.rambo1.irq_delay == 0 && self.rambo1.irq_enabled {
                self.rambo1.irq_pending = true;
            }
        }
    }

    fn clock_taito_tc0690_irq(&mut self) {
        if self.taito_tc0190.irq_counter == 0 || self.taito_tc0190.irq_reload {
            self.taito_tc0190.irq_counter = self.taito_tc0190.irq_latch;
            self.taito_tc0190.irq_reload = false;
        } else {
            self.taito_tc0190.irq_counter -= 1;
        }
        if self.taito_tc0190.irq_counter == 0 && self.taito_tc0190.irq_enabled {
            self.taito_tc0190.irq_delay = 4;
        }
    }

    fn clock_mmc3_irq(&mut self) {
        let previous = self.mmc3.irq_counter;
        let reloading = previous == 0 || self.mmc3.irq_reload;
        if reloading {
            self.mmc3.irq_counter = self.mmc3.irq_latch;
            self.mmc3.irq_reload = false;
        } else {
            self.mmc3.irq_counter -= 1;
        }
        let reached_zero = if self.submapper == 4 {
            !reloading && previous != 0 && self.mmc3.irq_counter == 0
        } else {
            self.mmc3.irq_counter == 0
        };
        if reached_zero && self.mmc3.irq_enabled {
            self.mmc3.irq_pending = true;
        }
    }

    fn observe_ppu_a12(&mut self, high: bool) {
        if self.mapper == 48 {
            if high {
                if !self.taito_tc0190.a12_high && self.taito_tc0190.a12_low_cycles >= 8 {
                    self.clock_taito_tc0690_irq();
                }
                self.taito_tc0190.a12_high = true;
                self.taito_tc0190.a12_low_cycles = 0;
            } else {
                self.taito_tc0190.a12_high = false;
                self.taito_tc0190.a12_low_cycles =
                    self.taito_tc0190.a12_low_cycles.saturating_add(1);
            }
            return;
        }
        if matches!(self.mapper, 64 | 158) {
            if !self.rambo1.irq_cycle_mode
                && high
                && !self.rambo1.a12_high
                && self.rambo1.a12_low_cycles >= 16
            {
                self.clock_rambo1_irq();
            }
            self.rambo1.a12_high = high;
            self.rambo1.a12_low_cycles = if high {
                0
            } else {
                self.rambo1.a12_low_cycles.saturating_add(1)
            };
            return;
        }
        if !matches!(self.mapper, 4 | 37 | 47 | 118 | 119) {
            return;
        }
        if self.submapper == 3 {
            if self.mmc3.a12_high && !high {
                self.mmc3.mc_acc_fall_counter += 1;
                if self.mmc3.mc_acc_fall_counter == 1 {
                    self.clock_mmc3_irq();
                } else if self.mmc3.mc_acc_fall_counter == 8 {
                    self.mmc3.mc_acc_fall_counter = 0;
                }
            }
            self.mmc3.a12_high = high;
            self.mmc3.a12_low_cycles = if high {
                0
            } else {
                self.mmc3.a12_low_cycles.saturating_add(1)
            };
            return;
        }
        if high {
            if !self.mmc3.a12_high && self.mmc3.a12_low_cycles >= 8 {
                self.clock_mmc3_irq();
            }
            self.mmc3.a12_high = true;
            self.mmc3.a12_low_cycles = 0;
        } else {
            self.mmc3.a12_high = false;
            self.mmc3.a12_low_cycles = self.mmc3.a12_low_cycles.saturating_add(1);
        }
    }

    fn observe_ppu_latch_address(&mut self, address: u16) -> bool {
        if self.mapper == 96 {
            let dd = ((address >> 12) & 3) as u8;
            let entered_nametable = self.oeka_last_dd != 2 && dd == 2;
            self.oeka_last_dd = dd;
            if !entered_nametable {
                return false;
            }
            let inner = ((address >> 8) & 3) as u8;
            if inner == self.oeka_inner_chr {
                return false;
            }
            self.oeka_inner_chr = inner;
            return true;
        }
        if !matches!(self.mapper, 9 | 10) {
            return false;
        }
        let previous = self.mmc2.latch_fe;
        match address & 0x1fff {
            0x0fd8..=0x0fdf => self.mmc2.latch_fe[0] = false,
            0x0fe8..=0x0fef => self.mmc2.latch_fe[0] = true,
            0x1fd8..=0x1fdf => self.mmc2.latch_fe[1] = false,
            0x1fe8..=0x1fef => self.mmc2.latch_fe[1] = true,
            _ => {}
        }
        self.mmc2.latch_fe != previous
    }

    fn observe_ppu_address(&mut self, address: u16) -> bool {
        self.observe_ppu_a12(address & 0x1000 != 0);
        self.observe_ppu_latch_address(address)
    }

    fn observe_mmc5_ppu_dot(&mut self, scanline: u16, cycle: u16, rendering: bool) {
        if self.mapper != 5 {
            return;
        }
        if !rendering || scanline >= 240 {
            if self.mmc5.in_frame {
                self.mmc5.in_frame = false;
                self.mmc5.irq_counter = 0;
                self.mmc5.irq_pending = false;
            }
            return;
        }
        if cycle != 4 {
            return;
        }
        if !self.mmc5.in_frame {
            self.mmc5.in_frame = true;
            self.mmc5.irq_counter = 0;
            self.mmc5.irq_pending = false;
        } else {
            self.mmc5.irq_counter = self.mmc5.irq_counter.wrapping_add(1);
            if self.mmc5.irq_compare != 0 && self.mmc5.irq_counter == self.mmc5.irq_compare {
                self.mmc5.irq_pending = true;
            }
        }
    }

    fn reset_mmc5_scanline_counter(&mut self) {
        if self.mapper == 5 {
            self.mmc5.in_frame = false;
            self.mmc5.irq_counter = 0;
            self.mmc5.irq_pending = false;
        }
    }

    fn observe_mmc5_ppu_mask_write(&mut self, previous: u8, value: u8) {
        if self.mapper == 5 && (value & 0x18 == 0 || (previous & 0x18 == 0 && value & 0x18 != 0)) {
            self.reset_mmc5_scanline_counter();
        }
    }

    fn observe_mmc5_pcm_read(&mut self, address: u16, value: u8) {
        if self.mapper != 5 || !self.mmc5.pcm_mode || !(0x8000..=0xbfff).contains(&address) {
            return;
        }
        if value == 0 {
            self.mmc5.pcm_irq_pending = true;
        } else {
            self.mmc5.pcm = value;
            self.mmc5.pcm_irq_pending = false;
        }
    }

    fn irq_pending(&self) -> bool {
        (self.mapper == 5
            && ((self.mmc5.irq_pending && self.mmc5.irq_enabled)
                || (self.mmc5.pcm_irq_pending && self.mmc5.pcm_irq_enabled)))
            || (matches!(self.mapper, 4 | 37 | 47 | 118 | 119) && self.mmc3.irq_pending)
            || (self.mapper == 48 && self.taito_tc0190.irq_pending)
            || (matches!(self.mapper, 16 | 153 | 157 | 159) && self.bandai.irq_pending)
            || (self.mapper == 19 && self.namco163.irq_pending)
            || (matches!(self.mapper, 64 | 158) && self.rambo1.irq_pending)
            || (self.mapper == 65 && self.irem_h3001.irq_pending)
            || (self.mapper == 67 && self.sunsoft3.irq_pending)
            || (self.mapper == 18 && self.jaleco_ss88006.irq_pending)
            || (self.is_vrc4() && self.vrc4.irq_pending)
            || (self.is_vrc6() && self.vrc6.irq_pending)
            || (self.mapper == 85 && self.vrc7.irq_pending)
            || (self.mapper == 69 && self.fme7.irq_pending)
            || (self.mapper == 73 && self.vrc3.irq_pending)
            || (matches!(self.mapper, 82 | 552) && self.taito_x1017.irq_pending)
    }

    fn clock_vrc_irq(&mut self) {
        if self.vrc4.irq_counter == 0xff {
            self.vrc4.irq_counter = self.vrc4.irq_latch;
            self.vrc4.irq_pending = true;
        } else {
            self.vrc4.irq_counter = self.vrc4.irq_counter.wrapping_add(1);
        }
    }

    fn clock_vrc6_irq(&mut self) {
        if self.vrc6.irq_counter == 0xff {
            self.vrc6.irq_counter = self.vrc6.irq_latch;
            self.vrc6.irq_pending = true;
        } else {
            self.vrc6.irq_counter = self.vrc6.irq_counter.wrapping_add(1);
        }
    }

    fn clock_vrc7_irq(&mut self) {
        if self.vrc7.irq_counter == 0xff {
            self.vrc7.irq_counter = self.vrc7.irq_latch;
            self.vrc7.irq_pending = true;
        } else {
            self.vrc7.irq_counter = self.vrc7.irq_counter.wrapping_add(1);
        }
    }

    fn clock_jaleco_ss88006_irq(&mut self) {
        let mask = if self.jaleco_ss88006.irq_control & 0x08 != 0 {
            0x000f
        } else if self.jaleco_ss88006.irq_control & 0x04 != 0 {
            0x00ff
        } else if self.jaleco_ss88006.irq_control & 0x02 != 0 {
            0x0fff
        } else {
            0xffff
        };
        let upper = self.jaleco_ss88006.irq_counter & !mask;
        let counter = (self.jaleco_ss88006.irq_counter & mask).wrapping_sub(1) & mask;
        if counter == 0 {
            self.jaleco_ss88006.irq_pending = true;
        }
        self.jaleco_ss88006.irq_counter = upper | counter;
    }

    fn tick_cpu_cycle(&mut self) {
        if self.mapper == 48 && self.taito_tc0190.irq_delay != 0 {
            self.taito_tc0190.irq_delay -= 1;
            if self.taito_tc0190.irq_delay == 0 && self.taito_tc0190.irq_enabled {
                self.taito_tc0190.irq_pending = true;
            }
        }
        if self.mapper == 5 {
            self.mmc5.tick_audio();
        }
        if self.mapper == 19 {
            self.namco163.tick_audio();
            if self.namco163.irq_enabled && self.namco163.irq_counter < 0x7fff {
                self.namco163.irq_counter += 1;
                if self.namco163.irq_counter == 0x7fff {
                    self.namco163.irq_pending = true;
                }
            }
        }
        if matches!(self.mapper, 16 | 153 | 157 | 159) && self.bandai.irq_enabled {
            if self.bandai.irq_counter == 0 {
                self.bandai.irq_pending = true;
            } else {
                self.bandai.irq_counter -= 1;
                if self.bandai.irq_counter == 0 {
                    self.bandai.irq_pending = true;
                }
            }
        }
        if self.mapper == 65 && self.irem_h3001.irq_enabled {
            self.irem_h3001.irq_counter = self.irem_h3001.irq_counter.wrapping_sub(1);
            if self.irem_h3001.irq_counter == 0 {
                self.irem_h3001.irq_enabled = false;
                self.irem_h3001.irq_pending = true;
            }
        }
        if self.mapper == 67 && self.sunsoft3.irq_enabled {
            let previous = self.sunsoft3.irq_counter;
            self.sunsoft3.irq_counter = self.sunsoft3.irq_counter.wrapping_sub(1);
            if previous == 0 {
                self.sunsoft3.irq_enabled = false;
                self.sunsoft3.irq_pending = true;
            }
        }
        if matches!(self.mapper, 82 | 552)
            && self.taito_x1017.irq_control & 1 != 0
            && self.taito_x1017.irq_control & 4 == 0
            && self.taito_x1017.irq_counter != 0
        {
            self.taito_x1017.irq_counter -= 1;
            if self.taito_x1017.irq_counter == 0 && self.taito_x1017.irq_control & 2 != 0 {
                self.taito_x1017.irq_pending = true;
            }
        }
        if matches!(self.mapper, 64 | 158) {
            self.tick_rambo1_cpu_irq();
        }
        if self.mapper == 18 && self.jaleco_ss88006.irq_control & 1 != 0 {
            self.clock_jaleco_ss88006_irq();
        }

        if self.is_vrc6() {
            self.vrc6.tick_audio();
            if self.vrc6.irq_enabled {
                if self.vrc6.irq_cycle_mode {
                    self.clock_vrc6_irq();
                } else {
                    self.vrc6.irq_prescaler -= 3;
                    if self.vrc6.irq_prescaler <= 0 {
                        self.vrc6.irq_prescaler += 341;
                        self.clock_vrc6_irq();
                    }
                }
            }
        }

        if self.is_vrc4() && self.vrc4.irq_enabled {
            if self.vrc4.irq_cycle_mode {
                self.clock_vrc_irq();
            } else {
                self.vrc4.irq_prescaler -= 3;
                if self.vrc4.irq_prescaler <= 0 {
                    self.vrc4.irq_prescaler += 341;
                    self.clock_vrc_irq();
                }
            }
        }

        if self.mapper == 85 {
            self.vrc7.tick_audio(self.region.cpu_clock() as f32);
            if self.vrc7.irq_enabled {
                if self.vrc7.irq_cycle_mode {
                    self.clock_vrc7_irq();
                } else {
                    self.vrc7.irq_prescaler -= 3;
                    if self.vrc7.irq_prescaler <= 0 {
                        self.vrc7.irq_prescaler += 341;
                        self.clock_vrc7_irq();
                    }
                }
            }
        }

        if self.mapper == 69 {
            self.fme7.audio.tick();
            if self.fme7.irq_counter_enabled {
                let previous = self.fme7.irq_counter;
                self.fme7.irq_counter = self.fme7.irq_counter.wrapping_sub(1);
                if previous == 0 && self.fme7.irq_output_enabled {
                    self.fme7.irq_pending = true;
                }
            }
        }

        if self.mapper == 73 && self.vrc3.irq_enabled {
            if self.vrc3.irq_mode_8bit {
                let low = self.vrc3.irq_counter as u8;
                if low == 0xff {
                    self.vrc3.irq_counter =
                        (self.vrc3.irq_counter & 0xff00) | (self.vrc3.irq_latch & 0x00ff);
                    self.vrc3.irq_pending = true;
                } else {
                    self.vrc3.irq_counter =
                        (self.vrc3.irq_counter & 0xff00) | u16::from(low.wrapping_add(1));
                }
            } else if self.vrc3.irq_counter == 0xffff {
                self.vrc3.irq_counter = self.vrc3.irq_latch;
                self.vrc3.irq_pending = true;
            } else {
                self.vrc3.irq_counter = self.vrc3.irq_counter.wrapping_add(1);
            }
        }
    }

    fn expansion_audio_output(&self) -> f32 {
        if self.mapper == 5 {
            self.mmc5.audio_output()
        } else if self.mapper == 19 {
            self.namco163.audio_output
        } else if self.is_vrc6() {
            self.vrc6.audio_output()
        } else if self.mapper == 85 {
            self.vrc7.audio_output
        } else if self.mapper == 69 {
            self.fme7.audio.output()
        } else {
            0.0
        }
    }

    fn mmc6_ram_permissions(&self, address: u16) -> Option<(usize, bool, bool, bool)> {
        if self.mapper != 4 || self.submapper != 1 || address < 0x7000 || self.prg_ram.is_empty() {
            return None;
        }
        let index = (address as usize - 0x7000) & 0x03ff;
        let upper = index >= 0x0200;
        let read_enabled = self.mmc3.prg_ram_protect & if upper { 0x08 } else { 0x02 } != 0;
        let write_enabled = self.mmc3.prg_ram_protect & if upper { 0x04 } else { 0x01 } != 0;
        let global_enabled = self.mmc3.bank_select & 0x20 != 0;
        Some((index, global_enabled, read_enabled, write_enabled))
    }

    fn mmc1_ram_bank(&self) -> usize {
        match self.prg_ram.len() {
            0..=0x2000 => 0,
            0x2001..=0x4000 if self.chr_banks_1k > 8 => usize::from((self.mmc1.chr0 >> 4) & 1),
            0x2001..=0x4000 => usize::from((self.mmc1.chr0 >> 3) & 1),
            _ => usize::from((self.mmc1.chr0 >> 2) & 3),
        }
    }

    fn mmc1_ram_enabled(&self) -> bool {
        if self.mapper == 155 {
            return true;
        }
        if self.mmc1.prg & 0x10 != 0 {
            return false;
        }
        let snrom = self.prg_rom.len() <= 256 * 1024
            && self.chr_is_ram
            && self.chr_banks_1k == 8
            && self.prg_ram.len() <= 8 * 1024;
        !snrom || self.mmc1.chr0 & 0x10 == 0
    }

    fn ram_readable(&self) -> bool {
        if self.prg_ram.is_empty() {
            return false;
        }
        match self.mapper {
            1 | 155 => self.mmc1_ram_enabled(),
            4 | 118 => self.mmc3.prg_ram_protect & 0x80 != 0,
            153 => self.simple_reg & 0x20 != 0,
            18 => self.jaleco_ss88006.ram_control & 1 != 0,
            9 | 22 => false,
            21 | 23 | 25 if self.is_vrc4() => self.vrc4.wram_enable,
            23 if self.submapper == 3 => false,
            24 | 26 => self.vrc6.ppu_control & 0x80 != 0,
            68 => self.sunsoft4.prg_control & 0x10 != 0,
            85 => self.vrc7.control & 0x80 != 0,
            75 | 76 | 88 | 95 | 154 | 206 => false,
            80 | 207 => self.taito_x1005.ram_enabled,
            82 | 552 => self
                .taito_x1017
                .ram_enabled
                .into_iter()
                .any(|enabled| enabled),
            210 => self.namco210.is_175 && self.namco210.ram_enabled,
            _ => true,
        }
    }
    fn ram_writable(&self, address: u16) -> bool {
        if !self.ram_readable() {
            return false;
        }
        match self.mapper {
            4 | 118 => self.mmc3.prg_ram_protect & 0x40 == 0,
            18 => self.jaleco_ss88006.ram_control & 2 != 0,
            19 => {
                let window = usize::from((address - 0x6000) / 0x0800);
                self.namco163.wram_protect & 0xf0 == 0x40
                    && self.namco163.wram_protect & (1 << window) == 0
            }
            _ => true,
        }
    }
    fn read_ram(&self, address: u16) -> u8 {
        if self.mapper == 86 {
            return 0xff;
        }
        if matches!(self.mapper, 82 | 552) {
            let mapped = match address {
                0x6000..=0x67ff if self.taito_x1017.ram_enabled[0] => {
                    Some(address as usize - 0x6000)
                }
                0x6800..=0x6fff if self.taito_x1017.ram_enabled[1] => {
                    Some(0x800 + address as usize - 0x6800)
                }
                0x7000..=0x73ff if self.taito_x1017.ram_enabled[2] => {
                    Some(0x1000 + address as usize - 0x7000)
                }
                _ => None,
            };
            return mapped.map_or(0, |index| self.prg_ram[index]);
        }
        if matches!(self.mapper, 80 | 207) {
            if !self.taito_x1005.ram_enabled || address < 0x7f00 {
                return 0xff;
            }
            return self.prg_ram[(address as usize - 0x7f00) & 0x7f];
        }
        if self.mapper == 157 {
            let eeprom_output = self.bandai.eeprom.output
                && (!self.bandai.external_eeprom_present || self.bandai.external_eeprom.output);
            return (u8::from(eeprom_output) << 4) | (u8::from(self.bandai.barcode_line) << 3);
        }
        if matches!(self.mapper, 16 | 159) && self.bandai.eeprom_present {
            return u8::from(self.bandai.eeprom.output) << 4;
        }
        if self.mapper == 69 {
            let bank = self.fme7.bank_6000;
            let offset = address as usize - 0x6000;
            if bank & 0x40 == 0 {
                let banks = self.prg_rom.len() / 0x2000;
                let selected = usize::from(bank & 0x3f) % banks;
                return self.prg_rom[selected * 0x2000 + offset];
            }
            if bank & 0x80 == 0 || self.prg_ram.is_empty() {
                return 0xff;
            }
            let banks = (self.prg_ram.len() / 0x2000).max(1);
            let selected = usize::from(bank & 0x3f) % banks;
            return self.prg_ram[(selected * 0x2000 + offset) % self.prg_ram.len()];
        }
        if self.mapper == 5 {
            if self.prg_ram.is_empty() {
                return 0xff;
            }
            let banks = (self.prg_ram.len() / 0x2000).max(1);
            let bank = usize::from(self.mmc5.prg_ram_bank & 0x0f) % banks;
            return self.prg_ram[bank * 0x2000 + (address as usize - 0x6000)];
        }
        if self.vrc2_uses_latch() {
            return if address < 0x7000 {
                0x60 | self.vrc4.latch_6000
            } else {
                0xff
            };
        }
        if self.mapper == 4 && self.submapper == 1 {
            let Some((index, global, readable, _)) = self.mmc6_ram_permissions(address) else {
                return 0xff;
            };
            if !global {
                return 0xff;
            }
            if readable {
                return self.prg_ram[index];
            }
            let any_readable = self.mmc3.prg_ram_protect & 0x0a != 0;
            return if any_readable { 0x00 } else { 0xff };
        }
        if matches!(self.mapper, 1 | 155) {
            if !self.ram_readable() {
                return 0xff;
            }
            let banks = (self.prg_ram.len() / 0x2000).max(1);
            let bank = self.mmc1_ram_bank() % banks;
            let offset = address as usize - 0x6000;
            return self.prg_ram[bank * 0x2000 + offset];
        }
        if !self.ram_readable() {
            return 0xff;
        }
        let index = (address as usize - 0x6000) % self.prg_ram.len();
        self.prg_ram[index]
    }
    fn write_ram(&mut self, address: u16, value: u8) {
        if matches!(self.mapper, 37 | 47) {
            if self.mmc3.prg_ram_protect & 0xc0 == 0x80 {
                self.simple_reg = if self.mapper == 37 {
                    value & 7
                } else {
                    value & 1
                };
            }
            return;
        }
        if self.mapper == 86 {
            self.write_jaleco_jf13(address, value);
            return;
        }
        if matches!(self.mapper, 82 | 552) {
            let mapped = match address {
                0x6000..=0x67ff if self.taito_x1017.ram_enabled[0] => {
                    Some(address as usize - 0x6000)
                }
                0x6800..=0x6fff if self.taito_x1017.ram_enabled[1] => {
                    Some(0x800 + address as usize - 0x6800)
                }
                0x7000..=0x73ff if self.taito_x1017.ram_enabled[2] => {
                    Some(0x1000 + address as usize - 0x7000)
                }
                _ => None,
            };
            if let Some(index) = mapped {
                self.prg_ram[index] = value;
            }
            return;
        }
        if matches!(self.mapper, 80 | 207) {
            if self.taito_x1005.ram_enabled && address >= 0x7f00 {
                self.prg_ram[(address as usize - 0x7f00) & 0x7f] = value;
            }
            return;
        }
        if self.mapper == 69 {
            let bank = self.fme7.bank_6000;
            if bank & 0xc0 != 0xc0 || self.prg_ram.is_empty() {
                return;
            }
            let banks = (self.prg_ram.len() / 0x2000).max(1);
            let selected = usize::from(bank & 0x3f) % banks;
            let offset = address as usize - 0x6000;
            let index = (selected * 0x2000 + offset) % self.prg_ram.len();
            self.prg_ram[index] = value;
            return;
        }
        if self.mapper == 5 {
            if !self.mmc5.ram_write_enabled() || self.prg_ram.is_empty() {
                return;
            }
            let banks = (self.prg_ram.len() / 0x2000).max(1);
            let bank = usize::from(self.mmc5.prg_ram_bank & 0x0f) % banks;
            self.prg_ram[bank * 0x2000 + (address as usize - 0x6000)] = value;
            return;
        }
        if self.mapper == 4 && self.submapper == 1 {
            let Some((index, global, readable, writable)) = self.mmc6_ram_permissions(address)
            else {
                return;
            };
            if global && readable && writable {
                self.prg_ram[index] = value;
            }
            return;
        }
        if matches!(self.mapper, 1 | 155) {
            if !self.ram_writable(address) {
                return;
            }
            let banks = (self.prg_ram.len() / 0x2000).max(1);
            let bank = self.mmc1_ram_bank() % banks;
            let offset = address as usize - 0x6000;
            self.prg_ram[bank * 0x2000 + offset] = value;
            return;
        }
        if !self.ram_writable(address) {
            return;
        }
        let index = (address as usize - 0x6000) % self.prg_ram.len();
        self.prg_ram[index] = value;
    }
}

struct Ppu {
    chr: Vec<u8>,
    chr_ram: bool,
    chr_rom_bank_count: usize,
    chr_access_disabled: bool,
    chr_map: [usize; 8],
    bg_chr_map: [usize; 8],
    sprite_chr_map: [usize; 8],
    chr_bank_count: usize,
    mirroring: Mirroring,
    nametable: [u8; 4096],
    ciram_page_map: Option<[u8; 4]>,
    nametable_chr_map: Option<[usize; 4]>,
    n163_enabled: bool,
    n163_pattern: [u8; 8],
    n163_nametable: [u8; 4],
    n163_chr_disable_low: bool,
    n163_chr_disable_high: bool,
    mmc5_enabled: bool,
    mmc5_exram_mode: u8,
    mmc5_nametable_map: u8,
    mmc5_fill_tile: u8,
    mmc5_fill_color: u8,
    mmc5_chr_upper: u8,
    mmc5_split_control: u8,
    mmc5_split_scroll: u8,
    mmc5_split_bank: u8,
    mmc5_exram: [u8; 1024],
    palette: [u8; 32],
    oam: [u8; 256],
    video: VideoBuffer,
    ctrl: u8,
    mask: u8,
    status: u8,
    oam_addr: u8,
    v: u16,
    t: u16,
    fine_x: u8,
    write_toggle: bool,
    data_buffer: u8,
    io_latch: u8,
    scroll_x: u8,
    scroll_y: u8,
    bg_pattern_low_shift: u16,
    bg_pattern_high_shift: u16,
    bg_attr_low_shift: u16,
    bg_attr_high_shift: u16,
    bg_next_tile: u8,
    bg_next_attr: u8,
    bg_next_ex_attr: u8,
    bg_next_split: bool,
    bg_next_split_row: u8,
    bg_next_low: u8,
    bg_next_high: u8,
    cycle: u16,
    scanline: u16,
    frame: u64,
    odd_frame: bool,
    nmi_pending: bool,
    suppress_vblank: bool,
    mapper_address: Option<u16>,
    region: NesRegion,
}

impl Ppu {
    fn new(chr: Vec<u8>, chr_ram: bool, mirroring: Mirroring) -> Self {
        let chr_bank_count = (chr.len() / 0x0400).max(1);
        let chr_rom_bank_count = if chr_ram { 0 } else { chr_bank_count };
        Self {
            chr_bank_count,
            chr_rom_bank_count,
            chr_access_disabled: false,
            chr_map: [0, 1, 2, 3, 4, 5, 6, 7],
            bg_chr_map: [0, 1, 2, 3, 4, 5, 6, 7],
            sprite_chr_map: [0, 1, 2, 3, 4, 5, 6, 7],
            chr,
            chr_ram,
            mirroring,
            nametable: [0; 4096],
            ciram_page_map: None,
            nametable_chr_map: None,
            n163_enabled: false,
            n163_pattern: [0; 8],
            n163_nametable: [0xe0, 0xe1, 0xe0, 0xe1],
            n163_chr_disable_low: false,
            n163_chr_disable_high: false,
            mmc5_enabled: false,
            mmc5_exram_mode: 0,
            mmc5_nametable_map: 0,
            mmc5_fill_tile: 0,
            mmc5_fill_color: 0,
            mmc5_chr_upper: 0,
            mmc5_split_control: 0,
            mmc5_split_scroll: 0,
            mmc5_split_bank: 0,
            mmc5_exram: [0; 1024],
            palette: [0; 32],
            oam: [0; 256],
            video: VideoBuffer::new(WIDTH, HEIGHT),
            ctrl: 0,
            mask: 0,
            status: 0,
            oam_addr: 0,
            v: 0,
            t: 0,
            fine_x: 0,
            write_toggle: false,
            data_buffer: 0,
            io_latch: 0,
            scroll_x: 0,
            scroll_y: 0,
            bg_pattern_low_shift: 0,
            bg_pattern_high_shift: 0,
            bg_attr_low_shift: 0,
            bg_attr_high_shift: 0,
            bg_next_tile: 0,
            bg_next_attr: 0,
            bg_next_ex_attr: 0,
            bg_next_split: false,
            bg_next_split_row: 0,
            bg_next_low: 0,
            bg_next_high: 0,
            cycle: 0,
            scanline: 0,
            frame: 0,
            odd_frame: false,
            nmi_pending: false,
            suppress_vblank: false,
            mapper_address: None,
            region: NesRegion::Ntsc,
        }
    }

    fn append_chr_ram(&mut self, size: usize) {
        debug_assert!(size.is_multiple_of(0x400));
        self.chr.resize(self.chr.len() + size, 0);
        self.chr_bank_count = (self.chr.len() / 0x400).max(1);
        self.chr_ram = true;
    }

    fn chr_bank_writable(&self, bank: usize) -> bool {
        self.chr_ram && bank >= self.chr_rom_bank_count
    }

    fn reset(&mut self) {
        self.ctrl = 0;
        self.mask = 0;
        self.status = 0;
        self.oam_addr = 0;
        self.v = 0;
        self.t = 0;
        self.fine_x = 0;
        self.write_toggle = false;
        self.data_buffer = 0;
        self.io_latch = 0;
        self.scroll_x = 0;
        self.scroll_y = 0;
        self.bg_pattern_low_shift = 0;
        self.bg_pattern_high_shift = 0;
        self.bg_attr_low_shift = 0;
        self.bg_attr_high_shift = 0;
        self.bg_next_tile = 0;
        self.bg_next_attr = 0;
        self.bg_next_ex_attr = 0;
        self.bg_next_split = false;
        self.bg_next_split_row = 0;
        self.bg_next_low = 0;
        self.bg_next_high = 0;
        self.cycle = 0;
        self.scanline = 0;
        self.frame = 0;
        self.odd_frame = false;
        self.nmi_pending = false;
        self.suppress_vblank = false;
        self.mapper_address = None;
        self.video.clear([0, 0, 0, 255]);
    }

    fn read_chr_mapped(&self, address: u16, map: &[usize; 8]) -> u8 {
        if self.chr_access_disabled {
            return 0xff;
        }
        let slot = address as usize / 0x400;
        let bank = map[slot] % self.chr_bank_count;
        self.chr[bank * 0x400 + (address as usize & 0x3ff)]
    }

    fn write_chr_mapped(&mut self, address: u16, value: u8, map: &[usize; 8]) {
        if self.chr_access_disabled {
            return;
        }
        let slot = address as usize / 0x400;
        let bank = map[slot] % self.chr_bank_count;
        if !self.chr_bank_writable(bank) {
            return;
        }
        self.chr[bank * 0x400 + (address as usize & 0x3ff)] = value;
    }

    fn n163_pattern_uses_ciram(&self, slot: usize, raw: u8) -> bool {
        raw >= 0xe0
            && if slot < 4 {
                !self.n163_chr_disable_low
            } else {
                !self.n163_chr_disable_high
            }
    }

    fn read_n163_pattern(&self, address: u16) -> u8 {
        let slot = usize::from(address) / 0x400;
        let raw = self.n163_pattern[slot];
        let inner = usize::from(address) & 0x3ff;
        if self.n163_pattern_uses_ciram(slot, raw) {
            self.nametable[usize::from(raw & 1) * 0x400 + inner]
        } else {
            let bank = usize::from(raw) % self.chr_bank_count;
            self.chr[bank * 0x400 + inner]
        }
    }

    fn write_n163_pattern(&mut self, address: u16, value: u8) {
        let slot = usize::from(address) / 0x400;
        let raw = self.n163_pattern[slot];
        let inner = usize::from(address) & 0x3ff;
        if self.n163_pattern_uses_ciram(slot, raw) {
            self.nametable[usize::from(raw & 1) * 0x400 + inner] = value;
        } else if self.chr_ram {
            let bank = usize::from(raw) % self.chr_bank_count;
            self.chr[bank * 0x400 + inner] = value;
        }
    }

    fn read_n163_nametable(&self, address: u16) -> u8 {
        let offset = (usize::from(address) - 0x2000) & 0x0fff;
        let raw = self.n163_nametable[offset / 0x400];
        let inner = offset & 0x3ff;
        if raw >= 0xe0 {
            self.nametable[usize::from(raw & 1) * 0x400 + inner]
        } else {
            let bank = usize::from(raw) % self.chr_bank_count;
            self.chr[bank * 0x400 + inner]
        }
    }

    fn write_n163_nametable(&mut self, address: u16, value: u8) {
        let offset = (usize::from(address) - 0x2000) & 0x0fff;
        let raw = self.n163_nametable[offset / 0x400];
        let inner = offset & 0x3ff;
        if raw >= 0xe0 {
            self.nametable[usize::from(raw & 1) * 0x400 + inner] = value;
        } else if self.chr_ram {
            let bank = usize::from(raw) % self.chr_bank_count;
            self.chr[bank * 0x400 + inner] = value;
        }
    }

    fn read_background_pattern(&self, address: u16) -> u8 {
        if self.n163_enabled {
            return self.read_n163_pattern(address);
        }
        if self.mmc5_enabled && self.bg_next_split {
            let bank_4k = usize::from(self.mmc5_split_bank);
            let bank = bank_4k * 4 + ((address as usize >> 10) & 3);
            return self.chr[(bank % self.chr_bank_count) * 0x400 + (address as usize & 0x3ff)];
        }
        if self.mmc5_enabled && self.mmc5_exram_mode == 1 && self.rendering_enabled() {
            let bank_4k = (usize::from(self.mmc5_chr_upper & 3) << 6)
                | usize::from(self.bg_next_ex_attr & 0x3f);
            let bank = bank_4k * 4 + ((address as usize >> 10) & 3);
            return self.chr[(bank % self.chr_bank_count) * 0x400 + (address as usize & 0x3ff)];
        }
        let map = if self.mmc5_enabled && self.ctrl & 0x20 == 0 {
            &self.sprite_chr_map
        } else {
            &self.bg_chr_map
        };
        self.read_chr_mapped(address, map)
    }

    fn read_sprite_pattern(&self, address: u16) -> u8 {
        if self.n163_enabled {
            self.read_n163_pattern(address)
        } else {
            self.read_chr_mapped(address, &self.sprite_chr_map)
        }
    }

    fn mmc5_nametable_source(&self, address: u16) -> (u8, usize) {
        let offset = (address as usize - 0x2000) & 0x0fff;
        let table = offset / 0x400;
        ((self.mmc5_nametable_map >> (table * 2)) & 3, offset & 0x3ff)
    }

    fn read_nametable(&self, address: u16) -> u8 {
        if self.n163_enabled {
            return self.read_n163_nametable(address);
        }
        if let Some(map) = self.nametable_chr_map {
            let offset = (usize::from(address) - 0x2000) & 0x0fff;
            let bank = map[offset / 0x400] % self.chr_bank_count;
            return self.chr[bank * 0x400 + (offset & 0x3ff)];
        }
        if !self.mmc5_enabled {
            return self.nametable[self.mirror_nametable(address)];
        }
        let (source, inner) = self.mmc5_nametable_source(address);
        match source {
            0 | 1 => self.nametable[usize::from(source) * 0x400 + inner],
            2 if self.mmc5_exram_mode <= 1 => self.mmc5_exram[inner],
            2 => 0,
            3 if inner < 0x3c0 => self.mmc5_fill_tile,
            3 => (self.mmc5_fill_color & 3) * 0x55,
            _ => unreachable!(),
        }
    }

    fn write_nametable(&mut self, address: u16, value: u8) {
        if self.n163_enabled {
            self.write_n163_nametable(address, value);
            return;
        }
        if self.nametable_chr_map.is_some() {
            return;
        }
        if !self.mmc5_enabled {
            let index = self.mirror_nametable(address);
            self.nametable[index] = value;
            return;
        }
        let (source, inner) = self.mmc5_nametable_source(address);
        match source {
            0 | 1 => self.nametable[usize::from(source) * 0x400 + inner] = value,
            2 if self.mmc5_exram_mode <= 1 => self.mmc5_exram[inner] = value,
            _ => {}
        }
    }

    fn read_mmc5_exram_cpu(&self, address: u16) -> u8 {
        if !self.mmc5_enabled || self.mmc5_exram_mode < 2 {
            return 0xff;
        }
        self.mmc5_exram[usize::from(address - 0x5c00)]
    }

    fn write_mmc5_exram_cpu(&mut self, address: u16, value: u8) {
        if !self.mmc5_enabled || self.mmc5_exram_mode == 3 {
            return;
        }
        let rendering = self.rendering_enabled() && self.scanline < 240;
        if self.mmc5_exram_mode == 2 || rendering {
            self.mmc5_exram[usize::from(address - 0x5c00)] = value;
        }
    }

    fn mirror_nametable(&self, address: u16) -> usize {
        let offset = (address as usize - 0x2000) & 0x0fff;
        let table = offset / 0x400;
        let inner = offset & 0x3ff;
        let physical = if !matches!(self.mirroring, Mirroring::FourScreen) {
            self.ciram_page_map
                .map(|pages| usize::from(pages[table] & 1))
                .unwrap_or_else(|| match self.mirroring {
                    Mirroring::Vertical => table & 1,
                    Mirroring::Horizontal => table >> 1,
                    Mirroring::SingleScreen0 => 0,
                    Mirroring::SingleScreen1 => 1,
                    Mirroring::FourScreen => unreachable!(),
                })
        } else {
            table
        };
        physical * 0x400 + inner
    }

    fn palette_index(address: u16) -> usize {
        let mut index = (address as usize - 0x3f00) & 0x1f;
        if matches!(index, 0x10 | 0x14 | 0x18 | 0x1c) {
            index -= 0x10;
        }
        index
    }
    fn read_vram(&self, address: u16) -> u8 {
        let address = address & 0x3fff;
        match address {
            0x0000..=0x1fff if self.n163_enabled => self.read_n163_pattern(address),
            0x0000..=0x1fff => self.read_chr_mapped(address, &self.chr_map),
            0x2000..=0x3eff => self.read_nametable(if address >= 0x3000 {
                address - 0x1000
            } else {
                address
            }),
            0x3f00..=0x3fff => self.palette[Self::palette_index(address)],
            _ => unreachable!(),
        }
    }

    fn write_vram(&mut self, address: u16, value: u8) {
        let address = address & 0x3fff;
        match address {
            0x0000..=0x1fff if self.n163_enabled => self.write_n163_pattern(address, value),
            0x0000..=0x1fff => {
                let map = self.chr_map;
                self.write_chr_mapped(address, value, &map);
            }
            0x2000..=0x3eff => self.write_nametable(
                if address >= 0x3000 {
                    address - 0x1000
                } else {
                    address
                },
                value,
            ),
            0x3f00..=0x3fff => {
                let index = Self::palette_index(address);
                self.palette[index] = value & 0x3f;
            }
            _ => unreachable!(),
        }
    }
    fn read_register(&mut self, register: u16) -> u8 {
        let value = match register & 7 {
            2 => {
                let value = (self.status & 0xe0) | (self.io_latch & 0x1f);
                if self.scanline == self.region.vblank_scanline() {
                    match self.cycle {
                        0 => {
                            self.suppress_vblank = true;
                            self.nmi_pending = false;
                        }
                        1 | 2 => self.nmi_pending = false,
                        _ => {}
                    }
                }
                self.status &= !0x80;
                self.write_toggle = false;
                value
            }
            4 => self.read_oam_data(),
            7 => {
                let address = self.v & 0x3fff;
                let raw = self.read_vram(address);
                let value = if address < 0x3f00 {
                    let buffered = self.data_buffer;
                    self.data_buffer = raw;
                    buffered
                } else {
                    self.data_buffer = self.read_vram(address.wrapping_sub(0x1000));
                    let palette_value = if self.mask & 1 != 0 {
                        raw & 0x30
                    } else {
                        raw & 0x3f
                    };
                    palette_value | (self.io_latch & 0xc0)
                };
                self.v = self
                    .v
                    .wrapping_add(if self.ctrl & 0x04 != 0 { 32 } else { 1 })
                    & 0x7fff;
                value
            }
            _ => self.io_latch,
        };
        self.io_latch = value;
        value
    }
    fn write_register(&mut self, register: u16, value: u8) {
        self.io_latch = value;
        match register & 7 {
            0 => {
                let had_nmi = self.ctrl & 0x80 != 0;
                self.ctrl = value;
                self.t = (self.t & !0x0c00) | (((value as u16) & 3) << 10);
                if !had_nmi && value & 0x80 != 0 && self.status & 0x80 != 0 {
                    self.nmi_pending = true;
                }
            }
            1 => self.mask = value,
            3 => self.oam_addr = value,
            4 => self.write_oam_data(value),
            5 => {
                if !self.write_toggle {
                    self.scroll_x = value;
                    self.fine_x = value & 7;
                    self.t = (self.t & !0x001f) | ((value as u16) >> 3);
                } else {
                    self.scroll_y = value;
                    self.t = (self.t & !0x73e0)
                        | (((value as u16 & 0xf8) << 2) & 0x03e0)
                        | (((value as u16) & 7) << 12);
                }
                self.write_toggle = !self.write_toggle;
            }
            6 => {
                if !self.write_toggle {
                    self.t = (self.t & 0x00ff) | (((value as u16) & 0x3f) << 8);
                } else {
                    self.t = (self.t & 0x7f00) | value as u16;
                    self.v = self.t;
                }
                self.write_toggle = !self.write_toggle;
            }
            7 => {
                let address = self.v & 0x3fff;
                self.write_vram(address, value);
                self.v = self
                    .v
                    .wrapping_add(if self.ctrl & 0x04 != 0 { 32 } else { 1 })
                    & 0x7fff;
            }
            _ => {}
        }
    }

    fn rendering_oam_access_active(&self) -> bool {
        self.rendering_enabled()
            && (self.scanline < 240 || self.scanline == self.region.pre_render_scanline())
    }

    fn read_oam_data(&self) -> u8 {
        if self.rendering_oam_access_active() && (1..=64).contains(&self.cycle) {
            0xff
        } else {
            self.oam[self.oam_addr as usize]
        }
    }

    fn write_oam_data(&mut self, value: u8) {
        if self.rendering_oam_access_active() {
            self.oam_addr = self.oam_addr.wrapping_add(4);
            return;
        }
        self.oam[self.oam_addr as usize] = value;
        self.oam_addr = self.oam_addr.wrapping_add(1);
    }

    fn write_oam_dma_byte(&mut self, value: u8) {
        self.write_oam_data(value);
    }

    fn set_chr_map(&mut self, map: [usize; 8]) {
        self.mmc5_enabled = false;
        self.n163_enabled = false;
        self.ciram_page_map = None;
        for (slot, mapped) in map.into_iter().enumerate() {
            let bank = mapped % self.chr_bank_count;
            self.chr_map[slot] = bank;
            self.bg_chr_map[slot] = bank;
            self.sprite_chr_map[slot] = bank;
        }
    }

    fn set_ciram_page_map(&mut self, map: Option<[u8; 4]>) {
        self.ciram_page_map = map;
    }

    fn set_nametable_chr_map(&mut self, map: Option<[usize; 4]>) {
        self.nametable_chr_map = map;
    }

    fn set_namco163(&mut self, namco163: &Namco163) {
        self.mmc5_enabled = false;
        self.n163_enabled = true;
        self.ciram_page_map = None;
        self.nametable_chr_map = None;
        self.n163_pattern = namco163.chr;
        self.n163_nametable = namco163.nametable;
        self.n163_chr_disable_low = namco163.chr_disable_low;
        self.n163_chr_disable_high = namco163.chr_disable_high;
    }

    fn set_mmc5(&mut self, mmc5: &Mmc5, bg_map: [usize; 8], sprite_map: [usize; 8]) {
        self.mmc5_enabled = true;
        self.n163_enabled = false;
        self.ciram_page_map = None;
        self.nametable_chr_map = None;
        self.mmc5_exram_mode = mmc5.exram_mode;
        self.mmc5_nametable_map = mmc5.nametable_map;
        self.mmc5_fill_tile = mmc5.fill_tile;
        self.mmc5_fill_color = mmc5.fill_color;
        self.mmc5_chr_upper = mmc5.chr_upper;
        self.mmc5_split_control = mmc5.split_control;
        self.mmc5_split_scroll = mmc5.split_scroll;
        self.mmc5_split_bank = mmc5.split_bank;
        for slot in 0..8 {
            self.bg_chr_map[slot] = bg_map[slot] % self.chr_bank_count;
            self.sprite_chr_map[slot] = sprite_map[slot] % self.chr_bank_count;
            self.chr_map[slot] = if mmc5.chr_io_bg {
                self.bg_chr_map[slot]
            } else {
                self.sprite_chr_map[slot]
            };
        }
    }

    fn set_mirroring(&mut self, mirroring: Mirroring) {
        self.mirroring = mirroring;
    }

    fn take_nmi(&mut self) -> bool {
        let pending = self.nmi_pending;
        self.nmi_pending = false;
        pending
    }

    fn take_mapper_address(&mut self) -> Option<u16> {
        self.mapper_address.take()
    }

    fn rendering_enabled(&self) -> bool {
        self.mask & 0x18 != 0
    }

    fn increment_coarse_x(&mut self) {
        if self.v & 0x001f == 31 {
            self.v &= !0x001f;
            self.v ^= 0x0400;
        } else {
            self.v = self.v.wrapping_add(1);
        }
    }

    fn increment_y(&mut self) {
        if self.v & 0x7000 != 0x7000 {
            self.v = self.v.wrapping_add(0x1000);
            return;
        }
        self.v &= !0x7000;
        let mut coarse_y = (self.v & 0x03e0) >> 5;
        if coarse_y == 29 {
            coarse_y = 0;
            self.v ^= 0x0800;
        } else if coarse_y == 31 {
            coarse_y = 0;
        } else {
            coarse_y += 1;
        }
        self.v = (self.v & !0x03e0) | (coarse_y << 5);
    }

    fn copy_horizontal_scroll(&mut self) {
        self.v = (self.v & !0x041f) | (self.t & 0x041f);
    }

    fn copy_vertical_scroll(&mut self) {
        self.v = (self.v & !0x7be0) | (self.t & 0x7be0);
    }

    fn sprite_height(&self) -> u8 {
        if self.ctrl & 0x20 != 0 {
            16
        } else {
            8
        }
    }

    fn sprite_row_on_scanline(&self, sprite: usize, scanline: u16) -> Option<u8> {
        let row = (scanline as u8)
            .wrapping_sub(self.oam[sprite * 4])
            .wrapping_sub(1);
        (row < self.sprite_height()).then_some(row)
    }

    fn selected_sprites_for_scanline(&self, scanline: u16) -> Vec<(usize, u8)> {
        if scanline == 0 {
            return Vec::new();
        }
        let mut selected = Vec::with_capacity(8);
        for sprite in 0..64usize {
            if let Some(row) = self.sprite_row_on_scanline(sprite, scanline) {
                selected.push((sprite, row));
                if selected.len() == 8 {
                    break;
                }
            }
        }
        selected
    }

    fn sprite_overflow_for_scanline(&self, scanline: u16) -> bool {
        let mut found = 0usize;
        let mut n = 0usize;
        while n < 64 && found < 8 {
            if self.sprite_row_on_scanline(n, scanline).is_some() {
                found += 1;
            }
            n += 1;
        }
        if found < 8 {
            return false;
        }

        let height = self.sprite_height();
        let scanline = scanline as u8;
        let mut m = 0usize;
        while n < 64 {
            let value = self.oam[n * 4 + m];
            let row = scanline.wrapping_sub(value).wrapping_sub(1);
            if row < height {
                return true;
            }
            n += 1;
            m = (m + 1) & 3;
        }
        false
    }

    fn sprite_fetch_a12(&self, slot: usize) -> bool {
        if self.ctrl & 0x20 == 0 {
            return self.ctrl & 0x08 != 0;
        }
        if self.scanline == self.region.pre_render_scanline() {
            return true;
        }
        let target_scanline = self.scanline + 1;
        let selected = self.selected_sprites_for_scanline(target_scanline);
        selected
            .get(slot)
            .map(|(sprite, _)| self.oam[sprite * 4 + 1] & 1 != 0)
            .unwrap_or(true)
    }

    fn sprite_pattern_address(&self, slot: usize, high_plane: bool) -> Option<u16> {
        if self.scanline == self.region.pre_render_scanline() {
            return None;
        }
        let target_scanline = self.scanline + 1;
        let selected = self.selected_sprites_for_scanline(target_scanline);
        let &(sprite, row) = selected.get(slot)?;
        let base = sprite * 4;
        let tile = self.oam[base + 1];
        let attributes = self.oam[base + 2];
        let height = self.sprite_height();
        let source_y = if attributes & 0x80 != 0 {
            height - 1 - row
        } else {
            row
        };
        let (pattern_base, tile_index, pattern_row) = if height == 16 {
            let pattern_base = if tile & 1 != 0 { 0x1000 } else { 0 };
            let tile_index = (tile & 0xfe).wrapping_add(u8::from(source_y >= 8));
            (pattern_base, tile_index, u16::from(source_y & 7))
        } else {
            (
                if self.ctrl & 0x08 != 0 { 0x1000 } else { 0 },
                tile,
                u16::from(source_y),
            )
        };
        Some(
            pattern_base
                + u16::from(tile_index) * 16
                + pattern_row
                + if high_plane { 8 } else { 0 },
        )
    }

    fn mapper_sprite_address_for_dot(&self) -> Option<u16> {
        if !self.rendering_enabled() || self.scanline >= 240 || !(257..=320).contains(&self.cycle) {
            return None;
        }
        let offset = self.cycle - 257;
        let phase = offset & 7;
        if phase == 0 {
            return Some(0x2000);
        }
        if !matches!(phase, 4 | 6) {
            return None;
        }
        self.sprite_pattern_address(usize::from(offset / 8), phase == 6)
    }

    fn bus_a12_for_dot(&self) -> bool {
        if !self.rendering_enabled()
            || !(self.scanline < 240 || self.scanline == self.region.pre_render_scanline())
        {
            return self.v & 0x1000 != 0;
        }
        match self.cycle {
            1..=256 | 321..=336 if matches!(self.cycle & 7, 4..=7) => self.ctrl & 0x10 != 0,
            257..=320 if matches!((self.cycle - 256) & 7, 4..=7) => {
                self.sprite_fetch_a12(((self.cycle - 257) / 8) as usize)
            }
            _ => false,
        }
    }

    fn load_background_shifters(&mut self) {
        self.bg_pattern_low_shift =
            (self.bg_pattern_low_shift & 0xff00) | u16::from(self.bg_next_low);
        self.bg_pattern_high_shift =
            (self.bg_pattern_high_shift & 0xff00) | u16::from(self.bg_next_high);
        self.bg_attr_low_shift = (self.bg_attr_low_shift & 0xff00)
            | if self.bg_next_attr & 1 != 0 {
                0x00ff
            } else {
                0
            };
        self.bg_attr_high_shift = (self.bg_attr_high_shift & 0xff00)
            | if self.bg_next_attr & 2 != 0 {
                0x00ff
            } else {
                0
            };
    }

    fn shift_background_shifters(&mut self) {
        self.bg_pattern_low_shift <<= 1;
        self.bg_pattern_high_shift <<= 1;
        self.bg_attr_low_shift <<= 1;
        self.bg_attr_high_shift <<= 1;
    }

    fn background_tile_count(&self) -> usize {
        if (1..=256).contains(&self.cycle) {
            usize::from((self.cycle - 1) / 8)
        } else {
            32 + usize::from((self.cycle.saturating_sub(321)) / 8)
        }
    }

    fn mmc5_split_active(&self, tile_count: usize) -> bool {
        if !self.mmc5_enabled
            || self.mmc5_exram_mode >= 2
            || self.mmc5_split_control & 0x80 == 0
            || tile_count >= 32
        {
            return false;
        }
        let threshold = usize::from(self.mmc5_split_control & 0x1f);
        if self.mmc5_split_control & 0x40 != 0 {
            tile_count >= threshold
        } else {
            tile_count < threshold
        }
    }

    fn fetch_background_tile(&mut self) {
        let tile_count = self.background_tile_count();
        self.bg_next_split = self.mmc5_split_active(tile_count);
        if self.bg_next_split {
            let split_y = (usize::from(self.scanline) + usize::from(self.mmc5_split_scroll)) % 240;
            let tile_y = split_y / 8;
            let tile_x = tile_count & 31;
            self.bg_next_split_row = (split_y & 7) as u8;
            self.bg_next_tile = self.mmc5_exram[tile_y * 32 + tile_x];
            self.bg_next_ex_attr = 0;
            return;
        }

        let address = 0x2000 | (self.v & 0x0fff);
        self.bg_next_tile = self.read_vram(address);
        self.mapper_address = Some(address);
        self.bg_next_ex_attr = if self.mmc5_enabled && self.mmc5_exram_mode == 1 {
            self.mmc5_exram[usize::from(address & 0x03ff)]
        } else {
            0
        };
    }

    fn fetch_background_attribute(&mut self) {
        if self.bg_next_split {
            let tile_count = self.background_tile_count();
            let split_y = (usize::from(self.scanline) + usize::from(self.mmc5_split_scroll)) % 240;
            let tile_y = split_y / 8;
            let tile_x = tile_count & 31;
            let index = 0x3c0 + (tile_y / 4) * 8 + tile_x / 4;
            let shift = ((tile_y & 2) << 1) | (tile_x & 2);
            self.bg_next_attr = (self.mmc5_exram[index] >> shift) & 3;
            return;
        }
        if self.mmc5_enabled && self.mmc5_exram_mode == 1 {
            self.bg_next_attr = self.bg_next_ex_attr >> 6;
            return;
        }
        let address =
            0x23c0 | (self.v & 0x0c00) | ((self.v >> 4) & 0x0038) | ((self.v >> 2) & 0x0007);
        let value = self.read_vram(address);
        let shift = ((self.v >> 4) & 4) | (self.v & 2);
        self.bg_next_attr = (value >> shift) & 3;
    }

    fn clock_background_pipeline(&mut self) {
        if !self.rendering_enabled()
            || !(self.scanline < 240 || self.scanline == self.region.pre_render_scanline())
        {
            return;
        }

        if (1..=256).contains(&self.cycle) || (321..=336).contains(&self.cycle) {
            match self.cycle & 7 {
                1 => {
                    self.load_background_shifters();
                    self.fetch_background_tile();
                }
                3 => self.fetch_background_attribute(),
                5 => {
                    let fine_y = if self.bg_next_split {
                        u16::from(self.bg_next_split_row)
                    } else {
                        (self.v >> 12) & 7
                    };
                    let base = if self.ctrl & 0x10 != 0 { 0x1000 } else { 0 };
                    let address = base + u16::from(self.bg_next_tile) * 16 + fine_y;
                    self.bg_next_low = self.read_background_pattern(address);
                    self.mapper_address = Some(address);
                }
                7 => {
                    let fine_y = if self.bg_next_split {
                        u16::from(self.bg_next_split_row)
                    } else {
                        (self.v >> 12) & 7
                    };
                    let base = if self.ctrl & 0x10 != 0 { 0x1000 } else { 0 };
                    let address = base + u16::from(self.bg_next_tile) * 16 + fine_y + 8;
                    self.bg_next_high = self.read_background_pattern(address);
                    self.mapper_address = Some(address);
                }
                0 => self.increment_coarse_x(),
                _ => {}
            }
            self.shift_background_shifters();
        }

        if self.cycle == 256 {
            self.increment_y();
        }
        if self.cycle == 257 {
            self.load_background_shifters();
            self.copy_horizontal_scroll();
        }
        if self.scanline == self.region.pre_render_scanline() && (280..=304).contains(&self.cycle) {
            self.copy_vertical_scroll();
        }
        if matches!(self.cycle, 337 | 339) {
            self.bg_next_split = false;
            let address = 0x2000 | (self.v & 0x0fff);
            self.bg_next_tile = self.read_vram(address);
            self.mapper_address = Some(address);
        }
    }

    fn tick_dot(&mut self) -> bool {
        self.cycle += 1;
        self.mapper_address = self.mapper_sprite_address_for_dot();
        let a12_high = self.bus_a12_for_dot();
        let rendering = self.rendering_enabled();

        if self.scanline == self.region.vblank_scanline() && self.cycle == 1 {
            self.frame = self.frame.wrapping_add(1);
            if self.suppress_vblank {
                self.suppress_vblank = false;
                self.status &= !0x80;
                self.nmi_pending = false;
            } else {
                self.status |= 0x80;
                if self.ctrl & 0x80 != 0 {
                    self.nmi_pending = true;
                }
            }
        } else if self.scanline == self.region.pre_render_scanline() && self.cycle == 1 {
            self.status &= !0xe0;
            self.suppress_vblank = false;
        }

        if self.scanline < 240 && (1..=256).contains(&self.cycle) {
            self.render_dot(u32::from(self.cycle - 1), u32::from(self.scanline));
        }

        if rendering && self.scanline < 240 && self.cycle == 256 {
            let target_scanline = self.scanline + 1;
            if self.sprite_overflow_for_scanline(target_scanline) {
                self.status |= 0x20;
            }
        }

        self.clock_background_pipeline();

        if self.region == NesRegion::Ntsc
            && self.scanline == self.region.pre_render_scanline()
            && self.cycle == 339
            && rendering
            && self.odd_frame
        {
            self.cycle = 0;
            self.scanline = 0;
            self.odd_frame = false;
        } else if self.cycle >= 341 {
            self.cycle = 0;
            self.scanline += 1;
            if self.scanline >= self.region.total_scanlines() {
                self.scanline = 0;
                self.odd_frame = !self.odd_frame;
            }
        }
        a12_high
    }

    fn display_color(&self, index: u8) -> [u8; 4] {
        let index = if self.mask & 1 != 0 {
            index & 0x30
        } else {
            index & 0x3f
        };
        let mut rgba = nes_color(index);
        let emphasis = (self.mask >> 5) & 7;
        if emphasis == 0 {
            return rgba;
        }

        let dim = |value: u8| ((u16::from(value) * 3 + 2) / 4) as u8;
        if emphasis == 7 {
            rgba[0] = dim(rgba[0]);
            rgba[1] = dim(rgba[1]);
            rgba[2] = dim(rgba[2]);
        } else {
            if emphasis & 1 == 0 {
                rgba[0] = dim(rgba[0]);
            }
            if emphasis & 2 == 0 {
                rgba[1] = dim(rgba[1]);
            }
            if emphasis & 4 == 0 {
                rgba[2] = dim(rgba[2]);
            }
        }
        rgba
    }

    fn background_shift_pixel(&self) -> ([u8; 4], bool) {
        let bit = 0x8000u16 >> self.fine_x;
        let low = u8::from(self.bg_pattern_low_shift & bit != 0);
        let high = u8::from(self.bg_pattern_high_shift & bit != 0);
        let color = low | (high << 1);
        let palette = u8::from(self.bg_attr_low_shift & bit != 0)
            | (u8::from(self.bg_attr_high_shift & bit != 0) << 1);
        let palette_index = if color == 0 {
            0
        } else {
            u16::from(palette * 4 + color)
        };
        (
            self.display_color(self.read_vram(0x3f00 + palette_index) & 0x3f),
            color != 0,
        )
    }

    #[cfg(test)]
    fn background_pixel(&self, x: u32, y: u32) -> ([u8; 4], bool) {
        let world_x = x as usize + self.scroll_x as usize;
        let world_y = y as usize + self.scroll_y as usize;
        let base_x = (self.ctrl & 1) as usize;
        let base_y = ((self.ctrl >> 1) & 1) as usize;
        let nt_x = (base_x + world_x / 256) & 1;
        let nt_y = (base_y + world_y / 240) & 1;
        let local_x = world_x % 256;
        let local_y = world_y % 240;
        let tile_x = local_x / 8;
        let tile_y = local_y / 8;
        let table = nt_y * 2 + nt_x;
        let base = 0x2000 + (table as u16) * 0x400;
        let tile = self.read_vram(base + (tile_y * 32 + tile_x) as u16);
        let attribute = self.read_vram(base + 0x3c0 + ((tile_y / 4) * 8 + tile_x / 4) as u16);
        let quadrant = ((tile_y & 2) << 1) | (tile_x & 2);
        let palette_select = (attribute >> quadrant) & 3;
        let pattern_base = if self.ctrl & 0x10 != 0 { 0x1000 } else { 0 };
        let row = (local_y & 7) as u16;
        let pattern = pattern_base + tile as u16 * 16 + row;
        let low = self.read_chr_mapped(pattern, &self.bg_chr_map);
        let high = self.read_chr_mapped(pattern + 8, &self.bg_chr_map);
        let bit = 7 - (local_x & 7);
        let color = ((low >> bit) & 1) | (((high >> bit) & 1) << 1);
        let palette_index = if color == 0 {
            0
        } else {
            palette_select * 4 + color
        } as u16;
        (
            self.display_color(self.read_vram(0x3f00 + palette_index) & 0x3f),
            color != 0,
        )
    }

    fn sprite_pixel(&self, screen_x: u32, screen_y: u32) -> Option<(usize, u8, bool)> {
        if screen_x < 8 && self.mask & 0x04 == 0 {
            return None;
        }
        let sprite_height = self.sprite_height();
        let mut selected = 0usize;
        for sprite_index in 0..64usize {
            let Some(row) = self.sprite_row_on_scanline(sprite_index, screen_y as u16) else {
                continue;
            };
            if selected == 8 {
                break;
            }
            selected += 1;

            let base = sprite_index * 4;
            let tile = self.oam[base + 1];
            let attributes = self.oam[base + 2];
            let sprite_x = i32::from(self.oam[base + 3]);
            let pixel = screen_x as i32 - sprite_x;
            if !(0..8).contains(&pixel) {
                continue;
            }
            let source_y = if attributes & 0x80 != 0 {
                sprite_height - 1 - row
            } else {
                row
            };
            let (pattern_base, tile_index, pattern_row) = if sprite_height == 16 {
                let table = if tile & 1 != 0 { 0x1000 } else { 0 };
                let pair = tile & 0xfe;
                let which = u8::from(source_y >= 8);
                (table, pair.wrapping_add(which), u16::from(source_y & 7))
            } else {
                (
                    if self.ctrl & 0x08 != 0 { 0x1000 } else { 0 },
                    tile,
                    u16::from(source_y),
                )
            };
            let pattern = pattern_base + u16::from(tile_index) * 16 + pattern_row;
            let low = self.read_sprite_pattern(pattern);
            let high = self.read_sprite_pattern(pattern + 8);
            let pixel = pixel as u8;
            let source_x = if attributes & 0x40 != 0 {
                7 - pixel
            } else {
                pixel
            };
            let bit = 7 - source_x;
            let color = ((low >> bit) & 1) | (((high >> bit) & 1) << 1);
            if color == 0 {
                continue;
            }
            let palette = 0x3f10 + u16::from(attributes & 3) * 4 + u16::from(color);
            return Some((
                sprite_index,
                self.read_vram(palette) & 0x3f,
                attributes & 0x20 != 0,
            ));
        }
        None
    }

    fn render_dot(&mut self, screen_x: u32, screen_y: u32) {
        let background_visible = self.mask & 0x08 != 0 && (screen_x >= 8 || self.mask & 0x02 != 0);
        let (background, background_opaque) = if background_visible {
            self.background_shift_pixel()
        } else {
            let backdrop = if !self.rendering_enabled() && self.v & 0x3f00 == 0x3f00 {
                self.read_vram(self.v & 0x3fff) & 0x3f
            } else {
                self.palette[0] & 0x3f
            };
            (self.display_color(backdrop), false)
        };
        let mut rgba = background;

        if self.mask & 0x10 != 0 {
            if let Some((sprite_index, palette_index, behind_background)) =
                self.sprite_pixel(screen_x, screen_y)
            {
                if sprite_index == 0 && background_opaque && screen_x < 255 {
                    self.status |= 0x40;
                }
                if !behind_background || !background_opaque {
                    rgba = self.display_color(palette_index);
                }
            }
        }

        let offset = ((screen_y * WIDTH + screen_x) * 4) as usize;
        self.video.pixels_mut()[offset..offset + 4].copy_from_slice(&rgba);
    }

    #[cfg(test)]
    fn render_background(&mut self) {
        for y in 0..HEIGHT {
            for x in 0..WIDTH {
                if x < 8 && self.mask & 0x02 == 0 {
                    continue;
                }
                let (rgba, _) = self.background_pixel(x, y);
                let offset = ((y * WIDTH + x) * 4) as usize;
                self.video.pixels_mut()[offset..offset + 4].copy_from_slice(&rgba);
            }
        }
    }

    #[cfg(test)]
    fn render_sprites(&mut self) {
        let sprite_height = self.sprite_height();
        for screen_y in 0..HEIGHT {
            let selected = self.selected_sprites_for_scanline(screen_y as u16);
            let mut sprite_pixels = [None; WIDTH as usize];

            for (sprite_index, row) in selected {
                let base = sprite_index * 4;
                let tile = self.oam[base + 1];
                let attributes = self.oam[base + 2];
                let source_y = if attributes & 0x80 != 0 {
                    sprite_height - 1 - row
                } else {
                    row
                };
                let (pattern_base, tile_index, pattern_row) = if sprite_height == 16 {
                    let table = if tile & 1 != 0 { 0x1000 } else { 0 };
                    let pair = tile & 0xfe;
                    let which = u8::from(source_y >= 8);
                    (table, pair.wrapping_add(which), u16::from(source_y & 7))
                } else {
                    (
                        if self.ctrl & 0x08 != 0 { 0x1000 } else { 0 },
                        tile,
                        u16::from(source_y),
                    )
                };
                let pattern = pattern_base + u16::from(tile_index) * 16 + pattern_row;
                let low = self.read_sprite_pattern(pattern);
                let high = self.read_sprite_pattern(pattern + 8);
                let sprite_x = i32::from(self.oam[base + 3]);

                for pixel in 0..8u8 {
                    let screen_x = sprite_x + i32::from(pixel);
                    if !(0..WIDTH as i32).contains(&screen_x) {
                        continue;
                    }
                    if screen_x < 8 && self.mask & 0x04 == 0 {
                        continue;
                    }
                    let x = screen_x as usize;
                    if sprite_pixels[x].is_some() {
                        continue;
                    }
                    let source_x = if attributes & 0x40 != 0 {
                        7 - pixel
                    } else {
                        pixel
                    };
                    let bit = 7 - source_x;
                    let color = ((low >> bit) & 1) | (((high >> bit) & 1) << 1);
                    if color == 0 {
                        continue;
                    }
                    let palette = 0x3f10 + u16::from(attributes & 3) * 4 + u16::from(color);
                    let palette_index = self.read_vram(palette) & 0x3f;
                    sprite_pixels[x] = Some((sprite_index, palette_index, attributes & 0x20 != 0));
                }
            }

            for (screen_x, sprite) in sprite_pixels.into_iter().enumerate() {
                let Some((sprite_index, palette_index, behind_background)) = sprite else {
                    continue;
                };
                let background_visible =
                    self.mask & 0x08 != 0 && (screen_x >= 8 || self.mask & 0x02 != 0);
                let background_opaque =
                    background_visible && self.background_pixel(screen_x as u32, screen_y).1;
                if sprite_index == 0 && background_opaque && screen_x < 255 {
                    self.status |= 0x40;
                }
                if behind_background && background_opaque {
                    continue;
                }
                let rgba = self.display_color(palette_index);
                let offset = ((screen_y * WIDTH + screen_x as u32) * 4) as usize;
                self.video.pixels_mut()[offset..offset + 4].copy_from_slice(&rgba);
            }
        }
    }
}

fn nes_color(index: u8) -> [u8; 4] {
    const PALETTE: [u32; 64] = [
        0x545454, 0x001e74, 0x081090, 0x300088, 0x440064, 0x5c0030, 0x540400, 0x3c1800, 0x202a00,
        0x083a00, 0x004000, 0x003c00, 0x00323c, 0x000000, 0x000000, 0x000000, 0x989698, 0x084cc4,
        0x3032ec, 0x5c1ee4, 0x8814b0, 0xa01464, 0x982220, 0x783c00, 0x545a00, 0x287200, 0x087c00,
        0x007628, 0x006678, 0x000000, 0x000000, 0x000000, 0xeceeec, 0x4c9aec, 0x787cec, 0xb062ec,
        0xe454ec, 0xec58b4, 0xec6a64, 0xd48820, 0xa0aa00, 0x74c400, 0x4cd020, 0x38cc6c, 0x38b4cc,
        0x3c3c3c, 0x000000, 0x000000, 0xeceeec, 0xa8ccec, 0xbcbcec, 0xd4b2ec, 0xecaeec, 0xecaed4,
        0xecb4b0, 0xe4c490, 0xccd278, 0xb4de78, 0xa8e290, 0x98e2b4, 0xa0d6e4, 0xa0a2a0, 0x000000,
        0x000000,
    ];
    let rgb = PALETTE[index as usize];
    [
        ((rgb >> 16) & 0xff) as u8,
        ((rgb >> 8) & 0xff) as u8,
        (rgb & 0xff) as u8,
        255,
    ]
}

#[derive(Default)]
struct Controller {
    live: u8,
    latched: u8,
    shift: u8,
    strobe: bool,
}

impl Controller {
    fn set_buttons(&mut self, mask: u64) {
        self.live = u8::from(mask & FACE_SOUTH != 0)
            | (u8::from(mask & FACE_EAST != 0) << 1)
            | (u8::from(mask & SELECT != 0) << 2)
            | (u8::from(mask & START != 0) << 3)
            | (u8::from(mask & UP != 0) << 4)
            | (u8::from(mask & DOWN != 0) << 5)
            | (u8::from(mask & LEFT != 0) << 6)
            | (u8::from(mask & RIGHT != 0) << 7);
        if self.strobe {
            self.latch();
        }
    }

    fn latch(&mut self) {
        self.latched = self.live;
        self.shift = self.latched;
    }
    fn write_strobe(&mut self, value: u8) {
        let next = value & 1 != 0;
        if self.strobe && !next {
            self.latch();
        }
        self.strobe = next;
        if next {
            self.latch();
        }
    }

    fn read(&mut self) -> u8 {
        if self.strobe {
            return (self.live & 1) | 0x40;
        }
        let value = (self.shift & 1) | 0x40;
        self.shift = (self.shift >> 1) | 0x80;
        value
    }
}

const OEKA_ACK_ADVANCE_CYCLES: u16 = 225;
const OEKA_ACK_RELEASE_CYCLES: u16 = 154;

#[derive(Clone, Copy)]
struct OekaKidsTablet {
    x: u8,
    y: u8,
    touching: bool,
    clicked: bool,
    strobe: bool,
    advance: bool,
    report: u32,
    ack_high: bool,
    ack_target_high: bool,
    ack_cycles: u16,
}

impl Default for OekaKidsTablet {
    fn default() -> Self {
        Self {
            x: 0,
            y: 0,
            touching: false,
            clicked: false,
            strobe: false,
            advance: false,
            report: 0,
            ack_high: true,
            ack_target_high: true,
            ack_cycles: 0,
        }
    }
}

impl OekaKidsTablet {
    fn screen_axis(value: i16, maximum: i32) -> i32 {
        let unsigned = i32::from(value) - i32::from(i16::MIN);
        (unsigned * maximum + 32_767) / 65_535
    }

    fn set_input(&mut self, x_axis: i16, y_axis: i16, touching: bool, clicked: bool) {
        let screen_x = Self::screen_axis(x_axis, 255);
        let screen_y = Self::screen_axis(y_axis, 239);
        self.x = (screen_x * 240 / 256 + 8).clamp(0, 255) as u8;
        self.y = (screen_y * 256 / 240 - 12).clamp(0, 255) as u8;
        self.touching = touching || clicked;
        self.clicked = clicked;
    }

    fn latch_report(&mut self) {
        self.report = (u32::from(self.x) << 10)
            | (u32::from(self.y) << 2)
            | (u32::from(self.touching) << 1)
            | u32::from(self.clicked);
    }

    fn write(&mut self, value: u8) {
        let next_strobe = value & 1 != 0;
        let next_advance = value & 2 != 0;

        if !next_strobe {
            self.latch_report();
            self.strobe = false;
            self.advance = false;
            self.ack_high = true;
            self.ack_target_high = true;
            self.ack_cycles = 0;
            return;
        }

        if !self.strobe {
            self.strobe = true;
            self.advance = false;
            self.ack_high = true;
            self.ack_target_high = true;
            self.ack_cycles = 0;
        }

        if !self.advance && next_advance {
            self.report = (self.report << 1) & 0x7ffff;
            self.ack_target_high = false;
            self.ack_cycles = OEKA_ACK_ADVANCE_CYCLES;
        } else if self.advance && !next_advance {
            self.ack_target_high = true;
            self.ack_cycles = OEKA_ACK_RELEASE_CYCLES;
        }
        self.advance = next_advance;
    }

    fn tick(&mut self) {
        if self.ack_cycles == 0 {
            return;
        }
        self.ack_cycles -= 1;
        if self.ack_cycles == 0 {
            self.ack_high = self.ack_target_high;
        }
    }

    fn read(&self) -> u8 {
        if !self.strobe {
            return 0;
        }
        let mut value = u8::from(self.ack_high) << 2;
        if self.advance && !self.ack_high && self.report & 0x40000 == 0 {
            value |= 0x08;
        }
        value
    }

    fn reset_serial(&mut self) {
        self.strobe = false;
        self.advance = false;
        self.report = 0;
        self.ack_high = true;
        self.ack_target_high = true;
        self.ack_cycles = 0;
    }

    fn save(&self, out: &mut StateWriter) {
        out.u8(self.x);
        out.u8(self.y);
        out.u8(self.touching as u8);
        out.u8(self.clicked as u8);
        out.u8(self.strobe as u8);
        out.u8(self.advance as u8);
        out.u32(self.report);
        out.u8(self.ack_high as u8);
        out.u8(self.ack_target_high as u8);
        out.u16(self.ack_cycles);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.x = input.u8()?;
        self.y = input.u8()?;
        self.touching = input.u8()? != 0;
        self.clicked = input.u8()? != 0;
        self.strobe = input.u8()? != 0;
        self.advance = input.u8()? != 0;
        self.report = input.u32()?;
        self.ack_high = input.u8()? != 0;
        self.ack_target_high = input.u8()? != 0;
        self.ack_cycles = input.u16()?;
        if self.report > 0x7ffff
            || self.ack_cycles > OEKA_ACK_ADVANCE_CYCLES.max(OEKA_ACK_RELEASE_CYCLES)
        {
            return Err("NES Oeka Kids tablet state is invalid".into());
        }
        Ok(())
    }
}

struct NesBus {
    ram: [u8; 2048],
    cartridge: Cartridge,
    ppu: Ppu,
    controllers: [Controller; 2],
    oeka_tablet: OekaKidsTablet,
    apu: Apu,
    dma_page: Option<u8>,
}

impl NesBus {
    fn new(cartridge: Cartridge, ppu: Ppu) -> Self {
        let region = cartridge.region;
        Self {
            ram: [0; 2048],
            cartridge,
            ppu,
            controllers: [Controller::default(), Controller::default()],
            oeka_tablet: OekaKidsTablet::default(),
            apu: Apu::new(region.cpu_hz(), region.uses_pal_apu()),
            dma_page: None,
        }
    }
    fn tick_ppu(&mut self, ticks: u32) {
        for _ in 0..ticks {
            let a12_high = self.ppu.tick_dot();
            let mapper_address = self.ppu.take_mapper_address();
            self.cartridge.observe_mmc5_ppu_dot(
                self.ppu.scanline,
                self.ppu.cycle,
                self.ppu.rendering_enabled(),
            );
            self.cartridge.observe_ppu_a12(a12_high);
            if let Some(address) = mapper_address {
                if self.cartridge.observe_ppu_latch_address(address) {
                    self.cartridge.sync_ppu(&mut self.ppu);
                }
            }
        }
    }
}

impl Bus8 for NesBus {
    fn read8(&mut self, address: u16) -> u8 {
        match address {
            0x0000..=0x1fff => self.ram[(address as usize) & 0x07ff],
            0x2000..=0x3fff => {
                let register = 0x2000 | (address & 7);
                let ppu_address = (register == 0x2007).then_some(self.ppu.v & 0x3fff);
                let value = self.ppu.read_register(register);
                let value = if register == 0x2007 {
                    self.cartridge.filter_cnrom185_ppudata_read(value)
                } else {
                    value
                };
                if register == 0x2002 && value & 0x80 != 0 {
                    self.cartridge.reset_mmc5_scanline_counter();
                }
                if let Some(ppu_address) = ppu_address {
                    if self.cartridge.observe_ppu_address(ppu_address) {
                        self.cartridge.sync_ppu(&mut self.ppu);
                    }
                }
                value
            }
            0x4015 => self.apu.read_status(),
            0x4016 => self.controllers[0].read(),
            0x4017 => {
                let mut value = self.controllers[1].read();
                if self.cartridge.mapper == 96 {
                    value |= self.oeka_tablet.read();
                }
                value
            }
            0x4020..=0x5bff => self.cartridge.read_expansion_mapper(address).unwrap_or(0),
            0x5c00..=0x5fff => self.ppu.read_mmc5_exram_cpu(address),
            0x6000..=0x7fff => self.cartridge.read_ram(address),
            0x8000..=0xffff => {
                let value = self.cartridge.read_prg(address);
                self.cartridge.observe_mmc5_pcm_read(address, value);
                if matches!(address, 0xfffa | 0xfffb) {
                    self.cartridge.reset_mmc5_scanline_counter();
                }
                value
            }
            _ => 0,
        }
    }

    fn write8(&mut self, address: u16, value: u8) {
        match address {
            0x0000..=0x1fff => self.ram[(address as usize) & 0x07ff] = value,
            0x2000..=0x3fff => {
                let register = 0x2000 | (address & 7);
                let previous_mask = (register == 0x2001).then_some(self.ppu.mask);
                let completed_ppu_address = match register {
                    0x2006 if self.ppu.write_toggle => {
                        self.ppu.write_register(register, value);
                        Some(self.ppu.v & 0x3fff)
                    }
                    0x2007 => {
                        let ppu_address = self.ppu.v & 0x3fff;
                        self.ppu.write_register(register, value);
                        Some(ppu_address)
                    }
                    _ => {
                        self.ppu.write_register(register, value);
                        None
                    }
                };
                if let Some(previous_mask) = previous_mask {
                    self.cartridge
                        .observe_mmc5_ppu_mask_write(previous_mask, value);
                }
                if let Some(ppu_address) = completed_ppu_address {
                    if self.cartridge.observe_ppu_address(ppu_address) {
                        self.cartridge.sync_ppu(&mut self.ppu);
                    }
                }
            }
            0x4000..=0x4013 => self.apu.write_register(address, value),
            0x4014 => {
                self.dma_page = Some(value);
                self.cartridge.reset_mmc5_scanline_counter();
            }
            0x4015 => self.apu.write_register(address, value),
            0x4016 => {
                self.controllers[0].write_strobe(value);
                self.controllers[1].write_strobe(value);
                if self.cartridge.mapper == 96 {
                    self.oeka_tablet.write(value);
                }
            }
            0x4017 => self.apu.write_register(address, value),
            0x4020..=0x5bff => {
                if self.cartridge.write_expansion_mapper(address, value) {
                    self.cartridge.sync_ppu(&mut self.ppu);
                }
            }
            0x5c00..=0x5fff => self.ppu.write_mmc5_exram_cpu(address, value),
            0x6000..=0x7fff => {
                if self.cartridge.write_low_mapper(address, value) {
                    self.cartridge.sync_ppu(&mut self.ppu);
                } else {
                    self.cartridge.write_ram(address, value);
                }
            }
            0x8000..=0xffff => {
                self.cartridge.write_mapper(address, value);
                self.cartridge.sync_ppu(&mut self.ppu);
            }
            _ => {}
        }
    }
}

pub struct NesMachine {
    cpu: Mos6502,
    bus: NesBus,
    audio: AudioBuffer,
    powered: bool,
    ppu_phase: u8,
}

impl NesMachine {
    pub fn from_rom(rom: &[u8]) -> Result<Self, String> {
        let (cartridge, ppu) = Cartridge::parse(rom)?;
        let mut bus = NesBus::new(cartridge, ppu);
        let mut cpu = Mos6502::default();
        cpu.set_decimal_supported(false);
        cpu.reset(&mut bus);
        Ok(Self {
            cpu,
            bus,
            audio: AudioBuffer::new(48_000, 2),
            powered: true,
            ppu_phase: 0,
        })
    }

    fn tick_device_cycle(&mut self) {
        let (numerator, denominator) = self.bus.cartridge.region.ppu_ratio();
        self.ppu_phase = self.ppu_phase.wrapping_add(numerator);
        let ppu_ticks = self.ppu_phase / denominator;
        self.ppu_phase %= denominator;
        self.bus.tick_ppu(u32::from(ppu_ticks));
        self.bus.cartridge.tick_cpu_cycle();
        if self.bus.cartridge.mapper == 96 {
            self.bus.oeka_tablet.tick();
        }
        let expansion = self.bus.cartridge.expansion_audio_output();
        self.bus.apu.tick_cpu_cycle_with_expansion(expansion);
    }

    fn service_dmc_dma(&mut self, address: u16, stall_cycles: u8) {
        for _ in 0..stall_cycles {
            self.cpu.cycles = self.cpu.cycles.wrapping_add(1);
            self.tick_device_cycle();
        }
        let value = self.bus.read8(address);
        self.bus.apu.supply_dmc_byte(value);
    }

    fn tick_devices(&mut self, cpu_cycles: u32) {
        for _ in 0..cpu_cycles {
            self.tick_device_cycle();
            if let Some((address, stall_cycles)) = self.bus.apu.take_dmc_fetch_request() {
                self.service_dmc_dma(address, stall_cycles);
            }
        }
    }

    fn tick_oam_dma_cycle(&mut self) {
        self.cpu.cycles = self.cpu.cycles.wrapping_add(1);
        self.tick_device_cycle();
    }

    fn service_dmc_during_oam(&mut self) {
        let Some((address, _)) = self.bus.apu.take_dmc_fetch_request() else {
            return;
        };
        let value = self.bus.read8(address);
        self.tick_oam_dma_cycle();
        self.bus.apu.supply_dmc_byte(value);
        self.tick_oam_dma_cycle();
    }

    fn perform_oam_dma(&mut self, page: u8) {
        let base = (page as u16) << 8;
        let needs_alignment = self.cpu.cycles & 1 != 0;

        self.tick_oam_dma_cycle();
        if needs_alignment {
            self.tick_oam_dma_cycle();
        }

        for offset in 0..256u16 {
            self.service_dmc_during_oam();
            let value = self.bus.read8(base | offset);
            self.tick_oam_dma_cycle();
            self.bus.ppu.write_oam_dma_byte(value);
            self.tick_oam_dma_cycle();
        }
        self.service_dmc_during_oam();
    }

    fn clock_cpu_instruction(&mut self) -> u32 {
        let cycles = self.cpu.step(&mut self.bus);
        if cycles == 0 {
            return 0;
        }
        self.tick_devices(cycles);
        if let Some(page) = self.bus.dma_page.take() {
            self.perform_oam_dma(page);
        }
        if self.bus.ppu.take_nmi() {
            let before = self.cpu.cycles;
            self.cpu.nmi(&mut self.bus);
            let interrupt_cycles = (self.cpu.cycles - before) as u32;
            self.tick_devices(interrupt_cycles);
        }
        if self.bus.apu.irq_pending() || self.bus.cartridge.irq_pending() {
            let before = self.cpu.cycles;
            self.cpu.irq(&mut self.bus);
            let interrupt_cycles = (self.cpu.cycles - before) as u32;
            self.tick_devices(interrupt_cycles);
        }
        cycles
    }

    fn finish_audio_frame(&mut self) {
        self.bus.apu.drain_audio(&mut self.audio);
    }
}

impl Machine for NesMachine {
    fn platform(&self) -> PlatformId {
        PlatformId::Nes
    }

    fn reset(&mut self) {
        self.bus.ppu.reset();
        self.bus.apu.reset();
        self.bus.oeka_tablet.reset_serial();
        self.cpu.reset(&mut self.bus);
        self.powered = true;
        self.ppu_phase = 0;
        self.audio.begin_frame();
    }

    fn run_frame(&mut self, input: &InputState) {
        self.bus.controllers[0].set_buttons(input.buttons[0]);
        self.bus.controllers[1].set_buttons(input.buttons[1]);
        if self.bus.cartridge.mapper == 96 {
            self.bus.oeka_tablet.set_input(
                input.axes[0][AXIS_AUX_X],
                input.axes[0][AXIS_AUX_Y],
                input.buttons[0] & POINTER_TOUCH != 0,
                input.buttons[0] & POINTER_CLICK != 0,
            );
        }
        let target = self.bus.ppu.frame.wrapping_add(1);
        let region = self.bus.cartridge.region;
        let cycle_budget = (region.cpu_clock() / region.frame_rate() * 2.0).ceil() as u64;
        let cycle_deadline = self.cpu.cycles.saturating_add(cycle_budget);
        while self.bus.ppu.frame != target && self.cpu.cycles < cycle_deadline {
            if self.clock_cpu_instruction() == 0 {
                self.powered = false;
                break;
            }
        }
        self.finish_audio_frame();
    }

    fn frame_rate(&self) -> f64 {
        self.bus.cartridge.region.frame_rate()
    }
    fn video(&self) -> &VideoBuffer {
        &self.bus.ppu.video
    }
    fn audio(&self) -> &AudioBuffer {
        &self.audio
    }
    fn save_state(&self) -> Result<Vec<u8>, String> {
        let mut out = StateWriter::new(PlatformId::Nes, 22);
        out.u16(self.cpu.pc);
        out.u8(self.cpu.sp);
        out.u8(self.cpu.a);
        out.u8(self.cpu.x);
        out.u8(self.cpu.y);
        out.u8(self.cpu.p);
        out.u64(self.cpu.cycles);
        out.u8(self.cpu.stopped as u8);
        out.blob(&self.bus.ram);
        out.u16(self.bus.cartridge.mapper);
        out.u8(self.bus.cartridge.submapper);
        out.u8(self.bus.cartridge.region.state_id());
        out.u8(self.ppu_phase);
        out.u64(self.bus.cartridge.prg_bank as u64);
        save_mapper_state(&mut out, &self.bus.cartridge);
        out.blob(&self.bus.cartridge.prg_ram);
        out.u8(self.bus.ppu.chr_ram as u8);
        if self.bus.ppu.chr_ram {
            let writable_start = self.bus.ppu.chr_rom_bank_count * 0x400;
            out.blob(&self.bus.ppu.chr[writable_start..]);
        } else {
            out.blob(&[]);
        }
        out.blob(&self.bus.ppu.nametable);
        out.blob(&self.bus.ppu.mmc5_exram);
        out.blob(&self.bus.ppu.palette);
        out.blob(&self.bus.ppu.oam);
        save_ppu_registers(&mut out, &self.bus.ppu);
        for controller in &self.bus.controllers {
            save_controller(&mut out, controller);
        }
        self.bus.oeka_tablet.save(&mut out);
        out.u8(self.bus.dma_page.unwrap_or(0));
        out.u8(self.bus.dma_page.is_some() as u8);
        self.bus.apu.save(&mut out);
        out.u8(self.powered as u8);
        out.blob(self.bus.ppu.video.pixels());
        Ok(out.finish())
    }

    fn load_state(&mut self, bytes: &[u8]) -> Result<(), String> {
        let mut input = StateReader::new(bytes, PlatformId::Nes, 22)?;
        self.cpu.pc = input.u16()?;
        self.cpu.sp = input.u8()?;
        self.cpu.a = input.u8()?;
        self.cpu.x = input.u8()?;
        self.cpu.y = input.u8()?;
        self.cpu.p = input.u8()?;
        self.cpu.cycles = input.u64()?;
        self.cpu.stopped = input.u8()? != 0;
        read_exact_blob(&mut input, &mut self.bus.ram)?;
        let mapper = input.u16()?;
        if mapper != self.bus.cartridge.mapper {
            return Err("NES save state mapper differs from loaded cartridge".into());
        }
        let submapper = input.u8()?;
        if submapper != self.bus.cartridge.submapper {
            return Err("NES save state submapper differs from loaded cartridge".into());
        }
        let region = input.u8()?;
        if region != self.bus.cartridge.region.state_id() {
            return Err("NES save state timing region differs from loaded cartridge".into());
        }
        let denominator = self.bus.cartridge.region.ppu_ratio().1;
        self.ppu_phase = input.u8()? % denominator;
        self.bus.cartridge.prg_bank = input.u64()? as usize;
        load_mapper_state(&mut input, &mut self.bus.cartridge)?;
        read_vec_blob(&mut input, &mut self.bus.cartridge.prg_ram)?;
        let chr_ram = input.u8()? != 0;
        if chr_ram != self.bus.ppu.chr_ram {
            return Err("NES save state CHR memory type differs from loaded cartridge".into());
        }
        let chr = input.blob()?;
        if chr_ram {
            let writable_start = self.bus.ppu.chr_rom_bank_count * 0x400;
            let writable = &mut self.bus.ppu.chr[writable_start..];
            if chr.len() != writable.len() {
                return Err("NES CHR RAM size differs".into());
            }
            writable.copy_from_slice(chr);
        }
        self.bus.cartridge.sync_ppu(&mut self.bus.ppu);
        read_exact_blob(&mut input, &mut self.bus.ppu.nametable)?;
        read_exact_blob(&mut input, &mut self.bus.ppu.mmc5_exram)?;
        read_exact_blob(&mut input, &mut self.bus.ppu.palette)?;
        read_exact_blob(&mut input, &mut self.bus.ppu.oam)?;
        load_ppu_registers(&mut input, &mut self.bus.ppu)?;
        for controller in &mut self.bus.controllers {
            load_controller(&mut input, controller)?;
        }
        self.bus.oeka_tablet.load(&mut input)?;
        let dma = input.u8()?;
        self.bus.dma_page = if input.u8()? != 0 { Some(dma) } else { None };
        self.bus.apu.load(&mut input)?;
        self.powered = input.u8()? != 0;
        let video = input.blob()?;
        if video.len() != self.bus.ppu.video.pixels().len() {
            return Err("NES framebuffer size differs".into());
        }
        self.bus.ppu.video.pixels_mut().copy_from_slice(video);
        self.audio.begin_frame();
        input.finish()
    }
    fn persistent_len(&self, kind: ResourceKind, slot: u32) -> usize {
        if kind != ResourceKind::Storage {
            return 0;
        }
        if self.bus.cartridge.mapper == 157 {
            return match slot {
                0 => self.bus.cartridge.bandai.eeprom.data.len(),
                1 if self.bus.cartridge.bandai.external_eeprom_present => 128,
                _ => 0,
            };
        }
        if slot != 0 {
            return 0;
        }
        if matches!(self.bus.cartridge.mapper, 16 | 159) && self.bus.cartridge.bandai.eeprom_present
        {
            if self.bus.cartridge.mapper == 159 {
                128
            } else {
                self.bus.cartridge.bandai.eeprom.data.len()
            }
        } else if self.bus.cartridge.mapper == 19 && self.bus.cartridge.namco163.internal_battery {
            self.bus.cartridge.battery_len + self.bus.cartridge.namco163.ram.len()
        } else {
            self.bus.cartridge.battery_len
        }
    }
    fn read_persistent(&self, kind: ResourceKind, slot: u32, out: &mut [u8]) -> Result<(), String> {
        let len = self.persistent_len(kind, slot);
        if len == 0 {
            return Err("NES cartridge has no battery-backed storage in this slot".into());
        }
        if out.len() != len {
            return Err(format!(
                "persistent output has {} bytes; expected {len}",
                out.len()
            ));
        }
        if self.bus.cartridge.mapper == 157 {
            if slot == 0 {
                out.copy_from_slice(&self.bus.cartridge.bandai.eeprom.data);
            } else {
                out.copy_from_slice(&self.bus.cartridge.bandai.external_eeprom.data[..len]);
            }
        } else if matches!(self.bus.cartridge.mapper, 16 | 159)
            && self.bus.cartridge.bandai.eeprom_present
        {
            out.copy_from_slice(&self.bus.cartridge.bandai.eeprom.data[..len]);
        } else if self.bus.cartridge.mapper == 19 && self.bus.cartridge.namco163.internal_battery {
            let external = self.bus.cartridge.battery_len;
            out[..external].copy_from_slice(&self.bus.cartridge.prg_ram[..external]);
            out[external..].copy_from_slice(&self.bus.cartridge.namco163.ram);
        } else {
            out.copy_from_slice(&self.bus.cartridge.prg_ram[..len]);
        }
        Ok(())
    }
    fn write_persistent(
        &mut self,
        kind: ResourceKind,
        slot: u32,
        data: &[u8],
    ) -> Result<(), String> {
        let len = self.persistent_len(kind, slot);
        if len == 0 {
            return Err("NES cartridge has no battery-backed storage in this slot".into());
        }
        if data.len() != len {
            return Err(format!(
                "persistent input has {} bytes; expected {len}",
                data.len()
            ));
        }
        if self.bus.cartridge.mapper == 157 {
            if slot == 0 {
                self.bus.cartridge.bandai.eeprom.data.copy_from_slice(data);
            } else {
                self.bus.cartridge.bandai.external_eeprom.data[..len].copy_from_slice(data);
            }
        } else if matches!(self.bus.cartridge.mapper, 16 | 159)
            && self.bus.cartridge.bandai.eeprom_present
        {
            self.bus.cartridge.bandai.eeprom.data[..len].copy_from_slice(data);
        } else if self.bus.cartridge.mapper == 19 && self.bus.cartridge.namco163.internal_battery {
            let external = self.bus.cartridge.battery_len;
            self.bus.cartridge.prg_ram[..external].copy_from_slice(&data[..external]);
            self.bus
                .cartridge
                .namco163
                .ram
                .copy_from_slice(&data[external..]);
        } else {
            self.bus.cartridge.prg_ram[..len].copy_from_slice(data);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn synthetic_nrom() -> Vec<u8> {
        let mut rom = vec![0u8; 16 + 16 * 1024 + 8 * 1024];
        rom[..4].copy_from_slice(b"NES\x1a");
        rom[4] = 1;
        rom[5] = 1;
        let prg = 16;
        let program = [
            0x78, 0xd8, 0xa2, 0xff, 0x9a, 0xa9, 0x3f, 0x8d, 0x06, 0x20, 0xa9, 0x00, 0x8d, 0x06,
            0x20, 0xa9, 0x0f, 0x8d, 0x07, 0x20, 0xa9, 0x30, 0x8d, 0x07, 0x20, 0xa9, 0x80, 0x8d,
            0x00, 0x20, 0xa9, 0x08, 0x8d, 0x01, 0x20, 0x4c, 0x23, 0x80,
        ];
        rom[prg..prg + program.len()].copy_from_slice(&program);
        let vectors = prg + 0x3ffa;
        rom[vectors..vectors + 6].copy_from_slice(&[0x00, 0x80, 0x00, 0x80, 0x00, 0x80]);
        let chr = prg + 16 * 1024;
        rom[chr..chr + 8].fill(0xff);
        rom
    }

    #[test]
    fn nrom_machine_executes_cpu_and_renders_a_frame() {
        let rom = synthetic_nrom();
        let mut machine = NesMachine::from_rom(&rom).unwrap();
        machine.run_frame(&InputState::default());
        assert!(machine.powered);
        assert!(machine.cpu.cycles > NesRegion::Ntsc.cpu_hz() / 100);
        assert_eq!(machine.video().width(), WIDTH);
        assert_eq!(machine.video().height(), HEIGHT);
        assert!(machine
            .video()
            .pixels()
            .as_chunks::<4>()
            .0
            .iter()
            .any(|pixel| pixel[0] > 100));
    }

    #[test]
    fn cartridge_header_selects_ntsc_pal_and_dendy_timing() {
        let mut pal = synthetic_nrom();
        pal[7] = 0x08;
        pal[12] = 1;
        let (pal_cart, pal_ppu) = Cartridge::parse(&pal).unwrap();
        assert_eq!(pal_cart.region, NesRegion::Pal);
        assert_eq!(pal_ppu.region, NesRegion::Pal);

        let mut dendy = synthetic_nrom();
        dendy[7] = 0x08;
        dendy[12] = 3;
        let (dendy_cart, dendy_ppu) = Cartridge::parse(&dendy).unwrap();
        assert_eq!(dendy_cart.region, NesRegion::Dendy);
        assert_eq!(dendy_ppu.region, NesRegion::Dendy);

        let mut ines_pal = synthetic_nrom();
        ines_pal[9] = 1;
        let (ines_cart, _) = Cartridge::parse(&ines_pal).unwrap();
        assert_eq!(ines_cart.region, NesRegion::Pal);
    }

    #[test]
    fn pal_ppu_uses_312_scanlines_without_odd_frame_dot_skip() {
        let mut ppu = Ppu::new(vec![0; 0x2000], false, Mirroring::Horizontal);
        ppu.region = NesRegion::Pal;
        ppu.mask = 0x18;
        ppu.odd_frame = true;
        ppu.scanline = 311;
        ppu.cycle = 338;
        ppu.tick_dot();
        assert_eq!((ppu.scanline, ppu.cycle), (311, 339));
        ppu.cycle = 340;
        ppu.tick_dot();
        assert_eq!((ppu.scanline, ppu.cycle), (0, 0));
    }

    #[test]
    fn dendy_delays_vblank_until_scanline_291() {
        let mut ppu = Ppu::new(vec![0; 0x2000], false, Mirroring::Horizontal);
        ppu.region = NesRegion::Dendy;
        ppu.ctrl = 0x80;
        ppu.scanline = 241;
        ppu.cycle = 0;
        ppu.tick_dot();
        assert_eq!(ppu.status & 0x80, 0);
        assert!(!ppu.nmi_pending);

        ppu.scanline = 291;
        ppu.cycle = 0;
        ppu.tick_dot();
        assert_ne!(ppu.status & 0x80, 0);
        assert!(ppu.nmi_pending);
        assert_eq!(ppu.frame, 1);
    }

    #[test]
    fn pal_machine_schedules_sixteen_ppu_dots_per_five_cpu_cycles() {
        let mut rom = synthetic_nrom();
        rom[7] = 0x08;
        rom[12] = 1;
        let mut machine = NesMachine::from_rom(&rom).unwrap();
        machine.bus.ppu.scanline = 0;
        machine.bus.ppu.cycle = 0;
        machine.ppu_phase = 0;
        for _ in 0..5 {
            machine.tick_device_cycle();
        }
        assert_eq!(machine.bus.ppu.cycle, 16);
        assert_eq!(machine.ppu_phase, 0);
        assert_eq!(machine.bus.cartridge.region, NesRegion::Pal);
    }

    #[test]
    fn pal_save_state_preserves_fractional_ppu_phase() {
        let mut rom = synthetic_nrom();
        rom[7] = 0x08;
        rom[12] = 1;
        let mut machine = NesMachine::from_rom(&rom).unwrap();
        machine.tick_device_cycle();
        assert_eq!(machine.ppu_phase, 1);
        let state = machine.save_state().unwrap();

        let mut restored = NesMachine::from_rom(&rom).unwrap();
        restored.load_state(&state).unwrap();
        assert_eq!(restored.ppu_phase, 1);
        assert_eq!(restored.bus.cartridge.region, NesRegion::Pal);
    }

    #[test]
    fn visible_pixels_are_committed_at_ppu_dot_time() {
        let mut ppu = Ppu::new(vec![0; 0x2000], false, Mirroring::Horizontal);
        ppu.palette[0] = 1;
        ppu.tick_dot();
        let first = ppu.video.pixels()[..4].to_vec();
        assert_eq!(first.as_slice(), &nes_color(1));

        ppu.palette[0] = 2;
        ppu.tick_dot();
        assert_eq!(&ppu.video.pixels()[4..8], &nes_color(2));
        assert_eq!(&ppu.video.pixels()[..4], first.as_slice());

        ppu.palette[0] = 3;
        ppu.scanline = 241;
        ppu.cycle = 0;
        ppu.tick_dot();
        assert_eq!(&ppu.video.pixels()[..4], first.as_slice());
    }

    #[test]
    fn ppumask_grayscale_and_ntsc_emphasis_modify_palette_output() {
        let mut ppu = Ppu::new(vec![0; 0x2000], false, Mirroring::Horizontal);
        ppu.palette[0] = 0x2a;
        ppu.mask = 0x01;
        assert_eq!(ppu.display_color(0x2a), nes_color(0x20));
        ppu.v = 0x3f00;
        ppu.io_latch = 0xc0;
        assert_eq!(ppu.read_register(0x2007), 0xe0);
        assert_eq!(ppu.palette[0], 0x2a);

        let base = nes_color(0x30);
        let dim = |value: u8| ((u16::from(value) * 3 + 2) / 4) as u8;
        ppu.mask = 0x20;
        let red = ppu.display_color(0x30);
        assert_eq!(red[0], base[0]);
        assert_eq!(red[1], dim(base[1]));
        assert_eq!(red[2], dim(base[2]));

        ppu.mask = 0xe0;
        let all = ppu.display_color(0x30);
        assert_eq!(all[0], dim(base[0]));
        assert_eq!(all[1], dim(base[1]));
        assert_eq!(all[2], dim(base[2]));
    }

    #[test]
    fn rendering_disabled_palette_address_overrides_backdrop() {
        let mut ppu = Ppu::new(vec![0; 0x2000], false, Mirroring::Horizontal);
        ppu.palette[0] = 0x01;
        ppu.palette[5] = 0x30;
        ppu.mask = 0;
        ppu.v = 0x3f05;
        ppu.render_dot(0, 0);
        assert_eq!(&ppu.video.pixels()[..4], &nes_color(0x30));

        ppu.v = 0x2000;
        ppu.render_dot(1, 0);
        assert_eq!(&ppu.video.pixels()[4..8], &nes_color(0x01));
    }

    #[test]
    fn background_fetch_pipeline_prefetches_visible_pixels() {
        let mut chr = vec![0u8; 0x2000];
        chr[16] = 0x80;
        let mut ppu = Ppu::new(chr, false, Mirroring::Horizontal);
        ppu.mask = 0x0a;
        ppu.nametable[0] = 1;
        ppu.palette[1] = 0x30;
        ppu.scanline = 261;
        ppu.cycle = 320;

        for _ in 0..21 {
            ppu.tick_dot();
        }
        assert_eq!(ppu.scanline, 0);
        assert_eq!(ppu.cycle, 0);

        ppu.tick_dot();
        assert_eq!(&ppu.video.pixels()[..4], &nes_color(0x30));
    }

    #[test]
    fn sprite_zero_hit_is_raised_on_the_overlap_dot() {
        let mut chr = vec![0u8; 0x2000];
        chr[..8].fill(0xff);
        let mut ppu = Ppu::new(chr, false, Mirroring::Horizontal);
        ppu.mask = 0x1e;
        ppu.oam.fill(0xf0);
        ppu.oam[0] = 0;
        ppu.oam[1] = 0;
        ppu.oam[2] = 0;
        ppu.oam[3] = 10;
        ppu.bg_pattern_low_shift = 0xffff;
        ppu.scanline = 1;
        ppu.cycle = 9;

        ppu.tick_dot();
        assert_eq!(ppu.status & 0x40, 0);
        ppu.tick_dot();
        assert_ne!(ppu.status & 0x40, 0);
    }

    #[test]
    fn oam_dma_uses_halt_alignment_and_alternating_transfer_cycles() {
        let rom = synthetic_nrom();
        let mut machine = NesMachine::from_rom(&rom).unwrap();
        for offset in 0..256usize {
            machine.bus.ram[0x200 + offset] = offset as u8;
        }
        let before = machine.cpu.cycles;
        let expected = 513 + (before & 1);
        machine.perform_oam_dma(0x02);
        assert_eq!(machine.cpu.cycles - before, expected);
        assert_eq!(&machine.bus.ppu.oam[..], &machine.bus.ram[0x200..0x300]);
        assert_eq!(machine.bus.ppu.oam_addr, 0);
    }

    #[test]
    fn ppu_io_latch_drives_open_bus_status_and_palette_read_bits() {
        let mut ppu = Ppu::new(vec![0; 0x2000], false, Mirroring::Horizontal);
        ppu.write_register(0x2001, 0x1b);
        assert_eq!(ppu.read_register(0x2000), 0x1b);

        ppu.status = 0xa0;
        ppu.io_latch = 0x1d;
        assert_eq!(ppu.read_register(0x2002), 0xbd);
        assert_eq!(ppu.status & 0x80, 0);
        assert_eq!(ppu.io_latch, 0xbd);

        ppu.mask = 0;
        ppu.palette[0] = 0x2a;
        ppu.v = 0x3f00;
        ppu.io_latch = 0xc0;
        assert_eq!(ppu.read_register(0x2007), 0xea);
        assert_eq!(ppu.io_latch, 0xea);
    }

    #[test]
    fn ppustatus_read_suppresses_vblank_and_nmi_at_boundary() {
        let mut ppu = Ppu::new(vec![0; 0x2000], false, Mirroring::Horizontal);
        ppu.ctrl = 0x80;
        ppu.scanline = 241;
        ppu.cycle = 0;
        assert_eq!(ppu.read_register(0x2002) & 0x80, 0);
        assert!(ppu.suppress_vblank);
        ppu.tick_dot();
        assert_eq!(ppu.cycle, 1);
        assert_eq!(ppu.frame, 1);
        assert_eq!(ppu.status & 0x80, 0);
        assert!(!ppu.nmi_pending);

        let mut same_dot = Ppu::new(vec![0; 0x2000], false, Mirroring::Horizontal);
        same_dot.ctrl = 0x80;
        same_dot.scanline = 241;
        same_dot.cycle = 0;
        same_dot.tick_dot();
        assert!(same_dot.nmi_pending);
        assert_ne!(same_dot.read_register(0x2002) & 0x80, 0);
        assert!(!same_dot.nmi_pending);

        let mut next_dot = Ppu::new(vec![0; 0x2000], false, Mirroring::Horizontal);
        next_dot.ctrl = 0x80;
        next_dot.scanline = 241;
        next_dot.cycle = 0;
        next_dot.tick_dot();
        next_dot.tick_dot();
        assert_eq!(next_dot.cycle, 2);
        assert_ne!(next_dot.read_register(0x2002) & 0x80, 0);
        assert!(!next_dot.nmi_pending);
    }

    #[test]
    fn oamdata_rendering_writes_are_suppressed_and_step_sprite_index() {
        let mut ppu = Ppu::new(vec![0; 0x2000], false, Mirroring::Horizontal);
        ppu.mask = 0x18;
        ppu.scanline = 20;
        ppu.cycle = 100;
        ppu.oam_addr = 5;
        ppu.oam[5] = 0x11;

        ppu.write_register(0x2004, 0xaa);
        assert_eq!(ppu.oam[5], 0x11);
        assert_eq!(ppu.oam_addr, 9);
        ppu.write_oam_dma_byte(0xbb);
        assert_eq!(ppu.oam[9], 0);
        assert_eq!(ppu.oam_addr, 13);

        ppu.scanline = 241;
        ppu.write_register(0x2004, 0xcc);
        assert_eq!(ppu.oam[13], 0xcc);
        assert_eq!(ppu.oam_addr, 14);
    }

    #[test]
    fn oamdata_reads_return_ff_during_secondary_oam_clear() {
        let mut ppu = Ppu::new(vec![0; 0x2000], false, Mirroring::Horizontal);
        ppu.mask = 0x18;
        ppu.scanline = 10;
        ppu.cycle = 32;
        ppu.oam_addr = 12;
        ppu.oam[12] = 0x55;
        assert_eq!(ppu.read_register(0x2004), 0xff);

        ppu.scanline = 241;
        assert_eq!(ppu.read_register(0x2004), 0x55);
    }

    #[test]
    fn dmc_dma_overlaps_oam_dma_instead_of_charging_full_stall() {
        let rom = synthetic_nrom();
        let mut machine = NesMachine::from_rom(&rom).unwrap();
        for offset in 0..256usize {
            machine.bus.ram[0x200 + offset] = (offset as u8).wrapping_mul(3);
        }
        machine.bus.apu.write_register(0x4012, 0x00);
        machine.bus.apu.write_register(0x4013, 0x00);
        machine.bus.apu.write_register(0x4015, 0x10);
        machine.bus.apu.tick_cpu_cycles(2);

        let before = machine.cpu.cycles;
        let base_cycles = 513 + (before & 1);
        machine.perform_oam_dma(0x02);
        assert_eq!(machine.cpu.cycles - before, base_cycles + 2);
        assert_eq!(&machine.bus.ppu.oam[..], &machine.bus.ram[0x200..0x300]);
    }

    #[test]
    fn save_state_round_trip_is_deterministic() {
        let rom = synthetic_nrom();
        let mut first = NesMachine::from_rom(&rom).unwrap();
        first.run_frame(&InputState::default());
        first.bus.ram[17] = 0x5a;
        first.bus.ppu.odd_frame = true;
        first.bus.ppu.io_latch = 0x6d;
        first.bus.ppu.suppress_vblank = true;
        first.bus.cartridge.mmc3.a12_high = true;
        first.bus.cartridge.mmc3.a12_low_cycles = 7;
        first.bus.cartridge.mmc3.mc_acc_fall_counter = 5;
        let state = first.save_state().unwrap();
        let mut second = NesMachine::from_rom(&rom).unwrap();
        second.load_state(&state).unwrap();
        assert_eq!(second.save_state().unwrap(), state);
        first.run_frame(&InputState::default());
        second.run_frame(&InputState::default());
        assert_eq!(first.cpu.cycles, second.cpu.cycles);
        assert_eq!(first.video().pixels(), second.video().pixels());
        assert_eq!(first.save_state().unwrap(), second.save_state().unwrap());
    }

    #[test]
    fn sprite_evaluation_keeps_first_eight_and_sets_overflow() {
        let mut ppu = Ppu::new(vec![0; 0x2000], false, Mirroring::Horizontal);
        ppu.mask = 0x10;
        ppu.oam.fill(0xf0);
        for sprite in 0..9usize {
            let base = sprite * 4;
            ppu.oam[base] = 9;
            ppu.oam[base + 3] = (sprite * 8) as u8;
        }
        let selected = ppu.selected_sprites_for_scanline(10);
        assert_eq!(selected.len(), 8);
        assert_eq!(selected.first().map(|entry| entry.0), Some(0));
        assert_eq!(selected.last().map(|entry| entry.0), Some(7));
        assert!(ppu.sprite_overflow_for_scanline(10));

        ppu.scanline = 9;
        ppu.cycle = 255;
        ppu.tick_dot();
        assert_ne!(ppu.status & 0x20, 0);
    }

    #[test]
    fn lower_oam_sprite_priority_blocks_later_sprite_behind_background() {
        let mut chr = vec![0u8; 0x2000];
        chr[0..8].fill(0xff);
        let mut ppu = Ppu::new(chr, false, Mirroring::Horizontal);
        ppu.mask = 0x1e;
        ppu.palette[1] = 0x30;
        ppu.palette[0x11] = 0x16;
        ppu.palette[0x15] = 0x2a;
        ppu.oam.fill(0xf0);
        ppu.oam[4] = 0;
        ppu.oam[5] = 0;
        ppu.oam[6] = 0x20;
        ppu.oam[7] = 16;
        ppu.oam[8] = 0;
        ppu.oam[9] = 0;
        ppu.oam[10] = 0x01;
        ppu.oam[11] = 16;

        ppu.render_background();
        let offset = ((WIDTH + 16) * 4) as usize;
        let background = ppu.video.pixels()[offset..offset + 4].to_vec();
        ppu.render_sprites();
        assert_eq!(
            &ppu.video.pixels()[offset..offset + 4],
            background.as_slice()
        );
    }
}

fn save_bandai_eeprom_state(out: &mut StateWriter, eeprom: &Eeprom24c02, data_len: usize) {
    out.blob(&eeprom.data[..data_len]);
    out.u8(eeprom.mode);
    out.u8(eeprom.next_mode);
    out.u8(eeprom.shift);
    out.u8(eeprom.address);
    out.u8(eeprom.bits);
    out.u8(eeprom.output as u8);
    out.u8(eeprom.prev_scl as u8);
    out.u8(eeprom.prev_sda as u8);
    out.u8(eeprom.ack_clocked as u8);
    out.u8(eeprom.master_ack as u8);
}

fn load_bandai_eeprom_state(
    input: &mut StateReader<'_>,
    eeprom: &mut Eeprom24c02,
    data_len: usize,
    label: &str,
) -> Result<(), String> {
    let data = input.blob()?;
    if data.len() != data_len || data_len > eeprom.data.len() {
        return Err(format!("NES {label} EEPROM state has wrong size"));
    }
    eeprom.data.fill(0xff);
    eeprom.data[..data_len].copy_from_slice(data);
    eeprom.mode = input.u8()?;
    eeprom.next_mode = input.u8()?;
    if eeprom.mode > Eeprom24c02::WAIT_MASTER_ACK || eeprom.next_mode > Eeprom24c02::WAIT_MASTER_ACK
    {
        return Err(format!("NES {label} EEPROM protocol state is invalid"));
    }
    eeprom.shift = input.u8()?;
    eeprom.address = input.u8()?;
    eeprom.bits = input.u8()?.min(8);
    eeprom.output = input.u8()? != 0;
    eeprom.prev_scl = input.u8()? != 0;
    eeprom.prev_sda = input.u8()? != 0;
    eeprom.ack_clocked = input.u8()? != 0;
    eeprom.master_ack = input.u8()? != 0;
    Ok(())
}

fn save_mapper_state(out: &mut StateWriter, cart: &Cartridge) {
    out.u8(cart.simple_reg);
    if cart.mapper == 86 {
        out.u8(cart.jaleco_d7756_control);
    }
    if cart.mapper == 96 {
        out.u8(cart.oeka_inner_chr);
        out.u8(cart.oeka_last_dd);
    }
    out.blob(&cart.nina_chr);
    if cart.mapper == 185 {
        out.u8(cart.cnrom185_reads_remaining);
    }
    out.u8(cart.mmc1.shift);
    out.u8(cart.mmc1.bits);
    out.u8(cart.mmc1.control);
    out.u8(cart.mmc1.chr0);
    out.u8(cart.mmc1.chr1);
    out.u8(cart.mmc1.prg);
    out.blob(&cart.mmc2.chr_fd);
    out.blob(&cart.mmc2.chr_fe);
    out.u8(cart.mmc2.latch_fe[0] as u8);
    out.u8(cart.mmc2.latch_fe[1] as u8);
    out.u8(cart.mmc2.mirror_horizontal as u8);
    out.u8(cart.mmc3.bank_select);
    out.blob(&cart.mmc3.regs);
    out.u8(cart.mmc3.mirror_horizontal as u8);
    out.u8(cart.mmc3.prg_ram_protect);
    out.u8(cart.mmc3.irq_latch);
    out.u8(cart.mmc3.irq_counter);
    out.u8(cart.mmc3.irq_reload as u8);
    out.u8(cart.mmc3.irq_enabled as u8);
    out.u8(cart.mmc3.irq_pending as u8);
    out.u8(cart.mmc3.a12_high as u8);
    out.u16(cart.mmc3.a12_low_cycles);
    out.u8(cart.mmc3.mc_acc_fall_counter);
    out.u8(cart.rambo1.bank_select);
    out.blob(&cart.rambo1.regs);
    out.u8(cart.rambo1.mirror_horizontal as u8);
    out.u8(cart.rambo1.irq_latch);
    out.u8(cart.rambo1.irq_counter);
    out.u8(cart.rambo1.irq_reload as u8);
    out.u8(cart.rambo1.irq_cycle_mode as u8);
    out.u8(cart.rambo1.irq_enabled as u8);
    out.u8(cart.rambo1.irq_pending as u8);
    out.u8(cart.rambo1.irq_prescaler);
    out.u8(cart.rambo1.irq_delay);
    out.u8(cart.rambo1.a12_high as u8);
    out.u16(cart.rambo1.a12_low_cycles);
    out.blob(&cart.bandai.chr);
    out.u8(cart.bandai.prg);
    out.u8(cart.bandai.mirroring);
    out.u16(cart.bandai.irq_latch);
    out.u16(cart.bandai.irq_counter);
    out.u8(cart.bandai.irq_enabled as u8);
    out.u8(cart.bandai.irq_pending as u8);
    save_bandai_eeprom_state(out, &cart.bandai.eeprom, cart.bandai.eeprom.data.len());
    if cart.mapper == 157 {
        out.u8(cart.bandai.external_scl as u8);
        out.u8(cart.bandai.barcode_line as u8);
        if cart.bandai.external_eeprom_present {
            save_bandai_eeprom_state(out, &cart.bandai.external_eeprom, 128);
        }
    }
    out.blob(&cart.namco163.chr);
    out.blob(&cart.namco163.nametable);
    out.blob(&cart.namco163.prg);
    out.u8(cart.namco163.chr_disable_low as u8);
    out.u8(cart.namco163.chr_disable_high as u8);
    out.u8(cart.namco163.sound_disabled as u8);
    out.blob(&cart.namco163.ram);
    out.u8(cart.namco163.ram_address);
    out.u8(cart.namco163.ram_autoincrement as u8);
    out.u8(cart.namco163.wram_protect);
    out.u16(cart.namco163.irq_counter);
    out.u8(cart.namco163.irq_enabled as u8);
    out.u8(cart.namco163.irq_pending as u8);
    out.u8(cart.namco163.audio_divider);
    out.u8(cart.namco163.audio_channel);
    out.u32(cart.namco163.audio_output.to_bits());
    out.blob(&cart.namco210.chr);
    out.blob(&cart.namco210.prg);
    out.u8(cart.namco210.ram_enabled as u8);
    out.u8(cart.namco210.mirroring);
    out.u8(cart.mmc5.prg_mode);
    out.u8(cart.mmc5.chr_mode);
    out.blob(&cart.mmc5.prg_protect);
    out.u8(cart.mmc5.exram_mode);
    out.u8(cart.mmc5.nametable_map);
    out.u8(cart.mmc5.fill_tile);
    out.u8(cart.mmc5.fill_color);
    out.u8(cart.mmc5.prg_ram_bank);
    out.blob(&cart.mmc5.prg);
    for bank in cart.mmc5.chr_sprite {
        out.u16(bank);
    }
    for bank in cart.mmc5.chr_bg {
        out.u16(bank);
    }
    out.u8(cart.mmc5.chr_upper);
    out.u8(cart.mmc5.chr_io_bg as u8);
    out.u8(cart.mmc5.split_control);
    out.u8(cart.mmc5.split_scroll);
    out.u8(cart.mmc5.split_bank);
    out.u8(cart.mmc5.irq_compare);
    out.u8(cart.mmc5.irq_counter);
    out.u8(cart.mmc5.irq_enabled as u8);
    out.u8(cart.mmc5.irq_pending as u8);
    out.u8(cart.mmc5.in_frame as u8);
    out.u8(cart.mmc5.multiplicand);
    out.u8(cart.mmc5.multiplier);
    for pulse in &cart.mmc5.pulses {
        out.u8(pulse.control);
        out.u16(pulse.period);
        out.u16(pulse.timer);
        out.u8(pulse.step);
        out.u8(pulse.length);
        out.u8(pulse.envelope_start as u8);
        out.u8(pulse.envelope_divider);
        out.u8(pulse.envelope_decay);
    }
    out.u8(cart.mmc5.pulse_enable);
    out.u16(cart.mmc5.audio_divider);
    out.u8(cart.mmc5.pcm_mode as u8);
    out.u8(cart.mmc5.pcm_irq_enabled as u8);
    out.u8(cart.mmc5.pcm_irq_pending as u8);
    out.u8(cart.mmc5.pcm);
    out.blob(&cart.vrc1.prg);
    out.blob(&cart.vrc1.chr);
    out.u8(cart.vrc1.mirror_horizontal as u8);
    out.u16(cart.vrc3.irq_latch);
    out.u16(cart.vrc3.irq_counter);
    out.u8(cart.vrc3.irq_enable_after_ack as u8);
    out.u8(cart.vrc3.irq_enabled as u8);
    out.u8(cart.vrc3.irq_mode_8bit as u8);
    out.u8(cart.vrc3.irq_pending as u8);
    if cart.mapper == 65 {
        out.blob(&cart.irem_h3001.prg);
        out.blob(&cart.irem_h3001.chr);
        out.u8(cart.irem_h3001.swap_prg as u8);
        out.u8(cart.irem_h3001.mirroring);
        out.u16(cart.irem_h3001.irq_reload);
        out.u16(cart.irem_h3001.irq_counter);
        out.u8(cart.irem_h3001.irq_enabled as u8);
        out.u8(cart.irem_h3001.irq_pending as u8);
    }
    if cart.mapper == 67 {
        out.blob(&cart.sunsoft3.chr_2k);
        out.u8(cart.sunsoft3.prg);
        out.u8(cart.sunsoft3.mirroring);
        out.u16(cart.sunsoft3.irq_counter);
        out.u8(cart.sunsoft3.irq_low_next as u8);
        out.u8(cart.sunsoft3.irq_enabled as u8);
        out.u8(cart.sunsoft3.irq_pending as u8);
    }
    out.blob(&cart.irem_g101.prg);
    out.blob(&cart.irem_g101.chr);
    out.u8(cart.irem_g101.swap_prg as u8);
    out.u8(cart.irem_g101.mirror_horizontal as u8);
    out.blob(&cart.taito_tc0190.prg);
    out.blob(&cart.taito_tc0190.chr_2k);
    out.blob(&cart.taito_tc0190.chr_1k);
    out.u8(cart.taito_tc0190.mirror_horizontal as u8);
    if cart.mapper == 48 {
        out.u8(cart.taito_tc0190.irq_latch);
        out.u8(cart.taito_tc0190.irq_counter);
        out.u8(cart.taito_tc0190.irq_reload as u8);
        out.u8(cart.taito_tc0190.irq_enabled as u8);
        out.u8(cart.taito_tc0190.irq_pending as u8);
        out.u8(cart.taito_tc0190.irq_delay);
        out.u8(cart.taito_tc0190.a12_high as u8);
        out.u16(cart.taito_tc0190.a12_low_cycles);
    }
    if cart.mapper == 68 {
        out.blob(&cart.sunsoft4.chr_2k);
        out.blob(&cart.sunsoft4.nametable_chr);
        out.u8(cart.sunsoft4.nametable_control);
        out.u8(cart.sunsoft4.prg_control);
    }
    if matches!(cart.mapper, 80 | 207) {
        out.blob(&cart.taito_x1005.prg);
        out.blob(&cart.taito_x1005.chr);
        out.u8(cart.taito_x1005.mirror_horizontal as u8);
        out.u8(cart.taito_x1005.ram_enabled as u8);
    }
    if matches!(cart.mapper, 82 | 552) {
        out.blob(&cart.taito_x1017.prg);
        out.blob(&cart.taito_x1017.chr);
        out.u8(cart.taito_x1017.chr_inverted as u8);
        out.u8(cart.taito_x1017.mirror_vertical as u8);
        for enabled in cart.taito_x1017.ram_enabled {
            out.u8(enabled as u8);
        }
        out.u8(cart.taito_x1017.irq_latch);
        out.u16(cart.taito_x1017.irq_counter);
        out.u8(cart.taito_x1017.irq_control);
        out.u8(cart.taito_x1017.irq_pending as u8);
    }
    if matches!(cart.mapper, 72 | 92) {
        out.u8(cart.jaleco_jf17.prg);
        out.u8(cart.jaleco_jf17.chr);
        out.u8(cart.jaleco_jf17.control);
    }
    out.blob(&cart.jaleco_ss88006.prg);
    out.blob(&cart.jaleco_ss88006.chr);
    out.u8(cart.jaleco_ss88006.ram_control);
    out.u16(cart.jaleco_ss88006.irq_reload);
    out.u16(cart.jaleco_ss88006.irq_counter);
    out.u8(cart.jaleco_ss88006.irq_control);
    out.u8(cart.jaleco_ss88006.irq_pending as u8);
    out.u8(cart.jaleco_ss88006.mirroring);
    out.u8(cart.jaleco_ss88006.sound_control);
    out.blob(&cart.vrc4.prg);
    for bank in cart.vrc4.chr {
        out.u16(bank);
    }
    out.u8(cart.vrc4.mirroring);
    out.u8(cart.vrc4.swap_mode as u8);
    out.u8(cart.vrc4.wram_enable as u8);
    out.u8(cart.vrc4.latch_6000);
    out.u8(cart.vrc4.irq_latch);
    out.u8(cart.vrc4.irq_counter);
    out.u16(cart.vrc4.irq_prescaler as u16);
    out.u8(cart.vrc4.irq_cycle_mode as u8);
    out.u8(cart.vrc4.irq_enabled as u8);
    out.u8(cart.vrc4.irq_enable_after_ack as u8);
    out.u8(cart.vrc4.irq_pending as u8);
    out.u8(cart.vrc6.prg16);
    out.u8(cart.vrc6.prg8);
    out.blob(&cart.vrc6.chr);
    out.u8(cart.vrc6.ppu_control);
    for pulse in &cart.vrc6.pulses {
        out.u8(pulse.volume);
        out.u8(pulse.duty);
        out.u8(pulse.mode as u8);
        out.u16(pulse.period);
        out.u16(pulse.timer);
        out.u8(pulse.step);
        out.u8(pulse.enabled as u8);
    }
    out.u8(cart.vrc6.saw.rate);
    out.u16(cart.vrc6.saw.period);
    out.u16(cart.vrc6.saw.timer);
    out.u8(cart.vrc6.saw.step);
    out.u8(cart.vrc6.saw.accumulator);
    out.u8(cart.vrc6.saw.enabled as u8);
    out.u8(cart.vrc6.frequency_control);
    out.u8(cart.vrc6.irq_latch);
    out.u8(cart.vrc6.irq_counter);
    out.u16(cart.vrc6.irq_prescaler as u16);
    out.u8(cart.vrc6.irq_cycle_mode as u8);
    out.u8(cart.vrc6.irq_enabled as u8);
    out.u8(cart.vrc6.irq_enable_after_ack as u8);
    out.u8(cart.vrc6.irq_pending as u8);
    out.blob(&cart.vrc7.prg);
    out.blob(&cart.vrc7.chr);
    out.u8(cart.vrc7.control);
    out.u8(cart.vrc7.irq_latch);
    out.u8(cart.vrc7.irq_counter);
    out.u16(cart.vrc7.irq_prescaler as u16);
    out.u8(cart.vrc7.irq_cycle_mode as u8);
    out.u8(cart.vrc7.irq_enabled as u8);
    out.u8(cart.vrc7.irq_enable_after_ack as u8);
    out.u8(cart.vrc7.irq_pending as u8);
    out.u8(cart.vrc7.selected_audio_register);
    out.blob(&cart.vrc7.audio_registers);
    for channel in &cart.vrc7.channels {
        out.u32(channel.mod_phase.to_bits());
        out.u32(channel.carrier_phase.to_bits());
        out.u32(channel.envelope.to_bits());
        out.u32(channel.feedback.to_bits());
        out.u8(channel.key_on as u8);
        out.u8(channel.attacking as u8);
    }
    out.u8(cart.vrc7.audio_divider);
    out.u32(cart.vrc7.lfo_phase.to_bits());
    out.u32(cart.vrc7.audio_output.to_bits());
    out.u8(cart.fme7.command);
    out.blob(&cart.fme7.chr);
    out.blob(&cart.fme7.prg);
    out.u8(cart.fme7.bank_6000);
    out.u8(cart.fme7.mirroring);
    out.u16(cart.fme7.irq_counter);
    out.u8(cart.fme7.irq_counter_enabled as u8);
    out.u8(cart.fme7.irq_output_enabled as u8);
    out.u8(cart.fme7.irq_pending as u8);
    out.u8(cart.fme7.audio.selected);
    out.u8(cart.fme7.audio.writes_disabled as u8);
    out.blob(&cart.fme7.audio.regs);
    out.u8(cart.fme7.audio.divider);
    for counter in cart.fme7.audio.tone_counter {
        out.u16(counter);
    }
    for high in cart.fme7.audio.tone_high {
        out.u8(high as u8);
    }
    out.u8(cart.fme7.audio.noise_counter);
    out.u32(cart.fme7.audio.noise_lfsr);
    out.u16(cart.fme7.audio.envelope_counter);
    out.u8(cart.fme7.audio.envelope_step);
    out.u8(cart.fme7.audio.envelope_attack_mask);
    out.u8(cart.fme7.audio.envelope_hold as u8);
    out.u8(cart.fme7.audio.envelope_alternate as u8);
    out.u8(cart.fme7.audio.envelope_holding as u8);
}

fn load_mapper_state(input: &mut StateReader<'_>, cart: &mut Cartridge) -> Result<(), String> {
    cart.simple_reg = input.u8()?;
    if cart.mapper == 86 {
        cart.jaleco_d7756_control = input.u8()? & 0x3f;
    }
    if cart.mapper == 96 {
        cart.oeka_inner_chr = input.u8()? & 3;
        cart.oeka_last_dd = input.u8()? & 3;
    }
    let nina_chr = input.blob()?;
    if nina_chr.len() != 2 {
        return Err("NES NINA-001 CHR register state has wrong size".into());
    }
    cart.nina_chr.copy_from_slice(nina_chr);
    if cart.mapper == 185 {
        cart.cnrom185_reads_remaining = input.u8()?.min(2);
    }
    cart.mmc1.shift = input.u8()?;
    cart.mmc1.bits = input.u8()?;
    cart.mmc1.control = input.u8()?;
    cart.mmc1.chr0 = input.u8()?;
    cart.mmc1.chr1 = input.u8()?;
    cart.mmc1.prg = input.u8()?;
    let mmc2_fd = input.blob()?;
    if mmc2_fd.len() != 2 {
        return Err("NES MMC2 FD CHR register state has wrong size".into());
    }
    cart.mmc2.chr_fd.copy_from_slice(mmc2_fd);
    let mmc2_fe = input.blob()?;
    if mmc2_fe.len() != 2 {
        return Err("NES MMC2 FE CHR register state has wrong size".into());
    }
    cart.mmc2.chr_fe.copy_from_slice(mmc2_fe);
    cart.mmc2.latch_fe[0] = input.u8()? != 0;
    cart.mmc2.latch_fe[1] = input.u8()? != 0;
    cart.mmc2.mirror_horizontal = input.u8()? != 0;
    cart.mmc3.bank_select = input.u8()?;
    let regs = input.blob()?;
    if regs.len() != 8 {
        return Err("NES MMC3 register state has wrong size".into());
    }
    cart.mmc3.regs.copy_from_slice(regs);
    cart.mmc3.mirror_horizontal = input.u8()? != 0;
    cart.mmc3.prg_ram_protect = input.u8()?;
    cart.mmc3.irq_latch = input.u8()?;
    cart.mmc3.irq_counter = input.u8()?;
    cart.mmc3.irq_reload = input.u8()? != 0;
    cart.mmc3.irq_enabled = input.u8()? != 0;
    cart.mmc3.irq_pending = input.u8()? != 0;
    cart.mmc3.a12_high = input.u8()? != 0;
    cart.mmc3.a12_low_cycles = input.u16()?;
    cart.mmc3.mc_acc_fall_counter = input.u8()? % 8;
    cart.rambo1.bank_select = input.u8()?;
    read_exact_blob(input, &mut cart.rambo1.regs)?;
    cart.rambo1.mirror_horizontal = input.u8()? != 0;
    cart.rambo1.irq_latch = input.u8()?;
    cart.rambo1.irq_counter = input.u8()?;
    cart.rambo1.irq_reload = input.u8()? != 0;
    cart.rambo1.irq_cycle_mode = input.u8()? != 0;
    cart.rambo1.irq_enabled = input.u8()? != 0;
    cart.rambo1.irq_pending = input.u8()? != 0;
    cart.rambo1.irq_prescaler = input.u8()? % 4;
    cart.rambo1.irq_delay = input.u8()?.min(2);
    cart.rambo1.a12_high = input.u8()? != 0;
    cart.rambo1.a12_low_cycles = input.u16()?;
    read_exact_blob(input, &mut cart.bandai.chr)?;
    cart.bandai.prg = input.u8()? & 0x0f;
    cart.bandai.mirroring = input.u8()? & 3;
    cart.bandai.irq_latch = input.u16()?;
    cart.bandai.irq_counter = input.u16()?;
    cart.bandai.irq_enabled = input.u8()? != 0;
    cart.bandai.irq_pending = input.u8()? != 0;
    load_bandai_eeprom_state(input, &mut cart.bandai.eeprom, 256, "Bandai internal")?;
    if cart.mapper == 157 {
        cart.bandai.external_scl = input.u8()? != 0;
        cart.bandai.barcode_line = input.u8()? != 0;
        if cart.bandai.external_eeprom_present {
            load_bandai_eeprom_state(
                input,
                &mut cart.bandai.external_eeprom,
                128,
                "Datach external",
            )?;
        }
    }
    read_exact_blob(input, &mut cart.namco163.chr)?;
    read_exact_blob(input, &mut cart.namco163.nametable)?;
    read_exact_blob(input, &mut cart.namco163.prg)?;
    for bank in &mut cart.namco163.prg {
        *bank &= 0x3f;
    }
    cart.namco163.chr_disable_low = input.u8()? != 0;
    cart.namco163.chr_disable_high = input.u8()? != 0;
    cart.namco163.sound_disabled = input.u8()? != 0;
    read_exact_blob(input, &mut cart.namco163.ram)?;
    cart.namco163.ram_address = input.u8()? & 0x7f;
    cart.namco163.ram_autoincrement = input.u8()? != 0;
    cart.namco163.wram_protect = input.u8()?;
    cart.namco163.irq_counter = input.u16()? & 0x7fff;
    cart.namco163.irq_enabled = input.u8()? != 0;
    cart.namco163.irq_pending = input.u8()? != 0;
    cart.namco163.audio_divider = input.u8()?.min(14);
    cart.namco163.audio_channel = input.u8()? & 7;
    cart.namco163.audio_output = f32::from_bits(input.u32()?);
    read_exact_blob(input, &mut cart.namco210.chr)?;
    read_exact_blob(input, &mut cart.namco210.prg)?;
    for bank in &mut cart.namco210.prg {
        *bank &= 0x3f;
    }
    cart.namco210.ram_enabled = input.u8()? != 0;
    cart.namco210.mirroring = input.u8()? & 3;
    cart.mmc5.prg_mode = input.u8()? & 3;
    cart.mmc5.chr_mode = input.u8()? & 3;
    read_exact_blob(input, &mut cart.mmc5.prg_protect)?;
    cart.mmc5.prg_protect[0] &= 3;
    cart.mmc5.prg_protect[1] &= 3;
    cart.mmc5.exram_mode = input.u8()? & 3;
    cart.mmc5.nametable_map = input.u8()?;
    cart.mmc5.fill_tile = input.u8()?;
    cart.mmc5.fill_color = input.u8()? & 3;
    cart.mmc5.prg_ram_bank = input.u8()? & 0x0f;
    read_exact_blob(input, &mut cart.mmc5.prg)?;
    for bank in &mut cart.mmc5.chr_sprite {
        *bank = input.u16()? & 0x03ff;
    }
    for bank in &mut cart.mmc5.chr_bg {
        *bank = input.u16()? & 0x03ff;
    }
    cart.mmc5.chr_upper = input.u8()? & 3;
    cart.mmc5.chr_io_bg = input.u8()? != 0;
    cart.mmc5.split_control = input.u8()? & 0xdf;
    cart.mmc5.split_scroll = input.u8()?;
    cart.mmc5.split_bank = input.u8()?;
    cart.mmc5.irq_compare = input.u8()?;
    cart.mmc5.irq_counter = input.u8()?;
    cart.mmc5.irq_enabled = input.u8()? != 0;
    cart.mmc5.irq_pending = input.u8()? != 0;
    cart.mmc5.in_frame = input.u8()? != 0;
    cart.mmc5.multiplicand = input.u8()?;
    cart.mmc5.multiplier = input.u8()?;
    for pulse in &mut cart.mmc5.pulses {
        pulse.control = input.u8()?;
        pulse.period = input.u16()? & 0x07ff;
        pulse.timer = input.u16()?.max(1);
        pulse.step = input.u8()? & 7;
        pulse.length = input.u8()?;
        pulse.envelope_start = input.u8()? != 0;
        pulse.envelope_divider = input.u8()? & 0x0f;
        pulse.envelope_decay = input.u8()? & 0x0f;
    }
    cart.mmc5.pulse_enable = input.u8()? & 3;
    cart.mmc5.audio_divider = input.u16()? % 7424;
    cart.mmc5.pcm_mode = input.u8()? != 0;
    cart.mmc5.pcm_irq_enabled = input.u8()? != 0;
    cart.mmc5.pcm_irq_pending = input.u8()? != 0;
    cart.mmc5.pcm = input.u8()?;
    let vrc1_prg = input.blob()?;
    if vrc1_prg.len() != 3 {
        return Err("NES VRC1 PRG register state has wrong size".into());
    }
    cart.vrc1.prg.copy_from_slice(vrc1_prg);
    let vrc1_chr = input.blob()?;
    if vrc1_chr.len() != 2 {
        return Err("NES VRC1 CHR register state has wrong size".into());
    }
    cart.vrc1.chr.copy_from_slice(vrc1_chr);
    cart.vrc1.mirror_horizontal = input.u8()? != 0;
    cart.vrc3.irq_latch = input.u16()?;
    cart.vrc3.irq_counter = input.u16()?;
    cart.vrc3.irq_enable_after_ack = input.u8()? != 0;
    cart.vrc3.irq_enabled = input.u8()? != 0;
    cart.vrc3.irq_mode_8bit = input.u8()? != 0;
    cart.vrc3.irq_pending = input.u8()? != 0;
    if cart.mapper == 65 {
        read_exact_blob(input, &mut cart.irem_h3001.prg)?;
        read_exact_blob(input, &mut cart.irem_h3001.chr)?;
        cart.irem_h3001.swap_prg = input.u8()? != 0;
        cart.irem_h3001.mirroring = input.u8()? & 3;
        cart.irem_h3001.irq_reload = input.u16()?;
        cart.irem_h3001.irq_counter = input.u16()?;
        cart.irem_h3001.irq_enabled = input.u8()? != 0;
        cart.irem_h3001.irq_pending = input.u8()? != 0;
    }
    if cart.mapper == 67 {
        read_exact_blob(input, &mut cart.sunsoft3.chr_2k)?;
        cart.sunsoft3.prg = input.u8()? & 0x0f;
        cart.sunsoft3.mirroring = input.u8()? & 3;
        cart.sunsoft3.irq_counter = input.u16()?;
        cart.sunsoft3.irq_low_next = input.u8()? != 0;
        cart.sunsoft3.irq_enabled = input.u8()? != 0;
        cart.sunsoft3.irq_pending = input.u8()? != 0;
    }
    let irem_prg = input.blob()?;
    if irem_prg.len() != 2 {
        return Err("NES Irem G-101 PRG register state has wrong size".into());
    }
    cart.irem_g101.prg.copy_from_slice(irem_prg);
    let irem_chr = input.blob()?;
    if irem_chr.len() != 8 {
        return Err("NES Irem G-101 CHR register state has wrong size".into());
    }
    cart.irem_g101.chr.copy_from_slice(irem_chr);
    cart.irem_g101.swap_prg = input.u8()? != 0;
    cart.irem_g101.mirror_horizontal = input.u8()? != 0;
    let taito_prg = input.blob()?;
    if taito_prg.len() != 2 {
        return Err("NES Taito TC0190 PRG register state has wrong size".into());
    }
    cart.taito_tc0190.prg.copy_from_slice(taito_prg);
    let taito_chr_2k = input.blob()?;
    if taito_chr_2k.len() != 2 {
        return Err("NES Taito TC0190 2 KiB CHR register state has wrong size".into());
    }
    cart.taito_tc0190.chr_2k.copy_from_slice(taito_chr_2k);
    let taito_chr_1k = input.blob()?;
    if taito_chr_1k.len() != 4 {
        return Err("NES Taito TC0190 1 KiB CHR register state has wrong size".into());
    }
    cart.taito_tc0190.chr_1k.copy_from_slice(taito_chr_1k);
    cart.taito_tc0190.mirror_horizontal = input.u8()? != 0;
    if cart.mapper == 48 {
        cart.taito_tc0190.irq_latch = input.u8()?;
        cart.taito_tc0190.irq_counter = input.u8()?;
        cart.taito_tc0190.irq_reload = input.u8()? != 0;
        cart.taito_tc0190.irq_enabled = input.u8()? != 0;
        cart.taito_tc0190.irq_pending = input.u8()? != 0;
        cart.taito_tc0190.irq_delay = input.u8()?.min(4);
        cart.taito_tc0190.a12_high = input.u8()? != 0;
        cart.taito_tc0190.a12_low_cycles = input.u16()?;
    }
    if cart.mapper == 68 {
        read_exact_blob(input, &mut cart.sunsoft4.chr_2k)?;
        read_exact_blob(input, &mut cart.sunsoft4.nametable_chr)?;
        for bank in &mut cart.sunsoft4.nametable_chr {
            *bank &= 0x7f;
        }
        cart.sunsoft4.nametable_control = input.u8()? & 0x13;
        cart.sunsoft4.prg_control = input.u8()? & 0x1f;
    }
    if matches!(cart.mapper, 80 | 207) {
        read_exact_blob(input, &mut cart.taito_x1005.prg)?;
        read_exact_blob(input, &mut cart.taito_x1005.chr)?;
        cart.taito_x1005.mirror_horizontal = input.u8()? != 0;
        cart.taito_x1005.ram_enabled = input.u8()? != 0;
    }
    if matches!(cart.mapper, 82 | 552) {
        read_exact_blob(input, &mut cart.taito_x1017.prg)?;
        read_exact_blob(input, &mut cart.taito_x1017.chr)?;
        cart.taito_x1017.chr_inverted = input.u8()? != 0;
        cart.taito_x1017.mirror_vertical = input.u8()? != 0;
        for enabled in &mut cart.taito_x1017.ram_enabled {
            *enabled = input.u8()? != 0;
        }
        cart.taito_x1017.irq_latch = input.u8()?;
        cart.taito_x1017.irq_counter = input.u16()?;
        cart.taito_x1017.irq_control = input.u8()? & 7;
        cart.taito_x1017.irq_pending = input.u8()? != 0;
    }
    if matches!(cart.mapper, 72 | 92) {
        cart.jaleco_jf17.prg = input.u8()? & if cart.mapper == 72 { 7 } else { 0x0f };
        cart.jaleco_jf17.chr = input.u8()? & 0x0f;
        cart.jaleco_jf17.control = input.u8()? & 0xf0;
    }
    let jaleco_prg = input.blob()?;
    if jaleco_prg.len() != 3 {
        return Err("NES Jaleco SS88006 PRG register state has wrong size".into());
    }
    cart.jaleco_ss88006.prg.copy_from_slice(jaleco_prg);
    let jaleco_chr = input.blob()?;
    if jaleco_chr.len() != 8 {
        return Err("NES Jaleco SS88006 CHR register state has wrong size".into());
    }
    cart.jaleco_ss88006.chr.copy_from_slice(jaleco_chr);
    cart.jaleco_ss88006.ram_control = input.u8()? & 3;
    cart.jaleco_ss88006.irq_reload = input.u16()?;
    cart.jaleco_ss88006.irq_counter = input.u16()?;
    cart.jaleco_ss88006.irq_control = input.u8()? & 0x0f;
    cart.jaleco_ss88006.irq_pending = input.u8()? != 0;
    cart.jaleco_ss88006.mirroring = input.u8()? & 3;
    cart.jaleco_ss88006.sound_control = input.u8()?;
    let vrc_prg = input.blob()?;
    if vrc_prg.len() != 2 {
        return Err("NES VRC PRG register state has wrong size".into());
    }
    cart.vrc4.prg.copy_from_slice(vrc_prg);
    for bank in &mut cart.vrc4.chr {
        *bank = input.u16()? & 0x01ff;
    }
    cart.vrc4.mirroring = input.u8()? & 3;
    cart.vrc4.swap_mode = input.u8()? != 0;
    cart.vrc4.wram_enable = input.u8()? != 0;
    cart.vrc4.latch_6000 = input.u8()? & 1;
    cart.vrc4.irq_latch = input.u8()?;
    cart.vrc4.irq_counter = input.u8()?;
    cart.vrc4.irq_prescaler = (input.u16()? % 342).max(1) as i16;
    cart.vrc4.irq_cycle_mode = input.u8()? != 0;
    cart.vrc4.irq_enabled = input.u8()? != 0;
    cart.vrc4.irq_enable_after_ack = input.u8()? != 0;
    cart.vrc4.irq_pending = input.u8()? != 0;
    cart.vrc6.prg16 = input.u8()? & 0x0f;
    cart.vrc6.prg8 = input.u8()? & 0x1f;
    let vrc6_chr = input.blob()?;
    if vrc6_chr.len() != 8 {
        return Err("NES VRC6 CHR register state has wrong size".into());
    }
    cart.vrc6.chr.copy_from_slice(vrc6_chr);
    cart.vrc6.ppu_control = input.u8()?;
    for pulse in &mut cart.vrc6.pulses {
        pulse.volume = input.u8()? & 0x0f;
        pulse.duty = input.u8()? & 7;
        pulse.mode = input.u8()? != 0;
        pulse.period = input.u16()? & 0x0fff;
        pulse.timer = input.u16()? & 0x0fff;
        pulse.step = input.u8()? & 0x0f;
        pulse.enabled = input.u8()? != 0;
    }
    cart.vrc6.saw.rate = input.u8()? & 0x3f;
    cart.vrc6.saw.period = input.u16()? & 0x0fff;
    cart.vrc6.saw.timer = input.u16()? & 0x0fff;
    cart.vrc6.saw.step = input.u8()? % 14;
    cart.vrc6.saw.accumulator = input.u8()?;
    cart.vrc6.saw.enabled = input.u8()? != 0;
    cart.vrc6.frequency_control = input.u8()? & 7;
    cart.vrc6.irq_latch = input.u8()?;
    cart.vrc6.irq_counter = input.u8()?;
    cart.vrc6.irq_prescaler = (input.u16()? % 342).max(1) as i16;
    cart.vrc6.irq_cycle_mode = input.u8()? != 0;
    cart.vrc6.irq_enabled = input.u8()? != 0;
    cart.vrc6.irq_enable_after_ack = input.u8()? != 0;
    cart.vrc6.irq_pending = input.u8()? != 0;
    read_exact_blob(input, &mut cart.vrc7.prg)?;
    for bank in &mut cart.vrc7.prg {
        *bank &= 0x3f;
    }
    read_exact_blob(input, &mut cart.vrc7.chr)?;
    cart.vrc7.control = input.u8()?;
    cart.vrc7.irq_latch = input.u8()?;
    cart.vrc7.irq_counter = input.u8()?;
    cart.vrc7.irq_prescaler = (input.u16()? % 342).max(1) as i16;
    cart.vrc7.irq_cycle_mode = input.u8()? != 0;
    cart.vrc7.irq_enabled = input.u8()? != 0;
    cart.vrc7.irq_enable_after_ack = input.u8()? != 0;
    cart.vrc7.irq_pending = input.u8()? != 0;
    cart.vrc7.selected_audio_register = input.u8()?;
    read_exact_blob(input, &mut cart.vrc7.audio_registers)?;
    for channel in &mut cart.vrc7.channels {
        channel.mod_phase = f32::from_bits(input.u32()?);
        channel.carrier_phase = f32::from_bits(input.u32()?);
        channel.envelope = f32::from_bits(input.u32()?);
        channel.feedback = f32::from_bits(input.u32()?);
        channel.key_on = input.u8()? != 0;
        channel.attacking = input.u8()? != 0;
    }
    cart.vrc7.audio_divider = input.u8()? % 36;
    cart.vrc7.lfo_phase = f32::from_bits(input.u32()?);
    cart.vrc7.audio_output = f32::from_bits(input.u32()?);
    cart.fme7.command = input.u8()? & 0x0f;
    let fme7_chr = input.blob()?;
    if fme7_chr.len() != 8 {
        return Err("NES FME-7 CHR register state has wrong size".into());
    }
    cart.fme7.chr.copy_from_slice(fme7_chr);
    let fme7_prg = input.blob()?;
    if fme7_prg.len() != 3 {
        return Err("NES FME-7 PRG register state has wrong size".into());
    }
    cart.fme7.prg.copy_from_slice(fme7_prg);
    cart.fme7.bank_6000 = input.u8()?;
    cart.fme7.mirroring = input.u8()? & 3;
    cart.fme7.irq_counter = input.u16()?;
    cart.fme7.irq_counter_enabled = input.u8()? != 0;
    cart.fme7.irq_output_enabled = input.u8()? != 0;
    cart.fme7.irq_pending = input.u8()? != 0;
    cart.fme7.audio.selected = input.u8()? & 0x0f;
    cart.fme7.audio.writes_disabled = input.u8()? != 0;
    let audio_regs = input.blob()?;
    if audio_regs.len() != 16 {
        return Err("NES Sunsoft 5B audio register state has wrong size".into());
    }
    cart.fme7.audio.regs.copy_from_slice(audio_regs);
    cart.fme7.audio.divider = input.u8()? & 0x1f;
    for counter in &mut cart.fme7.audio.tone_counter {
        *counter = input.u16()?;
    }
    for high in &mut cart.fme7.audio.tone_high {
        *high = input.u8()? != 0;
    }
    cart.fme7.audio.noise_counter = input.u8()?;
    cart.fme7.audio.noise_lfsr = input.u32()? & 0x1ffff;
    cart.fme7.audio.envelope_counter = input.u16()?;
    cart.fme7.audio.envelope_step = input.u8()? & 0x1f;
    cart.fme7.audio.envelope_attack_mask = input.u8()? & 0x1f;
    cart.fme7.audio.envelope_hold = input.u8()? != 0;
    cart.fme7.audio.envelope_alternate = input.u8()? != 0;
    cart.fme7.audio.envelope_holding = input.u8()? != 0;
    Ok(())
}

fn read_exact_blob<const N: usize>(
    input: &mut StateReader<'_>,
    target: &mut [u8; N],
) -> Result<(), String> {
    let blob = input.blob()?;
    if blob.len() != N {
        return Err(format!(
            "save-state blob has {} bytes; expected {N}",
            blob.len()
        ));
    }
    target.copy_from_slice(blob);
    Ok(())
}
fn read_vec_blob(input: &mut StateReader<'_>, target: &mut [u8]) -> Result<(), String> {
    let blob = input.blob()?;
    if blob.len() != target.len() {
        return Err(format!(
            "save-state blob has {} bytes; expected {}",
            blob.len(),
            target.len()
        ));
    }
    target.copy_from_slice(blob);
    Ok(())
}
fn save_controller(out: &mut StateWriter, controller: &Controller) {
    out.u8(controller.live);
    out.u8(controller.latched);
    out.u8(controller.shift);
    out.u8(controller.strobe as u8);
}
fn load_controller(input: &mut StateReader<'_>, controller: &mut Controller) -> Result<(), String> {
    controller.live = input.u8()?;
    controller.latched = input.u8()?;
    controller.shift = input.u8()?;
    controller.strobe = input.u8()? != 0;
    Ok(())
}

fn save_ppu_registers(out: &mut StateWriter, ppu: &Ppu) {
    out.u8(ppu.ctrl);
    out.u8(ppu.mask);
    out.u8(ppu.status);
    out.u8(ppu.oam_addr);
    out.u16(ppu.v);
    out.u16(ppu.t);
    out.u8(ppu.fine_x);
    out.u8(ppu.write_toggle as u8);
    out.u8(ppu.data_buffer);
    out.u8(ppu.io_latch);
    out.u8(ppu.scroll_x);
    out.u8(ppu.scroll_y);
    out.u16(ppu.bg_pattern_low_shift);
    out.u16(ppu.bg_pattern_high_shift);
    out.u16(ppu.bg_attr_low_shift);
    out.u16(ppu.bg_attr_high_shift);
    out.u8(ppu.bg_next_tile);
    out.u8(ppu.bg_next_attr);
    out.u8(ppu.bg_next_ex_attr);
    out.u8(ppu.bg_next_split as u8);
    out.u8(ppu.bg_next_split_row);
    out.u8(ppu.bg_next_low);
    out.u8(ppu.bg_next_high);
    out.u16(ppu.cycle);
    out.u16(ppu.scanline);
    out.u64(ppu.frame);
    out.u8(ppu.odd_frame as u8);
    out.u8(ppu.nmi_pending as u8);
    out.u8(ppu.suppress_vblank as u8);
}
fn load_ppu_registers(input: &mut StateReader<'_>, ppu: &mut Ppu) -> Result<(), String> {
    ppu.ctrl = input.u8()?;
    ppu.mask = input.u8()?;
    ppu.status = input.u8()?;
    ppu.oam_addr = input.u8()?;
    ppu.v = input.u16()?;
    ppu.t = input.u16()?;
    ppu.fine_x = input.u8()?;
    ppu.write_toggle = input.u8()? != 0;
    ppu.data_buffer = input.u8()?;
    ppu.io_latch = input.u8()?;
    ppu.scroll_x = input.u8()?;
    ppu.scroll_y = input.u8()?;
    ppu.bg_pattern_low_shift = input.u16()?;
    ppu.bg_pattern_high_shift = input.u16()?;
    ppu.bg_attr_low_shift = input.u16()?;
    ppu.bg_attr_high_shift = input.u16()?;
    ppu.bg_next_tile = input.u8()?;
    ppu.bg_next_attr = input.u8()? & 3;
    ppu.bg_next_ex_attr = input.u8()?;
    ppu.bg_next_split = input.u8()? != 0;
    ppu.bg_next_split_row = input.u8()? & 7;
    ppu.bg_next_low = input.u8()?;
    ppu.bg_next_high = input.u8()?;
    ppu.cycle = input.u16()?;
    ppu.scanline = input.u16()?;
    ppu.frame = input.u64()?;
    ppu.odd_frame = input.u8()? != 0;
    ppu.nmi_pending = input.u8()? != 0;
    ppu.suppress_vblank = input.u8()? != 0;
    ppu.mapper_address = None;
    Ok(())
}

#[cfg(test)]
mod mapper_tests {
    use super::*;

    fn mapper_rom(mapper: u16, prg_banks: u8, chr_banks: u8) -> Vec<u8> {
        let mut rom = vec![0u8; 16 + prg_banks as usize * 0x4000 + chr_banks as usize * 0x2000];
        rom[..4].copy_from_slice(b"NES\x1a");
        rom[4] = prg_banks;
        rom[5] = chr_banks;
        rom[6] = ((mapper & 0x0f) as u8) << 4;
        rom[7] = (mapper & 0xf0) as u8;
        rom
    }

    fn nes2_mapper_rom(mapper: u16, submapper: u8, prg_banks: u8, chr_banks: u8) -> Vec<u8> {
        let mut rom = mapper_rom(mapper, prg_banks, chr_banks);
        rom[7] = (rom[7] & 0xf0) | 0x08;
        rom[8] = (submapper << 4) | ((mapper >> 8) as u8 & 0x0f);
        rom
    }

    fn mapper4_nes2_rom(submapper: u8, prg_banks: u8, chr_banks: u8) -> Vec<u8> {
        nes2_mapper_rom(4, submapper, prg_banks, chr_banks)
    }

    #[test]
    fn uxrom_switches_lower_prg_and_fixes_last_bank() {
        let mut rom = mapper_rom(2, 4, 0);
        for bank in 0..4usize {
            rom[16 + bank * 0x4000] = bank as u8;
        }
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();
        assert_eq!(cart.read_prg(0x8000), 0);
        assert_eq!(cart.read_prg(0xc000), 3);
        cart.write_mapper(0x8000, 2);
        assert_eq!(cart.read_prg(0x8000), 2);
        assert_eq!(cart.read_prg(0xc000), 3);
    }
    #[test]
    fn cnrom_switches_chr_banks_inside_the_same_machine() {
        let mut rom = mapper_rom(3, 2, 2);
        let chr_start = 16 + 2 * 0x4000;
        rom[chr_start] = 0x11;
        rom[chr_start + 0x2000] = 0x77;
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        assert_eq!(ppu.read_vram(0), 0x11);
        cart.write_mapper(0x8000, 1);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.read_vram(0), 0x77);
    }

    #[test]
    fn cnrom_submapper_two_ands_writes_with_prg_while_one_does_not() {
        for (submapper, prg_byte, expected_latch) in [(1, 0x00, 3usize), (2, 0x01, 1usize)] {
            let mut rom = nes2_mapper_rom(3, submapper, 2, 2);
            rom[16] = prg_byte;
            let chr_start = 16 + 2 * 0x4000;
            rom[chr_start] = 0x11;
            rom[chr_start + 0x2000] = 0x77;
            let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
            cart.write_mapper(0x8000, 3);
            cart.sync_ppu(&mut ppu);
            assert_eq!(cart.prg_bank, expected_latch);
            assert_eq!(ppu.read_vram(0), 0x77);
        }
    }

    #[test]
    fn mapper185_known_submappers_gate_chr_by_latch_code() {
        for submapper in 4u8..=7 {
            let mut rom = nes2_mapper_rom(185, submapper, 2, 1);
            rom[16] = 0xff;
            let chr_start = 16 + 2 * 0x4000;
            rom[chr_start..chr_start + 0x2000].fill(0x5a);
            let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
            let enable_code = submapper - 4;

            cart.write_mapper(0x8000, enable_code);
            cart.sync_ppu(&mut ppu);
            assert_eq!(ppu.read_vram(0), 0x5a);

            cart.write_mapper(0x8000, enable_code ^ 1);
            cart.sync_ppu(&mut ppu);
            assert_eq!(ppu.read_vram(0), 0xff);
        }
    }

    #[test]
    fn mapper185_bus_conflict_controls_protection_latch() {
        let mut rom = nes2_mapper_rom(185, 5, 2, 1);
        rom[16] = 0x01;
        let chr_start = 16 + 2 * 0x4000;
        rom[chr_start] = 0x66;
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        cart.write_mapper(0x8000, 0x03);
        cart.sync_ppu(&mut ppu);
        assert_eq!(cart.prg_bank & 3, 1);
        assert_eq!(ppu.read_vram(0), 0x66);
    }

    #[test]
    fn mapper185_legacy_header_masks_first_two_ppudata_reads_and_saves_phase() {
        let mut rom = mapper_rom(185, 2, 1);
        rom[16] = 0xff;
        let chr_start = 16 + 2 * 0x4000;
        rom[chr_start..chr_start + 4].fill(0x42);
        let mut machine = NesMachine::from_rom(&rom).unwrap();
        machine.bus.ppu.v = 0;

        assert_eq!(machine.bus.read8(0x2007), 0xff);
        assert_eq!(machine.bus.cartridge.cnrom185_reads_remaining, 1);
        let state = machine.save_state().unwrap();
        assert_eq!(machine.bus.read8(0x2007), 0xff);
        assert_eq!(machine.bus.read8(0x2007), 0x42);

        machine.load_state(&state).unwrap();
        assert_eq!(machine.bus.cartridge.cnrom185_reads_remaining, 1);
        assert_eq!(machine.bus.read8(0x2007), 0xff);
        assert_eq!(machine.bus.read8(0x2007), 0x42);
    }
    #[test]
    fn bandai_74161_mapper70_banks_prg_chr_and_applies_bus_conflicts() {
        let mut rom = mapper_rom(70, 8, 16);
        for bank in 0..8usize {
            rom[16 + bank * 0x4000..16 + (bank + 1) * 0x4000].fill((0x20 + bank) as u8);
        }
        let chr_start = 16 + 8 * 0x4000;
        for bank in 0..16usize {
            rom[chr_start + bank * 0x2000..chr_start + (bank + 1) * 0x2000]
                .fill((0x40 + bank) as u8);
        }
        rom[16 + 0x1234] = 0xff;
        rom[16 + 3 * 0x4000 + 0x2345] = 0x12;
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();

        cart.write_mapper(0x9234, 0x35);
        cart.sync_ppu(&mut ppu);
        assert_eq!(cart.read_prg(0x8000), 0x23);
        assert_eq!(cart.read_prg(0xc000), 0x27);
        assert_eq!(ppu.read_vram(0), 0x45);
        assert_eq!(ppu.mirroring, Mirroring::Horizontal);

        cart.write_mapper(0xa345, 0xff);
        cart.sync_ppu(&mut ppu);
        assert_eq!(cart.simple_reg, 0x12);
        assert_eq!(cart.read_prg(0x8000), 0x21);
        assert_eq!(ppu.read_vram(0), 0x42);
    }

    #[test]
    fn bandai_74161_mapper152_uses_bit_seven_for_one_screen_mirroring() {
        let mut rom = mapper_rom(152, 8, 16);
        for bank in 0..8usize {
            rom[16 + bank * 0x4000..16 + (bank + 1) * 0x4000].fill((0x30 + bank) as u8);
        }
        let chr_start = 16 + 8 * 0x4000;
        for bank in 0..16usize {
            rom[chr_start + bank * 0x2000..chr_start + (bank + 1) * 0x2000]
                .fill((0x50 + bank) as u8);
        }
        rom[16 + 0x1000] = 0xff;
        rom[16 + 3 * 0x4000 + 0x1100] = 0xff;
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();

        cart.write_mapper(0x9000, 0xb4);
        cart.sync_ppu(&mut ppu);
        assert_eq!(cart.read_prg(0x8000), 0x33);
        assert_eq!(ppu.read_vram(0), 0x54);
        assert_eq!(ppu.mirroring, Mirroring::SingleScreen1);

        cart.write_mapper(0x9100, 0x24);
        cart.sync_ppu(&mut ppu);
        assert_eq!(cart.read_prg(0x8000), 0x32);
        assert_eq!(ppu.mirroring, Mirroring::SingleScreen0);
    }

    #[test]
    fn mapper70_legacy_alternative_nametable_header_uses_mapper152_wiring() {
        let mut rom = mapper_rom(70, 16, 16);
        rom[6] |= 0x08;
        for bank in 0..16usize {
            rom[16 + bank * 0x4000..16 + (bank + 1) * 0x4000].fill(bank as u8);
        }
        rom[16 + 0x0800] = 0xff;
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        cart.write_mapper(0x8800, 0xb2);
        cart.sync_ppu(&mut ppu);
        assert_eq!(cart.read_prg(0x8000), 3);
        assert_eq!(ppu.mirroring, Mirroring::SingleScreen1);
    }

    #[test]
    fn mapper78_submapper_one_uses_one_screen_wiring_and_bus_conflicts() {
        let mut rom = nes2_mapper_rom(78, 1, 8, 16);
        for bank in 0..8usize {
            rom[16 + bank * 0x4000..16 + (bank + 1) * 0x4000].fill((0x20 + bank) as u8);
        }
        let chr_start = 16 + 8 * 0x4000;
        for bank in 0..16usize {
            rom[chr_start + bank * 0x2000..chr_start + (bank + 1) * 0x2000]
                .fill((0x40 + bank) as u8);
        }
        rom[16 + 0x1000] = 0xff;
        rom[16 + 5 * 0x4000 + 0x1100] = 0xff;
        rom[16 + 5 * 0x4000 + 0x1200] = 0x8b;
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();

        cart.write_mapper(0x9000, 0xc5);
        cart.sync_ppu(&mut ppu);
        assert_eq!(cart.read_prg(0x8000), 0x25);
        assert_eq!(cart.read_prg(0xc000), 0x27);
        assert_eq!(ppu.read_vram(0), 0x4c);
        assert_eq!(ppu.mirroring, Mirroring::SingleScreen0);

        cart.write_mapper(0x9100, 0xcd);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.mirroring, Mirroring::SingleScreen1);
        cart.write_mapper(0x9200, 0xff);
        cart.sync_ppu(&mut ppu);
        assert_eq!(cart.simple_reg, 0x8b);
        assert_eq!(cart.read_prg(0x8000), 0x23);
        assert_eq!(ppu.read_vram(0), 0x48);
    }

    #[test]
    fn mapper78_submapper_three_uses_holy_diver_horizontal_vertical_wiring() {
        let mut rom = nes2_mapper_rom(78, 3, 8, 16);
        for bank in 0..8usize {
            rom[16 + bank * 0x4000..16 + (bank + 1) * 0x4000].fill(bank as u8);
        }
        let chr_start = 16 + 8 * 0x4000;
        for bank in 0..16usize {
            rom[chr_start + bank * 0x2000..chr_start + (bank + 1) * 0x2000].fill(bank as u8);
        }
        rom[16 + 0x0800] = 0xff;
        rom[16 + 2 * 0x4000 + 0x0900] = 0xff;
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        cart.write_mapper(0x8800, 0xa2);
        cart.sync_ppu(&mut ppu);
        assert_eq!(cart.read_prg(0x8000), 2);
        assert_eq!(ppu.read_vram(0), 10);
        assert_eq!(ppu.mirroring, Mirroring::Horizontal);
        cart.write_mapper(0x8900, 0xaa);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.mirroring, Mirroring::Vertical);
    }

    #[test]
    fn mapper78_legacy_header_uses_alternative_nametable_bit_to_choose_board() {
        let mut uchuusen = mapper_rom(78, 8, 16);
        uchuusen[16 + 0x0400] = 0xff;
        let (mut uchuusen, mut uchuusen_ppu) = Cartridge::parse(&uchuusen).unwrap();
        uchuusen.write_mapper(0x8400, 0x08);
        uchuusen.sync_ppu(&mut uchuusen_ppu);
        assert_eq!(uchuusen_ppu.mirroring, Mirroring::SingleScreen1);

        let mut holy_diver = mapper_rom(78, 8, 16);
        holy_diver[6] |= 0x08;
        holy_diver[16 + 0x0400] = 0xff;
        let (mut holy_diver, mut holy_diver_ppu) = Cartridge::parse(&holy_diver).unwrap();
        holy_diver.write_mapper(0x8400, 0x08);
        holy_diver.sync_ppu(&mut holy_diver_ppu);
        assert_eq!(holy_diver_ppu.mirroring, Mirroring::Vertical);
    }

    #[test]
    fn sunsoft2_mapper89_banks_prg_chr_mirroring_and_obeys_bus_conflicts() {
        let mut rom = mapper_rom(89, 8, 16);
        for bank in 0..8usize {
            rom[16 + bank * 0x4000..16 + (bank + 1) * 0x4000].fill((0x20 + bank) as u8);
        }
        let chr_start = 16 + 8 * 0x4000;
        for bank in 0..16usize {
            rom[chr_start + bank * 0x2000..chr_start + (bank + 1) * 0x2000]
                .fill((0x40 + bank) as u8);
        }
        rom[16 + 0x1000] = 0xff;
        rom[16 + 5 * 0x4000 + 0x1100] = 0x42;
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        cart.write_mapper(0x9000, 0xdb);
        cart.sync_ppu(&mut ppu);
        assert_eq!(cart.read_prg(0x8000), 0x25);
        assert_eq!(cart.read_prg(0xc000), 0x27);
        assert_eq!(ppu.read_vram(0), 0x4b);
        assert_eq!(ppu.mirroring, Mirroring::SingleScreen1);

        cart.write_mapper(0x9100, 0xff);
        cart.sync_ppu(&mut ppu);
        assert_eq!(cart.simple_reg, 0x42);
        assert_eq!(cart.read_prg(0x8000), 0x24);
        assert_eq!(ppu.read_vram(0), 0x42);
        assert_eq!(ppu.mirroring, Mirroring::SingleScreen0);
    }

    #[test]
    fn sunsoft2_mapper93_gates_chr_ram_and_keeps_header_mirroring() {
        let mut rom = mapper_rom(93, 8, 0);
        for bank in 0..8usize {
            rom[16 + bank * 0x4000..16 + (bank + 1) * 0x4000].fill((0x30 + bank) as u8);
        }
        rom[16 + 0x1000] = 0xff;
        rom[16 + 3 * 0x4000 + 0x1100] = 0x40;
        rom[16 + 4 * 0x4000 + 0x1200] = 0xff;
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        assert_eq!(ppu.read_vram(0), 0xff);
        ppu.write_vram(0, 0x11);
        assert_eq!(ppu.read_vram(0), 0xff);

        cart.write_mapper(0x9000, 0x31);
        cart.sync_ppu(&mut ppu);
        assert_eq!(cart.read_prg(0x8000), 0x33);
        assert_eq!(cart.read_prg(0xc000), 0x37);
        assert_eq!(ppu.mirroring, Mirroring::Horizontal);
        ppu.write_vram(0, 0x5a);
        assert_eq!(ppu.read_vram(0), 0x5a);

        cart.write_mapper(0x9100, 0xff);
        cart.sync_ppu(&mut ppu);
        assert_eq!(cart.simple_reg, 0x40);
        assert_eq!(cart.read_prg(0x8000), 0x34);
        assert_eq!(ppu.read_vram(0), 0xff);
        ppu.write_vram(0, 0xa5);

        cart.write_mapper(0x9200, 0x41);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.read_vram(0), 0x5a);
    }

    #[test]
    fn jaleco_jf17_mapper72_latches_prg_chr_and_bus_conflicts() {
        let mut rom = mapper_rom(72, 8, 16);
        for bank in 0..8usize {
            rom[16 + bank * 0x4000..16 + (bank + 1) * 0x4000].fill((0x20 + bank) as u8);
        }
        let chr_start = 16 + 8 * 0x4000;
        for bank in 0..16usize {
            rom[chr_start + bank * 0x2000..chr_start + (bank + 1) * 0x2000]
                .fill((0x40 + bank) as u8);
        }
        rom[16 + 0x1000] = 0xff;
        rom[16 + 0x1100] = 0xff;
        rom[16 + 0x1200] = 0xff;
        rom[16 + 5 * 0x4000 + 0x1300] = 0x82;

        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        assert_eq!(cart.read_prg(0x8000), 0x20);
        assert_eq!(cart.read_prg(0xc000), 0x27);

        cart.write_mapper(0x9000, 0x43);
        cart.sync_ppu(&mut ppu);
        assert_eq!(cart.read_prg(0x8000), 0x20);
        assert_eq!(ppu.read_vram(0), 0x43);

        cart.write_mapper(0x9100, 0x03);
        cart.write_mapper(0x9200, 0x85);
        assert_eq!(cart.read_prg(0x8000), 0x25);
        assert_eq!(cart.read_prg(0xc000), 0x27);

        cart.write_mapper(0x9300, 0x05);
        cart.write_mapper(0x9300, 0x8f);
        assert_eq!(cart.jaleco_jf17.prg, 2);
        assert_eq!(cart.read_prg(0x8000), 0x22);
    }

    #[test]
    fn jaleco_jf19_mapper92_fixes_lower_prg_and_switches_upper() {
        let mut rom = mapper_rom(92, 16, 16);
        for bank in 0..16usize {
            rom[16 + bank * 0x4000..16 + (bank + 1) * 0x4000].fill((0x10 + bank) as u8);
        }
        let chr_start = 16 + 16 * 0x4000;
        for bank in 0..16usize {
            rom[chr_start + bank * 0x2000..chr_start + (bank + 1) * 0x2000]
                .fill((0x60 + bank) as u8);
        }
        rom[16 + 0x1000] = 0xff;
        rom[16 + 0x1100] = 0xff;

        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        cart.write_mapper(0x9000, 0x8a);
        cart.write_mapper(0x9100, 0x4c);
        cart.sync_ppu(&mut ppu);

        assert_eq!(cart.read_prg(0x8000), 0x10);
        assert_eq!(cart.read_prg(0xc000), 0x1a);
        assert_eq!(ppu.read_vram(0), 0x6c);
        assert_eq!(cart.jaleco_jf17.prg, 10);
        assert_eq!(cart.jaleco_jf17.chr, 12);
    }

    #[test]
    fn jaleco_jf17_state_round_trip_preserves_latch_phase() {
        let mut rom = mapper_rom(72, 8, 16);
        rom[16 + 0x1000] = 0xff;
        rom[16 + 0x1100] = 0xff;
        rom[16 + 0x1200] = 0xff;
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();
        cart.write_mapper(0x9000, 0x44);

        let mut writer = StateWriter::new(PlatformId::Nes, 99);
        save_mapper_state(&mut writer, &cart);
        let bytes = writer.finish();
        let (mut restored, _) = Cartridge::parse(&rom).unwrap();
        let mut reader = StateReader::new(&bytes, PlatformId::Nes, 99).unwrap();
        load_mapper_state(&mut reader, &mut restored).unwrap();
        reader.finish().unwrap();

        assert_eq!(restored.jaleco_jf17.prg, cart.jaleco_jf17.prg);
        assert_eq!(restored.jaleco_jf17.chr, cart.jaleco_jf17.chr);
        assert_eq!(restored.jaleco_jf17.control, cart.jaleco_jf17.control);

        restored.write_mapper(0x9100, 0x45);
        assert_eq!(restored.jaleco_jf17.chr, 4);
        restored.write_mapper(0x9200, 0x05);
        restored.write_mapper(0x9100, 0x45);
        assert_eq!(restored.jaleco_jf17.chr, 5);
    }

    #[test]
    fn oeka_kids_tablet_serializes_eighteen_bits_with_delayed_acknowledgements() {
        let mut tablet = OekaKidsTablet {
            x: 0x96,
            y: 0x5a,
            touching: true,
            clicked: true,
            ..OekaKidsTablet::default()
        };
        let expected = (u32::from(tablet.x) << 10)
            | (u32::from(tablet.y) << 2)
            | (u32::from(tablet.touching) << 1)
            | u32::from(tablet.clicked);

        tablet.write(0);
        tablet.write(1);
        assert_eq!(tablet.read() & 0x0c, 0x04);

        let mut received = 0u32;
        for _ in 0..18 {
            tablet.write(3);
            assert_eq!(tablet.read() & 0x04, 0x04);
            for _ in 1..OEKA_ACK_ADVANCE_CYCLES {
                tablet.tick();
            }
            assert_eq!(tablet.read() & 0x04, 0x04);
            tablet.tick();
            assert_eq!(tablet.read() & 0x04, 0);

            received = (received << 1) | u32::from(tablet.read() & 0x08 == 0);

            tablet.write(1);
            assert_eq!(tablet.read() & 0x04, 0);
            for _ in 1..OEKA_ACK_RELEASE_CYCLES {
                tablet.tick();
            }
            assert_eq!(tablet.read() & 0x04, 0);
            tablet.tick();
            assert_eq!(tablet.read() & 0x04, 0x04);
        }
        assert_eq!(received, expected);
    }

    #[test]
    fn oeka_kids_tablet_maps_aux_pointer_axes_and_contact_state() {
        let mut tablet = OekaKidsTablet::default();
        tablet.set_input(i16::MIN, i16::MIN, false, false);
        assert_eq!(
            (tablet.x, tablet.y, tablet.touching, tablet.clicked),
            (8, 0, false, false)
        );

        tablet.set_input(i16::MAX, i16::MAX, false, true);
        assert_eq!((tablet.x, tablet.y), (247, 242));
        assert!(tablet.touching);
        assert!(tablet.clicked);
    }

    #[test]
    fn oeka_kids_tablet_machine_state_preserves_mid_ack_phase() {
        let rom = mapper_rom(96, 8, 0);
        let mut machine = NesMachine::from_rom(&rom).unwrap();
        machine.bus.oeka_tablet.x = 0x96;
        machine.bus.oeka_tablet.y = 0x5a;
        machine.bus.oeka_tablet.touching = true;
        machine.bus.write8(0x4016, 0);
        machine.bus.write8(0x4016, 1);
        machine.bus.write8(0x4016, 3);
        for _ in 0..73 {
            machine.bus.oeka_tablet.tick();
        }

        let state = machine.save_state().unwrap();
        for _ in 73..OEKA_ACK_ADVANCE_CYCLES {
            machine.bus.oeka_tablet.tick();
        }
        let expected = machine.bus.read8(0x4017) & 0x0c;

        machine.load_state(&state).unwrap();
        for _ in 73..OEKA_ACK_ADVANCE_CYCLES {
            machine.bus.oeka_tablet.tick();
        }
        assert_eq!(machine.bus.read8(0x4017) & 0x0c, expected);
        assert_eq!(
            machine.bus.oeka_tablet.report,
            (0x96u32 << 11) | (0x5au32 << 3) | 4
        );
    }

    #[test]
    fn oeka_kids_mapper96_switches_prg_outer_chr_and_nametable_latched_inner_chr() {
        let mut rom = mapper_rom(96, 8, 0);
        for bank in 0..4usize {
            rom[16 + bank * 0x8000..16 + (bank + 1) * 0x8000].fill((0x20 + bank) as u8);
        }
        rom[16 + 0x1000] = 0xff;
        rom[16 + 3 * 0x8000 + 0x1100] = 0xff;
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        for bank in 0..8usize {
            ppu.chr[bank * 0x1000] = (0x40 + bank) as u8;
        }

        cart.write_mapper(0x9000, 0x07);
        cart.sync_ppu(&mut ppu);
        assert_eq!(cart.read_prg(0x8000), 0x23);
        assert_eq!(ppu.read_vram(0x0000), 0x44);
        assert_eq!(ppu.read_vram(0x1000), 0x47);
        assert_eq!(ppu.mirroring, Mirroring::Vertical);

        assert!(cart.observe_ppu_address(0x2200));
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.read_vram(0x0000), 0x46);
        assert!(!cart.observe_ppu_address(0x2300));
        assert!(!cart.observe_ppu_address(0x0000));
        assert!(cart.observe_ppu_address(0x2300));
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.read_vram(0x0000), 0x47);

        cart.write_mapper(0x9100, 0x02);
        cart.sync_ppu(&mut ppu);
        assert_eq!(cart.read_prg(0x8000), 0x22);
        assert_eq!(ppu.read_vram(0x0000), 0x43);
        assert_eq!(ppu.read_vram(0x1000), 0x43);
    }

    #[test]
    fn oeka_kids_mapper96_state_round_trip_preserves_address_latch_phase() {
        let mut rom = mapper_rom(96, 8, 0);
        rom[16] = 0xff;
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();
        cart.write_mapper(0x8000, 0x04);
        assert!(cart.observe_ppu_address(0x2200));

        let mut writer = StateWriter::new(PlatformId::Nes, 99);
        save_mapper_state(&mut writer, &cart);
        let bytes = writer.finish();
        let (mut restored, _) = Cartridge::parse(&rom).unwrap();
        let mut reader = StateReader::new(&bytes, PlatformId::Nes, 99).unwrap();
        load_mapper_state(&mut reader, &mut restored).unwrap();
        reader.finish().unwrap();

        assert_eq!(restored.simple_reg, cart.simple_reg);
        assert_eq!(restored.oeka_inner_chr, 2);
        assert_eq!(restored.oeka_last_dd, 2);
        assert!(!restored.observe_ppu_address(0x2300));
        assert!(!restored.observe_ppu_address(0x0000));
        assert!(restored.observe_ppu_address(0x2300));
        assert_eq!(restored.oeka_inner_chr, 3);
    }

    #[test]
    fn irem_tam_s1_mapper97_fixes_last_lower_bank_and_switches_upper_without_conflicts() {
        let mut rom = mapper_rom(97, 32, 1);
        for bank in 0..32usize {
            rom[16 + bank * 0x4000..16 + (bank + 1) * 0x4000].fill((0x20 + bank) as u8);
        }
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        assert!(cart.prg_ram.is_empty());
        assert_eq!(cart.read_prg(0x8000), 0x3f);
        assert_eq!(cart.read_prg(0xc000), 0x20);
        assert_eq!(ppu.mirroring, Mirroring::Horizontal);

        cart.write_mapper(0x9000, 0x91);
        cart.sync_ppu(&mut ppu);
        assert_eq!(cart.read_prg(0x8000), 0x3f);
        assert_eq!(cart.read_prg(0xc000), 0x31);
        assert_eq!(ppu.mirroring, Mirroring::Vertical);

        cart.write_mapper(0xc000, 0x03);
        assert_eq!(cart.read_prg(0xc000), 0x31);
        cart.write_mapper(0xbfff, 0x03);
        cart.sync_ppu(&mut ppu);
        assert_eq!(cart.read_prg(0xc000), 0x23);
        assert_eq!(ppu.mirroring, Mirroring::Horizontal);
    }

    #[test]
    fn jaleco_jf13_mapper86_banks_prg_chr_and_decodes_low_and_high_register_mirrors() {
        let mut rom = mapper_rom(86, 8, 8);
        for bank in 0..4usize {
            rom[16 + bank * 0x8000..16 + (bank + 1) * 0x8000].fill((0x20 + bank) as u8);
        }
        let chr_start = 16 + 8 * 0x4000;
        for bank in 0..8usize {
            rom[chr_start + bank * 0x2000..chr_start + (bank + 1) * 0x2000]
                .fill((0x40 + bank) as u8);
        }
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        assert!(cart.prg_ram.is_empty());

        cart.write_ram(0x6000, 0x63);
        cart.sync_ppu(&mut ppu);
        assert_eq!(cart.read_prg(0x8000), 0x22);
        assert_eq!(ppu.read_vram(0), 0x47);

        cart.write_ram(0x7000, 0x2a);
        assert_eq!(cart.jaleco_d7756_control, 0x2a);
        cart.write_mapper(0xe000, 0x11);
        cart.sync_ppu(&mut ppu);
        assert_eq!(cart.read_prg(0x8000), 0x21);
        assert_eq!(ppu.read_vram(0), 0x41);
        cart.write_mapper(0xf000, 0x3f);
        assert_eq!(cart.jaleco_d7756_control, 0x3f);
        assert_eq!(cart.read_ram(0x6000), 0xff);
    }

    #[test]
    fn jaleco_jf13_mapper86_state_round_trip_preserves_banks_and_audio_control() {
        let rom = mapper_rom(86, 8, 8);
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();
        cart.write_ram(0x6000, 0x52);
        cart.write_ram(0x7000, 0x2c);

        let mut writer = StateWriter::new(PlatformId::Nes, 99);
        save_mapper_state(&mut writer, &cart);
        let bytes = writer.finish();
        let (mut restored, _) = Cartridge::parse(&rom).unwrap();
        let mut reader = StateReader::new(&bytes, PlatformId::Nes, 99).unwrap();
        load_mapper_state(&mut reader, &mut restored).unwrap();
        reader.finish().unwrap();

        assert_eq!(restored.simple_reg, 0x52);
        assert_eq!(restored.jaleco_d7756_control, 0x2c);
    }

    fn mmc1_write(cart: &mut Cartridge, address: u16, value: u8) {
        for bit in 0..5 {
            cart.write_mapper(address, (value >> bit) & 1);
        }
    }

    fn fme7_write(cart: &mut Cartridge, command: u8, value: u8) {
        cart.write_mapper(0x8000, command);
        cart.write_mapper(0xa000, value);
    }

    fn mmc5_test_rom(chr_banks: u8) -> Vec<u8> {
        let mut rom = mapper_rom(5, 8, chr_banks);
        for bank in 0..16usize {
            rom[16 + bank * 0x2000..16 + (bank + 1) * 0x2000].fill(bank as u8);
        }
        let chr_start = 16 + 8 * 0x4000;
        for bank in 0..usize::from(chr_banks) * 8 {
            rom[chr_start + bank * 0x400..chr_start + (bank + 1) * 0x400].fill(bank as u8);
        }
        rom
    }

    fn rambo1_test_rom(mapper: u16) -> Vec<u8> {
        let mut rom = mapper_rom(mapper, 16, 32);
        for bank in 0..32usize {
            rom[16 + bank * 0x2000..16 + (bank + 1) * 0x2000].fill(bank as u8);
        }
        let chr_start = 16 + 16 * 0x4000;
        for bank in 0..256usize {
            rom[chr_start + bank * 0x400..chr_start + (bank + 1) * 0x400].fill(bank as u8);
        }
        rom
    }

    fn txsrom_test_rom() -> Vec<u8> {
        let mut rom = mapper_rom(118, 16, 16);
        for bank in 0..32usize {
            rom[16 + bank * 0x2000..16 + (bank + 1) * 0x2000].fill(bank as u8);
        }
        let chr_start = 16 + 16 * 0x4000;
        for bank in 0..128usize {
            rom[chr_start + bank * 0x400..chr_start + (bank + 1) * 0x400].fill(bank as u8);
        }
        rom
    }

    fn namco3425_test_rom() -> Vec<u8> {
        let mut rom = mapper_rom(95, 8, 8);
        for bank in 0..16usize {
            rom[16 + bank * 0x2000..16 + (bank + 1) * 0x2000].fill(bank as u8);
        }
        let chr_start = 16 + 8 * 0x4000;
        for bank in 0..64usize {
            rom[chr_start + bank * 0x400..chr_start + (bank + 1) * 0x400].fill(bank as u8);
        }
        rom
    }

    fn bandai_test_rom(submapper: u8, eeprom: bool) -> Vec<u8> {
        let mut rom = nes2_mapper_rom(16, submapper, 8, 8);
        if eeprom {
            rom[10] = 0x20;
        }
        for bank in 0..8usize {
            rom[16 + bank * 0x4000..16 + (bank + 1) * 0x4000].fill(bank as u8);
        }
        let chr_start = 16 + 8 * 0x4000;
        for bank in 0..64usize {
            rom[chr_start + bank * 0x400..chr_start + (bank + 1) * 0x400].fill(bank as u8);
        }
        rom
    }

    fn bandai_i2c_lines(cart: &mut Cartridge, scl: bool, sda: bool, release: bool) {
        let value = (u8::from(release) << 7) | (u8::from(sda) << 6) | (u8::from(scl) << 5);
        cart.write_mapper(0x800d, value);
    }

    fn bandai_i2c_start(cart: &mut Cartridge) {
        bandai_i2c_lines(cart, false, true, false);
        bandai_i2c_lines(cart, true, true, false);
        bandai_i2c_lines(cart, true, false, false);
        bandai_i2c_lines(cart, false, false, false);
    }

    fn bandai_i2c_write_byte(cart: &mut Cartridge, value: u8) {
        for bit in (0..8).rev() {
            let high = value & (1 << bit) != 0;
            bandai_i2c_lines(cart, false, high, false);
            bandai_i2c_lines(cart, true, high, false);
            bandai_i2c_lines(cart, false, high, false);
        }
        bandai_i2c_lines(cart, false, true, true);
        bandai_i2c_lines(cart, true, true, true);
        assert_eq!(cart.read_ram(0x6000) & 0x10, 0);
        bandai_i2c_lines(cart, false, true, true);
    }

    fn bandai_i2c_read_byte(cart: &mut Cartridge) -> u8 {
        let mut value = 0u8;
        for _ in 0..8 {
            bandai_i2c_lines(cart, false, true, true);
            bandai_i2c_lines(cart, true, true, true);
            value = (value << 1) | u8::from(cart.read_ram(0x6000) & 0x10 != 0);
            bandai_i2c_lines(cart, false, true, true);
        }
        bandai_i2c_lines(cart, false, true, false);
        bandai_i2c_lines(cart, true, true, false);
        bandai_i2c_lines(cart, false, true, false);
        value
    }

    fn bandai_i2c_stop(cart: &mut Cartridge) {
        bandai_i2c_lines(cart, false, false, false);
        bandai_i2c_lines(cart, true, false, false);
        bandai_i2c_lines(cart, true, true, false);
        bandai_i2c_lines(cart, false, true, false);
    }

    fn datach_external_lines(cart: &mut Cartridge, scl: bool, sda: bool, release: bool) {
        let data = (u8::from(release) << 7) | (u8::from(sda) << 6);
        if cart.bandai.external_scl && !scl {
            cart.write_mapper(0x8000, 0);
            cart.write_mapper(0x800d, data);
        } else {
            cart.write_mapper(0x800d, data);
            cart.write_mapper(0x8000, u8::from(scl) << 3);
        }
    }

    fn datach_external_i2c_start(cart: &mut Cartridge) {
        datach_external_lines(cart, false, true, false);
        datach_external_lines(cart, true, true, false);
        datach_external_lines(cart, true, false, false);
        datach_external_lines(cart, false, false, false);
    }

    fn datach_external_i2c_write_byte(cart: &mut Cartridge, value: u8) {
        for bit in (0..8).rev() {
            let high = value & (1 << bit) != 0;
            datach_external_lines(cart, false, high, false);
            datach_external_lines(cart, true, high, false);
            datach_external_lines(cart, false, high, false);
        }
        datach_external_lines(cart, false, true, true);
        datach_external_lines(cart, true, true, true);
        assert_eq!(cart.read_ram(0x6000) & 0x10, 0);
        datach_external_lines(cart, false, true, true);
    }

    fn datach_external_i2c_read_byte(cart: &mut Cartridge) -> u8 {
        let mut value = 0u8;
        for _ in 0..8 {
            datach_external_lines(cart, false, true, true);
            datach_external_lines(cart, true, true, true);
            value = (value << 1) | u8::from(cart.read_ram(0x6000) & 0x10 != 0);
            datach_external_lines(cart, false, true, true);
        }
        datach_external_lines(cart, false, true, false);
        datach_external_lines(cart, true, true, false);
        datach_external_lines(cart, false, true, false);
        value
    }

    fn datach_external_i2c_stop(cart: &mut Cartridge) {
        datach_external_lines(cart, false, false, false);
        datach_external_lines(cart, true, false, false);
        datach_external_lines(cart, true, true, false);
        datach_external_lines(cart, false, true, false);
    }

    fn namco210_test_rom(submapper: u8, prg_ram_shift: u8) -> Vec<u8> {
        let mut rom = nes2_mapper_rom(210, submapper, 16, 16);
        rom[10] = prg_ram_shift << 4;
        for bank in 0..32usize {
            rom[16 + bank * 0x2000..16 + (bank + 1) * 0x2000].fill(bank as u8);
        }
        let chr_start = 16 + 16 * 0x4000;
        for bank in 0..128usize {
            rom[chr_start + bank * 0x400..chr_start + (bank + 1) * 0x400].fill(bank as u8);
        }
        rom
    }

    fn namco163_test_rom(submapper: u8, prg_nvram_shift: u8, battery: bool) -> Vec<u8> {
        let mut rom = nes2_mapper_rom(19, submapper, 16, 16);
        if battery {
            rom[6] |= 0x02;
        }
        rom[10] = prg_nvram_shift << 4;
        for bank in 0..32usize {
            rom[16 + bank * 0x2000..16 + (bank + 1) * 0x2000].fill(bank as u8);
        }
        let chr_start = 16 + 16 * 0x4000;
        for bank in 0..128usize {
            rom[chr_start + bank * 0x400..chr_start + (bank + 1) * 0x400].fill(bank as u8);
        }
        rom
    }

    fn vrc7_test_rom(submapper: u8) -> Vec<u8> {
        let mut rom = nes2_mapper_rom(85, submapper, 16, 16);
        rom[10] = 7;
        for bank in 0..32usize {
            rom[16 + bank * 0x2000..16 + (bank + 1) * 0x2000].fill((0x20 + bank) as u8);
        }
        let chr_start = 16 + 16 * 0x4000;
        for bank in 0..128usize {
            rom[chr_start + bank * 0x400..chr_start + (bank + 1) * 0x400].fill((0x40 + bank) as u8);
        }
        rom
    }

    #[test]
    fn bandai_lz93d50_banks_prg_chr_mirroring_and_latched_irq() {
        let rom = bandai_test_rom(5, false);
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        assert!(cart.prg_ram.is_empty());
        cart.write_mapper(0x8008, 3);
        assert_eq!(cart.read_prg(0x8000), 3);
        assert_eq!(cart.read_prg(0xc000), 7);
        for bank in 0..8u8 {
            cart.write_mapper(0x8000 + u16::from(bank), bank + 8);
        }
        cart.sync_ppu(&mut ppu);
        for slot in 0..8usize {
            assert_eq!(ppu.read_vram((slot * 0x400) as u16), (slot + 8) as u8);
        }
        cart.write_mapper(0x8009, 2);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.mirroring, Mirroring::SingleScreen0);
        cart.write_mapper(0x800b, 2);
        cart.write_mapper(0x800c, 0);
        cart.write_mapper(0x800a, 1);
        assert_eq!(cart.bandai.irq_counter, 2);
        cart.tick_cpu_cycle();
        assert!(!cart.irq_pending());
        cart.tick_cpu_cycle();
        assert!(cart.irq_pending());
        cart.write_mapper(0x800a, 0);
        assert!(!cart.irq_pending());
        cart.write_mapper(0x800b, 0);
        cart.write_mapper(0x800c, 0);
        cart.write_mapper(0x800a, 1);
        assert!(cart.irq_pending());
    }

    #[test]
    fn bandai_fcg_submapper_four_uses_low_register_window_and_direct_counter() {
        let rom = bandai_test_rom(4, false);
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();
        cart.write_mapper(0x8008, 4);
        assert_eq!(cart.read_prg(0x8000), 0);
        assert!(cart.write_low_mapper(0x6008, 4));
        assert_eq!(cart.read_prg(0x8000), 4);
        cart.write_low_mapper(0x600b, 2);
        cart.write_low_mapper(0x600c, 0);
        assert_eq!(cart.bandai.irq_counter, 2);
        cart.write_low_mapper(0x600a, 1);
        assert_eq!(cart.bandai.irq_counter, 2);
        cart.tick_cpu_cycle();
        assert!(!cart.irq_pending());
        cart.tick_cpu_cycle();
        assert!(cart.irq_pending());
    }

    #[test]
    fn bandai_mapper153_selects_outer_prg_and_gates_battery_ram() {
        let mut rom = nes2_mapper_rom(153, 0, 32, 0);
        for bank in 0..32usize {
            rom[16 + bank * 0x4000] = (0x20 + bank) as u8;
        }
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        assert_eq!(cart.prg_ram.len(), 8 * 1024);
        assert_eq!(cart.battery_len, 8 * 1024);
        assert_eq!(cart.chr_banks_1k, 8);
        cart.write_ram(0x6000, 0x5a);
        assert_eq!(cart.read_ram(0x6000), 0xff);

        cart.write_mapper(0x800d, 0x20);
        cart.write_ram(0x6000, 0x5a);
        assert_eq!(cart.read_ram(0x6000), 0x5a);
        cart.write_mapper(0x8008, 2);
        assert_eq!(cart.read_prg(0x8000), 0x22);
        assert_eq!(cart.read_prg(0xc000), 0x2f);
        cart.write_mapper(0x8000, 1);
        assert_eq!(cart.read_prg(0x8000), 0x32);
        assert_eq!(cart.read_prg(0xc000), 0x3f);

        cart.write_mapper(0x8009, 3);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.mirroring, Mirroring::SingleScreen1);
        cart.write_mapper(0x800b, 2);
        cart.write_mapper(0x800c, 0);
        cart.write_mapper(0x800a, 1);
        cart.tick_cpu_cycle();
        assert!(!cart.irq_pending());
        cart.tick_cpu_cycle();
        assert!(cart.irq_pending());
    }

    #[test]
    fn bandai_mapper159_x24c01_bitbang_and_persistence() {
        let mut rom = nes2_mapper_rom(159, 0, 8, 8);
        rom[10] = 0x10;
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        assert!(cart.prg_ram.is_empty());
        assert!(cart.bandai.eeprom_present);

        cart.write_mapper(0x8008, 3);
        assert_eq!(cart.bandai.prg, 3);
        cart.write_mapper(0x8000, 7);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.chr_map[0], 7);

        bandai_i2c_start(&mut cart);
        bandai_i2c_write_byte(&mut cart, 0x42 << 1);
        bandai_i2c_write_byte(&mut cart, 0x5a);
        bandai_i2c_stop(&mut cart);
        assert_eq!(cart.bandai.eeprom.data[0x42], 0x5a);

        bandai_i2c_start(&mut cart);
        bandai_i2c_write_byte(&mut cart, (0x42 << 1) | 1);
        assert_eq!(bandai_i2c_read_byte(&mut cart), 0x5a);
        bandai_i2c_stop(&mut cart);

        bandai_i2c_start(&mut cart);
        bandai_i2c_write_byte(&mut cart, 0x7e << 1);
        for value in [0x11, 0x22, 0x33, 0x44] {
            bandai_i2c_write_byte(&mut cart, value);
        }
        bandai_i2c_stop(&mut cart);
        assert_eq!(cart.bandai.eeprom.data[0x7e], 0x11);
        assert_eq!(cart.bandai.eeprom.data[0x7f], 0x22);
        assert_eq!(cart.bandai.eeprom.data[0x7c], 0x33);
        assert_eq!(cart.bandai.eeprom.data[0x7d], 0x44);

        let mut machine = NesMachine::from_rom(&rom).unwrap();
        assert_eq!(machine.persistent_len(ResourceKind::Storage, 0), 128);
        let saved = [0xa5u8; 128];
        machine
            .write_persistent(ResourceKind::Storage, 0, &saved)
            .unwrap();
        let mut loaded = [0u8; 128];
        machine
            .read_persistent(ResourceKind::Storage, 0, &mut loaded)
            .unwrap();
        assert_eq!(loaded, saved);
    }

    #[test]
    fn datach_mapper157_routes_dual_eeproms_banking_irq_and_persistence() {
        let mut rom = nes2_mapper_rom(157, 0, 16, 0);
        rom[10] = 0x10;
        for bank in 0..16usize {
            rom[16 + bank * 0x4000..16 + (bank + 1) * 0x4000].fill(bank as u8);
        }

        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        assert!(cart.prg_ram.is_empty());
        assert!(cart.bandai.eeprom_present);
        assert!(cart.bandai.external_eeprom_present);
        assert!(ppu.chr_ram);

        let initial_chr_map = ppu.chr_map;
        cart.write_mapper(0x8000, 0xff);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.chr_map, initial_chr_map);

        cart.write_mapper(0x8008, 3);
        assert_eq!(cart.read_prg(0x8000), 3);
        assert_eq!(cart.read_prg(0xc000), 15);
        cart.write_mapper(0x8009, 2);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.mirroring, Mirroring::SingleScreen0);

        cart.write_mapper(0x800b, 2);
        cart.write_mapper(0x800c, 0);
        cart.write_mapper(0x800a, 1);
        cart.tick_cpu_cycle();
        assert!(!cart.irq_pending());
        cart.tick_cpu_cycle();
        assert!(cart.irq_pending());

        bandai_i2c_start(&mut cart);
        bandai_i2c_write_byte(&mut cart, 0xa0);
        bandai_i2c_write_byte(&mut cart, 0x12);
        bandai_i2c_write_byte(&mut cart, 0x5a);
        bandai_i2c_stop(&mut cart);
        assert_eq!(cart.bandai.eeprom.data[0x12], 0x5a);

        datach_external_i2c_start(&mut cart);
        datach_external_i2c_write_byte(&mut cart, 0x42 << 1);
        datach_external_i2c_write_byte(&mut cart, 0xa5);
        datach_external_i2c_stop(&mut cart);
        assert_eq!(cart.bandai.external_eeprom.data[0x42], 0xa5);

        datach_external_i2c_start(&mut cart);
        datach_external_i2c_write_byte(&mut cart, (0x42 << 1) | 1);
        assert_eq!(datach_external_i2c_read_byte(&mut cart), 0xa5);
        datach_external_i2c_stop(&mut cart);

        cart.bandai.eeprom.output = true;
        cart.bandai.external_eeprom.output = true;
        cart.bandai.barcode_line = true;
        assert_eq!(cart.read_ram(0x6000) & 0x18, 0x18);
        cart.bandai.external_eeprom.output = false;
        assert_eq!(cart.read_ram(0x6000) & 0x18, 0x08);
        cart.bandai.external_eeprom.output = true;

        cart.write_mapper(0x8000, 0x08);
        let mut writer = StateWriter::new(PlatformId::Nes, 99);
        save_mapper_state(&mut writer, &cart);
        let bytes = writer.finish();
        let (mut restored, _) = Cartridge::parse(&rom).unwrap();
        let mut reader = StateReader::new(&bytes, PlatformId::Nes, 99).unwrap();
        load_mapper_state(&mut reader, &mut restored).unwrap();
        reader.finish().unwrap();
        assert_eq!(restored.bandai.prg, 3);
        assert!(restored.bandai.external_scl);
        assert!(restored.bandai.barcode_line);
        assert_eq!(restored.bandai.eeprom.data[0x12], 0x5a);
        assert_eq!(restored.bandai.external_eeprom.data[0x42], 0xa5);

        let mut machine = NesMachine::from_rom(&rom).unwrap();
        assert_eq!(machine.persistent_len(ResourceKind::Storage, 0), 256);
        assert_eq!(machine.persistent_len(ResourceKind::Storage, 1), 128);
        let internal = [0x3cu8; 256];
        let external = [0xa6u8; 128];
        machine
            .write_persistent(ResourceKind::Storage, 0, &internal)
            .unwrap();
        machine
            .write_persistent(ResourceKind::Storage, 1, &external)
            .unwrap();
        let mut internal_loaded = [0u8; 256];
        let mut external_loaded = [0u8; 128];
        machine
            .read_persistent(ResourceKind::Storage, 0, &mut internal_loaded)
            .unwrap();
        machine
            .read_persistent(ResourceKind::Storage, 1, &mut external_loaded)
            .unwrap();
        assert_eq!(internal_loaded, internal);
        assert_eq!(external_loaded, external);

        let mut no_external_rom = rom.clone();
        no_external_rom[10] = 0;
        let no_external = NesMachine::from_rom(&no_external_rom).unwrap();
        assert!(!no_external.bus.cartridge.bandai.external_eeprom_present);
        assert_eq!(no_external.persistent_len(ResourceKind::Storage, 0), 256);
        assert_eq!(no_external.persistent_len(ResourceKind::Storage, 1), 0);
    }

    #[test]
    fn bandai_24c02_bitbang_persistence_and_state_round_trip() {
        let rom = bandai_test_rom(5, true);
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();
        assert!(cart.bandai.eeprom_present);
        bandai_i2c_start(&mut cart);
        bandai_i2c_write_byte(&mut cart, 0xa0);
        bandai_i2c_write_byte(&mut cart, 0x42);
        bandai_i2c_write_byte(&mut cart, 0x5a);
        bandai_i2c_stop(&mut cart);
        assert_eq!(cart.bandai.eeprom.data[0x42], 0x5a);

        bandai_i2c_start(&mut cart);
        bandai_i2c_write_byte(&mut cart, 0xa0);
        bandai_i2c_write_byte(&mut cart, 0x42);
        bandai_i2c_start(&mut cart);
        bandai_i2c_write_byte(&mut cart, 0xa1);
        assert_eq!(bandai_i2c_read_byte(&mut cart), 0x5a);
        bandai_i2c_stop(&mut cart);

        cart.write_mapper(0x8008, 5);
        cart.write_mapper(0x800b, 7);
        cart.write_mapper(0x800c, 1);
        cart.write_mapper(0x800a, 1);
        cart.tick_cpu_cycle();
        let mut writer = StateWriter::new(PlatformId::Nes, 99);
        save_mapper_state(&mut writer, &cart);
        let bytes = writer.finish();
        let (mut restored, _) = Cartridge::parse(&rom).unwrap();
        let mut reader = StateReader::new(&bytes, PlatformId::Nes, 99).unwrap();
        load_mapper_state(&mut reader, &mut restored).unwrap();
        reader.finish().unwrap();
        assert_eq!(restored.bandai.prg, cart.bandai.prg);
        assert_eq!(restored.bandai.irq_counter, cart.bandai.irq_counter);
        assert_eq!(restored.bandai.eeprom.data[0x42], 0x5a);

        let mut machine = NesMachine::from_rom(&rom).unwrap();
        assert_eq!(machine.persistent_len(ResourceKind::Storage, 0), 256);
        let saved = [0x3cu8; 256];
        machine
            .write_persistent(ResourceKind::Storage, 0, &saved)
            .unwrap();
        let mut loaded = [0u8; 256];
        machine
            .read_persistent(ResourceKind::Storage, 0, &mut loaded)
            .unwrap();
        assert_eq!(loaded, saved);
    }

    #[test]
    fn namco163_banks_prg_and_routes_chr_or_ciram() {
        let rom = namco163_test_rom(3, 0, false);
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        for (address, bank) in [(0xe000, 3), (0xe800, 4), (0xf000, 5)] {
            cart.write_mapper(address, bank);
        }
        assert_eq!(cart.read_prg(0x8000), 3);
        assert_eq!(cart.read_prg(0xa000), 4);
        assert_eq!(cart.read_prg(0xc000), 5);
        assert_eq!(cart.read_prg(0xe000), 31);

        cart.write_mapper(0x8000, 6);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.read_vram(0), 6);
        ppu.nametable[0x400] = 0xa5;
        cart.write_mapper(0x8000, 0xe1);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.read_vram(0), 0xa5);
        cart.write_mapper(0xe800, 0x44);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.read_vram(0), 0xe1 % 128);

        cart.write_mapper(0xc000, 7);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.read_vram(0x2000), 7);
        ppu.nametable[0] = 0x3c;
        cart.write_mapper(0xc000, 0xe0);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.read_vram(0x2000), 0x3c);
    }

    #[test]
    fn namco163_irq_chip_ram_and_wram_protection_follow_hardware_registers() {
        let rom = namco163_test_rom(2, 7, true);
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();
        assert_eq!(cart.prg_ram.len(), 8 * 1024);

        cart.write_mapper(0xf800, 0x40);
        cart.write_ram(0x6000, 0x11);
        assert_eq!(cart.read_ram(0x6000), 0x11);
        cart.write_mapper(0xf800, 0x41);
        cart.write_ram(0x6000, 0x22);
        cart.write_ram(0x6800, 0x33);
        assert_eq!(cart.read_ram(0x6000), 0x11);
        assert_eq!(cart.read_ram(0x6800), 0x33);

        cart.write_mapper(0xf800, 0xfe);
        assert!(cart.write_expansion_mapper(0x4800, 0x12));
        assert!(cart.write_expansion_mapper(0x4800, 0x34));
        assert_eq!(cart.namco163.ram[0x7e], 0x12);
        assert_eq!(cart.namco163.ram[0x7f], 0x34);
        assert_eq!(cart.namco163.ram_address, 0x7f);

        assert!(cart.write_expansion_mapper(0x5000, 0xfe));
        assert!(cart.write_expansion_mapper(0x5800, 0xff));
        cart.tick_cpu_cycle();
        assert_eq!(cart.namco163.irq_counter, 0x7fff);
        assert!(cart.irq_pending());
        cart.write_expansion_mapper(0x5000, 0);
        assert!(!cart.irq_pending());
    }

    #[test]
    fn namco163_audio_steps_wavetable_channels_every_fifteen_cpu_cycles() {
        let rom = namco163_test_rom(3, 0, false);
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();
        cart.write_mapper(0xe000, 0x00);
        for (address, value) in [
            (0x00, 0xf8),
            (0x78, 0x00),
            (0x79, 0x00),
            (0x7a, 0x00),
            (0x7b, 0x00),
            (0x7c, 0x01),
            (0x7d, 0x00),
            (0x7e, 0x00),
            (0x7f, 0x0f),
        ] {
            cart.write_mapper(0xf800, address);
            cart.write_expansion_mapper(0x4800, value);
        }
        for _ in 0..14 {
            cart.tick_cpu_cycle();
        }
        assert_eq!(cart.namco163.audio_output, 0.0);
        cart.tick_cpu_cycle();
        assert!(cart.namco163.audio_output > 0.0);
        assert_eq!(cart.namco163.ram[0x7d], 1);
        cart.write_mapper(0xe000, 0x40);
        assert_eq!(cart.namco163.audio_output, 0.0);
    }

    #[test]
    fn namco163_battery_storage_and_machine_state_round_trip() {
        let rom = namco163_test_rom(3, 7, true);
        let mut machine = NesMachine::from_rom(&rom).unwrap();
        let len = machine.persistent_len(ResourceKind::Storage, 0);
        assert_eq!(len, 8 * 1024 + 128);
        let mut saved = vec![0x3c; len];
        saved[8 * 1024..].fill(0xa5);
        machine
            .write_persistent(ResourceKind::Storage, 0, &saved)
            .unwrap();
        let mut loaded = vec![0; len];
        machine
            .read_persistent(ResourceKind::Storage, 0, &mut loaded)
            .unwrap();
        assert_eq!(loaded, saved);

        machine.bus.cartridge.write_mapper(0xe000, 5);
        machine.bus.cartridge.write_mapper(0x8000, 0xe1);
        machine.bus.cartridge.write_mapper(0xf800, 0x40);
        machine.bus.cartridge.write_expansion_mapper(0x5000, 0xfe);
        machine.bus.cartridge.write_expansion_mapper(0x5800, 0xff);
        let state = machine.save_state().unwrap();
        machine.bus.cartridge.namco163 = Namco163::default();
        machine.load_state(&state).unwrap();
        assert_eq!(machine.bus.cartridge.namco163.prg[0], 5);
        assert_eq!(machine.bus.cartridge.namco163.chr[0], 0xe1);
        assert_eq!(machine.bus.cartridge.namco163.irq_counter, 0x7ffe);
        assert_eq!(machine.bus.cartridge.namco163.ram[0x7f], 0xa5);
    }

    #[test]
    fn namco175_banks_prg_chr_and_mirrors_two_kib_wram() {
        let mut rom = namco210_test_rom(1, 5);
        rom[6] |= 1;
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        assert!(cart.namco210.is_175);
        assert_eq!(cart.prg_ram.len(), 2 * 1024);
        assert_eq!(cart.battery_len, 2 * 1024);
        assert_eq!(cart.read_ram(0x6000), 0xff);

        for (address, bank) in [(0xe000, 3), (0xe800, 4), (0xf000, 5)] {
            cart.write_mapper(address, bank);
        }
        assert_eq!(cart.read_prg(0x8000), 3);
        assert_eq!(cart.read_prg(0xa000), 4);
        assert_eq!(cart.read_prg(0xc000), 5);
        assert_eq!(cart.read_prg(0xe000), 31);
        for slot in 0..8u16 {
            cart.write_mapper(0x8000 + slot * 0x0800, (slot + 9) as u8);
        }
        cart.sync_ppu(&mut ppu);
        for slot in 0..8usize {
            assert_eq!(ppu.read_vram((slot * 0x400) as u16), (slot + 9) as u8);
        }

        cart.write_mapper(0xc000, 1);
        cart.write_ram(0x6000, 0x3c);
        assert_eq!(cart.read_ram(0x6800), 0x3c);
        assert_eq!(cart.read_ram(0x7000), 0x3c);
        assert_eq!(cart.read_ram(0x7800), 0x3c);
        cart.write_mapper(0xe000, 0xc3);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.mirroring, Mirroring::Vertical);
    }

    #[test]
    fn namco340_switches_all_four_mirroring_modes_without_wram() {
        let rom = namco210_test_rom(2, 0);
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        assert!(!cart.namco210.is_175);
        assert!(cart.prg_ram.is_empty());
        for (bits, expected) in [
            (0x00, Mirroring::SingleScreen0),
            (0x40, Mirroring::Vertical),
            (0x80, Mirroring::SingleScreen1),
            (0xc0, Mirroring::Horizontal),
        ] {
            cart.write_mapper(0xe000, bits | 7);
            cart.sync_ppu(&mut ppu);
            assert_eq!(cart.read_prg(0x8000), 7);
            assert_eq!(ppu.mirroring, expected);
        }
    }

    #[test]
    fn namco210_mapper_state_round_trip_preserves_registers() {
        let rom = namco210_test_rom(1, 7);
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();
        cart.write_mapper(0x9800, 0x23);
        cart.write_mapper(0xe000, 9);
        cart.write_mapper(0xe800, 10);
        cart.write_mapper(0xf000, 11);
        cart.write_mapper(0xc000, 1);

        let mut writer = StateWriter::new(PlatformId::Nes, 99);
        save_mapper_state(&mut writer, &cart);
        let bytes = writer.finish();
        let (mut restored, _) = Cartridge::parse(&rom).unwrap();
        let mut reader = StateReader::new(&bytes, PlatformId::Nes, 99).unwrap();
        load_mapper_state(&mut reader, &mut restored).unwrap();
        reader.finish().unwrap();
        assert_eq!(restored.namco210.chr, cart.namco210.chr);
        assert_eq!(restored.namco210.prg, cart.namco210.prg);
        assert_eq!(restored.namco210.ram_enabled, cart.namco210.ram_enabled);
        assert_eq!(restored.namco210.mirroring, cart.namco210.mirroring);
    }

    #[test]
    fn rambo1_switches_three_prg_windows_and_full_chr_modes() {
        let rom = rambo1_test_rom(64);
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        assert!(cart.prg_ram.is_empty());
        for (register, value) in [(6, 2), (7, 3), (15, 4)] {
            cart.write_mapper(0x8000, register);
            cart.write_mapper(0x8001, value);
        }
        assert_eq!(cart.read_prg(0x8000), 2);
        assert_eq!(cart.read_prg(0xa000), 3);
        assert_eq!(cart.read_prg(0xc000), 4);
        assert_eq!(cart.read_prg(0xe000), 31);
        cart.write_mapper(0x8000, 0x46);
        cart.write_mapper(0x8001, 5);
        assert_eq!(cart.read_prg(0x8000), 4);
        assert_eq!(cart.read_prg(0xc000), 5);

        for (register, value) in [(0, 2), (1, 4), (2, 6), (3, 7), (4, 8), (5, 9)] {
            cart.write_mapper(0x8000, register);
            cart.write_mapper(0x8001, value);
        }
        cart.sync_ppu(&mut ppu);
        let expected = [2, 3, 4, 5, 6, 7, 8, 9];
        for (slot, value) in expected.into_iter().enumerate() {
            assert_eq!(ppu.read_vram((slot * 0x400) as u16), value);
        }

        for (register, value) in [
            (0, 10),
            (8, 11),
            (1, 12),
            (9, 13),
            (2, 14),
            (3, 15),
            (4, 16),
            (5, 17),
        ] {
            cart.write_mapper(0x8000, 0x20 | register);
            cart.write_mapper(0x8001, value);
        }
        cart.sync_ppu(&mut ppu);
        for (slot, value) in [10, 11, 12, 13, 14, 15, 16, 17].into_iter().enumerate() {
            assert_eq!(ppu.read_vram((slot * 0x400) as u16), value);
        }
        cart.write_mapper(0x8000, 0xa0);
        cart.sync_ppu(&mut ppu);
        for (slot, value) in [14, 15, 16, 17, 10, 11, 12, 13].into_iter().enumerate() {
            assert_eq!(ppu.read_vram((slot * 0x400) as u16), value);
        }
    }

    #[test]
    fn rambo1_mapper158_routes_ciram_from_chr_a17() {
        let rom = rambo1_test_rom(158);
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        for (register, value) in [(0, 0x80), (8, 0x00), (1, 0x81), (9, 0x01)] {
            cart.write_mapper(0x8000, 0x20 | register);
            cart.write_mapper(0x8001, value);
        }
        cart.sync_ppu(&mut ppu);
        ppu.nametable[0] = 0x11;
        ppu.nametable[0x400] = 0x22;
        assert_eq!(ppu.read_vram(0x2000), 0x22);
        assert_eq!(ppu.read_vram(0x2400), 0x11);
        assert_eq!(ppu.read_vram(0x2800), 0x22);
        assert_eq!(ppu.read_vram(0x2c00), 0x11);
        cart.write_mapper(0xa000, 1);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.read_vram(0x2000), 0x22);
        assert_eq!(ppu.read_vram(0x2400), 0x11);
    }

    #[test]
    fn rambo1_cycle_and_scanline_irq_paths_are_delayed() {
        let rom = rambo1_test_rom(64);
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();
        cart.write_mapper(0xc000, 1);
        cart.write_mapper(0xc001, 1);
        cart.write_mapper(0xe001, 0);
        for _ in 0..8 {
            cart.tick_cpu_cycle();
        }
        assert!(!cart.irq_pending());
        cart.tick_cpu_cycle();
        assert!(cart.irq_pending());
        cart.write_mapper(0xe000, 0);
        assert!(!cart.irq_pending());

        cart.write_mapper(0xc000, 1);
        cart.write_mapper(0xc001, 0);
        cart.write_mapper(0xe001, 0);
        for _ in 0..16 {
            cart.observe_ppu_a12(false);
        }
        cart.observe_ppu_a12(true);
        for _ in 0..16 {
            cart.observe_ppu_a12(false);
        }
        cart.observe_ppu_a12(true);
        assert!(!cart.irq_pending());
        cart.tick_cpu_cycle();
        assert!(!cart.irq_pending());
        cart.tick_cpu_cycle();
        assert!(cart.irq_pending());
        cart.write_mapper(0xe000, 0);
        assert!(!cart.irq_pending());
    }

    #[test]
    fn rambo1_mapper_state_round_trip_preserves_banks_and_irq_phase() {
        let rom = rambo1_test_rom(64);
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();
        cart.write_mapper(0x8000, 0x28);
        cart.write_mapper(0x8001, 0x81);
        cart.write_mapper(0x8000, 0x4f);
        cart.write_mapper(0x8001, 7);
        cart.write_mapper(0xa000, 1);
        cart.write_mapper(0xc000, 3);
        cart.write_mapper(0xc001, 1);
        cart.write_mapper(0xe001, 0);
        for _ in 0..5 {
            cart.tick_cpu_cycle();
        }
        cart.observe_ppu_a12(false);

        let mut writer = StateWriter::new(PlatformId::Nes, 99);
        save_mapper_state(&mut writer, &cart);
        let bytes = writer.finish();
        let (mut restored, _) = Cartridge::parse(&rom).unwrap();
        let mut reader = StateReader::new(&bytes, PlatformId::Nes, 99).unwrap();
        load_mapper_state(&mut reader, &mut restored).unwrap();
        reader.finish().unwrap();

        assert_eq!(restored.rambo1.bank_select, cart.rambo1.bank_select);
        assert_eq!(restored.rambo1.regs, cart.rambo1.regs);
        assert_eq!(
            restored.rambo1.mirror_horizontal,
            cart.rambo1.mirror_horizontal
        );
        assert_eq!(restored.rambo1.irq_latch, cart.rambo1.irq_latch);
        assert_eq!(restored.rambo1.irq_counter, cart.rambo1.irq_counter);
        assert_eq!(restored.rambo1.irq_cycle_mode, cart.rambo1.irq_cycle_mode);
        assert_eq!(restored.rambo1.irq_prescaler, cart.rambo1.irq_prescaler);
        assert_eq!(restored.rambo1.irq_delay, cart.rambo1.irq_delay);
        assert_eq!(restored.rambo1.a12_low_cycles, cart.rambo1.a12_low_cycles);
    }

    #[test]
    fn mmc5_prg_modes_ram_protection_and_multiplier_work() {
        let rom = mmc5_test_rom(1);
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();
        assert_eq!(cart.read_prg(0xe000), 15);
        cart.write_mmc5_register(0x5100, 2);
        cart.write_mmc5_register(0x5115, 0x82);
        cart.write_mmc5_register(0x5116, 0x84);
        cart.write_mmc5_register(0x5117, 0x87);
        assert_eq!(cart.read_prg(0x8000), 2);
        assert_eq!(cart.read_prg(0xa000), 3);
        assert_eq!(cart.read_prg(0xc000), 4);
        assert_eq!(cart.read_prg(0xe000), 7);

        cart.write_mmc5_register(0x5102, 2);
        cart.write_mmc5_register(0x5103, 1);
        cart.write_mmc5_register(0x5113, 3);
        cart.write_ram(0x6000, 0x5a);
        assert_eq!(cart.read_ram(0x6000), 0x5a);
        cart.write_mmc5_register(0x5100, 3);
        cart.write_mmc5_register(0x5114, 0x00);
        cart.write_mapper(0x8000, 0xa5);
        assert_eq!(cart.read_prg(0x8000), 0xa5);
        cart.write_mmc5_register(0x5114, 0x80);
        assert_eq!(cart.read_prg(0x8000), 0);

        cart.write_mmc5_register(0x5205, 13);
        cart.write_mmc5_register(0x5206, 17);
        assert_eq!(cart.read_expansion_mapper(0x5205), Some(221));
        assert_eq!(cart.read_expansion_mapper(0x5206), Some(0));
    }

    #[test]
    fn mmc5_chr_modes_separate_sprite_and_background_banks() {
        let rom = mmc5_test_rom(16);
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        cart.write_mmc5_register(0x5101, 3);
        cart.write_mmc5_register(0x5120, 1);
        cart.write_mmc5_register(0x5128, 9);
        ppu.write_register(0x2000, 0x20);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.read_sprite_pattern(0), 1);
        assert_eq!(ppu.read_background_pattern(0), 9);

        ppu.write_register(0x2000, 0x00);
        assert_eq!(ppu.read_background_pattern(0), 1);

        ppu.write_register(0x2000, 0x20);
        cart.write_mmc5_register(0x5101, 1);
        cart.write_mmc5_register(0x5123, 2);
        cart.write_mmc5_register(0x512b, 3);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.read_sprite_pattern(0), 8);
        assert_eq!(ppu.read_background_pattern(0), 12);
    }

    #[test]
    fn mmc5_routes_ciram_exram_fill_and_extended_attributes() {
        let rom = mmc5_test_rom(16);
        let (cart, ppu) = Cartridge::parse(&rom).unwrap();
        let mut bus = NesBus::new(cart, ppu);
        bus.write8(0x5104, 2);
        bus.write8(0x5c00, 0x42);
        assert_eq!(bus.read8(0x5c00), 0x42);
        bus.write8(0x5104, 0);
        bus.write8(0x5105, 0xe4);
        bus.write8(0x5106, 0x77);
        bus.write8(0x5107, 2);
        bus.ppu.write_nametable(0x2000, 0x11);
        bus.ppu.write_nametable(0x2400, 0x22);
        assert_eq!(bus.ppu.read_nametable(0x2000), 0x11);
        assert_eq!(bus.ppu.read_nametable(0x2400), 0x22);
        assert_eq!(bus.ppu.read_nametable(0x2800), 0x42);
        assert_eq!(bus.ppu.read_nametable(0x2c00), 0x77);
        assert_eq!(bus.ppu.read_nametable(0x2fc0), 0xaa);

        bus.write8(0x5104, 2);
        bus.write8(0x5c00, 0x83);
        bus.write8(0x5104, 1);
        bus.write8(0x5105, 0);
        bus.ppu.mask = 0x08;
        bus.ppu.v = 0;
        bus.ppu.cycle = 1;
        bus.ppu.scanline = 0;
        bus.ppu.fetch_background_tile();
        bus.ppu.fetch_background_attribute();
        assert_eq!(bus.ppu.bg_next_attr, 2);
        assert_eq!(bus.ppu.read_background_pattern(0), 12);
    }

    #[test]
    fn mmc5_vertical_split_uses_exram_scroll_and_split_chr_bank() {
        let rom = mmc5_test_rom(16);
        let (cart, ppu) = Cartridge::parse(&rom).unwrap();
        let mut bus = NesBus::new(cart, ppu);
        bus.write8(0x5104, 2);
        bus.write8(0x5c20, 6);
        bus.write8(0x5fc0, 3);
        bus.write8(0x5104, 0);
        bus.write8(0x5200, 0x88);
        bus.write8(0x5201, 5);
        bus.write8(0x5202, 4);
        bus.ppu.mask = 0x08;
        bus.ppu.scanline = 3;
        bus.ppu.cycle = 1;
        bus.ppu.fetch_background_tile();
        bus.ppu.fetch_background_attribute();
        assert!(bus.ppu.bg_next_split);
        assert_eq!(bus.ppu.bg_next_tile, 6);
        assert_eq!(bus.ppu.bg_next_split_row, 0);
        assert_eq!(bus.ppu.bg_next_attr, 3);
        assert_eq!(bus.ppu.read_background_pattern(0), 16);
    }

    #[test]
    fn mmc5_scanline_irq_status_and_acknowledge_are_deterministic() {
        let rom = mmc5_test_rom(1);
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();
        cart.write_mmc5_register(0x5203, 2);
        cart.write_mmc5_register(0x5204, 0x80);
        cart.observe_mmc5_ppu_dot(0, 4, true);
        assert_eq!(cart.mmc5.irq_counter, 0);
        cart.observe_mmc5_ppu_dot(1, 4, true);
        assert!(!cart.irq_pending());
        cart.observe_mmc5_ppu_dot(2, 4, true);
        assert!(cart.irq_pending());
        assert_eq!(cart.read_expansion_mapper(0x5204), Some(0xc0));
        assert!(!cart.mmc5.irq_pending);
        assert!(cart.mmc5.in_frame);
        cart.reset_mmc5_scanline_counter();
        assert!(!cart.mmc5.in_frame);
        assert_eq!(cart.mmc5.irq_counter, 0);
    }

    #[test]
    fn mmc5_audio_pulses_and_pcm_irq_reach_expansion_output() {
        let rom = mmc5_test_rom(1);
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();
        cart.write_mmc5_register(0x5015, 1);
        cart.write_mmc5_register(0x5000, 0xdf);
        cart.write_mmc5_register(0x5002, 8);
        cart.write_mmc5_register(0x5003, 0x08);
        assert!(cart.expansion_audio_output().abs() > 0.01);
        cart.write_mmc5_register(0x5011, 0x80);
        assert_ne!(cart.mmc5.pcm, 0);
        cart.write_mmc5_register(0x5010, 0x80);
        cart.write_mmc5_register(0x5011, 0);
        assert!(cart.irq_pending());
        assert_eq!(cart.read_expansion_mapper(0x5010), Some(0x80));
        assert!(!cart.mmc5.pcm_irq_pending);
    }

    #[test]
    fn mmc5_machine_state_round_trip_preserves_ram_exram_and_pipeline() {
        let mut rom = mmc5_test_rom(1);
        let vectors = 16 + 8 * 0x4000 - 6;
        rom[vectors..vectors + 6].copy_from_slice(&[0x00, 0x80, 0x00, 0x80, 0x00, 0x80]);
        let mut machine = NesMachine::from_rom(&rom).unwrap();
        machine.bus.write8(0x5102, 2);
        machine.bus.write8(0x5103, 1);
        machine.bus.write8(0x5113, 2);
        machine.bus.write8(0x6000, 0xa5);
        machine.bus.write8(0x5104, 2);
        machine.bus.write8(0x5c00, 0x5a);
        machine.bus.write8(0x5205, 13);
        machine.bus.write8(0x5206, 17);
        machine.bus.ppu.bg_next_ex_attr = 0x83;
        machine.bus.ppu.bg_next_split = true;
        machine.bus.ppu.bg_next_split_row = 6;
        let state = machine.save_state().unwrap();

        machine.bus.write8(0x5113, 2);
        machine.bus.write8(0x6000, 0);
        machine.bus.write8(0x5c00, 0);
        machine.bus.ppu.bg_next_ex_attr = 0;
        machine.load_state(&state).unwrap();
        assert_eq!(machine.bus.read8(0x6000), 0xa5);
        assert_eq!(machine.bus.read8(0x5c00), 0x5a);
        assert_eq!(machine.bus.cartridge.mmc5.multiplicand, 13);
        assert_eq!(machine.bus.cartridge.mmc5.multiplier, 17);
        assert_eq!(machine.bus.ppu.bg_next_ex_attr, 0x83);
        assert!(machine.bus.ppu.bg_next_split);
        assert_eq!(machine.bus.ppu.bg_next_split_row, 6);
    }

    #[test]
    fn mmc1_surom_and_sxrom_select_outer_prg_and_large_ram_banks() {
        let mut rom = mapper_rom(1, 32, 0);
        rom[8] = 4;
        for bank in 0..32usize {
            rom[16 + bank * 0x4000] = bank as u8;
        }
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();
        mmc1_write(&mut cart, 0xa000, 0x10);
        mmc1_write(&mut cart, 0xe000, 2);
        assert_eq!(cart.read_prg(0x8000), 18);
        assert_eq!(cart.read_prg(0xc000), 31);

        for (bank, value) in [(0x10, 0x31), (0x14, 0x42), (0x18, 0x53), (0x1c, 0x64)] {
            mmc1_write(&mut cart, 0xa000, bank);
            cart.write_ram(0x6000, value);
        }
        for (bank, value) in [(0x10, 0x31), (0x14, 0x42), (0x18, 0x53), (0x1c, 0x64)] {
            mmc1_write(&mut cart, 0xa000, bank);
            assert_eq!(cart.read_ram(0x6000), value);
        }
    }

    #[test]
    fn mmc1_sorom_and_szrom_use_their_board_specific_ram_select_line() {
        let mut sorom = mapper_rom(1, 16, 0);
        sorom[8] = 2;
        let (mut sorom, _) = Cartridge::parse(&sorom).unwrap();
        mmc1_write(&mut sorom, 0xa000, 0x00);
        sorom.write_ram(0x6000, 0x11);
        mmc1_write(&mut sorom, 0xa000, 0x08);
        sorom.write_ram(0x6000, 0x22);
        mmc1_write(&mut sorom, 0xa000, 0x00);
        assert_eq!(sorom.read_ram(0x6000), 0x11);
        mmc1_write(&mut sorom, 0xa000, 0x08);
        assert_eq!(sorom.read_ram(0x6000), 0x22);

        let mut szrom = mapper_rom(1, 16, 2);
        szrom[8] = 2;
        let (mut szrom, _) = Cartridge::parse(&szrom).unwrap();
        mmc1_write(&mut szrom, 0xa000, 0x00);
        szrom.write_ram(0x6000, 0x33);
        mmc1_write(&mut szrom, 0xa000, 0x10);
        szrom.write_ram(0x6000, 0x44);
        mmc1_write(&mut szrom, 0xa000, 0x00);
        assert_eq!(szrom.read_ram(0x6000), 0x33);
        mmc1_write(&mut szrom, 0xa000, 0x10);
        assert_eq!(szrom.read_ram(0x6000), 0x44);
    }

    #[test]
    fn mmc1a_keeps_ram_enabled_and_routes_prg_bit_three_to_a17() {
        let mut rom = mapper_rom(155, 16, 0);
        for bank in 0..16usize {
            rom[16 + bank * 0x4000] = bank as u8;
        }
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();
        mmc1_write(&mut cart, 0xe000, 0x1b);
        assert_eq!(cart.read_prg(0x8000), 11);
        assert_eq!(cart.read_prg(0xc000), 15);
        cart.write_ram(0x6000, 0x5a);
        assert_eq!(cart.read_ram(0x6000), 0x5a);
    }

    #[test]
    fn mmc1_submapper_five_is_unbanked_and_submapper_seven_has_fixed_mirroring() {
        let mut fixed = nes2_mapper_rom(1, 5, 2, 2);
        fixed[16] = 0x12;
        fixed[16 + 0x4000] = 0x34;
        let (mut fixed, _) = Cartridge::parse(&fixed).unwrap();
        mmc1_write(&mut fixed, 0x8000, 0x08);
        mmc1_write(&mut fixed, 0xe000, 0);
        assert_eq!(fixed.read_prg(0x8000), 0x12);
        assert_eq!(fixed.read_prg(0xc000), 0x34);

        let mut hardwired = nes2_mapper_rom(1, 7, 4, 2);
        hardwired[6] &= !1;
        let (mut hardwired, mut ppu) = Cartridge::parse(&hardwired).unwrap();
        mmc1_write(&mut hardwired, 0x8000, 0x02);
        hardwired.sync_ppu(&mut ppu);
        assert_eq!(ppu.mirroring, Mirroring::Horizontal);
    }

    #[test]
    fn mmc1_switches_prg_and_chr_banks() {
        let mut rom = mapper_rom(1, 4, 2);
        for bank in 0..4usize {
            rom[16 + bank * 0x4000] = (0x10 + bank) as u8;
        }
        let chr_start = 16 + 4 * 0x4000;
        for bank in 0..4usize {
            rom[chr_start + bank * 0x1000] = (0x40 + bank) as u8;
        }
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        mmc1_write(&mut cart, 0xe000, 2);
        assert_eq!(cart.read_prg(0x8000), 0x12);
        assert_eq!(cart.read_prg(0xc000), 0x13);
        mmc1_write(&mut cart, 0x8000, 0x1c);
        mmc1_write(&mut cart, 0xa000, 2);
        mmc1_write(&mut cart, 0xc000, 3);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.read_vram(0x0000), 0x42);
        assert_eq!(ppu.read_vram(0x1000), 0x43);
        assert_eq!(ppu.mirroring, Mirroring::SingleScreen0);
    }

    #[test]
    fn nina_03_and_sachen_146_switch_prg_chr_through_expansion_space() {
        for mapper in [79, 146] {
            let mut rom = mapper_rom(mapper, 4, 8);
            for bank in 0..2usize {
                rom[16 + bank * 0x8000] = (0x20 + bank) as u8;
            }
            let chr_start = 16 + 4 * 0x4000;
            for bank in 0..8usize {
                rom[chr_start + bank * 0x2000] = (0x40 + bank) as u8;
            }
            let (cart, ppu) = Cartridge::parse(&rom).unwrap();
            let mut bus = NesBus::new(cart, ppu);
            bus.write8(0x4100, 0x0d);
            assert_eq!(bus.cartridge.read_prg(0x8000), 0x21);
            assert_eq!(bus.ppu.read_vram(0), 0x45);
        }
    }

    #[test]
    fn hes_ntd8_switches_extended_prg_chr_and_mirroring() {
        let mut rom = mapper_rom(113, 16, 16);
        for bank in 0..8usize {
            rom[16 + bank * 0x8000] = (0x20 + bank) as u8;
        }
        let chr_start = 16 + 16 * 0x4000;
        for bank in 0..16usize {
            rom[chr_start + bank * 0x2000] = (0x40 + bank) as u8;
        }
        let (cart, ppu) = Cartridge::parse(&rom).unwrap();
        let mut bus = NesBus::new(cart, ppu);
        bus.write8(0x4100, 0xe9);
        assert_eq!(bus.cartridge.read_prg(0x8000), 0x25);
        assert_eq!(bus.ppu.read_vram(0), 0x49);
        assert_eq!(bus.ppu.mirroring, Mirroring::Vertical);
    }

    #[test]
    fn camerica_quattro_selects_outer_and_inner_banks() {
        let mut rom = mapper_rom(232, 16, 0);
        for bank in 0..16usize {
            rom[16 + bank * 0x4000] = bank as u8;
        }
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();
        assert_eq!(cart.read_prg(0x8000), 12);
        assert_eq!(cart.read_prg(0xc000), 15);
        cart.write_mapper(0x8000, 0x08);
        cart.write_mapper(0xc000, 2);
        assert_eq!(cart.read_prg(0x8000), 6);
        assert_eq!(cart.read_prg(0xc000), 7);

        let mut aladdin = nes2_mapper_rom(232, 1, 16, 0);
        for bank in 0..16usize {
            aladdin[16 + bank * 0x4000] = bank as u8;
        }
        let (mut aladdin, _) = Cartridge::parse(&aladdin).unwrap();
        aladdin.write_mapper(0x8000, 0x08);
        aladdin.write_mapper(0xc000, 2);
        assert_eq!(aladdin.read_prg(0x8000), 10);
        assert_eq!(aladdin.read_prg(0xc000), 11);
    }

    #[test]
    fn irem_h3001_banks_prg_chr_and_controls_mirroring() {
        let mut rom = mapper_rom(65, 16, 32);
        for bank in 0..32usize {
            rom[16 + bank * 0x2000..16 + (bank + 1) * 0x2000].fill(bank as u8);
        }
        let chr_start = 16 + 16 * 0x4000;
        for bank in 0..256usize {
            rom[chr_start + bank * 0x400..chr_start + (bank + 1) * 0x400].fill(bank as u8);
        }
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        assert_eq!(cart.read_prg(0x8000), 0);
        assert_eq!(cart.read_prg(0xa000), 1);
        assert_eq!(cart.read_prg(0xc000), 30);
        assert_eq!(cart.read_prg(0xe000), 31);

        cart.write_mapper(0x8fff, 3);
        cart.write_mapper(0xa123, 4);
        cart.write_mapper(0xc000, 9);
        assert_eq!(cart.read_prg(0x8000), 3);
        assert_eq!(cart.read_prg(0xa000), 4);
        assert_eq!(cart.read_prg(0xc000), 30);
        cart.write_mapper(0x9000, 0x80);
        assert_eq!(cart.read_prg(0x8000), 30);
        assert_eq!(cart.read_prg(0xc000), 3);

        for slot in 0..8u16 {
            cart.write_mapper(0xb100 | slot, 0x20 + slot as u8);
        }
        cart.sync_ppu(&mut ppu);
        for slot in 0..8usize {
            assert_eq!(ppu.read_vram((slot * 0x400) as u16), 0x20 + slot as u8);
        }
        cart.write_mapper(0x9001, 0x80);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.mirroring, Mirroring::Horizontal);
        cart.write_mapper(0x9001, 0x40);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.mirroring, Mirroring::SingleScreen0);
    }

    #[test]
    fn irem_h3001_irq_reloads_counts_once_and_acknowledges() {
        let rom = mapper_rom(65, 8, 8);
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();
        cart.write_mapper(0x9005, 0x00);
        cart.write_mapper(0x9006, 0x03);
        cart.write_mapper(0x9004, 0);
        cart.write_mapper(0x9003, 0x80);
        assert_eq!(cart.irem_h3001.irq_counter, 3);
        cart.tick_cpu_cycle();
        cart.tick_cpu_cycle();
        assert!(!cart.irq_pending());
        cart.tick_cpu_cycle();
        assert!(cart.irq_pending());
        assert!(!cart.irem_h3001.irq_enabled);
        assert_eq!(cart.irem_h3001.irq_counter, 0);

        cart.write_mapper(0x9003, 0);
        assert!(!cart.irq_pending());
        cart.write_mapper(0x9004, 0);
        assert_eq!(cart.irem_h3001.irq_counter, 3);
        cart.write_mapper(0x9003, 0x80);
        cart.tick_cpu_cycle();
        assert_eq!(cart.irem_h3001.irq_counter, 2);
    }

    #[test]
    fn irem_h3001_mapper_state_round_trip_preserves_banks_and_irq_phase() {
        let rom = mapper_rom(65, 8, 8);
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();
        cart.write_mapper(0x8000, 7);
        cart.write_mapper(0xa000, 8);
        cart.write_mapper(0xb003, 11);
        cart.write_mapper(0x9000, 0x80);
        cart.write_mapper(0x9001, 0x40);
        cart.write_mapper(0x9005, 0x12);
        cart.write_mapper(0x9006, 0x34);
        cart.write_mapper(0x9004, 0);
        cart.write_mapper(0x9003, 0x80);
        cart.tick_cpu_cycle();

        let mut writer = StateWriter::new(PlatformId::Nes, 99);
        save_mapper_state(&mut writer, &cart);
        let bytes = writer.finish();
        let (mut restored, _) = Cartridge::parse(&rom).unwrap();
        let mut reader = StateReader::new(&bytes, PlatformId::Nes, 99).unwrap();
        load_mapper_state(&mut reader, &mut restored).unwrap();
        reader.finish().unwrap();

        assert_eq!(restored.irem_h3001.prg, cart.irem_h3001.prg);
        assert_eq!(restored.irem_h3001.chr, cart.irem_h3001.chr);
        assert_eq!(restored.irem_h3001.swap_prg, cart.irem_h3001.swap_prg);
        assert_eq!(restored.irem_h3001.mirroring, cart.irem_h3001.mirroring);
        assert_eq!(restored.irem_h3001.irq_reload, cart.irem_h3001.irq_reload);
        assert_eq!(restored.irem_h3001.irq_counter, cart.irem_h3001.irq_counter);
        assert_eq!(restored.irem_h3001.irq_enabled, cart.irem_h3001.irq_enabled);
        assert_eq!(restored.irem_h3001.irq_pending, cart.irem_h3001.irq_pending);
    }

    #[test]
    fn irem_g101_banks_prg_chr_mirroring_and_wram() {
        let mut rom = mapper_rom(32, 8, 16);
        for bank in 0..16usize {
            rom[16 + bank * 0x2000] = (0x20 + bank) as u8;
        }
        let chr_start = 16 + 8 * 0x4000;
        for bank in 0..128usize {
            rom[chr_start + bank * 0x0400..chr_start + (bank + 1) * 0x0400]
                .fill((0x40 + bank) as u8);
        }
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        cart.write_mapper(0x8000, 3);
        cart.write_mapper(0xa000, 4);
        assert_eq!(cart.read_prg(0x8000), 0x23);
        assert_eq!(cart.read_prg(0xa000), 0x24);
        assert_eq!(cart.read_prg(0xc000), 0x2e);
        assert_eq!(cart.read_prg(0xe000), 0x2f);

        cart.write_mapper(0x9000, 3);
        assert_eq!(cart.read_prg(0x8000), 0x2e);
        assert_eq!(cart.read_prg(0xc000), 0x23);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.mirroring, Mirroring::Horizontal);

        for slot in 0..8usize {
            cart.write_mapper(0xb000 + slot as u16, (slot + 5) as u8);
        }
        cart.sync_ppu(&mut ppu);
        for slot in 0..8usize {
            assert_eq!(ppu.read_vram((slot * 0x400) as u16), 0x45 + slot as u8);
        }
        cart.write_ram(0x6000, 0x5a);
        assert_eq!(cart.read_ram(0x6000), 0x5a);
    }

    #[test]
    fn irem_g101_major_league_submapper_ignores_control_register() {
        let mut rom = nes2_mapper_rom(32, 1, 8, 16);
        for bank in 0..16usize {
            rom[16 + bank * 0x2000] = (0x30 + bank) as u8;
        }
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        cart.write_mapper(0x8000, 3);
        cart.write_mapper(0x9000, 3);
        assert_eq!(cart.read_prg(0x8000), 0x33);
        assert_eq!(cart.read_prg(0xc000), 0x3e);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.mirroring, Mirroring::SingleScreen1);
        assert!(!cart.irem_g101.swap_prg);
        assert!(!cart.irem_g101.mirror_horizontal);
    }

    #[test]
    fn irem_g101_mapper_state_round_trip_preserves_banks() {
        let rom = mapper_rom(32, 8, 16);
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();
        cart.write_mapper(0x8000, 7);
        cart.write_mapper(0xa000, 8);
        cart.write_mapper(0x9000, 3);
        for slot in 0..8usize {
            cart.write_mapper(0xb000 + slot as u16, (slot * 3 + 1) as u8);
        }
        let mut writer = StateWriter::new(PlatformId::Nes, 99);
        save_mapper_state(&mut writer, &cart);
        let bytes = writer.finish();
        let (mut restored, _) = Cartridge::parse(&rom).unwrap();
        let mut reader = StateReader::new(&bytes, PlatformId::Nes, 99).unwrap();
        load_mapper_state(&mut reader, &mut restored).unwrap();
        reader.finish().unwrap();
        assert_eq!(restored.irem_g101.prg, cart.irem_g101.prg);
        assert_eq!(restored.irem_g101.chr, cart.irem_g101.chr);
        assert_eq!(restored.irem_g101.swap_prg, cart.irem_g101.swap_prg);
        assert_eq!(
            restored.irem_g101.mirror_horizontal,
            cart.irem_g101.mirror_horizontal
        );
    }

    #[test]
    fn taito_tc0190_banks_prg_chr_and_controls_mirroring() {
        let mut rom = mapper_rom(33, 32, 64);
        for bank in 0..64usize {
            rom[16 + bank * 0x2000] = bank as u8;
        }
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        cart.write_mapper(0x8000, 0x45);
        cart.write_mapper(0x8001, 7);
        cart.write_mapper(0x8002, 0x80);
        cart.write_mapper(0x8003, 0x7f);
        for (address, value) in [(0xa000, 3), (0xa001, 4), (0xa002, 5), (0xa003, 6)] {
            cart.write_mapper(address, value);
        }
        assert_eq!(cart.read_prg(0x8000), 5);
        assert_eq!(cart.read_prg(0xa000), 7);
        assert_eq!(cart.read_prg(0xc000), 62);
        assert_eq!(cart.read_prg(0xe000), 63);
        assert_eq!(cart.chr_map(), [256, 257, 254, 255, 3, 4, 5, 6]);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.mirroring, Mirroring::Horizontal);
    }

    #[test]
    fn taito_tc0190_mapper_state_round_trip_preserves_registers() {
        let rom = mapper_rom(33, 8, 8);
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();
        for (address, value) in [
            (0x8000, 0x43),
            (0x8001, 4),
            (0x8002, 5),
            (0x8003, 6),
            (0xa000, 7),
            (0xa001, 8),
            (0xa002, 9),
            (0xa003, 10),
        ] {
            cart.write_mapper(address, value);
        }
        let mut writer = StateWriter::new(PlatformId::Nes, 99);
        save_mapper_state(&mut writer, &cart);
        let bytes = writer.finish();
        let (mut restored, _) = Cartridge::parse(&rom).unwrap();
        let mut reader = StateReader::new(&bytes, PlatformId::Nes, 99).unwrap();
        load_mapper_state(&mut reader, &mut restored).unwrap();
        reader.finish().unwrap();
        assert_eq!(restored.taito_tc0190.prg, cart.taito_tc0190.prg);
        assert_eq!(restored.taito_tc0190.chr_2k, cart.taito_tc0190.chr_2k);
        assert_eq!(restored.taito_tc0190.chr_1k, cart.taito_tc0190.chr_1k);
        assert_eq!(
            restored.taito_tc0190.mirror_horizontal,
            cart.taito_tc0190.mirror_horizontal
        );
    }

    #[test]
    fn taito_tc0690_mapper48_banks_prg_chr_and_separates_mirroring_register() {
        let mut rom = mapper_rom(48, 32, 64);
        for bank in 0..64usize {
            rom[16 + bank * 0x2000] = bank as u8;
        }
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        assert!(cart.prg_ram.is_empty());

        cart.write_mapper(0x8000, 0x45);
        cart.write_mapper(0x8001, 7);
        cart.write_mapper(0x8002, 0x81);
        cart.write_mapper(0x8003, 0x7f);
        for (address, value) in [(0xa000, 3), (0xa001, 4), (0xa002, 5), (0xa003, 6)] {
            cart.write_mapper(address, value);
        }
        assert_eq!(cart.read_prg(0x8000), 5);
        assert_eq!(cart.read_prg(0xa000), 7);
        assert_eq!(cart.read_prg(0xc000), 62);
        assert_eq!(cart.read_prg(0xe000), 63);
        assert_eq!(cart.chr_map(), [258, 259, 254, 255, 3, 4, 5, 6]);

        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.mirroring, Mirroring::Horizontal);
        cart.write_mapper(0xe000, 0x00);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.mirroring, Mirroring::Vertical);
        cart.write_mapper(0xe000, 0x40);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.mirroring, Mirroring::Horizontal);
    }

    #[test]
    fn taito_tc0690_mapper48_inverts_reload_and_delays_scanline_irq_four_cpu_cycles() {
        let rom = mapper_rom(48, 8, 8);
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();
        cart.write_mapper(0xc000, 0xfe);
        assert_eq!(cart.taito_tc0190.irq_latch, 1);
        cart.write_mapper(0xc001, 0);
        cart.write_mapper(0xc002, 0);

        for _ in 0..8 {
            cart.observe_ppu_a12(false);
        }
        cart.observe_ppu_a12(true);
        assert_eq!(cart.taito_tc0190.irq_counter, 1);
        assert!(!cart.irq_pending());

        for _ in 0..8 {
            cart.observe_ppu_a12(false);
        }
        cart.observe_ppu_a12(true);
        assert_eq!(cart.taito_tc0190.irq_counter, 0);
        assert_eq!(cart.taito_tc0190.irq_delay, 4);
        for remaining in (1..=3).rev() {
            cart.tick_cpu_cycle();
            assert_eq!(cart.taito_tc0190.irq_delay, remaining);
            assert!(!cart.irq_pending());
        }
        cart.tick_cpu_cycle();
        assert!(cart.irq_pending());

        cart.write_mapper(0xc003, 0);
        assert!(!cart.taito_tc0190.irq_enabled);
        assert!(!cart.irq_pending());
        assert_eq!(cart.taito_tc0190.irq_delay, 0);
    }

    #[test]
    fn taito_tc0690_mapper48_state_round_trip_preserves_irq_phase() {
        let rom = mapper_rom(48, 8, 8);
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();
        cart.write_mapper(0x8000, 3);
        cart.write_mapper(0x8002, 5);
        cart.write_mapper(0xe000, 0x40);
        cart.write_mapper(0xc000, 0xfe);
        cart.write_mapper(0xc001, 0);
        cart.write_mapper(0xc002, 0);
        for _ in 0..8 {
            cart.observe_ppu_a12(false);
        }
        cart.observe_ppu_a12(true);
        for _ in 0..8 {
            cart.observe_ppu_a12(false);
        }
        cart.observe_ppu_a12(true);
        cart.tick_cpu_cycle();
        cart.tick_cpu_cycle();
        assert_eq!(cart.taito_tc0190.irq_delay, 2);

        let mut writer = StateWriter::new(PlatformId::Nes, 99);
        save_mapper_state(&mut writer, &cart);
        let bytes = writer.finish();
        let (mut restored, _) = Cartridge::parse(&rom).unwrap();
        let mut reader = StateReader::new(&bytes, PlatformId::Nes, 99).unwrap();
        load_mapper_state(&mut reader, &mut restored).unwrap();
        reader.finish().unwrap();

        assert_eq!(restored.taito_tc0190.prg, cart.taito_tc0190.prg);
        assert_eq!(restored.taito_tc0190.chr_2k, cart.taito_tc0190.chr_2k);
        assert_eq!(restored.taito_tc0190.irq_latch, 1);
        assert_eq!(restored.taito_tc0190.irq_counter, 0);
        assert_eq!(restored.taito_tc0190.irq_delay, 2);
        assert!(restored.taito_tc0190.irq_enabled);
        restored.tick_cpu_cycle();
        assert!(!restored.irq_pending());
        restored.tick_cpu_cycle();
        assert!(restored.irq_pending());
    }

    #[test]
    fn taito_x1005_mapper80_banks_prg_chr_mirroring_and_internal_ram() {
        let mut rom = mapper_rom(80, 16, 32);
        for bank in 0..32usize {
            rom[16 + bank * 0x2000] = bank as u8;
        }
        let chr_start = 16 + 16 * 0x4000;
        for bank in 0..256usize {
            rom[chr_start + bank * 0x400..chr_start + (bank + 1) * 0x400].fill(bank as u8);
        }
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        assert_eq!(cart.prg_ram.len(), 128);
        assert_eq!(cart.read_ram(0x7f00), 0xff);

        for (address, value) in [
            (0x7e70, 0x07),
            (0x7ef1, 0x0a),
            (0x7ef2, 0x20),
            (0x7ef3, 0x21),
            (0x7ef4, 0x22),
            (0x7ef5, 0x23),
            (0x7e7a, 3),
            (0x7efc, 4),
            (0x7efe, 5),
            (0x7ef6, 1),
        ] {
            assert!(cart.write_low_mapper(address, value));
        }
        assert_eq!(cart.read_prg(0x8000), 3);
        assert_eq!(cart.read_prg(0xa000), 4);
        assert_eq!(cart.read_prg(0xc000), 5);
        assert_eq!(cart.read_prg(0xe000), 31);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.read_vram(0x0000), 6);
        assert_eq!(ppu.read_vram(0x0400), 7);
        assert_eq!(ppu.read_vram(0x0800), 10);
        assert_eq!(ppu.read_vram(0x1000), 0x20);
        assert_eq!(ppu.mirroring, Mirroring::Horizontal);
        assert!(!cart.write_low_mapper(0x7ef7, 0));

        assert!(cart.write_low_mapper(0x7ef8, 0xa3));
        cart.write_ram(0x7f00, 0x5a);
        assert_eq!(cart.read_ram(0x7f80), 0x5a);
        cart.write_ram(0x7fff, 0xc3);
        assert_eq!(cart.read_ram(0x7f7f), 0xc3);
        assert!(cart.write_low_mapper(0x7ef9, 0));
        assert_eq!(cart.read_ram(0x7f00), 0xff);
    }

    #[test]
    fn taito_x1005_mapper207_routes_chr_a17_to_ciram() {
        let mut rom = mapper_rom(207, 8, 16);
        let chr_start = 16 + 8 * 0x4000;
        for bank in 0..128usize {
            rom[chr_start + bank * 0x400..chr_start + (bank + 1) * 0x400].fill(bank as u8);
        }
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        for (address, value) in [(0x7ef0, 0x82), (0x7ef1, 0x04)] {
            assert!(cart.write_low_mapper(address, value));
        }
        assert!(!cart.write_low_mapper(0x7ef6, 1));
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.read_vram(0x0000), 2);
        assert_eq!(ppu.read_vram(0x0400), 3);
        assert_eq!(ppu.read_vram(0x0800), 4);
        assert_eq!(ppu.read_vram(0x0c00), 5);

        ppu.nametable[0] = 0x11;
        ppu.nametable[0x400] = 0x22;
        assert_eq!(ppu.read_vram(0x2000), 0x22);
        assert_eq!(ppu.read_vram(0x2400), 0x22);
        assert_eq!(ppu.read_vram(0x2800), 0x11);
        assert_eq!(ppu.read_vram(0x2c00), 0x11);
    }

    #[test]
    fn taito_x1005_state_and_battery_persistence_preserve_internal_ram() {
        let mut rom = mapper_rom(80, 8, 16);
        rom[6] |= 0x02;
        let mut machine = NesMachine::from_rom(&rom).unwrap();
        assert_eq!(machine.persistent_len(ResourceKind::Storage, 0), 128);
        machine.bus.write8(0x7e7a, 3);
        machine.bus.write8(0x7ef2, 9);
        machine.bus.write8(0x7ef6, 1);
        machine.bus.write8(0x7ef8, 0xa3);
        machine.bus.write8(0x7f00, 0x5a);
        let state = machine.save_state().unwrap();

        machine.bus.write8(0x7ef8, 0);
        machine.bus.write8(0x7f00, 0);
        machine.load_state(&state).unwrap();
        assert_eq!(machine.bus.cartridge.taito_x1005.prg[0], 3);
        assert_eq!(machine.bus.cartridge.taito_x1005.chr[2], 9);
        assert!(machine.bus.cartridge.taito_x1005.mirror_horizontal);
        assert!(machine.bus.cartridge.taito_x1005.ram_enabled);
        assert_eq!(machine.bus.read8(0x7f00), 0x5a);

        let mut persistent = [0u8; 128];
        machine
            .read_persistent(ResourceKind::Storage, 0, &mut persistent)
            .unwrap();
        assert_eq!(persistent[0], 0x5a);
    }

    #[test]
    fn taito_x1017_mapper82_banks_prg_chr_and_protected_ram() {
        let mut rom = mapper_rom(82, 8, 32);
        for bank in 0..16usize {
            rom[16 + bank * 0x2000] = bank as u8;
        }
        let chr_start = 16 + 8 * 0x4000;
        for bank in 0..256usize {
            rom[chr_start + bank * 0x400..chr_start + (bank + 1) * 0x400].fill(bank as u8);
        }
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        assert_eq!(cart.prg_ram.len(), 5 * 1024);
        for (address, value) in [
            (0x7ef0, 6),
            (0x7ef1, 10),
            (0x7ef2, 0x20),
            (0x7ef3, 0x21),
            (0x7ef4, 0x22),
            (0x7ef5, 0x23),
            (0x7efa, 3 << 2),
            (0x7efb, 7 << 2),
            (0x7efc, 14 << 2),
        ] {
            assert!(cart.write_low_mapper(address, value));
        }
        assert_eq!(cart.read_prg(0x8000), 3);
        assert_eq!(cart.read_prg(0xa000), 7);
        assert_eq!(cart.read_prg(0xc000), 14);
        assert_eq!(cart.read_prg(0xe000), 15);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.read_vram(0x0000), 6);
        assert_eq!(ppu.read_vram(0x0400), 7);
        assert_eq!(ppu.read_vram(0x1000), 0x20);

        assert!(cart.write_low_mapper(0x7ef6, 0x03));
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.mirroring, Mirroring::Vertical);
        assert_eq!(ppu.read_vram(0x0000), 0x20);
        assert_eq!(ppu.read_vram(0x0c00), 0x23);
        assert_eq!(ppu.read_vram(0x1000), 6);
        assert_eq!(ppu.read_vram(0x1400), 7);

        assert_eq!(cart.read_ram(0x6000), 0);
        for (address, key, value) in [
            (0x7ef7, 0xca, 0x11),
            (0x7ef8, 0x69, 0x22),
            (0x7ef9, 0x84, 0x33),
        ] {
            assert!(cart.write_low_mapper(address, key));
            let ram_address = match address {
                0x7ef7 => 0x6000,
                0x7ef8 => 0x6800,
                _ => 0x7000,
            };
            cart.write_ram(ram_address, value);
            assert_eq!(cart.read_ram(ram_address), value);
        }
        assert_eq!(cart.read_ram(0x7400), 0);
        assert!(cart.write_low_mapper(0x7ef8, 0));
        assert_eq!(cart.read_ram(0x6800), 0);
    }

    #[test]
    fn taito_x1017_mapper552_uses_physical_prg_bit_order() {
        let mut rom = nes2_mapper_rom(552, 0, 32, 32);
        for bank in 0..64usize {
            rom[16 + bank * 0x2000] = bank as u8;
        }
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();
        assert!(cart.write_low_mapper(0x7efa, 0x20));
        assert!(cart.write_low_mapper(0x7efb, 0x01));
        assert!(cart.write_low_mapper(0x7efc, 0x3f));
        assert_eq!(cart.read_prg(0x8000), 1);
        assert_eq!(cart.read_prg(0xa000), 32);
        assert_eq!(cart.read_prg(0xc000), 63);
        assert_eq!(cart.read_prg(0xe000), 63);
    }

    #[test]
    fn taito_x1017_irq_counts_acknowledges_and_honors_mode_stop() {
        let rom = mapper_rom(82, 8, 8);
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();
        assert!(cart.write_low_mapper(0x7efd, 2));
        assert!(cart.write_low_mapper(0x7efe, 0));
        assert_eq!(cart.taito_x1017.irq_counter, 64);
        assert!(cart.write_low_mapper(0x7efe, 3));
        for _ in 0..63 {
            cart.tick_cpu_cycle();
        }
        assert!(!cart.irq_pending());
        assert_eq!(cart.taito_x1017.irq_counter, 1);
        cart.tick_cpu_cycle();
        assert!(cart.irq_pending());

        assert!(cart.write_low_mapper(0x7eff, 0));
        assert!(!cart.irq_pending());
        assert_eq!(cart.taito_x1017.irq_counter, 48);
        assert!(cart.write_low_mapper(0x7efe, 7));
        for _ in 0..16 {
            cart.tick_cpu_cycle();
        }
        assert_eq!(cart.taito_x1017.irq_counter, 48);
    }

    #[test]
    fn taito_x1017_state_and_battery_persistence_preserve_ram_and_irq_phase() {
        let mut rom = mapper_rom(82, 8, 8);
        rom[6] |= 0x02;
        let mut machine = NesMachine::from_rom(&rom).unwrap();
        assert_eq!(machine.persistent_len(ResourceKind::Storage, 0), 5 * 1024);
        machine.bus.write8(0x7ef7, 0xca);
        machine.bus.write8(0x7ef8, 0x69);
        machine.bus.write8(0x6000, 0x41);
        machine.bus.write8(0x6800, 0x52);
        machine.bus.write8(0x7efd, 3);
        machine.bus.write8(0x7efe, 3);
        for _ in 0..11 {
            machine.bus.cartridge.tick_cpu_cycle();
        }
        let state = machine.save_state().unwrap();
        let counter = machine.bus.cartridge.taito_x1017.irq_counter;

        machine.bus.write8(0x7ef7, 0);
        machine.bus.write8(0x6000, 0);
        machine.load_state(&state).unwrap();
        assert!(machine.bus.cartridge.taito_x1017.ram_enabled[0]);
        assert!(machine.bus.cartridge.taito_x1017.ram_enabled[1]);
        assert_eq!(machine.bus.read8(0x6000), 0x41);
        assert_eq!(machine.bus.read8(0x6800), 0x52);
        assert_eq!(machine.bus.cartridge.taito_x1017.irq_counter, counter);

        let mut persistent = vec![0u8; 5 * 1024];
        machine
            .read_persistent(ResourceKind::Storage, 0, &mut persistent)
            .unwrap();
        assert_eq!(persistent[0], 0x41);
        assert_eq!(persistent[0x800], 0x52);
    }

    #[test]
    fn jaleco_ss88006_banks_prg_chr_controls_ram_and_mirroring() {
        let mut rom = mapper_rom(18, 32, 32);
        for bank in 0..64usize {
            rom[16 + bank * 0x2000] = bank as u8;
        }
        let chr_start = 16 + 32 * 0x4000;
        for bank in 0..256usize {
            rom[chr_start + bank * 0x0400..chr_start + (bank + 1) * 0x0400].fill(bank as u8);
        }
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        for (low, high, value) in [
            (0x8000, 0x8001, 0x12),
            (0x8002, 0x8003, 0x23),
            (0x9000, 0x9001, 0x34),
        ] {
            cart.write_mapper(low, value & 0x0f);
            cart.write_mapper(high, value >> 4);
        }
        assert_eq!(cart.read_prg(0x8000), 0x12);
        assert_eq!(cart.read_prg(0xa000), 0x23);
        assert_eq!(cart.read_prg(0xc000), 0x34);
        assert_eq!(cart.read_prg(0xe000), 0x3f);

        for bank in 0..8u16 {
            let value = 0x20 + bank as u8;
            let base = 0xa000 + (bank / 2) * 0x1000 + (bank & 1) * 2;
            cart.write_mapper(base, value & 0x0f);
            cart.write_mapper(base + 1, value >> 4);
        }
        cart.sync_ppu(&mut ppu);
        for slot in 0..8usize {
            assert_eq!(ppu.read_vram((slot * 0x400) as u16), 0x20 + slot as u8);
        }

        assert_eq!(cart.read_ram(0x6000), 0xff);
        cart.write_mapper(0x9002, 3);
        cart.write_ram(0x6000, 0x5a);
        assert_eq!(cart.read_ram(0x6000), 0x5a);
        cart.write_mapper(0x9002, 1);
        cart.write_ram(0x6000, 0xa5);
        assert_eq!(cart.read_ram(0x6000), 0x5a);

        for (value, expected) in [
            (0, Mirroring::Horizontal),
            (1, Mirroring::Vertical),
            (2, Mirroring::SingleScreen0),
            (3, Mirroring::SingleScreen1),
        ] {
            cart.write_mapper(0xf002, value);
            cart.sync_ppu(&mut ppu);
            assert_eq!(ppu.mirroring, expected);
        }
    }

    #[test]
    fn jaleco_ss88006_irq_uses_selected_counter_width_and_one_to_zero_trigger() {
        let rom = mapper_rom(18, 8, 8);
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();
        for (control, expected_wrap) in [(1, 0xffff), (3, 0x0fff), (5, 0x00ff), (9, 0x000f)] {
            for address in 0xe000..=0xe003 {
                cart.write_mapper(address, 0);
            }
            cart.write_mapper(0xf000, 0);
            cart.write_mapper(0xf001, control);
            cart.tick_cpu_cycle();
            assert_eq!(cart.jaleco_ss88006.irq_counter, expected_wrap);
            assert!(!cart.irq_pending());

            cart.write_mapper(0xe000, 1);
            cart.write_mapper(0xf000, 0);
            cart.write_mapper(0xf001, control);
            cart.tick_cpu_cycle();
            assert_eq!(cart.jaleco_ss88006.irq_counter, 0);
            assert!(cart.irq_pending());
            cart.write_mapper(0xf000, 0);
            assert!(!cart.irq_pending());
        }
    }

    #[test]
    fn jaleco_ss88006_mapper_state_round_trip_preserves_phase() {
        let rom = mapper_rom(18, 8, 8);
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();
        for (address, value) in [
            (0x8000, 5),
            (0x8001, 2),
            (0x9002, 3),
            (0xa000, 7),
            (0xa001, 4),
            (0xe000, 9),
            (0xe001, 8),
            (0xe002, 7),
            (0xe003, 6),
            (0xf000, 0),
            (0xf001, 5),
            (0xf002, 3),
            (0xf003, 0xa5),
        ] {
            cart.write_mapper(address, value);
        }
        for _ in 0..17 {
            cart.tick_cpu_cycle();
        }
        let mut writer = StateWriter::new(PlatformId::Nes, 99);
        save_mapper_state(&mut writer, &cart);
        let bytes = writer.finish();
        let (mut restored, _) = Cartridge::parse(&rom).unwrap();
        let mut reader = StateReader::new(&bytes, PlatformId::Nes, 99).unwrap();
        load_mapper_state(&mut reader, &mut restored).unwrap();
        reader.finish().unwrap();
        assert_eq!(restored.jaleco_ss88006.prg, cart.jaleco_ss88006.prg);
        assert_eq!(restored.jaleco_ss88006.chr, cart.jaleco_ss88006.chr);
        assert_eq!(
            restored.jaleco_ss88006.ram_control,
            cart.jaleco_ss88006.ram_control
        );
        assert_eq!(
            restored.jaleco_ss88006.irq_reload,
            cart.jaleco_ss88006.irq_reload
        );
        assert_eq!(
            restored.jaleco_ss88006.irq_counter,
            cart.jaleco_ss88006.irq_counter
        );
        assert_eq!(
            restored.jaleco_ss88006.irq_control,
            cart.jaleco_ss88006.irq_control
        );
        assert_eq!(
            restored.jaleco_ss88006.irq_pending,
            cart.jaleco_ss88006.irq_pending
        );
        assert_eq!(
            restored.jaleco_ss88006.mirroring,
            cart.jaleco_ss88006.mirroring
        );
        assert_eq!(
            restored.jaleco_ss88006.sound_control,
            cart.jaleco_ss88006.sound_control
        );
    }

    #[test]
    fn vrc1_banks_prg_chr_and_mirroring_without_wram() {
        let mut rom = mapper_rom(75, 8, 16);
        for bank in 0..16usize {
            rom[16 + bank * 0x2000] = (0x20 + bank) as u8;
        }
        let chr_start = 16 + 8 * 0x4000;
        for bank in 0..32usize {
            rom[chr_start + bank * 0x1000] = (0x40 + bank) as u8;
        }
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        cart.write_mapper(0x8000, 3);
        cart.write_mapper(0xa000, 4);
        cart.write_mapper(0xc000, 5);
        assert_eq!(cart.read_prg(0x8000), 0x23);
        assert_eq!(cart.read_prg(0xa000), 0x24);
        assert_eq!(cart.read_prg(0xc000), 0x25);
        assert_eq!(cart.read_prg(0xe000), 0x2f);

        cart.write_mapper(0xe000, 2);
        cart.write_mapper(0xf000, 3);
        cart.write_mapper(0x9000, 0x07);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.read_vram(0x0000), 0x52);
        assert_eq!(ppu.read_vram(0x1000), 0x53);
        assert_eq!(ppu.mirroring, Mirroring::Horizontal);
        assert_eq!(cart.read_ram(0x6000), 0xff);
    }

    #[test]
    fn vrc3_banks_prg_and_supports_sixteen_and_eight_bit_irqs() {
        let mut rom = mapper_rom(73, 8, 0);
        for bank in 0..8usize {
            rom[16 + bank * 0x4000] = (0x20 + bank) as u8;
        }
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();
        cart.write_mapper(0xf000, 3);
        assert_eq!(cart.read_prg(0x8000), 0x23);
        assert_eq!(cart.read_prg(0xc000), 0x27);

        for (address, value) in [
            (0x8000, 0x0e),
            (0x9000, 0x0f),
            (0xa000, 0x0f),
            (0xb000, 0x0f),
        ] {
            cart.write_mapper(address, value);
        }
        cart.write_mapper(0xc000, 0x03);
        cart.tick_cpu_cycle();
        assert_eq!(cart.vrc3.irq_counter, 0xffff);
        assert!(!cart.irq_pending());
        cart.tick_cpu_cycle();
        assert_eq!(cart.vrc3.irq_counter, 0xfffe);
        assert!(cart.irq_pending());
        cart.write_mapper(0xd000, 0);
        assert!(!cart.irq_pending());
        assert!(cart.vrc3.irq_enabled);

        for (address, value) in [
            (0x8000, 0x0e),
            (0x9000, 0x0f),
            (0xa000, 0x02),
            (0xb000, 0x01),
        ] {
            cart.write_mapper(address, value);
        }
        cart.write_mapper(0xc000, 0x06);
        assert_eq!(cart.vrc3.irq_counter, 0x12fe);
        cart.tick_cpu_cycle();
        assert_eq!(cart.vrc3.irq_counter, 0x12ff);
        cart.tick_cpu_cycle();
        assert_eq!(cart.vrc3.irq_counter, 0x12fe);
        assert!(cart.irq_pending());
        cart.write_mapper(0xd000, 0);
        assert!(!cart.vrc3.irq_enabled);
    }

    #[test]
    fn vrc1_and_vrc3_mapper_state_round_trip_preserves_phase() {
        let vrc1_rom = mapper_rom(75, 8, 16);
        let (mut vrc1, _) = Cartridge::parse(&vrc1_rom).unwrap();
        vrc1.write_mapper(0x8000, 5);
        vrc1.write_mapper(0x9000, 7);
        vrc1.write_mapper(0xe000, 9);
        vrc1.write_mapper(0xf000, 6);
        let mut writer = StateWriter::new(PlatformId::Nes, 99);
        save_mapper_state(&mut writer, &vrc1);
        let bytes = writer.finish();
        let (mut restored_vrc1, _) = Cartridge::parse(&vrc1_rom).unwrap();
        let mut reader = StateReader::new(&bytes, PlatformId::Nes, 99).unwrap();
        load_mapper_state(&mut reader, &mut restored_vrc1).unwrap();
        reader.finish().unwrap();
        assert_eq!(restored_vrc1.vrc1.prg, vrc1.vrc1.prg);
        assert_eq!(restored_vrc1.vrc1.chr, vrc1.vrc1.chr);
        assert_eq!(
            restored_vrc1.vrc1.mirror_horizontal,
            vrc1.vrc1.mirror_horizontal
        );

        let vrc3_rom = mapper_rom(73, 4, 0);
        let (mut vrc3, _) = Cartridge::parse(&vrc3_rom).unwrap();
        for (address, value) in [(0x8000, 4), (0x9000, 3), (0xa000, 2), (0xb000, 1)] {
            vrc3.write_mapper(address, value);
        }
        vrc3.write_mapper(0xc000, 0x07);
        for _ in 0..23 {
            vrc3.tick_cpu_cycle();
        }
        let mut writer = StateWriter::new(PlatformId::Nes, 99);
        save_mapper_state(&mut writer, &vrc3);
        let bytes = writer.finish();
        let (mut restored_vrc3, _) = Cartridge::parse(&vrc3_rom).unwrap();
        let mut reader = StateReader::new(&bytes, PlatformId::Nes, 99).unwrap();
        load_mapper_state(&mut reader, &mut restored_vrc3).unwrap();
        reader.finish().unwrap();
        assert_eq!(restored_vrc3.vrc3.irq_latch, vrc3.vrc3.irq_latch);
        assert_eq!(restored_vrc3.vrc3.irq_counter, vrc3.vrc3.irq_counter);
        assert_eq!(restored_vrc3.vrc3.irq_enabled, vrc3.vrc3.irq_enabled);
        assert_eq!(restored_vrc3.vrc3.irq_mode_8bit, vrc3.vrc3.irq_mode_8bit);
    }

    #[test]
    fn vrc4_banks_prg_chr_and_controls_wram_and_mirroring() {
        let mut rom = mapper_rom(21, 8, 8);
        for bank in 0..16usize {
            rom[16 + bank * 0x2000] = (0x20 + bank) as u8;
        }
        let chr_start = 16 + 8 * 0x4000;
        for bank in 0..64usize {
            rom[chr_start + bank * 0x0400] = (0x60 + bank) as u8;
        }
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        cart.write_mapper(0x8000, 3);
        cart.write_mapper(0xa000, 4);
        assert_eq!(cart.read_prg(0x8000), 0x23);
        assert_eq!(cart.read_prg(0xa000), 0x24);
        assert_eq!(cart.read_prg(0xc000), 0x2e);
        assert_eq!(cart.read_prg(0xe000), 0x2f);

        cart.write_mapper(0xb000, 2);
        cart.write_mapper(0xb002, 1);
        cart.write_mapper(0xb004, 3);
        cart.write_mapper(0xb006, 0);
        cart.write_mapper(0x9000, 3);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.read_vram(0x0000), 0x72);
        assert_eq!(ppu.read_vram(0x0400), 0x63);
        assert_eq!(ppu.mirroring, Mirroring::SingleScreen1);

        cart.write_mapper(0x9004, 3);
        assert_eq!(cart.read_prg(0x8000), 0x2e);
        assert_eq!(cart.read_prg(0xc000), 0x23);
        cart.write_ram(0x6000, 0x5a);
        assert_eq!(cart.read_ram(0x6000), 0x5a);
    }

    #[test]
    fn vrc4_submappers_decode_their_pcb_address_lines() {
        for (mapper, submapper, high_address) in [
            (21u16, 2u8, 0xb040u16),
            (23, 2, 0xb004),
            (25, 1, 0xb002),
            (25, 2, 0xb008),
        ] {
            let rom = nes2_mapper_rom(mapper, submapper, 4, 8);
            let (mut cart, _) = Cartridge::parse(&rom).unwrap();
            cart.write_mapper(0xb000, 0x0a);
            cart.write_mapper(high_address, 0x01);
            assert_eq!(
                cart.vrc4.chr[0], 0x01a,
                "mapper {mapper} submapper {submapper}"
            );
        }
    }

    #[test]
    fn vrc2a_uses_shifted_chr_banks_and_the_six_thousand_latch() {
        let rom = mapper_rom(22, 4, 8);
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        cart.write_mapper(0xb000, 4);
        cart.write_mapper(0xb002, 1);
        cart.sync_ppu(&mut ppu);
        assert_eq!(cart.vrc4.chr[0], 0x14);
        assert_eq!(ppu.chr_map[0], 0x0a);

        assert!(cart.write_low_mapper(0x6000, 1));
        assert_eq!(cart.read_ram(0x6000), 0x61);
        assert_eq!(cart.read_ram(0x7000), 0xff);
        cart.write_mapper(0x9002, 1);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.mirroring, Mirroring::Horizontal);
    }

    #[test]
    fn vrc4_irq_supports_cycle_and_scanline_prescaler_modes() {
        let rom = mapper_rom(21, 4, 1);
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();
        cart.write_mapper(0xf000, 0x0e);
        cart.write_mapper(0xf002, 0x0f);
        cart.write_mapper(0xf004, 0x07);
        cart.tick_cpu_cycle();
        assert!(!cart.irq_pending());
        cart.tick_cpu_cycle();
        assert!(cart.irq_pending());

        cart.write_mapper(0xf006, 0);
        assert!(!cart.irq_pending());
        assert!(cart.vrc4.irq_enabled);
        cart.tick_cpu_cycle();
        cart.tick_cpu_cycle();
        assert!(cart.irq_pending());

        cart.write_mapper(0xf000, 0x0f);
        cart.write_mapper(0xf002, 0x0f);
        cart.write_mapper(0xf004, 0x02);
        for _ in 0..113 {
            cart.tick_cpu_cycle();
        }
        assert!(!cart.irq_pending());
        cart.tick_cpu_cycle();
        assert!(cart.irq_pending());
    }

    #[test]
    fn vrc_mapper_state_round_trip_preserves_irq_phase() {
        let rom = nes2_mapper_rom(21, 1, 4, 1);
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();
        cart.write_mapper(0x8000, 5);
        cart.write_mapper(0xb000, 7);
        cart.write_mapper(0xb002, 1);
        cart.write_mapper(0xf000, 0x0c);
        cart.write_mapper(0xf002, 0x0d);
        cart.write_mapper(0xf004, 0x03);
        for _ in 0..37 {
            cart.tick_cpu_cycle();
        }

        let mut writer = StateWriter::new(PlatformId::Nes, 99);
        save_mapper_state(&mut writer, &cart);
        let bytes = writer.finish();
        let (mut restored, _) = Cartridge::parse(&rom).unwrap();
        let mut reader = StateReader::new(&bytes, PlatformId::Nes, 99).unwrap();
        load_mapper_state(&mut reader, &mut restored).unwrap();
        reader.finish().unwrap();

        assert_eq!(restored.vrc4.prg, cart.vrc4.prg);
        assert_eq!(restored.vrc4.chr, cart.vrc4.chr);
        assert_eq!(restored.vrc4.irq_latch, cart.vrc4.irq_latch);
        assert_eq!(restored.vrc4.irq_counter, cart.vrc4.irq_counter);
        assert_eq!(restored.vrc4.irq_prescaler, cart.vrc4.irq_prescaler);
        assert_eq!(restored.vrc4.irq_enabled, cart.vrc4.irq_enabled);
        assert_eq!(restored.vrc4.irq_pending, cart.vrc4.irq_pending);
    }

    #[test]
    fn vrc6_banks_prg_chr_and_controls_wram_and_mirroring() {
        let mut rom = mapper_rom(24, 8, 8);
        for bank in 0..16usize {
            rom[16 + bank * 0x2000] = (0x20 + bank) as u8;
        }
        let chr_start = 16 + 8 * 0x4000;
        for bank in 0..64usize {
            rom[chr_start + bank * 0x0400] = (0x40 + bank) as u8;
        }
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();

        cart.write_mapper(0x8000, 3);
        cart.write_mapper(0xc000, 5);
        assert_eq!(cart.read_prg(0x8000), 0x26);
        assert_eq!(cart.read_prg(0xa000), 0x27);
        assert_eq!(cart.read_prg(0xc000), 0x25);
        assert_eq!(cart.read_prg(0xe000), 0x2f);
        assert_eq!(cart.read_ram(0x6000), 0xff);

        for (slot, address) in [
            0xd000u16, 0xd001, 0xd002, 0xd003, 0xe000, 0xe001, 0xe002, 0xe003,
        ]
        .into_iter()
        .enumerate()
        {
            cart.write_mapper(address, (slot + 1) as u8);
        }
        cart.write_mapper(0xb003, 0xa4);
        cart.sync_ppu(&mut ppu);
        for slot in 0..8usize {
            assert_eq!(ppu.read_vram((slot * 0x400) as u16), 0x41 + slot as u8);
        }
        assert_eq!(ppu.mirroring, Mirroring::Horizontal);
        cart.write_ram(0x6000, 0x5a);
        assert_eq!(cart.read_ram(0x6000), 0x5a);
    }

    #[test]
    fn vrc6b_swaps_a0_a1_for_chr_and_audio_registers() {
        let rom = mapper_rom(26, 4, 8);
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();

        cart.write_mapper(0xd001, 5);
        cart.write_mapper(0xd002, 7);
        cart.sync_ppu(&mut ppu);
        assert_eq!(cart.vrc6.chr[2], 5);
        assert_eq!(cart.vrc6.chr[1], 7);
        assert_eq!(ppu.chr_map[2], 5);
        assert_eq!(ppu.chr_map[1], 7);

        cart.write_mapper(0x9002, 0x34);
        cart.write_mapper(0x9001, 0x82);
        assert_eq!(cart.vrc6.pulses[0].period, 0x234);
        assert!(cart.vrc6.pulses[0].enabled);
    }

    #[test]
    fn vrc6_audio_clocks_pulse_saw_and_frequency_halt() {
        let rom = mapper_rom(24, 4, 1);
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();

        cart.write_mapper(0x9000, 0x8f);
        cart.write_mapper(0x9001, 0);
        cart.write_mapper(0x9002, 0x80);
        assert!(cart.expansion_audio_output() < 0.0);
        cart.tick_cpu_cycle();
        let pulse_step = cart.vrc6.pulses[0].step;
        cart.write_mapper(0x9003, 1);
        for _ in 0..8 {
            cart.tick_cpu_cycle();
        }
        assert_eq!(cart.vrc6.pulses[0].step, pulse_step);

        cart.write_mapper(0x9002, 0);
        cart.write_mapper(0x9003, 0);
        cart.write_mapper(0xb000, 8);
        cart.write_mapper(0xb001, 0);
        cart.write_mapper(0xb002, 0x80);
        cart.tick_cpu_cycle();
        cart.tick_cpu_cycle();
        assert_eq!(cart.vrc6.saw.accumulator, 8);
        assert_eq!(cart.vrc6.saw.output(), 1);
    }

    #[test]
    fn vrc6_irq_supports_cycle_scanline_and_acknowledge_modes() {
        let rom = mapper_rom(24, 4, 1);
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();

        cart.write_mapper(0xf000, 0xfe);
        cart.write_mapper(0xf001, 0x07);
        cart.tick_cpu_cycle();
        assert!(!cart.irq_pending());
        cart.tick_cpu_cycle();
        assert!(cart.irq_pending());
        cart.write_mapper(0xf002, 0);
        assert!(!cart.irq_pending());
        assert!(cart.vrc6.irq_enabled);

        cart.write_mapper(0xf000, 0xff);
        cart.write_mapper(0xf001, 0x02);
        for _ in 0..113 {
            cart.tick_cpu_cycle();
        }
        assert!(!cart.irq_pending());
        cart.tick_cpu_cycle();
        assert!(cart.irq_pending());
    }

    #[test]
    fn vrc6_mapper_state_round_trip_preserves_audio_and_irq_phase() {
        let rom = mapper_rom(24, 4, 1);
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();
        cart.write_mapper(0x8000, 3);
        cart.write_mapper(0xd000, 9);
        cart.write_mapper(0x9000, 0x4c);
        cart.write_mapper(0x9001, 0x56);
        cart.write_mapper(0x9002, 0x83);
        cart.write_mapper(0xb000, 0x15);
        cart.write_mapper(0xb001, 0x22);
        cart.write_mapper(0xb002, 0x81);
        cart.write_mapper(0xf000, 0xd0);
        cart.write_mapper(0xf001, 0x03);
        for _ in 0..37 {
            cart.tick_cpu_cycle();
        }

        let mut writer = StateWriter::new(PlatformId::Nes, 99);
        save_mapper_state(&mut writer, &cart);
        let bytes = writer.finish();
        let (mut restored, _) = Cartridge::parse(&rom).unwrap();
        let mut reader = StateReader::new(&bytes, PlatformId::Nes, 99).unwrap();
        load_mapper_state(&mut reader, &mut restored).unwrap();
        reader.finish().unwrap();

        assert_eq!(restored.vrc6.prg16, cart.vrc6.prg16);
        assert_eq!(restored.vrc6.chr, cart.vrc6.chr);
        assert_eq!(restored.vrc6.pulses[0].period, cart.vrc6.pulses[0].period);
        assert_eq!(restored.vrc6.pulses[0].step, cart.vrc6.pulses[0].step);
        assert_eq!(restored.vrc6.saw.accumulator, cart.vrc6.saw.accumulator);
        assert_eq!(restored.vrc6.irq_counter, cart.vrc6.irq_counter);
        assert_eq!(restored.vrc6.irq_prescaler, cart.vrc6.irq_prescaler);
        assert_eq!(restored.vrc6.irq_enabled, cart.vrc6.irq_enabled);
    }

    #[test]
    fn vrc7a_banks_prg_chr_controls_wram_and_mirroring() {
        let rom = vrc7_test_rom(2);
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();

        cart.write_mapper(0x8000, 3);
        cart.write_mapper(0x8010, 4);
        cart.write_mapper(0x9000, 5);
        assert_eq!(cart.read_prg(0x8000), 0x23);
        assert_eq!(cart.read_prg(0xa000), 0x24);
        assert_eq!(cart.read_prg(0xc000), 0x25);
        assert_eq!(cart.read_prg(0xe000), 0x3f);

        for (slot, address) in [
            0xa000u16, 0xa010, 0xb000, 0xb010, 0xc000, 0xc010, 0xd000, 0xd010,
        ]
        .into_iter()
        .enumerate()
        {
            cart.write_mapper(address, (slot + 1) as u8);
        }
        cart.write_mapper(0xe000, 0x81);
        cart.sync_ppu(&mut ppu);
        for slot in 0..8usize {
            assert_eq!(ppu.read_vram((slot * 0x400) as u16), 0x41 + slot as u8);
        }
        assert_eq!(ppu.mirroring, Mirroring::Horizontal);
        cart.write_ram(0x6000, 0x5a);
        assert_eq!(cart.read_ram(0x6000), 0x5a);
    }

    #[test]
    fn vrc7b_uses_a3_decode_and_has_no_fm_output() {
        let rom = vrc7_test_rom(1);
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();

        cart.write_mapper(0x8008, 6);
        assert_eq!(cart.read_prg(0xa000), 0x26);
        cart.write_mapper(0x8010, 7);
        assert_eq!(cart.read_prg(0xa000), 0x26);

        cart.write_mapper(0xa008, 5);
        cart.write_mapper(0xa010, 9);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.read_vram(0x0400), 0x45);

        cart.write_mapper(0x9010, 0x30);
        cart.write_mapper(0x9030, 0x10);
        assert_eq!(cart.vrc7.selected_audio_register, 0);
        assert!(cart.vrc7.audio_registers.iter().all(|&value| value == 0));
        for _ in 0..144 {
            cart.tick_cpu_cycle();
        }
        assert_eq!(cart.expansion_audio_output(), 0.0);
    }

    #[test]
    fn vrc7_irq_supports_cycle_scanline_and_acknowledge_modes() {
        let rom = vrc7_test_rom(2);
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();

        cart.write_mapper(0xe010, 0xfe);
        cart.write_mapper(0xf000, 0x07);
        cart.tick_cpu_cycle();
        assert!(!cart.irq_pending());
        cart.tick_cpu_cycle();
        assert!(cart.irq_pending());
        cart.write_mapper(0xf010, 0);
        assert!(!cart.irq_pending());
        assert!(cart.vrc7.irq_enabled);

        cart.write_mapper(0xe010, 0xff);
        cart.write_mapper(0xf000, 0x02);
        for _ in 0..113 {
            cart.tick_cpu_cycle();
        }
        assert!(!cart.irq_pending());
        cart.tick_cpu_cycle();
        assert!(cart.irq_pending());
    }

    #[test]
    fn vrc7_audio_registers_reset_and_generate_fm_output() {
        let rom = vrc7_test_rom(2);
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();

        for (register, value) in [(0x30, 0x10), (0x10, 0x80), (0x20, 0x16)] {
            cart.write_mapper(0x9010, register);
            cart.write_mapper(0x9030, value);
        }
        let mut peak = 0.0f32;
        for _ in 0..5000 {
            cart.tick_cpu_cycle();
            peak = peak.max(cart.expansion_audio_output().abs());
        }
        assert!(cart.vrc7.channels[0].envelope > 0.0);
        assert!(peak > 0.000001);

        cart.write_mapper(0xe000, 0x40);
        assert_eq!(cart.expansion_audio_output(), 0.0);
        assert!(cart.vrc7.audio_registers.iter().all(|&value| value == 0));
        assert!(cart
            .vrc7
            .channels
            .iter()
            .all(|channel| channel.envelope == 0.0));
    }

    #[test]
    fn vrc7_mapper_state_round_trip_preserves_audio_and_irq_phase() {
        let rom = vrc7_test_rom(2);
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();
        cart.write_mapper(0x8000, 3);
        cart.write_mapper(0x8010, 4);
        cart.write_mapper(0xa000, 9);
        cart.write_mapper(0xe000, 0x81);
        cart.write_mapper(0xe010, 0xd0);
        cart.write_mapper(0xf000, 0x03);
        for (register, value) in [(0x30, 0x20), (0x10, 0x56), (0x20, 0x17)] {
            cart.write_mapper(0x9010, register);
            cart.write_mapper(0x9030, value);
        }
        for _ in 0..257 {
            cart.tick_cpu_cycle();
        }

        let mut writer = StateWriter::new(PlatformId::Nes, 99);
        save_mapper_state(&mut writer, &cart);
        let bytes = writer.finish();
        let (mut restored, _) = Cartridge::parse(&rom).unwrap();
        let mut reader = StateReader::new(&bytes, PlatformId::Nes, 99).unwrap();
        load_mapper_state(&mut reader, &mut restored).unwrap();
        reader.finish().unwrap();

        assert_eq!(restored.vrc7.prg, cart.vrc7.prg);
        assert_eq!(restored.vrc7.chr, cart.vrc7.chr);
        assert_eq!(restored.vrc7.control, cart.vrc7.control);
        assert_eq!(restored.vrc7.irq_counter, cart.vrc7.irq_counter);
        assert_eq!(restored.vrc7.irq_prescaler, cart.vrc7.irq_prescaler);
        assert_eq!(restored.vrc7.audio_registers, cart.vrc7.audio_registers);
        assert_eq!(
            restored.vrc7.channels[0].mod_phase.to_bits(),
            cart.vrc7.channels[0].mod_phase.to_bits()
        );
        assert_eq!(
            restored.vrc7.channels[0].envelope.to_bits(),
            cart.vrc7.channels[0].envelope.to_bits()
        );
        assert_eq!(
            restored.vrc7.audio_output.to_bits(),
            cart.vrc7.audio_output.to_bits()
        );
    }

    #[test]
    fn sunsoft3_banks_prg_chr_and_controls_all_mirroring_modes() {
        let mut rom = mapper_rom(67, 16, 16);
        for bank in 0..16usize {
            rom[16 + bank * 0x4000..16 + (bank + 1) * 0x4000].fill((0x20 + bank) as u8);
        }
        let chr_start = 16 + 16 * 0x4000;
        for bank in 0..64usize {
            rom[chr_start + bank * 0x800..chr_start + (bank + 1) * 0x800].fill(bank as u8);
        }
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        assert_eq!(cart.read_prg(0x8000), 0x20);
        assert_eq!(cart.read_prg(0xc000), 0x2f);
        cart.write_mapper(0xffff, 5);
        assert_eq!(cart.read_prg(0x8000), 0x25);
        assert_eq!(cart.read_prg(0xc000), 0x2f);

        for (address, bank) in [(0x8fff, 3), (0x9fff, 7), (0xafff, 11), (0xbfff, 15)] {
            cart.write_mapper(address, bank);
        }
        cart.sync_ppu(&mut ppu);
        for (slot, bank) in [3u8, 7, 11, 15].into_iter().enumerate() {
            assert_eq!(ppu.read_vram((slot * 0x800) as u16), bank);
        }
        for (value, expected) in [
            (0, Mirroring::Vertical),
            (1, Mirroring::Horizontal),
            (2, Mirroring::SingleScreen0),
            (3, Mirroring::SingleScreen1),
        ] {
            cart.write_mapper(0xe800, value);
            cart.sync_ppu(&mut ppu);
            assert_eq!(ppu.mirroring, expected);
        }
    }

    #[test]
    fn sunsoft3_irq_wraps_once_resets_toggle_and_acks_only_at_8000() {
        let rom = mapper_rom(67, 8, 8);
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();
        cart.write_mapper(0xc800, 0x00);
        cart.write_mapper(0xc800, 0x01);
        assert_eq!(cart.sunsoft3.irq_counter, 1);
        cart.write_mapper(0xd800, 0x10);
        cart.tick_cpu_cycle();
        assert_eq!(cart.sunsoft3.irq_counter, 0);
        assert!(!cart.irq_pending());
        cart.tick_cpu_cycle();
        assert_eq!(cart.sunsoft3.irq_counter, 0xffff);
        assert!(cart.irq_pending());
        assert!(!cart.sunsoft3.irq_enabled);

        cart.write_mapper(0xd800, 0x10);
        assert!(cart.irq_pending());
        cart.write_mapper(0x8000, 0);
        assert!(!cart.irq_pending());
        cart.write_mapper(0xc800, 0x12);
        cart.write_mapper(0xc800, 0x34);
        assert_eq!(cart.sunsoft3.irq_counter, 0x1234);
    }

    #[test]
    fn sunsoft3_mapper_state_round_trip_preserves_irq_write_phase() {
        let rom = mapper_rom(67, 8, 8);
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();
        cart.write_mapper(0xf800, 3);
        cart.write_mapper(0xa800, 9);
        cart.write_mapper(0xe800, 3);
        cart.write_mapper(0xc800, 0x45);
        cart.write_mapper(0xd800, 0x10);
        cart.write_mapper(0xc800, 0x67);
        assert!(cart.sunsoft3.irq_low_next);

        let mut writer = StateWriter::new(PlatformId::Nes, 99);
        save_mapper_state(&mut writer, &cart);
        let bytes = writer.finish();
        let (mut restored, _) = Cartridge::parse(&rom).unwrap();
        let mut reader = StateReader::new(&bytes, PlatformId::Nes, 99).unwrap();
        load_mapper_state(&mut reader, &mut restored).unwrap();
        reader.finish().unwrap();

        assert_eq!(restored.sunsoft3.chr_2k, cart.sunsoft3.chr_2k);
        assert_eq!(restored.sunsoft3.prg, cart.sunsoft3.prg);
        assert_eq!(restored.sunsoft3.mirroring, cart.sunsoft3.mirroring);
        assert_eq!(restored.sunsoft3.irq_counter, cart.sunsoft3.irq_counter);
        assert_eq!(restored.sunsoft3.irq_low_next, cart.sunsoft3.irq_low_next);
        assert_eq!(restored.sunsoft3.irq_enabled, cart.sunsoft3.irq_enabled);
        assert_eq!(restored.sunsoft3.irq_pending, cart.sunsoft3.irq_pending);
    }

    #[test]
    fn sunsoft4_banks_prg_chr_controls_mirroring_and_wram() {
        let mut rom = mapper_rom(68, 16, 32);
        for bank in 0..16usize {
            rom[16 + bank * 0x4000] = bank as u8;
        }
        let chr_start = 16 + 16 * 0x4000;
        for bank in 0..256usize {
            rom[chr_start + bank * 0x400..chr_start + (bank + 1) * 0x400].fill(bank as u8);
        }
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        assert_eq!(cart.read_ram(0x6000), 0xff);

        for (address, bank) in [(0x8000, 3), (0x9000, 5), (0xa000, 7), (0xb000, 9)] {
            cart.write_mapper(address, bank);
        }
        cart.write_mapper(0xf000, 0x15);
        assert_eq!(cart.read_prg(0x8000), 5);
        assert_eq!(cart.read_prg(0xc000), 15);
        cart.write_ram(0x6000, 0x5a);
        assert_eq!(cart.read_ram(0x6000), 0x5a);

        cart.write_mapper(0xe000, 1);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.mirroring, Mirroring::Horizontal);
        assert_eq!(ppu.read_vram(0x0000), 6);
        assert_eq!(ppu.read_vram(0x0400), 7);
        assert_eq!(ppu.read_vram(0x0800), 10);
        assert_eq!(ppu.read_vram(0x1000), 14);
        assert_eq!(ppu.read_vram(0x1800), 18);

        cart.write_mapper(0xf000, 2);
        assert_eq!(cart.read_ram(0x6000), 0xff);
    }

    #[test]
    fn sunsoft4_maps_read_only_chr_rom_into_nametables() {
        let mut rom = mapper_rom(68, 8, 32);
        let chr_start = 16 + 8 * 0x4000;
        for bank in 0..256usize {
            rom[chr_start + bank * 0x400..chr_start + (bank + 1) * 0x400].fill(bank as u8);
        }
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        cart.write_mapper(0xc000, 2);
        cart.write_mapper(0xd000, 4);
        cart.write_mapper(0xe000, 0x10);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.read_vram(0x2000), 0x82);
        assert_eq!(ppu.read_vram(0x2400), 0x84);
        assert_eq!(ppu.read_vram(0x2800), 0x82);
        assert_eq!(ppu.read_vram(0x2c00), 0x84);
        ppu.write_vram(0x2000, 0x11);
        assert_eq!(ppu.read_vram(0x2000), 0x82);

        cart.write_mapper(0xe000, 0x13);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.read_vram(0x2000), 0x84);
        assert_eq!(ppu.read_vram(0x2c00), 0x84);

        cart.write_mapper(0xe000, 0x00);
        cart.sync_ppu(&mut ppu);
        ppu.write_vram(0x2000, 0x33);
        assert_eq!(ppu.read_vram(0x2000), 0x33);
        assert_eq!(ppu.read_vram(0x2800), 0x33);
    }

    #[test]
    fn sunsoft4_state_round_trip_restores_mapper_and_rom_nametable_mode() {
        let mut rom = mapper_rom(68, 8, 32);
        rom[6] |= 0x02;
        for bank in 0..8usize {
            rom[16 + bank * 0x4000] = bank as u8;
        }
        let chr_start = 16 + 8 * 0x4000;
        for bank in 0..256usize {
            rom[chr_start + bank * 0x400..chr_start + (bank + 1) * 0x400].fill(bank as u8);
        }
        let mut machine = NesMachine::from_rom(&rom).unwrap();
        machine.bus.write8(0x8000, 6);
        machine.bus.write8(0xc000, 3);
        machine.bus.write8(0xd000, 5);
        machine.bus.write8(0xe000, 0x11);
        machine.bus.write8(0xf000, 0x12);
        machine.bus.write8(0x6000, 0xa5);
        let state = machine.save_state().unwrap();

        machine.bus.write8(0xe000, 0);
        machine.bus.write8(0xf000, 0);
        machine.load_state(&state).unwrap();
        assert_eq!(machine.bus.cartridge.sunsoft4.chr_2k[0], 6);
        assert_eq!(machine.bus.cartridge.read_prg(0x8000), 2);
        assert_eq!(machine.bus.read8(0x6000), 0xa5);
        assert_eq!(machine.bus.ppu.read_vram(0x2000), 0x83);
        assert_eq!(machine.bus.ppu.read_vram(0x2400), 0x83);
        assert_eq!(machine.bus.ppu.read_vram(0x2800), 0x85);
        assert_eq!(machine.save_state().unwrap(), state);
    }

    #[test]
    fn sunsoft4_dual_cartridge_submapper_distinguishes_internal_rom_select() {
        let mut rom = nes2_mapper_rom(68, 1, 8, 8);
        for bank in 0..8usize {
            rom[16 + bank * 0x4000] = bank as u8;
        }
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();
        cart.write_mapper(0xf000, 2);
        assert_eq!(cart.read_prg(0x8000), 0xff);
        cart.write_mapper(0xf000, 0x0a);
        assert_eq!(cart.read_prg(0x8000), 2);
    }

    #[test]
    fn fme7_banks_prg_chr_mirroring_and_six_thousand_window() {
        let mut rom = mapper_rom(69, 8, 8);
        for bank in 0..16usize {
            rom[16 + bank * 0x2000] = (0x20 + bank) as u8;
        }
        let chr_start = 16 + 8 * 0x4000;
        for bank in 0..64usize {
            rom[chr_start + bank * 0x0400] = (0x40 + bank) as u8;
        }
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();

        fme7_write(&mut cart, 9, 3);
        fme7_write(&mut cart, 10, 4);
        fme7_write(&mut cart, 11, 5);
        assert_eq!(cart.read_prg(0x8000), 0x23);
        assert_eq!(cart.read_prg(0xa000), 0x24);
        assert_eq!(cart.read_prg(0xc000), 0x25);
        assert_eq!(cart.read_prg(0xe000), 0x2f);

        fme7_write(&mut cart, 0, 7);
        fme7_write(&mut cart, 7, 12);
        fme7_write(&mut cart, 12, 3);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.read_vram(0x0000), 0x47);
        assert_eq!(ppu.read_vram(0x1c00), 0x4c);
        assert_eq!(ppu.mirroring, Mirroring::SingleScreen1);

        fme7_write(&mut cart, 8, 2);
        assert_eq!(cart.read_ram(0x6000), 0x22);
        fme7_write(&mut cart, 8, 0x40);
        assert_eq!(cart.read_ram(0x6000), 0xff);
        cart.write_ram(0x6000, 0xaa);
        fme7_write(&mut cart, 8, 0xc0);
        cart.write_ram(0x6000, 0x5a);
        assert_eq!(cart.read_ram(0x6000), 0x5a);
    }

    #[test]
    fn fme7_irq_decrements_underflows_and_acknowledges() {
        let rom = mapper_rom(69, 4, 1);
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();
        fme7_write(&mut cart, 14, 1);
        fme7_write(&mut cart, 15, 0);
        fme7_write(&mut cart, 13, 0x81);
        cart.tick_cpu_cycle();
        assert_eq!(cart.fme7.irq_counter, 0);
        assert!(!cart.irq_pending());
        cart.tick_cpu_cycle();
        assert_eq!(cart.fme7.irq_counter, 0xffff);
        assert!(cart.irq_pending());

        fme7_write(&mut cart, 13, 0x80);
        assert!(!cart.irq_pending());
        assert!(cart.fme7.irq_counter_enabled);
        assert!(!cart.fme7.irq_output_enabled);
        let before = cart.fme7.irq_counter;
        cart.tick_cpu_cycle();
        assert_eq!(cart.fme7.irq_counter, before.wrapping_sub(1));
    }

    #[test]
    fn sunsoft_5b_clocks_tone_noise_envelope_and_write_disable() {
        let rom = mapper_rom(69, 4, 1);
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();

        cart.write_mapper(0xc000, 7);
        cart.write_mapper(0xe000, 0x09);
        cart.write_mapper(0xc000, 8);
        cart.write_mapper(0xe000, 0x0f);
        assert!(cart.expansion_audio_output() < 0.0);

        cart.write_mapper(0xc000, 7);
        cart.write_mapper(0xe000, 0x08);
        cart.write_mapper(0xc000, 0);
        cart.write_mapper(0xe000, 1);
        cart.write_mapper(0xc000, 1);
        cart.write_mapper(0xe000, 0);
        for _ in 0..16 {
            cart.tick_cpu_cycle();
        }
        assert!(cart.fme7.audio.tone_high[0]);

        cart.write_mapper(0xc000, 6);
        cart.write_mapper(0xe000, 1);
        let noise_before = cart.fme7.audio.noise_lfsr;
        for _ in 0..32 {
            cart.tick_cpu_cycle();
        }
        assert_ne!(cart.fme7.audio.noise_lfsr, noise_before);

        cart.write_mapper(0xc000, 11);
        cart.write_mapper(0xe000, 1);
        cart.write_mapper(0xc000, 12);
        cart.write_mapper(0xe000, 0);
        cart.write_mapper(0xc000, 13);
        cart.write_mapper(0xe000, 0x0c);
        let envelope_before = cart.fme7.audio.envelope_output();
        for _ in 0..16 {
            cart.tick_cpu_cycle();
        }
        assert!(cart.fme7.audio.envelope_output() > envelope_before);

        cart.write_mapper(0xc000, 0xf8);
        cart.write_mapper(0xe000, 0x00);
        assert_eq!(cart.fme7.audio.regs[8], 0x0f);
    }

    #[test]
    fn fme7_state_round_trip_preserves_mapper_and_psg_phase() {
        let rom = mapper_rom(69, 4, 1);
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();
        fme7_write(&mut cart, 9, 2);
        fme7_write(&mut cart, 14, 0x34);
        fme7_write(&mut cart, 15, 0x12);
        fme7_write(&mut cart, 13, 0x81);
        cart.write_mapper(0xc000, 8);
        cart.write_mapper(0xe000, 0x0d);
        for _ in 0..73 {
            cart.tick_cpu_cycle();
        }

        let mut writer = StateWriter::new(PlatformId::Nes, 99);
        save_mapper_state(&mut writer, &cart);
        let bytes = writer.finish();
        let (mut restored, _) = Cartridge::parse(&rom).unwrap();
        let mut reader = StateReader::new(&bytes, PlatformId::Nes, 99).unwrap();
        load_mapper_state(&mut reader, &mut restored).unwrap();
        reader.finish().unwrap();

        assert_eq!(restored.fme7.prg, cart.fme7.prg);
        assert_eq!(restored.fme7.irq_counter, cart.fme7.irq_counter);
        assert_eq!(restored.fme7.audio.regs, cart.fme7.audio.regs);
        assert_eq!(restored.fme7.audio.divider, cart.fme7.audio.divider);
        assert_eq!(
            restored.fme7.audio.tone_counter,
            cart.fme7.audio.tone_counter
        );
        assert_eq!(restored.fme7.audio.noise_lfsr, cart.fme7.audio.noise_lfsr);
    }

    #[test]
    fn namco76_switches_two_kib_chr_and_prg_banks() {
        let mut rom = mapper_rom(76, 8, 16);
        for bank in 0..16usize {
            rom[16 + bank * 0x2000] = (0x20 + bank) as u8;
        }
        let chr_start = 16 + 8 * 0x4000;
        for bank in 0..64usize {
            rom[chr_start + bank * 0x0800..chr_start + (bank + 1) * 0x0800]
                .fill((0x40 + bank) as u8);
        }
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();

        for (register, bank) in [(6, 3), (7, 4)] {
            cart.write_mapper(0x8000, register);
            cart.write_mapper(0x8001, bank);
        }
        assert_eq!(cart.read_prg(0x8000), 0x23);
        assert_eq!(cart.read_prg(0xa000), 0x24);
        assert_eq!(cart.read_prg(0xc000), 0x2e);
        assert_eq!(cart.read_prg(0xe000), 0x2f);

        for (register, bank) in [(2, 5), (3, 6), (4, 7), (5, 8)] {
            cart.write_mapper(0x8000, register);
            cart.write_mapper(0x8001, bank);
        }
        cart.sync_ppu(&mut ppu);
        for (address, expected) in [
            (0x0000, 0x45),
            (0x0800, 0x46),
            (0x1000, 0x47),
            (0x1800, 0x48),
        ] {
            assert_eq!(ppu.read_vram(address), expected);
        }

        cart.write_mapper(0x8000, 0);
        cart.write_mapper(0x8001, 0x3f);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.read_vram(0x0000), 0x45);
        assert_eq!(cart.read_ram(0x6000), 0xff);
    }

    #[test]
    fn namco3425_routes_ciram_from_chr_a15() {
        let rom = namco3425_test_rom();
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        assert!(!cart.ram_readable());
        for (register, bank) in [(6, 3), (7, 4), (0, 0x20), (1, 0x02)] {
            cart.write_mapper(0x8000, register);
            cart.write_mapper(0x8001, bank);
        }
        assert_eq!(cart.read_prg(0x8000), 3);
        assert_eq!(cart.read_prg(0xa000), 4);
        assert_eq!(cart.read_prg(0xc000), 14);
        assert_eq!(cart.read_prg(0xe000), 15);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.read_vram(0x0000), 32);
        assert_eq!(ppu.read_vram(0x0400), 33);
        assert_eq!(ppu.read_vram(0x0800), 2);
        assert_eq!(ppu.read_vram(0x0c00), 3);
        ppu.nametable[0] = 0x11;
        ppu.nametable[0x400] = 0x22;
        assert_eq!(ppu.read_vram(0x2000), 0x22);
        assert_eq!(ppu.read_vram(0x2400), 0x22);
        assert_eq!(ppu.read_vram(0x2800), 0x11);
        assert_eq!(ppu.read_vram(0x2c00), 0x11);
        cart.write_mapper(0x8000, 1);
        cart.write_mapper(0x8001, 0x22);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.read_vram(0x2000), 0x22);
        assert_eq!(ppu.read_vram(0x2400), 0x22);
        assert_eq!(ppu.read_vram(0x2800), 0x22);
        assert_eq!(ppu.read_vram(0x2c00), 0x22);
    }

    #[test]
    fn namco206_switches_mixed_chr_and_prg_banks() {
        let mut rom = mapper_rom(206, 8, 8);
        for bank in 0..16usize {
            rom[16 + bank * 0x2000] = (0x20 + bank) as u8;
        }
        let chr_start = 16 + 8 * 0x4000;
        for bank in 0..64usize {
            rom[chr_start + bank * 0x0400..chr_start + (bank + 1) * 0x0400]
                .fill((0x40 + bank) as u8);
        }
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        for (register, bank) in [
            (6, 3),
            (7, 4),
            (0, 5),
            (1, 6),
            (2, 8),
            (3, 9),
            (4, 10),
            (5, 11),
        ] {
            cart.write_mapper(0x8000, register);
            cart.write_mapper(0x8001, bank);
        }
        assert_eq!(cart.read_prg(0x8000), 0x23);
        assert_eq!(cart.read_prg(0xa000), 0x24);
        assert_eq!(cart.read_prg(0xc000), 0x2e);
        assert_eq!(cart.read_prg(0xe000), 0x2f);
        cart.sync_ppu(&mut ppu);
        for (address, expected) in [
            (0x0000, 0x44),
            (0x0400, 0x45),
            (0x0800, 0x46),
            (0x0c00, 0x47),
            (0x1000, 0x48),
            (0x1400, 0x49),
            (0x1800, 0x4a),
            (0x1c00, 0x4b),
        ] {
            assert_eq!(ppu.read_vram(address), expected);
        }
        assert_eq!(ppu.mirroring, Mirroring::Horizontal);
        assert_eq!(cart.read_ram(0x6000), 0xff);
    }

    #[test]
    fn namco206_submapper_one_keeps_thirty_two_kib_prg_unbanked() {
        let mut rom = nes2_mapper_rom(206, 1, 2, 8);
        for bank in 0..4usize {
            rom[16 + bank * 0x2000] = (0x30 + bank) as u8;
        }
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();
        cart.write_mapper(0x8000, 6);
        cart.write_mapper(0x8001, 0);
        cart.write_mapper(0x8000, 7);
        cart.write_mapper(0x8001, 0);
        assert_eq!(cart.read_prg(0x8000), 0x30);
        assert_eq!(cart.read_prg(0xa000), 0x31);
        assert_eq!(cart.read_prg(0xc000), 0x32);
        assert_eq!(cart.read_prg(0xe000), 0x33);
    }

    #[test]
    fn namco88_and_154_force_upper_chr_half_and_154_one_screen_mirroring() {
        for mapper in [88u16, 154] {
            let mut rom = mapper_rom(mapper, 8, 16);
            let chr_start = 16 + 8 * 0x4000;
            for bank in 0..128usize {
                rom[chr_start + bank * 0x0400..chr_start + (bank + 1) * 0x0400].fill(bank as u8);
            }
            let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
            cart.write_mapper(0x8000, 0);
            cart.write_mapper(0x8001, 4);
            cart.write_mapper(0x8000, 2);
            cart.write_mapper(0x8001, 5);
            cart.sync_ppu(&mut ppu);
            assert_eq!(ppu.read_vram(0x0000), 4);
            assert_eq!(ppu.read_vram(0x0400), 5);
            assert_eq!(ppu.read_vram(0x1000), 69);

            if mapper == 154 {
                cart.write_mapper(0xe000, 0x40);
                cart.sync_ppu(&mut ppu);
                assert_eq!(ppu.mirroring, Mirroring::SingleScreen1);
                cart.write_mapper(0xa000, 0x00);
                cart.sync_ppu(&mut ppu);
                assert_eq!(ppu.mirroring, Mirroring::SingleScreen0);
            } else {
                assert_eq!(ppu.mirroring, Mirroring::Horizontal);
            }
        }
    }

    #[test]
    fn mmc2_and_mmc4_use_their_native_prg_bank_geometry() {
        let mut mmc2 = mapper_rom(9, 8, 8);
        for bank in 0..16usize {
            mmc2[16 + bank * 0x2000] = (0x20 + bank) as u8;
        }
        let (mut mmc2_cart, _) = Cartridge::parse(&mmc2).unwrap();
        mmc2_cart.write_mapper(0xa000, 3);
        assert_eq!(mmc2_cart.read_prg(0x8000), 0x23);
        assert_eq!(mmc2_cart.read_prg(0xa000), 0x2d);
        assert_eq!(mmc2_cart.read_prg(0xc000), 0x2e);
        assert_eq!(mmc2_cart.read_prg(0xe000), 0x2f);
        assert_eq!(mmc2_cart.read_ram(0x6000), 0xff);

        let mut mmc4 = mapper_rom(10, 8, 8);
        for bank in 0..8usize {
            mmc4[16 + bank * 0x4000] = (0x40 + bank) as u8;
        }
        let (mut mmc4_cart, _) = Cartridge::parse(&mmc4).unwrap();
        mmc4_cart.write_mapper(0xa000, 3);
        assert_eq!(mmc4_cart.read_prg(0x8000), 0x43);
        assert_eq!(mmc4_cart.read_prg(0xc000), 0x47);
        mmc4_cart.write_ram(0x6000, 0x5a);
        assert_eq!(mmc4_cart.read_ram(0x6000), 0x5a);
    }

    #[test]
    fn mmc2_chr_latches_select_four_kib_banks_and_mirroring() {
        let mut rom = mapper_rom(9, 4, 8);
        let chr_start = 16 + 4 * 0x4000;
        for bank in 0..16usize {
            rom[chr_start + bank * 0x1000] = (0x60 + bank) as u8;
        }
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        cart.write_mapper(0xb000, 1);
        cart.write_mapper(0xc000, 2);
        cart.write_mapper(0xd000, 3);
        cart.write_mapper(0xe000, 4);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.read_vram(0x0000), 0x62);
        assert_eq!(ppu.read_vram(0x1000), 0x64);

        assert!(cart.observe_ppu_latch_address(0x0fd8));
        assert!(cart.observe_ppu_latch_address(0x1fd8));
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.read_vram(0x0000), 0x61);
        assert_eq!(ppu.read_vram(0x1000), 0x63);

        cart.write_mapper(0xf000, 1);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.mirroring, Mirroring::Horizontal);
        cart.write_mapper(0xf000, 0);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.mirroring, Mirroring::Vertical);
    }

    #[test]
    fn mmc2_latch_is_driven_by_background_pattern_fetches() {
        let rom = mapper_rom(9, 4, 8);
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        cart.write_mapper(0xb000, 1);
        cart.write_mapper(0xc000, 2);
        cart.sync_ppu(&mut ppu);
        assert!(cart.mmc2.latch_fe[0]);

        ppu.mask = 0x08;
        ppu.nametable[0] = 0xfd;
        ppu.scanline = 261;
        ppu.cycle = 320;
        let mut bus = NesBus::new(cart, ppu);
        bus.tick_ppu(7);

        assert!(!bus.cartridge.mmc2.latch_fe[0]);
        assert_eq!(bus.ppu.chr_map[0], 4);
    }

    #[test]
    fn mmc2_latch_is_driven_by_sprite_pattern_fetches() {
        let rom = mapper_rom(9, 4, 8);
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        cart.write_mapper(0xb000, 1);
        cart.write_mapper(0xc000, 2);
        cart.sync_ppu(&mut ppu);
        assert!(cart.mmc2.latch_fe[0]);

        ppu.mask = 0x10;
        ppu.oam.fill(0xf0);
        ppu.oam[0] = 0;
        ppu.oam[1] = 0xfd;
        ppu.scanline = 0;
        ppu.cycle = 256;
        let mut bus = NesBus::new(cart, ppu);
        bus.tick_ppu(7);

        assert!(!bus.cartridge.mmc2.latch_fe[0]);
        assert_eq!(bus.ppu.chr_map[0], 4);
    }

    #[test]
    fn mmc3_switches_prg_chr_and_raises_scanline_irq() {
        let mut rom = mapper_rom(4, 4, 2);
        for bank in 0..8usize {
            rom[16 + bank * 0x2000] = (0x20 + bank) as u8;
        }
        let chr_start = 16 + 4 * 0x4000;
        for bank in 0..16usize {
            rom[chr_start + bank * 0x400] = (0x60 + bank) as u8;
        }
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        cart.write_mapper(0x8000, 6);
        cart.write_mapper(0x8001, 3);
        assert_eq!(cart.read_prg(0x8000), 0x23);
        assert_eq!(cart.read_prg(0xe000), 0x27);
        cart.write_mapper(0x8000, 0x46);
        cart.write_mapper(0x8001, 2);
        assert_eq!(cart.read_prg(0x8000), 0x26);
        assert_eq!(cart.read_prg(0xc000), 0x22);
        cart.write_mapper(0x8000, 2);
        cart.write_mapper(0x8001, 5);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.read_vram(0x1000), 0x65);
        cart.write_mapper(0xc000, 2);
        cart.write_mapper(0xc001, 0);
        cart.write_mapper(0xe001, 0);
        for _ in 0..3 {
            for _ in 0..8 {
                cart.observe_ppu_a12(false);
            }
            cart.observe_ppu_a12(true);
        }
        assert!(cart.irq_pending());
        cart.write_mapper(0xe000, 0);
        assert!(!cart.irq_pending());
    }

    #[test]
    fn mapper37_applies_nintendo_multicart_outer_windows_and_mmc3_irq() {
        let mut rom = mapper_rom(37, 16, 32);
        for bank in 0..32usize {
            rom[16 + bank * 0x2000] = bank as u8;
        }
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();

        cart.write_mapper(0x8000, 6);
        cart.write_mapper(0x8001, 9);
        assert_eq!(cart.read_prg(0x8000), 1);
        assert_eq!(cart.read_prg(0xe000), 7);
        cart.write_ram(0x6000, 3);
        assert_eq!(cart.read_prg(0x8000), 9);
        assert_eq!(cart.read_prg(0xe000), 15);
        cart.write_ram(0x6000, 4);
        assert_eq!(cart.read_prg(0x8000), 25);
        assert_eq!(cart.read_prg(0xe000), 31);

        cart.write_mapper(0xa001, 0xc0);
        cart.write_ram(0x6000, 0);
        assert_eq!(cart.simple_reg, 4);
        cart.write_mapper(0xa001, 0x80);
        cart.write_ram(0x6000, 0);
        cart.write_mapper(0x8000, 2);
        cart.write_mapper(0x8001, 5);
        assert_eq!(cart.chr_map()[4], 5);
        cart.write_ram(0x6000, 4);
        assert_eq!(cart.chr_map()[4], 133);

        cart.write_mapper(0xa000, 1);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.mirroring, Mirroring::Horizontal);
        cart.write_mapper(0xc000, 1);
        cart.write_mapper(0xc001, 0);
        cart.write_mapper(0xe001, 0);
        for _ in 0..2 {
            for _ in 0..8 {
                cart.observe_ppu_a12(false);
            }
            cart.observe_ppu_a12(true);
        }
        assert!(cart.irq_pending());
    }

    #[test]
    fn mapper47_selects_independent_128k_prg_and_chr_blocks() {
        let mut rom = mapper_rom(47, 16, 32);
        for bank in 0..32usize {
            rom[16 + bank * 0x2000] = (0x40 + bank) as u8;
        }
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();

        cart.write_mapper(0x8000, 6);
        cart.write_mapper(0x8001, 2);
        assert_eq!(cart.read_prg(0x8000), 0x42);
        assert_eq!(cart.read_prg(0xe000), 0x4f);
        cart.write_ram(0x6000, 1);
        assert_eq!(cart.read_prg(0x8000), 0x52);
        assert_eq!(cart.read_prg(0xe000), 0x5f);

        cart.write_mapper(0x8000, 2);
        cart.write_mapper(0x8001, 9);
        assert_eq!(cart.chr_map()[4], 137);
        cart.write_ram(0x6000, 0);
        assert_eq!(cart.chr_map()[4], 9);
        assert_eq!(cart.read_ram(0x6000), 0xff);
    }

    #[test]
    fn mapper47_state_round_trip_preserves_outer_bank_and_mmc3_phase() {
        let rom = mapper_rom(47, 16, 32);
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();
        cart.write_mapper(0x8000, 6);
        cart.write_mapper(0x8001, 7);
        cart.write_mapper(0xc000, 3);
        cart.write_mapper(0xc001, 0);
        cart.write_mapper(0xe001, 0);
        cart.write_ram(0x6000, 1);
        for _ in 0..8 {
            cart.observe_ppu_a12(false);
        }
        cart.observe_ppu_a12(true);

        let mut writer = StateWriter::new(PlatformId::Nes, 99);
        save_mapper_state(&mut writer, &cart);
        let bytes = writer.finish();
        let (mut restored, _) = Cartridge::parse(&rom).unwrap();
        let mut reader = StateReader::new(&bytes, PlatformId::Nes, 99).unwrap();
        load_mapper_state(&mut reader, &mut restored).unwrap();
        reader.finish().unwrap();

        assert_eq!(restored.simple_reg, 1);
        assert_eq!(restored.mmc3.regs, cart.mmc3.regs);
        assert_eq!(restored.mmc3.irq_counter, cart.mmc3.irq_counter);
        assert_eq!(restored.mmc3.irq_reload, cart.mmc3.irq_reload);
        assert_eq!(restored.read_prg(0x8000), cart.read_prg(0x8000));
        assert_eq!(restored.chr_map(), cart.chr_map());
    }

    #[test]
    fn tqrom_selects_chr_rom_or_ram_per_mmc3_chr_bank() {
        let mut rom = mapper_rom(119, 8, 16);
        let chr_start = 16 + 8 * 0x4000;
        for bank in 0..128usize {
            rom[chr_start + bank * 0x400..chr_start + (bank + 1) * 0x400].fill(bank as u8);
        }
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        assert_eq!(ppu.chr_rom_bank_count, 128);
        assert_eq!(ppu.chr_bank_count, 136);
        assert!(!cart.ram_readable());

        for (register, bank) in [(2, 0x05), (3, 0x81), (4, 0x45)] {
            cart.write_mapper(0x8000, register);
            cart.write_mapper(0x8001, bank);
        }
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.read_vram(0x1000), 5);
        assert_eq!(ppu.read_vram(0x1400), 65);
        assert_eq!(ppu.read_vram(0x1800), 0);

        ppu.write_vram(0x1000, 0xaa);
        assert_eq!(ppu.read_vram(0x1000), 5);
        ppu.write_vram(0x1800, 0xbb);
        assert_eq!(ppu.read_vram(0x1800), 0xbb);

        cart.write_mapper(0x8000, 4);
        cart.write_mapper(0x8001, 0x7d);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.read_vram(0x1800), 0xbb);
    }

    #[test]
    fn tqrom_save_state_restores_chr_ram_without_serializing_chr_rom() {
        let mut rom = mapper_rom(119, 8, 8);
        let chr_start = 16 + 8 * 0x4000;
        rom[chr_start + 5 * 0x400] = 0x55;
        let mut machine = NesMachine::from_rom(&rom).unwrap();
        machine.bus.cartridge.write_mapper(0x8000, 2);
        machine.bus.cartridge.write_mapper(0x8001, 0x05);
        machine.bus.cartridge.write_mapper(0x8000, 3);
        machine.bus.cartridge.write_mapper(0x8001, 0x45);
        machine.bus.cartridge.sync_ppu(&mut machine.bus.ppu);
        machine.bus.ppu.write_vram(0x1400, 0xa5);
        let state = machine.save_state().unwrap();

        let mut restored = NesMachine::from_rom(&rom).unwrap();
        restored.load_state(&state).unwrap();
        assert_eq!(restored.bus.ppu.read_vram(0x1000), 0x55);
        assert_eq!(restored.bus.ppu.read_vram(0x1400), 0xa5);
        assert_eq!(restored.save_state().unwrap(), state);
    }

    #[test]
    fn txsrom_routes_ciram_through_chr_a17_and_ignores_a000() {
        let rom = txsrom_test_rom();
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        cart.write_mapper(0x8000, 6);
        cart.write_mapper(0x8001, 3);
        assert_eq!(cart.read_prg(0x8000), 3);
        assert_eq!(cart.read_prg(0xc000), 30);
        for (register, value) in [
            (0, 0x80),
            (1, 0x02),
            (2, 0x83),
            (3, 0x04),
            (4, 0x85),
            (5, 0x06),
        ] {
            cart.write_mapper(0x8000, register);
            cart.write_mapper(0x8001, value);
        }
        cart.sync_ppu(&mut ppu);
        ppu.nametable[0] = 0x11;
        ppu.nametable[0x400] = 0x22;
        assert_eq!(ppu.read_vram(0x0000), 0);
        assert_eq!(ppu.read_vram(0x0400), 1);
        assert_eq!(ppu.read_vram(0x2000), 0x22);
        assert_eq!(ppu.read_vram(0x2400), 0x22);
        assert_eq!(ppu.read_vram(0x2800), 0x11);
        assert_eq!(ppu.read_vram(0x2c00), 0x11);
        cart.write_mapper(0xa000, 1);
        assert!(!cart.mmc3.mirror_horizontal);
        cart.write_mapper(0x8000, 0x82);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.read_vram(0x0000), 3);
        assert_eq!(ppu.read_vram(0x0400), 4);
        assert_eq!(ppu.read_vram(0x2000), 0x22);
        assert_eq!(ppu.read_vram(0x2400), 0x11);
        assert_eq!(ppu.read_vram(0x2800), 0x22);
        assert_eq!(ppu.read_vram(0x2c00), 0x11);
    }

    #[test]
    fn txsrom_uses_mmc3_filtered_scanline_irq() {
        let rom = txsrom_test_rom();
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();
        cart.write_mapper(0xc000, 2);
        cart.write_mapper(0xc001, 0);
        cart.write_mapper(0xe001, 0);
        for _ in 0..3 {
            for _ in 0..8 {
                cart.observe_ppu_a12(false);
            }
            cart.observe_ppu_a12(true);
        }
        assert!(cart.irq_pending());
        cart.write_mapper(0xe000, 0);
        assert!(!cart.irq_pending());
    }

    #[test]
    fn mmc3_ppuaddr_writes_clock_filtered_a12() {
        let rom = mapper_rom(4, 2, 1);
        let (cart, ppu) = Cartridge::parse(&rom).unwrap();
        let mut bus = NesBus::new(cart, ppu);
        bus.write8(0xc000, 1);
        bus.write8(0xc001, 0);
        bus.write8(0xe001, 0);

        bus.write8(0x2006, 0x00);
        bus.write8(0x2006, 0x00);
        bus.tick_ppu(12);
        bus.write8(0x2006, 0x10);
        bus.write8(0x2006, 0x00);
        assert_eq!(bus.cartridge.mmc3.irq_counter, 1);
        assert!(!bus.cartridge.irq_pending());

        bus.write8(0x2006, 0x00);
        bus.write8(0x2006, 0x00);
        bus.tick_ppu(12);
        bus.write8(0x2006, 0x10);
        bus.write8(0x2006, 0x00);
        assert_eq!(bus.cartridge.mmc3.irq_counter, 0);
        assert!(bus.cartridge.irq_pending());
    }

    #[test]
    fn mc_acc_clocks_first_of_each_eight_falling_edges() {
        let rom = mapper4_nes2_rom(3, 2, 1);
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();
        cart.write_mmc3(0xc000, 2);
        cart.write_mmc3(0xc001, 0);
        cart.write_mmc3(0xe001, 0);

        cart.observe_ppu_a12(true);
        cart.observe_ppu_a12(false);
        assert_eq!(cart.mmc3.irq_counter, 2);
        assert_eq!(cart.mmc3.mc_acc_fall_counter, 1);

        for _ in 0..7 {
            cart.observe_ppu_a12(true);
            cart.observe_ppu_a12(false);
        }
        assert_eq!(cart.mmc3.irq_counter, 2);
        assert_eq!(cart.mmc3.mc_acc_fall_counter, 0);

        cart.observe_ppu_a12(true);
        cart.observe_ppu_a12(false);
        assert_eq!(cart.mmc3.irq_counter, 1);
        cart.write_mmc3(0xc001, 0);
        assert_eq!(cart.mmc3.mc_acc_fall_counter, 0);
        cart.observe_ppu_a12(true);
        cart.observe_ppu_a12(false);
        assert_eq!(cart.mmc3.irq_counter, 2);
    }

    #[test]
    fn mc_acc_irq_is_four_ppu_dots_later_than_sharp_mmc3() {
        let mut buses = Vec::new();
        for submapper in [0u8, 3u8] {
            let rom = mapper4_nes2_rom(submapper, 2, 1);
            let (cart, mut ppu) = Cartridge::parse(&rom).unwrap();
            ppu.mask = 0x18;
            ppu.ctrl = 0x08;
            let mut bus = NesBus::new(cart, ppu);
            bus.write8(0xc000, 0);
            bus.write8(0xc001, 0);
            bus.write8(0xe001, 0);
            buses.push(bus);
        }

        for bus in &mut buses {
            bus.tick_ppu(259);
            assert!(!bus.cartridge.irq_pending());
        }
        buses[0].tick_ppu(1);
        buses[1].tick_ppu(1);
        assert!(buses[0].cartridge.irq_pending());
        assert!(!buses[1].cartridge.irq_pending());
        buses[1].tick_ppu(3);
        assert!(!buses[1].cartridge.irq_pending());
        buses[1].tick_ppu(1);
        assert!(buses[1].cartridge.irq_pending());
    }

    #[test]
    fn mmc6_uses_internal_one_kib_ram_and_per_half_protection() {
        let rom = mapper4_nes2_rom(1, 2, 1);
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();
        assert_eq!(cart.prg_ram.len(), 1024);
        assert_eq!(cart.read_ram(0x7000), 0xff);

        cart.write_mmc3(0x8000, 0x20);
        cart.write_mmc3(0xa001, 0x03);
        cart.write_ram(0x7000, 0x5a);
        assert_eq!(cart.read_ram(0x7000), 0x5a);
        assert_eq!(cart.read_ram(0x7400), 0x5a);
        assert_eq!(cart.read_ram(0x7200), 0x00);
        cart.write_ram(0x7200, 0xa5);
        assert_eq!(cart.prg_ram[0x200], 0x00);

        cart.write_mmc3(0xa001, 0x0c);
        assert_eq!(cart.read_ram(0x7000), 0x00);
        cart.write_ram(0x7200, 0xa5);
        assert_eq!(cart.read_ram(0x7200), 0xa5);

        cart.write_mmc3(0x8000, 0x00);
        cart.write_mmc3(0xa001, 0x0f);
        assert_eq!(cart.mmc3.prg_ram_protect, 0);
        assert_eq!(cart.read_ram(0x7000), 0xff);
    }

    #[test]
    fn mmc3_submapper_two_keeps_header_mirroring_hard_wired() {
        let mut rom = mapper4_nes2_rom(2, 2, 1);
        rom[6] |= 1;
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        assert_eq!(ppu.mirroring, Mirroring::Vertical);
        cart.write_mmc3(0xa000, 1);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.mirroring, Mirroring::Vertical);
    }

    #[test]
    fn nec_mmc3_zero_latch_does_not_repeat_irq_like_sharp_mmc3() {
        for (submapper, expected_irq) in [(0u8, true), (4u8, false)] {
            let rom = mapper4_nes2_rom(submapper, 2, 1);
            let (mut cart, _) = Cartridge::parse(&rom).unwrap();
            cart.write_mmc3(0xc000, 0);
            cart.write_mmc3(0xc001, 0);
            cart.write_mmc3(0xe001, 0);
            for _ in 0..8 {
                cart.observe_ppu_a12(false);
            }
            cart.observe_ppu_a12(true);
            assert_eq!(cart.irq_pending(), expected_irq, "submapper {submapper}");
        }
    }

    #[test]
    fn axrom_switches_32k_prg_and_one_screen_mirroring() {
        let mut rom = mapper_rom(7, 4, 0);
        rom[16] = 0x31;
        rom[16 + 0x8000] = 0x72;
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        assert_eq!(cart.read_prg(0x8000), 0x31);
        cart.write_mapper(0x8000, 0x11);
        cart.sync_ppu(&mut ppu);
        assert_eq!(cart.read_prg(0x8000), 0x72);
        assert_eq!(ppu.mirroring, Mirroring::SingleScreen1);
    }

    #[test]
    fn gxrom_and_color_dreams_switch_prg_and_chr_together() {
        for mapper in [11u16, 66u16] {
            let mut rom = mapper_rom(mapper, 4, 4);
            rom[16] = 0x10;
            rom[16 + 0x8000] = 0x20;
            let chr_start = 16 + 4 * 0x4000;
            rom[chr_start] = 0x40;
            rom[chr_start + 0x2000] = 0x50;
            rom[chr_start + 0x4000] = 0x60;
            let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
            let value = if mapper == 11 { 0x21 } else { 0x11 };
            cart.write_mapper(0x8000, value);
            cart.sync_ppu(&mut ppu);
            assert_eq!(cart.read_prg(0x8000), 0x20);
            assert_eq!(ppu.read_vram(0), if mapper == 11 { 0x60 } else { 0x50 });
        }
    }
    #[test]
    fn cprom_banks_upper_chr_ram_half() {
        let rom = mapper_rom(13, 2, 0);
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        assert_eq!(ppu.chr.len(), 16 * 1024);
        ppu.chr[0] = 0x11;
        ppu.chr[8 * 1024] = 0x77;
        cart.write_mapper(0x8000, 2);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.read_vram(0x0000), 0x11);
        assert_eq!(ppu.read_vram(0x1000), 0x77);
    }

    #[test]
    fn mapper34_distinguishes_bnrom_and_nina001() {
        let mut bnrom = mapper_rom(34, 8, 0);
        for bank in 0..4usize {
            bnrom[16 + bank * 0x8000] = (0x20 + bank) as u8;
        }
        let (mut bnrom_cart, _) = Cartridge::parse(&bnrom).unwrap();
        assert!(!bnrom_cart.is_nina_001());
        bnrom_cart.write_mapper(0x8000, 2);
        assert_eq!(bnrom_cart.read_prg(0x8000), 0x22);

        let mut nina = mapper_rom(34, 4, 8);
        for bank in 0..2usize {
            nina[16 + bank * 0x8000] = (0x30 + bank) as u8;
        }
        let chr_start = 16 + 4 * 0x4000;
        for bank in 0..16usize {
            nina[chr_start + bank * 0x1000] = (0x40 + bank) as u8;
        }
        let (mut nina_cart, mut ppu) = Cartridge::parse(&nina).unwrap();
        assert!(nina_cart.is_nina_001());
        assert!(nina_cart.write_low_mapper(0x7ffd, 1));
        assert!(nina_cart.write_low_mapper(0x7ffe, 2));
        assert!(nina_cart.write_low_mapper(0x7fff, 3));
        nina_cart.sync_ppu(&mut ppu);
        assert_eq!(nina_cart.read_prg(0x8000), 0x31);
        assert_eq!(ppu.read_vram(0x0000), 0x42);
        assert_eq!(ppu.read_vram(0x1000), 0x43);
    }

    #[test]
    fn camerica_banks_prg_and_supports_fire_hawk_mirroring() {
        let mut rom = mapper_rom(71, 8, 0);
        for bank in 0..8usize {
            rom[16 + bank * 0x4000] = (0x50 + bank) as u8;
        }
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        cart.write_mapper(0xc000, 3);
        assert_eq!(cart.read_prg(0x8000), 0x53);
        assert_eq!(cart.read_prg(0xc000), 0x57);
        cart.write_mapper(0x9000, 0x10);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.mirroring, Mirroring::SingleScreen1);
    }

    #[test]
    fn mapper87_reverses_two_chr_bank_bits() {
        let mut rom = mapper_rom(87, 2, 4);
        let chr_start = 16 + 2 * 0x4000;
        for bank in 0..4usize {
            rom[chr_start + bank * 0x2000] = (0x60 + bank) as u8;
        }
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        assert!(cart.write_low_mapper(0x6000, 1));
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.read_vram(0), 0x62);
        assert!(cart.write_low_mapper(0x6000, 2));
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.read_vram(0), 0x61);
    }

    #[test]
    fn mapper101_keeps_chr_bank_bits_in_order_and_exposes_no_prg_ram() {
        let mut rom = mapper_rom(101, 2, 8);
        let chr_start = 16 + 2 * 0x4000;
        for bank in 0..8usize {
            rom[chr_start + bank * 0x2000] = (0x70 + bank) as u8;
        }
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        assert!(cart.prg_ram.is_empty());
        assert!(cart.write_low_mapper(0x6000, 5));
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.read_vram(0), 0x75);
    }

    #[test]
    fn un1rom_uses_bits_five_through_two_for_prg_bank() {
        let mut rom = mapper_rom(94, 8, 0);
        for bank in 0..8usize {
            rom[16 + bank * 0x4000] = (0x70 + bank) as u8;
        }
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();
        cart.write_mapper(0x8000, 0x0c);
        assert_eq!(cart.read_prg(0x8000), 0x73);
        assert_eq!(cart.read_prg(0xc000), 0x77);
    }

    #[test]
    fn mapper140_moves_gxrom_register_to_low_window() {
        let mut rom = mapper_rom(140, 8, 4);
        for bank in 0..4usize {
            rom[16 + bank * 0x8000] = (0x80 + bank) as u8;
        }
        let chr_start = 16 + 8 * 0x4000;
        for bank in 0..4usize {
            rom[chr_start + bank * 0x2000] = (0x90 + bank) as u8;
        }
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        assert!(cart.write_low_mapper(0x6000, 0x21));
        cart.sync_ppu(&mut ppu);
        assert_eq!(cart.read_prg(0x8000), 0x82);
        assert_eq!(ppu.read_vram(0), 0x91);
    }

    #[test]
    fn mapper180_keeps_first_prg_bank_fixed_and_switches_upper() {
        let mut rom = mapper_rom(180, 8, 0);
        for bank in 0..8usize {
            rom[16 + bank * 0x4000] = (0xa0 + bank) as u8;
        }
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();
        cart.write_mapper(0x8000, 5);
        assert_eq!(cart.read_prg(0x8000), 0xa0);
        assert_eq!(cart.read_prg(0xc000), 0xa5);
    }

    #[test]
    fn sunsoft1_selects_independent_four_kib_chr_banks() {
        let mut rom = mapper_rom(184, 2, 4);
        let chr_start = 16 + 2 * 0x4000;
        for bank in 0..8usize {
            rom[chr_start + bank * 0x1000] = (0xb0 + bank) as u8;
        }
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        assert!(cart.write_low_mapper(0x6000, 0x65));
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.read_vram(0x0000), 0xb5);
        assert_eq!(ppu.read_vram(0x1000), 0xb6);
    }

    #[test]
    fn mapper241_banks_prg_without_losing_wram() {
        let mut rom = mapper_rom(241, 8, 0);
        for bank in 0..4usize {
            rom[16 + bank * 0x8000] = (0xc0 + bank) as u8;
        }
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();
        cart.write_mapper(0x8000, 3);
        assert_eq!(cart.read_prg(0x8000), 0xc3);
        cart.write_ram(0x6123, 0x5a);
        assert_eq!(cart.read_ram(0x6123), 0x5a);
    }

    #[test]
    fn nes2_header_decodes_submapper_and_nonvolatile_ram() {
        let mut rom = vec![0u8; 16 + 32 * 1024 + 8 * 1024];
        rom[..4].copy_from_slice(b"NES\x1a");
        rom[4] = 2;
        rom[5] = 1;
        rom[6] = 0x22; // mapper 2 + battery flag
        rom[7] = 0x08; // NES 2.0 marker
        rom[8] = 0x30; // submapper 3, mapper high bits zero
        rom[10] = 0x70; // 8 KiB PRG NVRAM
        let (cart, _) = Cartridge::parse(&rom).unwrap();
        assert_eq!(cart.mapper, 2);
        assert_eq!(cart.submapper, 3);
        assert_eq!(cart.prg_ram.len(), 8 * 1024);
        assert_eq!(cart.battery_len, 8 * 1024);
    }

    #[test]
    fn nes2_exponent_multiplier_rom_size_is_checked() {
        let encoded = (10u8 << 2) | 1;
        assert_eq!(nes2_rom_size(encoded, 0x0f, 16 * 1024).unwrap(), 3072);
    }
}
