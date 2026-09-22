use crate::kernel::AudioBuffer;
use crate::state::{StateReader, StateWriter};

const NTSC_CPU_HZ: u64 = 1_789_773;
const SAMPLE_RATE: u32 = 48_000;
const LENGTH_TABLE: [u8; 32] = [
    10, 254, 20, 2, 40, 4, 80, 6, 160, 8, 60, 10, 14, 12, 26, 14, 12, 16, 24, 18, 48, 20, 96, 22,
    192, 24, 72, 26, 16, 28, 32, 30,
];
const DUTY_TABLE: [[u8; 8]; 4] = [
    [0, 1, 0, 0, 0, 0, 0, 0],
    [0, 1, 1, 0, 0, 0, 0, 0],
    [0, 1, 1, 1, 1, 0, 0, 0],
    [1, 0, 0, 1, 1, 1, 1, 1],
];
const TRIANGLE_TABLE: [u8; 32] = [
    15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,
    13, 14, 15,
];
#[derive(Debug, Clone, Default)]
struct Envelope {
    loop_flag: bool,
    constant: bool,
    period: u8,
    divider: u8,
    decay: u8,
    start: bool,
}

impl Envelope {
    fn write(&mut self, value: u8) {
        self.loop_flag = value & 0x20 != 0;
        self.constant = value & 0x10 != 0;
        self.period = value & 0x0f;
    }
    fn restart(&mut self) {
        self.start = true;
    }
    fn quarter_frame(&mut self) {
        if self.start {
            self.start = false;
            self.decay = 15;
            self.divider = self.period;
        } else if self.divider == 0 {
            self.divider = self.period;
            if self.decay > 0 {
                self.decay -= 1;
            } else if self.loop_flag {
                self.decay = 15;
            }
        } else {
            self.divider -= 1;
        }
    }
    fn volume(&self) -> u8 {
        if self.constant {
            self.period
        } else {
            self.decay
        }
    }
}
#[derive(Debug, Clone, Default)]
struct Pulse {
    enabled: bool,
    channel_one: bool,
    duty: u8,
    sequence: u8,
    timer_period: u16,
    timer: u16,
    length: u8,
    envelope: Envelope,
    sweep_enabled: bool,
    sweep_period: u8,
    sweep_negate: bool,
    sweep_shift: u8,
    sweep_divider: u8,
    sweep_reload: bool,
}

impl Pulse {
    fn new(channel_one: bool) -> Self {
        Self {
            channel_one,
            ..Self::default()
        }
    }
    fn write_control(&mut self, value: u8) {
        self.duty = value >> 6;
        self.envelope.write(value);
    }
    fn write_sweep(&mut self, value: u8) {
        self.sweep_enabled = value & 0x80 != 0;
        self.sweep_period = (value >> 4) & 7;
        self.sweep_negate = value & 0x08 != 0;
        self.sweep_shift = value & 7;
        self.sweep_reload = true;
    }
    fn write_timer_low(&mut self, value: u8) {
        self.timer_period = (self.timer_period & 0x0700) | value as u16;
    }
    fn write_timer_high(&mut self, value: u8) {
        self.timer_period = (self.timer_period & 0x00ff) | (((value as u16) & 7) << 8);
        if self.enabled {
            self.length = LENGTH_TABLE[(value >> 3) as usize];
        }
        self.sequence = 0;
        self.envelope.restart();
    }
    fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
        if !enabled {
            self.length = 0;
        }
    }
    fn tick_timer(&mut self) {
        if self.timer == 0 {
            self.timer = self.timer_period;
            self.sequence = (self.sequence + 1) & 7;
        } else {
            self.timer -= 1;
        }
    }
    fn target_period(&self) -> u16 {
        if self.sweep_negate && self.sweep_shift == 0 {
            return 0;
        }
        let delta = self.timer_period >> self.sweep_shift;
        if self.sweep_negate {
            self.timer_period
                .wrapping_sub(delta)
                .wrapping_sub(u16::from(self.channel_one))
        } else {
            self.timer_period.saturating_add(delta)
        }
    }
    fn sweep_muted(&self) -> bool {
        self.timer_period < 8 || self.target_period() > 0x07ff
    }
    fn quarter_frame(&mut self) {
        self.envelope.quarter_frame();
    }
    fn half_frame(&mut self) {
        if !self.envelope.loop_flag && self.length > 0 {
            self.length -= 1;
        }
        if self.sweep_divider == 0
            && self.sweep_enabled
            && self.sweep_shift != 0
            && !self.sweep_muted()
        {
            self.timer_period = self.target_period();
        }
        if self.sweep_divider == 0 || self.sweep_reload {
            self.sweep_divider = self.sweep_period;
            self.sweep_reload = false;
        } else {
            self.sweep_divider -= 1;
        }
    }
    fn output(&self) -> u8 {
        if !self.enabled || self.length == 0 || self.sweep_muted() {
            return 0;
        }
        if DUTY_TABLE[self.duty as usize][self.sequence as usize] == 0 {
            return 0;
        }
        self.envelope.volume()
    }
    fn active(&self) -> bool {
        self.length != 0
    }
}

#[derive(Debug, Clone, Default)]
struct Triangle {
    enabled: bool,
    control: bool,
    linear_reload: u8,
    linear_counter: u8,
    linear_reload_flag: bool,
    timer_period: u16,
    timer: u16,
    length: u8,
    sequence: u8,
}
impl Triangle {
    fn write_control(&mut self, value: u8) {
        self.control = value & 0x80 != 0;
        self.linear_reload = value & 0x7f;
    }
    fn write_timer_low(&mut self, value: u8) {
        self.timer_period = (self.timer_period & 0x0700) | value as u16;
    }
    fn write_timer_high(&mut self, value: u8) {
        self.timer_period = (self.timer_period & 0x00ff) | (((value as u16) & 7) << 8);
        if self.enabled {
            self.length = LENGTH_TABLE[(value >> 3) as usize];
        }
        self.linear_reload_flag = true;
    }
    fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
        if !enabled {
            self.length = 0;
        }
    }
    fn tick_timer(&mut self) {
        if self.timer == 0 {
            self.timer = self.timer_period;
            if self.length != 0 && self.linear_counter != 0 {
                self.sequence = (self.sequence + 1) & 31;
            }
        } else {
            self.timer -= 1;
        }
    }
    fn quarter_frame(&mut self) {
        if self.linear_reload_flag {
            self.linear_counter = self.linear_reload;
        } else if self.linear_counter > 0 {
            self.linear_counter -= 1;
        }
        if !self.control {
            self.linear_reload_flag = false;
        }
    }
    fn half_frame(&mut self) {
        if !self.control && self.length > 0 {
            self.length -= 1;
        }
    }
    fn output(&self) -> u8 {
        TRIANGLE_TABLE[self.sequence as usize]
    }
    fn active(&self) -> bool {
        self.length != 0
    }
}

#[derive(Debug, Clone)]
struct Noise {
    enabled: bool,
    mode: bool,
    period_index: u8,
    timer: u16,
    shift: u16,
    length: u8,
    envelope: Envelope,
}

impl Default for Noise {
    fn default() -> Self {
        Self {
            enabled: false,
            mode: false,
            period_index: 0,
            timer: 0,
            shift: 1,
            length: 0,
            envelope: Envelope::default(),
        }
    }
}

const NOISE_PERIOD_NTSC: [u16; 16] = [
    4, 8, 16, 32, 64, 96, 128, 160, 202, 254, 380, 508, 762, 1016, 2034, 4068,
];
const NOISE_PERIOD_PAL: [u16; 16] = [
    4, 8, 14, 30, 60, 88, 118, 148, 188, 236, 354, 472, 708, 944, 1890, 3778,
];
impl Noise {
    fn write_control(&mut self, value: u8) {
        self.envelope.write(value);
    }
    fn write_period(&mut self, value: u8) {
        self.mode = value & 0x80 != 0;
        self.period_index = value & 0x0f;
    }
    fn write_length(&mut self, value: u8) {
        if self.enabled {
            self.length = LENGTH_TABLE[(value >> 3) as usize];
        }
        self.envelope.restart();
    }
    fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
        if !enabled {
            self.length = 0;
        }
    }
    fn tick_timer(&mut self, pal: bool) {
        if self.timer == 0 {
            let periods = if pal {
                &NOISE_PERIOD_PAL
            } else {
                &NOISE_PERIOD_NTSC
            };
            self.timer = periods[self.period_index as usize].saturating_sub(1);
            let tap = if self.mode { 6 } else { 1 };
            let feedback = (self.shift & 1) ^ ((self.shift >> tap) & 1);
            self.shift = (self.shift >> 1) | (feedback << 14);
        } else {
            self.timer -= 1;
        }
    }
    fn quarter_frame(&mut self) {
        self.envelope.quarter_frame();
    }
    fn half_frame(&mut self) {
        if !self.envelope.loop_flag && self.length > 0 {
            self.length -= 1;
        }
    }
    fn output(&self) -> u8 {
        if !self.enabled || self.length == 0 || self.shift & 1 != 0 {
            0
        } else {
            self.envelope.volume()
        }
    }
    fn active(&self) -> bool {
        self.length != 0
    }
}
const DMC_RATE_NTSC: [u16; 16] = [
    428, 380, 340, 320, 286, 254, 226, 214, 190, 160, 142, 128, 106, 84, 72, 54,
];
const DMC_RATE_PAL: [u16; 16] = [
    398, 354, 316, 298, 276, 236, 210, 198, 176, 148, 132, 118, 98, 78, 66, 50,
];

#[derive(Debug, Clone)]
struct Dmc {
    enabled: bool,
    irq_enabled: bool,
    loop_flag: bool,
    rate_index: u8,
    timer: u16,
    output_level: u8,
    sample_address: u16,
    sample_length: u16,
    current_address: u16,
    bytes_remaining: u16,
    sample_buffer: Option<u8>,
    shift_register: u8,
    bits_remaining: u8,
    silence: bool,
    irq_pending: bool,
    fetch_pending: bool,
    fetch_delay: u8,
    fetch_cycles: u8,
}

impl Default for Dmc {
    fn default() -> Self {
        Self {
            enabled: false,
            irq_enabled: false,
            loop_flag: false,
            rate_index: 0,
            timer: DMC_RATE_NTSC[0],
            output_level: 0,
            sample_address: 0xc000,
            sample_length: 1,
            current_address: 0xc000,
            bytes_remaining: 0,
            sample_buffer: None,
            shift_register: 0,
            bits_remaining: 8,
            silence: true,
            irq_pending: false,
            fetch_pending: false,
            fetch_delay: 0,
            fetch_cycles: 0,
        }
    }
}

impl Dmc {
    fn write_control(&mut self, value: u8) {
        self.irq_enabled = value & 0x80 != 0;
        self.loop_flag = value & 0x40 != 0;
        self.rate_index = value & 0x0f;
        if !self.irq_enabled {
            self.irq_pending = false;
        }
    }
    fn write_direct(&mut self, value: u8) {
        self.output_level = value & 0x7f;
    }
    fn write_address(&mut self, value: u8) {
        self.sample_address = 0xc000 | ((value as u16) << 6);
    }
    fn write_length(&mut self, value: u8) {
        self.sample_length = (value as u16) * 16 + 1;
    }
    fn set_enabled(&mut self, enabled: bool, cpu_cycle: u64) {
        self.enabled = enabled;
        self.irq_pending = false;
        if !enabled {
            self.bytes_remaining = 0;
            self.fetch_pending = false;
            self.fetch_delay = 0;
            self.fetch_cycles = 0;
        } else if self.bytes_remaining == 0 {
            self.restart_sample();
            if self.sample_buffer.is_none() && !self.fetch_pending {
                self.fetch_delay = if cpu_cycle & 1 == 0 { 2 } else { 3 };
                self.fetch_cycles = 3;
            }
        }
    }
    fn restart_sample(&mut self) {
        self.current_address = self.sample_address;
        self.bytes_remaining = self.sample_length;
    }
    fn schedule_reload_fetch(&mut self) {
        if self.enabled
            && self.bytes_remaining != 0
            && self.sample_buffer.is_none()
            && !self.fetch_pending
            && self.fetch_cycles == 0
        {
            self.fetch_delay = 0;
            self.fetch_cycles = 4;
        }
    }
    fn tick(&mut self, pal: bool) {
        if self.fetch_delay > 0 {
            self.fetch_delay -= 1;
        }
        if self.timer == 0 {
            let rates = if pal { &DMC_RATE_PAL } else { &DMC_RATE_NTSC };
            self.timer = rates[self.rate_index as usize].saturating_sub(1);
            if !self.silence {
                if self.shift_register & 1 != 0 {
                    if self.output_level <= 125 {
                        self.output_level += 2;
                    }
                } else if self.output_level >= 2 {
                    self.output_level -= 2;
                }
            }
            self.shift_register >>= 1;
            if self.bits_remaining > 0 {
                self.bits_remaining -= 1;
            }
            if self.bits_remaining == 0 {
                self.bits_remaining = 8;
                if let Some(sample) = self.sample_buffer.take() {
                    self.shift_register = sample;
                    self.silence = false;
                    self.schedule_reload_fetch();
                } else {
                    self.silence = true;
                }
            }
        } else {
            self.timer -= 1;
        }
    }

    fn take_fetch_request(&mut self) -> Option<(u16, u8)> {
        if !self.enabled
            || self.fetch_pending
            || self.fetch_delay != 0
            || self.fetch_cycles == 0
            || self.sample_buffer.is_some()
            || self.bytes_remaining == 0
        {
            return None;
        }
        self.fetch_pending = true;
        let cycles = self.fetch_cycles;
        self.fetch_cycles = 0;
        Some((self.current_address, cycles))
    }

    fn supply_byte(&mut self, value: u8) {
        if !self.fetch_pending {
            return;
        }
        self.fetch_pending = false;
        self.sample_buffer = Some(value);
        self.current_address = if self.current_address == 0xffff {
            0x8000
        } else {
            self.current_address + 1
        };
        self.bytes_remaining = self.bytes_remaining.saturating_sub(1);
        if self.bytes_remaining == 0 {
            if self.loop_flag {
                self.restart_sample();
            } else if self.irq_enabled {
                self.irq_pending = true;
            }
        }
    }
    fn active(&self) -> bool {
        self.bytes_remaining != 0
    }

    fn save(&self, out: &mut StateWriter) {
        out.u8(self.enabled as u8);
        out.u8(self.irq_enabled as u8);
        out.u8(self.loop_flag as u8);
        out.u8(self.rate_index);
        out.u16(self.timer);
        out.u8(self.output_level);
        out.u16(self.sample_address);
        out.u16(self.sample_length);
        out.u16(self.current_address);
        out.u16(self.bytes_remaining);
        out.u8(self.sample_buffer.unwrap_or(0));
        out.u8(self.sample_buffer.is_some() as u8);
        out.u8(self.shift_register);
        out.u8(self.bits_remaining);
        out.u8(self.silence as u8);
        out.u8(self.irq_pending as u8);
        out.u8(self.fetch_pending as u8);
        out.u8(self.fetch_delay);
        out.u8(self.fetch_cycles);
    }
    fn load(input: &mut StateReader<'_>) -> Result<Self, String> {
        let enabled = input.u8()? != 0;
        let irq_enabled = input.u8()? != 0;
        let loop_flag = input.u8()? != 0;
        let rate_index = input.u8()?;
        let timer = input.u16()?;
        let output_level = input.u8()?;
        let sample_address = input.u16()?;
        let sample_length = input.u16()?;
        let current_address = input.u16()?;
        let bytes_remaining = input.u16()?;
        let sample = input.u8()?;
        let has_sample = input.u8()? != 0;
        let shift_register = input.u8()?;
        let bits_remaining = input.u8()?;
        let silence = input.u8()? != 0;
        let irq_pending = input.u8()? != 0;
        let fetch_pending = input.u8()? != 0;
        let fetch_delay = input.u8()?.min(3);
        let fetch_cycles = input.u8()?.min(4);
        Ok(Self {
            enabled,
            irq_enabled,
            loop_flag,
            rate_index,
            timer,
            output_level,
            sample_address,
            sample_length,
            current_address,
            bytes_remaining,
            sample_buffer: if has_sample { Some(sample) } else { None },
            shift_register,
            bits_remaining,
            silence,
            irq_pending,
            fetch_pending,
            fetch_delay,
            fetch_cycles,
        })
    }
}

#[derive(Debug, Clone, Default)]
struct HighPassFilter {
    previous_input: f32,
    previous_output: f32,
}

impl HighPassFilter {
    fn process(&mut self, input: f32, alpha: f32) -> f32 {
        let output = alpha * (self.previous_output + input - self.previous_input);
        self.previous_input = input;
        self.previous_output = output;
        output
    }
}

#[derive(Debug, Clone, Default)]
struct LowPassFilter {
    output: f32,
}

impl LowPassFilter {
    fn process(&mut self, input: f32, alpha: f32) -> f32 {
        self.output += alpha * (input - self.output);
        self.output
    }
}

#[derive(Debug, Clone)]
pub struct Apu {
    pulse1: Pulse,
    pulse2: Pulse,
    triangle: Triangle,
    noise: Noise,
    dmc: Dmc,
    frame_cycle: u32,
    five_step: bool,
    irq_inhibit: bool,
    frame_irq: bool,
    frame_write_value: Option<u8>,
    frame_write_delay: u8,
    block_frame_counter_tick: u8,
    cpu_cycle: u64,
    sample_phase: u64,
    cpu_hz: u64,
    pal: bool,
    high_pass_90_alpha: f32,
    high_pass_440_alpha: f32,
    low_pass_14k_alpha: f32,
    high_pass_90: HighPassFilter,
    high_pass_440: HighPassFilter,
    low_pass_14k: LowPassFilter,
    samples: Vec<f32>,
}

impl Default for Apu {
    fn default() -> Self {
        Self::new(NTSC_CPU_HZ, false)
    }
}

impl Apu {
    pub fn new(cpu_hz: u64, pal: bool) -> Self {
        let cpu_hz_f32 = cpu_hz as f32;
        let high_pass_90_alpha = cpu_hz_f32 / (cpu_hz_f32 + core::f32::consts::TAU * 90.0);
        let high_pass_440_alpha = cpu_hz_f32 / (cpu_hz_f32 + core::f32::consts::TAU * 440.0);
        let low_pass_14k_alpha =
            (core::f32::consts::TAU * 14_000.0) / (cpu_hz_f32 + core::f32::consts::TAU * 14_000.0);
        let mut dmc = Dmc::default();
        if pal {
            dmc.timer = DMC_RATE_PAL[0];
        }
        Self {
            pulse1: Pulse::new(true),
            pulse2: Pulse::new(false),
            triangle: Triangle::default(),
            noise: Noise::default(),
            dmc,
            frame_cycle: 0,
            five_step: false,
            irq_inhibit: false,
            frame_irq: false,
            frame_write_value: None,
            frame_write_delay: 0,
            block_frame_counter_tick: 0,
            cpu_cycle: 0,
            sample_phase: 0,
            cpu_hz,
            pal,
            high_pass_90_alpha,
            high_pass_440_alpha,
            low_pass_14k_alpha,
            high_pass_90: HighPassFilter::default(),
            high_pass_440: HighPassFilter::default(),
            low_pass_14k: LowPassFilter::default(),
            samples: Vec::with_capacity(1024),
        }
    }

    pub fn reset(&mut self) {
        *self = Self::new(self.cpu_hz, self.pal);
    }
    pub fn irq_pending(&self) -> bool {
        self.frame_irq || self.dmc.irq_pending
    }
    pub fn take_dmc_fetch_request(&mut self) -> Option<(u16, u8)> {
        self.dmc.take_fetch_request()
    }
    pub fn supply_dmc_byte(&mut self, value: u8) {
        self.dmc.supply_byte(value);
    }
    pub fn write_register(&mut self, address: u16, value: u8) {
        match address {
            0x4000 => self.pulse1.write_control(value),
            0x4001 => self.pulse1.write_sweep(value),
            0x4002 => self.pulse1.write_timer_low(value),
            0x4003 => self.pulse1.write_timer_high(value),
            0x4004 => self.pulse2.write_control(value),
            0x4005 => self.pulse2.write_sweep(value),
            0x4006 => self.pulse2.write_timer_low(value),
            0x4007 => self.pulse2.write_timer_high(value),
            0x4008 => self.triangle.write_control(value),
            0x400a => self.triangle.write_timer_low(value),
            0x400b => self.triangle.write_timer_high(value),
            0x400c => self.noise.write_control(value),
            0x400e => self.noise.write_period(value),
            0x400f => self.noise.write_length(value),
            0x4010 => self.dmc.write_control(value),
            0x4011 => self.dmc.write_direct(value),
            0x4012 => self.dmc.write_address(value),
            0x4013 => self.dmc.write_length(value),
            0x4015 => self.write_status(value),
            0x4017 => self.write_frame_counter(value),
            _ => {}
        }
    }

    fn write_status(&mut self, value: u8) {
        self.pulse1.set_enabled(value & 0x01 != 0);
        self.pulse2.set_enabled(value & 0x02 != 0);
        self.triangle.set_enabled(value & 0x04 != 0);
        self.noise.set_enabled(value & 0x08 != 0);
        self.dmc.set_enabled(value & 0x10 != 0, self.cpu_cycle);
    }
    fn write_frame_counter(&mut self, value: u8) {
        self.frame_write_value = Some(value);
        self.frame_write_delay = if self.cpu_cycle & 1 == 0 { 3 } else { 4 };
        self.irq_inhibit = value & 0x40 != 0;
        if self.irq_inhibit {
            self.frame_irq = false;
        }
    }

    pub fn read_status(&mut self) -> u8 {
        let value = u8::from(self.pulse1.active())
            | (u8::from(self.pulse2.active()) << 1)
            | (u8::from(self.triangle.active()) << 2)
            | (u8::from(self.noise.active()) << 3)
            | (u8::from(self.dmc.active()) << 4)
            | (u8::from(self.frame_irq) << 6)
            | (u8::from(self.dmc.irq_pending) << 7);
        self.frame_irq = false;
        value
    }

    fn quarter_frame(&mut self) {
        self.pulse1.quarter_frame();
        self.pulse2.quarter_frame();
        self.triangle.quarter_frame();
        self.noise.quarter_frame();
    }
    fn half_frame(&mut self) {
        self.pulse1.half_frame();
        self.pulse2.half_frame();
        self.triangle.half_frame();
        self.noise.half_frame();
    }
    fn clock_frame_units(&mut self, half_frame: bool) {
        if self.block_frame_counter_tick != 0 {
            return;
        }
        self.quarter_frame();
        if half_frame {
            self.half_frame();
        }
        self.block_frame_counter_tick = 2;
    }

    fn apply_pending_frame_counter_write(&mut self) {
        let Some(value) = self.frame_write_value else {
            return;
        };
        if self.frame_write_delay > 0 {
            self.frame_write_delay -= 1;
        }
        if self.frame_write_delay != 0 {
            return;
        }

        self.frame_write_value = None;
        self.five_step = value & 0x80 != 0;
        self.frame_cycle = 0;
        if self.five_step {
            self.clock_frame_units(true);
        }
    }

    fn clock_frame_counter(&mut self) {
        self.frame_cycle += 1;
        if self.five_step {
            if self.pal {
                match self.frame_cycle {
                    8313 | 24939 => self.clock_frame_units(false),
                    16627 | 41565 => self.clock_frame_units(true),
                    _ => {}
                }
                if self.frame_cycle >= 41566 {
                    self.frame_cycle = 0;
                }
            } else {
                match self.frame_cycle {
                    7457 | 22371 => self.clock_frame_units(false),
                    14913 | 37281 => self.clock_frame_units(true),
                    _ => {}
                }
                if self.frame_cycle >= 37282 {
                    self.frame_cycle = 0;
                }
            }
        } else if self.pal {
            if matches!(self.frame_cycle, 33252..=33254) && !self.irq_inhibit {
                self.frame_irq = true;
            }
            match self.frame_cycle {
                8313 | 24939 => self.clock_frame_units(false),
                16627 | 33253 => self.clock_frame_units(true),
                _ => {}
            }
            if self.frame_cycle >= 33254 {
                self.frame_cycle = 0;
            }
        } else {
            if matches!(self.frame_cycle, 29828..=29830) && !self.irq_inhibit {
                self.frame_irq = true;
            }
            match self.frame_cycle {
                7457 | 22371 => self.clock_frame_units(false),
                14913 | 29829 => self.clock_frame_units(true),
                _ => {}
            }
            if self.frame_cycle >= 29830 {
                self.frame_cycle = 0;
            }
        }

        self.apply_pending_frame_counter_write();
        if self.block_frame_counter_tick > 0 {
            self.block_frame_counter_tick -= 1;
        }
    }

    #[cfg(test)]
    pub fn tick_cpu_cycles(&mut self, cycles: u32) {
        for _ in 0..cycles {
            self.tick_cpu_cycle_with_expansion(0.0);
        }
    }

    pub fn tick_cpu_cycle_with_expansion(&mut self, expansion: f32) {
        self.cpu_cycle = self.cpu_cycle.wrapping_add(1);
        self.triangle.tick_timer();
        self.noise.tick_timer(self.pal);
        self.dmc.tick(self.pal);
        if self.cpu_cycle & 1 == 0 {
            self.pulse1.tick_timer();
            self.pulse2.tick_timer();
        }
        self.clock_frame_counter();

        let mixed = (self.mix() + expansion).clamp(-1.0, 1.0);
        let filtered = self.filter_output(mixed).clamp(-1.0, 1.0);
        self.sample_phase += SAMPLE_RATE as u64;
        if self.sample_phase >= self.cpu_hz {
            self.sample_phase -= self.cpu_hz;
            self.samples.push(filtered);
        }
    }
    fn filter_output(&mut self, input: f32) -> f32 {
        let high_pass_90 = self.high_pass_90.process(input, self.high_pass_90_alpha);
        let high_pass_440 = self
            .high_pass_440
            .process(high_pass_90, self.high_pass_440_alpha);
        self.low_pass_14k
            .process(high_pass_440, self.low_pass_14k_alpha)
    }
    fn mix(&self) -> f32 {
        let pulse_sum = (self.pulse1.output() + self.pulse2.output()) as f32;
        let pulse = if pulse_sum == 0.0 {
            0.0
        } else {
            95.88 / (8128.0 / pulse_sum + 100.0)
        };
        let triangle = self.triangle.output() as f32;
        let noise = self.noise.output() as f32;
        let dmc = self.dmc.output_level as f32;
        let tnd_input = triangle / 8227.0 + noise / 12241.0 + dmc / 22638.0;
        let tnd = if tnd_input == 0.0 {
            0.0
        } else {
            159.79 / (1.0 / tnd_input + 100.0)
        };
        (pulse + tnd).clamp(0.0, 1.0)
    }

    pub fn drain_audio(&mut self, output: &mut AudioBuffer) {
        output.begin_frame();
        if self.samples.is_empty() {
            output.push_stereo(0.0, 0.0);
            return;
        }
        for sample in self.samples.drain(..) {
            output.push_stereo(sample, sample);
        }
    }

    pub fn save(&self, out: &mut StateWriter) {
        save_pulse(out, &self.pulse1);
        save_pulse(out, &self.pulse2);
        save_triangle(out, &self.triangle);
        save_noise(out, &self.noise);
        self.dmc.save(out);
        out.u32(self.frame_cycle);
        out.u8(self.five_step as u8);
        out.u8(self.irq_inhibit as u8);
        out.u8(self.frame_irq as u8);
        out.u8(self.frame_write_value.unwrap_or(0));
        out.u8(self.frame_write_value.is_some() as u8);
        out.u8(self.frame_write_delay);
        out.u8(self.block_frame_counter_tick);
        out.u64(self.cpu_cycle);
        out.u64(self.sample_phase);
        out.f32(self.high_pass_90.previous_input);
        out.f32(self.high_pass_90.previous_output);
        out.f32(self.high_pass_440.previous_input);
        out.f32(self.high_pass_440.previous_output);
        out.f32(self.low_pass_14k.output);
    }
    pub fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.pulse1 = load_pulse(input)?;
        self.pulse1.channel_one = true;
        self.pulse2 = load_pulse(input)?;
        self.pulse2.channel_one = false;
        self.triangle = load_triangle(input)?;
        self.noise = load_noise(input)?;
        self.dmc = Dmc::load(input)?;
        self.frame_cycle = input.u32()?;
        self.five_step = input.u8()? != 0;
        self.irq_inhibit = input.u8()? != 0;
        self.frame_irq = input.u8()? != 0;
        let frame_write_value = input.u8()?;
        let has_frame_write = input.u8()? != 0;
        self.frame_write_delay = input.u8()?.min(4);
        self.block_frame_counter_tick = input.u8()?.min(2);
        self.frame_write_value = has_frame_write.then_some(frame_write_value);
        if !has_frame_write {
            self.frame_write_delay = 0;
        }
        self.cpu_cycle = input.u64()?;
        self.sample_phase = input.u64()?;
        self.high_pass_90.previous_input = input.f32()?;
        self.high_pass_90.previous_output = input.f32()?;
        self.high_pass_440.previous_input = input.f32()?;
        self.high_pass_440.previous_output = input.f32()?;
        self.low_pass_14k.output = input.f32()?;
        self.samples.clear();
        Ok(())
    }
}

fn save_envelope(out: &mut StateWriter, env: &Envelope) {
    out.u8(env.loop_flag as u8);
    out.u8(env.constant as u8);
    out.u8(env.period);
    out.u8(env.divider);
    out.u8(env.decay);
    out.u8(env.start as u8);
}
fn load_envelope(input: &mut StateReader<'_>) -> Result<Envelope, String> {
    Ok(Envelope {
        loop_flag: input.u8()? != 0,
        constant: input.u8()? != 0,
        period: input.u8()?,
        divider: input.u8()?,
        decay: input.u8()?,
        start: input.u8()? != 0,
    })
}

fn save_pulse(out: &mut StateWriter, pulse: &Pulse) {
    out.u8(pulse.enabled as u8);
    out.u8(pulse.duty);
    out.u8(pulse.sequence);
    out.u16(pulse.timer_period);
    out.u16(pulse.timer);
    out.u8(pulse.length);
    save_envelope(out, &pulse.envelope);
    out.u8(pulse.sweep_enabled as u8);
    out.u8(pulse.sweep_period);
    out.u8(pulse.sweep_negate as u8);
    out.u8(pulse.sweep_shift);
    out.u8(pulse.sweep_divider);
    out.u8(pulse.sweep_reload as u8);
}
fn load_pulse(input: &mut StateReader<'_>) -> Result<Pulse, String> {
    Ok(Pulse {
        enabled: input.u8()? != 0,
        channel_one: false,
        duty: input.u8()?,
        sequence: input.u8()?,
        timer_period: input.u16()?,
        timer: input.u16()?,
        length: input.u8()?,
        envelope: load_envelope(input)?,
        sweep_enabled: input.u8()? != 0,
        sweep_period: input.u8()?,
        sweep_negate: input.u8()? != 0,
        sweep_shift: input.u8()?,
        sweep_divider: input.u8()?,
        sweep_reload: input.u8()? != 0,
    })
}

fn save_triangle(out: &mut StateWriter, tri: &Triangle) {
    out.u8(tri.enabled as u8);
    out.u8(tri.control as u8);
    out.u8(tri.linear_reload);
    out.u8(tri.linear_counter);
    out.u8(tri.linear_reload_flag as u8);
    out.u16(tri.timer_period);
    out.u16(tri.timer);
    out.u8(tri.length);
    out.u8(tri.sequence);
}
fn load_triangle(input: &mut StateReader<'_>) -> Result<Triangle, String> {
    Ok(Triangle {
        enabled: input.u8()? != 0,
        control: input.u8()? != 0,
        linear_reload: input.u8()?,
        linear_counter: input.u8()?,
        linear_reload_flag: input.u8()? != 0,
        timer_period: input.u16()?,
        timer: input.u16()?,
        length: input.u8()?,
        sequence: input.u8()?,
    })
}
fn save_noise(out: &mut StateWriter, noise: &Noise) {
    out.u8(noise.enabled as u8);
    out.u8(noise.mode as u8);
    out.u8(noise.period_index);
    out.u16(noise.timer);
    out.u16(noise.shift);
    out.u8(noise.length);
    save_envelope(out, &noise.envelope);
}
fn load_noise(input: &mut StateReader<'_>) -> Result<Noise, String> {
    Ok(Noise {
        enabled: input.u8()? != 0,
        mode: input.u8()? != 0,
        period_index: input.u8()?,
        timer: input.u16()?,
        shift: input.u16()?,
        length: input.u8()?,
        envelope: load_envelope(input)?,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::platform::PlatformId;

    #[test]
    fn enabled_pulse_generates_nonzero_audio() {
        let mut apu = Apu::default();
        apu.write_register(0x4015, 0x01);
        apu.write_register(0x4000, 0b1011_1111);
        apu.write_register(0x4002, 0x80);
        apu.write_register(0x4003, 0x08);
        apu.tick_cpu_cycles(30_000);
        assert!(apu.samples.iter().any(|sample| *sample > 0.0));
    }
    #[test]
    fn expansion_audio_changes_the_filtered_apu_waveform() {
        let mut baseline = Apu::default();
        let mut expanded = Apu::default();
        for _ in 0..4_000 {
            baseline.tick_cpu_cycle_with_expansion(0.0);
            expanded.tick_cpu_cycle_with_expansion(-0.2);
        }
        assert_eq!(baseline.samples.len(), expanded.samples.len());
        assert!(baseline
            .samples
            .iter()
            .zip(&expanded.samples)
            .any(|(left, right)| (left - right).abs() > 0.01));
    }

    #[test]
    fn analog_output_filters_reject_dc_bias() {
        let mut apu = Apu::default();
        let mut output = 0.0;
        for _ in 0..100_000 {
            output = apu.filter_output(0.5);
        }
        assert!(output.abs() < 0.001);
        assert!(apu.low_pass_14k.output.abs() < 0.001);
    }

    #[test]
    fn nonlinear_mixer_matches_nes_dac_equations_without_extra_gain() {
        let mut apu = Apu::default();
        apu.triangle.sequence = 15;
        apu.dmc.output_level = 127;
        let tnd_input = 127.0 / 22638.0;
        let expected_tnd = 159.79 / (1.0 / tnd_input + 100.0);
        assert!((apu.mix() - expected_tnd).abs() < 1.0e-6);

        apu.dmc.output_level = 0;
        apu.pulse1.enabled = true;
        apu.pulse1.length = 1;
        apu.pulse1.timer_period = 8;
        apu.pulse1.duty = 0;
        apu.pulse1.sequence = 1;
        apu.pulse1.envelope.constant = true;
        apu.pulse1.envelope.period = 15;
        let expected_pulse = 95.88 / (8128.0 / 15.0 + 100.0);
        assert!((apu.mix() - expected_pulse).abs() < 1.0e-6);
    }

    #[test]
    fn noise_period_table_counts_cpu_cycles() {
        let mut apu = Apu::default();
        apu.noise.period_index = 0;
        apu.noise.timer = 0;
        apu.noise.shift = 1;

        apu.tick_cpu_cycle_with_expansion(0.0);
        let first_shift = apu.noise.shift;
        assert_eq!(apu.noise.timer, 3);
        for _ in 0..3 {
            apu.tick_cpu_cycle_with_expansion(0.0);
            assert_eq!(apu.noise.shift, first_shift);
        }
        apu.tick_cpu_cycle_with_expansion(0.0);
        assert_ne!(apu.noise.shift, first_shift);
    }

    #[test]
    fn pulse_sweep_zero_shift_still_controls_target_overflow_muting() {
        let mut pulse = Pulse::new(true);
        pulse.timer_period = 0x03ff;
        pulse.sweep_shift = 0;
        assert_eq!(pulse.target_period(), 0x07fe);
        assert!(!pulse.sweep_muted());

        pulse.timer_period = 0x0400;
        assert_eq!(pulse.target_period(), 0x0800);
        assert!(pulse.sweep_muted());

        for channel_one in [false, true] {
            let mut negated = Pulse::new(channel_one);
            negated.timer_period = 0x0600;
            negated.sweep_negate = true;
            negated.sweep_shift = 0;
            assert_eq!(negated.target_period(), 0);
            assert!(!negated.sweep_muted());
        }
    }

    #[test]
    fn pal_uses_regional_noise_dmc_and_frame_counter_periods() {
        let mut apu = Apu::new(1_662_607, true);
        apu.noise.period_index = 2;
        apu.noise.timer = 0;
        apu.dmc.rate_index = 0;
        apu.dmc.timer = 0;
        apu.tick_cpu_cycle_with_expansion(0.0);
        assert_eq!(apu.noise.timer, 13);
        assert_eq!(apu.dmc.timer, 397);

        let mut frame = Apu::new(1_662_607, true);
        frame.tick_cpu_cycles(33_252);
        assert_eq!(frame.frame_cycle, 33_252);
        assert!(frame.irq_pending());
        assert_ne!(frame.read_status() & 0x40, 0);
        assert!(!frame.irq_pending());

        let mut five_step = Apu::new(1_662_607, true);
        five_step.five_step = true;
        five_step.frame_cycle = 41_564;
        five_step.tick_cpu_cycles(1);
        assert_eq!(five_step.frame_cycle, 41_565);
        five_step.tick_cpu_cycles(1);
        assert_eq!(five_step.frame_cycle, 0);
    }

    #[test]
    fn triangle_period_zero_and_one_continue_ultrasonic_sequence() {
        let mut period_zero = Triangle {
            enabled: true,
            length: 1,
            linear_counter: 1,
            ..Default::default()
        };
        period_zero.tick_timer();
        assert_eq!(period_zero.sequence, 1);
        period_zero.tick_timer();
        assert_eq!(period_zero.sequence, 2);

        let mut period_one = Triangle {
            enabled: true,
            length: 1,
            linear_counter: 1,
            timer_period: 1,
            ..Default::default()
        };
        period_one.tick_timer();
        assert_eq!(period_one.sequence, 1);
        period_one.tick_timer();
        assert_eq!(period_one.sequence, 1);
        period_one.tick_timer();
        assert_eq!(period_one.sequence, 2);
    }

    #[test]
    fn triangle_holds_dac_level_when_counters_halt() {
        let mut triangle = Triangle {
            enabled: true,
            length: 1,
            linear_counter: 1,
            timer_period: 2,
            sequence: 6,
            ..Default::default()
        };
        triangle.tick_timer();
        let held_sequence = triangle.sequence;
        let held_output = triangle.output();
        triangle.set_enabled(false);
        triangle.linear_counter = 0;
        for _ in 0..8 {
            triangle.tick_timer();
        }
        assert_eq!(triangle.sequence, held_sequence);
        assert_eq!(triangle.output(), held_output);
    }

    #[test]
    fn frame_irq_is_reported_and_cleared_by_status_read() {
        let mut apu = Apu::default();
        apu.tick_cpu_cycles(29_828);
        for expected_cycle in [29_828, 29_829, 0] {
            assert_eq!(apu.frame_cycle, expected_cycle);
            assert!(apu.irq_pending());
            assert_ne!(apu.read_status() & 0x40, 0);
            assert!(!apu.irq_pending());
            if expected_cycle != 0 {
                apu.tick_cpu_cycles(1);
            }
        }
    }

    #[test]
    fn frame_counter_write_uses_three_or_four_cycle_delay() {
        let mut even = Apu {
            frame_cycle: 100,
            ..Default::default()
        };
        even.pulse1.enabled = true;
        even.pulse1.length = 2;
        even.frame_irq = true;
        even.write_register(0x4017, 0xc0);
        assert!(!even.frame_irq);
        assert!(!even.five_step);
        assert_eq!(even.frame_write_delay, 3);
        even.tick_cpu_cycles(2);
        assert!(!even.five_step);
        assert_eq!(even.pulse1.length, 2);
        even.tick_cpu_cycles(1);
        assert!(even.five_step);
        assert_eq!(even.frame_cycle, 0);
        assert_eq!(even.pulse1.length, 1);

        let mut odd = Apu {
            cpu_cycle: 1,
            frame_cycle: 200,
            ..Default::default()
        };
        odd.write_register(0x4017, 0x00);
        assert_eq!(odd.frame_write_delay, 4);
        odd.tick_cpu_cycles(3);
        assert_eq!(odd.frame_cycle, 203);
        odd.tick_cpu_cycles(1);
        assert_eq!(odd.frame_cycle, 0);
        assert!(!odd.five_step);
    }

    #[test]
    fn state_round_trip_preserves_channel_sequencers() {
        let mut apu = Apu::default();
        apu.write_register(0x4015, 0x0f);
        apu.write_register(0x4000, 0x9f);
        apu.write_register(0x4002, 0x41);
        apu.write_register(0x4003, 0x18);
        apu.tick_cpu_cycles(12_345);
        apu.write_register(0x4017, 0x80);
        apu.tick_cpu_cycles(1);
        let mut writer = StateWriter::new(PlatformId::Nes, 99);
        apu.save(&mut writer);
        let bytes = writer.finish();
        let mut restored = Apu::default();
        let mut reader = StateReader::new(&bytes, PlatformId::Nes, 99).unwrap();
        restored.load(&mut reader).unwrap();
        reader.finish().unwrap();
        assert_eq!(restored.cpu_cycle, apu.cpu_cycle);
        assert_eq!(restored.pulse1.sequence, apu.pulse1.sequence);
        assert_eq!(restored.pulse1.length, apu.pulse1.length);
        assert_eq!(restored.frame_write_value, apu.frame_write_value);
        assert_eq!(restored.frame_write_delay, apu.frame_write_delay);
        assert_eq!(
            restored.block_frame_counter_tick,
            apu.block_frame_counter_tick
        );
        assert_eq!(
            restored.high_pass_90.previous_output.to_bits(),
            apu.high_pass_90.previous_output.to_bits()
        );
        assert_eq!(
            restored.high_pass_440.previous_output.to_bits(),
            apu.high_pass_440.previous_output.to_bits()
        );
        assert_eq!(
            restored.low_pass_14k.output.to_bits(),
            apu.low_pass_14k.output.to_bits()
        );

        apu.samples.clear();
        restored.samples.clear();
        for _ in 0..4_096 {
            apu.tick_cpu_cycle_with_expansion(0.05);
            restored.tick_cpu_cycle_with_expansion(0.05);
        }
        assert_eq!(restored.samples, apu.samples);
    }
    #[test]
    fn dmc_requests_sample_memory_and_raises_terminal_irq() {
        let mut apu = Apu::default();
        apu.write_register(0x4010, 0x80);
        apu.write_register(0x4012, 0xff);
        apu.write_register(0x4013, 0x00);
        apu.write_register(0x4015, 0x10);
        assert_eq!(apu.take_dmc_fetch_request(), None);
        apu.tick_cpu_cycles(1);
        assert_eq!(apu.take_dmc_fetch_request(), None);
        apu.tick_cpu_cycles(1);
        assert_eq!(apu.take_dmc_fetch_request(), Some((0xffc0, 3)));
        apu.supply_dmc_byte(0xa5);
        assert!(apu.irq_pending());
        let status = apu.read_status();
        assert_eq!(status & 0x10, 0);
        assert_ne!(status & 0x80, 0);
        assert!(apu.irq_pending());
        apu.write_register(0x4015, 0x00);
        assert!(!apu.irq_pending());
    }

    #[test]
    fn dmc_reload_uses_four_cycle_dma_after_sample_buffer_empties() {
        let mut apu = Apu::default();
        apu.write_register(0x4012, 0x00);
        apu.write_register(0x4013, 0x01);
        apu.write_register(0x4015, 0x10);
        apu.tick_cpu_cycles(2);
        assert_eq!(apu.take_dmc_fetch_request(), Some((0xc000, 3)));
        apu.supply_dmc_byte(0x5a);

        apu.dmc.timer = 0;
        apu.dmc.bits_remaining = 1;
        apu.tick_cpu_cycles(1);
        assert_eq!(apu.take_dmc_fetch_request(), Some((0xc001, 4)));
    }
}
