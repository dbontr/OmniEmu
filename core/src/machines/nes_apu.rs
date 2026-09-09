use crate::kernel::AudioBuffer;
use crate::state::{StateReader, StateWriter};

const CPU_HZ: u64 = 1_789_773;
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
        self.timer_period < 8 || (self.sweep_shift != 0 && self.target_period() > 0x07ff)
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
            if self.length != 0 && self.linear_counter != 0 && self.timer_period > 1 {
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
        if !self.enabled || self.length == 0 || self.linear_counter == 0 {
            0
        } else {
            TRIANGLE_TABLE[self.sequence as usize]
        }
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

const NOISE_PERIOD: [u16; 16] = [
    4, 8, 16, 32, 64, 96, 128, 160, 202, 254, 380, 508, 762, 1016, 2034, 4068,
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
    fn tick_timer(&mut self) {
        if self.timer == 0 {
            self.timer = NOISE_PERIOD[self.period_index as usize];
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
const DMC_RATE: [u16; 16] = [
    428, 380, 340, 320, 286, 254, 226, 214, 190, 160, 142, 128, 106, 84, 72, 54,
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
}

impl Default for Dmc {
    fn default() -> Self {
        Self {
            enabled: false,
            irq_enabled: false,
            loop_flag: false,
            rate_index: 0,
            timer: DMC_RATE[0],
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
    fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
        self.irq_pending = false;
        if !enabled {
            self.bytes_remaining = 0;
            self.fetch_pending = false;
        } else if self.bytes_remaining == 0 {
            self.restart_sample();
        }
    }
    fn restart_sample(&mut self) {
        self.current_address = self.sample_address;
        self.bytes_remaining = self.sample_length;
    }
    fn tick(&mut self) {
        if self.timer == 0 {
            self.timer = DMC_RATE[self.rate_index as usize].saturating_sub(1);
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
                } else {
                    self.silence = true;
                }
            }
        } else {
            self.timer -= 1;
        }
    }

    fn take_fetch_request(&mut self) -> Option<u16> {
        if !self.enabled
            || self.fetch_pending
            || self.sample_buffer.is_some()
            || self.bytes_remaining == 0
        {
            return None;
        }
        self.fetch_pending = true;
        Some(self.current_address)
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
        })
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
    cpu_cycle: u64,
    sample_phase: u64,
    samples: Vec<f32>,
}

impl Default for Apu {
    fn default() -> Self {
        Self {
            pulse1: Pulse::new(true),
            pulse2: Pulse::new(false),
            triangle: Triangle::default(),
            noise: Noise::default(),
            dmc: Dmc::default(),
            frame_cycle: 0,
            five_step: false,
            irq_inhibit: false,
            frame_irq: false,
            cpu_cycle: 0,
            sample_phase: 0,
            samples: Vec::with_capacity(1024),
        }
    }
}

impl Apu {
    pub fn reset(&mut self) {
        *self = Self::default();
    }
    pub fn irq_pending(&self) -> bool {
        self.frame_irq || self.dmc.irq_pending
    }
    pub fn take_dmc_fetch_request(&mut self) -> Option<u16> {
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
        self.dmc.set_enabled(value & 0x10 != 0);
    }
    fn write_frame_counter(&mut self, value: u8) {
        self.five_step = value & 0x80 != 0;
        self.irq_inhibit = value & 0x40 != 0;
        if self.irq_inhibit {
            self.frame_irq = false;
        }
        self.frame_cycle = 0;
        if self.five_step {
            self.quarter_frame();
            self.half_frame();
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
    fn clock_frame_counter(&mut self) {
        self.frame_cycle += 1;
        if self.five_step {
            match self.frame_cycle {
                7457 | 22371 => self.quarter_frame(),
                14913 | 37281 => {
                    self.quarter_frame();
                    self.half_frame();
                }
                _ => {}
            }
            if self.frame_cycle >= 37282 {
                self.frame_cycle = 0;
            }
        } else {
            match self.frame_cycle {
                7457 | 22371 => self.quarter_frame(),
                14913 => {
                    self.quarter_frame();
                    self.half_frame();
                }
                29829 => {
                    self.quarter_frame();
                    self.half_frame();
                    if !self.irq_inhibit {
                        self.frame_irq = true;
                    }
                }
                _ => {}
            }
            if self.frame_cycle >= 29830 {
                self.frame_cycle = 0;
            }
        }
    }

    pub fn tick_cpu_cycles(&mut self, cycles: u32) {
        for _ in 0..cycles {
            self.cpu_cycle = self.cpu_cycle.wrapping_add(1);
            self.triangle.tick_timer();
            self.dmc.tick();
            if self.cpu_cycle & 1 == 0 {
                self.pulse1.tick_timer();
                self.pulse2.tick_timer();
                self.noise.tick_timer();
            }
            self.clock_frame_counter();
            self.sample_phase += SAMPLE_RATE as u64;
            if self.sample_phase >= CPU_HZ {
                self.sample_phase -= CPU_HZ;
                self.samples.push(self.mix());
            }
        }
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
        ((pulse + tnd) * 1.35).clamp(0.0, 1.0)
    }

    pub fn drain_audio(&mut self, output: &mut AudioBuffer) {
        output.begin_frame();
        if self.samples.is_empty() {
            output.push_stereo(0.0, 0.0);
            return;
        }
        for sample in self.samples.drain(..) {
            let centered = (sample - 0.25).clamp(-1.0, 1.0);
            output.push_stereo(centered, centered);
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
        out.u64(self.cpu_cycle);
        out.u64(self.sample_phase);
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
        self.cpu_cycle = input.u64()?;
        self.sample_phase = input.u64()?;
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
    fn frame_irq_is_reported_and_cleared_by_status_read() {
        let mut apu = Apu::default();
        apu.tick_cpu_cycles(29_829);
        assert!(apu.irq_pending());
        assert_ne!(apu.read_status() & 0x40, 0);
        assert!(!apu.irq_pending());
    }

    #[test]
    fn state_round_trip_preserves_channel_sequencers() {
        let mut apu = Apu::default();
        apu.write_register(0x4015, 0x0f);
        apu.write_register(0x4000, 0x9f);
        apu.write_register(0x4002, 0x41);
        apu.write_register(0x4003, 0x18);
        apu.tick_cpu_cycles(12_345);
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
    }
    #[test]
    fn dmc_requests_sample_memory_and_raises_terminal_irq() {
        let mut apu = Apu::default();
        apu.write_register(0x4010, 0x80);
        apu.write_register(0x4012, 0xff);
        apu.write_register(0x4013, 0x00);
        apu.write_register(0x4015, 0x10);
        assert_eq!(apu.take_dmc_fetch_request(), Some(0xffc0));
        apu.supply_dmc_byte(0xa5);
        assert!(apu.irq_pending());
        let status = apu.read_status();
        assert_eq!(status & 0x10, 0);
        assert_ne!(status & 0x80, 0);
        assert!(apu.irq_pending());
        apu.write_register(0x4015, 0x00);
        assert!(!apu.irq_pending());
    }
}
