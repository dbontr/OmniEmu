use crate::state::{StateReader, StateWriter};

const SAMPLE_RATE: u64 = 48_000;
const CHANNELS: usize = 6;
const OPERATORS: usize = 4;

#[derive(Debug, Clone, Copy)]
struct Operator {
    detune: u8,
    multiple: u8,
    total_level: u8,
    rate_scale: u8,
    attack: u8,
    decay: u8,
    sustain_rate: u8,
    sustain_level: u8,
    release: u8,
    amplitude_modulation: bool,
    ssg_eg: u8,
    phase: f64,
    envelope: f32,
    envelope_stage: u8,
    key_on: bool,
}

impl Default for Operator {
    fn default() -> Self {
        Self {
            detune: 0,
            multiple: 1,
            total_level: 0x7f,
            rate_scale: 0,
            attack: 0,
            decay: 0,
            sustain_rate: 0,
            sustain_level: 0,
            release: 0,
            amplitude_modulation: false,
            ssg_eg: 0,
            phase: 0.0,
            envelope: 0.0,
            envelope_stage: 3,
            key_on: false,
        }
    }
}
#[derive(Debug, Clone, Copy)]
struct Channel {
    fnum: u16,
    block: u8,
    algorithm: u8,
    feedback: u8,
    pan_left: bool,
    pan_right: bool,
    amplitude_sensitivity: u8,
    phase_sensitivity: u8,
    operators: [Operator; OPERATORS],
}

impl Default for Channel {
    fn default() -> Self {
        Self {
            fnum: 0,
            block: 0,
            algorithm: 0,
            feedback: 0,
            pan_left: true,
            pan_right: true,
            amplitude_sensitivity: 0,
            phase_sensitivity: 0,
            operators: [Operator::default(); OPERATORS],
        }
    }
}

#[derive(Debug, Clone)]
pub struct Ym2612 {
    clock_hz: u64,
    address: [u8; 2],
    channels: [Channel; CHANNELS],
    frequency_high: [u8; CHANNELS],
    channel3_fnum: [u16; 3],
    channel3_block: [u8; 3],
    channel3_high: [u8; 3],
    timer_a: u16,
    timer_b: u8,
    timer_control: u8,
    timer_a_counter: u32,
    timer_b_counter: u32,
    status: u8,
    lfo_enabled: bool,
    lfo_frequency: u8,
    lfo_phase: f64,
    dac_enabled: bool,
    dac_value: u8,
    sample_phase: u64,
    samples: Vec<(f32, f32)>,
}

impl Ym2612 {
    pub fn new(clock_hz: u64) -> Self {
        Self {
            clock_hz,
            address: [0; 2],
            channels: [Channel::default(); CHANNELS],
            frequency_high: [0; CHANNELS],
            channel3_fnum: [0; 3],
            channel3_block: [0; 3],
            channel3_high: [0; 3],
            timer_a: 0,
            timer_b: 0,
            timer_control: 0,
            timer_a_counter: 0,
            timer_b_counter: 0,
            status: 0,
            lfo_enabled: false,
            lfo_frequency: 0,
            lfo_phase: 0.0,
            dac_enabled: false,
            dac_value: 0x80,
            sample_phase: 0,
            samples: Vec::with_capacity(1024),
        }
    }

    pub fn reset(&mut self) {
        let clock_hz = self.clock_hz;
        *self = Self::new(clock_hz);
    }

    pub fn read_status(&self) -> u8 {
        self.status
    }
    pub fn write_port(&mut self, port: u8, value: u8) {
        match port & 3 {
            0 => self.address[0] = value,
            1 => self.write_register(0, self.address[0], value),
            2 => self.address[1] = value,
            3 => self.write_register(1, self.address[1], value),
            _ => unreachable!(),
        }
    }

    fn channel_for(bank: usize, address: u8) -> Option<usize> {
        let local = usize::from(address & 3);
        if local == 3 {
            None
        } else {
            Some(bank * 3 + local)
        }
    }

    fn operator_for(address: u8) -> usize {
        match (address >> 2) & 3 {
            0 => 0,
            1 => 2,
            2 => 1,
            _ => 3,
        }
    }

    fn write_register(&mut self, bank: usize, address: u8, value: u8) {
        if bank == 0 {
            match address {
                0x22 => {
                    self.lfo_enabled = value & 0x08 != 0;
                    self.lfo_frequency = value & 7;
                    if !self.lfo_enabled {
                        self.lfo_phase = 0.0;
                    }
                    return;
                }
                0x24 => {
                    self.timer_a = (self.timer_a & 0x0003) | (u16::from(value) << 2);
                    return;
                }
                0x25 => {
                    self.timer_a = (self.timer_a & 0x03fc) | u16::from(value & 3);
                    return;
                }
                0x26 => {
                    self.timer_b = value;
                    return;
                }
                0x27 => {
                    self.timer_control = value;
                    if value & 0x10 != 0 {
                        self.status &= !1;
                    }
                    if value & 0x20 != 0 {
                        self.status &= !2;
                    }
                    return;
                }
                0x28 => {
                    let raw_channel = value & 7;
                    let channel = match raw_channel {
                        0..=2 => usize::from(raw_channel),
                        4..=6 => usize::from(raw_channel - 1),
                        _ => return,
                    };
                    for operator in 0..OPERATORS {
                        let on = value & (0x10 << operator) != 0;
                        let op = &mut self.channels[channel].operators[operator];
                        if on && !op.key_on {
                            op.envelope_stage = 0;
                            op.envelope = op.envelope.max(0.001);
                        } else if !on && op.key_on {
                            op.envelope_stage = 3;
                        }
                        op.key_on = on;
                    }
                    return;
                }
                0x2a => {
                    self.dac_value = value;
                    return;
                }
                0x2b => {
                    self.dac_enabled = value & 0x80 != 0;
                    return;
                }
                _ => {}
            }
        }
        if (0x30..=0x9f).contains(&address) {
            let Some(channel) = Self::channel_for(bank, address) else {
                return;
            };
            let operator = Self::operator_for(address);
            let op = &mut self.channels[channel].operators[operator];
            match address & 0xf0 {
                0x30 => {
                    op.detune = (value >> 4) & 7;
                    op.multiple = value & 0x0f;
                }
                0x40 => op.total_level = value & 0x7f,
                0x50 => {
                    op.rate_scale = (value >> 6) & 3;
                    op.attack = value & 0x1f;
                }
                0x60 => {
                    op.amplitude_modulation = value & 0x80 != 0;
                    op.decay = value & 0x1f;
                }
                0x70 => op.sustain_rate = value & 0x1f,
                0x80 => {
                    op.sustain_level = value >> 4;
                    op.release = value & 0x0f;
                }
                0x90 => op.ssg_eg = value & 0x0f,
                _ => {}
            }
            return;
        }
        if bank == 0 && matches!(address, 0xa8..=0xaa | 0xac..=0xae) {
            let slot = match address & 3 {
                0 => 2,
                1 => 0,
                2 => 1,
                _ => return,
            };
            if address & 4 == 0 {
                let high = self.channel3_high[slot];
                self.channel3_fnum[slot] = u16::from(value) | (u16::from(high & 7) << 8);
                self.channel3_block[slot] = (high >> 3) & 7;
            } else {
                self.channel3_high[slot] = value & 0x3f;
            }
            return;
        }
        if (0xa0..=0xa6).contains(&address) {
            let Some(channel) = Self::channel_for(bank, address) else {
                return;
            };
            if address & 4 == 0 {
                let high = self.frequency_high[channel];
                self.channels[channel].fnum = u16::from(value) | (u16::from(high & 7) << 8);
                self.channels[channel].block = (high >> 3) & 7;
            } else {
                self.frequency_high[channel] = value & 0x3f;
            }
            return;
        }
        if (0xb0..=0xb6).contains(&address) {
            let Some(channel) = Self::channel_for(bank, address) else {
                return;
            };
            if address & 4 == 0 {
                self.channels[channel].algorithm = value & 7;
                self.channels[channel].feedback = (value >> 3) & 7;
            } else {
                self.channels[channel].pan_left = value & 0x80 != 0;
                self.channels[channel].pan_right = value & 0x40 != 0;
                self.channels[channel].amplitude_sensitivity = (value >> 4) & 3;
                self.channels[channel].phase_sensitivity = value & 7;
            }
        }
    }

    fn timer_tick(&mut self, cycles: u32) {
        if self.timer_control & 1 != 0 {
            self.timer_a_counter = self.timer_a_counter.saturating_add(cycles);
            let period = u32::from(1024u16.saturating_sub(self.timer_a.max(1))).max(1) * 18;
            while self.timer_a_counter >= period {
                self.timer_a_counter -= period;
                if (self.timer_control >> 6) & 3 == 2 {
                    for operator in &mut self.channels[2].operators {
                        if operator.key_on {
                            continue;
                        }
                        operator.phase = 0.0;
                        operator.envelope = if operator.attack >= 30 { 1.0 } else { 0.0 };
                        operator.envelope_stage = 3;
                    }
                }
                if self.timer_control & 4 != 0 {
                    self.status |= 1;
                }
            }
        }
        if self.timer_control & 2 != 0 {
            self.timer_b_counter = self.timer_b_counter.saturating_add(cycles);
            let period = u32::from(256u16.saturating_sub(u16::from(self.timer_b)).max(1)) * 288;
            while self.timer_b_counter >= period {
                self.timer_b_counter -= period;
                if self.timer_control & 8 != 0 {
                    self.status |= 2;
                }
            }
        }
    }
    fn sustain_amplitude(level: u8) -> f32 {
        if level >= 15 {
            0.0
        } else {
            10.0f32.powf(-(f32::from(level) * 3.0) / 20.0)
        }
    }

    fn envelope_rate(rate: u8, scale: f32) -> f32 {
        if rate == 0 {
            0.0
        } else {
            let normalized = f32::from(rate) / 31.0;
            normalized * normalized * scale
        }
    }

    fn advance_envelope(operator: &mut Operator) {
        match operator.envelope_stage {
            0 => {
                if operator.attack >= 31 {
                    operator.envelope = 1.0;
                    operator.envelope_stage = 1;
                } else {
                    let rate = Self::envelope_rate(operator.attack, 0.08);
                    operator.envelope += (1.0 - operator.envelope) * rate;
                    if operator.envelope >= 0.999 {
                        operator.envelope = 1.0;
                        operator.envelope_stage = 1;
                    }
                }
            }
            1 => {
                let target = Self::sustain_amplitude(operator.sustain_level);
                let rate = Self::envelope_rate(operator.decay, 0.01);
                operator.envelope = (operator.envelope - rate).max(target);
                if operator.envelope <= target {
                    operator.envelope_stage = 2;
                }
            }
            2 => {
                let rate = Self::envelope_rate(operator.sustain_rate, 0.004);
                operator.envelope = (operator.envelope - rate).max(0.0);
            }
            _ => {
                let release = operator.release.saturating_mul(2).saturating_add(1).min(31);
                let rate = Self::envelope_rate(release, 0.006);
                operator.envelope = (operator.envelope - rate).max(0.0);
            }
        }
    }

    fn operator_sample(operator: &mut Operator, base_hz: f64, modulation: f64) -> f32 {
        Self::advance_envelope(operator);
        let total_level_db = f32::from(operator.total_level) * 0.75;
        let level = 10.0f32.powf(-total_level_db / 20.0);
        let multiple = if operator.multiple == 0 {
            0.5
        } else {
            f64::from(operator.multiple)
        };
        let phase_step = std::f64::consts::TAU * base_hz * multiple / SAMPLE_RATE as f64;
        operator.phase = (operator.phase + phase_step) % std::f64::consts::TAU;
        ((operator.phase + modulation).sin() as f32) * operator.envelope * level
    }

    fn operator_sample_lfo(
        operator: &mut Operator,
        base_hz: f64,
        modulation: f64,
        lfo: f64,
        amplitude_sensitivity: u8,
    ) -> f32 {
        let mut sample = Self::operator_sample(operator, base_hz, modulation);
        if operator.amplitude_modulation && amplitude_sensitivity != 0 {
            let depth_db = [0.0f64, 1.4, 5.9, 11.8][usize::from(amplitude_sensitivity & 3)];
            sample *= 10.0f32.powf((-(lfo * depth_db) / 20.0) as f32);
        }
        sample
    }

    fn channel_sample(
        channel: &mut Channel,
        clock_hz: u64,
        lfo: f64,
        special_frequency: Option<([u16; 3], [u8; 3])>,
    ) -> f32 {
        if channel.fnum == 0 && special_frequency.is_none() {
            return 0.0;
        }
        let frequency_hz = |fnum: u16, block: u8| {
            let block_scale = 2f64.powi(i32::from(block) - 1);
            f64::from(fnum) * block_scale * clock_hz as f64 / (144.0 * 1_048_576.0)
        };
        let normal_hz = frequency_hz(channel.fnum, channel.block);
        let mut base_hz = [normal_hz; OPERATORS];
        if let Some((fnum, block)) = special_frequency {
            for operator in 0..3 {
                base_hz[operator] = frequency_hz(fnum[operator], block[operator]);
            }
        }
        let phase_depth = [0.0f64, 0.034, 0.067, 0.10, 0.14, 0.20, 0.40, 0.80]
            [usize::from(channel.phase_sensitivity & 7)];
        if phase_depth != 0.0 {
            let ratio = 2.0f64.powf((lfo * phase_depth) / 12.0);
            for frequency in &mut base_hz {
                *frequency *= ratio;
            }
        }
        let feedback = if channel.feedback == 0 {
            0.0
        } else {
            channel.operators[0].phase.sin() * f64::from(1u16 << (channel.feedback - 1)) * 0.015
        };
        let modulation_scale = 4.0f64;
        let ams = channel.amplitude_sensitivity;

        let s1 =
            Self::operator_sample_lfo(&mut channel.operators[0], base_hz[0], feedback, lfo, ams);
        match channel.algorithm {
            0 => {
                let s3 = Self::operator_sample_lfo(
                    &mut channel.operators[2],
                    base_hz[2],
                    f64::from(s1) * modulation_scale,
                    lfo,
                    ams,
                );
                let s2 = Self::operator_sample_lfo(
                    &mut channel.operators[1],
                    base_hz[1],
                    f64::from(s3) * modulation_scale,
                    lfo,
                    ams,
                );
                Self::operator_sample_lfo(
                    &mut channel.operators[3],
                    base_hz[3],
                    f64::from(s2) * modulation_scale,
                    lfo,
                    ams,
                )
            }
            1 => {
                let s3 =
                    Self::operator_sample_lfo(&mut channel.operators[2], base_hz[2], 0.0, lfo, ams);
                let s2 = Self::operator_sample_lfo(
                    &mut channel.operators[1],
                    base_hz[1],
                    f64::from(s1 + s3) * modulation_scale,
                    lfo,
                    ams,
                );
                Self::operator_sample_lfo(
                    &mut channel.operators[3],
                    base_hz[3],
                    f64::from(s2) * modulation_scale,
                    lfo,
                    ams,
                )
            }
            2 => {
                let s3 =
                    Self::operator_sample_lfo(&mut channel.operators[2], base_hz[2], 0.0, lfo, ams);
                let s2 = Self::operator_sample_lfo(
                    &mut channel.operators[1],
                    base_hz[1],
                    f64::from(s3) * modulation_scale,
                    lfo,
                    ams,
                );
                Self::operator_sample_lfo(
                    &mut channel.operators[3],
                    base_hz[3],
                    f64::from(s1 + s2) * modulation_scale,
                    lfo,
                    ams,
                )
            }
            3 => {
                let s3 = Self::operator_sample_lfo(
                    &mut channel.operators[2],
                    base_hz[2],
                    f64::from(s1) * modulation_scale,
                    lfo,
                    ams,
                );
                let s2 =
                    Self::operator_sample_lfo(&mut channel.operators[1], base_hz[1], 0.0, lfo, ams);
                Self::operator_sample_lfo(
                    &mut channel.operators[3],
                    base_hz[3],
                    f64::from(s2 + s3) * modulation_scale,
                    lfo,
                    ams,
                )
            }
            4 => {
                let s2 = Self::operator_sample_lfo(
                    &mut channel.operators[1],
                    base_hz[1],
                    f64::from(s1) * modulation_scale,
                    lfo,
                    ams,
                );
                let s3 =
                    Self::operator_sample_lfo(&mut channel.operators[2], base_hz[2], 0.0, lfo, ams);
                let s4 = Self::operator_sample_lfo(
                    &mut channel.operators[3],
                    base_hz[3],
                    f64::from(s3) * modulation_scale,
                    lfo,
                    ams,
                );
                (s2 + s4) * 0.5
            }
            5 => {
                let modulation = f64::from(s1) * modulation_scale;
                let s2 = Self::operator_sample_lfo(
                    &mut channel.operators[1],
                    base_hz[1],
                    modulation,
                    lfo,
                    ams,
                );
                let s3 = Self::operator_sample_lfo(
                    &mut channel.operators[2],
                    base_hz[2],
                    modulation,
                    lfo,
                    ams,
                );
                let s4 = Self::operator_sample_lfo(
                    &mut channel.operators[3],
                    base_hz[3],
                    modulation,
                    lfo,
                    ams,
                );
                (s2 + s3 + s4) / 3.0
            }
            6 => {
                let s2 = Self::operator_sample_lfo(
                    &mut channel.operators[1],
                    base_hz[1],
                    f64::from(s1) * modulation_scale,
                    lfo,
                    ams,
                );
                let s3 =
                    Self::operator_sample_lfo(&mut channel.operators[2], base_hz[2], 0.0, lfo, ams);
                let s4 =
                    Self::operator_sample_lfo(&mut channel.operators[3], base_hz[3], 0.0, lfo, ams);
                (s2 + s3 + s4) / 3.0
            }
            _ => {
                let s2 =
                    Self::operator_sample_lfo(&mut channel.operators[1], base_hz[1], 0.0, lfo, ams);
                let s3 =
                    Self::operator_sample_lfo(&mut channel.operators[2], base_hz[2], 0.0, lfo, ams);
                let s4 =
                    Self::operator_sample_lfo(&mut channel.operators[3], base_hz[3], 0.0, lfo, ams);
                (s1 + s2 + s3 + s4) * 0.25
            }
        }
    }
    fn render_sample(&mut self) -> (f32, f32) {
        let lfo = if self.lfo_enabled {
            let value = self.lfo_phase.sin();
            let frequency = [3.82f64, 5.33, 5.77, 6.11, 6.60, 9.23, 46.11, 69.22]
                [usize::from(self.lfo_frequency & 7)];
            self.lfo_phase = (self.lfo_phase
                + std::f64::consts::TAU * frequency / SAMPLE_RATE as f64)
                % std::f64::consts::TAU;
            value
        } else {
            0.0
        };
        let mut left = 0.0f32;
        let mut right = 0.0f32;
        let channel3_special = (self.timer_control >> 6) & 3 != 0;
        let channel3_fnum = self.channel3_fnum;
        let channel3_block = self.channel3_block;
        for (index, channel) in self.channels.iter_mut().enumerate() {
            let special =
                (index == 2 && channel3_special).then_some((channel3_fnum, channel3_block));
            let mut sample = Self::channel_sample(channel, self.clock_hz, lfo, special) * 0.24;
            if index == 5 && self.dac_enabled {
                sample = (f32::from(self.dac_value) - 128.0) / 128.0 * 0.35;
            }
            if channel.pan_left {
                left += sample;
            }
            if channel.pan_right {
                right += sample;
            }
        }
        (left.clamp(-1.0, 1.0), right.clamp(-1.0, 1.0))
    }
    pub fn begin_frame(&mut self) {
        self.samples.clear();
    }

    pub fn tick(&mut self, cycles: u32) {
        self.timer_tick(cycles);
        for _ in 0..cycles {
            self.sample_phase += SAMPLE_RATE;
            if self.sample_phase >= self.clock_hz {
                self.sample_phase -= self.clock_hz;
                let sample = self.render_sample();
                self.samples.push(sample);
            }
        }
    }

    pub fn samples(&self) -> &[(f32, f32)] {
        &self.samples
    }

    #[cfg(test)]
    pub fn irq_pending(&self) -> bool {
        self.status & 3 != 0
    }
    pub fn save(&self, out: &mut StateWriter) {
        out.u8(self.address[0]);
        out.u8(self.address[1]);
        for value in self.frequency_high {
            out.u8(value);
        }
        for value in self.channel3_fnum {
            out.u16(value);
        }
        for value in self.channel3_block {
            out.u8(value);
        }
        for value in self.channel3_high {
            out.u8(value);
        }
        for channel in &self.channels {
            out.u16(channel.fnum);
            out.u8(channel.block);
            out.u8(channel.algorithm);
            out.u8(channel.feedback);
            out.u8(channel.pan_left as u8);
            out.u8(channel.pan_right as u8);
            out.u8(channel.amplitude_sensitivity);
            out.u8(channel.phase_sensitivity);
            for operator in &channel.operators {
                out.u8(operator.detune);
                out.u8(operator.multiple);
                out.u8(operator.total_level);
                out.u8(operator.rate_scale);
                out.u8(operator.attack);
                out.u8(operator.decay);
                out.u8(operator.sustain_rate);
                out.u8(operator.sustain_level);
                out.u8(operator.release);
                out.u8(u8::from(operator.amplitude_modulation));
                out.u8(operator.ssg_eg);
                out.f64(operator.phase);
                out.f32(operator.envelope);
                out.u8(operator.envelope_stage);
                out.u8(u8::from(operator.key_on));
            }
        }
        out.u16(self.timer_a);
        out.u8(self.timer_b);
        out.u8(self.timer_control);
        out.u32(self.timer_a_counter);
        out.u32(self.timer_b_counter);
        out.u8(self.status);
        out.u8(u8::from(self.lfo_enabled));
        out.u8(self.lfo_frequency);
        out.f64(self.lfo_phase);
        out.u8(self.dac_enabled as u8);
        out.u8(self.dac_value);
        out.u64(self.sample_phase);
    }
    pub fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.address[0] = input.u8()?;
        self.address[1] = input.u8()?;
        for value in &mut self.frequency_high {
            *value = input.u8()? & 0x3f;
        }
        for value in &mut self.channel3_fnum {
            *value = input.u16()? & 0x07ff;
        }
        for value in &mut self.channel3_block {
            *value = input.u8()? & 7;
        }
        for value in &mut self.channel3_high {
            *value = input.u8()? & 0x3f;
        }
        for channel in &mut self.channels {
            channel.fnum = input.u16()?;
            channel.block = input.u8()?;
            channel.algorithm = input.u8()?;
            channel.feedback = input.u8()?;
            channel.pan_left = input.u8()? != 0;
            channel.pan_right = input.u8()? != 0;
            channel.amplitude_sensitivity = input.u8()? & 3;
            channel.phase_sensitivity = input.u8()? & 7;
            for operator in &mut channel.operators {
                operator.detune = input.u8()? & 7;
                operator.multiple = input.u8()? & 0x0f;
                operator.total_level = input.u8()? & 0x7f;
                operator.rate_scale = input.u8()? & 3;
                operator.attack = input.u8()? & 0x1f;
                operator.decay = input.u8()? & 0x1f;
                operator.sustain_rate = input.u8()? & 0x1f;
                operator.sustain_level = input.u8()? & 0x0f;
                operator.release = input.u8()? & 0x0f;
                operator.amplitude_modulation = input.u8()? != 0;
                operator.ssg_eg = input.u8()? & 0x0f;
                operator.phase = input.f64()?;
                operator.envelope = input.f32()?.clamp(0.0, 1.0);
                operator.envelope_stage = input.u8()?.min(3);
                operator.key_on = input.u8()? != 0;
            }
        }
        self.timer_a = input.u16()?;
        self.timer_b = input.u8()?;
        self.timer_control = input.u8()?;
        self.timer_a_counter = input.u32()?;
        self.timer_b_counter = input.u32()?;
        self.status = input.u8()?;
        self.lfo_enabled = input.u8()? != 0;
        self.lfo_frequency = input.u8()? & 7;
        self.lfo_phase = input.f64()? % std::f64::consts::TAU;
        self.dac_enabled = input.u8()? != 0;
        self.dac_value = input.u8()?;
        self.sample_phase = input.u64()?;
        self.samples.clear();
        Ok(())
    }
}
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dac_generates_stereo_samples() {
        let mut ym = Ym2612::new(7_670_454);
        ym.write_port(0, 0x2b);
        ym.write_port(1, 0x80);
        ym.write_port(0, 0x2a);
        ym.write_port(1, 0xff);
        ym.tick(7_670_454 / 60);
        assert!(ym
            .samples()
            .iter()
            .any(|(left, right)| *left > 0.1 && *right > 0.1));
    }

    #[test]
    fn keyed_operator_path_generates_audio() {
        let mut ym = Ym2612::new(7_670_454);
        for slot in [0x40, 0x44, 0x48, 0x4c] {
            ym.write_port(0, slot);
            ym.write_port(1, 0x00);
        }
        for slot in [0x50, 0x54, 0x58, 0x5c] {
            ym.write_port(0, slot);
            ym.write_port(1, 0x1f);
        }
        ym.write_port(0, 0xa0);
        ym.write_port(1, 0x80);
        ym.write_port(0, 0xa4);
        ym.write_port(1, 0x22);
        ym.write_port(0, 0x28);
        ym.write_port(1, 0xf0);
        ym.tick(7_670_454 / 60);
        assert!(ym.samples().iter().any(|(left, _)| left.abs() > 0.001));
    }
    #[test]
    fn operator_registers_keep_decay_sustain_and_level_fields_independent() {
        let mut ym = Ym2612::new(7_670_454);
        for (register, value) in [
            (0x30, 0x30),
            (0x50, 0xdf),
            (0x60, 0x9a),
            (0x70, 0x15),
            (0x80, 0xa7),
            (0x90, 0x0d),
        ] {
            ym.write_port(0, register);
            ym.write_port(1, value);
        }
        let op = ym.channels[0].operators[0];
        assert_eq!(op.detune, 3);
        assert_eq!(op.multiple, 0);
        assert_eq!(op.rate_scale, 3);
        assert_eq!(op.attack, 0x1f);
        assert!(op.amplitude_modulation);
        assert_eq!(op.decay, 0x1a);
        assert_eq!(op.sustain_rate, 0x15);
        assert_eq!(op.sustain_level, 0x0a);
        assert_eq!(op.release, 7);
        assert_eq!(op.ssg_eg, 0x0d);
    }

    #[test]
    fn zero_multiple_runs_at_half_the_operator_frequency() {
        let mut half = Operator {
            multiple: 0,
            total_level: 0,
            envelope: 1.0,
            envelope_stage: 2,
            ..Operator::default()
        };
        let mut one = Operator {
            multiple: 1,
            total_level: 0,
            envelope: 1.0,
            envelope_stage: 2,
            ..Operator::default()
        };
        let _ = Ym2612::operator_sample(&mut half, 440.0, 0.0);
        let _ = Ym2612::operator_sample(&mut one, 440.0, 0.0);
        assert!((half.phase * 2.0 - one.phase).abs() < 1.0e-12);
    }

    #[test]
    fn algorithm_zero_requires_final_carrier_while_seven_sums_all_operators() {
        let mut channel = Channel {
            fnum: 0x400,
            block: 4,
            algorithm: 0,
            ..Channel::default()
        };
        channel.operators[0].total_level = 0;
        channel.operators[0].envelope = 1.0;
        channel.operators[0].envelope_stage = 2;
        channel.operators[0].phase = std::f64::consts::FRAC_PI_2;
        let chained = Ym2612::channel_sample(&mut channel, 7_670_454, 0.0, None);
        assert!(chained.abs() < 1.0e-6);

        channel.algorithm = 7;
        channel.operators[0].phase = std::f64::consts::FRAC_PI_2;
        let additive = Ym2612::channel_sample(&mut channel, 7_670_454, 0.0, None);
        assert!(additive.abs() > 0.05);
    }
    #[test]
    fn lfo_and_channel_sensitivity_registers_drive_phase_and_amplitude_modulation() {
        let mut ym = Ym2612::new(7_670_454);
        ym.write_port(0, 0x22);
        ym.write_port(1, 0x0f);
        ym.write_port(0, 0xb4);
        ym.write_port(1, 0xb7);
        assert!(ym.lfo_enabled);
        assert_eq!(ym.lfo_frequency, 7);
        assert!(ym.channels[0].pan_left);
        assert!(!ym.channels[0].pan_right);
        assert_eq!(ym.channels[0].amplitude_sensitivity, 3);
        assert_eq!(ym.channels[0].phase_sensitivity, 7);

        let mut plain = Channel {
            fnum: 0x400,
            block: 4,
            algorithm: 7,
            ..Channel::default()
        };
        plain.operators[0].total_level = 0;
        plain.operators[0].envelope = 1.0;
        plain.operators[0].envelope_stage = 2;
        let mut modulated = plain;
        modulated.phase_sensitivity = 7;
        let _ = Ym2612::channel_sample(&mut plain, 7_670_454, 1.0, None);
        let _ = Ym2612::channel_sample(&mut modulated, 7_670_454, 1.0, None);
        assert!(modulated.operators[0].phase > plain.operators[0].phase);

        let mut dry = Operator {
            multiple: 1,
            total_level: 0,
            envelope: 1.0,
            envelope_stage: 2,
            phase: std::f64::consts::FRAC_PI_2,
            ..Operator::default()
        };
        let mut wet = dry;
        wet.amplitude_modulation = true;
        let dry_sample = Ym2612::operator_sample_lfo(&mut dry, 0.0, 0.0, 1.0, 3);
        let wet_sample = Ym2612::operator_sample_lfo(&mut wet, 0.0, 0.0, 1.0, 3);
        assert!(wet_sample.abs() < dry_sample.abs() * 0.4);
    }
    #[test]
    fn frequency_writes_latch_high_half_and_channel_three_special_mode_is_per_operator() {
        let mut ym = Ym2612::new(7_670_454);
        ym.write_port(0, 0xa4);
        ym.write_port(1, 0x22);
        assert_eq!(ym.channels[0].fnum, 0);
        assert_eq!(ym.channels[0].block, 0);
        ym.write_port(0, 0xa0);
        ym.write_port(1, 0x80);
        assert_eq!(ym.channels[0].fnum, 0x280);
        assert_eq!(ym.channels[0].block, 4);

        for (high_register, low_register, high, low) in [
            (0xad, 0xa9, 0x19, 0x11),
            (0xae, 0xaa, 0x22, 0x22),
            (0xac, 0xa8, 0x2b, 0x33),
        ] {
            ym.write_port(0, high_register);
            ym.write_port(1, high);
            ym.write_port(0, low_register);
            ym.write_port(1, low);
        }
        ym.write_port(0, 0xa6);
        ym.write_port(1, 0x34);
        ym.write_port(0, 0xa2);
        ym.write_port(1, 0x44);
        assert_eq!(ym.channel3_fnum, [0x111, 0x222, 0x333]);
        assert_eq!(ym.channel3_block, [3, 4, 5]);
        assert_eq!(ym.channels[2].fnum, 0x444);
        assert_eq!(ym.channels[2].block, 6);

        let mut channel = ym.channels[2];
        channel.algorithm = 7;
        for operator in &mut channel.operators {
            operator.multiple = 1;
            operator.total_level = 0;
            operator.envelope = 1.0;
            operator.envelope_stage = 2;
            operator.sustain_rate = 0;
        }
        let _ = Ym2612::channel_sample(
            &mut channel,
            7_670_454,
            0.0,
            Some((ym.channel3_fnum, ym.channel3_block)),
        );
        assert!(channel.operators[0].phase < channel.operators[1].phase);
        assert!(channel.operators[1].phase < channel.operators[2].phase);
        assert!(channel.operators[2].phase < channel.operators[3].phase);
    }

    #[test]
    fn csm_timer_a_pulses_only_manually_keyed_off_channel_three_operators() {
        let mut ym = Ym2612::new(7_670_454);
        for operator in &mut ym.channels[2].operators {
            operator.attack = 31;
            operator.release = 1;
            operator.phase = 1.0;
        }
        ym.timer_a = 0x03ff;
        ym.timer_control = 0x81;
        ym.timer_tick(18);
        for operator in &ym.channels[2].operators {
            assert_eq!(operator.phase, 0.0);
            assert_eq!(operator.envelope, 1.0);
            assert_eq!(operator.envelope_stage, 3);
            assert!(!operator.key_on);
        }

        ym.channels[2].operators[0].key_on = true;
        ym.channels[2].operators[0].phase = 1.25;
        ym.channels[2].operators[0].envelope = 0.5;
        ym.timer_tick(18);
        assert_eq!(ym.channels[2].operators[0].phase, 1.25);
        assert_eq!(ym.channels[2].operators[0].envelope, 0.5);
        assert!(ym.channels[2].operators[0].key_on);
    }
    #[test]
    fn timers_and_state_round_trip() {
        let mut ym = Ym2612::new(7_670_454);
        ym.write_port(0, 0x24);
        ym.write_port(1, 0xff);
        ym.write_port(0, 0x25);
        ym.write_port(1, 0x03);
        ym.write_port(0, 0x27);
        ym.write_port(1, 0x05);
        ym.tick(200);
        assert!(ym.irq_pending());
        let mut out = StateWriter::new(crate::platform::PlatformId::Genesis, 1);
        ym.save(&mut out);
        let bytes = out.finish();
        let mut restored = Ym2612::new(7_670_454);
        let mut input = StateReader::new(&bytes, crate::platform::PlatformId::Genesis, 1).unwrap();
        restored.load(&mut input).unwrap();
        input.finish().unwrap();
        assert_eq!(restored.read_status(), ym.read_status());
        assert_eq!(restored.dac_enabled, ym.dac_enabled);
        assert_eq!(restored.channels[0].fnum, ym.channels[0].fnum);
    }
}
