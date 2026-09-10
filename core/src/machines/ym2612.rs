use crate::state::{StateReader, StateWriter};

const SAMPLE_RATE: u64 = 48_000;
const CHANNELS: usize = 6;
const OPERATORS: usize = 4;

#[derive(Debug, Clone, Copy)]
struct Operator {
    multiple: u8,
    total_level: u8,
    attack: u8,
    decay: u8,
    sustain: u8,
    release: u8,
    phase: f64,
    envelope: f32,
    key_on: bool,
}

impl Default for Operator {
    fn default() -> Self {
        Self {
            multiple: 1,
            total_level: 0x7f,
            attack: 0,
            decay: 0,
            sustain: 0,
            release: 0,
            phase: 0.0,
            envelope: 0.0,
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
            operators: [Operator::default(); OPERATORS],
        }
    }
}

#[derive(Debug, Clone)]
pub struct Ym2612 {
    clock_hz: u64,
    address: [u8; 2],
    channels: [Channel; CHANNELS],
    timer_a: u16,
    timer_b: u8,
    timer_control: u8,
    timer_a_counter: u32,
    timer_b_counter: u32,
    status: u8,
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
            timer_a: 0,
            timer_b: 0,
            timer_control: 0,
            timer_a_counter: 0,
            timer_b_counter: 0,
            status: 0,
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
                0x22 => return,
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
                            op.envelope = op.envelope.max(0.001);
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
                0x30 => op.multiple = (value & 0x0f).max(1),
                0x40 => op.total_level = value & 0x7f,
                0x50 => op.attack = value & 0x1f,
                0x60 => op.decay = value & 0x1f,
                0x70 => op.sustain = value & 0x1f,
                0x80 => {
                    op.sustain = value >> 4;
                    op.release = value & 0x0f;
                }
                _ => {}
            }
            return;
        }
        if (0xa0..=0xa6).contains(&address) {
            let Some(channel) = Self::channel_for(bank, address) else {
                return;
            };
            if address & 4 == 0 {
                self.channels[channel].fnum =
                    (self.channels[channel].fnum & 0x0700) | u16::from(value);
            } else {
                self.channels[channel].fnum =
                    (self.channels[channel].fnum & 0x00ff) | (u16::from(value & 7) << 8);
                self.channels[channel].block = (value >> 3) & 7;
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
            }
        }
    }

    fn timer_tick(&mut self, cycles: u32) {
        if self.timer_control & 1 != 0 {
            self.timer_a_counter = self.timer_a_counter.saturating_add(cycles);
            let period = u32::from(1024u16.saturating_sub(self.timer_a.max(1))).max(1) * 18;
            while self.timer_a_counter >= period {
                self.timer_a_counter -= period;
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
    fn operator_sample(operator: &mut Operator, base_hz: f64, modulation: f64) -> f32 {
        let attack = (f32::from(operator.attack) + 1.0) / 2048.0;
        let release = (f32::from(operator.release) + 1.0) / 16384.0;
        if operator.key_on {
            operator.envelope = (operator.envelope + attack).min(1.0);
        } else {
            operator.envelope = (operator.envelope - release).max(0.0);
        }
        let level = (1.0 - f32::from(operator.total_level) / 127.0).max(0.0);
        let multiple = f64::from(operator.multiple.max(1));
        let phase_step = std::f64::consts::TAU * base_hz * multiple / SAMPLE_RATE as f64;
        operator.phase = (operator.phase + phase_step) % std::f64::consts::TAU;
        ((operator.phase + modulation).sin() as f32) * operator.envelope * level
    }

    fn channel_sample(channel: &mut Channel) -> f32 {
        if channel.fnum == 0 {
            return 0.0;
        }
        let block_scale = 2f64.powi(i32::from(channel.block) - 1);
        let base_hz = f64::from(channel.fnum) * block_scale * 7_670_454.0 / (144.0 * 1_048_576.0);
        let feedback = f64::from(channel.feedback) * 0.08;
        let o0 = Self::operator_sample(&mut channel.operators[0], base_hz, 0.0);
        let o1_mod = if channel.algorithm <= 3 {
            f64::from(o0) * (1.0 + feedback)
        } else {
            0.0
        };
        let o1 = Self::operator_sample(&mut channel.operators[1], base_hz, o1_mod);
        let o2_mod = if channel.algorithm <= 1 {
            f64::from(o1)
        } else {
            0.0
        };
        let o2 = Self::operator_sample(&mut channel.operators[2], base_hz, o2_mod);
        let o3_mod = if channel.algorithm <= 3 {
            f64::from(o2)
        } else {
            0.0
        };
        let o3 = Self::operator_sample(&mut channel.operators[3], base_hz, o3_mod);
        match channel.algorithm {
            0..=1 => o3,
            2..=3 => (o1 + o3) * 0.5,
            4..=5 => (o0 + o2 + o3) / 3.0,
            _ => (o0 + o1 + o2 + o3) * 0.25,
        }
    }
    fn render_sample(&mut self) -> (f32, f32) {
        let mut left = 0.0f32;
        let mut right = 0.0f32;
        for (index, channel) in self.channels.iter_mut().enumerate() {
            let mut sample = Self::channel_sample(channel) * 0.24;
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
        for channel in &self.channels {
            out.u16(channel.fnum);
            out.u8(channel.block);
            out.u8(channel.algorithm);
            out.u8(channel.feedback);
            out.u8(channel.pan_left as u8);
            out.u8(channel.pan_right as u8);
            for operator in &channel.operators {
                out.u8(operator.multiple);
                out.u8(operator.total_level);
                out.u8(operator.attack);
                out.u8(operator.decay);
                out.u8(operator.sustain);
                out.u8(operator.release);
                out.f64(operator.phase);
                out.f32(operator.envelope);
                out.u8(operator.key_on as u8);
            }
        }
        out.u16(self.timer_a);
        out.u8(self.timer_b);
        out.u8(self.timer_control);
        out.u32(self.timer_a_counter);
        out.u32(self.timer_b_counter);
        out.u8(self.status);
        out.u8(self.dac_enabled as u8);
        out.u8(self.dac_value);
        out.u64(self.sample_phase);
    }
    pub fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.address[0] = input.u8()?;
        self.address[1] = input.u8()?;
        for channel in &mut self.channels {
            channel.fnum = input.u16()?;
            channel.block = input.u8()?;
            channel.algorithm = input.u8()?;
            channel.feedback = input.u8()?;
            channel.pan_left = input.u8()? != 0;
            channel.pan_right = input.u8()? != 0;
            for operator in &mut channel.operators {
                operator.multiple = input.u8()?;
                operator.total_level = input.u8()?;
                operator.attack = input.u8()?;
                operator.decay = input.u8()?;
                operator.sustain = input.u8()?;
                operator.release = input.u8()?;
                operator.phase = input.f64()?;
                operator.envelope = input.f32()?;
                operator.key_on = input.u8()? != 0;
            }
        }
        self.timer_a = input.u16()?;
        self.timer_b = input.u8()?;
        self.timer_control = input.u8()?;
        self.timer_a_counter = input.u32()?;
        self.timer_b_counter = input.u32()?;
        self.status = input.u8()?;
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
