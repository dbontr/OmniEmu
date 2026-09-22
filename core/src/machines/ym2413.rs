use crate::state::{StateReader, StateWriter};

const CLOCK_HZ: u64 = 3_579_545;
const SAMPLE_RATE: u64 = 48_000;
const CHANNELS: usize = 9;

const PATCHES: [[u8; 8]; 16] = [
    [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
    [0x71, 0x61, 0x1e, 0x17, 0xd0, 0x78, 0x00, 0x17],
    [0x13, 0x41, 0x1a, 0x0d, 0xd8, 0xf7, 0x23, 0x13],
    [0x13, 0x01, 0x99, 0x00, 0xf2, 0xc4, 0x11, 0x23],
    [0x31, 0x61, 0x0e, 0x07, 0xa8, 0x64, 0x70, 0x27],
    [0x32, 0x21, 0x1e, 0x06, 0xe0, 0x76, 0x00, 0x28],
    [0x31, 0x22, 0x16, 0x05, 0xe0, 0x71, 0x00, 0x18],
    [0x21, 0x61, 0x1d, 0x07, 0x82, 0x81, 0x10, 0x07],
    [0x23, 0x21, 0x2d, 0x14, 0xa2, 0x72, 0x00, 0x07],
    [0x61, 0x61, 0x1b, 0x06, 0x64, 0x65, 0x10, 0x17],
    [0x41, 0x61, 0x0b, 0x18, 0x85, 0xf7, 0x71, 0x07],
    [0x13, 0x01, 0x83, 0x11, 0xfa, 0xe4, 0x10, 0x04],
    [0x17, 0xc1, 0x24, 0x07, 0xf8, 0xf8, 0x22, 0x12],
    [0x61, 0x50, 0x0c, 0x05, 0xc2, 0xf5, 0x20, 0x42],
    [0x01, 0x01, 0x55, 0x03, 0xc9, 0x95, 0x03, 0x02],
    [0x61, 0x41, 0x89, 0x03, 0xf1, 0xe4, 0x40, 0x13],
];

#[derive(Clone, Debug, Default)]
struct Channel {
    mod_phase: f32,
    carrier_phase: f32,
    envelope: f32,
    feedback: f32,
    key_on: bool,
    attacking: bool,
}

#[derive(Clone, Debug)]
pub struct Ym2413 {
    selected_register: u8,
    registers: [u8; 0x40],
    channels: [Channel; CHANNELS],
    sample_phase: u64,
    lfo_phase: f32,
    rhythm_phase: [f32; 5],
    rhythm_envelope: [f32; 5],
    rhythm_key: [bool; 5],
    noise_lfsr: u32,
    samples: Vec<f32>,
}

impl Default for Ym2413 {
    fn default() -> Self {
        Self {
            selected_register: 0,
            registers: [0; 0x40],
            channels: std::array::from_fn(|_| Channel::default()),
            sample_phase: 0,
            lfo_phase: 0.0,
            rhythm_phase: [0.0; 5],
            rhythm_envelope: [0.0; 5],
            rhythm_key: [false; 5],
            noise_lfsr: 1,
            samples: Vec::with_capacity(1024),
        }
    }
}

impl Ym2413 {
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    pub fn begin_frame(&mut self) {
        self.samples.clear();
    }

    pub fn select_register(&mut self, value: u8) {
        self.selected_register = value & 0x3f;
    }

    pub fn write_data(&mut self, value: u8) {
        self.registers[usize::from(self.selected_register)] = value;
    }

    pub fn samples(&self) -> &[f32] {
        &self.samples
    }

    fn patch(&self, channel: usize) -> [u8; 8] {
        let instrument = usize::from(self.registers[0x30 + channel] >> 4);
        if instrument == 0 {
            std::array::from_fn(|index| self.registers[index])
        } else {
            PATCHES[instrument]
        }
    }

    fn frequency(&self, channel: usize) -> f32 {
        let low = self.registers[0x10 + channel];
        let high = self.registers[0x20 + channel];
        let fnum = u16::from(low) | (u16::from(high & 1) << 8);
        let block = (high >> 1) & 7;
        f32::from(fnum) * 2.0_f32.powi(i32::from(block)) * CLOCK_HZ as f32 / (36.0 * 524_288.0)
    }

    fn wave(phase: f32, half_wave: bool) -> f32 {
        let sample = (std::f32::consts::TAU * phase).sin();
        if half_wave {
            sample.max(0.0)
        } else {
            sample
        }
    }

    fn advance_envelope(state: &mut Channel, key_on: bool, patch: [u8; 8]) {
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
        let dt = 1.0 / SAMPLE_RATE as f32;
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
    }

    fn melodic_sample(&mut self) -> f32 {
        let rhythm_mode = self.registers[0x0e] & 0x20 != 0;
        let channel_count = if rhythm_mode { 6 } else { CHANNELS };
        let vibrato = (std::f32::consts::TAU * self.lfo_phase).sin();
        let tremolo = (std::f32::consts::TAU * self.lfo_phase * 0.61).sin();
        let multipliers = [
            0.5, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 10.0, 12.0, 12.0, 15.0, 15.0,
        ];
        let mut mixed = 0.0;
        for channel in 0..channel_count {
            let patch = self.patch(channel);
            let high = self.registers[0x20 + channel];
            let key_on = high & 0x10 != 0;
            let volume = self.registers[0x30 + channel] & 0x0f;
            let mut frequency = self.frequency(channel);
            if patch[0] & 0x40 != 0 || patch[1] & 0x40 != 0 {
                frequency *= 1.0 + vibrato * 0.004;
            }
            let state = &mut self.channels[channel];
            Self::advance_envelope(state, key_on, patch);
            if state.envelope == 0.0 || frequency == 0.0 {
                continue;
            }
            let mod_mul = multipliers[usize::from(patch[0] & 0x0f)];
            let carrier_mul = multipliers[usize::from(patch[1] & 0x0f)];
            state.mod_phase = (state.mod_phase + frequency * mod_mul / SAMPLE_RATE as f32).fract();
            state.carrier_phase =
                (state.carrier_phase + frequency * carrier_mul / SAMPLE_RATE as f32).fract();
            let feedback_gain = f32::from(patch[3] & 7) * 0.16;
            let mod_level = 10.0_f32.powf(-(f32::from(patch[2] & 0x3f) * 0.75) / 20.0);
            let mod_sample = Self::wave(
                state.mod_phase + state.feedback * feedback_gain,
                patch[3] & 0x08 != 0,
            );
            state.feedback = mod_sample;
            let carrier = Self::wave(
                state.carrier_phase + mod_sample * mod_level * 0.64,
                patch[3] & 0x10 != 0,
            );
            let volume_gain = 10.0_f32.powf(-(f32::from(volume) * 3.0) / 20.0);
            let tremolo_gain = if patch[1] & 0x80 != 0 {
                0.92 + tremolo * 0.08
            } else {
                1.0
            };
            mixed += carrier * state.envelope * volume_gain * tremolo_gain;
        }
        mixed
    }

    fn rhythm_volume(&self, voice: usize) -> u8 {
        match voice {
            0 => self.registers[0x36] & 0x0f,
            1 => self.registers[0x37] & 0x0f,
            2 => self.registers[0x38] >> 4,
            3 => self.registers[0x38] & 0x0f,
            _ => self.registers[0x37] >> 4,
        }
    }

    fn rhythm_sample(&mut self) -> f32 {
        if self.registers[0x0e] & 0x20 == 0 {
            self.rhythm_key = [false; 5];
            return 0.0;
        }
        let masks = [0x10, 0x08, 0x04, 0x02, 0x01];
        let channels = [6usize, 7, 8, 8, 7];
        let frequency_scale = [1.0f32, 1.75, 1.0, 2.25, 3.5];
        let decay_seconds = [0.28f32, 0.09, 0.22, 0.18, 0.055];
        let feedback = ((self.noise_lfsr ^ (self.noise_lfsr >> 3)) & 1) << 22;
        self.noise_lfsr = (self.noise_lfsr >> 1) | feedback;
        let noise = if self.noise_lfsr & 1 == 0 { -1.0 } else { 1.0 };
        let mut mixed = 0.0;
        for voice in 0..5 {
            let key_on = self.registers[0x0e] & masks[voice] != 0;
            if key_on && !self.rhythm_key[voice] {
                self.rhythm_phase[voice] = 0.0;
                self.rhythm_envelope[voice] = 1.0;
            }
            self.rhythm_key[voice] = key_on;
            let envelope = self.rhythm_envelope[voice];
            if envelope <= 0.0001 {
                continue;
            }
            let frequency = self.frequency(channels[voice]).max(55.0) * frequency_scale[voice];
            self.rhythm_phase[voice] =
                (self.rhythm_phase[voice] + frequency / SAMPLE_RATE as f32).fract();
            let tonal = (std::f32::consts::TAU * self.rhythm_phase[voice]).sin();
            let source = match voice {
                0 => tonal,
                1 => tonal * 0.35 + noise * 0.65,
                2 => tonal,
                3 => tonal.signum() * 0.35 + noise * 0.65,
                _ => noise,
            };
            let volume = self.rhythm_volume(voice);
            let volume_gain = 10.0_f32.powf(-(f32::from(volume) * 3.0) / 20.0);
            mixed += source * envelope * volume_gain;
            let decay = (-1.0 / (decay_seconds[voice] * SAMPLE_RATE as f32)).exp();
            self.rhythm_envelope[voice] *= decay;
        }
        mixed
    }

    fn mix(&mut self) -> f32 {
        self.lfo_phase = (self.lfo_phase + 6.1 / SAMPLE_RATE as f32).fract();
        let melodic = self.melodic_sample();
        let rhythm = self.rhythm_sample();
        (melodic * 0.075 + rhythm * 0.09).clamp(-0.8, 0.8)
    }

    pub fn tick_cpu_cycles(&mut self, cycles: u32) {
        for _ in 0..cycles {
            self.sample_phase += SAMPLE_RATE;
            if self.sample_phase >= CLOCK_HZ {
                self.sample_phase -= CLOCK_HZ;
                let sample = self.mix();
                self.samples.push(sample);
            }
        }
    }

    pub fn save(&self, out: &mut StateWriter) {
        out.u8(self.selected_register);
        out.blob(&self.registers);
        for channel in &self.channels {
            out.u32(channel.mod_phase.to_bits());
            out.u32(channel.carrier_phase.to_bits());
            out.u32(channel.envelope.to_bits());
            out.u32(channel.feedback.to_bits());
            out.u8(channel.key_on as u8);
            out.u8(channel.attacking as u8);
        }
        out.u64(self.sample_phase);
        out.u32(self.lfo_phase.to_bits());
        for phase in self.rhythm_phase {
            out.u32(phase.to_bits());
        }
        for envelope in self.rhythm_envelope {
            out.u32(envelope.to_bits());
        }
        for key in self.rhythm_key {
            out.u8(key as u8);
        }
        out.u32(self.noise_lfsr);
    }

    pub fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.selected_register = input.u8()? & 0x3f;
        let registers = input.blob()?;
        if registers.len() != self.registers.len() {
            return Err("invalid YM2413 register state length".into());
        }
        self.registers.copy_from_slice(registers);
        for channel in &mut self.channels {
            channel.mod_phase = f32::from_bits(input.u32()?);
            channel.carrier_phase = f32::from_bits(input.u32()?);
            channel.envelope = f32::from_bits(input.u32()?);
            channel.feedback = f32::from_bits(input.u32()?);
            channel.key_on = input.u8()? != 0;
            channel.attacking = input.u8()? != 0;
        }
        self.sample_phase = input.u64()?;
        self.lfo_phase = f32::from_bits(input.u32()?);
        for phase in &mut self.rhythm_phase {
            *phase = f32::from_bits(input.u32()?);
        }
        for envelope in &mut self.rhythm_envelope {
            *envelope = f32::from_bits(input.u32()?);
        }
        for key in &mut self.rhythm_key {
            *key = input.u8()? != 0;
        }
        self.noise_lfsr = input.u32()?.max(1);
        self.samples.clear();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::platform::PlatformId;

    fn write_register(chip: &mut Ym2413, register: u8, value: u8) {
        chip.select_register(register);
        chip.write_data(value);
    }

    #[test]
    fn melodic_channel_generates_audio_from_standard_registers() {
        let mut chip = Ym2413::default();
        write_register(&mut chip, 0x30, 0x10);
        write_register(&mut chip, 0x10, 0x80);
        write_register(&mut chip, 0x20, 0x17);
        chip.tick_cpu_cycles((CLOCK_HZ / 20) as u32);
        assert!(chip.samples().iter().any(|sample| sample.abs() > 0.0001));
    }

    #[test]
    fn rhythm_mode_generates_percussion_from_dedicated_key_bits() {
        let mut chip = Ym2413::default();
        write_register(&mut chip, 0x16, 0x20);
        write_register(&mut chip, 0x26, 0x05);
        write_register(&mut chip, 0x36, 0x00);
        write_register(&mut chip, 0x0e, 0x30);
        chip.tick_cpu_cycles((CLOCK_HZ / 30) as u32);
        assert!(chip.samples().iter().any(|sample| sample.abs() > 0.0001));
    }

    #[test]
    fn state_round_trip_preserves_registers_and_synthesis_phase() {
        let mut chip = Ym2413::default();
        write_register(&mut chip, 0x30, 0x20);
        write_register(&mut chip, 0x10, 0x44);
        write_register(&mut chip, 0x20, 0x15);
        chip.tick_cpu_cycles(1000);
        let mut writer = StateWriter::new(PlatformId::MasterSystem, 99);
        chip.save(&mut writer);
        let bytes = writer.finish();

        let mut restored = Ym2413::default();
        let mut reader = StateReader::new(&bytes, PlatformId::MasterSystem, 99).unwrap();
        restored.load(&mut reader).unwrap();
        reader.finish().unwrap();
        assert_eq!(restored.registers, chip.registers);
        assert_eq!(restored.sample_phase, chip.sample_phase);
        assert_eq!(
            restored.channels[0].carrier_phase,
            chip.channels[0].carrier_phase
        );
    }
}
