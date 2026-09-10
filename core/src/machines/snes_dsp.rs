use crate::state::{StateReader, StateWriter};

const VOICES: usize = 8;
const DSP_RATE: u32 = 32_000;
const SPC_CYCLES_PER_SAMPLE: u32 = 32;
const ENV_MAX: u16 = 0x07ff;

#[derive(Clone, Copy)]
struct Voice {
    active: bool,
    release: bool,
    block_addr: u16,
    loop_addr: u16,
    decoded: [i16; 16],
    sample_index: u8,
    pitch_phase: u32,
    history1: i32,
    history2: i32,
    envelope: u16,
    env_stage: u8,
    end_after_block: bool,
}

impl Default for Voice {
    fn default() -> Self {
        Self {
            active: false,
            release: false,
            block_addr: 0,
            loop_addr: 0,
            decoded: [0; 16],
            sample_index: 16,
            pitch_phase: 0,
            history1: 0,
            history2: 0,
            envelope: 0,
            env_stage: 0,
            end_after_block: false,
        }
    }
}

pub(crate) struct SnesDsp {
    regs: [u8; 128],
    voices: [Voice; VOICES],
    key_on: u8,
    key_off: u8,
    cycle_phase: u32,
    noise: u16,
    noise_phase: u32,
    samples: Vec<f32>,
}

impl Default for SnesDsp {
    fn default() -> Self {
        Self {
            regs: [0; 128],
            voices: [Voice::default(); VOICES],
            key_on: 0,
            key_off: 0,
            cycle_phase: 0,
            noise: 0x4000,
            noise_phase: 0,
            samples: Vec::with_capacity(2048),
        }
    }
}

impl SnesDsp {
    pub(crate) fn sample_rate(&self) -> u32 {
        DSP_RATE
    }

    pub(crate) fn read(&self, address: u8) -> u8 {
        self.regs[usize::from(address & 0x7f)]
    }

    pub(crate) fn write(&mut self, address: u8, value: u8) {
        let index = usize::from(address & 0x7f);
        match index {
            0x4c => self.key_on |= value,
            0x5c => self.key_off |= value,
            0x7c => self.regs[0x7c] &= !value,
            0x6c if value & 0x80 != 0 => {
                self.regs[index] = value;
                self.voices = [Voice::default(); VOICES];
            }
            _ => self.regs[index] = value,
        }
    }

    fn read16(ram: &[u8], address: u16) -> u16 {
        let lo = ram[usize::from(address)];
        let hi = ram[usize::from(address.wrapping_add(1))];
        u16::from_le_bytes([lo, hi])
    }

    fn key_on_voice(&mut self, index: usize, ram: &[u8]) {
        let base = index << 4;
        let directory = u16::from(self.regs[0x5d]) << 8;
        let source = u16::from(self.regs[base + 4]);
        let entry = directory.wrapping_add(source * 4);
        let start = Self::read16(ram, entry);
        let loop_addr = Self::read16(ram, entry.wrapping_add(2));
        self.voices[index] = Voice {
            active: true,
            block_addr: start,
            loop_addr,
            ..Voice::default()
        };
        self.regs[0x7c] &= !(1 << index);
        self.regs[base + 8] = 0;
        self.regs[base + 9] = 0;
    }
    fn apply_keys(&mut self, ram: &[u8]) {
        let key_off = std::mem::take(&mut self.key_off);
        for index in 0..VOICES {
            if key_off & (1 << index) != 0 {
                self.voices[index].release = true;
                self.voices[index].env_stage = 3;
            }
        }
        let key_on = std::mem::take(&mut self.key_on);
        for index in 0..VOICES {
            if key_on & (1 << index) != 0 {
                self.key_on_voice(index, ram);
            }
        }
    }

    fn brr_sample(nibble: u8, range: u8) -> i32 {
        let signed = if nibble & 8 != 0 {
            i32::from(nibble) - 16
        } else {
            i32::from(nibble)
        };
        if range <= 12 {
            signed << range
        } else if signed < 0 {
            -2048
        } else {
            0
        }
    }
    fn filtered_sample(filter: u8, sample: i32, h1: i32, h2: i32) -> i32 {
        let value = match filter {
            0 => sample,
            1 => sample + ((h1 * 15) >> 4),
            2 => sample + ((h1 * 61) >> 5) - ((h2 * 15) >> 4),
            _ => sample + ((h1 * 115) >> 6) - ((h2 * 13) >> 4),
        };
        value.clamp(i32::from(i16::MIN), i32::from(i16::MAX))
    }

    fn decode_block(&mut self, index: usize, ram: &[u8]) {
        let voice = &mut self.voices[index];
        if !voice.active {
            return;
        }
        let header = ram[usize::from(voice.block_addr)];
        let range = header >> 4;
        let filter = (header >> 2) & 3;
        let mut h1 = voice.history1;
        let mut h2 = voice.history2;
        for sample_index in 0..16 {
            let byte =
                ram[usize::from(voice.block_addr.wrapping_add(1 + (sample_index / 2) as u16))];
            let nibble = if sample_index & 1 == 0 {
                byte >> 4
            } else {
                byte & 0x0f
            };
            let raw = Self::brr_sample(nibble, range);
            let decoded = Self::filtered_sample(filter, raw, h1, h2);
            voice.decoded[sample_index] = decoded as i16;
            h2 = h1;
            h1 = decoded;
        }
        voice.history1 = h1;
        voice.history2 = h2;
        voice.sample_index = 0;
        let end = header & 1 != 0;
        let looping = header & 2 != 0;
        if end {
            self.regs[0x7c] |= 1 << index;
            if looping {
                voice.block_addr = voice.loop_addr;
                voice.end_after_block = false;
            } else {
                voice.block_addr = voice.block_addr.wrapping_add(9);
                voice.end_after_block = true;
            }
        } else {
            voice.block_addr = voice.block_addr.wrapping_add(9);
        }
    }

    fn advance_sample(&mut self, index: usize, ram: &[u8]) {
        if self.voices[index].sample_index >= 16 {
            self.decode_block(index, ram);
        }
        if !self.voices[index].active {
            return;
        }
        let pitch = u32::from(
            u16::from_le_bytes([self.regs[index * 16 + 2], self.regs[index * 16 + 3]]) & 0x3fff,
        );
        self.voices[index].pitch_phase = self.voices[index].pitch_phase.saturating_add(pitch);
        while self.voices[index].pitch_phase >= 0x1000 {
            self.voices[index].pitch_phase -= 0x1000;
            self.voices[index].sample_index = self.voices[index].sample_index.saturating_add(1);
            if self.voices[index].sample_index >= 16 {
                if self.voices[index].end_after_block {
                    self.voices[index].active = false;
                    self.voices[index].envelope = 0;
                    break;
                }
                self.decode_block(index, ram);
                if !self.voices[index].active {
                    break;
                }
            }
        }
    }

    fn envelope_step(&mut self, index: usize) {
        let base = index << 4;
        let adsr1 = self.regs[base + 5];
        let adsr2 = self.regs[base + 6];
        let gain = self.regs[base + 7];
        let voice = &mut self.voices[index];
        if !voice.active {
            voice.envelope = 0;
            return;
        }
        if voice.release {
            voice.envelope = voice.envelope.saturating_sub(8);
            if voice.envelope == 0 {
                voice.active = false;
            }
            return;
        }
        if adsr1 & 0x80 == 0 {
            if gain & 0x80 == 0 {
                voice.envelope = (u16::from(gain & 0x7f) << 4).min(ENV_MAX);
            } else {
                let amount = 1 + u16::from(gain & 0x1f);
                match (gain >> 5) & 3 {
                    0 => voice.envelope = voice.envelope.saturating_sub(amount),
                    1 => voice.envelope = voice.envelope.saturating_sub((amount / 2).max(1)),
                    2 => voice.envelope = (voice.envelope + amount).min(ENV_MAX),
                    _ => voice.envelope = (voice.envelope + amount * 2).min(ENV_MAX),
                }
            }
            return;
        }
        let sustain = (u16::from((adsr2 >> 5) & 7) + 1) * 0x100 - 1;
        match voice.env_stage {
            0 => {
                let step = if adsr1 & 0x0f == 0x0f {
                    0x400
                } else {
                    8 + u16::from(adsr1 & 0x0f) * 8
                };
                voice.envelope = (voice.envelope + step).min(ENV_MAX);
                if voice.envelope == ENV_MAX {
                    voice.env_stage = 1;
                }
            }
            1 => {
                let step = 1 + u16::from((adsr1 >> 4) & 7) * 2;
                voice.envelope = voice.envelope.saturating_sub(step);
                if voice.envelope <= sustain {
                    voice.env_stage = 2;
                }
            }
            _ => {
                let rate = u16::from(adsr2 & 0x1f);
                let step = if rate == 0 { 0 } else { 1 + rate / 4 };
                voice.envelope = voice.envelope.saturating_sub(step);
                if voice.envelope > sustain {
                    voice.envelope = sustain;
                }
            }
        }
    }

    fn noise_sample(&mut self) -> i16 {
        self.noise_phase = self.noise_phase.wrapping_add(1);
        let rate = u32::from(self.regs[0x6c] & 0x1f) + 1;
        if self.noise_phase >= (32 - rate.min(31)).max(1) {
            self.noise_phase = 0;
            let feedback = (self.noise ^ (self.noise >> 1)) & 1;
            self.noise = (self.noise >> 1) | (feedback << 14);
            if self.noise == 0 {
                self.noise = 0x4000;
            }
        }
        ((self.noise as i16) << 1) >> 1
    }

    fn voice_sample(&mut self, index: usize, ram: &[u8], noise: i16) -> i16 {
        if !self.voices[index].active {
            return 0;
        }
        if self.voices[index].sample_index >= 16 {
            self.decode_block(index, ram);
        }
        let sample = if self.regs[0x3d] & (1 << index) != 0 {
            noise
        } else {
            self.voices[index].decoded[usize::from(self.voices[index].sample_index.min(15))]
        };
        self.envelope_step(index);
        let envelope = i32::from(self.voices[index].envelope);
        let output = (i32::from(sample) * envelope / i32::from(ENV_MAX))
            .clamp(i32::from(i16::MIN), i32::from(i16::MAX));
        self.regs[index * 16 + 8] = (self.voices[index].envelope >> 4) as u8;
        self.regs[index * 16 + 9] = (output >> 8) as u8;
        self.advance_sample(index, ram);
        output as i16
    }

    fn signed_volume(value: u8) -> f32 {
        f32::from(value as i8) / 128.0
    }

    fn tick_sample(&mut self, ram: &[u8]) {
        self.apply_keys(ram);
        let noise = self.noise_sample();
        let mut left = 0.0f32;
        let mut right = 0.0f32;
        for index in 0..VOICES {
            let sample = f32::from(self.voice_sample(index, ram, noise)) / 32768.0;
            let base = index << 4;
            left += sample * Self::signed_volume(self.regs[base]);
            right += sample * Self::signed_volume(self.regs[base + 1]);
        }
        left *= Self::signed_volume(self.regs[0x0c]);
        right *= Self::signed_volume(self.regs[0x1c]);
        if self.regs[0x6c] & 0x40 != 0 {
            left = 0.0;
            right = 0.0;
        }
        self.samples.push(left.clamp(-1.0, 1.0));
        self.samples.push(right.clamp(-1.0, 1.0));
    }

    pub(crate) fn tick(&mut self, ram: &[u8], cycles: u32) {
        self.cycle_phase = self.cycle_phase.saturating_add(cycles);
        while self.cycle_phase >= SPC_CYCLES_PER_SAMPLE {
            self.cycle_phase -= SPC_CYCLES_PER_SAMPLE;
            self.tick_sample(ram);
        }
    }

    pub(crate) fn take_samples(&mut self) -> Vec<f32> {
        std::mem::take(&mut self.samples)
    }

    pub(crate) fn save(&self, out: &mut StateWriter) {
        out.blob(&self.regs);
        out.u8(self.key_on);
        out.u8(self.key_off);
        out.u32(self.cycle_phase);
        out.u16(self.noise);
        out.u32(self.noise_phase);
        for voice in &self.voices {
            out.u8(voice.active as u8);
            out.u8(voice.release as u8);
            out.u16(voice.block_addr);
            out.u16(voice.loop_addr);
            for sample in voice.decoded {
                out.u16(sample as u16);
            }
            out.u8(voice.sample_index);
            out.u32(voice.pitch_phase);
            out.u32(voice.history1 as u32);
            out.u32(voice.history2 as u32);
            out.u16(voice.envelope);
            out.u8(voice.env_stage);
            out.u8(voice.end_after_block as u8);
        }
    }

    pub(crate) fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let regs = input.blob()?;
        if regs.len() != self.regs.len() {
            return Err("invalid SNES DSP register state length".into());
        }
        self.regs.copy_from_slice(regs);
        self.key_on = input.u8()?;
        self.key_off = input.u8()?;
        self.cycle_phase = input.u32()? % SPC_CYCLES_PER_SAMPLE;
        self.noise = input.u16()? & 0x7fff;
        self.noise_phase = input.u32()?;
        for voice in &mut self.voices {
            voice.active = input.u8()? != 0;
            voice.release = input.u8()? != 0;
            voice.block_addr = input.u16()?;
            voice.loop_addr = input.u16()?;
            for sample in &mut voice.decoded {
                *sample = input.u16()? as i16;
            }
            voice.sample_index = input.u8()?.min(16);
            voice.pitch_phase = input.u32()? & 0x0fff;
            voice.history1 = input.u32()? as i32;
            voice.history2 = input.u32()? as i32;
            voice.envelope = input.u16()?.min(ENV_MAX);
            voice.env_stage = input.u8()?.min(3);
            voice.end_after_block = input.u8()? != 0;
        }
        self.samples.clear();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn configure_test_voice(dsp: &mut SnesDsp, ram: &mut [u8]) {
        ram[0x1000..0x1004].copy_from_slice(&[0x00, 0x20, 0x00, 0x20]);
        ram[0x2000] = 0xc3;
        for byte in &mut ram[0x2001..0x2009] {
            *byte = 0x77;
        }
        dsp.write(0x5d, 0x10);
        dsp.write(0x00, 0x7f);
        dsp.write(0x01, 0x7f);
        dsp.write(0x02, 0x00);
        dsp.write(0x03, 0x10);
        dsp.write(0x04, 0x00);
        dsp.write(0x05, 0x00);
        dsp.write(0x07, 0x7f);
        dsp.write(0x0c, 0x7f);
        dsp.write(0x1c, 0x7f);
        dsp.write(0x4c, 0x01);
    }

    #[test]
    fn brr_voice_generates_stereo_samples() {
        let mut ram = vec![0; 65_536];
        let mut dsp = SnesDsp::default();
        configure_test_voice(&mut dsp, &mut ram);
        dsp.tick(&ram, 32 * 64);
        let samples = dsp.take_samples();
        assert_eq!(samples.len(), 128);
        assert!(samples.iter().any(|sample| sample.abs() > 0.01));
        assert_ne!(dsp.read(0x7c) & 1, 0);
    }

    #[test]
    fn key_off_releases_voice() {
        let mut ram = vec![0; 65_536];
        let mut dsp = SnesDsp::default();
        configure_test_voice(&mut dsp, &mut ram);
        dsp.tick(&ram, 32 * 8);
        dsp.write(0x5c, 1);
        dsp.tick(&ram, 32 * 300);
        assert_eq!(dsp.regs[0x08], 0);
    }
    #[test]
    fn state_round_trip_preserves_voice_and_registers() {
        let mut ram = vec![0; 65_536];
        let mut dsp = SnesDsp::default();
        configure_test_voice(&mut dsp, &mut ram);
        dsp.tick(&ram, 32 * 4);
        let mut out = StateWriter::new(crate::platform::PlatformId::Snes, 99);
        dsp.save(&mut out);
        let bytes = out.finish();
        dsp.write(0x0c, 0);
        let mut input = StateReader::new(&bytes, crate::platform::PlatformId::Snes, 99).unwrap();
        dsp.load(&mut input).unwrap();
        input.finish().unwrap();
        assert_eq!(dsp.read(0x0c), 0x7f);
        assert!(dsp.voices[0].active);
    }
}
