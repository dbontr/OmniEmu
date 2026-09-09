use crate::state::{StateReader, StateWriter};

const PSG_CLOCK: u64 = 3_579_545;
const SAMPLE_RATE: u64 = 48_000;
const VOLUME: [f32; 16] = [
    1.0,
    0.794_328_2,
    0.630_957_37,
    0.501_187_2,
    0.398_107_17,
    0.316_227_76,
    0.251_188_64,
    0.199_526_24,
    0.158_489_32,
    0.125_892_53,
    0.1,
    0.079_432_82,
    0.063_095_73,
    0.050_118_72,
    0.039_810_72,
    0.0,
];

#[derive(Debug, Clone)]
pub struct Sn76489 {
    tone_period: [u16; 3],
    tone_counter: [u16; 3],
    tone_level: [bool; 3],
    volume: [u8; 4],
    noise_control: u8,
    noise_counter: u16,
    noise_lfsr: u16,
    noise_level: bool,
    latched_register: u8,
    divider: u8,
    sample_phase: u64,
    samples: Vec<f32>,
}

impl Default for Sn76489 {
    fn default() -> Self {
        Self {
            tone_period: [1; 3],
            tone_counter: [1; 3],
            tone_level: [false; 3],
            volume: [15; 4],
            noise_control: 0,
            noise_counter: 0x10,
            noise_lfsr: 0x8000,
            noise_level: false,
            latched_register: 0,
            divider: 0,
            sample_phase: 0,
            samples: Vec::with_capacity(1024),
        }
    }
}

impl Sn76489 {
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    pub fn begin_frame(&mut self) {
        self.samples.clear();
    }

    pub fn write(&mut self, value: u8) {
        if value & 0x80 != 0 {
            self.latched_register = (value >> 4) & 7;
            self.write_latched(value & 0x0f, true);
        } else {
            self.write_latched(value & 0x3f, false);
        }
    }

    fn write_latched(&mut self, data: u8, latch: bool) {
        let channel = (self.latched_register >> 1) as usize;
        let volume = self.latched_register & 1 != 0;
        if volume {
            if latch {
                self.volume[channel] = data & 0x0f;
            }
            return;
        }
        if channel == 3 {
            if latch {
                self.noise_control = data & 7;
                self.noise_lfsr = 0x8000;
                self.reload_noise();
            }
            return;
        }
        if latch {
            self.tone_period[channel] =
                (self.tone_period[channel] & 0x03f0) | u16::from(data & 0x0f);
        } else {
            self.tone_period[channel] =
                (self.tone_period[channel] & 0x000f) | (u16::from(data & 0x3f) << 4);
        }
        if self.tone_period[channel] == 0 {
            self.tone_period[channel] = 0x400;
        }
    }

    pub fn tick_cpu_cycles(&mut self, cycles: u32) {
        for _ in 0..cycles {
            self.sample_phase += SAMPLE_RATE;
            if self.sample_phase >= PSG_CLOCK {
                self.sample_phase -= PSG_CLOCK;
                self.samples.push(self.mix());
            }
            self.divider = self.divider.wrapping_add(1);
            if self.divider == 16 {
                self.divider = 0;
                self.tick_divided_clock();
            }
        }
    }

    fn tick_divided_clock(&mut self) {
        for channel in 0..3 {
            if self.tone_counter[channel] == 0 {
                self.tone_counter[channel] = self.tone_period[channel].max(1);
                self.tone_level[channel] = !self.tone_level[channel];
            } else {
                self.tone_counter[channel] -= 1;
            }
        }
        if self.noise_counter == 0 {
            self.reload_noise();
            let bit0 = self.noise_lfsr & 1;
            let feedback = if self.noise_control & 4 != 0 {
                bit0 ^ ((self.noise_lfsr >> 3) & 1)
            } else {
                bit0
            };
            self.noise_lfsr = (self.noise_lfsr >> 1) | (feedback << 15);
            self.noise_level = self.noise_lfsr & 1 == 0;
        } else {
            self.noise_counter -= 1;
        }
    }

    fn reload_noise(&mut self) {
        self.noise_counter = match self.noise_control & 3 {
            0 => 0x10,
            1 => 0x20,
            2 => 0x40,
            _ => self.tone_period[2].max(1),
        };
    }

    fn mix(&self) -> f32 {
        let mut sample = 0.0;
        for channel in 0..3 {
            if self.tone_level[channel] {
                sample += VOLUME[self.volume[channel] as usize];
            }
        }
        if self.noise_level {
            sample += VOLUME[self.volume[3] as usize];
        }
        (sample * 0.22).clamp(-1.0, 1.0)
    }

    pub fn samples(&self) -> &[f32] {
        &self.samples
    }

    pub fn save(&self, out: &mut StateWriter) {
        for value in self.tone_period {
            out.u16(value);
        }
        for value in self.tone_counter {
            out.u16(value);
        }
        for value in self.tone_level {
            out.u8(value as u8);
        }
        for value in self.volume {
            out.u8(value);
        }
        out.u8(self.noise_control);
        out.u16(self.noise_counter);
        out.u16(self.noise_lfsr);
        out.u8(self.noise_level as u8);
        out.u8(self.latched_register);
        out.u8(self.divider);
        out.u64(self.sample_phase);
    }

    pub fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        for value in &mut self.tone_period {
            *value = input.u16()?;
        }
        for value in &mut self.tone_counter {
            *value = input.u16()?;
        }
        for value in &mut self.tone_level {
            *value = input.u8()? != 0;
        }
        for value in &mut self.volume {
            *value = input.u8()?;
        }
        self.noise_control = input.u8()?;
        self.noise_counter = input.u16()?;
        self.noise_lfsr = input.u16()?;
        self.noise_level = input.u8()? != 0;
        self.latched_register = input.u8()?;
        self.divider = input.u8()?;
        self.sample_phase = input.u64()?;
        self.samples.clear();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tone_channel_produces_audio() {
        let mut psg = Sn76489::default();
        psg.write(0x80 | 0x04);
        psg.write(0x10);
        psg.write(0x90);
        psg.tick_cpu_cycles(PSG_CLOCK as u32 / 30);
        assert!(psg.samples().iter().any(|sample| *sample > 0.0));
    }

    #[test]
    fn state_round_trip_preserves_registers() {
        let mut psg = Sn76489::default();
        psg.write(0x85);
        psg.write(0x12);
        let mut writer = StateWriter::new(crate::platform::PlatformId::MasterSystem, 99);
        psg.save(&mut writer);
        let bytes = writer.finish();
        let mut restored = Sn76489::default();
        let mut reader =
            StateReader::new(&bytes, crate::platform::PlatformId::MasterSystem, 99).unwrap();
        restored.load(&mut reader).unwrap();
        reader.finish().unwrap();
        assert_eq!(restored.tone_period, psg.tone_period);
        assert_eq!(restored.volume, psg.volume);
    }
}
