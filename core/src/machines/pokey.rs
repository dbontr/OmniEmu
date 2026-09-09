use crate::state::{StateReader, StateWriter};

const SAMPLE_RATE: u32 = 48_000;
const DEFAULT_CPU_HZ: u64 = 1_789_790;

#[derive(Debug, Clone, Copy)]
struct AudioChannel {
    frequency: u8,
    control: u8,
    counter: u16,
    phase: bool,
}

impl Default for AudioChannel {
    fn default() -> Self {
        Self {
            frequency: 0,
            control: 0,
            counter: 1,
            phase: false,
        }
    }
}

impl AudioChannel {
    fn period(&self, clock_divider: u16) -> u16 {
        (u16::from(self.frequency) + 1)
            .saturating_mul(clock_divider)
            .max(1)
    }

    fn volume(&self) -> f32 {
        f32::from(self.control & 0x0f) / 15.0
    }

    fn volume_only(&self) -> bool {
        self.control & 0x10 != 0
    }
}

#[derive(Debug, Clone)]
pub struct Pokey {
    channels: [AudioChannel; 4],
    audctl: u8,
    irq_enable: u8,
    irq_status: u8,
    skctl: u8,
    pots: [u8; 8],
    keyboard: u8,
    random: u32,
    cpu_hz: u64,
    sample_phase: u64,
    samples: Vec<f32>,
}

impl Default for Pokey {
    fn default() -> Self {
        Self::new(DEFAULT_CPU_HZ)
    }
}

impl Pokey {
    pub fn new(cpu_hz: u64) -> Self {
        Self {
            channels: [AudioChannel::default(); 4],
            audctl: 0,
            irq_enable: 0,
            irq_status: 0xff,
            skctl: 0,
            pots: [0x80; 8],
            keyboard: 0xff,
            random: 0x1ffff,
            cpu_hz,
            sample_phase: 0,
            samples: Vec::with_capacity(1024),
        }
    }

    pub fn reset(&mut self) {
        let cpu_hz = self.cpu_hz;
        let pots = self.pots;
        *self = Self::new(cpu_hz);
        self.pots = pots;
    }

    pub fn set_pot(&mut self, index: usize, value: u8) {
        if let Some(pot) = self.pots.get_mut(index) {
            *pot = value;
        }
    }

    pub fn set_keyboard(&mut self, value: u8) {
        self.keyboard = value;
    }

    pub fn read(&mut self, address: u16) -> u8 {
        match address & 0x0f {
            0x00..=0x07 => self.pots[(address & 7) as usize],
            0x08 => self.pots.iter().copied().max().unwrap_or(0xff),
            0x09 => self.keyboard,
            0x0a => {
                self.clock_random();
                (self.random & 0xff) as u8
            }
            0x0d => self.irq_status,
            0x0f => self.skctl,
            _ => 0xff,
        }
    }

    pub fn write(&mut self, address: u16, value: u8) {
        match address & 0x0f {
            0x00 | 0x02 | 0x04 | 0x06 => {
                let channel = ((address & 0x06) >> 1) as usize;
                self.channels[channel].frequency = value;
            }
            0x01 | 0x03 | 0x05 | 0x07 => {
                let channel = ((address & 0x06) >> 1) as usize;
                self.channels[channel].control = value;
            }
            0x08 => self.audctl = value,
            0x09 => {
                for channel in &mut self.channels {
                    channel.counter = 1;
                }
            }
            0x0a => self.irq_status = 0xff,
            0x0d => {
                self.irq_enable = value;
                self.irq_status |= !value;
            }
            0x0f => self.skctl = value,
            _ => {}
        }
    }

    pub fn irq_pending(&self) -> bool {
        self.irq_status & self.irq_enable != self.irq_enable
    }

    fn channel_divider(&self, index: usize) -> u16 {
        let base = if self.audctl & 0x01 != 0 { 1 } else { 28 };
        if (index == 0 && self.audctl & 0x40 != 0) || (index == 2 && self.audctl & 0x20 != 0) {
            1
        } else {
            base
        }
    }

    fn clock_random(&mut self) {
        let feedback = ((self.random >> 16) ^ (self.random >> 11)) & 1;
        self.random = ((self.random << 1) | feedback) & 0x1ffff;
        if self.random == 0 {
            self.random = 0x1ffff;
        }
    }

    fn tick_audio_cycle(&mut self) {
        self.clock_random();
        for index in 0..self.channels.len() {
            let divider = self.channel_divider(index);
            let channel = &mut self.channels[index];
            if channel.counter <= 1 {
                channel.counter = channel.period(divider);
                let distortion = channel.control & 0xe0;
                channel.phase = match distortion {
                    0xa0 | 0xe0 => !channel.phase,
                    0x80 | 0xc0 => self.random & 1 != 0,
                    _ => (self.random >> (index + 1)) & 1 != 0,
                };
                if self.irq_enable & (1 << index) != 0 {
                    self.irq_status &= !(1 << index);
                }
            } else {
                channel.counter -= 1;
            }
        }
    }

    fn mix(&self) -> f32 {
        let mut total = 0.0f32;
        for channel in &self.channels {
            if channel.volume_only() || channel.phase {
                total += channel.volume();
            }
        }
        (total / 4.0).clamp(0.0, 1.0)
    }

    pub fn tick(&mut self, cycles: u32) {
        for _ in 0..cycles {
            self.tick_audio_cycle();
            self.sample_phase += u64::from(SAMPLE_RATE);
            if self.sample_phase >= self.cpu_hz {
                self.sample_phase -= self.cpu_hz;
                self.samples.push(self.mix());
            }
        }
    }

    pub fn take_samples(&mut self) -> Vec<f32> {
        std::mem::take(&mut self.samples)
    }

    pub fn save(&self, out: &mut StateWriter) {
        for channel in &self.channels {
            out.u8(channel.frequency);
            out.u8(channel.control);
            out.u16(channel.counter);
            out.u8(channel.phase as u8);
        }
        out.u8(self.audctl);
        out.u8(self.irq_enable);
        out.u8(self.irq_status);
        out.u8(self.skctl);
        out.blob(&self.pots);
        out.u8(self.keyboard);
        out.u32(self.random);
        out.u64(self.sample_phase);
    }

    pub fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        for channel in &mut self.channels {
            channel.frequency = input.u8()?;
            channel.control = input.u8()?;
            channel.counter = input.u16()?;
            channel.phase = input.u8()? != 0;
        }
        self.audctl = input.u8()?;
        self.irq_enable = input.u8()?;
        self.irq_status = input.u8()?;
        self.skctl = input.u8()?;
        let pots = input.blob()?;
        if pots.len() != self.pots.len() {
            return Err("POKEY state has invalid POT register length".into());
        }
        self.pots.copy_from_slice(pots);
        self.keyboard = input.u8()?;
        self.random = input.u32()? & 0x1ffff;
        if self.random == 0 {
            self.random = 0x1ffff;
        }
        self.sample_phase = input.u64()?;
        self.samples.clear();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::platform::PlatformId;

    #[test]
    fn tone_channel_generates_samples_and_irq() {
        let mut pokey = Pokey::new(DEFAULT_CPU_HZ);
        pokey.write(0x00, 3);
        pokey.write(0x01, 0xaf);
        pokey.write(0x0d, 0x01);
        pokey.tick(20_000);
        let samples = pokey.take_samples();
        assert!(!samples.is_empty());
        assert!(samples.iter().any(|&sample| sample > 0.0));
        assert!(pokey.irq_pending());
    }

    #[test]
    fn state_round_trip_preserves_registers_and_pots() {
        let mut pokey = Pokey::new(DEFAULT_CPU_HZ);
        pokey.write(0x04, 17);
        pokey.write(0x05, 0xcf);
        pokey.set_pot(3, 0x42);
        pokey.tick(1234);
        let mut writer = StateWriter::new(PlatformId::Atari5200, 9);
        pokey.save(&mut writer);
        let state = writer.finish();
        let mut restored = Pokey::new(DEFAULT_CPU_HZ);
        let mut reader = StateReader::new(&state, PlatformId::Atari5200, 9).unwrap();
        restored.load(&mut reader).unwrap();
        reader.finish().unwrap();
        assert_eq!(restored.channels[2].frequency, 17);
        assert_eq!(restored.channels[2].control, 0xcf);
        assert_eq!(restored.pots[3], 0x42);
        assert_eq!(restored.sample_phase, pokey.sample_phase);
    }
}
