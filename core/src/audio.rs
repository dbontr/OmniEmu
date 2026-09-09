#[derive(Debug, Clone)]
pub struct AudioMixer {
    sample_rate: u32,
    frames: usize,
    stereo: Vec<f32>,
}

impl AudioMixer {
    pub fn new(sample_rate: u32) -> Result<Self, String> {
        if sample_rate == 0 {
            return Err("audio sample rate cannot be zero".into());
        }
        Ok(Self {
            sample_rate,
            frames: 0,
            stereo: Vec::new(),
        })
    }

    pub fn begin(&mut self, frames: usize) {
        self.frames = frames;
        self.stereo.clear();
        self.stereo.resize(frames.saturating_mul(2), 0.0);
    }

    pub fn sample_rate(&self) -> u32 {
        self.sample_rate
    }
    pub fn frames(&self) -> usize {
        self.frames
    }
    pub fn samples(&self) -> &[f32] {
        &self.stereo
    }

    pub fn mix_interleaved(
        &mut self,
        source: &[f32],
        source_rate: u32,
        channels: u8,
        gain: f32,
    ) -> Result<(), String> {
        if source_rate == 0 {
            return Err("source sample rate cannot be zero".into());
        }
        if channels != 1 && channels != 2 {
            return Err("audio mixer supports mono or stereo sources".into());
        }
        let channels_usize = channels as usize;
        if !source.len().is_multiple_of(channels_usize) {
            return Err("audio source is not aligned to its channel count".into());
        }
        let source_frames = source.len() / channels_usize;
        if source_frames == 0 || self.frames == 0 {
            return Ok(());
        }

        for output_frame in 0..self.frames {
            let position_num = (output_frame as u128) * (source_rate as u128);
            let base = usize::try_from(position_num / self.sample_rate as u128)
                .map_err(|_| "audio resample position overflow".to_string())?;
            if base >= source_frames {
                break;
            }
            let next = (base + 1).min(source_frames - 1);
            let fraction_num = (position_num % self.sample_rate as u128) as f32;
            let fraction = fraction_num / self.sample_rate as f32;
            let sample = |frame: usize, channel: usize| -> f32 {
                let actual_channel = if channels == 1 { 0 } else { channel };
                source[frame * channels_usize + actual_channel]
            };
            for channel in 0..2 {
                let a = sample(base, channel);
                let b = sample(next, channel);
                self.stereo[output_frame * 2 + channel] += (a + (b - a) * fraction) * gain;
            }
        }
        Ok(())
    }

    pub fn clamp(&mut self) {
        for sample in &mut self.stereo {
            *sample = sample.clamp(-1.0, 1.0);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mixes_mono_and_stereo_sources_into_one_output() {
        let mut mixer = AudioMixer::new(48_000).unwrap();
        mixer.begin(4);
        mixer
            .mix_interleaved(&[0.25, 0.5, 0.75, 1.0], 48_000, 1, 1.0)
            .unwrap();
        mixer
            .mix_interleaved(
                &[0.1, -0.1, 0.2, -0.2, 0.3, -0.3, 0.4, -0.4],
                48_000,
                2,
                1.0,
            )
            .unwrap();
        assert!((mixer.samples()[0] - 0.35).abs() < 1e-6);
        assert!((mixer.samples()[1] - 0.15).abs() < 1e-6);
    }

    #[test]
    fn resamples_using_integer_position_math() {
        let mut mixer = AudioMixer::new(48_000).unwrap();
        mixer.begin(4);
        mixer.mix_interleaved(&[0.0, 1.0], 24_000, 1, 1.0).unwrap();
        assert!((mixer.samples()[2] - 0.5).abs() < 1e-6);
        assert!((mixer.samples()[4] - 1.0).abs() < 1e-6);
    }

    #[test]
    fn clamp_limits_combined_output() {
        let mut mixer = AudioMixer::new(48_000).unwrap();
        mixer.begin(1);
        mixer.mix_interleaved(&[2.0], 48_000, 1, 1.0).unwrap();
        mixer.clamp();
        assert_eq!(mixer.samples(), &[1.0, 1.0]);
    }
}
