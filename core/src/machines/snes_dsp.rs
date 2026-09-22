use crate::state::{StateReader, StateWriter};
use std::sync::OnceLock;

const VOICES: usize = 8;
const DSP_RATE: u32 = 32_000;
const SPC_CYCLES_PER_SAMPLE: u32 = 32;
const ENV_MAX: u16 = 0x07ff;
const COUNTER_RANGE: u16 = 30_720;
const COUNTER_RATE: [u16; 32] = [
    0, 2048, 1536, 1280, 1024, 768, 640, 512, 384, 320, 256, 192, 160, 128, 96, 80, 64, 48, 40, 32,
    24, 20, 16, 12, 10, 8, 6, 5, 4, 3, 2, 1,
];
const COUNTER_OFFSET: [u16; 32] = [
    0, 0, 1040, 536, 0, 1040, 536, 0, 1040, 536, 0, 1040, 536, 0, 1040, 536, 0, 1040, 536, 0, 1040,
    536, 0, 1040, 536, 0, 1040, 536, 0, 1040, 0, 0,
];

static GAUSSIAN_TABLE: OnceLock<[i16; 512]> = OnceLock::new();

fn gaussian_table() -> &'static [i16; 512] {
    GAUSSIAN_TABLE.get_or_init(|| {
        let mut source = [0.0f64; 512];
        let mut table = [0i16; 512];
        for n in 0..512usize {
            let k = 0.5 + n as f64;
            let s = (std::f64::consts::PI * k * 1.280 / 1024.0).sin();
            let t = ((std::f64::consts::PI * k * 2.000 / 1023.0).cos() - 1.0) * 0.50;
            let u = ((std::f64::consts::PI * k * 4.000 / 1023.0).cos() - 1.0) * 0.08;
            source[511 - n] = s * (t + u + 1.0) / k;
        }
        for phase in 0..128usize {
            let sum =
                source[phase] + source[phase + 256] + source[511 - phase] + source[255 - phase];
            let scale = 2048.0 / sum;
            for index in [phase, phase + 256, 511 - phase, 255 - phase] {
                table[index] = (source[index] * scale + 0.5) as i16;
            }
        }
        table
    })
}

#[derive(Clone, Copy)]
struct Voice {
    active: bool,
    release: bool,
    block_addr: u16,
    loop_addr: u16,
    decoded: [i16; 16],
    previous_samples: [i16; 3],
    lookahead: [i16; 2],
    sample_index: u8,
    pitch_phase: u32,
    history1: i32,
    history2: i32,
    envelope: u16,
    envelope_latch: i32,
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
            previous_samples: [0; 3],
            lookahead: [0; 2],
            sample_index: 16,
            pitch_phase: 0,
            history1: 0,
            history2: 0,
            envelope: 0,
            envelope_latch: 0,
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
    counter: u16,
    noise: u16,
    echo_history: [[i16; 8]; 2],
    echo_history_pos: u8,
    echo_offset: u16,
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
            counter: 0,
            noise: 0x4000,
            echo_history: [[0; 8]; 2],
            echo_history_pos: 0,
            echo_offset: 0,
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

    fn decode_samples(
        ram: &[u8],
        address: u16,
        mut history1: i32,
        mut history2: i32,
    ) -> ([i16; 16], i32, i32, u8) {
        let header = ram[usize::from(address)];
        let range = header >> 4;
        let filter = (header >> 2) & 3;
        let mut decoded = [0i16; 16];
        for sample_index in 0..16 {
            let byte = ram[usize::from(address.wrapping_add(1 + (sample_index / 2) as u16))];
            let nibble = if sample_index & 1 == 0 {
                byte >> 4
            } else {
                byte & 0x0f
            };
            let raw = Self::brr_sample(nibble, range);
            let sample = Self::filtered_sample(filter, raw, history1, history2);
            decoded[sample_index] = sample as i16;
            history2 = history1;
            history1 = sample;
        }
        (decoded, history1, history2, header)
    }

    fn decode_block(&mut self, index: usize, ram: &[u8]) {
        if !self.voices[index].active {
            return;
        }

        let current_addr = self.voices[index].block_addr;
        let loop_addr = self.voices[index].loop_addr;
        let old_tail = [
            self.voices[index].decoded[13],
            self.voices[index].decoded[14],
            self.voices[index].decoded[15],
        ];
        let (decoded, history1, history2, header) = Self::decode_samples(
            ram,
            current_addr,
            self.voices[index].history1,
            self.voices[index].history2,
        );
        let end = header & 1 != 0;
        let looping = header & 2 != 0;
        let next_addr = if end && looping {
            loop_addr
        } else {
            current_addr.wrapping_add(9)
        };
        let lookahead = if end && !looping {
            [0, 0]
        } else {
            let (next, _, _, _) = Self::decode_samples(ram, next_addr, history1, history2);
            [next[0], next[1]]
        };

        let voice = &mut self.voices[index];
        voice.previous_samples = old_tail;
        voice.decoded = decoded;
        voice.lookahead = lookahead;
        voice.history1 = history1;
        voice.history2 = history2;
        voice.sample_index = 0;
        if end {
            self.regs[0x7c] |= 1 << index;
            if looping {
                voice.block_addr = voice.loop_addr;
                voice.end_after_block = false;
            } else {
                voice.block_addr = current_addr.wrapping_add(9);
                voice.end_after_block = true;
            }
        } else {
            voice.block_addr = current_addr.wrapping_add(9);
            voice.end_after_block = false;
        }
    }

    fn advance_sample(&mut self, index: usize, ram: &[u8], pitch: u32) {
        if self.voices[index].sample_index >= 16 {
            self.decode_block(index, ram);
        }
        if !self.voices[index].active {
            return;
        }
        self.voices[index].pitch_phase = self.voices[index]
            .pitch_phase
            .saturating_add(pitch.min(0x3fff));
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

    fn counter_tick(&mut self) {
        if self.counter == 0 {
            self.counter = COUNTER_RANGE;
        }
        self.counter -= 1;
    }

    fn counter_poll(&self, rate: u8) -> bool {
        let rate = usize::from(rate & 0x1f);
        if rate == 0 {
            return false;
        }
        (self.counter + COUNTER_OFFSET[rate]).is_multiple_of(COUNTER_RATE[rate])
    }

    fn envelope_step(&mut self, index: usize) {
        if !self.voices[index].active {
            self.voices[index].envelope = 0;
            self.voices[index].envelope_latch = 0;
            return;
        }
        if self.voices[index].release {
            let envelope = i32::from(self.voices[index].envelope).saturating_sub(8);
            self.voices[index].envelope = envelope.max(0) as u16;
            self.voices[index].envelope_latch = envelope;
            if self.voices[index].envelope == 0 {
                self.voices[index].active = false;
            }
            return;
        }

        let base = index << 4;
        let adsr1 = self.regs[base + 5];
        let adsr2 = self.regs[base + 6];
        let gain = self.regs[base + 7];
        let mut stage = self.voices[index].env_stage;
        let mut envelope = i32::from(self.voices[index].envelope);
        let envelope_data;
        let rate: u8;

        if adsr1 & 0x80 != 0 {
            envelope_data = adsr2;
            if stage >= 1 {
                envelope -= 1;
                envelope -= envelope >> 8;
                rate = if stage == 1 {
                    ((adsr1 >> 4) & 7) * 2 + 16
                } else {
                    adsr2 & 0x1f
                };
            } else {
                rate = (adsr1 & 0x0f) * 2 + 1;
                envelope += if rate < 31 { 0x20 } else { 0x400 };
            }
        } else {
            envelope_data = gain;
            let mode = gain >> 5;
            if mode < 4 {
                envelope = i32::from(gain & 0x7f) << 4;
                rate = 31;
            } else {
                rate = gain & 0x1f;
                match mode {
                    4 => envelope -= 0x20,
                    5 => {
                        envelope -= 1;
                        envelope -= envelope >> 8;
                    }
                    6 => envelope += 0x20,
                    _ => {
                        envelope += if self.voices[index].envelope_latch >= 0x600 {
                            0x08
                        } else {
                            0x20
                        };
                    }
                }
            }
        }

        if stage == 1 && (envelope >> 8) == i32::from(envelope_data >> 5) {
            stage = 2;
        }
        let raw_envelope = envelope;
        if !(0..=i32::from(ENV_MAX)).contains(&envelope) {
            envelope = if envelope < 0 { 0 } else { i32::from(ENV_MAX) };
            if stage == 0 {
                stage = 1;
            }
        }

        let update = self.counter_poll(rate);
        let voice = &mut self.voices[index];
        voice.envelope_latch = raw_envelope;
        voice.env_stage = stage;
        if update {
            voice.envelope = envelope as u16;
        }
    }

    fn noise_sample(&mut self) -> i16 {
        if self.counter_poll(self.regs[0x6c] & 0x1f) {
            let feedback = ((self.noise << 13) ^ (self.noise << 14)) & 0x4000;
            self.noise = feedback | (self.noise >> 1);
        }
        ((self.noise as i16) << 1) >> 1
    }

    fn interpolation_sample(voice: &Voice, relative: i32) -> i16 {
        let index = i32::from(voice.sample_index) + relative;
        match index {
            -3..=-1 => voice.previous_samples[(index + 3) as usize],
            0..=15 => voice.decoded[index as usize],
            16..=17 => voice.lookahead[(index - 16) as usize],
            _ => 0,
        }
    }

    fn gaussian_interpolated_sample(voice: &Voice) -> i16 {
        let phase = ((voice.pitch_phase >> 4) & 0xff) as usize;
        let table = gaussian_table();
        let samples = [
            Self::interpolation_sample(voice, -1),
            Self::interpolation_sample(voice, 0),
            Self::interpolation_sample(voice, 1),
            Self::interpolation_sample(voice, 2),
        ];
        let mut output = (i32::from(table[255 - phase]) * i32::from(samples[0])) >> 11;
        output += (i32::from(table[511 - phase]) * i32::from(samples[1])) >> 11;
        output += (i32::from(table[256 + phase]) * i32::from(samples[2])) >> 11;
        output = i32::from(output as i16);
        output += (i32::from(table[phase]) * i32::from(samples[3])) >> 11;
        output.clamp(i32::from(i16::MIN), i32::from(i16::MAX)) as i16 & !1
    }

    fn voice_sample(&mut self, index: usize, ram: &[u8], noise: i16, previous_output: i16) -> i16 {
        if !self.voices[index].active {
            return 0;
        }
        if self.voices[index].sample_index >= 16 {
            self.decode_block(index, ram);
        }
        if !self.voices[index].active {
            return 0;
        }

        let sample = if self.regs[0x3d] & (1 << index) != 0 {
            noise
        } else {
            Self::gaussian_interpolated_sample(&self.voices[index])
        };
        let envelope = i32::from(self.voices[index].envelope);
        let output = (i32::from(sample) * envelope / i32::from(ENV_MAX))
            .clamp(i32::from(i16::MIN), i32::from(i16::MAX)) as i16;
        self.regs[index * 16 + 8] = (self.voices[index].envelope >> 4) as u8;
        self.regs[index * 16 + 9] = (output >> 8) as u8;
        self.envelope_step(index);

        let base_pitch = i32::from(
            u16::from_le_bytes([self.regs[index * 16 + 2], self.regs[index * 16 + 3]]) & 0x3fff,
        );
        let pitch = if index != 0 && self.regs[0x2d] & (1 << index) != 0 {
            let delta = ((i32::from(previous_output) >> 5) * base_pitch) >> 10;
            (base_pitch + delta).clamp(0, 0x3fff) as u32
        } else {
            base_pitch as u32
        };
        self.advance_sample(index, ram, pitch);
        output
    }

    fn signed_volume(value: u8) -> i32 {
        i32::from(value as i8)
    }

    fn clamp_i16(value: i32) -> i16 {
        value.clamp(i32::from(i16::MIN), i32::from(i16::MAX)) as i16
    }

    fn read_ram_i16(ram: &[u8], address: u16) -> i16 {
        let lo = ram[usize::from(address)];
        let hi = ram[usize::from(address.wrapping_add(1))];
        i16::from_le_bytes([lo, hi])
    }

    fn write_ram_i16(ram: &mut [u8], address: u16, value: i16) {
        let [lo, hi] = value.to_le_bytes();
        ram[usize::from(address)] = lo;
        ram[usize::from(address.wrapping_add(1))] = hi;
    }

    fn echo_fir_sample(&self, channel: usize) -> i16 {
        let mut terms = [0i32; 8];
        for (tap, term) in terms.iter_mut().enumerate() {
            let history_index = (usize::from(self.echo_history_pos) + tap + 1) & 7;
            let coefficient = i32::from(self.regs[0x0f + tap * 0x10] as i8);
            *term = (i32::from(self.echo_history[channel][history_index]) * coefficient) >> 6;
        }
        let first_seven = terms[..7].iter().copied().sum::<i32>() as i16;
        Self::clamp_i16(i32::from(first_seven) + i32::from(terms[7] as i16)) & !1
    }

    fn process_echo(&mut self, ram: &mut [u8], main: [i32; 2], echo_send: [i32; 2]) -> [i16; 2] {
        self.echo_history_pos = self.echo_history_pos.wrapping_add(1) & 7;
        let echo_address = (u16::from(self.regs[0x6d]) << 8).wrapping_add(self.echo_offset);
        let left_memory = Self::read_ram_i16(ram, echo_address);
        let right_memory = Self::read_ram_i16(ram, echo_address.wrapping_add(2));
        self.echo_history[0][usize::from(self.echo_history_pos)] = left_memory >> 1;
        self.echo_history[1][usize::from(self.echo_history_pos)] = right_memory >> 1;

        let filtered = [self.echo_fir_sample(0), self.echo_fir_sample(1)];
        let main_volume = [
            Self::signed_volume(self.regs[0x0c]),
            Self::signed_volume(self.regs[0x1c]),
        ];
        let echo_volume = [
            Self::signed_volume(self.regs[0x2c]),
            Self::signed_volume(self.regs[0x3c]),
        ];
        let feedback = Self::signed_volume(self.regs[0x0d]);

        let output = [
            Self::clamp_i16(
                ((main[0] * main_volume[0]) >> 7)
                    + ((i32::from(filtered[0]) * echo_volume[0]) >> 7),
            ),
            Self::clamp_i16(
                ((main[1] * main_volume[1]) >> 7)
                    + ((i32::from(filtered[1]) * echo_volume[1]) >> 7),
            ),
        ];

        let feedback_sample = [
            Self::clamp_i16(echo_send[0] + ((i32::from(filtered[0]) * feedback) >> 7)) & !1,
            Self::clamp_i16(echo_send[1] + ((i32::from(filtered[1]) * feedback) >> 7)) & !1,
        ];
        if self.regs[0x6c] & 0x20 == 0 {
            Self::write_ram_i16(ram, echo_address, feedback_sample[0]);
            Self::write_ram_i16(ram, echo_address.wrapping_add(2), feedback_sample[1]);
        }

        let delay_bytes = usize::from(self.regs[0x7d] & 0x0f)
            .saturating_mul(2048)
            .max(4);
        self.echo_offset = self.echo_offset.wrapping_add(4);
        if usize::from(self.echo_offset) >= delay_bytes {
            self.echo_offset = 0;
        }
        output
    }

    fn tick_sample(&mut self, ram: &mut [u8]) {
        self.apply_keys(ram);
        self.counter_tick();
        let noise = self.noise_sample();
        let mut main = [0i32; 2];
        let mut echo_send = [0i32; 2];
        let mut previous_output = 0i16;
        for index in 0..VOICES {
            let output = self.voice_sample(index, ram, noise, previous_output);
            previous_output = output;
            let base = index << 4;
            let left = (i32::from(output) * Self::signed_volume(self.regs[base])) >> 7;
            let right = (i32::from(output) * Self::signed_volume(self.regs[base + 1])) >> 7;
            main[0] = i32::from(Self::clamp_i16(main[0] + left));
            main[1] = i32::from(Self::clamp_i16(main[1] + right));
            if self.regs[0x4d] & (1 << index) != 0 {
                echo_send[0] = i32::from(Self::clamp_i16(echo_send[0] + left));
                echo_send[1] = i32::from(Self::clamp_i16(echo_send[1] + right));
            }
        }

        let mut output = self.process_echo(ram, main, echo_send);
        if self.regs[0x6c] & 0x40 != 0 {
            output = [0, 0];
        }
        self.samples.push(f32::from(output[0]) / 32768.0);
        self.samples.push(f32::from(output[1]) / 32768.0);
    }

    pub(crate) fn tick(&mut self, ram: &mut [u8], cycles: u32) {
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
        out.u16(self.counter);
        out.u16(self.noise);
        for channel in &self.echo_history {
            for sample in channel {
                out.u16(*sample as u16);
            }
        }
        out.u8(self.echo_history_pos);
        out.u16(self.echo_offset);
        for voice in &self.voices {
            out.u8(voice.active as u8);
            out.u8(voice.release as u8);
            out.u16(voice.block_addr);
            out.u16(voice.loop_addr);
            for sample in voice.decoded {
                out.u16(sample as u16);
            }
            for sample in voice.previous_samples {
                out.u16(sample as u16);
            }
            for sample in voice.lookahead {
                out.u16(sample as u16);
            }
            out.u8(voice.sample_index);
            out.u32(voice.pitch_phase);
            out.u32(voice.history1 as u32);
            out.u32(voice.history2 as u32);
            out.u16(voice.envelope);
            out.u32(voice.envelope_latch as u32);
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
        self.counter = input.u16()? % COUNTER_RANGE;
        self.noise = input.u16()? & 0x7fff;
        for channel in &mut self.echo_history {
            for sample in channel {
                *sample = input.u16()? as i16;
            }
        }
        self.echo_history_pos = input.u8()? & 7;
        self.echo_offset = input.u16()? & !3;
        for voice in &mut self.voices {
            voice.active = input.u8()? != 0;
            voice.release = input.u8()? != 0;
            voice.block_addr = input.u16()?;
            voice.loop_addr = input.u16()?;
            for sample in &mut voice.decoded {
                *sample = input.u16()? as i16;
            }
            for sample in &mut voice.previous_samples {
                *sample = input.u16()? as i16;
            }
            for sample in &mut voice.lookahead {
                *sample = input.u16()? as i16;
            }
            voice.sample_index = input.u8()?.min(16);
            voice.pitch_phase = input.u32()? & 0x0fff;
            voice.history1 = input.u32()? as i32;
            voice.history2 = input.u32()? as i32;
            voice.envelope = input.u16()?.min(ENV_MAX);
            voice.envelope_latch = input.u32()? as i32;
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
        dsp.tick(&mut ram, 32 * 64);
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
        dsp.tick(&mut ram, 32 * 8);
        dsp.write(0x5c, 1);
        dsp.tick(&mut ram, 32 * 300);
        assert_eq!(dsp.regs[0x08], 0);
    }
    #[test]
    fn gaussian_interpolation_blends_fractional_sample_position() {
        let mut voice = Voice {
            active: true,
            sample_index: 0,
            ..Voice::default()
        };
        voice.previous_samples[2] = 0;
        voice.decoded[0] = 0;
        voice.decoded[1] = 20_000;
        voice.decoded[2] = 20_000;

        voice.pitch_phase = 0;
        let start = SnesDsp::gaussian_interpolated_sample(&voice);
        voice.pitch_phase = 0x0800;
        let midpoint = SnesDsp::gaussian_interpolated_sample(&voice);
        voice.pitch_phase = 0x0ff0;
        let end = SnesDsp::gaussian_interpolated_sample(&voice);

        assert!(start < midpoint);
        assert!(midpoint < end);
        assert!(end < 20_000);
    }

    #[test]
    fn pitch_modulation_uses_previous_voice_output() {
        let ram = vec![0; 65_536];
        let mut dsp = SnesDsp::default();
        dsp.voices[1].active = true;
        dsp.voices[1].sample_index = 0;
        dsp.voices[1].decoded.fill(12_000);
        dsp.voices[1].previous_samples.fill(12_000);
        dsp.voices[1].lookahead.fill(12_000);
        dsp.voices[1].envelope = ENV_MAX;
        dsp.regs[0x12] = 0x00;
        dsp.regs[0x13] = 0x10;
        dsp.regs[0x17] = 0x7f;
        dsp.regs[0x2d] = 0x02;

        let _ = dsp.voice_sample(1, &ram, 0, 16_384);

        assert_eq!(dsp.voices[1].sample_index, 1);
        assert_eq!(dsp.voices[1].pitch_phase, 0x0800);
    }

    #[test]
    fn echo_fir_reads_and_feedback_writes_apuram() {
        let mut ram = vec![0; 65_536];
        let mut dsp = SnesDsp::default();
        dsp.regs[0x2c] = 0x7f;
        dsp.regs[0x3c] = 0x7f;
        dsp.regs[0x7f] = 0x7f;
        dsp.regs[0x6d] = 0x20;
        dsp.regs[0x7d] = 1;
        dsp.regs[0x6c] = 0x20;
        SnesDsp::write_ram_i16(&mut ram, 0x2000, 0x4000);
        SnesDsp::write_ram_i16(&mut ram, 0x2002, -0x4000);

        dsp.tick_sample(&mut ram);
        let output = dsp.take_samples();
        assert!(output[0] > 0.45);
        assert!(output[1] < -0.45);
        assert_eq!(SnesDsp::read_ram_i16(&ram, 0x2000), 0x4000);

        dsp.regs[0x6c] = 0;
        dsp.regs[0x7f] = 0;
        dsp.echo_offset = 0;
        let _ = dsp.process_echo(&mut ram, [0, 0], [1000, -1000]);
        assert_eq!(SnesDsp::read_ram_i16(&ram, 0x2000), 1000);
        assert_eq!(SnesDsp::read_ram_i16(&ram, 0x2002), -1000);
    }

    #[test]
    fn hardware_counter_rate_zero_never_polls_and_rate_31_polls_every_sample() {
        let mut dsp = SnesDsp::default();
        for _ in 0..(COUNTER_RANGE * 2) {
            dsp.counter_tick();
            assert!(!dsp.counter_poll(0));
            assert!(dsp.counter_poll(31));
        }
    }

    #[test]
    fn adsr_attack_uses_hardware_rate_and_transitions_to_decay() {
        let mut dsp = SnesDsp::default();
        dsp.voices[0].active = true;
        dsp.regs[0x05] = 0x8f;
        dsp.regs[0x06] = 0xe0;

        dsp.counter_tick();
        dsp.envelope_step(0);
        assert_eq!(dsp.voices[0].envelope, 0x400);
        assert_eq!(dsp.voices[0].env_stage, 0);

        dsp.counter_tick();
        dsp.envelope_step(0);
        assert_eq!(dsp.voices[0].envelope, ENV_MAX);
        assert_eq!(dsp.voices[0].env_stage, 1);
    }

    #[test]
    fn gain_direct_and_noise_rate_follow_hardware_counter() {
        let mut dsp = SnesDsp::default();
        dsp.voices[0].active = true;
        dsp.regs[0x05] = 0x00;
        dsp.regs[0x07] = 0x55;

        dsp.counter_tick();
        dsp.envelope_step(0);
        assert_eq!(dsp.voices[0].envelope, 0x550);

        dsp.regs[0x6c] = 0x00;
        let held = dsp.noise;
        for _ in 0..64 {
            dsp.counter_tick();
            let _ = dsp.noise_sample();
        }
        assert_eq!(dsp.noise, held);

        dsp.regs[0x6c] = 0x1f;
        dsp.counter_tick();
        let _ = dsp.noise_sample();
        assert_ne!(dsp.noise, held);
    }

    #[test]
    fn state_round_trip_preserves_voice_and_registers() {
        let mut ram = vec![0; 65_536];
        let mut dsp = SnesDsp::default();
        configure_test_voice(&mut dsp, &mut ram);
        dsp.tick(&mut ram, 32 * 4);
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
