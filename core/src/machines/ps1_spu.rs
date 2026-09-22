use std::collections::VecDeque;

use crate::state::{StateReader, StateWriter};

const RAM_BYTES: usize = 512 * 1024;
const VOICE_COUNT: usize = 24;
const CPU_HZ: u64 = 33_868_800;
const SAMPLE_HZ: u64 = 44_100;
const ADDRESS_MASK: u32 = (RAM_BYTES as u32) - 1;
const MAX_QUEUED_FRAMES: usize = 4096;
const MAX_CD_INPUT_FRAMES: usize = 9_408;
const FILTERS: [(i32, i32); 5] = [(0, 0), (60, 0), (115, -52), (98, -55), (122, -60)];
const GAUSS: [i32; 512] = [
    -0x001, -0x001, -0x001, -0x001, -0x001, -0x001, -0x001, -0x001, -0x001, -0x001, -0x001, -0x001,
    -0x001, -0x001, -0x001, -0x001, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0001,
    0x0001, 0x0001, 0x0001, 0x0002, 0x0002, 0x0002, 0x0003, 0x0003, 0x0003, 0x0004, 0x0004, 0x0005,
    0x0005, 0x0006, 0x0007, 0x0007, 0x0008, 0x0009, 0x0009, 0x000a, 0x000b, 0x000c, 0x000d, 0x000e,
    0x000f, 0x0010, 0x0011, 0x0012, 0x0013, 0x0015, 0x0016, 0x0018, 0x0019, 0x001b, 0x001c, 0x001e,
    0x0020, 0x0021, 0x0023, 0x0025, 0x0027, 0x0029, 0x002c, 0x002e, 0x0030, 0x0033, 0x0035, 0x0038,
    0x003a, 0x003d, 0x0040, 0x0043, 0x0046, 0x0049, 0x004d, 0x0050, 0x0054, 0x0057, 0x005b, 0x005f,
    0x0063, 0x0067, 0x006b, 0x006f, 0x0074, 0x0078, 0x007d, 0x0082, 0x0087, 0x008c, 0x0091, 0x0096,
    0x009c, 0x00a1, 0x00a7, 0x00ad, 0x00b3, 0x00ba, 0x00c0, 0x00c7, 0x00cd, 0x00d4, 0x00db, 0x00e3,
    0x00ea, 0x00f2, 0x00fa, 0x0101, 0x010a, 0x0112, 0x011b, 0x0123, 0x012c, 0x0135, 0x013f, 0x0148,
    0x0152, 0x015c, 0x0166, 0x0171, 0x017b, 0x0186, 0x0191, 0x019c, 0x01a8, 0x01b4, 0x01c0, 0x01cc,
    0x01d9, 0x01e5, 0x01f2, 0x0200, 0x020d, 0x021b, 0x0229, 0x0237, 0x0246, 0x0255, 0x0264, 0x0273,
    0x0283, 0x0293, 0x02a3, 0x02b4, 0x02c4, 0x02d6, 0x02e7, 0x02f9, 0x030b, 0x031d, 0x0330, 0x0343,
    0x0356, 0x036a, 0x037e, 0x0392, 0x03a7, 0x03bc, 0x03d1, 0x03e7, 0x03fc, 0x0413, 0x042a, 0x0441,
    0x0458, 0x0470, 0x0488, 0x04a0, 0x04b9, 0x04d2, 0x04ec, 0x0506, 0x0520, 0x053b, 0x0556, 0x0572,
    0x058e, 0x05aa, 0x05c7, 0x05e4, 0x0601, 0x061f, 0x063e, 0x065c, 0x067c, 0x069b, 0x06bb, 0x06dc,
    0x06fd, 0x071e, 0x0740, 0x0762, 0x0784, 0x07a7, 0x07cb, 0x07ef, 0x0813, 0x0838, 0x085d, 0x0883,
    0x08a9, 0x08d0, 0x08f7, 0x091e, 0x0946, 0x096f, 0x0998, 0x09c1, 0x09eb, 0x0a16, 0x0a40, 0x0a6c,
    0x0a98, 0x0ac4, 0x0af1, 0x0b1e, 0x0b4c, 0x0b7a, 0x0ba9, 0x0bd8, 0x0c07, 0x0c38, 0x0c68, 0x0c99,
    0x0ccb, 0x0cfd, 0x0d30, 0x0d63, 0x0d97, 0x0dcb, 0x0e00, 0x0e35, 0x0e6b, 0x0ea1, 0x0ed7, 0x0f0f,
    0x0f46, 0x0f7f, 0x0fb7, 0x0ff1, 0x102a, 0x1065, 0x109f, 0x10db, 0x1116, 0x1153, 0x118f, 0x11cd,
    0x120b, 0x1249, 0x1288, 0x12c7, 0x1307, 0x1347, 0x1388, 0x13c9, 0x140b, 0x144d, 0x1490, 0x14d4,
    0x1517, 0x155c, 0x15a0, 0x15e6, 0x162c, 0x1672, 0x16b9, 0x1700, 0x1747, 0x1790, 0x17d8, 0x1821,
    0x186b, 0x18b5, 0x1900, 0x194b, 0x1996, 0x19e2, 0x1a2e, 0x1a7b, 0x1ac8, 0x1b16, 0x1b64, 0x1bb3,
    0x1c02, 0x1c51, 0x1ca1, 0x1cf1, 0x1d42, 0x1d93, 0x1de5, 0x1e37, 0x1e89, 0x1edc, 0x1f2f, 0x1f82,
    0x1fd6, 0x202a, 0x207f, 0x20d4, 0x2129, 0x217f, 0x21d5, 0x222c, 0x2282, 0x22da, 0x2331, 0x2389,
    0x23e1, 0x2439, 0x2492, 0x24eb, 0x2545, 0x259e, 0x25f8, 0x2653, 0x26ad, 0x2708, 0x2763, 0x27be,
    0x281a, 0x2876, 0x28d2, 0x292e, 0x298b, 0x29e7, 0x2a44, 0x2aa1, 0x2aff, 0x2b5c, 0x2bba, 0x2c18,
    0x2c76, 0x2cd4, 0x2d33, 0x2d91, 0x2df0, 0x2e4f, 0x2eae, 0x2f0d, 0x2f6c, 0x2fcc, 0x302b, 0x308b,
    0x30ea, 0x314a, 0x31aa, 0x3209, 0x3269, 0x32c9, 0x3329, 0x3389, 0x33e9, 0x3449, 0x34a9, 0x3509,
    0x3569, 0x35c9, 0x3629, 0x3689, 0x36e8, 0x3748, 0x37a8, 0x3807, 0x3867, 0x38c6, 0x3926, 0x3985,
    0x39e4, 0x3a43, 0x3aa2, 0x3b00, 0x3b5f, 0x3bbd, 0x3c1b, 0x3c79, 0x3cd7, 0x3d35, 0x3d92, 0x3def,
    0x3e4c, 0x3ea9, 0x3f05, 0x3f62, 0x3fbd, 0x4019, 0x4074, 0x40d0, 0x412a, 0x4185, 0x41df, 0x4239,
    0x4292, 0x42eb, 0x4344, 0x439c, 0x43f4, 0x444c, 0x44a3, 0x44fa, 0x4550, 0x45a6, 0x45fc, 0x4651,
    0x46a6, 0x46fa, 0x474e, 0x47a1, 0x47f4, 0x4846, 0x4898, 0x48e9, 0x493a, 0x498a, 0x49d9, 0x4a29,
    0x4a77, 0x4ac5, 0x4b13, 0x4b5f, 0x4bac, 0x4bf7, 0x4c42, 0x4c8d, 0x4cd7, 0x4d20, 0x4d68, 0x4db0,
    0x4df7, 0x4e3e, 0x4e84, 0x4ec9, 0x4f0e, 0x4f52, 0x4f95, 0x4fd7, 0x5019, 0x505a, 0x509a, 0x50da,
    0x5118, 0x5156, 0x5194, 0x51d0, 0x520c, 0x5247, 0x5281, 0x52ba, 0x52f3, 0x532a, 0x5361, 0x5397,
    0x53cc, 0x5401, 0x5434, 0x5467, 0x5499, 0x54ca, 0x54fa, 0x5529, 0x5558, 0x5585, 0x55b2, 0x55de,
    0x5609, 0x5632, 0x565b, 0x5684, 0x56ab, 0x56d1, 0x56f6, 0x571b, 0x573e, 0x5761, 0x5782, 0x57a3,
    0x57c3, 0x57e2, 0x57ff, 0x581c, 0x5838, 0x5853, 0x586d, 0x5886, 0x589e, 0x58b5, 0x58cb, 0x58e0,
    0x58f4, 0x5907, 0x5919, 0x592a, 0x593a, 0x5949, 0x5958, 0x5965, 0x5971, 0x597c, 0x5986, 0x598f,
    0x5997, 0x599e, 0x59a4, 0x59a9, 0x59ad, 0x59b0, 0x59b2, 0x59b3,
];

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum EnvelopePhase {
    #[default]
    Off,
    Attack,
    Decay,
    Sustain,
    Release,
}

impl EnvelopePhase {
    fn encode(self) -> u8 {
        match self {
            Self::Off => 0,
            Self::Attack => 1,
            Self::Decay => 2,
            Self::Sustain => 3,
            Self::Release => 4,
        }
    }

    fn decode(value: u8) -> Result<Self, String> {
        match value {
            0 => Ok(Self::Off),
            1 => Ok(Self::Attack),
            2 => Ok(Self::Decay),
            3 => Ok(Self::Sustain),
            4 => Ok(Self::Release),
            _ => Err("invalid PlayStation SPU envelope phase".into()),
        }
    }
}

#[derive(Clone, Copy)]
struct Voice {
    volume_left: u16,
    volume_right: u16,
    volume_left_level: i32,
    volume_right_level: i32,
    volume_left_counter: u32,
    volume_right_counter: u32,
    pitch: u16,
    start_address: u16,
    adsr_low: u16,
    adsr_high: u16,
    current_volume: u16,
    repeat_address: u16,
    current_address: u32,
    pitch_counter: u32,
    decoded: [i16; 28],
    decoded_index: u8,
    interpolation: [i16; 4],
    interpolation_ready: bool,
    block_flags: u8,
    block_loaded: bool,
    previous_1: i32,
    previous_2: i32,
    envelope_level: i32,
    envelope_counter: u32,
    phase: EnvelopePhase,
    active: bool,
}

impl Default for Voice {
    fn default() -> Self {
        Self {
            volume_left: 0,
            volume_right: 0,
            volume_left_level: 0,
            volume_right_level: 0,
            volume_left_counter: 0,
            volume_right_counter: 0,
            pitch: 0,
            start_address: 0,
            adsr_low: 0,
            adsr_high: 0,
            current_volume: 0,
            repeat_address: 0,
            current_address: 0,
            pitch_counter: 0,
            decoded: [0; 28],
            decoded_index: 0,
            interpolation: [0; 4],
            interpolation_ready: false,
            block_flags: 0,
            block_loaded: false,
            previous_1: 0,
            previous_2: 0,
            envelope_level: 0,
            envelope_counter: 0,
            phase: EnvelopePhase::Off,
            active: false,
        }
    }
}

impl Voice {
    fn key_on(&mut self) {
        self.current_address = (u32::from(self.start_address) * 8) & ADDRESS_MASK;
        self.pitch_counter = 0;
        self.decoded_index = 0;
        self.interpolation = [0; 4];
        self.interpolation_ready = false;
        self.block_loaded = false;
        self.previous_1 = 0;
        self.previous_2 = 0;
        self.envelope_level = 0;
        self.envelope_counter = 0;
        self.current_volume = 0;
        self.phase = EnvelopePhase::Attack;
        self.active = true;
    }

    fn key_off(&mut self) {
        if self.active {
            self.phase = EnvelopePhase::Release;
            self.envelope_counter = 0;
        }
    }

    fn save(&self, out: &mut StateWriter) {
        out.u16(self.volume_left);
        out.u16(self.volume_right);
        out.u32(self.volume_left_level as u32);
        out.u32(self.volume_right_level as u32);
        out.u32(self.volume_left_counter);
        out.u32(self.volume_right_counter);
        out.u16(self.pitch);
        out.u16(self.start_address);
        out.u16(self.adsr_low);
        out.u16(self.adsr_high);
        out.u16(self.current_volume);
        out.u16(self.repeat_address);
        out.u32(self.current_address);
        out.u32(self.pitch_counter);
        for sample in self.decoded {
            out.u16(sample as u16);
        }
        out.u8(self.decoded_index);
        for sample in self.interpolation {
            out.u16(sample as u16);
        }
        out.u8(self.interpolation_ready as u8);
        out.u8(self.block_flags);
        out.u8(self.block_loaded as u8);
        out.u32(self.previous_1 as u32);
        out.u32(self.previous_2 as u32);
        out.u32(self.envelope_level as u32);
        out.u32(self.envelope_counter);
        out.u8(self.phase.encode());
        out.u8(self.active as u8);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.volume_left = input.u16()?;
        self.volume_right = input.u16()?;
        self.volume_left_level = input.u32()? as i32;
        self.volume_right_level = input.u32()? as i32;
        self.volume_left_counter = input.u32()? & 0x7fff;
        self.volume_right_counter = input.u32()? & 0x7fff;
        self.pitch = input.u16()?;
        self.start_address = input.u16()?;
        self.adsr_low = input.u16()?;
        self.adsr_high = input.u16()?;
        self.current_volume = input.u16()?;
        self.repeat_address = input.u16()?;
        self.current_address = input.u32()? & ADDRESS_MASK;
        self.pitch_counter = input.u32()? & 0xffff;
        for sample in &mut self.decoded {
            *sample = input.u16()? as i16;
        }
        self.decoded_index = input.u8()?;
        if self.decoded_index > 28 {
            return Err("invalid PlayStation SPU decoded-sample index".into());
        }
        for sample in &mut self.interpolation {
            *sample = input.u16()? as i16;
        }
        self.interpolation_ready = input.u8()? != 0;
        self.block_flags = input.u8()?;
        self.block_loaded = input.u8()? != 0;
        self.previous_1 = input.u32()? as i32;
        self.previous_2 = input.u32()? as i32;
        self.envelope_level = (input.u32()? as i32).clamp(-0x8000, 0x7fff);
        self.envelope_counter = input.u32()? & 0x7fff;
        self.phase = EnvelopePhase::decode(input.u8()?)?;
        self.active = input.u8()? != 0;
        Ok(())
    }
}

pub struct Ps1Spu {
    ram: Vec<u8>,
    voices: [Voice; VOICE_COUNT],
    main_volume_left: u16,
    main_volume_right: u16,
    main_volume_left_level: i32,
    main_volume_right_level: i32,
    main_volume_left_counter: u32,
    main_volume_right_counter: u32,
    reverb_volume_left: u16,
    reverb_volume_right: u16,
    reverb_registers: [u16; 32],
    reverb_cursor: u32,
    reverb_phase: bool,
    reverb_output_left: i32,
    reverb_output_right: i32,
    pitch_modulation: u32,
    noise_enable: u32,
    reverb_enable: u32,
    end_flags: u32,
    reverb_base: u16,
    irq_address: u16,
    transfer_address: u16,
    transfer_cursor: u32,
    control: u16,
    transfer_control: u16,
    status: u16,
    cd_volume_left: u16,
    cd_volume_right: u16,
    cd_audio: VecDeque<[i16; 2]>,
    external_volume_left: u16,
    external_volume_right: u16,
    sample_phase: u64,
    noise_level: u16,
    noise_timer: i32,
    irq_pending: bool,
    output: Vec<f32>,
}

impl Default for Ps1Spu {
    fn default() -> Self {
        Self::new()
    }
}

impl Ps1Spu {
    pub fn new() -> Self {
        Self {
            ram: vec![0; RAM_BYTES],
            voices: [Voice::default(); VOICE_COUNT],
            main_volume_left: 0,
            main_volume_right: 0,
            main_volume_left_level: 0,
            main_volume_right_level: 0,
            main_volume_left_counter: 0,
            main_volume_right_counter: 0,
            reverb_volume_left: 0,
            reverb_volume_right: 0,
            reverb_registers: [0; 32],
            reverb_cursor: 0,
            reverb_phase: false,
            reverb_output_left: 0,
            reverb_output_right: 0,
            pitch_modulation: 0,
            noise_enable: 0,
            reverb_enable: 0,
            end_flags: 0,
            reverb_base: 0,
            irq_address: 0,
            transfer_address: 0,
            transfer_cursor: 0,
            control: 0,
            transfer_control: 0,
            status: 0,
            cd_volume_left: 0,
            cd_volume_right: 0,
            cd_audio: VecDeque::with_capacity(1176),
            external_volume_left: 0,
            external_volume_right: 0,
            sample_phase: 0,
            noise_level: 1,
            noise_timer: 0x10000,
            irq_pending: false,
            output: Vec::with_capacity(2048),
        }
    }

    pub fn reset(&mut self) {
        self.voices = [Voice::default(); VOICE_COUNT];
        self.main_volume_left = 0;
        self.main_volume_right = 0;
        self.main_volume_left_level = 0;
        self.main_volume_right_level = 0;
        self.main_volume_left_counter = 0;
        self.main_volume_right_counter = 0;
        self.reverb_volume_left = 0;
        self.reverb_volume_right = 0;
        self.reverb_registers = [0; 32];
        self.reverb_cursor = 0;
        self.reverb_phase = false;
        self.reverb_output_left = 0;
        self.reverb_output_right = 0;
        self.pitch_modulation = 0;
        self.noise_enable = 0;
        self.reverb_enable = 0;
        self.end_flags = 0;
        self.reverb_base = 0;
        self.irq_address = 0;
        self.transfer_address = 0;
        self.transfer_cursor = 0;
        self.control = 0;
        self.transfer_control = 0;
        self.status = 0;
        self.cd_volume_left = 0;
        self.cd_volume_right = 0;
        self.cd_audio.clear();
        self.external_volume_left = 0;
        self.external_volume_right = 0;
        self.sample_phase = 0;
        self.noise_level = 1;
        self.noise_timer = 0x10000;
        self.irq_pending = false;
        self.output.clear();
    }

    pub fn read8(&self, address: u32) -> u8 {
        let aligned = address & !1;
        let value = self.read16(aligned);
        if address & 1 == 0 {
            value as u8
        } else {
            (value >> 8) as u8
        }
    }

    pub fn write8(&mut self, address: u32, value: u8) {
        if address & 1 == 0 {
            self.write16(address, u16::from(value));
        }
    }

    pub fn read16(&self, address: u32) -> u16 {
        let relative = address.wrapping_sub(0x1f80_1c00);
        if relative < 0x180 {
            let voice = &self.voices[(relative / 0x10) as usize];
            return match relative & 0x0f {
                0x0 => voice.volume_left,
                0x2 => voice.volume_right,
                0x4 => voice.pitch,
                0x6 => voice.start_address,
                0x8 => voice.adsr_low,
                0xa => voice.adsr_high,
                0xc => voice.current_volume,
                0xe => voice.repeat_address,
                _ => 0,
            };
        }
        if (0x1c0..0x200).contains(&relative) {
            return self.reverb_registers[((relative - 0x1c0) / 2) as usize];
        }
        if (0x200..0x260).contains(&relative) {
            let voice = &self.voices[((relative - 0x200) / 4) as usize];
            return if relative & 2 == 0 {
                voice.volume_left_level as i16 as u16
            } else {
                voice.volume_right_level as i16 as u16
            };
        }
        match relative {
            0x180 => self.main_volume_left,
            0x182 => self.main_volume_right,
            0x184 => self.reverb_volume_left,
            0x186 => self.reverb_volume_right,
            0x190 => self.pitch_modulation as u16,
            0x192 => (self.pitch_modulation >> 16) as u16,
            0x194 => self.noise_enable as u16,
            0x196 => (self.noise_enable >> 16) as u16,
            0x198 => self.reverb_enable as u16,
            0x19a => (self.reverb_enable >> 16) as u16,
            0x19c => self.end_flags as u16,
            0x19e => (self.end_flags >> 16) as u16,
            0x1a2 => self.reverb_base,
            0x1a4 => self.irq_address,
            0x1a6 => self.transfer_address,
            0x1aa => self.control,
            0x1ac => self.transfer_control,
            0x1ae => self.status,
            0x1b0 => self.cd_volume_left,
            0x1b2 => self.cd_volume_right,
            0x1b4 => self.external_volume_left,
            0x1b6 => self.external_volume_right,
            0x1b8 => self.main_volume_left_level as i16 as u16,
            0x1ba => self.main_volume_right_level as i16 as u16,
            _ => 0,
        }
    }

    pub fn write16(&mut self, address: u32, value: u16) {
        let relative = address.wrapping_sub(0x1f80_1c00);
        if relative < 0x180 {
            let voice = &mut self.voices[(relative / 0x10) as usize];
            match relative & 0x0f {
                0x0 => Self::write_volume(
                    &mut voice.volume_left,
                    &mut voice.volume_left_level,
                    &mut voice.volume_left_counter,
                    value,
                ),
                0x2 => Self::write_volume(
                    &mut voice.volume_right,
                    &mut voice.volume_right_level,
                    &mut voice.volume_right_counter,
                    value,
                ),
                0x4 => voice.pitch = value,
                0x6 => voice.start_address = value,
                0x8 => voice.adsr_low = value,
                0xa => voice.adsr_high = value,
                0xc => {
                    voice.current_volume = value;
                    voice.envelope_level = i32::from(value as i16).clamp(0, 0x7fff);
                }
                0xe => voice.repeat_address = value,
                _ => {}
            }
            return;
        }
        if (0x1c0..0x200).contains(&relative) {
            self.reverb_registers[((relative - 0x1c0) / 2) as usize] = value;
            return;
        }
        match relative {
            0x180 => Self::write_volume(
                &mut self.main_volume_left,
                &mut self.main_volume_left_level,
                &mut self.main_volume_left_counter,
                value,
            ),
            0x182 => Self::write_volume(
                &mut self.main_volume_right,
                &mut self.main_volume_right_level,
                &mut self.main_volume_right_counter,
                value,
            ),
            0x184 => self.reverb_volume_left = value,
            0x186 => self.reverb_volume_right = value,
            0x188 => self.key_on_mask(u32::from(value)),
            0x18a => self.key_on_mask(u32::from(value) << 16),
            0x18c => self.key_off_mask(u32::from(value)),
            0x18e => self.key_off_mask(u32::from(value) << 16),
            0x190 => {
                self.pitch_modulation = (self.pitch_modulation & 0xffff_0000) | u32::from(value)
            }
            0x192 => {
                self.pitch_modulation =
                    (self.pitch_modulation & 0x0000_ffff) | (u32::from(value) << 16)
            }
            0x194 => self.noise_enable = (self.noise_enable & 0xffff_0000) | u32::from(value),
            0x196 => {
                self.noise_enable = (self.noise_enable & 0x0000_ffff) | (u32::from(value) << 16)
            }
            0x198 => self.reverb_enable = (self.reverb_enable & 0xffff_0000) | u32::from(value),
            0x19a => {
                self.reverb_enable = (self.reverb_enable & 0x0000_ffff) | (u32::from(value) << 16)
            }
            0x1a2 => {
                self.reverb_base = value;
                self.reverb_cursor = (u32::from(value) * 8) & ADDRESS_MASK;
            }
            0x1a4 => self.irq_address = value,
            0x1a6 => {
                self.transfer_address = value;
                self.transfer_cursor = (u32::from(value) * 8) & ADDRESS_MASK;
                self.check_irq_address(self.transfer_cursor);
            }
            0x1a8 => self.transfer_write_halfword(value),
            0x1aa => self.write_control(value),
            0x1ac => self.transfer_control = value,
            0x1b0 => self.cd_volume_left = value,
            0x1b2 => self.cd_volume_right = value,
            0x1b4 => self.external_volume_left = value,
            0x1b6 => self.external_volume_right = value,
            _ => {}
        }
    }

    fn write_control(&mut self, value: u16) {
        self.control = value;
        self.status = (self.status & !0x003f) | (value & 0x003f);
        if value & (1 << 6) == 0 {
            self.irq_pending = false;
            self.status &= !(1 << 6);
        }
        let mode = (value >> 4) & 3;
        self.status &= !((1 << 7) | (1 << 8) | (1 << 9) | (1 << 10));
        match mode {
            1 => self.status |= 1 << 7,
            2 => self.status |= (1 << 7) | (1 << 8),
            3 => self.status |= (1 << 7) | (1 << 9),
            _ => {}
        }
    }

    fn key_on_mask(&mut self, mask: u32) {
        for index in 0..VOICE_COUNT {
            if mask & (1 << index) != 0 {
                self.voices[index].key_on();
                self.end_flags &= !(1 << index);
            }
        }
    }

    fn key_off_mask(&mut self, mask: u32) {
        for index in 0..VOICE_COUNT {
            if mask & (1 << index) != 0 {
                self.voices[index].key_off();
            }
        }
    }

    fn check_irq_address(&mut self, address: u32) {
        if self.control & (1 << 15) != 0
            && self.control & (1 << 6) != 0
            && address & ADDRESS_MASK == ((u32::from(self.irq_address) * 8) & ADDRESS_MASK)
        {
            self.irq_pending = true;
            self.status |= 1 << 6;
        }
    }

    pub fn irq_pending(&self) -> bool {
        self.irq_pending
    }

    pub fn transfer_write_halfword(&mut self, value: u16) {
        let address = self.transfer_cursor & ADDRESS_MASK;
        let [lo, hi] = value.to_le_bytes();
        self.ram[address as usize] = lo;
        self.ram[((address + 1) & ADDRESS_MASK) as usize] = hi;
        self.check_irq_address(address);
        self.transfer_cursor = (address + 2) & ADDRESS_MASK;
    }

    pub fn transfer_read_halfword(&mut self) -> u16 {
        let address = self.transfer_cursor & ADDRESS_MASK;
        let value = u16::from_le_bytes([
            self.ram[address as usize],
            self.ram[((address + 1) & ADDRESS_MASK) as usize],
        ]);
        self.check_irq_address(address);
        self.transfer_cursor = (address + 2) & ADDRESS_MASK;
        value
    }

    pub fn queue_cd_audio(&mut self, samples: impl IntoIterator<Item = [i16; 2]>) {
        for frame in samples {
            if self.cd_audio.len() >= MAX_CD_INPUT_FRAMES {
                self.cd_audio.pop_front();
            }
            self.cd_audio.push_back(frame);
        }
    }

    pub fn tick_cpu_cycles(&mut self, cycles: u32) {
        self.sample_phase = self
            .sample_phase
            .saturating_add(u64::from(cycles) * SAMPLE_HZ);
        while self.sample_phase >= CPU_HZ {
            self.sample_phase -= CPU_HZ;
            self.mix_sample();
        }
    }

    fn mix_sample(&mut self) {
        self.update_noise();
        let mut left = 0i64;
        let mut right = 0i64;
        let mut reverb_left = 0i64;
        let mut reverb_right = 0i64;
        let mut previous_voice_output = 0i32;
        for index in 0..VOICE_COUNT {
            let mut voice = self.voices[index];
            Self::tick_volume(
                voice.volume_left,
                &mut voice.volume_left_level,
                &mut voice.volume_left_counter,
            );
            Self::tick_volume(
                voice.volume_right,
                &mut voice.volume_right_level,
                &mut voice.volume_right_counter,
            );
            if !voice.active {
                self.voices[index] = voice;
                previous_voice_output = 0;
                continue;
            }
            let raw = if self.noise_enable & (1 << index) != 0 {
                self.noise_level as i16 as i32
            } else {
                let step = Self::pitch_step(
                    voice.pitch,
                    index,
                    self.pitch_modulation,
                    previous_voice_output,
                );
                self.sample_voice(index, &mut voice, step)
            };
            Self::tick_envelope(&mut voice);
            let enveloped = (i64::from(raw) * i64::from(voice.envelope_level) / 0x7fff) as i32;
            previous_voice_output = enveloped.clamp(i32::from(i16::MIN), i32::from(i16::MAX));
            let voice_left =
                i64::from(previous_voice_output) * i64::from(voice.volume_left_level) / 0x7fff;
            let voice_right =
                i64::from(previous_voice_output) * i64::from(voice.volume_right_level) / 0x7fff;
            left += voice_left;
            right += voice_right;
            if self.reverb_enable & (1 << index) != 0 {
                reverb_left += voice_left;
                reverb_right += voice_right;
            }
            self.voices[index] = voice;
        }

        let cd = self.cd_audio.pop_front().unwrap_or([0, 0]);
        if self.control & 1 != 0 {
            let cd_left = Self::signed_volume(i32::from(cd[0]), self.cd_volume_left);
            let cd_right = Self::signed_volume(i32::from(cd[1]), self.cd_volume_right);
            left += i64::from(cd_left);
            right += i64::from(cd_right);
            if self.control & (1 << 2) != 0 {
                reverb_left += i64::from(cd_left);
                reverb_right += i64::from(cd_right);
            }
        }

        self.reverb_phase = !self.reverb_phase;
        if self.reverb_phase {
            self.process_reverb(
                reverb_left.clamp(i64::from(i16::MIN), i64::from(i16::MAX)) as i32,
                reverb_right.clamp(i64::from(i16::MIN), i64::from(i16::MAX)) as i32,
            );
        }
        left += i64::from(self.reverb_output_left);
        right += i64::from(self.reverb_output_right);

        Self::tick_volume(
            self.main_volume_left,
            &mut self.main_volume_left_level,
            &mut self.main_volume_left_counter,
        );
        Self::tick_volume(
            self.main_volume_right,
            &mut self.main_volume_right_level,
            &mut self.main_volume_right_counter,
        );
        if self.control & (1 << 15) == 0 || self.control & (1 << 14) == 0 {
            left = 0;
            right = 0;
        }
        left = left * i64::from(self.main_volume_left_level) / 0x7fff;
        right = right * i64::from(self.main_volume_right_level) / 0x7fff;
        let left = left.clamp(i64::from(i16::MIN), i64::from(i16::MAX)) as i16;
        let right = right.clamp(i64::from(i16::MIN), i64::from(i16::MAX)) as i16;
        if self.output.len() >= MAX_QUEUED_FRAMES * 2 {
            self.output.drain(..2);
        }
        self.output.push(f32::from(left) / 32768.0);
        self.output.push(f32::from(right) / 32768.0);
    }

    fn pitch_step(pitch: u16, index: usize, modulation: u32, previous: i32) -> u32 {
        let mut step = i32::from(pitch as i16);
        if index > 0 && modulation & (1 << index) != 0 {
            let factor = previous.clamp(-0x8000, 0x7fff) + 0x8000;
            step = (step * factor) >> 15;
            step &= 0xffff;
        } else {
            step = i32::from(pitch);
        }
        u32::try_from(step).unwrap_or(0).min(0x4000)
    }

    fn next_voice_sample(&mut self, index: usize, voice: &mut Voice) -> Option<i16> {
        if !voice.block_loaded {
            self.decode_block(index, voice);
        }
        if !voice.active || !voice.block_loaded {
            return None;
        }
        let sample = voice.decoded[usize::from(voice.decoded_index.min(27))];
        voice.decoded_index = voice.decoded_index.saturating_add(1);
        if voice.decoded_index >= 28 {
            self.finish_block(index, voice);
            if voice.active {
                self.decode_block(index, voice);
            }
        }
        Some(sample)
    }

    fn push_interpolation_sample(&mut self, index: usize, voice: &mut Voice) -> bool {
        let Some(sample) = self.next_voice_sample(index, voice) else {
            return false;
        };
        voice.interpolation.rotate_left(1);
        voice.interpolation[3] = sample;
        true
    }

    fn gaussian_sample(voice: &Voice) -> i32 {
        let i = ((voice.pitch_counter >> 4) & 0xff) as usize;
        let oldest = i32::from(voice.interpolation[0]);
        let older = i32::from(voice.interpolation[1]);
        let old = i32::from(voice.interpolation[2]);
        let new = i32::from(voice.interpolation[3]);
        ((GAUSS[0x0ff - i] * oldest) >> 15)
            + ((GAUSS[0x1ff - i] * older) >> 15)
            + ((GAUSS[0x100 + i] * old) >> 15)
            + ((GAUSS[i] * new) >> 15)
    }

    fn sample_voice(&mut self, index: usize, voice: &mut Voice, step: u32) -> i32 {
        if !voice.interpolation_ready {
            if !self.push_interpolation_sample(index, voice) {
                return 0;
            }
            voice.interpolation_ready = true;
        }
        let sample = Self::gaussian_sample(voice).clamp(i32::from(i16::MIN), i32::from(i16::MAX));
        voice.pitch_counter = voice.pitch_counter.wrapping_add(step);
        while voice.pitch_counter >= 0x1000 && voice.active {
            voice.pitch_counter -= 0x1000;
            if !self.push_interpolation_sample(index, voice) {
                break;
            }
        }
        sample
    }

    fn decode_block(&mut self, index: usize, voice: &mut Voice) {
        let address = voice.current_address & ADDRESS_MASK;
        self.check_irq_address(address);
        let header = self.ram[address as usize];
        let flags = self.ram[((address + 1) & ADDRESS_MASK) as usize];
        let shift = (header & 0x0f).min(12);
        let filter = usize::from((header >> 4).min(4));
        let (positive, negative) = FILTERS[filter];
        if flags & 4 != 0 {
            voice.repeat_address = (address >> 3) as u16;
        }
        for sample_index in 0..28usize {
            let packed =
                self.ram[((address + 2 + (sample_index / 2) as u32) & ADDRESS_MASK) as usize];
            let nibble = if sample_index & 1 == 0 {
                packed & 0x0f
            } else {
                packed >> 4
            };
            let signed = if nibble & 8 != 0 {
                i32::from(nibble) - 16
            } else {
                i32::from(nibble)
            };
            let source = (signed << 12) >> shift;
            let predicted = (voice.previous_1 * positive + voice.previous_2 * negative + 32) >> 6;
            let decoded = (source + predicted).clamp(i32::from(i16::MIN), i32::from(i16::MAX));
            voice.decoded[sample_index] = decoded as i16;
            voice.previous_2 = voice.previous_1;
            voice.previous_1 = decoded;
        }
        voice.decoded_index = 0;
        voice.block_flags = flags;
        voice.block_loaded = true;
        self.voices[index].repeat_address = voice.repeat_address;
    }

    fn finish_block(&mut self, index: usize, voice: &mut Voice) {
        let flags = voice.block_flags;
        voice.block_loaded = false;
        voice.decoded_index = 0;
        if flags & 1 != 0 {
            self.end_flags |= 1 << index;
            voice.current_address = (u32::from(voice.repeat_address) * 8) & ADDRESS_MASK;
            if flags & 2 == 0 {
                voice.envelope_level = 0;
                voice.current_volume = 0;
                voice.phase = EnvelopePhase::Off;
                voice.active = false;
            }
        } else {
            voice.current_address = (voice.current_address + 16) & ADDRESS_MASK;
        }
    }

    fn tick_envelope(voice: &mut Voice) {
        if !voice.active {
            voice.current_volume = 0;
            return;
        }
        match voice.phase {
            EnvelopePhase::Off => {
                voice.envelope_level = 0;
                voice.active = false;
            }
            EnvelopePhase::Attack => {
                let shift = (voice.adsr_low >> 10) & 0x1f;
                let step = (voice.adsr_low >> 8) & 3;
                let exponential = voice.adsr_low & 0x8000 != 0;
                Self::envelope_rate(voice, shift, step, exponential, false);
                if voice.envelope_level >= 0x7fff {
                    voice.envelope_level = 0x7fff;
                    voice.phase = EnvelopePhase::Decay;
                    voice.envelope_counter = 0;
                }
            }
            EnvelopePhase::Decay => {
                let shift = ((voice.adsr_low >> 4) & 0x0f) * 2;
                Self::envelope_rate(voice, shift, 0, true, true);
                let sustain = (i32::from(voice.adsr_low & 0x0f) + 1) * 0x800;
                if voice.envelope_level <= sustain.min(0x7fff) {
                    voice.phase = EnvelopePhase::Sustain;
                    voice.envelope_counter = 0;
                }
            }
            EnvelopePhase::Sustain => {
                let shift = (voice.adsr_high >> 8) & 0x1f;
                let step = (voice.adsr_high >> 6) & 3;
                let decreasing = voice.adsr_high & 0x4000 != 0;
                let exponential = voice.adsr_high & 0x8000 != 0;
                Self::envelope_rate(voice, shift, step, exponential, decreasing);
            }
            EnvelopePhase::Release => {
                let shift = voice.adsr_high & 0x1f;
                let exponential = voice.adsr_high & 0x20 != 0;
                Self::envelope_rate(voice, shift, 0, exponential, true);
                if voice.envelope_level <= 0 {
                    voice.envelope_level = 0;
                    voice.phase = EnvelopePhase::Off;
                    voice.active = false;
                }
            }
        }
        voice.envelope_level = voice.envelope_level.clamp(0, 0x7fff);
        voice.current_volume = voice.envelope_level as u16;
    }

    fn envelope_rate(
        voice: &mut Voice,
        shift: u16,
        step_value: u16,
        exponential: bool,
        decreasing: bool,
    ) {
        if shift == 31 && step_value == 3 {
            return;
        }
        let mut step = i32::from(7 - step_value);
        if decreasing {
            step = !step;
        }
        if shift < 11 {
            step <<= u32::from(11 - shift);
        }
        let mut increment = if shift > 11 {
            0x8000u32 >> u32::from((shift - 11).min(15))
        } else {
            0x8000
        };
        increment = increment.max(1);
        if exponential && !decreasing && voice.envelope_level > 0x6000 {
            if shift < 10 {
                step >>= 2;
            } else if shift >= 11 {
                increment = (increment >> 2).max(1);
            } else {
                step >>= 1;
            }
        } else if exponential && decreasing {
            step = step * voice.envelope_level / 0x8000;
        }
        voice.envelope_counter = voice.envelope_counter.wrapping_add(increment);
        if voice.envelope_counter & 0x8000 == 0 {
            return;
        }
        voice.envelope_counter &= 0x7fff;
        voice.envelope_level = (voice.envelope_level + step).clamp(0, 0x7fff);
    }

    fn fixed_volume(value: u16) -> i32 {
        i32::from(((value & 0x7fff) << 1) as i16)
    }

    fn write_volume(register: &mut u16, level: &mut i32, counter: &mut u32, value: u16) {
        *register = value;
        *counter = 0;
        if value & 0x8000 == 0 {
            *level = Self::fixed_volume(value);
        }
    }

    fn tick_volume(register: u16, level: &mut i32, counter: &mut u32) {
        if register & 0x8000 == 0 {
            *level = Self::fixed_volume(register);
            *counter = 0;
            return;
        }
        let exponential = register & 0x4000 != 0;
        let decreasing = register & 0x2000 != 0;
        let phase_negative = register & 0x1000 != 0;
        let shift = (register >> 2) & 0x1f;
        let step_value = register & 3;
        if step_value | (shift << 2) == 0x7f {
            return;
        }
        let mut step = i32::from(7 - step_value);
        if decreasing ^ phase_negative {
            step = !step;
        }
        if shift < 11 {
            step <<= u32::from(11 - shift);
        }
        let mut increment = if shift > 11 {
            0x8000u32 >> u32::from((shift - 11).min(15))
        } else {
            0x8000
        };
        if exponential && !decreasing && *level > 0x6000 {
            if shift < 10 {
                step >>= 2;
            } else if shift >= 11 {
                increment = (increment >> 2).max(1);
            } else {
                step >>= 1;
                increment = (increment >> 1).max(1);
            }
        } else if exponential && decreasing {
            step = step * *level / 0x8000;
        }
        *counter = counter.wrapping_add(increment.max(1));
        if *counter & 0x8000 == 0 {
            return;
        }
        *counter &= 0x7fff;
        let next = *level + step;
        *level = if !decreasing {
            next.clamp(-0x8000, 0x7fff)
        } else if phase_negative {
            next.clamp(-0x8000, 0)
        } else {
            next.max(0)
        };
    }

    fn signed_volume(sample: i32, volume: u16) -> i32 {
        ((i64::from(sample) * i64::from(volume as i16)) / 0x8000) as i32
    }

    fn reverb_address(&self, byte_offset: i64) -> u32 {
        let base = (u32::from(self.reverb_base) * 8) & ADDRESS_MASK;
        let span = (RAM_BYTES as u32).saturating_sub(base).max(2);
        let cursor = if self.reverb_cursor < base {
            base
        } else {
            self.reverb_cursor
        };
        let relative = (i64::from(cursor - base) + byte_offset).rem_euclid(i64::from(span));
        (base + relative as u32) & !1
    }

    fn reverb_offset(&self, register: usize) -> i64 {
        i64::from(self.reverb_registers[register] as i16) * 8
    }

    fn read_reverb(&mut self, register: usize, extra: i64) -> i32 {
        let address = self.reverb_address(self.reverb_offset(register) + extra);
        self.check_irq_address(address);
        i32::from(i16::from_le_bytes([
            self.ram[address as usize],
            self.ram[((address + 1) & ADDRESS_MASK) as usize],
        ]))
    }

    fn read_reverb_displaced(
        &mut self,
        address_register: usize,
        displacement_register: usize,
    ) -> i32 {
        let offset =
            self.reverb_offset(address_register) - self.reverb_offset(displacement_register);
        let address = self.reverb_address(offset);
        self.check_irq_address(address);
        i32::from(i16::from_le_bytes([
            self.ram[address as usize],
            self.ram[((address + 1) & ADDRESS_MASK) as usize],
        ]))
    }

    fn write_reverb(&mut self, register: usize, value: i32) {
        let address = self.reverb_address(self.reverb_offset(register));
        self.check_irq_address(address);
        let sample = value.clamp(i32::from(i16::MIN), i32::from(i16::MAX)) as i16;
        let [lo, hi] = sample.to_le_bytes();
        self.ram[address as usize] = lo;
        self.ram[((address + 1) & ADDRESS_MASK) as usize] = hi;
    }

    fn process_reverb(&mut self, input_left: i32, input_right: i32) {
        let base = (u32::from(self.reverb_base) * 8) & ADDRESS_MASK;
        if self.reverb_cursor < base {
            self.reverb_cursor = base;
        }

        if self.control & (1 << 7) != 0 {
            let lin = Self::signed_volume(input_left, self.reverb_registers[30]);
            let rin = Self::signed_volume(input_right, self.reverb_registers[31]);
            let wall = self.reverb_registers[7];
            let iir = self.reverb_registers[2];

            let left_same_old = self.read_reverb(10, -2);
            let right_same_old = self.read_reverb(11, -2);
            let left_diff_old = self.read_reverb(18, -2);
            let right_diff_old = self.read_reverb(19, -2);
            let left_same = Self::signed_volume(
                lin + Self::signed_volume(self.read_reverb(16, 0), wall) - left_same_old,
                iir,
            ) + left_same_old;
            let right_same = Self::signed_volume(
                rin + Self::signed_volume(self.read_reverb(17, 0), wall) - right_same_old,
                iir,
            ) + right_same_old;
            let left_diff = Self::signed_volume(
                lin + Self::signed_volume(self.read_reverb(25, 0), wall) - left_diff_old,
                iir,
            ) + left_diff_old;
            let right_diff = Self::signed_volume(
                rin + Self::signed_volume(self.read_reverb(24, 0), wall) - right_diff_old,
                iir,
            ) + right_diff_old;
            self.write_reverb(10, left_same);
            self.write_reverb(11, right_same);
            self.write_reverb(18, left_diff);
            self.write_reverb(19, right_diff);
        }

        let mut left = Self::signed_volume(self.read_reverb(12, 0), self.reverb_registers[3])
            + Self::signed_volume(self.read_reverb(14, 0), self.reverb_registers[4])
            + Self::signed_volume(self.read_reverb(20, 0), self.reverb_registers[5])
            + Self::signed_volume(self.read_reverb(22, 0), self.reverb_registers[6]);
        let mut right = Self::signed_volume(self.read_reverb(13, 0), self.reverb_registers[3])
            + Self::signed_volume(self.read_reverb(15, 0), self.reverb_registers[4])
            + Self::signed_volume(self.read_reverb(21, 0), self.reverb_registers[5])
            + Self::signed_volume(self.read_reverb(23, 0), self.reverb_registers[6]);

        let left_apf1 = self.read_reverb_displaced(26, 0);
        let right_apf1 = self.read_reverb_displaced(27, 0);
        left -= Self::signed_volume(left_apf1, self.reverb_registers[8]);
        right -= Self::signed_volume(right_apf1, self.reverb_registers[8]);
        if self.control & (1 << 7) != 0 {
            self.write_reverb(26, left);
            self.write_reverb(27, right);
        }
        left = Self::signed_volume(left, self.reverb_registers[8]) + left_apf1;
        right = Self::signed_volume(right, self.reverb_registers[8]) + right_apf1;

        let left_apf2 = self.read_reverb_displaced(28, 1);
        let right_apf2 = self.read_reverb_displaced(29, 1);
        left -= Self::signed_volume(left_apf2, self.reverb_registers[9]);
        right -= Self::signed_volume(right_apf2, self.reverb_registers[9]);
        if self.control & (1 << 7) != 0 {
            self.write_reverb(28, left);
            self.write_reverb(29, right);
        }
        left = Self::signed_volume(left, self.reverb_registers[9]) + left_apf2;
        right = Self::signed_volume(right, self.reverb_registers[9]) + right_apf2;

        self.reverb_output_left = Self::signed_volume(left, self.reverb_volume_left);
        self.reverb_output_right = Self::signed_volume(right, self.reverb_volume_right);
        let next = (self.reverb_cursor + 2) & ADDRESS_MASK;
        self.reverb_cursor = if next < base { base } else { next };
    }

    fn update_noise(&mut self) {
        let shift = i32::from((self.control >> 10) & 0x0f);
        let step = i32::from(((self.control >> 8) & 3) + 4);
        self.noise_timer -= step;
        let reload = (0x20_000i32 >> shift).max(1);
        for _ in 0..2 {
            if self.noise_timer >= 0 {
                break;
            }
            let parity = ((self.noise_level >> 15)
                ^ (self.noise_level >> 12)
                ^ (self.noise_level >> 11)
                ^ (self.noise_level >> 10)
                ^ 1)
                & 1;
            self.noise_level = self.noise_level.wrapping_shl(1) | parity;
            self.noise_timer += reload;
        }
    }

    pub fn drain_output(&mut self) -> Vec<f32> {
        std::mem::take(&mut self.output)
    }

    pub fn save(&self, out: &mut StateWriter) {
        out.blob(&self.ram);
        for voice in &self.voices {
            voice.save(out);
        }
        out.u16(self.main_volume_left);
        out.u16(self.main_volume_right);
        out.u32(self.main_volume_left_level as u32);
        out.u32(self.main_volume_right_level as u32);
        out.u32(self.main_volume_left_counter);
        out.u32(self.main_volume_right_counter);
        out.u16(self.reverb_volume_left);
        out.u16(self.reverb_volume_right);
        for register in self.reverb_registers {
            out.u16(register);
        }
        out.u32(self.reverb_cursor);
        out.u8(self.reverb_phase as u8);
        out.u32(self.reverb_output_left as u32);
        out.u32(self.reverb_output_right as u32);
        out.u32(self.pitch_modulation);
        out.u32(self.noise_enable);
        out.u32(self.reverb_enable);
        out.u32(self.end_flags);
        out.u16(self.reverb_base);
        out.u16(self.irq_address);
        out.u16(self.transfer_address);
        out.u32(self.transfer_cursor);
        out.u16(self.control);
        out.u16(self.transfer_control);
        out.u16(self.status);
        out.u16(self.cd_volume_left);
        out.u16(self.cd_volume_right);
        out.u32(self.cd_audio.len() as u32);
        for frame in &self.cd_audio {
            out.u16(frame[0] as u16);
            out.u16(frame[1] as u16);
        }
        out.u16(self.external_volume_left);
        out.u16(self.external_volume_right);
        out.u64(self.sample_phase);
        out.u16(self.noise_level);
        out.u32(self.noise_timer as u32);
        out.u8(self.irq_pending as u8);
        out.u32(self.output.len() as u32);
        for sample in &self.output {
            out.f32(*sample);
        }
    }

    pub fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let ram = input.blob()?;
        if ram.len() != RAM_BYTES {
            return Err("invalid PlayStation SPU RAM state length".into());
        }
        self.ram.copy_from_slice(ram);
        for voice in &mut self.voices {
            voice.load(input)?;
        }
        self.main_volume_left = input.u16()?;
        self.main_volume_right = input.u16()?;
        self.main_volume_left_level = input.u32()? as i32;
        self.main_volume_right_level = input.u32()? as i32;
        self.main_volume_left_counter = input.u32()? & 0x7fff;
        self.main_volume_right_counter = input.u32()? & 0x7fff;
        self.reverb_volume_left = input.u16()?;
        self.reverb_volume_right = input.u16()?;
        for register in &mut self.reverb_registers {
            *register = input.u16()?;
        }
        self.reverb_cursor = input.u32()? & ADDRESS_MASK;
        self.reverb_phase = input.u8()? != 0;
        self.reverb_output_left = input.u32()? as i32;
        self.reverb_output_right = input.u32()? as i32;
        self.pitch_modulation = input.u32()? & 0x00ff_ffff;
        self.noise_enable = input.u32()? & 0x00ff_ffff;
        self.reverb_enable = input.u32()? & 0x00ff_ffff;
        self.end_flags = input.u32()? & 0x00ff_ffff;
        self.reverb_base = input.u16()?;
        self.irq_address = input.u16()?;
        self.transfer_address = input.u16()?;
        self.transfer_cursor = input.u32()? & ADDRESS_MASK;
        self.control = input.u16()?;
        self.transfer_control = input.u16()?;
        self.status = input.u16()?;
        self.cd_volume_left = input.u16()?;
        self.cd_volume_right = input.u16()?;
        let cd_audio_len = input.u32()? as usize;
        if cd_audio_len > MAX_CD_INPUT_FRAMES {
            return Err("invalid PlayStation SPU CD audio queue length".into());
        }
        self.cd_audio.clear();
        for _ in 0..cd_audio_len {
            self.cd_audio
                .push_back([input.u16()? as i16, input.u16()? as i16]);
        }
        self.external_volume_left = input.u16()?;
        self.external_volume_right = input.u16()?;
        self.sample_phase = input.u64()? % CPU_HZ;
        self.noise_level = input.u16()?;
        self.noise_timer = input.u32()? as i32;
        self.irq_pending = input.u8()? != 0;
        let output_len = input.u32()? as usize;
        if output_len > MAX_QUEUED_FRAMES * 2 {
            return Err("invalid PlayStation SPU queued-audio length".into());
        }
        self.output.clear();
        self.output.reserve(output_len);
        for _ in 0..output_len {
            self.output.push(input.f32()?);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::platform::PlatformId;

    fn write_block(spu: &mut Ps1Spu, address: u16, header: u8, flags: u8, packed: u8) {
        spu.write16(0x1f80_1da6, address);
        for byte_pair in [u16::from_le_bytes([header, flags])]
            .into_iter()
            .chain(std::iter::repeat_n(u16::from_le_bytes([packed, packed]), 7))
        {
            spu.transfer_write_halfword(byte_pair);
        }
    }

    fn start_voice(spu: &mut Ps1Spu, start: u16) {
        spu.write16(0x1f80_1c00, 0x3fff);
        spu.write16(0x1f80_1c02, 0x3fff);
        spu.write16(0x1f80_1c04, 0x1000);
        spu.write16(0x1f80_1c06, start);
        spu.write16(0x1f80_1c08, 0x000f);
        spu.write16(0x1f80_1c0a, 0x0000);
        spu.write16(0x1f80_1d80, 0x3fff);
        spu.write16(0x1f80_1d82, 0x3fff);
        spu.write16(0x1f80_1daa, 0xc000);
        spu.write16(0x1f80_1d88, 1);
    }

    #[test]
    fn adpcm_voice_generates_stereo_audio_and_end_flag() {
        let mut spu = Ps1Spu::new();
        write_block(&mut spu, 0x200, 0x0c, 0x01, 0x77);
        start_voice(&mut spu, 0x200);
        spu.tick_cpu_cycles((CPU_HZ / 100) as u32);
        let output = spu.drain_output();
        assert!(!output.is_empty());
        assert!(output.iter().any(|sample| *sample != 0.0));
        assert_eq!(spu.read16(0x1f80_1d9c) & 1, 1);
    }

    #[test]
    fn gaussian_interpolation_uses_four_sample_history_and_pitch_fraction() {
        let mut voice = Voice {
            interpolation: [1_000, 2_000, 3_000, 4_000],
            interpolation_ready: true,
            ..Voice::default()
        };
        voice.pitch_counter = 0;
        let phase_zero = Ps1Spu::gaussian_sample(&voice);
        voice.pitch_counter = 0x0ff0;
        let phase_end = Ps1Spu::gaussian_sample(&voice);
        assert_ne!(phase_zero, phase_end);
        assert!((1_000..=4_000).contains(&phase_zero));
        assert!((1_000..=4_000).contains(&phase_end));

        voice.interpolation = [10_000; 4];
        voice.pitch_counter = 0x0800;
        let constant = Ps1Spu::gaussian_sample(&voice);
        assert!((9_950..=9_970).contains(&constant));
    }

    #[test]
    fn dma_transfer_cursor_round_trips_sound_ram() {
        let mut spu = Ps1Spu::new();
        spu.write16(0x1f80_1da6, 0x1234);
        spu.transfer_write_halfword(0x5678);
        spu.transfer_write_halfword(0xabcd);
        spu.write16(0x1f80_1da6, 0x1234);
        assert_eq!(spu.transfer_read_halfword(), 0x5678);
        assert_eq!(spu.transfer_read_halfword(), 0xabcd);
    }

    #[test]
    fn irq_address_and_control_acknowledgement_follow_spu_state() {
        let mut spu = Ps1Spu::new();
        spu.write16(0x1f80_1da4, 0x0040);
        spu.write16(0x1f80_1daa, 0x8040);
        spu.write16(0x1f80_1da6, 0x0040);
        assert!(spu.irq_pending());
        assert_ne!(spu.read16(0x1f80_1dae) & (1 << 6), 0);
        spu.write16(0x1f80_1daa, 0x8000);
        assert!(!spu.irq_pending());
        assert_eq!(spu.read16(0x1f80_1dae) & (1 << 6), 0);
    }

    #[test]
    fn fixed_and_swept_volume_update_current_level_registers() {
        let mut spu = Ps1Spu::new();
        spu.write16(0x1f80_1c00, 0x2000);
        assert_eq!(spu.read16(0x1f80_1e00), 0x4000);

        spu.write16(0x1f80_1c00, 0x8000);
        spu.tick_cpu_cycles((CPU_HZ / SAMPLE_HZ) as u32);
        assert!(spu.read16(0x1f80_1e00) > 0x4000);

        spu.write16(0x1f80_1d80, 0x1000);
        assert_eq!(spu.read16(0x1f80_1db8), 0x2000);
        spu.write16(0x1f80_1d80, 0x807f);
        for _ in 0..8 {
            spu.tick_cpu_cycles((CPU_HZ / SAMPLE_HZ) as u32);
        }
        assert_eq!(spu.read16(0x1f80_1db8), 0x2000);
    }

    #[test]
    fn volume_sweep_phase_and_direction_follow_signed_envelope_rules() {
        let mut level = 0x2000;
        let mut counter = 0;
        Ps1Spu::tick_volume(0xa000, &mut level, &mut counter);
        assert!(level < 0x2000);

        level = -0x2000;
        counter = 0;
        Ps1Spu::tick_volume(0xb000, &mut level, &mut counter);
        assert!(level > -0x2000);
    }

    #[test]
    fn cd_audio_input_obeys_spu_enable_and_signed_input_volume() {
        let mut spu = Ps1Spu::new();
        spu.write16(0x1f80_1d80, 0x3fff);
        spu.write16(0x1f80_1d82, 0x3fff);
        spu.write16(0x1f80_1db0, 0x7fff);
        spu.write16(0x1f80_1db2, 0x7fff);
        spu.write16(0x1f80_1daa, 0xc001);
        spu.queue_cd_audio([[0x4000, -0x2000]]);
        spu.mix_sample();
        let enabled = spu.drain_output();
        assert_eq!(enabled.len(), 2);
        assert!(enabled[0] > 0.45);
        assert!(enabled[1] < -0.20);

        spu.write16(0x1f80_1daa, 0xc000);
        spu.queue_cd_audio([[0x4000, 0x4000]]);
        spu.mix_sample();
        assert_eq!(spu.drain_output(), vec![0.0, 0.0]);
    }

    #[test]
    fn reverb_registers_process_input_and_master_disable_blocks_writes() {
        let mut spu = Ps1Spu::new();
        spu.write16(0x1f80_1d84, 0x7fff);
        spu.write16(0x1f80_1d86, 0x7fff);
        spu.write16(0x1f80_1da2, 0x7000);
        spu.write16(0x1f80_1dc4, 0x7fff);
        spu.write16(0x1f80_1dc6, 0x7fff);
        spu.write16(0x1f80_1dd0, 0x7fff);
        spu.write16(0x1f80_1dd2, 0x7fff);
        for (address, value) in [
            (0x1f80_1dd4, 1),
            (0x1f80_1dd6, 2),
            (0x1f80_1dd8, 1),
            (0x1f80_1dda, 2),
            (0x1f80_1df4, 3),
            (0x1f80_1df6, 4),
            (0x1f80_1df8, 5),
            (0x1f80_1dfa, 6),
        ] {
            spu.write16(address, value);
        }
        spu.write16(0x1f80_1dfc, 0x7fff);
        spu.write16(0x1f80_1dfe, 0x7fff);
        assert_eq!(spu.read16(0x1f80_1dc6), 0x7fff);

        spu.write16(0x1f80_1daa, 0xc080);
        let initial_cursor = spu.reverb_cursor;
        let same_address = spu.reverb_address(spu.reverb_offset(10));
        spu.process_reverb(0x4000, 0x2000);
        assert!(spu.reverb_output_left > 0);
        assert!(spu.reverb_output_right > 0);
        assert_eq!(spu.reverb_cursor, initial_cursor + 2);
        let written = i16::from_le_bytes([
            spu.ram[same_address as usize],
            spu.ram[((same_address + 1) & ADDRESS_MASK) as usize],
        ]);
        assert!(written > 0);

        spu.write16(0x1f80_1daa, 0xc000);
        let disabled_address = spu.reverb_address(spu.reverb_offset(10));
        let before = [
            spu.ram[disabled_address as usize],
            spu.ram[((disabled_address + 1) & ADDRESS_MASK) as usize],
        ];
        spu.process_reverb(0x6000, 0x6000);
        assert_eq!(
            [
                spu.ram[disabled_address as usize],
                spu.ram[((disabled_address + 1) & ADDRESS_MASK) as usize],
            ],
            before
        );
    }

    #[test]
    fn state_round_trip_preserves_voice_ram_and_pending_audio() {
        let mut first = Ps1Spu::new();
        write_block(&mut first, 0x100, 0x0c, 0x03, 0x55);
        start_voice(&mut first, 0x100);
        first.tick_cpu_cycles((CPU_HZ / 200) as u32);
        let mut writer = StateWriter::new(PlatformId::PlayStation, 99);
        first.save(&mut writer);
        let state = writer.finish();
        let mut reader = StateReader::new(&state, PlatformId::PlayStation, 99).unwrap();
        let mut second = Ps1Spu::new();
        second.load(&mut reader).unwrap();
        reader.finish().unwrap();
        let mut first_writer = StateWriter::new(PlatformId::PlayStation, 99);
        first.save(&mut first_writer);
        let mut second_writer = StateWriter::new(PlatformId::PlayStation, 99);
        second.save(&mut second_writer);
        assert_eq!(first_writer.finish(), second_writer.finish());
    }
}
