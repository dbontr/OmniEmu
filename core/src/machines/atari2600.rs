use crate::bus::Bus8;
use crate::cpu6502::Mos6502;
use crate::input::{
    AXIS_AUX_X, AXIS_AUX_Y, AXIS_LEFT_X, AXIS_LEFT_Y, DOWN, FACE_SOUTH, LEFT, POINTER_CLICK,
    POINTER_TOUCH, RIGHT, SELECT, START, UP,
};
use crate::kernel::{AudioBuffer, InputState, VideoBuffer};
use crate::machine::Machine;
use crate::platform::PlatformId;
use crate::state::{StateReader, StateWriter};

const WIDTH: u32 = 160;
const HEIGHT: u32 = 192;
const CPU_HZ: u64 = 1_193_182;
const SAMPLE_RATE: u64 = 48_000;
const FRAME_RATE: f64 = 59.922_743;
const STATE_VERSION: u32 = 7;
const COLOR_CLOCKS_PER_LINE: u16 = 228;
const HBLANK_CLOCKS: u16 = 68;
const PADDLE_MAX_RESISTANCE: u32 = 1_000_000;
const PADDLE_SERIES_RESISTANCE: f64 = 1_800.0;
const PADDLE_CAPACITANCE: f64 = 68e-9;
const PADDLE_DUMP_RESISTANCE: f64 = 50.0;
const PADDLE_SUPPLY_VOLTAGE: f64 = 5.0;
const PADDLE_TRIP_LINES: f64 = 379.0;

fn contains_signature(image: &[u8], signature: &[u8], minimum_hits: usize) -> bool {
    if signature.is_empty() || image.len() < signature.len() {
        return false;
    }
    image
        .windows(signature.len())
        .filter(|window| *window == signature)
        .take(minimum_hits)
        .count()
        >= minimum_hits
}

fn contains_any_signature(image: &[u8], signatures: &[&[u8]]) -> bool {
    signatures
        .iter()
        .any(|signature| contains_signature(image, signature, 1))
}

fn is_probably_superchip(image: &[u8]) -> bool {
    image.len().is_multiple_of(0x1000)
        && image
            .as_chunks::<0x1000>()
            .0
            .iter()
            .all(|bank| bank[..0x80] == bank[0x80..0x100])
}

fn is_probably_e0(image: &[u8]) -> bool {
    contains_any_signature(
        image,
        &[
            &[0x8d, 0xe0, 0x1f],
            &[0x8d, 0xe0, 0x5f],
            &[0x8d, 0xe9, 0xff],
            &[0x0c, 0xe0, 0x1f],
            &[0xad, 0xe0, 0x1f],
            &[0xad, 0xe9, 0xff],
            &[0xad, 0xed, 0xff],
            &[0xad, 0xf3, 0xbf],
        ],
    )
}

fn is_probably_e7(image: &[u8]) -> bool {
    contains_any_signature(
        image,
        &[
            &[0xad, 0xe2, 0xff],
            &[0xad, 0xe5, 0xff],
            &[0xad, 0xe5, 0x1f],
            &[0xad, 0xe7, 0x1f],
            &[0x0c, 0xe7, 0x1f],
            &[0x8d, 0xe7, 0xff],
            &[0x8d, 0xe7, 0x1f],
        ],
    )
}

fn is_probably_e7_8k(image: &[u8]) -> bool {
    contains_any_signature(
        image,
        &[
            &[0xad, 0xe4, 0xff],
            &[0xad, 0xe5, 0xff],
            &[0xad, 0xe6, 0xff],
        ],
    )
}

fn is_probably_ef(image: &[u8]) -> bool {
    contains_any_signature(
        image,
        &[
            &[0x0c, 0xe0, 0xff],
            &[0xad, 0xe0, 0xff],
            &[0x0c, 0xe0, 0x1f],
            &[0xad, 0xe0, 0x1f],
        ],
    )
}

fn has_tail_signature(image: &[u8], signature: &[u8; 4]) -> bool {
    image.len() >= 8
        && image[image.len() - 8..]
            .windows(4)
            .any(|window| window == signature)
}

fn is_probably_bf(image: &[u8]) -> bool {
    has_tail_signature(image, b"BFBF")
}

fn is_probably_bfsc(image: &[u8]) -> bool {
    has_tail_signature(image, b"BFSC")
}

fn is_probably_df(image: &[u8]) -> bool {
    has_tail_signature(image, b"DFDF")
}

fn is_probably_dfsc(image: &[u8]) -> bool {
    has_tail_signature(image, b"DFSC")
}

fn is_probably_3e_plus(image: &[u8]) -> bool {
    contains_signature(image, b"TJ3E", 1)
}

fn is_probably_3e(image: &[u8]) -> bool {
    contains_signature(image, &[0x85, 0x3e, 0xa9, 0x00], 1)
}

fn is_probably_3f(image: &[u8]) -> bool {
    contains_signature(image, &[0x85, 0x3f], 2)
}

fn is_probably_sb(image: &[u8]) -> bool {
    contains_any_signature(image, &[&[0xbd, 0x00, 0x08], &[0xad, 0x00, 0x08]])
}

fn is_probably_ua(image: &[u8]) -> bool {
    contains_any_signature(
        image,
        &[
            &[0x8d, 0x40, 0x02],
            &[0xad, 0x40, 0x02],
            &[0xbd, 0x1f, 0x02],
            &[0x2c, 0xc0, 0x02],
            &[0x8d, 0xc0, 0x02],
            &[0xad, 0xc0, 0x02],
            &[0x2c, 0xb0, 0x0f],
        ],
    )
}

fn is_probably_fe(image: &[u8]) -> bool {
    contains_any_signature(
        image,
        &[
            &[0x20, 0x00, 0xd0, 0xc6, 0xc5],
            &[0x20, 0xc3, 0xf8, 0xa5, 0x82],
            &[0xd0, 0xfb, 0x20, 0x73, 0xfe],
            &[0xd0, 0xfb, 0x20, 0x68, 0xfe],
            &[0x20, 0x00, 0xf0, 0x84, 0xd6],
        ],
    )
}

fn has_f8_signature(image: &[u8]) -> bool {
    contains_signature(image, &[0x8d, 0xf9, 0x1f], 2)
        || contains_signature(image, &[0x8d, 0xf9, 0xff], 2)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CartScheme {
    TwoK,
    FourK,
    Bf,
    BfSc,
    F8,
    F8Sc,
    F6,
    F6Sc,
    F4,
    F4Sc,
    Df,
    DfSc,
    Dpc,
    E0,
    E7,
    Ef,
    EfSc,
    ThreeE,
    ThreeEPlus,
    ThreeF,
    Fe,
    Ua,
    F0,
    Fa,
    Sb,
}

struct AtariCartridge {
    rom: Vec<u8>,
    scheme: CartScheme,
    bank: usize,
    segments: [usize; 4],
    segment_ram: [bool; 4],
    ram: Vec<u8>,
    ram_bank: usize,
    ram_selected: bool,
    last_access_was_fe: bool,
    dpc_tops: [u8; 8],
    dpc_bottoms: [u8; 8],
    dpc_counters: [u16; 8],
    dpc_flags: [u8; 8],
    dpc_music_mode: [bool; 3],
    dpc_random: u8,
    dpc_music_accum: u64,
}

impl AtariCartridge {
    fn new(rom: &[u8]) -> Result<Self, String> {
        if rom.is_empty() {
            return Err("Atari 2600 cartridge image is empty".into());
        }
        let scheme = match rom.len() {
            size if (0x1000..=0x10000).contains(&size)
                && size.is_multiple_of(0x400)
                && is_probably_3e_plus(rom) =>
            {
                CartScheme::ThreeEPlus
            }
            1..=0x0800 => CartScheme::TwoK,
            0x1000 => CartScheme::FourK,
            0x2000 if is_probably_3e(rom) => CartScheme::ThreeE,
            0x2000 if is_probably_superchip(rom) => CartScheme::F8Sc,
            0x2000 if rom[..0x1000] == rom[0x1000..] => CartScheme::FourK,
            0x2000 if is_probably_e0(rom) => CartScheme::E0,
            0x2000 if is_probably_3f(rom) => CartScheme::ThreeF,
            0x2000 if is_probably_ua(rom) => CartScheme::Ua,
            0x2000 if is_probably_fe(rom) && !has_f8_signature(rom) => CartScheme::Fe,
            0x2000 if is_probably_e7_8k(rom) => CartScheme::E7,
            0x2000 => CartScheme::F8,
            0x2800 => CartScheme::Dpc,
            0x3000 if is_probably_e7(rom) => CartScheme::E7,
            0x3000 => CartScheme::Fa,
            0x4000 if is_probably_3e(rom) => CartScheme::ThreeE,
            0x4000 if is_probably_superchip(rom) => CartScheme::F6Sc,
            0x4000 if is_probably_e7(rom) => CartScheme::E7,
            0x4000 => CartScheme::F6,
            0x8000 if is_probably_3e(rom) => CartScheme::ThreeE,
            0x8000 if is_probably_superchip(rom) => CartScheme::F4Sc,
            0x8000 if is_probably_3f(rom) => CartScheme::ThreeF,
            0x8000 => CartScheme::F4,
            0x10000 if is_probably_3e(rom) => CartScheme::ThreeE,
            0x10000 if is_probably_ef(rom) && is_probably_superchip(rom) => CartScheme::EfSc,
            0x10000 if is_probably_ef(rom) => CartScheme::Ef,
            0x10000 if is_probably_3f(rom) => CartScheme::ThreeF,
            0x10000 => CartScheme::F0,
            0x20000 if is_probably_dfsc(rom) => CartScheme::DfSc,
            0x20000 if is_probably_df(rom) => CartScheme::Df,
            0x40000 if is_probably_bfsc(rom) => CartScheme::BfSc,
            0x40000 if is_probably_bf(rom) => CartScheme::Bf,
            0x20000 | 0x40000 if is_probably_sb(rom) => CartScheme::Sb,
            0x20000 | 0x40000 | 0x80000 if is_probably_3e(rom) => CartScheme::ThreeE,
            0x20000 | 0x40000 | 0x80000 if is_probably_3f(rom) => CartScheme::ThreeF,
            size => {
                return Err(format!(
                    "Atari 2600 cartridge size {size} or banking scheme is not supported"
                ));
            }
        };
        let bank = match scheme {
            CartScheme::TwoK
            | CartScheme::FourK
            | CartScheme::E0
            | CartScheme::E7
            | CartScheme::ThreeE
            | CartScheme::ThreeEPlus
            | CartScheme::ThreeF
            | CartScheme::Fe
            | CartScheme::Ua => 0,
            CartScheme::Bf
            | CartScheme::BfSc
            | CartScheme::Ef
            | CartScheme::EfSc
            | CartScheme::Dpc
            | CartScheme::F8
            | CartScheme::F8Sc => 1,
            CartScheme::F6 | CartScheme::F6Sc => 3,
            CartScheme::F4 | CartScheme::F4Sc => 7,
            CartScheme::Df | CartScheme::DfSc | CartScheme::F0 => 15,
            CartScheme::Fa => 2,
            CartScheme::Sb => rom.len() / 0x1000 - 1,
        };
        let ram = match scheme {
            CartScheme::BfSc
            | CartScheme::F8Sc
            | CartScheme::F6Sc
            | CartScheme::F4Sc
            | CartScheme::DfSc
            | CartScheme::EfSc => vec![0; 0x80],
            CartScheme::E7 => vec![0; 0x800],
            CartScheme::ThreeE | CartScheme::ThreeEPlus => vec![0; 0x8000],
            CartScheme::Fa => vec![0; 0x100],
            _ => Vec::new(),
        };
        Ok(Self {
            rom: rom.to_vec(),
            scheme,
            bank,
            segments: if scheme == CartScheme::ThreeEPlus {
                [0; 4]
            } else {
                [4, 5, 6, 0]
            },
            segment_ram: [false; 4],
            ram,
            ram_bank: 0,
            ram_selected: false,
            last_access_was_fe: false,
            dpc_tops: [0; 8],
            dpc_bottoms: [0; 8],
            dpc_counters: [0; 8],
            dpc_flags: [0; 8],
            dpc_music_mode: [false; 3],
            dpc_random: 1,
            dpc_music_accum: 0,
        })
    }

    fn select_hotspot(&mut self, address: u16) {
        match self.scheme {
            CartScheme::Dpc | CartScheme::F8 | CartScheme::F8Sc
                if matches!(address, 0x1ff8 | 0x1ff9) =>
            {
                self.bank = usize::from(address - 0x1ff8);
            }
            CartScheme::F6 | CartScheme::F6Sc if (0x1ff6..=0x1ff9).contains(&address) => {
                self.bank = usize::from(address - 0x1ff6);
            }
            CartScheme::F4 | CartScheme::F4Sc if (0x1ff4..=0x1ffb).contains(&address) => {
                self.bank = usize::from(address - 0x1ff4);
            }
            CartScheme::E0 if (0x1fe0..=0x1fe7).contains(&address) => {
                self.segments[0] = usize::from(address & 7);
            }
            CartScheme::E0 if (0x1fe8..=0x1fef).contains(&address) => {
                self.segments[1] = usize::from(address & 7);
            }
            CartScheme::E0 if (0x1ff0..=0x1ff7).contains(&address) => {
                self.segments[2] = usize::from(address & 7);
            }
            CartScheme::E7 if (0x1fe0..=0x1fe7).contains(&address) => {
                let banks = self.rom.len() / 0x800;
                self.bank = match banks {
                    4 if address >= 0x1fe4 => usize::from(address & 3),
                    6 => [0, 1, 0, 1, 2, 3, 4, 5][usize::from(address & 7)],
                    8 => usize::from(address & 7),
                    _ => self.bank,
                };
            }
            CartScheme::E7 if (0x1fe8..=0x1feb).contains(&address) => {
                self.ram_bank = usize::from(address & 3);
            }
            CartScheme::Bf | CartScheme::BfSc if (0x1f80..=0x1fbf).contains(&address) => {
                self.bank = usize::from(address - 0x1f80);
            }
            CartScheme::Df | CartScheme::DfSc if (0x1fc0..=0x1fdf).contains(&address) => {
                self.bank = usize::from(address - 0x1fc0);
            }
            CartScheme::Ef | CartScheme::EfSc if (0x1fe0..=0x1fef).contains(&address) => {
                self.bank = usize::from(address - 0x1fe0);
            }
            CartScheme::F0 if address == 0x1ff0 => {
                self.bank = (self.bank + 1) & 0x0f;
            }
            CartScheme::Fa if (0x1ff8..=0x1ffa).contains(&address) => {
                self.bank = usize::from(address - 0x1ff8);
            }
            _ => {}
        }
    }

    fn dpc_clock_random(&mut self) {
        const FEEDBACK: [u8; 16] = [1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1];
        let selector = usize::from((self.dpc_random >> 3) & 0x07)
            | if self.dpc_random & 0x80 != 0 { 0x08 } else { 0 };
        self.dpc_random = self.dpc_random.wrapping_shl(1) | FEEDBACK[selector];
    }

    fn dpc_tick(&mut self, cycles: u32) {
        if self.scheme != CartScheme::Dpc {
            return;
        }
        self.dpc_music_accum = self
            .dpc_music_accum
            .saturating_add(u64::from(cycles).saturating_mul(20_000));
        let clocks = self.dpc_music_accum / CPU_HZ;
        self.dpc_music_accum %= CPU_HZ;
        if clocks == 0 {
            return;
        }

        for index in 5..8usize {
            if !self.dpc_music_mode[index - 5] {
                continue;
            }
            let top = u64::from(self.dpc_tops[index]);
            let new_low = if top == 0 {
                0
            } else {
                let period = top + 1;
                let low = u64::from((self.dpc_counters[index] & 0x00ff) as u8);
                ((low + period - (clocks % period)) % period) as u8
            };
            if new_low <= self.dpc_bottoms[index] {
                self.dpc_flags[index] = 0;
            } else if new_low <= self.dpc_tops[index] {
                self.dpc_flags[index] = 0xff;
            }
            self.dpc_counters[index] = (self.dpc_counters[index] & 0x0700) | u16::from(new_low);
        }
    }

    fn dpc_read_register(&mut self, offset: usize) -> u8 {
        let index = offset & 0x07;
        let function = (offset >> 3) & 0x07;
        let low = (self.dpc_counters[index] & 0x00ff) as u8;
        if low == self.dpc_tops[index] {
            self.dpc_flags[index] = 0xff;
        } else if low == self.dpc_bottoms[index] {
            self.dpc_flags[index] = 0;
        }

        let result = match function {
            0 => {
                if index < 4 {
                    self.dpc_random
                } else {
                    const AMPLITUDES: [u8; 8] = [0x00, 0x04, 0x05, 0x09, 0x06, 0x0a, 0x0b, 0x0f];
                    let mut bits = 0usize;
                    if self.dpc_music_mode[0] && self.dpc_flags[5] != 0 {
                        bits |= 1;
                    }
                    if self.dpc_music_mode[1] && self.dpc_flags[6] != 0 {
                        bits |= 2;
                    }
                    if self.dpc_music_mode[2] && self.dpc_flags[7] != 0 {
                        bits |= 4;
                    }
                    AMPLITUDES[bits]
                }
            }
            1 => {
                let display = 0x2000 + 2047 - usize::from(self.dpc_counters[index] & 0x07ff);
                self.rom[display]
            }
            2 => {
                let display = 0x2000 + 2047 - usize::from(self.dpc_counters[index] & 0x07ff);
                self.rom[display] & self.dpc_flags[index]
            }
            7 => self.dpc_flags[index],
            _ => 0,
        };

        if index < 5 || !self.dpc_music_mode[index - 5] {
            self.dpc_counters[index] = self.dpc_counters[index].wrapping_sub(1) & 0x07ff;
        }
        result
    }

    fn dpc_write_register(&mut self, offset: usize, value: u8) {
        let index = offset & 0x07;
        let function = (offset >> 3) & 0x07;
        match function {
            0 => {
                self.dpc_tops[index] = value;
                self.dpc_flags[index] = 0;
            }
            1 => self.dpc_bottoms[index] = value,
            2 => {
                let low = if index >= 5 && self.dpc_music_mode[index - 5] {
                    self.dpc_tops[index]
                } else {
                    value
                };
                self.dpc_counters[index] = (self.dpc_counters[index] & 0x0700) | u16::from(low);
            }
            3 => {
                self.dpc_counters[index] =
                    (u16::from(value & 0x07) << 8) | (self.dpc_counters[index] & 0x00ff);
                if index >= 5 {
                    self.dpc_music_mode[index - 5] = value & 0x10 != 0;
                }
            }
            6 => self.dpc_random = 1,
            _ => {}
        }
    }

    fn reset(&mut self) {
        self.bank = match self.scheme {
            CartScheme::Bf
            | CartScheme::BfSc
            | CartScheme::Ef
            | CartScheme::EfSc
            | CartScheme::Dpc
            | CartScheme::F8
            | CartScheme::F8Sc => 1,
            CartScheme::F6 | CartScheme::F6Sc => 3,
            CartScheme::F4 | CartScheme::F4Sc => 7,
            CartScheme::Df | CartScheme::DfSc | CartScheme::F0 => 15,
            CartScheme::Fa => 2,
            CartScheme::Sb => self.rom.len() / 0x1000 - 1,
            _ => 0,
        };
        self.segments = if self.scheme == CartScheme::ThreeEPlus {
            [0; 4]
        } else {
            [4, 5, 6, 0]
        };
        self.segment_ram = [false; 4];
        self.ram_bank = 0;
        self.ram_selected = false;
        self.last_access_was_fe = false;
        self.dpc_tops = [0; 8];
        self.dpc_bottoms = [0; 8];
        self.dpc_counters = [0; 8];
        self.dpc_flags = [0; 8];
        self.dpc_music_mode = [false; 3];
        self.dpc_random = 1;
        self.dpc_music_accum = 0;
    }

    fn read(&mut self, address: u16) -> u8 {
        if self.scheme == CartScheme::Dpc {
            self.dpc_clock_random();
        }
        let address = 0x1000 | (address & 0x0fff);
        self.select_hotspot(address);
        let offset = usize::from(address & 0x0fff);
        match self.scheme {
            CartScheme::TwoK => self.rom[offset % self.rom.len()],
            CartScheme::FourK => self.rom[offset],
            CartScheme::F8
            | CartScheme::F6
            | CartScheme::F4
            | CartScheme::Bf
            | CartScheme::Df
            | CartScheme::Ef
            | CartScheme::Fe
            | CartScheme::Ua
            | CartScheme::F0
            | CartScheme::Sb => self.rom[self.bank * 0x1000 + offset],
            CartScheme::Dpc => {
                if offset < 0x40 {
                    self.dpc_read_register(offset)
                } else {
                    self.rom[self.bank * 0x1000 + offset]
                }
            }
            CartScheme::BfSc
            | CartScheme::F8Sc
            | CartScheme::F6Sc
            | CartScheme::F4Sc
            | CartScheme::DfSc
            | CartScheme::EfSc => {
                if offset < 0x100 {
                    self.ram[offset & 0x7f]
                } else {
                    self.rom[self.bank * 0x1000 + offset]
                }
            }
            CartScheme::E0 => {
                let segment = offset >> 10;
                let bank = if segment < 3 {
                    self.segments[segment]
                } else {
                    7
                };
                self.rom[bank * 0x400 + (offset & 0x03ff)]
            }
            CartScheme::E7 => {
                let banks = self.rom.len() / 0x800;
                let ram_selector = banks - 1;
                match offset {
                    0x000..=0x7ff if self.bank == ram_selector => self.ram[offset & 0x03ff],
                    0x000..=0x7ff => self.rom[self.bank * 0x800 + offset],
                    0x800..=0x9ff => self.ram[0x400 + self.ram_bank * 0x100 + (offset & 0xff)],
                    _ => self.rom[ram_selector * 0x800 + (offset & 0x07ff)],
                }
            }
            CartScheme::ThreeE => {
                let banks = self.rom.len() / 0x800;
                if offset >= 0x800 {
                    self.rom[(banks - 1) * 0x800 + (offset & 0x07ff)]
                } else if self.ram_selected {
                    if offset < 0x400 {
                        self.ram[self.ram_bank * 0x400 + offset]
                    } else {
                        0xff
                    }
                } else {
                    self.rom[(self.bank % banks) * 0x800 + offset]
                }
            }
            CartScheme::ThreeEPlus => {
                let segment = offset >> 10;
                let within = offset & 0x03ff;
                if self.segment_ram[segment] {
                    if within < 0x200 {
                        self.ram[self.segments[segment] * 0x200 + within]
                    } else {
                        0xff
                    }
                } else {
                    let banks = (self.rom.len() / 0x400).max(1);
                    let bank = self.segments[segment] % banks;
                    self.rom[bank * 0x400 + within]
                }
            }
            CartScheme::ThreeF => {
                let banks = self.rom.len() / 0x800;
                let bank = if offset < 0x800 {
                    self.bank % banks
                } else {
                    banks - 1
                };
                self.rom[bank * 0x800 + (offset & 0x07ff)]
            }
            CartScheme::Fa => {
                if offset < 0x200 {
                    self.ram[offset & 0xff]
                } else {
                    self.rom[self.bank * 0x1000 + offset]
                }
            }
        }
    }

    fn write(&mut self, address: u16, value: u8) {
        if self.scheme == CartScheme::Dpc {
            self.dpc_clock_random();
        }
        let address = 0x1000 | (address & 0x0fff);
        self.select_hotspot(address);
        let offset = usize::from(address & 0x0fff);
        match self.scheme {
            CartScheme::Dpc => {
                if (0x40..0x80).contains(&offset) {
                    self.dpc_write_register(offset, value);
                }
            }
            CartScheme::BfSc
            | CartScheme::F8Sc
            | CartScheme::F6Sc
            | CartScheme::F4Sc
            | CartScheme::DfSc
            | CartScheme::EfSc
                if offset < 0x80 =>
            {
                self.ram[offset] = value;
            }
            CartScheme::E7 => {
                let ram_selector = self.rom.len() / 0x800 - 1;
                if self.bank == ram_selector && offset < 0x400 {
                    self.ram[offset] = value;
                } else if (0x800..0x900).contains(&offset) {
                    let index = 0x400 + self.ram_bank * 0x100 + (offset & 0xff);
                    self.ram[index] = value;
                }
            }
            CartScheme::ThreeE if self.ram_selected && (0x400..0x800).contains(&offset) => {
                let index = self.ram_bank * 0x400 + (offset & 0x03ff);
                self.ram[index] = value;
            }
            CartScheme::ThreeEPlus => {
                let segment = offset >> 10;
                let within = offset & 0x03ff;
                if self.segment_ram[segment] && within >= 0x200 {
                    let index = self.segments[segment] * 0x200 + (within & 0x01ff);
                    self.ram[index] = value;
                }
            }
            CartScheme::Fa if offset < 0x100 => self.ram[offset] = value,
            _ => {}
        }
    }

    fn observe_bus_access(&mut self, address: u16, value: u8, write: bool) {
        let address = address & 0x1fff;
        match self.scheme {
            CartScheme::Sb if (address & 0x1800) == 0x0800 => {
                let banks = self.rom.len() / 0x1000;
                self.bank = usize::from(address) & (banks - 1);
            }
            CartScheme::ThreeE if write && address == 0x003e => {
                self.ram_bank = usize::from(value & 0x1f);
                self.ram_selected = true;
            }
            CartScheme::ThreeE if write && address == 0x003f => {
                let banks = self.rom.len() / 0x800;
                self.bank = usize::from(value) % banks;
                self.ram_selected = false;
            }
            CartScheme::ThreeEPlus if write && address == 0x003e => {
                let segment = usize::from(value >> 6);
                self.segments[segment] = usize::from(value & 0x3f);
                self.segment_ram[segment] = true;
            }
            CartScheme::ThreeEPlus if write && address == 0x003f => {
                let segment = usize::from(value >> 6);
                let banks = (self.rom.len() / 0x400).max(1);
                self.segments[segment] = usize::from(value & 0x3f) % banks;
                self.segment_ram[segment] = false;
            }
            CartScheme::ThreeF if write && address <= 0x003f => {
                let banks = self.rom.len() / 0x800;
                self.bank = usize::from(value) % banks;
            }
            CartScheme::Ua => match address & 0x1260 {
                0x0220 => self.bank = 0,
                0x0240 => self.bank = 1,
                _ => {}
            },
            CartScheme::Fe => {
                if self.last_access_was_fe {
                    self.bank = usize::from(((value >> 5) ^ 7) & 1);
                    self.last_access_was_fe = false;
                }
                if address == 0x01fe {
                    self.last_access_was_fe = true;
                }
            }
            _ => {}
        }
    }

    fn save(&self, out: &mut StateWriter) {
        out.u16(self.bank as u16);
        for segment in self.segments {
            out.u8(segment as u8);
        }
        for is_ram in self.segment_ram {
            out.u8(is_ram as u8);
        }
        out.u8(self.ram_bank as u8);
        out.u8(self.ram_selected as u8);
        out.u8(self.last_access_was_fe as u8);
        out.blob(&self.dpc_tops);
        out.blob(&self.dpc_bottoms);
        for counter in self.dpc_counters {
            out.u16(counter);
        }
        out.blob(&self.dpc_flags);
        for mode in self.dpc_music_mode {
            out.u8(mode as u8);
        }
        out.u8(self.dpc_random);
        out.u64(self.dpc_music_accum);
        out.blob(&self.ram);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let bank = usize::from(input.u16()?);
        let bank_count = match self.scheme {
            CartScheme::TwoK | CartScheme::FourK | CartScheme::E0 | CartScheme::ThreeEPlus => 1,
            CartScheme::Dpc
            | CartScheme::F8
            | CartScheme::F8Sc
            | CartScheme::F6
            | CartScheme::F6Sc
            | CartScheme::F4
            | CartScheme::F4Sc
            | CartScheme::Bf
            | CartScheme::BfSc
            | CartScheme::Df
            | CartScheme::DfSc
            | CartScheme::Ef
            | CartScheme::EfSc
            | CartScheme::Fe
            | CartScheme::Sb
            | CartScheme::Ua
            | CartScheme::F0
            | CartScheme::Fa => (self.rom.len() / 0x1000).max(1),
            CartScheme::E7 | CartScheme::ThreeE | CartScheme::ThreeF => {
                (self.rom.len() / 0x800).max(1)
            }
        };
        if bank >= bank_count {
            return Err("Atari 2600 state contains an invalid cartridge bank".into());
        }
        let mut segments = [0usize; 4];
        for segment in &mut segments {
            *segment = usize::from(input.u8()?);
        }
        let mut segment_ram = [false; 4];
        for is_ram in &mut segment_ram {
            *is_ram = input.u8()? != 0;
        }
        if self.scheme == CartScheme::ThreeEPlus {
            let rom_banks = (self.rom.len() / 0x400).max(1);
            for (segment, is_ram) in segments.iter().zip(segment_ram) {
                let limit = if is_ram { 64 } else { rom_banks };
                if *segment >= limit {
                    return Err("Atari 2600 state contains an invalid 3E+ segment bank".into());
                }
            }
        } else if segments[..3].iter().any(|segment| *segment >= 8) {
            return Err("Atari 2600 state contains an invalid E0 segment bank".into());
        }
        let ram_bank = usize::from(input.u8()?);
        let ram_bank_count = if self.scheme == CartScheme::ThreeE {
            32
        } else {
            4
        };
        if ram_bank >= ram_bank_count {
            return Err("Atari 2600 state contains an invalid cartridge RAM bank".into());
        }
        let ram_selected = input.u8()? != 0;
        let last_access_was_fe = input.u8()? != 0;

        let dpc_tops_blob = input.blob()?;
        if dpc_tops_blob.len() != 8 {
            return Err("Atari 2600 state contains invalid DPC top registers".into());
        }
        let mut dpc_tops = [0u8; 8];
        dpc_tops.copy_from_slice(dpc_tops_blob);

        let dpc_bottoms_blob = input.blob()?;
        if dpc_bottoms_blob.len() != 8 {
            return Err("Atari 2600 state contains invalid DPC bottom registers".into());
        }
        let mut dpc_bottoms = [0u8; 8];
        dpc_bottoms.copy_from_slice(dpc_bottoms_blob);

        let mut dpc_counters = [0u16; 8];
        for counter in &mut dpc_counters {
            *counter = input.u16()? & 0x07ff;
        }

        let dpc_flags_blob = input.blob()?;
        if dpc_flags_blob.len() != 8 {
            return Err("Atari 2600 state contains invalid DPC flag registers".into());
        }
        let mut dpc_flags = [0u8; 8];
        dpc_flags.copy_from_slice(dpc_flags_blob);

        let mut dpc_music_mode = [false; 3];
        for mode in &mut dpc_music_mode {
            *mode = input.u8()? != 0;
        }
        let dpc_random = input.u8()?;
        if dpc_random == 0 {
            return Err("Atari 2600 state contains an invalid DPC random register".into());
        }
        let dpc_music_accum = input.u64()? % CPU_HZ;

        let ram = input.blob()?;
        if ram.len() != self.ram.len() {
            return Err("Atari 2600 state cartridge RAM size differs from loaded cartridge".into());
        }
        self.bank = bank;
        self.segments = segments;
        self.segment_ram = segment_ram;
        self.ram_bank = ram_bank;
        self.ram_selected = ram_selected;
        self.last_access_was_fe = last_access_was_fe;
        self.dpc_tops = dpc_tops;
        self.dpc_bottoms = dpc_bottoms;
        self.dpc_counters = dpc_counters;
        self.dpc_flags = dpc_flags;
        self.dpc_music_mode = dpc_music_mode;
        self.dpc_random = dpc_random;
        self.dpc_music_accum = dpc_music_accum;
        self.ram.copy_from_slice(ram);
        Ok(())
    }
}

#[derive(Clone)]
struct Riot6532 {
    ram: [u8; 128],
    swcha: u8,
    porta_out: u8,
    swacnt: u8,
    swchb: u8,
    portb_out: u8,
    swbcnt: u8,
    timer: u8,
    timer_divider: u16,
    timer_phase: u16,
    timer_irq: bool,
}

impl Default for Riot6532 {
    fn default() -> Self {
        Self {
            ram: [0; 128],
            swcha: 0xff,
            porta_out: 0xff,
            swacnt: 0,
            swchb: 0x3f,
            portb_out: 0xff,
            swbcnt: 0,
            timer: 0xff,
            timer_divider: 1,
            timer_phase: 1,
            timer_irq: false,
        }
    }
}

impl Riot6532 {
    fn reset(&mut self) {
        let ram = self.ram;
        *self = Self::default();
        self.ram = ram;
    }

    fn set_inputs(&mut self, input: &InputState) {
        self.swcha = 0xff;
        self.swchb = 0x3f;
        let p0 = input.buttons[0];
        let p1 = input.buttons[1];
        let lightgun = p0 & POINTER_TOUCH != 0;
        Self::clear(
            &mut self.swcha,
            4,
            if lightgun {
                p0 & POINTER_CLICK != 0
            } else {
                p0 & UP != 0
            },
        );
        Self::clear(&mut self.swcha, 5, p0 & DOWN != 0);
        Self::clear(&mut self.swcha, 6, p0 & LEFT != 0);
        Self::clear(&mut self.swcha, 7, p0 & RIGHT != 0);
        Self::clear(&mut self.swcha, 0, p1 & UP != 0);
        Self::clear(&mut self.swcha, 1, p1 & DOWN != 0);
        Self::clear(&mut self.swcha, 2, p1 & LEFT != 0);
        Self::clear(&mut self.swcha, 3, p1 & RIGHT != 0);
        Self::clear(&mut self.swchb, 0, p0 & START != 0);
        Self::clear(&mut self.swchb, 1, p0 & SELECT != 0);
    }

    fn clear(value: &mut u8, bit: u8, pressed: bool) {
        if pressed {
            *value &= !(1 << bit);
        }
    }

    fn tick(&mut self, cycles: u32) {
        for _ in 0..cycles {
            if self.timer_phase > 1 {
                self.timer_phase -= 1;
                continue;
            }
            self.timer_phase = self.timer_divider.max(1);
            if self.timer == 0 {
                self.timer = 0xff;
                self.timer_divider = 1;
                self.timer_phase = 1;
                self.timer_irq = true;
            } else {
                self.timer = self.timer.wrapping_sub(1);
            }
        }
    }

    fn read_io(&mut self, address: u16) -> u8 {
        match address & 0x1f {
            0x00 => (self.swcha & !self.swacnt) | (self.porta_out & self.swacnt),
            0x01 => self.swacnt,
            0x02 => (self.swchb & !self.swbcnt) | (self.portb_out & self.swbcnt),
            0x03 => self.swbcnt,
            0x04 => {
                self.timer_irq = false;
                self.timer
            }
            0x05 => {
                if self.timer_irq {
                    0x80
                } else {
                    0
                }
            }
            _ => 0xff,
        }
    }

    fn write_io(&mut self, address: u16, value: u8) {
        match address & 0x1f {
            0x00 => self.porta_out = value,
            0x01 => self.swacnt = value,
            0x02 => self.portb_out = value,
            0x03 => self.swbcnt = value,
            0x14 => self.set_timer(value, 1),
            0x15 => self.set_timer(value, 8),
            0x16 => self.set_timer(value, 64),
            0x17 => self.set_timer(value, 1024),
            _ => {}
        }
    }

    fn set_timer(&mut self, value: u8, divider: u16) {
        self.timer = value;
        self.timer_divider = divider;
        self.timer_phase = divider;
        self.timer_irq = false;
    }

    fn save(&self, out: &mut StateWriter) {
        out.blob(&self.ram);
        out.u8(self.swcha);
        out.u8(self.porta_out);
        out.u8(self.swacnt);
        out.u8(self.swchb);
        out.u8(self.portb_out);
        out.u8(self.swbcnt);
        out.u8(self.timer);
        out.u16(self.timer_divider);
        out.u16(self.timer_phase);
        out.u8(self.timer_irq as u8);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let ram = input.blob()?;
        if ram.len() != self.ram.len() {
            return Err("invalid RIOT RAM state length".into());
        }
        self.ram.copy_from_slice(ram);
        self.swcha = input.u8()?;
        self.porta_out = input.u8()?;
        self.swacnt = input.u8()?;
        self.swchb = input.u8()?;
        self.portb_out = input.u8()?;
        self.swbcnt = input.u8()?;
        self.timer = input.u8()?;
        self.timer_divider = input.u16()?;
        self.timer_phase = input.u16()?;
        self.timer_irq = input.u8()? != 0;
        Ok(())
    }
}

#[derive(Clone)]
struct TiaAudioChannel {
    control: u8,
    frequency: u8,
    volume: u8,
    divider: u16,
    output: bool,
    lfsr: u16,
}

impl Default for TiaAudioChannel {
    fn default() -> Self {
        Self {
            control: 0,
            frequency: 0,
            volume: 0,
            divider: 1,
            output: false,
            lfsr: 0x1ff,
        }
    }
}

impl TiaAudioChannel {
    fn tick(&mut self) {
        if self.divider > 1 {
            self.divider -= 1;
            return;
        }
        self.divider = u16::from(self.frequency & 0x1f) + 1;
        match self.control & 0x0f {
            0 => self.output = true,
            1 | 2 | 3 | 8 | 9 | 10 => self.output = !self.output,
            _ => {
                let tap = if self.control & 1 != 0 { 4 } else { 1 };
                let feedback = (self.lfsr ^ (self.lfsr >> tap)) & 1;
                self.lfsr = (self.lfsr >> 1) | (feedback << 8);
                self.output = self.lfsr & 1 != 0;
            }
        }
    }

    fn sample(&self) -> f32 {
        if self.output {
            f32::from(self.volume & 0x0f) / 15.0
        } else {
            0.0
        }
    }

    fn save(&self, out: &mut StateWriter) {
        out.u8(self.control);
        out.u8(self.frequency);
        out.u8(self.volume);
        out.u16(self.divider);
        out.u8(self.output as u8);
        out.u16(self.lfsr);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.control = input.u8()?;
        self.frequency = input.u8()?;
        self.volume = input.u8()?;
        self.divider = input.u16()?;
        self.output = input.u8()? != 0;
        self.lfsr = input.u16()?;
        Ok(())
    }
}

struct Tia {
    regs: [u8; 0x2d],
    collisions: [u8; 8],
    hclock: u16,
    scanline: u16,
    output_y: u16,
    frame: u64,
    vsync_active: bool,
    vsync_lines: u8,
    wsync: bool,
    pos_p0: i16,
    pos_p1: i16,
    pos_m0: i16,
    pos_m1: i16,
    pos_bl: i16,
    fire: [bool; 2],
    lightgun_active: bool,
    lightgun_x: i16,
    lightgun_y: i16,
    paddle_voltage: [f64; 4],
    paddle_resistance: [u32; 4],
    paddle_timestamp: [u64; 4],
    paddle_dumped: bool,
    audio: [TiaAudioChannel; 2],
    sample_phase: u64,
    samples: Vec<f32>,
    video: VideoBuffer,
}

impl Default for Tia {
    fn default() -> Self {
        Self {
            regs: [0; 0x2d],
            collisions: [0; 8],
            hclock: 0,
            scanline: 0,
            output_y: 0,
            frame: 0,
            vsync_active: false,
            vsync_lines: 0,
            wsync: false,
            pos_p0: 0,
            pos_p1: 0,
            pos_m0: 0,
            pos_m1: 0,
            pos_bl: 0,
            fire: [false; 2],
            lightgun_active: false,
            lightgun_x: 0,
            lightgun_y: 0,
            paddle_voltage: [0.0; 4],
            paddle_resistance: [500_000; 4],
            paddle_timestamp: [0; 4],
            paddle_dumped: false,
            audio: [TiaAudioChannel::default(), TiaAudioChannel::default()],
            sample_phase: 0,
            samples: Vec::with_capacity(1024),
            video: VideoBuffer::new(WIDTH, HEIGHT),
        }
    }
}

impl Tia {
    fn reset(&mut self) {
        *self = Self::default();
    }

    fn begin_frame(&mut self) {
        self.samples.clear();
    }

    fn color_clock_timestamp(&self) -> u64 {
        self.frame
            .saturating_mul(262)
            .saturating_add(u64::from(self.scanline))
            .saturating_mul(u64::from(COLOR_CLOCKS_PER_LINE))
            .saturating_add(u64::from(self.hclock))
    }

    fn paddle_threshold_voltage() -> f64 {
        let clock_hz = (CPU_HZ * 3) as f64;
        let elapsed = PADDLE_TRIP_LINES * f64::from(COLOR_CLOCKS_PER_LINE) / clock_hz;
        PADDLE_SUPPLY_VOLTAGE
            * (1.0
                - (-elapsed
                    / (f64::from(PADDLE_MAX_RESISTANCE) + PADDLE_SERIES_RESISTANCE)
                    / PADDLE_CAPACITANCE)
                    .exp())
    }

    fn update_paddle_charge(&mut self, index: usize, timestamp: u64) {
        let elapsed_clocks = timestamp.saturating_sub(self.paddle_timestamp[index]);
        if elapsed_clocks == 0 {
            return;
        }
        let elapsed_seconds = elapsed_clocks as f64 / (CPU_HZ * 3) as f64;
        if self.paddle_dumped {
            self.paddle_voltage[index] *=
                (-elapsed_seconds / PADDLE_DUMP_RESISTANCE / PADDLE_CAPACITANCE).exp();
        } else {
            let resistance = f64::from(self.paddle_resistance[index]) + PADDLE_SERIES_RESISTANCE;
            self.paddle_voltage[index] = PADDLE_SUPPLY_VOLTAGE
                * (1.0
                    - (1.0 - self.paddle_voltage[index] / PADDLE_SUPPLY_VOLTAGE)
                        * (-elapsed_seconds / resistance / PADDLE_CAPACITANCE).exp());
        }
        self.paddle_timestamp[index] = timestamp;
    }

    fn update_all_paddles(&mut self, timestamp: u64) {
        for index in 0..4 {
            self.update_paddle_charge(index, timestamp);
        }
    }

    fn paddle_resistance(axis: i16) -> u32 {
        let normalized = i32::from(axis) - i32::from(i16::MIN);
        ((u64::try_from(normalized).unwrap_or_default() * u64::from(PADDLE_MAX_RESISTANCE))
            / u64::from(u16::MAX)) as u32
    }

    fn pointer_coordinate(axis: i16, extent: u32) -> i16 {
        let normalized = i64::from(axis) - i64::from(i16::MIN);
        let span = i64::from(extent.saturating_sub(1));
        ((normalized * span) / i64::from(u16::MAX)) as i16
    }

    fn lightgun_sensor_high(&self) -> bool {
        if !self.lightgun_active {
            return !self.fire[0];
        }
        const LIGHTGUN_X_OFFSET: i16 = -23;
        const LIGHTGUN_Y_OFFSET: i16 = 1;
        const LIGHTGUN_WINDOW: i16 = 15;

        let mut beam_x =
            i32::from(self.hclock) - i32::from(HBLANK_CLOCKS) + i32::from(LIGHTGUN_X_OFFSET);
        if beam_x < 0 {
            beam_x += i32::from(COLOR_CLOCKS_PER_LINE);
        }
        let beam_y = i32::from(self.output_y) + i32::from(LIGHTGUN_Y_OFFSET);
        let dx = beam_x - i32::from(self.lightgun_x);
        dx < 0 || dx >= i32::from(LIGHTGUN_WINDOW) || beam_y < i32::from(self.lightgun_y)
    }

    fn set_inputs(&mut self, input: &InputState) {
        let timestamp = self.color_clock_timestamp();
        self.update_all_paddles(timestamp);
        self.lightgun_active = input.buttons[0] & POINTER_TOUCH != 0;
        self.lightgun_x = Self::pointer_coordinate(input.axes[0][AXIS_AUX_X], WIDTH);
        self.lightgun_y = Self::pointer_coordinate(input.axes[0][AXIS_AUX_Y], HEIGHT);
        self.fire[0] = input.buttons[0] & FACE_SOUTH != 0;
        self.fire[1] = input.buttons[1] & FACE_SOUTH != 0;
        self.paddle_resistance = [
            Self::paddle_resistance(input.axes[0][AXIS_LEFT_X]),
            Self::paddle_resistance(input.axes[0][AXIS_LEFT_Y]),
            Self::paddle_resistance(input.axes[1][AXIS_LEFT_X]),
            Self::paddle_resistance(input.axes[1][AXIS_LEFT_Y]),
        ];
    }

    fn current_x(&self) -> i16 {
        if self.hclock < HBLANK_CLOCKS {
            0
        } else {
            (self.hclock - HBLANK_CLOCKS).min(WIDTH as u16 - 1) as i16
        }
    }

    fn read(&mut self, address: u16) -> u8 {
        match (address & 0x0f) as usize {
            0..=7 => self.collisions[(address & 7) as usize],
            8..=11 => {
                let index = usize::from((address & 0x0f) - 8);
                let timestamp = self.color_clock_timestamp();
                self.update_paddle_charge(index, timestamp);
                if !self.paddle_dumped
                    && self.paddle_voltage[index] > Self::paddle_threshold_voltage()
                {
                    0x80
                } else {
                    0
                }
            }
            12 => {
                if self.lightgun_sensor_high() {
                    0x80
                } else {
                    0
                }
            }
            13 => {
                if self.fire[1] {
                    0
                } else {
                    0x80
                }
            }
            _ => 0,
        }
    }

    fn write(&mut self, address: u16, value: u8) {
        let reg = (address & 0x3f) as usize;
        if reg >= self.regs.len() {
            return;
        }
        self.regs[reg] = value;
        match reg {
            0x00 => {
                let next = value & 0x02 != 0;
                if next && !self.vsync_active {
                    self.vsync_lines = 0;
                }
                self.vsync_active = next;
            }
            0x01 => {
                let timestamp = self.color_clock_timestamp();
                self.update_all_paddles(timestamp);
                self.paddle_dumped = value & 0x80 != 0;
            }
            0x02 => self.wsync = true,
            0x10 => self.pos_p0 = self.current_x(),
            0x11 => self.pos_p1 = self.current_x(),
            0x12 => self.pos_m0 = self.current_x(),
            0x13 => self.pos_m1 = self.current_x(),
            0x14 => self.pos_bl = self.current_x(),
            0x15 => self.audio[0].control = value,
            0x16 => self.audio[1].control = value,
            0x17 => self.audio[0].frequency = value,
            0x18 => self.audio[1].frequency = value,
            0x19 => self.audio[0].volume = value,
            0x1a => self.audio[1].volume = value,
            0x28 if value & 0x02 != 0 => self.pos_m0 = self.pos_p0,
            0x29 if value & 0x02 != 0 => self.pos_m1 = self.pos_p1,
            0x2a => self.apply_hmove(),
            0x2b => {
                for motion in &mut self.regs[0x20..=0x24] {
                    *motion = 0;
                }
            }
            0x2c => self.collisions = [0; 8],
            _ => {}
        }
    }

    fn apply_hmove(&mut self) {
        self.pos_p0 = wrap_x(self.pos_p0 - motion(self.regs[0x20]));
        self.pos_p1 = wrap_x(self.pos_p1 - motion(self.regs[0x21]));
        self.pos_m0 = wrap_x(self.pos_m0 - motion(self.regs[0x22]));
        self.pos_m1 = wrap_x(self.pos_m1 - motion(self.regs[0x23]));
        self.pos_bl = wrap_x(self.pos_bl - motion(self.regs[0x24]));
    }

    fn tick_cpu_cycles(&mut self, cycles: u32) {
        for _ in 0..cycles {
            for channel in &mut self.audio {
                channel.tick();
            }
            self.sample_phase += SAMPLE_RATE;
            if self.sample_phase >= CPU_HZ {
                self.sample_phase -= CPU_HZ;
                self.samples.push(
                    ((self.audio[0].sample() + self.audio[1].sample()) * 0.4).clamp(0.0, 1.0),
                );
            }
            self.tick_color_clock();
            self.tick_color_clock();
            self.tick_color_clock();
        }
    }

    fn tick_color_clock(&mut self) {
        if self.hclock >= HBLANK_CLOCKS && self.hclock < HBLANK_CLOCKS + WIDTH as u16 {
            let x = usize::from(self.hclock - HBLANK_CLOCKS);
            if self.output_y < HEIGHT as u16 && self.regs[0x01] & 0x02 == 0 && !self.vsync_active {
                self.draw_pixel(x, usize::from(self.output_y));
            }
        }
        self.hclock += 1;
        if self.hclock >= COLOR_CLOCKS_PER_LINE {
            self.hclock = 0;
            self.wsync = false;
            if self.vsync_active {
                self.vsync_lines = self.vsync_lines.saturating_add(1);
            }
            if self.regs[0x01] & 0x02 == 0 && !self.vsync_active && self.output_y < HEIGHT as u16 {
                self.output_y += 1;
            }
            self.scanline += 1;
            if self.scanline >= 262 {
                self.scanline = 0;
                self.output_y = 0;
                self.frame = self.frame.wrapping_add(1);
            }
        }
    }

    fn draw_pixel(&mut self, x: usize, y: usize) {
        let p0 = self.player_on(0, x);
        let p1 = self.player_on(1, x);
        let m0 = self.missile_on(0, x);
        let m1 = self.missile_on(1, x);
        let ball = self.ball_on(x);
        let playfield = self.playfield_on(x);
        self.record_collisions(p0, p1, m0, m1, ball, playfield);

        let playfield_priority = self.regs[0x0a] & 0x04 != 0;
        let score_mode = self.regs[0x0a] & 0x02 != 0;
        let pf_color = if score_mode {
            if x < 80 {
                self.regs[0x06]
            } else {
                self.regs[0x07]
            }
        } else {
            self.regs[0x08]
        };
        let color = if playfield_priority && (playfield || ball) {
            pf_color
        } else if p0 || m0 {
            self.regs[0x06]
        } else if p1 || m1 {
            self.regs[0x07]
        } else if playfield || ball {
            pf_color
        } else {
            self.regs[0x09]
        };
        let rgba = tia_color(color);
        let offset = (y * WIDTH as usize + x) * 4;
        self.video.pixels_mut()[offset..offset + 4].copy_from_slice(&rgba);
    }

    fn player_on(&self, player: usize, x: usize) -> bool {
        let (nusiz, graphics, reflect, position) = if player == 0 {
            (
                self.regs[0x04],
                self.regs[0x1b],
                self.regs[0x0b] & 0x08 != 0,
                self.pos_p0,
            )
        } else {
            (
                self.regs[0x05],
                self.regs[0x1c],
                self.regs[0x0c] & 0x08 != 0,
                self.pos_p1,
            )
        };
        let (offsets, count, scale) = copy_pattern(nusiz & 7);
        for offset in offsets.iter().take(count) {
            let start = wrap_x(position + *offset as i16);
            let local = ((x as i16 - start).rem_euclid(WIDTH as i16)) as usize;
            if local < 8 * scale {
                let source = local / scale;
                let bit = if reflect { source } else { 7 - source };
                if graphics & (1 << bit) != 0 {
                    return true;
                }
            }
        }
        false
    }

    fn missile_on(&self, player: usize, x: usize) -> bool {
        let (nusiz, enabled, locked, player_pos, missile_pos) = if player == 0 {
            (
                self.regs[0x04],
                self.regs[0x1d] & 0x02 != 0,
                self.regs[0x28] & 0x02 != 0,
                self.pos_p0,
                self.pos_m0,
            )
        } else {
            (
                self.regs[0x05],
                self.regs[0x1e] & 0x02 != 0,
                self.regs[0x29] & 0x02 != 0,
                self.pos_p1,
                self.pos_m1,
            )
        };
        if !enabled {
            return false;
        }
        let position = if locked { player_pos } else { missile_pos };
        let width = 1usize << ((nusiz >> 4) & 3);
        let (offsets, count, _) = copy_pattern(nusiz & 7);
        offsets.iter().take(count).any(|offset| {
            let start = wrap_x(position + *offset as i16);
            ((x as i16 - start).rem_euclid(WIDTH as i16) as usize) < width
        })
    }

    fn ball_on(&self, x: usize) -> bool {
        if self.regs[0x1f] & 0x02 == 0 {
            return false;
        }
        let width = 1usize << ((self.regs[0x0a] >> 4) & 3);
        ((x as i16 - self.pos_bl).rem_euclid(WIDTH as i16) as usize) < width
    }

    fn playfield_on(&self, x: usize) -> bool {
        let half_x = x % 80;
        let mut index = half_x / 4;
        if x >= 80 && self.regs[0x0a] & 0x01 != 0 {
            index = 19 - index;
        }
        match index {
            0..=3 => self.regs[0x0d] & (1 << (4 + index)) != 0,
            4..=11 => self.regs[0x0e] & (1 << (11 - index)) != 0,
            12..=19 => self.regs[0x0f] & (1 << (index - 12)) != 0,
            _ => false,
        }
    }

    fn record_collisions(&mut self, p0: bool, p1: bool, m0: bool, m1: bool, bl: bool, pf: bool) {
        if m0 && p1 {
            self.collisions[0] |= 0x80;
        }
        if m0 && p0 {
            self.collisions[0] |= 0x40;
        }
        if m1 && p0 {
            self.collisions[1] |= 0x80;
        }
        if m1 && p1 {
            self.collisions[1] |= 0x40;
        }
        if p0 && pf {
            self.collisions[2] |= 0x80;
        }
        if p0 && bl {
            self.collisions[2] |= 0x40;
        }
        if p1 && pf {
            self.collisions[3] |= 0x80;
        }
        if p1 && bl {
            self.collisions[3] |= 0x40;
        }
        if m0 && pf {
            self.collisions[4] |= 0x80;
        }
        if m0 && bl {
            self.collisions[4] |= 0x40;
        }
        if m1 && pf {
            self.collisions[5] |= 0x80;
        }
        if m1 && bl {
            self.collisions[5] |= 0x40;
        }
        if bl && pf {
            self.collisions[6] |= 0x80;
        }
        if p0 && p1 {
            self.collisions[7] |= 0x80;
        }
        if m0 && m1 {
            self.collisions[7] |= 0x40;
        }
    }

    fn cycles_until_scanline_end(&self) -> u32 {
        let clocks = COLOR_CLOCKS_PER_LINE - self.hclock;
        u32::from(clocks).div_ceil(3)
    }

    fn take_wsync(&mut self) -> bool {
        let value = self.wsync;
        self.wsync = false;
        value
    }

    fn frame(&self) -> u64 {
        self.frame
    }
    fn video(&self) -> &VideoBuffer {
        &self.video
    }
    fn samples(&self) -> &[f32] {
        &self.samples
    }

    fn save(&self, out: &mut StateWriter) {
        out.blob(&self.regs);
        out.blob(&self.collisions);
        out.u16(self.hclock);
        out.u16(self.scanline);
        out.u16(self.output_y);
        out.u64(self.frame);
        out.u8(self.vsync_active as u8);
        out.u8(self.vsync_lines);
        out.u8(self.wsync as u8);
        for position in [
            self.pos_p0,
            self.pos_p1,
            self.pos_m0,
            self.pos_m1,
            self.pos_bl,
        ] {
            out.u16(position as u16);
        }
        out.u8(self.fire[0] as u8);
        out.u8(self.fire[1] as u8);
        out.u8(self.lightgun_active as u8);
        out.u16(self.lightgun_x as u16);
        out.u16(self.lightgun_y as u16);
        for voltage in self.paddle_voltage {
            out.u64(voltage.to_bits());
        }
        for resistance in self.paddle_resistance {
            out.u32(resistance);
        }
        for timestamp in self.paddle_timestamp {
            out.u64(timestamp);
        }
        out.u8(self.paddle_dumped as u8);
        self.audio[0].save(out);
        self.audio[1].save(out);
        out.u64(self.sample_phase);
        out.blob(self.video.pixels());
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let regs = input.blob()?;
        if regs.len() != self.regs.len() {
            return Err("invalid TIA register state length".into());
        }
        self.regs.copy_from_slice(regs);
        let collisions = input.blob()?;
        if collisions.len() != self.collisions.len() {
            return Err("invalid TIA collision state length".into());
        }
        self.collisions.copy_from_slice(collisions);
        self.hclock = input.u16()?;
        self.scanline = input.u16()?;
        self.output_y = input.u16()?;
        self.frame = input.u64()?;
        self.vsync_active = input.u8()? != 0;
        self.vsync_lines = input.u8()?;
        self.wsync = input.u8()? != 0;
        self.pos_p0 = input.u16()? as i16;
        self.pos_p1 = input.u16()? as i16;
        self.pos_m0 = input.u16()? as i16;
        self.pos_m1 = input.u16()? as i16;
        self.pos_bl = input.u16()? as i16;
        self.fire[0] = input.u8()? != 0;
        self.fire[1] = input.u8()? != 0;
        self.lightgun_active = input.u8()? != 0;
        self.lightgun_x = input.u16()? as i16;
        self.lightgun_y = input.u16()? as i16;
        if !(0..WIDTH as i16).contains(&self.lightgun_x)
            || !(0..HEIGHT as i16).contains(&self.lightgun_y)
        {
            return Err("invalid TIA light-gun target".into());
        }
        for voltage in &mut self.paddle_voltage {
            *voltage = f64::from_bits(input.u64()?);
            if !voltage.is_finite() || !(0.0..=PADDLE_SUPPLY_VOLTAGE).contains(voltage) {
                return Err("invalid TIA paddle capacitor voltage".into());
            }
        }
        for resistance in &mut self.paddle_resistance {
            *resistance = input.u32()?;
            if *resistance > PADDLE_MAX_RESISTANCE {
                return Err("invalid TIA paddle resistance".into());
            }
        }
        for timestamp in &mut self.paddle_timestamp {
            *timestamp = input.u64()?;
        }
        self.paddle_dumped = input.u8()? != 0;
        self.audio[0].load(input)?;
        self.audio[1].load(input)?;
        self.sample_phase = input.u64()?;
        let video = input.blob()?;
        if video.len() != self.video.pixels().len() {
            return Err("invalid TIA framebuffer state length".into());
        }
        self.video.pixels_mut().copy_from_slice(video);
        self.samples.clear();
        Ok(())
    }
}

fn motion(value: u8) -> i16 {
    i16::from((value as i8) >> 4)
}

fn wrap_x(value: i16) -> i16 {
    value.rem_euclid(WIDTH as i16)
}

fn copy_pattern(mode: u8) -> ([usize; 3], usize, usize) {
    match mode & 7 {
        0 => ([0, 0, 0], 1, 1),
        1 => ([0, 16, 0], 2, 1),
        2 => ([0, 32, 0], 2, 1),
        3 => ([0, 16, 32], 3, 1),
        4 => ([0, 64, 0], 2, 1),
        5 => ([0, 0, 0], 1, 2),
        6 => ([0, 32, 64], 3, 1),
        _ => ([0, 0, 0], 1, 4),
    }
}

fn tia_color(value: u8) -> [u8; 4] {
    const HUES: [[u8; 3]; 16] = [
        [200, 200, 200],
        [200, 180, 80],
        [220, 140, 60],
        [220, 90, 60],
        [210, 60, 70],
        [190, 70, 150],
        [140, 70, 200],
        [80, 80, 220],
        [60, 110, 220],
        [60, 160, 210],
        [60, 190, 170],
        [70, 190, 100],
        [120, 180, 70],
        [170, 170, 60],
        [180, 130, 70],
        [170, 170, 170],
    ];
    let hue = HUES[usize::from(value >> 4)];
    let luminance = f32::from(value & 0x0e) / 14.0;
    let scale = 0.18 + luminance * 0.82;
    [
        (f32::from(hue[0]) * scale).min(255.0) as u8,
        (f32::from(hue[1]) * scale).min(255.0) as u8,
        (f32::from(hue[2]) * scale).min(255.0) as u8,
        255,
    ]
}

struct AtariBus {
    cartridge: AtariCartridge,
    riot: Riot6532,
    tia: Tia,
}

impl AtariBus {
    fn new(cartridge: AtariCartridge) -> Self {
        Self {
            cartridge,
            riot: Riot6532::default(),
            tia: Tia::default(),
        }
    }

    fn set_inputs(&mut self, input: &InputState) {
        self.riot.set_inputs(input);
        self.tia.set_inputs(input);
    }

    fn tick(&mut self, cycles: u32) {
        self.cartridge.dpc_tick(cycles);
        self.riot.tick(cycles);
        self.tia.tick_cpu_cycles(cycles);
    }
}

impl Bus8 for AtariBus {
    fn read8(&mut self, address: u16) -> u8 {
        let address = address & 0x1fff;
        let value = if address & 0x1000 != 0 {
            self.cartridge.read(address)
        } else if address & 0x0080 == 0 {
            self.tia.read(address)
        } else if address & 0x0200 == 0 {
            self.riot.ram[address as usize & 0x7f]
        } else {
            self.riot.read_io(address)
        };
        self.cartridge.observe_bus_access(address, value, false);
        value
    }

    fn write8(&mut self, address: u16, value: u8) {
        let address = address & 0x1fff;
        if address & 0x1000 != 0 {
            self.cartridge.write(address, value);
        } else if address & 0x0080 == 0 {
            self.tia.write(address, value);
        } else if address & 0x0200 == 0 {
            self.riot.ram[address as usize & 0x7f] = value;
        } else {
            self.riot.write_io(address, value);
        }
        self.cartridge.observe_bus_access(address, value, true);
    }
}

pub struct Atari2600Machine {
    cpu: Mos6502,
    bus: AtariBus,
    audio: AudioBuffer,
    powered: bool,
}

impl Atari2600Machine {
    pub fn from_rom(rom: &[u8]) -> Result<Self, String> {
        let mut bus = AtariBus::new(AtariCartridge::new(rom)?);
        let mut cpu = Mos6502::default();
        cpu.reset(&mut bus);
        Ok(Self {
            cpu,
            bus,
            audio: AudioBuffer::new(48_000, 2),
            powered: true,
        })
    }

    fn clock_instruction(&mut self) -> u32 {
        let cycles = self.cpu.step(&mut self.bus);
        if cycles == 0 {
            return 0;
        }
        self.bus.tick(cycles);
        if self.bus.tia.take_wsync() {
            let stall = self.bus.tia.cycles_until_scanline_end();
            self.cpu.cycles = self.cpu.cycles.saturating_add(u64::from(stall));
            self.bus.tick(stall);
        }
        cycles
    }

    fn flush_audio(&mut self) {
        self.audio.begin_frame();
        for sample in self.bus.tia.samples() {
            self.audio.push_stereo(*sample, *sample);
        }
    }

    fn save_cpu(&self, out: &mut StateWriter) {
        out.u16(self.cpu.pc);
        out.u8(self.cpu.sp);
        out.u8(self.cpu.a);
        out.u8(self.cpu.x);
        out.u8(self.cpu.y);
        out.u8(self.cpu.p);
        out.u64(self.cpu.cycles);
        out.u8(self.cpu.stopped as u8);
    }

    fn load_cpu(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.cpu.pc = input.u16()?;
        self.cpu.sp = input.u8()?;
        self.cpu.a = input.u8()?;
        self.cpu.x = input.u8()?;
        self.cpu.y = input.u8()?;
        self.cpu.p = input.u8()?;
        self.cpu.cycles = input.u64()?;
        self.cpu.stopped = input.u8()? != 0;
        Ok(())
    }
}

impl Machine for Atari2600Machine {
    fn platform(&self) -> PlatformId {
        PlatformId::Atari2600
    }

    fn reset(&mut self) {
        self.bus.riot.reset();
        self.bus.tia.reset();
        self.bus.cartridge.reset();
        self.cpu.reset(&mut self.bus);
        self.powered = true;
        self.audio.begin_frame();
    }

    fn run_frame(&mut self, input: &InputState) {
        self.bus.set_inputs(input);
        self.bus.tia.begin_frame();
        let target = self.bus.tia.frame().wrapping_add(1);
        let deadline = self
            .cpu
            .cycles
            .saturating_add((CPU_HZ as f64 / FRAME_RATE * 2.0).ceil() as u64);
        while self.bus.tia.frame() != target && self.cpu.cycles < deadline {
            if self.clock_instruction() == 0 {
                self.powered = false;
                break;
            }
        }
        if self.bus.tia.frame() != target {
            self.powered = false;
        }
        self.flush_audio();
    }

    fn frame_rate(&self) -> f64 {
        FRAME_RATE
    }
    fn video(&self) -> &VideoBuffer {
        self.bus.tia.video()
    }
    fn audio(&self) -> &AudioBuffer {
        &self.audio
    }

    fn save_state(&self) -> Result<Vec<u8>, String> {
        let mut out = StateWriter::new(PlatformId::Atari2600, STATE_VERSION);
        self.save_cpu(&mut out);
        self.bus.cartridge.save(&mut out);
        self.bus.riot.save(&mut out);
        self.bus.tia.save(&mut out);
        out.u8(self.powered as u8);
        Ok(out.finish())
    }

    fn load_state(&mut self, bytes: &[u8]) -> Result<(), String> {
        let mut input = StateReader::new(bytes, PlatformId::Atari2600, STATE_VERSION)?;
        self.load_cpu(&mut input)?;
        self.bus.cartridge.load(&mut input)?;
        self.bus.riot.load(&mut input)?;
        self.bus.tia.load(&mut input)?;
        self.powered = input.u8()? != 0;
        self.audio.begin_frame();
        input.finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn synthetic_4k_rom() -> Vec<u8> {
        let mut rom = vec![0xea; 0x1000];
        let program: &[u8] = &[
            0x78, 0xd8, 0xa2, 0xff, 0x9a, 0xa9, 0x00, 0x85, 0x01, 0xa9, 0x2e, 0x85, 0x09, 0xa9,
            0x4e, 0x85, 0x08, 0xa9, 0xf0, 0x85, 0x0d, 0xa9, 0xff, 0x85, 0x0e, 0x85, 0x0f, 0xa9,
            0x04, 0x85, 0x15, 0xa9, 0x08, 0x85, 0x17, 0xa9, 0x0f, 0x85, 0x19, 0xa9, 0x00, 0x85,
            0x02, 0x4c, 0x27, 0xf0,
        ];
        rom[..program.len()].copy_from_slice(program);
        rom[0x0ffa..0x1000].copy_from_slice(&[0x00, 0xf0, 0x00, 0xf0, 0x00, 0xf0]);
        rom
    }

    #[test]
    fn paddle_rc_inputs_follow_vblank_dump_resistance_and_state() {
        let mut tia = Tia::default();
        let mut input = InputState::default();
        input.axes[0][AXIS_LEFT_X] = i16::MIN;
        input.axes[0][AXIS_LEFT_Y] = i16::MAX;
        tia.set_inputs(&input);

        tia.write(0x01, 0x80);
        assert_eq!(tia.read(0x08), 0);
        assert_eq!(tia.read(0x09), 0);

        tia.write(0x01, 0x00);
        tia.tick_cpu_cycles(100);
        assert_eq!(tia.read(0x08), 0x80);
        assert_eq!(tia.read(0x09), 0);

        let mut out = StateWriter::new(PlatformId::Atari2600, STATE_VERSION);
        tia.save(&mut out);
        let state = out.finish();

        tia.tick_cpu_cycles(29_000);
        assert_eq!(tia.read(0x09), 0x80);
        tia.write(0x01, 0x80);
        assert_eq!(tia.read(0x08), 0);
        assert_eq!(tia.read(0x09), 0);

        let mut restored = Tia::default();
        let mut reader = StateReader::new(&state, PlatformId::Atari2600, STATE_VERSION).unwrap();
        restored.load(&mut reader).unwrap();
        reader.finish().unwrap();

        assert_eq!(restored.paddle_resistance[0], 0);
        assert_eq!(restored.paddle_resistance[1], PADDLE_MAX_RESISTANCE);
        assert!(!restored.paddle_dumped);
        assert_eq!(restored.read(0x08), 0x80);
        assert_eq!(restored.read(0x09), 0);
    }

    #[test]
    fn xg1_lightgun_uses_swcha_trigger_and_tia_beam_window() {
        let mut tia = Tia::default();
        let mut riot = Riot6532::default();
        let mut input = InputState::default();
        input.buttons[0] = POINTER_TOUCH | POINTER_CLICK;
        input.axes[0][AXIS_AUX_X] = i16::MIN;
        input.axes[0][AXIS_AUX_Y] = i16::MAX;
        tia.set_inputs(&input);
        riot.set_inputs(&input);

        assert_eq!(riot.read_io(0x00) & 0x10, 0);

        tia.output_y = HEIGHT as u16 - 2;
        tia.hclock = HBLANK_CLOCKS + 22;
        assert_eq!(tia.read(0x0c), 0x80);
        tia.hclock = HBLANK_CLOCKS + 23;
        assert_eq!(tia.read(0x0c), 0);
        tia.hclock = HBLANK_CLOCKS + 38;
        assert_eq!(tia.read(0x0c), 0x80);

        tia.hclock = HBLANK_CLOCKS + 23;
        tia.output_y = HEIGHT as u16 - 3;
        assert_eq!(tia.read(0x0c), 0x80);
        tia.output_y = HEIGHT as u16 - 2;
        assert_eq!(tia.read(0x0c), 0);

        input.buttons[0] = POINTER_TOUCH;
        riot.set_inputs(&input);
        assert_ne!(riot.read_io(0x00) & 0x10, 0);

        let mut out = StateWriter::new(PlatformId::Atari2600, STATE_VERSION);
        tia.save(&mut out);
        let state = out.finish();
        let mut restored = Tia::default();
        let mut reader = StateReader::new(&state, PlatformId::Atari2600, STATE_VERSION).unwrap();
        restored.load(&mut reader).unwrap();
        reader.finish().unwrap();
        assert!(restored.lightgun_active);
        assert_eq!(restored.lightgun_x, 0);
        assert_eq!(restored.lightgun_y, HEIGHT as i16 - 1);
        assert_eq!(restored.read(0x0c), 0);
    }

    #[test]
    fn synthetic_kernel_draws_playfield_and_generates_audio() {
        let rom = synthetic_4k_rom();
        let mut machine = Atari2600Machine::from_rom(&rom).unwrap();
        machine.run_frame(&InputState::default());
        assert_eq!(machine.video().width(), WIDTH);
        assert_eq!(machine.video().height(), HEIGHT);
        assert!(machine.video().pixels().iter().any(|value| *value != 0));
        assert!(machine.audio().samples().iter().any(|sample| *sample > 0.0));
        assert!(machine.powered);
    }

    #[test]
    fn f8_hotspots_switch_4k_banks() {
        let mut rom = vec![0x11; 0x2000];
        rom[1] = 0x10;
        rom[0x1000..].fill(0x22);
        let mut cart = AtariCartridge::new(&rom).unwrap();
        assert_eq!(cart.read(0x1000), 0x22);
        let _ = cart.read(0x1ff8);
        assert_eq!(cart.read(0x1000), 0x11);
        let _ = cart.read(0x1ff9);
        assert_eq!(cart.read(0x1000), 0x22);
    }

    #[test]
    fn superchip_autodetection_maps_split_ram_window() {
        let mut rom = vec![0u8; 0x2000];
        rom[0x0200] = 0x11;
        rom[0x1200] = 0x22;
        let mut cart = AtariCartridge::new(&rom).unwrap();
        assert_eq!(cart.scheme, CartScheme::F8Sc);
        cart.write(0x1000, 0x5a);
        assert_eq!(cart.read(0x1080), 0x5a);
        let _ = cart.read(0x1ff8);
        assert_eq!(cart.read(0x1200), 0x11);
        assert_eq!(cart.read(0x1080), 0x5a);
    }

    #[test]
    fn e0_switches_three_independent_one_kib_segments() {
        let mut rom = vec![0u8; 0x2000];
        for bank in 0..8usize {
            rom[bank * 0x400..(bank + 1) * 0x400].fill(bank as u8);
        }
        rom[0x0080..0x0083].copy_from_slice(&[0x8d, 0xe0, 0x1f]);
        let mut cart = AtariCartridge::new(&rom).unwrap();
        assert_eq!(cart.scheme, CartScheme::E0);
        assert_eq!(
            (cart.read(0x1000), cart.read(0x1400), cart.read(0x1800)),
            (4, 5, 6)
        );
        let _ = cart.read(0x1fe2);
        let _ = cart.read(0x1feb);
        let _ = cart.read(0x1ff1);
        assert_eq!(
            (cart.read(0x1000), cart.read(0x1400), cart.read(0x1800)),
            (2, 3, 1)
        );
        assert_eq!(cart.read(0x1c00), 7);
    }

    #[test]
    fn three_f_snoops_tia_writes_and_keeps_last_two_kib_fixed() {
        let mut rom = vec![0u8; 0x2000];
        for bank in 0..4usize {
            rom[bank * 0x800..(bank + 1) * 0x800].fill((0x10 + bank) as u8);
        }
        rom[0x0080..0x0084].copy_from_slice(&[0x85, 0x3f, 0x85, 0x3f]);
        let cart = AtariCartridge::new(&rom).unwrap();
        assert_eq!(cart.scheme, CartScheme::ThreeF);
        let mut bus = AtariBus::new(cart);
        bus.write8(0x003f, 2);
        assert_eq!(bus.read8(0x1000), 0x12);
        assert_eq!(bus.read8(0x1800), 0x13);
    }

    #[test]
    fn sb_hotspots_select_low_address_bank_for_128k_and_256k_images() {
        for bank_count in [32usize, 64usize] {
            let mut rom = vec![0u8; bank_count * 0x1000];
            for bank in 0..bank_count {
                rom[bank * 0x1000..(bank + 1) * 0x1000].fill(bank as u8);
            }
            rom[0x0200..0x0203].copy_from_slice(&[0xad, 0x00, 0x08]);

            let cart = AtariCartridge::new(&rom).unwrap();
            assert_eq!(cart.scheme, CartScheme::Sb);
            let mut bus = AtariBus::new(cart);
            assert_eq!(bus.read8(0x1000), (bank_count - 1) as u8);

            let top_hotspot = 0x0800u16 | (bank_count as u16 - 1);
            let _ = bus.read8(top_hotspot);
            assert_eq!(bus.read8(0x1000), (bank_count - 1) as u8);

            let _ = bus.read8(0x0a05);
            assert_eq!(bus.read8(0x1000), 5);

            let mut out = StateWriter::new(PlatformId::Atari2600, STATE_VERSION);
            bus.cartridge.save(&mut out);
            let state = out.finish();

            bus.write8(0x0812, 0);
            assert_eq!(bus.read8(0x1000), 0x12 & (bank_count as u8 - 1));

            let mut input = StateReader::new(&state, PlatformId::Atari2600, STATE_VERSION).unwrap();
            bus.cartridge.load(&mut input).unwrap();
            input.finish().unwrap();
            assert_eq!(bus.read8(0x1000), 5);

            bus.cartridge.reset();
            assert_eq!(bus.read8(0x1000), (bank_count - 1) as u8);
        }
    }

    #[test]
    fn bf_hotspots_select_sixty_four_four_kib_banks_and_reset_to_one() {
        let mut rom = vec![0u8; 0x40000];
        for bank in 0..64usize {
            rom[bank * 0x1000..(bank + 1) * 0x1000].fill(bank as u8);
        }
        let tail = rom.len() - 8;
        rom[tail..tail + 4].copy_from_slice(b"BFBF");

        let mut cart = AtariCartridge::new(&rom).unwrap();
        assert_eq!(cart.scheme, CartScheme::Bf);
        assert_eq!(cart.read(0x1000), 1);

        let _ = cart.read(0x1f80);
        assert_eq!(cart.read(0x1000), 0);

        let _ = cart.read(0x1fbf);
        assert_eq!(cart.read(0x1000), 63);

        cart.reset();
        assert_eq!(cart.read(0x1000), 1);
    }

    #[test]
    fn bfsc_hotspots_keep_split_ram_across_sixty_four_rom_banks() {
        let mut rom = vec![0u8; 0x40000];
        for bank in 0..64usize {
            rom[bank * 0x1000..(bank + 1) * 0x1000].fill((0x80u16 + bank as u16) as u8);
        }
        let tail = rom.len() - 8;
        rom[tail..tail + 4].copy_from_slice(b"BFSC");

        let mut cart = AtariCartridge::new(&rom).unwrap();
        assert_eq!(cart.scheme, CartScheme::BfSc);
        assert_eq!(cart.read(0x1200), 0x81);

        cart.write(0x1000, 0x5a);
        assert_eq!(cart.read(0x1080), 0x5a);

        cart.write(0x1fbf, 0);
        assert_eq!(cart.read(0x1200), 0xbf);
        assert_eq!(cart.read(0x1080), 0x5a);

        cart.write(0x1f82, 0);
        assert_eq!(cart.read(0x1200), 0x82);
        assert_eq!(cart.read(0x1080), 0x5a);

        cart.reset();
        assert_eq!(cart.read(0x1200), 0x81);
    }

    #[test]
    fn dpc_fetchers_rng_music_and_state_round_trip() {
        let mut rom = vec![0u8; 0x2800];
        rom[..0x1000].fill(0x11);
        rom[0x1000..0x2000].fill(0x22);
        rom[0x2000..].fill(0x33);
        rom[0x2000 + 2044] = 0xa5;

        let mut cart = AtariCartridge::new(&rom).unwrap();
        assert_eq!(cart.scheme, CartScheme::Dpc);
        assert_eq!(cart.read(0x1100), 0x22);

        let _ = cart.read(0x1ff8);
        assert_eq!(cart.read(0x1100), 0x11);
        let _ = cart.read(0x1ff9);
        assert_eq!(cart.read(0x1100), 0x22);

        cart.write(0x1070, 0);
        assert_eq!(cart.read(0x1000), 3);
        assert_eq!(cart.read(0x1000), 7);

        cart.write(0x1040, 3);
        cart.write(0x1048, 1);
        cart.write(0x1050, 3);
        cart.write(0x1058, 0);
        assert_eq!(cart.read(0x1008), 0xa5);
        assert_eq!(cart.read(0x1038), 0xff);
        assert_eq!(cart.read(0x1038), 0x00);

        cart.write(0x1045, 2);
        cart.write(0x104d, 0);
        cart.write(0x105d, 0x10);
        cart.write(0x1055, 0xff);
        assert_eq!(cart.dpc_counters[5] & 0xff, 2);

        cart.dpc_tick(60);
        assert_eq!(cart.dpc_counters[5] & 0xff, 1);
        assert_eq!(cart.read(0x1004), 0x04);

        let saved_random = cart.dpc_random;
        let saved_counter = cart.dpc_counters[5];
        let mut out = StateWriter::new(PlatformId::Atari2600, STATE_VERSION);
        cart.save(&mut out);
        let state = out.finish();

        cart.dpc_tick(60);
        assert_eq!(cart.dpc_counters[5] & 0xff, 0);
        assert_eq!(cart.read(0x1004), 0x00);
        cart.write(0x1045, 7);

        let mut input = StateReader::new(&state, PlatformId::Atari2600, STATE_VERSION).unwrap();
        cart.load(&mut input).unwrap();
        input.finish().unwrap();
        assert_eq!(cart.dpc_tops[5], 2);
        assert!(cart.dpc_music_mode[0]);
        assert_eq!(cart.dpc_counters[5], saved_counter);
        assert_eq!(cart.dpc_random, saved_random);

        cart.reset();
        assert_eq!(cart.bank, 1);
        assert_eq!(cart.dpc_random, 1);
        assert_eq!(cart.dpc_counters, [0; 8]);
    }

    #[test]
    fn df_hotspots_select_thirty_two_four_kib_banks_and_reset_to_fifteen() {
        let mut rom = vec![0u8; 0x20000];
        for bank in 0..32usize {
            rom[bank * 0x1000..(bank + 1) * 0x1000].fill((0x40 + bank) as u8);
        }
        let tail = rom.len() - 8;
        rom[tail..tail + 4].copy_from_slice(b"DFDF");

        let mut cart = AtariCartridge::new(&rom).unwrap();
        assert_eq!(cart.scheme, CartScheme::Df);
        assert_eq!(cart.read(0x1000), 0x4f);

        let _ = cart.read(0x1fc0);
        assert_eq!(cart.read(0x1000), 0x40);

        let _ = cart.read(0x1fdf);
        assert_eq!(cart.read(0x1000), 0x5f);

        cart.reset();
        assert_eq!(cart.read(0x1000), 0x4f);
    }

    #[test]
    fn dfsc_hotspots_keep_split_ram_independent_of_rom_bank() {
        let mut rom = vec![0u8; 0x20000];
        for bank in 0..32usize {
            rom[bank * 0x1000..(bank + 1) * 0x1000].fill((0x20 + bank) as u8);
        }
        let tail = rom.len() - 8;
        rom[tail..tail + 4].copy_from_slice(b"DFSC");

        let mut cart = AtariCartridge::new(&rom).unwrap();
        assert_eq!(cart.scheme, CartScheme::DfSc);
        assert_eq!(cart.read(0x1200), 0x2f);

        cart.write(0x1000, 0x5a);
        assert_eq!(cart.read(0x1080), 0x5a);

        cart.write(0x1fdf, 0);
        assert_eq!(cart.read(0x1200), 0x3f);
        assert_eq!(cart.read(0x1080), 0x5a);

        cart.write(0x1fc2, 0);
        assert_eq!(cart.read(0x1200), 0x22);
        assert_eq!(cart.read(0x1080), 0x5a);
    }

    #[test]
    fn ef_hotspots_select_sixteen_four_kib_banks() {
        let mut rom = vec![0u8; 0x10000];
        for bank in 0..16usize {
            let value = (0x60 + bank) as u8;
            rom[bank * 0x1000..(bank + 1) * 0x1000].fill(value);
            rom[bank * 0x1000 + 1] ^= 0x01;
        }
        rom[0x0200..0x0203].copy_from_slice(&[0xad, 0xe0, 0xff]);

        let mut cart = AtariCartridge::new(&rom).unwrap();
        assert_eq!(cart.scheme, CartScheme::Ef);
        assert_eq!(cart.read(0x1000), 0x61);

        let _ = cart.read(0x1fe3);
        assert_eq!(cart.read(0x1000), 0x63);

        cart.write(0x1fe0, 0);
        assert_eq!(cart.read(0x1000), 0x60);

        cart.reset();
        assert_eq!(cart.read(0x1000), 0x61);
    }

    #[test]
    fn efsc_hotspots_preserve_split_ram_across_banks_and_state() {
        let mut rom = vec![0u8; 0x10000];
        for bank in 0..16usize {
            rom[bank * 0x1000..(bank + 1) * 0x1000].fill((0x70 + bank) as u8);
        }
        rom[0x0200..0x0203].copy_from_slice(&[0xad, 0xe0, 0xff]);

        let mut cart = AtariCartridge::new(&rom).unwrap();
        assert_eq!(cart.scheme, CartScheme::EfSc);
        assert_eq!(cart.read(0x1200), 0x71);

        cart.write(0x1000, 0x5a);
        assert_eq!(cart.read(0x1080), 0x5a);

        let _ = cart.read(0x1fe4);
        assert_eq!(cart.read(0x1200), 0x74);
        assert_eq!(cart.read(0x1080), 0x5a);

        let mut out = StateWriter::new(PlatformId::Atari2600, STATE_VERSION);
        cart.save(&mut out);
        let state = out.finish();

        let _ = cart.read(0x1fe1);
        cart.write(0x1000, 0xa5);
        assert_eq!(cart.read(0x1200), 0x71);

        let mut input = StateReader::new(&state, PlatformId::Atari2600, STATE_VERSION).unwrap();
        cart.load(&mut input).unwrap();
        input.finish().unwrap();
        assert_eq!(cart.bank, 4);
        assert_eq!(cart.read(0x1080), 0x5a);
        assert_eq!(cart.read(0x1200), 0x74);
    }

    #[test]
    fn three_e_plus_maps_four_independent_rom_and_ram_segments() {
        let mut rom = vec![0u8; 0x10000];
        for bank in 0..64usize {
            rom[bank * 0x400..(bank + 1) * 0x400].fill(bank as u8);
        }
        rom[0x0080..0x0084].copy_from_slice(b"TJ3E");

        let cart = AtariCartridge::new(&rom).unwrap();
        assert_eq!(cart.scheme, CartScheme::ThreeEPlus);
        let mut bus = AtariBus::new(cart);

        assert_eq!(bus.read8(0x1000), 0);
        assert_eq!(bus.read8(0x1c00), 0);

        bus.write8(0x003f, 0x45);
        bus.write8(0x003f, 0xbf);
        bus.write8(0x003f, 0xc7);
        assert_eq!(bus.read8(0x1400), 5);
        assert_eq!(bus.read8(0x1800), 63);
        assert_eq!(bus.read8(0x1c00), 7);

        bus.write8(0x003e, 9);
        assert_eq!(bus.read8(0x1000), 0);
        assert_eq!(bus.read8(0x1200), 0xff);
        bus.write8(0x1200, 0x5a);
        assert_eq!(bus.read8(0x1000), 0x5a);

        bus.write8(0x003e, 0x4a);
        bus.write8(0x1600, 0xa5);
        assert_eq!(bus.read8(0x1400), 0xa5);

        bus.write8(0x003e, 10);
        assert_eq!(bus.read8(0x1000), 0xa5);

        bus.write8(0x003f, 4);
        assert_eq!(bus.read8(0x1000), 4);

        let mut out = StateWriter::new(PlatformId::Atari2600, STATE_VERSION);
        bus.cartridge.save(&mut out);
        let state = out.finish();

        bus.write8(0x003f, 1);
        bus.write8(0x003f, 0x41);
        bus.write8(0x003f, 0x81);
        bus.write8(0x003f, 0xc1);
        assert_eq!(bus.read8(0x1000), 1);
        assert_eq!(bus.read8(0x1400), 1);

        let mut input = StateReader::new(&state, PlatformId::Atari2600, STATE_VERSION).unwrap();
        bus.cartridge.load(&mut input).unwrap();
        input.finish().unwrap();
        assert_eq!(bus.read8(0x1000), 4);
        assert_eq!(bus.read8(0x1400), 0xa5);
        assert_eq!(bus.read8(0x1800), 63);
        assert_eq!(bus.read8(0x1c00), 7);

        bus.cartridge.reset();
        assert_eq!(bus.read8(0x1c00), 0);
    }

    #[test]
    fn three_e_switches_rom_and_one_kib_ram_banks_with_fixed_upper_rom() {
        let mut rom = vec![0u8; 0x8000];
        for bank in 0..16usize {
            rom[bank * 0x800..(bank + 1) * 0x800].fill((0x30 + bank) as u8);
        }
        rom[0x0100..0x0104].copy_from_slice(&[0x85, 0x3e, 0xa9, 0x00]);

        let cart = AtariCartridge::new(&rom).unwrap();
        assert_eq!(cart.scheme, CartScheme::ThreeE);
        let mut bus = AtariBus::new(cart);

        assert_eq!(bus.read8(0x1000), 0x30);
        assert_eq!(bus.read8(0x1800), 0x3f);

        bus.write8(0x003f, 5);
        assert_eq!(bus.read8(0x1000), 0x35);
        assert_eq!(bus.read8(0x1800), 0x3f);

        bus.write8(0x003e, 7);
        assert_eq!(bus.read8(0x1000), 0);
        assert_eq!(bus.read8(0x1400), 0xff);
        bus.write8(0x1400, 0x5a);
        assert_eq!(bus.read8(0x1000), 0x5a);

        bus.write8(0x003e, 8);
        assert_eq!(bus.read8(0x1000), 0);
        bus.write8(0x17ff, 0xa5);
        assert_eq!(bus.read8(0x13ff), 0xa5);

        bus.write8(0x003e, 7);
        assert_eq!(bus.read8(0x1000), 0x5a);

        let mut out = StateWriter::new(PlatformId::Atari2600, STATE_VERSION);
        bus.cartridge.save(&mut out);
        let state = out.finish();

        bus.write8(0x003f, 2);
        assert_eq!(bus.read8(0x1000), 0x32);

        let mut input = StateReader::new(&state, PlatformId::Atari2600, STATE_VERSION).unwrap();
        bus.cartridge.load(&mut input).unwrap();
        input.finish().unwrap();
        assert!(bus.cartridge.ram_selected);
        assert_eq!(bus.cartridge.ram_bank, 7);
        assert_eq!(bus.read8(0x1000), 0x5a);
        assert_eq!(bus.read8(0x1800), 0x3f);
    }

    #[test]
    fn ua_hotspots_snoop_low_address_bus_cycles() {
        let mut rom = vec![0u8; 0x2000];
        rom[..0x1000].fill(0x11);
        rom[0x1000..].fill(0x22);
        rom[0x0080..0x0083].copy_from_slice(&[0x8d, 0x40, 0x02]);
        let cart = AtariCartridge::new(&rom).unwrap();
        assert_eq!(cart.scheme, CartScheme::Ua);
        let mut bus = AtariBus::new(cart);
        assert_eq!(bus.read8(0x1000), 0x11);
        let _ = bus.read8(0x0240);
        assert_eq!(bus.read8(0x1000), 0x22);
        let _ = bus.read8(0x0220);
        assert_eq!(bus.read8(0x1000), 0x11);
    }

    #[test]
    fn fe_uses_next_bus_cycle_data_bit_five_for_bank_select() {
        let mut rom = vec![0u8; 0x2000];
        rom[..0x1000].fill(0x11);
        rom[0x1000..].fill(0x22);
        rom[0x0080..0x0085].copy_from_slice(&[0x20, 0xc3, 0xf8, 0xa5, 0x82]);
        let cart = AtariCartridge::new(&rom).unwrap();
        assert_eq!(cart.scheme, CartScheme::Fe);
        let mut bus = AtariBus::new(cart);
        assert_eq!(bus.read8(0x1000), 0x11);
        let _ = bus.read8(0x01fe);
        assert_eq!(bus.read8(0x1000), 0x11);
        assert_eq!(bus.read8(0x1000), 0x22);
        let _ = bus.read8(0x01fe);
        bus.write8(0x0080, 0x20);
        assert_eq!(bus.read8(0x1000), 0x11);
    }

    #[test]
    fn e7_switches_rom_and_banked_ram_windows() {
        let mut rom = vec![0u8; 0x4000];
        for bank in 0..8usize {
            rom[bank * 0x800..(bank + 1) * 0x800].fill((0x20 + bank) as u8);
        }
        rom[0x0080..0x0083].copy_from_slice(&[0xad, 0xe7, 0x1f]);
        let mut cart = AtariCartridge::new(&rom).unwrap();
        assert_eq!(cart.scheme, CartScheme::E7);
        assert_eq!(cart.read(0x1000), 0x20);
        assert_eq!(cart.read(0x1a00), 0x27);
        let _ = cart.read(0x1fe3);
        assert_eq!(cart.read(0x1000), 0x23);
        let _ = cart.read(0x1fe7);
        cart.write(0x1000, 0x5a);
        assert_eq!(cart.read(0x1400), 0x5a);
        let _ = cart.read(0x1fea);
        cart.write(0x1800, 0x6b);
        assert_eq!(cart.read(0x1900), 0x6b);
    }

    #[test]
    fn fa_switches_three_banks_and_maps_ram_plus_window() {
        let mut rom = vec![0u8; 0x3000];
        for bank in 0..3usize {
            rom[bank * 0x1000..(bank + 1) * 0x1000].fill((0x30 + bank) as u8);
        }
        let mut cart = AtariCartridge::new(&rom).unwrap();
        assert_eq!(cart.scheme, CartScheme::Fa);
        assert_eq!(cart.read(0x1200), 0x32);
        cart.write(0x1000, 0x71);
        assert_eq!(cart.read(0x1100), 0x71);
        let _ = cart.read(0x1ff8);
        assert_eq!(cart.read(0x1200), 0x30);
        let _ = cart.read(0x1ffa);
        assert_eq!(cart.read(0x1200), 0x32);
    }

    #[test]
    fn f0_advances_sequential_four_kib_banks() {
        let mut rom = vec![0u8; 0x10000];
        for bank in 0..16usize {
            rom[bank * 0x1000..(bank + 1) * 0x1000].fill((0x40 + bank) as u8);
        }
        let mut cart = AtariCartridge::new(&rom).unwrap();
        assert_eq!(cart.scheme, CartScheme::F0);
        assert_eq!(cart.read(0x1000), 0x4f);
        let _ = cart.read(0x1ff0);
        assert_eq!(cart.read(0x1000), 0x40);
        let _ = cart.read(0x1ff0);
        assert_eq!(cart.read(0x1000), 0x41);
    }

    #[test]
    fn cartridge_state_round_trip_preserves_banking_and_ram() {
        let mut rom = vec![0u8; 0x4000];
        for bank in 0..8usize {
            rom[bank * 0x800..(bank + 1) * 0x800].fill((0x50 + bank) as u8);
        }
        rom[0x0080..0x0083].copy_from_slice(&[0xad, 0xe7, 0x1f]);
        let mut cart = AtariCartridge::new(&rom).unwrap();
        let _ = cart.read(0x1fe7);
        let _ = cart.read(0x1feb);
        cart.write(0x1000, 0xa5);
        cart.write(0x1800, 0x5a);
        let mut out = StateWriter::new(PlatformId::Atari2600, STATE_VERSION);
        cart.save(&mut out);
        let state = out.finish();

        cart.reset();
        let mut input = StateReader::new(&state, PlatformId::Atari2600, STATE_VERSION).unwrap();
        cart.load(&mut input).unwrap();
        input.finish().unwrap();
        assert_eq!(cart.bank, 7);
        assert_eq!(cart.ram_bank, 3);
        assert_eq!(cart.read(0x1400), 0xa5);
        assert_eq!(cart.read(0x1900), 0x5a);
    }

    #[test]
    fn save_state_round_trip_restores_cpu_tia_and_riot() {
        let rom = synthetic_4k_rom();
        let mut machine = Atari2600Machine::from_rom(&rom).unwrap();
        machine.run_frame(&InputState::default());
        machine.bus.riot.ram[7] = 0xa5;
        let saved = machine.save_state().unwrap();
        let pc = machine.cpu.pc;
        let frame = machine.bus.tia.frame();
        machine.run_frame(&InputState::default());
        machine.bus.riot.ram[7] = 0;
        machine.load_state(&saved).unwrap();
        assert_eq!(machine.cpu.pc, pc);
        assert_eq!(machine.bus.tia.frame(), frame);
        assert_eq!(machine.bus.riot.ram[7], 0xa5);
    }
}
