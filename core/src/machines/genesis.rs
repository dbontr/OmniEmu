use crate::cpu68000::{Bus68000, M68000};
use crate::cpu_z80::{Z80Bus, Z80};
use crate::input::{
    DOWN, FACE_EAST, FACE_NORTH, FACE_SOUTH, FACE_WEST, L1, LEFT, R1, RIGHT, SELECT, START, UP,
};
use crate::kernel::{AudioBuffer, InputState, VideoBuffer};
use crate::machine::Machine;
use crate::platform::PlatformId;
use crate::resources::ResourceKind;
use crate::state::{StateReader, StateWriter};

use super::genesis_vdp::GenesisVdp;
use super::sn76489::Sn76489;
use super::ym2612::Ym2612;

const NTSC_M68K_HZ: u64 = 7_670_454;
const PAL_M68K_HZ: u64 = 7_600_489;
const NTSC_Z80_HZ: u64 = 3_579_545;
const PAL_Z80_HZ: u64 = 3_546_893;
const CPU_CYCLES_PER_LINE: u64 = 488;
pub(super) const M68K_HZ: u64 = NTSC_M68K_HZ;
pub(super) const FRAME_RATE: f64 = 59.922_743;
const STATE_VERSION: u32 = 11;
const MAX_ROM: usize = 8 * 1024 * 1024;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum GenesisRegion {
    Ntsc,
    Pal,
}

impl GenesisRegion {
    fn m68k_hz(self) -> u64 {
        match self {
            Self::Ntsc => NTSC_M68K_HZ,
            Self::Pal => PAL_M68K_HZ,
        }
    }

    fn z80_hz(self) -> u64 {
        match self {
            Self::Ntsc => NTSC_Z80_HZ,
            Self::Pal => PAL_Z80_HZ,
        }
    }

    fn scanlines(self) -> u64 {
        match self {
            Self::Ntsc => 262,
            Self::Pal => 313,
        }
    }

    fn frame_rate(self) -> f64 {
        self.m68k_hz() as f64 / (CPU_CYCLES_PER_LINE * self.scanlines()) as f64
    }

    fn version_byte(self) -> u8 {
        0xa0 | if self == Self::Pal { 0x40 } else { 0 }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum GenesisEepromProfile {
    SegaX24c01,
    EaX24c01,
    Acclaim16X24c02,
    Acclaim32C02,
    Acclaim32C04,
    Acclaim32C16,
    Acclaim32C65,
    JcartC08,
    JcartC16,
    JcartC65,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum GenesisEepromState {
    Standby,
    WaitStop,
    Address,
    DeviceAddress,
    WordAddressHigh,
    WordAddressLow,
    WriteData,
    ReadData,
}

struct GenesisEeprom {
    profile: GenesisEepromProfile,
    data: Vec<u8>,
    state: GenesisEepromState,
    cycles: u8,
    read: bool,
    device_address: u16,
    word_address: u16,
    buffer: u8,
    scl: bool,
    sda: bool,
    old_scl: bool,
    old_sda: bool,
    acclaim_rom_enabled: bool,
}

impl GenesisEeprom {
    fn address_bits_for(profile: GenesisEepromProfile) -> u8 {
        match profile {
            GenesisEepromProfile::SegaX24c01 | GenesisEepromProfile::EaX24c01 => 7,
            GenesisEepromProfile::Acclaim32C65 | GenesisEepromProfile::JcartC65 => 16,
            _ => 8,
        }
    }

    fn size_mask_for(profile: GenesisEepromProfile) -> u16 {
        match profile {
            GenesisEepromProfile::SegaX24c01 | GenesisEepromProfile::EaX24c01 => 0x007f,
            GenesisEepromProfile::Acclaim16X24c02 | GenesisEepromProfile::Acclaim32C02 => 0x00ff,
            GenesisEepromProfile::Acclaim32C04 => 0x01ff,
            GenesisEepromProfile::JcartC08 => 0x03ff,
            GenesisEepromProfile::Acclaim32C16 | GenesisEepromProfile::JcartC16 => 0x07ff,
            GenesisEepromProfile::Acclaim32C65 | GenesisEepromProfile::JcartC65 => 0x1fff,
        }
    }

    fn page_mask_for(profile: GenesisEepromProfile) -> u16 {
        match profile {
            GenesisEepromProfile::SegaX24c01
            | GenesisEepromProfile::EaX24c01
            | GenesisEepromProfile::Acclaim16X24c02 => 0x0003,
            GenesisEepromProfile::Acclaim32C02 => 0x0007,
            GenesisEepromProfile::Acclaim32C04
            | GenesisEepromProfile::Acclaim32C16
            | GenesisEepromProfile::JcartC08
            | GenesisEepromProfile::JcartC16 => 0x000f,
            GenesisEepromProfile::Acclaim32C65 | GenesisEepromProfile::JcartC65 => 0x003f,
        }
    }

    fn new(profile: GenesisEepromProfile) -> Self {
        let size = usize::from(Self::size_mask_for(profile)) + 1;
        Self {
            profile,
            data: vec![0xff; size],
            state: GenesisEepromState::Standby,
            cycles: 0,
            read: false,
            device_address: 0,
            word_address: 0,
            buffer: 0,
            scl: true,
            sda: true,
            old_scl: true,
            old_sda: true,
            acclaim_rom_enabled: matches!(
                profile,
                GenesisEepromProfile::Acclaim32C02
                    | GenesisEepromProfile::Acclaim32C04
                    | GenesisEepromProfile::Acclaim32C16
                    | GenesisEepromProfile::Acclaim32C65
            ),
        }
    }

    fn address_bits(&self) -> u8 {
        Self::address_bits_for(self.profile)
    }

    fn size_mask(&self) -> u16 {
        Self::size_mask_for(self.profile)
    }

    fn page_mask(&self) -> u16 {
        Self::page_mask_for(self.profile)
    }

    fn is_acclaim32(&self) -> bool {
        matches!(
            self.profile,
            GenesisEepromProfile::Acclaim32C02
                | GenesisEepromProfile::Acclaim32C04
                | GenesisEepromProfile::Acclaim32C16
                | GenesisEepromProfile::Acclaim32C65
        )
    }

    fn is_jcart(&self) -> bool {
        matches!(
            self.profile,
            GenesisEepromProfile::JcartC08
                | GenesisEepromProfile::JcartC16
                | GenesisEepromProfile::JcartC65
        )
    }

    fn reset_protocol(&mut self) {
        self.state = GenesisEepromState::Standby;
        self.cycles = 0;
        self.read = false;
        self.device_address = 0;
        self.word_address = 0;
        self.buffer = 0;
        self.scl = true;
        self.sda = true;
        self.old_scl = true;
        self.old_sda = true;
        self.acclaim_rom_enabled = self.is_acclaim32();
    }

    fn input_bits(&self) -> (u8, u8, u8) {
        match self.profile {
            GenesisEepromProfile::SegaX24c01 => (1, 0, 0),
            GenesisEepromProfile::EaX24c01 => (6, 7, 7),
            GenesisEepromProfile::Acclaim16X24c02 => (1, 0, 1),
            GenesisEepromProfile::Acclaim32C02
            | GenesisEepromProfile::Acclaim32C04
            | GenesisEepromProfile::Acclaim32C16
            | GenesisEepromProfile::Acclaim32C65 => (0, 0, 0),
            GenesisEepromProfile::JcartC08
            | GenesisEepromProfile::JcartC16
            | GenesisEepromProfile::JcartC65 => (1, 0, 7),
        }
    }

    fn read_mapped(&self, address: u32) -> bool {
        if self.is_acclaim32() {
            !self.acclaim_rom_enabled && (0x200000..=0x2fffff).contains(&address)
        } else if self.is_jcart() {
            (0x380000..=0x3fffff).contains(&address)
        } else {
            (0x200000..=0x3fffff).contains(&address)
        }
    }

    fn write_mapped(&self, address: u32) -> bool {
        if self.is_acclaim32() {
            (0x200000..=0x2fffff).contains(&address)
        } else if self.is_jcart() {
            (0x300000..=0x37ffff).contains(&address)
        } else {
            (0x200000..=0x3fffff).contains(&address)
        }
    }

    fn output_bit(&self) -> bool {
        if self.state == GenesisEepromState::ReadData && (1..9).contains(&self.cycles) {
            let shift = 8 - self.cycles;
            let address = (self.device_address | self.word_address) & self.size_mask();
            return self.data[usize::from(address)] & (1 << shift) != 0;
        }
        if self.cycles == 9 {
            return false;
        }
        self.sda
    }

    fn read_byte(&self, address: u32) -> Option<u8> {
        if !self.read_mapped(address) {
            return None;
        }
        if address & 1 == 0 {
            return Some(0xff);
        }
        let (_, _, out_bit) = self.input_bits();
        Some(u8::from(self.output_bit()) << out_bit)
    }

    fn read_word(&self, address: u32) -> Option<u16> {
        self.read_mapped(address).then(|| {
            let (_, _, out_bit) = self.input_bits();
            u16::from(self.output_bit()) << out_bit
        })
    }

    fn write_byte(&mut self, address: u32, value: u8) -> bool {
        if !self.write_mapped(address) {
            return false;
        }
        if self.is_acclaim32() {
            if address & 1 != 0 {
                self.sda = value & 1 != 0;
            } else {
                self.scl = value & 1 != 0;
            }
            self.update_protocol();
            self.old_scl = self.scl;
            self.old_sda = self.sda;
        } else if self.profile == GenesisEepromProfile::Acclaim16X24c02
            || self.is_jcart()
            || address & 1 != 0
        {
            let (scl_bit, sda_bit, _) = self.input_bits();
            self.update_lines(value & (1 << scl_bit) != 0, value & (1 << sda_bit) != 0);
        }
        true
    }

    fn write_word(&mut self, address: u32, value: u16) -> bool {
        if !self.write_mapped(address) {
            return false;
        }
        if self.is_acclaim32() {
            self.acclaim_rom_enabled = value & 1 != 0;
        } else {
            let (scl_bit, sda_bit, _) = self.input_bits();
            self.update_lines(value & (1 << scl_bit) != 0, value & (1 << sda_bit) != 0);
        }
        true
    }

    fn update_lines(&mut self, scl: bool, sda: bool) {
        self.scl = scl;
        self.sda = sda;
        self.update_protocol();
        self.old_scl = self.scl;
        self.old_sda = self.sda;
    }

    fn update_protocol(&mut self) {
        let start = self.old_scl && self.scl && self.old_sda && !self.sda;
        let stop = self.old_scl && self.scl && !self.old_sda && self.sda;
        if start {
            self.cycles = 0;
            self.read = false;
            self.buffer = 0;
            if self.address_bits() == 7 {
                self.word_address = 0;
                self.state = GenesisEepromState::Address;
            } else {
                self.device_address = 0;
                self.state = GenesisEepromState::DeviceAddress;
            }
            return;
        }
        if stop {
            self.state = GenesisEepromState::Standby;
            self.cycles = 0;
            return;
        }

        let falling = self.old_scl && !self.scl;
        let rising = !self.old_scl && self.scl;
        match self.state {
            GenesisEepromState::Standby | GenesisEepromState::WaitStop => {}
            GenesisEepromState::Address => {
                if falling {
                    if self.cycles < 9 {
                        self.cycles += 1;
                    } else {
                        self.cycles = 1;
                        self.state = if self.read {
                            GenesisEepromState::ReadData
                        } else {
                            GenesisEepromState::WriteData
                        };
                        self.buffer = 0;
                    }
                } else if rising {
                    if self.cycles < 8 {
                        self.word_address |= u16::from(self.sda) << (7 - self.cycles);
                    } else if self.cycles == 8 {
                        self.read = self.sda;
                    }
                }
            }
            GenesisEepromState::DeviceAddress => {
                if falling {
                    if self.cycles < 9 {
                        self.cycles += 1;
                    } else {
                        self.device_address = self
                            .device_address
                            .checked_shl(u32::from(self.address_bits()))
                            .unwrap_or(0);
                        self.cycles = 1;
                        if self.read {
                            self.state = GenesisEepromState::ReadData;
                        } else {
                            self.word_address = 0;
                            self.state = if self.address_bits() == 16 {
                                GenesisEepromState::WordAddressHigh
                            } else {
                                GenesisEepromState::WordAddressLow
                            };
                        }
                    }
                } else if rising {
                    if self.cycles > 4 && self.cycles < 8 {
                        self.device_address |= u16::from(self.sda) << (7 - self.cycles);
                    } else if self.cycles == 8 {
                        self.read = self.sda;
                    }
                }
            }
            GenesisEepromState::WordAddressHigh => {
                if falling {
                    if self.cycles < 9 {
                        self.cycles += 1;
                    } else {
                        self.cycles = 1;
                        self.state = GenesisEepromState::WordAddressLow;
                    }
                } else if rising && self.cycles < 9 {
                    let shift = 16 - self.cycles;
                    if u32::from(self.size_mask()) < (1u32 << shift) {
                        self.device_address >>= 1;
                    } else {
                        self.word_address |= u16::from(self.sda) << shift;
                    }
                }
            }
            GenesisEepromState::WordAddressLow => {
                if falling {
                    if self.cycles < 9 {
                        self.cycles += 1;
                    } else {
                        self.cycles = 1;
                        self.state = GenesisEepromState::WriteData;
                        self.buffer = 0;
                    }
                } else if rising && self.cycles < 9 {
                    let shift = 8 - self.cycles;
                    if u32::from(self.size_mask()) < (1u32 << shift) {
                        self.device_address >>= 1;
                    } else {
                        self.word_address |= u16::from(self.sda) << shift;
                    }
                }
            }
            GenesisEepromState::WriteData => {
                if falling {
                    if self.cycles < 9 {
                        self.cycles += 1;
                    } else {
                        self.cycles = 1;
                    }
                } else if rising {
                    if self.cycles < 9 {
                        self.buffer |= u8::from(self.sda) << (8 - self.cycles);
                    } else {
                        let address = (self.device_address | self.word_address) & self.size_mask();
                        self.data[usize::from(address)] = self.buffer;
                        self.buffer = 0;
                        let page_mask = self.page_mask();
                        self.word_address = (self.word_address & !page_mask)
                            | (self.word_address.wrapping_add(1) & page_mask);
                    }
                }
            }
            GenesisEepromState::ReadData => {
                if falling {
                    if self.cycles < 9 {
                        self.cycles += 1;
                    } else {
                        self.cycles = 1;
                    }
                } else if rising && self.cycles == 9 {
                    if self.sda {
                        self.state = GenesisEepromState::WaitStop;
                    } else {
                        self.word_address = self.word_address.wrapping_add(1) & self.size_mask();
                    }
                }
            }
        }
    }

    fn save(&self, out: &mut StateWriter) {
        let profile = match self.profile {
            GenesisEepromProfile::SegaX24c01 => 1,
            GenesisEepromProfile::EaX24c01 => 2,
            GenesisEepromProfile::Acclaim16X24c02 => 3,
            GenesisEepromProfile::Acclaim32C02 => 4,
            GenesisEepromProfile::Acclaim32C04 => 5,
            GenesisEepromProfile::Acclaim32C16 => 6,
            GenesisEepromProfile::Acclaim32C65 => 7,
            GenesisEepromProfile::JcartC08 => 8,
            GenesisEepromProfile::JcartC16 => 9,
            GenesisEepromProfile::JcartC65 => 10,
        };
        let state = match self.state {
            GenesisEepromState::Standby => 0,
            GenesisEepromState::WaitStop => 1,
            GenesisEepromState::Address => 2,
            GenesisEepromState::DeviceAddress => 3,
            GenesisEepromState::WordAddressHigh => 4,
            GenesisEepromState::WordAddressLow => 5,
            GenesisEepromState::WriteData => 6,
            GenesisEepromState::ReadData => 7,
        };
        out.u8(profile);
        out.blob(&self.data);
        out.u8(state);
        out.u8(self.cycles);
        out.u8(self.read as u8);
        out.u16(self.device_address);
        out.u16(self.word_address);
        out.u8(self.buffer);
        out.u8(self.scl as u8);
        out.u8(self.sda as u8);
        out.u8(self.old_scl as u8);
        out.u8(self.old_sda as u8);
        out.u8(self.acclaim_rom_enabled as u8);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let expected_profile = match self.profile {
            GenesisEepromProfile::SegaX24c01 => 1,
            GenesisEepromProfile::EaX24c01 => 2,
            GenesisEepromProfile::Acclaim16X24c02 => 3,
            GenesisEepromProfile::Acclaim32C02 => 4,
            GenesisEepromProfile::Acclaim32C04 => 5,
            GenesisEepromProfile::Acclaim32C16 => 6,
            GenesisEepromProfile::Acclaim32C65 => 7,
            GenesisEepromProfile::JcartC08 => 8,
            GenesisEepromProfile::JcartC16 => 9,
            GenesisEepromProfile::JcartC65 => 10,
        };
        if input.u8()? != expected_profile {
            return Err("Genesis EEPROM state profile mismatch".into());
        }
        let data = input.blob()?;
        if data.len() != self.data.len() {
            return Err("Genesis EEPROM state storage size mismatch".into());
        }
        self.data.copy_from_slice(data);
        self.state = match input.u8()? {
            0 => GenesisEepromState::Standby,
            1 => GenesisEepromState::WaitStop,
            2 => GenesisEepromState::Address,
            3 => GenesisEepromState::DeviceAddress,
            4 => GenesisEepromState::WordAddressHigh,
            5 => GenesisEepromState::WordAddressLow,
            6 => GenesisEepromState::WriteData,
            7 => GenesisEepromState::ReadData,
            _ => return Err("Genesis EEPROM state protocol mode is invalid".into()),
        };
        self.cycles = input.u8()?;
        if self.cycles > 9 {
            return Err("Genesis EEPROM state cycle counter is invalid".into());
        }
        self.read = input.u8()? != 0;
        self.device_address = input.u16()?;
        self.word_address = input.u16()?;
        self.buffer = input.u8()?;
        self.scl = input.u8()? != 0;
        self.sda = input.u8()? != 0;
        self.old_scl = input.u8()? != 0;
        self.old_sda = input.u8()? != 0;
        self.acclaim_rom_enabled = input.u8()? != 0;
        Ok(())
    }
}

pub(super) struct GenesisCartridge {
    rom: Vec<u8>,
    region: GenesisRegion,
    sram: Vec<u8>,
    sram_start: u32,
    sram_end: u32,
    sram_enabled: bool,
    sram_write_protected: bool,
    ssf2_mapper: bool,
    ssf2_banks: [u8; 8],
    jcart: bool,
    jcart_th: bool,
    eeprom: Option<GenesisEeprom>,
}

impl GenesisCartridge {
    fn decode_image(image: &[u8]) -> Result<Vec<u8>, String> {
        if image.len() > MAX_ROM + 512 {
            return Err("Genesis image exceeds the current 8 MiB cartridge limit".into());
        }
        if image.len() > 512
            && (image.len() - 512).is_multiple_of(0x4000)
            && image.len() % 0x4000 == 512
        {
            let input = &image[512..];
            let mut output = vec![0u8; input.len()];
            for (block_index, block) in input.as_chunks::<0x4000>().0.iter().enumerate() {
                let base = block_index * 0x4000;
                for i in 0..0x2000 {
                    output[base + i * 2] = block[0x2000 + i];
                    output[base + i * 2 + 1] = block[i];
                }
            }
            return Ok(output);
        }
        if image.len() < 8 {
            return Err("Genesis cartridge image is too small to contain reset vectors".into());
        }
        Ok(image.to_vec())
    }

    fn contains_marker(bytes: &[u8], marker: &[u8]) -> bool {
        bytes.windows(marker.len()).any(|window| window == marker)
    }

    fn detect_eeprom_profile(rom: &[u8], sram_len: usize) -> Option<GenesisEepromProfile> {
        let product = rom.get(0x180..0x18e).unwrap_or_default();
        let checksum = rom
            .get(0x18e..0x190)
            .map(|bytes| u16::from_be_bytes([bytes[0], bytes[1]]))
            .unwrap_or(0);
        let stack_pointer = u32::from_be_bytes(rom[0..4].try_into().unwrap());
        let ea_ids: &[&[u8]] = &[b"T-50176", b"T-50396", b"T-50446", b"T-50516", b"T-50606"];
        if ea_ids.iter().any(|id| Self::contains_marker(product, id)) {
            return Some(GenesisEepromProfile::EaX24c01);
        }

        let sega_ids: &[&[u8]] = &[
            b"T-12046",
            b"T-12053",
            b"MK-1215",
            b"MK-1228",
            b"G-5538",
            b"PR-1993",
            b"G-4060",
            b"00001211",
            b"00004076",
            b"G-4524",
            b"00054503",
        ];
        if let Some(id) = sega_ids
            .iter()
            .copied()
            .find(|id| Self::contains_marker(product, id))
        {
            let known_sram_patch = matches!(id, b"T-12046" | b"T-12053" | b"G-4060");
            if !known_sram_patch || sram_len <= 2 {
                return Some(GenesisEepromProfile::SegaX24c01);
            }
        }

        if [b"T-81033".as_slice(), b"T-081326".as_slice()]
            .iter()
            .any(|id| Self::contains_marker(product, id))
        {
            return Some(GenesisEepromProfile::Acclaim16X24c02);
        }
        if Self::contains_marker(product, b"T-081276") {
            return Some(GenesisEepromProfile::Acclaim32C02);
        }
        if Self::contains_marker(product, b"T-81406") {
            return Some(GenesisEepromProfile::Acclaim32C04);
        }
        if Self::contains_marker(product, b"T-081586") {
            return Some(GenesisEepromProfile::Acclaim32C16);
        }
        if [b"T-81476".as_slice(), b"T-81576".as_slice()]
            .iter()
            .any(|id| Self::contains_marker(product, id))
        {
            return Some(GenesisEepromProfile::Acclaim32C65);
        }

        if Self::contains_marker(product, b"T-120106") {
            return Some(GenesisEepromProfile::JcartC08);
        }
        if Self::contains_marker(product, b"T-120096") {
            return Some(GenesisEepromProfile::JcartC16);
        }
        if Self::contains_marker(product, b"T-120146") {
            return Some(GenesisEepromProfile::JcartC65);
        }
        if Self::contains_marker(product, b"00000000") && stack_pointer == 0x444e_4c44 {
            if checksum == 0x168b {
                return Some(GenesisEepromProfile::JcartC08);
            }
            if checksum == 0x165e {
                return Some(GenesisEepromProfile::JcartC16);
            }
        }

        if rom.get(0x1b0..0x1b2) == Some(b"RA") && (rom.get(0x1b2) == Some(&0xe8) || sram_len <= 2)
        {
            return Some(GenesisEepromProfile::SegaX24c01);
        }
        None
    }

    fn detect_region(rom: &[u8]) -> GenesisRegion {
        let field = rom.get(0x1f0..0x200).unwrap_or_default();
        let codes: Vec<u8> = field
            .iter()
            .copied()
            .filter(|byte| !matches!(*byte, 0 | b' '))
            .map(|byte| byte.to_ascii_uppercase())
            .collect();

        let supports_japan = codes.contains(&b'J');
        let supports_usa = codes.contains(&b'U');
        let supports_europe = codes.contains(&b'E');
        if supports_europe && !supports_japan && !supports_usa {
            return GenesisRegion::Pal;
        }

        if codes.len() == 1 {
            let value = match codes[0] {
                b'0'..=b'9' => Some(codes[0] - b'0'),
                b'A'..=b'F' => Some(codes[0] - b'A' + 10),
                _ => None,
            };
            if let Some(mask) = value {
                let supports_ntsc = mask & 0x05 != 0;
                let supports_pal = mask & 0x08 != 0;
                if supports_pal && !supports_ntsc {
                    return GenesisRegion::Pal;
                }
            }
        }

        GenesisRegion::Ntsc
    }

    fn detect_jcart(rom: &[u8]) -> bool {
        let product = rom.get(0x180..0x18e).unwrap_or_default();
        let checksum = rom
            .get(0x18e..0x190)
            .map(|bytes| u16::from_be_bytes([bytes[0], bytes[1]]))
            .unwrap_or(0);
        let stack_pointer = u32::from_be_bytes(rom[0..4].try_into().unwrap());
        if [b"T-120096", b"T-120066", b"T-123456"]
            .iter()
            .any(|id| Self::contains_marker(product, *id))
        {
            return true;
        }
        if Self::contains_marker(product, b"XXXXXXXX") && checksum == 0xdf39 {
            return true;
        }
        Self::contains_marker(product, b"00000000")
            && matches!(checksum, 0x168b | 0x165e)
            && matches!(stack_pointer, 0x444e_4c44 | 0xffff_fffc)
    }

    pub(super) fn new(image: &[u8]) -> Result<Self, String> {
        let rom = Self::decode_image(image)?;
        let console = rom.get(0x100..0x110).unwrap_or_default();
        let domestic = rom.get(0x120..0x150).unwrap_or_default();
        let ssf2_mapper = rom.len() > 0x400000
            && (Self::contains_marker(console, b"SEGA SSF")
                || Self::contains_marker(console, b"SEGA DOA")
                || Self::contains_marker(domestic, b"SUPER STREET FIGHTER2")
                || Self::contains_marker(domestic, b"SUPER STREET FIGHTER 2"));
        let region = Self::detect_region(&rom);
        let jcart = Self::detect_jcart(&rom);
        let mut sram_start = 0;
        let mut sram_end = 0;
        let mut sram = Vec::new();
        if rom.len() >= 0x1bc && &rom[0x1b0..0x1b2] == b"RA" {
            sram_start = u32::from_be_bytes(rom[0x1b4..0x1b8].try_into().unwrap()) & 0x00ff_ffff;
            sram_end = u32::from_be_bytes(rom[0x1b8..0x1bc].try_into().unwrap()) & 0x00ff_ffff;
            if sram_end >= sram_start {
                let len = usize::try_from(sram_end - sram_start + 1)
                    .unwrap_or(0)
                    .min(1024 * 1024);
                sram.resize(len, 0xff);
            }
        }
        let eeprom = Self::detect_eeprom_profile(&rom, sram.len()).map(GenesisEeprom::new);
        if eeprom.is_some() {
            sram.clear();
            sram_start = 0;
            sram_end = 0;
        }
        Ok(Self {
            rom,
            region,
            sram,
            sram_start,
            sram_end,
            sram_enabled: true,
            sram_write_protected: false,
            ssf2_mapper,
            ssf2_banks: [0, 1, 2, 3, 4, 5, 6, 7],
            jcart,
            jcart_th: true,
            eeprom,
        })
    }
    pub(super) fn read(&self, address: u32) -> u8 {
        if let Some(eeprom) = &self.eeprom {
            if let Some(value) = eeprom.read_byte(address) {
                return value;
            }
        }
        if self.sram_enabled
            && !self.sram.is_empty()
            && (self.sram_start..=self.sram_end).contains(&address)
        {
            let offset = usize::try_from(address - self.sram_start).unwrap_or(usize::MAX);
            return self.sram.get(offset).copied().unwrap_or(0xff);
        }
        if self.ssf2_mapper {
            if address < 0x400000 {
                let slot = (address >> 19) as usize;
                let bank = usize::from(self.ssf2_banks[slot]);
                let offset = address as usize & 0x7ffff;
                return self
                    .rom
                    .get(bank * 0x80000 + offset)
                    .copied()
                    .unwrap_or(0xff);
            }
            return 0xff;
        }
        self.rom.get(address as usize).copied().unwrap_or(0xff)
    }

    fn read_word(&self, address: u32) -> Option<u16> {
        let eeprom = self.eeprom.as_ref()?;
        if self.jcart && eeprom.is_jcart() && (0x380000..=0x3fffff).contains(&address) {
            return None;
        }
        eeprom.read_word(address)
    }

    fn write_word(&mut self, address: u32, value: u16) -> bool {
        self.eeprom
            .as_mut()
            .is_some_and(|eeprom| eeprom.write_word(address, value))
    }

    fn reset_mapper(&mut self) {
        self.ssf2_banks = [0, 1, 2, 3, 4, 5, 6, 7];
        self.jcart_th = true;
        if let Some(eeprom) = &mut self.eeprom {
            eeprom.reset_protocol();
        }
    }

    fn set_sram_control(&mut self, value: u8) {
        self.sram_enabled = value & 0x01 != 0;
        self.sram_write_protected = value & 0x02 != 0;
    }

    fn write_mapper(&mut self, address: u32, value: u8) {
        if !self.ssf2_mapper || address & 1 == 0 || !(0xa130f3..=0xa130ff).contains(&address) {
            return;
        }
        let slot = ((address & 0x0f) >> 1) as usize;
        if (1..8).contains(&slot) {
            self.ssf2_banks[slot] = value & 0x0f;
        }
    }

    pub(super) fn rom(&self) -> &[u8] {
        &self.rom
    }

    pub(super) fn write(&mut self, address: u32, value: u8) {
        if let Some(eeprom) = &mut self.eeprom {
            if eeprom.write_byte(address, value) {
                return;
            }
        }
        if self.sram_enabled
            && !self.sram_write_protected
            && !self.sram.is_empty()
            && (self.sram_start..=self.sram_end).contains(&address)
        {
            let offset = usize::try_from(address - self.sram_start).unwrap_or(usize::MAX);
            if let Some(slot) = self.sram.get_mut(offset) {
                *slot = value;
            }
        }
    }

    fn persistent_len(&self) -> usize {
        self.eeprom
            .as_ref()
            .map_or(self.sram.len(), |eeprom| eeprom.data.len())
    }
    fn persistent(&self) -> &[u8] {
        self.eeprom
            .as_ref()
            .map_or(self.sram.as_slice(), |eeprom| eeprom.data.as_slice())
    }
    fn set_persistent(&mut self, data: &[u8]) -> Result<(), String> {
        if let Some(eeprom) = &mut self.eeprom {
            if data.len() != eeprom.data.len() {
                return Err(format!(
                    "Genesis EEPROM expects {} bytes, got {}",
                    eeprom.data.len(),
                    data.len()
                ));
            }
            eeprom.data.copy_from_slice(data);
            return Ok(());
        }
        if data.len() != self.sram.len() {
            return Err(format!(
                "Genesis SRAM expects {} bytes, got {}",
                self.sram.len(),
                data.len()
            ));
        }
        self.sram.copy_from_slice(data);
        Ok(())
    }

    fn save(&self, out: &mut StateWriter) {
        out.blob(&self.sram);
        out.u8(self.sram_enabled as u8);
        out.u8(self.sram_write_protected as u8);
        out.blob(&self.ssf2_banks);
        out.u8(self.jcart_th as u8);
        out.u8(self.eeprom.is_some() as u8);
        if let Some(eeprom) = &self.eeprom {
            eeprom.save(out);
        }
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let sram = input.blob()?;
        if sram.len() != self.sram.len() {
            return Err("Genesis state SRAM size mismatch".into());
        }
        self.sram.copy_from_slice(sram);
        self.sram_enabled = input.u8()? != 0;
        self.sram_write_protected = input.u8()? != 0;
        let banks = input.blob()?;
        if banks.len() != self.ssf2_banks.len() {
            return Err("Genesis state SSF2 mapper bank count mismatch".into());
        }
        self.ssf2_banks.copy_from_slice(banks);
        self.ssf2_banks[0] = 0;
        for bank in &mut self.ssf2_banks[1..] {
            *bank &= 0x0f;
        }
        self.jcart_th = input.u8()? != 0;
        let state_has_eeprom = input.u8()? != 0;
        if state_has_eeprom != self.eeprom.is_some() {
            return Err("Genesis state EEPROM profile differs from cartridge".into());
        }
        if let Some(eeprom) = &mut self.eeprom {
            eeprom.load(input)?;
        }
        Ok(())
    }
}
#[derive(Clone)]
pub(super) struct GenesisIo {
    region: GenesisRegion,
    data: [u8; 3],
    control: [u8; 3],
    input: InputState,
    phase: [u8; 4],
}

impl Default for GenesisIo {
    fn default() -> Self {
        Self::new(GenesisRegion::Ntsc)
    }
}

impl GenesisIo {
    fn new(region: GenesisRegion) -> Self {
        Self {
            region,
            data: [0x7f; 3],
            control: [0; 3],
            input: InputState::default(),
            phase: [0; 4],
        }
    }

    pub(super) fn set_input(&mut self, input: &InputState) {
        self.input = input.clone();
        self.phase = [0; 4];
    }

    fn effective_th(&self, port: usize) -> bool {
        if self.control[port] & 0x40 != 0 {
            self.data[port] & 0x40 != 0
        } else {
            true
        }
    }

    fn update_phase(&mut self, port: usize, old_th: bool) {
        if port < 2 && old_th != self.effective_th(port) {
            self.phase[port] = self.phase[port].saturating_add(1).min(7);
        }
    }

    fn update_jcart_phase(&mut self, old_th: bool, new_th: bool) {
        if old_th != new_th {
            for phase in &mut self.phase[2..4] {
                *phase = phase.saturating_add(1).min(7);
            }
        }
    }

    fn write_data_port(&mut self, port: usize, value: u8) {
        let old_th = self.effective_th(port);
        self.data[port] = value;
        self.update_phase(port, old_th);
    }

    fn write_control_port(&mut self, port: usize, value: u8) {
        let old_th = self.effective_th(port);
        self.control[port] = value;
        self.update_phase(port, old_th);
    }

    fn controller_lines(&self, player: usize, th: bool) -> u8 {
        let buttons = self.input.buttons[player.min(3)];
        let phase = self.phase[player.min(3)];
        if !th && phase == 5 {
            let mut lines = 0x30;
            clear(&mut lines, 4, buttons & FACE_WEST != 0);
            clear(&mut lines, 5, buttons & START != 0);
            return lines;
        }
        if th && phase == 6 {
            let mut lines = 0x3f;
            clear(&mut lines, 0, buttons & R1 != 0);
            clear(&mut lines, 1, buttons & FACE_NORTH != 0);
            clear(&mut lines, 2, buttons & L1 != 0);
            clear(&mut lines, 3, buttons & SELECT != 0);
            clear(&mut lines, 4, buttons & FACE_SOUTH != 0);
            clear(&mut lines, 5, buttons & FACE_EAST != 0);
            return lines;
        }
        if !th && phase >= 7 {
            let mut lines = 0x3f;
            clear(&mut lines, 4, buttons & FACE_WEST != 0);
            clear(&mut lines, 5, buttons & START != 0);
            return lines;
        }

        let mut lines = 0x3f;
        clear(&mut lines, 0, buttons & UP != 0);
        clear(&mut lines, 1, buttons & DOWN != 0);
        if th {
            clear(&mut lines, 2, buttons & LEFT != 0);
            clear(&mut lines, 3, buttons & RIGHT != 0);
            clear(&mut lines, 4, buttons & FACE_SOUTH != 0);
            clear(&mut lines, 5, buttons & FACE_EAST != 0);
        } else {
            lines &= !0x0c;
            clear(&mut lines, 4, buttons & FACE_WEST != 0);
            clear(&mut lines, 5, buttons & START != 0);
        }
        lines
    }

    fn read_data(&self, port: usize) -> u8 {
        if port >= 2 {
            return self.data[2];
        }
        let th = self.effective_th(port);
        let inputs = self.controller_lines(port, th) | if th { 0x40 } else { 0 };
        (inputs & !self.control[port]) | (self.data[port] & self.control[port])
    }

    pub(super) fn read(&self, offset: u32) -> u8 {
        match offset & 0x1f {
            0x01 => self.region.version_byte(),
            0x03 => self.read_data(0),
            0x05 => self.read_data(1),
            0x07 => self.read_data(2),
            0x09 => self.control[0],
            0x0b => self.control[1],
            0x0d => self.control[2],
            _ => 0xff,
        }
    }

    pub(super) fn write(&mut self, offset: u32, value: u8) {
        match offset & 0x1f {
            0x03 => self.write_data_port(0, value),
            0x05 => self.write_data_port(1, value),
            0x07 => self.data[2] = value,
            0x09 => self.write_control_port(0, value),
            0x0b => self.write_control_port(1, value),
            0x0d => self.control[2] = value,
            _ => {}
        }
    }

    fn save(&self, out: &mut StateWriter) {
        out.blob(&self.data);
        out.blob(&self.control);
        out.blob(&self.phase);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let data = input.blob()?;
        let control = input.blob()?;
        let phase = input.blob()?;
        if data.len() != 3 || control.len() != 3 || phase.len() != 4 {
            return Err("invalid Genesis I/O state".into());
        }
        self.data.copy_from_slice(data);
        self.control.copy_from_slice(control);
        self.phase.copy_from_slice(phase);
        for value in &mut self.phase {
            *value = (*value).min(7);
        }
        Ok(())
    }
}
fn clear(value: &mut u8, bit: u8, pressed: bool) {
    if pressed {
        *value &= !(1 << bit);
    }
}

fn jcart_read_word(io: &GenesisIo, cartridge: &GenesisCartridge) -> u16 {
    let th = cartridge.jcart_th;
    let eeprom_out = cartridge
        .eeprom
        .as_ref()
        .filter(|eeprom| eeprom.is_jcart())
        .is_none_or(GenesisEeprom::output_bit);
    let low = io.controller_lines(2, th) | (u8::from(th) << 6) | (u8::from(eeprom_out) << 7);
    let high = io.controller_lines(3, th) & 0x3f;
    (u16::from(high) << 8) | u16::from(low)
}

fn jcart_read_byte(io: &GenesisIo, cartridge: &GenesisCartridge, address: u32) -> u8 {
    let word = jcart_read_word(io, cartridge);
    if address & 1 == 0 {
        (word >> 8) as u8
    } else {
        word as u8
    }
}

fn jcart_write(io: &mut GenesisIo, cartridge: &mut GenesisCartridge, value: u16) {
    let old_th = cartridge.jcart_th;
    let new_th = value & 1 != 0;
    io.update_jcart_phase(old_th, new_th);
    cartridge.jcart_th = new_th;
}

pub(super) struct GenesisBus {
    pub(super) cartridge: GenesisCartridge,
    pub(super) main_ram: Box<[u8; 0x10000]>,
    pub(super) z80_ram: Box<[u8; 0x2000]>,
    pub(super) io: GenesisIo,
    pub(super) vdp: GenesisVdp,
    pub(super) ym: Ym2612,
    pub(super) psg: Sn76489,
    pub(super) z80_bus_requested: bool,
    pub(super) z80_running: bool,
    pub(super) z80_bank: u32,
}

impl GenesisBus {
    pub(super) fn new(cartridge: GenesisCartridge) -> Self {
        let region = cartridge.region;
        Self {
            cartridge,
            main_ram: Box::new([0; 0x10000]),
            z80_ram: Box::new([0; 0x2000]),
            io: GenesisIo::new(region),
            vdp: GenesisVdp::new(region == GenesisRegion::Pal),
            ym: Ym2612::new(region.m68k_hz()),
            psg: Sn76489::new(region.z80_hz()),
            z80_bus_requested: false,
            z80_running: false,
            z80_bank: 0,
        }
    }

    pub(super) fn reset_devices(&mut self) {
        self.vdp.reset();
        self.ym.reset();
        self.psg.reset();
        self.io = GenesisIo::new(self.cartridge.region);
        self.cartridge.reset_mapper();
        self.z80_bus_requested = false;
        self.z80_running = false;
        self.z80_bank = 0;
    }
    fn read_vdp_word(&mut self, address: u32) -> u16 {
        match address & 0x1c {
            0x00 => self.vdp.read_data(),
            0x04 => self.vdp.read_status(),
            0x08 => self.vdp.read_hv_counter(),
            _ => 0xffff,
        }
    }

    fn write_vdp_word(&mut self, address: u32, value: u16) {
        match address & 0x1c {
            0x00 => self.vdp.write_data(value),
            0x04 => self.vdp.write_control(value),
            0x10 => self.psg.write(value as u8),
            _ => {}
        }
    }

    fn dma_read_word(&self, address: u32) -> u16 {
        let address = address & 0x00ff_fffe;
        if let Some(value) = self.cartridge.read_word(address) {
            return value;
        }
        match address {
            0x380000..=0x3ffffe if self.cartridge.jcart => {
                jcart_read_word(&self.io, &self.cartridge)
            }
            0x000000..=0x7ffffe => u16::from_be_bytes([
                self.cartridge.read(address),
                self.cartridge.read(address.wrapping_add(1)),
            ]),
            0xff0000..=0xfffffe => u16::from_be_bytes([
                self.main_ram[address as usize & 0xffff],
                self.main_ram[address.wrapping_add(1) as usize & 0xffff],
            ]),
            _ => 0xffff,
        }
    }

    pub(super) fn service_vdp_dma(&mut self) -> Option<u32> {
        if !self.vdp.memory_dma_pending() {
            return None;
        }
        let source = self.vdp.memory_dma_source()?;
        let word = self.dma_read_word(source);
        self.vdp.service_memory_dma_word(word)
    }

    fn read_main8(&mut self, address: u32) -> u8 {
        let address = address & 0x00ff_ffff;
        match address {
            0x380000..=0x3fffff if self.cartridge.jcart => {
                jcart_read_byte(&self.io, &self.cartridge, address)
            }
            0x000000..=0x7fffff => self.cartridge.read(address),
            0xa00000..=0xa01fff if self.z80_bus_requested => {
                self.z80_ram[(address as usize) & 0x1fff]
            }
            0xa04000..=0xa04003 if self.z80_bus_requested => self.ym.read_status(),
            0xa10000..=0xa1001f => self.io.read(address - 0xa10000),
            0xa11100..=0xa11101 => {
                if self.z80_bus_requested {
                    0
                } else {
                    1
                }
            }
            0xa11200..=0xa11201 => u8::from(self.z80_running),
            0xc00000..=0xc0001f => {
                let word = self.read_vdp_word(address & !1);
                if address & 1 == 0 {
                    (word >> 8) as u8
                } else {
                    word as u8
                }
            }
            0xff0000..=0xffffff => self.main_ram[(address as usize) & 0xffff],
            _ => 0xff,
        }
    }
    fn write_main8(&mut self, address: u32, value: u8) {
        let address = address & 0x00ff_ffff;
        match address {
            0x380000..=0x3fffff if self.cartridge.jcart => {
                jcart_write(&mut self.io, &mut self.cartridge, u16::from(value));
            }
            0x000000..=0x7fffff => self.cartridge.write(address, value),
            0xa00000..=0xa01fff if self.z80_bus_requested => {
                self.z80_ram[(address as usize) & 0x1fff] = value;
            }
            0xa04000..=0xa04003 if self.z80_bus_requested => {
                self.ym.write_port((address & 3) as u8, value);
            }
            0xa06000..=0xa060ff if self.z80_bus_requested => {
                self.z80_bank = ((self.z80_bank >> 1) | (u32::from(value & 1) << 8)) & 0x01ff;
            }
            0xa10000..=0xa1001f => self.io.write(address - 0xa10000, value),
            0xa11100..=0xa11101 => self.z80_bus_requested = value & 1 != 0,
            0xa11200..=0xa11201 => self.z80_running = value & 1 != 0,
            0xa130f0..=0xa130f1 => self.cartridge.set_sram_control(value),
            0xa130f2..=0xa130ff => self.cartridge.write_mapper(address, value),
            0xa14000..=0xa14003 => {}
            0xc00000..=0xc0001f => {
                let word = u16::from(value) * 0x0101;
                self.write_vdp_word(address & !1, word);
            }
            0xff0000..=0xffffff => self.main_ram[(address as usize) & 0xffff] = value,
            _ => {}
        }
    }

    pub(super) fn tick_main_cycles(&mut self, cycles: u32) {
        self.vdp.tick_cpu_cycles(cycles);
        self.ym.tick(cycles);
    }
}

impl Bus68000 for GenesisBus {
    fn read8(&mut self, address: u32) -> u8 {
        self.read_main8(address)
    }
    fn write8(&mut self, address: u32, value: u8) {
        self.write_main8(address, value);
    }
    fn tas_write8(&mut self, _address: u32, _value: u8) {}
    fn read16(&mut self, address: u32) -> u16 {
        let address = address & 0x00ff_ffff;
        if let Some(value) = self.cartridge.read_word(address) {
            return value;
        }
        match address {
            0x380000..=0x3fffff if self.cartridge.jcart => {
                jcart_read_word(&self.io, &self.cartridge)
            }
            0xa11100 => {
                if self.z80_bus_requested {
                    0x0000
                } else {
                    0x0100
                }
            }
            0xa11200 => {
                if self.z80_running {
                    0x0100
                } else {
                    0x0000
                }
            }
            0xc00000..=0xc0001f => self.read_vdp_word(address),
            _ => u16::from_be_bytes([
                self.read_main8(address),
                self.read_main8(address.wrapping_add(1)),
            ]),
        }
    }

    fn write16(&mut self, address: u32, value: u16) {
        let address = address & 0x00ff_ffff;
        if self.cartridge.write_word(address, value) {
            return;
        }
        match address {
            0x380000..=0x3fffff if self.cartridge.jcart => {
                jcart_write(&mut self.io, &mut self.cartridge, value);
            }
            0xa11100 => self.z80_bus_requested = value & 0x0100 != 0,
            0xa11200 => self.z80_running = value & 0x0100 != 0,
            0xa130f0 => self.cartridge.set_sram_control(value as u8),
            0xc00000..=0xc0001f => self.write_vdp_word(address, value),
            _ => {
                let [high, low] = value.to_be_bytes();
                self.write_main8(address, high);
                self.write_main8(address.wrapping_add(1), low);
            }
        }
    }

    fn read32(&mut self, address: u32) -> u32 {
        (u32::from(self.read16(address)) << 16) | u32::from(self.read16(address.wrapping_add(2)))
    }

    fn write32(&mut self, address: u32, value: u32) {
        self.write16(address, (value >> 16) as u16);
        self.write16(address.wrapping_add(2), value as u16);
    }
}
struct GenesisZ80Bridge<'a> {
    ram: &'a mut [u8; 0x2000],
    bank: &'a mut u32,
    cartridge: &'a mut GenesisCartridge,
    io: &'a mut GenesisIo,
    main_ram: &'a mut [u8; 0x10000],
    ym: &'a mut Ym2612,
    psg: &'a mut Sn76489,
}

impl GenesisZ80Bridge<'_> {
    fn banked_address(&self, address: u16) -> u32 {
        ((*self.bank & 0x01ff) << 15) | u32::from(address & 0x7fff)
    }

    fn banked_read(&mut self, address: u16) -> u8 {
        let target = self.banked_address(address) & 0x00ff_ffff;
        match target {
            0x380000..=0x3fffff if self.cartridge.jcart => {
                jcart_read_byte(self.io, self.cartridge, target)
            }
            0x000000..=0x7fffff => self.cartridge.read(target),
            0xff0000..=0xffffff => self.main_ram[(target as usize) & 0xffff],
            _ => 0xff,
        }
    }

    fn banked_write(&mut self, address: u16, value: u8) {
        let target = self.banked_address(address) & 0x00ff_ffff;
        match target {
            0x380000..=0x3fffff if self.cartridge.jcart => {
                jcart_write(self.io, self.cartridge, u16::from(value));
            }
            0x000000..=0x7fffff => self.cartridge.write(target, value),
            0xff0000..=0xffffff => self.main_ram[(target as usize) & 0xffff] = value,
            _ => {}
        }
    }
}

impl Z80Bus for GenesisZ80Bridge<'_> {
    fn mem_read(&mut self, address: u16) -> u8 {
        match address {
            0x0000..=0x3fff => self.ram[usize::from(address) & 0x1fff],
            0x4000..=0x4003 => self.ym.read_status(),
            0x8000..=0xffff => self.banked_read(address),
            _ => 0xff,
        }
    }
    fn mem_write(&mut self, address: u16, value: u8) {
        match address {
            0x0000..=0x3fff => self.ram[usize::from(address) & 0x1fff] = value,
            0x4000..=0x4003 => self.ym.write_port((address & 3) as u8, value),
            0x6000..=0x60ff => {
                *self.bank = ((*self.bank >> 1) | (u32::from(value & 1) << 8)) & 0x01ff;
            }
            0x7f00..=0x7fff if address & 0x1f == 0x11 => self.psg.write(value),
            0x8000..=0xffff => self.banked_write(address, value),
            _ => {}
        }
    }
}

pub struct GenesisMachine {
    pub(super) main_cpu: M68000,
    pub(super) z80: Z80,
    pub(super) bus: GenesisBus,
    pub(super) audio: AudioBuffer,
    pub(super) z80_phase: u64,
    pub(super) z80_cycle_credit: i64,
    pub(super) powered: bool,
}

impl GenesisMachine {
    pub fn from_rom(image: &[u8]) -> Result<Self, String> {
        let cartridge = GenesisCartridge::new(image)?;
        let mut bus = GenesisBus::new(cartridge);
        let mut main_cpu = M68000::default();
        main_cpu.reset(&mut bus);
        let mut z80 = Z80::default();
        z80.reset();
        Ok(Self {
            main_cpu,
            z80,
            bus,
            audio: AudioBuffer::new(48_000, 2),
            z80_phase: 0,
            z80_cycle_credit: 0,
            powered: true,
        })
    }
    pub(super) fn run_z80_for_main_cycles(&mut self, main_cycles: u32) {
        let region = self.bus.cartridge.region;
        self.z80_phase = self
            .z80_phase
            .saturating_add(u64::from(main_cycles) * region.z80_hz());
        let z80_cycles = self.z80_phase / region.m68k_hz();
        self.z80_phase %= region.m68k_hz();
        self.z80_cycle_credit = self.z80_cycle_credit.saturating_add(z80_cycles as i64);
        self.bus.psg.tick_cpu_cycles(z80_cycles as u32);
        if !self.bus.z80_running {
            self.z80.reset();
            self.z80_cycle_credit = 0;
            return;
        }
        if self.bus.z80_bus_requested {
            return;
        }
        while self.z80_cycle_credit > 0 {
            let mut bridge = GenesisZ80Bridge {
                ram: &mut self.bus.z80_ram,
                bank: &mut self.bus.z80_bank,
                cartridge: &mut self.bus.cartridge,
                io: &mut self.bus.io,
                main_ram: &mut self.bus.main_ram,
                ym: &mut self.bus.ym,
                psg: &mut self.bus.psg,
            };
            let used = self.z80.step(&mut bridge);
            if used == 0 {
                break;
            }
            self.z80_cycle_credit -= i64::from(used);
            if self.z80_cycle_credit < -32 {
                self.z80_cycle_credit = -32;
            }
        }
    }

    fn clock_main_instruction(&mut self) -> u32 {
        if let Some(used) = self.bus.service_vdp_dma() {
            self.main_cpu.cycles = self.main_cpu.cycles.wrapping_add(u64::from(used));
            self.bus.tick_main_cycles(used);
            self.run_z80_for_main_cycles(used);
            return used;
        }

        let used = self.main_cpu.step(&mut self.bus);
        if used == 0 {
            return 0;
        }
        self.bus.tick_main_cycles(used);
        self.run_z80_for_main_cycles(used);
        let level = self.bus.vdp.irq_level();
        if level != 0 {
            let interrupt_cycles = self.main_cpu.interrupt(&mut self.bus, level, 24 + level);
            if interrupt_cycles != 0 {
                self.bus.vdp.acknowledge_irq(level);
                self.bus.tick_main_cycles(interrupt_cycles);
                self.run_z80_for_main_cycles(interrupt_cycles);
            }
        }
        used
    }
    pub(super) fn flush_audio(&mut self) {
        self.audio.begin_frame();
        let fm = self.bus.ym.samples();
        let psg = self.bus.psg.samples();
        let len = fm.len().max(psg.len());
        for index in 0..len {
            let (fm_l, fm_r) = fm.get(index).copied().unwrap_or((0.0, 0.0));
            let tone = psg.get(index).copied().unwrap_or(0.0) * 0.35;
            self.audio.push_stereo(
                (fm_l + tone).clamp(-1.0, 1.0),
                (fm_r + tone).clamp(-1.0, 1.0),
            );
        }
    }

    pub(super) fn begin_audio_frame(&mut self) {
        self.bus.ym.begin_frame();
        self.bus.psg.begin_frame();
        self.audio.begin_frame();
    }

    pub(super) fn save_bus(&self, out: &mut StateWriter) {
        out.blob(self.bus.main_ram.as_slice());
        out.blob(self.bus.z80_ram.as_slice());
        self.bus.cartridge.save(out);
        self.bus.io.save(out);
        self.bus.vdp.save(out);
        self.bus.ym.save(out);
        self.bus.psg.save(out);
        out.u8(self.bus.z80_bus_requested as u8);
        out.u8(self.bus.z80_running as u8);
        out.u32(self.bus.z80_bank);
    }

    pub(super) fn load_bus(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let ram = input.blob()?;
        if ram.len() != self.bus.main_ram.len() {
            return Err("Genesis main RAM state size mismatch".into());
        }
        self.bus.main_ram.copy_from_slice(ram);
        let zram = input.blob()?;
        if zram.len() != self.bus.z80_ram.len() {
            return Err("Genesis Z80 RAM state size mismatch".into());
        }
        self.bus.z80_ram.copy_from_slice(zram);
        self.bus.cartridge.load(input)?;
        self.bus.io.load(input)?;
        self.bus.vdp.load(input)?;
        self.bus.ym.load(input)?;
        self.bus.psg.load(input)?;
        self.bus.z80_bus_requested = input.u8()? != 0;
        self.bus.z80_running = input.u8()? != 0;
        self.bus.z80_bank = input.u32()? & 0x01ff;
        Ok(())
    }
}
impl Machine for GenesisMachine {
    fn platform(&self) -> PlatformId {
        PlatformId::Genesis
    }

    fn reset(&mut self) {
        self.bus.reset_devices();
        self.main_cpu.reset(&mut self.bus);
        self.z80.reset();
        self.z80_phase = 0;
        self.z80_cycle_credit = 0;
        self.powered = true;
        self.begin_audio_frame();
    }

    fn run_frame(&mut self, input: &InputState) {
        self.bus.io.set_input(input);
        self.begin_audio_frame();
        let target = self.bus.vdp.frame().wrapping_add(1);
        let region = self.bus.cartridge.region;
        let deadline = self
            .main_cpu
            .cycles
            .saturating_add((region.m68k_hz() as f64 / region.frame_rate() * 2.0).ceil() as u64);
        while self.bus.vdp.frame() != target && self.main_cpu.cycles < deadline {
            if self.clock_main_instruction() == 0 {
                self.powered = false;
                break;
            }
        }
        if self.bus.vdp.frame() != target {
            self.powered = false;
        }
        self.flush_audio();
    }

    fn frame_rate(&self) -> f64 {
        self.bus.cartridge.region.frame_rate()
    }
    fn video(&self) -> &VideoBuffer {
        self.bus.vdp.video()
    }
    fn audio(&self) -> &AudioBuffer {
        &self.audio
    }

    fn save_state(&self) -> Result<Vec<u8>, String> {
        let mut out = StateWriter::new(PlatformId::Genesis, STATE_VERSION);
        self.main_cpu.save(&mut out);
        self.z80.save(&mut out);
        self.save_bus(&mut out);
        out.u64(self.z80_phase);
        out.u64(self.z80_cycle_credit as u64);
        out.u8(self.powered as u8);
        Ok(out.finish())
    }
    fn load_state(&mut self, bytes: &[u8]) -> Result<(), String> {
        let mut input = StateReader::new(bytes, PlatformId::Genesis, STATE_VERSION)?;
        self.main_cpu.load(&mut input)?;
        self.z80.load(&mut input)?;
        self.load_bus(&mut input)?;
        self.z80_phase = input.u64()? % self.bus.cartridge.region.m68k_hz();
        self.z80_cycle_credit = input.u64()? as i64;
        self.powered = input.u8()? != 0;
        self.begin_audio_frame();
        input.finish()
    }

    fn persistent_len(&self, kind: ResourceKind, slot: u32) -> usize {
        if kind == ResourceKind::Storage && slot == 0 {
            self.bus.cartridge.persistent_len()
        } else {
            0
        }
    }

    fn read_persistent(&self, kind: ResourceKind, slot: u32, out: &mut [u8]) -> Result<(), String> {
        if kind != ResourceKind::Storage || slot != 0 {
            return Err("Genesis persistent resource is Storage slot 0".into());
        }
        if out.len() != self.bus.cartridge.persistent_len() {
            return Err("Genesis persistent output length mismatch".into());
        }
        out.copy_from_slice(self.bus.cartridge.persistent());
        Ok(())
    }

    fn write_persistent(
        &mut self,
        kind: ResourceKind,
        slot: u32,
        data: &[u8],
    ) -> Result<(), String> {
        if kind != ResourceKind::Storage || slot != 0 {
            return Err("Genesis persistent resource is Storage slot 0".into());
        }
        self.bus.cartridge.set_persistent(data)
    }
}
#[cfg(test)]
mod tests {
    use super::*;

    fn write_word(rom: &mut [u8], address: usize, word: u16) {
        rom[address..address + 2].copy_from_slice(&word.to_be_bytes());
    }

    fn write_long(rom: &mut [u8], address: usize, value: u32) {
        rom[address..address + 4].copy_from_slice(&value.to_be_bytes());
    }

    fn synthetic_eeprom_rom(product: &[u8]) -> Vec<u8> {
        let mut rom = synthetic_rom(false);
        rom[0x180..0x18e].fill(b' ');
        let len = product.len().min(0x0e);
        rom[0x180..0x180 + len].copy_from_slice(&product[..len]);
        rom
    }

    fn eeprom_lines(cart: &mut GenesisCartridge, scl: bool, sda: bool) {
        let profile = cart.eeprom.as_ref().unwrap().profile;
        if matches!(
            profile,
            GenesisEepromProfile::Acclaim32C02
                | GenesisEepromProfile::Acclaim32C04
                | GenesisEepromProfile::Acclaim32C16
                | GenesisEepromProfile::Acclaim32C65
        ) {
            cart.write(0x200000, u8::from(scl));
            cart.write(0x200001, u8::from(sda));
        } else {
            let eeprom = cart.eeprom.as_ref().unwrap();
            let (scl_bit, sda_bit, _) = eeprom.input_bits();
            let address = if eeprom.is_jcart() {
                0x300000
            } else {
                0x200000
            };
            let value = (u16::from(scl) << scl_bit) | (u16::from(sda) << sda_bit);
            assert!(cart.write_word(address, value));
        }
    }

    fn eeprom_start(cart: &mut GenesisCartridge) {
        eeprom_lines(cart, true, true);
        eeprom_lines(cart, true, false);
        eeprom_lines(cart, false, false);
    }

    fn eeprom_stop(cart: &mut GenesisCartridge) {
        eeprom_lines(cart, false, false);
        eeprom_lines(cart, true, false);
        eeprom_lines(cart, true, true);
    }

    fn eeprom_write_byte(cart: &mut GenesisCartridge, value: u8) {
        for bit in (0..8).rev() {
            let high = value & (1 << bit) != 0;
            eeprom_lines(cart, false, high);
            eeprom_lines(cart, true, high);
            eeprom_lines(cart, false, high);
        }
        eeprom_lines(cart, false, true);
        eeprom_lines(cart, true, true);
        assert!(!cart.eeprom.as_ref().unwrap().output_bit());
        eeprom_lines(cart, false, true);
    }

    fn eeprom_read_byte(cart: &mut GenesisCartridge) -> u8 {
        let mut value = 0u8;
        for _ in 0..8 {
            eeprom_lines(cart, false, true);
            eeprom_lines(cart, true, true);
            value = (value << 1) | u8::from(cart.eeprom.as_ref().unwrap().output_bit());
            eeprom_lines(cart, false, true);
        }
        eeprom_lines(cart, false, true);
        eeprom_lines(cart, true, true);
        eeprom_lines(cart, false, true);
        value
    }

    fn synthetic_ssf2_rom() -> Vec<u8> {
        let mut rom = vec![0; 0x500000];
        for bank in 0..10usize {
            rom[bank * 0x80000..(bank + 1) * 0x80000].fill(bank as u8);
        }
        let title = b"SUPER STREET FIGHTER2";
        rom[0x120..0x120 + title.len()].copy_from_slice(title);
        rom
    }

    fn synthetic_jcart_rom() -> Vec<u8> {
        let mut rom = vec![0xff; 0x300000];
        write_long(&mut rom, 0, 0x00ff_ff00);
        write_long(&mut rom, 4, 0x0000_0200);
        rom[0x180..0x188].copy_from_slice(b"T-120066");
        rom
    }

    fn synthetic_rom(with_sram: bool) -> Vec<u8> {
        let mut rom = vec![0xff; 0x40000];
        write_long(&mut rom, 0, 0x00ff_ff00);
        write_long(&mut rom, 4, 0x0000_0200);
        if with_sram {
            rom[0x1b0..0x1b2].copy_from_slice(b"RA");
            write_long(&mut rom, 0x1b4, 0x0020_0000);
            write_long(&mut rom, 0x1b8, 0x0020_00ff);
        }
        let words = [
            0x13fc, 0x0084, 0x00c0, 0x0011, 0x13fc, 0x0010, 0x00c0, 0x0011, 0x13fc, 0x0090, 0x00c0,
            0x0011, 0x60fe,
        ];
        for (index, word) in words.into_iter().enumerate() {
            write_word(&mut rom, 0x200 + index * 2, word);
        }
        rom
    }
    fn synthetic_region_rom(code: &[u8]) -> Vec<u8> {
        let mut rom = synthetic_rom(false);
        rom[0x1f0..0x200].fill(b' ');
        let len = code.len().min(16);
        rom[0x1f0..0x1f0 + len].copy_from_slice(&code[..len]);
        rom
    }

    #[test]
    fn cartridge_region_detection_prefers_ntsc_for_multiregion_roms() {
        assert_eq!(
            GenesisCartridge::new(&synthetic_region_rom(b"E"))
                .unwrap()
                .region,
            GenesisRegion::Pal
        );
        assert_eq!(
            GenesisCartridge::new(&synthetic_region_rom(b"8"))
                .unwrap()
                .region,
            GenesisRegion::Pal
        );
        assert_eq!(
            GenesisCartridge::new(&synthetic_region_rom(b"JUE"))
                .unwrap()
                .region,
            GenesisRegion::Ntsc
        );
        assert_eq!(
            GenesisCartridge::new(&synthetic_region_rom(b"C"))
                .unwrap()
                .region,
            GenesisRegion::Ntsc
        );
    }

    #[test]
    fn pal_region_updates_version_register_frame_rate_and_state_restore() {
        let rom = synthetic_region_rom(b"E");
        let mut machine = GenesisMachine::from_rom(&rom).unwrap();
        assert_eq!(machine.bus.cartridge.region, GenesisRegion::Pal);
        assert_eq!(machine.bus.io.read(0x01), 0xe0);
        assert_ne!(machine.bus.vdp.read_status() & 1, 0);
        assert!(machine.frame_rate() > 49.0 && machine.frame_rate() < 51.0);

        machine.run_frame(&InputState::default());
        assert_eq!(machine.bus.vdp.frame(), 1);
        let state = machine.save_state().unwrap();
        let phase = machine.z80_phase;
        machine.run_frame(&InputState::default());
        machine.load_state(&state).unwrap();
        assert_eq!(machine.z80_phase, phase);
        assert_eq!(machine.bus.io.read(0x01), 0xe0);
        assert_ne!(machine.bus.vdp.read_status() & 1, 0);

        let ntsc = GenesisMachine::from_rom(&synthetic_region_rom(b"U")).unwrap();
        assert_eq!(ntsc.bus.io.read(0x01), 0xa0);
        assert!(ntsc.frame_rate() > machine.frame_rate());
    }

    #[test]
    fn synthetic_machine_runs_68000_vdp_psg_and_state() {
        let mut machine = GenesisMachine::from_rom(&synthetic_rom(false)).unwrap();
        machine.run_frame(&InputState::default());
        assert!(machine.powered);
        assert_eq!(machine.video().width(), 320);
        assert_eq!(machine.video().height(), 224);
        assert_eq!(machine.bus.vdp.frame(), 1);
        assert!(machine
            .audio()
            .samples()
            .iter()
            .any(|sample| sample.abs() > 0.001));
        let saved = machine.save_state().unwrap();
        let pc = machine.main_cpu.pc;
        machine.run_frame(&InputState::default());
        machine.load_state(&saved).unwrap();
        assert_eq!(machine.main_cpu.pc, pc);
        assert_eq!(machine.bus.vdp.frame(), 1);
    }

    #[test]
    fn vdp_dma_reads_68k_work_ram_through_the_machine_bus() {
        let mut machine = GenesisMachine::from_rom(&synthetic_rom(false)).unwrap();
        machine.bus.main_ram[..4].copy_from_slice(&[0x11, 0x22, 0x33, 0x44]);
        let control = 0x00c0_0004;
        for (register, value) in [
            (1u16, 0x10u16),
            (15, 0x02),
            (19, 0x02),
            (20, 0x00),
            (21, 0x00),
            (22, 0x80),
            (23, 0x7f),
        ] {
            machine
                .bus
                .write16(control, 0x8000 | (register << 8) | value);
        }
        machine.bus.write16(control, 0x4100);
        machine.bus.write16(control, 0x0080);
        assert!(machine.bus.vdp.memory_dma_pending());
        assert_ne!(machine.bus.read16(control) & 0x0002, 0);

        let pc = machine.main_cpu.pc;
        assert_ne!(machine.clock_main_instruction(), 0);
        assert_eq!(machine.main_cpu.pc, pc);
        assert!(machine.bus.vdp.memory_dma_pending());
        assert_ne!(machine.clock_main_instruction(), 0);
        assert_eq!(machine.main_cpu.pc, pc);
        assert!(!machine.bus.vdp.memory_dma_pending());

        machine.bus.write16(control, 0x0100);
        machine.bus.write16(control, 0x0000);
        assert_eq!(machine.bus.read16(0x00c0_0000), 0x1122);
        assert_eq!(machine.bus.read16(0x00c0_0000), 0x3344);
        assert_eq!(machine.bus.read16(control) & 0x0002, 0);
    }

    #[test]
    fn internal_vdp_dma_keeps_the_68000_running() {
        let mut machine = GenesisMachine::from_rom(&synthetic_rom(false)).unwrap();
        let control = 0x00c0_0004;
        for (register, value) in [
            (1u16, 0x50u16),
            (15, 0x01),
            (19, 0x40),
            (20, 0x00),
            (21, 0x10),
            (22, 0x00),
            (23, 0xc0),
        ] {
            machine
                .bus
                .write16(control, 0x8000 | (register << 8) | value);
        }
        machine.bus.write16(control, 0x0020);
        machine.bus.write16(control, 0x00c0);
        assert_ne!(machine.bus.read16(control) & 0x0002, 0);

        let pc = machine.main_cpu.pc;
        assert_ne!(machine.clock_main_instruction(), 0);
        assert_ne!(machine.main_cpu.pc, pc);
        assert_ne!(machine.bus.read16(control) & 0x0002, 0);
    }

    #[test]
    fn cartridge_sram_uses_generic_persistence_contract() {
        let mut machine = GenesisMachine::from_rom(&synthetic_rom(true)).unwrap();
        assert_eq!(machine.persistent_len(ResourceKind::Storage, 0), 0x100);
        let input = vec![0x5a; 0x100];
        machine
            .write_persistent(ResourceKind::Storage, 0, &input)
            .unwrap();
        let mut output = vec![0; 0x100];
        machine
            .read_persistent(ResourceKind::Storage, 0, &mut output)
            .unwrap();
        assert_eq!(input, output);
        machine.bus.cartridge.write(0x0020_0010, 0xa5);
        assert_eq!(machine.bus.cartridge.read(0x0020_0010), 0xa5);

        machine.bus.write16(0x00a1_30f0, 0x0003);
        assert!(machine.bus.cartridge.sram_write_protected);
        machine.bus.cartridge.write(0x0020_0010, 0x3c);
        assert_eq!(machine.bus.cartridge.read(0x0020_0010), 0xa5);

        machine.bus.write16(0x00a1_30f0, 0x0001);
        assert!(!machine.bus.cartridge.sram_write_protected);
        machine.bus.cartridge.write(0x0020_0010, 0x3c);
        assert_eq!(machine.bus.cartridge.read(0x0020_0010), 0x3c);

        machine.bus.write16(0x00a1_30f0, 0x0000);
        assert_eq!(machine.bus.cartridge.read(0x0020_0010), 0xff);
        machine.bus.write16(0x00a1_30f0, 0x0001);
        assert_eq!(machine.bus.cartridge.read(0x0020_0010), 0x3c);
    }
    #[test]
    fn sega_x24c01_bitbang_page_wrap_persistence_and_state_are_live() {
        let rom = synthetic_eeprom_rom(b"MK-1215");
        let mut cart = GenesisCartridge::new(&rom).unwrap();
        assert!(matches!(
            cart.eeprom.as_ref().map(|eeprom| eeprom.profile),
            Some(GenesisEepromProfile::SegaX24c01)
        ));
        assert!(cart.sram.is_empty());
        assert_eq!(cart.persistent_len(), 128);
        assert_eq!(cart.read_word(0x200000), Some(1));
        assert_eq!(cart.read(0x200000), 0xff);
        assert_eq!(cart.read(0x200001), 1);

        eeprom_start(&mut cart);
        eeprom_write_byte(&mut cart, 0x13 << 1);
        eeprom_write_byte(&mut cart, 0x5a);
        eeprom_write_byte(&mut cart, 0xa5);
        eeprom_stop(&mut cart);
        assert_eq!(cart.eeprom.as_ref().unwrap().data[0x13], 0x5a);
        assert_eq!(cart.eeprom.as_ref().unwrap().data[0x10], 0xa5);

        eeprom_start(&mut cart);
        eeprom_write_byte(&mut cart, (0x13 << 1) | 1);
        assert_eq!(eeprom_read_byte(&mut cart), 0x5a);
        eeprom_stop(&mut cart);

        let persistent = cart.persistent().to_vec();
        let mut persisted = GenesisCartridge::new(&rom).unwrap();
        persisted.set_persistent(&persistent).unwrap();
        assert_eq!(persisted.eeprom.as_ref().unwrap().data[0x10], 0xa5);

        eeprom_start(&mut cart);
        let command = 0x22u8 << 1;
        for bit in (4..8).rev() {
            let high = command & (1 << bit) != 0;
            eeprom_lines(&mut cart, false, high);
            eeprom_lines(&mut cart, true, high);
            eeprom_lines(&mut cart, false, high);
        }
        let mut writer = StateWriter::new(PlatformId::Genesis, 99);
        cart.save(&mut writer);
        let state = writer.finish();
        let mut restored = GenesisCartridge::new(&rom).unwrap();
        let mut reader = StateReader::new(&state, PlatformId::Genesis, 99).unwrap();
        restored.load(&mut reader).unwrap();
        reader.finish().unwrap();
        for bit in (0..4).rev() {
            let high = command & (1 << bit) != 0;
            eeprom_lines(&mut restored, false, high);
            eeprom_lines(&mut restored, true, high);
            eeprom_lines(&mut restored, false, high);
        }
        eeprom_lines(&mut restored, false, true);
        eeprom_lines(&mut restored, true, true);
        assert!(!restored.eeprom.as_ref().unwrap().output_bit());
        eeprom_lines(&mut restored, false, true);
        eeprom_write_byte(&mut restored, 0x3c);
        eeprom_stop(&mut restored);
        assert_eq!(restored.eeprom.as_ref().unwrap().data[0x22], 0x3c);
    }

    #[test]
    fn ea_x24c01_uses_d7_data_and_d6_clock() {
        let rom = synthetic_eeprom_rom(b"T-50396");
        let mut cart = GenesisCartridge::new(&rom).unwrap();
        assert!(matches!(
            cart.eeprom.as_ref().map(|eeprom| eeprom.profile),
            Some(GenesisEepromProfile::EaX24c01)
        ));
        assert_eq!(cart.read_word(0x200000), Some(0x80));
        assert_eq!(cart.read(0x200001), 0x80);

        eeprom_start(&mut cart);
        eeprom_write_byte(&mut cart, 0x2a << 1);
        eeprom_write_byte(&mut cart, 0xc3);
        eeprom_stop(&mut cart);
        eeprom_start(&mut cart);
        eeprom_write_byte(&mut cart, (0x2a << 1) | 1);
        assert_eq!(eeprom_read_byte(&mut cart), 0xc3);
        eeprom_stop(&mut cart);
    }

    #[test]
    fn acclaim_x24c02_uses_device_and_word_address_cycles() {
        let rom = synthetic_eeprom_rom(b"T-081326");
        let mut cart = GenesisCartridge::new(&rom).unwrap();
        assert!(matches!(
            cart.eeprom.as_ref().map(|eeprom| eeprom.profile),
            Some(GenesisEepromProfile::Acclaim16X24c02)
        ));
        assert_eq!(cart.persistent_len(), 256);

        eeprom_start(&mut cart);
        eeprom_write_byte(&mut cart, 0xa0);
        eeprom_write_byte(&mut cart, 0x42);
        eeprom_write_byte(&mut cart, 0x5a);
        eeprom_stop(&mut cart);
        assert_eq!(cart.eeprom.as_ref().unwrap().data[0x42], 0x5a);

        eeprom_start(&mut cart);
        eeprom_write_byte(&mut cart, 0xa0);
        eeprom_write_byte(&mut cart, 0x42);
        eeprom_start(&mut cart);
        eeprom_write_byte(&mut cart, 0xa1);
        assert_eq!(eeprom_read_byte(&mut cart), 0x5a);
        eeprom_stop(&mut cart);
    }

    #[test]
    fn acclaim_c65_uses_16bit_address_and_64byte_page_wrap() {
        let rom = synthetic_eeprom_rom(b"T-81476");
        let mut cart = GenesisCartridge::new(&rom).unwrap();
        assert!(matches!(
            cart.eeprom.as_ref().map(|eeprom| eeprom.profile),
            Some(GenesisEepromProfile::Acclaim32C65)
        ));
        assert_eq!(cart.persistent_len(), 8192);
        assert_eq!(cart.read_word(0x200000), None);
        assert!(cart.write_word(0x200000, 0));
        assert_eq!(cart.read_word(0x200000), Some(1));

        eeprom_start(&mut cart);
        eeprom_write_byte(&mut cart, 0xa0);
        eeprom_write_byte(&mut cart, 0x12);
        eeprom_write_byte(&mut cart, 0x3f);
        eeprom_write_byte(&mut cart, 0x5a);
        eeprom_write_byte(&mut cart, 0xa5);
        eeprom_stop(&mut cart);
        assert_eq!(cart.eeprom.as_ref().unwrap().data[0x123f], 0x5a);
        assert_eq!(cart.eeprom.as_ref().unwrap().data[0x1200], 0xa5);

        assert!(cart.write_word(0x200000, 1));
        assert_eq!(cart.read_word(0x200000), None);
    }

    #[test]
    fn acclaim_product_ids_select_expected_eeprom_geometry() {
        for (product, profile, size) in [
            (
                b"T-081276".as_slice(),
                GenesisEepromProfile::Acclaim32C02,
                256usize,
            ),
            (
                b"T-81406".as_slice(),
                GenesisEepromProfile::Acclaim32C04,
                512,
            ),
            (
                b"T-081586".as_slice(),
                GenesisEepromProfile::Acclaim32C16,
                2048,
            ),
            (
                b"T-81576".as_slice(),
                GenesisEepromProfile::Acclaim32C65,
                8192,
            ),
        ] {
            let cart = GenesisCartridge::new(&synthetic_eeprom_rom(product)).unwrap();
            assert!(cart
                .eeprom
                .as_ref()
                .is_some_and(|eeprom| { eeprom.profile == profile && eeprom.data.len() == size }));
        }
    }

    #[test]
    fn jcart_eeprom_combines_d7_with_extra_controller_bus() {
        let rom = synthetic_eeprom_rom(b"T-120096");
        let mut cart = GenesisCartridge::new(&rom).unwrap();
        assert!(cart.jcart);
        assert!(matches!(
            cart.eeprom.as_ref().map(|eeprom| eeprom.profile),
            Some(GenesisEepromProfile::JcartC16)
        ));
        assert_eq!(cart.persistent_len(), 2048);
        assert_eq!(cart.read_word(0x380000), None);

        eeprom_start(&mut cart);
        eeprom_write_byte(&mut cart, 0xa0);
        eeprom_write_byte(&mut cart, 0x00);
        eeprom_write_byte(&mut cart, 0x00);
        eeprom_stop(&mut cart);
        eeprom_start(&mut cart);
        eeprom_write_byte(&mut cart, 0xa0);
        eeprom_write_byte(&mut cart, 0x00);
        eeprom_start(&mut cart);
        eeprom_write_byte(&mut cart, 0xa1);

        let io = GenesisIo::default();
        assert_eq!(jcart_read_word(&io, &cart) & 0x0080, 0);
        cart.eeprom.as_mut().unwrap().data[0] = 0x80;
        assert_eq!(jcart_read_word(&io, &cart) & 0x0080, 0x0080);
        eeprom_stop(&mut cart);

        let mut writer = StateWriter::new(PlatformId::Genesis, 99);
        cart.save(&mut writer);
        let state = writer.finish();
        let mut restored = GenesisCartridge::new(&rom).unwrap();
        let mut reader = StateReader::new(&state, PlatformId::Genesis, 99).unwrap();
        restored.load(&mut reader).unwrap();
        reader.finish().unwrap();
        assert_eq!(restored.eeprom.as_ref().unwrap().data[0], 0x80);
    }

    #[test]
    fn jcart_eeprom_only_products_do_not_enable_extra_controllers() {
        for (product, profile, size) in [
            (
                b"T-120106".as_slice(),
                GenesisEepromProfile::JcartC08,
                1024usize,
            ),
            (
                b"T-120146".as_slice(),
                GenesisEepromProfile::JcartC65,
                8192usize,
            ),
        ] {
            let mut cart = GenesisCartridge::new(&synthetic_eeprom_rom(product)).unwrap();
            assert!(!cart.jcart);
            assert!(cart
                .eeprom
                .as_ref()
                .is_some_and(|eeprom| { eeprom.profile == profile && eeprom.data.len() == size }));
            assert_eq!(cart.read_word(0x380000), Some(0x80));

            eeprom_start(&mut cart);
            eeprom_write_byte(&mut cart, 0xa0);
            if profile == GenesisEepromProfile::JcartC65 {
                eeprom_write_byte(&mut cart, 0x00);
            }
            eeprom_write_byte(&mut cart, 0x42);
            eeprom_write_byte(&mut cart, 0x5a);
            eeprom_stop(&mut cart);
            assert_eq!(cart.eeprom.as_ref().unwrap().data[0x42], 0x5a);
        }
    }

    #[test]
    fn sram_patched_wily_wars_is_not_reclassified_as_eeprom() {
        let mut rom = synthetic_eeprom_rom(b"T-12046");
        rom[0x1b0..0x1b2].copy_from_slice(b"RA");
        write_long(&mut rom, 0x1b4, 0x0020_0000);
        write_long(&mut rom, 0x1b8, 0x0020_00ff);
        let cart = GenesisCartridge::new(&rom).unwrap();
        assert!(cart.eeprom.is_none());
        assert_eq!(cart.sram.len(), 0x100);
    }

    #[test]
    fn ssf2_mapper_switches_512k_slots_for_cpu_dma_and_state() {
        let rom = synthetic_ssf2_rom();
        let cartridge = GenesisCartridge::new(&rom).unwrap();
        assert!(cartridge.ssf2_mapper);
        let mut bus = GenesisBus::new(cartridge);
        assert_eq!(bus.read16(0x080000), 0x0101);
        assert_eq!(bus.read16(0x300000), 0x0606);
        assert_eq!(bus.read16(0x380000), 0x0707);

        bus.write16(0xa130fc, 0x0008);
        bus.write16(0xa130fe, 0x0009);
        assert_eq!(bus.read16(0x300000), 0x0808);
        assert_eq!(bus.read16(0x380000), 0x0909);
        assert_eq!(bus.dma_read_word(0x300000), 0x0808);
        assert_eq!(bus.read16(0x000000), 0x0000);

        let mut writer = StateWriter::new(PlatformId::Genesis, 99);
        bus.cartridge.save(&mut writer);
        let bytes = writer.finish();
        let mut restored = GenesisCartridge::new(&rom).unwrap();
        let mut reader = StateReader::new(&bytes, PlatformId::Genesis, 99).unwrap();
        restored.load(&mut reader).unwrap();
        reader.finish().unwrap();
        assert_eq!(restored.read(0x300000), 8);
        assert_eq!(restored.read(0x380000), 9);

        bus.reset_devices();
        assert_eq!(bus.read16(0x300000), 0x0606);
        assert_eq!(bus.read16(0x380000), 0x0707);
    }

    #[test]
    fn jcart_exposes_two_extra_controllers_and_preserves_handshake_state() {
        let rom = synthetic_jcart_rom();
        let cartridge = GenesisCartridge::new(&rom).unwrap();
        assert!(cartridge.jcart);
        let mut bus = GenesisBus::new(cartridge);
        let mut input = InputState::default();
        input.buttons[2] = UP | FACE_SOUTH | R1 | FACE_NORTH | L1 | SELECT;
        input.buttons[3] = DOWN | FACE_WEST | START;
        bus.io.set_input(&input);

        assert_eq!(bus.read16(0x0038_0000), 0x3dee);
        assert_eq!(bus.read8(0x0038_0000), 0x3d);
        assert_eq!(bus.read8(0x0038_0001), 0xee);
        assert_eq!(bus.read16(0x0038_0000) & 0x4000, 0);

        bus.write16(0x0038_0000, 0x0000);
        assert!(!bus.cartridge.jcart_th);
        assert_eq!(&bus.io.phase[2..4], &[1, 1]);
        assert_eq!(bus.read16(0x0038_0000), 0x01b2);

        for value in [1u16, 0, 1, 0, 1] {
            bus.write16(0x0038_0000, value);
        }
        assert!(bus.cartridge.jcart_th);
        assert_eq!(&bus.io.phase[2..4], &[6, 6]);
        assert_eq!(bus.read16(0x0038_0000) & 0x000f, 0);

        let mut writer = StateWriter::new(PlatformId::Genesis, 99);
        bus.cartridge.save(&mut writer);
        bus.io.save(&mut writer);
        let state = writer.finish();
        bus.write16(0x0038_0000, 0x0000);

        let mut restored_cartridge = GenesisCartridge::new(&rom).unwrap();
        let mut restored_io = GenesisIo {
            input,
            ..Default::default()
        };
        let mut reader = StateReader::new(&state, PlatformId::Genesis, 99).unwrap();
        restored_cartridge.load(&mut reader).unwrap();
        restored_io.load(&mut reader).unwrap();
        reader.finish().unwrap();
        assert!(restored_cartridge.jcart_th);
        assert_eq!(&restored_io.phase[2..4], &[6, 6]);
        assert_eq!(
            jcart_read_word(&restored_io, &restored_cartridge) & 0x000f,
            0
        );
    }

    #[test]
    fn controller_th_line_selects_three_button_groups() {
        let mut io = GenesisIo::default();
        let mut input = InputState::default();
        input.buttons[0] = UP | LEFT | FACE_SOUTH | FACE_WEST | START;
        io.set_input(&input);
        io.control[0] = 0x40;
        io.data[0] = 0x40;
        let high = io.read_data(0);
        assert_eq!(high & 0x01, 0);
        assert_eq!(high & 0x04, 0);
        assert_eq!(high & 0x10, 0);
        io.data[0] = 0x00;
        let low = io.read_data(0);
        assert_eq!(low & 0x10, 0);
        assert_eq!(low & 0x20, 0);
    }

    #[test]
    fn six_button_handshake_exposes_extra_buttons_and_identification_phase() {
        let mut io = GenesisIo::default();
        let mut input = InputState::default();
        input.buttons[0] =
            FACE_EAST | FACE_SOUTH | FACE_WEST | START | L1 | FACE_NORTH | R1 | SELECT;
        io.set_input(&input);
        io.write(0x09, 0x40);

        for value in [0x00, 0x40, 0x00, 0x40] {
            io.write(0x03, value);
        }
        io.write(0x03, 0x00);
        let identify = io.read_data(0);
        assert_eq!(identify & 0x0f, 0x00);
        assert_eq!(identify & 0x30, 0x00);

        io.write(0x03, 0x40);
        let extra = io.read_data(0);
        assert_eq!(extra & 0x3f, 0x00);
        assert_eq!(io.phase[0], 6);

        let mut out = StateWriter::new(PlatformId::Genesis, 99);
        io.save(&mut out);
        let bytes = out.finish();
        let mut restored = GenesisIo {
            input: input.clone(),
            ..Default::default()
        };
        let mut reader = StateReader::new(&bytes, PlatformId::Genesis, 99).unwrap();
        restored.load(&mut reader).unwrap();
        reader.finish().unwrap();
        assert_eq!(restored.phase[0], 6);
        assert_eq!(restored.read_data(0) & 0x3f, 0x00);

        io.write(0x03, 0x00);
        let final_low = io.read_data(0);
        assert_eq!(final_low & 0x0f, 0x0f);
        assert_eq!(final_low & 0x30, 0x00);

        io.set_input(&input);
        assert_eq!(io.phase[0], 0);
        assert_eq!(io.read_data(0) & 0x0f, 0x03);
    }

    #[test]
    fn controller_th_defaults_high_when_the_port_pin_is_an_input() {
        let mut io = GenesisIo::default();
        let mut input = InputState::default();
        input.buttons[0] = LEFT | FACE_SOUTH;
        io.set_input(&input);
        io.data[0] = 0x00;
        io.control[0] = 0x00;
        let value = io.read_data(0);
        assert_eq!(value & 0x04, 0x00);
        assert_eq!(value & 0x10, 0x00);
        assert_ne!(value & 0x40, 0x00);
    }
    #[test]
    fn genesis_main_bus_suppresses_tas_memory_writeback() {
        let mut rom = synthetic_rom(false);
        write_word(&mut rom, 0x200, 0x4ad0); // TAS (A0)
        write_word(&mut rom, 0x202, 0x60fe);
        let cartridge = GenesisCartridge::new(&rom).unwrap();
        let mut bus = GenesisBus::new(cartridge);
        bus.main_ram[0x10] = 0x01;
        let mut cpu = M68000::default();
        cpu.reset(&mut bus);
        cpu.a[0] = 0xff0010;

        assert_eq!(cpu.step(&mut bus), 10);
        assert_eq!(bus.main_ram[0x10], 0x01);
    }
    #[test]
    fn smd_interleaving_is_decoded_into_linear_bytes() {
        let mut raw = vec![0u8; 512 + 0x4000];
        for i in 0..0x2000usize {
            raw[512 + i] = (i & 0xff) as u8;
            raw[512 + 0x2000 + i] = (0x80 | (i & 0x7f)) as u8;
        }
        let decoded = GenesisCartridge::decode_image(&raw).unwrap();
        assert_eq!(decoded.len(), 0x4000);
        assert_eq!(decoded[0], 0x80);
        assert_eq!(decoded[1], 0x00);
        assert_eq!(decoded[2], 0x81);
        assert_eq!(decoded[3], 0x01);
    }
}
